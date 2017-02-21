/*
 * 		FILE: 		FAC_Full_DCDC_20kHz.c
 * 		PROJECT: 	DRS v2.0
 * 		CREATION:
 * 		MODIFIED:	28/04/2016
 *
 * 		AUTHOR: 	Gabriel O. B.  (LNLS/ELP)
 *
 * 		DESCRIPTION: Firmware for control of two DC/DC stage for
 * 					 a 1k1A FAC v2.0 prototype
 *
 *
 *		TODO:
 */

#include "F28M36x_ELP_DRS.h"
#include "FAC_Full_DCDC_20kHz.h"

// Prototype statements for functions found within this file.

#pragma CODE_SECTION(isr_ePWM_CTR_ZERO, "ramfuncs");
#pragma CODE_SECTION(isr_ePWM_CTR_ZERO_1st, "ramfuncs");
#pragma CODE_SECTION(PS_turnOn, "ramfuncs");
#pragma CODE_SECTION(PS_turnOff, "ramfuncs");
#pragma CODE_SECTION(Set_SoftInterlock, "ramfuncs");
#pragma CODE_SECTION(Set_HardInterlock, "ramfuncs");
#pragma CODE_SECTION(isr_SoftInterlock, "ramfuncs");
#pragma CODE_SECTION(isr_HardInterlock, "ramfuncs");

static interrupt void isr_ePWM_CTR_ZERO(void);
static interrupt void isr_ePWM_CTR_ZERO_1st(void);

void main_FAC_Full_DCDC_20kHz(void);

static void PS_turnOn(void);
static void PS_turnOff(void);

static void Set_SoftInterlock(Uint32 itlk);
static void Set_HardInterlock(Uint32 itlk);
static interrupt void isr_SoftInterlock(void);
static interrupt void isr_HardInterlock(void);

static void InitPeripheralsDrivers(void);
static void InitControllers(void);
static void ResetControllers(void);
static void InitInterruptions(void);

volatile static Uint32 valorCounter;

void main_FAC_Full_DCDC_20kHz(void)
{
	InitPeripheralsDrivers();

	InitControllers();

	InitInterruptions();

	ConfigCpuTimer(&CpuTimer0, C28_FREQ_MHZ, 10000000);
	CpuTimer1Regs.TCR.all = 0xC000;

	stop_DMA();
	DELAY_US(5);
	start_DMA();
	EnablePWM_TBCLK();

	while(1)
	{
		if(PIN_STATUS_ACDC_INTERLOCK)
		{
			if(CHECK_INTERLOCK(ACDC_FAULT))
			{
				Set_HardInterlock(ACDC_FAULT);
			}
		}

		if(CHECK_INTERLOCKS)
		{
			PIN_CLEAR_DCDC_INTERLOCK;
		}

		TunningPWM_MEP_SFO();
	}

}


/*
 * Initialization of peripheral drivers:
 *
 * 		- HRADC boards
 * 		- PWM modules
 * 		- GPIO's
 * 		- Timers
 */

static void InitPeripheralsDrivers(void)
{
	stop_DMA();

	/* Initialization of HRADC boards */

	Init_DMA_McBSP_nBuffers(3, DECIMATION_FACTOR, HRADC_SPI_CLK);

	Init_SPIMaster_McBSP(HRADC_SPI_CLK);
	Init_SPIMaster_Gpio();
	InitMcbspa20bit();

	HRADCs_Info.HRADC_boards[0] = &HRADC0_board;
	Init_HRADC_Info(HRADCs_Info.HRADC_boards[0], 0, DECIMATION_FACTOR, buffers_HRADC.buffer_0, TRANSDUCER_0_GAIN, HRADC_0_R_BURDEN);
	Config_HRADC_board(HRADCs_Info.HRADC_boards[0], TRANSDUCER_0_OUTPUT_TYPE, HEATER_DISABLE, RAILS_DISABLE);

	HRADCs_Info.HRADC_boards[1] = &HRADC1_board;
	Init_HRADC_Info(HRADCs_Info.HRADC_boards[1], 1, DECIMATION_FACTOR, buffers_HRADC.buffer_1, TRANSDUCER_1_GAIN, HRADC_1_R_BURDEN);
	Config_HRADC_board(HRADCs_Info.HRADC_boards[1], TRANSDUCER_1_OUTPUT_TYPE, HEATER_DISABLE, RAILS_DISABLE);

	HRADCs_Info.HRADC_boards[2] = &HRADC2_board;
	Init_HRADC_Info(HRADCs_Info.HRADC_boards[2], 2, DECIMATION_FACTOR, buffers_HRADC.buffer_2, TRANSDUCER_2_GAIN, HRADC_2_R_BURDEN);
	Config_HRADC_board(HRADCs_Info.HRADC_boards[2], TRANSDUCER_2_OUTPUT_TYPE, HEATER_DISABLE, RAILS_DISABLE);

	AverageFilter = 1.0/((float) DECIMATION_FACTOR);

	Config_HRADC_SoC(HRADC_FREQ_SAMP);

	/* Initialization of PWM modules */

    EALLOW;
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO29 = 1;			// Auxiliar GPIO for which GPTRIP1 is selected
	GpioDataRegs.GPASET.bit.GPIO29 = 1;
	EDIS;

	PWM_Modules.N_modules = 4;
	PWM_Modules.PWM_Regs[0] = &EPwm1Regs;
	PWM_Modules.PWM_Regs[1] = &EPwm2Regs;
	PWM_Modules.PWM_Regs[2] = &EPwm7Regs;
	PWM_Modules.PWM_Regs[3] = &EPwm8Regs;

	DisablePWMOutputs();
	DisablePWM_TBCLK();
	InitPWM_MEP_SFO();

	InitPWMModule(PWM_Modules.PWM_Regs[0], PWM_FREQ, 0, MasterPWM, 0, COMPLEMENTARY, PWM_DEAD_TIME);
	InitPWMModule(PWM_Modules.PWM_Regs[1], PWM_FREQ, 1, SlavePWM, 180, COMPLEMENTARY, PWM_DEAD_TIME);
	InitPWMModule(PWM_Modules.PWM_Regs[2], PWM_FREQ, 0, SlavePWM, 90, COMPLEMENTARY, PWM_DEAD_TIME);
	InitPWMModule(PWM_Modules.PWM_Regs[3], PWM_FREQ, 7, SlavePWM, 270, COMPLEMENTARY, PWM_DEAD_TIME);

	InitEPwm1Gpio();
	InitEPwm2Gpio();
	InitEPwm7Gpio();
	InitEPwm8Gpio();

	/* Initialization of GPIOs */

	EALLOW;

	GpioCtrlRegs.GPDMUX2.bit.GPIO126 = 0;
	GpioDataRegs.GPDCLEAR.bit.GPIO126 = 1;		// GPDI1: AC/DC Interlock
	GpioCtrlRegs.GPDDIR.bit.GPIO126 = 0;

	GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 0;
	GpioDataRegs.GPCSET.bit.GPIO67 = 1;			// GPDO1: DC/DC Interlock
	GpioCtrlRegs.GPCDIR.bit.GPIO67 = 1;

	INIT_DEBUG_GPIO1;

	EDIS;

	/* Initialization of timers */
	InitCpuTimers();
	CpuTimer0Regs.TCR.bit.TIE = 0;
	CpuTimer1Regs.TCR.bit.TIE = 0;
	CpuTimer2Regs.TCR.bit.TIE = 0;
}

static void InitControllers(void)
{
	/* Initialization of IPC module */
	InitIPC(&PS_turnOn, &PS_turnOff, &isr_SoftInterlock, &isr_HardInterlock);

	/* Initiaization of DP Framework */
	InitDP_Framework(&DP_Framework, &(IPC_CtoM_Msg.PSModule.IRef));

	/***********************************************/
	/* INITIALIZATION OF LOAD CURRENT CONTROL LOOP */
	/***********************************************/

	/*
	 * 	      name: 	SRLIM_ILOAD_REFERENCE
	 * description: 	Load current reference slew-rate limiter
	 *    DP class:     ELP_SRLim
	 *     	    in:		DP_Framework.Ref
	 * 		   out:		NetSignals[0]
	 */

	Init_ELP_SRLim(SRLIM_ILOAD_REFERENCE, MAX_REF_SLEWRATE, CONTROL_FREQ, DP_Framework.Ref, &DP_Framework.NetSignals[0]);

	/*
	 * 	      name: 	ERROR_CALCULATOR
	 * description: 	Load current reference error
	 *    DP class:     ELP_Error
	 *     	    +:		NetSignals[0]
	 *     	    -:		NetSignals[1]
	 * 		   out:		NetSignals[8]
	 */

	Init_ELP_Error(ERROR_CALCULATOR, &DP_Framework.NetSignals[0], &DP_Framework.NetSignals[1], &DP_Framework.NetSignals[8]);

	/*
	 * 	      name: 	PI_DAWU_CONTROLLER_ILOAD
	 * description: 	Load current PI controller
	 *    DP class:     ELP_PI_dawu
	 *     	    in:		NetSignals[8]
	 * 		   out:		NetSignals[9]
	 */

	Init_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD, KP_ILOAD, KI_ILOAD, CONTROL_FREQ, PWM_MAX_DUTY, PWM_MIN_DUTY, &DP_Framework.NetSignals[8], &DP_Framework.NetSignals[9]);

	/*
	 * 	      name: 	IIR_2P2Z_FF_REFERENCE
	 * description: 	Reference load current feed-forward
	 *    DP class:     ELP_IIR_2P2Z;
	 *     	    in:		NetSignals[0]
	 * 		   out:		NetSignals[13]
	 */

	// Fctrl = 20480 Hz
	Init_ELP_IIR_2P2Z(IIR_2P2Z_FF_REFERENCE, 0.03163094210223, -0.031626812567677, 0.0,
					  -0.969783893510680, 0.0, PWM_MAX_DUTY, PWM_MIN_DUTY, &DP_Framework.NetSignals[0], &DP_Framework.NetSignals[13]);

	// Fctrl = 40960 Hz
	//Init_ELP_IIR_2P2Z(IIR_2P2Z_FF_REFERENCE, 0.031870661544369, -0.031868581061066, 0.0,
	//					-0.984776951441255, 0.0, FLT_MAX, -FLT_MAX, &DP_Framework.NetSignals[0], &DP_Framework.NetSignals[13]);


	/************************************************/
	/* INITIALIZATION OF CURRENT SHARE CONTROL LOOP */
	/************************************************/

	/*
	 * 	      name: 	ISHARE_ERROR_CALCULATOR
	 * description: 	Current share error
	 *    DP class:     ELP_Error
	 *     	    +:		MtoC.NetSignals[0]
	 *     	    -:		NetSignals[12]
	 * 		   out:		NetSignals[10]
	 */

	Init_ELP_Error(ISHARE_ERROR_CALCULATOR, &DP_Framework_MtoC.NetSignals[0], &DP_Framework.NetSignals[12], &DP_Framework.NetSignals[10]);

	/*
	 * 	      name: 	PI_DAWU_CONTROLLER_ISHARE
	 * description: 	Current share PI controller
	 *    DP class:     ELP_PI_dawu
	 *     	    in:		NetSignals[10]
	 * 		   out:		NetSignals[11]
	 */

	Init_ELP_PI_dawu(PI_DAWU_CONTROLLER_ISHARE, KP_ISHARE, KI_ISHARE, (CONTROL_FREQ/ISHARE_DECIMATION), PWM_MAX_SHARE_DUTY, -PWM_MAX_SHARE_DUTY, &DP_Framework.NetSignals[10], &DP_Framework.NetSignals[11]);

	/*****************************************/
	/* INITIALIZATION OF DC LINK FEEDFORWARD */
	/*****************************************/

	/*
	 * 	      name: 	IIR_2P2Z_LPF_VDCLINK_MOD1
	 * description: 	Module 1 DC-Link voltage low-pass filter
	 *    DP class:     ELP_IIR_2P2Z
	 *     	    in:		NetSignals[17]
	 * 		   out:		NetSignals[18]
	 */

	Init_ELP_IIR_2P2Z(IIR_2P2Z_LPF_VDCLINK_MOD1, 0.126216944768300, 0.252433889536601, 0.126216944768300,
					  -0.774934273867545, 0.279802052940746, FLT_MAX, -FLT_MAX, &DP_Framework.NetSignals[17], &DP_Framework.NetSignals[18]);

	/*
	 * 	      name: 	IIR_2P2Z_LPF_VDCLINK_MOD2
	 * description: 	Module 2 DC-Link voltage low-pass filter
	 *    DP class:     ELP_IIR_2P2Z
	 *     	    in:		NetSignals[19]
	 * 		   out:		NetSignals[20]
	 */

	Init_ELP_IIR_2P2Z(IIR_2P2Z_LPF_VDCLINK_MOD2, 0.126216944768300, 0.252433889536601, 0.126216944768300,
			          -0.774934273867545, 0.279802052940746, FLT_MAX, -FLT_MAX, &DP_Framework.NetSignals[19], &DP_Framework.NetSignals[20]);

	/*
	 * 	      name: 	FF_VDCLINK_MOD1
	 * description: 	Module 1 DC-Link voltage feed-forward law
	 *    DP class:     ELP_DCLink_FF
	 *    vdc_meas:		NetSignals[18]
	 *     	    in:		NetSignals[15]
	 * 		   out:		DutySignals[0]
	 */

	Init_ELP_DCLink_FF(FF_VDCLINK_MOD1, NOM_VDCLINK, MIN_DCLINK, &DP_Framework.NetSignals[18], &DP_Framework.NetSignals[15], &DP_Framework.DutySignals[0]);


	/*
	 * 	      name: 	FF_VDCLINK_MOD2
	 * description: 	Module 2 DC-Link voltage feed-forward law
	 *    DP class:     ELP_DCLink_FF
	 *    vdc_meas:		NetSignals[20]
	 *     	    in:		NetSignals[16]
	 * 		   out:		DutySignals[1]
	 */

	Init_ELP_DCLink_FF(FF_VDCLINK_MOD2, NOM_VDCLINK, MIN_DCLINK, &DP_Framework.NetSignals[20], &DP_Framework.NetSignals[16], &DP_Framework.DutySignals[1]);

	/*********************************************/
	/* INITIALIZATION OF SIGNAL GENERATOR MODULE */
	/*********************************************/

	/*
	 * 	      name: 	SignalGenerator
	 * description: 	Signal generator module
	 * 		   out:		DP_Framework.Ref
	 */

	Disable_ELP_SigGen(&SignalGenerator);
	Init_ELP_SigGen(&SignalGenerator, Sine, 0.0, 0.0, 0.0, CONTROL_FREQ, &IPC_MtoC_Msg.SigGen.Freq, &DP_Framework.NetSignals[N_MAX_NET_SIGNALS-2],
					&DP_Framework.NetSignals[N_MAX_NET_SIGNALS-1], &IPC_MtoC_Msg.SigGen.Aux, DP_Framework.Ref);

	/*
	 * 	      name: 	SRLIM_SIGGEN_AMP
	 * description: 	Signal generator amplitude slew-rate limiter
	 *    DP class:     ELP_SRLim
	 *     	    in:		IPC_MtoC_Msg.SigGen.Amplitude
	 * 		   out:		NetSignals[N_MAX_NET_SIGNALS-2]
	 */

	Init_ELP_SRLim(SRLIM_SIGGEN_AMP, MAX_SR_SIGGEN_AMP, CONTROL_FREQ, IPC_MtoC_Msg.SigGen.Amplitude, &DP_Framework.NetSignals[N_MAX_NET_SIGNALS-2]);

	/*
	 * 	      name: 	SRLIM_SIGGEN_OFFSET
	 * description: 	Signal generator offset slew-rate limiter
	 *    DP class:     ELP_SRLim
	 *     	    in:		IPC_MtoC_Msg.SigGen.Amplitude
	 * 		   out:		NetSignals[N_MAX_NET_SIGNALS-1]
	 */

	Init_ELP_SRLim(SRLIM_SIGGEN_OFFSET, MAX_SR_SIGGEN_OFFSET, CONTROL_FREQ, &IPC_MtoC_Msg.SigGen.Offset, &DP_Framework.NetSignals[N_MAX_NET_SIGNALS-1]);

	/**********************************/
	/* INITIALIZATION OF TIME SLICERS */
	/**********************************/

	// 0: Time-slicer for WfmRef sweep decimation
	Set_TimeSlicer(0, CONTROL_FREQ/WFMREF_SAMPLING_FREQ);

	// 1: Time-slicer for SamplesBuffer
	Set_TimeSlicer(1, BUFFER_DECIMATION);

	// 2: Time-slicer for current share controller
	Set_TimeSlicer(2, ISHARE_DECIMATION);

	ResetControllers();
}

static void ResetControllers(void)
{
	SetPWMDutyCycle_HBridge(PWM_Modules.PWM_Regs[0], 0.0);
	SetPWMDutyCycle_HBridge(PWM_Modules.PWM_Regs[2], 0.0);

	Reset_ELP_SRLim(SRLIM_ILOAD_REFERENCE);
	Reset_ELP_Error(ERROR_CALCULATOR);
	Reset_ELP_Error(ISHARE_ERROR_CALCULATOR);
	Reset_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD);
	Reset_ELP_PI_dawu(PI_DAWU_CONTROLLER_ISHARE);

	Reset_ELP_SRLim(SRLIM_SIGGEN_AMP);
	Reset_ELP_SRLim(SRLIM_SIGGEN_OFFSET);

	IPC_CtoM_Msg.PSModule.IRef = 0.0;

	Reset_TimeSlicers();
}

/*
 * Initialization of application interruptions
 *
 * 		- PWM interruptions as main ISR for control loop (INT3)
 * 		- IPC interruptions (INT11)
 * 		-
 */

static void InitInterruptions(void)
{
	EALLOW;
	PieVectTable.EPWM1_INT =  &isr_ePWM_CTR_ZERO_1st;
	PieVectTable.EPWM2_INT =  &isr_ePWM_CTR_ZERO;
	//PieVectTable.EPWM7_INT =  &isr_ePWM_CTR_ZERO;
	//PieVectTable.EPWM8_INT =  &isr_ePWM_CTR_ZERO;
	EDIS;

	PieCtrlRegs.PIEIER3.bit.INTx1 = 1;  // ePWM1
	PieCtrlRegs.PIEIER3.bit.INTx2 = 1;  // ePWM2
	//PieCtrlRegs.PIEIER3.bit.INTx7 = 1;  // ePWM7
	//PieCtrlRegs.PIEIER3.bit.INTx8 = 1;  // ePWM8

	EnablePWMInterrupt(PWM_Modules.PWM_Regs[0]);
	EnablePWMInterrupt(PWM_Modules.PWM_Regs[1]);
	//EnablePWMInterrupt(PWM_Modules.PWM_Regs[2]);
	//EnablePWMInterrupt(PWM_Modules.PWM_Regs[3]);

	IER |= M_INT1;
	IER |= M_INT3;
	IER |= M_INT11;

	DELAY_US(3000000);

	/* Enable global interrupts (EINT) */
	EINT;
	ERTM;
}

//*****************************************************************************
// Esvazia buffer FIFO com valores amostrados e recebidos via SPI
//*****************************************************************************
interrupt void isr_ePWM_CTR_ZERO(void)
{
	static Uint16 i, bypass_SRLim;
	static float temp0, temp1, temp2;

	//SET_DEBUG_GPIO1;
	if((IPC_CtoM_Msg.PSModule.OpMode == WfmRef) && ((IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferK == IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferStart) || wfmSyncFlag == 1))
	{
		CLEAR_DEBUG_GPIO1;
		wfmSyncFlag = 0;
	}

	temp0 = 0.0;
	temp1 = 0.0;
	temp2 = 0.0;
	
	bypass_SRLim = USE_MODULE;

	for(i = 0; i < DECIMATION_FACTOR; i++)
	{
		temp0 += (float) *(HRADCs_Info.HRADC_boards[0]->SamplesBuffer++);
		temp1 += (float) *(HRADCs_Info.HRADC_boards[1]->SamplesBuffer++);
		temp2 += (float) *(HRADCs_Info.HRADC_boards[2]->SamplesBuffer++);
	}

	HRADCs_Info.HRADC_boards[0]->SamplesBuffer = buffers_HRADC.buffer_0;
	HRADCs_Info.HRADC_boards[1]->SamplesBuffer = buffers_HRADC.buffer_1;
	HRADCs_Info.HRADC_boards[2]->SamplesBuffer = buffers_HRADC.buffer_2;

	temp0 *= AverageFilter;
	temp0 -= *(HRADCs_Info.HRADC_boards[0]->offset);
	temp0 *= *(HRADCs_Info.HRADC_boards[0]->gain);
	temp0 *= HRADC_0_GAIN_ERROR;
	temp0 += HRADC_0_OFFSET_ERROR;

	temp1 *= AverageFilter;
	temp1 -= *(HRADCs_Info.HRADC_boards[1]->offset);
	temp1 *= *(HRADCs_Info.HRADC_boards[1]->gain);

	temp2 *= AverageFilter;
	temp2 -= *(HRADCs_Info.HRADC_boards[2]->offset);
	temp2 *= *(HRADCs_Info.HRADC_boards[2]->gain);

	DP_Framework.NetSignals[1] = temp0;
	DP_Framework.NetSignals[17] = temp1;
	DP_Framework.NetSignals[19] = temp2;

	if(fabs(temp0) > MAX_ILOAD_MEASURED)
	{
		if(CHECK_INTERLOCK(LOAD_OVERCURRENT))
		{
			Set_HardInterlock(LOAD_OVERCURRENT);
		}
	}

	if(fabs(temp1) > MAX_DCLINK || fabs(temp2) > MAX_DCLINK)
	{
		if(CHECK_INTERLOCK(IN_OVERVOLTAGE))
		{
			Set_HardInterlock(IN_OVERVOLTAGE);
		}
	}

	Run_ELP_IIR_2P2Z(IIR_2P2Z_LPF_VDCLINK_MOD1);
	Run_ELP_IIR_2P2Z(IIR_2P2Z_LPF_VDCLINK_MOD2);

	if(IPC_CtoM_Msg.PSModule.OnOff)
	{
		switch(IPC_CtoM_Msg.PSModule.OpMode)
		{
			case FastRef:
				bypass_SRLim = BYPASS_MODULE;
				break;

			case WfmRef:

				switch(IPC_CtoM_Msg.WfmRef.SyncMode)
				{
					case OneShot:
					{
						RUN_TIMESLICE(0); /************************************************************/

							if(IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferK <= IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferEnd)
							{
								IPC_CtoM_Msg.PSModule.IRef = *(IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferK++) * IPC_CtoM_Msg.WfmRef.Gain + IPC_CtoM_Msg.WfmRef.Offset;
							}

						END_TIMESLICE(0); /************************************************************/
						break;
					}

					case SampleBySample:
					case SampleBySample_Continuous:
					{
						if(IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferK <= IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferEnd)
						{
							IPC_CtoM_Msg.PSModule.IRef = *(IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferK) * IPC_CtoM_Msg.WfmRef.Gain + IPC_CtoM_Msg.WfmRef.Offset;
						}
						break;
					}
				}

				bypass_SRLim = BYPASS_MODULE;
				break;

			case SigGen:
				Run_ELP_SRLim(SRLIM_SIGGEN_AMP, USE_MODULE);
				Run_ELP_SRLim(SRLIM_SIGGEN_OFFSET, USE_MODULE);
				SignalGenerator.Run_ELP_SigGen(&SignalGenerator);
				bypass_SRLim = BYPASS_MODULE;
				break;

			default:
				break;
		}

		Run_ELP_SRLim(SRLIM_ILOAD_REFERENCE, bypass_SRLim);

		if(IPC_CtoM_Msg.PSModule.OpenLoop)
		{
			DP_Framework.DutySignals[0] = 0.01*DP_Framework.NetSignals[0];				// For open loop, Iref value represents duty-cycle
			DP_Framework.DutySignals[1] = 0.01*DP_Framework.NetSignals[0];
			SATURATE(DP_Framework.DutySignals[0], PWM_MAX_DUTY_OL, PWM_MIN_DUTY_OL);	// in percentage (0 - 100 A => 0 - 100 %)
			SATURATE(DP_Framework.DutySignals[1], PWM_MAX_DUTY_OL, PWM_MIN_DUTY_OL);	// in percentage (0 - 100 A => 0 - 100 %)
		}
		else
		{
			SATURATE(DP_Framework.NetSignals[0], MAX_REF, MIN_REF);
			Run_ELP_Error(ERROR_CALCULATOR);
			Run_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD);
			Run_ELP_IIR_2P2Z(IIR_2P2Z_FF_REFERENCE);
			DP_Framework.NetSignals[14] = DP_Framework.NetSignals[9] + DP_Framework.NetSignals[13];

			RUN_TIMESLICE(2); /************************************************************/

				DP_Framework.NetSignals[12] = DP_Framework.NetSignals[1] * 0.5;
				Run_ELP_Error(ISHARE_ERROR_CALCULATOR);
				Run_ELP_PI_dawu(PI_DAWU_CONTROLLER_ISHARE);

			END_TIMESLICE(2); /************************************************************/

			DP_Framework.NetSignals[15] = DP_Framework.NetSignals[14] - DP_Framework.NetSignals[11];
			DP_Framework.NetSignals[16] = DP_Framework.NetSignals[14] + DP_Framework.NetSignals[11];

			Run_ELP_DCLink_FF(FF_VDCLINK_MOD1);
			Run_ELP_DCLink_FF(FF_VDCLINK_MOD2);

			SATURATE(DP_Framework.DutySignals[0], PWM_MAX_DUTY, PWM_MIN_DUTY);
			SATURATE(DP_Framework.DutySignals[1], PWM_MAX_DUTY, PWM_MIN_DUTY);
		}

		SetPWMDutyCycle_HBridge(PWM_Modules.PWM_Regs[0], DP_Framework.DutySignals[0]);
		SetPWMDutyCycle_HBridge(PWM_Modules.PWM_Regs[2], DP_Framework.DutySignals[1]);
	}

	RUN_TIMESLICE(1); /************************************************************/

		WriteBuffer(&IPC_CtoM_Msg.SamplesBuffer, DP_Framework.NetSignals[8]);		// iLoad error signal

	END_TIMESLICE(1); /************************************************************/

	for(i = 0; i < PWM_Modules.N_modules; i++)
	{
		PWM_Modules.PWM_Regs[i]->ETCLR.bit.INT = 1;
	}

	//CLEAR_DEBUG_GPIO1;

	PieCtrlRegs.PIEACK.all |= M_INT3;
}

static interrupt void isr_ePWM_CTR_ZERO_1st(void)
{
	// Contador auxiliar
	static Uint16 i;

	//GpioG1DataRegs.GPDSET.bit.GPIO111 = 1;

    // Remapeia a ISR que esvazia buffer FIFO
	EALLOW;
    PieVectTable.EPWM1_INT = &isr_ePWM_CTR_ZERO;
    EDIS;

	for(i = 0; i < PWM_Modules.N_modules; i++)
	{
		PWM_Modules.PWM_Regs[i]->ETSEL.bit.INTSEL = ET_CTR_ZERO;
		PWM_Modules.PWM_Regs[i]->ETCLR.bit.INT = 1;
	}

	//GpioG1DataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all |= M_INT3;
}

static void Set_SoftInterlock(Uint32 itlk)
{
	PS_turnOff();
	IPC_CtoM_Msg.PSModule.SoftInterlocks |= itlk;
	//SendIpcFlag(SOFT_INTERLOCK_CTOM);
}

static void Set_HardInterlock(Uint32 itlk)
{
	PS_turnOff();
	IPC_CtoM_Msg.PSModule.HardInterlocks |= itlk;
	SendIpcFlag(HARD_INTERLOCK_CTOM);

	if(itlk != ACDC_FAULT)
	{
		PIN_SET_DCDC_INTERLOCK;
	}
}

static interrupt void isr_SoftInterlock(void)
{
	CtoMIpcRegs.MTOCIPCACK.all = SOFT_INTERLOCK_MTOC;

	PS_turnOff();
	IPC_CtoM_Msg.PSModule.SoftInterlocks |= IPC_MtoC_Msg.PSModule.SoftInterlocks;
	//SendIpcFlag(SOFT_INTERLOCK_CTOM);

	PieCtrlRegs.PIEACK.all |= M_INT11;
}

static interrupt void isr_HardInterlock(void)
{
	CtoMIpcRegs.MTOCIPCACK.all = HARD_INTERLOCK_MTOC;

	PS_turnOff();
	IPC_CtoM_Msg.PSModule.HardInterlocks |= IPC_MtoC_Msg.PSModule.HardInterlocks;
	//SendIpcFlag(HARD_INTERLOCK_CTOM);

	if(IPC_MtoC_Msg.PSModule.HardInterlocks != ACDC_FAULT)
	{
		PIN_SET_DCDC_INTERLOCK;
	}

	PieCtrlRegs.PIEACK.all |= M_INT11;
}

static void PS_turnOn(void)
{
	if(CHECK_INTERLOCKS && CHECK_SOFTINTERLOCK(ACDC_FAULT))
	{
		//ResetControllers();

		IPC_CtoM_Msg.PSModule.IRef = 0.0;
		IPC_CtoM_Msg.PSModule.OpenLoop = 1;
		IPC_CtoM_Msg.PSModule.OnOff = 1;

		EnablePWMOutputs();
	}
}

static void PS_turnOff(void)
{
	DisablePWMOutputs();

	IPC_CtoM_Msg.PSModule.OnOff = 0;
	IPC_CtoM_Msg.PSModule.OpenLoop = 1;

	ResetControllers();
}

