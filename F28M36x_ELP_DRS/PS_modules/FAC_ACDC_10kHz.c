/*
 * 		FILE: 		main_FAC_ACDC.c
 * 		PROJECT: 	DRS v2.0
 * 		CREATION:
 * 		MODIFIED:	07/04/2016
 *
 * 		AUTHOR: 	Gabriel O. B.  (LNLS/ELP)
 *
 * 		DESCRIPTION: Firmware for control of AC/DC stage of prototype FAC v2.0
 *
 *
 *		TODO:
 *				- Interlocks
 *				- Malhas de corrente e tensão
 *				- Decimação do controle de tensão
 */

#include "F28M36x_ELP_DRS.h"
#include "FAC_ACDC_10kHz.h"

// Prototype statements for functions found within this file.

#pragma CODE_SECTION(isr_ePWM_CTR_ZERO, "ramfuncs");
#pragma CODE_SECTION(isr_ePWM_CTR_ZERO_1st, "ramfuncs");
#pragma CODE_SECTION(PS_turnOn, "ramfuncs");
#pragma CODE_SECTION(PS_turnOff, "ramfuncs");
#pragma CODE_SECTION(Set_SoftInterlock, "ramfuncs");
#pragma CODE_SECTION(Set_HardInterlock, "ramfuncs");
#pragma CODE_SECTION(isr_SoftInterlock, "ramfuncs");
#pragma CODE_SECTION(isr_HardInterlock, "ramfuncs");

static __interrupt void isr_ePWM_CTR_ZERO(void);
static __interrupt void isr_ePWM_CTR_ZERO_1st(void);

void main_FAC_ACDC_10kHz(void);

static void PS_turnOn(void);
static void PS_turnOff(void);

static void Set_SoftInterlock(Uint32 itlk);
static void Set_HardInterlock(Uint32 itlk);
static interrupt void isr_SoftInterlock(void);
static interrupt void isr_HardInterlock(void);

static void InitPeripheralsDrivers(void);
static void ResetPeripheralsDrivers(void);

static void InitControllers(void);
static void ResetControllers(void);

static void InitInterruptions(void);

static Uint16 pinStatus_AC_Contactor, pinStatus_Ext_Itlk, pintStatus_DCDC_Itlk;
static Uint32 valorCounter;

void main_FAC_ACDC_10kHz(void)
{
	InitPeripheralsDrivers();

	InitControllers();

	InitInterruptions();

	stop_DMA();
	DELAY_US(5);
	start_DMA();
	EnablePWM_TBCLK();

	while(1)
	{
		DINT;

		pinStatus_AC_Contactor = PIN_STATUS_AC_CONTACTOR;
		pinStatus_Ext_Itlk = PIN_STATUS_EXT_INTERLOCK;
		pintStatus_DCDC_Itlk = PIN_STATUS_DCDC_INTERLOCK;

		if(pinStatus_Ext_Itlk)
		{
			if(CHECK_INTERLOCK(EXTERNAL_INTERLOCK))
			{
				//Set_HardInterlock(EXTERNAL_INTERLOCK);
			}
		}

		if(pintStatus_DCDC_Itlk)
		{
			if(CHECK_INTERLOCK(DCDC_FAULT))
			{
				Set_HardInterlock(DCDC_FAULT);
			}
		}

		if(IPC_CtoM_Msg.PSModule.OnOff && !pinStatus_AC_Contactor)
		{
			if(CHECK_INTERLOCK(AC_FAULT))
			{
				Set_HardInterlock(AC_FAULT);
			}
		}

		if(CHECK_INTERLOCKS)
		{
			PIN_CLEAR_ACDC_INTERLOCK;
		}

		EINT;

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
	/* Initialization of HRADC boards */

	stop_DMA();

	Init_DMA_McBSP_nBuffers(2, DECIMATION_FACTOR, HRADC_SPI_CLK);

	Init_SPIMaster_McBSP(HRADC_SPI_CLK);
	Init_SPIMaster_Gpio();
	InitMcbspa20bit();

	HRADCs_Info.HRADC_boards[0] = &HRADC0_board;
	Init_HRADC_Info(HRADCs_Info.HRADC_boards[0], 0, DECIMATION_FACTOR, buffers_HRADC.buffer_0, TRANSDUCER_0_GAIN, HRADC_0_R_BURDEN);
	Config_HRADC_board(HRADCs_Info.HRADC_boards[0], TRANSDUCER_0_OUTPUT_TYPE, HEATER_DISABLE, RAILS_DISABLE);

	HRADCs_Info.HRADC_boards[1] = &HRADC1_board;
	Init_HRADC_Info(HRADCs_Info.HRADC_boards[1], 1, DECIMATION_FACTOR, buffers_HRADC.buffer_1, TRANSDUCER_1_GAIN, HRADC_1_R_BURDEN);
	Config_HRADC_board(HRADCs_Info.HRADC_boards[1], TRANSDUCER_1_OUTPUT_TYPE, HEATER_DISABLE, RAILS_DISABLE);

	AverageFilter = 1.0/((float) DECIMATION_FACTOR);

	Config_HRADC_SoC(HRADC_FREQ_SAMP);

	/* Initialization of PWM modules */

    EALLOW;
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO29 = 1;			// Auxiliar GPIO for which GPTRIP1 is selected
	GpioDataRegs.GPASET.bit.GPIO29 = 1;
	EDIS;

	PWM_Modules.N_modules = 1;
	PWM_Modules.PWM_Regs[0] = &EPwm1Regs;

	DisablePWMOutputs();
	DisablePWM_TBCLK();
	InitPWM_MEP_SFO();

	InitPWMModule(PWM_Modules.PWM_Regs[0], PWM_FREQ, 0, MasterPWM, 0, NO_COMPLEMENTARY, PWM_DEAD_TIME);

	InitPWMDAC(PWM_DAC_MODULE, PWM_DAC_FREQ);

	InitEPwm1Gpio();
	InitEPwm4Gpio();

	/* Initialization of GPIOs */

	EALLOW;

	GpioCtrlRegs.GPDMUX2.bit.GPIO126 = 0;
	GpioDataRegs.GPDCLEAR.bit.GPIO126 = 1;		// GPDI1: AC mains contactor status
	GpioCtrlRegs.GPDDIR.bit.GPIO126 = 0;

	GpioCtrlRegs.GPDMUX2.bit.GPIO127 = 0;
	GpioDataRegs.GPDCLEAR.bit.GPIO127 = 1;		// GPDI2: External interlock signal
	GpioCtrlRegs.GPDDIR.bit.GPIO127 = 0;

	GpioCtrlRegs.GPDMUX2.bit.GPIO124 = 0;
	GpioDataRegs.GPDCLEAR.bit.GPIO124 = 1;		// GPDI3: DC/DC Interlock
	GpioCtrlRegs.GPDDIR.bit.GPIO124 = 0;

	GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 0;
	GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;		// GPDO1: AC mains contactor switch
	GpioCtrlRegs.GPCDIR.bit.GPIO67 = 1;

	GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 0;
	GpioDataRegs.GPCSET.bit.GPIO65 = 1;			// GPDO2: AC/DC Interlock
	GpioCtrlRegs.GPCDIR.bit.GPIO65 = 1;

	INIT_DEBUG_GPIO1;

	EDIS;

	/* Initialization of timers */
	InitCpuTimers();
	CpuTimer0Regs.TCR.bit.TIE = 0;
	CpuTimer1Regs.TCR.bit.TIE = 0;
	CpuTimer2Regs.TCR.bit.TIE = 0;
}

static void ResetPeripheralsDrivers(void)
{
	DisablePWM_TBCLK();
	stop_DMA();

	Init_DMA_McBSP_nBuffers(2, DECIMATION_FACTOR, HRADC_SPI_CLK);

	Init_SPIMaster_McBSP(HRADC_SPI_CLK);
	Init_SPIMaster_Gpio();
	InitMcbspa20bit();

	HRADCs_Info.HRADC_boards[0] = &HRADC0_board;
	HRADCs_Info.HRADC_boards[1] = &HRADC1_board;

	Config_HRADC_board(HRADCs_Info.HRADC_boards[0], TRANSDUCER_0_OUTPUT_TYPE, HEATER_DISABLE, RAILS_DISABLE);
	Config_HRADC_board(HRADCs_Info.HRADC_boards[1], TRANSDUCER_1_OUTPUT_TYPE, HEATER_DISABLE, RAILS_DISABLE);

	stop_DMA();
	DELAY_US(5);
	start_DMA();
	EnablePWM_TBCLK();
}

static void InitControllers(void)
{
	/* Initialization of IPC module */
	InitIPC(&PS_turnOn, &PS_turnOff, &isr_SoftInterlock, &isr_HardInterlock);
	IPC_CtoM_Msg.SamplesBuffer.BufferBusy = Buffer_All;

	/* Initiaization of DP Framework */
	InitDP_Framework(&DP_Framework, &(IPC_CtoM_Msg.PSModule.IRef));

	/***************************************************/
	/* INITIALIZATION OF CAP BANK VOLTAGE CONTROL LOOP */
	/***************************************************/

	/*
	 * 	      name: 	SRLIM_V_CAPBANK_REFERENCE
	 * description: 	Cap bank voltage reference slew-rate limiter
	 *    DP class:     ELP_SRLim
	 *     	    in:		DP_Framework.Ref
	 * 		   out:		NetSignals[0]
	 */

	Init_ELP_SRLim(SRLIM_V_CAPBANK_REFERENCE, MAX_REF_SLEWRATE, V_CONTROL_FREQ, DP_Framework.Ref, &DP_Framework.NetSignals[0]);

	/*
	 * 	      name: 	V_ERROR_CALCULATOR
	 * description: 	Cap bank voltage reference error
	 *    DP class:     ELP_Error
	 *     	    +:		NetSignals[0]
	 *     	    -:		NetSignals[9]
	 * 		   out:		NetSignals[10]
	 */

	Init_ELP_Error(V_ERROR_CALCULATOR, &DP_Framework.NetSignals[0], &DP_Framework.NetSignals[9], &DP_Framework.NetSignals[10]);

	/*
	 * 	      name: 	PI_DAWU_CONTROLLER_VCAPBANK
	 * description: 	Cap bank voltage PI controller
	 *    DP class:     ELP_PI_dawu
	 *     	    in:		NetSignals[10]
	 * 		   out:		NetSignals[11]
	 */

	//Init_ELP_PI_dawu(PI_DAWU_CONTROLLER_VCAPBANK, KP, KI, V_CONTROL_FREQ, MAX_IIN_REF, -MAX_IIN_REF, &DP_Framework.NetSignals[10], &DP_Framework.NetSignals[11]);
	Init_ELP_PI_dawu(PI_DAWU_CONTROLLER_VCAPBANK, KP, KI, V_CONTROL_FREQ, MAX_IIN_REF, MIN_IIN_REF, &DP_Framework.NetSignals[10], &DP_Framework.NetSignals[11]);

	/*
	 * 	      name: 	IIR_2P2Z_CONTROLLER_VCAPBANK
	 * description: 	Cap bank voltage IIR 2P2Z controller
	 *    DP class:     ELP_IIR_2P2Z
	 *     	    in:		NetSignals[10]
	 * 		   out:		NetSignals[11]
	 */

	Init_ELP_IIR_2P2Z(IIR_2P2Z_CONTROLLER_VCAPBANK, 0.107277884239820e-5, 0.214555768479641e-5, 0.107277884239820e-5,
					  -1.999804706571624, 0.999804706571624, MAX_IIN_REF, MIN_IIN_REF, &DP_Framework.NetSignals[10], &DP_Framework.NetSignals[11]);

	/*
	 * 	      name: 	NF_V_CAPBANK_2HZ
	 * description: 	Cap bank voltage notch filter (Fcut = 2 Hz)
	 *    DP class:     ELP_IIR_2P2Z
	 *     	    in:		NetSignals[7]
	 * 		   out:		NetSignals[8]
	 */

	Init_ELP_NF_IIR(NF_V_CAPBANK_2HZ, NF_ALPHA, 2.0, V_CONTROL_FREQ, FLT_MAX, -FLT_MAX, &DP_Framework.NetSignals[7], &DP_Framework.NetSignals[8]);
	//Init_ELP_IIR_2P2Z(NF_V_CAPBANK_2HZ, 0.998441278934479, -1.996732234954830, 0.998441278934479,
	//					  -1.996732234954830, 0.996882617473602, FLT_MAX, -FLT_MAX, &DP_Framework.NetSignals[7], &DP_Framework.NetSignals[8]);

	/*
	 * 	      name: 	NF_V_CAPBANK_4HZ
	 * description: 	Cap bank voltage notch filter (Fcut = 4 Hz)
	 *    DP class:     ELP_IIR_2P2Z
	 *     	    in:		NetSignals[8]
	 * 		   out:		NetSignals[9]
	 */

	Init_ELP_NF_IIR(NF_V_CAPBANK_4HZ, NF_ALPHA, 4.0, V_CONTROL_FREQ, FLT_MAX, -FLT_MAX, &DP_Framework.NetSignals[8], &DP_Framework.NetSignals[9]);
	//Init_ELP_IIR_2P2Z(NF_V_CAPBANK_4HZ, 0.998441278934479, -1.996281147003170, 0.998441278934479,
	//					  -1.996281147003170, 0.996882617473602, FLT_MAX, -FLT_MAX, &DP_Framework.NetSignals[8], &DP_Framework.NetSignals[2]);

	/*********************************************************/
	/* INITIALIZATION OF INPUT INDUCTOR CURRENT CONTROL LOOP */
	/*********************************************************/

	/*
	 * 	      name: 	I_ERROR_CALCULATOR
	 * description: 	Input inductor current reference error
	 *    DP class:     ELP_Error
	 *     	     +:		NetSignals[11]
	 *     	     -:		NetSignals[5]
	 * 		   out:		NetSignals[12]
	 */

	Init_ELP_Error(I_ERROR_CALCULATOR, &DP_Framework.NetSignals[11], &DP_Framework.NetSignals[5], &DP_Framework.NetSignals[12]);

	/*
	 * 	      name: 	PI_DAWU_CONTROLLER_IIN
	 * description: 	Input inductor current PI controller
	 *    DP class:     ELP_PI_dawu
	 *     	    in:		NetSignals[12]
	 * 		   out:		DutySignals[0]
	 */
	//Init_ELP_PI_dawu(PI_DAWU_CONTROLLER_IIN, KP2, KI2, CONTROL_FREQ, PWM_MAX_DUTY, -PWM_MAX_DUTY, &DP_Framework.NetSignals[12], &DP_Framework.DutySignals[0]);
	Init_ELP_PI_dawu(PI_DAWU_CONTROLLER_IIN, KP2, KI2, CONTROL_FREQ, PWM_MAX_DUTY, PWM_MIN_DUTY, &DP_Framework.NetSignals[12], &DP_Framework.DutySignals[0]);

	/*
	 * 	      name: 	IIR_2P2Z_CONTROLLER_IIN
	 * description: 	Input inductor current IIR 2P2Z controller
	 *    DP class:     ELP_IIR_2P2Z
	 *     	    in:		NetSignals[12]
	 * 		   out:		DutySignals[0]
	 */

	Init_ELP_IIR_2P2Z(IIR_2P2Z_CONTROLLER_IIN, 0.049231877100336, -0.049087854056649, 0.0,
					  -0.966394623139702, 0.0, PWM_MAX_DUTY, PWM_MIN_DUTY, &DP_Framework.NetSignals[12], &DP_Framework.DutySignals[0]);

	/*********************************************/
	/* INITIALIZATION OF SIGNAL GENERATOR MODULE */
	/*********************************************/

	/*
	 * 	      name: 	SignalGenerator
	 * description: 	Signal generator module
	 * 		   out:		DP_Framework.Ref
	 */

	Disable_ELP_SigGen(&SignalGenerator);
	Init_ELP_SigGen(&SignalGenerator, Sine, 0.0, 0.0, 10, V_CONTROL_FREQ, &IPC_MtoC_Msg.SigGen.Freq,
					&DP_Framework.NetSignals[N_MAX_NET_SIGNALS-2], &DP_Framework.NetSignals[N_MAX_NET_SIGNALS-1], &IPC_MtoC_Msg.SigGen.Aux, DP_Framework.Ref);

	/*
	 * 	      name: 	SRLIM_SIGGEN_AMP
	 * description: 	Signal generator amplitude slew-rate limiter
	 *    DP class:     ELP_SRLim
	 *     	    in:		IPC_MtoC_Msg.SigGen.Amplitude
	 * 		   out:		NetSignals[N_MAX_NET_SIGNALS-2]
	 */

	Init_ELP_SRLim(SRLIM_SIGGEN_AMP, MAX_SR_SIGGEN_AMP, V_CONTROL_FREQ, IPC_MtoC_Msg.SigGen.Amplitude, &DP_Framework.NetSignals[N_MAX_NET_SIGNALS-2]);

	/*
	 * 	      name: 	SRLIM_SIGGEN_OFFSET
	 * description: 	Signal generator offset slew-rate limiter
	 *    DP class:     ELP_SRLim
	 *     	    in:		IPC_MtoC_Msg.SigGen.Amplitude
	 * 		   out:		NetSignals[N_MAX_NET_SIGNALS-1]
	 */

	Init_ELP_SRLim(SRLIM_SIGGEN_OFFSET, MAX_SR_SIGGEN_OFFSET, V_CONTROL_FREQ, &IPC_MtoC_Msg.SigGen.Offset, &DP_Framework.NetSignals[N_MAX_NET_SIGNALS-1]);


	/**********************************/
	/* INITIALIZATION OF TIME SLICERS */
	/**********************************/

	// 0: Time-slicer for WfmRef sweep decimation
	Set_TimeSlicer(0, V_CONTROL_FREQ/WFMREF_SAMPLING_FREQ);

	// 1: Time-slicer for SamplesBuffer
	Set_TimeSlicer(1, BUFFER_DECIMATION);

	// 2: Time-slicer for cap bank voltage control loop
	Set_TimeSlicer(2, V_CONTROL_DECIMATION);

	ResetControllers();
}

static void ResetControllers(void)
{
	SetPWMDutyCycle_ChA(PWM_Modules.PWM_Regs[0], 0.0);

	Reset_ELP_SRLim(SRLIM_V_CAPBANK_REFERENCE);
	Reset_ELP_Error(V_ERROR_CALCULATOR);
	Reset_ELP_PI_dawu(PI_DAWU_CONTROLLER_VCAPBANK);
	Reset_ELP_IIR_2P2Z(NF_V_CAPBANK_2HZ);
	Reset_ELP_IIR_2P2Z(NF_V_CAPBANK_4HZ);
	Reset_ELP_IIR_2P2Z(IIR_2P2Z_CONTROLLER_VCAPBANK);

	Reset_ELP_Error(I_ERROR_CALCULATOR);
	Reset_ELP_PI_dawu(PI_DAWU_CONTROLLER_IIN);
	Reset_ELP_IIR_2P2Z(IIR_2P2Z_CONTROLLER_IIN);

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
	EDIS;

	PieCtrlRegs.PIEIER3.bit.INTx1 = 1;  // ePWM1
	EnablePWMInterrupt(PWM_Modules.PWM_Regs[0]);

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
static __interrupt void isr_ePWM_CTR_ZERO(void)
{
	static Uint16 i, bypass_SRLim;
	static float temp0, temp1;

	temp0 = 0.0;
	temp1 = 0.0;

	bypass_SRLim = USE_MODULE;

	SET_DEBUG_GPIO1;
	//StartCpuTimer0();

	//while(!McbspaRegs.SPCR1.bit.RRDY){}

	for(i = 0; i < DECIMATION_FACTOR; i++)
	{
		temp0 += (float) *(HRADCs_Info.HRADC_boards[0]->SamplesBuffer++);
		temp1 += (float) *(HRADCs_Info.HRADC_boards[1]->SamplesBuffer++);
	}

	HRADCs_Info.HRADC_boards[0]->SamplesBuffer = buffers_HRADC.buffer_0;
	HRADCs_Info.HRADC_boards[1]->SamplesBuffer = buffers_HRADC.buffer_1;

	temp0 *= AverageFilter;
	temp1 *= AverageFilter;

	temp0 -= *(HRADCs_Info.HRADC_boards[0]->offset);
	temp0 *= *(HRADCs_Info.HRADC_boards[0]->gain);

	temp1 -= *(HRADCs_Info.HRADC_boards[1]->offset);
	temp1 *= *(HRADCs_Info.HRADC_boards[1]->gain);

	DP_Framework.NetSignals[7] = temp0;
	DP_Framework.NetSignals[5] = temp1;

	if(fabs(temp0) > MAX_LOAD)
	{
		if(CHECK_INTERLOCK(OUT_OVERVOLTAGE))
		{
			Set_HardInterlock(OUT_OVERVOLTAGE);
		}
	}

	if(fabs(temp1) > MAX_IIN)
	{
		if(CHECK_INTERLOCK(IN_OVERCURRENT))
		{
			Set_HardInterlock(IN_OVERCURRENT);
		}
	}

	RUN_TIMESLICE(2); /************************************************************/

		Run_ELP_IIR_2P2Z(NF_V_CAPBANK_2HZ);
		Run_ELP_IIR_2P2Z(NF_V_CAPBANK_4HZ);

		if(IPC_CtoM_Msg.PSModule.OnOff)
		{
			switch(IPC_CtoM_Msg.PSModule.OpMode)
			{
				case FastRef:
					bypass_SRLim = BYPASS_MODULE;
					break;

				case WfmRef:

					RUN_TIMESLICE(0); /************************************************************/

						/*if(IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferK <= IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferEnd)
						{
							IPC_CtoM_Msg.PSModule.IRef = *(IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferK++) * IPC_CtoM_Msg.WfmRef.Gain + IPC_CtoM_Msg.WfmRef.Offset;
						}
						else
						{
							CLEAR_DEBUG_GPIO1;
						}*/

					END_TIMESLICE(0); /************************************************************/

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

			Run_ELP_SRLim(SRLIM_V_CAPBANK_REFERENCE,bypass_SRLim);

			if(IPC_CtoM_Msg.PSModule.OpenLoop)
			{
				DP_Framework.DutySignals[0] = 0.01*DP_Framework.NetSignals[0];				// For open loop, Iref value represents duty-cycle
				SATURATE(DP_Framework.DutySignals[0], PWM_MAX_DUTY_OL, PWM_MIN_DUTY_OL);	// in percentage (0 - 100 A => 0 - 100 %)
			}
			else
			{
				SATURATE(DP_Framework.NetSignals[0], MAX_REF, MIN_REF);
				Run_ELP_Error(V_ERROR_CALCULATOR);
				Run_ELP_PI_dawu(PI_DAWU_CONTROLLER_VCAPBANK);
				//Run_ELP_IIR_2P2Z(IIR_2P2Z_CONTROLLER_VCAPBANK);
				SATURATE(DP_Framework.NetSignals[11], MAX_IIN_REF, MIN_IIN_REF);
			}
		}

	END_TIMESLICE(2); /************************************************************/

	if(IPC_CtoM_Msg.PSModule.OnOff)
	{
		if(!IPC_CtoM_Msg.PSModule.OpenLoop)
		{
			Run_ELP_Error(I_ERROR_CALCULATOR);
			Run_ELP_PI_dawu(PI_DAWU_CONTROLLER_IIN);
			//Run_ELP_IIR_2P2Z(IIR_2P2Z_CONTROLLER_IIN);
			SATURATE(DP_Framework.DutySignals[0], PWM_MAX_DUTY, PWM_MIN_DUTY);
		}

		SetPWMDutyCycle_ChA(PWM_Modules.PWM_Regs[0], DP_Framework.DutySignals[0]);
	}

	RUN_TIMESLICE(1); /************************************************************/

		WriteBuffer(&IPC_CtoM_Msg.SamplesBuffer, DP_Framework.NetSignals[7]);

	END_TIMESLICE(1); /************************************************************/

	for(i = 0; i < PWM_Modules.N_modules; i++)
	{
		PWM_Modules.PWM_Regs[i]->ETCLR.bit.INT = 1;
	}

	CLEAR_DEBUG_GPIO1;

	/*StopCpuTimer0();
	valorCounter = ReadCpuTimer0Counter();
	ReloadCpuTimer0();*/

	PieCtrlRegs.PIEACK.all |= M_INT3;
}

static __interrupt void isr_ePWM_CTR_ZERO_1st(void)
{
	// Contador auxiliar
	static Uint16 i;

    // Remapeia a ISR que esvazia buffer FIFO
	EALLOW;
    PieVectTable.EPWM1_INT = &isr_ePWM_CTR_ZERO;
    EDIS;

	for(i = 0; i < PWM_Modules.N_modules; i++)
	{
		PWM_Modules.PWM_Regs[i]->ETSEL.bit.INTSEL = ET_CTR_ZERO;
		PWM_Modules.PWM_Regs[i]->ETCLR.bit.INT = 1;
	}

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

	if(itlk != DCDC_FAULT)
	{
		PIN_SET_ACDC_INTERLOCK;
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

	if(IPC_MtoC_Msg.PSModule.HardInterlocks != DCDC_FAULT)
	{
		PIN_SET_ACDC_INTERLOCK;
	}

	PieCtrlRegs.PIEACK.all |= M_INT11;
}

static void PS_turnOn(void)
{
	if(CHECK_INTERLOCKS && CHECK_SOFTINTERLOCK(DCDC_FAULT))
	//if(CHECK_INTERLOCKS && !PIN_STATUS_EXT_INTERLOCK)
	{
		// Configure CPU Timer 1 for DC link contactor timeout monitor
		ConfigCpuTimer(&CpuTimer1, C28_FREQ_MHZ, TIMEOUT_uS_AC_CONTACTOR);
		CpuTimer1Regs.TCR.all = 0x8000;

		PIN_CLOSE_AC_CONTACTOR;

		// Start timeout monitor
		StartCpuTimer1();

		// Check DClink contactor status
		while(!PIN_STATUS_AC_CONTACTOR)
		{
			// If timeout, set interlock
			if(CpuTimer1Regs.TCR.bit.TIF)
			{
				Set_HardInterlock(AC_FAULT);
				StopCpuTimer1();
				CpuTimer1Regs.TCR.all = 0x8000;
				return;
			}
		}

		DELAY_US(100000);

		IPC_CtoM_Msg.PSModule.IRef = 0.0;
		IPC_CtoM_Msg.PSModule.OpenLoop = OPEN_LOOP;
		IPC_CtoM_Msg.PSModule.OnOff = 1;

		EnablePWMOutputs();
	}
}

static void PS_turnOff(void)
{
	DisablePWMOutputs();

	// Configure CPU Timer 1 for DC link contactor timeout monitor
	ConfigCpuTimer(&CpuTimer1, C28_FREQ_MHZ, TIMEOUT_uS_AC_CONTACTOR);
	CpuTimer1Regs.TCR.all = 0x8000;

	PIN_OPEN_AC_CONTACTOR;

	// Start timeout monitor
	StartCpuTimer1();

	// Check DClink contactor status
	while(PIN_STATUS_AC_CONTACTOR)
	{
		// If timeout, set interlock
		if(CpuTimer1Regs.TCR.bit.TIF)
		{
			IPC_CtoM_Msg.PSModule.HardInterlocks |= AC_FAULT;
			SendIpcFlag(HARD_INTERLOCK_CTOM);
			StopCpuTimer1();
			CpuTimer1Regs.TCR.all = 0x8000;
			break;
		}
	}

	IPC_CtoM_Msg.PSModule.OnOff = 0;
	IPC_CtoM_Msg.PSModule.OpenLoop = OPEN_LOOP;

	ResetPeripheralsDrivers();
	ResetControllers();
}
