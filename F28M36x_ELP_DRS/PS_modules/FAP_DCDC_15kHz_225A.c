/*
 * 		FILE: 		FAP_DCDC_15kHz_225A.c
 * 		PROJECT: 	DRS v2.0
 * 		CREATION:	06/06/2016
 * 		MODIFIED:
 *
 * 		AUTHOR: 	Gabriel O. B.  (LNLS/ELP)
 *
 * 		DESCRIPTION: Firmware for control of DC/DC stage for
 * 					 200 A / 50 V FAP prototype v2.0
 *
 *
 *		TODO:
 */

#include "F28M36x_ELP_DRS.h"
#include "FAP_DCDC_15kHz_225A.h"

// Prototype statements for functions found within this file.

#pragma CODE_SECTION(isr_ePWM_CTR_ZERO, "ramfuncs");
#pragma CODE_SECTION(isr_ePWM_CTR_ZERO_1st, "ramfuncs");
#pragma CODE_SECTION(PS_turnOn, "ramfuncs");
#pragma CODE_SECTION(PS_turnOff, "ramfuncs");
#pragma CODE_SECTION(Set_SoftInterlock, "ramfuncs");
#pragma CODE_SECTION(Set_HardInterlock, "ramfuncs");
#pragma CODE_SECTION(isr_SoftInterlock, "ramfuncs");
#pragma CODE_SECTION(isr_HardInterlock, "ramfuncs");
#pragma CODE_SECTION(ResetControllers, "ramfuncs");

static interrupt void isr_ePWM_CTR_ZERO(void);
static interrupt void isr_ePWM_CTR_ZERO_1st(void);

void main_FAP_DCDC_15kHz_225A(void);

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

static Uint16 pinStatus_AC_Contactor;

void main_FAP_DCDC_15kHz_225A(void)
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

		if(IPC_CtoM_Msg.PSModule.OnOff)
		{
			if( CHECK_INTERLOCK(AC_FAULT) && !pinStatus_AC_Contactor )
			{
				Set_HardInterlock(AC_FAULT);
			}
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
	stop_DMA();

	/* Initialization of HRADC boards */

	Init_DMA_McBSP_nBuffers(1, DECIMATION_FACTOR, HRADC_SPI_CLK);

	Init_SPIMaster_McBSP(HRADC_SPI_CLK);
	Init_SPIMaster_Gpio();
	InitMcbspa20bit();

	HRADCs_Info.HRADC_boards[0] = &HRADC0_board;

	Init_HRADC_Info(HRADCs_Info.HRADC_boards[0], 0, DECIMATION_FACTOR, buffers_HRADC.buffer_0, TRANSDUCER_0_GAIN, HRADC_0_R_BURDEN);

	Config_HRADC_board(HRADCs_Info.HRADC_boards[0], TRANSDUCER_0_OUTPUT_TYPE, HEATER_DISABLE, RAILS_DISABLE);

	AverageFilter = 1.0/((float) DECIMATION_FACTOR);

	Config_HRADC_SoC(HRADC_FREQ_SAMP);

	/* Initialization of PWM modules */

    EALLOW;
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO29 = 1;			// Auxiliar GPIO for which GPTRIP1 is selected
	GpioDataRegs.GPASET.bit.GPIO29 = 1;
	EDIS;

	PWM_Modules.N_modules = 2;
	PWM_Modules.PWM_Regs[0] = &EPwm1Regs;
	PWM_Modules.PWM_Regs[1] = &EPwm2Regs;

	DisablePWMOutputs();
	DisablePWM_TBCLK();
	InitPWM_MEP_SFO();

	InitPWMModule(PWM_Modules.PWM_Regs[0], PWM_FREQ, 0, MasterPWM, 0, NO_COMPLEMENTARY, PWM_DEAD_TIME);
	InitPWMModule(PWM_Modules.PWM_Regs[1], PWM_FREQ, 0, SlavePWM, 180, NO_COMPLEMENTARY, PWM_DEAD_TIME);

	InitPWMDAC(PWM_DAC_MODULE, PWM_DAC_FREQ);

	InitEPwm1Gpio();
	InitEPwm2Gpio();
	InitEPwm4Gpio();

	/* Initialization of GPIOs */

	EALLOW;

	GpioCtrlRegs.GPDMUX2.bit.GPIO126 = 0;
	GpioDataRegs.GPDCLEAR.bit.GPIO126 = 1;		// GPDI1: AC mains contactor status
	GpioCtrlRegs.GPDDIR.bit.GPIO126 = 0;


	GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 0;
	GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;		// GPDO1: AC mains contactor switch
	GpioCtrlRegs.GPCDIR.bit.GPIO67 = 1;

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

	Init_DMA_McBSP_nBuffers(1, DECIMATION_FACTOR, HRADC_SPI_CLK);

	Init_SPIMaster_McBSP(HRADC_SPI_CLK);
	Init_SPIMaster_Gpio();
	InitMcbspa20bit();

	HRADCs_Info.HRADC_boards[0] = &HRADC0_board;

	Config_HRADC_board(HRADCs_Info.HRADC_boards[0], TRANSDUCER_0_OUTPUT_TYPE, HEATER_DISABLE, RAILS_DISABLE);

	stop_DMA();
	DELAY_US(5);
	start_DMA();
	EnablePWM_TBCLK();
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
	 * 	      name: 	ISHARE_ERROR_CALCULATOR
	 * description: 	Current share error
	 *    DP class:     ELP_Error
	 *     	    +:		MtoC.NetSignals[0]
	 *     	    -:		NetSignals[12]
	 * 		   out:		NetSignals[10]
	 */

	//Init_ELP_Error(ISHARE_ERROR_CALCULATOR, &DP_Framework_MtoC.NetSignals[2], &DP_Framework_MtoC.NetSignals[3], &DP_Framework.NetSignals[10]);
	Init_ELP_Error(ISHARE_ERROR_CALCULATOR, &DP_Framework_MtoC.NetSignals[0], &DP_Framework.NetSignals[12], &DP_Framework.NetSignals[10]);

	/*
	 * 	      name: 	PI_DAWU_CONTROLLER_ISHARE
	 * description: 	Current share PI controller
	 *    DP class:     ELP_PI_dawu
	 *     	    in:		NetSignals[10]
	 * 		   out:		NetSignals[11]
	 */

	Init_ELP_PI_dawu(PI_DAWU_CONTROLLER_ISHARE, KP_ISHARE, KI_ISHARE, ISHARE_CONTROL_FREQ, PWM_MAX_SHARE_DUTY, -PWM_MAX_SHARE_DUTY, &DP_Framework.NetSignals[10], &DP_Framework.NetSignals[11]);

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
	Set_TimeSlicer(2, CONTROL_FREQ/ISHARE_CONTROL_FREQ);

	ResetControllers();
}

static void ResetControllers(void)
{
	IPC_CtoM_Msg.PSModule.IRef = 0.0;

	SetPWMDutyCycle_ChA(PWM_Modules.PWM_Regs[0], 0.0);
	SetPWMDutyCycle_ChA(PWM_Modules.PWM_Regs[1], 0.0);

	Reset_ELP_SRLim(SRLIM_ILOAD_REFERENCE);

	Reset_ELP_Error(ERROR_CALCULATOR);
	Reset_ELP_Error(ISHARE_ERROR_CALCULATOR);

	Reset_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD);
	Reset_ELP_PI_dawu(PI_DAWU_CONTROLLER_ISHARE);

	Reset_ELP_SRLim(SRLIM_SIGGEN_AMP);
	Reset_ELP_SRLim(SRLIM_SIGGEN_OFFSET);

	Reset_TimeSlicers();
}

/*
 * Initialization of application interruptions
 *
 * 		- PWM interruptions as main ISR for control loop (INT3)
 * 		- IPC interruptions (INT11)
 */

static void InitInterruptions(void)
{
	EALLOW;
	PieVectTable.EPWM1_INT =  &isr_ePWM_CTR_ZERO_1st;
	PieVectTable.EPWM2_INT =  &isr_ePWM_CTR_ZERO;
	EDIS;

	PieCtrlRegs.PIEIER3.bit.INTx1 = 1;  // ePWM1
	PieCtrlRegs.PIEIER3.bit.INTx2 = 1;  // ePWM2

	EnablePWMInterrupt(PWM_Modules.PWM_Regs[0]);
	EnablePWMInterrupt(PWM_Modules.PWM_Regs[1]);

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
static interrupt void isr_ePWM_CTR_ZERO(void)
{
	static Uint16 i, bypass_SRLim;
	static float temp0;

	//StartCpuTimer0();
	//SET_DEBUG_GPIO1;

	temp0 = 0.0;

	bypass_SRLim = USE_MODULE;

	for(i = 0; i < DECIMATION_FACTOR; i++)
	{
		temp0 += (float) *(HRADCs_Info.HRADC_boards[0]->SamplesBuffer++);
	}

	HRADCs_Info.HRADC_boards[0]->SamplesBuffer = buffers_HRADC.buffer_0;

	temp0 *= AverageFilter;

	temp0 -= *(HRADCs_Info.HRADC_boards[0]->offset);
	temp0 *= *(HRADCs_Info.HRADC_boards[0]->gain);

	DP_Framework.NetSignals[1] = temp0;

	if(fabs(temp0) > MAX_LOAD)
	{
		if(CHECK_INTERLOCK(LOAD_OVERCURRENT))
		{
			Set_HardInterlock(LOAD_OVERCURRENT);
		}
	}

	if(IPC_CtoM_Msg.PSModule.OnOff)
	{

		switch(IPC_CtoM_Msg.PSModule.OpMode)
		{
			case FastRef:
				bypass_SRLim = BYPASS_MODULE;
				break;

			case WfmRef:

				if(IPC_CtoM_Msg.WfmRef.SyncMode == OneShot)
				{
					RUN_TIMESLICE(0); /************************************************************/

						if(IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferK <= IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferEnd)
						{
							IPC_CtoM_Msg.PSModule.IRef = *(IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferK++) * IPC_CtoM_Msg.WfmRef.Gain + IPC_CtoM_Msg.WfmRef.Offset;
						}

					END_TIMESLICE(0); /************************************************************/
				}
				else if(IPC_CtoM_Msg.WfmRef.SyncMode == SampleBySample)
				{
					if(IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferK <= IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferEnd)
					{
						IPC_CtoM_Msg.PSModule.IRef = *(IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferK) * IPC_CtoM_Msg.WfmRef.Gain + IPC_CtoM_Msg.WfmRef.Offset;
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

			RUN_TIMESLICE(2); /************************************************************/

				DP_Framework.NetSignals[12] = DP_Framework.NetSignals[1] * 0.5;
				Run_ELP_Error(ISHARE_ERROR_CALCULATOR);
				Run_ELP_PI_dawu(PI_DAWU_CONTROLLER_ISHARE);

			END_TIMESLICE(2); /************************************************************/

			DP_Framework.DutySignals[0] = DP_Framework.NetSignals[9] - DP_Framework.NetSignals[11];
			DP_Framework.DutySignals[1] = DP_Framework.NetSignals[9] + DP_Framework.NetSignals[11];

			SATURATE(DP_Framework.DutySignals[0], PWM_MAX_DUTY, PWM_MIN_DUTY);
			SATURATE(DP_Framework.DutySignals[1], PWM_MAX_DUTY, PWM_MIN_DUTY);
		}

		SetPWMDutyCycle_ChA(PWM_Modules.PWM_Regs[0], DP_Framework.DutySignals[0]);
		SetPWMDutyCycle_ChA(PWM_Modules.PWM_Regs[1], DP_Framework.DutySignals[1]);

		SetPWMDutyCycle_ChA(PWM_DAC_MODULE, DP_Framework.NetSignals[1]); // delta(Net) = 0.05 mA => delta(DAC_OUT) = 0.05 mV
		SetPWMDutyCycle_ChB(PWM_DAC_MODULE, DP_Framework.DutySignals[0]);
	}

	RUN_TIMESLICE(1); /************************************************************/

		WriteBuffer(&IPC_CtoM_Msg.SamplesBuffer, DP_Framework.NetSignals[1]);

	END_TIMESLICE(1); /************************************************************/

	for(i = 0; i < PWM_Modules.N_modules; i++)
	{
		PWM_Modules.PWM_Regs[i]->ETCLR.bit.INT = 1;
	}

	//CLEAR_DEBUG_GPIO1;
	//StopCpuTimer0();
	//valorCounter = ReadCpuTimer0Counter();
	//ReloadCpuTimer0();

	PieCtrlRegs.PIEACK.all |= M_INT3;
}

static interrupt void isr_ePWM_CTR_ZERO_1st(void)
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
}

static interrupt void isr_SoftInterlock(void)
{
	CtoMIpcRegs.MTOCIPCACK.all = SOFT_INTERLOCK_MTOC;

	PS_turnOff();
	IPC_CtoM_Msg.PSModule.SoftInterlocks |= IPC_MtoC_Msg.PSModule.SoftInterlocks;

	PieCtrlRegs.PIEACK.all |= M_INT11;
}

static interrupt void isr_HardInterlock(void)
{
	CtoMIpcRegs.MTOCIPCACK.all = HARD_INTERLOCK_MTOC;

	PS_turnOff();
	IPC_CtoM_Msg.PSModule.HardInterlocks |= IPC_MtoC_Msg.PSModule.HardInterlocks;

	PieCtrlRegs.PIEACK.all |= M_INT11;
}

static void PS_turnOn(void)
{
	if(CHECK_INTERLOCKS)
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

		DELAY_US(50000);

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

	DELAY_US(20000);

	ResetPeripheralsDrivers();
	ResetControllers();
}

