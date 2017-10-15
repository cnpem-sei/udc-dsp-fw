/*
 * 		FILE: 		main_FBP_100kHz.c
 * 		PROJECT: 	DRS v2.0
 * 		CREATION:
 * 		MODIFIED:	01/28/2016
 *
 * 		AUTHOR: 	Gabriel O. B.  (LNLS/ELP)
 *
 * 		DESCRIPTION:	Firmware for control of FBP v2.0.
 * 						Features:
 * 							- freqPWM = 50 kHz
 * 							- freqCtrl = 100 kHz
 * 							- Current control: Dynamic anti-windup PI
 * 							- Limited reference slew-rate
 * 							- Feedback filtered with oversampled moving
 * 							  average filter
 *
 *		TODO: Não executar PS_TurnOn e PS_TurnOff se fonte já estiver ligada/desligada
 */

#include "F28M36x_ELP_DRS.h"
#include "FBP_100kHz.h"

/*
 * DP modules mnemonics
 */

#define SRLIM_ILOAD_REFERENCE 		&DP_Framework.DPlibrary.ELP_SRLim[0]
#define ERROR_CALCULATOR			&DP_Framework.DPlibrary.ELP_Error[0]
#define	PI_DAWU_CONTROLLER_ILOAD	&DP_Framework.DPlibrary.ELP_PI_dawu[0]

#define	SRLIM_SIGGEN_AMP			&DP_Framework.DPlibrary.ELP_SRLim[1]
#define	SRLIM_SIGGEN_OFFSET			&DP_Framework.DPlibrary.ELP_SRLim[2]

/*
 * Digital IO's defines
 */

#define PIN_OPEN_DCLINK_RELAY		GpioDataRegs.GPCCLEAR.bit.GPIO65 = 1;
#define PIN_CLOSE_DCLINK_RELAY		GpioDataRegs.GPCSET.bit.GPIO65 = 1;

#define PIN_OPEN_OUTPUT_RELAY		GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;
#define PIN_CLOSE_OUTPUT_RELAY		GpioDataRegs.GPCSET.bit.GPIO67 = 1;

/*
 * Prototype statements for functions found within this file.
 */

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

void main_FBP_100kHz(void);

static void PS_turnOn(void);
static void PS_turnOff(void);

static void Set_SoftInterlock(Uint32 itlk);
static void Set_HardInterlock(Uint32 itlk);
static interrupt void isr_SoftInterlock(void);
static interrupt void isr_HardInterlock(void);

static void InitPeripheralsDrivers(void);
static void TerminatePeripheralsDrivers(void);

static void InitControllers(void);
static void ResetControllers(void);

static void InitInterruptions(void);
static void TerminateInterruptions(void);

volatile static Uint32 valorCounter;


void main_FBP_100kHz(void)
{
	InitPeripheralsDrivers();

	InitControllers();

	InitInterruptions();

	stop_DMA();
	DELAY_US(5);
	start_DMA();
	EnablePWM_TBCLK();

	//while(IPC_MtoC_Msg.PSModule.Model == FBP_100kHz)
	while(1)
	{
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

	Init_DMA_McBSP_nBuffers(1, DECIMATION_FACTOR, HRADC_SPI_CLK);

    Init_SPIMaster_McBSP(HRADC_SPI_CLK);
    Init_SPIMaster_Gpio();
    InitMcbspa20bit();

    HRADCs_Info.HRADC_boards[0] = &HRADC0_board;
    Init_HRADC_Info(HRADCs_Info.HRADC_boards[0], 0, DECIMATION_FACTOR, &buffers_HRADC.buffer_0[0], TRANSDUCER_0_GAIN, HRADC_0_R_BURDEN);
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

    InitPWMModule(PWM_Modules.PWM_Regs[0], PWM_FREQ, 0, MasterPWM, 0, COMPLEMENTARY, PWM_DEAD_TIME);
    InitPWMModule(PWM_Modules.PWM_Regs[1], PWM_FREQ, 1, SlavePWM, 180, COMPLEMENTARY, PWM_DEAD_TIME);

    InitPWMDAC(PWM_DAC_MODULE, PWM_DAC_FREQ);

    InitEPwm1Gpio();
    InitEPwm2Gpio();
	InitEPwm4Gpio();

	/* Initialization of GPIOs */

	EALLOW;
	GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 0;
	GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;		// GPDO1: Output Relay (FBP v2.0)
	GpioCtrlRegs.GPCDIR.bit.GPIO67 = 1;

	GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 0;
	GpioDataRegs.GPCCLEAR.bit.GPIO65 = 1;		// GPDO2: DC-Link Relay (FBP v2.0)
	GpioCtrlRegs.GPCDIR.bit.GPIO65 = 1;

	INIT_DEBUG_GPIO1;

	EDIS;

	/* Initialization of timers */

    InitCpuTimers();
	ConfigCpuTimer(&CpuTimer0, C28_FREQ_MHZ, 1000000);
	CpuTimer0Regs.TCR.bit.TIE = 0;
}

static void TerminatePeripheralsDrivers(void)
{

}

static void InitControllers(void)
{
	DELAY_US(500000);

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
	 * 		   out:		DutySignals[0]
	 */

	Init_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD, IPC_MtoC_Msg.DPModule.Coeffs[0], IPC_MtoC_Msg.DPModule.Coeffs[1], CONTROL_FREQ, PWM_MAX_DUTY, PWM_MIN_DUTY, &DP_Framework.NetSignals[8], &DP_Framework.DutySignals[0]);

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

	ResetControllers();
}

static void ResetControllers(void)
{
	SetPWMDutyCycle_HBridge(PWM_Modules.PWM_Regs[0], 0.0);

	SetPWMDutyCycle_ChA(PWM_DAC_MODULE, 0.0);
	SetPWMDutyCycle_ChB(PWM_DAC_MODULE, 0.0);

	Reset_ELP_SRLim(SRLIM_ILOAD_REFERENCE);
	Reset_ELP_Error(ERROR_CALCULATOR);
	Reset_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD);

	Reset_ELP_SRLim(SRLIM_SIGGEN_AMP);
	Reset_ELP_SRLim(SRLIM_SIGGEN_OFFSET);

	Reset_ELP_SigGen(&SignalGenerator);

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

static void TerminateInterruptions(void)
{
	/* Disable global interrupts (EINT) */
	DINT;
	DRTM;

	/* Clear enables */
	IER = 0;

	PieCtrlRegs.PIEIER3.bit.INTx1 = 0;  // ePWM1
	PieCtrlRegs.PIEIER3.bit.INTx2 = 0;  // ePWM2

	DisablePWMInterrupt(PWM_Modules.PWM_Regs[0]);
	DisablePWMInterrupt(PWM_Modules.PWM_Regs[1]);

	/* Clear flags */
	PieCtrlRegs.PIEACK.all |= M_INT1 | M_INT3 | M_INT11;
}


//*****************************************************************************
// Esvazia buffer FIFO com valores amostrados e recebidos via SPI
//*****************************************************************************
static interrupt void isr_ePWM_CTR_ZERO(void)
{
	static Uint16 i, bypass_SRLim;
	static float temp0;

	temp0 = 0.0;
	bypass_SRLim = USE_MODULE;

	SET_DEBUG_GPIO1;
	StartCpuTimer0();

	while(!McbspaRegs.SPCR1.bit.RRDY){}

	for(i = 0; i < DECIMATION_FACTOR; i++)
	{
		temp0 += (float) *(HRADCs_Info.HRADC_boards[0]->SamplesBuffer++);
	}

	HRADCs_Info.HRADC_boards[0]->SamplesBuffer = buffers_HRADC.buffer_0;

	temp0 *= AverageFilter;
	temp0 -= *(HRADCs_Info.HRADC_boards[0]->offset);
	temp0 *= *(HRADCs_Info.HRADC_boards[0]->gain);

	temp0 *= HRADC_0_GAIN_ERROR;
	temp0 += HRADC_0_OFFSET_ERROR;

	DP_Framework.NetSignals[1] = temp0;

	if(fabs(temp0) > MAX_LOAD)
	{
		if(CHECK_INTERLOCK(LOAD_OVERCURRENT))
		{
			Set_HardInterlock(LOAD_OVERCURRENT);
		}
	}

	else if(IPC_CtoM_Msg.PSModule.OnOff)
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

		Run_ELP_SRLim(SRLIM_ILOAD_REFERENCE,bypass_SRLim);

		if(IPC_CtoM_Msg.PSModule.OpenLoop)
		{
			DP_Framework.DutySignals[0] = 0.01*DP_Framework.NetSignals[0];				// For open loop, Iref value represents duty-cycle
			SATURATE(DP_Framework.DutySignals[0], PWM_MAX_DUTY_OL, PWM_MIN_DUTY_OL);	// in percentage (0 - 100 A => 0 - 100 %)
		}
		else
		{
			SATURATE(DP_Framework.NetSignals[0], MAX_REF, MIN_REF);
			Run_ELP_Error(ERROR_CALCULATOR);
			Run_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD);
			SATURATE(DP_Framework.DutySignals[0], PWM_MAX_DUTY, PWM_MIN_DUTY);
		}

		SetPWMDutyCycle_HBridge(PWM_Modules.PWM_Regs[0],DP_Framework.DutySignals[0]);
	}

	RUN_TIMESLICE(1); /************************************************************/

		WriteBuffer(&IPC_CtoM_Msg.SamplesBuffer, DP_Framework.NetSignals[1]);

	END_TIMESLICE(1); /************************************************************/

	StopCpuTimer0();
	valorCounter = ReadCpuTimer0Counter();
	ReloadCpuTimer0();

	CLEAR_DEBUG_GPIO1;

	for(i = 0; i < PWM_Modules.N_modules; i++)
	{
		PWM_Modules.PWM_Regs[i]->ETCLR.bit.INT = 1;
	}

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
}

static interrupt void isr_SoftInterlock(void)
{
	CtoMIpcRegs.MTOCIPCACK.all = SOFT_INTERLOCK_MTOC;

	PS_turnOff();
	IPC_CtoM_Msg.PSModule.SoftInterlocks |= EXTERNAL_INTERLOCK;
	//SendIpcFlag(SOFT_INTERLOCK_CTOM);

	PieCtrlRegs.PIEACK.all |= M_INT11;
}

static interrupt void isr_HardInterlock(void)
{
	CtoMIpcRegs.MTOCIPCACK.all = HARD_INTERLOCK_MTOC;

	PS_turnOff();
	IPC_CtoM_Msg.PSModule.HardInterlocks |= EXTERNAL_INTERLOCK;
	//SendIpcFlag(HARD_INTERLOCK_CTOM);

	PieCtrlRegs.PIEACK.all |= M_INT11;
}

static void PS_turnOn(void)
{
	if(!IPC_CtoM_Msg.PSModule.HardInterlocks)
	{
		ResetControllers();

		IPC_CtoM_Msg.PSModule.IRef = 0.0;
		IPC_CtoM_Msg.PSModule.OpenLoop = 0;
		IPC_CtoM_Msg.PSModule.OnOff = 1;

		PIN_CLOSE_DCLINK_RELAY;
		DELAY_US(500000);			// Wait 0.5 s
		PIN_CLOSE_OUTPUT_RELAY;
		DELAY_US(500000);			// Wait 0.5 s

		EnablePWMOutputs();
	}
}

static void PS_turnOff(void)
{
	DisablePWMOutputs();

	PIN_OPEN_DCLINK_RELAY;
	PIN_OPEN_OUTPUT_RELAY;

	IPC_CtoM_Msg.PSModule.OnOff = 0;
	IPC_CtoM_Msg.PSModule.OpenLoop = 1;

	ResetControllers();
}
