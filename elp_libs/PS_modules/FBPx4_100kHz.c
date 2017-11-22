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
 *		TODO: N�o executar PS_TurnOn e PS_TurnOff se fonte j� estiver ligada/desligada
 */

#include "F28M36x_ELP_DRS.h"
#include "FBPx4_100kHz.h"



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

void main_FBPx4_100kHz(void);

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


void main_FBPx4_100kHz(void)
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

	TerminatePeripheralsDrivers();
	TerminateInterruptions();
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

	Init_DMA_McBSP_nBuffers(n_PS, DECIMATION_FACTOR, HRADC_SPI_CLK);

    Init_SPIMaster_McBSP(HRADC_SPI_CLK);
    Init_SPIMaster_Gpio();
    InitMcbspa20bit();

    HRADCs_Info.HRADC_boards[0] = &HRADC0_board;
	HRADCs_Info.HRADC_boards[1] = &HRADC1_board;
	//HRADCs_Info.HRADC_boards[2] = &HRADC2_board;
	//HRADCs_Info.HRADC_boards[3] = &HRADC3_board;

	Init_HRADC_Info(HRADCs_Info.HRADC_boards[0], 0, DECIMATION_FACTOR, buffers_HRADC.buffer_0, TRANSDUCER_0_GAIN, HRADC_0_R_BURDEN);
	Init_HRADC_Info(HRADCs_Info.HRADC_boards[1], 1, DECIMATION_FACTOR, buffers_HRADC.buffer_1, TRANSDUCER_1_GAIN, HRADC_1_R_BURDEN);
	//Init_HRADC_Info(HRADCs_Info.HRADC_boards[2], 2, DECIMATION_FACTOR, buffers_HRADC.buffer_2, TRANSDUCER_2_GAIN, HRADC_2_R_BURDEN);
	//Init_HRADC_Info(HRADCs_Info.HRADC_boards[3], 3, DECIMATION_FACTOR, buffers_HRADC.buffer_3, TRANSDUCER_3_GAIN, HRADC_3_R_BURDEN);

	Config_HRADC_board(HRADCs_Info.HRADC_boards[0], TRANSDUCER_0_OUTPUT_TYPE, HEATER_DISABLE, RAILS_DISABLE);
	Config_HRADC_board(HRADCs_Info.HRADC_boards[1], TRANSDUCER_1_OUTPUT_TYPE, HEATER_DISABLE, RAILS_DISABLE);
	//Config_HRADC_board(HRADCs_Info.HRADC_boards[2], TRANSDUCER_2_OUTPUT_TYPE, HEATER_DISABLE, RAILS_DISABLE);
	//Config_HRADC_board(HRADCs_Info.HRADC_boards[3], TRANSDUCER_3_OUTPUT_TYPE, HEATER_DISABLE, RAILS_DISABLE);

	AverageFilter = 1.0/((float) DECIMATION_FACTOR);

    Config_HRADC_SoC(HRADC_FREQ_SAMP);

    /* Initialization of PWM modules */

    EALLOW;
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO29 = 1;			// Auxiliar GPIO for which GPTRIP1 is selected
	GpioDataRegs.GPASET.bit.GPIO29 = 1;
	EDIS;

    PWM_Modules.N_modules = 8;
    PWM_Modules.PWM_Regs[0] = &EPwm1Regs;		// PS-4 Positive polarity switches
    PWM_Modules.PWM_Regs[1] = &EPwm2Regs;		// PS-4 Negative polarity switches
    PWM_Modules.PWM_Regs[2] = &EPwm3Regs;		// PS-3 Positive polarity switches
    PWM_Modules.PWM_Regs[3] = &EPwm4Regs;		// PS-3 Negative polarity switches
	PWM_Modules.PWM_Regs[4] = &EPwm5Regs;		// PS-2 Positive polarity switches
	PWM_Modules.PWM_Regs[5] = &EPwm6Regs;		// PS-2 Negative polarity switches
	PWM_Modules.PWM_Regs[6] = &EPwm7Regs;		// PS-1 Positive polarity switches
	PWM_Modules.PWM_Regs[7] = &EPwm8Regs;		// PS-1 Negative polarity switches

    DisablePWMOutputs();
    DisablePWM_TBCLK();
    InitPWM_MEP_SFO();

    // PS-4 PWM initialization
    InitPWMModule(PWM_Modules.PWM_Regs[0], PWM_FREQ, 0, MasterPWM, 0, COMPLEMENTARY, PWM_DEAD_TIME);
    InitPWMModule(PWM_Modules.PWM_Regs[1], PWM_FREQ, 1, SlavePWM, 180, COMPLEMENTARY, PWM_DEAD_TIME);

    // PS-3 PWM initialization
    InitPWMModule(PWM_Modules.PWM_Regs[2], PWM_FREQ, 0, SlavePWM, 0, COMPLEMENTARY, PWM_DEAD_TIME);
    InitPWMModule(PWM_Modules.PWM_Regs[3], PWM_FREQ, 3, SlavePWM, 180, COMPLEMENTARY, PWM_DEAD_TIME);

    // PS-2 PWM initialization
    InitPWMModule(PWM_Modules.PWM_Regs[4], PWM_FREQ, 0, SlavePWM, 0, COMPLEMENTARY, PWM_DEAD_TIME);
    InitPWMModule(PWM_Modules.PWM_Regs[5], PWM_FREQ, 5, SlavePWM, 180, COMPLEMENTARY, PWM_DEAD_TIME);

    // PS-1 PWM initialization
    InitPWMModule(PWM_Modules.PWM_Regs[6], PWM_FREQ, 0, SlavePWM, 0, COMPLEMENTARY, PWM_DEAD_TIME);
    InitPWMModule(PWM_Modules.PWM_Regs[7], PWM_FREQ, 7, SlavePWM, 180, COMPLEMENTARY, PWM_DEAD_TIME);

    //InitEPwm1Gpio();
    //InitEPwm2Gpio();
    //InitEPwm3Gpio();
    //InitEPwm4Gpio();
    InitEPwm5Gpio();
    InitEPwm6Gpio();
    InitEPwm7Gpio();
    InitEPwm8Gpio();

	/* Initialization of GPIOs */

	EALLOW;

	GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 0;
	GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;		// GPDO1: PS3_OUTPUT_CTRL (FBP v4.0)
	GpioCtrlRegs.GPCDIR.bit.GPIO67 = 1;

	GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 0;
	GpioDataRegs.GPCCLEAR.bit.GPIO65 = 1;		// GPDO2: PS4_OUTPUT_CTRL (FBP v4.0)
	GpioCtrlRegs.GPCDIR.bit.GPIO65 = 1;

	GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 0;
	GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;		// GPDO3: PS2_OUTPUT_CTRL (FBP v4.0)
	GpioCtrlRegs.GPCDIR.bit.GPIO66 = 1;

	GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 0;
	GpioDataRegs.GPCCLEAR.bit.GPIO64 = 1;		// GPDO4: PS1_OUTPUT_CTRL (FBP v4.0)
	GpioCtrlRegs.GPCDIR.bit.GPIO64 = 1;

	INIT_DEBUG_GPIO1;

	EDIS;

	/* Initialization of timers */

    InitCpuTimers();
	ConfigCpuTimer(&CpuTimer0, C28_FREQ_MHZ, 1000000);
	CpuTimer0Regs.TCR.bit.TIE = 0;
}

static void InitControllers(void)
{
	DELAY_US(500000);

	/* Initialization of IPC module */
	InitIPC(&PS_turnOn, &PS_turnOff, &isr_SoftInterlock, &isr_HardInterlock);

	/* Initiaization of DP Framework */
	InitDP_Framework(&DP_Framework, &(IPC_CtoM_Msg.PSModule.IRef));

	/******************************************************************/
	/* INITIALIZATION OF LOAD CURRENT CONTROL LOOP FOR POWER SUPPLY 1 */
	/******************************************************************/

	/*
	 * 	      name: 	ERROR_CALCULATOR_PS1
	 * description: 	Load current reference error
	 *    DP class:     ELP_Error
	 *     	    +:		NetSignals[1]
	 *     	    -:		NetSignals[5]
	 * 		   out:		NetSignals[6]
	 */

	Init_ELP_Error(ERROR_CALCULATOR_PS1, &DP_Framework.NetSignals[1], &DP_Framework.NetSignals[5], &DP_Framework.NetSignals[6]);

	/*
	 * 	      name: 	PI_DAWU_CONTROLLER_ILOAD_PS1
	 * description: 	Load current PI controller
	 *    DP class:     ELP_PI_dawu
	 *     	    in:		NetSignals[6]
	 * 		   out:		DutySignals[0]
	 */

	//Init_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD_PS1, IPC_MtoC_Msg.DPModule.Coeffs[0], IPC_MtoC_Msg.DPModule.Coeffs[1], CONTROL_FREQ, PWM_MAX_DUTY, PWM_MIN_DUTY, &DP_Framework.NetSignals[6], &DP_Framework.DutySignals[0]);
	Init_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD_PS1, KP, KI, CONTROL_FREQ, PWM_MAX_DUTY, PWM_MIN_DUTY, &DP_Framework.NetSignals[6], &DP_Framework.DutySignals[0]);

	/******************************************************************/
	/* INITIALIZATION OF LOAD CURRENT CONTROL LOOP FOR POWER SUPPLY 2 */
	/******************************************************************/

	/*
	 * 	      name: 	ERROR_CALCULATOR_PS2
	 * description: 	Load current reference error
	 *    DP class:     ELP_Error
	 *     	    +:		NetSignals[2]
	 *     	    -:		NetSignals[7]
	 * 		   out:		NetSignals[8]
	 */

	Init_ELP_Error(ERROR_CALCULATOR_PS2, &DP_Framework.NetSignals[2], &DP_Framework.NetSignals[7], &DP_Framework.NetSignals[8]);

	/*
	 * 	      name: 	PI_DAWU_CONTROLLER_ILOAD_PS2
	 * description: 	Load current PI controller
	 *    DP class:     ELP_PI_dawu
	 *     	    in:		NetSignals[8]
	 * 		   out:		DutySignals[1]
	 */

	//Init_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD_PS2, IPC_MtoC_Msg.DPModule.Coeffs[0], IPC_MtoC_Msg.DPModule.Coeffs[1], CONTROL_FREQ, PWM_MAX_DUTY, PWM_MIN_DUTY, &DP_Framework.NetSignals[8], &DP_Framework.DutySignals[1]);
	Init_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD_PS2, KP, KI, CONTROL_FREQ, PWM_MAX_DUTY, PWM_MIN_DUTY, &DP_Framework.NetSignals[8], &DP_Framework.DutySignals[1]);

	/******************************************************************/
	/* INITIALIZATION OF LOAD CURRENT CONTROL LOOP FOR POWER SUPPLY 3 */
	/******************************************************************/

	/*
	 * 	      name: 	ERROR_CALCULATOR_PS3
	 * description: 	Load current reference error
	 *    DP class:     ELP_Error
	 *     	    +:		NetSignals[3]
	 *     	    -:		NetSignals[9]
	 * 		   out:		NetSignals[10]
	 */

	Init_ELP_Error(ERROR_CALCULATOR_PS3, &DP_Framework.NetSignals[3], &DP_Framework.NetSignals[9], &DP_Framework.NetSignals[10]);

	/*
	 * 	      name: 	PI_DAWU_CONTROLLER_ILOAD_PS3
	 * description: 	Load current PI controller
	 *    DP class:     ELP_PI_dawu
	 *     	    in:		NetSignals[10]
	 * 		   out:		DutySignals[2]
	 */

	//Init_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD_PS3, IPC_MtoC_Msg.DPModule.Coeffs[0], IPC_MtoC_Msg.DPModule.Coeffs[1], CONTROL_FREQ, PWM_MAX_DUTY, PWM_MIN_DUTY, &DP_Framework.NetSignals[10], &DP_Framework.DutySignals[2]);
	Init_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD_PS3, KP, KI, CONTROL_FREQ, PWM_MAX_DUTY, PWM_MIN_DUTY, &DP_Framework.NetSignals[10], &DP_Framework.DutySignals[2]);

	/******************************************************************/
	/* INITIALIZATION OF LOAD CURRENT CONTROL LOOP FOR POWER SUPPLY 4 */
	/******************************************************************/

	/*
	 * 	      name: 	ERROR_CALCULATOR_PS4
	 * description: 	Load current reference error
	 *    DP class:     ELP_Error
	 *     	    +:		NetSignals[4]
	 *     	    -:		NetSignals[11]
	 * 		   out:		NetSignals[12]
	 */

	Init_ELP_Error(ERROR_CALCULATOR_PS4, &DP_Framework.NetSignals[4], &DP_Framework.NetSignals[11], &DP_Framework.NetSignals[12]);

	/*
	 * 	      name: 	PI_DAWU_CONTROLLER_ILOAD_PS4
	 * description: 	Load current PI controller
	 *    DP class:     ELP_PI_dawu
	 *     	    in:		NetSignals[12]
	 * 		   out:		DutySignals[3]
	 */

	//Init_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD_PS4, IPC_MtoC_Msg.DPModule.Coeffs[0], IPC_MtoC_Msg.DPModule.Coeffs[1], CONTROL_FREQ, PWM_MAX_DUTY, PWM_MIN_DUTY, &DP_Framework.NetSignals[12], &DP_Framework.DutySignals[3]);
	Init_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD_PS4, KP, KI, CONTROL_FREQ, PWM_MAX_DUTY, PWM_MIN_DUTY, &DP_Framework.NetSignals[12], &DP_Framework.DutySignals[3]);

	/*********************************************/
	/* INITIALIZATION OF SIGNAL GENERATOR MODULE */
	/*********************************************/

	/*
	 * 	      name: 	SignalGenerator
	 * description: 	Signal generator module
	 * 		   out:		DP_Framework.Ref
	 */

	Disable_ELP_SigGen(&SignalGenerator);
	Init_ELP_SigGen(&SignalGenerator, Sine, 0.0, 0.0, 0.0, CONTROL_FREQ, &IPC_MtoC_Msg.SigGen.Freq, IPC_MtoC_Msg.SigGen.Amplitude, &IPC_MtoC_Msg.SigGen.Offset, &IPC_MtoC_Msg.SigGen.Aux, DP_Framework.Ref);

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
	SetPWMDutyCycle_HBridge(PWM_Modules.PWM_Regs[2], 0.0);
	SetPWMDutyCycle_HBridge(PWM_Modules.PWM_Regs[4], 0.0);
	SetPWMDutyCycle_HBridge(PWM_Modules.PWM_Regs[6], 0.0);

	Reset_ELP_Error(ERROR_CALCULATOR_PS1);
	Reset_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD_PS1);

	Reset_ELP_Error(ERROR_CALCULATOR_PS2);
	Reset_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD_PS2);

	Reset_ELP_Error(ERROR_CALCULATOR_PS3);
	Reset_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD_PS3);

	Reset_ELP_Error(ERROR_CALCULATOR_PS4);
	Reset_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD_PS4);

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
	static Uint16 i;
	static float temp0, temp1, temp2, temp3;

	//StartCpuTimer0();
	SET_DEBUG_GPIO1;

	//PIN_CLOSE_PS4_DCLINK_RELAY;

	temp0 = 0.0;
	temp1 = 0.0;
	temp2 = 0.0;
	temp3 = 0.0;

	for(i = 0; i < DECIMATION_FACTOR; i++)
	{
		temp0 = (float) *(HRADCs_Info.HRADC_boards[0]->SamplesBuffer++);
		temp1 = (float) *(HRADCs_Info.HRADC_boards[1]->SamplesBuffer++);
		temp2 = (float) *(HRADCs_Info.HRADC_boards[2]->SamplesBuffer++);
		temp3 = (float) *(HRADCs_Info.HRADC_boards[3]->SamplesBuffer++);
	}

	HRADCs_Info.HRADC_boards[0]->SamplesBuffer = buffers_HRADC.buffer_0;
	HRADCs_Info.HRADC_boards[1]->SamplesBuffer = buffers_HRADC.buffer_1;
	HRADCs_Info.HRADC_boards[2]->SamplesBuffer = buffers_HRADC.buffer_2;
	HRADCs_Info.HRADC_boards[3]->SamplesBuffer = buffers_HRADC.buffer_3;

	temp0 *= AverageFilter;
	temp1 *= AverageFilter;
	temp2 *= AverageFilter;
	temp3 *= AverageFilter;

	if((IPC_CtoM_Msg.PSModule.OpMode == WfmRef) && ((IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferK == IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferStart) || wfmSyncFlag == 1))
	{
		CLEAR_DEBUG_GPIO1;
		wfmSyncFlag = 0;
	}

	temp0 = (float) *(HRADCs_Info.HRADC_boards[0]->SamplesBuffer);
	temp1 = (float) *(HRADCs_Info.HRADC_boards[1]->SamplesBuffer);
	temp2 = (float) *(HRADCs_Info.HRADC_boards[2]->SamplesBuffer);
	temp3 = (float) *(HRADCs_Info.HRADC_boards[3]->SamplesBuffer);

	temp0 -= *(HRADCs_Info.HRADC_boards[0]->offset);
	temp0 *= *(HRADCs_Info.HRADC_boards[0]->gain);

	temp1 -= *(HRADCs_Info.HRADC_boards[1]->offset);
	temp1 *= *(HRADCs_Info.HRADC_boards[1]->gain);

	temp2 -= *(HRADCs_Info.HRADC_boards[2]->offset);
	temp2 *= *(HRADCs_Info.HRADC_boards[2]->gain);

	temp3 -= *(HRADCs_Info.HRADC_boards[3]->offset);
	temp3 *= *(HRADCs_Info.HRADC_boards[3]->gain);

	DP_Framework.NetSignals[5] = temp0;
	DP_Framework.NetSignals[7] = temp1;
	DP_Framework.NetSignals[9] = temp2;
	DP_Framework.NetSignals[11] = temp3;

	//if((fabs(temp0) > MAX_LOAD) || (fabs(temp1) > MAX_LOAD) || (fabs(temp2) > MAX_LOAD) || (fabs(temp3) > MAX_LOAD))
	if((fabs(temp0) > MAX_LOAD) || (fabs(temp1) > MAX_LOAD))
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
			case SlowRef:
				/*DP_Framework.NetSignals[0] = IPC_CtoM_Msg.PSModule.IRef;
				DP_Framework.NetSignals[1] = DP_Framework.NetSignals[0];
				DP_Framework.NetSignals[2] = DP_Framework.NetSignals[0];
				DP_Framework.NetSignals[3] = DP_Framework.NetSignals[0];
				DP_Framework.NetSignals[4] = DP_Framework.NetSignals[0];*/

				break;

			case WfmRef:
				if(INDEPENDENT_SETPOINT_TEST)
				{
					i = (Uint16) IPC_CtoM_Msg.WfmRef.Gain;
					SATURATE(i,4,1);

					IPC_CtoM_Msg.PSModule.IRef = IPC_CtoM_Msg.WfmRef.Offset;
					DP_Framework.NetSignals[0] = IPC_CtoM_Msg.WfmRef.Offset;
					DP_Framework.NetSignals[i] = IPC_CtoM_Msg.WfmRef.Offset;
				}
				else
				{
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

					DP_Framework.NetSignals[0] = IPC_CtoM_Msg.PSModule.IRef;
					DP_Framework.NetSignals[1] = DP_Framework.NetSignals[0];
					DP_Framework.NetSignals[2] = DP_Framework.NetSignals[0];
					DP_Framework.NetSignals[3] = DP_Framework.NetSignals[0];
					DP_Framework.NetSignals[4] = DP_Framework.NetSignals[0];
				}

				break;

			case SigGen:

				if(CROSSTALK_TEST)
				{
					DP_Framework.NetSignals[0] = IPC_MtoC_Msg.SigGen.Amplitude[0] + IPC_MtoC_Msg.SigGen.Offset;

					for(i = 1; i < 5; i++)
					{
						if(i != ((Uint16) IPC_MtoC_Msg.SigGen.Freq))
						{
							DP_Framework.NetSignals[i] = DP_Framework.NetSignals[0];
						}
					}
				}

				else
				{
					//Run_ELP_SRLim(SRLIM_SIGGEN_AMP, USE_MODULE);
					//Run_ELP_SRLim(SRLIM_SIGGEN_OFFSET, USE_MODULE);
					SignalGenerator.Run_ELP_SigGen(&SignalGenerator);
					DP_Framework.NetSignals[0] = IPC_CtoM_Msg.PSModule.IRef;
					DP_Framework.NetSignals[1] = DP_Framework.NetSignals[0];
					DP_Framework.NetSignals[2] = DP_Framework.NetSignals[0];
					DP_Framework.NetSignals[3] = DP_Framework.NetSignals[0];
					DP_Framework.NetSignals[4] = DP_Framework.NetSignals[0];
				}

				break;

			default:
				break;
		}

		if(IPC_CtoM_Msg.PSModule.OpenLoop)
		{
			DP_Framework.DutySignals[0] = 0.01*DP_Framework.NetSignals[1];				// For open loop, Iref value represents duty-cycle
			DP_Framework.DutySignals[1] = 0.01*DP_Framework.NetSignals[2];				// in percentage (0 - 100 A => 0 - 100 %)
			DP_Framework.DutySignals[2] = 0.01*DP_Framework.NetSignals[3];
			DP_Framework.DutySignals[3] = 0.01*DP_Framework.NetSignals[4];

			SATURATE(DP_Framework.DutySignals[0], PWM_MAX_DUTY_OL, PWM_MIN_DUTY_OL);
			SATURATE(DP_Framework.DutySignals[1], PWM_MAX_DUTY_OL, PWM_MIN_DUTY_OL);
			SATURATE(DP_Framework.DutySignals[2], PWM_MAX_DUTY_OL, PWM_MIN_DUTY_OL);
			SATURATE(DP_Framework.DutySignals[3], PWM_MAX_DUTY_OL, PWM_MIN_DUTY_OL);
		}

		else
		{
			SATURATE(DP_Framework.NetSignals[1], MAX_REF, MIN_REF);
			SATURATE(DP_Framework.NetSignals[2], MAX_REF, MIN_REF);
			SATURATE(DP_Framework.NetSignals[3], MAX_REF, MIN_REF);
			SATURATE(DP_Framework.NetSignals[4], MAX_REF, MIN_REF);

			Run_ELP_Error(ERROR_CALCULATOR_PS1);
			Run_ELP_Error(ERROR_CALCULATOR_PS2);
			Run_ELP_Error(ERROR_CALCULATOR_PS3);
			Run_ELP_Error(ERROR_CALCULATOR_PS4);

			Run_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD_PS1);
			Run_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD_PS2);
			Run_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD_PS3);
			Run_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD_PS4);

			SATURATE(DP_Framework.DutySignals[0], PWM_MAX_DUTY, PWM_MIN_DUTY);
			SATURATE(DP_Framework.DutySignals[1], PWM_MAX_DUTY, PWM_MIN_DUTY);
			SATURATE(DP_Framework.DutySignals[2], PWM_MAX_DUTY, PWM_MIN_DUTY);
			SATURATE(DP_Framework.DutySignals[3], PWM_MAX_DUTY, PWM_MIN_DUTY);
		}

		//SetPWMDutyCycle_HBridge(PWM_Modules.PWM_Regs[0],DP_Framework.DutySignals[3]);
		//SetPWMDutyCycle_HBridge(PWM_Modules.PWM_Regs[2],DP_Framework.DutySignals[2]);
		SetPWMDutyCycle_HBridge(PWM_Modules.PWM_Regs[4],DP_Framework.DutySignals[1]);
		SetPWMDutyCycle_HBridge(PWM_Modules.PWM_Regs[6],DP_Framework.DutySignals[0]);
	}

	RUN_TIMESLICE(1); /************************************************************/

		WriteBuffer(&IPC_CtoM_Msg.SamplesBuffer, DP_Framework.NetSignals[5]);
		WriteBuffer(&IPC_CtoM_Msg.SamplesBuffer, DP_Framework.NetSignals[7]);
		//WriteBuffer(&IPC_CtoM_Msg.SamplesBuffer, DP_Framework.NetSignals[9]);
		//WriteBuffer(&IPC_CtoM_Msg.SamplesBuffer, DP_Framework.NetSignals[11]);

	END_TIMESLICE(1); /************************************************************/

/*	StopCpuTimer0();
	valorCounter = ReadCpuTimer0Counter();
	ReloadCpuTimer0();*/

	CLEAR_DEBUG_GPIO1;

	for(i = 0; i < PWM_Modules.N_modules; i++)
	{
		PWM_Modules.PWM_Regs[i]->ETCLR.bit.INT = 1;
	}

	//PIN_OPEN_PS4_DCLINK_RELAY;

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

	PieCtrlRegs.PIEACK.all |= M_INT11;
}

static void PS_turnOn(void)
{
	if(!IPC_CtoM_Msg.PSModule.HardInterlocks)
	{
		ResetControllers();

		IPC_CtoM_Msg.PSModule.IRef = 0.0;
		IPC_CtoM_Msg.PSModule.OpenLoop = 1;
		IPC_CtoM_Msg.PSModule.OnOff = 1;

		PIN_CLOSE_PS1_DCLINK_RELAY;
		PIN_CLOSE_PS2_DCLINK_RELAY;
		//PIN_CLOSE_PS3_DCLINK_RELAY;
		//PIN_CLOSE_PS4_DCLINK_RELAY;

		DELAY_US(500000);			// Wait 0.5 s

		EnablePWMOutputs();
	}
}

static void PS_turnOff(void)
{
	DisablePWMOutputs();

	PIN_OPEN_PS1_DCLINK_RELAY;
	PIN_OPEN_PS2_DCLINK_RELAY;
	//PIN_OPEN_PS3_DCLINK_RELAY;
	//PIN_OPEN_PS4_DCLINK_RELAY;

	IPC_CtoM_Msg.PSModule.OnOff = 0;
	IPC_CtoM_Msg.PSModule.OpenLoop = 1;

	ResetControllers();
}
