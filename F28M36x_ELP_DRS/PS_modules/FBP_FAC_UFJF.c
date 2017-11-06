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
#include "FBP_FAC_UFJF.h"
#include "FBP_FAC_UFJF_Observer.h"
#include "FBP_FAC_UFJF_Controller.h"

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

void main_FBP_FAC_UFJF(void);

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
volatile tELP_MultiplyMatrix Observer, Controller;

void main_FBP_FAC_UFJF(void)
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

	Init_DMA_McBSP_nBuffers(N_HRADC, DECIMATION_FACTOR, HRADC_SPI_CLK);

    Init_SPIMaster_McBSP(HRADC_SPI_CLK);
    Init_SPIMaster_Gpio();
    InitMcbspa20bit();

    HRADCs_Info.HRADC_boards[0] = &HRADC0_board;
	HRADCs_Info.HRADC_boards[1] = &HRADC1_board;
	HRADCs_Info.HRADC_boards[2] = &HRADC2_board;

	Init_HRADC_Info(HRADCs_Info.HRADC_boards[0], 0, DECIMATION_FACTOR, buffers_HRADC.buffer_0, TRANSDUCER_0_GAIN, HRADC_0_R_BURDEN);
	Init_HRADC_Info(HRADCs_Info.HRADC_boards[1], 1, DECIMATION_FACTOR, buffers_HRADC.buffer_1, TRANSDUCER_1_GAIN, HRADC_1_R_BURDEN);
	Init_HRADC_Info(HRADCs_Info.HRADC_boards[2], 2, DECIMATION_FACTOR, buffers_HRADC.buffer_2, TRANSDUCER_2_GAIN, HRADC_2_R_BURDEN);

	Config_HRADC_board(HRADCs_Info.HRADC_boards[0], TRANSDUCER_0_OUTPUT_TYPE, HEATER_DISABLE, RAILS_DISABLE);
	Config_HRADC_board(HRADCs_Info.HRADC_boards[1], TRANSDUCER_1_OUTPUT_TYPE, HEATER_DISABLE, RAILS_DISABLE);
	Config_HRADC_board(HRADCs_Info.HRADC_boards[2], TRANSDUCER_2_OUTPUT_TYPE, HEATER_DISABLE, RAILS_DISABLE);

	AverageFilter = 1.0/((float) DECIMATION_FACTOR);

    Config_HRADC_SoC(HRADC_FREQ_SAMP);

    /* Initialization of PWM modules */

    EALLOW;
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO29 = 1;			// Auxiliar GPIO for which GPTRIP1 is selected
	GpioDataRegs.GPASET.bit.GPIO29 = 1;
	EDIS;

    PWM_Modules.N_modules = 8;

    PWM_MOD1_A = &EPwm7Regs;        // PS-1 Positive polarity switches
    PWM_MOD1_B = &EPwm8Regs;        // PS-1 Negative polarity switches

    PWM_MOD2_A = &EPwm5Regs;        // PS-2 Positive polarity switches
    PWM_MOD2_B = &EPwm6Regs;        // PS-2 Negative polarity switches

    PWM_MOD3_A = &EPwm3Regs;        // PS-3 Positive polarity switches
    PWM_MOD3_B = &EPwm4Regs;        // PS-3 Negative polarity switches

    PWM_MOD4_A = &EPwm1Regs;		// PS-4 Positive polarity switches
    PWM_MOD4_B = &EPwm2Regs;		// PS-4 Negative polarity switches

    DisablePWMOutputs();
    DisablePWM_TBCLK();
    InitPWM_MEP_SFO();

    // PS-4 PWM initialization
    InitPWMModule(PWM_MOD4_A, PWM_FREQ, 0, MasterPWM, 0, COMPLEMENTARY, PWM_DEAD_TIME);
    InitPWMModule(PWM_MOD4_B, PWM_FREQ, 1, SlavePWM, 180, COMPLEMENTARY, PWM_DEAD_TIME);

    // PS-3 PWM initialization
    InitPWMModule(PWM_MOD3_A, PWM_FREQ, 0, SlavePWM, 0, COMPLEMENTARY, PWM_DEAD_TIME);
    InitPWMModule(PWM_MOD3_B, PWM_FREQ, 3, SlavePWM, 180, COMPLEMENTARY, PWM_DEAD_TIME);

    // PS-2 PWM initialization
    InitPWMModule(PWM_MOD2_A, PWM_FREQ, 0, SlavePWM, 0, COMPLEMENTARY, PWM_DEAD_TIME);
    InitPWMModule(PWM_MOD2_B, PWM_FREQ, 5, SlavePWM, 180, COMPLEMENTARY, PWM_DEAD_TIME);

    // PS-1 PWM initialization
    InitPWMModule(PWM_MOD1_A, PWM_FREQ, 0, SlavePWM, 0, COMPLEMENTARY, PWM_DEAD_TIME);
    InitPWMModule(PWM_MOD1_B, PWM_FREQ, 7, SlavePWM, 180, COMPLEMENTARY, PWM_DEAD_TIME);

    //InitEPwm1Gpio();
    //InitEPwm2Gpio();
    //InitEPwm3Gpio();
    //InitEPwm4Gpio();
    //InitEPwm5Gpio();
    //InitEPwm6Gpio();
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

	/************************************/
	/* INITIALIZATION OF STATE OBSERVER */
	/************************************/

	/*
	 * 	      name: 	STATE_OBSERVER
	 * description: 	State observer
	 *    DP class:     ELP_MultiplyMatrix
	 *     	    in:		NetSignals[1..7]
	 * 		   out:		NetSignals[8..9]
	 */

	Init_ELP_MultiplyMatrix(STATE_OBSERVER, NUM_ROWS_OBSERVER, NUM_COLUMNS_OBSERVER, O, STATE_OBSERVER_IN, STATE_OBSERVER_OUT);

    /**************************************/
    /* INITIALIZATION OF STATE CONTROLLER */
    /**************************************/

    /*
     *        name:     STATE_CONTROLLER
     * description:     State controller
     *    DP class:     ELP_MultiplyMatrix
     *          in:     NetSignals[6..10]
     *         out:     DutySignals[0]
     */

    Init_ELP_MultiplyMatrix(STATE_CONTROLLER, NUM_ROWS_CONTROLLER, NUM_COLUMNS_CONTROLLER, Kp, STATE_CONTROLLER_IN, STATE_CONTROLLER_OUT);

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
	Set_TimeSlicer(TIMESLICER_WFMREF, CONTROL_FREQ/WFMREF_SAMPLING_FREQ);

	// 1: Time-slicer for SamplesBuffer
	Set_TimeSlicer(TIMESLICER_BUFFER, BUFFER_DECIMATION);

	ResetControllers();
}

static void ResetControllers(void)
{
    Uint16 i;

	SetPWMDutyCycle_HBridge(PWM_MOD1_A, 0.0);
	SetPWMDutyCycle_HBridge(PWM_MOD2_A, 0.0);
	SetPWMDutyCycle_HBridge(PWM_MOD3_A, 0.0);
	SetPWMDutyCycle_HBridge(PWM_MOD4_A, 0.0);

	Reset_ELP_MultiplyMatrix(STATE_OBSERVER);

#ifdef  FULL_OBSERVER
	XMOD1_0 = 0.0;
	XMOD1_1 = 0.0;
	XMOD1_2 = 0.0;
	XMOD1_3 = 0.0;
#endif

#ifdef REDUCED_OBSERVER
	for(i = 1; i < (NUM_ROWS_OBSERVER + NUM_COLUMNS_OBSERVER + 1); i++)
	{
	    DP_Framework.NetSignals[i] = 0.0;
	}
#endif

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
	//PieVectTable.EPWM2_INT =  &isr_ePWM_CTR_ZERO;
	EDIS;

	PieCtrlRegs.PIEIER3.bit.INTx1 = 1;  // ePWM1
	//PieCtrlRegs.PIEIER3.bit.INTx2 = 1;  // ePWM2

	EnablePWMInterrupt(PWM_MOD4_A);
	//EnablePWMInterrupt(PWM_Modules.PWM_Regs[1]);

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

	SET_DEBUG_GPIO1;

	temp0 = 0.0;
	temp1 = 0.0;
	temp2 = 0.0;
	temp3 = 0.0;

	temp0 = (float) *(HRADCs_Info.HRADC_boards[0]->SamplesBuffer++);
	temp1 = (float) *(HRADCs_Info.HRADC_boards[1]->SamplesBuffer++);
	temp2 = (float) *(HRADCs_Info.HRADC_boards[2]->SamplesBuffer++);
	//temp3 = (float) *(HRADCs_Info.HRADC_boards[3]->SamplesBuffer++);

	HRADCs_Info.HRADC_boards[0]->SamplesBuffer = buffers_HRADC.buffer_0;
	HRADCs_Info.HRADC_boards[1]->SamplesBuffer = buffers_HRADC.buffer_1;
	HRADCs_Info.HRADC_boards[2]->SamplesBuffer = buffers_HRADC.buffer_2;
	//HRADCs_Info.HRADC_boards[3]->SamplesBuffer = buffers_HRADC.buffer_3;

	temp0 *= AverageFilter;
	temp1 *= AverageFilter;
	temp2 *= AverageFilter;
	//temp3 *= AverageFilter;

	/*
	if((IPC_CtoM_Msg.PSModule.OpMode == WfmRef) && ((IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferK == IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferStart) || wfmSyncFlag == 1))
	{
		CLEAR_DEBUG_GPIO1;
		wfmSyncFlag = 0;
	}*/

	temp0 -= *(HRADCs_Info.HRADC_boards[0]->offset);
	temp0 *= *(HRADCs_Info.HRADC_boards[0]->gain);

	temp1 -= *(HRADCs_Info.HRADC_boards[1]->offset);
	temp1 *= *(HRADCs_Info.HRADC_boards[1]->gain);

	temp2 -= *(HRADCs_Info.HRADC_boards[2]->offset);
	temp2 *= *(HRADCs_Info.HRADC_boards[2]->gain);

	//temp3 -= *(HRADCs_Info.HRADC_boards[3]->offset);
	//temp3 *= *(HRADCs_Info.HRADC_boards[3]->gain);

	IMOD1 = temp0;
	IMOD2 = temp1;
	ILOAD = temp2;
	VLOAD_MOD1_DSP = VLOAD_MOD1_ARM;

	//if((fabs(temp0) > MAX_LOAD) || (fabs(temp1) > MAX_LOAD) || (fabs(temp2) > MAX_LOAD) || (fabs(temp3) > MAX_LOAD))

	if(fabs(IMOD1) > MAX_IMOD)
    {
        if(CHECK_INTERLOCK(OUT1_OVERCURRENT))
        {
            Set_HardInterlock(OUT1_OVERCURRENT);
        }
    }

	if(fabs(ILOAD) > MAX_LOAD)
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

                break;

			case SigGen:
                SignalGenerator.Run_ELP_SigGen(&SignalGenerator);
				break;

			default:
				break;
		}

		REFERENCE = IPC_CtoM_Msg.PSModule.IRef;

#ifdef REDUCED_OBSERVER
		Run_ELP_MatrixMultiplication(STATE_OBSERVER);
#endif

		if(IPC_CtoM_Msg.PSModule.OpenLoop)
		{
		    // For open loop, Iref value represents duty-cycle
		    DUTY_MOD1 = 0.01*REFERENCE;
			SATURATE(DUTY_MOD1, PWM_MAX_DUTY_OL, PWM_MIN_DUTY_OL);
		}

		else
		{
			SATURATE(REFERENCE, MAX_REF, MIN_REF);

			Run_ELP_MatrixMultiplication(STATE_CONTROLLER);

			SATURATE(DUTY_MOD1, PWM_MAX_DUTY, PWM_MIN_DUTY);
		}

#ifdef FULL_OBSERVER
		U1 = DUTY_MOD1;

		Run_ELP_MatrixMultiplication(STATE_OBSERVER);

        XMOD1_0 = XMOD1_0_FUT;
        XMOD1_1 = XMOD1_1_FUT;
        XMOD1_2 = XMOD1_2_FUT;
        XMOD1_3 = XMOD1_3_FUT;
#endif

#ifdef REDUCED_OBSERVER
        X_A_MOD1_0_PAST = X_A_MOD1_0;
        X_A_MOD1_1_PAST = X_A_MOD1_1;
        X_B_MOD1_0_PAST = X_B_MOD1_0;
        X_B_MOD1_1_PAST = X_B_MOD1_1;
        U_MOD1_PAST = DUTY_MOD1;
#endif

		SetPWMDutyCycle_HBridge(PWM_MOD1_A, DUTY_MOD1);
	}

	RUN_TIMESLICE(TIMESLICER_BUFFER); /************************************************************/

		WriteBuffer(&IPC_CtoM_Msg.SamplesBuffer, ILOAD);
		WriteBuffer(&IPC_CtoM_Msg.SamplesBuffer, DUTY_MOD1);
		WriteBuffer(&IPC_CtoM_Msg.SamplesBuffer, VLOAD_MOD1_DSP);

	END_TIMESLICE(TIMESLICER_BUFFER); /************************************************************/

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
		PIN_CLOSE_PS3_DCLINK_RELAY;
		PIN_CLOSE_PS4_DCLINK_RELAY;

		DELAY_US(500000);			// Wait 0.5 s

		EnablePWMOutputs();
	}
}

static void PS_turnOff(void)
{
	DisablePWMOutputs();

	PIN_OPEN_PS1_DCLINK_RELAY;
	PIN_OPEN_PS2_DCLINK_RELAY;
	PIN_OPEN_PS3_DCLINK_RELAY;
	PIN_OPEN_PS4_DCLINK_RELAY;

	IPC_CtoM_Msg.PSModule.OnOff = 0;
	IPC_CtoM_Msg.PSModule.OpenLoop = 1;

	ResetControllers();
}
