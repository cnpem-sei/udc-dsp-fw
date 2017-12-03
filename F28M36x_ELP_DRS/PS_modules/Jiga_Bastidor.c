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
#include "Jiga_Bastidor.h"


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

void main_Jiga_Bastidor(void);

static void PS_turnOn(Uint16 ps_modules);
static void PS_turnOff(Uint16 ps_modules);

static void Set_SoftInterlock(Uint32 itlk, Uint16 ps_modules);
static void Set_HardInterlock(Uint32 itlk, Uint16 ps_modules);
static interrupt void isr_SoftInterlock(void);
static interrupt void isr_HardInterlock(void);

static void InitPeripheralsDrivers(void);
static void TerminatePeripheralsDrivers(void);

static void InitControllers(void);
static void ResetControllers(Uint16 ps_modules);

static void InitInterruptions(void);
static void TerminateInterruptions(void);

volatile static Uint16 nFBPs;
volatile static Uint32 valorCounter;

void main_Jiga_Bastidor(void)
{
    Init_SPIMaster_McBSP(HRADC_SPI_CLK);
    Init_SPIMaster_Gpio();
    InitMcbspa20bit();

    DELAY_US(500000);

    SendIpcFlag(ENABLE_HRADC_BOARDS);

    DELAY_US(500000);

    while( (IPC_MtoC_Msg.HRADCConfig.nHRADC < 1) || (IPC_MtoC_Msg.HRADCConfig.nHRADC > 4) ){}

    nFBPs = IPC_MtoC_Msg.HRADCConfig.nHRADC;

	InitPeripheralsDrivers();
	InitControllers();
	InitInterruptions();

	stop_DMA();
	DELAY_US(5);
	start_DMA();
	EnablePWM_TBCLK();

	while(1)
	{
		if(nFBPs >= 1)
		{
			if(fabs(PS1_DCLINK_VOLTAGE) > MAX_DCLINK)
			{
				if(CHECK_INTERLOCK(PS1_DCLINK_OVERVOLTAGE))
				{
					Set_HardInterlock(PS1_DCLINK_OVERVOLTAGE,PS1_ID);
				}
			}

			if(fabs(PS1_DCLINK_VOLTAGE) < MIN_DCLINK)
			{
				if(CHECK_INTERLOCK(PS1_DCLINK_UNDERVOLTAGE))
				{
					Set_HardInterlock(PS1_DCLINK_UNDERVOLTAGE,PS1_ID);
				}
			}

			if(fabs(PS1_LOAD_VOLTAGE) > MAX_VLOAD)
			{
				if(CHECK_INTERLOCK(PS1_LOAD_OVERVOLTAGE))
				{
					Set_SoftInterlock(PS1_LOAD_OVERVOLTAGE,PS1_ID);
				}
			}

			if(fabs(PS1_TEMPERATURE) > MAX_TEMP)
			{
				if(CHECK_INTERLOCK(PS1_OVERTEMP))
				{
					//Set_HardInterlock(PS1_OVERTEMP,PS1_ID);
				    DP_Framework.NetSignals[20] = PS1_TEMPERATURE;
				    DP_Framework.NetSignals[21] = 1.0;
				}
			}

			if(CHECK_INTERLOCK(PS1_FUSE_FAIL) && !PIN_STATUS_PS1_FUSE)
			{
				Set_HardInterlock(PS1_FUSE_FAIL,PS1_ID);
			}

			if(CHECK_INTERLOCK(PS1_DRIVER_FAIL) && !PIN_STATUS_PS1_DRIVER_ERROR)
			{
				Set_HardInterlock(PS1_DRIVER_FAIL,PS1_ID);
			}

			if((IPC_CtoM_Msg.PSModule.OnOff & PS1_ID) && !PIN_STATUS_PS1_DCLINK_RELAY)
			{
				if(CHECK_INTERLOCK(PS1_DCLINK_RELAY_FAIL))
				{
					Set_HardInterlock(PS1_DCLINK_RELAY_FAIL,PS1_ID);
				}
			}

			if(nFBPs >= 2)
			{
				if(fabs(PS2_DCLINK_VOLTAGE) > MAX_DCLINK)
				{
					if(CHECK_INTERLOCK(PS2_DCLINK_OVERVOLTAGE))
					{
						Set_HardInterlock(PS2_DCLINK_OVERVOLTAGE,PS2_ID);
					}
				}

				if(fabs(PS2_DCLINK_VOLTAGE) < MIN_DCLINK)
				{
					if(CHECK_INTERLOCK(PS2_DCLINK_UNDERVOLTAGE))
					{
						Set_HardInterlock(PS2_DCLINK_UNDERVOLTAGE,PS2_ID);
					}
				}

				if(fabs(PS2_LOAD_VOLTAGE) > MAX_VLOAD)
				{
					if(CHECK_INTERLOCK(PS2_LOAD_OVERVOLTAGE))
					{
						Set_SoftInterlock(PS2_LOAD_OVERVOLTAGE,PS2_ID);
					}
				}

				if(fabs(PS2_TEMPERATURE) > MAX_TEMP)
				{
					if(CHECK_INTERLOCK(PS2_OVERTEMP))
					{
						//Set_HardInterlock(PS2_OVERTEMP,PS2_ID);
					    DP_Framework.NetSignals[20] = PS2_TEMPERATURE;
					    DP_Framework.NetSignals[21] = 2.0;
					}
				}

				if(CHECK_INTERLOCK(PS2_FUSE_FAIL) && !PIN_STATUS_PS2_FUSE)
				{
					Set_HardInterlock(PS2_FUSE_FAIL,PS2_ID);
				}

				if(CHECK_INTERLOCK(PS2_DRIVER_FAIL) && !PIN_STATUS_PS2_DRIVER_ERROR)
				{
					Set_HardInterlock(PS2_DRIVER_FAIL,PS2_ID);
				}

				if((IPC_CtoM_Msg.PSModule.OnOff & PS2_ID) && !PIN_STATUS_PS2_DCLINK_RELAY)
				{
					if(CHECK_INTERLOCK(PS2_DCLINK_RELAY_FAIL))
					{
						Set_HardInterlock(PS2_DCLINK_RELAY_FAIL,PS2_ID);
					}
				}

				if(nFBPs >= 3)
				{
					if(fabs(PS3_DCLINK_VOLTAGE) > MAX_DCLINK)
					{
						if(CHECK_INTERLOCK(PS3_DCLINK_OVERVOLTAGE))
						{
							Set_HardInterlock(PS3_DCLINK_OVERVOLTAGE,PS3_ID);
						}
					}

					if(fabs(PS3_DCLINK_VOLTAGE) < MIN_DCLINK)
					{
						if(CHECK_INTERLOCK(PS3_DCLINK_UNDERVOLTAGE))
						{
							Set_HardInterlock(PS3_DCLINK_UNDERVOLTAGE,PS3_ID);
						}
					}

					if(fabs(PS3_LOAD_VOLTAGE) > MAX_VLOAD)
					{
						if(CHECK_INTERLOCK(PS3_LOAD_OVERVOLTAGE))
						{
							Set_SoftInterlock(PS3_LOAD_OVERVOLTAGE,PS3_ID);
						}
					}

					if(fabs(PS3_TEMPERATURE) > MAX_TEMP)
					{
						if(CHECK_INTERLOCK(PS3_OVERTEMP))
						{
							//Set_HardInterlock(PS3_OVERTEMP,PS3_ID);
						    DP_Framework.NetSignals[20] = PS3_TEMPERATURE;
						    DP_Framework.NetSignals[21] = 3.0;
						}
					}

					if(CHECK_INTERLOCK(PS3_FUSE_FAIL) && !PIN_STATUS_PS3_FUSE)
					{
						Set_HardInterlock(PS3_FUSE_FAIL,PS3_ID);
					}

					if(CHECK_INTERLOCK(PS3_DRIVER_FAIL) && !PIN_STATUS_PS3_DRIVER_ERROR)
					{
						Set_HardInterlock(PS3_DRIVER_FAIL,PS3_ID);
					}

					if((IPC_CtoM_Msg.PSModule.OnOff & PS3_ID) && !PIN_STATUS_PS3_DCLINK_RELAY)
					{
						if(CHECK_INTERLOCK(PS3_DCLINK_RELAY_FAIL))
						{
							Set_HardInterlock(PS3_DCLINK_RELAY_FAIL,PS3_ID);
						}
					}

					if(nFBPs >= 4)
					{
						if(fabs(PS4_DCLINK_VOLTAGE) > MAX_DCLINK)
						{
							if(CHECK_INTERLOCK(PS4_DCLINK_OVERVOLTAGE))
							{
								Set_HardInterlock(PS4_DCLINK_OVERVOLTAGE,PS4_ID);
							}
						}

						if(fabs(PS4_DCLINK_VOLTAGE) < MIN_DCLINK)
						{
							if(CHECK_INTERLOCK(PS4_DCLINK_UNDERVOLTAGE))
							{
								Set_HardInterlock(PS4_DCLINK_UNDERVOLTAGE,PS4_ID);
							}
						}

						if(fabs(PS4_LOAD_VOLTAGE) > MAX_VLOAD)
						{
							if(CHECK_INTERLOCK(PS4_LOAD_OVERVOLTAGE))
							{
								Set_SoftInterlock(PS4_LOAD_OVERVOLTAGE,PS4_ID);
							}
						}

						if(fabs(PS4_TEMPERATURE) > MAX_TEMP)
						{
							if(CHECK_INTERLOCK(PS4_OVERTEMP))
							{
								//Set_HardInterlock(PS4_OVERTEMP,PS4_ID);
								DP_Framework.NetSignals[20] = PS4_TEMPERATURE;
								DP_Framework.NetSignals[21] = 4.0;
							}
						}

						if(CHECK_INTERLOCK(PS4_FUSE_FAIL) && !PIN_STATUS_PS4_FUSE)
						{
							Set_HardInterlock(PS4_FUSE_FAIL,PS4_ID);
						}

						if(CHECK_INTERLOCK(PS4_DRIVER_FAIL) && !PIN_STATUS_PS4_DRIVER_ERROR)
						{
							Set_HardInterlock(PS4_DRIVER_FAIL,PS4_ID);
						}

						if((IPC_CtoM_Msg.PSModule.OnOff & PS4_ID) && !PIN_STATUS_PS4_DCLINK_RELAY)
						{
							if(CHECK_INTERLOCK(PS4_DCLINK_RELAY_FAIL))
							{
								Set_HardInterlock(PS4_DCLINK_RELAY_FAIL,PS4_ID);
							}

							CLEAR_DEBUG_GPIO1;
						}

					}
				}
			}
		}
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
    Uint16 i;

	/* Initialization of HRADC boards */

    stop_DMA();
    Init_DMA_McBSP_nBuffers(IPC_MtoC_Msg.HRADCConfig.nHRADC, DECIMATION_FACTOR, HRADC_SPI_CLK);

    for(i = 0; i < IPC_MtoC_Msg.HRADCConfig.nHRADC; i++)
    {
        Init_HRADC_Info(&HRADCs_Info.HRADC_boards[i], i, DECIMATION_FACTOR, buffers_HRADC[i], TRANSDUCER_GAIN);
        Config_HRADC_board(&HRADCs_Info.HRADC_boards[i], Iin_bipolar, HEATER_DISABLE, RAILS_DISABLE);
    }

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

	Init_ELP_Error(ERROR_CALCULATOR_PS1, &DP_Framework.NetSignals[1], &PS1_LOAD_CURRENT, &DP_Framework.NetSignals[6]);

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

	Init_ELP_Error(ERROR_CALCULATOR_PS2, &DP_Framework.NetSignals[2], &PS2_LOAD_CURRENT, &DP_Framework.NetSignals[8]);

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

	Init_ELP_Error(ERROR_CALCULATOR_PS3, &DP_Framework.NetSignals[3], &PS3_LOAD_CURRENT, &DP_Framework.NetSignals[10]);

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

	Init_ELP_Error(ERROR_CALCULATOR_PS4, &DP_Framework.NetSignals[4], &PS4_LOAD_CURRENT, &DP_Framework.NetSignals[12]);

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

	ResetControllers(PS1_ID | PS2_ID | PS3_ID | PS4_ID);
}

static void ResetControllers(Uint16 ps_modules)
{
	if(ps_modules & PS1_ID)
	{
		SetPWMDutyCycle_HBridge(PWM_Modules.PWM_Regs[6], 0.0);
		Reset_ELP_Error(ERROR_CALCULATOR_PS1);
		Reset_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD_PS1);
	}

	if(ps_modules & PS2_ID)
	{
		SetPWMDutyCycle_HBridge(PWM_Modules.PWM_Regs[4], 0.0);
		Reset_ELP_Error(ERROR_CALCULATOR_PS2);
		Reset_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD_PS2);
	}

	if(ps_modules & PS3_ID)
	{
		SetPWMDutyCycle_HBridge(PWM_Modules.PWM_Regs[2], 0.0);
		Reset_ELP_Error(ERROR_CALCULATOR_PS3);
		Reset_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD_PS3);
	}

	if(ps_modules & PS4_ID)
	{
		SetPWMDutyCycle_HBridge(PWM_Modules.PWM_Regs[0], 0.0);
		Reset_ELP_Error(ERROR_CALCULATOR_PS4);
		Reset_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD_PS4);
	}

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

	SET_DEBUG_GPIO1;

	temp0 = (float) *(HRADCs_Info.HRADC_boards[0].SamplesBuffer);
	temp1 = (float) *(HRADCs_Info.HRADC_boards[1].SamplesBuffer);
	temp2 = (float) *(HRADCs_Info.HRADC_boards[2].SamplesBuffer);
	temp3 = (float) *(HRADCs_Info.HRADC_boards[3].SamplesBuffer);

	temp0 *= HRADCs_Info.HRADC_boards[0].gain;
	temp0 += HRADCs_Info.HRADC_boards[0].offset;

	temp1 *= HRADCs_Info.HRADC_boards[1].gain;
	temp1 += HRADCs_Info.HRADC_boards[1].offset;

    temp2 *= HRADCs_Info.HRADC_boards[2].gain;
    temp2 += HRADCs_Info.HRADC_boards[2].offset;

    temp3 *= HRADCs_Info.HRADC_boards[3].gain;
    temp3 += HRADCs_Info.HRADC_boards[3].offset;

	PS1_LOAD_CURRENT = temp0;
	PS2_LOAD_CURRENT = temp1;
	PS3_LOAD_CURRENT = temp2;
	PS4_LOAD_CURRENT = temp3;

	if(fabs(PS1_LOAD_CURRENT) > MAX_ILOAD)
	{
		if(CHECK_INTERLOCK(PS1_LOAD_OVERCURRENT))
		{
			Set_HardInterlock(PS1_LOAD_OVERCURRENT,PS1_ID);
		}
	}

	if(fabs(PS2_LOAD_CURRENT) > MAX_ILOAD)
	{
		if(CHECK_INTERLOCK(PS2_LOAD_OVERCURRENT))
		{
			Set_HardInterlock(PS2_LOAD_OVERCURRENT,PS2_ID);
		}
	}

	if(fabs(PS3_LOAD_CURRENT) > MAX_ILOAD)
	{
		if(CHECK_INTERLOCK(PS3_LOAD_OVERCURRENT))
		{
			Set_HardInterlock(PS3_LOAD_OVERCURRENT,PS3_ID);
		}
	}

	if(fabs(PS4_LOAD_CURRENT) > MAX_ILOAD)
	{
		if(CHECK_INTERLOCK(PS4_LOAD_OVERCURRENT))
		{
			Set_HardInterlock(PS4_LOAD_OVERCURRENT,PS4_ID);
		}
	}

	if(1)//IPC_CtoM_Msg.PSModule.OnOff)
	{

		switch(IPC_CtoM_Msg.PSModule.OpMode)
		{
			case SlowRef:
			{
				/*DP_Framework.NetSignals[0] = IPC_CtoM_Msg.PSModule.IRef;
				DP_Framework.NetSignals[1] = DP_Framework.NetSignals[0];
				DP_Framework.NetSignals[2] = DP_Framework.NetSignals[0];
				DP_Framework.NetSignals[3] = DP_Framework.NetSignals[0];
				DP_Framework.NetSignals[4] = DP_Framework.NetSignals[0];*/

				break;
			}

			case WfmRef:
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
				break;
			}

			case SigGen:
			{
			    SignalGenerator.Run_ELP_SigGen(&SignalGenerator);
                DP_Framework.NetSignals[0] = IPC_CtoM_Msg.PSModule.IRef;
                DP_Framework.NetSignals[1] = DP_Framework.NetSignals[0];
                DP_Framework.NetSignals[2] = DP_Framework.NetSignals[0];
                DP_Framework.NetSignals[3] = DP_Framework.NetSignals[0];
                DP_Framework.NetSignals[4] = DP_Framework.NetSignals[0];
				break;
			}

			default:
			{
				break;
			}
		}

		if(IPC_CtoM_Msg.PSModule.OpenLoop & PS1_ID)
		{
			DP_Framework.DutySignals[0] = 0.01*DP_Framework.NetSignals[1];
			SATURATE(DP_Framework.DutySignals[0], PWM_MAX_DUTY_OL, PWM_MIN_DUTY_OL);
		}
		else
		{
			SATURATE(DP_Framework.NetSignals[1], MAX_REF, MIN_REF);
			Run_ELP_Error(ERROR_CALCULATOR_PS1);
			Run_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD_PS1);
			SATURATE(DP_Framework.DutySignals[0], PWM_MAX_DUTY, PWM_MIN_DUTY);
		}


		if(IPC_CtoM_Msg.PSModule.OpenLoop & PS2_ID)
		{
			DP_Framework.DutySignals[1] = 0.01*DP_Framework.NetSignals[2];
			SATURATE(DP_Framework.DutySignals[1], PWM_MAX_DUTY_OL, PWM_MIN_DUTY_OL);
		}
		else
		{
			SATURATE(DP_Framework.NetSignals[2], MAX_REF, MIN_REF);
			Run_ELP_Error(ERROR_CALCULATOR_PS2);
			Run_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD_PS2);
			SATURATE(DP_Framework.DutySignals[1], PWM_MAX_DUTY, PWM_MIN_DUTY);
		}


		if(IPC_CtoM_Msg.PSModule.OpenLoop & PS3_ID)
		{
			DP_Framework.DutySignals[2] = 0.01*DP_Framework.NetSignals[3];
			SATURATE(DP_Framework.DutySignals[2], PWM_MAX_DUTY_OL, PWM_MIN_DUTY_OL);
		}
		else
		{
			SATURATE(DP_Framework.NetSignals[3], MAX_REF, MIN_REF);
			Run_ELP_Error(ERROR_CALCULATOR_PS3);
			Run_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD_PS3);
			SATURATE(DP_Framework.DutySignals[2], PWM_MAX_DUTY, PWM_MIN_DUTY);
		}


		if(IPC_CtoM_Msg.PSModule.OpenLoop & PS4_ID)
		{
			DP_Framework.DutySignals[3] = 0.01*DP_Framework.NetSignals[4];
			SATURATE(DP_Framework.DutySignals[3], PWM_MAX_DUTY_OL, PWM_MIN_DUTY_OL);
		}
		else
		{
			SATURATE(DP_Framework.NetSignals[4], MAX_REF, MIN_REF);
			Run_ELP_Error(ERROR_CALCULATOR_PS4);
			Run_ELP_PI_dawu(PI_DAWU_CONTROLLER_ILOAD_PS4);
			SATURATE(DP_Framework.DutySignals[3], PWM_MAX_DUTY, PWM_MIN_DUTY);
		}


		SetPWMDutyCycle_HBridge(PWM_Modules.PWM_Regs[0],DP_Framework.DutySignals[3]);
		SetPWMDutyCycle_HBridge(PWM_Modules.PWM_Regs[2],DP_Framework.DutySignals[2]);
		SetPWMDutyCycle_HBridge(PWM_Modules.PWM_Regs[4],DP_Framework.DutySignals[1]);
		SetPWMDutyCycle_HBridge(PWM_Modules.PWM_Regs[6],DP_Framework.DutySignals[0]);
	}

	for(i = 0; i < PWM_Modules.N_modules; i++)
	{
		PWM_Modules.PWM_Regs[i]->ETCLR.bit.INT = 1;
	}

	CLEAR_DEBUG_GPIO1;

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

static void Set_SoftInterlock(Uint32 itlk, Uint16 ps_modules)
{
	//PS_turnOff(ps_modules);
	IPC_CtoM_Msg.PSModule.SoftInterlocks |= itlk;
	//SendIpcFlag(SOFT_INTERLOCK_CTOM);
}

static void Set_HardInterlock(Uint32 itlk, Uint16 ps_modules)
{
#ifdef USE_ITLK
    PS_turnOff(ps_modules);
#endif

	IPC_CtoM_Msg.PSModule.HardInterlocks |= itlk;
	SendIpcFlag(HARD_INTERLOCK_CTOM);
}

static interrupt void isr_SoftInterlock(void)
{
	CtoMIpcRegs.MTOCIPCACK.all = SOFT_INTERLOCK_MTOC;

	//PS_turnOff(PS_ALL_ID);
	IPC_CtoM_Msg.PSModule.SoftInterlocks |= IPC_MtoC_Msg.PSModule.SoftInterlocks;

	PieCtrlRegs.PIEACK.all |= M_INT11;
}

static interrupt void isr_HardInterlock(void)
{
	CtoMIpcRegs.MTOCIPCACK.all = HARD_INTERLOCK_MTOC;

#ifdef USE_ITLK
    PS_turnOff(PS_ALL_ID);
#endif

	IPC_CtoM_Msg.PSModule.HardInterlocks |= IPC_MtoC_Msg.PSModule.HardInterlocks;

	PieCtrlRegs.PIEACK.all |= M_INT11;
}

static void PS_turnOn(Uint16 ps_modules)
{
	ResetControllers(ps_modules);

#ifdef USE_ITLK
	if((ps_modules & PS1_ID) && !(IPC_CtoM_Msg.PSModule.HardInterlocks & 0x000000FF))
#else
	if(ps_modules & PS1_ID)
#endif
	{
		PIN_CLOSE_PS1_DCLINK_RELAY;
		DELAY_US(100000);

#ifdef USE_ITLK
		if(!PIN_STATUS_PS1_DCLINK_RELAY && CHECK_INTERLOCK(PS1_DCLINK_RELAY_FAIL))
		{
			Set_HardInterlock(PS1_DCLINK_RELAY_FAIL,PS1_ID);
		}
		else
#endif
		{
			IPC_CtoM_Msg.PSModule.OpenLoop |= PS1_ID;
			IPC_CtoM_Msg.PSModule.OnOff |= PS1_ID;

			EnablePWMOutput(6);
			EnablePWMOutput(7);
		}
	}

#ifdef USE_ITLK
	if((ps_modules & PS2_ID) && !(IPC_CtoM_Msg.PSModule.HardInterlocks & 0x0000FF00))
#else
	if(ps_modules & PS2_ID)
#endif
	{
		PIN_CLOSE_PS2_DCLINK_RELAY;
		DELAY_US(100000);

#ifdef USE_ITLK
		if(!PIN_STATUS_PS2_DCLINK_RELAY && CHECK_INTERLOCK(PS2_DCLINK_RELAY_FAIL))
		{
			Set_HardInterlock(PS2_DCLINK_RELAY_FAIL,PS2_ID);
		}
		else
#endif
		{
			IPC_CtoM_Msg.PSModule.OpenLoop |= PS2_ID;
			IPC_CtoM_Msg.PSModule.OnOff |= PS2_ID;

			EnablePWMOutput(4);
			EnablePWMOutput(5);
		}
	}

#ifdef USE_ITLK
	if((ps_modules & PS3_ID) && !(IPC_CtoM_Msg.PSModule.HardInterlocks & 0x00FF0000))
#else
    if(ps_modules & PS3_ID)
#endif
	{
		PIN_CLOSE_PS3_DCLINK_RELAY;
		DELAY_US(100000);

#ifdef USE_ITLK
		if(!PIN_STATUS_PS3_DCLINK_RELAY && CHECK_INTERLOCK(PS3_DCLINK_RELAY_FAIL))
		{
			Set_HardInterlock(PS3_DCLINK_RELAY_FAIL,PS3_ID);
		}
		else
#endif
		{
			IPC_CtoM_Msg.PSModule.OpenLoop |= PS3_ID;
			IPC_CtoM_Msg.PSModule.OnOff |= PS3_ID;

			EnablePWMOutput(2);
			EnablePWMOutput(3);
		}
	}

#ifdef USE_ITLK
	if((ps_modules & PS4_ID) && !(IPC_CtoM_Msg.PSModule.HardInterlocks & 0xFF000000))
#else
    if(ps_modules & PS4_ID)
#endif
	{
		PIN_CLOSE_PS4_DCLINK_RELAY;
		DELAY_US(100000);

#ifdef USE_ITLK
		if(!PIN_STATUS_PS4_DCLINK_RELAY && CHECK_INTERLOCK(PS4_DCLINK_RELAY_FAIL))
		{
			Set_HardInterlock(PS4_DCLINK_RELAY_FAIL,PS4_ID);
		}
		else
#endif
		{
			IPC_CtoM_Msg.PSModule.OpenLoop |= PS4_ID;
			IPC_CtoM_Msg.PSModule.OnOff |= PS4_ID;

			EnablePWMOutput(0);
			EnablePWMOutput(1);
		}
	}
}

static void PS_turnOff(Uint16 ps_modules)
{
	if(ps_modules & PS1_ID)
	{
		DisablePWMOutput(6);
		DisablePWMOutput(7);
		PIN_OPEN_PS1_DCLINK_RELAY;
		IPC_CtoM_Msg.PSModule.OnOff &= ~PS1_ID; //0xFFFE;
		IPC_CtoM_Msg.PSModule.OpenLoop |= PS1_ID;
	}

	if(ps_modules & PS2_ID)
	{
		DisablePWMOutput(4);
		DisablePWMOutput(5);
		PIN_OPEN_PS2_DCLINK_RELAY;
		IPC_CtoM_Msg.PSModule.OnOff &= ~PS2_ID; //0xFFFD;
		IPC_CtoM_Msg.PSModule.OpenLoop |= PS2_ID;
	}

	if(ps_modules & PS3_ID)
	{
		DisablePWMOutput(2);
		DisablePWMOutput(3);
		PIN_OPEN_PS3_DCLINK_RELAY;
		IPC_CtoM_Msg.PSModule.OnOff &= ~PS3_ID; //0xFFFB;
		IPC_CtoM_Msg.PSModule.OpenLoop |= PS3_ID;
	}

	if(ps_modules & PS4_ID)
	{
		DisablePWMOutput(0);
		DisablePWMOutput(1);
		PIN_OPEN_PS4_DCLINK_RELAY;
		IPC_CtoM_Msg.PSModule.OnOff &= ~PS4_ID; //0xFFF7
		IPC_CtoM_Msg.PSModule.OpenLoop |= PS4_ID;
	}

	ResetControllers(ps_modules);
}
