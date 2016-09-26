	/*
 * 		FILE: 		FAP_ACDC.c
 * 		PROJECT: 	DRS v2.0
 * 		CREATION:	06/06/2016
 * 		MODIFIED:
 *
 * 		AUTHOR: 	Gabriel O. B.  (LNLS/ELP)
 *
 * 		DESCRIPTION: Firmware for control of AC/DC stage for
 * 					 200 A / 50 V FAP prototype v2.0
 *
 *
 *		TODO:
 */

#include "F28M36x_ELP_DRS.h"
#include "FAP_ACDC.h"

/*
 * Timeouts
 */

#define TIMEOUT_uS_AC_CONTACTOR						100000
#define TIMEOUT_uS_PRECHARGERS_BYPASS_CONTACTORS	100000
#define TIMEOUT_uS_OUTPUT_CAP_CHARGE				10000000

/*
 * Digital IO's defines
 */

#define PIN_STATUS_ALL						GpioDataRegs.GPDDATA.all && 0xD0000000
#define PIN_STATUS_AC_CONTACTOR 			GpioDataRegs.GPDDAT.bit.GPIO126					// GPDI1
#define PIN_STATUS_BYPASS_PRECHARGER_1 		GpioDataRegs.GPDDAT.bit.GPIO127					// GPDI2
#define PIN_STATUS_BYPASS_PRECHARGER_2		GpioDataRegs.GPDDAT.bit.GPIO124					// GPDI3

#define PIN_OPEN_AC_CONTACTOR				GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;			// GPDO1
#define PIN_CLOSE_AC_CONTACTOR				GpioDataRegs.GPCSET.bit.GPIO67 = 1;

#define PIN_OPEN_PRECHARGERS_BYPASS_CONTACTOR		GpioDataRegs.GPCCLEAR.bit.GPIO65 = 1;	// GPDO2
#define PIN_CLOSE_PRECHARGERS_BYPASS_CONTACTOR		GpioDataRegs.GPCSET.bit.GPIO65 = 1;

// Prototype statements for functions found within this file.

#pragma CODE_SECTION(PS_turnOn, "ramfuncs");
#pragma CODE_SECTION(PS_turnOff, "ramfuncs");
#pragma CODE_SECTION(Set_SoftInterlock, "ramfuncs");
#pragma CODE_SECTION(Set_HardInterlock, "ramfuncs");
#pragma CODE_SECTION(isr_SoftInterlock, "ramfuncs");
#pragma CODE_SECTION(isr_HardInterlock, "ramfuncs");

void main_FAP_ACDC(void);

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
static void TerminateInterruptions(void);

volatile static Uint32 valorCounter;
static Uint16 pinStatus_ACContactor, pinStatus_BypassPrecharger1, pinStatus_BypassPrecharger2;

void main_FAP_ACDC(void)
{
	InitPeripheralsDrivers();

	InitControllers();

	InitInterruptions();

	//while(IPC_MtoC_Msg.PSModule.Model == FAP_ACDC)
	while(1)
	{
		SET_DEBUG_GPIO1;

		DINT;

		pinStatus_ACContactor = PIN_STATUS_AC_CONTACTOR;
		pinStatus_BypassPrecharger1 = PIN_STATUS_BYPASS_PRECHARGER_1;
		pinStatus_BypassPrecharger2 = PIN_STATUS_BYPASS_PRECHARGER_2;

		if(IPC_CtoM_Msg.PSModule.OnOff)
		{
			if( CHECK_INTERLOCK(AC_FAULT) && !pinStatus_ACContactor )
			{
				Set_HardInterlock(AC_FAULT);
			}

			if( CHECK_INTERLOCK(PRECHARGERS_FAULT) && !(pinStatus_BypassPrecharger1 && pinStatus_BypassPrecharger2) )
			{
				Set_HardInterlock(PRECHARGERS_FAULT);
			}
		}

		EINT;
		/*else
		{
			if( CHECK_INTERLOCK(AC_FAULT) && PIN_STATUS_AC_CONTACTOR )
			{
				Set_HardInterlock(AC_FAULT);
			}

			if( CHECK_INTERLOCK(PRECHARGERS_FAULT) && (PIN_STATUS_BYPASS_PRECHARGER_1 || PIN_STATUS_BYPASS_PRECHARGER_2) )
			{
				Set_HardInterlock(PRECHARGERS_FAULT);
			}
		}*/

		if( CHECK_INTERLOCK(OUT1_OVERVOLTAGE) && (DP_Framework_MtoC.NetSignals[9] > MAX_V_OUT) )
		{
			Set_HardInterlock(OUT1_OVERVOLTAGE);
		}

		if( CHECK_INTERLOCK(OUT2_OVERVOLTAGE) && (DP_Framework_MtoC.NetSignals[10] > MAX_V_OUT) )
		{
			Set_HardInterlock(OUT2_OVERVOLTAGE);
		}

		CLEAR_DEBUG_GPIO1;
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
	PWM_Modules.N_modules = 1;
	PWM_Modules.PWM_Regs[0] = &EPwm1Regs;

	DisablePWMOutputs();
	DisablePWM_TBCLK();
	InitPWM_MEP_SFO();

	InitPWMModule(PWM_Modules.PWM_Regs[0], PWM_FREQ, 0, MasterPWM, 0, COMPLEMENTARY, PWM_DEAD_TIME);

	InitPWMDAC(PWM_DAC_MODULE, PWM_DAC_FREQ);

	InitEPwm1Gpio();

	/* Initialization of GPIOs */

	EALLOW;

	GpioCtrlRegs.GPDMUX2.bit.GPIO126 = 0;
	GpioDataRegs.GPDCLEAR.bit.GPIO126 = 1;		// GPDI1: AC mains contactor status
	GpioCtrlRegs.GPDDIR.bit.GPIO126 = 0;

	GpioCtrlRegs.GPDMUX2.bit.GPIO127 = 0;
	GpioDataRegs.GPDCLEAR.bit.GPIO127 = 1;		// GPDI2: Capacitor pre-charger 1 contactor status
	GpioCtrlRegs.GPDDIR.bit.GPIO127 = 0;

	GpioCtrlRegs.GPDMUX2.bit.GPIO124 = 0;
	GpioDataRegs.GPDCLEAR.bit.GPIO124 = 1;		// GPDI3: Capacitor pre-charger 2 contactor status
	GpioCtrlRegs.GPDDIR.bit.GPIO124 = 0;

	GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 0;
	GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;		// GPDO1: AC mains contactor switch
	GpioCtrlRegs.GPCDIR.bit.GPIO67 = 1;

	GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 0;
	GpioDataRegs.GPCCLEAR.bit.GPIO65 = 1;			// GPDO2: Capacitors pre-chargers contactor switch
	GpioCtrlRegs.GPCDIR.bit.GPIO65 = 1;

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

	/* This variable is used to indicate the output capacitors are charged and
	 * pre-charge resistors are bypassed (OpenLoop = 1)
	 */
	IPC_CtoM_Msg.PSModule.OpenLoop = CLOSED_LOOP;

	/* Initiaization of DP Framework */
	InitDP_Framework(&DP_Framework, &(IPC_CtoM_Msg.PSModule.IRef));

	/**********************************/
	/* INITIALIZATION OF TIME SLICERS */
	/**********************************/

	// 0: Time-slicer for WfmRef sweep decimation
	Set_TimeSlicer(0, BUFFER_DECIMATION);

	ResetControllers();
}

static void ResetControllers(void)
{
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
	IER |= M_INT11;
    //IER |= M_INT13;		// CPU Timer 1: Timeout controller

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

	/* Clear flags */
	PieCtrlRegs.PIEACK.all |= M_INT11 ;
}

static void Set_SoftInterlock(Uint32 itlk)
{
	PS_turnOff();
	IPC_CtoM_Msg.PSModule.SoftInterlocks |= itlk;
	SendIpcFlag(SOFT_INTERLOCK_CTOM);
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
	if(CHECK_INTERLOCKS)
	{
		if(!(IPC_CtoM_Msg.PSModule.OnOff || PIN_STATUS_AC_CONTACTOR))
		{
			// Configure CPU Timer 1 for pre-chargers timeout monitor
			ConfigCpuTimer(&CpuTimer1, C28_FREQ_MHZ, TIMEOUT_uS_PRECHARGERS_BYPASS_CONTACTORS);
			CpuTimer1Regs.TCR.all = 0x8000;

			// Open bypass of pre-charger capacitors
			PIN_OPEN_PRECHARGERS_BYPASS_CONTACTOR;

			// Start timeout monitor
			StartCpuTimer1();

			// Check pre-chargers status
			while(PIN_STATUS_BYPASS_PRECHARGER_1 || PIN_STATUS_BYPASS_PRECHARGER_2)
			{
				// If timeout, set interlock
				if(CpuTimer1Regs.TCR.bit.TIF)
				{
					Set_HardInterlock(PRECHARGERS_FAULT);
					StopCpuTimer1();
					CpuTimer1Regs.TCR.all = 0x8000;
					return;
				}
			}

			IPC_CtoM_Msg.PSModule.OpenLoop = CLOSED_LOOP;

			// Configure CPU Timer 1 for AC mains timeout monitor
			ConfigCpuTimer(&CpuTimer1, C28_FREQ_MHZ, TIMEOUT_uS_AC_CONTACTOR);
			CpuTimer1Regs.TCR.all = 0x8000;

			// Close AC mains contactor
			PIN_CLOSE_AC_CONTACTOR;

			// Start timeout monitor
			StartCpuTimer1();

			// Check AC mains status
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

			IPC_CtoM_Msg.PSModule.OnOff = 1;

			DELAY_US(20000);

			// Configure and start CPU Timer 1 for output capacitors charge timeout monitor
			ConfigCpuTimer(&CpuTimer1, C28_FREQ_MHZ, TIMEOUT_uS_OUTPUT_CAP_CHARGE);
			CpuTimer1Regs.TCR.all = 0x8000;
			StartCpuTimer1();

			// Monitor output voltages
			while((DP_Framework_MtoC.NetSignals[9] < MIN_V_OUT_CHARGE) || (DP_Framework_MtoC.NetSignals[10] < MIN_V_OUT_CHARGE))
			{
				// If timeout, set interlock
				if(CpuTimer1Regs.TCR.bit.TIF)
				{
					Set_HardInterlock(OUTPUT_CAP_CHARGE_FAULT);
					StopCpuTimer1();
					CpuTimer1Regs.TCR.all = 0x8000;
					return;
				}

				if(!PIN_STATUS_AC_CONTACTOR)
				{
					// If AC contactor is off, and it wasn't occasioned by IIB, then interlock for AC fault
					if(!CtoMIpcRegs.MTOCIPCSTS.all)
					{
						Set_HardInterlock(AC_FAULT);
					}
					return;
				}
				if(PIN_STATUS_BYPASS_PRECHARGER_1 || PIN_STATUS_BYPASS_PRECHARGER_2)
				{
					Set_HardInterlock(PRECHARGERS_FAULT);
					return;
				}
			}


			// Configure CPU Timer 1 for pre-chargers timeout monitor
			ConfigCpuTimer(&CpuTimer1, C28_FREQ_MHZ, TIMEOUT_uS_PRECHARGERS_BYPASS_CONTACTORS);
			CpuTimer1Regs.TCR.all = 0x8000;

			// Bypass pre-charger capacitors
			PIN_CLOSE_PRECHARGERS_BYPASS_CONTACTOR;

			// Start timeout monitor
			StartCpuTimer1();

			// Check pre-chargers status
			while( !(PIN_STATUS_BYPASS_PRECHARGER_1 && PIN_STATUS_BYPASS_PRECHARGER_2) )
			{
				// If timeout, set interlock
				if(CpuTimer1Regs.TCR.bit.TIF)
				{
					Set_HardInterlock(PRECHARGERS_FAULT);
					StopCpuTimer1();
					CpuTimer1Regs.TCR.all = 0x8000;
					return;
				}
			}

			IPC_CtoM_Msg.PSModule.OpenLoop = OPEN_LOOP;

			StopCpuTimer1();
			CpuTimer1Regs.TCR.all = 0x8000;

			DELAY_US(20000);
		}
	}
}

static void PS_turnOff(void)
{
	PIN_OPEN_AC_CONTACTOR;

	// Configure and start CPU Timer 1 for AC mains timeout monitor
	ConfigCpuTimer(&CpuTimer1, C28_FREQ_MHZ, TIMEOUT_uS_AC_CONTACTOR);
	CpuTimer1Regs.TCR.all = 0x8000;
	StartCpuTimer1();

	// Check AC mains status
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

	// Open bypass of pre-charger capacitors
	PIN_OPEN_PRECHARGERS_BYPASS_CONTACTOR;

	// Configure and start CPU Timer 1 for pre-chargers timeout monitor
	ConfigCpuTimer(&CpuTimer1, C28_FREQ_MHZ, TIMEOUT_uS_PRECHARGERS_BYPASS_CONTACTORS);
	CpuTimer1Regs.TCR.all = 0x8000;
	StartCpuTimer1();

	// Check pre-chargers status
	while(PIN_STATUS_BYPASS_PRECHARGER_1 || PIN_STATUS_BYPASS_PRECHARGER_2)
	{
		// If timeout, set interlock
		if(CpuTimer1Regs.TCR.bit.TIF)
		{
			IPC_CtoM_Msg.PSModule.HardInterlocks |= PRECHARGERS_FAULT;
			SendIpcFlag(HARD_INTERLOCK_CTOM);
			StopCpuTimer1();
			CpuTimer1Regs.TCR.all = 0x8000;
			break;
		}
	}

	IPC_CtoM_Msg.PSModule.OpenLoop = CLOSED_LOOP;
	IPC_CtoM_Msg.PSModule.OnOff = 0;
	ResetControllers();
}

