/*
 * 		FILE: 		Test_HRADC.c
 * 		PROJECT: 	DRS v2.0
 * 		CREATION:	04/04/2016
 * 		MODIFIED:	04/04/2016
 *
 * 		AUTHOR: 	Gabriel O. B.  (LNLS/ELP)
 *
 * 		DESCRIPTION:
 *
 * 			Firmware for automated test of HRADC boards
 *
 *		TODO:
 */

#include "F28M36x_ELP_DRS.h"
#include "Jiga_HRADC_v2_1.h"

#define	N_MAX_SAMPLING_TESTS			6
#define UFM_BUFFER_SIZE 				100

#define HRADC_CONFIG_FAULT		0x00000001
#define OUT_OF_RANGE_FAULT		0x00000002

/*
 * Prototype statements for functions found within this file.
 */

#pragma CODE_SECTION(isr_ePWM_CTR_ZERO, "ramfuncs");
#pragma CODE_SECTION(isr_ePWM_CTR_ZERO_1st, "ramfuncs");

static interrupt void isr_ePWM_CTR_ZERO(void);
static interrupt void isr_ePWM_CTR_ZERO_1st(void);

void main_Jiga_HRADC_v2_1(void);

static void InitPeripheralsDrivers(void);

static void InitControllers(void);

static void InitInterruptions(void);

static void PS_turnOn(void);
static void PS_turnOff(void);

static Uint32 RunTests(void);

static void Set_SoftInterlock(Uint32 itlk);
static void Set_HardInterlock(Uint32 itlk);
static interrupt void isr_SoftInterlock(void);
static interrupt void isr_HardInterlock(void);

static eInputType *ptr_CurrentAnalogInput;
static eInputType TestAnalogInputs[N_MAX_SAMPLING_TESTS] = { Iin_bipolar, Vin_bipolar, GND, Vref_bipolar_p, Vref_bipolar_n, Temp };

static Uint16 UFM_buffer[UFM_BUFFER_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static Uint32 valorCounter;

static float HRADC_Sample, *ptr_CurrentValue, *ptr_CurrentTolerance;
static float TestValues[N_MAX_SAMPLING_TESTS] = {0.0, 0.0, 0.0, 5.0, -5.0, -0.35};
static float TestTolerances[N_MAX_SAMPLING_TESTS] = {0.005, 0.005, 0.005, 0.005, 0.005, 0.1};


void main_Jiga_HRADC_v2_1(void)
{
	InitPeripheralsDrivers();
	InitControllers();
	InitInterruptions();

	while(1)
	{
		if(IPC_CtoM_Msg.PSModule.OnOff)
		{
			RunTests();
		}
	}

}

/*
 * Initialization of peripheral drivers:
 *
 *		- HRADC board
 * 		- PWM modules
 * 		- GPIOs
 * 		- Timers
 */

static void InitPeripheralsDrivers(void)
{
	/* Initialization of HRADC boards */

	stop_DMA();

    Init_DMA_McBSP_nBuffers(N_HRADC_BOARDS, DECIMATION_FACTOR, HRADC_SPI_CLK);

    Init_SPIMaster_McBSP(HRADC_SPI_CLK);
    Init_SPIMaster_Gpio();
    InitMcbspa20bit();

    HRADCs_Info.enable_Sampling = 0;
    HRADCs_Info.n_HRADC_boards = N_HRADC_BOARDS;

    HRADCs_Info.HRADC_boards[0] = &HRADC0_board;
    HRADCs_Info.HRADC_boards[1] = &HRADC1_board;
    HRADCs_Info.HRADC_boards[2] = &HRADC2_board;
    HRADCs_Info.HRADC_boards[3] = &HRADC3_board;

    Init_HRADC_Info(HRADCs_Info.HRADC_boards[0], 0, DECIMATION_FACTOR, buffers_HRADC.buffer_0, TRANSDUCER_0_GAIN, HRADC_0_R_BURDEN);
    Init_HRADC_Info(HRADCs_Info.HRADC_boards[1], 1, DECIMATION_FACTOR, buffers_HRADC.buffer_1, TRANSDUCER_1_GAIN, HRADC_1_R_BURDEN);
    Init_HRADC_Info(HRADCs_Info.HRADC_boards[2], 2, DECIMATION_FACTOR, buffers_HRADC.buffer_2, TRANSDUCER_2_GAIN, HRADC_2_R_BURDEN);
    Init_HRADC_Info(HRADCs_Info.HRADC_boards[3], 3, DECIMATION_FACTOR, buffers_HRADC.buffer_3, TRANSDUCER_3_GAIN, HRADC_3_R_BURDEN);

    Config_HRADC_board(HRADCs_Info.HRADC_boards[0], TRANSDUCER_0_OUTPUT_TYPE, HEATER_DISABLE, RAILS_DISABLE);
    Config_HRADC_board(HRADCs_Info.HRADC_boards[1], TRANSDUCER_1_OUTPUT_TYPE, HEATER_DISABLE, RAILS_DISABLE);
    Config_HRADC_board(HRADCs_Info.HRADC_boards[2], TRANSDUCER_2_OUTPUT_TYPE, HEATER_DISABLE, RAILS_DISABLE);
    Config_HRADC_board(HRADCs_Info.HRADC_boards[3], TRANSDUCER_3_OUTPUT_TYPE, HEATER_DISABLE, RAILS_DISABLE);

    Config_HRADC_SoC(HRADC_FREQ_SAMP);

    stop_DMA();

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

    InitPWMModule(PWM_Modules.PWM_Regs[0], PWM_FREQ, 0, MasterPWM, 0, NO_COMPLEMENTARY, 0);

    SetPWMDutyCycle_ChA(&EPwm1Regs,0.0);

    InitEPwm1Gpio();

	/* Initialization of GPIOs */

	EALLOW;
	INIT_DEBUG_GPIO1;		// Debug GPIO's
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
	IPC_CtoM_Msg.SamplesBuffer.BufferBusy = Buffer_All;
}

static void InitInterruptions(void)
{
	EALLOW;
	PieVectTable.EPWM1_INT =  &isr_ePWM_CTR_ZERO_1st;
	EDIS;

	PieCtrlRegs.PIEIER3.bit.INTx1 = 1;  // ePWM1
	EnablePWMInterrupt(PWM_Modules.PWM_Regs[0]);

	IER |= M_INT3;
	IER |= M_INT11;

	/* Enable global interrupts (EINT) */
	EINT;
	ERTM;
}

//*****************************************************************************
// Esvazia buffer FIFO com valores amostrados e recebidos via SPI
//*****************************************************************************
interrupt void isr_ePWM_CTR_ZERO(void)
{
	Uint16 i;

	SET_DEBUG_GPIO1;

	while(!McbspaRegs.SPCR1.bit.RRDY){}

	HRADC_Sample = (float) *(HRADCs_Info.HRADC_boards[0]->SamplesBuffer);
	HRADC_Sample -= *(HRADCs_Info.HRADC_boards[0]->offset);
	HRADC_Sample *= *(HRADCs_Info.HRADC_boards[0]->gain);

	DP_Framework.NetSignals[1] = HRADC_Sample;

	if(WriteBuffer(&IPC_CtoM_Msg.SamplesBuffer, HRADC_Sample))
	{
		Disable_HRADC_Sampling();
		PWM_Modules.PWM_Regs[0]->ETCLR.bit.INT = 1;
		PieCtrlRegs.PIEIFR3.all = 0x0000;
	}

	for(i = 0; i < PWM_Modules.N_modules; i++)
	{
		PWM_Modules.PWM_Regs[i]->ETCLR.bit.INT = 1;
	}

	PieCtrlRegs.PIEACK.all |= M_INT3;
	CLEAR_DEBUG_GPIO1;
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
	IPC_CtoM_Msg.PSModule.SoftInterlocks |= itlk;
	//SendIpcFlag(SOFT_INTERLOCK_CTOM);
}

static void Set_HardInterlock(Uint32 itlk)
{
	IPC_CtoM_Msg.PSModule.OpenLoop = 1;
	IPC_CtoM_Msg.PSModule.HardInterlocks |= itlk;
	SendIpcFlag(HARD_INTERLOCK_CTOM);
}

static interrupt void isr_SoftInterlock(void)
{
}

static interrupt void isr_HardInterlock(void)
{
}

static void PS_turnOn(void)
{
	IPC_CtoM_Msg.PSModule.OnOff = 1;
}

static void PS_turnOff(void)
{
	IPC_CtoM_Msg.PSModule.OnOff = 0;
}


static Uint32 RunTests(void)
{
	Uint16 i;

	HRADC_Sample = 0.0;
	ptr_CurrentValue = TestValues;
	ptr_CurrentTolerance = TestTolerances;
	ptr_CurrentAnalogInput = TestAnalogInputs;
	IPC_CtoM_Msg.PSModule.IRef = 0.0;
	IPC_CtoM_Msg.PSModule.OpenLoop = 1;

	for(i = 0; i < N_MAX_SAMPLING_TESTS; i++)
	{
		IPC_CtoM_Msg.PSModule.IRef++;

		if(Try_Config_HRADC_board(HRADCs_Info.HRADC_boards[0], *ptr_CurrentAnalogInput, HEATER_DISABLE, RAILS_DISABLE))
		{
			Set_HardInterlock(HRADC_CONFIG_FAULT);
			break;
		}

		Enable_HRADC_Sampling();

		// Waits samplesBuffer is full, then proceed
		while(HRADCs_Info.enable_Sampling){}

		if(Test_SamplesLimit(&IPC_CtoM_Msg.SamplesBuffer, *ptr_CurrentValue, *ptr_CurrentTolerance))
		{
			Set_HardInterlock(OUT_OF_RANGE_FAULT);
			break;
		}

		ptr_CurrentAnalogInput++;
		ptr_CurrentValue++;
		ptr_CurrentTolerance++;

		DELAY_US(1000000);
	}

	if(CHECK_INTERLOCKS)
	{
		IPC_CtoM_Msg.PSModule.OpenLoop = 0;
	}

	IPC_CtoM_Msg.PSModule.OnOff = 0;

	return IPC_CtoM_Msg.PSModule.HardInterlocks;
}
