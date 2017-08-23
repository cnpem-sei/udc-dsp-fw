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

static void Set_SoftInterlock(Uint32 itlk);
static void Set_HardInterlock(Uint32 itlk);
static interrupt void isr_SoftInterlock(void);
static interrupt void isr_HardInterlock(void);

static Uint16 UFM_buffer[UFM_BUFFER_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static Uint32 valorCounter;

void main_Jiga_HRADC_v2_1(void)
{
	while( (IPC_MtoC_Msg.HRADCConfig.nHRADC < 1) || (IPC_MtoC_Msg.HRADCConfig.nHRADC > 4) ){}

	InitPeripheralsDrivers();
	InitControllers();
	InitInterruptions();

	while(1)
	{
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

	HRADCs_Info.enable_Sampling = 0;
	HRADCs_Info.n_HRADC_boards = IPC_MtoC_Msg.HRADCConfig.nHRADC;

    Init_DMA_McBSP_nBuffers(HRADCs_Info.n_HRADC_boards, DECIMATION_FACTOR, HRADC_SPI_CLK);

    Init_SPIMaster_McBSP(HRADC_SPI_CLK);
    Init_SPIMaster_Gpio();
    InitMcbspa20bit();

    /*HRADCs_Info.HRADC_boards[0] = &HRADC0_board;
    HRADCs_Info.HRADC_boards[1] = &HRADC1_board;
    HRADCs_Info.HRADC_boards[2] = &HRADC2_board;
    HRADCs_Info.HRADC_boards[3] = &HRADC3_board;*/

    Init_HRADC_Info(&HRADCs_Info.HRADC_boards[0], 0, DECIMATION_FACTOR, buffers_HRADC.buffer_0, TRANSDUCER_0_GAIN, HRADC_0_R_BURDEN);
    Init_HRADC_Info(&HRADCs_Info.HRADC_boards[1], 1, DECIMATION_FACTOR, buffers_HRADC.buffer_1, TRANSDUCER_1_GAIN, HRADC_1_R_BURDEN);
    Init_HRADC_Info(&HRADCs_Info.HRADC_boards[2], 2, DECIMATION_FACTOR, buffers_HRADC.buffer_2, TRANSDUCER_2_GAIN, HRADC_2_R_BURDEN);
    Init_HRADC_Info(&HRADCs_Info.HRADC_boards[3], 3, DECIMATION_FACTOR, buffers_HRADC.buffer_3, TRANSDUCER_3_GAIN, HRADC_3_R_BURDEN);

    /*Config_HRADC_board(&HRADCs_Info.HRADC_boards[0], TRANSDUCER_0_OUTPUT_TYPE, HEATER_DISABLE, RAILS_DISABLE);
    Config_HRADC_board(&HRADCs_Info.HRADC_boards[1], TRANSDUCER_1_OUTPUT_TYPE, HEATER_DISABLE, RAILS_DISABLE);
    Config_HRADC_board(&HRADCs_Info.HRADC_boards[2], TRANSDUCER_2_OUTPUT_TYPE, HEATER_DISABLE, RAILS_DISABLE);
    Config_HRADC_board(&HRADCs_Info.HRADC_boards[3], TRANSDUCER_3_OUTPUT_TYPE, HEATER_DISABLE, RAILS_DISABLE);*/

    Config_HRADC_SoC(HRADC_FREQ_SAMP);

    stop_DMA();

    /* Initialization of PWM modules */

    PWM_Modules.N_modules = 1;
    PWM_Modules.PWM_Regs[0] = &EPwm1Regs;

	DisablePWMOutputs();
    DisablePWM_TBCLK();
    InitPWM_MEP_SFO();

    InitPWMModule(PWM_Modules.PWM_Regs[0], PWM_FREQ, 0, MasterPWM, 0, NO_COMPLEMENTARY, 0);


	/* Initialization of GPIOs */

	EALLOW;

	INIT_DEBUG_GPIO1;		// Debug GPIO's

	GpioG1CtrlRegs.GPAMUX1.all = 0x0000;
	GpioG1DataRegs.GPACLEAR.all = 0x0000FFFF; // PWM1 to PWM16 as GPDO
	GpioG1CtrlRegs.GPADIR.all = 0x0000FFFF;

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
	IPC_CtoM_Msg.SamplesBuffer.BufferBusy = Buffer_Idle;
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
	static Uint16 i;
	static float temp[4];

	SET_DEBUG_GPIO1;

	/*temp[0] = 0.0;
	temp[1] = 0.0;
	temp[2] = 0.0;
	temp[3] = 0.0;*/

	CLEAR_DEBUG_GPIO1;

	while(!McbspaRegs.SPCR1.bit.RRDY){}

	SET_DEBUG_GPIO1;
	//CLEAR_DEBUG_GPIO1;

	for(i = 0; i < HRADCs_Info.n_HRADC_boards; i++)
	{
		temp[i] = (float) *(HRADCs_Info.HRADC_boards[i].SamplesBuffer++);
	}

	//SET_DEBUG_GPIO1

	HRADCs_Info.HRADC_boards[0].SamplesBuffer = buffers_HRADC.buffer_0;
	HRADCs_Info.HRADC_boards[1].SamplesBuffer = buffers_HRADC.buffer_1;
	HRADCs_Info.HRADC_boards[2].SamplesBuffer = buffers_HRADC.buffer_2;
	HRADCs_Info.HRADC_boards[3].SamplesBuffer = buffers_HRADC.buffer_3;

	temp[0] -= *(HRADCs_Info.HRADC_boards[0].offset);
	temp[0] *= *(HRADCs_Info.HRADC_boards[0].gain);

	temp[1] -= *(HRADCs_Info.HRADC_boards[1].offset);
	temp[1] *= *(HRADCs_Info.HRADC_boards[1].gain);

	temp[2] -= *(HRADCs_Info.HRADC_boards[2].offset);
	temp[2] *= *(HRADCs_Info.HRADC_boards[2].gain);

	temp[3] -= *(HRADCs_Info.HRADC_boards[3].offset);
	temp[3] *= *(HRADCs_Info.HRADC_boards[3].gain);

	DP_Framework.NetSignals[0] = temp[0];
	DP_Framework.NetSignals[1] = temp[1];
	DP_Framework.NetSignals[2] = temp[2];
	DP_Framework.NetSignals[3] = temp[3];

	WriteBuffer(&IPC_CtoM_Msg.SamplesBuffer, DP_Framework.NetSignals[IPC_MtoC_Msg.HRADCConfig.ID]);

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

