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
#include "Test_HRADC.h"

#define BUFFER_SIZE 2048

/*
 * Prototype statements for functions found within this file.
 */

#pragma CODE_SECTION(isr_ePWM_CTR_ZERO, "ramfuncs");
#pragma CODE_SECTION(isr_ePWM_CTR_ZERO_1st, "ramfuncs");
#pragma CODE_SECTION(isr_DMA_CH2, "ramfuncs");

static interrupt void isr_ePWM_CTR_ZERO(void);
static interrupt void isr_ePWM_CTR_ZERO_1st(void);
static interrupt void isr_DMA_CH2(void);
static interrupt void isr_SoftInterlock(void);
static interrupt void isr_HardInterlock(void);

void main_Test_HRADC(void);

static void InitPeripheralsDrivers(void);
static void InitControllers(void);
static void InitInterruptions(void);
static void PS_turnOn(void);
static void PS_turnOff(void);

volatile Uint16 SamplingEnable;
volatile Uint16 SendCommand;
volatile Uint16 ID;
volatile Uint16 enableHeater;
volatile Uint16 enableRailsMonitor;

volatile Uint32 valorCounter;
volatile enum_AN_INPUT AnalogInput;

volatile float HRADC0;
volatile float HRADC1;
volatile float HRADC2;
volatile float HRADC3;

void main_Test_HRADC(void)
{
	ID = 0;
	AnalogInput = Iin_bipolar;
	enableHeater = HEATER_DISABLE;
	enableRailsMonitor = RAILS_DISABLE;

	SamplingEnable = 0;
	SendCommand = 0;

	InitPeripheralsDrivers();
	InitControllers();
	InitInterruptions();

	while(1)
	{
		if(SendCommand){

		   SendCommand = 0;
		   Config_HRADC_board(HRADCs_Info.HRADC_boards[ID], AnalogInput, enableHeater, enableRailsMonitor);
		}

		if(SamplingEnable){

		   start_DMA();
		   EnablePWMOutputs();
		   EnablePWM_TBCLK();

		   while(SamplingEnable){}

		   DisablePWMOutputs();
		   DisablePWM_TBCLK();

		   DELAY_US(2);
		   stop_DMA();

		}

		TunningPWM_MEP_SFO();
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

    Init_DMA_McBSP_nBuffers(N_HRADC_BOARDS, DECIMATION_FACTOR);

	Init_SPIMaster_McBSP();
    Init_SPIMaster_Gpio();
    InitMcbspa20bit();

    HRADCs_Info.HRADC_boards[0] = &HRADC0_board;
    HRADCs_Info.HRADC_boards[1] = &HRADC1_board;
    HRADCs_Info.HRADC_boards[2] = &HRADC2_board;
    HRADCs_Info.HRADC_boards[3] = &HRADC3_board;

    Init_HRADC_Info(HRADCs_Info.HRADC_boards[0], 0, DECIMATION_FACTOR, &buffers_HRADC.buffer_0[0], TRANSDUCER_0_GAIN, HRADC_0_R_BURDEN);
    Init_HRADC_Info(HRADCs_Info.HRADC_boards[1], 1, DECIMATION_FACTOR, &buffers_HRADC.buffer_1[0], TRANSDUCER_1_GAIN, HRADC_1_R_BURDEN);
    Init_HRADC_Info(HRADCs_Info.HRADC_boards[2], 2, DECIMATION_FACTOR, &buffers_HRADC.buffer_2[0], TRANSDUCER_2_GAIN, HRADC_2_R_BURDEN);
    Init_HRADC_Info(HRADCs_Info.HRADC_boards[3], 3, DECIMATION_FACTOR, &buffers_HRADC.buffer_3[0], TRANSDUCER_3_GAIN, HRADC_3_R_BURDEN);

    Config_HRADC_board(HRADCs_Info.HRADC_boards[0], TRANSDUCER_0_OUTPUT_TYPE, HEATER_DISABLE, RAILS_DISABLE);
    //Config_HRADC_board(HRADCs_Info.HRADC_boards[1], TRANSDUCER_1_OUTPUT_TYPE, HEATER_DISABLE, RAILS_DISABLE);
    /*Config_HRADC_board(HRADCs_Info.HRADC_boards[2], TRANSDUCER_2_OUTPUT_TYPE, HEATER_DISABLE, RAILS_DISABLE);
    Config_HRADC_board(HRADCs_Info.HRADC_boards[3], TRANSDUCER_3_OUTPUT_TYPE, HEATER_DISABLE, RAILS_DISABLE);*/

    AverageFilter = 1.0/((float) DECIMATION_FACTOR);

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
	ConfigCpuTimer(&CpuTimer0, C28_FREQ_MHZ, 1000000);
	CpuTimer0Regs.TCR.bit.TIE = 0;
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
static interrupt void isr_ePWM_CTR_ZERO(void)
{
	static Uint16 i;
	static float temp0, temp1, temp2, temp3;

	StartCpuTimer0();
	SET_DEBUG_GPIO1;

	temp0 = 0.0;
	temp1 = 0.0;
	temp2 = 0.0;
	temp3 = 0.0;

	for(i = 0; i < DECIMATION_FACTOR; i++)
	{
		temp0 += (float) *(HRADCs_Info.HRADC_boards[0]->SamplesBuffer++);
		temp1 += (float) *(HRADCs_Info.HRADC_boards[1]->SamplesBuffer++);
		temp2 += (float) *(HRADCs_Info.HRADC_boards[2]->SamplesBuffer++);
		temp3 += (float) *(HRADCs_Info.HRADC_boards[3]->SamplesBuffer++);
	}

	temp0 *= AverageFilter;
	temp1 *= AverageFilter;
	temp2 *= AverageFilter;
	temp3 *= AverageFilter;

	HRADCs_Info.HRADC_boards[0]->SamplesBuffer = buffers_HRADC.buffer_0;
	HRADCs_Info.HRADC_boards[1]->SamplesBuffer = buffers_HRADC.buffer_1;
	HRADCs_Info.HRADC_boards[2]->SamplesBuffer = buffers_HRADC.buffer_2;
	HRADCs_Info.HRADC_boards[3]->SamplesBuffer = buffers_HRADC.buffer_3;

	temp0 -= *(HRADCs_Info.HRADC_boards[0]->offset);
	temp0 *= *(HRADCs_Info.HRADC_boards[0]->gain);

	temp1 -= *(HRADCs_Info.HRADC_boards[1]->offset);
	temp1 *= *(HRADCs_Info.HRADC_boards[1]->gain);

	temp2 -= *(HRADCs_Info.HRADC_boards[2]->offset);
	temp2 *= *(HRADCs_Info.HRADC_boards[2]->gain);

	temp3 -= *(HRADCs_Info.HRADC_boards[3]->offset);
	temp3 *= *(HRADCs_Info.HRADC_boards[3]->gain);

	HRADC0 = temp0;
	HRADC1 = temp1;
	HRADC2 = temp2;
	HRADC3 = temp3;

	WriteBuffer(&IPC_CtoM_Msg.SamplesBuffer, HRADC0);
	/*WriteBuffer(&IPC_CtoM_Msg.SamplesBuffer, HRADC1);
	WriteBuffer(&IPC_CtoM_Msg.SamplesBuffer, HRADC2);
	WriteBuffer(&IPC_CtoM_Msg.SamplesBuffer, HRADC3);*/


	for(i = 0; i < PWM_Modules.N_modules; i++)
	{
		PWM_Modules.PWM_Regs[i]->ETCLR.bit.INT = 1;
	}

	StopCpuTimer0();
	valorCounter = ReadCpuTimer0Counter();
	ReloadCpuTimer0();

	CLEAR_DEBUG_GPIO1;

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

static __interrupt void isr_DMA_CH2(void)
{
	SET_DEBUG_GPIO1;
	EALLOW;
	DmaRegs.CH3.CONTROL.bit.RUN = 1;
	EDIS;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
    CLEAR_DEBUG_GPIO1;
}

static interrupt void isr_SoftInterlock(void)
{
}

static interrupt void isr_HardInterlock(void)
{
}

static void PS_turnOn(void)
{
}

static void PS_turnOff(void)
{
}
