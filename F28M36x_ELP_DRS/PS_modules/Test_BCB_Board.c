/*
 * 		FILE: 		FAP_DCDC_20kHz.c
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
#include "Test_BCB_Board.h"

/*
 * Timeouts
 */

#define TIMEOUT_uS_DCLINK_CONTACTOR		3000000


// Prototype statements for functions found within this file.

#pragma CODE_SECTION(isr_ePWM_CTR_ZERO, "ramfuncs");
#pragma CODE_SECTION(isr_ePWM_CTR_ZERO_1st, "ramfuncs");
#pragma CODE_SECTION(sciaRxFifoIsr, "ramfuncs");
#pragma CODE_SECTION(PS_turnOn, "ramfuncs");
#pragma CODE_SECTION(PS_turnOff, "ramfuncs");
#pragma CODE_SECTION(Set_SoftInterlock, "ramfuncs");
#pragma CODE_SECTION(Set_HardInterlock, "ramfuncs");
#pragma CODE_SECTION(isr_SoftInterlock, "ramfuncs");
#pragma CODE_SECTION(isr_HardInterlock, "ramfuncs");
#pragma CODE_SECTION(ResetControllers, "ramfuncs");

static interrupt void isr_ePWM_CTR_ZERO(void);
static interrupt void isr_ePWM_CTR_ZERO_1st(void);
static interrupt void sciaRxFifoIsr(void);

void main_Test_BCB_Board(void);

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

Uint16 rdataA[2];

void main_Test_BCB_Board(void)
{
	Uint16 i;

	InitPeripheralsDrivers();

	InitControllers();

	InitInterruptions();

	ConfigCpuTimer(&CpuTimer0, C28_FREQ_MHZ, 10000000);
	CpuTimer1Regs.TCR.all = 0xC000;

	stop_DMA();
	DELAY_US(5);
	start_DMA();
	EnablePWM_TBCLK();

	EnablePWMOutputs();

	for(i = 0; i < PWM_Modules.N_modules; i++)
	{
		SetPWMDutyCycle_ChA(PWM_Modules.PWM_Regs[i], 0.9);
		//DELAY_US(500000);
		SetPWMDutyCycle_ChB(PWM_Modules.PWM_Regs[i], 0.9);
		//DELAY_US(500000);
	}

	/*for(i = 0; i < PWM_Modules.N_modules; i++)
	{
		SetPWMDutyCycle_ChA(PWM_Modules.PWM_Regs[i], 0.0);
		DELAY_US(500000);
		SetPWMDutyCycle_ChB(PWM_Modules.PWM_Regs[i], 0.0);
		DELAY_US(500000);
	}*/

	while(1)
	{
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

	Init_DMA_McBSP_nBuffers(N_HRADC_BOARDS, DECIMATION_FACTOR, HRADC_SPI_CLK);

	Init_SPIMaster_McBSP(HRADC_SPI_CLK);
	Init_SPIMaster_Gpio();
	InitMcbspa20bit();

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
	PWM_Modules.PWM_Regs[0] = &EPwm1Regs;
	PWM_Modules.PWM_Regs[1] = &EPwm2Regs;
	PWM_Modules.PWM_Regs[2] = &EPwm3Regs;
	PWM_Modules.PWM_Regs[3] = &EPwm4Regs;
	PWM_Modules.PWM_Regs[4] = &EPwm5Regs;
	PWM_Modules.PWM_Regs[5] = &EPwm6Regs;
	PWM_Modules.PWM_Regs[6] = &EPwm7Regs;
	PWM_Modules.PWM_Regs[7] = &EPwm8Regs;

	DisablePWMOutputs();
	DisablePWM_TBCLK();
	InitPWM_MEP_SFO();

	InitPWMModule(PWM_Modules.PWM_Regs[0], PWM_FREQ, 0, MasterPWM, 0, NO_COMPLEMENTARY, 0.0);
	InitPWMModule(PWM_Modules.PWM_Regs[1], PWM_FREQ, 0, SlavePWM, 0, NO_COMPLEMENTARY, 0.0);
	InitPWMModule(PWM_Modules.PWM_Regs[2], PWM_FREQ, 0, SlavePWM, 0, NO_COMPLEMENTARY, 0.0);
	InitPWMModule(PWM_Modules.PWM_Regs[3], PWM_FREQ, 0, SlavePWM, 0, NO_COMPLEMENTARY, 0.0);
	InitPWMModule(PWM_Modules.PWM_Regs[4], PWM_FREQ, 0, SlavePWM, 0, NO_COMPLEMENTARY, 0.0);
	InitPWMModule(PWM_Modules.PWM_Regs[5], PWM_FREQ, 0, SlavePWM, 0, NO_COMPLEMENTARY, 0.0);
	InitPWMModule(PWM_Modules.PWM_Regs[6], PWM_FREQ, 0, SlavePWM, 0, NO_COMPLEMENTARY, 0.0);
	InitPWMModule(PWM_Modules.PWM_Regs[7], PWM_FREQ, 0, SlavePWM, 0, NO_COMPLEMENTARY, 0.0);

	InitEPwm1Gpio();
	InitEPwm2Gpio();
	InitEPwm3Gpio();
	InitEPwm4Gpio();
	InitEPwm5Gpio();
	InitEPwm6Gpio();
	InitEPwm7Gpio();
	InitEPwm8Gpio();

	/* Initialization of SCI interface */
	scia_fifo_init();

	/* Initialization of GPIOs */

	EALLOW;

	if(UDC_V2_1)
	{
		GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 0;	// GPIO29: INT_C28
		GpioCtrlRegs.GPADIR.bit.GPIO29 = 0;

		GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 0;	// GPIO55: INT_GENERAL/SYNC_IN
		GpioCtrlRegs.GPBDIR.bit.GPIO55 = 0;

		GpioCtrlRegs.GPDMUX2.bit.GPIO126 = 0;	// GPDI1
		GpioCtrlRegs.GPDDIR.bit.GPIO126 = 0;

		GpioCtrlRegs.GPDMUX2.bit.GPIO127 = 0;	// GPDI2
		GpioCtrlRegs.GPDDIR.bit.GPIO127 = 0;

		GpioCtrlRegs.GPDMUX2.bit.GPIO124 = 0;	// GPDI3
		GpioCtrlRegs.GPDDIR.bit.GPIO124 = 0;

		GpioCtrlRegs.GPDMUX2.bit.GPIO125 = 0;	// GPDI4
		GpioCtrlRegs.GPDDIR.bit.GPIO125 = 0;
	}

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

}

static void InitControllers(void)
{

	/* Initialization of IPC module */
	InitIPC(&PS_turnOn, &PS_turnOff, &isr_SoftInterlock, &isr_HardInterlock);

	/* Initiaization of DP Framework */
	InitDP_Framework(&DP_Framework, &(IPC_CtoM_Msg.PSModule.IRef));

	ResetControllers();
}

static void ResetControllers(void)
{
	SetPWMDutyCycle_ChA(PWM_Modules.PWM_Regs[0], 0.0);
	SetPWMDutyCycle_ChB(PWM_Modules.PWM_Regs[0], 0.0);
	SetPWMDutyCycle_ChA(PWM_Modules.PWM_Regs[1], 0.0);
	SetPWMDutyCycle_ChB(PWM_Modules.PWM_Regs[1], 0.0);
	SetPWMDutyCycle_ChA(PWM_Modules.PWM_Regs[2], 0.0);
	SetPWMDutyCycle_ChB(PWM_Modules.PWM_Regs[2], 0.0);
	SetPWMDutyCycle_ChA(PWM_Modules.PWM_Regs[3], 0.0);
	SetPWMDutyCycle_ChB(PWM_Modules.PWM_Regs[3], 0.0);
	SetPWMDutyCycle_ChA(PWM_Modules.PWM_Regs[4], 0.0);
	SetPWMDutyCycle_ChB(PWM_Modules.PWM_Regs[4], 0.0);
	SetPWMDutyCycle_ChA(PWM_Modules.PWM_Regs[5], 0.0);
	SetPWMDutyCycle_ChB(PWM_Modules.PWM_Regs[5], 0.0);
	SetPWMDutyCycle_ChA(PWM_Modules.PWM_Regs[6], 0.0);
	SetPWMDutyCycle_ChB(PWM_Modules.PWM_Regs[6], 0.0);
	SetPWMDutyCycle_ChA(PWM_Modules.PWM_Regs[7], 0.0);
	SetPWMDutyCycle_ChB(PWM_Modules.PWM_Regs[7], 0.0);

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
	PieVectTable.EPWM1_INT = &isr_ePWM_CTR_ZERO_1st;
	PieVectTable.SCIRXINTA = &sciaRxFifoIsr;
	EDIS;

	PieCtrlRegs.PIEIER3.bit.INTx1 = 1;  // ePWM1
    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;	// SCIRXINTA

    //PieCtrlRegs.PIECTRL.bit.ENPIE = 1;

	EnablePWMInterrupt(PWM_Modules.PWM_Regs[0]);

	IER |= M_INT1;
	IER |= M_INT3;
	IER |= M_INT9;
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
	static Uint16 i;
	static float temp0, temp1, temp2, temp3;

	//StartCpuTimer0();
	SET_DEBUG_GPIO1;

	temp0 = 0.0;
	temp1 = 0.0;
	temp2 = 0.0;
	temp3 = 0.0;

	while(!McbspaRegs.SPCR1.bit.RRDY){}

	for(i = 0; i < DECIMATION_FACTOR; i++)
	{
		temp0 += (float) *(HRADCs_Info.HRADC_boards[0]->SamplesBuffer++);
		temp1 += (float) *(HRADCs_Info.HRADC_boards[1]->SamplesBuffer++);
		temp2 += (float) *(HRADCs_Info.HRADC_boards[2]->SamplesBuffer++);
		//temp3 += (float) *(HRADCs_Info.HRADC_boards[3]->SamplesBuffer++);
	}

	temp0 *= AverageFilter;
	temp1 *= AverageFilter;
	temp2 *= AverageFilter;
	temp3 *= AverageFilter;

	HRADCs_Info.HRADC_boards[0]->SamplesBuffer = buffers_HRADC.buffer_0;
	HRADCs_Info.HRADC_boards[1]->SamplesBuffer = buffers_HRADC.buffer_1;
	HRADCs_Info.HRADC_boards[2]->SamplesBuffer = buffers_HRADC.buffer_2;
	HRADCs_Info.HRADC_boards[3]->SamplesBuffer = buffers_HRADC.buffer_3;

	DP_Framework.NetSignals[4] = temp0;
	DP_Framework.NetSignals[5] = temp1;
	DP_Framework.NetSignals[6] = temp2;
	DP_Framework.NetSignals[7] = temp3;

	WriteBuffer(&IPC_CtoM_Msg.SamplesBuffer, temp0);
	/*WriteBuffer(&IPC_CtoM_Msg.SamplesBuffer, temp1);
	WriteBuffer(&IPC_CtoM_Msg.SamplesBuffer, temp2);
	WriteBuffer(&IPC_CtoM_Msg.SamplesBuffer, temp3);*/

	for(i = 0; i < PWM_Modules.N_modules; i++)
	{
		PWM_Modules.PWM_Regs[i]->ETCLR.bit.INT = 1;
	}

	CLEAR_DEBUG_GPIO1;
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

	EnablePWMInterrupt(PWM_Modules.PWM_Regs[1]);

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
	EnablePWMOutputs();
}

static void PS_turnOff(void)
{
	DisablePWMOutputs();
}

void scia_fifo_init()
{
	EALLOW;

	GpioCtrlRegs.GPDMUX2.bit.GPIO117 = 0;
	GpioDataRegs.GPDCLEAR.bit.GPIO117 = 1;		// GPIO117: SCI_RD
	GpioCtrlRegs.GPDDIR.bit.GPIO117 = 1;

	GpioCtrlRegs.GPDMUX2.bit.GPIO118 = 2;		// GPIO118: SCITXDA

	GpioCtrlRegs.GPDMUX2.bit.GPIO119 = 2;		// GPIO119: SCIRXDA
	GpioCtrlRegs.GPDQSEL2.bit.GPIO119 = 3;

    EDIS;

	// SCICCR
    //    Character length 8 bits
    //    1 stop bit
    //    Parity not enabled
    //    Idle-line mode protocol selected
    SciaRegs.SCICCR.bit.SCICHAR =(0x0008-1);
    SciaRegs.SCICCR.bit.STOPBITS = 0;
    SciaRegs.SCICCR.bit.PARITYENA = 0;
    SciaRegs.SCICCR.bit.ADDRIDLE_MODE = 0;

    // SCICTL1:
    //    Enable RX and TX
    //    Leave SLEEP mode disabled
    //    Leave RX Err disabled
    //    Leave transmitter wakeup feature disabled
    // SCICTL2:
    //    Enable RXRDY/BRKDT interrupt
    SciaRegs.SCICTL1.bit.RXENA = 1;
    SciaRegs.SCICTL1.bit.TXENA = 1;
    SciaRegs.SCICTL2.bit.TXINTENA = 0;
    SciaRegs.SCICTL2.bit.RXBKINTENA = 1;
    SciaRegs.SCIHBAUD = 0x0000;
    SciaRegs.SCILBAUD = SCI_PRD;

    // Transmit FIFO reset
    // 2 level transmit FIFO
    // Disable transmit FIFO interrupt
    // Enable SCI FIFO enhancements
    // Receive FIFO reset
    // 2 level receive FIFO
    // Enable receive FIFO interrupt
    //
    SciaRegs.SCIFFTX.all=0xC002;
    SciaRegs.SCIFFRX.all=0x0022;

    // Release the SCI from reset
    // Release the FIFOs from reset
    SciaRegs.SCICTL1.all = 0x0023;
    SciaRegs.SCIFFTX.bit.TXFIFOXRESET=1;
    SciaRegs.SCIFFRX.bit.RXFIFORESET=1;

}

//*****************************************************************************
// SCI receive from FIFO interrupt service routine
//*****************************************************************************
static interrupt void sciaRxFifoIsr(void)
{
    Uint16 i;

    // Read 2 characters
    rdataA[0]=SciaRegs.SCIRXBUF.all;
    rdataA[1]=SciaRegs.SCIRXBUF.all;

    GpioDataRegs.GPDSET.bit.GPIO117 = 1;

    SciaRegs.SCITXBUF = rdataA[0];
    SciaRegs.SCITXBUF = rdataA[1];

    DELAY_US(1000);

    GpioDataRegs.GPDCLEAR.bit.GPIO117 = 1;

    //
    // Clear Interrupt flag
    // Issue PIE acknowledge to enable more interrupts from this group
    SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;
    PieCtrlRegs.PIEACK.all |= M_INT9;
}
