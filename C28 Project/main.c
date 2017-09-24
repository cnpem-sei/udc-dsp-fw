/*
 * 		FILE: 		main_FAC_ACDC_v2_0_2.c
 * 		PROJECT: 	DRS v2.0
 * 		CREATION:	28/10/2015
 * 		MODIFIED:
 *
 * 		AUTHOR: 	Gabriel O. B.  (LNLS/ELP)
 *
 * 		DESCRIPTION: Firmware for control of AC/DC stage of prototype FAC v2.0
 *
 *
 *		TODO:
 *				- Incluir teste da flag do ARM que libera o C28 para inicialização
 */

#include <string.h>
#include "F28M36x_ELP_DRS.h"

/*extern Uint16 RamfuncsLoadStart_RAML3;
extern Uint16 RamfuncsLoadSize_RAML3;
extern Uint16 RamfuncsRunStart_RAML3;*/

extern void main_Jiga_HRADC_v2_1(void);

void main(void)
{
	// Initialize the Control System:
	// Enable peripheral clocks
	// This example function is found in the F28M36x_SysCtrl.c file.
	InitSysCtrl();

	// Copy time critical code and Flash setup code to RAM
	// This includes the following functions:  InitFlash();
	// The  RamfuncsLoadStart, RamfuncsLoadSize, and RamfuncsRunStart
	// symbols are created by the linker. Refer to the device .cmd file.
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
    //memcpy(&RamfuncsRunStart_RAML3, &RamfuncsLoadStart_RAML3, (size_t)&RamfuncsLoadSize_RAML3);

	// Call Flash Initialization to setup flash waitstates
	// This function must reside in RAM
	InitFlash();

	// Clear all interrupts and initialize PIE vector table:
	// Disable CPU interrupts
	DINT;

	// Initialize the PIE control registers to their default state.
	// The default state is all PIE interrupts disabled and flags
	// are cleared.
	// This function is found in the F28M36x_PieCtrl.c file.
	InitPieCtrl();

	// Disable CPU interrupts and clear all CPU interrupt flags:
	IER = 0x0000;
	IFR = 0x0000;

	// Initialize the PIE vector table with pointers to the shell Interrupt
	// Service Routines (ISR).
	// This will populate the entire table, even if the interrupt
	// is not used in this example.  This is useful for debug purposes.
	// The shell ISR routines are found in F28M36x_DefaultIsr.c.
	// This function is found in F28M36x_PieVect.c.
	InitPieVectTable();


	/*
	 * 	TODO: Antes de realizar esta inicialização, garantir que ARM já fez a sua
	 */

	if(UDC_V2_1)
	{
		PWM_Modules.N_modules = 8;

		DisablePWMOutputs();
		DisablePWM_TBCLK();

		InitPWMModule(&EPwm9Regs, BUZZER_PITCH_FREQ, 0, MasterPWM, 0, NO_COMPLEMENTARY, 0);
		InitPWMModule(&EPwm11Regs, 0.0, 0, MasterPWM, 0, NO_COMPLEMENTARY, 0);

		EALLOW;

		EPwm11Regs.TBPRD = BUZZER_MOD_PERIOD;					// Set frequency manually

		EPwm11Regs.TBCTL.bit.CLKDIV = BUZZER_MOD_CLKDIV;		// Set clock pre-scaler, due to
		EPwm11Regs.TBCTL.bit.HSPCLKDIV = BUZZER_MOD_HSPCLKDIV;	// very low frequency

		GpioCtrlRegs.GPEMUX1.bit.GPIO128 = 1;   				// Configure GPIO128 as EPWM9A
		GpioCtrlRegs.GPEMUX1.bit.GPIO132 = 1; 	  				// Configure GPIO132 as EPWM11A

		// Disable trip
		EPwm9Regs.TZSEL.bit.OSHT1 = 0;
		EPwm11Regs.TZSEL.bit.OSHT1 = 0;

		// Enable PWM outputs
		EPwm9Regs.TZCLR.bit.OST = 1;
		EPwm11Regs.TZCLR.bit.OST = 1;

		// Set 10% duty cycle
		SetPWMDutyCycle_ChA(&EPwm9Regs, 0.01);
		SetPWMDutyCycle_ChA(&EPwm11Regs, 0.5);

		EDIS;
	}

	/* Initialization of GPIOs */

	EALLOW;

	INIT_DEBUG_GPIO1;		// Debug GPIO's

	GpioG1CtrlRegs.GPAMUX1.all = 0x0000;
	GpioG1DataRegs.GPACLEAR.all = 0x0000FFFF; // PWM1 to PWM16 as GPDO
	GpioG1CtrlRegs.GPADIR.all = 0x0000FFFF;

	EDIS;

	DELAY_US(1000000);

	SendIpcFlag(ENABLE_HRADC_BOARDS);

	main_Jiga_HRADC_v2_1();

	while(1){}
}
