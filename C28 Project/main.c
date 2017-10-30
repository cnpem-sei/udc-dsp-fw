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

extern void main_FBP_100kHz(void);
extern void main_FAC_ACDC_10kHz(void);
extern void main_FAC_DCDC_20kHz(void);
extern void main_FAC_Full_ACDC_10kHz(void);
extern void main_FAC_Full_DCDC_20kHz(void);
extern void main_FAP_ACDC(void);
extern void main_FAP_DCDC_20kHz(void);
extern void main_Test_HRPWM(void);
extern void main_Test_HRADC(void);
extern void main_Jiga_HRADC(void);
extern void main_FAP_DCDC_15kHz_225A(void);
extern void main_FBPx4_100kHz(void);
extern void main_FAP_6U_DCDC_20kHz(void);
extern void main_FBP_FAC_UFJF(void);

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

		// Set 50% duty cycle
		SetPWMDutyCycle_ChA(&EPwm9Regs, 0.01);
		SetPWMDutyCycle_ChA(&EPwm11Regs, 0.5);

		EDIS;
	}

	while(1)
	{
		switch(IPC_MtoC_Msg.PSModule.Model)
		//switch(PS_MODEL)
		{
			case FBP_100kHz:
			{
				//main_FBP_100kHz();
				//main_Test_BCB_Board();
				break;
			}

			case FBP_Parallel_100kHz:
			{
				//main_FBP_Parallel_100kHz();
				break;
			}

			case FAC_ACDC_10kHz:
			{
				//main_FAC_ACDC_10kHz();
				break;
			}

			case FAC_DCDC_20kHz:
			{
				//main_FAC_DCDC_20kHz();
				break;
			}

			case FAC_Full_ACDC_10kHz:
			{
				//main_FAC_Full_ACDC_10kHz();
				break;
			}

			case FAC_Full_DCDC_20kHz:
			{
				//main_FAC_Full_DCDC_20kHz();
				break;
			}

			case FAP_ACDC:
			{
				//main_FAP_ACDC();
				break;
			}

			case FAP_DCDC_20kHz:
			{
				//main_FAP_DCDC_20kHz();
				break;
			}

			case TEST_HRPWM:
			{
				//main_Test_HRPWM();
				break;
			}

			case TEST_HRADC:
			{
				//main_Test_HRADC();
				break;
			}

			case JIGA_HRADC:
			{
				//main_Jiga_HRADC();
			}

			case FAP_DCDC_15kHz_225A:
			{
				//main_FAP_DCDC_15kHz_225A();
			}

			case FBPx4_100kHz:
			{
				main_FBPx4_100kHz();
			}

			case FAP_6U_DCDC_20kHz:
			{
				main_FAP_6U_DCDC_20kHz();
			}

			case FBP_FAC_UFJF:
			{
			    main_FBP_FAC_UFJF();
			}

			default:
			{
				break;
			}
		}

	}
}
