/*
 * 		FILE: 		Test_HRPWM.c
 * 		PROJECT: 	DRS v2.0
 * 		CREATION:	03/07/2016
 * 		MODIFIED:	03/07/2016
 *
 * 		AUTHOR: 	Gabriel O. B.  (LNLS/ELP)
 *
 * 		DESCRIPTION:
 *
 *		TODO:
 */

#include "F28M36x_ELP_DRS.h"
#include "Test_HRPWM.h"

#define BUFFER_SIZE 2048

/*
 * Prototype statements for functions found within this file.
 */

void main_Test_HRPWM(void);

static void InitPeripheralsDrivers(void);
static Uint16 inc, incAux, incEnable, idxBuffer;
static Uint16 incBuffer[BUFFER_SIZE];

void main_Test_HRPWM(void)
{

	inc = 0;
	incAux = 0;
	incEnable = 0;
	idxBuffer = 0;

	DELAY_US(1000000);

	InitPeripheralsDrivers();

	SetPWMDutyCycle_ChA(&EPwm1Regs,0.5);
	SetPWMDutyCycle_ChB(&EPwm1Regs,0.5);

	EnablePWMOutputs();
	EnablePWM_TBCLK();

	while(1)
	{

		/*for(inc = 0.5; inc < 0.514625	; inc += 0.001125)
		{
			SetPWMDutyCycle_ChA(&EPwm1Regs,inc);
			DELAY_US(1000000);
		}*/
		StartCpuTimer0();
		//inc = (MEP_ScaleFactor - 4) << 8;
		//incAux = MEP_ScaleFactor << 8;
		inc = (43 - 5) << 8;
		incAux = 43 << 8;
		incBuffer[idxBuffer++] = MEP_ScaleFactor;

		if(idxBuffer == BUFFER_SIZE)
		{
			idxBuffer = 0;
		}

		if(incEnable)
		{
			while(inc <= incAux)
			{
				EPwm1Regs.CMPAM2.half.CMPAHR = inc;
				inc = inc + 0x0100;
				DELAY_US(1000000);
			}

			inc = 0x0000;

			EPwm1Regs.CMPAM2.half.CMPA++;

			while(inc < 0x0500)
			{
				EPwm1Regs.CMPAM2.half.CMPAHR = inc;
				inc = inc + 0x0100;
				DELAY_US(1000000);
			}

			EPwm1Regs.CMPAM2.half.CMPA--;
		}

		TunningPWM_MEP_SFO();

	}
}

/*
 * Initialization of peripheral drivers:
 *
 * 		- PWM modules
 */

static void InitPeripheralsDrivers(void)
{
    /* Initialization of PWM modules */

    DisablePWMOutputs();
    DisablePWM_TBCLK();
    InitPWM_MEP_SFO();

    PWM_Modules.N_modules = 1;
    PWM_Modules.PWM_Regs[0] = &EPwm1Regs;
    //PWM_Modules.PWM_Regs[1] = &EPwm2Regs;

    InitPWMModule(PWM_Modules.PWM_Regs[0], PWM_FREQ, 0, MasterPWM, 0, NO_COMPLEMENTARY, 0);
    //InitPWMModule(PWM_Modules.PWM_Regs[1], PWM_FREQ, 1, SlavePWM, 180, COMPLEMENTARY, DEAD_TIME);

    InitEPwm1Gpio();
    //InitEPwm2Gpio();

	/* Initialization of GPIOs */

	EALLOW;

	GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 0;
	GpioDataRegs.GPCCLEAR.bit.GPIO65 = 1;		// GPIO DC-Link Relay (FBP v2.0)
	GpioCtrlRegs.GPCDIR.bit.GPIO65 = 1;

	GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 0;
	GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;		// GPIO Output Relay (FBP v2.0)
	GpioCtrlRegs.GPCDIR.bit.GPIO67 = 1;

	GpioCtrlRegs.GPDMUX1.bit.GPIO111 = 0;
	GpioDataRegs.GPDSET.bit.GPIO111 = 1;		// GPIO Debug 1
	GpioCtrlRegs.GPDDIR.bit.GPIO111 = 1;

	GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO14 = 1;			// Auxiliar GPIO for which GPTRIP1 is selected
	GpioDataRegs.GPASET.bit.GPIO14 = 1;
	EDIS;

	/* Initialization of timers */

    InitCpuTimers();
	ConfigCpuTimer(&CpuTimer0, C28_FREQ_MHZ, 1000000);
	CpuTimer0Regs.TCR.bit.TIE = 0;
}
