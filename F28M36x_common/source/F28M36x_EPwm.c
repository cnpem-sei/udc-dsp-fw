//###########################################################################
// FILE:   F28M36x_EPwm.c
// TITLE:  F28M36x EPwm Initialization & Support Functions.
//###########################################################################
// $TI Release: F28M36x Support Library v202 $
// $Release Date: Tue Apr  8 12:36:34 CDT 2014 $
//###########################################################################

#include "F28M36x_Device.h"     // F28M36x Headerfile Include File
#include "F28M36x_Examples.h"   // F28M36x Examples Include File

//---------------------------------------------------------------------------
// InitEPwm:
//---------------------------------------------------------------------------
// This function initializes the EPwm(s) to a known state.
void InitEPwm(void)
{
    // Initialize EPwm1/2/3/4/5/6/7/8/9

    //tbd...

}

//---------------------------------------------------------------------------
// Example: InitEPwmGpio:
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as EPwm pins
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.

void InitEPwmGpio(void)
{
    InitEPwm1Gpio();
    InitEPwm2Gpio();
    InitEPwm3Gpio();
    InitEPwm4Gpio();
    InitEPwm5Gpio();
    InitEPwm6Gpio();
    InitEPwm7Gpio();
    InitEPwm8Gpio();
    InitEPwm9Gpio();
}

void InitEPwm1Gpio(void)
{
    EALLOW;

/* Enable internal pull-up for the selected pins */

// Pull-up enable should be set in the
// GPIO_O_PUR register in the M3 code.

/* Configure EPWM-1 pins using GPIO regs*/

// This specifies which of the possible GPIO pins will be EPWM1 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B

    EDIS;
}

void InitEPwm2Gpio(void)
{
    EALLOW;

/* Enable internal pull-up for the selected pins */

// Pull-up enable should be set in the
// GPIO_O_PUR register in the M3 code.

/* Configure EPwm-2 pins using GPIO regs*/

// This specifies which of the possible GPIO pins will be EPWM2 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO3 as EPWM2B

    EDIS;
}

void InitEPwm3Gpio(void)
{
    EALLOW;

/* Enable internal pull-up for the selected pins */

// Pull-up enable should be set in the
// GPIO_O_PUR register in the M3 code.

/* Configure EPwm-3 pins using GPIO regs*/

// This specifies which of the possible GPIO pins will be EPWM3 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM3B

    EDIS;
}

void InitEPwm4Gpio(void)
{
    EALLOW;

/* Enable internal pull-up for the selected pins */

// Pull-up enable should be set in the
// GPIO_O_PUR register in the M3 code.

/* Configure EPWM-4 pins using GPIO regs*/

// This specifies which of the possible GPIO pins will be EPWM4 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // Configure GPIO6 as EPWM4A
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;   // Configure GPIO7 as EPWM4B

    EDIS;
}

void InitEPwm5Gpio(void)
{
    EALLOW;

/* Enable internal pull-up for the selected pins */

// Pull-up enable should be set in the
// GPIO_O_PUR register in the M3 code.

/* Configure EPWM-5 pins using GPIO regs*/

// This specifies which of the possible GPIO pins will be EPWM5 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;   // Configure GPIO8 as EPWM5A
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;   // Configure GPIO9 as EPWM5B

    EDIS;
}

void InitEPwm6Gpio(void)
{
    EALLOW;

/* Enable internal pull-up for the selected pins */

// Pull-up enable should be set in the
// GPIO_O_PUR register in the M3 code.

/* Configure EPWM-6 pins using GPIO regs*/

// This specifies which of the possible GPIO pins will be EPWM6 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;   // Configure GPIO10 as EPWM6A
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;   // Configure GPIO11 as EPWM6B

    EDIS;
}

void InitEPwm7Gpio(void)
{
    EALLOW;

/* Enable internal pull-up for the selected pins */

// Pull-up enable should be set in the
// GPIO_O_PUR register in the M3 code.

/* Configure EPWM-7 pins using GPIO regs*/

// This specifies which of the possible GPIO pins will be EPWM7 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;   // Configure GPIO12 as EPWM7A
//  GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 3;   // Configure GPIO58 as EPWM7A
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 1;   // Configure GPIO13 as EPWM7B
//    GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 3;   // Configure GPIO59 as EPWM7B

    EDIS;
}

void InitEPwm8Gpio(void)
{
    EALLOW;

/* Enable internal pull-up for the selected pins */

// Pull-up enable should be set in the
// GPIO_O_PUR register in the M3 code.

/* Configure EPWM-8 pins using GPIO regs*/

// This specifies which of the possible GPIO pins will be EPWM8 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 1;   // Configure GPIO14 as EPWM8A
//  GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 3;   // Configure GPIO60 as EPWM8A
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 1;   // Configure GPIO15 as EPWM8B
//    GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 3;   // Configure GPIO61 as EPWM8B

    EDIS;
}

void InitEPwm9Gpio(void)
{
    EALLOW;

/* Enable internal pull-up for the selected pins */

// Pull-up enable should be set in the
// GPIO_O_PUR register in the M3 code.

/* Configure EPWM-9 pins using GPIO regs*/

// This specifies which of the possible GPIO pins will be EPWM9 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 3;   // Configure GPIO30 as EPWM9A
//  GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 3;   // Configure GPIO62 as EPWM9A
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 3;   // Configure GPIO31 as EPWM9B
//    GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 3;   // Configure GPIO63 as EPWM9B

    EDIS;
}

//---------------------------------------------------------------------------
// Example: InitEPwmSyncGpio:
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as EPwm Synch pins

void InitEPwmSyncGpio(void)
{

//   EALLOW;

/* Configure EPWMSYNCI  */

// EPWMSYNCI input is via the GPTRIPx mux's of the C28 GPIO Mux

/* Configure EPWMSYNC0  */
/* Enable internal pull-up for the selected pins */

// Pull-up enable should be set in the
// GPIO_O_PUR register in the M3 code.

// GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 3;   // Configures GPIO6 for EPWMSYNCO
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 2; // Configures GPIO33 for EPWMSYNCO

}

//---------------------------------------------------------------------------
// Example: InitTzGpio:
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as Trip Zone (TZ) pins

void InitTzGpio(void)
{
    EALLOW;

/* Configure TzGpio  */

// TzGpio input is via the GPTRIPx mux's of the C28 GPIO Mux

    EDIS;

}



