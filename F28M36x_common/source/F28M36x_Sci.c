//###########################################################################
// FILE:    F28M36x_Sci.c
// TITLE:    F28M36x SCI Initialization & Support Functions.
//###########################################################################
// $TI Release: F28M36x Support Library v202 $
// $Release Date: Tue Apr  8 12:36:34 CDT 2014 $
//###########################################################################

#include "F28M36x_Device.h"     // F28M36x Headerfile Include File
#include "F28M36x_Examples.h"   // F28M36x Examples Include File

//---------------------------------------------------------------------------
// InitSci:
//---------------------------------------------------------------------------
// This function initializes the SCI(s) to a known state.
void InitSci(void)
{
    // Initialize SCI-A:

    //tbd...

}

//---------------------------------------------------------------------------
// Example: InitSciGpio:
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as SCI pins
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.
// Caution:
// Only one GPIO pin should be enabled for SCITXDA operation.
// Only one GPIO pin shoudl be enabled for SCIRXDA operation.
// Comment out other unwanted lines.

void InitSciGpio()
{
    InitSciaGpio();
}

void InitSciaGpio()
{
    EALLOW;

/* Enable internal pull-up for the selected pins */

// Pull-up enable should be set in the
// GPIO_O_PUR register in the M3 code.

/* Set qualification for selected pins to asynch only */

// Inputs are synchronized to SYSCLKOUT by default.
// This will select asynch (no qualification) for the selected pins.

    GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (SCIRXDA)
//    GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 3;  // Asynch input GPIO32 (SCIRXDA)
//    GpioCtrlRegs.GPBQSEL1.bit.GPIO34 = 3;  // Asynch input GPIO34 (SCIRXDA)
//    GpioCtrlRegs.GPBQSEL1.bit.GPIO36 = 3;  // Asynch input GPIO36 (SCIRXDA)

/* Configure SCI-A pins using GPIO regs*/

// This specifies which of the possible GPIO pins will be SCI functional pins.

    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;   // Configure GPIO28 for SCIRXDA
                                           // operation
//    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 2;   // Configure GPIO32 for SCIRXDA
// operation
//    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 2;   // Configure GPIO34 for SCIRXDA
// operation
//    GpioCtrlRegs.GPBMUX1.bit.GPIO36 = 1;   // Configure GPIO36 for SCIRXDA
// operation

    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;   // Configure GPIO29 for SCITXDA
                                           // operation
//    GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 1;   // Configure GPIO35 for SCITXDA
// operation

    EDIS;
}



