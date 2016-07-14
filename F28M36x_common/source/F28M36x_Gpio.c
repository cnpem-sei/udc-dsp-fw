//###########################################################################
// FILE:    F28M36x_Gpio.c
// TITLE:    F28M36x General Purpose I/O Initialization & Support Functions.
//###########################################################################
// $TI Release: F28M36x Support Library v202 $
// $Release Date: Tue Apr  8 12:36:34 CDT 2014 $
//###########################################################################

#include "F28M36x_Device.h"     // F28M36x Headerfile Include File
#include "F28M36x_Examples.h"   // F28M36x Examples Include File

//---------------------------------------------------------------------------
// InitGpio:
//---------------------------------------------------------------------------
// This function initializes the Gpio to a known (default) state.
// For more details on configuring GPIO's as peripheral functions,
// refer to the individual peripheral examples and/or GPIO setup example.
void InitGpio(void)
{
    EALLOW;

    // Each GPIO pin can be:
    // a) a GPIO input/output
    // b) peripheral function 1
    // c) peripheral function 2
    // d) peripheral function 3
    // By default, all are GPIO Inputs
    GpioG1CtrlRegs.GPAMUX1.all = 0x0000;    // GPIO functionality GPIO0-GPIO15
    GpioG1CtrlRegs.GPAMUX2.all = 0x0000;    // GPIO functionality GPIO16-GPIO31
    GpioG1CtrlRegs.GPBMUX1.all = 0x0000;    // GPIO functionality GPIO32-GPIO44
    GpioG1CtrlRegs.GPBMUX2.all = 0x0000;    // GPIO functionality GPIO45-GPIO63
    GpioG1CtrlRegs.GPCMUX1.all = 0x0000;    // GPIO functionality GPIO64-GPIO79
    GpioG1CtrlRegs.GPCMUX2.all = 0x0000;    // GPIO functionality GPIO80-GPIO95
    GpioG1CtrlRegs.GPDMUX1.all = 0x0000;    // GPIO functionality GPIO96-GPIO111
    GpioG1CtrlRegs.GPDMUX2.all = 0x0000;    // GPIO functionality GPIO112-GPIO127
    GpioG1CtrlRegs.GPEMUX1.all = 0x0000;    // GPIO functionality GPIO128-GPIO135
    
    GpioG2CtrlRegs.GPGMUX1.all = 0x0000;    // GPIO functionality GPIO192-199
    GpioG2CtrlRegs.AIOMUX1.all = 0x0000;    // Dig.IO funct. applies to 
                                            // AIO2,4,6,10,12,14
    GpioG2CtrlRegs.AIOMUX2.all = 0x0000;    // Dig.IO funct. applies to
                                            // AIO18,20,22,26,28,30

    GpioG1CtrlRegs.GPADIR.all = 0x0000;     // GPIO0-GPIO31 are inputs
    GpioG1CtrlRegs.GPBDIR.all = 0x0000;     // GPIO32-GPIO63 are inputs
    GpioG1CtrlRegs.GPCDIR.all = 0x0000;     // GPIO64-GPIO95 are inputs
    GpioG1CtrlRegs.GPDDIR.all = 0x0000;     // GPIO96-GPIO127 are inputs
    GpioG1CtrlRegs.GPEDIR.all = 0x0000;     // GPIO128-GPIO159 are inputs
    GpioG2CtrlRegs.GPGDIR.all = 0x0000;     // GPIO192-199 are inputs
    GpioG2CtrlRegs.AIODIR.all = 0x0000;     //
                                            //
                                            // AIO2,4,6,19,12,14,18,20,22,26,28,30
                                            // are digital inputs

    // Each input can have different qualification
    // a) input synchronized to SYSCLKOUT
    // b) input qualified by a sampling window
    // c) input sent asynchronously (valid for peripheral inputs only)
    GpioG1CtrlRegs.GPAQSEL1.all = 0x0000;   // GPIO0-GPIO15 Synch to SYSCLKOUT
    GpioG1CtrlRegs.GPAQSEL2.all = 0x0000;   // GPIO16-GPIO31 Synch to SYSCLKOUT
    GpioG1CtrlRegs.GPBQSEL1.all = 0x0000;   // GPIO32-GPIO44 Synch to SYSCLKOUT
    GpioG1CtrlRegs.GPBQSEL2.all = 0x0000;   // GPIO45-GPIO63 Synch to SYSCLKOUT
    GpioG1CtrlRegs.GPCQSEL1.all = 0x0000;   // GPIO64-GPIO79 Synch to SYSCLKOUT
    GpioG1CtrlRegs.GPCQSEL2.all = 0x0000;   // GPIO80-GPIO95 Synch to SYSCLKOUT
    GpioG1CtrlRegs.GPDQSEL1.all = 0x0000;   // GPIO96-GPIO111 Synch to SYSCLKOUT
    GpioG1CtrlRegs.GPDQSEL2.all = 0x0000;   // GPIO112-GPIO127 Synch to SYSCLKOUT
    GpioG1CtrlRegs.GPEQSEL1.all = 0x0000;   // GPIO128-GPIO135 Synch to SYSCLKOUT
    GpioG2CtrlRegs.GPGQSEL1.all = 0x0000;   // GPIO192-GPIO199 Synch to SYSCLKOUT

/* Enable internal pull-up for the selected pins */

// Pull-up enable should be set in the
// GPIO_O_PUR register in the M3 code.

    EDIS;

}



