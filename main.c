/******************************************************************************
 * Copyright (C) 2017 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file main.c
 * @brief Main file of firmware for C28 core from DRS-UDC board.
 *
 * Main file of firmware for C28 core from DRS-UDC board. This firmware
 * implements digital controllers for magnet power supplies from Sirius Project.
 * 
 * At initialization, the ARM core reads from non-volatile memory which power
 * supply model the controller is set, in order to both cores be initialized
 * with the proper power supply module (ps_module).
 *
 * @author gabriel.brunheira
 * @date 20/10/2017
 *
 */

#include <string.h>
#include "F28M36x_ELP_DRS.h"
#include "udc_c28.h"

/**
 * TODO: Put here your defines. Just what is local. If you don't
 * need to access it from other module, consider use a constant (const)
 */

/**
 * TODO: Put here your constants and variables. Always use static for 
 * private members.
 */

/**
 * TODO: Put here your function prototypes for private functions. Use
 * static in declaration.
 */


/**
 * TODO: Put here the implementation for your public functions.
 */

/**
 * @brief Main function
 */
void main(void)
{
    /**
     * Initialize the Control System:
     * Enable peripheral clocks
     * This example function is found in the F28M36x_SysCtrl.c file.
     */
    InitSysCtrl();

    /**
     * Copy time critical code and Flash setup code to RAM
     * This includes the following functions:  InitFlash();
     * The  RamfuncsLoadStart, RamfuncsLoadSize, and RamfuncsRunStart
     * symbols are created by the linker. Refer to the device .cmd file.
     */
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);

    /**
     * Call Flash Initialization to setup flash waitstates
     * This function must reside in RAM
     */
    InitFlash();

    /**
     * Disable CPU interrupts
     */
    DINT;

    /**
     * Initialize the PIE control registers to their default state.
     * The default state is all PIE interrupts disabled and flags are cleared.
     * This function is found in the F28M36x_PieCtrl.c file.
     */
    InitPieCtrl();

    /**
     * Disable CPU interrupts and clear all CPU interrupt flags:
     */
    IER = 0x0000;
    IFR = 0x0000;

    /**
     * Initialize the PIE vector table with pointers to the shell Interrupt
     * Service Routines (ISR).
     * This will populate the entire table, even if the interrupt
     * is not used in this example.  This is useful for debug purposes.
     * The shell ISR routines are found in F28M36x_DefaultIsr.c.
     * This function is found in F28M36x_PieVect.c.
     */
    InitPieVectTable();


    /**
     *  TODO: Make sure ARM is already initialized to continue from here
     */

    init_gpios();
    init_buzzer(1);     /// Volume: 1%


    while(1)
    {
        /**
         * Select power supply module
         */
        switch(IPC_MtoC_Msg.PSModule.Model)
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
                //main_FBPx4_100kHz();
            }

            case FAP_6U_DCDC_20kHz:
            {
                //main_FAP_6U_DCDC_20kHz();
            }

            default:
            {
                break;
            }
        }

    }
}

/**
 * TODO: Put here the implementation for your private functions.
 */

