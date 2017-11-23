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
 * @file fbp.c
 * @brief FBP v4.0 module
 * 
 * Module for control of FBP v4.0 power supplies (Low-Power Power Supply).
 *
 * @author gabriel.brunheira
 * @date 23/11/2017
 *
 */

#include "fbp.h"

/**
 * TODO: Put here your defines. Just what is local. If you don't
 * need to access it from other module, consider use a constant (const)
 */


/**
 * TODO: Put here your constants and variables. Always use static for 
 * private members.
 */

#pragma CODE_SECTION(isr_init_controller, "ramfuncs");
#pragma CODE_SECTION(isr_controller, "ramfuncs");
#pragma CODE_SECTION(turn_on, "ramfuncs");
#pragma CODE_SECTION(turn_off, "ramfuncs");
#pragma CODE_SECTION(set_hard_interlock, "ramfuncs");
#pragma CODE_SECTION(set_soft_interlock, "ramfuncs");
#pragma CODE_SECTION(isr_hard_interlock, "ramfuncs");
#pragma CODE_SECTION(isr_soft_interlock, "ramfuncs");

static void init_peripherals_drivers(void);
static void term_peripherals_drivers(void);

static void init_controller(void);
static void reset_controller(void);
static void enable_controller();
static void disable_controller();
interrupt void isr_init_controller(void);
interrupt void isr_controller(void);

static void init_interruptions(void);
static void term_interruptions(void);

static void turn_on(void);
static void turn_off(void);

static void set_hard_interlock(uint32_t itlk);
static void set_soft_interlock(uint32_t itlk);
interrupt void isr_hard_interlock(void);
interrupt void isr_soft_interlock(void);

/**
 * TODO: Put here the implementation for your public functions.
 */
void main_fbp(void)
{
    init_peripherals_drivers();
    init_controller();
    init_interruptions();
    enable_controller();

    /// TODO: include condition for re-initialization
    while(1)
    {

    }

    turn_off();
    disable_controller();
    term_interruptions();
    reset_controller;
    term_peripherals_drivers();
}

static void init_peripherals_drivers(void)
{

}

static void term_peripherals_drivers(void)
{

}

static void init_controller(void)
{

}

static void reset_controller(void)
{

}

static void enable_controller()
{

}

static void disable_controller()
{

}

interrupt void isr_init_controller(void)
{

}

interrupt void isr_controller(void)
{

}

static void init_interruptions(void)
{

}

static void term_interruptions(void)
{

}

static void turn_on(void)
{

}

static void turn_off(void)
{

}

static void set_hard_interlock(uint32_t itlk)
{

}

static void set_soft_interlock(uint32_t itlk)
{

}

interrupt void isr_hard_interlock(void)
{

}

interrupt void isr_soft_interlock(void)
{

}
