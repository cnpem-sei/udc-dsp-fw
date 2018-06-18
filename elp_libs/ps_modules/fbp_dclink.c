/******************************************************************************
 * Copyright (C) 2018 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file fbp_dclink.C
 * @brief FBP DC-Link controller module
 *
 * Module for control of DC-Link crate from FBP FAC power supplies.
 *
 * @author gabriel.brunheira
 * @date 05/06/2018
 *
 */

#include "boards/udc_c28.h"
#include "control/control.h"
#include "ipc/ipc.h"

#include "fbp_dclink.h"

#define USE_ITLK

/**
 * Configuration parameters
 */
#define TIMEOUT_DCLINK_RELAY    1000000

/**
 * Control parameters
 */
#define MAX_REF                 g_ipc_mtoc.control.max_ref
#define MIN_REF                 g_ipc_mtoc.control.min_ref
#define MAX_REF_OL              g_ipc_mtoc.control.max_ref_openloop
#define MIN_REF_OL              g_ipc_mtoc.control.min_ref_openloop
/**
 * Digital I/O's operations and status
 */
#define PIN_OPEN_DCLINK_RELAY           CLEAR_GPDO1
#define PIN_CLOSE_DCLINK_RELAY          SET_GPDO1

#define PIN_OPEN_EXTERNAL_RELAY         CLEAR_GPDO2
#define PIN_CLOSE_EXTERNAL_RELAY        SET_GPDO2

#define PIN_STATUS_PS1_FAIL             GET_GPDI1
#define PIN_STATUS_PS2_FAIL             GET_GPDI2
#define PIN_STATUS_PS3_FAIL             GET_GPDI3

#define PIN_STATUS_SMOKE_DETECTOR       GET_GPDI4
#define PIN_STATUS_EXTERNAL_INTERLOCK   GET_GPDI5

#define PIN_STATUS_ALL_PS_FAIL          g_controller_ctom.net_signals[0].u32

#define V_DCLINK_OUTPUT                 g_controller_mtoc.net_signals[0].f   // ANI0
#define V_PS1_OUTPUT                    g_controller_mtoc.net_signals[1].f   // ANI1
#define V_PS2_OUTPUT                    g_controller_mtoc.net_signals[2].f   // ANI3
#define V_PS3_OUTPUT                    g_controller_mtoc.net_signals[3].f   // ANI2

#define DIGITAL_POT_VOLTAGE             g_controller_mtoc.net_signals[4]

/**
 * Analog variables parameters
 */
#define MAX_V_ALL_PS                    g_ipc_mtoc.analog_vars.max[0]
#define MIN_V_ALL_PS                    g_ipc_mtoc.analog_vars.min[0]

#define MAX_V_PS1                       g_ipc_mtoc.analog_vars.max[1]
#define MIN_V_PS1                       g_ipc_mtoc.analog_vars.min[1]

#define MAX_V_PS2                       g_ipc_mtoc.analog_vars.max[2]
#define MIN_V_PS2                       g_ipc_mtoc.analog_vars.min[2]

#define MAX_V_PS3                       g_ipc_mtoc.analog_vars.max[3]
#define MIN_V_PS3                       g_ipc_mtoc.analog_vars.min[3]

/**
 * Interlock defines
 */
#define PS1_FAIL                        0x00000001
#define PS2_FAIL                        0x00000002
#define PS3_FAIL                        0x00000004
#define DCLINK_OVERVOLTAGE              0x00000008
#define PS1_OVERVOLTAGE                 0x00000010
#define PS2_OVERVOLTAGE                 0x00000020
#define PS3_OVERVOLTAGE                 0x00000040
#define DCLINK_UNDERVOLTAGE             0x00000080
#define PS1_UNDERVOLTAGE                0x00000100
#define PS2_UNDERVOLTAGE                0x00000200
#define PS3_UNDERVOLTAGE                0x00000400
#define SMOKE_DETECTOR                  0x00000800
#define EXTERNAL_INTERLOCK              0x00001000

/**
 * Private functions
 */
static void init_controller(void);

static void init_interruptions(void);
static void term_interruptions(void);

static void turn_on(uint16_t id);
static void turn_off(uint16_t id);

static void set_hard_interlock(uint32_t itlk);
static interrupt void isr_hard_interlock(void);
static interrupt void isr_soft_interlock(void);
static void reset_interlocks(uint16_t id);

//static inline void check_interlocks_ps_module(void);
static void check_interlocks_ps_module(uint16_t id);


void main_fbp_dclink(void)
{
    uint16_t i;

    DELAY_US(1000000);

    init_controller();
    init_interruptions();

    /// TODO: check why first sync_pulse occurs
    g_ipc_ctom.counter_sync_pulse = 0;

    /// Enable TBCLK for buzzer and interlock LED PWM signals
    enable_pwm_tbclk();

    /// TODO: include condition for re-initialization
    while(1)
    {
        /// Group all pin status
        PIN_STATUS_ALL_PS_FAIL = ( PIN_STATUS_PS1_FAIL |
                                  (PIN_STATUS_PS2_FAIL << 1) |
                                  (PIN_STATUS_PS3_FAIL << 2) ) & 0x00000007;

        /// Check interlocks for specified power module
        for(i = 0; i < g_ipc_mtoc.num_ps_modules; i++)
        {
            check_interlocks_ps_module(i);
        }

        /// Saturate and reference
        if(g_ipc_ctom.ps_module[0].ps_status.bit.openloop)
        {
            SATURATE(g_ipc_ctom.ps_module[0].ps_setpoint, MAX_REF_OL, MIN_REF_OL);
        }
        else
        {
            SATURATE(g_ipc_ctom.ps_module[0].ps_setpoint, MAX_REF, MIN_REF);
        }

        g_ipc_ctom.ps_module[0].ps_reference = g_ipc_ctom.ps_module[0].ps_setpoint;
    }

    turn_off(0);
    term_interruptions();
}

static void init_controller(void)
{
    init_ps_module(&g_ipc_ctom.ps_module[0],
                   g_ipc_mtoc.ps_module[0].ps_status.bit.model,
                   &turn_on, &turn_off, &isr_soft_interlock,
                   &isr_hard_interlock, &reset_interlocks);

    init_ipc();
    init_control_framework(&g_controller_ctom);

    g_ipc_ctom.ps_module[0].ps_setpoint = g_ipc_mtoc.ps_module[0].ps_setpoint;
}

static void init_interruptions(void)
{
    IER |= M_INT11;

    /// Enable global interrupts (EINT)
    EINT;
    ERTM;
}

/**
 * Check variables from specified power supply for interlocks
 *
 * @param id specified power supply
 */
/*static inline void check_interlocks_ps_module(void)
{
    if(PIN_STATUS_SMOKE_DETECTOR)
    {
        set_hard_interlock(SMOKE_DETECTOR);
    }

    if(PIN_STATUS_EXTERNAL_INTERLOCK)
    {
        set_hard_interlock(EXTERNAL_INTERLOCK);
    }

    /// Check overvoltage conditions
    if(V_DCLINK_OUTPUT > MAX_V_ALL_PS)
    {
        set_hard_interlock(DCLINK_OVERVOLTAGE);
    }

    if(V_PS1_OUTPUT > MAX_V_PS1)
    {
        set_hard_interlock(PS1_OVERVOLTAGE);
    }

    if(V_PS2_OUTPUT > MAX_V_PS2)
    {
        set_hard_interlock(PS2_OVERVOLTAGE);
    }

    if(V_PS3_OUTPUT > MAX_V_PS3)
    {
        set_hard_interlock(PS3_OVERVOLTAGE);
    }

    DINT;

    if(g_ipc_ctom.ps_module[0].ps_status.bit.state > Interlock)
    {
        /// Check digital status
        if(PIN_STATUS_PS1_FAIL)
        {
            set_hard_interlock(PS1_FAIL);
        }

        if(PIN_STATUS_PS2_FAIL)
        {
            set_hard_interlock(PS2_FAIL);
        }

        if(PIN_STATUS_PS3_FAIL)
        {
            set_hard_interlock(PS3_FAIL);
        }

        /// Check undervoltage conditions
        if(V_DCLINK_OUTPUT < MIN_V_ALL_PS)
        {
            set_hard_interlock(DCLINK_UNDERVOLTAGE);
        }

        if(V_PS1_OUTPUT < MIN_V_PS1)
        {
            set_hard_interlock(PS1_UNDERVOLTAGE);
        }
        if(V_PS2_OUTPUT < MIN_V_PS2)
        {
            set_hard_interlock(PS2_UNDERVOLTAGE);
        }
        if(V_PS3_OUTPUT < MIN_V_PS3)
        {
            set_hard_interlock(PS3_UNDERVOLTAGE);
        }
    }

    else
    {
        /// Check digital status
        if(!PIN_STATUS_PS1_FAIL)
        {
            set_hard_interlock(PS1_FAIL);
        }

        if(!PIN_STATUS_PS2_FAIL)
        {
            set_hard_interlock(PS2_FAIL);
        }

        if(!PIN_STATUS_PS3_FAIL)
        {
            set_hard_interlock(PS3_FAIL);
        }
    }

    EINT;
}*/

/**
 * Turn on specified power supply.
 *
 * @param id specified power supply
 */
static void turn_on(uint16_t id)
{
    #ifdef USE_ITLK
    if(g_ipc_ctom.ps_module[0].ps_status.bit.state == Off)
    #else
    if(g_ipc_ctom.ps_module[0].ps_status.bit.state <= Interlock)
    #endif
    {
        PIN_CLOSE_DCLINK_RELAY;
        PIN_CLOSE_EXTERNAL_RELAY;
        DELAY_US(TIMEOUT_DCLINK_RELAY);

        g_ipc_ctom.ps_module[0].ps_status.bit.state = SlowRef;
        g_ipc_ctom.ps_module[0].ps_setpoint = g_ipc_mtoc.ps_module[0].ps_setpoint;
        g_ipc_ctom.ps_module[0].ps_reference = g_ipc_mtoc.ps_module[0].ps_setpoint;
    }
}

/**
 * Turn off specified power supply.
 *
 * @param id specified power supply
 */
static void turn_off(uint16_t id)
{
    PIN_OPEN_DCLINK_RELAY;
    PIN_OPEN_EXTERNAL_RELAY;
    DELAY_US(TIMEOUT_DCLINK_RELAY);

    if(g_ipc_ctom.ps_module[0].ps_status.bit.state != Interlock)
    {
        g_ipc_ctom.ps_module[0].ps_status.bit.state = Off;
    }
}

/**
 * Termination of interruptions.
 */
static void term_interruptions(void)
{
    /// Disable global interrupts (EINT)
    DINT;
    DRTM;

    /// Clear enables
    IER = 0;

    /// Clear flags
    PieCtrlRegs.PIEACK.all |= M_INT11;
}

/**
 * Set specified hard interlock for specified power supply.
 *
 * @param id specified power supply
 * @param itlk specified hard interlock
 */
static void set_hard_interlock(uint32_t itlk)
{
    if(!(g_ipc_ctom.ps_module[0].ps_hard_interlock & itlk))
    {
        #ifdef USE_ITLK
        turn_off(0);
        g_ipc_ctom.ps_module[0].ps_status.bit.state = Interlock;
        #endif

        g_ipc_ctom.ps_module[0].ps_hard_interlock |= itlk;
    }
}

/**
 * ISR for MtoC hard interlock request.
 */
static interrupt void isr_hard_interlock(void)
{
    if(! (g_ipc_ctom.ps_module[0].ps_hard_interlock &
         g_ipc_mtoc.ps_module[0].ps_hard_interlock))
    {
        #ifdef USE_ITLK
        turn_off(0);
        g_ipc_ctom.ps_module[0].ps_status.bit.state = Interlock;
        #endif

        g_ipc_ctom.ps_module[0].ps_hard_interlock |=
        g_ipc_mtoc.ps_module[0].ps_hard_interlock;
    }
}

/**
 * ISR for MtoC soft interlock request.
 */
static interrupt void isr_soft_interlock(void)
{
    if(!(g_ipc_ctom.ps_module[0].ps_soft_interlock &
         g_ipc_mtoc.ps_module[0].ps_soft_interlock))
    {
        #ifdef USE_ITLK
        turn_off(0);
        g_ipc_ctom.ps_module[0].ps_status.bit.state = Interlock;
        #endif

        g_ipc_ctom.ps_module[0].ps_soft_interlock |=
        g_ipc_mtoc.ps_module[0].ps_soft_interlock;
    }
}

/**
 * Reset interlocks for specified power supply.
 *
 * @param id specified power supply
 */
static void reset_interlocks(uint16_t id)
{
    g_ipc_ctom.ps_module[0].ps_hard_interlock = 0;
    g_ipc_ctom.ps_module[0].ps_soft_interlock = 0;

    if(g_ipc_ctom.ps_module[0].ps_status.bit.state < Initializing)
    {
        g_ipc_ctom.ps_module[0].ps_status.bit.state = Off;
    }
}

/**
 * Check variables from specified power supply for interlocks
 *
 * @param id specified power supply
 */
static void check_interlocks_ps_module(uint16_t id)
{
    if(PIN_STATUS_SMOKE_DETECTOR)
    {
        set_hard_interlock(SMOKE_DETECTOR);
    }

    if(PIN_STATUS_EXTERNAL_INTERLOCK)
    {
        set_hard_interlock(EXTERNAL_INTERLOCK);
    }

    /// Check overvoltage conditions
    if(V_DCLINK_OUTPUT > MAX_V_ALL_PS)
    {
        set_hard_interlock(DCLINK_OVERVOLTAGE);
    }

    DINT;

    switch(id)
    {
        case 0:
        {
            if(V_PS1_OUTPUT > MAX_V_PS1)
            {
                set_hard_interlock(PS1_OVERVOLTAGE);
            }

            if(g_ipc_ctom.ps_module[0].ps_status.bit.state > Interlock)
            {
                if(PIN_STATUS_PS1_FAIL)
                {
                    set_hard_interlock(PS1_FAIL);
                }

                if(V_DCLINK_OUTPUT < MIN_V_ALL_PS)
                {
                    set_hard_interlock(DCLINK_UNDERVOLTAGE);
                }

                if(V_PS1_OUTPUT < MIN_V_PS1)
                {
                    set_hard_interlock(PS1_UNDERVOLTAGE);
                }
            }

            else
            {
                if(!PIN_STATUS_PS1_FAIL)
                {
                    set_hard_interlock(PS1_FAIL);
                }
            }

            break;
        }

        case 1:
        {
            if(V_PS2_OUTPUT > MAX_V_PS2)
            {
                set_hard_interlock(PS2_OVERVOLTAGE);
            }

            if(g_ipc_ctom.ps_module[0].ps_status.bit.state > Interlock)
            {
                if(PIN_STATUS_PS2_FAIL)
                {
                    set_hard_interlock(PS2_FAIL);
                }

                if(V_DCLINK_OUTPUT < MIN_V_ALL_PS)
                {
                    set_hard_interlock(DCLINK_UNDERVOLTAGE);
                }

                if(V_PS2_OUTPUT < MIN_V_PS2)
                {
                    set_hard_interlock(PS2_UNDERVOLTAGE);
                }
            }

            else
            {
                if(!PIN_STATUS_PS2_FAIL)
                {
                    set_hard_interlock(PS2_FAIL);
                }
            }

            break;
        }

        case 2:
        {
            if(V_PS3_OUTPUT > MAX_V_PS3)
            {
                set_hard_interlock(PS3_OVERVOLTAGE);
            }

            if(g_ipc_ctom.ps_module[0].ps_status.bit.state > Interlock)
            {
                if(PIN_STATUS_PS3_FAIL)
                {
                    set_hard_interlock(PS3_FAIL);
                }

                if(V_DCLINK_OUTPUT < MIN_V_ALL_PS)
                {
                    set_hard_interlock(DCLINK_UNDERVOLTAGE);
                }

                if(V_PS3_OUTPUT < MIN_V_PS3)
                {
                    set_hard_interlock(PS3_UNDERVOLTAGE);
                }
            }

            else
            {
                if(!PIN_STATUS_PS3_FAIL)
                {
                    set_hard_interlock(PS3_FAIL);
                }
            }

            break;
        }
    }

    EINT;
}
