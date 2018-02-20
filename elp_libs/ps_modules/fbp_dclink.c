/*
 * fbp_dclink.c
 *
 *  Created on: 20 de fev de 2018
 *      Author:
 */

#include "fbp_dclink.h"

static void init_interruptions(void);
static void enable_controller();

void main_fbp_dclink(void)
{
    uint16_t i;

    init_interruptions();
    enable_controller();

    /// TODO: check why first sync_pulse occurs
    g_ipc_ctom.counter_sync_pulse = 0;

    /// TODO: include condition for re-initialization
    while(1)
    {
        for(i = 0; i < NUM_MAX_PS_MODULES; i++)
        {
            if(g_ipc_ctom.ps_module[i].ps_status.bit.active)
            {
                check_interlocks_ps_module(i);
            }
        }
    }

    for(i = 0; i < NUM_MAX_PS_MODULES; i++)
    {
        turn_off(i);
    }

    disable_controller();
    term_interruptions();
}

/**
 * Initialization of interruptions.
 */
static void init_interruptions(void)
{
    EALLOW;
    PieVectTable.EPWM1_INT =  &isr_init_controller;
    PieVectTable.EPWM2_INT =  &isr_controller;
    EDIS;

    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;  /// ePWM1
    PieCtrlRegs.PIEIER3.bit.INTx2 = 1;  /// ePWM2

    enable_pwm_interrupt(PS4_PWM_MODULATOR);
    enable_pwm_interrupt(PS4_PWM_MODULATOR_NEG);

    IER |= M_INT1;
    IER |= M_INT3;
    IER |= M_INT11;

    /// Enable global interrupts (EINT)
    EINT;
    ERTM;
}

/**
 * Enable control ISR
 */
static void enable_controller()
{
    stop_DMA();
    DELAY_US(5);
    start_DMA();
    HRADCs_Info.enable_Sampling = 1;
    enable_pwm_tbclk();
}

/**
 * Check variables from specified power supply for interlocks
 *
 * @param id specified power supply
 */
static void check_interlocks_ps_module(uint16_t id)
{
    if(fabs(g_controller_ctom.net_signals[id]) > MAX_ILOAD)
    {
        set_hard_interlock(id, LOAD_OVERCURRENT);
    }

    if(fabs(g_controller_mtoc.net_signals[id]) > MAX_DCLINK)
    {
        set_hard_interlock(id, DCLINK_OVERVOLTAGE);
    }

    if(fabs(g_controller_mtoc.net_signals[id]) < MIN_DCLINK)
    {
        set_hard_interlock(id, DCLINK_UNDERVOLTAGE);
    }

    if(fabs(g_controller_mtoc.net_signals[id+4]) > MAX_VLOAD)
    {
        set_hard_interlock(id, LOAD_OVERVOLTAGE);
    }

    if(fabs(g_controller_mtoc.net_signals[id+8]) > MAX_TEMP)
    {
        set_soft_interlock(id, OVERTEMP);
    }

    switch(id)
    {
        case 0:
        {
            if(!PIN_STATUS_PS1_FUSE)
            {
                set_hard_interlock(id, FUSE_FAIL);
            }

            if(!PIN_STATUS_PS1_DRIVER_ERROR)
            {
                set_hard_interlock(id, DRIVER_FAIL);
            }

            if ( (g_ipc_ctom.ps_module[id].ps_status.bit.state <= Interlock) &&
                 (PIN_STATUS_PS1_DCLINK_RELAY) )
            {
                set_hard_interlock(id, DCLINK_RELAY_FAIL);
            }

            else if ( (g_ipc_ctom.ps_module[id].ps_status.bit.state > Interlock)
                      && (!PIN_STATUS_PS1_DCLINK_RELAY) )
            {
                set_hard_interlock(id, DCLINK_RELAY_FAIL);
            }

            break;
        }

        case 1:
        {
            if(!PIN_STATUS_PS2_FUSE)
            {
                set_hard_interlock(id, FUSE_FAIL);
            }

            if(!PIN_STATUS_PS2_DRIVER_ERROR)
            {
                set_hard_interlock(id, DRIVER_FAIL);
            }

            if ( (g_ipc_ctom.ps_module[id].ps_status.bit.state <= Interlock) &&
                (PIN_STATUS_PS2_DCLINK_RELAY))
            {
                set_hard_interlock(id, DCLINK_RELAY_FAIL);
            }

            else if ( (g_ipc_ctom.ps_module[id].ps_status.bit.state > Interlock)
                      && (!PIN_STATUS_PS2_DCLINK_RELAY) )
            {
                set_hard_interlock(id, DCLINK_RELAY_FAIL);
            }

            break;
        }

        case 2:
        {
            if(!PIN_STATUS_PS3_FUSE)
            {
                set_hard_interlock(id, FUSE_FAIL);
            }

            if(!PIN_STATUS_PS3_DRIVER_ERROR)
            {
                set_hard_interlock(id, DRIVER_FAIL);
            }

            if ( (g_ipc_ctom.ps_module[id].ps_status.bit.state <= Interlock) &&
                (PIN_STATUS_PS3_DCLINK_RELAY)) {
                set_hard_interlock(id, DCLINK_RELAY_FAIL);
            }

            else if ( (g_ipc_ctom.ps_module[id].ps_status.bit.state > Interlock)
                      && (!PIN_STATUS_PS3_DCLINK_RELAY) )
            {
                set_hard_interlock(id, DCLINK_RELAY_FAIL);
            }

            break;
        }

        case 3:
        {
            if(!PIN_STATUS_PS4_FUSE)
            {
                set_hard_interlock(id, FUSE_FAIL);
            }

            if(!PIN_STATUS_PS4_DRIVER_ERROR)
            {
                set_hard_interlock(id, DRIVER_FAIL);
            }

            if ( (g_ipc_ctom.ps_module[id].ps_status.bit.state <= Interlock) &&
                (PIN_STATUS_PS4_DCLINK_RELAY)) {
                set_hard_interlock(id, DCLINK_RELAY_FAIL);
            }

            else if ( (g_ipc_ctom.ps_module[id].ps_status.bit.state > Interlock)
                      && (!PIN_STATUS_PS4_DCLINK_RELAY) )
            {
                set_hard_interlock(id, DCLINK_RELAY_FAIL);
            }

            break;
        }
    }
}

/**
 * Turn off specified power supply.
 *
 * @param id specified power supply
 */
static void turn_off(uint16_t id)
{
    disable_pwm_output(2*id);
    disable_pwm_output((2*id)+1);

    open_relay(id);
    DELAY_US(TIMEOUT_DCLINK_RELAY);

    g_ipc_ctom.ps_module[id].ps_status.bit.openloop = OPEN_LOOP;
    if (g_ipc_ctom.ps_module[id].ps_status.bit.state != Interlock){
        g_ipc_ctom.ps_module[id].ps_status.bit.state = Off;
    }
    reset_controller(id);
}

/**
 * Disable control ISR
 */
static void disable_controller()
{
    disable_pwm_tbclk();
    HRADCs_Info.enable_Sampling = 0;
    stop_DMA();

    reset_controllers(num_active_ps_modules);
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

    PieCtrlRegs.PIEIER3.bit.INTx1 = 0;  /// ePWM1
    PieCtrlRegs.PIEIER3.bit.INTx2 = 0;  /// ePWM2

    disable_pwm_interrupt(PS4_PWM_MODULATOR);
    disable_pwm_interrupt(PS4_PWM_MODULATOR_NEG);

    /// Clear flags
    PieCtrlRegs.PIEACK.all |= M_INT1 | M_INT3 | M_INT11;
}
