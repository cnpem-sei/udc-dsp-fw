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
 * @file event_manager.C
 * @brief Event manager module
 *
 * This module
 * 
 * @author gabriel.brunheira
 * @date 14/08/2018
 *
 */

#include <stdint.h>
#include "boards/udc_c28.h"
#include "event_manager/event_manager.h"
#include "ipc/ipc.h"

#define MAX_DEBOUNCE_TIME_US    5000000
#define MAX_RESET_TIME_US       10000000

const static uint32_t lut_bit_position[32] =
{
    0x00000001, 0x00000002, 0x00000004, 0x00000008,
    0x00000010, 0x00000020, 0x00000040, 0x00000080,
    0x00000100, 0x00000200, 0x00000400, 0x00000800,
    0x00001000, 0x00002000, 0x00004000, 0x00008000,
    0x00010000, 0x00020000, 0x00040000, 0x00080000,
    0x00100000, 0x00200000, 0x00400000, 0x00800000,
    0x01000000, 0x02000000, 0x04000000, 0x08000000,
    0x10000000, 0x20000000, 0x40000000, 0x80000000
};

volatile event_manager_t g_event_manager[NUM_MAX_PS_MODULES];

#pragma CODE_SECTION(set_hard_interlock, "ramfuncs");
#pragma CODE_SECTION(set_soft_interlock, "ramfuncs");
#pragma CODE_SECTION(isr_hard_interlock, "ramfuncs");
#pragma CODE_SECTION(isr_soft_interlock, "ramfuncs");

void init_event_manager(uint16_t id, float freq_timebase,
                        uint16_t num_hard_itlks, uint16_t num_soft_itlks,
                        uint32_t *hard_itlks_debounce_time_us,
                        uint32_t *hard_itlks_reset_time_us,
                        uint32_t *soft_itlks_debounce_time_us,
                        uint32_t *soft_itlks_reset_time_us)
{
    uint16_t i;
    uint32_t debounce_time_us, reset_time_us, max_reset_counts;

    max_reset_counts = (uint32_t) ((freq_timebase * MAX_RESET_TIME_US) * 1e-6);

    g_event_manager[id].timebase_flag = 0;
    g_event_manager[id].freq_timebase = freq_timebase;
    g_event_manager[id].hard_interlocks.num_events = num_hard_itlks;
    g_event_manager[id].soft_interlocks.num_events = num_soft_itlks;

    for(i = 0; i < NUM_MAX_EVENT_COUNTER; i++)
    {
        g_event_manager[id].hard_interlocks.event[i].flag = 0;
        g_event_manager[id].hard_interlocks.event[i].counter = 0;

        /// Initialization of hard interlocks
        if(i < num_hard_itlks)
        {
            debounce_time_us = *(hard_itlks_debounce_time_us + i);
            reset_time_us = *(hard_itlks_reset_time_us+i);

            SATURATE(debounce_time_us, MAX_DEBOUNCE_TIME_US , 0);

            g_event_manager[id].hard_interlocks.event[i].debounce_count =
                    (uint32_t) ( (freq_timebase * debounce_time_us) * 1e-6);
            g_event_manager[id].hard_interlocks.event[i].reset_count =
                    (uint32_t) ( (freq_timebase * reset_time_us) * 1e-6);

            SATURATE(g_event_manager[id].hard_interlocks.event[i].reset_count,
                     max_reset_counts,
                     g_event_manager[id].hard_interlocks.event[i].debounce_count+1);

        }
        else
        {
            g_event_manager[id].hard_interlocks.event[i].debounce_count = 0;
            g_event_manager[id].hard_interlocks.event[i].reset_count = 0;
        }

        /// Initialization of soft interlocks
        g_event_manager[id].soft_interlocks.event[i].flag = 0;
        g_event_manager[id].soft_interlocks.event[i].counter = 0;

        if(i < num_soft_itlks)
        {
            debounce_time_us = *(soft_itlks_debounce_time_us + i);
            reset_time_us = *(soft_itlks_reset_time_us+i);

            SATURATE(debounce_time_us, MAX_DEBOUNCE_TIME_US , 0);

            g_event_manager[id].soft_interlocks.event[i].debounce_count =
                    (uint32_t) ( (freq_timebase * debounce_time_us) * 1e-6);
            g_event_manager[id].soft_interlocks.event[i].reset_count =
                    (uint32_t) ( (freq_timebase * reset_time_us) * 1e-6);

            SATURATE(g_event_manager[id].soft_interlocks.event[i].reset_count,
                     max_reset_counts,
                     g_event_manager[id].soft_interlocks.event[i].debounce_count+1);
        }
        else
        {
            g_event_manager[id].soft_interlocks.event[i].debounce_count = 0;
            g_event_manager[id].soft_interlocks.event[i].reset_count = 0;
        }
    }
}

void check_events(uint16_t id)
{
    uint16_t i;

    if(g_event_manager[id].timebase_flag)
    {
        //SET_DEBUG_GPIO1;
        for(i = 0; i < g_event_manager[id].hard_interlocks.num_events; i++)
        {
            if( g_event_manager[id].hard_interlocks.event[i].flag )
            {
                if(++g_event_manager[id].hard_interlocks.event[i].counter >=
                     g_event_manager[id].hard_interlocks.event[i].reset_count)
                {
                    g_event_manager[id].hard_interlocks.event[i].flag = 0;
                    g_event_manager[id].hard_interlocks.event[i].counter = 0;
                    CLEAR_DEBUG_GPIO1;
                }
            }
        }

        for(i = 0; i < g_event_manager[id].soft_interlocks.num_events; i++)
        {
            if( g_event_manager[id].soft_interlocks.event[i].flag )
            {
                if(++g_event_manager[id].soft_interlocks.event[i].counter >=
                     g_event_manager[id].soft_interlocks.event[i].reset_count)
                {
                    g_event_manager[id].soft_interlocks.event[i].flag = 0;
                    g_event_manager[id].soft_interlocks.event[i].counter = 0;
                    CLEAR_DEBUG_GPIO1;
                }
            }
        }

        g_event_manager[id].timebase_flag = 0;
        //CLEAR_DEBUG_GPIO1;
    }
}

/**
 * Set specified hard interlock for specified module.
 *
 * @param module    specified AC/DC module
 * @param itlk      specified hard interlock
 */
void set_hard_interlock(uint16_t module, uint32_t itlk)
{
    g_event_manager[module].hard_interlocks.event[itlk].flag = 1;

    if(g_event_manager[module].hard_interlocks.event[itlk].counter >=
       g_event_manager[module].hard_interlocks.event[itlk].debounce_count)
    {
        if(!(g_ipc_ctom.ps_module[module].ps_hard_interlock & lut_bit_position[itlk]))
        {
            SET_DEBUG_GPIO1
            #ifdef USE_ITLK
            g_ipc_ctom.ps_module[module].turn_off(module);
            g_ipc_ctom.ps_module[module].ps_status.bit.state = Interlock;
            #endif

            g_ipc_ctom.ps_module[module].ps_hard_interlock |= lut_bit_position[itlk];
        }

        g_event_manager[module].hard_interlocks.event[itlk].flag = 0;
        g_event_manager[module].hard_interlocks.event[itlk].counter = 0;
    }
}

/**
 * Set specified soft interlock for specified module.
 *
 * @param module    specified module
 * @param itlk      specified soft interlock
 */
void set_soft_interlock(uint16_t module, uint32_t itlk)
{
    g_event_manager[module].soft_interlocks.event[itlk].flag = 1;

    if(g_event_manager[module].soft_interlocks.event[itlk].counter >=
       g_event_manager[module].soft_interlocks.event[itlk].debounce_count)
    {
        if(!(g_ipc_ctom.ps_module[module].ps_soft_interlock & lut_bit_position[itlk]))
        {

            #ifdef USE_ITLK
            g_ipc_ctom.ps_module[module].turn_off(module);
            g_ipc_ctom.ps_module[module].ps_status.bit.state = Interlock;
            #endif

            g_ipc_ctom.ps_module[module].ps_soft_interlock |= lut_bit_position[itlk];
        }

        g_event_manager[module].soft_interlocks.event[itlk].flag = 0;
        g_event_manager[module].soft_interlocks.event[itlk].counter = 0;
    }
}

/**
 * ISR for MtoC hard interlock request.
 */
interrupt void isr_hard_interlock(void)
{
    if(!(g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_hard_interlock &
         g_ipc_mtoc.ps_module[g_ipc_mtoc.msg_id].ps_hard_interlock))
    {
        #ifdef USE_ITLK
        g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].turn_off(g_ipc_mtoc.msg_id);
        g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_status.bit.state = Interlock;
        #endif

        g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_hard_interlock |=
        g_ipc_mtoc.ps_module[g_ipc_mtoc.msg_id].ps_hard_interlock;
    }
}

/**
 * ISR for MtoC soft interlock request.
 */
interrupt void isr_soft_interlock(void)
{
    if(!(g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_soft_interlock &
         g_ipc_mtoc.ps_module[g_ipc_mtoc.msg_id].ps_soft_interlock))
    {
        #ifdef USE_ITLK
        g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].turn_off(g_ipc_mtoc.msg_id);
        g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_status.bit.state = Interlock;
        #endif

        g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_soft_interlock |=
        g_ipc_mtoc.ps_module[g_ipc_mtoc.msg_id].ps_soft_interlock;
    }
}
