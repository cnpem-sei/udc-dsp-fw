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
 * @file event_manager.h
 * @brief Event manager module
 *
 * This module is responsible for
 * 
 * @author gabriel.brunheira
 * @date 14/08/2018
 *
 */

#ifndef EVENT_MANAGER_H_
#define EVENT_MANAGER_H_

#include <stdint.h>
#include "ps_modules/ps_modules.h"

#define USE_ITLK
#define NUM_MAX_EVENT_COUNTER       32

typedef struct
{
    uint16_t flag;
    uint32_t counter;
    uint32_t event_count;
    uint32_t counter_overflow;
} event_counter_t;

typedef struct
{
    uint16_t num_events;
    event_counter_t event[NUM_MAX_EVENT_COUNTER];
} event_counters_t;

typedef struct
{
    uint16_t timebase_flag;
    float freq_timebase;
    event_counters_t hard_interlocks;
    event_counters_t soft_interlocks;
} event_manager_t;

extern volatile event_manager_t g_event_manager[NUM_MAX_PS_MODULES];

extern void init_event_manager(uint16_t id, float freq_timebase,
                               uint16_t num_hard_itlks, uint16_t num_soft_itlks,
                               float *hard_itlks_count_us,
                               float *hard_itlks_overflow_us,
                               float *soft_itlks_count_us,
                               float *soft_itlks_overflow_us);
extern void check_events(uint16_t id);
extern void set_hard_interlock_2(uint16_t id, uint32_t itlk);
extern void set_soft_interlock_2(uint16_t id, uint32_t itlk);

#endif /* EVENT_MANAGER_H_ */
