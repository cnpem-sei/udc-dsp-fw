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
 * @file wfmref.h
 * @brief Waveform references module
 * 
 * This module implements waveform references functionality.
 *
 * @author gabriel.brunheira
 * @date 22 de nov de 2017
 *
 */

#ifndef WFMREF_H_
#define WFMREF_H_

#include <stdint.h>
#include "common/structs.h"

#define SIZE_WFMREF             4096
#define WFMREF                  g_ipc_ctom.wfmref

#define INTERPOLATE(a, b, f)    (a * (1.0 - f)) + (b * f)

typedef enum
{
    SampleBySample,
    SampleBySample_OneCycle,
    OneShot
} sync_mode_t;

typedef volatile struct
{
    uint16_t        counter;
    uint16_t        max_count;
    float           fraction;
    float           out;
} wfmref_lerp_t;

typedef volatile struct
{
    buf_t           wfmref_data;
    float           gain;
    float           offset;
    uint16_t        wfmref_selected;
    sync_mode_t     sync_mode;
} wfmref_t;

/**
 * TODO: Put here your functions prototypes. Just what need 
 * to be accessed by other modules.
 */

extern volatile wfmref_lerp_t g_wfmref_lerp;

extern void init_wfmref_lerp(float freq_base, float freq_lerp);
extern void reset_wfmref(wfmref_t *p_wfmref);

#endif /* WFMREF_H_ */
