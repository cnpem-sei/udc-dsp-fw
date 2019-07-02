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
 * @file wfmref.c
 * @brief Waveform references module
 * 
 * This module implements waveform references functionality.
 *
 * @author gabriel.brunheira
 * @date 22 de nov de 2017
 *
 */
#include <math.h>
#include "wfmref.h"

/**
 * TODO: Put here your defines. Just what is local. If you don't
 * need to access it from other module, consider use a constant (const)
 */

/**
 * TODO: Put here your constants and variables. Always use static for 
 * private members.
 */
volatile wfmref_lerp_t g_wfmref_lerp;

/**
 * TODO: Put here your function prototypes for private functions. Use
 * static in declaration.
 */


/**
 * TODO: Put here the implementation for your public functions.
 */
void init_wfmref_lerp(float freq_base, float freq_lerp)
{
    g_wfmref_lerp.counter = 0;
    g_wfmref_lerp.max_count = (uint16_t) roundf(freq_lerp / freq_base);
    g_wfmref_lerp.fraction = freq_base / freq_lerp;
    g_wfmref_lerp.out = 0.0;
}

/**
 * TODO: Put here the implementation for your private functions.
 */
