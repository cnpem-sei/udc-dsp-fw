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
 * @file ipc.h
 * @brief Interprocessor Communication module
 *
 * This module is responsible for definition of interprocessor communication
 * functionalities, between ARM and C28 cores.
 * 
 * @author gabriel.brunheira
 * @date 22/11/2017
 *
 */

#ifndef IPC_H_
#define IPC_H_

#include <stdint.h>
#include "ps_modules/ps_modules.h"
#include "control/siggen/siggen.h"
#include "control/wfmref/wfmref.h"

typedef enum {No_Error_CtoM,
              Error1,
              Error2,
              Error3,
              Error4} error_ctom;

typedef enum {No_Error_MtoC,
              Invalid_Argument,
              IPC_LowPriority_Full,
              HRADC_Config_Error} error_mtoc;

typedef volatile struct
{
    ps_module_t     ps_module;
    siggen_t        siggen;
    wfmref_t        wfmref;
    buf_t           buf_samples;
    error_ctom      error_ctom;
} ipc_ctom_t;

typedef volatile struct
{
    ps_module_t     ps_module;
    siggen_t        siggen;
    wfmref_t        wfmref;
    buf_t           buf_samples;
    error_mtoc      error_mtoc;
} ipc_mtoc_t;

extern void init_ipc(ipc_ctom_t *p_ipc_ctom);
extern void send_ipc_flag(uint32_t flag);

#endif /* IPC_H_ */
