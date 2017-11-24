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
#include "common/structs.h"
#include "ps_modules/ps_modules.h"
#include "siggen/siggen.h"
#include "wfmref/wfmref.h"

/**
 * Shared resources defines
 */

#define SIZE_BUF_SAMPLES    4096

/*
 * MtoC Message Defines
 */
#define TURN_ON                 0x00000011 // IPC1 + IPC5
#define TURN_OFF                0x00000021 // IPC1 + IPC6
#define OPEN_LOOP               0x00000041 // IPC1 + IPC7
#define CLOSE_LOOP              0x00000081 // IPC1 + IPC8
#define OPERATING_MODE          0x00000101 // IPC1 + IPC9
#define RESET_INTERLOCKS        0x00000201 // IPC1 + IPC10
#define UNLOCK_UDC              0x00000401 // IPC1 + IPC11
#define LOCK_UDC                0x00000801 // IPC1 + IPC12
#define CONFIG_BUF_SAMPLES      0x00001001 // IPC1 + IPC13
#define ENABLE_BUF_SAMPLES      0x00002001 // IPC1 + IPC14
#define DISABLE_BUF_SAMPLES     0x00004001 // IPC1 + IPC15
#define SET_SLOWREF             0x00008001 // IPC1 + IPC16
#define SET_SLOWREF_ALL_PS      0x00010001 // IPC1 + IPC17
#define CONFIG_WFMREF           0x00020001 // IPC1 + IPC18
#define SELECT_WFMREF           0x00040001 // IPC1 + IPC19
#define RESET_WFMREF            0x00080001 // IPC1 + IPC20
#define CONFIG_SIGGEN           0x00100001 // IPC1 + IPC21
#define SCALE_SIGGEN            0x00200001 // IPC1 + IPC22
#define ENABLE_SIGGEN           0x00400001 // IPC1 + IPC23
#define DISABLE_SIGGEN          0x00800001 // IPC1 + IPC24
#define IPC_25                  0x01000001 // IPC1 + IPC25
#define IPC_26                  0x02000001 // IPC1 + IPC26
#define IPC_27                  0x04000001 // IPC1 + IPC27
#define HRADC_SAMPLING_DISABLE  0x08000001 // IPC1 + IPC28
#define HRADC_SAMPLING_ENABLE   0x10000001 // IPC1 + IPC29
#define HRADC_OPMODE            0x20000001 // IPC1 + IPC30
#define HRADC_CONFIG            0x40000001 // IPC1 + IPC31
#define CTOM_MESSAGE_ERROR      0x80000001 // IPC1 + IPC32

#define SYNC_PULSE              0x00000002 // IPC2

/*
 * CtoM Message Defines
 */
#define MTOC_MESSAGE_ERROR      0x80000001 // IPC1+IPC32

#define HARD_INTERLOCK          0x00000004 // IPC3
#define SOFT_INTERLOCK          0x00000008 // IPC4


typedef enum {No_Error_CtoM,     //!< No_Error_CtoM
              Error1,            //!< Error1
              Error2,            //!< Error2
              Error3,            //!< Error3
              Error4} error_ctom;//!< Error4

typedef enum {No_Error_MtoC,
              Invalid_Argument,
              Invalid_OpMode,
              IPC_LowPriority_Full,
              HRADC_Config_Error} error_mtoc;

typedef volatile struct
{
    uint16_t        msg_id;
    error_mtoc      error_mtoc;
    ps_module_t     ps_module[NUM_MAX_PS_MODULES];
    siggen_t        siggen[NUM_MAX_PS_MODULES];
    wfmref_t        wfmref[NUM_MAX_PS_MODULES];
    buf_t           buf_samples[NUM_MAX_PS_MODULES];
} ipc_ctom_t;

typedef volatile struct
{
    uint16_t        msg_id;
    error_ctom      error_ctom;
    ps_module_t     ps_module[NUM_MAX_PS_MODULES];
    siggen_t        siggen[NUM_MAX_PS_MODULES];
    wfmref_t        wfmref[NUM_MAX_PS_MODULES];
    buf_t           buf_samples[NUM_MAX_PS_MODULES];
} ipc_mtoc_t;

extern ipc_ctom_t g_ipc_ctom;
extern ipc_mtoc_t g_ipc_mtoc;

extern void init_ipc(void);
extern void send_ipc_msg(uint16_t msg_id, uint32_t flag);

#endif /* IPC_H_ */
