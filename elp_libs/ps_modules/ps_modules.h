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
 * @file ps_modules.h
 * @brief Power supplies modules.
 * 
 * Main source file for power supply modules. It includes macros and enumerates
 * related to operation of power supplies from ELP group on Sirius Project.
 *
 * @author gabriel.brunheira
 * @date 25/10/2017
 *
 */

#ifndef PS_MODULES_H_
#define PS_MODULES_H_

#include <stdint.h>
#include <string.h>
#include "../control/siggen/siggen.h"

/**
 * TODO: update macros for interlock check
 */
#define CHECK_SOFTINTERLOCK(itlk)   !(IPC_CtoM_Msg.PSModule.SoftInterlocks & itlk)
#define CHECK_INTERLOCK(itlk)       !(IPC_CtoM_Msg.PSModule.HardInterlocks & itlk)
#define CHECK_INTERLOCKS            !(IPC_CtoM_Msg.PSModule.HardInterlocks)

#define tCLOSED_LOOP     0
#define tOPEN_LOOP       1

#define INACTIVE        0
#define ACTIVE          1

#define LOCKED          0
#define UNLOCKED        1


typedef enum
{
    Off,
    Interlock,
    Initializing,
    //SlowRef,
    SlowRefSync,
    //FastRef,
    RmpWfm,
    MigWfm,
    Cycle
} ps_state_t;


typedef enum
{
    Remote,
    Local,
    PCHost
} ps_interface_t;

typedef enum
{
    FBP
} ps_model_t;

typedef struct
{
    uint16_t state      : 4;    // 3:0      Operation state
    uint16_t openloop   : 1;    // 4        Control loop state
    uint16_t interface  : 2;    // 6:5      Communication interface
    uint16_t active     : 1;    // 7        Power supply active?
    uint16_t model      : 5;    // 12:8     Power supply model
    uint16_t unlocked   : 1;    // 13       Unlocked?
    uint16_t reserved   : 2;    // 15:14    Reserved for future use
} ps_status_bits_t;

typedef union
{
    uint16_t            all;
    ps_status_bits_t    bit;
} ps_status_t;

typedef struct
{
    ps_status_t     ps_status;
    float           ps_setpoint;
    float           ps_reference;
    void            (*turn_on)(void);
    void            (*turn_off)(void);
    void            (*set_softinterlock)(void);
    void            (*set_hardinterlock)(void);
    void            (*reset_interlocks)(void);
    //wfmref_t        wfmref;
    siggen_t        siggen;
} ps_module_t;

/**
 * Initialization of power supply module. It requires address of specific power
 * supply functions ```turn_on```, ```turn_off``` and ```reset_interlocks```.
 *
 * @param p_ps_module pointer to the ps module struct
 * @param model power supply model to be initialized
 * @param turn_on address of ```turn_on()``` function to that power supply
 * @param turn_off address of ```turn_off()``` function
 * @param set_softinterlock address of ```set_softinterlock()``` function
 * @param set_hardinterlock address of ```set_hardinterlock()``` function
 * @param reset_interlocks address of ```reset_interlocks()``` function to that power supply
 */
extern void init_ps_module(ps_module_t *p_ps_module, ps_model_t model,
                    void (*turn_on)(void), void (*turn_off)(void),
                    void (*set_softinterlock)(void),
                    void (*set_hardinterlock)(void),
                    void (*reset_interlocks)(void));

extern void cfg_ps_operation_mode(ps_module_t *p_ps_module, ps_state_t op_mode);


#endif /* PS_MODULES_H_ */
