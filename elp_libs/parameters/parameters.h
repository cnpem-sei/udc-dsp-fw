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
 * @file parameters.h
 * @brief Power supply parameters bank module.
 * 
 * This module implements a data structure for initialization and configuration
 * of parameters for operation of the power supplies applications.
 *
 * @author gabriel.brunheira
 * @date 23/02/2018
 *
 */

#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <stdint.h>
#include <math.h>
#include "common/structs.h"
#include "common/timeslicer.h"
#include "event_manager/event_manager.h"
#include "ps_modules/ps_modules.h"
#include "siggen/siggen.h"

#define NUM_MAX_ANALOG_VAR      64
#define NUM_MAX_DIGITAL_VAR     12
#define NUM_MAX_HRADC           4

#define NUM_MAX_PARAMETERS      64
#define NUM_MAX_FLOATS          200

typedef enum
{
    PS_Model,
    Num_PS_Modules,

    Command_Interface,
    RS485_Baudrate,
    RS485_Address,
    RS485_Termination,
    UDCNet_Address,
    Ethernet_IP,
    Ethernet_Subnet_Mask,
    Buzzer_Volume,

    Freq_ISR_Controller,
    Freq_TimeSlicer,
    Control_Loop_State,
    Max_Ref,
    Min_Ref,
    Max_Ref_OpenLoop,
    Min_Ref_OpenLoop,
    Max_SlewRate_SlowRef,
    Max_SlewRate_SigGen_Amp,
    Max_SlewRate_SigGen_Offset,
    Max_SlewRate_WfmRef,

    PWM_Freq,
    PWM_DeadTime,
    PWM_Max_Duty,
    PWM_Min_Duty,
    PWM_Max_Duty_OpenLoop,
    PWM_Min_Duty_OpenLoop,
    PWM_Lim_Duty_Share,

    HRADC_Num_Boards,
    HRADC_Freq_SPICLK,
    HRADC_Freq_Sampling,
    HRADC_Enable_Heater,
    HRADC_Enable_Monitor,
    HRADC_Type_Transducer,
    HRADC_Gain_Transducer,
    HRADC_Offset_Transducer,

    SigGen_Type,
    SigGen_Num_Cycles,
    SigGen_Freq,
    SigGen_Amplitude,
    SigGen_Offset,
    SigGen_Aux_Param,

    WfmRef_ID_WfmRef,
    WfmRef_SyncMode,
    WfmRef_Gain,
    WfmRef_Offset,

    Analog_Var_Max,
    Analog_Var_Min,

    Hard_Interlocks_Debounce_Time,
    Hard_Interlocks_Reset_Time,
    Soft_Interlocks_Debounce_Time,
    Soft_Interlocks_Reset_Time
} param_id_t;

typedef enum
{
    is_uint16_t,
    is_uint32_t,
    is_float
} param_type_t;

typedef union
{
    uint16_t    *u16;
    uint32_t    *u32;
    float       *f;
} p_param_t;

typedef struct
{
    param_id_t      id;
    param_type_t    type;
    uint16_t        num_elements;
    p_param_t       p_val;
} param_t;

typedef struct
{
    float           rs485_baud;
    uint16_t        rs485_address[NUM_MAX_PS_MODULES];
    uint16_t        rs485_termination;
    uint16_t        udcnet_address;
    uint32_t        ethernet_ip;
    uint32_t        ethernet_mask;
    uint16_t        buzzer_volume;
    uint16_t        command_interface;
} param_communication_t;

typedef struct
{
    float   freq_isr_control;
    float   freq_timeslicer[NUM_MAX_TIMESLICERS];
    uint16_t loop_state;
    float   max_ref;
    float   min_ref;
    float   max_ref_openloop;
    float   min_ref_openloop;
    float   slewrate_slowref;
    float   slewrate_siggen_amp;
    float   slewrate_siggen_offset;
    float   slewrate_wfmref;
} param_control_t;

typedef struct
{
    float   freq_pwm;
    float   dead_time;
    float   max_duty;
    float   min_duty;
    float   max_duty_openloop;
    float   min_duty_openloop;
    float   lim_duty_share;
} param_pwm_t;

typedef struct
{
    uint16_t    num_hradc;
    uint16_t    freq_spiclk;
    float       freq_hradc_sampling;
    uint16_t    enable_heater[NUM_MAX_HRADC];
    uint16_t    enable_monitor[NUM_MAX_HRADC];
    uint16_t    type_transducer_output[NUM_MAX_HRADC];
    float       gain_transducer[NUM_MAX_HRADC];
    float       offset_transducer[NUM_MAX_HRADC];
} param_hradc_t;

typedef struct
{
    float   max[NUM_MAX_ANALOG_VAR];
    float   min[NUM_MAX_ANALOG_VAR];
} param_analog_vars_t;

typedef struct
{
    uint32_t    hard_itlks_debounce_time[NUM_MAX_HARD_INTERLOCKS];
    uint32_t    hard_itlks_reset_time[NUM_MAX_HARD_INTERLOCKS];
    uint32_t    soft_itlks_debounce_time[NUM_MAX_SOFT_INTERLOCKS];
    uint32_t    soft_itlks_reset_time[NUM_MAX_SOFT_INTERLOCKS];
} param_interlocks_t;

extern volatile param_t g_parameters[NUM_MAX_PARAMETERS];

extern void init_param(param_id_t id, param_type_t type, uint16_t num_elements,
                       uint16_t *p_param);
extern uint16_t set_param(param_id_t id, uint16_t n, float val);
extern float get_param(param_id_t id, uint16_t n);

#endif /* PARAMETERS_H_ */
