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
 * @file fbp.c
 * @brief FBP v4.0 module
 * 
 * Module for control of FBP v4.0 power supplies (Low-Power Power Supply).
 *
 * @author gabriel.brunheira
 * @date 23/11/2017
 *
 */

#include <float.h>

#include "boards/udc_c28.h"
#include "common/timeslicer.h"
#include "control/control.h"
#include "HRADC_board/HRADC_Boards.h"
#include "ipc/ipc.h"
#include "parameters/parameters.h"
#include "pwm/pwm.h"

#include "fbp.h"

/**
 * Configuration parameters
 *
 * TODO: transfer this to param bank
 */

#define USE_ITLK
#define TIMEOUT_DCLINK_RELAY    200000

#define PWM_FREQ                g_ipc_mtoc.pwm.freq_pwm
#define PWM_DEAD_TIME           g_ipc_mtoc.pwm.dead_time
#define PWM_MAX_DUTY            g_ipc_mtoc.pwm.max_duty
#define PWM_MIN_DUTY            g_ipc_mtoc.pwm.min_duty
#define PWM_MAX_DUTY_OL         g_ipc_mtoc.pwm.max_duty_openloop
#define PWM_MIN_DUTY_OL         g_ipc_mtoc.pwm.min_duty_openloop

#define MAX_REF                 g_ipc_mtoc.control.max_ref
#define MIN_REF                 g_ipc_mtoc.control.min_ref
#define MAX_REF_SLEWRATE        g_ipc_mtoc.control.slewrate_slowref
#define MAX_SR_SIGGEN_OFFSET    g_ipc_mtoc.control.slewrate_siggen_offset
#define MAX_SR_SIGGEN_AMP       g_ipc_mtoc.control.slewrate_siggen_amp
#define CONTROL_FREQ            g_ipc_mtoc.control.freq_isr_control
#define HRADC_FREQ_SAMP         g_ipc_mtoc.hradc.freq_hradc_sampling
#define HRADC_SPI_CLK           g_ipc_mtoc.hradc.freq_spiclk
#define DECIMATION_FACTOR       1//(HRADC_FREQ_SAMP/CONTROL_FREQ)

#define TIMESLICER_BUFFER       1
#define BUFFER_DECIMATION       (1.0 / g_ipc_mtoc.control.freq_timeslicer[TIMESLICER_BUFFER])

#define MAX_ILOAD               g_ipc_mtoc.analog_vars.max[0]
#define MAX_VLOAD               g_ipc_mtoc.analog_vars.max[1]
#define MIN_DCLINK              g_ipc_mtoc.analog_vars.min[2]
#define MAX_DCLINK              g_ipc_mtoc.analog_vars.max[2]
#define MAX_TEMP                g_ipc_mtoc.analog_vars.max[3]

#define SIGGEN                  g_ipc_ctom.siggen
#define SIGGEN_OUTPUT           g_controller_ctom.net_signals[12].f

#define TRANSDUCER_OUTPUT_TYPE  g_ipc_mtoc.hradc.type_transducer_output[0]
#if (HRADC_v2_0)
    #define TRANSDUCER_GAIN     -g_ipc_mtoc.hradc.gain_transducer[0]
#endif
#if (HRADC_v2_1)
    #define TRANSDUCER_GAIN     g_ipc_mtoc.hradc.gain_transducer[0]
#endif

/**
 * All power supplies defines
 *
 */
#define PS_ALL_ID   0x000F

#define LOAD_OVERCURRENT            0x00000001
#define LOAD_OVERVOLTAGE            0x00000002
#define DCLINK_OVERVOLTAGE          0x00000004
#define DCLINK_UNDERVOLTAGE         0x00000008
#define DCLINK_RELAY_FAIL           0x00000010
#define FUSE_FAIL                   0x00000020
#define DRIVER_FAIL                 0x00000040

#define OVERTEMP                    0x00000001

/**
 * Power supply 1 defines
 */
#define PS1_ID                          0x0000

#define PIN_OPEN_PS1_DCLINK_RELAY       CLEAR_GPDO4;
#define PIN_CLOSE_PS1_DCLINK_RELAY      SET_GPDO4;

#define PIN_STATUS_PS1_DCLINK_RELAY     GET_GPDI4
#define PIN_STATUS_PS1_DRIVER_ERROR     GET_GPDI5
#define PIN_STATUS_PS1_FUSE             GET_GPDI14

#define PS1_LOAD_CURRENT                g_controller_ctom.net_signals[0].f  // HRADC0
#define PS1_DCLINK_VOLTAGE              g_controller_mtoc.net_signals[0].f  // ANI2
#define PS1_LOAD_VOLTAGE                g_controller_mtoc.net_signals[4].f  // ANI6
#define PS1_TEMPERATURE                 g_controller_mtoc.net_signals[8].f  // I2C Add 0x48

#define PS1_SETPOINT                    g_ipc_ctom.ps_module[0].ps_setpoint
#define PS1_REFERENCE                   g_ipc_ctom.ps_module[0].ps_reference

#define ERROR_CALCULATOR_PS1            &g_controller_ctom.dsp_modules.dsp_error[0]
#define PI_CONTROLLER_ILOAD_PS1         &g_controller_ctom.dsp_modules.dsp_pi[0]
#define PI_CONTROLLER_ILOAD_PS1_COEFFS  g_controller_mtoc.dsp_modules.dsp_pi[0].coeffs.s

#define PS1_KP                          PI_CONTROLLER_ILOAD_PS1_COEFFS.kp
#define PS1_KI                          PI_CONTROLLER_ILOAD_PS1_COEFFS.ki

#define PS1_PWM_MODULATOR               g_pwm_modules.pwm_regs[0]
#define PS1_PWM_MODULATOR_NEG           g_pwm_modules.pwm_regs[1]

/**
 * Power supply 2 defines
 */
#define PS2_ID                          0x0001

#define PIN_OPEN_PS2_DCLINK_RELAY       CLEAR_GPDO3;
#define PIN_CLOSE_PS2_DCLINK_RELAY      SET_GPDO3;

#define PIN_STATUS_PS2_DCLINK_RELAY     GET_GPDI11
#define PIN_STATUS_PS2_DRIVER_ERROR     GET_GPDI9
#define PIN_STATUS_PS2_FUSE             GET_GPDI16

#define PS2_LOAD_CURRENT                g_controller_ctom.net_signals[1].f  // HRADC1
#define PS2_DCLINK_VOLTAGE              g_controller_mtoc.net_signals[1].f  // ANI1
#define PS2_LOAD_VOLTAGE                g_controller_mtoc.net_signals[5].f  // ANI7
#define PS2_TEMPERATURE                 g_controller_mtoc.net_signals[9].f  // I2C Add 0x49

#define PS2_SETPOINT                    g_ipc_ctom.ps_module[1].ps_setpoint
#define PS2_REFERENCE                   g_ipc_ctom.ps_module[1].ps_reference

#define ERROR_CALCULATOR_PS2            &g_controller_ctom.dsp_modules.dsp_error[1]
#define PI_CONTROLLER_ILOAD_PS2         &g_controller_ctom.dsp_modules.dsp_pi[1]
#define PI_CONTROLLER_ILOAD_PS2_COEFFS  g_controller_mtoc.dsp_modules.dsp_pi[1].coeffs.s

#define PS2_KP                          PI_CONTROLLER_ILOAD_PS2_COEFFS.kp
#define PS2_KI                          PI_CONTROLLER_ILOAD_PS2_COEFFS.ki

#define PS2_PWM_MODULATOR               g_pwm_modules.pwm_regs[2]
#define PS2_PWM_MODULATOR_NEG           g_pwm_modules.pwm_regs[3]

/**
 * Power supply 3 defines
 */
#define PS3_ID                          0x0002

#define PIN_OPEN_PS3_DCLINK_RELAY       CLEAR_GPDO1;
#define PIN_CLOSE_PS3_DCLINK_RELAY      SET_GPDO1;

#define PIN_STATUS_PS3_DCLINK_RELAY     GET_GPDI8
#define PIN_STATUS_PS3_DRIVER_ERROR     GET_GPDI1
#define PIN_STATUS_PS3_FUSE             GET_GPDI13

#define PS3_LOAD_CURRENT                g_controller_ctom.net_signals[2].f  // HRADC2
#define PS3_DCLINK_VOLTAGE              g_controller_mtoc.net_signals[2].f  // ANI4
#define PS3_LOAD_VOLTAGE                g_controller_mtoc.net_signals[6].f  // ANI3
#define PS3_TEMPERATURE                 g_controller_mtoc.net_signals[10].f // I2C Add 0x4A

#define PS3_SETPOINT                    g_ipc_ctom.ps_module[2].ps_setpoint
#define PS3_REFERENCE                   g_ipc_ctom.ps_module[2].ps_reference

#define ERROR_CALCULATOR_PS3            &g_controller_ctom.dsp_modules.dsp_error[2]
#define PI_CONTROLLER_ILOAD_PS3         &g_controller_ctom.dsp_modules.dsp_pi[2]
#define PI_CONTROLLER_ILOAD_PS3_COEFFS  g_controller_mtoc.dsp_modules.dsp_pi[2].coeffs.s

#define PS3_KP                          PI_CONTROLLER_ILOAD_PS3_COEFFS.kp
#define PS3_KI                          PI_CONTROLLER_ILOAD_PS3_COEFFS.ki

#define PS3_PWM_MODULATOR               g_pwm_modules.pwm_regs[4]
#define PS3_PWM_MODULATOR_NEG           g_pwm_modules.pwm_regs[5]

/**
 * Power supply 4 defines
 */
#define PS4_ID                          0x0003

#define PIN_OPEN_PS4_DCLINK_RELAY       CLEAR_GPDO2;
#define PIN_CLOSE_PS4_DCLINK_RELAY      SET_GPDO2;

#define PIN_STATUS_PS4_DCLINK_RELAY     GET_GPDI2
#define PIN_STATUS_PS4_DRIVER_ERROR     GET_GPDI3
#define PIN_STATUS_PS4_FUSE             GET_GPDI15

#define PS4_LOAD_CURRENT                g_controller_ctom.net_signals[3].f  // HRADC3
#define PS4_DCLINK_VOLTAGE              g_controller_mtoc.net_signals[3].f  // ANI0
#define PS4_LOAD_VOLTAGE                g_controller_mtoc.net_signals[7].f  // ANI5
#define PS4_TEMPERATURE                 g_controller_mtoc.net_signals[11].f // I2C Add 0x4C

#define PS4_SETPOINT                    g_ipc_ctom.ps_module[3].ps_setpoint
#define PS4_REFERENCE                   g_ipc_ctom.ps_module[3].ps_reference

#define ERROR_CALCULATOR_PS4            &g_controller_ctom.dsp_modules.dsp_error[3]
#define PI_CONTROLLER_ILOAD_PS4         &g_controller_ctom.dsp_modules.dsp_pi[3]
#define PI_CONTROLLER_ILOAD_PS4_COEFFS  g_controller_mtoc.dsp_modules.dsp_pi[3].coeffs.s

#define PS4_KP                          PI_CONTROLLER_ILOAD_PS4_COEFFS.kp
#define PS4_KI                          PI_CONTROLLER_ILOAD_PS4_COEFFS.ki


#define PS4_PWM_MODULATOR               g_pwm_modules.pwm_regs[6]
#define PS4_PWM_MODULATOR_NEG           g_pwm_modules.pwm_regs[7]

#pragma CODE_SECTION(isr_init_controller, "ramfuncs");
#pragma CODE_SECTION(isr_controller, "ramfuncs");
#pragma CODE_SECTION(turn_off, "ramfuncs");
#pragma CODE_SECTION(set_hard_interlock, "ramfuncs");
#pragma CODE_SECTION(set_soft_interlock, "ramfuncs");
#pragma CODE_SECTION(isr_hard_interlock, "ramfuncs");
#pragma CODE_SECTION(isr_soft_interlock, "ramfuncs");
#pragma CODE_SECTION(open_relay, "ramfuncs");
#pragma CODE_SECTION(get_relay_status, "ramfuncs");

static void init_peripherals_drivers(void);
static void term_peripherals_drivers(void);

static void init_controller(void);
static void reset_controller(uint16_t id);
static void reset_controllers(void);
static void enable_controller();
static void disable_controller();
static interrupt void isr_init_controller(void);
static interrupt void isr_controller(void);

static void init_interruptions(void);
static void term_interruptions(void);

static void turn_on(uint16_t id);
static void turn_off(uint16_t id);

static void reset_interlocks(uint16_t id);
static void set_hard_interlock(uint16_t id, uint32_t itlk);
static void set_soft_interlock(uint16_t id, uint32_t itlk);
static interrupt void isr_hard_interlock(void);
static interrupt void isr_soft_interlock(void);

static void open_relay(uint16_t id);
static void close_relay(uint16_t id);
static uint16_t get_relay_status(uint16_t id);
static void check_interlocks_ps_module(uint16_t id);

/**
 * Main function for this power supply module
 */
void main_fbp(void)
{
    uint16_t i;

    init_controller();
    init_peripherals_drivers();
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
    reset_controllers();
    term_peripherals_drivers();
}

static void init_peripherals_drivers(void)
{
    uint16_t i;

    /// Initialization of HRADC boards
    stop_DMA();

    HRADCs_Info.enable_Sampling = 0;

    Init_DMA_McBSP_nBuffers(g_ipc_mtoc.num_ps_modules, DECIMATION_FACTOR, HRADC_SPI_CLK);

    Init_SPIMaster_McBSP(HRADC_SPI_CLK);
    Init_SPIMaster_Gpio();
    InitMcbspa20bit();

    DELAY_US(500000);
    send_ipc_lowpriority_msg(0,Enable_HRADC_Boards);
    DELAY_US(2000000);

    for(i = 0; i < g_ipc_mtoc.num_ps_modules; i++)
    {
        Init_HRADC_Info(&HRADCs_Info.HRADC_boards[i], i, DECIMATION_FACTOR,
                        buffers_HRADC[i], TRANSDUCER_GAIN);
        Config_HRADC_board(&HRADCs_Info.HRADC_boards[i], Iin_bipolar,
                           HEATER_DISABLE, RAILS_DISABLE);
    }

    HRADCs_Info.n_HRADC_boards = g_ipc_mtoc.num_ps_modules;

    Config_HRADC_SoC(HRADC_FREQ_SAMP);

    /// Initialization of PWM modules
    g_pwm_modules.num_modules = 8;

    PS1_PWM_MODULATOR       = &EPwm7Regs;   // PS-1 Positive polarity switches
    PS1_PWM_MODULATOR_NEG   = &EPwm8Regs;   // PS-1 Negative polarity switches

    PS2_PWM_MODULATOR       = &EPwm5Regs;   // PS-2 Positive polarity switches
    PS2_PWM_MODULATOR_NEG   = &EPwm6Regs;   // PS-2 Negative polarity switches

    PS3_PWM_MODULATOR       = &EPwm3Regs;   // PS-3 Positive polarity switches
    PS3_PWM_MODULATOR_NEG   = &EPwm4Regs;   // PS-3 Negative polarity switches

    PS4_PWM_MODULATOR       = &EPwm1Regs;   // PS-4 Positive polarity switches
    PS4_PWM_MODULATOR_NEG   = &EPwm2Regs;   // PS-4 Negative polarity switches

    disable_pwm_outputs();
    disable_pwm_tbclk();
    init_pwm_mep_sfo();

    /// PS-4 PWM initialization
    init_pwm_module(PS4_PWM_MODULATOR, PWM_FREQ, 0, PWM_Sync_Master, 0,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);
    init_pwm_module(PS4_PWM_MODULATOR_NEG, PWM_FREQ, 1, PWM_Sync_Slave, 180,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);

    /// PS-3 PWM initialization
    init_pwm_module(PS3_PWM_MODULATOR, PWM_FREQ, 0, PWM_Sync_Slave, 0,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);
    init_pwm_module(PS3_PWM_MODULATOR_NEG, PWM_FREQ, 3, PWM_Sync_Slave, 180,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);

    /// PS-2 PWM initialization
    init_pwm_module(PS2_PWM_MODULATOR, PWM_FREQ, 0, PWM_Sync_Slave, 0,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);
    init_pwm_module(PS2_PWM_MODULATOR_NEG, PWM_FREQ, 5, PWM_Sync_Slave, 180,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);

    /// PS-1 PWM initialization
    init_pwm_module(PS1_PWM_MODULATOR, PWM_FREQ, 0, PWM_Sync_Slave, 0,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);
    init_pwm_module(PS1_PWM_MODULATOR_NEG, PWM_FREQ, 7, PWM_Sync_Slave, 180,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);

    InitEPwm1Gpio();
    InitEPwm2Gpio();
    InitEPwm3Gpio();
    InitEPwm4Gpio();
    InitEPwm5Gpio();
    InitEPwm6Gpio();
    InitEPwm7Gpio();
    InitEPwm8Gpio();

    /// Initialization of timers
    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, C28_FREQ_MHZ, 1000000);
    CpuTimer0Regs.TCR.bit.TIE = 0;
}

static void term_peripherals_drivers(void)
{
}

static void init_controller(void)
{
    static uint16_t i;

    for(i = 0; i < NUM_MAX_PS_MODULES; i++)
    {
        init_ps_module(&g_ipc_ctom.ps_module[i],
                       g_ipc_mtoc.ps_module[i].ps_status.bit.model,
                       &turn_on, &turn_off, &isr_soft_interlock,
                       &isr_hard_interlock, &reset_interlocks);

        if(!g_ipc_mtoc.ps_module[i].ps_status.bit.active)
        {
            g_ipc_ctom.ps_module[i].ps_status.bit.active = 0;
        }
    }

    init_ipc();
    init_control_framework(&g_controller_ctom);

    /// Initialization of signal generator module
    disable_siggen(&g_ipc_ctom.siggen);
    init_siggen(&g_ipc_ctom.siggen, CONTROL_FREQ, &SIGGEN_OUTPUT);
    cfg_siggen(&g_ipc_ctom.siggen, g_ipc_mtoc.siggen.type,
               g_ipc_mtoc.siggen.num_cycles, g_ipc_mtoc.siggen.freq,
               g_ipc_mtoc.siggen.amplitude, g_ipc_mtoc.siggen.offset,
               g_ipc_mtoc.siggen.aux_param);

    /**
     * TODO: initialize WfmRef and Samples Buffer
     */

    /// INITIALIZATION OF LOAD CURRENT CONTROL LOOP FOR POWER SUPPLY

    /**
     *        name:     ERROR_CALCULATOR_PS1
     * description:     Load current reference error
     *  dsp module:     DSP_Error
     *           +:     ps_module[0].ps_reference
     *           -:     net_signals[0]
     *         out:     net_signals[4]
     */

    init_dsp_error(ERROR_CALCULATOR_PS1, &PS1_REFERENCE, &PS1_LOAD_CURRENT,
                   &g_controller_ctom.net_signals[4].f);

    /**
     *        name:     PI_CONTROLLER_ILOAD_PS1
     * description:     Load current PI controller
     *  dsp module:     DSP_PI
     *          in:     net_signals[4]
     *         out:     output_signals[0]
     */

    init_dsp_pi(PI_CONTROLLER_ILOAD_PS1, PS1_KP, PS1_KI, CONTROL_FREQ,
                PWM_MAX_DUTY, PWM_MIN_DUTY, &g_controller_ctom.net_signals[4].f,
                &g_controller_ctom.output_signals[0].f);

    /// INITIALIZATION OF LOAD CURRENT CONTROL LOOP FOR POWER SUPPLY 2

    /**
     *        name:     ERROR_CALCULATOR_PS2
     * description:     Load current reference error
     *  dsp module:     DSP_Error
     *           +:     ps_module[1].ps_reference
     *           -:     net_signals[1]
     *         out:     net_signals[5]
     */

    init_dsp_error(ERROR_CALCULATOR_PS2, &PS2_REFERENCE, &PS2_LOAD_CURRENT,
                   &g_controller_ctom.net_signals[5].f);

    /**
     *        name:     PI_CONTROLLER_ILOAD_PS2
     * description:     Load current PI controller
     *  dsp module:     DSP_PI
     *          in:     net_signals[5]
     *         out:     output_signals[1]
     */

    init_dsp_pi(PI_CONTROLLER_ILOAD_PS2, PS2_KP, PS2_KI, CONTROL_FREQ,
                PWM_MAX_DUTY, PWM_MIN_DUTY, &g_controller_ctom.net_signals[5].f,
                &g_controller_ctom.output_signals[1].f);

    /// INITIALIZATION OF LOAD CURRENT CONTROL LOOP FOR POWER SUPPLY 3

    /**
     *        name:     ERROR_CALCULATOR_PS3
     * description:     Load current reference error
     *  dsp module:     DSP_Error
     *           +:     ps_module[2].ps_reference
     *           -:     net_signals[2]
     *         out:     net_signals[6]
     */

    init_dsp_error(ERROR_CALCULATOR_PS3, &PS3_REFERENCE, &PS3_LOAD_CURRENT,
                   &g_controller_ctom.net_signals[6].f);

    /**
     *        name:     PI_CONTROLLER_ILOAD_PS3
     * description:     Load current PI controller
     *  dsp module:     DSP_PI
     *          in:     net_signals[6]
     *         out:     output_signals[2]
     */

    init_dsp_pi(PI_CONTROLLER_ILOAD_PS3, PS3_KP, PS3_KI, CONTROL_FREQ,
                PWM_MAX_DUTY, PWM_MIN_DUTY, &g_controller_ctom.net_signals[6].f,
                &g_controller_ctom.output_signals[2].f);

    /// INITIALIZATION OF LOAD CURRENT CONTROL LOOP FOR POWER SUPPLY 4

    /**
     *        name:     ERROR_CALCULATOR_PS4
     * description:     Load current reference error
     *  dsp module:     DSP_Error
     *           +:     ps_module[3].ps_reference
     *           -:     net_signals[3]
     *         out:     net_signals[7]
     */

    init_dsp_error(ERROR_CALCULATOR_PS4, &PS4_REFERENCE, &PS4_LOAD_CURRENT,
                   &g_controller_ctom.net_signals[7].f);

    /**
     *        name:     PI_CONTROLLER_ILOAD_PS4
     * description:     Load current PI controller
     *  dsp module:     DSP_PI
     *          in:     net_signals[7]
     *         out:     output_signals[3]
     */
    init_dsp_pi(PI_CONTROLLER_ILOAD_PS4, PS4_KP, PS4_KI, CONTROL_FREQ,
                PWM_MAX_DUTY, PWM_MIN_DUTY, &g_controller_ctom.net_signals[7].f,
                &g_controller_ctom.output_signals[3].f);

    /// INITIALIZATION OF TIME SLICERS

    /// 0: Time-slicer for WfmRef sweep decimation

    cfg_timeslicer(TIMESLICER_WFMREF, WFMREF_DECIMATION);

    /// 1: Time-slicer for SamplesBuffer
    cfg_timeslicer(TIMESLICER_BUFFER, BUFFER_DECIMATION);

    /// Reset all internal variables
    reset_controllers();
}

/**
 * Reset all internal variables for controller of specified power supply
 *
 * @param id specified power supply
 */
static void reset_controller(uint16_t id)
{
    set_pwm_duty_hbridge(g_pwm_modules.pwm_regs[id*2], 0.0);

    g_ipc_ctom.ps_module[id].ps_setpoint = 0.0;
    g_ipc_ctom.ps_module[id].ps_reference = 0.0;

    reset_dsp_error(&g_controller_ctom.dsp_modules.dsp_error[id]);
    reset_dsp_pi(&g_controller_ctom.dsp_modules.dsp_pi[id]);

    disable_siggen(&SIGGEN);
    reset_timeslicers();
}

/**
 * Reset all internal variables for all active power supplies
 */
static void reset_controllers(void)
{
    uint16_t i;

    for(i = 0; i < g_ipc_mtoc.num_ps_modules; i++)
    {
        reset_controller(i);
    }
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
 * Disable control ISR
 */
static void disable_controller()
{
    disable_pwm_tbclk();
    HRADCs_Info.enable_Sampling = 0;
    stop_DMA();

    reset_controllers();
}

/**
 * ISR for control initialization
 */
static interrupt void isr_init_controller(void)
{
    uint16_t i;

    EALLOW;
    PieVectTable.EPWM1_INT = &isr_controller;
    EDIS;

    for(i = 0; i < g_pwm_modules.num_modules; i++)
    {
        g_pwm_modules.pwm_regs[i]->ETSEL.bit.INTSEL = ET_CTR_ZERO;
        g_pwm_modules.pwm_regs[i]->ETCLR.bit.INT = 1;
    }

    PieCtrlRegs.PIEACK.all |= M_INT3;
}

/**
 * Control ISR
 */
static interrupt void isr_controller(void)
{
    static uint16_t i, flag_siggen = 0;
    static float temp[4];

    SET_DEBUG_GPIO1;

    /// Get HRADC samples
    temp[0] = (float) *(HRADCs_Info.HRADC_boards[0].SamplesBuffer);
    temp[1] = (float) *(HRADCs_Info.HRADC_boards[1].SamplesBuffer);
    temp[2] = (float) *(HRADCs_Info.HRADC_boards[2].SamplesBuffer);
    temp[3] = (float) *(HRADCs_Info.HRADC_boards[3].SamplesBuffer);

    temp[0] *= HRADCs_Info.HRADC_boards[0].gain;
    temp[0] += HRADCs_Info.HRADC_boards[0].offset;

    temp[1] *= HRADCs_Info.HRADC_boards[1].gain;
    temp[1] += HRADCs_Info.HRADC_boards[1].offset;

    temp[2] *= HRADCs_Info.HRADC_boards[2].gain;
    temp[2] += HRADCs_Info.HRADC_boards[2].offset;

    temp[3] *= HRADCs_Info.HRADC_boards[3].gain;
    temp[3] += HRADCs_Info.HRADC_boards[3].offset;

    PS1_LOAD_CURRENT = temp[0];
    PS2_LOAD_CURRENT = temp[1];
    PS3_LOAD_CURRENT = temp[2];
    PS4_LOAD_CURRENT = temp[3];

    /// Loop through active power supplies
    for(i = 0; i < NUM_MAX_PS_MODULES; i++)
    {
        /// Check whether power supply is active
        if(g_ipc_ctom.ps_module[i].ps_status.bit.active)
        {
            /// Check whether power supply is ON
            if(g_ipc_ctom.ps_module[i].ps_status.bit.state > Interlock)
            {
                /// Calculate reference according to operation mode
                switch(g_ipc_ctom.ps_module[i].ps_status.bit.state)
                {
                    case SlowRef:
                    case SlowRefSync:
                    {
                        g_ipc_ctom.ps_module[i].ps_reference =
                        g_ipc_ctom.ps_module[i].ps_setpoint;
                        break;
                    }
                    case RmpWfm:
                    {
                        break;
                    }
                    case MigWfm:
                    {
                        break;
                    }
                    case Cycle:
                    {
                        if(!flag_siggen)
                        {
                            SIGGEN.amplitude = g_ipc_mtoc.siggen.amplitude;
                            SIGGEN.offset = g_ipc_mtoc.siggen.offset;
                            //SET_DEBUG_GPIO1;
                            SIGGEN.p_run_siggen(&SIGGEN);
                            //CLEAR_DEBUG_GPIO1;
                            flag_siggen = 1;
                        }

                        g_ipc_ctom.ps_module[i].ps_reference = SIGGEN_OUTPUT;

                        break;
                    }
                    default:
                    {
                        break;
                    }
                }

                /// Open-loop
                if(g_ipc_ctom.ps_module[i].ps_status.bit.openloop)
                {
                    g_controller_ctom.output_signals[i].f =
                            0.01 * g_ipc_ctom.ps_module[i].ps_reference;

                    SATURATE(g_controller_ctom.output_signals[i].f,
                             PWM_MAX_DUTY_OL, PWM_MIN_DUTY_OL);
                }
                /// Closed-loop
                else
                {
                    SATURATE(g_ipc_ctom.ps_module[i].ps_reference, MAX_REF, MIN_REF);

                    //run_dsp_error(&g_controller_ctom.dsp_modules.dsp_error[i]);

                    *g_controller_ctom.dsp_modules.dsp_error[i].error =
                            *g_controller_ctom.dsp_modules.dsp_error[i].pos -
                            *g_controller_ctom.dsp_modules.dsp_error[i].neg;

                    run_dsp_pi(&g_controller_ctom.dsp_modules.dsp_pi[i]);

                    //SATURATE(g_controller_ctom.output_signals[i].f,
                    //         PWM_MAX_DUTY, PWM_MIN_DUTY);
                }

                set_pwm_duty_hbridge(g_pwm_modules.pwm_regs[i*2],
                                     g_controller_ctom.output_signals[i].f);
            }
        }

        /// TODO: save on buffers
    }

    PS1_PWM_MODULATOR->ETCLR.bit.INT = 1;
    PS1_PWM_MODULATOR_NEG->ETCLR.bit.INT = 1;
    PS2_PWM_MODULATOR->ETCLR.bit.INT = 1;
    PS2_PWM_MODULATOR_NEG->ETCLR.bit.INT = 1;
    PS3_PWM_MODULATOR->ETCLR.bit.INT = 1;
    PS3_PWM_MODULATOR_NEG->ETCLR.bit.INT = 1;
    PS4_PWM_MODULATOR->ETCLR.bit.INT = 1;
    PS4_PWM_MODULATOR_NEG->ETCLR.bit.INT = 1;

    flag_siggen = 0;

    CLEAR_DEBUG_GPIO1;

    PieCtrlRegs.PIEACK.all |= M_INT3;
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

/**
 * Turn on specified power supply.
 *
 * @param id specified power supply
 */
static void turn_on(uint16_t id)
{
    if(g_ipc_ctom.ps_module[id].ps_status.bit.active)
    {
        #ifdef USE_ITLK
        if(g_ipc_ctom.ps_module[id].ps_status.bit.state == Off)
        #else
        if(g_ipc_ctom.ps_module[id].ps_status.bit.state <= Interlock)
        #endif
        {
            reset_controller(id);
            close_relay(id);

            DELAY_US(TIMEOUT_DCLINK_RELAY);

            if(!get_relay_status(id))
            {
                set_hard_interlock(id,DCLINK_RELAY_FAIL);
            }
            else
            {
                g_ipc_ctom.ps_module[id].ps_status.bit.openloop = OPEN_LOOP;
                g_ipc_ctom.ps_module[id].ps_status.bit.state = SlowRef;

                enable_pwm_output(2*id);
                enable_pwm_output((2*id)+1);
            }
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
    if(g_ipc_ctom.ps_module[id].ps_status.bit.active)
    {
        disable_pwm_output(2*id);
        disable_pwm_output((2*id)+1);

        open_relay(id);
        DELAY_US(TIMEOUT_DCLINK_RELAY);

        g_ipc_ctom.ps_module[id].ps_status.bit.openloop = OPEN_LOOP;
        if (g_ipc_ctom.ps_module[id].ps_status.bit.state != Interlock)
        {
            g_ipc_ctom.ps_module[id].ps_status.bit.state = Off;
        }
        reset_controller(id);
    }
}

/**
 * Reset interlocks for specified power supply.
 *
 * @param id specified power supply
 */
static void reset_interlocks(uint16_t id)
{
    g_ipc_ctom.ps_module[id].ps_hard_interlock = 0;
    g_ipc_ctom.ps_module[id].ps_soft_interlock = 0;

    if(g_ipc_ctom.ps_module[id].ps_status.bit.state < Initializing)
    {
        g_ipc_ctom.ps_module[id].ps_status.bit.state = Off;
    }
}

/**
 * Set specified hard interlock for specified power supply.
 *
 * @param id specified power supply
 * @param itlk specified hard interlock
 */
static void set_hard_interlock(uint16_t id, uint32_t itlk)
{
    if(!(g_ipc_ctom.ps_module[id].ps_hard_interlock & itlk))
    {
        #ifdef USE_ITLK
        turn_off(id);
        g_ipc_ctom.ps_module[id].ps_status.bit.state = Interlock;
        #endif

        g_ipc_ctom.ps_module[id].ps_hard_interlock |= itlk;
    }
}

/**
 * Set specified soft interlock for specified power supply.
 *
 * @param id specified power supply
 * @param itlk specified soft interlock
 */
static void set_soft_interlock(uint16_t id, uint32_t itlk)
{
    if(!(g_ipc_ctom.ps_module[id].ps_soft_interlock & itlk))
    {
        #ifdef USE_ITLK
        turn_off(id);
        g_ipc_ctom.ps_module[id].ps_status.bit.state = Interlock;
        #endif

        g_ipc_ctom.ps_module[id].ps_soft_interlock |= itlk;
    }
}

/**
 * ISR for MtoC hard interlock request.
 */
static interrupt void isr_hard_interlock(void)
{
    if(! (g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_hard_interlock &
         g_ipc_mtoc.ps_module[g_ipc_mtoc.msg_id].ps_hard_interlock))
    {
        #ifdef USE_ITLK
        turn_off(g_ipc_mtoc.msg_id);
        g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_status.bit.state = Interlock;
        #endif

        g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_hard_interlock |=
        g_ipc_mtoc.ps_module[g_ipc_mtoc.msg_id].ps_hard_interlock;
    }
}

/**
 * ISR for MtoC soft interlock request.
 */
static interrupt void isr_soft_interlock(void)
{
    if(! (g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_soft_interlock &
         g_ipc_mtoc.ps_module[g_ipc_mtoc.msg_id].ps_soft_interlock))
    {
        #ifdef USE_ITLK
        turn_off(g_ipc_mtoc.msg_id);
        g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_status.bit.state = Interlock;
        #endif

        g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_soft_interlock |=
        g_ipc_mtoc.ps_module[g_ipc_mtoc.msg_id].ps_soft_interlock;
    }
}

/**
 * Open relay from specified power supply.
 *
 * @param id specified power supply
 */
static void open_relay(uint16_t id)
{
    switch(id)
    {
        case PS1_ID:
        {
            PIN_OPEN_PS1_DCLINK_RELAY;
            break;
        }

        case PS2_ID:
        {
            PIN_OPEN_PS2_DCLINK_RELAY;
            break;
        }

        case PS3_ID:
        {
            PIN_OPEN_PS3_DCLINK_RELAY;
            break;
        }

        case PS4_ID:
        {
            PIN_OPEN_PS4_DCLINK_RELAY;
            break;
        }

        default:
        {
            break;
        }
    }
}

/**
 * Close relay from specified power supply.
 *
 * @param id specified power supply
 */
static void close_relay(uint16_t id)
{
    switch(id)
    {
        case PS1_ID:
        {
            PIN_CLOSE_PS1_DCLINK_RELAY;
            break;
        }

        case PS2_ID:
        {
            PIN_CLOSE_PS2_DCLINK_RELAY;
            break;
        }

        case PS3_ID:
        {
            PIN_CLOSE_PS3_DCLINK_RELAY;
            break;
        }

        case PS4_ID:
        {
            PIN_CLOSE_PS4_DCLINK_RELAY;
            break;
        }

        default:
        {
            break;
        }
    }
}

/**
 * Get relay status from specified power supply.
 *
 * @param id specified power supply
 */
static uint16_t get_relay_status(uint16_t id)
{
    switch(id)
    {
        case PS1_ID:
        {
            return PIN_STATUS_PS1_DCLINK_RELAY;
        }

        case PS2_ID:
        {
            return PIN_STATUS_PS2_DCLINK_RELAY;
        }

        case PS3_ID:
        {
            return PIN_STATUS_PS3_DCLINK_RELAY;
        }

        case PS4_ID:
        {
            return PIN_STATUS_PS4_DCLINK_RELAY;
        }

        default:
        {
            return 0;
        }
    }
}

/**
 * Check variables from specified power supply for interlocks
 *
 * @param id specified power supply
 */
static void check_interlocks_ps_module(uint16_t id)
{
    if(fabs(g_controller_ctom.net_signals[id].f) > MAX_ILOAD)
    {
        set_hard_interlock(id, LOAD_OVERCURRENT);
    }

    if(fabs(g_controller_mtoc.net_signals[id].f) > MAX_DCLINK)
    {
        set_hard_interlock(id, DCLINK_OVERVOLTAGE);
    }

    if(fabs(g_controller_mtoc.net_signals[id].f) < MIN_DCLINK)
    {
        set_hard_interlock(id, DCLINK_UNDERVOLTAGE);
    }

    if(fabs(g_controller_mtoc.net_signals[id+4].f) > MAX_VLOAD)
    {
        set_hard_interlock(id, LOAD_OVERVOLTAGE);
    }

    if(fabs(g_controller_mtoc.net_signals[id+8].f) > MAX_TEMP)
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
