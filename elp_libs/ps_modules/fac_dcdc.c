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
 * @file fac_dcdc.c
 * @brief FAC DC/DC Stage module
 * 
 * Module for control of DC/DC module of FAC power supplies. It implements the
 * controller for load current.
 *
 * @author gabriel.brunheira
 * @date 01/05/2018
 *
 */

#include <float.h>

#include "boards/udc_c28.h"
#include "common/structs.h"
#include "common/timeslicer.h"
#include "control/control.h"
#include "event_manager/event_manager.h"
#include "HRADC_board/HRADC_Boards.h"
#include "ipc/ipc.h"
#include "parameters/parameters.h"
#include "pwm/pwm.h"

#include "fac_dcdc.h"

/**
 * PWM parameters
 */
#define PWM_FREQ                g_ipc_mtoc.pwm.freq_pwm
#define PWM_DEAD_TIME           g_ipc_mtoc.pwm.dead_time
#define PWM_MAX_DUTY            g_ipc_mtoc.pwm.max_duty
#define PWM_MIN_DUTY            g_ipc_mtoc.pwm.min_duty
#define PWM_MAX_DUTY_OL         g_ipc_mtoc.pwm.max_duty_openloop
#define PWM_MIN_DUTY_OL         g_ipc_mtoc.pwm.min_duty_openloop

/**
 * Control parameters
 */
#define MAX_REF                 g_ipc_mtoc.control.max_ref
#define MIN_REF                 g_ipc_mtoc.control.min_ref
#define MAX_REF                 g_ipc_mtoc.control.max_ref
#define MAX_REF_OL              g_ipc_mtoc.control.max_ref_openloop
#define MIN_REF_OL              g_ipc_mtoc.control.min_ref_openloop
#define MAX_REF_SLEWRATE        g_ipc_mtoc.control.slewrate_slowref
#define MAX_SR_SIGGEN_OFFSET    g_ipc_mtoc.control.slewrate_siggen_offset
#define MAX_SR_SIGGEN_AMP       g_ipc_mtoc.control.slewrate_siggen_amp

#define ISR_CONTROL_FREQ        g_ipc_mtoc.control.freq_isr_control

#define HRADC_FREQ_SAMP         g_ipc_mtoc.hradc.freq_hradc_sampling
#define HRADC_SPI_CLK           g_ipc_mtoc.hradc.freq_spiclk
#define NUM_HRADC_BOARDS        g_ipc_mtoc.hradc.num_hradc

/*#define TIMESLICER_WFMREF       0
#define WFMREF_FREQ             g_ipc_mtoc.control.freq_timeslicer[TIMESLICER_WFMREF]
#define WFMREF_DECIMATION       (uint16_t) roundf(ISR_CONTROL_FREQ / WFMREF_FREQ)*/

#define TIMESLICER_BUFFER       1
#define BUFFER_FREQ             g_ipc_mtoc.control.freq_timeslicer[TIMESLICER_BUFFER]
#define BUFFER_DECIMATION       (uint16_t) roundf(ISR_CONTROL_FREQ / BUFFER_FREQ)

#define SIGGEN                  g_ipc_ctom.siggen

/**
 * HRADC parameters
 */
#define HRADC_HEATER_ENABLE     g_ipc_mtoc.hradc.enable_heater
#define HRADC_MONITOR_ENABLE    g_ipc_mtoc.hradc.enable_monitor
#define TRANSDUCER_OUTPUT_TYPE  g_ipc_mtoc.hradc.type_transducer_output
#if (HRADC_v2_0)
    #define TRANSDUCER_GAIN     -g_ipc_mtoc.hradc.gain_transducer
#endif
#if (HRADC_v2_1)
    #define TRANSDUCER_GAIN     g_ipc_mtoc.hradc.gain_transducer
#endif

/**
 * Analog variables parameters
 */
#define MAX_ILOAD               g_ipc_mtoc.analog_vars.max[0]
#define MAX_VLOAD               g_ipc_mtoc.analog_vars.max[1]
#define MAX_V_CAPBANK           g_ipc_mtoc.analog_vars.max[2]
#define MIN_V_CAPBANK           g_ipc_mtoc.analog_vars.min[2]

#define NOM_V_CAPBANK_FF        g_ipc_mtoc.analog_vars.max[3]
#define MIN_V_CAPBANK_FF        g_ipc_mtoc.analog_vars.min[3]

#define MAX_TEMP_INDUCTORS      g_ipc_mtoc.analog_vars.max[4]
#define MAX_TEMP_IGBT           g_ipc_mtoc.analog_vars.max[5]

#define MAX_DCCTS_DIFF          g_ipc_mtoc.analog_vars.max[6]

#define MAX_I_IDLE_DCCT         g_ipc_mtoc.analog_vars.max[7]
#define MIN_I_ACTIVE_DCCT       g_ipc_mtoc.analog_vars.min[7]

#define NETSIGNAL_ELEM_CTOM_BUF g_ipc_mtoc.analog_vars.max[8]
#define NETSIGNAL_ELEM_MTOC_BUF g_ipc_mtoc.analog_vars.min[8]

#define NETSIGNAL_CTOM_BUF      g_controller_ctom.net_signals[(uint16_t) NETSIGNAL_ELEM_CTOM_BUF].f
#define NETSIGNAL_MTOC_BUF      g_controller_mtoc.net_signals[(uint16_t) NETSIGNAL_ELEM_MTOC_BUF].f

/**
 * Controller defines
 */
#define I_LOAD_1                        g_controller_ctom.net_signals[0].f  // HRADC0
#define I_LOAD_2                        g_controller_ctom.net_signals[1].f  // HRADC1
#define V_CAPBANK                       g_controller_ctom.net_signals[2].f  // HRADC2

#define I_LOAD_SETPOINT_FILTERED        g_controller_ctom.net_signals[3].f

#define I_LOAD_MEAN                     g_controller_ctom.net_signals[4].f
#define I_LOAD_ERROR                    g_controller_ctom.net_signals[5].f
#define I_LOAD_DIFF                     g_controller_ctom.net_signals[10].f

#define V_LOAD                          g_controller_mtoc.net_signals[0].f
#define TEMP_INDUCTORS                  g_controller_mtoc.net_signals[1].f
#define TEMP_IGBT                       g_controller_mtoc.net_signals[2].f

#define DUTY_CYCLE                      g_controller_ctom.output_signals[0].f

#define I_LOAD_SETPOINT                 g_ipc_ctom.ps_module[0].ps_setpoint
#define I_LOAD_REFERENCE                g_ipc_ctom.ps_module[0].ps_reference

#define SRLIM_I_LOAD_REFERENCE          &g_controller_ctom.dsp_modules.dsp_srlim[0]
#define ERROR_I_LOAD                    &g_controller_ctom.dsp_modules.dsp_error[0]

#define IIR_2P2Z_REFERENCE_FILTER               &g_controller_ctom.dsp_modules.dsp_iir_2p2z[0]
#define IIR_2P2Z_REFERENCE_FILTER_COEFFS        g_controller_mtoc.dsp_modules.dsp_iir_2p2z[0].coeffs.s

#define IIR_3P3Z_REFERENCE_FILTER               &g_controller_ctom.dsp_modules.dsp_iir_3p3z[0]
#define IIR_3P3Z_REFERENCE_FILTER_COEFFS        g_controller_mtoc.dsp_modules.dsp_iir_3p3z[0].coeffs.s

#define IIR_2P2Z_REFERENCE_FEEDFORWARD          &g_controller_ctom.dsp_modules.dsp_iir_2p2z[1]
#define IIR_2P2Z_REFERENCE_FEEDFORWARD_COEFFS   g_controller_mtoc.dsp_modules.dsp_iir_2p2z[1].coeffs.s

#define PI_CONTROLLER_I_LOAD                &g_controller_ctom.dsp_modules.dsp_pi[0]
#define PI_CONTROLLER_I_LOAD_COEFFS         g_controller_mtoc.dsp_modules.dsp_pi[0].coeffs.s
#define KP_I_LOAD                           PI_CONTROLLER_I_LOAD_COEFFS.kp
#define KI_I_LOAD                           PI_CONTROLLER_I_LOAD_COEFFS.ki

#define IIR_2P2Z_CONTROLLER_I_LOAD          &g_controller_ctom.dsp_modules.dsp_iir_2p2z[3]
#define IIR_2P2Z_CONTROLLER_I_LOAD_COEFFS   g_controller_mtoc.dsp_modules.dsp_iir_2p2z[3].coeffs.s

#define IIR_2P2Z_LPF_V_CAPBANK              &g_controller_ctom.dsp_modules.dsp_iir_2p2z[2]
#define IIR_2P2Z_LPF_V_CAPBANK_COEFFS       g_controller_mtoc.dsp_modules.dsp_iir_2p2z[2].coeffs.s

#define FF_V_CAPBANK                 &g_controller_ctom.dsp_modules.dsp_ff[0]

#define PWM_MODULATOR_Q1            g_pwm_modules.pwm_regs[0]
#define PWM_MODULATOR_Q2            g_pwm_modules.pwm_regs[1]

#define SRLIM_SIGGEN_AMP            &g_controller_ctom.dsp_modules.dsp_srlim[1]
#define SRLIM_SIGGEN_OFFSET         &g_controller_ctom.dsp_modules.dsp_srlim[2]

#define BUF_SAMPLES                 &g_ipc_ctom.buf_samples[0]

/**
 * Digital I/O's status
 */
#define PIN_STATUS_DCCT_1_STATUS    GET_GPDI9
#define PIN_STATUS_DCCT_1_ACTIVE    GET_GPDI10
#define PIN_STATUS_DCCT_2_STATUS    GET_GPDI11
#define PIN_STATUS_DCCT_2_ACTIVE    GET_GPDI12

/**
 * Interlocks defines
 */
typedef enum
{
    Load_Overcurrent,
    Load_Overvoltage,
    CapBank_Overvoltage,
    CapBank_Undervoltage,
    IGBT_Driver_Fault,
    IIB_Itlk
} hard_interlocks_t;

typedef enum
{
    Inductors_Overtemperature,
    IGBT_Overtemperature,
    DCCT_1_Fault,
    DCCT_2_Fault,
    DCCT_High_Difference,
    Load_Feedback_1_Fault,
    Load_Feedback_2_Fault,
} soft_interlocks_t;

#define NUM_HARD_INTERLOCKS     IGBT_Driver_Fault + 1
#define NUM_SOFT_INTERLOCKS     Load_Feedback_2_Fault + 1

/**
 *  Private variables
 */
static uint16_t decimation_factor;
static float decimation_coeff;

/**
 * Private functions
 */
#pragma CODE_SECTION(isr_init_controller, "ramfuncs");
#pragma CODE_SECTION(isr_controller, "ramfuncs");
#pragma CODE_SECTION(turn_off, "ramfuncs");

static void init_peripherals_drivers(void);
static void term_peripherals_drivers(void);

static void init_controller(void);
static void reset_controller(void);
static void enable_controller();
static void disable_controller();
static interrupt void isr_init_controller(void);
static interrupt void isr_controller(void);

static void init_interruptions(void);
static void term_interruptions(void);

static void turn_on(uint16_t dummy);
static void turn_off(uint16_t dummy);

static void reset_interlocks(uint16_t dummy);
static inline void check_interlocks(void);

/**
 * Main function for this power supply module
 */
void main_fac_dcdc(void)
{
    init_controller();
    init_peripherals_drivers();
    init_interruptions();
    enable_controller();

    /// TODO: check why first sync_pulse occurs
    g_ipc_ctom.counter_sync_pulse = 0;

    /// TODO: include condition for re-initialization
    while(1)
    {
        check_interlocks();
    }

    turn_off(0);

    disable_controller();
    term_interruptions();
    reset_controller();
    term_peripherals_drivers();
}

static void init_peripherals_drivers(void)
{
    uint16_t i;

    /// Initialization of HRADC boards
    stop_DMA();

    decimation_factor = (uint16_t) roundf(HRADC_FREQ_SAMP / ISR_CONTROL_FREQ);
    decimation_coeff = 1.0 / (float) decimation_factor;


    HRADCs_Info.enable_Sampling = 0;
    HRADCs_Info.n_HRADC_boards = NUM_HRADC_BOARDS;

    Init_DMA_McBSP_nBuffers(NUM_HRADC_BOARDS, decimation_factor, HRADC_SPI_CLK);

    Init_SPIMaster_McBSP(HRADC_SPI_CLK);
    Init_SPIMaster_Gpio();
    InitMcbspa20bit();

    DELAY_US(500000);
    send_ipc_lowpriority_msg(0,Enable_HRADC_Boards);
    DELAY_US(2000000);

    for(i = 0; i < NUM_HRADC_BOARDS; i++)
    {
        Init_HRADC_Info(&HRADCs_Info.HRADC_boards[i], i, decimation_factor,
                        buffers_HRADC[i], TRANSDUCER_GAIN[i]);
        Config_HRADC_board(&HRADCs_Info.HRADC_boards[i], TRANSDUCER_OUTPUT_TYPE[i],
                           HRADC_HEATER_ENABLE[i], HRADC_MONITOR_ENABLE[i]);
    }

    Config_HRADC_SoC(HRADC_FREQ_SAMP);

    /// Initialization of PWM modules
    g_pwm_modules.num_modules = 2;

    PWM_MODULATOR_Q1 = &EPwm1Regs;
    PWM_MODULATOR_Q2 = &EPwm2Regs;

    disable_pwm_outputs();
    disable_pwm_tbclk();
    init_pwm_mep_sfo();

    /// PWM initialization
    init_pwm_module(PWM_MODULATOR_Q1, PWM_FREQ, 0, PWM_Sync_Master, 0,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);
    init_pwm_module(PWM_MODULATOR_Q2, PWM_FREQ, 1, PWM_Sync_Slave, 180,
                    PWM_ChB_Complementary_Swapped, PWM_DEAD_TIME);

    InitEPwm1Gpio();
    InitEPwm2Gpio();

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
    /**
     * TODO: initialize WfmRef and Samples Buffer
     */

    init_ps_module(&g_ipc_ctom.ps_module[0],
                   g_ipc_mtoc.ps_module[0].ps_status.bit.model,
                   &turn_on, &turn_off, &isr_soft_interlock,
                   &isr_hard_interlock, &reset_interlocks);

    g_ipc_ctom.ps_module[1].ps_status.all = 0;
    g_ipc_ctom.ps_module[2].ps_status.all = 0;
    g_ipc_ctom.ps_module[3].ps_status.all = 0;

    init_event_manager(0, ISR_CONTROL_FREQ,
                       NUM_HARD_INTERLOCKS, NUM_SOFT_INTERLOCKS,
                       &HARD_INTERLOCKS_DEBOUNCE_TIME,
                       &HARD_INTERLOCKS_RESET_TIME,
                       &SOFT_INTERLOCKS_DEBOUNCE_TIME,
                       &SOFT_INTERLOCKS_RESET_TIME);

    init_ipc();
    init_control_framework(&g_controller_ctom);

    /***********************************************/
    /** INITIALIZATION OF SIGNAL GENERATOR MODULE **/
    /***********************************************/

    disable_siggen(&SIGGEN);

    init_siggen(&SIGGEN, ISR_CONTROL_FREQ,
                &g_ipc_ctom.ps_module[0].ps_reference);

    cfg_siggen(&SIGGEN, g_ipc_mtoc.siggen.type, g_ipc_mtoc.siggen.num_cycles,
               g_ipc_mtoc.siggen.freq, g_ipc_mtoc.siggen.amplitude,
               g_ipc_mtoc.siggen.offset, g_ipc_mtoc.siggen.aux_param);

    /**
     *        name:     SRLIM_SIGGEN_AMP
     * description:     Signal generator amplitude slew-rate limiter
     *  dsp module:     DSP_SRLim
     *          in:     g_ipc_mtoc.siggen.amplitude
     *         out:     g_ipc_ctom.siggen.amplitude
     */

    init_dsp_srlim(SRLIM_SIGGEN_AMP, MAX_SR_SIGGEN_AMP, ISR_CONTROL_FREQ,
                   &g_ipc_mtoc.siggen.amplitude, &g_ipc_ctom.siggen.amplitude);

    /**
     *        name:     SRLIM_SIGGEN_OFFSET
     * description:     Signal generator offset slew-rate limiter
     *  dsp module:     DSP_SRLim
     *          in:     g_ipc_mtoc.siggen.offset
     *         out:     g_ipc_ctom.siggen.offset
     */

    init_dsp_srlim(SRLIM_SIGGEN_OFFSET, MAX_SR_SIGGEN_OFFSET,
                   ISR_CONTROL_FREQ, &g_ipc_mtoc.siggen.offset,
                   &g_ipc_ctom.siggen.offset);

    /*************************************************/
    /** INITIALIZATION OF LOAD CURRENT CONTROL LOOP **/
    /*************************************************/

    /**
     *        name:     IIR_2P2Z_REFERENCE_FILTER
     * description:     Load current reference filter
     *  dsp module:     DSP_IIR_2P2Z
     *          in:     I_LOAD_SETPOINT
     *         out:     I_LOAD_SETPOINT_FILTERED
     */

    init_dsp_iir_2p2z(IIR_2P2Z_REFERENCE_FILTER,
                      IIR_2P2Z_REFERENCE_FILTER_COEFFS.b0,
                      IIR_2P2Z_REFERENCE_FILTER_COEFFS.b1,
                      IIR_2P2Z_REFERENCE_FILTER_COEFFS.b2,
                      IIR_2P2Z_REFERENCE_FILTER_COEFFS.a1,
                      IIR_2P2Z_REFERENCE_FILTER_COEFFS.a2,
                      //FLT_MAX, -FLT_MAX, &I_LOAD_SETPOINT,
                      //&I_LOAD_SETPOINT_FILTERED);
                      FLT_MAX, -FLT_MAX, &I_LOAD_SETPOINT_FILTERED,
                      &I_LOAD_REFERENCE);

    /**
     *        name:     IIR_3P3Z_REFERENCE_FILTER
     * description:     Load current reference filter
     *  dsp module:     DSP_IIR_3P3Z
     *          in:     I_LOAD_SETPOINT
     *         out:     I_LOAD_SETPOINT_FILTERED
     */
    init_dsp_iir_3p3z(IIR_3P3Z_REFERENCE_FILTER,
                      IIR_3P3Z_REFERENCE_FILTER_COEFFS.b0,
                      IIR_3P3Z_REFERENCE_FILTER_COEFFS.b1,
                      IIR_3P3Z_REFERENCE_FILTER_COEFFS.b2,
                      IIR_3P3Z_REFERENCE_FILTER_COEFFS.b3,
                      IIR_3P3Z_REFERENCE_FILTER_COEFFS.a1,
                      IIR_3P3Z_REFERENCE_FILTER_COEFFS.a2,
                      IIR_3P3Z_REFERENCE_FILTER_COEFFS.a3,
                      FLT_MAX, -FLT_MAX, &I_LOAD_SETPOINT_FILTERED,
                      &I_LOAD_REFERENCE);

    /**
     *        name:     SRLIM_I_LOAD_REFERENCE
     * description:     Load current slew-rate limiter
     *  dsp module:     DSP_SRLim
     *          in:     I_LOAD_SETPOINT_FILTERED
     *         out:     I_LOAD_REFERENCE
     */

    init_dsp_srlim(SRLIM_I_LOAD_REFERENCE, MAX_REF_SLEWRATE, ISR_CONTROL_FREQ,
    //               &I_LOAD_SETPOINT_FILTERED, &I_LOAD_REFERENCE);
                   &I_LOAD_SETPOINT, &I_LOAD_SETPOINT_FILTERED);

    /**
     *        name:     ERROR_I_LOAD
     * description:     Load current reference error
     *  dsp module:     DSP_Error
     *           +:     I_LOAD_REFERENCE
     *           -:     I_LOAD_MEAN
     *         out:     I_LOAD_ERROR
     */

    init_dsp_error(ERROR_I_LOAD, &I_LOAD_REFERENCE, &I_LOAD_MEAN, &I_LOAD_ERROR);

    /**
     *        name:     PI_CONTROLLER_I_LOAD
     * description:     Capacitor bank voltage PI controller
     *  dsp module:     DSP_PI
     *          in:     I_LOAD_ERROR
     *         out:     net_signals[6]
     */

    init_dsp_pi(PI_CONTROLLER_I_LOAD, KP_I_LOAD, KI_I_LOAD, ISR_CONTROL_FREQ,
                PWM_MAX_DUTY, PWM_MIN_DUTY, &I_LOAD_ERROR,
                &g_controller_ctom.net_signals[6].f);

    /**
     *        name:     IIR_2P2Z_CONTROLLER_I_LOAD
     * description:     Load current IIR 2P2Z controller
     *  dsp module:     DSP_IIR_2P2Z
     *          in:     I_LOAD_ERROR
     *         out:     net_signals[6]
     */

    init_dsp_iir_2p2z(IIR_2P2Z_CONTROLLER_I_LOAD,
                      IIR_2P2Z_CONTROLLER_I_LOAD_COEFFS.b0,
                      IIR_2P2Z_CONTROLLER_I_LOAD_COEFFS.b1,
                      IIR_2P2Z_CONTROLLER_I_LOAD_COEFFS.b2,
                      IIR_2P2Z_CONTROLLER_I_LOAD_COEFFS.a1,
                      IIR_2P2Z_CONTROLLER_I_LOAD_COEFFS.a2,
                      PWM_MAX_DUTY, PWM_MIN_DUTY, &I_LOAD_ERROR,
                      &g_controller_ctom.net_signals[6].f);

    /**
     *        name:     IIR_2P2Z_REFERENCE_FEEDFORWARD
     * description:     Load current IIR 2P2Z controller
     *  dsp module:     DSP_IIR_2P2Z
     *          in:     I_LOAD_REFERENCE
     *         out:     net_signals[7]
     */

    init_dsp_iir_2p2z(IIR_2P2Z_REFERENCE_FEEDFORWARD,
                      IIR_2P2Z_REFERENCE_FEEDFORWARD_COEFFS.b0,
                      IIR_2P2Z_REFERENCE_FEEDFORWARD_COEFFS.b1,
                      IIR_2P2Z_REFERENCE_FEEDFORWARD_COEFFS.b2,
                      IIR_2P2Z_REFERENCE_FEEDFORWARD_COEFFS.a1,
                      IIR_2P2Z_REFERENCE_FEEDFORWARD_COEFFS.a2,
                      PWM_MAX_DUTY, PWM_MIN_DUTY, &I_LOAD_REFERENCE,
                      &g_controller_ctom.net_signals[7].f);

    /**********************************************************/
    /** INITIALIZATION OF CAPACITOR BANK VOLTAGE FEEDFORWARD **/
    /**********************************************************/

    /**
     *        name:     IIR_2P2Z_LPF_V_CAPBANK
     * description:     Capacitor bank voltage low-pass filter
     *    DP class:     ELP_IIR_2P2Z
     *          in:     net_signals[2]
     *         out:     net_signals[9]
     */

    init_dsp_iir_2p2z(IIR_2P2Z_LPF_V_CAPBANK,
                      IIR_2P2Z_LPF_V_CAPBANK_COEFFS.b0,
                      IIR_2P2Z_LPF_V_CAPBANK_COEFFS.b1,
                      IIR_2P2Z_LPF_V_CAPBANK_COEFFS.b2,
                      IIR_2P2Z_LPF_V_CAPBANK_COEFFS.a1,
                      IIR_2P2Z_LPF_V_CAPBANK_COEFFS.a2,
                      FLT_MAX, -FLT_MAX, &g_controller_ctom.net_signals[2].f,
                      &g_controller_ctom.net_signals[9].f);

    /**
     *        name:     FF_V_CAPBANK
     * description:     Capacitor bank voltage feed-forward
     *    DP class:     DSP_VdcLink_FeedForward
     *    vdc_meas:     net_signals[9]
     *          in:     net_signals[8]
     *         out:     DUTY_CYCLE
     */

    init_dsp_vdclink_ff(FF_V_CAPBANK, NOM_V_CAPBANK_FF, MIN_V_CAPBANK_FF,
                        &g_controller_ctom.net_signals[9].f,
                        &g_controller_ctom.net_signals[8].f, &DUTY_CYCLE);

    /************************************/
    /** INITIALIZATION OF TIME SLICERS **/
    /************************************/

    /**
     * Time-slicer for WfmRef sweep decimation
     */
    cfg_timeslicer(TIMESLICER_WFMREF, WFMREF_DECIMATION);

    /**
     * Time-slicer for SamplesBuffer
     */
    cfg_timeslicer(TIMESLICER_BUFFER, BUFFER_DECIMATION);

    /**
     * Samples buffer initialization
     */
    init_buffer(BUF_SAMPLES, &g_buf_samples_ctom, SIZE_BUF_SAMPLES_CTOM);
    enable_buffer(BUF_SAMPLES);

    /**
     * Reset all internal variables
     */
    reset_controller();
}

/**
 * Reset all internal variables from controller
 */
static void reset_controller(void)
{
    set_pwm_duty_hbridge(PWM_MODULATOR_Q1, 0.0);

    I_LOAD_SETPOINT = 0.0;
    I_LOAD_REFERENCE = 0.0;

    reset_dsp_iir_2p2z(IIR_2P2Z_REFERENCE_FILTER);
    reset_dsp_iir_3p3z(IIR_3P3Z_REFERENCE_FILTER);
    reset_dsp_srlim(SRLIM_I_LOAD_REFERENCE);

    reset_dsp_error(ERROR_I_LOAD);
    reset_dsp_pi(PI_CONTROLLER_I_LOAD);
    reset_dsp_iir_2p2z(IIR_2P2Z_CONTROLLER_I_LOAD);

    reset_dsp_iir_2p2z(IIR_2P2Z_REFERENCE_FEEDFORWARD);

    reset_dsp_iir_2p2z(IIR_2P2Z_LPF_V_CAPBANK);
    reset_dsp_vdclink_ff(FF_V_CAPBANK);

    reset_dsp_srlim(SRLIM_SIGGEN_AMP);
    reset_dsp_srlim(SRLIM_SIGGEN_OFFSET);
    disable_siggen(&SIGGEN);

    reset_timeslicers();
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

    reset_controller();
}

/**
 * ISR for control initialization
 */
static interrupt void isr_init_controller(void)
{
    EALLOW;
    PieVectTable.EPWM1_INT = &isr_controller;
    EDIS;

    PWM_MODULATOR_Q1->ETSEL.bit.INTSEL = ET_CTR_ZERO;
    PWM_MODULATOR_Q1->ETCLR.bit.INT = 1;

    PWM_MODULATOR_Q2->ETSEL.bit.INTSEL = ET_CTR_ZERO;
    PWM_MODULATOR_Q2->ETCLR.bit.INT = 1;

    PieCtrlRegs.PIEACK.all |= M_INT3;
}

/**
 * Control ISR
 */
interrupt void isr_controller(void)
{
    static float temp[4];
    static uint16_t i;

    //CLEAR_DEBUG_GPIO1;
    SET_DEBUG_GPIO1;

    temp[0] = 0.0;
    temp[1] = 0.0;
    temp[2] = 0.0;
    temp[3] = 0.0;

    /// Get HRADC samples
    for(i = 0; i < decimation_factor; i++)
    {
        temp[0] += (float) *(HRADCs_Info.HRADC_boards[0].SamplesBuffer++);
        temp[1] += (float) *(HRADCs_Info.HRADC_boards[1].SamplesBuffer++);
        temp[2] += (float) *(HRADCs_Info.HRADC_boards[2].SamplesBuffer++);
        temp[3] += (float) *(HRADCs_Info.HRADC_boards[3].SamplesBuffer++);
    }

    //CLEAR_DEBUG_GPIO1;

    HRADCs_Info.HRADC_boards[0].SamplesBuffer = buffers_HRADC[0];
    HRADCs_Info.HRADC_boards[1].SamplesBuffer = buffers_HRADC[1];
    HRADCs_Info.HRADC_boards[2].SamplesBuffer = buffers_HRADC[2];
    HRADCs_Info.HRADC_boards[3].SamplesBuffer = buffers_HRADC[3];

    temp[0] *= HRADCs_Info.HRADC_boards[0].gain * decimation_coeff;
    temp[0] += HRADCs_Info.HRADC_boards[0].offset;

    temp[1] *= HRADCs_Info.HRADC_boards[1].gain * decimation_coeff;
    temp[1] += HRADCs_Info.HRADC_boards[1].offset;

    temp[2] *= HRADCs_Info.HRADC_boards[2].gain * decimation_coeff;
    temp[2] += HRADCs_Info.HRADC_boards[2].offset;

    temp[3] *= HRADCs_Info.HRADC_boards[3].gain * decimation_coeff;
    temp[3] += HRADCs_Info.HRADC_boards[3].offset;

    I_LOAD_1 = temp[0];
    I_LOAD_2 = temp[1];
    I_LOAD_MEAN = 0.5*(I_LOAD_1 + I_LOAD_2);
    I_LOAD_DIFF = I_LOAD_1 - I_LOAD_2;

    g_controller_ctom.net_signals[2].f = temp[2];
    g_controller_ctom.net_signals[20].f = temp[3];

    run_dsp_iir_2p2z(IIR_2P2Z_LPF_V_CAPBANK);

    /// Check whether power supply is ON
    if(g_ipc_ctom.ps_module[0].ps_status.bit.state > Interlock)
    {
        /// Calculate reference according to operation mode
        switch(g_ipc_ctom.ps_module[0].ps_status.bit.state)
        {
            case SlowRef:
            case SlowRefSync:
            {
                //run_dsp_iir_2p2z(IIR_2P2Z_REFERENCE_FILTER);
                run_dsp_srlim(SRLIM_I_LOAD_REFERENCE, USE_MODULE);
                break;
            }
            case Cycle:
            {
                run_dsp_srlim(SRLIM_SIGGEN_AMP, USE_MODULE);
                run_dsp_srlim(SRLIM_SIGGEN_OFFSET, USE_MODULE);
                SIGGEN.p_run_siggen(&SIGGEN);
                break;
            }
            case RmpWfm:
            {
                switch(WFMREF.sync_mode)
                {
                    case OneShot:
                    {   /*********************************************/
                        RUN_TIMESLICER(TIMESLICER_WFMREF)
                            if( WFMREF.wfmref_data.p_buf_idx <=
                                WFMREF.wfmref_data.p_buf_end)
                            {
                                I_LOAD_SETPOINT_FILTERED =
                                //I_LOAD_REFERENCE =
                                        *(WFMREF.wfmref_data.p_buf_idx++) *
                                        (WFMREF.gain) + WFMREF.offset;
                            }
                        END_TIMESLICER(TIMESLICER_WFMREF)
                        /*********************************************/
                        break;
                    }

                    case SampleBySample:
                    case SampleBySample_OneCycle:
                    {
                        if(WFMREF.wfmref_data.p_buf_idx <= WFMREF.wfmref_data.p_buf_end)
                        {
                            I_LOAD_SETPOINT_FILTERED = *(WFMREF.wfmref_data.p_buf_idx) *
                            //I_LOAD_REFERENCE =  *(WFMREF.wfmref_data.p_buf_idx) *
                                                 (WFMREF.gain) + WFMREF.offset;
                        }
                        break;
                    }
                }
                break;
            }
            case MigWfm:
            {
                break;
            }
            default:
            {
                break;
            }
        }

        /// Open-loop
        if(g_ipc_ctom.ps_module[0].ps_status.bit.openloop)
        {
            I_LOAD_REFERENCE = I_LOAD_SETPOINT_FILTERED;
            SATURATE(I_LOAD_REFERENCE, MAX_REF_OL, MIN_REF_OL);
            DUTY_CYCLE = 0.01 * I_LOAD_REFERENCE;
            SATURATE(DUTY_CYCLE, PWM_MAX_DUTY_OL, PWM_MIN_DUTY_OL);
        }
        /// Closed-loop
        else
        {
            //run_dsp_iir_2p2z(IIR_2P2Z_REFERENCE_FILTER);
            run_dsp_iir_3p3z(IIR_3P3Z_REFERENCE_FILTER);
            SATURATE(I_LOAD_REFERENCE, MAX_REF, MIN_REF);
            run_dsp_error(ERROR_I_LOAD);
            run_dsp_pi(PI_CONTROLLER_I_LOAD);
            //run_dsp_iir_2p2z(IIR_2P2Z_CONTROLLER_I_LOAD);
            run_dsp_iir_2p2z(IIR_2P2Z_REFERENCE_FEEDFORWARD);

            g_controller_ctom.net_signals[8].f = g_controller_ctom.net_signals[6].f +
                                               g_controller_ctom.net_signals[7].f;

            run_dsp_vdclink_ff(FF_V_CAPBANK);

            SATURATE(DUTY_CYCLE, PWM_MAX_DUTY, PWM_MIN_DUTY);
        }

        set_pwm_duty_hbridge(PWM_MODULATOR_Q1, DUTY_CYCLE);
    }

    /*********************************************/
    RUN_TIMESLICER(TIMESLICER_BUFFER)
    /*********************************************/
        //insert_buffer(BUF_SAMPLES, g_controller_ctom.net_signals[3].f);
        //insert_buffer(BUF_SAMPLES, I_LOAD_REFERENCE);
        insert_buffer(BUF_SAMPLES, NETSIGNAL_CTOM_BUF);
        //insert_buffer(BUF_SAMPLES, I_LOAD_2);
        //insert_buffer(BUF_SAMPLES, g_controller_ctom.net_signals[10].f);
    /*********************************************/
    END_TIMESLICER(TIMESLICER_BUFFER)
    /*********************************************/

    SET_INTERLOCKS_TIMEBASE_FLAG(0);

    PWM_MODULATOR_Q1->ETCLR.bit.INT = 1;
    PWM_MODULATOR_Q2->ETCLR.bit.INT = 1;

    PieCtrlRegs.PIEACK.all |= M_INT3;

    CLEAR_DEBUG_GPIO1;
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

    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
    enable_pwm_interrupt(PWM_MODULATOR_Q1);
    enable_pwm_interrupt(PWM_MODULATOR_Q2);

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
    disable_pwm_interrupt(PWM_MODULATOR_Q1);
    disable_pwm_interrupt(PWM_MODULATOR_Q2);

    /// Clear flags
    PieCtrlRegs.PIEACK.all |= M_INT1 | M_INT3 | M_INT11;
}

/**
 * Turn power supply on.
 *
 * @param dummy dummy argument due to ps_module pointer
 */
static void turn_on(uint16_t dummy)
{
    #ifdef USE_ITLK
    if(g_ipc_ctom.ps_module[0].ps_status.bit.state == Off)
    #else
    if(g_ipc_ctom.ps_module[0].ps_status.bit.state <= Interlock)
    #endif
    {
        if(V_CAPBANK < MIN_V_CAPBANK)
        {
            set_hard_interlock(0, CapBank_Undervoltage);
        }

        #ifdef USE_ITLK
        else
        {
        #endif
            reset_controller();

            g_ipc_ctom.ps_module[0].ps_status.bit.openloop = OPEN_LOOP;
            g_ipc_ctom.ps_module[0].ps_status.bit.state = SlowRef;
            enable_pwm_output(0);
            enable_pwm_output(1);
        #ifdef USE_ITLK
        }
        #endif
    }
}

/**
 * Turn off specified power supply.
 *
 * @param dummy dummy argument due to ps_module pointer
 */
static void turn_off(uint16_t dummy)
{
    disable_pwm_output(0);
    disable_pwm_output(1);

    reset_controller();

    if(g_ipc_ctom.ps_module[0].ps_status.bit.state != Interlock)
    {
        g_ipc_ctom.ps_module[0].ps_status.bit.state = Off;
    }
}

/**
 * Reset interlocks for specified power supply.
 *
 * @param dummy dummy argument due to ps_module pointer
 */
static void reset_interlocks(uint16_t dummy)
{
    g_ipc_ctom.ps_module[0].ps_hard_interlock = 0;
    g_ipc_ctom.ps_module[0].ps_soft_interlock = 0;

    if(g_ipc_ctom.ps_module[0].ps_status.bit.state < Initializing)
    {
        g_ipc_ctom.ps_module[0].ps_status.bit.state = Off;
    }
}

/**
 * Check interlocks of this specific power supply topology
 */
static inline void check_interlocks(void)
{
    if(fabs(I_LOAD_MEAN) > MAX_ILOAD)
    {
        set_hard_interlock(0, Load_Overcurrent);
    }

    if(fabs(I_LOAD_DIFF) > MAX_DCCTS_DIFF)
    {
        set_soft_interlock(0, DCCT_High_Difference);
    }

    if(V_CAPBANK > MAX_V_CAPBANK)
    {
        set_hard_interlock(0, CapBank_Overvoltage);
    }

    if(!PIN_STATUS_DCCT_1_STATUS)
    {
        set_soft_interlock(0, DCCT_1_Fault);
    }

    if(!PIN_STATUS_DCCT_2_STATUS)
    {
        set_soft_interlock(0, DCCT_2_Fault);
    }

    if(PIN_STATUS_DCCT_1_ACTIVE)
    {
        if(fabs(I_LOAD_1) < MIN_I_ACTIVE_DCCT)
        {
            set_soft_interlock(0, Load_Feedback_1_Fault);
        }
    }
    else
    {
        if(fabs(I_LOAD_1) > MAX_I_IDLE_DCCT)
        {
            set_soft_interlock(0, Load_Feedback_1_Fault);
        }
    }

    if(PIN_STATUS_DCCT_2_ACTIVE)
    {
        if(fabs(I_LOAD_2) < MIN_I_ACTIVE_DCCT)
        {
            set_soft_interlock(0, Load_Feedback_2_Fault);
        }
    }
    else
    {
        if(fabs(I_LOAD_2) > MAX_I_IDLE_DCCT)
        {
            set_soft_interlock(0, Load_Feedback_2_Fault);
        }
    }

    DINT;

    if ( (g_ipc_ctom.ps_module[0].ps_status.bit.state > Interlock)
              && (V_CAPBANK < MIN_V_CAPBANK) )
    {
        set_hard_interlock(0, CapBank_Undervoltage);
    }

    EINT;

    //SET_DEBUG_GPIO1;
    run_interlocks_debouncing(0);
    //CLEAR_DEBUG_GPIO1;
}
