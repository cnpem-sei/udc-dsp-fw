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
 * @file fac_acdc.c
 * @brief FAC AC/DC Stage module
 * 
 * Module for control of AC/DC module of FAC power supplies. It implements the
 * controller for input current and capacitor bank voltage.
 *
 * @author gabriel.brunheira
 * @date 10/04/2018
 *
 */

#include <float.h>

#include "boards/udc_c28.h"
#include "common/structs.h"
#include "common/timeslicer.h"
#include "control/control.h"
#include "HRADC_board/HRADC_Boards.h"
#include "ipc/ipc.h"
#include "parameters/parameters.h"
#include "pwm/pwm.h"

#include "fac_acdc.h"

#define USE_ITLK

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
#define MAX_REF_OL              g_ipc_mtoc.control.max_ref_openloop
#define MIN_REF_OL              g_ipc_mtoc.control.min_ref_openloop
#define MAX_REF_SLEWRATE        g_ipc_mtoc.control.slewrate_slowref
#define MAX_SR_SIGGEN_OFFSET    g_ipc_mtoc.control.slewrate_siggen_offset
#define MAX_SR_SIGGEN_AMP       g_ipc_mtoc.control.slewrate_siggen_amp

#define CONTROL_FREQ            g_ipc_mtoc.control.freq_isr_control

#define HRADC_FREQ_SAMP         g_ipc_mtoc.hradc.freq_hradc_sampling
#define HRADC_SPI_CLK           g_ipc_mtoc.hradc.freq_spiclk
#define NUM_HRADC_BOARDS        g_ipc_mtoc.hradc.num_hradc

#define TIMESLICER_BUFFER       1
#define BUFFER_FREQ             g_ipc_mtoc.control.freq_timeslicer[TIMESLICER_BUFFER]
#define BUFFER_DECIMATION       (uint16_t) roundf(CONTROL_FREQ / BUFFER_FREQ)

#define TIMESLICER_V_CAPBANK_CONTROLLER     2
#define V_CAPBANK_CONTROLLER_FREQ           g_ipc_mtoc.control.freq_timeslicer[TIMESLICER_V_CAPBANK_CONTROLLER]
#define V_CAPBANK_CONTROLLER_DECIMATION     (uint16_t) roundf(CONTROL_FREQ / V_CAPBANK_CONTROLLER_FREQ)

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
#define MAX_V_CAPBANK           g_ipc_mtoc.analog_vars.max[0]

#define MAX_VOUT_RECT           g_ipc_mtoc.analog_vars.max[1]
#define MAX_IOUT_RECT           g_ipc_mtoc.analog_vars.max[2]
#define MAX_IOUT_RECT_REF       g_ipc_mtoc.analog_vars.max[3]
#define MIN_IOUT_RECT_REF       g_ipc_mtoc.analog_vars.min[3]

#define MAX_TEMP_HEATSINK       g_ipc_mtoc.analog_vars.max[4]
#define MAX_TEMP_INDUCTORS      g_ipc_mtoc.analog_vars.max[5]

#define TIMEOUT_AC_MAINS_CONTACTOR_CLOSED_MS   g_ipc_mtoc.analog_vars.max[6]
#define TIMEOUT_AC_MAINS_CONTACTOR_OPENED_MS   g_ipc_mtoc.analog_vars.max[7]

/**
 * Power supply defines
 */
#define PIN_OPEN_AC_MAINS_CONTACTOR     CLEAR_GPDO1;
#define PIN_CLOSE_AC_MAINS_CONTACTOR    SET_GPDO1;

#define PIN_STATUS_AC_MAINS_CONTACTOR   GET_GPDI5

#define V_CAPBANK                       g_controller_ctom.net_signals[0].f  // HRADC0
#define IOUT_RECT                       g_controller_ctom.net_signals[1].f  // HRADC1

#define VOUT_RECT                       g_controller_mtoc.net_signals[0].f
#define TEMP_HEATSINK                   g_controller_mtoc.net_signals[1].f
#define TEMP_INDUCTORS                  g_controller_mtoc.net_signals[2].f

#define DUTY_CYCLE                      g_controller_ctom.output_signals[0].f

#define V_CAPBANK_SETPOINT              g_ipc_ctom.ps_module[0].ps_setpoint
#define V_CAPBANK_REFERENCE             g_ipc_ctom.ps_module[0].ps_reference

#define SRLIM_V_CAPBANK_REFERENCE       &g_controller_ctom.dsp_modules.dsp_srlim[0]
#define ERROR_V_CAPBANK                 &g_controller_ctom.dsp_modules.dsp_error[0]

#define PI_CONTROLLER_V_CAPBANK         &g_controller_ctom.dsp_modules.dsp_pi[0]
#define PI_CONTROLLER_V_CAPBANK_COEFFS  g_controller_mtoc.dsp_modules.dsp_pi[0].coeffs.s
#define KP_V_CAPBANK                    PI_CONTROLLER_V_CAPBANK_COEFFS.kp
#define KI_V_CAPBANK                    PI_CONTROLLER_V_CAPBANK_COEFFS.ki

#define IIR_2P2Z_CONTROLLER_V_CAPBANK           &g_controller_ctom.dsp_modules.dsp_iir_2p2z[2]
#define IIR_2P2Z_CONTROLLER_V_CAPBANK_COEFFS    g_controller_mtoc.dsp_modules.dsp_iir_2p2z[2].coeffs.s

#define NOTCH_FILT_2HZ_V_CAPBANK                &g_controller_ctom.dsp_modules.dsp_iir_2p2z[0]
#define NOTCH_FILT_2HZ_V_CAPBANK_COEFFS         g_controller_ctom.dsp_modules.dsp_iir_2p2z[0].coeffs.s
#define NOTCH_FILT_4HZ_V_CAPBANK                &g_controller_ctom.dsp_modules.dsp_iir_2p2z[1]
#define NOTCH_FILT_4HZ_V_CAPBANK_COEFFS         g_controller_ctom.dsp_modules.dsp_iir_2p2z[1].coeffs.s
#define NF_ALPHA                                0.99

#define ERROR_IOUT_RECT                 &g_controller_ctom.dsp_modules.dsp_error[1]
#define PI_CONTROLLER_IOUT_RECT         &g_controller_ctom.dsp_modules.dsp_pi[1]
#define PI_CONTROLLER_IOUT_RECT_COEFFS  g_controller_mtoc.dsp_modules.dsp_pi[1].coeffs.s
#define KP_IOUT_RECT                    PI_CONTROLLER_IOUT_RECT_COEFFS.kp
#define KI_IOUT_RECT                    PI_CONTROLLER_IOUT_RECT_COEFFS.ki

#define IIR_2P2Z_CONTROLLER_IOUT_RECT           &g_controller_ctom.dsp_modules.dsp_iir_2p2z[3]
#define IIR_2P2Z_CONTROLLER_IOUT_RECT_COEFFS    g_controller_mtoc.dsp_modules.dsp_iir_2p2z[3].coeffs.s

#define PWM_MODULATOR                   g_pwm_modules.pwm_regs[0]

#define SRLIM_SIGGEN_AMP                &g_controller_ctom.dsp_modules.dsp_srlim[1]
#define SRLIM_SIGGEN_OFFSET             &g_controller_ctom.dsp_modules.dsp_srlim[2]

/**
 * Interlock defines
 */
#define CAPBANK_OVERVOLTAGE         0x00000001
#define RECTIFIER_OVERVOLTAGE       0x00000002
#define RECTIFIER_UNDERVOLTAGE      0x00000004
#define RECTIFIER_OVERCURRENT       0x00000008
#define AC_MAINS_CONTACTOR_FAIL     0x00000010
#define IGBT_DRIVER_FAIL            0x00000020

#define HEATSINK_OVERTEMP           0x00000001
#define INDUCTORS_OVERTEMP          0x00000002

#define BUF_SAMPLES                 &g_ipc_ctom.buf_samples[0]

static float decimation_factor;
static float decimation_coeff;

/**
 * Private functions
 */
#pragma CODE_SECTION(isr_init_controller, "ramfuncs");
#pragma CODE_SECTION(isr_controller, "ramfuncs");
#pragma CODE_SECTION(turn_off, "ramfuncs");
#pragma CODE_SECTION(set_hard_interlock, "ramfuncs");
#pragma CODE_SECTION(set_soft_interlock, "ramfuncs");
#pragma CODE_SECTION(isr_hard_interlock, "ramfuncs");
#pragma CODE_SECTION(isr_soft_interlock, "ramfuncs");

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
static void set_hard_interlock(uint32_t itlk);
static void set_soft_interlock(uint32_t itlk);
static interrupt void isr_hard_interlock(void);
static interrupt void isr_soft_interlock(void);

static inline void check_interlocks(void);

/**
 * Main function for this power supply module
 */
void main_fac_acdc(void)
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

    decimation_factor = HRADC_FREQ_SAMP / CONTROL_FREQ;
    decimation_coeff = 1.0 / decimation_factor;


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
    g_pwm_modules.num_modules = 1;

    PWM_MODULATOR = &EPwm1Regs;

    disable_pwm_outputs();
    disable_pwm_tbclk();
    init_pwm_mep_sfo();

    /// PWM initialization
    init_pwm_module(PWM_MODULATOR, PWM_FREQ, 0, PWM_Sync_Master, 0,
                    PWM_ChB_Independent, PWM_DEAD_TIME);

    InitEPwm1Gpio();

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

    init_ipc();
    init_control_framework(&g_controller_ctom);

    /***********************************************/
    /** INITIALIZATION OF SIGNAL GENERATOR MODULE **/
    /***********************************************/

    disable_siggen(&g_ipc_ctom.siggen);

    init_siggen(&g_ipc_ctom.siggen, V_CAPBANK_CONTROLLER_FREQ,
                &g_ipc_ctom.ps_module[0].ps_reference);

    cfg_siggen(&g_ipc_ctom.siggen, g_ipc_mtoc.siggen.type,
               g_ipc_mtoc.siggen.num_cycles, g_ipc_mtoc.siggen.freq,
               g_ipc_mtoc.siggen.amplitude, g_ipc_mtoc.siggen.offset,
               g_ipc_mtoc.siggen.aux_param);

    /**
     *        name:     SRLIM_SIGGEN_AMP
     * description:     Signal generator amplitude slew-rate limiter
     *    DP class:     DSP_SRLim
     *          in:     g_ipc_mtoc.siggen.amplitude
     *         out:     g_ipc_ctom.siggen.amplitude
     */

    init_dsp_srlim(SRLIM_SIGGEN_AMP, MAX_SR_SIGGEN_AMP,
                   V_CAPBANK_CONTROLLER_FREQ, &g_ipc_mtoc.siggen.amplitude,
                   &g_ipc_ctom.siggen.amplitude);

    /**
     *        name:     SRLIM_SIGGEN_OFFSET
     * description:     Signal generator offset slew-rate limiter
     *    DP class:     DSP_SRLim
     *          in:     g_ipc_mtoc.siggen.offset
     *         out:     g_ipc_ctom.siggen.offset
     */

    init_dsp_srlim(SRLIM_SIGGEN_OFFSET, MAX_SR_SIGGEN_OFFSET,
                   V_CAPBANK_CONTROLLER_FREQ, &g_ipc_mtoc.siggen.offset,
                   &g_ipc_ctom.siggen.offset);

    /***********************************************************/
    /** INITIALIZATION OF CAPACITOR BANK VOLTAGE CONTROL LOOP **/
    /***********************************************************/

    /**
     *        name:     SRLIM_V_CAPBANK_REFERENCE
     * description:     Capacitor bank voltage reference slew-rate limiter
     *    DP class:     DSP_SRLim
     *          in:     V_CAPBANK_SETPOINT
     *         out:     V_CAPBANK_REFERENCE
     */

    init_dsp_srlim(SRLIM_V_CAPBANK_REFERENCE, MAX_REF_SLEWRATE,
                   V_CAPBANK_CONTROLLER_FREQ, &V_CAPBANK_SETPOINT,
                   &V_CAPBANK_REFERENCE);

    /**
     *        name:     ERROR_V_CAPBANK
     * description:     Capacitor bank voltage reference error
     *  dsp module:     DSP_Error
     *           +:     ps_module[0].ps_reference
     *           -:     net_signals[0]
     *         out:     net_signals[4]
     */

    init_dsp_error(ERROR_V_CAPBANK, &V_CAPBANK_REFERENCE, &V_CAPBANK,
                   &g_controller_ctom.net_signals[4].f);

    /**
     *        name:     PI_CONTROLLER_V_CAPBANK
     * description:     Capacitor bank voltage PI controller
     *  dsp module:     DSP_PI
     *          in:     net_signals[4]
     *         out:     net_signals[5]
     */

    init_dsp_pi(PI_CONTROLLER_V_CAPBANK, KP_V_CAPBANK, KI_V_CAPBANK,
                V_CAPBANK_CONTROLLER_FREQ, MAX_IOUT_RECT_REF, MIN_IOUT_RECT_REF,
                &g_controller_ctom.net_signals[4].f,
                &g_controller_ctom.net_signals[5].f);

    /**
     *        name:     IIR_2P2Z_CONTROLLER_V_CAPBANK
     * description:     Cap bank voltage IIR 2P2Z controller
     *    DP class:     DSP_IIR_2P2Z
     *          in:     net_signals[4]
     *         out:     net_signals[5]
     */

    init_dsp_iir_2p2z(IIR_2P2Z_CONTROLLER_V_CAPBANK,
                      IIR_2P2Z_CONTROLLER_V_CAPBANK_COEFFS.b0,
                      IIR_2P2Z_CONTROLLER_V_CAPBANK_COEFFS.b1,
                      IIR_2P2Z_CONTROLLER_V_CAPBANK_COEFFS.b2,
                      IIR_2P2Z_CONTROLLER_V_CAPBANK_COEFFS.a1,
                      IIR_2P2Z_CONTROLLER_V_CAPBANK_COEFFS.a2,
                      MAX_IOUT_RECT_REF, MIN_IOUT_RECT_REF,
                      &g_controller_ctom.net_signals[4].f,
                      &g_controller_ctom.net_signals[5].f);

    /**
     *        name:     NOTCH_FILT_2HZ_V_CAPBANK
     * description:     Cap bank voltage notch filter (Fcut = 2 Hz)
     *    DP class:     DSP_IIR_2P2Z
     *          in:     net_signals[0]
     *         out:     net_signals[2]
     */

    init_dsp_notch_2p2z(NOTCH_FILT_2HZ_V_CAPBANK, NF_ALPHA, 2.0,
                        V_CAPBANK_CONTROLLER_FREQ, FLT_MAX, -FLT_MAX,
                        &g_controller_ctom.net_signals[0].f,
                        &g_controller_ctom.net_signals[2].f);

    /*init_dsp_iir_2p2z(NOTCH_FILT_2HZ_V_CAPBANK,
                      NOTCH_FILT_2HZ_V_CAPBANK_COEFFS.b0,
                      NOTCH_FILT_2HZ_V_CAPBANK_COEFFS.b1,
                      NOTCH_FILT_2HZ_V_CAPBANK_COEFFS.b2,
                      NOTCH_FILT_2HZ_V_CAPBANK_COEFFS.a1,
                      NOTCH_FILT_2HZ_V_CAPBANK_COEFFS.a2,
                      FLT_MAX, -FLT_MAX,
                      &g_controller_ctom.net_signals[0].f,
                      &g_controller_ctom.net_signals[2].f);*/

    /**
     *        name:     NOTCH_FILT_4HZ_V_CAPBANK
     * description:     Cap bank voltage notch filter (Fcut = 4 Hz)
     *    DP class:     DSP_IIR_2P2Z
     *          in:     net_signals[2]
     *         out:     net_signals[3]
     */

    init_dsp_notch_2p2z(NOTCH_FILT_4HZ_V_CAPBANK, NF_ALPHA, 4.0,
                        V_CAPBANK_CONTROLLER_FREQ, FLT_MAX, -FLT_MAX,
                        &g_controller_ctom.net_signals[2].f,
                        &g_controller_ctom.net_signals[3].f);

    /*init_dsp_iir_2p2z(NOTCH_FILT_4HZ_V_CAPBANK,
                      NOTCH_FILT_4HZ_V_CAPBANK_COEFFS.b0,
                      NOTCH_FILT_4HZ_V_CAPBANK_COEFFS.b1,
                      NOTCH_FILT_4HZ_V_CAPBANK_COEFFS.b2,
                      NOTCH_FILT_4HZ_V_CAPBANK_COEFFS.a1,
                      NOTCH_FILT_4HZ_V_CAPBANK_COEFFS.a2,
                      FLT_MAX, -FLT_MAX,
                      &g_controller_ctom.net_signals[2].f,
                      &g_controller_ctom.net_signals[3].f);*/

    /*************************************************************/
    /** INITIALIZATION OF RECTIFIER OUTPUT CURRENT CONTROL LOOP **/
    /*************************************************************/

    /**
     *        name:     ERROR_IOUT_RECT
     * description:     Rectifier output current reference error
     *    DP class:     DSP_Error
     *           +:     net_signals[5]
     *           -:     net_signals[1]
     *         out:     net_signals[6]
     */

    init_dsp_error(ERROR_IOUT_RECT, &g_controller_ctom.net_signals[5].f,
                   &IOUT_RECT, &g_controller_ctom.net_signals[6].f);

    /**
     *        name:     PI_CONTROLLER_IOUT_RECT
     * description:     Rectifier output current PI controller
     *    DP class:     ELP_PI_dawu
     *          in:     net_signals[6]
     *         out:     output_signals[0]
     */
    init_dsp_pi(PI_CONTROLLER_IOUT_RECT, KP_IOUT_RECT, KI_IOUT_RECT,
                CONTROL_FREQ, PWM_MAX_DUTY, PWM_MIN_DUTY,
                &g_controller_ctom.net_signals[6].f, &DUTY_CYCLE);

    /*init_dsp_pi(PI_CONTROLLER_IOUT_RECT, KP_IOUT_RECT, KI_IOUT_RECT,
                CONTROL_FREQ, PWM_MAX_DUTY, -PWM_MAX_DUTY,
                &g_controller_ctom.net_signals[6], &DUTY_CYCLE);*/

    /**
     *        name:     IIR_2P2Z_CONTROLLER_IOUT_RECT
     * description:     Rectifier output current IIR 2P2Z controller
     *    DP class:     ELP_IIR_2P2Z
     *          in:     net_signals[6]
     *         out:     output_signals[0]
     */

    init_dsp_iir_2p2z(IIR_2P2Z_CONTROLLER_IOUT_RECT,
                      IIR_2P2Z_CONTROLLER_IOUT_RECT_COEFFS.b0,
                      IIR_2P2Z_CONTROLLER_IOUT_RECT_COEFFS.b1,
                      IIR_2P2Z_CONTROLLER_IOUT_RECT_COEFFS.b2,
                      IIR_2P2Z_CONTROLLER_IOUT_RECT_COEFFS.a1,
                      IIR_2P2Z_CONTROLLER_IOUT_RECT_COEFFS.a2,
                      PWM_MAX_DUTY, PWM_MIN_DUTY,
                      &g_controller_ctom.net_signals[0].f,
                      &DUTY_CYCLE);

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
     * Time-slicer for Rectifier Output Current Controller
     */
    cfg_timeslicer(TIMESLICER_V_CAPBANK_CONTROLLER,
                   V_CAPBANK_CONTROLLER_DECIMATION);

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
    set_pwm_duty_chA(PWM_MODULATOR, 0.0);

    g_ipc_ctom.ps_module[0].ps_setpoint = 0.0;
    g_ipc_ctom.ps_module[0].ps_reference = 0.0;

    reset_dsp_srlim(SRLIM_V_CAPBANK_REFERENCE);
    reset_dsp_error(ERROR_V_CAPBANK);
    reset_dsp_pi(PI_CONTROLLER_V_CAPBANK);
    reset_dsp_iir_2p2z(NOTCH_FILT_2HZ_V_CAPBANK);
    reset_dsp_iir_2p2z(NOTCH_FILT_4HZ_V_CAPBANK);
    reset_dsp_iir_2p2z(IIR_2P2Z_CONTROLLER_V_CAPBANK);

    reset_dsp_error(ERROR_IOUT_RECT);
    reset_dsp_pi(PI_CONTROLLER_IOUT_RECT);
    reset_dsp_iir_2p2z(IIR_2P2Z_CONTROLLER_IOUT_RECT);

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

    PWM_MODULATOR->ETSEL.bit.INTSEL = ET_CTR_ZERO;
    PWM_MODULATOR->ETCLR.bit.INT = 1;

    PieCtrlRegs.PIEACK.all |= M_INT3;
}

/**
 * Control ISR
 */
static interrupt void isr_controller(void)
{
    static float temp[4];
    static uint16_t i;

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

    CLEAR_DEBUG_GPIO1;

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

    V_CAPBANK = temp[0];
    IOUT_RECT = temp[1];
    g_controller_ctom.net_signals[10].f = temp[2];
    g_controller_ctom.net_signals[11].f = temp[3];

    /*********************************************/
    RUN_TIMESLICER(TIMESLICER_V_CAPBANK_CONTROLLER)
    /*********************************************/

        run_dsp_iir_2p2z(NOTCH_FILT_2HZ_V_CAPBANK);
        run_dsp_iir_2p2z(NOTCH_FILT_4HZ_V_CAPBANK);

        /// Check whether power supply is ON
        if(g_ipc_ctom.ps_module[0].ps_status.bit.state > Interlock)
        {
            /// Calculate reference according to operation mode
            switch(g_ipc_ctom.ps_module[0].ps_status.bit.state)
            {
                case SlowRef:
                case SlowRefSync:
                {
                    run_dsp_srlim(SRLIM_V_CAPBANK_REFERENCE, USE_MODULE);
                    break;
                }
                case Cycle:
                {
                    /*run_dsp_srlim(SRLIM_SIGGEN_AMP, USE_MODULE);
                    run_dsp_srlim(SRLIM_SIGGEN_OFFSET, USE_MODULE);
                    SIGGEN.p_run_siggen(&SIGGEN);
                    break;*/
                }
                case RmpWfm:
                {
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
                SATURATE(V_CAPBANK_REFERENCE, MAX_REF_OL, MIN_REF_OL);
                DUTY_CYCLE = 0.01 * V_CAPBANK_REFERENCE;
                SATURATE(DUTY_CYCLE, PWM_MAX_DUTY_OL, PWM_MIN_DUTY_OL);
            }
            /// Closed-loop
            else
            {
                SATURATE(g_ipc_ctom.ps_module[0].ps_reference, MAX_REF, MIN_REF);
                run_dsp_error(ERROR_V_CAPBANK);
                run_dsp_pi(PI_CONTROLLER_V_CAPBANK);
                //run_dsp_iir_2p2z(IIR_2P2Z_CONTROLLER_V_CAPBANK);

                SATURATE(DUTY_CYCLE, PWM_MAX_DUTY, PWM_MIN_DUTY);
            }
        }

    /*********************************************/
    END_TIMESLICER(TIMESLICER_V_CAPBANK_CONTROLLER)
    /*********************************************/

    /// Check whether power supply is ON
    if(g_ipc_ctom.ps_module[0].ps_status.bit.state > Interlock)
    {
        /// Open-loop
        if(!g_ipc_ctom.ps_module[0].ps_status.bit.openloop)
        {
            run_dsp_error(ERROR_IOUT_RECT);
            run_dsp_pi(PI_CONTROLLER_IOUT_RECT);
            //run_dsp_iir_2p2z(IIR_2P2Z_CONTROLLER_IOUT_RECT);
            SATURATE(DUTY_CYCLE, PWM_MAX_DUTY, PWM_MIN_DUTY);
        }

        set_pwm_duty_chA(PWM_MODULATOR, DUTY_CYCLE);
    }

    /*********************************************/
    RUN_TIMESLICER(TIMESLICER_BUFFER)
    /*********************************************/
        insert_buffer(BUF_SAMPLES, g_controller_ctom.net_signals[3].f);
        //insert_buffer(BUF_SAMPLES, V_CAPBANK);
        //insert_buffer(BUF_SAMPLES, IOUT_RECT);

    /*********************************************/
    END_TIMESLICER(TIMESLICER_BUFFER)
    /*********************************************/

    /// TODO: save on buffers

    CLEAR_DEBUG_GPIO1;

    PWM_MODULATOR->ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all |= M_INT3;
}

/**
 * Initialization of interruptions.
 */
static void init_interruptions(void)
{
    EALLOW;
    PieVectTable.EPWM1_INT =  &isr_init_controller;
    EDIS;

    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
    enable_pwm_interrupt(PWM_MODULATOR);

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
    disable_pwm_interrupt(PWM_MODULATOR);

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
        reset_controller();

        g_ipc_ctom.ps_module[0].ps_status.bit.state = Initializing;

        PIN_CLOSE_AC_MAINS_CONTACTOR;
        DELAY_US(TIMEOUT_AC_MAINS_CONTACTOR_CLOSED_MS*1000);

        if(!PIN_STATUS_AC_MAINS_CONTACTOR)
        {
            set_hard_interlock(AC_MAINS_CONTACTOR_FAIL);
        }
        #ifdef USE_ITLK
        else
        {
        #endif
            g_ipc_ctom.ps_module[0].ps_status.bit.openloop = OPEN_LOOP;
            g_ipc_ctom.ps_module[0].ps_status.bit.state = SlowRef;
            enable_pwm_output(0);
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

    PIN_OPEN_AC_MAINS_CONTACTOR;
    DELAY_US(TIMEOUT_AC_MAINS_CONTACTOR_OPENED_MS*1000);

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
 * Set specified hard interlock for specified power supply.
 *
 * @param itlk specified hard interlock
 */
static void set_hard_interlock(uint32_t itlk)
{
    if(!(g_ipc_ctom.ps_module[0].ps_hard_interlock & itlk))
    {
        #ifdef USE_ITLK
        turn_off(0);
        g_ipc_ctom.ps_module[0].ps_status.bit.state = Interlock;
        #endif

        g_ipc_ctom.ps_module[0].ps_hard_interlock |= itlk;
    }
}

/**
 * Set specified soft interlock for specified power supply.
 *
 * @param itlk specified soft interlock
 */
static void set_soft_interlock(uint32_t itlk)
{
    if(!(g_ipc_ctom.ps_module[0].ps_soft_interlock & itlk))
    {
        #ifdef USE_ITLK
        turn_off(0);
        g_ipc_ctom.ps_module[0].ps_status.bit.state = Interlock;
        #endif

        g_ipc_ctom.ps_module[0].ps_soft_interlock |= itlk;
    }
}

/**
 * ISR for MtoC hard interlock request.
 */
static interrupt void isr_hard_interlock(void)
{
    if(!(g_ipc_ctom.ps_module[0].ps_hard_interlock &
         g_ipc_mtoc.ps_module[0].ps_hard_interlock))
    {
        #ifdef USE_ITLK
        turn_off(0);
        g_ipc_ctom.ps_module[0].ps_status.bit.state = Interlock;
        #endif

        g_ipc_ctom.ps_module[0].ps_hard_interlock |=
        g_ipc_mtoc.ps_module[0].ps_hard_interlock;
    }
}

/**
 * ISR for MtoC soft interlock request.
 */
static interrupt void isr_soft_interlock(void)
{
    if(!(g_ipc_ctom.ps_module[0].ps_soft_interlock &
         g_ipc_mtoc.ps_module[0].ps_soft_interlock))
    {
        #ifdef USE_ITLK
        turn_off(0);
        g_ipc_ctom.ps_module[0].ps_status.bit.state = Interlock;
        #endif

        g_ipc_ctom.ps_module[0].ps_soft_interlock |=
        g_ipc_mtoc.ps_module[0].ps_soft_interlock;
    }
}

/**
 * Check interlocks of this specific power supply topology
 */
static inline void check_interlocks(void)
{
    if(fabs(V_CAPBANK) > MAX_V_CAPBANK)
    {
        set_hard_interlock(CAPBANK_OVERVOLTAGE);
    }

    if(fabs(IOUT_RECT) > MAX_IOUT_RECT)
    {
        set_hard_interlock(RECTIFIER_OVERCURRENT);
    }

    DINT;

    if ( (g_ipc_ctom.ps_module[0].ps_status.bit.state <= Interlock) &&
         (PIN_STATUS_AC_MAINS_CONTACTOR) )
    {
        set_hard_interlock(AC_MAINS_CONTACTOR_FAIL);
    }

    else if ( (g_ipc_ctom.ps_module[0].ps_status.bit.state > Interlock)
              && (!PIN_STATUS_AC_MAINS_CONTACTOR) )
    {
        set_hard_interlock(AC_MAINS_CONTACTOR_FAIL);
    }

    EINT;
}
