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
#include "event_manager/event_manager.h"
#include "HRADC_board/HRADC_Boards.h"
#include "ipc/ipc.h"
#include "parameters/parameters.h"
#include "pwm/pwm.h"

#include "fac_acdc.h"

/**
 * Control parameters
 */
#define TIMESLICER_CONTROLLER_IDX       2
#define TIMESLICER_CONTROLLER           g_controller_ctom.timeslicer[TIMESLICER_CONTROLLER_IDX]
#define CONTROLLER_FREQ_SAMP            TIMESLICER_FREQ[TIMESLICER_CONTROLLER_IDX]

#define SIGGEN                      SIGGEN_CTOM[0]

/**
 * Analog variables parameters
 */
#define MAX_V_CAPBANK           ANALOG_VARS_MAX[0]

#define MAX_VOUT_RECT           ANALOG_VARS_MAX[1]
#define MAX_IOUT_RECT           ANALOG_VARS_MAX[2]
#define MAX_IOUT_RECT_REF       ANALOG_VARS_MAX[3]
#define MIN_IOUT_RECT_REF       ANALOG_VARS_MIN[3]

#define MAX_TEMP_HEATSINK       ANALOG_VARS_MAX[4]
#define MAX_TEMP_INDUCTORS      ANALOG_VARS_MAX[5]

#define TIMEOUT_AC_MAINS_CONTACTOR_CLOSED_MS   ANALOG_VARS_MAX[6]
#define TIMEOUT_AC_MAINS_CONTACTOR_OPENED_MS   ANALOG_VARS_MAX[7]

#define NETSIGNAL_ELEM_CTOM_BUF                 ANALOG_VARS_MAX[8]
#define NETSIGNAL_ELEM_MTOC_BUF                 ANALOG_VARS_MIN[8]

#define NETSIGNAL_CTOM_BUF      g_controller_ctom.net_signals[(uint16_t) NETSIGNAL_ELEM_CTOM_BUF].f
#define NETSIGNAL_MTOC_BUF      g_controller_mtoc.net_signals[(uint16_t) NETSIGNAL_ELEM_MTOC_BUF].f

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

#define MAX_SLEWRATE_SLOWREF            g_controller_mtoc.dsp_modules.dsp_srlim[0].coeffs.s.max_slewrate
#define MAX_SLEWRATE_SIGGEN_AMP         g_controller_mtoc.dsp_modules.dsp_srlim[1].coeffs.s.max_slewrate
#define MAX_SLEWRATE_SIGGEN_OFFSET      g_controller_mtoc.dsp_modules.dsp_srlim[2].coeffs.s.max_slewrate

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

#define RESSONANT_2HZ_CONTROLLER_IOUT_RECT          &g_controller_ctom.dsp_modules.dsp_iir_2p2z[3]
#define RESSONANT_2HZ_CONTROLLER_IOUT_RECT_COEFFS    g_controller_mtoc.dsp_modules.dsp_iir_2p2z[3].coeffs.s

#define RESSONANT_4HZ_CONTROLLER_IOUT_RECT           &g_controller_ctom.dsp_modules.dsp_iir_2p2z[4]
#define RESSONANT_4HZ_CONTROLLER_IOUT_RECT_COEFFS    g_controller_mtoc.dsp_modules.dsp_iir_2p2z[4].coeffs.s

#define PWM_MODULATOR                   g_pwm_modules.pwm_regs[0]

#define SRLIM_SIGGEN_AMP                &g_controller_ctom.dsp_modules.dsp_srlim[1]
#define SRLIM_SIGGEN_OFFSET             &g_controller_ctom.dsp_modules.dsp_srlim[2]

/// Scope
#define SCOPE                           SCOPE_CTOM[0]

/**
 * Interlocks defines
 */
typedef enum
{
    CapBank_Overvoltage,
    Rectifier_Overvoltage,
    Rectifier_Undervoltage,
    Rectifier_Overcurrent,
    AC_Mains_Contactor_Fault,
    IGBT_Driver_Fault,
    IIB_Itlk
} hard_interlocks_t;

typedef enum
{
    Heatsink_Overtemperature,
    Inductors_Overtemperature
} soft_interlocks_t;

#define NUM_HARD_INTERLOCKS     IGBT_Driver_Fault + 1
#define NUM_SOFT_INTERLOCKS     Inductors_Overtemperature + 1

/**
 *  Private variables
 */
static float decimation_factor;
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

    decimation_factor = HRADC_FREQ_SAMP / ISR_CONTROL_FREQ;
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

    init_siggen(&SIGGEN, CONTROLLER_FREQ_SAMP,
                &g_ipc_ctom.ps_module[0].ps_reference);

    cfg_siggen(&SIGGEN, SIGGEN_TYPE_PARAM, SIGGEN_NUM_CYCLES_PARAM,
               SIGGEN_FREQ_PARAM, SIGGEN_AMP_PARAM,
               SIGGEN_OFFSET_PARAM, SIGGEN_AUX_PARAM);

    /**
     *        name:     SRLIM_SIGGEN_AMP
     * description:     Signal generator amplitude slew-rate limiter
     *    DP class:     DSP_SRLim
     *          in:     SIGGEN_MTOC[0].amplitude
     *         out:     SIGGEN_CTOM[0].amplitude
     */

    init_dsp_srlim(SRLIM_SIGGEN_AMP, MAX_SLEWRATE_SIGGEN_AMP,
                   CONTROLLER_FREQ_SAMP, &SIGGEN_MTOC[0].amplitude,
                   &SIGGEN_CTOM[0].amplitude);

    /**
     *        name:     SRLIM_SIGGEN_OFFSET
     * description:     Signal generator offset slew-rate limiter
     *    DP class:     DSP_SRLim
     *          in:     SIGGEN_MTOC[0].offset
     *         out:     SIGGEN_CTOM[0].offset
     */

    init_dsp_srlim(SRLIM_SIGGEN_OFFSET, MAX_SLEWRATE_SIGGEN_OFFSET,
                   CONTROLLER_FREQ_SAMP, &SIGGEN_MTOC[0].offset,
                   &SIGGEN_CTOM[0].offset);

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

    init_dsp_srlim(SRLIM_V_CAPBANK_REFERENCE, MAX_SLEWRATE_SLOWREF,
                   CONTROLLER_FREQ_SAMP, &V_CAPBANK_SETPOINT,
                   &V_CAPBANK_REFERENCE);

    /**
     *        name:     ERROR_V_CAPBANK
     * description:     Capacitor bank voltage reference error
     *  dsp module:     DSP_Error
     *           +:     ps_module[0].ps_reference
     *           -:     net_signals[0]
     *         out:     net_signals[4]
     */

    init_dsp_error(ERROR_V_CAPBANK, &V_CAPBANK_REFERENCE,
                   &g_controller_ctom.net_signals[3].f,
                   &g_controller_ctom.net_signals[4].f);

    /**
     *        name:     PI_CONTROLLER_V_CAPBANK
     * description:     Capacitor bank voltage PI controller
     *  dsp module:     DSP_PI
     *          in:     net_signals[4]
     *         out:     net_signals[5]
     */

    init_dsp_pi(PI_CONTROLLER_V_CAPBANK, KP_V_CAPBANK, KI_V_CAPBANK,
                CONTROLLER_FREQ_SAMP, MAX_IOUT_RECT_REF, MIN_IOUT_RECT_REF,
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
                        CONTROLLER_FREQ_SAMP, FLT_MAX, -FLT_MAX,
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
                        CONTROLLER_FREQ_SAMP, FLT_MAX, -FLT_MAX,
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
     *        name:     RESSONANT_2HZ_CONTROLLER_IOUT_RECT
     * description:     Rectifier output current 2 Hz ressonant controller
     *    DP class:     ELP_IIR_2P2Z
     *          in:     net_signals[6]
     *         out:     net_signals[7]
     */

    init_dsp_iir_2p2z(RESSONANT_2HZ_CONTROLLER_IOUT_RECT,
                      RESSONANT_2HZ_CONTROLLER_IOUT_RECT_COEFFS.b0,
                      RESSONANT_2HZ_CONTROLLER_IOUT_RECT_COEFFS.b1,
                      RESSONANT_2HZ_CONTROLLER_IOUT_RECT_COEFFS.b2,
                      RESSONANT_2HZ_CONTROLLER_IOUT_RECT_COEFFS.a1,
                      RESSONANT_2HZ_CONTROLLER_IOUT_RECT_COEFFS.a2,
                      FLT_MAX, -FLT_MAX,
                      &g_controller_ctom.net_signals[6].f,
                      &g_controller_ctom.net_signals[7].f);

    /**
     *        name:     RESSONANT_4HZ_CONTROLLER_IOUT_RECT
     * description:     Rectifier output current 4 Hz ressonant controller
     *    DP class:     ELP_IIR_2P2Z
     *          in:     net_signals[7]
     *         out:     net_signals[8]
     */

    init_dsp_iir_2p2z(RESSONANT_4HZ_CONTROLLER_IOUT_RECT,
                      RESSONANT_4HZ_CONTROLLER_IOUT_RECT_COEFFS.b0,
                      RESSONANT_4HZ_CONTROLLER_IOUT_RECT_COEFFS.b1,
                      RESSONANT_4HZ_CONTROLLER_IOUT_RECT_COEFFS.b2,
                      RESSONANT_4HZ_CONTROLLER_IOUT_RECT_COEFFS.a1,
                      RESSONANT_4HZ_CONTROLLER_IOUT_RECT_COEFFS.a2,
                      FLT_MAX, -FLT_MAX,
                      &g_controller_ctom.net_signals[7].f,
                      &g_controller_ctom.net_signals[8].f);

    /**
     *        name:     PI_CONTROLLER_IOUT_RECT
     * description:     Rectifier output current PI controller
     *    DP class:     ELP_PI_dawu
     *          in:     net_signals[8]
     *         out:     output_signals[0]
     */
    init_dsp_pi(PI_CONTROLLER_IOUT_RECT, KP_IOUT_RECT, KI_IOUT_RECT,
                CONTROLLER_FREQ_SAMP, PWM_MAX_DUTY, PWM_MIN_DUTY,
                &g_controller_ctom.net_signals[8].f, &DUTY_CYCLE);

    /*init_dsp_pi(PI_CONTROLLER_IOUT_RECT, KP_IOUT_RECT, KI_IOUT_RECT,
                ISR_CONTROL_FREQ, PWM_MAX_DUTY, -PWM_MAX_DUTY,
                &g_controller_ctom.net_signals[6], &DUTY_CYCLE);*/

    /************************************/
    /** INITIALIZATION OF TIME SLICERS **/
    /************************************/

    /**
     * Time-slicer for controller
     */
    init_timeslicer(&TIMESLICER_CONTROLLER, ISR_CONTROL_FREQ);
    cfg_timeslicer(&TIMESLICER_CONTROLLER, CONTROLLER_FREQ_SAMP);

    /******************************/
    /** INITIALIZATION OF SCOPES **/
    /******************************/

    init_scope(&SCOPE, ISR_CONTROL_FREQ, SCOPE_FREQ_SAMPLING_PARAM[0],
               &g_buf_samples_ctom[0], SIZE_BUF_SAMPLES_CTOM,
               SCOPE_SOURCE_PARAM[0], &run_scope_shared_ram);


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

    /// Reset capacitor bank voltage controller
    reset_dsp_srlim(SRLIM_V_CAPBANK_REFERENCE);
    reset_dsp_error(ERROR_V_CAPBANK);
    reset_dsp_pi(PI_CONTROLLER_V_CAPBANK);
    reset_dsp_iir_2p2z(NOTCH_FILT_2HZ_V_CAPBANK);
    reset_dsp_iir_2p2z(NOTCH_FILT_4HZ_V_CAPBANK);
    reset_dsp_iir_2p2z(IIR_2P2Z_CONTROLLER_V_CAPBANK);

    /// Reset rectifier output current controller
    reset_dsp_error(ERROR_IOUT_RECT);
    reset_dsp_iir_2p2z(RESSONANT_2HZ_CONTROLLER_IOUT_RECT);
    reset_dsp_iir_2p2z(RESSONANT_4HZ_CONTROLLER_IOUT_RECT);
    reset_dsp_pi(PI_CONTROLLER_IOUT_RECT);

    reset_dsp_srlim(SRLIM_SIGGEN_AMP);
    reset_dsp_srlim(SRLIM_SIGGEN_OFFSET);
    disable_siggen(&SIGGEN);
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

    //CLEAR_DEBUG_GPIO1;
    SET_DEBUG_GPIO0;
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

    V_CAPBANK = temp[0];
    IOUT_RECT = temp[1];
    g_controller_ctom.net_signals[10].f = temp[2];
    g_controller_ctom.net_signals[11].f = temp[3];

    /******** Timeslicer for controllers *********/
    RUN_TIMESLICER(TIMESLICER_CONTROLLER)
    /*********************************************/

        /// Run notch filters for capacitor bank voltage feedback
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
                SATURATE(V_CAPBANK_REFERENCE, MAX_REF_OL[0], MIN_REF_OL[0]);
                DUTY_CYCLE = 0.01 * V_CAPBANK_REFERENCE;
                SATURATE(DUTY_CYCLE, PWM_MAX_DUTY_OL, PWM_MIN_DUTY_OL);
            }
            /// Closed-loop
            else
            {
                /// Run capacitor bank voltage control law
                SATURATE(g_ipc_ctom.ps_module[0].ps_reference, MAX_REF[0], MIN_REF[0]);
                run_dsp_error(ERROR_V_CAPBANK);
                run_dsp_pi(PI_CONTROLLER_V_CAPBANK);
                //run_dsp_iir_2p2z(IIR_2P2Z_CONTROLLER_V_CAPBANK);

                /// Run rectifier output current control law
                run_dsp_error(ERROR_IOUT_RECT);
                run_dsp_iir_2p2z(RESSONANT_2HZ_CONTROLLER_IOUT_RECT);
                run_dsp_iir_2p2z(RESSONANT_4HZ_CONTROLLER_IOUT_RECT);
                run_dsp_pi(PI_CONTROLLER_IOUT_RECT);

                SATURATE(DUTY_CYCLE, PWM_MAX_DUTY, PWM_MIN_DUTY);
            }

            set_pwm_duty_chA(PWM_MODULATOR, DUTY_CYCLE);
        }

    /*********************************************/
    END_TIMESLICER(TIMESLICER_CONTROLLER)
    /*********************************************/

    RUN_SCOPE(SCOPE);

    SET_INTERLOCKS_TIMEBASE_FLAG(0);

    PWM_MODULATOR->ETCLR.bit.INT = 1;
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
            set_hard_interlock(0, AC_Mains_Contactor_Fault);
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
 * Check interlocks of this specific power supply topology
 */
static inline void check_interlocks(void)
{
    if(fabs(V_CAPBANK) > MAX_V_CAPBANK)
    {
        set_hard_interlock(0, CapBank_Overvoltage);
    }

    if(fabs(IOUT_RECT) > MAX_IOUT_RECT)
    {
        set_hard_interlock(0, Rectifier_Overcurrent);
    }

    DINT;

    if ( (g_ipc_ctom.ps_module[0].ps_status.bit.state <= Interlock) &&
         (PIN_STATUS_AC_MAINS_CONTACTOR) )
    {
        set_hard_interlock(0, AC_Mains_Contactor_Fault);
    }

    else if ( (g_ipc_ctom.ps_module[0].ps_status.bit.state > Interlock)
              && (!PIN_STATUS_AC_MAINS_CONTACTOR) )
    {
        set_hard_interlock(0, AC_Mains_Contactor_Fault);
    }

    EINT;

    SET_DEBUG_GPIO1;
    run_interlocks_debouncing(0);
    CLEAR_DEBUG_GPIO1;
}
