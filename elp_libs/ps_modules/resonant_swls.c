/******************************************************************************
 * Copyright (C) 2022 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file resonant_swls.h
 * @brief Resonant converter module for SWLS
 *
 * Module for control of resonant convert power supply designed for the
 * superconducting Wavelength Shifter. It implements the controller for load
 * current using frequency modulation strategy, instead of pulse-width
 * modulation (PWM).
 *
 * @author gabriel.brunheira
 * @date 18/07/2022
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

#include "resonant_swls.h"

/**
 * Analog variables parameters
 */
#define MAX_ILOAD                               ANALOG_VARS_MAX[0]

#define MAX_V_DCLINK                            ANALOG_VARS_MAX[1]
#define MIN_V_DCLINK                            ANALOG_VARS_MIN[1]

#define MAX_V_DCLINK_TURN_ON                    ANALOG_VARS_MAX[2]

#define MAX_DCCTS_DIFF                          ANALOG_VARS_MAX[3]

/// Fixed turn-on time for discontinuous-conduction mode operation [us]
#define T_ON_US                                 ANALOG_VARS_MAX[4]

/// Fixed frequency offset summed to control effort to compensate dead-zone [Hz]
#define FREQ_DEADZONE_HZ                        ANALOG_VARS_MAX[5]

/// NUM_DCCTs == 0 : 1 DCCT
/// NUM_DCCTs > 0  : 2 DCCT's
#define NUM_DCCTs                               ANALOG_VARS_MAX[6]

#define TIMEOUT_DCLINK_CONTACTOR_CLOSED_MS      ANALOG_VARS_MAX[7]
#define TIMEOUT_DCLINK_CONTACTOR_OPENED_MS      ANALOG_VARS_MAX[8]
#define RESET_PULSE_TIME_DCLINK_CONTACTOR_MS    ANALOG_VARS_MAX[9]

/**
 * Controller defines
 */

/// DSP Net Signals
#define I_LOAD_1                g_controller_ctom.net_signals[0].f  // HRADC0
#define I_LOAD_2                g_controller_ctom.net_signals[1].f  // HRADC1

#define I_LOAD_MEAN             g_controller_ctom.net_signals[2].f
#define I_LOAD_ERROR            g_controller_ctom.net_signals[3].f
#define I_LOAD_DIFF             g_controller_ctom.net_signals[4].f

#define FREQ_MODULATED          g_controller_ctom.net_signals[5].f

#define FREQ_MODULATED_COMPENS  g_controller_ctom.output_signals[0].f

/// ARM Net Signals
#define V_DCLINK                g_controller_mtoc.net_signals[0].f

/// Reference
#define I_LOAD_SETPOINT             g_ipc_ctom.ps_module[0].ps_setpoint
#define I_LOAD_REFERENCE            g_ipc_ctom.ps_module[0].ps_reference

#define SRLIM_I_LOAD_REFERENCE      &g_controller_ctom.dsp_modules.dsp_srlim[0]

#define WFMREF                      g_ipc_ctom.wfmref[0]

#define SIGGEN                      SIGGEN_CTOM[0]
#define SRLIM_SIGGEN_AMP            &g_controller_ctom.dsp_modules.dsp_srlim[1]
#define SRLIM_SIGGEN_OFFSET         &g_controller_ctom.dsp_modules.dsp_srlim[2]

#define MAX_SLEWRATE_SLOWREF            g_controller_mtoc.dsp_modules.dsp_srlim[0].coeffs.s.max_slewrate
#define MAX_SLEWRATE_SIGGEN_AMP         g_controller_mtoc.dsp_modules.dsp_srlim[1].coeffs.s.max_slewrate
#define MAX_SLEWRATE_SIGGEN_OFFSET      g_controller_mtoc.dsp_modules.dsp_srlim[2].coeffs.s.max_slewrate

/// Load current controller
#define ERROR_I_LOAD                    &g_controller_ctom.dsp_modules.dsp_error[0]

#define PI_CONTROLLER_I_LOAD            &g_controller_ctom.dsp_modules.dsp_pi[0]
#define PI_CONTROLLER_I_LOAD_COEFFS     g_controller_mtoc.dsp_modules.dsp_pi[0].coeffs.s
#define KP_I_LOAD                       PI_CONTROLLER_I_LOAD_COEFFS.kp
#define KI_I_LOAD                       PI_CONTROLLER_I_LOAD_COEFFS.ki

/// PWM modulators
#define PWM_MODULATOR_1                 g_pwm_modules.pwm_regs[0]
#define PWM_MODULATOR_2                 g_pwm_modules.pwm_regs[1]
#define PWM_ISR_CONTROLLER              g_pwm_modules.pwm_regs[2]

/// Scope
#define SCOPE                           SCOPE_CTOM[0]

/**
 * Digital I/O's status
 */
#define PIN_OPEN_DCLINK_CONTACTOR       CLEAR_GPDO1
#define PIN_CLOSE_DCLINK_CONTACTOR      SET_GPDO1

#define PIN_SET_MAGNAPOWER_INTERLOCK    CLEAR_GPDO2
#define PIN_CLEAR_MAGNAPOWER_INTERLOCK  SET_GPDO2

#define PIN_STATUS_EXTERNAL_INTERLOCK   GET_GPDI6
#define PIN_STATUS_DCLINK_CONTACTOR     GET_GPDI5
#define PIN_STATUS_DCCT_1_STATUS        GET_GPDI4
#define PIN_STATUS_DCCT_2_STATUS        GET_GPDI3

/**
 * Interlocks and alarms defines
 */
typedef enum
{
    Load_Overcurrent,
    DCLink_Overvoltage,
    DCLink_Undervoltage,
    Welded_Contactor_Fault,
    Opened_Contactor_Fault,
    External_Itlk,
    IIB_Itlk
} hard_interlocks_t;

typedef enum
{
    DCCT_1_Fault,
    DCCT_2_Fault,
    DCCT_High_Difference
} soft_interlocks_t;

typedef enum
{
    High_Sync_Input_Frequency = 0x00000001
} alarms_t;

#define NUM_HARD_INTERLOCKS             IIB_Itlk + 1
#define NUM_SOFT_INTERLOCKS             DCCT_High_Difference + 1

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
void main_resonant_swls(void)
{
    /// Clear interlock signal to MagnaPower caused by UDC power-on
    PIN_CLEAR_MAGNAPOWER_INTERLOCK;

    init_controller();
    init_peripherals_drivers();
    init_interruptions();
    reset_controller();
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
    float duty_cycle;

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
    g_pwm_modules.num_modules = 3;

    PWM_MODULATOR_1 = &EPwm1Regs;
    PWM_MODULATOR_2 = &EPwm2Regs;
    PWM_ISR_CONTROLLER = &EPwm3Regs;

    disable_pwm_outputs();
    disable_pwm_tbclk();
    init_pwm_mep_sfo();

    /// PWM initialization
    init_pwm_module(PWM_MODULATOR_1, PWM_FREQ, 0, PWM_Sync_Master, 0,
                    PWM_ChB_Independent, PWM_DEAD_TIME);
    init_pwm_module(PWM_MODULATOR_2, PWM_FREQ, 1, PWM_Sync_Slave, 180,
                    PWM_ChB_Independent, PWM_DEAD_TIME);

    /// Fix turn-on time
    duty_cycle = T_ON_US*1e-6*PWM_FREQ;

    set_pwm_duty_chA(PWM_MODULATOR_1, duty_cycle);
    set_pwm_duty_chA(PWM_MODULATOR_2, duty_cycle);

    /// Set time-base period register on shadow mode (update on next cycle)
    PWM_MODULATOR_1->TBCTL.bit.PRDLD = TB_SHADOW;
    PWM_MODULATOR_2->TBCTL.bit.PRDLD = TB_SHADOW;

    // This setting allows large frequency steps to happen without error
    PWM_MODULATOR_2->TBCTL2.bit.PRDLDSYNC = 0x01;


    FREQ_MODULATED = PWM_FREQ;

    /// PWM module for ISR controller
    init_pwm_module(PWM_ISR_CONTROLLER, ISR_CONTROL_FREQ, 0, PWM_Sync_Master, 0,
                    PWM_ChB_Independent, PWM_DEAD_TIME);
    set_pwm_duty_chA(PWM_ISR_CONTROLLER, 0.5);


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

    init_control_framework(&g_controller_ctom);

    init_ipc();

    init_wfmref(&WFMREF, WFMREF_SELECTED_PARAM[0], WFMREF_SYNC_MODE_PARAM[0],
                ISR_CONTROL_FREQ, WFMREF_FREQUENCY_PARAM[0], WFMREF_GAIN_PARAM[0],
                WFMREF_OFFSET_PARAM[0], &g_wfmref_data.data, SIZE_WFMREF,
                &I_LOAD_REFERENCE);

    /***********************************************/
    /** INITIALIZATION OF SIGNAL GENERATOR MODULE **/
    /***********************************************/

    disable_siggen(&SIGGEN);

    init_siggen(&SIGGEN, ISR_CONTROL_FREQ, &I_LOAD_REFERENCE);

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

    init_dsp_srlim(SRLIM_SIGGEN_AMP, MAX_SLEWRATE_SIGGEN_AMP, ISR_CONTROL_FREQ,
                   &SIGGEN_MTOC[0].amplitude, &SIGGEN.amplitude);

    /**
     *        name:     SRLIM_SIGGEN_OFFSET
     * description:     Signal generator offset slew-rate limiter
     *    DP class:     DSP_SRLim
     *          in:     SIGGEN_MTOC[0].offset
     *         out:     SIGGEN_CTOM[0].offset
     */

    init_dsp_srlim(SRLIM_SIGGEN_OFFSET, MAX_SLEWRATE_SIGGEN_OFFSET,
                   ISR_CONTROL_FREQ, &SIGGEN_MTOC[0].offset,
                   &SIGGEN_CTOM[0].offset);

    /*************************************************/
    /** INITIALIZATION OF LOAD CURRENT CONTROL LOOP **/
    /*************************************************/

    /**
     *        name:     SRLIM_I_LOAD_REFERENCE
     * description:     Load current slew-rate limiter
     *    DP class:     DSP_SRLim
     *          in:     I_LOAD_SETPOINT
     *         out:     I_LOAD_REFERENCE
     */

    init_dsp_srlim(SRLIM_I_LOAD_REFERENCE, MAX_SLEWRATE_SLOWREF, ISR_CONTROL_FREQ,
                   &I_LOAD_SETPOINT, &I_LOAD_REFERENCE);

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
     * description:     Load current PI controller
     *  dsp module:     DSP_PI
     *          in:     I_LOAD_ERROR
     *         out:     FREQ_MODULATED
     */

    init_dsp_pi(PI_CONTROLLER_I_LOAD, KP_I_LOAD, KI_I_LOAD, ISR_CONTROL_FREQ,
                MAX_REF_OL[0], MIN_REF_OL[0], &I_LOAD_ERROR, &FREQ_MODULATED);

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
    set_pwm_freq(PWM_MODULATOR_1, PWM_FREQ);
    set_pwm_freq(PWM_MODULATOR_2, PWM_FREQ);

    cfg_pwm_sync(PWM_MODULATOR_1, PWM_Sync_Master, 0.0);
    cfg_pwm_sync(PWM_MODULATOR_2, PWM_Sync_Slave, 180.0);

    g_ipc_ctom.ps_module[0].ps_status.bit.openloop = LOOP_STATE;

    /// Change in the default reference according to loop-state:
    if(LOOP_STATE == CLOSED_LOOP)
    {
        I_LOAD_SETPOINT = MIN_REF[0];
        I_LOAD_REFERENCE = MIN_REF[0];
    }
    else
    {
        I_LOAD_SETPOINT = MIN_REF_OL[0];
        I_LOAD_REFERENCE = MIN_REF_OL[0];
    }

    reset_dsp_srlim(SRLIM_I_LOAD_REFERENCE);
    reset_dsp_error(ERROR_I_LOAD);
    reset_dsp_pi(PI_CONTROLLER_I_LOAD);

    reset_dsp_srlim(SRLIM_SIGGEN_AMP);
    reset_dsp_srlim(SRLIM_SIGGEN_OFFSET);
    disable_siggen(&SIGGEN);

    reset_wfmref(&WFMREF);
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
    PieVectTable.EPWM3_INT = &isr_controller;
    EDIS;

    PWM_ISR_CONTROLLER->ETSEL.bit.INTSEL = ET_CTR_ZERO;
    PWM_ISR_CONTROLLER->ETCLR.bit.INT = 1;

    /**
     *  Enable XINT2 (external interrupt 2) interrupt used for sync pulses for
     *  the first time
     *
     *  TODO: include here mechanism described in section 1.5.4.3 from F28M36
     *  Technical Reference Manual (SPRUHE8E) to clear flag before enabling, to
     *  avoid false alarms that may occur when sync pulses are received during
     *  firmware initialization.
     */
    PieCtrlRegs.PIEIER1.bit.INTx5 = 1;

    /// Clear interrupt flag for PWM interrupts group
    PieCtrlRegs.PIEACK.all |= M_INT3;
}

/**
 * Control ISR
 */
static interrupt void isr_controller(void)
{
    static float temp[4];
    static uint16_t i;

    //SET_DEBUG_GPIO0;
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

    if(NUM_DCCTs)
    {
        I_LOAD_1 = temp[0] - TRANSDUCER_OFFSET[0];
        I_LOAD_2 = temp[1] - TRANSDUCER_OFFSET[1];

        I_LOAD_MEAN = 0.5*(I_LOAD_1 + I_LOAD_2);
        I_LOAD_DIFF = I_LOAD_1 - I_LOAD_2;
    }
    else
    {
        I_LOAD_1 = temp[0];

        I_LOAD_MEAN = I_LOAD_1 - TRANSDUCER_OFFSET[0];
        I_LOAD_DIFF = 0;
    }

    /// Check whether power supply is ON
    if(g_ipc_ctom.ps_module[0].ps_status.bit.state >= SlowRef)
    {
        /// Calculate reference according to operation mode
        switch(g_ipc_ctom.ps_module[0].ps_status.bit.state)
        {
            case SlowRef:
            case SlowRefSync:
            {
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
            case MigWfm:
            {
                run_wfmref(&WFMREF);
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
            SATURATE(I_LOAD_REFERENCE, MAX_REF_OL[0], MIN_REF_OL[0])
            FREQ_MODULATED = I_LOAD_REFERENCE;
        }
        /// Closed-loop
        else
        {
            SATURATE(I_LOAD_REFERENCE, MAX_REF[0], MIN_REF[0]);
            run_dsp_error(ERROR_I_LOAD);
            run_dsp_pi(PI_CONTROLLER_I_LOAD);
            SATURATE(FREQ_MODULATED, MAX_REF_OL[0], MIN_REF_OL[0]);
        }

        /// Modulation frequency dead-zone compensation
        FREQ_MODULATED_COMPENS = FREQ_MODULATED + FREQ_DEADZONE_HZ;
        SATURATE(FREQ_MODULATED_COMPENS, MAX_REF_OL[0], MIN_REF_OL[0]);

        set_pwm_freq(PWM_MODULATOR_1, FREQ_MODULATED_COMPENS);
        set_pwm_freq(PWM_MODULATOR_2, FREQ_MODULATED_COMPENS);

        cfg_pwm_sync(PWM_MODULATOR_1, PWM_Sync_Master, 0.0);
        cfg_pwm_sync(PWM_MODULATOR_2, PWM_Sync_Slave, 180.0);
    }

    RUN_SCOPE(SCOPE);

    SET_INTERLOCKS_TIMEBASE_FLAG(0);

    /**
     * Re-enable external interrupt 2 (XINT2) interrupts to allow sync pulses to
     * be handled once per isr_controller
     */
    if(PieCtrlRegs.PIEIER1.bit.INTx5 == 0)
    {
        /// Set alarm if counter is below limit when receiving new sync pulse
        if(counter_sync_period < MIN_NUM_ISR_CONTROLLER_SYNC)
        {
            g_ipc_ctom.ps_module[0].ps_alarms = High_Sync_Input_Frequency;
        }

        /// Store counter value on BSMP variable
        g_ipc_ctom.period_sync_pulse = counter_sync_period;
        counter_sync_period = 0;
    }

    counter_sync_period++;

    /**
     * Reset counter to threshold to avoid false alarms during its overflow
     */
    if(counter_sync_period == MAX_NUM_ISR_CONTROLLER_SYNC)
    {
        counter_sync_period = MIN_NUM_ISR_CONTROLLER_SYNC;
    }

    /// Re-enable XINT2 (external interrupt 2) interrupt used for sync pulses
    PieCtrlRegs.PIEIER1.bit.INTx5 = 1;

    /// Clear interrupt flags for PWM interrupts
    //PWM_MODULATOR_1->ETCLR.bit.INT = 1;
    //PWM_MODULATOR_2->ETCLR.bit.INT = 1;
    PWM_ISR_CONTROLLER->ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all |= M_INT3;

    CLEAR_DEBUG_GPIO1;
}

/**
 * Initialization of interruptions.
 */
static void init_interruptions(void)
{
    EALLOW;
    //PieVectTable.EPWM1_INT =  &isr_init_controller;
    //PieVectTable.EPWM2_INT =  &isr_controller;
    PieVectTable.EPWM3_INT =  &isr_init_controller;
    EDIS;

    //PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
    //PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx3 = 1;

    //enable_pwm_interrupt(PWM_MODULATOR_1);
    //enable_pwm_interrupt(PWM_MODULATOR_2);
    enable_pwm_interrupt(PWM_ISR_CONTROLLER);

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
    //PieCtrlRegs.PIEIER3.bit.INTx1 = 0;  /// ePWM1
    //PieCtrlRegs.PIEIER3.bit.INTx2 = 0;  /// ePWM2
    PieCtrlRegs.PIEIER3.bit.INTx3 = 0;  /// ePWM3

    //disable_pwm_interrupt(PWM_MODULATOR_1);
    //disable_pwm_interrupt(PWM_MODULATOR_2);
    disable_pwm_interrupt(PWM_ISR_CONTROLLER);

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
        if(V_DCLINK > MAX_V_DCLINK_TURN_ON)
        {
            BYPASS_HARD_INTERLOCK_DEBOUNCE(0, DCLink_Overvoltage);
            set_hard_interlock(0, DCLink_Overvoltage);
        }

        #ifdef USE_ITLK
        else
        {
        #endif

            PIN_CLOSE_DCLINK_CONTACTOR;
            DELAY_US(TIMEOUT_DCLINK_CONTACTOR_CLOSED_MS*1000);

            if(!PIN_STATUS_DCLINK_CONTACTOR)
            {
                BYPASS_HARD_INTERLOCK_DEBOUNCE(0, Opened_Contactor_Fault);
                set_hard_interlock(0, Opened_Contactor_Fault);
            }

            #ifdef USE_ITLK
            else
            {
            #endif
                g_ipc_ctom.ps_module[0].ps_status.bit.state = Initializing;
            #ifdef USE_ITLK
            }
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

    PIN_OPEN_DCLINK_CONTACTOR;
    DELAY_US(TIMEOUT_DCLINK_CONTACTOR_OPENED_MS*1000);

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
    g_ipc_ctom.ps_module[0].ps_alarms = 0;

    if(g_ipc_ctom.ps_module[0].ps_status.bit.state < Initializing)
    {
        if(PIN_STATUS_DCLINK_CONTACTOR)
        {
            PIN_CLOSE_DCLINK_CONTACTOR;
            DELAY_US(RESET_PULSE_TIME_DCLINK_CONTACTOR_MS*1000);
            PIN_OPEN_DCLINK_CONTACTOR;
            DELAY_US(TIMEOUT_DCLINK_CONTACTOR_OPENED_MS*1000);
        }

        PIN_CLEAR_MAGNAPOWER_INTERLOCK;

        g_ipc_ctom.ps_module[0].ps_status.bit.state = Off;
    }
}

/**
 * Check interlocks of this specific power supply topology
 */
static inline void check_interlocks(void)
{
    //SET_DEBUG_GPIO1;

    if(fabs(I_LOAD_MEAN) > MAX_ILOAD)
    {
        set_hard_interlock(0, Load_Overcurrent);
    }

    if(fabs(I_LOAD_DIFF) > MAX_DCCTS_DIFF)
    {
        set_soft_interlock(0, DCCT_High_Difference);
    }

    if(V_DCLINK > MAX_V_DCLINK)
    {
        set_hard_interlock(0, DCLink_Overvoltage);
    }

    if(!PIN_STATUS_DCCT_1_STATUS)
    {
        set_soft_interlock(0, DCCT_1_Fault);
    }

    if( NUM_DCCTs && !PIN_STATUS_DCCT_2_STATUS )
    {
        set_soft_interlock(0, DCCT_2_Fault);
    }

    if(!PIN_STATUS_EXTERNAL_INTERLOCK)
    {
        set_hard_interlock(0, External_Itlk);
    }

    DINT;

    if(g_ipc_ctom.ps_module[0].ps_status.bit.state <= Interlock)
    {
        if(PIN_STATUS_DCLINK_CONTACTOR)
        {
            set_hard_interlock(0, Welded_Contactor_Fault);
        }
    }

    else
    {
        if(!PIN_STATUS_DCLINK_CONTACTOR)
        {
            set_hard_interlock(0, Opened_Contactor_Fault);
        }

        if(g_ipc_ctom.ps_module[0].ps_status.bit.state == Initializing)
        {
            if(V_DCLINK > MIN_V_DCLINK)
            {
                g_ipc_ctom.ps_module[0].ps_status.bit.state = SlowRef;
                enable_pwm_output(0);
                enable_pwm_output(1);
            }
        }

        else if(g_ipc_ctom.ps_module[0].ps_status.bit.state > Initializing) /// Power supply ON
        {
            if(V_DCLINK < MIN_V_DCLINK)
            {
                set_hard_interlock(0, DCLink_Undervoltage);
            }
        }
    }

    EINT;

    run_interlocks_debouncing(0);

    #ifdef USE_ITLK
    if(g_ipc_ctom.ps_module[0].ps_status.bit.state == Interlock)
    #else
    if(g_ipc_ctom.ps_module[0].ps_hard_interlock || g_ipc_ctom.ps_module[0].ps_soft_interlock)
    #endif
    {
        PIN_SET_MAGNAPOWER_INTERLOCK;
    }

    //CLEAR_DEBUG_GPIO1;
}
