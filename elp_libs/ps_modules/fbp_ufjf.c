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
 * @file fbp_ufjf.c
 * @brief FBP for UFJF controllers
 * 
 * Module for control of FBP crate adapted for usage on tests of controllers
 * developed in CNPEM-UFJF partnership
 *
 * @author gabriel.brunheira
 * @date 16/08/2022
 *
 */

#include <float.h>

#include "boards/udc_c28.h"
#include "common/timeslicer.h"
#include "control/control.h"
#include "event_manager/event_manager.h"
#include "HRADC_board/HRADC_Boards.h"
#include "ipc/ipc.h"
#include "parameters/parameters.h"
#include "pwm/pwm.h"

#include "fbp_ufjf.h"

/**
 * Analog variables parameters
 */
#define MAX_ILOAD           ANALOG_VARS_MAX[0]

/**
 * Controller defines
 */

/// Reference
#define DECIMATION_FACTOR               1

#define SIGGEN                          SIGGEN_CTOM
#define SIGGEN_OUTPUT                   g_controller_ctom.net_signals[20].f
#define SRLIM_SIGGEN_AMP                &g_controller_ctom.dsp_modules.dsp_srlim[1]
#define SRLIM_SIGGEN_OFFSET             &g_controller_ctom.dsp_modules.dsp_srlim[2]

#define WFMREF                          g_ipc_ctom.wfmref
#define WFMREF_OUTPUT                   g_controller_ctom.net_signals[21].f

#define I_LOAD_SETPOINT                 g_ipc_ctom.ps_module[0].ps_setpoint
#define I_LOAD_REFERENCE                g_ipc_ctom.ps_module[0].ps_reference

#define SRLIM_I_LOAD_REFERENCE          &g_controller_ctom.dsp_modules.dsp_srlim[0]

/// DSP Net Signals
#define I1_LOAD_CURRENT                 g_controller_ctom.net_signals[0].f  // HRADC0
#define PS2_LOAD_CURRENT                g_controller_ctom.net_signals[1].f  // HRADC1
#define PS3_LOAD_CURRENT                g_controller_ctom.net_signals[2].f  // HRADC2
#define I2_LOAD_CURRENT                 g_controller_ctom.net_signals[3].f  // HRADC3

#define I1_LOAD_ERROR                   g_controller_ctom.net_signals[4].f
#define I1_LOAD_ERROR_TO_M11            g_controller_ctom.net_signals[5].f
#define M11                             g_controller_ctom.net_signals[6].f
#define I1_LOAD_ERROR_TO_M12            g_controller_ctom.net_signals[7].f
#define M12                             g_controller_ctom.net_signals[8].f

#define I2_LOAD_ERROR                   g_controller_ctom.net_signals[9].f
#define I2_LOAD_ERROR_TO_M21            g_controller_ctom.net_signals[10].f
#define M21                             g_controller_ctom.net_signals[11].f
#define I2_LOAD_ERROR_TO_M22            g_controller_ctom.net_signals[12].f
#define M22                             g_controller_ctom.net_signals[13].f

#define DUTY_CYCLE_I1                   g_controller_ctom.output_signals[0].f
#define DUTY_CYCLE_I2                   g_controller_ctom.output_signals[1].f

/// ARM Net Signals
#define PS1_LOAD_VOLTAGE                g_controller_mtoc.net_signals[4].f  // ANI6
#define PS2_LOAD_VOLTAGE                g_controller_mtoc.net_signals[5].f  // ANI7
#define PS3_LOAD_VOLTAGE                g_controller_mtoc.net_signals[6].f  // ANI3
#define PS4_LOAD_VOLTAGE                g_controller_mtoc.net_signals[7].f  // ANI5

/// I1 load current controller
#define ERROR_CALCULATOR_I1             &g_controller_ctom.dsp_modules.dsp_error[0]

#define M11_LOAD_CURRENT_CONTROLLER_A            &g_controller_ctom.dsp_modules.dsp_iir_2p2z[0]
#define M11_LOAD_CURRENT_CONTROLLER_A_COEFFS     &g_controller_mtoc.dsp_modules.dsp_iir_2p2z[0].coeffs.s

#define M11_LOAD_CURRENT_CONTROLLER_B            &g_controller_ctom.dsp_modules.dsp_iir_2p2z[1]
#define M11_LOAD_CURRENT_CONTROLLER_B_COEFFS     &g_controller_mtoc.dsp_modules.dsp_iir_2p2z[1].coeffs.s

#define M21_LOAD_CURRENT_CONTROLLER_A            &g_controller_ctom.dsp_modules.dsp_iir_2p2z[4]
#define M21_LOAD_CURRENT_CONTROLLER_A_COEFFS     &g_controller_mtoc.dsp_modules.dsp_iir_2p2z[4].coeffs.s

#define M21_LOAD_CURRENT_CONTROLLER_B            &g_controller_ctom.dsp_modules.dsp_iir_2p2z[5]
#define M21_LOAD_CURRENT_CONTROLLER_B_COEFFS     &g_controller_mtoc.dsp_modules.dsp_iir_2p2z[5].coeffs.s

/// I2 load current controller
#define ERROR_CALCULATOR_I2            &g_controller_ctom.dsp_modules.dsp_error[1]

#define M12_LOAD_CURRENT_CONTROLLER_A            &g_controller_ctom.dsp_modules.dsp_iir_2p2z[2]
#define M12_LOAD_CURRENT_CONTROLLER_A_COEFFS     &g_controller_mtoc.dsp_modules.dsp_iir_2p2z[2].coeffs.s

#define M12_LOAD_CURRENT_CONTROLLER_B            &g_controller_ctom.dsp_modules.dsp_iir_2p2z[3]
#define M12_LOAD_CURRENT_CONTROLLER_B_COEFFS     &g_controller_mtoc.dsp_modules.dsp_iir_2p2z[3].coeffs.s

#define M22_LOAD_CURRENT_CONTROLLER_A            &g_controller_ctom.dsp_modules.dsp_iir_2p2z[6]
#define M22_LOAD_CURRENT_CONTROLLER_A_COEFFS     &g_controller_mtoc.dsp_modules.dsp_iir_2p2z[6].coeffs.s

#define M22_LOAD_CURRENT_CONTROLLER_B            &g_controller_ctom.dsp_modules.dsp_iir_2p2z[7]
#define M22_LOAD_CURRENT_CONTROLLER_B_COEFFS     &g_controller_mtoc.dsp_modules.dsp_iir_2p2z[7].coeffs.s

/// PWM modulators
#define PS1_PWM_MODULATOR               g_pwm_modules.pwm_regs[0]
#define PS1_PWM_MODULATOR_NEG           g_pwm_modules.pwm_regs[1]
#define PS2_PWM_MODULATOR               g_pwm_modules.pwm_regs[2]
#define PS2_PWM_MODULATOR_NEG           g_pwm_modules.pwm_regs[3]
#define PS3_PWM_MODULATOR               g_pwm_modules.pwm_regs[4]
#define PS3_PWM_MODULATOR_NEG           g_pwm_modules.pwm_regs[5]
#define PS4_PWM_MODULATOR               g_pwm_modules.pwm_regs[6]
#define PS4_PWM_MODULATOR_NEG           g_pwm_modules.pwm_regs[7]

/// Scope
#define PS_SCOPE                       SCOPE_CTOM[0]

/**
 * Interlocks defines
 */
typedef enum
{
    Load_Overcurrent
} hard_interlocks_t;

typedef enum
{
} soft_interlocks_t;

typedef enum
{
    High_Sync_Input_Frequency = 0x00000001
} alarms_t;

#define NUM_HARD_INTERLOCKS             Load_Overcurrent + 1
#define NUM_SOFT_INTERLOCKS             0

#define ISR_FREQ_INTERLOCK_TIMEBASE     5000.0

/**
 * Private functions
 */
#pragma CODE_SECTION(isr_init_controller, "ramfuncs");
#pragma CODE_SECTION(isr_controller, "ramfuncs");
#pragma CODE_SECTION(turn_off, "ramfuncs");
#pragma CODE_SECTION(open_relay, "ramfuncs");

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

static void open_relay(uint16_t id);
static void close_relay(uint16_t id);

static void reset_interlocks(uint16_t id);
static void check_interlocks_ps_module(uint16_t id);

static inline void run_dsp_pi_inline(dsp_pi_t *p_pi);
static inline void set_pwm_duty_hbridge_inline(volatile struct EPWM_REGS
                                               *p_pwm_module, float duty_pu);

/**
 * Main function for this power supply module
 */
void main_fbp_ufjf(void)
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

    Init_DMA_McBSP_nBuffers(NUM_PS_MODULES, DECIMATION_FACTOR, HRADC_SPI_CLK);

    Init_SPIMaster_McBSP(HRADC_SPI_CLK);
    Init_SPIMaster_Gpio();
    InitMcbspa20bit();

    DELAY_US(500000);
    send_ipc_lowpriority_msg(0,Enable_HRADC_Boards);
    DELAY_US(2000000);

    for(i = 0; i < NUM_PS_MODULES; i++)
    {
        Init_HRADC_Info(&HRADCs_Info.HRADC_boards[i], i, DECIMATION_FACTOR,
                        buffers_HRADC[i], TRANSDUCER_GAIN[i]);
        Config_HRADC_board(&HRADCs_Info.HRADC_boards[i], TRANSDUCER_OUTPUT_TYPE[i],
                           HRADC_HEATER_ENABLE[i], HRADC_MONITOR_ENABLE[i]);
    }

    HRADCs_Info.n_HRADC_boards = NUM_PS_MODULES;

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
    ConfigCpuTimer(&CpuTimer0, C28_FREQ_MHZ,
                   (1000000.0/ISR_FREQ_INTERLOCK_TIMEBASE));
    CpuTimer0Regs.TCR.bit.TIE = 0;
}

static void term_peripherals_drivers(void)
{
}

static void init_controller(void)
{
    init_ps_module(&g_ipc_ctom.ps_module[0],
                   g_ipc_mtoc.ps_module[0].ps_status.bit.model,
                   &turn_on, &turn_off, &isr_soft_interlock,
                   &isr_hard_interlock, &reset_interlocks);

    g_ipc_ctom.ps_module[1].ps_status.all = 0;
    g_ipc_ctom.ps_module[2].ps_status.all = 0;
    g_ipc_ctom.ps_module[3].ps_status.all = 0;

    init_event_manager(0, ISR_FREQ_INTERLOCK_TIMEBASE,
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


    /********************************************************/
    /** INITIALIZATION OF LOAD CURRENT I1 & I2 CONTROLLERS **/
    /********************************************************/

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
     *        name:     ERROR_CALCULATOR_I1
     * description:     Load current I1 reference error
     *  dsp module:     DSP_Error
     *           +:     I_LOAD_REFERENCE
     *           -:     I1_LOAD_CURRENT
     *         out:     I1_LOAD_ERROR
     */

    init_dsp_error(ERROR_CALCULATOR_I1, &I_LOAD_REFERENCE, &I1_LOAD_CURRENT,
                   &I1_LOAD_ERROR);

    /**
     *        name:     ERROR_CALCULATOR_I2
     * description:     Load current I2 reference error
     *  dsp module:     DSP_Error
     *           +:     I_LOAD_REFERENCE
     *           -:     I2_LOAD_CURRENT
     *         out:     I2_LOAD_ERROR
     */

    init_dsp_error(ERROR_CALCULATOR_I2, &I_LOAD_REFERENCE, &I2_LOAD_CURRENT,
                   &I2_LOAD_ERROR);

    /****************************************/
    /** INITIALIZATION OF M11 CONTROL LOOP **/
    /****************************************/

    /**
     *        name:     M11_LOAD_CURRENT_CONTROLLER_A
     * description:     Load current I1 M11 IIR 2P2Z controller A
     *  dsp module:     DSP_IIR_2P2Z
     *          in:     I1_LOAD_ERROR
     *         out:     I1_LOAD_ERROR_TO_M11
     */

    init_dsp_iir_2p2z(M11_LOAD_CURRENT_CONTROLLER_A,
                      M11_LOAD_CURRENT_CONTROLLER_A_COEFFS.b0,
                      M11_LOAD_CURRENT_CONTROLLER_A_COEFFS.b1,
                      M11_LOAD_CURRENT_CONTROLLER_A_COEFFS.b2,
                      M11_LOAD_CURRENT_CONTROLLER_A_COEFFS.a1,
                      M11_LOAD_CURRENT_CONTROLLER_A_COEFFS.a2,
                      FLT_MAX, -FLT_MAX,
                      &I1_LOAD_ERROR, &I1_LOAD_ERROR_TO_M11);

    /**
     *        name:     M11_LOAD_CURRENT_CONTROLLER_B
     * description:     Load current I1 M11 IIR 2P2Z controller B
     *  dsp module:     DSP_IIR_2P2Z
     *          in:     I1_LOAD_ERROR_TO_M11
     *         out:     M11
     */

    init_dsp_iir_2p2z(M11_LOAD_CURRENT_CONTROLLER_B,
                      M11_LOAD_CURRENT_CONTROLLER_B_COEFFS.b0,
                      M11_LOAD_CURRENT_CONTROLLER_B_COEFFS.b1,
                      M11_LOAD_CURRENT_CONTROLLER_B_COEFFS.b2,
                      M11_LOAD_CURRENT_CONTROLLER_B_COEFFS.a1,
                      M11_LOAD_CURRENT_CONTROLLER_B_COEFFS.a2,
                      PWM_MAX_DUTY, PWM_MIN_DUTY,
                      &I1_LOAD_ERROR_TO_M11, &M11);

    /****************************************/
    /** INITIALIZATION OF M12 CONTROL LOOP **/
    /****************************************/

    /**
     *        name:     M12_LOAD_CURRENT_CONTROLLER_A
     * description:     Load current I1 M12 IIR 2P2Z controller A
     *  dsp module:     DSP_IIR_2P2Z
     *          in:     I1_LOAD_ERROR
     *         out:     I1_LOAD_ERROR_TO_M12
     */

    init_dsp_iir_2p2z(M12_LOAD_CURRENT_CONTROLLER_A,
                      M12_LOAD_CURRENT_CONTROLLER_A_COEFFS.b0,
                      M12_LOAD_CURRENT_CONTROLLER_A_COEFFS.b1,
                      M12_LOAD_CURRENT_CONTROLLER_A_COEFFS.b2,
                      M12_LOAD_CURRENT_CONTROLLER_A_COEFFS.a1,
                      M12_LOAD_CURRENT_CONTROLLER_A_COEFFS.a2,
                      FLT_MAX, -FLT_MAX,
                      &I1_LOAD_ERROR, &I1_LOAD_ERROR_TO_M12);

    /**
     *        name:     M12_LOAD_CURRENT_CONTROLLER_B
     * description:     Load current I1 M12 IIR 2P2Z controller B
     *  dsp module:     DSP_IIR_2P2Z
     *          in:     I1_LOAD_ERROR_TO_M12
     *         out:     M11
     */

    init_dsp_iir_2p2z(M12_LOAD_CURRENT_CONTROLLER_B,
                      M12_LOAD_CURRENT_CONTROLLER_B_COEFFS.b0,
                      M12_LOAD_CURRENT_CONTROLLER_B_COEFFS.b1,
                      M12_LOAD_CURRENT_CONTROLLER_B_COEFFS.b2,
                      M12_LOAD_CURRENT_CONTROLLER_B_COEFFS.a1,
                      M12_LOAD_CURRENT_CONTROLLER_B_COEFFS.a2,
                      PWM_MAX_DUTY, PWM_MIN_DUTY,
                      &I1_LOAD_ERROR_TO_M12, &M12);

    /****************************************/
    /** INITIALIZATION OF M21 CONTROL LOOP **/
    /****************************************/

    /**
     *        name:     M21_LOAD_CURRENT_CONTROLLER_A
     * description:     Load current I1 M21 IIR 2P2Z controller A
     *  dsp module:     DSP_IIR_2P2Z
     *          in:     I2_LOAD_ERROR
     *         out:     I2_LOAD_ERROR_TO_M21
     */

    init_dsp_iir_2p2z(M21_LOAD_CURRENT_CONTROLLER_A,
                      M21_LOAD_CURRENT_CONTROLLER_A_COEFFS.b0,
                      M21_LOAD_CURRENT_CONTROLLER_A_COEFFS.b1,
                      M21_LOAD_CURRENT_CONTROLLER_A_COEFFS.b2,
                      M21_LOAD_CURRENT_CONTROLLER_A_COEFFS.a1,
                      M21_LOAD_CURRENT_CONTROLLER_A_COEFFS.a2,
                      FLT_MAX, -FLT_MAX,
                      &I2_LOAD_ERROR, &I2_LOAD_ERROR_TO_M21);

    /**
     *        name:     M21_LOAD_CURRENT_CONTROLLER_B
     * description:     Load current I1 M21 IIR 2P2Z controller B
     *  dsp module:     DSP_IIR_2P2Z
     *          in:     I2_LOAD_ERROR_TO_M21
     *         out:     M21
     */

    init_dsp_iir_2p2z(M21_LOAD_CURRENT_CONTROLLER_B,
                      M21_LOAD_CURRENT_CONTROLLER_B_COEFFS.b0,
                      M21_LOAD_CURRENT_CONTROLLER_B_COEFFS.b1,
                      M21_LOAD_CURRENT_CONTROLLER_B_COEFFS.b2,
                      M21_LOAD_CURRENT_CONTROLLER_B_COEFFS.a1,
                      M21_LOAD_CURRENT_CONTROLLER_B_COEFFS.a2,
                      PWM_MAX_DUTY, PWM_MIN_DUTY,
                      &I2_LOAD_ERROR_TO_M21, &M21);

    /****************************************/
    /** INITIALIZATION OF M22 CONTROL LOOP **/
    /****************************************/

    /**
     *        name:     M12_LOAD_CURRENT_CONTROLLER_A
     * description:     Load current I2 M22 IIR 2P2Z controller A
     *  dsp module:     DSP_IIR_2P2Z
     *          in:     I2_LOAD_ERROR
     *         out:     I2_LOAD_ERROR_TO_M22
     */

    init_dsp_iir_2p2z(M22_LOAD_CURRENT_CONTROLLER_A,
                      M22_LOAD_CURRENT_CONTROLLER_A_COEFFS.b0,
                      M22_LOAD_CURRENT_CONTROLLER_A_COEFFS.b1,
                      M22_LOAD_CURRENT_CONTROLLER_A_COEFFS.b2,
                      M22_LOAD_CURRENT_CONTROLLER_A_COEFFS.a1,
                      M22_LOAD_CURRENT_CONTROLLER_A_COEFFS.a2,
                      FLT_MAX, -FLT_MAX,
                      &I2_LOAD_ERROR, &I2_LOAD_ERROR_TO_M22);

    /**
     *        name:     M22_LOAD_CURRENT_CONTROLLER_B
     * description:     Load current I2 M22 IIR 2P2Z controller B
     *  dsp module:     DSP_IIR_2P2Z
     *          in:     I2_LOAD_ERROR_TO_M22
     *         out:     M22
     */

    init_dsp_iir_2p2z(M22_LOAD_CURRENT_CONTROLLER_B,
                      M22_LOAD_CURRENT_CONTROLLER_B_COEFFS.b0,
                      M22_LOAD_CURRENT_CONTROLLER_B_COEFFS.b1,
                      M22_LOAD_CURRENT_CONTROLLER_B_COEFFS.b2,
                      M22_LOAD_CURRENT_CONTROLLER_B_COEFFS.a1,
                      M22_LOAD_CURRENT_CONTROLLER_B_COEFFS.a2,
                      PWM_MAX_DUTY, PWM_MIN_DUTY,
                      &I2_LOAD_ERROR_TO_M22, &M22);


    /// Reset all internal variables
    reset_controller();
}

/**
 * Reset all internal variables from controller
 */
static void reset_controller(void)
{
    set_pwm_duty_hbridge(PS1_PWM_MODULATOR, 0.0);
    set_pwm_duty_hbridge(PS2_PWM_MODULATOR, 0.0);
    set_pwm_duty_hbridge(PS3_PWM_MODULATOR, 0.0);
    set_pwm_duty_hbridge(PS4_PWM_MODULATOR, 0.0);

    g_ipc_ctom.ps_module[0].ps_status.bit.openloop = LOOP_STATE;

    PS_SETPOINT = 0.0;
    PS_REFERENCE = 0.0;

    reset_dsp_srlim(SRLIM_I_LOAD_REFERENCE);

    reset_dsp_error(ERROR_CALCULATOR_I1);
    reset_dsp_error(ERROR_CALCULATOR_I2);

    reset_dsp_iir_2p2z(M11_LOAD_CURRENT_CONTROLLER_A);
    reset_dsp_iir_2p2z(M11_LOAD_CURRENT_CONTROLLER_B);
    reset_dsp_iir_2p2z(M21_LOAD_CURRENT_CONTROLLER_A);
    reset_dsp_iir_2p2z(M21_LOAD_CURRENT_CONTROLLER_B);
    reset_dsp_iir_2p2z(M12_LOAD_CURRENT_CONTROLLER_A);
    reset_dsp_iir_2p2z(M12_LOAD_CURRENT_CONTROLLER_B);
    reset_dsp_iir_2p2z(M22_LOAD_CURRENT_CONTROLLER_A);
    reset_dsp_iir_2p2z(M22_LOAD_CURRENT_CONTROLLER_B);

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

    /// Enable interlocks time-base timer
    CpuTimer0Regs.TCR.all = 0x4000;
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
    static uint16_t i;
    static float temp[4];

    //SET_DEBUG_GPIO0;
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
                        PS_REFERENCE(i) = PS_SETPOINT(i);
                        break;
                    }
                    case RmpWfm:
                    case MigWfm:
                    {
                        run_wfmref(&WFMREF[i]);
                        break;
                    }
                    case Cycle:
                    {
                        SIGGEN[i].amplitude = SIGGEN_MTOC[i].amplitude;
                        SIGGEN[i].offset = SIGGEN_MTOC[i].offset;
                        SIGGEN[i].p_run_siggen(&SIGGEN[i]);

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
                    g_controller_ctom.output_signals[i].f = 0.01 * PS_REFERENCE(i);

                    SATURATE(g_controller_ctom.output_signals[i].f,
                             PWM_MAX_DUTY_OL, PWM_MIN_DUTY_OL);
                }
                /// Closed-loop
                else
                {
                    SATURATE(PS_REFERENCE(i), MAX_REF[i], MIN_REF[i]);

                    //run_dsp_error(&g_controller_ctom.dsp_modules.dsp_error[i]);

                    *g_controller_ctom.dsp_modules.dsp_error[i].error =
                            *g_controller_ctom.dsp_modules.dsp_error[i].pos -
                            *g_controller_ctom.dsp_modules.dsp_error[i].neg;

                    run_dsp_pi_inline(&g_controller_ctom.dsp_modules.dsp_pi[i]);

                    //SATURATE(g_controller_ctom.output_signals[i].f,
                    //         PWM_MAX_DUTY, PWM_MIN_DUTY);
                }

                set_pwm_duty_hbridge_inline(g_pwm_modules.pwm_regs[i*2],
                                     g_controller_ctom.output_signals[i].f);
            }
        }
    }

    RUN_SCOPE(PS1_SCOPE);
    RUN_SCOPE(PS2_SCOPE);
    RUN_SCOPE(PS3_SCOPE);
    RUN_SCOPE(PS4_SCOPE);

    /**
     * Re-enable external interrupt 2 (XINT2) interrupts to allow sync pulses to
     * be handled once per isr_controller
     */
    if(PieCtrlRegs.PIEIER1.bit.INTx5 == 0)
    {
        /// Set alarm if counter is below limit when receiving new sync pulse
        if(counter_sync_period < MIN_NUM_ISR_CONTROLLER_SYNC)
        {
            /// Loop through active power supplies to set alarm
           for(i = 0; i < NUM_MAX_PS_MODULES; i++)
           {
               if(g_ipc_ctom.ps_module[i].ps_status.bit.active)
               {
                   g_ipc_ctom.ps_module[i].ps_alarms = High_Sync_Input_Frequency;
               }
           }
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
    PS1_PWM_MODULATOR->ETCLR.bit.INT = 1;
    PS1_PWM_MODULATOR_NEG->ETCLR.bit.INT = 1;
    PS2_PWM_MODULATOR->ETCLR.bit.INT = 1;
    PS2_PWM_MODULATOR_NEG->ETCLR.bit.INT = 1;
    PS3_PWM_MODULATOR->ETCLR.bit.INT = 1;
    PS3_PWM_MODULATOR_NEG->ETCLR.bit.INT = 1;
    PS4_PWM_MODULATOR->ETCLR.bit.INT = 1;
    PS4_PWM_MODULATOR_NEG->ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all |= M_INT3;

    //CLEAR_DEBUG_GPIO0;
    CLEAR_DEBUG_GPIO1;
}

/**
 * Initialization of interruptions.
 */
static void init_interruptions(void)
{
    EALLOW;
    PieVectTable.EPWM1_INT = &isr_init_controller;
    //PieVectTable.EPWM2_INT = &isr_controller;
    PieVectTable.TINT0     = &isr_interlocks_timebase;
    EDIS;

    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;  /// ePWM1
    //PieCtrlRegs.PIEIER3.bit.INTx2 = 1;  /// ePWM2
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;  /// CpuTimer0

    enable_pwm_interrupt(PS4_PWM_MODULATOR);
    //enable_pwm_interrupt(PS4_PWM_MODULATOR_NEG);

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
            if(fabs(g_controller_mtoc.net_signals[id].f) < MIN_DCLINK(id))
            {
                BYPASS_HARD_INTERLOCK_DEBOUNCE(id, DCLink_Undervoltage);
                set_hard_interlock(id, DCLink_Undervoltage);
            }

            /*switch(id)
            {
                case 0:
                {
                    if(!PIN_STATUS_PS1_FUSE)
                    {
                        BYPASS_HARD_INTERLOCK_DEBOUNCE(0, DCLink_Fuse_Fault);
                        set_hard_interlock(0, DCLink_Fuse_Fault);
                    }
                    break;
                }

                case 1:
                {
                    if(!PIN_STATUS_PS2_FUSE)
                    {
                        BYPASS_HARD_INTERLOCK_DEBOUNCE(1, DCLink_Fuse_Fault);
                        set_hard_interlock(1, DCLink_Fuse_Fault);
                    }
                    break;
                }

                case 2:
                {
                    if(!PIN_STATUS_PS3_FUSE)
                    {
                        BYPASS_HARD_INTERLOCK_DEBOUNCE(2, DCLink_Fuse_Fault);
                        set_hard_interlock(2, DCLink_Fuse_Fault);
                    }
                    break;
                }

                case 3:
                {
                    if(!PIN_STATUS_PS4_FUSE)
                    {
                        BYPASS_HARD_INTERLOCK_DEBOUNCE(3, DCLink_Fuse_Fault);
                        set_hard_interlock(3, DCLink_Fuse_Fault);
                    }
                    break;
                }
            }*/

            #ifdef USE_ITLK
            if(g_ipc_ctom.ps_module[id].ps_status.bit.state == Off)
            #else
            if(g_ipc_ctom.ps_module[id].ps_status.bit.state <= Interlock)
            #endif
            {
                close_relay(id);

                g_ipc_ctom.ps_module[id].ps_status.bit.state = SlowRef;

                enable_pwm_output(2*id);
                enable_pwm_output((2*id)+1);
            }

            else if(g_ipc_ctom.ps_module[id].ps_status.bit.state == Interlock)
            {
                reset_controller(id);
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
    g_ipc_ctom.ps_module[id].ps_alarms = 0;

    if(g_ipc_ctom.ps_module[id].ps_status.bit.state < Initializing)
    {
        g_ipc_ctom.ps_module[id].ps_status.bit.state = Off;
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
 * Check variables from specified power supply for interlocks
 *
 * @param id specified power supply
 */
static void check_interlocks_ps_module(uint16_t id)
{
    if(fabs(g_controller_ctom.net_signals[id].f) > MAX_ILOAD(id))
    {
        set_hard_interlock(id, Load_Overcurrent);
    }

    if(fabs(g_controller_mtoc.net_signals[id].f) > MAX_DCLINK(id))
    {
        set_hard_interlock(id, DCLink_Overvoltage);
    }
/*
    if(fabs(g_controller_mtoc.net_signals[id+4].f) > MAX_VLOAD(id))
    {
        set_hard_interlock(id, Load_Overvoltage);
    }

    if(fabs(g_controller_mtoc.net_signals[id+8].f) > MAX_TEMP(id))
    {
        set_soft_interlock(id, Heatsink_Overtemperature);
    }
*/
    switch(id)
    {
        case 0:
        {
            /*if(!PIN_STATUS_PS1_DRIVER_ERROR)
            {
                set_hard_interlock(0, MOSFETs_Driver_Fault);
            }

            IER &= ~M_INT11;

            if ( (g_ipc_ctom.ps_module[0].ps_status.bit.state <= Interlock) &&
                 (PIN_STATUS_PS1_DCLINK_RELAY) )
            {
                set_hard_interlock(0, Welded_Relay_Fault);
            }

            else*/ if (g_ipc_ctom.ps_module[0].ps_status.bit.state > Interlock)
            {
                /*if(!PIN_STATUS_PS1_DCLINK_RELAY)
                {
                    set_hard_interlock(0, Opened_Relay_Fault);
                }

                if(!PIN_STATUS_PS1_FUSE)
                {
                    set_hard_interlock(0, DCLink_Fuse_Fault);
                }*/

                if(fabs(g_controller_mtoc.net_signals[0].f) < MIN_DCLINK(0))
                {
                    set_hard_interlock(0, DCLink_Undervoltage);
                }
            }

            break;
        }

        case 1:
        {
            /*if(!PIN_STATUS_PS2_DRIVER_ERROR)
            {
                set_hard_interlock(1, MOSFETs_Driver_Fault);
            }

            IER &= ~M_INT11;

            if ( (g_ipc_ctom.ps_module[1].ps_status.bit.state <= Interlock) &&
                (PIN_STATUS_PS2_DCLINK_RELAY))
            {
                set_hard_interlock(1, Welded_Relay_Fault);
            }

            else*/ if (g_ipc_ctom.ps_module[1].ps_status.bit.state > Interlock)
            {
                /*if(!PIN_STATUS_PS2_DCLINK_RELAY)
                {
                    set_hard_interlock(1, Opened_Relay_Fault);
                }

                if(!PIN_STATUS_PS2_FUSE)
                {
                    set_hard_interlock(1, DCLink_Fuse_Fault);
                }*/

                if(fabs(g_controller_mtoc.net_signals[1].f) < MIN_DCLINK(1))
                {
                    set_hard_interlock(1, DCLink_Undervoltage);
                }
            }

            break;
        }

        case 2:
        {
            /*if(!PIN_STATUS_PS3_DRIVER_ERROR)
            {
                set_hard_interlock(2, MOSFETs_Driver_Fault);
            }

            IER &= ~M_INT11;

            if ( (g_ipc_ctom.ps_module[2].ps_status.bit.state <= Interlock) &&
                (PIN_STATUS_PS3_DCLINK_RELAY)) {
                set_hard_interlock(2, Welded_Relay_Fault);
            }

            else*/ if (g_ipc_ctom.ps_module[2].ps_status.bit.state > Interlock)
            {
                /*if(!PIN_STATUS_PS3_DCLINK_RELAY)
                {
                    set_hard_interlock(2, Opened_Relay_Fault);
                }

                if(!PIN_STATUS_PS3_FUSE)
                {
                    set_hard_interlock(2, DCLink_Fuse_Fault);
                }*/

                if(fabs(g_controller_mtoc.net_signals[2].f) < MIN_DCLINK(2))
                {
                    set_hard_interlock(2, DCLink_Undervoltage);
                }

            }

            break;
        }

        case 3:
        {
            /*if(!PIN_STATUS_PS4_DRIVER_ERROR)
            {
                set_hard_interlock(3, MOSFETs_Driver_Fault);
            }

            IER &= ~M_INT11;

            if ( (g_ipc_ctom.ps_module[3].ps_status.bit.state <= Interlock) &&
                (PIN_STATUS_PS4_DCLINK_RELAY)) {
                set_hard_interlock(3, Welded_Relay_Fault);
            }

            else*/ if (g_ipc_ctom.ps_module[3].ps_status.bit.state > Interlock)
            {
                /*if(!PIN_STATUS_PS4_DCLINK_RELAY)
                {
                    set_hard_interlock(3, Opened_Relay_Fault);
                }

                if(!PIN_STATUS_PS4_FUSE)
                {
                    set_hard_interlock(3, DCLink_Fuse_Fault);
                }*/

                if(fabs(g_controller_mtoc.net_signals[3].f) < MIN_DCLINK(3))
                {
                    set_hard_interlock(3, DCLink_Undervoltage);
                }
            }

            break;
        }
    }

    IER |= M_INT11;

    run_interlocks_debouncing(id);
}

static inline void run_dsp_pi_inline(dsp_pi_t *p_pi)
{
    float dyn_max;
    float dyn_min;
    float temp;

    temp = *(p_pi->in) * p_pi->coeffs.s.kp;
    SATURATE(temp, p_pi->coeffs.s.u_max, p_pi->coeffs.s.u_min);
    p_pi->u_prop = temp;

    dyn_max = (p_pi->coeffs.s.u_max - temp);
    dyn_min = (p_pi->coeffs.s.u_min - temp);

    temp = p_pi->u_int + *(p_pi->in) * p_pi->coeffs.s.ki;
    SATURATE(temp, dyn_max, dyn_min);
    p_pi->u_int = temp;

    *(p_pi->out) = p_pi->u_int + p_pi->u_prop;
}

static inline void set_pwm_duty_hbridge_inline(volatile struct EPWM_REGS
                                               *p_pwm_module, float duty_pu)
{
    uint16_t duty_int;
    uint16_t duty_frac;
    float duty;

    duty = (0.5 * duty_pu + 0.5) * (float)p_pwm_module->TBPRD;

    duty_int  = (uint16_t) duty;
    duty_frac = ((uint16_t) ((duty - (float)duty_int) * MEP_ScaleFactor)) << 8;
    duty_frac += 0x0180;

    p_pwm_module->CMPAM2.half.CMPA    = duty_int;
    p_pwm_module->CMPAM2.half.CMPAHR  = duty_frac;
}
