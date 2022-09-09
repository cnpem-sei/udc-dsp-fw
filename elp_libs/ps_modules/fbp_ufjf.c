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
#define MAX_VLOAD           ANALOG_VARS_MAX[1]

/**
 * Controller defines
 */
#define DECIMATION_FACTOR               1

/// Reference
#define SIGGEN                          SIGGEN_CTOM[0]
#define SIGGEN_OUTPUT                   g_controller_ctom.net_signals[20].f
#define SRLIM_SIGGEN_AMP                &g_controller_ctom.dsp_modules.dsp_srlim[1]
#define SRLIM_SIGGEN_OFFSET             &g_controller_ctom.dsp_modules.dsp_srlim[2]

#define MAX_SLEWRATE_SLOWREF            g_controller_mtoc.dsp_modules.dsp_srlim[0].coeffs.s.max_slewrate
#define MAX_SLEWRATE_SIGGEN_AMP         g_controller_mtoc.dsp_modules.dsp_srlim[1].coeffs.s.max_slewrate
#define MAX_SLEWRATE_SIGGEN_OFFSET      g_controller_mtoc.dsp_modules.dsp_srlim[2].coeffs.s.max_slewrate

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
#define PS1_LOAD_VOLTAGE                g_controller_mtoc.net_signals[0].f  // ANI6
#define PS2_LOAD_VOLTAGE                g_controller_mtoc.net_signals[1].f  // ANI7
#define PS3_LOAD_VOLTAGE                g_controller_mtoc.net_signals[2].f  // ANI3
#define PS4_LOAD_VOLTAGE                g_controller_mtoc.net_signals[3].f  // ANI5

/// I1 load current controller
#define ERROR_CALCULATOR_I1             &g_controller_ctom.dsp_modules.dsp_error[0]

#define M11_LOAD_CURRENT_CONTROLLER_A            &g_controller_ctom.dsp_modules.dsp_iir_2p2z[0]
#define M11_LOAD_CURRENT_CONTROLLER_A_COEFFS     g_controller_mtoc.dsp_modules.dsp_iir_2p2z[0].coeffs.s

#define M11_LOAD_CURRENT_CONTROLLER_B            &g_controller_ctom.dsp_modules.dsp_iir_2p2z[1]
#define M11_LOAD_CURRENT_CONTROLLER_B_COEFFS     g_controller_mtoc.dsp_modules.dsp_iir_2p2z[1].coeffs.s

#define M21_LOAD_CURRENT_CONTROLLER_A            &g_controller_ctom.dsp_modules.dsp_iir_2p2z[4]
#define M21_LOAD_CURRENT_CONTROLLER_A_COEFFS     g_controller_mtoc.dsp_modules.dsp_iir_2p2z[4].coeffs.s

#define M21_LOAD_CURRENT_CONTROLLER_B            &g_controller_ctom.dsp_modules.dsp_iir_2p2z[5]
#define M21_LOAD_CURRENT_CONTROLLER_B_COEFFS     g_controller_mtoc.dsp_modules.dsp_iir_2p2z[5].coeffs.s

/// I2 load current controller
#define ERROR_CALCULATOR_I2            &g_controller_ctom.dsp_modules.dsp_error[1]

#define M12_LOAD_CURRENT_CONTROLLER_A            &g_controller_ctom.dsp_modules.dsp_iir_2p2z[2]
#define M12_LOAD_CURRENT_CONTROLLER_A_COEFFS     g_controller_mtoc.dsp_modules.dsp_iir_2p2z[2].coeffs.s

#define M12_LOAD_CURRENT_CONTROLLER_B            &g_controller_ctom.dsp_modules.dsp_iir_2p2z[3]
#define M12_LOAD_CURRENT_CONTROLLER_B_COEFFS     g_controller_mtoc.dsp_modules.dsp_iir_2p2z[3].coeffs.s

#define M22_LOAD_CURRENT_CONTROLLER_A            &g_controller_ctom.dsp_modules.dsp_iir_2p2z[6]
#define M22_LOAD_CURRENT_CONTROLLER_A_COEFFS     g_controller_mtoc.dsp_modules.dsp_iir_2p2z[6].coeffs.s

#define M22_LOAD_CURRENT_CONTROLLER_B            &g_controller_ctom.dsp_modules.dsp_iir_2p2z[7]
#define M22_LOAD_CURRENT_CONTROLLER_B_COEFFS     g_controller_mtoc.dsp_modules.dsp_iir_2p2z[7].coeffs.s

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
    I1_Load_Overcurrent,
    I2_Load_Overcurrent,
    Load_Overvoltage_Mod_1,
    Load_Overvoltage_Mod_2,
    Load_Overvoltage_Mod_3,
    Load_Overvoltage_Mod_4
} hard_interlocks_t;

typedef enum
{
    High_Sync_Input_Frequency = 0x00000001
} alarms_t;

#define NUM_HARD_INTERLOCKS             Load_Overvoltage_Mod_4 + 1
#define NUM_SOFT_INTERLOCKS             0

#define ISR_FREQ_INTERLOCK_TIMEBASE     5000.0

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
static void reset_controllers(void);
static void enable_controller();
static void disable_controller();
static interrupt void isr_init_controller(void);
static interrupt void isr_controller(void);

static void init_interruptions(void);
static void term_interruptions(void);

static void turn_on(uint16_t dummy);
static void turn_off(uint16_t dummy);

static void reset_interlocks(uint16_t dummy);
static void check_interlocks(void);

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
        check_interlocks();
    }

    turn_off(0);

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

    Init_DMA_McBSP_nBuffers(NUM_HRADC_BOARDS, DECIMATION_FACTOR, HRADC_SPI_CLK);

    Init_SPIMaster_McBSP(HRADC_SPI_CLK);
    Init_SPIMaster_Gpio();
    InitMcbspa20bit();

    DELAY_US(500000);
    send_ipc_lowpriority_msg(0,Enable_HRADC_Boards);
    DELAY_US(2000000);

    for(i = 0; i < NUM_HRADC_BOARDS; i++)
    {
        Init_HRADC_Info(&HRADCs_Info.HRADC_boards[i], i, DECIMATION_FACTOR,
                        buffers_HRADC[i], TRANSDUCER_GAIN[i]);
        Config_HRADC_board(&HRADCs_Info.HRADC_boards[i], TRANSDUCER_OUTPUT_TYPE[i],
                           HRADC_HEATER_ENABLE[i], HRADC_MONITOR_ENABLE[i]);
    }

    HRADCs_Info.n_HRADC_boards = NUM_HRADC_BOARDS;

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

    /******************************/
    /** INITIALIZATION OF SCOPES **/
    /******************************/
    init_scope(&PS_SCOPE, ISR_CONTROL_FREQ, SCOPE_FREQ_SAMPLING_PARAM[0],
               &g_buf_samples_ctom[0], SIZE_BUF_SAMPLES_CTOM,
               SCOPE_SOURCE_PARAM[0], &run_scope_shared_ram);

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

    I_LOAD_SETPOINT = 0.0;
    I_LOAD_REFERENCE = 0.0;

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

    disable_siggen(&SIGGEN);
    reset_dsp_srlim(SRLIM_SIGGEN_AMP);
    reset_dsp_srlim(SRLIM_SIGGEN_OFFSET);

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
static void isr_controller(void)
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

    I1_LOAD_CURRENT = temp[0];
    PS2_LOAD_CURRENT = temp[1];
    PS3_LOAD_CURRENT = temp[2];
    I2_LOAD_CURRENT = temp[3];

    /// Check whether power supply is ON
    if(g_ipc_ctom.ps_module[0].ps_status.bit.state > Interlock)
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
            SATURATE(I_LOAD_REFERENCE, MAX_REF_OL[0], MIN_REF_OL[0]);
            DUTY_CYCLE_I1 = 0.01 * I_LOAD_REFERENCE;
            SATURATE(DUTY_CYCLE_I1, PWM_MAX_DUTY_OL, PWM_MIN_DUTY_OL);

            DUTY_CYCLE_I2 = DUTY_CYCLE_I1;
        }
        /// Closed-loop
        else
        {
            SATURATE(I_LOAD_REFERENCE, MAX_REF[0], MIN_REF[0]);

            /// Load current controller
            run_dsp_error(ERROR_CALCULATOR_I1);
            run_dsp_error(ERROR_CALCULATOR_I2);

            run_dsp_iir_2p2z(M11_LOAD_CURRENT_CONTROLLER_A);
            run_dsp_iir_2p2z(M11_LOAD_CURRENT_CONTROLLER_B);
            run_dsp_iir_2p2z(M12_LOAD_CURRENT_CONTROLLER_A);
            run_dsp_iir_2p2z(M12_LOAD_CURRENT_CONTROLLER_B);
            run_dsp_iir_2p2z(M21_LOAD_CURRENT_CONTROLLER_A);
            run_dsp_iir_2p2z(M21_LOAD_CURRENT_CONTROLLER_B);
            run_dsp_iir_2p2z(M22_LOAD_CURRENT_CONTROLLER_A);
            run_dsp_iir_2p2z(M22_LOAD_CURRENT_CONTROLLER_B);

            DUTY_CYCLE_I1 = M11 + M21;
            DUTY_CYCLE_I2 = M12 + M22;

            SATURATE(DUTY_CYCLE_I1, PWM_MAX_DUTY, PWM_MIN_DUTY);
            SATURATE(DUTY_CYCLE_I2, PWM_MAX_DUTY, PWM_MIN_DUTY);
        }

        set_pwm_duty_hbridge(PS1_PWM_MODULATOR, DUTY_CYCLE_I1);
        set_pwm_duty_hbridge(PS2_PWM_MODULATOR, DUTY_CYCLE_I2);
        set_pwm_duty_hbridge(PS3_PWM_MODULATOR, DUTY_CYCLE_I1);
        set_pwm_duty_hbridge(PS4_PWM_MODULATOR, DUTY_CYCLE_I2);
    }

    RUN_SCOPE(PS_SCOPE);

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
static void turn_on(uint16_t dummy)
{
    #ifdef USE_ITLK
    if(g_ipc_ctom.ps_module[0].ps_status.bit.state == Off)
    #else
    if(g_ipc_ctom.ps_module[id].ps_status.bit.state <= Interlock)
    #endif
    {

        g_ipc_ctom.ps_module[0].ps_status.bit.state = SlowRef;

        enable_pwm_output(0);
        enable_pwm_output(1);
        enable_pwm_output(2);
        enable_pwm_output(3);
        enable_pwm_output(4);
        enable_pwm_output(5);
        enable_pwm_output(6);
        enable_pwm_output(7);
    }
}

/**
 * Turn off specified power supply.
 *
 * @param id specified power supply
 */
static void turn_off(uint16_t dummy)
{
    disable_pwm_output(0);
    disable_pwm_output(1);
    disable_pwm_output(2);
    disable_pwm_output(3);
    disable_pwm_output(4);
    disable_pwm_output(5);
    disable_pwm_output(6);
    disable_pwm_output(7);

    if (g_ipc_ctom.ps_module[0].ps_status.bit.state != Interlock)
    {
        g_ipc_ctom.ps_module[0].ps_status.bit.state = Off;
    }

    reset_controller();
}

/**
 * Reset interlocks for specified power supply.
 *
 * @param id specified power supply
 */
static void reset_interlocks(uint16_t dummy)
{
    g_ipc_ctom.ps_module[0].ps_hard_interlock = 0;
    g_ipc_ctom.ps_module[0].ps_soft_interlock = 0;
    g_ipc_ctom.ps_module[0].ps_alarms = 0;

    if(g_ipc_ctom.ps_module[0].ps_status.bit.state < Initializing)
    {
        g_ipc_ctom.ps_module[0].ps_status.bit.state = Off;
    }
}

/**
 * Check variables from specified power supply for interlocks
 */
static void check_interlocks(void)
{
    if(fabs(I1_LOAD_CURRENT) > MAX_ILOAD)
    {
        set_hard_interlock(0, I1_Load_Overcurrent);
    }

    if(fabs(I2_LOAD_CURRENT) > MAX_ILOAD)
    {
        set_hard_interlock(0, I2_Load_Overcurrent);
    }

    if(fabs(PS1_LOAD_VOLTAGE) > MAX_VLOAD)
    {
        set_hard_interlock(0, Load_Overvoltage_Mod_1);
    }

    if(fabs(PS2_LOAD_VOLTAGE) > MAX_VLOAD)
    {
        set_hard_interlock(0, Load_Overvoltage_Mod_2);
    }

    if(fabs(PS3_LOAD_VOLTAGE) > MAX_VLOAD)
    {
        set_hard_interlock(0, Load_Overvoltage_Mod_3);
    }

    if(fabs(PS4_LOAD_VOLTAGE) > MAX_VLOAD)
    {
        set_hard_interlock(0, Load_Overvoltage_Mod_4);
    }

    run_interlocks_debouncing(0);
}
