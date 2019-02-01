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
 * @file fbp_2s_ufjf.c
 * @brief Module of FPB-2S for UFJF
 * 
 * Module for control of two FBP modules operating in series, used by partners
 * from UFJF to study new control strategies for Sirius.
 *
 * @author gabriel.brunheira
 * @date 01/02/2019
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

#include "fbp_2s_ufjf.h"

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

#define ISR_CONTROL_FREQ        g_ipc_mtoc.control.freq_isr_control

#define TIMESLICER_BUFFER       1
#define BUFFER_FREQ             g_ipc_mtoc.control.freq_timeslicer[TIMESLICER_BUFFER]
#define BUFFER_DECIMATION       (uint16_t) roundf(ISR_CONTROL_FREQ / BUFFER_FREQ)

#define SIGGEN                  g_ipc_ctom.siggen

#define BUF_SAMPLES             &g_ipc_ctom.buf_samples[0]

/**
 * HRADC parameters
 */
#define HRADC_FREQ_SAMP         g_ipc_mtoc.hradc.freq_hradc_sampling
#define HRADC_SPI_CLK           g_ipc_mtoc.hradc.freq_spiclk
#define NUM_HRADC_BOARDS        g_ipc_mtoc.hradc.num_hradc
#define DECIMATION_FACTOR       1//(HRADC_FREQ_SAMP/ISR_CONTROL_FREQ)

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
#define MAX_ILOAD                   g_ipc_mtoc.analog_vars.max[0]
#define MAX_VLOAD                   g_ipc_mtoc.analog_vars.max[1]
#define MIN_DCLINK                  g_ipc_mtoc.analog_vars.min[2]
#define MAX_DCLINK                  g_ipc_mtoc.analog_vars.max[2]
#define MAX_TEMP                    g_ipc_mtoc.analog_vars.max[3]

#define NETSIGNAL_ELEM_CTOM_BUF_1   g_ipc_mtoc.analog_vars.max[4]
#define NETSIGNAL_ELEM_CTOM_BUF_2   g_ipc_mtoc.analog_vars.max[5]

#define NETSIGNAL_CTOM_BUF_1      g_controller_ctom.net_signals[(uint16_t) NETSIGNAL_ELEM_CTOM_BUF_1].f
#define NETSIGNAL_CTOM_BUF_2      g_controller_ctom.net_signals[(uint16_t) NETSIGNAL_ELEM_CTOM_BUF_2].f

/**
 * Controller defines
 */
#define I_LOAD_SETPOINT         g_ipc_ctom.ps_module[0].ps_setpoint
#define I_LOAD_REFERENCE        g_ipc_ctom.ps_module[0].ps_reference

#define SRLIM_I_LOAD_REFERENCE  &g_controller_ctom.dsp_modules.dsp_srlim[0]
#define ERROR_I_LOAD            &g_controller_ctom.dsp_modules.dsp_error[0]

#define STATE_OBSERVER          &g_controller_ctom.dsp_modules.dsp_vect_product[0]
#define STATE_OBSERVER_IN       &I_LOAD_K_1         // In:  net_signals[1..7]
#define STATE_OBSERVER_OUT      &I_L_MOD1_K         // Out: net_signals[8..9]
#define NUM_ROWS_OBSERVER       4
#define NUM_COLUMNS_OBSERVER    12

#define STATE_CONTROLLER        &g_controller_ctom.dsp_modules.dsp_vect_product[1]
#define STATE_CONTROLLER_IN     &M_MOD1_K_1         // In:  net_signals[5..11]
#define STATE_CONTROLLER_OUT    &DUTY_CYCLE_MOD_1   // Out: output_signals[0]
#define NUM_ROWS_CONTROLLER     2
#define NUM_COLUMNS_CONTROLLER  11

#define DUTY_CYCLE_MOD_1        g_controller_ctom.output_signals[0].f
#define DUTY_CYCLE_MOD_3        g_controller_ctom.output_signals[1].f

/**
 * States vector defines
 */
#define I_LOAD_K_1          g_controller_ctom.net_signals[1].f  //-----
#define V_C_MOD1_K_1        g_controller_ctom.net_signals[2].f  //  |
#define V_C_MOD3_K_1        g_controller_ctom.net_signals[3].f  //  |
#define I_L_MOD1_K_1        g_controller_ctom.net_signals[4].f  //  |
#define V_D_MOD1_K_1        g_controller_ctom.net_signals[5].f  //  |
#define I_L_MOD3_K_1        g_controller_ctom.net_signals[6].f  // STATE_OBSERVER_IN
#define V_D_MOD3_K_1        g_controller_ctom.net_signals[7].f  //  |
#define M_MOD1_K_1          g_controller_ctom.net_signals[8].f  //  |     -----
#define M_MOD3_K_1          g_controller_ctom.net_signals[9].f  //  |       |
#define I_LOAD_K            g_controller_ctom.net_signals[10].f //  |       |
#define V_C_MOD1_K          g_controller_ctom.net_signals[11].f //  |       |
#define V_C_MOD3_K          g_controller_ctom.net_signals[12].f //-----     |
#define I_L_MOD1_K          g_controller_ctom.net_signals[13].f //          |
#define I_L_MOD3_K          g_controller_ctom.net_signals[14].f //  STATE_CONTROLLER_IN
#define V_D_MOD1_K          g_controller_ctom.net_signals[15].f //          |
#define V_D_MOD3_K          g_controller_ctom.net_signals[16].f //          |
#define REF_K               g_controller_ctom.net_signals[17].f //          |
#define Q_K_1               g_controller_ctom.net_signals[18].f //        -----

/**
 * Auxiliary states defines
 */
#define Q_K                 g_controller_ctom.net_signals[19].f
#define I_LOAD_ERROR        g_controller_ctom.net_signals[20].f
#define SIZE_STATE_VECTOR   18

/**
 * Power supply 1 defines
 */
#define MOD1_ID                          0x0000

#define PIN_OPEN_MOD1_DCLINK_RELAY       CLEAR_GPDO4;
#define PIN_CLOSE_MOD1_DCLINK_RELAY      SET_GPDO4;

#define PIN_STATUS_MOD1_DCLINK_RELAY     GET_GPDI4
#define PIN_STATUS_MOD1_DRIVER_ERROR     GET_GPDI5
#define PIN_STATUS_MOD1_FUSE             GET_GPDI14

#define MOD1_DCLINK_VOLTAGE              g_controller_mtoc.net_signals[0].f  // ANI2
#define MOD1_LOAD_VOLTAGE                g_controller_mtoc.net_signals[4].f  // ANI6
#define MOD1_TEMPERATURE                 g_controller_mtoc.net_signals[8].f  // I2C Add 0x48

#define PWM_MOD1_A                       g_pwm_modules.pwm_regs[0]
#define PWM_MOD1_B                       g_pwm_modules.pwm_regs[1]

/**
 * Power supply 2 defines
 */
#define MOD2_ID                          0x0001

#define PIN_OPEN_MOD2_DCLINK_RELAY       CLEAR_GPDO3;
#define PIN_CLOSE_MOD2_DCLINK_RELAY      SET_GPDO3;

#define PIN_STATUS_MOD2_DCLINK_RELAY     GET_GPDI11
#define PIN_STATUS_MOD2_DRIVER_ERROR     GET_GPDI9
#define PIN_STATUS_MOD2_FUSE             GET_GPDI16

#define MOD2_DCLINK_VOLTAGE              g_controller_mtoc.net_signals[1].f  // ANI1
#define MOD2_LOAD_VOLTAGE                g_controller_mtoc.net_signals[5].f  // ANI7
#define MOD2_TEMPERATURE                 g_controller_mtoc.net_signals[9].f  // I2C Add 0x49

#define PWM_MOD2_A                       g_pwm_modules.pwm_regs[2]
#define PWM_MOD2_B                       g_pwm_modules.pwm_regs[3]

/**
 * Power supply 3 defines
 */
#define MOD3_ID                          0x0002

#define PIN_OPEN_MOD3_DCLINK_RELAY       CLEAR_GPDO1;
#define PIN_CLOSE_MOD3_DCLINK_RELAY      SET_GPDO1;

#define PIN_STATUS_MOD3_DCLINK_RELAY     GET_GPDI8
#define PIN_STATUS_MOD3_DRIVER_ERROR     GET_GPDI1
#define PIN_STATUS_MOD3_FUSE             GET_GPDI13

#define MOD3_DCLINK_VOLTAGE              g_controller_mtoc.net_signals[2].f  // ANI4
#define MOD3_LOAD_VOLTAGE                g_controller_mtoc.net_signals[6].f  // ANI3
#define MOD3_TEMPERATURE                 g_controller_mtoc.net_signals[10].f // I2C Add 0x4A

#define PWM_MOD3_A                       g_pwm_modules.pwm_regs[4]
#define PWM_MOD3_B                       g_pwm_modules.pwm_regs[5]

/**
 * Power supply 4 defines
 */
#define MOD4_ID                          0x0003

#define PIN_OPEN_MOD4_DCLINK_RELAY       CLEAR_GPDO2;
#define PIN_CLOSE_MOD4_DCLINK_RELAY      SET_GPDO2;

#define PIN_STATUS_MOD4_DCLINK_RELAY     GET_GPDI2
#define PIN_STATUS_MOD4_DRIVER_ERROR     GET_GPDI3
#define PIN_STATUS_MOD4_FUSE             GET_GPDI15

#define MOD4_DCLINK_VOLTAGE              g_controller_mtoc.net_signals[3].f  // ANI0
#define MOD4_LOAD_VOLTAGE                g_controller_mtoc.net_signals[7].f  // ANI5
#define MOD4_TEMPERATURE                 g_controller_mtoc.net_signals[11].f // I2C Add 0x4C

#define PWM_MOD4_A                       g_pwm_modules.pwm_regs[6]
#define PWM_MOD4_B                       g_pwm_modules.pwm_regs[7]

/**
 * Interlocks defines
 */
typedef enum
{
    Load_Overcurrent,
    Load_Overvoltage,
    DCLink_Overvoltage,
    DCLink_Undervoltage,
    DCLink_Relay_Fault,
    DCLink_Fuse_Fault,
    MOSFETs_Driver_Fault
} hard_interlocks_t;

typedef enum
{
    Heatsink_Overtemperature
} soft_interlocks_t;

#define NUM_HARD_INTERLOCKS             MOSFETs_Driver_Fault + 1
#define NUM_SOFT_INTERLOCKS             Heatsink_Overtemperature + 1

#define ISR_FREQ_INTERLOCK_TIMEBASE     5000.0

/**
 * Matrices for State-space controller
 */
static float observer[NUM_ROWS_OBSERVER][NUM_COLUMNS_OBSERVER] =
{{0.969041209900195,-0.043965647799023,0.000402928453119,0.024521862813578,-0.210876274118827,0.000510449105958,0.000257030257273,1.343864000756771,-0.003101784376373,-0.000056375149930,0.035514921487651,0.000827103995989},
{0.969041209900196,0.000402928453119,-0.043965647799023,0.000510449105958,0.000257030257273,0.024521862813578,-0.210876274118827,-0.003101784376373,1.343864000756770,-0.000056375149930,0.000827103995989,0.035514921487651},
{-0.194427074441541,0.086492835605399,-0.000035401122785,0.195558089871086,0.699232500061746,0.001335327771268,0.000307350271742,-0.940412069816523,0.001405671235379,0.001349231582072,0.369122137898272,0.000179173424345},
{-0.194427074441541,-0.000035401122785,0.086492835605399,0.001335327771268,0.000307350271742,0.195558089871086,0.699232500061746,0.001405671235379,-0.940412069816521,0.001349231582072,0.000179173424345,0.369122137898271}};

static float controller[NUM_ROWS_CONTROLLER][NUM_COLUMNS_CONTROLLER] =
{{-0.041506935552427,   -0.035578915727007,   -4.053385511037375,  0.000100909684059,   -0.000029358577199,   -0.014034771730487,  -0.013409582708159,     -0.000223255078557,   -0.000350386725972,  4.053385511037375,  0.123595600030828},
{ -0.035578915727007,   -0.041506935552427,   -4.053385511037396,  -0.000029358577199,  0.000100909684059,   -0.013409582708159,   -0.01403477173048,     -0.000350386725972,   -0.000223255078557,  4.053385511037396,  0.123595600030829}};


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
static void reset_controller(void);
static void enable_controller();
static void disable_controller();
static interrupt void isr_init_controller(void);
static interrupt void isr_controller(void);

static void init_interruptions(void);
static void term_interruptions(void);

static void turn_on(uint16_t dummy);
static void turn_off(uint16_t dummy);

static void open_relay(uint16_t id);
static void close_relay(uint16_t id);

static void reset_interlocks(uint16_t dummy);
static void check_interlocks_ps_module(uint16_t id);

static inline void set_pwm_duty_hbridge_inline(volatile struct EPWM_REGS
                                               *p_pwm_module, float duty_pu);

/**
 * Main function for this power supply module
 */
void main_fbp_2s_ufjf(void)
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
    reset_controller();
    term_peripherals_drivers();
}

static void init_peripherals_drivers(void)
{
    uint16_t i;

    /// Initialization of HRADC boards
    stop_DMA();

    HRADCs_Info.enable_Sampling = 0;
    HRADCs_Info.n_HRADC_boards = NUM_HRADC_BOARDS;

    Init_DMA_McBSP_nBuffers(g_ipc_mtoc.num_ps_modules, DECIMATION_FACTOR,
                            HRADC_SPI_CLK);

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

    Config_HRADC_SoC(HRADC_FREQ_SAMP);

    /// Initialization of PWM modules
    g_pwm_modules.num_modules = 8;

    PWM_MOD1_A = &EPwm7Regs;        // PS-1 Positive polarity switches
    PWM_MOD1_B = &EPwm8Regs;        // PS-1 Negative polarity switches

    PWM_MOD2_A = &EPwm5Regs;        // PS-2 Positive polarity switches
    PWM_MOD2_B = &EPwm6Regs;        // PS-2 Negative polarity switches

    PWM_MOD3_A = &EPwm3Regs;        // PS-3 Positive polarity switches
    PWM_MOD3_B = &EPwm4Regs;        // PS-3 Negative polarity switches

    PWM_MOD4_A = &EPwm1Regs;        // PS-4 Positive polarity switches
    PWM_MOD4_B = &EPwm2Regs;        // PS-4 Negative polarity switches

    disable_pwm_outputs();
    disable_pwm_tbclk();
    init_pwm_mep_sfo();

    /// PS-4 PWM initialization
    init_pwm_module(PWM_MOD4_A, PWM_FREQ, 0, PWM_Sync_Master, 0,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);
    init_pwm_module(PWM_MOD4_B, PWM_FREQ, 1, PWM_Sync_Slave, 180,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);

    /// PS-3 PWM initialization
    init_pwm_module(PWM_MOD3_A, PWM_FREQ, 0, PWM_Sync_Slave, 0,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);
    init_pwm_module(PWM_MOD3_B, PWM_FREQ, 3, PWM_Sync_Slave, 180,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);

    /// PS-2 PWM initialization
    init_pwm_module(PWM_MOD2_A, PWM_FREQ, 0, PWM_Sync_Slave, 0,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);
    init_pwm_module(PWM_MOD2_B, PWM_FREQ, 5, PWM_Sync_Slave, 180,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);

    /// PS-1 PWM initialization
    init_pwm_module(PWM_MOD1_A, PWM_FREQ, 0, PWM_Sync_Slave, 0,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);
    init_pwm_module(PWM_MOD1_B, PWM_FREQ, 7, PWM_Sync_Slave, 180,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);

    //InitEPwm1Gpio();
    //InitEPwm2Gpio();
    InitEPwm3Gpio();
    InitEPwm4Gpio();
    //InitEPwm5Gpio();
    //InitEPwm6Gpio();
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
    static uint16_t i;

    for(i = 0; i < NUM_MAX_PS_MODULES; i++)
    {
        init_ps_module(&g_ipc_ctom.ps_module[i],
                       g_ipc_mtoc.ps_module[i].ps_status.bit.model,
                       &turn_on, &turn_off, &isr_soft_interlock,
                       &isr_hard_interlock, &reset_interlocks);

        init_event_manager(i, ISR_FREQ_INTERLOCK_TIMEBASE,
                           NUM_HARD_INTERLOCKS, NUM_SOFT_INTERLOCKS,
                           &HARD_INTERLOCKS_DEBOUNCE_TIME,
                           &HARD_INTERLOCKS_RESET_TIME,
                           &SOFT_INTERLOCKS_DEBOUNCE_TIME,
                           &SOFT_INTERLOCKS_RESET_TIME);

        if(!g_ipc_mtoc.ps_module[i].ps_status.bit.active)
        {
            g_ipc_ctom.ps_module[i].ps_status.bit.active = 0;
        }
    }

    init_ipc();
    init_control_framework(&g_controller_ctom);

    /// Initialization of signal generator module
    disable_siggen(&SIGGEN);
    init_siggen(&SIGGEN, ISR_CONTROL_FREQ, &I_LOAD_REFERENCE);
    cfg_siggen(&SIGGEN, g_ipc_mtoc.siggen.type, g_ipc_mtoc.siggen.num_cycles,
               g_ipc_mtoc.siggen.freq, g_ipc_mtoc.siggen.amplitude,
               g_ipc_mtoc.siggen.offset, g_ipc_mtoc.siggen.aux_param);

    /**
     * TODO: initialize WfmRef and Samples Buffer
     */

    /// INITIALIZATION OF LOAD CURRENT CONTROL LOOP

    /**
     *        name:     ERROR_I_LOAD
     * description:     Load current reference error
     *  dsp module:     DSP_Error
     *           +:     I_LOAD_REFERENCE
     *           -:     I_LOAD_K
     *         out:     I_LOAD_ERROR
     */

    init_dsp_error(ERROR_I_LOAD, &I_LOAD_REFERENCE, &I_LOAD_K, &I_LOAD_ERROR);

    /**
     *        name:     STATE_OBSERVER
     * description:     State observer
     *  dsp module:     DSP_Vect_Product
     *          in:     STATE_OBSERVER_IN  >> net_signals[1..7]
     *         out:     STATE_OBSERVER_OUT >> net_signals[8..9]
     */
    init_dsp_vect_product(STATE_OBSERVER, NUM_ROWS_OBSERVER,
                          NUM_COLUMNS_OBSERVER, observer, STATE_OBSERVER_IN,
                          STATE_OBSERVER_OUT);

    /**
     *        name:     STATE_CONTROLLER
     * description:     State controller
     *  dsp module:     DSP_Vect_Product
     *          in:     STATE_CONTROLLER_IN  >> net_signals[6..10]
     *         out:     STATE_CONTROLLER_OUT >> output_signals[0]
     */
    init_dsp_vect_product(STATE_CONTROLLER, NUM_ROWS_CONTROLLER,
                          NUM_COLUMNS_CONTROLLER, controller, STATE_CONTROLLER_IN,
                          STATE_CONTROLLER_OUT);

    /// INITIALIZATION OF TIME SLICERS

    /// 0: Time-slicer for WfmRef sweep decimation

    cfg_timeslicer(TIMESLICER_WFMREF, WFMREF_DECIMATION);

    /// 1: Time-slicer for SamplesBuffer
    cfg_timeslicer(TIMESLICER_BUFFER, BUFFER_DECIMATION);

    /**
     * Samples buffer initialization
     */
    init_buffer(BUF_SAMPLES, &g_buf_samples_ctom, SIZE_BUF_SAMPLES_CTOM);
    enable_buffer(BUF_SAMPLES);

    /// Reset all internal variables
    reset_controller();
}

/**
 * Reset all internal variables from controller
 */
static void reset_controller(void)
{
    uint16_t i;

    set_pwm_duty_hbridge(PWM_MOD1_A, 0.0);
    set_pwm_duty_hbridge(PWM_MOD2_A, 0.0);
    set_pwm_duty_hbridge(PWM_MOD3_A, 0.0);
    set_pwm_duty_hbridge(PWM_MOD4_A, 0.0);

    I_LOAD_SETPOINT = 0.0;
    I_LOAD_REFERENCE = 0.0;

    reset_dsp_error(ERROR_I_LOAD);

    /// Reset all state variables: net_signals[1..13]
    for(i = 1; i <= SIZE_STATE_VECTOR; i++)
    {
        g_controller_ctom.net_signals[i].f = 0.0;
    }

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

    reset_controller();
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

    I_LOAD_K = temp[0];
    V_C_MOD1_K = MOD1_LOAD_VOLTAGE;
    V_C_MOD3_K = MOD3_LOAD_VOLTAGE;

    //MOD2_LOAD_CURRENT = temp[1];
    //MOD3_LOAD_CURRENT = temp[2];
    //MOD4_LOAD_CURRENT = temp[3];

    /// Check whether power supply is ON
    if(g_ipc_ctom.ps_module[0].ps_status.bit.state > Interlock)
    {
        /// Calculate reference according to operation mode
        switch(g_ipc_ctom.ps_module[0].ps_status.bit.state)
        {
            case SlowRef:
            case SlowRefSync:
            {
                I_LOAD_REFERENCE = I_LOAD_SETPOINT;
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
                                I_LOAD_REFERENCE =
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
                            I_LOAD_REFERENCE = *(WFMREF.wfmref_data.p_buf_idx) *
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
            case Cycle:
            {
                SIGGEN.amplitude = g_ipc_mtoc.siggen.amplitude;
                SIGGEN.offset = g_ipc_mtoc.siggen.offset;
                SIGGEN.p_run_siggen(&SIGGEN);
                break;
            }
            default:
            {
                break;
            }
        }

        /**
         * TODO: Include dead-time compensation
         */

        /// Calculate state observer
        run_dsp_vect_product(STATE_OBSERVER);

        /// Open-loop
        if(g_ipc_ctom.ps_module[0].ps_status.bit.openloop)
        {
            SATURATE(I_LOAD_REFERENCE, MAX_REF_OL, MIN_REF_OL);
            REF_K = I_LOAD_REFERENCE;

            DUTY_CYCLE_MOD_1 = 0.01 * I_LOAD_REFERENCE;

            SATURATE(DUTY_CYCLE_MOD_1, PWM_MAX_DUTY_OL, PWM_MIN_DUTY_OL);
            DUTY_CYCLE_MOD_3 = DUTY_CYCLE_MOD_1;
        }

        /// Closed-loop
        else
        {

            SATURATE(I_LOAD_REFERENCE, MAX_REF, MIN_REF);
            REF_K = I_LOAD_REFERENCE;

            run_dsp_error(ERROR_I_LOAD);

            /// Integrator
            Q_K = Q_K_1 + I_LOAD_ERROR;

            /// Calculate state controller
            run_dsp_vect_product(STATE_CONTROLLER);

            //SATURATE(DUTY_CYCLE_MOD_1, PWM_MAX_DUTY, PWM_MIN_DUTY);
            if(DUTY_CYCLE_MOD_1 > PWM_MAX_DUTY)
            {
                DUTY_CYCLE_MOD_1 = PWM_MAX_DUTY;
                Q_K = Q_K_1;
            }
            else if(DUTY_CYCLE_MOD_1 < PWM_MIN_DUTY)
            {
                DUTY_CYCLE_MOD_1 = PWM_MIN_DUTY;
                Q_K = Q_K_1;
            }

            if(DUTY_CYCLE_MOD_3 > PWM_MAX_DUTY)
            {
                DUTY_CYCLE_MOD_3 = PWM_MAX_DUTY;
                Q_K = Q_K_1;
            }
            else if(DUTY_CYCLE_MOD_3 < PWM_MIN_DUTY)
            {
                DUTY_CYCLE_MOD_3 = PWM_MIN_DUTY;
                Q_K = Q_K_1;
            }
        }

        //set_pwm_duty_hbridge(PWM_MOD1_A, DUTY_CYCLE_MOD_1);
        //set_pwm_duty_hbridge(PWM_MOD3_A, DUTY_CYCLE_MOD_3);
        set_pwm_duty_hbridge_inline(PWM_MOD1_A, DUTY_CYCLE_MOD_1);
        set_pwm_duty_hbridge_inline(PWM_MOD3_A, DUTY_CYCLE_MOD_3);

        /// Save current samples as past samples
        I_LOAD_K_1   = I_LOAD_K;
        V_C_MOD1_K_1 = V_C_MOD1_K;
        V_C_MOD3_K_1 = V_C_MOD3_K;
        I_L_MOD1_K_1 = I_L_MOD1_K;
        I_L_MOD3_K_1 = I_L_MOD3_K;
        V_D_MOD1_K_1 = V_D_MOD1_K;
        V_D_MOD3_K_1 = V_D_MOD3_K;
        M_MOD1_K_1   = DUTY_CYCLE_MOD_1;
        M_MOD3_K_1   = DUTY_CYCLE_MOD_3;
        Q_K_1        = Q_K;
    }

    PWM_MOD1_A->ETCLR.bit.INT = 1;
    PWM_MOD1_B->ETCLR.bit.INT = 1;
    PWM_MOD2_A->ETCLR.bit.INT = 1;
    PWM_MOD2_B->ETCLR.bit.INT = 1;
    PWM_MOD3_A->ETCLR.bit.INT = 1;
    PWM_MOD3_B->ETCLR.bit.INT = 1;
    PWM_MOD4_A->ETCLR.bit.INT = 1;
    PWM_MOD4_B->ETCLR.bit.INT = 1;

    /*********************************************/
    RUN_TIMESLICER(TIMESLICER_BUFFER)
    /*********************************************/
        insert_buffer(BUF_SAMPLES, NETSIGNAL_CTOM_BUF_1);
        insert_buffer(BUF_SAMPLES, NETSIGNAL_CTOM_BUF_2);
    /*********************************************/
    END_TIMESLICER(TIMESLICER_BUFFER)
    /*********************************************/

    PieCtrlRegs.PIEACK.all |= M_INT3;

    CLEAR_DEBUG_GPIO1;
}

/**
 * Initialization of interruptions.
 */
static void init_interruptions(void)
{
    EALLOW;
    PieVectTable.EPWM1_INT = &isr_init_controller;
    PWM_MOD4_A->ETPS.bit.INTPRD = ET_2ND;
    //PieVectTable.EPWM2_INT = &isr_controller;
    PieVectTable.TINT0     = &isr_interlocks_timebase;
    EDIS;

    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;  /// ePWM1
    //PieCtrlRegs.PIEIER3.bit.INTx2 = 1;  /// ePWM2
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;  /// CpuTimer0

    enable_pwm_interrupt(PWM_MOD4_A);
    //enable_pwm_interrupt(PWM_MOD4_B);

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
    //PieCtrlRegs.PIEIER3.bit.INTx2 = 0;  /// ePWM2

    disable_pwm_interrupt(PWM_MOD4_A);
    //disable_pwm_interrupt(PWM_MOD4_B);

    /// Clear flags
    PieCtrlRegs.PIEACK.all |= M_INT1 | M_INT3 | M_INT11;
}

/**
 * Turn power supply on.
 *
 * TODO: Fix DC-Link undervoltage check and turn_on
 * @param dummy dummy argument due to ps_module pointer
 */
static void turn_on(uint16_t dummy)
{
    if(g_ipc_ctom.ps_module[0].ps_status.bit.active)
    {
        #ifdef USE_ITLK
        if(g_ipc_ctom.ps_module[0].ps_status.bit.state == Off)
        #else
        if(g_ipc_ctom.ps_module[0].ps_status.bit.state <= Interlock)
        #endif
        {
            if(fabs(V_C_MOD1_K) < MIN_DCLINK)
            {
                BYPASS_HARD_INTERLOCK_DEBOUNCE(0, DCLink_Undervoltage);
                set_hard_interlock(0, DCLink_Undervoltage);
            }

            if(fabs(V_C_MOD3_K) < MIN_DCLINK)
            {
                BYPASS_HARD_INTERLOCK_DEBOUNCE(0, DCLink_Undervoltage);
                set_hard_interlock(0, DCLink_Undervoltage);
            }

            #ifdef USE_ITLK
            if(g_ipc_ctom.ps_module[0].ps_status.bit.state == Off)
            #else
            if(g_ipc_ctom.ps_module[0].ps_status.bit.state <= Interlock)
            #endif
            {
                reset_controller();
                close_relay(0);     /// Module 1
                close_relay(1);     /// Module 3

                g_ipc_ctom.ps_module[0].ps_status.bit.openloop = OPEN_LOOP;
                g_ipc_ctom.ps_module[0].ps_status.bit.state = SlowRef;

                /// Module 1
                enable_pwm_output(0);
                enable_pwm_output(1);

                /// Module 3
                enable_pwm_output(4);
                enable_pwm_output(5);
            }
        }
    }
}

/**
 * Turn off specified power supply.
 *
 * @param dummy dummy argument due to ps_module pointer
 */
static void turn_off(uint16_t dummy)
{
    if(g_ipc_ctom.ps_module[0].ps_status.bit.active)
    {
        /// Module 1
        disable_pwm_output(0);
        disable_pwm_output(1);

        /// Module 3
        disable_pwm_output(4);
        disable_pwm_output(5);

        open_relay(0);      /// Module 1
        open_relay(1);      /// Module 3

        g_ipc_ctom.ps_module[0].ps_status.bit.openloop = OPEN_LOOP;
        if (g_ipc_ctom.ps_module[0].ps_status.bit.state != Interlock)
        {
            g_ipc_ctom.ps_module[0].ps_status.bit.state = Off;
        }
        reset_controller();
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
 * Open relay from specified power supply.
 *
 * @param id specified power supply
 */
static void open_relay(uint16_t id)
{
    switch(id)
    {
        case MOD1_ID:
        {
            PIN_OPEN_MOD1_DCLINK_RELAY;
            break;
        }

        case MOD2_ID:
        {
            PIN_OPEN_MOD2_DCLINK_RELAY;
            break;
        }

        case MOD3_ID:
        {
            PIN_OPEN_MOD3_DCLINK_RELAY;
            break;
        }

        case MOD4_ID:
        {
            PIN_OPEN_MOD4_DCLINK_RELAY;
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
        case MOD1_ID:
        {
            PIN_CLOSE_MOD1_DCLINK_RELAY;
            break;
        }

        case MOD2_ID:
        {
            PIN_CLOSE_MOD2_DCLINK_RELAY;
            break;
        }

        case MOD3_ID:
        {
            PIN_CLOSE_MOD3_DCLINK_RELAY;
            break;
        }

        case MOD4_ID:
        {
            PIN_CLOSE_MOD4_DCLINK_RELAY;
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
    if(fabs(I_LOAD_K) > MAX_ILOAD)
    {
        set_hard_interlock(0, Load_Overcurrent);
    }

    if(fabs(V_C_MOD1_K) > MAX_VLOAD)
    {
        set_hard_interlock(0, Load_Overvoltage);
    }

    if(fabs(V_C_MOD3_K) > MAX_VLOAD)
    {
        set_hard_interlock(0, Load_Overvoltage);
    }

    switch(id)
    {
        case 0:
        {
            if(fabs(MOD1_DCLINK_VOLTAGE) > MAX_DCLINK)
            {
                set_hard_interlock(0, DCLink_Overvoltage);
            }

            if(fabs(MOD1_TEMPERATURE) > MAX_TEMP)
            {
                set_soft_interlock(0, Heatsink_Overtemperature);
            }

            if(!PIN_STATUS_MOD1_DRIVER_ERROR)
            {
                //set_hard_interlock(0, MOSFETs_Driver_Fault);
            }

            IER &= ~M_INT11;

            if ( (g_ipc_ctom.ps_module[0].ps_status.bit.state <= Interlock) &&
                 (PIN_STATUS_MOD1_DCLINK_RELAY) )
            {
                //set_hard_interlock(0, DCLink_Relay_Fault);
            }

            else if (g_ipc_ctom.ps_module[0].ps_status.bit.state > Interlock)
            {
                if(!PIN_STATUS_MOD1_DCLINK_RELAY)
                {
                    //set_hard_interlock(0, DCLink_Relay_Fault);
                }

                if(!PIN_STATUS_MOD1_FUSE)
                {
                    //set_hard_interlock(0, DCLink_Fuse_Fault);
                }

                if(fabs(g_controller_mtoc.net_signals[0].f) < MIN_DCLINK)
                {
                    set_hard_interlock(0, DCLink_Undervoltage);
                }
            }

            break;
        }

        case 1:
        {
            if(fabs(MOD2_DCLINK_VOLTAGE) > MAX_DCLINK)
            {
                set_hard_interlock(1, DCLink_Overvoltage);
            }

            if(fabs(MOD2_TEMPERATURE) > MAX_TEMP)
            {
                set_soft_interlock(1, Heatsink_Overtemperature);
            }

            if(!PIN_STATUS_MOD2_DRIVER_ERROR)
            {
                //set_hard_interlock(1, MOSFETs_Driver_Fault);
            }

            IER &= ~M_INT11;

            if ( (g_ipc_ctom.ps_module[1].ps_status.bit.state <= Interlock) &&
                (PIN_STATUS_MOD2_DCLINK_RELAY))
            {
                //set_hard_interlock(1, DCLink_Relay_Fault);
            }

            else if (g_ipc_ctom.ps_module[1].ps_status.bit.state > Interlock)
            {
                if(!PIN_STATUS_MOD2_DCLINK_RELAY)
                {
                    //set_hard_interlock(1, DCLink_Relay_Fault);
                }

                if(!PIN_STATUS_MOD2_FUSE)
                {
                    //set_hard_interlock(1, DCLink_Fuse_Fault);
                }

                if(fabs(g_controller_mtoc.net_signals[1].f) < MIN_DCLINK)
                {
                    set_hard_interlock(1, DCLink_Undervoltage);
                }
            }

            break;
        }

        case 2:
        {
            if(fabs(MOD3_DCLINK_VOLTAGE) > MAX_DCLINK)
            {
                set_hard_interlock(2, DCLink_Overvoltage);
            }

            if(fabs(MOD3_TEMPERATURE) > MAX_TEMP)
            {
                set_soft_interlock(2, Heatsink_Overtemperature);
            }

            if(!PIN_STATUS_MOD3_DRIVER_ERROR)
            {
                //set_hard_interlock(2, MOSFETs_Driver_Fault);
            }

            IER &= ~M_INT11;

            if ( (g_ipc_ctom.ps_module[2].ps_status.bit.state <= Interlock) &&
                (PIN_STATUS_MOD3_DCLINK_RELAY))
            {
                //set_hard_interlock(2, DCLink_Relay_Fault);
            }

            else if (g_ipc_ctom.ps_module[2].ps_status.bit.state > Interlock)
            {
                if(!PIN_STATUS_MOD3_DCLINK_RELAY)
                {
                    // set_hard_interlock(2, DCLink_Relay_Fault);
                }

                if(!PIN_STATUS_MOD3_FUSE)
                {
                    //set_hard_interlock(2, DCLink_Fuse_Fault);
                }

                if(fabs(g_controller_mtoc.net_signals[2].f) < MIN_DCLINK)
                {
                    set_hard_interlock(2, DCLink_Undervoltage);
                }

            }

            break;
        }

        case 3:
        {
            if(fabs(MOD4_DCLINK_VOLTAGE) > MAX_DCLINK)
            {
                set_hard_interlock(3, DCLink_Overvoltage);
            }

            if(fabs(MOD4_TEMPERATURE) > MAX_TEMP)
            {
                set_soft_interlock(3, Heatsink_Overtemperature);
            }

            if(!PIN_STATUS_MOD4_DRIVER_ERROR)
            {
                //set_hard_interlock(3, MOSFETs_Driver_Fault);
            }

            IER &= ~M_INT11;

            if ( (g_ipc_ctom.ps_module[3].ps_status.bit.state <= Interlock) &&
                (PIN_STATUS_MOD4_DCLINK_RELAY))
            {
                //set_hard_interlock(3, DCLink_Relay_Fault);
            }

            else if (g_ipc_ctom.ps_module[3].ps_status.bit.state > Interlock)
            {
                if(!PIN_STATUS_MOD4_DCLINK_RELAY)
                {
                    //set_hard_interlock(3, DCLink_Relay_Fault);
                }

                if(!PIN_STATUS_MOD4_FUSE)
                {
                    //set_hard_interlock(3, DCLink_Fuse_Fault);
                }

                if(fabs(g_controller_mtoc.net_signals[3].f) < MIN_DCLINK)
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
