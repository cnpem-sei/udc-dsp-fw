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

#include "fbp.h"
#include "boards/udc_c28.h"
#include "control/control.h"
#include "ipc/ipc.h"
#include "common/timeslicer.h"
#include "HRADC_board/HRADC_Boards.h"


/**
 * Configuration parameters
 *
 * TODO: transfer this to param bank
 */
#define PWM_FREQ                50000.0     // PWM frequency [Hz]
#define PWM_DEAD_TIME           300         // PWM dead-time [ns]
#define PWM_MAX_DUTY            0.9         // Max duty cycle [p.u.]
#define PWM_MIN_DUTY            -0.9        // Min duty cycle [p.u.]
#define PWM_MAX_DUTY_OL         0.9         // Max open loop duty cycle [p.u.]
#define PWM_MIN_DUTY_OL         -0.9        // Min open loop duty cycle [p.u.]

#define MAX_REF                 10.0        // Reference over-saturation level [A]
#define MIN_REF                 -10.0       // Reference under-saturation level [A]
#define MAX_ILOAD               10.5        // Reference limit for interlock [A]
#define MAX_VLOAD               10.5        // Load voltage limit for interlock [V]
#define MIN_DCLINK              3.0         // DC Link under limit for interlock [V]
#define MAX_DCLINK              17.0        // DC Link over limit for interlock [V]
#define MAX_TEMP                80.0        // Temperature limit for interlock [ºC]

#define MAX_REF_SLEWRATE        1000000.0   // Max reference slew-rate [A/s]
#define MAX_SR_SIGGEN_OFFSET    50.0        // Max SigGen offset slew-rate [A/s]
#define MAX_SR_SIGGEN_AMP       100.0       // Max SigGen amplitude slew-rate [A/s]

#define KP                      1.9
#define KI                      559.0

#define CONTROL_FREQ            (2.0*PWM_FREQ)
#define CONTROL_PERIOD          (1.0/CONTROL_FREQ)
#define DECIMATION_FACTOR       1
#define TRANSFER_BUFFER_SIZE    DECIMATION_FACTOR
#define HRADC_FREQ_SAMP         (float) CONTROL_FREQ*DECIMATION_FACTOR
#define HRADC_SPI_CLK           SPI_15MHz

#define BUFFER_DECIMATION       1
#define WFMREF_SAMPLING_FREQ    4096

#define TRANSDUCER_INPUT_RATED      12.5            // ** DCCT LEM ITN 12-P **
#define TRANSDUCER_OUTPUT_RATED     0.05            // In_rated   = +/- 12.5 A
#define TRANSDUCER_OUTPUT_TYPE      Iin_bipolar     // Out_rated  = +/- 50 mA
#define HRADC_R_BURDEN              20.0            // Burden resistor = 20 R
#if (HRADC_v2_0)
    #define TRANSDUCER_GAIN         -(TRANSDUCER_INPUT_RATED/TRANSDUCER_OUTPUT_RATED)
#endif
#if (HRADC_v2_1)
    #define TRANSDUCER_GAIN         (TRANSDUCER_INPUT_RATED/TRANSDUCER_OUTPUT_RATED)
#endif

/**
 * All power supplies defines
 *
 * TODO: use new control modules definitions
 */
#define PS_ALL_ID   0x000F

#define LOAD_OVERCURRENT            0x00000001
#define LOAD_OVERVOLTAGE            0x00000002
#define DCLINK_OVERVOLTAGE          0x00000004
#define DCLINK_UNDERVOLTAGE         0x00000008
#define DCLINK_RELAY_FAIL           0x00000010
#define FUSE_FAIL                   0x00000020
#define DRIVER_FAIL                 0x00000040
#define OVERTEMP                    0x00000080

/**
 * Power supply 1 defines
 */

#define PS1_ID                          0x0001

#define PIN_OPEN_PS1_DCLINK_RELAY       CLEAR_GPDO4;
#define PIN_CLOSE_PS1_DCLINK_RELAY      SET_GPDO4;

#define PIN_STATUS_PS1_DCLINK_RELAY     GET_GPDI4
#define PIN_STATUS_PS1_DRIVER_ERROR     GET_GPDI5
#define PIN_STATUS_PS1_FUSE             GET_GPDI14

#define PS1_LOAD_CURRENT                g_controller_ctom.net_signals[0]    // HRADC0
#define PS1_LOAD_VOLTAGE                g_controller_mtoc.net_signals[9]    // ANI6
#define PS1_DCLINK_VOLTAGE              g_controller_mtoc.net_signals[5]    // ANI2
#define PS1_TEMPERATURE                 g_controller_mtoc.net_signals[13]   // I2C Add 0x48

#define PS1_SETPOINT                    g_ipc_ctom.ps_module[0].ps_setpoint
#define PS1_REFERENCE                   g_ipc_ctom.ps_module[0].ps_reference

#define ERROR_CALCULATOR_PS1            &g_controller_ctom.dsp_modules.dsp_error[0]
#define PI_DAWU_CONTROLLER_ILOAD_PS1    &g_controller_ctom.dsp_modules.dsp_pi[0]

#define PS1_PWM_MODULATOR               g_pwm_modules.pwm_regs[6]
#define PS1_PWM_MODULATOR_NEG           g_pwm_modules.pwm_regs[7]

/**
 * Power supply 2 defines
 */

#define PS2_ID                          0x0002

#define PIN_OPEN_PS2_DCLINK_RELAY       CLEAR_GPDO3;
#define PIN_CLOSE_PS2_DCLINK_RELAY      SET_GPDO3;

#define PIN_STATUS_PS2_DCLINK_RELAY     GET_GPDI11
#define PIN_STATUS_PS2_DRIVER_ERROR     GET_GPDI9
#define PIN_STATUS_PS2_FUSE             GET_GPDI15

#define PS2_LOAD_CURRENT                g_controller_ctom.net_signals[2]    // HRADC1
#define PS2_LOAD_VOLTAGE                g_controller_mtoc.net_signals[10]   // ANI7
#define PS2_DCLINK_VOLTAGE              g_controller_mtoc.net_signals[6]    // ANI1
#define PS2_TEMPERATURE                 g_controller_mtoc.net_signals[14]   // I2C Add 0x49

#define PS2_SETPOINT                    g_ipc_ctom.ps_module[1].ps_setpoint
#define PS2_REFERENCE                   g_ipc_ctom.ps_module[1].ps_reference

#define ERROR_CALCULATOR_PS2            &g_controller_ctom.dsp_modules.dsp_error[1]
#define PI_DAWU_CONTROLLER_ILOAD_PS2    &g_controller_ctom.dsp_modules.dsp_pi[1]

#define PS2_PWM_MODULATOR               g_pwm_modules.pwm_regs[4]
#define PS2_PWM_MODULATOR_NEG           g_pwm_modules.pwm_regs[5]

/**
 * Power supply 3 defines
 */

#define PS3_ID                          0x0004

#define PIN_OPEN_PS3_DCLINK_RELAY       CLEAR_GPDO1;
#define PIN_CLOSE_PS3_DCLINK_RELAY      SET_GPDO1;

#define PIN_STATUS_PS3_DCLINK_RELAY     GET_GPDI8
#define PIN_STATUS_PS3_DRIVER_ERROR     GET_GPDI1
#define PIN_STATUS_PS3_FUSE             GET_GPDI13

#define PS3_LOAD_CURRENT                g_controller_ctom.net_signals[4]    // HRADC2
#define PS3_LOAD_VOLTAGE                g_controller_mtoc.net_signals[11]   // ANI3
#define PS3_DCLINK_VOLTAGE              g_controller_mtoc.net_signals[7]    // ANI4
#define PS3_TEMPERATURE                 g_controller_mtoc.net_signals[15]   // I2C Add 0x4A

#define PS3_SETPOINT                    g_ipc_ctom.ps_module[2].ps_setpoint
#define PS3_REFERENCE                   g_ipc_ctom.ps_module[2].ps_reference

#define ERROR_CALCULATOR_PS3            &g_controller_ctom.dsp_modules.dsp_error[2]
#define PI_DAWU_CONTROLLER_ILOAD_PS3    &g_controller_ctom.dsp_modules.dsp_pi[2]

#define PS3_PWM_MODULATOR               g_pwm_modules.pwm_regs[2]
#define PS3_PWM_MODULATOR_NEG           g_pwm_modules.pwm_regs[3]

/**
 * Power supply 4 defines
 */

#define PS4_ID                          0x0008

#define PIN_OPEN_PS4_DCLINK_RELAY       CLEAR_GPDO2;
#define PIN_CLOSE_PS4_DCLINK_RELAY      SET_GPDO2;

#define PIN_STATUS_PS4_DCLINK_RELAY     GET_GPDI2
#define PIN_STATUS_PS4_DRIVER_ERROR     GET_GPDI3
#define PIN_STATUS_PS4_FUSE             GET_GPDI16

#define PS4_LOAD_CURRENT                g_controller_ctom.net_signals[5]   // HRADC3
#define PS4_LOAD_VOLTAGE                g_controller_mtoc.net_signals[12]   // ANI5
#define PS4_DCLINK_VOLTAGE              g_controller_mtoc.net_signals[8]    // ANI0
#define PS4_TEMPERATURE                 g_controller_mtoc.net_signals[16]   // I2C Add 0x4C

#define PS4_SETPOINT                    g_ipc_ctom.ps_module[3].ps_setpoint
#define PS4_REFERENCE                   g_ipc_ctom.ps_module[3].ps_reference

#define ERROR_CALCULATOR_PS4            &g_controller_ctom.dsp_modules.dsp_error[3]
#define PI_DAWU_CONTROLLER_ILOAD_PS4    &g_controller_ctom.dsp_modules.dsp_pi[3]

#define PS4_PWM_MODULATOR               g_pwm_modules.pwm_regs[0]
#define PS4_PWM_MODULATOR_NEG           g_pwm_modules.pwm_regs[1]

/**
 * TODO: Put here your constants and variables. Always use static for 
 * private members.
 */

static uint16_t num_ps = 0;

#pragma CODE_SECTION(isr_init_controller, "ramfuncs");
#pragma CODE_SECTION(isr_controller, "ramfuncs");
#pragma CODE_SECTION(turn_on, "ramfuncs");
#pragma CODE_SECTION(turn_off, "ramfuncs");
#pragma CODE_SECTION(set_hard_interlock, "ramfuncs");
#pragma CODE_SECTION(set_soft_interlock, "ramfuncs");
#pragma CODE_SECTION(isr_hard_interlock, "ramfuncs");
#pragma CODE_SECTION(isr_soft_interlock, "ramfuncs");

static void init_peripherals_drivers(void);
static void term_peripherals_drivers(void);

static void init_controller(void);
static void reset_controller(uint16_t id);
static void reset_controllers(void);
static void enable_controller();
static void disable_controller();
interrupt void isr_init_controller(void);
interrupt void isr_controller(void);

static void init_interruptions(void);
static void term_interruptions(void);

static void turn_on(void);
static void turn_off(void);

static void reset_interlock(void);
static void set_hard_interlock(uint32_t itlk);
static void set_soft_interlock(uint32_t itlk);
interrupt void isr_hard_interlock(void);
interrupt void isr_soft_interlock(void);

/**
 * Main function for this power supply module
 */
void main_fbp(void)
{
    num_ps  = 0;

    init_controller();
    init_peripherals_drivers();
    init_interruptions();
    enable_controller();

    /// TODO: include condition for re-initialization
    while(1)
    {

    }

    turn_off();
    disable_controller();
    term_interruptions();
    reset_controllers();
    term_peripherals_drivers();
}

static void init_peripherals_drivers(void)
{
    uint16_t i;

    /* Initialization of HRADC boards */

    stop_DMA();

    Init_DMA_McBSP_nBuffers(num_ps, DECIMATION_FACTOR, HRADC_SPI_CLK);

    Init_SPIMaster_McBSP(HRADC_SPI_CLK);
    Init_SPIMaster_Gpio();
    InitMcbspa20bit();

    for(i = 0; i < num_ps; i++)
    {
        Init_HRADC_Info(&HRADCs_Info.HRADC_boards[i], i, DECIMATION_FACTOR,
                        buffers_HRADC[i], TRANSDUCER_GAIN);
        Config_HRADC_board(&HRADCs_Info.HRADC_boards[i], Iin_bipolar,
                           HEATER_DISABLE, RAILS_DISABLE);
    }

    Config_HRADC_SoC(HRADC_FREQ_SAMP);

    /* Initialization of PWM modules */
    g_pwm_modules.num_modules = 8;
    PS4_PWM_MODULATOR       = &EPwm1Regs;   // PS-4 Positive polarity switches
    PS4_PWM_MODULATOR_NEG   = &EPwm2Regs;   // PS-4 Negative polarity switches
    PS3_PWM_MODULATOR       = &EPwm3Regs;   // PS-3 Positive polarity switches
    PS3_PWM_MODULATOR_NEG   = &EPwm4Regs;   // PS-3 Negative polarity switches
    PS2_PWM_MODULATOR       = &EPwm5Regs;   // PS-2 Positive polarity switches
    PS2_PWM_MODULATOR_NEG   = &EPwm6Regs;   // PS-2 Negative polarity switches
    PS1_PWM_MODULATOR       = &EPwm7Regs;   // PS-1 Positive polarity switches
    PS1_PWM_MODULATOR_NEG   = &EPwm8Regs;   // PS-1 Negative polarity switches

    disable_pwm_outputs();
    disable_pwm_tbclk();
    init_pwm_mep_sfo();

    // PS-4 PWM initialization
    init_pwm_module(PS4_PWM_MODULATOR, PWM_FREQ, 0, PWM_Sync_Master, 0,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);
    init_pwm_module(PS4_PWM_MODULATOR_NEG, PWM_FREQ, 1, PWM_Sync_Slave, 180,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);

    // PS-3 PWM initialization
    init_pwm_module(PS3_PWM_MODULATOR, PWM_FREQ, 0, PWM_Sync_Slave, 0,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);
    init_pwm_module(PS3_PWM_MODULATOR_NEG, PWM_FREQ, 3, PWM_Sync_Slave, 180,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);

    // PS-2 PWM initialization
    init_pwm_module(PS2_PWM_MODULATOR, PWM_FREQ, 0, PWM_Sync_Slave, 0,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);
    init_pwm_module(PS2_PWM_MODULATOR_NEG, PWM_FREQ, 5, PWM_Sync_Slave, 180,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);

    // PS-1 PWM initialization
    init_pwm_module(PS1_PWM_MODULATOR, PWM_FREQ, 0, PWM_Sync_Slave, 0,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);
    init_pwm_module(PS1_PWM_MODULATOR_NEG, PWM_FREQ, 7, PWM_Sync_Slave, 180,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);

    /* Initialization of timers */
    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, C28_FREQ_MHZ, 1000000);
    CpuTimer0Regs.TCR.bit.TIE = 0;
}

static void term_peripherals_drivers(void)
{

}

static void init_controller(void)
{
    while(g_ipc_mtoc.ps_module[num_ps].ps_status.bit.active)
    {
        init_ps_module(&g_ipc_ctom.ps_module[num_ps],
                       g_ipc_mtoc.ps_module[num_ps].ps_status.bit.model,
                       &turn_on, &turn_off, &isr_soft_interlock,
                       &isr_hard_interlock, &reset_interlock);

        /**
         * TODO: initialize SigGen, WfmRef and Samples Buffer
         */

        num_ps++;
    }

    init_ipc();
    init_control_framework(&g_controller_ctom);

    /******************************************************************/
    /* INITIALIZATION OF LOAD CURRENT CONTROL LOOP FOR POWER SUPPLY 1 */
    /******************************************************************/

    /*
     *        name:     ERROR_CALCULATOR_PS1
     * description:     Load current reference error
     *    DP class:     DSP_Error
     *          +:      ps_module[0].ps_reference
     *          -:      net_signals[0]
     *         out:     net_signals[1]
     */

    init_dsp_error(ERROR_CALCULATOR_PS1, &PS1_REFERENCE,
                   &g_controller_ctom.net_signals[0],
                   &g_controller_ctom.net_signals[1]);

    /*
     *        name:     PI_DAWU_CONTROLLER_ILOAD_PS1
     * description:     Load current PI controller
     *    DP class:     DSP_PI
     *          in:     net_signals[1]
     *         out:     output_signals[0]
     */

    init_dsp_pi(PI_DAWU_CONTROLLER_ILOAD_PS1, KP, KI, CONTROL_FREQ,
                PWM_MAX_DUTY, PWM_MIN_DUTY, &g_controller_ctom.net_signals[1],
                &g_controller_ctom.output_signals[0]);

    /******************************************************************/
    /* INITIALIZATION OF LOAD CURRENT CONTROL LOOP FOR POWER SUPPLY 2 */
    /******************************************************************/

    /*
     *        name:     ERROR_CALCULATOR_PS2
     * description:     Load current reference error
     *    DP class:     DSP_Error
     *          +:      ps_module[1].ps_reference
     *          -:      net_signals[2]
     *         out:     net_signals[3]
     */

    init_dsp_error(ERROR_CALCULATOR_PS2, &PS2_REFERENCE,
                   &g_controller_ctom.net_signals[2],
                   &g_controller_ctom.net_signals[3]);

    /*
     *        name:     PI_DAWU_CONTROLLER_ILOAD_PS2
     * description:     Load current PI controller
     *    DP class:     DSP_PI
     *          in:     net_signals[3]
     *         out:     output_signals[10]
     */

    init_dsp_pi(PI_DAWU_CONTROLLER_ILOAD_PS2, KP, KI, CONTROL_FREQ,
                PWM_MAX_DUTY, PWM_MIN_DUTY, &g_controller_ctom.net_signals[3],
                &g_controller_ctom.output_signals[1]);

    /******************************************************************/
    /* INITIALIZATION OF LOAD CURRENT CONTROL LOOP FOR POWER SUPPLY 3 */
    /******************************************************************/

    /*
     *        name:     ERROR_CALCULATOR_PS3
     * description:     Load current reference error
     *    DP class:     DSP_Error
     *          +:      ps_module[2].ps_reference
     *          -:      net_signals[4]
     *         out:     net_signals[5]
     */

    init_dsp_error(ERROR_CALCULATOR_PS3, &PS3_REFERENCE,
                   &g_controller_ctom.net_signals[4],
                   &g_controller_ctom.net_signals[5]);

    /*
     *        name:     PI_DAWU_CONTROLLER_ILOAD_PS3
     * description:     Load current PI controller
     *    DP class:     DSP_PI
     *          in:     net_signals[5]
     *         out:     output_signals[2]
     */

    init_dsp_pi(PI_DAWU_CONTROLLER_ILOAD_PS3, KP, KI, CONTROL_FREQ,
                PWM_MAX_DUTY, PWM_MIN_DUTY, &g_controller_ctom.net_signals[5],
                &g_controller_ctom.output_signals[2]);

    /******************************************************************/
    /* INITIALIZATION OF LOAD CURRENT CONTROL LOOP FOR POWER SUPPLY 4 */
    /******************************************************************/

    /*
     *        name:     ERROR_CALCULATOR_PS4
     * description:     Load current reference error
     *    DP class:     DSP_Error
     *          +:      ps_module[3].ps_reference
     *          -:      net_signals[6]
     *         out:     net_signals[7]
     */

    init_dsp_error(ERROR_CALCULATOR_PS4, &PS4_REFERENCE,
                   &g_controller_ctom.net_signals[6],
                   &g_controller_ctom.net_signals[7]);

    /*
     *        name:     PI_DAWU_CONTROLLER_ILOAD_PS4
     * description:     Load current PI controller
     *    DP class:     DSP_PI
     *          in:     net_signals[7]
     *         out:     output_signals[3]
     */
    init_dsp_pi(PI_DAWU_CONTROLLER_ILOAD_PS4, KP, KI, CONTROL_FREQ,
                PWM_MAX_DUTY, PWM_MIN_DUTY, &g_controller_ctom.net_signals[7],
                &g_controller_ctom.output_signals[3]);

    /**********************************/
    /* INITIALIZATION OF TIME SLICERS */
    /**********************************/

    // 0: Time-slicer for WfmRef sweep decimation
    cfg_timeslicer(0, CONTROL_FREQ/WFMREF_SAMPLING_FREQ);

    // 1: Time-slicer for SamplesBuffer
    cfg_timeslicer(1, BUFFER_DECIMATION);

    reset_controllers();
}

static void reset_controller(uint16_t id)
{
    switch(id)
    {
        case 0:
        {
            set_pwm_duty_hbridge(PS1_PWM_MODULATOR, 0.0);
            reset_dsp_error(ERROR_CALCULATOR_PS1);
            reset_dsp_pi(PI_DAWU_CONTROLLER_ILOAD_PS1);
            PS1_REFERENCE = 0.0;
            break;
        }

        case 1:
        {
            set_pwm_duty_hbridge(PS2_PWM_MODULATOR, 0.0);
            reset_dsp_error(ERROR_CALCULATOR_PS2);
            reset_dsp_pi(PI_DAWU_CONTROLLER_ILOAD_PS2);
            PS2_REFERENCE = 0.0;
            break;
        }

        case 2:
        {
            set_pwm_duty_hbridge(PS3_PWM_MODULATOR, 0.0);
            reset_dsp_error(ERROR_CALCULATOR_PS3);
            reset_dsp_pi(PI_DAWU_CONTROLLER_ILOAD_PS3);
            PS3_REFERENCE = 0.0;
            break;
        }

        case 3:
        {
            set_pwm_duty_hbridge(PS4_PWM_MODULATOR, 0.0);
            reset_dsp_error(ERROR_CALCULATOR_PS4);
            reset_dsp_pi(PI_DAWU_CONTROLLER_ILOAD_PS4);
            PS4_REFERENCE = 0.0;
            break;
        }

        default:
        {
            break;
        }
    }

    reset_timeslicers();
}

static void reset_controllers(void)
{
    uint16_t i;

    for(i = 0; i < num_ps; i++)
    {
        reset_controller(i);
    }
}

static void enable_controller()
{

}

static void disable_controller()
{

}

interrupt void isr_init_controller(void)
{

}

interrupt void isr_controller(void)
{

}

static void init_interruptions(void)
{

}

static void term_interruptions(void)
{

}

static void turn_on(void)
{

}

static void turn_off(void)
{

}

static void reset_interlock(void)
{

}

static void set_hard_interlock(uint32_t itlk)
{

}

static void set_soft_interlock(uint32_t itlk)
{

}

interrupt void isr_hard_interlock(void)
{

}

interrupt void isr_soft_interlock(void)
{

}
