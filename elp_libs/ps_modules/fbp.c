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
#include "ipc/ipc.h"

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

#define KP
#define KI

#define CONTROL_FREQ            (2.0*PWM_FREQ)
#define CONTROL_PERIOD          (1.0/CONTROL_FREQ)
#define DECIMATION_FACTOR       1
#define TRANSFER_BUFFER_SIZE    DECIMATION_FACTOR
#define HRADC_FREQ_SAMP         (float) CONTROL_FREQ*DECIMATION_FACTOR
#define HRADC_SPI_CLK           SPI_15MHz

#define BUFFER_DECIMATION       1

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

#define PS1_LOAD_CURRENT                DP_Framework.NetSignals[5]          // HRADC0
#define PS1_LOAD_VOLTAGE                DP_Framework_MtoC.NetSignals[9]     // ANI6
#define PS1_DCLINK_VOLTAGE              DP_Framework_MtoC.NetSignals[5]     // ANI2
#define PS1_TEMPERATURE                 DP_Framework_MtoC.NetSignals[13]    // I2C Add 0x48

#define ERROR_CALCULATOR_PS1            &DP_Framework.DPlibrary.ELP_Error[0]
#define PI_DAWU_CONTROLLER_ILOAD_PS1    &DP_Framework.DPlibrary.ELP_PI_dawu[0]

/**
 * Power supply 2 defines
 */

#define PS2_ID                          0x0002

#define PIN_OPEN_PS2_DCLINK_RELAY       CLEAR_GPDO3;
#define PIN_CLOSE_PS2_DCLINK_RELAY      SET_GPDO3;

#define PIN_STATUS_PS2_DCLINK_RELAY     GET_GPDI11
#define PIN_STATUS_PS2_DRIVER_ERROR     GET_GPDI9
#define PIN_STATUS_PS2_FUSE             GET_GPDI15

#define PS2_LOAD_CURRENT                DP_Framework.NetSignals[7]          // HRADC1
#define PS2_LOAD_VOLTAGE                DP_Framework_MtoC.NetSignals[10]    // ANI7
#define PS2_DCLINK_VOLTAGE              DP_Framework_MtoC.NetSignals[6]     // ANI1
#define PS2_TEMPERATURE                 DP_Framework_MtoC.NetSignals[14]    // I2C Add 0x49

#define ERROR_CALCULATOR_PS2            &DP_Framework.DPlibrary.ELP_Error[1]
#define PI_DAWU_CONTROLLER_ILOAD_PS2    &DP_Framework.DPlibrary.ELP_PI_dawu[1]

/**
 * Power supply 3 defines
 */

#define PS3_ID                          0x0004

#define PIN_OPEN_PS3_DCLINK_RELAY       CLEAR_GPDO1;
#define PIN_CLOSE_PS3_DCLINK_RELAY      SET_GPDO1;

#define PIN_STATUS_PS3_DCLINK_RELAY     GET_GPDI8
#define PIN_STATUS_PS3_DRIVER_ERROR     GET_GPDI1
#define PIN_STATUS_PS3_FUSE             GET_GPDI13

#define PS3_LOAD_CURRENT                DP_Framework.NetSignals[9]          // HRADC2
#define PS3_LOAD_VOLTAGE                DP_Framework_MtoC.NetSignals[11]    // ANI3
#define PS3_DCLINK_VOLTAGE              DP_Framework_MtoC.NetSignals[7]     // ANI4
#define PS3_TEMPERATURE                 DP_Framework_MtoC.NetSignals[15]    // I2C Add 0x4A

#define ERROR_CALCULATOR_PS3            &DP_Framework.DPlibrary.ELP_Error[2]
#define PI_DAWU_CONTROLLER_ILOAD_PS3    &DP_Framework.DPlibrary.ELP_PI_dawu[2]

/**
 * Power supply 4 defines
 */

#define PS4_ID                          0x0008

#define PIN_OPEN_PS4_DCLINK_RELAY       CLEAR_GPDO2;
#define PIN_CLOSE_PS4_DCLINK_RELAY      SET_GPDO2;

#define PIN_STATUS_PS4_DCLINK_RELAY     GET_GPDI2
#define PIN_STATUS_PS4_DRIVER_ERROR     GET_GPDI3
#define PIN_STATUS_PS4_FUSE             GET_GPDI16

#define PS4_LOAD_CURRENT                DP_Framework.NetSignals[11]         // HRADC3
#define PS4_LOAD_VOLTAGE                DP_Framework_MtoC.NetSignals[12]    // ANI5
#define PS4_DCLINK_VOLTAGE              DP_Framework_MtoC.NetSignals[8]     // ANI0
#define PS4_TEMPERATURE                 DP_Framework_MtoC.NetSignals[16]    // I2C Add 0x4C

#define ERROR_CALCULATOR_PS4            &DP_Framework.DPlibrary.ELP_Error[3]
#define PI_DAWU_CONTROLLER_ILOAD_PS4    &DP_Framework.DPlibrary.ELP_PI_dawu[3]

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
static void reset_controller(void);
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
    uint16_t i;

    i = 0;
    num_ps  = 0;

    init_peripherals_drivers();
    init_controller();
    init_interruptions();
    enable_controller();

    /// TODO: include condition for re-initialization
    while(1)
    {

    }

    turn_off();
    disable_controller();
    term_interruptions();
    reset_controller;
    term_peripherals_drivers();
}

static void init_peripherals_drivers(void)
{

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

    /**
     * TODO: initialize control laws and time-slicers
     */
}

static void reset_controller(void)
{

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
