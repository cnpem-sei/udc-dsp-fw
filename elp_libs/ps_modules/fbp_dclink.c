/*
 * fbp_dclink.c
 *
 *  Created on: 20 de fev de 2018
 *      Author:
 */

#include "fbp_dclink.h"
#include "boards/udc_c28.h"
#include "control/control.h"
#include "ipc/ipc.h"
#include "common/timeslicer.h"

/**
 * Configuration parameters
 *
 * TODO: transfer this to param bank
 */

#define USE_ITLK
#define TIMEOUT_DCLINK_RELAY    200000

#define PWM_FREQ                50000.0     /// PWM frequency [Hz]
#define PWM_DEAD_TIME           300         /// PWM dead-time [ns]
#define PWM_MAX_DUTY            0.9         /// Max duty cycle [p.u.]
#define PWM_MIN_DUTY            -0.9        /// Min duty cycle [p.u.]
#define PWM_MAX_DUTY_OL         0.9         /// Max open loop duty cycle [p.u.]
#define PWM_MIN_DUTY_OL         -0.9        /// Min open loop duty cycle [p.u.]

#define MAX_REF                 10.0        /// Reference over-saturation level [A]
#define MIN_REF                 -10.0       /// Reference under-saturation level [A]
#define MAX_ILOAD               10.5        /// Reference limit for interlock [A]
#define MAX_VLOAD               13.5        /// Load voltage limit for interlock [V]
#define MIN_DCLINK              3.0         /// DC Link under limit for interlock [V]
#define NOM_VDCLINK             15.0        /// Nominal DC Link
#define MAX_DCLINK              17.0        /// DC Link over limit for interlock [V]
#define MAX_TEMP                80.0        /// Temperature limit for interlock [ÂºC]

#define MAX_REF_SLEWRATE        1000000.0   /// Max reference slew-rate [A/s]
#define MAX_SR_SIGGEN_OFFSET    50.0        /// Max SigGen offset slew-rate [A/s]
#define MAX_SR_SIGGEN_AMP       100.0       /// Max SigGen amplitude slew-rate [A/s]

#define APPLICATION             UVX_LINAC_RACK1

#define UVX_LINAC_RACK1     0
#define UVX_LINAC_RACK2     1
#define WEG_PILOT_BATCH_CHARACTERIZATION    2
#define ELP_FAC_CON_TESTS   3

///
#define CONTROL_FREQ            (2.0*PWM_FREQ)
#define CONTROL_PERIOD          (1.0/CONTROL_FREQ)
#define DECIMATION_FACTOR       1
#define TRANSFER_BUFFER_SIZE    DECIMATION_FACTOR
#define HRADC_FREQ_SAMP         (float) CONTROL_FREQ*DECIMATION_FACTOR
#define HRADC_SPI_CLK           SPI_15MHz

#define BUFFER_DECIMATION       1
#define WFMREF_SAMPLING_FREQ    8000.0
#define SIGGEN                  g_ipc_ctom.siggen[0]
#define SIGGEN_OUTPUT           g_controller_ctom.net_signals[12]

#define TRANSDUCER_INPUT_RATED      12.5            /// ** DCCT LEM ITN 12-P **
#define TRANSDUCER_OUTPUT_RATED     0.05            /// In_rated   = +/- 12.5 A
#define TRANSDUCER_OUTPUT_TYPE      Iin_bipolar     /// Out_rated  = +/- 50 mA
#define HRADC_R_BURDEN              20.0            /// Burden resistor = 20 R
#if (HRADC_v2_0)
    #define TRANSDUCER_GAIN         -(TRANSDUCER_INPUT_RATED/TRANSDUCER_OUTPUT_RATED)
#endif
#if (HRADC_v2_1)
    #define TRANSDUCER_GAIN         (TRANSDUCER_INPUT_RATED/TRANSDUCER_OUTPUT_RATED)
#endif
///

/**
 * Set Pins
 */
#define PIN_OPEN_ALL_DCLINK_RELAY       CLEAR_GPDO1; // Pin On/Off Fontes.
#define PIN_CLOSE_ALL_DCLINK_RELAY      SET_GPDO1;

#define PIN_OPEN_EXT_RELAY              CLEAR_GPDO2; // Pin On/Off Manual.
#define PIN_CLOSE_EXT_RELAY             SET_GPDO2;

#define PIN_IN_DCLINK                   GET_GPDI4;   // In_1 p/ Sensor_de_Fumaça.
#define PIN_IN_SMO2_DCLINK              GET_GPDI5;   // In_2 p/ Sensor_de_Fumaça.

#define PIN_STATUS_FAIL_PF1             GET_GPDI1;   //Check PF1.
#define PIN_STATUS_FAIL_PF2             GET_GPDI2;   //Check PF2.
#define PIN_STATUS_FAIL_PF3             GET_GPDI3;   //Check PF3.
//

#pragma CODE_SECTION(isr_init_controller, "ramfuncs");
#pragma CODE_SECTION(isr_controller, "ramfuncs");
#pragma CODE_SECTION(turn_on, "ramfuncs");
#pragma CODE_SECTION(turn_off, "ramfuncs");
#pragma CODE_SECTION(set_hard_interlock, "ramfuncs");
#pragma CODE_SECTION(set_soft_interlock, "ramfuncs");
#pragma CODE_SECTION(isr_hard_interlock, "ramfuncs");
#pragma CODE_SECTION(isr_soft_interlock, "ramfuncs");
#pragma CODE_SECTION(open_relay, "ramfuncs");
#pragma CODE_SECTION(close_relay, "ramfuncs");
#pragma CODE_SECTION(get_relay_status, "ramfuncs");

static void init_interruptions(void);
static void enable_controller();

static void turn_on(uint16_t id);
static void turn_off(uint16_t id);

static void check_interlocks_ps_module(uint16_t id);
static void reset_controller(uint16_t id);
static void set_hard_interlock(uint16_t id, uint32_t itlk);
static uint16_t get_relay_status();

void main_fbp_dclink(void)
{
    init_interruptions();
    enable_controller();

    /// TODO: include condition for re-initialization
    while(1)
    {
        check_interlocks_ps_module(0); //Sempre conferir o interlock da fonte.
    }

    turn_off(0);

    disable_controller();
    term_interruptions();
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
 * Check variables from specified power supply for interlocks
 *
 * @param id specified power supply
 */
static void check_interlocks_ps_module(uint16_t id)
{
    if(fabs(g_controller_ctom.net_signals[id]) > MAX_ILOAD)
    {
        set_hard_interlock(id, LOAD_OVERCURRENT);
    }

    if(fabs(g_controller_mtoc.net_signals[id]) > MAX_DCLINK)
    {
        set_hard_interlock(id, DCLINK_OVERVOLTAGE);
    }

    if(fabs(g_controller_mtoc.net_signals[id]) < MIN_DCLINK)
    {
        set_hard_interlock(id, DCLINK_UNDERVOLTAGE);
    }

    if(fabs(g_controller_mtoc.net_signals[id+4]) > MAX_VLOAD)
    {
        set_hard_interlock(id, LOAD_OVERVOLTAGE);
    }

    if(fabs(g_controller_mtoc.net_signals[id+8]) > MAX_TEMP)
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

/**
 * Turn on specified power supply.
 *
 * @param id specified power supply
 */
static void turn_on(uint16_t id)
{
    #ifdef USE_ITLK
    if(g_ipc_ctom.ps_module[0].ps_status.bit.state == Off)
    #else
    if(g_ipc_ctom.ps_module[0].ps_status.bit.state <= Interlock)
    #endif
    {
        PIN_CLOSE_ALL_DCLINK_RELAY;
        PIN_CLOSE_EXT_RELAY;

        DELAY_US(TIMEOUT_DCLINK_RELAY);

        if(get_relay_status() == 1)
        {
            set_hard_interlock(0,DCLINK_RELAY_FAIL);
        }
        else
        {
            g_ipc_ctom.ps_module[0].ps_status.bit.openloop = OPEN_LOOP;
            g_ipc_ctom.ps_module[0].ps_status.bit.state = SlowRef;

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
    PIN_OPEN_ALL_DCLINK_RELAY;
    PIN_OPEN_EXT_RELAY;

    DELAY_US(TIMEOUT_DCLINK_RELAY);

    g_ipc_ctom.ps_module[0].ps_status.bit.openloop = OPEN_LOOP;
    if (g_ipc_ctom.ps_module[0].ps_status.bit.state != Interlock)
    {
        g_ipc_ctom.ps_module[0].ps_status.bit.state = Off;
    }
}

/**
 * Disable control ISR
 */
static void disable_controller()
{
    disable_pwm_tbclk();
    HRADCs_Info.enable_Sampling = 0;
    stop_DMA();
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
 * Set specified hard interlock for specified power supply.
 *
 * @param id specified power supply
 * @param itlk specified hard interlock
 */
static void set_hard_interlock(uint16_t id, uint32_t itlk)
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
 * Get relay status from specified power supply.
 *
 * @param id specified power supply
 */
static uint16_t get_relay_status()
{
    if ((PIN_STATUS_FAIL_PF1==0)||(PIN_STATUS_FAIL_PF2==0)||(PIN_STATUS_FAIL_PF3==0))
    {
        return (1);
    }
    else
    {
        return (0);
    }
}
