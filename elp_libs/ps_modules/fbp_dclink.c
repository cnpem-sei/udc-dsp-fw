/*
 * fbp_dclink.c
 *
 *  Created on: 20 de fev de 2018
 *      Author: paulo.santos
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

#define USE_ITLK                0
#define TIMEOUT_DCLINK_RELAY    200000

#define PF1_FAIL                0x00000001
#define PF2_FAIL                0x00000002
#define PF3_FAIL                0x00000004
#define SEN_SMOKE               0x00000008
#define SEN_EXTER               0x00000010
#define VPF1_OVERVOLTAGE        0x00000020
#define VPF2_OVERVOLTAGE        0x00000040
#define VPF3_OVERVOLTAGE        0x00000080
#define VPF1_UNDERVOLTAGE       0x00000100
#define VPF2_UNDERVOLTAGE       0x00000200
#define VPF3_UNDERVOLTAGE       0x00000400
#define ALL_PS_FAIL             0x00000800

#define APPLICATION             UVX_LINAC_RACK1

#define UVX_LINAC_RACK1                     0
#define UVX_LINAC_RACK2                     1
#define WEG_PILOT_BATCH_CHARACTERIZATION    2
#define ELP_FAC_CON_TESTS                   3

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
    #define TRANSDUCER_GAIN -(TRANSDUCER_INPUT_RATED/TRANSDUCER_OUTPUT_RATED)
#endif
#if (HRADC_v2_1)
    #define TRANSDUCER_GAIN (TRANSDUCER_INPUT_RATED/TRANSDUCER_OUTPUT_RATED)
#endif
///

/**
 * Set Pins
 */
#define PIN_OPEN_ALL_DCLINK_RELAY   CLEAR_GPDO1;    // Pin On/Off Fontes.
#define PIN_CLOSE_ALL_DCLINK_RELAY  SET_GPDO1;

#define PIN_OPEN_EXT_RELAY          CLEAR_GPDO2;    // Pin On/Off Manual.
#define PIN_CLOSE_EXT_RELAY         SET_GPDO2;

#define PIN_IN_SMOKE_DCLINK         GET_GPDI4       // In_1 p/ Sensor_de_Fumaça.
#define PIN_IN_EXTER_DCLINK         GET_GPDI5       // In_2 p/ Entrada Externa.

#define GET_PIN_STATUS_FAIL_PS1     GET_GPDI1       //Check PS1.
#define GET_PIN_STATUS_FAIL_PS2     GET_GPDI2       //Check PS2.
#define GET_PIN_STATUS_FAIL_PS3     GET_GPDI3       //Check PS3.

#define PIN_STATUS_FAIL_PS_ALL      g_controller_ctom.net_signals[0].u32

#define SOUR01_DCLINK_VOLTAGE       g_controller_mtoc.net_signals[0].f   // ANI1
#define SOUR02_DCLINK_VOLTAGE       g_controller_mtoc.net_signals[1].f   // ANI2
#define SOUR03_DCLINK_VOLTAGE       g_controller_mtoc.net_signals[2].f   // ANI0
//
//Voltage limit(Range) for interlock [V]
#define MAX_VPS1                    20.0
#define MAX_VPS2                    20.0
#define MAX_VPS3                    20.0
#define MIN_VPS1                    3.0
#define MIN_VPS2                    3.0
#define MIN_VPS3                    3.0
//

/*
 * Input Function Prototype
 */
static void init_controller(void);
static void init_interruptions(void);
static void check_interlocks_ps_module(void);
static void turn_on(uint16_t id);
static void turn_off(uint16_t id);
static void term_interruptions(void);
static void set_hard_interlock(uint32_t itlk);
static uint16_t get_relay_status(void);
static interrupt void isr_hard_interlock(void);
static interrupt void isr_soft_interlock(void);
static void reset_interlocks(uint16_t id);

//

void main_fbp_dclink(void)
{
    init_controller();
    init_interruptions();
    turn_off(0);

    /// TODO: include condition for re-initialization
    while(1)
    {
        check_interlocks_ps_module();

        PIN_STATUS_FAIL_PS_ALL |= (!GET_PIN_STATUS_FAIL_PS1);
        PIN_STATUS_FAIL_PS_ALL |= (!GET_PIN_STATUS_FAIL_PS2) << 1;
        PIN_STATUS_FAIL_PS_ALL |= (!GET_PIN_STATUS_FAIL_PS3) << 2;
    }

    turn_off(0);
    term_interruptions();
}

static void init_controller(void)
{
    init_ps_module(&g_ipc_ctom.ps_module[0],
                   g_ipc_mtoc.ps_module[0].ps_status.bit.model,
                   &turn_on, &turn_off, &isr_soft_interlock,
                   &isr_hard_interlock, &reset_interlocks);

    init_ipc();
    init_control_framework(&g_controller_ctom);
}

static void init_interruptions(void)
{
    IER |= M_INT11;

    /// Enable global interrupts (EINT)
    EINT;
    ERTM;
}

/**
 * Check variables from specified power supply for interlocks
 *
 * @param id specified power supply
 */
static void check_interlocks_ps_module(void)
{
    //Se uma das entradas das fontes acusar 0 ZERO.
    if (!GET_PIN_STATUS_FAIL_PS1)
    {
        set_hard_interlock(PF1_FAIL);
    }
    if (!GET_PIN_STATUS_FAIL_PS2)
    {
        set_hard_interlock(PF2_FAIL);
    }
    if (!GET_PIN_STATUS_FAIL_PS3)
    {
        set_hard_interlock(PF3_FAIL);
    }
    //

    //Se todas as entradas das Fontes acusarem 0 ZERO.
//    if ((!GET_PIN_STATUS_FAIL_PS1) && (!GET_PIN_STATUS_FAIL_PS2) && (!GET_PIN_STATUS_FAIL_PS3))
//    {
//        if(!(g_ipc_ctom.ps_module[0].ps_hard_interlock & ALL_PS_FAIL))
//        {
//            #ifdef USE_ITLK
//            turn_off(0);
//            g_ipc_ctom.ps_module[0].ps_status.bit.state = Interlock;
//            #endif
//            g_ipc_ctom.ps_module[0].ps_hard_interlock |= ALL_PS_FAIL;
//        }
//    }

    //Sensor de fumaça.
    if (PIN_IN_SMOKE_DCLINK)
    {
        set_hard_interlock(SEN_SMOKE);
    }
    if (PIN_IN_EXTER_DCLINK)
    {
        set_hard_interlock(SEN_EXTER);
    }
    //

    //OVERVOLTAGE Interlock.
    if (fabs(SOUR01_DCLINK_VOLTAGE) > MAX_VPS1)
    {
        set_hard_interlock(VPF1_OVERVOLTAGE);
    }
    if (fabs(SOUR02_DCLINK_VOLTAGE) > MAX_VPS2)
    {
        set_hard_interlock(VPF2_OVERVOLTAGE);
    }
    if (fabs(SOUR03_DCLINK_VOLTAGE) > MAX_VPS3)
    {
        set_hard_interlock(VPF3_OVERVOLTAGE);
    }
    //

    //UNDERVOLTAGE Interlock.
    if (fabs(SOUR01_DCLINK_VOLTAGE) < MIN_VPS1)
    {
        set_hard_interlock(VPF1_UNDERVOLTAGE);
    }
    if (fabs(SOUR02_DCLINK_VOLTAGE) < MIN_VPS2)
    {
        set_hard_interlock(VPF2_UNDERVOLTAGE);
    }
    if (fabs(SOUR03_DCLINK_VOLTAGE) < MIN_VPS3)
    {
        set_hard_interlock(VPF3_UNDERVOLTAGE);
    }
    //
}

/**
 * Turn on specified power supply.
 *
 * @param id specified power supply
 */
static void turn_on(uint16_t id)
{
    #ifdef USE_ITLK
    if(g_ipc_ctom.ps_module[0].ps_status.bit.state <= Interlock)
    #else
    if(g_ipc_ctom.ps_module[0].ps_status.bit.state <= Interlock)
    #endif
    {
        PIN_CLOSE_ALL_DCLINK_RELAY;
        PIN_CLOSE_EXT_RELAY;

        DELAY_US(TIMEOUT_DCLINK_RELAY);
        g_ipc_ctom.ps_module[0].ps_status.bit.state = SlowRef;
    }
}

/**
 * Turn off specified power supply.
 *
 * @param id specified power supply
 */
static void turn_off(uint16_t id)
{
    if (g_ipc_ctom.ps_module[0].ps_status.bit.state >= Interlock)
    {
        PIN_OPEN_ALL_DCLINK_RELAY;
        PIN_OPEN_EXT_RELAY;

        DELAY_US(TIMEOUT_DCLINK_RELAY);
        g_ipc_ctom.ps_module[0].ps_status.bit.state = Off;
    }
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

    /// Clear flags
    PieCtrlRegs.PIEACK.all |= M_INT11;
}

/**
 * Set specified hard interlock for specified power supply.
 *
 * @param id specified power supply
 * @param itlk specified hard interlock
 */
static void set_hard_interlock(uint32_t itlk)
{
    if(!(g_ipc_ctom.ps_module[0].ps_hard_interlock & itlk))
    {
        #ifdef USE_ITLK
        //turn_off(0);
        g_ipc_ctom.ps_module[0].ps_status.bit.state = Interlock;
        #endif
        g_ipc_ctom.ps_module[0].ps_hard_interlock |= itlk;
    }
}

/**
 * ISR for MtoC hard interlock request.
 */
static interrupt void isr_hard_interlock(void)
{
    if(! (g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_hard_interlock &
         g_ipc_mtoc.ps_module[g_ipc_mtoc.msg_id].ps_hard_interlock))
    {
        #ifdef USE_ITLK
        turn_off(0);
        g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_status.bit.state = Interlock;
        #endif

        g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_hard_interlock |=
        g_ipc_mtoc.ps_module[g_ipc_mtoc.msg_id].ps_hard_interlock;
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

    if(g_ipc_ctom.ps_module[id].ps_status.bit.state < Initializing)
    {
        g_ipc_ctom.ps_module[id].ps_status.bit.state = Off;
    }
}

/**
 * ISR for MtoC soft interlock request.
 */
static interrupt void isr_soft_interlock(void)
{
    if(! (g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_soft_interlock &
         g_ipc_mtoc.ps_module[g_ipc_mtoc.msg_id].ps_soft_interlock))
    {
        #ifdef USE_ITLK
        turn_off(0);
        g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_status.bit.state = Interlock;
        #endif

        g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_soft_interlock |=
        g_ipc_mtoc.ps_module[g_ipc_mtoc.msg_id].ps_soft_interlock;
    }
}
