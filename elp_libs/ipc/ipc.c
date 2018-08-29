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
 * @file ipc.c
 * @brief Interprocessor Communication module
 *
 * This module is responsible for definition of interprocessor communication
 * functionalities, between ARM and C28 cores.
 * 
 * @author gabriel.brunheira
 * @date 22/11/2017
 *
 */

#include <stdint.h>
#include "boards/udc_c28.h"
#include "common/structs.h"
#include "common/timeslicer.h"
#include "control/control.h"
#include "ipc.h"

#pragma DATA_SECTION(g_wfmref,"SHARERAMS23")
volatile float g_wfmref[SIZE_WFMREF];

#pragma DATA_SECTION(g_buf_samples_ctom,"SHARERAMS45")
volatile float g_buf_samples_ctom[SIZE_BUF_SAMPLES_CTOM];

#pragma DATA_SECTION(g_buf_samples_mtoc,"SHARERAMS67")
volatile float g_buf_samples_mtoc[SIZE_BUF_SAMPLES_MTOC];

#pragma DATA_SECTION(g_ipc_ctom,"CTOM_MSG_RAM");
#pragma DATA_SECTION(g_ipc_mtoc,"MTOC_MSG_RAM");
volatile ipc_ctom_t g_ipc_ctom;
volatile ipc_mtoc_t g_ipc_mtoc;

/**
 * Interrupt service routine for handling Low Priority MtoC IPC messages
 */
interrupt void isr_ipc_lowpriority_msg(void);

/**
 * Interrupt service routine for Synchronization Pulse
 */
interrupt void isr_ipc_sync_pulse(void);

/**
 * Initialization of interprocessor communication (IPC)
 *
 * @param p_ipc_ctom CtoM IPC struct (C28 to ARM)
 */
void init_ipc(void)
{
    uint16_t i;

    for(i = 0; i < SIZE_VERSION; i++)
    {
        g_ipc_ctom.udc_c28_version[i] = udc_c28_version[i];
    }

    g_ipc_ctom.msg_mtoc = 0;
    g_ipc_ctom.msg_id = 0;
    g_ipc_ctom.error_mtoc = No_Error_MtoC;
    g_ipc_ctom.counter_set_slowref =  0;
    g_ipc_ctom.counter_sync_pulse =  0;

    WFMREF = g_ipc_mtoc.wfmref;

    EALLOW;

    /**
    *  Set synchronization via INT_GENERAL or EPWMSYNCI pin (defined by UDC
    *  version)
    *
    *   - Define GPIO32/38/55 as interrupt source XINT2
    *   - Negative-edge triggering
    *   - Qualification is asynchronous
    *   - Enable XINT2
    *   - Map XINT2 ISR
    */
    if(UDC_V2_0)
    {
        GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;
        GpioCtrlRegs.GPBDIR.bit.GPIO32 = 0;
        GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 1;

        GpioTripRegs.GPTRIP5SEL.bit.GPTRIP5SEL = 32;
        /*GpioTripRegs.GPTRIP6SEL.bit.GPTRIP6SEL = 32;*/
    }

    else if(UDC_V2_1)
    {
        GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 0;    // GPIO55: INT_GENERAL/SYNC_IN
        GpioCtrlRegs.GPBDIR.bit.GPIO55 = 0;
        GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 1;

        GpioCtrlRegs.GPBMUX1.bit.GPIO38 = 0;    // GPIO38: EPWMSYNCI
        GpioCtrlRegs.GPBDIR.bit.GPIO38 = 0;
        GpioCtrlRegs.GPBQSEL1.bit.GPIO38 = 1;

        /**
         * TODO: improve GPIO selection
         */
        GpioTripRegs.GPTRIP5SEL.bit.GPTRIP5SEL = 55;//38;
        /*GpioTripRegs.GPTRIP6SEL.bit.GPTRIP6SEL = 38;*/
    }

    XIntruptRegs.XINT2CR.bit.ENABLE = 1;
    XIntruptRegs.XINT2CR.bit.POLARITY = 0;
    PieVectTable.XINT2 = &isr_ipc_sync_pulse;

    /**
     * TODO: choose between XINT2 or XINT3
     */
    /*XIntruptRegs.XINT3CR.bit.ENABLE = 0;
    XIntruptRegs.XINT3CR.bit.POLARITY = 0;*/

    /**
     * TODO: create enable and disable EPWMSYNCO functions on pwm module
     */
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 2; // Configures GPIO33 for EPWMSYNCO

    /**
     * TODO: verify how to specify isr_interlocks
     */
    /* Map IPC_MtoC interrupts */
    //PieVectTable.XINT3      = &isr_ipc_sync_pulse;
    PieVectTable.MTOCIPC_INT1 = &isr_ipc_lowpriority_msg;
    PieVectTable.MTOCIPC_INT2 = &isr_ipc_sync_pulse;
    //PieVectTable.MTOCIPC_INT3 = &isr_hard_interlock;
    //PieVectTable.MTOCIPC_INT4 = &isr_soft_interlock;

    /* Enable interrupts */

    PieCtrlRegs.PIEIER1.bit.INTx5  = 1;                 //    XINT2
    //PieCtrlRegs.PIEIER12.bit.INTx1 = 1;               //    XINT3

    PieCtrlRegs.PIEIER11.bit.INTx1 = 1;                 // MTOCIPCINT1
    PieCtrlRegs.PIEIER11.bit.INTx2 = 1;                 // MTOCIPCINT2
    PieCtrlRegs.PIEIER11.bit.INTx3 = 0;                 // MTOCIPCINT3
    PieCtrlRegs.PIEIER11.bit.INTx4 = 0;                 // MTOCIPCINT4

    CtoMIpcRegs.MTOCIPCACK.all = SYNC_PULSE;
    PieCtrlRegs.PIEACK.all |= M_INT1;
    PieCtrlRegs.PIEACK.all |= M_INT11;

    EDIS;
}

/**
 * Send IPC CtoM message. This function must be used with care, because it
 * directly sets CTOMIPC register bits according to the argument `msg` when
 * there's no pending messages.
 *
 * @param msg_id specified IPC module
 * @param msg specified message
 */
void send_ipc_msg(uint16_t msg_id, uint32_t msg)
{
    if(CtoMIpcRegs.CTOMIPCFLG.all == 0x00000000)
    {
        g_ipc_ctom.msg_id = msg_id;
        CtoMIpcRegs.CTOMIPCSET.all = msg;
    }
}

/**
 * Send IPC CtoM Low Priority message. It ignores attempts to set high priority
 * bits or to subscribe pending low priority messages.
 *
 * @param msg_id specified IPC module
 * @param msg specified message
 */
void send_ipc_lowpriority_msg(uint16_t msg_id, ipc_ctom_lowpriority_msg_t msg)
{
    if(CtoMIpcRegs.CTOMIPCFLG.all == 0x00000000)
    {
        g_ipc_ctom.msg_id = msg_id;
        CtoMIpcRegs.CTOMIPCSET.all = (uint32_t) ( ( (uint16_t) msg << 4 ) &
                                     0x000FFFF0 | IPC_CTOM_LOWPRIORITY_MSG);
    }
}

/**
 * Interrupt Service Routine for IPC MtoC Low Priority Messages.
 */
interrupt void isr_ipc_lowpriority_msg(void)
{
    static uint16_t i;

    g_ipc_ctom.msg_mtoc = CtoMIpcRegs.MTOCIPCSTS.all;
    CtoMIpcRegs.MTOCIPCACK.all = g_ipc_ctom.msg_mtoc;

    if(g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_status.bit.active)
    {
        switch(GET_IPC_MTOC_LOWPRIORITY_MSG)
        {
            case Turn_On:
            {
                /**
                 * TODO: where should disable siggen + reset wfmref be?
                 */
                g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].turn_on(g_ipc_mtoc.msg_id);
                break;
            }

            case Turn_Off:
            {
                /**
                 * TODO: where should disable siggen + reset wfmref be?
                 */
                g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].turn_off(g_ipc_mtoc.msg_id);
                break;
            }

            case Open_Loop:
            {
                open_loop(&g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id]);
                break;
            }

            case Close_Loop:
            {
                close_loop(&g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id]);
                break;
            }

            case Operating_Mode:
            {
                /**
                 * TODO:
                 */
                if(g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_status.bit.state
                   > Interlock)
                {
                    switch(g_ipc_mtoc.ps_module[g_ipc_mtoc.msg_id].ps_status.bit.state)
                    {
                        case RmpWfm:
                        case MigWfm:
                        {
                            WFMREF = g_ipc_mtoc.wfmref;
                            WFMREF.wfmref_data.status = Buffering;
                            break;
                        }

                        case Cycle:
                        {
                            WFMREF.wfmref_data.status = Idle;
                            disable_siggen(&g_ipc_ctom.siggen);
                            break;
                        }

                        default:
                        {
                            WFMREF.wfmref_data.status = Idle;
                            break;
                        }
                    }

                    cfg_ps_operation_mode(&g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id],
                                          g_ipc_mtoc.ps_module[g_ipc_mtoc.msg_id].ps_status.bit.state);
                }

                break;
            }

            case Reset_Interlocks:
            {
                g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].reset_interlocks(g_ipc_mtoc.msg_id);
                break;
            }

            case Unlock_UDC:
            {
                unlock_ps_module(&g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id]);
                break;
            }

            case Lock_UDC:
            {
                lock_ps_module(&g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id]);
                break;
            }

            case Cfg_Buf_Samples:
            {
                /**
                 * TODO: implement cfg_buf_samples
                 */
                break;
            }

            case Enable_Buf_Samples:
            {
                enable_buffer(&g_ipc_ctom.buf_samples[g_ipc_mtoc.msg_id]);
                break;
            }

            case Disable_Buf_Samples:
            {
                /**
                 * TODO: It sets as Postmortem to wait buffer complete. Maybe
                 * it's better to create a postmortem BSMP function
                 */
                postmortem_buffer(&g_ipc_ctom.buf_samples[g_ipc_mtoc.msg_id]);
                //disable_buffer(&g_ipc_ctom.buf_samples[g_ipc_mtoc.msg_id]);
                break;
            }

            case Set_SlowRef:
            {
                SET_DEBUG_GPIO1;

                if(g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_status.bit.state == SlowRef)
                {
                    g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_setpoint =
                    g_ipc_mtoc.ps_module[g_ipc_mtoc.msg_id].ps_setpoint;
                }

                else if(g_ipc_ctom.ps_module[g_ipc_mtoc.msg_id].ps_status.bit.state != SlowRefSync)
                {
                    g_ipc_ctom.error_mtoc = Invalid_OpMode;
                    send_ipc_lowpriority_msg(g_ipc_mtoc.msg_id, MtoC_Message_Error);
                }

                g_ipc_ctom.counter_set_slowref++;

                break;
            }

            case Set_SlowRef_All_PS:
            {
                SET_DEBUG_GPIO1;

                for(i = 0; i < NUM_MAX_PS_MODULES; i++)
                {
                    if(g_ipc_ctom.ps_module[i].ps_status.bit.active)
                    {
                        if(g_ipc_ctom.ps_module[i].ps_status.bit.state == SlowRef)
                        {
                            g_ipc_ctom.ps_module[i].ps_setpoint =
                            g_ipc_mtoc.ps_module[i].ps_setpoint;
                        }
                        else if(g_ipc_ctom.ps_module[i].ps_status.bit.state != SlowRefSync)
                        {
                            g_ipc_ctom.error_mtoc = Invalid_OpMode;
                            send_ipc_lowpriority_msg(g_ipc_mtoc.msg_id, MtoC_Message_Error);
                        }
                    }
                }

                g_ipc_ctom.counter_set_slowref++;

                break;
            }

            case Reset_WfmRef:
            {
                WFMREF = g_ipc_mtoc.wfmref;
                WFMREF.wfmref_data.status = Buffering;
            }

            case Cfg_SigGen:
            {
                cfg_siggen(&g_ipc_ctom.siggen,g_ipc_mtoc.siggen.type,
                           g_ipc_mtoc.siggen.num_cycles,
                           g_ipc_mtoc.siggen.freq,
                           g_ipc_mtoc.siggen.amplitude,
                           g_ipc_mtoc.siggen.offset,
                           g_ipc_mtoc.siggen.aux_param);
                break;
            }

            case Set_SigGen:
            {
                set_siggen_freq(&g_ipc_ctom.siggen, g_ipc_mtoc.siggen.freq);
                break;
            }

            case Enable_SigGen:
            {
                enable_siggen(&g_ipc_ctom.siggen);
                break;
            }

            case Disable_SigGen:
            {
                disable_siggen(&g_ipc_ctom.siggen);
                break;
            }

            case Set_Param:
            {
                break;
            }

            case Set_DSP_Coeffs:
            {
                set_dsp_coeffs(g_ipc_mtoc.dsp_module.dsp_class,
                               g_ipc_mtoc.dsp_module.id);
                break;
            }

            case Reset_Counters:
            {
                g_ipc_ctom.counter_set_slowref =  0;
                g_ipc_ctom.counter_sync_pulse =  0;
                break;
            }

            case CtoM_Message_Error:
            {
                /**
                 * TODO: take action when receiving error
                 */
                break;
            }

            default:
            {
                /**
                 * TODO: check
                 */
                g_ipc_ctom.error_mtoc = IPC_LowPriority_Full;
                send_ipc_lowpriority_msg(g_ipc_mtoc.msg_id, MtoC_Message_Error);
                break;
            }
        }
    }

    PieCtrlRegs.PIEACK.all |= M_INT11;
}

interrupt void isr_ipc_sync_pulse(void)
{
    uint16_t i;

    SET_DEBUG_GPIO1;

    for(i = 0; i < NUM_MAX_PS_MODULES; i++)
    {
        if(g_ipc_ctom.ps_module[i].ps_status.bit.active)
        {
            switch(g_ipc_ctom.ps_module[i].ps_status.bit.state)
            {
                case SlowRefSync:
                {
                    g_ipc_ctom.ps_module[i].ps_setpoint =
                    g_ipc_mtoc.ps_module[i].ps_setpoint;
                    break;
                }

                case RmpWfm:
                {
                    switch(WFMREF.sync_mode)
                    {
                        case SampleBySample:
                        {
                            if(WFMREF.wfmref_data.p_buf_idx++ >=
                               WFMREF.wfmref_data.p_buf_end)
                            {
                                WFMREF.wfmref_data.p_buf_idx =
                                        WFMREF.wfmref_data.p_buf_start;
                                WFMREF.gain = g_ipc_mtoc.wfmref.gain;
                                WFMREF.offset = g_ipc_mtoc.wfmref.offset;
                            }

                            break;
                        }

                        case SampleBySample_OneCycle:
                        {
                            if(WFMREF.wfmref_data.p_buf_idx++ ==
                               WFMREF.wfmref_data.p_buf_end)
                            {
                                WFMREF.wfmref_data.p_buf_idx =
                                        WFMREF.wfmref_data.p_buf_end;
                            }
                            else if(WFMREF.wfmref_data.p_buf_idx++ >
                                    WFMREF.wfmref_data.p_buf_end)
                            {
                                WFMREF.wfmref_data.p_buf_idx =
                                        WFMREF.wfmref_data.p_buf_start;
                                WFMREF.gain = g_ipc_mtoc.wfmref.gain;
                                WFMREF.offset = g_ipc_mtoc.wfmref.offset;
                            }

                            break;
                        }

                        case OneShot:
                        {
                            WFMREF = g_ipc_mtoc.wfmref;
                            WFMREF.wfmref_data.status = Buffering;
                            g_timeslicers.counter[TIMESLICER_WFMREF] =
                                    g_timeslicers.freq_ratio[TIMESLICER_WFMREF];

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
                    enable_siggen(&g_ipc_ctom.siggen);
                    break;
                }

                default:
                {
                    break;
                }
            }
        }
    }

    g_ipc_ctom.counter_sync_pulse++;

    CtoMIpcRegs.MTOCIPCACK.all = SYNC_PULSE;
    PieCtrlRegs.PIEACK.all |= M_INT1;
    PieCtrlRegs.PIEACK.all |= M_INT11;

    //CLEAR_DEBUG_GPIO1;
}
