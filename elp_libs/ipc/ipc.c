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

#include "ipc.h"

ipc_ctom_t ipc_ctom_msg;
ipc_mtoc_t ipc_mtoc_msg;

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
void init_ipc(ipc_ctom_t *p_ipc_ctom, ID)
{
    p_ipc_ctom->msg_id = ID;
    p_ipc_ctom->error_ctom = No_Error_CtoM;

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

    /* Map IPC_MtoC interrupts */
    //PieVectTable.XINT3      = &isr_ipc_sync_pulse;
    PieVectTable.MTOCIPC_INT1 = &isr_ipc_sync_pulse;
    PieVectTable.MTOCIPC_INT2 = &isr_ipc_lowpriority_msg;
    PieVectTable.MTOCIPC_INT3 = isr_SoftItlk;       // Soft interlock
    PieVectTable.MTOCIPC_INT4 = isr_HardItlk;       // Hard interlock

    /* Enable IPC_MtoC interrupts */

    PieCtrlRegs.PIEIER11.bit.INTx1 = 1;                 // MTOCIPCINT1
    PieCtrlRegs.PIEIER11.bit.INTx2 = 1;                 // MTOCIPCINT2  --- Enable only in
    PieCtrlRegs.PIEIER1.bit.INTx5  = 1;                 //    XINT2     --- WfmRef OpMode
    //PieCtrlRegs.PIEIER12.bit.INTx1 = 1;               //    XINT3     --- WfmRef OpMode
    PieCtrlRegs.PIEIER11.bit.INTx3 = 1;                 // MTOCIPCINT3
    PieCtrlRegs.PIEIER11.bit.INTx4 = 1;                 // MTOCIPCINT4

    EDIS;
}

/**
 * Send specified IPC message
 * @param flag indicates bitwise which IPC CtoM message should be sent
 */
void send_ipc_msg(uint16_t msg_id, uint32_t flag)
{
    ipc_ctom_msg.msg_id = msg_id;
    CtoMIpcRegs.CTOMIPCSET.all |= flag;
}

interrupt void isr_ipc_lowpriority_msg(void)
{
    static Uint32 aux;

    aux = (CtoMIpcRegs.MTOCIPCSTS.all & 0xFFFFFFF1);

    switch(aux)
    {
        case TURN_ON:   //IPC1 + IPC5
        {
            /**
             * TODO: where should disable siggen + reset wfmref be?
             */
            CtoMIpcRegs.MTOCIPCACK.all = TURN_ON;
            ipc_ctom.ps_module[ipc_mtoc.msg_id].turn_on();
            PieCtrlRegs.PIEACK.all |= M_INT11;
            break;
        }

        case TURN_OFF:  //IPC1 + IPC6
        {
            /**
             * TODO: where should disable siggen + reset wfmref be?
             */
            CtoMIpcRegs.MTOCIPCACK.all = TURN_OFF;
            ipc_ctom.ps_module[ipc_mtoc.msg_id].turn_off();
            PieCtrlRegs.PIEACK.all |= M_INT11;
            break;
        }

        case OPEN_LOOP: //IPC1 + IPC7
        {
            CtoMIpcRegs.MTOCIPCACK.all = OPEN_LOOP;
            open_loop(&ipc_ctom.ps_module[ipc_mtoc.msg_id]);
            PieCtrlRegs.PIEACK.all |= M_INT11;
            break;
        }

        case CLOSE_LOOP: //IPC1 + IPC8
        {
            CtoMIpcRegs.MTOCIPCACK.all = CLOSE_LOOP;
            close_loop(&ipc_ctom.ps_module[ipc_mtoc.msg_id]);
            PieCtrlRegs.PIEACK.all |= M_INT11;
            break;
        }

        case OPERATING_MODE: //IPC1 + IPC9
        {
            CtoMIpcRegs.MTOCIPCACK.all = OPERATING_MODE;

            switch(ipc_ctom.ps_module[ipc_mtoc.msg_id].ps_status.bit.state)
            {
                case RmpWfm:
                case MigWfm:
                {
                    reset_wfmref(&ipc_ctom.wfmref[ipc_mtoc.msg_id]);
                    break;
                }

                case Cycle:
                {
                    disable_siggen(&ipc_ctom.siggen[ipc_mtoc.msg_id]);
                    reset_siggen(&ipc_ctom.siggen[ipc_mtoc.msg_id]);
                    break;
                }
            }

            cfg_ps_operation_mode( &ipc_ctom.ps_module[ipc_mtoc.msg_id],
                                   ipc_mtoc.ps_module[ipc_mtoc.msg_id].ps_status.bit.state );

            PieCtrlRegs.PIEACK.all |= M_INT11;
            break;
        }

        case RESET_INTERLOCKS: // IPC1 + IPC10
        {
            CtoMIpcRegs.MTOCIPCACK.all = RESET_INTERLOCKS;
            ipc_ctom.ps_module[ipc_mtoc.msg_id].reset_interlocks();
            PieCtrlRegs.PIEACK.all |= M_INT11;
            break;
        }

        case UNLOCK_UDC: //IPC1 + IPC11
        {
            CtoMIpcRegs.MTOCIPCACK.all = UNLOCK_UDC;
            unlock_ps_module(&ipc_ctom.ps_module[ipc_mtoc.msg_id]);
            PieCtrlRegs.PIEACK.all |= M_INT11;
            break;
        }

        case LOCK_UDC: //IPC1 + IPC12
        {
            CtoMIpcRegs.MTOCIPCACK.all = LOCK_UDC;
            lock_ps_module(&ipc_ctom.ps_module[ipc_mtoc.msg_id]);
            PieCtrlRegs.PIEACK.all |= M_INT11;
            break;
        }

        case CONFIG_BUF_SAMPLES: //IPC1 + IPC13
        {
            CtoMIpcRegs.MTOCIPCACK.all = CONFIG_BUF_SAMPLES;
            /**
             * TODO: implement cfg_buf_samples
             */
            PieCtrlRegs.PIEACK.all |= M_INT11;
            break;
        }

        case ENABLE_BUF_SAMPLES: //IPC1 + IPC14
        {
            CtoMIpcRegs.MTOCIPCACK.all = CONFIG_BUF_SAMPLES;
            enable_buffer(&ipc_ctom.buf_samples[ipc_mtoc.msg_id]);
            PieCtrlRegs.PIEACK.all |= M_INT11;
            break;
        }

        case DISABLE_BUF_SAMPLES: //IPC1 + IPC15
        {
            CtoMIpcRegs.MTOCIPCACK.all = CONFIG_BUF_SAMPLES;
            disable_buffer(&ipc_ctom.buf_samples[ipc_mtoc.msg_id]);
            PieCtrlRegs.PIEACK.all |= M_INT11;
            break;
        }

        case SET_SLOWREF: //IPC1 + IPC16
        {
            CtoMIpcRegs.MTOCIPCACK.all = SET_SLOWREF;

            if(ipc_ctom.ps_module[ipc_mtoc.msg_id].ps_status.bit.state == SlowRef)
            {
                ipc_ctom.ps_module[ipc_mtoc.msg_id].ps_setpoint =
                ipc_mtoc.ps_module[ipc_mtoc.msg_id].ps_setpoint;
            }

            else
            {
                IPC_CtoM_Msg.PSModule.ErrorMtoC = INVALID_OPMODE;
                SendIpcFlag(MTOC_MESSAGE_ERROR);
            }

            PieCtrlRegs.PIEACK.all |= M_INT11;
            break;
        }

        case SIGGEN_ENABLE: //IPC1 +IPC9
        {
            CtoMIpcRegs.MTOCIPCACK.all = SIGGEN_ENABLE;

            if(IPC_MtoC_Msg.SigGen.Enable)
            {
                Enable_ELP_SigGen(&SignalGenerator);
            }
            else
            {
                Reset_ELP_SigGen(&SignalGenerator);
                Disable_ELP_SigGen(&SignalGenerator);
            }

            PieCtrlRegs.PIEACK.all |= M_INT11;
            break;
        }

        case SIGGEN_CONFIG: //IPC1 +IPC10
        {
            CtoMIpcRegs.MTOCIPCACK.all = SIGGEN_CONFIG;

            Init_ELP_SigGen(&SignalGenerator, IPC_MtoC_Msg.SigGen.Type, IPC_MtoC_Msg.SigGen.PhaseStart, IPC_MtoC_Msg.SigGen.PhaseEnd, IPC_MtoC_Msg.SigGen.Ncycles,
                            SignalGenerator.FreqSample, SignalGenerator.ptr_FreqSignal, SignalGenerator.ptr_Amp, SignalGenerator.ptr_Offset, SignalGenerator.ptr_Aux, DP_Framework.Ref);

            PieCtrlRegs.PIEACK.all |= M_INT11;
            break;
        }

        case DPMODULES_CONFIG: //IPC1 +IPC11
        {
            CtoMIpcRegs.MTOCIPCACK.all = DPMODULES_CONFIG;

            if(!ConfigDP_Module(&DP_Framework, IPC_MtoC_Msg.DPModule.ID, IPC_MtoC_Msg.DPModule.DPclass, IPC_MtoC_Msg.DPModule.Coeffs))
            {
                IPC_CtoM_Msg.PSModule.ErrorMtoC = INVALID_DP_MODULE;
                SendIpcFlag(MTOC_MESSAGE_ERROR);
            }


            PieCtrlRegs.PIEACK.all |= M_INT11;
            break;
        }

        case SAMPLES_BUFFER_ON_OFF: //IPC1 + IPC12
        {
            CtoMIpcRegs.MTOCIPCACK.all = SAMPLES_BUFFER_ON_OFF;
            IPC_CtoM_Msg.PSModule.BufferOnOff = IPC_MtoC_Msg.PSModule.BufferOnOff;
            IPC_CtoM_Msg.SamplesBuffer.BufferBusy = (eBlockBusy) IPC_MtoC_Msg.PSModule.BufferOnOff;
            PieCtrlRegs.PIEACK.all |= M_INT11;
            break;
        }

        case RESET_WFMREF:  // IPC1 + IPC14
        {
            CtoMIpcRegs.MTOCIPCACK.all = RESET_WFMREF;
            IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferK = IPC_CtoM_Msg.WfmRef.BufferInfo.PtrBufferEnd + 1;
            DP_Framework.NetSignals[13] = 0.0;  // Use only on FBPx4
            PieCtrlRegs.PIEACK.all |= M_INT11;
            break;
        }

        case SLOWREFX4_UPDATE: //IPC1 +IPC15
        {
            CtoMIpcRegs.MTOCIPCACK.all = SLOWREFX4_UPDATE;

            if(IPC_CtoM_Msg.PSModule.OpMode == SlowRef)
            {
                //IPC_CtoM_Msg.PSModule.IRef = IPC_MtoC_Msg.PSModule.ISlowRef;
                DP_Framework.NetSignals[1] = IPC_MtoC_Msg.DPModule.Coeffs[0];
                DP_Framework.NetSignals[2] = IPC_MtoC_Msg.DPModule.Coeffs[1];
                DP_Framework.NetSignals[3] = IPC_MtoC_Msg.DPModule.Coeffs[2];
                DP_Framework.NetSignals[4] = IPC_MtoC_Msg.DPModule.Coeffs[3];

                DP_Framework.NetSignals[13]++;

            }
            else
            {
                IPC_CtoM_Msg.PSModule.ErrorMtoC = INVALID_OPMODE;
                SendIpcFlag(MTOC_MESSAGE_ERROR);
            }

            PieCtrlRegs.PIEACK.all |= M_INT11;
            break;
        }

        case HRADC_SAMPLING_DISABLE: //IPC1 +IPC28
        {
            CtoMIpcRegs.MTOCIPCACK.all = HRADC_SAMPLING_DISABLE;
            Disable_HRADC_Sampling();
            PieCtrlRegs.PIEACK.all |= M_INT11;
            break;
        }

        case HRADC_SAMPLING_ENABLE: //IPC1 +IPC29
        {
            CtoMIpcRegs.MTOCIPCACK.all = HRADC_SAMPLING_ENABLE;
            Enable_HRADC_Sampling();
            PieCtrlRegs.PIEACK.all |= M_INT11;
            break;
        }

        case HRADC_OPMODE: //IPC1 +IPC30
        {
            CtoMIpcRegs.MTOCIPCACK.all = HRADC_OPMODE;
            switch(IPC_MtoC_Msg.HRADCConfig.OpMode)
            {
                case HRADC_Sampling:
                {
                    Config_HRADC_Sampling_OpMode(IPC_MtoC_Msg.HRADCConfig.ID);
                    break;
                }
                case HRADC_UFM:
                {
                    Config_HRADC_UFM_OpMode(IPC_MtoC_Msg.HRADCConfig.ID);
                    break;
                }

                default:
                {
                    IPC_CtoM_Msg.PSModule.ErrorMtoC = INVALID_OPMODE;
                    SendIpcFlag(MTOC_MESSAGE_ERROR);
                    break;
                }
            }
            PieCtrlRegs.PIEACK.all |= M_INT11;
            break;
        }

        case HRADC_CONFIG: //IPC1 +IPC31
        {
            CtoMIpcRegs.MTOCIPCACK.all = HRADC_CONFIG;

            Config_HRADC_SoC(IPC_MtoC_Msg.HRADCConfig.FreqSampling);

            if(Try_Config_HRADC_board(HRADCs_Info.HRADC_boards[IPC_MtoC_Msg.HRADCConfig.ID],
               IPC_MtoC_Msg.HRADCConfig.InputType,
               IPC_MtoC_Msg.HRADCConfig.EnableHeater,
               IPC_MtoC_Msg.HRADCConfig.EnableMonitor))
            {
                IPC_CtoM_Msg.PSModule.ErrorMtoC = HRADC_CONFIG_ERROR;
                SendIpcFlag(MTOC_MESSAGE_ERROR);
            }

            PieCtrlRegs.PIEACK.all |= M_INT11;
            break;
        }

        case CTOM_MESSAGE_ERROR: //IPC1 +IPC32
        {
            CtoMIpcRegs.MTOCIPCACK.all = CTOM_MESSAGE_ERROR;

            PieCtrlRegs.PIEACK.all |= M_INT11;
            break;
        }

        default:
        {
            CtoMIpcRegs.MTOCIPCACK.all = 0x000000001;

            IPC_CtoM_Msg.PSModule.ErrorMtoC = IPC_LOW_PRIORITY_MSG_FULL;
            SendIpcFlag(MTOC_MESSAGE_ERROR);

            PieCtrlRegs.PIEACK.all |= M_INT11;
            break;
        }
    }
}

interrupt void isr_ipc_sync_pulse(void)
{
    uint16_t i;

    SET_DEBUG_GPIO1;

    for(i = 0; i < NUM_MAX_PS_MODULES; i++)
    {
        if(ipc_ctom.ps_module[i].ps_status.bit.active)
        {
            switch(ipc_ctom.ps_module[i].ps_status.bit.state)
            {
                case SlowRefSync:
                {
                    ipc_ctom.ps_module[i].ps_setpoint = ipc_mtoc.ps_module[i].ps_setpoint;
                    break;
                }

                case RmpWfm:
                {
                    break;
                }

                case MigWfm:
                {
                    break;
                }

                case Cycle:
                {
                    break;
                }

                default:
                {
                    break;
                }
            }
        }
    }

    CtoMIpcRegs.MTOCIPCACK.all = WFMREF_SYNC;
    PieCtrlRegs.PIEACK.all |= M_INT1;
    PieCtrlRegs.PIEACK.all |= M_INT11;

    CLEAR_DEBUG_GPIO1;
}
