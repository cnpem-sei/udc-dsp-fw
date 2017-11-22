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

/**
 * TODO: Put here your defines. Just what is local. If you don't
 * need to access it from other module, consider use a constant (const)
 */


/**
 * TODO: Put here your constants and variables. Always use static for 
 * private members.
 */

/**
 * TODO: Put here your function prototypes for private functions. Use
 * static in declaration.
 */


/**
 * TODO: Put here the implementation for your public functions.
 */


/**
 * Initialization of interprocessor communication (IPC)
 *
 * @param p_ipc_ctom CtoM IPC struct (C28 to ARM)
 */
void init_ipc(ipc_ctom_t *p_ipc_ctom)
{
    EALLOW;

    /**
     * TODO: move INT_GENERAL/EPWMSYNCI initializations to wfmref module
     */

    /**
     * Set synchronization output EPWMSYNCO
     *
     * TODO: move this to pwm_module
     */

    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 2; // Configures GPIO33 for EPWMSYNCO

    /* Map IPC_MtoC interrupts */

    PieVectTable.XINT2        = &isr_ipc_sync_pulse;
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

    p_ipc_ctom->error_ctom = No_Error_CtoM;
}

/**
 * TODO: Put here the implementation for your private functions.
 */
interrupt void isr_ipc_lowpriority_msg(void)
{

}

interrupt void isr_ipc_sync_pulse(void)
{

}
