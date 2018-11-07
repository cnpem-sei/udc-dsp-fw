/******************************************************************************
 * Copyright (C) 2018 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file udc_net.c
 * @brief UDC Net module
 * 
 * This module is responsible for the implementation of UDC Net, a dedicated
 * RS-485 network among UDC boards.s
 *
 * @author gabriel.brunheira
 * @date 30/10/2018
 *
 */

#include "udc_net.h"
#include "ipc/ipc.h"
#include "event_manager/event_manager.h"

#pragma CODE_SECTION(send_udc_net_cmd, "ramfuncs");
#pragma CODE_SECTION(isr_sci_rx_fifo_udc_net, "ramfuncs");
#pragma CODE_SECTION(isr_udc_net_tx_end, "ramfuncs");
#pragma CODE_SECTION(isr_udc_net_slave_timeout, "ramfuncs");

interrupt void isr_sci_rx_fifo_udc_net(void);
interrupt void isr_udc_net_tx_end(void);
interrupt void isr_udc_net_slave_timeout(void);

volatile udc_net_t g_udc_net;

/**
 * Initialization of UDC Net module
 *
 * @param node_type
 * @param add
 * @param p_process_data
 */
void init_udc_net(uint16_t add, uint16_t node_type, void (*p_process_data)(void))
{
    uint16_t i;

    g_udc_net.add = add;
    g_udc_net.node_type = node_type;
    g_udc_net.enable_tx = 1;
    g_udc_net.p_process_data = p_process_data;

    for(i = 0; i < SIZE_SCI_FIFO; i ++)
    {
        g_udc_net.send_msg.u16[i] = 0;
        g_udc_net.recv_msg.u16[i] = 0;
    }

    for(i = 0; i < NUM_MAX_UDC_NET_NODES; i++)
    {
        g_udc_net.ps_status_nodes[i].all = 0;
    }

    /// Initialize SCI driver
    init_sci(UDC_NET_BAUDRATE, SIZE_SCI_FIFO);
    init_sci_rx_fifo_interrupt(&isr_sci_rx_fifo_udc_net);

    /// Initialize CpuTimer0
    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, C28_FREQ_MHZ, TIMEOUT_SCI_RD_US);
    CpuTimer0Regs.TCR.bit.TIE = 0;

    EALLOW;
    PieVectTable.TINT0 = &isr_udc_net_tx_end;
    EDIS;
}

void send_udc_net_cmd(uint16_t add, uint16_t cmd, uint16_t data)
{
    g_udc_net.send_msg.bit.checksum = 0;
    g_udc_net.send_msg.bit.add = add;
    g_udc_net.send_msg.bit.cmd = cmd;
    g_udc_net.send_msg.bit.data = data;
    g_udc_net.send_msg.bit.checksum -= (g_udc_net.send_msg.u16[0] +
                                        g_udc_net.send_msg.u16[1]);

    SET_DEBUG_GPIO1;
    SET_SCI_RD;
    SciaRegs.SCITXBUF = g_udc_net.send_msg.u16[0];
    SciaRegs.SCITXBUF = g_udc_net.send_msg.u16[1];
    SciaRegs.SCITXBUF = g_udc_net.send_msg.u16[2];
    CLEAR_DEBUG_GPIO1;
    //RESET_SCI_RD;
}

/**
 * TODO: reply for invalid checksum
 */
interrupt void isr_sci_rx_fifo_udc_net(void)
{
    Uint16 sum;

    g_udc_net.recv_msg.u16[0] = SciaRegs.SCIRXBUF.all;
    g_udc_net.recv_msg.u16[1] = SciaRegs.SCIRXBUF.all;
    g_udc_net.recv_msg.u16[2] = SciaRegs.SCIRXBUF.all;

    sum = g_udc_net.recv_msg.u16[0];
    sum += g_udc_net.recv_msg.u16[1];
    sum += g_udc_net.recv_msg.u16[2];
    sum &= 0x00FF;

    if(!sum)
    {
        if( (g_udc_net.recv_msg.bit.add == g_udc_net.add) ||
            (g_udc_net.recv_msg.bit.add == UDC_NET_BROADCAST_ADD) )
        {
            g_udc_net.p_process_data();
        }
    }


    // Clear Interrupt flag
    // Issue PIE acknowledge to enable more interrupts from this group
    SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;
    PieCtrlRegs.PIEACK.all |= M_INT9;
}

/**
 *
 */
interrupt void isr_udc_net_tx_end(void)
{
    RESET_SCI_RD;

    /// Clear interrupt flag and disable timer
    CpuTimer0Regs.TCR.all = 0xC010;

    if(g_udc_net.node_type = UDC_NET_MASTER)
    {
        SET_DEBUG_GPIO1;
        CpuTimer0Regs.PRD.all = TIMEOUT_SCI_SLAVE_SYSCLK;

        EALLOW;
        PieVectTable.TINT0 = &isr_udc_net_slave_timeout;
        EDIS;

        /// Reload period and enable timer
        CpuTimer0Regs.TCR.all = 0x4020;
    }

    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP1;
}

/**
 *
 */
interrupt void isr_udc_net_slave_timeout(void)
{
    /// LIMPAR BUFFER
    /// SETAR INTERLOCK
    /// SETAR TIMER
    /// HABILITAR TX
    ///
    if(g_udc_net.node_type = UDC_NET_MASTER)
    {
        SET_DEBUG_GPIO1;
        /// Clear interrupt flag and disable timer
        CpuTimer0Regs.TCR.all = 0xC010;
        CpuTimer0Regs.PRD.all = TIMEOUT_SCI_RD_SYSCLK;

        EALLOW;
        PieVectTable.TINT0 = &isr_udc_net_tx_end;
        EDIS;

        ///LIMPAR BUFFER FIFO
        ///set_hard_interlock(id, Slave)
        g_udc_net.enable_tx = 1;

        CLEAR_DEBUG_GPIO1;

        PieCtrlRegs.PIEACK.all |= PIEACK_GROUP1;
    }
}
