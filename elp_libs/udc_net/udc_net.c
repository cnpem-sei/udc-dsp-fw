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

#pragma CODE_SECTION(send_udc_net_cmd, "ramfuncs");
//#pragma CODE_SECTION(get_status_udc_net, "ramfuncs");
#pragma CODE_SECTION(isr_sci_rx_fifo_udc_net, "ramfuncs");

interrupt void isr_sci_rx_fifo_udc_net(void);

volatile udc_net_t g_udc_net;

/**
 * Initialization of UDC Net module
 *
 * @param node_type
 * @param add
 * @param p_process_data
 */
void init_udc_net(uint16_t add, void (*p_process_data)(void))
{
    uint16_t i;

    init_sci(UDC_NET_BAUDRATE, SIZE_SCI_FIFO);

    g_udc_net.add = add;
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

    init_sci_rx_fifo_interrupt(&isr_sci_rx_fifo_udc_net);
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
