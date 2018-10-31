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
#include "sci/sci.h"


#pragma CODE_SECTION(send_udc_net_cmd, "ramfuncs");
#pragma CODE_SECTION(get_status_udc_net, "ramfuncs");
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
void init_udc_net( uint16_t node_type, uint16_t add,
                   void (*p_process_data)(void) )
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
    udc_net_msg_t msg;
    uint16_t i;

    g_udc_net.send_msg.bit.add = add;
    g_udc_net.send_msg.bit.cmd = cmd;
    g_udc_net.send_msg.bit.data = data;
    g_udc_net.send_msg.bit.checksum -= (g_udc_net.send_msg.u16[0] +
                                        g_udc_net.send_msg.u16[1]);

    SciaRegs.SCITXBUF = g_udc_net.send_msg.u16[0];
    SciaRegs.SCITXBUF = g_udc_net.send_msg.u16[1];
    SciaRegs.SCITXBUF = g_udc_net.send_msg.u16[2];
}

inline void set_interlock_udc_net(void)
{
    SciaRegs.SCITXBUF = 0x00F2;
    SciaRegs.SCITXBUF = g_udc_net.add;
    SciaRegs.SCITXBUF = 0x000E - g_udc_net.add;
}

inline void reset_interlock_udc_net(void)
{
    SciaRegs.SCITXBUF = 0x00F3;
    SciaRegs.SCITXBUF = 0x0000;
    SciaRegs.SCITXBUF = 0x000D;
}

inline void get_status_udc_net(uint16_t add)
{
    SciaRegs.SCITXBUF = 0x0004 | (add << 4);
    SciaRegs.SCITXBUF = 0x0000;
    SciaRegs.SCITXBUF = 0x00FC - (add << 4);
}

interrupt void isr_sci_rx_fifo_udc_net(void)
{
    Uint16 i, sum;

    g_udc_net.recv_msg.u16[0] = SciaRegs.SCIRXBUF.all;
    g_udc_net.recv_msg.u16[1] = SciaRegs.SCIRXBUF.all;
    g_udc_net.recv_msg.u16[2] = SciaRegs.SCIRXBUF.all;

    sum = g_udc_net.recv_msg.u16[0];
    sum += g_udc_net.recv_msg.u16[1];
    sum += g_udc_net.recv_msg.u16[2];

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


static void process_data_udc_net_slave(void)
{
    switch(g_udc_net.recv_msg.bit.cmd)
    {
        case Turn_On_UDC_Net:
        {
            turn_on(0);
            break;
        }

        case Turn_Off_UDC_Net:
        {
            turn_off(0);
            break;
        }

        case Set_Interlock_UDC_Net:
        {
            set_hard_interlock(0, DRS_Master_Interlock +
                                  g_udc_net.recv_msg.bit.data);
            break;
        }

        case Reset_Interlock_UDC_Net:
        {
            reset_interlocks(0);
            break;
        }

        case Get_Status_UDC_Net:
        {
            if(g_ipc_ctom.ps_module[0].ps_status.bit.state == Interlock)
            {
                set_interlock_udc_net();
            }
            else
            {
            send_udc_net_cmd( 0, Get_Status_UDC_Net,
                              (uint16_t) g_ipc_ctom.ps_module[0].ps_status.all );
            }

            break;
        }

        default:
        {
            break;
        }
    }

    g_udc_net.ps_status_nodes[g_udc_net.add] =
                                      (ps_status_t) g_udc_net.recv_msg.bit.data;

}

static void process_data_udc_net_master(void)
{
    uint16_t last_send_add;

    last_send_add = g_udc_net.send_msg.bit.add;

    switch(g_udc_net.recv_msg.bit.cmd)
    {
        case Set_Interlock_UDC_Net:
        {
            set_hard_interlock(0, DRS_Master_Interlock +
                                  g_udc_net.recv_msg.bit.data);
            break;
        }

        case Get_Status_UDC_Net:
        {
            g_udc_net.ps_status_nodes[last_send_add] =
                                      (ps_status_t) g_udc_net.recv_msg.bit.data;

            if(g_udc_net.ps_status_nodes[last_send_add].bit.state == Interlock)
            {
                set_hard_interlock(0, DRS_Master_Interlock + last_send_add);
            }

            break;
        }

        default:
        {
            break;
        }
    }
}
