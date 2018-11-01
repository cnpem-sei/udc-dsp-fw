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
 * @file udc_net.h
 * @brief UDC Net module
 * 
 * This module is responsible for the implementation of UDC Net, a dedicated
 * RS-485 network among UDC boards.
 *
 * @author gabriel.brunheira
 * @date 30/10/2018
 *
 */


#ifndef UDC_NET_H_
#define UDC_NET_H_

#include "boards/udc_c28.h"
#include "ps_modules/ps_modules.h"
#include "sci/sci.h"

#define SIZE_SCI_FIFO           3

#define UDC_NET_BAUDRATE        5000000

#define UDC_NET_MASTER          0
#define UDC_NET_SLAVE           1

#define UDC_NET_BROADCAST_ADD   0xF

#define NUM_MAX_UDC_NET_NODES   5

typedef enum
{
    Turn_On_UDC_Net,
    Turn_Off_UDC_Net,
    Set_Interlock_UDC_Net,
    Reset_Interlock_UDC_Net,
    Get_Status_UDC_Net
} udc_net_cmd_t;

typedef struct
{
    uint16_t    cmd         : 4;
    uint16_t    add         : 4;
    uint16_t    reserved1   : 8;
    uint16_t    data        : 8;
    uint16_t    reserved2   : 8;
    uint16_t    checksum    : 8;
    uint16_t    reserved3   : 8;
} udc_net_msg_bit_t;

typedef union
{
    uint16_t            u16[SIZE_SCI_FIFO];
    udc_net_msg_bit_t   bit;
} udc_net_msg_t;

typedef struct
{
    uint16_t        add;
    udc_net_msg_t   send_msg;
    udc_net_msg_t   recv_msg;
    void            (*p_process_data)(void);
    ps_status_t     ps_status_nodes[NUM_MAX_UDC_NET_NODES];
} udc_net_t;

extern volatile udc_net_t g_udc_net;

extern void init_udc_net(uint16_t add, void (*p_process_data)(void));

extern void send_udc_net_cmd(uint16_t add, uint16_t cmd, uint16_t data);


inline void set_interlock_udc_net(void)
{
    SET_DEBUG_GPIO1;
    SET_SCI_RD;
    SciaRegs.SCITXBUF = 0x00F2;
    SciaRegs.SCITXBUF = g_udc_net.add;
    SciaRegs.SCITXBUF = 0x000E - g_udc_net.add;
    //RESET_SCI_RD;
}

inline void reset_interlock_udc_net(void)
{
    SET_DEBUG_GPIO1;
    SET_SCI_RD;
    SciaRegs.SCITXBUF = 0x00F3;
    SciaRegs.SCITXBUF = 0x0000;
    SciaRegs.SCITXBUF = 0x000D;
    //RESET_SCI_RD;
}

inline void get_status_udc_net(uint16_t add)
{
    SET_DEBUG_GPIO1;
    SET_SCI_RD;
    SciaRegs.SCITXBUF = 0x0004 | (add << 4);
    SciaRegs.SCITXBUF = 0x0000;
    SciaRegs.SCITXBUF = 0x00FC - (add << 4);
    //RESET_SCI_RD;
}

#endif /* UDC_NET_H_ */
