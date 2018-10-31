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
 * @file sci.c
 * @brief SCI module
 * 
 * This module is responsible for configuration and operation of SCI interface
 * available on the BCB board.
 *
 * @author gabriel.brunheira
 * @date 23/10/2018
 *
 */

#include "sci.h"

/**
 *
 *
 * @param baudrate
 * @param rxfifo_lvl
 */
void init_sci(float baudrate, uint16_t rxfifo_lvl)
{
    /**
     *  SCICCR Register:
     *    Character length 8 bits
     *    1 stop bit
     *    Parity and loopback not enabled
     *    Idle-line mode protocol selected
     */
    SciaRegs.SCICCR.bit.SCICHAR = (0x0008-1);
    SciaRegs.SCICCR.bit.STOPBITS = 0;
    SciaRegs.SCICCR.bit.PARITYENA = 0;
    SciaRegs.SCICCR.bit.LOOPBKENA = 0;
    SciaRegs.SCICCR.bit.ADDRIDLE_MODE = 0;

    /**
     *  SCICTL1 Register:
     *    Enable RX and TX
     *    Leave SLEEP mode, TXWAKE feature and RX Err Int disabled
     *
     *  SCICTL2 Register:
     *    Disable TX and RXRDY/BRKDT interrupts
     */
    SciaRegs.SCICTL1.bit.RXENA = 1;
    SciaRegs.SCICTL1.bit.TXENA = 1;
    SciaRegs.SCICTL1.bit.SLEEP = 0;
    SciaRegs.SCICTL1.bit.TXWAKE = 0;
    SciaRegs.SCICTL1.bit.RXERRINTENA = 0;

    SciaRegs.SCICTL2.bit.TXINTENA = 0;
    SciaRegs.SCICTL2.bit.RXBKINTENA = 0;

    /**
     * Baud-rate selection
     */
    SciaRegs.SCIHBAUD = 0x0000;
    SciaRegs.SCILBAUD = LSPCLK_FREQ/(baudrate * 8) - 1;

    /**
     * FIFO TX
     *  Leave transmit FIFO interrupt disabled
     *  Enable SCI FIFO enhancements
     *  Transmit FIFO reset
     *
     * FIFO RX
     *  Receive FIFO reset
     *  4-level receive FIFO
     *  Enable receive FIFO interrupt
     */
    SciaRegs.SCIFFTX.all = 0xC000;
    SciaRegs.SCIFFRX.all = 0x0020;
    SciaRegs.SCIFFRX.bit.RXFFIL = rxfifo_lvl;

    /**
     *  Release the SCI from reset
     *  Release the FIFOs from reset
     */
    SciaRegs.SCICTL1.all = 0x0023;
    SciaRegs.SCIFFTX.bit.TXFIFOXRESET = 1;
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;
}

void init_sci_rx_fifo_interrupt(void (*p_isr_sci_rx_fifo)(void))
{
    EALLOW;
    PieVectTable.SCIRXINTA = p_isr_sci_rx_fifo;
    EDIS;

    SciaRegs.SCIFFRX.bit.RXFFIENA = 1;
}
