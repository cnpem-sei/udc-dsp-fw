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
 * @file sci.h
 * @brief SCI module
 * 
 * This module is responsible for configuration and operation of SCI interface
 * available on the BCB board.
 *
 * @author gabriel.brunheira
 * @date 23/10/2018
 *
 */

#ifndef SCI_H_
#define SCI_H_

#include "boards/udc_c28.h"

extern void init_sci(float baudrate, uint16_t rxfifo_lvl);
extern void init_sci_rx_fifo_interrupt(void (*p_isr_sci_rx_fifo)(void));

#endif /* SCI_H_ */
