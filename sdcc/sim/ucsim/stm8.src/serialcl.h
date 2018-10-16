/*
 * Simulator of microcontrollers (serialcl.h)
 *
 * Copyright (C) 1999,99 Drotos Daniel, Talker Bt.
 * 
 * To contact author send email to drdani@mazsola.iit.uni-miskolc.hu
 *
 */

/* This file is part of microcontroller simulator: ucsim.

UCSIM is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

UCSIM is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with UCSIM; see the file COPYING.  If not, write to the Free
Software Foundation, 59 Temple Place - Suite 330, Boston, MA
02111-1307, USA. */
/*@1@*/

#ifndef STM8_SERIALCL_HEADER
#define STM8_SERIALCL_HEADER

#include "fiocl.h"
#include "stypes.h"
#include "pobjcl.h"

#include "uccl.h"
#include "serial_hwcl.h"

#include "newcmdposixcl.h"


class cl_serial_listener;

class cl_serial: public cl_serial_hw
{
 private:
  static const int UART_SR_PE = 0x01;
  static const int UART_SR_FE = 0x02;
  static const int UART_SR_NF = 0x04;
  static const int UART_SR_OR = 0x08;
  static const int UART_SR_IDLE = 0x10;
  static const int UART_SR_RXNE = 0x20;
  static const int UART_SR_TC = 0x40;
  static const int UART_SR_TXE = 0x80;

  static const int UART_CR1_M = 0x10;
  static const int UART_CR1_UARTD = 0x20;

  static const int UART_CR2_REN = 0x04;
  static const int UART_CR2_TEN = 0x08;
  static const int UART_CR2_RIEN = 0x20;
  static const int UART_CR2_TCIEN = 0x40;
  static const int UART_CR2_TIEN = 0x80;

 protected:
  class cl_memory_cell
    *sr,
    *dr,
    *brr1,
    *brr2,
    *cr1,
    *cr2,
    *cr3;

  bool clk_enabled;
  t_addr base;
  int txit, rxit;
  unsigned int sample_clk, sample_div;
  unsigned int uart_clk, uart_div;
  int hw_updating;	// >0 = hw writes, ==0 = app writes
  bool    sr_read;	// last op was read of SR
  u8_t s_out;	// Serial channel output reg
  u8_t s_txd;	// TX data register
  bool    s_sending;	// Transmitter is working (s_out is not empty)
  bool    s_receiving;	// Receiver is working (s_in is shifting)
  int     s_rec_bit;	// Bit clock counter of receiver
  int     s_tr_bit;	// Bit clock counter of transmitter
  uchar   bitclkmax;	// Nr of cycles it takes to send/receive one character
 public:
  cl_serial(class cl_uc *auc,
	    t_addr abase,
	    int aid, int atxit, int arxit);
  virtual ~cl_serial(void);
  virtual int init(void);
  virtual int cfg_size(void) { return 10; }

  virtual void new_hw_added(class cl_hw *new_hw);
  virtual void added_to_uc(void);
  virtual t_mem read(class cl_memory_cell *cell);
  virtual void write(class cl_memory_cell *cell, t_mem *val);
  virtual t_mem conf_op(cl_memory_cell *cell, t_addr addr, t_mem *val);

  virtual int tick(int cycles);
  virtual void reset(void);
  virtual void happen(class cl_hw *where, enum hw_event he,
                      void *params);

  virtual void print_info(class cl_console_base *con);
};


#endif

/* End of stm8.src/serialcl.h */
