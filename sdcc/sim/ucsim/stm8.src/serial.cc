/*
 * Simulator of microcontrollers (serial.cc)
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

#include "ddconfig.h"

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/time.h>
#include <strings.h>

// prj
#include "globals.h"
#include "utils.h"

// cmd
#include "cmdutil.h"

// sim
#include "itsrccl.h"

// local
#include "clkcl.h"
#include "serialcl.h"


enum reg_idx {
  sr	= 0,
  dr	= 1,
  brr1	= 2,
  brr2	= 3,
  cr1	= 4,
  cr2	= 5,
  cr3	= 6,
  cr4	= 7,
  cr5	= 8,
  cr6	= 9,
  gtr	= 10,
  pscr	= 11
};


cl_serial::cl_serial(class cl_uc *auc,
		     t_addr abase,
		     int aid, int atxit, int arxit):
  cl_serial_hw(auc, aid, "uart")
{
  base= abase;
  txit= atxit;
  rxit= arxit;
}


cl_serial::~cl_serial(void)
{
  hw_updating = 0;
}

int
cl_serial::init(void)
{
  class cl_it_src *is;

  cl_serial_hw::init();

  if (uc->type->subtype & (DEV_STM8S003|
                           DEV_STM8S007|
                           DEV_STM8S103|
                           DEV_STM8S207|
                           DEV_STM8S208|
                           DEV_STM8S903|
                           DEV_STM8AF52))
    {
      chars s("uart");
      s.append("%d", id);
      set_name(s);
    }
  else
    {
      chars s("usart");
      s.append("%d", id);
      set_name(s);
    }

  clk_enabled= false;

  hwreg(uc->rom, base+0, "SR", "Status register",
    "TXE",    7, 7, "Transmit data register empty",
    "TC",     6, 6, "Transmit complete",
    "RXNE",   5, 5, "Read data register not empty",
    "IDLE",   4, 4, "IDLE line detected",
    "OR_LHE", 3, 3, "Overrun error / LIN header error",
    "NF",     2, 2, "Noise flag",
    "FE",     1, 1, "Framing error",
    "PE",     0, 0, "Parity error",
    NULL);
  sr = register_cell(uc->rom, base+0);

  hwreg(uc->rom, base+1, "DR", "Data register", NULL);
  dr = register_cell(uc->rom, base+1);

  hwreg(uc->rom, base+2, "BRR1", "Baud rate register 1", NULL);
  brr1 = register_cell(uc->rom, base+2);

  hwreg(uc->rom, base+3, "BRR2", "Baud rate register 2", NULL);
  brr2 = register_cell(uc->rom, base+3);

  hwreg(uc->rom, base+4, "CR1", "Control register 1",
    "R8",    7, 7, "Receive data bit 8",
    "T8",    6, 6, "Transmit data bit 8",
    "UARTD", 5, 5, "UART disable (for low power consumption)",
    "M",     4, 4, "Word length",
    "WAKE",  3, 3, "Wakeup method",
    "PCEN",  2, 2, "Parity control enable",
    "PS",    1, 1, "Parity selection",
    "PIEN",  0, 0, "Parity interrupt enable",
    NULL);
  cr1 = register_cell(uc->rom, base+4);

  hwreg(uc->rom, base+5, "CR2", "Control register 2",
    "TIEN",  7, 7, "Transmitter interrupt enable",
    "TCIEN", 6, 6, "Transmission complete interrupt enable",
    "RIEN",  5, 5, "Receiver interrupt enable",
    "ILIEN", 4, 4, "IDLE line interrupt enable",
    "TEN",   3, 3, "Transmitter enable",
    "REN",   2, 2, "Receiver enable",
    "RWU",   1, 1, "Receiver wakeup",
    "SBRK",  0, 0, "Send break",
    NULL);
  cr2 = register_cell(uc->rom, base+5);

  hwreg(uc->rom, base+6, "CR3", "Control register 3",
    "LINEN", 6, 6, "LIN mode enable",
    "STOP",  5, 4, "STOP bits",
    "CLKEN", 3, 3, "Clock enable",
    "CPOL",  2, 2, "Clock polarity",
    "CPHA",  1, 1, "Clock phase",
    "LBCL",  0, 0, "Last bit clock pulse",
    NULL);
  cr3 = register_cell(uc->rom, base+6);

  hwreg(uc->rom, base+7, "CR4", "Control register 4",
    "LBDIEN", 6, 6, "LIN Break Detection Interrupt Enable",
    "LBDL",   5, 5, "LIN Break Detection Length",
    "LBDF",   4, 4, "LIN Break Detection Flag",
    "ADD",    3, 0, "Address of the UART node",
    NULL);
  //cr4 = register_cell(uc->rom, base+7);

  hwreg(uc->rom, base+8, "CR5", "Control register 5",
    "SCEN",  5, 5, "Smartcard mode enable",
    "NACK",  4, 4, "Smartcard NACK enable",
    "HDSEL", 3, 3, "Half-Duplex Selection",
    "IRLP",  2, 2, "IrDA Low Power",
    "IREN",  1, 1, "IrDA mode enable",
    NULL);
  //cr5 = register_cell(uc->rom, base+8);

  hwreg(uc->rom, base+9, "CR6", "Control register 6",
    "LDUM",   7, 7, "LIN Divider Update Method",
    "LSLV",   5, 5, "LIN Slave Enable",
    "LASE",   4, 4, "LIN automatic resynchronization enable",
    "LHDIEN", 2, 2, "LIN Header Detection Interrupt Enable",
    "LHDF",   1, 1, "LIN Header Detection Flag",
    "LSF",    0, 0, "LIN Sync Field",
    NULL);
  //cr6 = register_cell(uc->rom, base+9);

  hwreg(uc->rom, base+10, "GTR", "Guard time register",
    "GT", 7, 0, "Guard time value",
    NULL);
  //gtr = register_cell(uc->rom, base+10);

  hwreg(uc->rom, base+11, "PSCR", "Prescalar register",
    "PSC", 7, 0, "Prescalar value",
    NULL);
  //pscr = register_cell(uc->rom, base+11);


  uc->it_sources->add(is= new cl_it_src(uc, txit,
					cr2, UART_CR2_TIEN,
					sr, UART_SR_TXE,
					0x8008+txit*4, false, false,
					chars("", "usart%d transmit register empty", id), 20*10+1));
  is->init();
  uc->it_sources->add(is= new cl_it_src(uc, txit,
					cr2, UART_CR2_TCIEN,
					sr, UART_SR_TC,
					0x8008+txit*4, false, false,
					chars("", "usart%d transmit complete", id), 20*10+2));
  is->init();
  uc->it_sources->add(is= new cl_it_src(uc, rxit,
					cr2, UART_CR2_RIEN,
					sr, UART_SR_RXNE,
					0x8008+rxit*4, false, false,
					chars("", "usart%d receive", id), 20*10+3));
  is->init();

  sr_read= false;

  return(0);
}


void
cl_serial::new_hw_added(class cl_hw *new_hw)
{
}

void
cl_serial::added_to_uc(void)
{
}

t_mem
cl_serial::read(class cl_memory_cell *cell)
{
  if (cell == dr)
    {
      hw_updating++;
      if (sr_read)
	sr->write_bit0(UART_SR_IDLE | UART_SR_OR | UART_SR_NF | UART_SR_FE | UART_SR_PE);
      sr->write_bit0(UART_SR_RXNE);
      cfg_set(serconf_able_receive, 1);
      hw_updating--;
      return dr->get();
    }
  sr_read= (cell == sr);
  conf(cell, NULL);
  return cell->get();
}

void
cl_serial::write(class cl_memory_cell *cell, t_mem *val)
{
  if (conf(cell, val))
    return;

  if (cell == sr)
    {
      if (!hw_updating)
        {
          // The only change allowed is to set TC to 0
          // FIXME: RXNE can be set to 0 for UART2 and UART3
          if ((*val & UART_SR_TC) == 0)
	      *val= cell->get() & (~UART_SR_TC);
        }
    }
  else if (cell == brr1)
    {
      u8_t b1= brr1->get();
      u8_t b2= brr2->get();
      uart_div= ((b2 & 0xf0) << 8) + (b1 << 4) + (b2 & 0xf);
      // Specs say UART_DIV must be always be greater than or equal to 16.
      if (uart_div < 16)
        uart_div= 16;
      sample_div= uart_div >> 4;
      sample_clk= 0;
      uart_clk= 0;
    }
  else if (cell == cr1 || cell == cr3)
    {
      if (*val & UART_CR1_M)
        bitclkmax = 2 * (1 + 9 + 1);
      else
        {
          bitclkmax = 2 * (1 + 8);
	  switch ((cr3->get() >> 4) & 0x03) {
            case 0x00:
            case 0x01:
              bitclkmax += 2;
	      break;
            case 0x02:
              bitclkmax += 4;
	      break;
            case 0x03:
              bitclkmax += 3;
	      break;
	  }
        }
    }
  else if (cell == cr2)
    {
      t_mem cr2_val = cr2->get();

      if (!(cr2_val & UART_CR2_TEN) && (*val & UART_CR2_TEN))
        {
	  // UM 22.3.2: An idle frame will be sent after the TEN bit is enabled.
          // UM 22.7.6: When TEN is set there is a 1 bit-time delay before the transmission starts.
          s_tr_bit = -(bitclkmax + 1);
        }
      if ((cr2_val & UART_CR2_REN) && !(*val & UART_CR2_REN))
        {
          // If the REN bit is reset the reception of the current character is aborted.
	  s_receiving= 0;
	  s_rec_bit= 0;
        }
    }
  else if (cell == dr && !hw_updating)
    {
      hw_updating++;
      if (sr_read)
        sr->write_bit0(UART_SR_TC);

      if (!s_sending && (cr2->get() & UART_CR2_TEN))
	{
          s_out= *val;
          s_sending= true;
	  // TXE should already be set but we need to do this to generate an interrupt.
          sr->write_bit1(UART_SR_TXE);
	}
      else
	{
          s_txd= *val;
          sr->write_bit0(UART_SR_TXE);
	}

      hw_updating--;
    }

  sr_read= false;
}

t_mem
cl_serial::conf_op(cl_memory_cell *cell, t_addr addr, t_mem *val)
{
  if (addr < serconf_common)
    return cl_serial_hw::conf_op(cell, addr, val);
  switch ((enum serial_cfg)addr)
    {
      /*
    case serial_:
      if (val)
	{
	  if (*val)
	    on= true;
	  else
	    on= false;
	}
      else
	{
	  cell->write(on?1:0);
	}
      break;
      */
    default:
      break;
    }
  return cell->get();
}

int
cl_serial::tick(int cycles)
{
  if (!clk_enabled)
    return 0;

  for (sample_clk += cycles; sample_clk >= sample_div; sample_clk -= sample_div)
    {
      hw_updating++;

      uart_clk += sample_div;

      if (uart_clk >= uart_div)
        {
          uart_clk -= uart_div;

          if (!(cr1->get() & UART_CR1_UARTD))
            {
              if (s_sending && (cr2->get() & UART_CR2_TEN) && ++s_tr_bit >= bitclkmax)
                {
                  io->dd_printf("%c", s_out);
                  if (!(sr->get() & UART_SR_TXE))
                    {
                      s_tr_bit-= bitclkmax;
                      s_out= s_txd;
                      s_sending= true;
                      sr->write_bit1(UART_SR_TXE);
                    }
                  else
                    {
                      s_tr_bit= 0;
                      s_sending= false;
                      sr->write_bit1(UART_SR_TC);
                    }
                }

              if (s_receiving && (cr2->get() & UART_CR2_REN) && ++s_rec_bit >= bitclkmax)
                {
                  dr->write(input);
                  cfg_write(serconf_received, input);
                  input_avail= false;
                  s_receiving= false;
                  s_rec_bit-= bitclkmax;

                  if (!(sr->get() & UART_SR_RXNE))
                    sr->write_bit1(UART_SR_RXNE);
                  else
                    sr->write_bit1(UART_SR_OR);
                }
            }
        }

      // Start bits can begin on any sample tick rather than a baud clock tick
      // so we check for more input every sample. But _after_ we have checked
      // for a RX completion on a baud clock tick otherwise we will introduce
      // cycle delays between back-to-back frames that shouldn't be there.
      if (!(cr1->get() & UART_CR1_UARTD) && !s_receiving && io->get_fin())
        {
          if (cfg_get(serconf_check_often))
            {
              if (io->input_avail())
                io->proc_input(0);
            }

          if (input_avail)
            s_receiving= true;
          else if (!(sr->get() & UART_SR_IDLE))
            {
              s_rec_bit= 0;
              sr->write_bit1(UART_SR_IDLE);
            }
        }

      hw_updating--;
    }

  return(0);
}

void
cl_serial::reset(void)
{
  hw_updating++;

  sr->write(UART_SR_TXE | UART_SR_TC);
  dr->write(0);
  brr1->write(0);
  brr2->write(0);
  cr1->write(0);
  cr2->write(0);
  cr3->write(0);

  hw_updating--;

  s_rec_bit= s_tr_bit= 0;
  s_receiving= false;
}

void
cl_serial::happen(class cl_hw *where, enum hw_event he,
		  void *params)
{
  if ((he == EV_CLK_ON) ||
      (he == EV_CLK_OFF))
    {
      cl_clk_event *e= (cl_clk_event *)params;
      if ((e->cath == HW_UART) &&
	  (e->id == id))
	clk_enabled= he == EV_CLK_ON;
    }
}

void
cl_serial::print_info(class cl_console_base *con)
{
  con->dd_printf("%s[%d] at 0x%06x %s\n", id_string, id, base, ((cr1->get() & UART_CR1_UARTD) ? "off" : "on"));
  con->dd_printf("clk %s\n", clk_enabled?"enabled":"disabled");
  con->dd_printf("Input: ");
  class cl_f *fin= io->get_fin(), *fout= io->get_fout();
  if (fin)
    con->dd_printf("%s/%d ", fin->get_file_name(), fin->file_id);
  con->dd_printf("Output: ");
  if (fout)
    con->dd_printf("%s/%d", fout->get_file_name(), fout->file_id);
  con->dd_printf("\n");
  print_cfg_info(con);
}


/* End of stm8.src/serial.cc */
