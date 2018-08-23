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
		     int ttype, int atxit, int arxit):
  cl_serial_hw(auc, ttype, "uart")
{
  type= ttype;
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
  int i;
  class cl_it_src *is;
  
  set_name("stm8_uart");
  cl_serial_hw::init();
  clk_enabled= false;
  for (i= 0; i < 12; i++)
    {
      regs[i]= register_cell(uc->rom, base+i);
    }

  uc->it_sources->add(is= new cl_it_src(uc, txit,
					regs[cr2], UART_CR2_TIEN,
					regs[sr], UART_SR_TXE,
					0x8008+txit*4, false, false,
					chars("", "usart%d transmit register empty", id), 20*10+1));
  is->init();
  uc->it_sources->add(is= new cl_it_src(uc, txit,
					regs[cr2], UART_CR2_TCIEN,
					regs[sr], UART_SR_TC,
					0x8008+txit*4, false, false,
					chars("", "usart%d transmit complete", id), 20*10+2));
  is->init();
  uc->it_sources->add(is= new cl_it_src(uc, rxit,
					regs[cr2], UART_CR2_RIEN,
					regs[sr], UART_SR_RXNE,
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
  if (cell == regs[dr])
    {
      hw_updating++;
      if (sr_read)
	regs[sr]->write_bit0(UART_SR_IDLE | UART_SR_OR | UART_SR_NF | UART_SR_FE | UART_SR_PE);
      regs[sr]->write_bit0(UART_SR_RXNE);
      cfg_set(serconf_able_receive, 1);
      hw_updating--;
      return regs[dr]->get();
    }
  sr_read= (cell == regs[sr]);
  conf(cell, NULL);
  return cell->get();
}

void
cl_serial::write(class cl_memory_cell *cell, t_mem *val)
{
  if (conf(cell, val))
    return;

  if (cell == regs[sr])
    {
      if (!hw_updating)
        {
          // The only change allowed is to set TC to 0
          // FIXME: RXNE can be set to 0 for UART2 and UART3
          if ((*val & UART_SR_TC) == 0)
	      *val= cell->get() & (~UART_SR_TC);
        }
    }
  else if (cell == regs[brr1])
    {
      u8_t b1= regs[brr1]->get();
      u8_t b2= regs[brr2]->get();
      uart_div= ((b2 & 0xf0) << 8) + (b1 << 4) + (b2 & 0xf);
      // Specs say UART_DIV must be always be greater than or equal to 16.
      if (uart_div < 16)
        uart_div= 16;
      sample_div= uart_div >> 4;
      sample_clk= 0;
      uart_clk= 0;
    }
  else if (cell == regs[cr1] || cell == regs[cr3])
    {
      if (*val & UART_CR1_M)
        bitclkmax = 2 * (1 + 9 + 1);
      else
        {
          bitclkmax = 2 * (1 + 8);
	  switch ((regs[cr3]->get() >> 4) & 0x03) {
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
  else if (cell == regs[cr2])
    {
      t_mem cr2_val = regs[cr2]->get();

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
  else if (cell == regs[dr] && !hw_updating)
    {
      hw_updating++;
      if (sr_read)
        regs[sr]->write_bit0(UART_SR_TC);

      if (!s_sending && (regs[cr2]->get() & UART_CR2_TEN))
	{
          s_out= *val;
          s_sending= true;
	  // TXE should already be set but we need to do this to generate an interrupt.
          regs[sr]->write_bit1(UART_SR_TXE);
	}
      else
	{
          s_txd= *val;
          regs[sr]->write_bit0(UART_SR_TXE);
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

          if (!(regs[cr1]->get() & UART_CR1_UARTD))
            {
              if (s_sending && (regs[cr2]->get() & UART_CR2_TEN) && ++s_tr_bit >= bitclkmax)
                {
                  io->dd_printf("%c", s_out);
                  if (!(regs[sr]->get() & UART_SR_TXE))
                    {
                      s_tr_bit-= bitclkmax;
                      s_out= s_txd;
                      s_sending= true;
                      regs[sr]->write_bit1(UART_SR_TXE);
                    }
                  else
                    {
                      s_tr_bit= 0;
                      s_sending= false;
                      regs[sr]->write_bit1(UART_SR_TC);
                    }
                }

              if (s_receiving && (regs[cr2]->get() & UART_CR2_REN) && ++s_rec_bit >= bitclkmax)
                {
                  dr->write(input);
                  cfg_write(serconf_received, input);
                  input_avail= false;
                  s_receiving= false;
                  s_rec_bit-= bitclkmax;

                  if (!(regs[sr]->get() & UART_SR_RXNE))
                    regs[sr]->write_bit1(UART_SR_RXNE);
                  else
                    regs[sr]->write_bit1(UART_SR_OR);
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
          else if (!(regs[sr]->get() & UART_SR_IDLE))
            {
              s_rec_bit= 0;
              regs[sr]->write_bit1(UART_SR_IDLE);
            }
        }

      hw_updating--;
    }

  return(0);
}

void
cl_serial::reset(void)
{
  int i;
  hw_updating++;
  for (i= 2; i < 12; i++)
    regs[i]->write(0);
  regs[sr]->write(UART_SR_TXE | UART_SR_TC);
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
  con->dd_printf("%s[%d] at 0x%06x %s\n", id_string, id, base, ((regs[cr1]->get() & UART_CR1_UARTD) ? "off" : "on"));
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
