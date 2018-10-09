/*
 * Simulator of microcontrollers (stm8.src/port.cc)
 *
 * Copyright (C) 2017,17 Drotos Daniel, Talker Bt.
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

#include "stm8cl.h"
#include "itsrccl.h"
#include "portcl.h"

cl_port::cl_port(class cl_uc *auc, int iportnr/*, int aid*/, const char *aname):
	cl_hw(auc, HW_PORT, /*aid*/0, aname)
{
  portnr= iportnr;
  base= 0x5000 + portnr * 5;
  set_name(aname);
}

int
cl_port::init(void)
{
  class cl_itc *itc = ((cl_stm8 *)uc)->itc;

  // We need to hook EXTI_CR* because changing interrupt sensitivity
  // may lead to an immediate interrupt.
  register_cell(itc->exti_cr1);
  register_cell(itc->exti_cr2);
  register_cell(itc->exti_cr3);
  register_cell(itc->exti_cr4);

  // We need to hook EXTI_SR* because a level-triggered interrupt
  // may come straight back when cleared.
  register_cell(itc->exti_sr1);
  register_cell(itc->exti_sr2);

  // We need to hook EXTI_CR* because changing them may change what
  // interrupts are presented.
  // N.B. We assume the presentation of edge-triggered interrupts
  // is fixed at the time the event is handled. Documentation is
  // not clear on this point but it seems reasonable?
  register_cell(itc->exti_conf1);
  register_cell(itc->exti_conf2);

  cl_hw::init();
  // ODR
  cell_p= register_cell(uc->rom, base + 0);
  // IDR
  cell_in= register_cell(uc->rom, base + 1);
  // DDR: 0=input, 1=output
  cell_dir= register_cell(uc->rom, base + 2);
  // CR1
  cell_cr1= register_cell(uc->rom, base + 3);
  // CR2
  cell_cr2= register_cell(uc->rom, base + 4);

  cl_var *v;
  chars pn= cchars(get_name());
  uc->vars->add(v= new cl_var(pn+chars("_odr"), uc->rom, base+0,
			      "Output data register", 7, 0));
  v->init();
  uc->vars->add(v= new cl_var(pn+chars("_idr"), uc->rom, base+1,
			      "Input data register (outside value of port pins)", 7, 0));
  v->init();
  uc->vars->add(v= new cl_var(pn+chars("_pin"), uc->rom, base+1,
			      "Outside value of port pins", 7, 0));
  v->init();
  uc->vars->add(v= new cl_var(pn+chars("_pins"), uc->rom, base+1,
			      "Outside value of port pins", 7, 0));
  v->init();
  uc->vars->add(v= new cl_var(pn+chars("_ddr"), uc->rom, base+2,
			      "Direction register", 7, 0));
  v->init();
  uc->vars->add(v= new cl_var(pn+chars("_cr1"), uc->rom, base+3,
			      "Control register 1", 7, 0));
  v->init();
  uc->vars->add(v= new cl_var(pn+chars("_cr2"), uc->rom, base+4,
			      "Control register 2", 7, 0));
  v->init();

  return 0;
}

void
cl_port::reset(void)
{
  cell_cr2->write(0);
  if (uc->type->type == CPU_STM8S)
    cell_cr1->write(base == 0x500f ? 0x02: 0); // PD_CR1 reset value is 0x02 (RM0016: 11.9.5)
  else
    cell_cr1->write(base == 0x5000 ? 0x01: 0); // PA_CR1 reset value is 0x01 (RM0013: 10.9.4 and RM0031: 10.9.4)
  cell_dir->write(0);
  cell_p->write(0);
  cell_in->write(0);
}

bool
cl_port::pin_or_port_high(t_mem exti_conf1, t_mem exti_conf2)
{
  switch (portnr)
    {
      case 0: // Port A
        return (uc->type->type == CPU_STM8S);
      case 1: // Port B
        return (uc->type->type == CPU_STM8S) || (exti_conf1 & (1U << 1));
      case 2: // Port C
        return (uc->type->type == CPU_STM8S);
      case 3: // Port D
        return (uc->type->type == CPU_STM8S) || (exti_conf1 & (1U << 3));
      case 4: // Port E
        return (uc->type->type == CPU_STM8S) || (exti_conf1 & (1U << 5));
      case 5: // Port F
        return (uc->type->type == CPU_STM8S) || (exti_conf2 & (1U << 0));
      case 6: // Port G
        return (exti_conf2 & (1U << 2));
      case 7: // Port H
        return (exti_conf2 & (1U << 4));
      case 8: // Port I
        break;
    }
  return false; // Bits [7:4] generate pin interrupts
}

bool
cl_port::pin_or_port_low(t_mem exti_conf1, t_mem exti_conf2)
{
  switch (portnr)
    {
      case 0: // Port A
        return (uc->type->type == CPU_STM8S);
      case 1: // Port B
        return (uc->type->type == CPU_STM8S) || (exti_conf1 & (1U << 0));
      case 2: // Port C
        return (uc->type->type == CPU_STM8S);
      case 3: // Port D
        return (uc->type->type == CPU_STM8S) || (exti_conf1 & (1U << 2));
      case 4: // Port E
        return (uc->type->type == CPU_STM8S) || (exti_conf1 & (1U << 4));
      case 5: // Port F
        return (uc->type->type == CPU_STM8S) || (exti_conf1 & (1U << 6));
      case 6: // Port G
        return (exti_conf2 & (1U << 1));
      case 7: // Port H
        return (exti_conf2 & (1U << 3));
      case 8: // Port I
        break;
    }
  return false; // Bits [3:0] generate pin interrupts
}

bool
cl_port::port_used_for_interrupt(t_mem exti_conf1, t_mem exti_conf2)
{
  switch (portnr)
    {
      case 0: // Port A
        return (uc->type->type == CPU_STM8S);
      case 1: // Port B
        return (uc->type->type == CPU_STM8S) || !(exti_conf2 & (1U << 5));
      case 2: // Port C
        return (uc->type->type == CPU_STM8S) || (uc->type->type == CPU_STM8S);
      case 3: // Port D
        return (uc->type->type == CPU_STM8S) || !(exti_conf2 & (1U << 6));
      case 4: // Port E
        return (uc->type->type == CPU_STM8S) || !(exti_conf1 & (1U << 7));
      case 5: // Port F
        return (uc->type->type != CPU_STM8S) && (exti_conf1 & (1U << 7));
      case 6: // Port G
        return (exti_conf2 & (1U << 5));
      case 7: // Port H
        return (exti_conf2 & (1U << 6));
      case 8: // Port I
        break;
    }
  return false;
}

int
cl_port::port_sensitivity(t_mem exti_cr1, t_mem exti_cr2, t_mem exti_cr3, t_mem exti_cr4)
{
  if (uc->type->type == CPU_STM8S)
    {
      switch (portnr)
        {
          case 0: // Port A
            return ((exti_cr1 >> 0) & 0x11);
          case 1: // Port B
            return ((exti_cr1 >> 2) & 0x11);
          case 2: // Port C
            return ((exti_cr1 >> 4) & 0x11);
          case 3: // Port D
            return ((exti_cr1 >> 6) & 0x11);
          case 4: // Port E
            return ((exti_cr2 >> 0) & 0x11);
        }
    }
  else
    {
      switch (portnr)
        {
          case 0: // Port A
            break;
          case 1: // Port B
            return ((exti_cr3 >> 0) & 0x11);
          case 2: // Port C
            break;
          case 3: // Port D
            return ((exti_cr3 >> 2) & 0x11);
          case 4: // Port E
            return ((exti_cr3 >> 4) & 0x11);
          case 5: // Port F
            return ((exti_cr3 >> 6) & 0x11);
          case 6: // Port G
            return ((exti_cr4 >> 0) & 0x11);
          case 7: // Port H
            return ((exti_cr4 >> 2) & 0x11);
          case 8: // Port I
            break;
        }
    }
  return 0; // Falling edge and low level
}

void
cl_port::port_interrupt(t_mem *exti_sr1, t_mem *exti_sr2)
{
  if (uc->type->type == CPU_STM8S)
      *exti_sr1 |= (1U << portnr);
  else
      *exti_sr2 |= (1U << (portnr == 1 ? 0 : portnr - 2));
}

void
cl_port::port_check_interrupt(t_mem input_and_enabled, t_mem in, t_mem prev_in, t_mem bitmask, int bithigh, int bitlow, t_mem exti_cr1, t_mem exti_cr2, t_mem exti_cr3, t_mem exti_cr4, t_mem *exti_sr1, t_mem *edge_exti_sr1, t_mem *exti_sr2, t_mem *edge_exti_sr2)
{
  switch (port_sensitivity(exti_cr1, exti_cr2, exti_cr3, exti_cr4))
    {
      case 0x00: // Falling edge and low level
        //printf("port_check_interrupt: in 0x%02x, input_and_enabled 0x%02x, bitmask 0x%02x\n", in, input_and_enabled, bitmask);
        if (uc->type->type == CPU_STM8S)
            *edge_exti_sr1 &= ~(1U << portnr);
        if ((in & input_and_enabled & bitmask) != (input_and_enabled & bitmask))
          port_interrupt(exti_sr1, exti_sr2);
        break;

      case 0x01: // Rising edge only
        for (int i = bitlow; i <= bithigh; i++)
          {
            if ((input_and_enabled & (1U << i)) && !(prev_in & (1U << i)) && (in & (1U << i)))
              {
                port_interrupt(exti_sr1, exti_sr2);
                break;
              }
          }
        break;

      case 0x10: // Falling edge only
        for (int i = bitlow; i <= bithigh; i++)
          {
            if ((input_and_enabled & (1U << i)) && (prev_in & (1U << i)) && !(in & (1U << i)))
              {
                port_interrupt(exti_sr1, exti_sr2);
                break;
              }
          }
        break;

      case 0x11: // Rising and falling edge only
        for (int i = bitlow; i <= bithigh; i++)
          {
            if ((input_and_enabled & (1U << i)) && (prev_in & (1U << i)) != (in & (1U << i)))
              {
                port_interrupt(exti_sr1, exti_sr2);
                break;
              }
          }
        break;
    }
}

void
cl_port::pin_check_interrupt(t_mem input_and_enabled, t_mem exti_crn, t_mem in, t_mem prev_in, int bithigh, int bitlow, t_mem *exti_sr1, t_mem *edge_exti_sr1)
{
  for (int i = bitlow; i <= bithigh; i++)
    {
      t_mem bit = 1U << i;

      switch (exti_crn >> ((i - bitlow) * 2) & 0x11)
        {
          case 0x00: // Falling edge and low level
            if (uc->type->type == CPU_STM8S)
              *edge_exti_sr1 &= ~bit;
            if ((input_and_enabled & bit) && !(in & bit))
              *exti_sr1 |= bit;
            break;

          case 0x01: // Rising edge only
            if ((input_and_enabled & bit) && !(prev_in & bit) && (in & bit))
              *exti_sr1 |= bit;
            break;

          case 0x10: // Falling edge only
            if ((input_and_enabled & bit) && (prev_in & bit) && !(in & bit))
              *exti_sr1 |= bit;
            break;

          case 0x11: // Rising and falling edge only
            if ((input_and_enabled & bit) && (prev_in & bit) != (in & bit))
              *exti_sr1 |= bit;
            break;
        }
    }
}

void
cl_port::write(class cl_memory_cell *cell, t_mem *val)
{
  class cl_itc *itc = ((cl_stm8 *)uc)->itc;

  t_mem exti_cr1 = (cell == itc->exti_cr2 ? *val : itc->exti_cr1->get());
  t_mem exti_cr2 = (cell == itc->exti_cr2 ? *val : itc->exti_cr2->get());
  t_mem exti_cr3 = (cell == itc->exti_cr3 ? *val : itc->exti_cr3->get());
  t_mem exti_cr4 = (cell == itc->exti_cr4 ? *val : itc->exti_cr4->get());
  t_mem exti_conf1 = (cell == itc->exti_conf1 ? *val : itc->exti_conf1->get());
  t_mem exti_conf2 = (cell == itc->exti_conf2 ? *val : itc->exti_conf2->get());

  t_mem in = (cell == cell_in ? *val : cell_in->get());
  t_mem dir = (cell == cell_dir ? *val : cell_dir->get());
  //t_mem cr1 = (cell == cell_cr1 ? *val : cell_cr1->get());
  t_mem cr2 = (cell == cell_cr2 ? *val : cell_cr2->get());

  t_mem orig_exti_sr1 = (cell == itc->exti_sr1 ? *val : itc->exti_sr1->get());
  t_mem edge_exti_sr1 = ~0U;
  t_mem exti_sr1 = 0U;

  t_mem orig_exti_sr2 = (cell == itc->exti_sr2 ? *val : itc->exti_sr2->get());
  t_mem edge_exti_sr2 = ~0U;
  t_mem exti_sr2 = 0U;

#if 0
	  if (cell == itc->exti_cr1)
		  printf("Port %d: write 0x%02x to exti_cr1\n", portnr, *val);
	  else if (cell == itc->exti_cr2)
		  printf("Port %d: write 0x%02x to exti_cr2\n", portnr, *val);
	  else if (cell == itc->exti_cr3)
		  printf("Port %d: write 0x%02x to exti_cr3\n", portnr, *val);
	  else if (cell == itc->exti_cr4)
		  printf("Port %d: write 0x%02x to exti_cr4\n", portnr, *val);
	  else if (cell == itc->exti_conf1)
		  printf("Port %d: write 0x%02x to exti_conf1\n", portnr, *val);
	  else if (cell == itc->exti_conf2)
		  printf("Port %d: write 0x%02x to exti_conf2\n", portnr, *val);
	  else if (cell == cell_in)
		  printf("Port %d: write 0x%02x to in\n", portnr, *val);
	  else if (cell == cell_dir)
		  printf("Port %d: write 0x%02x to dir\n", portnr, *val);
	  else if (cell == cell_cr2)
		  printf("Port %d: write 0x%02x to cr2\n", portnr, *val);
	  else if (cell == itc->exti_sr1)
		  printf("Port %d: write 0x%02x to exti_sr1\n", portnr, *val);
	  else if (cell == itc->exti_sr2)
		  printf("Port %d: write 0x%02x to exti_sr2\n", portnr, *val);
#endif

  t_mem input_and_enabled = (~dir) & cr2;

  if (cell == itc->exti_sr1 || cell == itc->exti_sr2)
    {
      // The STM8S Value line MCUs do not expose EXTI_SR1 or EXTI_SR2 and interrupts
      // are cleared by the simulator's ITC setting bits back to zero. With other
      // MCUs the interrupt has to be explicitly cleared by software writing a '1'
      // to the relevant bit. (RM0013: 5.9.6 & RM0031: 12.9.7)
      if (uc->type->type != CPU_STM8S)
        *val = cell->get() & (~*val);
    }

  if (cell != cell_p /* && cell != cell_cr1 */) // CR1 is not hooked...
    {
      t_mem prev_in = (cell == cell_in ? cell_in->get() : in);

      if (pin_or_port_high(exti_conf1, exti_conf2))
        {
          //printf("Port %d high: port mode\n", portnr);
          if (port_used_for_interrupt(exti_conf1, exti_conf2))
            {
              //printf("Port %d high: port used for interrupts\n", portnr);
              port_check_interrupt(input_and_enabled, in, prev_in, 0xf0, 7, 4, exti_cr1, exti_cr2, exti_cr3, exti_cr4, &exti_sr1, &edge_exti_sr1, &exti_sr2, &edge_exti_sr2);
            }
        }
      else
        {
          //printf("Port %d high: pin mode\n", portnr);
          pin_check_interrupt(input_and_enabled, exti_cr2, in, prev_in, 7, 4, &exti_sr1, &edge_exti_sr1);
        }

      if (pin_or_port_low(exti_conf1, exti_conf2))
        {
          //printf("Port %d low: port mode\n", portnr);
          if (port_used_for_interrupt(exti_conf1, exti_conf2))
            {
              //printf("Port %d low: port used for interrupts\n", portnr);
              port_check_interrupt(input_and_enabled, in, prev_in, 0x0f, 3, 0, exti_cr1, exti_cr2, exti_cr3, exti_cr4, &exti_sr1, &edge_exti_sr1, &exti_sr2, &edge_exti_sr2);
            }
        }
      else
        {
          //printf("Port %d low: pin mode\n", portnr);
          pin_check_interrupt(input_and_enabled, exti_cr1, in, prev_in, 3, 0, &exti_sr1, &edge_exti_sr1);
        }

      // We have to write to EXTI_SR1 and EXTI_SR2 in order to ensure the
      // interrupt controller sees our changes regardless of the ordering
      // of the hw element chain. Whatever change triggered this change
      // in interrupt state must be written before that second pass in
      // order that the second pass is working from the correct data.
      exti_sr1 = (orig_exti_sr1 & edge_exti_sr1) | exti_sr1;
      if (exti_sr1 != orig_exti_sr1)
        {
          //printf("Port %d: write SR1 edge 0x%02x, ints 0x%02x, curr 0x%02x, orig 0x%02x, new 0x%02x\n", portnr, edge_exti_sr1, exti_sr1, itc->exti_sr1->get(), orig_exti_sr1, exti_sr1);
          if (cell == itc->exti_sr1)
            *val = exti_sr1;

	  // Whatever triggered this change in interrupt state must be
	  // put in place before the second pass so it can work from
	  // the correct data.
	  cell->set(*val);

	  // Set the new value before the write so that the second pass
	  // does not go recursive trying to do something that is already
	  // being done.
          itc->exti_sr1->set(exti_sr1);
          itc->exti_sr1->write(exti_sr1);
        }

      exti_sr2 = (orig_exti_sr2 & edge_exti_sr2) | exti_sr2;
      if (exti_sr2 != orig_exti_sr2)
        {
          //printf("Port %d: write SR2 edge 0x%02x, ints 0x%02x, curr 0x%02x, orig 0x%02x, new 0x%02x\n", portnr, edge_exti_sr2, exti_sr2, itc->exti_sr2->get(), orig_exti_sr2, exti_sr2);
          if (cell == itc->exti_sr2)
            *val = exti_sr2;

	  // Whatever triggered this change in interrupt state must be
	  // put in place before the second pass so it can work from
	  // the correct data.
	  cell->set(*val);

	  // Set the new value before the write so that the second pass
	  // does not go recursive trying to do something that is already
	  // being done.
          itc->exti_sr2->set(exti_sr2);
          itc->exti_sr2->write(exti_sr2);
        }
    }

  if ((cell == cell_p) ||
      (cell == cell_in) ||
      (cell == cell_dir))
    {
      // Any pins configured as outputs will be mirrored in the inputs as well.
      t_mem p= (cell == cell_p ? *val : cell_p->get());
      in&= ~dir;
      in|= (p & dir);
      cell_in->set(in);
      if (cell == cell_in)
        *val= in;
    }
}

void
cl_port::print_info(class cl_console_base *con)
{
  int m;
  t_mem o= cell_p->get(),
    i= cell_in->get(),
    d= cell_dir->get();
  con->dd_printf("%s at 0x%04x\n", get_name(), base);
  con->dd_printf("dir: 0x%02x ", d);
  for (m= 0x80; m; m>>= 1)
    con->dd_printf("%c", (d & m)?'O':'I');
  con->dd_printf("\n");
  con->dd_printf("out: 0x%02x ", o);
  for (m= 0x80; m; m>>= 1)
    {
      if (d & m)
	con->dd_printf("%c", (o & m)?'1':'0');
      else
	con->dd_printf("-");
    }
  con->dd_printf("\n");
  con->dd_printf("in : 0x%02x ", i);
  for (m= 0x80; m; m>>= 1)
    {
      //if (!(d & m))
	con->dd_printf("%c", (i & m)?'1':'0');
	//else
	//con->dd_printf("-");
    }
  con->dd_printf("\n");
  i= cell_cr1->get();
  con->dd_printf("cr1: 0x%02x ", i);
  for (m= 0x80; m; m>>= 1)
    con->dd_printf("%c", (i & m)?'1':'0');
  con->dd_printf("\n");
  i= cell_cr2->get();
  con->dd_printf("cr2: 0x%02x ", i);
  for (m= 0x80; m; m>>= 1)
    con->dd_printf("%c", (i & m)?'1':'0');
  con->dd_printf("\n");
  print_cfg_info(con);
}


/* End of stm8.src/port.cc */
