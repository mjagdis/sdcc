/*
 * Simulator of microcontrollers (stm8.src/clk.cc)
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

#include "utils.h"

#include "stm8cl.h"

#include "clkcl.h"


// FIXME: there needs to be some way to set the value of a cell ignoring
// the CELL_READ_ONLY flag.
static void
write_ro(class cl_memory_cell *cell, t_mem val)
{
  cell->set_flag(CELL_READ_ONLY, false);
  cell->write(val);
  cell->set_flag(CELL_READ_ONLY, true);
}


cl_clk::cl_clk(class cl_uc *auc):
       cl_hw(auc, HW_CLOCK, 0, "clk")
{
  f_LSI = 0;
  f_LSE = 0;

  CLK_SOURCE_HSI = 0;
  CLK_SOURCE_LSI = 0;
  CLK_SOURCE_HSE = 0;
  CLK_SOURCE_LSE = 0;

  base= 0x50C0;

  cmsr= NULL;
  swr= NULL;
  swcr= NULL;
  ckdivr= NULL;
  pckenr1= NULL;
  pckenr2= NULL;
  pckenr3= NULL;
}

int
cl_clk::init(void)
{
  cl_hw::init();

  uc->xtal = f_HSI;

  make_partner(HW_TIMER, 1);
  make_partner(HW_TIMER, 2);
  make_partner(HW_TIMER, 3);
  make_partner(HW_TIMER, 4);
  make_partner(HW_TIMER, 5);
  make_partner(HW_TIMER, 6);

  make_partner(HW_UART, 1);
  make_partner(HW_UART, 2);
  make_partner(HW_UART, 3);
  make_partner(HW_UART, 4);
  
  return 0;
}

int
cl_clk::clock_per_cycle(void)
{
  return 1;
}

int
cl_clk::xtal_per_clock(t_mem ckdivr_val)
{
  return (1 << (ckdivr_val & 0x03));
}

void
cl_clk::set_osc(t_mem osc)
{
  if (osc == CLK_SOURCE_HSI)
    {
      if (ickr)
        ickr->write((ickr->get() | CLK_ICKR_HSIRDY));
      write_ro(cmsr, CLK_SOURCE_HSI);
      f_OSC = f_HSI;
    }
  else if (osc == CLK_SOURCE_HSE)
    {
      if (eckr)
        eckr->write((eckr->get() | CLK_ECKR_HSERDY));
      write_ro(cmsr, CLK_SOURCE_HSE);
      f_OSC= uc->xtal_option->get_value(uc->xtal);
    }
  else if (osc == CLK_SOURCE_LSI)
    {
      // FIXME: if LSI_EN option bit is set after SWEN is set and SWBSY is clear
      // should it trigger a clock change?
      class cl_address_space *opts = uc->address_space(cchars("option_chip"));
      if (opts && (opts->get(0x4805) & 0x08) && (opts->get(0x4806) & ~0x08))
        {
          if (ickr)
            ickr->write((ickr->get() | CLK_ICKR_LSIRDY));
          write_ro(cmsr, CLK_SOURCE_LSI);
          f_OSC = f_LSI;
        }
    }
  else if (osc == CLK_SOURCE_LSE)
    {
      if (eckr)
        eckr->write((eckr->get() | CLK_ECKR_LSERDY));
      write_ro(cmsr, CLK_SOURCE_LSE);
      f_OSC = f_LSE;
    }

  uc->xtal = (ckdivr ? f_OSC / xtal_per_clock(ckdivr->get()) : f_OSC);
}

void
cl_clk::write(class cl_memory_cell *cell, t_mem *val)
{
  if (ckdivr && cell == ckdivr)
    uc->xtal = f_OSC / xtal_per_clock(*val);
  else if (swr && cell == swr)
    {
      swcr->write(swcr->get() | CLK_SWCR_SWBSY);

      // If the new clock was not already active we should now wait for the oscillator
      // to stabilize (<1μs for HSI) then clear SWBSY (and do the change if SWEN is set)
      // however it is rare that oscillators are stopped and restarted (and not possible
      // for HSI according to the erratas) so we assume it is ready.
      // N.B. Manually clearing SWBSY cancels a pending oscillator change so this should
      // ONLY happen if SWBSY is still set. The check is redundant here but would be
      // required if we add a stabilization delay.
      t_mem v = swcr->get();
      if (v & CLK_SWCR_SWBSY)
        {
          if (v & CLK_SWCR_SWEN)
            set_osc(*val);
          swcr->write(v & ~CLK_SWCR_SWBSY);
        }
    }
  else if (swcr && cell == swcr)
    {
      // Setting SWEN when SWBSY is clear triggers a clock change
      if ((swcr->get() & (CLK_SWCR_SWEN|CLK_SWCR_SWBSY)) == 0x00 && (*val & (CLK_SWCR_SWEN|CLK_SWCR_SWBSY)) == CLK_SWCR_SWEN)
        set_osc(swr->get());
    }
  else if ((cell == pckenr1) ||
      (cell == pckenr2) ||
      (cell == pckenr3))
    {
      cl_clk_event e;
      hw_event ev;

      e.set(HW_TIMER, 1);
      ev= tim(e.id, val)?EV_CLK_ON:EV_CLK_OFF;
      inform_partners(ev, &e);

      e.id= 2;
      ev= tim(e.id, val)?EV_CLK_ON:EV_CLK_OFF;
      inform_partners(ev, &e);

      e.id= 3;
      ev= tim(e.id, val)?EV_CLK_ON:EV_CLK_OFF;
      inform_partners(ev, &e);

      e.id= 4;
      ev= tim(e.id, val)?EV_CLK_ON:EV_CLK_OFF;
      inform_partners(ev, &e);

      e.id= 5;
      ev= tim(e.id, val)?EV_CLK_ON:EV_CLK_OFF;
      inform_partners(ev, &e);

      e.id= 6;
      ev= tim(e.id, val)?EV_CLK_ON:EV_CLK_OFF;
      inform_partners(ev, &e);

      e.set(HW_UART, 1);
      ev= usart(e.id, val)?EV_CLK_ON:EV_CLK_OFF;
      inform_partners(ev, &e);

      e.id= 2;
      ev= usart(e.id, val)?EV_CLK_ON:EV_CLK_OFF;
      inform_partners(ev, &e);

      e.id= 3;
      ev= usart(e.id, val)?EV_CLK_ON:EV_CLK_OFF;
      inform_partners(ev, &e);

      e.id= 4;
      ev= usart(e.id, val)?EV_CLK_ON:EV_CLK_OFF;
      inform_partners(ev, &e);
    }
}

void
cl_clk::print_info(class cl_console_base *con)
{
  con->dd_printf("%s[%d] at 0x%06x\n", id_string, id, base);

  if (cmsr)
    {
      t_mem curr = cmsr->get();

      if (curr == CLK_SOURCE_HSI)
          con->dd_printf("  HSI");
      else if (curr == CLK_SOURCE_HSE)
          con->dd_printf("  HSE");
      else if (curr == CLK_SOURCE_LSI)
          con->dd_printf("  LSI");
      else if (curr == CLK_SOURCE_LSE)
          con->dd_printf("  LSE");

      t_mem want = swr->get();
      if (want != curr)
        {
          if (want == CLK_SOURCE_HSI)
            con->dd_printf("[->HSI]");
	  else if (want == CLK_SOURCE_HSE)
            con->dd_printf("[->HSE]");
	  else if (want == CLK_SOURCE_LSI)
            con->dd_printf("[->LSI]");
	  else if (want == CLK_SOURCE_LSE)
            con->dd_printf("[->LSE]");
        }

      con->dd_printf("\n");
    }

  const char *prefix;
  double sfreq= si_prefix(f_OSC, &prefix);
  con->dd_printf("  f_OSC:    %.0lf%sHz\n", sfreq, prefix);

  if (ckdivr)
    {
      con->dd_printf("  HSIDIV:   %d\n", xtal_per_clock(ckdivr->get()));
      sfreq= si_prefix(uc->xtal, &prefix);
      con->dd_printf("  f_MASTER: %.0lf%sHz\n", sfreq, prefix);
      con->dd_printf("  CPUDIV:   %d\n", clock_per_cycle());
      sfreq= si_prefix(uc->xtal / clock_per_cycle(), &prefix);
      con->dd_printf("  f_CPU:    %.0lf%sHz\n", sfreq, prefix);
    }

  if (pckenr1)
    con->dd_printf("  PCKENR1:  0x%02x\n", pckenr1->get());
  if (pckenr2)
    con->dd_printf("  PCKENR2:  0x%02x\n", pckenr2->get());
  if (pckenr3)
    con->dd_printf("  PCKENR3:  0x%02x\n", pckenr3->get());
}


/* SAF */

cl_clk_saf::cl_clk_saf(class cl_uc *auc):
  cl_clk(auc)
{
  f_LSI = 128000;

  CLK_SOURCE_HSI = 0xe1;
  CLK_SOURCE_LSI = 0xd2;
  CLK_SOURCE_HSE = 0xb4;
}

int
cl_clk_saf::init(void)
{
  cl_clk::init();

  hwreg(uc->rom, base+0, "ICKR", "Internal clock register",
    "REGAH",  5, 5, "Regulator power off in Active-halt mode",
    "LSIRDY", 4, 4, "Low speed internal oscillator ready",
    "LSIEN",  3, 3, "Low speed internal RC oscillator enable",
    "FHWU",   2, 2, "Fast wakeup from Halt/Active-halt modes",
    "HSIRDY", 1, 1, "High speed internal oscillator ready",
    "HSIEN",  0, 0, "High speed internal RC oscillator enable",
    NULL);
  ickr = register_cell(uc->rom, base+0);

  hwreg(uc->rom, base+1, "ECKR", "External clock register",
    "HSERDY", 1, 1, "High speed external crystal oscillator ready",
    "HSEEN",  0, 0, "High speed external crystal oscillator enable",
    NULL);
  eckr = register_cell(uc->rom, base+1);

  hwreg(uc->rom, base+3, "CMSR", "Clock master status register",
    "CKM", 7, 0, "Clock master status bits",
    NULL);
  cmsr = register_cell(uc->rom, base+3);
  cmsr->set_flag(CELL_READ_ONLY, true);

  hwreg(uc->rom, base+4, "SWR", "Clock master switch register",
    "SWI", 7, 0, "Clock master selection bits",
    NULL);
  swr = register_cell(uc->rom, base+4);

  hwreg(uc->rom, base+5, "SWCR", "Clock switch control register",
    "SWIF",  3, 3, "Clock switch interrupt flag",
    "SWIEN", 2, 2, "Clock switch interrupt enable",
    "SWEN",  1, 1, "Switch start/stop",
    "SWBSY", 0, 0, "Switch busy",
    NULL);
  swcr = register_cell(uc->rom, base+5);

  hwreg(uc->rom, base+6, "CKDIVR", "Clock divider register",
    "HSIDIV", 4, 3, "High speed internal clock prescalar",
    "CPUDIV", 2, 0, "CPU clock prescalar",
    NULL);
  ckdivr = register_cell(uc->rom, base+6);

  hwreg(uc->rom, base+7, "PCKENR1", "Peripheral clock gating register 1",
    "TIM1",     7, 7, "Peripheral clock enable TIM1",
    "TIM3",     6, 6, "Peripheral clock enable TIM3",
    "TIM25",    5, 5, "Peripheral clock enable TIM2/TIM5",
    "TIM46",    4, 4, "Peripheral clock enable TIM4/TIM6",
    "UART1234", 3, 3, "Peripheral clock enable UART1/2/3/4",
    "UART1234", 2, 2, "Peripheral clock enable UART1/2/3/4",
    "SPI",      1, 1, "Peripheral clock enable SPI",
    "I2C",      0, 0, "Peripheral clock enable I2C",
    NULL);
  pckenr1 = register_cell(uc->rom, base+7);

  hwreg(uc->rom, base+8, "CSSR", "Clock security system register",
    "CSSD",   3, 3, "Clock security system detection",
    "CSSDIE", 2, 2, "Clock security system detection interrupt enable",
    "AUX",    1, 1, "Auxiliary oscillator connected to master clock",
    "CSSEN",  0, 0, "Clock security system enable",
    NULL);
  // cssr = register_cell(uc->rom, base+8);

  hwreg(uc->rom, base+9, "CCOR", "Configurable clock output register",
    "CCOBSY", 6, 6, "Configurable clock output busy",
    "CCORDY", 5, 5, "Configurable clock output ready",
    "CCOSEL", 4, 1, "Configurable clock output selection",
    "CCOEN",  0, 0, "Configurable clock output enable",
    NULL);
  // ccor = register_cell(uc->rom, base+9);

  hwreg(uc->rom, base+10, "PCKENR2", "Peripheral clock gating register 2",
    "CAN", 7, 7, "Peripheral clock enable CAN",
    "ADC", 3, 3, "Peripheral clock enable ADC",
    "AWU", 2, 2, "Peripheral clock enable AWU",
    NULL);
  pckenr2 = register_cell(uc->rom, base+10);

  hwreg(uc->rom, base+12, "HSITRIMR", "HSI clock calibration trimming register",
    "HSITRIM", 3, 0, "HSI trimming value",
    NULL);
  // hsitrimr = register_cell(uc->rom, base+12);

  hwreg(uc->rom, base+13, "SWIMCCR", "SWIM clock control register",
    "SWIMCLK", 0, 0, "SWIM clock divider",
    NULL);
  // swimccr = register_cell(uc->rom, base+13);

  return 0;
}

int
cl_clk_saf::clock_per_cycle(void)
{
  return (1 << (ckdivr->get() & 0x07));
}

int
cl_clk_saf::xtal_per_clock(t_mem ckdivr_val)
{
  if (cmsr->get() == CLK_SOURCE_HSI)
    return (1 << ((ckdivr_val >> 3) & 0x03));
  else
    return 1;
}

void
cl_clk_saf::reset(void)
{
  ickr->write(0x01);
  eckr->write(0x00);
  ckdivr->write(0x18);
  swr->write(CLK_SOURCE_HSI);
  swcr->write(0x00);
  pckenr1->write(0xff);
  uc->rom->write(base+8, 0x00); //cssr->write(0x00);
  uc->rom->write(base+9, 0x00); //ccor->write(0x00);
  pckenr2->write(0xff);
  uc->rom->write(base+12, 0x00); //hsitrimr->write(0x00);
  uc->rom->write(base+13, 0x00); //swimccr->write(0x00);

  set_osc(CLK_SOURCE_HSI);
}

void
cl_clk_saf::write(class cl_memory_cell *cell, t_mem *val)
{
  if (cell == eckr)
    {
      t_mem preserve = 0xfc;

      // RM0016: 9.9.2: HSERDY: This bit is set and cleared by hardware.
      preserve |= CLK_ECKR_HSERDY;

      // RM0016 9.9.2: It [HSEEN] cannot be cleared when HSE is selected as clock
      // master (CLK_CMSR register) or as the active CCO source.
      // FIXME: CCO is not yet implemented.
      if (cmsr->get() == CLK_SOURCE_HSE)
        preserve |= CLK_ECKR_HSEEN;

      *val = (*val & (~preserve)) | (eckr->get() & preserve);
    }
  else if (cell == ickr)
    {
      t_mem preserve = 0;

      // RM0016: 9.9.1: HSIRDY: This bit is set and cleared by hardware.
      // RM0016: 9.9.1: LSIRDY: This bit is set and cleared by hardware.
      preserve = CLK_ICKR_HSIRDY | CLK_ICKR_LSIRDY;

      // RM0016 9.9.1: It [LSIEN] cannot be cleared when LSI is selected as clock
      // master (CLK_CMSR register), as the active CCO source or as a clock source
      // for the AWU peripheral or independent Watchdog.
      // FIXME: CCO, AWU and IWDG are not yet implemented.
      if (cmsr->get() == CLK_SOURCE_LSI)
        preserve |= CLK_ICKR_LSIEN;

      // RM0016 9.9.1: It [HSIEN] cannot be cleared when HSI is selected as clock
      // master (CLK_CMSR register), as the active CCO source or if the safe oscillator
      // (AUX) is enabled.
      // FIXME: CCO and AUX are not yet implemented.
      if (cmsr->get() == CLK_SOURCE_HSI)
        preserve |= CLK_ICKR_HSIEN;

      *val = (*val & (~preserve)) | (ickr->get() & preserve);
    }
  else if (cell == swr)
    {
      if (*val == CLK_SOURCE_LSI)
        {
          // RM0016 9.9.1: It [LSIEN] is set by hardware whenever the LSI oscillator is
          // required, for example: [...] When switching to LSI clock (see CLK_SWR register).
          ickr->write(ickr->get() | CLK_ICKR_LSIEN);
        }
      if (*val == CLK_SOURCE_HSI)
        {
          // RM0016 9.9.1: It [HSIEN] is set by hardware whenever the HSI oscillator is
          // required, for example: [...] When switching to HSI clock (see CLK_SWR register).
          ickr->write(ickr->get() | CLK_ICKR_HSIEN);
        }
    }

  cl_clk::write(cell, val);
}

bool
cl_clk_saf::tim(int id, t_mem *val)
{
  switch (id)
    {
    case 1:
      return pckenr1 && (*val & 0x80);
    case 2: case 5:
      return pckenr1 && (*val & 0x20);
    case 3:
      return pckenr1 && (*val & 0x40);
    case 4: case 6:
      return pckenr1 && (*val & 0x10);
    }
  return false;
}

bool
cl_clk_saf::usart(int id, t_mem *val)
{
  cl_stm8 *u= (cl_stm8 *)uc;
  if (id == 1)
    switch (u->type->subtype)
      {
      case DEV_STM8S003: case DEV_STM8S103: case DEV_STM8S903:
	return pckenr1 && (*val & 0x08);
      case DEV_STM8S007: case DEV_STM8S207: case DEV_STM8S208:
      case DEV_STM8AF52:
	return pckenr1 && (*val & 0x04);
      }
  else if (id == 2)
    switch (u->type->subtype)
      {
      case DEV_STM8S005: case DEV_STM8S105: case DEV_STM8AF62_46:
	return pckenr1 && (*val & 0x08);
      }
  else if (id == 3)
    switch (u->type->subtype)
      {
      case DEV_STM8S007: case DEV_STM8S207: case DEV_STM8S208:
      case DEV_STM8AF52:
	return pckenr1 && (*val & 0x08);
      }
  else if (id == 4)
    switch (u->type->subtype)
      {
      case DEV_STM8AF62_12:
	return pckenr1 && (*val & 0x08);
      }
  return false;
}

/* ALL */

cl_clk_all::cl_clk_all(class cl_uc *auc):
  cl_clk(auc)
{
  f_LSI = 38000;
  f_LSE = 32768;

  CLK_SOURCE_HSI = 0x01;
  CLK_SOURCE_LSI = 0x02;
  CLK_SOURCE_HSE = 0x04;
  CLK_SOURCE_LSE = 0x08;
}

int
cl_clk_all::init(void)
{
  cl_clk::init();

  hwreg(uc->rom, base+0, "CKDIVR", "Clock divider register",
    "CKM", 2, 0, "System clock prescalar",
    NULL);
  ckdivr = register_cell(uc->rom, base+0);

  hwreg(uc->rom, base+1, "CRTCR", "Clock RTC register",
    "RTCDIV",   7, 5, "Clock RTC prescalar",
    "RTCSEL",   4, 1, "Configurable RTC clock source selection",
    "RTCSWBSY", 0, 0, "RTC clock busy",
    NULL);
  //crtcr = register_cell(uc->rom, base+1);

  hwreg(uc->rom, base+2, "ICKCR", "Internal clock register",
    "BEEPHALT", 6, 6, "BEEP clock Halt/Active-halt mode",
    "FHWU",     5, 5, "Fast wakeup from Halt/Active-halt modes",
    "SAHALT",   4, 4, "Slow Halt/Active-halt mode",
    "LSIRDY",   3, 3, "Low speed internal oscillator ready",
    "LSION",    2, 2, "Low speed internal oscillator enable",
    "HSIRDY",   1, 1, "High speed internal oscillator ready",
    "HSION",    0, 0, "High speed internal oscillator enable",
    NULL);
  ickr = register_cell(uc->rom, base+2);

  hwreg(uc->rom, base+3, "PCKENR1", "Peripheral clock gating register 1",
    "DAC",    7, 7, "Peripheral clock enable DAC",
    "BEEP",   6, 6, "Peripheral clock enable BEEP",
    "USART1", 5, 5, "Peripheral clock enable USART1",
    "SPI",    4, 4, "Peripheral clock enable SPI1",
    "I2C1",   3, 3, "Peripheral clock enable I2C1",
    "TIM4",   2, 2, "Peripheral clock enable TIM4",
    "TIM3",   1, 1, "Peripheral clock enable TIM3",
    "TIM2",   0, 0, "Peripheral clock enable TIM2",
    NULL);
  pckenr1 = register_cell(uc->rom, base+3);

  hwreg(uc->rom, base+4, "PCKENR2", "Peripheral clock gating register 2",
    "BOOTROM", 7, 7, "Peripheral clock enable boot ROM",
    "COMP12",  5, 5, "Peripheral clock enable COMP1 and COMP2",
    "DMA1",    4, 4, "Peripheral clock enable DMA1",
    "LCD",     3, 3, "Peripheral clock enable LCD",
    "RTC",     2, 2, "Peripheral clock enable RTC",
    "TIM1",    1, 1, "Peripheral clock enable TIM1",
    "ADC1",    0, 0, "Peripheral clock enable ADC1",
    NULL);
  pckenr2 = register_cell(uc->rom, base+4);

  hwreg(uc->rom, base+5, "CCOR", "Configurable clock output register",
    "CCODIV",   7, 5, "Configurable clock output prescalar",
    "CCOSEL",   4, 1, "Configurable clock output selection",
    "CCOSWBSY", 0, 0, "Configurable clock output switch busy",
    NULL);
  //ccor = register_cell(uc->rom, base+5);

  hwreg(uc->rom, base+6, "ECKCR", "External clock register",
    "LSEBYP", 5, 5, "Low speed external clock bypass",
    "HSEBYP", 4, 4, "High speed external clock bypass",
    "LSERDY", 3, 3, "Low speed external crystal oscillator ready",
    "LSEON",  2, 2, "Low speed external crystal oscillator enable",
    "HSERDY", 1, 1, "High speed external crystal oscillator ready",
    "HSEON",  0, 0, "High speed external crystal oscillator enable",
    NULL);
  eckr = register_cell(uc->rom, base+6);

  hwreg(uc->rom, base+7, "SCSR", "System clock status register",
    "CKM", 7, 0, "System clock status bits",
    NULL);
  cmsr = register_cell(uc->rom, base+7);
  uc->rom->set_cell_flag(base+7, true, CELL_READ_ONLY);

  hwreg(uc->rom, base+8, "SWR", "System clock switch register",
    "SWI", 7, 0, "System clock selection bits",
    NULL);
  swr = register_cell(uc->rom, base+8);

  hwreg(uc->rom, base+9, "SWCR", "Switch control register",
    "SWIF",  3, 3, "Clock switch interrupt flag",
    "SWIEN", 2, 2, "Clock switch interrupt enable",
    "SWEN",  1, 1, "Switch start/stop",
    "SWBSY", 0, 0, "Switch busy",
    NULL);
  swcr = register_cell(uc->rom, base+9);

  hwreg(uc->rom, base+10, "CSSR", "Clock security system register",
    "CSSDGON", 4, 4, "Clock security system deglitcher system",
    "CSSD",    3, 3, "Clock security system detection",
    "CSSDIE",  2, 2, "Clock security system detection interrupt enable",
    "AUX",     1, 1, "Auxiliary oscillator connected to master clock",
    "CSSEN",   0, 0, "Clock security system enable",
    NULL);
  //cssr = register_cell(uc->rom, base+10);

  hwreg(uc->rom, base+11, "CBEEPR", "Clock BEEP register",
    "CLKBEEPSEL", 2, 1, "Configurable BEEP clock source selection",
    "BEEPSWBSY",  0, 0, "System busy during BEEP clock change",
    NULL);
  //cbeepr = register_cell(uc->rom, base+11);

  hwreg(uc->rom, base+12, "HSICALR", "HSI calibration register",
    "HSICAL", 7, 0, "HSI calibration",
    NULL);
  //hsicalr = register_cell(uc->rom, base+12);

  hwreg(uc->rom, base+13, "HSITRIMR", "HSI clock calibration trimming register",
    "HSITRIM", 7, 0, "HSI trimming value",
    NULL);
  //hsitrimr = register_cell(uc->rom, base+13);

  hwreg(uc->rom, base+14, "HSIUNLCKR", "HSI unlock register",
    "HSIUNLCK", 7, 0, "HSI unlock mechanism",
    NULL);
  //hsiunlckr = register_cell(uc->rom, base+14);

  hwreg(uc->rom, base+15, "REGCSR", "Main regulator control status register",
    "EEREADY",  7, 7, "Flash program memory and data EEPROM ready",
    "EEBUSY",   6, 6, "Flash program memory and data EEPROM busy",
    "LSEPD",    5, 5, "LSE power-down",
    "HSEPD",    4, 4, "HSE power-down",
    "LSIPD",    3, 3, "LSI power-down",
    "HSIPD",    2, 2, "HSI power-down",
    "REGOFF",   1, 1, "Main regulator OFF",
    "REGREADY", 0, 0, "Main regulator ready",
    NULL);
  //regcsr = register_cell(uc->rom, base+15);

  hwreg(uc->rom, base+16, "PCKENR3", "Peripheral clock gating register 3",
    "CSS_LSE", 5, 5, "Peripheral clock enable CSS_LSE",
    "USART3",  4, 4, "Peripheral clock enable USART3",
    "USART2",  3, 3, "Peripheral clock enable USART2",
    "SPI2",    2, 2, "Peripheral clock enable SPI2",
    "TIM5",    1, 1, "Peripheral clock enable TIM5",
    "AES",     0, 0, "Peripheral clock enable AES",
    NULL);
  pckenr3 = register_cell(uc->rom, base+16);

  return 0;
}

void
cl_clk_all::reset(void)
{
  ckdivr->write(0x03);
  uc->rom->write(base+1, 0x00); //crtcr->write(0x00);
  ickr->write(0x11);
  uc->rom->write(base+5, 0x00); //ccor->write(0x00);
  eckr->write(0x00);
  swr->write(CLK_SOURCE_HSI);
  swcr->write(0x00);
  uc->rom->write(base+10, 0x00); //cssr->write(0x00);
  uc->rom->write(base+11, 0x00); //cbeepr->write(0x00);
  uc->rom->write(base+12, 0x00); //hsicalr->write(0x00); // Actually a factory calibration value...
  uc->rom->write(base+13, 0x00); //hsitrimr->write(0x00);
  uc->rom->write(base+14, 0x00); //hsiunlckr->write(0x00);
  // FIXME: CLKREGCSR should reset to 0xb9 but that means LSE, HSE and LSI are powered down.
  // Since we don't emulate the auto-power up behaviour and reflect that in CLKREGCSR
  // we had best pretend they are always powered up.
  uc->rom->write(base+15, 0x81); //clkregcsr->write(0x00);
  pckenr1->write(0x00);
  pckenr2->write(0x80);
  pckenr3->write(0x00);

  set_osc(CLK_SOURCE_HSI);
}

void
cl_clk_all::write(class cl_memory_cell *cell, t_mem *val)
{
  if (cell == eckr)
    {
      t_mem preserve = 0xc0;

      // RM0031: 9.14.8: HSERDY: This bit is set and cleared by hardware.
      preserve |= CLK_ECKR_HSERDY;

      t_mem clk = cmsr->get();
      if (clk == CLK_SOURCE_HSE)
        {
          // RM0031 9.14.8: It [HSEON] cannot be cleared when HSE is selected as system
          // clock (CLK_SCSR register) or as the active CCO source or as active RTC
          // clock source.
          // FIXME: CCO and RTC are not yet implemented.
          preserve |= CLK_ECKR_HSEEN;
        }
      else if (clk == CLK_SOURCE_LSE)
        {
          // RM0031 9.14.8: It [LSEON] cannot be cleared when LSE is selected as system
          // clock (CLK_SCSR register), as active CCO source, as clock source for the
          // BEEP peripheral and BEEPAHALT is set or as active RTC.
          // FIXME: CCO, BEEP and RTC are not yet implemented.
          preserve |= CLK_ECKR_LSEEN;
        }

      *val = (*val & (~preserve)) | (eckr->get() & preserve);
    }
  else if (cell == ickr)
    {
      t_mem preserve = 0;

      // RM0031: 9.14.3: HSIRDY: This bit is set and cleared by hardware.
      // RM0031: 9.14.3: LSIRDY: This bit is set and cleared by hardware.
      preserve = CLK_ICKR_HSIRDY | CLK_ICKR_LSIRDY;

      t_mem clk = cmsr->get();
      if (clk == CLK_SOURCE_LSI)
        {
          // RM0031 9.14.3: It [LSION] cannot be cleared when LSI is selected as clock
          // master (CLK_SCSR register), as active CCO source, as clock source for the
          // BEEP peripheral while BEEPAHALT is set or as active clock source for RTC.
          // FIXME: CCO, BEEP and RTC are not yet implemented.
          preserve |= CLK_ICKR_LSIEN;
        }
      else if (clk == CLK_SOURCE_HSI)
        {
          // RM0031 9.14.3: It [HSIEN] cannot be cleared when HSI is selected as clock
          // master (CLK_CMSR register), as the active CCO source or if the safe oscillator
          // (AUX) is enabled.
          // FIXME: CCO and AUX are not yet implemented.
          preserve |= CLK_ICKR_HSIEN;
        }

      *val = (*val & (~preserve)) | (ickr->get() & preserve);
    }
  else if (cell == swr)
    {
      if (*val == CLK_SOURCE_LSE)
        {
          // RM0031 9.14.8: It [LSEON] is set by hardware whenever the LSE oscillator is
          // required, for example: [...] When switching to LSI clock (see CLK_SWR register).
          eckr->write(eckr->get() | CLK_ECKR_LSEEN);
        }
      else if (*val == CLK_SOURCE_HSE)
        {
          // RM0031 9.14.8: It [HSEON] is set by hardware whenever the HSE oscillator is
          // required, for example: [...] When switching to HSI clock (see CLK_SWR register).
          eckr->write(eckr->get() | CLK_ECKR_HSEEN);
        }
      else if (*val == CLK_SOURCE_LSI)
        {
          // RM0031 9.14.3: It [LSION] is set by hardware whenever the LSI oscillator is
          // required, for example: [...] When switching to LSI clock (see CLK_SWR register).
          ickr->write(ickr->get() | CLK_ICKR_LSIEN);
        }
      else if (*val == CLK_SOURCE_HSI)
        {
          // RM0031 9.14.3: It [HSION] is set by hardware whenever the HSI oscillator is
          // required, for example: [...] When switching to HSI clock (see CLK_SWR register).
          ickr->write(ickr->get() | CLK_ICKR_HSIEN);
        }
    }

  cl_clk::write(cell, val);
}

bool
cl_clk_all::tim(int id, t_mem *val)
{
  switch (id)
    {
    case 1:
      return pckenr2 && (*val & 0x02);
    case 2:
      return pckenr1 && (*val & 0x01);
    case 3:
      return pckenr1 && (*val & 0x02);
    case 4:
      return pckenr1 && (*val & 0x04);
    case 5:
      return pckenr3 && (*val & 0x02);
    }
  return false;
}

bool
cl_clk_all::usart(int id, t_mem *val)
{
  switch (id)
    {
    case 1:
      return pckenr1 && (*val & 0x20);
    case 2:
      return pckenr3 && (*val & 0x08);
    case 3:
      return pckenr3 && (*val & 0x10);
    }
  return false;
}


/* L101 */

cl_clk_l101::cl_clk_l101(class cl_uc *auc):
  cl_clk(auc)
{
}

int
cl_clk_l101::init(void)
{
  cl_clk::init();

  hwreg(uc->rom, base+0, "CKDIVR", "Clock divider register",
    "HSIDIV", 1, 0, "High speed internal clock prescalar",
    NULL);
  ckdivr = register_cell(uc->rom, base+0);

  hwreg(uc->rom, base+3, "PCKENR", "Peripheral clock gating register",
    "AWU_BEEP", 6, 6, "Peripheral clock enable AWU+BEEP",
    "USART",    5, 5, "Peripheral clock enable USART",
    "SPI",      4, 4, "Peripheral clock enable SPI",
    "I2C",      3, 3, "Peripheral clock enable I2C",
    "TIM4",     2, 2, "Peripheral clock enable TIM4",
    "TIM3",     1, 1, "Peripheral clock enable TIM3",
    "TIM2",     0, 0, "Peripheral clock enable TIM2",
    NULL);
  pckenr1 = register_cell(uc->rom, base+3);

  hwreg(uc->rom, base+5, "CCOR", "Configurable clock output register",
    "CCOSEL", 2, 1, "Configurable clock output selection",
    "CCOEN",  0, 0, "Configurable clock output enable",
    NULL);
  //ccor = register_cell(uc->rom, base+5);

  return 0;
}

void
cl_clk_l101::reset(void)
{
  ckdivr->write(0x03);
  pckenr1->write(0x00);
  uc->rom->write(base+5, 0x00); //ccor->write(0x00);
}

bool
cl_clk_l101::tim(int id, t_mem *val)
{
  switch (id)
    {
    case 2:
      return pckenr1 && (*val & 0x01);
    case 3:
      return pckenr1 && (*val & 0x02);
    case 4:
      return pckenr1 && (*val & 0x04);
    }
  return false;
}

bool
cl_clk_l101::usart(int id, t_mem *val)
{
  switch (id)
    {
    case 1:
      return pckenr1 && (*val & 0x20);
    }
  return false;
}


/* End of stm8.src/clk.cc */
