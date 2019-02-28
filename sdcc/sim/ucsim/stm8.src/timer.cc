/*
 * Simulator of microcontrollers (stm8.src/timer.cc)
 *
 * Copyright (C) 2016,16 Drotos Daniel, Talker Bt.
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

#include "itsrccl.h"

#include "clkcl.h"
#include "timercl.h"


enum tim_cr1_bits {
  cen	= 0x01,
  udis	= 0x02,
  urs	= 0x04,
  opm	= 0x08,
  dir	= 0x10,
  cms	= 0x60,
  cms0	= 0x00, // edge aligned mode
  cms1	= 0x20, // center aligned 1 (irq during downcount)
  cms2	= 0x40, // center aligned 2 (irq during upcount)
  cms3	= 0x60, // center aligned 3 (irq in both counting dir)
  arpe	= 0x80
};

enum tim_smcr_bits {
  msm   = 0x80,
  ts    = 0x70,
  occs  = 0x10,
  sms   = 0x07, // Clock/trigger/slave mode selection
  sms0  = 0x00  //   disabled
};

enum tim_sr1_bits {
  uif	= 0x01
};

enum tim_ier_bits {
  uie	= 0x01
};

enum tim_egr_bits {
  ug	= 0x01
};

cl_tim::cl_tim(class cl_uc *auc, int aid, tim_type atype, int airqnr, t_addr abase, int aier_offset, int acapcom_channels, bool abidir, bool ahas_smcr, u16_t aprescaler_mask):
  cl_hw(auc, HW_TIMER, aid, "tim")
{
  type= atype;
  irqnr= airqnr;
  base= abase;
  ier_offset= aier_offset;
  capcom_channels = acapcom_channels;
  bidir= abidir;
  has_smcr= ahas_smcr;
  chained = false;
  prescaler_mask = aprescaler_mask;
  cr1= NULL;
  smcr= NULL;
  ier= NULL;
  sr1= NULL;
  egr= NULL;
  cntrh= cntrl= NULL;
  pscrh= pscrl= NULL;
  arrh= arrl= NULL;
}

int
cl_tim::init(void)
{
  cl_hw::init();

  chars s("tim");
  s.append("%d", id);
  set_name(s);

  clk_enabled= false;

  int offset = ier_offset;

  hwreg(uc->rom, base+0, "CR1", "Control register 1",
    "ARPE", 7, 7,  "Auto-reload preload enable",
    "OPM",  3, 3,  "One-pulse mode",
    "URS",  2, 2,  "Update request source",
    "UDIS", 1, 1,  "Update disable",
    "CEN",  0, 0,  "Counter enable",
    NULL);
  if (type == tim_advanced)
    hwreg("CR1",
      "CMS", 6, 5, "Center-aligned mode selection",
      NULL);
  cr1 = register_cell(uc->rom, base+0);

  if (type == tim_advanced || chained)
    {
      hwreg(uc->rom, base+1, "CR2", "Control register 2", NULL);
      if (chained)
        hwreg("CR2",
          "MMS", 6, 4, "Master mode selection",
          NULL);
      if (type == tim_advanced)
        hwreg("CR2",
          "COMS", 2, 2, "Capture/compare control update selction",
          "CCPC", 0, 0, "Capture/compare preloaded control",
          NULL);
      //cr2 = register_cell(uc->rom, base+1);
    }

  if (has_smcr)
    {
      hwreg(uc->rom, base+2, "SMCR", "Slave mode control register",
        "MSM", 7, 7, "Master/slave mode",
        "TS",  6, 4, "Trigger selection",
        "SMS", 2, 0, "Clock/trigger/slave mode selection",
        NULL);
      smcr = register_cell(uc->rom, base+2);
    }

  if (type == tim_advanced)
    {
      hwreg(uc->rom, base+3, "ETR", "External trigger register",
        "ETP",  7, 7,  "External trigger polarity",
        "ECE",  6, 6,  "External clock enable",
        "ETPS", 5, 4,  "External trigger prescaler",
        "ETF",  3, 0,  "External trigger filter",
        NULL);
      //etr = register_cell(uc->rom, base+3);
    }
 
  //hwreg(uc->rom, base+4, "DER", "DMA request enable register", NULL);
  //der = register_cell(uc->rom, base+4);

  hwreg(uc->rom, base+offset, "IER", "Interrupt enable register",
    "TIE", 6, 6, "Trigger interrupt enable",
    "UIE", 0, 0, "Update interrupt enable",
    NULL);
  if ((type & tim_general))
    {
      hwreg("IER",
        "CC3IE", 3, 3, "Capture/compare 3 interrupt enable",
        "CC2IE", 2, 2, "Capture/compare 2 interrupt enable",
        "CC1IE", 1, 1, "Capture/compare 1 interrupt enable",
        NULL);
      if (type == tim_advanced)
        hwreg("IER",
          "BIE",   7, 7, "Break interrupt enable",
          "COMIE", 5, 5, "Commutation interrupt enable",
          "CC4IE", 4, 4, "Capture/compare 4 interrupt enable",
          NULL);
    }
  ier = register_cell(uc->rom, base+offset++);

  if ((type & tim_general))
    {
      hwreg(uc->rom, base+offset, "SR", "Status register",
        "TIF", 6, 6, "Trigger interrupt flag",
        "UIF", 0, 0, "Update interrupt flag",
        NULL);
      sr1 = register_cell(uc->rom, base+offset++);
    }
  else
    {
      hwreg(uc->rom, base+offset, "SR1", "Status register 1",
        "TIF",   6, 6, "Trigger interrupt flag",
        "CC3IF", 3, 3, "Capture/compare 3 interrupt flag",
        "CC2IF", 2, 2, "Capture/compare 2 interrupt flag",
        "CC1IF", 1, 1, "Capture/compare 1 interrupt flag",
        "UIF",   0, 0, "Update interrupt flag",
        NULL);
      if (type == tim_advanced)
        hwreg("SR1",
          "BIF",   7, 7, "Break interrupt flag",
	  "COMIF", 5, 5, "Commutation interrupt flag",
          "CC4IF", 4, 4, "Capture/compare 4 interrupt flag",
          NULL);
      sr1 = register_cell(uc->rom, base+offset++);
    }

  if ((type & tim_general))
    {
      hwreg(uc->rom, base+offset, "SR2", "Status register 2",
        "CC3OF", 3, 3, "Capture/compare 3 overcapture flag",
        "CC2OF", 2, 2, "Capture/compare 2 overcapture flag",
        "CC1OF", 1, 1, "Capture/compare 1 overcapture flag",
        NULL);
      if (type == tim_advanced)
        hwreg("SR2",
          "CC4OF", 4, 4, "Capture/compare 4 overcapture flag",
          NULL);
      offset++; //sr2 = register_cell(uc->rom, base+offset++);
    }

  hwreg(uc->rom, base+offset, "EGR", "Event generation register",
    "TG", 6, 6, "Trigger generation",
    "UG", 0, 0, "Update generation",
    NULL);
  if ((type & tim_general))
    {
      hwreg("EGR",
        "CC3G", 3, 3, "Capture/compare 3 generation",
        "CC2G", 2, 2, "Capture/compare 2 generation",
        "CC1G", 1, 1, "Capture/compare 1 generation",
        NULL);
      if (type == tim_advanced)
        hwreg("EGR",
          "BG",   7, 7, "Break generation",
          "COMG", 5, 5, "Capture/compare control update generation",
          "CC4G", 4, 4, "Capture/compare 4 generation",
          NULL);
    }
  egr = register_cell(uc->rom, base+offset++);

  if (capcom_channels > 0)
    {
      hwreg(uc->rom, base+offset, "CCMR1", "Capture/compare mode register 1",
        "OC1M",  6, 4, "Output compare 1 mode",
	"OC1PE", 3, 3, "Output compare 1 preload enable",
	"CC1S",  1, 0, "Capture/compare 1 selection",
        NULL);
      if (type == tim_advanced)
        hwreg("CCMR1",
          "OC1CE", 7, 7, "Output compare 1 clear enable",
          "OC1FE", 2, 2, "Output compare 1 fast enable",
          NULL);
      offset++; //ccmr1 = register_cell(uc->rom, base+offset++);
    }
  if (capcom_channels > 1)
    {
      hwreg(uc->rom, base+offset, "CCMR2", "Capture/compare mode register 2",
        "OC2M",  6, 4, "Output compare 2 mode",
	"OC2PE", 3, 3, "Output compare 2 preload enable",
	"CC2S",  1, 0, "Capture/compare 2 selection",
        NULL);
      if (type == tim_advanced)
        hwreg("CCMR2",
          "OC2CE", 7, 7, "Output compare 2 clear enable",
          "OC2FE", 2, 2, "Output compare 2 fast enable",
          NULL);
      offset++; //ccmr2 = register_cell(uc->rom, base+offset++);
    }
  if (capcom_channels > 2)
    {
      hwreg(uc->rom, base+offset, "CCMR3", "Capture/compare mode register 3",
        "OC3M",  6, 4, "Output compare 3 mode",
	"OC3PE", 3, 3, "Output compare 3 preload enable",
	"CC3S",  1, 0, "Capture/compare 3 selection",
        NULL);
      if (type == tim_advanced)
        hwreg("CCMR3",
          "OC3CE", 7, 7, "Output compare 3 clear enable",
          "OC3FE", 2, 2, "Output compare 3 fast enable",
          NULL);
      offset++; //ccmr3 = register_cell(uc->rom, base+offset++);
    }
  if (capcom_channels > 3)
    {
      hwreg(uc->rom, base+offset, "CCMR4", "Capture/compare mode register 4",
        "OC4M",  6, 4, "Output compare 4 mode",
	"OC4PE", 3, 3, "Output compare 4 preload enable",
	"CC4S",  1, 0, "Capture/compare 4 selection",
        NULL);
      if (type == tim_advanced)
        hwreg("CCMR4",
          "OC4CE", 7, 7, "Output compare 4 clear enable",
          "OC4FE", 2, 2, "Output compare 4 fast enable",
          NULL);
      offset++; //ccmr4 = register_cell(uc->rom, base+offset++);
    }

  if (capcom_channels > 0)
    {
      hwreg(uc->rom, base+offset, "CCER1", "Capture/compare enable register 1",
        "CC2P", 5, 5, "Capture/compare 2 output polarity",
        "CC2E", 4, 4, "Capture/compare 2 output enable",
        "CC1P", 1, 1, "Capture/compare 1 output polarity",
        "CC1E", 0, 0, "Capture/compare 1 output enable",
        NULL);
      if (type == tim_advanced)
        hwreg("CCER1",
          "CC2NP", 7, 7, "Capture/compare 2 complementary output polarity",
	  "CC2NE", 6, 6, "Capture/compare 2 complementary output enable",
          "CC1NP", 3, 3, "Capture/compare 1 complementary output polarity",
	  "CC1NE", 2, 2, "Capture/compare 1 complementary output enable",
          NULL);
      offset++; //ccer1 = register_cell(uc->rom, base+offset++);
    }

  if (capcom_channels > 1)
    {
      hwreg(uc->rom, base+offset, "CCER2", "Capture/compare enable register 2",
        "CC3P", 1, 1, "Capture/compare 3 output polarity",
        "CC3E", 0, 0, "Capture/compare 3 output enable",
        NULL);
      if (type == tim_advanced)
        hwreg("CCER2",
          "CC4P",  5, 5, "Capture/compare 4 complementary output polarity",
	  "CC4E",  4, 4, "Capture/compare 4 complementary output enable",
          "CC3NP", 3, 3, "Capture/compare 3 complementary output polarity",
	  "CC3NE", 2, 2, "Capture/compare 3 complementary output enable",
          NULL);
      offset++; //ccer1 = register_cell(uc->rom, base+offset++);
    }

  if ((type & tim_general))
    {
      hwreg(uc->rom, base+offset, "CNTRH", "Counter high", NULL);
      cntrh = register_cell(uc->rom, base+offset++);
      hwreg(uc->rom, base+offset, "CNTRL", "Counter low", NULL);
      cntrl = register_cell(uc->rom, base+offset++);
    }
  else
    {
      hwreg(uc->rom, base+offset, "CNTR", "Counter", NULL);
      cntrl = register_cell(uc->rom, base+offset++);
    }

  if (!prescaler_mask)
    {
      hwreg(uc->rom, base+offset, "PSCRH", "Pre-scaler high", NULL);
      pscrh = register_cell(uc->rom, base+offset++);
      hwreg(uc->rom, base+offset, "PSCRL", "Pre-scaler low", NULL);
      pscrl = register_cell(uc->rom, base+offset++);
    }
  else
    {
      hwreg(uc->rom, base+offset, "PSCR", "Pre-scaler register",
        "PSC", (prescaler_mask == tim_prescaler_3bit ? 2 : 3), 0, "Prescaler value",
        NULL);
      pscrl = register_cell(uc->rom, base+offset++);
    }

  if ((type & tim_general))
    {
      hwreg(uc->rom, base+offset, "ARRH", "Auto-reload register high", NULL);
      arrh = register_cell(uc->rom, base+offset++);
      hwreg(uc->rom, base+offset, "ARRL", "Auto-reload register low", NULL);
      arrl = register_cell(uc->rom, base+offset++);
    }
  else
    {
      hwreg(uc->rom, base+offset, "ARR", "Auto-reload register", NULL);
      arrl = register_cell(uc->rom, base+offset++);
    }

  chars desc = cchars(get_name()) + chars("_update");
  class cl_it_src *is;
  uc->it_sources->add(is = new cl_it_src(uc, irqnr,
                                         ier, uie,
                                         sr1, uif,
                                         0x8008+irqnr*4, false, false,
                                         desc,
                                         30*10+1));
  //printf("%s: irqnr %d\n", get_name(), irqnr);
  is->init();

  return 0;
}

char *
cl_tim::cfg_help(t_addr addr)
{
  switch (addr)
    {
    case stm8_tim_on: return (char*)"Turn simulation of timer on/off (bool, RW)";
    }
  return (char*)"Not used";
}

int
cl_tim::tick(int cycles)
{
  if (!on ||
      !clk_enabled)
    return resGO;
  
  while (cycles--)
    {
      // count prescaler
      if (prescaler_cnt)
        prescaler_cnt--;

      if (prescaler_cnt == 0)
        {
          prescaler_cnt= calc_prescaler() - 1;

          // The STM8 Reference Manuals state that CEN gates the _output_
          // of the prescaler, CLK_CNT, rather than the input. Setting CEN
          // should normally be followed by a forced update event.
          if (cr1->get() & cen)
            {
              u8_t c1= cr1->get();
              if (!bidir || !(cr1->get() & dir))
                {
                  // up
                  set_counter(cnt+1);
                  if (cnt == get_arr())
                    {
                      if (!bidir || (c1 & cms) == cms0)
                        // edge aligned
                        set_counter(0);
                      else
                        // center aligned
                        cr1->write(c1|= dir);
                      if ((c1 & udis) == 0)
                        update_event();
                    }
                }
              else
                {
                  // down
                  set_counter(cnt-1);
                  if (cnt == 0)
                    {
                      if ((c1 & cms) == cms0)
                        // edge aligned
                        set_counter(get_arr());
                      else
                        // center aligned
                        cr1->write(c1&= ~dir);
                      if ((c1 & udis) == 0)
                        update_event();
                    }
                }
            }
        }
    }
  
  return resGO;
}

void
cl_tim::reset(void)
{
  int i;
  
  cnt= 0;
  prescaler_cnt= 0;
  prescaler_preload= 0;

  for (i= 0; i<32+6; i++)
    uc->rom->get_cell(base + i)->write(0);
  if (arrh)
    arrh->write(0xff);
  arrl->write(0xff);

  update_event();
  sr1->write_bit0(uif);
}

void
cl_tim::happen(class cl_hw *where, enum hw_event he,
	       void *params)
{
  if ((he == EV_CLK_ON) ||
      (he == EV_CLK_OFF))
    {
      cl_clk_event *e= (cl_clk_event *)params;
      if ((e->cath == HW_TIMER) &&
	  (e->id == id))
	clk_enabled= he == EV_CLK_ON;
    }
}

t_mem
cl_tim::read(class cl_memory_cell *cell)
{
  t_mem v= cell->get();

  if (conf(cell, NULL))
    return v;

  if (cell == pscrl)
    v= prescaler_preload && 0xff;
  else if (cell == pscrh)
    v= (prescaler_preload >> 8) & 0xff;

  else if (cell == cntrh)
    timer_ls_buffer= cntrl->get();
  else if (cell == cntrl)
    {
      if (cntrh)
	v= timer_ls_buffer;
    }
  
  return v;
}

void
cl_tim::write(class cl_memory_cell *cell, t_mem *val)
{
  if (conf(cell, val))
    return;
  
  if (conf(cell, NULL))
    return;

  if (cell == cr1)
    {
      // DIR is read only when a bidirectional timer is configured
      // in centre-aligned or encoder mode.
      u8_t v= cell->get();
      if (bidir && ((v & cms) != 0 || (smcr && (smcr->get() & sms) == sms0)))
        *val= (*val & (~dir)) | (v & dir);
    }
  else if (cell == egr)
    {
      if (*val & ug)
	{
	  update_event();
	  prescaler_cnt= calc_prescaler() - 1;
	}
      *val= 0;
    }
  else if (cell == pscrh)
    {
      prescaler_ms_buffer= *val;
    }
  else if (cell == pscrl)
    {
      prescaler_preload= *val;
      if (pscrh)
	prescaler_preload+= prescaler_ms_buffer * 256;
    }
    
  else if (cell == arrh)
    {
      if ((cr1->get() & arpe) != 0)
        arr_ms_buffer= *val;
    }
  else if (cell == arrl)
    {
      if ((cr1->get() & arpe) != 0)
	{
	  if (arrh)
	    arrh->write(arr_ms_buffer);
	  if ((cr1->get() & arpe) == 0)
	    set_counter(arr_ms_buffer*256 + *val);
	}
    }
}

t_mem
cl_tim::conf_op(cl_memory_cell *cell, t_addr addr, t_mem *val)
{
  switch ((enum stm8_tim_cfg)addr)
    {
    case stm8_tim_on:
      if (val)
	{
	  if (*val)
	    on= true;
	  else
	    on= false;
	}
      else
	cell->write(on?1:0);
      break;
    case stm8_tim_nuof_cfg:
      break;
    }
  return cell->get();
}


u16_t
cl_tim::set_counter(u16_t val)
{
  cnt= val;
  cntrl->write(val & 0xff);
  if (cntrh)
    cntrh->write(val >> 8);
  return val;
}

void
cl_tim::update_event(void)
{
  u8_t c1= cr1->get();

  if (c1 & opm)
    cr1->write_bit0(cen);
  else
    {
      if (!bidir || !(cr1->get() & dir))
	{
	  // up
	  set_counter(0);
	}
      else
	{
	  // down
	  set_counter(get_arr());
	}
    }
  sr1->write_bit1(uif);
}

u16_t
cl_tim::get_arr()
{
  if (arrh)
    return arrh->get() * 256 + arrl->get();
  return arrl->get();
}

u16_t
cl_tim::calc_prescaler()
{
  return (prescaler_mask ? (1 << (prescaler_preload & prescaler_mask)) : prescaler_preload + 1);
}

void
cl_tim::print_info(class cl_console_base *con)
{
  u8_t c1= cr1->get();
  // features
  con->dd_printf("Simulation of %s is %s\n", get_name(), on?"ON":"OFF");
  con->dd_printf("%s %d bit %s counter at 0x%06x\n", get_name(),
                 (arrh ? 16 : 8),
                 (bidir ? "Up/Down" : "Up"),
                 base);
  // actual values
  con->dd_printf("clk= %s\n", clk_enabled?"enabled":"disabled");
  con->dd_printf("cnt= 0x%04x %d %s\n", cnt, cnt, (c1&cen)?"on":"off");
  con->dd_printf("dir= %s\n", (c1&dir)?"down":"up");
  con->dd_printf("prs= 0x%04x %d of 0x%04x %d\n",
		 prescaler_cnt, prescaler_cnt,
		 calc_prescaler(), calc_prescaler());
  con->dd_printf("arr= 0x%04x %d\n", get_arr(), get_arr());
  print_cfg_info(con);
}


/* End of stm8.src/timer.cc */
