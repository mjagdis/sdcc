/*
 * Simulator of microcontrollers (stm8.src/timercl.h)
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

#ifndef STM8_TIMERCL_HEADER
#define STM8_TIMERCL_HEADER

#include "hwcl.h"


enum stm8_tim_cfg {
  stm8_tim_on= 0,
  stm8_tim_nuof_cfg= 1
};


enum tim_type {
  tim_basic = 0x00,
  tim_general = 0x01,
  tim_advanced = tim_general | 0x02,
};

static const bool tim_is_bidir = true;
static const bool tim_is_not_bidir = false;

static const bool tim_has_smcr = true;
static const bool tim_no_smcr = false;

enum tim_prescaler {
  tim_prescaler_16bit = 0,
  tim_prescaler_3bit = 0x07,
  tim_prescaler_4bit = 0x0f,
};


class cl_tim: public cl_hw
{
 private:
  tim_type type;
  t_addr base;
  int irqnr;
  int ier_offset;
  int capcom_channels;
  bool bidir;
  bool has_smcr;
  u16_t prescaler_mask;

  cl_memory_cell *cr1;
  cl_memory_cell *smcr;
  cl_memory_cell *ier;
  cl_memory_cell *sr1;
  cl_memory_cell *egr;
  cl_memory_cell *cntrh, *cntrl;
  cl_memory_cell *pscrh, *pscrl;
  cl_memory_cell *arrh, *arrl;

  bool clk_enabled;
  
  int cnt; // copy of counter value

  // Internal "regs"
  u16_t prescaler_cnt; // actual downcounter
  u16_t prescaler_preload; // start value of prescaler downcount
  u8_t prescaler_ms_buffer; // written MS buffered until LS write
  u8_t arr_ms_buffer; // written MS buffered until LS write
  u8_t timer_ls_buffer; // LS buffered at MS read

 protected:
  bool chained;

 public:
  cl_tim(class cl_uc *auc, int aid, tim_type type, int irqnr, t_addr base, int ier_offset, int capcom_channels, bool bidir, bool has_smcr, u16_t prescaler_mask);
  virtual int init(void);

  virtual int cfg_size(void) { return stm8_tim_nuof_cfg; }
  virtual char *cfg_help(t_addr addr);
 
  virtual int tick(int cycles);
  virtual void reset(void);
  virtual void happen(class cl_hw *where, enum hw_event he,
                      void *params);

  virtual t_mem read(class cl_memory_cell *cell);
  virtual void write(class cl_memory_cell *cell, t_mem *val);
  virtual t_mem conf_op(cl_memory_cell *cell, t_addr addr, t_mem *val);

  virtual u16_t set_counter(u16_t val);
  virtual void update_event(void);
  virtual u16_t get_arr(void);
  virtual u16_t calc_prescaler(void);
  
  virtual void print_info(class cl_console_base *con);
};


class cl_tim1_saf: public cl_tim
{
 public:
  cl_tim1_saf(class cl_uc *auc, int aid, t_addr abase): cl_tim(auc, aid, tim_advanced, 11, abase, 4, 4, tim_is_bidir, tim_has_smcr, tim_prescaler_16bit) { chained = true; }
};

class cl_tim1_all: public cl_tim
{
 public:
  cl_tim1_all(class cl_uc *auc, int aid, t_addr abase): cl_tim(auc, aid, tim_advanced, 23, abase, 5, 4, tim_is_bidir, tim_has_smcr, tim_prescaler_16bit) {}
};


// General purpose 2, 3, and 5
class cl_tim2_saf_a: public cl_tim
{
 public:
  cl_tim2_saf_a(class cl_uc *auc, int aid, t_addr abase): cl_tim(auc, aid, tim_general, 13, abase, 1, 3, tim_is_not_bidir, tim_no_smcr, tim_prescaler_4bit) {}
};

class cl_tim2_saf_b: public cl_tim
{
 public:
  cl_tim2_saf_b(class cl_uc *auc, int aid, t_addr abase): cl_tim(auc, aid, tim_general, 13, abase, 3, 3, tim_is_not_bidir, tim_no_smcr, tim_prescaler_4bit) {}
};

class cl_tim2_all: public cl_tim
{
 public:
  cl_tim2_all(class cl_uc *auc, int aid, t_addr abase): cl_tim(auc, aid, tim_general, 19, abase, 5, 2, tim_is_bidir, tim_has_smcr, tim_prescaler_3bit) {}
};

class cl_tim2_l101: public cl_tim
{
 public:
  cl_tim2_l101(class cl_uc *auc, int aid, t_addr abase): cl_tim(auc, aid, tim_general, 19, abase, 4, 2, tim_is_bidir, tim_has_smcr, tim_prescaler_3bit) {}
};


class cl_tim3_saf: public cl_tim
{
 public:
  cl_tim3_saf(class cl_uc *auc, int aid, t_addr abase): cl_tim(auc, aid, tim_general, 15, abase, 1, 2, tim_is_not_bidir, tim_no_smcr, tim_prescaler_4bit) {}
};

class cl_tim3_all: public cl_tim
{
 public:
  cl_tim3_all(class cl_uc *auc, int aid, t_addr abase): cl_tim(auc, aid, tim_general, 21, abase, 5, 2, tim_is_bidir, tim_has_smcr, tim_prescaler_3bit) {}
};

class cl_tim3_l101: public cl_tim
{
 public:
  cl_tim3_l101(class cl_uc *auc, int aid, t_addr abase): cl_tim(auc, aid, tim_general, 21, abase, 4, 2, tim_is_bidir, tim_has_smcr, tim_prescaler_3bit) {}
};


class cl_tim5_saf: public cl_tim
{
 public:
  cl_tim5_saf(class cl_uc *auc, int aid, t_addr abase): cl_tim(auc, aid, tim_general, 13, abase, 3, 3, tim_is_not_bidir, tim_has_smcr, tim_prescaler_4bit) { chained = true; }
};

class cl_tim5_all: public cl_tim
{
 public:
  cl_tim5_all(class cl_uc *auc, int aid, t_addr abase): cl_tim(auc, aid, tim_general, 27, abase, 5, 2, tim_is_bidir, tim_no_smcr, tim_prescaler_3bit) {}
};


// Basic 4 and 6
class cl_tim4_saf_a: public cl_tim
{
 public:
  cl_tim4_saf_a(class cl_uc *auc, int aid, t_addr abase): cl_tim(auc, aid, tim_basic, 23, abase, 1, 0, tim_is_not_bidir, tim_no_smcr, tim_prescaler_3bit) {}
};

class cl_tim4_saf_b: public cl_tim
{
 public:
  cl_tim4_saf_b(class cl_uc *auc, int aid, t_addr abase): cl_tim(auc, aid, tim_basic, 23, abase, 3, 0, tim_is_not_bidir, tim_no_smcr, tim_prescaler_3bit) {}
};

class cl_tim4_all: public cl_tim
{
 public:
  cl_tim4_all(class cl_uc *auc, int aid, t_addr abase): cl_tim(auc, aid, tim_basic, 25, abase, 4, 0, tim_is_not_bidir, tim_no_smcr, tim_prescaler_4bit) {}
};

class cl_tim4_l101: public cl_tim
{
 public:
  cl_tim4_l101(class cl_uc *auc, int aid, t_addr abase): cl_tim(auc, aid, tim_basic, 25, abase, 3, 0, tim_is_not_bidir, tim_no_smcr, tim_prescaler_4bit) {}
};

class cl_tim6_saf: public cl_tim
{
 public:
  cl_tim6_saf(class cl_uc *auc, int aid, t_addr abase): cl_tim(auc, aid, tim_basic, 23, abase, 6, 0, tim_is_not_bidir, tim_no_smcr, tim_prescaler_3bit) { chained = true; }
};


#endif

/* End of stm8.src/timercl.h */
