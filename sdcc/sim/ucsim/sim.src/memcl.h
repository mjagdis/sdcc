/*
 * Simulator of microcontrollers (sim.src/memcl.h)
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

#ifndef SIM_MEMCL_HEADER
#define SIM_MEMCL_HEADER

#include <stdio.h>

#include "ddconfig.h"

// prj
#include "stypes.h"
#include "pobjcl.h"

// gui.src
#include "guiobjcl.h"


class cl_event_handler;

// Cell flags
enum cell_flag {
  CELL_NONE		= 0x00,
  CELL_VAR		= 0x01, /* At least one variable points to it */
  CELL_INST		= 0x04,	/* Marked as instruction */
  CELL_FETCH_BRK	= 0x08,	/* Fetch breakpoint */
  CELL_READ_ONLY	= 0x10, /* Cell is readonly */
  CELL_DECODED		= 0x80	/* Cell is decoded */
};


#define CELL_GENERAL	(CELL_NORMAL|CELL_INST|CELL_FETCH_BRK)


/*
 * 3rd version of memory system
 */

class cl_memory: public cl_base
{
public:
  t_addr start_address;
protected:
  class cl_uc *uc;
  t_addr size;
public:
  char *addr_format, *data_format;
  int addr_format_width, data_format_width;
  int width; // in bits
  t_mem data_mask;
  bool hidden;
protected:
  t_addr dump_finished;
public:
  cl_memory(const char *id, t_addr asize, int awidth);
  virtual ~cl_memory(void);
  virtual int init(void);

  t_addr get_start_address(void) { return(start_address); }
  t_addr get_size(void) { return(size); }
  virtual void set_uc(class cl_uc *auc) { uc= auc; }
  virtual bool valid_address(t_addr addr);
  virtual t_addr inc_address(t_addr addr, int val);
  virtual t_addr inc_address(t_addr addr);
  virtual t_addr validate_address(t_addr addr);

  virtual bool is_chip(void) { return(false); }
  virtual bool is_address_space(void) { return(false); }

  virtual void err_inv_addr(t_addr addr);
  virtual void err_non_decoded(t_addr addr);

  virtual class cl_var *var_for(t_addr addr, int bitnr_high, int bitnr_low, t_index &var_i);
  virtual t_addr dump(class cl_console_base *con, int smart, t_addr start, t_addr stop, int bitnr_high, int bitnr_low, int bpl);
  virtual t_addr dump(class cl_console_base *con, int smart, t_addr start, t_addr stop, int bpl) {
    return dump(con, smart, start, stop, -1, -1, bpl);
  }
  virtual t_addr dump(class cl_console_base *con, t_addr addr, int bitnr_high, int bitnr_low) {
    return dump(con, 2, addr, addr, bitnr_high, bitnr_low, -1);
  }
  virtual t_addr dump(class cl_console_base *con, t_addr start, t_addr stop, int bpl) {
    return dump(con, 1, start, stop, bpl);
  }
  virtual t_addr dump_s(t_addr start, t_addr stop, int bpl, class cl_f *f);
  virtual t_addr dump_b(t_addr start, t_addr stop, int bpl, class cl_f *f);
  virtual t_addr dump_i(t_addr start, t_addr stop, int bpl, class cl_f *f);
  //virtual t_addr dump(class cl_f *f) { return(dump(df_smart, -1, -1, -1, f)); }
  virtual bool search_next(bool case_sensitive,
			   t_mem *array, int len, t_addr *addr);


  virtual t_addr lowest_valid_address(void) { return(start_address); }
  virtual t_addr highest_valid_address(void) { return(start_address+size-1); }

  virtual t_mem read(t_addr addr)=0;
  virtual t_mem read(t_addr addr, enum hw_cath skip)=0;
  virtual t_mem get(t_addr addr)=0;
  virtual t_mem write(t_addr addr, t_mem val)=0;
  virtual void set(t_addr addr, t_mem val)=0;
  virtual void set_bit1(t_addr addr, t_mem bits)=0;
  virtual void set_bit0(t_addr addr, t_mem bits)=0;

  virtual void print_info(chars pre, class cl_console_base *con);
};


/*
 * Operators for memory cells
 */

class cl_banker;

class cl_memory_operator: public cl_base
{
protected:
  //t_addr address;
  t_mem mask;
  class cl_memory_operator *next_operator;
  class cl_memory_cell *cell;
public:
  cl_memory_operator(class cl_memory_cell *acell/*, t_addr addr*/);

  virtual class cl_memory_operator *get_next(void) { return(next_operator); }
  virtual void set_next(class cl_memory_operator *next) { next_operator= next;}

  virtual bool match(class cl_hw *the_hw) { return(false); }
  virtual bool match(class cl_brk *brk) { return(false); }

  virtual t_mem read(void);
  virtual t_mem read(enum hw_cath skip) { return(read()); }
  virtual t_mem write(t_mem val);
};

class cl_bank_switcher_operator: public cl_memory_operator
{
 protected:
  class cl_banker *banker;
 public:
  cl_bank_switcher_operator(class cl_memory_cell *acell/*, t_addr addr*/,
			    class cl_banker *the_banker);
  
  virtual t_mem write(t_mem val);
};

class cl_hw_operator: public cl_memory_operator
{
protected:
  class cl_hw *hw;
public:
  cl_hw_operator(class cl_memory_cell *acell/*, t_addr addr*/,
		 /*t_mem *data_place, t_mem the_mask,*/ class cl_hw *ahw);

  virtual bool match(class cl_hw *the_hw) { return(hw == the_hw); }

  virtual t_mem read(void);
  virtual t_mem read(enum hw_cath skip);
  virtual t_mem write(t_mem val);
};

class cl_event_break_operator: public cl_memory_operator
{
protected:
  class cl_uc *uc;
  class cl_brk *bp;
public:
 cl_event_break_operator(class cl_memory_cell *acell/*, t_addr addr*/,
			  class cl_uc *auc, class cl_brk *the_bp):
  cl_memory_operator(acell/*, addr*/)
  {
    uc= auc;
    bp= the_bp;
  }

  virtual bool match(class cl_brk *brk) { return(bp == brk); }
};

class cl_write_operator: public cl_event_break_operator
{
public:
  cl_write_operator(class cl_memory_cell *acell/*, t_addr addr*/,
		    class cl_uc *auc, class cl_brk *the_bp);

  virtual t_mem write(t_mem val);
};

class cl_read_operator: public cl_event_break_operator
{
public:
  cl_read_operator(class cl_memory_cell *acell/*, t_addr addr*/,
		   class cl_uc *auc, class cl_brk *the_bp);

  virtual t_mem read(void);
};


/*
 * version 3 of cell
 */

class cl_memory_cell: public cl_abs_base
{
#ifdef STATISTIC
 public:
  unsigned long nuof_writes, nuof_reads;
#endif
 public:
  class cl_memory_chip *chip;
  t_addr chipaddr;
  t_mem mask;
 protected:
  t_mem *data;
  t_mem def_data;
  class cl_memory_operator *operators;
 public:
  cl_memory_cell(cl_memory_chip *chip, t_addr addr);
  virtual ~cl_memory_cell(void);
  virtual int init(void);

  virtual t_mem get_mask(void) { return(mask); }
  virtual void set_mask(t_mem m) { mask= m; }
  virtual /*TYPE_UBYTE*/uchar get_flags(void);
  virtual bool get_flag(enum cell_flag flag);
  virtual void set_flag(enum cell_flag flag, bool val);
  
  virtual void decode(class cl_memory_chip *chip, t_addr addr) { this->chip = chip; this->chipaddr = addr; }
  virtual void decode(t_mem *data_ptr) {}

  virtual t_mem read(void);
  virtual t_mem read(enum hw_cath skip);
  virtual t_mem get(void);
  virtual t_mem write(t_mem val);
  virtual t_mem set(t_mem val);
  virtual t_mem download(t_mem val);
  
  virtual t_mem add(long what);
  virtual t_mem wadd(long what);

  virtual void set_bit1(t_mem bits);
  virtual void write_bit1(t_mem bits);
  virtual void set_bit0(t_mem bits);
  virtual void write_bit0(t_mem bits);
  virtual void toggle_bits(t_mem bits);
  virtual void wtoggle_bits(t_mem bits);
  
  virtual void append_operator(class cl_memory_operator *op);
  virtual void prepend_operator(class cl_memory_operator *op);
  virtual void del_operator(class cl_brk *brk);
  virtual void del_operator(class cl_hw *hw);

  virtual class cl_memory_cell *add_hw(class cl_hw *hw/*, t_addr addr*/);
  virtual void remove_hw(class cl_hw *hw);

  virtual void print_info(chars pre, class cl_console_base *con);
  virtual void print_operators(cchars pre, class cl_console_base *con);
};

class cl_dummy_cell: public cl_memory_cell
{
public:
  cl_dummy_cell(cl_memory_chip *chip, t_addr addr): cl_memory_cell(chip, addr) {}
  class cl_dummy_cell *cell_init(void) { init(); return this; }

  virtual t_mem write(t_mem val);
  virtual t_mem set(t_mem val);

  virtual uchar get_flags(void) { return 0; }
  virtual bool get_flag(enum cell_flag flag) { return 0; }
  virtual void set_flag(enum cell_flag flag, bool val) {}
};


/*
 * Address space
 */

class cl_memory_chip;

class cl_address_space: public cl_memory
{
 public:
  static class cl_memory_chip *dummy_chip;
  static class cl_dummy_cell *dummy;
 public:
  class cl_decoder_list *decoders;
 public:
  cl_address_space(const char *id, t_addr astart, t_addr asize, int awidth);
  virtual ~cl_address_space(void);

  virtual bool is_address_space(void) { return(true); }

  virtual void decode(t_addr as_begin, class cl_memory_chip *chip, t_addr chip_begin, t_addr size);
  virtual void decode(t_addr as_begin, class cl_memory_chip *chip);

  virtual t_mem read(t_addr addr);
  virtual t_mem read(t_addr addr, enum hw_cath skip);
  virtual t_mem get(t_addr addr);
  virtual t_mem write(t_addr addr, t_mem val);
  virtual void set(t_addr addr, t_mem val);
  virtual void download(t_addr, t_mem val);
  
  virtual t_mem wadd(t_addr addr, long what);
  virtual void set_bit1(t_addr addr, t_mem bits);
  virtual void set_bit0(t_addr addr, t_mem bits);
  
  virtual class cl_memory_cell *get_cell(t_addr addr);
  virtual int get_cell_flag(t_addr addr);
  virtual bool get_cell_flag(t_addr addr, enum cell_flag flag);
  virtual void set_cell_flag(t_addr addr, bool set_to, enum cell_flag flag);
  virtual void set_cell_flag(t_addr start_addr, t_addr end_addr, bool set_to, enum cell_flag flag);
  virtual class cl_memory_cell *search_cell(enum cell_flag flag, bool value,
					    t_addr *addr);
  virtual bool is_owned(class cl_memory_cell *cell, t_addr *addr);
  
  virtual class cl_address_decoder *get_decoder_of(t_addr addr);
  virtual void undecode_area(class cl_address_decoder *skip,
			     t_addr begin, t_addr end, class cl_console_base *con);

  virtual class cl_memory_cell *register_hw(t_addr addr, class cl_hw *hw,
					    bool announce);
  //virtual void unregister_hw(class cl_hw *hw);

  virtual void set_brk(t_addr addr, class cl_brk *brk);
  virtual void del_brk(t_addr addr, class cl_brk *brk);

#ifdef STATISTIC
  virtual unsigned long get_nuof_reads(void) { return(0); }
  virtual unsigned long get_nuof_writes(void) { return(0); }
  virtual void set_nuof_reads(unsigned long value) {}
  virtual void set_nuof_writes(unsigned long value) {}
#endif

  virtual void print_info(chars pre, class cl_console_base *con);
};

class cl_memory_list: public cl_list
{
protected:
  class cl_uc *uc;
public:
  cl_memory_list(class cl_uc *the_uc, const char *name);
  virtual t_index add(class cl_memory *mem);
};


/*
 * Memory chip (storage)
 */

class cl_memory_chip: public cl_memory
{
protected:
  class cl_memory_cell **cella;
  t_mem *array;
  uchar *flags;
  int init_value;
  bool array_is_mine;
public:
  cl_memory_chip(const char *id, int asize, int awidth, int initial= -1);
  cl_memory_chip(const char *id, int asize, int awidth, t_mem *aarray);
  virtual ~cl_memory_chip(void);
  virtual int init(void);
  virtual class cl_memory_chip *chip_init(void) { init(); return this; }

  virtual bool is_chip(void) { return(true); }

  virtual class cl_memory_cell *get_cell(t_addr addr);
  virtual t_mem *get_slot(t_addr addr);

  virtual uchar get_flags(t_addr addr);
  virtual bool get_flag(t_addr addr, enum cell_flag flag);
  virtual void set_flag(t_addr addr, enum cell_flag flag, bool val);

  virtual t_mem read(t_addr addr);
  virtual t_mem read(t_addr addr, enum hw_cath skip);
  virtual t_mem get(t_addr addr);
  virtual t_mem write(t_addr addr, t_mem val);
  virtual void download(t_addr addr, t_mem val);
  virtual void set(t_addr addr, t_mem val);
  virtual void set_bit1(t_addr addr, t_mem bits);
  virtual void set_bit0(t_addr addr, t_mem bits);
  virtual t_mem wadd(t_addr addr, long what);
  virtual bool is_owned(class cl_memory_cell *cell, t_addr *addr, int start, int end);

  virtual void print_info(chars pre, class cl_console_base *con);
};

  
/*
 * Address decoder
 */

class cl_address_decoder: public cl_base
{
public:
  class cl_address_space *address_space;
  class cl_memory_chip *chip;
  t_addr as_begin, as_end;
  t_addr chip_begin;
  bool activated;
public:
  cl_address_decoder(class cl_memory *as, class cl_memory *chip,
		     t_addr asb, t_addr ase, t_addr cb);
  virtual ~cl_address_decoder(void);
  virtual int init(void);
  virtual bool is_banker() { return false; }

  virtual bool activate(class cl_console_base *con);

  t_addr as_to_chip(t_addr addr) { return addr - as_begin + chip_begin; }
  t_addr chip_to_as(t_addr addr) { return addr - chip_begin + as_begin; }

  virtual bool fully_covered_by(t_addr begin, t_addr end);
  virtual bool is_in(t_addr begin, t_addr end);
  virtual bool covers(t_addr begin, t_addr end);

  virtual bool shrink_out_of(t_addr begin, t_addr end);
  virtual class cl_address_decoder *split(t_addr begin, t_addr end);

  virtual void print_info(chars pre, class cl_console_base *con);
};


/*
 * Address decoder with bank switching support
 */

class cl_banker: public cl_address_decoder
{
 protected:
  struct bank_def {
    class cl_memory_chip *chip;
    t_addr chip_begin;
  };
  struct bank_def *banks;
  class cl_address_space *banker_as;
  t_addr banker_addr;
  t_mem banker_mask;
  int nuof_banks;
  int bank;
  int shift_by;
 public:
  cl_banker(class cl_address_space *the_banker_as,
	    t_addr the_banker_addr,
	    t_mem the_banker_mask,
	    class cl_address_space *the_as,
	    t_addr the_asb,
	    t_addr the_ase);

  virtual ~cl_banker();
  virtual int init();
  virtual bool is_banker() { return true; }

  virtual void add_bank(int bank_nr, class cl_memory_chip *chip, t_addr chip_begin);

  virtual t_mem actual_bank();
  virtual bool activate(class cl_console_base *con);
  virtual bool switch_to(int bank_nr, class cl_console_base *con);
  
  virtual void print_info(chars pre, class cl_console_base *con);
};


/*
 * Address decoder which maps to individual bits
 */

class cl_bander: public cl_memory_chip
{
 private:
  class cl_memory_chip *chip;
  t_addr chip_begin;
  int bpc;
  int distance;
  t_mem mask;
  uchar shift;
 public:
  cl_bander(const char *id, class cl_memory_chip *chip, t_addr chip_begin, int bpc, int distance);
  virtual ~cl_bander(void) {}
  virtual class cl_bander *chip_init(void) { init(); return this; }
 protected:
  virtual t_mem get(t_addr addr);
  virtual void set(t_addr addr, t_mem val);

  virtual void print_info(chars pre, class cl_console_base *con);
};


/* List of address decoders */

class cl_decoder_list: public cl_sorted_list
{
protected:
  bool by_chip;
public:
  cl_decoder_list(t_index alimit, t_index adelta, bool bychip);

  virtual void *key_of(void *item);
  virtual int compare(const void *key1, const void *key2);
};


/*
 * Messages
 */

#include "eventcl.h"

class cl_event_address_space_added: public cl_event
{
public:
  class cl_address_space *as;
  cl_event_address_space_added(class cl_address_space *the_as):
    cl_event(ev_address_space_added)
  { as= the_as; }
};


/*
 * Errors in memory handling
 */

#include "errorcl.h"

class cl_error_mem: public cl_error
{
protected:
  class cl_memory *mem;
  t_addr addr;
public:
  cl_error_mem(class cl_memory *amem, t_addr aaddr);
};

class cl_error_mem_invalid_address: public cl_error_mem
{
public:
  cl_error_mem_invalid_address(class cl_memory *amem, t_addr aaddr);

  virtual void print(class cl_commander_base *c);
};

 class cl_error_mem_non_decoded: public cl_error_mem
{
public:
  cl_error_mem_non_decoded(class cl_memory *amem, t_addr aaddr);

  virtual void print(class cl_commander_base *c);
};

class cl_mem_error_registry: public cl_error_registry
{
public:
  cl_mem_error_registry(void);
};


#endif

/* End of memcl.h */
