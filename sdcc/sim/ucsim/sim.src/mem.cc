/*
 * Simulator of microcontrollers (mem.cc)
 *
 * Copyright (C) 1999,99 Drotos Daniel, Talker Bt.
 *
 * To contact author send email to drdani@mazsola.iit.uni-miskolc.hu
 *
 */

/*
   This file is part of microcontroller simulator: ucsim.

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
   02111-1307, USA.
*/
/*@1@*/

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdarg.h>
#include "i_string.h"

// prj
#include "utils.h"
#include "globals.h"

// sim
#include "simcl.h"

// cmd
#include "newcmdcl.h"
#include "cmdutil.h"

// local
#include "memcl.h"
#include "hwcl.h"


static class cl_mem_error_registry mem_error_registry;

/*
 *                                                3rd version of memory system
 */

cl_memory::cl_memory(const char *id, t_addr asize, int awidth):
  cl_base()
{
  if ((size= asize) > max_mem_size)
    size= max_mem_size;
  set_name(id);
  addr_format= data_format= 0;
  addr_format_width= data_format_width= 0;
  width= awidth;
  start_address= 0;
  uc= 0;
  hidden= false;
}

cl_memory::~cl_memory(void)
{
  if (addr_format)
    free(addr_format);
  if (data_format)
    free(data_format);
}

int
cl_memory::init(void)
{
  addr_format_width= 2 +
                     (size-1<=0xf ? 1 :
                      (size-1<=0xff ? 2 :
                       (size-1<=0xfff ? 3 :
                        (size-1<=0xffff ? 4 :
                         (size-1<=0xfffff ? 5 :
                          (size-1<=0xffffff ? 6 : 12))))));

  chars c= chars("", "0x%%0%d", addr_format_width - 2);
  if (sizeof(t_addr) > sizeof(long))
    c+= cchars("L");//strcat(addr_format, "L");
  else if (sizeof(t_addr) > sizeof(int))
    c+= cchars("l");//strcat(addr_format, "l");
  c+= cchars("x");//strcat(addr_format, "x");
  addr_format= strdup((char*)c);

  data_format_width= (width + 3) / 4;

  c= chars("", "%%0%d", data_format_width);
  if (sizeof(t_mem) > sizeof(long))
    c+= cchars("L");//strcat(data_format, "L");
  else if (sizeof(t_mem) > sizeof(int))
    c+= cchars("l");//strcat(data_format, "l");
  c+= cchars("x");//strcat(data_format, "x");
  data_format= strdup((char*)c);
  data_mask= 1;
  int w= width;
  for (--w; w; w--)
    {
      data_mask<<= 1;
      data_mask|= 1;
    }
  dump_finished= start_address;
  return(0);
}


bool
cl_memory::valid_address(t_addr addr)
{
  return(addr >= start_address &&
	 addr < start_address+size);
}

t_addr
cl_memory::inc_address(t_addr addr, int val)
{
  if (!start_address)
    return(((signed)addr+val)%size);
  addr-= start_address;
  addr+= val;
  addr%= size;
  addr+= start_address;
  return(addr);
}

t_addr
cl_memory::inc_address(t_addr addr)
{
  if (!start_address)
    return(((signed)addr+1)%size);
  addr-= start_address;
  addr++;
  addr%= size;
  addr+= start_address;
  return(addr);
}

t_addr
cl_memory::validate_address(t_addr addr)
{
  while (addr < start_address)
    addr+= size;
  if (addr > start_address+size)
    {
      addr-= start_address;
      addr%= size;
      addr+= start_address;
    }
  return(addr);
}


void
cl_memory::err_inv_addr(t_addr addr)
{
  if (!uc)
    return;
  class cl_error *e= new cl_error_mem_invalid_address(this, addr);
  uc->error(e);
}

void
cl_memory::err_non_decoded(t_addr addr)
{
  if (!uc)
    return;
  class cl_error *e= new cl_error_mem_non_decoded(this, addr);
  uc->error(e);
}

class cl_var *
cl_memory::var_for(t_addr addr, int bitnr_high, int bitnr_low, t_index &var_i)
{
  class cl_var *var = NULL;

  if (uc->vars->by_addr.search(this, addr, bitnr_high, bitnr_low, var_i) ||
      uc->vars->by_addr.search(this, addr, width - 1, 0, var_i) ||
      (bitnr_high >= 0 && uc->vars->by_addr.search(this, addr, -1, -1, var_i)))
    var = uc->vars->by_addr.at(var_i);

  return var;
}

t_addr
cl_memory::dump(class cl_console_base *con, int smart, t_addr start, t_addr stop, int bitnr_high, int bitnr_low, int bpl)
{
  if (!con)
    return dump_finished;

  if (!is_address_space())
    smart = 0;

  if (bpl < 0)
    bpl= 8;

  if (start < 0)
    start= dump_finished;

  t_addr lva= lowest_valid_address();
  t_addr hva= highest_valid_address();

  if (start < lva)
    start= lva;
  if (start > hva)
    return dump_finished;

  int lines= -1;
  if (stop < 0)
    {
      stop= hva;
      lines= 10;
    }
  if (stop > hva)
    stop= hva;
  if (stop < lva)
    return dump_finished;

  int i, step;
  if (stop >= start)
    {
      step= +1;
      stop++;
      if (start + bpl > stop)
        bpl= stop - start;
    }
  else
    {
      step= -1;
      stop--;
      if (start - bpl < stop)
        bpl= start - stop;
    }

  long label_width = -1;
  cl_option *o = application->options->get_option("label_width");
  if (o)
    o->get_value(&label_width);
  if (label_width < 0)
    label_width = uc->vars->get_max_name_len();

  t_index var_i;
  class cl_var *var = var_for(start, bitnr_high, bitnr_low, var_i);
  class cl_var *var_next = NULL;

  while ((step>0)?(start < stop):(start > stop))
    {
      int n;

      if (smart && this != uc->rom)
        con->dd_printf("%s[", get_name("?"));
      con->dd_printf(addr_format, start);
      if (smart && this != uc->rom)
        con->dd_printf("]");

      if (smart)
        {
          if (bitnr_high >= 0 && bitnr_high == bitnr_low)
            con->dd_printf(".%d   ", bitnr_high);
          else if (bitnr_low > 0 || (bitnr_high > 0 && bitnr_high < width - 1))
            con->dd_printf("[%d:%d]", bitnr_high, bitnr_low);
          else if (smart == 2)
            con->dd_printf("     ");

          if (var)
            {
              // If we asked for specific bits but got a label or all-bits register we
              // need to qualify the var.
              if (bitnr_high >= 0 &&
                  (var->bitnr_high < 0 ||
                   (var->bitnr_high == width - 1 && var->bitnr_low == 0)))
                {
                  if (bitnr_high >= 0 && bitnr_high == bitnr_low)
                    con->dd_printf(" %s.%d:   %*s", var->get_name(), bitnr_high, label_width - strlen(var->get_name()), "");
                  else if (bitnr_low > 0 || (bitnr_high > 0 && bitnr_high < width - 1))
                    con->dd_printf(" %s[%d:%d]:%*s", var->get_name(), bitnr_high, bitnr_low, label_width - strlen(var->get_name()), "");
                  else
                    con->dd_printf(" %s:%*s", var->get_name(), label_width - strlen(var->get_name()) + (smart == 2 ? 5 : 0), "");
                }
              else
                con->dd_printf(" %s:%*s", var->get_name(), label_width - strlen(var->get_name()) + (smart == 2 ? 5 : 0), "");

              // If the next var matches we do not need to output data now.
              if (++var_i < uc->vars->by_addr.count)
                {
                  if ((var_next = uc->vars->by_addr.at(var_i)) &&
                      var_next->mem == this &&
                      var_next->addr == var->addr &&
                      ((var_next->bitnr_high == bitnr_high && var_next->bitnr_low == bitnr_low) ||
                       (bitnr_high < 0 && var_next->bitnr_high == width - 1 && var_next->bitnr_low == 0)))
                    {
                      con->dd_printf("\n");
                      if (lines > 0)
                        lines--;
                      var = var_next;
                      var_next = NULL;
                      continue;
                    }
                }
            }
          else
            con->dd_printf(" %-*s %s", label_width, "", (smart == 2 ? "     " : ""));

          if (smart == 2 || (var && var->bitnr_high >= 0))
            {
              int b_high, b_low;

              if (var && var->bitnr_high >= 0)
                {
                  b_high = (bitnr_high < 0 || var->bitnr_high < bitnr_high ? var->bitnr_high : bitnr_high);
                  b_low = (bitnr_low < 0 || var->bitnr_low > bitnr_low ? var->bitnr_low : bitnr_low);
                }
              else if (bitnr_high >= 0)
                b_high = bitnr_high, b_low = bitnr_low;
              else
                b_high = width - 1, b_low = 0;

              con->dd_printf(" ");

              t_mem m= read(start);

              int i;
              con->dd_printf("0b");
              for (i= width - 1; i > b_high; i--)
                con->dd_printf("-");
              for (; i >= b_low; i--)
                con->dd_printf("%c", (m & (1U << i)) ? '1' : '0');
              for (; i >= 0; i--)
                con->dd_printf("-");

              int nbits = b_high - b_low + 1;

              m = (m >> b_low) & ((1U << nbits) - 1);

              con->dd_printf(" 0x%0*x", (nbits > 16 ? 8 : (nbits > 8 ? 4 : 2)), m);

              con->dd_printf(" '");
              for (int i= (nbits - 1) - ((nbits - 1) % 8); i >= 0; i -= 8)
                con->dd_printf("%c", (isprint(m >> i) ? m >> i : '.'));
              con->dd_printf("'");

              con->dd_printf(" %*u", (nbits > 16 ? 10 : (nbits > 8 ? 5 : 3)), m);

              if ((m & (1U << (nbits - 1))))
                con->dd_printf(" (%*d)", (nbits > 16 ? 10 : (nbits > 8 ? 5 : 3)), 0 - ((1U << nbits) - m));

              con->dd_printf("\n");
              if (lines > 0)
                lines--;

              // Only advance if there is no more to say about this location.
              var = NULL;
              while (var_i < uc->vars->by_addr.count)
                {
                  if ((var_next = uc->vars->by_addr.at(var_i)) &&
                      var_next->mem == this && var_next->addr == start &&
                      (bitnr_high < 0 ||
                       (var_next->bitnr_high <= bitnr_high && var_next->bitnr_low >= bitnr_low)))
                    {
                      var = var_next;
                      break;
                    }
                  var_i++;
                }

              if (!var)
                {
                  start += step;
                  dump_finished= start;
                  if (lines == 0)
                    break;
                  var = var_for(start, bitnr_high, bitnr_low, var_i);
                }
              continue;
            }
          else
            ; // Not bit-formatted so drop through to normal output.
        }

      con->dd_printf(" ");

      if (smart && step > 0 && this == uc->rom && uc->inst_at(start))
        {
          n= uc->inst_length(start);
          for (int i= 0; i < n; i++)
            {
              con->dd_printf(data_format, get(start+i*step));
              con->dd_printf(" ");
            }
          var= var_for(start+n*step, bitnr_high, bitnr_low, var_i);
        }
      else
        {
          for (n= 0;
               (n < bpl) &&
                 (start+n*step >= lva) &&
                 (start+n*step <= hva) &&
                 (start+n*step != stop);
               n++)
            {
              if (smart && n)
                {
                  var= var_for(start+n*step, bitnr_high, bitnr_low, var_i);
                  if (var)
                    break;
                  if (step > 0 && this == uc->rom && uc->inst_at(start+n*step))
                    break;
                }
              con->dd_printf(data_format, get(start+n*step));
              con->dd_printf(" ");
            }
        }
      for (i= n; i < bpl; i++)
        {
          for (int j= width/4 + ((width%4)?1:0) + 1; j; j--)
            con->dd_printf(" ");
        }
      if (!smart && n)
        con->dd_printf(" ");
      if (smart && step > 0 && this == uc->rom && uc->inst_at(start))
        {
          uc->disass(con, start, NULL);
        }
      else
        {
          for (i= 0; i < n &&
                 start+i*step >= lva &&
                 start+i*step <= hva &&
                 start+i*step != stop;
               i++)
            {
              long c= read(start+i*step);
              con->dd_printf("%c", isprint(255&c)?(255&c):'.');
              if (width > 8)
                con->dd_printf("%c", isprint(255&(c>>8))?(255&(c>>8)):'.');
              if (width > 16)
                con->dd_printf("%c", isprint(255&(c>>16))?(255&(c>>16)):'.');
              if (width > 24)
                con->dd_printf("%c", isprint(255&(c>>24))?(255&(c>>24)):'.');
            }
        }
      con->dd_printf("\n");

      start+= n*step;
      dump_finished= start;
      if (lines > 0 && --lines == 0)
        break;
    }

  return(dump_finished);
}

t_addr
cl_memory::dump_s(t_addr start, t_addr stop, int bpl, class cl_f *f)
{
  t_addr lva= lowest_valid_address();
  t_addr hva= highest_valid_address();

  if (stop < 0)
    stop= start + 10 * bpl - 1;

  t_addr a= start;
  t_mem d= read(a);
  char last= '\n';
  while ((a <= stop) &&
	 (d != 0) &&
	 (a <= hva))
    {
      char c= d;
      if (a >= lva)
	{
	  f->write(&c, 1);
	  last= c;
	}
      d= read(++a);
    }
  if (last != '\n')
    f->write_str("\n");
  return dump_finished= a;
}

t_addr
cl_memory::dump_b(t_addr start, t_addr stop, int bpl, class cl_f *f)
{
  t_addr lva= lowest_valid_address();
  t_addr hva= highest_valid_address();

  if (stop < 0)
    stop= start + 10 * bpl - 1;

  t_addr a= start;
  t_mem d= read(a);
  while ((a <= stop) &&
	 (a <= hva))
    {
      char c= d;
      if (a >= lva)
	{
	  f->write(&c, 1);
	}
      d= read(++a);
    }
  return dump_finished= a;
}

t_addr
cl_memory::dump_i(t_addr start, t_addr stop, int bpl, class cl_f *f)
{
  t_addr lva= lowest_valid_address();
  t_addr hva= highest_valid_address();
  unsigned int sum;
  t_addr start_line;
  
  if (stop < 0)
    stop= start + 10 * bpl - 1;

  if (start < lva)
    start= lva;
  if (stop > hva)
    stop= hva;

  if (start < lva)
    start= lva;
  if (start > hva)
    return dump_finished;
  if (stop > hva)
    stop= hva;
  if (stop < lva)
    return dump_finished;
  
  if (start > stop)
    return dump_finished= stop;
  if (bpl > 32)
    bpl= 32;
  t_addr a= start;
  sum= 0;
  start_line= a;
  while (a <= stop)
    {
      a++;
      if (((a % bpl) == 0) ||
	  (a > stop))
	{
	  // dump line
	  if ((a - start_line) > 0)
	    {
	      unsigned char c;	      
	      sum= 0;
	      c= a-start_line;
	      f->prntf(":%02X%04X00", c, start_line);
	      sum+= c;
	      c= int(start_line >> 8) & 0xff;
	      sum+= c;
	      c= start_line & 0xff;
	      sum+= c;
	      int i;
	      for (i= 0; i < a-start_line; i++)
		{
		  c= read(start_line + i);
		  f->prntf("%02X", c);
		  sum+= c;
		}
	      sum&= 0xff;
	      unsigned char chk= 0x100 - sum;
	      f->prntf("%02X\r\n", chk);
	    }
	  start_line= a;
	}
    }
  f->write_str(":00000001FF\r\n");
  return dump_finished= a;
}

bool
cl_memory::search_next(bool case_sensitive,
		       t_mem *array, int len, t_addr *addr)
{
  t_addr a;
  int i;
  bool found;

  if (addr == NULL)
    a= 0;
  else
    a= *addr;

  if (a+len > size)
    return(false);

  found= false;
  while (!found &&
	 a+len <= size)
    {
      bool match= true;
      for (i= 0; i < len && match; i++)
	{
	  t_mem d1, d2;
	  d1= get(a+i);
	  d2= array[i];
	  if (!case_sensitive)
	    {
	      if (/*d1 < 128*/isalpha(d1))
		d1= toupper(d1);
	      if (/*d2 < 128*/isalpha(d2))
		d2= toupper(d2);
	    }
	  match= d1 == d2;
	}
      found= match;
      if (!found)
	a++;
    }

  if (addr)
    *addr= a;
  return(found);
}

void
cl_memory::print_info(chars pre, class cl_console_base *con)
{
  char *n= (char*)(get_name());
  if (!hidden)
    {
      con->dd_printf("%s0x%06x-0x%06x %8d %s (%d,%s,%s)\n", (char*)pre,
		     AU(get_start_address()),
		     AU(highest_valid_address()),
		     AU(get_size()),
		     n,
		     width, data_format, addr_format);
    }
}


/*
 *                                                             Memory operators
 */

cl_memory_operator::cl_memory_operator(class cl_memory_cell *acell/*,
								    t_addr addr*/):
  cl_base()
{
  cell= acell;
  if (cell)
    {
      //data= cell->data;
      mask= cell->mask;
    }
  else
    {
      //data= 0;
      mask= ~0;
    }
  next_operator= 0;
  //address= addr;
}
/*
cl_memory_operator::cl_memory_operator(class cl_memory_cell *acell,
				       t_addr addr,
				       t_mem *data_place, t_mem the_mask):
  cl_base()
{
  cell= acell;
  //data= data_place;
  mask= the_mask;
  next_operator= 0;
  address= addr;
}
*/
/*
void
cl_memory_operator::set_data(t_mem *data_place, t_mem the_mask)
{
  data= data_place;
  mask= the_mask;
}
*/

t_mem
cl_memory_operator::read(void)
{
  if (next_operator)
    return(next_operator->read());
  else if (cell)
    return(/* *data*/cell->get());
  return 0;
}

t_mem
cl_memory_operator::write(t_mem val)
{
  if (next_operator)
    return(next_operator->write(val));
  return val;
}


/* Memory operator for bank switcher */

cl_bank_switcher_operator::cl_bank_switcher_operator(class cl_memory_cell *acell,
						     /*t_addr addr,*/
						     class cl_banker *the_banker):
  cl_memory_operator(acell/*, addr*/)
{
  banker= the_banker;
  set_name("bank_switcher");
}

t_mem
cl_bank_switcher_operator::write(t_mem val)
{
  if (next_operator)
    next_operator->write(val);
  if (cell) /* *data=*/ cell->set(val & mask);
  banker->activate(NULL);
  if (cell)
    return cell->get();
  return /* *data*/ 0;  
}


/* Memory operator for hw callbacks */

cl_hw_operator::cl_hw_operator(class cl_memory_cell *acell/*, t_addr addr*/,
			       //t_mem *data_place, t_mem the_mask,
			       class cl_hw *ahw):
  cl_memory_operator(acell/*, addr*//*, data_place, the_mask*/)
{
  hw= ahw;
  set_name(chars("hw:")+hw->get_name());
}


t_mem
cl_hw_operator::read(void)
{
  t_mem d1= 0, d2= 0;

  if (hw)
    d1= hw->read(cell);

  if (next_operator)
    d2= next_operator->read();

  return(hw?d1:d2);
}

t_mem
cl_hw_operator::read(enum hw_cath skip)
{
  t_mem d1= 0/* *data*/, d2= d1;
  bool use= false;

  if (hw &&
      hw->cathegory != skip)
    use= true, d1= hw->read(cell);

  if (next_operator)
    d2= next_operator->read();
  else if (cell)
    d2= cell->get();
  else
    return use= true;

  return(use?d1:d2);
}

t_mem
cl_hw_operator::write(t_mem val)
{
  if (hw)
    hw->write(cell, &val);
  if (next_operator)
    val= next_operator->write(val);
  //if (cell) return(/* *data=*//*cell->set(val & mask)*/val);
  return val;
}


/* Write event break on cell */

cl_write_operator::cl_write_operator(class cl_memory_cell *acell/*, t_addr addr*/,
				     //t_mem *data_place, t_mem the_mask,
				     class cl_uc *auc, class cl_brk *the_bp):
  cl_event_break_operator(acell/*, addr*//*, data_place, the_mask*/, auc, the_bp)
{
  uc= auc;
  bp= the_bp;
  set_name("write_event");
}

t_mem
cl_write_operator::write(t_mem val)
{
  //printf("write event at 0x%x bp=%p\n",address,bp);
  if (bp->do_hit())
    uc->events->add(bp);
  if (next_operator)
    return(next_operator->write(val));
  else if (cell)
    return(/* *data=*/cell->set(val & mask));
  return val;
}


/* Read event break on cell */

cl_read_operator::cl_read_operator(class cl_memory_cell *acell/*, t_addr addr*/,
				   //t_mem *data_place, t_mem the_mask,
				   class cl_uc *auc, class cl_brk *the_bp):
  cl_event_break_operator(acell/*, addr*//*, data_place, the_mask*/, auc, the_bp)
{
  uc= auc;
  bp= the_bp;
  set_name("read_event");
}

t_mem
cl_read_operator::read(void)
{
  //printf("read event at 0x%x bp=%p\n",address,bp);
  if (bp->do_hit())
    uc->events->add(bp);
  if (next_operator)
    return(next_operator->read());
  else if (cell)
    return(/* *data*/cell->get());
  return 0;
}



/*
 *                                                                  Memory cell
 */

cl_memory_cell::cl_memory_cell(class cl_memory_chip *chip, t_addr addr)//: cl_base()
{
  this->chip= chip;
  this->chipaddr= addr;

  data= chip->get_slot(addr);
  if (!data)
    data= &def_data;

  def_data= 0;
  operators= NULL;
  //bank= 0;
  //banked_data_ptrs= 0;
#ifdef STATISTIC
  nuof_writes= nuof_reads= 0;
#endif
  mask= 1;
  int w= chip->width;
  for (--w; w; w--)
    {
      mask<<= 1;
      mask|= 1;
    }
}

cl_memory_cell::~cl_memory_cell(void)
{
}

int
cl_memory_cell::init(void)
{
  data= &def_data;
  return(0);
}


uchar
cl_memory_cell::get_flags(void)
{
  return chip->get_flags(chipaddr);
}

bool
cl_memory_cell::get_flag(enum cell_flag flag)
{
  return chip->get_flag(chipaddr, flag);
}

void
cl_memory_cell::set_flag(enum cell_flag flag, bool val)
{
  chip->set_flag(chipaddr, flag, val);
}


t_mem
cl_memory_cell::read(void)
{
#ifdef STATISTIC
  nuof_reads++;
#endif
  if (operators)
    return(operators->read());
  else
    return get();
}

t_mem
cl_memory_cell::read(enum hw_cath skip)
{
#ifdef STATISTIC
  nuof_reads++;
#endif
  if (operators)
    return(operators->read(skip));
  else
    return get();
}

t_mem
cl_memory_cell::get(void)
{
  return chip->get(chipaddr);
}

t_mem
cl_memory_cell::write(t_mem val)
{
#ifdef STATISTIC
  nuof_writes++;
#endif
  if (chip->width != 1)
    val &= mask;

  if (operators)
    val = operators->write(val) & mask;

  return set(val);
}

t_mem
cl_memory_cell::set(t_mem val)
{
  chip->set(chipaddr, val);
  return get();
}

t_mem
cl_memory_cell::download(t_mem val)
{
  if (chip->width != 1)
    val &= mask;

  chip->set(chipaddr, val);
  return get();
}

t_mem
cl_memory_cell::add(long what)
{
  return set(get() + what);
}

t_mem
cl_memory_cell::wadd(long what)
{
  return write(get() + what);
}

void
cl_memory_cell::set_bit1(t_mem bits)
{
  set(get() | bits);
}

void
cl_memory_cell::write_bit1(t_mem bits)
{
  write(get() | bits);
}

void
cl_memory_cell::set_bit0(t_mem bits)
{
  set(get() & (~bits));
}

void
cl_memory_cell::write_bit0(t_mem bits)
{
  write(get() & (~bits));
}

void
cl_memory_cell::toggle_bits(t_mem bits)
{
  set(get() ^ bits);
}

void
cl_memory_cell::wtoggle_bits(t_mem bits)
{
  write(get() ^ bits);
}


void
cl_memory_cell::append_operator(class cl_memory_operator *op)
{
  if (!operators)
    operators= op;
  else
    {
      class cl_memory_operator *o= operators, *n;
      n= o->get_next();
      while (n)
	{
	  o= n;
	  n= o->get_next();
	}
      o->set_next(op);
    }
}

void
cl_memory_cell::prepend_operator(class cl_memory_operator *op)
{
  if (op)
    {
      op->set_next(operators);
      operators= op;
    }
}

void
cl_memory_cell::del_operator(class cl_brk *brk)
{
  if (!operators)
    return;
  class cl_memory_operator *op= operators;
  if (operators->match(brk))
    {
      operators= op->get_next();
      delete op;
    }
  else
    {
      while (op->get_next() &&
	     !op->get_next()->match(brk))
	op= op->get_next();
      if (op->get_next())
	{
	  class cl_memory_operator *m= op->get_next();
	  op->set_next(m->get_next());;
	  delete m;
	}
    }
}

void 	 
cl_memory_cell::del_operator(class cl_hw *hw)
{
  if (!operators)
    return;
  class cl_memory_operator *op= operators;
  if (operators->match(hw))
    {
      operators= op->get_next();
      delete op;
    }
  else
    {
      while (op->get_next() &&
	     !op->get_next()->match(hw))
	op= op->get_next();
      if (op->get_next())
	{
	  class cl_memory_operator *m= op->get_next();
	  op->set_next(m->get_next());
	  delete m;
	}
    }
}

class cl_banker *
cl_memory_cell::get_banker(void)
{
  class cl_memory_operator *op= operators;
  class cl_banker *b= NULL;

  while (op)
    {
      b= op->get_banker();
      if (b)
	return b;
      op= op->get_next();
    }
  return NULL;
}

class cl_memory_cell *
cl_memory_cell::add_hw(class cl_hw *hw/*, t_addr addr*/)
{
  class cl_hw_operator *o= new cl_hw_operator(this/*, addr*//*, data, mask*/, hw);
  append_operator(o);
  return(this);
}

void 	 
cl_memory_cell::remove_hw(class cl_hw *hw) 	 
{ 	 
  del_operator(hw); 	 
}

/*class cl_hw *
cl_memory_cell::get_hw(int ith)
{
  return(0);
}*/

class cl_event_handler *
cl_memory_cell::get_event_handler(void)
{
  return(0);
}

void
cl_memory_cell::print_info(chars pre, class cl_console_base *con)
{
  uchar flags = chip->get_flags(chipaddr);

  con->dd_printf("%sFlags:", (char*)pre);
  if (flags & CELL_VAR)
    con->dd_printf(" VAR");
  if (flags & CELL_INST)
    con->dd_printf(" INST");
  if (flags & CELL_FETCH_BRK)
    con->dd_printf(" FBRK");
  if (flags & CELL_READ_ONLY)
    con->dd_printf(" RO");
  if (!(flags & CELL_DECODED))
    con->dd_printf(" NDC");
  con->dd_printf("\n");
  print_operators(pre, con);
}

void
cl_memory_cell::print_operators(chars pre, class cl_console_base *con)
{
  class cl_memory_operator *o= operators;
  if (!operators)
    return;
  int i= 0;
  while (o)
    {
      printf("%s %02d. %s\n", (char*)pre, i, o->get_name("?"));
      i++;
      o= o->get_next();
    }
}


/*
 * Dummy cell for non-existent addresses
 */

t_mem
cl_dummy_cell::write(t_mem val)
{
#ifdef STATISTIC
  nuof_writes++;
#endif
  *data= rand() & mask;
  return(*data);
}

t_mem
cl_dummy_cell::set(t_mem val)
{
  *data= rand() & mask;
  return(*data);
}


/*
 *                                                                Address space
 */

// Dummies are used so we can avoid a lot of null checks and simplify the code.
class cl_memory_chip *cl_address_space::dummy_chip = (new cl_memory_chip("dummy", 1, 8))->chip_init();
class cl_dummy_cell *cl_address_space::dummy = (new cl_dummy_cell(cl_address_space::dummy_chip, 0))->cell_init();

cl_address_space::cl_address_space(const char *id, t_addr astart, t_addr asize, int awidth):
  cl_memory(id, asize, awidth)
{
  start_address= astart;
  decoders= new cl_decoder_list(2, 2, false);
}

cl_address_space::~cl_address_space(void)
{
  delete decoders;
}

void
cl_address_space::decode(t_addr as_begin, class cl_memory_chip *chip, t_addr chip_begin, t_addr size)
{
  class cl_address_decoder *ad= new cl_address_decoder(this, chip, as_begin, as_begin + size - 1, chip_begin);
  ad->init();
  decoders->add(ad);
  ad->activate(0);
}

void
cl_address_space::decode(t_addr as_begin, class cl_memory_chip *chip)
{
  decode(as_begin, chip, 0, chip->get_size());
}

t_mem
cl_address_space::read(t_addr addr)
{
  class cl_address_decoder *ad = get_decoder_of(addr);
  if (ad)
    return ad->chip->read(ad->as_to_chip(addr));
  err_inv_addr(addr);
  return 0;
}

t_mem
cl_address_space::read(t_addr addr, enum hw_cath skip)
{
  class cl_address_decoder *ad = get_decoder_of(addr);
  if (ad)
    return ad->chip->read(ad->as_to_chip(addr), skip);
  err_inv_addr(addr);
  return 0;
}

t_mem
cl_address_space::write(t_addr addr, t_mem val)
{
  class cl_address_decoder *ad = get_decoder_of(addr);
  if (ad)
    return ad->chip->write(ad->as_to_chip(addr), val);
  err_inv_addr(addr);
  return 0;
}

t_mem
cl_address_space::get(t_addr addr)
{
  class cl_address_decoder *ad = get_decoder_of(addr);
  if (ad)
    return ad->chip->get(ad->as_to_chip(addr));
  err_inv_addr(addr);
  return 0;
}

void
cl_address_space::set(t_addr addr, t_mem val)
{
  class cl_address_decoder *ad = get_decoder_of(addr);
  if (ad)
    ad->chip->set(ad->as_to_chip(addr), val);
  else
    err_inv_addr(addr);
}

void
cl_address_space::download(t_addr addr, t_mem val)
{
  class cl_address_decoder *ad = get_decoder_of(addr);
  if (ad)
    ad->chip->download(ad->as_to_chip(addr), val);
  else
    err_inv_addr(addr);
}

t_mem
cl_address_space::wadd(t_addr addr, long what)
{
  class cl_address_decoder *ad = get_decoder_of(addr);
  if (ad)
    return ad->chip->wadd(ad->as_to_chip(addr), what);
  err_inv_addr(addr);
  return 0;
}

/* Set or clear bits, without callbacks */

void
cl_address_space::set_bit1(t_addr addr, t_mem bits)
{
  class cl_address_decoder *ad = get_decoder_of(addr);
  if (ad)
    ad->chip->set_bit1(ad->as_to_chip(addr), bits);
  else
    err_inv_addr(addr);
}

void
cl_address_space::set_bit0(t_addr addr, t_mem bits)
{
  class cl_address_decoder *ad = get_decoder_of(addr);
  if (ad)
    ad->chip->set_bit0(ad->as_to_chip(addr), bits);
  else
    err_inv_addr(addr);
}


class cl_memory_cell *
cl_address_space::get_cell(t_addr addr)
{
  class cl_address_decoder *ad = get_decoder_of(addr);
  if (ad)
    {
      class cl_memory_cell *cell = ad->chip->get_cell(ad->as_to_chip(addr));
      if (cell)
        return cell;
    }
  err_inv_addr(addr);
  return(dummy);
}


int
cl_address_space::get_cell_flag(t_addr addr)
{
  class cl_address_decoder *ad = get_decoder_of(addr);
  if (ad)
    return ad->chip->get_flags(ad->as_to_chip(addr));
  return 0;
}

bool
cl_address_space::get_cell_flag(t_addr addr, enum cell_flag flag)
{
  class cl_address_decoder *ad = get_decoder_of(addr);
  if (ad)
    return ad->chip->get_flag(ad->as_to_chip(addr), flag);
  return 0;
}

void
cl_address_space::set_cell_flag(t_addr addr, bool set_to, enum cell_flag flag)
{
  class cl_address_decoder *ad = get_decoder_of(addr);
  if (ad)
    ad->chip->set_flag(ad->as_to_chip(addr), flag, set_to);
}

void
cl_address_space::set_cell_flag(t_addr start_addr, t_addr end_addr, bool set_to, enum cell_flag flag)
{
  t_addr a;

  for (a= start_addr; a <= end_addr; a++)
    set_cell_flag(a, set_to, flag);
}

class cl_memory_cell *
cl_address_space::search_cell(enum cell_flag flag, bool value, t_addr *addr)
{
  int i;

  for (i= 0; i < size; i++)
    {
      bool f= get_cell_flag(i, flag);
      if ((f && value) ||
	  (!f && !value))
	{
	  if (addr)
	    *addr= i;
	  return get_cell(i);
	}
    }
  return NULL;
}

bool
cl_address_space::is_owned(class cl_memory_cell *cell, t_addr *addr)
{
  for (int i= 0; i < decoders->count; i++)
    {
      class cl_address_decoder *ad= (class cl_address_decoder *)(decoders->at(i));
      if (ad->chip->is_owned(cell, addr, ad->chip_begin, ad->as_to_chip(ad->as_end)))
        {
          *addr += ad->as_begin - ad->chip_begin;
	  return true;
	}
    }
  return false;
}

class cl_address_decoder *
cl_address_space::get_decoder_of(t_addr addr)
{
  for (int i= 0; i < decoders->count; i++)
    {
      class cl_address_decoder *ad= (class cl_address_decoder *)(decoders->at(i));
      if (ad->covers(addr, addr))
	return ad;
    }
  return NULL;
}
  
void
cl_address_space::undecode_area(class cl_address_decoder *skip,
				t_addr begin, t_addr end,
				class cl_console_base *con)
{
#define D if (con) con->debug
  //#define D printf
  D("Undecoding area 0x%lx-0x%lx of %s (skip=%s)\n", begin, end, get_name(), skip?(skip->get_name()):"-");
  int i;
  for (i= 0; i < decoders->count; i++)
    {
      class cl_address_decoder *d=
	dynamic_cast<class cl_address_decoder *>(decoders->object_at(i));
      if (!d ||
	  d == skip)
	continue;
      D("  Checking decoder 0x%lx-0x%lx -> %s[0x%lx]\n",
	d->as_begin, d->as_end, (d->chip)?(d->chip->get_name()):"(none)", d->chip_begin);
      if (d->fully_covered_by(begin, end))
	{
	  // decoder can be removed
	  D("    Can be removed\n");
	  decoders->disconn(d);
	  i--;
	  delete d;
	  if (decoders->count == 0)
	    break;
	}
      else if (d->covers(begin, end))
	{
	  // decoder must be split
	  D("    Must be split\n");
	  class cl_address_decoder *nd= d->split(begin, end);
	  D("    After split:\n");
	  D("      0x%lx-0x%lx -> %s[0x%lx]\n",
	    d->as_begin, d->as_end, (d->chip)?(d->chip->get_name()):"(none)", d->chip_begin);
	  if (nd)
	    {
	      decoders->add(nd);
	      D("      0x%lx-0x%lx -> %s[0x%lx]\n",
		nd->as_begin, nd->as_end, (nd->chip)?(nd->chip->get_name()):"none", nd->chip_begin);
	      nd->activate(con);
	    }
	}
      else if (d->is_in(begin, end))
	{
	  // decoder sould shrink
	  D("    Sould shrink\n");
	  if (d->shrink_out_of(begin, end))
	    {
	      D("    Can be removed after shrink\n");
	      decoders->disconn(d);
	      i--;
	      delete d;
	      if (decoders->count == 0)
		break;
	    }
	  else
	    {
	      D("    Shrinked to 0x%lx-0x%lx -> %s[0x%lx]\n",
		d->as_begin, d->as_end, (d->chip)?(d->chip->get_name()):"(none)", d->chip_begin);
	    }
	}
    }
#undef D
}


class cl_memory_cell *
cl_address_space::register_hw(t_addr addr, class cl_hw *hw,
			      bool announce)
{
  class cl_memory_cell *cell= get_cell(addr);
  cell->add_hw(hw);
  if (announce)
    ;//uc->sim->/*app->*/mem_cell_changed(this, addr);//FIXME
  return(cell);
}

/*
void 	 
cl_address_space::unregister_hw(class cl_hw *hw)
{
  t_addr idx;

  for (idx= 0; idx < size; idx++)
    {
      class cl_memory_cell *cell= &cella[idx];
      cell->remove_hw(hw);
    }
}
*/

void
cl_address_space::set_brk(t_addr addr, class cl_brk *brk)
{
  class cl_memory_cell *cell= get_cell(addr);
  class cl_memory_operator *op;

  switch (brk->get_event())
    {
    case brkWRITE: case brkWXRAM: case brkWIRAM: case brkWSFR:
      //e= 'W';
      op= new cl_write_operator(cell, uc, brk);
      break;
    case brkREAD: case brkRXRAM: case brkRCODE: case brkRIRAM: case brkRSFR:
      //e= 'R';
      op= new cl_read_operator(cell, uc, brk);
      break;
    case brkNONE:
      set_cell_flag(addr, true, CELL_FETCH_BRK);
      return;
      break;
    default:
      //e= '.';
      op= 0;
      break;
    }
  if (op)
    cell->append_operator(op);
}

void
cl_address_space::del_brk(t_addr addr, class cl_brk *brk)
{
  class cl_memory_cell *cell= get_cell(addr);

  switch (brk->get_event())
    {
    case brkWRITE: case brkWXRAM: case brkWIRAM: case brkWSFR:
    case brkREAD: case brkRXRAM: case brkRCODE: case brkRIRAM: case brkRSFR:
      cell->del_operator(brk);
      break;
    case brkNONE:
      set_cell_flag(addr, false, CELL_FETCH_BRK);
      return;
      break;
    default:
      break;
    }
}

void
cl_address_space::print_info(chars pre, class cl_console_base *con)
{
  char *n= (char*)(get_name());
  if (!hidden)
    {
      con->dd_printf("%s0x%06x-0x%06x %8d %s (%d,%s,%s)\n", (char*)pre,
		     AU(get_start_address()),
		     AU(highest_valid_address()),
		     AU(get_size()),
		     n,
		     width, data_format, addr_format);
    }
}


/*
 * List of address spaces
 */

cl_memory_list::cl_memory_list(class cl_uc *the_uc, const char *name):
  cl_list(2, 2, name)
{
  uc= the_uc;
}

t_index
cl_memory_list::add(class cl_memory *mem)
{
  t_index ret= cl_list::add(mem);
  mem->set_uc(uc);
  if (uc && mem->is_address_space())
    {
      class cl_event_address_space_added e((cl_address_space *)mem);
      uc->handle_event(e);
    }
  return(ret);
}


/*
 *                                                                  Memory chip
 */

cl_memory_chip::cl_memory_chip(const char *id,
			       int asize,
			       int awidth,
			       int initial):
  cl_memory(id, asize, awidth)
{
  array= (t_mem *)malloc(size * sizeof(array[0]));
  flags= (uchar *)calloc(size, sizeof(flags[0]));
  init_value= initial;
  array_is_mine= true;
  cella= NULL;
}

cl_memory_chip::cl_memory_chip(const char *id,
			       int asize,
			       int awidth,
			       t_mem *aarray):
  cl_memory(id, asize, awidth)
{
  array= aarray;
  flags= (uchar *)calloc(size, sizeof(flags[0]));
  init_value= 0;
  array_is_mine= false;
  cella= NULL;
}

cl_memory_chip::~cl_memory_chip(void)
{
  if (cella)
    {
      for (int i= 0; i < size; i++)
         if (cella[i])
          cella[i]->~cl_memory_cell();
      free(cella);
    }

  if (array &&
      array_is_mine)
    free(array);

  free(flags);
}

int
cl_memory_chip::init(void)
{
  cl_memory::init();

  if (array_is_mine)
    {
      for (int i= 0; i < size; i++)
	set(i, (init_value<0)?rand():(init_value));
    }

  return 0;
}


class cl_memory_cell *
cl_memory_chip::get_cell(t_addr addr)
{
  if (size <= addr)
    return(0);

  if (!cella)
    cella= (cl_memory_cell **)calloc(size, sizeof(cl_memory_cell *));

  if (!cella[addr])
    {
      cella[addr] = new cl_memory_cell(this, addr);
      cella[addr]->init();
    }

  return cella[addr];
}

t_mem *
cl_memory_chip::get_slot(t_addr addr)
{
  if (!array ||
      size <= addr)
    return(0);
  return(&array[addr]);
}

uchar
cl_memory_chip::get_flags(t_addr addr)
{
  if (array && addr < size)
    return flags[addr];

  return 0;
}

bool
cl_memory_chip::get_flag(t_addr addr, enum cell_flag flag)
{
  if (array && addr < size)
    return flags[addr] & flag;

  return 0;
}

void
cl_memory_chip::set_flag(t_addr addr, enum cell_flag flag, bool val)
{
  if (array && addr < size)
    {
      if (val)
        flags[addr] |= flag;
      else
        flags[addr] &= ~flag;
    }
}

t_mem
cl_memory_chip::read(t_addr addr)
{
  if (array && addr < size)
    {
      if (cella && cella[addr])
        return cella[addr]->read();
      return get(addr);
    }
  return 0;
}

t_mem
cl_memory_chip::read(t_addr addr, enum hw_cath skip)
{
  if (array && addr < size)
    {
      if (cella && cella[addr])
        return cella[addr]->read(skip);
      return get(addr);
    }
  return 0;
}

t_mem
cl_memory_chip::write(t_addr addr, t_mem val)
{
  if (array && addr < size)
    {
      if (cella && cella[addr])
        return cella[addr]->write(val);
      set(addr, val);
      return get(addr);
    }
  return 0;
}

t_mem
cl_memory_chip::get(t_addr addr)
{
  if (!array ||
      size <= addr)
    return(0);
  return(array[addr]);
}

void
cl_memory_chip::set(t_addr addr, t_mem val)
{
  if (!array ||
      size <= addr)
    return;

  if (!(flags[addr] & CELL_READ_ONLY))
    array[addr]= val & data_mask;
}

void
cl_memory_chip::download(t_addr addr, t_mem val)
{
  if (!array ||
      size <= addr)
    return;
  array[addr]= val & data_mask;
}

void
cl_memory_chip::set_bit1(t_addr addr, t_mem bits)
{
  if (!array ||
      size <= addr)
    return;
  array[addr]|= (bits & data_mask);
}

void
cl_memory_chip::set_bit0(t_addr addr, t_mem bits)
{
  if (!array ||
      size <= addr)
    return;
  array[addr]&= ((~bits) & data_mask);
}

t_mem
cl_memory_chip::wadd(t_addr addr, long what)
{
  if (!array ||
      size <= addr)
    return 0;
  return array[addr]= (array[addr] + what) & data_mask;
}

bool
cl_memory_chip::is_owned(class cl_memory_cell *cell, t_addr *addr, int start, int end)
{
  for (int i = start; i < end; i++)
    {
      if (cella && cella[i] == cell)
        {
          if (addr)
            *addr = i;
          return true;
	}
    }

  return false;
}

void
cl_memory_chip::print_info(chars pre, class cl_console_base *con)
{
  char *n= (char*)(get_name());
  if (!hidden)
    {
      //con->dd_printf(pre0);
      con->dd_printf("%s0x%06x-0x%06x %8d %s (%d,%s,%s)\n", (char*)pre,
		     AU(get_start_address()),
		     AU(highest_valid_address()),
		     AU(get_size()),
		     n,
		     width, data_format, addr_format);
    }
}


/*
 *                                                              Address decoder
 */

cl_address_decoder::cl_address_decoder(class cl_memory *as,
				       class cl_memory *chip,
				       t_addr asb, t_addr ase, t_addr cb)
{
  if (as && (as->is_address_space()))
    address_space= (class cl_address_space *)as;
  else
    address_space= 0;

  if (chip && (chip->is_chip()))
    this->chip= (class cl_memory_chip *)chip;
  else
    this->chip= 0;

  as_begin= asb;
  as_end= ase;
  chip_begin= cb;
  activated= false;
}

cl_address_decoder::~cl_address_decoder(void)
{
}

int
cl_address_decoder::init(void)
{
  return(0);
}


bool
cl_address_decoder::activate(class cl_console_base *con)
{
#define D if (con) con->debug
  //#define D printf
  D("Activation of an address decoder %s (%s[%06lx-%06lx]\n", get_name(""), address_space->get_name(), as_begin, as_end);
  if (activated)
    {
      D("Already activated\n");
      return(false);
    }
  if (!address_space ||
      !address_space->is_address_space())
    {
      D("No or non address space\n");
      return(false);
    }
  if (!chip ||
      !chip->is_chip())
    {
      D("No or non memory chip\n");
      return(false);
    }
  if (as_begin > as_end)
    {
      D("Wrong address area specification\n");
      return(false);
    }
  if (chip_begin >= chip->get_size())
    {
      D("Wrong chip area specification\n");
      return(false);
    }
  if (as_begin < address_space->start_address ||
      as_end >= address_space->start_address + address_space->get_size())
    {
      D("Specified area is out of address space\n");
      return(false);
    }
  if (as_end-as_begin > chip->get_size()-chip_begin)
    {
      D("Specified area is out of chip size\n");
      return(false);
    }

  address_space->undecode_area(this, as_begin, as_end, con);

  D("Decoder maps %s[%06lx-%06lx] -> %s[%06lx]...\n",address_space->get_name(),as_begin,as_end,chip->get_name(),chip_begin);
  activated= true;

  for (t_addr addr = chip_begin; addr < as_to_chip(as_end); addr++)
     chip->set_flag(addr, CELL_DECODED, true);

#undef D
  return(activated);
}

/* Check if this DEC is fully within the specified area

   as_begin....................as_end
 ^                                    ^
 begin                              end

*/

bool
cl_address_decoder::fully_covered_by(t_addr begin, t_addr end)
{
  if (begin <= as_begin &&
      end >= as_end)
    return(true);
  return(false);
}

/* Check if some part of this DEC is in the specified area:

   as_begin......................as_end
                         ^               ^
                         begin         end

   as_begin......................as_end
^               ^
begin           end

*/

bool
cl_address_decoder::is_in(t_addr begin, t_addr end)
{
  if (begin >= as_begin &&
      begin <= as_end)
    return(true);
  if (end >= as_begin &&
      end <= as_end)
    return(true);
  return(false);
}

/* Check if this DEC covers the specified area:

   as_begin....................as_end
             ^             ^
             begin       end

*/

bool
cl_address_decoder::covers(t_addr begin, t_addr end)
{
  if (begin >= as_begin &&
      end <= as_end)
    return(true);
  return(false);
}


/* Returns TRUE if shrunken decoder is unnecessary */

bool
cl_address_decoder::shrink_out_of(t_addr begin, t_addr end)
{
  if (!address_space)
    return(true);
  if (begin > as_begin)
    as_end= begin-1;
  if (as_end > end)
    {
      chip_begin+= (end-as_begin+1);
      as_begin= end+1;
    }
  if (as_end < as_begin)
    return(true);
  return(false);
}

class cl_address_decoder *
cl_address_decoder::split(t_addr begin, t_addr end)
{
  class cl_address_decoder *nd= 0;
  if (begin > as_begin)
    {
      if (as_end > end)
	nd= new cl_address_decoder(address_space, chip,
				   end+1, as_end, chip_begin+(end-as_begin)+1);
      shrink_out_of(begin, as_end);
    }
  else if (end < as_end)
    {
      if (as_begin < begin)
	nd= new cl_address_decoder(address_space, chip,
				   as_begin, begin-1, chip_begin);
      shrink_out_of(as_begin, end);
    }
  if (nd)
    nd->init();
  return(nd);
}

void
cl_address_decoder::print_info(chars pre, class cl_console_base *con)
{
  if (address_space &&
      address_space->hidden)
    return;
  if (chip &&
      chip->hidden)
    return;
  con->dd_printf(pre);
  if (address_space)
    {
      con->dd_printf("%s ", address_space->get_name("unknown"));
      con->dd_printf(address_space->addr_format, as_begin);
      con->dd_printf(" ");
      con->dd_printf(address_space->addr_format, as_end);
    }
  else
    con->dd_printf("x");
  con->dd_printf(" -> ");
  if (chip)
    {
      con->dd_printf("%s ", chip->get_name("unknown"));
      con->dd_printf(chip->addr_format, chip_begin);
    }
  else
    con->dd_printf("x");
  con->dd_printf(" %s\n", (activated)?"activated":"inactive");
}


/*
 * Bank switcher
 */

cl_banker::cl_banker(class cl_address_space *the_banker_as,
		     t_addr the_banker_addr,
		     t_mem the_banker_mask,
		     class cl_address_space *the_as,
		     t_addr the_asb,
		     t_addr the_ase):
  cl_address_decoder(the_as, NULL, the_asb, the_ase, (t_addr)-1)
{
  banker_as= the_banker_as;
  banker_addr= the_banker_addr;
  banker_mask= the_banker_mask;
  nuof_banks= 0;
  banks= 0;
  bank= -1;
}

int
cl_banker::init()
{
  int m= banker_mask;
  int b;

  shift_by= 0;
  if (m == 0)
    nuof_banks= 0;
  else
    {
      while ((m&1) == 0)
	m>>= 1, shift_by++;
      b= 1;
      m>>= 1;
      while ((m&1) != 0)
	{
	  m>>= 1;
	  b++;
	}
      nuof_banks= 1 << b;
    }

  if (nuof_banks > 0)
    banks = new struct bank_def[nuof_banks];

  class cl_memory_cell *cell= banker_as->get_cell(banker_addr);
  if (cell)
    cell->prepend_operator(new cl_bank_switcher_operator(cell, this));

  return 0;
}

cl_banker::~cl_banker()
{
  int i;
  if (banks)
    {
      for (i= 0; i < nuof_banks; i++)
	{
	  if (banks[i].chip)
	    delete banks[i].chip;
	}
      delete banks;
    }
}

void
cl_banker::add_bank(int bank_nr, class cl_memory_chip *chip, t_addr chip_begin)
{
  if (!chip)
    return;
  if (!address_space)
    return;
  if (!chip->is_chip())
    return;

  if (bank_nr >= nuof_banks)
    return;
  
  banks[bank_nr].chip = chip;
  banks[bank_nr].chip_begin = chip_begin;

  activate(0);
}

t_mem
cl_banker::actual_bank()
{
  t_mem v= banker_as->read(banker_addr) & banker_mask;

  return (v >> shift_by);
}

bool
cl_banker::activate(class cl_console_base *con)
{
  switch_to(actual_bank(), con);
  return true;
}

bool
cl_banker::switch_to(int b, class cl_console_base *con)
{
  if (b == bank)
    return true;
  if (banks[b].chip == NULL)
    return true;

  chip = banks[b].chip;
  chip_begin = banks[b].chip_begin;
  bank= b;

  return true;
}

void
cl_banker::print_info(chars pre, class cl_console_base *con)
{
  int b;
  con->dd_printf(pre);
  //con->dd_printf("  banked area= ");
  if (address_space)
    {
      con->dd_printf("%s ", address_space->get_name("unknown"));
      con->dd_printf(address_space->addr_format, as_begin);
      con->dd_printf(" ");
      con->dd_printf(address_space->addr_format, as_end);
    }
  else
    con->dd_printf("x");
  con->dd_printf(" -> banked\n");

  con->dd_printf(pre);
  con->dd_printf("  bank selector: %s[", banker_as->get_name("unknown"));
  con->dd_printf(banker_as->addr_format, banker_addr);
  con->dd_printf("] mask=0x%x banks=%d act=%d\n",
		 banker_mask, nuof_banks,
		 b= actual_bank());

  con->dd_printf(pre);
  con->dd_printf("  banks:\n");

  int i;
  for (i= 0; i < nuof_banks; i++)
    {
      con->dd_printf(pre);
      con->dd_printf("    %c %2d. ", (b==i)?'*':' ', i);
      if (banks[i].chip)
        {
          con->dd_printf("%s ", banks[i].chip->get_name("unknown"));
          con->dd_printf(banks[i].chip->addr_format, banks[i].chip_begin);
        }
      else
        con->dd_printf("x");
      con->dd_printf("\n");
    }
}


/* 
 * Bit bander
 */

cl_bander::cl_bander(class cl_address_space *the_as,
		     t_addr the_asb,
		     t_addr the_ase,
		     class cl_memory *the_chip,
		     t_addr the_cb,
		     int the_bpc,
		     int the_distance):
  cl_address_decoder(the_as, the_chip, the_asb, the_ase, the_cb)
{
  bpc= the_bpc;
  distance= the_distance;
}

bool
cl_bander::activate(class cl_console_base *con)
{
  address_space->undecode_area(this, as_begin, as_end, con);

  t_addr asa, ca;
  int b, m;
  for (asa= as_begin, ca= chip_begin, b= 0, m= 1;
       asa <= as_end;
       asa++)
    {
      if (b >= bpc)
	{
	  ca+= distance;
	  b= 0;
	  m= 1;
	}
      t_mem *slot= chip->get_slot(ca);
      cl_memory_cell *c= address_space->get_cell(asa);
      c->decode(slot, m);
      b++;
      m<<= 1;
    }
  return activated= true;
}

void
cl_bander::print_info(chars pre, class cl_console_base *con)
{
  if (address_space &&
      address_space->hidden)
    return;
  if (chip &&
      chip->hidden)
    return;
  con->dd_printf(pre);
  if (address_space)
    {
      con->dd_printf("%s ", address_space->get_name("unknown"));
      con->dd_printf(address_space->addr_format, as_begin);
      con->dd_printf(" ");
      con->dd_printf(address_space->addr_format, as_end);
    }
  else
    con->dd_printf("x");
  con->dd_printf(" -> bander(%d/%d) ", bpc, distance);
  if (chip)
    {
      con->dd_printf("%s ", chip->get_name("unknown"));
      con->dd_printf(chip->addr_format, chip_begin);
    }
  else
    con->dd_printf("x");
  con->dd_printf(" %s\n", (activated)?"activated":"inactive");
}


/*
 * List of address decoders
 */

cl_decoder_list::cl_decoder_list(t_index alimit, t_index adelta, bool bychip):
  cl_sorted_list(alimit, adelta, "decoder list")
{
  Duplicates= true;
  by_chip= bychip;
}

void *
cl_decoder_list::key_of(void *item)
{
  class cl_address_decoder *d= (class cl_address_decoder *)item;
  if (by_chip)
    return(&(d->chip_begin));
  else
    return(&(d->as_begin));
}

int
cl_decoder_list::compare(const void *key1, const void *key2)
{
  t_addr k1= *((t_addr*)key1), k2= *((t_addr*)key2);
  if (k1 == k2)
    return(0);
  else if (k1 > k2)
    return(1);
  return(-1);
}


/*
 * Errors in memory handling
 */

/* All of memory errors */

cl_error_mem::cl_error_mem(class cl_memory *amem, t_addr aaddr)
{
  mem= amem;
  addr= aaddr;
  classification= mem_error_registry.find("memory");
}

/* Invalid address in memory access */

cl_error_mem_invalid_address::
cl_error_mem_invalid_address(class cl_memory *amem, t_addr aaddr):
  cl_error_mem(amem, aaddr)
{
  classification= mem_error_registry.find("invalid_address");
}

void
cl_error_mem_invalid_address::print(class cl_commander_base *c)
{
  //FILE *f= c->get_out();
  /*cmd_fprintf(f,*/c->dd_printf("%s: invalid address ", get_type_name());
  /*cmd_fprintf(f,*/c->dd_printf(mem->addr_format, addr);
  /*cmd_fprintf(f,*/c->dd_printf(" in memory %s.\n", mem->get_name());
}

/* Non-decoded address space access */

cl_error_mem_non_decoded::
cl_error_mem_non_decoded(class cl_memory *amem, t_addr aaddr):
  cl_error_mem(amem, aaddr)
{
  classification= mem_error_registry.find("non_decoded");
}

void
cl_error_mem_non_decoded::print(class cl_commander_base *c)
{
  //FILE *f= c->get_out();
  /*cmd_fprintf(f,*/c->dd_printf("%s: access of non-decoded address ", get_type_name());
  /*cmd_fprintf(f,*/c->dd_printf(mem->addr_format, addr);
  /*cmd_fprintf(f,*/c->dd_printf(" in memory %s.\n", mem->get_name());
}

cl_mem_error_registry::cl_mem_error_registry(void)
{
  class cl_error_class *prev = mem_error_registry.find("non-classified");
  prev = register_error(new cl_error_class(err_error, "memory", prev, ERROR_OFF));
  prev = register_error(new cl_error_class(err_error, "invalid_address", prev));
  prev = register_error(new cl_error_class(err_error, "non_decoded", prev));
}

/* End of mem.cc */
