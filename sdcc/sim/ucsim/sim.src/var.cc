/*
 * Simulator of microcontrollers (sim.src/var.cc)
 *
 * Copyright (C) @@S@@,@@Y@@ Drotos Daniel, Talker Bt.
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

#include "varcl.h"


cl_var::cl_var(const char *iname, class cl_address_space *ias, t_addr iaddr, chars adesc, int ibitnr_high, int ibitnr_low):
  cl_base()
{
  as= ias;
  addr= iaddr;
  if (ibitnr_low < ibitnr_high)
    {
      bitnr_low= ibitnr_low;
      bitnr_high= ibitnr_high;
    }
  else
    {
      bitnr_low= ibitnr_high;
      bitnr_high= ibitnr_low;
    }
  desc= adesc;
  
  set_name(iname);
  
  cell= NULL;
}

int
cl_var::init(void)
{
  if (!as ||
      !as->is_address_space() ||
      !as->valid_address(addr))
    return 0;
  cell= as->get_cell(addr);
  if (cell)
    cell->set_flag(CELL_VAR, true);
  return 0;
}


void
cl_var::print_info(cl_console_base *con)
{
  con->dd_printf("%s ", get_name("?"));
  if (cell)
    {
      con->dd_printf("%s", as->get_name("?"));
      con->dd_printf("[");
      con->dd_printf(as->addr_format, addr);
      con->dd_printf("]");
      t_mem m= cell->get();
      if (bitnr_low >= 0)
        {
          if (bitnr_high >= 0 && bitnr_high != bitnr_low)
            {
              con->dd_printf("[%u:%u] = 0b", bitnr_high, bitnr_low);
              for (int i= bitnr_high; i >= bitnr_low; i--)
                con->dd_printf("%c", (m & (1U << i)) ? '1' : '0');
              con->dd_printf("\n");
            }
          else
            con->dd_printf(".%u = %c", bitnr_low, (m & (1U << bitnr_low)) ? '1' : '0');
        }
      else
        {
          con->dd_printf(" = ");
          con->dd_printf(as->data_format, m);
        }
    }
  con->dd_printf("\n");
  if (!desc.empty())
    con->dd_printf("  %s\n", (char*)desc);
}


void *
cl_var_list::key_of(void *item)
{
  class cl_var *v= (class cl_var *)item;
  return (void*)v->get_name();
}

int
cl_var_list::compare(void *key1, void *key2)
{
  char *k1, *k2;

  k1= (char*)key1;
  k2= (char*)key2;
  if (k1 && k2)
    return strcmp(k1, k2);
  return 0;
}


struct cell_name_matcher_args {
  class cl_memory_cell *cell;
  int bitnr_high;
  int bitnr_low;
};

static int
cell_name_matcher(void *a, void *b)
{
  class cl_var *v = (class cl_var *)a;
  struct cell_name_matcher_args *args = (struct cell_name_matcher_args *)b;
  return (v->get_cell() == args->cell && v->bitnr_low == args->bitnr_low && v->bitnr_high == args->bitnr_high);
}

const char *
cl_var_list::cell_name(class cl_memory_cell *cell, int bitnr_high, int bitnr_low)
{
  if (cell->get_flag(CELL_VAR))
    {
      struct cell_name_matcher_args args = { .cell = cell, .bitnr_high = bitnr_high, .bitnr_low = bitnr_low };
      class cl_var *v = (cl_var *)first_that(cell_name_matcher, &args);
      if (v)
        return v->get_name();
    }
  return NULL;
}


/* End of sim.src/var.cc */
