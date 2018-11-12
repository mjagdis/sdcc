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


cl_var::cl_var(const char *iname, class cl_memory *imem, t_addr iaddr, chars adesc, int ibitnr_high, int ibitnr_low):
  cl_base()
{
  mem= imem;
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
  if (!mem ||
      !mem->is_address_space() ||
      !mem->valid_address(addr))
    return 0;
  cell= ((cl_address_space *)mem)->get_cell(addr);
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
      con->dd_printf("%s", mem->get_name("?"));
      con->dd_printf("[");
      con->dd_printf(mem->addr_format, addr);
      con->dd_printf("]");
      t_mem m= mem->get(addr);
      if (bitnr_high >= 0)
        {
          if (bitnr_high >= 0 && bitnr_high != bitnr_low)
            {
              con->dd_printf("[%u:%u] = 0b", bitnr_high, bitnr_low);
              for (int i= bitnr_high; i >= bitnr_low; i--)
                con->dd_printf("%c", (m & (1U << i)) ? '1' : '0');
            }
          else
            con->dd_printf(".%u = %c", bitnr_low, (m & (1U << bitnr_low)) ? '1' : '0');
        }
      else
        {
          con->dd_printf(" = ");
          con->dd_printf(mem->data_format, m);
        }
    }
  con->dd_printf("\n");
  if (!desc.empty())
    con->dd_printf("  %s\n", (char*)desc);
}


cl_var_by_name_list::~cl_var_by_name_list(void)
{
}

void *
cl_var_by_name_list::key_of(void *item)
{
  class cl_var *v= (class cl_var *)item;
  return (void*)v->get_name();
}

int
cl_var_by_name_list::compare(void *key1, void *key2)
{
  char *k1, *k2;

  k1= (char*)key1;
  k2= (char*)key2;
  if (k1 && k2)
    return strcmp(k1, k2);
  return 0;
}


cl_var_by_addr_list::~cl_var_by_addr_list(void)
{
}

int
cl_var_by_addr_list::compare_addr(class cl_var *var, class cl_memory *mem, t_addr addr, int bitnr_high, int bitnr_low)
{
  int ret;

  if (!(ret = (var->mem - mem)) &&
     (!(ret = var->addr - addr)) &&
     (!(ret = (var->bitnr_high < 0
               ? (bitnr_high < 0 ? 0 : -1)
               : (bitnr_high < 0
                  ? 1
                  : bitnr_high - var->bitnr_high)))))
    ret = (var->bitnr_low < 0
           ? (bitnr_low < 0 ? 0 : -1)
           : (bitnr_low < 0
              ? 1
              : var->bitnr_low - bitnr_low));

  return ret;
}

int
cl_var_by_addr_list::compare(void *key1, void *key2)
{
  class cl_var *k1 = (cl_var *)key1;
  class cl_var *k2 = (cl_var *)key2;
  int ret;

  // An addr may have multiple names as long as they are all different.
  if (!(ret = compare_addr(k1, k2->mem, k2->addr, k2->bitnr_high, k2->bitnr_low)))
    ret = strcmp(k1->get_name(), k2->get_name());

  return ret;
}

bool
cl_var_by_addr_list::search(class cl_memory *mem, t_addr addr, int bitnr_high, int bitnr_low, t_index &index)
{
  t_index l  = 0;
  t_index h  = count - 1;
  bool    res= false;

  while (l <= h)
    {
      t_index i= (l + h) >> 1;
      t_index c= compare_addr((cl_var *)key_of(Items[i]), mem, addr, bitnr_high, bitnr_low);
      if (c < 0) l= i + 1;
      else
        {
          h= i - 1;
          if (c == 0)
            {
              res= true;
              // We want the _first_ name for the given addr.
              for (l = i; l > 0 && !compare_addr((cl_var *)key_of(Items[l-1]), mem, addr, bitnr_high, bitnr_low); l--);
            }
        }
    }

  index= l;
  return(res);
}

void
cl_var_list::add(cl_var *item)
{
  const char *name = item->get_name();

  if (name)
    {
      int l = strlen(name);
      if (l > max_name_len)
        max_name_len = l;
    }

  by_name.add(item);
  by_addr.add(item);
}


/* End of sim.src/var.cc */
