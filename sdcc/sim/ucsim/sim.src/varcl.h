/*
 * Simulator of microcontrollers (sim.src/varcl.h)
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

#ifndef SIM_VARCL_HEADER
#define SIM_VARCL_HEADER


#include "pobjcl.h"

#include "newcmdcl.h"

#include "memcl.h"


class cl_var: public cl_base
{
 public:
  class cl_memory *mem;
  t_addr addr;
  int bitnr_high, bitnr_low;
  int instance;
  chars desc;
 protected:
  class cl_memory_cell *cell;
 public:
  cl_var(const char *iname, class cl_memory *imem, t_addr iaddr, chars adesc, int ibitnr_high= -1, int ibitnr_low= -1);
  virtual int init(void);
  virtual class cl_memory_cell *get_cell(void) { return cell; }
  
  virtual void print_info(cl_console_base *con);
};


class cl_var_by_name_list: public cl_sorted_list
{
 public:
  cl_var_by_name_list(): cl_sorted_list(10, 10, "symlist") {}
  virtual ~cl_var_by_name_list(void);

  virtual class cl_var *at(t_index index) { return (cl_var *)cl_sorted_list::at(index); }
  virtual void *key_of(void *item);
  virtual int compare(const void *key1, const void *key2);
};

class cl_var_by_addr_list: public cl_sorted_list
{
 public:
  cl_var_by_addr_list(): cl_sorted_list(10, 10, "symlist_by_addr") {}
  virtual ~cl_var_by_addr_list(void);

  virtual class cl_var *at(t_index index) { return (cl_var *)cl_sorted_list::at(index); }
  virtual int compare_addr(class cl_var *var, class cl_memory *mem, t_addr addr, int bitnr_high, int bitnr_low);
  virtual int compare(const void *key1, const void *key2);
  virtual bool search(class cl_memory *mem, t_addr addr, int bitnr_high, int bitnr_low, t_index &index);
};

class cl_var_list: public cl_base
{
 private:
  int max_name_len = 0;
 public:
  class cl_var_by_name_list by_name;
  class cl_var_by_addr_list by_addr;

  cl_var_list() {}

  virtual void add(cl_var *item);
  virtual void del(const char *name);
  virtual int get_max_name_len(void);
};


#endif

/* End of sim.src/varcl.h */
