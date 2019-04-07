/*
 * Simulator of microcontrollers (cmd.src/timer.cc)
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

#include "stdio.h"
#include "i_string.h"

// prj
#include "globals.h"
#include "utils.h"

// sim
#include "simcl.h"

// local
#include "cmd_timercl.h"


void
set_timer_help(class cl_cmd *cmd)
{
  cmd->set_help("timer subcommand",
		"Manage timers",
		"Long of timer");
}

/*
 * Command: timer
 *----------------------------------------------------------------------------
 */

//int
//cl_timer_cmd::do_work(class cl_sim *sim,
//		      class cl_cmdline *cmdline, class cl_console *con)
COMMAND_DO_WORK_UC(cl_timer_cmd)
{
  class cl_cmd_arg *param= cmdline->param(0);

  if (!param)
    {
      con->dd_printf("Timer id is missing.");
      return(false);
    }
  if (param->as_number())
    {
      as_nr= true;
      id_str= NULL;
      id_nr= param->value.number;
      if (id_nr < 0)
	{
	  con->dd_printf("Error: "
			 "Timer id must be greater than zero or a string\n");
	  return(true);
	}
      else if (id_nr == 0)
        ticker= uc->ticks;
      else
        ticker= uc->get_counter(id_nr);
    }
  else
    {
      as_nr= false;
      id_str= strdup(param->s_value);
      ticker= uc->get_counter(id_str);
    }
  cmdline->shift();
  return(false);
}

CMDHELP(cl_timer_cmd,
	"timer subcommand",
	"Manage timers",
	"long help of timer")

/*
 * Command: timer add
 *-----------------------------------------------------------------------------
 * Add a new timer to the list
 */

COMMAND_DO_WORK_UC(cl_timer_add_cmd)
  //add(class cl_uc *uc, class cl_cmdline *cmdline, class cl_console *con)
{
  if (cl_timer_cmd::do_work(uc, cmdline, con))
    return(false);
  if (ticker)
    {
      if (id_str)
        {
          con->dd_printf("Error: Timer \"%s\" already exists\n", id_str);
          free((char *)id_str);
        }
      else
	con->dd_printf("Error: Timer %d already exists\n", id_nr);
      return(false);
    }

  class cl_cmd_arg *param = cmdline->param(0);
  int arg_i = 1;
  bool rtime = false;
  double freq = uc->ticks->freq;
  int dir = +1;
  enum cpu_state state = stUNDEF;
  bool inisr = false;
  bool error = false;

  // Historical syntax was: timer add <name>|<n> <step> [<isr-flag>]
  // Current syntax is: timer add <name>|<n> [[<step> [<isr-flag>] | <state>] [key [value]]...]
  if (param && param->as_number())
    {
      state = stGO;
      dir = param->value.number;
      freq = uc->xtal / uc->clock_per_cycle();
      param = cmdline->param(arg_i++);

      if (param && param->as_number())
        {
          inisr = (param->value.number != 0);
          param = cmdline->param(arg_i++);
        }
    }
  else if (param && param->as_string())
    {
      // "powerdown" was previously called "halt" and "run" was called "main".
      // Furthermore "run" (aka "main") only counts non-interrupt execution
      // and there is an "inisr" that counts excution within an interrupt
      // service routine.
      if (!strcmp(param->value.string.string, "halt"))
        {
          state = stPD;
          freq = uc->xtal;
          dir = +1;
        }
      else if (!strcmp(param->value.string.string, "inisr") ||
          !strcmp(param->value.string.string, "isr"))
        {
          state = stGO;
          inisr = true;
          freq = uc->xtal / uc->clock_per_cycle();
          dir = +1;
        }
      else if (!strcmp(param->value.string.string, "main"))
        {
          state = stGO;
          inisr = false;
          freq = uc->xtal / uc->clock_per_cycle();
          dir = +1;
        }
      else
        {
          struct id_element *e;
          for (e = cpu_states; e->id_string && strcasecmp(e->id_string, param->value.string.string); e++);
          if (e->id_string)
            {
              state = (enum cpu_state)e->id;
              freq = (state == stGO ? uc->xtal / uc->clock_per_cycle() : uc->xtal);
              dir = +1;
            }
          else
            error = true;
        }
      param = cmdline->param(arg_i++);
    }

  while (!error && param)
    {
      if (!param->as_string())
        error = true;
      else
        {
          if (!strcmp(param->value.string.string, "rtime"))
            {
              // A (simulated) real-time timer for a _specific_ state.
              rtime = true;
            }
          else if (!strcmp(param->value.string.string, "down") ||
              !strcmp(param->value.string.string, "dec"))
            dir = -1;
          else if (!strcmp(param->value.string.string, "up") ||
              !strcmp(param->value.string.string, "inc"))
            dir = +1;
          else
            {
              class cl_cmd_arg *arg = cmdline->param(arg_i++);

              if (!arg)
                error = true;
              else
                {
                  if (!strcmp(param->value.string.string, "freq"))
                    error = ((freq = strtod_unscaled((char *)cmdline->tokens->at(arg_i-1))) == 0.0);
                  else if (!strcmp(param->value.string.string, "step") && arg->as_number())
                    dir = arg->value.number;
                  else
                    error = true;
                }
            }
        }

      param = cmdline->param(arg_i++);
    }

  if (error || param)
    con->dd_printf("%s\n", (short_help ? short_help : "Error: wrong syntax\n"));
  else
    {
      ticker= new cl_ticker(id_str, rtime, (rtime ? uc->ticks->freq : freq), dir, state, inisr);
      uc->add_counter(ticker, ticker->get_name());
    }

  if (!id_str)
    free((char *)id_str);

  return(false);
}

CMDHELP(cl_timer_add_cmd,
        "timer add <name>|<n> [[<step> [<isr-flag>] | isr|halt|main|<cpustate>] [rtime] [freq <n>] [step <n>]\n",
	"Create a clock counter (timer)",
	"log help of timer add")

/*
 * Command: timer delete
 *-----------------------------------------------------------------------------
 * Delete a timer from the list
 */

COMMAND_DO_WORK_UC(cl_timer_delete_cmd)
  //del(class cl_uc *uc, class cl_cmdline *cmdline, class cl_console *con)
{
  if (cl_timer_cmd::do_work(uc, cmdline, con))
    return(false);
  if (!ticker)
    {
      if (id_str)
        {
	  con->dd_printf("Timer \"%s\" does not exist\n", id_str);
          free((char *)id_str);
        }
      else
	con->dd_printf("Timer %d does not exist\n", id_nr);
      return(false);
    }
  if (id_str)
    {
      uc->del_counter(id_str);
      free((char *)id_str);
    }
  else
    uc->del_counter(id_nr);

  return(false);
}

CMDHELP(cl_timer_delete_cmd,
	"timer delete <name>|<id>",
	"Delete a timer",
	"long help of timer delete")

/*
 * Command: timer get
 *-----------------------------------------------------------------------------
 * Get the value of just one timer or all of them
 */

COMMAND_DO_WORK_UC(cl_timer_get_cmd)
  //get(class cl_uc *uc, class cl_cmdline *cmdline, class cl_console *con)
{
  if (cmdline->nuof_params())
    {
      if (cl_timer_cmd::do_work(uc, cmdline, con))
	return(false);
    }
  else
    ticker= 0;
  if (ticker)
    ticker->dump(uc, id_nr, con);
  else
    {
      uc->ticks->dump(uc, 0, con);
      for (id_nr= 0; id_nr < uc->counters->count; id_nr++)
	{
	  ticker= uc->get_counter(id_nr);
	  if (ticker)
	    ticker->dump(uc, id_nr, con);
	}
    }

  if (id_str)
    free((char *)id_str);

  return(false);
}

CMDHELP(cl_timer_get_cmd,
	"timer get [<name>|<id>]",
	"Get value of a timer, or all",
	"long help of timer get")

/*
 * Command: timer run
 *-----------------------------------------------------------------------------
 * Allow a timer to run
 */

COMMAND_DO_WORK_UC(cl_timer_run_cmd)
  //run(class cl_uc *uc, class cl_cmdline *cmdline, class cl_console *con)
{
  if (cl_timer_cmd::do_work(uc, cmdline, con))
    return(false);
  if (!ticker)
    {
      if (id_str)
        {
	  con->dd_printf("Timer \"%s\" does not exist\n", id_str);
          free((char *)id_str);
        }
      else
	con->dd_printf("Timer %d does not exist\n", id_nr);
      return(0);
    }
  ticker->run = true;

  if (id_str)
    free((char *)id_str);

  return(false);
}

CMDHELP(cl_timer_run_cmd,
	"timer start <name>|<id>",
	"Start a timer",
	"long help of timer run")

/*
 * Command: timer stop
 *-----------------------------------------------------------------------------
 * Stop a timer
 */

COMMAND_DO_WORK_UC(cl_timer_stop_cmd)
  //stop(class cl_uc *uc, class cl_cmdline *cmdline, class cl_console *con)
{
  if (cl_timer_cmd::do_work(uc, cmdline, con))
    return(false);

  if (!ticker)
    {
      if (id_str)
        {
	  con->dd_printf("Timer \"%s\" does not exist\n", id_str);
          free((char *)id_str);
        }
      else
	con->dd_printf("Timer %d does not exist\n", id_nr);
      return(false);
    }
  ticker->run = false;

  if (id_str)
    free((char *)id_str);

  return(false);
}

CMDHELP(cl_timer_stop_cmd,
	"timer stop <name>|<id>",
	"Stop a timer",
	"long help of timer stop")

/*
 * Command: timer ticks
 *-----------------------------------------------------------------------------
 * Set a timer to a specified tick count
 */

COMMAND_DO_WORK_UC(cl_timer_ticks_cmd)
  //val(class cl_uc *uc, class cl_cmdline *cmdline, class cl_console *con)
{
  char *arg;

  if (cl_timer_cmd::do_work(uc, cmdline, con))
    return(false);
  if (!ticker)
    {
      if (id_str)
	con->dd_printf("Error: Timer \"%s\" does not exist\n", id_str);
      else
	con->dd_printf("Error: Timer %d does not exist\n", id_nr);
    }
  else if ((arg = (char *)cmdline->tokens->at(0)))
    ticker->ticks= strtod_unscaled(arg);
  else
    con->dd_printf("Error: Value is missing\n");

  if (id_str)
    free((char *)id_str);

  return(false);
}

CMDHELP(cl_timer_ticks_cmd,
	"timer ticks <name>|<id> value",
	"Set a timer value",
	"long help of timer ticks")

/*
 * Command: timer time
 *-----------------------------------------------------------------------------
 * Set a timer to a specified time
 */

COMMAND_DO_WORK_UC(cl_timer_time_cmd)
  //val(class cl_uc *uc, class cl_cmdline *cmdline, class cl_console *con)
{
  char *arg;

  if (cl_timer_cmd::do_work(uc, cmdline, con))
    return(false);
  if (!ticker)
    {
      if (id_str)
	con->dd_printf("Error: Timer \"%s\" does not exist\n", id_str);
      else
	con->dd_printf("Error: Timer %d does not exist\n", id_nr);
    }
  else if ((arg = (char *)cmdline->tokens->at(0)))
    ticker->ticks= strtod_unscaled(arg) * ticker->freq;
  else
    con->dd_printf("Error: Value is missing\n");

  if (id_str)
    free((char *)id_str);

  return(false);
}

CMDHELP(cl_timer_time_cmd,
	"timer time <name>|<id> value",
	"Set a timer value",
	"long help of timer time")

/* End of cmd.src/cmd_timer.cc */
