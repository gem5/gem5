/*****************************************************************************

  Licensed to Accellera Systems Initiative Inc. (Accellera) under one or
  more contributor license agreements.  See the NOTICE file distributed
  with this work for additional information regarding copyright ownership.
  Accellera licenses this file to you under the Apache License, Version 2.0
  (the "License"); you may not use this file except in compliance with the
  License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
  implied.  See the License for the specific language governing
  permissions and limitations under the License.

 *****************************************************************************/

/*****************************************************************************

  star110089.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/***********************************************************************

Things I had to do to make this work with the SystemC compiler

Remove LP_STATS-related #ifdefs

Added SystemC module interface.  Odd function wrapper, wait()s.

Removed use of pointers into lp_table (replaced with array references).

Global variables are not allowed, so move lp_table into the function.

Pointers appear to have problems, so comment out test of the "dev" field.

The LP_POLLED macro checks lp_table[minor].dev->port->irq (three dereferences),
which is not synthesizable.  I replaced it with a constant.

w_ctr writes to the control I/O port.  We want to observe this, so added
a port to the module, made the function a member function (not just static).

Want to observe successive values on the control port, so added a wait() after
it is written.

Adding the main do loop (there's an if () return statement earlier)
gives "WAITs not balanced" errors, even without any wait() statements
at all.  Adding them doesn't appear to help (Evidentally, I'm not
adding them correctly.)

How to model memory?  Specifically, how to model copy_from_user() and
accesses to buffer memory?  I'll create address, data, and read/write ports
for each memory space, memory access functions that toggle these ports,
and finally a replacement for copy_from_user().

Pointers seem to be broken (BC gives the inscrutable

    Error: Width mismatch on port 'n5[1]' of reference to 'group1' in
   'lp_write_buf.db:loop_117_design'. (LINK-3)

so I'll use a pchar_t type as a replacement.

The const declarations don't work now with typedef unsigned int pchar_t, so
I removed them.

Was getting "unschedulable" errors until I inserted a few wait() statements.
(Very unhelpful diagnostics.)

Inserting and removing wait()s at will seems to help, but I'm still getting
width mismatch errors.

The width mismatch errors are due to a SystemC compiler bug.  Adding
-Xgroup_idx_logic=0 to the command line eliminates them.

Many "mismatched clock" statements appear to be due to the use of nested
return stataments.  BC doesn't support exits from any more than a single
level of loop nesting, so whatever hack the SystemC people did is not solid.

if statements with a return in one branch seems to cause unsolvable wait
balance problems.  I'll rewrite many of them to put the code in the "else"
branch.

For if-then with a return within a loop, I appear to need to add an error
flag and checks for it.

Doing this makes the system schedulable, but even these few lines of code
turn into over 1 MB of .db file, take 300MB of core and ten minutes to run.
This corresponds to roughly 140 lines of C code: four conditionals, two
nested loops, and one function call.

***********************************************************************/

typedef unsigned int pchar_t;

/**********************************************************************

SystemC-specific things

**********************************************************************/

#include "systemc.h"

SC_MODULE(lp_write_buf) {
  sc_in_clk         clk;
  sc_in<bool>       reset;
  sc_in<unsigned>   minor;
  sc_in<unsigned>   buf;   /* a pointer */
  sc_in<int>        count;

  sc_out<int>       result;       /* return code */
  sc_out<bool>      result_ready; /* return flag */

  sc_out<char>      control;      /* printer control port */

  sc_inout<unsigned>  user_addr;	  /* user memory address */
  sc_in<char>       user_read_data; /* user data value */
  sc_out<bool>      user_read;    /* user memory read flag */

  sc_out<unsigned>  kernel_addr;  /* kernel memory address */
  sc_in<char>       kernel_read_data; /* kernel data value */
  sc_out<char>      kernel_write_data; /* kernel data value */
  sc_out<bool>      kernel_read;
  sc_out<bool>      kernel_write;

  sc_out<bool>      invalid_read; /* debugging flag: should never be true */

  void lp_write_buf_body();
  int lp_write_buf_f(unsigned int minor,  pchar_t buf, int count);

  /* User and kernel memory operations */
  char read_from_user(unsigned a);
  char read_from_kernel(unsigned a);
  void write_to_kernel(unsigned a, char d); 

  int copy_from_user(pchar_t,  pchar_t, unsigned long);

  /* Output verification */
  void check_read_address();
  
  SC_CTOR(lp_write_buf)
  {
    SC_CTHREAD(lp_write_buf_body, clk.pos());
    reset_signal_is(reset,true);
    SC_CTHREAD(check_read_address, clk.pos());
    reset_signal_is(reset,true);
  }
};

void lp_write_buf::check_read_address()
{
  /* This causes the systemC compiler a fatal internal error */
  for (;;) {
    invalid_read = user_addr < buf || user_addr >= buf + count;
    wait();
  }
}


void lp_write_buf::lp_write_buf_body()
{
  unsigned dummy; /* These three statements compensate for BC weirdness */
  dummy = 0;
  wait();
  for (;;) {
    result = 0; result_ready = 0;
    unsigned res = lp_write_buf_f(minor, buf, count);
    result = (int) res; result_ready = 1;
    wait();
    result_ready = 0;
    wait();
  }
}

char lp_write_buf::read_from_user(unsigned a)
{
  char d;
  user_addr = a;
  user_read = 1;
  wait();
  d = user_read_data;
  user_read = 0;
  wait();
  return d;
}

char lp_write_buf::read_from_kernel(unsigned a)
{
  char d;
  kernel_addr = a;
  kernel_read = 1;
  wait();
  d = kernel_read_data;
  kernel_read = 0;
  wait();
  return d;
}

void lp_write_buf::write_to_kernel(unsigned a, char d)
{
  kernel_addr = a;
  kernel_write_data = d;
  kernel_write = 1;
  wait();
  kernel_write = 0;
  wait();
}

int lp_write_buf::copy_from_user(pchar_t src,  pchar_t dest,
				 unsigned long count)
{
  for ( ; count > 0 ; count--)
    write_to_kernel(dest++, read_from_user(src++));
  /* What about an error? */
  return 0;
}

/********************************************************************
  "Normal" C begins here
 ********************************************************************/

#define	ENXIO		 6	/* No such device or address */
#define	EFAULT		14	/* Bad address */

// #define NULL             0

#define LP_PINTEN	0x10  /* high to read data in or-ed with data out */
#define LP_PSELECP	0x08  /* inverted output, active low */
#define LP_PINITP	0x04  /* unchanged output, active low */
#define LP_PAUTOLF	0x02  /* inverted output, active low */
#define LP_PSTROBE	0x01  /* short high output on raising edge */

/* #define LP_POLLED(minor) (lp_table[(minor)].dev->port->irq == PARPORT_IRQ_NONE) */
#define LP_POLLED(minor) 1

#define w_ctr(dev,val) (control = (val))

#define LP_NO 3
#define LP_BUFFER_SIZE 32

struct lp_struct {
  /* struct pardevice *dev; */
  pchar_t dev;
  unsigned long flags;
  unsigned int chars;
  unsigned int time;
  /*	unsigned int wait; */
  pchar_t lp_buffer;
  /* struct wait_queue *wait_q; */
  unsigned int last_error;
  volatile unsigned int irq_detected:1;
  volatile unsigned int irq_missed:1;
};

/* Write count bytes starting at buf in user space to the parallel port
   defined by the minor device number */

int lp_write_buf::lp_write_buf_f(unsigned int minor,
				 pchar_t buf, int count)
{
  int err;
  struct lp_struct lp_table[LP_NO];

  /* Added to fake a device */
  lp_table[minor].dev = 5;

  unsigned long copy_size;
  unsigned long total_bytes_written = 0;
  unsigned long bytes_written;
  /* Removed because pointers are prohibited */
  /* struct lp_struct *lp = &lp_table[minor]; */

  if (minor >= LP_NO) {
    wait();
    return -ENXIO;
  } else {

    /*   if (lp->dev == NULL) */ /* Removed because of a pointer */
    /* The following causes a BC error (bad matching width of n15[1]) */
    if (lp_table[minor].dev == 0) {
      wait();
      return -ENXIO;
    } else {

      lp_table[minor].last_error = 0;
      lp_table[minor].irq_detected = 0;
      lp_table[minor].irq_missed = 1;

      if (LP_POLLED(minor))
	w_ctr(minor, LP_PSELECP | LP_PINITP);
      else
	w_ctr(minor, LP_PSELECP | LP_PINITP | LP_PINTEN);

      err = 0;
      wait();
      do {
	wait();
	bytes_written = 0;
	copy_size = (count <= LP_BUFFER_SIZE ? count : LP_BUFFER_SIZE);


	/* Adding this gives width mismatch errors */
	if (copy_from_user(lp_table[minor].lp_buffer, buf, copy_size)) {
	  wait();
	  w_ctr(minor, LP_PSELECP | LP_PINITP);
	  // return -EFAULT;
	  err = -EFAULT;
	} else {

	/** Fake! **/
	bytes_written = copy_size;

#if 0

	while (copy_size) {
	  if (lp_char(lp->lp_buffer[bytes_written], minor)) {
	    --copy_size;
	    ++bytes_written;
	  } else {
	    int rc = total_bytes_written + bytes_written;


	    if (signal_pending(current))
	      {
		w_ctr(minor, LP_PSELECP | LP_PINITP);
		if (total_bytes_written + bytes_written)
		  return total_bytes_written + bytes_written;
		else
		  return -EINTR;
	      }

	    if (lp_check_status(minor))
	      {
		w_ctr(minor, LP_PSELECP | LP_PINITP);
		return rc ? rc : -EIO;
	      }

	    if (LP_POLLED(minor) ||
		lp_table[minor].irq_missed)
	      {
	      lp_polling:
		current->state = TASK_INTERRUPTIBLE;
		lp_schedule(minor, LP_TIME(minor));
	      } else {
		cli();
		if (LP_PREEMPTED(minor))
		  {
		    /*
		     * We can' t sleep on the interrupt
		     * since another pardevice need the port.
		     * We must check this in a cli() protected
		     * envinroment to avoid parport sharing
		     * starvation.
		     */
		    sti();
		    goto lp_polling;
		  }
		if (!lp_table[minor].irq_detected)
		  interruptible_sleep_on_timeout(&lp->wait_q, LP_TIMEOUT_INTERRUPT);
		sti();
	      }
	  }
	}

#endif

	total_bytes_written += bytes_written;
	buf += bytes_written;
	count -= bytes_written;

	wait();
	}

      } while (count > 0 && err >= 0);

      wait();

      if (err >= 0 ) {

	w_ctr(minor, LP_PSELECP | LP_PINITP);
	return total_bytes_written;
      } else {
	return err;
      }

    }
  }
}

