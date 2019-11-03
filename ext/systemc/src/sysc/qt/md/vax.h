/*
 * QuickThreads -- Threads-building toolkit.
 * Copyright (c) 1993 by David Keppel
 *
 * Permission to use, copy, modify and distribute this software and
 * its documentation for any purpose and without fee is hereby
 * granted, provided that the above copyright notice and this notice
 * appear in all copies.  This software is provided as a
 * proof-of-concept and for demonstration purposes; there is no
 * representation about the suitability of this software for any
 * purpose.
 */

#ifndef QUICKTHREADS_VAX_H
#define QUICKTHREADS_VAX_H

typedef unsigned long qt_word_t;

/* Thread's initial stack layout on the VAX:

   non-varargs:

   +---
   | arg[2]	=== `userf' on startup
   | arg[1]	=== `pt' on startup
   | arg[0]	=== `pu' on startup
   | ...	=== `only' on startup.
   +---
   | ret pc	=== `qt_start' on startup
   | fp		=== 0 on startup
   | ap		=== 0 on startup
   | <mask>
   | 0 (handler)			<--- qt_t.sp
   +---

   When a non-varargs thread is started, it ``returns'' to the start
   routine, which calls the client's `only' function.

   The varargs case is clearly bad code.  The various values should be
   stored in a save area and snarfed in to callee-save registers on
   startup.  However, it's too painful to figure out the register
   mask (right now), so do it the slow way.

   +---
   | arg[n-1]
   | ..
   | arg[0]
   | nargs
   +---
   |		=== `cleanup'
   |		=== `vuserf'
   |		=== `startup'
   |		=== `pt'
   +---
   | ret pc	=== `qt_start' on startup
   | fp		=== 0 on startup
   | ap		=== 0 on startup
   | <mask>
   | 0 (handler)			<--- qt_t.sp
   +---

   When a varargs thread is started, it ``returns'' to the `qt_vstart'
   startup code.  The startup code pops all the extra arguments, then
   calls the appropriate functions. */


/* What to do to start a thread running. */
extern void qt_start (void);
extern void qt_vstart (void);


/* Initial call frame for non-varargs and varargs cases. */
#define QUICKTHREADS_STKBASE	(10 * 4)
#define QUICKTHREADS_VSTKBASE	(9 * 4)


/* Stack "must be" 4-byte aligned.  (Actually, no, but it's
   easiest and probably fastest to do so.) */

#define QUICKTHREADS_STKALIGN	(4)


/* Where to place various arguments. */
#define QUICKTHREADS_ONLY_INDEX	(5)
#define QUICKTHREADS_USER_INDEX	(8)
#define QUICKTHREADS_ARGT_INDEX	(7)
#define QUICKTHREADS_ARGU_INDEX	(6)

#define QUICKTHREADS_VSTARTUP_INDEX	(6)
#define QUICKTHREADS_VUSERF_INDEX		(7)
#define QUICKTHREADS_VCLEANUP_INDEX	(8)
#define QUICKTHREADS_VARGT_INDEX		(5)


/* Stack grows down.  The top of the stack is the first thing to
   pop off (predecrement, postincrement). */
#define QUICKTHREADS_GROW_DOWN


extern void qt_error (void);

#define QUICKTHREADS_VAX_GMASK_NOREGS	(0)

/* Push on the error return address, null termination to call chains,
   number of arguments to `only', register save mask (save no
   registers). */

#define QUICKTHREADS_ARGS_MD(sto) \
    (QUICKTHREADS_SPUT (sto, 0, 0), \
     QUICKTHREADS_SPUT (sto, 1, QUICKTHREADS_VAX_GMASK_NOREGS), \
     QUICKTHREADS_SPUT (sto, 2, 0), \
     QUICKTHREADS_SPUT (sto, 3, 0), \
     QUICKTHREADS_SPUT (sto, 4, qt_start))

#define QUICKTHREADS_VARGS_MD0(sto, nbytes) \
    (QUICKTHREADS_SPUT (sto, (-(nbytes)/4)-1, (nbytes)/4), \
     ((char *)(((sto)-4) - QUICKTHREADS_STKROUNDUP(nbytes))))

#define QUICKTHREADS_VARGS_ADJUST(sp)	((char *)sp + 4)

#define QUICKTHREADS_VARGS_MD1(sto) \
    (QUICKTHREADS_SPUT (sto, 0, 0), \
     QUICKTHREADS_SPUT (sto, 1, QUICKTHREADS_VAX_GMASK_NOREGS), \
     QUICKTHREADS_SPUT (sto, 2, 0), \
     QUICKTHREADS_SPUT (sto, 3, 0), \
     QUICKTHREADS_SPUT (sto, 4, qt_vstart))

#define QUICKTHREADS_VARGS_DEFAULT

#endif /* QUICKTHREADS_VAX_H */
