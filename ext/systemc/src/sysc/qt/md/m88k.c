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

#include <stdarg.h>
#include "qt.h"

/* Varargs is harder on the m88k.  Parameters are saved on the stack as
   something like (stack grows down to low memory; low at bottom of
   picture):

	|  :
	| arg8		<-- va_list.__va_stk
	+---
	|  :
	+---
	| arg7
	|  :
	| iarg0		<-- va_list.__va_reg
	+---
	|  :
	| va_list { __va_arg, __va_stk, __va_reg }
	|  :
	+---

   Here, `va_list.__va_arg' is the number of word-size arguments
   that have already been skipped.  Doubles must be double-arligned.

   What this means for us is that the user's routine needs to be
   called with an arg list where some of the words in the `__va_stk'
   part of the parameter list have to be promoted to registers.

   BUG: doubleword register arguments must be double-aligned.  If
   something is passed as an even # arg and used as an odd # arg or
   vice-versa, the code in the called routine (in the new thread) that
   decides how to adjust the index will get it wrong, because it will
   be expect it to be, say, doubleword aligned and it will really be
   singleword aligned.

   I'm not sure you can solve this without knowing the types of all
   the arguments.  All in all, we never promised varargs would work
   reliably. */



#define QUICKTHREADS_VADJ(sp)	(((char *)sp) - QUICKTHREADS_VSTKBASE)

/* Always allocate at least enough space for 8 args; waste some space
   at the base of the stack to ensure the startup routine doesn't read
   off the end of the stack. */

#define QUICKTHREADS_VARGS_MD0(sp, vabytes) \
   ((qt_t *)(((char *)(sp)) - 8*4 - QUICKTHREADS_STKROUNDUP(vabytes)))

extern void qt_vstart(void);
#define QUICKTHREADS_VARGS_MD1(sp)	(QUICKTHREADS_SPUT (sp, QUICKTHREADS_1, qt_vstart))


  struct qt_t *
qt_vargs (struct qt_t *qsp, int nbytes, void *vargs,
	  void *pt, qt_function_t *startup,
	  qt_function_t *vuserf, qt_function_t *cleanup)
{
  va_list ap;
  int i;
  int n;		/* Number of words into original arg list. */
  qt_word_t *sp;
  int *reg;		/* Where to read passed-in-reg args. */
  int *stk;		/* Where to read passed-on-stk args. */

  ap = *(va_list *)vargs;
  qsp = QUICKTHREADS_VARGS_MD0 (qsp, nbytes);
  sp = (qt_word_t *)qsp;

  reg = (ap.__va_arg < 8)
    ? &ap.__va_reg[ap.__va_arg]
    : 0;
  stk = &ap.__va_stk[8];
  n = ap.__va_arg;
  for (i=0; i<nbytes/sizeof(qt_word_t) && n<8; ++i,++n) {
    sp[i] = *reg++;
  }
  for (; i<nbytes/sizeof(qt_word_t); ++i) {
    sp[i] = *stk++;
  }

#ifdef QUICKTHREADS_NDEF
  for (i=0; i<nbytes/sizeof(qt_word_t); ++i) {
    sp[i] = (n < 8)
      ? *reg++
      : *stk++;
    ++n;
  }
#endif

  QUICKTHREADS_VARGS_MD1 (QUICKTHREADS_VADJ(sp));
  QUICKTHREADS_SPUT (QUICKTHREADS_VADJ(sp), QUICKTHREADS_VARGT_INDEX, pt);
  QUICKTHREADS_SPUT (QUICKTHREADS_VADJ(sp), QUICKTHREADS_VSTARTUP_INDEX, startup);
  QUICKTHREADS_SPUT (QUICKTHREADS_VADJ(sp), QUICKTHREADS_VUSERF_INDEX, vuserf);
  QUICKTHREADS_SPUT (QUICKTHREADS_VADJ(sp), QUICKTHREADS_VCLEANUP_INDEX, cleanup);
  return ((qt_t *)QUICKTHREADS_VADJ(sp));
}
