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


/* Varargs is harder on the AXP.  Parameters are saved on the stack as
   something like (stack grows down to low memory; low at bottom of
   picture):

	|  :
	| arg6
	+---
	| iarg5
	|  :
	| iarg3		<-- va_list._a0 + va_list._offset
	|  :
	| iarg0		<-- va_list._a0
	+---
	| farg5
	|  :
	| farg0
	+---

   When some of the arguments have known type, there is no need to
   save all of them in the struct.  So, for example, if the routine is
   called

	zork (int a0, float a1, int a2, ...)
	{
	  va_list ap;
	  va_start (ap, a2);
	  qt_vargs (... &ap ...);
	}

   then offset is set to 3 * 8 (8 === sizeof machine word) = 24.

   What this means for us is that the user's routine needs to be
   called with an arg list where some of the words in the `any type'
   parameter list have to be split and moved up in to the int/fp
   region.

   Ways in which this can fail:
    - The user might not know the size of the pushed arguments anyway.
    - Structures have funny promotion rules.
    - Probably lots of other things.

   All in all, we never promised varargs would work reliably. */



#define QUICKTHREADS_VADJ(sp)	(((char *)sp) - QUICKTHREADS_VSTKBASE)

#define QUICKTHREADS_VARGS_MD0(sp, vabytes) \
   ((qt_t *)(((char *)(sp)) - 6*2*8 - QUICKTHREADS_STKROUNDUP(vabytes)))

extern void qt_vstart(void);
#define QUICKTHREADS_VARGS_MD1(sp)	(QUICKTHREADS_SPUT (sp, QUICKTHREADS_R26, qt_vstart))


/* Different machines use different implementations for varargs.
   Unfortunately, the code below ``looks in to'' the varargs
   structure, `va_list', and thus depends on the conventions.
   The following #defines try to deal with it but don't catch
   everything. */

#ifdef __GNUC__
#define _a0		__base
#define _offset		__offset
#else
#ifdef __OSF1__
#define _a0		a0
#define _offset		offset
#endif
#endif /* def __GNUC__ */


  struct qt_t *
qt_vargs (struct qt_t *qsp, int nbytes, struct va_list *vargs,
	  void *pt, qt_function_t *startup,
	  qt_function_t *vuserf, qt_function_t *cleanup)
{
  va_list ap;
  int i;
  int max;		/* Maximum *words* of args to copy. */
  int tmove;		/* *Words* of args moved typed->typed. */
  qt_word_t *sp;

  ap = *(va_list *)vargs;
  qsp = QUICKTHREADS_VARGS_MD0 (qsp, nbytes);
  sp = (qt_word_t *)qsp;

  tmove = 6 - ap._offset/sizeof(qt_word_t);

  /* Copy from one typed area to the other. */
  for (i=0; i<tmove; ++i) {
    /* Integer args: */
    sp[i+6] = ((qt_word_t *)(ap._a0 + ap._offset))[i];
    /* Fp args: */
    sp[i] = ((qt_word_t *)(ap._a0 + ap._offset))[i-6];
  }

  max = nbytes/sizeof(qt_word_t);

  /* Copy from the untyped area to the typed area.  Split each arg.
     in to integer and floating-point save areas. */
  for (; i<6 && i<max; ++i) {
    sp[i] = sp[i+6] = ((qt_word_t *)(ap._a0 + ap._offset))[i];
  }

  /* Copy from the untyped area to the other untyped area. */
  for (; i<max; ++i) {
    sp[i+6] = ((qt_word_t *)(ap._a0 + ap._offset))[i];
  }

  QUICKTHREADS_VARGS_MD1 (QUICKTHREADS_VADJ(sp));
  QUICKTHREADS_SPUT (QUICKTHREADS_VADJ(sp), QUICKTHREADS_VARGT_INDEX, pt);
  QUICKTHREADS_SPUT (QUICKTHREADS_VADJ(sp), QUICKTHREADS_VSTARTUP_INDEX, startup);
  QUICKTHREADS_SPUT (QUICKTHREADS_VADJ(sp), QUICKTHREADS_VUSERF_INDEX, vuserf);
  QUICKTHREADS_SPUT (QUICKTHREADS_VADJ(sp), QUICKTHREADS_VCLEANUP_INDEX, cleanup);
  return ((qt_t *)QUICKTHREADS_VADJ(sp));
}
