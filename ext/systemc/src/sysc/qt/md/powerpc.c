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

// This static is used to find the end of the stack for variable

static void *qt_sp_bottom_save;

/* We actually don't know how the compiler accomodates arguments in the 
 * va_list. In some implementation (e.g. Linux PPC) we cannot scan the
 * list as an array. To avoid this problem, this version of "qt_varg",
 * retrieves arguments by means of the standard "va_arg" macro defined
 * in stdargs.h.
 *
 * Notice that we still suppose that the number of arguments is given
 * by nbytes/sizeof(qt_word_t) and we load the stack of "qt_vstart"
 * assuming that all parameters are alligned to the size of qt_word_t.
 *
 * Marco Bucci <marco.bucci@inwind.it>
 * December 2002
 */

/*

qt_t *qt_vargs (qt_t *sp, int nbytes, void *vargs,
	  void *pt, qt_startup_t *startup,
	  qt_vuserf_t *vuserf, qt_cleanup_t *cleanup)
	  
*/

  qt_t *
qt_vargs_stdarg (qt_t *sp, int nbytes, va_list vargs,
	  void *pt, qt_startup_t *startup,
	  qt_vuserf_t *vuserf, qt_cleanup_t *cleanup)
	  
	  
	
{
	int i;
	qt_word_t arg;

	sp = QUICKTHREADS_VARGS_MD0 (sp, nbytes);

	for ( i=0;i<(int)(nbytes/sizeof(qt_word_t)); i++)
    {
    	arg = va_arg(vargs, qt_word_t);
      	QUICKTHREADS_SPUT (QUICKTHREADS_VARGS_ADJUST(sp), i, arg);
    }

	QUICKTHREADS_VARGS_MD1 (QUICKTHREADS_VADJ(sp));
	QUICKTHREADS_SPUT (QUICKTHREADS_VADJ(sp), QUICKTHREADS_VARGT_INDEX, pt);
	QUICKTHREADS_SPUT (QUICKTHREADS_VADJ(sp), QUICKTHREADS_VSTARTUP_INDEX, startup);
	QUICKTHREADS_SPUT (QUICKTHREADS_VADJ(sp), QUICKTHREADS_VUSERF_INDEX, vuserf);
	QUICKTHREADS_SPUT (QUICKTHREADS_VADJ(sp), QUICKTHREADS_VCLEANUP_INDEX, cleanup);
	return ((qt_t *)QUICKTHREADS_VADJ(sp));
}
