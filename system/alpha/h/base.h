#ifndef __BASE_H_LOADED
#define __BASE_H_LOADED
/*****************************************************************************

       Copyright © 1993, 1994 Digital Equipment Corporation,
                       Maynard, Massachusetts.

                        All Rights Reserved

Permission to use, copy, modify, and distribute this software and its
documentation for any purpose and without fee is hereby granted, provided
that the copyright notice and this permission notice appear in all copies
of software and supporting documentation, and that the name of Digital not
be used in advertising or publicity pertaining to distribution of the software
without specific, written prior permission. Digital grants this permission
provided that you prominently mark, as not part of the original, any
modifications made to this software or documentation.

Digital Equipment Corporation disclaims all warranties and/or guarantees
with regard to this software, including all implied warranties of fitness for
a particular purpose and merchantability, and makes no representations
regarding the use of, or the results of the use of, the software and
documentation in terms of correctness, accuracy, reliability, currentness or
otherwise; and you rely on the software, documentation and results solely at
your own risk.

******************************************************************************/

/*
 *  $Id: base.h,v 1.1.1.1 1997/10/30 23:27:14 verghese Exp $;
 */

/*
 * $Log: base.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:14  verghese
 * current 10/29/97
 *
 * Revision 1.4  1994/08/05  20:13:47  fdh
 * Updated Copyright header and RCS $Id: identifier.
 *
 * Revision 1.3  1994/06/20  14:18:59  fdh
 * Fixup header file preprocessor #include conditionals.
 *
 * Revision 1.2  1994/06/17  19:34:01  fdh
 * Clean-up...
 *
 * Revision 1.1  1993/06/08  19:56:11  fdh
 * Initial revision
 *
 */

#include "lib.h"

/* priority and critical sections */

extern int base_ensurepriority(int priority);
extern void base_setpriority(int priority);

/* timing */
extern void base_msdelay(int ms);

/* second clock */
time_t base_seconds;
time_t base_time(int delta);
extern int base_jiffies;  /* raw interrupt count */

/* interrupts */

/* process control */
extern void base_yield();

/* general */

extern void base_kickthedog();   /* don't reboot for another XXX seconds */
extern int base_watchdog;  /* if 0, timer interrupt handles watchdog
        if non-zero, user code is responsible */

#endif /* __BASE_H_LOADED */
