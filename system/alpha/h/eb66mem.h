#ifndef __EB66MEM_H_LOADED
#define __EB66MEM_H_LOADED
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
 *  $Id: eb66mem.h,v 1.1.1.1 1997/10/30 23:27:15 verghese Exp $;
 */

/*
 * $Log: eb66mem.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:15  verghese
 * current 10/29/97
 *
 * Revision 1.2  1994/08/05  20:13:47  fdh
 * Updated Copyright header and RCS $Id: identifier.
 *
 * Revision 1.1  1994/07/21  18:11:01  fdh
 * Initial revision
 *
 */

#ifdef EB66

#include "system.h"
#include "prtrace.h"
#include "lib.h"

/*
 *  Definition for the Cache Register (CAR):
 */
#define CAR_OFFSET 		0x78
#define CAR_M_BCE		0x1

/*
 *  Macros used to access the memory controller
 *  csr's.
 */
#define MEM_CSR(x)        (((ul)0x12<<28)|(ul)(x))
#define _IN_MEM_CSR(p)    (ReadQ(MEM_CSR(p)))
#define _OUT_MEM_CSR(p,d) WriteQ((MEM_CSR(p)),d);mb();

#define BANKSIZE_MAX 128
#define BANKSIZE_MIN 8
#define _max(a,b) (a>b ? a : b)
#define _min(a,b) (a>b ? b : a)
#define NONCACHEABLE (0x2<<28)
#define PATTERN (ui) 0x5A5A5A5A
#define MEM_RW1C 0x1686

#endif /* EB66 */
#endif /* __EB66MEM_H_LOADED */
