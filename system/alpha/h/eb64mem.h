#ifndef __EB64MEM_H_LOADED
#define __EB64MEM_H_LOADED
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
 *  $Id: eb64mem.h,v 1.1.1.1 1997/10/30 23:27:14 verghese Exp $;
 */

/*
 * $Log: eb64mem.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:14  verghese
 * current 10/29/97
 *
 * Revision 1.4  1994/11/22  23:47:01  fdh
 * Don't include PALcode include files for make depend.
 *
 * Revision 1.3  1994/11/07  11:58:55  rusling
 * Include system.h and lib.h in the correct order.
 *
 * Revision 1.2  1994/08/05  20:13:47  fdh
 * Updated Copyright header and RCS $Id: identifier.
 *
 * Revision 1.1  1994/07/21  18:11:01  fdh
 * Initial revision
 *
 */

#ifdef EB64

#include "system.h"
#include "lib.h"

#ifndef MAKEDEPEND
#include "cserve.h"
#include "dc21064.h"
#endif

#define SCTL_M_BCE (1<<25)
#define BYTE_ENABLE_SHIFT 7
#define TRANSFER_LENGTH_SHIFT 5
#define IO	  (((ul)(-1)<<42)|((ul)1<<33))	/*    CPU Adr[33]=1 select I/O space.	*/
#define Lng       ((ui)3<<TRANSFER_LENGTH_SHIFT)
#define ROM(x) (IO|Lng|(((ul)(x)&0x7fffff)<<BYTE_ENABLE_SHIFT))

#define _IN_SCTL     ((ReadL(ROM(0)))&0xFFFFFF00)
#define _OUT_SCTL(d) WriteL(ROM(0),(d&0xFFFFFF00));mb();

#endif /* EB64 */
#endif /* __EB64MEM_H_LOADED */
