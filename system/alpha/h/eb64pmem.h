#ifndef __EB64PMEM_H_LOADED
#define __EB64PMEM_H_LOADED
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
 *  $Id: eb64pmem.h,v 1.1.1.1 1997/10/30 23:27:15 verghese Exp $;
 */

/*
 * $Log: eb64pmem.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:15  verghese
 * current 10/29/97
 *
 * Revision 1.4  1994/11/22  23:47:01  fdh
 * Don't include PALcode include files for make depend.
 *
 * Revision 1.3  1994/11/07  11:59:41  rusling
 *  Include system.h and lib.h in the correct order.
 *
 * Revision 1.2  1994/08/05  20:13:47  fdh
 * Updated Copyright header and RCS $Id: identifier.
 *
 * Revision 1.1  1994/07/21  18:11:01  fdh
 * Initial revision
 *
 */

#ifdef EB64P

#include "system.h"
#include "lib.h"

#ifndef MAKEDEPEND
#include "cserve.h"
#include "dc21064.h"
#endif

/*
 *  Definition for the General Control Register (GCR):
 */
#define GCR_OFFSET 		0x00
#define GCR_M_BCE		0x20

/*
 *  Macros used to access the memory controller
 *  csr's.
 */
#define MEM_CSR(x)        (((ul)0x18<<28)|(ul)(x))
#define _IN_MEM_CSR(p)    (ReadL(MEM_CSR(p)))
#define _OUT_MEM_CSR(p,d) WriteL((MEM_CSR(p)),d);mb();

#endif /* EB64P */
#endif /* __EB64PMEM_H_LOADED */
