#ifndef __KERNEL_EXTRA_H_LOADED
#define __KERNEL_EXTRA_H_LOADED
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
 *  $Id: k_extra.h,v 1.1.1.1 1997/10/30 23:27:16 verghese Exp $;
 */

/*
 * $Log: k_extra.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:16  verghese
 * current 10/29/97
 *
 * Revision 1.4  1994/08/05  20:13:47  fdh
 * Updated Copyright header and RCS $Id: identifier.
 *
 * Revision 1.3  1994/06/28  20:08:21  fdh
 * Modified filenames and build precedure to fit into a FAT filesystem.
 *
 * Revision 1.2  1994/06/20  14:18:59  fdh
 * Fixup header file preprocessor #include conditionals.
 *
 * Revision 1.1  1994/03/09  12:48:33  berent
 * Initial revision
 *
 *
 * Derived from old kernel.h by splitting out the extra functions that are not part of
 * the target independent debug kernel interface.
 *
 * Log of kernel.h,v:
 * Revision 1.3  1993/10/03  00:19:49  berent
 * Merge in development by Anthony Berent
 *
 *>> Revision 1.4  1993/10/01  15:47:00  berent
 *>> Added saved_user_pc; used to avoid need for special ethernet PAL code
 *>>
 *>> Revision 1.3  1993/08/09  11:43:38  berent
 *>> Correct return types of some functions
 *
 * Revision 1.2  1993/06/08  22:32:06  berent
 * Changed to improve ladbx server communications
 *
 * Revision 1.1  1993/06/08  19:56:36  fdh
 * Initial revision
 *
 */

#include "system.h"
#include "server_t.h"

void kinitpalentry(void);

void kstart(void);

int kwait(void);

void kinstall_breakpoints(void);

void kprint_breakpoints(void);

void kenableserver(void);

extern address_value saved_user_pc;

#endif /* KERNEL_EXTRA_H_LOADED */
