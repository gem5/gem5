#ifndef __EB66P_H_LOADED
#define __EB66P_H_LOADED
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
 *  $Id: eb66p.h,v 1.1.1.1 1997/10/30 23:27:15 verghese Exp $;
 */

/*
 * $Log: eb66p.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:15  verghese
 * current 10/29/97
 *
 * Revision 1.2  1995/02/10  02:22:26  fdh
 * corrected EB66+ banner.
 *
 * Revision 1.1  1994/12/08  16:42:29  fdh
 * Initial revision
 *
 */

#ifdef EB66P
/*
 * Declare special definitions for systems with only
 * minor variations from the standard system described
 * by this file.  To build for a variant board, declare
 * the variant definition in addition to the standard
 * definition.  example... -DEB66 -DEB66P
 */
#include "eb66.h"

#undef BANNER
#define BANNER "Alpha 21066A Evaluation Board (EB66+) Debug Monitor"
#undef PROMPT
#define PROMPT "EB66+ "
#undef ROMSIZE
#define ROMSIZE 0x100000
#define NEEDFLASHMEMORY
#define INTEL_28F008SA

#endif /* EB66P */
#endif /* __EB66P_H_LOADED */
