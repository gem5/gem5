#ifndef __LOCAL_H_LOADED
#define __LOCAL_H_LOADED
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
 *  $Id: local.h,v 1.1.1.1 1997/10/30 23:27:16 verghese Exp $;
 */

/*
 * $Log: local.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:16  verghese
 * current 10/29/97
 *
 * Revision 1.1  1995/02/10  02:23:49  fdh
 * Initial revision
 *
 */

/*
 * Source files that are common between the ebfw source
 * tree and the ebtools source tree could have inherent
 * conflicts in their requirements for libraries, definitions,
 * etc.  The basic difference is that ebtools are built
 * to run natively with an operating system where the ebfw
 * tree is built to run in a freestanding environment on,
 * typically, an evaluation board target.  Therefore, this
 * file is used to provide the proper environment for building
 * those common files in the ebfw source tree.
 */

#include "lib.h"
#include "system.h"

#endif /* __LOCAL_H_LOADED */
