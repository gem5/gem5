#ifndef	__SERVERTYPES_H_LOADED
#define	__SERVERTYPES_H_LOADED
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
 *  $Id: server_t.h,v 1.1.1.1 1997/10/30 23:27:17 verghese Exp $;
 */
/*
 * This file contains named types used by the server.
 * $Log: server_t.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:17  verghese
 * current 10/29/97
 *
 * Revision 1.5  1994/11/08  21:44:17  fdh
 * Include lib.h
 *
 * Revision 1.4  1994/08/05  20:13:47  fdh
 * Updated Copyright header and RCS $Id: identifier.
 *
 * Revision 1.3  1994/06/20  14:18:59  fdh
 * Fixup header file preprocessor #include conditionals.
 *
 * Revision 1.2  1994/06/03  20:13:54  fdh
 * Properly handle this header file when included multiple times.
 *
 * Revision 1.1  1994/03/09  12:48:33  berent
 * Initial revision
 *
 */

#include "lib.h"

typedef ul address_value;
typedef ul register_value;
typedef ui instruction_value;

#endif /* __SERVERTYPES_H_LOADED */
