#ifndef __ROM_H_LOADED
#define __ROM_H_LOADED
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
 *  $Id: rom.h,v 1.1.1.1 1997/10/30 23:27:17 verghese Exp $;
 */

/*
 * $Log: rom.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:17  verghese
 * current 10/29/97
 *
 * Revision 1.1  1995/02/28  03:06:09  fdh
 * Initial revision
 *
 */

#include "romhead.h"  /* Includes required typedefs. */

/*::::::::::::::
rom.c
::::::::::::::*/
extern int read_rom(int argc , char *arg, ul address);
extern void list_rom_headers(void );
extern void set_romboot(char * arg);
extern ui romimage_size(romheader_t * rhead);

#endif /* __ROM_H_LOADED */
