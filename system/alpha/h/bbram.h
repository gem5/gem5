#ifndef __BBRAM_H_LOADED
#define __BBRAM_H_LOADED
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
 *  $Id: bbram.h,v 1.1.1.1 1997/10/30 23:27:14 verghese Exp $;
 */

/*
 * $Log: bbram.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:14  verghese
 * current 10/29/97
 *
 * Revision 1.1  1995/06/23  16:13:56  berc
 * Initial revision
 *
 * Revision 1.6  1995/02/10  02:20:00  fdh
 * Added definitions for OSTYPE location in battary backed RAM.
 *
 * Revision 1.5  1994/08/05  20:13:47  fdh
 * Updated Copyright header and RCS $Id: identifier.
 *
 * Revision 1.4  1994/07/11  19:46:38  fdh
 * Remove typecast used on the value set by the BBRAM_READ macro.
 *
 * Revision 1.3  1994/06/22  15:10:20  rusling
 * Fixed up WNT compile warnings.
 *
 * Revision 1.2  1994/06/20  14:18:59  fdh
 * Fixup header file preprocessor #include conditionals.
 *
 * Revision 1.1  1994/03/16  00:13:48  fdh
 * Initial revision
 *
 */


/*
 * The TOY clock contains some BBRAM that we can use to
 * store useful stuff.  The BBRAM that is available to us
 * is in bytes 14-127.
 *
 * Currently, we use:
 *
 * Bytes	Contents
 *
 * 14:18	bootadr + pattern
 * 21:22	rmode + pattern
 * 23:30 	default mac address + 2 bytes of checksum.
 *
 * Below some macros and offsets are defined to make programming
 * this area easier.
 */

#define BBRAM_BASE	14

#define BBRAM_READ(offset,value)	\
        {				\
        outportb(RTCADDR, BBRAM_BASE + (offset));		\
        (value) = inportb(RTCDATA) & 0xFF;	\
        }

#define BBRAM_WRITE(offset,value)	\
        {				\
        outportb(RTCADDR, BBRAM_BASE + (offset));		\
        outportb(RTCDATA, (value) & 0xFF);			\
        }

#define BBRAM_BOOTADR		0
#define BBRAM_BOOTADR_COOKIE	0xA1
#define BBRAM_RMODE		7
#define BBRAM_RMODE_COOKIE	0x1B
#define BBRAM_MACADDR		9
#define BBRAM_MACADDR_COOKIE	0x1C
#define BBRAM_OSTYPE		(0x3f-BBRAM_BASE)

#endif /* __BBRAM_H_LOADED */
