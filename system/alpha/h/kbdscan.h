#ifndef __KBDSCAN_H_LOADED
#define __KBDSCAN_H_LOADED
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
 *  $Id: kbdscan.h,v 1.1.1.1 1997/10/30 23:27:16 verghese Exp $;
 */

/*
 * $Log: kbdscan.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:16  verghese
 * current 10/29/97
 *
 * Revision 1.3  1994/08/05  20:13:47  fdh
 * Updated Copyright header and RCS $Id: identifier.
 *
 * Revision 1.2  1994/06/20  14:18:59  fdh
 * Fixup header file preprocessor #include conditionals.
 *
 * Revision 1.1  1993/06/08  19:56:14  fdh
 * Initial revision
 *
 */




/* Some Key Defs */
/* Function keys */
#define	F1		(0x80|0x00)
#define	F2		(0x80|0x01)
#define	F3		(0x80|0x02)
#define	F4		(0x80|0x03)
#define	F5		(0x80|0x04)
#define	F6		(0x80|0x05)
#define	F7		(0x80|0x06)
#define	F8		(0x80|0x07)
#define	F9		(0x80|0x08)
#define	F10		(0x80|0x09)
#define	F11		(0x80|0x0a)
#define	F12		(0x80|0x0b)

/* Num lock affects these */
#define	END		(0x80|0x0c)
#define	DOWN		(0x80|0x0d)
#define	PGDWN		(0x80|0x0e)
#define	LEFT		(0x80|0x0f)
#define	MIDDLE		(0x80|0x10)
#define	RIGHT		(0x80|0x11)
#define	HOME		(0x80|0x12)
#define	UP		(0x80|0x13)
#define	PGUP		(0x80|0x14)
#define	INS		(0x80|0x15)
#define	DEL		(0x80|0x16)

/* Others */
#define	PRTSC		(0x80|0x17)
#define	PAUSE		(0x80|0x18)
#define	ALTDWN		(0x80|0x19)
#define	ALTUP		(0x80|0x1a)

#endif /* __KBDSCAN_H_LOADED */
