#ifndef __UART_H_LOADED
#define __UART_H_LOADED
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
 *  $Id: uart.h,v 1.1.1.1 1997/10/30 23:27:18 verghese Exp $;
 */

/*
 * $Log: uart.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:18  verghese
 * current 10/29/97
 *
 * Revision 1.1  1995/06/07  04:33:20  berc
 * Initial revision
 *
 * Revision 1.3  1994/08/05  20:13:47  fdh
 * Updated Copyright header and RCS $Id: identifier.
 *
 * Revision 1.2  1994/06/20  14:18:59  fdh
 * Fixup header file preprocessor #include conditionals.
 *
 * Revision 1.1  1993/06/08  19:56:16  fdh
 * Initial revision
 *
 */



#include "kbdscan.h"

#define	COM1	1
#define	COM2	2
#define	LPT	3
#define	KBD	4
#define	VGA	4
#define	P8514	5
#define TGA	6

#ifdef XXM

#include <xxm.h>

#else /* XXM */

#define com1Rbr	0x3f8
#define com1Thr	0x3f8
#define com1Ier	0x3f9
#define com1Iir	0x3fa
#define com1Lcr	0x3fb
#define com1Mcr	0x3fc
#define com1Lsr	0x3fd
#define com1Msr	0x3fe
#define com1Scr	0x3ff
#define com1Dll	0x3f8
#define com1Dlm	0x3f9

#define com2Rbr	0x2f8
#define com2Thr	0x2f8
#define com2Ier	0x2f9
#define com2Iir	0x2fa
#define com2Lcr	0x2fb
#define com2Mcr	0x2fc
#define com2Lsr	0x2fd
#define com2Msr	0x2fe
#define com2Scr	0x2ff
#define com2Dll	0x2f8
#define com2Dlm	0x2f9

#define	lptDr	0x0
#define	lptSr	0x0
#define	lptCr	0x0

#define	lptSTB	0x01
#define	lptAFD	0x02
#define	lptnInit 0x04
#define	lptSlct	0x08
#define	lptIrq	0x10
#define	lptDir	0x20

#define	lptDly	100000
#endif /* XXM */

#endif /* __UART_H_LOADED */
