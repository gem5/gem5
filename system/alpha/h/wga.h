#ifndef _WGA_H_LOADED
#define _WGA_H_LOADED
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
 *  $Id: wga.h,v 1.1.1.1 1997/10/30 23:27:18 verghese Exp $;
 */

/*
 * $Log: wga.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:18  verghese
 * current 10/29/97
 *
 * Revision 1.3  1994/08/05  20:13:47  fdh
 * Updated Copyright header and RCS $Id: identifier.
 *
 * Revision 1.2  1994/06/20  14:18:59  fdh
 * Fixup header file preprocessor #include conditionals.
 *
 * Revision 1.1  1993/06/08  19:56:17  fdh
 * Initial revision
 *
 */



/* ALU to MERGE translation table */
extern unsigned char mergexlate[];

/* Color Palette Registers */
#define PALMASK		0x02EA
#define PALREAD_ADDR	0x02EB
#define PALWRITE_ADDR	0x02EC
#define PALDATA		0x02ED

/* Video Timing Registers */
#define H_TOTAL		0x02E8 /* Horizontal Total */
#define H_DISP		0x06E8 /* Horizontal Displayed */
#define H_SYNC_START	0x0AE8 /* Horizontal Sync Start */
#define H_SYNC_WID	0x0EE8 /* Horizontal Sync Width and Polarity */
#define V_TOTAL		0x12E8 /* Vertical Total */
#define V_DISP		0x16E8 /* Vertical Displayed */
#define V_SYNC_START	0x1AE8 /* Vertical Sync Start */
#define V_SYNC_WID	0x1EE8 /* Vertical Sync Width and Polarity */


#define DISP_CNTL	0x22E8 /* Display Control */
#define SUBSYS_CNTL	0x42E8 /* Subsystem Control */
#define ADVFUNC_CNTL	0x4AE8 /* Advanced Function Control */
#define SUBSYS_STAT	0x42E8 /* Subsystem Status */
#define GP_STAT		0x9AE8 /* Graphics Processor Status */

/* this block gets the 0x4000 bit turned on */
#define CUR_Y		(0x82E8 | 0x0000)
#define CUR_X		(0x86E8 | 0x0000)
#define MULTIFUNC_CNTL	(0xBEE8 | 0x0000)
#define ERR_TERM	(0x92E8 | 0x0000)
#define DESTY_AXSTP	(0x8AE8 | 0x0000)
#define DESTX_DIASTP	(0x8EE8 | 0x0000)
#define MAJ_AXIS_PCNT	(0x96E8 | 0x0000)
#define FG_COLOR	(0xA6E8 | 0x0000)
#define BG_COLOR	(0xA2E8 | 0x0000)
#define WR_MASK		(0xAAE8 | 0x0000)
#define FG_MIX		(0xBAE8 | 0x0000)
#define BG_MIX		(0xB6E8 | 0x0000)
#define RD_MASK		(0xAEE8 | 0x0000)
#define COLOR_CMP	(0xB2E8 | 0x0000)
#define SHORT_STROKE	(0x9EE8 | 0x0000)
#define CMD		(0x9AE8 | 0x0000)


#define PIX_TRANS	0xE2E8

#define EXT_FIFO_STAT	0x9AEE

/************** Command Register (0x9AE8) bit definitions ***************/
/*
 * Bits 15-13 - Drawing Function Command
 */
#define CMD_NOOP	0x0000
#define CMD_LINE	0x2000
#define CMD_XRECT	0x4000
#define CMD_YRECT	0x6000
#define CMD_FRECT	0x8000
#define CMD_OUTLINE	0xA000
#define CMD_COPYRECT	0xC000
/*
 * Bit 12 - Word Byte Order
 */
#define CMD_LSBFIRST	0x1000
/*
 * Bit 9 - Bus Width
 */
#define CMD_WORDBUS	0x0200
#define CMD_BYTEBUS	0x0000
/*
 * Bit 8 - Variable Data Select
 */
#define CMD_VDATA	0x0100
#define CMD_FDATA	0x0000
/*
 * Bit 7 - Y Direction
 */
#define CMD_INCY	0x0080
/*
 * Bit 6 - Major Axis
 */
#define CMD_YMAJOR	0x0040
/*
 * Bit 5 - X Direction
 */
#define CMD_INCX	0x0020
/*
 * Bit 4 - Draw Enable
 */
#define CMD_DRAW	0x0010
/*
 * Bit 3 - Coded Direction / Short Stroke
 */
#define CMD_SHORTSTROKE	0x0008
/*
 * Bit 2 - Draw Last Pixel
 */
#define CMD_NOLASTPIXEL	0x0004
/*
 * Bit 1 - Plane Mode
 */
#define CMD_ACROSS	0x0002
#define CMD_THROUGH	0x0000
/*
 * Bit 0 - Read/Write
 */
#define CMD_WRITE	0x0001
#define CMD_READ	0x0000

#endif /* _WGA_H_LOADED */
