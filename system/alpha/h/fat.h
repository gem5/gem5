#ifndef __FAT_H_LOADED
#define __FAT_H_LOADED
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
 *  $Id: fat.h,v 1.1.1.1 1997/10/30 23:27:15 verghese Exp $;
 */

/*
 * $Log: fat.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:15  verghese
 * current 10/29/97
 *
 * Revision 1.4  1994/08/05  20:13:47  fdh
 * Updated Copyright header and RCS $Id: identifier.
 *
 * Revision 1.3  1994/06/20  14:18:59  fdh
 * Fixup header file preprocessor #include conditionals.
 *
 * Revision 1.2  1994/06/17  19:34:01  fdh
 * Clean-up...
 *
 * Revision 1.1  1993/06/08  19:56:13  fdh
 * Initial revision
 *
 */



#define BOOT_SECTOR 0
#define FAT 12
#define FILES 20
#define DATA 100
#define BINTYPE 0
#define FATTYPE 1
#define DIRTYPE 2
#define EXERTYPE 3
struct _fat {
  ub jmp[3];
  ub oem[8];
  ub bps[2];
  ub spc[1];
  ub rs[2];
  ub numfat[1];
  ub numdir[2];
  ub totsec[2];
  ub medsc[1];
  ub spf[2];
  ub spt[2];
  ub nhead[2];
};

struct _file {
  ub fname[8];
  ub ext[3];
  ub attrib[1];
  ub reserv[10];
  ub time[2];
  ub date[2];
  ub cluster[2];
  ub fsize[4];
};

#endif /* __FAT_H_LOADED */
