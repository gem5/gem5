#ifndef __BUFFER_H_LOADED
#define __BUFFER_H_LOADED
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
 *  $Id: buffer.h,v 1.1.1.1 1997/10/30 23:27:14 verghese Exp $;
 */

/*
 * $Log: buffer.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:14  verghese
 * current 10/29/97
 *
 * Revision 1.3  1994/08/05  20:13:47  fdh
 * Updated Copyright header and RCS $Id: identifier.
 *
 * Revision 1.2  1994/06/20  14:18:59  fdh
 * Fixup header file preprocessor #include conditionals.
 *
 * Revision 1.1  1993/06/08  19:56:12  fdh
 * Initial revision
 *
 */


/* buffer.h */
/* definitions for buffer management */


typedef struct Packet {
    struct Packet *link;
    char dest[6];
    char src[6];
    short unsigned count;
    char data[1536];
  } PACKET;

#define BSIZE (sizeof(PACKET))
#define buffer_NIL ((PACKET *) 0)


extern void buffer_init(PACKET *start, int size);
extern PACKET *buffer_alloc();
extern void buffer_free(PACKET *p);

#endif /* __BUFFER_H_LOADED */
