#ifndef __PALDEFS_H_LOADED
#define __PALDEFS_H_LOADED
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
 *  $Id: paldefs.h,v 1.1.1.1 1997/10/30 23:27:16 verghese Exp $;
 */

/*
 * $Log: paldefs.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:16  verghese
 * current 10/29/97
 *
 * Revision 1.1  1995/02/24  15:54:10  fdh
 * Initial revision
 *
 */

#define PAL_halt        0x0000
#define PAL_cflush      0x0001
#define PAL_draina      0x0002
#define PAL_cobratt     0x0009
#define PAL_ipir        0x000d
#define PAL_mtpr_mces   0x0011
#define PAL_wrfen       0x002b
#define PAL_wrvptptr    0x002d
#define PAL_jtopal      0x002e
#define PAL_swpctx      0x0030
#define PAL_wrval       0x0031
#define PAL_rdval       0x0032
#define PAL_tbi         0x0033
#define PAL_wrent       0x0034
#define PAL_swpipl      0x0035
#define PAL_rdps        0x0036
#define PAL_wrkgp       0x0037
#define PAL_wrusp       0x0038
#define PAL_wrperfmon   0x0039
#define PAL_rdusp       0x003a
#define PAL_whami       0x003c
#define PAL_rtsys       0x003d
#define PAL_rti         0x003f
#define PAL_bpt         0x0080
#define PAL_bugchk      0x0081
#define PAL_chmk        0x0083
#define PAL_callsys     0x0083
#define PAL_imb         0x0086
#define PAL_rduniq      0x009e
#define PAL_wruniq      0x009f
#define PAL_gentrap     0x00aa
#define PAL_nphalt      0x00be

#endif /* __PALDEFS_H_LOADED */
