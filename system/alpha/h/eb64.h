#ifndef __EB64_H_LOADED
#define __EB64_H_LOADED
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
 *  $Id: eb64.h,v 1.1.1.1 1997/10/30 23:27:14 verghese Exp $;
 */

/*
 * $Log: eb64.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:14  verghese
 * current 10/29/97
 *
 * Revision 1.19  1995/03/05  12:06:43  fdh
 * Adjusted ROMINC to access ROM bytes at
 * longword boundaries as required.
 *
 * Revision 1.18  1995/03/05  04:18:20  fdh
 * Changed ROMBASE and ROMINC definitions to use the
 * I/O bus addresses by using inrom().
 *
 * Revision 1.17  1995/02/10  02:56:08  fdh
 * Disabled bootopt command because the current SROM doesn't support it.
 *
 * Revision 1.16  1994/11/28  18:26:46  fdh
 * Added definition to enable the special ROM access
 * required for the EB64.
 *
 * Revision 1.15  1994/11/08  21:36:10  fdh
 * Added ROM definitions.  Moved from rom.c
 *
 * Revision 1.14  1994/08/05  20:13:47  fdh
 * Updated Copyright header and RCS $Id: identifier.
 *
 * Revision 1.13  1994/07/21  18:09:32  fdh
 * Added MAXIMUM_SYSTEM_CACHE definition.
 *
 * Revision 1.12  1994/06/20  14:18:59  fdh
 * Fixup header file preprocessor #include conditionals.
 *
 * Revision 1.11  1994/04/06  05:09:11  fdh
 * Removed an ethernet driver.
 *
 * Revision 1.10  1994/04/04  15:15:44  fdh
 * Added definition for LEDPORT.
 *
 * Revision 1.9  1994/04/01  13:57:22  fdh
 * Added MINIMUM_SYSTEM_MEMORY definition and
 * removed obsoleted PAL_IMPURE definition.
 *
 * Revision 1.8  1994/03/24  21:42:14  fdh
 * Removed unnecessary compile-time conditionals.
 *
 * Revision 1.7  1994/02/14  16:25:01  rusling
 * Allow remote debug support in the NT build case.
 *
 * Revision 1.6  1994/01/19  10:22:28  rusling
 * Ported to ALpha Windows NT.
 *
 * Revision 1.5  1993/11/22  15:24:30  rusling
 * Fixed up am79c960 embedded definitions.
 *
 * Revision 1.4  1993/11/22  14:20:41  rusling
 * Modified am79c960 definitions.
 *
 * Revision 1.3  1993/11/22  13:17:13  rusling
 * Merged with PCI/21040 changes.
 *
 * Revision 1.2  1993/11/22  12:16:20  rusling
 * Added in further definitions
 *
 * Revision 1.1  1993/11/22  11:42:50  rusling
 * Initial revision
 *
 */

#ifdef EB64

#define BANNER "DECchip 21064 Evaluation Board (EB64) Debug Monitor"
#define PROMPT "EB64> "

/****************************************************************************
 * Basic                                                                    *
 ****************************************************************************/

#define NEEDSCTL
#define NEEDDEBUGGER
#define NEEDAM79C960
#define NEEDFLOPPY
#define NEEDEB64SPECIALROMACCESS
#define DISABLEBOOTOPTION   /* Current SROM does not support this */

/****************************************************************************
 * Plug in cards, what does it have?                                        *
 ****************************************************************************/
/*
 * The Allied Telesis card is an ISA card based on the am79c960
 * chip.
 */
#define ALLIED_TELESIS_AT_1500T

/*
 *  The DEC Etherworks ISA card is a LANCE like device.
 */
#define DEC_ETHERWORKS

/****************************************************************************
 * ISA Address Space                                                        *
 ****************************************************************************/

#define LEDPORT                             0x80  /* JDR Microdevices P.O.S.T. Diagnostics Card. */

/*
 *  am79c960 definitions, see /h/am79c960.h for more definitions
 *  and /ether/am79c960_driver.c for the device driver.
 */
#ifdef NEEDAM79C960
#define EMBEDDED_AM79C960
#define EMBEDDED_AM79C960_BASE              0x360
#define EMBEDDED_AM79C960_INT 	            9
#define EMBEDDED_AM79C960_DMA               1
#endif

/*
 * Allied telesis is optional.  We treat it like an optional
 * am79c960 as far as the driver is concerned, see /h/am79c960.h
 * for more definitions.
 */
#ifdef ALLIED_TELESIS_AT_1500T
#define OPTIONAL_AM79C960		/* warning only define this once */
#define OPTIONAL_AM79C960_BASE              0x340
#define OPTIONAL_AM79C960_INT               12
#define OPTIONAL_AM79C960_DMA               2
#endif

#ifdef DEC_ETHERWORKS
#define DEC_ETHERWORKS_BASE                 0x300
#endif

/****************************************************************************
 * System Address Space                                                     *
 ****************************************************************************/

#define MINIMUM_SYSTEM_MEMORY    0x400000
#define MAXIMUM_SYSTEM_CACHE     0x80000
#define DMA_BUFF_BASE            0x100000

#define ROMBASE 0x0
#define ROMSIZE 0x80000
#define ROMINC 0x4

#endif /* EB64 */
#endif /* __EB64_H_LOADED */



