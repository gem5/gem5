#ifndef __EB64P_H_LOADED
#define __EB64P_H_LOADED
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
 *  $Id: eb64p.h,v 1.1.1.1 1997/10/30 23:27:14 verghese Exp $;
 */

/*
 * $Log: eb64p.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:14  verghese
 * current 10/29/97
 *
 * Revision 1.24  1995/02/10  02:56:08  fdh
 * Disabled bootopt command because the current SROM doesn't support it.
 *
 * Revision 1.23  1994/12/08  16:42:29  fdh
 * Removed Cabriolet (EB64L) definitions. Moved to eb64l.h
 *
 * Revision 1.22  1994/11/17  14:15:05  fdh
 * Modified definitions for EB64LC
 *
 * Revision 1.21  1994/11/08  21:36:39  fdh
 * Added ROM definitions. Moved from rom.c
 * Also added Flash ROM definitions to support
 * a variant of the EB64+ design.
 *
 * Revision 1.20  1994/08/05  20:13:47  fdh
 * Updated Copyright header and RCS $Id: identifier.
 *
 * Revision 1.19  1994/07/21  18:09:32  fdh
 * Added MAXIMUM_SYSTEM_CACHE definition.
 *
 * Revision 1.18  1994/06/28  20:08:21  fdh
 * Modified filenames and build precedure to fit into a FAT filesystem.
 *
 * Revision 1.17  1994/06/20  14:18:59  fdh
 * Fixup header file preprocessor #include conditionals.
 *
 * Revision 1.16  1994/04/06  05:09:55  fdh
 * Removed and ethernet driver.
 *
 * Revision 1.15  1994/04/04  15:16:18  fdh
 * Added definition for LEDPORT.
 *
 * Revision 1.14  1994/04/01  13:58:13  fdh
 * Added MINIMUM_SYSTEM_MEMORY definition and
 * removed obsoleted PAL_IMPURE definition.
 * Modified BANNER definition
 *
 * Revision 1.13  1994/03/24  21:42:37  fdh
 * Removed unnecessary compile-time conditionals.
 *
 * Revision 1.12  1994/03/16  14:28:28  rusling
 * Let WNT versions have the debug code in them.
 *
 * Revision 1.11  1994/03/05  22:22:20  fdh
 * Added Apecs pass2.0 support. Use -DAPECS_PASS1to support pass 1.0.
 *
 * Revision 1.10  1994/01/27  20:36:23  fdh
 * Modified the Amd79c960 definitions to accomodate Ladebug.
 *
 * Revision 1.9  1994/01/27  18:12:14  fdh
 * Modify PCI Target window definitions for a
 * 1 megabyte window at 1 megabyte mapping into 1 megabyte.
 *
 * Revision 1.8  1994/01/19  10:22:28  rusling
 * Ported to ALpha Windows NT.
 *
 * Revision 1.7  1993/11/30  15:29:22  rusling
 * Mostly complete versions.  I've integrated changes to
 * the PCI to system address space mappings with all the
 * definitions in the system specific include file.
 *
 * Revision 1.6  1993/11/23  10:43:50  rusling
 * Added in APECS information (taking it from pci.h).
 *
 * Revision 1.5  1993/11/22  16:30:23  rusling
 * Fixed up (#ifdef EB64p -> EB64P).
 *
 * Revision 1.4  1993/11/22  14:23:03  rusling
 * Modified the am8=79c960 definitions.
 *
 * Revision 1.3  1993/11/22  13:17:55  rusling
 * *** empty log message ***
 *
 * Revision 1.2  1993/11/22  12:16:20  rusling
 * Added in further definitions
 *
 * Revision 1.2  1993/11/22  12:16:20  rusling
 * Added in further definitions
 *
 * Revision 1.1  1993/11/22  11:43:11  rusling
 * Initial revision
 *
 */

#ifdef EB64P

#define BANNER "DECchip 21064 PCI Evaluation Board (EB64+) Debug Monitor"
#define PROMPT "EB64+ "

/****************************************************************************
 * Basic                                                                    *
 ****************************************************************************/

#define NEEDPCI
#define NEEDDEBUGGER
#define NEEDFLOPPY
#define DISABLEBOOTOPTION   /* Current SROM does not support this */

/****************************************************************************
 * EPIC (Part of APECS) definitions                                         *
 ****************************************************************************/
#define EPIC_CSR                       0x000
#define EPIC_PCI_TBASE_1               0x0C0
#define EPIC_PCI_TBASE_2               0x0E0
#define EPIC_PCI_BASE_1                0x100
#define EPIC_PCI_BASE_2                0x120
#define EPIC_PCI_MASK_1                0x140
#define EPIC_PCI_MASK_2                0x160
#define EPIC_HAXR0                     0x180
#define EPIC_HAXR1                     0x1A0
#define EPIC_HAXR2                     0x1C0

#define EPIC_CSR_M_NDEV                0x800

#ifdef APECS_PASS1
#define EPIC_CSR_M_RWC                 0xFFE0
#else
#define EPIC_CSR_M_RWC                 0xFF60
#endif

#define EPIC_PCI_BASE_M_WENB           0x80000

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

#define NEED21040

/****************************************************************************
 * ISA Address Space                                                        *
 ****************************************************************************/

#define RTCBASE                             0x70
#define LEDPORT                             0x80  /* JDR Microdevices P.O.S.T. Diagnostics Card. */


/*
 * Allied telesis is optional.  We treat it like an optional
 * am79c960 as far as the driver is concerned, see /h/am79c960.h
 * for more definitions and /ether/am79c960.c for the device
 * driver.
 */
#ifdef ALLIED_TELESIS_AT_1500T
#define OPTIONAL_AM79C960		/* warning only define this once */
#define NEEDAM79C960			/* same for this */
#define OPTIONAL_AM79C960_BASE              0x360
#define OPTIONAL_AM79C960_INT               9

/*
 * This parameter determines the
 * DMA Mask for the DMA2 controller.
 *       Mask     Channel
 *       ----     -------
 *        1          5
 *        2          6
 *        3          7
 *        4          8
 */
#define OPTIONAL_AM79C960_DMA               1 /* Selects DMA Channel 5 */
#endif

#ifdef DEC_ETHERWORKS
#define DEC_ETHERWORKS_BASE                 0x300
#endif

#ifdef DEC_ETHERWORKS
#define DEC_ETHERWORKS_BASE                 0x300
#endif


/****************************************************************************
 * System Address Space                                                     *
 ****************************************************************************/

#define MINIMUM_SYSTEM_MEMORY    0x1000000
#define MAXIMUM_SYSTEM_CACHE     0x200000
#define DMA_BUFF_BASE            0x100000


/****************************************************************************
 * PCI I/O Address Space                                                    *
 ****************************************************************************/
/*
 * Definitions for the windows mapping PCI addresses into
 * system addresses
 *
 * 1 megabyte window at 1 megabyte mapping into 1 megabyte.
 */
#define PCI_BASE_1_USED                     1
#define PCI_BASE_1                          0x000100000
#define PCI_MASK_1                          0x000000000
#define PCI_TBASE_1                         0x000100000

#define PCI_BASE_2_USED                     0
#define PCI_BASE_2                          0x010000000
#define PCI_MASK_2                          0x00FF00000
#define PCI_TBASE_2                         0x000000000

/*
 * Each mask translates to a number of these units.  For
 * APECS this unit is 1Kbyte.
 */
#define PCI_MASK_UNIT                       0x100000

/*
 * Where do we start allocating addresses from in PCI I/O space?
 */
#define PCI_IO_BASE                         0xB000

/*
 * ROM definitions.
 */
#define ROMBASE 0xFFF80000
#define ROMSIZE 0x80000
#define ROMINC 0x1

#endif /* EB64P */
#endif /* __EB64P_H_LOADED */
