#ifndef __XXM_H_LOADED
#define __XXM_H_LOADED
/*****************************************************************************

Copyright © 1993, Digital Equipment Corporation, Maynard, Massachusetts.

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
 *  $Id: xxm.h,v 1.1.1.1 1997/10/30 23:27:18 verghese Exp $;
 */

/*
 * $Log: xxm.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:18  verghese
 * current 10/29/97
 *
 * Revision 1.3  1996/04/15 16:28:36  berc
 * adding PCI ifdefs
 *
 * Revision 1.1  1995/06/07  03:27:16  berc
 * Initial revision
 *
 */

#ifdef XXM

#define KSEG ((ul)0xfffffc0000000000)
#define PHYS_TO_KSEG(x)(((ul)x) | KSEG)
#define KSEG_TO_PHYS(x)(((ul)x) & ~KSEG)

#define BANNER "SRC XXX Debug Monitor - Se Habla Peligro"
#define PROMPT "XXX> "

/****************************************************************************
 * Basic                                                                    *
 ****************************************************************************/

#define NEEDPCI
/*#define NEEDDEBUGGER*/
/*#define NO_ETHERNET*/
/*#define NEEDWAVELAN*/
#define NEEDDEPCM
#define NEED21040
/*#define TRACE_ENABLE*/

/****************************************************************************
 * System Address Space                                                     *
 ****************************************************************************/

#define MINIMUM_SYSTEM_MEMORY	0x4000000 /* 64MB */
#define DMA_BUFF_BASE           (KSEG | (ul)0x3000000)	/* Ether buffers */
#define AOUT_LOAD_ADDR		(KSEG | (ul)0x2000000)	/* Kernel image */

/****************************************************************************
 * PCI I/O Address Space                                                    *
 ****************************************************************************/

/*
 * Allocate PCI IO space above the 10-bit ISA space
 */
#define PCI_IO_BASE                         0x0400
/* 2 gig + 16mb for slot D + two 16 meg windows for the PCMCIA cards (if any) */
#define PCI_MEM_BASE (1 << 31) + (3 * (1 << 24))
/*
 * Where does the SROM leave PCMCIA slot D?
 */
#define PCMCIA_SLOT_D_BASE                         0x0

/*
 * ROM definitions.
 */
#ifdef undef
#define NEEDFLASHMEMORY
#define INTEL_28F008SA
#define ROMBASE 0xFFF80000
#define ROMSIZE 0x100000
#define ROMINC 0x1
#endif

/****************************************************************************
 * PCI I/O Address Space Access                                             *
 ****************************************************************************/

/* XXM ADDRESS BIT DEFINITIONS
 *
 *  PCI Sparse Memory Space (2GB)
 *
 *  3 3 3 3|3 3 3 3|3 3 2 2|2 2 2 2|2 2 2 2|1 1 1 1|1 1 1 1|1 1
 *  9 8 7 6|5 4 3 2|1 0 9 8|7 6 5 4|3 2 1 0|9 8 7 6|5 4 3 2|1 0 9 8|7 6 5 4|3 2 1 0
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |1|0|0| |A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|S|S|S|S|0| | |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *  | \_/  \________________________________________________________/ \_____/ |
 *  |  |                 Address (pA[31] ==1, pA[30..02])                |   MBZ
 *  |  +-Space identifier                                                |
 *  +-- IO space, not cached                           Transfer Length --+
 *
 *
 *  PCI Sparce Other Space (16MB)
 *
 *  3 3 3 3|3 3 3 3|3 3 2 2|2 2 2 2|2 2 2 2|1 1 1 1|1 1 1 1|1 1
 *  9 8 7 6|5 4 3 2|1 0 9 8|7 6 5 4|3 2 1 0|9 8 7 6|5 4 3 2|1 0 9 8|7 6 5 4|3 2 1 0
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |1|0|1| | | | | |T|T|T|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|S|S|S|S|0| | |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *  | \_/           \___/ \_________________________________________/ \_____/ |
 *  |  |              |       Address (pA[31..24] == 0 pA[23..02])       |   MBZ
 *  |  +-Space        +x00 - memory, x01 - I/O x10 - Type0, x11 - Type1  |
 *  +-- IO space, not cached                           Transfer Length --+
 *
 *
 *  PCI Dense Memory Space (2GB)
 *
 *  3 3 3 3|3 3 3 3|3 3 2 2|2 2 2 2|2 2 2 2|1 1 1 1|1 1 1 1|1 1
 *  9 8 7 6|5 4 3 2|1 0 9 8|7 6 5 4|3 2 1 0|9 8 7 6|5 4 3 2|1 0 9 8|7 6 5 4|3 2 1 0
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |1|1|0| | | | | | |A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A|A| | |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *  | \_/            \________________________________________________________/
 *  |  |                         Address (pA[31] ==1, pA[30..02])
 *  |  +-Space identifier
 *  +-- IO space, not cached
 *
 */

#define BYTE_ENABLE_SHIFT 5
#define TRANSFER_LENGTH_SHIFT 3
#define IO_MASK 0xffffff         /* IO mask is 24 bits */
#define PCI_MEM   ((ul)0x400<<29)    /*    CPU Adr[39:29]=0x400 select PCI Mem.	*/
#define PCI_IO	  ((ul)0x501<<29)    /*    CPU Adr[39:29]=0x501 select PCI I/O.	*/
#define PCI_CFG0  ((ul)0x502<<29)    /*    CPU Adr[39:29]=0x502 select PCI Cfg.	*/
#define PCI_CFG1  ((ul)0x503<<29)    /*    CPU Adr[39:29]=0x503 select PCI Cfg.	*/

#define PCI_MEM_BITS(a)  ((PCI_MEM  >> BYTE_ENABLE_SHIFT) | a)
#define PCI_IO_BITS(a)   ((PCI_IO   >> BYTE_ENABLE_SHIFT) | a)
#define PCI_CFG0_BITS(a) ((PCI_CFG0 >> BYTE_ENABLE_SHIFT) | a)
#define PCI_CFG1_BITS(a) ((PCI_CFG1 >> BYTE_ENABLE_SHIFT) | a)

/****************************************************************************
 * Slot D Physical Address Map                                              *
 ****************************************************************************/

#define SLOT_D_ROM                 PCI_MEM_BITS(0x000000)
#define SLOT_D_AUDIO_RAM           PCI_MEM_BITS(0x100000)
#define SLOT_D_KEYBOARD            PCI_MEM_BITS(0x120000)
#define SLOT_D_AUDIO_CODEC         PCI_MEM_BITS(0x130000)
#define SLOT_D_COM1                PCI_MEM_BITS(0x140000)
#define SLOT_D_COM2                PCI_MEM_BITS(0x150000)
#define SLOT_D_CLOCK_ADDR          PCI_MEM_BITS(0x160000)
#define SLOT_D_CLOCK_DATA          PCI_MEM_BITS(0x170000)
#define SLOT_D_PCI_ERROR_REGISTER  PCI_MEM_BITS(0x180000)
#define SLOT_D_SCSI                PCI_MEM_BITS(0x190000)
#define SLOT_D_AUDIO_PLAY_ADDR     PCI_MEM_BITS(0x1a0000)
#define SLOT_D_AUDIO_PLAY_LIMIT    PCI_MEM_BITS(0x1b0000)
#define SLOT_D_AUDIO_CAPTURE_ADDR  PCI_MEM_BITS(0x1c0000)
#define SLOT_D_AUDIO_CAPTURE_LIMIT PCI_MEM_BITS(0x1d0000)
#define SLOT_D_INTERRUPT_STATUS    PCI_MEM_BITS(0x1e0000)
#define SLOT_D_INTERRUPT_ENABLE    PCI_MEM_BITS(0x1f0000)

/****************************************************************************
 * UART Definitions                                                         *
 ****************************************************************************/

#define com1Rbr	(SLOT_D_COM1 | (0x0 << 1))
#define com1Thr (SLOT_D_COM1 | (0x0 << 1))
#define com1Ier (SLOT_D_COM1 | (0x1 << 1))
#define com1Iir (SLOT_D_COM1 | (0x2 << 1))
#define com1Lcr (SLOT_D_COM1 | (0x3 << 1))
#define com1Mcr (SLOT_D_COM1 | (0x4 << 1))
#define com1Lsr (SLOT_D_COM1 | (0x5 << 1))
#define com1Msr (SLOT_D_COM1 | (0x6 << 1))
#define com1Scr (SLOT_D_COM1 | (0x7 << 1))
#define com1Dll (SLOT_D_COM1 | (0x8 << 1))
#define com1Dlm (SLOT_D_COM1 | (0x9 << 1))

#define com2Rbr	(SLOT_D_COM2 | (0x0 << 1))
#define com2Thr (SLOT_D_COM2 | (0x0 << 1))
#define com2Ier (SLOT_D_COM2 | (0x1 << 1))
#define com2Iir (SLOT_D_COM2 | (0x2 << 1))
#define com2Lcr (SLOT_D_COM2 | (0x3 << 1))
#define com2Mcr (SLOT_D_COM2 | (0x4 << 1))
#define com2Lsr (SLOT_D_COM2 | (0x5 << 1))
#define com2Msr (SLOT_D_COM2 | (0x6 << 1))
#define com2Scr (SLOT_D_COM2 | (0x7 << 1))
#define com2Dll (SLOT_D_COM2 | (0x8 << 1))
#define com2Dlm (SLOT_D_COM2 | (0x9 << 1))

#define	lptDr 0x0
#define	lptSr 0x0
#define	lptCr 0x0

#define	lptSTB 0x0
#define	lptAFD 0x0
#define	lptnInit 0x0
#define	lptSlct 0x0
#define	lptIrq 0x0
#define	lptDir 0x0

#define	lptDly	100000

#define RTCADDR SLOT_D_CLOCK_ADDR
#define RTCDATA SLOT_D_CLOCK_DATA
/* 114 bytes of NVRAM in the clock chip */
#define RTCRAM 114
#define ROMBASE SLOT_D_ROM
#define ROMINC 2 /* Byte wide, with bytes in low half of 16-bit word */
#define ROMSIZE (512 * 1024)

/* Map 16MB of slot A space at offset 16MB of IO space */
#define SLOT_A_IO_BASE		   PCI_IO_BITS(0x1000000)

#endif /* XXM */
#endif /* __XXM_H_LOADED */
