#ifndef __FLASH_H_LOADED
#define __FLASH_H_LOADED

/*****************************************************************************

Copyright © 1994, Digital Equipment Corporation, Maynard, Massachusetts.

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
#define FLASH_H_RCSID "$Id: flash.h,v 1.1.1.1 1997/10/30 23:27:15 verghese Exp $"

/*
 * Defines constants and macros for controlling the flash memory device.
 *
 * $Log: flash.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:15  verghese
 * current 10/29/97
 *
 * Revision 1.2  1995/02/02  21:42:06  cruz
 * Changed prototype for two flash functions.
 *
 * Revision 1.1  1994/11/08  21:40:02  fdh
 * Initial revision
 *
 *
 */

#define FLASH_BYTE 1
#define FLASH_WORD 2
#define FLASH_LONG 4
#define FLASH_QUAD 8

#if (defined INTEL_28F008SA || defined INTEL_28F004BX_T)
/* Flash commands */
#define	READ_RESET	0xff
#define IID		0x90
#define READ_STATUS	0x70
#define CLEAR_STATUS	0x50
#define ERASE_SETUP	0x20
#define ERASE_CONFIRM	0xd0
#define ERASE_SUSPEND	0xB0
#define BYTE_WRITE	0x40
#define	ALT_WR_SET_WR	0x10

/* Status bits */
#define	SR_READY	(1 << 7)
#define SR_ERASE_SUSP	(1 << 6)
#define SR_ERASE_ERR	(1 << 5)
#define SR_WRITE_ERR	(1 << 4)
#define SR_VPP_LOW	(1 << 3)
#define SR_CMD_SEQ_ERR	(SR_WRITE_ERR | SR_ERASE_ERR)
#endif

#ifdef INTEL_28F008SA
#define BX_T_M_CODE	0x89
#define BX_T_D_CODE	0xA2
#define DEVICE_NAME	"Intel 28F008SA"

#define NUMSEGS		16
#define SEGSIZE		(64 * 1024)
#endif /* INTEL_28F008SA */

#ifdef INTEL_28F004BX_T
#define BX_T_M_CODE	0x89
#define BX_T_D_CODE	0x78
#define DEVICE_NAME	"Intel 28F004BX-T"

#define NUMSEGS		4
#define SEGSIZE		(128 * 1024)
#endif /* INTEL_28F004BX_T */

/* prototypes */
static ui inflash(ui size , ui offset);
static void outflash(ui size , ui offset , ui d);
static int flash_present(void);
static int flash_write_segs (ui segstart, ui segcnt, ui total_size, unsigned char *buf);
static int read_status(void);
static int flash_decode(int status);
static int flash_erase(ui segnum);
static int flash_program (ui offset, ui size, unsigned char *buf);
static int clear_status(void);

#endif /* __FLASH_H_LOADED */
