#ifndef __FLOPPY_H_LOADED
#define __FLOPPY_H_LOADED
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
 *  $Id: floppy.h,v 1.1.1.1 1997/10/30 23:27:15 verghese Exp $;
 */

/*
 * $Log: floppy.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:15  verghese
 * current 10/29/97
 *
 * Revision 1.5  1994/08/05  20:13:47  fdh
 * Updated Copyright header and RCS $Id: identifier.
 *
 * Revision 1.4  1994/06/22  15:22:35  rusling
 * Fixed up minor OSF build problem.  Changed fdacmd()
 * definition.
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



#define	TRUE	1
#define	FALSE	0
#define REALFLOPPY 1

/* pic */
#define	PICBOCW	0x20
#define	PICBIRR	0x20
#define	READIRR	0x0A
#define	FDAMASK	0x40

/* dma */
#define DMACMD03 0x08
#define DMACMD47 0xD0
#define DMAMODE03 0x0B
#define DMAMODE47 0xD6
#define DMAXMODE03 0x40B
#define DMAXMODE47 0x4D6
#define	DMAOFFS	0x04
#define	DMALPAG	0x81
#define DMAHPAG 0x481
#define	DMACNT	0x05
#define	DMAHCNT	0x405
#define	DMACBP	0x0C
#define DMAMASK03 0x0A
#define DMAMASK47 0xD4

/* fda */
#define	FDADCR	0x03F2
#define	FDAMSR	0x03F4
#define	FDADR	0x03F5
#define	FDADRR	0x03F7
#define	FDADCB	0x03F7

/* dcr */
#define	DCRSELA	0x00
#define	DCRSELB	0x01
#define	DCRSELC	0x02
#define	DCRSELD	0x03
#define	DCRNRES	0x04
#define	DCRENAB	0x08
#define	DCRMTRA	0x10
#define	DCRMTRB	0x20
#define	DCRMTRC	0x40
#define	DCRMTRD	0x80

/* msr */
#define	MSRDIO	0x40
#define	MSRRQM	0x80

/* drr */
#define	DRR500	0x00
#define	DRR250	0x02

/* dcb */
#define	DCBNCHG	0x80

/* st0 */
#define	ST0IC	0xC0
#define	ST0NT	0x00
#define	ST0NR	0x08

/* st1 */
#define	ST1NW	0x02

/* cmd */
#define	NREAD	0x66
#define	NWRITE	0x45
#define	NRECAL	0x07
#define	NSEEK	0x0F
#define	NSINTS	0x08
#define	NSPEC	0x03

#define	FDTOMEM	0
#define	MEMTOFD	1

#define	NCMD	9
#define	NSTS	7

#define	UNITNUM	0
#define	UNITSEL	(DCRSELA)
#define	UNITMTR	(DCRMTRA)

#define	lSRT	0xE0			/* 4 mS				*/
#define	lHUT	0x08			/* 256 mS			*/

#define	hSRT	0xC0			/* 4 mS				*/
#define	hHUT	0x0F			/* 256 mS			*/

#define	HLT	0x02			/* 4 mS				*/
#define	ND	0x00			/* Use DMA			*/

int	fdactrk;

static void read_sector(int sec , int loc);
static int get_file_info(char * file2load , int flag);
static void prep_filename(ub * fn , ub * nfn , ub * ext);
static int get_boot_info(void );
static void load_file(void );
static int get_sec(int gcluster);
static int get_next_cluster(ui gcluster , ui ifat);
static void fdainit(int rate);
static void fdaspinup(void );
static void fdaspindown(void );
static int fdaio(char * buf , int sec , int type);
static void fdacmd(unsigned char cmd[] , int ncmd);
static int fdasts(void );
static void fdawait(void );
static void fdasleep(int nmsec);
static void init_pic(void );

#define fdREAD  0		               /* read command */
#define fdWRITE 1			       /* write command */

#endif /* __FLOPPY_H_LOADED */
