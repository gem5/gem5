#ifndef	__SYS_H_LOADED
#define	__SYS_H_LOADED
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
 *  $Id: sys.h,v 1.1.1.1 1997/10/30 23:27:17 verghese Exp $;
 */

/*
 * $Log: sys.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:17  verghese
 * current 10/29/97
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

/*
**++
*****************************************************************************
**									    *
**  Copyright (c) 1992							    *
**  by Digital Equipment Corporation, Maynard, Ma.			    *
** 									    *
**  This software is furnished under a license and may be used and  copied  *
**  only  in  accordance  with  the  terms  of  such  license and with the  *
**  inclusion of the above copyright notice.  This software or  any  other  *
**  copies  thereof may not be provided or otherwise made available to any  *
**  other person.  No title to and ownership of  the  software  is  hereby  *
**  transferred.							    *
** 									    *
**  The information in this software is subject to change  without  notice  *
**  and  should  not  be  construed  as  a commitment by Digital Equipment  *
**  Corporation.							    *
** 									    *
**  Digital assumes no responsibility for the use or  reliability  of  its  *
**  software on equipment which is not supplied by Digital.		    *
**									    *
*****************************************************************************
**
**  FACILITY:
**
**	sys.h
**
**  MODULE DESCRIPTION:
**
**      DECchip 21064 Evaluation and Development System (ED64)
**	specific definitions.
**
**  CREATION DATE:  17-Nov-1992
**
**  DESIGN ISSUES:
**
**      {@tbs@}
**
**  [@optional module tags@]...
**
**  MODIFICATION HISTORY:
**
**      Who	When		What
**	-----	-----------	---------------------------------------------
**	ES	12-Jan-1993	Add SYS$K_HIER_CRE
**	ES	14-Jan-1993	Add some impure area symbols
**	ES	15-Jan-1993	Can't use # for comments on defines
**	ES	20-Jan-1993	Put in intmask
**	ES	27-Jan-1993	SYS$K_HIERW_CRE is covered by abox control
**--
*/

/*
**
**  External cache (Bcache) definitions:
**
*/
#define BC$M_PA_DIS	    0x000E  /* Cache only 1st quadrant of PA space */
#define BC$M_SIZE	    0x2000  /* External cache size = 512K bytes */
#define BC$M_WE_CTL4	    0x0000  /* External cache write control */
#define BC$M_WE_CTL3	    0x0000  /* Write pulse on 3rd, 4th & 5th cycle */
#define BC$M_WE_CTL2	    0x0001
#define BC$M_WE_CTL1	    0xC000
#define BC$M_WR_SPD	    0x0600  /* External cache write speed = 7 cycles */
#define BC$M_RD_SPD	    0x0040  /* External cache read speed = 5 cycles */
#define BC$M_ENA	    0x0001  /* External cache enable */

/*
**  We may want to move the BIU$M_INIT definitions to another module
**  that contains user configurable options.
*/
#define BIU$M_INIT_47_32    BC$M_PA_DIS

#define BIU$M_INIT_31_16    BC$M_SIZE	 | \
                            BC$M_WE_CTL4 | \
                            BC$M_WE_CTL3 | \
                            BC$M_WE_CTL2

#define BIU$M_INIT_15_00    BC$M_WE_CTL1 | \
                            BC$M_WR_SPD	 | \
                            BC$M_RD_SPD	 | \
                            BC$M_ENA
/*
** Interrupt Mask
** ED64 specific IRQ pins are:
**               IRQ0            PIC     ipl <=2
**               IRQ1            NMI     disabled
**               IRQ2            RTC     ipl <= 4
*/
#define	SYS$M_HIGHINTMASK   0x00000008
#define SYS$M_LOWINTMASK    0x080A0A0A

/*
**
**  SCB offsets
**
*/
#define	SCB$Q_PROCMCHK		0x0670   /* Offset for mchk */
#define	SCB$Q_SYSERR		0x0620   /* Offset for sce */


/*
**
**  Machine Check Rev level
**
*/
#define	MCHK$K_REV		0x0001

/*
**
**  Short logout frame
**
*/
#define LAS$Q_BASE		0x0000	/* Base relative to logout area */

#define	LAS$L_FRAME		0x0000
#define	LAS$L_FLAG		0x0004
#define	LAS$L_CPU		0x0008
#define	LAS$L_SYS		0x000C
#define	LAS$Q_MCHK_CODE		0x0010

#define	LAS$Q_BIU_STAT		0x0018
#define	LAS$Q_BIU_ADDR		0x0020
#define	LAS$Q_DC_STAT		0x0028
#define	LAS$Q_FILL_SYNDROME	0x0030
#define	LAS$Q_FILL_ADDR		0x0038
#define	LAS$Q_BC_TAG		0x0040

#define	LAS$K_SIZE		0x0048	/* Frame size */

/*
**
**  Long logout frame
**
*/
#define LAF$Q_BASE		LAS$K_SIZE  /* Base relative to logout area */

#define	LAF$L_FRAME		0x0000
#define	LAF$L_FLAG		0x0004
#define	LAF$L_CPU		0x0008
#define	LAF$L_SYS		0x000C

#define	LAF$Q_PT0		0x0010
#define	LAF$Q_PT1		0x0018
#define	LAF$Q_PT2		0x0020
#define	LAF$Q_PT3		0x0028
#define	LAF$Q_PT4		0x0030
#define	LAF$Q_PT5		0x0038
#define	LAF$Q_PT6		0x0040
#define	LAF$Q_PT7		0x0048
#define	LAF$Q_PT8		0x0050
#define	LAF$Q_PT9		0x0058
#define	LAF$Q_PT10		0x0060
#define	LAF$Q_PT11		0x0068
#define	LAF$Q_PT12		0x0070
#define	LAF$Q_PT13		0x0078
#define	LAF$Q_PT14		0x0080
#define	LAF$Q_PT15		0x0088
#define	LAF$Q_PT16		0x0090
#define	LAF$Q_PT17		0x0098
#define	LAF$Q_PT18		0x00A0
#define	LAF$Q_PT19		0x00A8
#define	LAF$Q_PT20		0x00B0
#define	LAF$Q_PT21		0x00B8
#define	LAF$Q_PT22		0x00C0
#define	LAF$Q_PT23		0x00C8
#define	LAF$Q_PT24		0x00D0
#define	LAF$Q_PT25		0x00D8
#define	LAF$Q_PT26		0x00E0
#define	LAF$Q_PT27		0x00E8
#define	LAF$Q_PT28		0x00F0
#define	LAF$Q_PT29		0x00F8
#define	LAF$Q_PT30		0x0100
#define	LAF$Q_PT31		0x0108

#define LAF$Q_EXC_ADDR		0x0110
#define LAF$Q_PAL_BASE		0x0130
#define LAF$Q_HIER		0x0138
#define LAF$Q_HIRR		0x0140
#define LAF$Q_MM_CSR		0x0148
#define LAF$Q_DC_STAT		0x0150
#define LAF$Q_DC_ADDR		0x0158
#define LAF$Q_ABOX_CTL		0x0160
#define LAF$Q_BIU_STAT		0x0168
#define LAF$Q_BIU_ADDR		0x0170
#define LAF$Q_BIU_CTL		0x0178
#define LAF$Q_FILL_SYNDROME	0x0180
#define LAF$Q_FILL_ADDR		0x0188
#define LAF$Q_VA		0x0190
#define LAF$Q_BC_TAG		0x0198

#define LAF$K_SIZE		0x01A0	    /* Frame size */

#define LAF$Q_SYS_BASE		0x01A0	    /* Currently no system stuff */

/*
**
**  Impure Area Offset Definitions
**
*/
#define IMP$Q_ABOX_CTL		0x0380
#define IMP$Q_BIU_CTL		0x0388
#define IMP$Q_LOGOUT_AREA	0x2000

#endif				/* __SYS_H_LOADED */
