#ifndef	__EV4_H_LOADED
#define	__EV4_H_LOADED
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
 *  $Id: ev4.h,v 1.1.1.1 1997/10/30 23:27:15 verghese Exp $;
 */

/*
 * $Log: ev4.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:15  verghese
 * current 10/29/97
 *
 * Revision 1.3  1994/08/05  20:13:47  fdh
 * Updated Copyright header and RCS $Id: identifier.
 *
 * Revision 1.2  1994/06/20  14:18:59  fdh
 * Fixup header file preprocessor #include conditionals.
 *
 * Revision 1.1  1993/06/08  19:56:13  fdh
 * Initial revision
 *
 */



/*
**++
*****************************************************************************
**									    *
**  Copyright (c) 1992, 1993						    *
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
**	ev4.h
**
**  MODULE DESCRIPTION:
**
**      DECchip 21064 (EV4) specific definitions
**
**  CREATION DATE:  29-Oct-1992
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
**      ES	10-Feb-1993	Need physical load lock and store conditional
**	ER	05-Feb-1993	Add HWPCB offset definitions.
**	ES	12-Jan-1993	Some HIRR additions
**	ES	04-Jan-1993	Changed names of BIU_STAT bits
**	ES	16-Dec-1992	Added BIU_STAT and SL_CLR definitions.
**	ER	10-Dec-1992	Added xtbAsm to invalidate all ITB & DTB
**				entries with ASM=0.
**	ER	03-Dec-1992	Added definitions for architecturally reserved
**				opcodes.
**	ER	16-Nov-1992	Changed BIU$M_INIT definition.
**	ER	12-Nov-1992	Added FPR definitions.
**	ER	09-Nov-1992	Added GPR and BIU$M_INIT definitions.
**	ES	09-Nov-1992	Added privileged and unprivileged CALL_PAL
**				entry points.
**	ER	02-Nov-1992	Added MMCSR$M_FAULT bit mask.
**	ER	30-Oct-1992	Added xtbZap to invalidate all ITB & DTB
**				entries.
**
**--
*/

/*
**
**  INTERNAL PROCESSOR REGISTER DEFINITIONS
**
*/

#define IPR$V_PAL	7
#define IPR$M_PAL	0x80
#define IPR$V_ABX	6
#define IPR$M_ABX	0x40
#define IPR$V_IBX	5
#define IPR$M_IBX	0x20
#define IPR$V_INDEX	0
#define IPR$M_INDEX	0x1F

/*
**
**  Ibox IPR Definitions
**
*/

#define tbTag		IPR$M_IBX + 0x0
#define itbPte		IPR$M_IBX + 0x1
#define iccsr		IPR$M_IBX + 0x2
#define itbPteTemp	IPR$M_IBX + 0x3
#define excAddr		IPR$M_IBX + 0x4
#define slRcv		IPR$M_IBX + 0x5
#define itbZap		IPR$M_IBX + 0x6
#define itbAsm		IPR$M_IBX + 0x7
#define itbIs		IPR$M_IBX + 0x8
#define ps		IPR$M_IBX + 0x9
#define excSum		IPR$M_IBX + 0xA
#define palBase		IPR$M_IBX + 0xB
#define hirr		IPR$M_IBX + 0xC
#define sirr		IPR$M_IBX + 0xD
#define astrr		IPR$M_IBX + 0xE
#define hier		IPR$M_IBX + 0x10
#define sier		IPR$M_IBX + 0x11
#define aster		IPR$M_IBX + 0x12
#define slClr		IPR$M_IBX + 0x13
#define slXmit		IPR$M_IBX + 0x16

/*
**
**  Instruction Cache Control and Status Register (ICCSR) Bit Summary
**
**	  Loc	Size	Name	Function
**	 -----	----	----	---------------------------------
**	  <42>	  1	FPE	Floating Point Enable
**	  <41>	  1	MAP	I-stream superpage mapping enable
**	  <40>	  1	HWE	Allow PALRES to be issued in kernel mode
**	  <39>	  1	DI	Dual Issue enable
**	  <38>	  1	BHE	Branch History Enable
**	  <37>	  1	JSE	JSR Stack Enable
**	  <36>	  1	BPE	Branch Prediction Enable
**	  <35>	  1	PIPE	Pipeline enable
**
*/

#define ICCSR$V_FPE	    42
#define ICCSR$M_FPE	    0x0400
#define ICCSR$V_MAP	    41
#define ICCSR$M_MAP	    0x0200
#define ICCSR$V_HWE	    40
#define ICCSR$M_HWE	    0x0100
#define ICCSR$V_DI	    39
#define ICCSR$M_DI	    0x0080
#define ICCSR$V_BHE	    38
#define ICCSR$M_BHE	    0x0040
#define ICCSR$V_JSE	    37
#define ICCSR$M_JSE	    0x0020
#define ICCSR$V_BPE	    36
#define ICCSR$M_BPE	    0x0010
#define ICCSR$V_PIPE	    35
#define ICCSR$M_PIPE	    0x0008

/*
**  We may want to move the ICCSR$M_INIT definition to another module
**  that contains user configurable options.
*/
#define ICCSR$M_INIT	    (ICCSR$M_FPE | ICCSR$M_MAP | ICCSR$M_DI  | \
                             ICCSR$M_BHE | ICCSR$M_JSE | ICCSR$M_BPE | \
                             ICCSR$M_PIPE)
/*
**
**  Exception Summary Register (EXC_SUM) Bit Summary
**
**	 Loc	Size	Name	Function
**	-----	----	----	------------------------------------
**	 <33>	 1	MSK	Exception Register Write Mask window
**	  <8>	 1	IOV	Integer overflow
**	  <7>	 1	INE	Inexact result
**	  <6>	 1	UNF	Underflow
**	  <5>	 1	OVF	Overflow
**	  <4>	 1	DZE	Division by zero
**	  <3>	 1	INV	Invalid operation
**	  <2>	 1	SWC	Software completion
**
*/

#define EXC$V_MSK	33
#define EXC$M_MSK	0x0002
#define EXC$V_IOV	8
#define EXC$M_IOV	0x0100
#define EXC$V_INE	7
#define EXC$M_INE	0x0080
#define EXC$V_UNF	6
#define EXC$M_UNF	0x0040
#define EXC$V_OVF	5
#define EXC$M_OVF	0x0020
#define EXC$V_DZE	4
#define	EXC$M_DZE	0x0010
#define EXC$V_INV	3
#define EXC$M_INV	0x0008
#define	EXC$V_SWC	2
#define EXC$M_SWC	0x0004

/*
**
**  Hardware Interrupt Request Register (HIRR) Bit Summary
**
**	  Loc	Size	Name	Function
**	 -----	----	----	---------------------------------
**	  <13>	  1	SLR	Serial Line interrupt
**	   <9>	  1	PC0	Performance Counter 0 interrupt
**	   <8>	  1	PC1	Performance Counter 1 interrupt
**	   <4>	  1	CRR	Correctable read data interrupt
**	   <3>	  1	ATR	AST interrupt
**	   <2>	  1	SWR	Software interrupt
**	   <1>	  1	HWR	Hardware interrupt
**
**/

#define HIRR$V_SLR	13
#define HIRR$M_SLR	0x2000
#define HIRR$V_PC0	9
#define HIRR$M_PC0	0x0200
#define HIRR$V_PC1	8
#define HIRR$M_PC1	0x0100
#define HIRR$V_CRR	4
#define HIRR$M_CRR	0x0010
#define HIRR$V_ATR	3
#define HIRR$M_ATR	0x0008
#define HIRR$V_SWR	2
#define HIRR$M_SWR	0x0004
#define HIRR$V_HWR	1
#define HIRR$M_HWR	0x0002

#define HIRR$V_IRQ0	10
#define HIRR$V_IRQ1	11
#define HIRR$V_IRQ2	12
#define HIRR$V_IRQ3	5
#define HIRR$V_IRQ4	6
#define HIRR$V_IRQ5	7

/*
**
**  Hardware Interrupt Enable Register (HIER) Write Bit Summary
**
**	  Loc	Size	Name	Function
**	 -----	----	----	---------------------------------
**	  <32>	  1	SLE	Serial Line interrupt enable
**	  <15>	  1	PC1	Performance Counter 1 interrupt enable
**	<14:9>	  6	HIER	Interrupt enables for Irq_h<5:0>
**	   <8>	  1	PC0	Performance Counter 0 interrupt enable
**	   <2>	  1	CRE	Correctable read data interrupt enable
**
**/

#define HIERW$V_SLE	32
#define HIERW$V_PC1	15
#define HIERW$V_PC0	8
#define HIERW$V_CRE	2

/*
**
**  Clear Serial Line Interrupt Register (SL_CLR) Bit Summary
**
**	  Loc	Size	Name	    Function
**	 -----	----	----	    ---------------------------------
**	   <32>	  1	SLC	    W0C -- Clear serial line int request
**	   <15>	  1	PC1	    W0C -- Clear PC1 interrupt request
**	    <8>	  1	PC0	    W0C -- Clear PC0 interrupt request
**	    <2>	  1	CRD	    W0C -- Clear CRD interrupt request
**
*/

#define SL_CLR$V_SLC	    32
#define SL_CLR$V_PC1	    15
#define SL_CLR$V_PC0	    8
#define SL_CLR$V_CRD	    2

/*
**
**  Abox IPR Definitions
**
*/

#define dtbCtl		IPR$M_ABX + 0x0
#define tbCtl		IPR$M_ABX + 0x0
#define dtbPte		IPR$M_ABX + 0x2
#define dtbPteTemp	IPR$M_ABX + 0x3
#define mmcsr		IPR$M_ABX + 0x4
#define va		IPR$M_ABX + 0x5
#define dtbZap		IPR$M_ABX + 0x6
#define dtbAsm		IPR$M_ABX + 0x7
#define dtbIs		IPR$M_ABX + 0x8
#define biuAddr		IPR$M_ABX + 0x9
#define biuStat		IPR$M_ABX + 0xA
#define dcAddr		IPR$M_ABX + 0xB
#define dcStat		IPR$M_ABX + 0xC
#define fillAddr	IPR$M_ABX + 0xD
#define aboxCtl		IPR$M_ABX + 0xE
#define altMode		IPR$M_ABX + 0xF
#define cc		IPR$M_ABX + 0x10
#define ccCtl		IPR$M_ABX + 0x11
#define biuCtl		IPR$M_ABX + 0x12
#define fillSyndrome	IPR$M_ABX + 0x13
#define bcTag		IPR$M_ABX + 0x14
#define flushIc		IPR$M_ABX + 0x15
#define flushIcAsm	IPR$M_ABX + 0x17
#define xtbZap		IPR$M_ABX + IPR$M_IBX + 0x6
#define xtbAsm		IPR$M_ABX + IPR$M_IBX + 0x7

/*
**
**  Memory Management Control and Status Register (MM_CSR) Bit Summary
**
**	  Loc	Size	Name	Function
**	 -----	----	----	---------------------------------
**	<14:9>	  6	OPC	Opcode of faulting instruction
**	 <8:4>	  5	RA	Ra field of faulting instruction
**	   <3>	  1	FOW	Fault on write
**	   <2>	  1	FOR	Fault on read
**	   <1>	  1	ACV	Access violation
**	   <0>	  1	WR	Faulting reference is a write
**
*/

#define	MMCSR$V_OPC	9
#define MMCSR$M_OPC	0x7E00
#define MMCSR$V_RA	4
#define MMCSR$M_RA	0x01F0
#define MMCSR$V_FOW	3
#define MMCSR$M_FOW	0x0008
#define MMCSR$V_FOR	2
#define MMCSR$M_FOR	0x0004
#define MMCSR$V_ACV	1
#define MMCSR$M_ACV	0x0002
#define MMCSR$V_WR	0
#define MMCSR$M_WR	0x0001

#define MMCSR$M_FAULT	0x000E

/*
**
** Abox Control Register (ABOX_CTL) Bit Summary
**
**	  Loc	Size	Name	    Function
**	 -----	----	----	    ---------------------------------
**	  <11>	  1	DC_FHIT	    Dcache Force Hit
**	  <10>	  1	DC_ENA	    Dcache Enable
**	   <6>	  1	EMD_EN	    Limited big endian support enable
**	   <5>	  1	SPE_2	    D-stream superpage 1 enable
**	   <4>	  1	SPE_1	    D-stream superpage 2 enable
**	   <3>	  1	IC_SBUF_EN  Icache Stream Buffer Enable
**	   <2>	  1	CRD_EN	    Corrected Read Data Enable
**	   <1>	  1	MCHK_EN	    Machine Check Enable
**	   <0>	  1	WB_DIS	    Write Buffer unload Disable
**
*/

#define ABOX$V_DC_FHIT	    11
#define ABOX$M_DC_FHIT	    0x0800
#define ABOX$V_DC_ENA	    10
#define	ABOX$M_DC_ENA	    0x0400
#define ABOX$V_EMD_EN	    6
#define ABOX$M_EMD_EN	    0x0040
#define ABOX$V_SPE_2	    5
#define ABOX$M_SPE_2	    0x0020
#define ABOX$V_SPE_1	    4
#define ABOX$M_SPE_1	    0x0010
#define ABOX$V_IC_SBUF_EN   3
#define ABOX$M_IC_SBUF_EN   0x0008
#define ABOX$V_CRD_EN	    2
#define ABOX$M_CRD_EN	    0x0004
#define ABOX$V_MCHK_EN	    1
#define ABOX$M_MCHK_EN	    0x0002
#define ABOX$V_WB_DIS	    0
#define	ABOX$M_WB_DIS	    0x0001

/*
**  We may want to move the ABOX$M_INIT definition to another module
**  that contains user configurable options.
*/
#define	ABOX$M_INIT	    (ABOX$M_DC_ENA | ABOX$M_SPE_2 | ABOX$M_IC_SBUF_EN)

/*
**
**  Bus Interface Unit Control Register (BIU_CTL) Bit Summary
**
**	  Loc	Size	Name	    Function
**	 -----	----	----	    ---------------------------------
**	   <36>	  1	BAD_DP	    Force bad data parity/ECC check bits
**	<35:32>	  4	BC_PA_DIS   Don't cache PA quadrant specified
**	   <31>	  1	BAD_TCP	    Force bad tag parity
**	<30:28>	  3	BC_SIZE	    External cache size
**	<27:13>	 16	BC_WE_CTL   External cache write enable control
**	 <11:8>	  4	BC_WR_SPD   External cache write speed
**	  <7:4>	  4	BC_RD_SPD   External cache read speed
**	    <3>	  1	BC_FHIT	    External cache force hit
**	    <2>	  1	OE	    Output enable
**	    <1>	  1	ECC	    Enable ECC
**	    <0>	  1	BC_ENA	    External cache enable
**
*/

#define	BIU$V_BAD_DP	    36
#define BIU$M_BAD_DP	    0x0010
#define BIU$V_BC_PA_DIS	    32
#define BIU$M_BC_PA_DIS	    0x000F
#define BIU$V_BAD_TCP	    31
#define BIU$M_BAD_TCP	    0x8000
#define BIU$V_BC_SIZE	    28
#define BIU$M_BC_SIZE	    0x7000
#define BIU$V_BC_WE_CTL	    13
#define BIU$M_BC_WE_CTL	    0xFFFE
#define BIU$V_BC_WR_SPD	    8
#define BIU$M_BC_WR_SPD	    0x0F00
#define BIU$V_BC_RD_SPD	    4
#define BIU$M_BC_RD_SPD	    0x00F0
#define BIU$V_BC_FHIT	    3
#define BIU$M_BC_FHIT	    0x0008
#define BIU$V_OE	    2
#define BIU$M_OE	    0x0004
#define BIU$V_ECC	    1
#define BIU$M_ECC	    0x0002
#define BIU$V_BC_ENA	    0
#define BIU$M_BC_ENA	    0x0001

/*
**  We may want to move the BIU$M_INIT definition to another module
**  that contains user configurable options.
*/
#define	BIU$M_INIT	    BIU$M_OE

/*
**
**  Bus Interface Unit Status Register (BIU_STAT) Bit Summary
**
**	  Loc	Size	Name	    Function
**	 -----	----	----	    ---------------------------------
**	   <14>	  1	FILL_SEO    Second error while FILL_ECC or FILL_DPERR
**	<13:12>	  2	FILL_QW	    Used with FILL_ADDR for physical address
**	   <11>	  1	FILL_IRD    Icache fill when FILL_ECC or FILL_DPERR
**	   <10>	  1	FILL_DPERR  Fill parity error or double bit ECC
**	    <9>	  1	FILL_CRD    Corrected read data
**	    <8>	  1	FILL_ECC    ECC error
**	    <7>	  1	BIU_SEO	    Second error while BIU or BC error
**	  <6:4>	  3	BIU_CMD	    Cycle type
**	    <3>	  1	BC_TCPERR   Tag control parity error on external cache
**	    <2>	  1	BC_TPERR    Tag address parity error on external cache
**	    <1>	  1	BIU_SERR    cAck_h pins indicate SOFT_ERROR
**	    <0>	  1	BIU_HERR    cAck_h pins indicate HARD_ERROR
**
*/

#define BIU_STAT$V_FILL_SEO	14
#define BIU_STAT$M_FILL_SEO	0x4000
#define BIU_STAT$V_FILL_CRD	9
#define BIU_STAT$M_FILL_CRD	0x0200
#define	BIU_STAT$V_FILL_ECC	8
#define BIU_STAT$M_FILL_ECC	0x0100
#define BIU_STAT$V_BC_TCPERR	3
#define BIU_STAT$M_BC_TCPERR	0x0008
#define BIU_STAT$V_BC_TPERR	2
#define BIU_STAT$M_BC_TPERR	0x0004
#define BIU_STAT$V_BIU_SERR	1
#define BIU_STAT$M_BIU_SERR	0x0002
#define BIU_STAT$V_BIU_HERR	0
#define BIU_STAT$M_BIU_HERR	0x0001

/*
**
**  General Register Definitions
**
*/

#define	r0		$0
#define r1		$1
#define r2		$2
#define r3		$3
#define r4		$4
#define r5		$5
#define r6		$6
#define r7		$7
#define r8		$8
#define r9		$9
#define r10		$10
#define r11		$11
#define r12		$12
#define r13		$13
#define r14		$14
#define	r15		$15
#define	r16		$16
#define	r17		$17
#define	r18		$18
#define	r19		$19
#define	r20		$20
#define	r21		$21
#define r22		$22
#define r23		$23
#define r24		$24
#define r25		$25
#define r26		$26
#define r27		$27
#define r28		$28
#define r29		$29
#define r30		$30
#define r31		$31

/*
**
**  Floating Point Register Definitions
**
*/

#define f0		$f0
#define f1		$f1
#define f2		$f2
#define f3		$f3
#define f4		$f4
#define f5		$f5
#define f6		$f6
#define f7		$f7
#define f8		$f8
#define f9		$f9
#define f10		$f10
#define f11		$f11
#define f12		$f12
#define f13		$f13
#define f14		$f14
#define f15		$f15
#define f16		$f16
#define f17		$f17
#define f18		$f18
#define f19		$f19
#define f20		$f20
#define f21		$f21
#define f22		$f22
#define f23		$f23
#define f24		$f24
#define f25		$f25
#define f26		$f26
#define f27		$f27
#define f28		$f28
#define f29		$f29
#define f30		$f30
#define f31		$f31

/*
**
**  PAL Temporary Register Definitions
**
*/

#define	pt0		IPR$M_PAL + 0x0
#define	pt1		IPR$M_PAL + 0x1
#define	pt2		IPR$M_PAL + 0x2
#define	pt3		IPR$M_PAL + 0x3
#define	pt4		IPR$M_PAL + 0x4
#define	pt5		IPR$M_PAL + 0x5
#define	pt6		IPR$M_PAL + 0x6
#define	pt7		IPR$M_PAL + 0x7
#define	pt8		IPR$M_PAL + 0x8
#define	pt9		IPR$M_PAL + 0x9
#define	pt10		IPR$M_PAL + 0xA
#define	pt11		IPR$M_PAL + 0xB
#define	pt12		IPR$M_PAL + 0xC
#define	pt13		IPR$M_PAL + 0xD
#define	pt14		IPR$M_PAL + 0XE
#define	pt15		IPR$M_PAL + 0xF
#define	pt16		IPR$M_PAL + 0x10
#define	pt17		IPR$M_PAL + 0x11
#define	pt18		IPR$M_PAL + 0x12
#define	pt19		IPR$M_PAL + 0x13
#define	pt20		IPR$M_PAL + 0x14
#define	pt21		IPR$M_PAL + 0x15
#define	pt22		IPR$M_PAL + 0x16
#define	pt23		IPR$M_PAL + 0x17
#define	pt24		IPR$M_PAL + 0x18
#define	pt25		IPR$M_PAL + 0x19
#define	pt26		IPR$M_PAL + 0x1A
#define	pt27		IPR$M_PAL + 0x1B
#define	pt28		IPR$M_PAL + 0x1C
#define	pt29		IPR$M_PAL + 0x1D
#define	pt30		IPR$M_PAL + 0x1E
#define	pt31		IPR$M_PAL + 0x1F

/*
**
**  DECchip 21064 Privileged Architecture Library Entry Points
**
**	Entry Name	    Offset (Hex)	Length (Instructions)
**
**	RESET		     0000		    8
**	MCHK		     0020		   16
**	ARITH		     0060		   32
**	INTERRUPT	     00E0		   64
**	D_FAULT		     01E0		  128
**	ITB_MISS	     03E0		  256
**	ITB_ACV		     07E0		   64
**	DTB_MISS (Native)    08E0		   64
**	DTB_MISS (PAL)	     09E0		  512
**	UNALIGN		     11E0		  128
**	OPCDEC		     13E0		  256
**	FEN		     17E0		  520
**	CALL_PAL	     2000
**
*/

#define PAL$RESET_ENTRY		    0x0000
#define PAL$MCHK_ENTRY		    0x0020
#define PAL$ARITH_ENTRY		    0x0060
#define PAL$INTERRUPT_ENTRY	    0x00E0
#define PAL$D_FAULT_ENTRY	    0x01E0
#define PAL$ITB_MISS_ENTRY	    0x03E0
#define PAL$ITB_ACV_ENTRY	    0x07E0
#define PAL$NDTB_MISS_ENTRY	    0x08E0
#define PAL$PDTB_MISS_ENTRY	    0x09E0
#define PAL$UNALIGN_ENTRY	    0x11E0
#define PAL$OPCDEC_ENTRY	    0x13E0
#define PAL$FEN_ENTRY		    0x17E0
#define PAL$CALL_PAL_PRIV_ENTRY	    0x2000
#define PAL$CALL_PAL_UNPRIV_ENTRY   0x3000

/*
**
**  DECchip 21064 Parameters
**
*/

#define	CACHE_SIZE		8192
#define	CACHE_BLOCK_SIZE	32

/*
**
** Architecturally Reserved Opcode Definitions
**
*/
#define	mtpr	    hw_mtpr
#define	mfpr	    hw_mfpr

#define	ldl_a	    hw_ldl/a
#define ldq_a	    hw_ldq/a
#define stq_a	    hw_stq/a
#define stl_a	    hw_stl/a

#define ldl_p	    hw_ldl/p
#define ldq_p	    hw_ldq/p
#define stl_p	    hw_stl/p
#define stq_p	    hw_stq/p

/*
** This set defines the physical versions of load locked and store conditional
*/
#define ldl_pa	    hw_ldl/pa
#define ldq_pa	    hw_ldq/pa
#define stl_pa	    hw_stl/pa
#define stq_pa	    hw_stq/pa

/*
**
** Hardware Privileged Process Context Block (HWPCB) offsets
**
*/

#define	HWPCB$Q_KSP		0x00
#define HWPCB$Q_ESP		0x08
#define HWPCB$Q_SSP		0x10
#define HWPCB$Q_USP		0x18
#define HWPCB$Q_PTBR		0x20
#define HWPCB$Q_ASN		0x28
#define HWPCB$Q_AST		0x30
#define HWPCB$Q_FEN		0x38
#define HWPCB$Q_PCC		0x40
#define HWPCB$Q_UNIQUE		0x48
#define HWPCB$Q_IMPURE		0x50

#endif				    /* __EV4_H_LOADED */
