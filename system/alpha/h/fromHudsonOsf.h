#ifndef FROMHUDSONOSF_INCLUDED
#define FROMHUDSONOSF_INCLUDED 1
/*
 *      VID: [T1.2] PT: [Fri Apr 21 16:47:14 1995] SF: [osf.h]
 *       TI: [/sae_users/cruz/bin/vice -iplatform.s -l// -p# -DEB164 -h -m -aeb164 ]
 */
#define	__OSF_LOADED	1
/*
*****************************************************************************
**                                                                          *
**  Copyright © 1993, 1994						    *
**  by Digital Equipment Corporation, Maynard, Massachusetts.		    *
**                                                                          *
**  All Rights Reserved							    *
**                                                                          *
**  Permission  is  hereby  granted  to  use, copy, modify and distribute   *
**  this  software  and  its  documentation,  in  both  source  code  and   *
**  object  code  form,  and without fee, for the purpose of distribution   *
**  of this software  or  modifications  of this software within products   *
**  incorporating  an  integrated   circuit  implementing  Digital's  AXP   *
**  architecture,  regardless  of the  source of such integrated circuit,   *
**  provided that the  above copyright  notice and this permission notice   *
**  appear  in  all copies,  and  that  the  name  of  Digital  Equipment   *
**  Corporation  not  be  used  in advertising or publicity pertaining to   *
**  distribution of the  document  or  software without specific, written   *
**  prior permission.							    *
**                                                                          *
**  Digital  Equipment  Corporation   disclaims  all   warranties  and/or   *
**  guarantees  with  regard  to  this  software,  including  all implied   *
**  warranties of fitness for  a  particular purpose and merchantability,   *
**  and makes  no  representations  regarding  the use of, or the results   *
**  of the use of, the software and documentation in terms of correctness,  *
**  accuracy,  reliability,  currentness  or  otherwise;  and you rely on   *
**  the software, documentation and results solely at your own risk.	    *
**                                                                          *
**  AXP is a trademark of Digital Equipment Corporation.		    *
**                                                                          *
*****************************************************************************
**
**  FACILITY:
**
**	DECchip 21164 PALcode
**
**  MODULE:
**
**	osf.h
**
**  MODULE DESCRIPTION:
**
**      OSF/1 specific definitions
**
**  AUTHOR: ER
**
**  CREATION DATE:  24-Nov-1993
**
**  $Id: fromHudsonOsf.h,v 1.1.1.1 1997/10/30 23:27:19 verghese Exp $
**
**  MODIFICATION HISTORY:
**
**  $Log: fromHudsonOsf.h,v $
**  Revision 1.1.1.1  1997/10/30 23:27:19  verghese
**  current 10/29/97
**
**  Revision 1.1  1995/11/18 01:46:31  boyle
**  Initial revision
**
**  Revision 1.11  1995/04/21  02:06:30  fdh
**  Replaced C++ style comments with Standard C style comments.
**
**  Revision 1.10  1994/09/26  14:17:47  samberg
**  Complete VICE work and EB164/SD164 breakout.
**
**  Revision 1.9  1994/07/26  17:39:10  samberg
**  Changes for SD164.
**
**  Revision 1.8  1994/07/08  17:03:48  samberg
**  Changes to support platform specific additions
**
**  Revision 1.7  1994/05/20  19:23:51  ericr
**  Moved STACK_FRAME macro from osfpal.s to here
**
**  Revision 1.6  1994/05/20  18:08:19  ericr
**  Changed line comments to C++ style comment character
**
**  Revision 1.5  1994/01/11  18:43:33  ericr
**  Removed PAL version/revision and size constants
**
**  Revision 1.4  1994/01/05  16:22:32  ericr
**  Added more SCB vector offsets and MCHK error code
**
**  Revision 1.3  1994/01/03  19:35:40  ericr
**  Derive mask definitions from field constants
**
**  Revision 1.2  1993/12/22  20:43:01  eric
**  Added mask definitions for MCES bits
**
**  Revision 1.1  1993/12/16  21:55:05  eric
**  Initial revision
**
**
**--
*/

/*
**  Seg0 and Seg1 Virtual Address (VA) Format
**
**	  Loc	Size	Name	Function
**	 -----	----	----	---------------------------------
**	<42:33>  10	SEG1	First level page table offset
**	<32:23>  10	SEG2	Second level page table offset
**	<22:13>  10	SEG3	Third level page table offset
**	<12:00>  13	OFFSET	Byte within page offset
*/

#define VA_V_SEG1	33
#define	VA_M_SEG1	(0x3FF<<VA_V_SEG1)
#define VA_V_SEG2	23
#define VA_M_SEG2	(0x3FF<<VA_V_SEG2)
#define VA_V_SEG3	13
#define VA_M_SEG3	(0x3FF<<VA_V_SEG3)
#define VA_V_OFFSET	0
#define VA_M_OFFSET	0x1FFF

/*
**  Virtual Address Options: 8K byte page size
*/

#define	VA_S_SIZE	43
#define	VA_S_OFF	13
#define	va_s_off	13
#define VA_S_SEG	10
#define VA_S_PAGE_SIZE	8192

/*
**  Page Table Entry (PTE) Format
**
**	 Extent	Size	Name	Function
**	 ------	----	----	---------------------------------
**	<63:32>	  32	PFN	Page Frame Number
**	<31:16>	  16	SW	Reserved for software
**	<15:14>	   2	RSV0	Reserved for hardware SBZ
**	   <13>	   1	UWE	User Write Enable
**	   <12>	   1	KWE	Kernel Write Enable
**	<11:10>	   2	RSV1	Reserved for hardware SBZ
**	    <9>	   1	URE	User Read Enable
**	    <8>	   1	KRE	Kernel Read Enable
**	    <7>	   1	RSV2	Reserved for hardware SBZ
**	  <6:5>	   2	GH	Granularity Hint
**	    <4>	   1	ASM	Address Space Match
**	    <3>	   1	FOE	Fault On Execute
**	    <2>	   1	FOW	Fault On Write
**	    <1>	   1	FOR	Fault On Read
**	    <0>	   1	V	Valid
*/

#define	PTE_V_PFN	32
#define PTE_M_PFN	0xFFFFFFFF00000000
#define PTE_V_SW	16
#define PTE_M_SW	0x00000000FFFF0000
#define PTE_V_UWE	13
#define PTE_M_UWE	(1<<PTE_V_UWE)
#define PTE_V_KWE	12
#define PTE_M_KWE	(1<<PTE_V_KWE)
#define PTE_V_URE	9
#define PTE_M_URE	(1<<PTE_V_URE)
#define PTE_V_KRE	8
#define PTE_M_KRE	(1<<PTE_V_KRE)
#define PTE_V_GH	5
#define PTE_M_GH	(3<<PTE_V_GH)
#define PTE_V_ASM	4
#define PTE_M_ASM	(1<<PTE_V_ASM)
#define PTE_V_FOE	3
#define PTE_M_FOE	(1<<PTE_V_FOE)
#define PTE_V_FOW	2
#define PTE_M_FOW	(1<<PTE_V_FOW)
#define PTE_V_FOR	1
#define PTE_M_FOR	(1<<PTE_V_FOR)
#define PTE_V_VALID	0
#define PTE_M_VALID	(1<<PTE_V_VALID)

#define PTE_M_KSEG	0x1111
#define PTE_M_PROT	0x3300
#define pte_m_prot	0x3300

/*
**  System Entry Instruction Fault (entIF) Constants:
*/

#define IF_K_BPT        0x0
#define IF_K_BUGCHK     0x1
#define IF_K_GENTRAP    0x2
#define IF_K_FEN        0x3
#define IF_K_OPCDEC     0x4

/*
**  System Entry Hardware Interrupt (entInt) Constants:
*/

#define INT_K_IP	0x0
#define INT_K_CLK	0x1
#define INT_K_MCHK	0x2
#define INT_K_DEV	0x3
#define INT_K_PERF	0x4

/*
**  System Entry MM Fault (entMM) Constants:
*/

#define	MM_K_TNV	0x0
#define MM_K_ACV	0x1
#define MM_K_FOR	0x2
#define MM_K_FOE	0x3
#define MM_K_FOW	0x4

/*
**  Process Control Block (PCB) Offsets:
*/

#define PCB_Q_KSP	0x0000
#define PCB_Q_USP	0x0008
#define PCB_Q_PTBR	0x0010
#define PCB_L_PCC	0x0018
#define PCB_L_ASN	0x001C
#define PCB_Q_UNIQUE	0x0020
#define PCB_Q_FEN	0x0028
#define PCB_Q_RSV0	0x0030
#define PCB_Q_RSV1	0x0038

/*
**  Processor Status Register (PS) Bit Summary
**
**	Extent	Size	Name	Function
**	------	----	----	---------------------------------
**	  <3>	 1	CM	Current Mode
**	<2:0>	 3	IPL	Interrupt Priority Level
**/

#define	PS_V_CM		3
#define PS_M_CM		(1<<PS_V_CM)
#define	PS_V_IPL	0
#define	PS_M_IPL	(7<<PS_V_IPL)

#define	PS_K_KERN	(0<<PS_V_CM)
#define PS_K_USER	(1<<PS_V_CM)

#define	IPL_K_ZERO	0x0
#define IPL_K_SW0	0x1
#define IPL_K_SW1	0x2
#define IPL_K_DEV0	0x3
#define IPL_K_DEV1	0x4
#define IPL_K_CLK	0x5
#define IPL_K_RT	0x6
#define IPL_K_PERF      0x6
#define IPL_K_PFAIL     0x6
#define IPL_K_MCHK	0x7

#define IPL_K_LOW	0x0
#define IPL_K_HIGH	0x7

/*
**  SCB Offset Definitions:
*/

#define SCB_Q_FEN	    	0x0010
#define SCB_Q_ACV		0x0080
#define SCB_Q_TNV		0x0090
#define SCB_Q_FOR		0x00A0
#define SCB_Q_FOW		0x00B0
#define SCB_Q_FOE		0x00C0
#define SCB_Q_ARITH		0x0200
#define SCB_Q_KAST		0x0240
#define SCB_Q_EAST		0x0250
#define SCB_Q_SAST		0x0260
#define SCB_Q_UAST		0x0270
#define SCB_Q_UNALIGN		0x0280
#define SCB_Q_BPT		0x0400
#define SCB_Q_BUGCHK		0x0410
#define SCB_Q_OPCDEC		0x0420
#define SCB_Q_ILLPAL		0x0430
#define SCB_Q_TRAP		0x0440
#define SCB_Q_CHMK		0x0480
#define SCB_Q_CHME		0x0490
#define SCB_Q_CHMS		0x04A0
#define SCB_Q_CHMU		0x04B0
#define SCB_Q_SW0		0x0500
#define SCB_Q_SW1		0x0510
#define SCB_Q_SW2		0x0520
#define SCB_Q_SW3		0x0530
#define	SCB_Q_SW4		0x0540
#define SCB_Q_SW5		0x0550
#define SCB_Q_SW6		0x0560
#define SCB_Q_SW7		0x0570
#define SCB_Q_SW8		0x0580
#define SCB_Q_SW9		0x0590
#define SCB_Q_SW10		0x05A0
#define SCB_Q_SW11		0x05B0
#define SCB_Q_SW12		0x05C0
#define SCB_Q_SW13		0x05D0
#define SCB_Q_SW14		0x05E0
#define SCB_Q_SW15		0x05F0
#define SCB_Q_CLOCK		0x0600
#define SCB_Q_INTER		0x0610
#define SCB_Q_SYSERR        	0x0620
#define SCB_Q_PROCERR		0x0630
#define SCB_Q_PWRFAIL		0x0640
#define SCB_Q_PERFMON		0x0650
#define SCB_Q_SYSMCHK		0x0660
#define SCB_Q_PROCMCHK      	0x0670
#define SCB_Q_PASSREL		0x0680

/*
**  Stack Frame (FRM) Offsets:
**
**  There are two types of system entries for OSF/1 - those for the
**  callsys CALL_PAL function and those for exceptions and interrupts.
**  Both entry types use the same stack frame layout.  The stack frame
**  contains space for the PC, the PS, the saved GP, and the saved
**  argument registers a0, a1, and a2.  On entry, SP points to the
**  saved PS.
*/

#define	FRM_Q_PS	0x0000
#define FRM_Q_PC	0x0008
#define FRM_Q_GP	0x0010
#define FRM_Q_A0	0x0018
#define FRM_Q_A1	0x0020
#define FRM_Q_A2	0x0028

#define FRM_K_SIZE	48

#define STACK_FRAME(tmp1,tmp2)	\
        sll	ps, 63-PS_V_CM, p7;	\
        bge	p7, 0f;			\
        bis	zero, zero, ps;		\
        mtpr	sp, ptUsp;		\
        mfpr	sp, ptKsp;		\
0:	lda	sp, 0-FRM_K_SIZE(sp);	\
        stq	tmp1, FRM_Q_PS(sp);	\
        stq	tmp2, FRM_Q_PC(sp);	\
        stq	gp, FRM_Q_GP(sp);	\
        stq	a0, FRM_Q_A0(sp);	\
        stq	a1, FRM_Q_A1(sp);	\
        stq	a2, FRM_Q_A2(sp)

/*
**  Halt Codes:
*/

#define HLT_K_RESET	    0x0000
#define HLT_K_HW_HALT	    0x0001
#define HLT_K_KSP_INVAL	    0x0002
#define HLT_K_SCBB_INVAL    0x0003
#define HLT_K_PTBR_INVAL    0x0004
#define HLT_K_SW_HALT	    0x0005
#define HLT_K_DBL_MCHK	    0x0006
#define HLT_K_MCHK_FROM_PAL 0x0007

/*
**  Machine Check Codes:
*/

#define MCHK_K_TPERR	    0x0080
#define MCHK_K_TCPERR	    0x0082
#define MCHK_K_HERR	    0x0084
#define MCHK_K_ECC_C	    0x0086
#define MCHK_K_ECC_NC	    0x0088
#define MCHK_K_UNKNOWN	    0x008A
#define MCHK_K_CACKSOFT	    0x008C
#define MCHK_K_BUGCHECK	    0x008E
#define MCHK_K_OS_BUGCHECK  0x0090
#define MCHK_K_DCPERR	    0x0092
#define MCHK_K_ICPERR	    0x0094
#define MCHK_K_RETRY_IRD    0x0096
#define MCHK_K_PROC_HERR    0x0098

/*
** System Machine Check Codes:
*/

#define MCHK_K_READ_NXM     0x0200
#define MCHK_K_SYS_HERR     0x0202

/*
**  Machine Check Error Status Summary (MCES) Register Format
**
**	 Extent	Size	Name	Function
**	 ------	----	----	---------------------------------
**	  <0>	  1	MIP	Machine check in progress
**	  <1>	  1	SCE	System correctable error in progress
**	  <2>	  1	PCE	Processor correctable error in progress
**	  <3>	  1	DPC	Disable PCE error reporting
**	  <4>	  1	DSC	Disable SCE error reporting
*/

#define MCES_V_MIP	0
#define MCES_M_MIP	(1<<MCES_V_MIP)
#define MCES_V_SCE	1
#define MCES_M_SCE	(1<<MCES_V_SCE)
#define MCES_V_PCE	2
#define MCES_M_PCE	(1<<MCES_V_PCE)
#define MCES_V_DPC	3
#define MCES_M_DPC	(1<<MCES_V_DPC)
#define MCES_V_DSC	4
#define MCES_M_DSC	(1<<MCES_V_DSC)

#define MCES_M_ALL      (MCES_M_MIP | MCES_M_SCE | MCES_M_PCE | MCES_M_DPC \
                         | MCES_M_DSC)

/*
**  Who-Am-I (WHAMI) Register Format
**
**	 Extent	Size	Name	Function
**	 ------	----	----	---------------------------------
**	  <7:0>	  8	ID	Who-Am-I identifier
**	  <15:8>   1	SWAP	Swap PALcode flag - character 'S'
*/

#define WHAMI_V_SWAP	8
#define WHAMI_M_SWAP	(1<<WHAMI_V_SWAP)
#define WHAMI_V_ID	0
#define WHAMI_M_ID	0xFF

#define WHAMI_K_SWAP    0x53    /* Character 'S' */

/*
**  Conventional Register Usage Definitions
**
**  Assembler temporary `at' is `AT' so it doesn't conflict with the
**  `.set at' assembler directive.
*/

#define v0		$0	/* Function Return Value Register */
#define t0		$1	/* Scratch (Temporary) Registers ... */
#define t1		$2
#define t2		$3
#define t3		$4
#define t4		$5
#define t5		$6
#define t6		$7
#define t7		$8
#define s0		$9	/* Saved (Non-Volatile) Registers ... */
#define s1		$10
#define s2		$11
#define s3		$12
#define s4		$13
#define s5		$14
#define fp		$15	/* Frame Pointer Register, Or S6 */
#define s6		$15
#define a0		$16	/* Argument Registers ... */
#define a1		$17
#define a2		$18
#define a3		$19
#define a4		$20
#define a5		$21
#define t8		$22	/* Scratch (Temporary) Registers ... */
#define t9		$23
#define t10		$24
#define t11		$25
#define ra		$26	/* Return Address Register */
#define pv		$27	/* Procedure Value Register, Or T12 */
#define t12		$27
#define AT		$28	/* Assembler Temporary (Volatile) Register */
#define gp		$29	/* Global Pointer Register */
#define sp		$30	/* Stack Pointer Register */
#define zero		$31	/* Zero Register */

/*
**  OSF/1 Unprivileged CALL_PAL Entry Offsets:
**
**	Entry Name	    Offset (Hex)
**
**	bpt		     0080
**	bugchk		     0081
**	callsys		     0083
**	imb		     0086
**	rdunique	     009E
**	wrunique	     009F
**	gentrap		     00AA
**	dbgstop		     00AD
*/

#define UNPRIV			    0x80
#define	PAL_BPT_ENTRY		    0x80
#define PAL_BUGCHK_ENTRY	    0x81
#define PAL_CALLSYS_ENTRY	    0x83
#define PAL_IMB_ENTRY		    0x86
#define PAL_RDUNIQUE_ENTRY	    0x9E
#define PAL_WRUNIQUE_ENTRY	    0x9F
#define PAL_GENTRAP_ENTRY	    0xAA

#if defined(KDEBUG)
#define	PAL_DBGSTOP_ENTRY	    0xAD
/* #define NUM_UNPRIV_CALL_PALS	    10 */
#else
/* #define NUM_UNPRIV_CALL_PALS	    9  */
#endif /* KDEBUG */

/*
**  OSF/1 Privileged CALL_PAL Entry Offsets:
**
**	Entry Name	    Offset (Hex)
**
**	halt		     0000
**	cflush		     0001
**	draina		     0002
**	cserve		     0009
**	swppal		     000A
**	rdmces		     0010
**	wrmces		     0011
**	wrfen		     002B
**	wrvptptr	     002D
**	swpctx		     0030
**	wrval		     0031
**	rdval		     0032
**	tbi		     0033
**	wrent		     0034
**	swpipl		     0035
**	rdps		     0036
**	wrkgp		     0037
**	wrusp		     0038
**	rdusp		     003A
**	whami		     003C
**	retsys		     003D
**	rti		     003F
*/

#define PAL_HALT_ENTRY	    0x0000
#define PAL_CFLUSH_ENTRY    0x0001
#define PAL_DRAINA_ENTRY    0x0002
#define PAL_CSERVE_ENTRY    0x0009
#define PAL_SWPPAL_ENTRY    0x000A
#define PAL_WRIPIR_ENTRY    0x000D
#define PAL_RDMCES_ENTRY    0x0010
#define PAL_WRMCES_ENTRY    0x0011
#define PAL_WRFEN_ENTRY	    0x002B
#define PAL_WRVPTPTR_ENTRY  0x002D
#define PAL_SWPCTX_ENTRY    0x0030
#define PAL_WRVAL_ENTRY	    0x0031
#define PAL_RDVAL_ENTRY	    0x0032
#define PAL_TBI_ENTRY	    0x0033
#define PAL_WRENT_ENTRY	    0x0034
#define PAL_SWPIPL_ENTRY    0x0035
#define PAL_RDPS_ENTRY	    0x0036
#define PAL_WRKGP_ENTRY	    0x0037
#define PAL_WRUSP_ENTRY	    0x0038
#define PAL_RDUSP_ENTRY	    0x003A
#define PAL_WHAMI_ENTRY	    0x003C
#define PAL_RETSYS_ENTRY    0x003D
#define PAL_RTI_ENTRY	    0x003F

#define NUM_PRIV_CALL_PALS  23

#endif

