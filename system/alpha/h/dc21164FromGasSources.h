/*
 * Copyright (c) 1993 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DC21164FROMGASSOURCES_INCLUDED
#define	DC21164FROMGASSOURCES_INCLUDED	1

/*
**
**  INTERNAL PROCESSOR REGISTER DEFINITIONS
**
**  The internal processor register definitions below are annotated
**  with one of the following symbols:
**
**	RW - The register may be read and written
**	RO - The register may only be read
**	WO - The register may only be written
**
**  For RO and WO registers, all bits and fields within the register are
**  also read-only or write-only.  For RW registers, each bit or field
**  within the register is annotated with one of the following:
**
**	RW  - The bit/field may be read and written
** 	RO  - The bit/field may be read; writes are ignored
**	WO  - The bit/field may be written; reads return UNPREDICTABLE
**	WZ  - The bit/field may be written; reads return a zero value
**	W0C - The bit/field may be read; write-zero-to-clear
**	W1C - The bit/field may be read; write-one-to-clear
**	WA  - The bit/field may be read; write-anything-to-clear
**	RC  - The bit/field may be read, causing state to clear;
**	      writes are ignored
**
*/


/*
**
**  Ibox IPR Definitions:
**
*/

// replaced by ev5_defs.h #define isr		0x100	/* RO - Interrupt Summary */
#define itbTag		0x101	/* WO - ITB Tag */
#define	itbPte		0x102	/* RW - ITB Page Table Entry */
#define itbAsn		0x103	/* RW - ITB Address Space Number */
#define itbPteTemp	0x104	/* RO - ITB Page Table Entry Temporary */
#define	itbIa		0x105	/* WO - ITB Invalidate All */
#define itbIap		0x106	/* WO - ITB Invalidate All Process */
#define itbIs		0x107	/* WO - ITB Invalidate Single */
// replaced by ev5_defs.h #define sirr		0x108	/* RW - Software Interrupt Request */
// replaced by ev5_defs.h #define astrr		0x109	/* RW - Async. System Trap Request */
// replaced by ev5_defs.h #define aster		0x10A	/* RW - Async. System Trap Enable */
#define excAddr		0x10B	/* RW - Exception Address */
#define excSum		0x10C	/* RW - Exception Summary */
#define excMask		0x10D	/* RO - Exception Mask */
#define palBase		0x10E	/* RW - PAL Base */
#define ips		0x10F	/* RW - Processor Status */
// replaced by ev5_defs.h #define ipl		0x110	/* RW - Interrupt Priority Level */
#define intId		0x111	/* RO - Interrupt ID */
#define iFaultVaForm	0x112	/* RO - Formatted Faulting VA */
#define iVptBr		0x113	/* RW - I-Stream Virtual Page Table Base */
#define hwIntClr	0x115	/* WO - Hardware Interrupt Clear */
#define slXmit		0x116	/* WO - Serial Line Transmit */
#define slRcv		0x117	/* RO - Serial Line Receive */
// replaced by ev5_defs.h #define icsr		0x118	/* RW - Ibox Control/Status */
#define icFlush		0x119	/* WO - I-Cache Flush Control */
#define flushIc         0x119   /* WO - I-Cache Flush Control (DC21064 Symbol) */
#define icPerr		0x11A	/* RW - I-Cache Parity Error Status */
#define PmCtr		0x11C	/* RW - Performance Counter */

/*
**
**  Ibox Control/Status Register (ICSR) Bit Summary
**
**	Extent	Size	Name	Type	Function
**	------	----	----	----	------------------------------------
**	 <39>	 1	TST	RW,0	Assert Test Status
**	 <38>	 1	ISTA	RO	I-Cache BIST Status
**	 <37>	 1	DBS	RW,1	Debug Port Select
**	 <36>	 1	FBD	RW,0	Force Bad I-Cache Data Parity
**	 <35>	 1	FBT	RW,0	Force Bad I-Cache Tag Parity
**	 <34>	 1	FMS	RW,0	Force I-Cache Miss
**	 <33>	 1	SLE	RW,0	Enable Serial Line Interrupts
**	 <32>	 1	CRDE	RW,0	Enable Correctable Error Interrupts
**	 <30>	 1	SDE	RW,0	Enable PAL Shadow Registers
**	<29:28>	 2	SPE	RW,0	Enable I-Stream Super Page Mode
**	 <27>	 1	HWE	RW,0	Enable PALRES Instrs in Kernel Mode
**	 <26>	 1	FPE	RW,0	Enable Floating Point Instructions
**	 <25>	 1	TMD	RW,0	Disable Ibox Timeout Counter
**	 <24>	 1	TMM	RW,0	Timeout Counter Mode
**
*/

#define ICSR_V_TST	39
#define ICSR_M_TST	(1<<ICSR_V_TST)
#define ICSR_V_ISTA	38
#define ICSR_M_ISTA	(1<<ICSR_V_ISTA)
#define ICSR_V_DBS	37
#define ICSR_M_DBS	(1<<ICSR_V_DBS)
#define ICSR_V_FBD	36
#define ICSR_M_FBD	(1<<ICSR_V_FBD)
#define ICSR_V_FBT	35
#define	ICSR_M_FBT	(1<<ICSR_V_FBT)
#define ICSR_V_FMS	34
#define ICSR_M_FMS	(1<<ICSR_V_FMS)
#define	ICSR_V_SLE	33
#define ICSR_M_SLE	(1<<ICSR_V_SLE)
#define ICSR_V_CRDE	32
#define ICSR_M_CRDE	(1<<ICSR_V_CRDE)
#define ICSR_V_SDE	30
#define ICSR_M_SDE	(1<<ICSR_V_SDE)
#define ICSR_V_SPE	28
#define ICSR_M_SPE	(3<<ICSR_V_SPE)
#define ICSR_V_HWE	27
#define ICSR_M_HWE	(1<<ICSR_V_HWE)
#define ICSR_V_FPE	26
#define ICSR_M_FPE	(1<<ICSR_V_FPE)
#define ICSR_V_TMD	25
#define ICSR_M_TMD	(1<<ICSR_V_TMD)
#define ICSR_V_TMM	24
#define ICSR_M_TMM	(1<<ICSR_V_TMM)

/*
**
**  Serial Line Tranmit Register (SL_XMIT)
**
**	Extent	Size	Name	Type	Function
**	------	----	----	----	------------------------------------
**	 <7>	 1	TMT	WO,1	Serial line transmit data
**
*/

#define	SLXMIT_V_TMT   	7
#define SLXMIT_M_TMT	(1<<SLXMIT_V_TMT)

/*
**
**  Serial Line Receive Register (SL_RCV)
**
**	Extent	Size	Name	Type	Function
**	------	----	----	----	------------------------------------
**	 <6>	 1	RCV	RO	Serial line receive data
**
*/

#define	SLRCV_V_RCV   	6
#define SLRCV_M_RCV	(1<<SLRCV_V_RCV)

/*
**
**  Icache Parity Error Status Register (ICPERR) Bit Summary
**
**	Extent	Size	Name	Type	Function
**	------	----	----	----	------------------------------------
**	 <13>	 1	TMR	W1C	Timeout reset error
**	 <12>	 1	TPE	W1C	Tag parity error
**	 <11>	 1	DPE	W1C	Data parity error
**
*/

#define	ICPERR_V_TMR   	13
#define ICPERR_M_TMR	(1<<ICPERR_V_TMR)
#define ICPERR_V_TPE	12
#define ICPERR_M_TPE	(1<<ICPERR_V_TPE)
#define ICPERR_V_DPE	11
#define ICPERR_M_DPE	(1<<ICPERR_V_DPE)

#define ICPERR_M_ALL	(ICPERR_M_TMR | ICPERR_M_TPE | ICPERR_M_DPE)

/*
**
**  Exception Summary Register (EXC_SUM) Bit Summary
**
**	Extent	Size	Name	Type	Function
**	------	----	----	----	------------------------------------
**	 <16>	 1	IOV	 WA	Integer overflow
**	 <15>	 1	INE	 WA	Inexact result
**	 <14>	 1	UNF	 WA	Underflow
**	 <13>	 1	FOV	 WA	Overflow
**	 <12>	 1	DZE	 WA	Division by zero
**	 <11>	 1	INV	 WA	Invalid operation
**	 <10>	 1	SWC	 WA	Software completion
**
*/

#define EXC_V_IOV	16
#define EXC_M_IOV	(1<<EXC_V_IOV)
#define EXC_V_INE	15
#define EXC_M_INE	(1<<EXC_V_INE)
#define EXC_V_UNF	14
#define EXC_M_UNF	(1<<EXC_V_UNF)
#define EXC_V_FOV	13
#define EXC_M_FOV	(1<<EXC_V_FOV)
#define EXC_V_DZE	12
#define	EXC_M_DZE	(1<<EXC_V_DZE)
#define EXC_V_INV	11
#define EXC_M_INV	(1<<EXC_V_INV)
#define	EXC_V_SWC	10
#define EXC_M_SWC	(1<<EXC_V_SWC)

/*
**
**  Hardware Interrupt Clear Register (HWINT_CLR) Bit Summary
**
**	 Extent	Size	Name	Type	Function
**	 ------	----	----	----	---------------------------------
**	  <33>	  1	SLC	W1C	Clear Serial Line interrupt
**	  <32>	  1	CRDC	W1C	Clear Correctable Read Data interrupt
**	  <29>	  1	PC2C	W1C	Clear Performance Counter 2 interrupt
**	  <28>	  1	PC1C	W1C	Clear Performance Counter 1 interrupt
**	  <27>	  1	PC0C    W1C	Clear Performance Counter 0 interrupt
**
*/

#define HWINT_V_SLC	33
#define HWINT_M_SLC	(1<<HWINT_V_SLC)
#define HWINT_V_CRDC	32
#define HWINT_M_CRDC	(1<<HWINT_V_CRDC)
#define HWINT_V_PC2C	29
#define HWINT_M_PC2C	(1<<HWINT_V_PC2C)
#define HWINT_V_PC1C	28
#define HWINT_M_PC1C	(1<<HWINT_V_PC1C)
#define HWINT_V_PC0C	27
#define HWINT_M_PC0C	(1<<HWINT_V_PC0C)

/*
**
**  Interrupt Summary Register (ISR) Bit Summary
**
**	 Extent	Size	Name	Type	Function
**	 ------	----	----	----	---------------------------------
**	  <34>	  1	HLT    	RO	External Halt interrupt
**	  <33>	  1	SLI	RO	Serial Line interrupt
**	  <32>	  1	CRD	RO	Correctable ECC errors
**	  <31>	  1	MCK	RO	System Machine Check
**	  <30>	  1	PFL	RO	Power Fail
**	  <29>	  1	PC2	RO	Performance Counter 2 interrupt
**	  <28>	  1	PC1	RO	Performance Counter 1 interrupt
**	  <27>	  1	PC0	RO	Performance Counter 0 interrupt
**	  <23>	  1	I23	RO	External Hardware interrupt
**	  <22>	  1	I22	RO	External Hardware interrupt
**	  <21>	  1	I21	RO	External Hardware interrupt
**	  <20>	  1	I20	RO	External Hardware interrupt
**	  <19>	  1	ATR	RO	Async. System Trap request
**	 <18:4>	 15	SIRR	RO,0	Software Interrupt request
**	  <3:0>	  4	ASTRR	RO	Async. System Trap request (USEK)
**
**/

#define ISR_V_HLT	34
#define ISR_M_HLT	(1<<ISR_V_HLT)
#define ISR_V_SLI	33
#define ISR_M_SLI	(1<<ISR_V_SLI)
#define ISR_V_CRD	32
#define ISR_M_CRD	(1<<ISR_V_CRD)
#define ISR_V_MCK	31
#define ISR_M_MCK	(1<<ISR_V_MCK)
#define ISR_V_PFL	30
#define ISR_M_PFL	(1<<ISR_V_PFL)
#define ISR_V_PC2	29
#define ISR_M_PC2	(1<<ISR_V_PC2)
#define ISR_V_PC1	28
#define ISR_M_PC1	(1<<ISR_V_PC1)
#define ISR_V_PC0	27
#define ISR_M_PC0	(1<<ISR_V_PC0)
#define ISR_V_I23	23
#define ISR_M_I23	(1<<ISR_V_I23)
#define ISR_V_I22	22
#define ISR_M_I22	(1<<ISR_V_I22)
#define ISR_V_I21	21
#define ISR_M_I21	(1<<ISR_V_I21)
#define ISR_V_I20	20
#define ISR_M_I20	(1<<ISR_V_I20)
#define ISR_V_ATR	19
#define ISR_M_ATR	(1<<ISR_V_ATR)
#define ISR_V_SIRR	4
#define ISR_M_SIRR	(0x7FFF<<ISR_V_SIRR)
#define ISR_V_ASTRR	0
#define ISR_M_ASTRR	(0xF<<ISR_V_ASTRR)

/*
**
**  Mbox and D-Cache IPR Definitions:
**
*/

#define dtbAsn		0x200	/* WO - DTB Address Space Number */
#define dtbCm		0x201	/* WO - DTB Current Mode */
#define dtbTag		0x202	/* WO - DTB Tag */
#define dtbPte		0x203	/* RW - DTB Page Table Entry */
#define dtbPteTemp	0x204	/* RO - DTB Page Table Entry Temporary */
#define mmStat		0x205	/* RO - D-Stream MM Fault Status */
// replaced by ev5_defs.h #define va		0x206	/* RO - Faulting Virtual Address */
#define vaForm		0x207	/* RO - Formatted Virtual Address */
#define mVptBr		0x208	/* WO - Mbox Virtual Page Table Base */
#define dtbIap		0x209	/* WO - DTB Invalidate All Process */
#define dtbIa		0x20A	/* WO - DTB Invalidate All */
#define dtbIs		0x20B	/* WO - DTB Invalidate Single */
#define altMode		0x20C	/* WO - Alternate Mode */
// replaced by ev5_defs.h #define cc		0x20D	/* WO - Cycle Counter */
#define ccCtl		0x20E	/* WO - Cycle Counter Control */
// replaced by ev5_defs.h #define mcsr		0x20F	/* RW - Mbox Control Register */
#define dcFlush		0x210	/* WO - Dcache Flush */
#define dcPerr	        0x212	/* RW - Dcache Parity Error Status */
#define dcTestCtl	0x213	/* RW - Dcache Test Tag Control */
#define dcTestTag	0x214	/* RW - Dcache Test Tag */
#define dcTestTagTemp	0x215	/* RW - Dcache Test Tag Temporary */
#define dcMode		0x216	/* RW - Dcache Mode */
#define mafMode		0x217	/* RW - Miss Address File Mode */

/*
**
**  D-Stream MM Fault Status Register (MM_STAT) Bit Summary
**
**	 Extent	Size	Name	  Type	Function
**	 ------	----	----	  ----	---------------------------------
**	<16:11>	  6	OPCODE 	  RO	Opcode of faulting instruction
**	<10:06>	  5	RA	  RO	Ra field of faulting instruction
**          <5>	  1	BAD_VA	  RO	Bad virtual address
**	    <4>	  1	DTB_MISS  RO	Reference resulted in DTB miss
**	    <3>	  1	FOW	  RO	Fault on write
**	    <2>	  1	FOR	  RO	Fault on read
**	    <1>   1     ACV	  RO	Access violation
**          <0>	  1	WR	  RO	Reference type
**
*/

#define	MMSTAT_V_OPC		11
#define MMSTAT_M_OPC		(0x3F<<MMSTAT_V_OPC)
#define MMSTAT_V_RA		6
#define MMSTAT_M_RA		(0x1F<<MMSTAT_V_RA)
#define MMSTAT_V_BAD_VA		5
#define MMSTAT_M_BAD_VA		(1<<MMSTAT_V_BAD_VA)
#define MMSTAT_V_DTB_MISS	4
#define MMSTAT_M_DTB_MISS	(1<<MMSTAT_V_DTB_MISS)
#define MMSTAT_V_FOW		3
#define MMSTAT_M_FOW		(1<<MMSTAT_V_FOW)
#define MMSTAT_V_FOR		2
#define MMSTAT_M_FOR		(1<<MMSTAT_V_FOR)
#define MMSTAT_V_ACV		1
#define MMSTAT_M_ACV		(1<<MMSTAT_V_ACV)
#define MMSTAT_V_WR		0
#define MMSTAT_M_WR		(1<<MMSTAT_V_WR)


/*
**
** Mbox Control Register (MCSR) Bit Summary
**
**	 Extent	Size	Name	Type	Function
**	 ------	----	----	----	---------------------------------
**	   <5>	  1	DBG1	RW,0   	Mbox Debug Packet Select
**	   <4>	  1	E_BE	RW,0	Ebox Big Endian mode enable
**	   <3>	  1	DBG0	RW,0	Debug Test Select
**	  <2:1>	  2	SP	RW,0   	Superpage mode enable
**	   <0>	  1	M_BE	RW,0    Mbox Big Endian mode enable
**
*/

#define MCSR_V_DBG1	5
#define MCSR_M_DBG1	(1<<MCSR_V_DBG1)
#define MCSR_V_E_BE	4
#define MCSR_M_E_BE	(1<<MCSR_V_E_BE)
#define MCSR_V_DBG0	3
#define MCSR_M_DBG0	(1<<MCSR_V_DBG0)
#define MCSR_V_SP	1
#define MCSR_M_SP	(3<<MCSR_V_SP)
#define MCSR_V_M_BE	0
#define MCSR_M_M_BE	(1<<MCSR_V_M_BE)

/*
**
**  Dcache Parity Error Status Register (DCPERR) Bit Summary
**
**	Extent	Size	Name	Type	Function
**	------	----	----	----	------------------------------------
**	 <5>	 1	TP1	RO	Dcache bank 1 tag parity error
**	 <4>	 1	TP0	RO	Dcache bank 0 tag parity error
**	 <3>	 1	DP1	RO	Dcache bank 1 data parity error
**	 <2>	 1	DP0	RO	Dcache bank 0 data parity error
**	 <1>	 1	LOCK	W1C	Locks/clears bits <5:2>
**	 <0>	 1	SEO	W1C	Second Dcache parity error occurred
**
*/

#define DCPERR_V_TP1	5
#define DCPERR_M_TP1	(1<<DCPERR_V_TP1)
#define	DCPERR_V_TP0   	4
#define DCPERR_M_TP0	(1<<DCPERR_V_TP0)
#define DCPERR_V_DP1	3
#define DCPERR_M_DP1	(1<<DCPERR_V_DP1)
#define DCPERR_V_DP0    2
#define DCPERR_M_DP0	(1<<DCPERR_V_DP0)
#define DCPERR_V_LOCK	1
#define DCPERR_M_LOCK	(1<<DCPERR_V_LOCK)
#define DCPERR_V_SEO	0
#define DCPERR_M_SEO	(1<<DCPERR_V_SEO)

#define DCPERR_M_ALL	(DCPERR_M_LOCK | DCPERR_M_SEO)

/*
**
**  Dcache Mode Register (DC_MODE) Bit Summary
**
**	 Extent	Size	Name	  Type	Function
**	 ------	----	----	  ----	---------------------------------
**	   <4>	  1	DOA	  RO    Hardware Dcache Disable
**	   <3>	  1	PERR_DIS  RW,0	Disable Dcache Parity Error reporting
**	   <2>	  1	BAD_DP	  RW,0	Force Dcache data bad parity
**	   <1>	  1	FHIT	  RW,0	Force Dcache hit
**	   <0>	  1	ENA 	  RW,0	Software Dcache Enable
**
*/

#define	DC_V_DOA	4
#define DC_M_DOA        (1<<DC_V_DOA)
#define DC_V_PERR_DIS	3
#define DC_M_PERR_DIS	(1<<DC_V_PERR_DIS)
#define DC_V_BAD_DP	2
#define DC_M_BAD_DP	(1<<DC_V_BAD_DP)
#define DC_V_FHIT	1
#define DC_M_FHIT	(1<<DC_V_FHIT)
#define DC_V_ENA	0
#define DC_M_ENA	(1<<DC_V_ENA)

/*
**
**  Miss Address File Mode Register (MAF_MODE) Bit Summay
**
**	 Extent	Size	Name	  Type	Function
**	 ------	----	----	  ----	---------------------------------
**         <7>    1     WB        RO,0  If set, pending WB request
**	   <6>	  1	DREAD	  RO,0  If set, pending D-read request
**
*/

#define MAF_V_WB_PENDING        7
#define MAF_M_WB_PENDING        (1<<MAF_V_WB_PENDING)
#define MAF_V_DREAD_PENDING     6
#define MAF_M_DREAD_PENDING     (1<<MAF_V_DREAD_PENDING)

/*
**
**  Cbox IPR Definitions:
**
*/

#define scCtl		0x0A8	/* RW - Scache Control */
#define scStat		0x0E8	/* RO - Scache Error Status */
#define scAddr		0x188	/* RO - Scache Error Address */
#define	bcCtl		0x128	/* WO - Bcache/System Interface Control */
#define bcCfg		0x1C8	/* WO - Bcache Configuration Parameters */
#define bcTagAddr	0x108	/* RO - Bcache Tag */
#define eiStat		0x168	/* RO - Bcache/System Error Status */
#define eiAddr		0x148	/* RO - Bcache/System Error Address */
#define fillSyn		0x068	/* RO - Fill Syndrome */
#define ldLock		0x1E8	/* RO - LDx_L Address */

/*
**
**  Scache Control Register (SC_CTL) Bit Summary
**
**	 Extent	Size	Name	  Type	Function
**	 ------	----	----	  ----	---------------------------------
**	 <15:13>  3	SET_EN	  RW,1  Set enable
**	    <12>  1	BLK_SIZE  RW,1	Scache/Bcache block size select
**	 <11:08>  4	FB_DP	  RW,0	Force bad data parity
**	 <07:02>  6	TAG_STAT  RW	Tag status and parity
**	     <1>  1	FLUSH	  RW,0	If set, clear all tag valid bits
**	     <0>  1     FHIT	  RW,0  Force hits
**
*/

#define	SC_V_SET_EN	13
#define SC_M_SET_EN	(7<<SC_V_SET_EN)
#define SC_V_BLK_SIZE	12
#define SC_M_BLK_SIZE	(1<<SC_V_BLK_SIZE)
#define SC_V_FB_DP	8
#define SC_M_FB_DP	(0xF<<SC_V_FB_DP)
#define SC_V_TAG_STAT	2
#define SC_M_TAG_STAT	(0x3F<<SC_V_TAG_STAT)
#define SC_V_FLUSH	1
#define SC_M_FLUSH	(1<<SC_V_FLUSH)
#define SC_V_FHIT	0
#define SC_M_FHIT	(1<<SC_V_FHIT)

/*
**
**  Bcache Control Register (BC_CTL) Bit Summary
**
**	 Extent	Size  Name	    Type  Function
**	 ------	----  ----	    ----  ---------------------------------
**	    <27>  1   DIS_VIC_BUF   WO,0  Disable Scache victim buffer
**	    <26>  1   DIS_BAF_BYP   WO,0  Disable speculative Bcache reads
**	    <25>  1   DBG_MUX_SEL   WO,0  Debug MUX select
**	 <24:19>  6   PM_MUX_SEL    WO,0  Performance counter MUX select
**       <18:17>  2   BC_WAVE       WO,0  Number of cycles of wave pipelining
**	    <16>  1   TL_PIPE_LATCH WO,0  Pipe system control pins
**	    <15>  1   EI_DIS_ERR    WO,1  Disable ECC (parity) error
**       <14:13>  2   BC_BAD_DAT    WO,0  Force bad data
**       <12:08>  5   BC_TAG_STAT   WO    Bcache tag status and parity
**           <7>  1   BC_FHIT       WO,0  Bcache force hit
**           <6>  1   EI_ECC        WO,1  ECC or byte parity mode
**           <5>  1   VTM_FIRST     WO,1  Drive out victim block address first
**           <4>  1   CORR_FILL_DAT WO,1  Correct fill data
**           <3>  1   EI_CMD_GRP3   WO,0  Drive MB command to external pins
**           <2>  1   EI_CMD_GRP2   WO,0  Drive LOCK & SET_DIRTY to ext. pins
**           <1>  1   ALLOC_CYC     WO,0  Allocate cycle for non-cached LDs.
**           <0>  1   BC_ENA        W0,0  Bcache enable
**
*/
#define BC_V_DIS_SC_VIC_BUF	27
#define BC_M_DIS_SC_VIC_BUF	(1<<BC_V_DIS_SC_VIC_BUF)
#define BC_V_DIS_BAF_BYP	26
#define BC_M_DIS_BAF_BYP	(1<<BC_V_DIS_BAF_BYP)
#define BC_V_DBG_MUX_SEL	25
#define BC_M_DBG_MUX_SEL	(1<<BC_V_DBG_MUX_SEL)
#define BC_V_PM_MUX_SEL		19
#define BC_M_PM_MUX_SEL		(0x3F<<BC_V_PM_MUX_SEL)
#define BC_V_BC_WAVE		17
#define BC_M_BC_WAVE		(3<<BC_V_BC_WAVE)
#define BC_V_TL_PIPE_LATCH	16
#define BC_M_TL_PIPE_LATCH	(1<<BC_V_TL_PIPE_LATCH)
#define BC_V_EI_DIS_ERR		15
#define BC_M_EI_DIS_ERR		(1<<BC_V_EI_DIS_ERR)
#define BC_V_BC_BAD_DAT		13
#define BC_M_BC_BAD_DAT		(3<<BC_V_BC_BAD_DAT)
#define BC_V_BC_TAG_STAT	8
#define BC_M_BC_TAG_STAT	(0x1F<<BC_V_BC_TAG_STAT)
#define BC_V_BC_FHIT		7
#define BC_M_BC_FHIT		(1<<BC_V_BC_FHIT)
#define BC_V_EI_ECC_OR_PARITY	6
#define BC_M_EI_ECC_OR_PARITY	(1<<BC_V_EI_ECC_OR_PARITY)
#define BC_V_VTM_FIRST		5
#define BC_M_VTM_FIRST		(1<<BC_V_VTM_FIRST)
#define BC_V_CORR_FILL_DAT	4
#define BC_M_CORR_FILL_DAT	(1<<BC_V_CORR_FILL_DAT)
#define BC_V_EI_CMD_GRP3	3
#define BC_M_EI_CMD_GRP3	(1<<BC_V_EI_CMD_GRP3)
#define BC_V_EI_CMD_GRP2	2
#define BC_M_EI_CMD_GRP2	(1<<BC_V_EI_CMD_GRP2)
#define BC_V_ALLOC_CYC		1
#define BC_M_ALLOC_CYC		(1<<BC_V_ALLOC_CYC)
#define BC_V_BC_ENA		0
#define BC_M_BC_ENA		(1<<BC_V_BC_ENA)

#define BC_K_DFAULT \
        (((BC_M_EI_DIS_ERR)       | \
          (BC_M_EI_ECC_OR_PARITY) | \
          (BC_M_VTM_FIRST)        | \
          (BC_M_CORR_FILL_DAT))>>1)
/*
**
**  Bcache Configuration Register (BC_CONFIG) Bit Summary
**
**	 Extent	Size  Name	    Type  Function
**	 ------	----  ----	    ----  ---------------------------------
**	<35:29>   7   RSVD	    WO    Reserved - Must Be Zero
**	<28:20>   9   WE_CTL        WO,0  Bcache write enable control
**	<19:19>   1   RSVD	    WO,0  Reserved - Must Be Zero
**	<18:16>   3   WE_OFF        WO,1  Bcache fill write enable pulse offset
**	<15:15>   1   RSVD          WO,0  Reserved - Must Be Zero
**	<14:12>   3   RD_WR_SPC     WO,7  Bcache private read/write spacing
**	<11:08>   4   WR_SPD        WO,4  Bcache write speed in CPU cycles
**	<07:04>   4   RD_SPD	    WO,4  Bcache read speed in CPU cycles
**	<03:03>   1   RSVD	    WO,0  Reserved - Must Be Zero
**	<02:00>   3   SIZE	    WO,1  Bcache size
*/
#define	BC_V_WE_CTL	20
#define BC_M_WE_CTL	(0x1FF<<BC_V_WE_CTL)
#define BC_V_WE_OFF	16
#define BC_M_WE_OFF	(0x7<<BC_V_WE_OFF)
#define BC_V_RD_WR_SPC	12
#define BC_M_RD_WR_SPC	(0x7<<BC_V_RD_WR_SPC)
#define BC_V_WR_SPD	8
#define BC_M_WR_SPD	(0xF<<BC_V_WR_SPD)
#define BC_V_RD_SPD	4
#define BC_M_RD_SPD	(0xF<<BC_V_RD_SPD)
#define BC_V_SIZE	0
#define BC_M_SIZE	(0x7<<BC_V_SIZE)

#define BC_K_CONFIG \
        ((0x1<<BC_V_WE_OFF)    | \
         (0x7<<BC_V_RD_WR_SPC) | \
         (0x4<<BC_V_WR_SPD)    | \
         (0x4<<BC_V_RD_SPD)    | \
         (0x1<<BC_V_SIZE))

/*
**
**  DECchip 21164 Privileged Architecture Library Entry Offsets:
**
**	Entry Name	    Offset (Hex)
**
**	RESET			0000
**	IACCVIO			0080
**	INTERRUPT	       	0100
**	ITB_MISS		0180
**	DTB_MISS (Single)       0200
**	DTB_MISS (Double)       0280
**	UNALIGN			0300
**	D_FAULT			0380
**	MCHK			0400
**	OPCDEC			0480
**	ARITH			0500
**	FEN			0580
**	CALL_PAL (Privileged)	2000
**	CALL_PAL (Unprivileged)	3000
**
*/

#define PAL_RESET_ENTRY		    0x0000
#define PAL_IACCVIO_ENTRY	    0x0080
#define PAL_INTERRUPT_ENTRY	    0x0100
#define PAL_ITB_MISS_ENTRY	    0x0180
#define PAL_DTB_MISS_ENTRY	    0x0200
#define PAL_DOUBLE_MISS_ENTRY	    0x0280
#define PAL_UNALIGN_ENTRY	    0x0300
#define PAL_D_FAULT_ENTRY	    0x0380
#define PAL_MCHK_ENTRY		    0x0400
#define PAL_OPCDEC_ENTRY	    0x0480
#define PAL_ARITH_ENTRY	    	    0x0500
#define PAL_FEN_ENTRY		    0x0580
#define PAL_CALL_PAL_PRIV_ENTRY	    0x2000
#define PAL_CALL_PAL_UNPRIV_ENTRY   0x3000

/*
**
** Architecturally Reserved Opcode (PALRES) Definitions:
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
** Virtual PTE fetch variants of HW_LD.
*/
#define ld_vpte     hw_ldq/v

/*
** Physical mode load-lock and store-conditional variants of
** HW_LD and HW_ST.
*/

#define ldq_lp	    hw_ldq/pl
#define stq_cp	    hw_stq/pc

/*
**
**  General Purpose Register Definitions:
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
** Floating Point Register Definitions:
**
*/

#define	f0		$f0
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
#define	f15		$f15
#define	f16		$f16
#define	f17		$f17
#define	f18		$f18
#define	f19		$f19
#define	f20		$f20
#define	f21		$f21
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
**  PAL Temporary Register Definitions:
**
*/

// covered by fetch distribution..pb Nov/95

// #define	pt0		0x140
// #define	pt1		0x141
// #define	pt2		0x142
// #define	pt3		0x143
// #define	pt4		0x144
// #define	pt5		0x145
// #define	pt6		0x146
// #define	pt7		0x147
// #define	pt8		0x148
// #define	pt9		0x149
// #define	pt10		0x14A
// #define	pt11		0x14B
// #define	pt12		0x14C
// #define	pt13		0x14D
// #define	pt14		0x14E
// #define	pt15		0x14F
// #define	pt16		0x150
// #define	pt17		0x151
// #define	pt18		0x152
// #define	pt19		0x153
// #define	pt20		0x154
// #define	pt21		0x155
// #define	pt22		0x156
// #define	pt23		0x157

/*
**  PAL Shadow Registers:
**
**  The DECchip 21164 shadows r8-r14 and r25 when in PALmode and
**  ICSR<SDE> = 1.
*/

#define	p0		r8	/* ITB/DTB Miss Scratch */
#define p1		r9	/* ITB/DTB Miss Scratch */
#define p2		r10	/* ITB/DTB Miss Scratch */
#define p3		r11
// #define ps		r11	/* Processor Status */
#define p4		r12	/* Local Scratch */
#define p5		r13	/* Local Scratch */
#define p6		r14	/* Local Scratch */
#define p7		r25	/* Local Scratch */

/*
** SRM Defined State Definitions:
*/

/*
**  This table is an accounting of the DECchip 21164 storage used to
**  implement the SRM defined state for OSF/1.
**
** 	IPR Name			Internal Storage
**      --------                        ----------------
**	Processor Status		ps, dtbCm, ipl, r11
**	Program Counter			Ibox
**	Interrupt Entry			ptEntInt
**	Arith Trap Entry		ptEntArith
**	MM Fault Entry			ptEntMM
**	Unaligned Access Entry		ptEntUna
**	Instruction Fault Entry		ptEntIF
**	Call System Entry		ptEntSys
**	User Stack Pointer		ptUsp
**	Kernel Stack Pointer		ptKsp
**	Kernel Global Pointer		ptKgp
**	System Value			ptSysVal
**	Page Table Base Register	ptPtbr
**	Virtual Page Table Base		iVptBr, mVptBr
**	Process Control Block Base	ptPcbb
**	Address Space Number		itbAsn, dtbAsn
**	Cycle Counter			cc, ccCtl
**	Float Point Enable		icsr
**	Lock Flag			Cbox/System
**	Unique				PCB
**	Who-Am-I			ptWhami
*/

#define ptEntUna	pt2	/* Unaligned Access Dispatch Entry */
#define ptImpure	pt3	/* Pointer To PAL Scratch Area */
#define ptEntIF		pt7	/* Instruction Fault Dispatch Entry */
#define ptIntMask	pt8	/* Interrupt Enable Mask */
#define ptEntSys	pt9	/* Call System Dispatch Entry */
#define ptTrap          pt11
#define ptEntInt	pt11	/* Hardware Interrupt Dispatch Entry */
#define ptEntArith	pt12	/* Arithmetic Trap Dispatch Entry */
#if defined(KDEBUG)
#define ptEntDbg	pt13	/* Kernel Debugger Dispatch Entry */
#endif /* KDEBUG */
#define ptMisc          pt16    /* Miscellaneous Flags */
#define ptWhami		pt16	/* Who-Am-I Register Pt16<15:8> */
#define ptMces		pt16	/* Machine Check Error Summary Pt16<4:0> */
#define ptSysVal	pt17	/* Per-Processor System Value */
#define ptUsp		pt18	/* User Stack Pointer */
#define ptKsp		pt19	/* Kernel Stack Pointer */
#define ptPtbr		pt20	/* Page Table Base Register */
#define ptEntMM		pt21	/* MM Fault Dispatch Entry */
#define ptKgp		pt22	/* Kernel Global Pointer */
#define ptPcbb		pt23	/* Process Control Block Base */

/*
**
**   Miscellaneous PAL State Flags (ptMisc) Bit Summary
**
**	 Extent	Size  Name	Function
**	 ------	----  ----	---------------------------------
**	 <55:48>  8   SWAP      Swap PALcode flag -- character 'S'
**	 <47:32> 16   MCHK      Machine Check Error code
**	 <31:16> 16   SCB       System Control Block vector
**	 <15:08>  8   WHAMI     Who-Am-I identifier
**       <04:00>  5   MCES      Machine Check Error Summary bits
**
*/

#define PT16_V_MCES	0
#define PT16_V_WHAMI	8
#define PT16_V_SCB	16
#define PT16_V_MCHK	32
#define PT16_V_SWAP	48

#endif /* DC21164FROMGASSOURCES_INCLUDED */
