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

#ifndef EV5_ALPHA_DEFS_INCLUDED
#define EV5_ALPHA_DEFS_INCLUDED 1

// from ev5_alpha_defs.mar from Lance's fetch directory
// Lower-caseified and $ signs removed ... pb Nov/95

//
// PS Layout - PS
//	Loc	Size	name 	function
//	------	------	______	-----------------------------------
//	<31:29>	3	SA	stack alignment
//	<31:13>	24	RES	Reserved MBZ
//	<12:8>	5	IPL	Priority level
//	<7>	1	VMM	Virtual Mach Monitor
//	<6:5>	2	RES	Reserved MBZ
//	<4:3>	2	CM	Current Mode
//	<2>	1	IP	Interrupt Pending
//	<1:0>	2	SW	Software bits
//

#define ps_v_sw		0
#define ps_m_sw		(3<<ps_v_sw)

#define ps_v_ip		2
#define ps_m_ip		(1<<ps_v_ip)

#define ps_v_cm		3
#define ps_m_cm		(3<<ps_v_cm)

#define ps_v_vmm	7
#define ps_m_vmm	(1<<ps_v_vmm)

#define ps_v_ipl	8
#define ps_m_ipl	(0x1f<<ps_v_ipl)

#define ps_v_sp		(0x38)
#define ps_m_sp		(0x3f<<ps_v_sp)


#define ps_c_kern	(0x00)
#define ps_c_exec	(0x08)
#define ps_c_supr	(0x10)
#define ps_c_user	(0x18)
#define ps_c_ipl0	(0x0000)
#define ps_c_ipl1	(0x0100)
#define ps_c_ipl2	(0x0200)
#define ps_c_ipl3	(0x0300)
#define ps_c_ipl4	(0x0400)
#define ps_c_ipl5	(0x0500)
#define ps_c_ipl6	(0x0600)
#define ps_c_ipl7	(0x0700)
#define ps_c_ipl8	(0x0800)
#define ps_c_ipl9	(0x0900)
#define ps_c_ipl10	(0x0A00)
#define ps_c_ipl11	(0x0B00)
#define ps_c_ipl12	(0x0C00)
#define ps_c_ipl13	(0x0D00)
#define ps_c_ipl14	(0x0E00)
#define ps_c_ipl15	(0x0F00)
#define ps_c_ipl16	(0x1000)
#define ps_c_ipl17	(0x1100)
#define ps_c_ipl18	(0x1200)
#define ps_c_ipl19	(0x1300)
#define ps_c_ipl20	(0x1400)
#define ps_c_ipl21	(0x1500)
#define ps_c_ipl22	(0x1600)
#define ps_c_ipl23	(0x1700)
#define ps_c_ipl24	(0x1800)
#define ps_c_ipl25	(0x1900)
#define ps_c_ipl26	(0x1A00)
#define ps_c_ipl27	(0x1B00)
#define ps_c_ipl28	(0x1C00)
#define ps_c_ipl29	(0x1D00)
#define ps_c_ipl30	(0x1E00)
#define ps_c_ipl31	(0x1F00)

//
// PTE layout - symbol prefix PTE_
//
//	Loc	Size	name 	function
//	------	------	------	-----------------------------------
//	<63:32>	32	PFN	Page Frame Number
//	<31:16>	16	SOFT	Bits reserved for software use
//	<15>	1	UWE	User write enable
//	<14>	1	SWE	Super write enable
//	<13>	1	EWE	Exec write enable
//	<12>	1	KWE	Kernel write enable
//	<11>	1	URE	User read enable
//	<10>	1	SRE	Super read enable
//	<9>	1	ERE	Exec read enable
//	<8>	1	KRE	Kernel read enable
//	<7:6>	2	RES	Reserved SBZ
//	<5>	1	HPF	Huge Page Flag
//	<4>	1	ASM	Wild card address space number match
//	<3>	1	FOE	Fault On execute
//	<2>	1	FOW	Fault On Write
//	<1>	1	FOR	Fault On Read
// 	<0>	1	V	valid bit
//

#define pte_v_pfn	32
#define pte_m_soft	(0xFFFF0000)
#define pte_v_soft	16
#define pte_m_uwe	(0x8000)
#define pte_v_uwe	15
#define pte_m_swe	(0x4000)
#define pte_v_swe	14
#define pte_m_ewe	(0x2000)
#define pte_v_ewe	13
#define pte_m_kwe	(0x1000)
#define pte_v_kwe	12
#define pte_m_ure	(0x0800)
#define pte_v_ure	11
#define pte_m_sre	(0x0400)
#define pte_v_sre	10
#define pte_m_ere	(0x0200)
#define pte_v_ere	 9
#define pte_m_kre	(0x0100)
#define pte_v_kre	 8
#define pte_m_hpf	(0x0020)
#define pte_v_hpf	5
#define pte_m_asm	(0x0010)
#define pte_v_asm	4
#define pte_m_foe	(0x0008)
#define pte_v_foe	3
#define pte_m_fow	(0x0004)
#define pte_v_fow	2
#define pte_m_for	(0x0002)
#define pte_v_for	1
#define pte_m_v		(0x0001)
#define pte_v_v		0

//
// VA layout - symbol prefix VA_
//
//	Loc	Size	name 	function
//	------	------	-------	-----------------------------------
//	<42:33>	10	SEG1	First seg table offset for mapping
//	<32:23>	10	SEG2	Second seg table offset for mapping
//	<22:13>	10	SEG3	Third seg table offset for mapping
//	<12:0>	13	OFFSET	Byte within page
//

#define va_m_offset	(0x000000001FFF)
#define va_v_offset	0
#define va_m_seg3	(0x0000007FE000)
#define va_v_seg3	13
#define va_m_seg2	(0x0001FF800000)
#define va_v_seg2	23
#define va_m_seg1	(0x7FE00000000)
#define va_v_seg1	33

//
//PRIVILEGED CONTEXT BLOCK (PCB)
//
#define pcb_q_ksp	0
#define pcb_q_esp	8
#define pcb_q_ssp	16
#define pcb_q_usp	24
#define pcb_q_ptbr	32
#define pcb_q_asn	40
#define pcb_q_ast	48
#define pcb_q_fen	56
#define pcb_q_cc	64
#define pcb_q_unq	72
#define pcb_q_sct	80

#define pcb_v_asten	0
#define pcb_m_asten	(0x0f<<pcb_v_asten)
#define pcb_v_astsr	4
#define pcb_m_astsr	(0x0f<<pcb_v_astsr)
#define pcb_v_dat	63
#define pcb_v_pme	62

//
// SYSTEM CONTROL BLOCK (SCB)
//

#define scb_v_fen		(0x0010)
#define scb_v_acv		(0x0080)
#define scb_v_tnv		(0x0090)
#define scb_v_for		(0x00A0)
#define scb_v_fow		(0x00B0)
#define scb_v_foe		(0x00C0)
#define scb_v_arith		(0x0200)
#define scb_v_kast		(0x0240)
#define scb_v_east		(0x0250)
#define scb_v_sast		(0x0260)
#define scb_v_uast		(0x0270)
#define scb_v_unalign		(0x0280)
#define scb_v_bpt		(0x0400)
#define scb_v_bugchk		(0x0410)
#define scb_v_opcdec		(0x0420)
#define scb_v_illpal		(0x0430)
#define scb_v_trap		(0x0440)
#define scb_v_chmk		(0x0480)
#define scb_v_chme		(0x0490)
#define scb_v_chms		(0x04A0)
#define scb_v_chmu		(0x04B0)
#define scb_v_sw0		(0x0500)
#define scb_v_sw1		(0x0510)
#define scb_v_sw2		(0x0520)
#define scb_v_sw3		(0x0530)
#define scb_v_sw4		(0x0540)
#define scb_v_sw5		(0x0550)
#define scb_v_sw6		(0x0560)
#define scb_v_sw7		(0x0570)
#define scb_v_sw8		(0x0580)
#define scb_v_sw9		(0x0590)
#define scb_v_sw10		(0x05A0)
#define scb_v_sw11		(0x05B0)
#define scb_v_sw12		(0x05C0)
#define scb_v_sw13		(0x05D0)
#define scb_v_sw14		(0x05E0)
#define scb_v_sw15		(0x05F0)
#define scb_v_clock		(0x0600)
#define scb_v_inter		(0x0610)
#define scb_v_sys_corr_err	(0x0620)
#define scb_v_proc_corr_err	(0x0630)
#define scb_v_pwrfail		(0x0640)
#define scb_v_perfmon		(0x0650)
#define scb_v_sysmchk		(0x0660)
#define scb_v_procmchk		(0x0670)
#define scb_v_passive_rel	(0x06F0)

//
// Stack frame (FRM)
//

#define frm_v_r2		(0x0000)
#define frm_v_r3		(0x0008)
#define frm_v_r4		(0x0010)
#define frm_v_r5		(0x0018)
#define frm_v_r6		(0x0020)
#define frm_v_r7		(0x0028)
#define frm_v_pc		(0x0030)
#define frm_v_ps		(0x0038)

//
// Exeception summary register (EXS)
//
// exs_v_swc		<0>	; Software completion
// exs_v_inv		<1>	; Ivalid operation
// exs_v_dze		<2>	; Div by zero
// exs_v_fov		<3>	; Floating point overflow
// exs_v_unf		<4>	; Floating point underflow
// exs_v_ine		<5>	; Floating point inexact
// exs_v_iov		<6>	; Floating convert to integer overflow
#define exs_v_swc	  0
#define exs_v_inv	  1
#define exs_v_dze	  2
#define exs_v_fov	  3
#define exs_v_unf	  4
#define exs_v_ine	  5
#define exs_v_iov	  6

#define exs_m_swc               (1<<exs_v_swc)
#define exs_m_inv               (1<<exs_v_inv)
#define exs_m_dze               (1<<exs_v_dze)
#define exs_m_fov               (1<<exs_v_fov)
#define exs_m_unf               (1<<exs_v_unf)
#define exs_m_ine               (1<<exs_v_ine)
#define exs_m_iov               (1<<exs_v_iov)

//
// machine check error summary register (mces)
//
// mces_v_mchk		<0>	; machine check in progress
// mces_v_sce		<1>	; system correctable error
// mces_v_pce		<2>	; processor correctable error
// mces_v_dpc		<3>	; disable reporting of processor correctable errors
// mces_v_dsc		<4>	; disable reporting of system correctable errors
#define mces_v_mchk	 0
#define mces_v_sce	 1
#define mces_v_pce	 2
#define mces_v_dpc	 3
#define mces_v_dsc	 4

#define mces_m_mchk              (1<<mces_v_mchk)
#define mces_m_sce               (1<<mces_v_sce)
#define mces_m_pce               (1<<mces_v_pce)
#define mces_m_dpc               (1<<mces_v_dpc)
#define mces_m_dsc               (1<<mces_v_dsc)
#define mces_m_all		 ((1<<mces_v_mchk) | (1<<mces_v_sce) | (1<<mces_v_pce) | (1<<mces_v_dpc) | (1<<mces_v_dsc))

#endif
