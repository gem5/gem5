// build_fixed_image: not sure what means
// real_mm to be replaced during rewrite
// remove_save_state  remove_restore_state can be remooved to save space ??
#define egore 0
#define acore 0
#define beh_model 0
#define ev5_p2 1
#define ev5_p1 0
#define ldvpte_bug_fix 1
#define spe_fix 0
#define osf_chm_fix  0
#define build_fixed_image 0
#define enable_p4_fixups 0
#define osf_svmin 1
#define enable_physical_console 0
#define fill_err_hack 0
#define icflush_on_tbix 0
#define max_cpuid 1
#define perfmon_debug 0
#define rax_mode 0

#define hw_rei_spe hw_rei
	
#include "ev5_defs.h"
#include "ev5_impure.h"
#include "ev5_alpha_defs.h"
#include "ev5_paldef.h"
#include "ev5_osfalpha_defs.h"
#include "fromHudsonMacros.h"
#include "fromHudsonOsf.h"
#include "dc21164FromGasSources.h"
#include "cserve.h"

#define ldlp ldl_p
#define ldqp ldq_p
#define stqp stq_p
#define stqpc stqp

#define pt_entInt pt_entint
#define pt_entArith pt_entarith
#define mchk_size ((mchk_cpu_base + 7  + 8) &0xfff8)
#define mchk_flag CNS_Q_FLAG
#define mchk_sys_base 56
#define mchk_cpu_base (CNS_Q_LD_LOCK + 8)
#define mchk_offsets CNS_Q_EXC_ADDR
#define mchk_mchk_code 8
#define mchk_ic_perr_stat CNS_Q_ICPERR_STAT
#define mchk_dc_perr_stat CNS_Q_DCPERR_STAT
#define mchk_sc_addr CNS_Q_SC_ADDR
#define mchk_sc_stat CNS_Q_SC_STAT
#define mchk_ei_addr CNS_Q_EI_ADDR
#define mchk_bc_tag_addr CNS_Q_BC_TAG_ADDR
#define mchk_fill_syn CNS_Q_FILL_SYN
#define mchk_ei_stat CNS_Q_EI_STAT
#define mchk_exc_addr CNS_Q_EXC_ADDR
#define mchk_ld_lock CNS_Q_LD_LOCK
#define osfpcb_q_Ksp pcb_q_ksp
#define pal_impure_common_size ((0x200 + 7) & 0xfff8)

#define RTCADD 0x160000
#define RTCDAT 0x170000

/* Serial Port (COM) Definitions: */
#define DLA_K_BRG               12      /* Baud Rate Divisor = 9600 */

#define LSR_V_THRE              5       /* Xmit Holding Register Empty Bit */

#define LCR_M_WLS               3       /* Word Length Select Mask */
#define LCR_M_STB               4       /* Number Of Stop Bits Mask */
#define LCR_M_PEN               8       /* Parity Enable Mask */
#define LCR_M_DLAB              128     /* Divisor Latch Access Bit Mask */

#define LCR_K_INIT              (LCR_M_WLS | LCR_M_STB)

#define MCR_M_DTR               1       /* Data Terminal Ready Mask */
#define MCR_M_RTS               2       /* Request To Send Mask */
#define MCR_M_OUT1              4       /* Output 1 Control Mask */
#define MCR_M_OUT2              8       /* UART Interrupt Mask Enable */

#define MCR_K_INIT              (MCR_M_DTR  | \
                                 MCR_M_RTS  | \
                                 MCR_M_OUT1 | \
                                 MCR_M_OUT2)
#define SLOT_D_COM1                (0x140000)
#define COM1_RBR (SLOT_D_COM1 | (0x0 << 1)) /* Receive Buffer Register Offset */
#define COM1_THR (SLOT_D_COM1 | (0x0 << 1)) /* Xmit Holding Register Offset */
#define COM1_IER (SLOT_D_COM1 | (0x1 << 1)) /* Interrupt Enable Register Offset
*/
#define COM1_IIR (SLOT_D_COM1 | (0x2 << 1)) /* Interrupt ID Register Offset */
#define COM1_LCR (SLOT_D_COM1 | (0x3 << 1)) /* Line Control Register Offset */
#define COM1_MCR (SLOT_D_COM1 | (0x4 << 1)) /* Modem Control Register Offset */
#define COM1_LSR (SLOT_D_COM1 | (0x5 << 1)) /* Line Status Register Offset */
#define COM1_MSR (SLOT_D_COM1 | (0x6 << 1)) /* Modem Status Register Offset */
#define COM1_SCR (SLOT_D_COM1 | (0x7 << 1)) /* Scratch Register Offset */
#define COM1_DLL (SLOT_D_COM1 | (0x8 << 1)) /* Divisor Latch (LS) Offset */
#define COM1_DLH (SLOT_D_COM1 | (0x9 << 1)) /* Divisor Latch (MS) Offset */


#define BYTE_ENABLE_SHIFT    5
#define PCI_MEM   0x400

#ifdef SIMOS
#define OutPortByte(port,val,a,b)
#define InPortByte(port,val,a)
#else

#define OutPortByte(port,val,tmp0,tmp1) \
	LDLI	(tmp0, port); \
	sll	tmp0, BYTE_ENABLE_SHIFT, tmp0; \
	lda	tmp1, PCI_MEM(zero); \
	sll	tmp1, 29, tmp1; \
	bis	tmp0, tmp1, tmp0; \
	lda	tmp1, (val)(zero); \
	sll	tmp1, 8*(port & 3), tmp1; \
	stl_p	tmp1, 0x00(tmp0); \
	mb

#define InPortByte(port,tmp0,tmp1) \
	LDLI	(tmp0, port); \
	sll	tmp0, BYTE_ENABLE_SHIFT, tmp0; \
	lda	tmp1, PCI_MEM(zero); \
	sll	tmp1, 29, tmp1; \
	bis	tmp0, tmp1, tmp0; \
	ldl_p	tmp0, 0x00(tmp0); \
	srl	tmp0, (8 * (port & 3)), tmp0; \
	zap	tmp0, 0xfe, tmp0
#endif
        
#define r0 $0
#define r1 $1
#define r2 $2 
#define r3 $3 
#define r4 $4
#define r5 $5
#define r6 $6
#define r7 $7
#define r8 $8
#define r9 $9
#define r10 $10
#define r11 $11
#define r12 $12
#define r13 $13
#define r14 $14
#define r15 $15
#define r16 $16
#define r17 $17
#define r18 $18
#define r19 $19
#define r20 $20
#define r21 $21
#define r22 $22
#define r23 $23
#define r24 $24
#define r25 $25
#define r26 $26
#define r27 $27
#define r28 $28
#define r29 $29
#define r30 $30
#define r31 $31

#ifdef SIMOS
#define DEBUGSTORE(c)
#define DEBUG_EXC_ADDR()
#else
#define DEBUGSTORE(c) \
	lda	r13, c(zero) ; \
	bsr	r25, debugstore

#define DEBUG_EXC_ADDR() \
	bsr	r25, put_exc_addr; \
	DEBUGSTORE(13)		;  \
	DEBUGSTORE(10)

#endif /* SIMOS */

#define ALIGN_BLOCK \
	.align 5

#define ALIGN_BRANCH \
	.align 3
	
// This module is for all the OSF system specific code.
// This version is for the EV5 behavioral model.
// .sbttl	"Edit History"
//+
// Who		Rev	When		What	
// ------------	---	-----------	--------------------------------
//
// [ deleted several pages of checking comments to make the file smaller - lance ]
//
// PALcode merge done here .... JRH ....8/30/94
// JM            1.00     1-aug-1994     Add support for pass2 to sys_perfmon code
// JM            1.01     2-aug-1994     Initialize cns_bc_config in reset
// JM            1.02     5-aug-1994     Add ARITH_AND_MCHK routine.
//                                       Fix typo in bc_config init in rax reset flow.
// JM            1.03    19-aug-1994     BUG: sys_perfmon not generating mux control mask properly, overwriting counters 0 & 1;
//                                                       mode select masks messed up too.
//
// JH		1.03A	26-Oct-1994	Log PCI error2 register ...needed for CIA Pass 2 support
//
// JM		1.04	16-sep-1994	Moved perfmon code to ev5_osf_pal.m64
// JM		1.05 	 9-jan-1995	Fix to EI_STAT entry in MCHK logout frame -- OR in lower 28 bits (previously wiped out)
// JM		1.06     2-feb-1995     Change "HW_REI" to "HW_REI_SPE" everywhere that we may be changing to kernel mode from user
//						(part of super-page bug fix).
//					Initialize DC_TEST_CTL in reset flow.					
//					
// PALcode merge done here .... TLC ....02/06/95
//
// JM            1.07     3-mar-1995	Add init of dc_test_ctl; fix pvc_jsr statement in ret from set_sc_bc_ctl
// ES		1.08	17-mar-1995	Add osf_chm_fix to disable dcache in reset
//					
// PALcode merge done here .... TLC ....03/21/95
//
//
// Entry points
//	SYS_CFLUSH - Cache flush
//	SYS_CSERVE - Console service
//	SYS_WRIPIR - interprocessor interrupts
//	SYS_HALT_INTERRUPT - Halt interrupt
//	SYS_PASSIVE_RELEASE - Interrupt, passive release
//	SYS_INTERRUPT - Interrupt
//	SYS_RESET - Reset
//	SYS_ENTER_CONSOLE
//      SYS_PERFMON - Performance monitoring setup
//
//------------------------------------------------------------------------------
// 
// EB164 specific library files ....
// 
//------------------------------------------------------------------------------
#if eb164 != 0


// .sbttl "EB164 firmware required definition files"
_pal_def	    // console definition file 
_cserve_def	    // cserve definition file

#define hlt_c_callback 33

//
// rtc based constants
//
#define rtc_base 0xe00		// RTC device is at offset 70 << 5
#define rtc_ptr 0x0			// RTC index register (offset 0 from rtc base) 
#define rtc_data 0x20			// RTC data  register (offset 1 from rtc base)
#define rtc_idx_csrc 0x0c			// point to CSR C in the TOY chip

//
// interrupt vectors
//
#define vec_eisa_base  0x800		   // base for pci vectors
#endif

	ALIGN_BLOCK
	.globl sys_wripir
sys_wripir:				// R16 has the processor number.
					// The XXM has no interprocessor interrupts
	hw_rei

// .sbttl	"CFLUSH- PALcode for CFLUSH instruction"
//+
// SYS_CFLUSH
// Entry:
//
//	R16 - contains the PFN of the page to be flushed
//
// Function:
//	Flush all Dstream caches of 1 entire page
//
//-
	ALIGN_BLOCK
	.globl sys_cflush
sys_cflush:
	// convert pfn to addr, and clean off <63:20>
	sll	r16, ((page_offset_size_bits)+(63-20)), r12

	lda	r13, 0x10(r31)				   // assume 16Mbytes of cache  
	sll	r13, 20, r13				   // convert to bytes

	srl	r12, 63-20, r12	// shift back to normal position
	xor	r12, r13, r12		// xor addr bit that aligns with cache size
	
	or	r31, 8192/(32*8), r13	// get count of loads
	nop

CFLUSH_LOOP:
        subq    r13, 1, r13            // decr counter
        mfpr    r25, ev5__intid         // Fetch level of interruptor

	ldqp	r31, 32*0(r12)		// do a load
	ldqp	r31, 32*1(r12)		// do next load

	ldqp	r31, 32*2(r12)		// do next load
	ldqp	r31, 32*3(r12)		// do next load

	ldqp	r31, 32*4(r12)		// do next load
	ldqp	r31, 32*5(r12)		// do next load

	ldqp	r31, 32*6(r12)		// do next load
	ldqp	r31, 32*7(r12)		// do next load

        mfpr    r14, ev5__ipl           // Fetch current level
        lda     r12, (32*8)(r12)        // skip to next cache block addr

        cmple   r25, r14, r25           // R25 = 1 if intid .less than or eql ipl
        beq     r25, CFLUSH_LOOP_10_                // if any int's pending, re-queue CFLUSH

        bne     r13, CFLUSH_LOOP        // loop till done
        hw_rei                          // back to user

	ALIGN_BRANCH
CFLUSH_LOOP_10_:					// Here if interrupted
	mfpr	r12, exc_addr
	subq	r12, 4, r12		// Backup PC to point to CFLUSH
	
	mtpr	r12, exc_addr
	nop
	
	mfpr	r31, pt0		// Pad exc_addr write	
	hw_rei


// .sbttl	"CSERVE- PALcode for CSERVE instruction"
//+
// SYS_CSERVE
//
// Function:
//	Various functions for private use of console software
//
//	option selector in r0
//	arguments in r16....
//
//
//	r0 = 0	unknown
//
//	r0 = 1	ldqp
//	r0 = 2	stqp
//		args, are as for normal STQP/LDQP in VMS PAL
//
//	r0 = 3	dump_tb's
//	r16 = detination PA to dump tb's to.
//
//	r0<0> = 1, success
//	r0<0> = 0, failure, or option not supported
//	r0<63:1> = (generally 0, but may be function dependent)
//	r0 - load data on ldqp
//
//-
	ALIGN_BLOCK
	.globl sys_cserve
sys_cserve:
	cmpeq	r18, CSERVE_K_RD_IMPURE, r0
	bne	r0, Sys_Cserve_Rd_Impure
	
	cmpeq	r18, CSERVE_K_JTOPAL, r0
	bne	r0, Sys_Cserve_Jtopal

	// For now the XXM doesn't support any console callbacks
	DEBUGSTORE(0x40)
	bis	r18, zero, r0
	bsr	r25, put_hex
	DEBUGSTORE(13)
	DEBUGSTORE(10)
	or	r31, r31, r0
	hw_rei				// and back we go

Sys_Cserve_Rd_Impure:
	mfpr	r0, pt_impure		// Get base of impure scratch area.
	hw_rei

// .sbttl	"SYS_INTERRUPT - Interrupt processing code"
//+
// SYS_INTERRUPT
//
//	Current state:
//		Stack is pushed
//		ps, sp and gp are updated
//		r12, r14 - available
//		r13 - INTID (new EV5 IPL)
//		r14 - exc_addr
//		r25 - ISR
//		r16, r17, r18 - available
//
//-
	ALIGN_BLOCK
	.globl sys_interrupt
sys_interrupt:
	extbl	r12, r14, r14		// Translate new OSFIPL->EV5IPL
	mfpr	r29, pt_kgp		// update gp

	mtpr	r14, ev5__ipl		// load the new IPL into Ibox
	cmpeq	r13, 31, r12
	bne	r12, sys_int_mchk_or_crd // Check for level 31 interrupt (machine check or crd)

	cmpeq	r13, 30, r12
	bne	r12, sys_int_powerfail	// Check for level 30 interrupt (powerfail)

	cmpeq	r13, 29, r12
	bne	r12, sys_int_perf_cnt	// Check for level 29 interrupt (performance counters)

	cmpeq	r13, 23, r12
	bne	r12, sys_int_23 	// Check for level 23 interrupt
	
	cmpeq	r13, 22, r12
	bne	r12, sys_int_22 	// Check for level 22 interrupt (clock)

	cmpeq	r13, 21, r12
	bne	r12, sys_int_21 	// Check for level 21 interrupt

	cmpeq	r13, 20, r12
	bne	r12, sys_int_20		// Check for level 20 interrupt (I/O)

	mfpr	r14, exc_addr		// ooops, something is wrong
	br	r31, pal_pal_bug_check_from_int

//+
//sys_int_2*
//	Routines to handle device interrupts at IPL 23-20.
//	System specific method to ack/clear the interrupt, detect passive release,
//	  detect interprocessor (22),  interval clock (22),  corrected
//	  system error (20)
//
//	Current state:
//		Stack is pushed
//		ps, sp and gp are updated
//		r12, r14 - available
//		r13 - INTID (new EV5 IPL)
//		r25 - ISR
//
//	On exit:
//		Interrupt has been ack'd/cleared
//		a0/r16 - signals IO device interrupt
//		a1/r17 - contains interrupt vector
//		exit to ent_int address
//
//-
	ALIGN_BLOCK
sys_int_23:
//   INT23 is unused on the XXM
	nop
	DEBUGSTORE(0x21)
	DEBUGSTORE(0x32)
	DEBUGSTORE(0x33)
	DEBUGSTORE(13)
	DEBUGSTORE(10)
	lda	r14, 0xffff(zero)
	br	r31, sys_int_mchk	// log as a machine check

//   INT22 is the Real Time Clock
//
//	Dismiss the interrupt in the TOY
//	Dispatch to kernel
//
	ALIGN_BLOCK
sys_int_22:
	OutPortByte(RTCADD,0x0C,r12,r14)// Set up RTCADD to index register C.
	InPortByte(RTCDAT,r12,r14)	// Read to clear interrupt.
	mfpr	r12, pt_entInt
	lda	r17, 0x600(r31)	// r17 = interrupt vector for interval timer
	mtpr	r12, exc_addr
//	DEBUGSTORE(0x2a)
//	DEBUG_EXC_ADDR()
	lda	r16, 0x1(r31)	// r16 = interrupt type for interval timer
	STALL
	hw_rei

	ALIGN_BLOCK
sys_int_21:
	// Not connected on the XXM
	nop
	DEBUGSTORE(0x21)
	DEBUGSTORE(0x32)
	DEBUGSTORE(0x31)
	DEBUGSTORE(13)
	DEBUGSTORE(10)
	lda	r14, 0xffff(zero)
	br	r31, sys_int_mchk		// go log the interrupt as a machine check

//   INT20 are device interrupts
//	Dispatch to kernel
//
	ALIGN_BLOCK
sys_int_20:
	mfpr	r12, pt_entInt		// Get pointer to kernel handler.
	lda	r17, 0x800(zero)	// Hardcode device interrupt for now
	mtpr	r12, exc_addr		// Load kernel entry address
	bis	zero, 0x3, r16		// Signal I/O device interrupt
	STALL
	hw_rei


//+
// sys_passive_release
//	Just pretend the interrupt never occurred.
//
// make sure we restore the old PS before calling this ....
//-
	ALIGN_BRANCH
	.globl sys_passive_release
sys_passive_release:
	mtpr	r11, ev5__dtb_cm	// Restore Mbox current mode for ps
	nop
	
	mfpr	r31, pt0		// Pad write to dtb_cm

	hw_rei_spe

//+
//sys_int_powerfail
//	The XXM doesn't generate these, right?

	ALIGN_BLOCK
sys_int_powerfail:
	nop
	lda	r14, 0xffff(zero)
	br	r31, sys_int_mchk		// go log the interrupt as a machine check

//+
// sys_halt_interrupt
//       A halt interrupt has been detected.  Pass control to the console.
//
//
//-
	ALIGN_BLOCK
	.globl sys_halt_interrupt
sys_halt_interrupt:
	// wait for halt to go away 
SYS_HALT_PIN:					// REPEAT
	mfpr	r25, ev5__isr			// : Fetch interrupt summary register
	srl     r25, isr_v_hlt, r25		// : Get HLT bit
	blbs	r25, SYS_HALT_PIN		// UNTIL interrupt goes away
	LDLI	(r25,  25000000)		// Loop 100msec on an XXM
SYS_HALT_KEY_DEBOUNCE:
	subq	r25, 1, r25
	bne	r25, SYS_HALT_KEY_DEBOUNCE
	mfpr	r25, ev5__isr
	srl     r25, isr_v_hlt, r25
	blbs	r25, SYS_HALT_PIN
	
	mtpr	r11, dtb_cm		// Restore Mbox current mode
	mtpr	r0, pt0
//      pvc_jsr updpcb, bsr=1
//      bsr     r0, pal_update_pcb      // update the pcb

//      lda     r0, hlt_c_hw_halt(r31)  // set halt code to hw halt
        br      r31, sys_enter_console  // enter the console


//+
// sys_int_mchk_or_crd
//
//	Current state:
//		Stack is pushed
//		ps, sp and gp are updated
//		r12
//		r13 - INTID (new EV5 IPL)
//		r14 - exc_addr
//		r25 - ISR
//		r16, r17, r18 - available
//
//-
	ALIGN_BLOCK

sys_int_mchk_or_crd:
	srl	r25, isr_v_mck, r12
	lda	r14, 7(zero)
	blbs	r12, sys_int_mchk
	bsr	r12, sys_int_mchk	// on the XXM it's always a mchk



// .sbttl "SYS_INT_MCHK - MCHK Interrupt code"
//+
// Machine check interrupt from the system.  Setup and join the
// regular machine check flow.
// On exit:
//       pt0     - saved r0
//       pt1     - saved r1
//       pt4     - saved r4
//       pt5     - saved r5
//       pt6     - saved r6
//       pt10    - saved exc_addr
//       pt_misc<47:32> - mchk code
//       pt_misc<31:16> - scb vector
//       r14     - base of Cbox IPRs in IO space
//       MCES<mchk> is set
//-
	ALIGN_BLOCK
sys_int_mchk:
//
// Common code to setup machine so we can join with 
// the code to log processor detected machine checks
//
// Input Registers:
//
//	r14 - machine check code 
//
	mfpr	r12, exc_addr

	mtpr	r6, pt6
	mtpr	r12, pt10			// Stash exc_addr

	bis	r14, r31, r6			// save machine check code
	sll	r14, 32, r14			// Move mchk code to position

	mfpr	r12, pt_misc			// Get MCES and scratch
	mtpr	r0, pt0				// Stash for scratch

	zap	r12, 0x3c, r12			// Clear scratch
        blbs    r12, sys_double_machine_check   // MCHK halt if double machine check		

	or	r12, r14, r12			// Combine mchk code 
	lda	r14, scb_v_sysmchk(r31)		// Get SCB vector

	sll	r14, 16, r14			// Move SCBv to position
	or	r12, r14, r14			// Combine SCBv

	bis	r14, 1<<mces_v_mchk, r14	// Set MCES<MCHK> bit
	mtpr	r14, pt_misc			// Save mchk code!scbv!whami!mces

	ldah	r14, 0xfff0(r31)
	mtpr	r1, pt1				// Stash for scratch

	zap	r14, 0xE0, r14			// Get Cbox IPR base
	mtpr	r4, pt4

	mtpr	r5, pt5

	// Start to collect the IPRs.  Common entry point for mchk flows.
	//
	// Current state:
	//	pt0	- saved r0
	//	pt1	- saved	r1
	//	pt4	- saved r4
	//	pt5	- saved r5
	//	pt6	- saved r6
	//	pt10	- saved exc_addr
	//	pt_misc<47:32> - mchk code 
	//	pt_misc<31:16> - scb vector
	//       r6      - saved machine check code
	//	r0, r1, r4, r5, r12, r13, r25 - available
	//	r8, r9, r10 - available as all loads are physical
	//	MCES<mchk> is set and machine check code is saved

	ldah	r14, 0xfff0(r31)
	mtpr	r1, pt1		// Stash for scratch - 30 instructions

	zap	r14, 0xE0, r14	// Get Cbox IPR base

	mb			// MB before reading Scache IPRs
	mfpr	r1, icperr_stat

	mfpr	r8, dcperr_stat
	mtpr	r31, dc_flush	// Flush the Dcache
	
	mfpr	r31, pt0	// Pad Mbox instructions from dc_flush
	mfpr	r31, pt0
	nop
	nop

	ldqp	r9, sc_addr(r14)// SC_ADDR IPR
	bis	r9, r31, r31	// Touch ld to make sure it completes before
				// read of SC_STAT
	ldqp	r10, sc_stat(r14)	// SC_STAT, also unlocks SC_ADDR

	ldqp	r12, ei_addr(r14)// EI_ADDR IPR
	ldqp	r13, bc_tag_addr(r14)	// BC_TAG_ADDR IPR
	ldqp	r0, fill_syn(r14)	// FILL_SYN IPR
	// Touch lds to make sure they complete before reading EI_STAT
	bis	r12, r13, r31
	bis	r0, r0, r31
	ldqp	r25, ei_stat(r14)	// EI_STAT, unlock EI_ADDR, BC_TAG_ADDR, FILL_SYN
	ldqp	r31, ei_stat(r14)	// Read again to insure it is unlocked
	br	r31, sys_mchk_write_logout_frame	// Join common machine check flow


// .sbttl  "SYS_INT_PERF_CNT - Performance counter interrupt code"
//+
//sys_int_perf_cnt
//
//	A performance counter interrupt has been detected.  The stack has been pushed.
//	IPL and PS are updated as well.
//
//	on exit to interrupt entry point ENTINT:: 
//		a0 = osfint_c_perf
//		a1 = scb_v_perfmon (650)
//		a2 = 0 if performance counter 0 fired
//		a2 = 1 if performance counter 1 fired
//		a2 = 2 if performance counter 2 fired
//		     (if more than one counter overflowed, an interrupt will be 
//			generated for each counter that overflows)
//	
//
//-
	ALIGN_BLOCK
sys_int_perf_cnt:			// Performance counter interrupt
	lda	r17, scb_v_perfmon(r31)	// a1 to interrupt vector
	mfpr	r25, pt_entInt

	lda	r16, osfint_c_perf(r31)	// a0 to perf counter code
	mtpr	r25, exc_addr
	
	//isolate which perf ctr fired, load code in a2, and ack
	mfpr	r25, isr
	or	r31, r31, r18			// assume interrupt was pc0

	srl	r25, isr_v_pc1, r25		// isolate 
	cmovlbs	r25, 1, r18			// if pc1 set, load 1 into r14

	srl	r25, 1, r25			// get pc2
	cmovlbs r25, 2, r18			// if pc2 set, load 2 into r14

	lda	r25, 1(r31)			// get a one
	sll	r25, r18, r25

	sll	r25, hwint_clr_v_pc0c, r25	// ack only the perf counter that generated the interrupt
	mtpr	r25, hwint_clr
	
	hw_rei_spe



// .sbttl	"System specific RESET code"
//+
//  RESET code
//   On entry:
//	r1 = pal_base +8
//
//	Entry state on trap:
//       r0 = whami
//       r2 = base of scratch area
//       r3 = halt code
//	and the following 3 if init_cbox is enabled:
//       r5 = sc_ctl
//       r6 = bc_ctl
//       r7 = bc_cnfg
//
//       Entry state on switch:
// 	r17 - new PC
// 	r18 - new PCBB
// 	r19 - new VPTB
//
//-
	ALIGN_BLOCK
	.globl sys_reset
sys_reset:
	bis	r1, r31, r22
	bis	r2, r31, r23
	bis	r3, r31, r24
	
	lda	r0, 0(zero)		// The XXM has only one CPU
	lda	r2, 0x2000(zero)	// KLUDGE the impure area address
	lda	r3, 0(zero)		// Machine restart
	mtpr	r31, pt_misc		// Zero out whami & swppal flag
	mtpr	r31, pt0		// halt code of "reset"
	mtpr	r31, pt1		// whami is always 0 on XXM

/* Check to see if the transfer from the POST (Power-on-Self Test) code
** is following a standard protocol and that the other input parameter
** values may be trusted. Register a3 (r19) will contain a signature if so.
**
** Register values:
**
** t0 (r1)	bcCtl value, saved into pt4
** t1 (r2)	bcCfg value
** t2 (r3)	bcCfgOff value (values for bcache off)
**
** s6 (r15)	encoded srom.s RCS revision
** a0 (r16)	processor identification (a la SRM)
** a1 (r17)	size of contiguous, good memory in bytes
** a2 (r18)	cycle count in picoseconds
** a3 (r19)	signature (0xDECB) in <31:16> and system revision ID in <15:0>
** a4 (r20)	active processor mask
** a5 (r21)	system context value
*/
	lda	r4, CNS_Q_IPR(r2)	// Point to base of IPR area.
	lda	r4, pal_impure_common_size(r4)	// Bias by PAL common size
	SAVE_SHADOW(r22,CNS_Q_BC_CTL,r4)	// Save shadow of bcCtl.
	SAVE_SHADOW(r23,CNS_Q_BC_CFG,r4)	// Save shadow of bcCfg.
#ifdef undef
	SAVE_SHADOW(r24,CNS_Q_BC_CFG_OFF,r4)	// Save shadow of bcCfg.
#endif
	SAVE_SHADOW(r15,CNS_Q_SROM_REV,r4)	// Save srom revision.
	SAVE_SHADOW(r16,CNS_Q_PROC_ID,r4)	// Save processor id.
	SAVE_SHADOW(r17,CNS_Q_MEM_SIZE,r4)	// Save memory size.
	SAVE_SHADOW(r18,CNS_Q_CYCLE_CNT,r4)	// Save cycle count.
	SAVE_SHADOW(r19,CNS_Q_SIGNATURE,r4)	// Save signature and sys rev.
	SAVE_SHADOW(r20,CNS_Q_PROC_MASK,r4)	// Save processor mask.
	SAVE_SHADOW(r21,CNS_Q_SYSCTX,r4)	// Save system context.

//;;;;;;	mtpr	r31, ic_flush_ctl; do not flush the icache - done by hardware before SROM load

/*
 * Initialize the serial ports
 *
 *	Baud rate:	9600 baud
 *	Word length:	8 bit characters
 *	Stop bits:	1 stop bit
 *	Parity:		No parity
 *	Modem control:	DTR, RTS active, OUT1 lov, UART interrupts enabled
 */

/* Initialize COM1*/
	OutPortByte(COM1_LCR,LCR_M_DLAB,r12,r14)// Access clock divisor latch.
	OutPortByte(COM1_DLL,DLA_K_BRG,r12,r14)	// Set the baud rate.
	OutPortByte(COM1_DLH,0,r12,r14)
	OutPortByte(COM1_LCR,LCR_K_INIT,r12,r14)// Set line control register.
	OutPortByte(COM1_MCR,MCR_K_INIT,r12,r14)// Set modem control register.
	OutPortByte(COM1_IER,0x0f,r12,r14)	// Turn on interrupts.
	
/* Flush COM1's receive buffer*/
#ifndef SIMOS
Sys_ResetFlushCom1:
	InPortByte(COM1_LSR,r12,r14)		// Read the line status.
	blbc	t0, Sys_ResetCom1Done		// Are we done yet?
	InPortByte(COM1_RBR,r12,r14)		// Read receive buffer reg.
	br	zero, Sys_ResetFlushCom1	// Loop till done.

Sys_ResetCom1Done:
#endif
	mb

	mtpr	r31, itb_ia		// clear the ITB
	mtpr	r31, dtb_ia		// clear the DTB

//;;;;;;	lda     r1, 0x1FFF(R31)
//;;;;;;	mtpr    r1, dc_test_ctl         ; initialize tag index to all 1's.

	/* Hardwire the base address of the PALcode */
	lda	r1, 0x4000(r31)		// point to start of code
	mtpr	r1, pal_base		// initialize PAL_BASE

	// Interrupts
	mtpr	r31, astrr		// stop ASTs
	mtpr	r31, aster		// stop ASTs
	mtpr	r31, sirr		// clear software interrupts

	// enable shadow registers, floating point, kseg addressing
	ldah	r1, ((1<<(icsr_v_sde-16)) | (1<<(icsr_v_fpe-16)) | (2<<(icsr_v_spe-16)))(r31)
	mtpr	r1, icsr

	// Mbox/Dcache init
	lda	r1, (1<<(mcsr_v_sp1))(r31)
	mtpr	r1, mcsr	// MCSR - Super page enabled
	lda	r1, 1<<dc_mode_v_dc_ena(r31)
	bsr	r31, 1f

	ALIGN_BRANCH
1:	mtpr	r1, dc_mode	// turn Dcache on
	nop
	STALL			// No Mbox instr in 1,2,3,4
	STALL
	STALL
	STALL
	mtpr	r31, dc_flush	// flush Dcache

	// build PS (IPL=7,CM=K,VMM=0,SW=0)
	lda	r11, 0x7(r31)	// Shadow copy of PS - kern mode, IPL=7
	lda	r1, 0x1e(r31)		

	mtpr	r1, ev5__ipl		// set internal <ipl>
	mtpr	r31, ips	// set new ps<cm>=0, Ibox copy
	mtpr	r31, dtb_cm	// set new ps<cm>=0, Mbox copy

	// Create the PALtemp pt_intmask -
	//   MAP:
	//	OSF IPL		EV5 internal IPL(hex)	note
	//	0		0
	//	1		1
	//	2		2
	//	3		14			device
	//	4		15			device
	//	5		16			device	
	//	6		1e			device
	//	7		1e  make sure we can take mchks at ipl 31
	
	ldah	r1, 0x1e1e(r31)	// Create upper lw of int_mask
	lda	r1, 0x1615(r1)

	sll	r1, 32, r1
	ldah	r1, 0x1402(r1)	// Create lower lw of int_mask
	
	lda	r1, 0x0100(r1)
	mtpr	r1, pt_intmask	// Stash in PALtemp

	// Unlock a bunch of chip internal IPRs
	mtpr	r31, exc_sum	// clear exeception summary and exc_mask
	mfpr	r31, va		// unlock va, mmstat
	lda	r8, ((1<<icperr_stat_v_dpe) | (1<<icperr_stat_v_tpe) | (1<<icperr_stat_v_tmr))(r31)
	mtpr	r8, icperr_stat	// Clear Icache parity error & timeout
	lda	r8, ((1<<dcperr_stat_v_lock) | (1<<dcperr_stat_v_seo))(r31)
	mtpr	r8, dcperr_stat	// Clear Dcache parity error status

	rc	r0		// clear intr_flag
	mtpr	r31, pt_trap

	mfpr	r0, pt_misc
	srl	r0, pt_misc_v_switch, r1
	blbs	r1, sys_reset_switch	// see if we got here from swppal

	// Rest of the "real" reset flow
	// ASN
	mtpr	r31, dtb_asn
	mtpr	r31, itb_asn

	lda     r1, 0x1FFF(r31)
        mtpr    r1, dc_test_ctl	// initialize tag index to all 1's.

	lda	r1, 0x67(r31)
	sll	r1, hwint_clr_v_pc0c, r1
	mtpr	r1, hwint_clr	// Clear hardware interrupt requests

	lda	r1, 1<<mces_v_dpc(r31) // 1 in disable processor correctable error
	mfpr	r0, pt1		// get whami
	insbl	r0, 1, r0	// isolate whami in correct pt_misc position
	or	r0, r1, r1	// combine whami and mces
	mtpr	r1, pt_misc	// store whami and mces, swap bit clear

	// CPU specific impure pointer
	extbl   r0, 1, r0             //get whami again
#if eb164 != 0
// compile error .....
//
//	mulq    r0, ((pal_impure_specific_size+mchk_size+mchk_crd_size)/8), r0 ; whami * per_node size/8
	lda	r1, ((pal_impure_specific_size+mchk_size+mchk_crd_size)/8)(r31)
	mulq	r0, r1, r0		//
#endif

	addq    r0, pal_impure_common_size/8, r0// add common area
	sll     r0, 3, r0              // * 8
	addq    r2, r0, r2              // addr our impure area offset
	mtpr	r2, pt_impure

	zapnot	r3, 1, r0		// isolate halt code
	mtpr	r0, pt0			// save entry type

	// Cycle counter
	or	r31, 1, r9		// get a one
	sll	r9, 32, r9		// shift to <32>
	mtpr	r31, cc			// clear Cycle Counter
	mtpr	r9, cc_ctl		// clear and enable the Cycle Counter
	mtpr	r31, pt_scc		// clear System Cycle Counter


	// Misc PALtemps
	mtpr    r31, maf_mode           // no mbox instructions for 3 cycles
	or	r31, 1, r1		// get bogus scbb value
	mtpr	r1, pt_scbb		// load scbb
	mtpr	r31, pt_prbr		// clear out prbr
	mfpr	r1, pal_base
	lda	r1, (kludge_initial_pcbb-0x4000)(r1) // get address for temp pcbb

	mtpr	r1, pt_pcbb		// load pcbb
	lda	r1, 2(r31)		// get a two
	sll	r1, 32, r1		// gen up upper bits
	mtpr	r1, mvptbr
	mtpr	r1, ivptbr
	mtpr	r31, pt_ptbr
	// Performance counters
	mtpr    r31, pmctr

        // Clear pmctr_ctl in impure area
	mfpr	r1, pt_impure
	stqp	r31, CNS_Q_PM_CTL(r1)
	
	ldah	r14, 0xfff0(r31)
	zap	r14, 0xE0, r14		// Get Cbox IPR base

#ifndef SIMOS
	ldqp	r31, sc_stat(r14)	// Clear sc_stat and sc_addr
	ldqp	r31, ei_stat(r14)
	ldqp	r31, ei_stat(r14)	// Clear ei_stat, ei_addr, bc_tag_addr, fill_syn
#endif
        mfpr	r13, pt_impure
	stqpc	r31, 0(r13)		// Clear lock_flag

	lda	r0, 0(zero)
	mfpr	r1, pt_impure
	bsr	r3, pal_save_state

	mfpr	r0, pt0			// get entry type
	br	r31, sys_enter_console	// enter the cosole


	// swppal entry
	// r0 - pt_misc
	// r17 - new PC
	// r18 - new PCBB
	// r19 - new VPTB
sys_reset_switch:
	or	r31, 1, r9
	sll	r9, pt_misc_v_switch, r9
	bic	r0, r9, r0		// clear switch bit
	mtpr	r0, pt_misc

	rpcc	r1			// get cyccounter

	ldqp	r22, osfpcb_q_fen(r18)	// get new fen/pme
	ldlp	r23, osfpcb_l_cc(r18)	// get cycle counter
	ldlp	r24, osfpcb_l_asn(r18)	// get new asn

	ldqp	r25, osfpcb_q_Mmptr(r18)// get new mmptr
	sll	r25, page_offset_size_bits, r25 // convert pfn to pa
	mtpr	r25, pt_ptbr		// load the new mmptr
	mtpr	r18, pt_pcbb		// set new pcbb

	bic	r17, 3, r17		// clean use pc
	mtpr	r17, exc_addr		// set new pc
	mtpr	r19, mvptbr
	mtpr	r19, ivptbr

	ldqp	r30, osfpcb_q_Usp(r18)	// get new usp
	mtpr	r30, pt_usp		// save usp
	
	sll	r24, dtb_asn_v_asn, r8
	mtpr	r8, dtb_asn
	sll	r24, itb_asn_v_asn, r24
	mtpr	r24, itb_asn

	mfpr	r25, icsr		// get current icsr
	lda	r24, 1(r31)
	sll	r24, icsr_v_fpe, r24	// 1 in icsr<fpe> position
	bic	r25, r24, r25		// clean out old fpe
	and	r22, 1, r22		// isolate new fen bit
	sll	r22, icsr_v_fpe, r22
	or	r22, r25, r25		// or in new fpe
	mtpr	r25, icsr		// update ibox ipr

	subl	r23, r1, r1		// gen new cc offset
	insll	r1, 4, r1		// << 32
	mtpr	r1, cc			// set new offset

	or	r31, r31, r0		// set success
   	ldqp	r30, osfpcb_q_Ksp(r18)	// get new ksp
	mfpr	r31, pt0		// stall
	hw_rei_stall

// .sbttl	"SYS_MACHINE_CHECK - Machine check PAL"
	ALIGN_BLOCK
//+
//sys_machine_check
// 	A machine_check trap has occurred.  The Icache has been flushed.
//
//-
	.globl sys_machine_check
sys_machine_check:
	DEBUGSTORE(052)
	DEBUGSTORE(052)
	DEBUGSTORE(052)
	// Need to fill up the refill buffer (32 instructions) and
	// then flush the Icache again.
	// Also, due to possible 2nd Cbox register file write for
	// uncorrectable errors, no register file read or write for 7 cycles.

	nop
	mtpr	r0, pt0	// Stash for scratch -- OK if Cbox overwrites r0 later

	nop
	nop

	nop
	nop

	nop
	nop

	nop
	nop
			// 10 instructions; 5 cycles

	nop
	nop

	nop
	nop

	// Register file can now be written
	lda	r0, scb_v_procmchk(r31)	// SCB vector
	mfpr	r13, pt_mces		// Get MCES
	sll	r0, 16, r0		// Move SCBv to correct position
	bis	r13, 1<<mces_v_mchk, r14	// Set MCES<MCHK> bit

	zap	r14, 0x3C, r14		// Clear mchk_code word and SCBv word 
	mtpr	r14, pt_mces		// 20 instructions

	nop
	or	r14, r0, r14		// Insert new SCB vector
	lda	r0, mchk_c_proc_hrd_error(r31)	// MCHK code
	mfpr	r12, exc_addr

	sll	r0, 32, r0		// Move MCHK code to correct position
	mtpr	r4, pt4
	or	r14, r0, r14		// Insert new MCHK code
	mtpr	r14, pt_misc		// Store updated MCES, MCHK code, and SCBv

	ldah	r14, 0xfff0(r31)
	mtpr	r1, pt1			// Stash for scratch - 30 instructions

	zap	r14, 0xE0, r14		// Get Cbox IPR base
	mtpr	r12, pt10		// Stash exc_addr

	mtpr	r31, ic_flush_ctl	// Second Icache flush, now it is really flushed.
//	blbs	r13, sys_double_machine_check		; MCHK halt if double machine check
	blbs	r12, sys_machine_check_while_in_pal	// MCHK halt if machine check in pal

	mtpr	r6, pt6
	mtpr	r5, pt5

	
	//+
	// Start to collect the IPRs.  Common entry point for mchk flows.
	//
	// Current state:
	//	pt0	- saved r0
	//	pt1	- saved	r1
	//	pt4	- saved r4
	//	pt5	- saved r5
	//	pt6	- saved r6
	//	pt10	- saved exc_addr
	//	pt_misc<47:32> - mchk code 
	//	pt_misc<31:16> - scb vector
	//	r14	- base of Cbox IPRs in IO space
	//	r0, r1, r4, r5, r6, r12, r13, r25 - available
	//	r8, r9, r10 - available as all loads are physical
	//	MCES<mchk> is set and machine check code is saved
	//
	//-

	ALIGN_BRANCH
	.globl sys_mchk_collect_iprs
sys_mchk_collect_iprs:


	mb				// MB before reading Scache IPRs
	mfpr	r1, icperr_stat

	mfpr	r8, dcperr_stat
	mtpr	r31, dc_flush		// Flush the Dcache
	
	mfpr	r31, pt0		// Pad Mbox instructions from dc_flush
	mfpr	r31, pt0
	nop
	nop

	ldqp	r9, sc_addr(r14)	// SC_ADDR IPR
	bis	r9, r31, r31		// Touch ld to make sure it completes before
					// read of SC_STAT
	ldqp	r10, sc_stat(r14)	// SC_STAT, also unlocks SC_ADDR

	ldqp	r12, ei_addr(r14)	// EI_ADDR IPR
	ldqp	r13, bc_tag_addr(r14)	// BC_TAG_ADDR IPR
	ldqp	r0, fill_syn(r14)	// FILL_SYN IPR
	// Touch lds to make sure they complete before reading EI_STAT
	bis	r12, r13, r31
	//	wait for r0 ldqp to complete
	bis	r0, r0, r31
	ldqp	r25, ei_stat(r14)	// EI_STAT, unlock EI_ADDR, BC_TAG_ADDR, FILL_SYN
	ldqp	r31, ei_stat(r14)	// Read again to insure it is unlocked
	mfpr	r6, pt_misc
	extwl	r6, 4, r6				// Fetch mchk code
	br	r31,  sys_mchk_write_logout_frame	//
	


	//+
	// Write the logout frame
	//
	// Current state:
	//	r0	- fill_syn
	//	r1	- icperr_stat
	//	r4	- available
	// 	r5<0>  	- retry flag
	//	r6     	- mchk code
	//	r8	- dcperr_stat
	//	r9	- sc_addr
	//	r10	- sc_stat
	//	r12	- ei_addr
	//	r13	- bc_tag_addr
	//	r14	- available
	//	r25	- ei_stat (shifted)
	//	pt0	- saved r0
	//	pt1	- saved	r1
	//	pt4	- saved r4
	//	pt5	- saved r5
	//	pt6	- saved r6
	//	pt10	- saved exc_addr
	//
	//-
	ALIGN_BRANCH
sys_mchk_write_logout_frame:
    //------------------------------------------------------------------------------------
    // EB164 specific code ....
    //
    // R14 - uncorrectable error logout frame address = 6000h + size of CRD frame
    //
    //------------------------------------------------------------------------------------
#if eb164 != 0


	lda	r14,PAL_LOGOUT_BASE(r31)	// get the start of logout frame location
	lda	r14,mchk_mchk_base(r14)		// add in the size of the CRD frame
#endif

	// Write the first 2 quadwords of the logout area:
	
	sll	r5, 63, r5		// Move retry flag to bit 63
	lda	r4, mchk_size(r5)	// Combine retry flag and frame size
	stqp	r4, mchk_flag(r14)	// store flag/frame size
	lda	r4, mchk_sys_base(r31)	// sys offset
	sll	r4, 32, r4
	lda	r4, mchk_cpu_base(r4)	// cpu offset
	stqp	r4, mchk_offsets(r14)	// store sys offset/cpu offset into logout frame

	//+
	// Write the mchk code to the logout area
	// Write error IPRs already fetched to the logout area
	// Restore some GPRs from PALtemps
	//-

	mfpr	r5, pt5
	stqp	r6, mchk_mchk_code(r14)
	mfpr	r4, pt4
	stqp	r1, mchk_ic_perr_stat(r14)
	mfpr	r6, pt6
	stqp	r8, mchk_dc_perr_stat(r14)
	mfpr	r1, pt1
	stqp	r9, mchk_sc_addr(r14)		
	stqp	r10, mchk_sc_stat(r14)		
	stqp	r12, mchk_ei_addr(r14)
	stqp	r13, mchk_bc_tag_addr(r14)
	stqp	r0,  mchk_fill_syn(r14)
	mfpr	r0, pt0
#define ei_stat_v_bc_tperr 28
	sll	r25, ei_stat_v_bc_tperr, r25	// Move EI_STAT status bits back to expected position
	//
	// Steve Shirron ei_stat.chip_id bits fix
	//
	// r25 has the ei_stat<35:28> bits aligned to bit 0.
	// Because this alignment trashed the ei_stat.chip_id<27:24> bits,
	// Steve's fix re-reads the ei_stat to recover the chip_id bits before
	// writing the ei_stat to the log.
	// 
        ldah	r13, 0xfff0(r31)	// r13 <- pointer to cbox
        zapnot	r13, 0x1f, r13		// : 
        ldqp	r13, ei_stat(r13)	// r13 <- contents of ei_stat register
        sll	r13, 64-ei_stat_v_bc_tperr, r13	// clear bits <63:36>
	srl	r13, 64-ei_stat_v_bc_tperr, r13	// put ei_stat bits back 
	or	r25, r13, r25	// or-in the stat bits from the original read
	stqp	r25, mchk_ei_stat(r14)			// write to the log

	mfpr	r25, pt10
	stqp	r25, mchk_exc_addr(r14)

	// complete the CPU-specific part of the logout frame

#define mchk_logout(regName, regOff) \
	mfpr	r25, regName ; \
	stqp	r25, CNS_Q_/**/regOff(r14)

	mchk_logout(mm_stat, MM_STAT)
	mchk_logout(va, VA)			// Unlocks VA and MM_STAT
	mchk_logout(isr, ISR)
	mchk_logout(icsr, ICSR)
	mchk_logout(pal_base, PAL_BASE)
	mchk_logout(exc_mask, EXC_MASK)
	mchk_logout(exc_sum, EXC_SUM)

	ldah	r13, 0xfff0(r31)
	zap	r13, 0xE0, r13			// Get Cbox IPR base
	ldqp	r13, ld_lock(r13)		// Get ld_lock IPR
	stqp	r13, mchk_ld_lock(r14)		// and stash it in the frame

	//+
	// complete the PAL-specific part of the logout frame
	//-
#define svpt(n)\
	mfpr	r25, pt/**/n ;\
	stqp	r25, CNS_Q_PT+(8*n)(r14)
	
	svpt(0)
	svpt(1)
	svpt(2)
	svpt(3)
	svpt(4)
	svpt(5)
	svpt(6)
	svpt(7)
	svpt(8)
	svpt(9)
	svpt(10)
	svpt(11)
	svpt(12)
	svpt(13)
	svpt(14)
	svpt(15)
	svpt(16)
	svpt(17)
	svpt(18)
	svpt(19)
	svpt(20)
	svpt(21)
	svpt(22)
	svpt(23)
	
	//+
	// Log system specific info here
	//-

	// Unlock IPRs
	lda	r8, ((1<<dcperr_stat_v_lock) | (1<<dcperr_stat_v_seo))(r31)
	mtpr	r8, dcperr_stat		// Clear Dcache parity error status

	lda	r8, ((1<<icperr_stat_v_dpe) | (1<<icperr_stat_v_tpe) | (1<<icperr_stat_v_tmr))(r31)
	mtpr	r8, icperr_stat	// Clear Icache parity error & timeout status

//	pvc_jsr armc, bsr=1
        bsr     r12, sys_arith_and_mchk  // go check for and deal with arith trap

	mtpr	r31, exc_sum		// Clear Exception Summary

        mfpr    r25, pt10               // write exc_addr after arith_and_mchk to pickup new pc
        stqp    r25, mchk_exc_addr(r14)

	//+
	// Set up the km trap
	//-

sys_post_mchk_trap:
	mfpr	r25, pt_misc		// Check for flag from mchk interrupt
	extwl	r25, 4, r25
	blbs	r25, sys_mchk_stack_done // Stack from already pushed if from interrupt flow
	
	bis	r14, r31, r12		// stash pointer to logout area
	mfpr	r14, pt10		// get exc_addr

	sll	r11, 63-3, r25		// get mode to msb
	bge	r25, sys_post_mchk_trap_30_	

	mtpr	r31, dtb_cm
	mtpr	r31, ips

	mtpr	r30, pt_usp		// save user stack
	mfpr	r30, pt_ksp

sys_post_mchk_trap_30_:	 
	lda	sp, 0-osfsf_c_size(sp)	// allocate stack space 	
	nop

	stq	r18, osfsf_a2(sp) 	// a2
	stq	r11, osfsf_ps(sp)	// save ps

	stq	r14, osfsf_pc(sp)	// save pc
	mfpr	r25, pt_entInt		// get the VA of the interrupt routine

	stq	r16, osfsf_a0(sp)	// a0
	lda	r16, osfint_c_mchk(r31)	// flag as mchk in a0

	stq	r17, osfsf_a1(sp)	// a1
	mfpr	r17, pt_misc		// get vector

	stq	r29, osfsf_gp(sp) 	// old gp
	mtpr	r25, exc_addr		// 

	or	r31, 7, r11		// get new ps (km, high ipl)
	subq	r31, 1, r18		// get a -1

	extwl	r17, 2, r17		// a1 <- interrupt vector
	bis	r31, ipl_machine_check, r25

	mtpr	r25, ev5__ipl		// Set internal ipl
	srl    	r18, 42, r18          	// shift off low bits of kseg addr

	sll    	r18, 42, r18          	// shift back into position
	mfpr	r29, pt_kgp		// get the kern r29

        or    	r12, r18, r18          	// EV4 algorithm - pass pointer to mchk frame as kseg address

	hw_rei_spe				// out to interrupt dispatch routine


	//+
	// The stack is pushed.  Load up a0,a1,a2 and vector via entInt
	//
	//-
	ALIGN_BRANCH
sys_mchk_stack_done:
	lda	r16, osfint_c_mchk(r31)	// flag as mchk/crd in a0
	lda	r17, scb_v_sysmchk(r31) // a1 <- interrupt vector

        subq    r31, 1, r18            // get a -1
	mfpr	r25, pt_entInt

        srl     r18, 42, r18           // shift off low bits of kseg addr
	mtpr	r25, exc_addr		// load interrupt vector

        sll     r18, 42, r18           // shift back into position
        or    	r14, r18, r18           // EV4 algorithm - pass pointer to mchk frame as kseg address
	
	hw_rei_spe			// done




//sys_double_machine_check - a machine check was started, but MCES<MCHK> was
//	already set.  We will now double machine check halt.
//
//	pt0 - old R0
//
	ALIGN_BLOCK
	.globl sys_double_machine_check
sys_double_machine_check:

//	pvc_jsr updpcb, bsr=1
//      bsr    r0, pal_update_pcb       // update the pcb
	lda	r0, hlt_c_dbl_mchk(r31)
	br	r31, sys_enter_console

//sys_machine_check_while_in_pal - a machine check was started, exc_addr points to
//	a PAL PC.  We will now machine check halt.
//
//	pt0 - old R0
//
sys_machine_check_while_in_pal:

//	pvc_jsr updpcb, bsr=1
//      bsr    r0, pal_update_pcb       // update the pcb
	lda	r0, hlt_c_mchk_from_pal(r31)
	br	r31, sys_enter_console

//ARITH and MCHK
//  Check for arithmetic errors and build trap frame,
//  but don't post the trap.
//  on entry:
//	pt10 - exc_addr
//	r12  - return address
//	r14  - logout frame pointer
//	r13 - available
//	r8,r9,r10 - available except across stq's
//	pt0,1,6 - available
//
//  on exit:
//	pt10 - new exc_addr
//	r17 = exc_mask
//	r16 = exc_sum
//	r14 - logout frame pointer
//
	ALIGN_BRANCH
sys_arith_and_mchk:
	mfpr	r13, ev5__exc_sum
	srl	r13, exc_sum_v_swc, r13
	bne	r13, handle_arith_and_mchk

//	pvc_jsr armc, bsr=1, dest=1
        ret     r31, (r12)              // return if no outstanding arithmetic error

handle_arith_and_mchk:
        mtpr    r31, ev5__dtb_cm        // Set Mbox current mode to kernel -
                                        //     no virt ref for next 2 cycles
	mtpr	r14, pt0	

	mtpr	r1, pt1			// get a scratch reg
	and     r11, osfps_m_mode, r1 // get mode bit

	bis     r11, r31, r25           // save ps
        beq     r1, handle_arith_and_mchk_10_                 // if zero we are in kern now

        bis     r31, r31, r25           // set the new ps
        mtpr    r30, pt_usp             // save user stack

        mfpr    r30, pt_ksp             // get kern stack
handle_arith_and_mchk_10_: 
        mfpr    r14, exc_addr           // get pc into r14 in case stack writes fault

	lda     sp, 0-osfsf_c_size(sp)  // allocate stack space
        mtpr    r31, ev5__ps            // Set Ibox current mode to kernel

        mfpr    r1, pt_entArith
        stq     r14, osfsf_pc(sp)       // save pc

        stq     r17, osfsf_a1(sp)
        mfpr    r17, ev5__exc_mask      // Get exception register mask IPR - no mtpr exc_sum in next cycle

        stq     r29, osfsf_gp(sp)
        stq     r16, osfsf_a0(sp)       // save regs

	bis	r13, r31, r16		// move exc_sum to r16
        stq     r18, osfsf_a2(sp)

        stq     r11, osfsf_ps(sp)       // save ps
        mfpr    r29, pt_kgp             // get the kern gp

	mfpr	r14, pt0		// restore logout frame pointer from pt0
        bis     r25, r31, r11           // set new ps

        mtpr    r1, pt10		// Set new PC
	mfpr	r1, pt1

//	pvc_jsr armc, bsr=1, dest=1
        ret     r31, (r12)              // return if no outstanding arithmetic error





// .sbttl	"SYS_ENTER_CONSOLE - Common PALcode for ENTERING console"

// SYS_enter_console
//
// Entry:
//	Entered when PAL wants to enter the console.
//	usually as the result of a HALT instruction or button,
//	or catastrophic error.
//
// Regs on entry...
//
//	R0 	= halt code
//	pt0	<- r0
//
// Function:
//
//	Save all readable machine state, and "call" the console
//	
// Returns:
//
//
// Notes:
//
//	In these routines, once the save state routine has been executed,
//	the remainder of the registers become scratchable, as the only
//	"valid" copy of them is the "saved" copy.
//
//	Any registers or PTs that are modified before calling the save 
//	routine will have there data lost. The code below will save all
//	state, but will loose pt 0,4,5.
//	
//-
#define KSEG 0xfffffc0000000000
	ALIGN_BLOCK
	.globl sys_enter_console
sys_enter_console:

	DEBUGSTORE(0x7a)
	DEBUG_EXC_ADDR()
	DEBUGSTORE(13)
	DEBUGSTORE(10)
	mtpr	r1, pt4
	mtpr	r3, pt5
	STALL
	STALL
	mfpr	r1, pt_impure
	bsr	r3, pal_save_state
	
	// build PS (IPL=7,CM=K,VMM=0,SW=0)
	lda	r11, 0x7(r31)	// Shadow copy of PS - kern mode, IPL=7
	lda	r1, 0x1e(r31)		
	mtpr	r1, ev5__ipl	// set internal <ipl>

	/* kernel sp: KSEG + 0xffe000 */
	subq	r31, 1, sp
	sll	sp, 42, sp
	lda	r3, 0xffe(zero)
	sll	r3, 12, r3
	addq	r3, sp, sp
	mtpr	sp, ptKsp		// Update the saved KSP value.
	mtpr	zero, ips		// Set mode to kernel, IPL = 0.
	mtpr	zero, dtb_ia		// Flush the DTB
	mtpr	zero, itb_ia		// Flush the ITB
	mtpr	zero, astrr		// Clear all ASTs ...
	mtpr	zero, aster
	mtpr	zero, sirr		// Clear all software interrupts.
//	lda	r3, pal_enter_console_ptr(r31) //find stored vector
//	ldqp	r1, 0(r3)
	/* put the KSEG address in the top, then add 0x10000 to it */
	subq	r31, 1, r1
	sll	r1, 42, r1

	ldah	r1, 1(r1)
	mtpr	r1, exc_addr
	mfpr	r1, pt4
	mfpr	r3, pt5
	STALL
	STALL
	hw_rei_stall

	.align 7
kludge_initial_pcbb:			// PCB is 128 bytes long
	.quad   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

	.globl debugstore
debugstore:
        stq_p   r16,0(zero)
        stq_p   r17,8(zero)
        lda     r16, 0x400(zero)
        sll     r16, 29, r16
        ldah    r16, 0x280(r16)
9:      lda     r17, 0x140(r16)
        ldl_p   r17, 0(r17)
        srl     r17, 16, r17
        and     r17, 0x20, r17
        beq     r17, 9b
        stl_p   r13, 0(r16)
        mb
        ldq_p   r17, 8(zero)
        ldq_p   r16, 0(zero)
	jmp	r31, (r25)

//
	 .globl put_exc_addr
put_exc_addr:
        stq_p   r16,0(zero)
        stq_p   r17,8(zero)
        stq_p   r18,16(zero)
        stq_p   r19,24(zero)

	mfpr	r16, exc_addr
	bis	r31, 64-4, r17		// shift count for quadword

1:	lda     r18, 0x400(zero)	// Wait for UART ready
        sll     r18, 29, r18
        ldah    r18, 0x280(r18)
9:      lda     r19, 0x140(r18)
        ldl_p   r19, 0(r19)
        srl     r19, 16, r19
        and     r19, 0x20, r19
        beq     r19, 9b
	
	srl	r16, r17, r19		// grab a nibble
	and	r19, 0x0f, r19
	addq	r19, 0x30, r19		// make ascii
	cmple	r19, 0x39, r18
	bne	r18, 0f
	addq	r19, 0x27, r19		// 0x07 gets A-F

0:	lda     r18, 0x400(zero)	// Wait for UART ready
        sll     r18, 29, r18
        ldah    r18, 0x280(r18)
	
        stl_p   r19, 0(r18)
        mb

	subq	r17, 4, r17		// do all nibbles
	bge	r17, 1b

        ldq_p   r16,0(zero)
        ldq_p   r17,8(zero)
        ldq_p   r18,16(zero)
        ldq_p   r19,24(zero)

	ret	r31, (r25)

	// print out r13
	 .globl put_hex
put_hex:
        stq_p   r16,0(zero)
        stq_p   r17,8(zero)
        stq_p   r18,16(zero)
        stq_p   r19,24(zero)

	bis	r13, zero, r16
	bis	r31, 64-4, r17		// shift count for quadword

1:	lda     r18, 0x400(zero)	// Wait for UART ready
        sll     r18, 29, r18
        ldah    r18, 0x280(r18)
9:      lda     r19, 0x140(r18)
        ldl_p   r19, 0(r19)
        srl     r19, 16, r19
        and     r19, 0x20, r19
        beq     r19, 9b
	
	srl	r16, r17, r19		// grab a nibble
	and	r19, 0x0f, r19
	addq	r19, 0x30, r19		// make ascii
	cmple	r19, 0x39, r18
	bne	r18, 0f
	addq	r19, 0x27, r19		// 0x07 gets A-F

0:	lda     r18, 0x400(zero)	// Wait for UART ready
        sll     r18, 29, r18
        ldah    r18, 0x280(r18)
	
        stl_p   r19, 0(r18)
        mb
	
	subq	r17, 4, r17		// do all nibbles
	bge	r17, 1b

        ldq_p   r16,0(zero)
        ldq_p   r17,8(zero)
        ldq_p   r18,16(zero)
        ldq_p   r19,24(zero)

	ret	r31, (r25)


/*
**
** FUNCTIONAL DESCRIPTION:
**
**	Transfer control to the specified address, passed in 
**      register a0, in PAL mode.
**
** INPUT PARAMETERS:
**
**      a0 (r16) - Transfer address
**
** OUTPUT PARAMETERS:
**
**   DECchip 21064 specific parameters:
**   
**      t0 (r1)  - bcCtl
**      t1 (r2)  - bcCfg
**
**   Firmware specific parameters:
**
**	s6 (r15) - Encoded srom.s RCS revision
**	a0 (r16) - Processor identification (a la SRM)
**      a1 (r17) - Size of good memory in bytes
**      a2 (r18) - Cycle count in picoseconds
**      a3 (r19) - Protocol signature and system revision
**      a4 (r20) - Active processor mask
**      a5 (r21) - System Context value
**
** SIDE EFFECTS:
**
*/
	ALIGN_BRANCH

Sys_Cserve_Jtopal:
	bic	a0, 3, t8		// Clear out low 2 bits of address
	bis	t8, 1, t8		// Or in PAL mode bit

	mfpr	t9, ptImpure		// Get base of impure scratch area.
        lda     t9, CNS_Q_IPR(t9)    	// Point to start of IPR area.

	RESTORE_SHADOW(a3,CNS_Q_SIGNATURE,t9)	// Get signature.
        srl     a3, 16, t0              // Shift signature into lower word.

	LDLI(t10,0xDECB)		// Load the expected valid signature.

        cmpeq   t0, t10, t0		// Check if saved signature was valid.
        blbc    t0, 1f                  // If invalid, pass nothing.
/*
** Load the processor specific parameters ...
*/
	RESTORE_SHADOW(t0,CNS_Q_BC_CTL,t9)	// Get bcCtl.
	RESTORE_SHADOW(t1,CNS_Q_BC_CFG,t9)	// Get bcCfg.
/*	RESTORE_SHADOW(t2,CNS_Q_BC_CFG_OFF,t9)	// Get bcCfg.*/
/*
** Load the firmware specific parameters ...
*/
	RESTORE_SHADOW(s6,CNS_Q_SROM_REV,t9)	// Get srom revision.
	RESTORE_SHADOW(a0,CNS_Q_PROC_ID,t9)	// Get processor id.
	RESTORE_SHADOW(a1,CNS_Q_MEM_SIZE,t9)	// Get memory size.
	RESTORE_SHADOW(a2,CNS_Q_CYCLE_CNT,t9)	// Get cycle count.
	RESTORE_SHADOW(a4,CNS_Q_PROC_MASK,t9)	// Get processor mask.
	RESTORE_SHADOW(a5,CNS_Q_SYSCTX,t9)	// Get system context.

	STALL
	STALL

1:	mtpr	zero, ptWhami		// Clear WHAMI and swap flag.
	mtpr	t8, excAddr		// Load the dispatch address.
	br	zero, 2f

	ALIGN_BLOCK

2:	NOP
	mtpr	zero, icFlush		// Flush the icache.
	NOP
	NOP

	NOP                           // Required NOPs ... 1-10
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP

	NOP                           // Required NOPs ... 11-20
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP

	NOP                           // Required NOPs ... 21-30
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP

	NOP                           // Required NOPs ... 31-40
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP



	NOP				// Required NOPs ... 41-44
	NOP
	NOP
	NOP

	hw_rei				// Dispatch in PAL mode ...
