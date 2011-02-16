/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
 * Copyright (c) 1993 Hewlett-Packard Development Company
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
 *
 * Authors: Ali G. Saidi
 *          Nathan L. Binkert
 */

#define max_cpuid 1
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
#include "tlaser.h"

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

#if defined(BIG_TSUNAMI)
#define MAXPROC         0x3f
#define IPIQ_addr       0x800
#define IPIQ_shift      0
#define IPIR_addr       0x840
#define IPIR_shift      0
#define RTC_addr        0x880
#define RTC_shift       0
#define DIR_addr        0xa2
#elif defined(TSUNAMI)
#define MAXPROC         0x3
#define IPIQ_addr       0x080
#define IPIQ_shift      12
#define IPIR_addr       0x080
#define IPIR_shift      8
#define RTC_addr        0x080
#define RTC_shift       4
#define DIR_addr        0xa0
#elif defined(TLASER)
#define MAXPROC         0xf
#else
#error Must define BIG_TSUNAMI, TSUNAMI, or TLASER
#endif

#define ALIGN_BLOCK \
        .align 5

#define ALIGN_BRANCH \
        .align 3

#define EXPORT(_x)	\
        .align 5;	\
        .globl _x;	\
_x:

// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// XXX the following is 'made up'
// XXX bugnion

// XXX bugnion not sure how to align 'quad'
#define ALIGN_QUAD \
        .align  3

#define ALIGN_128 \
        .align  7


#define GET_IMPURE(_r) mfpr _r,pt_impure
#define GET_ADDR(_r1,_off,_r2)  lda _r1,_off(_r2)


#define BIT(_x) (1<<(_x))


// System specific code - beh model version
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
//
//
// Macro to read TLINTRSUMx
//
// Based on the CPU_NUMBER, read either the TLINTRSUM0 or TLINTRSUM1 register
//
// Assumed register usage:
//   rsum TLINTRSUMx contents
//   raddr node space address
//   scratch scratch register
//
#define Read_TLINTRSUMx(_rsum, _raddr, _scratch)	                  \
    nop;                                                                  \
    mfpr  _scratch, pt_whami;      /* Get our whami (VID) */              \
    extbl _scratch, 1, _scratch;   /* shift down to bit 0 */              \
    lda	  _raddr, 0xff88(zero);    /* Get base node space address bits */ \
    sll	  _raddr, 24, _raddr;      /* Shift up to proper position */      \
    srl	  _scratch, 1, _rsum;      /* Shift off the cpu number */         \
    sll   _rsum, 22, _rsum;        /* Get our node offset */		  \
    addq  _raddr, _rsum, _raddr;   /* Get our base node space address */  \
    blbs  _scratch, 1f;                                                   \
    lda	  _raddr, 0x1180(_raddr);                                         \
    br	  r31, 2f;                                                        \
1:  lda	  _raddr, 0x11c0(_raddr);                                         \
2:  ldl_p _rsum, 0(_raddr)         /* read the right tlintrsum reg */

//
// Macro to write TLINTRSUMx
//
//  Based on the CPU_NUMBER, write either the TLINTRSUM0 or TLINTRSUM1 register
//
// Assumed register usage:
//   rsum TLINTRSUMx write data
//   raddr node space address
//   scratch scratch register
//
#define Write_TLINTRSUMx(_rsum,_raddr,_whami)                              \
    nop;                                                                   \
    mfpr  _whami, pt_whami;       /* Get our whami (VID) */                \
    extbl _whami, 1, _whami;      /* shift down to bit 0 */                \
    lda   _raddr, 0xff88(zero);   /* Get base node space address bits */   \
    sll   _raddr, 24, _raddr;     /* Shift up to proper position */        \
    blbs  _whami, 1f;                                                      \
    lda   _raddr, 0x1180(_raddr);                                          \
    br    zero, 2f;                                                        \
1:  lda   _raddr, 0x11c0(_raddr);                                          \
2:  srl	  _whami, 1, _whami;      /* Get our node offset */                \
    addq  _raddr, _whami, _raddr; /* Get our base node space address */    \
    mb;                                                                    \
    stq_p _rsum, 0(_raddr);       /* write the right tlintrsum reg */      \
    ldq_p _rsum, 0(_raddr);       /* dummy read to tlintrsum */            \
    bis   _rsum, _rsum, _rsum     /* needed to complete the ldqp above */


//
// Macro to determine highest priority TIOP Node ID from interrupt pending mask
//
// Assumed register usage:
//  rmask - TLINTRSUMx contents, shifted to isolate IOx bits
//  rid - TLSB Node ID of highest TIOP
//
#define Intr_Find_TIOP(_rmask,_rid)              \
    srl  _rmask,3,_rid;    /* check IOP8 */      \
    blbc _rid,1f;          /* not IOP8 */        \
    lda  _rid,8(zero);     /* IOP8 */            \
    br   zero,6f;                                \
1:  srl  _rmask,3,_rid;    /* check IOP7 */      \
    blbc _rid, 2f;         /* not IOP7 */        \
    lda  _rid, 7(r31);     /* IOP7 */            \
    br   r31, 6f;                                \
2:  srl  _rmask, 2, _rid;  /* check IOP6 */      \
    blbc _rid, 3f;         /* not IOP6 */        \
    lda  _rid, 6(r31);     /* IOP6 */            \
    br   r31, 6f;                                \
3:  srl  _rmask, 1, _rid;  /* check IOP5 */      \
    blbc _rid, 4f;         /* not IOP5 */        \
    lda  _rid, 5(r31);     /* IOP5 */            \
    br   r31, 6f;                                \
4:  srl  _rmask, 0, _rid;  /* check IOP4 */      \
    blbc _rid, 5f;         /* not IOP4 */        \
    lda  r14, 4(r31);      /* IOP4 */            \
    br   r31, 6f;                                \
5:  lda  r14, 0(r31);      /* passive release */ \
6:

//
// Macro to calculate base node space address for given node id
//
// Assumed register usage:
//  rid - TLSB node id
//  raddr - base node space address
#define Get_TLSB_Node_Address(_rid,_raddr)  \
    sll  _rid, 22, _rid;                    \
    lda  _raddr, 0xff88(zero);              \
    sll  _raddr, 24, _raddr;                \
    addq _raddr, _rid, _raddr


#define OSFmchk_TLEPstore_1(_rlog,_rs,_rs1,_nodebase,_tlepreg)     \
    lda   _rs1, tlep_##_tlepreg(zero);                             \
    or    _rs1, _nodebase, _rs1;                                   \
    ldl_p _rs1, 0(_rs1);                                           \
    stl_p _rs, mchk_##_tlepreg(_rlog)   /* store in frame */

#define OSFmchk_TLEPstore(_tlepreg) \
    OSFmchk_TLEPstore_1(r14,r8,r4,r13,_tlepreg)

#define OSFcrd_TLEPstore_1(_rlog,_rs,_rs1,_nodebase,_tlepreg) 		\
        lda	_rs1, tlep_##_tlepreg(zero);				\
        or	_rs1, _nodebase, _rs1;  				\
        ldl_p	_rs1, 0(_rs1);						\
        stl_p	_rs, mchk_crd_##_tlepreg(_rlog)

#define OSFcrd_TLEPstore_tlsb_1(_rlog,_rs,_rs1,_nodebase,_tlepreg) 	\
        lda	_rs1, tlsb_##_tlepreg(zero);				\
        or	_rs1, _nodebase, _rs1;  				\
        ldl_p	_rs1, 0(_rs1);						\
        stl_p	_rs,mchk_crd_##_tlepreg(_rlog)

#define OSFcrd_TLEPstore_tlsb_clr_1(_rlog,_rs,_rs1,_nodebase,_tlepreg) 	\
        lda	_rs1,tlsb_##_tlepreg(zero);				\
        or	_rs1, _nodebase,_rs1;  					\
        ldl_p	_rs1, 0(_rs1);						\
        stl_p	_rs, mchk_crd_##_tlepreg(_rlog);			\
        stl_p   _rs, 0(_rs1)

#define OSFcrd_TLEPstore(_tlepreg) \
    OSFcrd_TLEPstore_1(r14,r8,r4,r13,_tlepreg)
#define OSFcrd_TLEPstore_tlsb(_tlepreg) \
    OSFcrd_TLEPstore_tlsb_1(r14,r8,r4,r13,_tlepreg)
#define OSFcrd_TLEPstore_tlsb_clr(_tlepreg) \
    OSFcrd_TLEPstore_tlsb_clr_1(r14,r8,r4,r13,_tlepreg)


#define save_pcia_intr(_irq)                                            \
    and   r13, 0xf, r25;            /* isolate low 4 bits */            \
    addq  r14, 4, r14;              /* format the TIOP Node id field */ \
    sll   r14, 4, r14;              /* shift the TIOP Node id */        \
    or    r14, r25, r10;            /* merge Node id/hose/HPC */        \
    mfpr  r14, pt14;                /* get saved value */               \
    extbl r14, _irq, r25;           /* confirm none outstanding */      \
    bne   r25, sys_machine_check_while_in_pal;                          \
    insbl r10, _irq, r10;           /* align new info */                \
    or    r14, r10, r14;            /* merge info */                    \
    mtpr  r14, pt14;                /* save it */                       \
    bic   r13, 0xf, r13             /* clear low 4 bits of vector */


// wripir - PALcode for wripir instruction
// R16 has the processor number.
//
        ALIGN_BLOCK
EXPORT(sys_wripir)
    //
    // Convert the processor number to a CPU mask
    //
    and   r16, MAXPROC, r14	// mask the top stuff: MAXPROC+1 CPUs supported
    bis   r31, 0x1, r16		// get a one
    sll   r16, r14, r14		// shift the bit to the right place
#if defined(TSUNAMI) || defined(BIG_TSUNAMI)
    sll   r14,IPIQ_shift,r14
#endif


    //
    // Build the Broadcast Space base address
    //
#if defined(TSUNAMI) || defined(BIG_TSUNAMI)
    lda   r16,0xf01(r31)
    sll   r16,32,r16
    ldah  r13,0xa0(r31)
    sll   r13,8,r13
    bis   r16,r13,r16
    lda   r16,IPIQ_addr(r16)
#elif defined(TLASER)
    lda   r13, 0xff8e(r31)	// Load the upper address bits
    sll   r13, 24, r13		// shift them to the top
#endif

    //
    // Send out the IP Intr
    //
#if defined(TSUNAMI) || defined(BIG_TSUNAMI)
    stq_p r14, 0(r16)		// Tsunami MISC Register
#elif defined(TLASER)
    stq_p r14, 0x40(r13)	// Write to TLIPINTR reg
#endif
    wmb				// Push out the store
    hw_rei


// cflush - PALcode for CFLUSH instruction
//
// SYS_CFLUSH
// Entry:
//	R16 - contains the PFN of the page to be flushed
//
// Function:
//	Flush all Dstream caches of 1 entire page
//
//
        ALIGN_BLOCK
EXPORT(sys_cflush)

//      #convert pfn to addr, and clean off <63:20>
//      #sll	r16, <page_offset_size_bits>+<63-20>>, r12
        sll	r16, page_offset_size_bits+(63-20),r12

//      #ldah	r13,<<1@22>+32768>@-16(r31)// + xxx<31:16>
//      # stolen from srcmax code. XXX bugnion
        lda	r13, 0x10(r31)				   // assume 16Mbytes of cache
        sll	r13, 20, r13				   // convert to bytes


        srl	r12, 63-20, r12	// shift back to normal position
        xor	r12, r13, r12		// xor addr<18>

        or	r31, 8192/(32*8), r13	// get count of loads
        nop

cflush_loop:
        subq	r13, 1, r13		// decr counter
        mfpr    r25, ev5__intid         // Fetch level of interruptor

        ldq_p	r31, 32*0(r12)		// do a load
        ldq_p	r31, 32*1(r12)		// do next load

        ldq_p	r31, 32*2(r12)		// do next load
        ldq_p	r31, 32*3(r12)		// do next load

        ldq_p	r31, 32*4(r12)		// do next load
        ldq_p	r31, 32*5(r12)		// do next load

        ldq_p	r31, 32*6(r12)		// do next load
        ldq_p	r31, 32*7(r12)		// do next load

        mfpr    r14, ev5__ipl           // Fetch current level
        lda	r12, (32*8)(r12)	// skip to next cache block addr

        cmple   r25, r14, r25           // R25 = 1 if intid .less than or eql ipl
        beq	r25, 1f		// if any int's pending, re-queue CFLUSH -- need to check for hlt interrupt???

        bne	r13, cflush_loop 	// loop till done
        hw_rei				// back to user

        ALIGN_BRANCH
1:					// Here if interrupted
        mfpr	r12, exc_addr
        subq	r12, 4, r12		// Backup PC to point to CFLUSH

        mtpr	r12, exc_addr
        nop

        mfpr	r31, pt0		// Pad exc_addr write
        hw_rei


        ALIGN_BLOCK
//
// sys_cserve - PALcode for CSERVE instruction
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
//	r0 = 1	ldq_p
//	r0 = 2	stq_p
//		args, are as for normal STQ_P/LDQ_P in VMS PAL
//
//	r0 = 3	dump_tb's
//	r16 = detination PA to dump tb's to.
//
//	r0<0> = 1, success
//	r0<0> = 0, failure, or option not supported
//	r0<63:1> = (generally 0, but may be function dependent)
//	r0 - load data on ldq_p
//
//
EXPORT(sys_cserve)

        /* taken from scrmax */
        cmpeq	r18, CSERVE_K_RD_IMPURE, r0
        bne	r0, Sys_Cserve_Rd_Impure

        cmpeq	r18, CSERVE_K_JTOPAL, r0
        bne	r0, Sys_Cserve_Jtopal
        call_pal        0

        or	r31, r31, r0
        hw_rei				// and back we go

Sys_Cserve_Rd_Impure:
        mfpr	r0, pt_impure		// Get base of impure scratch area.
        hw_rei

        ALIGN_BRANCH

Sys_Cserve_Jtopal:
        bic	a0, 3, t8		// Clear out low 2 bits of address
        bis	t8, 1, t8		// Or in PAL mode bit
        mtpr    t8,exc_addr
        hw_rei

        // ldq_p
        ALIGN_QUAD
1:
        ldq_p	r0,0(r17)		// get the data
        nop				// pad palshadow write

        hw_rei				// and back we go


        // stq_p
        ALIGN_QUAD
2:
        stq_p	r18, 0(r17)		// store the data
        lda     r0,17(r31) // bogus
        hw_rei				// and back we go


        ALIGN_QUAD
csrv_callback:
        ldq	r16, 0(r17)		// restore r16
        ldq	r17, 8(r17)		// restore r17
        lda	r0, hlt_c_callback(r31)
        br	r31, sys_enter_console


csrv_identify:
        mfpr	r0, pal_base
        ldq_p	r0, 8(r0)
        hw_rei


// dump tb's
        ALIGN_QUAD
0:
        // DTB PTEs - 64 entries
        addq	r31, 64, r0		// initialize loop counter
        nop

1:	mfpr	r12, ev5__dtb_pte_temp	// read out next pte to temp
        mfpr	r12, ev5__dtb_pte	// read out next pte to reg file

        subq	r0, 1, r0		// decrement loop counter
        nop				// Pad - no Mbox instr in cycle after mfpr

        stq_p	r12, 0(r16)		// store out PTE
        addq	r16, 8 ,r16		// increment pointer

        bne	r0, 1b

        ALIGN_BRANCH
        // ITB PTEs - 48 entries
        addq	r31, 48, r0		// initialize loop counter
        nop

2:	mfpr	r12, ev5__itb_pte_temp	// read out next pte to temp
        mfpr	r12, ev5__itb_pte	// read out next pte to reg file

        subq	r0, 1, r0		// decrement loop counter
        nop				//

        stq_p	r12, 0(r16)		// store out PTE
        addq	r16, 8 ,r16		// increment pointer

        bne	r0, 2b
        or	r31, 1, r0		// set success

        hw_rei				// and back we go


//
// SYS_INTERRUPT  - Interrupt processing code
//
//	Current state:
//		Stack is pushed
//		ps, sp and gp are updated
//		r12, r14 - available
//		r13 - INTID (new EV5 IPL)
//		r25 - ISR
//		r16, r17, r18 - available
//
//
EXPORT(sys_interrupt)
    cmpeq  r13, 31, r12			// Check for level 31 interrupt
    bne    r12, sys_int_mchk_or_crd	// machine check or crd

    cmpeq  r13, 30, r12			// Check for level 30 interrupt
    bne    r12, sys_int_powerfail	// powerfail

    cmpeq  r13, 29, r12			// Check for level 29 interrupt
    bne    r12, sys_int_perf_cnt	// performance counters

    cmpeq  r13, 23, r12			// Check for level 23 interrupt
    bne    r12, sys_int_23 		// IPI in Tsunami

    cmpeq  r13, 22, r12			// Check for level 22 interrupt
    bne    r12, sys_int_22 		// timer interrupt

    cmpeq  r13, 21, r12			// Check for level 21 interrupt
    bne    r12, sys_int_21	 	// I/O

    cmpeq  r13, 20, r12			// Check for level 20 interrupt
    bne    r12, sys_int_20		// system error interrupt
                                        // (might be corrected)

    mfpr   r14, exc_addr		// ooops, something is wrong
    br     r31, pal_pal_bug_check_from_int


//
//sys_int_2*
//	Routines to handle device interrupts at IPL 23-20.
//	System specific method to ack/clear the interrupt, detect passive
//      release, detect interprocessor (22),  interval clock (22),  corrected
//	system error (20)
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
//

#if defined(TSUNAMI) || defined(BIG_TSUNAMI)
        ALIGN_BRANCH
sys_int_23:
        or      r31,0,r16                        // IPI interrupt A0 = 0
        lda     r12,0xf01(r31)                   // build up an address for the MISC register
        sll     r12,16,r12
        lda     r12,0xa000(r12)
        sll     r12,16,r12
        lda     r12,IPIR_addr(r12)

        mfpr    r10, pt_whami                   // get CPU ID
        extbl	r10, 1, r10		        // Isolate just whami bits
        or      r31,0x1,r14                     // load r14 with bit to clear
        sll     r14,r10,r14                     // left shift by CPU ID
        sll     r14,IPIR_shift,r14
        stq_p   r14, 0(r12)                     // clear the ipi interrupt

        br	r31, pal_post_interrupt		// Notify the OS


        ALIGN_BRANCH
sys_int_22:
        or      r31,1,r16                       // a0 means it is a clock interrupt
        lda     r12,0xf01(r31)                  // build up an address for the MISC register
        sll     r12,16,r12
        lda     r12,0xa000(r12)
        sll     r12,16,r12
        lda     r12,RTC_addr(r12)

        mfpr    r10, pt_whami                   // get CPU ID
        extbl	r10, 1, r10		        // Isolate just whami bits
        or      r31,0x1,r14                     // load r14 with bit to clear
        sll     r14,r10,r14                     // left shift by CPU ID
        sll     r14,RTC_shift,r14               // put the bits in the right position
        stq_p   r14, 0(r12)                     // clear the rtc interrupt

        br	r31, pal_post_interrupt		// Tell the OS


        ALIGN_BRANCH
sys_int_20:
        Read_TLINTRSUMx(r13,r10,r14)		// read the right TLINTRSUMx
        srl	r13, 12, r13			// shift down to examine IPL15

        Intr_Find_TIOP(r13,r14)
        beq	r14, 1f

        Get_TLSB_Node_Address(r14,r10)
        lda	r10, 0xa40(r10)	// Get base TLILID address

        ldl_p	r13, 0(r10)			// Read the TLILID register
        bne	r13, pal_post_dev_interrupt
        beq	r13, 1f

        and	r13, 0x3, r10			// check for PCIA bits
        beq	r10, pal_post_dev_interrupt	// done if nothing set
        save_pcia_intr(1)
        br	r31, pal_post_dev_interrupt	//

1:	lda	r16, osfint_c_passrel(r31)	// passive release
        br	r31, pal_post_interrupt		//


        ALIGN_BRANCH
sys_int_21:

    lda     r12,0xf01(r31)                // calculate DIRn address
    sll     r12,32,r12
    ldah    r13,DIR_addr(r31)
    sll	    r13,8,r13
    bis	    r12,r13,r12

    mfpr    r13, pt_whami                   // get CPU ID
    extbl   r13, 1, r13		            // Isolate just whami bits

#ifdef BIG_TSUNAMI
    sll     r13,4,r13
    or      r12,r13,r12
#else
    lda     r12,0x0080(r12)
    and     r13,0x1,r14                     // grab LSB and shift left 6
    sll     r14,6,r14
    and     r13,0x2,r10                     // grabl LSB+1 and shift left 9
    sll     r10,9,r10

    mskbl   r12,0,r12                       // calculate DIRn address
    lda     r13,0x280(r31)
    bis     r12,r13,r12
    or      r12,r14,r12
    or      r12,r10,r12
#endif

    ldq_p    r13, 0(r12)                     // read DIRn

    or      r31,1,r14                       // set bit 55 (ISA Interrupt)
    sll     r14,55,r14

    and     r13, r14, r14                    // check if bit 55 is set
    lda     r16,0x900(r31)                  // load offset for normal into r13
    beq     r14, normal_int                 // if not compute the vector normally

    lda     r16,0x800(r31)                  // replace with offset for pic
    lda     r12,0xf01(r31)                   // build an addr to access PIC
    sll     r12,32,r12                        // at f01fc000000
    ldah    r13,0xfc(r31)
    sll	    r13,8,r13
    bis	    r12,r13,r12
    ldq_p    r13,0x0020(r12)                   // read PIC1 ISR for interrupting dev

normal_int:
    //ctlz    r13,r14                          // count the number of leading zeros
    // EV5 doesn't have ctlz, but we do, so lets use it
    .byte 0x4e
    .byte 0x06
    .byte 0xed
    .byte 0x73
    lda     r10,63(r31)
    subq    r10,r14,r17                     // subtract from

    lda	    r13,0x10(r31)
    mulq    r17,r13,r17                    // compute 0x900 + (0x10 * Highest DIRn-bit)
    addq    r17,r16,r17

    or      r31,3,r16                       // a0 means it is a I/O interrupt

    br      r31, pal_post_interrupt

#elif defined(TLASER)
        ALIGN_BRANCH
sys_int_23:
        Read_TLINTRSUMx(r13,r10,r14)		// read the right TLINTRSUMx
        srl	r13, 22, r13			// shift down to examine IPL17

        Intr_Find_TIOP(r13,r14)
        beq	r14, 1f

        Get_TLSB_Node_Address(r14,r10)
        lda	r10, 0xac0(r10)	// Get base TLILID address

        ldl_p	r13, 0(r10)			// Read the TLILID register
        bne	r13, pal_post_dev_interrupt

1:	lda	r16, osfint_c_passrel(r31)	// passive release
        br	r31, pal_post_interrupt		//


        ALIGN_BRANCH
sys_int_22:
        Read_TLINTRSUMx(r13,r10,r14)		// read the right TLINTRSUMx
        srl	r13, 6, r14			// check the Intim bit

        blbs	r14, tlep_intim			// go service Intim
        srl	r13, 5, r14			// check the IP Int bit

        blbs	r14, tlep_ipint			// go service IP Int
        srl	r13, 17, r13			// shift down to examine IPL16

        Intr_Find_TIOP(r13,r14)
        beq	r14, 1f

        Get_TLSB_Node_Address(r14,r10)
        lda	r10, 0xa80(r10)	// Get base TLILID address

        ldl_p	r13, 0(r10)			// Read the TLILID register
        bne	r13, pal_post_dev_interrupt
        beq	r13, 1f

        and	r13, 0x3, r10			// check for PCIA bits
        beq	r10, pal_post_dev_interrupt	// done if nothing set
        save_pcia_intr(2)
        br	r31, pal_post_dev_interrupt	//

1:	lda	r16, osfint_c_passrel(r31)	// passive release
        br	r31, pal_post_interrupt		//


        ALIGN_BRANCH
sys_int_21:
        Read_TLINTRSUMx(r13,r10,r14)		// read the right TLINTRSUMx
        srl	r13, 12, r13			// shift down to examine IPL15

        Intr_Find_TIOP(r13,r14)
        beq	r14, 1f

        Get_TLSB_Node_Address(r14,r10)
        lda	r10, 0xa40(r10)	// Get base TLILID address

        ldl_p	r13, 0(r10)			// Read the TLILID register
        bne	r13, pal_post_dev_interrupt
        beq	r13, 1f

        and	r13, 0x3, r10			// check for PCIA bits
        beq	r10, pal_post_dev_interrupt	// done if nothing set
        save_pcia_intr(1)
        br	r31, pal_post_dev_interrupt	//

1:	lda	r16, osfint_c_passrel(r31)	// passive release
        br	r31, pal_post_interrupt		//


        ALIGN_BRANCH
sys_int_20:
        lda	r13, 1(r31)			// Duart0 bit
        Write_TLINTRSUMx(r13,r10,r14)		// clear the duart0 bit

        Read_TLINTRSUMx(r13,r10,r14)		// read the right TLINTRSUMx
        blbs	r13, tlep_uart0			// go service UART int

        srl	r13, 7, r13			// shift down to examine IPL14
        Intr_Find_TIOP(r13,r14)

        beq	r14, tlep_ecc			// Branch if not IPL14
        Get_TLSB_Node_Address(r14,r10)

        lda	r10, 0xa00(r10)	                // Get base TLILID0 address
        ldl_p	r13, 0(r10)			// Read the TLILID register

        bne	r13, pal_post_dev_interrupt
        beq	r13, 1f

        and	r13, 0x3, r10			// check for PCIA bits
        beq	r10, pal_post_dev_interrupt	// done if nothing set
        save_pcia_intr(0)
        br	r31, pal_post_dev_interrupt	//
1:	lda	r16, osfint_c_passrel(r31)	// passive release
        br	r31, pal_post_interrupt		//


        ALIGN_BRANCH
tlep_intim:
        lda	r13, 0xffb(r31)			// get upper GBUS address bits
        sll	r13, 28, r13			// shift up to top

        lda	r13, (0x300)(r13)  // full CSRC address (tlep watch csrc offset)
        ldq_p	r13, 0(r13)			// read CSRC

        lda	r13, 0x40(r31)			// load Intim bit
        Write_TLINTRSUMx(r13,r10,r14)		// clear the Intim bit

        lda	r16, osfint_c_clk(r31)		// passive release
        br	r31, pal_post_interrupt		// Build the stack frame


        ALIGN_BRANCH
tlep_ipint:
        lda	r13, 0x20(r31)			// load IP Int bit
        Write_TLINTRSUMx(r13,r10,r14)		// clear the IP Int bit

        lda	r16, osfint_c_ip(r31)		// passive release
        br	r31, pal_post_interrupt		// Build the stack frame


        ALIGN_BRANCH
tlep_uart0:
        lda	r13, 0xffa(r31)			// get upper GBUS address bits
        sll	r13, 28, r13			// shift up to top

        ldl_p	r14, 0x80(r13)			// zero pointer register
        lda	r14, 3(r31)			// index to RR3

        stl_p	r14, 0x80(r13)			// write pointer register
        mb

        mb
        ldl_p	r14, 0x80(r13)			// read RR3

        srl	r14, 5, r10			// is it Channel A RX?
        blbs	r10, uart0_rx

        srl	r14, 4, r10			// is it Channel A TX?
        blbs	r10, uart0_tx

        srl	r14, 2, r10			// is it Channel B RX?
        blbs	r10, uart1_rx

        srl	r14, 1, r10			// is it Channel B TX?
        blbs	r10, uart1_tx

        lda	r8, 0(r31)			// passive release
        br	r31, clear_duart0_int		// clear tlintrsum and post


        ALIGN_BRANCH
uart0_rx:
        lda	r8, 0x680(r31)			// UART0 RX vector
        br	r31, clear_duart0_int		// clear tlintrsum and post


        ALIGN_BRANCH
uart0_tx:
        lda	r14, 0x28(r31)			// Reset TX Int Pending code
        mb
        stl_p	r14, 0x80(r13)			// write Channel A WR0
        mb

        lda	r8, 0x6c0(r31)			// UART0 TX vector
        br	r31, clear_duart0_int		// clear tlintrsum and post


        ALIGN_BRANCH
uart1_rx:
        lda	r8, 0x690(r31)			// UART1 RX vector
        br	r31, clear_duart0_int		// clear tlintrsum and post


        ALIGN_BRANCH
uart1_tx:
        lda	r14, 0x28(r31)			// Reset TX Int Pending code
        stl_p	r14, 0(r13)			// write Channel B WR0

        lda	r8, 0x6d0(r31)			// UART1 TX vector
        br	r31, clear_duart0_int		// clear tlintrsum and post


        ALIGN_BRANCH
clear_duart0_int:
        lda	r13, 1(r31)			// load duart0 bit
        Write_TLINTRSUMx(r13,r10,r14)		// clear the duart0 bit

        beq	r8, 1f
        or	r8, r31, r13			// move vector to r13
        br	r31, pal_post_dev_interrupt	// Build the stack frame
1:	nop
        nop
        hw_rei
//	lda	r16, osfint_c_passrel(r31)	// passive release
//	br	r31, pal_post_interrupt		//


        ALIGN_BRANCH
tlep_ecc:
        mfpr	r14, pt_whami			// get our node id
        extbl	r14, 1, r14			// shift to bit 0

        srl	r14, 1, r14			// shift off cpu number
        Get_TLSB_Node_Address(r14,r10)		// compute our nodespace address

        ldl_p	r13, 0x40(r10)	// read our TLBER WAS tlsb_tlber_offset
        srl	r13, 17, r13			// shift down the CWDE/CRDE bits

        and	r13, 3, r13			// mask the CWDE/CRDE bits
        beq	r13, 1f

        ornot	r31, r31, r12			// set flag
        lda	r9, mchk_c_sys_ecc(r31)		// System Correctable error MCHK code
        br	r31, sys_merge_sys_corr		// jump to CRD logout frame code

1:	lda	r16, osfint_c_passrel(r31)	// passive release

#endif // if TSUNAMI || BIG_TSUNAMI elif TLASER

        ALIGN_BRANCH
pal_post_dev_interrupt:
        or	r13, r31, r17			// move vector to a1
        or	r31, osfint_c_dev, r16		// a0 signals IO device interrupt

pal_post_interrupt:
        mfpr	r12, pt_entint

        mtpr	r12, exc_addr

        nop
        nop

        hw_rei_spe


//
// sys_passive_release
//	Just pretend the interrupt never occurred.
//

EXPORT(sys_passive_release)
        mtpr	r11, ev5__dtb_cm	// Restore Mbox current mode for ps
        nop

        mfpr	r31, pt0		// Pad write to dtb_cm
        hw_rei

//
// sys_int_powerfail
//	A powerfail interrupt has been detected.  The stack has been pushed.
//	IPL and PS are updated as well.
//
//	I'm not sure what to do here, I'm treating it as an IO device interrupt
//
//

        ALIGN_BLOCK
sys_int_powerfail:
        lda	r12, 0xffc4(r31)		// get GBUS_MISCR address bits
        sll	r12, 24, r12			// shift to proper position
        ldq_p	r12, 0(r12)			// read GBUS_MISCR
        srl	r12, 5, r12			// isolate bit <5>
        blbc	r12, 1f 			// if clear, no missed mchk

                                                // Missed a CFAIL mchk
        lda	r13, 0xffc7(r31)		// get GBUS$SERNUM address bits
        sll	r13, 24, r13			// shift to proper position
        lda	r14, 0x40(r31)			// get bit <6> mask
        ldq_p	r12, 0(r13)			// read GBUS$SERNUM
        or	r12, r14, r14			// set bit <6>
        stq_p	r14, 0(r13)			// clear GBUS$SERNUM<6>
        mb
        mb

1:	br	r31, sys_int_mchk		// do a machine check

        lda	r17, scb_v_pwrfail(r31)	// a1 to interrupt vector
        mfpr	r25, pt_entint

        lda	r16, osfint_c_dev(r31)	// a0 to device code
        mtpr	r25, exc_addr

        nop				// pad exc_addr write
        nop

        hw_rei_spe

//
// sys_halt_interrupt
//       A halt interrupt has been detected.  Pass control to the console.
//
//
//
        EXPORT(sys_halt_interrupt)

        ldah	r13, 0x1800(r31)		// load Halt/^PHalt bits
        Write_TLINTRSUMx(r13,r10,r14)		// clear the ^PHalt bits

        mtpr	r11, dtb_cm		// Restore Mbox current mode
        nop
        nop
        mtpr	r0, pt0
        lda     r0, hlt_c_hw_halt(r31)  // set halt code to hw halt
        br      r31, sys_enter_console  // enter the console



//
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
//
        ALIGN_BLOCK
sys_int_mchk_or_crd:
        srl	r25, isr_v_mck, r12
        blbs	r12, sys_int_mchk
        //
        // Not a Machine check interrupt, so must be an Internal CRD interrupt
        //

        mb					//Clear out Cbox prior to reading IPRs
        srl 	r25, isr_v_crd, r13		//Check for CRD
        blbc	r13, pal_pal_bug_check_from_int	//If CRD not set, shouldn't be here!!!

        lda	r9, 1(r31)
        sll 	r9, hwint_clr_v_crdc, r9	// get ack bit for crd
        mtpr	r9, ev5__hwint_clr		// ack the crd interrupt

        or	r31, r31, r12			// clear flag
        lda	r9, mchk_c_ecc_c(r31)		// Correctable error MCHK code

sys_merge_sys_corr:
        ldah	r14, 0xfff0(r31)
        mtpr   	r0, pt0				// save r0 for scratch
        zap	r14, 0xE0, r14			// Get Cbox IPR base
        mtpr   	r1, pt1				// save r0 for scratch

        ldq_p	r0, ei_addr(r14)		// EI_ADDR IPR
        ldq_p	r10, fill_syn(r14)		// FILL_SYN IPR
        bis	r0, r10, r31			// Touch lds to make sure they complete before doing scrub

        blbs	r12, 1f				// no scrubbing for IRQ0 case
// XXX bugnion	pvc_jsr	crd_scrub_mem, bsr=1
        bsr	r13, sys_crd_scrub_mem		// and go scrub

                                                // ld/st pair in scrub routine will have finished due
                                                // to ibox stall of stx_c.  Don't need another mb.
        ldq_p	r8, ei_stat(r14)		// EI_STAT, unlock EI_ADDR, BC_TAG_ADDR, FILL_SYN
        or	r8, r31, r12			// Must only be executed once in this flow, and must
        br	r31, 2f				// be after the scrub routine.

1:	ldq_p	r8, ei_stat(r14)		// EI_STAT, unlock EI_ADDR, BC_TAG_ADDR, FILL_SYN
                                                // For IRQ0 CRD case only - meaningless data.

2:	mfpr	r13, pt_mces			// Get MCES
        srl	r12, ei_stat_v_ei_es, r14	// Isolate EI_STAT:EI_ES
        blbc	r14, 6f			// branch if 630
        srl	r13, mces_v_dsc, r14		// check if 620 reporting disabled
        blbc	r14, 5f				// branch if enabled
        or	r13, r31, r14			// don't set SCE if disabled
        br	r31, 8f			// continue
5:	bis	r13, BIT(mces_v_sce), r14	// Set MCES<SCE> bit
        br	r31, 8f

6:     	srl	r13, mces_v_dpc, r14		// check if 630 reporting disabled
        blbc	r14, 7f			// branch if enabled
        or	r13, r31, r14			// don't set PCE if disabled
        br	r31, 8f			// continue
7:	bis	r13, BIT(mces_v_pce), r14	// Set MCES<PCE> bit

        // Setup SCB if dpc is not set
8:	mtpr	r14, pt_mces			// Store updated MCES
        srl	r13, mces_v_sce, r1		// Get SCE
        srl	r13, mces_v_pce, r14		// Get PCE
        or	r1, r14, r1			// SCE OR PCE, since they share
                                                // the CRD logout frame
        // Get base of the logout area.
        GET_IMPURE(r14)				 // addr of per-cpu impure area
        GET_ADDR(r14,(pal_logout_area+mchk_crd_base),r14)

        blbc	r1, sys_crd_write_logout_frame	// If pce/sce not set, build the frame

        // Set the 2nd error flag in the logout area:

        lda     r1, 3(r31)			// Set retry and 2nd error flags
        sll	r1, 30, r1			// Move to bits 31:30 of logout frame flag longword
        stl_p	r1, mchk_crd_flag+4(r14)	// store flag longword
        br 	sys_crd_ack

sys_crd_write_logout_frame:
        // should only be here if neither the pce or sce bits are set

        //
        // Write the mchk code to the logout area
        //
        stq_p	r9, mchk_crd_mchk_code(r14)


        //
        // Write the first 2 quadwords of the logout area:
        //
        lda     r1, 1(r31)		  	// Set retry flag
        sll	r1, 63, r9		  	// Move retry flag to bit 63
        lda	r1, mchk_crd_size(r9)	  	// Combine retry flag and frame size
        stq_p	r1, mchk_crd_flag(r14)	  	// store flag/frame size

        //
        // Write error IPRs already fetched to the logout area
        //
        stq_p	r0, mchk_crd_ei_addr(r14)
        stq_p	r10, mchk_crd_fill_syn(r14)
        stq_p	r8, mchk_crd_ei_stat(r14)
        stq_p	r25, mchk_crd_isr(r14)
        //
        // Log system specific info here
        //
crd_storeTLEP_:
        lda	r1, 0xffc4(r31)			// Get GBUS$MISCR address
        sll	r1, 24, r1
        ldq_p	r1, 0(r1)			// Read GBUS$MISCR
        sll	r1, 16, r1			// shift up to proper field
        mfpr	r10, pt_whami			// get our node id
        extbl	r10, 1, r10			// shift to bit 0
        or	r1, r10, r1			// merge MISCR and WHAMI
        stl_p	r1, mchk_crd_whami(r14)		// write to crd logout area
        srl	r10, 1, r10			// shift off cpu number

        Get_TLSB_Node_Address(r10,r0)		// compute our nodespace address

        OSFcrd_TLEPstore_tlsb(tldev)
        OSFcrd_TLEPstore_tlsb_clr(tlber)
        OSFcrd_TLEPstore_tlsb_clr(tlesr0)
        OSFcrd_TLEPstore_tlsb_clr(tlesr1)
        OSFcrd_TLEPstore_tlsb_clr(tlesr2)
        OSFcrd_TLEPstore_tlsb_clr(tlesr3)

sys_crd_ack:
        mfpr	r0, pt0					// restore r0
        mfpr	r1, pt1					// restore r1

        srl	r12, ei_stat_v_ei_es, r12
        blbc	r12, 5f
        srl	r13, mces_v_dsc, r10			// logging enabled?
        br	r31, 6f
5:	srl	r13, mces_v_dpc, r10			// logging enabled?
6:	blbc	r10, sys_crd_post_interrupt		// logging enabled -- report it

                                                        // logging not enabled
        // Get base of the logout area.
        GET_IMPURE(r13)				 // addr of per-cpu impure area
        GET_ADDR(r13,(pal_logout_area+mchk_crd_base),r13)
        ldl_p	r10, mchk_crd_rsvd(r13)			// bump counter
        addl	r10, 1, r10
        stl_p	r10, mchk_crd_rsvd(r13)
        mb
        br	r31, sys_crd_dismiss_interrupt		// just return

        //
        // The stack is pushed.  Load up a0,a1,a2 and vector via entInt
        //
        //

        ALIGN_BRANCH
sys_crd_post_interrupt:
        lda	r16, osfint_c_mchk(r31)	// flag as mchk/crd in a0
        lda	r17, scb_v_proc_corr_err(r31) // a1 <- interrupt vector

        blbc	r12, 1f
        lda	r17, scb_v_sys_corr_err(r31) // a1 <- interrupt vector

1:	subq    r31, 1, r18            // get a -1
        mfpr	r25, pt_entInt

        srl     r18, 42, r18           // shift off low bits of kseg addr
        mtpr	r25, exc_addr		// load interrupt vector

        sll     r18, 42, r18           // shift back into position
        or    	r14, r18, r18           // EV4 algorithm - pass pointer to mchk frame as kseg address

        hw_rei_spe			// done


        //
        // The stack is pushed.  Need to back out of it all.
        //

sys_crd_dismiss_interrupt:
        br	r31, Call_Pal_Rti


// sys_crd_scrub_mem
//
//	r0 = addr of cache block
//
        ALIGN_BLOCK	// align for branch target
sys_crd_scrub_mem:
        // now find error in memory, and attempt to scrub that cache block
        // This routine just scrubs the failing octaword
        // Only need to "touch" one quadword per octaword to accomplish the scrub
        srl	r0, 39, r8		// get high bit of bad pa
        blbs	r8, 1f  		// don't attempt fixup on IO space addrs
        nop				// needed to align the ldq_pl to octaword boundary
        nop				//             "

        ldq_p 	r8,  0(r0) 		// attempt to read the bad memory
                                        // location
                                        //    (Note bits 63:40,3:0 of ei_addr
                                        //     are set to 1, but as long as
                                        //     we are doing a phys ref, should
                                        //     be ok)
        nop				// Needed to keep the Ibox from swapping the ldq_p into E1

        stq_p 	r8,  0(r0) 		// Store it back if it is still there.
                                        // If store fails, location already
                                        //  scrubbed by someone else

        nop				// needed to align the ldq_p to octaword boundary

        lda	r8, 0x20(r31)		// flip bit 5 to touch next hexaword
        xor	r8, r0, r0
        nop				// needed to align the ldq_p to octaword boundary
        nop				//             "

        ldq_p 	r8,  0(r0) 		// attempt to read the bad memory
                                        // location
                                        //    (Note bits 63:40,3:0 of ei_addr
                                        //     are set to 1, but as long as
                                        //     we are doing a phys ref, should
                                        //     be ok)
        nop				// Needed to keep the Ibox from swapping the ldq_p into E1

        stq_p 	r8,  0(r0) 		// Store it back if it is still there.
                                        // If store fails, location already
                                        //  scrubbed by someone else

        lda	r8, 0x20(r31)		// restore r0 to original address
        xor	r8, r0, r0

        //at this point, ei_stat could be locked due to a new corr error on the ld,
        //so read ei_stat to unlock AFTER this routine.

// XXX bugnion	pvc$jsr	crd_scrub_mem, bsr=1, dest=1
1:	ret	r31, (r13)		// and back we go


//
// sys_int_mchk - MCHK Interrupt code
//
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
//
        ALIGN_BLOCK
sys_int_mchk:
        lda	r14, mchk_c_sys_hrd_error(r31)
        mfpr	r12, exc_addr

        addq	r14, 1, r14			// Flag as interrupt
        nop

        sll	r14, 32, r14			// Move mchk code to position
        mtpr	r12, pt10			// Stash exc_addr

        mfpr	r12, pt_misc			// Get MCES and scratch
        mtpr	r0, pt0				// Stash for scratch

        zap	r12, 0x3c, r12			// Clear scratch
        blbs    r12, sys_double_machine_check   // MCHK halt if double machine check

        or	r12, r14, r12			// Combine mchk code
        lda	r14, scb_v_sysmchk(r31)		// Get SCB vector

        sll	r14, 16, r14			// Move SCBv to position
        or	r12, r14, r14			// Combine SCBv

        bis	r14, BIT(mces_v_mchk), r14	// Set MCES<MCHK> bit
        mtpr	r14, pt_misc			// Save mchk code!scbv!whami!mces

        ldah	r14, 0xfff0(r31)
        mtpr	r1, pt1				// Stash for scratch

        zap	r14, 0xE0, r14			// Get Cbox IPR base
        mtpr	r4, pt4

        mtpr	r5, pt5

        mtpr	r6, pt6
        br	r31, sys_mchk_collect_iprs	// Join common machine check flow


//
// sys_int_perf_cnt - Performance counter interrupt code
//
//	A performance counter interrupt has been detected.  The stack
//	has been pushed. IPL and PS are updated as well.
//
//	on exit to interrupt entry point ENTINT::
//		a0 = osfint$c_perf
//		a1 = scb$v_perfmon (650)
//		a2 = 0 if performance counter 0 fired
//		a2 = 1 if performance counter 1 fired
//		a2 = 2 if performance counter 2 fired
//		     (if more than one counter overflowed, an interrupt will be
//			generated for each counter that overflows)
//
//
//
        ALIGN_BLOCK
sys_int_perf_cnt:			// Performance counter interrupt
        lda	r17, scb_v_perfmon(r31)	// a1 to interrupt vector
        mfpr	r25, pt_entint

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



//
//  sys_reset - System specific RESET code
//	On entry:
//       r1 = pal_base +8
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
//	Entry state on switch:
//       r17 - new PC
//       r18 - new PCBB
//       r19 - new VPTB
//

        ALIGN_BLOCK
        .globl sys_reset
sys_reset:
//	mtpr	r31, ic_flush_ctl	// do not flush the icache - done by hardware before SROM load
        mtpr	r31, itb_ia		// clear the ITB
        mtpr	r31, dtb_ia		// clear the DTB

        lda	r1, -8(r1)		// point to start of code
        mtpr	r1, pal_base		// initialize PAL_BASE

        // Interrupts
        mtpr	r31, astrr		// stop ASTs
        mtpr	r31, aster		// stop ASTs
        mtpr	r31, sirr		// clear software interrupts

        mtpr	r0, pt1			// r0 is whami (unless we entered via swp)

        ldah     r1,(BIT(icsr_v_sde-16)|BIT(icsr_v_fpe-16)|BIT(icsr_v_spe-16+1))(zero)

        bis	r31, 1, r0
        sll	r0, icsr_v_crde, r0	// A 1 in iscr<corr_read_enable>
        or	r0, r1, r1		// Set the bit

        mtpr	r1, icsr		// ICSR - Shadows enabled, Floating point enable,
                                        //	super page enabled, correct read per assembly option

        // Mbox/Dcache init
        lda     r1,BIT(mcsr_v_sp1)(zero)

        mtpr	r1, mcsr		// MCSR - Super page enabled
        lda	r1, BIT(dc_mode_v_dc_ena)(r31)
        ALIGN_BRANCH
//	mtpr	r1, dc_mode		// turn Dcache on
        nop

        mfpr	r31, pt0		// No Mbox instr in 1,2,3,4
        mfpr	r31, pt0
        mfpr	r31, pt0
        mfpr	r31, pt0
        mtpr	r31, dc_flush		// flush Dcache

        // build PS (IPL=7,CM=K,VMM=0,SW=0)
        lda	r11, 0x7(r31)		// Set shadow copy of PS - kern mode, IPL=7
        lda	r1, 0x1F(r31)
        mtpr	r1, ipl			// set internal <ipl>=1F
        mtpr	r31, ev5__ps			// set new ps<cm>=0, Ibox copy
        mtpr	r31, dtb_cm		// set new ps<cm>=0, Mbox copy

        // Create the PALtemp pt_intmask
        //   MAP:
        //	OSF IPL		EV5 internal IPL(hex)	note
        //	0		0
        //	1		1
        //	2		2
        //	3		14			device
        //	4		15			device
        //	5		16			device
        //	6		1E			device,performance counter, powerfail
        //	7		1F
        //

        ldah	r1, 0x1f1E(r31)		// Create upper lw of int_mask
        lda	r1, 0x1615(r1)

        sll	r1, 32, r1
        ldah	r1, 0x1402(r1)		// Create lower lw of int_mask

        lda	r1, 0x0100(r1)
        mtpr	r1, pt_intmask		// Stash in PALtemp

        // Unlock a bunch of chip internal IPRs
        mtpr	r31, exc_sum		// clear out exeception summary and exc_mask
        mfpr	r31, va			// unlock va, mmstat
        lda     r8,(BIT(icperr_stat_v_dpe)|BIT(icperr_stat_v_tpe)|BIT(icperr_stat_v_tmr))(zero)

        mtpr	r8, icperr_stat			// Clear Icache parity error & timeout status
        lda	r8,(BIT(dcperr_stat_v_lock)|BIT(dcperr_stat_v_seo))(r31)

        mtpr	r8, dcperr_stat			// Clear Dcache parity error status

        rc	r0			// clear intr_flag
        mtpr	r31, pt_trap

        mfpr	r0, pt_misc
        srl	r0, pt_misc_v_switch, r1
        blbs	r1, sys_reset_switch	// see if we got here from swppal

        // Rest of the "real" reset flow
        // ASN
        mtpr	r31, dtb_asn
        mtpr	r31, itb_asn

        lda	r1, 0x67(r31)
        sll	r1, hwint_clr_v_pc0c, r1
        mtpr	r1, hwint_clr		// Clear hardware interrupt requests

        lda	r1, BIT(mces_v_dpc)(r31) // 1 in disable processor correctable error
        mfpr	r0, pt1			// get whami
        insbl	r0, 1, r0		// isolate whami in correct pt_misc position
        or	r0, r1, r1		// combine whami and mces
        mtpr	r1, pt_misc		// store whami and mces, swap bit clear

        zapnot	r3, 1, r0		// isolate halt code
        mtpr	r0, pt0			// save entry type

        // Cycle counter
        or	r31, 1, r9		// get a one
        sll	r9, 32, r9		// shift to <32>
        mtpr	r31, cc			// clear Cycle Counter
        mtpr	r9, cc_ctl		// clear and enable the Cycle Counter
        mtpr	r31, pt_scc		// clear System Cycle Counter


        // Misc PALtemps
        mtpr	r31, maf_mode		// no mbox instructions for 3 cycles
        or	r31, 1, r1		// get bogus scbb value
        mtpr	r1, pt_scbb		// load scbb
        mtpr	r31, pt_prbr		// clear out prbr
#if defined(TSUNAMI) || defined(BIG_TSUNAMI)
        // yes, this is ugly, but you figure out a better
        // way to get the address of the kludge_initial_pcbb
        // in r1 with an uncooperative assembler --ali
        br     r1, kludge_getpcb_addr
        br     r31, kludge_initial_pcbb
kludge_getpcb_addr:
        ldq_p   r19, 0(r1)
        sll    r19, 44, r19
        srl    r19, 44, r19
        mulq   r19,4,r19
        addq   r19, r1, r1
        addq   r1,4,r1
#elif defined(TLASER)
        // or      zero,kludge_initial_pcbb,r1
        GET_ADDR(r1, (kludge_initial_pcbb-pal_base), r1)
#endif
        mtpr	r1, pt_pcbb		// load pcbb
        lda	r1, 2(r31)		// get a two
        sll	r1, 32, r1		// gen up upper bits
        mtpr	r1, mvptbr
        mtpr	r1, ivptbr
        mtpr	r31, pt_ptbr
        // Performance counters
        mtpr	r31, pmctr

        // Clear pmctr_ctl in impure area


        ldah	r14, 0xfff0(r31)
        zap	r14, 0xE0, r14		// Get Cbox IPR base
        GET_IMPURE(r13)
        stq_p	r31, 0(r13)		// Clear lock_flag

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

        ldq_p	r22, osfpcb_q_fen(r18)	// get new fen/pme
        ldl_p	r23, osfpcb_l_cc(r18)	// get cycle counter
        ldl_p	r24, osfpcb_l_asn(r18)	// get new asn


        ldq_p	r25, osfpcb_q_Mmptr(r18)// get new mmptr
        sll	r25, page_offset_size_bits, r25 // convert pfn to pa
        mtpr	r25, pt_ptbr		// load the new mmptr
        mtpr	r18, pt_pcbb		// set new pcbb

        bic	r17, 3, r17		// clean use pc
        mtpr	r17, exc_addr		// set new pc
        mtpr	r19, mvptbr
        mtpr	r19, ivptbr

        ldq_p	r30, osfpcb_q_Usp(r18)	// get new usp
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
        ldq_p	r30, osfpcb_q_Ksp(r18)	// get new ksp
        mfpr	r31, pt0		// stall
        hw_rei_stall

//
//sys_machine_check - Machine check PAL
// 	A machine_check trap has occurred.  The Icache has been flushed.
//
//

        ALIGN_BLOCK
EXPORT(sys_machine_check)
        // Need to fill up the refill buffer (32 instructions) and
        // then flush the Icache again.
        // Also, due to possible 2nd Cbox register file write for
        // uncorrectable errors, no register file read or write for 7 cycles.

        //nop
        .long 0x4000054 // call M5 Panic
        mtpr	r0, pt0		// Stash for scratch -- OK if Cbox overwrites
                                //    r0 later
        nop
        nop

        nop
        nop

        nop
        nop

        nop
        nop
                                // 10 instructions// 5 cycles

        nop
        nop

        nop
        nop

                                                // Register file can now be written
        lda	r0, scb_v_procmchk(r31)		// SCB vector
        mfpr	r13, pt_mces			// Get MCES
        sll	r0, 16, r0			// Move SCBv to correct position
        bis	r13, BIT(mces_v_mchk), r14	// Set MCES<MCHK> bit


        zap	r14, 0x3C, r14			// Clear mchk_code word and SCBv word
        mtpr	r14, pt_mces
                                                // 20 instructions

        nop
        or	r14, r0, r14			// Insert new SCB vector
        lda	r0, mchk_c_proc_hrd_error(r31)	// MCHK code
        mfpr	r12, exc_addr

        sll	r0, 32, r0			// Move MCHK code to correct position
        mtpr	r4, pt4
        or	r14, r0, r14			// Insert new MCHK code
        mtpr	r14, pt_misc			// Store updated MCES, MCHK code, and SCBv

        ldah	r14, 0xfff0(r31)
        mtpr	r1, pt1				// Stash for scratch - 30 instructions

        zap	r14, 0xE0, r14			// Get Cbox IPR base
        mtpr	r12, pt10			// Stash exc_addr



        mtpr	r31, ic_flush_ctl			// Second Icache flush, now it is really flushed.
        blbs	r13, sys_double_machine_check		// MCHK halt if double machine check

        mtpr	r6, pt6
        mtpr	r5, pt5

        // Look for the powerfail cases here....
        mfpr	r4, isr
        srl	r4, isr_v_pfl, r4
        blbc	r4, sys_mchk_collect_iprs	// skip if no powerfail interrupt pending
        lda	r4, 0xffc4(r31)			// get GBUS$MISCR address bits
        sll	r4, 24, r4			// shift to proper position
        ldq_p	r4, 0(r4)			// read GBUS$MISCR
        srl	r4, 5, r4			// isolate bit <5>
        blbc	r4, sys_mchk_collect_iprs	// skip if already cleared
                                                // No missed CFAIL mchk
        lda	r5, 0xffc7(r31)			// get GBUS$SERNUM address bits
        sll	r5, 24, r5			// shift to proper position
        lda	r6, 0x40(r31)			// get bit <6> mask
        ldq_p	r4, 0(r5)			// read GBUS$SERNUM
        or	r4, r6, r6			// set bit <6>
        stq_p	r6, 0(r5)			// clear GBUS$SERNUM<6>
        mb
        mb


        //
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
        //	MCES<mchk> is set
        //
        //

EXPORT(sys_mchk_collect_iprs)
        .long 0x4000054 // call M5 Panic
        //mb						// MB before reading Scache IPRs
        mfpr	r1, icperr_stat

        mfpr	r8, dcperr_stat
        mtpr	r31, dc_flush				// Flush the Dcache

        mfpr	r31, pt0				// Pad Mbox instructions from dc_flush
        mfpr	r31, pt0
        nop
        nop

        ldq_p	r9, sc_addr(r14)			// SC_ADDR IPR
        bis	r9, r31, r31				// Touch ld to make sure it completes before
                                                        // read of SC_STAT
        ldq_p	r10, sc_stat(r14)			// SC_STAT, also unlocks SC_ADDR

        ldq_p	r12, ei_addr(r14)			// EI_ADDR IPR
        ldq_p	r13, bc_tag_addr(r14)			// BC_TAG_ADDR IPR
        ldq_p	r0, fill_syn(r14)			// FILL_SYN IPR
        bis	r12, r13, r31				// Touch lds to make sure they complete before reading EI_STAT
        bis	r0, r0, r31				// Touch lds to make sure they complete before reading EI_STAT
        ldq_p	r25, ei_stat(r14)			// EI_STAT, unlock EI_ADDR, BC_TAG_ADDR, FILL_SYN
        ldq_p	r31, ei_stat(r14)			// Read again to insure it is unlocked




        //
        // Look for nonretryable cases
        // In this segment:
        //	r5<0> = 1 means retryable
        //	r4, r6, and r14 are available for scratch
        //
        //


        bis	r31, r31, r5				// Clear local retryable flag
        srl	r25, ei_stat_v_bc_tperr, r25		// Move EI_STAT status bits to low bits

        lda	r4, 1(r31)
        sll	r4, icperr_stat_v_tmr, r4
        and 	r1, r4, r4				// Timeout reset
        bne	r4, sys_cpu_mchk_not_retryable

        and	r8, BIT(dcperr_stat_v_lock), r4		// DCache parity error locked
        bne	r4, sys_cpu_mchk_not_retryable

        lda	r4, 1(r31)
        sll	r4, sc_stat_v_sc_scnd_err, r4
        and	r10, r4, r4				// 2nd Scache error occurred
        bne	r4, sys_cpu_mchk_not_retryable


        bis	r31, 0xa3, r4				// EI_STAT Bcache Tag Parity Error, Bcache Tag Control
                                                        // Parity Error, Interface Parity Error, 2nd Error

        and	r25, r4, r4
        bne	r4, sys_cpu_mchk_not_retryable

//	bis	r31, #<1@<ei_stat$v_unc_ecc_err-ei_stat$v_bc_tperr>>, r4
        bis	r31, BIT((ei_stat_v_unc_ecc_err-ei_stat_v_bc_tperr)), r4
        and	r25, r4, r4				// Isolate the Uncorrectable Error Bit
//	bis	r31, #<1@<ei_stat$v_fil_ird-ei_stat$v_bc_tperr>>, r6
        bis	r31, BIT((ei_stat_v_fil_ird-ei_stat_v_bc_tperr)), r6 // Isolate the Iread bit
        cmovne	r6, 0, r4				// r4 = 0 if IRD or if No Uncorrectable Error
        bne     r4, sys_cpu_mchk_not_retryable

        lda	r4, 7(r31)
        and 	r10, r4, r4				// Isolate the Scache Tag Parity Error bits
        bne	r4, sys_cpu_mchk_not_retryable		// All Scache Tag PEs are not retryable


        lda	r4, 0x7f8(r31)
        and	r10, r4, r4				// Isolate the Scache Data Parity Error bits
        srl	r10, sc_stat_v_cbox_cmd, r6
        and	r6, 0x1f, r6				// Isolate Scache Command field
        subq	r6, 1, r6				// Scache Iread command = 1
        cmoveq	r6, 0, r4				// r4 = 0 if IRD or if No Parity Error
        bne     r4, sys_cpu_mchk_not_retryable

        // Look for the system unretryable cases here....

        mfpr	r4, isr					// mchk_interrupt pin asserted
        srl	r4, isr_v_mck, r4
        blbs	r4, sys_cpu_mchk_not_retryable



        //
        // Look for retryable cases
        // In this segment:
        //	r5<0> = 1 means retryable
        //	r6 - holds the mchk code
        //	r4 and r14 are available for scratch
        //
        //


        // Within the chip, the retryable cases are Istream errors
        lda	r4, 3(r31)
        sll	r4, icperr_stat_v_dpe, r4
        and	r1, r4, r4
        cmovne	r4, 1, r5				// Retryable if just Icache parity error


        lda	r4, 0x7f8(r31)
        and	r10, r4, r4				// Isolate the Scache Data Parity Error bits
        srl	r10, sc_stat_v_cbox_cmd, r14
        and	r14, 0x1f, r14				// Isolate Scache Command field
        subq	r14, 1, r14				// Scache Iread command = 1
        cmovne	r4, 1, r4				// r4 = 1 if Scache data parity error bit set
        cmovne	r14, 0, r4				// r4 = 1 if Scache PE and Iread
        bis	r4, r5, r5				// Accumulate


        bis	r31, BIT((ei_stat_v_unc_ecc_err-ei_stat_v_bc_tperr)), r4
        and	r25, r4, r4				// Isolate the Uncorrectable Error Bit
        and	r25, BIT((ei_stat_v_fil_ird-ei_stat_v_bc_tperr)), r14 // Isolate the Iread bit
        cmovne	r4, 1, r4				// r4 = 1 if uncorr error
        cmoveq	r14, 0, r4				// r4 = 1 if uncorr and Iread
        bis	r4, r5, r5				// Accumulate

        mfpr	r6, pt_misc
        extwl	r6, 4, r6				// Fetch mchk code
        bic	r6, 1, r6				// Clear flag from interrupt flow
        cmovne	r5, mchk_c_retryable_ird, r6		// Set mchk code


        //
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
        //

sys_mchk_write_logout_frame:
        // Get base of the logout area.
        GET_IMPURE(r14)				 // addr of per-cpu impure area
        GET_ADDR(r14,pal_logout_area+mchk_mchk_base,r14)

        // Write the first 2 quadwords of the logout area:

        sll	r5, 63, r5				// Move retry flag to bit 63
        lda	r4, mchk_size(r5)			// Combine retry flag and frame size
        stq_p	r4, mchk_flag(r14)			// store flag/frame size
        lda	r4, mchk_sys_base(r31)			// sys offset
        sll	r4, 32, r4
        lda	r4, mchk_cpu_base(r4)			// cpu offset
        stq_p	r4, mchk_offsets(r14)			// store sys offset/cpu offset into logout frame

        //
        // Write the mchk code to the logout area
        // Write error IPRs already fetched to the logout area
        // Restore some GPRs from PALtemps
        //

        mfpr	r5, pt5
        stq_p	r6, mchk_mchk_code(r14)
        mfpr	r4, pt4
        stq_p	r1, mchk_ic_perr_stat(r14)
        mfpr	r6, pt6
        stq_p	r8, mchk_dc_perr_stat(r14)
        mfpr	r1, pt1
        stq_p	r9, mchk_sc_addr(r14)
        stq_p	r10, mchk_sc_stat(r14)
        stq_p	r12, mchk_ei_addr(r14)
        stq_p	r13, mchk_bc_tag_addr(r14)
        stq_p	r0,  mchk_fill_syn(r14)
        mfpr	r0, pt0
        sll	r25, ei_stat_v_bc_tperr, r25		// Move EI_STAT status bits back to expected position
        // retrieve lower 28 bits again from ei_stat and restore before storing to logout frame
        ldah    r13, 0xfff0(r31)
        zapnot  r13, 0x1f, r13
        ldq_p    r13, ei_stat(r13)
        sll     r13, 64-ei_stat_v_bc_tperr, r13
        srl     r13, 64-ei_stat_v_bc_tperr, r13
        or      r25, r13, r25
        stq_p	r25, mchk_ei_stat(r14)




        //
        // complete the CPU-specific part of the logout frame
        //

        ldah	r13, 0xfff0(r31)
        zap	r13, 0xE0, r13			// Get Cbox IPR base
        ldq_p	r13, ld_lock(r13)		// Get ld_lock IPR
        stq_p	r13, mchk_ld_lock(r14)		// and stash it in the frame

        // Unlock IPRs
        lda	r8, (BIT(dcperr_stat_v_lock)|BIT(dcperr_stat_v_seo))(r31)
        mtpr	r8, dcperr_stat			// Clear Dcache parity error status

        lda	r8, (BIT(icperr_stat_v_dpe)|BIT(icperr_stat_v_tpe)|BIT(icperr_stat_v_tmr))(r31)
        mtpr	r8, icperr_stat			// Clear Icache parity error & timeout status

1:	ldq_p	r8, mchk_ic_perr_stat(r14)	// get ICPERR_STAT value
        GET_ADDR(r0,0x1800,r31)		// get ICPERR_STAT value
        and	r0, r8, r0			// compare
        beq	r0, 2f				// check next case if nothing set
        lda	r0, mchk_c_retryable_ird(r31)	// set new MCHK code
        br	r31, do_670			// setup new vector

2:	ldq_p	r8, mchk_dc_perr_stat(r14)	// get DCPERR_STAT value
        GET_ADDR(r0,0x3f,r31)			// get DCPERR_STAT value
        and	r0, r8, r0			// compare
        beq	r0, 3f				// check next case if nothing set
        lda	r0, mchk_c_dcperr(r31)		// set new MCHK code
        br	r31, do_670			// setup new vector

3:	ldq_p	r8, mchk_sc_stat(r14)		// get SC_STAT value
        GET_ADDR(r0,0x107ff,r31)		// get SC_STAT value
        and	r0, r8, r0			// compare
        beq	r0, 4f				// check next case if nothing set
        lda	r0, mchk_c_scperr(r31)		// set new MCHK code
        br	r31, do_670			// setup new vector

4:	ldq_p	r8, mchk_ei_stat(r14)		// get EI_STAT value
        GET_ADDR(r0,0x30000000,r31)		// get EI_STAT value
        and	r0, r8, r0			// compare
        beq	r0, 5f				// check next case if nothing set
        lda	r0, mchk_c_bcperr(r31)		// set new MCHK code
        br	r31, do_670			// setup new vector

5:	ldl_p	r8, mchk_tlber(r14)		// get TLBER value
        GET_ADDR(r0,0xfe01,r31)	        	// get high TLBER mask value
        sll	r0, 16, r0			// shift into proper position
        GET_ADDR(r1,0x03ff,r31)		        // get low TLBER mask value
        or	r0, r1, r0			// merge mask values
        and	r0, r8, r0			// compare
        beq	r0, 6f				// check next case if nothing set
        GET_ADDR(r0, 0xfff0, r31)		// set new MCHK code
        br	r31, do_660			// setup new vector

6:	ldl_p	r8, mchk_tlepaerr(r14)		// get TLEPAERR value
        GET_ADDR(r0,0xff7f,r31) 		// get TLEPAERR mask value
        and	r0, r8, r0			// compare
        beq	r0, 7f				// check next case if nothing set
        GET_ADDR(r0, 0xfffa, r31)		// set new MCHK code
        br	r31, do_660			// setup new vector

7:	ldl_p	r8, mchk_tlepderr(r14)		// get TLEPDERR value
        GET_ADDR(r0,0x7,r31)			// get TLEPDERR mask value
        and	r0, r8, r0			// compare
        beq	r0, 8f				// check next case if nothing set
        GET_ADDR(r0, 0xfffb, r31)		// set new MCHK code
        br	r31, do_660			// setup new vector

8:	ldl_p	r8, mchk_tlepmerr(r14)		// get TLEPMERR value
        GET_ADDR(r0,0x3f,r31)			// get TLEPMERR mask value
        and	r0, r8, r0			// compare
        beq	r0, 9f				// check next case if nothing set
        GET_ADDR(r0, 0xfffc, r31)		// set new MCHK code
        br	r31, do_660			// setup new vector

9:	ldq_p	r8, mchk_ei_stat(r14)		// get EI_STAT value
        GET_ADDR(r0,0xb,r31)			// get EI_STAT mask value
        sll	r0, 32, r0			// shift to upper lw
        and	r0, r8, r0			// compare
        beq	r0, 1f				// check next case if nothing set
        GET_ADDR(r0,0xfffd,r31) 		// set new MCHK code
        br	r31, do_660			// setup new vector

1:	ldl_p	r8, mchk_tlepaerr(r14)		// get TLEPAERR value
        GET_ADDR(r0,0x80,r31)			// get TLEPAERR mask value
        and	r0, r8, r0			// compare
        beq	r0, cont_logout_frame		// check next case if nothing set
        GET_ADDR(r0, 0xfffe, r31)		// set new MCHK code
        br	r31, do_660			// setup new vector

do_670:	lda	r8, scb_v_procmchk(r31)		// SCB vector
        br	r31, do_6x0_cont
do_660:	lda	r8, scb_v_sysmchk(r31)		// SCB vector
do_6x0_cont:
        sll	r8, 16, r8			// shift to proper position
        mfpr	r1, pt_misc			// fetch current pt_misc
        GET_ADDR(r4,0xffff, r31)		// mask for vector field
        sll	r4, 16, r4			// shift to proper position
        bic	r1, r4, r1			// clear out old vector field
        or	r1, r8, r1			// merge in new vector
        mtpr	r1, pt_misc			// save new vector field
        stl_p	r0, mchk_mchk_code(r14)		// save new mchk code

cont_logout_frame:
        // Restore some GPRs from PALtemps
        mfpr	r0, pt0
        mfpr	r1, pt1
        mfpr	r4, pt4

        mfpr	r12, pt10			// fetch original PC
        blbs	r12, sys_machine_check_while_in_pal	// MCHK halt if machine check in pal

//XXXbugnion        pvc_jsr armc, bsr=1
        bsr     r12, sys_arith_and_mchk     	// go check for and deal with arith trap

        mtpr	r31, exc_sum			// Clear Exception Summary

        mfpr	r25, pt10			// write exc_addr after arith_and_mchk to pickup new pc
        stq_p	r25, mchk_exc_addr(r14)

        //
        // Set up the km trap
        //


sys_post_mchk_trap:
        mfpr	r25, pt_misc		// Check for flag from mchk interrupt
        extwl	r25, 4, r25
        blbs	r25, sys_mchk_stack_done // Stack from already pushed if from interrupt flow

        bis	r14, r31, r12		// stash pointer to logout area
        mfpr	r14, pt10		// get exc_addr

        sll	r11, 63-3, r25		// get mode to msb
        bge	r25, 3f

        mtpr	r31, dtb_cm
        mtpr	r31, ev5__ps

        mtpr	r30, pt_usp		// save user stack
        mfpr	r30, pt_ksp

3:
        lda	sp, 0-osfsf_c_size(sp)	// allocate stack space
        nop

        stq	r18, osfsf_a2(sp) 	// a2
        stq	r11, osfsf_ps(sp)	// save ps

        stq	r14, osfsf_pc(sp)	// save pc
        mfpr	r25, pt_entint		// get the VA of the interrupt routine

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

        mtpr	r25, ipl		// Set internal ipl
        srl    	r18, 42, r18          	// shift off low bits of kseg addr

        sll    	r18, 42, r18          	// shift back into position
        mfpr	r29, pt_kgp		// get the kern r29

        or    	r12, r18, r18          	// EV4 algorithm - pass pointer to mchk frame as kseg address
        hw_rei_spe			// out to interrupt dispatch routine


        //
        // The stack is pushed.  Load up a0,a1,a2 and vector via entInt
        //
        //
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


        ALIGN_BRANCH
sys_cpu_mchk_not_retryable:
        mfpr	r6, pt_misc
        extwl	r6, 4, r6				// Fetch mchk code
        br	r31,  sys_mchk_write_logout_frame	//



//
//sys_double_machine_check - a machine check was started, but MCES<MCHK> was
//	already set.  We will now double machine check halt.
//
//	pt0 - old R0
//
//

EXPORT(sys_double_machine_check)
        lda	r0, hlt_c_dbl_mchk(r31)
        br	r31, sys_enter_console

//
// sys_machine_check_while_in_pal - a machine check was started,
//	exc_addr points to a PAL PC.  We will now machine check halt.
//
//	pt0 - old R0
//
//
sys_machine_check_while_in_pal:
        stq_p	r12, mchk_exc_addr(r14)		// exc_addr has not yet been written
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

// XXX bugnion        pvc$jsr armc, bsr=1, dest=1
        ret     r31, (r12)              // return if no outstanding arithmetic error

handle_arith_and_mchk:
        mtpr    r31, ev5__dtb_cm        // Set Mbox current mode to kernel
                                        //     no virt ref for next 2 cycles
        mtpr	r14, pt0

        mtpr	r1, pt1			// get a scratch reg
        and     r11, osfps_m_mode, r1 // get mode bit

        bis     r11, r31, r25           // save ps
        beq     r1, 1f                 // if zero we are in kern now

        bis     r31, r31, r25           // set the new ps
        mtpr    r30, pt_usp             // save user stack

        mfpr    r30, pt_ksp             // get kern stack
1:
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

// XXX bugnion        pvc$jsr armc, bsr=1, dest=1
        ret     r31, (r12)              // return if no outstanding arithmetic error



// sys_enter_console - Common PALcode for ENTERING console
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
//

        ALIGN_BLOCK
EXPORT(sys_enter_console)
        mtpr	r1, pt4
        mtpr	r3, pt5
        subq	r31, 1, r1
        sll	r1, 42, r1
        ldah	r1, 1(r1)

        /* taken from scrmax, seems like the obvious thing to do */
        mtpr	r1, exc_addr
        mfpr	r1, pt4
        mfpr	r3, pt5
        STALL
        STALL
        hw_rei_stall


//
// sys_exit_console - Common PALcode for ENTERING console
//
// Entry:
//	Entered when console wants to reenter PAL.
//	usually as the result of a CONTINUE.
//
//
// Regs' on entry...
//
//
// Function:
//
//	Restore all readable machine state, and return to user code.
//
//
//
//
        ALIGN_BLOCK
sys_exit_console:

        GET_IMPURE(r1)

        // clear lock and intr_flags prior to leaving console
        rc	r31			// clear intr_flag
        // lock flag cleared by restore_state
        // TB's have been flushed

        ldq_p	r3, (cns_gpr+(8*3))(r1)		// restore r3
        ldq_p	r1, (cns_gpr+8)(r1)		// restore r1
        hw_rei_stall				// back to user


// kludge_initial_pcbb - PCB for Boot use only

        ALIGN_128
.globl kludge_initial_pcbb
kludge_initial_pcbb:			// PCB is 128 bytes long
        nop
        nop
        nop
        nop

        nop
        nop
        nop
        nop

        nop
        nop
        nop
        nop

        nop
        nop
        nop
        nop


// SET_SC_BC_CTL subroutine
//
// Subroutine to set the SC_CTL, BC_CONFIG, and BC_CTL registers and
// flush the Scache
// There must be no outstanding memory references -- istream or
// dstream -- when these registers are written.  EV5 prefetcher is
// difficult to turn off.  So, this routine needs to be exactly 32
// instructions long// the final jmp must be in the last octaword of a
// page (prefetcher doesn't go across page)
//
//
// Register expecations:
//	r0	base address of CBOX iprs
//      r5      value to set sc_ctl to (flush bit is added in)
//      r6      value to set bc_ctl to
//	r7	value to set bc_config to
//	r10	return address
// 	r19     old sc_ctl value
// 	r20	old value of bc_ctl
//	r21	old value of bc_config
//	r23	flush scache flag
// Register usage:
//      r17     sc_ctl with flush bit cleared
//	r22	loop address
//
//
set_sc_bc_ctl:
        ret	r31, (r10)		// return to where we came from
