// modified to use the Hudson style "impure.h" instead of ev5_impure.sdl
// since we don't have a mechanism to expand the data structures.... pb Nov/95		

// build_fixed_image: not sure what means
// real_mm to be replaced during rewrite
// remove_save_state  remove_restore_state can be remooved to save space ??


#include "ev5_defs.h"
#include "ev5_impure.h"
#include "ev5_alpha_defs.h"
#include "ev5_paldef.h"
#include "ev5_osfalpha_defs.h"
#include "fromHudsonMacros.h"
#include "fromHudsonOsf.h"
#include "dc21164FromGasSources.h"

#ifdef SIMOS
#define DEBUGSTORE(c) nop
#else
#define DEBUGSTORE(c) \
	lda	r13, c(zero) ; \
	bsr	r25, debugstore
#endif
        
#define DEBUG_EXC_ADDR()\
	bsr	r25, put_exc_addr; \
	DEBUGSTORE(13)		; \
	DEBUGSTORE(10)

#define egore 0
#define acore 0
#define beh_model 0
#define ev5_p2 1
#define ev5_p1 0
#define ldvpte_bug_fix 1
#define osf_chm_fix  0
	
// Do we want to do this?? pb 	
#define spe_fix 0
// Do we want to do this?? pb 	
#define build_fixed_image 0
	
#define ev5_pass2	
#define enable_p4_fixups 0
#define osf_svmin 1
#define enable_physical_console 0
#define fill_err_hack 0
#define icflush_on_tbix 0
#define max_cpuid 1
#define perfmon_debug 0
#define rawhide_system 0
#define rax_mode 0


// This is the fix for the user-mode super page references causing the machine to crash.
#if (spe_fix == 1) && (build_fixed_image==1)
#define hw_rei_spe	br	r31, hw_rei_update_spe
#else	
#define hw_rei_spe	hw_rei
#endif	
	

// redefine a few of the distribution-code names to match the Hudson gas names.	
// opcodes	
#define ldqp ldq_p
#define stqp stq_p
#define ldlp ldl_p
#define stlp stl_p
	
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

// 	.title	"EV5 OSF PAL" 
// 	.ident	"V1.18"
//
//****************************************************************************
//*									    *
//*  Copyright (c) 1992, 1993, 1994, 1995                      		    *
//*  by DIGITAL Equipment Corporation, Maynard, Mass.			    *
//* 									    *
//*  This software is furnished under a license and may be used and  copied  *
//*  only  in  accordance  with  the  terms  of  such  license and with the  *
//*  inclusion of the above copyright notice.  This software or  any  other  *
//*  copies  thereof may not be provided or otherwise made available to any  *
//*  other person.  No title to and ownership of  the  software  is  hereby  *
//*  transferred.							    *
//* 									    *
//*  The information in this software is subject to change  without  notice  *
//*  and  should  not  be  construed  as  a commitment by DIGITAL Equipment  *
//*  Corporation.							    *
//* 									    *
//*  DIGITAL assumes no responsibility for the use or  reliability  of  its  *
//*  software on equipment which is not supplied by DIGITAL.		    *
//*									    *
//****************************************************************************

// .sbttl	"Edit History"
//+
// Who		Rev	When		What	
// ------------	---	-----------	--------------------------------
// DB		0.0	03-Nov-1992	Start
// DB		0.1	28-Dec-1992	add swpctx
// DB		0.2	05-Jan-1993	Bug: PVC found mtpr dtb_CM -> virt ref bug
// DB		0.3	11-Jan-1993	rearrange trap entry points
// DB		0.4	01-Feb-1993	add tbi
// DB		0.5	04-Feb-1993	real MM, kludge reset flow, kludge swppal
// DB		0.6	09-Feb-1993	Bug: several stack pushers used r16 for pc (should be r14)
// DB		0.7	10-Feb-1993	Bug: pushed wrong PC (+8) on CALL_PAL OPCDEC
//					Bug: typo on register number for store in wrunique
//					Bug: rti to kern uses r16 as scratch 
//					Bug: callsys saving wrong value in pt_usp
// DB		0.8	16-Feb-1993	PVC: fix possible pt write->read bug in wrkgp, wrusp
// DB		0.9	18-Feb-1993	Bug: invalid_dpte_handler shifted pte twice
//					Bug: rti stl_c could corrupt the stack
//					Bug: unaligned returning wrong value in r17 (or should be and)
// DB		0.10	19-Feb-1993	Add draina, rd/wrmces, cflush, cserve, interrupt
// DB		0.11	23-Feb-1993	Turn caches on in reset flow
// DB		0.12	10-Mar-1993	Bug: wrong value for icsr for FEN in kern mode flow
// DB		0.13	15-Mar-1993	Bug: wrong value pushed for PC in invalid_dpte_handler if stack push tbmisses
// DB		0.14	23-Mar-1993	Add impure pointer paltemp, reshuffle some other paltemps to match VMS
// DB		0.15	15-Apr-1993	Combine paltemps for WHAMI and MCES
// DB		0.16    12-May-1993	Update reset
//                                       New restriction: no mfpr exc_addr in cycle 1 of call_pal flows
//					Bug: in wrmces, not clearing DPC, DSC
//					Update swppal
//					Add pal bugchecks, pal_save_state, pal_restore_state
// DB		0.17    24-May-1993	Add dfault_in_pal flow; fixup stack builder to have common state for pc/ps.
//                                       New restriction: No hw_rei_stall in 0,1,2 after mtpr itb_asn
// DB		0.18    26-May-1993	PVC fixes
// JM  		0.19	01-jul-1993	Bug: OSFPAL_CALPAL_OPCDEC, TRAP_OPCDEC -- move mt exc_addr after stores
// JM  		0.20	07-jul-1993	Update cns_ and mchk_ names for impure.mar conversion to .sdl
//					Bug:  exc_addr was being loaded before stores that could dtb_miss in the following
//						routines: TRAP_FEN,FEN_TO_OPCDEC,CALL_PAL_CALLSYS,RTI_TO_KERN
// JM 		0.21	26-jul-1993	Bug: move exc_addr load after ALL stores in the following routines:
//						TRAP_IACCVIO::,TRAP_OPCDEC::,TRAP_ARITH::,TRAP_FEN::
//						dfault_trap_cont:,fen_to_opcdec:,invalid_dpte_handler:
//						osfpal_calpal_opcdec:,CALL_PAL_callsys::,TRAP_UNALIGN::
//					Bugs from PVC: trap_unalign - mt pt0 ->mf pt0 within 2 cycles
// JM 		0.22	28-jul-1993	Add WRIPIR instruction
// JM 		0.23	05-aug-1993	Bump version number for release
// JM 		0.24	11-aug-1993	Bug: call_pal_swpipl - palshadow write -> hw_rei violation
// JM		0.25	09-sep-1993	Disable certain "hidden" pvc checks in call_pals;  
//					New restriction: No hw_rei_stall in 0,1,2,3,4 after mtpr itb_asn - affects HALT(raxmode), 
//						and SWPCTX
// JM		0.26	07-oct-1993	Re-implement pal_version
// JM		0.27	12-oct-1993	One more time:  change pal_version format to conform to SRM
// JM		0.28	14-oct-1993	Change ic_flush routine to pal_ic_flush
// JM		0.29	19-oct-1993	BUG(?): dfault_in_pal: use exc_addr to check for dtbmiss,itbmiss check instead
//						of mm_stat<opcode>.  mm_stat contains original opcode, not hw_ld.
// JM		0.30	28-oct-1993	BUG: PVC violation - mf exc_addr in first cycles of call_pal in rti,retsys
// JM		0.31	15-nov-1993	BUG: WRFEN trashing r0
// JM            0.32	21-nov-1993	BUG: dtb_ldq,itb_ldq (used in dfault_in_pal) not defined when real_mm=0
// JM		0.33	24-nov-1993	save/restore_state - 
//						BUG: use ivptbr to restore mvptbr
// 						BUG: adjust hw_ld/st base/offsets to accomodate 10-bit offset limit
//					     	CHANGE: Load 2 pages into dtb to accomodate compressed logout area/multiprocessors
// JM		0.34	20-dec-1993	BUG: set r11<mode> to kernel for ksnv halt case
//					BUG: generate ksnv halt when tb miss on kernel stack accesses
//					     save exc_addr in r14 for invalid_dpte stack builder
// JM		0.35	30-dec-1993	BUG: PVC violation in trap_arith - mt exc_sum in shadow of store with mf exc_mask in 
//					     the same shadow
// JM		0.36	 6-jan-1994	BUG: fen_to_opcdec - savePC should be PC+4, need to save old PS, update new PS
//					      New palcode restiction: mt icsr<fpe,hwe> --> 3 bubbles to hw_rei --affects wrfen
// JM		0.37	25-jan-1994	BUG: PVC violations in restore_state - mt dc_mode/maf_mode ->mbox instructions
//					Hide impure area manipulations in macros
//					BUG: PVC violation in save and restore state-- move mt icsr out of shadow of ld/st
//					Add some pvc_violate statements
// JM		0.38	 1-feb-1994	Changes to save_state:  save pt1; don't save r31,f31; update comments to reflect reality; 
//					Changes to restore_state: restore pt1, icsr; don't restore r31,f31; update comments
//						Add code to ensure fen bit set in icsr before ldt
//					conditionally compile rax_more_reset out.
//					move ldqp,stqp macro definitions to ev5_pal_macros.mar and add .mcall's for them here
//					move rax reset stuff to ev5_osf_system_pal.m64
// JM		0.39	 7-feb-1994	Move impure pointer to pal scratch space.  Use former pt_impure for bc_ctl shadow
//						and performance monitoring bits
//					Change to save_state routine to save more iprs.
// JM		0.40	19-feb-1994	Change algorithm in save/restore_state routines; add f31,r31 back in
// JM		0.41	21-feb-1994     Add flags to compile out save/restore state (not needed in some systems)
//						remove_save_state,remove_restore_state;fix new pvc violation in save_state
// JM		0.42	22-feb-1994     BUG: save_state overwriting r3
// JM		0.43	24-feb-1994	BUG: save_state saving wrong icsr
// JM		0.44	28-feb-1994	Remove ic_flush from wr_tbix instructions
// JM		0.45	15-mar-1994	BUG: call_pal_tbi trashes a0 prior to range check (instruction order problem)
//					New pal restriction in pal_restore_state: icsr<fpe>->floating instr = 3 bubbles
//					Add exc_sum and exc_mask to pal_save_state (not restore)
// JM		0.46	22-apr-1994	Move impure pointer back into paltemp;  Move bc_ctl shadow and pmctr_ctl into impure 
//						area.  
//					Add performance counter support to swpctx and wrperfmon
// JM            0.47    9-may-1994	Bump version # (for ev5_osf_system_pal.m64 sys_perfmon fix)
// JM		0.48	13-jun-1994	BUG: trap_interrupt --> put new ev5 ipl at 30 for all osfipl6 interrupts
// JM		0.49	8-jul-1994	BUG: In the unlikely (impossible?) event that the branch to pal_pal_bug_check is
//						taken in the interrupt flow, stack is pushed twice.
//					SWPPAL - update to support ECO 59 to allow 0 as a valid address
//					Add itb flush to save/restore state routines
//					Change hw_rei to hw_rei_stall in ic_flush routine.  Shouldn't be necessary, but
//						conforms to itbia restriction.
//					Added enable_physical_console flag (for enter/exit console routines only)
// JM		0.50	29-jul-1994	Add code to dfault & invalid_dpte_handler to ignore exceptions on a 
//						load to r31/f31.  changed dfault_fetch_err to dfault_fetch_ldr31_err and
//						nmiss_fetch_err to nmiss_fetch_ldr31_err.
// JM		1.00	 1-aug-1994	Add pass2 support (swpctx)
// JM		1.01	 2-aug-1994	swppal now passes bc_ctl/bc_config in r1/r2
// JM		1.02	15-sep-1994	BUG: swpctx missing shift of pme bit to correct position in icsr (pass2)
//					Moved perfmon code here from system file.
//					BUG: pal_perfmon - enable function not saving correct enables when pme not set (pass1)
// JM		1.03	3-oct-1994	Added (pass2 only) code to wrperfmon enable function to look at pme bit.
// JM		1.04	14-oct-1994	BUG: trap_interrupt - ISR read (and saved) before INTID -- INTID can change
//						after ISR read, but we won't catch the ISR update.  reverse order
// JM		1.05	17-nov-1994	Add code to dismiss UNALIGN trap if LD r31/F31
// JM		1.06	28-nov-1994	BUG: missing mm_stat shift for store case in trap_unalign (new bug due to "dismiss" code)
// JM		1.07	 1-dec-1994	EV5 PASS1,2,3 BUG WORKAROUND:  Add flag LDVPTE_BUG_FIX.  In DTBMISS_DOUBLE, branch to 
//					DTBMISS_SINGLE if not in palmode.
// JM            1.08     9-jan-1995     Bump version number for change to EV5_OSF_SYSTEM_PAL.M64 - ei_stat fix in mchk logout frame
// JM 		1.09	 2-feb-1995	Add flag "spe_fix" and accompanying code to workaround pre-pass4 bug:  Disable Ibox 
//					superpage mode in User mode and re-enable in kernel mode.
//					EV5_OSF_SYSTEM_PAL.M64 and EV5_PALDEF.MAR (added pt_misc_v_cm) also changed to support this.
// JM		1.10    24-feb-1995	Set ldvpte_bug_fix regardless of ev5 pass.   set default to ev5_p2
// ES		1.11	10-mar-1995	Add flag "osf_chm_fix" to enable dcache in user mode only to avoid
//					cpu bug.
// JM		1.12	17-mar-1995	BUG FIX: Fix F0 corruption problem in pal_restore_state
// ES		1.13	17-mar-1995	Refine osf_chm_fix
// ES		1.14	20-mar-1995	Don't need as many stalls before hw_rei_stall in chm_fix
// ES		1.15	21-mar-1995	Add a stall to avoid a pvc violation in pal_restore_state
//					Force pvc checking of exit_console
// ES		1.16	26-apr-1995	In the wrperfmon disable function, correct meaning of R17<2:0> to ctl2,ctl2,ctl0
// ES		1.17	01-may-1995	In hw_rei_update_spe code, in the osf_chm fix, use bic and bis (self-correcting)
//					instead of xor to maintain previous mode in pt_misc
// ES		1.18	14-jul-1995	In wrperfmon enable on pass2, update pmctr even if current process does
//					not have pme set. The bits in icsr maintain the master enable state.
//					In sys_reset, add icsr<17>=1 for ev56 byte/word eco enable
//
#define vmaj 1
#define vmin 18
#define vms_pal 1 
#define osf_pal 2
#define pal_type osf_pal
#define osfpal_version_l ((pal_type<<16) | (vmaj<<8) | (vmin<<0))
//-

// .sbttl	"PALtemp register usage"

//+
//  The EV5 Ibox holds 24 PALtemp registers.  This maps the OSF PAL usage
//  for these PALtemps:
//
//	pt0   local scratch
//	pt1   local scratch
//	pt2   entUna					pt_entUna
//	pt3   CPU specific impure area pointer		pt_impure
//	pt4   memory management temp
//	pt5   memory management temp
//	pt6   memory management temp   
//	pt7   entIF					pt_entIF
//	pt8   intmask					pt_intmask
//	pt9   entSys					pt_entSys
//	pt10  
//	pt11  entInt					pt_entInt
//	pt12  entArith					pt_entArith
//	pt13  reserved for system specific PAL
//	pt14  reserved for system specific PAL
//	pt15  reserved for system specific PAL
//	pt16  MISC: scratch ! WHAMI<7:0> ! 0 0 0 MCES<4:0> pt_misc, pt_whami, pt_mces
//	pt17  sysval					pt_sysval
//	pt18  usp					pt_usp
//	pt19  ksp					pt_ksp
//	pt20  PTBR					pt_ptbr
//	pt21  entMM					pt_entMM
//	pt22  kgp					pt_kgp
//	pt23  PCBB					pt_pcbb
//
//-

// .sbttl	"PALshadow register usage"
//
//+
//
// EV5 shadows R8-R14 and R25 when in PALmode and ICSR<shadow_enable> = 1.
// This maps the OSF PAL usage of R8 - R14 and R25:
//
// 	r8    ITBmiss/DTBmiss scratch
// 	r9    ITBmiss/DTBmiss scratch
// 	r10   ITBmiss/DTBmiss scratch
//	r11   PS 
//	r12   local scratch
//	r13   local scratch
//	r14   local scratch
//	r25   local scratch
//
//
//-

// .sbttl	"ALPHA symbol definitions"
// 	_OSF_PSDEF	GLOBAL
// 	_OSF_PTEDEF	GLOBAL
// 	_OSF_VADEF	GLOBAL
// 	_OSF_PCBDEF	GLOBAL
//         _OSF_SFDEF      GLOBAL
//         _OSF_MMCSR_DEF  GLOBAL
// 	_SCBDEF		GLOBAL
// 	_FRMDEF		GLOBAL
// 	_EXSDEF		GLOBAL
// 	_OSF_A0_DEF	GLOBAL
// 	_MCESDEF	GLOBAL

// .sbttl	"EV5 symbol definitions"

// 	_EV5DEF
// 	_PALTEMP
// 	_MM_STAT_DEF
//         _EV5_MM
//         _EV5_IPLDEF

//         _HALT_CODES     GLOBAL
//         _MCHK_CODES     GLOBAL

//         _PAL_IMPURE
//         _PAL_LOGOUT


                             

// .sbttl	"PALcode configuration options"

// There are a number of options that may be assembled into this version of
// PALcode. They should be adjusted in a prefix assembly file (i.e. do not edit
// the following). The options that can be adjusted cause the resultant PALcode
// to reflect the desired target system.


#define osfpal 1				// This is the PALcode for OSF.

#ifndef rawhide_system 

#define rawhide_system 0
#endif


#ifndef real_mm 
// Page table translation vs 1-1 mapping
#define real_mm 1
#endif


#ifndef rax_mode 

#define rax_mode 0
#endif

#ifndef egore 
// End of reset flow starts a program at 200000(hex).
#define egore 1
#endif

#ifndef acore 
// End of reset flow starts a program at 40000(hex).
#define acore 0
#endif


// 	assume acore+egore+rax_mode lt 2	// Assertion checker

#ifndef beh_model 
// EV5 behavioral model specific code
#define beh_model 1
#endif

#ifndef init_cbox 
// Reset flow init of Bcache and Scache
#define init_cbox 1
#endif

#ifndef disable_crd 
// Decides whether the reset flow will disable
#define disable_crd 0
#endif

						// correctable read interrupts via ICSR
#ifndef perfmon_debug 
#define perfmon_debug 0
#endif

#ifndef icflush_on_tbix 
#define icflush_on_tbix 0
#endif

#ifndef remove_restore_state 
#define remove_restore_state 0
#endif

#ifndef remove_save_state 
#define remove_save_state 0
#endif

#ifndef enable_physical_console 
#define enable_physical_console 0
#endif

#ifndef ev5_p1 
#define ev5_p1 0
#endif

#ifndef ev5_p2 
#define ev5_p2 1
#endif

// 	assume ev5_p1+ev5_p2 eq 1

#ifndef ldvpte_bug_fix
#define ldvpte_bug_fix 1			// If set, fix ldvpte bug in dtbmiss_double flow.
#endif

#ifndef spe_fix 
// If set, disable super-page mode in user mode and re-enable
#define spe_fix 0
#endif
						// in kernel.  Workaround for cpu bug.
#ifndef build_fixed_image 
#define build_fixed_image 0
#endif


#ifndef fill_err_hack 
// If set, disable fill_error mode in user mode and re-enable
#define fill_err_hack 0
#endif

						    // in kernel.  Workaround for cpu bug.

//	.macro hw_rei_spe
//	.iif eq spe_fix, hw_rei
//#if spe_fix != 0
//
//
//#define hw_rei_chm_count hw_rei_chm_count + 1
//	p4_fixup_label	\hw_rei_chm_count
//	.iif eq	build_fixed_image,	br	r31, hw_rei_update_spe
//	.iif ne build_fixed_image,	hw_rei
//#endif
//
//	.endm

// Add flag "osf_chm_fix" to enable dcache in user mode only
// to avoid cpu bug.

#ifndef osf_chm_fix 
// If set, enable D-Cache in 
#define osf_chm_fix 0
#endif

#if osf_chm_fix != 0
// user mode only.
#define hw_rei_chm_count 0
#endif

#if osf_chm_fix != 0

#define hw_rei_stall_chm_count 0
#endif

#ifndef enable_p4_fixups 

#define enable_p4_fixups 0
#endif

					// If set, do EV5 Pass 4 fixups
#if spe_fix == 0

#define osf_chm_fix 0
#endif

#if spe_fix == 0

#define enable_p4_fixups 0
#endif

					// Only allow fixups if fix enabled

	//Turn off fill_errors and MEM_NEM in user mode
//	.macro fill_error_hack ?L10_, ?L20_, ?L30_, ?L40_
//	//save r22,r23,r24
//	stqp r22, 0x150(r31)	//add
//	stqp r23, 0x158(r31)	//contents
//	stqp r24, 0x160(r31)	//bit mask
//
//        lda     r22, 0x82(r31)
//        ldah    r22, 0x8740(r22)
//        sll     r22, 8, r22
//        ldlp    r23, 0x80(r22)          // r23 <- contents of CIA_MASK
//        bis     r23,r31,r23
//
//	lda	r24, 0x8(r31)		// r24 <- MEM_NEM bit
//	beq	r10, L10_		// IF user mode (r10<0> == 0) pal mode
//	bic	r23, r24, r23		// set fillerr_en bit
//	br	r31, L20_		// ELSE
//L10_:	bis	r23, r24, r23		// clear fillerr_en bit
//L20_:					// ENDIF
//
//	stlp	r23, 0x80(r22)		// write back the CIA_MASK register
//	mb
//	ldlp    r23, 0x80(r22)
//	bis     r23,r31,r23
//	mb
//
//	lda	r22, 1(r31)		// r22 <- 87.4000.0100 ptr to CIA_CTRL
//	ldah	r22, 0x8740(r22)
//	sll	r22, 8, r22
//	ldlp	r23, 0(r22)		// r23 <- contents of CIA_CTRL
//	bis     r23,r31,r23
//
//
//	lda	r24, 0x400(r31)		// r9 <- fillerr_en bit
//	beq	r10, L30_		// IF user mode (r10<0> == 0) pal mode
//	bic	r23, r24, r23		// set fillerr_en bit
//	br	r31, L40_		// ELSE
//L30_:	bis	r23, r24, r23		// clear fillerr_en bit
//L40_:					// ENDIF
//
//	stlp	r23, 0(r22)		// write back the CIA_CTRL register
//	mb
//	ldlp    r23, 0(r22)
//	bis     r23,r31,r23
//	mb
//
//	//restore r22,r23,r24
//	ldqp r22, 0x150(r31)
//	ldqp r23, 0x158(r31)
//	ldqp r24, 0x160(r31)
//
//	.endm

// multiprocessor support can be enabled for a max of n processors by
// setting the following to the number of processors on the system.
// Note that this is really the max cpuid.

#ifndef max_cpuid 
#define max_cpuid 8
#endif

#ifndef osf_svmin			// platform specific palcode version number
#define osf_svmin 0
#endif


#define osfpal_version_h ((max_cpuid<<16) | (osf_svmin<<0))

// .mcall ldqp		// override macro64 definition with macro from library
// .mcall stqp		// override macro64 definition with macro from library


// 	.psect	_pal,mix		
// huh pb pal_base:
// huh pb #define current_block_base . - pal_base	

// .sbttl	"RESET	-  Reset Trap Entry Point"
//+
// RESET - offset 0000
// Entry:
//	Vectored into via hardware trap on reset, or branched to
//	on swppal.
//
//	r0 = whami
//	r1 = pal_base
//	r2 = base of scratch area
//	r3 = halt code
//
//
// Function:
//
//-

	.text	0
	. = 0x0000
	.globl Pal_Base
Pal_Base:
        HDW_VECTOR(PAL_RESET_ENTRY)
Trap_Reset:
	nop
#ifdef SIMOS
        /*
         * store into r1
         */
        br r1,sys_reset
#else
        /* following is a srcmax change */
        
	DEBUGSTORE(0x41)
	/* The original code jumped using r1 as a linkage register to pass the base
	   of PALcode to the platform specific code.  We use r1 to pass a parameter
	   from the SROM, so we hardcode the address of Pal_Base in platform.s
	 */
	br	r31, sys_reset
#endif
        
	// Specify PAL version info as a constant 
	// at a known location (reset + 8).

	.long osfpal_version_l		// <pal_type@16> ! <vmaj@8> ! <vmin@0>
	.long osfpal_version_h		// <max_cpuid@16> ! <osf_svmin@0>
	.long 0
	.long 0
pal_impure_start:
	.quad 0	
pal_debug_ptr:
	.quad 0				// reserved for debug pointer ; 20
#if beh_model == 0


#if enable_p4_fixups != 0


	.quad 0
	.long p4_fixup_hw_rei_fixup_table
#endif

#else

	.quad 0				// 
	.quad 0	//0x0030
	.quad 0
	.quad 0 //0x0040
	.quad 0
	.quad 0 //0x0050
	.quad 0								
	.quad 0 //0x0060
	.quad 0
pal_enter_cns_address:
	.quad 0				//0x0070 -- address to jump to from enter_console
	.long <<sys_exit_console-pal_base>+1>      //0x0078 -- offset to sys_exit_console (set palmode bit)
#endif




// .sbttl	"IACCVIO- Istream Access Violation Trap Entry Point"

//+
// IACCVIO - offset 0080
// Entry:
//	Vectored into via hardware trap on Istream access violation or sign check error on PC.
//
// Function:
//	Build stack frame
//	a0 <- Faulting VA
//	a1 <- MMCSR  (1 for ACV)
//	a2 <- -1 (for ifetch fault)
//	vector via entMM
//-

        HDW_VECTOR(PAL_IACCVIO_ENTRY)
Trap_Iaccvio:
	DEBUGSTORE(0x42)
	sll	r11, 63-osfps_v_mode, r25 // Shift mode up to MS bit
	mtpr	r31, ev5__ps		// Set Ibox current mode to kernel

	bis	r11, r31, r12		// Save PS 
	bge	r25, TRAP_IACCVIO_10_		// no stack swap needed if cm=kern


	mtpr	r31, ev5__dtb_cm	// Set Mbox current mode to kernel - 
					//     no virt ref for next 2 cycles
	mtpr	r30, pt_usp		// save user stack

	bis	r31, r31, r12		// Set new PS	
	mfpr	r30, pt_ksp

TRAP_IACCVIO_10_:	
	lda	sp, 0-osfsf_c_size(sp)// allocate stack space 	
	mfpr	r14, exc_addr		// get pc

	stq	r16, osfsf_a0(sp)	// save regs
	bic	r14, 3, r16		// pass pc/va as a0

	stq	r17, osfsf_a1(sp)	// a1
	or	r31, mmcsr_c_acv, r17	// pass mm_csr as a1

	stq	r18, osfsf_a2(sp) 	// a2
	mfpr	r13, pt_entmm		// get entry point

	stq	r11, osfsf_ps(sp)	// save old ps
	bis	r12, r31, r11		// update ps

	stq	r16, osfsf_pc(sp)	// save pc
	stq	r29, osfsf_gp(sp) 	// save gp

	mtpr	r13, exc_addr		// load exc_addr with entMM
					// 1 cycle to hw_rei
	mfpr	r29, pt_kgp		// get the kgp

	subq	r31, 1, r18		// pass flag of istream, as a2
	hw_rei_spe


// .sbttl	"INTERRUPT- Interrupt Trap Entry Point"

//+
// INTERRUPT - offset 0100
// Entry:
//	Vectored into via trap on hardware interrupt
//
// Function:
//	check for halt interrupt
//	check for passive release (current ipl geq requestor)
//	if necessary, switch to kernel mode
//	push stack frame, update ps (including current mode and ipl copies), sp, and gp
//	pass the interrupt info to the system module
//
//-


        HDW_VECTOR(PAL_INTERRUPT_ENTRY)
Trap_Interrupt:
        mfpr    r13, ev5__intid         // Fetch level of interruptor
        mfpr    r25, ev5__isr           // Fetch interrupt summary register

        srl     r25, isr_v_hlt, r9     // Get HLT bit
	mfpr	r14, ev5__ipl

	mtpr	r31, ev5__dtb_cm	// Set Mbox current mode to kern
        blbs    r9, sys_halt_interrupt	// halt_interrupt if HLT bit set

        cmple   r13, r14, r8            // R8 = 1 if intid .less than or eql. ipl
        bne     r8, sys_passive_release // Passive release is current rupt is lt or eq ipl

	and	r11, osfps_m_mode, r10 // get mode bit
	beq	r10, TRAP_INTERRUPT_10_		// Skip stack swap in kernel

	mtpr	r30, pt_usp		// save user stack
	mfpr	r30, pt_ksp		// get kern stack

TRAP_INTERRUPT_10_:
	lda	sp, (0-osfsf_c_size)(sp)// allocate stack space 	
	mfpr	r14, exc_addr		// get pc

	stq	r11, osfsf_ps(sp) 	// save ps
	stq	r14, osfsf_pc(sp) 	// save pc

        stq     r29, osfsf_gp(sp)       // push gp                                                 
	stq	r16, osfsf_a0(sp)	// a0	

//	pvc_violate 354			// ps is cleared anyway,  if store to stack faults.
        mtpr    r31, ev5__ps            // Set Ibox current mode to kernel
	stq	r17, osfsf_a1(sp)	// a1

	stq	r18, osfsf_a2(sp) 	// a2
	subq	r13, 0x11, r12		// Start to translate from EV5IPL->OSFIPL
	
	srl	r12, 1, r8		// 1d, 1e: ipl 6.  1f: ipl 7.
	subq	r13, 0x1d, r9		// Check for 1d, 1e, 1f
	
	cmovge	r9, r8, r12		// if .ge. 1d, then take shifted value
	bis	r12, r31, r11		// set new ps

	mfpr	r12, pt_intmask
	and	r11, osfps_m_ipl, r14	// Isolate just new ipl (not really needed, since all non-ipl bits zeroed already)

#ifdef SIMOS
        /*
         * Lance had space problems. We don't.
         */
	extbl	r12, r14, r14		// Translate new OSFIPL->EV5IPL
	mfpr	r29, pt_kgp		// update gp
        mtpr	r14, ev5__ipl		// load the new IPL into Ibox
#else
// Moved the following three lines to sys_interrupt to make room for debug
//	extbl	r12, r14, r14		// Translate new OSFIPL->EV5IPL
//	mfpr	r29, pt_kgp		// update gp

//	mtpr	r14, ev5__ipl		// load the new IPL into Ibox
#endif         
	br	r31, sys_interrupt	// Go handle interrupt



// .sbttl	"ITBMISS- Istream TBmiss Trap Entry Point"

//+
// ITBMISS - offset 0180
// Entry:
//	Vectored into via hardware trap on Istream translation buffer miss.
//
// Function:
//       Do a virtual fetch of the PTE, and fill the ITB if the PTE is valid.
//       Can trap into DTBMISS_DOUBLE.
//       This routine can use the PALshadow registers r8, r9, and r10
//	
//-

        HDW_VECTOR(PAL_ITB_MISS_ENTRY)
Trap_Itbmiss:
#if real_mm == 0
			

					// Simple 1-1 va->pa mapping

	nop				// Pad to align to E1
	mfpr 	r8, exc_addr
	
	srl	r8, page_offset_size_bits, r9
	sll	r9, 32, r9
	
	lda	r9, 0x3301(r9)		// Make PTE, V set, all KRE, URE, KWE, UWE
	mtpr	r9, itb_pte		// E1
	
	hw_rei_stall			// Nital says I don't have to obey shadow wait rule here.
#else

					// Real MM mapping
	nop
        mfpr	r8, ev5__ifault_va_form // Get virtual address of PTE. 
	
	nop
        mfpr    r10, exc_addr           // Get PC of faulting instruction in case of DTBmiss. 
	
pal_itb_ldq:
        ld_vpte r8, 0(r8)             	// Get PTE, traps to DTBMISS_DOUBLE in case of TBmiss
	mtpr	r10, exc_addr		// Restore exc_address if there was a trap.
	
	mfpr	r31, ev5__va		// Unlock VA in case there was a double miss
	nop

	and	r8, osfpte_m_foe, r25 	// Look for FOE set.
	blbc	r8, invalid_ipte_handler // PTE not valid.
	
	nop
	bne	r25, foe_ipte_handler	// FOE is set
	
	nop
	mtpr	r8, ev5__itb_pte	// Ibox remembers the VA, load the PTE into the ITB.

	hw_rei_stall			// 

#endif




// .sbttl	"DTBMISS_SINGLE	- Dstream Single TBmiss Trap Entry Point"

//+
// DTBMISS_SINGLE - offset 0200
// Entry:
//	Vectored into via hardware trap on Dstream single translation buffer miss.
//
// Function:
//	Do a virtual fetch of the PTE, and fill the DTB if the PTE is valid.
//	Can trap into DTBMISS_DOUBLE.
//	This routine can use the PALshadow registers r8, r9, and r10
//-

        HDW_VECTOR(PAL_DTB_MISS_ENTRY)
Trap_Dtbmiss_Single:
#if real_mm == 0
					// Simple 1-1 va->pa mapping
	mfpr 	r8, va			// E0
	srl	r8, page_offset_size_bits, r9

	sll	r9, 32, r9
	lda	r9, 0x3301(r9)		// Make PTE, V set, all KRE, URE, KWE, UWE

	mtpr	r9, dtb_pte		// E0
	nop				// Pad to align to E0
	

	
	mtpr	r8, dtb_tag		// E0
	nop
	
	nop				// Pad tag write
	nop
	
	nop				// Pad tag write
	nop
	
	hw_rei
#else
        mfpr	r8, ev5__va_form      	// Get virtual address of PTE - 1 cycle delay.  E0.
        mfpr    r10, exc_addr           // Get PC of faulting instruction in case of error.  E1.
	
//	DEBUGSTORE(0x45)
//	DEBUG_EXC_ADDR()
					// Real MM mapping
        mfpr    r9, ev5__mm_stat	// Get read/write bit.  E0. 
	mtpr	r10, pt6		// Stash exc_addr away

pal_dtb_ldq:
        ld_vpte r8, 0(r8)             	// Get PTE, traps to DTBMISS_DOUBLE in case of TBmiss
	nop				// Pad MF VA
	
	mfpr	r10, ev5__va            // Get original faulting VA for TB load.  E0.	
	nop

        mtpr    r8, ev5__dtb_pte       	// Write DTB PTE part.   E0.
        blbc    r8, invalid_dpte_handler    // Handle invalid PTE
	
        mtpr    r10, ev5__dtb_tag      	// Write DTB TAG part, completes DTB load.  No virt ref for 3 cycles.
	mfpr	r10, pt6

					// Following 2 instructions take 2 cycles
        mtpr    r10, exc_addr           // Return linkage in case we trapped.  E1.
	mfpr	r31,  pt0		// Pad the write to dtb_tag

        hw_rei                          // Done, return
#endif




// .sbttl	"DTBMISS_DOUBLE	- Dstream Double TBmiss Trap Entry Point"

//+
// DTBMISS_DOUBLE - offset 0280
// Entry:
//	Vectored into via hardware trap on Double TBmiss from single miss flows.
//
//	r8   - faulting VA
//	r9   - original MMstat
//	r10 - original exc_addr (both itb,dtb miss)
//	pt6 - original exc_addr (dtb miss flow only)
//	VA IPR - locked with original faulting VA
//
// Function:
// 	Get PTE, if valid load TB and return.
//	If not valid then take TNV/ACV exception.
//
//	pt4 and pt5 are reserved for this flow.
//
//
//-

        HDW_VECTOR(PAL_DOUBLE_MISS_ENTRY)
Trap_Dtbmiss_double:
#if ldvpte_bug_fix != 0
	mtpr 	r8, pt4			// save r8 to do exc_addr check
	mfpr	r8, exc_addr
	blbc	r8, Trap_Dtbmiss_Single	//if not in palmode, should be in the single routine, dummy!
	mfpr	r8, pt4			// restore r8
#endif
	nop
	mtpr	r22, pt5		// Get some scratch space. E1.
					// Due to virtual scheme, we can skip the first lookup and go
					// right to fetch of level 2 PTE
	sll     r8, (64-((2*page_seg_size_bits)+page_offset_size_bits)), r22  // Clean off upper bits of VA
	mtpr	r21, pt4		// Get some scratch space. E1.

	srl    	r22, 61-page_seg_size_bits, r22 // Get Va<seg1>*8
	mfpr	r21, pt_ptbr		// Get physical address of the page table.

	nop
	addq    r21, r22, r21           // Index into page table for level 2 PTE.
	
	sll    	r8, (64-((1*page_seg_size_bits)+page_offset_size_bits)), r22  // Clean off upper bits of VA
	ldqp   	r21, 0(r21)            	// Get level 2 PTE (addr<2:0> ignored)
	
	srl    	r22, 61-page_seg_size_bits, r22	// Get Va<seg1>*8
	blbc 	r21, double_pte_inv		// Check for Invalid PTE. 

        srl    	r21, 32, r21			// extract PFN from PTE
	sll     r21, page_offset_size_bits, r21	// get PFN * 2^13 for add to <seg3>*8

	addq    r21, r22, r21           // Index into page table for level 3 PTE.
	nop

	ldqp   	r21, 0(r21)            	// Get level 3 PTE (addr<2:0> ignored)
	blbc	r21, double_pte_inv	// Check for invalid PTE.
	
	mtpr	r21, ev5__dtb_pte	// Write the PTE.  E0.
	mfpr	r22, pt5		// Restore scratch register
	
	mtpr	r8, ev5__dtb_tag	// Write the TAG. E0.  No virtual references in subsequent 3 cycles.
	mfpr	r21, pt4		// Restore scratch register

	nop				// Pad write to tag.
	nop
	
	nop				// Pad write to tag.
	nop

	hw_rei



// .sbttl	"UNALIGN -- Dstream unalign trap"
//+
// UNALIGN - offset 0300
// Entry:
//	Vectored into via hardware trap on unaligned Dstream reference.
//
// Function:
//	Build stack frame
//	a0 <- Faulting VA
//	a1 <- Opcode
//	a2 <- src/dst register number
//	vector via entUna
//-

        HDW_VECTOR(PAL_UNALIGN_ENTRY)
Trap_Unalign:
/*	DEBUGSTORE(0x47)*/
	sll	r11, 63-osfps_v_mode, r25 // Shift mode up to MS bit
	mtpr	r31, ev5__ps		// Set Ibox current mode to kernel

	mfpr	r8, ev5__mm_stat	// Get mmstat --ok to use r8, no tbmiss
	mfpr	r14, exc_addr		// get pc

	srl	r8, mm_stat_v_ra, r13	// Shift Ra field to ls bits
	blbs	r14, pal_pal_bug_check  // Bugcheck if unaligned in PAL

	blbs	r8, UNALIGN_NO_DISMISS // lsb only set on store or fetch_m
					// not set, must be a load
	and	r13, 0x1F, r8		// isolate ra

	cmpeq   r8, 0x1F, r8		// check for r31/F31
	bne     r8, dfault_fetch_ldr31_err // if its a load to r31 or f31 -- dismiss the fault

UNALIGN_NO_DISMISS:
	bis	r11, r31, r12		// Save PS
	bge	r25, UNALIGN_NO_DISMISS_10_		// no stack swap needed if cm=kern


	mtpr	r31, ev5__dtb_cm	// Set Mbox current mode to kernel - 
					//     no virt ref for next 2 cycles
	mtpr	r30, pt_usp		// save user stack

	bis	r31, r31, r12		// Set new PS	
	mfpr	r30, pt_ksp

UNALIGN_NO_DISMISS_10_:	
	mfpr	r25, ev5__va		// Unlock VA
	lda	sp, 0-osfsf_c_size(sp)// allocate stack space 	

	mtpr	r25, pt0		// Stash VA
	stq	r18, osfsf_a2(sp) 	// a2

	stq	r11, osfsf_ps(sp)	// save old ps
	srl	r13, mm_stat_v_opcode-mm_stat_v_ra, r25// Isolate opcode

	stq	r29, osfsf_gp(sp) 	// save gp
	addq	r14, 4, r14		// inc PC past the ld/st

	stq	r17, osfsf_a1(sp)	// a1
	and	r25, mm_stat_m_opcode, r17// Clean opocde for a1

	stq	r16, osfsf_a0(sp)	// save regs
	mfpr	r16, pt0		// a0 <- va/unlock

	stq	r14, osfsf_pc(sp)	// save pc
	mfpr	r25, pt_entuna		// get entry point


	bis	r12, r31, r11		// update ps
	br 	r31, unalign_trap_cont




// .sbttl	"DFAULT	- Dstream Fault Trap Entry Point"

//+
// DFAULT - offset 0380
// Entry:
//	Vectored into via hardware trap on dstream fault or sign check error on DVA.
//
// Function:
//	Ignore faults on FETCH/FETCH_M
//	Check for DFAULT in PAL
//	Build stack frame
//	a0 <- Faulting VA
//	a1 <- MMCSR (1 for ACV, 2 for FOR, 4 for FOW)
//	a2 <- R/W
//	vector via entMM
//
//-
        HDW_VECTOR(PAL_D_FAULT_ENTRY)
Trap_Dfault:
//	DEBUGSTORE(0x48)
	sll	r11, 63-osfps_v_mode, r25 // Shift mode up to MS bit
	mtpr	r31, ev5__ps		// Set Ibox current mode to kernel

	mfpr	r13, ev5__mm_stat	// Get mmstat
	mfpr	r8, exc_addr		// get pc, preserve r14
	
	srl	r13, mm_stat_v_opcode, r9 // Shift opcode field to ls bits
	blbs	r8, dfault_in_pal

	bis	r8, r31, r14		// move exc_addr to correct place
	bis	r11, r31, r12		// Save PS 
	
	mtpr	r31, ev5__dtb_cm	// Set Mbox current mode to kernel - 
					//     no virt ref for next 2 cycles
	and	r9, mm_stat_m_opcode, r9 // Clean all but opcode
	
        cmpeq   r9, evx_opc_sync, r9 	// Is the opcode fetch/fetchm?
        bne     r9, dfault_fetch_ldr31_err   // Yes, dismiss the fault	

	//dismiss exception if load to r31/f31
	blbs	r13, dfault_no_dismiss	// mm_stat<0> set on store or fetchm

					// not a store or fetch, must be a load
	srl	r13, mm_stat_v_ra, r9	// Shift rnum to low bits
	
	and	r9, 0x1F, r9		// isolate rnum
	nop

	cmpeq   r9, 0x1F, r9   	// Is the rnum r31 or f31?
        bne     r9, dfault_fetch_ldr31_err    // Yes, dismiss the fault

dfault_no_dismiss:
	and	r13, 0xf, r13	// Clean extra bits in mm_stat
	bge	r25, dfault_trap_cont	// no stack swap needed if cm=kern


	mtpr	r30, pt_usp		// save user stack
	bis	r31, r31, r12		// Set new PS	

	mfpr	r30, pt_ksp
	br	r31, dfault_trap_cont





// .sbttl	"MCHK	-  Machine Check Trap Entry Point"

//+
// MCHK - offset 0400
// Entry:
//	Vectored into via hardware trap on machine check.
//
// Function:
//
//-

        HDW_VECTOR(PAL_MCHK_ENTRY)
Trap_Mchk:
	DEBUGSTORE(0x49)
        mtpr    r31, ic_flush_ctl       // Flush the Icache
        br      r31, sys_machine_check




// .sbttl	"OPCDEC	-  Illegal Opcode Trap Entry Point"

//+
// OPCDEC - offset 0480
// Entry:
//	Vectored into via hardware trap on illegal opcode.
//
//	Build stack frame
//	a0 <- code
//	a1 <- unpred
//	a2 <- unpred
//	vector via entIF
//
//-

        HDW_VECTOR(PAL_OPCDEC_ENTRY)
Trap_Opcdec:
	DEBUGSTORE(0x4a)
//simos	DEBUG_EXC_ADDR()
	sll	r11, 63-osfps_v_mode, r25 // Shift mode up to MS bit
	mtpr	r31, ev5__ps		// Set Ibox current mode to kernel

	mfpr	r14, exc_addr		// get pc
	blbs	r14, pal_pal_bug_check	// check opcdec in palmode

	bis	r11, r31, r12		// Save PS
	bge	r25, TRAP_OPCDEC_10_		// no stack swap needed if cm=kern


	mtpr	r31, ev5__dtb_cm	// Set Mbox current mode to kernel - 
					//     no virt ref for next 2 cycles
	mtpr	r30, pt_usp		// save user stack

	bis	r31, r31, r12		// Set new PS	
	mfpr	r30, pt_ksp

TRAP_OPCDEC_10_:	
	lda	sp, 0-osfsf_c_size(sp)// allocate stack space 	
	addq	r14, 4, r14		// inc pc

	stq	r16, osfsf_a0(sp)	// save regs
	bis	r31, osf_a0_opdec, r16	// set a0

	stq	r11, osfsf_ps(sp)	// save old ps
	mfpr	r13, pt_entif		// get entry point

	stq	r18, osfsf_a2(sp) 	// a2
	stq	r17, osfsf_a1(sp)	// a1

	stq	r29, osfsf_gp(sp) 	// save gp
	stq	r14, osfsf_pc(sp)	// save pc

	bis	r12, r31, r11		// update ps
	mtpr	r13, exc_addr		// load exc_addr with entIF
					// 1 cycle to hw_rei, E1

	mfpr	r29, pt_kgp		// get the kgp, E1

	hw_rei_spe			// done, E1
	





// .sbttl	"ARITH	-  Arithmetic Exception Trap Entry Point"

//+
// ARITH - offset 0500
// Entry:
//	Vectored into via hardware trap on arithmetic excpetion.
//
// Function:
//	Build stack frame
//	a0 <- exc_sum
//	a1 <- exc_mask
//	a2 <- unpred
//	vector via entArith
//
//-
        HDW_VECTOR(PAL_ARITH_ENTRY)
Trap_Arith:
	DEBUGSTORE(0x4b)
	and	r11, osfps_m_mode, r12 // get mode bit
	mfpr	r31, ev5__va		// unlock mbox

	bis	r11, r31, r25		// save ps
	mfpr	r14, exc_addr		// get pc

	nop
	blbs	r14, pal_pal_bug_check	// arith trap from PAL

        mtpr    r31, ev5__dtb_cm        // Set Mbox current mode to kernel -
                                        //     no virt ref for next 2 cycles
	beq	r12, TRAP_ARITH_10_		// if zero we are in kern now

	bis	r31, r31, r25		// set the new ps
	mtpr	r30, pt_usp		// save user stack

	nop
	mfpr	r30, pt_ksp		// get kern stack

TRAP_ARITH_10_: 	lda	sp, 0-osfsf_c_size(sp)	// allocate stack space 	
	mtpr	r31, ev5__ps		// Set Ibox current mode to kernel
	
	nop				// Pad current mode write and stq
 	mfpr	r13, ev5__exc_sum	// get the exc_sum

	mfpr	r12, pt_entarith
	stq	r14, osfsf_pc(sp)	// save pc

	stq	r17, osfsf_a1(sp)
        mfpr    r17, ev5__exc_mask      // Get exception register mask IPR - no mtpr exc_sum in next cycle

	stq	r11, osfsf_ps(sp)	// save ps
	bis	r25, r31, r11		// set new ps

	stq	r16, osfsf_a0(sp)	// save regs
	srl	r13, exc_sum_v_swc, r16// shift data to correct position

	stq	r18, osfsf_a2(sp) 
//	pvc_violate 354			// ok, but make sure reads of exc_mask/sum are not in same trap shadow
	mtpr	r31, ev5__exc_sum	// Unlock exc_sum and exc_mask

	stq	r29, osfsf_gp(sp)
	mtpr	r12, exc_addr		// Set new PC - 1 bubble to hw_rei - E1

	mfpr	r29, pt_kgp		// get the kern gp - E1
	hw_rei_spe			// done - E1






// .sbttl	"FEN	-  Illegal Floating Point Operation Trap Entry Point"

//+
// FEN - offset 0580
// Entry:
//	Vectored into via hardware trap on illegal FP op.
//
// Function:
//	Build stack frame
//	a0 <- code
//	a1 <- unpred
//	a2 <- unpred
//	vector via entIF
//
//-

        HDW_VECTOR(PAL_FEN_ENTRY)
Trap_Fen:
	sll	r11, 63-osfps_v_mode, r25 // Shift mode up to MS bit
	mtpr	r31, ev5__ps		// Set Ibox current mode to kernel

	mfpr	r14, exc_addr		// get pc
	blbs	r14, pal_pal_bug_check	// check opcdec in palmode

	mfpr	r13, ev5__icsr
	nop

	bis	r11, r31, r12		// Save PS
	bge	r25, TRAP_FEN_10_		// no stack swap needed if cm=kern

	mtpr	r31, ev5__dtb_cm	// Set Mbox current mode to kernel - 
					//     no virt ref for next 2 cycles
	mtpr	r30, pt_usp		// save user stack

	bis	r31, r31, r12		// Set new PS	
	mfpr	r30, pt_ksp

TRAP_FEN_10_:	
	lda	sp, 0-osfsf_c_size(sp)// allocate stack space 	
        srl     r13, icsr_v_fpe, r25   // Shift FP enable to bit 0
	

	stq	r16, osfsf_a0(sp)	// save regs
	mfpr	r13, pt_entif		// get entry point

	stq	r18, osfsf_a2(sp) 	// a2
	stq	r11, osfsf_ps(sp)	// save old ps

	stq	r29, osfsf_gp(sp) 	// save gp
	bis	r12, r31, r11		// set new ps

	stq	r17, osfsf_a1(sp)	// a1
	blbs	r25,fen_to_opcdec	// If FP is enabled, this is really OPCDEC.

	bis	r31, osf_a0_fen, r16	// set a0
	stq	r14, osfsf_pc(sp)	// save pc

	mtpr	r13, exc_addr		// load exc_addr with entIF
					// 1 cycle to hw_rei -E1

	mfpr	r29, pt_kgp		// get the kgp -E1

	hw_rei_spe			// done -E1

//	FEN trap was taken, but the fault is really opcdec.
	ALIGN_BRANCH
fen_to_opcdec:
	addq	r14, 4, r14		// save PC+4
	bis	r31, osf_a0_opdec, r16	// set a0

	stq	r14, osfsf_pc(sp)	// save pc
	mtpr	r13, exc_addr		// load exc_addr with entIF
					// 1 cycle to hw_rei

	mfpr	r29, pt_kgp		// get the kgp
	hw_rei_spe			// done

	

// .sbttl	"Misc handlers"
						// Start area for misc code.
//+
//dfault_trap_cont
//	A dfault trap has been taken.  The sp has been updated if necessary.
//	Push a stack frame a vector via entMM.
//
//	Current state:
//		r12 - new PS
//		r13 - MMstat
//		VA - locked
//		
//-
	ALIGN_BLOCK
dfault_trap_cont:
	lda	sp, 0-osfsf_c_size(sp)// allocate stack space 	
	mfpr	r25, ev5__va		// Fetch VA/unlock

	stq	r18, osfsf_a2(sp) 	// a2
	and	r13, 1, r18		// Clean r/w bit for a2

	stq	r16, osfsf_a0(sp)	// save regs
	bis	r25, r31, r16		// a0 <- va

	stq	r17, osfsf_a1(sp)	// a1
	srl	r13, 1, r17		// shift fault bits to right position

	stq	r11, osfsf_ps(sp)	// save old ps
	bis	r12, r31, r11		// update ps

	stq	r14, osfsf_pc(sp)	// save pc
	mfpr	r25, pt_entmm		// get entry point
	
	stq	r29, osfsf_gp(sp) 	// save gp
	cmovlbs	r17, 1, r17		// a2. acv overrides fox.

	mtpr	r25, exc_addr		// load exc_addr with entMM
					// 1 cycle to hw_rei
	mfpr	r29, pt_kgp		// get the kgp

	hw_rei_spe			// done
	
//+
//unalign_trap_cont
//	An unalign trap has been taken.  Just need to finish up a few things.
//
//	Current state:
//		r25 - entUna
//		r13 - shifted MMstat
//		
//-
	ALIGN_BLOCK
unalign_trap_cont:
	mtpr	r25, exc_addr		// load exc_addr with entUna
					// 1 cycle to hw_rei


	mfpr	r29, pt_kgp		// get the kgp
	and	r13, mm_stat_m_ra, r18	// Clean Ra for a2

	hw_rei_spe			// done
	


//+
// dfault_in_pal
//	Dfault trap was taken, exc_addr points to a PAL PC.
//	r9 - mmstat<opcode> right justified
//	r8 - exception address
//
//	These are the cases:
//		opcode was STQ -- from a stack builder, KSP not valid halt
//			r14 - original exc_addr
//			r11 - original PS
//		opcode was STL_C  -- rti or retsys clear lock_flag by stack write,
//					KSP not valid halt
//			r11 - original PS
//			r14 - original exc_addr
//		opcode was LDQ -- retsys or rti stack read, KSP not valid halt
//			r11 - original PS
//			r14 - original exc_addr
//		opcode was HW_LD -- itbmiss or dtbmiss, bugcheck due to fault on page tables
//			r10 - original exc_addr
//			r11 - original PS
//
//
//-
	ALIGN_BLOCK
dfault_in_pal:
	DEBUGSTORE(0x50)
	bic     r8, 3, r8            // Clean PC
	mfpr	r9, pal_base

	mfpr	r31, va			// unlock VA
#if real_mm != 0
 			// if not real_mm, should never get here from miss flows

        subq    r9, r8, r8            // pal_base - offset

        lda     r9, pal_itb_ldq-pal_base(r8)
	nop

	beq 	r9, dfault_do_bugcheck
        lda     r9, pal_dtb_ldq-pal_base(r8)

	beq 	r9, dfault_do_bugcheck
#endif

//
// KSP invalid halt case --
ksp_inval_halt:
	DEBUGSTORE(76)
	bic	r11, osfps_m_mode, r11	// set ps to kernel mode
        mtpr    r0, pt0

	mtpr	r31, dtb_cm		// Make sure that the CM IPRs are all kernel mode
	mtpr	r31, ips

	mtpr	r14, exc_addr		// Set PC to instruction that caused trouble
//orig	pvc_jsr updpcb, bsr=1
        bsr     r0, pal_update_pcb      // update the pcb

        lda     r0, hlt_c_ksp_inval(r31)  // set halt code to hw halt
        br      r31, sys_enter_console  // enter the console

	ALIGN_BRANCH
dfault_do_bugcheck:
	bis	r10, r31, r14		// bugcheck expects exc_addr in r14
	br	r31, pal_pal_bug_check	


	ALIGN_BLOCK
//+
// dfault_fetch_ldr31_err - ignore faults on fetch(m) and loads to r31/f31
//	On entry -
//		r14 - exc_addr
//		VA is locked
//
//-
dfault_fetch_ldr31_err:
	mtpr	r11, ev5__dtb_cm
	mtpr	r11, ev5__ps		// Make sure ps hasn't changed

	mfpr	r31, va			// unlock the mbox
	addq	r14, 4, r14		// inc the pc to skip the fetch
	
	mtpr	r14, exc_addr		// give ibox new PC
	mfpr	r31, pt0		// pad exc_addr write
	
	hw_rei



	ALIGN_BLOCK
//+
// sys_from_kern
//	callsys from kernel mode - OS bugcheck machine check
//
//-
sys_from_kern:
	mfpr	r14, exc_addr			// PC points to call_pal
	subq	r14, 4, r14

	lda	r25, mchk_c_os_bugcheck(r31)    // fetch mchk code
        br      r31, pal_pal_mchk


// .sbttl	"Continuation of long call_pal flows"
	ALIGN_BLOCK
//+
// wrent_tbl
//	Table to write *int in paltemps.
//	4 instructions/entry
//	r16 has new value
//
//-
wrent_tbl:
//orig	pvc_jsr	wrent, dest=1
	nop
	mtpr	r16, pt_entint
	
	mfpr	r31, pt0		// Pad for mt->mf paltemp rule
	hw_rei


//orig	pvc_jsr	wrent, dest=1
	nop
	mtpr	r16, pt_entarith

        mfpr    r31, pt0                // Pad for mt->mf paltemp rule
	hw_rei


//orig	pvc_jsr	wrent, dest=1
	nop
	mtpr	r16, pt_entmm
	
        mfpr    r31, pt0                // Pad for mt->mf paltemp rule	
	hw_rei


//orig	pvc_jsr	wrent, dest=1
	nop
	mtpr	r16, pt_entif

        mfpr    r31, pt0                // Pad for mt->mf paltemp rule
	hw_rei


//orig	pvc_jsr	wrent, dest=1
	nop
	mtpr	r16, pt_entuna

        mfpr    r31, pt0                // Pad for mt->mf paltemp rule
	hw_rei


//orig	pvc_jsr	wrent, dest=1
	nop
	mtpr	r16, pt_entsys

        mfpr    r31, pt0                // Pad for mt->mf paltemp rule
	hw_rei

	ALIGN_BLOCK
//+
// tbi_tbl
//	Table to do tbi instructions
//	4 instructions per entry
//-
tbi_tbl:
	// -2 tbia
//orig	pvc_jsr tbi, dest=1
	mtpr	r31, ev5__dtb_ia	// Flush DTB
	mtpr	r31, ev5__itb_ia	// Flush ITB

#if icflush_on_tbix != 0
	

	br	r31, pal_ic_flush		// Flush Icache
#else 

 	hw_rei_stall
#endif	

	nop				// Pad table

	// -1 tbiap
//orig	pvc_jsr tbi, dest=1
	mtpr	r31, ev5__dtb_iap	// Flush DTB
	mtpr	r31, ev5__itb_iap	// Flush ITB
	
#if icflush_on_tbix != 0
	

	br	r31, pal_ic_flush		// Flush Icache
#else 

 	hw_rei_stall
#endif	

	nop				// Pad table


	// 0 unused
//orig	pvc_jsr tbi, dest=1
	hw_rei				// Pad table
	nop
	nop
	nop


	// 1 tbisi
//orig	pvc_jsr tbi, dest=1
#if icflush_on_tbix != 0
	


	nop	
	br	r31, pal_ic_flush_and_tbisi		// Flush Icache
	nop
	nop				// Pad table
#else 

	nop
	nop
	mtpr	r17, ev5__itb_is	// Flush ITB
	hw_rei_stall
#endif	



	// 2 tbisd
//orig	pvc_jsr tbi, dest=1
	mtpr	r17, ev5__dtb_is	// Flush DTB.
	nop

	nop
	hw_rei_stall


	// 3 tbis
//orig	pvc_jsr tbi, dest=1
	mtpr	r17, ev5__dtb_is	// Flush DTB
#if icflush_on_tbix != 0
	

	br	r31, pal_ic_flush_and_tbisi	// Flush Icache and ITB
#else
	br	r31, tbi_finish	
	ALIGN_BRANCH	
tbi_finish:
	mtpr	r17, ev5__itb_is	// Flush ITB
	hw_rei_stall
#endif



	ALIGN_BLOCK
//+
// bpt_bchk_common:
//	Finish up the bpt/bchk instructions
//-
bpt_bchk_common:
	stq	r18, osfsf_a2(sp) 	// a2
	mfpr	r13, pt_entif		// get entry point

	stq	r12, osfsf_ps(sp)	// save old ps
	stq	r14, osfsf_pc(sp)	// save pc

	stq	r29, osfsf_gp(sp) 	// save gp
	mtpr	r13, exc_addr		// load exc_addr with entIF
					// 1 cycle to hw_rei

	mfpr	r29, pt_kgp		// get the kgp


	hw_rei_spe			// done


	ALIGN_BLOCK
//+
// rti_to_user
//	Finish up the rti instruction
//-
rti_to_user:
	mtpr	r11, ev5__dtb_cm	// set Mbox current mode - no virt ref for 2 cycles
	mtpr	r11, ev5__ps		// set Ibox current mode - 2 bubble to hw_rei

	mtpr	r31, ev5__ipl		// set the ipl. No hw_rei for 2 cycles
	mtpr	r25, pt_ksp		// save off incase RTI to user

	mfpr	r30, pt_usp
	hw_rei_spe			// and back


	ALIGN_BLOCK
//+
// rti_to_kern
//	Finish up the rti instruction
//-
rti_to_kern:
	and	r12, osfps_m_ipl, r11	// clean ps
	mfpr	r12, pt_intmask		// get int mask

	extbl	r12, r11, r12		// get mask for this ipl
	mtpr	r25, pt_ksp		// save off incase RTI to user

	mtpr	r12, ev5__ipl		// set the new ipl.
	or	r25, r31, sp		// sp

//	pvc_violate 217			// possible hidden mt->mf ipl not a problem in callpals 
	hw_rei	

	ALIGN_BLOCK
//+
// swpctx_cont
//	Finish up the swpctx instruction
//-
	
swpctx_cont:	
#if ev5_p1 != 0


	bic	r25, r24, r25		// clean icsr<FPE>
	get_impure r8			// get impure pointer

	sll	r12, icsr_v_fpe, r12	// shift new fen to pos
	fix_impure_ipr r8		// adjust impure pointer

	restore_reg1 pmctr_ctl, r8, r8, ipr=1	// "ldqp" - get pmctr_ctl bits
	srl	r23, 32, r24		// move asn to low asn pos

	ldqp	r14, osfpcb_q_mmptr(r16)// get new mmptr
	srl	r22, osfpcb_v_pme, r22		// get pme down to bit 0
	
	or	r25, r12, r25		// icsr with new fen
	sll	r24, itb_asn_v_asn, r12

#else

	bic	r25, r24, r25		// clean icsr<FPE,PMP>
	sll	r12, icsr_v_fpe, r12	// shift new fen to pos

	ldqp	r14, osfpcb_q_mmptr(r16)// get new mmptr
	srl	r22, osfpcb_v_pme, r22	// get pme down to bit 0

	or	r25, r12, r25		// icsr with new fen
	srl	r23, 32, r24		// move asn to low asn pos

	and	r22, 1, r22
	sll	r24, itb_asn_v_asn, r12

	sll	r22, icsr_v_pmp, r22
	nop

	or	r25, r22, r25		// icsr with new pme
#endif

	sll	r24, dtb_asn_v_asn, r24

	subl	r23, r13, r13		// gen new cc offset
	mtpr	r12, itb_asn		// no hw_rei_stall in 0,1,2,3,4

	mtpr	r24, dtb_asn		// Load up new ASN
	mtpr	r25, icsr		// write the icsr

	sll	r14, page_offset_size_bits, r14 // Move PTBR into internal position.
	ldqp	r25, osfpcb_q_usp(r16)	// get new usp

	insll	r13, 4, r13		// >> 32
//	pvc_violate 379			// ldqp can't trap except replay.  only problem if mf same ipr in same shadow
	mtpr	r14, pt_ptbr		// load the new ptbr

	mtpr	r13, cc			// set new offset
   	ldqp	r30, osfpcb_q_ksp(r16)	// get new ksp

//	pvc_violate 379			// ldqp can't trap except replay.  only problem if mf same ipr in same shadow
	mtpr	r25, pt_usp		// save usp

#if ev5_p1 != 0


	blbc	r8, no_pm_change		// if monitoring all processes -- no need to change pm 

	// otherwise, monitoring select processes - update pm
	lda	r25, 0x3F(r31)
	cmovlbc	r22, r31, r8			// if pme set, disable counters, otherwise use saved encodings

	sll	r25, pmctr_v_ctl2, r25		// create ctl field bit mask
	mfpr	r22, ev5__pmctr

	and	r8, r25, r8			// mask new ctl value
	bic	r22, r25, r22			// clear ctl field in pmctr

	or	r8, r22, r8
	mtpr	r8, ev5__pmctr

no_pm_change:
#endif


#if osf_chm_fix != 0


	p4_fixup_hw_rei_stall		// removes this section for Pass 4 by placing a hw_rei_stall here

#if build_fixed_image != 0


	hw_rei_stall
#else

	mfpr	r9, pt_pcbb		// get FEN
#endif

	ldqp	r9, osfpcb_q_fen(r9)
	blbc	r9, no_pm_change_10_			// skip if FEN disabled

	mb				// ensure no outstanding fills					
	lda r12, 1<<dc_mode_v_dc_ena(r31)
	mtpr	r12, dc_mode		// turn dcache on so we can flush it
	nop				// force correct slotting
	mfpr	r31, pt0		// no mbox instructions in 1,2,3,4
	mfpr	r31, pt0		// no mbox instructions in 1,2,3,4
	mfpr	r31, pt0		// no mbox instructions in 1,2,3,4
	mfpr	r31, pt0		// no mbox instructions in 1,2,3,4

	lda	r8, 0(r31)		// flood the dcache with junk data
no_pm_change_5_:	ldqp	r31, 0(r8)			
	lda	r8, 0x20(r8)		// touch each cache block
	srl	r8, 13, r9
	blbc	r9, no_pm_change_5_	

	mb				// ensure no outstanding fills
	mtpr	r31, dc_mode		// turn the dcache back off
	nop				// force correct slotting
	mfpr	r31, pt0		// no hw_rei_stall in 0,1
#endif


no_pm_change_10_:	hw_rei_stall			// back we go

	ALIGN_BLOCK
//+
// swppal_cont - finish up the swppal call_pal
//-

swppal_cont:
	mfpr	r2, pt_misc		// get misc bits
	sll	r0, pt_misc_v_switch, r0 // get the "I've switched" bit
	or	r2, r0, r2		// set the bit
	mtpr	r31, ev5__alt_mode	// ensure alt_mode set to 0 (kernel)
	mtpr	r2, pt_misc		// update the chip

	or	r3, r31, r4
	mfpr	r3, pt_impure		// pass pointer to the impure area in r3
//orig	fix_impure_ipr	r3		// adjust impure pointer for ipr read
//orig	restore_reg1	bc_ctl, r1, r3, ipr=1		// pass cns_bc_ctl in r1
//orig	restore_reg1	bc_config, r2, r3, ipr=1	// pass cns_bc_config in r2
//orig	unfix_impure_ipr r3		// restore impure pointer
	lda	r3, CNS_Q_IPR(r3)
	RESTORE_SHADOW(r1,CNS_Q_BC_CTL,r3); 
	RESTORE_SHADOW(r1,CNS_Q_BC_CFG,r3); 
	lda	r3, -CNS_Q_IPR(r3)
	
	or	r31, r31, r0		// set status to success
//	pvc_violate	1007
	jmp	r31, (r4)		// and call our friend, it's her problem now


swppal_fail:
	addq	r0, 1, r0		// set unknown pal or not loaded
	hw_rei				// and return


// .sbttl	"Memory management"

	ALIGN_BLOCK
//+
//foe_ipte_handler
// IFOE detected on level 3 pte, sort out FOE vs ACV
//
// on entry:
//	with
//	R8	 = pte
//	R10	 = pc
//
// Function
//	Determine TNV vs ACV vs FOE. Build stack and dispatch
//	Will not be here if TNV.
//-

foe_ipte_handler:
	sll	r11, 63-osfps_v_mode, r25 // Shift mode up to MS bit
	mtpr	r31, ev5__ps		// Set Ibox current mode to kernel

	bis	r11, r31, r12		// Save PS for stack write
	bge	r25, foe_ipte_handler_10_		// no stack swap needed if cm=kern


	mtpr	r31, ev5__dtb_cm	// Set Mbox current mode to kernel - 
					//     no virt ref for next 2 cycles
	mtpr	r30, pt_usp		// save user stack

	bis	r31, r31, r11		// Set new PS	
	mfpr	r30, pt_ksp

	srl	r8, osfpte_v_ure-osfpte_v_kre, r8 // move pte user bits to kern 
	nop

foe_ipte_handler_10_:	srl	r8, osfpte_v_kre, r25	// get kre to <0>
	lda	sp, 0-osfsf_c_size(sp)// allocate stack space 	
	
	or	r10, r31, r14		// Save pc/va in case TBmiss or fault on stack
	mfpr	r13, pt_entmm		// get entry point

	stq	r16, osfsf_a0(sp)	// a0
	or	r14, r31, r16		// pass pc/va as a0

	stq	r17, osfsf_a1(sp)	// a1
	nop

	stq	r18, osfsf_a2(sp) 	// a2
	lda	r17, mmcsr_c_acv(r31)	// assume ACV

	stq	r16, osfsf_pc(sp)	// save pc
	cmovlbs r25, mmcsr_c_foe, r17	// otherwise FOE

	stq	r12, osfsf_ps(sp)	// save ps
	subq	r31, 1, r18		// pass flag of istream as a2

	stq	r29, osfsf_gp(sp) 
	mtpr	r13, exc_addr		// set vector address

	mfpr	r29, pt_kgp		// load kgp
	hw_rei_spe			// out to exec

	ALIGN_BLOCK
//+
//invalid_ipte_handler
// TNV detected on level 3 pte, sort out TNV vs ACV
//
// on entry:
//	with
//	R8	 = pte
//	R10	 = pc
//
// Function
//	Determine TNV vs ACV. Build stack and dispatch.
//-

invalid_ipte_handler:
	sll	r11, 63-osfps_v_mode, r25 // Shift mode up to MS bit
	mtpr	r31, ev5__ps		// Set Ibox current mode to kernel

	bis	r11, r31, r12		// Save PS for stack write
	bge	r25, invalid_ipte_handler_10_		// no stack swap needed if cm=kern


	mtpr	r31, ev5__dtb_cm	// Set Mbox current mode to kernel - 
					//     no virt ref for next 2 cycles
	mtpr	r30, pt_usp		// save user stack

	bis	r31, r31, r11		// Set new PS	
	mfpr	r30, pt_ksp

	srl	r8, osfpte_v_ure-osfpte_v_kre, r8 // move pte user bits to kern 
	nop

invalid_ipte_handler_10_:	srl	r8, osfpte_v_kre, r25	// get kre to <0>
	lda	sp, 0-osfsf_c_size(sp)// allocate stack space 	
	
	or	r10, r31, r14		// Save pc/va in case TBmiss on stack
	mfpr	r13, pt_entmm		// get entry point

	stq	r16, osfsf_a0(sp)	// a0
	or	r14, r31, r16		// pass pc/va as a0

	stq	r17, osfsf_a1(sp)	// a1
	nop

	stq	r18, osfsf_a2(sp) 	// a2
	and	r25, 1, r17		// Isolate kre

	stq	r16, osfsf_pc(sp)	// save pc
	xor	r17, 1, r17		// map to acv/tnv as a1

	stq	r12, osfsf_ps(sp)	// save ps
	subq	r31, 1, r18		// pass flag of istream as a2

	stq	r29, osfsf_gp(sp) 
	mtpr	r13, exc_addr		// set vector address

	mfpr	r29, pt_kgp		// load kgp
	hw_rei_spe			// out to exec
	
	


	ALIGN_BLOCK
//+
//invalid_dpte_handler
// INVALID detected on level 3 pte, sort out TNV vs ACV
//
// on entry:
//	with
//	R10	 = va
//	R8	 = pte
//	R9	 = mm_stat
//	PT6	 = pc
//
// Function
//	Determine TNV vs ACV. Build stack and dispatch
//-


invalid_dpte_handler:
	mfpr	r12, pt6
	blbs	r12, tnv_in_pal		// Special handler if original faulting reference was in PALmode
	
	bis	r12, r31, r14		// save PC in case of tbmiss or fault
	srl	r9, mm_stat_v_opcode, r25	// shift opc to <0>

	mtpr	r11, pt0		// Save PS for stack write
	and 	r25, mm_stat_m_opcode, r25	// isolate opcode

	cmpeq	r25, evx_opc_sync, r25	// is it FETCH/FETCH_M?
	blbs	r25, nmiss_fetch_ldr31_err	// yes

	//dismiss exception if load to r31/f31
	blbs	r9, invalid_dpte_no_dismiss	// mm_stat<0> set on store or fetchm

					// not a store or fetch, must be a load
	srl	r9, mm_stat_v_ra, r25	// Shift rnum to low bits
	
	and	r25, 0x1F, r25		// isolate rnum
	nop

	cmpeq   r25, 0x1F, r25  	// Is the rnum r31 or f31?
        bne     r25, nmiss_fetch_ldr31_err    // Yes, dismiss the fault

invalid_dpte_no_dismiss:
	sll	r11, 63-osfps_v_mode, r25 // Shift mode up to MS bit
	mtpr	r31, ev5__ps		// Set Ibox current mode to kernel

	mtpr	r31, ev5__dtb_cm	// Set Mbox current mode to kernel - 
					//     no virt ref for next 2 cycles
	bge	r25, invalid_dpte_no_dismiss_10_		// no stack swap needed if cm=kern

	srl	r8, osfpte_v_ure-osfpte_v_kre, r8 // move pte user bits to kern 
	mtpr	r30, pt_usp		// save user stack

	bis	r31, r31, r11		// Set new PS	
	mfpr	r30, pt_ksp

invalid_dpte_no_dismiss_10_:	srl	r8, osfpte_v_kre, r12	// get kre to <0>
	lda	sp, 0-osfsf_c_size(sp)// allocate stack space 	
	
	or	r10, r31, r25		// Save va in case TBmiss on stack
	and	r9, 1, r13		// save r/w flag

	stq	r16, osfsf_a0(sp)	// a0
	or	r25, r31, r16		// pass va as a0

	stq	r17, osfsf_a1(sp)	// a1
	or	r31, mmcsr_c_acv, r17 	// assume acv 

	srl	r12, osfpte_v_kwe-osfpte_v_kre, r25 // get write enable to <0>
	stq	r29, osfsf_gp(sp) 

	stq	r18, osfsf_a2(sp) 	// a2
	cmovlbs r13, r25, r12		// if write access move acv based on write enable

	or	r13, r31, r18		// pass flag of dstream access and read vs write
	mfpr	r25, pt0		// get ps

	stq	r14, osfsf_pc(sp)	// save pc
	mfpr	r13, pt_entmm		// get entry point

	stq	r25, osfsf_ps(sp)	// save ps
	mtpr	r13, exc_addr		// set vector address

	mfpr	r29, pt_kgp		// load kgp
	cmovlbs	r12, mmcsr_c_tnv, r17 	// make p2 be tnv if access ok else acv

	hw_rei_spe			// out to exec

//+
//
// We come here if we are erring on a dtb_miss, and the instr is a
// fetch, fetch_m, of load to r31/f31.
// The PC is incremented, and we return to the program.
// essentially ignoring the instruction and error.
//
//-
	ALIGN_BLOCK
nmiss_fetch_ldr31_err:			
	mfpr	r12, pt6
	addq	r12, 4, r12		// bump pc to pc+4

	mtpr	r12, exc_addr		// and set entry point
	mfpr	r31, pt0		// pad exc_addr write

	hw_rei				// 

	ALIGN_BLOCK
//+
// double_pte_inv
//	We had a single tbmiss which turned into a double tbmiss which found
//	an invalid PTE.  Return to single miss with a fake pte, and the invalid
//	single miss flow will report the error.
//
// on entry:
//	r21  	PTE
//	r22	available
//	VA IPR	locked with original fault VA
//       pt4  	saved r21
//	pt5  	saved r22
//	pt6	original exc_addr
//
// on return to tbmiss flow:
//	r8	fake PTE
//
//
//-
double_pte_inv:    
	srl	r21, osfpte_v_kre, r21	// get the kre bit to <0>
	mfpr	r22, exc_addr		// get the pc

	lda	r22, 4(r22)		// inc the pc
	lda	r8, osfpte_m_prot(r31)	 // make a fake pte with xre and xwe set

	cmovlbc r21, r31, r8		// set to all 0 for acv if pte<kre> is 0
	mtpr	r22, exc_addr		// set for rei

	mfpr	r21, pt4		// restore regs
	mfpr	r22, pt5		// restore regs

 	hw_rei				// back to tb miss

	ALIGN_BLOCK
//+
//tnv_in_pal
//	The only places in pal that ld or store are the
// 	stack builders, rti or retsys.  Any of these mean we
//	need to take a ksp not valid halt.
//
//-
tnv_in_pal:
	

	br	r31, ksp_inval_halt


// .sbttl	"Icache flush routines"

	ALIGN_BLOCK
//+
// Common Icache flush routine.
//
//
//-
pal_ic_flush:
	nop
	mtpr	r31, ev5__ic_flush_ctl		// Icache flush - E1 
	nop
	nop

// Now, do 44 NOPs.  3RFB prefetches (24) + IC buffer,IB,slot,issue (20)
	nop	
	nop
	nop
	nop				

	nop
	nop
	nop
	nop

	nop
	nop		// 10

	nop	
	nop
	nop
	nop				

	nop
	nop
	nop
	nop

	nop
	nop		// 20

	nop	
	nop
	nop
	nop				

	nop
	nop
	nop
	nop

	nop
	nop		// 30
	nop	
	nop
	nop
	nop				

	nop
	nop
	nop
	nop

	nop
	nop		// 40

	nop
	nop

one_cycle_and_hw_rei:
	nop
	nop
	
	hw_rei_stall

#if icflush_on_tbix != 0


	ALIGN_BLOCK

//+
// Common Icache flush and ITB invalidate single routine.
// ITBIS and hw_rei_stall must be in same octaword.
//	r17 - has address to invalidate
//
//-
PAL_IC_FLUSH_AND_TBISI:
	nop
	mtpr	r31, ev5__ic_flush_ctl		// Icache flush - E1 
	nop
	nop

// Now, do 44 NOPs.  3RFB prefetches (24) + IC buffer,IB,slot,issue (20)
	nop	
	nop
	nop
	nop				

	nop
	nop
	nop
	nop

	nop
	nop		// 10

	nop	
	nop
	nop
	nop				

	nop
	nop
	nop
	nop

	nop
	nop		// 20

	nop	
	nop
	nop
	nop				

	nop
	nop
	nop
	nop

	nop
	nop		// 30
	nop	
	nop
	nop
	nop				

	nop
	nop
	nop
	nop

	nop
	nop		// 40


	nop
	nop

	nop
	nop

	// A quadword is 64 bits, so an octaword is 128 bits -> 16 bytes -> 4 instructions
	// 44 nops plus 4 instructions before it is 48 instructions.
	// Since this routine started on a 32-byte (8 instruction) boundary, 
	// the following 2 instructions will be in the same octword as required.
//	ALIGN_BRANCH	
	mtpr	r17, ev5__itb_is	// Flush ITB
	hw_rei_stall

#endif

	ALIGN_BLOCK
//+
//osfpal_calpal_opcdec
//  Here for all opcdec CALL_PALs
//
//	Build stack frame
//	a0 <- code
//	a1 <- unpred
//	a2 <- unpred
//	vector via entIF
//
//-

osfpal_calpal_opcdec:
	sll	r11, 63-osfps_v_mode, r25 // Shift mode up to MS bit
	mtpr	r31, ev5__ps		// Set Ibox current mode to kernel

	mfpr	r14, exc_addr		// get pc
	nop

	bis	r11, r31, r12		// Save PS for stack write
	bge	r25, osfpal_calpal_opcdec_10_		// no stack swap needed if cm=kern


	mtpr	r31, ev5__dtb_cm	// Set Mbox current mode to kernel - 
					//     no virt ref for next 2 cycles
	mtpr	r30, pt_usp		// save user stack

	bis	r31, r31, r11		// Set new PS	
	mfpr	r30, pt_ksp

osfpal_calpal_opcdec_10_:	
	lda	sp, 0-osfsf_c_size(sp)// allocate stack space 	
	nop

	stq	r16, osfsf_a0(sp)	// save regs
	bis	r31, osf_a0_opdec, r16	// set a0

	stq	r18, osfsf_a2(sp) 	// a2
	mfpr	r13, pt_entif		// get entry point

	stq	r12, osfsf_ps(sp)	// save old ps
	stq	r17, osfsf_a1(sp)	// a1

	stq	r14, osfsf_pc(sp)	// save pc
 	nop

	stq	r29, osfsf_gp(sp) 	// save gp
	mtpr	r13, exc_addr		// load exc_addr with entIF
					// 1 cycle to hw_rei

	mfpr	r29, pt_kgp		// get the kgp


	hw_rei_spe			// done





//+
//pal_update_pcb
//	Update the PCB with the current SP, AST, and CC info
//
//	r0 - return linkage
//-
	ALIGN_BLOCK

pal_update_pcb:
	mfpr	r12, pt_pcbb		// get pcbb
	and	r11, osfps_m_mode, r25	// get mode 
	beq	r25, pal_update_pcb_10_		// in kern? no need to update user sp
	mtpr	r30, pt_usp		// save user stack
	stqp	r30, osfpcb_q_usp(r12)	// store usp
	br	r31, pal_update_pcb_20_		// join common
pal_update_pcb_10_:	stqp	r30, osfpcb_q_ksp(r12)	// store ksp
pal_update_pcb_20_:	rpcc	r13			// get cyccounter
	srl	r13, 32, r14		// move offset
 	addl	r13, r14, r14		// merge for new time
  	stlp	r14, osfpcb_l_cc(r12)	// save time

//orig	pvc_jsr	updpcb, bsr=1, dest=1
	ret	r31, (r0)

	

#if remove_save_state == 0
	
// .sbttl  "PAL_SAVE_STATE"
//+
//
// Pal_save_state
//
//	Function
//		All chip state saved, all PT's, SR's FR's, IPR's
//
//
// Regs' on entry...
//
//	R0 	= halt code
//	pt0	= r0
//	R1	= pointer to impure
//	pt4	= r1
//	R3	= return addr
//	pt5	= r3
//
//	register usage:
//		r0 = halt_code
//		r1 = addr of impure area
//		r3 = return_address
//		r4 = scratch
//
//-


	ALIGN_BLOCK
	.globl pal_save_state
pal_save_state:
//
//
// start of implementation independent save routine
//
// 		the impure area is larger than the addressibility of hw_ld and hw_st
//		therefore, we need to play some games:  The impure area 
//		is informally divided into the "machine independent" part and the
//		"machine dependent" part.  The state that will be saved in the
//    		"machine independent" part are gpr's, fpr's, hlt, flag, mchkflag (use  (un)fix_impure_gpr macros).
//		All others will be in the "machine dependent" part (use (un)fix_impure_ipr macros).
//		The impure pointer will need to be adjusted by a different offset for each.  The store/restore_reg
//		macros will automagically adjust the offset correctly.
//

// The distributed code is commented out and followed by corresponding SRC code.
// Beware: SAVE_IPR and RESTORE_IPR blow away r0(v0)
	
//orig	fix_impure_gpr	r1		// adjust impure area pointer for stores to "gpr" part of impure area
	lda	r1, 0x200(r1)		// Point to center of CPU segment
//orig	store_reg1 flag, r31, r1, ipr=1	// clear dump area flag
	SAVE_GPR(r31,CNS_Q_FLAG,r1)	// Clear the valid flag
//orig	store_reg1 hlt, r0, r1, ipr=1
	SAVE_GPR(r0,CNS_Q_HALT,r1)	// Save the halt code

	mfpr	r0, pt0			// get r0 back			//orig
//orig	store_reg1 0, r0, r1		// save r0
	SAVE_GPR(r0,CNS_Q_GPR+0x00,r1)	// Save r0

	mfpr	r0, pt4			// get r1 back			//orig
//orig	store_reg1 1, r0, r1		// save r1
	SAVE_GPR(r0,CNS_Q_GPR+0x08,r1)	// Save r1

//orig	store_reg 2			// save r2
	SAVE_GPR(r2,CNS_Q_GPR+0x10,r1)	// Save r2

	mfpr	r0, pt5			// get r3 back			//orig
//orig	store_reg1 3, r0, r1		// save r3
	SAVE_GPR(r0,CNS_Q_GPR+0x18,r1)	// Save r3

	// reason code has been saved
	// r0 has been saved
	// r1 has been saved
	// r2 has been saved
	// r3 has been saved
	// pt0, pt4, pt5 have been lost

	//
	// Get out of shadow mode
	//

	mfpr	r2, icsr		// Get icsr			//orig
//orig	ldah	r0, <1@<icsr_v_sde-16>>(r31)	// Get a one in SHADOW_ENABLE bit location
	ldah	r0, (1<<(icsr_v_sde-16))(r31)
	bic	r2, r0, r0		// ICSR with SDE clear		//orig
	mtpr	r0, icsr		// Turn off SDE			//orig
	
	mfpr	r31, pt0		// SDE bubble cycle 1		//orig
	mfpr	r31, pt0		// SDE bubble cycle 2		//orig
	mfpr	r31, pt0		// SDE bubble cycle 3		//orig
	nop								//orig

	
	// save integer regs R4-r31
//orig  #define t 4
//orig	.repeat 28
//orig	  store_reg \t
//orig #define t t + 1
//orig	.endr
	SAVE_GPR(r4,CNS_Q_GPR+0x20,r1)
	SAVE_GPR(r5,CNS_Q_GPR+0x28,r1)
	SAVE_GPR(r6,CNS_Q_GPR+0x30,r1)
	SAVE_GPR(r7,CNS_Q_GPR+0x38,r1)
	SAVE_GPR(r8,CNS_Q_GPR+0x40,r1)
	SAVE_GPR(r9,CNS_Q_GPR+0x48,r1)
	SAVE_GPR(r10,CNS_Q_GPR+0x50,r1)
	SAVE_GPR(r11,CNS_Q_GPR+0x58,r1)
	SAVE_GPR(r12,CNS_Q_GPR+0x60,r1)
	SAVE_GPR(r13,CNS_Q_GPR+0x68,r1)
	SAVE_GPR(r14,CNS_Q_GPR+0x70,r1)
	SAVE_GPR(r15,CNS_Q_GPR+0x78,r1)
	SAVE_GPR(r16,CNS_Q_GPR+0x80,r1)
	SAVE_GPR(r17,CNS_Q_GPR+0x88,r1)
	SAVE_GPR(r18,CNS_Q_GPR+0x90,r1)
	SAVE_GPR(r19,CNS_Q_GPR+0x98,r1)
	SAVE_GPR(r20,CNS_Q_GPR+0xA0,r1)
	SAVE_GPR(r21,CNS_Q_GPR+0xA8,r1)
	SAVE_GPR(r22,CNS_Q_GPR+0xB0,r1)
	SAVE_GPR(r23,CNS_Q_GPR+0xB8,r1)
	SAVE_GPR(r24,CNS_Q_GPR+0xC0,r1)
	SAVE_GPR(r25,CNS_Q_GPR+0xC8,r1)
	SAVE_GPR(r26,CNS_Q_GPR+0xD0,r1)
	SAVE_GPR(r27,CNS_Q_GPR+0xD8,r1)
	SAVE_GPR(r28,CNS_Q_GPR+0xE0,r1)
	SAVE_GPR(r29,CNS_Q_GPR+0xE8,r1)
	SAVE_GPR(r30,CNS_Q_GPR+0xF0,r1)
	SAVE_GPR(r31,CNS_Q_GPR+0xF8,r1)

	// save all paltemp regs except pt0
	
//orig	unfix_impure_gpr	r1		// adjust impure area pointer for gpr stores
//orig	fix_impure_ipr	r1			// adjust impure area pointer for pt stores
//orig #define t 1
//orig	.repeat 23
//orig	  store_reg \t	, pal=1
//orig #define t t + 1
//orig	.endr

	lda	r1, -0x200(r1)		// Restore the impure base address.
	lda	r1, CNS_Q_IPR(r1)	// Point to the base of IPR area.
	SAVE_IPR(pt0,CNS_Q_PT+0x00,r1)		// the osf code didn't save/restore palTemp 0 ?? pboyle
	SAVE_IPR(pt1,CNS_Q_PT+0x08,r1)
	SAVE_IPR(pt2,CNS_Q_PT+0x10,r1)
	SAVE_IPR(pt3,CNS_Q_PT+0x18,r1)
	SAVE_IPR(pt4,CNS_Q_PT+0x20,r1)
	SAVE_IPR(pt5,CNS_Q_PT+0x28,r1)
	SAVE_IPR(pt6,CNS_Q_PT+0x30,r1)
	SAVE_IPR(pt7,CNS_Q_PT+0x38,r1)
	SAVE_IPR(pt8,CNS_Q_PT+0x40,r1)
	SAVE_IPR(pt9,CNS_Q_PT+0x48,r1)
	SAVE_IPR(pt10,CNS_Q_PT+0x50,r1)
	SAVE_IPR(pt11,CNS_Q_PT+0x58,r1)
	SAVE_IPR(pt12,CNS_Q_PT+0x60,r1)
	SAVE_IPR(pt13,CNS_Q_PT+0x68,r1)
	SAVE_IPR(pt14,CNS_Q_PT+0x70,r1)
	SAVE_IPR(pt15,CNS_Q_PT+0x78,r1)
	SAVE_IPR(pt16,CNS_Q_PT+0x80,r1)
	SAVE_IPR(pt17,CNS_Q_PT+0x88,r1)
	SAVE_IPR(pt18,CNS_Q_PT+0x90,r1)
	SAVE_IPR(pt19,CNS_Q_PT+0x98,r1)
	SAVE_IPR(pt20,CNS_Q_PT+0xA0,r1)
	SAVE_IPR(pt21,CNS_Q_PT+0xA8,r1)
	SAVE_IPR(pt22,CNS_Q_PT+0xB0,r1)
	SAVE_IPR(pt23,CNS_Q_PT+0xB8,r1)

	// Restore shadow mode
	mfpr	r31, pt0		// pad write to icsr out of shadow of store (trap does not abort write)	//orig
	mfpr	r31, pt0											//orig
	mtpr	r2, icsr		// Restore original ICSR						//orig

	mfpr	r31, pt0		// SDE bubble cycle 1							//orig
	mfpr	r31, pt0		// SDE bubble cycle 2							//orig
	mfpr	r31, pt0		// SDE bubble cycle 3							//orig
	nop													//orig

	// save all integer shadow regs
	
//orig #define t 8
//orig	.repeat 7
//orig	  store_reg \t,  shadow=1
//orig #define t t + 1
//orig	.endr
//orig	store_reg 25,  shadow=1

	SAVE_SHADOW( r8,CNS_Q_SHADOW+0x00,r1)	// also called p0...p7 in the Hudson code
	SAVE_SHADOW( r9,CNS_Q_SHADOW+0x08,r1)
	SAVE_SHADOW(r10,CNS_Q_SHADOW+0x10,r1)
	SAVE_SHADOW(r11,CNS_Q_SHADOW+0x18,r1)
	SAVE_SHADOW(r12,CNS_Q_SHADOW+0x20,r1)
	SAVE_SHADOW(r13,CNS_Q_SHADOW+0x28,r1)
	SAVE_SHADOW(r14,CNS_Q_SHADOW+0x30,r1)
	SAVE_SHADOW(r25,CNS_Q_SHADOW+0x38,r1)

//orig	store_reg exc_addr,	ipr=1	// save ipr
//orig	store_reg pal_base,	ipr=1	// save ipr
//orig	store_reg mm_stat,	ipr=1	// save ipr
//orig	store_reg va,		ipr=1	// save ipr
//orig	store_reg icsr,		ipr=1   // save ipr
//orig	store_reg ipl,		ipr=1	// save ipr
//orig	store_reg ps,		ipr=1	// save ipr
//orig	store_reg itb_asn,	ipr=1   // save ipr
//orig	store_reg aster,	ipr=1	// save ipr
//orig	store_reg astrr,	ipr=1	// save ipr
//orig	store_reg sirr,		ipr=1	// save ipr
//orig	store_reg isr,		ipr=1	// save ipr
//orig	store_reg ivptbr,	ipr=1	// save ipr
//orig	store_reg mcsr,		ipr=1	// save ipr
//orig	store_reg dc_mode,	ipr=1	// save ipr
	
	SAVE_IPR(excAddr,CNS_Q_EXC_ADDR,r1)
	SAVE_IPR(palBase,CNS_Q_PAL_BASE,r1)
	SAVE_IPR(mmStat,CNS_Q_MM_STAT,r1)
	SAVE_IPR(va,CNS_Q_VA,r1)
	SAVE_IPR(icsr,CNS_Q_ICSR,r1)
	SAVE_IPR(ipl,CNS_Q_IPL,r1)
	SAVE_IPR(ips,CNS_Q_IPS,r1)
	SAVE_IPR(itbAsn,CNS_Q_ITB_ASN,r1)
	SAVE_IPR(aster,CNS_Q_ASTER,r1)
	SAVE_IPR(astrr,CNS_Q_ASTRR,r1)
	SAVE_IPR(sirr,CNS_Q_SIRR,r1)
	SAVE_IPR(isr,CNS_Q_ISR,r1)
	SAVE_IPR(iVptBr,CNS_Q_IVPTBR,r1)
	SAVE_IPR(mcsr,CNS_Q_MCSR,r1)
	SAVE_IPR(dcMode,CNS_Q_DC_MODE,r1)
	
//orig	pvc_violate 379			// mf maf_mode after a store ok (pvc doesn't distinguish ld from st)
//orig	store_reg maf_mode,	ipr=1	// save ipr -- no mbox instructions for 
//orig                                  // PVC violation applies only to
pvc$osf35$379:				    // loads. HW_ST ok here, so ignore
	SAVE_IPR(mafMode,CNS_Q_MAF_MODE,r1) // MBOX INST->MF MAF_MODE IN 0,1,2
	
	
	//the following iprs are informational only -- will not be restored
	
//orig	store_reg icperr_stat,	ipr=1
//orig	store_reg pmctr,	ipr=1
//orig	store_reg intid,	ipr=1
//orig	store_reg exc_sum,	ipr=1
//orig	store_reg exc_mask,	ipr=1
//orig	ldah	r14, 0xfff0(r31)
//orig	zap	r14, 0xE0, r14			// Get Cbox IPR base
//orig	nop					// pad mf dcperr_stat out of shadow of last store
//orig	nop
//orig	nop
//orig	store_reg dcperr_stat,	ipr=1

	SAVE_IPR(icPerr,CNS_Q_ICPERR_STAT,r1)
	SAVE_IPR(PmCtr,CNS_Q_PM_CTR,r1)
	SAVE_IPR(intId,CNS_Q_INT_ID,r1)
	SAVE_IPR(excSum,CNS_Q_EXC_SUM,r1)
	SAVE_IPR(excMask,CNS_Q_EXC_MASK,r1)
	ldah	r14, 0xFFF0(zero)
	zap	r14, 0xE0, r14		// Get base address of CBOX IPRs
	NOP				// Pad mfpr dcPerr out of shadow of
	NOP				// last store
	NOP
	SAVE_IPR(dcPerr,CNS_Q_DCPERR_STAT,r1)
	
	// read cbox ipr state

//orig	mb
//orig	ldqp	r2, ev5__sc_ctl(r14)
//orig	ldqp	r13, ld_lock(r14)
//orig	ldqp	r4, ev5__sc_addr(r14)
//orig	ldqp	r5, ev5__ei_addr(r14)
//orig	ldqp	r6, ev5__bc_tag_addr(r14)
//orig	ldqp	r7, ev5__fill_syn(r14)
//orig	bis	r5, r4, r31
//orig	bis	r7, r6, r31		// make sure previous loads finish before reading stat registers which unlock them
//orig	ldqp	r8, ev5__sc_stat(r14)	// unlocks sc_stat,sc_addr
//orig	ldqp	r9, ev5__ei_stat(r14)	// may unlock ei_*, bc_tag_addr, fill_syn
//orig	ldqp	r31, ev5__ei_stat(r14)	// ensures it is really unlocked
//orig	mb

#ifndef SIMOS
	mb
	ldq_p	r2, scCtl(r14)
	ldq_p	r13, ldLock(r14)
	ldq_p	r4, scAddr(r14)
	ldq_p	r5, eiAddr(r14)
	ldq_p	r6, bcTagAddr(r14)
	ldq_p	r7, fillSyn(r14)
	bis	r5, r4, zero		// Make sure all loads complete before
	bis	r7, r6, zero		// reading registers that unlock them.
	ldq_p	r8, scStat(r14)		// Unlocks scAddr.
	ldq_p	r9, eiStat(r14)		// Unlocks eiAddr, bcTagAddr, fillSyn.
	ldq_p	zero, eiStat(r14)	// Make sure it is really unlocked.
	mb
#endif
//orig	// save cbox ipr state
//orig	store_reg1 sc_ctl, r2, r1, ipr=1	
//orig	store_reg1 ld_lock, r13, r1, ipr=1	
//orig	store_reg1 sc_addr, r4, r1, ipr=1	
//orig	store_reg1 ei_addr, r5, r1, ipr=1	
//orig	store_reg1 bc_tag_addr, r6, r1, ipr=1	
//orig	store_reg1 fill_syn, r7, r1, ipr=1	
//orig	store_reg1 sc_stat, r8, r1, ipr=1	
//orig	store_reg1 ei_stat, r9, r1, ipr=1	
//orig //bc_config? sl_rcv?
	
	SAVE_SHADOW(r2,CNS_Q_SC_CTL,r1);
	SAVE_SHADOW(r13,CNS_Q_LD_LOCK,r1);
	SAVE_SHADOW(r4,CNS_Q_SC_ADDR,r1);
	SAVE_SHADOW(r5,CNS_Q_EI_ADDR,r1); 
	SAVE_SHADOW(r6,CNS_Q_BC_TAG_ADDR,r1); 
	SAVE_SHADOW(r7,CNS_Q_FILL_SYN,r1); 
	SAVE_SHADOW(r8,CNS_Q_SC_STAT,r1); 
	SAVE_SHADOW(r9,CNS_Q_EI_STAT,r1);
	
// restore impure base								//orig
//orig	unfix_impure_ipr r1
	lda	r1, -CNS_Q_IPR(r1)
	
// save all floating regs							//orig
	mfpr	r0, icsr		// get icsr				//orig
	or	r31, 1, r2		// get a one				//orig
//orig	sll	r2, #icsr_v_fpe, r2	// shift for fpu spot			//orig
	sll	r2, icsr_v_fpe, r2	// Shift it into ICSR<FPE> position
	or	r2, r0, r0		// set FEN on				//orig
	mtpr	r0, icsr		// write to icsr, enabling FEN		//orig

// map the save area virtually
// orig	mtpr	r31, dtb_ia		// clear the dtb
// orig	srl	r1, page_offset_size_bits, r0 // Clean off low bits of VA
// orig	sll	r0, 32, r0		// shift to PFN field			
// orig	lda	r2, 0xff(r31)		// all read enable and write enable bits set
// orig	sll	r2, 8, r2		// move to PTE location	
// orig	addq	r0, r2, r0		// combine with PFN	
// orig	mtpr	r0, dtb_pte		// Load PTE and set TB valid bit
// orig	mtpr	r1, dtb_tag		// write TB tag			

	mtpr	r31, dtbIa		// Clear all DTB entries
	srl	r1, va_s_off, r0	// Clean off byte-within-page offset
	sll	r0, pte_v_pfn, r0	// Shift to form PFN
	lda	r0, pte_m_prot(r0)	// Set all read/write enable bits
	mtpr	r0, dtbPte		// Load the PTE and set valid
	mtpr	r1, dtbTag		// Write the PTE and tag into the DTB
	

//orig // map the next page too - in case the impure area crosses a page boundary
//orig	lda 	r4, 1@page_offset_size_bits(r1)	// generate address for next page
//orig	srl	r4, page_offset_size_bits, r0 // Clean off low bits of VA
//orig	sll	r0, 32, r0		// shift to PFN field			
//orig	lda	r2, 0xff(r31)		// all read enable and write enable bits set
//orig	sll	r2, 8, r2		// move to PTE location				
//orig	addq	r0, r2, r0		// combine with PFN				
//orig	mtpr	r0, dtb_pte		// Load PTE and set TB valid bit		
//orig	mtpr	r4, dtb_tag		// write TB tag					

	lda	r4, (1<<va_s_off)(r1)	// Generate address for next page
	srl	r4, va_s_off, r0	// Clean off byte-within-page offset
	sll	r0, pte_v_pfn, r0	// Shift to form PFN
	lda	r0, pte_m_prot(r0)	// Set all read/write enable bits
	mtpr	r0, dtbPte		// Load the PTE and set valid
	mtpr	r4, dtbTag		// Write the PTE and tag into the DTB
	
	sll	r31, 0, r31		// stall cycle 1				// orig
	sll	r31, 0, r31		// stall cycle 2				// orig
	sll	r31, 0, r31		// stall cycle 3				// orig
	nop										// orig

//orig // add offset for saving fpr regs
//orig	fix_impure_gpr r1

	lda	r1, 0x200(r1)		// Point to center of CPU segment
	
// now save the regs - F0-F31
	
//orig #define t 0
//orig	.repeat 32
//orig	  store_reg \t , fpu=1
//orig #define t t + 1
//orig	.endr
	
	mf_fpcr  f0			// original

	SAVE_FPR(f0,CNS_Q_FPR+0x00,r1)
	SAVE_FPR(f1,CNS_Q_FPR+0x08,r1)
	SAVE_FPR(f2,CNS_Q_FPR+0x10,r1)
	SAVE_FPR(f3,CNS_Q_FPR+0x18,r1)
	SAVE_FPR(f4,CNS_Q_FPR+0x20,r1)
	SAVE_FPR(f5,CNS_Q_FPR+0x28,r1)
	SAVE_FPR(f6,CNS_Q_FPR+0x30,r1)
	SAVE_FPR(f7,CNS_Q_FPR+0x38,r1)
	SAVE_FPR(f8,CNS_Q_FPR+0x40,r1)
	SAVE_FPR(f9,CNS_Q_FPR+0x48,r1)
	SAVE_FPR(f10,CNS_Q_FPR+0x50,r1)
	SAVE_FPR(f11,CNS_Q_FPR+0x58,r1)
	SAVE_FPR(f12,CNS_Q_FPR+0x60,r1)
	SAVE_FPR(f13,CNS_Q_FPR+0x68,r1)
	SAVE_FPR(f14,CNS_Q_FPR+0x70,r1)
	SAVE_FPR(f15,CNS_Q_FPR+0x78,r1)
	SAVE_FPR(f16,CNS_Q_FPR+0x80,r1)
	SAVE_FPR(f17,CNS_Q_FPR+0x88,r1)
	SAVE_FPR(f18,CNS_Q_FPR+0x90,r1)
	SAVE_FPR(f19,CNS_Q_FPR+0x98,r1)
	SAVE_FPR(f20,CNS_Q_FPR+0xA0,r1)
	SAVE_FPR(f21,CNS_Q_FPR+0xA8,r1)
	SAVE_FPR(f22,CNS_Q_FPR+0xB0,r1)
	SAVE_FPR(f23,CNS_Q_FPR+0xB8,r1)
	SAVE_FPR(f24,CNS_Q_FPR+0xC0,r1)
	SAVE_FPR(f25,CNS_Q_FPR+0xC8,r1)
	SAVE_FPR(f26,CNS_Q_FPR+0xD0,r1)
	SAVE_FPR(f27,CNS_Q_FPR+0xD8,r1)
	SAVE_FPR(f28,CNS_Q_FPR+0xE0,r1)
	SAVE_FPR(f29,CNS_Q_FPR+0xE8,r1)
	SAVE_FPR(f30,CNS_Q_FPR+0xF0,r1)
	SAVE_FPR(f31,CNS_Q_FPR+0xF8,r1)

//orig	//switch impure offset from gpr to ipr---
//orig	unfix_impure_gpr	r1
//orig	fix_impure_ipr	r1
//orig	store_reg1 fpcsr, f0, r1, fpcsr=1
	
	SAVE_FPR(f0,CNS_Q_FPCSR,r1)	// fpcsr loaded above into f0 -- can it reach// pb
	lda	r1, -0x200(r1)		// Restore the impure base address
	
//orig	// and back to gpr ---
//orig	unfix_impure_ipr	r1
//orig	fix_impure_gpr	r1

//orig	lda	r0, cns_mchksize(r31)	// get size of mchk area
//orig	store_reg1 mchkflag, r0, r1, ipr=1
//orig	mb

	lda	r1, CNS_Q_IPR(r1)	// Point to base of IPR area again
	// save this using the IPR base (it is closer) not the GRP base as they used...pb
	lda	r0, MACHINE_CHECK_SIZE(r31)	// get size of mchk area
	SAVE_SHADOW(r0,CNS_Q_MCHK,r1); 
	mb

//orig	or	r31, 1, r0		// get a one	
//orig	store_reg1 flag, r0, r1, ipr=1	// set dump area flag
//orig	mb
	
	lda	r1, -CNS_Q_IPR(r1)	// back to the base
	lda	r1, 0x200(r1)		// Point to center of CPU segment
	or	r31, 1, r0		// get a one
	SAVE_GPR(r0,CNS_Q_FLAG,r1)	// // set dump area valid flag
	mb
		
//orig	// restore impure area base
//orig	unfix_impure_gpr r1
	lda	r1, -0x200(r1)		// Point to center of CPU segment
	
	mtpr	r31, dtb_ia		// clear the dtb	//orig
	mtpr	r31, itb_ia		// clear the itb	//orig
	
//orig	pvc_jsr	savsta, bsr=1, dest=1
	ret	r31, (r3)		// and back we go
#endif


#if remove_restore_state == 0


// .sbttl  "PAL_RESTORE_STATE"
//+
//
//	Pal_restore_state
//
//
//	register usage:
//		r1 = addr of impure area
//		r3 = return_address
//		all other regs are scratchable, as they are about to
//		be reloaded from ram.
//
//	Function:
//		All chip state restored, all SRs, FRs, PTs, IPRs
//					*** except R1, R3, PT0, PT4, PT5 ***
//
//-
	ALIGN_BLOCK
pal_restore_state:

//need to restore sc_ctl,bc_ctl,bc_config??? if so, need to figure out a safe way to do so.

//orig	// map the console io area virtually
//orig	mtpr	r31, dtb_ia		// clear the dtb
//orig	srl	r1, page_offset_size_bits, r0 // Clean off low bits of VA
//orig	sll	r0, 32, r0		// shift to PFN field
//orig	lda	r2, 0xff(r31)		// all read enable and write enable bits set
//orig	sll	r2, 8, r2		// move to PTE location
//orig	addq	r0, r2, r0		// combine with PFN
//orig
//orig	mtpr	r0, dtb_pte		// Load PTE and set TB valid bit
//orig	mtpr	r1, dtb_tag		// write TB tag
//orig	

	mtpr	r31, dtbIa		// Clear all DTB entries
	srl	r1, va_s_off, r0	// Clean off byte-within-page offset
	sll	r0, pte_v_pfn, r0	// Shift to form PFN
	lda	r0, pte_m_prot(r0)	// Set all read/write enable bits
	mtpr	r0, dtbPte		// Load the PTE and set valid
	mtpr	r1, dtbTag		// Write the PTE and tag into the DTB


//orig	// map the next page too, in case impure area crosses page boundary
//orig	lda 	r4, 1@page_offset_size_bits(r1)	// generate address for next page
//orig	srl	r4, page_offset_size_bits, r0 // Clean off low bits of VA
//orig	sll	r0, 32, r0		// shift to PFN field
//orig	lda	r2, 0xff(r31)		// all read enable and write enable bits set
//orig	sll	r2, 8, r2		// move to PTE location
//orig	addq	r0, r2, r0		// combine with PFN
//orig
//orig	mtpr	r0, dtb_pte		// Load PTE and set TB valid bit
//orig	mtpr	r4, dtb_tag		// write TB tag - no virtual mbox instruction for 3 cycles
	
	lda	r4, (1<<VA_S_OFF)(r1)	// Generate address for next page
	srl	r4, va_s_off, r0	// Clean off byte-within-page offset
	sll	r0, pte_v_pfn, r0	// Shift to form PFN
	lda	r0, pte_m_prot(r0)	// Set all read/write enable bits
	mtpr	r0, dtbPte		// Load the PTE and set valid
	mtpr	r4, dtbTag		// Write the PTE and tag into the DTB

//orig	// save all floating regs
//orig	mfpr	r0, icsr		// get icsr
//orig// 	assume	ICSR_V_SDE gt <ICSR_V_FPE>		// assertion checker
//orig	or	r31, <<1@<ICSR_V_SDE-ICSR_V_FPE>> ! 1>, r2	// set SDE and FPE
//orig	sll	r2, #icsr_v_fpe, r2	// shift for fpu spot
//orig	or	r2, r0, r0		// set FEN on
//orig	mtpr	r0, icsr		// write to icsr, enabling FEN and SDE.  3 bubbles to floating instr.

	mfpr	r0, icsr		// Get current ICSR
	bis	zero, 1, r2		// Get a '1'
	or	r2, (1<<(icsr_v_sde-icsr_v_fpe)), r2
	sll	r2, icsr_v_fpe, r2	// Shift bits into position
	bis	r2, r2, r0		// Set ICSR<SDE> and ICSR<FPE>
	mtpr	r0, icsr		// Update the chip

	mfpr	r31, pt0		// FPE bubble cycle 1		//orig
	mfpr	r31, pt0		// FPE bubble cycle 2		//orig
	mfpr	r31, pt0		// FPE bubble cycle 3		//orig
	
//orig	fix_impure_ipr r1
//orig	restore_reg1 fpcsr, f0, r1, fpcsr=1
//orig	mt_fpcr  f0
//orig
//orig	unfix_impure_ipr r1
//orig	fix_impure_gpr r1		// adjust impure pointer offset for gpr access
//orig
//orig	// restore all floating regs
//orig#define t 0
//orig	.repeat 32
//orig	  restore_reg \t , fpu=1
//orig#define t t + 1
//orig	.endr

	lda	r1, 200(r1)	// Point to base of IPR area again
	RESTORE_FPR(f0,CNS_Q_FPCSR,r1)		// can it reach?? pb
	mt_fpcr  f0			// original

	lda	r1, 0x200(r1)		// point to center of CPU segment
	RESTORE_FPR(f0,CNS_Q_FPR+0x00,r1)
	RESTORE_FPR(f1,CNS_Q_FPR+0x08,r1)
	RESTORE_FPR(f2,CNS_Q_FPR+0x10,r1)
	RESTORE_FPR(f3,CNS_Q_FPR+0x18,r1)
	RESTORE_FPR(f4,CNS_Q_FPR+0x20,r1)
	RESTORE_FPR(f5,CNS_Q_FPR+0x28,r1)
	RESTORE_FPR(f6,CNS_Q_FPR+0x30,r1)
	RESTORE_FPR(f7,CNS_Q_FPR+0x38,r1)
	RESTORE_FPR(f8,CNS_Q_FPR+0x40,r1)
	RESTORE_FPR(f9,CNS_Q_FPR+0x48,r1)
	RESTORE_FPR(f10,CNS_Q_FPR+0x50,r1)
	RESTORE_FPR(f11,CNS_Q_FPR+0x58,r1)
	RESTORE_FPR(f12,CNS_Q_FPR+0x60,r1)
	RESTORE_FPR(f13,CNS_Q_FPR+0x68,r1)
	RESTORE_FPR(f14,CNS_Q_FPR+0x70,r1)
	RESTORE_FPR(f15,CNS_Q_FPR+0x78,r1)
	RESTORE_FPR(f16,CNS_Q_FPR+0x80,r1)
	RESTORE_FPR(f17,CNS_Q_FPR+0x88,r1)
	RESTORE_FPR(f18,CNS_Q_FPR+0x90,r1)
	RESTORE_FPR(f19,CNS_Q_FPR+0x98,r1)
	RESTORE_FPR(f20,CNS_Q_FPR+0xA0,r1)
	RESTORE_FPR(f21,CNS_Q_FPR+0xA8,r1)
	RESTORE_FPR(f22,CNS_Q_FPR+0xB0,r1)
	RESTORE_FPR(f23,CNS_Q_FPR+0xB8,r1)
	RESTORE_FPR(f24,CNS_Q_FPR+0xC0,r1)
	RESTORE_FPR(f25,CNS_Q_FPR+0xC8,r1)
	RESTORE_FPR(f26,CNS_Q_FPR+0xD0,r1)
	RESTORE_FPR(f27,CNS_Q_FPR+0xD8,r1)
	RESTORE_FPR(f28,CNS_Q_FPR+0xE0,r1)
	RESTORE_FPR(f29,CNS_Q_FPR+0xE8,r1)
	RESTORE_FPR(f30,CNS_Q_FPR+0xF0,r1)
	RESTORE_FPR(f31,CNS_Q_FPR+0xF8,r1)

//orig	// switch impure pointer from gpr to ipr area --
//orig	unfix_impure_gpr r1
//orig	fix_impure_ipr r1
//orig
//orig	// restore all pal regs
//orig#define t 1
//orig	.repeat 23
//orig	  restore_reg \t	, pal=1
//orig#define t t + 1
//orig	.endr

	lda	r1, -0x200(r1)		// Restore base address of impure area.
	lda	r1, CNS_Q_IPR(r1)	// Point to base of IPR area.
	RESTORE_IPR(pt0,CNS_Q_PT+0x00,r1)		// the osf code didn't save/restore palTemp 0 ?? pboyle
	RESTORE_IPR(pt1,CNS_Q_PT+0x08,r1)
	RESTORE_IPR(pt2,CNS_Q_PT+0x10,r1)
	RESTORE_IPR(pt3,CNS_Q_PT+0x18,r1)
	RESTORE_IPR(pt4,CNS_Q_PT+0x20,r1)
	RESTORE_IPR(pt5,CNS_Q_PT+0x28,r1)
	RESTORE_IPR(pt6,CNS_Q_PT+0x30,r1)
	RESTORE_IPR(pt7,CNS_Q_PT+0x38,r1)
	RESTORE_IPR(pt8,CNS_Q_PT+0x40,r1)
	RESTORE_IPR(pt9,CNS_Q_PT+0x48,r1)
	RESTORE_IPR(pt10,CNS_Q_PT+0x50,r1)
	RESTORE_IPR(pt11,CNS_Q_PT+0x58,r1)
	RESTORE_IPR(pt12,CNS_Q_PT+0x60,r1)
	RESTORE_IPR(pt13,CNS_Q_PT+0x68,r1)
	RESTORE_IPR(pt14,CNS_Q_PT+0x70,r1)
	RESTORE_IPR(pt15,CNS_Q_PT+0x78,r1)
	RESTORE_IPR(pt16,CNS_Q_PT+0x80,r1)
	RESTORE_IPR(pt17,CNS_Q_PT+0x88,r1)
	RESTORE_IPR(pt18,CNS_Q_PT+0x90,r1)
	RESTORE_IPR(pt19,CNS_Q_PT+0x98,r1)
	RESTORE_IPR(pt20,CNS_Q_PT+0xA0,r1)
	RESTORE_IPR(pt21,CNS_Q_PT+0xA8,r1)
	RESTORE_IPR(pt22,CNS_Q_PT+0xB0,r1)
	RESTORE_IPR(pt23,CNS_Q_PT+0xB8,r1)

	
//orig	restore_reg exc_addr,	ipr=1	// restore ipr
//orig	restore_reg pal_base,	ipr=1	// restore ipr
//orig	restore_reg ipl,	ipr=1	// restore ipr
//orig	restore_reg ps,		ipr=1	// restore ipr
//orig	mtpr	r0, dtb_cm		// set current mode in mbox too
//orig	restore_reg itb_asn,	ipr=1
//orig	srl	r0, itb_asn_v_asn, r0
//orig	sll	r0, dtb_asn_v_asn, r0
//orig	mtpr	r0, dtb_asn		// set ASN in Mbox too
//orig	restore_reg ivptbr,	ipr=1
//orig	mtpr	r0, mvptbr			// use ivptbr value to restore mvptbr
//orig	restore_reg mcsr,	ipr=1
//orig	restore_reg aster,	ipr=1
//orig	restore_reg astrr,	ipr=1
//orig	restore_reg sirr,	ipr=1
//orig	restore_reg maf_mode, 	ipr=1		// no mbox instruction for 3 cycles
//orig	mfpr	r31, pt0			// (may issue with mt maf_mode)
//orig	mfpr	r31, pt0			// bubble cycle 1
//orig	mfpr	r31, pt0                        // bubble cycle 2
//orig	mfpr	r31, pt0                        // bubble cycle 3
//orig	mfpr	r31, pt0			// (may issue with following ld)

	// r0 gets the value of RESTORE_IPR in the macro and this code uses this side effect (gag)
	RESTORE_IPR(excAddr,CNS_Q_EXC_ADDR,r1)
	RESTORE_IPR(palBase,CNS_Q_PAL_BASE,r1)
	RESTORE_IPR(ipl,CNS_Q_IPL,r1)
	RESTORE_IPR(ips,CNS_Q_IPS,r1)
	mtpr	r0, dtbCm			// Set Mbox current mode too. 
	RESTORE_IPR(itbAsn,CNS_Q_ITB_ASN,r1)
	srl	r0, 4, r0
	sll	r0, 57, r0
	mtpr	r0, dtbAsn			// Set Mbox ASN too
	RESTORE_IPR(iVptBr,CNS_Q_IVPTBR,r1)
	mtpr	r0, mVptBr			// Set Mbox VptBr too
	RESTORE_IPR(mcsr,CNS_Q_MCSR,r1)
	RESTORE_IPR(aster,CNS_Q_ASTER,r1)
	RESTORE_IPR(astrr,CNS_Q_ASTRR,r1)
	RESTORE_IPR(sirr,CNS_Q_SIRR,r1)
	RESTORE_IPR(mafMode,CNS_Q_MAF_MODE,r1)
	STALL
	STALL
	STALL
	STALL
	STALL


	// restore all integer shadow regs
//orig#define t 8
//orig	.repeat 7
//orig	  restore_reg \t, shadow=1
//orig#define t t + 1
//orig	.endr
//orig	restore_reg 25, shadow=1
//orig	restore_reg dc_mode, 	ipr=1		// no mbox instructions for 4 cycles

	RESTORE_SHADOW( r8,CNS_Q_SHADOW+0x00,r1)	// also called p0...p7 in the Hudson code
	RESTORE_SHADOW( r9,CNS_Q_SHADOW+0x08,r1)
	RESTORE_SHADOW(r10,CNS_Q_SHADOW+0x10,r1)
	RESTORE_SHADOW(r11,CNS_Q_SHADOW+0x18,r1)
	RESTORE_SHADOW(r12,CNS_Q_SHADOW+0x20,r1)
	RESTORE_SHADOW(r13,CNS_Q_SHADOW+0x28,r1)
	RESTORE_SHADOW(r14,CNS_Q_SHADOW+0x30,r1)
	RESTORE_SHADOW(r25,CNS_Q_SHADOW+0x38,r1)
	RESTORE_IPR(dcMode,CNS_Q_DC_MODE,r1)
	
	//
	// Get out of shadow mode
	//

	mfpr	r31, pt0		// pad last load to icsr write (in case of replay, icsr will be written anyway)	//orig
	mfpr	r31, pt0		// ""										//orig
	mfpr	r0, icsr		// Get icsr									//orig
//orig	ldah	r2,  <1@<icsr_v_sde-16>>(r31)	// Get a one in SHADOW_ENABLE bit location
	ldah	r2,  (1<<(ICSR_V_SDE-16))(r31)	// Get a one in SHADOW_ENABLE bit location				//orig
	bic	r0, r2, r2		// ICSR with SDE clear								//orig
	mtpr	r2, icsr		// Turn off SDE - no palshadow rd/wr for 3 bubble cycles			//orig
					
	mfpr	r31, pt0		// SDE bubble cycle 1								//orig
	mfpr	r31, pt0		// SDE bubble cycle 2								//orig
	mfpr	r31, pt0		// SDE bubble cycle 3								//orig
	nop														//orig

//orig	// switch impure pointer from ipr to gpr area --
//orig	unfix_impure_ipr	r1
//orig	fix_impure_gpr	r1
//orig	// restore all integer regs
//orig#define t 4
//orig	.repeat 28
//orig	  restore_reg \t
//orig#define t t + 1
//orig	.endr

// Restore GPRs (r0, r2 are restored later, r1 and r3 are trashed) ...

	lda	r1, -CNS_Q_IPR(r1)	// Restore base address of impure area
	lda	r1, 0x200(r1)		// Point to center of CPU segment

	RESTORE_GPR(r4,CNS_Q_GPR+0x20,r1)
	RESTORE_GPR(r5,CNS_Q_GPR+0x28,r1)
	RESTORE_GPR(r6,CNS_Q_GPR+0x30,r1)
	RESTORE_GPR(r7,CNS_Q_GPR+0x38,r1)
	RESTORE_GPR(r8,CNS_Q_GPR+0x40,r1)
	RESTORE_GPR(r9,CNS_Q_GPR+0x48,r1)
	RESTORE_GPR(r10,CNS_Q_GPR+0x50,r1)
	RESTORE_GPR(r11,CNS_Q_GPR+0x58,r1)
	RESTORE_GPR(r12,CNS_Q_GPR+0x60,r1)
	RESTORE_GPR(r13,CNS_Q_GPR+0x68,r1)
	RESTORE_GPR(r14,CNS_Q_GPR+0x70,r1)
	RESTORE_GPR(r15,CNS_Q_GPR+0x78,r1)
	RESTORE_GPR(r16,CNS_Q_GPR+0x80,r1)
	RESTORE_GPR(r17,CNS_Q_GPR+0x88,r1)
	RESTORE_GPR(r18,CNS_Q_GPR+0x90,r1)
	RESTORE_GPR(r19,CNS_Q_GPR+0x98,r1)
	RESTORE_GPR(r20,CNS_Q_GPR+0xA0,r1)
	RESTORE_GPR(r21,CNS_Q_GPR+0xA8,r1)
	RESTORE_GPR(r22,CNS_Q_GPR+0xB0,r1)
	RESTORE_GPR(r23,CNS_Q_GPR+0xB8,r1)
	RESTORE_GPR(r24,CNS_Q_GPR+0xC0,r1)
	RESTORE_GPR(r25,CNS_Q_GPR+0xC8,r1)
	RESTORE_GPR(r26,CNS_Q_GPR+0xD0,r1)
	RESTORE_GPR(r27,CNS_Q_GPR+0xD8,r1)
	RESTORE_GPR(r28,CNS_Q_GPR+0xE0,r1)
	RESTORE_GPR(r29,CNS_Q_GPR+0xE8,r1)
	RESTORE_GPR(r30,CNS_Q_GPR+0xF0,r1)
	RESTORE_GPR(r31,CNS_Q_GPR+0xF8,r1)
	
//orig	// switch impure pointer from gpr to ipr area --
//orig	unfix_impure_gpr	r1
//orig	fix_impure_ipr	r1
//orig	restore_reg icsr, ipr=1		// restore original icsr- 4 bubbles to hw_rei

	lda	t0, -0x200(t0)		// Restore base address of impure area.
	lda	t0, CNS_Q_IPR(t0)	// Point to base of IPR area again.
	RESTORE_IPR(icsr,CNS_Q_ICSR,r1)	

//orig	// and back again --
//orig	unfix_impure_ipr	r1
//orig	fix_impure_gpr	r1
//orig	store_reg1 	flag, r31, r1, ipr=1 // clear dump area valid flag
//orig	mb

	lda	t0, -CNS_Q_IPR(t0)	// Back to base of impure area again,
	lda	t0, 0x200(t0)		// and back to center of CPU segment
	SAVE_GPR(r31,CNS_Q_FLAG,r1)	// Clear the dump area valid flag
	mb
	
//orig	// and back we go
//orig//	restore_reg 3
//orig	restore_reg 2
//orig//	restore_reg 1
//orig	restore_reg 0
//orig	// restore impure area base
//orig	unfix_impure_gpr r1

	RESTORE_GPR(r2,CNS_Q_GPR+0x10,r1)
	RESTORE_GPR(r0,CNS_Q_GPR+0x00,r1)
	lda	r1, -0x200(r1)		// Restore impure base address
	
	mfpr	r31, pt0		// stall for ldqp above		//orig

	mtpr	r31, dtb_ia		// clear the tb			//orig
	mtpr	r31, itb_ia		// clear the itb		//orig

//orig	pvc_jsr	rststa, bsr=1, dest=1
	ret	r31, (r3)		// back we go			//orig
#endif


//+
// pal_pal_bug_check -- code has found a bugcheck situation.
//	Set things up and join common machine check flow.
//
// Input:
//	r14 	- exc_addr
//
// On exit:
//	pt0	- saved r0
//	pt1	- saved	r1
//	pt4	- saved r4
//	pt5	- saved r5
//	pt6	- saved r6
//	pt10	- saved exc_addr
//       pt_misc<47:32> - mchk code
//       pt_misc<31:16> - scb vector
//	r14	- base of Cbox IPRs in IO space
//	MCES<mchk> is set
//-
	
		ALIGN_BLOCK
	.globl pal_pal_bug_check_from_int
pal_pal_bug_check_from_int:
	DEBUGSTORE(0x79)
//simos	DEBUG_EXC_ADDR()
	DEBUGSTORE(0x20)
//simos	bsr	r25, put_hex
	lda	r25, mchk_c_bugcheck(r31)
	addq	r25, 1, r25			// set flag indicating we came from interrupt and stack is already pushed
	br	r31, pal_pal_mchk
	nop

pal_pal_bug_check:
        lda     r25, mchk_c_bugcheck(r31)

pal_pal_mchk:
	sll	r25, 32, r25			// Move mchk code to position

	mtpr	r14, pt10			// Stash exc_addr
	mtpr	r14, exc_addr

	mfpr	r12, pt_misc			// Get MCES and scratch
	zap	r12, 0x3c, r12

	or	r12, r25, r12			// Combine mchk code 
	lda	r25, scb_v_procmchk(r31)	// Get SCB vector

	sll	r25, 16, r25			// Move SCBv to position
	or	r12, r25, r25			// Combine SCBv

	mtpr	r0, pt0				// Stash for scratch
	bis	r25, mces_m_mchk, r25	// Set MCES<MCHK> bit

	mtpr	r25, pt_misc			// Save mchk code!scbv!whami!mces
	ldah	r14, 0xfff0(r31)

	mtpr	r1, pt1				// Stash for scratch
	zap	r14, 0xE0, r14			// Get Cbox IPR base

	mtpr	r4, pt4
	mtpr	r5, pt5

	mtpr	r6, pt6
	blbs	r12, sys_double_machine_check   // MCHK halt if double machine check

	br	r31, sys_mchk_collect_iprs	// Join common machine check flow

//	align_to_call_pal_section	// Align to address of first call_pal entry point - 2000

// .sbttl	"HALT	- PALcode for HALT instruction"

//+
// 
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//
// Function:
//	GO to console code
//
//-

 	.text	1
//	. = 0x2000
       CALL_PAL_PRIV(PAL_HALT_ENTRY)
call_pal_halt:
#if rax_mode == 0
	mfpr	r31, pt0		// Pad exc_addr read
	mfpr	r31, pt0

	mfpr	r12, exc_addr		// get PC
	subq	r12, 4, r12		// Point to the HALT

	mtpr	r12, exc_addr
	mtpr	r0, pt0

//orig	pvc_jsr updpcb, bsr=1
        bsr    r0, pal_update_pcb      	// update the pcb
        lda    r0, hlt_c_sw_halt(r31)  	// set halt code to sw halt
        br     r31, sys_enter_console  	// enter the console

#else					// RAX mode
        mb
        mb
	mtpr	r9, ev5__dtb_asn	// no Dstream virtual ref for next 3 cycles.
	mtpr	r9, ev5__itb_asn	// E1.  Update ITB ASN.  No hw_rei for 5 cycles. 
        mtpr    r8, exc_addr		// no HW_REI for 1 cycle.
	blbc	r9, not_begin_case
        mtpr    r31, ev5__dtb_ia        // clear DTB. No Dstream virtual ref for 2 cycles.
        mtpr    r31, ev5__itb_ia        // clear ITB.

not_begin_case:
	nop
	nop

	nop
	nop				// pad mt itb_asn ->hw_rei_stall

        hw_rei_stall
#endif

// .sbttl	"CFLUSH- PALcode for CFLUSH instruction"

//+
// 
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//
//	R16 - contains the PFN of the page to be flushed
//
// Function:
//	Flush all Dstream caches of 1 entire page
//	The CFLUSH routine is in the system specific module.
//
//-

        CALL_PAL_PRIV(PAL_CFLUSH_ENTRY)
Call_Pal_Cflush:
	br	r31, sys_cflush

// .sbttl	"DRAINA	- PALcode for DRAINA instruction"
//+
// 
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//	Implicit TRAPB performed by hardware.
//
// Function:
//	Stall instruction issue until all prior instructions are guaranteed to 
//	complete without incurring aborts.  For the EV5 implementation, this
//	means waiting until all pending DREADS are returned.
//
//-

        CALL_PAL_PRIV(PAL_DRAINA_ENTRY)
Call_Pal_Draina:
	ldah	r14, 0x100(r31)		// Init counter.  Value?
	nop
	
DRAINA_LOOP:
	subq	r14, 1, r14		// Decrement counter
	mfpr	r13, ev5__maf_mode	// Fetch status bit

	srl	r13, maf_mode_v_dread_pending, r13
	ble	r14, DRAINA_LOOP_TOO_LONG

	nop
	blbs	r13, DRAINA_LOOP	// Wait until all DREADS clear
	
	hw_rei

DRAINA_LOOP_TOO_LONG:
	br	r31, call_pal_halt

// .sbttl	"CALL_PAL OPCDECs"

	CALL_PAL_PRIV(0x0003)
CallPal_OpcDec03:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x0004)
CallPal_OpcDec04:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x0005)
CallPal_OpcDec05:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x0006)
CallPal_OpcDec06:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x0007)
CallPal_OpcDec07:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x0008)
CallPal_OpcDec08:
	br	r31, osfpal_calpal_opcdec

// .sbttl	"CSERVE- PALcode for CSERVE instruction"
//+
// 
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//
// Function:
//       Various functions for private use of console software
//
//       option selector in r0
//       arguments in r16....
//	The CSERVE routine is in the system specific module.
//
//-

        CALL_PAL_PRIV(PAL_CSERVE_ENTRY)
Call_Pal_Cserve:
	br	r31, sys_cserve

// .sbttl	"swppal - PALcode for swppal instruction"

//+
// 
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//       Vectored into via hardware PALcode instruction dispatch.
//               R16 contains the new PAL identifier
//               R17:R21 contain implementation-specific entry parameters
//
//               R0  receives status:
//                0 success (PAL was switched)
//                1 unknown PAL variant
//                2 known PAL variant, but PAL not loaded
//
//
// Function:
//       Swap control to another PAL.
//-

        CALL_PAL_PRIV(PAL_SWPPAL_ENTRY)
Call_Pal_Swppal:
	cmpule	r16, 255, r0		// see if a kibble was passed
        cmoveq  r16, r16, r0            // if r16=0 then a valid address (ECO 59)

	or	r16, r31, r3		// set r3 incase this is a address
	blbc	r0, swppal_cont		// nope, try it as an address

	cmpeq	r16, 2, r0		// is it our friend OSF?
	blbc	r0, swppal_fail		// nope, don't know this fellow

	br	r2, CALL_PAL_SWPPAL_10_			// tis our buddy OSF

//	.global	osfpal_hw_entry_reset
//	.weak	osfpal_hw_entry_reset
//	.long	<osfpal_hw_entry_reset-pal_start>
//orig	halt				// don't know how to get the address here - kludge ok, load pal at 0
	.long	0			// ?? hack upon hack...pb
	
CALL_PAL_SWPPAL_10_: 	ldlp	r3, 0(r2)		// fetch target addr
//	ble	r3, swppal_fail		; if OSF not linked in say not loaded.
	mfpr	r2, pal_base		// fetch pal base

	addq	r2, r3, r3		// add pal base
	lda	r2, 0x3FFF(r31)		// get pal base checker mask

	and	r3, r2, r2		// any funky bits set?
	cmpeq	r2, 0, r0		// 

	blbc	r0, swppal_fail		// return unknown if bad bit set.
	br	r31, swppal_cont

// .sbttl	"CALL_PAL OPCDECs"

	CALL_PAL_PRIV(0x000B)
CallPal_OpcDec0B:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x000C)
CallPal_OpcDec0C:
	br	r31, osfpal_calpal_opcdec

// .sbttl	"wripir- PALcode for wripir instruction"
//+
// 
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//	r16 = processor number to interrupt
//
// Function:
//	IPIR	<- R16
//	Handled in system-specific code
//
// Exit:
//	interprocessor interrupt is recorded on the target processor
//	and is initiated when the proper enabling conditions are present.
//-

        CALL_PAL_PRIV(PAL_WRIPIR_ENTRY)
Call_Pal_Wrpir:
	br	r31, sys_wripir

// .sbttl	"CALL_PAL OPCDECs"

	CALL_PAL_PRIV(0x000E)
CallPal_OpcDec0E:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x000F)
CallPal_OpcDec0F:
	br	r31, osfpal_calpal_opcdec

// .sbttl	"rdmces- PALcode for rdmces instruction"

//+
// 
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//
// Function:
//	R0 <- ZEXT(MCES)
//-

        CALL_PAL_PRIV(PAL_RDMCES_ENTRY)
Call_Pal_Rdmces:
	mfpr	r0, pt_mces		// Read from PALtemp
	and	r0, mces_m_all, r0	// Clear other bits
	
	hw_rei

// .sbttl	"wrmces- PALcode for wrmces instruction"

//+
// 
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//
// Function:
//	If {R16<0> EQ 1} then MCES<0> <- 0 (MCHK)
//	If {R16<1> EQ 1} then MCES<1> <- 0 (SCE)
//	If {R16<2> EQ 1} then MCES<2> <- 0 (PCE)
//	MCES<3> <- R16<3>		   (DPC)
//	MCES<4> <- R16<4>		   (DSC)
//
//-

        CALL_PAL_PRIV(PAL_WRMCES_ENTRY)
Call_Pal_Wrmces:
	and	r16, ((1<<mces_v_mchk) | (1<<mces_v_sce) | (1<<mces_v_pce)), r13	// Isolate MCHK, SCE, PCE
	mfpr	r14, pt_mces		// Get current value

	ornot	r31, r13, r13		// Flip all the bits
	and	r16, ((1<<mces_v_dpc) | (1<<mces_v_dsc)), r17

	and	r14, r13, r1		// Update MCHK, SCE, PCE
	bic	r1, ((1<<mces_v_dpc) | (1<<mces_v_dsc)), r1	// Clear old DPC, DSC

	or	r1, r17, r1		// Update DPC and DSC
	mtpr	r1, pt_mces		// Write MCES back

#if rawhide_system == 0
	nop				// Pad to fix PT write->read restriction
#else
	blbs	r16, RAWHIDE_clear_mchk_lock	// Clear logout from lock
#endif

	nop
	hw_rei



// .sbttl	"CALL_PAL OPCDECs"

	CALL_PAL_PRIV(0x0012)
CallPal_OpcDec12:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x0013)
CallPal_OpcDec13:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x0014)
CallPal_OpcDec14:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x0015)
CallPal_OpcDec15:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x0016)
CallPal_OpcDec16:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x0017)
CallPal_OpcDec17:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x0018)
CallPal_OpcDec18:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x0019)
CallPal_OpcDec19:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x001A)
CallPal_OpcDec1A:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x001B)
CallPal_OpcDec1B:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x001C)
CallPal_OpcDec1C:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x001D)
CallPal_OpcDec1D:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x001E)
CallPal_OpcDec1E:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x001F)
CallPal_OpcDec1F:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x0020)
CallPal_OpcDec20:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x0021)
CallPal_OpcDec21:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x0022)
CallPal_OpcDec22:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x0023)
CallPal_OpcDec23:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x0024)
CallPal_OpcDec24:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x0025)
CallPal_OpcDec25:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x0026)
CallPal_OpcDec26:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x0027)
CallPal_OpcDec27:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x0028)
CallPal_OpcDec28:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x0029)
CallPal_OpcDec29:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x002A)
CallPal_OpcDec2A:
	br	r31, osfpal_calpal_opcdec

// .sbttl	"wrfen - PALcode for wrfen instruction"

//+
// 
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//
// Function:
//	a0<0> -> ICSR<FPE>
//	Store new FEN in PCB
//	Final value of t0 (r1), t8..t10 (r22..r24) and a0 (r16) are UNPREDICTABLE
//
// Issue: What about pending FP loads when FEN goes from on->off????
//-

        CALL_PAL_PRIV(PAL_WRFEN_ENTRY)
Call_Pal_Wrfen:
	or	r31, 1, r13		// Get a one
	mfpr	r1, ev5__icsr		// Get current FPE

	sll	r13, icsr_v_fpe, r13	// shift 1 to icsr<fpe> spot, e0 
	and	r16, 1, r16		// clean new fen

	sll	r16, icsr_v_fpe, r12	// shift new fen to correct bit position
	bic	r1, r13, r1		// zero icsr<fpe>

	or	r1, r12, r1		// Or new FEN into ICSR
	mfpr	r12, pt_pcbb		// Get PCBB - E1 

	mtpr	r1, ev5__icsr		// write new ICSR.  3 Bubble cycles to HW_REI
	stlp	r16, osfpcb_q_fen(r12)	// Store FEN in PCB.

	mfpr	r31, pt0		// Pad ICSR<FPE> write.
	mfpr	r31, pt0

	mfpr	r31, pt0
//	pvc_violate 	225		// cuz PVC can't distinguish which bits changed
	hw_rei


	CALL_PAL_PRIV(0x002C)
CallPal_OpcDec2C:
	br	r31, osfpal_calpal_opcdec

// .sbttl	"wrvptpr - PALcode for wrvptpr instruction"
//+
// 
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//
// Function:
//	vptptr <- a0 (r16)
//-

        CALL_PAL_PRIV(PAL_WRVPTPTR_ENTRY)
Call_Pal_Wrvptptr:
        mtpr    r16, ev5__mvptbr                // Load Mbox copy 
        mtpr    r16, ev5__ivptbr                // Load Ibox copy
        nop                                     // Pad IPR write
        nop
        hw_rei

	CALL_PAL_PRIV(0x002E)
CallPal_OpcDec2E:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_PRIV(0x002F)
CallPal_OpcDec2F:
	br	r31, osfpal_calpal_opcdec

// .sbttl	"swpctx- PALcode for swpctx instruction"

//+
// 
// Entry:
//       hardware dispatch via callPal instruction
//       R16 -> new pcb
//
// Function:
//       dynamic state moved to old pcb
//       new state loaded from new pcb
//       pcbb pointer set
//       old pcbb returned in R0
//
//  Note: need to add perf monitor stuff
//-

        CALL_PAL_PRIV(PAL_SWPCTX_ENTRY)
Call_Pal_Swpctx:
	rpcc	r13			// get cyccounter
	mfpr	r0, pt_pcbb		// get pcbb

	ldqp	r22, osfpcb_q_fen(r16)	// get new fen/pme
	ldqp	r23, osfpcb_l_cc(r16)	// get new asn

	srl	r13, 32, r25		// move offset
	mfpr	r24, pt_usp		// get usp

	stqp	r30, osfpcb_q_ksp(r0)	// store old ksp
//	pvc_violate 379			// stqp can't trap except replay.  only problem if mf same ipr in same shadow.
	mtpr	r16, pt_pcbb		// set new pcbb

	stqp	r24, osfpcb_q_usp(r0)	// store usp
 	addl	r13, r25, r25		// merge for new time

  	stlp	r25, osfpcb_l_cc(r0)	// save time
	ldah	r24, (1<<(icsr_v_fpe-16))(r31)

	and	r22, 1, r12		// isolate fen
	mfpr	r25, icsr		// get current icsr

	ev5_pass2 	lda	r24, (1<<icsr_v_pmp)(r24)
	br	r31, swpctx_cont

// .sbttl	"wrval - PALcode for wrval instruction"
//+
// 
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//
// Function:
//	sysvalue <- a0 (r16)
//-

        CALL_PAL_PRIV(PAL_WRVAL_ENTRY)
Call_Pal_Wrval:
	nop
	mtpr	r16, pt_sysval		// Pad paltemp write
	nop
	nop
	hw_rei


// .sbttl	"rdval - PALcode for rdval instruction"

//+
// 
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//
// Function:
//	v0 (r0) <- sysvalue
//-

        CALL_PAL_PRIV(PAL_RDVAL_ENTRY)
Call_Pal_Rdval:
	nop
	mfpr	r0, pt_sysval
	nop
	hw_rei

// .sbttl	"tbi - PALcode for tbi instruction"
//+
// 
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//
// Function:
//	TB invalidate
//       r16/a0 = TBI type
//       r17/a1 = Va for TBISx instructions
//-

        CALL_PAL_PRIV(PAL_TBI_ENTRY)
Call_Pal_Tbi:
	addq	r16, 2, r16			// change range to 0-2
	br	r23, CALL_PAL_tbi_10_		// get our address

CALL_PAL_tbi_10_: cmpult	r16, 6, r22		// see if in range
	lda	r23, tbi_tbl-CALL_PAL_tbi_10_(r23)	// set base to start of table
	sll	r16, 4, r16		// * 16
	blbc	r22, CALL_PAL_tbi_30_		// go rei, if not

	addq	r23, r16, r23		// addr of our code
//orig	pvc_jsr	tbi
	jmp	r31, (r23)		// and go do it

CALL_PAL_tbi_30_:
	hw_rei
	nop

// .sbttl	"wrent - PALcode for wrent instruction"
//+
// 
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//
// Function:
//	Update ent* in paltemps
//       r16/a0 = Address of entry routine
//       r17/a1 = Entry Number 0..5
//
//       r22, r23 trashed
//-

        CALL_PAL_PRIV(PAL_WRENT_ENTRY)
Call_Pal_Wrent:
	cmpult	r17, 6, r22			// see if in range
	br	r23, CALL_PAL_wrent_10_		// get our address

CALL_PAL_wrent_10_:	bic	r16, 3, r16	// clean pc
	blbc	r22, CALL_PAL_wrent_30_		// go rei, if not in range

	lda	r23, wrent_tbl-CALL_PAL_wrent_10_(r23)	// set base to start of table
	sll	r17, 4, r17				// *16
	
	addq  	r17, r23, r23		// Get address in table
//orig	pvc_jsr	wrent
	jmp	r31, (r23)		// and go do it

CALL_PAL_wrent_30_:
	hw_rei				// out of range, just return

// .sbttl	"swpipl - PALcode for swpipl instruction"
//+
// 
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//
// Function:
//	v0 (r0)  <- PS<IPL>
//	PS<IPL>  <- a0<2:0>  (r16)
//
//	t8 (r22) is scratch
//-

        CALL_PAL_PRIV(PAL_SWPIPL_ENTRY)
Call_Pal_Swpipl:
	and	r16, osfps_m_ipl, r16	// clean New ipl
	mfpr	r22, pt_intmask		// get int mask

	extbl	r22, r16, r22		// get mask for this ipl
	bis	r11, r31, r0		// return old ipl

	bis	r16, r31, r11		// set new ps
	mtpr	r22, ev5__ipl		// set new mask

	mfpr	r31, pt0		// pad ipl write
	mfpr	r31, pt0		// pad ipl write

	hw_rei				// back

// .sbttl	"rdps - PALcode for rdps instruction"
//+
// 
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//
// Function:
//	v0 (r0) <- ps
//-

        CALL_PAL_PRIV(PAL_RDPS_ENTRY)
Call_Pal_Rdps:
	bis	r11, r31, r0		// Fetch PALshadow PS
	nop				// Must be 2 cycles long
	hw_rei

// .sbttl	"wrkgp - PALcode for wrkgp instruction"
//+
// 
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//
// Function:
//	kgp <- a0 (r16)
//-

        CALL_PAL_PRIV(PAL_WRKGP_ENTRY)
Call_Pal_Wrkgp:
	nop
	mtpr	r16, pt_kgp
	nop				// Pad for pt write->read restriction
	nop
	hw_rei

// .sbttl	"wrusp - PALcode for wrusp instruction"
//+
// 
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//
// Function:
//       usp <- a0 (r16)
//-

        CALL_PAL_PRIV(PAL_WRUSP_ENTRY)
Call_Pal_Wrusp:
	nop
	mtpr	r16, pt_usp
	nop				// Pad possible pt write->read restriction
	nop
	hw_rei

// .sbttl	"wrperfmon - PALcode for wrperfmon instruction"
//+
// 
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//
//
// Function:
//	Various control functions for the onchip performance counters
//
//	option selector in r16
//	option argument in r17
//	returned status in r0
//
//
//	r16 = 0	Disable performance monitoring for one or more cpu's
//	  r17 = 0		disable no counters
//	  r17 = bitmask		disable counters specified in bit mask (1=disable)
//
//	r16 = 1	Enable performance monitoring for one or more cpu's
//	  r17 = 0		enable no counters
//	  r17 = bitmask		enable counters specified in bit mask (1=enable)
//
//	r16 = 2	Mux select for one or more cpu's
//	  r17 = Mux selection (cpu specific)
//    		<24:19>  	 bc_ctl<pm_mux_sel> field (see spec)
//		<31>,<7:4>,<3:0> pmctr <sel0>,<sel1>,<sel2> fields (see spec)
//
//	r16 = 3	Options
//	  r17 = (cpu specific)
//		<0> = 0 	log all processes
//		<0> = 1		log only selected processes
//		<30,9,8> 		mode select - ku,kp,kk
//
//	r16 = 4	Interrupt frequency select
//	  r17 = (cpu specific)	indicates interrupt frequencies desired for each
//				counter, with "zero interrupts" being an option
//				frequency info in r17 bits as defined by PMCTR_CTL<FRQx> below
//
//	r16 = 5	Read Counters
//	  r17 = na
//	  r0  = value (same format as ev5 pmctr)
//	        <0> = 0		Read failed
//	        <0> = 1		Read succeeded
//
//	r16 = 6	Write Counters
//	  r17 = value (same format as ev5 pmctr; all counters written simultaneously)
//
//	r16 = 7	Enable performance monitoring for one or more cpu's and reset counter to 0
//	  r17 = 0		enable no counters
//	  r17 = bitmask		enable & clear counters specified in bit mask (1=enable & clear)
// 
//=============================================================================
//Assumptions:
//PMCTR_CTL:
//
//       <15:14>         CTL0 -- encoded frequency select and enable - CTR0
//       <13:12>         CTL1 --			"		   - CTR1
//       <11:10>         CTL2 --			"		   - CTR2
//
//       <9:8>           FRQ0 -- frequency select for CTR0 (no enable info)
//       <7:6>           FRQ1 -- frequency select for CTR1
//       <5:4>           FRQ2 -- frequency select for CTR2
//
//       <0>		all vs. select processes (0=all,1=select)
//
//     where
//	FRQx<1:0>
//	     0 1	disable interrupt  
//	     1 0	frequency = 65536 (16384 for ctr2)
//	     1 1	frequency = 256
//	note:  FRQx<1:0> = 00 will keep counters from ever being enabled.
//
//=============================================================================
//
        CALL_PAL_PRIV(0x0039)
// unsupported in Hudson code .. pboyle Nov/95
CALL_PAL_Wrperfmon:
#if perfmon_debug == 0
	// "real" performance monitoring code
	cmpeq	r16, 1, r0		// check for enable
	bne	r0, perfmon_en		// br if requested to enable

	cmpeq	r16, 2, r0		// check for mux ctl
	bne	r0, perfmon_muxctl	// br if request to set mux controls

	cmpeq	r16, 3, r0		// check for options
	bne	r0, perfmon_ctl		// br if request to set options

	cmpeq	r16, 4, r0		// check for interrupt frequency select
	bne	r0, perfmon_freq	// br if request to change frequency select

	cmpeq	r16, 5, r0		// check for counter read request
	bne	r0, perfmon_rd		// br if request to read counters

	cmpeq	r16, 6, r0		// check for counter write request
	bne	r0, perfmon_wr		// br if request to write counters

	cmpeq	r16, 7, r0		// check for counter clear/enable request
	bne	r0, perfmon_enclr	// br if request to clear/enable counters

	beq	r16, perfmon_dis	// br if requested to disable (r16=0)
	br	r31, perfmon_unknown	// br if unknown request
#else

	br	r31, pal_perfmon_debug
#endif

// .sbttl	"rdusp - PALcode for rdusp instruction"
//+
// 
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//
// Function:
//	v0 (r0) <- usp
//-

        CALL_PAL_PRIV(PAL_RDUSP_ENTRY)
Call_Pal_Rdusp:
	nop
	mfpr	r0, pt_usp
	hw_rei


	CALL_PAL_PRIV(0x003B)
CallPal_OpcDec3B:
	br	r31, osfpal_calpal_opcdec

// .sbttl	"whami - PALcode for whami instruction"
//+
// 
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//
// Function:
//	v0 (r0) <- whami
//-
        CALL_PAL_PRIV(PAL_WHAMI_ENTRY)
Call_Pal_Whami:
        nop
        mfpr    r0, pt_whami            // Get Whami
	extbl	r0, 1, r0		// Isolate just whami bits
        hw_rei

// .sbttl	"retsys - PALcode for retsys instruction"
// 
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//       00(sp) contains return pc
//       08(sp) contains r29
//
// Function:
//	Return from system call.
//       mode switched from kern to user.
//       stacks swapped, ugp, upc restored.
//       r23, r25 junked
//-

        CALL_PAL_PRIV(PAL_RETSYS_ENTRY)
Call_Pal_Retsys:
	lda	r25, osfsf_c_size(sp) 	// pop stack
	bis	r25, r31, r14		// touch r25 & r14 to stall mf exc_addr

	mfpr	r14, exc_addr		// save exc_addr in case of fault
	ldq	r23, osfsf_pc(sp) 	// get pc

	ldq	r29, osfsf_gp(sp) 	// get gp
	stl_c	r31, -4(sp)		// clear lock_flag

	lda	r11, 1<<osfps_v_mode(r31)// new PS:mode=user 
	mfpr	r30, pt_usp		// get users stack

	bic	r23, 3, r23		// clean return pc
	mtpr	r31, ev5__ipl		// zero ibox IPL - 2 bubbles to hw_rei
	
	mtpr	r11, ev5__dtb_cm	// set Mbox current mode - no virt ref for 2 cycles
	mtpr	r11, ev5__ps		// set Ibox current mode - 2 bubble to hw_rei

	mtpr	r23, exc_addr		// set return address - 1 bubble to hw_rei
	mtpr	r25, pt_ksp		// save kern stack

	rc	r31			// clear inter_flag
//	pvc_violate 248			// possible hidden mt->mf pt violation ok in callpal 
	hw_rei_spe			// and back	


	CALL_PAL_PRIV(0x003E)
CallPal_OpcDec3E:
	br	r31, osfpal_calpal_opcdec

// .sbttl	"rti - PALcode for rti instruction"
//+
// 
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//
// Function:
//	00(sp) -> ps
//	08(sp) -> pc
//	16(sp) -> r29 (gp)
//	24(sp) -> r16 (a0)
//	32(sp) -> r17 (a1)
//	40(sp) -> r18 (a3)
//-

        CALL_PAL_PRIV(PAL_RTI_ENTRY)
#ifdef SIMOS
        /* called once by platform_tlaser */
        .globl Call_Pal_Rti
#endif
Call_Pal_Rti:
 	lda	r25, osfsf_c_size(sp)	// get updated sp
	bis	r25, r31, r14		// touch r14,r25 to stall mf exc_addr

	mfpr	r14, exc_addr		// save PC in case of fault
	rc	r31			// clear intr_flag

	ldq	r12, -6*8(r25)		// get ps
	ldq	r13, -5*8(r25)		// pc

	ldq	r18, -1*8(r25)		// a2
	ldq	r17, -2*8(r25)		// a1

	ldq	r16, -3*8(r25)		// a0
	ldq	r29, -4*8(r25)		// gp

	bic	r13, 3, r13		// clean return pc
	stl_c	r31, -4(r25)		// clear lock_flag

	and	r12, osfps_m_mode, r11	// get mode
	mtpr	r13, exc_addr		// set return address

	beq	r11, rti_to_kern	// br if rti to Kern
	br	r31, rti_to_user	// out of call_pal space


// .sbttl  "Start the Unprivileged CALL_PAL Entry Points"
// .sbttl	"bpt- PALcode for bpt instruction"
//+
// 
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//
// Function:
//	Build stack frame
//	a0 <- code
//	a1 <- unpred
//	a2 <- unpred
//	vector via entIF
//
//-
//
	.text	1
//	. = 0x3000
        CALL_PAL_UNPRIV(PAL_BPT_ENTRY)
Call_Pal_Bpt:
	sll	r11, 63-osfps_v_mode, r25 // Shift mode up to MS bit
	mtpr	r31, ev5__ps		// Set Ibox current mode to kernel

	bis	r11, r31, r12		// Save PS for stack write
	bge	r25, CALL_PAL_bpt_10_		// no stack swap needed if cm=kern

	mtpr	r31, ev5__dtb_cm	// Set Mbox current mode to kernel - 
					//     no virt ref for next 2 cycles
	mtpr	r30, pt_usp		// save user stack

	bis	r31, r31, r11		// Set new PS	
	mfpr	r30, pt_ksp

CALL_PAL_bpt_10_:	
	lda	sp, 0-osfsf_c_size(sp)// allocate stack space 	
	mfpr	r14, exc_addr		// get pc

	stq	r16, osfsf_a0(sp)	// save regs
	bis	r31, osf_a0_bpt, r16	// set a0

	stq	r17, osfsf_a1(sp)	// a1
	br	r31, bpt_bchk_common	// out of call_pal space


// .sbttl	"bugchk- PALcode for bugchk instruction"
//+
// 
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//
// Function:
//	Build stack frame
//	a0 <- code
//	a1 <- unpred
//	a2 <- unpred
//	vector via entIF
//
//-
//
        CALL_PAL_UNPRIV(PAL_BUGCHK_ENTRY)
Call_Pal_Bugchk:
	sll	r11, 63-osfps_v_mode, r25 // Shift mode up to MS bit
	mtpr	r31, ev5__ps		// Set Ibox current mode to kernel

	bis	r11, r31, r12		// Save PS for stack write
	bge	r25, CALL_PAL_bugchk_10_		// no stack swap needed if cm=kern

	mtpr	r31, ev5__dtb_cm	// Set Mbox current mode to kernel - 
					//     no virt ref for next 2 cycles
	mtpr	r30, pt_usp		// save user stack

	bis	r31, r31, r11		// Set new PS	
	mfpr	r30, pt_ksp

CALL_PAL_bugchk_10_:	
	lda	sp, 0-osfsf_c_size(sp)// allocate stack space 	
	mfpr	r14, exc_addr		// get pc

	stq	r16, osfsf_a0(sp)	// save regs
	bis	r31, osf_a0_bugchk, r16	// set a0

	stq	r17, osfsf_a1(sp)	// a1
	br	r31, bpt_bchk_common	// out of call_pal space


	CALL_PAL_UNPRIV(0x0082)
CallPal_OpcDec82:
	br	r31, osfpal_calpal_opcdec

// .sbttl	"callsys - PALcode for callsys instruction"
//+
// 
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//
// Function:
// 	Switch mode to kernel and build a callsys stack frame.
//       sp = ksp
//       gp = kgp
//	t8 - t10 (r22-r24) trashed
//
//-
//
        CALL_PAL_UNPRIV(PAL_CALLSYS_ENTRY)
Call_Pal_Callsys:

	and	r11, osfps_m_mode, r24	// get mode
	mfpr	r22, pt_ksp		// get ksp

	beq	r24, sys_from_kern 	// sysCall from kern is not allowed
 	mfpr	r12, pt_entsys		// get address of callSys routine

//+
// from here on we know we are in user going to Kern
//-
	mtpr	r31, ev5__dtb_cm	// set Mbox current mode - no virt ref for 2 cycles
	mtpr	r31, ev5__ps		// set Ibox current mode - 2 bubble to hw_rei

	bis	r31, r31, r11		// PS=0 (mode=kern)
 	mfpr	r23, exc_addr		// get pc

	mtpr	r30, pt_usp		// save usp
	lda	sp, 0-osfsf_c_size(r22)// set new sp

	stq	r29, osfsf_gp(sp)	// save user gp/r29
	stq	r24, osfsf_ps(sp)	// save ps

	stq	r23, osfsf_pc(sp)	// save pc
	mtpr	r12, exc_addr		// set address
					// 1 cycle to hw_rei
	
	mfpr	r29, pt_kgp		// get the kern gp/r29

	hw_rei_spe			// and off we go!


	CALL_PAL_UNPRIV(0x0084)
CallPal_OpcDec84:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x0085)
CallPal_OpcDec85:
	br	r31, osfpal_calpal_opcdec

// .sbttl	"imb - PALcode for imb instruction"
//+
// 
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//
// Function:
//       Flush the writebuffer and flush the Icache
//
//-
//
        CALL_PAL_UNPRIV(PAL_IMB_ENTRY)
Call_Pal_Imb:
        mb                              // Clear the writebuffer
        mfpr    r31, ev5__mcsr          // Sync with clear
        nop
        nop
        br      r31, pal_ic_flush           // Flush Icache

	
// .sbttl	"CALL_PAL OPCDECs"

	CALL_PAL_UNPRIV(0x0087)
CallPal_OpcDec87:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x0088)
CallPal_OpcDec88:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x0089)
CallPal_OpcDec89:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x008A)
CallPal_OpcDec8A:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x008B)
CallPal_OpcDec8B:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x008C)
CallPal_OpcDec8C:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x008D)
CallPal_OpcDec8D:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x008E)
CallPal_OpcDec8E:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x008F)
CallPal_OpcDec8F:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x0090)
CallPal_OpcDec90:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x0091)
CallPal_OpcDec91:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x0092)
CallPal_OpcDec92:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x0093)
CallPal_OpcDec93:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x0094)
CallPal_OpcDec94:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x0095)
CallPal_OpcDec95:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x0096)
CallPal_OpcDec96:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x0097)
CallPal_OpcDec97:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x0098)
CallPal_OpcDec98:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x0099)
CallPal_OpcDec99:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x009A)
CallPal_OpcDec9A:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x009B)
CallPal_OpcDec9B:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x009C)
CallPal_OpcDec9C:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x009D)
CallPal_OpcDec9D:
	br	r31, osfpal_calpal_opcdec

// .sbttl	"rdunique - PALcode for rdunique instruction"
//+
// 
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//
// Function:
//	v0 (r0) <- unique
//
//-
//
        CALL_PAL_UNPRIV(PAL_RDUNIQUE_ENTRY)
CALL_PALrdunique_:
	mfpr	r0, pt_pcbb		// get pcb pointer
	ldqp	r0, osfpcb_q_unique(r0) // get new value
	
	hw_rei

// .sbttl	"wrunique - PALcode for wrunique instruction"
//+
// 
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//
// Function:
//	unique <- a0 (r16)
//
//-
//
CALL_PAL_UNPRIV(PAL_WRUNIQUE_ENTRY)
CALL_PAL_Wrunique:
	nop
	mfpr	r12, pt_pcbb		// get pcb pointer
	stqp	r16, osfpcb_q_unique(r12)// get new value
	nop				// Pad palshadow write
	hw_rei				// back

// .sbttl	"CALL_PAL OPCDECs"

	CALL_PAL_UNPRIV(0x00A0)
CallPal_OpcDecA0:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x00A1)
CallPal_OpcDecA1:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x00A2)
CallPal_OpcDecA2:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x00A3)
CallPal_OpcDecA3:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x00A4)
CallPal_OpcDecA4:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x00A5)
CallPal_OpcDecA5:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x00A6)
CallPal_OpcDecA6:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x00A7)
CallPal_OpcDecA7:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x00A8)
CallPal_OpcDecA8:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x00A9)
CallPal_OpcDecA9:
	br	r31, osfpal_calpal_opcdec


// .sbttl	"gentrap - PALcode for gentrap instruction"
//+
// CALL_PAL_gentrap:
// Entry:
//	Vectored into via hardware PALcode instruction dispatch.
//
// Function:
//	Build stack frame
//	a0 <- code
//	a1 <- unpred
//	a2 <- unpred
//	vector via entIF
//
//-

	CALL_PAL_UNPRIV(0x00AA)
// unsupported in Hudson code .. pboyle Nov/95
CALL_PAL_gentrap:
	sll	r11, 63-osfps_v_mode, r25 // Shift mode up to MS bit
	mtpr	r31, ev5__ps		// Set Ibox current mode to kernel

	bis	r11, r31, r12			// Save PS for stack write
	bge	r25, CALL_PAL_gentrap_10_	// no stack swap needed if cm=kern

	mtpr	r31, ev5__dtb_cm	// Set Mbox current mode to kernel - 
					//     no virt ref for next 2 cycles
	mtpr	r30, pt_usp		// save user stack

	bis	r31, r31, r11		// Set new PS	
	mfpr	r30, pt_ksp

CALL_PAL_gentrap_10_:	
	lda	sp, 0-osfsf_c_size(sp)// allocate stack space 	
	mfpr	r14, exc_addr		// get pc

	stq	r16, osfsf_a0(sp)	// save regs
	bis	r31, osf_a0_gentrap, r16// set a0

	stq	r17, osfsf_a1(sp)	// a1
	br	r31, bpt_bchk_common	// out of call_pal space

	
// .sbttl	"CALL_PAL OPCDECs"

	CALL_PAL_UNPRIV(0x00AB)
CallPal_OpcDecAB:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x00AC)
CallPal_OpcDecAC:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x00AD)
CallPal_OpcDecAD:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x00AE)
CallPal_OpcDecAE:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x00AF)
CallPal_OpcDecAF:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x00B0)
CallPal_OpcDecB0:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x00B1)
CallPal_OpcDecB1:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x00B2)
CallPal_OpcDecB2:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x00B3)
CallPal_OpcDecB3:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x00B4)
CallPal_OpcDecB4:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x00B5)
CallPal_OpcDecB5:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x00B6)
CallPal_OpcDecB6:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x00B7)
CallPal_OpcDecB7:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x00B8)
CallPal_OpcDecB8:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x00B9)
CallPal_OpcDecB9:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x00BA)
CallPal_OpcDecBA:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x00BB)
CallPal_OpcDecBB:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x00BC)
CallPal_OpcDecBC:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x00BD)
CallPal_OpcDecBD:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x00BE)
CallPal_OpcDecBE:
	br	r31, osfpal_calpal_opcdec

	CALL_PAL_UNPRIV(0x00BF)
CallPal_OpcDecBF:
	// MODIFIED BY EGH 2/25/04
	br	r31, copypal_impl
	
	
/*======================================================================*/
/*                   OSF/1 CALL_PAL CONTINUATION AREA                   */
/*======================================================================*/

	.text	2

	. = 0x4000


// .sbttl	"Continuation of MTPR_PERFMON"
	ALIGN_BLOCK
#if perfmon_debug == 0
          // "real" performance monitoring code
// mux ctl
perfmon_muxctl:
        lda     r8, 1(r31) 			// get a 1
        sll     r8, pmctr_v_sel0, r8		// move to sel0 position
        or      r8, ((0xf<<pmctr_v_sel1) | (0xf<<pmctr_v_sel2)), r8	// build mux select mask
	and	r17, r8, r25			// isolate pmctr mux select bits
	mfpr	r0, ev5__pmctr
	bic	r0, r8, r0			// clear old mux select bits
	or	r0,r25, r25			// or in new mux select bits
	mtpr	r25, ev5__pmctr		

	// ok, now tackle cbox mux selects
        ldah    r14, 0xfff0(r31)
	zap     r14, 0xE0, r14                 // Get Cbox IPR base
//orig	get_bc_ctl_shadow	r16		// bc_ctl returned in lower longword
// adapted from ev5_pal_macros.mar
	mfpr	r16, pt_impure
	lda	r16, CNS_Q_IPR(r16)
	RESTORE_SHADOW(r16,CNS_Q_BC_CTL,r16); 

	lda	r8, 0x3F(r31)			// build mux select mask
	sll	r8, bc_ctl_v_pm_mux_sel, r8

	and	r17, r8, r25			// isolate bc_ctl mux select bits
	bic	r16, r8, r16			// isolate old mux select bits
	or	r16, r25, r25			// create new bc_ctl
	mb					// clear out cbox for future ipr write
	stqp	r25, ev5__bc_ctl(r14)		// store to cbox ipr
	mb					// clear out cbox for future ipr write

//orig	update_bc_ctl_shadow	r25, r16	// r25=value, r16-overwritten with adjusted impure ptr
// adapted from ev5_pal_macros.mar
	mfpr	r16, pt_impure
	lda	r16, CNS_Q_IPR(r16)
	SAVE_SHADOW(r25,CNS_Q_BC_CTL,r16); 

	br 	r31, perfmon_success
	

// requested to disable perf monitoring
perfmon_dis:
	mfpr	r14, ev5__pmctr		// read ibox pmctr ipr
perfmon_dis_ctr0:			// and begin with ctr0
	blbc	r17, perfmon_dis_ctr1	// do not disable ctr0
	lda 	r8, 3(r31)
	sll	r8, pmctr_v_ctl0, r8
	bic	r14, r8, r14		// disable ctr0
perfmon_dis_ctr1:
	srl	r17, 1, r17
	blbc	r17, perfmon_dis_ctr2	// do not disable ctr1
	lda 	r8, 3(r31)
	sll	r8, pmctr_v_ctl1, r8
	bic	r14, r8, r14		// disable ctr1
perfmon_dis_ctr2:
	srl	r17, 1, r17
	blbc	r17, perfmon_dis_update	// do not disable ctr2
	lda 	r8, 3(r31)
	sll	r8, pmctr_v_ctl2, r8
	bic	r14, r8, r14		// disable ctr2
perfmon_dis_update:
	mtpr	r14, ev5__pmctr		// update pmctr ipr
//;the following code is not needed for ev5 pass2 and later, but doesn't hurt anything to leave in
// adapted from ev5_pal_macros.mar
//orig	get_pmctr_ctl	r8, r25		// pmctr_ctl bit in r8.  adjusted impure pointer in r25
	mfpr	r25, pt_impure
	lda	r25, CNS_Q_IPR(r25)
	RESTORE_SHADOW(r8,CNS_Q_PM_CTL,r25); 

	lda	r17, 0x3F(r31)		// build mask
	sll	r17, pmctr_v_ctl2, r17 // shift mask to correct position
	and 	r14, r17, r14		// isolate ctl bits
	bic	r8, r17, r8		// clear out old ctl bits
	or	r14, r8, r14		// create shadow ctl bits 
//orig	store_reg1 pmctr_ctl, r14, r25, ipr=1	// update pmctr_ctl register
//adjusted impure pointer still in r25	
	SAVE_SHADOW(r14,CNS_Q_PM_CTL,r25); 

	br 	r31, perfmon_success


// requested to enable perf monitoring
//;the following code can be greatly simplified for pass2, but should work fine as is.


perfmon_enclr:
	lda	r9, 1(r31)		// set enclr flag
	br perfmon_en_cont

perfmon_en:
	bis	r31, r31, r9		// clear enclr flag

perfmon_en_cont:
	mfpr	r8, pt_pcbb		// get PCB base
//orig	get_pmctr_ctl r25, r25
	mfpr	r25, pt_impure
	lda	r25, CNS_Q_IPR(r25)
	RESTORE_SHADOW(r25,CNS_Q_PM_CTL,r25); 

	ldqp	r16, osfpcb_q_fen(r8)	// read DAT/PME/FEN quadword
	mfpr	r14, ev5__pmctr		// read ibox pmctr ipr
	srl 	r16, osfpcb_v_pme, r16	// get pme bit
	mfpr	r13, icsr
	and	r16,  1, r16		// isolate pme bit

	// this code only needed in pass2 and later
//orig	sget_addr	r12, 1<<icsr_v_pmp, r31
	lda	r12, 1<<icsr_v_pmp(r31)		// pb
	bic	r13, r12, r13		// clear pmp bit
	sll	r16, icsr_v_pmp, r12	// move pme bit to icsr<pmp> position
	or	r12, r13, r13		// new icsr with icsr<pmp> bit set/clear
	ev5_pass2 	mtpr	r13, icsr		// update icsr

#if ev5_p1 != 0
	lda	r12, 1(r31)
	cmovlbc	r25, r12, r16		// r16<0> set if either pme=1 or sprocess=0 (sprocess in bit 0 of r25)
#else
	bis	r31, 1, r16		// set r16<0> on pass2 to update pmctr always (icsr provides real enable)
#endif

	sll	r25, 6, r25		// shift frequency bits into pmctr_v_ctl positions
	bis	r14, r31, r13		// copy pmctr

perfmon_en_ctr0:			// and begin with ctr0
	blbc	r17, perfmon_en_ctr1	// do not enable ctr0
	
	blbc	r9, perfmon_en_noclr0	// enclr flag set, clear ctr0 field
	lda	r8, 0xffff(r31)
	zapnot  r8, 3, r8		// ctr0<15:0> mask
	sll	r8, pmctr_v_ctr0, r8
	bic	r14, r8, r14		// clear ctr bits
	bic	r13, r8, r13		// clear ctr bits

perfmon_en_noclr0:
//orig	get_addr r8, 3<<pmctr_v_ctl0, r31
	LDLI(r8, (3<<pmctr_v_ctl0))
	and 	r25, r8, r12		//isolate frequency select bits for ctr0
	bic	r14, r8, r14		// clear ctl0 bits in preparation for enabling
	or	r14,r12,r14		// or in new ctl0 bits
		
perfmon_en_ctr1:			// enable ctr1
	srl	r17, 1, r17		// get ctr1 enable
	blbc	r17, perfmon_en_ctr2	// do not enable ctr1

	blbc	r9, perfmon_en_noclr1   // if enclr flag set, clear ctr1 field
	lda	r8, 0xffff(r31)
	zapnot  r8, 3, r8		// ctr1<15:0> mask
	sll	r8, pmctr_v_ctr1, r8
	bic	r14, r8, r14		// clear ctr bits
	bic	r13, r8, r13		// clear ctr bits

perfmon_en_noclr1:
//orig	get_addr r8, 3<<pmctr_v_ctl1, r31
	LDLI(r8, (3<<pmctr_v_ctl1))
	and 	r25, r8, r12		//isolate frequency select bits for ctr1
	bic	r14, r8, r14		// clear ctl1 bits in preparation for enabling
	or	r14,r12,r14		// or in new ctl1 bits

perfmon_en_ctr2:			// enable ctr2
	srl	r17, 1, r17		// get ctr2 enable
	blbc	r17, perfmon_en_return	// do not enable ctr2 - return

	blbc	r9, perfmon_en_noclr2	// if enclr flag set, clear ctr2 field
	lda	r8, 0x3FFF(r31)		// ctr2<13:0> mask
	sll	r8, pmctr_v_ctr2, r8
	bic	r14, r8, r14		// clear ctr bits
	bic	r13, r8, r13		// clear ctr bits

perfmon_en_noclr2:
//orig	get_addr r8, 3<<pmctr_v_ctl2, r31
	LDLI(r8, (3<<pmctr_v_ctl2))
	and 	r25, r8, r12		//isolate frequency select bits for ctr2
	bic	r14, r8, r14		// clear ctl2 bits in preparation for enabling
	or	r14,r12,r14		// or in new ctl2 bits

perfmon_en_return:
	cmovlbs	r16, r14, r13		// if pme enabled, move enables into pmctr
					// else only do the counter clears
	mtpr	r13, ev5__pmctr		// update pmctr ipr

//;this code not needed for pass2 and later, but does not hurt to leave it in
	lda	r8, 0x3F(r31)
//orig	get_pmctr_ctl r25, r12         	// read pmctr ctl; r12=adjusted impure pointer
	mfpr	r12, pt_impure
	lda	r12, CNS_Q_IPR(r12)
	RESTORE_SHADOW(r25,CNS_Q_PM_CTL,r12); 

	sll	r8, pmctr_v_ctl2, r8	// build ctl mask
	and	r8, r14, r14		// isolate new ctl bits
	bic	r25, r8, r25		// clear out old ctl value
	or	r25, r14, r14		// create new pmctr_ctl
//orig	store_reg1 pmctr_ctl, r14, r12, ipr=1
	SAVE_SHADOW(r14,CNS_Q_PM_CTL,r12); // r12 still has the adjusted impure ptr

	br 	r31, perfmon_success


// options...
perfmon_ctl:

// set mode 
//orig	get_pmctr_ctl r14, r12         	// read shadow pmctr ctl; r12=adjusted impure pointer
	mfpr	r12, pt_impure
	lda	r12, CNS_Q_IPR(r12)
	RESTORE_SHADOW(r14,CNS_Q_PM_CTL,r12); 
	
//orig	get_addr r8, (1<<pmctr_v_killu) | (1<<pmctr_v_killp) | (1<<pmctr_v_killk), r31          // build mode mask for pmctr register
	LDLI(r8, ((1<<pmctr_v_killu) | (1<<pmctr_v_killp) | (1<<pmctr_v_killk)))
	mfpr	r0, ev5__pmctr
	and	r17, r8, r25			// isolate pmctr mode bits
	bic	r0, r8, r0			// clear old mode bits
	or	r0, r25, r25			// or in new mode bits
	mtpr	r25, ev5__pmctr		

//;the following code will only be used in pass2, but should not hurt anything if run in pass1.
	mfpr	r8, icsr
	lda	r25, 1<<icsr_v_pma(r31)		// set icsr<pma> if r17<0>=0
	bic 	r8, r25, r8			// clear old pma bit
	cmovlbs r17, r31, r25			// and clear icsr<pma> if r17<0>=1
	or	r8, r25, r8
	ev5_pass2 mtpr	r8, icsr		// 4 bubbles to hw_rei
	mfpr	r31, pt0			// pad icsr write
	mfpr	r31, pt0			// pad icsr write

//;the following code not needed for pass2 and later, but should work anyway.
	bis     r14, 1, r14       		// set for select processes
	blbs	r17, perfmon_sp			// branch if select processes
	bic	r14, 1, r14			// all processes
perfmon_sp:
//orig	store_reg1 pmctr_ctl, r14, r12, ipr=1   // update pmctr_ctl register
	SAVE_SHADOW(r14,CNS_Q_PM_CTL,r12); // r12 still has the adjusted impure ptr
	br 	r31, perfmon_success
	
// counter frequency select
perfmon_freq:
//orig	get_pmctr_ctl r14, r12         	// read shadow pmctr ctl; r12=adjusted impure pointer
	mfpr	r12, pt_impure
	lda	r12, CNS_Q_IPR(r12)
	RESTORE_SHADOW(r14,CNS_Q_PM_CTL,r12); 

	lda	r8, 0x3F(r31)
//orig	sll	r8, pmctr_ctl_v_frq2, r8		// build mask for frequency select field
// I guess this should be a shift of 4 bits from the above control register structure	.. pb 
#define	pmctr_ctl_v_frq2_SHIFT 4
	sll	r8, pmctr_ctl_v_frq2_SHIFT, r8		// build mask for frequency select field

	and 	r8, r17, r17
	bic 	r14, r8, r14				// clear out old frequency select bits

	or 	r17, r14, r14				// or in new frequency select info
//orig	store_reg1 pmctr_ctl, r14, r12, ipr=1   // update pmctr_ctl register
	SAVE_SHADOW(r14,CNS_Q_PM_CTL,r12); // r12 still has the adjusted impure ptr

	br 	r31, perfmon_success

// read counters
perfmon_rd:
	mfpr	r0, ev5__pmctr
	or	r0, 1, r0	// or in return status
	hw_rei			// back to user
	
// write counters
perfmon_wr:
	mfpr	r14, ev5__pmctr
	lda	r8, 0x3FFF(r31)		// ctr2<13:0> mask
	sll	r8, pmctr_v_ctr2, r8

//orig	get_addr r9, 0xFFFFFFFF, r31, verify=0	// ctr2<15:0>,ctr1<15:0> mask
	LDLI(r9, (0xFFFFFFFF))
	sll	r9, pmctr_v_ctr1, r9
	or	r8, r9, r8		// or ctr2, ctr1, ctr0 mask
	bic	r14, r8, r14		// clear ctr fields
	and	r17, r8, r25		// clear all but ctr  fields
	or	r25, r14, r14		// write ctr fields
	mtpr	r14, ev5__pmctr		// update pmctr ipr

	mfpr	r31, pt0		// pad pmctr write (needed only to keep PVC happy)

perfmon_success:
	or      r31, 1, r0                     // set success
	hw_rei					// back to user

perfmon_unknown:	
	or	r31, r31, r0		// set fail
	hw_rei				// back to user

#else
	
// end of "real code", start of debug code

//+ 
// Debug environment:
// (in pass2, always set icsr<pma> to ensure master counter enable is on)
// 	R16 = 0		Write to on-chip performance monitor ipr
//	   r17 = 	  on-chip ipr
//	   r0 = 	  return value of read of on-chip performance monitor ipr
//	R16 = 1		Setup Cbox mux selects
//	   r17 = 	  Cbox mux selects in same position as in bc_ctl ipr.
//	   r0 = 	  return value of read of on-chip performance monitor ipr
//
//-
pal_perfmon_debug:
	mfpr	r8, icsr
	lda	r9, 1<<icsr_v_pma(r31)
	bis	r8, r9, r8
	mtpr	r8, icsr

	mfpr	r0,  ev5__pmctr		// read old value
	bne	r16, cbox_mux_sel

	mtpr	r17, ev5__pmctr		// update pmctr ipr
	br	r31, end_pm

cbox_mux_sel:
	// ok, now tackle cbox mux selects
        ldah    r14, 0xfff0(r31)
	zap     r14, 0xE0, r14                 // Get Cbox IPR base
//orig	get_bc_ctl_shadow	r16		// bc_ctl returned
	mfpr	r16, pt_impure
	lda	r16, CNS_Q_IPR(r16)
	RESTORE_SHADOW(r16,CNS_Q_BC_CTL,r16); 

	lda	r8, 0x3F(r31)			// build mux select mask
	sll	r8, BC_CTL_V_PM_MUX_SEL, r8

	and	r17, r8, r25			// isolate bc_ctl mux select bits
	bic	r16, r8, r16			// isolate old mux select bits
	or	r16, r25, r25			// create new bc_ctl
	mb					// clear out cbox for future ipr write
	stqp	r25, ev5__bc_ctl(r14)		// store to cbox ipr
	mb					// clear out cbox for future ipr write
//orig	update_bc_ctl_shadow	r25, r16	// r25=value, r16-overwritten with adjusted impure ptr
	mfpr	r16, pt_impure
	lda	r16, CNS_Q_IPR(r16)
	SAVE_SHADOW(r25,CNS_Q_BC_CTL,r16); 

end_pm:	hw_rei

#endif


//;The following code is a workaround for a cpu bug where Istream prefetches to 
//;super-page address space in user mode may escape off-chip.
#if spe_fix != 0
	
	ALIGN_BLOCK
hw_rei_update_spe:
	mfpr	r12, pt_misc			// get previous mode
	srl	r11, osfps_v_mode, r10		// isolate current mode bit
	and	r10, 1, r10		 	
	extbl	r12, 7, r8			// get previous mode field
	and	r8, 1, r8	 		// isolate previous mode bit
	cmpeq	r10, r8, r8			// compare previous and current modes
	beq	r8, hw_rei_update_spe_5_				
	hw_rei					// if same, just return

hw_rei_update_spe_5_:
	
#if fill_err_hack != 0

	fill_error_hack
#endif

	mfpr	r8, icsr			// get current icsr value
	ldah	r9, (2<<(icsr_v_spe-16))(r31)	// get spe bit mask
	bic	r8, r9, r8			// disable spe
	xor	r10, 1, r9			// flip mode for new spe bit
	sll	r9, icsr_v_spe+1, r9		// shift into position
	bis	r8, r9, r8			// enable/disable spe
	lda	r9, 1(r31)			// now update our flag
	sll	r9, pt_misc_v_cm, r9		// previous mode saved bit mask
	bic	r12, r9, r12			// clear saved previous mode
	sll	r10, pt_misc_v_cm, r9		// current mode saved bit mask
	bis	r12, r9, r12			// set saved current mode
	mtpr	r12, pt_misc			// update pt_misc
	mtpr	r8, icsr			// update icsr

#if osf_chm_fix != 0


	blbc	r10, hw_rei_update_spe_10_			// branch if not user mode

	mb					// ensure no outstanding fills
	lda	r12, 1<<dc_mode_v_dc_ena(r31)	// User mode
	mtpr	r12, dc_mode			// Turn on dcache
	mtpr	r31, dc_flush			// and flush it
	br	r31, pal_ic_flush

hw_rei_update_spe_10_:	mfpr	r9, pt_pcbb			// Kernel mode
	ldqp	r9, osfpcb_q_Fen(r9)		// get FEN
	blbc	r9, pal_ic_flush		// return if FP disabled
	mb					// ensure no outstanding fills
	mtpr	r31, dc_mode			// turn off dcache
#endif


	br	r31, pal_ic_flush		// Pal restriction - must flush Icache if changing ICSR<SPE>
#endif


copypal_impl:	
	mov r16, r0
	bic r18, 63, r8
	and r18, 63, r18
	beq r8, cache_copy_done
cache_loop:
	ldf f17, 0(r16)		
	stf f17, 0(r16)
	addq r17, 64, r17
	addq r16, 64, r16
	subq r8, 64, r8
	bne r8, cache_loop
cache_copy_done:	 	 
	ble r18, finished	#if len <=0 we are finished 
	ldq_u r8, 0(r17) 
	xor r17, r16, r9 
	and r9, 7, r9 
	and r16, 7, r10	
	bne r9, unaligned 
	beq r10, aligned	 
	ldq_u r9, 0(r16) 
	addq r18, r10, r18
	mskqh r8, r17, r8
	mskql r9, r17, r9
	bis r8, r9, r8
aligned:
	subq r18, 1, r10
	bic r10, 7, r10
	and r18, 7, r18
	beq r10, aligned_done
loop:
	stq_u r8, 0(r16)
	ldq_u r8, 8(r17)
	subq r10, 8, r10
	lda r16,8(r16)
	lda r17,8(r17)
	bne r10, loop
aligned_done:
	bne r18, few_left
	stq_u r8, 0(r16)
	br r31, finished
	few_left:
	mskql r8, r18, r10
	ldq_u r9, 0(r16)
	mskqh r9, r18, r9
	bis r10, r9, r10
	stq_u r10, 0(r16)
	br r31, finished
unaligned:
	addq r17, r18, r25
	cmpule r18, 8, r9
	bne r9, unaligned_few_left
	beq r10, unaligned_dest_aligned
	and r16, 7, r10
	subq r31, r10, r10
	addq r10, 8, r10
	ldq_u r9, 7(r17)
	extql r8, r17, r8
	extqh r9, r17, r9
	bis r8, r9, r12
	insql r12, r16, r12
	ldq_u r13, 0(r16)
	mskql r13, r16, r13
	bis r12, r13, r12
	stq_u r12, 0(r16)
	addq r16, r10, r16
	addq r17, r10, r17
	subq r18, r10, r18
	ldq_u r8, 0(r17)
unaligned_dest_aligned:
	subq r18, 1, r10
	bic r10, 7, r10
	and r18, 7, r18
	beq r10, unaligned_partial_left
unaligned_loop:
	ldq_u r9, 7(r17)
	lda r17, 8(r17)
	extql r8, r17, r12
	extqh r9, r17, r13
	subq r10, 8, r10
	bis r12, r13, r13
	stq r13, 0(r16)
	lda r16, 8(r16)
	beq r10, unaligned_second_partial_left
	ldq_u r8, 7(r17)
	lda r17, 8(r17)
	extql r9, r17, r12
	extqh r8, r17, r13
	bis r12, r13, r13
	subq r10, 8, r10
	stq r13, 0(r16)
	lda r16, 8(r16)
	bne r10, unaligned_loop
unaligned_partial_left:
	mov r8, r9
unaligned_second_partial_left:
	ldq_u r8, -1(r25)
	extql r9, r17, r9
	extqh r8, r17, r8
	bis r8, r9, r8
	bne r18, few_left
	stq_u r8, 0(r16)
	br r31, finished
unaligned_few_left:
	ldq_u r9, -1(r25)
	extql r8, r17, r8
	extqh r9, r17, r9
	bis r8, r9, r8
	insqh r8, r16, r9
	insql r8, r16, r8
	lda r12, -1(r31)
	mskql r12, r18, r13
	cmovne r13, r13, r12
	insqh r12, r16, r13
	insql r12, r16, r12
	addq r16, r18, r10
	ldq_u r14, 0(r16)
	ldq_u r25, -1(r10)
	bic r14, r12, r14
	bic r25, r13, r25
	and r8, r12, r8
	and r9, r13, r9
	bis r8, r14, r8
	bis r9, r25, r9
	stq_u r9, -1(r10)
	stq_u r8, 0(r16)
finished:
	hw_rei
