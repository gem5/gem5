#ifndef EV5_IMPURE_INCLUDED
#define EV5_IMPURE_INCLUDED

/*
// This uses the Hudson file format from "impure.h" but with the fields from
// the distrubuted palcode "ev5_impure.sdl" .. pboyle Nov/95

//orig  file:	impure.sdl
//orig
//orig  Abstract:	PAL impure scratch area and logout area data structure definitions for
//orig 		Alpha firmware.
//orig
//orig
//orig  module	$pal_impure;
//orig
//orig  Edit   Date     Who       Description
//orig  ---- ---------  ---  ---------------------
//orig     1   7-Jul-93 JEM   Initial Entry
//orig     2  18-nov-93 JEM   Add shadow bc_ctl and pmctr_ctl to impure area
//orig 			 Delete mvptbr
//orig 			 Calculate pal$logout from end of impure area
//orig     3   6-dec-93 JEM   Add pmctr_ctl bitfield definitions
//orig     4   3-feb-94 JEM   Remove f31,r31 from impure area; Remove bc_ctl, pmctr_ctl;
//orig 				add ic_perr_stat, pmctr, dc_perr_stat, sc_stat, sc_addr, sc_ctl,
//orig 				    bc_tag_addr, ei_stat, ei_addr, fill_syn, ld_lock
//orig     5  19-feb-94 JEM   add gpr constants, and add f31,r31 back in to be consistent with ev4
//orig 			 add cns$ipr_offset
//orig     6  18-apr-94 JEM   Add shadow bc_ctl and pmctr_ctl to impure area again.
//orig     7  18-jul-94 JEM   Add bc_config shadow.   Add mchk$sys_base constant to mchk logout frame
//orig
//orig
//orig     constant REVISION equals 7 prefix IMPURE$;            // Revision number of this file
//orig

** Macros for saving/restoring data to/from the PAL impure scratch
** area.
**
** The console save state area is larger than the addressibility
** of the HW_LD/ST instructions (10-bit signed byte displacement),
** so some adjustments to the base offsets, as well as the offsets
** within each base region, are necessary.
**
** The console save state area is divided into two segments; the
** CPU-specific segment and the platform-specific segment.  The
** state that is saved in the CPU-specific segment includes GPRs,
** FPRs, IPRs, halt code, MCHK flag, etc.  All other state is saved
** in the platform-specific segment.
**
** The impure pointer will need to be adjusted by a different offset
** value for each region within a given segment.  The SAVE and RESTORE
** macros will auto-magically adjust the offsets accordingly.
**
*/
#if 0
#define SAVE_GPR(reg,offset,base) \
        stq_p	reg, ((offset-0x200)&0x3FF)(base)

#define RESTORE_GPR(reg,offset,base) \
        ldq_p	reg, ((offset-0x200)&0x3FF)(base)


#define SAVE_FPR(reg,offset,base) \
        stt	reg, ((offset-0x200)&0x3FF)(base)

#define RESTORE_FPR(reg,offset,base) \
        ldt	reg, ((offset-0x200)&0x3FF)(base)

#define SAVE_IPR(reg,offset,base) \
        mfpr	v0, reg;	  \
        stq_p	v0, ((offset-CNS_Q_IPR)&0x3FF)(base)

#define RESTORE_IPR(reg,offset,base) \
        ldq_p	v0, ((offset-CNS_Q_IPR)&0x3FF)(base); \
        mtpr	v0, reg

#define SAVE_SHADOW(reg,offset,base) \
        stq_p	reg, ((offset-CNS_Q_IPR)&0x3FF)(base)

#define	RESTORE_SHADOW(reg,offset,base)\
        ldq_p	reg, ((offset-CNS_Q_IPR)&0x3FF)(base)
#else
//#define SEXT10(X) (((X) & 0x200) ? ((X) | 0xfffffffffffffc00) : (X))
#define SEXT10(X) ((X) & 0x3ff)
//#define SEXT10(X) (((X) << 55) >> 55)

#define SAVE_GPR(reg,offset,base) \
        stq_p	reg, (SEXT10(offset-0x200))(base)

#define RESTORE_GPR(reg,offset,base) \
        ldq_p	reg, (SEXT10(offset-0x200))(base)


#define SAVE_FPR(reg,offset,base) \
        stt	reg, (SEXT10(offset-0x200))(base)

#define RESTORE_FPR(reg,offset,base) \
        ldt	reg, (SEXT10(offset-0x200))(base)

#define SAVE_IPR(reg,offset,base) \
        mfpr	v0, reg;	  \
        stq_p	v0, (SEXT10(offset-CNS_Q_IPR))(base)

#define RESTORE_IPR(reg,offset,base) \
        ldq_p	v0, (SEXT10(offset-CNS_Q_IPR))(base); \
        mtpr	v0, reg

#define SAVE_SHADOW(reg,offset,base) \
        stq_p	reg, (SEXT10(offset-CNS_Q_IPR))(base)

#define	RESTORE_SHADOW(reg,offset,base)\
        ldq_p	reg, (SEXT10(offset-CNS_Q_IPR))(base)
#endif
/* orig  Structure of the processor-specific impure area */

/* orig aggregate impure struct prefix "" tag "";
 * orig 	cns$flag	quadword;
 * orig 	cns$hlt		quadword;
*/

/* Define base for debug monitor compatibility */
#define CNS_Q_BASE      0x000
#define CNS_Q_FLAG	0x100
#define CNS_Q_HALT	0x108


/* orig constant (
 * orig 	cns$r0,cns$r1,cns$r2,cns$r3,cns$r4,cns$r5,cns$r6,cns$r7,
 * orig 	cns$r8,cns$r9,cns$r10,cns$r11,cns$r12,cns$r13,cns$r14,cns$r15,
 * orig 	cns$r16,cns$r17,cns$r18,cns$r19,cns$r20,cns$r21,cns$r22,cns$r23,
 * orig 	cns$r24,cns$r25,cns$r26,cns$r27,cns$r28,cns$r29,cns$r30,cns$r31
 * orig 	) equals . increment 8 prefix "" tag "";
 * orig 	cns$gpr	quadword dimension 32;
*/
/* Offset to base of saved GPR area - 32 quadword */
#define CNS_Q_GPR	0x110
#define cns_gpr CNS_Q_GPR

/* orig constant (
 * orig 	cns$f0,cns$f1,cns$f2,cns$f3,cns$f4,cns$f5,cns$f6,cns$f7,
 * orig 	cns$f8,cns$f9,cns$f10,cns$f11,cns$f12,cns$f13,cns$f14,cns$f15,
 * orig 	cns$f16,cns$f17,cns$f18,cns$f19,cns$f20,cns$f21,cns$f22,cns$f23,
 * orig 	cns$f24,cns$f25,cns$f26,cns$f27,cns$f28,cns$f29,cns$f30,cns$f31
 * orig 	) equals . increment 8 prefix "" tag "";
 * orig 	cns$fpr	quadword dimension 32;
*/
/* Offset to base of saved FPR area - 32 quadwords */
#define CNS_Q_FPR	0x210

/* orig 	#t=.;
 * orig 	cns$mchkflag quadword;
*/
#define CNS_Q_MCHK	0x310

/* orig 	constant cns$pt_offset equals .;
 * orig  constant (
 * orig 	cns$pt0,cns$pt1,cns$pt2,cns$pt3,cns$pt4,cns$pt5,cns$pt6,
 * orig 	cns$pt7,cns$pt8,cns$pt9,cns$pt10,cns$pt11,cns$pt12,cns$pt13,
 * orig 	cns$pt14,cns$pt15,cns$pt16,cns$pt17,cns$pt18,cns$pt19,cns$pt20,
 * orig 	cns$pt21,cns$pt22,cns$pt23
 * orig 	) equals . increment 8 prefix "" tag "";
 * orig 	cns$pt	quadword dimension 24;
*/
/* Offset to base of saved PALtemp area - 25 quadwords */
#define CNS_Q_PT	0x318

/* orig 	cns$shadow8	quadword;
 * orig 	cns$shadow9	quadword;
 * orig 	cns$shadow10	quadword;
 * orig 	cns$shadow11	quadword;
 * orig 	cns$shadow12	quadword;
 * orig 	cns$shadow13	quadword;
 * orig 	cns$shadow14	quadword;
 * orig 	cns$shadow25	quadword;
*/
/* Offset to base of saved PALshadow area - 8 quadwords */
#define CNS_Q_SHADOW	0x3D8

/* Offset to base of saved IPR area */
#define CNS_Q_IPR	0x418

/* orig 	constant cns$ipr_offset equals .; */
/* orig 	cns$exc_addr	quadword; */
#define CNS_Q_EXC_ADDR		0x418
/* orig 	cns$pal_base	quadword; */
#define CNS_Q_PAL_BASE		0x420
/* orig 	cns$mm_stat	quadword; */
#define CNS_Q_MM_STAT		0x428
/* orig 	cns$va		quadword; */
#define CNS_Q_VA		0x430
/* orig 	cns$icsr	quadword; */
#define CNS_Q_ICSR		0x438
/* orig 	cns$ipl		quadword; */
#define CNS_Q_IPL		0x440
/* orig 	cns$ps		quadword;	// Ibox current mode */
#define CNS_Q_IPS		0x448
/* orig 	cns$itb_asn	quadword; */
#define CNS_Q_ITB_ASN		0x450
/* orig 	cns$aster	quadword; */
#define CNS_Q_ASTER		0x458
/* orig 	cns$astrr	quadword; */
#define CNS_Q_ASTRR		0x460
/* orig 	cns$isr		quadword; */
#define CNS_Q_ISR		0x468
/* orig 	cns$ivptbr	quadword; */
#define CNS_Q_IVPTBR		0x470
/* orig 	cns$mcsr	quadword; */
#define CNS_Q_MCSR		0x478
/* orig 	cns$dc_mode	quadword; */
#define CNS_Q_DC_MODE		0x480
/* orig 	cns$maf_mode	quadword; */
#define CNS_Q_MAF_MODE		0x488
/* orig 	cns$sirr	quadword; */
#define CNS_Q_SIRR		0x490
/* orig 	cns$fpcsr	quadword; */
#define CNS_Q_FPCSR		0x498
/* orig 	cns$icperr_stat	quadword; */
#define CNS_Q_ICPERR_STAT	0x4A0
/* orig 	cns$pmctr	quadword; */
#define CNS_Q_PM_CTR		0x4A8
/* orig 	cns$exc_sum	quadword; */
#define CNS_Q_EXC_SUM		0x4B0
/* orig 	cns$exc_mask	quadword; */
#define CNS_Q_EXC_MASK		0x4B8
/* orig 	cns$intid	quadword; */
#define CNS_Q_INT_ID		0x4C0
/* orig 	cns$dcperr_stat quadword; */
#define CNS_Q_DCPERR_STAT	0x4C8
/* orig 	cns$sc_stat	quadword; */
#define CNS_Q_SC_STAT		0x4D0
/* orig 	cns$sc_addr	quadword; */
#define CNS_Q_SC_ADDR		0x4D8
/* orig 	cns$sc_ctl	quadword; */
#define CNS_Q_SC_CTL		0x4E0
/* orig 	cns$bc_tag_addr	quadword; */
#define CNS_Q_BC_TAG_ADDR	0x4E8
/* orig 	cns$ei_stat	quadword; */
#define CNS_Q_EI_STAT		0x4F0
/* orig 	cns$ei_addr	quadword; */
#define CNS_Q_EI_ADDR		0x4F8
/* orig 	cns$fill_syn	quadword; */
#define CNS_Q_FILL_SYN		0x500
/* orig 	cns$ld_lock	quadword; */
#define CNS_Q_LD_LOCK		0x508
/* orig 	cns$bc_ctl	quadword;	// shadow of on chip bc_ctl  */
#define CNS_Q_BC_CTL		0x510
/* orig 	cns$pmctr_ctl   quadword;	// saved frequency select info for performance monitor counter */
#define CNS_Q_PM_CTL		0x518
/* orig 	cns$bc_config	quadword;	// shadow of on chip bc_config */
#define CNS_Q_BC_CFG            0x520

/* orig 	constant cns$size equals .;
 * orig
 * orig 	constant pal$impure_common_size equals (%x0200 +7) & %xfff8;
 * orig 	constant pal$impure_specific_size equals (.+7) & %xfff8;
 * orig 	constant cns$mchksize equals (.+7-#t) & %xfff8;
 * orig 	constant pal$logout_area	equals pal$impure_specific_size ;
 * orig end impure;
*/

/* This next set of stuff came from the old code ..pb */
#define CNS_Q_SROM_REV          0x528
#define CNS_Q_PROC_ID           0x530
#define CNS_Q_MEM_SIZE          0x538
#define CNS_Q_CYCLE_CNT         0x540
#define CNS_Q_SIGNATURE         0x548
#define CNS_Q_PROC_MASK         0x550
#define CNS_Q_SYSCTX            0x558



#define MACHINE_CHECK_CRD_BASE 0
#define MACHINE_CHECK_SIZE ((CNS_Q_SYSCTX + 7 - CNS_Q_MCHK) & 0xfff8)



/* orig
 * orig aggregate EV5PMCTRCTL_BITS structure fill prefix PMCTR_CTL$;
 * orig 	SPROCESS bitfield length 1 ;
 * orig 	FILL_0 bitfield length 3 fill tag $$;
 * orig 	FRQ2 bitfield length 2 ;
 * orig 	FRQ1 bitfield length 2 ;
 * orig 	FRQ0 bitfield length 2 ;
 * orig 	CTL2 bitfield length 2 ;
 * orig 	CTL1 bitfield length 2 ;
 * orig 	CTL0 bitfield length 2 ;
 * orig 	FILL_1 bitfield length 16 fill tag $$;
 * orig 	FILL_2 bitfield length 32 fill tag $$;
 * orig end EV5PMCTRCTL_BITS;
 * orig
 * orig end_module $pal_impure;
 * orig
 * orig module	$pal_logout;
 * orig
 * orig //
 * orig // Start definition of Corrected Error Frame
 * orig //
 */

/*
 * orig aggregate crd_logout struct prefix "" tag "";
 */

#ifdef SIMOS
#define pal_logout_area 0x600
#define mchk_crd_base  0
#endif

/* orig 	mchk$crd_flag		quadword; */
#define mchk_crd_flag 0
/* orig 	mchk$crd_offsets	quadword; */
#define mchk_crd_offsets 8
/* orig
 * orig 	// Pal-specific information	*/
#define mchk_crd_mchk_code 0x10
/* orig 	mchk$crd_mchk_code	quadword;
 * orig
 * orig 	// CPU-specific information
 * orig 	constant mchk$crd_cpu_base equals . ;
 * orig 	mchk$crd_ei_addr	quadword; */
#define mchk_crd_ei_addr 0x18
/* orig 	mchk$crd_fill_syn	quadword; */
#define mchk_crd_fill_syn 0x20
/* orig 	mchk$crd_ei_stat	quadword; */
#define mchk_crd_ei_stat 0x28
/* orig 	mchk$crd_isr		quadword; */
#define mchk_crd_isr 0x30

/*
 * Hacked up constants for the turbolaser build. Hope
 * this is moreless correct
 */

#define mchk_crd_whami   0x38
#define mchk_crd_tldev   0x40
#define mchk_crd_tlber   0x48
#define mchk_crd_tlesr0  0x50
#define mchk_crd_tlesr1  0x58
#define mchk_crd_tlesr2  0x60
#define mchk_crd_tlesr3  0x68
#define mchk_crd_rsvd    0x70


#ifdef SIMOS
/*
 * mchk area seems different for tlaser
 */

#define mchk_crd_size   0x80
#define mchk_mchk_base (mchk_crd_size)

#define mchk_tlber      0x0
#define mchk_tlepaerr   0x8
#define mchk_tlepderr   0x10
#define mchk_tlepmerr   0x18


#endif


/* orig
 * orig 	// System-specific information
 * orig 	constant mchk$crd_sys_base equals . ;
 * orig 	constant mchk$crd_size equals (.+7) & %xfff8;
 * orig
 * orig end crd_logout;
 * orig //
 * orig // Start definition of Machine check logout Frame
 * orig //
 * orig aggregate logout struct prefix "" tag "";
 * orig 	mchk$flag		quadword; */
/* orig 	mchk$offsets		quadword; */
/* orig
 * orig  // Pal-specific information
 * orig 	mchk$mchk_code		quadword; */
/*

 * orig 	mchk$pt	quadword dimension 24;
 * orig
 * orig  // CPU-specific information
 * orig 	constant mchk$cpu_base equals . ;
 * orig 	mchk$exc_addr		quadword;
 * orig 	mchk$exc_sum		quadword;
 * orig 	mchk$exc_mask		quadword;
 * orig 	mchk$pal_base		quadword;
 * orig 	mchk$isr		quadword;
 * orig 	mchk$icsr		quadword;
 * orig 	mchk$ic_perr_stat       quadword;
 * orig 	mchk$dc_perr_stat	quadword;
 * orig 	mchk$va		        quadword;
 * orig 	mchk$mm_stat		quadword;
 * orig 	mchk$sc_addr		quadword;
 * orig 	mchk$sc_stat		quadword;
 * orig 	mchk$bc_tag_addr	quadword;
 * orig 	mchk$ei_addr		quadword;
 * orig 	mchk$fill_syn		quadword;
 * orig 	mchk$ei_stat		quadword;
 * orig 	mchk$ld_lock		quadword;
 * orig
 * orig         // System-specific information
 * orig
 * orig 	constant mchk$sys_base equals . ;
 * orig 	mchk$sys_ipr1		quadword	; // Holder for system-specific stuff
 * orig
 * orig 	constant mchk$size equals (.+7) & %xfff8;
 * orig
 * orig
 * orig 	constant mchk$crd_base	equals 0 ;
 * orig 	constant mchk$mchk_base	equals mchk$crd_size ;
 * orig
 * orig
 * orig end logout;
 * orig
 * orig end_module $pal_logout;
*/




/* this is lingering in the old ladbx code but looks like it was from ev4 days.
 * This was 0x160 in the old days..pb
*/
#define LAF_K_SIZE         MACHINE_CHECK_SIZE
#endif
