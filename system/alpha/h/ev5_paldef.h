#ifndef EV5_PALDEF_INCLUDED
#define EV5_PALDEF_INCLUDED 1

// from ev5_paldef.mar from Lance's fetch directory...pb Nov/95
// some entries have been superceeded by the more recent evt_defs.h

// These are lower-caseified and have the $ signs (unnecessarily we now discover) removed.

// Note that at the bottom of this file is the version of ev5_defs.mar
// which is more recent than the top part of the file and contains overlapping information...pb Nov/95

//	.MACRO	PALDEFS
//	PALDEF_VER == 1		; Flag the version number of this file.
//	.ENDM
//
//
//	.MACRO	_HALT_CODES,_GBL
//	_DEFINI	HALT_CODES,_GBL

#define hlt_c_reset		0
#define hlt_c_hw_halt		1
#define hlt_c_ksp_inval		2
#define hlt_c_scbb_inval	3
#define hlt_c_ptbr_inval	4
#define hlt_c_sw_halt		5
#define hlt_c_dbl_mchk		6
#define hlt_c_mchk_from_pal	7
#define hlt_c_start		32
#define hlt_c_callback		33
#define hlt_c_mpstart		34
#define hlt_c_lfu_start		35

// halt codes above 255 reserved for platform specific errors
//	_DEFEND	HALT_CODES,_GBL,DEF
//	.ENDM
//	.MACRO	_MCHK_CODES,_GBL
//	_DEFINI	MCHK_CODES,_GBL

#define mchk_c_tperr			(64<<1)
#define mchk_c_tcperr			(65<<1)
#define mchk_c_herr			(66<<1)
#define mchk_c_ecc_c			(67<<1)
#define mchk_c_ecc_nc			(68<<1)
#define mchk_c_unknown		        (69<<1)
#define mchk_c_cacksoft			(70<<1)
#define mchk_c_bugcheck			(71<<1)
#define mchk_c_os_bugcheck		(72<<1)
#define mchk_c_dcperr			(73<<1)
#define mchk_c_icperr			(74<<1)
#define mchk_c_retryable_ird		(75<<1)
#define mchk_c_proc_hrd_error		(76<<1)
#define mchk_c_scperr			(77<<1)
#define mchk_c_bcperr			(78<<1)
//; mchk codes above 255 reserved for platform specific errors


#define mchk_c_read_nxm			(256<<1)
#define mchk_c_sys_hrd_error		(257<<1)
#define mchk_c_sys_ecc			(258<<1)

//	_DEFEND	MCHK_CODES,_GBL,DEF
//	.ENDM

//	.MACRO	_EV5_MM,_GBL
//	_DEFINI	_EV5_MM,_GBL

#define page_seg_size_bits	 10
#define page_offset_size_bits	 13
#define page_size_bytes		 8192
#define va_size_bits		 43
#define pa_size_bits		 45

//	_DEFEND	_EV5_MM,_GBL,DEF
//	.ENDM

//	.MACRO	_PALTEMP,_GBL
//	_DEFINI	_PALTEMP,_GBL

// replaced by ev5_defs.h #define pt0  		(0x140)
// replaced by ev5_defs.h #define pt1  		(0x141)
// replaced by ev5_defs.h #define pt2  		(0x142)
#define pt_entuna	(0x142)
// replaced by ev5_defs.h #define pt3	 	(0x143)
#define pt_impure	(0x143)
// replaced by ev5_defs.h #define pt4  		(0x144)
// replaced by ev5_defs.h #define pt5  		(0x145)
// replaced by ev5_defs.h #define pt6  		(0x146)
// replaced by ev5_defs.h #define pt7  		(0x147)
#define pt_entif	(0x147)
// replaced by ev5_defs.h #define pt8  		(0x148)
#define pt_intmask	(0x148)
// replaced by ev5_defs.h #define pt9  		(0x149)
#define pt_entsys	(0x149)
#define pt_ps  		(0x149)
// replaced by ev5_defs.h #define pt10  		(0x14a)
// replaced by ev5_defs.h #define pt11  		(0x14b)
#define pt_trap		(0x14b)
#define pt_entint	(0x14b)
// replaced by ev5_defs.h #define pt12  		(0x14c)
#define pt_entarith	(0x14c)
// replaced by ev5_defs.h #define pt13		(0x14d)
#define pt_sys0		(0x14d)
// replaced by ev5_defs.h #define pt14		(0x14e)
#define pt_sys1		(0x14e)
// replaced by ev5_defs.h #define pt15		(0x14f)
#define pt_sys2		(0x14f)
// replaced by ev5_defs.h #define pt16  		(0x150)
#define pt_whami	(0x150)
#define pt_mces		(0x150)
#define pt_misc 	(0x150)
// replaced by ev5_defs.h #define pt17  		(0x151)
#define pt_scc 		(0x151)
#define pt_sysval	(0x151)
// replaced by ev5_defs.h #define pt18  		(0x152)
#define pt_prbr		(0x152)
#define pt_usp		(0x152)
// replaced by ev5_defs.h #define pt19  		(0x153)
#define pt_ksp 		(0x153)
// replaced by ev5_defs.h #define pt20  		(0x154)
#define pt_ptbr		(0x154)
// replaced by ev5_defs.h #define pt21  		(0x155)
#define pt_vptbr	(0x155)
#define pt_entmm	(0x155)
// replaced by ev5_defs.h #define pt22  		(0x156)
#define pt_scbb		(0x156)
#define pt_kgp		(0x156)
// replaced by ev5_defs.h #define pt23  		(0x157)
#define pt_pcbb		(0x157)


#define pt_misc_v_switch 48
#define pt_misc_v_cm     56

//	_DEFEND	_PALTEMP,_GBL,DEF
//	.ENDM

//	.MACRO	_OSF_MMCSR_DEF,_GBL
//	_DEFINI	OSFMMCSR,_GBL

#define mmcsr_c_tnv		0
#define mmcsr_c_acv		1
#define mmcsr_c_for		2
#define mmcsr_c_foe		3
#define mmcsr_c_fow		4

//	_DEFEND	OSFMMCSR,_GBL,DEF
//	.ENDM

//	.MACRO	_MM_STAT_DEF,_GBL
//	_DEFINI	MMSTATDEF,_GBL

#define mm_stat_m_opcode  	(0x3F)
#define mm_stat_m_ra  		(0x1F)
#define evx_opc_sync	 	(0x18)
#define EVX_OPC_SYNC	 	(0x18)
#define evx_opc_hw_ld	 	(0x1B)

//	_DEFEND	MMSTATDEF,_GBL,DEF
//	.ENDM

//	.MACRO	_OSF_A0_DEF,_GBL
//	_DEFINI	OSFA0DEF,_GBL

#define osf_a0_bpt	  	(0x0)
#define osf_a0_bugchk	  	(0x1)
#define osf_a0_gentrap	  	(0x2)
#define osf_a0_fen	  	(0x3)
#define osf_a0_opdec	  	(0x4)

//	_DEFEND	OSFA0DEF,_GBL,DEF
//	.ENDM

//	.MACRO	_EV5_IPLDEF,_GBL
//	_DEFINI	EV5_IPLDEF,_GBL

#define ipl_machine_check	31
#define ipl_powerfail		30
#define ipl_perf_count		29
#define ipl_clock		22
#define ipl_interprocessor	22

//	_DEFEND	EV5_IPLDEF,_GBL,DEF
//	.ENDM

#endif
