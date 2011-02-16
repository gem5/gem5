/*
 * Copyright (c) 1995 The Hewlett-Packard Development Company
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

#ifndef EV5_DEFS_INCLUDED
#define EV5_DEFS_INCLUDED 1

// adapted from the version emailed to lance..pb Nov/95

//  In the definitions below, registers are annotated with one of the
//  following symbols:
//
//      RW - The register may be read and written
//   	RO - The register may only be read
//   	WO - The register may only be written
//
//  For RO and WO registers, all bits and fields within the register
//  are also read-only or write-only.  For RW registers, each bit or
//  field within the register is annotated with one of the following:
//
//   	RW - The bit/field may be read and written
//   	RO - The bit/field may be read; writes are ignored
//   	WO - The bit/field may be written; reads return an UNPREDICTABLE result
//   	WZ - The bit/field may be written; reads return a 0
//   	WC - The bit/field may be read; writes cause state to clear
//   	RC - The bit/field may be read, which also causes state to clear;
//           writes are ignored
//  Architecturally-defined (SRM) registers for EVMS

#define pt0 320
#define pt1 321
#define pt2 322
#define pt3 323
#define pt4 324
#define pt5 325
#define pt6 326
#define pt7 327
#define pt8 328
#define pt9 329
#define pt10 330
#define pt11 331
#define pt12 332
#define pt13 333
#define pt14 334
#define pt15 335
#define pt16 336
#define pt17 337
#define pt18 338
#define pt19 339
#define pt20 340
#define pt21 341
#define pt22 342
#define pt23 343
#define cbox_ipr_offset 16777200
#define sc_ctl 168
#define sc_stat 232
#define sc_addr 392
#define sc_addr_nm 392
#define sc_addr_fhm 392
#define bc_ctl 296
#define bc_config 456
#define ei_stat 360
#define ei_addr 328
#define fill_syn 104
#define bc_tag_addr 264
#define ld_lock 488
#define aster 266
#define astrr 265
#define exc_addr 267
#define exc_sum 268
#define exc_mask 269
#define hwint_clr 277
#define ic_flush_ctl 281
#define icperr_stat 282
#define ic_perr_stat 282
#define ic_row_map 283
#define icsr 280
#define ifault_va_form 274
#define intid 273
#define ipl 272
#define isr 256
#define itb_is 263
#define itb_asn 259
#define itb_ia 261
#define itb_iap 262
#define itb_pte 258
#define itb_pte_temp 260
#define itb_tag 257
#define ivptbr 275
#define pal_base 270
#define pmctr 284
// this is not the register ps .. pb #define ps 271
#define sirr 264
#define sl_txmit 278
#define sl_rcv 279
#define alt_mode 524
#define cc 525
#define cc_ctl 526
#define dc_flush 528
#define dcperr_stat 530
#define dc_test_ctl 531
#define dc_test_tag 532
#define dc_test_tag_temp 533
#define dtb_asn 512
#define dtb_cm 513
#define dtb_ia 522
#define dtb_iap 521
#define dtb_is 523
#define dtb_pte 515
#define dtb_pte_temp 516
#define dtb_tag 514
#define mcsr 527
#define dc_mode 534
#define maf_mode 535
#define mm_stat 517
#define mvptbr 520
#define va 518
#define va_form 519
#define ev5_srm__ps 0
#define ev5_srm__pc 0
#define ev5_srm__asten 0
#define ev5_srm__astsr 0
#define ev5_srm__ipir 0
#define ev5_srm__ipl 0
#define ev5_srm__mces 0
#define ev5_srm__pcbb 0
#define ev5_srm__prbr 0
#define ev5_srm__ptbr 0
#define ev5_srm__scbb 0
#define ev5_srm__sirr 0
#define ev5_srm__sisr 0
#define ev5_srm__tbchk 0
#define ev5_srm__tb1a 0
#define ev5_srm__tb1ap 0
#define ev5_srm__tb1ad 0
#define ev5_srm__tb1ai 0
#define ev5_srm__tbis 0
#define ev5_srm__ksp 0
#define ev5_srm__esp 0
#define ev5_srm__ssp 0
#define ev5_srm__usp 0
#define ev5_srm__vptb 0
#define ev5_srm__whami 0
#define ev5_srm__cc 0
#define ev5_srm__unq 0
//  processor-specific iprs.
#define ev5__sc_ctl 168
#define ev5__sc_stat 232
#define ev5__sc_addr 392
#define ev5__bc_ctl 296
#define ev5__bc_config 456
#define bc_config_k_size_1mb 1
#define bc_config_k_size_2mb 2
#define bc_config_k_size_4mb 3
#define bc_config_k_size_8mb 4
#define bc_config_k_size_16mb 5
#define bc_config_k_size_32mb 6
#define bc_config_k_size_64mb 7
#define ev5__ei_stat 360
#define ev5__ei_addr 328
#define ev5__fill_syn 104
#define ev5__bc_tag_addr 264
#define ev5__aster 266
#define ev5__astrr 265
#define ev5__exc_addr 267
#define exc_addr_v_pa 2
#define exc_addr_s_pa 62
#define ev5__exc_sum 268
#define ev5__exc_mask 269
#define ev5__hwint_clr 277
#define ev5__ic_flush_ctl 281
#define ev5__icperr_stat 282
#define ev5__ic_perr_stat 282
#define ev5__ic_row_map 283
#define ev5__icsr 280
#define ev5__ifault_va_form 274
#define ev5__ifault_va_form_nt 274
#define ifault_va_form_nt_v_vptb 30
#define ifault_va_form_nt_s_vptb 34
#define ev5__intid 273
#define ev5__ipl 272
#define ev5__itb_is 263
#define ev5__itb_asn 259
#define ev5__itb_ia 261
#define ev5__itb_iap 262
#define ev5__itb_pte 258
#define ev5__itb_pte_temp 260
#define ev5__itb_tag 257
#define ev5__ivptbr 275
#define ivptbr_v_vptb 30
#define ivptbr_s_vptb 34
#define ev5__pal_base 270
#define ev5__pmctr 284
#define ev5__ps 271
#define ev5__isr 256
#define ev5__sirr 264
#define ev5__sl_txmit 278
#define ev5__sl_rcv 279
#define ev5__alt_mode 524
#define ev5__cc 525
#define ev5__cc_ctl 526
#define ev5__dc_flush 528
#define ev5__dcperr_stat 530
#define ev5__dc_test_ctl 531
#define ev5__dc_test_tag 532
#define ev5__dc_test_tag_temp 533
#define ev5__dtb_asn 512
#define ev5__dtb_cm 513
#define ev5__dtb_ia 522
#define ev5__dtb_iap 521
#define ev5__dtb_is 523
#define ev5__dtb_pte 515
#define ev5__dtb_pte_temp 516
#define ev5__dtb_tag 514
#define ev5__mcsr 527
#define ev5__dc_mode 534
#define ev5__maf_mode 535
#define ev5__mm_stat 517
#define ev5__mvptbr 520
#define ev5__va 518
#define ev5__va_form 519
#define ev5__va_form_nt 519
#define va_form_nt_s_va 19
#define va_form_nt_v_vptb 30
#define va_form_nt_s_vptb 34
#define ev5s_ev5_def 10
#define ev5_def 0
//  cbox registers.
#define sc_ctl_v_sc_fhit 0
#define sc_ctl_v_sc_flush 1
#define sc_ctl_s_sc_tag_stat 6
#define sc_ctl_v_sc_tag_stat 2
#define sc_ctl_s_sc_fb_dp 4
#define sc_ctl_v_sc_fb_dp 8
#define sc_ctl_v_sc_blk_size 12
#define sc_ctl_s_sc_set_en 3
#define sc_ctl_v_sc_set_en 13
#define sc_ctl_s_sc_soft_repair 3
#define sc_ctl_v_sc_soft_repair 16
#define sc_stat_s_sc_tperr 3
#define sc_stat_v_sc_tperr 0
#define sc_stat_s_sc_dperr 8
#define sc_stat_v_sc_dperr 3
#define sc_stat_s_cbox_cmd 5
#define sc_stat_v_cbox_cmd 11
#define sc_stat_v_sc_scnd_err 16
#define sc_addr_fhm_v_sc_tag_parity 4
#define sc_addr_fhm_s_tag_stat_sb0 3
#define sc_addr_fhm_v_tag_stat_sb0 5
#define sc_addr_fhm_s_tag_stat_sb1 3
#define sc_addr_fhm_v_tag_stat_sb1 8
#define sc_addr_fhm_s_ow_mod0 2
#define sc_addr_fhm_v_ow_mod0 11
#define sc_addr_fhm_s_ow_mod1 2
#define sc_addr_fhm_v_ow_mod1 13
#define sc_addr_fhm_s_tag_lo 17
#define sc_addr_fhm_v_tag_lo 15
#define sc_addr_fhm_s_tag_hi 7
#define sc_addr_fhm_v_tag_hi 32
#define bc_ctl_v_bc_enabled 0
#define bc_ctl_v_alloc_cyc 1
#define bc_ctl_v_ei_opt_cmd 2
#define bc_ctl_v_ei_opt_cmd_mb 3
#define bc_ctl_v_corr_fill_dat 4
#define bc_ctl_v_vtm_first 5
#define bc_ctl_v_ei_ecc_or_parity 6
#define bc_ctl_v_bc_fhit 7
#define bc_ctl_s_bc_tag_stat 5
#define bc_ctl_v_bc_tag_stat 8
#define bc_ctl_s_bc_bad_dat 2
#define bc_ctl_v_bc_bad_dat 13
#define bc_ctl_v_ei_dis_err 15
#define bc_ctl_v_tl_pipe_latch 16
#define bc_ctl_s_bc_wave_pipe 2
#define bc_ctl_v_bc_wave_pipe 17
#define bc_ctl_s_pm_mux_sel 6
#define bc_ctl_v_pm_mux_sel 19
#define bc_ctl_v_dbg_mux_sel 25
#define bc_ctl_v_dis_baf_byp 26
#define bc_ctl_v_dis_sc_vic_buf 27
#define bc_ctl_v_dis_sys_addr_par 28
#define bc_ctl_v_read_dirty_cln_shr 29
#define bc_ctl_v_write_read_bubble 30
#define bc_ctl_v_bc_wave_pipe_2 31
#define bc_ctl_v_auto_dack 32
#define bc_ctl_v_dis_byte_word 33
#define bc_ctl_v_stclk_delay 34
#define bc_ctl_v_write_under_miss 35
#define bc_config_s_bc_size 3
#define bc_config_v_bc_size 0
#define bc_config_s_bc_rd_spd 4
#define bc_config_v_bc_rd_spd 4
#define bc_config_s_bc_wr_spd 4
#define bc_config_v_bc_wr_spd 8
#define bc_config_s_bc_rd_wr_spc 3
#define bc_config_v_bc_rd_wr_spc 12
#define bc_config_s_fill_we_offset 3
#define bc_config_v_fill_we_offset 16
#define bc_config_s_bc_we_ctl 9
#define bc_config_v_bc_we_ctl 20
//  cbox registers, continued
#define ei_stat_s_sys_id 4
#define ei_stat_v_sys_id 24
#define ei_stat_v_bc_tperr 28
#define ei_stat_v_bc_tc_perr 29
#define ei_stat_v_ei_es 30
#define ei_stat_v_cor_ecc_err 31
#define ei_stat_v_unc_ecc_err 32
#define ei_stat_v_ei_par_err 33
#define ei_stat_v_fil_ird 34
#define ei_stat_v_seo_hrd_err 35
//
#define bc_tag_addr_v_hit 12
#define bc_tag_addr_v_tagctl_p 13
#define bc_tag_addr_v_tagctl_d 14
#define bc_tag_addr_v_tagctl_s 15
#define bc_tag_addr_v_tagctl_v 16
#define bc_tag_addr_v_tag_p 17
#define bc_tag_addr_s_bc_tag 19
#define bc_tag_addr_v_bc_tag 20
//  ibox and icache registers.
#define aster_v_kar 0
#define aster_v_ear 1
#define aster_v_sar 2
#define aster_v_uar 3
#define astrr_v_kar 0
#define astrr_v_ear 1
#define astrr_v_sar 2
#define astrr_v_uar 3
#define exc_addr_v_pal 0
#define exc_sum_v_swc 10
#define exc_sum_v_inv 11
#define exc_sum_v_dze 12
#define exc_sum_v_fov 13
#define exc_sum_v_unf 14
#define exc_sum_v_ine 15
#define exc_sum_v_iov 16
#define hwint_clr_v_pc0c 27
#define hwint_clr_v_pc1c 28
#define hwint_clr_v_pc2c 29
#define hwint_clr_v_crdc 32
#define hwint_clr_v_slc 33
//  ibox and icache registers, continued
#define icperr_stat_v_dpe 11
#define icperr_stat_v_tpe 12
#define icperr_stat_v_tmr 13
#define ic_perr_stat_v_dpe 11
#define ic_perr_stat_v_tpe 12
#define ic_perr_stat_v_tmr 13
#define icsr_v_pma 8
#define icsr_v_pmp 9
#define icsr_v_byt 17
#define icsr_v_fmp 18
#define icsr_v_im0 20
#define icsr_v_im1 21
#define icsr_v_im2 22
#define icsr_v_im3 23
#define icsr_v_tmm 24
#define icsr_v_tmd 25
#define icsr_v_fpe 26
#define icsr_v_hwe 27
#define icsr_s_spe 2
#define icsr_v_spe 28
#define icsr_v_sde 30
#define icsr_v_crde 32
#define icsr_v_sle 33
#define icsr_v_fms 34
#define icsr_v_fbt 35
#define icsr_v_fbd 36
#define icsr_v_dbs 37
#define icsr_v_ista 38
#define icsr_v_tst 39
#define ifault_va_form_s_va 30
#define ifault_va_form_v_va 3
#define ifault_va_form_s_vptb 31
#define ifault_va_form_v_vptb 33
#define ifault_va_form_nt_s_va 19
#define ifault_va_form_nt_v_va 3
#define intid_s_intid 5
#define intid_v_intid 0
//  ibox and icache registers, continued
#define ipl_s_ipl 5
#define ipl_v_ipl 0
#define itb_is_s_va 30
#define itb_is_v_va 13
#define itb_asn_s_asn 7
#define itb_asn_v_asn 4
#define itb_pte_v_asm 4
#define itb_pte_s_gh 2
#define itb_pte_v_gh 5
#define itb_pte_v_kre 8
#define itb_pte_v_ere 9
#define itb_pte_v_sre 10
#define itb_pte_v_ure 11
#define itb_pte_s_pfn 27
#define itb_pte_v_pfn 32
#define itb_pte_temp_v_asm 13
#define itb_pte_temp_v_kre 18
#define itb_pte_temp_v_ere 19
#define itb_pte_temp_v_sre 20
#define itb_pte_temp_v_ure 21
#define itb_pte_temp_s_gh 3
#define itb_pte_temp_v_gh 29
#define itb_pte_temp_s_pfn 27
#define itb_pte_temp_v_pfn 32
//  ibox and icache registers, continued
#define itb_tag_s_va 30
#define itb_tag_v_va 13
#define pal_base_s_pal_base 26
#define pal_base_v_pal_base 14
#define pmctr_s_sel2 4
#define pmctr_v_sel2 0
#define pmctr_s_sel1 4
#define pmctr_v_sel1 4
#define pmctr_v_killk 8
#define pmctr_v_killp 9
#define pmctr_s_ctl2 2
#define pmctr_v_ctl2 10
#define pmctr_s_ctl1 2
#define pmctr_v_ctl1 12
#define pmctr_s_ctl0 2
#define pmctr_v_ctl0 14
#define pmctr_s_ctr2 14
#define pmctr_v_ctr2 16
#define pmctr_v_killu 30
#define pmctr_v_sel0 31
#define pmctr_s_ctr1 16
#define pmctr_v_ctr1 32
#define pmctr_s_ctr0 16
#define pmctr_v_ctr0 48
#define ps_v_cm0 3
#define ps_v_cm1 4
#define isr_s_astrr 4
#define isr_v_astrr 0
#define isr_s_sisr 15
#define isr_v_sisr 4
#define isr_v_atr 19
#define isr_v_i20 20
#define isr_v_i21 21
#define isr_v_i22 22
#define isr_v_i23 23
#define isr_v_pc0 27
#define isr_v_pc1 28
#define isr_v_pc2 29
#define isr_v_pfl 30
#define isr_v_mck 31
#define isr_v_crd 32
#define isr_v_sli 33
#define isr_v_hlt 34
#define sirr_s_sirr 15
#define sirr_v_sirr 4
//  ibox and icache registers, continued
#define sl_txmit_v_tmt 7
#define sl_rcv_v_rcv 6
//  mbox and dcache registers.
#define alt_mode_v_am0 3
#define alt_mode_v_am1 4
#define cc_ctl_v_cc_ena 32
#define dcperr_stat_v_seo 0
#define dcperr_stat_v_lock 1
#define dcperr_stat_v_dp0 2
#define dcperr_stat_v_dp1 3
#define dcperr_stat_v_tp0 4
#define dcperr_stat_v_tp1 5
//  the following two registers are used exclusively for test and diagnostics.
//  they should not be referenced in normal operation.
#define dc_test_ctl_v_bank0 0
#define dc_test_ctl_v_bank1 1
#define dc_test_ctl_v_fill_0 2
#define dc_test_ctl_s_index 10
#define dc_test_ctl_v_index 3
#define dc_test_ctl_s_fill_1 19
#define dc_test_ctl_v_fill_1 13
#define dc_test_ctl_s_fill_2 32
#define dc_test_ctl_v_fill_2 32
//  mbox and dcache registers, continued.
#define dc_test_tag_v_tag_par 2
#define dc_test_tag_v_ow0 11
#define dc_test_tag_v_ow1 12
#define dc_test_tag_s_tag 26
#define dc_test_tag_v_tag 13
#define dc_test_tag_temp_v_tag_par 2
#define dc_test_tag_temp_v_d0p0 3
#define dc_test_tag_temp_v_d0p1 4
#define dc_test_tag_temp_v_d1p0 5
#define dc_test_tag_temp_v_d1p1 6
#define dc_test_tag_temp_v_ow0 11
#define dc_test_tag_temp_v_ow1 12
#define dc_test_tag_temp_s_tag 26
#define dc_test_tag_temp_v_tag 13
#define dtb_asn_s_asn 7
#define dtb_asn_v_asn 57
#define dtb_cm_v_cm0 3
#define dtb_cm_v_cm1 4
#define dtbis_s_va0 30
#define dtbis_v_va0 13
#define dtb_pte_v_for 1
#define dtb_pte_v_fow 2
#define dtb_pte_v_asm 4
#define dtb_pte_s_gh 2
#define dtb_pte_v_gh 5
#define dtb_pte_v_kre 8
#define dtb_pte_v_ere 9
#define dtb_pte_v_sre 10
#define dtb_pte_v_ure 11
#define dtb_pte_v_kwe 12
#define dtb_pte_v_ewe 13
#define dtb_pte_v_swe 14
#define dtb_pte_v_uwe 15
#define dtb_pte_s_pfn 27
#define dtb_pte_v_pfn 32
//  mbox and dcache registers, continued.
#define dtb_pte_temp_v_for 0
#define dtb_pte_temp_v_fow 1
#define dtb_pte_temp_v_kre 2
#define dtb_pte_temp_v_ere 3
#define dtb_pte_temp_v_sre 4
#define dtb_pte_temp_v_ure 5
#define dtb_pte_temp_v_kwe 6
#define dtb_pte_temp_v_ewe 7
#define dtb_pte_temp_v_swe 8
#define dtb_pte_temp_v_uwe 9
#define dtb_pte_temp_v_asm 10
#define dtb_pte_temp_s_fill_0 2
#define dtb_pte_temp_v_fill_0 11
#define dtb_pte_temp_s_pfn 27
#define dtb_pte_temp_v_pfn 13
#define dtb_tag_s_va 30
#define dtb_tag_v_va 13
//  most mcsr bits are used for testability and diagnostics only.
//  for normal operation, they will be supported in the following configuration:
//  split_dcache = 1, maf_nomerge = 0, wb_flush_always = 0, wb_nomerge = 0,
//  dc_ena<1:0> = 1, dc_fhit = 0, dc_bad_parity = 0
#define mcsr_v_big_endian 0
#define mcsr_v_sp0 1
#define mcsr_v_sp1 2
#define mcsr_v_mbox_sel 3
#define mcsr_v_e_big_endian 4
#define mcsr_v_dbg_packet_sel 5
#define dc_mode_v_dc_ena 0
#define dc_mode_v_dc_fhit 1
#define dc_mode_v_dc_bad_parity 2
#define dc_mode_v_dc_perr_dis 3
#define dc_mode_v_dc_doa 4
#define maf_mode_v_maf_nomerge 0
#define maf_mode_v_wb_flush_always 1
#define maf_mode_v_wb_nomerge 2
#define maf_mode_v_io_nomerge 3
#define maf_mode_v_wb_cnt_disable 4
#define maf_mode_v_maf_arb_disable 5
#define maf_mode_v_dread_pending 6
#define maf_mode_v_wb_pending 7
//  mbox and dcache registers, continued.
#define mm_stat_v_wr 0
#define mm_stat_v_acv 1
#define mm_stat_v_for 2
#define mm_stat_v_fow 3
#define mm_stat_v_dtb_miss 4
#define mm_stat_v_bad_va 5
#define mm_stat_s_ra 5
#define mm_stat_v_ra 6
#define mm_stat_s_opcode 6
#define mm_stat_v_opcode 11
#define mvptbr_s_vptb 31
#define mvptbr_v_vptb 33
#define va_form_s_va 30
#define va_form_v_va 3
#define va_form_s_vptb 31
#define va_form_v_vptb 33
#define va_form_nt_s_va 19
#define va_form_nt_v_va 3
//.endm

#endif
