/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 * Authors: Jaidev Patwardhan
 */

#ifndef __ARCH_MIPS_PRA_CONSTANTS_HH__
#define __ARCH_MIPS_PRA_CONSTANTS_HH__

#include "arch/mips/types.hh"
//#include "config/full_system.hh"

namespace MipsISA
{
  // See MIPS32(R) Architecture Reference Manual Volume - III
  // This header file uses definitions from Revision 2.50

  // Index Status Register - CP0 Reg 0, Sel 0

  const unsigned Index_P_HI = 31;
  const unsigned Index_P_LO = 31;
  // Need to figure out how to put in the TLB specific bits here
  // For now, we assume that the entire length is used by the index field
  // In reality, Index_HI = N-1, where Ceiling(log2(TLB Entries))=N
  const unsigned Index_HI = 30;
  const unsigned Index_LO = 0;

  // CP0 Reg 0, Sel 1-3 are MT registers, see mt_constants.hh

  // Random Register - CP0 Reg 1, Sel 0
  // This has a problem similar to the Index_HI fields. We'll keep both consistent at 30 for now
  const unsigned Random_HI = 30;
  const unsigned Random_LO = 0;

  // EntryLo0 - CP0 Reg2, Sel 0  - Table 8-6, ARM Vol-3
  const unsigned EntryLo0_Fill_HI = 31; // See Table 8-8, ARM Vol III
  const unsigned EntryLo0_Fill_LO = 30;
  const unsigned EntryLo0_PFN_HI  = 29; //PFN defines the Page Frame Number (see Table 8-7, ARM Vol III)
  const unsigned EntryLo0_PFN_LO  =  6;
  const unsigned EntryLo0_C_HI    =  5; // Coherency attribute of a Page (see Table 8-8, ARM Vol III)
  const unsigned EntryLo0_C_LO    =  3;
  const unsigned EntryLo0_D       =  2; // Dirty Bit, if D=1, page is writable. If D=0, a write causes a TLB Modified Exception
  const unsigned EntryLo0_V       =  1; // Valid Bit
  const unsigned EntryLo0_G       =  0; // Global Bit. From the ARM Vol-III, Table 8-5:
                                        // On a TLB write, the logical AND of the G bits from EntryLo0 and EntryLo1
                                        // becomes the G bit in the TLB entry. If the TLB entry G bit is 1, ASID comparisons are
                                        // ignored during TLB matches. On a read from a TLB entry, the G bits of both Lo0 and Lo1
                                        // reflect the state of the TLB G bit.

  // EntryLo1 - CP0 Reg3, Sel 0
  const unsigned EntryLo1_G       =  0;
  const unsigned EntryLo1_V       =  1; // Valid Bit
  const unsigned EntryLo1_D       =  2; // Dirty Bit, if D=1, page is writable. If D=0, a write causes a TLB Modified Exception
  const unsigned EntryLo1_C_HI    =  5; // Coherency attribute of a Page (see Table 8-8, ARM Vol III)
  const unsigned EntryLo1_C_LO    =  3;
  const unsigned EntryLo1_PFN_HI  = 29; //PFN defines the Page Frame Number (see Table 8-7, ARM Vol III)
  const unsigned EntryLo1_PFN_LO  =  6;
  const unsigned EntryLo1_Fill_LO = 30;
  const unsigned EntryLo1_Fill_HI = 31; // See Table 8-8, ARM Vol III


  // Context Register - CP0 Reg 4, Sel 0
  const unsigned Context_PTEBase_HI = 31; // Used by the OS to point into current PTE array
  const unsigned Context_PTEBase_LO = 23;
  const unsigned Context_BadVPN2_HI = 22; // This is written by hardware on a TLB exception. Contains bits 31-13 of the
  const unsigned Context_BadVPN2_LO = 4;  // virtual address
  // Bits 3-0 are zeros

  // PageMask Register - CP0 Reg 5, Sel 0
  // Bits 31-29 are 0
  const unsigned PageMask_Mask_HI = 28; // (Table 8-10, ARM Vol-III) The Mask field is a bit mask in which a "1" indicates that
  const unsigned PageMask_Mask_LO = 13; // the corresponding bit of the virtual address should not participate in the TLB match
  const unsigned PageMask_MaskX_HI = 12; // See Table 8-10, ARM Vol-III
  const unsigned PageMask_MaskX_LO = 11;
  // Bits 10-0 are zero


  // PageGrain Register - CP0 Reg 5, Sel 1
  const unsigned PageGrain_ASE_UP_HI = 31; // ASE specific bits (SmartMIPS)
  const unsigned PageGrain_ASE_UP_LO = 30; //
  const unsigned PageGrain_ELPA = 29; // Used to enable support for large physical addresses in MIPS64 processors, unused in MIPS32
  const unsigned PageGrain_ESP = 28; // Enables support for 1KB pages (1==enabled,0==disabled), See ARM Vol-III, Table 8-12
  const unsigned PageGrain_ASE_DN_HI = 12;
  const unsigned PageGrain_ASE_DN_LO = 8;
  // Bits 27-13, 7-0 are zeros

  // Wired Register - CPO Reg 6, Sel 0
  // See note on Index register (CP0, Sel0) above
  const unsigned Wired_HI = 30;
  const unsigned Wired_LO = 0;


  // HWREna Register - CP0 Reg 7, Sel 0
  const unsigned HWREna_IMPL_HI = 31; // These bits enable access to implementation dependent hardware registers 31
  const unsigned HWREna_IMPL_LO = 30; // and 30
  const unsigned HWREna_Mask_HI = 3; // Each bit enables access to a particular hardware register. If bit 'n' is 1, HW Reg n is accessible
  const unsigned HWREna_Mask_LO = 0; // See the RDHWR instruction for more details


  // BadVAddr Register - CP0 Reg 8, Sel 0
  const unsigned BadVAddr_HI = 31;
  const unsigned BadVAddr_LO = 0;

  // Count Register - CP0 Reg 9, Sel 0
  const unsigned Count_HI = 31;
  const unsigned Count_LO = 0;

  // EntryHI Register - CP0 Reg 10, Sel 0
  const unsigned Entry_HI_VPN2_HI = 31;  // This field is written by hardware on a TLB exception or on a TLB read
  const unsigned Entry_HI_VPN2_LO = 13;  // and is written by software before a TLB write
  const unsigned Entry_HI_VPN2X_HI = 12; // Extension to support 1KB pages
  const unsigned Entry_HI_VPN2X_LO = 11;
  const unsigned Entry_HI_ASID_HI = 7; // Address space identifier
  const unsigned Entry_HI_ASID_LO = 0;

  // Compare Register - CP0 Reg 11, Sel 0
  const unsigned Compare_HI = 31; // Used in conjunction with Count
  const unsigned Compare_LO = 0;

  // Status Register - CP Reg 12, Sel 0
  const unsigned Status_IE_HI = 0;
  const unsigned Status_IE_LO = 0;

  const unsigned Status_EXL_HI = 1;
  const unsigned Status_EXL_LO = 1;
  const unsigned Status_ERL_HI = 2;
  const unsigned Status_ERL_LO = 2;
  const unsigned Status_R0 = 3;
  const unsigned Status_UM = 4;
  const unsigned Status_KSU_HI = 4;  // R0 and UM are also aliased as KSU
  const unsigned Status_KSU_LO = 3;
  const unsigned Status_UX = 5;
  const unsigned Status_SX = 6;
  const unsigned Status_KX = 7;
  const unsigned Status_IM0 = 8;
  const unsigned Status_IM1 = 9;
  const unsigned Status_IM2 = 10;
  const unsigned Status_IM3 = 11;
  const unsigned Status_IM4 = 12;
  const unsigned Status_IM5 = 13;
  const unsigned Status_IM6 = 14;
  const unsigned Status_IM7 = 15;
  const unsigned Status_IPL_HI = 15;  // IM7..IM2 are also aliased as IPL
  const unsigned Status_IPL_LO = 10;
  const unsigned Status_IMPL_HI = 17;
  const unsigned Status_IMPL_LO = 16;
  const unsigned Status_NMI = 19;
  const unsigned Status_SR = 20;
  const unsigned Status_TS = 21;
  const unsigned Status_BEV = 22;
  const unsigned Status_PX = 23;
  const unsigned Status_MX = 24;
  const unsigned Status_RE = 25;
  const unsigned Status_FR = 26;
  const unsigned Status_RP = 27;
  const unsigned Status_CU3_HI = 31;
  const unsigned Status_CU3_LO = 31;
  const unsigned Status_CU2_HI = 30;
  const unsigned Status_CU2_LO = 30;
  const unsigned Status_CU1_HI = 29;
  const unsigned Status_CU1_LO = 29;
  const unsigned Status_CU0_HI = 28;
  const unsigned Status_CU0_LO = 28;

  // IntCtl Register - CP0 Reg 12, Sel 1
  // Interrupt System status and control
  const unsigned IntCtl_IPTI_HI = 31;
  const unsigned IntCtl_IPTI_LO = 29;
  const unsigned IntCtl_IPPCI_HI = 28;
  const unsigned IntCtl_IPPCI_LO = 26;
  const unsigned IntCtl_VS_HI = 9;
  const unsigned IntCtl_VS_LO = 5;
  // Bits 26-10, 4-0 are zeros

  // SRSCtl Register - CP0 Reg 12, Sel 2
  // Shadow Register Set Status and Control
  const unsigned SRSCtl_HSS_HI=29; // Highest Shadow Set
  const unsigned SRSCtl_HSS_LO=26;
  const unsigned SRSCtl_EICSS_HI=21; //EIC interrupt mode shadow set
  const unsigned SRSCtl_EICSS_LO=18;
  const unsigned SRSCtl_ESS_HI=15; // Exception Shadow Set
  const unsigned SRSCtl_ESS_LO=12;
  const unsigned SRSCtl_PSS_HI=9; // Previous Shadow Set
  const unsigned SRSCtl_PSS_LO=6;
  const unsigned SRSCtl_CSS_HI=3; // Current Shadow Set
  const unsigned SRSCtl_CSS_LO=0;

  // SRSMap Register - CP0 Reg 12, Sel 3
  // Shadow Set IPL mapping
  const unsigned SRSMap_SSV7_HI = 31; // Shadow sets for particular vector numbers (7..0)
  const unsigned SRSMap_SSV7_LO = 28;
  const unsigned SRSMap_SSV6_HI = 27;
  const unsigned SRSMap_SSV6_LO = 24;
  const unsigned SRSMap_SSV5_HI = 23;
  const unsigned SRSMap_SSV5_LO = 20;
  const unsigned SRSMap_SSV4_HI = 19;
  const unsigned SRSMap_SSV4_LO = 16;
  const unsigned SRSMap_SSV3_HI = 15;
  const unsigned SRSMap_SSV3_LO = 12;
  const unsigned SRSMap_SSV2_HI = 11;
  const unsigned SRSMap_SSV2_LO = 8;
  const unsigned SRSMap_SSV1_HI = 7;
  const unsigned SRSMap_SSV1_LO = 4;
  const unsigned SRSMap_SSV0_HI = 3;
  const unsigned SRSMap_SSV0_LO = 20;

  // Cause Register - CP0 Reg 13, Sel 0
  const unsigned Cause_BD = 31;
  const unsigned Cause_TI = 30;
  const unsigned Cause_CE_HI = 29;
  const unsigned Cause_CE_LO = 28;
  const unsigned Cause_DC = 27;
  const unsigned Cause_PCI = 26;
  const unsigned Cause_IV = 24;
  const unsigned Cause_WP = 23;
  const unsigned Cause_RIPL_HI = 15; // The individual bits of RIPL are also available as IP7..IP5
  const unsigned Cause_RIPL_LO = 10;
  const unsigned Cause_IP7 = 15;
  const unsigned Cause_IP6 = 14;
  const unsigned Cause_IP5 = 13;
  const unsigned Cause_IP4 = 12;
  const unsigned Cause_IP3 = 11;
  const unsigned Cause_IP2 = 10;
  const unsigned Cause_IP1 = 9;
  const unsigned Cause_IP0 = 8;
  const unsigned Cause_EXCCODE_HI = 6;
  const unsigned Cause_EXCCODE_LO = 2;
  // All intermediate undefined bits must be ZERO


  // EPC Register - CP0 Reg 14, Sel 0
  // Exception Program Counter
  const unsigned EPC_HI = 31;
  const unsigned EPC_LO = 0;

  // PRId Register - CP0 Reg 15, Sel 0
  // Processor Identification register
  const unsigned PRIdCoOp_HI = 31;
  const unsigned PRIdCoOp_LO = 24;
  const unsigned PRIdCoID_HI = 23;
  const unsigned PRIdCoID_LO = 16;
  const unsigned PRIdProc_ID_HI = 15;
  const unsigned PRIdProc_ID_LO = 8;
  const unsigned PRIdRev_HI = 7;
  const unsigned PRIdRev_LO = 0;


  // EBase Register - CP0 Reg 15, Sel 1
  // Exception Base Register
  const unsigned EBase_MSB = 31; // MUST BE = 1
  const unsigned EBase_EXCEPTION_Base_HI = 29;
  const unsigned EBase_EXCEPTION_Base_LO = 12;
  const unsigned EBase_CPUNum_HI = 9;
  const unsigned EBase_CPUNum_LO = 0;
  // Undefined bits must be zero

  // Config Register - CP0 Reg 16, Sel 0
  const unsigned Config_M = 31;
  const unsigned Config_K23_HI = 30;
  const unsigned Config_K23_LO = 28;
  const unsigned Config_KU_HI = 27;
  const unsigned Config_KU_LO = 25;
  const unsigned Config_IMPL_HI = 24;
  const unsigned Config_IMPL_LO = 16;
  const unsigned Config_BE = 15;
  const unsigned Config_AT_HI = 14;
  const unsigned Config_AT_LO = 13;
  const unsigned Config_AR_HI = 12;
  const unsigned Config_AR_LO = 10;
  const unsigned Config_MT_HI = 9;
  const unsigned Config_MT_LO = 7;
  const unsigned Config_VI = 3;
  const unsigned Config_K0_HI = 2;
  const unsigned Config_K0_LO = 0;

  // Config1 Register - CP0 Reg 16, Sel 1
  const unsigned Config1_M = 31;
  const unsigned Config1_MMUSize_HI = 30;
  const unsigned Config1_MMUSize_LO = 25;
  const unsigned Config1_IS_HI = 24;
  const unsigned Config1_IS_LO = 22;
  const unsigned Config1_IL_HI = 21;
  const unsigned Config1_IL_LO = 19;
  const unsigned Config1_IA_HI = 18;
  const unsigned Config1_IA_LO = 16;
  const unsigned Config1_DS_HI = 15;
  const unsigned Config1_DS_LO = 13;
  const unsigned Config1_DL_HI = 12;
  const unsigned Config1_DL_LO = 10;
  const unsigned Config1_DA_HI = 9;
  const unsigned Config1_DA_LO = 7;
  const unsigned Config1_C2 = 6;
  const unsigned Config1_MD = 5;
  const unsigned Config1_PC = 4;
  const unsigned Config1_WR = 3;
  const unsigned Config1_CA = 2;
  const unsigned Config1_EP = 1;
  const unsigned Config1_FP = 0;


  // Config2 Register - CP0 Reg 16, Sel 2
  const unsigned Config2_M = 31;
  const unsigned Config2_TU_HI = 30;
  const unsigned Config2_TU_LO = 28;
  const unsigned Config2_TS_HI = 27;
  const unsigned Config2_TS_LO = 24;
  const unsigned Config2_TL_HI = 23;
  const unsigned Config2_TL_LO = 20;
  const unsigned Config2_TA_HI = 19;
  const unsigned Config2_TA_LO = 16;
  const unsigned Config2_SU_HI = 15;
  const unsigned Config2_SU_LO = 12;
  const unsigned Config2_SS_HI = 11;
  const unsigned Config2_SS_LO = 8;
  const unsigned Config2_SL_HI = 7;
  const unsigned Config2_SL_LO = 4;
  const unsigned Config2_SA_HI = 3;
  const unsigned Config2_SA_LO = 0;

  // Config3 Register - CP0 Reg 16, Sel 3
  const unsigned Config3_M = 31;
  const unsigned Config3_DSPP = 10;
  const unsigned Config3_LPA=7;
  const unsigned Config3_VEIC=6;
  const unsigned Config3_VINT=5;
  const unsigned Config3_SP=4;
  const unsigned Config3_MT=2;
  const unsigned Config3_SM=1;
  const unsigned Config3_TL=0;


  // LLAddr Register - CP0 Reg 17, Sel 0
  // Load Linked Address (Physical)
  const unsigned LLAddr_PAddr_HI = 31;
  const unsigned LLAddr_PAddr_LO = 0;



  // WatchLo Register - CP0 Reg 18, Sel 0-n
  // See WatchHi to determine how many pairs of these registers are available
  const unsigned WatchLo_VAddr_HI = 31;
  const unsigned WatchLo_VAddr_LO = 3;
  const unsigned WatchLo_I = 2;
  const unsigned WatchLo_R = 1;
  const unsigned WatchLo_W = 0;


  // WatchHi Register - CP0 Reg 19, Sel 0-n
  const unsigned WatchHi_M = 31; // If M = 1, another pair of WatchHi/Lo registers exist
  const unsigned WatchHi_G = 30;
  const unsigned WatchHi_ASID_HI = 23;
  const unsigned WatchHi_ASID_LO = 16;
  const unsigned WatchHi_Mask_HI = 11;
  const unsigned WatchHi_Mask_LO = 3;
  const unsigned WatchHi_I = 2;
  const unsigned WatchHi_R = 1;
  const unsigned WatchHi_W = 0;

  // Debug Register - CP0 Reg 23, Sel 0

  // TraceControl Register - CP0 Reg 23, Sel 1
  // TraceControl2 Register - CP0 Reg 23, Sel 2
  // UserTraceData Register - CP0 Reg 23, Sel 3
  // TraceBPC Register - CP0 Reg 23, Sel 4
  // DEPC Register - CP0 Reg 24, Sel 0


  // PerfCnt Register - CP0 Reg 25, Sel 0-n
  // Each Perf. counter that exists is mapped onto even-odd select pairs of Reg 25
  // Even values are control registers, odd values are the actual counter
  // The format for the control reg is:
  const unsigned PerfCntCtl_M = 31; // Is there another pair of perf counter registers?
  const unsigned PerfCntCtl_W = 30;
  const unsigned PerfCntCtl_Event_HI = 10;
  const unsigned PerfCntCtl_Event_LO = 5;
  const unsigned PerfCntCtl_IE = 4;
  const unsigned PerfCntCtl_U = 3;
  const unsigned PerfCntCtl_S = 2;
  const unsigned PerfCntCtl_K = 1;
  const unsigned PerfCntCtl_EXL = 0;

  // The format for the counter is a 32-bit value (or 64-bit for MIPS64)
  const unsigned PerfCnt_Count_HI = 31;
  const unsigned PerfCnt_Count_LO = 0;

  // ErrCtl Register - CP0 Reg 26, Sel 0
  // This is implementation dependent, not defined by the ISA

  // CacheErr Register - CP0 Reg 27, Sel 0
  // NOTE: Page 65 of the ARM, Volume-III indicates that there are four sel. values (0-3)
  // used by the CacheErr registers. However, on page 134, only one sel value is shown
  const unsigned Cache_Err_ER = 31;
  const unsigned Cache_Err_EC = 30;
  const unsigned Cache_Err_ED = 29;
  const unsigned Cache_Err_ET = 28;
  const unsigned Cache_Err_ES = 27;
  const unsigned Cache_Err_EE = 26;
  const unsigned Cache_Err_EB = 25;
  const unsigned Cache_Err_IMPL_HI = 24;
  const unsigned Cache_Err_IMPL_LO = 22;
  const unsigned Cache_Err_Index_HI = 21;
  const unsigned Cache_Err_Index_LO = 0;

  // TagLo Register - CP0 Reg 28 - Even Selects (0,2)
  const unsigned TagLo_PTagLo_HI = 31;
  const unsigned TagLo_PTagLo_LO = 8;
  const unsigned TagLo_PState_HI = 7;
  const unsigned TagLo_PState_LO = 6;
  const unsigned TagLo_L = 5;
  const unsigned TagLo_IMPL_HI = 4;
  const unsigned TagLo_IMPL_LO = 3;
  const unsigned TagLo_P = 0;
  // undefined bits must be written 0


  // DataLo Register - CP0 Reg 28 - Odd Selects (1,3)
  const unsigned DataLo_HI = 31;
  const unsigned DataLo_LO = 0;

  // TagHi Register - CP0 Reg 29 - Even Selects (0,2)
  // Not defined by the architecture

  // DataHi Register - CP0 Reg 29 - Odd Selects (1,3)
  const unsigned DataHi_HI = 31;
  const unsigned DataHi_LO = 0;


  // ErrorEPC - CP0 Reg 30, Sel 0
  const unsigned ErrorPC_HI = 31;
  const unsigned ErrorPC_LO = 0;

  // DESAVE - CP0 Reg 31, Sel 0





} // namespace MipsISA

#endif
