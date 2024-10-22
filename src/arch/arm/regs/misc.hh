/*
 * Copyright (c) 2010-2023 Arm Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2009 The Regents of The University of Michigan
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

#ifndef __ARCH_ARM_REGS_MISC_HH__
#define __ARCH_ARM_REGS_MISC_HH__

#include <array>
#include <bitset>
#include <optional>
#include <tuple>

#include "arch/arm/regs/misc_types.hh"
#include "arch/arm/types.hh"
#include "base/compiler.hh"
#include "cpu/reg_class.hh"
#include "debug/MiscRegs.hh"
#include "dev/arm/generic_timer_miscregs_types.hh"

namespace gem5
{

class ArmSystem;
class ThreadContext;
class MiscRegOp64;

namespace ArmISA
{
    enum MiscRegIndex
    {
        MISCREG_CPSR = 0,
        MISCREG_SPSR,
        MISCREG_SPSR_FIQ,
        MISCREG_SPSR_IRQ,
        MISCREG_SPSR_SVC,
        MISCREG_SPSR_MON,
        MISCREG_SPSR_ABT,
        MISCREG_SPSR_HYP,
        MISCREG_SPSR_UND,
        MISCREG_ELR_HYP,
        MISCREG_FPSID,
        MISCREG_FPSCR,
        MISCREG_MVFR1,
        MISCREG_MVFR0,
        MISCREG_FPEXC,

        // Helper registers
        MISCREG_CPSR_MODE,
        MISCREG_CPSR_Q,
        MISCREG_FPSCR_EXC,
        MISCREG_FPSCR_QC,
        MISCREG_LOCKADDR,
        MISCREG_LOCKFLAG,
        MISCREG_PRRR_MAIR0,
        MISCREG_PRRR_MAIR0_NS,
        MISCREG_PRRR_MAIR0_S,
        MISCREG_NMRR_MAIR1,
        MISCREG_NMRR_MAIR1_NS,
        MISCREG_NMRR_MAIR1_S,
        MISCREG_PMXEVTYPER_PMCCFILTR,
        MISCREG_SEV_MAILBOX,
        MISCREG_TLBINEEDSYNC,

        // AArch32 CP14 registers (debug/trace control)
        MISCREG_DBGDIDR,
        MISCREG_DBGDSCRint,
        MISCREG_DBGDCCINT,
        MISCREG_DBGDTRTXint,
        MISCREG_DBGDTRRXint,
        MISCREG_DBGWFAR,
        MISCREG_DBGVCR,
        MISCREG_DBGDTRRXext,
        MISCREG_DBGDSCRext,
        MISCREG_DBGDTRTXext,
        MISCREG_DBGOSECCR,
        MISCREG_DBGBVR0,
        MISCREG_DBGBVR1,
        MISCREG_DBGBVR2,
        MISCREG_DBGBVR3,
        MISCREG_DBGBVR4,
        MISCREG_DBGBVR5,
        MISCREG_DBGBVR6,
        MISCREG_DBGBVR7,
        MISCREG_DBGBVR8,
        MISCREG_DBGBVR9,
        MISCREG_DBGBVR10,
        MISCREG_DBGBVR11,
        MISCREG_DBGBVR12,
        MISCREG_DBGBVR13,
        MISCREG_DBGBVR14,
        MISCREG_DBGBVR15,
        MISCREG_DBGBCR0,
        MISCREG_DBGBCR1,
        MISCREG_DBGBCR2,
        MISCREG_DBGBCR3,
        MISCREG_DBGBCR4,
        MISCREG_DBGBCR5,
        MISCREG_DBGBCR6,
        MISCREG_DBGBCR7,
        MISCREG_DBGBCR8,
        MISCREG_DBGBCR9,
        MISCREG_DBGBCR10,
        MISCREG_DBGBCR11,
        MISCREG_DBGBCR12,
        MISCREG_DBGBCR13,
        MISCREG_DBGBCR14,
        MISCREG_DBGBCR15,
        MISCREG_DBGWVR0,
        MISCREG_DBGWVR1,
        MISCREG_DBGWVR2,
        MISCREG_DBGWVR3,
        MISCREG_DBGWVR4,
        MISCREG_DBGWVR5,
        MISCREG_DBGWVR6,
        MISCREG_DBGWVR7,
        MISCREG_DBGWVR8,
        MISCREG_DBGWVR9,
        MISCREG_DBGWVR10,
        MISCREG_DBGWVR11,
        MISCREG_DBGWVR12,
        MISCREG_DBGWVR13,
        MISCREG_DBGWVR14,
        MISCREG_DBGWVR15,
        MISCREG_DBGWCR0,
        MISCREG_DBGWCR1,
        MISCREG_DBGWCR2,
        MISCREG_DBGWCR3,
        MISCREG_DBGWCR4,
        MISCREG_DBGWCR5,
        MISCREG_DBGWCR6,
        MISCREG_DBGWCR7,
        MISCREG_DBGWCR8,
        MISCREG_DBGWCR9,
        MISCREG_DBGWCR10,
        MISCREG_DBGWCR11,
        MISCREG_DBGWCR12,
        MISCREG_DBGWCR13,
        MISCREG_DBGWCR14,
        MISCREG_DBGWCR15,
        MISCREG_DBGDRAR,
        MISCREG_DBGBXVR0,
        MISCREG_DBGBXVR1,
        MISCREG_DBGBXVR2,
        MISCREG_DBGBXVR3,
        MISCREG_DBGBXVR4,
        MISCREG_DBGBXVR5,
        MISCREG_DBGBXVR6,
        MISCREG_DBGBXVR7,
        MISCREG_DBGBXVR8,
        MISCREG_DBGBXVR9,
        MISCREG_DBGBXVR10,
        MISCREG_DBGBXVR11,
        MISCREG_DBGBXVR12,
        MISCREG_DBGBXVR13,
        MISCREG_DBGBXVR14,
        MISCREG_DBGBXVR15,
        MISCREG_DBGOSLAR,
        MISCREG_DBGOSLSR,
        MISCREG_DBGOSDLR,
        MISCREG_DBGPRCR,
        MISCREG_DBGDSAR,
        MISCREG_DBGCLAIMSET,
        MISCREG_DBGCLAIMCLR,
        MISCREG_DBGAUTHSTATUS,
        MISCREG_DBGDEVID2,
        MISCREG_DBGDEVID1,
        MISCREG_DBGDEVID0,
        MISCREG_TEECR,  // not in ARM DDI 0487A.b+
        MISCREG_JIDR,
        MISCREG_TEEHBR, // not in ARM DDI 0487A.b+
        MISCREG_JOSCR,
        MISCREG_JMCR,

        // AArch32 CP15 registers (system control)
        MISCREG_MIDR,
        MISCREG_CTR,
        MISCREG_TCMTR,
        MISCREG_TLBTR,
        MISCREG_MPIDR,
        MISCREG_REVIDR,
        MISCREG_ID_PFR0,
        MISCREG_ID_PFR1,
        MISCREG_ID_DFR0,
        MISCREG_ID_AFR0,
        MISCREG_ID_MMFR0,
        MISCREG_ID_MMFR1,
        MISCREG_ID_MMFR2,
        MISCREG_ID_MMFR3,
        MISCREG_ID_MMFR4,
        MISCREG_ID_ISAR0,
        MISCREG_ID_ISAR1,
        MISCREG_ID_ISAR2,
        MISCREG_ID_ISAR3,
        MISCREG_ID_ISAR4,
        MISCREG_ID_ISAR5,
        MISCREG_ID_ISAR6,
        MISCREG_CCSIDR,
        MISCREG_CLIDR,
        MISCREG_AIDR,
        MISCREG_CSSELR,
        MISCREG_CSSELR_NS,
        MISCREG_CSSELR_S,
        MISCREG_VPIDR,
        MISCREG_VMPIDR,
        MISCREG_SCTLR,
        MISCREG_SCTLR_NS,
        MISCREG_SCTLR_S,
        MISCREG_ACTLR,
        MISCREG_ACTLR_NS,
        MISCREG_ACTLR_S,
        MISCREG_CPACR,
        MISCREG_SDCR,
        MISCREG_SCR,
        MISCREG_SDER,
        MISCREG_NSACR,
        MISCREG_HSCTLR,
        MISCREG_HACTLR,
        MISCREG_HCR,
        MISCREG_HCR2,
        MISCREG_HDCR,
        MISCREG_HCPTR,
        MISCREG_HSTR,
        MISCREG_HACR,
        MISCREG_TTBR0,
        MISCREG_TTBR0_NS,
        MISCREG_TTBR0_S,
        MISCREG_TTBR1,
        MISCREG_TTBR1_NS,
        MISCREG_TTBR1_S,
        MISCREG_TTBCR,
        MISCREG_TTBCR_NS,
        MISCREG_TTBCR_S,
        MISCREG_HTCR,
        MISCREG_VTCR,
        MISCREG_DACR,
        MISCREG_DACR_NS,
        MISCREG_DACR_S,
        MISCREG_DFSR,
        MISCREG_DFSR_NS,
        MISCREG_DFSR_S,
        MISCREG_IFSR,
        MISCREG_IFSR_NS,
        MISCREG_IFSR_S,
        MISCREG_ADFSR,
        MISCREG_ADFSR_NS,
        MISCREG_ADFSR_S,
        MISCREG_AIFSR,
        MISCREG_AIFSR_NS,
        MISCREG_AIFSR_S,
        MISCREG_HADFSR,
        MISCREG_HAIFSR,
        MISCREG_HSR,
        MISCREG_DFAR,
        MISCREG_DFAR_NS,
        MISCREG_DFAR_S,
        MISCREG_IFAR,
        MISCREG_IFAR_NS,
        MISCREG_IFAR_S,
        MISCREG_HDFAR,
        MISCREG_HIFAR,
        MISCREG_HPFAR,
        MISCREG_ICIALLUIS,
        MISCREG_BPIALLIS,
        MISCREG_PAR,
        MISCREG_PAR_NS,
        MISCREG_PAR_S,
        MISCREG_ICIALLU,
        MISCREG_ICIMVAU,
        MISCREG_CP15ISB,
        MISCREG_BPIALL,
        MISCREG_BPIMVA,
        MISCREG_DCIMVAC,
        MISCREG_DCISW,
        MISCREG_ATS1CPR,
        MISCREG_ATS1CPW,
        MISCREG_ATS1CUR,
        MISCREG_ATS1CUW,
        MISCREG_ATS12NSOPR,
        MISCREG_ATS12NSOPW,
        MISCREG_ATS12NSOUR,
        MISCREG_ATS12NSOUW,
        MISCREG_DCCMVAC,
        MISCREG_DCCSW,
        MISCREG_CP15DSB,
        MISCREG_CP15DMB,
        MISCREG_DCCMVAU,
        MISCREG_DCCIMVAC,
        MISCREG_DCCISW,
        MISCREG_ATS1HR,
        MISCREG_ATS1HW,
        MISCREG_TLBIALLIS,
        MISCREG_TLBIMVAIS,
        MISCREG_TLBIASIDIS,
        MISCREG_TLBIMVAAIS,
        MISCREG_TLBIMVALIS,
        MISCREG_TLBIMVAALIS,
        MISCREG_ITLBIALL,
        MISCREG_ITLBIMVA,
        MISCREG_ITLBIASID,
        MISCREG_DTLBIALL,
        MISCREG_DTLBIMVA,
        MISCREG_DTLBIASID,
        MISCREG_TLBIALL,
        MISCREG_TLBIMVA,
        MISCREG_TLBIASID,
        MISCREG_TLBIMVAA,
        MISCREG_TLBIMVAL,
        MISCREG_TLBIMVAAL,
        MISCREG_TLBIIPAS2IS,
        MISCREG_TLBIIPAS2LIS,
        MISCREG_TLBIALLHIS,
        MISCREG_TLBIMVAHIS,
        MISCREG_TLBIALLNSNHIS,
        MISCREG_TLBIMVALHIS,
        MISCREG_TLBIIPAS2,
        MISCREG_TLBIIPAS2L,
        MISCREG_TLBIALLH,
        MISCREG_TLBIMVAH,
        MISCREG_TLBIALLNSNH,
        MISCREG_TLBIMVALH,
        MISCREG_PMCR,
        MISCREG_PMCNTENSET,
        MISCREG_PMCNTENCLR,
        MISCREG_PMOVSR,
        MISCREG_PMSWINC,
        MISCREG_PMSELR,
        MISCREG_PMCEID0,
        MISCREG_PMCEID1,
        MISCREG_PMCCNTR,
        MISCREG_PMXEVTYPER,
        MISCREG_PMCCFILTR,
        MISCREG_PMXEVCNTR,
        MISCREG_PMUSERENR,
        MISCREG_PMINTENSET,
        MISCREG_PMINTENCLR,
        MISCREG_PMOVSSET,
        MISCREG_L2CTLR,
        MISCREG_L2ECTLR,
        MISCREG_PRRR,
        MISCREG_PRRR_NS,
        MISCREG_PRRR_S,
        MISCREG_MAIR0,
        MISCREG_MAIR0_NS,
        MISCREG_MAIR0_S,
        MISCREG_NMRR,
        MISCREG_NMRR_NS,
        MISCREG_NMRR_S,
        MISCREG_MAIR1,
        MISCREG_MAIR1_NS,
        MISCREG_MAIR1_S,
        MISCREG_AMAIR0,
        MISCREG_AMAIR0_NS,
        MISCREG_AMAIR0_S,
        MISCREG_AMAIR1,
        MISCREG_AMAIR1_NS,
        MISCREG_AMAIR1_S,
        MISCREG_HMAIR0,
        MISCREG_HMAIR1,
        MISCREG_HAMAIR0,
        MISCREG_HAMAIR1,
        MISCREG_VBAR,
        MISCREG_VBAR_NS,
        MISCREG_VBAR_S,
        MISCREG_MVBAR,
        MISCREG_RMR,
        MISCREG_ISR,
        MISCREG_HVBAR,
        MISCREG_FCSEIDR,
        MISCREG_CONTEXTIDR,
        MISCREG_CONTEXTIDR_NS,
        MISCREG_CONTEXTIDR_S,
        MISCREG_TPIDRURW,
        MISCREG_TPIDRURW_NS,
        MISCREG_TPIDRURW_S,
        MISCREG_TPIDRURO,
        MISCREG_TPIDRURO_NS,
        MISCREG_TPIDRURO_S,
        MISCREG_TPIDRPRW,
        MISCREG_TPIDRPRW_NS,
        MISCREG_TPIDRPRW_S,
        MISCREG_HTPIDR,
        // BEGIN Generic Timer (AArch32)
        MISCREG_CNTFRQ,
        MISCREG_CNTPCT,
        MISCREG_CNTVCT,
        MISCREG_CNTP_CTL,
        MISCREG_CNTP_CTL_NS,
        MISCREG_CNTP_CTL_S,
        MISCREG_CNTP_CVAL,
        MISCREG_CNTP_CVAL_NS,
        MISCREG_CNTP_CVAL_S,
        MISCREG_CNTP_TVAL,
        MISCREG_CNTP_TVAL_NS,
        MISCREG_CNTP_TVAL_S,
        MISCREG_CNTV_CTL,
        MISCREG_CNTV_CVAL,
        MISCREG_CNTV_TVAL,
        MISCREG_CNTKCTL,
        MISCREG_CNTHCTL,
        MISCREG_CNTHP_CTL,
        MISCREG_CNTHP_CVAL,
        MISCREG_CNTHP_TVAL,
        MISCREG_CNTVOFF,
        // END Generic Timer (AArch32)
        MISCREG_IL1DATA0,
        MISCREG_IL1DATA1,
        MISCREG_IL1DATA2,
        MISCREG_IL1DATA3,
        MISCREG_DL1DATA0,
        MISCREG_DL1DATA1,
        MISCREG_DL1DATA2,
        MISCREG_DL1DATA3,
        MISCREG_DL1DATA4,
        MISCREG_RAMINDEX,
        MISCREG_L2ACTLR,
        MISCREG_CBAR,
        MISCREG_HTTBR,
        MISCREG_VTTBR,
        MISCREG_CPUMERRSR,
        MISCREG_L2MERRSR,

        // AArch64 registers (Op0=2)
        MISCREG_MDCCINT_EL1,
        MISCREG_OSDTRRX_EL1,
        MISCREG_MDSCR_EL1,
        MISCREG_OSDTRTX_EL1,
        MISCREG_OSECCR_EL1,
        MISCREG_DBGBVR0_EL1,
        MISCREG_DBGBVR1_EL1,
        MISCREG_DBGBVR2_EL1,
        MISCREG_DBGBVR3_EL1,
        MISCREG_DBGBVR4_EL1,
        MISCREG_DBGBVR5_EL1,
        MISCREG_DBGBVR6_EL1,
        MISCREG_DBGBVR7_EL1,
        MISCREG_DBGBVR8_EL1,
        MISCREG_DBGBVR9_EL1,
        MISCREG_DBGBVR10_EL1,
        MISCREG_DBGBVR11_EL1,
        MISCREG_DBGBVR12_EL1,
        MISCREG_DBGBVR13_EL1,
        MISCREG_DBGBVR14_EL1,
        MISCREG_DBGBVR15_EL1,
        MISCREG_DBGBCR0_EL1,
        MISCREG_DBGBCR1_EL1,
        MISCREG_DBGBCR2_EL1,
        MISCREG_DBGBCR3_EL1,
        MISCREG_DBGBCR4_EL1,
        MISCREG_DBGBCR5_EL1,
        MISCREG_DBGBCR6_EL1,
        MISCREG_DBGBCR7_EL1,
        MISCREG_DBGBCR8_EL1,
        MISCREG_DBGBCR9_EL1,
        MISCREG_DBGBCR10_EL1,
        MISCREG_DBGBCR11_EL1,
        MISCREG_DBGBCR12_EL1,
        MISCREG_DBGBCR13_EL1,
        MISCREG_DBGBCR14_EL1,
        MISCREG_DBGBCR15_EL1,
        MISCREG_DBGWVR0_EL1,
        MISCREG_DBGWVR1_EL1,
        MISCREG_DBGWVR2_EL1,
        MISCREG_DBGWVR3_EL1,
        MISCREG_DBGWVR4_EL1,
        MISCREG_DBGWVR5_EL1,
        MISCREG_DBGWVR6_EL1,
        MISCREG_DBGWVR7_EL1,
        MISCREG_DBGWVR8_EL1,
        MISCREG_DBGWVR9_EL1,
        MISCREG_DBGWVR10_EL1,
        MISCREG_DBGWVR11_EL1,
        MISCREG_DBGWVR12_EL1,
        MISCREG_DBGWVR13_EL1,
        MISCREG_DBGWVR14_EL1,
        MISCREG_DBGWVR15_EL1,
        MISCREG_DBGWCR0_EL1,
        MISCREG_DBGWCR1_EL1,
        MISCREG_DBGWCR2_EL1,
        MISCREG_DBGWCR3_EL1,
        MISCREG_DBGWCR4_EL1,
        MISCREG_DBGWCR5_EL1,
        MISCREG_DBGWCR6_EL1,
        MISCREG_DBGWCR7_EL1,
        MISCREG_DBGWCR8_EL1,
        MISCREG_DBGWCR9_EL1,
        MISCREG_DBGWCR10_EL1,
        MISCREG_DBGWCR11_EL1,
        MISCREG_DBGWCR12_EL1,
        MISCREG_DBGWCR13_EL1,
        MISCREG_DBGWCR14_EL1,
        MISCREG_DBGWCR15_EL1,
        MISCREG_MDCCSR_EL0,
        MISCREG_MDDTR_EL0,
        MISCREG_MDDTRTX_EL0,
        MISCREG_MDDTRRX_EL0,
        MISCREG_DBGVCR32_EL2,
        MISCREG_MDRAR_EL1,
        MISCREG_OSLAR_EL1,
        MISCREG_OSLSR_EL1,
        MISCREG_OSDLR_EL1,
        MISCREG_DBGPRCR_EL1,
        MISCREG_DBGCLAIMSET_EL1,
        MISCREG_DBGCLAIMCLR_EL1,
        MISCREG_DBGAUTHSTATUS_EL1,
        MISCREG_TEECR32_EL1, // not in ARM DDI 0487A.b+
        MISCREG_TEEHBR32_EL1, // not in ARM DDI 0487A.b+

        // AArch64 registers (Op0=1,3)
        MISCREG_MIDR_EL1,
        MISCREG_MPIDR_EL1,
        MISCREG_REVIDR_EL1,
        MISCREG_ID_PFR0_EL1,
        MISCREG_ID_PFR1_EL1,
        MISCREG_ID_DFR0_EL1,
        MISCREG_ID_AFR0_EL1,
        MISCREG_ID_MMFR0_EL1,
        MISCREG_ID_MMFR1_EL1,
        MISCREG_ID_MMFR2_EL1,
        MISCREG_ID_MMFR3_EL1,
        MISCREG_ID_MMFR4_EL1,
        MISCREG_ID_ISAR0_EL1,
        MISCREG_ID_ISAR1_EL1,
        MISCREG_ID_ISAR2_EL1,
        MISCREG_ID_ISAR3_EL1,
        MISCREG_ID_ISAR4_EL1,
        MISCREG_ID_ISAR5_EL1,
        MISCREG_ID_ISAR6_EL1,
        MISCREG_MVFR0_EL1,
        MISCREG_MVFR1_EL1,
        MISCREG_MVFR2_EL1,
        MISCREG_ID_AA64PFR0_EL1,
        MISCREG_ID_AA64PFR1_EL1,
        MISCREG_ID_AA64DFR0_EL1,
        MISCREG_ID_AA64DFR1_EL1,
        MISCREG_ID_AA64AFR0_EL1,
        MISCREG_ID_AA64AFR1_EL1,
        MISCREG_ID_AA64ISAR0_EL1,
        MISCREG_ID_AA64ISAR1_EL1,
        MISCREG_ID_AA64MMFR0_EL1,
        MISCREG_ID_AA64MMFR1_EL1,
        MISCREG_CCSIDR_EL1,
        MISCREG_CLIDR_EL1,
        MISCREG_AIDR_EL1,
        MISCREG_CSSELR_EL1,
        MISCREG_CTR_EL0,
        MISCREG_DCZID_EL0,
        MISCREG_VPIDR_EL2,
        MISCREG_VMPIDR_EL2,
        MISCREG_SCTLR_EL1,
        MISCREG_SCTLR_EL12,
        MISCREG_SCTLR2_EL1,
        MISCREG_SCTLR2_EL12,
        MISCREG_ACTLR_EL1,
        MISCREG_CPACR_EL1,
        MISCREG_CPACR_EL12,
        MISCREG_SCTLR_EL2,
        MISCREG_SCTLR2_EL2,
        MISCREG_ACTLR_EL2,
        MISCREG_HCR_EL2,
        MISCREG_HCRX_EL2,
        MISCREG_MDCR_EL2,
        MISCREG_CPTR_EL2,
        MISCREG_HSTR_EL2,
        MISCREG_HACR_EL2,
        MISCREG_SCTLR_EL3,
        MISCREG_SCTLR2_EL3,
        MISCREG_ACTLR_EL3,
        MISCREG_SCR_EL3,
        MISCREG_SDER32_EL3,
        MISCREG_CPTR_EL3,
        MISCREG_MDCR_EL3,
        MISCREG_TTBR0_EL1,
        MISCREG_TTBR0_EL12,
        MISCREG_TTBR1_EL1,
        MISCREG_TTBR1_EL12,
        MISCREG_TCR_EL1,
        MISCREG_TCR_EL12,
        MISCREG_TCR2_EL1,
        MISCREG_TCR2_EL12,
        MISCREG_TTBR0_EL2,
        MISCREG_TCR_EL2,
        MISCREG_TCR2_EL2,
        MISCREG_VTTBR_EL2,
        MISCREG_VTCR_EL2,
        MISCREG_VSTTBR_EL2,
        MISCREG_VSTCR_EL2,
        MISCREG_TTBR0_EL3,
        MISCREG_TCR_EL3,
        MISCREG_DACR32_EL2,
        MISCREG_SPSR_EL1,
        MISCREG_SPSR_EL12,
        MISCREG_ELR_EL1,
        MISCREG_ELR_EL12,
        MISCREG_SP_EL0,
        MISCREG_SPSEL,
        MISCREG_CURRENTEL,
        MISCREG_NZCV,
        MISCREG_DAIF,
        MISCREG_FPCR,
        MISCREG_FPSR,
        MISCREG_DSPSR_EL0,
        MISCREG_DLR_EL0,
        MISCREG_SPSR_EL2,
        MISCREG_ELR_EL2,
        MISCREG_SP_EL1,
        MISCREG_SPSR_IRQ_AA64,
        MISCREG_SPSR_ABT_AA64,
        MISCREG_SPSR_UND_AA64,
        MISCREG_SPSR_FIQ_AA64,
        MISCREG_SPSR_EL3,
        MISCREG_ELR_EL3,
        MISCREG_SP_EL2,
        MISCREG_AFSR0_EL1,
        MISCREG_AFSR0_EL12,
        MISCREG_AFSR1_EL1,
        MISCREG_AFSR1_EL12,
        MISCREG_ESR_EL1,
        MISCREG_ESR_EL12,
        MISCREG_IFSR32_EL2,
        MISCREG_AFSR0_EL2,
        MISCREG_AFSR1_EL2,
        MISCREG_ESR_EL2,
        MISCREG_FPEXC32_EL2,
        MISCREG_AFSR0_EL3,
        MISCREG_AFSR1_EL3,
        MISCREG_ESR_EL3,
        MISCREG_FAR_EL1,
        MISCREG_FAR_EL12,
        MISCREG_FAR_EL2,
        MISCREG_HPFAR_EL2,
        MISCREG_FAR_EL3,
        MISCREG_IC_IALLUIS,
        MISCREG_PAR_EL1,
        MISCREG_IC_IALLU,
        MISCREG_DC_IVAC_Xt,
        MISCREG_DC_ISW_Xt,
        MISCREG_AT_S1E1R_Xt,
        MISCREG_AT_S1E1W_Xt,
        MISCREG_AT_S1E0R_Xt,
        MISCREG_AT_S1E0W_Xt,
        MISCREG_DC_CSW_Xt,
        MISCREG_DC_CISW_Xt,
        MISCREG_DC_ZVA_Xt,
        MISCREG_IC_IVAU_Xt,
        MISCREG_DC_CVAC_Xt,
        MISCREG_DC_CVAU_Xt,
        MISCREG_DC_CIVAC_Xt,
        MISCREG_AT_S1E2R_Xt,
        MISCREG_AT_S1E2W_Xt,
        MISCREG_AT_S12E1R_Xt,
        MISCREG_AT_S12E1W_Xt,
        MISCREG_AT_S12E0R_Xt,
        MISCREG_AT_S12E0W_Xt,
        MISCREG_AT_S1E3R_Xt,
        MISCREG_AT_S1E3W_Xt,
        MISCREG_TLBI_VMALLE1IS,
        MISCREG_TLBI_VMALLE1OS,
        MISCREG_TLBI_VAE1IS,
        MISCREG_TLBI_VAE1OS,
        MISCREG_TLBI_ASIDE1IS,
        MISCREG_TLBI_ASIDE1OS,
        MISCREG_TLBI_VAAE1IS,
        MISCREG_TLBI_VAAE1OS,
        MISCREG_TLBI_VALE1IS,
        MISCREG_TLBI_VALE1OS,
        MISCREG_TLBI_VAALE1IS,
        MISCREG_TLBI_VAALE1OS,
        MISCREG_TLBI_VMALLE1,
        MISCREG_TLBI_VAE1,
        MISCREG_TLBI_ASIDE1,
        MISCREG_TLBI_VAAE1,
        MISCREG_TLBI_VALE1,
        MISCREG_TLBI_VAALE1,
        MISCREG_TLBI_IPAS2E1IS,
        MISCREG_TLBI_IPAS2E1OS,
        MISCREG_TLBI_IPAS2LE1IS,
        MISCREG_TLBI_IPAS2LE1OS,
        MISCREG_TLBI_ALLE2IS,
        MISCREG_TLBI_ALLE2OS,
        MISCREG_TLBI_VAE2IS,
        MISCREG_TLBI_VAE2OS,
        MISCREG_TLBI_ALLE1IS,
        MISCREG_TLBI_ALLE1OS,
        MISCREG_TLBI_VALE2IS,
        MISCREG_TLBI_VALE2OS,
        MISCREG_TLBI_VMALLS12E1IS,
        MISCREG_TLBI_VMALLS12E1OS,
        MISCREG_TLBI_IPAS2E1,
        MISCREG_TLBI_IPAS2LE1,
        MISCREG_TLBI_ALLE2,
        MISCREG_TLBI_VAE2,
        MISCREG_TLBI_ALLE1,
        MISCREG_TLBI_VALE2,
        MISCREG_TLBI_VMALLS12E1,
        MISCREG_TLBI_ALLE3IS,
        MISCREG_TLBI_ALLE3OS,
        MISCREG_TLBI_VAE3IS,
        MISCREG_TLBI_VAE3OS,
        MISCREG_TLBI_VALE3IS,
        MISCREG_TLBI_VALE3OS,
        MISCREG_TLBI_ALLE3,
        MISCREG_TLBI_VAE3,
        MISCREG_TLBI_VALE3,
        MISCREG_TLBI_RVAE1,
        MISCREG_TLBI_RVAAE1,
        MISCREG_TLBI_RVALE1,
        MISCREG_TLBI_RVAALE1,
        MISCREG_TLBI_RIPAS2E1,
        MISCREG_TLBI_RIPAS2LE1,
        MISCREG_TLBI_RVAE2,
        MISCREG_TLBI_RVALE2,
        MISCREG_TLBI_RVAE3,
        MISCREG_TLBI_RVALE3,
        MISCREG_TLBI_RVAE1IS,
        MISCREG_TLBI_RVAAE1IS,
        MISCREG_TLBI_RVALE1IS,
        MISCREG_TLBI_RVAALE1IS,
        MISCREG_TLBI_RIPAS2E1IS,
        MISCREG_TLBI_RIPAS2LE1IS,
        MISCREG_TLBI_RVAE2IS,
        MISCREG_TLBI_RVALE2IS,
        MISCREG_TLBI_RVAE3IS,
        MISCREG_TLBI_RVALE3IS,
        MISCREG_TLBI_RVAE1OS,
        MISCREG_TLBI_RVAAE1OS,
        MISCREG_TLBI_RVALE1OS,
        MISCREG_TLBI_RVAALE1OS,
        MISCREG_TLBI_RIPAS2E1OS,
        MISCREG_TLBI_RIPAS2LE1OS,
        MISCREG_TLBI_RVAE2OS,
        MISCREG_TLBI_RVALE2OS,
        MISCREG_TLBI_RVAE3OS,
        MISCREG_TLBI_RVALE3OS,
        MISCREG_PMINTENSET_EL1,
        MISCREG_PMINTENCLR_EL1,
        MISCREG_PMCR_EL0,
        MISCREG_PMCNTENSET_EL0,
        MISCREG_PMCNTENCLR_EL0,
        MISCREG_PMOVSCLR_EL0,
        MISCREG_PMSWINC_EL0,
        MISCREG_PMSELR_EL0,
        MISCREG_PMCEID0_EL0,
        MISCREG_PMCEID1_EL0,
        MISCREG_PMCCNTR_EL0,
        MISCREG_PMXEVTYPER_EL0,
        MISCREG_PMCCFILTR_EL0,
        MISCREG_PMXEVCNTR_EL0,
        MISCREG_PMUSERENR_EL0,
        MISCREG_PMOVSSET_EL0,
        MISCREG_MAIR_EL1,
        MISCREG_MAIR_EL12,
        MISCREG_AMAIR_EL1,
        MISCREG_AMAIR_EL12,
        MISCREG_MAIR_EL2,
        MISCREG_AMAIR_EL2,
        MISCREG_MAIR_EL3,
        MISCREG_AMAIR_EL3,
        MISCREG_L2CTLR_EL1,
        MISCREG_L2ECTLR_EL1,
        MISCREG_VBAR_EL1,
        MISCREG_VBAR_EL12,
        MISCREG_RVBAR_EL1,
        MISCREG_ISR_EL1,
        MISCREG_VBAR_EL2,
        MISCREG_RVBAR_EL2,
        MISCREG_VBAR_EL3,
        MISCREG_RVBAR_EL3,
        MISCREG_RMR_EL3,
        MISCREG_CONTEXTIDR_EL1,
        MISCREG_CONTEXTIDR_EL12,
        MISCREG_TPIDR_EL1,
        MISCREG_TPIDR_EL0,
        MISCREG_TPIDRRO_EL0,
        MISCREG_TPIDR_EL2,
        MISCREG_TPIDR_EL3,
        // BEGIN Generic Timer (AArch64)
        MISCREG_CNTFRQ_EL0,
        MISCREG_CNTPCT_EL0,
        MISCREG_CNTVCT_EL0,
        MISCREG_CNTP_CTL_EL0,
        MISCREG_CNTP_CVAL_EL0,
        MISCREG_CNTP_TVAL_EL0,
        MISCREG_CNTV_CTL_EL0,
        MISCREG_CNTV_CVAL_EL0,
        MISCREG_CNTV_TVAL_EL0,
        MISCREG_CNTP_CTL_EL02,
        MISCREG_CNTP_CVAL_EL02,
        MISCREG_CNTP_TVAL_EL02,
        MISCREG_CNTV_CTL_EL02,
        MISCREG_CNTV_CVAL_EL02,
        MISCREG_CNTV_TVAL_EL02,
        MISCREG_CNTKCTL_EL1,
        MISCREG_CNTKCTL_EL12,
        MISCREG_CNTPS_CTL_EL1,
        MISCREG_CNTPS_CVAL_EL1,
        MISCREG_CNTPS_TVAL_EL1,
        MISCREG_CNTHCTL_EL2,
        MISCREG_CNTHP_CTL_EL2,
        MISCREG_CNTHP_CVAL_EL2,
        MISCREG_CNTHP_TVAL_EL2,
        MISCREG_CNTHPS_CTL_EL2,
        MISCREG_CNTHPS_CVAL_EL2,
        MISCREG_CNTHPS_TVAL_EL2,
        // IF Armv8.1-VHE
        MISCREG_CNTHV_CTL_EL2,
        MISCREG_CNTHV_CVAL_EL2,
        MISCREG_CNTHV_TVAL_EL2,
        MISCREG_CNTHVS_CTL_EL2,
        MISCREG_CNTHVS_CVAL_EL2,
        MISCREG_CNTHVS_TVAL_EL2,
        // ENDIF Armv8.1-VHE
        MISCREG_CNTVOFF_EL2,
        // END Generic Timer (AArch64)
        MISCREG_PMEVCNTR0_EL0,
        MISCREG_PMEVCNTR1_EL0,
        MISCREG_PMEVCNTR2_EL0,
        MISCREG_PMEVCNTR3_EL0,
        MISCREG_PMEVCNTR4_EL0,
        MISCREG_PMEVCNTR5_EL0,
        MISCREG_PMEVTYPER0_EL0,
        MISCREG_PMEVTYPER1_EL0,
        MISCREG_PMEVTYPER2_EL0,
        MISCREG_PMEVTYPER3_EL0,
        MISCREG_PMEVTYPER4_EL0,
        MISCREG_PMEVTYPER5_EL0,
        MISCREG_IL1DATA0_EL1,
        MISCREG_IL1DATA1_EL1,
        MISCREG_IL1DATA2_EL1,
        MISCREG_IL1DATA3_EL1,
        MISCREG_DL1DATA0_EL1,
        MISCREG_DL1DATA1_EL1,
        MISCREG_DL1DATA2_EL1,
        MISCREG_DL1DATA3_EL1,
        MISCREG_DL1DATA4_EL1,
        MISCREG_L2ACTLR_EL1,
        MISCREG_CPUACTLR_EL1,
        MISCREG_CPUECTLR_EL1,
        MISCREG_CPUMERRSR_EL1,
        MISCREG_L2MERRSR_EL1,
        MISCREG_CBAR_EL1,
        MISCREG_CONTEXTIDR_EL2,

        // Introduced in ARMv8.1
        MISCREG_TTBR1_EL2,

        MISCREG_ID_AA64MMFR2_EL1,
        MISCREG_ID_AA64MMFR3_EL1,

        //PAuth Key Regsiters
        MISCREG_APDAKeyHi_EL1,
        MISCREG_APDAKeyLo_EL1,
        MISCREG_APDBKeyHi_EL1,
        MISCREG_APDBKeyLo_EL1,
        MISCREG_APGAKeyHi_EL1,
        MISCREG_APGAKeyLo_EL1,
        MISCREG_APIAKeyHi_EL1,
        MISCREG_APIAKeyLo_EL1,
        MISCREG_APIBKeyHi_EL1,
        MISCREG_APIBKeyLo_EL1,

        // GICv3, CPU interface
        MISCREG_ICC_PMR_EL1,
        MISCREG_ICC_IAR0_EL1,
        MISCREG_ICC_EOIR0_EL1,
        MISCREG_ICC_HPPIR0_EL1,
        MISCREG_ICC_BPR0_EL1,
        MISCREG_ICC_AP0R0_EL1,
        MISCREG_ICC_AP0R1_EL1,
        MISCREG_ICC_AP0R2_EL1,
        MISCREG_ICC_AP0R3_EL1,
        MISCREG_ICC_AP1R0_EL1,
        MISCREG_ICC_AP1R0_EL1_NS,
        MISCREG_ICC_AP1R0_EL1_S,
        MISCREG_ICC_AP1R1_EL1,
        MISCREG_ICC_AP1R1_EL1_NS,
        MISCREG_ICC_AP1R1_EL1_S,
        MISCREG_ICC_AP1R2_EL1,
        MISCREG_ICC_AP1R2_EL1_NS,
        MISCREG_ICC_AP1R2_EL1_S,
        MISCREG_ICC_AP1R3_EL1,
        MISCREG_ICC_AP1R3_EL1_NS,
        MISCREG_ICC_AP1R3_EL1_S,
        MISCREG_ICC_DIR_EL1,
        MISCREG_ICC_RPR_EL1,
        MISCREG_ICC_SGI1R_EL1,
        MISCREG_ICC_ASGI1R_EL1,
        MISCREG_ICC_SGI0R_EL1,
        MISCREG_ICC_IAR1_EL1,
        MISCREG_ICC_EOIR1_EL1,
        MISCREG_ICC_HPPIR1_EL1,
        MISCREG_ICC_BPR1_EL1,
        MISCREG_ICC_BPR1_EL1_NS,
        MISCREG_ICC_BPR1_EL1_S,
        MISCREG_ICC_CTLR_EL1,
        MISCREG_ICC_CTLR_EL1_NS,
        MISCREG_ICC_CTLR_EL1_S,
        MISCREG_ICC_SRE_EL1,
        MISCREG_ICC_SRE_EL1_NS,
        MISCREG_ICC_SRE_EL1_S,
        MISCREG_ICC_IGRPEN0_EL1,
        MISCREG_ICC_IGRPEN1_EL1,
        MISCREG_ICC_IGRPEN1_EL1_NS,
        MISCREG_ICC_IGRPEN1_EL1_S,
        MISCREG_ICC_SRE_EL2,
        MISCREG_ICC_CTLR_EL3,
        MISCREG_ICC_SRE_EL3,
        MISCREG_ICC_IGRPEN1_EL3,

        // GICv3, CPU interface, virtualization
        MISCREG_ICH_AP0R0_EL2,
        MISCREG_ICH_AP0R1_EL2,
        MISCREG_ICH_AP0R2_EL2,
        MISCREG_ICH_AP0R3_EL2,
        MISCREG_ICH_AP1R0_EL2,
        MISCREG_ICH_AP1R1_EL2,
        MISCREG_ICH_AP1R2_EL2,
        MISCREG_ICH_AP1R3_EL2,
        MISCREG_ICH_HCR_EL2,
        MISCREG_ICH_VTR_EL2,
        MISCREG_ICH_MISR_EL2,
        MISCREG_ICH_EISR_EL2,
        MISCREG_ICH_ELRSR_EL2,
        MISCREG_ICH_VMCR_EL2,
        MISCREG_ICH_LR0_EL2,
        MISCREG_ICH_LR1_EL2,
        MISCREG_ICH_LR2_EL2,
        MISCREG_ICH_LR3_EL2,
        MISCREG_ICH_LR4_EL2,
        MISCREG_ICH_LR5_EL2,
        MISCREG_ICH_LR6_EL2,
        MISCREG_ICH_LR7_EL2,
        MISCREG_ICH_LR8_EL2,
        MISCREG_ICH_LR9_EL2,
        MISCREG_ICH_LR10_EL2,
        MISCREG_ICH_LR11_EL2,
        MISCREG_ICH_LR12_EL2,
        MISCREG_ICH_LR13_EL2,
        MISCREG_ICH_LR14_EL2,
        MISCREG_ICH_LR15_EL2,

        MISCREG_ICV_PMR_EL1,
        MISCREG_ICV_IAR0_EL1,
        MISCREG_ICV_EOIR0_EL1,
        MISCREG_ICV_HPPIR0_EL1,
        MISCREG_ICV_BPR0_EL1,
        MISCREG_ICV_AP0R0_EL1,
        MISCREG_ICV_AP0R1_EL1,
        MISCREG_ICV_AP0R2_EL1,
        MISCREG_ICV_AP0R3_EL1,
        MISCREG_ICV_AP1R0_EL1,
        MISCREG_ICV_AP1R0_EL1_NS,
        MISCREG_ICV_AP1R0_EL1_S,
        MISCREG_ICV_AP1R1_EL1,
        MISCREG_ICV_AP1R1_EL1_NS,
        MISCREG_ICV_AP1R1_EL1_S,
        MISCREG_ICV_AP1R2_EL1,
        MISCREG_ICV_AP1R2_EL1_NS,
        MISCREG_ICV_AP1R2_EL1_S,
        MISCREG_ICV_AP1R3_EL1,
        MISCREG_ICV_AP1R3_EL1_NS,
        MISCREG_ICV_AP1R3_EL1_S,
        MISCREG_ICV_DIR_EL1,
        MISCREG_ICV_RPR_EL1,
        MISCREG_ICV_SGI1R_EL1,
        MISCREG_ICV_ASGI1R_EL1,
        MISCREG_ICV_SGI0R_EL1,
        MISCREG_ICV_IAR1_EL1,
        MISCREG_ICV_EOIR1_EL1,
        MISCREG_ICV_HPPIR1_EL1,
        MISCREG_ICV_BPR1_EL1,
        MISCREG_ICV_BPR1_EL1_NS,
        MISCREG_ICV_BPR1_EL1_S,
        MISCREG_ICV_CTLR_EL1,
        MISCREG_ICV_CTLR_EL1_NS,
        MISCREG_ICV_CTLR_EL1_S,
        MISCREG_ICV_SRE_EL1,
        MISCREG_ICV_SRE_EL1_NS,
        MISCREG_ICV_SRE_EL1_S,
        MISCREG_ICV_IGRPEN0_EL1,
        MISCREG_ICV_IGRPEN1_EL1,
        MISCREG_ICV_IGRPEN1_EL1_NS,
        MISCREG_ICV_IGRPEN1_EL1_S,

        MISCREG_ICC_AP0R0,
        MISCREG_ICC_AP0R1,
        MISCREG_ICC_AP0R2,
        MISCREG_ICC_AP0R3,
        MISCREG_ICC_AP1R0,
        MISCREG_ICC_AP1R0_NS,
        MISCREG_ICC_AP1R0_S,
        MISCREG_ICC_AP1R1,
        MISCREG_ICC_AP1R1_NS,
        MISCREG_ICC_AP1R1_S,
        MISCREG_ICC_AP1R2,
        MISCREG_ICC_AP1R2_NS,
        MISCREG_ICC_AP1R2_S,
        MISCREG_ICC_AP1R3,
        MISCREG_ICC_AP1R3_NS,
        MISCREG_ICC_AP1R3_S,
        MISCREG_ICC_ASGI1R,
        MISCREG_ICC_BPR0,
        MISCREG_ICC_BPR1,
        MISCREG_ICC_BPR1_NS,
        MISCREG_ICC_BPR1_S,
        MISCREG_ICC_CTLR,
        MISCREG_ICC_CTLR_NS,
        MISCREG_ICC_CTLR_S,
        MISCREG_ICC_DIR,
        MISCREG_ICC_EOIR0,
        MISCREG_ICC_EOIR1,
        MISCREG_ICC_HPPIR0,
        MISCREG_ICC_HPPIR1,
        MISCREG_ICC_HSRE,
        MISCREG_ICC_IAR0,
        MISCREG_ICC_IAR1,
        MISCREG_ICC_IGRPEN0,
        MISCREG_ICC_IGRPEN1,
        MISCREG_ICC_IGRPEN1_NS,
        MISCREG_ICC_IGRPEN1_S,
        MISCREG_ICC_MCTLR,
        MISCREG_ICC_MGRPEN1,
        MISCREG_ICC_MSRE,
        MISCREG_ICC_PMR,
        MISCREG_ICC_RPR,
        MISCREG_ICC_SGI0R,
        MISCREG_ICC_SGI1R,
        MISCREG_ICC_SRE,
        MISCREG_ICC_SRE_NS,
        MISCREG_ICC_SRE_S,

        MISCREG_ICH_AP0R0,
        MISCREG_ICH_AP0R1,
        MISCREG_ICH_AP0R2,
        MISCREG_ICH_AP0R3,
        MISCREG_ICH_AP1R0,
        MISCREG_ICH_AP1R1,
        MISCREG_ICH_AP1R2,
        MISCREG_ICH_AP1R3,
        MISCREG_ICH_HCR,
        MISCREG_ICH_VTR,
        MISCREG_ICH_MISR,
        MISCREG_ICH_EISR,
        MISCREG_ICH_ELRSR,
        MISCREG_ICH_VMCR,
        MISCREG_ICH_LR0,
        MISCREG_ICH_LR1,
        MISCREG_ICH_LR2,
        MISCREG_ICH_LR3,
        MISCREG_ICH_LR4,
        MISCREG_ICH_LR5,
        MISCREG_ICH_LR6,
        MISCREG_ICH_LR7,
        MISCREG_ICH_LR8,
        MISCREG_ICH_LR9,
        MISCREG_ICH_LR10,
        MISCREG_ICH_LR11,
        MISCREG_ICH_LR12,
        MISCREG_ICH_LR13,
        MISCREG_ICH_LR14,
        MISCREG_ICH_LR15,
        MISCREG_ICH_LRC0,
        MISCREG_ICH_LRC1,
        MISCREG_ICH_LRC2,
        MISCREG_ICH_LRC3,
        MISCREG_ICH_LRC4,
        MISCREG_ICH_LRC5,
        MISCREG_ICH_LRC6,
        MISCREG_ICH_LRC7,
        MISCREG_ICH_LRC8,
        MISCREG_ICH_LRC9,
        MISCREG_ICH_LRC10,
        MISCREG_ICH_LRC11,
        MISCREG_ICH_LRC12,
        MISCREG_ICH_LRC13,
        MISCREG_ICH_LRC14,
        MISCREG_ICH_LRC15,

        // SVE
        MISCREG_ID_AA64ZFR0_EL1,
        MISCREG_ZCR_EL3,
        MISCREG_ZCR_EL2,
        MISCREG_ZCR_EL12,
        MISCREG_ZCR_EL1,

        // SME
        MISCREG_ID_AA64SMFR0_EL1,
        MISCREG_SVCR,
        MISCREG_SMIDR_EL1,
        MISCREG_SMPRI_EL1,
        MISCREG_SMPRIMAP_EL2,
        MISCREG_SMCR_EL3,
        MISCREG_SMCR_EL2,
        MISCREG_SMCR_EL12,
        MISCREG_SMCR_EL1,
        MISCREG_TPIDR2_EL0,
        MISCREG_MPAMSM_EL1,

        // FEAT_RNG
        MISCREG_RNDR,
        MISCREG_RNDRRS,

        // FEAT_FGT
        MISCREG_HFGITR_EL2,
        MISCREG_HFGRTR_EL2,
        MISCREG_HFGWTR_EL2,
        MISCREG_HDFGRTR_EL2,
        MISCREG_HDFGWTR_EL2,

        // FEAT_MPAM
        MISCREG_MPAMIDR_EL1,
        MISCREG_MPAM0_EL1,
        MISCREG_MPAM1_EL1,
        MISCREG_MPAM2_EL2,
        MISCREG_MPAM3_EL3,
        MISCREG_MPAM1_EL12,
        MISCREG_MPAMHCR_EL2,
        MISCREG_MPAMVPMV_EL2,
        MISCREG_MPAMVPM0_EL2,
        MISCREG_MPAMVPM1_EL2,
        MISCREG_MPAMVPM2_EL2,
        MISCREG_MPAMVPM3_EL2,
        MISCREG_MPAMVPM4_EL2,
        MISCREG_MPAMVPM5_EL2,
        MISCREG_MPAMVPM6_EL2,
        MISCREG_MPAMVPM7_EL2,

        // NUM_PHYS_MISCREGS specifies the number of actual physical
        // registers, not considering the following pseudo-registers
        // (dummy registers), like MISCREG_UNKNOWN, MISCREG_IMPDEF_UNIMPL.
        // Checkpointing should use this physical index when
        // saving/restoring register values.
        NUM_PHYS_MISCREGS,

        // Dummy registers
        MISCREG_NOP,
        MISCREG_RAZ,
        MISCREG_UNKNOWN,

        // Implementation defined register: this represent
        // a pool of unimplemented registers whose access can throw
        // either UNDEFINED or hypervisor trap exception.
        MISCREG_IMPDEF_UNIMPL,

        // RAS extension (unimplemented)
        MISCREG_ERRIDR_EL1,
        MISCREG_ERRSELR_EL1,
        MISCREG_ERXFR_EL1,
        MISCREG_ERXCTLR_EL1,
        MISCREG_ERXSTATUS_EL1,
        MISCREG_ERXADDR_EL1,
        MISCREG_ERXMISC0_EL1,
        MISCREG_ERXMISC1_EL1,
        MISCREG_DISR_EL1,
        MISCREG_VSESR_EL2,
        MISCREG_VDISR_EL2,

        // PSTATE
        MISCREG_PAN,
        MISCREG_UAO,

        // Total number of Misc Registers: Physical + Dummy
        NUM_MISCREGS
    };

    enum MiscRegInfo
    {
        MISCREG_IMPLEMENTED,
        MISCREG_UNVERIFIABLE,   // Does the value change on every read (e.g. a
                                // arch generic counter)
        MISCREG_UNSERIALIZE,    // Should the checkpointed value be restored?
        MISCREG_WARN_NOT_FAIL,  // If MISCREG_IMPLEMENTED is deasserted, it
                                // tells whether the instruction should raise a
                                // warning or fail
        MISCREG_MUTEX,  // True if the register corresponds to a pair of
                        // mutually exclusive registers
        MISCREG_BANKED,  // True if the register is banked between the two
                         // security states, and this is the parent node of the
                         // two banked registers
        MISCREG_BANKED64, // True if the register is banked between the two
                          // security states, and this is the parent node of
                          // the two banked registers. Used in AA64 only.
        MISCREG_BANKED_CHILD, // The entry is one of the child registers that
                              // forms a banked set of regs (along with the
                              // other child regs)

        // Access permissions
        // User mode
        MISCREG_USR_NS_RD,
        MISCREG_USR_NS_WR,
        MISCREG_USR_S_RD,
        MISCREG_USR_S_WR,
        // Privileged modes other than hypervisor or monitor
        MISCREG_PRI_NS_RD,
        MISCREG_PRI_NS_WR,
        MISCREG_PRI_S_RD,
        MISCREG_PRI_S_WR,
        // Hypervisor mode
        MISCREG_HYP_NS_RD,
        MISCREG_HYP_NS_WR,
        MISCREG_HYP_S_RD,
        MISCREG_HYP_S_WR,
        // Monitor mode, SCR.NS == 0
        MISCREG_MON_NS0_RD,
        MISCREG_MON_NS0_WR,
        // Monitor mode, SCR.NS == 1
        MISCREG_MON_NS1_RD,
        MISCREG_MON_NS1_WR,

        NUM_MISCREG_INFOS
    };

    /** MiscReg metadata **/
    struct MiscRegLUTEntry
    {
        uint32_t lower;  // Lower half mapped to this register
        uint32_t upper;  // Upper half mapped to this register
        uint64_t _reset; // value taken on reset (i.e. initialization)
        uint64_t _res0;  // reserved
        uint64_t _res1;  // reserved
        uint64_t _raz;   // read as zero (fixed at 0)
        uint64_t _rao;   // read as one (fixed at 1)
        std::bitset<NUM_MISCREG_INFOS> info;

        using FaultCB = std::function<
            Fault(const MiscRegLUTEntry &entry, ThreadContext *tc,
                  const MiscRegOp64 &inst)
        >;

        std::array<FaultCB, EL3 + 1> faultRead;
        std::array<FaultCB, EL3 + 1> faultWrite;

        Fault checkFault(ThreadContext *tc, const MiscRegOp64 &inst,
            ExceptionLevel el);

      protected:
        template <MiscRegInfo Sec, MiscRegInfo NonSec>
        static Fault defaultFault(const MiscRegLUTEntry &entry,
            ThreadContext *tc, const MiscRegOp64 &inst);

      public:
        MiscRegLUTEntry() :
            lower(0), upper(0),
            _reset(0), _res0(0), _res1(0), _raz(0), _rao(0), info(0),
            faultRead({defaultFault<MISCREG_USR_S_RD, MISCREG_USR_NS_RD>,
                       defaultFault<MISCREG_PRI_S_RD, MISCREG_PRI_NS_RD>,
                       defaultFault<MISCREG_HYP_S_RD, MISCREG_HYP_NS_RD>,
                       defaultFault<MISCREG_MON_NS0_RD, MISCREG_MON_NS1_RD>}),
            faultWrite({defaultFault<MISCREG_USR_S_WR, MISCREG_USR_NS_WR>,
                        defaultFault<MISCREG_PRI_S_WR, MISCREG_PRI_NS_WR>,
                        defaultFault<MISCREG_HYP_S_WR, MISCREG_HYP_NS_WR>,
                        defaultFault<MISCREG_MON_NS0_WR, MISCREG_MON_NS1_WR>})
        {}
        uint64_t reset() const { return _reset; }
        uint64_t res0()  const { return _res0; }
        uint64_t res1()  const { return _res1; }
        uint64_t raz()   const { return _raz; }
        uint64_t rao()   const { return _rao; }
        // raz/rao implies writes ignored
        uint64_t wi()    const { return _raz | _rao; }
    };

    /** Metadata table accessible via the value of the register */
    class MiscRegLUTEntryInitializer
    {
        struct MiscRegLUTEntry &entry;
        typedef const MiscRegLUTEntryInitializer& chain;
      public:
        chain
        mapsTo(uint32_t l, uint32_t u = 0) const
        {
            entry.lower = l;
            entry.upper = u;
            return *this;
        }
        chain
        reset(uint64_t res_val) const
        {
            entry._reset = res_val;
            return *this;
        }
        chain
        res0(uint64_t mask) const
        {
            entry._res0 = mask;
            return *this;
        }
        chain
        res1(uint64_t mask) const
        {
            entry._res1 = mask;
            return *this;
        }
        chain
        raz(uint64_t mask = (uint64_t)-1) const
        {
            entry._raz  = mask;
            return *this;
        }
        chain
        rao(uint64_t mask = (uint64_t)-1) const
        {
            entry._rao  = mask;
            return *this;
        }
        chain
        implemented(bool v = true) const
        {
            entry.info[MISCREG_IMPLEMENTED] = v;
            return *this;
        }
        chain
        unimplemented() const
        {
            return implemented(false);
        }
        chain
        unverifiable(bool v = true) const
        {
            entry.info[MISCREG_UNVERIFIABLE] = v;
            return *this;
        }
        chain
        unserialize(bool v = true) const
        {
            entry.info[MISCREG_UNSERIALIZE] = v;
            return *this;
        }
        chain
        warnNotFail(bool v = true) const
        {
            entry.info[MISCREG_WARN_NOT_FAIL] = v;
            return *this;
        }
        chain
        mutex(bool v = true) const
        {
            entry.info[MISCREG_MUTEX] = v;
            return *this;
        }
        chain
        banked(bool v = true) const
        {
            entry.info[MISCREG_BANKED] = v;
            return *this;
        }
        chain
        banked64(bool v = true) const
        {
            entry.info[MISCREG_BANKED64] = v;
            return *this;
        }
        chain
        bankedChild(bool v = true) const
        {
            entry.info[MISCREG_BANKED_CHILD] = v;
            return *this;
        }
        chain
        userNonSecureRead(bool v = true) const
        {
            entry.info[MISCREG_USR_NS_RD] = v;
            return *this;
        }
        chain
        userNonSecureWrite(bool v = true) const
        {
            entry.info[MISCREG_USR_NS_WR] = v;
            return *this;
        }
        chain
        userSecureRead(bool v = true) const
        {
            entry.info[MISCREG_USR_S_RD] = v;
            return *this;
        }
        chain
        userSecureWrite(bool v = true) const
        {
            entry.info[MISCREG_USR_S_WR] = v;
            return *this;
        }
        chain
        user(bool v = true) const
        {
            userNonSecureRead(v);
            userNonSecureWrite(v);
            userSecureRead(v);
            userSecureWrite(v);
            return *this;
        }
        chain
        privNonSecureRead(bool v = true) const
        {
            entry.info[MISCREG_PRI_NS_RD] = v;
            return *this;
        }
        chain
        privNonSecureWrite(bool v = true) const
        {
            entry.info[MISCREG_PRI_NS_WR] = v;
            return *this;
        }
        chain
        privNonSecure(bool v = true) const
        {
            privNonSecureRead(v);
            privNonSecureWrite(v);
            return *this;
        }
        chain
        privSecureRead(bool v = true) const
        {
            entry.info[MISCREG_PRI_S_RD] = v;
            return *this;
        }
        chain
        privSecureWrite(bool v = true) const
        {
            entry.info[MISCREG_PRI_S_WR] = v;
            return *this;
        }
        chain
        privSecure(bool v = true) const
        {
            privSecureRead(v);
            privSecureWrite(v);
            return *this;
        }
        chain
        priv(bool v = true) const
        {
            privSecure(v);
            privNonSecure(v);
            return *this;
        }
        chain
        privRead(bool v = true) const
        {
            privSecureRead(v);
            privNonSecureRead(v);
            return *this;
        }
        chain
        hypSecureRead(bool v = true) const
        {
            entry.info[MISCREG_HYP_S_RD] = v;
            return *this;
        }
        chain
        hypNonSecureRead(bool v = true) const
        {
            entry.info[MISCREG_HYP_NS_RD] = v;
            return *this;
        }
        chain
        hypRead(bool v = true) const
        {
            hypSecureRead(v);
            hypNonSecureRead(v);
            return *this;
        }
        chain
        hypSecureWrite(bool v = true) const
        {
            entry.info[MISCREG_HYP_S_WR] = v;
            return *this;
        }
        chain
        hypNonSecureWrite(bool v = true) const
        {
            entry.info[MISCREG_HYP_NS_WR] = v;
            return *this;
        }
        chain
        hypWrite(bool v = true) const
        {
            hypSecureWrite(v);
            hypNonSecureWrite(v);
            return *this;
        }
        chain
        hypSecure(bool v = true) const
        {
            hypSecureRead(v);
            hypSecureWrite(v);
            return *this;
        }
        chain
        hyp(bool v = true) const
        {
            hypRead(v);
            hypWrite(v);
            return *this;
        }
        chain
        monSecureRead(bool v = true) const
        {
            entry.info[MISCREG_MON_NS0_RD] = v;
            return *this;
        }
        chain
        monSecureWrite(bool v = true) const
        {
            entry.info[MISCREG_MON_NS0_WR] = v;
            return *this;
        }
        chain
        monNonSecureRead(bool v = true) const
        {
            entry.info[MISCREG_MON_NS1_RD] = v;
            return *this;
        }
        chain
        monNonSecureWrite(bool v = true) const
        {
            entry.info[MISCREG_MON_NS1_WR] = v;
            return *this;
        }
        chain
        mon(bool v = true) const
        {
            monSecureRead(v);
            monSecureWrite(v);
            monNonSecureRead(v);
            monNonSecureWrite(v);
            return *this;
        }
        chain
        monWrite(bool v = true) const
        {
            monSecureWrite(v);
            monNonSecureWrite(v);
            return *this;
        }
        chain
        monSecure(bool v = true) const
        {
            monSecureRead(v);
            monSecureWrite(v);
            return *this;
        }
        chain
        monNonSecure(bool v = true) const
        {
            monNonSecureRead(v);
            monNonSecureWrite(v);
            return *this;
        }
        chain
        allPrivileges(bool v = true) const
        {
            userNonSecureRead(v);
            userNonSecureWrite(v);
            userSecureRead(v);
            userSecureWrite(v);
            privNonSecureRead(v);
            privNonSecureWrite(v);
            privSecureRead(v);
            privSecureWrite(v);
            hypRead(v);
            hypWrite(v);
            monSecureRead(v);
            monSecureWrite(v);
            monNonSecureRead(v);
            monNonSecureWrite(v);
            return *this;
        }
        chain
        nonSecure(bool v = true) const
        {
            userNonSecureRead(v);
            userNonSecureWrite(v);
            privNonSecureRead(v);
            privNonSecureWrite(v);
            hypRead(v);
            hypWrite(v);
            monNonSecureRead(v);
            monNonSecureWrite(v);
            return *this;
        }
        chain
        secure(bool v = true) const
        {
            userSecureRead(v);
            userSecureWrite(v);
            privSecureRead(v);
            privSecureWrite(v);
            monSecureRead(v);
            monSecureWrite(v);
            return *this;
        }
        chain
        reads(bool v) const
        {
            userNonSecureRead(v);
            userSecureRead(v);
            privNonSecureRead(v);
            privSecureRead(v);
            hypRead(v);
            monSecureRead(v);
            monNonSecureRead(v);
            return *this;
        }
        chain
        writes(bool v) const
        {
            userNonSecureWrite(v);
            userSecureWrite(v);
            privNonSecureWrite(v);
            privSecureWrite(v);
            hypWrite(v);
            monSecureWrite(v);
            monNonSecureWrite(v);
            return *this;
        }
        chain
        exceptUserMode() const
        {
            user(0);
            return *this;
        }
        chain highest(ArmSystem *const sys) const;

        chain
        faultRead(ExceptionLevel el, MiscRegLUTEntry::FaultCB cb) const
        {
            entry.faultRead[el] = cb;
            return *this;
        }

        chain
        faultWrite(ExceptionLevel el, MiscRegLUTEntry::FaultCB cb) const
        {
            entry.faultWrite[el] = cb;
            return *this;
        }

        chain
        fault(ExceptionLevel el, MiscRegLUTEntry::FaultCB cb) const
        {
            return faultRead(el, cb).faultWrite(el, cb);
        }

        chain
        fault(MiscRegLUTEntry::FaultCB cb) const
        {
            return fault(EL0, cb).fault(EL1, cb).fault(EL2, cb).fault(EL3, cb);
        }

        MiscRegLUTEntryInitializer(struct MiscRegLUTEntry &e)
          : entry(e)
        {
            // force unimplemented registers to be thusly declared
            implemented(1).unserialize(1);
        }
    };

    extern std::vector<struct MiscRegLUTEntry> lookUpMiscReg;

    struct MiscRegNum32
    {
        MiscRegNum32(unsigned _coproc, unsigned _opc1,
                     unsigned _crn, unsigned _crm,
                     unsigned _opc2)
          : reg64(0), coproc(_coproc), opc1(_opc1), crn(_crn),
            crm(_crm), opc2(_opc2)
        {
            // MCR/MRC CP14 or CP15 register
            assert(coproc == 0b1110 || coproc == 0b1111);
            assert(opc1 < 8 && crn < 16 && crm < 16 && opc2 < 8);
        }

        MiscRegNum32(unsigned _coproc, unsigned _opc1,
                     unsigned _crm)
          : reg64(1), coproc(_coproc), opc1(_opc1), crn(0),
            crm(_crm), opc2(0)
        {
            // MCRR/MRRC CP14 or CP15 register
            assert(coproc == 0b1110 || coproc == 0b1111);
            assert(opc1 < 16 && crm < 16);
        }

        MiscRegNum32(const MiscRegNum32& rhs) = default;

        bool
        operator==(const MiscRegNum32 &other) const
        {
            return reg64 == other.reg64 &&
                coproc == other.coproc &&
                opc1 == other.opc1 &&
                crn == other.crn &&
                crm == other.crm &&
                opc2 == other.opc2;
        }

        uint32_t
        packed() const
        {
            return reg64 << 19  |
                   coproc << 15 |
                   opc1 << 11   |
                   crn << 7     |
                   crm << 3     |
                   opc2;
        }

        // 1 if the register is 64bit wide (accessed through MCRR/MRCC)
        // 0 otherwise. We need this when generating the hash as there
        // might be collisions between 32 and 64 bit registers
        const unsigned reg64;

        unsigned coproc;
        unsigned opc1;
        unsigned crn;
        unsigned crm;
        unsigned opc2;
    };

    struct MiscRegNum64
    {
        MiscRegNum64(unsigned _op0, unsigned _op1,
                     unsigned _crn, unsigned _crm,
                     unsigned _op2)
          : op0(_op0), op1(_op1), crn(_crn),
            crm(_crm), op2(_op2)
        {
            assert(op0 < 4 && op1 < 8 && crn < 16 && crm < 16 && op2 < 8);
        }

        MiscRegNum64(const MiscRegNum64& rhs) = default;

        bool
        operator==(const MiscRegNum64 &other) const
        {
            return op0 == other.op0 &&
                op1 == other.op1 &&
                crn == other.crn &&
                crm == other.crm &&
                op2 == other.op2;
        }

        uint32_t
        packed() const
        {
            return op0 << 14 |
                   op1 << 11 |
                   crn << 7  |
                   crm << 3  |
                   op2;
        }

        unsigned op0;
        unsigned op1;
        unsigned crn;
        unsigned crm;
        unsigned op2;
    };

    // Decodes 32-bit CP14 registers accessible through MCR/MRC instructions
    MiscRegIndex decodeCP14Reg(unsigned crn, unsigned opc1,
                               unsigned crm, unsigned opc2);
    MiscRegIndex decodeAArch64SysReg(unsigned op0, unsigned op1,
                                     unsigned crn, unsigned crm,
                                     unsigned op2);
    MiscRegIndex decodeAArch64SysReg(const MiscRegNum64 &misc_reg);
    std::optional<MiscRegNum64> encodeAArch64SysReg(MiscRegIndex misc_reg);

    // Whether a particular AArch64 system register is -always- read only.
    bool aarch64SysRegReadOnly(MiscRegIndex miscReg);

    // Decodes 32-bit CP15 registers accessible through MCR/MRC instructions
    MiscRegIndex decodeCP15Reg(unsigned crn, unsigned opc1,
                               unsigned crm, unsigned opc2);

    // Decodes 64-bit CP15 registers accessible through MCRR/MRRC instructions
    MiscRegIndex decodeCP15Reg64(unsigned crm, unsigned opc1);


    const char * const miscRegName[] = {
        "cpsr",
        "spsr",
        "spsr_fiq",
        "spsr_irq",
        "spsr_svc",
        "spsr_mon",
        "spsr_abt",
        "spsr_hyp",
        "spsr_und",
        "elr_hyp",
        "fpsid",
        "fpscr",
        "mvfr1",
        "mvfr0",
        "fpexc",

        // Helper registers
        "cpsr_mode",
        "cpsr_q",
        "fpscr_exc",
        "fpscr_qc",
        "lockaddr",
        "lockflag",
        "prrr_mair0",
        "prrr_mair0_ns",
        "prrr_mair0_s",
        "nmrr_mair1",
        "nmrr_mair1_ns",
        "nmrr_mair1_s",
        "pmxevtyper_pmccfiltr",
        "sev_mailbox",
        "tlbi_needsync",

        // AArch32 CP14 registers
        "dbgdidr",
        "dbgdscrint",
        "dbgdccint",
        "dbgdtrtxint",
        "dbgdtrrxint",
        "dbgwfar",
        "dbgvcr",
        "dbgdtrrxext",
        "dbgdscrext",
        "dbgdtrtxext",
        "dbgoseccr",
        "dbgbvr0",
        "dbgbvr1",
        "dbgbvr2",
        "dbgbvr3",
        "dbgbvr4",
        "dbgbvr5",
        "dbgbvr6",
        "dbgbvr7",
        "dbgbvr8",
        "dbgbvr9",
        "dbgbvr10",
        "dbgbvr11",
        "dbgbvr12",
        "dbgbvr13",
        "dbgbvr14",
        "dbgbvr15",
        "dbgbcr0",
        "dbgbcr1",
        "dbgbcr2",
        "dbgbcr3",
        "dbgbcr4",
        "dbgbcr5",
        "dbgbcr6",
        "dbgbcr7",
        "dbgbcr8",
        "dbgbcr9",
        "dbgbcr10",
        "dbgbcr11",
        "dbgbcr12",
        "dbgbcr13",
        "dbgbcr14",
        "dbgbcr15",
        "dbgwvr0",
        "dbgwvr1",
        "dbgwvr2",
        "dbgwvr3",
        "dbgwvr4",
        "dbgwvr5",
        "dbgwvr6",
        "dbgwvr7",
        "dbgwvr8",
        "dbgwvr9",
        "dbgwvr10",
        "dbgwvr11",
        "dbgwvr12",
        "dbgwvr13",
        "dbgwvr14",
        "dbgwvr15",
        "dbgwcr0",
        "dbgwcr1",
        "dbgwcr2",
        "dbgwcr3",
        "dbgwcr4",
        "dbgwcr5",
        "dbgwcr6",
        "dbgwcr7",
        "dbgwcr8",
        "dbgwcr9",
        "dbgwcr10",
        "dbgwcr11",
        "dbgwcr12",
        "dbgwcr13",
        "dbgwcr14",
        "dbgwcr15",
        "dbgdrar",
        "dbgbxvr0",
        "dbgbxvr1",
        "dbgbxvr2",
        "dbgbxvr3",
        "dbgbxvr4",
        "dbgbxvr5",
        "dbgbxvr6",
        "dbgbxvr7",
        "dbgbxvr8",
        "dbgbxvr9",
        "dbgbxvr10",
        "dbgbxvr11",
        "dbgbxvr12",
        "dbgbxvr13",
        "dbgbxvr14",
        "dbgbxvr15",
        "dbgoslar",
        "dbgoslsr",
        "dbgosdlr",
        "dbgprcr",
        "dbgdsar",
        "dbgclaimset",
        "dbgclaimclr",
        "dbgauthstatus",
        "dbgdevid2",
        "dbgdevid1",
        "dbgdevid0",
        "teecr",
        "jidr",
        "teehbr",
        "joscr",
        "jmcr",

        // AArch32 CP15 registers
        "midr",
        "ctr",
        "tcmtr",
        "tlbtr",
        "mpidr",
        "revidr",
        "id_pfr0",
        "id_pfr1",
        "id_dfr0",
        "id_afr0",
        "id_mmfr0",
        "id_mmfr1",
        "id_mmfr2",
        "id_mmfr3",
        "id_mmfr4",
        "id_isar0",
        "id_isar1",
        "id_isar2",
        "id_isar3",
        "id_isar4",
        "id_isar5",
        "id_isar6",
        "ccsidr",
        "clidr",
        "aidr",
        "csselr",
        "csselr_ns",
        "csselr_s",
        "vpidr",
        "vmpidr",
        "sctlr",
        "sctlr_ns",
        "sctlr_s",
        "actlr",
        "actlr_ns",
        "actlr_s",
        "cpacr",
        "sdcr",
        "scr",
        "sder",
        "nsacr",
        "hsctlr",
        "hactlr",
        "hcr",
        "hcr2",
        "hdcr",
        "hcptr",
        "hstr",
        "hacr",
        "ttbr0",
        "ttbr0_ns",
        "ttbr0_s",
        "ttbr1",
        "ttbr1_ns",
        "ttbr1_s",
        "ttbcr",
        "ttbcr_ns",
        "ttbcr_s",
        "htcr",
        "vtcr",
        "dacr",
        "dacr_ns",
        "dacr_s",
        "dfsr",
        "dfsr_ns",
        "dfsr_s",
        "ifsr",
        "ifsr_ns",
        "ifsr_s",
        "adfsr",
        "adfsr_ns",
        "adfsr_s",
        "aifsr",
        "aifsr_ns",
        "aifsr_s",
        "hadfsr",
        "haifsr",
        "hsr",
        "dfar",
        "dfar_ns",
        "dfar_s",
        "ifar",
        "ifar_ns",
        "ifar_s",
        "hdfar",
        "hifar",
        "hpfar",
        "icialluis",
        "bpiallis",
        "par",
        "par_ns",
        "par_s",
        "iciallu",
        "icimvau",
        "cp15isb",
        "bpiall",
        "bpimva",
        "dcimvac",
        "dcisw",
        "ats1cpr",
        "ats1cpw",
        "ats1cur",
        "ats1cuw",
        "ats12nsopr",
        "ats12nsopw",
        "ats12nsour",
        "ats12nsouw",
        "dccmvac",
        "dccsw",
        "cp15dsb",
        "cp15dmb",
        "dccmvau",
        "dccimvac",
        "dccisw",
        "ats1hr",
        "ats1hw",
        "tlbiallis",
        "tlbimvais",
        "tlbiasidis",
        "tlbimvaais",
        "tlbimvalis",
        "tlbimvaalis",
        "itlbiall",
        "itlbimva",
        "itlbiasid",
        "dtlbiall",
        "dtlbimva",
        "dtlbiasid",
        "tlbiall",
        "tlbimva",
        "tlbiasid",
        "tlbimvaa",
        "tlbimval",
        "tlbimvaal",
        "tlbiipas2is",
        "tlbiipas2lis",
        "tlbiallhis",
        "tlbimvahis",
        "tlbiallnsnhis",
        "tlbimvalhis",
        "tlbiipas2",
        "tlbiipas2l",
        "tlbiallh",
        "tlbimvah",
        "tlbiallnsnh",
        "tlbimvalh",
        "pmcr",
        "pmcntenset",
        "pmcntenclr",
        "pmovsr",
        "pmswinc",
        "pmselr",
        "pmceid0",
        "pmceid1",
        "pmccntr",
        "pmxevtyper",
        "pmccfiltr",
        "pmxevcntr",
        "pmuserenr",
        "pmintenset",
        "pmintenclr",
        "pmovsset",
        "l2ctlr",
        "l2ectlr",
        "prrr",
        "prrr_ns",
        "prrr_s",
        "mair0",
        "mair0_ns",
        "mair0_s",
        "nmrr",
        "nmrr_ns",
        "nmrr_s",
        "mair1",
        "mair1_ns",
        "mair1_s",
        "amair0",
        "amair0_ns",
        "amair0_s",
        "amair1",
        "amair1_ns",
        "amair1_s",
        "hmair0",
        "hmair1",
        "hamair0",
        "hamair1",
        "vbar",
        "vbar_ns",
        "vbar_s",
        "mvbar",
        "rmr",
        "isr",
        "hvbar",
        "fcseidr",
        "contextidr",
        "contextidr_ns",
        "contextidr_s",
        "tpidrurw",
        "tpidrurw_ns",
        "tpidrurw_s",
        "tpidruro",
        "tpidruro_ns",
        "tpidruro_s",
        "tpidrprw",
        "tpidrprw_ns",
        "tpidrprw_s",
        "htpidr",
        "cntfrq",
        "cntpct",
        "cntvct",
        "cntp_ctl",
        "cntp_ctl_ns",
        "cntp_ctl_s",
        "cntp_cval",
        "cntp_cval_ns",
        "cntp_cval_s",
        "cntp_tval",
        "cntp_tval_ns",
        "cntp_tval_s",
        "cntv_ctl",
        "cntv_cval",
        "cntv_tval",
        "cntkctl",
        "cnthctl",
        "cnthp_ctl",
        "cnthp_cval",
        "cnthp_tval",
        "cntvoff",
        "il1data0",
        "il1data1",
        "il1data2",
        "il1data3",
        "dl1data0",
        "dl1data1",
        "dl1data2",
        "dl1data3",
        "dl1data4",
        "ramindex",
        "l2actlr",
        "cbar",
        "httbr",
        "vttbr",
        "cpumerrsr",
        "l2merrsr",

        // AArch64 registers (Op0=2)
        "mdccint_el1",
        "osdtrrx_el1",
        "mdscr_el1",
        "osdtrtx_el1",
        "oseccr_el1",
        "dbgbvr0_el1",
        "dbgbvr1_el1",
        "dbgbvr2_el1",
        "dbgbvr3_el1",
        "dbgbvr4_el1",
        "dbgbvr5_el1",
        "dbgbvr6_el1",
        "dbgbvr7_el1",
        "dbgbvr8_el1",
        "dbgbvr9_el1",
        "dbgbvr10_el1",
        "dbgbvr11_el1",
        "dbgbvr12_el1",
        "dbgbvr13_el1",
        "dbgbvr14_el1",
        "dbgbvr15_el1",
        "dbgbcr0_el1",
        "dbgbcr1_el1",
        "dbgbcr2_el1",
        "dbgbcr3_el1",
        "dbgbcr4_el1",
        "dbgbcr5_el1",
        "dbgbcr6_el1",
        "dbgbcr7_el1",
        "dbgbcr8_el1",
        "dbgbcr9_el1",
        "dbgbcr10_el1",
        "dbgbcr11_el1",
        "dbgbcr12_el1",
        "dbgbcr13_el1",
        "dbgbcr14_el1",
        "dbgbcr15_el1",
        "dbgwvr0_el1",
        "dbgwvr1_el1",
        "dbgwvr2_el1",
        "dbgwvr3_el1",
        "dbgwvr4_el1",
        "dbgwvr5_el1",
        "dbgwvr6_el1",
        "dbgwvr7_el1",
        "dbgwvr8_el1",
        "dbgwvr9_el1",
        "dbgwvr10_el1",
        "dbgwvr11_el1",
        "dbgwvr12_el1",
        "dbgwvr13_el1",
        "dbgwvr14_el1",
        "dbgwvr15_el1",
        "dbgwcr0_el1",
        "dbgwcr1_el1",
        "dbgwcr2_el1",
        "dbgwcr3_el1",
        "dbgwcr4_el1",
        "dbgwcr5_el1",
        "dbgwcr6_el1",
        "dbgwcr7_el1",
        "dbgwcr8_el1",
        "dbgwcr9_el1",
        "dbgwcr10_el1",
        "dbgwcr11_el1",
        "dbgwcr12_el1",
        "dbgwcr13_el1",
        "dbgwcr14_el1",
        "dbgwcr15_el1",
        "mdccsr_el0",
        "mddtr_el0",
        "mddtrtx_el0",
        "mddtrrx_el0",
        "dbgvcr32_el2",
        "mdrar_el1",
        "oslar_el1",
        "oslsr_el1",
        "osdlr_el1",
        "dbgprcr_el1",
        "dbgclaimset_el1",
        "dbgclaimclr_el1",
        "dbgauthstatus_el1",
        "teecr32_el1",
        "teehbr32_el1",

        // AArch64 registers (Op0=1,3)
        "midr_el1",
        "mpidr_el1",
        "revidr_el1",
        "id_pfr0_el1",
        "id_pfr1_el1",
        "id_dfr0_el1",
        "id_afr0_el1",
        "id_mmfr0_el1",
        "id_mmfr1_el1",
        "id_mmfr2_el1",
        "id_mmfr3_el1",
        "id_mmfr4_el1",
        "id_isar0_el1",
        "id_isar1_el1",
        "id_isar2_el1",
        "id_isar3_el1",
        "id_isar4_el1",
        "id_isar5_el1",
        "id_isar6_el1",
        "mvfr0_el1",
        "mvfr1_el1",
        "mvfr2_el1",
        "id_aa64pfr0_el1",
        "id_aa64pfr1_el1",
        "id_aa64dfr0_el1",
        "id_aa64dfr1_el1",
        "id_aa64afr0_el1",
        "id_aa64afr1_el1",
        "id_aa64isar0_el1",
        "id_aa64isar1_el1",
        "id_aa64mmfr0_el1",
        "id_aa64mmfr1_el1",
        "ccsidr_el1",
        "clidr_el1",
        "aidr_el1",
        "csselr_el1",
        "ctr_el0",
        "dczid_el0",
        "vpidr_el2",
        "vmpidr_el2",
        "sctlr_el1",
        "sctlr_el12",
        "sctlr2_el1",
        "sctlr2_el12",
        "actlr_el1",
        "cpacr_el1",
        "cpacr_el12",
        "sctlr_el2",
        "sctlr2_el2",
        "actlr_el2",
        "hcr_el2",
        "hcrx_el2",
        "mdcr_el2",
        "cptr_el2",
        "hstr_el2",
        "hacr_el2",
        "sctlr_el3",
        "sctlr2_el3",
        "actlr_el3",
        "scr_el3",
        "sder32_el3",
        "cptr_el3",
        "mdcr_el3",
        "ttbr0_el1",
        "ttbr0_el12",
        "ttbr1_el1",
        "ttbr1_el12",
        "tcr_el1",
        "tcr_el12",
        "tcr2_el1",
        "tcr2_el12",
        "ttbr0_el2",
        "tcr_el2",
        "tcr2_el2",
        "vttbr_el2",
        "vtcr_el2",
        "vsttbr_el2",
        "vstcr_el2",
        "ttbr0_el3",
        "tcr_el3",
        "dacr32_el2",
        "spsr_el1",
        "spsr_el12",
        "elr_el1",
        "elr_el12",
        "sp_el0",
        "spsel",
        "currentel",
        "nzcv",
        "daif",
        "fpcr",
        "fpsr",
        "dspsr_el0",
        "dlr_el0",
        "spsr_el2",
        "elr_el2",
        "sp_el1",
        "spsr_irq_aa64",
        "spsr_abt_aa64",
        "spsr_und_aa64",
        "spsr_fiq_aa64",
        "spsr_el3",
        "elr_el3",
        "sp_el2",
        "afsr0_el1",
        "afsr0_el12",
        "afsr1_el1",
        "afsr1_el12",
        "esr_el1",
        "esr_el12",
        "ifsr32_el2",
        "afsr0_el2",
        "afsr1_el2",
        "esr_el2",
        "fpexc32_el2",
        "afsr0_el3",
        "afsr1_el3",
        "esr_el3",
        "far_el1",
        "far_el12",
        "far_el2",
        "hpfar_el2",
        "far_el3",
        "ic_ialluis",
        "par_el1",
        "ic_iallu",
        "dc_ivac_xt",
        "dc_isw_xt",
        "at_s1e1r_xt",
        "at_s1e1w_xt",
        "at_s1e0r_xt",
        "at_s1e0w_xt",
        "dc_csw_xt",
        "dc_cisw_xt",
        "dc_zva_xt",
        "ic_ivau_xt",
        "dc_cvac_xt",
        "dc_cvau_xt",
        "dc_civac_xt",
        "at_s1e2r_xt",
        "at_s1e2w_xt",
        "at_s12e1r_xt",
        "at_s12e1w_xt",
        "at_s12e0r_xt",
        "at_s12e0w_xt",
        "at_s1e3r_xt",
        "at_s1e3w_xt",
        "tlbi_vmalle1is",
        "tlbi_vmalle1os",
        "tlbi_vae1is",
        "tlbi_vae1os",
        "lbi_aside1is_xt",
        "tlbi_aside1os",
        "tlbi_vaae1is",
        "tlbi_vaae1os",
        "tlbi_vale1is",
        "tlbi_vale1os",
        "tlbi_vaale1is",
        "tlbi_vaale1os",
        "tlbi_vmalle1",
        "tlbi_vae1",
        "tlbi_aside1",
        "tlbi_vaae1",
        "tlbi_vale1",
        "tlbi_vaale1",
        "tlbi_ipas2e1is",
        "tlbi_ipas2e1os",
        "tlbi_ipas2le1is",
        "tlbi_ipas2le1os",
        "tlbi_alle2is",
        "tlbi_alle2os",
        "tlbi_vae2is",
        "tlbi_vae2os",
        "tlbi_alle1is",
        "tlbi_alle1os",
        "tlbi_vale2is",
        "tlbi_vale2os",
        "tlbi_vmalls12e1is",
        "tlbi_vmalls12e1os",
        "tlbi_ipas2e1",
        "tlbi_ipas2le1",
        "tlbi_alle2",
        "tlbi_vae2",
        "tlbi_alle1",
        "tlbi_vale2",
        "tlbi_vmalls12e1",
        "tlbi_alle3is",
        "tlbi_alle3os",
        "tlbi_vae3is",
        "tlbi_vae3os",
        "tlbi_vale3is",
        "tlbi_vale3os",
        "tlbi_alle3",
        "tlbi_vae3",
        "tlbi_vale3",
        "tlbi_rvae1",
        "tlbi_rvaae1",
        "tlbi_rvale1",
        "tlbi_rvaale1",
        "tlbi_ripas2e1",
        "tlbi_ripas2le1",
        "tlbi_rvae2",
        "tlbi_rvale2",
        "tlbi_rvae3",
        "tlbi_rvale3",
        "tlbi_rvae1is",
        "tlbi_rvaae1is",
        "tlbi_rvale1is",
        "tlbi_rvaale1is",
        "tlbi_ripas2e1is",
        "tlbi_ripas2le1is",
        "tlbi_rvae2is",
        "tlbi_rvale2is",
        "tlbi_rvae3is",
        "tlbi_rvale3is",
        "tlbi_rvae1os",
        "tlbi_rvaae1os",
        "tlbi_rvale1os",
        "tlbi_rvaale1os",
        "tlbi_ripas2e1os",
        "tlbi_ripas2le1os",
        "tlbi_rvae2os",
        "tlbi_rvale2os",
        "tlbi_rvae3os",
        "tlbi_rvale3os",
        "pmintenset_el1",
        "pmintenclr_el1",
        "pmcr_el0",
        "pmcntenset_el0",
        "pmcntenclr_el0",
        "pmovsclr_el0",
        "pmswinc_el0",
        "pmselr_el0",
        "pmceid0_el0",
        "pmceid1_el0",
        "pmccntr_el0",
        "pmxevtyper_el0",
        "pmccfiltr_el0",
        "pmxevcntr_el0",
        "pmuserenr_el0",
        "pmovsset_el0",
        "mair_el1",
        "mair_el12",
        "amair_el1",
        "amair_el12",
        "mair_el2",
        "amair_el2",
        "mair_el3",
        "amair_el3",
        "l2ctlr_el1",
        "l2ectlr_el1",
        "vbar_el1",
        "vbar_el12",
        "rvbar_el1",
        "isr_el1",
        "vbar_el2",
        "rvbar_el2",
        "vbar_el3",
        "rvbar_el3",
        "rmr_el3",
        "contextidr_el1",
        "contextidr_el12",
        "tpidr_el1",
        "tpidr_el0",
        "tpidrro_el0",
        "tpidr_el2",
        "tpidr_el3",
        "cntfrq_el0",
        "cntpct_el0",
        "cntvct_el0",
        "cntp_ctl_el0",
        "cntp_cval_el0",
        "cntp_tval_el0",
        "cntv_ctl_el0",
        "cntv_cval_el0",
        "cntv_tval_el0",
        "cntp_ctl_el02",
        "cntp_cval_el02",
        "cntp_tval_el02",
        "cntv_ctl_el02",
        "cntv_cval_el02",
        "cntv_tval_el02",
        "cntkctl_el1",
        "cntkctl_el12",
        "cntps_ctl_el1",
        "cntps_cval_el1",
        "cntps_tval_el1",
        "cnthctl_el2",
        "cnthp_ctl_el2",
        "cnthp_cval_el2",
        "cnthp_tval_el2",
        "cnthps_ctl_el2",
        "cnthps_cval_el2",
        "cnthps_tval_el2",
        "cnthv_ctl_el2",
        "cnthv_cval_el2",
        "cnthv_tval_el2",
        "cnthvs_ctl_el2",
        "cnthvs_cval_el2",
        "cnthvs_tval_el2",
        "cntvoff_el2",
        "pmevcntr0_el0",
        "pmevcntr1_el0",
        "pmevcntr2_el0",
        "pmevcntr3_el0",
        "pmevcntr4_el0",
        "pmevcntr5_el0",
        "pmevtyper0_el0",
        "pmevtyper1_el0",
        "pmevtyper2_el0",
        "pmevtyper3_el0",
        "pmevtyper4_el0",
        "pmevtyper5_el0",
        "il1data0_el1",
        "il1data1_el1",
        "il1data2_el1",
        "il1data3_el1",
        "dl1data0_el1",
        "dl1data1_el1",
        "dl1data2_el1",
        "dl1data3_el1",
        "dl1data4_el1",
        "l2actlr_el1",
        "cpuactlr_el1",
        "cpuectlr_el1",
        "cpumerrsr_el1",
        "l2merrsr_el1",
        "cbar_el1",
        "contextidr_el2",

        "ttbr1_el2",
        "id_aa64mmfr2_el1",
        "id_aa64mmfr3_el1",

        "apdakeyhi_el1",
        "apdakeylo_el1",
        "apdbkeyhi_el1",
        "apdbkeylo_el1",
        "apgakeyhi_el1",
        "apgakeylo_el1",
        "apiakeyhi_el1",
        "apiakeylo_el1",
        "apibkeyhi_el1",
        "apibkeylo_el1",
        // GICv3, CPU interface
        "icc_pmr_el1",
        "icc_iar0_el1",
        "icc_eoir0_el1",
        "icc_hppir0_el1",
        "icc_bpr0_el1",
        "icc_ap0r0_el1",
        "icc_ap0r1_el1",
        "icc_ap0r2_el1",
        "icc_ap0r3_el1",
        "icc_ap1r0_el1",
        "icc_ap1r0_el1_ns",
        "icc_ap1r0_el1_s",
        "icc_ap1r1_el1",
        "icc_ap1r1_el1_ns",
        "icc_ap1r1_el1_s",
        "icc_ap1r2_el1",
        "icc_ap1r2_el1_ns",
        "icc_ap1r2_el1_s",
        "icc_ap1r3_el1",
        "icc_ap1r3_el1_ns",
        "icc_ap1r3_el1_s",
        "icc_dir_el1",
        "icc_rpr_el1",
        "icc_sgi1r_el1",
        "icc_asgi1r_el1",
        "icc_sgi0r_el1",
        "icc_iar1_el1",
        "icc_eoir1_el1",
        "icc_hppir1_el1",
        "icc_bpr1_el1",
        "icc_bpr1_el1_ns",
        "icc_bpr1_el1_s",
        "icc_ctlr_el1",
        "icc_ctlr_el1_ns",
        "icc_ctlr_el1_s",
        "icc_sre_el1",
        "icc_sre_el1_ns",
        "icc_sre_el1_s",
        "icc_igrpen0_el1",
        "icc_igrpen1_el1",
        "icc_igrpen1_el1_ns",
        "icc_igrpen1_el1_s",
        "icc_sre_el2",
        "icc_ctlr_el3",
        "icc_sre_el3",
        "icc_igrpen1_el3",

        // GICv3, CPU interface, virtualization
        "ich_ap0r0_el2",
        "ich_ap0r1_el2",
        "ich_ap0r2_el2",
        "ich_ap0r3_el2",
        "ich_ap1r0_el2",
        "ich_ap1r1_el2",
        "ich_ap1r2_el2",
        "ich_ap1r3_el2",
        "ich_hcr_el2",
        "ich_vtr_el2",
        "ich_misr_el2",
        "ich_eisr_el2",
        "ich_elrsr_el2",
        "ich_vmcr_el2",
        "ich_lr0_el2",
        "ich_lr1_el2",
        "ich_lr2_el2",
        "ich_lr3_el2",
        "ich_lr4_el2",
        "ich_lr5_el2",
        "ich_lr6_el2",
        "ich_lr7_el2",
        "ich_lr8_el2",
        "ich_lr9_el2",
        "ich_lr10_el2",
        "ich_lr11_el2",
        "ich_lr12_el2",
        "ich_lr13_el2",
        "ich_lr14_el2",
        "ich_lr15_el2",

        "icv_pmr_el1",
        "icv_iar0_el1",
        "icv_eoir0_el1",
        "icv_hppir0_el1",
        "icv_bpr0_el1",
        "icv_ap0r0_el1",
        "icv_ap0r1_el1",
        "icv_ap0r2_el1",
        "icv_ap0r3_el1",
        "icv_ap1r0_el1",
        "icv_ap1r0_el1_ns",
        "icv_ap1r0_el1_s",
        "icv_ap1r1_el1",
        "icv_ap1r1_el1_ns",
        "icv_ap1r1_el1_s",
        "icv_ap1r2_el1",
        "icv_ap1r2_el1_ns",
        "icv_ap1r2_el1_s",
        "icv_ap1r3_el1",
        "icv_ap1r3_el1_ns",
        "icv_ap1r3_el1_s",
        "icv_dir_el1",
        "icv_rpr_el1",
        "icv_sgi1r_el1",
        "icv_asgi1r_el1",
        "icv_sgi0r_el1",
        "icv_iar1_el1",
        "icv_eoir1_el1",
        "icv_hppir1_el1",
        "icv_bpr1_el1",
        "icv_bpr1_el1_ns",
        "icv_bpr1_el1_s",
        "icv_ctlr_el1",
        "icv_ctlr_el1_ns",
        "icv_ctlr_el1_s",
        "icv_sre_el1",
        "icv_sre_el1_ns",
        "icv_sre_el1_s",
        "icv_igrpen0_el1",
        "icv_igrpen1_el1",
        "icv_igrpen1_el1_ns",
        "icv_igrpen1_el1_s",

        "icc_ap0r0",
        "icc_ap0r1",
        "icc_ap0r2",
        "icc_ap0r3",
        "icc_ap1r0",
        "icc_ap1r0_ns",
        "icc_ap1r0_s",
        "icc_ap1r1",
        "icc_ap1r1_ns",
        "icc_ap1r1_s",
        "icc_ap1r2",
        "icc_ap1r2_ns",
        "icc_ap1r2_s",
        "icc_ap1r3",
        "icc_ap1r3_ns",
        "icc_ap1r3_s",
        "icc_asgi1r",
        "icc_bpr0",
        "icc_bpr1",
        "icc_bpr1_ns",
        "icc_bpr1_s",
        "icc_ctlr",
        "icc_ctlr_ns",
        "icc_ctlr_s",
        "icc_dir",
        "icc_eoir0",
        "icc_eoir1",
        "icc_hppir0",
        "icc_hppir1",
        "icc_hsre",
        "icc_iar0",
        "icc_iar1",
        "icc_igrpen0",
        "icc_igrpen1",
        "icc_igrpen1_ns",
        "icc_igrpen1_s",
        "icc_mctlr",
        "icc_mgrpen1",
        "icc_msre",
        "icc_pmr",
        "icc_rpr",
        "icc_sgi0r",
        "icc_sgi1r",
        "icc_sre",
        "icc_sre_ns",
        "icc_sre_s",

        "ich_ap0r0",
        "ich_ap0r1",
        "ich_ap0r2",
        "ich_ap0r3",
        "ich_ap1r0",
        "ich_ap1r1",
        "ich_ap1r2",
        "ich_ap1r3",
        "ich_hcr",
        "ich_vtr",
        "ich_misr",
        "ich_eisr",
        "ich_elrsr",
        "ich_vmcr",
        "ich_lr0",
        "ich_lr1",
        "ich_lr2",
        "ich_lr3",
        "ich_lr4",
        "ich_lr5",
        "ich_lr6",
        "ich_lr7",
        "ich_lr8",
        "ich_lr9",
        "ich_lr10",
        "ich_lr11",
        "ich_lr12",
        "ich_lr13",
        "ich_lr14",
        "ich_lr15",
        "ich_lrc0",
        "ich_lrc1",
        "ich_lrc2",
        "ich_lrc3",
        "ich_lrc4",
        "ich_lrc5",
        "ich_lrc6",
        "ich_lrc7",
        "ich_lrc8",
        "ich_lrc9",
        "ich_lrc10",
        "ich_lrc11",
        "ich_lrc12",
        "ich_lrc13",
        "ich_lrc14",
        "ich_lrc15",

        "id_aa64zfr0_el1",
        "zcr_el3",
        "zcr_el2",
        "zcr_el12",
        "zcr_el1",

        "id_aa64smfr0_el1",
        "svcr",
        "smidr_el1",
        "smpri_el1",
        "smprimap_el2",
        "smcr_el3",
        "smcr_el2",
        "smcr_el12",
        "smcr_el1",
        "tpidr2_el0",
        "mpamsm_el1",

        "rndr",
        "rndrrs",

        "hfgitr_el2",
        "hfgrtr_el2",
        "hfgwtr_el2",
        "hdfgrtr_el2",
        "hdfgwtr_el2",

        // FEAT_MPAM
        "mpamidr_el1",
        "mpam0_el1",
        "mpam1_el1",
        "mpam2_el2",
        "mpam3_el3",
        "mpam1_el12",
        "mpamhcr_el2",
        "mpamvpmv_el2",
        "mpamvpm0_el2",
        "mpamvpm1_el2",
        "mpamvpm2_el2",
        "mpamvpm3_el2",
        "mpamvpm4_el2",
        "mpamvpm5_el2",
        "mpamvpm6_el2",
        "mpamvpm7_el2",

        "num_phys_regs",

        // Dummy registers
        "nop",
        "raz",
        "unknown",
        "impl_defined",
        "erridr_el1",
        "errselr_el1",
        "erxfr_el1",
        "erxctlr_el1",
        "erxstatus_el1",
        "erxaddr_el1",
        "erxmisc0_el1",
        "erxmisc1_el1",
        "disr_el1",
        "vsesr_el2",
        "vdisr_el2",

        // PSTATE
        "pan",
        "uao",
    };

    static_assert(sizeof(miscRegName) / sizeof(*miscRegName) == NUM_MISCREGS,
                  "The miscRegName array and NUM_MISCREGS are inconsistent.");

    class MiscRegClassOps : public RegClassOps
    {
      public:
        std::string
        regName(const RegId &id) const override
        {
            return miscRegName[id.index()];
        }
    };

    static inline MiscRegClassOps miscRegClassOps;

    inline constexpr RegClass miscRegClass =
        RegClass(MiscRegClass, MiscRegClassName, NUM_MISCREGS,
                debug::MiscRegs).
            ops(miscRegClassOps);

    // This mask selects bits of the CPSR that actually go in the CondCodes
    // integer register to allow renaming.
    static const uint32_t CondCodesMask   = 0xF00F0000;
    static const uint32_t CpsrMaskQ       = 0x08000000;

    // APSR (Application Program Status Register Mask). It is the user level
    // alias for the CPSR. The APSR is a subset of the CPSR. Although
    // bits[15:0] are UNKNOWN on reads, it is permitted that, on a read of
    // APSR:
    // Bit[9] returns the value of CPSR.E.
    // Bits[8:6] return the value of CPSR.{A,I, F}, the mask bits.
    static const uint32_t ApsrMask = CpsrMaskQ | CondCodesMask | 0x000001D0;

    // CPSR (Current Program Status Register Mask).
    static const uint32_t CpsrMask = ApsrMask | 0x00F003DF;

    // This mask selects bits of the FPSCR that actually go in the FpCondCodes
    // integer register to allow renaming.
    static const uint32_t FpCondCodesMask = 0xF0000000;
    // This mask selects the cumulative saturation flag of the FPSCR.
    static const uint32_t FpscrQcMask = 0x08000000;
    // This mask selects the AHP bit of the FPSCR.
    static const uint32_t FpscrAhpMask = 0x04000000;
    // This mask selects the cumulative FP exception flags of the FPSCR.
    static const uint32_t FpscrExcMask = 0x0000009F;

    /**
     * Check for permission to read coprocessor registers.
     *
     * Checks whether an instruction at the current program mode has
     * permissions to read the coprocessor registers. This function
     * returns whether the check is undefined and if not whether the
     * read access is permitted.
     *
     * @param the misc reg indicating the coprocessor
     * @param the SCR
     * @param the CPSR
     * @param the thread context on the core
     * @return a tuple of booleans: can_read, undefined
     */
    std::tuple<bool, bool> canReadCoprocReg(MiscRegIndex reg, SCR scr,
                                            CPSR cpsr, ThreadContext *tc);

    /**
     * Check for permission to write coprocessor registers.
     *
     * Checks whether an instruction at the current program mode has
     * permissions to write the coprocessor registers. This function
     * returns whether the check is undefined and if not whether the
     * write access is permitted.
     *
     * @param the misc reg indicating the coprocessor
     * @param the SCR
     * @param the CPSR
     * @param the thread context on the core
     * @return a tuple of booleans: can_write, undefined
     */
    std::tuple<bool, bool> canWriteCoprocReg(MiscRegIndex reg, SCR scr,
                                             CPSR cpsr, ThreadContext *tc);

    // Checks for UNDEFINED behaviours when accessing AArch32
    // Generic Timer system registers
    bool AArch32isUndefinedGenericTimer(MiscRegIndex reg, ThreadContext *tc);

    // Checks access permissions to AArch64 system registers
    Fault checkFaultAccessAArch64SysReg(MiscRegIndex reg, CPSR cpsr,
            ThreadContext *tc, const MiscRegOp64 &inst);

    // Uses just the scr.ns bit to pre flatten the misc regs. This is useful
    // for MCR/MRC instructions
    int
    snsBankedIndex(MiscRegIndex reg, ThreadContext *tc);

    // Flattens a misc reg index using the specified security state. This is
    // used for opperations (eg address translations) where the security
    // state of the register access may differ from the current state of the
    // processor
    int
    snsBankedIndex(MiscRegIndex reg, ThreadContext *tc, bool ns);

    int
    snsBankedIndex64(MiscRegIndex reg, ThreadContext *tc);

    // Takes a misc reg index and returns the root reg if its one of a set of
    // banked registers
    void
    preUnflattenMiscReg();

    int
    unflattenMiscReg(int reg);

} // namespace ArmISA
} // namespace gem5

namespace std
{
template<>
struct hash<gem5::ArmISA::MiscRegNum32>
{
    size_t
    operator()(const gem5::ArmISA::MiscRegNum32& reg) const
    {
        return reg.packed();
    }
};

template<>
struct hash<gem5::ArmISA::MiscRegNum64>
{
    size_t
    operator()(const gem5::ArmISA::MiscRegNum64& reg) const
    {
        return reg.packed();
    }
};
} // namespace std

#endif // __ARCH_ARM_REGS_MISC_HH__
