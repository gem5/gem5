/*
 * Copyright (c) 2011-2013,2017-2020 ARM Limited
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

#include "arch/arm/insts/misc64.hh"
#include "arch/arm/isa.hh"

using namespace ArmISA;

std::string
ImmOp64::generateDisassembly(Addr pc, const Loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ccprintf(ss, "#0x%x", imm);
    return ss.str();
}

std::string
RegRegImmImmOp64::generateDisassembly(
        Addr pc, const Loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    printIntReg(ss, dest);
    ss << ", ";
    printIntReg(ss, op1);
    ccprintf(ss, ", #%d, #%d", imm1, imm2);
    return ss.str();
}

std::string
RegRegRegImmOp64::generateDisassembly(
    Addr pc, const Loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    printIntReg(ss, dest);
    ss << ", ";
    printIntReg(ss, op1);
    ss << ", ";
    printIntReg(ss, op2);
    ccprintf(ss, ", #%d", imm);
    return ss.str();
}

std::string
UnknownOp64::generateDisassembly(
        Addr pc, const Loader::SymbolTable *symtab) const
{
    return csprintf("%-10s (inst %#08x)", "unknown", encoding());
}

Fault
MiscRegOp64::trap(ThreadContext *tc, MiscRegIndex misc_reg,
                  ExceptionLevel el, uint32_t immediate) const
{
    ExceptionClass ec = EC_TRAPPED_MSR_MRS_64;

    // Check for traps to supervisor (FP/SIMD regs)
    if (el <= EL1 && checkEL1Trap(tc, misc_reg, el, ec, immediate)) {
        return std::make_shared<SupervisorTrap>(machInst, immediate, ec);
    }

    // Check for traps to hypervisor
    if ((ArmSystem::haveVirtualization(tc) && el <= EL2) &&
        checkEL2Trap(tc, misc_reg, el, ec, immediate)) {
        return std::make_shared<HypervisorTrap>(machInst, immediate, ec);
    }

    // Check for traps to secure monitor
    if ((ArmSystem::haveSecurity(tc) && el <= EL3) &&
        checkEL3Trap(tc, misc_reg, el, ec, immediate)) {
        return std::make_shared<SecureMonitorTrap>(machInst, immediate, ec);
    }

    return NoFault;
}

bool
MiscRegOp64::checkEL1Trap(ThreadContext *tc, const MiscRegIndex misc_reg,
                          ExceptionLevel el, ExceptionClass &ec,
                          uint32_t &immediate) const
{
    const CPACR cpacr = tc->readMiscReg(MISCREG_CPACR_EL1);
    const SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
    const SCTLR sctlr = tc->readMiscReg(MISCREG_SCTLR_EL1);
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);

    bool trap_to_sup = false;
    switch (misc_reg) {
      case MISCREG_DAIF:
        trap_to_sup = !scr.ns && !scr.eel2 && !sctlr.uma && el == EL0;
        trap_to_sup = trap_to_sup ||
            (el == EL0 && (scr.ns || scr.eel2) && !hcr.tge && !sctlr.uma);
        break;
      case MISCREG_DC_ZVA_Xt:
        // In syscall-emulation mode, this test is skipped and DCZVA is always
        // allowed at EL0
        trap_to_sup =  el == EL0 && !sctlr.dze && FullSystem;
        break;
      case MISCREG_DC_CIVAC_Xt:
      case MISCREG_DC_CVAC_Xt:
        trap_to_sup = el == EL0 && !sctlr.uci;
        break;
      case MISCREG_FPCR:
      case MISCREG_FPSR:
      case MISCREG_FPEXC32_EL2:
        if ((el == EL0 && cpacr.fpen != 0x3) ||
            (el == EL1 && !(cpacr.fpen & 0x1))) {
            trap_to_sup = true;
            ec = EC_TRAPPED_SIMD_FP;
            immediate = 0x1E00000;
        }
        break;
      case MISCREG_DC_CVAU_Xt:
        trap_to_sup = !sctlr.uci && (!hcr.tge || (!scr.ns && !scr.eel2)) &&
            el == EL0;
        break;
      case MISCREG_CTR_EL0:
        trap_to_sup = el == EL0 && !sctlr.uct &&
            (!hcr.tge || (!scr.ns && !scr.eel2));
        break;
       case MISCREG_MDCCSR_EL0:
         {
             DBGDS32 mdscr = tc->readMiscReg(MISCREG_MDSCR_EL1);
             trap_to_sup = el == EL0 && mdscr.tdcc &&
                     (hcr.tge == 0x0 || ( scr.ns == 0x0));
         }
         break;
     case MISCREG_ZCR_EL1:
        trap_to_sup = el == EL1 && ((cpacr.zen & 0x1) == 0x0);
        break;
      // Generic Timer
      case MISCREG_CNTFRQ_EL0 ... MISCREG_CNTVOFF_EL2:
        trap_to_sup = el == EL0 &&
                      isGenericTimerSystemAccessTrapEL1(misc_reg, tc);
        break;
      default:
        break;
    }
    return trap_to_sup;
}

bool
MiscRegOp64::checkEL2Trap(ThreadContext *tc, const MiscRegIndex misc_reg,
                          ExceptionLevel el, ExceptionClass &ec,
                          uint32_t &immediate) const
{
    const CPTR cptr = tc->readMiscReg(MISCREG_CPTR_EL2);
    const SCTLR sctlr = tc->readMiscReg(MISCREG_SCTLR_EL1);
    const SCTLR sctlr2 = tc->readMiscReg(MISCREG_SCTLR_EL2);
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    const SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
    const HDCR mdcr = tc->readMiscReg(MISCREG_MDCR_EL3);

    bool trap_to_hyp = false;

    switch (misc_reg) {
      case MISCREG_IMPDEF_UNIMPL:
        trap_to_hyp = EL2Enabled(tc) && hcr.tidcp && el == EL1;
        break;
      // GICv3 regs
      case MISCREG_ICC_SGI0R_EL1:
        {
            auto *isa = static_cast<ArmISA::ISA *>(tc->getIsaPtr());
            if (isa->haveGICv3CpuIfc())
                trap_to_hyp = EL2Enabled(tc) && hcr.fmo && el == EL1;
        }
        break;
      case MISCREG_ICC_SGI1R_EL1:
      case MISCREG_ICC_ASGI1R_EL1:
        {
            auto *isa = static_cast<ArmISA::ISA *>(tc->getIsaPtr());
            if (isa->haveGICv3CpuIfc())
                trap_to_hyp = EL2Enabled(tc) && hcr.imo && el == EL1;
        }
        break;
      case MISCREG_FPCR:
      case MISCREG_FPSR:
      case MISCREG_FPEXC32_EL2:
        {
            bool from_el2 = (el == EL2) && (scr.ns || scr.eel2) &&
                             ELIs64(tc,EL2) &&
                             ((!hcr.e2h && cptr.tfp) ||
                              (hcr.e2h && (cptr.fpen == 0x0 ||
                                           cptr.fpen == 0xa)));
            bool from_el1 = (el == EL1) && hcr.nv &&
                            (!hcr.e2h || (hcr.e2h && !hcr.tge));
            trap_to_hyp = from_el2 || from_el1;
            ec = EC_TRAPPED_SIMD_FP;
            immediate = 0x1E00000;
        }
        break;
      case MISCREG_CPACR_EL1:
            trap_to_hyp =  EL2Enabled(tc) && (el == EL1) && cptr.tcpac;
        break;
      case MISCREG_SCTLR_EL1:
      case MISCREG_TTBR0_EL1:
      case MISCREG_TTBR1_EL1:
      case MISCREG_TCR_EL1:
      case MISCREG_ESR_EL1:
      case MISCREG_FAR_EL1:
      case MISCREG_AFSR0_EL1:
      case MISCREG_AFSR1_EL1:
      case MISCREG_MAIR_EL1:
      case MISCREG_AMAIR_EL1:
      case MISCREG_CONTEXTIDR_EL1:
        {
            bool tvm = miscRead? hcr.trvm: hcr.tvm;
            trap_to_hyp = EL2Enabled(tc) && (el == EL1) && tvm;
        }
        break;
      case MISCREG_CPACR_EL12:
      case MISCREG_SCTLR_EL12:
      case MISCREG_TTBR0_EL12:
      case MISCREG_TTBR1_EL12:
      case MISCREG_TCR_EL12:
      case MISCREG_ESR_EL12:
      case MISCREG_FAR_EL12:
      case MISCREG_AFSR0_EL12:
      case MISCREG_AFSR1_EL12:
      case MISCREG_MAIR_EL12:
      case MISCREG_AMAIR_EL12:
      case MISCREG_CONTEXTIDR_EL12:
      case MISCREG_SPSR_EL12:
      case MISCREG_ELR_EL12:
      case MISCREG_VBAR_EL12:
        trap_to_hyp = EL2Enabled(tc) && (el == EL1) &&
           (hcr.nv && (hcr.nv1 || !hcr.nv2));
        break;
      case MISCREG_TLBI_VMALLE1:
      case MISCREG_TLBI_VAE1_Xt:
      case MISCREG_TLBI_ASIDE1_Xt:
      case MISCREG_TLBI_VAAE1_Xt:
      case MISCREG_TLBI_VALE1_Xt:
      case MISCREG_TLBI_VAALE1_Xt:
//      case MISCREG_TLBI_RVAE1:
//      case MISCREG_TLBI_RVAAE1:
//      case MISCREG_TLBI_RVALE1:
//      case MISCREG_TLBI_RVAALE1:
      case MISCREG_TLBI_VMALLE1IS:
      case MISCREG_TLBI_VAE1IS_Xt:
      case MISCREG_TLBI_ASIDE1IS_Xt:
      case MISCREG_TLBI_VAAE1IS_Xt:
      case MISCREG_TLBI_VALE1IS_Xt:
      case MISCREG_TLBI_VAALE1IS_Xt:
//      case MISCREG_TLBI_RVAE1IS:
//      case MISCREG_TLBI_RVAAE1IS:
//      case MISCREG_TLBI_RVALE1IS:
//      case MISCREG_TLBI_RVAALE1IS:
//      case MISCREG_TLBI_VMALLE1OS:
//      case MISCREG_TLBI_VAE1OS:
//      case MISCREG_TLBI_ASIDE1OS:
//      case MISCREG_TLBI_VAAE1OS:
//      case MISCREG_TLBI_VALE1OS:
//      case MISCREG_TLBI_VAALE1OS:
//      case MISCREG_TLBI_RVAE1OS:
//      case MISCREG_TLBI_RVAAE1OS:
//      case MISCREG_TLBI_RVALE1OS:
//      case MISCREG_TLBI_RVAALE1OS:
        trap_to_hyp = EL2Enabled(tc) && (el == EL1) && hcr.ttlb;
        break;
      case MISCREG_IC_IVAU_Xt:
      case MISCREG_ICIALLU:
      case MISCREG_ICIALLUIS:
        trap_to_hyp = (el == EL1) && EL2Enabled(tc) && hcr.tpu;
        break;
      case MISCREG_DC_CVAU_Xt:
        {
            const bool el2_en = EL2Enabled(tc);
            if (el == EL0 && el2_en) {
                const bool in_host = hcr.e2h && hcr.tge;
                const bool general_trap = el2_en && !in_host && hcr.tge &&
                                          !sctlr.uci;
                const bool tpu_trap = el2_en && !in_host && hcr.tpu;
                const bool host_trap = el2_en && in_host && !sctlr2.uci;
                trap_to_hyp = general_trap || tpu_trap || host_trap;
            }
            else if (el == EL1 && el2_en) {
                trap_to_hyp = hcr.tpu;
            }
        }
        break;
      case MISCREG_DC_IVAC_Xt:
        trap_to_hyp = EL2Enabled(tc) && el == EL1 && hcr.tpc;
        break;
      case MISCREG_DC_CVAC_Xt:
//      case MISCREG_DC_CVAP_Xt:
      case MISCREG_DC_CIVAC_Xt:
        {
            const bool el2_en = EL2Enabled(tc);
            if (el == EL0 && el2_en) {

                const bool in_host = hcr.e2h && hcr.tge;
                const bool general_trap = el2_en && !in_host && hcr.tge &&
                                          !sctlr.uci;
                const bool tpc_trap = el2_en && !in_host && hcr.tpc;
                const bool host_trap = el2_en && in_host && !sctlr2.uci;
                trap_to_hyp = general_trap || tpc_trap || host_trap;
            } else if (el == EL1 && el2_en) {
                trap_to_hyp = hcr.tpc;
            }
        }
        break;
      case MISCREG_DC_ISW_Xt:
      case MISCREG_DC_CSW_Xt:
      case MISCREG_DC_CISW_Xt:
        trap_to_hyp = EL2Enabled(tc) && (el == EL1) && hcr.tsw;
        break;
      case MISCREG_ACTLR_EL1:
        trap_to_hyp = EL2Enabled (tc) && (el == EL1) && hcr.tacr;
        break;
      case MISCREG_APDAKeyHi_EL1:
      case MISCREG_APDAKeyLo_EL1:
      case MISCREG_APDBKeyHi_EL1:
      case MISCREG_APDBKeyLo_EL1:
      case MISCREG_APGAKeyHi_EL1:
      case MISCREG_APGAKeyLo_EL1:
      case MISCREG_APIAKeyHi_EL1:
      case MISCREG_APIAKeyLo_EL1:
      case MISCREG_APIBKeyHi_EL1:
      case MISCREG_APIBKeyLo_EL1:
        trap_to_hyp = EL2Enabled(tc) && el == EL1 && !hcr.apk;
        break;
      case MISCREG_ID_PFR0_EL1:
      case MISCREG_ID_PFR1_EL1:
      //case MISCREG_ID_PFR2_EL1:
      case MISCREG_ID_DFR0_EL1:
      case MISCREG_ID_AFR0_EL1:
      case MISCREG_ID_MMFR0_EL1:
      case MISCREG_ID_MMFR1_EL1:
      case MISCREG_ID_MMFR2_EL1:
      case MISCREG_ID_MMFR3_EL1:
      //case MISCREG_ID_MMFR4_EL1:
      case MISCREG_ID_ISAR0_EL1:
      case MISCREG_ID_ISAR1_EL1:
      case MISCREG_ID_ISAR2_EL1:
      case MISCREG_ID_ISAR3_EL1:
      case MISCREG_ID_ISAR4_EL1:
      case MISCREG_ID_ISAR5_EL1:
      case MISCREG_MVFR0_EL1:
      case MISCREG_MVFR1_EL1:
      case MISCREG_MVFR2_EL1:
      case MISCREG_ID_AA64PFR0_EL1:
      case MISCREG_ID_AA64PFR1_EL1:
      case MISCREG_ID_AA64DFR0_EL1:
      case MISCREG_ID_AA64DFR1_EL1:
      case MISCREG_ID_AA64ISAR0_EL1:
      case MISCREG_ID_AA64ISAR1_EL1:
      case MISCREG_ID_AA64MMFR0_EL1:
      case MISCREG_ID_AA64MMFR1_EL1:
      case MISCREG_ID_AA64MMFR2_EL1:
      case MISCREG_ID_AA64AFR0_EL1:
      case MISCREG_ID_AA64AFR1_EL1:
        trap_to_hyp =  EL2Enabled(tc) && el == EL1 && hcr.tid3;
        break;
      case MISCREG_CTR_EL0:
        {
            const bool el2_en = EL2Enabled(tc);
            if (el == EL0 && el2_en) {
                const bool in_host = hcr.e2h && hcr.tge;
                const bool general_trap = el2_en && !in_host && hcr.tge &&
                                          !sctlr.uct;
                const bool tid_trap = el2_en && !in_host && hcr.tid2;
                const bool host_trap = el2_en && in_host && !sctlr2.uct;
                trap_to_hyp = general_trap || tid_trap || host_trap;
            } else if (el == EL1 && el2_en) {
                trap_to_hyp = hcr.tid2;
            }
        }
        break;
      case MISCREG_CCSIDR_EL1:
//      case MISCREG_CCSIDR2_EL1:
      case MISCREG_CLIDR_EL1:
      case MISCREG_CSSELR_EL1:
            trap_to_hyp =  EL2Enabled(tc) && (el == EL1) && hcr.tid2;
        break;
      case MISCREG_AIDR_EL1:
      case MISCREG_REVIDR_EL1:
        trap_to_hyp =  EL2Enabled(tc) && (el == EL1) && hcr.tid1;
        break;
       // Generic Timer
      case MISCREG_CNTFRQ_EL0 ... MISCREG_CNTVOFF_EL2:
        trap_to_hyp = el <= EL1 &&
                      isGenericTimerSystemAccessTrapEL2(misc_reg, tc);
        break;
      case MISCREG_DAIF:
        trap_to_hyp = EL2Enabled(tc) && el == EL0 &&
                     (hcr.tge && (hcr.e2h || !sctlr.uma));
        break;
      case MISCREG_SPSR_EL1:
      case MISCREG_ELR_EL1:
      case MISCREG_VBAR_EL1:
        trap_to_hyp = EL2Enabled(tc) && (el == EL1) && hcr.nv1 && !hcr.nv2;
        break;
      case MISCREG_HCR_EL2:
      case MISCREG_HSTR_EL2:
      case MISCREG_SP_EL1:
      case MISCREG_TPIDR_EL2:
      case MISCREG_VTCR_EL2:
      case MISCREG_VTTBR_EL2:
        trap_to_hyp = EL2Enabled(tc) && (el == EL1) && hcr.nv && !hcr.nv2;
        break;
//      case MISCREG_AT_S1E1WP_Xt:
//      case MISCREG_AT_S1E1RP_Xt:
      case MISCREG_AT_S1E1R_Xt:
      case MISCREG_AT_S1E1W_Xt:
      case MISCREG_AT_S1E0W_Xt:
      case MISCREG_AT_S1E0R_Xt:
        trap_to_hyp = EL2Enabled(tc) && (el == EL1) && hcr.at;
        break;
      case MISCREG_ACTLR_EL2:
      case MISCREG_AFSR0_EL2:
      case MISCREG_AFSR1_EL2:
      case MISCREG_AMAIR_EL2:
      case MISCREG_CONTEXTIDR_EL2:
      case MISCREG_CPTR_EL2:
      case MISCREG_DACR32_EL2:
      case MISCREG_ESR_EL2:
      case MISCREG_FAR_EL2:
      case MISCREG_HACR_EL2:
      case MISCREG_HPFAR_EL2:
      case MISCREG_MAIR_EL2:
//      case MISCREG_RMR_EL2:
      case MISCREG_SCTLR_EL2:
      case MISCREG_TCR_EL2:
      case MISCREG_TTBR0_EL2:
      case MISCREG_TTBR1_EL2:
      case MISCREG_VBAR_EL2:
      case MISCREG_VMPIDR_EL2:
      case MISCREG_VPIDR_EL2:
      case MISCREG_TLBI_ALLE1:
      case MISCREG_TLBI_ALLE1IS:
//      case MISCREG_TLBI_ALLE1OS:
      case MISCREG_TLBI_ALLE2:
      case MISCREG_TLBI_ALLE2IS:
//      case MISCREG_TLBI_ALLE2OS:
      case MISCREG_TLBI_IPAS2E1_Xt:
      case MISCREG_TLBI_IPAS2E1IS_Xt:
//      case MISCREG_TLBI_IPAS2E1OS:
      case MISCREG_TLBI_IPAS2LE1_Xt:
      case MISCREG_TLBI_IPAS2LE1IS_Xt:
//      case MISCREG_TLBI_IPAS2LE1OS:
//      case MISCREG_TLBI_RIPAS2E1:
//      case MISCREG_TLBI_RIPAS2E1IS:
//      case MISCREG_TLBI_RIPAS2E1OS:
//      case MISCREG_TLBI_RIPAS2LE1:
//      case MISCREG_TLBI_RIPAS2LE1IS:
//      case MISCREG_TLBI_RIPAS2LE1OS:
//      case MISCREG_TLBI_RVAE2:
//      case MISCREG_TLBI_RVAE2IS:
//      case MISCREG_TLBI_RVAE2OS:
//      case MISCREG_TLBI_RVALE2:
//      case MISCREG_TLBI_RVALE2IS:
//      case MISCREG_TLBI_RVALE2OS:
      case MISCREG_TLBI_VAE2_Xt:
      case MISCREG_TLBI_VAE2IS_Xt:
//      case MISCREG_TLBI_VAE2OS:
      case MISCREG_TLBI_VALE2_Xt:
      case MISCREG_TLBI_VALE2IS_Xt:
//      case MISCREG_TLBI_VALE2OS:
      case MISCREG_TLBI_VMALLS12E1:
      case MISCREG_TLBI_VMALLS12E1IS:
//      case MISCREG_TLBI_VMALLS12E1OS:
      case MISCREG_AT_S1E2W_Xt:
      case MISCREG_AT_S1E2R_Xt:
      case MISCREG_AT_S12E1R_Xt:
      case MISCREG_AT_S12E1W_Xt:
      case MISCREG_AT_S12E0W_Xt:
      case MISCREG_AT_S12E0R_Xt:
      case MISCREG_SPSR_UND:
      case MISCREG_SPSR_IRQ:
      case MISCREG_SPSR_FIQ:
      case MISCREG_SPSR_ABT:
      case MISCREG_SPSR_EL2:
      case MISCREG_ELR_EL2:
      case MISCREG_IFSR32_EL2:
      case MISCREG_DBGVCR32_EL2:
      case MISCREG_MDCR_EL2:
        trap_to_hyp = EL2Enabled(tc) && (el == EL1) && hcr.nv;
        break;
//      case MISCREG_VSTTBR_EL2:
//      case MISCREG_VSTCR_EL2:
//        trap_to_hyp = (el == EL1) && !scr.ns && scr.eel2 && ELIs64(tc,EL2)
//              && !hcr.nv2 && hcr.nv && (!hcr.e2h|| (hcr.e2h && !hcr.tge));
//        break;

      //case MISCREG_LORC_EL1:
      //case MISCREG_LOREA_EL1:
      //case MISCREG_LORID_EL1:
      //case MISCREG_LORN_EL1:
      //case MISCREG_LORSA_EL1:
      //  trap_to_hyp = (el == EL1) && (scr.ns || scr.eel2) && ELIs64(tc,EL2)
      //      && hcr.tlor && (!hcr.e2h || (hcr.e2h && !hcr.tge));
      //  break;

      case MISCREG_DC_ZVA_Xt:
        {
            const bool el2_en = EL2Enabled(tc);
            if (el == EL0 && el2_en) {
                const bool in_host = hcr.e2h && hcr.tge;
                const bool general_trap = el2_en && !in_host && hcr.tge &&
                                          !sctlr.dze;
                const bool tdz_trap = el2_en && !in_host && hcr.tdz;
                const bool host_trap = el2_en && in_host && !sctlr2.dze;
                trap_to_hyp = general_trap || tdz_trap || host_trap;
            } else if (el == EL1 && el2_en) {
                trap_to_hyp = hcr.tdz;
            }
        }
        break;
      case MISCREG_DBGBVR0_EL1:
      case MISCREG_DBGBVR1_EL1:
      case MISCREG_DBGBVR2_EL1:
      case MISCREG_DBGBVR3_EL1:
      case MISCREG_DBGBVR4_EL1:
      case MISCREG_DBGBVR5_EL1:
      case MISCREG_DBGBVR6_EL1:
      case MISCREG_DBGBVR7_EL1:
      case MISCREG_DBGBVR8_EL1:
      case MISCREG_DBGBVR9_EL1:
      case MISCREG_DBGBVR10_EL1:
      case MISCREG_DBGBVR11_EL1:
      case MISCREG_DBGBVR12_EL1:
      case MISCREG_DBGBVR13_EL1:
      case MISCREG_DBGBVR14_EL1:
      case MISCREG_DBGBVR15_EL1:
      case MISCREG_DBGBCR0_EL1:
      case MISCREG_DBGBCR1_EL1:
      case MISCREG_DBGBCR2_EL1:
      case MISCREG_DBGBCR3_EL1:
      case MISCREG_DBGBCR4_EL1:
      case MISCREG_DBGBCR5_EL1:
      case MISCREG_DBGBCR6_EL1:
      case MISCREG_DBGBCR7_EL1:
      case MISCREG_DBGBCR8_EL1:
      case MISCREG_DBGBCR9_EL1:
      case MISCREG_DBGBCR10_EL1:
      case MISCREG_DBGBCR11_EL1:
      case MISCREG_DBGBCR12_EL1:
      case MISCREG_DBGBCR13_EL1:
      case MISCREG_DBGBCR14_EL1:
      case MISCREG_DBGBCR15_EL1:
      case MISCREG_DBGWVR0_EL1:
      case MISCREG_DBGWVR1_EL1:
      case MISCREG_DBGWVR2_EL1:
      case MISCREG_DBGWVR3_EL1:
      case MISCREG_DBGWVR4_EL1:
      case MISCREG_DBGWVR5_EL1:
      case MISCREG_DBGWVR6_EL1:
      case MISCREG_DBGWVR7_EL1:
      case MISCREG_DBGWVR8_EL1:
      case MISCREG_DBGWVR9_EL1:
      case MISCREG_DBGWVR10_EL1:
      case MISCREG_DBGWVR11_EL1:
      case MISCREG_DBGWVR12_EL1:
      case MISCREG_DBGWVR13_EL1:
      case MISCREG_DBGWVR14_EL1:
      case MISCREG_DBGWVR15_EL1:
      case MISCREG_DBGWCR0_EL1:
      case MISCREG_DBGWCR1_EL1:
      case MISCREG_DBGWCR2_EL1:
      case MISCREG_DBGWCR3_EL1:
      case MISCREG_DBGWCR4_EL1:
      case MISCREG_DBGWCR5_EL1:
      case MISCREG_DBGWCR6_EL1:
      case MISCREG_DBGWCR7_EL1:
      case MISCREG_DBGWCR8_EL1:
      case MISCREG_DBGWCR9_EL1:
      case MISCREG_DBGWCR10_EL1:
      case MISCREG_DBGWCR11_EL1:
      case MISCREG_DBGWCR12_EL1:
      case MISCREG_DBGWCR13_EL1:
      case MISCREG_DBGWCR14_EL1:
      case MISCREG_DBGWCR15_EL1:
      case MISCREG_MDCCINT_EL1:
        trap_to_hyp =  EL2Enabled(tc) && (el == EL1) && mdcr.tda;
        break;
      case MISCREG_ZCR_EL1:
        {
            bool from_el1 = (el == EL1) && EL2Enabled(tc) &&
                ELIs64(tc, EL2) && ((!hcr.e2h && cptr.tz) ||
                        (hcr.e2h && ((cptr.zen & 0x1) == 0x0)));
            bool from_el2 = (el == EL2) && ((!hcr.e2h && cptr.tz) ||
                    (hcr.e2h && ((cptr.zen & 0x1) == 0x0)));
            trap_to_hyp = from_el1 || from_el2;
        }
        ec = EC_TRAPPED_SVE;
        immediate = 0;
        break;
      case MISCREG_ZCR_EL2:
        {
            bool from_el1 = (el == EL1) && EL2Enabled(tc) && hcr.nv;
            bool from_el2 = (el == EL2) && ((!hcr.e2h && cptr.tz) ||
                    (hcr.e2h && ((cptr.zen & 0x1) == 0x0)));
            trap_to_hyp = from_el1 || from_el2;
            ec = from_el1 ? EC_TRAPPED_MSR_MRS_64: EC_TRAPPED_SVE;
        }
        immediate = 0;
        break;
      default:
        break;
    }
    return trap_to_hyp;
}

bool
MiscRegOp64::checkEL3Trap(ThreadContext *tc, const MiscRegIndex misc_reg,
                          ExceptionLevel el, ExceptionClass &ec,
                          uint32_t &immediate) const
{
    const CPTR cptr = tc->readMiscReg(MISCREG_CPTR_EL3);
    const SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
    const HDCR mdcr = tc->readMiscReg(MISCREG_MDCR_EL3);
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    bool trap_to_mon = false;

    switch (misc_reg) {
      // FP/SIMD regs
      case MISCREG_FPCR:
      case MISCREG_FPSR:
      case MISCREG_FPEXC32_EL2:
        trap_to_mon = cptr.tfp && ELIs64(tc, EL3);
        ec = EC_TRAPPED_SIMD_FP;
        immediate = 0x1E00000;
        break;
      // CPACR, CPTR
      case MISCREG_CPACR_EL12:
        trap_to_mon = ((el == EL2 && cptr.tcpac && ELIs64(tc, EL3)) ||
            (el == EL1 && cptr.tcpac && ELIs64(tc, EL3) &&
             (!hcr.nv2 || hcr.nv1 || !hcr.nv))) ;
        break;
      case MISCREG_CPACR_EL1:
        trap_to_mon = el <= EL2 && cptr.tcpac && ELIs64(tc, EL3);
        break;
      case MISCREG_CPTR_EL2:
        if (el == EL2) {
            trap_to_mon = cptr.tcpac;
        }
        break;
//      case MISCREG_LORC_EL1:
//      case MISCREG_LOREA_EL1:
//      case MISCREG_LORID_EL1:
//      case MISCREG_LORN_EL1:
//      case MISCREG_LORSA_EL1:
//        trap_to_mon = (el <= EL2) && scr.ns && ELIs64(tc,EL3)
//            && hcr.tlor && (!hcr.e2h || (hcr.e2h && !hcr.tge));
//        break;
       case MISCREG_MDCCSR_EL0:
         trap_to_mon = (el <= EL2) && ELIs64(tc, EL3) && mdcr.tda == 0x1;
         break;
      case MISCREG_APDAKeyHi_EL1:
      case MISCREG_APDAKeyLo_EL1:
      case MISCREG_APDBKeyHi_EL1:
      case MISCREG_APDBKeyLo_EL1:
      case MISCREG_APGAKeyHi_EL1:
      case MISCREG_APGAKeyLo_EL1:
      case MISCREG_APIAKeyHi_EL1:
      case MISCREG_APIAKeyLo_EL1:
      case MISCREG_APIBKeyHi_EL1:
      case MISCREG_APIBKeyLo_EL1:
        trap_to_mon = (el == EL1 || el == EL2) && scr.apk == 0 &&
                      ELIs64(tc, EL3);
        break;
      // Generic Timer
      case MISCREG_CNTFRQ_EL0 ... MISCREG_CNTVOFF_EL2:
        trap_to_mon = el == EL1 &&
                      isGenericTimerSystemAccessTrapEL3(misc_reg, tc);
        break;
      case MISCREG_DBGBVR0_EL1:
      case MISCREG_DBGBVR1_EL1:
      case MISCREG_DBGBVR2_EL1:
      case MISCREG_DBGBVR3_EL1:
      case MISCREG_DBGBVR4_EL1:
      case MISCREG_DBGBVR5_EL1:
      case MISCREG_DBGBVR6_EL1:
      case MISCREG_DBGBVR7_EL1:
      case MISCREG_DBGBVR8_EL1:
      case MISCREG_DBGBVR9_EL1:
      case MISCREG_DBGBVR10_EL1:
      case MISCREG_DBGBVR11_EL1:
      case MISCREG_DBGBVR12_EL1:
      case MISCREG_DBGBVR13_EL1:
      case MISCREG_DBGBVR14_EL1:
      case MISCREG_DBGBVR15_EL1:
      case MISCREG_DBGBCR0_EL1:
      case MISCREG_DBGBCR1_EL1:
      case MISCREG_DBGBCR2_EL1:
      case MISCREG_DBGBCR3_EL1:
      case MISCREG_DBGBCR4_EL1:
      case MISCREG_DBGBCR5_EL1:
      case MISCREG_DBGBCR6_EL1:
      case MISCREG_DBGBCR7_EL1:
      case MISCREG_DBGBCR8_EL1:
      case MISCREG_DBGBCR9_EL1:
      case MISCREG_DBGBCR10_EL1:
      case MISCREG_DBGBCR11_EL1:
      case MISCREG_DBGBCR12_EL1:
      case MISCREG_DBGBCR13_EL1:
      case MISCREG_DBGBCR14_EL1:
      case MISCREG_DBGBCR15_EL1:
      case MISCREG_DBGVCR32_EL2:
      case MISCREG_DBGWVR0_EL1:
      case MISCREG_DBGWVR1_EL1:
      case MISCREG_DBGWVR2_EL1:
      case MISCREG_DBGWVR3_EL1:
      case MISCREG_DBGWVR4_EL1:
      case MISCREG_DBGWVR5_EL1:
      case MISCREG_DBGWVR6_EL1:
      case MISCREG_DBGWVR7_EL1:
      case MISCREG_DBGWVR8_EL1:
      case MISCREG_DBGWVR9_EL1:
      case MISCREG_DBGWVR10_EL1:
      case MISCREG_DBGWVR11_EL1:
      case MISCREG_DBGWVR12_EL1:
      case MISCREG_DBGWVR13_EL1:
      case MISCREG_DBGWVR14_EL1:
      case MISCREG_DBGWVR15_EL1:
      case MISCREG_DBGWCR0_EL1:
      case MISCREG_DBGWCR1_EL1:
      case MISCREG_DBGWCR2_EL1:
      case MISCREG_DBGWCR3_EL1:
      case MISCREG_DBGWCR4_EL1:
      case MISCREG_DBGWCR5_EL1:
      case MISCREG_DBGWCR6_EL1:
      case MISCREG_DBGWCR7_EL1:
      case MISCREG_DBGWCR8_EL1:
      case MISCREG_DBGWCR9_EL1:
      case MISCREG_DBGWCR10_EL1:
      case MISCREG_DBGWCR11_EL1:
      case MISCREG_DBGWCR12_EL1:
      case MISCREG_DBGWCR13_EL1:
      case MISCREG_DBGWCR14_EL1:
      case MISCREG_DBGWCR15_EL1:
      case MISCREG_MDCCINT_EL1:
      case MISCREG_MDCR_EL2:
        trap_to_mon = ELIs64(tc, EL3) && mdcr.tda && (el == EL2);
        break;
      case MISCREG_ZCR_EL1:
        trap_to_mon = !cptr.ez && ((el == EL3) ||
              ((el <= EL2) && ArmSystem::haveEL(tc,EL3) && ELIs64(tc, EL3)));
        ec = EC_TRAPPED_SVE;
        immediate = 0;
        break;
      case MISCREG_ZCR_EL2:
        trap_to_mon = !cptr.ez && ((el == EL3) ||
              ((el == EL2) && ArmSystem::haveEL(tc,EL3) && ELIs64(tc, EL3)));
        ec = EC_TRAPPED_SVE;
        immediate = 0;
        break;
      case MISCREG_ZCR_EL3:
        trap_to_mon = !cptr.ez && (el == EL3);
        ec = EC_TRAPPED_SVE;
        immediate = 0;
        break;
      default:
        break;
    }
    return trap_to_mon;
}

RegVal
MiscRegImmOp64::miscRegImm() const
{
    if (dest == MISCREG_SPSEL) {
        return imm & 0x1;
    } else if (dest == MISCREG_PAN) {
        return (imm & 0x1) << 22;
    } else {
        panic("Not a valid PSTATE field register\n");
    }
}

std::string
MiscRegImmOp64::generateDisassembly(
        Addr pc, const Loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printMiscReg(ss, dest);
    ss << ", ";
    ccprintf(ss, "#0x%x", imm);
    return ss.str();
}

std::string
MiscRegRegImmOp64::generateDisassembly(
    Addr pc, const Loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printMiscReg(ss, dest);
    ss << ", ";
    printIntReg(ss, op1);
    return ss.str();
}

std::string
RegMiscRegImmOp64::generateDisassembly(
    Addr pc, const Loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printIntReg(ss, dest);
    ss << ", ";
    printMiscReg(ss, op1);
    return ss.str();
}

Fault
MiscRegImplDefined64::execute(ExecContext *xc,
                              Trace::InstRecord *traceData) const
{
    auto tc = xc->tcBase();
    const CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);
    const ExceptionLevel el = (ExceptionLevel) (uint8_t) cpsr.el;

    Fault fault = trap(tc, miscReg, el, imm);

    if (fault != NoFault) {
        return fault;

    } else if (warning) {
        warn_once("\tinstruction '%s' unimplemented\n", fullMnemonic.c_str());
        return NoFault;

    } else {
        return std::make_shared<UndefinedInstruction>(machInst, false,
                                                      mnemonic);
    }
}

std::string
MiscRegImplDefined64::generateDisassembly(
        Addr pc, const Loader::SymbolTable *symtab) const
{
    return csprintf("%-10s (implementation defined)", fullMnemonic.c_str());
}

std::string
RegNone::generateDisassembly(
    Addr pc, const Loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printIntReg(ss, dest);
    return ss.str();
}
