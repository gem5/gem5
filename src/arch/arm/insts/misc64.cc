/*
 * Copyright (c) 2011-2013,2017-2019 ARM Limited
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
 *
 * Authors: Gabe Black
 *          Giacomo Travaglini
 */

#include "arch/arm/insts/misc64.hh"
#include "arch/arm/isa.hh"

std::string
ImmOp64::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ccprintf(ss, "#0x%x", imm);
    return ss.str();
}

std::string
RegRegImmImmOp64::generateDisassembly(Addr pc, const SymbolTable *symtab) const
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
    Addr pc, const SymbolTable *symtab) const
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
UnknownOp64::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    return csprintf("%-10s (inst %#08x)", "unknown", encoding());
}

Fault
MiscRegOp64::trap(ThreadContext *tc, MiscRegIndex misc_reg,
                  ExceptionLevel el, uint32_t immediate) const
{
    bool is_vfp_neon = false;

    // Check for traps to supervisor (FP/SIMD regs)
    if (el <= EL1 && checkEL1Trap(tc, misc_reg, el)) {

        return std::make_shared<SupervisorTrap>(machInst, 0x1E00000,
                                                EC_TRAPPED_SIMD_FP);
    }

    // Check for traps to hypervisor
    if ((ArmSystem::haveVirtualization(tc) && el <= EL2) &&
        checkEL2Trap(tc, misc_reg, el, &is_vfp_neon)) {

        return std::make_shared<HypervisorTrap>(
            machInst, is_vfp_neon ? 0x1E00000 : immediate,
            is_vfp_neon ? EC_TRAPPED_SIMD_FP : EC_TRAPPED_MSR_MRS_64);
    }

    // Check for traps to secure monitor
    if ((ArmSystem::haveSecurity(tc) && el <= EL3) &&
        checkEL3Trap(tc, misc_reg, el, &is_vfp_neon)) {

        return std::make_shared<SecureMonitorTrap>(
            machInst,
            is_vfp_neon ? 0x1E00000 : immediate,
            is_vfp_neon ? EC_TRAPPED_SIMD_FP : EC_TRAPPED_MSR_MRS_64);
    }

    return NoFault;
}

bool
MiscRegOp64::checkEL1Trap(ThreadContext *tc, const MiscRegIndex misc_reg,
                          ExceptionLevel el) const
{
    const CPACR cpacr = tc->readMiscReg(MISCREG_CPACR_EL1);

    bool trap_to_sup = false;
    switch (misc_reg) {
      case MISCREG_FPCR:
      case MISCREG_FPSR:
      case MISCREG_FPEXC32_EL2:
        if ((el == EL0 && cpacr.fpen != 0x3) ||
            (el == EL1 && !(cpacr.fpen & 0x1)))
            trap_to_sup = true;
        break;
      default:
        break;
    }
    return trap_to_sup;
}

bool
MiscRegOp64::checkEL2Trap(ThreadContext *tc, const MiscRegIndex misc_reg,
                          ExceptionLevel el, bool * is_vfp_neon) const
{
    const CPTR cptr = tc->readMiscReg(MISCREG_CPTR_EL2);
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    const SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
    const CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);

    bool trap_to_hyp = false;
    *is_vfp_neon = false;

    if (!inSecureState(scr, cpsr) && (el != EL2)) {
        switch (misc_reg) {
          // FP/SIMD regs
          case MISCREG_FPCR:
          case MISCREG_FPSR:
          case MISCREG_FPEXC32_EL2:
            trap_to_hyp = cptr.tfp;
            *is_vfp_neon = true;
            break;
          // CPACR
          case MISCREG_CPACR_EL1:
            trap_to_hyp = cptr.tcpac && el == EL1;
            break;
          // Virtual memory control regs
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
            trap_to_hyp =
                ((hcr.trvm && miscRead) || (hcr.tvm && !miscRead)) &&
                el == EL1;
            break;
          // TLB maintenance instructions
          case MISCREG_TLBI_VMALLE1:
          case MISCREG_TLBI_VAE1_Xt:
          case MISCREG_TLBI_ASIDE1_Xt:
          case MISCREG_TLBI_VAAE1_Xt:
          case MISCREG_TLBI_VALE1_Xt:
          case MISCREG_TLBI_VAALE1_Xt:
          case MISCREG_TLBI_VMALLE1IS:
          case MISCREG_TLBI_VAE1IS_Xt:
          case MISCREG_TLBI_ASIDE1IS_Xt:
          case MISCREG_TLBI_VAAE1IS_Xt:
          case MISCREG_TLBI_VALE1IS_Xt:
          case MISCREG_TLBI_VAALE1IS_Xt:
            trap_to_hyp = hcr.ttlb && el == EL1;
            break;
          // Cache maintenance instructions to the point of unification
          case MISCREG_IC_IVAU_Xt:
          case MISCREG_ICIALLU:
          case MISCREG_ICIALLUIS:
          case MISCREG_DC_CVAU_Xt:
            trap_to_hyp = hcr.tpu && el <= EL1;
            break;
          // Data/Unified cache maintenance instructions to the
          // point of coherency
          case MISCREG_DC_IVAC_Xt:
          case MISCREG_DC_CIVAC_Xt:
          case MISCREG_DC_CVAC_Xt:
            trap_to_hyp = hcr.tpc && el <= EL1;
            break;
          // Data/Unified cache maintenance instructions by set/way
          case MISCREG_DC_ISW_Xt:
          case MISCREG_DC_CSW_Xt:
          case MISCREG_DC_CISW_Xt:
            trap_to_hyp = hcr.tsw && el == EL1;
            break;
          // ACTLR
          case MISCREG_ACTLR_EL1:
            trap_to_hyp = hcr.tacr && el == EL1;
            break;

          // @todo: Trap implementation-dependent functionality based on
          // hcr.tidcp

          // ID regs, group 3
          case MISCREG_ID_PFR0_EL1:
          case MISCREG_ID_PFR1_EL1:
          case MISCREG_ID_DFR0_EL1:
          case MISCREG_ID_AFR0_EL1:
          case MISCREG_ID_MMFR0_EL1:
          case MISCREG_ID_MMFR1_EL1:
          case MISCREG_ID_MMFR2_EL1:
          case MISCREG_ID_MMFR3_EL1:
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
            assert(miscRead);
            trap_to_hyp = hcr.tid3 && el == EL1;
            break;
          // ID regs, group 2
          case MISCREG_CTR_EL0:
          case MISCREG_CCSIDR_EL1:
          case MISCREG_CLIDR_EL1:
          case MISCREG_CSSELR_EL1:
            trap_to_hyp = hcr.tid2 && el <= EL1;
            break;
          // ID regs, group 1
          case MISCREG_AIDR_EL1:
          case MISCREG_REVIDR_EL1:
            assert(miscRead);
            trap_to_hyp = hcr.tid1 && el == EL1;
            break;
          case MISCREG_IMPDEF_UNIMPL:
            trap_to_hyp = hcr.tidcp && el == EL1;
            break;
          // GICv3 regs
          case MISCREG_ICC_SGI0R_EL1:
            if (tc->getIsaPtr()->haveGICv3CpuIfc())
                trap_to_hyp = hcr.fmo && el == EL1;
            break;
          case MISCREG_ICC_SGI1R_EL1:
          case MISCREG_ICC_ASGI1R_EL1:
            if (tc->getIsaPtr()->haveGICv3CpuIfc())
                trap_to_hyp = hcr.imo && el == EL1;
            break;
          default:
            break;
        }
    }
    return trap_to_hyp;
}

bool
MiscRegOp64::checkEL3Trap(ThreadContext *tc, const MiscRegIndex misc_reg,
                          ExceptionLevel el, bool * is_vfp_neon) const
{
    const CPTR cptr = tc->readMiscReg(MISCREG_CPTR_EL3);

    bool trap_to_mon = false;
    *is_vfp_neon = false;

    switch (misc_reg) {
      // FP/SIMD regs
      case MISCREG_FPCR:
      case MISCREG_FPSR:
      case MISCREG_FPEXC32_EL2:
        trap_to_mon = cptr.tfp;
        *is_vfp_neon = true;
        break;
      // CPACR, CPTR
      case MISCREG_CPACR_EL1:
        if (el == EL1 || el == EL2) {
           trap_to_mon = cptr.tcpac;
        }
        break;
      case MISCREG_CPTR_EL2:
        if (el == EL2) {
            trap_to_mon = cptr.tcpac;
        }
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
MiscRegImmOp64::generateDisassembly(Addr pc, const SymbolTable *symtab) const
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
    Addr pc, const SymbolTable *symtab) const
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
    Addr pc, const SymbolTable *symtab) const
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
MiscRegImplDefined64::generateDisassembly(Addr pc,
                                          const SymbolTable *symtab) const
{
    return csprintf("%-10s (implementation defined)", fullMnemonic.c_str());
}
