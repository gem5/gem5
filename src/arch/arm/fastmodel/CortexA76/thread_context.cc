/*
 * Copyright 2019 Google, Inc.
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

#include "arch/arm/fastmodel/CortexA76/thread_context.hh"

#include "arch/arm/fastmodel/iris/memory_spaces.hh"
#include "arch/arm/utility.hh"
#include "iris/detail/IrisCppAdapter.h"
#include "iris/detail/IrisObjects.h"

namespace gem5
{

namespace fastmodel
{

CortexA76TC::CortexA76TC(gem5::BaseCPU *cpu, int id, System *system,
                         gem5::BaseMMU *mmu, gem5::BaseISA *isa,
                         iris::IrisConnectionInterface *iris_if,
                         const std::string &iris_path)
    : ThreadContext(cpu, id, system, mmu, isa, iris_if, iris_path)
{}

bool
CortexA76TC::translateAddress(Addr &paddr, Addr vaddr)
{
    // Determine what memory spaces are currently active.
    Iris::CanonicalMsn in_msn;
    switch (ArmISA::currEL(this)) {
    case ArmISA::EL3:
        in_msn = Iris::SecureMonitorMsn;
        break;
    case ArmISA::EL2:
        in_msn = Iris::NsHypMsn;
        break;
    default:
        in_msn = Iris::GuestMsn;
        break;
    }

    Iris::CanonicalMsn out_msn = ArmISA::isSecure(this) ?
                                     Iris::PhysicalMemorySecureMsn :
                                     Iris::PhysicalMemoryNonSecureMsn;

    // Figure out what memory spaces match the canonical numbers we need.
    iris::MemorySpaceId in = getMemorySpaceId(in_msn);
    iris::MemorySpaceId out = getMemorySpaceId(out_msn);

    panic_if(in == iris::IRIS_UINT64_MAX || out == iris::IRIS_UINT64_MAX,
             "Canonical IRIS memory space numbers not found.");

    return ThreadContext::translateAddress(paddr, out, vaddr, in);
}

void
CortexA76TC::initFromIrisInstance(const ResourceMap &resources)
{
    ThreadContext::initFromIrisInstance(resources);

    pcRscId = extractResourceId(resources, "PC");

    extractResourceMap(miscRegIds, resources, miscRegIdxNameMap);

    extractResourceMap(intReg32Ids, resources, intReg32IdxNameMap);
    extractResourceMap(intReg64Ids, resources, intReg64IdxNameMap);

    extractResourceMap(flattenedIntIds, resources, flattenedIntIdxNameMap);

    extractResourceMap(ccRegIds, resources, ccRegIdxNameMap);

    extractResourceMap(vecRegIds, resources, vecRegIdxNameMap);
}

RegVal
CortexA76TC::readIntRegFlat(RegIndex idx) const
{
    ArmISA::CPSR orig_cpsr;

    auto *non_const_this = const_cast<CortexA76TC *>(this);

    if (idx == ArmISA::int_reg::R13Mon || idx == ArmISA::int_reg::R14Mon) {
        orig_cpsr = readMiscRegNoEffect(ArmISA::MISCREG_CPSR);
        ArmISA::CPSR new_cpsr = orig_cpsr;
        new_cpsr.mode = ArmISA::MODE_MON;
        non_const_this->setMiscReg(ArmISA::MISCREG_CPSR, new_cpsr);
    }

    RegVal val = ThreadContext::readIntRegFlat(idx);

    if (idx == ArmISA::int_reg::R13Mon || idx == ArmISA::int_reg::R14Mon) {
        non_const_this->setMiscReg(ArmISA::MISCREG_CPSR, orig_cpsr);
    }

    return val;
}

void
CortexA76TC::setIntRegFlat(RegIndex idx, RegVal val)
{
    ArmISA::CPSR orig_cpsr;

    if (idx == ArmISA::int_reg::R13Mon || idx == ArmISA::int_reg::R14Mon) {
        orig_cpsr = readMiscRegNoEffect(ArmISA::MISCREG_CPSR);
        ArmISA::CPSR new_cpsr = orig_cpsr;
        new_cpsr.mode = ArmISA::MODE_MON;
        setMiscReg(ArmISA::MISCREG_CPSR, new_cpsr);
    }

    ThreadContext::setIntRegFlat(idx, val);

    if (idx == ArmISA::int_reg::R13Mon || idx == ArmISA::int_reg::R14Mon) {
        setMiscReg(ArmISA::MISCREG_CPSR, orig_cpsr);
    }
}

RegVal
CortexA76TC::readCCRegFlat(RegIndex idx) const
{
    RegVal result = Iris::ThreadContext::readCCRegFlat(idx);
    switch (idx) {
    case ArmISA::cc_reg::Nz:
        result = ((ArmISA::CPSR)result).nz;
        break;
    case ArmISA::cc_reg::Fp:
        result = bits(result, 31, 28);
        break;
    default:
        break;
    }
    return result;
}

void
CortexA76TC::setCCRegFlat(RegIndex idx, RegVal val)
{
    switch (idx) {
    case ArmISA::cc_reg::Nz: {
        ArmISA::CPSR cpsr = readMiscRegNoEffect(ArmISA::MISCREG_CPSR);
        cpsr.nz = val;
        val = cpsr;
    } break;
    case ArmISA::cc_reg::Fp: {
        ArmISA::FPSCR fpscr = readMiscRegNoEffect(ArmISA::MISCREG_FPSCR);
        val = insertBits(fpscr, 31, 28, val);
    } break;
    default:
        break;
    }
    Iris::ThreadContext::setCCRegFlat(idx, val);
}

const std::vector<iris::MemorySpaceId> &
CortexA76TC::getBpSpaceIds() const
{
    if (bpSpaceIds.empty()) {
        std::vector<Iris::CanonicalMsn> msns{ Iris::SecureMonitorMsn,
                                              Iris::GuestMsn, Iris::NsHypMsn,
                                              Iris::HypAppMsn };
        for (auto &msn : msns) {
            auto id = getMemorySpaceId(msn);
            if (id != iris::IRIS_UINT64_MAX)
                bpSpaceIds.push_back(id);
        }
        panic_if(bpSpaceIds.empty(),
                 "Unable to find address space(s) for breakpoints.");
    }
    return bpSpaceIds;
}

Iris::ThreadContext::IdxNameMap CortexA76TC::miscRegIdxNameMap(
    { { ArmISA::MISCREG_CPSR, "CPSR" },
      { ArmISA::MISCREG_SPSR, "SPSR" },
      { ArmISA::MISCREG_SPSR_FIQ, "SPSR_fiq" },
      { ArmISA::MISCREG_SPSR_IRQ, "SPSR_irq" },
      // ArmISA::MISCREG_SPSR_SVC?
      // ArmISA::MISCREG_SPSR_MON?
      { ArmISA::MISCREG_SPSR_ABT, "SPSR_abt" },
      // ArmISA::MISCREG_SPSR_HYP?
      { ArmISA::MISCREG_SPSR_UND, "SPSR_und" },
      // ArmISA::MISCREG_ELR_HYP?
      // ArmISA::MISCREG_FPSID?
      { ArmISA::MISCREG_FPSCR, "FPSCR" },
      { ArmISA::MISCREG_MVFR1, "MVFR1_EL1" }, // XXX verify
      { ArmISA::MISCREG_MVFR0, "MVFR1_EL1" }, // XXX verify
      // ArmISA::MISCREG_FPEXC?

      // Helper registers
      { ArmISA::MISCREG_CPSR_MODE, "CPSR.MODE" },
      { ArmISA::MISCREG_CPSR_Q, "CPSR.Q" },
      // ArmISA::MISCREG_FPSCR_EXC?
      { ArmISA::MISCREG_FPSCR_QC, "FPSR.QC" },
      // ArmISA::MISCREG_LOCKADDR?
      // ArmISA::MISCREG_LOCKFLAG?
      // ArmISA::MISCREG_PRRR_MAIR0?
      // ArmISA::MISCREG_PRRR_MAIR0_NS?
      // ArmISA::MISCREG_PRRR_MAIR0_S?
      // ArmISA::MISCREG_NMRR_MAIR1?
      // ArmISA::MISCREG_NMRR_MAIR1_NS?
      // ArmISA::MISCREG_NMRR_MAIR1_S?
      // ArmISA::MISCREG_PMXEVTYPER_PMCCFILTR?
      // ArmISA::MISCREG_SCTLR_RST?
      { ArmISA::MISCREG_SEV_MAILBOX, "SEV_STATE" },

      // AArch32 CP14 registers (debug/trace control)
      // ArmISA::MISCREG_DBGDIDR?
      // ArmISA::MISCREG_DBGDSCRint?
      // ArmISA::MISCREG_DBGDCCINT?
      // ArmISA::MISCREG_DBGDTRTXint?
      // ArmISA::MISCREG_DBGDTRRXint?
      { ArmISA::MISCREG_DBGWFAR, "DBGWFAR" },
      // ArmISA::MISCREG_DBGVCR?
      { ArmISA::MISCREG_DBGDTRRXext, "DBGDTRRXext" },
      // ArmISA::MISCREG_DBGDSCRext?
      { ArmISA::MISCREG_DBGDTRTXext, "DBGDTRTXext" },
      // ArmISA::MISCREG_DBGOSECCR?
      { ArmISA::MISCREG_DBGBVR0, "DBGBVR0" },
      { ArmISA::MISCREG_DBGBVR1, "DBGBVR1" },
      { ArmISA::MISCREG_DBGBVR2, "DBGBVR2" },
      { ArmISA::MISCREG_DBGBVR3, "DBGBVR3" },
      { ArmISA::MISCREG_DBGBVR4, "DBGBVR4" },
      { ArmISA::MISCREG_DBGBVR5, "DBGBVR5" },
      { ArmISA::MISCREG_DBGBCR0, "DBGBCR0" },
      { ArmISA::MISCREG_DBGBCR1, "DBGBCR1" },
      { ArmISA::MISCREG_DBGBCR2, "DBGBCR2" },
      { ArmISA::MISCREG_DBGBCR3, "DBGBCR3" },
      { ArmISA::MISCREG_DBGBCR4, "DBGBCR4" },
      { ArmISA::MISCREG_DBGBCR5, "DBGBCR5" },
      { ArmISA::MISCREG_DBGWVR0, "DBGWVR0" },
      { ArmISA::MISCREG_DBGWVR1, "DBGWVR1" },
      { ArmISA::MISCREG_DBGWVR2, "DBGWVR2" },
      { ArmISA::MISCREG_DBGWVR3, "DBGWVR3" },
      { ArmISA::MISCREG_DBGWCR0, "DBGWCR0" },
      { ArmISA::MISCREG_DBGWCR1, "DBGWCR1" },
      { ArmISA::MISCREG_DBGWCR2, "DBGWCR2" },
      { ArmISA::MISCREG_DBGWCR3, "DBGWCR3" },
      // ArmISA::MISCREG_DBGDRAR?
      { ArmISA::MISCREG_DBGBXVR4, "DBGBXVR4" },
      { ArmISA::MISCREG_DBGBXVR5, "DBGBXVR5" },
      { ArmISA::MISCREG_DBGOSLAR, "DBGOSLAR" },
      // ArmISA::MISCREG_DBGOSLSR?
      // ArmISA::MISCREG_DBGOSDLR?
      // ArmISA::MISCREG_DBGPRCR?
      // ArmISA::MISCREG_DBGDSAR?
      { ArmISA::MISCREG_DBGCLAIMSET, "DBGCLAIMSET" },
      { ArmISA::MISCREG_DBGCLAIMCLR, "DBGCLAIMCLR" },
      { ArmISA::MISCREG_DBGAUTHSTATUS, "DBGAUTHSTATUS" },
      // ArmISA::MISCREG_DBGDEVID2?
      // ArmISA::MISCREG_DBGDEVID1?
      // ArmISA::MISCREG_DBGDEVID0?
      // ArmISA::MISCREG_TEECR? not in ARM DDI 0487A.b+
      // ArmISA::MISCREG_JIDR?
      // ArmISA::MISCREG_TEEHBR? not in ARM DDI 0487A.b+
      // ArmISA::MISCREG_JOSCR?
      // ArmISA::MISCREG_JMCR?

      // AArch32 CP15 registers (system control)
      { ArmISA::MISCREG_MIDR, "MIDR" },
      // ArmISA::MISCREG_CTR?
      // ArmISA::MISCREG_TCMTR?
      // ArmISA::MISCREG_TLBTR?
      // ArmISA::MISCREG_MPIDR?
      // ArmISA::MISCREG_REVIDR?
      // ArmISA::MISCREG_ID_PFR0?
      // ArmISA::MISCREG_ID_PFR1?
      // ArmISA::MISCREG_ID_DFR0?
      // ArmISA::MISCREG_ID_AFR0?
      // ArmISA::MISCREG_ID_MMFR0?
      // ArmISA::MISCREG_ID_MMFR1?
      // ArmISA::MISCREG_ID_MMFR2?
      // ArmISA::MISCREG_ID_MMFR3?
      // ArmISA::MISCREG_ID_MMFR4?
      // ArmISA::MISCREG_ID_ISAR0?
      // ArmISA::MISCREG_ID_ISAR1?
      // ArmISA::MISCREG_ID_ISAR2?
      // ArmISA::MISCREG_ID_ISAR3?
      // ArmISA::MISCREG_ID_ISAR4?
      // ArmISA::MISCREG_ID_ISAR5?
      // ArmISA::MISCREG_ID_ISAR6?
      // ArmISA::MISCREG_CCSIDR?
      // ArmISA::MISCREG_CLIDR?
      // ArmISA::MISCREG_AIDR?
      // ArmISA::MISCREG_CSSELR?
      // ArmISA::MISCREG_CSSELR_NS?
      // ArmISA::MISCREG_CSSELR_S?
      // ArmISA::MISCREG_VPIDR?
      // ArmISA::MISCREG_VMPIDR?,
      // ArmISA::MISCREG_SCTLR?
      // ArmISA::MISCREG_SCTLR_NS?
      // ArmISA::MISCREG_SCTLR_S?
      // ArmISA::MISCREG_ACTLR?
      // ArmISA::MISCREG_ACTLR_NS?
      // ArmISA::MISCREG_ACTLR_S?
      // ArmISA::MISCREG_CPACR?
      { ArmISA::MISCREG_SCR, "SCR" },
      { ArmISA::MISCREG_SDER, "SDER" },
      // ArmISA::MISCREG_NSACR?
      // ArmISA::MISCREG_HSCTLR?
      // ArmISA::MISCREG_HACTLR?
      // ArmISA::MISCREG_HCR?
      // ArmISA::MISCREG_HDCR?
      // ArmISA::MISCREG_HCPTR?
      // ArmISA::MISCREG_HSTR?
      // ArmISA::MISCREG_HACR?
      // ArmISA::MISCREG_TTBR0?
      // ArmISA::MISCREG_TTBR0_NS?
      // ArmISA::MISCREG_TTBR0_S?
      // ArmISA::MISCREG_TTBR1?
      // ArmISA::MISCREG_TTBR1_NS?
      // ArmISA::MISCREG_TTBR1_S?
      // ArmISA::MISCREG_TTBCR?
      // ArmISA::MISCREG_TTBCR_NS?
      // ArmISA::MISCREG_TTBCR_S?
      // ArmISA::MISCREG_HTCR?
      // ArmISA::MISCREG_VTCR?
      // ArmISA::MISCREG_DACR?
      // ArmISA::MISCREG_DACR_NS?
      // ArmISA::MISCREG_DACR_S?
      // ArmISA::MISCREG_DFSR?
      // ArmISA::MISCREG_DFSR_NS?
      // ArmISA::MISCREG_DFSR_S?
      // ArmISA::MISCREG_IFSR?
      // ArmISA::MISCREG_IFSR_NS?
      // ArmISA::MISCREG_IFSR_S?
      // ArmISA::MISCREG_ADFSR?
      // ArmISA::MISCREG_ADFSR_NS?
      // ArmISA::MISCREG_ADFSR_S?
      // ArmISA::MISCREG_AIFSR?
      // ArmISA::MISCREG_AIFSR_NS?
      // ArmISA::MISCREG_AIFSR_S?
      // ArmISA::MISCREG_HADFSR?
      // ArmISA::MISCREG_HAIFSR?
      // ArmISA::MISCREG_HSR?
      // ArmISA::MISCREG_DFAR?
      // ArmISA::MISCREG_DFAR_NS?
      // ArmISA::MISCREG_DFAR_S?
      // ArmISA::MISCREG_IFAR?
      // ArmISA::MISCREG_IFAR_NS?
      // ArmISA::MISCREG_IFAR_S?
      // ArmISA::MISCREG_HDFAR?
      // ArmISA::MISCREG_HIFAR?
      // ArmISA::MISCREG_HPFAR?
      // ArmISA::MISCREG_ICIALLUIS?
      // ArmISA::MISCREG_BPIALLIS?
      // ArmISA::MISCREG_PAR?
      // ArmISA::MISCREG_PAR_NS?
      // ArmISA::MISCREG_PAR_S?
      // ArmISA::MISCREG_ICIALLU?
      // ArmISA::MISCREG_ICIMVAU?
      // ArmISA::MISCREG_CP15ISB?
      // ArmISA::MISCREG_BPIALL?
      // ArmISA::MISCREG_BPIMVA?
      // ArmISA::MISCREG_DCIMVAC?
      // ArmISA::MISCREG_DCISW?
      // ArmISA::MISCREG_ATS1CPR?
      // ArmISA::MISCREG_ATS1CPW?
      // ArmISA::MISCREG_ATS1CUR?
      // ArmISA::MISCREG_ATS1CUW?
      // ArmISA::MISCREG_ATS12NSOPR?
      // ArmISA::MISCREG_ATS12NSOPW?
      // ArmISA::MISCREG_ATS12NSOUR?
      // ArmISA::MISCREG_ATS12NSOUW?
      // ArmISA::MISCREG_DCCMVAC?
      // ArmISA::MISCREG_DCCSW?
      // ArmISA::MISCREG_CP15DSB?
      // ArmISA::MISCREG_CP15DMB?
      // ArmISA::MISCREG_DCCMVAU?
      // ArmISA::MISCREG_DCCIMVAC?
      // ArmISA::MISCREG_DCCISW?
      // ArmISA::MISCREG_ATS1HR?
      // ArmISA::MISCREG_ATS1HW?
      // ArmISA::MISCREG_TLBIALLIS?
      // ArmISA::MISCREG_TLBIMVAIS?
      // ArmISA::MISCREG_TLBIASIDIS?
      // ArmISA::MISCREG_TLBIMVAAIS?
      // ArmISA::MISCREG_TLBIMVALIS?
      // ArmISA::MISCREG_TLBIMVAALIS?
      // ArmISA::MISCREG_ITLBIALL?
      // ArmISA::MISCREG_ITLBIMVA?
      // ArmISA::MISCREG_ITLBIASID?
      // ArmISA::MISCREG_DTLBIALL?
      // ArmISA::MISCREG_DTLBIMVA?
      // ArmISA::MISCREG_DTLBIASID?
      // ArmISA::MISCREG_TLBIALL?
      // ArmISA::MISCREG_TLBIMVA?
      // ArmISA::MISCREG_TLBIASID?
      // ArmISA::MISCREG_TLBIMVAA?
      // ArmISA::MISCREG_TLBIMVAL?
      // ArmISA::MISCREG_TLBIMVAAL?
      // ArmISA::MISCREG_TLBIIPAS2IS?
      // ArmISA::MISCREG_TLBIIPAS2LIS?
      // ArmISA::MISCREG_TLBIALLHIS?
      // ArmISA::MISCREG_TLBIMVAHIS?
      // ArmISA::MISCREG_TLBIALLNSNHIS?
      // ArmISA::MISCREG_TLBIMVALHIS?
      // ArmISA::MISCREG_TLBIIPAS2?
      // ArmISA::MISCREG_TLBIIPAS2L?
      // ArmISA::MISCREG_TLBIALLH?
      // ArmISA::MISCREG_TLBIMVAH?
      // ArmISA::MISCREG_TLBIALLNSNH?
      // ArmISA::MISCREG_TLBIMVALH?
      { ArmISA::MISCREG_PMCR, "PMCR" },
      { ArmISA::MISCREG_PMCNTENSET, "PMCNTENSET" },
      { ArmISA::MISCREG_PMCNTENCLR, "PMCNTENCLR" },
      { ArmISA::MISCREG_PMOVSR, "PMOVSR" },
      { ArmISA::MISCREG_PMSWINC, "PMSWINC" },
      { ArmISA::MISCREG_PMSELR, "PMSELR" },
      { ArmISA::MISCREG_PMCEID0, "PMCEID0" },
      { ArmISA::MISCREG_PMCEID1, "PMCEID1" },
      { ArmISA::MISCREG_PMCCNTR, "PMCCNTR" },
      { ArmISA::MISCREG_PMXEVTYPER, "PMXEVTYPER" },
      { ArmISA::MISCREG_PMCCFILTR, "PMCCFILTR" },
      { ArmISA::MISCREG_PMXEVCNTR, "PMXEVCNTR" },
      { ArmISA::MISCREG_PMUSERENR, "PMUSERENR" },
      { ArmISA::MISCREG_PMINTENSET, "PMINTENSET" },
      { ArmISA::MISCREG_PMINTENCLR, "PMINTENCLR" },
      { ArmISA::MISCREG_PMOVSSET, "PMOVSSET" },
      // ArmISA::MISCREG_L2CTLR?
      // ArmISA::MISCREG_L2ECTLR?
      // ArmISA::MISCREG_PRRR?
      // ArmISA::MISCREG_PRRR_NS?
      // ArmISA::MISCREG_PRRR_S?
      // ArmISA::MISCREG_MAIR0?
      // ArmISA::MISCREG_MAIR0_NS?
      // ArmISA::MISCREG_MAIR0_S?
      // ArmISA::MISCREG_NMRR?
      // ArmISA::MISCREG_NMRR_NS?
      // ArmISA::MISCREG_NMRR_S?
      // ArmISA::MISCREG_MAIR1?
      // ArmISA::MISCREG_MAIR1_NS?
      // ArmISA::MISCREG_MAIR1_S?
      // ArmISA::MISCREG_AMAIR0?
      // ArmISA::MISCREG_AMAIR0_N?
      // ArmISA::MISCREG_AMAIR0_S?
      // ArmISA::MISCREG_AMAIR1?
      // ArmISA::MISCREG_AMAIR1_NS?
      // ArmISA::MISCREG_AMAIR1_S?
      // ArmISA::MISCREG_HMAIR0?
      // ArmISA::MISCREG_HMAIR1?
      // ArmISA::MISCREG_HAMAIR0?
      // ArmISA::MISCREG_HAMAIR1?
      // ArmISA::MISCREG_VBAR?
      // ArmISA::MISCREG_VBAR_NS?
      // ArmISA::MISCREG_VBAR_S?
      { ArmISA::MISCREG_MVBAR, "MVBAR" },
      // ArmISA::MISCREG_RMR?
      // ArmISA::MISCREG_ISR?
      // ArmISA::MISCREG_HVBAR?
      // ArmISA::MISCREG_FCSEIDR?
      // ArmISA::MISCREG_CONTEXTIDR?
      // ArmISA::MISCREG_CONTEXTIDR_NS?
      // ArmISA::MISCREG_CONTEXTIDR_S?
      // ArmISA::MISCREG_TPIDRURW?
      // ArmISA::MISCREG_TPIDRURW_NS?
      // ArmISA::MISCREG_TPIDRURW_S?
      // ArmISA::MISCREG_TPIDRURO?
      // ArmISA::MISCREG_TPIDRURO_NS?
      // ArmISA::MISCREG_TPIDRURO_S?
      // ArmISA::MISCREG_TPIDRPRW?
      // ArmISA::MISCREG_TPIDRPRW_NS?
      // ArmISA::MISCREG_TPIDRPRW_S?
      // ArmISA::MISCREG_HTPIDR?
      { ArmISA::MISCREG_CNTFRQ, "CNTFRQ" },
      // ArmISA::MISCREG_CNTKCTL?
      { ArmISA::MISCREG_CNTP_TVAL, "CNTP_TVAL" },
      // ArmISA::MISCREG_CNTP_TVAL_NS?
      // ArmISA::MISCREG_CNTP_TVAL_S?
      { ArmISA::MISCREG_CNTP_CTL, "CNTP_CTL" },
      // ArmISA::MISCREG_CNTP_CTL_NS?
      // ArmISA::MISCREG_CNTP_CTL_S?
      { ArmISA::MISCREG_CNTV_TVAL, "CNTV_TVAL" },
      { ArmISA::MISCREG_CNTV_CTL, "CNTV_CTL" },
      // ArmISA::MISCREG_CNTHCTL?
      // ArmISA::MISCREG_CNTHP_TVAL?
      // ArmISA::MISCREG_CNTHP_CTL?
      // ArmISA::MISCREG_IL1DATA0?
      // ArmISA::MISCREG_IL1DATA1?
      // ArmISA::MISCREG_IL1DATA2?
      // ArmISA::MISCREG_IL1DATA3?
      // ArmISA::MISCREG_DL1DATA0?
      // ArmISA::MISCREG_DL1DATA1?
      // ArmISA::MISCREG_DL1DATA2?
      // ArmISA::MISCREG_DL1DATA3?
      // ArmISA::MISCREG_DL1DATA4?
      // ArmISA::MISCREG_RAMINDEX?
      // ArmISA::MISCREG_L2ACTLR?
      // ArmISA::MISCREG_CBAR?
      // ArmISA::MISCREG_HTTBR?
      // ArmISA::MISCREG_VTTBR?
      { ArmISA::MISCREG_CNTPCT, "CNTPCT" },
      { ArmISA::MISCREG_CNTVCT, "CNTVCT" },
      { ArmISA::MISCREG_CNTP_CVAL, "CNTP_CVAL" },
      // ArmISA::MISCREG_CNTP_CVAL_NS?
      // ArmISA::MISCREG_CNTP_CVAL_S?
      { ArmISA::MISCREG_CNTV_CVAL, "CNTV_CVAL" },
      { ArmISA::MISCREG_CNTVOFF, "CNTVOFF" },
      // ArmISA::MISCREG_CNTHP_CVAL?
      // ArmISA::MISCREG_CPUMERRSR?
      // ArmISA::MISCREG_L2MERRSR?

      // AArch64 registers (Op0=2)
      { ArmISA::MISCREG_MDCCINT_EL1, "MDCCINT_EL1" },
      { ArmISA::MISCREG_OSDTRRX_EL1, "OSDTRRX_EL1" },
      { ArmISA::MISCREG_MDSCR_EL1, "MDSCR_EL1" },
      { ArmISA::MISCREG_OSDTRTX_EL1, "OSDTRTX_EL1" },
      { ArmISA::MISCREG_OSECCR_EL1, "OSECCR_EL1" },
      { ArmISA::MISCREG_DBGBVR0_EL1, "DBGBVR0_EL1" },
      { ArmISA::MISCREG_DBGBVR1_EL1, "DBGBVR1_EL1" },
      { ArmISA::MISCREG_DBGBVR2_EL1, "DBGBVR2_EL1" },
      { ArmISA::MISCREG_DBGBVR3_EL1, "DBGBVR3_EL1" },
      { ArmISA::MISCREG_DBGBVR4_EL1, "DBGBVR4_EL1" },
      { ArmISA::MISCREG_DBGBVR5_EL1, "DBGBVR5_EL1" },
      { ArmISA::MISCREG_DBGBCR0_EL1, "DBGBCR0_EL1" },
      { ArmISA::MISCREG_DBGBCR1_EL1, "DBGBCR1_EL1" },
      { ArmISA::MISCREG_DBGBCR2_EL1, "DBGBCR2_EL1" },
      { ArmISA::MISCREG_DBGBCR3_EL1, "DBGBCR3_EL1" },
      { ArmISA::MISCREG_DBGBCR4_EL1, "DBGBCR4_EL1" },
      { ArmISA::MISCREG_DBGBCR5_EL1, "DBGBCR5_EL1" },
      { ArmISA::MISCREG_DBGWVR0_EL1, "DBGWVR0_EL1" },
      { ArmISA::MISCREG_DBGWVR1_EL1, "DBGWVR1_EL1" },
      { ArmISA::MISCREG_DBGWVR2_EL1, "DBGWVR2_EL1" },
      { ArmISA::MISCREG_DBGWVR3_EL1, "DBGWVR3_EL1" },
      { ArmISA::MISCREG_DBGWCR0_EL1, "DBGWCR0_EL1" },
      { ArmISA::MISCREG_DBGWCR1_EL1, "DBGWCR1_EL1" },
      { ArmISA::MISCREG_DBGWCR2_EL1, "DBGWCR2_EL1" },
      { ArmISA::MISCREG_DBGWCR3_EL1, "DBGWCR3_EL1" },
      { ArmISA::MISCREG_MDCCSR_EL0, "MDCCSR_EL0" },
      // ArmISA::MISCREG_MDDTR_EL0?
      // ArmISA::MISCREG_MDDTRTX_EL0?
      // ArmISA::MISCREG_MDDTRRX_EL0?
      // ArmISA::MISCREG_DBGVCR32_EL2?
      { ArmISA::MISCREG_MDRAR_EL1, "MDRAR_EL1" },
      { ArmISA::MISCREG_OSLAR_EL1, "OSLAR_EL1" },
      { ArmISA::MISCREG_OSLSR_EL1, "OSLSR_EL1" },
      { ArmISA::MISCREG_OSDLR_EL1, "OSDLR_EL1" },
      { ArmISA::MISCREG_DBGPRCR_EL1, "DBGPRCR_EL1" },
      { ArmISA::MISCREG_DBGCLAIMSET_EL1, "DBGCLAIMSET_EL1" },
      { ArmISA::MISCREG_DBGCLAIMCLR_EL1, "DBGCLAIMCLR_EL1" },
      { ArmISA::MISCREG_DBGAUTHSTATUS_EL1, "DBGAUTHSTATUS_EL1" },
      // ArmISA::MISCREG_TEECR32_EL1? not in ARM DDI 0487A.b+
      // ArmISA::MISCREG_TEEHBR32_EL1? not in ARM DDI 0487A.b+

      // AArch64 registers (Op0=1)
      { ArmISA::MISCREG_MIDR_EL1, "MIDR_EL1" },
      { ArmISA::MISCREG_MPIDR_EL1, "MPIDR_EL1" },
      { ArmISA::MISCREG_REVIDR_EL1, "REVIDR_EL1" },
      { ArmISA::MISCREG_ID_PFR0_EL1, "ID_PFR0_EL1" },
      { ArmISA::MISCREG_ID_PFR1_EL1, "ID_PFR1_EL1" },
      { ArmISA::MISCREG_ID_DFR0_EL1, "ID_DFR0_EL1" },
      { ArmISA::MISCREG_ID_AFR0_EL1, "ID_AFR0_EL1" },
      { ArmISA::MISCREG_ID_MMFR0_EL1, "ID_MMFR0_EL1" },
      { ArmISA::MISCREG_ID_MMFR1_EL1, "ID_MMFR1_EL1" },
      { ArmISA::MISCREG_ID_MMFR2_EL1, "ID_MMFR2_EL1" },
      { ArmISA::MISCREG_ID_MMFR3_EL1, "ID_MMFR3_EL1" },
      { ArmISA::MISCREG_ID_MMFR4_EL1, "ID_MMFR4_EL1" },
      { ArmISA::MISCREG_ID_ISAR0_EL1, "ID_ISAR0_EL1" },
      { ArmISA::MISCREG_ID_ISAR1_EL1, "ID_ISAR1_EL1" },
      { ArmISA::MISCREG_ID_ISAR2_EL1, "ID_ISAR2_EL1" },
      { ArmISA::MISCREG_ID_ISAR3_EL1, "ID_ISAR3_EL1" },
      { ArmISA::MISCREG_ID_ISAR4_EL1, "ID_ISAR4_EL1" },
      { ArmISA::MISCREG_ID_ISAR5_EL1, "ID_ISAR5_EL1" },
      { ArmISA::MISCREG_ID_ISAR6_EL1, "ID_ISAR6_EL1" },
      { ArmISA::MISCREG_MVFR0_EL1, "MVFR0_EL1" },
      { ArmISA::MISCREG_MVFR1_EL1, "MVFR1_EL1" },
      { ArmISA::MISCREG_MVFR2_EL1, "MVFR2_EL1" },
      { ArmISA::MISCREG_ID_AA64PFR0_EL1, "ID_AA64PFR0_EL1" },
      { ArmISA::MISCREG_ID_AA64PFR1_EL1, "ID_AA64PFR1_EL1" },
      { ArmISA::MISCREG_ID_AA64DFR0_EL1, "ID_AA64DFR0_EL1" },
      { ArmISA::MISCREG_ID_AA64DFR1_EL1, "ID_AA64DFR1_EL1" },
      { ArmISA::MISCREG_ID_AA64AFR0_EL1, "ID_AA64AFR0_EL1" },
      { ArmISA::MISCREG_ID_AA64AFR1_EL1, "ID_AA64AFR1_EL1" },
      { ArmISA::MISCREG_ID_AA64ISAR0_EL1, "ID_AA64ISAR0_EL1" },
      { ArmISA::MISCREG_ID_AA64ISAR1_EL1, "ID_AA64ISAR1_EL1" },
      { ArmISA::MISCREG_ID_AA64MMFR0_EL1, "ID_AA64MMFR0_EL1" },
      { ArmISA::MISCREG_ID_AA64MMFR1_EL1, "ID_AA64MMFR1_EL1" },
      { ArmISA::MISCREG_CCSIDR_EL1, "CCSIDR_EL1" },
      { ArmISA::MISCREG_CLIDR_EL1, "CLIDR_EL1" },
      { ArmISA::MISCREG_AIDR_EL1, "AIDR_EL1" },
      { ArmISA::MISCREG_CSSELR_EL1, "CSSELR_EL1" },
      { ArmISA::MISCREG_CTR_EL0, "CTR_EL0" },
      { ArmISA::MISCREG_DCZID_EL0, "DCZID_EL0" },
      { ArmISA::MISCREG_VPIDR_EL2, "VPIDR_EL2" },
      { ArmISA::MISCREG_VMPIDR_EL2, "VMPIDR_EL2" },
      { ArmISA::MISCREG_SCTLR_EL1, "SCTLR_EL1" },
      { ArmISA::MISCREG_ACTLR_EL1, "ACTLR_EL1" },
      { ArmISA::MISCREG_CPACR_EL1, "CPACR_EL1" },
      { ArmISA::MISCREG_SCTLR_EL2, "SCTLR_EL2" },
      { ArmISA::MISCREG_ACTLR_EL2, "ACTLR_EL2" },
      { ArmISA::MISCREG_HCR_EL2, "HCR_EL2" },
      { ArmISA::MISCREG_MDCR_EL2, "MDCR_EL2" },
      { ArmISA::MISCREG_CPTR_EL2, "CPTR_EL2" },
      { ArmISA::MISCREG_HSTR_EL2, "HSTR_EL2" },
      { ArmISA::MISCREG_HACR_EL2, "HACR_EL2" },
      { ArmISA::MISCREG_SCTLR_EL3, "SCTLR_EL3" },
      { ArmISA::MISCREG_ACTLR_EL3, "ACTLR_EL3" },
      { ArmISA::MISCREG_SCR_EL3, "SCR_EL3" },
      // ArmISA::MISCREG_SDER32_EL3?
      { ArmISA::MISCREG_CPTR_EL3, "CPTR_EL3" },
      { ArmISA::MISCREG_MDCR_EL3, "MDCR_EL3" },
      { ArmISA::MISCREG_TTBR0_EL1, "TTBR0_EL1" },
      { ArmISA::MISCREG_TTBR1_EL1, "TTBR1_EL1" },
      { ArmISA::MISCREG_TCR_EL1, "TCR_EL1" },
      { ArmISA::MISCREG_TTBR0_EL2, "TTBR0_EL2" },
      { ArmISA::MISCREG_TCR_EL2, "TCR_EL2" },
      { ArmISA::MISCREG_VTTBR_EL2, "VTTBR_EL2" },
      { ArmISA::MISCREG_VTCR_EL2, "VTCR_EL2" },
      { ArmISA::MISCREG_TTBR0_EL3, "TTBR0_EL3" },
      { ArmISA::MISCREG_TCR_EL3, "TCR_EL3" },
      // ArmISA::MISCREG_DACR32_EL2?
      { ArmISA::MISCREG_SPSR_EL1, "SPSR_EL1" },
      { ArmISA::MISCREG_ELR_EL1, "ELR_EL1" },
      { ArmISA::MISCREG_SP_EL0, "SP_EL0" },
      // ArmISA::MISCREG_SPSEL?
      // ArmISA::MISCREG_CURRENTEL?
      // ArmISA::MISCREG_NZCV?
      // ArmISA::MISCREG_DAIF?
      { ArmISA::MISCREG_FPCR, "FPCR" },
      { ArmISA::MISCREG_FPSR, "FPSR" },
      { ArmISA::MISCREG_DSPSR_EL0, "DSPSR_EL0" },
      { ArmISA::MISCREG_DLR_EL0, "DLR_EL0" },
      { ArmISA::MISCREG_SPSR_EL2, "SPSR_EL2" },
      { ArmISA::MISCREG_ELR_EL2, "ELR_EL2" },
      { ArmISA::MISCREG_SP_EL1, "SP_EL1" },
      // ArmISA::MISCREG_SPSR_IRQ_AA64?
      // ArmISA::MISCREG_SPSR_ABT_AA64?
      // ArmISA::MISCREG_SPSR_UND_AA64?
      // ArmISA::MISCREG_SPSR_FIQ_AA64?
      { ArmISA::MISCREG_SPSR_EL3, "SPSR_EL3" },
      { ArmISA::MISCREG_ELR_EL3, "ELR_EL3" },
      { ArmISA::MISCREG_SP_EL2, "SP_EL2" },
      { ArmISA::MISCREG_AFSR0_EL1, "AFSR0_EL1" },
      { ArmISA::MISCREG_AFSR1_EL1, "AFSR1_EL1" },
      { ArmISA::MISCREG_ESR_EL1, "ESR_EL1" },
      // ArmISA::MISCREG_IFSR32_EL2?
      { ArmISA::MISCREG_AFSR0_EL2, "AFSR0_EL2" },
      { ArmISA::MISCREG_AFSR1_EL2, "AFSR1_EL2" },
      { ArmISA::MISCREG_ESR_EL2, "ESR_EL2" },
      // ArmISA::MISCREG_FPEXC32_EL2?
      { ArmISA::MISCREG_AFSR0_EL3, "AFSR0_EL3" },
      { ArmISA::MISCREG_AFSR1_EL3, "AFSR1_EL3" },
      { ArmISA::MISCREG_ESR_EL3, "ESR_EL3" },
      { ArmISA::MISCREG_FAR_EL1, "FAR_EL1" },
      { ArmISA::MISCREG_FAR_EL2, "FAR_EL2" },
      { ArmISA::MISCREG_HPFAR_EL2, "HPFAR_EL2" },
      { ArmISA::MISCREG_FAR_EL3, "FAR_EL3" },
      { ArmISA::MISCREG_IC_IALLUIS, "IC IALLUIS" },
      { ArmISA::MISCREG_PAR_EL1, "PAR_EL1" },
      { ArmISA::MISCREG_IC_IALLU, "IC IALLU" },
      { ArmISA::MISCREG_DC_IVAC_Xt, "DC IVAC" },     // XXX verify
      { ArmISA::MISCREG_DC_ISW_Xt, "DC ISW" },       // XXX verify
      { ArmISA::MISCREG_AT_S1E1R_Xt, "AT S1E1R" },   // XXX verify
      { ArmISA::MISCREG_AT_S1E1W_Xt, "AT S1E1W" },   // XXX verify
      { ArmISA::MISCREG_AT_S1E0R_Xt, "AT S1E0R" },   // XXX verify
      { ArmISA::MISCREG_AT_S1E0W_Xt, "AT S1E0W" },   // XXX verify
      { ArmISA::MISCREG_DC_CSW_Xt, "DC CSW" },       // XXX verify
      { ArmISA::MISCREG_DC_CISW_Xt, "DC CISW" },     // XXX verify
      { ArmISA::MISCREG_DC_ZVA_Xt, "DC ZVA" },       // XXX verify
      { ArmISA::MISCREG_IC_IVAU_Xt, "IC IVAU" },     // XXX verify
      { ArmISA::MISCREG_DC_CVAC_Xt, "DC CVAC" },     // XXX verify
      { ArmISA::MISCREG_DC_CVAU_Xt, "DC CVAU" },     // XXX verify
      { ArmISA::MISCREG_DC_CIVAC_Xt, "DC CIVAC" },   // XXX verify
      { ArmISA::MISCREG_AT_S1E2R_Xt, "AT S1E2R" },   // XXX verify
      { ArmISA::MISCREG_AT_S1E2W_Xt, "AT S1E2W" },   // XXX verify
      { ArmISA::MISCREG_AT_S12E1R_Xt, "AT S12E1R" }, // XXX verify
      { ArmISA::MISCREG_AT_S12E1W_Xt, "AT S12E1W" }, // XXX verify
      { ArmISA::MISCREG_AT_S12E0R_Xt, "AT S12E0R" }, // XXX verify
      { ArmISA::MISCREG_AT_S12E0W_Xt, "AT S12E0W" }, // XXX verify
      { ArmISA::MISCREG_AT_S1E3R_Xt, "AT S1E3R" },   // XXX verify
      { ArmISA::MISCREG_AT_S1E3W_Xt, "AT S1E3W" },   // XXX verify
      { ArmISA::MISCREG_TLBI_VMALLE1IS, "TLBI VMALLE1IS" },
      { ArmISA::MISCREG_TLBI_VAE1IS_Xt, "TLBI VAE1IS" },     // XXX verify
      { ArmISA::MISCREG_TLBI_ASIDE1IS_Xt, "TLBI ASIDE1IS" }, // XXX verify
      { ArmISA::MISCREG_TLBI_VAAE1IS_Xt, "TLBI VAAE1IS" },   // XXX verify
      { ArmISA::MISCREG_TLBI_VALE1IS_Xt, "TLBI VALE1IS" },   // XXX verify
      { ArmISA::MISCREG_TLBI_VAALE1IS_Xt, "TLBI VAALE1IS" }, // XXX verify
      { ArmISA::MISCREG_TLBI_VMALLE1, "TLBI VMALLE1" },
      { ArmISA::MISCREG_TLBI_VAE1_Xt, "TLBI VAE1" },             // XXX verify
      { ArmISA::MISCREG_TLBI_ASIDE1_Xt, "TLBI ASIDE1" },         // XXX verify
      { ArmISA::MISCREG_TLBI_VAAE1_Xt, "TLBI VAAE1" },           // XXX verify
      { ArmISA::MISCREG_TLBI_VALE1_Xt, "TLBI VALE1" },           // XXX verify
      { ArmISA::MISCREG_TLBI_VAALE1_Xt, "TLBI VAALE1" },         // XXX verify
      { ArmISA::MISCREG_TLBI_IPAS2E1IS_Xt, "TLBI IPAS2E1IS" },   // XXX verify
      { ArmISA::MISCREG_TLBI_IPAS2LE1IS_Xt, "TLBI IPAS2LE1IS" }, // XXX verify
      { ArmISA::MISCREG_TLBI_ALLE2IS, "TLBI ALLE2IS" },
      { ArmISA::MISCREG_TLBI_VAE2IS_Xt, "TLBI VAE2IS" }, // XXX verify
      { ArmISA::MISCREG_TLBI_ALLE1IS, "TLBI ALLE1IS" },
      { ArmISA::MISCREG_TLBI_VALE2IS_Xt, "TLBI VALE2IS" }, // XXX verify
      { ArmISA::MISCREG_TLBI_VMALLS12E1IS, "TLBI VMALLS12E1IS" },
      { ArmISA::MISCREG_TLBI_IPAS2E1_Xt, "TLBI IPAS2E1" },   // XXX verify
      { ArmISA::MISCREG_TLBI_IPAS2LE1_Xt, "TLBI IPAS2LE1" }, // XXX verify
      { ArmISA::MISCREG_TLBI_ALLE2, "TLBI ALLE2" },
      { ArmISA::MISCREG_TLBI_VAE2_Xt, "TLBI VAE2" }, // XXX verify
      { ArmISA::MISCREG_TLBI_ALLE1, "TLBI ALLE1" },
      { ArmISA::MISCREG_TLBI_VALE2_Xt, "TLBI VALE2" }, // XXX verify
      { ArmISA::MISCREG_TLBI_VMALLS12E1, "TLBI VMALLS12E1" },
      { ArmISA::MISCREG_TLBI_ALLE3IS, "TLBI ALLE3IS" },
      { ArmISA::MISCREG_TLBI_VAE3IS_Xt, "TLBI VAE3IS" },   // XXX verify
      { ArmISA::MISCREG_TLBI_VALE3IS_Xt, "TLBI VALE3IS" }, // XXX verify
      { ArmISA::MISCREG_TLBI_ALLE3, "TLBI ALLE3" },
      { ArmISA::MISCREG_TLBI_VAE3_Xt, "TLBI VAE3" },   // XXX verify
      { ArmISA::MISCREG_TLBI_VALE3_Xt, "TLBI VALE3" }, // XXX verify
      { ArmISA::MISCREG_PMINTENSET_EL1, "PMINTENSET_EL1" },
      { ArmISA::MISCREG_PMINTENCLR_EL1, "PMINTENCLR_EL1" },
      { ArmISA::MISCREG_PMCR_EL0, "PMCR_EL0" },
      { ArmISA::MISCREG_PMCNTENSET_EL0, "PMCNTENSET_EL0" },
      { ArmISA::MISCREG_PMCNTENCLR_EL0, "PMCNTENCLR_EL0" },
      { ArmISA::MISCREG_PMOVSCLR_EL0, "PMOVSCLR_EL0" },
      { ArmISA::MISCREG_PMSWINC_EL0, "PMSWINC_EL0" },
      { ArmISA::MISCREG_PMSELR_EL0, "PMSELR_EL0" },
      { ArmISA::MISCREG_PMCEID0_EL0, "PMCEID0_EL0" },
      { ArmISA::MISCREG_PMCEID1_EL0, "PMCEID1_EL0" },
      { ArmISA::MISCREG_PMCCNTR_EL0, "PMCCNTR_EL0" },
      { ArmISA::MISCREG_PMXEVTYPER_EL0, "PMXEVTYPER_EL0" },
      { ArmISA::MISCREG_PMCCFILTR_EL0, "PMCCFILTR_EL0" },
      { ArmISA::MISCREG_PMXEVCNTR_EL0, "PMXEVCNTR_EL0" },
      { ArmISA::MISCREG_PMUSERENR_EL0, "PMUSERENR_EL0" },
      { ArmISA::MISCREG_PMOVSSET_EL0, "PMOVSSET_EL0" },
      { ArmISA::MISCREG_MAIR_EL1, "MAIR_EL1" },
      { ArmISA::MISCREG_AMAIR_EL1, "AMAIR_EL1" },
      { ArmISA::MISCREG_MAIR_EL2, "MAIR_EL2" },
      { ArmISA::MISCREG_AMAIR_EL2, "AMAIR_EL2" },
      { ArmISA::MISCREG_MAIR_EL3, "MAIR_EL3" },
      { ArmISA::MISCREG_AMAIR_EL3, "AMAIR_EL3" },
      // ArmISA::MISCREG_L2CTLR_EL1?
      // ArmISA::MISCREG_L2ECTLR_EL1?
      { ArmISA::MISCREG_VBAR_EL1, "VBAR_EL1" },
      // ArmISA::MISCREG_RVBAR_EL1?
      { ArmISA::MISCREG_ISR_EL1, "ISR_EL1" },
      { ArmISA::MISCREG_VBAR_EL2, "VBAR_EL2" },
      // ArmISA::MISCREG_RVBAR_EL2?
      { ArmISA::MISCREG_VBAR_EL3, "VBAR_EL3" },
      { ArmISA::MISCREG_RVBAR_EL3, "RVBAR_EL3" },
      { ArmISA::MISCREG_RMR_EL3, "RMR_EL3" },
      { ArmISA::MISCREG_CONTEXTIDR_EL1, "CONTEXTIDR_EL1" },
      { ArmISA::MISCREG_TPIDR_EL1, "TPIDR_EL1" },
      { ArmISA::MISCREG_TPIDR_EL0, "TPIDR_EL0" },
      { ArmISA::MISCREG_TPIDRRO_EL0, "TPIDRRO_EL0" },
      { ArmISA::MISCREG_TPIDR_EL2, "TPIDR_EL2" },
      { ArmISA::MISCREG_TPIDR_EL3, "TPIDR_EL3" },
      { ArmISA::MISCREG_CNTKCTL_EL1, "CNTKCTL_EL1" },
      { ArmISA::MISCREG_CNTFRQ_EL0, "CNTFRQ_EL0" },
      { ArmISA::MISCREG_CNTPCT_EL0, "CNTPCT_EL0" },
      { ArmISA::MISCREG_CNTVCT_EL0, "CNTVCT_EL0" },
      { ArmISA::MISCREG_CNTP_TVAL_EL0, "CNTP_TVAL_EL0" },
      { ArmISA::MISCREG_CNTP_CTL_EL0, "CNTP_CTL_EL0" },
      { ArmISA::MISCREG_CNTP_CVAL_EL0, "CNTP_CVAL_EL0" },
      { ArmISA::MISCREG_CNTV_TVAL_EL0, "CNTV_TVAL_EL0" },
      { ArmISA::MISCREG_CNTV_CTL_EL0, "CNTV_CTL_EL0" },
      { ArmISA::MISCREG_CNTV_CVAL_EL0, "CNTV_CVAL_EL0" },
      { ArmISA::MISCREG_PMEVCNTR0_EL0, "PMEVCNTR0_EL0" },
      { ArmISA::MISCREG_PMEVCNTR1_EL0, "PMEVCNTR1_EL0" },
      { ArmISA::MISCREG_PMEVCNTR2_EL0, "PMEVCNTR2_EL0" },
      { ArmISA::MISCREG_PMEVCNTR3_EL0, "PMEVCNTR3_EL0" },
      { ArmISA::MISCREG_PMEVCNTR4_EL0, "PMEVCNTR4_EL0" },
      { ArmISA::MISCREG_PMEVCNTR5_EL0, "PMEVCNTR5_EL0" },
      { ArmISA::MISCREG_PMEVTYPER0_EL0, "PMEVTYPER0_EL0" },
      { ArmISA::MISCREG_PMEVTYPER1_EL0, "PMEVTYPER1_EL0" },
      { ArmISA::MISCREG_PMEVTYPER2_EL0, "PMEVTYPER2_EL0" },
      { ArmISA::MISCREG_PMEVTYPER3_EL0, "PMEVTYPER3_EL0" },
      { ArmISA::MISCREG_PMEVTYPER4_EL0, "PMEVTYPER4_EL0" },
      { ArmISA::MISCREG_PMEVTYPER5_EL0, "PMEVTYPER5_EL0" },
      { ArmISA::MISCREG_CNTVOFF_EL2, "CNTVOFF_EL2" },
      { ArmISA::MISCREG_CNTHCTL_EL2, "CNTHCTL_EL2" },
      { ArmISA::MISCREG_CNTHP_TVAL_EL2, "CNTHP_TVAL_EL2" },
      { ArmISA::MISCREG_CNTHP_CTL_EL2, "CNTHP_CTL_EL2" },
      { ArmISA::MISCREG_CNTHP_CVAL_EL2, "CNTHP_CVAL_EL2" },
      { ArmISA::MISCREG_CNTPS_TVAL_EL1, "CNTPS_TVAL_EL1" },
      { ArmISA::MISCREG_CNTPS_CTL_EL1, "CNTPS_CTL_EL1" },
      { ArmISA::MISCREG_CNTPS_CVAL_EL1, "CNTPS_CVAL_EL1" },
      // ArmISA::MISCREG_IL1DATA0_EL1?
      // ArmISA::MISCREG_IL1DATA1_EL1?
      // ArmISA::MISCREG_IL1DATA2_EL1?
      // ArmISA::MISCREG_IL1DATA3_EL1?
      // ArmISA::MISCREG_DL1DATA0_EL1?
      // ArmISA::MISCREG_DL1DATA1_EL1?
      // ArmISA::MISCREG_DL1DATA2_EL1?
      // ArmISA::MISCREG_DL1DATA3_EL1?
      // ArmISA::MISCREG_DL1DATA4_EL1?
      // ArmISA::MISCREG_L2ACTLR_EL1?
      { ArmISA::MISCREG_CPUACTLR_EL1, "CPUACTLR_EL1" },
      { ArmISA::MISCREG_CPUECTLR_EL1, "CPUECTLR_EL1" },
      // ArmISA::MISCREG_CPUMERRSR_EL1?
      // ArmISA::MISCREG_L2MERRSR_EL1?
      // ArmISA::MISCREG_CBAR_EL1?
      { ArmISA::MISCREG_CONTEXTIDR_EL2, "CONTEXTIDR_EL2" },

      // Introduced in ARMv8.1
      { ArmISA::MISCREG_TTBR1_EL2, "TTBR1_EL2" },
      { ArmISA::MISCREG_CNTHV_CTL_EL2, "CNTHV_CTL_EL2" },
      { ArmISA::MISCREG_CNTHV_CVAL_EL2, "CNTHV_CVAL_EL2" },
      { ArmISA::MISCREG_CNTHV_TVAL_EL2, "CNTHV_TVAL_EL2" },

      // RAS extension (unimplemented)
      { ArmISA::MISCREG_ERRIDR_EL1, "ERRIDR_EL1" },
      { ArmISA::MISCREG_ERRSELR_EL1, "ERRSELR_EL1" },
      { ArmISA::MISCREG_ERXFR_EL1, "ERXFR_EL1" },
      { ArmISA::MISCREG_ERXCTLR_EL1, "ERXCTLR_EL1" },
      { ArmISA::MISCREG_ERXSTATUS_EL1, "ERXSTATUS_EL1" },
      { ArmISA::MISCREG_ERXADDR_EL1, "ERXADDR_EL1" },
      { ArmISA::MISCREG_ERXMISC0_EL1, "ERXMISC0_EL1" },
      { ArmISA::MISCREG_ERXMISC1_EL1, "ERXMISC1_EL1" },
      { ArmISA::MISCREG_DISR_EL1, "DISR_EL1" },
      { ArmISA::MISCREG_VSESR_EL2, "VSESR_EL2" },
      { ArmISA::MISCREG_VDISR_EL2, "VDISR_EL2" } });

Iris::ThreadContext::IdxNameMap
    CortexA76TC::intReg32IdxNameMap({ { ArmISA::int_reg::R0, "R0" },
                                      { ArmISA::int_reg::R1, "R1" },
                                      { ArmISA::int_reg::R2, "R2" },
                                      { ArmISA::int_reg::R3, "R3" },
                                      { ArmISA::int_reg::R4, "R4" },
                                      { ArmISA::int_reg::R5, "R5" },
                                      { ArmISA::int_reg::R6, "R6" },
                                      { ArmISA::int_reg::R7, "R7" },
                                      { ArmISA::int_reg::R8, "R8" },
                                      { ArmISA::int_reg::R9, "R9" },
                                      { ArmISA::int_reg::R10, "R10" },
                                      { ArmISA::int_reg::R11, "R11" },
                                      { ArmISA::int_reg::R12, "R12" },
                                      { ArmISA::int_reg::R13, "R13" },
                                      { ArmISA::int_reg::R14, "R14" },
                                      { ArmISA::int_reg::R15, "R15" } });

Iris::ThreadContext::IdxNameMap CortexA76TC::intReg64IdxNameMap({
    { ArmISA::int_reg::X0, "X0" },   { ArmISA::int_reg::X1, "X1" },
    { ArmISA::int_reg::X2, "X2" },   { ArmISA::int_reg::X3, "X3" },
    { ArmISA::int_reg::X4, "X4" },   { ArmISA::int_reg::X5, "X5" },
    { ArmISA::int_reg::X6, "X6" },   { ArmISA::int_reg::X7, "X7" },
    { ArmISA::int_reg::X8, "X8" },   { ArmISA::int_reg::X9, "X9" },
    { ArmISA::int_reg::X10, "X10" }, { ArmISA::int_reg::X11, "X11" },
    { ArmISA::int_reg::X12, "X12" }, { ArmISA::int_reg::X13, "X13" },
    { ArmISA::int_reg::X14, "X14" }, { ArmISA::int_reg::X15, "X15" },
    { ArmISA::int_reg::X16, "X16" }, { ArmISA::int_reg::X17, "X17" },
    { ArmISA::int_reg::X18, "X18" }, { ArmISA::int_reg::X19, "X19" },
    { ArmISA::int_reg::X20, "X20" }, { ArmISA::int_reg::X21, "X21" },
    { ArmISA::int_reg::X22, "X22" }, { ArmISA::int_reg::X23, "X23" },
    { ArmISA::int_reg::X24, "X24" }, { ArmISA::int_reg::X25, "X25" },
    { ArmISA::int_reg::X26, "X26" }, { ArmISA::int_reg::X27, "X27" },
    { ArmISA::int_reg::X28, "X28" }, { ArmISA::int_reg::X29, "X29" },
    { ArmISA::int_reg::X30, "X30" }, { ArmISA::int_reg::Spx, "SP" },
});

Iris::ThreadContext::IdxNameMap CortexA76TC::flattenedIntIdxNameMap({
    { ArmISA::int_reg::R0, "X0" },
    { ArmISA::int_reg::R1, "X1" },
    { ArmISA::int_reg::R2, "X2" },
    { ArmISA::int_reg::R3, "X3" },
    { ArmISA::int_reg::R4, "X4" },
    { ArmISA::int_reg::R5, "X5" },
    { ArmISA::int_reg::R6, "X6" },
    { ArmISA::int_reg::R7, "X7" },
    { ArmISA::int_reg::R8, "X8" },
    { ArmISA::int_reg::R9, "X9" },
    { ArmISA::int_reg::R10, "X10" },
    { ArmISA::int_reg::R11, "X11" },
    { ArmISA::int_reg::R12, "X12" },
    { ArmISA::int_reg::R13, "X13" },
    { ArmISA::int_reg::R14, "X14" },
    // Skip PC.
    { ArmISA::int_reg::R13Svc, "X19" },
    { ArmISA::int_reg::R14Svc, "X18" },
    { ArmISA::int_reg::R13Mon, "R13" }, // Need to be in monitor mode?
    { ArmISA::int_reg::R14Mon, "R14" }, // Need to be in monitor mode?
    { ArmISA::int_reg::R13Hyp, "X15" },
    { ArmISA::int_reg::R13Abt, "X21" },
    { ArmISA::int_reg::R14Abt, "X20" },
    { ArmISA::int_reg::R13Und, "X23" },
    { ArmISA::int_reg::R14Und, "X22" },
    { ArmISA::int_reg::R13Irq, "X17" },
    { ArmISA::int_reg::R14Irq, "X16" },
    { ArmISA::int_reg::R8Fiq, "X24" },
    { ArmISA::int_reg::R9Fiq, "X25" },
    { ArmISA::int_reg::R10Fiq, "X26" },
    { ArmISA::int_reg::R11Fiq, "X27" },
    { ArmISA::int_reg::R12Fiq, "X28" },
    { ArmISA::int_reg::R13Fiq, "X29" },
    { ArmISA::int_reg::R14Fiq, "X30" },
    // Skip zero, ureg0-2, and dummy regs.
    { ArmISA::int_reg::Sp0, "SP_EL0" },
    { ArmISA::int_reg::Sp1, "SP_EL1" },
    { ArmISA::int_reg::Sp2, "SP_EL2" },
    { ArmISA::int_reg::Sp3, "SP_EL3" },
});

Iris::ThreadContext::IdxNameMap CortexA76TC::ccRegIdxNameMap({
    { ArmISA::cc_reg::Nz, "CPSR" },
    { ArmISA::cc_reg::C, "CPSR.C" },
    { ArmISA::cc_reg::V, "CPSR.V" },
    { ArmISA::cc_reg::Ge, "CPSR.GE" },
    { ArmISA::cc_reg::Fp, "FPSCR" },
});

Iris::ThreadContext::IdxNameMap CortexA76TC::vecRegIdxNameMap(
    { { 0, "V0" },   { 1, "V1" },   { 2, "V2" },   { 3, "V3" },
      { 4, "V4" },   { 5, "V5" },   { 6, "V6" },   { 7, "V7" },
      { 8, "V8" },   { 9, "V9" },   { 10, "V10" }, { 11, "V11" },
      { 12, "V12" }, { 13, "V13" }, { 14, "V14" }, { 15, "V15" },
      { 16, "V16" }, { 17, "V17" }, { 18, "V18" }, { 19, "V19" },
      { 20, "V20" }, { 21, "V21" }, { 22, "V22" }, { 23, "V23" },
      { 24, "V24" }, { 25, "V25" }, { 26, "V26" }, { 27, "V27" },
      { 28, "V28" }, { 29, "V29" }, { 30, "V30" }, { 31, "V31" } });

std::vector<iris::MemorySpaceId> CortexA76TC::bpSpaceIds;

} // namespace fastmodel
} // namespace gem5
