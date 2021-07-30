/*
 * Copyright 2020 Google, Inc.
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

#include "arch/arm/fastmodel/CortexR52/thread_context.hh"

#include "arch/arm/fastmodel/iris/memory_spaces.hh"
#include "arch/arm/utility.hh"
#include "iris/detail/IrisCppAdapter.h"
#include "iris/detail/IrisObjects.h"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(FastModel, fastmodel);
namespace fastmodel
{

CortexR52TC::CortexR52TC(
        gem5::BaseCPU *cpu, int id, System *system, gem5::BaseMMU *mmu,
        gem5::BaseISA *isa, iris::IrisConnectionInterface *iris_if,
        const std::string &iris_path) :
    ThreadContext(cpu, id, system, mmu, isa, iris_if, iris_path)
{}

bool
CortexR52TC::translateAddress(Addr &paddr, Addr vaddr)
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
        Iris::PhysicalMemorySecureMsn : Iris::PhysicalMemoryNonSecureMsn;

    // Figure out what memory spaces match the canonical numbers we need.
    iris::MemorySpaceId in = getMemorySpaceId(in_msn);
    iris::MemorySpaceId out = getMemorySpaceId(out_msn);

    panic_if(in == iris::IRIS_UINT64_MAX || out == iris::IRIS_UINT64_MAX,
            "Canonical IRIS memory space numbers not found.");

    return ThreadContext::translateAddress(paddr, out, vaddr, in);
}

void
CortexR52TC::initFromIrisInstance(const ResourceMap &resources)
{
    ThreadContext::initFromIrisInstance(resources);

    pcRscId = extractResourceId(resources, "R15");

    extractResourceMap(miscRegIds, resources, miscRegIdxNameMap);

    extractResourceMap(intReg32Ids, resources, intReg32IdxNameMap);
    extractResourceMap(ccRegIds, resources, ccRegIdxNameMap);
}

RegVal
CortexR52TC::readIntReg(RegIndex reg_idx) const
{
    iris::ResourceReadResult result;
    call().resource_read(_instId, result, intReg32Ids.at(reg_idx));
    return result.data.at(0);
}

void
CortexR52TC::setIntReg(RegIndex reg_idx, RegVal val)
{
    iris::ResourceWriteResult result;
    call().resource_write(_instId, result, intReg32Ids.at(reg_idx), val);
}

RegVal
CortexR52TC::readCCRegFlat(RegIndex idx) const
{
    RegVal result = Iris::ThreadContext::readCCRegFlat(idx);
    switch (idx) {
      case ArmISA::CCREG_NZ:
        result = ((ArmISA::CPSR)result).nz;
        break;
      case ArmISA::CCREG_FP:
        result = bits(result, 31, 28);
        break;
      default:
        break;
    }
    return result;
}

void
CortexR52TC::setCCRegFlat(RegIndex idx, RegVal val)
{
    switch (idx) {
      case ArmISA::CCREG_NZ:
        {
            ArmISA::CPSR cpsr = readMiscRegNoEffect(ArmISA::MISCREG_CPSR);
            cpsr.nz = val;
            val = cpsr;
        }
        break;
      case ArmISA::CCREG_FP:
        {
            ArmISA::FPSCR fpscr = readMiscRegNoEffect(ArmISA::MISCREG_FPSCR);
            val = insertBits(fpscr, 31, 28, val);
        }
        break;
      default:
        break;
    }
    Iris::ThreadContext::setCCRegFlat(idx, val);
}

const std::vector<iris::MemorySpaceId> &
CortexR52TC::getBpSpaceIds() const
{
    if (bpSpaceIds.empty()) {
        std::vector<Iris::CanonicalMsn> msns{
            Iris::SecureMonitorMsn, Iris::GuestMsn, Iris::NsHypMsn,
            Iris::HypAppMsn};
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

Iris::ThreadContext::IdxNameMap CortexR52TC::miscRegIdxNameMap({
        { ArmISA::MISCREG_CPSR, "CPSR" },
        { ArmISA::MISCREG_SPSR, "SPSR" },
        // ArmISA::MISCREG_SPSR_FIQ?
        // ArmISA::MISCREG_SPSR_IRQ?
        // ArmISA::MISCREG_SPSR_SVC?
        // ArmISA::MISCREG_SPSR_MON?
        // ArmISA::MISCREG_SPSR_ABT?
        // ArmISA::MISCREG_SPSR_HYP?
        // ArmISA::MISCREG_SPSR_UND?
        // ArmISA::MISCREG_ELR_HYP?
        // ArmISA::MISCREG_FPSID?
        // ArmISA::MISCREG_FPSCR?
        // ArmISA::MISCREG_MVFR1?
        // ArmISA::MISCREG_MVFR0?
        // ArmISA::MISCREG_FPEXC?

        // Helper registers
        // ArmISA::MISCREG_CPSR_MODE?
        // ArmISA::MISCREG_CPSR_Q?
        // ArmISA::MISCREG_FPSCR_EXC?
        // ArmISA::MISCREG_FPSCR_QC?
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
        // ArmISA::MISCREG_SEV_MAILBOX?

        // AArch32 CP14 registers (debug/trace/ThumbEE/Jazelle control)
        // ArmISA::MISCREG_DBGDIDR?
        // ArmISA::MISCREG_DBGDSCRint?
        // ArmISA::MISCREG_DBGDCCINT?
        // ArmISA::MISCREG_DBGDTRTXint?
        // ArmISA::MISCREG_DBGDTRRXint?
        // ArmISA::MISCREG_DBGWFAR?
        // ArmISA::MISCREG_DBGVCR?
        // ArmISA::MISCREG_DBGDTRRXext?
        // ArmISA::MISCREG_DBGDSCRext?
        // ArmISA::MISCREG_DBGDTRTXext?
        // ArmISA::MISCREG_DBGOSECCR?
        // ArmISA::MISCREG_DBGBVR0?
        // ArmISA::MISCREG_DBGBVR1?
        // ArmISA::MISCREG_DBGBVR2?
        // ArmISA::MISCREG_DBGBVR3?
        // ArmISA::MISCREG_DBGBVR4?
        // ArmISA::MISCREG_DBGBVR5?
        // ArmISA::MISCREG_DBGBCR0?
        // ArmISA::MISCREG_DBGBCR1?
        // ArmISA::MISCREG_DBGBCR2?
        // ArmISA::MISCREG_DBGBCR3?
        // ArmISA::MISCREG_DBGBCR4?
        // ArmISA::MISCREG_DBGBCR5?
        // ArmISA::MISCREG_DBGWVR0?
        // ArmISA::MISCREG_DBGWVR1?
        // ArmISA::MISCREG_DBGWVR2?
        // ArmISA::MISCREG_DBGWVR3?
        // ArmISA::MISCREG_DBGWCR0?
        // ArmISA::MISCREG_DBGWCR1?
        // ArmISA::MISCREG_DBGWCR2?
        // ArmISA::MISCREG_DBGWCR3?
        // ArmISA::MISCREG_DBGDRAR?
        // ArmISA::MISCREG_DBGBXVR4?
        // ArmISA::MISCREG_DBGBXVR5?
        // ArmISA::MISCREG_DBGOSLAR?
        // ArmISA::MISCREG_DBGOSLSR?
        // ArmISA::MISCREG_DBGOSDLR?
        // ArmISA::MISCREG_DBGPRCR?
        // ArmISA::MISCREG_DBGDSAR?
        // ArmISA::MISCREG_DBGCLAIMSET?
        // ArmISA::MISCREG_DBGCLAIMCLR?
        // ArmISA::MISCREG_DBGAUTHSTATUS?
        // ArmISA::MISCREG_DBGDEVID2?
        // ArmISA::MISCREG_DBGDEVID1?
        // ArmISA::MISCREG_DBGDEVID0?
        // ArmISA::MISCREG_TEECR? not in ARM DDI 0487A.b+
        // ArmISA::MISCREG_JIDR?
        // ArmISA::MISCREG_TEEHBR? not in ARM DDI 0487A.b+
        // ArmISA::MISCREG_JOSCR?
        // ArmISA::MISCREG_JMCR?

        // AArch32 CP15 registers (system control)
        // ArmISA::MISCREG_MIDR?
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
        // ArmISA::MISCREG_VMPIDR?
        // ArmISA::MISCREG_SCTLR?
        // ArmISA::MISCREG_SCTLR_NS?
        // ArmISA::MISCREG_SCTLR_S?
        // ArmISA::MISCREG_ACTLR?
        // ArmISA::MISCREG_ACTLR_NS?
        // ArmISA::MISCREG_ACTLR_S?
        // ArmISA::MISCREG_CPACR?
        // ArmISA::MISCREG_SCR?
        // ArmISA::MISCREG_SDER?
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
        // ArmISA::MISCREG_PMCR?
        // ArmISA::MISCREG_PMCNTENSET?
        // ArmISA::MISCREG_PMCNTENCLR?
        // ArmISA::MISCREG_PMOVSR?
        // ArmISA::MISCREG_PMSWINC?
        // ArmISA::MISCREG_PMSELR?
        // ArmISA::MISCREG_PMCEID0?
        // ArmISA::MISCREG_PMCEID1?
        // ArmISA::MISCREG_PMCCNTR?
        // ArmISA::MISCREG_PMXEVTYPER?
        // ArmISA::MISCREG_PMCCFILTR?
        // ArmISA::MISCREG_PMXEVCNTR?
        // ArmISA::MISCREG_PMUSERENR?
        // ArmISA::MISCREG_PMINTENSET?
        // ArmISA::MISCREG_PMINTENCLR?
        // ArmISA::MISCREG_PMOVSSET?
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
        // ArmISA::MISCREG_AMAIR0_NS?
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
        // ArmISA::MISCREG_MVBAR?
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
        /// ArmISA::MISCREG_TPIDRPRW_S?
        // ArmISA::MISCREG_HTPIDR?
        // ArmISA::MISCREG_CNTFRQ?
        // ArmISA::MISCREG_CNTKCTL?
        // ArmISA::MISCREG_CNTP_TVAL?
        // ArmISA::MISCREG_CNTP_TVAL_NS?
        // ArmISA::MISCREG_CNTP_TVAL_S?
        // ArmISA::MISCREG_CNTP_CTL?
        // ArmISA::MISCREG_CNTP_CTL_NS?
        // ArmISA::MISCREG_CNTP_CTL_S?
        // ArmISA::MISCREG_CNTV_TVAL?
        // ArmISA::MISCREG_CNTV_CTL?
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
        // ArmISA::MISCREG_CNTPCT?
        // ArmISA::MISCREG_CNTVCT?
        // ArmISA::MISCREG_CNTP_CVAL?
        // ArmISA::MISCREG_CNTP_CVAL_NS?
        // ArmISA::MISCREG_CNTP_CVAL_S?
        // ArmISA::MISCREG_CNTV_CVAL?
        // ArmISA::MISCREG_CNTVOFF?
        // ArmISA::MISCREG_CNTHP_CVAL?
        // ArmISA::MISCREG_CPUMERRSR?
        // ArmISA::MISCREG_L2MERRSR?

        // AArch64 registers (Op0=2)
        // ArmISA::MISCREG_MDCCINT_EL1?
        // ArmISA::MISCREG_OSDTRRX_EL1?
        // ArmISA::MISCREG_MDSCR_EL1?
        // ArmISA::MISCREG_OSDTRTX_EL1?
        // ArmISA::MISCREG_OSECCR_EL1?
        // ArmISA::MISCREG_DBGBVR0_EL1?
        // ArmISA::MISCREG_DBGBVR1_EL1?
        // ArmISA::MISCREG_DBGBVR2_EL1?
        // ArmISA::MISCREG_DBGBVR3_EL1?
        // ArmISA::MISCREG_DBGBVR4_EL1?
        // ArmISA::MISCREG_DBGBVR5_EL1?
        // ArmISA::MISCREG_DBGBCR0_EL1?
        // ArmISA::MISCREG_DBGBCR1_EL1?
        // ArmISA::MISCREG_DBGBCR2_EL1?
        // ArmISA::MISCREG_DBGBCR3_EL1?
        // ArmISA::MISCREG_DBGBCR4_EL1?
        // ArmISA::MISCREG_DBGBCR5_EL1?
        // ArmISA::MISCREG_DBGWVR0_EL1?
        // ArmISA::MISCREG_DBGWVR1_EL1?
        // ArmISA::MISCREG_DBGWVR2_EL1?
        // ArmISA::MISCREG_DBGWVR3_EL1?
        // ArmISA::MISCREG_DBGWCR0_EL1?
        // ArmISA::MISCREG_DBGWCR1_EL1?
        // ArmISA::MISCREG_DBGWCR2_EL1?
        // ArmISA::MISCREG_DBGWCR3_EL1?
        // ArmISA::MISCREG_MDCCSR_EL0?
        // ArmISA::MISCREG_MDDTR_EL0?
        // ArmISA::MISCREG_MDDTRTX_EL0?
        // ArmISA::MISCREG_MDDTRRX_EL0?
        // ArmISA::MISCREG_DBGVCR32_EL2?
        // ArmISA::MISCREG_MDRAR_EL1?
        // ArmISA::MISCREG_OSLAR_EL1?
        // ArmISA::MISCREG_OSLSR_EL1?
        // ArmISA::MISCREG_OSDLR_EL1?
        // ArmISA::MISCREG_DBGPRCR_EL1?
        // ArmISA::MISCREG_DBGCLAIMSET_EL1?
        // ArmISA::MISCREG_DBGCLAIMCLR_EL1?
        // ArmISA::MISCREG_DBGAUTHSTATUS_EL1?
        // ArmISA::MISCREG_TEECR32_EL1? not in ARM DDI 0487A.b+
        // ArmISA::MISCREG_TEEHBR32_EL1? not in ARM DDI 0487A.b+

        // AArch64 registers (Op0=1)
        // ArmISA::MISCREG_MIDR_EL1?
        // ArmISA::MISCREG_MPIDR_EL1?
        // ArmISA::MISCREG_REVIDR_EL1?
        // ArmISA::MISCREG_ID_PFR0_EL1?
        // ArmISA::MISCREG_ID_PFR1_EL1?
        // ArmISA::MISCREG_ID_DFR0_EL1?
        // ArmISA::MISCREG_ID_AFR0_EL1?
        // ArmISA::MISCREG_ID_MMFR0_EL1?
        // ArmISA::MISCREG_ID_MMFR1_EL1?
        // ArmISA::MISCREG_ID_MMFR2_EL1?
        // ArmISA::MISCREG_ID_MMFR3_EL1?
        // ArmISA::MISCREG_ID_MMFR4_EL1?
        // ArmISA::MISCREG_ID_ISAR0_EL1?
        // ArmISA::MISCREG_ID_ISAR1_EL1?
        // ArmISA::MISCREG_ID_ISAR2_EL1?
        // ArmISA::MISCREG_ID_ISAR3_EL1?
        // ArmISA::MISCREG_ID_ISAR4_EL1?
        // ArmISA::MISCREG_ID_ISAR5_EL1?
        // ArmISA::MISCREG_ID_ISAR6_EL1?
        // ArmISA::MISCREG_MVFR0_EL1?
        // ArmISA::MISCREG_MVFR1_EL1?
        // ArmISA::MISCREG_MVFR2_EL1?
        // ArmISA::MISCREG_ID_AA64PFR0_EL1?
        // ArmISA::MISCREG_ID_AA64PFR1_EL1?
        // ArmISA::MISCREG_ID_AA64DFR0_EL1?
        // ArmISA::MISCREG_ID_AA64DFR1_EL1?
        // ArmISA::MISCREG_ID_AA64AFR0_EL1?
        // ArmISA::MISCREG_ID_AA64AFR1_EL1?
        // ArmISA::MISCREG_ID_AA64ISAR0_EL1?
        // ArmISA::MISCREG_ID_AA64ISAR1_EL1?
        // ArmISA::MISCREG_ID_AA64MMFR0_EL1?
        // ArmISA::MISCREG_ID_AA64MMFR1_EL1?
        // ArmISA::MISCREG_CCSIDR_EL1?
        // ArmISA::MISCREG_CLIDR_EL1?
        // ArmISA::MISCREG_AIDR_EL1?
        // ArmISA::MISCREG_CSSELR_EL1?
        // ArmISA::MISCREG_CTR_EL0?
        // ArmISA::MISCREG_DCZID_EL0?
        // ArmISA::MISCREG_VPIDR_EL2?
        // ArmISA::MISCREG_VMPIDR_EL2?
        // ArmISA::MISCREG_SCTLR_EL1?
        // ArmISA::MISCREG_ACTLR_EL1?
        // ArmISA::MISCREG_CPACR_EL1?
        // ArmISA::MISCREG_SCTLR_EL2?
        // ArmISA::MISCREG_ACTLR_EL2?
        // ArmISA::MISCREG_HCR_EL2?
        // ArmISA::MISCREG_MDCR_EL2?
        // ArmISA::MISCREG_CPTR_EL2?
        // ArmISA::MISCREG_HSTR_EL2?
        // ArmISA::MISCREG_HACR_EL2?
        // ArmISA::MISCREG_SCTLR_EL3?
        // ArmISA::MISCREG_ACTLR_EL3?
        // ArmISA::MISCREG_SCR_EL3?
        // ArmISA::MISCREG_SDER32_EL3?
        // ArmISA::MISCREG_CPTR_EL3?
        // ArmISA::MISCREG_MDCR_EL3?
        // ArmISA::MISCREG_TTBR0_EL1?
        // ArmISA::MISCREG_TTBR1_EL1?
        // ArmISA::MISCREG_TCR_EL1?
        // ArmISA::MISCREG_TTBR0_EL2?
        // ArmISA::MISCREG_TCR_EL2?
        // ArmISA::MISCREG_VTTBR_EL2?
        // ArmISA::MISCREG_VTCR_EL2?
        // ArmISA::MISCREG_TTBR0_EL3?
        // ArmISA::MISCREG_TCR_EL3?
        // ArmISA::MISCREG_DACR32_EL2?
        // ArmISA::MISCREG_SPSR_EL1?
        // ArmISA::MISCREG_ELR_EL1?
        // ArmISA::MISCREG_SP_EL0?
        // ArmISA::MISCREG_SPSEL?
        // ArmISA::MISCREG_CURRENTEL?
        // ArmISA::MISCREG_NZCV?
        // ArmISA::MISCREG_DAIF?
        // ArmISA::MISCREG_FPCR?
        // ArmISA::MISCREG_FPSR?
        // ArmISA::MISCREG_DSPSR_EL0?
        // ArmISA::MISCREG_DLR_EL0?
        // ArmISA::MISCREG_SPSR_EL2?
        // ArmISA::MISCREG_ELR_EL2?
        // ArmISA::MISCREG_SP_EL1?
        // ArmISA::MISCREG_SPSR_IRQ_AA64?
        // ArmISA::MISCREG_SPSR_ABT_AA64?
        // ArmISA::MISCREG_SPSR_UND_AA64?
        // ArmISA::MISCREG_SPSR_FIQ_AA64?
        // ArmISA::MISCREG_SPSR_EL3?
        // ArmISA::MISCREG_ELR_EL3?
        // ArmISA::MISCREG_SP_EL2?
        // ArmISA::MISCREG_AFSR0_EL1?
        // ArmISA::MISCREG_AFSR1_EL1?
        // ArmISA::MISCREG_ESR_EL1?
        // ArmISA::MISCREG_IFSR32_EL2?
        // ArmISA::MISCREG_AFSR0_EL2?
        // ArmISA::MISCREG_AFSR1_EL2?
        // ArmISA::MISCREG_ESR_EL2?
        // ArmISA::MISCREG_FPEXC32_EL2?
        // ArmISA::MISCREG_AFSR0_EL3?
        // ArmISA::MISCREG_AFSR1_EL3?
        // ArmISA::MISCREG_ESR_EL3?
        // ArmISA::MISCREG_FAR_EL1?
        // ArmISA::MISCREG_FAR_EL2?
        // ArmISA::MISCREG_HPFAR_EL2?
        // ArmISA::MISCREG_FAR_EL3?
        // ArmISA::MISCREG_IC_IALLUIS?
        // ArmISA::MISCREG_PAR_EL1?
        // ArmISA::MISCREG_IC_IALLU?
        // ArmISA::MISCREG_DC_IVAC_Xt?
        // ArmISA::MISCREG_DC_ISW_Xt?
        // ArmISA::MISCREG_AT_S1E1R_Xt?
        // ArmISA::MISCREG_AT_S1E1W_Xt?
        // ArmISA::MISCREG_AT_S1E0R_Xt?
        // ArmISA::MISCREG_AT_S1E0W_Xt?
        // ArmISA::MISCREG_DC_CSW_Xt?
        // ArmISA::MISCREG_DC_CISW_Xt?
        // ArmISA::MISCREG_DC_ZVA_Xt?
        // ArmISA::MISCREG_IC_IVAU_Xt?
        // ArmISA::MISCREG_DC_CVAC_Xt?
        // ArmISA::MISCREG_DC_CVAU_Xt?
        // ArmISA::MISCREG_DC_CIVAC_Xt?
        // ArmISA::MISCREG_AT_S1E2R_Xt?
        // ArmISA::MISCREG_AT_S1E2W_Xt?
        // ArmISA::MISCREG_AT_S12E1R_Xt?
        // ArmISA::MISCREG_AT_S12E1W_Xt?
        // ArmISA::MISCREG_AT_S12E0R_Xt?
        // ArmISA::MISCREG_AT_S12E0W_Xt?
        // ArmISA::MISCREG_AT_S1E3R_Xt?
        // ArmISA::MISCREG_AT_S1E3W_Xt?
        // ArmISA::MISCREG_TLBI_VMALLE1IS?
        // ArmISA::MISCREG_TLBI_VAE1IS_Xt?
        // ArmISA::MISCREG_TLBI_ASIDE1IS_Xt?
        // ArmISA::MISCREG_TLBI_VAAE1IS_Xt?
        // ArmISA::MISCREG_TLBI_VALE1IS_Xt?
        // ArmISA::MISCREG_TLBI_VAALE1IS_Xt?
        // ArmISA::MISCREG_TLBI_VMALLE1?
        // ArmISA::MISCREG_TLBI_VAE1_Xt?
        // ArmISA::MISCREG_TLBI_ASIDE1_Xt?
        // ArmISA::MISCREG_TLBI_VAAE1_Xt?
        // ArmISA::MISCREG_TLBI_VALE1_Xt?
        // ArmISA::MISCREG_TLBI_VAALE1_Xt?
        // ArmISA::MISCREG_TLBI_IPAS2E1IS_Xt?
        // ArmISA::MISCREG_TLBI_IPAS2LE1IS_Xt?
        // ArmISA::MISCREG_TLBI_ALLE2IS?
        // ArmISA::MISCREG_TLBI_VAE2IS_Xt?
        // ArmISA::MISCREG_TLBI_ALLE1IS?
        // ArmISA::MISCREG_TLBI_VALE2IS_Xt?
        // ArmISA::MISCREG_TLBI_VMALLS12E1IS?
        // ArmISA::MISCREG_TLBI_IPAS2E1_Xt?
        // ArmISA::MISCREG_TLBI_IPAS2LE1_Xt?
        // ArmISA::MISCREG_TLBI_ALLE2?
        // ArmISA::MISCREG_TLBI_VAE2_Xt?
        // ArmISA::MISCREG_TLBI_ALLE1?
        // ArmISA::MISCREG_TLBI_VALE2_Xt?
        // ArmISA::MISCREG_TLBI_VMALLS12E1?
        // ArmISA::MISCREG_TLBI_ALLE3IS?
        // ArmISA::MISCREG_TLBI_VAE3IS_Xt?
        // ArmISA::MISCREG_TLBI_VALE3IS_Xt?
        // ArmISA::MISCREG_TLBI_ALLE3?
        // ArmISA::MISCREG_TLBI_VAE3_Xt?
        // ArmISA::MISCREG_TLBI_VALE3_Xt?
        // ArmISA::MISCREG_PMINTENSET_EL1?
        // ArmISA::MISCREG_PMINTENCLR_EL1?
        // ArmISA::MISCREG_PMCR_EL0?
        // ArmISA::MISCREG_PMCNTENSET_EL0?
        // ArmISA::MISCREG_PMCNTENCLR_EL0?
        // ArmISA::MISCREG_PMOVSCLR_EL0?
        // ArmISA::MISCREG_PMSWINC_EL0?
        // ArmISA::MISCREG_PMSELR_EL0?
        // ArmISA::MISCREG_PMCEID0_EL0?
        // ArmISA::MISCREG_PMCEID1_EL0?
        // ArmISA::MISCREG_PMCCNTR_EL0?
        // ArmISA::MISCREG_PMXEVTYPER_EL0?
        // ArmISA::MISCREG_PMCCFILTR_EL0?
        // ArmISA::MISCREG_PMXEVCNTR_EL0?
        // ArmISA::MISCREG_PMUSERENR_EL0?
        // ArmISA::MISCREG_PMOVSSET_EL0?
        // ArmISA::MISCREG_MAIR_EL1?
        // ArmISA::MISCREG_AMAIR_EL1?
        // ArmISA::MISCREG_MAIR_EL2?
        // ArmISA::MISCREG_AMAIR_EL2?
        // ArmISA::MISCREG_MAIR_EL3?
        // ArmISA::MISCREG_AMAIR_EL3?
        // ArmISA::MISCREG_L2CTLR_EL1?
        // ArmISA::MISCREG_L2ECTLR_EL1?
        // ArmISA::MISCREG_VBAR_EL1?
        // ArmISA::MISCREG_RVBAR_EL1?
        // ArmISA::MISCREG_ISR_EL1?
        // ArmISA::MISCREG_VBAR_EL2?
        // ArmISA::MISCREG_RVBAR_EL2?
        // ArmISA::MISCREG_VBAR_EL3?
        // ArmISA::MISCREG_RVBAR_EL3?
        // ArmISA::MISCREG_RMR_EL3?
        // ArmISA::MISCREG_CONTEXTIDR_EL1?
        // ArmISA::MISCREG_TPIDR_EL1?
        // ArmISA::MISCREG_TPIDR_EL0?
        // ArmISA::MISCREG_TPIDRRO_EL0?
        // ArmISA::MISCREG_TPIDR_EL2?
        // ArmISA::MISCREG_TPIDR_EL3?
        // ArmISA::MISCREG_CNTKCTL_EL1?
        // ArmISA::MISCREG_CNTFRQ_EL0?
        // ArmISA::MISCREG_CNTPCT_EL0?
        // ArmISA::MISCREG_CNTVCT_EL0?
        // ArmISA::MISCREG_CNTP_TVAL_EL0?
        // ArmISA::MISCREG_CNTP_CTL_EL0?
        // ArmISA::MISCREG_CNTP_CVAL_EL0?
        // ArmISA::MISCREG_CNTV_TVAL_EL0?
        // ArmISA::MISCREG_CNTV_CTL_EL0?
        // ArmISA::MISCREG_CNTV_CVAL_EL0?
        // ArmISA::MISCREG_PMEVCNTR0_EL0?
        // ArmISA::MISCREG_PMEVCNTR1_EL0?
        // ArmISA::MISCREG_PMEVCNTR2_EL0?
        // ArmISA::MISCREG_PMEVCNTR3_EL0?
        // ArmISA::MISCREG_PMEVCNTR4_EL0?
        // ArmISA::MISCREG_PMEVCNTR5_EL0?
        // ArmISA::MISCREG_PMEVTYPER0_EL0?
        // ArmISA::MISCREG_PMEVTYPER1_EL0?
        // ArmISA::MISCREG_PMEVTYPER2_EL0?
        // ArmISA::MISCREG_PMEVTYPER3_EL0?
        // ArmISA::MISCREG_PMEVTYPER4_EL0?
        // ArmISA::MISCREG_PMEVTYPER5_EL0?
        // ArmISA::MISCREG_CNTVOFF_EL2?
        // ArmISA::MISCREG_CNTHCTL_EL2?
        // ArmISA::MISCREG_CNTHP_TVAL_EL2?
        // ArmISA::MISCREG_CNTHP_CTL_EL2?
        // ArmISA::MISCREG_CNTHP_CVAL_EL2?
        // ArmISA::MISCREG_CNTPS_TVAL_EL1?
        // ArmISA::MISCREG_CNTPS_CTL_EL1?
        // ArmISA::MISCREG_CNTPS_CVAL_EL1?
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
        // ArmISA::MISCREG_CPUACTLR_EL1?
        // ArmISA::MISCREG_CPUECTLR_EL1?
        // ArmISA::MISCREG_CPUMERRSR_EL1?
        // ArmISA::MISCREG_L2MERRSR_EL1?
        // ArmISA::MISCREG_CBAR_EL1?
        // ArmISA::MISCREG_CONTEXTIDR_EL2?

        // Introduced in ARMv8.1
        // ArmISA::MISCREG_TTBR1_EL2?
        // ArmISA::MISCREG_CNTHV_CTL_EL2?
        // ArmISA::MISCREG_CNTHV_CVAL_EL2?
        // ArmISA::MISCREG_CNTHV_TVAL_EL2?

        // RAS extension (unimplemented)
        // ArmISA::MISCREG_ERRIDR_EL1?
        // ArmISA::MISCREG_ERRSELR_EL1?
        // ArmISA::MISCREG_ERXFR_EL1?
        // ArmISA::MISCREG_ERXCTLR_EL1?
        // ArmISA::MISCREG_ERXSTATUS_EL1?
        // ArmISA::MISCREG_ERXADDR_EL1?
        // ArmISA::MISCREG_ERXMISC0_EL1?
        // ArmISA::MISCREG_ERXMISC1_EL1?
        // ArmISA::MISCREG_DISR_EL1?
        // ArmISA::MISCREG_VSESR_EL2?
        // ArmISA::MISCREG_VDISR_EL2?
});

Iris::ThreadContext::IdxNameMap CortexR52TC::intReg32IdxNameMap({
        { ArmISA::INTREG_R0, "R0" },
        { ArmISA::INTREG_R1, "R1" },
        { ArmISA::INTREG_R2, "R2" },
        { ArmISA::INTREG_R3, "R3" },
        { ArmISA::INTREG_R4, "R4" },
        { ArmISA::INTREG_R5, "R5" },
        { ArmISA::INTREG_R6, "R6" },
        { ArmISA::INTREG_R7, "R7" },
        { ArmISA::INTREG_R8, "R8" },
        { ArmISA::INTREG_R9, "R9" },
        { ArmISA::INTREG_R10, "R10" },
        { ArmISA::INTREG_R11, "R11" },
        { ArmISA::INTREG_R12, "R12" },
        { ArmISA::INTREG_R13, "R13" },
        { ArmISA::INTREG_R14, "R14" },
        { ArmISA::INTREG_R15, "R15" }
});

Iris::ThreadContext::IdxNameMap CortexR52TC::ccRegIdxNameMap({
        { ArmISA::CCREG_NZ, "CPSR" },
        { ArmISA::CCREG_C, "CPSR.C" },
        { ArmISA::CCREG_V, "CPSR.V" },
        { ArmISA::CCREG_GE, "CPSR.GE" },
        { ArmISA::CCREG_FP, "FPSCR" },
});

std::vector<iris::MemorySpaceId> CortexR52TC::bpSpaceIds;

} // namespace fastmodel
} // namespace gem5
