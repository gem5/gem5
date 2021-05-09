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
    iris::MemorySpaceId in = iris::IRIS_UINT64_MAX;
    iris::MemorySpaceId out = iris::IRIS_UINT64_MAX;

    for (auto &space: memorySpaces) {
        if (space.canonicalMsn == in_msn)
            in = space.spaceId;
        else if (space.canonicalMsn == out_msn)
            out = space.spaceId;
    }

    panic_if(in == iris::IRIS_UINT64_MAX || out == iris::IRIS_UINT64_MAX,
            "Canonical IRIS memory space numbers not found.");

    return ThreadContext::translateAddress(paddr, out, vaddr, in);
}

void
CortexR52TC::initFromIrisInstance(const ResourceMap &resources)
{
    ThreadContext::initFromIrisInstance(resources);

    pcRscId = extractResourceId(resources, "R15");

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
        for (auto &space: memorySpaces) {
            auto cmsn = space.canonicalMsn;
            if (cmsn == Iris::SecureMonitorMsn ||
                    cmsn == Iris::GuestMsn ||
                    cmsn == Iris::NsHypMsn ||
                    cmsn == Iris::HypAppMsn) {
                bpSpaceIds.push_back(space.spaceId);
            }
        }
        panic_if(bpSpaceIds.empty(),
                "Unable to find address space(s) for breakpoints.");
    }
    return bpSpaceIds;
}

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
