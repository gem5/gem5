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

#ifndef __ARCH_ARM_FASTMODEL_CORTEXR52_THREAD_CONTEXT_HH__
#define __ARCH_ARM_FASTMODEL_CORTEXR52_THREAD_CONTEXT_HH__

#include "arch/arm/fastmodel/iris/thread_context.hh"

namespace gem5
{

namespace fastmodel
{

// This ThreadContext class translates accesses to state using gem5's native
// to the Iris API. This includes extracting and translating register indices.
class CortexR52TC : public Iris::ThreadContext
{
  protected:
    static IdxNameMap miscRegIdxNameMap;
    static IdxNameMap intReg32IdxNameMap;
    static IdxNameMap ccRegIdxNameMap;
    static std::vector<iris::MemorySpaceId> bpSpaceIds;

  public:
    CortexR52TC(gem5::BaseCPU *cpu, int id, System *system, gem5::BaseMMU *mmu,
                gem5::BaseISA *isa, iris::IrisConnectionInterface *iris_if,
                const std::string &iris_path);

    bool translateAddress(Addr &paddr, Addr vaddr) override;

    void initFromIrisInstance(const ResourceMap &resources) override;
    void sendFunctional(PacketPtr pkt) override;

    // Since this CPU doesn't support aarch64, we override these two methods
    // and always assume we're 32 bit. More than likely we could be more
    // general than that, but that would require letting the default
    // implementation read the CPSR, and that's not currently implemented.
    RegVal readIntReg(RegIndex reg_idx) const override;
    void setIntReg(RegIndex reg_idx, RegVal val) override;

    RegVal readCCRegFlat(RegIndex idx) const override;
    void setCCRegFlat(RegIndex idx, RegVal val) override;

    const std::vector<iris::MemorySpaceId> &getBpSpaceIds() const override;

    // The map from gem5 indexes to IRIS resource names is not currently set
    // up. It will be a little more complicated for R52, since it won't have
    // many of the registers since it doesn't support aarch64. We may need to
    // just return dummy values on reads and throw away writes, throw an
    // error, or some combination of the two.
    RegVal
    readMiscRegNoEffect(RegIndex idx) const override
    {
        panic_if(miscRegIdxNameMap.find(idx) == miscRegIdxNameMap.end(),
                 "No mapping for index %#x.", idx);
        return Iris::ThreadContext::readMiscRegNoEffect(idx);
    }

    void
    setMiscRegNoEffect(RegIndex idx, const RegVal val) override
    {
        panic_if(miscRegIdxNameMap.find(idx) == miscRegIdxNameMap.end(),
                 "No mapping for index %#x.", idx);
        Iris::ThreadContext::setMiscRegNoEffect(idx, val);
    }

    // Like the Misc regs, not currently supported and a little complicated.
    RegVal
    readIntRegFlat(RegIndex idx) const override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    void
    setIntRegFlat(RegIndex idx, RegVal val) override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    // Not supported by the CPU. There isn't anything to set up here as far
    // as mapping, but the question still remains what to do about registers
    // that don't exist in the CPU.
    const ArmISA::VecRegContainer &
    readVecReg(const RegId &) const override
    {
        panic("%s not implemented.", __FUNCTION__);
    }
};

} // namespace fastmodel
} // namespace gem5

#endif // __ARCH_ARM_FASTMODEL_CORTEXR52_THREAD_CONTEXT_HH__
