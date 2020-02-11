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

#ifndef __ARCH_ARM_FASTMODEL_CORTEXA76_THREAD_CONTEXT_HH__
#define __ARCH_ARM_FASTMODEL_CORTEXA76_THREAD_CONTEXT_HH__

#include "arch/arm/fastmodel/iris/thread_context.hh"

namespace FastModel
{

// This ThreadContext class translates accesses to state using gem5's native
// to the Iris API. This includes extracting and translating register indices.
class CortexA76TC : public Iris::ThreadContext
{
  protected:
    static IdxNameMap miscRegIdxNameMap;
    static IdxNameMap intReg32IdxNameMap;
    static IdxNameMap intReg64IdxNameMap;
    static IdxNameMap flattenedIntIdxNameMap;
    static IdxNameMap ccRegIdxNameMap;
    static IdxNameMap vecRegIdxNameMap;
    static std::vector<iris::MemorySpaceId> bpSpaceIds;

  public:
    CortexA76TC(::BaseCPU *cpu, int id, System *system,
                ::BaseTLB *dtb, ::BaseTLB *itb,
                iris::IrisConnectionInterface *iris_if,
                const std::string &iris_path);

    bool translateAddress(Addr &paddr, Addr vaddr) override;

    void initFromIrisInstance(const ResourceMap &resources) override;

    RegVal readIntRegFlat(RegIndex idx) const override;
    void setIntRegFlat(RegIndex idx, RegVal val) override;

    RegVal readCCRegFlat(RegIndex idx) const override;
    void setCCRegFlat(RegIndex idx, RegVal val) override;

    const std::vector<iris::MemorySpaceId> &getBpSpaceIds() const override;
};

} // namespace FastModel

#endif // __ARCH_ARM_FASTMODEL_CORTEXA76_THREAD_CONTEXT_HH__
