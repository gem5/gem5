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

#include "arch/arm/fastmodel/iris/cpu.hh"

#include "arch/arm/fastmodel/iris/thread_context.hh"
#include "scx/scx.h"
#include "sim/serialize.hh"

namespace gem5
{

namespace Iris
{

BaseCPU::BaseCPU(const BaseCPUParams &params, sc_core::sc_module *_evs)
    : gem5::BaseCPU::BaseCPU(params),
      evs(_evs),
      evs_base_cpu(dynamic_cast<Iris::BaseCpuEvs *>(_evs))
{
    panic_if(!evs_base_cpu, "EVS should be of type BaseCpuEvs");

    // Make sure fast model knows we're using debugging mechanisms to control
    // the simulation, and it shouldn't shut down if simulation time stops
    // for some reason. Despite the misleading name, this doesn't start a CADI
    // server because it's first parameter is false.
    scx::scx_start_cadi_server(false, false, true);
}

BaseCPU::~BaseCPU()
{
    for (auto &tc : threadContexts)
        delete tc;
    threadContexts.clear();
}

Counter
BaseCPU::totalInsts() const
{
    Counter count = 0;
    for (auto *tc : threadContexts)
        count += tc->getCurrentInstCount();
    return count;
}

void
BaseCPU::serializeThread(CheckpointOut &cp, ThreadID tid) const
{
    gem5::serialize(*threadContexts[tid], cp);
}

} // namespace Iris
} // namespace gem5
