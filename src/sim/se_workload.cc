/*
 * Copyright 2020 Google Inc.
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

#include "sim/se_workload.hh"

#include "cpu/thread_context.hh"
#include "params/SEWorkload.hh"
#include "sim/process.hh"
#include "sim/system.hh"

namespace gem5
{

SEWorkload::SEWorkload(const Params &p, Addr page_shift) :
    Workload(p), memPools(page_shift)
{}

void
SEWorkload::setSystem(System *sys)
{
    Workload::setSystem(sys);

    AddrRangeList memories = sys->getPhysMem().getConfAddrRanges();
    const auto &m5op_range = sys->m5opRange();

    if (m5op_range.valid())
        memories -= m5op_range;

    memPools.populate(memories);
}

void
SEWorkload::serialize(CheckpointOut &cp) const
{
    memPools.serialize(cp);
}

void
SEWorkload::unserialize(CheckpointIn &cp)
{
    memPools.unserialize(cp);
}

void
SEWorkload::syscall(ThreadContext *tc)
{
    tc->getProcessPtr()->syscall(tc);
}

Addr
SEWorkload::allocPhysPages(int npages, int pool_id)
{
    return memPools.allocPhysPages(npages, pool_id);
}

Addr
SEWorkload::memSize(int pool_id) const
{
    return memPools.memSize(pool_id);
}

Addr
SEWorkload::freeMemSize(int pool_id) const
{
    return memPools.freeMemSize(pool_id);
}

} // namespace gem5
