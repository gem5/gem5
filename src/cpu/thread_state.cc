/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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

#include "cpu/thread_state.hh"

#include "base/output.hh"
#include "cpu/base.hh"
#include "mem/port.hh"
#include "mem/port_proxy.hh"
#include "mem/se_translating_port_proxy.hh"
#include "mem/translating_port_proxy.hh"
#include "sim/full_system.hh"
#include "sim/serialize.hh"
#include "sim/system.hh"

namespace gem5
{

ThreadState::ThreadState(BaseCPU *cpu, ThreadID _tid, Process *_process)
    : numInst(0),
      numOp(0),
      threadStats(cpu, _tid),
      numLoad(0),
      startNumLoad(0),
      _status(ThreadContext::Halted),
      baseCpu(cpu),
      _contextId(0),
      _threadId(_tid),
      lastActivate(0),
      lastSuspend(0),
      process(_process),
      storeCondFailures(0)
{}

void
ThreadState::serialize(CheckpointOut &cp) const
{
    SERIALIZE_ENUM(_status);
}

void
ThreadState::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_ENUM(_status);
}

ThreadState::ThreadStateStats::ThreadStateStats(BaseCPU *cpu,
                                                const ThreadID &tid)
    : statistics::Group(cpu, csprintf("thread_%i", tid).c_str()),
      ADD_STAT(numInsts, statistics::units::Count::get(),
               "Number of Instructions committed"),
      ADD_STAT(numOps, statistics::units::Count::get(),
               "Number of Ops committed"),
      ADD_STAT(numMemRefs, statistics::units::Count::get(),
               "Number of Memory References")
{}

} // namespace gem5
