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

ThreadState::ThreadState(BaseCPU *cpu, ThreadID _tid, Process *_process)
    : numInst(0), numOp(0), threadStats(cpu, _tid),
      numLoad(0), startNumLoad(0),
      _status(ThreadContext::Halted), baseCpu(cpu),
      _contextId(0), _threadId(_tid), lastActivate(0), lastSuspend(0),
      process(_process), physProxy(NULL), virtProxy(NULL),
      funcExeInst(0), storeCondFailures(0)
{
}

ThreadState::~ThreadState()
{
    if (physProxy != NULL)
        delete physProxy;
    if (virtProxy != NULL)
        delete virtProxy;
}

void
ThreadState::serialize(CheckpointOut &cp) const
{
    SERIALIZE_ENUM(_status);
    // thread_num and cpu_id are deterministic from the config
    SERIALIZE_SCALAR(funcExeInst);

    if (!FullSystem)
        return;
}

void
ThreadState::unserialize(CheckpointIn &cp)
{

    UNSERIALIZE_ENUM(_status);
    // thread_num and cpu_id are deterministic from the config
    UNSERIALIZE_SCALAR(funcExeInst);

    if (!FullSystem)
        return;
}

void
ThreadState::initMemProxies(ThreadContext *tc)
{
    // The port proxies only refer to the data port on the CPU side
    // and can safely be done at init() time even if the CPU is not
    // connected, i.e. when restoring from a checkpoint and later
    // switching the CPU in.
    if (FullSystem) {
        assert(physProxy == NULL);
        // This cannot be done in the constructor as the thread state
        // itself is created in the base cpu constructor and the
        // getSendFunctional is a virtual function
        physProxy = new PortProxy(baseCpu->getSendFunctional(),
                                  baseCpu->cacheLineSize());

        assert(virtProxy == NULL);
        virtProxy = new TranslatingPortProxy(tc);
    } else {
        assert(virtProxy == NULL);
        virtProxy = new SETranslatingPortProxy(
                tc, SETranslatingPortProxy::NextPage);
    }
}

PortProxy &
ThreadState::getPhysProxy()
{
    assert(FullSystem);
    assert(physProxy != NULL);
    return *physProxy;
}

PortProxy &
ThreadState::getVirtProxy()
{
    assert(virtProxy != NULL);
    return *virtProxy;
}

ThreadState::ThreadStateStats::ThreadStateStats(BaseCPU *cpu,
                                                const ThreadID& tid)
      : Stats::Group(cpu, csprintf("thread_%i", tid).c_str()),
      ADD_STAT(numInsts, "Number of Instructions committed"),
      ADD_STAT(numOps, "Number of Ops committed"),
      ADD_STAT(numMemRefs, "Number of Memory References")
{

}
