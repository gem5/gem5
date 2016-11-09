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
 *
 * Authors: Kevin Lim
 */

#include "cpu/thread_state.hh"

#include "arch/kernel_stats.hh"
#include "base/output.hh"
#include "cpu/base.hh"
#include "cpu/profile.hh"
#include "cpu/quiesce_event.hh"
#include "mem/fs_translating_port_proxy.hh"
#include "mem/port.hh"
#include "mem/port_proxy.hh"
#include "mem/se_translating_port_proxy.hh"
#include "sim/full_system.hh"
#include "sim/serialize.hh"
#include "sim/system.hh"

ThreadState::ThreadState(BaseCPU *cpu, ThreadID _tid, Process *_process)
    : numInst(0), numOp(0), numLoad(0), startNumLoad(0),
      _status(ThreadContext::Halted), baseCpu(cpu),
      _contextId(0), _threadId(_tid), lastActivate(0), lastSuspend(0),
      profile(NULL), profileNode(NULL), profilePC(0), quiesceEvent(NULL),
      kernelStats(NULL), process(_process), physProxy(NULL), virtProxy(NULL),
      proxy(NULL), funcExeInst(0), storeCondFailures(0)
{
}

ThreadState::~ThreadState()
{
    if (physProxy != NULL)
        delete physProxy;
    if (virtProxy != NULL)
        delete virtProxy;
    if (proxy != NULL)
        delete proxy;
}

void
ThreadState::serialize(CheckpointOut &cp) const
{
    SERIALIZE_ENUM(_status);
    // thread_num and cpu_id are deterministic from the config
    SERIALIZE_SCALAR(funcExeInst);

    if (!FullSystem)
        return;

    Tick quiesceEndTick = 0;
    if (quiesceEvent->scheduled())
        quiesceEndTick = quiesceEvent->when();
    SERIALIZE_SCALAR(quiesceEndTick);
    if (kernelStats)
        kernelStats->serialize(cp);
}

void
ThreadState::unserialize(CheckpointIn &cp)
{

    UNSERIALIZE_ENUM(_status);
    // thread_num and cpu_id are deterministic from the config
    UNSERIALIZE_SCALAR(funcExeInst);

    if (!FullSystem)
        return;

    Tick quiesceEndTick;
    UNSERIALIZE_SCALAR(quiesceEndTick);
    if (quiesceEndTick)
        baseCpu->schedule(quiesceEvent, quiesceEndTick);
    if (kernelStats)
        kernelStats->unserialize(cp);
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
        // getDataPort is a virtual function
        physProxy = new PortProxy(baseCpu->getDataPort(),
                                  baseCpu->cacheLineSize());

        assert(virtProxy == NULL);
        virtProxy = new FSTranslatingPortProxy(tc);
    } else {
        assert(proxy == NULL);
        proxy = new SETranslatingPortProxy(baseCpu->getDataPort(),
                                           process,
                                           SETranslatingPortProxy::NextPage);
    }
}

PortProxy &
ThreadState::getPhysProxy()
{
    assert(FullSystem);
    assert(physProxy != NULL);
    return *physProxy;
}

FSTranslatingPortProxy &
ThreadState::getVirtProxy()
{
    assert(FullSystem);
    assert(virtProxy != NULL);
    return *virtProxy;
}

SETranslatingPortProxy &
ThreadState::getMemProxy()
{
    assert(!FullSystem);
    assert(proxy != NULL);
    return *proxy;
}

void
ThreadState::profileClear()
{
    if (profile)
        profile->clear();
}

void
ThreadState::profileSample()
{
    if (profile)
        profile->sample(profileNode, profilePC);
}
