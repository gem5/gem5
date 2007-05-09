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

#include "base/output.hh"
#include "cpu/base.hh"
#include "cpu/profile.hh"
#include "cpu/thread_state.hh"
#include "mem/port.hh"
#include "mem/translating_port.hh"
#include "sim/serialize.hh"

#if FULL_SYSTEM
#include "arch/kernel_stats.hh"
#include "cpu/quiesce_event.hh"
#include "mem/vport.hh"
#endif

#if FULL_SYSTEM
ThreadState::ThreadState(BaseCPU *cpu, int _cpuId, int _tid)
    : baseCpu(cpu), cpuId(_cpuId), tid(_tid), lastActivate(0), lastSuspend(0),
      profile(NULL), profileNode(NULL), profilePC(0), quiesceEvent(NULL),
      physPort(NULL), virtPort(NULL),
      microPC(0), nextMicroPC(1), funcExeInst(0), storeCondFailures(0)
#else
ThreadState::ThreadState(BaseCPU *cpu, int _cpuId, int _tid, Process *_process,
                         short _asid)
    : baseCpu(cpu), cpuId(_cpuId), tid(_tid), lastActivate(0), lastSuspend(0),
      port(NULL), process(_process), asid(_asid),
      microPC(0), nextMicroPC(1), funcExeInst(0), storeCondFailures(0)
#endif
{
    numInst = 0;
    numLoad = 0;
}

ThreadState::~ThreadState()
{
#if !FULL_SYSTEM
    if (port) {
        delete port->getPeer();
        delete port;
    }
#endif
}

void
ThreadState::serialize(std::ostream &os)
{
    SERIALIZE_ENUM(_status);
    // thread_num and cpu_id are deterministic from the config
    SERIALIZE_SCALAR(funcExeInst);
    SERIALIZE_SCALAR(inst);
    SERIALIZE_SCALAR(microPC);
    SERIALIZE_SCALAR(nextMicroPC);

#if FULL_SYSTEM
    Tick quiesceEndTick = 0;
    if (quiesceEvent->scheduled())
        quiesceEndTick = quiesceEvent->when();
    SERIALIZE_SCALAR(quiesceEndTick);
    if (kernelStats)
        kernelStats->serialize(os);
#endif
}

void
ThreadState::unserialize(Checkpoint *cp, const std::string &section)
{

    UNSERIALIZE_ENUM(_status);
    // thread_num and cpu_id are deterministic from the config
    UNSERIALIZE_SCALAR(funcExeInst);
    UNSERIALIZE_SCALAR(inst);
    UNSERIALIZE_SCALAR(microPC);
    UNSERIALIZE_SCALAR(nextMicroPC);

#if FULL_SYSTEM
    Tick quiesceEndTick;
    UNSERIALIZE_SCALAR(quiesceEndTick);
    if (quiesceEndTick)
        quiesceEvent->schedule(quiesceEndTick);
    if (kernelStats)
        kernelStats->unserialize(cp, section);
#endif
}

#if FULL_SYSTEM
void
ThreadState::connectMemPorts()
{
    connectPhysPort();
    connectVirtPort();
}

void
ThreadState::connectPhysPort()
{
    // @todo: For now this disregards any older port that may have
    // already existed.  Fix this memory leak once the bus port IDs
    // for functional ports is resolved.
    if (physPort)
        physPort->removeConn();
    else
        physPort = new FunctionalPort(csprintf("%s-%d-funcport",
                                           baseCpu->name(), tid));
    connectToMemFunc(physPort);
}

void
ThreadState::connectVirtPort()
{
    // @todo: For now this disregards any older port that may have
    // already existed.  Fix this memory leak once the bus port IDs
    // for functional ports is resolved.
    if (virtPort)
        virtPort->removeConn();
    else
        virtPort = new VirtualPort(csprintf("%s-%d-vport",
                                        baseCpu->name(), tid));
    connectToMemFunc(virtPort);
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

#else
TranslatingPort *
ThreadState::getMemPort()
{
    if (port != NULL)
        return port;

    /* Use this port to for syscall emulation writes to memory. */
    port = new TranslatingPort(csprintf("%s-%d-funcport", baseCpu->name(), tid),
                               process, TranslatingPort::NextPage);

    connectToMemFunc(port);

    return port;
}
#endif

void
ThreadState::connectToMemFunc(Port *port)
{
    Port *dcache_port, *func_mem_port;

    dcache_port = baseCpu->getPort("dcache_port");
    assert(dcache_port != NULL);

    MemObject *mem_object = dcache_port->getPeer()->getOwner();
    assert(mem_object != NULL);

    func_mem_port = mem_object->getPort("functional");
    assert(func_mem_port != NULL);

    func_mem_port->setPeer(port);
    port->setPeer(func_mem_port);
}
