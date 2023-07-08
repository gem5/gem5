/*
 * Copyright (c) 2013, 2018-2019 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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

#include "dev/arm/smmu_v3_proc.hh"

#include <cassert>
#include <functional>

#include "dev/arm/smmu_v3.hh"
#include "sim/system.hh"

namespace gem5
{

SMMUProcess::SMMUProcess(const std::string &name, SMMUv3 &_smmu) :
    coroutine(NULL),
    myName(name),
    smmu(_smmu)
{}

SMMUProcess::~SMMUProcess()
{
    delete coroutine;
}

void
SMMUProcess::wakeup()
{
    smmu.runProcess(this, NULL);
}

void
SMMUProcess::reinit()
{
    delete coroutine;
    coroutine = new Coroutine(
        std::bind(&SMMUProcess::main, this, std::placeholders::_1));
}

void
SMMUProcess::doRead(Yield &yield, Addr addr, void *ptr, size_t size)
{
    doSemaphoreDown(yield, smmu.requestPortSem);
    doDelay(yield, Cycles(1)); // request - assume 1 cycle
    doSemaphoreUp(smmu.requestPortSem);

    SMMUAction a;
    a.type = ACTION_SEND_REQ;

    RequestPtr req = std::make_shared<Request>(
        addr, size, 0, smmu.requestorId);

    req->taskId(context_switch_task_id::DMA);

    a.pkt = new Packet(req, MemCmd::ReadReq);
    a.pkt->dataStatic(ptr);

    a.delay = 0;

    PacketPtr pkt = yield(a).get();

    assert(pkt);
    // >= because we may get the whole cache line
    assert(pkt->getSize() >= size);

    delete pkt;
}

void
SMMUProcess::doWrite(Yield &yield, Addr addr, const void *ptr, size_t size)
{
    unsigned nbeats = (size + (smmu.requestPortWidth-1))
                            / smmu.requestPortWidth;

    doSemaphoreDown(yield, smmu.requestPortSem);
    doDelay(yield, Cycles(nbeats));
    doSemaphoreUp(smmu.requestPortSem);


    SMMUAction a;
    a.type = ACTION_SEND_REQ;

    RequestPtr req = std::make_shared<Request>(
        addr, size, 0, smmu.requestorId);

    req->taskId(context_switch_task_id::DMA);

    a.pkt = new Packet(req, MemCmd::WriteReq);
    a.pkt->dataStatic(ptr);

    PacketPtr pkt = yield(a).get();

    delete pkt;
}

void
SMMUProcess::doDelay(Yield &yield, Cycles cycles)
{
    if (smmu.system.isTimingMode())
        scheduleWakeup(smmu.clockEdge(cycles));

    SMMUAction a;
    a.type = ACTION_DELAY;
    a.delay = cycles * smmu.clockPeriod();
    yield(a);
}

void
SMMUProcess::doSleep(Yield &yield)
{
    SMMUAction a;
    a.type = ACTION_SLEEP;
    yield(a);
}

void
SMMUProcess::doSemaphoreDown(Yield &yield, SMMUSemaphore &sem)
{
    while (sem.count == 0) {
        sem.queue.push(this);
        doSleep(yield);
    }

    sem.count--;
    return;
}

void
SMMUProcess::doSemaphoreUp(SMMUSemaphore &sem)
{
    sem.count++;
    if (!sem.queue.empty()) {
        SMMUProcess *next_proc = sem.queue.front();
        sem.queue.pop();

        // Schedule event in the current tick instead of
        // calling the function directly to avoid overflowing
        // the stack in this coroutine.
        next_proc->scheduleWakeup(curTick());
    }
}

void
SMMUProcess::doWaitForSignal(Yield &yield, SMMUSignal &sig)
{
    sig.waiting.push_back(this);
    doSleep(yield);
}

void
SMMUProcess::doBroadcastSignal(SMMUSignal &sig)
{
    if (!sig.waiting.empty()) {
        for (auto it : sig.waiting) {
            // Schedule event in the current tick instead of
            // calling the function directly to avoid overflowing
            // the stack in this coroutine.
            it->scheduleWakeup(curTick());
        }

        sig.waiting.clear();
    }
}

void
SMMUProcess::scheduleWakeup(Tick when)
{
    auto *ep = new MemberEventWrapper<&SMMUProcess::wakeup> (*this, true);

    smmu.schedule(ep, when);
}

SMMUAction
SMMUProcess::run(PacketPtr pkt)
{
    assert(coroutine != NULL);
    assert(*coroutine);
    return (*coroutine)(pkt).get();
}

} // namespace gem5
