/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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
 * Authors: Erik Hallnor
 */

/**
 * @file
 * Declaration of a memory trace CPU object. Uses a memory trace to drive the
 * provided memory hierarchy.
 */

#include <algorithm> // For min

#include "cpu/trace/reader/mem_trace_reader.hh"
#include "cpu/trace/trace_cpu.hh"
#include "mem/base_mem.hh" // For PARAM constructor
#include "mem/mem_interface.hh"
#include "params/TraceCPU.hh"
#include "sim/sim_events.hh"

using namespace std;

TraceCPU::TraceCPU(const string &name,
                   MemInterface *icache_interface,
                   MemInterface *dcache_interface,
                   MemTraceReader *data_trace)
    : SimObject(name), icacheInterface(icache_interface),
      dcacheInterface(dcache_interface),
      dataTrace(data_trace), outstandingRequests(0), tickEvent(this)
{
    assert(dcacheInterface);
    nextCycle = dataTrace->getNextReq(nextReq);
    tickEvent.schedule(0);
}

void
TraceCPU::tick()
{
    assert(outstandingRequests >= 0);
    assert(outstandingRequests < 1000);
    int instReqs = 0;
    int dataReqs = 0;

    while (nextReq && curTick() >= nextCycle) {
        assert(nextReq->thread_num < 4 && "Not enough threads");
        if (nextReq->isInstFetch() && icacheInterface) {
            if (icacheInterface->isBlocked())
                break;

            nextReq->time = curTick();
            if (nextReq->cmd == Squash) {
                icacheInterface->squash(nextReq->asid);
            } else {
                ++instReqs;
                if (icacheInterface->doEvents()) {
                    nextReq->completionEvent =
                        new TraceCompleteEvent(nextReq, this);
                    icacheInterface->access(nextReq);
                } else {
                    icacheInterface->access(nextReq);
                    completeRequest(nextReq);
                }
            }
        } else {
            if (dcacheInterface->isBlocked())
                break;

            ++dataReqs;
            nextReq->time = curTick();
            if (dcacheInterface->doEvents()) {
                nextReq->completionEvent =
                    new TraceCompleteEvent(nextReq, this);
                dcacheInterface->access(nextReq);
            } else {
                dcacheInterface->access(nextReq);
                completeRequest(nextReq);
            }

        }
        nextCycle = dataTrace->getNextReq(nextReq);
    }

    if (!nextReq) {
        // No more requests to send. Finish trailing events and exit.
        if (mainEventQueue.empty()) {
            exitSimLoop("end of memory trace reached");
        } else {
            tickEvent.schedule(mainEventQueue.nextEventTime() + ticks(1));
        }
    } else {
        tickEvent.schedule(max(curTick() + ticks(1), nextCycle));
    }
}

void
TraceCPU::completeRequest(MemReqPtr& req)
{
}

void
TraceCompleteEvent::process()
{
    tester->completeRequest(req);
}

const char *
TraceCompleteEvent::description() const
{
    return "trace access complete";
}

TraceCPU::TickEvent::TickEvent(TraceCPU *c)
    : Event(&mainEventQueue, CPU_Tick_Pri), cpu(c)
{
}

void
TraceCPU::TickEvent::process()
{
    cpu->tick();
}

const char *
TraceCPU::TickEvent::description() const
{
    return "TraceCPU tick";
}

TraceCPU *
TraceCPUParams::create()
{
    return new TraceCPU(name,
                        (icache) ? icache->getInterface() : NULL,
                        (dcache) ? dcache->getInterface() : NULL,
                        data_trace);
}
