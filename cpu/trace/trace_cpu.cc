/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

/**
 * @file
 * Declaration of a memory trace CPU object. Uses a memory trace to drive the
 * provided memory hierarchy.
 */

#include <algorithm> // For min

#include "cpu/trace/trace_cpu.hh"
#include "cpu/trace/reader/mem_trace_reader.hh"
#include "mem/base_mem.hh" // For PARAM constructor
#include "mem/mem_interface.hh"
#include "sim/builder.hh"
#include "sim/sim_events.hh"

using namespace std;

TraceCPU::TraceCPU(const string &name,
                   MemInterface *icache_interface,
                   MemInterface *dcache_interface,
                   MemTraceReader *inst_trace,
                   MemTraceReader *data_trace,
                   int icache_ports,
                   int dcache_ports)
    : BaseCPU(name, 4), icacheInterface(icache_interface),
      dcacheInterface(dcache_interface), instTrace(inst_trace),
      dataTrace(data_trace), icachePorts(icache_ports),
      dcachePorts(dcache_ports), outstandingRequests(0), tickEvent(this)
{
    if (instTrace) {
        assert(icacheInterface);
        nextInstCycle = instTrace->getNextReq(nextInstReq);
    }
    if (dataTrace) {
        assert(dcacheInterface);
        nextDataCycle = dataTrace->getNextReq(nextDataReq);
    }
    tickEvent.schedule(0);
}

void
TraceCPU::tick()
{
    assert(outstandingRequests >= 0);
    assert(outstandingRequests < 1000);
    int instReqs = 0;
    int dataReqs = 0;

    // Do data first to match tracing with FullCPU dumps

    while (nextDataReq && (dataReqs < dcachePorts) &&
           curTick >= nextDataCycle) {
        assert(nextDataReq->thread_num < 4 && "Not enough threads");
        if (dcacheInterface->isBlocked())
            break;

        ++dataReqs;
        nextDataReq->time = curTick;
        nextDataReq->completionEvent =
            new TraceCompleteEvent(nextDataReq, this);
        dcacheInterface->access(nextDataReq);
        nextDataCycle = dataTrace->getNextReq(nextDataReq);
    }

    while (nextInstReq && (instReqs < icachePorts) &&
           curTick >= nextInstCycle) {
        assert(nextInstReq->thread_num < 4 && "Not enough threads");
        if (icacheInterface->isBlocked())
            break;

        nextInstReq->time = curTick;
        if (nextInstReq->cmd == Squash) {
            icacheInterface->squash(nextInstReq->asid);
        } else {
            ++instReqs;
            nextInstReq->completionEvent =
                new TraceCompleteEvent(nextInstReq, this);
            icacheInterface->access(nextInstReq);
        }
        nextInstCycle = instTrace->getNextReq(nextInstReq);
    }

    if (!nextInstReq && !nextDataReq) {
        // No more requests to send. Finish trailing events and exit.
        if (mainEventQueue.empty()) {
            new SimExitEvent("Finshed Memory Trace");
        } else {
            tickEvent.schedule(mainEventQueue.nextEventTime() + 1);
        }
    } else {
        tickEvent.schedule(max(curTick + 1,
                               min(nextInstCycle, nextDataCycle)));
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
TraceCompleteEvent::description()
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
TraceCPU::TickEvent::description()
{
    return "TraceCPU tick event";
}



BEGIN_DECLARE_SIM_OBJECT_PARAMS(TraceCPU)

    SimObjectParam<BaseMem *> icache;
    SimObjectParam<BaseMem *> dcache;
    SimObjectParam<MemTraceReader *> inst_trace;
    SimObjectParam<MemTraceReader *> data_trace;
    Param<int> inst_ports;
    Param<int> data_ports;

END_DECLARE_SIM_OBJECT_PARAMS(TraceCPU)

BEGIN_INIT_SIM_OBJECT_PARAMS(TraceCPU)

    INIT_PARAM_DFLT(icache, "instruction cache", NULL),
    INIT_PARAM_DFLT(dcache, "data cache", NULL),
    INIT_PARAM_DFLT(inst_trace, "instruction trace", NULL),
    INIT_PARAM_DFLT(data_trace, "data trace", NULL),
    INIT_PARAM_DFLT(inst_ports, "instruction cache read ports", 4),
    INIT_PARAM_DFLT(data_ports, "data cache read/write ports", 4)

END_INIT_SIM_OBJECT_PARAMS(TraceCPU)

CREATE_SIM_OBJECT(TraceCPU)
{
    return new TraceCPU(getInstanceName(),
                        (icache) ? icache->getInterface() : NULL,
                        (dcache) ? dcache->getInterface() : NULL,
                        inst_trace, data_trace, inst_ports, data_ports);
}

REGISTER_SIM_OBJECT("TraceCPU", TraceCPU)

