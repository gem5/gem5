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

#ifndef __CPU_TRACE_TRACE_CPU_HH__
#define __CPU_TRACE_TRACE_CPU_HH__

#include <string>

#include "mem/mem_req.hh" // for MemReqPtr
#include "sim/eventq.hh" // for Event
#include "sim/sim_object.hh"

// Forward declaration.
class MemInterface;
class MemTraceReader;

/**
 * A cpu object for running memory traces through a memory hierarchy.
 */
class TraceCPU : public SimObject
{
  private:
    /** Interface for instruction trace requests, if any. */
    MemInterface *icacheInterface;
    /** Interface for data trace requests, if any. */
    MemInterface *dcacheInterface;

    /** Data reference trace. */
    MemTraceReader *dataTrace;

    /** Number of outstanding requests. */
    int outstandingRequests;

    /** Cycle of the next request, 0 if not available. */
    Tick nextCycle;

    /** Next request. */
    MemReqPtr nextReq;

    /**
     * Event to call the TraceCPU::tick
     */
    class TickEvent : public Event
    {
      private:
        /** The associated CPU */
        TraceCPU *cpu;

      public:
        /**
         * Construct this event;
         */
        TickEvent(TraceCPU *c);

        /**
         * Call the tick function.
         */
        void process();

        /**
         * Return a string description of this event.
         */
        const char *description() const;
    };

    TickEvent tickEvent;

  public:
    /**
     * Construct a TraceCPU object.
     */
    TraceCPU(const std::string &name,
             MemInterface *icache_interface,
             MemInterface *dcache_interface,
             MemTraceReader *data_trace);

    inline Tick ticks(int numCycles) { return numCycles; }

    /**
     * Perform all the accesses for one cycle.
     */
    void tick();

    /**
     * Handle a completed memory request.
     */
    void completeRequest(MemReqPtr &req);
};

class TraceCompleteEvent : public Event
{
    MemReqPtr req;
    TraceCPU *tester;

  public:

    TraceCompleteEvent(MemReqPtr &_req, TraceCPU *_tester)
        : Event(&mainEventQueue), req(_req), tester(_tester)
    {
        setFlags(AutoDelete);
    }

    void process();

    virtual const char *description() const;
};

#endif // __CPU_TRACE_TRACE_CPU_HH__

