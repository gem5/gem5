/*
 * Copyright (c) 2013-2014, 2017 ARM Limited
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

/**
 * @file
 *
 *  Base classes for ClockedObjects which have evaluate functions to
 *  look like clock ticking operations.  TickedObject attaches gem5's event
 *  queue to Ticked to apply actual scheduling.
 */

#ifndef __SIM_TICKED_OBJECT_HH__
#define __SIM_TICKED_OBJECT_HH__

#include "sim/clocked_object.hh"

class TickedObjectParams;

/** Ticked attaches gem5's event queue/scheduler to evaluate
 *  calls and provides a start/stop interface to ticking.
 *
 *  Ticked is not a ClockedObject but can be attached to one by
 *  inheritance and by calling regStats, serialize/unserialize */
class Ticked : public Serializable
{
  protected:
    /** ClockedObject who is responsible for this Ticked's actions/stats */
    ClockedObject &object;

    /** The wrapper for processClockEvent */
    EventFunctionWrapper event;

    /** Evaluate and reschedule */
    void processClockEvent();

    /** Have I been started? and am not stopped */
    bool running;

    /** Time of last stop event to calculate run time */
    Cycles lastStopped;

  private:
    /** Locally allocated stats */
    Stats::Scalar *numCyclesLocal;

  protected:
    /** Total number of cycles either ticked or spend stopped */
    Stats::Scalar &numCycles;

    /** Number of cycles ticked */
    Stats::Scalar tickCycles;

    /** Number of cycles stopped */
    Stats::Formula idleCycles;

  public:
    Ticked(ClockedObject &object_,
        Stats::Scalar *imported_num_cycles = NULL,
        Event::Priority priority = Event::CPU_Tick_Pri);

    virtual ~Ticked() { }

    /** Register {num,ticks}Cycles if necessary.  If numCycles is
     *  imported, be sure to register it *before* calling this regStats */
    void regStats();

    /** Start ticking */
    void
    start()
    {
        if (!running) {
            if (!event.scheduled())
                object.schedule(event, object.clockEdge(Cycles(1)));
            running = true;
            numCycles += cyclesSinceLastStopped();
            countCycles(cyclesSinceLastStopped());
        }
    }

    /** How long have we been stopped for? */
    Cycles
    cyclesSinceLastStopped() const
    {
        return object.curCycle() - lastStopped;
    }

    /** Reset stopped time to current time */
    void
    resetLastStopped()
    {
        lastStopped = object.curCycle();
    }

    /** Cancel the next tick event and issue no more */
    void
    stop()
    {
        if (running) {
            if (event.scheduled())
                object.deschedule(event);
            running = false;
            resetLastStopped();
        }
    }

    /** Checkpoint lastStopped */
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    /** Action to call on the clock tick */
    virtual void evaluate() = 0;

    /**
     * Callback to handle cycle statistics and probes.
     *
     * This callback is called at the beginning of a new cycle active
     * cycle and when restarting the ticked object. The delta
     * parameter indicates the number of cycles elapsed since the
     * previous call is normally '1' unless the object has been
     * stopped and restarted.
     *
     * @param delta Number of cycles since the previous call.
     */
    virtual void countCycles(Cycles delta) {}
};

/** TickedObject attaches Ticked to ClockedObject and can be used as
 *  a base class where ticked operation */
class TickedObject : public ClockedObject, public Ticked
{
  public:
    TickedObject(const TickedObjectParams *params,
        Event::Priority priority = Event::CPU_Tick_Pri);

    /** Disambiguate to make these functions overload correctly */
    using ClockedObject::regStats;
    using ClockedObject::serialize;
    using ClockedObject::unserialize;

    /** Pass on regStats, serialize etc. onto Ticked */
    void regStats() override;
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

#endif /* __SIM_TICKED_OBJECT_HH__ */
