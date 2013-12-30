/*
 * Copyright (c) 2012-2013 ARM Limited
 * Copyright (c) 2013 Cornell University
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
 *
 * Authors: Andreas Hansson
 *          Christopher Torng
 */

/**
 * @file
 * ClockedObject declaration and implementation.
 */

#ifndef __SIM_CLOCKED_OBJECT_HH__
#define __SIM_CLOCKED_OBJECT_HH__

#include "base/intmath.hh"
#include "base/misc.hh"
#include "params/ClockedObject.hh"
#include "sim/core.hh"
#include "sim/clock_domain.hh"
#include "sim/sim_object.hh"

/**
 * The ClockedObject class extends the SimObject with a clock and
 * accessor functions to relate ticks to the cycles of the object.
 */
class ClockedObject : public SimObject
{

  private:

    // the tick value of the next clock edge (>= curTick()) at the
    // time of the last call to update()
    mutable Tick tick;

    // The cycle counter value corresponding to the current value of
    // 'tick'
    mutable Cycles cycle;

    /**
     * Prevent inadvertent use of the copy constructor and assignment
     * operator by making them private.
     */
    ClockedObject(ClockedObject&);
    ClockedObject& operator=(ClockedObject&);

    /**
     *  Align cycle and tick to the next clock edge if not already done.
     */
    void update() const
    {
        // both tick and cycle are up-to-date and we are done, note
        // that the >= is important as it captures cases where tick
        // has already passed curTick()
        if (tick >= curTick())
            return;

        // optimise for the common case and see if the tick should be
        // advanced by a single clock period
        tick += clockPeriod();
        ++cycle;

        // see if we are done at this point
        if (tick >= curTick())
            return;

        // if not, we have to recalculate the cycle and tick, we
        // perform the calculations in terms of relative cycles to
        // allow changes to the clock period in the future
        Cycles elapsedCycles(divCeil(curTick() - tick, clockPeriod()));
        cycle += elapsedCycles;
        tick += elapsedCycles * clockPeriod();
    }

    /**
     * The clock domain this clocked object belongs to
     */
    ClockDomain &clockDomain;

  protected:

    /**
     * Create a clocked object and set the clock domain based on the
     * parameters.
     */
    ClockedObject(const ClockedObjectParams* p) :
        SimObject(p), tick(0), cycle(0), clockDomain(*p->clk_domain)
    {
        // Register with the clock domain, so that if the clock domain
        // frequency changes, we can update this object's tick.
        clockDomain.registerWithClockDomain(this);
    }

    /**
     * Virtual destructor due to inheritance.
     */
    virtual ~ClockedObject() { }

    /**
     * Reset the object's clock using the current global tick value. Likely
     * to be used only when the global clock is reset. Currently, this done
     * only when Ruby is done warming up the memory system.
     */
    void resetClock() const
    {
        Cycles elapsedCycles(divCeil(curTick(), clockPeriod()));
        cycle = elapsedCycles;
        tick = elapsedCycles * clockPeriod();
    }

  public:

    /**
     * Update the tick to the current tick.
     *
     */
    inline void updateClockPeriod() const
    {
        update();
    }

    /**
     * Determine the tick when a cycle begins, by default the current
     * one, but the argument also enables the caller to determine a
     * future cycle.
     *
     * @param cycles The number of cycles into the future
     *
     * @return The tick when the clock edge occurs
     */
    inline Tick clockEdge(Cycles cycles = Cycles(0)) const
    {
        // align tick to the next clock edge
        update();

        // figure out when this future cycle is
        return tick + clockPeriod() * cycles;
    }

    /**
     * Determine the current cycle, corresponding to a tick aligned to
     * a clock edge.
     *
     * @return The current cycle count
     */
    inline Cycles curCycle() const
    {
        // align cycle to the next clock edge.
        update();

        return cycle;
    }

    /**
     * Based on the clock of the object, determine the tick when the next
     * cycle begins, in other words, return the next clock edge.
     * (This can never be the current tick.)
     *
     * @return The tick when the next cycle starts
     */
    Tick nextCycle() const
    { return clockEdge(Cycles(1)); }

    inline uint64_t frequency() const
    {
        return SimClock::Frequency / clockPeriod();
    }

    inline Tick clockPeriod() const
    {
        return clockDomain.clockPeriod();
    }

    inline Cycles ticksToCycles(Tick t) const
    { return Cycles(divCeil(t, clockPeriod())); }

};

#endif //__SIM_CLOCKED_OBJECT_HH__
