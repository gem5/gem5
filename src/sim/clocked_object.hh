/*
 * Copyright (c) 2012 ARM Limited
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
 */

/**
 * @file
 * ClockedObject declaration and implementation.
 */

#ifndef __SIM_CLOCKED_OBJECT_HH__
#define __SIM_CLOCKED_OBJECT_HH__

#include "base/intmath.hh"
#include "params/ClockedObject.hh"
#include "sim/sim_object.hh"

/**
 * The ClockedObject class extends the SimObject with a clock and
 * accessor functions to relate ticks to the cycles of the object.
 */
class ClockedObject : public SimObject
{

  private:

    /**
     * Prevent inadvertent use of the copy constructor and assignment
     * operator by making them private.
     */
    ClockedObject(ClockedObject&);
    ClockedObject& operator=(ClockedObject&);

  protected:

    // Clock period in ticks
    Tick clock;

    /**
     * Create a clocked object and set the clock based on the
     * parameters.
     */
    ClockedObject(const ClockedObjectParams* p) : SimObject(p), clock(p->clock)
    { }

    /**
     * Virtual destructor due to inheritance.
     */
    virtual ~ClockedObject() { }

  public:

    /**
     * Based on the clock of the object, determine the tick when the
     * next cycle begins, in other words, round the curTick() to the
     * next tick that is a multiple of the clock.
     *
     * @return The tick when the next cycle starts
     */
    Tick nextCycle() const
    { return divCeil(curTick(), clock) * clock; }

    /**
     * Determine the next cycle starting from a given tick instead of
     * curTick().
     *
     * @param begin_tick The tick to round to a clock edge
     *
     * @return The tick when the cycle after or on begin_tick starts
     */
    Tick nextCycle(Tick begin_tick) const
    { return divCeil(begin_tick, clock) * clock; }

    inline Tick frequency() const { return SimClock::Frequency / clock; }

    inline Tick ticks(int numCycles) const { return clock * numCycles; }

    inline Tick curCycle() const { return curTick() / clock; }

    inline Tick tickToCycles(Tick val) const { return val / clock; }

};

#endif //__SIM_CLOCKED_OBJECT_HH__
