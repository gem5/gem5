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

#include "sim/ticked_object.hh"

#include "params/TickedObject.hh"
#include "sim/clocked_object.hh"
#include "sim/serialize.hh"

namespace gem5
{

Ticked::Ticked(ClockedObject &object_,
    statistics::Scalar *imported_num_cycles,
    Event::Priority priority) :
    object(object_),
    event([this]{ processClockEvent(); }, object_.name(), false, priority),
    running(false),
    lastStopped(0),
    /* Allocate numCycles if an external stat wasn't passed in */
    numCyclesLocal((imported_num_cycles ? NULL : new statistics::Scalar)),
    numCycles((imported_num_cycles ? *imported_num_cycles :
        *numCyclesLocal))
{ }

void
Ticked::processClockEvent() {
    ++tickCycles;
    ++numCycles;
    countCycles(Cycles(1));
    evaluate();
    if (running)
        object.schedule(event, object.clockEdge(Cycles(1)));
}

void
Ticked::regStats()
{
    if (numCyclesLocal) {
        numCycles
            .name(object.name() + ".totalTickCycles")
            .desc("Number of cycles that the object ticked or was stopped");
    }

    tickCycles
        .name(object.name() + ".tickCycles")
        .desc("Number of cycles that the object actually ticked");

    idleCycles
        .name(object.name() + ".idleCycles")
        .desc("Total number of cycles that the object has spent stopped");
    idleCycles = numCycles - tickCycles;
}

void
Ticked::serialize(CheckpointOut &cp) const
{
    uint64_t lastStoppedUint = lastStopped;

    paramOut(cp, "lastStopped", lastStoppedUint);
}

void
Ticked::unserialize(CheckpointIn &cp)
{
    uint64_t lastStoppedUint = 0;

    /* lastStopped is optional on checkpoint restore as this object may be
     *  being restored from one which has a common base (and so possibly
     *  many common checkpointed values) but where Ticked is used in the
     *  checkpointed object but not this one.
     *  An example would be a CPU model using Ticked restores from a
     *  simple CPU without without Ticked */
    optParamIn(cp, "lastStopped", lastStoppedUint);

    lastStopped = Cycles(lastStoppedUint);
}

TickedObject::TickedObject(const TickedObjectParams &params,
    Event::Priority priority) :
    ClockedObject(params),
    /* Make numCycles in Ticked */
    Ticked(*this, NULL, priority)
{ }

void
TickedObject::regStats()
{
    Ticked::regStats();
    ClockedObject::regStats();
}

void
TickedObject::serialize(CheckpointOut &cp) const
{
    Ticked::serialize(cp);
    ClockedObject::serialize(cp);
}
void
TickedObject::unserialize(CheckpointIn &cp)
{
    Ticked::unserialize(cp);
    ClockedObject::unserialize(cp);
}

} // namespace gem5
