/*
 * Copyright (c) 2023 The University of Wisconsin
 *
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

#include "mem/ruby/structures/ALUFreeListArray.hh"

#include "base/intmath.hh"
#include "mem/ruby/system/RubySystem.hh"
#include "sim/cur_tick.hh"

namespace gem5
{

namespace ruby
{

/*
 *
 * Models num_ALUs pipelined atomic ALUs with a depth of access_latency ticks.
 * Rather than reserving ALUs, this class assumes multiple requests can go
 * through an ALU at the same time. As such, up to numALU new requests can
 * go through at once, with the caveat that a line already being processed
 * in an ALU can't start processing again until the previous request has exited
 * the pipeline.
 *
 * ALUs aren't mapped directly to cache lines. Rather, ALUs are treated as
 * a free list.
 *
 * Behavior:
 *   Requests will go through unless one/both of the following are met:
 *       - There have been more than [numALUs] requests in the current cycle
 *       - The same line has been accessed in the past accessLatency ticks
 */

ALUFreeListArray::ALUFreeListArray(unsigned int num_ALUs, Tick access_latency)
{
    this->numALUs = num_ALUs;
    this->accessLatency = access_latency;
}

bool
ALUFreeListArray::tryAccess(Addr addr)
{
    uint32_t accesses_this_tick = 0;

    // Remove requests from the tail of the queue that occured more than
    // accessLatency ticks ago
    Tick oldestValidRecordStart = curTick() - this->accessLatency;

    while (accessQueue.size() > 0 &&
           (accessQueue.back().startTick < oldestValidRecordStart)) {
        accessQueue.pop_back();
    }

    for (AccessRecord &record : accessQueue) {
        // Block access if we would be using more ALUs than we have in a
        // single tick
        if (record.startTick == curTick() &&
            (++accesses_this_tick > numALUs)) {
            return false;
        }

        // Block access if the line is already being used
        if (record.lineAddr == makeLineAddress(addr)) {
            return false;
        }
    }

    return true;
}

void
ALUFreeListArray::reserve(Addr addr)
{
    // Only called after tryAccess, so we know queue is up to date and that
    // the access is valid

    // Add record to queue
    accessQueue.push_front(AccessRecord(makeLineAddress(addr), curTick()));
}

} // namespace ruby
} // namespace gem5
