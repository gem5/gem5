/*
 * Copyright (c) 2012-2013, 2016-2017 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed here under.  You may use the software subject to the license
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

#include "cpu/testers/traffic_gen/trace_gen.hh"

#include <algorithm>

#include "base/random.hh"
#include "base/trace.hh"
#include "debug/TrafficGen.hh"
#include "proto/packet.pb.h"

TraceGen::InputStream::InputStream(const std::string& filename)
    : trace(filename)
{
    init();
}

void
TraceGen::InputStream::init()
{
    // Create a protobuf message for the header and read it from the stream
    ProtoMessage::PacketHeader header_msg;
    if (!trace.read(header_msg)) {
        panic("Failed to read packet header from trace\n");
    } else if (header_msg.tick_freq() != SimClock::Frequency) {
        panic("Trace was recorded with a different tick frequency %d\n",
              header_msg.tick_freq());
    }
}

void
TraceGen::InputStream::reset()
{
    trace.reset();
    init();
}

bool
TraceGen::InputStream::read(TraceElement& element)
{
    ProtoMessage::Packet pkt_msg;
    if (trace.read(pkt_msg)) {
        element.cmd = pkt_msg.cmd();
        element.addr = pkt_msg.addr();
        element.blocksize = pkt_msg.size();
        element.tick = pkt_msg.tick();
        element.flags = pkt_msg.has_flags() ? pkt_msg.flags() : 0;
        return true;
    }

    // We have reached the end of the file
    return false;
}

Tick
TraceGen::nextPacketTick(bool elastic, Tick delay) const
{
    if (traceComplete) {
        DPRINTF(TrafficGen, "No next tick as trace is finished\n");
        // We are at the end of the file, thus we have no more data in
        // the trace Return MaxTick to signal that there will be no
        // more transactions in this active period for the state.
        return MaxTick;
    }

    assert(nextElement.isValid());

    DPRINTF(TrafficGen, "Next packet tick is %d\n", tickOffset +
            nextElement.tick);

    // if the playback is supposed to be elastic, add the delay
    if (elastic)
        tickOffset += delay;

    return std::max(tickOffset + nextElement.tick, curTick());
}

void
TraceGen::enter()
{
    // update the trace offset to the time where the state was entered.
    tickOffset = curTick();

    // clear everything
    currElement.clear();

    // read the first element in the file and set the complete flag
    traceComplete = !trace.read(nextElement);
}

PacketPtr
TraceGen::getNextPacket()
{
    // shift things one step forward
    currElement = nextElement;
    nextElement.clear();

    // read the next element and set the complete flag
    traceComplete = !trace.read(nextElement);

    // it is the responsibility of the traceComplete flag to ensure we
    // always have a valid element here
    assert(currElement.isValid());

    DPRINTF(TrafficGen, "TraceGen::getNextPacket: %c %d %d %d 0x%x\n",
            currElement.cmd.isRead() ? 'r' : 'w',
            currElement.addr,
            currElement.blocksize,
            currElement.tick,
            currElement.flags);

    PacketPtr pkt = getPacket(currElement.addr + addrOffset,
                              currElement.blocksize,
                              currElement.cmd, currElement.flags);

    if (!traceComplete)
        DPRINTF(TrafficGen, "nextElement: %c addr %d size %d tick %d (%d)\n",
                nextElement.cmd.isRead() ? 'r' : 'w',
                nextElement.addr,
                nextElement.blocksize,
                nextElement.tick + tickOffset,
                nextElement.tick);

    return pkt;
}

void
TraceGen::exit()
{
    // Check if we reached the end of the trace file. If we did not
    // then we want to generate a warning stating that not the entire
    // trace was played.
    if (!traceComplete) {
        warn("Trace player %s was unable to replay the entire trace!\n",
             name());
    }

    // Clear any flags and start over again from the beginning of the
    // file
    trace.reset();
}
