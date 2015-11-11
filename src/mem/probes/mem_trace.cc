/*
 * Copyright (c) 2015 ARM Limited
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
 *          Andreas Sandberg
 */

#include "mem/probes/mem_trace.hh"

#include "base/callback.hh"
#include "base/output.hh"
#include "params/MemTraceProbe.hh"
#include "proto/packet.pb.h"

MemTraceProbe::MemTraceProbe(MemTraceProbeParams *p)
    : BaseMemProbe(p),
      traceStream(nullptr)
{
    std::string filename;
    if (p->trace_file != "") {
        // If the trace file is not specified as an absolute path,
        // append the current simulation output directory
        filename = simout.resolve(p->trace_file);

        const std::string suffix = ".gz";
        // If trace_compress has been set, check the suffix. Append
        // accordingly.
        if (p->trace_compress &&
            filename.compare(filename.size() - suffix.size(), suffix.size(),
                             suffix) != 0)
            filename = filename + suffix;
    } else {
        // Generate a filename from the name of the SimObject. Append .trc
        // and .gz if we want compression enabled.
        filename = simout.resolve(name() + ".trc" +
                                  (p->trace_compress ? ".gz" : ""));
    }

    traceStream = new ProtoOutputStream(filename);

    // Create a protobuf message for the header and write it to
    // the stream
    ProtoMessage::PacketHeader header_msg;
    header_msg.set_obj_id(name());
    header_msg.set_tick_freq(SimClock::Frequency);
    traceStream->write(header_msg);

    // Register a callback to compensate for the destructor not
    // being called. The callback forces the stream to flush and
    // closes the output file.
    registerExitCallback(
        new MakeCallback<MemTraceProbe, &MemTraceProbe::closeStreams>(this));
}

void
MemTraceProbe::closeStreams()
{
    if (traceStream != NULL)
        delete traceStream;
}

void
MemTraceProbe::handleRequest(const ProbePoints::PacketInfo &pkt_info)
{
    ProtoMessage::Packet pkt_msg;

    pkt_msg.set_tick(curTick());
    pkt_msg.set_cmd(pkt_info.cmd.toInt());
    pkt_msg.set_flags(pkt_info.flags);
    pkt_msg.set_addr(pkt_info.addr);
    pkt_msg.set_size(pkt_info.size);

    traceStream->write(pkt_msg);
}


MemTraceProbe *
MemTraceProbeParams::create()
{
    return new MemTraceProbe(this);
}
