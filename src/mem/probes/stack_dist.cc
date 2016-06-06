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
 * Authors: Andreas Sandberg
 */

#include "mem/probes/stack_dist.hh"

#include "params/StackDistProbe.hh"
#include "sim/system.hh"

StackDistProbe::StackDistProbe(StackDistProbeParams *p)
    : BaseMemProbe(p),
      lineSize(p->line_size),
      disableLinearHists(p->disable_linear_hists),
      disableLogHists(p->disable_log_hists),
      calc(p->verify)
{
    fatal_if(p->system->cacheLineSize() > p->line_size,
             "The stack distance probe must use a cache line size that is "
             "larger or equal to the system's cahce line size.");
}

void
StackDistProbe::regStats()
{
    BaseMemProbe::regStats();

    const StackDistProbeParams *p(
        dynamic_cast<const StackDistProbeParams *>(params()));
    assert(p);

    using namespace Stats;

    readLinearHist
        .init(p->linear_hist_bins)
        .name(name() + ".readLinearHist")
        .desc("Reads linear distribution")
        .flags(disableLinearHists ? nozero : pdf);

    readLogHist
        .init(p->log_hist_bins)
        .name(name() + ".readLogHist")
        .desc("Reads logarithmic distribution")
        .flags(disableLogHists ? nozero : pdf);

    writeLinearHist
        .init(p->linear_hist_bins)
        .name(name() + ".writeLinearHist")
        .desc("Writes linear distribution")
        .flags(disableLinearHists ? nozero : pdf);

    writeLogHist
        .init(p->log_hist_bins)
        .name(name() + ".writeLogHist")
        .desc("Writes logarithmic distribution")
        .flags(disableLogHists ? nozero : pdf);

    infiniteSD
        .name(name() + ".infinity")
        .desc("Number of requests with infinite stack distance")
        .flags(nozero);
}

void
StackDistProbe::handleRequest(const ProbePoints::PacketInfo &pkt_info)
{
    // only capturing read and write requests (which allocate in the
    // cache)
    if (!pkt_info.cmd.isRead() && !pkt_info.cmd.isWrite())
        return;

    // Align the address to a cache line size
    const Addr aligned_addr(roundDown(pkt_info.addr, lineSize));

    // Calculate the stack distance
    const uint64_t sd(calc.calcStackDistAndUpdate(aligned_addr).first);
    if (sd == StackDistCalc::Infinity) {
        infiniteSD++;
        return;
    }

    // Sample the stack distance of the address in linear bins
    if (!disableLinearHists) {
        if (pkt_info.cmd.isRead())
            readLinearHist.sample(sd);
        else
            writeLinearHist.sample(sd);
    }

    if (!disableLogHists) {
        int sd_lg2 = sd == 0 ? 1 : floorLog2(sd);

        // Sample the stack distance of the address in log bins
        if (pkt_info.cmd.isRead())
            readLogHist.sample(sd_lg2);
        else
            writeLogHist.sample(sd_lg2);
    }
}


StackDistProbe *
StackDistProbeParams::create()
{
    return new StackDistProbe(this);
}
