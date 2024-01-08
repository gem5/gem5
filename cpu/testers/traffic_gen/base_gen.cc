/*
 * Copyright (c) 2012-2013, 2016-2018 ARM Limited
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

#include "cpu/testers/traffic_gen/base_gen.hh"

#include <algorithm>

#include "base/logging.hh"
#include "cpu/testers/traffic_gen/base.hh"

namespace gem5
{

BaseGen::BaseGen(SimObject &obj, RequestorID requestor_id, Tick _duration)
    : _name(obj.name()), requestorId(requestor_id),
      duration(_duration)
{
}

PacketPtr
BaseGen::getPacket(Addr addr, unsigned size, const MemCmd& cmd,
                   Request::FlagsType flags)
{
    // Create new request
    RequestPtr req = std::make_shared<Request>(addr, size, flags,
                                               requestorId);
    // Dummy PC to have PC-based prefetchers latch on; get entropy into higher
    // bits
    req->setPC(((Addr)requestorId) << 2);

    // Embed it in a packet
    PacketPtr pkt = new Packet(req, cmd);

    uint8_t* pkt_data = new uint8_t[req->getSize()];
    pkt->dataDynamic(pkt_data);

    if (cmd.isWrite()) {
        std::fill_n(pkt_data, req->getSize(), (uint8_t)requestorId);
    }

    return pkt;
}

StochasticGen::StochasticGen(SimObject &obj,
                             RequestorID requestor_id, Tick _duration,
                             Addr start_addr, Addr end_addr,
                             Addr _blocksize, Addr cacheline_size,
                             Tick min_period, Tick max_period,
                             uint8_t read_percent, Addr data_limit)
        : BaseGen(obj, requestor_id, _duration),
          startAddr(start_addr), endAddr(end_addr),
          blocksize(_blocksize), cacheLineSize(cacheline_size),
          minPeriod(min_period), maxPeriod(max_period),
          readPercent(read_percent), dataLimit(data_limit)
{
    if (blocksize > cacheLineSize)
        fatal("TrafficGen %s block size (%d) is larger than "
              "cache line size (%d)\n", name(),
              blocksize, cacheLineSize);

    if (read_percent > 100)
        fatal("%s cannot have more than 100% reads", name());

    if (min_period > max_period)
        fatal("%s cannot have min_period > max_period", name());
}

} // namespace gem5
