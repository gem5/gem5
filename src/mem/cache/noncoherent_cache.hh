/*
 * Copyright (c) 2012-2018 ARM Limited
 * All rights reserved.
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
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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

/**
 * @file
 * Specifies a non-coherent cache. The non-coherent cache is expected
 * to be located below the point of coherency. All valid blocks in the
 * non-coherent cache can always be written to without any prior
 * invalidations or snoops.
 */

#ifndef __MEM_CACHE_NONCOHERENT_CACHE_HH__
#define __MEM_CACHE_NONCOHERENT_CACHE_HH__

#include "base/compiler.hh"
#include "base/logging.hh"
#include "base/types.hh"
#include "mem/cache/base.hh"
#include "mem/packet.hh"

namespace gem5
{

class CacheBlk;
class MSHR;
struct NoncoherentCacheParams;

/**
 * A non-coherent cache
 */
class NoncoherentCache : public BaseCache
{
  protected:
    bool access(PacketPtr pkt, CacheBlk *&blk, Cycles &lat,
                PacketList &writebacks) override;

    void handleTimingReqMiss(PacketPtr pkt, CacheBlk *blk, Tick forward_time,
                             Tick request_time) override;

    void recvTimingReq(PacketPtr pkt) override;

    void doWritebacks(PacketList &writebacks, Tick forward_time) override;

    void doWritebacksAtomic(PacketList &writebacks) override;

    void serviceMSHRTargets(MSHR *mshr, const PacketPtr pkt,
                            CacheBlk *blk) override;

    void recvTimingResp(PacketPtr pkt) override;

    void
    recvTimingSnoopReq(PacketPtr pkt) override
    {
        panic("Unexpected timing snoop request %s", pkt->print());
    }

    void
    recvTimingSnoopResp(PacketPtr pkt) override
    {
        panic("Unexpected timing snoop response %s", pkt->print());
    }

    Cycles handleAtomicReqMiss(PacketPtr pkt, CacheBlk *&blk,
                               PacketList &writebacks) override;

    Tick recvAtomic(PacketPtr pkt) override;

    Tick
    recvAtomicSnoop(PacketPtr pkt) override
    {
        panic("Unexpected atomic snoop request %s", pkt->print());
    }

    void functionalAccess(PacketPtr pkt, bool from_cpu_side) override;

    void satisfyRequest(PacketPtr pkt, CacheBlk *blk,
                        bool deferred_response = false,
                        bool pending_downgrade = false) override;

    /*
     * Creates a new packet with the request to be send to the memory
     * below. The noncoherent cache is below the point of coherence
     * and therefore all fills bring in writable, therefore the
     * needs_writeble parameter is ignored.
     */
    PacketPtr createMissPacket(PacketPtr cpu_pkt, CacheBlk *blk,
                               bool needs_writable,
                               bool is_whole_line_write) const override;

    [[nodiscard]] PacketPtr evictBlock(CacheBlk *blk) override;

  public:
    NoncoherentCache(const NoncoherentCacheParams &p);
};

} // namespace gem5

#endif // __MEM_CACHE_NONCOHERENTCACHE_HH__
