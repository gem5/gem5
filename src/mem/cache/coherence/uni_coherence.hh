/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 *
 * Authors: Erik Hallnor
 */

#ifndef __UNI_COHERENCE_HH__
#define __UNI_COHERENCE_HH__

#include "base/trace.hh"
#include "base/misc.hh"
#include "mem/cache/cache_blk.hh"
#include "mem/cache/miss/mshr_queue.hh"
#include "mem/packet.hh"

class BaseCache;

class UniCoherence
{
  protected:
    /** Buffers to hold forwarded invalidates. */
    MSHRQueue cshrs;
    /** Pointer to the parent cache. */
    BaseCache *cache;

  public:
    /**
     * Construct and initialize this coherence policy.
     */
    UniCoherence();

    /**
     * Set the pointer to the parent cache.
     * @param _cache The parent cache.
     */
    void setCache(BaseCache *_cache)
    {
        cache = _cache;
    }

    /**
     * Register statistics.
     * @param name The name to prepend to stat descriptions.
     */
    void regStats(const std::string &name)
    {
    }

    /**
     * Return Read.
     * @param cmd The request's command.
     * @param state The current state of the cache block.
     * @return The proper bus command, as determined by the protocol.
     * @todo Make changes so writebacks don't get here.
     */
    MemCmd getBusCmd(MemCmd cmd, CacheBlk::State state)
    {
        if (cmd == MemCmd::HardPFReq && state)
            warn("Trying to issue a prefetch to a block we already have\n");
        if (cmd == MemCmd::Writeback)
            return MemCmd::Writeback;
        return MemCmd::ReadReq;
    }

    /**
     * Just return readable and writeable.
     * @param pkt The bus response.
     * @param current The current block state.
     * @return The new state.
     */
    CacheBlk::State getNewState(PacketPtr &pkt, CacheBlk::State current)
    {
        if (pkt->senderState) //Blocking Buffers don't get mshrs
        {
            if (((MSHR *)(pkt->senderState))->originalCmd == MemCmd::HardPFReq) {
                DPRINTF(HWPrefetch, "Marking a hardware prefetch as such in the state\n");
                return BlkHWPrefetched | BlkValid | BlkWritable;
            }
            else {
                return BlkValid | BlkWritable;
            }
        }
        //@todo What about prefetching with blocking buffers
        else
            return BlkValid | BlkWritable;
    }

    /**
     * Return outstanding invalidate to forward.
     * @return The next invalidate to forward to lower levels of cache.
     */
    PacketPtr getPacket();

    /**
     * Was the CSHR request was sent successfully?
     * @param pkt The request.
     * @param success True if the request was sent successfully.
     */
    void sendResult(PacketPtr &pkt, MSHR* cshr, bool success);

    /**
     * Handle snooped bus requests.
     * @param pkt The snooped bus request.
     * @param blk The cache block corresponding to the request, if any.
     * @param mshr The MSHR corresponding to the request, if any.
     * @param new_state The new coherence state of the block.
     * @return True if the request should be satisfied locally.
     */
    bool handleBusRequest(PacketPtr &pkt, CacheBlk *blk, MSHR *mshr,
                          CacheBlk::State &new_state);

    /**
     * Return true if this coherence policy can handle fast cache writes.
     */
    bool allowFastWrites() { return true; }

    bool hasProtocol() { return false; }

    bool propogateInvalidate(PacketPtr pkt, bool isTiming);
};

#endif //__UNI_COHERENCE_HH__
