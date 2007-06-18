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
 *          Ron Dreslinski
 */

/**
 * @file
 * Declaration of a simple coherence policy.
 */

#ifndef __SIMPLE_COHERENCE_HH__
#define __SIMPLE_COHERENCE_HH__

#include <string>

#include "mem/packet.hh"
#include "mem/cache/cache_blk.hh"
#include "mem/cache/miss/mshr_queue.hh"
#include "mem/cache/coherence/coherence_protocol.hh"

class BaseCache;

/**
 * A simple MP coherence policy. This policy assumes an atomic bus and only one
 * level of cache.
 */
class SimpleCoherence
{
  protected:
    /** Pointer to the parent cache. */
    BaseCache *cache;
    /** Pointer to the coherence protocol. */
    CoherenceProtocol *protocol;

  public:
    /**
     * Construct and initialize this coherence policy.
     * @param _protocol The coherence protocol to use.
     */
    SimpleCoherence(CoherenceProtocol *_protocol)
        : protocol(_protocol)
    {
    }

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
     * This policy does not forward invalidates, return NULL.
     * @return NULL.
     */
    PacketPtr getPacket()
    {
        return NULL;
    }

    /**
     * Return the proper state given the current state and the bus response.
     * @param pkt The bus response.
     * @param current The current block state.
     * @return The new state.
     */
    CacheBlk::State getNewState(PacketPtr pkt,
                                CacheBlk::State current = 0)
    {
        return protocol->getNewState(pkt, current);
    }

    /**
     * Handle snooped bus requests.
     * @param pkt The snooped bus request.
     * @param blk The cache block corresponding to the request, if any.
     * @param mshr The MSHR corresponding to the request, if any.
     * @param new_state Return the new state for the block.
     */
    bool handleBusRequest(PacketPtr &pkt, CacheBlk *blk, MSHR *mshr,
                          CacheBlk::State &new_state)
    {
//	assert(mshr == NULL);
//Got rid of, there could be an MSHR, but it can't be in service
        if (blk != NULL)
        {
            if (pkt->cmd != MemCmd::Writeback) {
                return protocol->handleBusRequest(cache, pkt, blk, mshr,
                                              new_state);
            }
            else { //It is a writeback, must be ownership protocol, just keep state
                new_state = blk->status;
            }
        }
        return false;
    }

    /**
     * Get the proper bus command for the given command and status.
     * @param cmd The request's command.
     * @param state The current state of the cache block.
     * @return The proper bus command, as determined by the protocol.
     */
    MemCmd getBusCmd(MemCmd cmd,
                                  CacheBlk::State state)
    {
        if (cmd == MemCmd::Writeback) return MemCmd::Writeback;
        return protocol->getBusCmd(cmd, state);
    }

    /**
     * Return true if this coherence policy can handle fast cache writes.
     */
    bool allowFastWrites() { return false; }

    bool hasProtocol() { return true; }
};

#endif //__SIMPLE_COHERENCE_HH__








