/*
 * Copyright (c) 2005 The Regents of The University of Michigan
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
 * Authors: Ron Dreslinski
 */

/**
 * @file
 * Miss and writeback queue declarations.
 */

#ifndef __MEM_CACHE_PREFETCH_BASE_PREFETCHER_HH__
#define __MEM_CACHE_PREFETCH_BASE_PREFETCHER_HH__

#include "mem/packet.hh"
#include <list>

class BaseCache;
class BasePrefetcher
{
  protected:

    /** The Prefetch Queue. */
    std::list<Packet *> pf;

    // PARAMETERS

    /** The number of MSHRs in the Prefetch Queue. */
    const int size;

    /** Pointr to the parent cache. */
    BaseCache* cache;

    /** The block size of the parent cache. */
    int blkSize;

    /** Do we prefetch across page boundaries. */
    bool pageStop;

    /** Do we remove prefetches with later times than a new miss.*/
    bool serialSquash;

    /** Do we check if it is in the cache when inserting into buffer,
        or removing.*/
    bool cacheCheckPush;

    /** Do we prefetch on only data reads, or on inst reads as well. */
    bool only_data;

  public:

    Stats::Scalar<> pfIdentified;
    Stats::Scalar<> pfMSHRHit;
    Stats::Scalar<> pfCacheHit;
    Stats::Scalar<> pfBufferHit;
    Stats::Scalar<> pfRemovedFull;
    Stats::Scalar<> pfRemovedMSHR;
    Stats::Scalar<> pfIssued;
    Stats::Scalar<> pfSpanPage;
    Stats::Scalar<> pfSquashed;

    void regStats(const std::string &name);

  public:
    BasePrefetcher(int numMSHRS, bool pageStop, bool serialSquash,
                   bool cacheCheckPush, bool onlyData);

    virtual ~BasePrefetcher() {}

    void setCache(BaseCache *_cache);

    void handleMiss(Packet * &pkt, Tick time);

    Packet * getPacket();

    bool havePending()
    {
        return !pf.empty();
    }

    virtual void calculatePrefetch(Packet * &pkt,
                                   std::list<Addr> &addresses,
                                   std::list<Tick> &delays) = 0;

    virtual bool inCache(Packet * &pkt) = 0;

    virtual bool inMissQueue(Addr address, int asid) = 0;

    std::list<Packet *>::iterator inPrefetch(Addr address);
};


#endif //__MEM_CACHE_PREFETCH_BASE_PREFETCHER_HH__
