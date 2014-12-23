/*
 * Copyright (c) 2013-2014 ARM Limited
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
 *          Mitch Hayenga
 */

/**
 * @file
 * Miss and writeback queue declarations.
 */

#ifndef __MEM_CACHE_PREFETCH_BASE_HH__
#define __MEM_CACHE_PREFETCH_BASE_HH__

#include "base/statistics.hh"
#include "mem/packet.hh"
#include "params/BasePrefetcher.hh"
#include "sim/clocked_object.hh"

class BaseCache;

class BasePrefetcher : public ClockedObject
{
  protected:

    // PARAMETERS

    /** Pointr to the parent cache. */
    BaseCache* cache;

    /** The block size of the parent cache. */
    unsigned blkSize;

    /** System we belong to */
    System* system;

    /** Only consult prefetcher on cache misses? */
    bool onMiss;

    /** Consult prefetcher on reads? */
    bool onRead;

    /** Consult prefetcher on reads? */
    bool onWrite;

    /** Consult prefetcher on data accesses? */
    bool onData;

    /** Consult prefetcher on instruction accesses? */
    bool onInst;

    /** Request id for prefetches */
    MasterID masterId;

    const Addr pageBytes;

    /** Determine if this access should be observed */
    bool observeAccess(const PacketPtr &pkt) const;

    /** Determine if address is in cache */
    bool inCache(Addr addr, bool is_secure) const;

    /** Determine if address is in cache miss queue */
    bool inMissQueue(Addr addr, bool is_secure) const;

    /** Determine if addresses are on the same page */
    bool samePage(Addr a, Addr b) const;

    Stats::Scalar pfIssued;

  public:

    BasePrefetcher(const BasePrefetcherParams *p);

    virtual ~BasePrefetcher() {}

    void setCache(BaseCache *_cache);

    /**
     * Notify prefetcher of cache access (may be any access or just
     * misses, depending on cache parameters.)
     * @retval Time of next prefetch availability, or MaxTick if none.
     */
    virtual Tick notify(const PacketPtr &pkt) = 0;

    virtual PacketPtr getPacket() = 0;

    virtual Tick nextPrefetchReadyTime() const = 0;

    virtual void regStats();
};
#endif //__MEM_CACHE_PREFETCH_BASE_HH__
