/*
 * Copyright (c) 2023 ARM Limited
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

#ifndef __MEM_CACHE_PROBE_ARG_HH__
#define __MEM_CACHE_PROBE_ARG_HH__

#include "mem/packet.hh"

namespace gem5
{

/**
 * Provides generic cache lookup functions. A cache may provide
 * a CacheAccessor object to other components that need to perform
 * a lookup outside the normal cache control flow. Currently this
 * is used by prefetchers that perform lookups when notified by
 * cache events.
 */
struct CacheAccessor
{
    /** Determine if address is in cache */
    virtual bool inCache(Addr addr, bool is_secure) const = 0;

    /** Determine if address has been prefetched */
    virtual bool hasBeenPrefetched(Addr addr, bool is_secure) const = 0;

    /** Determine if address has been prefetched by the requestor */
    virtual bool hasBeenPrefetched(Addr addr, bool is_secure,
                                   RequestorID requestor) const = 0;

    /** Determine if address is in cache miss queue */
    virtual bool inMissQueue(Addr addr, bool is_secure) const = 0;

    /** Determine if cache is coalescing writes */
    virtual bool coalesce() const = 0;
};

/**
 * Information provided to probes on a cache event.
 * @sa ppHit, ppMiss, ppFill in gem5::BaseCache (src/mem/cache/base.hh)
 */
class CacheAccessProbeArg
{
  public:
    /** Packet that triggered the cache access*/
    PacketPtr pkt;
    /** Accessor for the cache */
    CacheAccessor &cache;

    CacheAccessProbeArg(PacketPtr _pkt, CacheAccessor &_cache)
        : pkt(_pkt), cache(_cache)
    {}
};

/**
 * A data contents update is composed of the updated block's address,
 * the old contents, and the new contents.
 * @sa ppDataUpdate in gem5::BaseCache (src/mem/cache/base.hh)
 */
struct CacheDataUpdateProbeArg
{
    /** The updated block's address. */
    Addr addr;
    /** Whether the block belongs to the secure address space. */
    bool isSecure;
    /** Block original requestor */
    const RequestorID requestorID;
    /** The stale data contents. If zero-sized this update is a fill. */
    std::vector<uint64_t> oldData;
    /** The new data contents. If zero-sized this is an invalidation. */
    std::vector<uint64_t> newData;
    /** Set if the update is from a prefetch or evicting a prefetched
     *  block that was never used. */
    bool hwPrefetched;
    /** Accessor for the cache */
    CacheAccessor &accessor;

    CacheDataUpdateProbeArg(Addr _addr, bool is_secure,
                            RequestorID _requestorID, CacheAccessor &_accessor)
        : addr(_addr),
          isSecure(is_secure),
          requestorID(_requestorID),
          oldData(),
          newData(),
          accessor(_accessor)
    {}
};

} // namespace gem5

#endif //__MEM_CACHE_PROBE_ARG_HH__
