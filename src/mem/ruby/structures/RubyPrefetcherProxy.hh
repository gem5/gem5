/*
 * Copyright (c) 2023 ARM Limited
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
 */

#ifndef __MEM_RUBY_STRUCTURES_RUBY_PREFETCHER_WRAPPER_HH__
#define __MEM_RUBY_STRUCTURES_RUBY_PREFETCHER_WRAPPER_HH__

#include <unordered_map>

#include "mem/cache/cache_probe_arg.hh"
#include "mem/cache/prefetch/base.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "mem/ruby/slicc_interface/RubyRequest.hh"

namespace gem5
{

namespace ruby
{

/**
 * This is a proxy for prefetcher class in classic memory. This wrapper
 * enables a SLICC machine to interact with classic prefetchers.
 *
 * The expected use case for this class is to instantiate it in the SLICC
 * state machine definition and provide a pointer to the prefetcher object
 * (typically defined in SLICC as one of the SM's configuration parameters)
 * and the prefetch queue where prefetch requests will be inserted.
 *
 * The SLICC SM can them use the notifyPF* functions to notify the prefetcher.
 *
 * Notes:
 *
 * This object's regProbePoints() must be called explicitly. The SLICC SM may
 * defined it's own regProbePoints() to call it.
 *
 * completePrefetch(Addr) must be called when a request injected into the
 * prefetch queue is completed.
 *
 * A nullptr prefetcher can be provided, in which case the notifyPf* are
 * no-ops.
 *
 */
class RubyPrefetcherProxy : public CacheAccessor, public Named
{
  public:

    RubyPrefetcherProxy(AbstractController* parent,
                        prefetch::Base* prefetcher,
                        MessageBuffer *pf_queue);

    /** Deschedled the ready prefetch event */
    void deschedulePrefetch();

    /** Notifies a completed prefetch request */
    void completePrefetch(Addr addr);

    /**
     * Notify PF probes hit/miss/fill
     */
    void notifyPfHit(const RequestPtr& req, bool is_read,
                     const DataBlock& data_blk);
    void notifyPfMiss(const RequestPtr& req, bool is_read,
                      const DataBlock& data_blk);
    void notifyPfFill(const RequestPtr& req, const DataBlock& data_blk,
                      bool from_pf);
    void notifyPfEvict(Addr blkAddr, bool hwPrefetched,
                       RequestorID requestorID);

    /** Registers probes. */
    void regProbePoints();

  private:

    /** Schedule the next ready prefetch */
    void scheduleNextPrefetch();

    /** Issue prefetch to the contoller prefetch queue */
    void issuePrefetch();

    /** Prefetcher from classic memory */
    prefetch::Base* prefetcher;

    /** Ruby cache controller */
    AbstractController* cacheCntrl;

    /** Prefetch queue to the cache controller */
    MessageBuffer* pfQueue;

    /** List of issued prefetch request packets */
    std::unordered_map<Addr, PacketPtr> issuedPfPkts;

    /** Prefetch event */
    EventFunctionWrapper pfEvent;

    /** To probe when a cache hit occurs */
    ProbePointArg<CacheAccessProbeArg> *ppHit;

    /** To probe when a cache miss occurs */
    ProbePointArg<CacheAccessProbeArg> *ppMiss;

    /** To probe when a cache fill occurs */
    ProbePointArg<CacheAccessProbeArg> *ppFill;

    /**
     * To probe when the contents of a block are updated. Content updates
     * include data fills, overwrites, and invalidations, which means that
     * this probe partially overlaps with other probes.
     */
    ProbePointArg<CacheDataUpdateProbeArg> *ppDataUpdate;

  public:

    /** Accessor functions */

    bool inCache(Addr addr, bool is_secure) const override
    {
        return cacheCntrl->inCache(addr, is_secure);
    }

    bool hasBeenPrefetched(Addr addr, bool is_secure) const override
    {
        return cacheCntrl->hasBeenPrefetched(addr, is_secure);
    }

    bool hasBeenPrefetched(Addr addr, bool is_secure,
                            RequestorID requestor) const override
    {
        return cacheCntrl->hasBeenPrefetched(addr, is_secure, requestor);
    }

    bool inMissQueue(Addr addr, bool is_secure) const override
    {
        return cacheCntrl->inMissQueue(addr, is_secure);
    }

    bool coalesce() const override
    { return cacheCntrl->coalesce(); }

};

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_STRUCTURES_RUBY_PREFETCHER_WRAPPER_HH__
