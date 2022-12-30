/*
 * Copyright (c) 2013-2017,2019 ARM Limited
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

/**
 * @file
 * Implementation of a snoop filter.
 */

#include "mem/snoop_filter.hh"

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/SnoopFilter.hh"
#include "sim/system.hh"

namespace gem5
{

const int SnoopFilter::SNOOP_MASK_SIZE;

void
SnoopFilter::eraseIfNullEntry(SnoopFilterCache::iterator& sf_it)
{
    SnoopItem& sf_item = sf_it->second;
    if ((sf_item.requested | sf_item.holder).none()) {
        cachedLocations.erase(sf_it);
        DPRINTF(SnoopFilter, "%s:   Removed SF entry.\n",
                __func__);
    }
}

std::pair<SnoopFilter::SnoopList, Cycles>
SnoopFilter::lookupRequest(const Packet* cpkt, const ResponsePort&
                           cpu_side_port)
{
    DPRINTF(SnoopFilter, "%s: src %s packet %s\n", __func__,
            cpu_side_port.name(), cpkt->print());

    // check if the packet came from a cache
    bool allocate = !cpkt->req->isUncacheable() && cpu_side_port.isSnooping()
        && cpkt->fromCache();
    Addr line_addr = cpkt->getBlockAddr(linesize);
    if (cpkt->isSecure()) {
        line_addr |= LineSecure;
    }
    SnoopMask req_port = portToMask(cpu_side_port);
    reqLookupResult.it = cachedLocations.find(line_addr);
    bool is_hit = (reqLookupResult.it != cachedLocations.end());

    // If the snoop filter has no entry, and we should not allocate,
    // do not create a new snoop filter entry, simply return a NULL
    // portlist.
    if (!is_hit && !allocate)
        return snoopDown(lookupLatency);

    // If no hit in snoop filter create a new element and update iterator
    if (!is_hit) {
        reqLookupResult.it =
            cachedLocations.emplace(line_addr, SnoopItem()).first;
    }
    SnoopItem& sf_item = reqLookupResult.it->second;
    SnoopMask interested = sf_item.holder | sf_item.requested;

    // Store unmodified value of snoop filter item in temp storage in
    // case we need to revert because of a send retry in
    // updateRequest.
    reqLookupResult.retryItem = sf_item;

    stats.totRequests++;
    if (is_hit) {
        if (interested.count() == 1)
            stats.hitSingleRequests++;
        else
            stats.hitMultiRequests++;
    }

    DPRINTF(SnoopFilter, "%s:   SF value %x.%x\n",
            __func__, sf_item.requested, sf_item.holder);

    // If we are not allocating, we are done
    if (!allocate)
        return snoopSelected(maskToPortList(interested & ~req_port),
                             lookupLatency);

    if (cpkt->needsResponse()) {
        if (!cpkt->cacheResponding()) {
            // Max one request per address per port
            panic_if((sf_item.requested & req_port).any(),
                     "double request :( SF value %x.%x\n",
                     sf_item.requested, sf_item.holder);

            // Mark in-flight requests to distinguish later on
            sf_item.requested |= req_port;
            DPRINTF(SnoopFilter, "%s:   new SF value %x.%x\n",
                    __func__,  sf_item.requested, sf_item.holder);
        } else {
            // NOTE: The memInhibit might have been asserted by a cache closer
            // to the CPU, already -> the response will not be seen by this
            // filter -> we do not need to keep the in-flight request, but make
            // sure that we know that that cluster has a copy
            panic_if((sf_item.holder & req_port).none(),
                     "Need to hold the value!");
            DPRINTF(SnoopFilter,
                    "%s: not marking request. SF value %x.%x\n",
                    __func__,  sf_item.requested, sf_item.holder);
        }
    } else { // if (!cpkt->needsResponse())
        assert(cpkt->isEviction());
        // make sure that the sender actually had the line
        panic_if((sf_item.holder & req_port).none(), "requestor %x is not a " \
                 "holder :( SF value %x.%x\n", req_port,
                 sf_item.requested, sf_item.holder);
        // CleanEvicts and Writebacks -> the sender and all caches above
        // it may not have the line anymore.
        if (!cpkt->isBlockCached()) {
            sf_item.holder &= ~req_port;
            DPRINTF(SnoopFilter, "%s:   new SF value %x.%x\n",
                    __func__,  sf_item.requested, sf_item.holder);
        }
    }

    return snoopSelected(maskToPortList(interested & ~req_port), lookupLatency);
}

void
SnoopFilter::finishRequest(bool will_retry, Addr addr, bool is_secure)
{
    if (reqLookupResult.it != cachedLocations.end()) {
        // since we rely on the caller, do a basic check to ensure
        // that finishRequest is being called following lookupRequest
        assert(reqLookupResult.it->first == \
                (is_secure ? ((addr & ~(Addr(linesize - 1))) | LineSecure) : \
                 (addr & ~(Addr(linesize - 1)))));
        if (will_retry) {
            SnoopItem retry_item = reqLookupResult.retryItem;
            // Undo any changes made in lookupRequest to the snoop filter
            // entry if the request will come again. retryItem holds
            // the previous value of the snoopfilter entry.
            reqLookupResult.it->second = retry_item;

            DPRINTF(SnoopFilter, "%s:   restored SF value %x.%x\n",
                    __func__,  retry_item.requested, retry_item.holder);
        }

        eraseIfNullEntry(reqLookupResult.it);
    }
}

std::pair<SnoopFilter::SnoopList, Cycles>
SnoopFilter::lookupSnoop(const Packet* cpkt)
{
    DPRINTF(SnoopFilter, "%s: packet %s\n", __func__, cpkt->print());

    assert(cpkt->isRequest());

    Addr line_addr = cpkt->getBlockAddr(linesize);
    if (cpkt->isSecure()) {
        line_addr |= LineSecure;
    }
    auto sf_it = cachedLocations.find(line_addr);
    bool is_hit = (sf_it != cachedLocations.end());

    panic_if(!is_hit && (cachedLocations.size() >= maxEntryCount),
             "snoop filter exceeded capacity of %d cache blocks\n",
             maxEntryCount);

    // If the snoop filter has no entry, simply return a NULL
    // portlist, there is no point creating an entry only to remove it
    // later
    if (!is_hit)
        return snoopDown(lookupLatency);

    SnoopItem& sf_item = sf_it->second;

    SnoopMask interested = (sf_item.holder | sf_item.requested);

    stats.totSnoops++;

    if (interested.count() == 1)
        stats.hitSingleSnoops++;
    else
        stats.hitMultiSnoops++;

    // ReadEx and Writes require both invalidation and exlusivity, while reads
    // require neither. Writebacks on the other hand require exclusivity but
    // not the invalidation. Previously Writebacks did not generate upward
    // snoops so this was never an issue. Now that Writebacks generate snoops
    // we need a special case for Writebacks. Additionally cache maintenance
    // operations can generate snoops as they clean and/or invalidate all
    // caches down to the specified point of reference.
    assert(cpkt->isWriteback() || cpkt->req->isUncacheable() ||
           (cpkt->isInvalidate() == cpkt->needsWritable()) ||
           cpkt->req->isCacheMaintenance());
    if (cpkt->isInvalidate() && sf_item.requested.none()) {
        // Early clear of the holder, if no other request is currently going on
        // @todo: This should possibly be updated even though we do not filter
        // upward snoops
        DPRINTF(SnoopFilter, "%s:   old SF value %x.%x\n",
                __func__, sf_item.requested, sf_item.holder);
        sf_item.holder = 0;
        DPRINTF(SnoopFilter, "%s:   new SF value %x.%x\n",
                __func__, sf_item.requested, sf_item.holder);
        eraseIfNullEntry(sf_it);
    }

    return snoopSelected(maskToPortList(interested), lookupLatency);
}

void
SnoopFilter::updateSnoopResponse(const Packet* cpkt,
                                 const ResponsePort& rsp_port,
                                 const ResponsePort& req_port)
{
    DPRINTF(SnoopFilter, "%s: rsp %s req %s packet %s\n",
            __func__, rsp_port.name(), req_port.name(), cpkt->print());

    assert(cpkt->isResponse());
    assert(cpkt->cacheResponding());

    // if this snoop response is due to an uncacheable request, or is
    // being turned into a normal response, there is nothing more to
    // do
    if (cpkt->req->isUncacheable() || !req_port.isSnooping()) {
        return;
    }

    Addr line_addr = cpkt->getBlockAddr(linesize);
    if (cpkt->isSecure()) {
        line_addr |= LineSecure;
    }
    SnoopMask rsp_mask = portToMask(rsp_port);
    SnoopMask req_mask = portToMask(req_port);
    SnoopItem& sf_item = cachedLocations[line_addr];

    DPRINTF(SnoopFilter, "%s:   old SF value %x.%x\n",
            __func__,  sf_item.requested, sf_item.holder);

    // The source should have the line
    panic_if((sf_item.holder & rsp_mask).none(),
             "SF value %x.%x does not have the line\n",
             sf_item.requested, sf_item.holder);

    // The destination should have had a request in
    panic_if((sf_item.requested & req_mask).none(), "SF value %x.%x missing "\
             "the original request\n",  sf_item.requested, sf_item.holder);

    // If the snoop response has no sharers the line is passed in
    // Modified state, and we know that there are no other copies, or
    // they will all be invalidated imminently
    if (!cpkt->hasSharers()) {
        DPRINTF(SnoopFilter,
                "%s: dropping %x because non-shared snoop "
                "response SF val: %x.%x\n", __func__,  rsp_mask,
                sf_item.requested, sf_item.holder);
        sf_item.holder = 0;
    }
    assert(!cpkt->isWriteback());
    // @todo Deal with invalidating responses
    sf_item.holder |=  req_mask;
    sf_item.requested &= ~req_mask;
    assert((sf_item.requested | sf_item.holder).any());
    DPRINTF(SnoopFilter, "%s:   new SF value %x.%x\n",
            __func__, sf_item.requested, sf_item.holder);
}

void
SnoopFilter::updateSnoopForward(const Packet* cpkt,
        const ResponsePort& rsp_port, const RequestPort& req_port)
{
    DPRINTF(SnoopFilter, "%s: rsp %s req %s packet %s\n",
            __func__, rsp_port.name(), req_port.name(), cpkt->print());

    assert(cpkt->isResponse());
    assert(cpkt->cacheResponding());

    Addr line_addr = cpkt->getBlockAddr(linesize);
    if (cpkt->isSecure()) {
        line_addr |= LineSecure;
    }
    auto sf_it = cachedLocations.find(line_addr);
    bool is_hit = sf_it != cachedLocations.end();

    // Nothing to do if it is not a hit
    if (!is_hit)
        return;

    // If the snoop response has no sharers the line is passed in
    // Modified state, and we know that there are no other copies, or
    // they will all be invalidated imminently
    if (!cpkt->hasSharers()) {
        SnoopItem& sf_item = sf_it->second;

        DPRINTF(SnoopFilter, "%s:   old SF value %x.%x\n",
                __func__, sf_item.requested, sf_item.holder);
        sf_item.holder = 0;
        DPRINTF(SnoopFilter, "%s:   new SF value %x.%x\n",
                __func__, sf_item.requested, sf_item.holder);

        eraseIfNullEntry(sf_it);
    }
}

void
SnoopFilter::updateResponse(const Packet* cpkt, const ResponsePort&
                            cpu_side_port)
{
    DPRINTF(SnoopFilter, "%s: src %s packet %s\n",
            __func__, cpu_side_port.name(), cpkt->print());

    assert(cpkt->isResponse());

    // we only allocate if the packet actually came from a cache, but
    // start by checking if the port is snooping
    if (cpkt->req->isUncacheable() || !cpu_side_port.isSnooping())
        return;

    // next check if we actually allocated an entry
    Addr line_addr = cpkt->getBlockAddr(linesize);
    if (cpkt->isSecure()) {
        line_addr |= LineSecure;
    }
    auto sf_it = cachedLocations.find(line_addr);
    if (sf_it == cachedLocations.end())
        return;

    SnoopMask response_mask = portToMask(cpu_side_port);
    SnoopItem& sf_item = sf_it->second;

    DPRINTF(SnoopFilter, "%s:   old SF value %x.%x\n",
            __func__,  sf_item.requested, sf_item.holder);

    // Make sure we have seen the actual request, too
    panic_if((sf_item.requested & response_mask).none(),
             "SF value %x.%x missing request bit\n",
             sf_item.requested, sf_item.holder);

    sf_item.requested &= ~response_mask;
    // Update the residency of the cache line.

    if (cpkt->req->isCacheMaintenance()) {
        // A cache clean response does not carry any data so it
        // shouldn't change the holders, unless it is invalidating.
        if (cpkt->isInvalidate()) {
            sf_item.holder &= ~response_mask;
        }
        eraseIfNullEntry(sf_it);
    } else {
        // Any other response implies that a cache above will have the
        // block.
        sf_item.holder |= response_mask;
        assert((sf_item.holder | sf_item.requested).any());
    }
    DPRINTF(SnoopFilter, "%s:   new SF value %x.%x\n",
            __func__, sf_item.requested, sf_item.holder);
}

SnoopFilter::SnoopFilterStats::SnoopFilterStats(statistics::Group *parent)
    : statistics::Group(parent),
      ADD_STAT(totRequests, statistics::units::Count::get(),
               "Total number of requests made to the snoop filter."),
      ADD_STAT(hitSingleRequests, statistics::units::Count::get(),
               "Number of requests hitting in the snoop filter with a single "
               "holder of the requested data."),
      ADD_STAT(hitMultiRequests, statistics::units::Count::get(),
               "Number of requests hitting in the snoop filter with multiple "
               "(>1) holders of the requested data."),
      ADD_STAT(totSnoops, statistics::units::Count::get(),
               "Total number of snoops made to the snoop filter."),
      ADD_STAT(hitSingleSnoops, statistics::units::Count::get(),
               "Number of snoops hitting in the snoop filter with a single "
               "holder of the requested data."),
      ADD_STAT(hitMultiSnoops, statistics::units::Count::get(),
               "Number of snoops hitting in the snoop filter with multiple "
               "(>1) holders of the requested data.")
{}

void
SnoopFilter::regStats()
{
    SimObject::regStats();
}

} // namespace gem5
