/*
 * Copyright (c) 2013-2015 ARM Limited
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
 * Authors: Stephan Diestelhorst
 */

/**
 * @file
 * Implementation of a snoop filter.
 */

#include "base/misc.hh"
#include "base/trace.hh"
#include "debug/SnoopFilter.hh"
#include "mem/snoop_filter.hh"
#include "sim/system.hh"

void
SnoopFilter::eraseIfNullEntry(SnoopFilterCache::iterator& sf_it)
{
    SnoopItem& sf_item = sf_it->second;
    if (!(sf_item.requested | sf_item.holder)) {
        cachedLocations.erase(sf_it);
        DPRINTF(SnoopFilter, "%s:   Removed SF entry.\n",
                __func__);
    }
}

std::pair<SnoopFilter::SnoopList, Cycles>
SnoopFilter::lookupRequest(const Packet* cpkt, const SlavePort& slave_port)
{
    DPRINTF(SnoopFilter, "%s: packet src %s addr 0x%x cmd %s\n",
            __func__, slave_port.name(), cpkt->getAddr(), cpkt->cmdString());

    // Ultimately we should check if the packet came from an
    // allocating source, not just if the port is snooping
    bool allocate = !cpkt->req->isUncacheable() && slave_port.isSnooping();
    Addr line_addr = cpkt->getBlockAddr(linesize);
    SnoopMask req_port = portToMask(slave_port);
    reqLookupResult = cachedLocations.find(line_addr);
    bool is_hit = (reqLookupResult != cachedLocations.end());

    // If the snoop filter has no entry, and we should not allocate,
    // do not create a new snoop filter entry, simply return a NULL
    // portlist.
    if (!is_hit && !allocate)
        return snoopDown(lookupLatency);

    // If no hit in snoop filter create a new element and update iterator
    if (!is_hit)
        reqLookupResult = cachedLocations.emplace(line_addr, SnoopItem()).first;
    SnoopItem& sf_item = reqLookupResult->second;
    SnoopMask interested = sf_item.holder | sf_item.requested;

    // Store unmodified value of snoop filter item in temp storage in
    // case we need to revert because of a send retry in
    // updateRequest.
    retryItem = sf_item;

    totRequests++;
    if (is_hit) {
        // Single bit set -> value is a power of two
        if (isPow2(interested))
            hitSingleRequests++;
        else
            hitMultiRequests++;
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
            panic_if(sf_item.requested & req_port, "double request :( " \
                     "SF value %x.%x\n", sf_item.requested, sf_item.holder);

            // Mark in-flight requests to distinguish later on
            sf_item.requested |= req_port;
            DPRINTF(SnoopFilter, "%s:   new SF value %x.%x\n",
                    __func__,  sf_item.requested, sf_item.holder);
        } else {
            // NOTE: The memInhibit might have been asserted by a cache closer
            // to the CPU, already -> the response will not be seen by this
            // filter -> we do not need to keep the in-flight request, but make
            // sure that we know that that cluster has a copy
            panic_if(!(sf_item.holder & req_port), "Need to hold the value!");
            DPRINTF(SnoopFilter,
                    "%s: not marking request. SF value %x.%x\n",
                    __func__,  sf_item.requested, sf_item.holder);
        }
    } else { // if (!cpkt->needsResponse())
        assert(cpkt->isEviction());
        // make sure that the sender actually had the line
        panic_if(!(sf_item.holder & req_port), "requester %x is not a " \
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
SnoopFilter::finishRequest(bool will_retry, const Packet* cpkt)
{
    if (reqLookupResult != cachedLocations.end()) {
        // since we rely on the caller, do a basic check to ensure
        // that finishRequest is being called following lookupRequest
        assert(reqLookupResult->first == cpkt->getBlockAddr(linesize));
        if (will_retry) {
            // Undo any changes made in lookupRequest to the snoop filter
            // entry if the request will come again. retryItem holds
            // the previous value of the snoopfilter entry.
            reqLookupResult->second = retryItem;

            DPRINTF(SnoopFilter, "%s:   restored SF value %x.%x\n",
                    __func__,  retryItem.requested, retryItem.holder);
        }

        eraseIfNullEntry(reqLookupResult);
    }
}

std::pair<SnoopFilter::SnoopList, Cycles>
SnoopFilter::lookupSnoop(const Packet* cpkt)
{
    DPRINTF(SnoopFilter, "%s: packet addr 0x%x cmd %s\n",
            __func__, cpkt->getAddr(), cpkt->cmdString());

    assert(cpkt->isRequest());

    Addr line_addr = cpkt->getBlockAddr(linesize);
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

    DPRINTF(SnoopFilter, "%s:   old SF value %x.%x\n",
            __func__, sf_item.requested, sf_item.holder);

    SnoopMask interested = (sf_item.holder | sf_item.requested);

    totSnoops++;
    // Single bit set -> value is a power of two
    if (isPow2(interested))
        hitSingleSnoops++;
    else
        hitMultiSnoops++;

    // ReadEx and Writes require both invalidation and exlusivity, while reads
    // require neither. Writebacks on the other hand require exclusivity but
    // not the invalidation. Previously Writebacks did not generate upward
    // snoops so this was never an aissue. Now that Writebacks generate snoops
    // we need to special case for Writebacks.
    assert(cpkt->isWriteback() || cpkt->req->isUncacheable() ||
           (cpkt->isInvalidate() == cpkt->needsWritable()));
    if (cpkt->isInvalidate() && !sf_item.requested) {
        // Early clear of the holder, if no other request is currently going on
        // @todo: This should possibly be updated even though we do not filter
        // upward snoops
        sf_item.holder = 0;
    }

    eraseIfNullEntry(sf_it);
    DPRINTF(SnoopFilter, "%s:   new SF value %x.%x interest: %x \n",
            __func__, sf_item.requested, sf_item.holder, interested);

    return snoopSelected(maskToPortList(interested), lookupLatency);
}

void
SnoopFilter::updateSnoopResponse(const Packet* cpkt,
                                 const SlavePort& rsp_port,
                                 const SlavePort& req_port)
{
    DPRINTF(SnoopFilter, "%s: packet rsp %s req %s addr 0x%x cmd %s\n",
            __func__, rsp_port.name(), req_port.name(), cpkt->getAddr(),
            cpkt->cmdString());

    assert(cpkt->isResponse());
    assert(cpkt->cacheResponding());

    // Ultimately we should check if the packet came from an
    // allocating source, not just if the port is snooping
    bool allocate = !cpkt->req->isUncacheable() && req_port.isSnooping();
    if (!allocate)
        return;

    Addr line_addr = cpkt->getBlockAddr(linesize);
    SnoopMask rsp_mask = portToMask(rsp_port);
    SnoopMask req_mask = portToMask(req_port);
    SnoopItem& sf_item = cachedLocations[line_addr];

    DPRINTF(SnoopFilter, "%s:   old SF value %x.%x\n",
            __func__,  sf_item.requested, sf_item.holder);

    // The source should have the line
    panic_if(!(sf_item.holder & rsp_mask), "SF value %x.%x does not have "\
             "the line\n", sf_item.requested, sf_item.holder);

    // The destination should have had a request in
    panic_if(!(sf_item.requested & req_mask), "SF value %x.%x missing "\
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
    sf_item.holder |=  req_mask;
    sf_item.requested &= ~req_mask;
    assert(sf_item.requested | sf_item.holder);
    DPRINTF(SnoopFilter, "%s:   new SF value %x.%x\n",
            __func__, sf_item.requested, sf_item.holder);
}

void
SnoopFilter::updateSnoopForward(const Packet* cpkt,
        const SlavePort& rsp_port, const MasterPort& req_port)
{
    DPRINTF(SnoopFilter, "%s: packet rsp %s req %s addr 0x%x cmd %s\n",
            __func__, rsp_port.name(), req_port.name(), cpkt->getAddr(),
            cpkt->cmdString());

    assert(cpkt->isResponse());
    assert(cpkt->cacheResponding());

    Addr line_addr = cpkt->getBlockAddr(linesize);
    auto sf_it = cachedLocations.find(line_addr);
    bool is_hit = sf_it != cachedLocations.end();

    // Nothing to do if it is not a hit
    if (!is_hit)
        return;

    SnoopItem& sf_item = sf_it->second;

    DPRINTF(SnoopFilter, "%s:   old SF value %x.%x\n",
            __func__,  sf_item.requested, sf_item.holder);

    // If the snoop response has no sharers the line is passed in
    // Modified state, and we know that there are no other copies, or
    // they will all be invalidated imminently
    if (!cpkt->hasSharers()) {
        sf_item.holder = 0;
    }
    DPRINTF(SnoopFilter, "%s:   new SF value %x.%x\n",
            __func__, sf_item.requested, sf_item.holder);
    eraseIfNullEntry(sf_it);

}

void
SnoopFilter::updateResponse(const Packet* cpkt, const SlavePort& slave_port)
{
    DPRINTF(SnoopFilter, "%s: packet src %s addr 0x%x cmd %s\n",
            __func__, slave_port.name(), cpkt->getAddr(), cpkt->cmdString());

    assert(cpkt->isResponse());

    // Ultimately we should check if the packet came from an
    // allocating source, not just if the port is snooping
    bool allocate = !cpkt->req->isUncacheable() && slave_port.isSnooping();
    if (!allocate)
        return;

    Addr line_addr = cpkt->getBlockAddr(linesize);
    SnoopMask slave_mask = portToMask(slave_port);
    SnoopItem& sf_item = cachedLocations[line_addr];

    DPRINTF(SnoopFilter, "%s:   old SF value %x.%x\n",
            __func__,  sf_item.requested, sf_item.holder);

    // Make sure we have seen the actual request, too
    panic_if(!(sf_item.requested & slave_mask), "SF value %x.%x missing "\
             "request bit\n", sf_item.requested, sf_item.holder);

    // Update the residency of the cache line. If the response has no
    // sharers we know that the line has been invalidated in all
    // branches that are not where we are responding to.
     if (!cpkt->hasSharers())
        sf_item.holder = 0;
    sf_item.holder |=  slave_mask;
    sf_item.requested &= ~slave_mask;
    assert(sf_item.holder | sf_item.requested);
    DPRINTF(SnoopFilter, "%s:   new SF value %x.%x\n",
            __func__, sf_item.requested, sf_item.holder);
}

void
SnoopFilter::regStats()
{
    totRequests
        .name(name() + ".tot_requests")
        .desc("Total number of requests made to the snoop filter.");

    hitSingleRequests
        .name(name() + ".hit_single_requests")
        .desc("Number of requests hitting in the snoop filter with a single "\
              "holder of the requested data.");

    hitMultiRequests
        .name(name() + ".hit_multi_requests")
        .desc("Number of requests hitting in the snoop filter with multiple "\
              "(>1) holders of the requested data.");

    totSnoops
        .name(name() + ".tot_snoops")
        .desc("Total number of snoops made to the snoop filter.");

    hitSingleSnoops
        .name(name() + ".hit_single_snoops")
        .desc("Number of snoops hitting in the snoop filter with a single "\
              "holder of the requested data.");

    hitMultiSnoops
        .name(name() + ".hit_multi_snoops")
        .desc("Number of snoops hitting in the snoop filter with multiple "\
              "(>1) holders of the requested data.");
}

SnoopFilter *
SnoopFilterParams::create()
{
    return new SnoopFilter(this);
}
