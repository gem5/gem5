/*
 * Copyright (c) 2013 ARM Limited
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
 * Authors: Stephan Diestelhorst <stephan.diestelhorst@arm.com>
 */

/**
 * @file
 * Definition of a snoop filter.
 */

#include "base/misc.hh"
#include "base/trace.hh"
#include "debug/SnoopFilter.hh"
#include "mem/snoop_filter.hh"
#include "sim/system.hh"

std::pair<SnoopFilter::SnoopList, Cycles>
SnoopFilter::lookupRequest(const Packet* cpkt, const SlavePort& slave_port)
{
    DPRINTF(SnoopFilter, "%s: packet src %s addr 0x%x cmd %s\n",
            __func__, slave_port.name(), cpkt->getAddr(), cpkt->cmdString());

    Addr line_addr = cpkt->getAddr() & ~(linesize - 1);
    SnoopMask req_port = portToMask(slave_port);
    SnoopItem& sf_item  = cachedLocations[line_addr];

    DPRINTF(SnoopFilter, "%s:   SF value %x.%x\n",
            __func__, sf_item.requested, sf_item.holder);

    if (cpkt->needsResponse()) {
        if (!cpkt->memInhibitAsserted()) {
            // Max one request per address per port
            panic_if(sf_item.requested & req_port, "double request :( "\
                     "SF value %x.%x\n", sf_item.requested, sf_item.holder);

            // Mark in-flight requests to distinguish later on
            sf_item.requested |= req_port;
        } else {
            // NOTE: The memInhibit might have been asserted by a cache closer
            // to the CPU, already -> the response will not be seen by this
            // filter -> we do not need to keep the in-flight request, but make
            // sure that we know that that cluster has a copy
            panic_if(!(sf_item.holder & req_port), "Need to hold the value!");
            DPRINTF(SnoopFilter, "%s:   not marking request. SF value %x.%x\n",
                    __func__,  sf_item.requested, sf_item.holder);
        }
        DPRINTF(SnoopFilter, "%s:   new SF value %x.%x\n",
                __func__,  sf_item.requested, sf_item.holder);
    }
    SnoopMask interested = (sf_item.holder | sf_item.requested) & ~req_port;
    return snoopSelected(maskToPortList(interested), lookupLatency);
}

void
SnoopFilter::updateRequest(const Packet* cpkt, const SlavePort& slave_port,
                           bool will_retry)
{
    DPRINTF(SnoopFilter, "%s: packet src %s addr 0x%x cmd %s\n",
            __func__, slave_port.name(), cpkt->getAddr(), cpkt->cmdString());

    Addr line_addr = cpkt->getAddr() & ~(linesize - 1);
    SnoopMask req_port = portToMask(slave_port);
    SnoopItem& sf_item  = cachedLocations[line_addr];

    DPRINTF(SnoopFilter, "%s:   old SF value %x.%x retry: %i\n",
            __func__, sf_item.requested, sf_item.holder, will_retry);

    if (will_retry) {
        // Unmark a request that will come again.
        sf_item.requested &= ~req_port;
        return;
    }

    // will_retry == false
    if (!cpkt->needsResponse()) {
        // Packets that will not evoke a response but still need updates of the
        // snoop filter; WRITEBACKs for now only
        if (cpkt->cmd == MemCmd::Writeback) {
            // make sure that the sender actually had the line
            panic_if(sf_item.requested & req_port, "double request :( "\
                     "SF value %x.%x\n", sf_item.requested, sf_item.holder);
            panic_if(!(sf_item.holder & req_port), "requester %x is not a "\
                     "holder :( SF value %x.%x\n", req_port,
                     sf_item.requested, sf_item.holder);
            // Writebacks -> the sender does not have the line anymore
            sf_item.holder &= ~req_port;
        } else {
            assert(0 == "Handle non-writeback, here");
        }
        DPRINTF(SnoopFilter, "%s:   new SF value %x.%x\n",
                __func__,  sf_item.requested, sf_item.holder);
    }
}

std::pair<SnoopFilter::SnoopList, Cycles>
SnoopFilter::lookupSnoop(const Packet* cpkt)
{
    DPRINTF(SnoopFilter, "%s: packet addr 0x%x cmd %s\n",
            __func__, cpkt->getAddr(), cpkt->cmdString());

    assert(cpkt->isRequest());

    // Broadcast / filter upward snoops
    const bool filter_upward = true;  // @todo: Make configurable

    if (!filter_upward)
        return snoopAll(lookupLatency);

    Addr line_addr = cpkt->getAddr() & ~(linesize - 1);
    SnoopItem& sf_item = cachedLocations[line_addr];

    DPRINTF(SnoopFilter, "%s:   old SF value %x.%x\n",
            __func__, sf_item.requested, sf_item.holder);

    SnoopMask interested = (sf_item.holder | sf_item.requested);
    assert(cpkt->isInvalidate() == cpkt->needsExclusive());
    if (cpkt->isInvalidate() && !sf_item.requested) {
        // Early clear of the holder, if no other request is currently going on
        // @todo: This should possibly be updated even though we do not filter
        // upward snoops
        sf_item.holder = 0;
    }

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

    Addr line_addr = cpkt->getAddr() & ~(linesize - 1);
    SnoopMask rsp_mask = portToMask(rsp_port);
    SnoopMask req_mask = portToMask(req_port);
    SnoopItem& sf_item = cachedLocations[line_addr];

    assert(cpkt->isResponse());
    assert(cpkt->memInhibitAsserted());

    DPRINTF(SnoopFilter, "%s:   old SF value %x.%x\n",
            __func__,  sf_item.requested, sf_item.holder);

    // The source should have the line
    panic_if(!(sf_item.holder & rsp_mask), "SF value %x.%x does not have "\
             "the line\n", sf_item.requested, sf_item.holder);

    // The destination should have had a request in
    panic_if(!(sf_item.requested & req_mask), "SF value %x.%x missing "\
             "the original request\n",  sf_item.requested, sf_item.holder);

    // Update the residency of the cache line.
    if (cpkt->needsExclusive() || !cpkt->sharedAsserted()) {
        DPRINTF(SnoopFilter, "%s:  dropping %x because needs: %i shared: %i "\
                "SF val: %x.%x\n", __func__,  rsp_mask,
                cpkt->needsExclusive(), cpkt->sharedAsserted(),
                sf_item.requested, sf_item.holder);

        sf_item.holder &= ~rsp_mask;
        // The snoop filter does not see any ACKs from non-responding sharers
        // that have been invalidated :(  So below assert would be nice, but..
        //assert(sf_item.holder == 0);
        sf_item.holder = 0;
    }
    assert(cpkt->cmd != MemCmd::Writeback);
    sf_item.holder |=  req_mask;
    sf_item.requested &= ~req_mask;
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

    Addr line_addr = cpkt->getAddr() & ~(linesize - 1);
    SnoopItem& sf_item = cachedLocations[line_addr];
    SnoopMask rsp_mask M5_VAR_USED = portToMask(rsp_port);

    assert(cpkt->isResponse());
    assert(cpkt->memInhibitAsserted());

    DPRINTF(SnoopFilter, "%s:   old SF value %x.%x\n",
            __func__,  sf_item.requested, sf_item.holder);

    // Remote (to this snoop filter) snoops update the filter already when they
    // arrive from below, because we may not see any response.
    if (cpkt->needsExclusive()) {
        // If the request to this snoop response hit an in-flight transaction,
        // the holder was not reset -> no assertion & do that here, now!
        //assert(sf_item.holder == 0);
        sf_item.holder = 0;
    }
    DPRINTF(SnoopFilter, "%s:   new SF value %x.%x\n",
            __func__, sf_item.requested, sf_item.holder);
}

void
SnoopFilter::updateResponse(const Packet* cpkt, const SlavePort& slave_port)
{
    DPRINTF(SnoopFilter, "%s: packet src %s addr 0x%x cmd %s\n",
            __func__, slave_port.name(), cpkt->getAddr(), cpkt->cmdString());

    Addr line_addr = cpkt->getAddr() & ~(linesize - 1);
    SnoopMask slave_mask = portToMask(slave_port);
    SnoopItem& sf_item = cachedLocations[line_addr];

    assert(cpkt->isResponse());

    DPRINTF(SnoopFilter, "%s:   old SF value %x.%x\n",
            __func__,  sf_item.requested, sf_item.holder);

    // Make sure we have seen the actual request, too
    panic_if(!(sf_item.requested & slave_mask), "SF value %x.%x missing "\
             "request bit\n", sf_item.requested, sf_item.holder);

    // Update the residency of the cache line.
    if (cpkt->needsExclusive() || !cpkt->sharedAsserted())
        sf_item.holder = 0;
    sf_item.holder |=  slave_mask;
    sf_item.requested &= ~slave_mask;
    DPRINTF(SnoopFilter, "%s:   new SF value %x.%x\n",
            __func__, sf_item.requested, sf_item.holder);
}

SnoopFilter *
SnoopFilterParams::create()
{
    return new SnoopFilter(this);
}
