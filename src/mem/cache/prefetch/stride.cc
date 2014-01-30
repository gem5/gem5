/*
 * Copyright (c) 2012-2013 ARM Limited
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
 *          Steve Reinhardt
 */

/**
 * @file
 * Stride Prefetcher template instantiations.
 */

#include "base/trace.hh"
#include "debug/HWPrefetch.hh"
#include "mem/cache/prefetch/stride.hh"

void
StridePrefetcher::calculatePrefetch(PacketPtr &pkt, std::list<Addr> &addresses,
                                    std::list<Cycles> &delays)
{
    if (!pkt->req->hasPC()) {
        DPRINTF(HWPrefetch, "ignoring request with no PC");
        return;
    }

    Addr data_addr = pkt->getAddr();
    bool is_secure = pkt->isSecure();
    MasterID master_id = useMasterId ? pkt->req->masterId() : 0;
    Addr pc = pkt->req->getPC();
    assert(master_id < Max_Contexts);
    std::list<StrideEntry*> &tab = table[master_id];

    // Revert to simple N-block ahead prefetch for instruction fetches
    if (instTagged && pkt->req->isInstFetch()) {
        for (int d = 1; d <= degree; d++) {
            Addr new_addr = data_addr + d * blkSize;
            if (pageStop && !samePage(data_addr, new_addr)) {
                // Spanned the page, so now stop
                pfSpanPage += degree - d + 1;
                return;
            }
            DPRINTF(HWPrefetch, "queuing prefetch to %x @ %d\n",
                    new_addr, latency);
            addresses.push_back(new_addr);
            delays.push_back(latency);
        }
        return;
    }

    /* Scan Table for instAddr Match */
    std::list<StrideEntry*>::iterator iter;
    for (iter = tab.begin(); iter != tab.end(); iter++) {
        // Entries have to match on the security state as well
        if ((*iter)->instAddr == pc && (*iter)->isSecure == is_secure)
            break;
    }

    if (iter != tab.end()) {
        // Hit in table

        int new_stride = data_addr - (*iter)->missAddr;
        bool stride_match = (new_stride == (*iter)->stride);

        if (stride_match && new_stride != 0) {
            if ((*iter)->confidence < Max_Conf)
                (*iter)->confidence++;
        } else {
            (*iter)->stride = new_stride;
            if ((*iter)->confidence > Min_Conf)
                (*iter)->confidence = 0;
        }

        DPRINTF(HWPrefetch, "hit: PC %x data_addr %x (%s) stride %d (%s), "
                "conf %d\n", pc, data_addr, is_secure ? "s" : "ns", new_stride,
                stride_match ? "match" : "change",
                (*iter)->confidence);

        (*iter)->missAddr = data_addr;
        (*iter)->isSecure = is_secure;

        if ((*iter)->confidence <= 0)
            return;

        for (int d = 1; d <= degree; d++) {
            Addr new_addr = data_addr + d * new_stride;
            if (pageStop && !samePage(data_addr, new_addr)) {
                // Spanned the page, so now stop
                pfSpanPage += degree - d + 1;
                return;
            } else {
                DPRINTF(HWPrefetch, "  queuing prefetch to %x (%s) @ %d\n",
                        new_addr, is_secure ? "s" : "ns", latency);
                addresses.push_back(new_addr);
                delays.push_back(latency);
            }
        }
    } else {
        // Miss in table
        // Find lowest confidence and replace

        DPRINTF(HWPrefetch, "miss: PC %x data_addr %x (%s)\n", pc, data_addr,
                is_secure ? "s" : "ns");

        if (tab.size() >= 256) { //set default table size is 256
            std::list<StrideEntry*>::iterator min_pos = tab.begin();
            int min_conf = (*min_pos)->confidence;
            for (iter = min_pos, ++iter; iter != tab.end(); ++iter) {
                if ((*iter)->confidence < min_conf){
                    min_pos = iter;
                    min_conf = (*iter)->confidence;
                }
            }
            DPRINTF(HWPrefetch, "  replacing PC %x (%s)\n",
                    (*min_pos)->instAddr, (*min_pos)->isSecure ? "s" : "ns");

            // free entry and delete it
            delete *min_pos;
            tab.erase(min_pos);
        }

        StrideEntry *new_entry = new StrideEntry;
        new_entry->instAddr = pc;
        new_entry->missAddr = data_addr;
        new_entry->isSecure = is_secure;
        new_entry->stride = 0;
        new_entry->confidence = 0;
        tab.push_back(new_entry);
    }
}


StridePrefetcher*
StridePrefetcherParams::create()
{
   return new StridePrefetcher(this);
}
