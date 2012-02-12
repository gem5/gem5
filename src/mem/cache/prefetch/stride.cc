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
                                    std::list<Tick> &delays)
{
    if (!pkt->req->hasPC()) {
        DPRINTF(HWPrefetch, "ignoring request with no PC");
        return;
    }

    Addr blk_addr = pkt->getAddr() & ~(Addr)(blkSize-1);
    MasterID master_id = useMasterId ? pkt->req->masterId() : 0;
    Addr pc = pkt->req->getPC();
    assert(master_id < Max_Contexts);
    std::list<StrideEntry*> &tab = table[master_id];

    /* Scan Table for instAddr Match */
    std::list<StrideEntry*>::iterator iter;
    for (iter = tab.begin(); iter != tab.end(); iter++) {
        if ((*iter)->instAddr == pc)
            break;
    }

    if (iter != tab.end()) {
        // Hit in table

        int new_stride = blk_addr - (*iter)->missAddr;
        bool stride_match = (new_stride == (*iter)->stride);

        if (stride_match && new_stride != 0) {
            if ((*iter)->confidence < Max_Conf)
                (*iter)->confidence++;
        } else {
            (*iter)->stride = new_stride;
            if ((*iter)->confidence > Min_Conf)
                (*iter)->confidence = 0;
        }

        DPRINTF(HWPrefetch, "hit: PC %x blk_addr %x stride %d (%s), conf %d\n",
                pc, blk_addr, new_stride, stride_match ? "match" : "change",
                (*iter)->confidence);

        (*iter)->missAddr = blk_addr;

        if ((*iter)->confidence <= 0)
            return;

        for (int d = 1; d <= degree; d++) {
            Addr new_addr = blk_addr + d * new_stride;
            if (pageStop && !samePage(blk_addr, new_addr)) {
                // Spanned the page, so now stop
                pfSpanPage += degree - d + 1;
                return;
            } else {
                DPRINTF(HWPrefetch, "  queuing prefetch to %x @ %d\n",
                        new_addr, latency);
                addresses.push_back(new_addr);
                delays.push_back(latency);
            }
        }
    } else {
        // Miss in table
        // Find lowest confidence and replace

        DPRINTF(HWPrefetch, "miss: PC %x blk_addr %x\n", pc, blk_addr);

        if (tab.size() >= 256) { //set default table size is 256
            std::list<StrideEntry*>::iterator min_pos = tab.begin();
            int min_conf = (*min_pos)->confidence;
            for (iter = min_pos, ++iter; iter != tab.end(); ++iter) {
                if ((*iter)->confidence < min_conf){
                    min_pos = iter;
                    min_conf = (*iter)->confidence;
                }
            }
            DPRINTF(HWPrefetch, "  replacing PC %x\n", (*min_pos)->instAddr);

            // free entry and delete it
            delete *min_pos;
            tab.erase(min_pos);
        }

        StrideEntry *new_entry = new StrideEntry;
        new_entry->instAddr = pc;
        new_entry->missAddr = blk_addr;
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
