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
 * GHB Prefetcher implementation.
 */

#include "base/trace.hh"
#include "debug/HWPrefetch.hh"
#include "mem/cache/prefetch/ghb.hh"

void
GHBPrefetcher::calculatePrefetch(PacketPtr &pkt, std::list<Addr> &addresses,
                                 std::list<Tick> &delays)
{
    Addr blk_addr = pkt->getAddr() & ~(Addr)(blkSize-1);
    int master_id = useMasterId ? pkt->req->masterId() : 0;
    assert(master_id < Max_Masters);

    int new_stride = blk_addr - lastMissAddr[master_id];
    int old_stride = lastMissAddr[master_id] - secondLastMissAddr[master_id];

    secondLastMissAddr[master_id] = lastMissAddr[master_id];
    lastMissAddr[master_id] = blk_addr;

    if (new_stride == old_stride) {
        for (int d = 1; d <= degree; d++) {
            Addr new_addr = blk_addr + d * new_stride;
            if (pageStop && !samePage(blk_addr, new_addr)) {
                // Spanned the page, so now stop
                pfSpanPage += degree - d + 1;
                return;
            } else {
                addresses.push_back(new_addr);
                delays.push_back(latency);
            }
        }
    }
}


GHBPrefetcher*
GHBPrefetcherParams::create()
{
   return new GHBPrefetcher(this);
}
