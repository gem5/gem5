/*
 * Copyright (c) 2024 Samsung Electronics
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
 */

/**
 * @file
 * Describes a SMS prefetcher based on template policies.
 */

#include "mem/cache/prefetch/sms.hh"

#include "debug/HWPrefetch.hh"
#include "params/SmsPrefetcher.hh"

namespace gem5
{

namespace prefetch
{

Sms::Sms(const SmsPrefetcherParams &p)
    : Queued(p), Max_Contexts(p.ft_size), MAX_PHTSize(p.pht_size),
      Region_Size(p.region_size)
{
        AGT.clear();
        AGTPC.clear();
        FT.clear();
        PHT.clear();
        fifoFT.clear();
        lruAGT.clear();
        lruPHT.clear();

}
void
Sms::notifyEvict(const EvictionInfo &info)
{
    //Check if any active generation has ended
    Addr regionBase = roundDown(info.addr, Region_Size);
    std::pair <Addr,Addr> pc_offset = AGTPC[regionBase];
    if (AGT.find(regionBase) != AGT.end()) {
        //remove old recording
        if (PHT.find(pc_offset) != PHT.end()) {
            PHT[pc_offset].clear();
        }
        //Move from AGT to PHT
        for (std::set<Addr>::iterator it = AGT[regionBase].begin();
         it != AGT[regionBase].end(); it ++) {
            PHT[pc_offset].insert(*it);
        }
        lruPHT.push_front(pc_offset);
    }

    while (PHT.size() > MAX_PHTSize) {
        PHT.erase(lruPHT.back());
        lruPHT.pop_back();
    }

    AGTPC.erase(regionBase);
    AGT.erase(regionBase);


}
void
Sms::calculatePrefetch(const PrefetchInfo &pfi,
    std::vector<AddrPriority> &addresses,
    const CacheAccessor &cache)
{

    if (!pfi.hasPC()) {
        DPRINTF(HWPrefetch, "Ignoring request with no PC.\n");
        return;
    }

    Addr blk_addr = blockAddress(pfi.getAddr());
    Addr pc = pfi.getPC();
    Addr regionBase = roundDown(blk_addr, Region_Size);
    Addr offset = blk_addr - regionBase;

    //Training
    if (AGT.find(regionBase) != AGT.end()) {
        assert (FT.find(regionBase) == FT.end());
        // Record Pattern
        AGT[regionBase].insert(offset);
        //update LRU
        for (std::deque <Addr>::iterator lit = lruAGT.begin();
         lit != lruAGT.end(); lit ++) {
            if ((*lit) == regionBase) {
                lruAGT.erase(lit);
                lruAGT.push_front(regionBase);
                break;
            }
        }
    }
    else if (FT.find(regionBase) != FT.end()) {
        //move entry from FT to AGT
        AGT[regionBase].insert(FT[regionBase].second);
        AGTPC[regionBase] = FT[regionBase];
        lruAGT.push_front(regionBase);
        //Record latest offset
        AGT[regionBase].insert(offset);
        //Recycle FT entry
        FT.erase(regionBase);
        //Make space for next entry
        while (AGT.size() > Max_Contexts) {
            AGT.erase(lruAGT.back());
            AGTPC.erase(lruAGT.back());
            lruAGT.pop_back();
        }
    }
    else {
        // Trigger Access
        FT[regionBase] = std::make_pair (pc,offset);
        fifoFT.push_front(regionBase);
        while (FT.size() > Max_Contexts) {
            FT.erase(fifoFT.back());
            fifoFT.pop_back();
        }
    }

    //Prediction
    std::pair <Addr, Addr> pc_offset = std::make_pair(pc,offset);
    if (PHT.find(pc_offset) != PHT.end()) {
        for (std::set<Addr>::iterator it = PHT[pc_offset].begin();
         it != PHT[pc_offset].end(); it ++) {
            Addr prefAddr = blockAddress(regionBase + (*it));
            addresses.push_back(AddrPriority(prefAddr,0));
        }
        for (std::deque < std::pair <Addr,Addr> >::iterator lit
         = lruPHT.begin(); lit != lruPHT.end(); lit ++) {
            if ((*lit) == pc_offset) {
                    lruPHT.erase(lit);
                    lruPHT.push_front(pc_offset);
                    break;
            }
        }
    }

}

} // namespace prefetch
} // namespace gem5
