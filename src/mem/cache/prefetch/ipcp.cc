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
 * Describes a IPCP prefetcher based on template policies.
 */

#include "mem/cache/prefetch/ipcp.hh"

#include "debug/HWPrefetch.hh"
#include "params/IpcpPrefetcher.hh"

namespace gem5
{

namespace prefetch
{

Ipcp::Ipcp(const IpcpPrefetcherParams &p)
    : Queued(p), bitsIpTag(p.ip_tag_bits),
    bitsLastVpage(p.last_vpage_bits),
    bitsSignature(p.signature_bits),
    degree(p.degree), csOn(p.cs_on),
    cplxOn(p.cplx_on),
    //GS IP Prefetcher
    RM_SIZE(p.rm_size), REGION_SIZE(p.region_size),
    pageDegree(p.page_degree), pageOn(p.page_on),
    ipcpStats(this), nlOn(p.nl_on)
{
    bitsIpIndex = floorLog2(p.ip_entries);
    bitsCsptIndex = floorLog2(p.cspt_entries);

    std::cout << "bitsIpIndex: " << bitsIpIndex << std::endl;

    ipcpTable.clear();
    cspt.clear();

    //GS IP Prefetcher
    RST.clear();

    misses = 0;
    mpkc = 0;

}

Addr
Ipcp::getSignature(Addr curSignature, int64_t stride)
{
    //Handle -ve stride
    int64_t corStride = (stride < 0) ?
     (stride * (-1)
     + (1 << floorLog2(blkSize))) : stride;
    return (((curSignature << 1) ^ corStride)
     & ((1 << bitsSignature) - 1));
}

void
Ipcp::calculatePrefetch(const PrefetchInfo &pfi,
    std::vector<AddrPriority> &addresses,
    const CacheAccessor &cache)
{
    if (!pfi.hasPC()) {
        DPRINTF(HWPrefetch, "Ignoring request with no PC.\n");
        return;
    }

    ipcpStats.cplxEntries = cspt.size();

    //Calculate Misses per kilo cycles for NL
    if (pfi.isCacheMiss()) misses += 1;
    if ((ticksToCycles(curTick()) % 1000) == 0) {
        mpkc = misses/1000;
        misses = 0;
    }

    //Extract information
    Addr pc = pfi.getPC();
    Addr blkAddr = blockAddress(pfi.getAddr());
    Addr regionBase = roundDown(blkAddr, REGION_SIZE);
    Addr lineOffset = (blkAddr >> floorLog2(blkSize))
     & (pageBytes/blkSize - 1);
    //original index
    Addr ipIndex = pc & ((1 << bitsIpIndex) - 1);

    //Ideal
    //Addr ipIndex = pc;
    //Stride Index
    //const Addr hash1 = pc >> 1;
    //const Addr hash2 = hash1 >> (pc >> bitsIpIndex);
    //Addr ipIndex = (hash1 ^ hash2) & ((1 << bitsIpIndex) - 1);

    Addr ipTag = (pc >> bitsIpIndex) & ((1 << bitsIpTag) - 1);
    Addr curPage = (blkAddr >> floorLog2(pageBytes))
     & ((1 << bitsLastVpage) - 1);

    //GS IP Prefetcer
    //Training
    if (RST.find(regionBase) != RST.end()) {
        RST[regionBase].cacheLineTouched.insert(blkAddr);
        RST[regionBase].lastTouch = curTick();
        int64_t stride = blkAddr - RST[regionBase].lastAddr;
        if (stride != 0) {
            if (stride > 0)
               RST[regionBase].dir += 1;
            else RST[regionBase].dir -= 1;
        }
        if (RST[regionBase].dir > 63)
         RST[regionBase].dir = 63;
        if (RST[regionBase].dir < 0)
         RST[regionBase].dir = 0;
        RST[regionBase].lastAddr = blkAddr;

        if (RST[regionBase].cacheLineTouched.size() >
         (0.75 * (REGION_SIZE/blkSize))) {
           RST[regionBase].trained = true;
           //Update ipcpTable[ipIndex]
           for (auto &ip : ipcpTable) {
                if (ip.second.regionAddr == regionBase) {
                    ip.second.streamValid = true;
                    ip.second.dir = (RST[regionBase].dir > 32) ? 1 : -1;
                }
            }
        }
        //update region of current PC
        if (ipcpTable[ipIndex].regionAddr != regionBase) {
            ipcpTable[ipIndex].streamValid = (RST[regionBase].tentative
             || RST[regionBase].trained);
            ipcpTable[ipIndex].regionAddr = regionBase;
        }

    } else {
        RST[regionBase].lastTouch = curTick();
        RST[regionBase].cacheLineTouched.insert(blkAddr);
        RST[regionBase].dir = 32;
        RST[regionBase].trained = false;
        RST[regionBase].lastAddr = blkAddr;

        //Visit IP Table and check if previous was trained?
        if (RST.find(ipcpTable[ipIndex].regionAddr) != RST.end()) {
            RST[regionBase].tentative =
             RST[ipcpTable[ipIndex].regionAddr].trained;
        }

        ipcpTable[ipIndex].regionAddr = regionBase;
        ipcpTable[ipIndex].streamValid = (RST[regionBase].tentative
         || RST[regionBase].trained);
     }

    //Check for victim
    if (RST.size() > RM_SIZE) {
        //dummy victim
        std::map<Addr, RstEntry>::iterator victim = RST.begin();
        //Evict region that is dense, if not then LRU
        std::map<Addr, RstEntry>::iterator candidate;
        //LRU Victim
        for (candidate = RST.begin(); candidate != RST.end(); candidate++) {
           if (candidate->second.lastTouch
            < victim->second.lastTouch)
               victim = candidate;
           if (candidate->second.trained) {
               victim = candidate;
               ipcpStats.trainedVictim += 1;
               break;
           }
        }

        if (victim->second.trained) {
            for (auto &ip : ipcpTable) {
                if (ip.second.regionAddr == victim->first) {
                    ip.second.streamValid = true;
                    ip.second.dir = (victim->second.dir > 32) ? 1 : -1;
                }
            }
        }

        RST.erase(victim);
    }

   //CS, CPLX

    int64_t curStride = 0;

    //Lookup IPCP
    if (ipcpTable.find(ipIndex) != ipcpTable.end()) {
        if (ipcpTable[ipIndex].ipTag == ipTag) {
            //Hit in IPCP table
            ipcpTable[ipIndex].valid = 1;
        }
        else {
            //Check valid for taking replacement decision
            if (ipcpTable[ipIndex].valid == 0) {
                ipcpTable[ipIndex].ipTag = ipTag;
                ipcpTable[ipIndex].lastVpage = curPage;
                ipcpTable[ipIndex].lastLineOffset = lineOffset;
                ipcpTable[ipIndex].signature = 0;
                ipcpTable[ipIndex].stride = 0;
                ipcpTable[ipIndex].conf = 0;
                ipcpTable[ipIndex].valid = 1;
                ipcpTable[ipIndex].regionAddr = regionBase;
                ipcpTable[ipIndex].dir = 1;
                ipcpTable[ipIndex].streamValid = (RST[regionBase].tentative
                 || RST[regionBase].trained);
                ipcpStats.conflicts += 1;

            } else {
                ipcpTable[ipIndex].valid = 0;
            }
        return; //Stop here if conflict
        }
    } else {
        //New entry in IPCP Table
        ipcpTable[ipIndex].ipTag = ipTag;
        ipcpTable[ipIndex].lastVpage = curPage;
        ipcpTable[ipIndex].lastLineOffset = lineOffset;
        ipcpTable[ipIndex].signature = 0;
        ipcpTable[ipIndex].stride = 0;
        ipcpTable[ipIndex].conf = 0;
        //ipcpTable[ipIndex].conf = 4;
        ipcpTable[ipIndex].valid = 1;
        ipcpTable[ipIndex].regionAddr = regionBase;
        ipcpTable[ipIndex].dir = 1;
        ipcpTable[ipIndex].streamValid = (RST[regionBase].tentative
         || RST[regionBase].trained);
    }

    curStride = lineOffset - ipcpTable[ipIndex].lastLineOffset;
    // Page boundary learning
    if (ipcpTable[ipIndex].lastVpage != curPage) { //switching it off?
       if (curStride < 0) //Assuming the page moving in +ve direction
           curStride += blkSize;
       else
           curStride -= blkSize;
    }
    if (curStride == 0) return;

    if (ipcpTable[ipIndex].stride == curStride) {
        ipcpTable[ipIndex].conf += 1;
    }
    else
        // should we return from here?
        ipcpTable[ipIndex].conf -= 1;

    //Overflow fix
    if (ipcpTable[ipIndex].conf > 4) ipcpTable[ipIndex].conf = 4;
    if (ipcpTable[ipIndex].conf < 0) ipcpTable[ipIndex].conf = 0;

    //Update stride
    if (ipcpTable[ipIndex].conf == 0)
        ipcpTable[ipIndex].stride = curStride;

    //Update lastPage and lastlineOffset
    ipcpTable[ipIndex].lastVpage = curPage;
    ipcpTable[ipIndex].lastLineOffset = lineOffset;

    Addr lastSignature =  ipcpTable[ipIndex].signature;

    //Hit in CSPT
    if (cspt.find(lastSignature) != cspt.end()) {
        if (cspt[lastSignature].stride == curStride)
            cspt[lastSignature].conf += 1;
        else
            cspt[lastSignature].conf -= 1;
        //Overflow fix
        if (cspt[lastSignature].conf > 4) cspt[lastSignature].conf = 4;
        if (cspt[lastSignature].conf < 0) cspt[lastSignature].conf = 0;

        //Update stride
        if (cspt[lastSignature].conf == 0)
           cspt[lastSignature].stride = curStride;

    } else {
       cspt[lastSignature].stride = curStride; //this may not be correct
       cspt[lastSignature].conf = 0;
    }

    //update next signature
    Addr updatedSignature = getSignature(lastSignature, curStride);
    ipcpTable[ipIndex].signature = updatedSignature;

    //Prediction
    if (pageOn && ipcpTable[ipIndex].streamValid) {
        ipcpStats.pageChosen += 1;
        int64_t newStride = ipcpTable[ipIndex].dir * blkSize;
        if (regionBase != ipcpTable[ipIndex].regionAddr) {
            if (0 && !RST[regionBase].trained) {
               RST[regionBase].tentative = true;
            }
            ipcpTable[ipIndex].regionAddr = regionBase;
        }
        for (int i = 1; i <= pageDegree; i++) {
            Addr prefAddr = blockAddress(blkAddr + (i * newStride));
            addresses.push_back(AddrPriority(prefAddr,0));
            if (!samePage(blkAddr, prefAddr))
                ipcpStats.pageCross += 1;
        }
    } else if ((ipcpTable[ipIndex].conf > 1) &&
     (ipcpTable[ipIndex].stride != 0) && csOn) {
        //It is Constant stride IP
        ipcpStats.csChosen += 1;
        for (int i = 1; i <= degree; i++) {
            Addr prefAddr = blockAddress(blkAddr
             + ((i * ipcpTable[ipIndex].stride) * blkSize));
            addresses.push_back(AddrPriority(prefAddr,0));
            if (!samePage(blkAddr, prefAddr))
                ipcpStats.pageCross += 1;
        }
    } else if ((cspt[updatedSignature].conf > 0)
     && (cspt[updatedSignature].stride != 0)
     && cplxOn) {
        //It is Complex stride IP
        ipcpStats.cplxChosen += 1;
        int64_t prefOffset = 0;
        //ToDo: Add CPLX distance
        for (int i = 1; i <= degree; i++) {
            prefOffset += cspt[updatedSignature].stride;
            Addr prefAddr = blockAddress(blkAddr
             + (prefOffset * blkSize));
            if ((cspt[updatedSignature].conf > 0)
             && (prefOffset != 0)) {
                addresses.push_back(AddrPriority(prefAddr,0));
                if (!samePage(blkAddr, prefAddr))
                   ipcpStats.pageCross += 1;
            }
            updatedSignature = getSignature(updatedSignature,
             cspt[updatedSignature].stride);
            if ((cspt[updatedSignature].stride == 0) ||
             (cspt.find(updatedSignature) == cspt.end())) break;
        }
    } else if (nlOn && (mpkc < 50)) {
      //Tentative NL
      ipcpStats.nlChosen += 1;
      Addr prefAddr = blockAddress(blkAddr + blkSize);
      addresses.push_back(AddrPriority(prefAddr,0));
    } else ipcpStats.noPrefs += 1;


}
Ipcp::StatGroup::StatGroup(statistics::Group *parent)
  : statistics::Group(parent),
  ADD_STAT(trainedVictim, statistics::units::Count::get(),
        "number of times RST victim entry was trained"),
  ADD_STAT(cplxEntries, statistics::units::Count::get(),
        "number of cplx entries"),
  ADD_STAT(csChosen, statistics::units::Count::get(),
        "number of times CS component was used for prefetching"),
  ADD_STAT(cplxChosen, statistics::units::Count::get(),
        "number of times CPLX component was used for prefetching"),
  ADD_STAT(pageChosen, statistics::units::Count::get(),
        "number of times Page component was used for prefetching"),
  ADD_STAT(nlChosen, statistics::units::Count::get(),
        "number of times NL component was used for prefetching"),
  ADD_STAT(noPrefs, statistics::units::Count::get(),
        "number of times no component was chosen for prefetching"),
  ADD_STAT(conflicts, statistics::units::Count::get(),
        "number of times there was conflict"),
  ADD_STAT(pageCross, statistics::units::Count::get(),
        "number of times prefetchers were crossing page")
{
    using namespace statistics;
    trainedVictim.flags(total | nozero | nonan);
    cplxEntries.flags(total | nozero | nonan);
    csChosen.flags(total | nozero | nonan);
    cplxChosen.flags(total | nozero | nonan);
    pageChosen.flags(total | nozero | nonan);
    nlChosen.flags(total | nozero | nonan);
    noPrefs.flags(total | nozero | nonan);
    conflicts.flags(total | nozero | nonan);
    pageCross.flags(total | nozero | nonan);
}

} // namespace prefetch
} // namespace gem5
