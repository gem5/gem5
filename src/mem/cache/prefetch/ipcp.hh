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
 * Describes a IPCP prefetcher .
 */

#ifndef __MEM_CACHE_PREFETCH_IPCP_HH__
#define __MEM_CACHE_PREFETCH_IPCP_HH__

#include "base/statistics.hh"
#include "mem/cache/prefetch/queued.hh"
#include "mem/packet.hh"

namespace gem5
{

struct IpcpPrefetcherParams;

namespace prefetch
{

class Ipcp : public Queued
{
  protected:
   class IpEntry
   {
       public:
           Addr ipTag = 0;
           Addr lastVpage = 0;
           Addr lastLineOffset = 0;
           Addr signature = 0;
           Addr regionAddr = 0;
           int64_t stride = 0;
           int conf = 0;
           int valid = 0;
           int dir = 1;
           bool streamValid = false;

           IpEntry() {}
           ~IpEntry() = default;
   };

   class CsptEntry
   {
       public:
           int64_t stride = 0;
           int conf = 0;
           CsptEntry() {}
           ~CsptEntry() = default;
   };

   //IPCP Table (CS + CPLX)
   std::map<Addr, IpEntry> ipcpTable;
   //Complex stride prediction table (used by CPLX)
   std::map<Addr, CsptEntry> cspt;

   //Bit widths to create bitmask
   uint64_t bitsIpIndex;
   uint64_t bitsCsptIndex;
   uint64_t bitsIpTag;
   uint64_t bitsLastVpage;
   uint64_t bitsSignature;
   const int degree;
   bool csOn, cplxOn;
   Addr getSignature(Addr curSignature, int64_t stride);

   //GS IP
   //Structure sizes
   uint64_t RM_SIZE, REGION_SIZE;
   int pageDegree;
   bool pageOn;

   class RstEntry
   {
       public:
           //Addr regionAddr;
           int dir = 32;
           bool trained = false;
           bool tentative = false;
           Tick lastTouch;
           std::set<Addr> cacheLineTouched;
           Addr lastAddr = 0;

           RstEntry() {
               lastTouch = curTick();
               cacheLineTouched.clear();
           }

           ~RstEntry() = default;
   };
   std::map<Addr, RstEntry> RST;

 //Statistics
 struct StatGroup: public statistics::Group
 {
    StatGroup(statistics::Group *parent);
    statistics::Scalar trainedVictim;
    statistics::Scalar cplxEntries;
    statistics::Scalar csChosen;
    statistics::Scalar cplxChosen;
    statistics::Scalar pageChosen;
    statistics::Scalar nlChosen;
    statistics::Scalar noPrefs;
    statistics::Scalar conflicts;
    statistics::Scalar pageCross;
 } ipcpStats;

 //Next line prefetcher
 bool nlOn;
 uint64_t misses, mpkc;

  public:
    Ipcp(const IpcpPrefetcherParams &p);
    ~Ipcp() = default;

    void calculatePrefetch(const PrefetchInfo &pfi,
                           std::vector<AddrPriority> &addresses,
                           const CacheAccessor &cache) override;
};

} // namespace prefetch
} // namespace gem5

#endif // __MEM_CACHE_PREFETCH_IPCP_HH__
