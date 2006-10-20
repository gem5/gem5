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
 */

/**
 * @file
 * Describes a strided prefetcher based on template policies.
 */

#ifndef __MEM_CACHE_PREFETCH_STRIDE_PREFETCHER_HH__
#define __MEM_CACHE_PREFETCH_STRIDE_PREFETCHER_HH__

#include "base/misc.hh" // fatal, panic, and warn

#include "mem/cache/prefetch/prefetcher.hh"

/**
 * A template-policy based cache. The behavior of the cache can be altered by
 * supplying different template policies. TagStore handles all tag and data
 * storage @sa TagStore. Buffering handles all misses and writes/writebacks
 * @sa MissQueue. Coherence handles all coherence policy details @sa
 * UniCoherence, SimpleMultiCoherence.
 */
template <class TagStore, class Buffering>
class StridePrefetcher : public Prefetcher<TagStore, Buffering>
{
  protected:

    Buffering* mq;
    TagStore* tags;

    class strideEntry
    {
      public:
        Addr IAddr;
        Addr MAddr;
        int stride;
        int64_t confidence;

/*	bool operator < (strideEntry a,strideEntry b)
        {
            if (a.confidence == b.confidence) {
                return true; //??????
            }
            else return a.confidence < b.confidence;
            }*/
    };
    Addr* lastMissAddr[64/*MAX_CPUS*/];

    std::list<strideEntry*> table[64/*MAX_CPUS*/];
    Tick latency;
    int degree;
    bool useCPUId;


  public:

    StridePrefetcher(int size, bool pageStop, bool serialSquash,
                     bool cacheCheckPush, bool onlyData,
                     Tick latency, int degree, bool useCPUId)
        :Prefetcher<TagStore, Buffering>(size, pageStop, serialSquash,
                                         cacheCheckPush, onlyData),
         latency(latency), degree(degree), useCPUId(useCPUId)
    {
    }

    ~StridePrefetcher() {}

    void calculatePrefetch(PacketPtr &pkt, std::list<Addr> &addresses,
                           std::list<Tick> &delays)
    {
//	Addr blkAddr = pkt->paddr & ~(Addr)(this->blkSize-1);
        int cpuID = pkt->req->getCpuNum();
        if (!useCPUId) cpuID = 0;

        /* Scan Table for IAddr Match */
/*	std::list<strideEntry*>::iterator iter;
        for (iter=table[cpuID].begin();
             iter !=table[cpuID].end();
             iter++) {
            if ((*iter)->IAddr == pkt->pc) break;
        }

        if (iter != table[cpuID].end()) {
            //Hit in table

            int newStride = blkAddr - (*iter)->MAddr;
            if (newStride == (*iter)->stride) {
                (*iter)->confidence++;
            }
            else {
                (*iter)->stride = newStride;
                (*iter)->confidence--;
            }

            (*iter)->MAddr = blkAddr;

            for (int d=1; d <= degree; d++) {
                Addr newAddr = blkAddr + d * newStride;
                if (this->pageStop &&
                    (blkAddr & ~(TheISA::VMPageSize - 1)) !=
                    (newAddr & ~(TheISA::VMPageSize - 1)))
                {
                    //Spanned the page, so now stop
                    this->pfSpanPage += degree - d + 1;
                    return;
                }
                else
                {
                    addresses.push_back(newAddr);
                    delays.push_back(latency);
                }
            }
        }
        else {
            //Miss in table
            //Find lowest confidence and replace

        }
*/    }
};

#endif // __MEM_CACHE_PREFETCH_STRIDE_PREFETCHER_HH__
