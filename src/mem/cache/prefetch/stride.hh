/*
 * Copyright (c) 2012-2013, 2015 ARM Limited
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
 */

/**
 * @file
 * Describes a strided prefetcher.
 */

#ifndef __MEM_CACHE_PREFETCH_STRIDE_HH__
#define __MEM_CACHE_PREFETCH_STRIDE_HH__

#include <unordered_map>

#include "mem/cache/prefetch/queued.hh"
#include "params/StridePrefetcher.hh"

class StridePrefetcher : public QueuedPrefetcher
{
  protected:
    const int maxConf;
    const int threshConf;
    const int minConf;
    const int startConf;

    const int pcTableAssoc;
    const int pcTableSets;

    const bool useMasterId;

    const int degree;

    struct StrideEntry
    {
        StrideEntry() : instAddr(0), lastAddr(0), isSecure(false), stride(0),
                        confidence(0)
        { }

        Addr instAddr;
        Addr lastAddr;
        bool isSecure;
        int stride;
        int confidence;
    };

    class PCTable
    {
      public:
        PCTable(int assoc, int sets, const std::string name) :
            pcTableAssoc(assoc), pcTableSets(sets), _name(name) {}
        StrideEntry** operator[] (int context) {
            auto it = entries.find(context);
            if (it != entries.end())
                return it->second;

            return allocateNewContext(context);
        }

        ~PCTable();
      private:
        const std::string name() {return _name; }
        const int pcTableAssoc;
        const int pcTableSets;
        const std::string _name;
        std::unordered_map<int, StrideEntry**> entries;

        StrideEntry** allocateNewContext(int context);
    };
    PCTable pcTable;

    bool pcTableHit(Addr pc, bool is_secure, int master_id, StrideEntry* &entry);
    StrideEntry* pcTableVictim(Addr pc, int master_id);

    Addr pcHash(Addr pc) const;
  public:

    StridePrefetcher(const StridePrefetcherParams *p);

    void calculatePrefetch(const PacketPtr &pkt,
                           std::vector<AddrPriority> &addresses);
};

#endif // __MEM_CACHE_PREFETCH_STRIDE_HH__
