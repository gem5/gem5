/*
 * Copyright (c) 2018 Inria
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
 *          Daniel Carvalho
 */

/**
 * @file
 * Describes a strided prefetcher.
 */

#ifndef __MEM_CACHE_PREFETCH_STRIDE_HH__
#define __MEM_CACHE_PREFETCH_STRIDE_HH__

#include <string>
#include <unordered_map>
#include <vector>

#include "base/types.hh"
#include "mem/cache/prefetch/queued.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"
#include "mem/packet.hh"

class BaseReplacementPolicy;
struct StridePrefetcherParams;

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

    /** Replacement policy used in the PC tables. */
    BaseReplacementPolicy* replacementPolicy;

    struct StrideEntry : public ReplaceableEntry
    {
        /** Default constructor */
        StrideEntry();

        /** Invalidate the entry */
        void invalidate();

        Addr instAddr;
        Addr lastAddr;
        bool isSecure;
        int stride;
        int confidence;
    };

    class PCTable
    {
      public:
        /**
         * Default constructor. Create a table with given parameters.
         *
         * @param assoc Associativity of the table.
         * @param sets Number of sets in the table.
         * @param name Name of the prefetcher.
         * @param replacementPolicy Replacement policy used by the table.
         */
        PCTable(int assoc, int sets, const std::string name,
                BaseReplacementPolicy* replacementPolicy);

        /**
         * Default destructor.
         */
        ~PCTable();

        /**
         * Search for an entry in the pc table.
         *
         * @param pc The PC to look for.
         * @param is_secure True if the target memory space is secure.
         * @return Pointer to the entry.
         */
        StrideEntry* findEntry(Addr pc, bool is_secure);

        /**
         * Find a replacement victim to make room for given PC.
         *
         * @param pc The PC value.
         * @return The victimized entry.
         */
        StrideEntry* findVictim(Addr pc);

      private:
        const std::string name() {return _name; }
        const int pcTableSets;
        const std::string _name;
        std::vector<std::vector<StrideEntry>> entries;

        /**
         * Replacement policy used by StridePrefetcher.
         */
        BaseReplacementPolicy* replacementPolicy;

        /**
         * PC hashing function to index sets in the table.
         *
         * @param pc The PC value.
         * @return The set to which this PC maps.
         */
        Addr pcHash(Addr pc) const;
    };
    std::unordered_map<int, PCTable> pcTables;

    /**
     * Try to find a table of entries for the given context. If none is
     * found, a new table is created.
     *
     * @param context The context to be searched for.
     * @return The table corresponding to the given context.
     */
    PCTable* findTable(int context);

    /**
     * Create a PC table for the given context.
     *
     * @param context The context of the new PC table.
     * @return The new PC table
     */
    PCTable* allocateNewContext(int context);

  public:
    StridePrefetcher(const StridePrefetcherParams *p);

    void calculatePrefetch(const PrefetchInfo &pfi,
                           std::vector<AddrPriority> &addresses) override;
};

#endif // __MEM_CACHE_PREFETCH_STRIDE_HH__
