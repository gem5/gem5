/*
 * Copyright (c) 2018 Inria
 * Copyright (c) 2012-2013, 2015, 2022 Arm Limited
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

#include "base/sat_counter.hh"
#include "base/types.hh"
#include "mem/cache/prefetch/associative_set.hh"
#include "mem/cache/prefetch/queued.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"
#include "mem/cache/tags/indexing_policies/set_associative.hh"
#include "mem/packet.hh"
#include "params/StridePrefetcherHashedSetAssociative.hh"

namespace gem5
{

class BaseIndexingPolicy;
namespace replacement_policy
{
    class Base;
}
struct StridePrefetcherParams;

namespace prefetch
{

/**
 * Override the default set associative to apply a specific hash function
 * when extracting a set.
 */
class StridePrefetcherHashedSetAssociative : public SetAssociative
{
  protected:
    uint32_t extractSet(const Addr addr) const override;
    Addr extractTag(const Addr addr) const override;

  public:
    StridePrefetcherHashedSetAssociative(
        const StridePrefetcherHashedSetAssociativeParams &p)
      : SetAssociative(p)
    {
    }
    ~StridePrefetcherHashedSetAssociative() = default;
};

class Stride : public Queued
{
  protected:
    /** Initial confidence counter value for the pc tables. */
    const SatCounter8 initConfidence;

    /** Confidence threshold for prefetch generation. */
    const double threshConf;

    const bool useRequestorId;

    const int degree;

    /** How far ahead of the demand stream to start prefetching.
     *
     * Skip this number of strides ahead of the first identified
     * prefetch, then generate `degree` prefetches at `stride`
     * intervals. A value of zero indicates no skip.
     */
    const int distance;

    /**
     * Information used to create a new PC table. All of them behave equally.
     */
    const struct PCTableInfo
    {
        const int assoc;
        const int numEntries;

        BaseIndexingPolicy* const indexingPolicy;
        replacement_policy::Base* const replacementPolicy;

        PCTableInfo(int assoc, int num_entries,
                    BaseIndexingPolicy* indexing_policy,
                    replacement_policy::Base* repl_policy)
          : assoc(assoc), numEntries(num_entries),
            indexingPolicy(indexing_policy), replacementPolicy(repl_policy)
        {
        }
    } pcTableInfo;

    /** Tagged by hashed PCs. */
    struct StrideEntry : public TaggedEntry
    {
        StrideEntry(const SatCounter8& init_confidence);

        void invalidate() override;

        Addr lastAddr;
        int stride;
        SatCounter8 confidence;
    };
    typedef AssociativeSet<StrideEntry> PCTable;
    std::unordered_map<int, PCTable> pcTables;

    /**
     * If this parameter is set to true, then the prefetcher will operate at
     * the granularity of cache line. Otherwise it would operate on the
     * granularity of word addresses
     */
    const bool useCachelineAddr;

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
    Stride(const StridePrefetcherParams &p);

    void calculatePrefetch(const PrefetchInfo &pfi,
                           std::vector<AddrPriority> &addresses,
                           const CacheAccessor &cache) override;
};

} // namespace prefetch
} // namespace gem5

#endif // __MEM_CACHE_PREFETCH_STRIDE_HH__
