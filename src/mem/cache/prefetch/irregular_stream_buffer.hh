/**
 * Copyright (c) 2018 Metempsy Technology Consulting
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
 * Implementation of the Irregular Stream Buffer prefetcher
 * Reference:
 *   Jain, A., & Lin, C. (2013, December). Linearizing irregular memory
 *   accesses for improved correlated prefetching. In Proceedings of the
 *   46th Annual IEEE/ACM International Symposium on Microarchitecture
 *   (pp. 247-259). ACM.
 */

#ifndef __MEM_CACHE_PREFETCH_IRREGULAR_STREAM_BUFFER_HH__
#define __MEM_CACHE_PREFETCH_IRREGULAR_STREAM_BUFFER_HH__

#include "base/callback.hh"
#include "base/sat_counter.hh"
#include "mem/cache/prefetch/associative_set.hh"
#include "mem/cache/prefetch/queued.hh"

namespace gem5
{

struct IrregularStreamBufferPrefetcherParams;

namespace prefetch
{

class IrregularStreamBuffer : public Queued
{
    /** Size in bytes of a temporal stream */
    const size_t chunkSize;
    /** Number of prefetch candidates per Physical-to-Structural entry */
    const unsigned prefetchCandidatesPerEntry;
    /** Number of maximum prefetches requests created when predicting */
    const unsigned degree;

    /**
     * Training Unit Entry datatype, it holds the last accessed address and
     * its secure flag
     */
    struct TrainingUnitEntry : public TaggedEntry
    {
        TrainingUnitEntry(TaggedIndexingPolicy *ip)
          : TaggedEntry(ip), lastAddress(0), lastAddressSecure(false)
        {}
        Addr lastAddress;
        bool lastAddressSecure;
    };
    /** Map of PCs to Training unit entries */
    AssociativeSet<TrainingUnitEntry> trainingUnit;

    /** Address Mapping entry, holds an address and a confidence counter */
    struct AddressMapping
    {
        Addr address;
        SatCounter8 counter;
        AddressMapping(unsigned bits) : address(0), counter(bits)
        {}
    };

    /**
     * Maps a set of contiguous addresses to another set of (not necessarily
     * contiguos) addresses, with their corresponding confidence counters
     */
    struct AddressMappingEntry : public TaggedEntry
    {
        std::vector<AddressMapping> mappings;
        AddressMappingEntry(size_t num_mappings, unsigned counter_bits,
                            TaggedIndexingPolicy *ip)
          : TaggedEntry(ip), mappings(num_mappings, counter_bits)
        {
        }

        void
        invalidate() override
        {
            TaggedEntry::invalidate();
            for (auto &entry : mappings) {
                entry.address = 0;
                entry.counter.reset();
            }
        }
    };

    /** Physical-to-Structured mappings table */
    AssociativeSet<AddressMappingEntry> psAddressMappingCache;
    /** Structured-to-Physical mappings table */
    AssociativeSet<AddressMappingEntry> spAddressMappingCache;
    /**
     * Counter of allocated structural addresses, increased by "chunkSize",
     * each time a new structured address is allocated
     */
    uint64_t structuralAddressCounter;

    /**
     * Add a mapping to the Structured-to-Physica mapping table
     * @param structuralAddress structural address
     * @param is_secure whether this page is inside the secure memory area
     * @param physical_address corresponding physical address
     */
    void addStructuralToPhysicalEntry(Addr structuralAddress, bool is_secure,
                                      Addr physical_address);

    /**
     * Obtain the Physical-to-Structured mapping entry of the given physical
     * address. If the entry does not exist a new one is allocated, replacing
     * an existing one if needed.
     * @param paddr physical address
     * @param is_secure whether this page is inside the secure memory area
     * @result reference to the entry
     */
    AddressMapping& getPSMapping(Addr paddr, bool is_secure);
  public:
    IrregularStreamBuffer(const IrregularStreamBufferPrefetcherParams &p);
    ~IrregularStreamBuffer() = default;

    void calculatePrefetch(const PrefetchInfo &pfi,
                           std::vector<AddrPriority> &addresses,
                           const CacheAccessor &cache) override;
};

} // namespace prefetch
} // namespace gem5

#endif//__MEM_CACHE_PREFETCH_IRREGULAR_STREAM_BUFFER_HH__
