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

#include "mem/cache/prefetch/irregular_stream_buffer.hh"

#include "debug/HWPrefetch.hh"
#include "mem/cache/prefetch/associative_set_impl.hh"
#include "params/IrregularStreamBufferPrefetcher.hh"

namespace gem5
{

namespace prefetch
{

IrregularStreamBuffer::IrregularStreamBuffer(
    const IrregularStreamBufferPrefetcherParams &p)
  : Queued(p),
    chunkSize(p.chunk_size),
    prefetchCandidatesPerEntry(p.prefetch_candidates_per_entry),
    degree(p.degree),
    trainingUnit((name() + ".TrainingUnit").c_str(),
                 p.training_unit_entries,
                 p.training_unit_assoc,
                 p.training_unit_replacement_policy,
                 p.training_unit_indexing_policy,
                 TrainingUnitEntry(p.training_unit_indexing_policy)),
    psAddressMappingCache((name() + ".PSAddressMappingCache").c_str(),
        p.address_map_cache_entries,
        p.address_map_cache_assoc,
        p.ps_address_map_cache_replacement_policy,
        p.ps_address_map_cache_indexing_policy,
        AddressMappingEntry(prefetchCandidatesPerEntry,
            p.num_counter_bits, p.ps_address_map_cache_indexing_policy)),
    spAddressMappingCache((name() + ".SPAddressMappingCache").c_str(),
        p.address_map_cache_entries,
        p.address_map_cache_assoc,
        p.sp_address_map_cache_replacement_policy,
        p.sp_address_map_cache_indexing_policy,
        AddressMappingEntry(prefetchCandidatesPerEntry,
            p.num_counter_bits, p.sp_address_map_cache_indexing_policy)),
    structuralAddressCounter(0)
{
    assert(isPowerOf2(prefetchCandidatesPerEntry));
}

void
IrregularStreamBuffer::calculatePrefetch(const PrefetchInfo &pfi,
    std::vector<AddrPriority> &addresses,
    const CacheAccessor &cache)
{
    // This prefetcher requires a PC
    if (!pfi.hasPC()) {
        return;
    }
    bool is_secure = pfi.isSecure();
    Addr pc = pfi.getPC();
    Addr addr = blockIndex(pfi.getAddr());

    // Training, if the entry exists, then we found a correlation between
    // the entry lastAddress (named as correlated_addr_A) and the address of
    // the current access (named as correlated_addr_B)
    TrainingUnitEntry *entry = trainingUnit.findEntry(pc, is_secure);
    bool correlated_addr_found = false;
    Addr correlated_addr_A = 0;
    Addr correlated_addr_B = 0;
    if (entry != nullptr && entry->lastAddressSecure == is_secure) {
        trainingUnit.accessEntry(entry);
        correlated_addr_found = true;
        correlated_addr_A = entry->lastAddress;
        correlated_addr_B = addr;
    } else {
        entry = trainingUnit.findVictim(pc);
        assert(entry != nullptr);

        trainingUnit.insertEntry(pc, is_secure, entry);
    }
    // Update the entry
    entry->lastAddress = addr;
    entry->lastAddressSecure = is_secure;

    if (correlated_addr_found) {
        // If a correlation was found, update the Physical-to-Structural
        // table accordingly
        AddressMapping &mapping_A = getPSMapping(correlated_addr_A, is_secure);
        AddressMapping &mapping_B = getPSMapping(correlated_addr_B, is_secure);
        if (mapping_A.counter > 0 && mapping_B.counter > 0) {
            // Entry for A and B
            if (mapping_B.address == (mapping_A.address + 1)) {
                mapping_B.counter++;
            } else {
                if (mapping_B.counter == 1) {
                    // Counter would hit 0, reassign address while keeping
                    // counter at 1
                    mapping_B.address = mapping_A.address + 1;
                    addStructuralToPhysicalEntry(mapping_B.address, is_secure,
                            correlated_addr_B);
                } else {
                    mapping_B.counter--;
                }
            }
        } else {
            if (mapping_A.counter == 0) {
                // if A is not valid, generate a new structural address
                mapping_A.counter++;
                mapping_A.address = structuralAddressCounter;
                structuralAddressCounter += chunkSize;
                addStructuralToPhysicalEntry(mapping_A.address,
                        is_secure, correlated_addr_A);
            }
            mapping_B.counter.reset();
            mapping_B.counter++;
            mapping_B.address = mapping_A.address + 1;
            // update SP-AMC
            addStructuralToPhysicalEntry(mapping_B.address, is_secure,
                    correlated_addr_B);
        }
    }

    // Use the PS mapping to predict future accesses using the current address
    // - Look for the structured address
    // - if it exists, use it to generate prefetches for the subsequent
    //   addresses in ascending order, as many as indicated by the degree
    //   (given the structured address S, prefetch S+1, S+2, .. up to S+degree)
    Addr amc_address = addr / prefetchCandidatesPerEntry;
    Addr map_index   = addr % prefetchCandidatesPerEntry;
    AddressMappingEntry *ps_am = psAddressMappingCache.findEntry(amc_address,
                                                                 is_secure);
    if (ps_am != nullptr) {
        AddressMapping &mapping = ps_am->mappings[map_index];
        if (mapping.counter > 0) {
            Addr sp_address = mapping.address / prefetchCandidatesPerEntry;
            Addr sp_index   = mapping.address % prefetchCandidatesPerEntry;
            AddressMappingEntry *sp_am =
                spAddressMappingCache.findEntry(sp_address, is_secure);
            if (sp_am == nullptr) {
                // The entry has been evicted, can not generate prefetches
                return;
            }
            for (unsigned d = 1;
                    d <= degree && (sp_index + d) < prefetchCandidatesPerEntry;
                    d += 1)
            {
                AddressMapping &spm = sp_am->mappings[sp_index + d];
                //generate prefetch
                if (spm.counter > 0) {
                    Addr pf_addr = spm.address << lBlkSize;
                    addresses.push_back(AddrPriority(pf_addr, 0));
                }
            }
        }
    }
}

IrregularStreamBuffer::AddressMapping&
IrregularStreamBuffer::getPSMapping(Addr paddr, bool is_secure)
{
    Addr amc_address = paddr / prefetchCandidatesPerEntry;
    Addr map_index   = paddr % prefetchCandidatesPerEntry;
    AddressMappingEntry *ps_entry =
        psAddressMappingCache.findEntry(amc_address, is_secure);
    if (ps_entry != nullptr) {
        // A PS-AMC line already exists
        psAddressMappingCache.accessEntry(ps_entry);
    } else {
        ps_entry = psAddressMappingCache.findVictim(amc_address);
        assert(ps_entry != nullptr);

        psAddressMappingCache.insertEntry(amc_address, is_secure, ps_entry);
    }
    return ps_entry->mappings[map_index];
}

void
IrregularStreamBuffer::addStructuralToPhysicalEntry(
    Addr structural_address, bool is_secure, Addr physical_address)
{
    Addr amc_address = structural_address / prefetchCandidatesPerEntry;
    Addr map_index   = structural_address % prefetchCandidatesPerEntry;
    AddressMappingEntry *sp_entry =
        spAddressMappingCache.findEntry(amc_address, is_secure);
    if (sp_entry != nullptr) {
        spAddressMappingCache.accessEntry(sp_entry);
    } else {
        sp_entry = spAddressMappingCache.findVictim(amc_address);
        assert(sp_entry != nullptr);

        spAddressMappingCache.insertEntry(amc_address, is_secure, sp_entry);
    }
    AddressMapping &mapping = sp_entry->mappings[map_index];
    mapping.address = physical_address;
    mapping.counter.reset();
    mapping.counter++;
}

} // namespace prefetch
} // namespace gem5
