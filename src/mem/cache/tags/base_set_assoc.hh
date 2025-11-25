/*
 * Copyright (c) 2012-2014, 2017, 2023-2024 ARM Limited
 * All rights reserved.
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
 * Copyright (c) 2003-2005,2014 The Regents of The University of Michigan
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
 * Declaration of a base set associative tag store.
 */

#ifndef __MEM_CACHE_TAGS_BASE_SET_ASSOC_HH__
#define __MEM_CACHE_TAGS_BASE_SET_ASSOC_HH__

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "base/logging.hh"
#include "base/types.hh"
#include "mem/cache/base.hh"
#include "mem/cache/cache_blk.hh"
#include "mem/cache/replacement_policies/base.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"
#include "mem/cache/tags/base.hh"
#include "mem/cache/tags/indexing_policies/base.hh"
#include "mem/cache/tags/partitioning_policies/partition_manager.hh"
#include "mem/packet.hh"
#include "params/BaseSetAssoc.hh"

namespace gem5
{

/**
 * A basic cache tag store.
 * @sa  \ref gem5MemorySystem "gem5 Memory System"
 *
 * The BaseSetAssoc placement policy divides the cache into s sets of w
 * cache lines (ways).
 */
class BaseSetAssoc : public BaseTags
{
  protected:
    /** The allocatable associativity of the cache (alloc mask). */
    unsigned allocAssoc;

    /** The cache blocks. */
    std::vector<CacheBlk> blks;

    /** Whether tags and data are accessed sequentially. */
    const bool sequentialAccess;

    /** Replacement policy */
    replacement_policy::Base *replacementPolicy;

  public:
    /** Convenience typedef. */
     typedef BaseSetAssocParams Params;

    /**
     * Construct and initialize this tag store.
     */
    BaseSetAssoc(const Params &p);

    /**
     * Destructor
     */
    virtual ~BaseSetAssoc() {};

    /**
     * Initialize blocks as CacheBlk instances.
     */
    void tagsInit() override;

    /**
     * This function updates the tags when a block is invalidated. It also
     * updates the replacement data.
     *
     * @param blk The block to invalidate.
     */
    void invalidate(CacheBlk *blk) override;

    /**
     * Access block and update replacement data. May not succeed, in which case
     * nullptr is returned. This has all the implications of a cache access and
     * should only be used as such. Returns the tag lookup latency as a side
     * effect.
     *
     * @param pkt The packet holding the address to find.
     * @param lat The latency of the tag lookup.
     * @return Pointer to the cache block if found.
     */
    CacheBlk* accessBlock(const PacketPtr pkt, Cycles &lat) override
    {
        CacheBlk *blk = findBlock({pkt->getAddr(), pkt->isSecure()});

        if (blk && wayGuardTable && pkt && pkt->req && !pkt->req->isInstFetch()) {
            Addr vaddr = pkt->req->hasVaddr() ? pkt->req->getVaddr() : 0;
            bool looksLikeStack = (vaddr >= 0x7fff00000000ULL);
            if (looksLikeStack) {
                // still count this as a normal hit; do not apply DAWG mask.
                goto after_dawg_hit_mask;
            }
            const uint32_t domain = pkt->req->domainId();
            const uint32_t set    = blk->getSet();
            const uint32_t way    = blk->getWay();
            uint32_t mask         = wayGuardTable->getMask(set, domain);

            if (mask != 0 && !(mask & (1u << way))) {
        static int dawgHitMaskLogCount = 0;
        constexpr int maxDawgHitMaskLogs = 100;

                if (dawgHitMaskLogCount < maxDawgHitMaskLogs) {
                    bool inst  = pkt->req->isInstFetch();
                    Addr paddr = pkt->getAddr();
                    Addr vaddr = pkt->req->hasVaddr() ? pkt->req->getVaddr() : 0;

                    cprintf("DAWG-HIT-MASK: tick=%llu phys=0x%llx virt=0x%llx "
                            "cmd=%s inst=%d set=%u way=%u domain=%u "
                            "mask=0x%x -> miss\n",
                            curTick(), (unsigned long long)paddr,
                            (unsigned long long)vaddr,
                            pkt->cmd.toString(), inst,
                            set, way, domain, mask);

            dawgHitMaskLogCount++;
        }
                blk = nullptr;
            }
        }

after_dawg_hit_mask:

        // Access all tags in parallel, hence one in each way.  The data side
        // either accesses all blocks in parallel, or one block sequentially on
        // a hit.  Sequential access with a miss doesn't access data.
        stats.tagAccesses += allocAssoc;
        if (sequentialAccess) {
            if (blk != nullptr) {
                stats.dataAccesses += 1;
            }
        } else {
            stats.dataAccesses += allocAssoc;
        }

        // If a cache hit
        if (blk != nullptr) {
            // Update number of references to accessed block
            blk->increaseRefCount();

            // Update replacement data of accessed block
            replacementPolicy->touch(blk->replacementData, pkt);
        }

        // The tag lookup latency is the same for a hit or a miss
        lat = lookupLatency;

        return blk;
    }

    /**
     * Find replacement victim based on address. The list of evicted blocks
     * only contains the victim.
     *
     * @param addr Address to find a victim for.
     * @param is_secure True if the target memory space is secure.
     * @param size Size, in bits, of new block to allocate.
     * @param evict_blks Cache blocks to be evicted.
     * @param partition_id Partition ID for resource management.
     * @return Cache block to be replaced.
     */
    CacheBlk* findVictim(const CacheBlk::KeyType& key,
                         const std::size_t size,
                         std::vector<CacheBlk*>& evict_blks,
                         const uint64_t partition_id=0,
                         const PacketPtr pkt = nullptr) override
    {
        // Get possible entries to be victimized
        std::vector<ReplaceableEntry*> entries =
            indexingPolicy->getPossibleEntries(key);

        // Filter entries based on PartitionID
        if (partitionManager) {
            partitionManager->filterByPartition(entries, partition_id);
        }

        // if there is WayGuardTable and a packet -> apply domain mask

        // prefer domain-allowed ways but fall back to original candidates if no allowed ways exist
        std::vector<ReplaceableEntry*> filtered_entries;
        uint32_t dawg_mask = 0;
        if (wayGuardTable && pkt && pkt->req) {
            uint32_t domain = pkt->req->domainId();
            if (!entries.empty()) {
                const uint32_t set = entries.front()->getSet();
                dawg_mask = wayGuardTable->getMask(set, domain);
                // if mask is zero, treat it as "no policy" and allow all

                if (dawg_mask == 0) {
                    dawg_mask = (1u << allocAssoc) - 1;
                }
                for (auto *e : entries) {
                    uint32_t way = e->getWay();
                    if (dawg_mask & (1u << way)) {
                        filtered_entries.push_back(e);
                    } else {
                        // log if filtered by DAWG mask
                        cprintf("DAWG-FILTER: set=%u domain=%u way=%u filtered\n",
                                set, domain, way);
                        if (stats.dawgFilteredCandidatesPerDomain.size() >
                            domain) {
                            stats.dawgFilteredCandidatesPerDomain[domain]++;
                        }
                    }
                }
                // if DAWG mask filtered out every candidate, log a fallback notice
                if (filtered_entries.empty()) {
                    cprintf("DAWG-FALLBACK: set=%u domain=%u no_allowed_ways;\n"
                            " falling back to original candidates\n",
                            set, domain);
                }
            }
        }

        // Choose which candidate set to pass to replacement policy
        const std::vector<ReplaceableEntry*> &candidates =
            filtered_entries.empty() ? entries : filtered_entries;

        // Choose replacement victim from replacement candidates. Prefer
        // domain-aware API if implemented by the replacement policy.
        ReplaceableEntry *victim_entry = nullptr;
        uint32_t domain = pkt && pkt->req ? pkt->req->domainId() : 0;

        // If we filtered entries, log that domain-aware selection will be
        // performed on the reduced candidate set.
        if (!filtered_entries.empty() && !entries.empty()) {
            cprintf("DAWG-CANDIDATES: set=%u domain=%u candidates=%u\n",
                    entries.front()->getSet(), domain, filtered_entries.size());
        }

        // Try calling domain-aware API (new in replacement policies). The
        // default implementation falls back to getVictim().
        victim_entry = replacementPolicy->getVictimForDomain(candidates,
                                    domain);

        CacheBlk* victim = victim_entry ? static_cast<CacheBlk*>(victim_entry) : nullptr;

        // Debug check: if WayGuardTable was used to filter candidates,
        // ensure the replacement policy did not pick a victim way that
        // is disallowed by the DAWG mask. Log a deterministic message
        // and trigger a fatal in debug builds to catch violations.
        if (wayGuardTable && pkt && pkt->req) {
            uint32_t check_domain = pkt->req->domainId();
            if (!candidates.empty()) {
                const uint32_t check_set = candidates.front()->getSet();
                uint32_t check_mask = wayGuardTable->getMask(check_set, check_domain);
                if (check_mask == 0)
                    check_mask = (1u << allocAssoc) - 1;
                if (victim) {
                    uint32_t victim_way = victim->getWay();
                    if (!(check_mask & (1u << victim_way))) {
                        // Deterministic log for violation
                        cprintf("DAWG-ASSERT-FAIL: set=%u domain=%u victim_way=%u mask=0x%x\n",
                                check_set, check_domain, victim_way, check_mask);
                        // In debug builds this should raise immediate attention
                        panic("WayGuardTable violation: replacement selected disallowed way");
                    }
                }
            }
        }

        // There is only one eviction for this replacement
        evict_blks.push_back(victim);

        if (victim && pkt && pkt->req) {
            uint32_t install_domain = pkt->req->domainId();
            if (stats.dawgInstallsPerDomain.size() > install_domain) {
                stats.dawgInstallsPerDomain[install_domain]++;
            }
        }

        return victim;
    }

    /**
     * Insert the new block into the cache and update replacement data.
     *
     * @param pkt Packet holding the address to update
     * @param blk The block to update.
     */
    void insertBlock(const PacketPtr pkt, CacheBlk *blk) override
    {
        // Insert block
        BaseTags::insertBlock(pkt, blk);

        // Increment tag counter
        stats.tagsInUse++;

        if (partitionManager) {
            auto partition_id = partitionManager->readPacketPartitionID(pkt);
            partitionManager->notifyAcquire(partition_id);
        }

        // Update replacement policy
        replacementPolicy->reset(blk->replacementData, pkt);

        if (pkt && pkt->req) {
            uint32_t install_domain = pkt->req->domainId();
            if (stats.dawgInstallsPerDomain.size() > install_domain) {
                stats.dawgInstallsPerDomain[install_domain]++;
            }
        }
    }

    void moveBlock(CacheBlk *src_blk, CacheBlk *dest_blk) override;

    /**
     * Limit the allocation for the cache ways.
     * @param ways The maximum number of ways available for replacement.
     */
    virtual void setWayAllocationMax(int ways) override
    {
        fatal_if(ways < 1, "Allocation limit must be greater than zero");
        allocAssoc = ways;
    }

    /**
     * Get the way allocation mask limit.
     * @return The maximum number of ways available for replacement.
     */
    virtual int getWayAllocationMax() const override
    {
        return allocAssoc;
    }

    /**
     * Regenerate the block address from the tag and indexing location.
     *
     * @param block The block.
     * @return the block address.
     */
    Addr regenerateBlkAddr(const CacheBlk* blk) const override
    {
        return indexingPolicy->regenerateAddr({blk->getTag(), false}, blk);
    }

    bool anyBlk(std::function<bool(CacheBlk &)> visitor) override {
        for (CacheBlk& blk : blks) {
            if (visitor(blk)) {
                return true;
            }
        }
        return false;
    }
};

} // namespace gem5

#endif //__MEM_CACHE_TAGS_BASE_SET_ASSOC_HH__
