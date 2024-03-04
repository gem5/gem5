/*
 * Copyright (c) 2023-2024 ARM Limited
 * Copyright (c) 2018 Inria
 * Copyright (c) 2013,2016-2018 ARM Limited
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
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 * Definitions a fully associative LRU tagstore.
 */

#include "mem/cache/tags/fa_lru.hh"

#include <cassert>
#include <sstream>

#include "base/compiler.hh"
#include "base/intmath.hh"
#include "base/logging.hh"
#include "mem/cache/base.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"

namespace gem5
{

std::string
FALRUBlk::print() const
{
    return csprintf("%s inCachesMask: %#x", CacheBlk::print(), inCachesMask);
}

FALRU::FALRU(const Params &p)
    : BaseTags(p),

      cacheTracking(p.min_tracked_cache_size, size, blkSize, this)
{
    if (!isPowerOf2(blkSize))
        fatal("cache block size (in bytes) `%d' must be a power of two",
              blkSize);
    if (!isPowerOf2(size))
        fatal("Cache Size must be power of 2 for now");
    if (this->partitioningPolicies.size() != 0)
        fatal("Cannot use Cache Partitioning Policies with FALRU");

    blks = new FALRUBlk[numBlocks];
}

FALRU::~FALRU()
{
    delete[] blks;
}

void
FALRU::tagsInit()
{
    head = &(blks[0]);
    head->prev = nullptr;
    head->next = &(blks[1]);
    head->setPosition(0, 0);
    head->data = &dataBlks[0];

    for (unsigned i = 1; i < numBlocks - 1; i++) {
        blks[i].prev = &(blks[i-1]);
        blks[i].next = &(blks[i+1]);
        blks[i].setPosition(0, i);

        // Associate a data chunk to the block
        blks[i].data = &dataBlks[blkSize*i];
    }

    tail = &(blks[numBlocks - 1]);
    tail->prev = &(blks[numBlocks - 2]);
    tail->next = nullptr;
    tail->setPosition(0, numBlocks - 1);
    tail->data = &dataBlks[(numBlocks - 1) * blkSize];

    cacheTracking.init(head, tail);
}

void
FALRU::invalidate(CacheBlk *blk)
{
    // Erase block entry reference in the hash table
    [[maybe_unused]] auto num_erased =
        tagHash.erase(std::make_pair(blk->getTag(), blk->isSecure()));

    // Sanity check; only one block reference should be erased
    assert(num_erased == 1);

    // Invalidate block entry. Must be done after the hash is erased
    BaseTags::invalidate(blk);

    // Decrease the number of tags in use
    stats.tagsInUse--;

    // Move the block to the tail to make it the next victim
    moveToTail((FALRUBlk*)blk);
}

CacheBlk*
FALRU::accessBlock(const PacketPtr pkt, Cycles &lat)
{
    return accessBlock(pkt, lat, 0);
}

CacheBlk*
FALRU::accessBlock(const PacketPtr pkt, Cycles &lat,
                   CachesMask *in_caches_mask)
{
    CachesMask mask = 0;
    FALRUBlk* blk =
        static_cast<FALRUBlk*>(findBlock(pkt->getAddr(), pkt->isSecure()));

    // If a cache hit
    if (blk && blk->isValid()) {
        mask = blk->inCachesMask;

        moveToHead(blk);
    }

    if (in_caches_mask) {
        *in_caches_mask = mask;
    }

    cacheTracking.recordAccess(blk);

    // The tag lookup latency is the same for a hit or a miss
    lat = lookupLatency;

    return blk;
}

CacheBlk*
FALRU::findBlock(Addr addr, bool is_secure) const
{
    FALRUBlk* blk = nullptr;

    Addr tag = extractTag(addr);
    auto iter = tagHash.find(std::make_pair(tag, is_secure));
    if (iter != tagHash.end()) {
        blk = (*iter).second;
    }

    if (blk && blk->isValid()) {
        assert(blk->getTag() == tag);
        assert(blk->isSecure() == is_secure);
    }

    return blk;
}

ReplaceableEntry*
FALRU::findBlockBySetAndWay(int set, int way) const
{
    assert(set == 0);
    return &blks[way];
}

CacheBlk*
FALRU::findVictim(Addr addr, const bool is_secure, const std::size_t size,
                  std::vector<CacheBlk*>& evict_blks,
                  const uint64_t partition_id)
{
    // The victim is always stored on the tail for the FALRU
    FALRUBlk* victim = tail;

    // There is only one eviction for this replacement
    evict_blks.push_back(victim);

    return victim;
}

void
FALRU::insertBlock(const PacketPtr pkt, CacheBlk *blk)
{
    FALRUBlk* falruBlk = static_cast<FALRUBlk*>(blk);

    // Make sure block is not present in the cache
    assert(falruBlk->inCachesMask == 0);

    // Do common block insertion functionality
    BaseTags::insertBlock(pkt, blk);

    // Increment tag counter
    stats.tagsInUse++;

    // New block is the MRU
    moveToHead(falruBlk);

    // Insert new block in the hash table
    tagHash[std::make_pair(blk->getTag(), blk->isSecure())] = falruBlk;
}

void
FALRU::moveBlock(CacheBlk *src_blk, CacheBlk *dest_blk)
{
    panic("Moving blocks in FALRU has not been implemented");
}

void
FALRU::moveToHead(FALRUBlk *blk)
{
    // If block is not already head, do the moving
    if (blk != head) {
        cacheTracking.moveBlockToHead(blk);
        // If block is tail, set previous block as new tail
        if (blk == tail){
            assert(blk->next == nullptr);
            tail = blk->prev;
            tail->next = nullptr;
        // Inform block's surrounding blocks that it has been moved
        } else {
            blk->prev->next = blk->next;
            blk->next->prev = blk->prev;
        }

        // Swap pointers
        blk->next = head;
        blk->prev = nullptr;
        head->prev = blk;
        head = blk;

        cacheTracking.check(head, tail);
    }
}

void
FALRU::moveToTail(FALRUBlk *blk)
{
    // If block is not already tail, do the moving
    if (blk != tail) {
        cacheTracking.moveBlockToTail(blk);
        // If block is head, set next block as new head
        if (blk == head){
            assert(blk->prev == nullptr);
            head = blk->next;
            head->prev = nullptr;
        // Inform block's surrounding blocks that it has been moved
        } else {
            blk->prev->next = blk->next;
            blk->next->prev = blk->prev;
        }

        // Swap pointers
        blk->prev = tail;
        blk->next = nullptr;
        tail->next = blk;
        tail = blk;

        cacheTracking.check(head, tail);
    }
}

void
printSize(std::ostream &stream, size_t size)
{
    static const char *SIZES[] = { "B", "kB", "MB", "GB", "TB", "ZB" };
    int div = 0;
    while (size >= 1024 && div < (sizeof SIZES / sizeof *SIZES)) {
        div++;
        size >>= 10;
    }
    stream << size << SIZES[div];
}

FALRU::CacheTracking::CacheTracking(unsigned min_size, unsigned max_size,
    unsigned block_size, statistics::Group *parent)
    : statistics::Group(parent),
      blkSize(block_size),
      minTrackedSize(min_size),
      numTrackedCaches(max_size > min_size ?
                       floorLog2(max_size) - floorLog2(min_size) : 0),
      inAllCachesMask(mask(numTrackedCaches)),
      boundaries(numTrackedCaches),
      ADD_STAT(hits, statistics::units::Count::get(),
               "The number of hits in each cache size."),
      ADD_STAT(misses, statistics::units::Count::get(),
               "The number of misses in each cache size."),
      ADD_STAT(accesses, statistics::units::Count::get(),
               "The number of accesses to the FA LRU cache.")
{
    fatal_if(numTrackedCaches > sizeof(CachesMask) * 8,
             "Not enough bits (%s) in type CachesMask type to keep "
             "track of %d caches\n", sizeof(CachesMask),
             numTrackedCaches);

    hits
        .init(numTrackedCaches + 1);
    misses
        .init(numTrackedCaches + 1);

    for (unsigned i = 0; i < numTrackedCaches + 1; ++i) {
      std::stringstream size_str;
      printSize(size_str, minTrackedSize << i);
      hits.subname(i, size_str.str());
      hits.subdesc(i, "Hits in a " + size_str.str() + " cache");
      misses.subname(i, size_str.str());
      misses.subdesc(i, "Misses in a " + size_str.str() + " cache");
    }
}

void
FALRU::CacheTracking::check(const FALRUBlk *head, const FALRUBlk *tail) const
{
#ifdef FALRU_DEBUG
    const FALRUBlk* blk = head;
    unsigned curr_size = 0;
    unsigned tracked_cache_size = minTrackedSize;
    CachesMask in_caches_mask = inAllCachesMask;
    int j = 0;

    while (blk) {
        panic_if(blk->inCachesMask != in_caches_mask, "Expected cache mask "
                 "%x found %x", blk->inCachesMask, in_caches_mask);

        curr_size += blkSize;
        if (curr_size == tracked_cache_size && blk != tail) {
            panic_if(boundaries[j] != blk, "Unexpected boundary for the %d-th "
                     "cache", j);
            tracked_cache_size <<= 1;
            // from this point, blocks fit only in the larger caches
            in_caches_mask &= ~(1U << j);
            ++j;
        }
        blk = blk->next;
    }
#endif // FALRU_DEBUG
}

void
FALRU::CacheTracking::init(FALRUBlk *head, FALRUBlk *tail)
{
    // early exit if we are not tracking any extra caches
    FALRUBlk* blk = numTrackedCaches ? head : nullptr;
    unsigned curr_size = 0;
    unsigned tracked_cache_size = minTrackedSize;
    CachesMask in_caches_mask = inAllCachesMask;
    int j = 0;

    while (blk) {
        blk->inCachesMask = in_caches_mask;

        curr_size += blkSize;
        if (curr_size == tracked_cache_size && blk != tail) {
            boundaries[j] = blk;

            tracked_cache_size <<= 1;
            // from this point, blocks fit only in the larger caches
            in_caches_mask &= ~(1U << j);
            ++j;
        }
        blk = blk->next;
    }
}


void
FALRU::CacheTracking::moveBlockToHead(FALRUBlk *blk)
{
    // Get the mask of all caches, in which the block didn't fit
    // before moving it to the head
    CachesMask update_caches_mask = inAllCachesMask ^ blk->inCachesMask;

    for (int i = 0; i < numTrackedCaches; i++) {
        CachesMask current_cache_mask = 1U << i;
        if (current_cache_mask & update_caches_mask) {
            // if the ith cache didn't fit the block (before it is moved to
            // the head), move the ith boundary 1 block closer to the
            // MRU
            boundaries[i]->inCachesMask &= ~current_cache_mask;
            boundaries[i] = boundaries[i]->prev;
        } else if (boundaries[i] == blk) {
            // Make sure the boundary doesn't point to the block
            // we are about to move
            boundaries[i] = blk->prev;
        }
    }

    // Make block reside in all caches
    blk->inCachesMask = inAllCachesMask;
}

void
FALRU::CacheTracking::moveBlockToTail(FALRUBlk *blk)
{
    CachesMask update_caches_mask = blk->inCachesMask;

    for (int i = 0; i < numTrackedCaches; i++) {
        CachesMask current_cache_mask = 1U << i;
        if (current_cache_mask & update_caches_mask) {
            // if the ith cache fitted the block (before it is moved to
            // the tail), move the ith boundary 1 block closer to the
            // LRU
            boundaries[i] = boundaries[i]->next;
            if (boundaries[i] == blk) {
                // Make sure the boundary doesn't point to the block
                // we are about to move
                boundaries[i] = blk->next;
            }
            boundaries[i]->inCachesMask |= current_cache_mask;
        }
    }

    // The block now fits only in the actual cache
    blk->inCachesMask = 0;
}

void
FALRU::CacheTracking::recordAccess(FALRUBlk *blk)
{
    for (int i = 0; i < numTrackedCaches; i++) {
        if (blk && ((1U << i) & blk->inCachesMask)) {
            hits[i]++;
        } else {
            misses[i]++;
        }
    }

    // Record stats for the actual cache too
    if (blk && blk->isValid()) {
        hits[numTrackedCaches]++;
    } else {
        misses[numTrackedCaches]++;
    }

    accesses++;
}

} // namespace gem5
