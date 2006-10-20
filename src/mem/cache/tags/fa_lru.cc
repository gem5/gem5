/*
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
 *
 * Authors: Erik Hallnor
 */

/**
 * @file
 * Definitions a fully associative LRU tagstore.
 */

#include <sstream>

#include <assert.h>

#include "mem/cache/tags/fa_lru.hh"
#include "base/intmath.hh"
#include "base/misc.hh"

using namespace std;

FALRU::FALRU(int _blkSize, int _size, int hit_latency)
    : blkSize(_blkSize), size(_size),
      numBlks(size/blkSize), hitLatency(hit_latency)
{
    if (!isPowerOf2(blkSize))
        fatal("cache block size (in bytes) `%d' must be a power of two",
              blkSize);
    if (!(hitLatency > 0))
        fatal("Access latency in cycles must be at least one cycle");
    if (!isPowerOf2(size))
        fatal("Cache Size must be power of 2 for now");

    // Track all cache sizes from 128K up by powers of 2
    numCaches = floorLog2(size) - 17;
    if (numCaches >0){
        cacheBoundaries = new FALRUBlk *[numCaches];
        cacheMask = (1 << numCaches) - 1;
    } else {
        cacheMask = 0;
    }

    warmedUp = false;
    warmupBound = size/blkSize;

    blks = new FALRUBlk[numBlks];
    head = &(blks[0]);
    tail = &(blks[numBlks-1]);

    head->prev = NULL;
    head->next = &(blks[1]);
    head->inCache = cacheMask;

    tail->prev = &(blks[numBlks-2]);
    tail->next = NULL;
    tail->inCache = 0;

    int index = (1 << 17) / blkSize;
    int j = 0;
    int flags = cacheMask;
    for (int i = 1; i < numBlks-1; i++) {
        blks[i].inCache = flags;
        if (i == index - 1){
            cacheBoundaries[j] = &(blks[i]);
            flags &= ~ (1<<j);
            ++j;
            index = index << 1;
        }
        blks[i].prev = &(blks[i-1]);
        blks[i].next = &(blks[i+1]);
        blks[i].isTouched = false;
    }
    assert(j == numCaches);
    assert(index == numBlks);
    //assert(check());
}

void
FALRU::regStats(const string &name)
{
    using namespace Stats;
    BaseTags::regStats(name);
    hits
        .init(numCaches+1)
        .name(name + ".falru_hits")
        .desc("The number of hits in each cache size.")
        ;
    misses
        .init(numCaches+1)
        .name(name + ".falru_misses")
        .desc("The number of misses in each cache size.")
        ;
    accesses
        .name(name + ".falru_accesses")
        .desc("The number of accesses to the FA LRU cache.")
        ;

    for (int i = 0; i < numCaches+1; ++i) {
        stringstream size_str;
        if (i < 3){
            size_str << (1<<(i+7)) <<"K";
        } else {
            size_str << (1<<(i-3)) <<"M";
        }

        hits.subname(i, size_str.str());
        hits.subdesc(i, "Hits in a " + size_str.str() +" cache");
        misses.subname(i, size_str.str());
        misses.subdesc(i, "Misses in a " + size_str.str() +" cache");
    }
}

FALRUBlk *
FALRU::hashLookup(Addr addr) const
{
    tagIterator iter = tagHash.find(addr);
    if (iter != tagHash.end()) {
        return (*iter).second;
    }
    return NULL;
}

bool
FALRU::probe(Addr addr) const
{
    Addr blkAddr = blkAlign(addr);
    FALRUBlk* blk = hashLookup(blkAddr);
    return blk && blk->tag == blkAddr && blk->isValid();
}

void
FALRU::invalidateBlk(Addr addr)
{
    Addr blkAddr = blkAlign(addr);
    FALRUBlk* blk = (*tagHash.find(blkAddr)).second;
    if (blk) {
        assert(blk->tag == blkAddr);
        blk->status = 0;
        blk->isTouched = false;
        tagsInUse--;
    }
}

FALRUBlk*
FALRU::findBlock(Addr addr, int &lat, int *inCache)
{
    accesses++;
    int tmp_in_cache = 0;
    Addr blkAddr = blkAlign(addr);
    FALRUBlk* blk = hashLookup(blkAddr);

    if (blk && blk->isValid()) {
        assert(blk->tag == blkAddr);
        tmp_in_cache = blk->inCache;
        for (int i = 0; i < numCaches; i++) {
            if (1<<i & blk->inCache) {
                hits[i]++;
            } else {
                misses[i]++;
            }
        }
        hits[numCaches]++;
        if (blk != head){
            moveToHead(blk);
        }
    } else {
        blk = NULL;
        for (int i = 0; i < numCaches+1; ++i) {
            misses[i]++;
        }
    }
    if (inCache) {
        *inCache = tmp_in_cache;
    }

    lat = hitLatency;
    //assert(check());
    return blk;
}

FALRUBlk*
FALRU::findBlock(PacketPtr &pkt, int &lat, int *inCache)
{
    Addr addr = pkt->getAddr();

    accesses++;
    int tmp_in_cache = 0;
    Addr blkAddr = blkAlign(addr);
    FALRUBlk* blk = hashLookup(blkAddr);

    if (blk && blk->isValid()) {
        assert(blk->tag == blkAddr);
        tmp_in_cache = blk->inCache;
        for (int i = 0; i < numCaches; i++) {
            if (1<<i & blk->inCache) {
                hits[i]++;
            } else {
                misses[i]++;
            }
        }
        hits[numCaches]++;
        if (blk != head){
            moveToHead(blk);
        }
    } else {
        blk = NULL;
        for (int i = 0; i < numCaches+1; ++i) {
            misses[i]++;
        }
    }
    if (inCache) {
        *inCache = tmp_in_cache;
    }

    lat = hitLatency;
    //assert(check());
    return blk;
}

FALRUBlk*
FALRU::findBlock(Addr addr) const
{
    Addr blkAddr = blkAlign(addr);
    FALRUBlk* blk = hashLookup(blkAddr);

    if (blk && blk->isValid()) {
        assert(blk->tag == blkAddr);
    } else {
        blk = NULL;
    }
    return blk;
}

FALRUBlk*
FALRU::findReplacement(PacketPtr &pkt, PacketList &writebacks,
                       BlkList &compress_blocks)
{
    FALRUBlk * blk = tail;
    assert(blk->inCache == 0);
    moveToHead(blk);
    tagHash.erase(blk->tag);
    tagHash[blkAlign(pkt->getAddr())] = blk;
    if (blk->isValid()) {
        replacements[0]++;
    } else {
        tagsInUse++;
        blk->isTouched = true;
        if (!warmedUp && tagsInUse.value() >= warmupBound) {
            warmedUp = true;
            warmupCycle = curTick;
        }
    }
    //assert(check());
    return blk;
}

void
FALRU::moveToHead(FALRUBlk *blk)
{
    int updateMask = blk->inCache ^ cacheMask;
    for (int i = 0; i < numCaches; i++){
        if ((1<<i) & updateMask) {
            cacheBoundaries[i]->inCache &= ~(1<<i);
            cacheBoundaries[i] = cacheBoundaries[i]->prev;
        } else if (cacheBoundaries[i] == blk) {
            cacheBoundaries[i] = blk->prev;
        }
    }
    blk->inCache = cacheMask;
    if (blk != head) {
        if (blk == tail){
            assert(blk->next == NULL);
            tail = blk->prev;
            tail->next = NULL;
        } else {
            blk->prev->next = blk->next;
            blk->next->prev = blk->prev;
        }
        blk->next = head;
        blk->prev = NULL;
        head->prev = blk;
        head = blk;
    }
}

bool
FALRU::check()
{
    FALRUBlk* blk = head;
    int size = 0;
    int boundary = 1<<17;
    int j = 0;
    int flags = cacheMask;
    while (blk) {
        size += blkSize;
        if (blk->inCache != flags) {
            return false;
        }
        if (size == boundary && blk != tail) {
            if (cacheBoundaries[j] != blk) {
                return false;
            }
            flags &=~(1 << j);
            boundary = boundary<<1;
            ++j;
        }
        blk = blk->next;
    }
    return true;
}
