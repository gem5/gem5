/*
 * Copyright (c) 2013 ARM Limited
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
 *
 * Authors: Erik Hallnor
 */

/**
 * @file
 * Definitions a fully associative LRU tagstore.
 */

#include <cassert>
#include <sstream>

#include "base/intmath.hh"
#include "base/misc.hh"
#include "mem/cache/tags/fa_lru.hh"

using namespace std;

FALRU::FALRU(const Params *p)
    : BaseTags(p), cacheBoundaries(nullptr)
{
    if (!isPowerOf2(blkSize))
        fatal("cache block size (in bytes) `%d' must be a power of two",
              blkSize);
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

    warmupBound = size/blkSize;
    numBlocks = size/blkSize;

    blks = new FALRUBlk[numBlocks];
    head = &(blks[0]);
    tail = &(blks[numBlocks-1]);

    head->prev = NULL;
    head->next = &(blks[1]);
    head->inCache = cacheMask;

    tail->prev = &(blks[numBlocks-2]);
    tail->next = NULL;
    tail->inCache = 0;

    unsigned index = (1 << 17) / blkSize;
    unsigned j = 0;
    int flags = cacheMask;
    for (unsigned i = 1; i < numBlocks - 1; i++) {
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
        blks[i].set = 0;
        blks[i].way = i;
    }
    assert(j == numCaches);
    assert(index == numBlocks);
    //assert(check());
}

FALRU::~FALRU()
{
    if (numCaches)
        delete[] cacheBoundaries;

    delete[] blks;
}

void
FALRU::regStats()
{
    using namespace Stats;
    BaseTags::regStats();
    hits
        .init(numCaches+1)
        .name(name() + ".falru_hits")
        .desc("The number of hits in each cache size.")
        ;
    misses
        .init(numCaches+1)
        .name(name() + ".falru_misses")
        .desc("The number of misses in each cache size.")
        ;
    accesses
        .name(name() + ".falru_accesses")
        .desc("The number of accesses to the FA LRU cache.")
        ;

    for (unsigned i = 0; i <= numCaches; ++i) {
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

void
FALRU::invalidate(CacheBlk *blk)
{
    assert(blk);
    tagsInUse--;
}

CacheBlk*
FALRU::accessBlock(Addr addr, bool is_secure, Cycles &lat, int context_src)
{
    return accessBlock(addr, is_secure, lat, context_src, 0);
}

CacheBlk*
FALRU::accessBlock(Addr addr, bool is_secure, Cycles &lat, int context_src,
                   int *inCache)
{
    accesses++;
    int tmp_in_cache = 0;
    Addr blkAddr = blkAlign(addr);
    FALRUBlk* blk = hashLookup(blkAddr);

    if (blk && blk->isValid()) {
        assert(blk->tag == blkAddr);
        tmp_in_cache = blk->inCache;
        for (unsigned i = 0; i < numCaches; i++) {
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
        for (unsigned i = 0; i <= numCaches; ++i) {
            misses[i]++;
        }
    }
    if (inCache) {
        *inCache = tmp_in_cache;
    }

    lat = accessLatency;
    //assert(check());
    return blk;
}


CacheBlk*
FALRU::findBlock(Addr addr, bool is_secure) const
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

CacheBlk*
FALRU::findBlockBySetAndWay(int set, int way) const
{
    assert(set == 0);
    return &blks[way];
}

CacheBlk*
FALRU::findVictim(Addr addr)
{
    FALRUBlk * blk = tail;
    assert(blk->inCache == 0);
    moveToHead(blk);
    tagHash.erase(blk->tag);
    tagHash[blkAlign(addr)] = blk;
    if (blk->isValid()) {
        replacements[0]++;
    } else {
        tagsInUse++;
        blk->isTouched = true;
        if (!warmedUp && tagsInUse.value() >= warmupBound) {
            warmedUp = true;
            warmupCycle = curTick();
        }
    }
    //assert(check());
    return blk;
}

void
FALRU::insertBlock(PacketPtr pkt, CacheBlk *blk)
{
}

void
FALRU::moveToHead(FALRUBlk *blk)
{
    int updateMask = blk->inCache ^ cacheMask;
    for (unsigned i = 0; i < numCaches; i++){
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
    int tot_size = 0;
    int boundary = 1<<17;
    int j = 0;
    int flags = cacheMask;
    while (blk) {
        tot_size += blkSize;
        if (blk->inCache != flags) {
            return false;
        }
        if (tot_size == boundary && blk != tail) {
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

FALRU *
FALRUParams::create()
{
    return new FALRU(this);
}

