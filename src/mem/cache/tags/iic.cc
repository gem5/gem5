/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 * Definitions of the Indirect Index Cache tagstore.
 */

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "base/intmath.hh"
#include "base/trace.hh"
#include "debug/Cache.hh"
#include "debug/IIC.hh"
#include "debug/IICMore.hh"
#include "mem/cache/tags/iic.hh"
#include "mem/cache/base.hh"
#include "sim/core.hh"

using namespace std;

/** Track the number of accesses to each cache set. */
#define PROFILE_IIC 1

IIC::IIC(IIC::Params &params) :
    hashSets(params.numSets), blkSize(params.blkSize), assoc(params.assoc),
    hitLatency(params.hitLatency), subSize(params.subblockSize),
    numSub(blkSize/subSize),
    trivialSize((floorLog2(params.size/subSize)*numSub)/8),
    tagShift(floorLog2(blkSize)), blkMask(blkSize - 1),
    subShift(floorLog2(subSize)), subMask(numSub - 1),
    hashDelay(params.hashDelay),
    numTags(hashSets * assoc + params.size/blkSize -1),
    numSecondary(params.size/blkSize),
    tagNull(numTags),
    primaryBound(hashSets * assoc)
{
    // Check parameters
    if (blkSize < 4 || !isPowerOf2(blkSize)) {
        fatal("Block size must be at least 4 and a power of 2");
    }
    if (hashSets <= 0 || !isPowerOf2(hashSets)) {
        fatal("# of hashsets must be non-zero and a power of 2");
    }
    if (assoc <= 0) {
        fatal("associativity must be greater than zero");
    }
    if (hitLatency <= 0) {
        fatal("access latency must be greater than zero");
    }
    if (numSub*subSize != blkSize) {
        fatal("blocksize must be evenly divisible by subblock size");
    }

    // debug stuff
    freeSecond = numSecondary;

    warmedUp = false;
    warmupBound = params.size/blkSize;
    numBlocks = params.size/subSize;

    // Replacement Policy Initialization
    repl = params.rp;
    repl->setIIC(this);

    //last_miss_time = 0

    // allocate data reference counters
    dataReferenceCount = new int[numBlocks];
    memset(dataReferenceCount, 0, numBlocks*sizeof(int));

    // Allocate storage for both internal data and block fast access data.
    // We allocate it as one large chunk to reduce overhead and to make
    // deletion easier.
    unsigned data_index = 0;
    dataStore = new uint8_t[(numBlocks + numTags) * blkSize];
    dataBlks = new uint8_t*[numBlocks];
    for (unsigned i = 0; i < numBlocks; ++i) {
        dataBlks[i] = &dataStore[data_index];
        freeDataBlock(i);
        data_index += subSize;
    }

    assert(data_index == numBlocks * subSize);

    // allocate and init tag store
    tagStore = new IICTag[numTags];

    unsigned blkIndex = 0;
    // allocate and init sets
    sets = new IICSet[hashSets];
    for (unsigned i = 0; i < hashSets; ++i) {
        sets[i].assoc = assoc;
        sets[i].tags = new IICTag*[assoc];
        sets[i].chain_ptr = tagNull;

        for (unsigned j = 0; j < assoc; ++j) {
            IICTag *tag = &tagStore[blkIndex++];
            tag->chain_ptr = tagNull;
            tag->data_ptr.resize(numSub);
            tag->size = blkSize;
            tag->trivialData = new uint8_t[trivialSize];
            tag->numData = 0;
            sets[i].tags[j] = tag;
            tag->set = i;
            tag->data = &dataStore[data_index];
            data_index += blkSize;
        }
    }

    assert(blkIndex == primaryBound);

    for (unsigned i = primaryBound; i < tagNull; i++) {
        tagStore[i].chain_ptr = i+1;
        //setup data ptrs to subblocks
        tagStore[i].data_ptr.resize(numSub);
        tagStore[i].size = blkSize;
        tagStore[i].trivialData = new uint8_t[trivialSize];
        tagStore[i].numData = 0;
        tagStore[i].set = 0;
        tagStore[i].data = &dataStore[data_index];
        data_index += blkSize;
    }
    freelist = primaryBound;
}

IIC::~IIC()
{
    delete [] dataReferenceCount;
    delete [] dataStore;
    delete [] tagStore;
    delete [] sets;
}

/* register cache stats */
void
IIC::regStats(const string &name)
{
    using namespace Stats;

    BaseTags::regStats(name);

    hitHashDepth.init(0, 20, 1);
    missHashDepth.init(0, 20, 1);
    setAccess.init(0, hashSets, 1);

    /** IIC Statistics */
    hitHashDepth
        .name(name + ".hit_hash_depth_dist")
        .desc("Dist. of Hash lookup depths")
        .flags(pdf)
        ;

    missHashDepth
        .name(name + ".miss_hash_depth_dist")
        .desc("Dist. of Hash lookup depths")
        .flags(pdf)
        ;

    repl->regStatsWithSuffix(name);

    if (PROFILE_IIC)
        setAccess
            .name(name + ".set_access_dist")
            .desc("Dist. of Accesses across sets")
            .flags(pdf)
            ;

    missDepthTotal
        .name(name + ".miss_depth_total")
        .desc("Total of miss depths")
        ;

    hashMiss
        .name(name + ".hash_miss")
        .desc("Total of misses in hash table")
        ;

    hitDepthTotal
        .name(name + ".hit_depth_total")
        .desc("Total of hit depths")
        ;

    hashHit
        .name(name + ".hash_hit")
        .desc("Total of hites in hash table")
        ;
}


IICTag*
IIC::accessBlock(Addr addr, int &lat, int context_src)
{
    Addr tag = extractTag(addr);
    unsigned set = hash(addr);
    int set_lat;

    unsigned long chain_ptr = tagNull;

    if (PROFILE_IIC)
        setAccess.sample(set);

    IICTag *tag_ptr = sets[set].findTag(tag, chain_ptr);
    set_lat = 1;
    if (tag_ptr == NULL && chain_ptr != tagNull) {
        int secondary_depth;
        tag_ptr = secondaryChain(tag, chain_ptr, &secondary_depth);
        set_lat += secondary_depth;
        // set depth for statistics fix this later!!! egh
        sets[set].depth = set_lat;

        if (tag_ptr != NULL) {
            /* need to move tag into primary table */
            // need to preserve chain: fix this egh
            sets[set].tags[assoc-1]->chain_ptr = tag_ptr->chain_ptr;
            tagSwap(tag_ptr - tagStore, sets[set].tags[assoc-1] - tagStore);
            tag_ptr = sets[set].findTag(tag, chain_ptr);
            assert(tag_ptr!=NULL);
        }

    }
    set_lat = set_lat * hashDelay + hitLatency;
    if (tag_ptr != NULL) {
        // IIC replacement: if this is not the first element of
        //   list, reorder
        sets[set].moveToHead(tag_ptr);

        hitHashDepth.sample(sets[set].depth);
        hashHit++;
        hitDepthTotal += sets[set].depth;
        tag_ptr->status |= BlkReferenced;
        lat = set_lat;
        if (tag_ptr->whenReady > curTick() && tag_ptr->whenReady - curTick() > set_lat) {
            lat = tag_ptr->whenReady - curTick();
        }

        tag_ptr->refCount += 1;
    }
    else {
        // fall through: cache block not found, not a hit...
        missHashDepth.sample(sets[set].depth);
        hashMiss++;
        missDepthTotal += sets[set].depth;
        lat = set_lat;
    }
    return tag_ptr;
}


IICTag*
IIC::findBlock(Addr addr) const
{
    Addr tag = extractTag(addr);
    unsigned set = hash(addr);

    unsigned long chain_ptr = tagNull;

    IICTag *tag_ptr = sets[set].findTag(tag, chain_ptr);
    if (tag_ptr == NULL && chain_ptr != tagNull) {
        int secondary_depth;
        tag_ptr = secondaryChain(tag, chain_ptr, &secondary_depth);
    }
    return tag_ptr;
}


IICTag*
IIC::findVictim(Addr addr, PacketList &writebacks)
{
    DPRINTF(IIC, "Finding Replacement for %x\n", addr);
    unsigned set = hash(addr);
    IICTag *tag_ptr;
    unsigned long *tmp_data = new unsigned long[numSub];

    // Get a enough subblocks for a full cache line
    for (unsigned i = 0; i < numSub; ++i){
        tmp_data[i] = getFreeDataBlock(writebacks);
        assert(dataReferenceCount[tmp_data[i]]==0);
    }

    tag_ptr = getFreeTag(set, writebacks);

    tag_ptr->set = set;
    for (unsigned i = 0; i < numSub; ++i) {
        tag_ptr->data_ptr[i] = tmp_data[i];
        dataReferenceCount[tag_ptr->data_ptr[i]]++;
    }
    tag_ptr->numData = numSub;
    assert(tag_ptr - tagStore < primaryBound); // make sure it is in primary
    tag_ptr->chain_ptr = tagNull;
    sets[set].moveToHead(tag_ptr);
    delete [] tmp_data;

    list<unsigned long> tag_indexes;
    repl->doAdvance(tag_indexes);
/*
    while (!tag_indexes.empty()) {
        if (!tagStore[tag_indexes.front()].isCompressed()) {
            compress_blocks.push_back(&tagStore[tag_indexes.front()]);
        }
        tag_indexes.pop_front();
    }
*/

    tag_ptr->re = (void*)repl->add(tag_ptr-tagStore);

    return tag_ptr;
}

void
IIC::insertBlock(Addr addr, BlkType* blk, int context_src)
{
}

void
IIC::freeReplacementBlock(PacketList & writebacks)
{
    IICTag *tag_ptr;
    unsigned long data_ptr;
    /* consult replacement policy */
    tag_ptr = &tagStore[repl->getRepl()];
    assert(tag_ptr->isValid());

    DPRINTF(Cache, "Replacing %x in IIC: %s\n",
            regenerateBlkAddr(tag_ptr->tag,0),
            tag_ptr->isDirty() ? "writeback" : "clean");
    /* write back replaced block data */
    if (tag_ptr && (tag_ptr->isValid())) {
        replacements[0]++;
        totalRefs += tag_ptr->refCount;
        ++sampledRefs;
        tag_ptr->refCount = 0;

        if (tag_ptr->isDirty()) {
/*          PacketPtr writeback =
                buildWritebackReq(regenerateBlkAddr(tag_ptr->tag, 0),
                                  tag_ptr->req->asid, tag_ptr->xc, blkSize,
                                  tag_ptr->data,
                                  tag_ptr->size);
*/
            Request *writebackReq = new Request(regenerateBlkAddr(tag_ptr->tag, 0),
                                           blkSize, 0, Request::wbMasterId);
            PacketPtr writeback = new Packet(writebackReq, MemCmd::Writeback);
            writeback->allocate();
            memcpy(writeback->getPtr<uint8_t>(), tag_ptr->data, blkSize);

            writebacks.push_back(writeback);
        }
    }

    // free the data blocks
    for (int i = 0; i < tag_ptr->numData; ++i) {
        data_ptr = tag_ptr->data_ptr[i];
        assert(dataReferenceCount[data_ptr]>0);
        if (--dataReferenceCount[data_ptr] == 0) {
            freeDataBlock(data_ptr);
        }
    }
    freeTag(tag_ptr);
}

unsigned long
IIC::getFreeDataBlock(PacketList & writebacks)
{
    unsigned long data_ptr;

    /* find data block */
    while (blkFreelist.empty()) {
        freeReplacementBlock(writebacks);
    }

    data_ptr = blkFreelist.front();
    blkFreelist.pop_front();
    DPRINTF(IICMore,"Found free data at %d\n",data_ptr);
    return data_ptr;
}



IICTag*
IIC::getFreeTag(int set, PacketList & writebacks)
{
    unsigned long tag_index;
    IICTag *tag_ptr;
    // Add new tag
    tag_ptr = sets[set].findFree();
    // if no free in primary, and secondary exists
    if (!tag_ptr && numSecondary) {
        // need to spill a tag into secondary storage
        while (freelist == tagNull) {
            // get replacements until one is in secondary
            freeReplacementBlock(writebacks);
        }

        tag_index = freelist;
        freelist = tagStore[freelist].chain_ptr;
        freeSecond--;

        assert(tag_index != tagNull);
        tagSwap(tag_index, sets[set].tags[assoc-1] - tagStore);
        tagStore[tag_index].chain_ptr = sets[set].chain_ptr;
        sets[set].chain_ptr = tag_index;

        tag_ptr = sets[set].tags[assoc-1];
    }
    DPRINTF(IICMore,"Found free tag at %d\n",tag_ptr - tagStore);
    tagsInUse++;
    if (!warmedUp && tagsInUse.value() >= warmupBound) {
        warmedUp = true;
        warmupCycle = curTick();
    }

    return tag_ptr;
}

void
IIC::freeTag(IICTag *tag_ptr)
{
    unsigned long tag_index, tmp_index;
    // Fix tag_ptr
    if (tag_ptr) {
        // we have a tag to clear
        DPRINTF(IICMore,"Freeing Tag for %x\n",
                regenerateBlkAddr(tag_ptr->tag,0));
        tagsInUse--;
        tag_ptr->status = 0;
        tag_ptr->numData = 0;
        tag_ptr->re = NULL;
        tag_index = tag_ptr - tagStore;
        if (tag_index >= primaryBound) {
            // tag_ptr points to secondary store
            assert(tag_index < tagNull); // remove this?? egh
            if (tag_ptr->chain_ptr == tagNull) {
                // need to fix chain list
                unsigned tmp_set = hash(tag_ptr->tag << tagShift);
                if (sets[tmp_set].chain_ptr == tag_index) {
                    sets[tmp_set].chain_ptr = tagNull;
                } else {
                    tmp_index = sets[tmp_set].chain_ptr;
                    while (tmp_index != tagNull
                           && tagStore[tmp_index].chain_ptr != tag_index) {
                        tmp_index = tagStore[tmp_index].chain_ptr;
                    }
                    assert(tmp_index != tagNull);
                    tagStore[tmp_index].chain_ptr = tagNull;
                }
                tag_ptr->chain_ptr = freelist;
                freelist = tag_index;
                freeSecond++;
            } else {
                // copy next chained entry to this tag location
                tmp_index = tag_ptr->chain_ptr;
                tagSwap(tmp_index, tag_index);
                tagStore[tmp_index].chain_ptr = freelist;
                freelist = tmp_index;
                freeSecond++;
            }
        } else {
            // tag_ptr in primary hash table
            assert(tag_index < primaryBound);
            tag_ptr->status = 0;
            unsigned tmp_set = hash(tag_ptr->tag << tagShift);
            if (sets[tmp_set].chain_ptr != tagNull) { // collapse chain
                tmp_index = sets[tmp_set].chain_ptr;
                tagSwap(tag_index, tmp_index);
                tagStore[tmp_index].chain_ptr = freelist;
                freelist = tmp_index;
                freeSecond++;
                sets[tmp_set].chain_ptr = tag_ptr->chain_ptr;
                sets[tmp_set].moveToTail(tag_ptr);
            }
        }
    }
}

void
IIC::freeDataBlock(unsigned long data_ptr)
{
    assert(dataReferenceCount[data_ptr] == 0);
    DPRINTF(IICMore, "Freeing data at %d\n", data_ptr);
    blkFreelist.push_front(data_ptr);
}

/** Use a simple modulo hash. */
#define SIMPLE_HASH 0

unsigned
IIC::hash(Addr addr) const {
#if SIMPLE_HASH
    return extractTag(addr) % iic_hash_size;
#else
    Addr tag, mask, x, y;
    tag = extractTag(addr);
    mask = hashSets-1; /* assumes iic_hash_size is a power of 2 */
    x = tag & mask;
    y = (tag >> (int)(::log((double)hashSets)/::log((double)2))) & mask;
    assert (x < hashSets && y < hashSets);
    return x ^ y;
#endif
}


void
IICSet::moveToHead(IICTag *tag)
{
    if (tags[0] == tag)
        return;

    // write 'next' block into blks[i], moving up from MRU toward LRU
    // until we overwrite the block we moved to head.

    // start by setting up to write 'blk' into blks[0]
    int i = 0;
    IICTag *next = tag;

    do {
        assert(i < assoc);
        // swap blks[i] and next
        IICTag *tmp = tags[i];
        tags[i] = next;
        next = tmp;
        ++i;
    } while (next != tag);
}

void
IICSet::moveToTail(IICTag *tag)
{
    if (tags[assoc-1] == tag)
        return;

    // write 'next' block into blks[i], moving up from MRU toward LRU
    // until we overwrite the block we moved to head.

    // start by setting up to write 'blk' into blks[0]
    int i = assoc - 1;
    IICTag *next = tag;

    do {
        assert(i >= 0);
        // swap blks[i] and next
        IICTag *tmp = tags[i];
        tags[i] = next;
        next = tmp;
        --i;
    } while (next != tag);
}

void
IIC::tagSwap(unsigned long index1, unsigned long index2)
{
    DPRINTF(IIC,"Swapping tag[%d]=%x for tag[%d]=%x\n",index1,
            tagStore[index1].tag<<tagShift, index2,
            tagStore[index2].tag<<tagShift);
    IICTag tmp_tag;
    tmp_tag = tagStore[index1];
    tagStore[index1] = tagStore[index2];
    tagStore[index2] = tmp_tag;
    if (tagStore[index1].isValid())
        repl->fixTag(tagStore[index1].re, index2, index1);
    if (tagStore[index2].isValid())
        repl->fixTag(tagStore[index2].re, index1, index2);
}


IICTag *
IIC::secondaryChain(Addr tag, unsigned long chain_ptr,
                    int *_depth) const
{
    int depth = 0;
    while (chain_ptr != tagNull) {
        DPRINTF(IIC,"Searching secondary at %d for %x\n", chain_ptr,
                tag<<tagShift);
        if (tagStore[chain_ptr].tag == tag &&
            (tagStore[chain_ptr].isValid())) {
            *_depth = depth;
            return &tagStore[chain_ptr];
        }
        depth++;
        chain_ptr = tagStore[chain_ptr].chain_ptr;
    }
    *_depth = depth;
    return NULL;
}

void
IIC::invalidateBlk(IIC::BlkType *tag_ptr)
{
    if (tag_ptr) {
        for (int i = 0; i < tag_ptr->numData; ++i) {
            dataReferenceCount[tag_ptr->data_ptr[i]]--;
            if (dataReferenceCount[tag_ptr->data_ptr[i]] == 0) {
                freeDataBlock(tag_ptr->data_ptr[i]);
            }
        }
        repl->removeEntry(tag_ptr->re);
        freeTag(tag_ptr);
    }
}

void
IIC::clearLocks()
{
    for (int i = 0; i < numTags; i++){
        tagStore[i].clearLoadLocks();
    }
}

void
IIC::cleanupRefs()
{
    for (unsigned i = 0; i < numTags; ++i) {
        if (tagStore[i].isValid()) {
            totalRefs += tagStore[i].refCount;
            ++sampledRefs;
        }
    }
}
