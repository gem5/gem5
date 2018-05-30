/*
 * Copyright (c) 2012-2014 ARM Limited
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
 *
 * Authors: Erik Hallnor
 */

/**
 * @file
 * Definitions of a base set associative tag store.
 */

#include "mem/cache/tags/base_set_assoc.hh"

#include <string>

#include "base/intmath.hh"

BaseSetAssoc::BaseSetAssoc(const Params *p)
    :BaseTags(p), assoc(p->assoc), allocAssoc(p->assoc),
     blks(p->size / p->block_size),
     numSets(p->size / (p->block_size * p->assoc)),
     sequentialAccess(p->sequential_access),
     sets(p->size / (p->block_size * p->assoc)),
     replacementPolicy(p->replacement_policy)
{
    // Check parameters
    if (blkSize < 4 || !isPowerOf2(blkSize)) {
        fatal("Block size must be at least 4 and a power of 2");
    }
    if (!isPowerOf2(numSets)) {
        fatal("# of sets must be non-zero and a power of 2");
    }
    if (assoc <= 0) {
        fatal("associativity must be greater than zero");
    }

    setShift = floorLog2(blkSize);
    setMask = numSets - 1;
    tagShift = setShift + floorLog2(numSets);

    unsigned blkIndex = 0;       // index into blks array
    for (unsigned i = 0; i < numSets; ++i) {
        sets[i].assoc = assoc;

        sets[i].blks.resize(assoc);

        // link in the data blocks
        for (unsigned j = 0; j < assoc; ++j) {
            // Select block within the set to be linked
            BlkType*& blk = sets[i].blks[j];

            // Locate next cache block
            blk = &blks[blkIndex];

            // Associate a data chunk to the block
            blk->data = &dataBlks[blkSize*blkIndex];

            // Associate a replacement data entry to the block
            blk->replacementData = replacementPolicy->instantiateEntry();

            // Setting the tag to j is just to prevent long chains in the
            // hash table; won't matter because the block is invalid
            blk->tag = j;

            // Set its set and way
            blk->set = i;
            blk->way = j;

            // Update block index
            ++blkIndex;
        }
    }
}

void
BaseSetAssoc::invalidate(CacheBlk *blk)
{
    BaseTags::invalidate(blk);

    // Invalidate replacement data
    replacementPolicy->invalidate(blk->replacementData);
}

CacheBlk*
BaseSetAssoc::findBlock(Addr addr, bool is_secure) const
{
    Addr tag = extractTag(addr);
    unsigned set = extractSet(addr);
    BlkType *blk = sets[set].findBlk(tag, is_secure);
    return blk;
}

CacheBlk*
BaseSetAssoc::findBlockBySetAndWay(int set, int way) const
{
    return sets[set].blks[way];
}

BaseSetAssoc *
BaseSetAssocParams::create()
{
    return new BaseSetAssoc(this);
}
