/*
 * Copyright (c) 2014 The Regents of The University of Michigan
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
 * Authors: Anthony Gutierrez
 */

/**
 * @file
 * Definitions of a random replacement tag store.
 */

#include "mem/cache/tags/random_repl.hh"

#include "base/random.hh"
#include "debug/CacheRepl.hh"
#include "mem/cache/base.hh"

RandomRepl::RandomRepl(const Params *p)
    : BaseSetAssoc(p)

{
}

CacheBlk*
RandomRepl::accessBlock(Addr addr, bool is_secure, Cycles &lat, int master_id)
{
    return BaseSetAssoc::accessBlock(addr, is_secure, lat, master_id);
}

CacheBlk*
RandomRepl::findVictim(Addr addr)
{
    CacheBlk *blk = BaseSetAssoc::findVictim(addr);
    unsigned set = extractSet(addr);

    // if all blocks are valid, pick a replacement at random
    if (blk && blk->isValid()) {
        // find a random index within the bounds of the set
        int idx = random_mt.random<int>(0, assoc - 1);
        blk = sets[set].blks[idx];
        // Enforce allocation limit
        while (blk->way >= allocAssoc) {
            idx = (idx + 1) % assoc;
            blk = sets[set].blks[idx];
        }

        assert(idx < assoc);
        assert(idx >= 0);
        assert(blk->way < allocAssoc);

        DPRINTF(CacheRepl, "set %x: selecting blk %x for replacement\n",
                blk->set, regenerateBlkAddr(blk->tag, blk->set));
    }

    return blk;
}

void
RandomRepl::insertBlock(PacketPtr pkt, BlkType *blk)
{
    BaseSetAssoc::insertBlock(pkt, blk);
}

void
RandomRepl::invalidate(CacheBlk *blk)
{
    BaseSetAssoc::invalidate(blk);
}

RandomRepl*
RandomReplParams::create()
{
    return new RandomRepl(this);
}
