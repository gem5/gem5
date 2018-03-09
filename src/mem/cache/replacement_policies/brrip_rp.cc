/**
 * Copyright (c) 2018 Inria
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
 * Authors: Daniel Carvalho
 */

#include "mem/cache/replacement_policies/brrip_rp.hh"

#include "base/random.hh"
#include "debug/CacheRepl.hh"

BRRIPRP::BRRIPRP(const Params *p)
    : BaseReplacementPolicy(p),
      maxRRPV(p->max_RRPV), hitPriority(p->hit_priority), btp(p->btp)
{
    if (maxRRPV == 0){
        fatal("max_RRPV should be greater than zero.\n");
    }
}

void
BRRIPRP::touch(CacheBlk *blk)
{
    BaseReplacementPolicy::touch(blk);

    // Update RRPV if not 0 yet
    // Every hit in HP mode makes the block the last to be evicted, while
    // in FP mode a hit makes the block less likely to be evicted
    if (hitPriority) {
        blk->rrpv = 0;
    } else if (blk->rrpv > 0) {
        blk->rrpv--;
    }
}

void
BRRIPRP::reset(CacheBlk *blk)
{
    BaseReplacementPolicy::reset(blk);

    // Reset RRPV
    // Blocks are inserted as "long re-reference" if lower than btp,
    // "distant re-reference" otherwise
    if (random_mt.random<unsigned>(1, 100) <= btp) {
        blk->rrpv = maxRRPV-1;
    } else {
        blk->rrpv = maxRRPV;
    }
}

CacheBlk*
BRRIPRP::getVictim(const ReplacementCandidates& candidates)
{
    // There must be at least one replacement candidate
    assert(candidates.size() > 0);

    // Use visitor to search for the victim
    CacheBlk* blk = candidates[0];
    for (const auto& candidate : candidates) {
        // Stop iteration if found an invalid block
        if (!candidate->isValid()) {
            blk = candidate;
            blk->rrpv = maxRRPV;
            break;
        // Update victim block if necessary
        } else if (candidate->rrpv > blk->rrpv) {
            blk = candidate;
        }
    }

    // Make sure we don't have an invalid rrpv
    assert(blk->rrpv <= maxRRPV);

    // Get difference of block's RRPV to the highest possible RRPV in
    // order to update the RRPV of all the other blocks accordingly
    unsigned diff = maxRRPV - blk->rrpv;

    // No need to update RRPV if there is no difference
    if (diff > 0){
        // Update RRPV of all candidates
        for (const auto& candidate : candidates) {
            // Update the block's RPPV with the new value
            candidate->rrpv += diff;
        }
    }

    DPRINTF(CacheRepl, "set %x, way %x: selecting blk for replacement\n",
            blk->set, blk->way);

    return blk;
}

BRRIPRP*
BRRIPRPParams::create()
{
    return new BRRIPRP(this);
}
