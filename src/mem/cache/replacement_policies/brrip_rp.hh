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

/**
 * @file
 * Declaration of a Re-Reference Interval Prediction replacement policy.
 *
 * Not-Recently Used (NRU) is an approximation of LRU that uses a single bit
 * to determine if a block is going to be re-referenced in the near or distant
 * future.
 *
 * Re-Reference Interval Prediction (RRIP) is an extension of NRU that uses a
 * re-reference prediction value to determine if blocks are going to be re-
 * used in the near future or not.
 *
 * The higher the value of the RRPV, the more distant the block is from
 * its next access.
 *
 * Bimodal Re-Reference Interval Prediction (BRRIP) is an extension of RRIP
 * that has a probability of not inserting blocks as the LRU. This probability
 * is controlled by the bimodal throtle parameter (btp).
 *
 * From the original paper, this implementation of RRIP is also called
 * Static RRIP (SRRIP), as it always inserts blocks with the same RRPV.
 */

#ifndef __MEM_CACHE_REPLACEMENT_POLICIES_BRRIP_RP_HH__
#define __MEM_CACHE_REPLACEMENT_POLICIES_BRRIP_RP_HH__

#include "mem/cache/replacement_policies/base.hh"
#include "params/BRRIPRP.hh"

class BRRIPRP : public BaseReplacementPolicy
{
  protected:
    /**
     * Maximum Re-Reference Prediction Value possible. A block with this
     * value as the rrpv has the longest possible re-reference interval,
     * that is, it is likely not to be used in the near future, and is
     * among the best eviction candidates.
     * A maxRRPV of 1 implies in a NRU.
     */
    const unsigned maxRRPV;

    /**
     * The hit priority (HP) policy replaces blocks that do not receive cache
     * hits over any cache block that receives a hit, while the frequency
     * priority (FP) policy replaces infrequently re-referenced blocks.
     */
    const bool hitPriority;

    /**
     * Bimodal throtle parameter. Value in the range [0,100] used to decide
     * if a new block is inserted with long or distant re-reference.
     */
    const unsigned btp;

  public:
    /** Convenience typedef. */
    typedef BRRIPRPParams Params;

    /**
     * Construct and initiliaze this replacement policy.
     */
    BRRIPRP(const Params *p);

    /**
     * Destructor.
     */
    ~BRRIPRP() {}

    /**
     * Touch a block to update its replacement data.
     *
     * @param blk Cache block to be touched.
     */
    void touch(CacheBlk *blk) override;

    /**
     * Reset replacement data for a block. Used when a block is inserted.
     * Sets the insertion tick, and update correspondent replacement data.
     * Set RRPV according to the insertion policy used.
     *
     * @param blk Cache block to be reset.
     */
    void reset(CacheBlk *blk) override;

    /**
     * Find replacement victim using rrpv.
     *
     * @param cands Replacement candidates, selected by indexing policy.
     * @return Cache block to be replaced.
     */
    CacheBlk* getVictim(const ReplacementCandidates& candidates) override;
};

#endif // __MEM_CACHE_REPLACEMENT_POLICIES_BRRIP_RP_HH__
