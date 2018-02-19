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
 * Declaration of a random replacement policy.
 * The victim is chosen at random, if there are no invalid entries.
 */

#ifndef __MEM_CACHE_REPLACEMENT_POLICIES_RANDOM_RP_HH__
#define __MEM_CACHE_REPLACEMENT_POLICIES_RANDOM_RP_HH__

#include "mem/cache/replacement_policies/base.hh"
#include "params/RandomRP.hh"

class RandomRP : public BaseReplacementPolicy
{
  public:
    /** Convenience typedef. */
    typedef RandomRPParams Params;

    /**
     * Construct and initiliaze this replacement policy.
     */
    RandomRP(const Params *p);

    /**
     * Destructor.
     */
    ~RandomRP() {}

    /**
     * Find replacement victim at random.
     * @param candidates Replacement candidates, selected by indexing policy.
     * @return Cache block to be replaced.
     */
    CacheBlk* getVictim(const ReplacementCandidates& candidates) override;
};

#endif // __MEM_CACHE_REPLACEMENT_POLICIES_RANDOM_RP_HH__
