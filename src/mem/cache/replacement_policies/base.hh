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

#ifndef __MEM_CACHE_REPLACEMENT_POLICIES_BASE_HH__
#define __MEM_CACHE_REPLACEMENT_POLICIES_BASE_HH__

#include "mem/cache/base.hh"
#include "mem/cache/blk.hh"
#include "params/BaseReplacementPolicy.hh"
#include "sim/sim_object.hh"

/**
 * Replacement candidates as chosen by the indexing policy.
 *
 * The base functions touch() and reset() must be called by all subclasses
 * that override them.
 *
 * @todo
 *   Currently the replacement candidates are simply the cache blocks
 *   derived from the possible placement locations of an address, as
 *   defined by the getPossibleLocations() from BaseTags. In a future
 *   patch it should be an inheritable class to allow the replacement
 *   policies to be used with any table-like structure that needs to
 *   replace its entries.
 */
typedef std::vector<CacheBlk*> ReplacementCandidates;

/**
 * A common base class of cache replacement policy objects.
 */
class BaseReplacementPolicy : public SimObject
{
  public:
    /**
      * Convenience typedef.
      */
    typedef BaseReplacementPolicyParams Params;

    /**
     * Construct and initiliaze this replacement policy.
     */
    BaseReplacementPolicy(const Params *p);

    /**
     * Destructor.
     */
    virtual ~BaseReplacementPolicy() {}

    /**
     * Touch a block to update its replacement data.
     * Updates number of references.
     *
     * This base function must be called by all subclasses that override it.
     *
     * @param blk Cache block to be touched.
     */
    virtual void touch(CacheBlk *blk);

    /**
     * Reset replacement data for a block. Used when a block is inserted.
     * Sets the insertion tick, and update number of references.
     *
     * This base function must be called by all subclasses that override it.
     *
     * @param blk Cache block to be reset.
     */
    virtual void reset(CacheBlk *blk);

    /**
     * Find replacement victim among candidates.
     *
     * @param candidates Replacement candidates, selected by indexing policy.
     * @return Cache block to be replaced.
     */
    virtual CacheBlk* getVictim(const ReplacementCandidates& candidates) = 0;
};

#endif // __MEM_CACHE_REPLACEMENT_POLICIES_BASE_HH__
