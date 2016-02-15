/*
 * Copyright (c) 2013-2015 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Sooraj Puthoor
 */

#ifndef __MEM_RUBY_SYSTEM_WEIGHTEDLRUPOLICY_HH__
#define __MEM_RUBY_SYSTEM_WEIGHTEDLRUPOLICY_HH__

#include "mem/ruby/structures/AbstractReplacementPolicy.hh"
#include "mem/ruby/structures/CacheMemory.hh"
#include "params/WeightedLRUReplacementPolicy.hh"

/* Simple true LRU replacement policy */

class WeightedLRUPolicy : public AbstractReplacementPolicy
{
  public:
    typedef WeightedLRUReplacementPolicyParams Params;
    WeightedLRUPolicy(const Params* p);
    ~WeightedLRUPolicy();

    void touch(int64_t set, int64_t way, Tick time) override;
    void touch(int64_t set, int64_t way, Tick time, int occupancy);
    int64_t getVictim(int64_t set) const override;

    bool useOccupancy() const override { return true; }

    CacheMemory * m_cache;
    int **m_last_occ_ptr;
};

#endif // __MEM_RUBY_SYSTEM_WeightedLRUPolicy_HH__
