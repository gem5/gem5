/*
 * Copyright (c) 2024 Pranith Kumar
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
 */

/**
 * @file
 * Declaration of a BTB indexing policy.
 */

#ifndef __SIMPLE_BTB_INDEXING_POLICY_HH__
#define __SIMPLE_BTB_INDEXING_POLICY_HH__

#include <vector>

#include "base/intmath.hh"
#include "mem/cache/tags/indexing_policies/set_associative.hh"
#include "params/BTBIndexingPolicy.hh"

namespace gem5
{

class ReplaceableEntry;

class BTBIndexingPolicy : public SetAssociative
{
  public:
    /**
     * Convenience typedef.
     */
    typedef BTBIndexingPolicyParams Params;

    /**
     * Construct and initialize this policy.
     */
    BTBIndexingPolicy(const Params &p);

    /**
     * Destructor.
     */
    ~BTBIndexingPolicy() {};

  protected:
    uint32_t extractSet(const Addr instPC, ThreadID tid) const;

  public:
    /**
     * Find all possible entries for insertion and replacement of an address.
     *
     * @param addr The addr to a find possible entries for.
     * @param tid The thread id
     * @return The possible entries.
     */
    std::vector<ReplaceableEntry*>
    getPossibleEntries(const Addr addr, ThreadID tid) const
    {
        return sets[extractSet(addr,tid)];
    }

    void setNumThreads(unsigned num_threads)
    {
        log2NumThreads = log2i(num_threads);
    }

  private:
    unsigned log2NumThreads;
    using SetAssociative::extractSet;
    using SetAssociative::getPossibleEntries;
};

} // namespace gem5

#endif //__MEM_CACHE_INDEXING_POLICIES_SET_ASSOCIATIVE_HH__
