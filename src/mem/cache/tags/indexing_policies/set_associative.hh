/*
 * Copyright (c) 2018 Inria
 * Copyright (c) 2012-2014,2017 ARM Limited
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
 * Authors: Daniel Carvalho
 *          Erik Hallnor
 */

/**
 * @file
 * Declaration of a set associative indexing policy.
 */

#ifndef __MEM_CACHE_INDEXING_POLICIES_SET_ASSOCIATIVE_HH__
#define __MEM_CACHE_INDEXING_POLICIES_SET_ASSOCIATIVE_HH__

#include <vector>

#include "mem/cache/tags/indexing_policies/base.hh"
#include "params/SetAssociative.hh"

class ReplaceableEntry;

/**
 * A set associative indexing policy.
 * @sa  \ref gem5MemorySystem "gem5 Memory System"
 *
 * The set associative indexing policy has an immutable/identity mapping, so a
 * value x is always mapped to set x, independent of the way, that is,
 * Hash(A, 0) = Hash(A, 1) = Hash(A, N-1), where N is the number of ways.
 *
 * For example, let's assume address A maps to set 3 on way 0. This policy
 * makes so that A is also mappable to set 3 on every other way. Visually, the
 * possible locations of A are, for a table with 4 ways and 8 sets:
 *    Way 0   1   2   3
 *  Set   _   _   _   _
 *    0  |_| |_| |_| |_|
 *    1  |_| |_| |_| |_|
 *    2  |_| |_| |_| |_|
 *    3  |X| |X| |X| |X|
 *    4  |_| |_| |_| |_|
 *    5  |_| |_| |_| |_|
 *    6  |_| |_| |_| |_|
 *    7  |_| |_| |_| |_|
 */
class SetAssociative : public BaseIndexingPolicy
{
  private:
    /**
     * Apply a hash function to calculate address set.
     *
     * @param addr The address to calculate the set for.
     * @return The set index for given combination of address and way.
     */
    uint32_t extractSet(const Addr addr) const;

  public:
    /**
     * Convenience typedef.
     */
    typedef SetAssociativeParams Params;

    /**
     * Construct and initialize this policy.
     */
    SetAssociative(const Params *p);

    /**
     * Destructor.
     */
    ~SetAssociative() {};

    /**
     * Find all possible entries for insertion and replacement of an address.
     * Should be called immediately before ReplacementPolicy's findVictim()
     * not to break cache resizing.
     * Returns entries in all ways belonging to the set of the address.
     *
     * @param addr The addr to a find possible entries for.
     * @return The possible entries.
     */
    std::vector<ReplaceableEntry*> getPossibleEntries(const Addr addr) const
                                                                     override;

    /**
     * Regenerate an entry's address from its tag and assigned set and way.
     *
     * @param tag The tag bits.
     * @param entry The entry.
     * @return the entry's original addr value.
     */
    Addr regenerateAddr(const Addr tag, const ReplaceableEntry* entry) const
                                                                   override;
};

#endif //__MEM_CACHE_INDEXING_POLICIES_SET_ASSOCIATIVE_HH__
