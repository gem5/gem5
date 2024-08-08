/**
 * Copyright (c) 2024 Arm Limited
 * All rights reserved
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
 * Copyright (c) 2018 Metempsy Technology Consulting
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

#ifndef __CACHE_PREFETCH_ASSOCIATIVE_SET_HH__
#define __CACHE_PREFETCH_ASSOCIATIVE_SET_HH__

#include <type_traits>

#include "base/cache/associative_cache.hh"
#include "mem/cache/replacement_policies/base.hh"
#include "mem/cache/tags/indexing_policies/base.hh"
#include "mem/cache/tags/tagged_entry.hh"

namespace gem5
{

/**
 * Associative container based on the previosuly defined Entry type
 * Each element is indexed by a key of type Addr, an additional
 * bool value is used as an additional tag data of the entry.
 */
template<class Entry>
class AssociativeSet : public AssociativeCache<Entry>
{
    static_assert(std::is_base_of_v<TaggedEntry, Entry>,
                  "Entry must derive from TaggedEntry");

  public:
    /**
     * Public constructor
     * @param name Name of the cache
     * @param num_entries total number of entries of the container, the number
     *        of sets can be calculated dividing this balue by the 'assoc' value
     * @param assoc number of elements in each associative set
     * @param rpl_policy replacement policy
     * @param idx_policy indexing policy
     * @param init_val initial value of the elements of the set
     */
    AssociativeSet(const char *name, const size_t num_entries,
                   const size_t associativity_,
                   replacement_policy::Base *repl_policy,
                   typename Entry::IndexingPolicy *indexing_policy,
                   Entry const &init_val = Entry());

  private:
    // The following APIs are excluded since they lack the secure bit
    using AssociativeCache<Entry>::findEntry;
    using AssociativeCache<Entry>::insertEntry;
    using AssociativeCache<Entry>::getPossibleEntries;
    using AssociativeCache<Entry>::replPolicy;
    using AssociativeCache<Entry>::indexingPolicy;
};

} // namespace gem5

#endif//__CACHE_PREFETCH_ASSOCIATIVE_SET_HH__
