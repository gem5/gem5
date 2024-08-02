/**
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
                   BaseIndexingPolicy *indexing_policy,
                   Entry const &init_val = Entry());

    /**
     * Find an entry within the set
     * @param addr key element
     * @param is_secure tag element
     * @return returns a pointer to the wanted entry or nullptr if it does not
     *  exist.
     */
    Entry* findEntry(Addr addr, bool is_secure) const;

    /**
     * Indicate that an entry has just been inserted
     * @param addr key of the container
     * @param is_secure tag component of the container
     * @param entry pointer to the container entry to be inserted
     */
    void insertEntry(Addr addr, bool is_secure, Entry* entry);

  private:
    // The following APIs are excluded since they lack the secure bit
    using AssociativeCache<Entry>::getTag;
    using AssociativeCache<Entry>::accessEntryByAddr;
    using AssociativeCache<Entry>::findEntry;
    using AssociativeCache<Entry>::insertEntry;
    using AssociativeCache<Entry>::replPolicy;
    using AssociativeCache<Entry>::indexingPolicy;
};

} // namespace gem5

#endif//__CACHE_PREFETCH_ASSOCIATIVE_SET_HH__
