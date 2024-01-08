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

#ifndef __CACHE_PREFETCH_ASSOCIATIVE_SET_IMPL_HH__
#define __CACHE_PREFETCH_ASSOCIATIVE_SET_IMPL_HH__

#include "base/intmath.hh"
#include "mem/cache/prefetch/associative_set.hh"

namespace gem5
{

template<class Entry>
AssociativeSet<Entry>::AssociativeSet(int assoc, int num_entries,
        BaseIndexingPolicy *idx_policy, replacement_policy::Base *rpl_policy,
        Entry const &init_value)
  : associativity(assoc), numEntries(num_entries), indexingPolicy(idx_policy),
    replacementPolicy(rpl_policy), entries(numEntries, init_value)
{
    fatal_if(!isPowerOf2(num_entries), "The number of entries of an "
             "AssociativeSet<> must be a power of 2");
    fatal_if(!isPowerOf2(assoc), "The associativity of an AssociativeSet<> "
             "must be a power of 2");
    for (unsigned int entry_idx = 0; entry_idx < numEntries; entry_idx += 1) {
        Entry* entry = &entries[entry_idx];
        indexingPolicy->setEntry(entry, entry_idx);
        entry->replacementData = replacementPolicy->instantiateEntry();
    }
}

template<class Entry>
Entry*
AssociativeSet<Entry>::findEntry(Addr addr, bool is_secure) const
{
    Addr tag = indexingPolicy->extractTag(addr);
    const std::vector<ReplaceableEntry*> selected_entries =
        indexingPolicy->getPossibleEntries(addr);

    for (const auto& location : selected_entries) {
        Entry* entry = static_cast<Entry *>(location);
        if ((entry->getTag() == tag) && entry->isValid() &&
            entry->isSecure() == is_secure) {
            return entry;
        }
    }
    return nullptr;
}

template<class Entry>
void
AssociativeSet<Entry>::accessEntry(Entry *entry)
{
    replacementPolicy->touch(entry->replacementData);
}

template<class Entry>
Entry*
AssociativeSet<Entry>::findVictim(Addr addr)
{
    // Get possible entries to be victimized
    const std::vector<ReplaceableEntry*> selected_entries =
        indexingPolicy->getPossibleEntries(addr);
    Entry* victim = static_cast<Entry*>(replacementPolicy->getVictim(
                            selected_entries));
    // There is only one eviction for this replacement
    invalidate(victim);
    return victim;
}


template<class Entry>
std::vector<Entry *>
AssociativeSet<Entry>::getPossibleEntries(const Addr addr) const
{
    std::vector<ReplaceableEntry *> selected_entries =
        indexingPolicy->getPossibleEntries(addr);
    std::vector<Entry *> entries(selected_entries.size(), nullptr);

    unsigned int idx = 0;
    for (auto &entry : selected_entries) {
        entries[idx++] = static_cast<Entry *>(entry);
    }
    return entries;
}

template<class Entry>
void
AssociativeSet<Entry>::insertEntry(Addr addr, bool is_secure, Entry* entry)
{
   entry->insert(indexingPolicy->extractTag(addr), is_secure);
   replacementPolicy->reset(entry->replacementData);
}

template<class Entry>
void
AssociativeSet<Entry>::invalidate(Entry* entry)
{
    entry->invalidate();
    replacementPolicy->invalidate(entry->replacementData);
}

} // namespace gem5

#endif//__CACHE_PREFETCH_ASSOCIATIVE_SET_IMPL_HH__
