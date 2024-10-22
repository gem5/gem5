/*
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
 * Copyright (c) 2024 Pranith Kumar
 * Copyright (c) 2018 Metempsy Technology Consulting
 * All rights reserved
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

#ifndef __BASE_CACHE_ASSOCIATIVE_CACHE_HH__
#define __BASE_CACHE_ASSOCIATIVE_CACHE_HH__

#include <type_traits>
#include <vector>

#include "base/cache/cache_entry.hh"
#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/named.hh"
#include "base/types.hh"
#include "mem/cache/replacement_policies/base.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"
#include "mem/cache/tags/indexing_policies/base.hh"

namespace gem5
{

template <typename Entry>
class AssociativeCache : public Named
{
  protected:

    static_assert(std::is_base_of_v<ReplaceableEntry, Entry>,
                  "Entry should be derived from ReplaceableEntry");

    typedef replacement_policy::Base BaseReplacementPolicy;
    typedef typename Entry::IndexingPolicy IndexingPolicy;
    typedef typename Entry::KeyType KeyType;

    /** Associativity of the cache. */
    size_t associativity;

    /** The replacement policy of the cache. */
    BaseReplacementPolicy *replPolicy;

    /** Indexing policy of the cache */
    IndexingPolicy *indexingPolicy;

    /** The entries */
    std::vector<Entry> entries;

    const ::gem5::debug::SimpleFlag* debugFlag = nullptr;

  private:

    void
    initParams(size_t _num_entries, size_t _assoc)
    {
        fatal_if((_num_entries % _assoc) != 0, "The number of entries of an "
                 "AssociativeCache<> must be a multiple of its associativity");
        for (auto entry_idx = 0; entry_idx < _num_entries; entry_idx++) {
            Entry *entry = &entries[entry_idx];
            indexingPolicy->setEntry(entry, entry_idx);
            entry->replacementData = replPolicy->instantiateEntry();
        }
    }

  public:

    /**
     * Empty constructor - need to call init() later with all args
     */
    AssociativeCache(const char *name) : Named(std::string(name)) {}

    /**
     * Public constructor
     * @param name Name of the cache
     * @param num_entries total number of entries of the container, the number
     *   of sets can be calculated dividing this balue by the 'assoc' value
     * @param associativity number of elements in each associative set
     * @param repl_policy replacement policy
     * @param indexing_policy indexing policy
     */
    AssociativeCache(const char *name, const size_t num_entries,
                     const size_t associativity_,
                     BaseReplacementPolicy *repl_policy,
                     IndexingPolicy *indexing_policy,
                     Entry const &init_val = Entry())
        : Named(std::string(name)),
          associativity(associativity_),
          replPolicy(repl_policy),
          indexingPolicy(indexing_policy),
          entries(num_entries, init_val),
          debugFlag(nullptr)
    {
        initParams(num_entries, associativity);
    }

    /**
     * Default destructor
     */
    ~AssociativeCache()  = default;

    /**
     * Disable copy and assignment
     */
    AssociativeCache(const AssociativeCache&) = delete;
    AssociativeCache& operator=(const AssociativeCache&) = delete;

    /**
     * Clear the entries in the cache.
     */
    void
    clear()
    {
        for (auto &entry : entries) {
            invalidate(&entry);
        }
    }

    void
    init(const size_t num_entries,
         const size_t associativity_,
         BaseReplacementPolicy *_repl_policy,
         IndexingPolicy *_indexing_policy,
         Entry const &init_val = Entry())
    {
        associativity = associativity_;
        replPolicy = _repl_policy;
        indexingPolicy = _indexing_policy;
        entries.resize(num_entries, init_val);

        initParams(num_entries, associativity);
    }

    void
    setDebugFlag(const ::gem5::debug::SimpleFlag& flag)
    {
        debugFlag = &flag;
    }

    /**
     * Do an access to the entry if it exists.
     * This is required to update the replacement information data.
     * @param key key to the entry
     * @return The entry if it exists
     */
    virtual Entry*
    accessEntry(const KeyType &key)
    {
        auto entry = findEntry(key);

        if (entry) {
            accessEntry(entry);
        }

        return entry;
    }

    /**
     * Update the replacement information for an entry
     * @param Entry to access and upate
     */
    virtual void
    accessEntry(Entry *entry)
    {
        replPolicy->touch(entry->replacementData);
    }

    /**
     * Find an entry within the set
     * @param key key element
     * @return returns a pointer to the wanted entry or nullptr if it does not
     *  exist.
     */
    virtual Entry*
    findEntry(const KeyType &key) const
    {
        auto candidates = indexingPolicy->getPossibleEntries(key);

        for (auto candidate : candidates) {
            Entry *entry = static_cast<Entry*>(candidate);
            if (entry->match(key)) {
                return entry;
            }
        }

        return nullptr;
    }

    /**
     * Find a victim to be replaced
     * @param key key to select the possible victim
     * @result entry to be victimized
     */
    virtual Entry*
    findVictim(const KeyType &key)
    {
        auto candidates = indexingPolicy->getPossibleEntries(key);

        auto victim = static_cast<Entry*>(replPolicy->getVictim(candidates));

        if (debugFlag && debugFlag->tracing() && victim->isValid()) {
            ::gem5::trace::getDebugLogger()->dprintf_flag(
                curTick(), name(), debugFlag->name(),
                "Replacing entry: %s\n", victim->print());
        }

        invalidate(victim);

        return victim;
    }

    /**
     * Invalidate an entry and its respective replacement data.
     *
     * @param entry Entry to be invalidated.
     */
    virtual void
    invalidate(Entry *entry)
    {
        entry->invalidate();
        replPolicy->invalidate(entry->replacementData);
    }

    /**
     * Indicate that an entry has just been inserted
     * @param key key of the container
     * @param entry pointer to the container entry to be inserted
     */
    virtual void
    insertEntry(const KeyType &key, Entry *entry)
    {
        if (debugFlag && debugFlag->tracing()) {
            ::gem5::trace::getDebugLogger()->dprintf_flag(
                curTick(), name(), debugFlag->name(),
                "Inserting entry: %s\n", entry->print());
        }

        entry->insert(key);
        replPolicy->reset(entry->replacementData);

    }

    /**
     * Find the set of entries that could be replaced given
     * that we want to add a new entry with the provided key
     * @param addr key to select the set of entries
     * @result vector of candidates matching with the provided key
     */
    std::vector<Entry *>
    getPossibleEntries(const KeyType &key) const
    {
        std::vector<ReplaceableEntry *> selected_entries =
            indexingPolicy->getPossibleEntries(key);

        std::vector<Entry *> entries;

        std::transform(selected_entries.begin(), selected_entries.end(),
                       std::back_inserter(entries), [](auto &entry) {
                           return static_cast<Entry *>(entry);
                       });

        return entries;
    }

    /** Iterator types */
    using const_iterator = typename std::vector<Entry>::const_iterator;
    using iterator = typename std::vector<Entry>::iterator;

    /**
     * Returns an iterator to the first entry of the dictionary
     * @result iterator to the first element
     */
    iterator
    begin()
    {
        return entries.begin();
    }

    /**
     * Returns an iterator pointing to the end of the the dictionary
     * (placeholder element, should not be accessed)
     * @result iterator to the end element
     */
    iterator
    end()
    {
        return entries.end();
    }

    /**
     * Returns an iterator to the first entry of the dictionary
     * @result iterator to the first element
     */
    const_iterator
    begin() const
    {
        return entries.begin();
    }

    /**
     * Returns an iterator pointing to the end of the the dictionary
     * (placeholder element, should not be accessed)
     * @result iterator to the end element
     */
    const_iterator
    end() const
    {
        return entries.end();
    }
};

}

#endif
