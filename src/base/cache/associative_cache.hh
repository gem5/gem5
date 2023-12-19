/*
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
#include "base/cache/replacement_policies/base.hh"
#include "base/cache/replacement_policies/replaceable_entry.hh"
#include "base/cache/indexing_policies/base.hh"
#include "base/types.hh"

namespace gem5
{

/** Cache Library.
 *
 */
template <typename Entry>
class AssociativeCache : public Named
{
    static_assert(std::is_base_of_v<CacheEntry, Entry>,
                  "Entry should be derived from CacheEntry");

    typedef replacement_policy::Base BaseReplacementPolicy;

  protected:

    /** Associativity of the cache. */
    size_t associativity;

    /** The replacement policy of the cache. */
    BaseReplacementPolicy *replPolicy;

    /** Indexing policy of the cache */
    BaseIndexingPolicy *indexingPolicy;

    /** The entries */
    std::vector<Entry> entries;

  private:

    void initParams(size_t _num_entries, size_t _assoc)
    {
        fatal_if(!isPowerOf2(_num_entries), "The number of entries of an "
                 "AssociativeCache<> must be a power of 2");
        fatal_if(!isPowerOf2(_assoc), "The associativity of an "
                 "AssociativeCache<> must be a power of 2");
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
     * @param entry_size Size of each entry
     * @param repl_policy replacement policy
     * @param indexing_policy indexing policy
     */
    AssociativeCache(const char *name, const size_t num_entries,
                     const size_t associativity_,
                     BaseReplacementPolicy *repl_policy,
                     BaseIndexingPolicy *indexing_policy,
                     Entry const &init_val = Entry())
        : Named(std::string(name)),
          associativity(associativity_),
          replPolicy(repl_policy),
          indexingPolicy(indexing_policy),
          entries(num_entries, init_val)
    {
        initParams(num_entries, associativity);
        printf("Constructor called for %s\n", name);
    }

    ~AssociativeCache() {
        printf("Destructor called for %s\n", name().c_str());
    }

    /**
     * Disable copy and assignment
     */
    AssociativeCache(const AssociativeCache&) = delete;
    AssociativeCache& operator=(const AssociativeCache&) = delete;

    /**
     * Clear the entries in the cache.
     */
    void clear() {
        for (auto &entry : entries) {
            invalidate(&entry);
        }
    }

    void init(const size_t num_entries,
              const size_t associativity_,
              BaseReplacementPolicy *_repl_policy,
              BaseIndexingPolicy *_indexing_policy,
              Entry const &init_val = Entry())
    {
        associativity        = associativity_;
        replPolicy           = _repl_policy;
        indexingPolicy       = _indexing_policy;
        entries.resize(num_entries, init_val);

        initParams(num_entries, associativity);
    }

    /**
     * Get the tag for the addr
     * @param addr Addr to get the tag for
     * @return Tag for the address
     */
    virtual Addr getTag(const Addr addr) const
    {
        return indexingPolicy->extractTag(addr);
    }

    /**
     * Do an access to the entry if it exists.
     * This is required to update the replacement information data.
     * @param addr key to the entry
     * @return The entry if it exists
     */
    virtual Entry* accessEntry(const Addr addr)
    {
        auto entry = findEntry(addr);

        if (entry) {
            replPolicy->touch(entry->replacementData);
        }

        return entry;
    }

    /**
     * Update the replacement information for an entry
     * @param Entry to access and upate
     */
    virtual void accessEntry(Entry *entry)
    {
        replPolicy->touch(entry->replacementData);
    }

    /**
     * Find an entry within the set
     * @param addr key element
     * @return returns a pointer to the wanted entry or nullptr if it does not
     *  exist.
     */
    virtual Entry* findEntry(const Addr addr) const
    {
        auto tag   = getTag(addr);

        auto candidates = indexingPolicy->getPossibleEntries(addr);

        for (auto candidate : candidates) {
            Entry *entry = static_cast<Entry*>(candidate);
            if (entry->matchTag(tag)) {
                return entry;
            }
        }

        return nullptr;
    }

    /**
     * Find a victim to be replaced
     * @param addr key to select the possible victim
     * @result entry to be victimized
     */
    virtual Entry* findVictim(const Addr addr)
    {
        auto candidates = indexingPolicy->getPossibleEntries(addr);

        auto victim = static_cast<Entry*>(replPolicy->getVictim(candidates));

        invalidate(victim);

        return victim;
    }

    /**
     * Invalidate an entry and its respective replacement data.
     *
     * @param entry Entry to be invalidated.
     */
    virtual void invalidate(Entry *entry)
    {
        entry->invalidate();
        replPolicy->invalidate(entry->replacementData);
    }

    /**
     * Indicate that an entry has just been inserted
     * @param addr key of the container
     * @param entry pointer to the container entry to be inserted
     */
    virtual void insertEntry(Addr addr, Entry *entry)
    {
        entry->insert(indexingPolicy->extractTag(addr));
        replPolicy->reset(entry->replacementData);
    }

    /** Iterator types */
    using const_iterator = typename std::vector<Entry>::const_iterator;
    using iterator = typename std::vector<Entry>::iterator;

    /**
     * Returns an iterator to the first entry of the dictionary
     * @result iterator to the first element
     */
    iterator begin()
    {
        return entries.begin();
    }

    /**
     * Returns an iterator pointing to the end of the the dictionary
     * (placeholder element, should not be accessed)
     * @result iterator to the end element
     */
    iterator end()
    {
        return entries.end();
    }

    /**
     * Returns an iterator to the first entry of the dictionary
     * @result iterator to the first element
     */
    const_iterator begin() const
    {
        return entries.begin();
    }

    /**
     * Returns an iterator pointing to the end of the the dictionary
     * (placeholder element, should not be accessed)
     * @result iterator to the end element
     */
    const_iterator end() const
    {
        return entries.end();
    }
};

}

#endif
