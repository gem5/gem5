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
 *
 * Authors: Javier Bueno
 */

#ifndef __CACHE_PREFETCH_ASSOCIATIVE_SET_HH__
#define __CACHE_PREFETCH_ASSOCIATIVE_SET_HH__

#include "mem/cache/replacement_policies/base.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"
#include "mem/cache/tags/indexing_policies/base.hh"

/**
 * Entry used for set-associative tables, usable with replacement policies
 */
class TaggedEntry : public ReplaceableEntry {
    /** Tag for the entry */
    Addr tag;
    /** Valid bit */
    bool valid;
    /** Whether this entry refers to a memory area in the secure space */
    bool secure;
  public:
    TaggedEntry() : tag(0), valid(false), secure(false) {}
    virtual ~TaggedEntry() {}

    /**
     * Consult the valid bit
     * @return True if the entry is valid
     */
    bool isValid() const
    {
        return valid;
    }

    /**
     * Sets the entry to valid
     */
    void setValid()
    {
        valid = true;
    }

    /**
     * Sets the entry to invalid
     */
    void setInvalid() {
        valid = false;
    }

    /**
     * Obtain the entry tag
     * @return the tag value
     */
    Addr getTag() const
    {
        return tag;
    }

    /**
     * Sets the tag of the entry
     * @param t the tag value
     */
    void setTag(Addr t)
    {
        tag = t;
    }

    /**
     * Consult if this entry refers to a memory in the secure area
     * @return True if this entry refers to secure memory area
     */
    bool isSecure() const
    {
        return secure;
    }

    /**
     * Sets the secure value bit
     * @param s secure bit value
     */
    void setSecure(bool s)
    {
        secure = s;
    }

    /**
     * Resets the entry, this is called when an entry is evicted to allocate
     * a new one. Types inheriting this class should provide its own
     * implementation
     */
    virtual void reset () {
    }
};

/**
 * Associative container based on the previosuly defined Entry type
 * Each element is indexed by a key of type Addr, an additional
 * bool value is used as an additional tag data of the entry.
 */
template<class Entry>
class AssociativeSet {
    static_assert(std::is_base_of<TaggedEntry, Entry>::value,
                  "Entry must derive from TaggedEntry");

    /** Associativity of the container */
    const int associativity;
    /**
     * Total number of entries, entries are organized in sets of the provided
     * associativity. The number of associative sets is obtained by dividing
     * numEntries by associativity.
     */
    const int numEntries;
    /** Pointer to the indexing policy */
    BaseIndexingPolicy* const indexingPolicy;
    /** Pointer to the replacement policy */
    BaseReplacementPolicy* const replacementPolicy;
    /** Vector containing the entries of the container */
    std::vector<Entry> entries;

  public:
    /**
     * Public constructor
     * @param assoc number of elements in each associative set
     * @param num_entries total number of entries of the container, the number
     *   of sets can be calculated dividing this balue by the 'assoc' value
     * @param idx_policy indexing policy
     * @param rpl_policy replacement policy
     * @param initial value of the elements of the set
     */
    AssociativeSet(int assoc, int num_entries, BaseIndexingPolicy *idx_policy,
                   BaseReplacementPolicy *rpl_policy, Entry const &init_value =
                   Entry());

    /**
     * Find an entry within the set
     * @param addr key element
     * @param is_secure tag element
     * @return returns a pointer to the wanted entry or nullptr if it does not
     *  exist.
     */
    Entry* findEntry(Addr addr, bool is_secure) const;

    /**
     * Do an access to the entry, this is required to
     * update the replacement information data.
     * @param entry the accessed entry
     */
    void accessEntry(Entry *entry);

    /**
     * Find a victim to be replaced
     * @param addr key to select the possible victim
     * @result entry to be victimized
     */
    Entry* findVictim(Addr addr);

    /**
     * Find the set of entries that could be replaced given
     * that we want to add a new entry with the provided key
     * @param addr key to select the set of entries
     * @result vector of candidates matching with the provided key
     */
    std::vector<Entry *> getPossibleEntries(const Addr addr) const;

    /**
     * Indicate that an entry has just been inserted
     * @param addr key of the container
     * @param is_secure tag component of the container
     * @param entry pointer to the container entry to be inserted
     */
    void insertEntry(Addr addr, bool is_secure, Entry* entry);

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

#endif//__CACHE_PREFETCH_ASSOCIATIVE_SET_HH__
