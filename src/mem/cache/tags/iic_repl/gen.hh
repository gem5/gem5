/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 * Authors: Erik Hallnor
 */

/**
 * @file
 * Declarations of generational replacement policy
 */

#ifndef ___GEN_HH__
#define __GEN_HH__

#include <list>

#include "base/statistics.hh"
#include "mem/cache/tags/iic_repl/repl.hh"
#include "params/GenRepl.hh"

/**
 * Generational Replacement entry.
 */
class GenReplEntry
{
  public:
    /** Valid flag, used to quickly invalidate bogus entries. */
    bool valid;
    /** The difference between this entry and the previous in the pool. */
    int delta;
    /** Pointer to the corresponding tag in the IIC. */
    unsigned long tag_ptr;
};

/**
 * Generational replacement pool
 */
class GenPool
{
  public:
    /** The time the last entry was added. */
    Tick newest;
    /** The time the oldest entry was added. */
    Tick oldest;
    /** List of the replacement entries in this pool. */
    std::list<GenReplEntry*> entries;

    /** The number of entries in this pool. */
    int size;

    /**
     * Simple constructor.
     */
    GenPool() {
        newest = 0;
        oldest = 0;
        size = 0;
    }

    /**
     * Add an entry to this pool.
     * @param re The entry to add.
     * @param now The current time.
     */
    void push(GenReplEntry *re, Tick now) {
        ++size;
        if (!entries.empty()) {
            re->delta = now - newest;
            newest = now;
        } else {
            re->delta = 0;
            newest = oldest = now;
        }
        entries.push_back(re);
    }

    /**
     * Remove an entry from the pool.
     * @return The entry at the front of the list.
     */
    GenReplEntry* pop() {
        GenReplEntry *tmp = NULL;
        if (!entries.empty()) {
            --size;
            tmp = entries.front();
            entries.pop_front();
            oldest += tmp->delta;
        }
        return tmp;
    }

    /**
     * Return the entry at the front of the list.
     * @return the entry at the front of the list.
     */
    GenReplEntry* top() {
        return entries.front();
    }

    /**
     * Destructor.
     */
    ~GenPool() {
        while (!entries.empty()) {
            GenReplEntry *tmp = entries.front();
            entries.pop_front();
            delete tmp;
        }
    }
};

/**
 * Generational replacement policy for use with the IIC.
 * @todo update to use STL and for efficiency
 */
class GenRepl : public Repl
{
  public:
    /** The number of pools. */
    int num_pools;
    /** The amount of time to stay in the fresh pool. */
    int fresh_res;
    /** The amount of time to stay in the normal pools. */
    int pool_res;
    /** The maximum number of entries */
    int num_entries;
    /** The number of entries currently in the pools. */
    int num_pool_entries;
    /** The number of misses. Used as the internal time. */
    Tick misses;
    /** The array of pools. */
    GenPool *pools;

    // Statistics

    /**
     * @addtogroup CacheStatistics
     * @{
     */
    /** The number of replacements from each pool. */
    Stats::Distribution repl_pool;
    /** The number of advances out of each pool. */
    Stats::Distribution advance_pool;
    /** The number of demotions from each pool. */
    Stats::Distribution demote_pool;
    /**
     * @}
     */

    typedef GenReplParams Params;
    GenRepl(const Params *p);

    /**
     * Destructor.
     */
    ~GenRepl();

    /**
     * Returns the tag pointer of the cache block to replace.
     * @return The tag to replace.
     */
    virtual unsigned long getRepl();

    /**
     * Return an array of N tag pointers to replace.
     * @param n The number of tag pointer to return.
     * @return An array of tag pointers to replace.
     */
    virtual unsigned long *getNRepl(int n);

    /**
     * Update replacement data
     */
    virtual void doAdvance(std::list<unsigned long> &demoted);

    /**
     * Add a tag to the replacement policy and return a pointer to the
     * replacement entry.
     * @param tag_index The tag to add.
     * @return The replacement entry.
     */
    virtual void* add(unsigned long tag_index);

    /**
     * Register statistics.
     * @param name The name to prepend to statistic descriptions.
     */
    virtual void regStatsWithSuffix(const std::string name);

    /**
     * Update the tag pointer to when the tag moves.
     * @param re The replacement entry of the tag.
     * @param old_index The old tag pointer.
     * @param new_index The new tag pointer.
     * @return 1 if successful, 0 otherwise.
     */
    virtual int fixTag(void *re, unsigned long old_index,
                       unsigned long new_index);

    /**
     * Remove this entry from the replacement policy.
     * @param re The replacement entry to remove
     */
    virtual void removeEntry(void *re)
    {
        ((GenReplEntry*)re)->valid = false;
    }

  protected:
    /**
     * Debug function to verify that there is only one repl entry per tag.
     * @param index The tag index to check.
     */
    bool findTagPtr(unsigned long index);
};

#endif /* __GEN_HH__ */
