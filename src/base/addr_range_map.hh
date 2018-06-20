/*
 * Copyright (c) 2012, 2018 ARM Limited
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
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 * Authors: Ali Saidi
 *          Andreas Hansson
 */

#ifndef __BASE_ADDR_RANGE_MAP_HH__
#define __BASE_ADDR_RANGE_MAP_HH__

#include <cstddef>
#include <functional>
#include <list>
#include <map>
#include <utility>

#include "base/addr_range.hh"
#include "base/types.hh"

/**
 * The AddrRangeMap uses an STL map to implement an interval tree for
 * address decoding. The value stored is a template type and can be
 * e.g. a port identifier, or a pointer.
 */
template <typename V, int max_cache_size=0>
class AddrRangeMap
{
  private:
    typedef std::map<AddrRange, V> RangeMap;

  public:
    typedef typename RangeMap::iterator iterator;
    typedef typename RangeMap::const_iterator const_iterator;

    /**
     * Find entry that contains the given address range
     *
     * Searches through the ranges in the address map and returns an
     * iterator to the entry which range is a superset of the input
     * address range. Returns end() if none found.
     *
     * @param r An input address range
     * @return An iterator that contains the input address range
     */
    const_iterator
    contains(const AddrRange &r) const
    {
        return find(r, [r](const AddrRange r1) { return r.isSubset(r1); });
    }

    /**
     * Find entry that contains the given address
     *
     * Searches through the ranges in the address map and returns an
     * iterator to the entry which range is a superset of the input
     * address. Returns end() if none found.
     *
     * @param r An input address
     * @return An iterator that contains the input address
     */
    const_iterator
    contains(Addr r) const
    {
        return contains(RangeSize(r, 1));
    }

    /**
     * Find entry that intersects with the given address range
     *
     * Searches through the ranges in the address map and returns an
     * iterator to the first entry which range intersects with the
     * input address.
     *
     * @param r An input address
     * @return An iterator that intersects with the input address range
     */
    const_iterator
    intersects(const AddrRange &r) const
    {
        return find(r, [r](const AddrRange r1) { return r.intersects(r1); });
    }

    const_iterator
    insert(const AddrRange &r, const V& d)
    {
        if (intersects(r) != end())
            return tree.end();

        return tree.insert(std::make_pair(r, d)).first;
    }

    void
    erase(iterator p)
    {
        cache.remove(p);
        tree.erase(p);
    }

    void
    erase(iterator p, iterator q)
    {
        for (auto it = p; it != q; it++) {
            cache.remove(p);
        }
        tree.erase(p,q);
    }

    void
    clear()
    {
        cache.erase(cache.begin(), cache.end());
        tree.erase(tree.begin(), tree.end());
    }

    const_iterator
    begin() const
    {
        return tree.begin();
    }

    iterator
    begin()
    {
        return tree.begin();
    }

    const_iterator
    end() const
    {
        return tree.end();
    }

    iterator
    end()
    {
        return tree.end();
    }

    std::size_t
    size() const
    {
        return tree.size();
    }

    bool
    empty() const
    {
        return tree.empty();
    }

  private:
    /**
     * Add an address range map entry to the cache.
     *
     * @param it Iterator to the entry in the address range map
     */
    void
    addNewEntryToCache(const_iterator it) const
    {
        if (max_cache_size != 0) {
            // If there's a cache, add this element to it.
            if (cache.size() >= max_cache_size) {
                // If the cache is full, move the last element to the
                // front and overwrite it with the new value. This
                // avoids creating or destroying elements of the list.
                auto last = cache.end();
                last--;
                *last = it;
                if (max_cache_size > 1)
                    cache.splice(cache.begin(), cache, last);
            } else {
                cache.push_front(it);
            }
        }
    }

    /**
     * Find entry that satisfies a condition on an address range
     *
     * Searches through the ranges in the address map and returns an
     * iterator to the entry that satisfies the input conidition on
     * the input address range. Returns end() if none found.
     *
     * @param r An input address range
     * @param f A condition on an address range
     * @return An iterator that contains the input address range
     */
    const_iterator
    find(const AddrRange &r, std::function<bool(const AddrRange)> cond) const
    {
        // Check the cache first
        for (auto c = cache.begin(); c != cache.end(); c++) {
            auto it = *c;
            if (cond(it->first)) {
                // If this entry matches, promote it to the front
                // of the cache and return it.
                cache.splice(cache.begin(), cache, c);
                return it;
            }
        }

        const_iterator next = tree.upper_bound(r);
        if (next != end() && cond(next->first)) {
            addNewEntryToCache(next);
            return next;
        }
        if (next == begin())
            return end();
        next--;

        const_iterator i;
        do {
            i = next;
            if (cond(i->first)) {
                addNewEntryToCache(i);
                return i;
            }
            // Keep looking if the next range merges with the current one.
        } while (next != begin() &&
                 (--next)->first.mergesWith(i->first));

        return end();
    }

    RangeMap tree;

    /**
     * A list of iterator that correspond to the max_cache_size most
     * recently used entries in the address range map. This mainly
     * used to optimize lookups. The elements in the list should
     * always be valid iterators of the tree.
     */
    mutable std::list<const_iterator> cache;
};

#endif //__BASE_ADDR_RANGE_MAP_HH__
