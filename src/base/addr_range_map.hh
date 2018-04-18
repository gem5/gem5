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

#include <map>
#include <utility>

#include "base/addr_range.hh"

/**
 * The AddrRangeMap uses an STL map to implement an interval tree for
 * address decoding. The value stored is a template type and can be
 * e.g. a port identifier, or a pointer.
 */
template <typename V>
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
        tree.erase(p);
    }

    void
    erase(iterator p, iterator q)
    {
        tree.erase(p,q);
    }

    void
    clear()
    {
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
        const_iterator next = tree.upper_bound(r);
        if (next != end() && cond(next->first)) {
            return next;
        }
        if (next == begin())
            return end();
        next--;

        const_iterator i;
        do {
            i = next;
            if (cond(i->first)) {
                return i;
            }
            // Keep looking if the next range merges with the current one.
        } while (next != begin() &&
                 (--next)->first.mergesWith(i->first));

        return end();
    }

    RangeMap tree;
};

#endif //__BASE_ADDR_RANGE_MAP_HH__
