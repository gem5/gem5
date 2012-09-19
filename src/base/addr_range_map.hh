/*
 * Copyright (c) 2012 ARM Limited
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
    RangeMap tree;

  public:
    typedef typename RangeMap::iterator iterator;
    typedef typename RangeMap::const_iterator const_iterator;

    const_iterator
    find(const AddrRange &r) const
    {
        const_iterator i;

        i = tree.upper_bound(r);

        if (i == tree.begin()) {
            if (i->first.start <= r.end && i->first.end >= r.start)
                return i;
            else
                // Nothing could match, so return end()
                return tree.end();
        }

        --i;

        if (i->first.start <= r.end && i->first.end >= r.start)
            return i;

        return tree.end();
    }

    iterator
    find(const AddrRange &r)
    {
        iterator i;

        i = tree.upper_bound(r);

        if (i == tree.begin()) {
            if (i->first.start <= r.end && i->first.end >= r.start)
                return i;
            else
                // Nothing could match, so return end()
                return tree.end();
        }

        --i;

        if (i->first.start <= r.end && i->first.end >= r.start)
            return i;

        return tree.end();
    }

    const_iterator
    find(const Addr &r) const
    {
        return find(RangeSize(r, 1));
    }

    iterator
    find(const Addr &r)
    {
        return find(RangeSize(r, 1));
    }

    bool
    intersect(const AddrRange &r)
    {
        iterator i;
        i = find(r);
        if (i != tree.end())
            return true;
        return false;
    }

    iterator
    insert(const AddrRange &r, const V& d)
    {
        if (intersect(r))
            return tree.end();

        return tree.insert(std::make_pair(r, d)).first;
    }

    std::size_t
    erase(Addr k)
    {
        return tree.erase(k);
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
};

#endif //__BASE_ADDR_RANGE_MAP_HH__
