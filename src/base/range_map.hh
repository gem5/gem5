/*
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
 */

#ifndef __BASE_RANGE_MAP_HH__
#define __BASE_RANGE_MAP_HH__

#include <map>
#include <utility>

#include "base/range.hh"

/**
 * The range_map uses an STL map to implement an interval tree. The
 * type of both the key (range) and the value are template
 * parameters. It can, for example, be used for address decoding,
 * using a range of addresses to map to ports.
 */
template <class T,class V>
class range_map
{
  private:
    typedef std::map<Range<T>,V> RangeMap;
    RangeMap tree;

  public:
    typedef typename RangeMap::iterator iterator;
    typedef typename RangeMap::const_iterator const_iterator;

    template <class U>
    const_iterator
    find(const Range<U> &r) const
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

    template <class U>
    iterator
    find(const Range<U> &r)
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

    template <class U>
    const_iterator
    find(const U &r) const
    {
        return find(RangeSize(r, 1));
    }

    template <class U>
    iterator
    find(const U &r)
    {
        return find(RangeSize(r, 1));
    }

    template <class U>
    bool
    intersect(const Range<U> &r)
    {
        iterator i;
        i = find(r);
        if (i != tree.end())
            return true;
        return false;
    }

    template <class U,class W>
    iterator
    insert(const Range<U> &r, const W d)
    {
        if (intersect(r))
            return tree.end();

        return tree.insert(std::make_pair(r, d)).first;
    }

    size_t
    erase(T k)
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

    size_t
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


template <class T,class V>
class range_multimap
{
  private:
    typedef std::multimap<Range<T>,V> RangeMap;
    RangeMap tree;

  public:
    typedef typename RangeMap::iterator iterator;

    template <class U>
    std::pair<iterator,iterator> find(const Range<U> &r)
    {
        iterator i;
        iterator j;

        i = tree.lower_bound(r);

        if (i == tree.begin()) {
          if (i->first.start <= r.end && i->first.end >= r.start)
                return std::make_pair<iterator, iterator>(i,i);
          else
            // Nothing could match, so return end()
            return std::make_pair<iterator, iterator>(tree.end(), tree.end());
        }
        i--;

        if (i->first.start <= r.end && i->first.end >= r.start) {
            // we have at least one match
            j = i;

            i--;
            while (i->first.start <= r.end && i->first.end >=
                    r.start) {
                if (i == tree.begin())
                    break;
                i--;
            }
            if (i == tree.begin() && i->first.start <= r.end && i->first.end >=
                                        r.start)
                return std::make_pair<iterator, iterator>(i,j);
            i++;
            return std::make_pair<iterator, iterator>(i,j);

        }

        return std::make_pair<iterator, iterator>(tree.end(), tree.end());
    }

    template <class U>
    bool
    intersect(const Range<U> &r)
    {
        std::pair<iterator,iterator> p;
        p = find(r);
        if (p.first != tree.end())
            return true;
        return false;
    }


    template <class U,class W>
    iterator
    insert(const Range<U> &r, const W d)
    {
        std::pair<iterator,iterator> p;
        p = find(r);
        if ((p.first->first.start == r.start && p.first->first.end == r.end) ||
                p.first == tree.end())
            return tree.insert(std::make_pair<Range<T>,V>(r, d));
        else
            return tree.end();
    }

    size_t
    erase(T k)
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

    iterator
    begin()
    {
        return tree.begin();
    }

    iterator
    end()
    {
        return tree.end();
    }

    size_t
    size()
    {
        return tree.size();
    }

    bool
    empty()
    {
        return tree.empty();
    }
};

#endif //__BASE_RANGE_MAP_HH__
