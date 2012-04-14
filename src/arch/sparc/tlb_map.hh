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

#ifndef __ARCH_SPARC_TLB_MAP_HH__
#define __ARCH_SPARC_TLB_MAP_HH__

#include <map>

#include "arch/sparc/pagetable.hh"

namespace SparcISA
{

class TlbMap
{
  private:
    typedef std::map<TlbRange, TlbEntry*> RangeMap;
    RangeMap tree;

  public:
    typedef RangeMap::iterator iterator;

    iterator
    find(const TlbRange &r)
    {
        iterator i;

        i = tree.upper_bound(r);

        if (i == tree.begin()) {
            if (r.real == i->first.real &&
                r.partitionId == i->first.partitionId &&
                i->first.va < r.va + r.size &&
                i->first.va+i->first.size >= r.va &&
                (r.real || r.contextId == i->first.contextId))
                return i;
            else
                // Nothing could match, so return end()
                return tree.end();
        }

        i--;

        if (r.real != i->first.real)
            return tree.end();
        if (!r.real && r.contextId != i->first.contextId)
            return tree.end();
        if (r.partitionId != i->first.partitionId)
            return tree.end();
        if (i->first.va <= r.va+r.size &&
            i->first.va+i->first.size >= r.va)
            return i;

        return tree.end();
    }

    bool
    intersect(const TlbRange &r)
    {
        iterator i;
        i = find(r);
        if (i != tree.end())
            return true;
        return false;
    }


    iterator
    insert(TlbRange &r, TlbEntry *d)
    {
        if (intersect(r))
            return tree.end();

        return tree.insert(std::make_pair(r, d)).first;
    }

    size_t
    erase(TlbRange k)
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

    void
    print()
    {
        iterator i;
        i = tree.begin();
        while (i != tree.end()) {
           std::cout << std::hex << i->first.va << " " << i->first.size << " " <<
                i->first.contextId << " " << i->first.partitionId << " " <<
                i->first.real << " " << i->second << std::endl;
            i++;
        }
    }

};

};

#endif // __ARCH_SPARC_TLB_MAP_HH__
