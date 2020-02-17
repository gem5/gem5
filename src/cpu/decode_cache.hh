/*
 * Copyright (c) 2011 Google
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

#ifndef __CPU_DECODE_CACHE_HH__
#define __CPU_DECODE_CACHE_HH__

#include <unordered_map>

#include "arch/isa_traits.hh"
#include "arch/types.hh"
#include "config/the_isa.hh"
#include "cpu/static_inst_fwd.hh"

namespace TheISA
{
    class Decoder;
}

namespace DecodeCache
{

/// Hash for decoded instructions.
template <typename EMI>
using InstMap = std::unordered_map<EMI, StaticInstPtr>;

/// A sparse map from an Addr to a Value, stored in page chunks.
template<class Value>
class AddrMap
{
  protected:
    // A pages worth of cache entries.
    struct CachePage {
        Value items[TheISA::PageBytes];
    };
    // A map of cache pages which allows a sparse mapping.
    typedef typename std::unordered_map<Addr, CachePage *> PageMap;
    typedef typename PageMap::iterator PageIt;
    // Mini cache of recent lookups.
    PageIt recent[2];
    PageMap pageMap;

    /// Update the mini cache of recent lookups.
    /// @param recentest The most recent result;
    void
    update(PageIt recentest)
    {
        recent[1] = recent[0];
        recent[0] = recentest;
    }

    /// Attempt to find the CacheePage which goes with a particular
    /// address. First check the small cache of recent results, then
    /// actually look in the hash map.
    /// @param addr The address to look up.
    CachePage *
    getPage(Addr addr)
    {
        Addr page_addr = addr & ~(TheISA::PageBytes - 1);

        // Check against recent lookups.
        if (recent[0] != pageMap.end()) {
            if (recent[0]->first == page_addr)
                return recent[0]->second;
            if (recent[1] != pageMap.end() &&
                    recent[1]->first == page_addr) {
                update(recent[1]);
                // recent[1] has just become recent[0].
                return recent[0]->second;
            }
        }

        // Actually look in the has_map.
        PageIt it = pageMap.find(page_addr);
        if (it != pageMap.end()) {
            update(it);
            return it->second;
        }

        // Didn't find an existing page, so add a new one.
        CachePage *newPage = new CachePage;
        page_addr = page_addr & ~(TheISA::PageBytes - 1);
        typename PageMap::value_type to_insert(page_addr, newPage);
        update(pageMap.insert(to_insert).first);
        return newPage;
    }

  public:
    /// Constructor
    AddrMap()
    {
        recent[0] = recent[1] = pageMap.end();
    }

    Value &
    lookup(Addr addr)
    {
        CachePage *page = getPage(addr);
        return page->items[addr & (TheISA::PageBytes - 1)];
    }
};

} // namespace DecodeCache

#endif // __CPU_DECODE_CACHE_HH__
