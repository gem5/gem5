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

#include "base/bitfield.hh"
#include "base/compiler.hh"
#include "cpu/static_inst_fwd.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(DecodeCache, decode_cache);
namespace decode_cache
{

/// Hash for decoded instructions.
template <typename EMI>
using InstMap = std::unordered_map<EMI, StaticInstPtr>;

/// A sparse map from an Addr to a Value, stored in page chunks.
template<class Value, Addr CacheChunkShift = 12>
class AddrMap
{
  protected:
    static constexpr Addr CacheChunkBytes = 1ULL << CacheChunkShift;

    static constexpr Addr
    chunkOffset(Addr addr)
    {
        return addr & (CacheChunkBytes - 1);
    }

    static constexpr Addr
    chunkStart(Addr addr)
    {
        return addr & ~(CacheChunkBytes - 1);
    }

    // A chunk of cache entries.
    struct CacheChunk
    {
        Value items[CacheChunkBytes];
    };
    // A map of cache chunks which allows a sparse mapping.
    typedef typename std::unordered_map<Addr, CacheChunk *> ChunkMap;
    typedef typename ChunkMap::iterator ChunkIt;
    // Mini cache of recent lookups.
    ChunkIt recent[2];
    ChunkMap chunkMap;

    /// Update the mini cache of recent lookups.
    /// @param recentest The most recent result;
    void
    update(ChunkIt recentest)
    {
        recent[1] = recent[0];
        recent[0] = recentest;
    }

    /// Attempt to find the CacheChunk which goes with a particular
    /// address. First check the small cache of recent results, then
    /// actually look in the hash map.
    /// @param addr The address to look up.
    CacheChunk *
    getChunk(Addr addr)
    {
        Addr chunk_addr = chunkStart(addr);

        // Check against recent lookups.
        if (recent[0] != chunkMap.end()) {
            if (recent[0]->first == chunk_addr)
                return recent[0]->second;
            if (recent[1] != chunkMap.end() &&
                    recent[1]->first == chunk_addr) {
                update(recent[1]);
                // recent[1] has just become recent[0].
                return recent[0]->second;
            }
        }

        // Actually look in the hash_map.
        ChunkIt it = chunkMap.find(chunk_addr);
        if (it != chunkMap.end()) {
            update(it);
            return it->second;
        }

        // Didn't find an existing chunk, so add a new one.
        CacheChunk *newChunk = new CacheChunk;
        typename ChunkMap::value_type to_insert(chunk_addr, newChunk);
        update(chunkMap.insert(to_insert).first);
        return newChunk;
    }

  public:
    /// Constructor
    AddrMap()
    {
        recent[0] = recent[1] = chunkMap.end();
    }

    Value &
    lookup(Addr addr)
    {
        CacheChunk *chunk = getChunk(addr);
        return chunk->items[chunkOffset(addr)];
    }
};

} // namespace decode_cache
} // namespace gem5

#endif // __CPU_DECODE_CACHE_HH__
