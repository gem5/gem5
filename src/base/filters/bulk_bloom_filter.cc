/*
 * Copyright (c) 2019 Inria
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
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

#include "base/filters/bulk_bloom_filter.hh"

#include <limits>

#include "base/bitfield.hh"
#include "base/logging.hh"
#include "params/BloomFilterBulk.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(BloomFilter, bloom_filter);
namespace bloom_filter
{

Bulk::Bulk(const BloomFilterBulkParams &p)
    : MultiBitSel(p), sectorBits(floorLog2(parFilterSize))
{
    fatal_if((numHashes * sectorBits) >
        (std::numeric_limits<Addr>::digits - offsetBits),
        "Sectors need more bits than available");
}

Bulk::~Bulk()
{
}

int
Bulk::hash(Addr addr, int hash_number) const
{
    addr = permute(addr);

    // Get the sector-based c index
    int c = bits(addr, (offsetBits + (hash_number + 1) * sectorBits) - 1,
                 offsetBits + hash_number * sectorBits);
    assert(c < filter.size()/numHashes);

    // Transform the sector-based c index into a filder index (v)
    c += (numHashes - 1 - hash_number) * (filter.size()/numHashes);
    assert(c < filter.size());

    return c;
}

Addr
Bulk::permute(Addr addr) const
{
    // permutes the original address bits according to Table 5
    Addr part1  = bits(addr, offsetBits + 6, offsetBits),
         part2  = bits(addr, offsetBits + 9),
         part3  = bits(addr, offsetBits + 11),
         part4  = bits(addr, offsetBits + 17),
         part5  = bits(addr, offsetBits + 8, offsetBits + 7),
         part6  = bits(addr, offsetBits + 10),
         part7  = bits(addr, offsetBits + 12),
         part8  = bits(addr, offsetBits + 13),
         part9  = bits(addr, offsetBits + 16, offsetBits + 15),
         part10 = bits(addr, offsetBits + 20, offsetBits + 18),
         part11 = bits(addr, offsetBits + 14);

    Addr result =
        (part1 << 14) | (part2 << 13) | (part3 << 12) | (part4 << 11) |
        (part5 << 9)  | (part6 << 8)  | (part7 << 7)  | (part8 << 6)  |
        (part9 << 4)  | (part10 << 1) | (part11);

    // Select the remaining high-order bits
    Addr remaining_bits = bits(addr, std::numeric_limits<Addr>::digits - 1,
        offsetBits + 21) << 21;
    result = result | remaining_bits;

    return result;
}

} // namespace bloom_filter
} // namespace gem5
