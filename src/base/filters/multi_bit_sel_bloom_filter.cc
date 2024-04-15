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

#include "base/filters/multi_bit_sel_bloom_filter.hh"

#include <limits>

#include "base/bitfield.hh"
#include "base/logging.hh"
#include "params/BloomFilterMultiBitSel.hh"

namespace gem5
{

namespace bloom_filter
{

MultiBitSel::MultiBitSel(const BloomFilterMultiBitSelParams &p)
    : Base(p),
      numHashes(p.num_hashes),
      parFilterSize(p.size / numHashes),
      isParallel(p.is_parallel),
      skipBits(p.skip_bits)
{
    if (p.size % numHashes) {
        fatal("Can't divide filter (%d) in %d equal portions", p.size,
              numHashes);
    }
}

MultiBitSel::~MultiBitSel() {}

void
MultiBitSel::set(Addr addr)
{
    for (int i = 0; i < numHashes; i++) {
        filter[hash(addr, i)]++;
    }
}

int
MultiBitSel::getCount(Addr addr) const
{
    int count = 0;
    for (int i = 0; i < numHashes; i++) {
        count += filter[hash(addr, i)];
    }
    return count;
}

int
MultiBitSel::hash(Addr addr, int hash_number) const
{
    uint64_t value =
        bits(addr, std::numeric_limits<Addr>::digits - 1, offsetBits) >>
        skipBits;
    const int max_bits = std::numeric_limits<Addr>::digits - offsetBits;
    int result = 0;
    int bit, i;

    for (i = 0; i < sizeBits; i++) {
        bit = (hash_number + numHashes * i) % max_bits;
        if (value & (1 << bit)) {
            result += 1 << i;
        }
    }

    if (isParallel) {
        return (result % parFilterSize) + hash_number * parFilterSize;
    } else {
        return result % filter.size();
    }
}

} // namespace bloom_filter
} // namespace gem5
