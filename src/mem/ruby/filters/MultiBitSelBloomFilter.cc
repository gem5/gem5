/*
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

#include "mem/ruby/filters/MultiBitSelBloomFilter.hh"

#include <limits>

#include "base/bitfield.hh"
#include "params/MultiBitSelBloomFilter.hh"

MultiBitSelBloomFilter::MultiBitSelBloomFilter(
    const MultiBitSelBloomFilterParams* p)
    : AbstractBloomFilter(p), numHashes(p->num_hashes),
      skipBits(p->skip_bits),
      parFilterSize(p->size / numHashes),
      isParallel(p->is_parallel)
{
}

MultiBitSelBloomFilter::~MultiBitSelBloomFilter()
{
}

void
MultiBitSelBloomFilter::merge(const AbstractBloomFilter *other)
{
    auto cast_other = static_cast<const MultiBitSelBloomFilter*>(other);
    assert(filter.size() == cast_other->filter.size());
    for (int i = 0; i < filter.size(); ++i){
        filter[i] |= cast_other->filter[i];
    }
}

void
MultiBitSelBloomFilter::set(Addr addr)
{
    for (int i = 0; i < numHashes; i++) {
        int idx = hash(addr, i);
        filter[idx] = 1;
    }
}

int
MultiBitSelBloomFilter::getCount(Addr addr) const
{
    int count = 0;
    for (int i=0; i < numHashes; i++) {
        count += filter[hash(addr, i)];
    }
    return count;
}

int
MultiBitSelBloomFilter::hash(Addr addr, int hash_number) const
{
    uint64_t x = bits(addr, std::numeric_limits<Addr>::digits - 1,
        offsetBits) >> skipBits;
    int y = hashBitsel(x, hash_number, numHashes, 30, sizeBits);
    //36-bit addresses, 6-bit cache lines

    if (isParallel) {
        return (y % parFilterSize) + hash_number * parFilterSize;
    } else {
        return y % filter.size();
    }
}

int
MultiBitSelBloomFilter::hashBitsel(uint64_t value, int index, int jump,
                                    int maxBits, int numBits) const
{
    uint64_t mask = 1;
    int result = 0;
    int bit, i;

    for (i = 0; i < numBits; i++) {
        bit = (index + jump*i) % maxBits;
        if (value & (mask << bit)) result += mask << i;
    }
    return result;
}

MultiBitSelBloomFilter*
MultiBitSelBloomFilterParams::create()
{
    return new MultiBitSelBloomFilter(this);
}
