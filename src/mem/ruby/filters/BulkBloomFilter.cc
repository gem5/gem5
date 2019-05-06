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

#include "mem/ruby/filters/BulkBloomFilter.hh"

#include "mem/ruby/common/Address.hh"
#include "mem/ruby/system/RubySystem.hh"
#include "params/BulkBloomFilter.hh"

BulkBloomFilter::BulkBloomFilter(const BulkBloomFilterParams* p)
    : AbstractBloomFilter(p), sectorBits(sizeBits - 1)
{
}

BulkBloomFilter::~BulkBloomFilter()
{
}

void
BulkBloomFilter::set(Addr addr)
{
    // c0 contains the cache index bits
    int set_bits = sectorBits;
    int block_bits = RubySystem::getBlockSizeBits();
    int c0 = bitSelect(addr, block_bits, block_bits + set_bits - 1);
    // c1 contains the lower sectorBits permuted bits
    //Address permuted_bits = permute(addr);
    //int c1 = permuted_bits.bitSelect(0, set_bits-1);
    int c1 = bitSelect(addr, block_bits+set_bits, (block_bits+2*set_bits) - 1);
    //assert(c0 < (filter_size/2));
    //assert(c0 + (filter_size/2) < filter_size);
    //assert(c1 < (filter_size/2));
    // set v0 bit
    filter[c0 + (filter.size()/2)] = 1;
    // set v1 bit
    filter[c1] = 1;
}

bool
BulkBloomFilter::isSet(Addr addr) const
{
    // c0 contains the cache index bits
    const int filter_size = filter.size();
    int set_bits = sectorBits;
    int block_bits = RubySystem::getBlockSizeBits();
    int c0 = bitSelect(addr, block_bits, block_bits + set_bits - 1);
    // c1 contains the lower 10 permuted bits
    //Address permuted_bits = permute(addr);
    //int c1 = permuted_bits.bitSelect(0, set_bits-1);
    int c1 = bitSelect(addr, block_bits+set_bits, (block_bits+2*set_bits) - 1);
    //assert(c0 < (filter_size/2));
    //assert(c0 + (filter_size/2) < filter_size);
    //assert(c1 < (filter_size/2));
    // set v0 bit
    std::vector<int> temp_filter(filter.size(), 0);
    temp_filter[c0 + (filter_size/2)] = 1;
    // set v1 bit
    temp_filter[c1] = 1;

    // perform filter intersection. If any c part is 0, no possibility
    // of address being in signature.  get first c intersection part
    bool zero = false;
    for (int i = 0; i < filter_size/2; ++i){
        // get intersection of signatures
        temp_filter[i] = temp_filter[i] && filter[i];
        zero = zero || temp_filter[i];
    }
    zero = !zero;
    if (zero) {
        // one section is zero, no possiblility of address in signature
        // reset bits we just set
        temp_filter[c0 + (filter_size / 2)] = 0;
        temp_filter[c1] = 0;
        return false;
    }

    // check second section
    zero = false;
    for (int i = filter_size / 2; i < filter_size; ++i) {
        // get intersection of signatures
        temp_filter[i] =  temp_filter[i] && filter[i];
        zero = zero || temp_filter[i];
    }
    zero = !zero;
    if (zero) {
        // one section is zero, no possiblility of address in signature
        temp_filter[c0 + (filter_size / 2)] = 0;
        temp_filter[c1] = 0;
        return false;
    }
    // one section has at least one bit set
    temp_filter[c0 + (filter_size / 2)] = 0;
    temp_filter[c1] = 0;
    return true;
}

int
BulkBloomFilter::getCount(Addr addr) const
{
    // TODO as in the multi-hashed filters
    return 0;
}

Addr
BulkBloomFilter::hash(Addr addr) const
{
    // permutes the original address bits according to Table 5
    int block_offset = RubySystem::getBlockSizeBits();
    Addr part1 = bitSelect(addr, block_offset, block_offset + 6),
        part2 = bitSelect(addr, block_offset + 9, block_offset + 9),
        part3 = bitSelect(addr, block_offset + 11, block_offset + 11),
        part4 = bitSelect(addr, block_offset + 17, block_offset + 17),
        part5 = bitSelect(addr, block_offset + 7, block_offset + 8),
        part6 = bitSelect(addr, block_offset + 10, block_offset + 10),
        part7 = bitSelect(addr, block_offset + 12, block_offset + 12),
        part8 = bitSelect(addr, block_offset + 13, block_offset + 13),
        part9 = bitSelect(addr, block_offset + 15, block_offset + 16),
        part10 = bitSelect(addr, block_offset + 18, block_offset + 20),
        part11 = bitSelect(addr, block_offset + 14, block_offset + 14);

    Addr result =
        (part1 << 14) | (part2 << 13) | (part3 << 12) | (part4 << 11) |
        (part5 << 9)  | (part6 << 8)  | (part7 << 7)  | (part8 << 6)  |
        (part9 << 4)  | (part10 << 1) | (part11);

    // assume 32 bit addresses (both virtual and physical)
    // select the remaining high-order 11 bits
    Addr remaining_bits =
        bitSelect(addr, block_offset + 21, 31) << 21;
    result = result | remaining_bits;

    return result;
}

BulkBloomFilter*
BulkBloomFilterParams::create()
{
    return new BulkBloomFilter(this);
}
