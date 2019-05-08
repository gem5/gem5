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

#include "mem/ruby/filters/MultiGrainBloomFilter.hh"

#include "base/bitfield.hh"
#include "params/MultiGrainBloomFilter.hh"

MultiGrainBloomFilter::MultiGrainBloomFilter(
    const MultiGrainBloomFilterParams* p)
    : AbstractBloomFilter(p), pageFilter(p->page_filter_size),
      pageFilterSizeBits(floorLog2(p->page_filter_size))
{
}

MultiGrainBloomFilter::~MultiGrainBloomFilter()
{
}

void
MultiGrainBloomFilter::clear()
{
    AbstractBloomFilter::clear();
    for (auto& entry : pageFilter){
        entry = 0;
    }
}

void
MultiGrainBloomFilter::set(Addr addr)
{
    const int index = hash(addr);
    assert(index < filter.size());
    filter[index] = 1;

    const int page_index = pageHash(addr);
    assert(page_index < pageFilter.size());
    pageFilter[page_index] = 1;
}

int
MultiGrainBloomFilter::getCount(Addr addr) const
{
    const int index = hash(addr);
    const int page_index = pageHash(addr);
    assert(index < filter.size());
    assert(page_index < pageFilter.size());
    return filter[index] + pageFilter[page_index];
}

int
MultiGrainBloomFilter::getTotalCount() const
{
    int count = AbstractBloomFilter::getTotalCount();

    for (const auto& entry : pageFilter) {
        count += entry;
    }

    return count;
}

int
MultiGrainBloomFilter::hash(Addr addr) const
{
    // grap a chunk of bits after byte offset
    return bits(addr, offsetBits + sizeBits - 1, offsetBits);
}

int
MultiGrainBloomFilter::pageHash(Addr addr) const
{
    int num_bits = offsetBits + sizeBits - 1;

    // grap a chunk of bits after first chunk
    return bits(addr, num_bits + pageFilterSizeBits - 1, num_bits);
}

MultiGrainBloomFilter*
MultiGrainBloomFilterParams::create()
{
    return new MultiGrainBloomFilter(this);
}
