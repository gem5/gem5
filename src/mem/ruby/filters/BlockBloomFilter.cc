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

#include "mem/ruby/filters/BlockBloomFilter.hh"

#include "base/bitfield.hh"
#include "params/BlockBloomFilter.hh"

BlockBloomFilter::BlockBloomFilter(const BlockBloomFilterParams* p)
    : AbstractBloomFilter(p)
{
}

BlockBloomFilter::~BlockBloomFilter()
{
}

void
BlockBloomFilter::set(Addr addr)
{
    filter[hash(addr)] = 1;
}

void
BlockBloomFilter::unset(Addr addr)
{
    filter[hash(addr)] = 0;
}

int
BlockBloomFilter::getCount(Addr addr) const
{
    return filter[hash(addr)];
}

int
BlockBloomFilter::hash(Addr addr) const
{
    // Pull out some bit field ==> B1
    // Pull out additional bits, not the same as B1 ==> B2
    //  XOR B1 and B2 to get hash index
    Addr block_bits = bits(addr, 2 * offsetBits - 1, offsetBits);
    int offset = 5;
    Addr other_bits = bits(addr, 2 * offsetBits + offset + sizeBits - 1,
        2 * offsetBits + offset);
    int index = block_bits ^ other_bits;
    assert(index < filter.size());
    return index;
}

BlockBloomFilter*
BlockBloomFilterParams::create()
{
    return new BlockBloomFilter(this);
}
