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

#include "base/filters/block_bloom_filter.hh"

#include "base/bitfield.hh"
#include "base/logging.hh"
#include "params/BloomFilterBlock.hh"

namespace gem5
{

namespace bloom_filter
{

Block::Block(const BloomFilterBlockParams &p)
    : Base(p), masksLSBs(p.masks_lsbs),
      masksSizes(p.masks_sizes)
{
    fatal_if(masksLSBs.size() != masksSizes.size(),
        "Masks haven't been properly provided");
    fatal_if(masksLSBs.size() < 1,
        "There must be at least one mask to extract an address bitfield");

    for (int i = 0; i < masksLSBs.size(); i++) {
        fatal_if((masksSizes[i] > sizeBits) || (masksSizes[i] <= 0),
            "The bitfields must be indexable in the filter");
        fatal_if(masksLSBs[i] + masksSizes[i] >
            std::numeric_limits<Addr>::digits,
            "The total size of the bitfields cannot be bigger than the " \
            "number of bits in an address");
    }
}

Block::~Block()
{
}

void
Block::set(Addr addr)
{
    filter[hash(addr)]++;
}

void
Block::unset(Addr addr)
{
    filter[hash(addr)]--;
}

int
Block::getCount(Addr addr) const
{
    return filter[hash(addr)];
}

int
Block::hash(Addr addr) const
{
    Addr hashed_addr = 0;
    for (int i = 0; i < masksLSBs.size(); i++) {
        hashed_addr ^=
            bits(addr, offsetBits + masksLSBs[i] + masksSizes[i] - 1,
            offsetBits + masksLSBs[i]);
    }
    assert(hashed_addr < filter.size());
    return hashed_addr;
}

} // namespace bloom_filter
} // namespace gem5
