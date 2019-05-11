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

#include "mem/ruby/filters/LSB_CountingBloomFilter.hh"

#include "base/bitfield.hh"
#include "params/BloomFilterLSBCounting.hh"

namespace BloomFilter {

LSBCounting::LSBCounting(
    const BloomFilterLSBCountingParams* p)
    : Base(p), maxValue(p->max_value)
{
}

LSBCounting::~LSBCounting()
{
}

void
LSBCounting::merge(const Base* other)
{
    auto* cast_other = static_cast<const LSBCounting*>(other);
    assert(filter.size() == cast_other->filter.size());
    for (int i = 0; i < filter.size(); ++i){
        if (filter[i] < maxValue - cast_other->filter[i]) {
            filter[i] += cast_other->filter[i];
        } else {
            filter[i] = maxValue;
        }
    }
}

void
LSBCounting::set(Addr addr)
{
    const int i = hash(addr);
    if (filter[i] < maxValue)
        filter[i] += 1;
}

void
LSBCounting::unset(Addr addr)
{
    const int i = hash(addr);
    if (filter[i] > 0)
        filter[i] -= 1;
}

int
LSBCounting::getCount(Addr addr) const
{
    return filter[hash(addr)];
}

int
LSBCounting::hash(Addr addr) const
{
    return bits(addr, offsetBits + sizeBits - 1, offsetBits);
}

} // namespace BloomFilter

BloomFilter::LSBCounting*
BloomFilterLSBCountingParams::create()
{
    return new BloomFilter::LSBCounting(this);
}

