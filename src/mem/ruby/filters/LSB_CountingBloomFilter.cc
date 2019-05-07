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
#include "params/LSB_CountingBloomFilter.hh"

LSB_CountingBloomFilter::LSB_CountingBloomFilter(
    const LSB_CountingBloomFilterParams* p)
    : AbstractBloomFilter(p), maxValue(p->max_value)
{
}

LSB_CountingBloomFilter::~LSB_CountingBloomFilter()
{
}

void
LSB_CountingBloomFilter::set(Addr addr)
{
    const int i = hash(addr);
    if (filter[i] < maxValue)
        filter[i] += 1;
}

void
LSB_CountingBloomFilter::unset(Addr addr)
{
    const int i = hash(addr);
    if (filter[i] > 0)
        filter[i] -= 1;
}

int
LSB_CountingBloomFilter::getCount(Addr addr) const
{
    return filter[hash(addr)];
}

int
LSB_CountingBloomFilter::hash(Addr addr) const
{
    return bits(addr, offsetBits + sizeBits - 1, offsetBits);
}

LSB_CountingBloomFilter*
LSB_CountingBloomFilterParams::create()
{
    return new LSB_CountingBloomFilter(this);
}
