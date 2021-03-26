/*
 * Copyright (c) 2019 Inria
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

#include "base/filters/perfect_bloom_filter.hh"

#include "base/logging.hh"
#include "params/BloomFilterPerfect.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(BloomFilter, bloom_filter);
namespace bloom_filter
{

Perfect::Perfect(const BloomFilterPerfectParams &p)
    : Base(p)
{
    fatal_if(p.size != 1, "The perfect Bloom filter cannot be limited to a "
        "specific size.");
    fatal_if(p.num_bits != 1, "The perfect Bloom filter tracks entries "
        "perfectly using an unlimited amount of 1-bit entries.");
    fatal_if(p.threshold != 1, "The perfect Bloom filter uses 1-bit entries; "
        "thus, their thresholds must be 1.");
}

Perfect::~Perfect()
{
}

void
Perfect::clear()
{
    entries.clear();
}

void
Perfect::merge(const Base* other)
{
    auto* cast_other = static_cast<const Perfect*>(other);
    entries.insert(cast_other->entries.begin(), cast_other->entries.end());
}

void
Perfect::set(Addr addr)
{
    entries.insert(addr);
}

void
Perfect::unset(Addr addr)
{
    entries.erase(addr);
}

int
Perfect::getCount(Addr addr) const
{
    return entries.find(addr) != entries.end();
}

int
Perfect::getTotalCount() const
{
    return entries.size();
}

} // namespace bloom_filter
} // namespace gem5
