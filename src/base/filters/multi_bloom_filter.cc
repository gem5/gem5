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

#include "base/filters/multi_bloom_filter.hh"

#include "base/logging.hh"
#include "params/BloomFilterMulti.hh"

namespace BloomFilter {

Multi::Multi(const BloomFilterMultiParams* p)
    : Base(p), filters(p->filters)
{
}

Multi::~Multi()
{
}

void
Multi::clear()
{
    for (auto& sub_filter : filters) {
        sub_filter->clear();
    }
}

void
Multi::merge(const Base* other)
{
    auto* cast_other = static_cast<const Multi*>(other);
    assert(filters.size() == cast_other->filters.size());
    for (int i = 0; i < filters.size(); ++i){
        filters[i]->merge(cast_other->filters[i]);
    }
}

void
Multi::set(Addr addr)
{
    for (auto& sub_filter : filters) {
        sub_filter->set(addr);
    }
}

void
Multi::unset(Addr addr)
{
    for (auto& sub_filter : filters) {
        sub_filter->unset(addr);
    }
}

bool
Multi::isSet(Addr addr) const
{
    int count = 0;
    for (const auto& sub_filter : filters) {
        if (sub_filter->isSet(addr)) {
            count++;
        }
    }
    return count >= setThreshold;
}

int
Multi::getCount(Addr addr) const
{
    int count = 0;
    for (const auto& sub_filter : filters) {
        count += sub_filter->getCount(addr);
    }
    return count;
}

int
Multi::getTotalCount() const
{
    int count = 0;
    for (const auto& sub_filter : filters) {
        count += sub_filter->getTotalCount();
    }
    return count;
}

} // namespace BloomFilter

BloomFilter::Multi*
BloomFilterMultiParams::create()
{
    return new BloomFilter::Multi(this);
}

