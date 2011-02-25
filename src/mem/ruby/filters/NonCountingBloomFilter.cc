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

#include "base/intmath.hh"
#include "base/str.hh"
#include "mem/ruby/filters/NonCountingBloomFilter.hh"
#include "mem/ruby/system/System.hh"

using namespace std;

NonCountingBloomFilter::NonCountingBloomFilter(string str)
{
    string head, tail;
#ifndef NDEBUG
    bool success =
#endif
        split_first(str, head, tail, '_');
    assert(success);

    // head contains filter size, tail contains bit offset from block number
    m_filter_size = atoi(head.c_str());
    m_offset = atoi(tail.c_str());
    m_filter_size_bits = floorLog2(m_filter_size);

    m_filter.resize(m_filter_size);
    clear();
}

NonCountingBloomFilter::~NonCountingBloomFilter()
{
}

void
NonCountingBloomFilter::clear()
{
    for (int i = 0; i < m_filter_size; i++) {
        m_filter[i] = 0;
    }
}

void
NonCountingBloomFilter::increment(const Address& addr)
{
    // Not used
}

void
NonCountingBloomFilter::decrement(const Address& addr)
{
    // Not used
}

void
NonCountingBloomFilter::merge(AbstractBloomFilter *other_filter)
{
    // assumes both filters are the same size!
    NonCountingBloomFilter * temp = (NonCountingBloomFilter*) other_filter;
    for(int i = 0; i < m_filter_size; ++i){
        m_filter[i] |= (*temp)[i];
    }
}

void
NonCountingBloomFilter::set(const Address& addr)
{
    int i = get_index(addr);
    m_filter[i] = 1;
}

void
NonCountingBloomFilter::unset(const Address& addr)
{
    int i = get_index(addr);
    m_filter[i] = 0;
}

bool
NonCountingBloomFilter::isSet(const Address& addr)
{
    int i = get_index(addr);
    return (m_filter[i]);
}


int
NonCountingBloomFilter::getCount(const Address& addr)
{
    return m_filter[get_index(addr)];
}

int
NonCountingBloomFilter::getTotalCount()
{
    int count = 0;

    for (int i = 0; i < m_filter_size; i++) {
        count += m_filter[i];
    }
    return count;
}

void
NonCountingBloomFilter::print(ostream& out) const
{
}

int
NonCountingBloomFilter::getIndex(const Address& addr)
{
    return get_index(addr);
}

int
NonCountingBloomFilter::readBit(const int index)
{
    return m_filter[index];
}

void
NonCountingBloomFilter::writeBit(const int index, const int value)
{
    m_filter[index] = value;
}

int
NonCountingBloomFilter::get_index(const Address& addr)
{
    return addr.bitSelect(RubySystem::getBlockSizeBits() + m_offset,
                          RubySystem::getBlockSizeBits() + m_offset +
                          m_filter_size_bits - 1);
}


