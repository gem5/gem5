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
#include "mem/ruby/filters/MultiGrainBloomFilter.hh"
#include "mem/ruby/system/RubySystem.hh"

using namespace std;

MultiGrainBloomFilter::MultiGrainBloomFilter(int head, int tail)
{
    // head contains size of 1st bloom filter, tail contains size of
    // 2nd bloom filter
    m_filter_size = head;
    m_filter_size_bits = floorLog2(m_filter_size);

    m_page_filter_size = tail;
    m_page_filter_size_bits = floorLog2(m_page_filter_size);

    m_filter.resize(m_filter_size);
    m_page_filter.resize(m_page_filter_size);
    clear();
}

MultiGrainBloomFilter::~MultiGrainBloomFilter()
{
}

void
MultiGrainBloomFilter::clear()
{
    for (int i = 0; i < m_filter_size; i++) {
        m_filter[i] = 0;
    }
    for(int i=0; i < m_page_filter_size; ++i){
        m_page_filter[i] = 0;
    }
}

void
MultiGrainBloomFilter::increment(Addr addr)
{
    // Not used
}


void
MultiGrainBloomFilter::decrement(Addr addr)
{
    // Not used
}

void
MultiGrainBloomFilter::merge(AbstractBloomFilter *other_filter)
{
    // TODO
}

void
MultiGrainBloomFilter::set(Addr addr)
{
    int i = get_block_index(addr);
    assert(i < m_filter_size);
    assert(get_page_index(addr) < m_page_filter_size);
    m_filter[i] = 1;
    m_page_filter[i] = 1;

}

void
MultiGrainBloomFilter::unset(Addr addr)
{
    // not used
}

bool
MultiGrainBloomFilter::isSet(Addr addr)
{
    int i = get_block_index(addr);
    assert(i < m_filter_size);
    assert(get_page_index(addr) < m_page_filter_size);
    // we have to have both indices set
    return (m_filter[i] && m_page_filter[i]);
}

int
MultiGrainBloomFilter::getCount(Addr addr)
{
    // not used
    return 0;
}

int
MultiGrainBloomFilter::getTotalCount()
{
    int count = 0;

    for (int i = 0; i < m_filter_size; i++) {
        count += m_filter[i];
    }

    for(int i=0; i < m_page_filter_size; ++i) {
        count += m_page_filter[i] = 0;
    }

    return count;
}

int
MultiGrainBloomFilter::getIndex(Addr addr)
{
    return 0;
    // TODO
}

int
MultiGrainBloomFilter::readBit(const int index)
{
    return 0;
    // TODO
}

void
MultiGrainBloomFilter::writeBit(const int index, const int value)
{
    // TODO
}

void
MultiGrainBloomFilter::print(ostream& out) const
{
}

int
MultiGrainBloomFilter::get_block_index(Addr addr)
{
    // grap a chunk of bits after byte offset
    return bitSelect(addr, RubySystem::getBlockSizeBits(),
                     RubySystem::getBlockSizeBits() +
                     m_filter_size_bits - 1);
}

int
MultiGrainBloomFilter::get_page_index(Addr addr)
{
    int bits = RubySystem::getBlockSizeBits() + m_filter_size_bits - 1;

    // grap a chunk of bits after first chunk
    return bitSelect(addr, bits, bits + m_page_filter_size_bits - 1);
}




