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

#include <vector>

#include "base/intmath.hh"
#include "base/str.hh"
#include "mem/ruby/filters/MultiBitSelBloomFilter.hh"

using namespace std;

MultiBitSelBloomFilter::MultiBitSelBloomFilter(string str)
{
    vector<string> items;
    tokenize(items, str, '_');
    assert(items.size() == 4);

    // head contains filter size, tail contains bit offset from block number
    m_filter_size = atoi(items[0].c_str());
    m_num_hashes = atoi(items[1].c_str());
    m_skip_bits = atoi(items[2].c_str());

    if (items[3] == "Regular") {
        isParallel = false;
    } else if (items[3] == "Parallel") {
        isParallel = true;
    } else {
        panic("ERROR: Incorrect config string for MultiBitSel Bloom! :%s",
              str);
    }

    m_filter_size_bits = floorLog2(m_filter_size);

    m_par_filter_size = m_filter_size / m_num_hashes;
    m_par_filter_size_bits = floorLog2(m_par_filter_size);

    m_filter.resize(m_filter_size);
    clear();
}

MultiBitSelBloomFilter::~MultiBitSelBloomFilter()
{
}

void
MultiBitSelBloomFilter::clear()
{
    for (int i = 0; i < m_filter_size; i++) {
        m_filter[i] = 0;
    }
}

void
MultiBitSelBloomFilter::increment(Addr addr)
{
    // Not used
}


void
MultiBitSelBloomFilter::decrement(Addr addr)
{
    // Not used
}

void
MultiBitSelBloomFilter::merge(AbstractBloomFilter *other_filter)
{
    // assumes both filters are the same size!
    MultiBitSelBloomFilter * temp = (MultiBitSelBloomFilter*) other_filter;
    for (int i = 0; i < m_filter_size; ++i){
        m_filter[i] |= (*temp)[i];
    }
}

void
MultiBitSelBloomFilter::set(Addr addr)
{
    for (int i = 0; i < m_num_hashes; i++) {
        int idx = get_index(addr, i);
        m_filter[idx] = 1;
    }
}

void
MultiBitSelBloomFilter::unset(Addr addr)
{
    cout << "ERROR: Unset should never be called in a Bloom filter";
    assert(0);
}

bool
MultiBitSelBloomFilter::isSet(Addr addr)
{
    bool res = true;

    for (int i=0; i < m_num_hashes; i++) {
        int idx = get_index(addr, i);
        res = res && m_filter[idx];
    }
    return res;
}

int
MultiBitSelBloomFilter::getCount(Addr addr)
{
    return isSet(addr)? 1: 0;
}

int
MultiBitSelBloomFilter::getIndex(Addr addr)
{
    return 0;
}

int
MultiBitSelBloomFilter::readBit(const int index)
{
    return 0;
}

void
MultiBitSelBloomFilter::writeBit(const int index, const int value)
{
}

int
MultiBitSelBloomFilter::getTotalCount()
{
    int count = 0;

    for (int i = 0; i < m_filter_size; i++) {
        count += m_filter[i];
    }
    return count;
}

void
MultiBitSelBloomFilter::print(ostream& out) const
{
}

int
MultiBitSelBloomFilter::get_index(Addr addr, int i)
{
    // m_skip_bits is used to perform BitSelect after skipping some
    // bits. Used to simulate BitSel hashing on larger than cache-line
    // granularities
    uint64_t x = (makeLineAddress(addr) >> m_skip_bits);
    int y = hash_bitsel(x, i, m_num_hashes, 30, m_filter_size_bits);
    //36-bit addresses, 6-bit cache lines

    if (isParallel) {
        return (y % m_par_filter_size) + i*m_par_filter_size;
    } else {
        return y % m_filter_size;
    }
}

int
MultiBitSelBloomFilter::hash_bitsel(uint64_t value, int index, int jump,
                                    int maxBits, int numBits)
{
    uint64_t mask = 1;
    int result = 0;
    int bit, i;

    for (i = 0; i < numBits; i++) {
        bit = (index + jump*i) % maxBits;
        if (value & (mask << bit)) result += mask << i;
    }
    return result;
}
