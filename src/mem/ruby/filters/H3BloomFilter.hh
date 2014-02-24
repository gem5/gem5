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

#ifndef __MEM_RUBY_FILTERS_H3BLOOMFILTER_HH__
#define __MEM_RUBY_FILTERS_H3BLOOMFILTER_HH__

#include <iostream>
#include <string>
#include <vector>

#include "mem/ruby/common/Address.hh"
#include "mem/ruby/filters/AbstractBloomFilter.hh"

class H3BloomFilter : public AbstractBloomFilter
{
  public:
    H3BloomFilter(std::string config);
    ~H3BloomFilter();

    void clear();
    void increment(const Address& addr);
    void decrement(const Address& addr);
    void merge(AbstractBloomFilter * other_filter);
    void set(const Address& addr);
    void unset(const Address& addr);

    bool isSet(const Address& addr);
    int getCount(const Address& addr);
    int getTotalCount();
    void print(std::ostream& out) const;

    int getIndex(const Address& addr);
    int readBit(const int index);
    void writeBit(const int index, const int value);

    int
    operator[](const int index) const
    {
        return this->m_filter[index];
    }

  private:
    int get_index(const Address& addr, int hashNumber);

    int hash_H3(uint64 value, int index);

    std::vector<int> m_filter;
    int m_filter_size;
    int m_num_hashes;
    int m_filter_size_bits;

    int m_par_filter_size;
    int m_par_filter_size_bits;

    int primes_list[6];// = {9323,11279,10247,30637,25717,43711};
    int mults_list[6]; //= {255,29,51,3,77,43};
    int adds_list[6]; //= {841,627,1555,241,7777,65391};

    bool isParallel;
};

#endif // __MEM_RUBY_FILTERS_H3BLOOMFILTER_HH__
