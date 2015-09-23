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

#ifndef __MEM_RUBY_FILTERS_BULKBLOOMFILTER_HH__
#define __MEM_RUBY_FILTERS_BULKBLOOMFILTER_HH__

#include <iostream>
#include <vector>

#include "mem/ruby/common/Address.hh"
#include "mem/ruby/filters/AbstractBloomFilter.hh"

class BulkBloomFilter : public AbstractBloomFilter
{
  public:
    BulkBloomFilter(int size);
    ~BulkBloomFilter();

    void clear();
    void increment(Addr addr);
    void decrement(Addr addr);
    void merge(AbstractBloomFilter * other_filter);
    void set(Addr addr);
    void unset(Addr addr);

    bool isSet(Addr addr);
    int getCount(Addr addr);
    int getTotalCount();
    int getIndex(Addr addr);
    int readBit(const int index);
    void writeBit(const int index, const int value);

    void print(std::ostream& out) const;

  private:
    int get_index(Addr addr);
    Addr permute(Addr addr);

    std::vector<int> m_filter;
    std::vector<int> m_temp_filter;

    int m_filter_size;
    int m_filter_size_bits;

    int m_sector_bits;

};

#endif // __MEM_RUBY_FILTERS_BULKBLOOMFILTER_HH__
