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

#ifndef __MEM_RUBY_FILTERS_GENERICBLOOMFILTER_HH__
#define __MEM_RUBY_FILTERS_GENERICBLOOMFILTER_HH__

#include <iostream>
#include <string>

#include "mem/ruby/common/Address.hh"
#include "mem/ruby/filters/AbstractBloomFilter.hh"

class GenericBloomFilter
{
  public:
    GenericBloomFilter(std::string config);
    ~GenericBloomFilter();

    void clear();
    void increment(const Address& addr);
    void decrement(const Address& addr);
    void merge(GenericBloomFilter * other_filter);
    void set(const Address& addr);
    void unset(const Address& addr);
    AbstractBloomFilter *
    getFilter()
    {
        return m_filter;
    }

    bool isSet(const Address& addr);

    int getCount(const Address& addr);

    int getTotalCount();

    int getIndex(const Address& addr);
    int readBit(const int index);
    void writeBit(const int index, const int value);

    void print(std::ostream& out) const;

  private:
    AbstractBloomFilter* m_filter;
};

inline std::ostream&
operator<<(std::ostream& out, const GenericBloomFilter& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_FILTERS_GENERICBLOOMFILTER_HH__
