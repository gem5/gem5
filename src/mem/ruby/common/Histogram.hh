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

#ifndef __MEM_RUBY_COMMON_HISTOGRAM_HH__
#define __MEM_RUBY_COMMON_HISTOGRAM_HH__

#include <iostream>
#include <vector>

#include "mem/ruby/common/TypeDefines.hh"

class Histogram
{
  public:
    Histogram(int binsize = 1, uint32_t bins = 50);
    ~Histogram();

    void add(int64_t value);
    void add(Histogram& hist);
    void doubleBinSize();

    void clear() { clear(m_data.size()); }
    void clear(uint32_t bins);
    void clear(int binsize, uint32_t bins);

    uint64_t size() const { return m_count; }
    uint32_t getBins() const { return m_data.size(); }
    int getBinSize() const { return m_binsize; }
    int64_t getTotal() const { return m_sumSamples; }
    uint64_t getSquaredTotal() const { return m_sumSquaredSamples; }
    uint64_t getData(int index) const { return m_data[index]; }
    int64_t getMax() const { return m_max; }

    void printWithMultiplier(std::ostream& out, double multiplier) const;
    void printPercent(std::ostream& out) const;
    void print(std::ostream& out) const;

private:
    std::vector<uint64_t> m_data;
    int64_t m_max;          // the maximum value seen so far
    uint64_t m_count;                // the number of elements added
    int m_binsize;                // the size of each bucket
    uint32_t m_largest_bin;      // the largest bin used

    int64_t m_sumSamples;   // the sum of all samples
    uint64_t m_sumSquaredSamples; // the sum of the square of all samples

    double getStandardDeviation() const;
};

bool node_less_then_eq(const Histogram* n1, const Histogram* n2);

inline std::ostream&
operator<<(std::ostream& out, const Histogram& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_COMMON_HISTOGRAM_HH__
