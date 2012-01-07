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

#ifndef __MEM_RUBY_PROFILER_CACHEPROFILER_HH__
#define __MEM_RUBY_PROFILER_CACHEPROFILER_HH__

#include <iostream>
#include <string>
#include <vector>

#include "mem/protocol/GenericRequestType.hh"
#include "mem/protocol/PrefetchBit.hh"
#include "mem/protocol/RubyAccessMode.hh"
#include "mem/protocol/RubyRequestType.hh"
#include "mem/ruby/common/Global.hh"
#include "mem/ruby/common/Histogram.hh"

class CacheProfiler
{
  public:
    CacheProfiler(const std::string& description);
    ~CacheProfiler();

    void printStats(std::ostream& out) const;
    void clearStats();

    void addCacheStatSample(RubyRequestType requestType,
                            RubyAccessMode type,
                            PrefetchBit pfBit);

    void addGenericStatSample(GenericRequestType requestType, 
                              RubyAccessMode type,
                              PrefetchBit pfBit);

    void print(std::ostream& out) const;

  private:
    // Private copy constructor and assignment operator
    CacheProfiler(const CacheProfiler& obj);
    CacheProfiler& operator=(const CacheProfiler& obj);
    void addStatSample(RubyAccessMode type, PrefetchBit pfBit);

    std::string m_description;
    int64 m_misses;
    int64 m_demand_misses;
    int64 m_prefetches;
    int64 m_sw_prefetches;
    int64 m_hw_prefetches;
    int64 m_accessModeTypeHistogram[RubyAccessMode_NUM];

    std::vector<int> m_cacheRequestType;
    std::vector<int> m_genericRequestType;
};

inline std::ostream&
operator<<(std::ostream& out, const CacheProfiler& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_PROFILER_CACHEPROFILER_HH__
