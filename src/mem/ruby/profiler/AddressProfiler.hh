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

#ifndef __MEM_RUBY_PROFILER_ADDRESSPROFILER_HH__
#define __MEM_RUBY_PROFILER_ADDRESSPROFILER_HH__

#include <iostream>
#include <unordered_map>

#include "mem/ruby/common/Address.hh"
#include "mem/ruby/common/Histogram.hh"
#include "mem/ruby/profiler/AccessTraceForAddress.hh"
#include "mem/ruby/profiler/Profiler.hh"
#include "mem/ruby/protocol/AccessType.hh"
#include "mem/ruby/protocol/RubyRequest.hh"

namespace gem5
{

namespace ruby
{

class Set;

class AddressProfiler
{
  public:
    typedef std::unordered_map<Addr, AccessTraceForAddress> AddressMap;

  public:
    AddressProfiler(int num_of_sequencers, Profiler *profiler);
    ~AddressProfiler();

    void printStats(std::ostream &out) const;
    void clearStats();

    void addTraceSample(Addr data_addr, Addr pc_addr, RubyRequestType type,
                        RubyAccessMode access_mode, NodeID id,
                        bool sharing_miss);
    void profileRetry(Addr data_addr, AccessType type, int count);
    void profileGetX(Addr datablock, Addr PC, const Set &owner,
                     const Set &sharers, NodeID requestor);
    void profileGetS(Addr datablock, Addr PC, const Set &owner,
                     const Set &sharers, NodeID requestor);

    void print(std::ostream &out) const;

    // added by SS
    void setHotLines(bool hot_lines);
    void setAllInstructions(bool all_instructions);

    void
    regStats(const std::string &name)
    {}

    void
    collateStats()
    {}

  private:
    // Private copy constructor and assignment operator
    AddressProfiler(const AddressProfiler &obj);
    AddressProfiler &operator=(const AddressProfiler &obj);

    int64_t m_sharing_miss_counter;

    AddressMap m_dataAccessTrace;
    AddressMap m_macroBlockAccessTrace;
    AddressMap m_programCounterAccessTrace;
    AddressMap m_retryProfileMap;
    Histogram m_retryProfileHisto;
    Histogram m_retryProfileHistoWrite;
    Histogram m_retryProfileHistoRead;
    Histogram m_getx_sharing_histogram;
    Histogram m_gets_sharing_histogram;

    Profiler *m_profiler;

    // added by SS
    bool m_hot_lines;
    bool m_all_instructions;

    int m_num_of_sequencers;
};

AccessTraceForAddress &
lookupTraceForAddress(Addr addr, AddressProfiler::AddressMap &record_map);

void printSorted(std::ostream &out, int num_of_sequencers,
                 const AddressProfiler::AddressMap &record_map,
                 std::string description, Profiler *profiler);

inline std::ostream &
operator<<(std::ostream &out, const AddressProfiler &obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_PROFILER_ADDRESSPROFILER_HH__
