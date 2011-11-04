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

#ifndef __MEM_RUBY_PROFILER_MEMCNTRLPROFILER_HH__
#define __MEM_RUBY_PROFILER_MEMCNTRLPROFILER_HH__

#include <iostream>
#include <string>
#include <vector>

#include "mem/ruby/common/TypeDefines.hh"

class MemCntrlProfiler
{
  public:
    MemCntrlProfiler(const std::string& description, int banks_per_rank,
                     int ranks_per_dimm, int dimms_per_channel);
    ~MemCntrlProfiler();

    void printStats(std::ostream& out) const;
    void clearStats();

    void profileMemReq(int bank);
    void profileMemBankBusy();
    void profileMemBusBusy();
    void profileMemTfawBusy();
    void profileMemReadWriteBusy();
    void profileMemDataBusBusy();
    void profileMemRefresh();
    void profileMemRead();
    void profileMemWrite();
    void profileMemWaitCycles(int cycles);
    void profileMemInputQ(int cycles);
    void profileMemBankQ(int cycles);
    void profileMemArbWait(int cycles);
    void profileMemRandBusy();
    void profileMemNotOld();

    void print(std::ostream& out) const;

private:
    // Private copy constructor and assignment operator
    MemCntrlProfiler(const MemCntrlProfiler& obj);
    MemCntrlProfiler& operator=(const MemCntrlProfiler& obj);

    std::string m_description;
    uint64 m_memReq;
    uint64 m_memBankBusy;
    uint64 m_memBusBusy;
    uint64 m_memTfawBusy;
    uint64 m_memReadWriteBusy;
    uint64 m_memDataBusBusy;
    uint64 m_memRefresh;
    uint64 m_memRead;
    uint64 m_memWrite;
    uint64 m_memWaitCycles;
    uint64 m_memInputQ;
    uint64 m_memBankQ;
    uint64 m_memArbWait;
    uint64 m_memRandBusy;
    uint64 m_memNotOld;
    std::vector<uint64> m_memBankCount;
    int m_banks_per_rank;
    int m_ranks_per_dimm;
    int m_dimms_per_channel;
};

inline std::ostream&
operator<<(std::ostream& out, const MemCntrlProfiler& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_PROFILER_MEMCNTRLPROFILER_HH__
