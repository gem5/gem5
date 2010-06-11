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

#include "mem/ruby/profiler/MemCntrlProfiler.hh"

using namespace std;

MemCntrlProfiler::MemCntrlProfiler(const string& description,
    int banks_per_rank, int ranks_per_dimm, int dimms_per_channel)
{
    m_description = description;
    m_banks_per_rank = banks_per_rank;
    m_ranks_per_dimm = ranks_per_dimm;
    m_dimms_per_channel = dimms_per_channel;

    int totalBanks = banks_per_rank * ranks_per_dimm * dimms_per_channel;
    m_memBankCount.resize(totalBanks);

    clearStats();
}

MemCntrlProfiler::~MemCntrlProfiler()
{
}

void
MemCntrlProfiler::printStats(ostream& out) const
{
    if (!m_memReq && !m_memRefresh) {
        out << "Memory Controller: " << m_description
            << " no stats recorded." << endl
            << endl
            << endl;
        return;
    }

    // if there's a memory controller at all
    uint64 total_stalls = m_memInputQ + m_memBankQ + m_memWaitCycles;
    double stallsPerReq = total_stalls * 1.0 / m_memReq;
    out << "Memory controller: " << m_description << ":" << endl;

    // does not include refreshes
    out << "  memory_total_requests: " << m_memReq << endl;
    out << "  memory_reads: " << m_memRead << endl;
    out << "  memory_writes: " << m_memWrite << endl;
    out << "  memory_refreshes: " << m_memRefresh << endl;
    out << "  memory_total_request_delays: " << total_stalls << endl;
    out << "  memory_delays_per_request: " << stallsPerReq << endl;
    out << "  memory_delays_in_input_queue: " << m_memInputQ << endl;
    out << "  memory_delays_behind_head_of_bank_queue: "
        << m_memBankQ << endl;
    out << "  memory_delays_stalled_at_head_of_bank_queue: "
        << m_memWaitCycles << endl;

    // Note: The following "memory stalls" entries are a breakdown of
    // the cycles which already showed up in m_memWaitCycles.  The
    // order is significant; it is the priority of attributing the
    // cycles.  For example, bank_busy is before arbitration because
    // if the bank was busy, we didn't even check arbitration.
    // Note: "not old enough" means that since we grouped waiting
    // heads-of-queues into batches to avoid starvation, a request in
    // a newer batch didn't try to arbitrate yet because there are
    // older requests waiting.
    out << "  memory_stalls_for_bank_busy: " << m_memBankBusy << endl;
    out << "  memory_stalls_for_random_busy: " << m_memRandBusy << endl;
    out << "  memory_stalls_for_anti_starvation: " << m_memNotOld << endl;
    out << "  memory_stalls_for_arbitration: " << m_memArbWait << endl;
    out << "  memory_stalls_for_bus: " << m_memBusBusy << endl;
    out << "  memory_stalls_for_tfaw: " << m_memTfawBusy << endl;
    out << "  memory_stalls_for_read_write_turnaround: "
        << m_memReadWriteBusy << endl;
    out << "  memory_stalls_for_read_read_turnaround: "
        << m_memDataBusBusy << endl;
    out << "  accesses_per_bank: ";

    for (int bank = 0; bank < m_memBankCount.size(); bank++) {
        out << m_memBankCount[bank] << "  ";
    }
    out << endl;
    out << endl;
}

void
MemCntrlProfiler::clearStats()
{
    m_memReq = 0;
    m_memBankBusy = 0;
    m_memBusBusy = 0;
    m_memTfawBusy = 0;
    m_memReadWriteBusy = 0;
    m_memDataBusBusy = 0;
    m_memRefresh = 0;
    m_memRead = 0;
    m_memWrite = 0;
    m_memWaitCycles = 0;
    m_memInputQ = 0;
    m_memBankQ = 0;
    m_memArbWait = 0;
    m_memRandBusy = 0;
    m_memNotOld = 0;

    for (int bank = 0; bank < m_memBankCount.size(); bank++) {
        m_memBankCount[bank] = 0;
    }
}

void
MemCntrlProfiler::profileMemReq(int bank)
{
    m_memReq++;
    m_memBankCount[bank]++;
}

void
MemCntrlProfiler::profileMemBankBusy()
{
    m_memBankBusy++;
}

void
MemCntrlProfiler::profileMemBusBusy()
{
    m_memBusBusy++;
}

void
MemCntrlProfiler::profileMemReadWriteBusy()
{
    m_memReadWriteBusy++;
}

void
MemCntrlProfiler::profileMemDataBusBusy()
{
    m_memDataBusBusy++;
}

void
MemCntrlProfiler::profileMemTfawBusy()
{
    m_memTfawBusy++;
}

void
MemCntrlProfiler::profileMemRefresh()
{
    m_memRefresh++;
}

void
MemCntrlProfiler::profileMemRead()
{
    m_memRead++;
}

void
MemCntrlProfiler::profileMemWrite()
{
    m_memWrite++;
}

void
MemCntrlProfiler::profileMemWaitCycles(int cycles)
{
    m_memWaitCycles += cycles;
}

void
MemCntrlProfiler::profileMemInputQ(int cycles)
{
    m_memInputQ += cycles;
}

void
MemCntrlProfiler::profileMemBankQ(int cycles)
{
    m_memBankQ += cycles;
}

void
MemCntrlProfiler::profileMemArbWait(int cycles)
{
    m_memArbWait += cycles;
}

void
MemCntrlProfiler::profileMemRandBusy()
{
    m_memRandBusy++;
}

void
MemCntrlProfiler::profileMemNotOld()
{
    m_memNotOld++;
}


