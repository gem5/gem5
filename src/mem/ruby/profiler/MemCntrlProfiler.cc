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
}

MemCntrlProfiler::~MemCntrlProfiler()
{
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

void
MemCntrlProfiler::regStats()
{
    m_memReq
        .name(m_description + ".memReq")
        .desc("Total number of memory requests")
        .flags(Stats::nozero)
        ;

    m_memRead
        .name(m_description + ".memRead")
        .desc("Number of memory reads")
        .flags(Stats::nozero)
        ;

    m_memWrite
        .name(m_description + ".memWrite")
        .desc("Number of memory writes")
        .flags(Stats::nozero)
        ;

    m_memRefresh
        .name(m_description + ".memRefresh")
        .desc("Number of memory refreshes")
        .flags(Stats::nozero)
        ;

    m_memInputQ
        .name(m_description + ".memInputQ")
        .desc("Delay in the input queue")
        .flags(Stats::nozero)
        ;

    m_memBankQ
        .name(m_description + ".memBankQ")
        .desc("Delay behind the head of the bank queue")
        .flags(Stats::nozero)
        ;

    m_memWaitCycles
        .name(m_description + ".memWaitCycles")
        .desc("Delay stalled at the head of the bank queue")
        .flags(Stats::nozero)
        ;

    m_totalStalls
        .name(m_description + ".totalStalls")
        .desc("Total number of stall cycles")
        .flags(Stats::nozero)
        ;

    m_totalStalls = m_memInputQ + m_memBankQ + m_memWaitCycles;

    m_stallsPerReq
        .name(m_description + ".stallsPerReq")
        .desc("Expected number of stall cycles per request")
        .flags(Stats::nozero)
        ;

    m_stallsPerReq = m_totalStalls / m_memReq;

    // Note: The following "memory stalls" entries are a breakdown of
    // the cycles which already showed up in m_memWaitCycles.  The
    // order is significant; it is the priority of attributing the
    // cycles.  For example, bank_busy is before arbitration because
    // if the bank was busy, we didn't even check arbitration.
    // Note: "not old enough" means that since we grouped waiting
    // heads-of-queues into batches to avoid starvation, a request in
    // a newer batch didn't try to arbitrate yet because there are
    // older requests waiting.
    m_memBankBusy
        .name(m_description + ".memBankBusy")
        .desc("memory stalls due to busy bank")
        .flags(Stats::nozero)
        ;

    m_memRandBusy
        .name(m_description + ".memRandBusy")
        .desc("memory stalls due to busy random")
        .flags(Stats::nozero)
        ;

    m_memNotOld
        .name(m_description + ".memNotOld")
        .desc("memory stalls due to anti starvation")
        .flags(Stats::nozero)
        ;

    m_memArbWait
        .name(m_description + ".memArbWait")
        .desc("memory stalls due to arbitration")
        .flags(Stats::nozero)
        ;

    m_memBusBusy
        .name(m_description + ".memBusBusy")
        .desc("memory stalls due to busy bus")
        .flags(Stats::nozero)
        ;

    m_memTfawBusy
        .name(m_description + ".memTfawBusy")
        .desc("memory stalls for Tfaw")
        .flags(Stats::nozero)
        ;

    m_memReadWriteBusy
        .name(m_description + ".memReadWriteBusy")
        .desc("memory stalls due to read write turnaround")
        .flags(Stats::nozero)
        ;

    m_memDataBusBusy
        .name(m_description + ".memDataBusBusy")
        .desc("memory stalls due to read read turnaround")
        .flags(Stats::nozero)
        ;

    int totalBanks = m_banks_per_rank * m_ranks_per_dimm * m_dimms_per_channel;
    m_memBankCount
        .init(totalBanks)
        .name(m_description + ".memBankCount")
        .desc("Number of accesses per bank")
        .flags(Stats::pdf | Stats::total|Stats::oneline)
        ;
    for (int i = 0; i < totalBanks; i++) {
        m_memBankCount.subname(i, "");
    }
}

