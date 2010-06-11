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

#include "base/stl_helpers.hh"
#include "mem/gems_common/Map.hh"
#include "mem/gems_common/PrioHeap.hh"
#include "mem/protocol/CacheMsg.hh"
#include "mem/ruby/profiler/AccessTraceForAddress.hh"
#include "mem/ruby/profiler/AddressProfiler.hh"
#include "mem/ruby/profiler/Profiler.hh"
#include "mem/ruby/system/System.hh"

using namespace std;
typedef AddressProfiler::AddressMap AddressMap;

using m5::stl_helpers::operator<<;

// Helper functions
AccessTraceForAddress&
lookupTraceForAddress(const Address& addr, AddressMap* record_map)
{
    if (!record_map->exist(addr)) {
        record_map->add(addr, AccessTraceForAddress(addr));
    }
    return record_map->lookup(addr);
}

void
printSorted(ostream& out, int num_of_sequencers, const AddressMap* record_map,
            string description)
{
    const int records_printed = 100;

    uint64 misses = 0;
    PrioHeap<AccessTraceForAddress*> heap;
    std::vector<Address> keys = record_map->keys();
    for (int i = 0; i < keys.size(); i++) {
        AccessTraceForAddress* record = &(record_map->lookup(keys[i]));
        misses += record->getTotal();
        heap.insert(record);
    }

    out << "Total_entries_" << description << ": " << keys.size() << endl;
    if (g_system_ptr->getProfiler()->getAllInstructions())
        out << "Total_Instructions_" << description << ": " << misses << endl;
    else
        out << "Total_data_misses_" << description << ": " << misses << endl;

    out << "total | load store atomic | user supervisor | sharing | touched-by"
        << endl;

    Histogram remaining_records(1, 100);
    Histogram all_records(1, 100);
    Histogram remaining_records_log(-1);
    Histogram all_records_log(-1);

    // Allows us to track how many lines where touched by n processors
    std::vector<int64> m_touched_vec;
    std::vector<int64> m_touched_weighted_vec;
    m_touched_vec.resize(num_of_sequencers+1);
    m_touched_weighted_vec.resize(num_of_sequencers+1);
    for (int i = 0; i < m_touched_vec.size(); i++) {
        m_touched_vec[i] = 0;
        m_touched_weighted_vec[i] = 0;
    }

    int counter = 0;
    while (heap.size() > 0 && counter < records_printed) {
        AccessTraceForAddress* record = heap.extractMin();
        double percent = 100.0 * (record->getTotal() / double(misses));
        out << description << " | " << percent << " % " << *record << endl;
        all_records.add(record->getTotal());
        all_records_log.add(record->getTotal());
        counter++;
        m_touched_vec[record->getTouchedBy()]++;
        m_touched_weighted_vec[record->getTouchedBy()] += record->getTotal();
    }

    while (heap.size() > 0) {
        AccessTraceForAddress* record = heap.extractMin();
        all_records.add(record->getTotal());
        remaining_records.add(record->getTotal());
        all_records_log.add(record->getTotal());
        remaining_records_log.add(record->getTotal());
        m_touched_vec[record->getTouchedBy()]++;
        m_touched_weighted_vec[record->getTouchedBy()] += record->getTotal();
    }
    out << endl;
    out << "all_records_" << description << ": "
        << all_records << endl
        << "all_records_log_" << description << ": "
        << all_records_log << endl
        << "remaining_records_" << description << ": "
        << remaining_records << endl
        << "remaining_records_log_" << description << ": "
        << remaining_records_log << endl
        << "touched_by_" << description << ": "
        << m_touched_vec << endl
        << "touched_by_weighted_" << description << ": "
        << m_touched_weighted_vec << endl
        << endl;
}

AddressProfiler::AddressProfiler(int num_of_sequencers)
{
    m_dataAccessTrace = new AddressMap;
    m_macroBlockAccessTrace = new AddressMap;
    m_programCounterAccessTrace = new AddressMap;
    m_retryProfileMap = new AddressMap;
    m_num_of_sequencers = num_of_sequencers;
    clearStats();
}

AddressProfiler::~AddressProfiler()
{
    delete m_dataAccessTrace;
    delete m_macroBlockAccessTrace;
    delete m_programCounterAccessTrace;
    delete m_retryProfileMap;
}

void
AddressProfiler::setHotLines(bool hot_lines)
{
    m_hot_lines = hot_lines;
}

void
AddressProfiler::setAllInstructions(bool all_instructions)
{
    m_all_instructions = all_instructions;
}

void
AddressProfiler::printStats(ostream& out) const
{
    if (m_hot_lines) {
        out << endl;
        out << "AddressProfiler Stats" << endl;
        out << "---------------------" << endl;

        out << endl;
        out << "sharing_misses: " << m_sharing_miss_counter << endl;
        out << "getx_sharing_histogram: " << m_getx_sharing_histogram << endl;
        out << "gets_sharing_histogram: " << m_gets_sharing_histogram << endl;

        out << endl;
        out << "Hot Data Blocks" << endl;
        out << "---------------" << endl;
        out << endl;
        printSorted(out, m_num_of_sequencers, m_dataAccessTrace,
                    "block_address");

        out << endl;
        out << "Hot MacroData Blocks" << endl;
        out << "--------------------" << endl;
        out << endl;
        printSorted(out, m_num_of_sequencers, m_macroBlockAccessTrace,
                    "macroblock_address");

        out << "Hot Instructions" << endl;
        out << "----------------" << endl;
        out << endl;
        printSorted(out, m_num_of_sequencers, m_programCounterAccessTrace,
                    "pc_address");
    }

    if (m_all_instructions) {
        out << endl;
        out << "All Instructions Profile:" << endl;
        out << "-------------------------" << endl;
        out << endl;
        printSorted(out, m_num_of_sequencers, m_programCounterAccessTrace,
                    "pc_address");
        out << endl;
    }

    if (m_retryProfileHisto.size() > 0) {
        out << "Retry Profile" << endl;
        out << "-------------" << endl;
        out << endl;
        out << "retry_histogram_absolute: " << m_retryProfileHisto << endl;
        out << "retry_histogram_write: " << m_retryProfileHistoWrite << endl;
        out << "retry_histogram_read: " << m_retryProfileHistoRead << endl;

        out << "retry_histogram_percent: ";
        m_retryProfileHisto.printPercent(out);
        out << endl;

        printSorted(out, m_num_of_sequencers, m_retryProfileMap,
                    "block_address");
        out << endl;
    }
}

void
AddressProfiler::clearStats()
{
    // Clear the maps
    m_sharing_miss_counter = 0;
    m_dataAccessTrace->clear();
    m_macroBlockAccessTrace->clear();
    m_programCounterAccessTrace->clear();
    m_retryProfileMap->clear();
    m_retryProfileHisto.clear();
    m_retryProfileHistoRead.clear();
    m_retryProfileHistoWrite.clear();
    m_getx_sharing_histogram.clear();
    m_gets_sharing_histogram.clear();
}

void
AddressProfiler::profileGetX(const Address& datablock, const Address& PC,
                             const Set& owner, const Set& sharers,
                             NodeID requestor)
{
    Set indirection_set;
    indirection_set.addSet(sharers);
    indirection_set.addSet(owner);
    indirection_set.remove(requestor);
    int num_indirections = indirection_set.count();

    m_getx_sharing_histogram.add(num_indirections);
    bool indirection_miss = (num_indirections > 0);

    addTraceSample(datablock, PC, CacheRequestType_ST, AccessModeType(0),
                   requestor, indirection_miss);
}

void
AddressProfiler::profileGetS(const Address& datablock, const Address& PC,
                             const Set& owner, const Set& sharers,
                             NodeID requestor)
{
    Set indirection_set;
    indirection_set.addSet(owner);
    indirection_set.remove(requestor);
    int num_indirections = indirection_set.count();

    m_gets_sharing_histogram.add(num_indirections);
    bool indirection_miss = (num_indirections > 0);

    addTraceSample(datablock, PC, CacheRequestType_LD, AccessModeType(0),
                   requestor, indirection_miss);
}

void
AddressProfiler::addTraceSample(Address data_addr, Address pc_addr,
                                CacheRequestType type,
                                AccessModeType access_mode, NodeID id,
                                bool sharing_miss)
{
    if (m_all_instructions) {
        if (sharing_miss) {
            m_sharing_miss_counter++;
        }

        // record data address trace info
        data_addr.makeLineAddress();
        lookupTraceForAddress(data_addr, m_dataAccessTrace).
            update(type, access_mode, id, sharing_miss);

        // record macro data address trace info

        // 6 for datablock, 4 to make it 16x more coarse
        Address macro_addr(data_addr.maskLowOrderBits(10));
        lookupTraceForAddress(macro_addr, m_macroBlockAccessTrace).
            update(type, access_mode, id, sharing_miss);

        // record program counter address trace info
        lookupTraceForAddress(pc_addr, m_programCounterAccessTrace).
            update(type, access_mode, id, sharing_miss);
    }

    if (m_all_instructions) {
        // This code is used if the address profiler is an
        // all-instructions profiler record program counter address
        // trace info
        lookupTraceForAddress(pc_addr, m_programCounterAccessTrace).
            update(type, access_mode, id, sharing_miss);
    }
}

void
AddressProfiler::profileRetry(const Address& data_addr, AccessType type,
                              int count)
{
    m_retryProfileHisto.add(count);
    if (type == AccessType_Read) {
        m_retryProfileHistoRead.add(count);
    } else {
        m_retryProfileHistoWrite.add(count);
    }
    if (count > 1) {
        lookupTraceForAddress(data_addr, m_retryProfileMap).addSample(count);
    }
}
