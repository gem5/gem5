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

#include "mem/ruby/profiler/AddressProfiler.hh"

#include <vector>

#include "base/bitfield.hh"
#include "base/stl_helpers.hh"
#include "mem/ruby/profiler/Profiler.hh"
#include "mem/ruby/protocol/RubyRequest.hh"

namespace gem5
{

namespace ruby
{

typedef AddressProfiler::AddressMap AddressMap;

using gem5::stl_helpers::operator<<;

// Helper functions
AccessTraceForAddress&
lookupTraceForAddress(Addr addr, AddressMap& record_map)
{
    // we create a static default object here that is used to insert
    // since the insertion will create a copy of the object in the
    // process.  Perhaps this is optimizing early, but it doesn't seem
    // like it could hurt.
    static const AccessTraceForAddress dflt;

    std::pair<AddressMap::iterator, bool> r =
        record_map.insert(std::make_pair(addr, dflt));
    AddressMap::iterator i = r.first;
    AccessTraceForAddress &access_trace = i->second;
    if (r.second) {
        // there was nothing there and the insert succeed, so we need
        // to actually set the address.
        access_trace.setAddress(addr);
    }

    return access_trace;
}

void
printSorted(std::ostream& out, int num_of_sequencers,
        const AddressMap &record_map, std::string description,
        Profiler *profiler)
{
    const int records_printed = 100;

    uint64_t misses = 0;
    std::vector<const AccessTraceForAddress *> sorted;

    AddressMap::const_iterator i = record_map.begin();
    AddressMap::const_iterator end = record_map.end();
    for (; i != end; ++i) {
        const AccessTraceForAddress* record = &i->second;
        misses += record->getTotal();
        sorted.push_back(record);
    }
    sort(sorted.begin(), sorted.end(), AccessTraceForAddress::less_equal);

    out << "Total_entries_" << description << ": " << record_map.size()
        << std::endl;
    if (profiler->getAllInstructions()) {
        out << "Total_Instructions_" << description << ": " << misses
            << std::endl;
    } else {
        out << "Total_data_misses_" << description << ": " << misses
            << std::endl;
    }

    out << "total | load store atomic | user supervisor | sharing | touched-by"
        << std::endl;

    Histogram remaining_records(1, 100);
    Histogram all_records(1, 100);
    Histogram remaining_records_log(-1);
    Histogram all_records_log(-1);

    // Allows us to track how many lines where touched by n processors
    std::vector<int64_t> m_touched_vec;
    std::vector<int64_t> m_touched_weighted_vec;
    m_touched_vec.resize(num_of_sequencers+1);
    m_touched_weighted_vec.resize(num_of_sequencers+1);
    for (int j = 0; j < m_touched_vec.size(); j++) {
        m_touched_vec[j] = 0;
        m_touched_weighted_vec[j] = 0;
    }

    int counter = 0;
    int max = sorted.size();
    while (counter < max && counter < records_printed) {
        const AccessTraceForAddress* record = sorted[counter];
        double percent = 100.0 * (record->getTotal() / double(misses));
        out << description << " | " << percent << " % " << *record
            << std::endl;
        all_records.add(record->getTotal());
        all_records_log.add(record->getTotal());
        counter++;
        m_touched_vec[record->getTouchedBy()]++;
        m_touched_weighted_vec[record->getTouchedBy()] += record->getTotal();
    }

    while (counter < max) {
        const AccessTraceForAddress* record = sorted[counter];
        all_records.add(record->getTotal());
        remaining_records.add(record->getTotal());
        all_records_log.add(record->getTotal());
        remaining_records_log.add(record->getTotal());
        m_touched_vec[record->getTouchedBy()]++;
        m_touched_weighted_vec[record->getTouchedBy()] += record->getTotal();
    }
    out << std::endl;
    out << "all_records_" << description << ": "
        << all_records << std::endl
        << "all_records_log_" << description << ": "
        << all_records_log << std::endl
        << "remaining_records_" << description << ": "
        << remaining_records << std::endl
        << "remaining_records_log_" << description << ": "
        << remaining_records_log << std::endl
        << "touched_by_" << description << ": "
        << m_touched_vec << std::endl
        << "touched_by_weighted_" << description << ": "
        << m_touched_weighted_vec << std::endl
        << std::endl;
}

AddressProfiler::AddressProfiler(int num_of_sequencers, Profiler *profiler)
    : m_profiler(profiler)
{
    m_num_of_sequencers = num_of_sequencers;
    clearStats();
}

AddressProfiler::~AddressProfiler()
{
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
AddressProfiler::printStats(std::ostream& out) const
{
    if (m_hot_lines) {
        out << std::endl;
        out << "AddressProfiler Stats" << std::endl;
        out << "---------------------" << std::endl;

        out << std::endl;
        out << "sharing_misses: " << m_sharing_miss_counter << std::endl;
        out << "getx_sharing_histogram: " << m_getx_sharing_histogram
            << std::endl;
        out << "gets_sharing_histogram: " << m_gets_sharing_histogram
            << std::endl;

        out << std::endl;
        out << "Hot Data Blocks" << std::endl;
        out << "---------------" << std::endl;
        out << std::endl;
        printSorted(out, m_num_of_sequencers, m_dataAccessTrace,
                    "block_address", m_profiler);

        out << std::endl;
        out << "Hot MacroData Blocks" << std::endl;
        out << "--------------------" << std::endl;
        out << std::endl;
        printSorted(out, m_num_of_sequencers, m_macroBlockAccessTrace,
                    "macroblock_address", m_profiler);

        out << "Hot Instructions" << std::endl;
        out << "----------------" << std::endl;
        out << std::endl;
        printSorted(out, m_num_of_sequencers, m_programCounterAccessTrace,
                    "pc_address", m_profiler);
    }

    if (m_all_instructions) {
        out << std::endl;
        out << "All Instructions Profile:" << std::endl;
        out << "-------------------------" << std::endl;
        out << std::endl;
        printSorted(out, m_num_of_sequencers, m_programCounterAccessTrace,
                    "pc_address", m_profiler);
        out << std::endl;
    }

    if (m_retryProfileHisto.size() > 0) {
        out << "Retry Profile" << std::endl;
        out << "-------------" << std::endl;
        out << std::endl;
        out << "retry_histogram_absolute: " << m_retryProfileHisto
            << std::endl;
        out << "retry_histogram_write: " << m_retryProfileHistoWrite
            << std::endl;
        out << "retry_histogram_read: " << m_retryProfileHistoRead
            << std::endl;

        out << "retry_histogram_percent: ";
        m_retryProfileHisto.printPercent(out);
        out << std::endl;

        printSorted(out, m_num_of_sequencers, m_retryProfileMap,
                    "block_address", m_profiler);
        out << std::endl;
    }
}

void
AddressProfiler::clearStats()
{
    // Clear the maps
    m_sharing_miss_counter = 0;
    m_dataAccessTrace.clear();
    m_macroBlockAccessTrace.clear();
    m_programCounterAccessTrace.clear();
    m_retryProfileMap.clear();
    m_retryProfileHisto.clear();
    m_retryProfileHistoRead.clear();
    m_retryProfileHistoWrite.clear();
    m_getx_sharing_histogram.clear();
    m_gets_sharing_histogram.clear();
}

void
AddressProfiler::profileGetX(Addr datablock, Addr PC,
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

    addTraceSample(datablock, PC, RubyRequestType_ST, RubyAccessMode(0),
                   requestor, indirection_miss);
}

void
AddressProfiler::profileGetS(Addr datablock, Addr PC,
                             const Set& owner, const Set& sharers,
                             NodeID requestor)
{
    Set indirection_set;
    indirection_set.addSet(owner);
    indirection_set.remove(requestor);
    int num_indirections = indirection_set.count();

    m_gets_sharing_histogram.add(num_indirections);
    bool indirection_miss = (num_indirections > 0);

    addTraceSample(datablock, PC, RubyRequestType_LD, RubyAccessMode(0),
                   requestor, indirection_miss);
}

void
AddressProfiler::addTraceSample(Addr data_addr, Addr pc_addr,
                                RubyRequestType type,
                                RubyAccessMode access_mode, NodeID id,
                                bool sharing_miss)
{
    if (m_all_instructions) {
        if (sharing_miss) {
            m_sharing_miss_counter++;
        }

        // record data address trace info
        data_addr = makeLineAddress(data_addr);
        lookupTraceForAddress(data_addr, m_dataAccessTrace).
            update(type, access_mode, id, sharing_miss);

        // record macro data address trace info

        // 6 for datablock, 4 to make it 16x more coarse
        Addr macro_addr = mbits<Addr>(data_addr, 63, 10);
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
AddressProfiler::profileRetry(Addr data_addr, AccessType type, int count)
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

} // namespace ruby
} // namespace gem5
