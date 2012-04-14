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

/*
   This file has been modified by Kevin Moore and Dan Nussbaum of the
   Scalable Systems Research Group at Sun Microsystems Laboratories
   (http://research.sun.com/scalable/) to support the Adaptive
   Transactional Memory Test Platform (ATMTP).

   Please send email to atmtp-interest@sun.com with feedback, questions, or
   to request future announcements about ATMTP.

   ----------------------------------------------------------------------

   File modification date: 2008-02-23

   ----------------------------------------------------------------------
*/

// Allows use of times() library call, which determines virtual runtime
#include <sys/resource.h>
#include <sys/times.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <fstream>

#include "base/stl_helpers.hh"
#include "base/str.hh"
#include "mem/protocol/MachineType.hh"
#include "mem/protocol/RubyRequest.hh"
#include "mem/ruby/network/Network.hh"
#include "mem/ruby/profiler/AddressProfiler.hh"
#include "mem/ruby/profiler/Profiler.hh"
#include "mem/ruby/system/System.hh"

using namespace std;
using m5::stl_helpers::operator<<;

static double process_memory_total();
static double process_memory_resident();

Profiler::Profiler(const Params *p)
    : SimObject(p)
{
    m_inst_profiler_ptr = NULL;
    m_address_profiler_ptr = NULL;

    m_real_time_start_time = time(NULL); // Not reset in clearStats()
    m_stats_period = 1000000; // Default
    m_periodic_output_file_ptr = &cerr;

    m_hot_lines = p->hot_lines;
    m_all_instructions = p->all_instructions;

    m_num_of_sequencers = p->num_of_sequencers;

    m_hot_lines = false;
    m_all_instructions = false;

    m_address_profiler_ptr = new AddressProfiler(m_num_of_sequencers);
    m_address_profiler_ptr->setHotLines(m_hot_lines);
    m_address_profiler_ptr->setAllInstructions(m_all_instructions);

    if (m_all_instructions) {
        m_inst_profiler_ptr = new AddressProfiler(m_num_of_sequencers);
        m_inst_profiler_ptr->setHotLines(m_hot_lines);
        m_inst_profiler_ptr->setAllInstructions(m_all_instructions);
    }

    p->ruby_system->registerProfiler(this);
}

Profiler::~Profiler()
{
    if (m_periodic_output_file_ptr != &cerr) {
        delete m_periodic_output_file_ptr;
    }
}

void
Profiler::wakeup()
{
    // FIXME - avoid the repeated code

    vector<integer_t> perProcCycleCount(m_num_of_sequencers);

    for (int i = 0; i < m_num_of_sequencers; i++) {
        perProcCycleCount[i] =
            g_system_ptr->getCycleCount(i) - m_cycles_executed_at_start[i] + 1;
        // The +1 allows us to avoid division by zero
    }

    ostream &out = *m_periodic_output_file_ptr;

    out << "ruby_cycles: " << g_eventQueue_ptr->getTime()-m_ruby_start << endl
        << "mbytes_resident: " << process_memory_resident() << endl
        << "mbytes_total: " << process_memory_total() << endl;

    if (process_memory_total() > 0) {
        out << "resident_ratio: "
            << process_memory_resident() / process_memory_total() << endl;
    }

    out << "miss_latency: " << m_allMissLatencyHistogram << endl;

    out << endl;

    if (m_all_instructions) {
        m_inst_profiler_ptr->printStats(out);
    }

    //g_system_ptr->getNetwork()->printStats(out);
    g_eventQueue_ptr->scheduleEvent(this, m_stats_period);
}

void
Profiler::setPeriodicStatsFile(const string& filename)
{
    cout << "Recording periodic statistics to file '" << filename << "' every "
         << m_stats_period << " Ruby cycles" << endl;

    if (m_periodic_output_file_ptr != &cerr) {
        delete m_periodic_output_file_ptr;
    }

    m_periodic_output_file_ptr = new ofstream(filename.c_str());
    g_eventQueue_ptr->scheduleEvent(this, 1);
}

void
Profiler::setPeriodicStatsInterval(integer_t period)
{
    cout << "Recording periodic statistics every " << m_stats_period
         << " Ruby cycles" << endl;

    m_stats_period = period;
    g_eventQueue_ptr->scheduleEvent(this, 1);
}

void
Profiler::printConfig(ostream& out) const
{
    out << endl;
    out << "Profiler Configuration" << endl;
    out << "----------------------" << endl;
    out << "periodic_stats_period: " << m_stats_period << endl;
}

void
Profiler::print(ostream& out) const
{
    out << "[Profiler]";
}

void
Profiler::printStats(ostream& out, bool short_stats)
{
    out << endl;
    if (short_stats) {
        out << "SHORT ";
    }
    out << "Profiler Stats" << endl;
    out << "--------------" << endl;

    time_t real_time_current = time(NULL);
    double seconds = difftime(real_time_current, m_real_time_start_time);
    double minutes = seconds / 60.0;
    double hours = minutes / 60.0;
    double days = hours / 24.0;
    Time ruby_cycles = g_eventQueue_ptr->getTime()-m_ruby_start;

    if (!short_stats) {
        out << "Elapsed_time_in_seconds: " << seconds << endl;
        out << "Elapsed_time_in_minutes: " << minutes << endl;
        out << "Elapsed_time_in_hours: " << hours << endl;
        out << "Elapsed_time_in_days: " << days << endl;
        out << endl;
    }

    // print the virtual runtimes as well
    struct tms vtime;
    times(&vtime);
    seconds = (vtime.tms_utime + vtime.tms_stime) / 100.0;
    minutes = seconds / 60.0;
    hours = minutes / 60.0;
    days = hours / 24.0;
    out << "Virtual_time_in_seconds: " << seconds << endl;
    out << "Virtual_time_in_minutes: " << minutes << endl;
    out << "Virtual_time_in_hours:   " << hours << endl;
    out << "Virtual_time_in_days:    " << days << endl;
    out << endl;

    out << "Ruby_current_time: " << g_eventQueue_ptr->getTime() << endl;
    out << "Ruby_start_time: " << m_ruby_start << endl;
    out << "Ruby_cycles: " << ruby_cycles << endl;
    out << endl;

    if (!short_stats) {
        out << "mbytes_resident: " << process_memory_resident() << endl;
        out << "mbytes_total: " << process_memory_total() << endl;
        if (process_memory_total() > 0) {
            out << "resident_ratio: "
                << process_memory_resident()/process_memory_total() << endl;
        }
        out << endl;
    }

    vector<integer_t> perProcCycleCount(m_num_of_sequencers);

    for (int i = 0; i < m_num_of_sequencers; i++) {
        perProcCycleCount[i] =
            g_system_ptr->getCycleCount(i) - m_cycles_executed_at_start[i] + 1;
        // The +1 allows us to avoid division by zero
    }

    out << "ruby_cycles_executed: " << perProcCycleCount << endl;

    out << endl;

    if (!short_stats) {
        out << "Busy Controller Counts:" << endl;
        for (int i = 0; i < MachineType_NUM; i++) {
            int size = MachineType_base_count((MachineType)i);
            for (int j = 0; j < size; j++) {
                MachineID machID;
                machID.type = (MachineType)i;
                machID.num = j;
                out << machID << ":" << m_busyControllerCount[i][j] << "  ";
                if ((j + 1) % 8 == 0) {
                    out << endl;
                }
            }
            out << endl;
        }
        out << endl;

        out << "Busy Bank Count:" << m_busyBankCount << endl;
        out << endl;

        out << "sequencer_requests_outstanding: "
            << m_sequencer_requests << endl;
        out << endl;
    }

    if (!short_stats) {
        out << "All Non-Zero Cycle Demand Cache Accesses" << endl;
        out << "----------------------------------------" << endl;
        out << "miss_latency: " << m_allMissLatencyHistogram << endl;
        for (int i = 0; i < m_missLatencyHistograms.size(); i++) {
            if (m_missLatencyHistograms[i].size() > 0) {
                out << "miss_latency_" << RubyRequestType(i) << ": "
                    << m_missLatencyHistograms[i] << endl;
            }
        }
        for (int i = 0; i < m_machLatencyHistograms.size(); i++) {
            if (m_machLatencyHistograms[i].size() > 0) {
                out << "miss_latency_" << GenericMachineType(i) << ": "
                    << m_machLatencyHistograms[i] << endl;
            }
        }

        out << "miss_latency_wCC_issue_to_initial_request: " 
            << m_wCCIssueToInitialRequestHistogram << endl;
        out << "miss_latency_wCC_initial_forward_request: " 
            << m_wCCInitialRequestToForwardRequestHistogram << endl;
        out << "miss_latency_wCC_forward_to_first_response: " 
            << m_wCCForwardRequestToFirstResponseHistogram << endl;
        out << "miss_latency_wCC_first_response_to_completion: " 
            << m_wCCFirstResponseToCompleteHistogram << endl;
        out << "imcomplete_wCC_Times: " << m_wCCIncompleteTimes << endl;
        out << "miss_latency_dir_issue_to_initial_request: " 
            << m_dirIssueToInitialRequestHistogram << endl;
        out << "miss_latency_dir_initial_forward_request: " 
            << m_dirInitialRequestToForwardRequestHistogram << endl;
        out << "miss_latency_dir_forward_to_first_response: " 
            << m_dirForwardRequestToFirstResponseHistogram << endl;
        out << "miss_latency_dir_first_response_to_completion: " 
            << m_dirFirstResponseToCompleteHistogram << endl;
        out << "imcomplete_dir_Times: " << m_dirIncompleteTimes << endl;

        for (int i = 0; i < m_missMachLatencyHistograms.size(); i++) {
            for (int j = 0; j < m_missMachLatencyHistograms[i].size(); j++) {
                if (m_missMachLatencyHistograms[i][j].size() > 0) {
                    out << "miss_latency_" << RubyRequestType(i) 
                        << "_" << GenericMachineType(j) << ": "
                        << m_missMachLatencyHistograms[i][j] << endl;
                }
            }
        }

        out << endl;

        out << "All Non-Zero Cycle SW Prefetch Requests" << endl;
        out << "------------------------------------" << endl;
        out << "prefetch_latency: " << m_allSWPrefetchLatencyHistogram << endl;
        for (int i = 0; i < m_SWPrefetchLatencyHistograms.size(); i++) {
            if (m_SWPrefetchLatencyHistograms[i].size() > 0) {
                out << "prefetch_latency_" << RubyRequestType(i) << ": "
                    << m_SWPrefetchLatencyHistograms[i] << endl;
            }
        }
        for (int i = 0; i < m_SWPrefetchMachLatencyHistograms.size(); i++) {
            if (m_SWPrefetchMachLatencyHistograms[i].size() > 0) {
                out << "prefetch_latency_" << GenericMachineType(i) << ": "
                    << m_SWPrefetchMachLatencyHistograms[i] << endl;
            }
        }
        out << "prefetch_latency_L2Miss:"
            << m_SWPrefetchL2MissLatencyHistogram << endl;

        if (m_all_sharing_histogram.size() > 0) {
            out << "all_sharing: " << m_all_sharing_histogram << endl;
            out << "read_sharing: " << m_read_sharing_histogram << endl;
            out << "write_sharing: " << m_write_sharing_histogram << endl;

            out << "all_sharing_percent: ";
            m_all_sharing_histogram.printPercent(out);
            out << endl;

            out << "read_sharing_percent: ";
            m_read_sharing_histogram.printPercent(out);
            out << endl;

            out << "write_sharing_percent: ";
            m_write_sharing_histogram.printPercent(out);
            out << endl;

            int64 total_miss = m_cache_to_cache +  m_memory_to_cache;
            out << "all_misses: " << total_miss << endl;
            out << "cache_to_cache_misses: " << m_cache_to_cache << endl;
            out << "memory_to_cache_misses: " << m_memory_to_cache << endl;
            out << "cache_to_cache_percent: "
                << 100.0 * (double(m_cache_to_cache) / double(total_miss))
                << endl;
            out << "memory_to_cache_percent: "
                << 100.0 * (double(m_memory_to_cache) / double(total_miss))
                << endl;
            out << endl;
        }

        if (m_outstanding_requests.size() > 0) {
            out << "outstanding_requests: ";
            m_outstanding_requests.printPercent(out);
            out << endl;
            out << endl;
        }
    }

    if (!short_stats) {
        out << "Request vs. RubySystem State Profile" << endl;
        out << "--------------------------------" << endl;
        out << endl;

        map<string, int>::const_iterator i = m_requestProfileMap.begin();
        map<string, int>::const_iterator end = m_requestProfileMap.end();
        for (; i != end; ++i) {
            const string &key = i->first;
            int count = i->second;

            double percent = (100.0 * double(count)) / double(m_requests);
            vector<string> items;
            tokenize(items, key, ':');
            vector<string>::iterator j = items.begin();
            vector<string>::iterator end = items.end();
            for (; j != end; ++i)
                out << setw(10) << *j;
            out << setw(11) << count;
            out << setw(14) << percent << endl;
        }
        out << endl;

        out << "filter_action: " << m_filter_action_histogram << endl;

        if (!m_all_instructions) {
            m_address_profiler_ptr->printStats(out);
        }

        if (m_all_instructions) {
            m_inst_profiler_ptr->printStats(out);
        }

        out << endl;
        out << "Message Delayed Cycles" << endl;
        out << "----------------------" << endl;
        out << "Total_delay_cycles: " <<   m_delayedCyclesHistogram << endl;
        out << "Total_nonPF_delay_cycles: "
            << m_delayedCyclesNonPFHistogram << endl;
        for (int i = 0; i < m_delayedCyclesVCHistograms.size(); i++) {
            out << "  virtual_network_" << i << "_delay_cycles: "
                << m_delayedCyclesVCHistograms[i] << endl;
        }

        printResourceUsage(out);
    }
}

void
Profiler::printResourceUsage(ostream& out) const
{
    out << endl;
    out << "Resource Usage" << endl;
    out << "--------------" << endl;

    integer_t pagesize = getpagesize(); // page size in bytes
    out << "page_size: " << pagesize << endl;

    rusage usage;
    getrusage (RUSAGE_SELF, &usage);

    out << "user_time: " << usage.ru_utime.tv_sec << endl;
    out << "system_time: " << usage.ru_stime.tv_sec << endl;
    out << "page_reclaims: " << usage.ru_minflt << endl;
    out << "page_faults: " << usage.ru_majflt << endl;
    out << "swaps: " << usage.ru_nswap << endl;
    out << "block_inputs: " << usage.ru_inblock << endl;
    out << "block_outputs: " << usage.ru_oublock << endl;
}

void
Profiler::clearStats()
{
    m_ruby_start = g_eventQueue_ptr->getTime();

    m_cycles_executed_at_start.resize(m_num_of_sequencers);
    for (int i = 0; i < m_num_of_sequencers; i++) {
        if (g_system_ptr == NULL) {
            m_cycles_executed_at_start[i] = 0;
        } else {
            m_cycles_executed_at_start[i] = g_system_ptr->getCycleCount(i);
        }
    }

    m_busyControllerCount.resize(MachineType_NUM); // all machines
    for (int i = 0; i < MachineType_NUM; i++) {
        int size = MachineType_base_count((MachineType)i);
        m_busyControllerCount[i].resize(size);
        for (int j = 0; j < size; j++) {
            m_busyControllerCount[i][j] = 0;
        }
    }
    m_busyBankCount = 0;

    m_delayedCyclesHistogram.clear();
    m_delayedCyclesNonPFHistogram.clear();
    int size = RubySystem::getNetwork()->getNumberOfVirtualNetworks();
    m_delayedCyclesVCHistograms.resize(size);
    for (int i = 0; i < size; i++) {
        m_delayedCyclesVCHistograms[i].clear();
    }

    m_missLatencyHistograms.resize(RubyRequestType_NUM);
    for (int i = 0; i < m_missLatencyHistograms.size(); i++) {
        m_missLatencyHistograms[i].clear(200);
    }
    m_machLatencyHistograms.resize(GenericMachineType_NUM+1);
    for (int i = 0; i < m_machLatencyHistograms.size(); i++) {
        m_machLatencyHistograms[i].clear(200);
    }
    m_missMachLatencyHistograms.resize(RubyRequestType_NUM);
    for (int i = 0; i < m_missLatencyHistograms.size(); i++) {
        m_missMachLatencyHistograms[i].resize(GenericMachineType_NUM+1);
        for (int j = 0; j < m_missMachLatencyHistograms[i].size(); j++) {
            m_missMachLatencyHistograms[i][j].clear(200);
        }
    }
    m_allMissLatencyHistogram.clear(200);
    m_wCCIssueToInitialRequestHistogram.clear(200);
    m_wCCInitialRequestToForwardRequestHistogram.clear(200);
    m_wCCForwardRequestToFirstResponseHistogram.clear(200);
    m_wCCFirstResponseToCompleteHistogram.clear(200);
    m_wCCIncompleteTimes = 0;
    m_dirIssueToInitialRequestHistogram.clear(200);
    m_dirInitialRequestToForwardRequestHistogram.clear(200);
    m_dirForwardRequestToFirstResponseHistogram.clear(200);
    m_dirFirstResponseToCompleteHistogram.clear(200);
    m_dirIncompleteTimes = 0;

    m_SWPrefetchLatencyHistograms.resize(RubyRequestType_NUM);
    for (int i = 0; i < m_SWPrefetchLatencyHistograms.size(); i++) {
        m_SWPrefetchLatencyHistograms[i].clear(200);
    }
    m_SWPrefetchMachLatencyHistograms.resize(GenericMachineType_NUM+1);
    for (int i = 0; i < m_SWPrefetchMachLatencyHistograms.size(); i++) {
        m_SWPrefetchMachLatencyHistograms[i].clear(200);
    }
    m_allSWPrefetchLatencyHistogram.clear(200);

    m_sequencer_requests.clear();
    m_read_sharing_histogram.clear();
    m_write_sharing_histogram.clear();
    m_all_sharing_histogram.clear();
    m_cache_to_cache = 0;
    m_memory_to_cache = 0;

    // clear HashMaps
    m_requestProfileMap.clear();

    // count requests profiled
    m_requests = 0;

    m_outstanding_requests.clear();
    m_outstanding_persistent_requests.clear();

    // Flush the prefetches through the system - used so that there
    // are no outstanding requests after stats are cleared
    //g_eventQueue_ptr->triggerAllEvents();

    // update the start time
    m_ruby_start = g_eventQueue_ptr->getTime();
}

void
Profiler::addAddressTraceSample(const RubyRequest& msg, NodeID id)
{
    if (msg.getType() != RubyRequestType_IFETCH) {
        // Note: The following line should be commented out if you
        // want to use the special profiling that is part of the GS320
        // protocol

        // NOTE: Unless PROFILE_HOT_LINES is enabled, nothing will be
        // profiled by the AddressProfiler
        m_address_profiler_ptr->
            addTraceSample(msg.getLineAddress(), msg.getProgramCounter(),
                           msg.getType(), msg.getAccessMode(), id, false);
    }
}

void
Profiler::profileSharing(const Address& addr, AccessType type,
                         NodeID requestor, const Set& sharers,
                         const Set& owner)
{
    Set set_contacted(owner);
    if (type == AccessType_Write) {
        set_contacted.addSet(sharers);
    }
    set_contacted.remove(requestor);
    int number_contacted = set_contacted.count();

    if (type == AccessType_Write) {
        m_write_sharing_histogram.add(number_contacted);
    } else {
        m_read_sharing_histogram.add(number_contacted);
    }
    m_all_sharing_histogram.add(number_contacted);

    if (number_contacted == 0) {
        m_memory_to_cache++;
    } else {
        m_cache_to_cache++;
    }
}

void
Profiler::profileMsgDelay(int virtualNetwork, int delayCycles)
{
    assert(virtualNetwork < m_delayedCyclesVCHistograms.size());
    m_delayedCyclesHistogram.add(delayCycles);
    m_delayedCyclesVCHistograms[virtualNetwork].add(delayCycles);
    if (virtualNetwork != 0) {
        m_delayedCyclesNonPFHistogram.add(delayCycles);
    }
}

// profiles original cache requests including PUTs
void
Profiler::profileRequest(const string& requestStr)
{
    m_requests++;

    // if it doesn't exist, conveniently, it will be created with the
    // default value which is 0
    m_requestProfileMap[requestStr]++;
}

void
Profiler::controllerBusy(MachineID machID)
{
    m_busyControllerCount[(int)machID.type][(int)machID.num]++;
}

void
Profiler::profilePFWait(Time waitTime)
{
    m_prefetchWaitHistogram.add(waitTime);
}

void
Profiler::bankBusy()
{
    m_busyBankCount++;
}

// non-zero cycle demand request
void
Profiler::missLatency(Time cycles, 
                      RubyRequestType type,
                      const GenericMachineType respondingMach)
{
    m_allMissLatencyHistogram.add(cycles);
    m_missLatencyHistograms[type].add(cycles);
    m_machLatencyHistograms[respondingMach].add(cycles);
    m_missMachLatencyHistograms[type][respondingMach].add(cycles);
}

void
Profiler::missLatencyWcc(Time issuedTime,
                         Time initialRequestTime,
                         Time forwardRequestTime,
                         Time firstResponseTime,
                         Time completionTime)
{
    if ((issuedTime <= initialRequestTime) &&
        (initialRequestTime <= forwardRequestTime) &&
        (forwardRequestTime <= firstResponseTime) &&
        (firstResponseTime <= completionTime)) {
        m_wCCIssueToInitialRequestHistogram.add(initialRequestTime - issuedTime);
        
        m_wCCInitialRequestToForwardRequestHistogram.add(forwardRequestTime - 
                                                         initialRequestTime);
        
        m_wCCForwardRequestToFirstResponseHistogram.add(firstResponseTime - 
                                                        forwardRequestTime);
        
        m_wCCFirstResponseToCompleteHistogram.add(completionTime - 
                                                  firstResponseTime);
    } else {
        m_wCCIncompleteTimes++;
    }
}

void
Profiler::missLatencyDir(Time issuedTime,
                         Time initialRequestTime,
                         Time forwardRequestTime,
                         Time firstResponseTime,
                         Time completionTime)
{
    if ((issuedTime <= initialRequestTime) &&
        (initialRequestTime <= forwardRequestTime) &&
        (forwardRequestTime <= firstResponseTime) &&
        (firstResponseTime <= completionTime)) {
        m_dirIssueToInitialRequestHistogram.add(initialRequestTime - issuedTime);
        
        m_dirInitialRequestToForwardRequestHistogram.add(forwardRequestTime - 
                                                         initialRequestTime);
        
        m_dirForwardRequestToFirstResponseHistogram.add(firstResponseTime - 
                                                        forwardRequestTime);
        
        m_dirFirstResponseToCompleteHistogram.add(completionTime - 
                                                  firstResponseTime);
    } else {
        m_dirIncompleteTimes++;
    }
}

// non-zero cycle prefetch request
void
Profiler::swPrefetchLatency(Time cycles, 
                            RubyRequestType type,
                            const GenericMachineType respondingMach)
{
    m_allSWPrefetchLatencyHistogram.add(cycles);
    m_SWPrefetchLatencyHistograms[type].add(cycles);
    m_SWPrefetchMachLatencyHistograms[respondingMach].add(cycles);
    if (respondingMach == GenericMachineType_Directory ||
        respondingMach == GenericMachineType_NUM) {
        m_SWPrefetchL2MissLatencyHistogram.add(cycles);
    }
}

// Helper function
static double
process_memory_total()
{
    // 4kB page size, 1024*1024 bytes per MB,
    const double MULTIPLIER = 4096.0 / (1024.0 * 1024.0);
    ifstream proc_file;
    proc_file.open("/proc/self/statm");
    int total_size_in_pages = 0;
    int res_size_in_pages = 0;
    proc_file >> total_size_in_pages;
    proc_file >> res_size_in_pages;
    return double(total_size_in_pages) * MULTIPLIER; // size in megabytes
}

static double
process_memory_resident()
{
    // 4kB page size, 1024*1024 bytes per MB,
    const double MULTIPLIER = 4096.0 / (1024.0 * 1024.0);
    ifstream proc_file;
    proc_file.open("/proc/self/statm");
    int total_size_in_pages = 0;
    int res_size_in_pages = 0;
    proc_file >> total_size_in_pages;
    proc_file >> res_size_in_pages;
    return double(res_size_in_pages) * MULTIPLIER; // size in megabytes
}

void
Profiler::rubyWatch(int id)
{
    uint64 tr = 0;
    Address watch_address = Address(tr);

    DPRINTFN("%7s %3s RUBY WATCH %d\n", g_eventQueue_ptr->getTime(), id,
        watch_address);

    // don't care about success or failure
    m_watch_address_set.insert(watch_address);
}

bool
Profiler::watchAddress(Address addr)
{
    return m_watch_address_set.count(addr) > 0;
}

Profiler *
RubyProfilerParams::create()
{
    return new Profiler(this);
}
