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
#include "mem/ruby/system/Sequencer.hh"
#include "mem/ruby/system/System.hh"

using namespace std;
using m5::stl_helpers::operator<<;

Profiler::Profiler(const Params *p)
    : SimObject(p)
{
    m_inst_profiler_ptr = NULL;
    m_address_profiler_ptr = NULL;
    m_real_time_start_time = time(NULL); // Not reset in clearStats()

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
}

void
Profiler::print(ostream& out) const
{
    out << "[Profiler]";
}

void
Profiler::printRequestProfile(ostream &out) const
{
    out << "Request vs. RubySystem State Profile" << endl;
    out << "--------------------------------" << endl;
    out << endl;

    map<string, uint64_t> m_requestProfileMap;
    uint64_t m_requests = 0;

    for (uint32_t i = 0; i < MachineType_NUM; i++) {
        for (map<uint32_t, AbstractController*>::iterator it =
                  g_abs_controls[i].begin();
             it != g_abs_controls[i].end(); ++it) {

            AbstractController *ctr = (*it).second;
            map<string, uint64_t> mp = ctr->getRequestProfileMap();

            for (map<string, uint64_t>::iterator jt = mp.begin();
                 jt != mp.end(); ++jt) {

                map<string, uint64_t>::iterator kt =
                    m_requestProfileMap.find((*jt).first);
                if (kt != m_requestProfileMap.end()) {
                    (*kt).second += (*jt).second;
                } else {
                    m_requestProfileMap[(*jt).first] = (*jt).second;
                }
            }

            m_requests += ctr->getRequestCount();
        }
    }

    map<string, uint64_t>::const_iterator i = m_requestProfileMap.begin();
    map<string, uint64_t>::const_iterator end = m_requestProfileMap.end();
    for (; i != end; ++i) {
        const string &key = i->first;
        uint64_t count = i->second;

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
}

void
Profiler::printDelayProfile(ostream &out) const
{
    out << "Message Delayed Cycles" << endl;
    out << "----------------------" << endl;

    uint32_t numVNets = Network::getNumberOfVirtualNetworks();
    Histogram delayHistogram;
    std::vector<Histogram> delayVCHistogram(numVNets);

    for (uint32_t i = 0; i < MachineType_NUM; i++) {
        for (map<uint32_t, AbstractController*>::iterator it =
                  g_abs_controls[i].begin();
             it != g_abs_controls[i].end(); ++it) {

            AbstractController *ctr = (*it).second;
            delayHistogram.add(ctr->getDelayHist());

            for (uint32_t i = 0; i < numVNets; i++) {
                delayVCHistogram[i].add(ctr->getDelayVCHist(i));
            }
        }
    }

    out << "Total_delay_cycles: " <<   delayHistogram << endl;

    for (int i = 0; i < numVNets; i++) {
        out << "  virtual_network_" << i << "_delay_cycles: "
            << delayVCHistogram[i] << endl;
    }
}

void
Profiler::printOutstandingReqProfile(ostream &out) const
{
    Histogram sequencerRequests;

    for (uint32_t i = 0; i < MachineType_NUM; i++) {
        for (map<uint32_t, AbstractController*>::iterator it =
                  g_abs_controls[i].begin();
             it != g_abs_controls[i].end(); ++it) {

            AbstractController *ctr = (*it).second;
            Sequencer *seq = ctr->getSequencer();
            if (seq != NULL) {
                sequencerRequests.add(seq->getOutstandReqHist());
            }
        }
    }

    out << "sequencer_requests_outstanding: "
        << sequencerRequests << endl;
}

void
Profiler::printMissLatencyProfile(ostream &out) const
{
    // Collate the miss latencies histograms from all the sequencers
    Histogram latency_hist;
    std::vector<Histogram> type_latency_hist(RubyRequestType_NUM);

    Histogram hit_latency_hist;
    std::vector<Histogram> hit_type_latency_hist(RubyRequestType_NUM);

    std::vector<Histogram> hit_mach_latency_hist(MachineType_NUM);
    std::vector<std::vector<Histogram> >
        hit_type_mach_latency_hist(RubyRequestType_NUM,
                               std::vector<Histogram>(MachineType_NUM));

    Histogram miss_latency_hist;
    std::vector<Histogram> miss_type_latency_hist(RubyRequestType_NUM);

    std::vector<Histogram> miss_mach_latency_hist(MachineType_NUM);
    std::vector<std::vector<Histogram> >
        miss_type_mach_latency_hist(RubyRequestType_NUM,
                               std::vector<Histogram>(MachineType_NUM));

    std::vector<Histogram> issue_to_initial_delay_hist(MachineType_NUM);
    std::vector<Histogram> initial_to_forward_delay_hist(MachineType_NUM);
    std::vector<Histogram>
        forward_to_first_response_delay_hist(MachineType_NUM);
    std::vector<Histogram>
        first_response_to_completion_delay_hist(MachineType_NUM);
    std::vector<uint64_t> incomplete_times(MachineType_NUM);

    for (uint32_t i = 0; i < MachineType_NUM; i++) {
        for (map<uint32_t, AbstractController*>::iterator it =
                  g_abs_controls[i].begin();
             it != g_abs_controls[i].end(); ++it) {

            AbstractController *ctr = (*it).second;
            Sequencer *seq = ctr->getSequencer();
            if (seq != NULL) {
                // add all the latencies
                latency_hist.add(seq->getLatencyHist());
                hit_latency_hist.add(seq->getHitLatencyHist());
                miss_latency_hist.add(seq->getMissLatencyHist());

                // add the per request type latencies
                for (uint32_t j = 0; j < RubyRequestType_NUM; ++j) {
                    type_latency_hist[j]
                        .add(seq->getTypeLatencyHist(j));
                    hit_type_latency_hist[j]
                        .add(seq->getHitTypeLatencyHist(j));
                    miss_type_latency_hist[j]
                        .add(seq->getMissTypeLatencyHist(j));
                }

                // add the per machine type miss latencies
                for (uint32_t j = 0; j < MachineType_NUM; ++j) {
                    hit_mach_latency_hist[j]
                        .add(seq->getHitMachLatencyHist(j));
                    miss_mach_latency_hist[j]
                        .add(seq->getMissMachLatencyHist(j));

                    issue_to_initial_delay_hist[j].add(
                        seq->getIssueToInitialDelayHist(MachineType(j)));

                    initial_to_forward_delay_hist[j].add(
                        seq->getInitialToForwardDelayHist(MachineType(j)));
                    forward_to_first_response_delay_hist[j].add(seq->
                        getForwardRequestToFirstResponseHist(MachineType(j)));

                    first_response_to_completion_delay_hist[j].add(seq->
                        getFirstResponseToCompletionDelayHist(MachineType(j)));
                    incomplete_times[j] +=
                        seq->getIncompleteTimes(MachineType(j));
                }

                // add the per (request, machine) type miss latencies
                for (uint32_t j = 0; j < RubyRequestType_NUM; j++) {
                    for (uint32_t k = 0; k < MachineType_NUM; k++) {
                        hit_type_mach_latency_hist[j][k].add(
                            seq->getHitTypeMachLatencyHist(j,k));
                        miss_type_mach_latency_hist[j][k].add(
                            seq->getMissTypeMachLatencyHist(j,k));
                    }
                }
            }
        }
    }

    out << "latency: " << latency_hist << endl;
    for (int i = 0; i < RubyRequestType_NUM; i++) {
        if (type_latency_hist[i].size() > 0) {
            out << "latency: " << RubyRequestType(i) << ": "
                << type_latency_hist[i] << endl;
        }
    }

    out << "hit latency: " << hit_latency_hist << endl;
    for (int i = 0; i < RubyRequestType_NUM; i++) {
        if (hit_type_latency_hist[i].size() > 0) {
            out << "hit latency: " << RubyRequestType(i) << ": "
                << hit_type_latency_hist[i] << endl;
        }
    }

    for (int i = 0; i < MachineType_NUM; i++) {
        if (hit_mach_latency_hist[i].size() > 0) {
            out << "hit latency: " << MachineType(i) << ": "
                << hit_mach_latency_hist[i] << endl;
        }
    }

    for (int i = 0; i < RubyRequestType_NUM; i++) {
        for (int j = 0; j < MachineType_NUM; j++) {
            if (hit_type_mach_latency_hist[i][j].size() > 0) {
                out << "hit latency: " << RubyRequestType(i)
                    << ": " << MachineType(j) << ": "
                    << hit_type_mach_latency_hist[i][j] << endl;
            }
        }
    }

    out << "miss latency: " << miss_latency_hist << endl;
    for (int i = 0; i < RubyRequestType_NUM; i++) {
        if (miss_type_latency_hist[i].size() > 0) {
            out << "miss latency: " << RubyRequestType(i) << ": "
                << miss_type_latency_hist[i] << endl;
        }
    }

    for (int i = 0; i < MachineType_NUM; i++) {
        if (miss_mach_latency_hist[i].size() > 0) {
            out << "miss latency: " << MachineType(i) << ": "
                << miss_mach_latency_hist[i] << endl;

            out << "miss latency: " << MachineType(i)
                << "::issue_to_initial_request: "
                << issue_to_initial_delay_hist[i] << endl;
            out << "miss latency: " << MachineType(i)
                << "::initial_to_forward_request: "
                << initial_to_forward_delay_hist[i] << endl;
            out << "miss latency: " << MachineType(i)
                << "::forward_to_first_response: "
                << forward_to_first_response_delay_hist[i] << endl;
            out << "miss latency: " << MachineType(i)
                << "::first_response_to_completion: "
                << first_response_to_completion_delay_hist[i] << endl;
            out << "incomplete times: " << incomplete_times[i] << endl;
        }
    }

    for (int i = 0; i < RubyRequestType_NUM; i++) {
        for (int j = 0; j < MachineType_NUM; j++) {
            if (miss_type_mach_latency_hist[i][j].size() > 0) {
                out << "miss latency: " << RubyRequestType(i)
                    << ": " << MachineType(j) << ": "
                    << miss_type_mach_latency_hist[i][j] << endl;
            }
        }
    }

    out << endl;
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

    Cycles ruby_cycles = g_system_ptr->curCycle()-m_ruby_start;

    out << "Ruby_current_time: " << g_system_ptr->curCycle() << endl;
    out << "Ruby_start_time: " << m_ruby_start << endl;
    out << "Ruby_cycles: " << ruby_cycles << endl;
    out << endl;

    if (!short_stats) {
        out << "Busy Controller Counts:" << endl;
        for (uint32_t i = 0; i < MachineType_NUM; i++) {
            uint32_t size = MachineType_base_count((MachineType)i);

            for (uint32_t j = 0; j < size; j++) {
                MachineID machID;
                machID.type = (MachineType)i;
                machID.num = j;

                AbstractController *ctr =
                    (*(g_abs_controls[i].find(j))).second;
                out << machID << ":" << ctr->getFullyBusyCycles() << "  ";
                if ((j + 1) % 8 == 0) {
                    out << endl;
                }
            }
            out << endl;
        }
        out << endl;

        out << "Busy Bank Count:" << m_busyBankCount << endl;
        out << endl;

        printOutstandingReqProfile(out);
        out << endl;
    }

    if (!short_stats) {
        out << "All Non-Zero Cycle Demand Cache Accesses" << endl;
        out << "----------------------------------------" << endl;
        printMissLatencyProfile(out);

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

        printRequestProfile(out);

        if (!m_all_instructions) {
            m_address_profiler_ptr->printStats(out);
        }

        if (m_all_instructions) {
            m_inst_profiler_ptr->printStats(out);
        }

        out << endl;
        printDelayProfile(out);
    }
}

void
Profiler::clearStats()
{
    m_ruby_start = g_system_ptr->curCycle();
    m_real_time_start_time = time(NULL);

    m_busyBankCount = 0;
    m_read_sharing_histogram.clear();
    m_write_sharing_histogram.clear();
    m_all_sharing_histogram.clear();
    m_cache_to_cache = 0;
    m_memory_to_cache = 0;

    // update the start time
    m_ruby_start = g_system_ptr->curCycle();
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
Profiler::bankBusy()
{
    m_busyBankCount++;
}

void
Profiler::rubyWatch(int id)
{
    uint64 tr = 0;
    Address watch_address = Address(tr);

    DPRINTFN("%7s %3s RUBY WATCH %d\n", g_system_ptr->curCycle(), id,
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
