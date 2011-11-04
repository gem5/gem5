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

#ifndef __MEM_RUBY_PROFILER_PROFILER_HH__
#define __MEM_RUBY_PROFILER_PROFILER_HH__

#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "base/hashmap.hh"
#include "mem/protocol/AccessType.hh"
#include "mem/protocol/GenericMachineType.hh"
#include "mem/protocol/GenericRequestType.hh"
#include "mem/protocol/PrefetchBit.hh"
#include "mem/protocol/RubyAccessMode.hh"
#include "mem/protocol/RubyRequestType.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/common/Global.hh"
#include "mem/ruby/common/Histogram.hh"
#include "mem/ruby/common/Set.hh"
#include "mem/ruby/system/MachineID.hh"
#include "mem/ruby/system/MemoryControl.hh"
#include "params/RubyProfiler.hh"
#include "sim/sim_object.hh"

class RubyRequest;
class AddressProfiler;

class Profiler : public SimObject, public Consumer
{
  public:
    typedef RubyProfilerParams Params;
    Profiler(const Params *);
    ~Profiler();

    void wakeup();

    void setPeriodicStatsFile(const std::string& filename);
    void setPeriodicStatsInterval(integer_t period);

    void printStats(std::ostream& out, bool short_stats=false);
    void printShortStats(std::ostream& out) { printStats(out, true); }
    void printTraceStats(std::ostream& out) const;
    void clearStats();
    void printConfig(std::ostream& out) const;
    void printResourceUsage(std::ostream& out) const;

    AddressProfiler* getAddressProfiler() { return m_address_profiler_ptr; }
    AddressProfiler* getInstructionProfiler() { return m_inst_profiler_ptr; }

    void addAddressTraceSample(const RubyRequest& msg, NodeID id);

    void profileRequest(const std::string& requestStr);
    void profileSharing(const Address& addr, AccessType type,
                        NodeID requestor, const Set& sharers,
                        const Set& owner);

    void profileMulticastRetry(const Address& addr, int count);

    void profileFilterAction(int action);

    void profileConflictingRequests(const Address& addr);

    void
    profileOutstandingRequest(int outstanding)
    {
        m_outstanding_requests.add(outstanding);
    }

    void
    profileOutstandingPersistentRequest(int outstanding)
    {
        m_outstanding_persistent_requests.add(outstanding);
    }

    void
    profileAverageLatencyEstimate(int latency)
    {
        m_average_latency_estimate.add(latency);
    }

    void recordPrediction(bool wasGood, bool wasPredicted);

    void startTransaction(int cpu);
    void endTransaction(int cpu);
    void profilePFWait(Time waitTime);

    void controllerBusy(MachineID machID);
    void bankBusy();

    void missLatency(Time t, 
                     RubyRequestType type,
                     const GenericMachineType respondingMach);

    void missLatencyWcc(Time issuedTime,
                        Time initialRequestTime,
                        Time forwardRequestTime,
                        Time firstResponseTime,
                        Time completionTime);
    
    void missLatencyDir(Time issuedTime,
                        Time initialRequestTime,
                        Time forwardRequestTime,
                        Time firstResponseTime,
                        Time completionTime);
    
    void swPrefetchLatency(Time t, 
                           RubyRequestType type,
                           const GenericMachineType respondingMach);

    void sequencerRequests(int num) { m_sequencer_requests.add(num); }

    void profileMsgDelay(int virtualNetwork, int delayCycles);

    void print(std::ostream& out) const;

    void rubyWatch(int proc);
    bool watchAddress(Address addr);

    // return Ruby's start time
    Time
    getRubyStartTime()
    {
        return m_ruby_start;
    }

    // added by SS
    bool getHotLines() { return m_hot_lines; }
    bool getAllInstructions() { return m_all_instructions; }

  private:
    // Private copy constructor and assignment operator
    Profiler(const Profiler& obj);
    Profiler& operator=(const Profiler& obj);

    AddressProfiler* m_address_profiler_ptr;
    AddressProfiler* m_inst_profiler_ptr;

    std::vector<int64> m_instructions_executed_at_start;
    std::vector<int64> m_cycles_executed_at_start;

    std::ostream* m_periodic_output_file_ptr;
    integer_t m_stats_period;

    Time m_ruby_start;
    time_t m_real_time_start_time;

    std::vector<std::vector<integer_t> > m_busyControllerCount;
    integer_t m_busyBankCount;
    Histogram m_multicast_retry_histogram;

    Histogram m_filter_action_histogram;
    Histogram m_tbeProfile;

    Histogram m_sequencer_requests;
    Histogram m_read_sharing_histogram;
    Histogram m_write_sharing_histogram;
    Histogram m_all_sharing_histogram;
    int64 m_cache_to_cache;
    int64 m_memory_to_cache;

    Histogram m_prefetchWaitHistogram;

    std::vector<Histogram> m_missLatencyHistograms;
    std::vector<Histogram> m_machLatencyHistograms;
    std::vector< std::vector<Histogram> > m_missMachLatencyHistograms;
    Histogram m_wCCIssueToInitialRequestHistogram;
    Histogram m_wCCInitialRequestToForwardRequestHistogram;
    Histogram m_wCCForwardRequestToFirstResponseHistogram;
    Histogram m_wCCFirstResponseToCompleteHistogram;
    int64 m_wCCIncompleteTimes;
    Histogram m_dirIssueToInitialRequestHistogram;
    Histogram m_dirInitialRequestToForwardRequestHistogram;
    Histogram m_dirForwardRequestToFirstResponseHistogram;
    Histogram m_dirFirstResponseToCompleteHistogram;
    int64 m_dirIncompleteTimes;

    Histogram m_allMissLatencyHistogram;

    Histogram m_allSWPrefetchLatencyHistogram;
    Histogram m_SWPrefetchL2MissLatencyHistogram;
    std::vector<Histogram> m_SWPrefetchLatencyHistograms;
    std::vector<Histogram> m_SWPrefetchMachLatencyHistograms;

    Histogram m_delayedCyclesHistogram;
    Histogram m_delayedCyclesNonPFHistogram;
    std::vector<Histogram> m_delayedCyclesVCHistograms;

    Histogram m_outstanding_requests;
    Histogram m_outstanding_persistent_requests;

    Histogram m_average_latency_estimate;

    m5::hash_set<Address> m_watch_address_set;
    // counts all initiated cache request including PUTs
    int m_requests;
    std::map<std::string, int> m_requestProfileMap;

    //added by SS
    bool m_hot_lines;
    bool m_all_instructions;

    int m_num_of_sequencers;
};

inline std::ostream&
operator<<(std::ostream& out, const Profiler& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_PROFILER_PROFILER_HH__


