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

/*
 * Profiler.hh
 *
 * Description:
 *
 * $Id$
 *
 */

#ifndef PROFILER_H
#define PROFILER_H

#include "mem/ruby/common/Global.hh"
#include "mem/protocol/GenericMachineType.hh"
#include "mem/ruby/config/RubyConfig.hh"
#include "mem/ruby/common/Histogram.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/protocol/AccessModeType.hh"
#include "mem/protocol/AccessType.hh"
#include "mem/ruby/system/NodeID.hh"
#include "mem/ruby/system/MachineID.hh"
#include "mem/protocol/PrefetchBit.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/common/Set.hh"
#include "mem/protocol/CacheRequestType.hh"
#include "mem/protocol/GenericRequestType.hh"

class CacheMsg;
class CacheProfiler;
class AddressProfiler;

template <class KEY_TYPE, class VALUE_TYPE> class Map;

class Profiler : public Consumer {
public:
    // Constructors
    Profiler();

    // Destructor
    ~Profiler();

    // Public Methods
    void wakeup();

    void setPeriodicStatsFile(const string& filename);
    void setPeriodicStatsInterval(integer_t period);

    void printStats(ostream& out, bool short_stats=false);
    void printShortStats(ostream& out) { printStats(out, true); }
    void printTraceStats(ostream& out) const;
    void clearStats();
    void printConfig(ostream& out) const;
    void printResourceUsage(ostream& out) const;

    AddressProfiler* getAddressProfiler() { return m_address_profiler_ptr; }
    AddressProfiler* getInstructionProfiler() { return m_inst_profiler_ptr; }

    void addPrimaryStatSample(const CacheMsg& msg, NodeID id);
    void addSecondaryStatSample(GenericRequestType requestType,
                                AccessModeType type, int msgSize,
                                PrefetchBit pfBit, NodeID id);
    void addSecondaryStatSample(CacheRequestType requestType,
                                AccessModeType type, int msgSize,
                                PrefetchBit pfBit, NodeID id);
    void addAddressTraceSample(const CacheMsg& msg, NodeID id);

    void profileRequest(const string& requestStr);
    void profileSharing(const Address& addr, AccessType type,
                        NodeID requestor, const Set& sharers,
                        const Set& owner);

    void profileMulticastRetry(const Address& addr, int count);

    void profileFilterAction(int action);

    void profileConflictingRequests(const Address& addr);
    void profileOutstandingRequest(int outstanding) {
        m_outstanding_requests.add(outstanding);
    }

    void profileOutstandingPersistentRequest(int outstanding) {
        m_outstanding_persistent_requests.add(outstanding);
    }
    void profileAverageLatencyEstimate(int latency) {
        m_average_latency_estimate.add(latency);
    }

    void countBAUnicast() { m_num_BA_unicasts++; }
    void countBABroadcast() { m_num_BA_broadcasts++; }

    void recordPrediction(bool wasGood, bool wasPredicted);

    void startTransaction(int cpu);
    void endTransaction(int cpu);
    void profilePFWait(Time waitTime);

    void controllerBusy(MachineID machID);
    void bankBusy();
    void missLatency(Time t, CacheRequestType type,
                     GenericMachineType respondingMach);
    void swPrefetchLatency(Time t, CacheRequestType type,
                           GenericMachineType respondingMach);
    void stopTableUsageSample(int num) { m_stopTableProfile.add(num); }
    void L1tbeUsageSample(int num) { m_L1tbeProfile.add(num); }
    void L2tbeUsageSample(int num) { m_L2tbeProfile.add(num); }
    void sequencerRequests(int num) { m_sequencer_requests.add(num); }
    void storeBuffer(int size, int blocks) {
        m_store_buffer_size.add(size);
        m_store_buffer_blocks.add(blocks);
    }

    void profileGetXMaskPrediction(const Set& pred_set);
    void profileGetSMaskPrediction(const Set& pred_set);
    void profileTrainingMask(const Set& pred_set);
    void profileTransition(const string& component, NodeID id, NodeID version,
                           Address addr, const string& state,
                           const string& event, const string& next_state,
                           const string& note);
    void profileMsgDelay(int virtualNetwork, int delayCycles);

    void print(ostream& out) const;

    int64 getTotalInstructionsExecuted() const;
    int64 getTotalTransactionsExecuted() const;

    Time getRubyStartTime(){
      return m_ruby_start;
    }

    // added for MemoryControl:
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

private:
    // Private Methods
    void addL2StatSample(GenericRequestType requestType, AccessModeType type,
                         int msgSize, PrefetchBit pfBit, NodeID id);
    void addL1DStatSample(const CacheMsg& msg, NodeID id);
    void addL1IStatSample(const CacheMsg& msg, NodeID id);

    GenericRequestType CacheRequestType_to_GenericRequestType(const CacheRequestType& type);

    // Private copy constructor and assignment operator
    Profiler(const Profiler& obj);
    Profiler& operator=(const Profiler& obj);

    // Data Members (m_ prefix)
    CacheProfiler* m_L1D_cache_profiler_ptr;
    CacheProfiler* m_L1I_cache_profiler_ptr;
    CacheProfiler* m_L2_cache_profiler_ptr;
    AddressProfiler* m_address_profiler_ptr;
    AddressProfiler* m_inst_profiler_ptr;

    Vector<int64> m_instructions_executed_at_start;
    Vector<int64> m_cycles_executed_at_start;

    ostream* m_periodic_output_file_ptr;
    integer_t m_stats_period;

    Time m_ruby_start;
    time_t m_real_time_start_time;

    int m_num_BA_unicasts;
    int m_num_BA_broadcasts;

    Vector<integer_t> m_perProcTotalMisses;
    Vector<integer_t> m_perProcUserMisses;
    Vector<integer_t> m_perProcSupervisorMisses;
    Vector<integer_t> m_perProcStartTransaction;
    Vector<integer_t> m_perProcEndTransaction;
    Vector < Vector < integer_t > > m_busyControllerCount;
    integer_t m_busyBankCount;
    Histogram m_multicast_retry_histogram;

    Histogram m_L1tbeProfile;
    Histogram m_L2tbeProfile;
    Histogram m_stopTableProfile;

    Histogram m_filter_action_histogram;
    Histogram m_tbeProfile;

    Histogram m_sequencer_requests;
    Histogram m_store_buffer_size;
    Histogram m_store_buffer_blocks;
    Histogram m_read_sharing_histogram;
    Histogram m_write_sharing_histogram;
    Histogram m_all_sharing_histogram;
    int64 m_cache_to_cache;
    int64 m_memory_to_cache;

    Histogram m_prefetchWaitHistogram;

    Vector<Histogram> m_missLatencyHistograms;
    Vector<Histogram> m_machLatencyHistograms;
    Histogram m_L2MissLatencyHistogram;
    Histogram m_allMissLatencyHistogram;

    Histogram  m_allSWPrefetchLatencyHistogram;
    Histogram  m_SWPrefetchL2MissLatencyHistogram;
    Vector<Histogram> m_SWPrefetchLatencyHistograms;
    Vector<Histogram> m_SWPrefetchMachLatencyHistograms;

    Histogram m_delayedCyclesHistogram;
    Histogram m_delayedCyclesNonPFHistogram;
    Vector<Histogram> m_delayedCyclesVCHistograms;

    int m_predictions;
    int m_predictionOpportunities;
    int m_goodPredictions;

    Histogram m_gets_mask_prediction;
    Histogram m_getx_mask_prediction;
    Histogram m_explicit_training_mask;

    // For profiling possibly conflicting requests
    Map<Address, Time>* m_conflicting_map_ptr;
    Histogram m_conflicting_histogram;

    Histogram m_outstanding_requests;
    Histogram m_outstanding_persistent_requests;

    Histogram m_average_latency_estimate;

    Map<Address, int>* m_watch_address_list_ptr;
    // counts all initiated cache request including PUTs
    int m_requests;
    Map <string, int>* m_requestProfileMap_ptr;

    // added for MemoryControl:
    long long int m_memReq;
    long long int m_memBankBusy;
    long long int m_memBusBusy;
    long long int m_memTfawBusy;
    long long int m_memReadWriteBusy;
    long long int m_memDataBusBusy;
    long long int m_memRefresh;
    long long int m_memRead;
    long long int m_memWrite;
    long long int m_memWaitCycles;
    long long int m_memInputQ;
    long long int m_memBankQ;
    long long int m_memArbWait;
    long long int m_memRandBusy;
    long long int m_memNotOld;
    Vector<long long int> m_memBankCount;

};

// Output operator declaration
ostream& operator<<(ostream& out, const Profiler& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const Profiler& obj)
{
    obj.print(out);
    out << flush;
    return out;
}

#endif //PROFILER_H


