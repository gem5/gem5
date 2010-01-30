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

#include "mem/ruby/libruby.hh"

#include "mem/ruby/common/Global.hh"
#include "mem/protocol/GenericMachineType.hh"
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
#include "mem/ruby/system/MemoryControl.hh"

#include "params/RubyProfiler.hh"
#include "sim/sim_object.hh"

class CacheMsg;
class AddressProfiler;

template <class KEY_TYPE, class VALUE_TYPE> class Map;

class Profiler : public SimObject, public Consumer {
public:
  // Constructors
    typedef RubyProfilerParams Params;
  Profiler(const Params *);

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

  void addAddressTraceSample(const CacheMsg& msg, NodeID id);

  void profileRequest(const string& requestStr);
  void profileSharing(const Address& addr, AccessType type, NodeID requestor, const Set& sharers, const Set& owner);

  void profileMulticastRetry(const Address& addr, int count);

  void profileFilterAction(int action);

  void profileConflictingRequests(const Address& addr);
  void profileOutstandingRequest(int outstanding) { m_outstanding_requests.add(outstanding); }
  void profileOutstandingPersistentRequest(int outstanding) { m_outstanding_persistent_requests.add(outstanding); }
  void profileAverageLatencyEstimate(int latency) { m_average_latency_estimate.add(latency); }

  void recordPrediction(bool wasGood, bool wasPredicted);

  void startTransaction(int cpu);
  void endTransaction(int cpu);
  void profilePFWait(Time waitTime);

  void controllerBusy(MachineID machID);
  void bankBusy();
  void missLatency(Time t, RubyRequestType type);
  void swPrefetchLatency(Time t, CacheRequestType type, GenericMachineType respondingMach);
  void sequencerRequests(int num) { m_sequencer_requests.add(num); }

  void profileTransition(const string& component, NodeID version, Address addr,
                         const string& state, const string& event,
                         const string& next_state, const string& note);
  void profileMsgDelay(int virtualNetwork, int delayCycles);

  void print(ostream& out) const;

  int64 getTotalTransactionsExecuted() const;

  void rubyWatch(int proc);
  bool watchAddress(Address addr);

  // return Ruby's start time
  Time getRubyStartTime(){
    return m_ruby_start;
  }

  //added by SS
  bool getHotLines() { return m_hot_lines; }
  bool getAllInstructions() { return m_all_instructions; }

private:

  // Private copy constructor and assignment operator
  Profiler(const Profiler& obj);
  Profiler& operator=(const Profiler& obj);

  // Data Members (m_ prefix)
  AddressProfiler* m_address_profiler_ptr;
  AddressProfiler* m_inst_profiler_ptr;

  Vector<int64> m_instructions_executed_at_start;
  Vector<int64> m_cycles_executed_at_start;

  ostream* m_periodic_output_file_ptr;
  integer_t m_stats_period;

  Time m_ruby_start;
  time_t m_real_time_start_time;

  Vector<integer_t> m_perProcTotalMisses;
  Vector<integer_t> m_perProcUserMisses;
  Vector<integer_t> m_perProcSupervisorMisses;
  Vector<integer_t> m_perProcStartTransaction;
  Vector<integer_t> m_perProcEndTransaction;
  Vector < Vector < integer_t > > m_busyControllerCount;
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

  Vector<Histogram> m_missLatencyHistograms;
  Vector<Histogram> m_machLatencyHistograms;
  Histogram m_allMissLatencyHistogram;

  Histogram  m_allSWPrefetchLatencyHistogram;
  Histogram  m_SWPrefetchL2MissLatencyHistogram;
  Vector<Histogram> m_SWPrefetchLatencyHistograms;
  Vector<Histogram> m_SWPrefetchMachLatencyHistograms;

  Histogram m_delayedCyclesHistogram;
  Histogram m_delayedCyclesNonPFHistogram;
  Vector<Histogram> m_delayedCyclesVCHistograms;

  Histogram m_outstanding_requests;
  Histogram m_outstanding_persistent_requests;

  Histogram m_average_latency_estimate;

  Map<Address, int>* m_watch_address_list_ptr;
  // counts all initiated cache request including PUTs
  int m_requests;
  Map <string, int>* m_requestProfileMap_ptr;

  //added by SS
  bool m_hot_lines;
  bool m_all_instructions;

  int m_num_of_sequencers;
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


