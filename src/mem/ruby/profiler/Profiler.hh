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
 * Profiler.h
 *
 * Description:
 *
 * $Id$
 *
 */

#ifndef PROFILER_H
#define PROFILER_H

#include "Global.hh"
#include "GenericMachineType.hh"
#include "RubyConfig.hh"
#include "Histogram.hh"
#include "Consumer.hh"
#include "AccessModeType.hh"
#include "AccessType.hh"
#include "NodeID.hh"
#include "MachineID.hh"
#include "PrefetchBit.hh"
#include "Address.hh"
#include "Set.hh"
#include "CacheRequestType.hh"
#include "GenericRequestType.hh"
//#include "XactProfiler.hh" //gem5:Arka for decomissioning og log_tm

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

  void setXactVisualizerFile(char* filename);

  void printStats(ostream& out, bool short_stats=false);
  void printShortStats(ostream& out) { printStats(out, true); }
  void printTraceStats(ostream& out) const;
  void clearStats();
  void printConfig(ostream& out) const;
  void printResourceUsage(ostream& out) const;

  AddressProfiler* getAddressProfiler() { return m_address_profiler_ptr; }
  AddressProfiler* getInstructionProfiler() { return m_inst_profiler_ptr; }
  //XactProfiler*    getXactProfiler() { return m_xact_profiler_ptr;} //gem5:Arka for decomissioning og log_tm

  void addPrimaryStatSample(const CacheMsg& msg, NodeID id);
  void addSecondaryStatSample(GenericRequestType requestType, AccessModeType type, int msgSize, PrefetchBit pfBit, NodeID id);
  void addSecondaryStatSample(CacheRequestType requestType, AccessModeType type, int msgSize, PrefetchBit pfBit, NodeID id);
  void addAddressTraceSample(const CacheMsg& msg, NodeID id);

  void profileRequest(const string& requestStr);
  void profileSharing(const Address& addr, AccessType type, NodeID requestor, const Set& sharers, const Set& owner);

  void profileMulticastRetry(const Address& addr, int count);

  void profileFilterAction(int action);

  void profileConflictingRequests(const Address& addr);
  void profileOutstandingRequest(int outstanding) { m_outstanding_requests.add(outstanding); }
  void profileOutstandingPersistentRequest(int outstanding) { m_outstanding_persistent_requests.add(outstanding); }
  void profileAverageLatencyEstimate(int latency) { m_average_latency_estimate.add(latency); }

  void countBAUnicast() { m_num_BA_unicasts++; }
  void countBABroadcast() { m_num_BA_broadcasts++; }

  void recordPrediction(bool wasGood, bool wasPredicted);

  void startTransaction(int cpu);
  void endTransaction(int cpu);
  void profilePFWait(Time waitTime);

  void controllerBusy(MachineID machID);
  void bankBusy();
  void missLatency(Time t, CacheRequestType type, GenericMachineType respondingMach);
  void swPrefetchLatency(Time t, CacheRequestType type, GenericMachineType respondingMach);
  void stopTableUsageSample(int num) { m_stopTableProfile.add(num); }
  void L1tbeUsageSample(int num) { m_L1tbeProfile.add(num); }
  void L2tbeUsageSample(int num) { m_L2tbeProfile.add(num); }
  void sequencerRequests(int num) { m_sequencer_requests.add(num); }
  void storeBuffer(int size, int blocks) { m_store_buffer_size.add(size); m_store_buffer_blocks.add(blocks);}

  void profileGetXMaskPrediction(const Set& pred_set);
  void profileGetSMaskPrediction(const Set& pred_set);
  void profileTrainingMask(const Set& pred_set);
  void profileTransition(const string& component, NodeID id, NodeID version, Address addr,
                         const string& state, const string& event,
                         const string& next_state, const string& note);
  void profileMsgDelay(int virtualNetwork, int delayCycles);

  void print(ostream& out) const;

  int64 getTotalInstructionsExecuted() const;
  int64 getTotalTransactionsExecuted() const;

  //---- begin Transactional Memory CODE
  #if 0  //gem5:Arka for decomissioning og log_tm
  void profileTransCycles(int proc, int cycles) { getXactProfiler()->profileTransCycles(proc, cycles);}
  void profileNonTransCycles(int proc, int cycles) { getXactProfiler()->profileNonTransCycles(proc, cycles);}
  void profileStallTransCycles(int proc, int cycles) { getXactProfiler()->profileStallTransCycles(proc, cycles); }
  void profileStallNonTransCycles(int proc, int cycles) { getXactProfiler()->profileStallNonTransCycles(proc, cycles); }
  void profileAbortingTransCycles(int proc, int cycles) { getXactProfiler()->profileAbortingTransCycles(proc, cycles); }
  void profileCommitingTransCycles(int proc, int cycles) { getXactProfiler()->profileCommitingTransCycles(proc, cycles); }
  void profileBarrierCycles(int proc, int cycles) { getXactProfiler()->profileBarrierCycles(proc, cycles);}
  void profileBackoffTransCycles(int proc, int cycles) { getXactProfiler()->profileBackoffTransCycles(proc, cycles); }
  void profileGoodTransCycles(int proc, int cycles) {getXactProfiler()->profileGoodTransCycles(proc, cycles); }

  #endif //gem5:Arka TODO clean up the rest of this functions as well
  void profileTransaction(int size, int logSize, int readS, int writeS, int overflow_readS, int overflow_writeS, int retries, int cycles, bool nacked, int loadMisses, int storeMisses, int instrCount, int xid);
  void profileBeginTransaction(NodeID id, int tid, int xid, int thread, Address pc, bool isOpen);
  void profileCommitTransaction(NodeID id, int tid, int xid, int thread, Address pc, bool isOpen);
  void profileLoadTransaction(NodeID id, int tid, int xid, int thread, Address addr, Address logicalAddress, Address pc);
  void profileLoad(NodeID id, int tid, int xid, int thread, Address addr, Address logicalAddress, Address pc);
  void profileStoreTransaction(NodeID id, int tid, int xid, int thread, Address addr, Address logicalAddress, Address pc);
  void profileStore(NodeID id, int tid, int xid, int thread, Address addr, Address logicalAddress, Address pc);
  void profileLoadOverflow(NodeID id, int tid, int xid, int thread, Address addr, bool l1_overflow);
  void profileStoreOverflow(NodeID id, int tid, int xid, int thread, Address addr, bool l1_overflow);
  void profileNack(NodeID id, int tid, int xid, int thread, int nacking_thread, NodeID nackedBy, Address addr, Address logicalAddress, Address pc, uint64 seq_ts, uint64 nack_ts, bool possibleCycle);
  void profileExposedConflict(NodeID id, int xid, int thread, Address addr, Address pc);
  void profileTransWB();
  void profileExtraWB();
  void profileInferredAbort();
  void profileAbortTransaction(NodeID id, int tid, int xid, int thread, int delay, int abortingThread, int abortingProc, Address addr, Address pc);
  void profileExceptionStart(bool xact, NodeID proc_no, int thread, int val, int trap_level, uinteger_t pc, uinteger_t npc);
  void profileExceptionDone(bool xact, NodeID proc_no, int thread, int val, int trap_level, uinteger_t pc, uinteger_t npc, uinteger_t tpc, uinteger_t tnpc);
  void profileTimerInterrupt(NodeID id,
                             uinteger_t tick, uinteger_t tick_cmpr,
                             uinteger_t stick, uinteger_t stick_cmpr,
                             int trap_level,
                             uinteger_t pc, uinteger_t npc,
                             uinteger_t pstate, int pil);

  void profileAbortDelayConstants(int handlerStartupDelay, int handlerPerBlockDelay);
  void profileXactChange(int procs, int cycles);
  void profileReadSet(Address addr, bool bf_filter_result, bool perfect_filter_result, NodeID id, int thread);
  void profileWriteSet(Address addr, bool bf_filter_result, bool perfect_filter_result, NodeID id, int thread);
  void profileRemoteReadSet(Address addr, bool bf_filter_result, bool perfect_filter_result, NodeID id, int thread);
  void profileRemoteWriteSet(Address addr, bool bf_filter_result, bool perfect_filter_result, NodeID id, int thread);


  void profileReadFilterBitsSet(int xid, int bits, bool isCommit);
  void profileWriteFilterBitsSet(int xid, int bits, bool isCommit);

  void printTransactionState(bool can_skip);

  void watchpointsFalsePositiveTrigger();
  void watchpointsTrueTrigger();

  void profileTransactionLogOverflow(NodeID id, Address addr, Address pc);
  void profileTransactionCacheOverflow(NodeID id, Address addr, Address pc);
  void profileGetCPS(NodeID id, uint32 cps, Address pc);
  void profileTransactionTCC(NodeID id, Address pc);
  void profileTransactionUnsupInst(NodeID id, Address pc);
  void profileTransactionSaveInst(NodeID id, Address pc);
  void profileTransactionRestoreInst(NodeID id, Address pc);

  //---- end Transactional Memory CODE

  void rubyWatch(int proc);
  bool watchAddress(Address addr);

  // return Ruby's start time
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
  void addL2StatSample(GenericRequestType requestType, AccessModeType type, int msgSize, PrefetchBit pfBit, NodeID id);
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

//  XactProfiler*   m_xact_profiler_ptr; // gem5:Arka for decomissioning of log_tm

  Vector<int64> m_instructions_executed_at_start;
  Vector<int64> m_cycles_executed_at_start;

  ostream* m_periodic_output_file_ptr;
  integer_t m_stats_period;
  std::fstream m_xact_visualizer;
  std::ostream *m_xact_visualizer_ptr;

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

  //---- begin Transactional Memory CODE
  Map <int, int>* m_procsInXactMap_ptr;

  Histogram m_xactCycles;
  Histogram m_xactLogs;
  Histogram m_xactReads;
  Histogram m_xactWrites;
  Histogram m_xactOverflowReads;
  Histogram m_xactOverflowWrites;
  Histogram m_xactOverflowTotalReads;
  Histogram m_xactOverflowTotalWrites;
  Histogram m_xactSizes;
  Histogram m_xactRetries;
  Histogram m_abortDelays;
  Histogram m_xactLoadMisses;
  Histogram m_xactStoreMisses;
  Histogram m_xactInstrCount;
  int m_xactNacked;
  int m_transactionAborts;
  int m_transWBs;
  int m_extraWBs;
  int m_abortStarupDelay;
  int m_abortPerBlockDelay;
  int m_inferredAborts;
  Map <int, int>* m_nackXIDMap_ptr;
  // pairs of XIDs involved in NACKs
  Map<int, Map<int, int> * > * m_nackXIDPairMap_ptr;
  Map <Address, int>* m_nackPCMap_ptr;
  Map <int, int>* m_xactExceptionMap_ptr;
  Map <int, int>* m_abortIDMap_ptr;
  Map <int, int>* m_commitIDMap_ptr;
  Map <int, int>* m_xactRetryIDMap_ptr;
  Map <int, int>* m_xactCyclesIDMap_ptr;
  Map <int, int>* m_xactReadSetIDMap_ptr;
  Map <int, int>* m_xactWriteSetIDMap_ptr;
  Map <int, int>* m_xactLoadMissIDMap_ptr;
  Map <int, int>* m_xactStoreMissIDMap_ptr;
  Map <int, integer_t> *m_xactInstrCountIDMap_ptr;
  Map <Address, int>* m_abortPCMap_ptr;
  Map <Address, int>* m_abortAddressMap_ptr;
  Map <Address, int>* m_readSetMatch_ptr;
  Map <Address, int>* m_readSetNoMatch_ptr;
  Map <Address, int>* m_writeSetMatch_ptr;
  Map <Address, int>* m_writeSetNoMatch_ptr;
  Map <Address, int>* m_remoteReadSetMatch_ptr;
  Map <Address, int>* m_remoteReadSetNoMatch_ptr;
  Map <Address, int>* m_remoteWriteSetMatch_ptr;
  Map <Address, int>* m_remoteWriteSetNoMatch_ptr;
  long long int m_readSetEmptyChecks;
  long long int m_readSetMatch;
  long long int m_readSetNoMatch;
  long long int m_writeSetEmptyChecks;
  long long int m_writeSetMatch;
  long long int m_writeSetNoMatch;
  Map<int, Histogram> * m_xactReadFilterBitsSetOnCommit;
  Map<int, Histogram> * m_xactReadFilterBitsSetOnAbort;
  Map<int, Histogram> * m_xactWriteFilterBitsSetOnCommit;
  Map<int, Histogram> * m_xactWriteFilterBitsSetOnAbort;

  unsigned int m_watchpointsFalsePositiveTrigger;
  unsigned int m_watchpointsTrueTrigger;

  int m_transactionUnsupInsts;
  int m_transactionSaveRestAborts;

  int m_transactionLogOverflows;
  int m_transactionCacheOverflows;

  //---- end Transactional Memory CODE

  Map<Address, int>* m_watch_address_list_ptr;
  // counts all initiated cache request including PUTs
  int m_requests;
  Map <string, int>* m_requestProfileMap_ptr;

  Time m_xact_visualizer_last;

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


