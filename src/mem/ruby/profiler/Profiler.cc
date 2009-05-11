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
 * Profiler.C
 *
 * Description: See Profiler.h
 *
 * $Id$
 *
 */

#include "Profiler.hh"
#include "CacheProfiler.hh"
#include "AddressProfiler.hh"
#include "System.hh"
#include "Network.hh"
#include "PrioHeap.hh"
#include "CacheMsg.hh"
#include "Driver.hh"
#include "Protocol.hh"
#include "util.hh"
#include "Map.hh"
#include "Debug.hh"
#include "MachineType.hh"
// #include "TransactionInterfaceManager.hh"
#include "interface.hh"
//#include "XactVisualizer.hh"  //gem5:Arka for decomissioning log_tm
//#include "XactProfiler.hh"   //gem5:Arka for decomissioning log_tm

// extern "C" {
// #include "Rock.hh"
// }

// Allows use of times() library call, which determines virtual runtime
#include <sys/times.h>

extern std::ostream * debug_cout_ptr;
extern std::ostream * xact_cout_ptr;

static double process_memory_total();
static double process_memory_resident();

Profiler::Profiler()
  : m_conflicting_histogram(-1)
{
  m_requestProfileMap_ptr = new Map<string, int>;
  m_L1D_cache_profiler_ptr = new CacheProfiler("L1D_cache");
  m_L1I_cache_profiler_ptr = new CacheProfiler("L1I_cache");

  m_L2_cache_profiler_ptr = new CacheProfiler("L2_cache");

  m_address_profiler_ptr = new AddressProfiler;
  m_inst_profiler_ptr = NULL;
  if (PROFILE_ALL_INSTRUCTIONS) {
    m_inst_profiler_ptr = new AddressProfiler;
  }

  //m_xact_profiler_ptr = new XactProfiler; //gem5:Arka for decomissioning og log_tm

  m_conflicting_map_ptr = new Map<Address, Time>;

  m_real_time_start_time = time(NULL); // Not reset in clearStats()
  m_stats_period = 1000000; // Default
  m_periodic_output_file_ptr = &cerr;
  m_xact_visualizer_ptr      = &cout;

  //---- begin XACT_MEM code
  m_xactExceptionMap_ptr = new Map<int, int>;
  m_procsInXactMap_ptr = new Map<int, int>;
  m_abortIDMap_ptr = new Map<int, int>;
  m_commitIDMap_ptr = new Map<int, int>;
  m_xactRetryIDMap_ptr = new Map<int, int>;
  m_xactCyclesIDMap_ptr = new Map<int, int>;
  m_xactReadSetIDMap_ptr = new Map<int, int>;
  m_xactWriteSetIDMap_ptr = new Map<int, int>;
  m_xactLoadMissIDMap_ptr = new Map<int, int>;
  m_xactStoreMissIDMap_ptr = new Map<int, int>;
  m_xactInstrCountIDMap_ptr = new Map<int, integer_t>;
  m_abortPCMap_ptr = new Map<Address, int>;
  m_abortAddressMap_ptr = new Map<Address, int>;
  m_nackXIDMap_ptr = new Map<int, int>;
  m_nackXIDPairMap_ptr = new Map<int, Map<int, int> * >;
  m_nackPCMap_ptr = new Map<Address, int>;
  m_watch_address_list_ptr = new Map<Address, int>;
  m_readSetMatch_ptr = new Map<Address, int>;
  m_readSetNoMatch_ptr = new Map<Address, int>;
  m_writeSetMatch_ptr = new Map<Address, int>;
  m_writeSetNoMatch_ptr = new Map<Address, int>;
  m_xactReadFilterBitsSetOnCommit = new Map<int, Histogram>;
  m_xactReadFilterBitsSetOnAbort = new Map<int, Histogram>;
  m_xactWriteFilterBitsSetOnCommit = new Map<int, Histogram>;
  m_xactWriteFilterBitsSetOnAbort = new Map<int, Histogram>;
  //---- end XACT_MEM code

  // for MemoryControl:
  m_memReq = 0;
  m_memBankBusy = 0;
  m_memBusBusy = 0;
  m_memReadWriteBusy = 0;
  m_memDataBusBusy = 0;
  m_memTfawBusy = 0;
  m_memRefresh = 0;
  m_memRead = 0;
  m_memWrite = 0;
  m_memWaitCycles = 0;
  m_memInputQ = 0;
  m_memBankQ = 0;
  m_memArbWait = 0;
  m_memRandBusy = 0;
  m_memNotOld = 0;


  int totalBanks = RubyConfig::banksPerRank()
                 * RubyConfig::ranksPerDimm()
                 * RubyConfig::dimmsPerChannel();
  m_memBankCount.setSize(totalBanks);

  clearStats();
}

Profiler::~Profiler()
{
  if (m_periodic_output_file_ptr != &cerr) {
    delete m_periodic_output_file_ptr;
  }
  delete m_address_profiler_ptr;
  delete m_L1D_cache_profiler_ptr;
  delete m_L1I_cache_profiler_ptr;
  delete m_L2_cache_profiler_ptr;
  //delete m_xact_profiler_ptr; //gem5:Arka for decomissioning of log_tm
  delete m_requestProfileMap_ptr;
  delete m_conflicting_map_ptr;
}

void Profiler::wakeup()
{
  // FIXME - avoid the repeated code

  Vector<integer_t> perProcInstructionCount;
  perProcInstructionCount.setSize(RubyConfig::numberOfProcessors());

  Vector<integer_t> perProcCycleCount;
  perProcCycleCount.setSize(RubyConfig::numberOfProcessors());

  for(int i=0; i < RubyConfig::numberOfProcessors(); i++) {
    perProcInstructionCount[i] = g_system_ptr->getDriver()->getInstructionCount(i) - m_instructions_executed_at_start[i] + 1;
    perProcCycleCount[i] = g_system_ptr->getDriver()->getCycleCount(i) - m_cycles_executed_at_start[i] + 1;
    // The +1 allows us to avoid division by zero
  }

  integer_t total_misses = m_perProcTotalMisses.sum();
  integer_t instruction_executed = perProcInstructionCount.sum();
  integer_t simics_cycles_executed = perProcCycleCount.sum();
  integer_t transactions_started = m_perProcStartTransaction.sum();
  integer_t transactions_ended = m_perProcEndTransaction.sum();

  (*m_periodic_output_file_ptr) << "ruby_cycles: " << g_eventQueue_ptr->getTime()-m_ruby_start << endl;
  (*m_periodic_output_file_ptr) << "total_misses: " << total_misses << " " << m_perProcTotalMisses << endl;
  (*m_periodic_output_file_ptr) << "instruction_executed: " << instruction_executed << " " << perProcInstructionCount << endl;
  (*m_periodic_output_file_ptr) << "simics_cycles_executed: " << simics_cycles_executed << " " << perProcCycleCount << endl;
  (*m_periodic_output_file_ptr) << "transactions_started: " << transactions_started << " " << m_perProcStartTransaction << endl;
  (*m_periodic_output_file_ptr) << "transactions_ended: " << transactions_ended << " " << m_perProcEndTransaction << endl;
  (*m_periodic_output_file_ptr) << "L1TBE_usage: " << m_L1tbeProfile << endl;
  (*m_periodic_output_file_ptr) << "L2TBE_usage: " << m_L2tbeProfile << endl;
  (*m_periodic_output_file_ptr) << "mbytes_resident: " << process_memory_resident() << endl;
  (*m_periodic_output_file_ptr) << "mbytes_total: " << process_memory_total() << endl;
  if (process_memory_total() > 0) {
    (*m_periodic_output_file_ptr) << "resident_ratio: " << process_memory_resident()/process_memory_total() << endl;
  }
  (*m_periodic_output_file_ptr) << "miss_latency: " << m_allMissLatencyHistogram << endl;

  *m_periodic_output_file_ptr << endl;

  if (PROFILE_ALL_INSTRUCTIONS) {
    m_inst_profiler_ptr->printStats(*m_periodic_output_file_ptr);
  }

  //g_system_ptr->getNetwork()->printStats(*m_periodic_output_file_ptr);
  g_eventQueue_ptr->scheduleEvent(this, m_stats_period);
}

void Profiler::setPeriodicStatsFile(const string& filename)
{
  cout << "Recording periodic statistics to file '" << filename << "' every "
       << m_stats_period << " Ruby cycles" << endl;

  if (m_periodic_output_file_ptr != &cerr) {
    delete m_periodic_output_file_ptr;
  }

  m_periodic_output_file_ptr = new ofstream(filename.c_str());
  g_eventQueue_ptr->scheduleEvent(this, 1);
}

void Profiler::setPeriodicStatsInterval(integer_t period)
{
  cout << "Recording periodic statistics every " << m_stats_period << " Ruby cycles" << endl;
  m_stats_period = period;
  g_eventQueue_ptr->scheduleEvent(this, 1);
}

void Profiler::printConfig(ostream& out) const
{
  out << endl;
  out << "Profiler Configuration" << endl;
  out << "----------------------" << endl;
  out << "periodic_stats_period: " << m_stats_period << endl;
}

void Profiler::print(ostream& out) const
{
  out << "[Profiler]";
}

void Profiler::printStats(ostream& out, bool short_stats)
{
  out << endl;
  if (short_stats) {
    out << "SHORT ";
  }
  out << "Profiler Stats" << endl;
  out << "--------------" << endl;

  time_t real_time_current = time(NULL);
  double seconds = difftime(real_time_current, m_real_time_start_time);
  double minutes = seconds/60.0;
  double hours = minutes/60.0;
  double days = hours/24.0;
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
  out << "Virtual_time_in_days:    " << hours << endl;
  out << endl;

  out << "Ruby_current_time: " << g_eventQueue_ptr->getTime() << endl;
  out << "Ruby_start_time: " << m_ruby_start << endl;
  out << "Ruby_cycles: " << ruby_cycles << endl;
  out << endl;

  if (!short_stats) {
    out << "mbytes_resident: " << process_memory_resident() << endl;
    out << "mbytes_total: " << process_memory_total() << endl;
    if (process_memory_total() > 0) {
      out << "resident_ratio: " << process_memory_resident()/process_memory_total() << endl;
    }
    out << endl;

    if(m_num_BA_broadcasts + m_num_BA_unicasts != 0){
      out << endl;
      out << "Broadcast_percent: " << (float)m_num_BA_broadcasts/(m_num_BA_broadcasts+m_num_BA_unicasts) << endl;
    }
  }

  Vector<integer_t> perProcInstructionCount;
  Vector<integer_t> perProcCycleCount;
  Vector<double> perProcCPI;
  Vector<double> perProcMissesPerInsn;
  Vector<double> perProcInsnPerTrans;
  Vector<double> perProcCyclesPerTrans;
  Vector<double> perProcMissesPerTrans;

  perProcInstructionCount.setSize(RubyConfig::numberOfProcessors());
  perProcCycleCount.setSize(RubyConfig::numberOfProcessors());
  perProcCPI.setSize(RubyConfig::numberOfProcessors());
  perProcMissesPerInsn.setSize(RubyConfig::numberOfProcessors());

  perProcInsnPerTrans.setSize(RubyConfig::numberOfProcessors());
  perProcCyclesPerTrans.setSize(RubyConfig::numberOfProcessors());
  perProcMissesPerTrans.setSize(RubyConfig::numberOfProcessors());

  for(int i=0; i < RubyConfig::numberOfProcessors(); i++) {
    perProcInstructionCount[i] = g_system_ptr->getDriver()->getInstructionCount(i) - m_instructions_executed_at_start[i] + 1;
    perProcCycleCount[i] = g_system_ptr->getDriver()->getCycleCount(i) - m_cycles_executed_at_start[i] + 1;
    // The +1 allows us to avoid division by zero
    perProcCPI[i] = double(ruby_cycles)/perProcInstructionCount[i];
    perProcMissesPerInsn[i] = 1000.0 * (double(m_perProcTotalMisses[i]) / double(perProcInstructionCount[i]));

    int trans = m_perProcEndTransaction[i];
    if (trans == 0) {
      perProcInsnPerTrans[i] = 0;
      perProcCyclesPerTrans[i] = 0;
      perProcMissesPerTrans[i] = 0;
    } else {
      perProcInsnPerTrans[i] = perProcInstructionCount[i] / double(trans);
      perProcCyclesPerTrans[i] = ruby_cycles / double(trans);
      perProcMissesPerTrans[i] = m_perProcTotalMisses[i] / double(trans);
    }
  }

  integer_t total_misses = m_perProcTotalMisses.sum();
  integer_t user_misses = m_perProcUserMisses.sum();
  integer_t supervisor_misses = m_perProcSupervisorMisses.sum();
  integer_t instruction_executed = perProcInstructionCount.sum();
  integer_t simics_cycles_executed = perProcCycleCount.sum();
  integer_t transactions_started = m_perProcStartTransaction.sum();
  integer_t transactions_ended = m_perProcEndTransaction.sum();

  double instructions_per_transaction = (transactions_ended != 0) ? double(instruction_executed) / double(transactions_ended) : 0;
  double cycles_per_transaction = (transactions_ended != 0) ? (RubyConfig::numberOfProcessors() * double(ruby_cycles)) / double(transactions_ended) : 0;
  double misses_per_transaction = (transactions_ended != 0) ? double(total_misses) / double(transactions_ended) : 0;

  out << "Total_misses: " << total_misses << endl;
  out << "total_misses: " << total_misses << " " << m_perProcTotalMisses << endl;
  out << "user_misses: " << user_misses << " " << m_perProcUserMisses << endl;
  out << "supervisor_misses: " << supervisor_misses << " " << m_perProcSupervisorMisses << endl;
  out << endl;
  out << "instruction_executed: " << instruction_executed << " " << perProcInstructionCount << endl;
  out << "simics_cycles_executed: " << simics_cycles_executed << " " << perProcCycleCount << endl;
  out << "cycles_per_instruction: " << (RubyConfig::numberOfProcessors()*double(ruby_cycles))/double(instruction_executed) << " " << perProcCPI << endl;
  out << "misses_per_thousand_instructions: " << 1000.0 * (double(total_misses) / double(instruction_executed)) << " " << perProcMissesPerInsn << endl;
  out << endl;
  out << "transactions_started: " << transactions_started << " " << m_perProcStartTransaction << endl;
  out << "transactions_ended: " << transactions_ended << " " << m_perProcEndTransaction << endl;
  out << "instructions_per_transaction: " << instructions_per_transaction << " " << perProcInsnPerTrans << endl;
  out << "cycles_per_transaction: " << cycles_per_transaction  << " " << perProcCyclesPerTrans << endl;
  out << "misses_per_transaction: " << misses_per_transaction << " " << perProcMissesPerTrans << endl;

  out << endl;

  m_L1D_cache_profiler_ptr->printStats(out);
  m_L1I_cache_profiler_ptr->printStats(out);
  m_L2_cache_profiler_ptr->printStats(out);

  out << endl;

  if (m_memReq || m_memRefresh) {    // if there's a memory controller at all
    long long int total_stalls = m_memInputQ + m_memBankQ + m_memWaitCycles;
    double stallsPerReq = total_stalls * 1.0 / m_memReq;
    out << "Memory control:" << endl;
    out << "  memory_total_requests: " << m_memReq << endl;  // does not include refreshes
    out << "  memory_reads: " << m_memRead << endl;
    out << "  memory_writes: " << m_memWrite << endl;
    out << "  memory_refreshes: " << m_memRefresh << endl;
    out << "  memory_total_request_delays: " << total_stalls << endl;
    out << "  memory_delays_per_request: " << stallsPerReq << endl;
    out << "  memory_delays_in_input_queue: " << m_memInputQ << endl;
    out << "  memory_delays_behind_head_of_bank_queue: " << m_memBankQ << endl;
    out << "  memory_delays_stalled_at_head_of_bank_queue: " << m_memWaitCycles << endl;
    // Note:  The following "memory stalls" entries are a breakdown of the
    // cycles which already showed up in m_memWaitCycles.  The order is
    // significant; it is the priority of attributing the cycles.
    // For example, bank_busy is before arbitration because if the bank was
    // busy, we didn't even check arbitration.
    // Note:  "not old enough" means that since we grouped waiting heads-of-queues
    // into batches to avoid starvation, a request in a newer batch
    // didn't try to arbitrate yet because there are older requests waiting.
    out << "  memory_stalls_for_bank_busy: " << m_memBankBusy << endl;
    out << "  memory_stalls_for_random_busy: " << m_memRandBusy << endl;
    out << "  memory_stalls_for_anti_starvation: " << m_memNotOld << endl;
    out << "  memory_stalls_for_arbitration: " << m_memArbWait << endl;
    out << "  memory_stalls_for_bus: " << m_memBusBusy << endl;
    out << "  memory_stalls_for_tfaw: " << m_memTfawBusy << endl;
    out << "  memory_stalls_for_read_write_turnaround: " << m_memReadWriteBusy << endl;
    out << "  memory_stalls_for_read_read_turnaround: " << m_memDataBusBusy << endl;
    out << "  accesses_per_bank: ";
    for (int bank=0; bank < m_memBankCount.size(); bank++) {
      out << m_memBankCount[bank] << "  ";
      //if ((bank % 8) == 7) out << "                     " << endl;
    }
    out << endl;
    out << endl;
  }

  if (!short_stats) {
    out << "Busy Controller Counts:" << endl;
    for(int i=0; i < MachineType_NUM; i++) {
      for(int j=0; j < MachineType_base_count((MachineType)i); j++) {
        MachineID machID;
        machID.type = (MachineType)i;
        machID.num = j;
        out << machID << ":" << m_busyControllerCount[i][j] << "  ";
        if ((j+1)%8 == 0) {
          out << endl;
        }
      }
      out << endl;
    }
    out << endl;

    out << "Busy Bank Count:" << m_busyBankCount << endl;
    out << endl;

    out << "L1TBE_usage: " << m_L1tbeProfile << endl;
    out << "L2TBE_usage: " << m_L2tbeProfile << endl;
    out << "StopTable_usage: " << m_stopTableProfile << endl;
    out << "sequencer_requests_outstanding: " << m_sequencer_requests << endl;
    out << "store_buffer_size: " << m_store_buffer_size << endl;
    out << "unique_blocks_in_store_buffer: " << m_store_buffer_blocks << endl;
    out << endl;
  }

  if (!short_stats) {
    out << "All Non-Zero Cycle Demand Cache Accesses" << endl;
    out << "----------------------------------------" << endl;
    out << "miss_latency: " << m_allMissLatencyHistogram << endl;
    for(int i=0; i<m_missLatencyHistograms.size(); i++) {
      if (m_missLatencyHistograms[i].size() > 0) {
        out << "miss_latency_" << CacheRequestType(i) << ": " << m_missLatencyHistograms[i] << endl;
      }
    }
    for(int i=0; i<m_machLatencyHistograms.size(); i++) {
      if (m_machLatencyHistograms[i].size() > 0) {
        out << "miss_latency_" << GenericMachineType(i) << ": " << m_machLatencyHistograms[i] << endl;
      }
    }
    out << "miss_latency_L2Miss: " << m_L2MissLatencyHistogram << endl;

    out << endl;

    out << "All Non-Zero Cycle SW Prefetch Requests" << endl;
    out << "------------------------------------" << endl;
    out << "prefetch_latency: " << m_allSWPrefetchLatencyHistogram << endl;
    for(int i=0; i<m_SWPrefetchLatencyHistograms.size(); i++) {
      if (m_SWPrefetchLatencyHistograms[i].size() > 0) {
        out << "prefetch_latency_" << CacheRequestType(i) << ": " << m_SWPrefetchLatencyHistograms[i] << endl;
      }
    }
    for(int i=0; i<m_SWPrefetchMachLatencyHistograms.size(); i++) {
      if (m_SWPrefetchMachLatencyHistograms[i].size() > 0) {
        out << "prefetch_latency_" << GenericMachineType(i) << ": " << m_SWPrefetchMachLatencyHistograms[i] << endl;
      }
    }
    out << "prefetch_latency_L2Miss:" << m_SWPrefetchL2MissLatencyHistogram << endl;

    out << "multicast_retries: " << m_multicast_retry_histogram << endl;
    out << "gets_mask_prediction_count: " << m_gets_mask_prediction << endl;
    out << "getx_mask_prediction_count: " << m_getx_mask_prediction << endl;
    out << "explicit_training_mask: " << m_explicit_training_mask << endl;
    out << endl;

    if (m_all_sharing_histogram.size() > 0) {
      out << "all_sharing: " << m_all_sharing_histogram << endl;
      out << "read_sharing: " << m_read_sharing_histogram << endl;
      out << "write_sharing: " << m_write_sharing_histogram << endl;

      out << "all_sharing_percent: "; m_all_sharing_histogram.printPercent(out); out << endl;
      out << "read_sharing_percent: "; m_read_sharing_histogram.printPercent(out); out << endl;
      out << "write_sharing_percent: "; m_write_sharing_histogram.printPercent(out); out << endl;

      int64 total_miss = m_cache_to_cache +  m_memory_to_cache;
      out << "all_misses: " << total_miss << endl;
      out << "cache_to_cache_misses: " << m_cache_to_cache << endl;
      out << "memory_to_cache_misses: " << m_memory_to_cache << endl;
      out << "cache_to_cache_percent: " << 100.0 * (double(m_cache_to_cache) / double(total_miss)) << endl;
      out << "memory_to_cache_percent: " << 100.0 * (double(m_memory_to_cache) / double(total_miss)) << endl;
      out << endl;
    }

    if (m_conflicting_histogram.size() > 0) {
      out << "conflicting_histogram: " << m_conflicting_histogram << endl;
      out << "conflicting_histogram_percent: "; m_conflicting_histogram.printPercent(out); out << endl;
      out << endl;
    }

    if (m_outstanding_requests.size() > 0) {
      out << "outstanding_requests: "; m_outstanding_requests.printPercent(out); out << endl;
      if (m_outstanding_persistent_requests.size() > 0) {
        out << "outstanding_persistent_requests: "; m_outstanding_persistent_requests.printPercent(out); out << endl;
      }
      out << endl;
    }
  }

  if (XACT_MEMORY){
    // Transactional Memory stats
    out << "Transactional Memory Stats:" << endl;
    out << "------- xact --------" << endl;
    out << "xact_size_dist: " << m_xactSizes << endl;
    out << "xact_instr_count: " << m_xactInstrCount << endl;
    out << "xact_time_dist: " << m_xactCycles << endl;
    out << "xact_log_size_dist: " << m_xactLogs << endl;
    out << "xact_read_set_size_dist: " << m_xactReads << endl;
    out << "xact_write_set_size_dist: " << m_xactWrites << endl;
    out << "xact_overflow_read_lines_dist: " << m_xactOverflowReads << endl;
    out << "xact_overflow_write_lines_dist: " << m_xactOverflowWrites << endl;
    out << "xact_overflow_read_set_size_dist: " << m_xactOverflowTotalReads << endl;
    out << "xact_overflow_write_set_size_dist: " << m_xactOverflowTotalWrites << endl;
    out << "xact_miss_load_dist: " << m_xactLoadMisses << endl;
    out << "xact_miss_store_dist: " << m_xactStoreMisses << endl;
    out << "xact_nacked: " << m_xactNacked << endl;
    out << "xact_retries:        " << m_xactRetries << endl;
    out << "xact_abort_delays: " << m_abortDelays << endl;
    out << "xact_aborts:        " << m_transactionAborts << endl;
    if (ATMTP_ENABLED) {
      out << "xact_log_overflows: " << m_transactionLogOverflows << endl;
      out << "xact_cache_overflows: " << m_transactionCacheOverflows << endl;
      out << "xact_unsup_inst_aborts: "   << m_transactionUnsupInsts << endl;
      out << "xact_save_rest_aborts: "   << m_transactionSaveRestAborts << endl;
    }
    out << "xact_writebacks:    " << m_transWBs << endl;
    out << "xact_extra_wbs:    " << m_extraWBs << endl;
    out << "xact_handler_startup_delay: " << m_abortStarupDelay << endl;
    out << "xact_handler_per_block_delay: " << m_abortPerBlockDelay << endl;
    out << "xact_inferred_aborts: " << m_inferredAborts << endl;
    //out << "xact_histogram: " << m_procsInXact << endl;

    if (!short_stats) {
      Vector<int> nackedXIDKeys = m_nackXIDMap_ptr->keys();
      nackedXIDKeys.sortVector();
      out << endl;
      int total_nacks = 0;
      out << "------- xact Nacks by XID --------" << endl;
      for(int i=0; i<nackedXIDKeys.size(); i++) {
        int key = nackedXIDKeys[i];
        int count = m_nackXIDMap_ptr->lookup(key);
        total_nacks += count;
        out << "xact " << key << " "
          << setw(6) << dec << count
          << endl;
      }
      out << "Total Nacks: " << total_nacks << endl;
      out << "---------------" << endl;
      out << endl;

      // Print XID Nack Pairs
      Vector<int> nackedXIDPairKeys = m_nackXIDPairMap_ptr->keys();
      nackedXIDPairKeys.sortVector();
      out << endl;
      total_nacks = 0;
      out << "------- xact Nacks by XID Pairs --------" << endl;
      for(int i=0; i<nackedXIDPairKeys.size(); i++) {
        int key = nackedXIDPairKeys[i];
        Map<int, int> * my_map  = m_nackXIDPairMap_ptr->lookup(key);
        Vector<int> my_keys = my_map->keys();
        my_keys.sortVector();
        for(int j=0; j<my_keys.size(); j++){
        int nid = my_keys[j];
        int count = my_map->lookup(nid);
        total_nacks += count;
        out << "xact " << key << " nacked by xact " <<  nid << " "
            << setw(6) << dec << count
            << endl;
        }
      }
      out << "Total Nacks: " << total_nacks << endl;
      out << "---------------" << endl;
      out << endl;


      Vector<Address> nackedPCKeys = m_nackPCMap_ptr->keys();
      nackedPCKeys.sortVector();
      out << endl;
      out << "------- xact Nacks by PC --------" << endl;
      for(int i=0; i<nackedPCKeys.size(); i++) {
        Address key = nackedPCKeys[i];
        int count = m_nackPCMap_ptr->lookup(key);
        out << "xact_Nack " << key << " "
          << setw(4) << dec << count
          << endl;
      }
      out << "---------------" << endl;
      out << endl;


      Vector<int> xactExceptionKeys = m_xactExceptionMap_ptr->keys();
      xactExceptionKeys.sortVector();
      out << "------- xact exceptions --------" << endl;
      for(int i=0; i<xactExceptionKeys.size(); i++) {
        int key = xactExceptionKeys[i];
        int count = m_xactExceptionMap_ptr->lookup(key);
        out << "xact_exception("
          << hex << key << "):"
          << setw(4) << dec << count
          << endl;
      }
      out << endl;
      out << "---------------" << endl;
      out << endl;

      Vector<int> abortIDKeys = m_abortIDMap_ptr->keys();
      abortIDKeys.sortVector();
      out << "------- xact abort by XID --------" << endl;
      for(int i=0; i<abortIDKeys.size(); i++) {
        int count = m_abortIDMap_ptr->lookup(abortIDKeys[i]);
        out << "xact_aborts("
          << dec << abortIDKeys[i] << "):"
          << setw(7) << count
          << endl;
      }
      out << endl;
      out << "---------------" << endl;
      out << endl;

      Vector<Address> abortedPCKeys = m_abortPCMap_ptr->keys();
      abortedPCKeys.sortVector();
      out << endl;
      out << "------- xact Aborts by PC --------" << endl;
      for(int i=0; i<abortedPCKeys.size(); i++) {
        Address key = abortedPCKeys[i];
        int count = m_abortPCMap_ptr->lookup(key);
        out << "xact_abort_pc " << key
          << setw(4) << dec << count
          << endl;
      }
      out << "---------------" << endl;
      out << endl;

      Vector<Address> abortedAddrKeys = m_abortAddressMap_ptr->keys();
      abortedAddrKeys.sortVector();
      out << endl;
      out << "------- xact Aborts by Address --------" << endl;
      for(int i=0; i<abortedAddrKeys.size(); i++) {
        Address key = abortedAddrKeys[i];
        int count = m_abortAddressMap_ptr->lookup(key);
        out << "xact_abort_address " << key
          << setw(4) << dec << count
          << endl;
      }
      out << "---------------" << endl;
      out << endl;
    } // !short_stats

    Vector<int> commitIDKeys = m_commitIDMap_ptr->keys();
    commitIDKeys.sortVector();
    out << "------- xact Commit Stats by XID --------" << endl;
    for(int i=0; i<commitIDKeys.size(); i++) {
      int count = m_commitIDMap_ptr->lookup(commitIDKeys[i]);
      double retry_count = (double)m_xactRetryIDMap_ptr->lookup(commitIDKeys[i]) / count;
      double cycles_count = (double)m_xactCyclesIDMap_ptr->lookup(commitIDKeys[i]) / count;
      double readset_count = (double)m_xactReadSetIDMap_ptr->lookup(commitIDKeys[i]) / count;
      double writeset_count = (double)m_xactWriteSetIDMap_ptr->lookup(commitIDKeys[i]) / count;
      double loadmiss_count = (double)m_xactLoadMissIDMap_ptr->lookup(commitIDKeys[i]) / count;
      double storemiss_count = (double)m_xactStoreMissIDMap_ptr->lookup(commitIDKeys[i]) / count;
      double instr_count = (double)m_xactInstrCountIDMap_ptr->lookup(commitIDKeys[i]) / count;
      out << "xact_stats id: "
          << dec << commitIDKeys[i]
          << " count: " << setw(7) << count
          << " Cycles: " << setw(7) << cycles_count
          << " Instr: " << setw(7) << instr_count
          << " ReadSet: " << setw(7) << readset_count
          << " WriteSet: " << setw(7) << writeset_count
          << " LoadMiss: " << setw(7) << loadmiss_count
          << " StoreMiss: " << setw(7) << storemiss_count
          << " Retry Count: " << setw(7) << retry_count
          << endl;
    }
    out << endl;
    out << "---------------" << endl;
    out << endl;

    if (!short_stats) {
      Vector<int> procsInXactKeys = m_procsInXactMap_ptr->keys();
      procsInXactKeys.sortVector();
      out << "------- xact histogram --------" << endl;
      for(int i=0; i<procsInXactKeys.size(); i++) {
        int count = m_procsInXactMap_ptr->lookup(procsInXactKeys[i]);
        int key = procsInXactKeys[i];
        out << "xact_histogram("
            << dec << key << "):"
            << setw(8) << count
            << endl;
      }
      out << endl;
      out << "---------------" << endl;
      out << endl;

      // Read/Write set Bloom filter stats
      //int false_reads = 0;
      long long int false_reads = m_readSetNoMatch;
      Vector<Address> fp_read_keys = m_readSetNoMatch_ptr->keys();
      out << "------- xact read set false positives -------" << endl;
      for(int i=0; i < fp_read_keys.size(); ++i){
        int count = m_readSetNoMatch_ptr->lookup(fp_read_keys[i]);
        //out << "read_false_positive( " << fp_read_keys[i] << " ): "
        //   << setw(8) << dec << count << endl;
        false_reads += count;
      }
      out << "Total read set false positives : " << setw(8) << false_reads << endl;
      out << "-----------------------" << endl;
      out << endl;

      //int matching_reads = 0;
      long long int matching_reads = m_readSetMatch;
      long long int empty_checks = m_readSetEmptyChecks;
      Vector<Address> read_keys = m_readSetMatch_ptr->keys();
      out << "------- xact read set matches -------" << endl;
      for(int i=0; i < read_keys.size(); ++i){
        int count = m_readSetMatch_ptr->lookup(read_keys[i]);
        //out << "read_match( " << read_keys[i] << " ): "
        //    << setw(8) << dec << count << endl;
        matching_reads += count;
      }
      out << "Total read set matches : " << setw(8) << matching_reads << endl;
      out << "Total read set empty checks : " << setw(8) << empty_checks << endl;
      double false_positive_pct = 0.0;
      if((false_reads + matching_reads)> 0){
        false_positive_pct = (1.0*false_reads)/(false_reads+matching_reads)*100.0;
      }
      out << "Read set false positives rate : " << false_positive_pct << "%" << endl;
      out << "-----------------------" << endl;
      out << endl;

      // for write set
      //int false_writes = 0;
      long long int false_writes = m_writeSetNoMatch;
      Vector<Address> fp_write_keys = m_writeSetNoMatch_ptr->keys();
      out << "------- xact write set false positives -------" << endl;
      for(int i=0; i < fp_write_keys.size(); ++i){
        int count = m_writeSetNoMatch_ptr->lookup(fp_write_keys[i]);
        //out << "write_false_positive( " << fp_write_keys[i] << " ): "
        //   << setw(8) << dec << count << endl;
        false_writes += count;
      }
      out << "Total write set false positives : " << setw(8) << false_writes << endl;
      out << "-----------------------" << endl;
      out << endl;

      //int matching_writes = 0;
      long long int matching_writes = m_writeSetMatch;
      empty_checks = m_writeSetEmptyChecks;
      Vector<Address> write_keys = m_writeSetMatch_ptr->keys();
      out << "------- xact write set matches -------" << endl;
      for(int i=0; i < write_keys.size(); ++i){
        int count = m_writeSetMatch_ptr->lookup(write_keys[i]);
        //out << "write_match( " << write_keys[i] << " ): "
        //    << setw(8) << dec << count << endl;
        matching_writes += count;
      }
      out << "Total write set matches : " << setw(8) << matching_writes << endl;
      out << "Total write set empty checks : " << setw(8) << empty_checks << endl;
      false_positive_pct = 0.0;
      if((matching_writes+false_writes) > 0){
        false_positive_pct = (1.0*false_writes)/(false_writes+matching_writes)*100.0;
      }
      out << "Write set false positives rate : " << false_positive_pct << "%" << endl;
      out << "-----------------------" << endl;
      out << endl;

      out << "----- Xact Signature Stats ------" << endl;
      Vector<int> xids = m_xactReadFilterBitsSetOnCommit->keys();
      for(int i=0; i < xids.size(); ++i){
        int xid = xids[i];
        out << "xid " << xid << " Read set bits set on commit: " << (m_xactReadFilterBitsSetOnCommit->lookup(xid)) << endl;
      }
      xids = m_xactWriteFilterBitsSetOnCommit->keys();
      for(int i=0; i < xids.size(); ++i){
        int xid = xids[i];
        out << "xid " << xid << " Write set bits set on commit: " << (m_xactWriteFilterBitsSetOnCommit->lookup(xid)) << endl;
      }
      xids = m_xactReadFilterBitsSetOnAbort->keys();
      for(int i=0; i < xids.size(); ++i){
        int xid = xids[i];
        out << "xid " << xid << " Read set bits set on abort: " << (m_xactReadFilterBitsSetOnAbort->lookup(xid)) << endl;
      }
      xids = m_xactWriteFilterBitsSetOnAbort->keys();
      for(int i=0; i < xids.size(); ++i){
        int xid = xids[i];
        out << "xid " << xid << " Write set bits set on abort: " << (m_xactWriteFilterBitsSetOnAbort->lookup(xid)) << endl;
      }
      out << endl;

      cout << "------- WATCHPOINTS --------" << endl;
      cout << "False Triggers : " << m_watchpointsFalsePositiveTrigger << endl;
      cout << "True  Triggers : " << m_watchpointsTrueTrigger << endl;
      cout << "Total Triggers : " << m_watchpointsTrueTrigger + m_watchpointsFalsePositiveTrigger << endl;
      cout << "---------------" << endl;
      cout << endl;
    } // !short_stats
    //m_xact_profiler_ptr->printStats(out, short_stats); // gem5:Arka for decomissioning of log_tm
  } // XACT_MEMORY

  if (!short_stats) {
    out << "Request vs. RubySystem State Profile" << endl;
    out << "--------------------------------" << endl;
    out << endl;

    Vector<string> requestProfileKeys = m_requestProfileMap_ptr->keys();
    requestProfileKeys.sortVector();

    for(int i=0; i<requestProfileKeys.size(); i++) {
      int temp_int = m_requestProfileMap_ptr->lookup(requestProfileKeys[i]);
      double percent = (100.0*double(temp_int))/double(m_requests);
      while (requestProfileKeys[i] != "") {
        out << setw(10) << string_split(requestProfileKeys[i], ':');
      }
      out << setw(11) << temp_int;
      out << setw(14) << percent << endl;
    }
    out << endl;

    out << "filter_action: " << m_filter_action_histogram << endl;

    if (!PROFILE_ALL_INSTRUCTIONS) {
      m_address_profiler_ptr->printStats(out);
    }

    if (PROFILE_ALL_INSTRUCTIONS) {
      m_inst_profiler_ptr->printStats(out);
    }

    out << endl;
    out << "Message Delayed Cycles" << endl;
    out << "----------------------" << endl;
    out << "Total_delay_cycles: " <<   m_delayedCyclesHistogram << endl;
    out << "Total_nonPF_delay_cycles: " << m_delayedCyclesNonPFHistogram << endl;
    for (int i = 0; i < m_delayedCyclesVCHistograms.size(); i++) {
      out << "  virtual_network_" << i << "_delay_cycles: " << m_delayedCyclesVCHistograms[i] << endl;
    }

    printResourceUsage(out);
  }

}

void Profiler::printResourceUsage(ostream& out) const
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

void Profiler::clearStats()
{
  m_num_BA_unicasts = 0;
  m_num_BA_broadcasts = 0;

  m_ruby_start = g_eventQueue_ptr->getTime();

  m_instructions_executed_at_start.setSize(RubyConfig::numberOfProcessors());
  m_cycles_executed_at_start.setSize(RubyConfig::numberOfProcessors());
  for (int i=0; i < RubyConfig::numberOfProcessors(); i++) {
    if (g_system_ptr == NULL) {
      m_instructions_executed_at_start[i] = 0;
      m_cycles_executed_at_start[i] = 0;
    } else {
      m_instructions_executed_at_start[i] = g_system_ptr->getDriver()->getInstructionCount(i);
      m_cycles_executed_at_start[i] = g_system_ptr->getDriver()->getCycleCount(i);
    }
  }

  m_perProcTotalMisses.setSize(RubyConfig::numberOfProcessors());
  m_perProcUserMisses.setSize(RubyConfig::numberOfProcessors());
  m_perProcSupervisorMisses.setSize(RubyConfig::numberOfProcessors());
  m_perProcStartTransaction.setSize(RubyConfig::numberOfProcessors());
  m_perProcEndTransaction.setSize(RubyConfig::numberOfProcessors());

  for(int i=0; i < RubyConfig::numberOfProcessors(); i++) {
    m_perProcTotalMisses[i] = 0;
    m_perProcUserMisses[i] = 0;
    m_perProcSupervisorMisses[i] = 0;
    m_perProcStartTransaction[i] = 0;
    m_perProcEndTransaction[i] = 0;
  }

  m_busyControllerCount.setSize(MachineType_NUM); // all machines
  for(int i=0; i < MachineType_NUM; i++) {
    m_busyControllerCount[i].setSize(MachineType_base_count((MachineType)i));
    for(int j=0; j < MachineType_base_count((MachineType)i); j++) {
      m_busyControllerCount[i][j] = 0;
    }
  }
  m_busyBankCount = 0;

  m_delayedCyclesHistogram.clear();
  m_delayedCyclesNonPFHistogram.clear();
  m_delayedCyclesVCHistograms.setSize(NUMBER_OF_VIRTUAL_NETWORKS);
  for (int i = 0; i < NUMBER_OF_VIRTUAL_NETWORKS; i++) {
    m_delayedCyclesVCHistograms[i].clear();
  }

  m_gets_mask_prediction.clear();
  m_getx_mask_prediction.clear();
  m_explicit_training_mask.clear();

  m_missLatencyHistograms.setSize(CacheRequestType_NUM);
  for(int i=0; i<m_missLatencyHistograms.size(); i++) {
    m_missLatencyHistograms[i].clear(200);
  }
  m_machLatencyHistograms.setSize(GenericMachineType_NUM+1);
  for(int i=0; i<m_machLatencyHistograms.size(); i++) {
    m_machLatencyHistograms[i].clear(200);
  }
  m_allMissLatencyHistogram.clear(200);
  m_L2MissLatencyHistogram.clear(200);

  m_SWPrefetchLatencyHistograms.setSize(CacheRequestType_NUM);
  for(int i=0; i<m_SWPrefetchLatencyHistograms.size(); i++) {
    m_SWPrefetchLatencyHistograms[i].clear(200);
  }
  m_SWPrefetchMachLatencyHistograms.setSize(GenericMachineType_NUM+1);
  for(int i=0; i<m_SWPrefetchMachLatencyHistograms.size(); i++) {
    m_SWPrefetchMachLatencyHistograms[i].clear(200);
  }
  m_allSWPrefetchLatencyHistogram.clear(200);
  m_SWPrefetchL2MissLatencyHistogram.clear(200);

  m_multicast_retry_histogram.clear();

  m_L1tbeProfile.clear();
  m_L2tbeProfile.clear();
  m_stopTableProfile.clear();
  m_filter_action_histogram.clear();

  m_sequencer_requests.clear();
  m_store_buffer_size.clear();
  m_store_buffer_blocks.clear();
  m_read_sharing_histogram.clear();
  m_write_sharing_histogram.clear();
  m_all_sharing_histogram.clear();
  m_cache_to_cache = 0;
  m_memory_to_cache = 0;

  m_predictions = 0;
  m_predictionOpportunities = 0;
  m_goodPredictions = 0;

  // clear HashMaps
  m_requestProfileMap_ptr->clear();

  // count requests profiled
  m_requests = 0;

  // Conflicting requests
  m_conflicting_map_ptr->clear();
  m_conflicting_histogram.clear();

  m_outstanding_requests.clear();
  m_outstanding_persistent_requests.clear();

  m_L1D_cache_profiler_ptr->clearStats();
  m_L1I_cache_profiler_ptr->clearStats();
  m_L2_cache_profiler_ptr->clearStats();
  //m_xact_profiler_ptr->clearStats(); //gem5:Arka for decomissiong of log_tm

  //---- begin XACT_MEM code
  ASSERT(m_xactExceptionMap_ptr != NULL);
  ASSERT(m_procsInXactMap_ptr != NULL);
  ASSERT(m_abortIDMap_ptr != NULL);
  ASSERT(m_abortPCMap_ptr != NULL);
  ASSERT( m_nackXIDMap_ptr != NULL);
  ASSERT(m_nackPCMap_ptr != NULL);

  m_abortStarupDelay = -1;
  m_abortPerBlockDelay = -1;
  m_transWBs = 0;
  m_extraWBs = 0;
  m_transactionAborts = 0;
  m_transactionLogOverflows = 0;
  m_transactionCacheOverflows = 0;
  m_transactionUnsupInsts = 0;
  m_transactionSaveRestAborts = 0;
  m_inferredAborts = 0;
  m_xactNacked = 0;

  m_xactLogs.clear();
  m_xactCycles.clear();
  m_xactReads.clear();
  m_xactWrites.clear();
  m_xactSizes.clear();
  m_abortDelays.clear();
  m_xactRetries.clear();
  m_xactOverflowReads.clear();
  m_xactOverflowWrites.clear();
  m_xactLoadMisses.clear();
  m_xactStoreMisses.clear();
  m_xactOverflowTotalReads.clear();
  m_xactOverflowTotalWrites.clear();

  m_xactExceptionMap_ptr->clear();
  m_procsInXactMap_ptr->clear();
  m_abortIDMap_ptr->clear();
  m_commitIDMap_ptr->clear();
  m_xactRetryIDMap_ptr->clear();
  m_xactCyclesIDMap_ptr->clear();
  m_xactReadSetIDMap_ptr->clear();
  m_xactWriteSetIDMap_ptr->clear();
  m_xactLoadMissIDMap_ptr->clear();
  m_xactStoreMissIDMap_ptr->clear();
  m_xactInstrCountIDMap_ptr->clear();
  m_abortPCMap_ptr->clear();
  m_abortAddressMap_ptr->clear();
  m_nackXIDMap_ptr->clear();
  m_nackXIDPairMap_ptr->clear();
  m_nackPCMap_ptr->clear();

  m_xactReadFilterBitsSetOnCommit->clear();
  m_xactReadFilterBitsSetOnAbort->clear();
  m_xactWriteFilterBitsSetOnCommit->clear();
  m_xactWriteFilterBitsSetOnAbort->clear();

  m_readSetEmptyChecks = 0;
  m_readSetMatch = 0;
  m_readSetNoMatch = 0;
  m_writeSetEmptyChecks = 0;
  m_writeSetMatch = 0;
  m_writeSetNoMatch = 0;

  m_xact_visualizer_last = 0;
  m_watchpointsFalsePositiveTrigger = 0;
  m_watchpointsTrueTrigger = 0;
  //---- end XACT_MEM code

  // for MemoryControl:
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

  for (int bank=0; bank < m_memBankCount.size(); bank++) {
    m_memBankCount[bank] = 0;
  }

  // Flush the prefetches through the system - used so that there are no outstanding requests after stats are cleared
  //g_eventQueue_ptr->triggerAllEvents();

  // update the start time
  m_ruby_start = g_eventQueue_ptr->getTime();
}

void Profiler::addPrimaryStatSample(const CacheMsg& msg, NodeID id)
{
  if (Protocol::m_TwoLevelCache) {
    if (msg.getType() == CacheRequestType_IFETCH) {
      addL1IStatSample(msg, id);
    } else {
      addL1DStatSample(msg, id);
    }
    // profile the address after an L1 miss (outside of the processor for CMP)
    if (Protocol::m_CMP) {
      addAddressTraceSample(msg, id);
    }
  } else {
    addL2StatSample(CacheRequestType_to_GenericRequestType(msg.getType()),
                    msg.getAccessMode(), msg.getSize(), msg.getPrefetch(), id);
    addAddressTraceSample(msg, id);
  }
}

void Profiler::profileConflictingRequests(const Address& addr)
{
  assert(addr == line_address(addr));
  Time last_time = m_ruby_start;
  if (m_conflicting_map_ptr->exist(addr)) {
    Time last_time = m_conflicting_map_ptr->lookup(addr);
  }
  Time current_time = g_eventQueue_ptr->getTime();
  assert (current_time - last_time > 0);
  m_conflicting_histogram.add(current_time - last_time);
  m_conflicting_map_ptr->add(addr, current_time);
}

void Profiler::addSecondaryStatSample(CacheRequestType requestType, AccessModeType type, int msgSize, PrefetchBit pfBit, NodeID id)
{
  addSecondaryStatSample(CacheRequestType_to_GenericRequestType(requestType), type, msgSize, pfBit, id);
}

void Profiler::addSecondaryStatSample(GenericRequestType requestType, AccessModeType type, int msgSize, PrefetchBit pfBit, NodeID id)
{
  addL2StatSample(requestType, type, msgSize, pfBit, id);
}

void Profiler::addL2StatSample(GenericRequestType requestType, AccessModeType type, int msgSize, PrefetchBit pfBit, NodeID id)
{
  m_perProcTotalMisses[id]++;
  if (type == AccessModeType_SupervisorMode) {
    m_perProcSupervisorMisses[id]++;
  } else {
    m_perProcUserMisses[id]++;
  }
  m_L2_cache_profiler_ptr->addStatSample(requestType, type, msgSize, pfBit);
}

void Profiler::addL1DStatSample(const CacheMsg& msg, NodeID id)
{
  m_L1D_cache_profiler_ptr->addStatSample(CacheRequestType_to_GenericRequestType(msg.getType()),
                                          msg.getAccessMode(), msg.getSize(), msg.getPrefetch());
}

void Profiler::addL1IStatSample(const CacheMsg& msg, NodeID id)
{
  m_L1I_cache_profiler_ptr->addStatSample(CacheRequestType_to_GenericRequestType(msg.getType()),
                                          msg.getAccessMode(), msg.getSize(), msg.getPrefetch());
}

void Profiler::addAddressTraceSample(const CacheMsg& msg, NodeID id)
{
  if (msg.getType() != CacheRequestType_IFETCH) {

    // Note: The following line should be commented out if you want to
    // use the special profiling that is part of the GS320 protocol

    // NOTE: Unless PROFILE_HOT_LINES or PROFILE_ALL_INSTRUCTIONS are enabled, nothing will be profiled by the AddressProfiler
    m_address_profiler_ptr->addTraceSample(msg.getAddress(), msg.getProgramCounter(), msg.getType(), msg.getAccessMode(), id, false);
  }
}

void Profiler::profileSharing(const Address& addr, AccessType type, NodeID requestor, const Set& sharers, const Set& owner)
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

void Profiler::profileMsgDelay(int virtualNetwork, int delayCycles) {
  assert(virtualNetwork < m_delayedCyclesVCHistograms.size());
  m_delayedCyclesHistogram.add(delayCycles);
  m_delayedCyclesVCHistograms[virtualNetwork].add(delayCycles);
  if (virtualNetwork != 0) {
    m_delayedCyclesNonPFHistogram.add(delayCycles);
  }
}

// profiles original cache requests including PUTs
void Profiler::profileRequest(const string& requestStr)
{
  m_requests++;

  if (m_requestProfileMap_ptr->exist(requestStr)) {
    (m_requestProfileMap_ptr->lookup(requestStr))++;
  } else {
    m_requestProfileMap_ptr->add(requestStr, 1);
  }
}

void Profiler::recordPrediction(bool wasGood, bool wasPredicted)
{
  m_predictionOpportunities++;
  if(wasPredicted){
    m_predictions++;
    if(wasGood){
      m_goodPredictions++;
    }
  }
}

void Profiler::profileFilterAction(int action)
{
  m_filter_action_histogram.add(action);
}

void Profiler::profileMulticastRetry(const Address& addr, int count)
{
  m_multicast_retry_histogram.add(count);
}

void Profiler::startTransaction(int cpu)
{
  m_perProcStartTransaction[cpu]++;
}

void Profiler::endTransaction(int cpu)
{
  m_perProcEndTransaction[cpu]++;
}

void Profiler::controllerBusy(MachineID machID)
{
  m_busyControllerCount[(int)machID.type][(int)machID.num]++;
}

void Profiler::profilePFWait(Time waitTime)
{
  m_prefetchWaitHistogram.add(waitTime);
}

void Profiler::bankBusy()
{
  m_busyBankCount++;
}

// non-zero cycle demand request
void Profiler::missLatency(Time t, CacheRequestType type, GenericMachineType respondingMach)
{
  m_allMissLatencyHistogram.add(t);
  m_missLatencyHistograms[type].add(t);
  m_machLatencyHistograms[respondingMach].add(t);
  if(respondingMach == GenericMachineType_Directory || respondingMach == GenericMachineType_NUM) {
    m_L2MissLatencyHistogram.add(t);
  }
}

// non-zero cycle prefetch request
void Profiler::swPrefetchLatency(Time t, CacheRequestType type, GenericMachineType respondingMach)
{
  m_allSWPrefetchLatencyHistogram.add(t);
  m_SWPrefetchLatencyHistograms[type].add(t);
  m_SWPrefetchMachLatencyHistograms[respondingMach].add(t);
  if(respondingMach == GenericMachineType_Directory || respondingMach == GenericMachineType_NUM) {
    m_SWPrefetchL2MissLatencyHistogram.add(t);
  }
}

void Profiler::profileTransition(const string& component, NodeID id, NodeID version, Address addr,
                                 const string& state, const string& event,
                                 const string& next_state, const string& note)
{
  const int EVENT_SPACES = 20;
  const int ID_SPACES = 3;
  const int TIME_SPACES = 7;
  const int COMP_SPACES = 10;
  const int STATE_SPACES = 6;

  if ((g_debug_ptr->getDebugTime() > 0) &&
      (g_eventQueue_ptr->getTime() >= g_debug_ptr->getDebugTime())) {
    (* debug_cout_ptr).flags(ios::right);
    (* debug_cout_ptr) << setw(TIME_SPACES) << g_eventQueue_ptr->getTime() << " ";
    (* debug_cout_ptr) << setw(ID_SPACES) << id << " ";
    (* debug_cout_ptr) << setw(ID_SPACES) << version << " ";
    (* debug_cout_ptr) << setw(COMP_SPACES) << component;
    (* debug_cout_ptr) << setw(EVENT_SPACES) << event << " ";
    for (int i=0; i < RubyConfig::numberOfProcessors(); i++) {

      if (i == id) {
        (* debug_cout_ptr).flags(ios::right);
        (* debug_cout_ptr) << setw(STATE_SPACES) << state;
        (* debug_cout_ptr) << ">";
        (* debug_cout_ptr).flags(ios::left);
        (* debug_cout_ptr) << setw(STATE_SPACES) << next_state;
      } else {
        // cout << setw(STATE_SPACES) << " " << " " << setw(STATE_SPACES) << " ";
      }
    }
    (* debug_cout_ptr) << " " << addr << " " << note;

    (* debug_cout_ptr) << endl;
  }
}

// Helper function
static double process_memory_total()
{
  const double MULTIPLIER = 4096.0/(1024.0*1024.0); // 4kB page size, 1024*1024 bytes per MB,
  ifstream proc_file;
  proc_file.open("/proc/self/statm");
  int total_size_in_pages = 0;
  int res_size_in_pages = 0;
  proc_file >> total_size_in_pages;
  proc_file >> res_size_in_pages;
  return double(total_size_in_pages)*MULTIPLIER; // size in megabytes
}

static double process_memory_resident()
{
  const double MULTIPLIER = 4096.0/(1024.0*1024.0); // 4kB page size, 1024*1024 bytes per MB,
  ifstream proc_file;
  proc_file.open("/proc/self/statm");
  int total_size_in_pages = 0;
  int res_size_in_pages = 0;
  proc_file >> total_size_in_pages;
  proc_file >> res_size_in_pages;
  return double(res_size_in_pages)*MULTIPLIER; // size in megabytes
}

void Profiler::profileGetXMaskPrediction(const Set& pred_set)
{
  m_getx_mask_prediction.add(pred_set.count());
}

void Profiler::profileGetSMaskPrediction(const Set& pred_set)
{
  m_gets_mask_prediction.add(pred_set.count());
}

void Profiler::profileTrainingMask(const Set& pred_set)
{
  m_explicit_training_mask.add(pred_set.count());
}

int64 Profiler::getTotalInstructionsExecuted() const
{
  int64 sum = 1;     // Starting at 1 allows us to avoid division by zero
  for(int i=0; i < RubyConfig::numberOfProcessors(); i++) {
    sum += (g_system_ptr->getDriver()->getInstructionCount(i) - m_instructions_executed_at_start[i]);
  }
  return sum;
}

int64 Profiler::getTotalTransactionsExecuted() const
{
  int64 sum = m_perProcEndTransaction.sum();
  if (sum > 0) {
    return sum;
  } else {
    return 1;  // Avoid division by zero errors
  }
}


// The following case statement converts CacheRequestTypes to GenericRequestTypes
// allowing all profiling to be done with a single enum type instead of slow strings
GenericRequestType Profiler::CacheRequestType_to_GenericRequestType(const CacheRequestType& type) {
  switch (type) {
  case CacheRequestType_LD:
    return GenericRequestType_LD;
    break;
  case CacheRequestType_ST:
    return GenericRequestType_ST;
    break;
  case CacheRequestType_ATOMIC:
    return GenericRequestType_ATOMIC;
    break;
  case CacheRequestType_IFETCH:
    return GenericRequestType_IFETCH;
    break;
  case CacheRequestType_LD_XACT:
    return GenericRequestType_LD_XACT;
    break;
  case CacheRequestType_LDX_XACT:
    return GenericRequestType_LDX_XACT;
    break;
  case CacheRequestType_ST_XACT:
    return GenericRequestType_ST_XACT;
    break;
  case CacheRequestType_NULL:
    return GenericRequestType_NULL;
    break;
  default:
    ERROR_MSG("Unexpected cache request type");
  }
}

//---- begin Transactional Memory CODE
void Profiler::profileTransaction(int size, int logSize, int readS, int writeS, int overflow_readS, int overflow_writeS, int retries, int useful_cycles, bool nacked, int loadMisses, int storeMisses, int instrCount, int xid){
  m_xactLogs.add(logSize);
  m_xactSizes.add(size);
  m_xactReads.add(readS);
  m_xactWrites.add(writeS);
  m_xactRetries.add(retries);
  m_xactCycles.add(useful_cycles);
  m_xactLoadMisses.add(loadMisses);
  m_xactStoreMisses.add(storeMisses);
  m_xactInstrCount.add(instrCount);

  // was this transaction nacked?
  if(nacked){
    m_xactNacked++;
  }

  // for overflowed transactions
  if(overflow_readS > 0 || overflow_writeS > 0){
    m_xactOverflowReads.add(overflow_readS);
    m_xactOverflowWrites.add(overflow_writeS);
    m_xactOverflowTotalReads.add(readS);
    m_xactOverflowTotalWrites.add(writeS);
  }

  // Record commits by xid
  if(!m_commitIDMap_ptr->exist(xid)){
    m_commitIDMap_ptr->add(xid, 1);
    m_xactRetryIDMap_ptr->add(xid, retries);
    m_xactCyclesIDMap_ptr->add(xid, useful_cycles);
    m_xactReadSetIDMap_ptr->add(xid, readS);
    m_xactWriteSetIDMap_ptr->add(xid, writeS);
    m_xactLoadMissIDMap_ptr->add(xid, loadMisses);
    m_xactStoreMissIDMap_ptr->add(xid, storeMisses);
    m_xactInstrCountIDMap_ptr->add(xid, instrCount);
  } else {
    (m_commitIDMap_ptr->lookup(xid))++;
    (m_xactRetryIDMap_ptr->lookup(xid)) += retries;
    (m_xactCyclesIDMap_ptr->lookup(xid)) += useful_cycles;
    (m_xactReadSetIDMap_ptr->lookup(xid)) += readS;
    (m_xactWriteSetIDMap_ptr->lookup(xid)) += writeS;
    (m_xactLoadMissIDMap_ptr->lookup(xid)) += loadMisses;
    (m_xactStoreMissIDMap_ptr->lookup(xid)) += storeMisses;
    (m_xactInstrCountIDMap_ptr->lookup(xid)) += instrCount;
  }
}

void Profiler::profileBeginTransaction(NodeID id, int tid, int xid, int thread, Address pc, bool isOpen){
  //- if(PROFILE_XACT){
  if(PROFILE_XACT || (ATMTP_DEBUG_LEVEL >= 2)){
    const char* openStr = isOpen ? " OPEN" : " CLOSED";
    const int ID_SPACES = 3;
    const int TIME_SPACES = 7;
    physical_address_t myPhysPC = SIMICS_translate_address(id, pc);
    integer_t myInst = SIMICS_read_physical_memory(id, myPhysPC, 4);
    const char *myInstStr = SIMICS_disassemble_physical(id, myPhysPC);
    // The actual processor number
    int proc_no = id*RubyConfig::numberofSMTThreads() + thread;
    (* debug_cout_ptr).flags(ios::right);
    (* debug_cout_ptr) << setw(TIME_SPACES) << g_eventQueue_ptr->getTime() << " ";
    (* debug_cout_ptr) << setw(ID_SPACES) << proc_no << " [" << id << "," << thread << "]" <<  " TID " << tid
                       << " XACT BEGIN " << xid
                       << "  PC 0x" << hex << pc.getAddress()
                       << dec
                       << "  *PC 0x" << hex << myInst << dec
                       << " '" << myInstStr << "'"
                       << openStr
                       << endl;
  }
}

void Profiler::profileCommitTransaction(NodeID id, int tid, int xid, int thread, Address pc, bool isOpen){
  //- if(PROFILE_XACT){
  if(PROFILE_XACT || (ATMTP_DEBUG_LEVEL >= 2)){
    const char* openStr = isOpen ? " OPEN" : " CLOSED";
    const int ID_SPACES = 3;
    const int TIME_SPACES = 7;
    physical_address_t myPhysPC = SIMICS_translate_address(id, pc);
    integer_t myInst = SIMICS_read_physical_memory(id, myPhysPC, 4);
    const char *myInstStr = SIMICS_disassemble_physical(id, myPhysPC);
    // The actual processor number
    int proc_no = id*RubyConfig::numberofSMTThreads() + thread;
    (* debug_cout_ptr).flags(ios::right);
    (* debug_cout_ptr) << setw(TIME_SPACES) << g_eventQueue_ptr->getTime() << " ";
    (* debug_cout_ptr) << setw(ID_SPACES) << proc_no << " [" << id << "," << thread << "]" << " TID " << tid
                       << " XACT COMMIT " << xid
                       << "  PC 0x" << hex << pc.getAddress()
                       << dec
                       << "  *PC 0x" << hex << myInst << dec
                       << " '" << myInstStr << "'"
                       << openStr
                       << endl;
  }

}

// for profiling overflows
void Profiler::profileLoadOverflow(NodeID id, int tid, int xid, int thread, Address addr, bool l1_overflow){
  //- if(PROFILE_XACT){
  if(PROFILE_XACT || (ATMTP_DEBUG_LEVEL >= 1)){
    const int ID_SPACES = 3;
    const int TIME_SPACES = 7;
    string overflow_str = " XACT LOAD L1 OVERFLOW ";
    if(!l1_overflow){
      overflow_str = " XACT LOAD L2 OVERFLOW ";
    }
    // The actual processor number
    int proc_no = id*RubyConfig::numberofSMTThreads() + thread;
    (* debug_cout_ptr).flags(ios::right);
    (* debug_cout_ptr) << setw(TIME_SPACES) << g_eventQueue_ptr->getTime() << " ";
    (* debug_cout_ptr) << setw(ID_SPACES)  << proc_no << " [" << id << "," << thread << "]" <<  " TID " << tid
                       << overflow_str << xid
                       << "  ADDR " << addr
                       << endl;
  }
}

// for profiling overflows
void Profiler::profileStoreOverflow(NodeID id, int tid, int xid, int thread, Address addr, bool l1_overflow){
  //- if(PROFILE_XACT){
  if(PROFILE_XACT || (ATMTP_DEBUG_LEVEL >= 1)){
    const int ID_SPACES = 3;
    const int TIME_SPACES = 7;
    string overflow_str = " XACT STORE L1 OVERFLOW ";
    if(!l1_overflow){
      overflow_str = " XACT STORE L2 OVERFLOW ";
    }
    // The actual processor number
    int proc_no = id*RubyConfig::numberofSMTThreads() + thread;
    (* debug_cout_ptr).flags(ios::right);
    (* debug_cout_ptr) << setw(TIME_SPACES) << g_eventQueue_ptr->getTime() << " ";
    (* debug_cout_ptr) << setw(ID_SPACES)  << proc_no << " [" << id << "," << thread << "]" <<  " TID " << tid
                       << overflow_str << xid
                       << "  ADDR " << addr
                       << endl;
  }
}

void Profiler::profileLoadTransaction(NodeID id, int tid, int xid, int thread, Address addr, Address logicalAddress, Address pc){
  //- if(PROFILE_XACT){
  if(PROFILE_XACT || (ATMTP_DEBUG_LEVEL >= 3)){
    const int ID_SPACES = 3;
    const int TIME_SPACES = 7;
    physical_address_t myPhysPC = SIMICS_translate_address(id, pc);
    integer_t myInst = SIMICS_read_physical_memory(id, myPhysPC, 4);
    const char *myInstStr = SIMICS_disassemble_physical(id, myPhysPC);
    // The actual processor number
    int proc_no = id*RubyConfig::numberofSMTThreads() + thread;
    (* debug_cout_ptr).flags(ios::right);
    (* debug_cout_ptr) << setw(TIME_SPACES) << g_eventQueue_ptr->getTime() << " ";
    (* debug_cout_ptr) << setw(ID_SPACES)  << proc_no << " [" << id << "," << thread << "]" <<  " TID " << tid
                       << " XACT LOAD " << xid
                       << " " << addr
                       << " VA " << logicalAddress
                       << " PC " << pc
                       << "  *PC 0x" << hex << myInst << dec
                       << " '" << myInstStr << "'"
      //<< " VAL 0x" << hex << SIMICS_read_physical_memory(proc_no, SIMICS_translate_data_address(proc_no, logicalAddress), 4) << dec
                       << " VAL 0x" << hex << g_system_ptr->getDriver()->readPhysicalMemory(proc_no, addr.getAddress(), 4) << dec
                       << endl;
  }
}

void Profiler::profileLoad(NodeID id, int tid, int xid, int thread, Address addr, Address logicalAddress, Address pc){
  if(PROFILE_NONXACT){
    const int ID_SPACES = 3;
    const int TIME_SPACES = 7;
    // The actual processor number
    int proc_no = id*RubyConfig::numberofSMTThreads() + thread;
    (* debug_cout_ptr).flags(ios::right);
    (* debug_cout_ptr) << setw(TIME_SPACES) << g_eventQueue_ptr->getTime() << " ";
    (* debug_cout_ptr) << setw(ID_SPACES) << proc_no << " [" << id << "," << thread << "]" <<  " TID " << tid
                       << " LOAD " << xid
                       << " " << addr
                       << " VA " << logicalAddress
                       << " PC " << pc
      //<< " VAL 0x" << hex << SIMICS_read_physical_memory(proc_no, SIMICS_translate_data_address(proc_no, logicalAddress), 4) << dec
                       << " VAL 0x" << hex << g_system_ptr->getDriver()->readPhysicalMemory(proc_no, addr.getAddress(), 4) << dec
                       << endl;
  }
}

void Profiler::profileStoreTransaction(NodeID id, int tid, int xid, int thread, Address addr, Address logicalAddress, Address pc){
  //- if(PROFILE_XACT){
  if(PROFILE_XACT || (ATMTP_DEBUG_LEVEL >= 3)){
    const int ID_SPACES = 3;
    const int TIME_SPACES = 7;
    physical_address_t myPhysPC = SIMICS_translate_address(id, pc);
    integer_t myInst = SIMICS_read_physical_memory(id, myPhysPC, 4);
    const char *myInstStr = SIMICS_disassemble_physical(id, myPhysPC);
    // The actual processor number
    int proc_no = id*RubyConfig::numberofSMTThreads() + thread;
    (* debug_cout_ptr).flags(ios::right);
    (* debug_cout_ptr) << setw(TIME_SPACES) << g_eventQueue_ptr->getTime() << " ";
    (* debug_cout_ptr) << setw(ID_SPACES) << proc_no << " [" << id << "," << thread << "]" <<  " TID " << tid
                       << " XACT STORE " << xid
                       << " " << addr
                       << " VA " << logicalAddress
                       << " PC " << pc
                       << "  *PC 0x" << hex << myInst << dec
                       << " '" << myInstStr << "'"
                       << endl;
  }
}

void Profiler::profileStore(NodeID id, int tid, int xid, int thread, Address addr, Address logicalAddress, Address pc){
  if(PROFILE_NONXACT){
    const int ID_SPACES = 3;
    const int TIME_SPACES = 7;
    // The actual processor number
    int proc_no = id*RubyConfig::numberofSMTThreads() + thread;
    (* debug_cout_ptr).flags(ios::right);
    (* debug_cout_ptr) << setw(TIME_SPACES) << g_eventQueue_ptr->getTime() << " ";
    (* debug_cout_ptr) << setw(ID_SPACES) << proc_no << " [" << id << "," << thread << "]" <<  " TID " << tid
                       << " STORE " << xid
                       << " " << addr
                       << " VA " << logicalAddress
                       << " PC " << pc
                       << endl;
  }
}

void Profiler::profileNack(NodeID id, int tid, int xid, int thread, int nacking_thread, NodeID nackedBy, Address addr, Address logicalAddress, Address pc, uint64 seq_ts, uint64 nack_ts, bool possibleCycle){
  int nid = 0; // g_system_ptr->getChip(nackedBy/RubyConfig::numberOfProcsPerChip())->getTransactionInterfaceManager(nackedBy%RubyConfig::numberOfProcsPerChip())->getXID(nacking_thread);
  assert(0);
  //- if(PROFILE_XACT){
  if(PROFILE_XACT || (ATMTP_DEBUG_LEVEL >= 1)){
    const int ID_SPACES = 3;
    const int TIME_SPACES = 7;
    physical_address_t myPhysPC = SIMICS_translate_address(id, pc);
    integer_t myInst = SIMICS_read_physical_memory(id, myPhysPC, 4);
    const char *myInstStr = SIMICS_disassemble_physical(id, myPhysPC);
    // The actual processor number
    int proc_no = id*g_NUM_SMT_THREADS + thread;
    int nack_proc_no = nackedBy*g_NUM_SMT_THREADS + nacking_thread;
    Address nack_pc = SIMICS_get_program_counter(nack_proc_no);
    (* debug_cout_ptr).flags(ios::right);
    (* debug_cout_ptr) << setw(TIME_SPACES) << g_eventQueue_ptr->getTime() << " ";
    (* debug_cout_ptr) << setw(ID_SPACES) << proc_no << " [" << id << "," << thread << "]" <<  " TID " << tid
                       << " XACT NACK " << xid
                       << " by " << nack_proc_no
                       << " [ " << nackedBy
                       << ", " << nacking_thread
                       << " ]"
                       << " NID: " << nid
                       << " " << addr
                       << " VA " << logicalAddress
                       << "  PC " << pc
                       << "  *PC 0x" << hex << myInst << dec
                       << " '" << myInstStr << "'"
                       << " NackerPC " << nack_pc
                       << "  my_ts " << seq_ts
                       << "  nack_ts " << nack_ts
                       << " possible_cycle " << possibleCycle
                       << endl;
  }

  // Record nacks by xid
  if(!m_nackXIDMap_ptr->exist(xid)){
    m_nackXIDMap_ptr->add(xid, 1);
  } else {
    (m_nackXIDMap_ptr->lookup(xid))++;
  }

  // Record nack ID pairs by xid
  if(!m_nackXIDPairMap_ptr->exist(xid)){
    Map<int, int> * new_map = new Map<int, int>;
    new_map->add(nid, 1);
    m_nackXIDPairMap_ptr->add(xid, new_map);
  }
  else{
    // retrieve existing map
    Map<int, int> * my_map = m_nackXIDPairMap_ptr->lookup(xid);
    if(!my_map->exist(nid)){
      my_map->add(nid, 1);
    }
    else{
      (my_map->lookup(nid))++;
    }
  }

  // Record nacks by pc
  if(!m_nackPCMap_ptr->exist(pc)){
    m_nackPCMap_ptr->add(pc, 1);
  } else {
    (m_nackPCMap_ptr->lookup(pc))++;
  }
}

void Profiler::profileExposedConflict(NodeID id, int xid, int thread, Address addr, Address pc){
  //if(PROFILE_XACT){
    const int ID_SPACES = 3;
    const int TIME_SPACES = 7;
    // The actual processor number
    int proc_no = id*g_NUM_SMT_THREADS + thread;
    (* debug_cout_ptr).flags(ios::right);
    (* debug_cout_ptr) << setw(TIME_SPACES) << g_eventQueue_ptr->getTime() << " ";
    (* debug_cout_ptr) << setw(ID_SPACES)  << proc_no << " [" << id << "," << thread << "]" << " "
                       << " EXPOSED ACTION CONFLICT " << xid
                       << "  ADDR " << addr
                       << "  PC " << pc
                       << endl;
    //}
}

void Profiler::profileInferredAbort(){
  m_inferredAborts++;
}

void Profiler::profileAbortDelayConstants(int startupDelay, int perBlock){
  m_abortStarupDelay = startupDelay;
  m_abortPerBlockDelay = perBlock;
}

void Profiler::profileAbortTransaction(NodeID id, int tid, int xid, int thread, int delay, int abortingThread, int abortingProc, Address addr, Address pc){
  const int ID_SPACES = 3;
  const int TIME_SPACES = 7;
  int abortingXID = -1;
  // The actual processor number
  int proc_no = id*g_NUM_SMT_THREADS + thread;
  // we are passed in physical proc number. Compute logical abort proc_no
  int logical_abort_proc_no = abortingProc/g_NUM_SMT_THREADS;
  if(abortingProc >= 0){
    AbstractChip * c = g_system_ptr->getChip(logical_abort_proc_no/RubyConfig::numberOfProcsPerChip());
    abortingXID = 0; // c->getTransactionInterfaceManager(logical_abort_proc_no%RubyConfig::numberOfProcsPerChip())->getXID(abortingThread);
    assert(0);
  }
  //- if(PROFILE_XACT){
  if(PROFILE_XACT || (ATMTP_DEBUG_LEVEL >= 1)){
    physical_address_t myPhysPC = SIMICS_translate_address(id, pc);
    integer_t myInst = SIMICS_read_physical_memory(id, myPhysPC, 4);
    const char *myInstStr = SIMICS_disassemble_physical(id, myPhysPC);
    (* debug_cout_ptr).flags(ios::right);
    (* debug_cout_ptr) << setw(TIME_SPACES) << g_eventQueue_ptr->getTime() << " ";
    (* debug_cout_ptr) << setw(ID_SPACES) << proc_no << " [" << id << "," << thread << "]" <<  " TID " << tid
                       << " XACT ABORT " << xid
                       << " caused by " << abortingProc
                       << " [ " << logical_abort_proc_no
                       << ", " << abortingThread
                       << " ]"
                       << " xid: " << abortingXID << " "
                       << " address: " << addr
                       << " delay: " << delay
                       << "  PC " << pc
                       << "  *PC 0x" << hex << myInst << dec
                       << " '" << myInstStr << "'"
                       << endl;
  }
  m_transactionAborts++;

  // Record aborts by xid
  if(!m_abortIDMap_ptr->exist(xid)){
    m_abortIDMap_ptr->add(xid, 1);
  } else {
    (m_abortIDMap_ptr->lookup(xid))++;
  }
  m_abortDelays.add(delay);

  // Record aborts by pc
  if(!m_abortPCMap_ptr->exist(pc)){
    m_abortPCMap_ptr->add(pc, 1);
  } else {
    (m_abortPCMap_ptr->lookup(pc))++;
  }

  // Record aborts by address
  if(!m_abortAddressMap_ptr->exist(addr)){
    m_abortAddressMap_ptr->add(addr, 1);
  } else {
    (m_abortAddressMap_ptr->lookup(addr))++;
  }
}

void Profiler::profileTransWB(){
  m_transWBs++;
}

void Profiler::profileExtraWB(){
  m_extraWBs++;
}

void Profiler::profileXactChange(int procs, int cycles){
  if(!m_procsInXactMap_ptr->exist(procs)){
    m_procsInXactMap_ptr->add(procs, cycles);
  } else {
    (m_procsInXactMap_ptr->lookup(procs)) += cycles;
  }
}

void Profiler::profileReadSet(Address addr, bool bf_filter_result, bool perfect_filter_result, NodeID id, int thread){
  // do NOT count instances when signature is empty!
  if(!bf_filter_result && !perfect_filter_result){
    m_readSetEmptyChecks++;
    return;
  }

  if(bf_filter_result != perfect_filter_result){
    m_readSetNoMatch++;
    /*
    // we have a false positive
    if(!m_readSetNoMatch_ptr->exist(addr)){
      m_readSetNoMatch_ptr->add(addr, 1);
    }
    else{
      (m_readSetNoMatch_ptr->lookup(addr))++;
    }
    */
  }
  else{
    m_readSetMatch++;
    /*
    // Bloom filter agrees with perfect filter
    if(!m_readSetMatch_ptr->exist(addr)){
      m_readSetMatch_ptr->add(addr, 1);
    }
    else{
      (m_readSetMatch_ptr->lookup(addr))++;
    }
    */
  }
}


void Profiler::profileRemoteReadSet(Address addr, bool bf_filter_result, bool perfect_filter_result, NodeID id, int thread){
  if(bf_filter_result != perfect_filter_result){
    // we have a false positive
    if(!m_remoteReadSetNoMatch_ptr->exist(addr)){
      m_remoteReadSetNoMatch_ptr->add(addr, 1);
    }
    else{
      (m_remoteReadSetNoMatch_ptr->lookup(addr))++;
    }
  }
  else{
    // Bloom filter agrees with perfect filter
    if(!m_remoteReadSetMatch_ptr->exist(addr)){
      m_remoteReadSetMatch_ptr->add(addr, 1);
    }
    else{
      (m_remoteReadSetMatch_ptr->lookup(addr))++;
    }
  }
}

void Profiler::profileWriteSet(Address addr, bool bf_filter_result, bool perfect_filter_result, NodeID id, int thread){
  // do NOT count instances when signature is empty!
  if(!bf_filter_result && !perfect_filter_result){
    m_writeSetEmptyChecks++;
    return;
  }

  if(bf_filter_result != perfect_filter_result){
    m_writeSetNoMatch++;
    /*
    // we have a false positive
    if(!m_writeSetNoMatch_ptr->exist(addr)){
      m_writeSetNoMatch_ptr->add(addr, 1);
    }
    else{
      (m_writeSetNoMatch_ptr->lookup(addr))++;
    }
    */
  }
  else{
    m_writeSetMatch++;
    /*
    // Bloom filter agrees with perfect filter
    if(!m_writeSetMatch_ptr->exist(addr)){
      m_writeSetMatch_ptr->add(addr, 1);
    }
    else{
      (m_writeSetMatch_ptr->lookup(addr))++;
    }
    */
  }
}


void Profiler::profileRemoteWriteSet(Address addr, bool bf_filter_result, bool perfect_filter_result, NodeID id, int thread){
  if(bf_filter_result != perfect_filter_result){
    // we have a false positive
    if(!m_remoteWriteSetNoMatch_ptr->exist(addr)){
      m_remoteWriteSetNoMatch_ptr->add(addr, 1);
    }
    else{
      (m_remoteWriteSetNoMatch_ptr->lookup(addr))++;
    }
  }
  else{
    // Bloom filter agrees with perfect filter
    if(!m_remoteWriteSetMatch_ptr->exist(addr)){
      m_remoteWriteSetMatch_ptr->add(addr, 1);
    }
    else{
      (m_remoteWriteSetMatch_ptr->lookup(addr))++;
    }
  }
}

void Profiler::profileTransactionLogOverflow(NodeID id, Address addr, Address pc){
  if(PROFILE_XACT || (ATMTP_DEBUG_LEVEL >= 1)){
    const int ID_SPACES = 3;
    const int TIME_SPACES = 7;
    physical_address_t myPhysPC = SIMICS_translate_address(id, pc);
    integer_t myInst = SIMICS_read_physical_memory(id, myPhysPC, 4);
    const char *myInstStr = SIMICS_disassemble_physical(id, myPhysPC);
    (* debug_cout_ptr).flags(ios::right);
    (* debug_cout_ptr) << setw(TIME_SPACES) << g_eventQueue_ptr->getTime() << " ";
    (* debug_cout_ptr) << setw(ID_SPACES) << id << " "
                       << " XACT LOG OVERFLOW"
                       << "  ADDR " << addr
                       << "  PC " << pc
                       << "  *PC 0x" << hex << myInst << dec
                       << " '" << myInstStr << "'"
                       << endl;

  }
  m_transactionLogOverflows++;
}

void Profiler::profileTransactionCacheOverflow(NodeID id, Address addr, Address pc){
  if(PROFILE_XACT || (ATMTP_DEBUG_LEVEL >= 1)){
    const int ID_SPACES = 3;
    const int TIME_SPACES = 7;
    physical_address_t myPhysPC = SIMICS_translate_address(id, pc);
    integer_t myInst = SIMICS_read_physical_memory(id, myPhysPC, 4);
    const char *myInstStr = SIMICS_disassemble_physical(id, myPhysPC);
    (* debug_cout_ptr).flags(ios::right);
    (* debug_cout_ptr) << setw(TIME_SPACES) << g_eventQueue_ptr->getTime() << " ";
    (* debug_cout_ptr) << setw(ID_SPACES) << id << " "
                       << " XACT CACHE OVERFLOW "
                       << "  ADDR " << addr
                       << "  PC " << pc
                       << "  *PC 0x" << hex << myInst << dec
                       << " '" << myInstStr << "'"
                       << endl;

  }
  m_transactionCacheOverflows++;
}

void Profiler::profileGetCPS(NodeID id, uint32 cps, Address pc){
  if(PROFILE_XACT || (ATMTP_DEBUG_LEVEL >= 1)){
    const int ID_SPACES = 3;
    const int TIME_SPACES = 7;
    physical_address_t myPhysPC = SIMICS_translate_address(id, pc);
    integer_t myInst = SIMICS_read_physical_memory(id, myPhysPC, 4);
    const char *myInstStr = SIMICS_disassemble_physical(id, myPhysPC);

    (* debug_cout_ptr).flags(ios::right);
    (* debug_cout_ptr) << setw(TIME_SPACES) << g_eventQueue_ptr->getTime() << " ";
    (* debug_cout_ptr) << setw(ID_SPACES) << id << " "
                       << " XACT GET CPS"
                       << "  PC " << pc
                       << "  *PC 0x" << hex << myInst << dec
                       << " '" << myInstStr << "'"
                       << "  CPS 0x" << hex << cps << dec
                       << endl;
  }
}
//---- end Transactional Memory CODE


void Profiler::profileExceptionStart(bool xact, NodeID id, int thread, int val, int trap_level, uinteger_t pc, uinteger_t npc){
  if(xact){
    if(!m_xactExceptionMap_ptr->exist(val)){
      m_xactExceptionMap_ptr->add(val, 1);
    } else {
      (m_xactExceptionMap_ptr->lookup(val))++;
    }
  }

  if (!xact && !PROFILE_NONXACT) return;

  if(PROFILE_EXCEPTIONS){
    const int ID_SPACES = 3;
    const int TIME_SPACES = 7;
    // The actual processor number
    int proc_no = id*g_NUM_SMT_THREADS + thread;

    // get the excepting instruction
    const char * instruction;
    physical_address_t addr = SIMICS_translate_address( proc_no, Address(pc));
    if(val != 0x64 && addr != 0x0){
      // ignore instruction TLB miss
      instruction = SIMICS_disassemble_physical( proc_no, addr );
    }

    (* debug_cout_ptr).flags(ios::right);
    (* debug_cout_ptr) << setw(TIME_SPACES) << g_eventQueue_ptr->getTime() << " ";
    (* debug_cout_ptr) << setw(ID_SPACES) << proc_no << " [" << id << "," << thread << " ]" << " ";
    if (xact)
      (* debug_cout_ptr) << " XACT Exception(";
    else
      (* debug_cout_ptr) << "      Exception(";

    (* debug_cout_ptr)  << hex << val << dec << ")_START--Trap Level " << trap_level
                        << "--(PC=0x" << hex << pc << ", " << npc << ")"
                        << dec;

    if(val != 0x64 && addr != 0x0){
      (* debug_cout_ptr) << " instruction = " << instruction;
    }
    else{
      (* debug_cout_ptr) << " instruction = INSTRUCTION TLB MISS";
    }
    (* debug_cout_ptr)  << dec << endl;
  }
}

void Profiler::profileExceptionDone(bool xact, NodeID id, int thread, int val, int trap_level, uinteger_t pc, uinteger_t npc, uinteger_t tpc, uinteger_t tnpc){
  if (!xact && !PROFILE_NONXACT) return;

  if (PROFILE_EXCEPTIONS){
    const int ID_SPACES = 3;
    const int TIME_SPACES = 7;
    // The actual processor number
    int proc_no = id*g_NUM_SMT_THREADS + thread;

    // get the excepting instruction
    const char * instruction;
    instruction = SIMICS_disassemble_physical( proc_no, SIMICS_translate_address( proc_no, Address(pc) ) );


    (* debug_cout_ptr).flags(ios::right);
    (* debug_cout_ptr) << setw(TIME_SPACES) << g_eventQueue_ptr->getTime() << " ";
    (* debug_cout_ptr) << setw(ID_SPACES) << proc_no << " [" << id << "," << thread << " ]" << " ";
    if (xact)
      (* debug_cout_ptr) << " XACT Exception(";
    else
      (* debug_cout_ptr) << "      Exception(";

    (* debug_cout_ptr) << hex << val << dec << ")_DONE--Trap Level " << trap_level
                       << "--(PC=0x" << hex << pc << ", " << npc << dec << ")"
                       << "--(TPC=0x" << hex << tpc << ", " << tnpc << dec << ")"
                       << endl;
  }
}

void Profiler::rubyWatch(int id){
    int rn_g1 = SIMICS_get_register_number(id, "g1");
    uint64 tr = SIMICS_read_register(id, rn_g1);
    Address watch_address = Address(tr);
    const int ID_SPACES = 3;
    const int TIME_SPACES = 7;

    (* debug_cout_ptr).flags(ios::right);
    (* debug_cout_ptr) << setw(TIME_SPACES) << g_eventQueue_ptr->getTime() << " ";
    (* debug_cout_ptr) << setw(ID_SPACES) << id << " "
                       << "RUBY WATCH "
                       << watch_address
                       << endl;

    if(!m_watch_address_list_ptr->exist(watch_address)){
      m_watch_address_list_ptr->add(watch_address, 1);
    }
}

bool Profiler::watchAddress(Address addr){
    if (m_watch_address_list_ptr->exist(addr))
      return true;
    else
      return false;
}

void Profiler::profileReadFilterBitsSet(int xid, int bits, bool isCommit) {
  if (isCommit) {
    if(!m_xactReadFilterBitsSetOnCommit->exist(xid)){
      Histogram hist;
      hist.add(bits);
      m_xactReadFilterBitsSetOnCommit->add(xid, hist);
    }
    else{
      (m_xactReadFilterBitsSetOnCommit->lookup(xid)).add(bits);
    }
  } else {
    if(!m_xactReadFilterBitsSetOnAbort->exist(xid)){
      Histogram hist;
      hist.add(bits);
      m_xactReadFilterBitsSetOnAbort->add(xid, hist);
    }
    else{
      (m_xactReadFilterBitsSetOnAbort->lookup(xid)).add(bits);
    }
  }
}

void Profiler::profileWriteFilterBitsSet(int xid, int bits, bool isCommit) {
  if (isCommit) {
    if(!m_xactWriteFilterBitsSetOnCommit->exist(xid)){
      Histogram hist;
      hist.add(bits);
      m_xactWriteFilterBitsSetOnCommit->add(xid, hist);
    }
    else{
      (m_xactWriteFilterBitsSetOnCommit->lookup(xid)).add(bits);
    }
  } else {
    if(!m_xactWriteFilterBitsSetOnAbort->exist(xid)){
      Histogram hist;
      hist.add(bits);
      m_xactWriteFilterBitsSetOnAbort->add(xid, hist);
    }
    else{
      (m_xactWriteFilterBitsSetOnAbort->lookup(xid)).add(bits);
    }
  }
}
/*
                        //gem5:Arka for decomissioning log_tm

void Profiler::setXactVisualizerFile(char * filename){
  if ( (filename == NULL) ||
       (!strcmp(filename, "none")) ) {
    m_xact_visualizer_ptr = &cout;
    return;
  }

  if (m_xact_visualizer.is_open() ) {
    m_xact_visualizer.close ();
  }
  m_xact_visualizer.open (filename, std::ios::out);
  if (! m_xact_visualizer.is_open() ) {
    cerr << "setXactVisualizer: can't open file " << filename << endl;
  }
  else {
    m_xact_visualizer_ptr = &m_xact_visualizer;
  }
  cout << "setXactVisualizer file " << filename << endl;
}

void Profiler::printTransactionState(bool can_skip){
  if (!XACT_VISUALIZER) return;
  int num_processors = RubyConfig::numberOfProcessors() * RubyConfig::numberofSMTThreads();

  if (!g_system_ptr->getXactVisualizer()->existXactActivity() && can_skip)
    return;

  if (can_skip && ((g_eventQueue_ptr->getTime()/10000) <= m_xact_visualizer_last))
    return;

  Vector<char> xactStateVector = g_system_ptr->getXactVisualizer()->getTransactionStateVector();
  for (int i = 0 ; i < num_processors; i++){
    (* m_xact_visualizer_ptr) << xactStateVector[i] << " ";
  }
  (* m_xact_visualizer_ptr) << "   " << g_eventQueue_ptr->getTime() << endl;
  m_xact_visualizer_last = g_eventQueue_ptr->getTime() / 10000;
}
*/
void Profiler::watchpointsFalsePositiveTrigger()
{
  m_watchpointsFalsePositiveTrigger++;
}

void Profiler::watchpointsTrueTrigger()
{
  m_watchpointsTrueTrigger++;
}

// For MemoryControl:
void Profiler::profileMemReq(int bank) {
  m_memReq++;
  m_memBankCount[bank]++;
}
void Profiler::profileMemBankBusy() { m_memBankBusy++; }
void Profiler::profileMemBusBusy() { m_memBusBusy++; }
void Profiler::profileMemReadWriteBusy() { m_memReadWriteBusy++; }
void Profiler::profileMemDataBusBusy() { m_memDataBusBusy++; }
void Profiler::profileMemTfawBusy() { m_memTfawBusy++; }
void Profiler::profileMemRefresh() { m_memRefresh++; }
void Profiler::profileMemRead() { m_memRead++; }
void Profiler::profileMemWrite() { m_memWrite++; }
void Profiler::profileMemWaitCycles(int cycles) { m_memWaitCycles += cycles; }
void Profiler::profileMemInputQ(int cycles) { m_memInputQ += cycles; }
void Profiler::profileMemBankQ(int cycles) { m_memBankQ += cycles; }
void Profiler::profileMemArbWait(int cycles) { m_memArbWait += cycles; }
void Profiler::profileMemRandBusy() { m_memRandBusy++; }
void Profiler::profileMemNotOld() { m_memNotOld++; }


//----------- ATMTP -------------------//

void Profiler::profileTransactionTCC(NodeID id, Address pc){
  if(PROFILE_XACT || (ATMTP_DEBUG_LEVEL >= 1)){
    physical_address_t myPhysPC = SIMICS_translate_address(id, pc);
    integer_t myInst = SIMICS_read_physical_memory(id, myPhysPC, 4);
    const char *myInstStr = SIMICS_disassemble_physical(id, myPhysPC);

    const int ID_SPACES = 3;
    const int TIME_SPACES = 7;
    cout.flags(ios::right);
    cout << setw(TIME_SPACES) << g_eventQueue_ptr->getTime() << " ";
    cout << setw(ID_SPACES) << id << " "
         << " XACT Aborting! Executed TCC "
         << "  PC: " << pc
         << "  *PC: 0x" << hex << myInst << dec
         << " '" << myInstStr << "'"
         << endl;
  }
  m_transactionUnsupInsts++;
}

void Profiler::profileTransactionUnsupInst(NodeID id, Address pc){
  if(PROFILE_XACT || (ATMTP_DEBUG_LEVEL >= 1)){
    physical_address_t myPhysPC = SIMICS_translate_address(id, pc);
    integer_t myInst = SIMICS_read_physical_memory(id, myPhysPC, 4);
    const char *myInstStr = SIMICS_disassemble_physical(id, myPhysPC);

    const int ID_SPACES = 3;
    const int TIME_SPACES = 7;
    cout.flags(ios::right);
    cout << setw(TIME_SPACES) << g_eventQueue_ptr->getTime() << " ";
    cout << setw(ID_SPACES) << id << " "
         << " XACT Aborting! Executed Unsupported Instruction "
         << "  PC: " << pc
         << "  *PC: 0x" << hex << myInst << dec
         << " '" << myInstStr << "'"
         << endl;
  }
  m_transactionUnsupInsts++;
}

void Profiler::profileTransactionSaveInst(NodeID id, Address pc){
  if(PROFILE_XACT || (ATMTP_DEBUG_LEVEL >= 1)){
    physical_address_t myPhysPC = SIMICS_translate_address(id, pc);
    integer_t myInst = SIMICS_read_physical_memory(id, myPhysPC, 4);
    const char *myInstStr = SIMICS_disassemble_physical(id, myPhysPC);

    const int ID_SPACES = 3;
    const int TIME_SPACES = 7;
    cout.flags(ios::right);
    cout << setw(TIME_SPACES) << g_eventQueue_ptr->getTime() << " ";
    cout << setw(ID_SPACES) << id << " "
         << " XACT Aborting! Executed Save Instruction "
         << "  PC: " << pc
         << "  *PC: 0x" << hex << myInst << dec
         << " '" << myInstStr << "'"
         << endl;
  }
  m_transactionSaveRestAborts++;
}

void Profiler::profileTransactionRestoreInst(NodeID id, Address pc){
  if(PROFILE_XACT || (ATMTP_DEBUG_LEVEL >= 1)){
    physical_address_t myPhysPC = SIMICS_translate_address(id, pc);
    integer_t myInst = SIMICS_read_physical_memory(id, myPhysPC, 4);
    const char *myInstStr = SIMICS_disassemble_physical(id, myPhysPC);

    const int ID_SPACES = 3;
    const int TIME_SPACES = 7;
    cout.flags(ios::right);
    cout << setw(TIME_SPACES) << g_eventQueue_ptr->getTime() << " ";
    cout << setw(ID_SPACES) << id << " "
         << " XACT Aborting! Executed Restore Instruction "
         << "  PC: " << pc
         << "  *PC: 0x" << hex << myInst << dec
         << " '" << myInstStr << "'"
         << endl;
  }
  m_transactionSaveRestAborts++;
}

void Profiler::profileTimerInterrupt(NodeID id,
                                     uinteger_t tick, uinteger_t tick_cmpr,
                                     uinteger_t stick, uinteger_t stick_cmpr,
                                     int trap_level,
                                     uinteger_t pc, uinteger_t npc,
                                     uinteger_t pstate, int pil){
  if (PROFILE_EXCEPTIONS) {
    const int ID_SPACES = 3;
    const int TIME_SPACES = 7;
    cout.flags(ios::right);
    cout << setw(TIME_SPACES) << g_eventQueue_ptr->getTime() << " ";
    cout << setw(ID_SPACES) << id << " ";
    cout << hex << "Timer--(Tick=0x" << tick << ", TckCmp=0x" << tick_cmpr
         << ", STick=0x" << stick << ", STickCmp=0x" << stick_cmpr
         << ")--(PC=" << pc << ", " << npc
         << dec << ")--(TL=" << trap_level << ", pil=" << pil
         << hex << ", pstate=0x" << pstate
         << dec << ")" << endl;
  }
}
