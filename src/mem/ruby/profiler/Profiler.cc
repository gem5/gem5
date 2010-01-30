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
 * Profiler.cc
 *
 * Description: See Profiler.hh
 *
 * $Id$
 *
 */

#include "mem/ruby/profiler/Profiler.hh"
#include "mem/ruby/profiler/AddressProfiler.hh"
#include "mem/ruby/system/System.hh"
#include "mem/ruby/network/Network.hh"
#include "mem/gems_common/PrioHeap.hh"
#include "mem/protocol/CacheMsg.hh"
#include "mem/protocol/Protocol.hh"
#include "mem/gems_common/util.hh"
#include "mem/gems_common/Map.hh"
#include "mem/ruby/common/Debug.hh"
#include "mem/protocol/MachineType.hh"

#include "mem/ruby/system/System.hh"

// Allows use of times() library call, which determines virtual runtime
#include <sys/times.h>

extern std::ostream * debug_cout_ptr;

static double process_memory_total();
static double process_memory_resident();

Profiler::Profiler(const Params *p)
    : SimObject(p)
{
  m_requestProfileMap_ptr = new Map<string, int>;

  m_inst_profiler_ptr = NULL;
  m_address_profiler_ptr = NULL;

  m_real_time_start_time = time(NULL); // Not reset in clearStats()
  m_stats_period = 1000000; // Default
  m_periodic_output_file_ptr = &cerr;

  m_hot_lines = p->hot_lines;
  m_all_instructions = p->all_instructions;

  m_num_of_sequencers = p->num_of_sequencers;

  //
  // Initialize the memory controller profiler structs
  //
  m_mc_profilers.setSize(p->mem_cntrl_count);
  for (int mem_cntrl = 0; mem_cntrl < p->mem_cntrl_count; mem_cntrl++) {
    m_mc_profilers[mem_cntrl] = new memory_control_profiler;
    m_mc_profilers[mem_cntrl]->m_memReq = 0;
    m_mc_profilers[mem_cntrl]->m_memBankBusy = 0;
    m_mc_profilers[mem_cntrl]->m_memBusBusy = 0;
    m_mc_profilers[mem_cntrl]->m_memReadWriteBusy = 0;
    m_mc_profilers[mem_cntrl]->m_memDataBusBusy = 0;
    m_mc_profilers[mem_cntrl]->m_memTfawBusy = 0;
    m_mc_profilers[mem_cntrl]->m_memRefresh = 0;
    m_mc_profilers[mem_cntrl]->m_memRead = 0;
    m_mc_profilers[mem_cntrl]->m_memWrite = 0;
    m_mc_profilers[mem_cntrl]->m_memWaitCycles = 0;
    m_mc_profilers[mem_cntrl]->m_memInputQ = 0;
    m_mc_profilers[mem_cntrl]->m_memBankQ = 0;
    m_mc_profilers[mem_cntrl]->m_memArbWait = 0;
    m_mc_profilers[mem_cntrl]->m_memRandBusy = 0;
    m_mc_profilers[mem_cntrl]->m_memNotOld = 0;

    m_mc_profilers[mem_cntrl]->m_banks_per_rank = p->banks_per_rank;
    m_mc_profilers[mem_cntrl]->m_ranks_per_dimm = p->ranks_per_dimm;
    m_mc_profilers[mem_cntrl]->m_dimms_per_channel = 
      p->dimms_per_channel;

    int totalBanks = p->banks_per_rank * 
                     p->ranks_per_dimm * 
                     p->dimms_per_channel;

    m_mc_profilers[mem_cntrl]->m_memBankCount.setSize(totalBanks);
  }    

  m_hot_lines = false;
  m_all_instructions = false;

  m_address_profiler_ptr = new AddressProfiler(m_num_of_sequencers);
  m_address_profiler_ptr -> setHotLines(m_hot_lines);
  m_address_profiler_ptr -> setAllInstructions(m_all_instructions);

  if (m_all_instructions) {
    m_inst_profiler_ptr = new AddressProfiler(m_num_of_sequencers);
    m_inst_profiler_ptr -> setHotLines(m_hot_lines);
    m_inst_profiler_ptr -> setAllInstructions(m_all_instructions);
  }
}

Profiler::~Profiler()
{
  if (m_periodic_output_file_ptr != &cerr) {
    delete m_periodic_output_file_ptr;
  }

  for (int mem_cntrl = 0; 
       mem_cntrl < m_mc_profilers.size(); 
       mem_cntrl++) {
    delete m_mc_profilers[mem_cntrl];
  }

  delete m_requestProfileMap_ptr;
}

void Profiler::wakeup()
{
  // FIXME - avoid the repeated code

  Vector<integer_t> perProcCycleCount;
  perProcCycleCount.setSize(m_num_of_sequencers);

  for(int i=0; i < m_num_of_sequencers; i++) {
    perProcCycleCount[i] = g_system_ptr->getCycleCount(i) - m_cycles_executed_at_start[i] + 1;
    // The +1 allows us to avoid division by zero
  }

  integer_t total_misses = m_perProcTotalMisses.sum();
  integer_t simics_cycles_executed = perProcCycleCount.sum();
  integer_t transactions_started = m_perProcStartTransaction.sum();
  integer_t transactions_ended = m_perProcEndTransaction.sum();

  (*m_periodic_output_file_ptr) << "ruby_cycles: " 
                                << g_eventQueue_ptr->getTime()-m_ruby_start 
                                << endl;

  (*m_periodic_output_file_ptr) << "total_misses: " 
                                << total_misses 
                                << " " 
                                << m_perProcTotalMisses 
                                << endl;

  (*m_periodic_output_file_ptr) << "simics_cycles_executed: " 
                                << simics_cycles_executed 
                                << " " 
                                << perProcCycleCount 
                                << endl;

  (*m_periodic_output_file_ptr) << "transactions_started: " 
                                << transactions_started 
                                << " " 
                                << m_perProcStartTransaction 
                                << endl;

  (*m_periodic_output_file_ptr) << "transactions_ended: " 
                                << transactions_ended 
                                << " " 
                                << m_perProcEndTransaction 
                                << endl;

  (*m_periodic_output_file_ptr) << "mbytes_resident: " 
                                << process_memory_resident() 
                                << endl;

  (*m_periodic_output_file_ptr) << "mbytes_total: " 
                                << process_memory_total() 
                                << endl;

  if (process_memory_total() > 0) {
    (*m_periodic_output_file_ptr) << "resident_ratio: " 
                          << process_memory_resident()/process_memory_total() 
                          << endl;
  }

  (*m_periodic_output_file_ptr) << "miss_latency: " 
                                << m_allMissLatencyHistogram 
                                << endl;

  *m_periodic_output_file_ptr << endl;

  if (m_all_instructions) {
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
  cout << "Recording periodic statistics every " << m_stats_period 
       << " Ruby cycles" << endl;

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

  Vector<integer_t> perProcCycleCount;
  Vector<double> perProcCyclesPerTrans;
  Vector<double> perProcMissesPerTrans;


  perProcCycleCount.setSize(m_num_of_sequencers);
  perProcCyclesPerTrans.setSize(m_num_of_sequencers);
  perProcMissesPerTrans.setSize(m_num_of_sequencers);

  for(int i=0; i < m_num_of_sequencers; i++) {
    perProcCycleCount[i] = g_system_ptr->getCycleCount(i) - m_cycles_executed_at_start[i] + 1;
    // The +1 allows us to avoid division by zero

    int trans = m_perProcEndTransaction[i];
    if (trans == 0) {
      perProcCyclesPerTrans[i] = 0;
      perProcMissesPerTrans[i] = 0;
    } else {
      perProcCyclesPerTrans[i] = ruby_cycles / double(trans);
      perProcMissesPerTrans[i] = m_perProcTotalMisses[i] / double(trans);
    }
  }

  integer_t total_misses = m_perProcTotalMisses.sum();
  integer_t user_misses = m_perProcUserMisses.sum();
  integer_t supervisor_misses = m_perProcSupervisorMisses.sum();
  integer_t simics_cycles_executed = perProcCycleCount.sum();
  integer_t transactions_started = m_perProcStartTransaction.sum();
  integer_t transactions_ended = m_perProcEndTransaction.sum();

  double cycles_per_transaction = (transactions_ended != 0) ? (m_num_of_sequencers * double(ruby_cycles)) / double(transactions_ended) : 0;
  double misses_per_transaction = (transactions_ended != 0) ? double(total_misses) / double(transactions_ended) : 0;

  out << "Total_misses: " << total_misses << endl;
  out << "total_misses: " << total_misses << " " << m_perProcTotalMisses << endl;
  out << "user_misses: " << user_misses << " " << m_perProcUserMisses << endl;
  out << "supervisor_misses: " << supervisor_misses << " " << m_perProcSupervisorMisses << endl;
  out << endl;
  out << "ruby_cycles_executed: " << simics_cycles_executed << " " << perProcCycleCount << endl;
  out << endl;
  out << "transactions_started: " << transactions_started << " " << m_perProcStartTransaction << endl;
  out << "transactions_ended: " << transactions_ended << " " << m_perProcEndTransaction << endl;
  out << "cycles_per_transaction: " << cycles_per_transaction  << " " << perProcCyclesPerTrans << endl;
  out << "misses_per_transaction: " << misses_per_transaction << " " << perProcMissesPerTrans << endl;

  out << endl;

  out << endl;

  for (int mem_cntrl = 0; 
       mem_cntrl < m_mc_profilers.size(); 
       mem_cntrl++) {
    uint64 m_memReq = m_mc_profilers[mem_cntrl]->m_memReq;
    uint64 m_memRefresh = m_mc_profilers[mem_cntrl]->m_memRefresh;
    uint64 m_memInputQ = m_mc_profilers[mem_cntrl]->m_memInputQ;
    uint64 m_memBankQ = m_mc_profilers[mem_cntrl]->m_memBankQ;
    uint64 m_memWaitCycles = m_mc_profilers[mem_cntrl]->m_memWaitCycles;
    uint64 m_memRead = m_mc_profilers[mem_cntrl]->m_memRead;
    uint64 m_memWrite = m_mc_profilers[mem_cntrl]->m_memWrite;
    uint64 m_memBankBusy = m_mc_profilers[mem_cntrl]->m_memBankBusy;
    uint64 m_memRandBusy = m_mc_profilers[mem_cntrl]->m_memRandBusy;
    uint64 m_memNotOld = m_mc_profilers[mem_cntrl]->m_memNotOld;
    uint64 m_memArbWait = m_mc_profilers[mem_cntrl]->m_memArbWait;
    uint64 m_memBusBusy = m_mc_profilers[mem_cntrl]->m_memBusBusy;
    uint64 m_memTfawBusy = m_mc_profilers[mem_cntrl]->m_memTfawBusy;
    uint64 m_memReadWriteBusy = m_mc_profilers[mem_cntrl]->m_memReadWriteBusy;
    uint64 m_memDataBusBusy = m_mc_profilers[mem_cntrl]->m_memDataBusBusy;
    Vector<uint64> m_memBankCount = m_mc_profilers[mem_cntrl]->m_memBankCount;

    if (m_memReq || m_memRefresh) {    // if there's a memory controller at all
      uint64 total_stalls = m_memInputQ + m_memBankQ + m_memWaitCycles;
      double stallsPerReq = total_stalls * 1.0 / m_memReq;
      out << "Memory control " << mem_cntrl << ":" << endl;
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

    out << "sequencer_requests_outstanding: " << m_sequencer_requests << endl;
    out << endl;
  }

  if (!short_stats) {
    out << "All Non-Zero Cycle Demand Cache Accesses" << endl;
    out << "----------------------------------------" << endl;
    out << "miss_latency: " << m_allMissLatencyHistogram << endl;
    for(int i=0; i<m_missLatencyHistograms.size(); i++) {
      if (m_missLatencyHistograms[i].size() > 0) {
        out << "miss_latency_" << RubyRequestType(i) << ": " << m_missLatencyHistograms[i] << endl;
      }
    }
    for(int i=0; i<m_machLatencyHistograms.size(); i++) {
      if (m_machLatencyHistograms[i].size() > 0) {
        out << "miss_latency_" << GenericMachineType(i) << ": " << m_machLatencyHistograms[i] << endl;
      }
    }

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

    if (m_outstanding_requests.size() > 0) {
      out << "outstanding_requests: "; m_outstanding_requests.printPercent(out); out << endl;
      out << endl;
    }
  }

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
  m_ruby_start = g_eventQueue_ptr->getTime();

  m_cycles_executed_at_start.setSize(m_num_of_sequencers);
  for (int i=0; i < m_num_of_sequencers; i++) {
    if (g_system_ptr == NULL) {
      m_cycles_executed_at_start[i] = 0;
    } else {
      m_cycles_executed_at_start[i] = g_system_ptr->getCycleCount(i);
    }
  }

  m_perProcTotalMisses.setSize(m_num_of_sequencers);
  m_perProcUserMisses.setSize(m_num_of_sequencers);
  m_perProcSupervisorMisses.setSize(m_num_of_sequencers);
  m_perProcStartTransaction.setSize(m_num_of_sequencers);
  m_perProcEndTransaction.setSize(m_num_of_sequencers);

  for(int i=0; i < m_num_of_sequencers; i++) {
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
  m_delayedCyclesVCHistograms.setSize(RubySystem::getNetwork()->getNumberOfVirtualNetworks());
  for (int i = 0; i < RubySystem::getNetwork()->getNumberOfVirtualNetworks(); i++) {
    m_delayedCyclesVCHistograms[i].clear();
  }

  m_missLatencyHistograms.setSize(RubyRequestType_NUM);
  for(int i=0; i<m_missLatencyHistograms.size(); i++) {
    m_missLatencyHistograms[i].clear(200);
  }
  m_machLatencyHistograms.setSize(GenericMachineType_NUM+1);
  for(int i=0; i<m_machLatencyHistograms.size(); i++) {
    m_machLatencyHistograms[i].clear(200);
  }
  m_allMissLatencyHistogram.clear(200);

  m_SWPrefetchLatencyHistograms.setSize(CacheRequestType_NUM);
  for(int i=0; i<m_SWPrefetchLatencyHistograms.size(); i++) {
    m_SWPrefetchLatencyHistograms[i].clear(200);
  }
  m_SWPrefetchMachLatencyHistograms.setSize(GenericMachineType_NUM+1);
  for(int i=0; i<m_SWPrefetchMachLatencyHistograms.size(); i++) {
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
  m_requestProfileMap_ptr->clear();

  // count requests profiled
  m_requests = 0;

  m_outstanding_requests.clear();
  m_outstanding_persistent_requests.clear();

//added by SS
  vector<string>::iterator it;

  for (int mem_cntrl = 0; 
       mem_cntrl < m_mc_profilers.size(); 
       mem_cntrl++) {
    m_mc_profilers[mem_cntrl]->m_memReq = 0;
    m_mc_profilers[mem_cntrl]->m_memBankBusy = 0;
    m_mc_profilers[mem_cntrl]->m_memBusBusy = 0;
    m_mc_profilers[mem_cntrl]->m_memTfawBusy = 0;
    m_mc_profilers[mem_cntrl]->m_memReadWriteBusy = 0;
    m_mc_profilers[mem_cntrl]->m_memDataBusBusy = 0;
    m_mc_profilers[mem_cntrl]->m_memRefresh = 0;
    m_mc_profilers[mem_cntrl]->m_memRead = 0;
    m_mc_profilers[mem_cntrl]->m_memWrite = 0;
    m_mc_profilers[mem_cntrl]->m_memWaitCycles = 0;
    m_mc_profilers[mem_cntrl]->m_memInputQ = 0;
    m_mc_profilers[mem_cntrl]->m_memBankQ = 0;
    m_mc_profilers[mem_cntrl]->m_memArbWait = 0;
    m_mc_profilers[mem_cntrl]->m_memRandBusy = 0;
    m_mc_profilers[mem_cntrl]->m_memNotOld = 0;

    for (int bank=0; 
         bank < m_mc_profilers[mem_cntrl]->m_memBankCount.size(); 
         bank++) {
        m_mc_profilers[mem_cntrl]->m_memBankCount[bank] = 0;
    }
  }
  // Flush the prefetches through the system - used so that there are no outstanding requests after stats are cleared
  //g_eventQueue_ptr->triggerAllEvents();

  // update the start time
  m_ruby_start = g_eventQueue_ptr->getTime();
}

void Profiler::addAddressTraceSample(const CacheMsg& msg, NodeID id)
{
  if (msg.getType() != CacheRequestType_IFETCH) {

    // Note: The following line should be commented out if you want to
    // use the special profiling that is part of the GS320 protocol

    // NOTE: Unless PROFILE_HOT_LINES is enabled, nothing will be profiled by the AddressProfiler
    m_address_profiler_ptr->addTraceSample(msg.getLineAddress(), msg.getProgramCounter(), msg.getType(), msg.getAccessMode(), id, false);
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
void Profiler::missLatency(Time t, RubyRequestType type)
{
  m_allMissLatencyHistogram.add(t);
  m_missLatencyHistograms[type].add(t);
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

void Profiler::profileTransition(const string& component, NodeID version, Address addr,
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
    (* debug_cout_ptr) << setw(ID_SPACES) << version << " ";
    (* debug_cout_ptr) << setw(COMP_SPACES) << component;
    (* debug_cout_ptr) << setw(EVENT_SPACES) << event << " ";

    (* debug_cout_ptr).flags(ios::right);
    (* debug_cout_ptr) << setw(STATE_SPACES) << state;
    (* debug_cout_ptr) << ">";
    (* debug_cout_ptr).flags(ios::left);
    (* debug_cout_ptr) << setw(STATE_SPACES) << next_state;

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

void Profiler::rubyWatch(int id){
    //int rn_g1 = 0;//SIMICS_get_register_number(id, "g1");
  uint64 tr = 0;//SIMICS_read_register(id, rn_g1);
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

int64 Profiler::getTotalTransactionsExecuted() const {
  return m_perProcEndTransaction.sum();
}

// For MemoryControl:
void Profiler::profileMemReq(int mem_cntrl, int bank) {
  m_mc_profilers[mem_cntrl]->m_memReq++;
  m_mc_profilers[mem_cntrl]->m_memBankCount[bank]++;
}

void Profiler::profileMemBankBusy(int mem_cntrl) {    
  m_mc_profilers[mem_cntrl]->m_memBankBusy++; 
}

void Profiler::profileMemBusBusy(int mem_cntrl) {    
  m_mc_profilers[mem_cntrl]->m_memBusBusy++; 
}

void Profiler::profileMemReadWriteBusy(int mem_cntrl) {    
  m_mc_profilers[mem_cntrl]->m_memReadWriteBusy++; 
}

void Profiler::profileMemDataBusBusy(int mem_cntrl) {    
  m_mc_profilers[mem_cntrl]->m_memDataBusBusy++; 
}

void Profiler::profileMemTfawBusy(int mem_cntrl) {    
  m_mc_profilers[mem_cntrl]->m_memTfawBusy++; 
}

void Profiler::profileMemRefresh(int mem_cntrl) {    
  m_mc_profilers[mem_cntrl]->m_memRefresh++; 
}

void Profiler::profileMemRead(int mem_cntrl) {    
  m_mc_profilers[mem_cntrl]->m_memRead++; 
}

void Profiler::profileMemWrite(int mem_cntrl) {    
  m_mc_profilers[mem_cntrl]->m_memWrite++; 
}

void Profiler::profileMemWaitCycles(int mem_cntrl, int cycles) {
  m_mc_profilers[mem_cntrl]->m_memWaitCycles += cycles; 
}

void Profiler::profileMemInputQ(int mem_cntrl, int cycles) {    
  m_mc_profilers[mem_cntrl]->m_memInputQ += cycles; 
}

void Profiler::profileMemBankQ(int mem_cntrl, int cycles) {    
  m_mc_profilers[mem_cntrl]->m_memBankQ += cycles; 
}

void Profiler::profileMemArbWait(int mem_cntrl, int cycles) {    
  m_mc_profilers[mem_cntrl]->m_memArbWait += cycles; 
}

void Profiler::profileMemRandBusy(int mem_cntrl) {    
  m_mc_profilers[mem_cntrl]->m_memRandBusy++; 
}

void Profiler::profileMemNotOld(int mem_cntrl) {    
  m_mc_profilers[mem_cntrl]->m_memNotOld++; 
}


Profiler *
RubyProfilerParams::create()
{
    return new Profiler(this);
}
