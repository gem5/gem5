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

#include "mem/ruby/profiler/XactProfiler.hh"
#include "mem/protocol/CacheMsg.hh"
#include "mem/gems_common/Map.hh"
#include "mem/ruby/common/Debug.hh"
#include "mem/protocol/MachineType.hh"
#include "TransactionInterfaceManager.hh"
#include "mem/ruby/common/Driver.hh"
#include "interface.hh"

extern std::ostream * debug_cout_ptr;

XactProfiler::XactProfiler()
{
  int num_processors = RubyConfig::numberOfProcessors() * RubyConfig::numberofSMTThreads();

  m_xactTransCycles = new long long int[num_processors];
  m_xactStallTransCycles = new long long int[num_processors];
  m_xactStallNonTransCycles = new long long int[num_processors];
  m_xactAbortingCycles = new long long int[num_processors];
  m_xactCommitingCycles = new long long int[num_processors];
  m_xactBackoffCycles = new long long int[num_processors];
  m_BarrierCycles = new long long int[num_processors];

  m_xactGoodTransCycles = new long long int[num_processors];
  m_xactNonTransCycles = new long long int[num_processors];

  m_xactTimedCycles = new long long int[num_processors];
  m_xactBeginTimer = new long long int[num_processors];

  clearStats();
}

XactProfiler::~XactProfiler()
{
  delete [] m_xactTransCycles;
  delete [] m_xactStallTransCycles;
  delete [] m_xactStallNonTransCycles;
  delete [] m_xactAbortingCycles;
  delete [] m_xactBackoffCycles;

  delete [] m_xactGoodTransCycles;
  delete [] m_xactNonTransCycles;

  delete [] m_xactTimedCycles;
  delete [] m_xactBeginTimer;
}

void XactProfiler::printConfig(ostream& out) const
{
  out << endl;
  out << "XactProfiler Configuration" << endl;
  out << "----------------------" << endl;
}

void XactProfiler::print(ostream& out) const
{
  out << "[XactProfiler]";
}

void XactProfiler::printStats(ostream& out, bool short_stats)
{
  int num_processors = RubyConfig::numberOfProcessors() * RubyConfig::numberofSMTThreads();
  out << endl;

  out << "XactProfiler Stats" << endl;
  out << "--------------" << endl;

  if (max_hashFunction >= 0){
    out << "Hash values distribution" << endl;
    out << "------------------------" << endl;
    for(int i = 0; i <= max_hashFunction; i++){
      out << "Hash function " << i << ": " << m_hashProfile[i] << endl;
    }
    out << endl;
  }

    out << "xact_cycle_breakdown" << endl;
    out << "--------------------" << endl;
    long long int total_trans_cycles = 0;
    long long int total_aborting_trans_cycles = 0;
    long long int total_commiting_trans_cycles = 0;
    long long int total_backoff_trans_cycles = 0;
    long long int total_stall_trans_cycles = 0;
    long long int total_stall_nontrans_cycles = 0;
    long long int total_barrier_cycles = 0;

    long long int total_good_trans_cycles = 0;
    long long int total_nontrans_cycles = 0;

    long long int total_timed_cycles = 0;

    for(int i=0; i < num_processors; ++i){
      if (!short_stats){
        out << "xact_trans_cycles_processor_" << i << ": " << m_xactTransCycles[i] << endl;
        out << "xact_aborting_cycles_processor_" << i << ": " << m_xactAbortingCycles[i] << endl;
        out << "xact_barrier_cycles_processor_" << i << ": " << m_BarrierCycles[i] << endl;
        out << "xact_backoff_cycles_processor_" << i << ": " << m_xactBackoffCycles[i] << endl;
        out << "xact_stall_trans_cycles_processor_" << i << ": " << m_xactStallTransCycles[i] << endl;
        out << "xact_nontrans_cycles_processor_" << i << ": " << m_xactNonTransCycles[i] << endl;
        out << "xact_stall_nontrans_cycles_processor_" << i << ": " << m_xactStallNonTransCycles[i] << endl;
        out << "timed_cycles_processor_" << i << ": " << m_xactTimedCycles[i] << endl;
      }

      total_trans_cycles += m_xactTransCycles[i];
      total_stall_trans_cycles += m_xactStallTransCycles[i];
      total_aborting_trans_cycles += m_xactAbortingCycles[i];
      total_commiting_trans_cycles += m_xactCommitingCycles[i];
      total_backoff_trans_cycles += m_xactBackoffCycles[i];
      total_barrier_cycles += m_BarrierCycles[i];
      total_nontrans_cycles += m_xactNonTransCycles[i];
      total_good_trans_cycles += m_xactGoodTransCycles[i];
      total_stall_nontrans_cycles += m_xactStallNonTransCycles[i];
      total_timed_cycles += m_xactTimedCycles[i];

    }
    out << endl;
    out << " XACT CYCLE BREAKDOWN " << endl;
    out << " XACT_BREAKDOWN_NON_TRANS_CYCLES:  " << total_nontrans_cycles << endl;
    out << " XACT_BREAKDOWN_TRANS_CYCLES:      " << total_trans_cycles << endl;
    out << " XACT_BREAKDOWN_GOOD_TRANS_CYCLES: " << total_good_trans_cycles << endl;
    out << " XACT_BREAKDOWN_ABORTING_CYCLES:   " << total_aborting_trans_cycles << endl;
    out << " XACT_BREAKDOWN_COMMITING_CYCLES:  " << total_commiting_trans_cycles << endl;
    out << " XACT_BREAKDOWN_BACKOFF_CYCLES:    " << total_backoff_trans_cycles << endl;
    out << " XACT_BREAKDOWN_BARRIER_CYCLES:    " << total_barrier_cycles << endl;
    out << " XACT_BREAKDOWN_STALL_CYCLES:      " << total_stall_trans_cycles << endl;
    out << endl;
    out << " XACT_TIMED_CYCLES:      " << total_timed_cycles << endl;
    out << endl;

}

void XactProfiler::clearStats()
{
  int num_processors = RubyConfig::numberOfProcessors() * RubyConfig::numberofSMTThreads();
  for(int i=0; i < num_processors; ++i){
    m_xactTransCycles[i] = 0;
    m_xactStallTransCycles[i] = 0;
    m_xactGoodTransCycles[i] = 0;
    m_xactBackoffCycles[i]    = 0;
    m_xactAbortingCycles[i]   = 0;
    m_xactCommitingCycles[i]   = 0;
    m_xactNonTransCycles[i] = 0;
    m_xactStallNonTransCycles[i] = 0;
    m_BarrierCycles[i]  = 0;
    m_xactTimedCycles[i] = 0;
  }

  max_hashFunction = -1;
  m_hashProfile.setSize(16);
  for (int i = 0; i < 15; i++) {
    m_hashProfile[i].clear();
  }
}

void XactProfiler::profileTransCycles(int proc, int cycles){
  m_xactTransCycles[proc] += cycles;
}

long long int XactProfiler::getTransCycles(int proc){
  return m_xactTransCycles[proc];
}

void XactProfiler::profileStallTransCycles(int proc, int cycles){
  m_xactStallTransCycles[proc] += cycles;
}

long long int XactProfiler::getStallTransCycles(int proc){
  return m_xactStallTransCycles[proc];
}

void XactProfiler::profileGoodTransCycles(int proc, int cycles){
  m_xactGoodTransCycles[proc] += cycles;
}

long long int XactProfiler::getGoodTransCycles(int proc){
  return m_xactGoodTransCycles[proc];
}

void XactProfiler::profileAbortingTransCycles(int proc, int cycles){
  m_xactAbortingCycles[proc] += cycles;
}

long long int XactProfiler::getAbortingTransCycles(int proc){
  return m_xactAbortingCycles[proc];
}

void XactProfiler::profileCommitingTransCycles(int proc, int cycles){
  m_xactCommitingCycles[proc] += cycles;
}

long long int XactProfiler::getCommitingTransCycles(int proc){
  return m_xactCommitingCycles[proc];
}

void XactProfiler::profileBackoffTransCycles(int proc, int cycles){
  m_xactBackoffCycles[proc] += cycles;
}

long long int XactProfiler::getBackoffTransCycles(int proc){
  return m_xactBackoffCycles[proc];
}

void XactProfiler::profileStallNonTransCycles(int proc, int cycles){
  m_xactStallNonTransCycles[proc] += cycles;
}

void XactProfiler::profileNonTransCycles(int proc, int cycles){
  m_xactNonTransCycles[proc] += cycles;
}

long long int XactProfiler::getNonTransCycles(int proc){
  return m_xactNonTransCycles[proc];
}

void XactProfiler::profileBarrierCycles(int proc, int cycles){
  m_BarrierCycles[proc] += cycles;
}

long long int XactProfiler::getBarrierCycles(int proc){
  return m_BarrierCycles[proc];
}

void XactProfiler::profileBeginTimer(int proc){
  m_xactBeginTimer[proc] = (long long int) g_eventQueue_ptr->getTime();
}

void XactProfiler::profileEndTimer(int proc){
  m_xactTimedCycles[proc] += (long long int) g_eventQueue_ptr->getTime() - m_xactBeginTimer[proc];
}

void XactProfiler::profileHashValue(int hashFunction, int hashValue){
   if (hashFunction > max_hashFunction) max_hashFunction = hashFunction;

   m_hashProfile[hashFunction].add(hashValue);
}
