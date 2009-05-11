
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

#ifndef XACTPROFILER_H
#define XACTPROFILER_H

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

template <class KEY_TYPE, class VALUE_TYPE> class Map;

class XactProfiler {
public:
  // Constructors
  XactProfiler();

  // Destructor
  ~XactProfiler();

  void printStats(ostream& out, bool short_stats=false);
  void printShortStats(ostream& out) { printStats(out, true); }
  void clearStats();
  void printConfig(ostream& out) const;

  void print(ostream& out) const;

  void profileTransCycles(int proc, int cycles);
  void profileNonTransCycles(int proc, int cycles);
  void profileStallTransCycles(int proc, int cycles);
  void profileStallNonTransCycles(int proc, int cycles);
  void profileAbortingTransCycles(int proc, int cycles);
  void profileCommitingTransCycles(int proc, int cycles);
  void profileBarrierCycles(int proc, int cycles);
  void profileBackoffTransCycles(int proc, int cycles);
  void profileGoodTransCycles(int proc, int cycles);

  void profileBeginTimer(int proc);
  void profileEndTimer(int proc);

  long long int getTransCycles(int proc_no);
  long long int getGoodTransCycles(int proc_no);
  long long int getStallTransCycles(int proc_no);
  long long int getAbortingTransCycles(int proc_no);
  long long int getCommitingTransCycles(int proc_no);
  long long int getBackoffTransCycles(int proc_no);
  long long int getNonTransCycles(int proc_no);
  long long int getBarrierCycles(int proc_no);

  void profileHashValue(int hashFunction, int hashValue);

private:

  long long int * m_xactTransCycles;
  long long int * m_xactStallTransCycles;
  long long int * m_xactStallNonTransCycles;
  long long int * m_xactAbortingCycles;
  long long int * m_xactCommitingCycles;
  long long int * m_xactBackoffCycles;
  long long int * m_BarrierCycles;

  long long int * m_xactGoodTransCycles;
  long long int * m_xactNonTransCycles;

  long long int * m_xactTimedCycles;
  long long int * m_xactBeginTimer;

  int max_hashFunction;
  Vector<Histogram> m_hashProfile;
};

// Output operator declaration
ostream& operator<<(ostream& out, const XactProfiler& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const XactProfiler& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //XACTPROFILER_H


