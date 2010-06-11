
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
 * $Id$
 *
 * Description: 
 *
 */

#ifndef DETERMINISTICDRIVER_H
#define DETERMINISTICDRIVER_H

#include <map>
#include <vector>

#include "mem/ruby/common/Global.hh"
#include "mem/ruby/tester/Tester_Globals.hh"
#include "mem/ruby/common/Histogram.hh"            // includes global, but doesn't use anything, so it should be fine
#include "mem/protocol/CacheRequestType.hh"     // includes global, but doesn't use anything, so it should be fine
#include "mem/ruby/common/Address.hh"       // we redefined the address
#include "mem/ruby/tester/DetermGETXGenerator.hh"  // this is our file
#include "mem/ruby/tester/DetermSeriesGETSGenerator.hh"  // this is our file
#include "mem/ruby/tester/DetermInvGenerator.hh"  // this is our file
#include "mem/ruby/libruby.hh"
#include "mem/ruby/common/Driver.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/eventqueue/RubyEventQueue.hh"
#include "mem/protocol/SpecifiedGeneratorType.hh"

//class DMAGenerator;

class DeterministicDriver : public Driver, public Consumer {
public:
  friend class DetermGETXGenerator;
  friend class DetermSeriesGETSGenerator;
  friend class DetermInvGenerator;
  // Constructors
  DeterministicDriver(string generator_type, int num_completions, int num_procs, Time g_think_time, Time g_wait_time, int g_tester_length);

  // Destructor
  ~DeterministicDriver();
  
  // Public Methods
  void go();
  bool isStoreReady(NodeID node);
  bool isLoadReady(NodeID node);
  bool isStoreReady(NodeID node, Address addr);
  bool isLoadReady(NodeID node, Address addr);
  void loadCompleted(NodeID node, Address addr);
  void storeCompleted(NodeID node, Address addr);
  Address getNextLoadAddr(NodeID node);
  Address getNextStoreAddr(NodeID node);
  int getLoadsCompleted() { return m_loads_completed; }
  int getStoresCompleted() { return m_stores_completed; }

  void reportDone();
  void recordLoadLatency(Time time);
  void recordStoreLatency(Time time);

//  void dmaHitCallback();
  void hitCallback(int64_t request_id);
  void wakeup();
  void printStats(ostream& out) const;
  void clearStats() {}
  void printConfig(ostream& out) const {}

  void print(ostream& out) const;
  // Public copy constructor and assignment operator
  DeterministicDriver(const DeterministicDriver& obj);
  DeterministicDriver& operator=(const DeterministicDriver& obj);

private:
  // Private Methods

  Address getNextAddr(NodeID node, std::vector<NodeID> addr_vector);
  bool isAddrReady(NodeID node, std::vector<NodeID> addr_vector);
  bool isAddrReady(NodeID node, std::vector<NodeID> addr_vector, Address addr);
  void setNextAddr(NodeID node, Address addr, std::vector<NodeID>& addr_vector);

  
  // Data Members (m_ prefix)
  std::vector<Time> m_last_progress_vector;
  std::vector<SpecifiedGenerator*> m_generator_vector;
  //DMAGenerator* m_dma_generator;
  std::vector<NodeID> m_load_vector;  // Processor last to load the addr
  std::vector<NodeID> m_store_vector;  // Processor last to store the addr

  int last_proc;
  int m_done_counter;
  int m_loads_completed;
  int m_stores_completed;
  // enforces the previous node to have a certain # of completions
  // before next node starts
  
  map <int64_t, pair <int, Address> > requests;
  Time m_think_time;
  Time m_wait_time;
  int m_tester_length;
  int m_num_procs;
  RubyEventQueue * eventQueue;
  int m_numCompletionsPerNode;  

  Histogram m_load_latency;
  Histogram m_store_latency;
  Time m_finish_time;
  Time m_last_issue;
};

// Output operator declaration
ostream& operator<<(ostream& out, const DeterministicDriver& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline 
ostream& operator<<(ostream& out, const DeterministicDriver& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //DETERMINISTICDRIVER_H
