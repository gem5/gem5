
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

#include "mem/ruby/common/Global.hh"
#include "mem/ruby/common/Driver.hh"
#include "mem/ruby/common/Histogram.hh"
#include "mem/protocol/CacheRequestType.hh"

class RubySystem;
class SpecifiedGenerator;

class DeterministicDriver : public Driver, public Consumer {
public:
  // Constructors
  DeterministicDriver(RubySystem* sys_ptr);

  // Destructor
  ~DeterministicDriver();

  // Public Methods
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

  void hitCallback(NodeID proc, SubBlock& data, CacheRequestType type, int thread);
  void wakeup();
  void printStats(ostream& out) const;
  void clearStats() {}
  void printConfig(ostream& out) const {}

  void print(ostream& out) const;
private:
  // Private Methods
  void checkForDeadlock();

  Address getNextAddr(NodeID node, Vector<NodeID> addr_vector);
  bool isAddrReady(NodeID node, Vector<NodeID> addr_vector);
  bool isAddrReady(NodeID node, Vector<NodeID> addr_vector, Address addr);
  void setNextAddr(NodeID node, Address addr, Vector<NodeID>& addr_vector);

  // Private copy constructor and assignment operator
  DeterministicDriver(const DeterministicDriver& obj);
  DeterministicDriver& operator=(const DeterministicDriver& obj);

  // Data Members (m_ prefix)
  Vector<Time> m_last_progress_vector;
  Vector<SpecifiedGenerator*> m_generator_vector;
  Vector<NodeID> m_load_vector;  // Processor last to load the addr
  Vector<NodeID> m_store_vector;  // Processor last to store the addr

  int m_done_counter;
  int m_loads_completed;
  int m_stores_completed;
  // enforces the previous node to have a certain # of completions
  // before next node starts
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
