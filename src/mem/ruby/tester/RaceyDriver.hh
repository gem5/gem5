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
 * Description: The driver interface between racey pseudo threads and ruby
 *              memory timing simulator.
 *
 */

#ifndef RACEYDRIVER_H
#define RACEYDRIVER_H

#include <map>
#include <vector>

#include "mem/ruby/common/Global.hh"
#include "mem/ruby/tester/Tester_Globals.hh"
#include "mem/ruby/common/Driver.hh"
#include "mem/ruby/tester/RaceyPseudoThread.hh"
#include "mem/ruby/libruby.hh"

#define g_DEADLOCK_THRESHOLD 5000


struct address_data {
  Address address;
  uint8_t * data;
};

class RaceyDriver : public Driver, public Consumer {
public:
  friend class RaceyPseudoThread;
  // Constructors
  RaceyDriver(int num_procs, int tester_length);

  // Destructor
  ~RaceyDriver();

  // Public Methods
  int runningThreads();
  void registerThread0Wakeup();
  void joinThread();
  bool Thread0Initialized() {
    return m_racey_pseudo_threads[0]->getInitializedState();
  };

  void hitCallback(int64_t request_id);
  void wakeup();
  void printStats(ostream& out) const;
  void clearStats() {}
  void printConfig(ostream& out) const {}
  void go();

  integer_t getInstructionCount(int procID) const;

  // save/load cpu states
  void saveCPUStates(int cpu_id, string filename) {
    m_racey_pseudo_threads[cpu_id]->saveCPUStates(filename);
  };
  void loadCPUStates(int cpu_id, string filename) {
    m_racey_pseudo_threads[cpu_id]->loadCPUStates(filename);
  };

  // reset IC
  void resetIC(int cpu_id) {
    m_racey_pseudo_threads[cpu_id]->resetIC();
  }

  void print(ostream& out) const;
  
private:

  // Private copy constructor and assignment operator
  RaceyDriver(const RaceyDriver& obj);
  RaceyDriver& operator=(const RaceyDriver& obj);
  
  // Data Members (m_ prefix)
  std::vector<RaceyPseudoThread*> m_racey_pseudo_threads;
  int m_done_counter;
  bool m_wakeup_thread0;
  Time m_finish_time;
  map <int64_t, pair <int, struct address_data> > requests;
  RubyEventQueue * eventQueue;
  int m_num_procs;
  int m_tester_length;
};

// Output operator declaration
ostream& operator<<(ostream& out, const RaceyDriver& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline 
ostream& operator<<(ostream& out, const RaceyDriver& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //RACEYDRIVER_H

