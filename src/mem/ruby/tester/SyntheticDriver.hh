
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

#ifndef SYNTHETICDRIVER_H
#define SYNTHETICDRIVER_H

#include "mem/ruby/common/Global.hh"
#include "mem/ruby/common/Driver.hh"
#include "mem/ruby/common/Histogram.hh"
#include "mem/protocol/CacheRequestType.hh"

class RubySystem;
class RequestGenerator;

class SyntheticDriver : public Driver, public Consumer {
public:
  // Constructors
  SyntheticDriver(RubySystem* sys_ptr);

  // Destructor
  ~SyntheticDriver();

  // Public Methods
  Address pickAddress(NodeID node);
  void reportDone();
  void recordTestLatency(Time time);
  void recordSwapLatency(Time time);
  void recordReleaseLatency(Time time);

  void hitCallback(NodeID proc, SubBlock& data, CacheRequestType type, int thread);
  void conflictCallback(NodeID proc, SubBlock& data, CacheRequestType type, int thread) {assert(0);}
  void abortCallback(NodeID proc, SubBlock& data, CacheRequestType type, int thread);
  void wakeup();
  void printStats(ostream& out) const;
  void clearStats() {}
  void printConfig(ostream& out) const {}

  integer_t readPhysicalMemory(int procID, physical_address_t address,
                               int len );

  void writePhysicalMemory( int procID, physical_address_t address,
                            integer_t value, int len );

  void print(ostream& out) const;

  // For handling NACKs/retries
  //void notifySendNack( int procID, const Address & addr, uint64 remote_timestamp, const MachineID & remote_id);
  //void notifyReceiveNack( int procID, const Address & addr, uint64 remote_timestamp, const MachineID & remote_id);
  //void notifyReceiveNackFinal( int procID, const Address & addr);

private:
  // Private Methods
  void checkForDeadlock();

  // Private copy constructor and assignment operator
  SyntheticDriver(const SyntheticDriver& obj);
  SyntheticDriver& operator=(const SyntheticDriver& obj);

  // Data Members (m_ prefix)
  Vector<Time> m_last_progress_vector;
  Vector<RequestGenerator*> m_request_generator_vector;
  Vector<NodeID> m_lock_vector;  // Processor last to hold the lock
  int m_done_counter;

  Histogram m_test_latency;
  Histogram m_swap_latency;
  Histogram m_release_latency;
  Time m_finish_time;
};

// Output operator declaration
ostream& operator<<(ostream& out, const SyntheticDriver& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const SyntheticDriver& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //SYNTHETICDRIVER_H
