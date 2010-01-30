
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
 * $Id: Sequencer.hh 1.70 2006/09/27 14:56:41-05:00 bobba@s1-01.cs.wisc.edu $
 *
 * Description:
 *
 */

#ifndef SEQUENCER_H
#define SEQUENCER_H

#include "mem/ruby/common/Global.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/protocol/CacheRequestType.hh"
#include "mem/protocol/AccessModeType.hh"
#include "mem/protocol/GenericMachineType.hh"
#include "mem/protocol/PrefetchBit.hh"
#include "mem/ruby/system/RubyPort.hh"
#include "mem/gems_common/Map.hh"
#include "mem/ruby/common/Address.hh"

class DataBlock;
class CacheMsg;
class MachineID;
class CacheMemory;

class RubySequencerParams;

struct SequencerRequest {
  RubyRequest ruby_request;
  int64_t id;
  Time issue_time;

  SequencerRequest(const RubyRequest & _ruby_request, int64_t _id, Time _issue_time)
    : ruby_request(_ruby_request), id(_id), issue_time(_issue_time)
  {}
};

std::ostream& operator<<(std::ostream& out, const SequencerRequest& obj);

class Sequencer : public RubyPort, public Consumer {
public:
    typedef RubySequencerParams Params;
  // Constructors
  Sequencer(const Params *);

  // Destructor
  ~Sequencer();

  // Public Methods
  void wakeup(); // Used only for deadlock detection

  void printConfig(ostream& out) const;

  void printProgress(ostream& out) const;

  void writeCallback(const Address& address, DataBlock& data);
  void readCallback(const Address& address, DataBlock& data);

  // called by Tester or Simics
  int64_t makeRequest(const RubyRequest & request);
  int isReady(const RubyRequest& request);
  bool empty() const;

  void print(ostream& out) const;
  void printStats(ostream & out) const;
  void checkCoherence(const Address& address);

  //  bool getRubyMemoryValue(const Address& addr, char* value, unsigned int size_in_bytes);
  //  bool setRubyMemoryValue(const Address& addr, char *value, unsigned int size_in_bytes);

  void removeRequest(SequencerRequest* request);
private:
  // Private Methods
  bool tryCacheAccess(const Address& addr, CacheRequestType type, const Address& pc, AccessModeType access_mode, int size, DataBlock*& data_ptr);
  void issueRequest(const RubyRequest& request);

  void hitCallback(SequencerRequest* request, DataBlock& data);
  bool insertRequest(SequencerRequest* request);


  // Private copy constructor and assignment operator
  Sequencer(const Sequencer& obj);
  Sequencer& operator=(const Sequencer& obj);

private:
  int m_max_outstanding_requests;
  int m_deadlock_threshold;

  CacheMemory* m_dataCache_ptr;
  CacheMemory* m_instCache_ptr;

  Map<Address, SequencerRequest*> m_writeRequestTable;
  Map<Address, SequencerRequest*> m_readRequestTable;
  // Global outstanding request count, across all request tables
  int m_outstanding_count;
  bool m_deadlock_check_scheduled;

  int m_store_waiting_on_load_cycles;
  int m_store_waiting_on_store_cycles;
  int m_load_waiting_on_store_cycles;
  int m_load_waiting_on_load_cycles;

  bool m_usingRubyTester;

  class SequencerWakeupEvent : public Event
  {
      Sequencer *m_sequencer_ptr;

    public:
      SequencerWakeupEvent(Sequencer *_seq) : m_sequencer_ptr(_seq) {}
      void process() { m_sequencer_ptr->wakeup(); }
      const char *description() const { return "Sequencer deadlock check"; }
  };

  SequencerWakeupEvent deadlockCheckEvent;
};

// Output operator declaration
ostream& operator<<(ostream& out, const Sequencer& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const Sequencer& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //SEQUENCER_H

