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

#ifndef __MEM_RUBY_SYSTEM_SEQUENCER_HH__
#define __MEM_RUBY_SYSTEM_SEQUENCER_HH__

#include <iostream>

#include "base/hashmap.hh"
#include "mem/protocol/GenericMachineType.hh"
#include "mem/protocol/RubyRequestType.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/system/RubyPort.hh"

class DataBlock;
class CacheMemory;

struct RubySequencerParams;

struct SequencerRequest
{
    PacketPtr pkt;
    RubyRequestType m_type;
    Time issue_time;

    SequencerRequest(PacketPtr _pkt, RubyRequestType _m_type, Time _issue_time)
        : pkt(_pkt), m_type(_m_type), issue_time(_issue_time)
    {}
};

std::ostream& operator<<(std::ostream& out, const SequencerRequest& obj);

class Sequencer : public RubyPort, public Consumer
{
  public:
    typedef RubySequencerParams Params;
    Sequencer(const Params *);
    ~Sequencer();

    // Public Methods
    void wakeup(); // Used only for deadlock detection

    void printConfig(std::ostream& out) const;

    void printProgress(std::ostream& out) const;

    void writeCallback(const Address& address, DataBlock& data);

    void writeCallback(const Address& address, 
                       GenericMachineType mach, 
                       DataBlock& data);

    void writeCallback(const Address& address, 
                       GenericMachineType mach, 
                       DataBlock& data,
                       Time initialRequestTime,
                       Time forwardRequestTime,
                       Time firstResponseTime);

    void readCallback(const Address& address, DataBlock& data);

    void readCallback(const Address& address, 
                      GenericMachineType mach, 
                      DataBlock& data);

    void readCallback(const Address& address, 
                      GenericMachineType mach, 
                      DataBlock& data,
                      Time initialRequestTime,
                      Time forwardRequestTime,
                      Time firstResponseTime);

    RequestStatus makeRequest(PacketPtr pkt);
    bool empty() const;
    int outstandingCount() const { return m_outstanding_count; }
    bool
    isDeadlockEventScheduled() const
    {
        return deadlockCheckEvent.scheduled();
    }

    void
    descheduleDeadlockEvent()
    {
        deschedule(deadlockCheckEvent);
    }

    void print(std::ostream& out) const;
    void printStats(std::ostream& out) const;
    void checkCoherence(const Address& address);

    void markRemoved();
    void removeRequest(SequencerRequest* request);
    void evictionCallback(const Address& address);

  private:
    void issueRequest(PacketPtr pkt, RubyRequestType type);

    void hitCallback(SequencerRequest* request, 
                     GenericMachineType mach,
                     DataBlock& data,
                     bool success,
                     Time initialRequestTime,
                     Time forwardRequestTime,
                     Time firstResponseTime);

    RequestStatus insertRequest(PacketPtr pkt, RubyRequestType request_type);

    bool handleLlsc(const Address& address, SequencerRequest* request);

    // Private copy constructor and assignment operator
    Sequencer(const Sequencer& obj);
    Sequencer& operator=(const Sequencer& obj);

  private:
    int m_max_outstanding_requests;
    int m_deadlock_threshold;

    CacheMemory* m_dataCache_ptr;
    CacheMemory* m_instCache_ptr;

    typedef m5::hash_map<Address, SequencerRequest*> RequestTable;
    RequestTable m_writeRequestTable;
    RequestTable m_readRequestTable;
    // Global outstanding request count, across all request tables
    int m_outstanding_count;
    bool m_deadlock_check_scheduled;

    int m_store_waiting_on_load_cycles;
    int m_store_waiting_on_store_cycles;
    int m_load_waiting_on_store_cycles;
    int m_load_waiting_on_load_cycles;

    bool m_usingNetworkTester;

    class SequencerWakeupEvent : public Event
    {
      private:
        Sequencer *m_sequencer_ptr;

      public:
        SequencerWakeupEvent(Sequencer *_seq) : m_sequencer_ptr(_seq) {}
        void process() { m_sequencer_ptr->wakeup(); }
        const char *description() const { return "Sequencer deadlock check"; }
    };

    SequencerWakeupEvent deadlockCheckEvent;
};

inline std::ostream&
operator<<(std::ostream& out, const Sequencer& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_SYSTEM_SEQUENCER_HH__
