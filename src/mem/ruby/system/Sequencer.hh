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
#include "mem/protocol/MachineType.hh"
#include "mem/protocol/RubyRequestType.hh"
#include "mem/protocol/SequencerRequestType.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/system/CacheMemory.hh"
#include "mem/ruby/system/RubyPort.hh"
#include "params/RubySequencer.hh"

class DataBlock;

struct SequencerRequest
{
    PacketPtr pkt;
    RubyRequestType m_type;
    Cycles issue_time;

    SequencerRequest(PacketPtr _pkt, RubyRequestType _m_type,
                     Cycles _issue_time)
        : pkt(_pkt), m_type(_m_type), issue_time(_issue_time)
    {}
};

std::ostream& operator<<(std::ostream& out, const SequencerRequest& obj);

class Sequencer : public RubyPort
{
  public:
    typedef RubySequencerParams Params;
    Sequencer(const Params *);
    ~Sequencer();

    // Public Methods
    void wakeup(); // Used only for deadlock detection
    void printProgress(std::ostream& out) const;
    void clearStats();

    void writeCallback(const Address& address,
                       DataBlock& data,
                       const bool externalHit = false,
                       const MachineType mach = MachineType_NUM,
                       const Cycles initialRequestTime = Cycles(0),
                       const Cycles forwardRequestTime = Cycles(0),
                       const Cycles firstResponseTime = Cycles(0));

    void readCallback(const Address& address,
                      DataBlock& data,
                      const bool externalHit = false,
                      const MachineType mach = MachineType_NUM,
                      const Cycles initialRequestTime = Cycles(0),
                      const Cycles forwardRequestTime = Cycles(0),
                      const Cycles firstResponseTime = Cycles(0));

    RequestStatus makeRequest(PacketPtr pkt);
    bool empty() const;
    int outstandingCount() const { return m_outstanding_count; }

    bool isDeadlockEventScheduled() const
    { return deadlockCheckEvent.scheduled(); }

    void descheduleDeadlockEvent()
    { deschedule(deadlockCheckEvent); }

    void print(std::ostream& out) const;
    void printStats(std::ostream& out) const;
    void checkCoherence(const Address& address);

    void markRemoved();
    void removeRequest(SequencerRequest* request);
    void evictionCallback(const Address& address);
    void invalidateSC(const Address& address);

    void recordRequestType(SequencerRequestType requestType);
    Histogram& getOutstandReqHist() { return m_outstandReqHist; }

    Histogram& getLatencyHist() { return m_latencyHist; }
    Histogram& getTypeLatencyHist(uint32_t t)
    { return m_typeLatencyHist[t]; }

    Histogram& getHitLatencyHist() { return m_hitLatencyHist; }
    Histogram& getHitTypeLatencyHist(uint32_t t)
    { return m_hitTypeLatencyHist[t]; }

    Histogram& getHitMachLatencyHist(uint32_t t)
    { return m_hitMachLatencyHist[t]; }

    Histogram& getHitTypeMachLatencyHist(uint32_t r, uint32_t t)
    { return m_hitTypeMachLatencyHist[r][t]; }

    Histogram& getMissLatencyHist() { return m_missLatencyHist; }
    Histogram& getMissTypeLatencyHist(uint32_t t)
    { return m_missTypeLatencyHist[t]; }

    Histogram& getMissMachLatencyHist(uint32_t t)
    { return m_missMachLatencyHist[t]; }

    Histogram& getMissTypeMachLatencyHist(uint32_t r, uint32_t t)
    { return m_missTypeMachLatencyHist[r][t]; }

    Histogram& getIssueToInitialDelayHist(uint32_t t)
    { return m_IssueToInitialDelayHist[t]; }

    Histogram& getInitialToForwardDelayHist(const MachineType t)
    { return m_InitialToForwardDelayHist[t]; }

    Histogram& getForwardRequestToFirstResponseHist(const MachineType t)
    { return m_ForwardToFirstResponseDelayHist[t]; }

    Histogram& getFirstResponseToCompletionDelayHist(const MachineType t)
    { return m_FirstResponseToCompletionDelayHist[t]; }

    const uint64_t getIncompleteTimes(const MachineType t) const
    { return m_IncompleteTimes[t]; }

  private:
    void issueRequest(PacketPtr pkt, RubyRequestType type);

    void hitCallback(SequencerRequest* request, DataBlock& data,
                     bool llscSuccess,
                     const MachineType mach, const bool externalHit,
                     const Cycles initialRequestTime,
                     const Cycles forwardRequestTime,
                     const Cycles firstResponseTime);

    void recordMissLatency(const Cycles t, const RubyRequestType type,
                           const MachineType respondingMach,
                           bool isExternalHit, Cycles issuedTime,
                           Cycles initialRequestTime,
                           Cycles forwardRequestTime, Cycles firstResponseTime,
                           Cycles completionTime);

    RequestStatus insertRequest(PacketPtr pkt, RubyRequestType request_type);
    bool handleLlsc(const Address& address, SequencerRequest* request);

    // Private copy constructor and assignment operator
    Sequencer(const Sequencer& obj);
    Sequencer& operator=(const Sequencer& obj);

  private:
    int m_max_outstanding_requests;
    Cycles m_deadlock_threshold;

    CacheMemory* m_dataCache_ptr;
    CacheMemory* m_instCache_ptr;

    typedef m5::hash_map<Address, SequencerRequest*> RequestTable;
    RequestTable m_writeRequestTable;
    RequestTable m_readRequestTable;
    // Global outstanding request count, across all request tables
    int m_outstanding_count;
    bool m_deadlock_check_scheduled;

    uint32_t m_store_waiting_on_load_cycles;
    uint32_t m_store_waiting_on_store_cycles;
    uint32_t m_load_waiting_on_store_cycles;
    uint32_t m_load_waiting_on_load_cycles;

    bool m_usingNetworkTester;

    //! Histogram for number of outstanding requests per cycle.
    Histogram m_outstandReqHist;

    //! Histogram for holding latency profile of all requests.
    Histogram m_latencyHist;
    std::vector<Histogram> m_typeLatencyHist;

    //! Histogram for holding latency profile of all requests that
    //! hit in the controller connected to this sequencer.
    Histogram m_hitLatencyHist;
    std::vector<Histogram> m_hitTypeLatencyHist;

    //! Histograms for profiling the latencies for requests that
    //! did not required external messages.
    std::vector<Histogram> m_hitMachLatencyHist;
    std::vector< std::vector<Histogram> > m_hitTypeMachLatencyHist;

    //! Histogram for holding latency profile of all requests that
    //! miss in the controller connected to this sequencer.
    Histogram m_missLatencyHist;
    std::vector<Histogram> m_missTypeLatencyHist;

    //! Histograms for profiling the latencies for requests that
    //! required external messages.
    std::vector<Histogram> m_missMachLatencyHist;
    std::vector< std::vector<Histogram> > m_missTypeMachLatencyHist;

    //! Histograms for recording the breakdown of miss latency
    std::vector<Histogram> m_IssueToInitialDelayHist;
    std::vector<Histogram> m_InitialToForwardDelayHist;
    std::vector<Histogram> m_ForwardToFirstResponseDelayHist;
    std::vector<Histogram> m_FirstResponseToCompletionDelayHist;
    std::vector<uint64_t> m_IncompleteTimes;


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
