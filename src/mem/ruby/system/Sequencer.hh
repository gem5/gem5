/*
 * Copyright (c) 2019-2020 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
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
#include <list>
#include <unordered_map>

#include "mem/ruby/common/Address.hh"
#include "mem/ruby/protocol/MachineType.hh"
#include "mem/ruby/protocol/RubyRequestType.hh"
#include "mem/ruby/protocol/SequencerRequestType.hh"
#include "mem/ruby/structures/CacheMemory.hh"
#include "mem/ruby/system/RubyPort.hh"
#include "params/RubySequencer.hh"

struct SequencerRequest
{
    PacketPtr pkt;
    RubyRequestType m_type;
    RubyRequestType m_second_type;
    Cycles issue_time;
    SequencerRequest(PacketPtr _pkt, RubyRequestType _m_type,
                     RubyRequestType _m_second_type, Cycles _issue_time)
                : pkt(_pkt), m_type(_m_type), m_second_type(_m_second_type),
                  issue_time(_issue_time)
    {}

    bool functionalWrite(Packet *func_pkt) const
    {
        // Follow-up on RubyRequest::functionalWrite
        // This makes sure the hitCallback won't overrite the value we
        // expect to find
        assert(func_pkt->isWrite());
        return func_pkt->trySatisfyFunctional(pkt);
    }
};

std::ostream& operator<<(std::ostream& out, const SequencerRequest& obj);

class Sequencer : public RubyPort
{
  public:
    typedef RubySequencerParams Params;
    Sequencer(const Params *);
    ~Sequencer();

    /**
     * Proxy function to writeCallback that first
     * invalidates the line address in the local monitor.
     */
    void writeCallbackScFail(Addr address,
                        DataBlock& data);

    // Public Methods
    void wakeup(); // Used only for deadlock detection
    void resetStats() override;
    void collateStats();
    void regStats() override;

    void writeCallback(Addr address,
                       DataBlock& data,
                       const bool externalHit = false,
                       const MachineType mach = MachineType_NUM,
                       const Cycles initialRequestTime = Cycles(0),
                       const Cycles forwardRequestTime = Cycles(0),
                       const Cycles firstResponseTime = Cycles(0));

    void readCallback(Addr address,
                      DataBlock& data,
                      const bool externalHit = false,
                      const MachineType mach = MachineType_NUM,
                      const Cycles initialRequestTime = Cycles(0),
                      const Cycles forwardRequestTime = Cycles(0),
                      const Cycles firstResponseTime = Cycles(0));

    RequestStatus makeRequest(PacketPtr pkt) override;
    bool empty() const;
    int outstandingCount() const override { return m_outstanding_count; }

    bool isDeadlockEventScheduled() const override
    { return deadlockCheckEvent.scheduled(); }

    void descheduleDeadlockEvent() override
    { deschedule(deadlockCheckEvent); }

    void print(std::ostream& out) const;
    void checkCoherence(Addr address);

    void markRemoved();
    void evictionCallback(Addr address);
    int coreId() const { return m_coreId; }

    virtual int functionalWrite(Packet *func_pkt) override;

    void recordRequestType(SequencerRequestType requestType);
    Stats::Histogram& getOutstandReqHist() { return m_outstandReqHist; }

    Stats::Histogram& getLatencyHist() { return m_latencyHist; }
    Stats::Histogram& getTypeLatencyHist(uint32_t t)
    { return *m_typeLatencyHist[t]; }

    Stats::Histogram& getHitLatencyHist() { return m_hitLatencyHist; }
    Stats::Histogram& getHitTypeLatencyHist(uint32_t t)
    { return *m_hitTypeLatencyHist[t]; }

    Stats::Histogram& getHitMachLatencyHist(uint32_t t)
    { return *m_hitMachLatencyHist[t]; }

    Stats::Histogram& getHitTypeMachLatencyHist(uint32_t r, uint32_t t)
    { return *m_hitTypeMachLatencyHist[r][t]; }

    Stats::Histogram& getMissLatencyHist()
    { return m_missLatencyHist; }
    Stats::Histogram& getMissTypeLatencyHist(uint32_t t)
    { return *m_missTypeLatencyHist[t]; }

    Stats::Histogram& getMissMachLatencyHist(uint32_t t) const
    { return *m_missMachLatencyHist[t]; }

    Stats::Histogram&
    getMissTypeMachLatencyHist(uint32_t r, uint32_t t) const
    { return *m_missTypeMachLatencyHist[r][t]; }

    Stats::Histogram& getIssueToInitialDelayHist(uint32_t t) const
    { return *m_IssueToInitialDelayHist[t]; }

    Stats::Histogram&
    getInitialToForwardDelayHist(const MachineType t) const
    { return *m_InitialToForwardDelayHist[t]; }

    Stats::Histogram&
    getForwardRequestToFirstResponseHist(const MachineType t) const
    { return *m_ForwardToFirstResponseDelayHist[t]; }

    Stats::Histogram&
    getFirstResponseToCompletionDelayHist(const MachineType t) const
    { return *m_FirstResponseToCompletionDelayHist[t]; }

    Stats::Counter getIncompleteTimes(const MachineType t) const
    { return m_IncompleteTimes[t]; }

  private:
    void issueRequest(PacketPtr pkt, RubyRequestType type);

    void hitCallback(SequencerRequest* srequest, DataBlock& data,
                     bool llscSuccess,
                     const MachineType mach, const bool externalHit,
                     const Cycles initialRequestTime,
                     const Cycles forwardRequestTime,
                     const Cycles firstResponseTime);

    void recordMissLatency(SequencerRequest* srequest, bool llscSuccess,
                           const MachineType respondingMach,
                           bool isExternalHit, Cycles initialRequestTime,
                           Cycles forwardRequestTime,
                           Cycles firstResponseTime);

    RequestStatus insertRequest(PacketPtr pkt, RubyRequestType primary_type,
                                RubyRequestType secondary_type);

    // Private copy constructor and assignment operator
    Sequencer(const Sequencer& obj);
    Sequencer& operator=(const Sequencer& obj);

  private:
    int m_max_outstanding_requests;
    Cycles m_deadlock_threshold;

    CacheMemory* m_dataCache_ptr;
    CacheMemory* m_instCache_ptr;

    // The cache access latency for top-level caches (L0/L1). These are
    // currently assessed at the beginning of each memory access through the
    // sequencer.
    // TODO: Migrate these latencies into top-level cache controllers.
    Cycles m_data_cache_hit_latency;
    Cycles m_inst_cache_hit_latency;

    // RequestTable contains both read and write requests, handles aliasing
    std::unordered_map<Addr, std::list<SequencerRequest>> m_RequestTable;

    // Global outstanding request count, across all request tables
    int m_outstanding_count;
    bool m_deadlock_check_scheduled;

    int m_coreId;

    bool m_runningGarnetStandalone;

    //! Histogram for number of outstanding requests per cycle.
    Stats::Histogram m_outstandReqHist;

    //! Histogram for holding latency profile of all requests.
    Stats::Histogram m_latencyHist;
    std::vector<Stats::Histogram *> m_typeLatencyHist;

    //! Histogram for holding latency profile of all requests that
    //! hit in the controller connected to this sequencer.
    Stats::Histogram m_hitLatencyHist;
    std::vector<Stats::Histogram *> m_hitTypeLatencyHist;

    //! Histograms for profiling the latencies for requests that
    //! did not required external messages.
    std::vector<Stats::Histogram *> m_hitMachLatencyHist;
    std::vector< std::vector<Stats::Histogram *> > m_hitTypeMachLatencyHist;

    //! Histogram for holding latency profile of all requests that
    //! miss in the controller connected to this sequencer.
    Stats::Histogram m_missLatencyHist;
    std::vector<Stats::Histogram *> m_missTypeLatencyHist;

    //! Histograms for profiling the latencies for requests that
    //! required external messages.
    std::vector<Stats::Histogram *> m_missMachLatencyHist;
    std::vector< std::vector<Stats::Histogram *> > m_missTypeMachLatencyHist;

    //! Histograms for recording the breakdown of miss latency
    std::vector<Stats::Histogram *> m_IssueToInitialDelayHist;
    std::vector<Stats::Histogram *> m_InitialToForwardDelayHist;
    std::vector<Stats::Histogram *> m_ForwardToFirstResponseDelayHist;
    std::vector<Stats::Histogram *> m_FirstResponseToCompletionDelayHist;
    std::vector<Stats::Counter> m_IncompleteTimes;

    EventFunctionWrapper deadlockCheckEvent;

    // support for LL/SC

    /**
     * Places the cache line address into the global monitor
     * tagged with this Sequencer object's version id.
     */
    void llscLoadLinked(const Addr);

    /**
     * Removes the cache line address from the global monitor.
     * This is independent of this Sequencer object's version id.
     */
    void llscClearMonitor(const Addr);

    /**
     * Searches for cache line address in the global monitor
     * tagged with this Sequencer object's version id.
     * If a match is found, the entry is is erased from
     * the global monitor.
     *
     * @return a boolean indicating if the line address was found.
     */
    bool llscStoreConditional(const Addr);

  public:
    /**
     * Searches for cache line address in the global monitor
     * tagged with this Sequencer object's version id.
     *
     * @return a boolean indicating if the line address was found.
     */
    bool llscCheckMonitor(const Addr);
};

inline std::ostream&
operator<<(std::ostream& out, const Sequencer& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_SYSTEM_SEQUENCER_HH__
