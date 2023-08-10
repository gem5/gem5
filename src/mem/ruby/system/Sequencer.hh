/*
 * Copyright (c) 2019-2021 ARM Limited
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

namespace gem5
{

namespace ruby
{

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
    Sequencer(const Params &);
    ~Sequencer();

    /**
     * Proxy function to writeCallback that first
     * invalidates the line address in the local monitor.
     */
    void writeCallbackScFail(Addr address,
                        DataBlock& data);

    // Public Methods
    virtual void wakeup(); // Used only for deadlock detection
    void resetStats() override;
    void collateStats();

    void writeCallback(Addr address,
                       DataBlock& data,
                       const bool externalHit = false,
                       const MachineType mach = MachineType_NUM,
                       const Cycles initialRequestTime = Cycles(0),
                       const Cycles forwardRequestTime = Cycles(0),
                       const Cycles firstResponseTime = Cycles(0),
                       const bool noCoales = false);

    // Write callback that prevents coalescing
    void writeUniqueCallback(Addr address, DataBlock& data)
    {
        writeCallback(address, data, true, MachineType_NUM, Cycles(0),
                      Cycles(0), Cycles(0), true);
    }

    void readCallback(Addr address,
                      DataBlock& data,
                      const bool externalHit = false,
                      const MachineType mach = MachineType_NUM,
                      const Cycles initialRequestTime = Cycles(0),
                      const Cycles forwardRequestTime = Cycles(0),
                      const Cycles firstResponseTime = Cycles(0));

    void atomicCallback(Addr address,
                        DataBlock& data,
                        const bool externalHit = false,
                        const MachineType mach = MachineType_NUM,
                        const Cycles initialRequestTime = Cycles(0),
                        const Cycles forwardRequestTime = Cycles(0),
                        const Cycles firstResponseTime = Cycles(0));

    void unaddressedCallback(Addr unaddressedReqId,
                             RubyRequestType requestType,
                             const MachineType mach = MachineType_NUM,
                             const Cycles initialRequestTime = Cycles(0),
                             const Cycles forwardRequestTime = Cycles(0),
                             const Cycles firstResponseTime = Cycles(0));

    RequestStatus makeRequest(PacketPtr pkt) override;
    virtual bool empty() const;
    int outstandingCount() const override { return m_outstanding_count; }

    bool isDeadlockEventScheduled() const override
    { return deadlockCheckEvent.scheduled(); }

    void descheduleDeadlockEvent() override
    { deschedule(deadlockCheckEvent); }

    virtual void print(std::ostream& out) const;

    void markRemoved();
    void evictionCallback(Addr address);
    int coreId() const { return m_coreId; }

    virtual int functionalWrite(Packet *func_pkt) override;

    void recordRequestType(SequencerRequestType requestType);
    statistics::Histogram& getOutstandReqHist() { return m_outstandReqHist; }

    statistics::Histogram& getLatencyHist() { return m_latencyHist; }
    statistics::Histogram& getTypeLatencyHist(uint32_t t)
    { return *m_typeLatencyHist[t]; }

    statistics::Histogram& getHitLatencyHist() { return m_hitLatencyHist; }
    statistics::Histogram& getHitTypeLatencyHist(uint32_t t)
    { return *m_hitTypeLatencyHist[t]; }

    statistics::Histogram& getHitMachLatencyHist(uint32_t t)
    { return *m_hitMachLatencyHist[t]; }

    statistics::Histogram& getHitTypeMachLatencyHist(uint32_t r, uint32_t t)
    { return *m_hitTypeMachLatencyHist[r][t]; }

    statistics::Histogram& getMissLatencyHist()
    { return m_missLatencyHist; }
    statistics::Histogram& getMissTypeLatencyHist(uint32_t t)
    { return *m_missTypeLatencyHist[t]; }

    statistics::Histogram& getMissMachLatencyHist(uint32_t t) const
    { return *m_missMachLatencyHist[t]; }

    statistics::Histogram&
    getMissTypeMachLatencyHist(uint32_t r, uint32_t t) const
    { return *m_missTypeMachLatencyHist[r][t]; }

    statistics::Histogram& getIssueToInitialDelayHist(uint32_t t) const
    { return *m_IssueToInitialDelayHist[t]; }

    statistics::Histogram&
    getInitialToForwardDelayHist(const MachineType t) const
    { return *m_InitialToForwardDelayHist[t]; }

    statistics::Histogram&
    getForwardRequestToFirstResponseHist(const MachineType t) const
    { return *m_ForwardToFirstResponseDelayHist[t]; }

    statistics::Histogram&
    getFirstResponseToCompletionDelayHist(const MachineType t) const
    { return *m_FirstResponseToCompletionDelayHist[t]; }

    statistics::Counter getIncompleteTimes(const MachineType t) const
    { return m_IncompleteTimes[t]; }

  private:
    void issueRequest(PacketPtr pkt, RubyRequestType type);

    void hitCallback(SequencerRequest* srequest, DataBlock& data,
                     bool llscSuccess,
                     const MachineType mach, const bool externalHit,
                     const Cycles initialRequestTime,
                     const Cycles forwardRequestTime,
                     const Cycles firstResponseTime,
                     const bool was_coalesced);

    void recordMissLatency(SequencerRequest* srequest, bool llscSuccess,
                           const MachineType respondingMach,
                           bool isExternalHit, Cycles initialRequestTime,
                           Cycles forwardRequestTime,
                           Cycles firstResponseTime);

    // Private copy constructor and assignment operator
    Sequencer(const Sequencer& obj);
    Sequencer& operator=(const Sequencer& obj);

  protected:
    // RequestTable contains both read and write requests, handles aliasing
    std::unordered_map<Addr, std::list<SequencerRequest>> m_RequestTable;
    // UnadressedRequestTable contains "unaddressed" requests,
    // guaranteed not to alias each other
    std::unordered_map<uint64_t, SequencerRequest> m_UnaddressedRequestTable;

    Cycles m_deadlock_threshold;

    virtual RequestStatus insertRequest(PacketPtr pkt,
                                        RubyRequestType primary_type,
                                        RubyRequestType secondary_type);

  private:
    int m_max_outstanding_requests;

    CacheMemory* m_dataCache_ptr;

    // The cache access latency for top-level caches (L0/L1). These are
    // currently assessed at the beginning of each memory access through the
    // sequencer.
    // TODO: Migrate these latencies into top-level cache controllers.
    Cycles m_data_cache_hit_latency;
    Cycles m_inst_cache_hit_latency;

    // Global outstanding request count, across all request tables
    int m_outstanding_count;
    bool m_deadlock_check_scheduled;

    int m_coreId;

    uint64_t m_unaddressedTransactionCnt;

    bool m_runningGarnetStandalone;

    //! Histogram for number of outstanding requests per cycle.
    statistics::Histogram m_outstandReqHist;

    //! Histogram for holding latency profile of all requests.
    statistics::Histogram m_latencyHist;
    std::vector<statistics::Histogram *> m_typeLatencyHist;

    //! Histogram for holding latency profile of all requests that
    //! hit in the controller connected to this sequencer.
    statistics::Histogram m_hitLatencyHist;
    std::vector<statistics::Histogram *> m_hitTypeLatencyHist;

    //! Histograms for profiling the latencies for requests that
    //! did not required external messages.
    std::vector<statistics::Histogram *> m_hitMachLatencyHist;
    std::vector<std::vector<statistics::Histogram *>> m_hitTypeMachLatencyHist;

    //! Histogram for holding latency profile of all requests that
    //! miss in the controller connected to this sequencer.
    statistics::Histogram m_missLatencyHist;
    std::vector<statistics::Histogram *> m_missTypeLatencyHist;

    //! Histograms for profiling the latencies for requests that
    //! required external messages.
    std::vector<statistics::Histogram *> m_missMachLatencyHist;
    std::vector<std::vector<statistics::Histogram *>>
        m_missTypeMachLatencyHist;

    //! Histograms for recording the breakdown of miss latency
    std::vector<statistics::Histogram *> m_IssueToInitialDelayHist;
    std::vector<statistics::Histogram *> m_InitialToForwardDelayHist;
    std::vector<statistics::Histogram *> m_ForwardToFirstResponseDelayHist;
    std::vector<statistics::Histogram *> m_FirstResponseToCompletionDelayHist;
    std::vector<statistics::Counter> m_IncompleteTimes;

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


    /**
     * Increment the unaddressed transaction counter
     */
    void incrementUnaddressedTransactionCnt();

    /**
     * Generate the current unaddressed transaction ID based on the counter
     * and the Sequencer object's version id.
     */
    uint64_t getCurrentUnaddressedTransactionID() const;

  public:
    /**
     * Searches for cache line address in the global monitor
     * tagged with this Sequencer object's version id.
     *
     * @return a boolean indicating if the line address was found.
     */
    bool llscCheckMonitor(const Addr);


    /**
     * Removes all addresses from the local monitor.
     * This is independent of this Sequencer object's version id.
     */
    void llscClearLocalMonitor();
};

inline std::ostream&
operator<<(std::ostream& out, const Sequencer& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_SYSTEM_SEQUENCER_HH__
