/*
 * Copyright (c) 2013-2015 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Sooraj Puthoor
 */

#ifndef __MEM_RUBY_SYSTEM_GPU_COALESCER_HH__
#define __MEM_RUBY_SYSTEM_GPU_COALESCER_HH__

#include <iostream>
#include <unordered_map>

#include "base/statistics.hh"
#include "mem/protocol/HSAScope.hh"
#include "mem/protocol/HSASegment.hh"
#include "mem/protocol/PrefetchBit.hh"
#include "mem/protocol/RubyAccessMode.hh"
#include "mem/protocol/RubyRequestType.hh"
#include "mem/protocol/SequencerRequestType.hh"
#include "mem/request.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/system/Sequencer.hh"

class DataBlock;
class CacheMsg;
class MachineID;
class CacheMemory;

class RubyGPUCoalescerParams;

HSAScope reqScopeToHSAScope(Request* req);
HSASegment reqSegmentToHSASegment(Request* req);

struct GPUCoalescerRequest
{
    PacketPtr pkt;
    RubyRequestType m_type;
    Cycles issue_time;

    GPUCoalescerRequest(PacketPtr _pkt, RubyRequestType _m_type,
                        Cycles _issue_time)
        : pkt(_pkt), m_type(_m_type), issue_time(_issue_time)
    {}
};

class RequestDesc
{
  public:
    RequestDesc(PacketPtr pkt, RubyRequestType p_type, RubyRequestType s_type)
        : pkt(pkt), primaryType(p_type), secondaryType(s_type)
    {
    }

    RequestDesc() : pkt(nullptr), primaryType(RubyRequestType_NULL),
        secondaryType(RubyRequestType_NULL)
    {
    }

    PacketPtr pkt;
    RubyRequestType primaryType;
    RubyRequestType secondaryType;
};

std::ostream& operator<<(std::ostream& out, const GPUCoalescerRequest& obj);

class GPUCoalescer : public RubyPort
{
  public:
    typedef RubyGPUCoalescerParams Params;
    GPUCoalescer(const Params *);
    ~GPUCoalescer();

    // Public Methods
    void wakeup(); // Used only for deadlock detection

    void printProgress(std::ostream& out) const;
    void resetStats();
    void collateStats();
    void regStats();

    void writeCallback(Addr address, DataBlock& data);

    void writeCallback(Addr address,
                       MachineType mach,
                       DataBlock& data);

    void writeCallback(Addr address,
                       MachineType mach,
                       DataBlock& data,
                       Cycles initialRequestTime,
                       Cycles forwardRequestTime,
                       Cycles firstResponseTime,
                       bool isRegion);

    void writeCallback(Addr address,
                       MachineType mach,
                       DataBlock& data,
                       Cycles initialRequestTime,
                       Cycles forwardRequestTime,
                       Cycles firstResponseTime);

    void readCallback(Addr address, DataBlock& data);

    void readCallback(Addr address,
                      MachineType mach,
                      DataBlock& data);

    void readCallback(Addr address,
                      MachineType mach,
                      DataBlock& data,
                      Cycles initialRequestTime,
                      Cycles forwardRequestTime,
                      Cycles firstResponseTime);

    void readCallback(Addr address,
                      MachineType mach,
                      DataBlock& data,
                      Cycles initialRequestTime,
                      Cycles forwardRequestTime,
                      Cycles firstResponseTime,
                      bool isRegion);
    /* atomics need their own callback because the data
       might be const coming from SLICC */
    void atomicCallback(Addr address,
                        MachineType mach,
                        const DataBlock& data);

    void recordCPReadCallBack(MachineID myMachID, MachineID senderMachID);
    void recordCPWriteCallBack(MachineID myMachID, MachineID senderMachID);

    // Alternate implementations in VIPER Coalescer
    virtual RequestStatus makeRequest(PacketPtr pkt);

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

    bool empty() const;

    void print(std::ostream& out) const;
    void checkCoherence(Addr address);

    void markRemoved();
    void removeRequest(GPUCoalescerRequest* request);
    void evictionCallback(Addr address);
    void completeIssue();

    void insertKernel(int wavefront_id, PacketPtr pkt);

    void recordRequestType(SequencerRequestType requestType);
    Stats::Histogram& getOutstandReqHist() { return m_outstandReqHist; }

    Stats::Histogram& getLatencyHist() { return m_latencyHist; }
    Stats::Histogram& getTypeLatencyHist(uint32_t t)
    { return *m_typeLatencyHist[t]; }

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

  // Changed to protected to enable inheritance by VIPER Coalescer
  protected:
    bool tryCacheAccess(Addr addr, RubyRequestType type,
                        Addr pc, RubyAccessMode access_mode,
                        int size, DataBlock*& data_ptr);
    // Alternate implementations in VIPER Coalescer
    virtual void issueRequest(PacketPtr pkt, RubyRequestType type);

    void kernelCallback(int wavfront_id);

    void hitCallback(GPUCoalescerRequest* request,
                     MachineType mach,
                     DataBlock& data,
                     bool success,
                     Cycles initialRequestTime,
                     Cycles forwardRequestTime,
                     Cycles firstResponseTime,
                     bool isRegion);
    void recordMissLatency(GPUCoalescerRequest* request,
                           MachineType mach,
                           Cycles initialRequestTime,
                           Cycles forwardRequestTime,
                           Cycles firstResponseTime,
                           bool success, bool isRegion);
    void completeHitCallback(std::vector<PacketPtr> & mylist, int len);
    PacketPtr mapAddrToPkt(Addr address);


    RequestStatus getRequestStatus(PacketPtr pkt,
                                   RubyRequestType request_type);
    bool insertRequest(PacketPtr pkt, RubyRequestType request_type);

    bool handleLlsc(Addr address, GPUCoalescerRequest* request);

    EventFunctionWrapper issueEvent;


  // Changed to protected to enable inheritance by VIPER Coalescer
  protected:
    int m_max_outstanding_requests;
    int m_deadlock_threshold;

    CacheMemory* m_dataCache_ptr;
    CacheMemory* m_instCache_ptr;

    // The cache access latency for this GPU data cache. This is assessed at the
    // beginning of each access. This should be very similar to the
    // implementation in Sequencer() as this is very much like a Sequencer
    Cycles m_data_cache_hit_latency;

    // We need to track both the primary and secondary request types.
    // The secondary request type comprises a subset of RubyRequestTypes that
    // are understood by the L1 Controller. A primary request type can be any
    // RubyRequestType.
    typedef std::unordered_map<Addr, std::vector<RequestDesc>> CoalescingTable;
    CoalescingTable reqCoalescer;
    std::vector<Addr> newRequests;

    typedef std::unordered_map<Addr, GPUCoalescerRequest*> RequestTable;
    RequestTable m_writeRequestTable;
    RequestTable m_readRequestTable;
    // Global outstanding request count, across all request tables
    int m_outstanding_count;
    bool m_deadlock_check_scheduled;
    std::unordered_map<int, PacketPtr> kernelEndList;
    std::vector<int> newKernelEnds;

    int m_store_waiting_on_load_cycles;
    int m_store_waiting_on_store_cycles;
    int m_load_waiting_on_store_cycles;
    int m_load_waiting_on_load_cycles;

    bool m_runningGarnetStandalone;

    EventFunctionWrapper deadlockCheckEvent;
    bool assumingRfOCoherence;

    // m5 style stats for TCP hit/miss counts
    Stats::Scalar GPU_TCPLdHits;
    Stats::Scalar GPU_TCPLdTransfers;
    Stats::Scalar GPU_TCCLdHits;
    Stats::Scalar GPU_LdMiss;

    Stats::Scalar GPU_TCPStHits;
    Stats::Scalar GPU_TCPStTransfers;
    Stats::Scalar GPU_TCCStHits;
    Stats::Scalar GPU_StMiss;

    Stats::Scalar CP_TCPLdHits;
    Stats::Scalar CP_TCPLdTransfers;
    Stats::Scalar CP_TCCLdHits;
    Stats::Scalar CP_LdMiss;

    Stats::Scalar CP_TCPStHits;
    Stats::Scalar CP_TCPStTransfers;
    Stats::Scalar CP_TCCStHits;
    Stats::Scalar CP_StMiss;

    //! Histogram for number of outstanding requests per cycle.
    Stats::Histogram m_outstandReqHist;

    //! Histogram for holding latency profile of all requests.
    Stats::Histogram m_latencyHist;
    std::vector<Stats::Histogram *> m_typeLatencyHist;

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

private:
    // Private copy constructor and assignment operator
    GPUCoalescer(const GPUCoalescer& obj);
    GPUCoalescer& operator=(const GPUCoalescer& obj);
};

inline std::ostream&
operator<<(std::ostream& out, const GPUCoalescer& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_SYSTEM_GPU_COALESCER_HH__
