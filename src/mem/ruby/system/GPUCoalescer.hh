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
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
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
 */

#ifndef __MEM_RUBY_SYSTEM_GPU_COALESCER_HH__
#define __MEM_RUBY_SYSTEM_GPU_COALESCER_HH__

#include <iostream>
#include <unordered_map>

#include "base/statistics.hh"
#include "mem/request.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/protocol/HSAScope.hh"
#include "mem/ruby/protocol/HSASegment.hh"
#include "mem/ruby/protocol/PrefetchBit.hh"
#include "mem/ruby/protocol/RubyAccessMode.hh"
#include "mem/ruby/protocol/RubyRequestType.hh"
#include "mem/ruby/protocol/SequencerRequestType.hh"
#include "mem/ruby/system/Sequencer.hh"
#include "mem/token_port.hh"

class DataBlock;
class CacheMsg;
class MachineID;
class CacheMemory;

class RubyGPUCoalescerParams;

HSAScope reqScopeToHSAScope(const RequestPtr &req);
HSASegment reqSegmentToHSASegment(const RequestPtr &req);

// List of packets that belongs to a specific instruction.
typedef std::list<PacketPtr> PerInstPackets;

class UncoalescedTable
{
  public:
    UncoalescedTable(GPUCoalescer *gc);
    ~UncoalescedTable() {}

    void insertPacket(PacketPtr pkt);
    bool packetAvailable();
    void printRequestTable(std::stringstream& ss);

    // Returns a pointer to the list of packets corresponding to an
    // instruction in the instruction map or nullptr if there are no
    // instructions at the offset.
    PerInstPackets* getInstPackets(int offset);
    void updateResources();

    // Check if a packet hasn't been removed from instMap in too long.
    // Panics if a deadlock is detected and returns nothing otherwise.
    void checkDeadlock(Tick threshold);

  private:
    GPUCoalescer *coalescer;

    // Maps an instructions unique sequence number to a queue of packets
    // which need responses. This data structure assumes the sequence number
    // is monotonically increasing (which is true for CU class) in order to
    // issue packets in age order.
    std::map<uint64_t, PerInstPackets> instMap;
};

class CoalescedRequest
{
  public:
    CoalescedRequest(uint64_t _seqNum)
        : seqNum(_seqNum), issueTime(Cycles(0)),
          rubyType(RubyRequestType_NULL)
    {}
    ~CoalescedRequest() {}

    void insertPacket(PacketPtr pkt) { pkts.push_back(pkt); }
    void setSeqNum(uint64_t _seqNum) { seqNum = _seqNum; }
    void setIssueTime(Cycles _issueTime) { issueTime = _issueTime; }
    void setRubyType(RubyRequestType type) { rubyType = type; }

    uint64_t getSeqNum() const { return seqNum; }
    PacketPtr getFirstPkt() const { return pkts[0]; }
    Cycles getIssueTime() const { return issueTime; }
    RubyRequestType getRubyType() const { return rubyType; }
    std::vector<PacketPtr>& getPackets() { return pkts; }

  private:
    uint64_t seqNum;
    Cycles issueTime;
    RubyRequestType rubyType;
    std::vector<PacketPtr> pkts;
};

class GPUCoalescer : public RubyPort
{
  public:
    class GMTokenPort : public TokenSlavePort
    {
      public:
        GMTokenPort(const std::string& name, ClockedObject *owner,
                    PortID id = InvalidPortID)
            : TokenSlavePort(name, owner, id)
        { }
        ~GMTokenPort() { }

      protected:
        Tick recvAtomic(PacketPtr) { return Tick(0); }
        void recvFunctional(PacketPtr) { }
        bool recvTimingReq(PacketPtr) { return false; }
        AddrRangeList getAddrRanges() const
        {
            AddrRangeList ranges;
            return ranges;
        }
    };

    typedef RubyGPUCoalescerParams Params;
    GPUCoalescer(const Params *);
    ~GPUCoalescer();

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    // Public Methods
    void wakeup(); // Used only for deadlock detection
    void printRequestTable(std::stringstream& ss);

    void printProgress(std::ostream& out) const;
    void resetStats() override;
    void collateStats();
    void regStats() override;

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
    virtual RequestStatus makeRequest(PacketPtr pkt) override;

    int outstandingCount() const override { return m_outstanding_count; }

    bool
    isDeadlockEventScheduled() const override
    {
        return deadlockCheckEvent.scheduled();
    }

    void
    descheduleDeadlockEvent() override
    {
        deschedule(deadlockCheckEvent);
    }

    bool empty() const;

    void print(std::ostream& out) const;

    void evictionCallback(Addr address);
    void completeIssue();

    void insertKernel(int wavefront_id, PacketPtr pkt);

    GMTokenPort& getGMTokenPort() { return gmTokenPort; }

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
    virtual void issueRequest(CoalescedRequest* crequest);

    void kernelCallback(int wavfront_id);

    void hitCallback(CoalescedRequest* crequest,
                     MachineType mach,
                     DataBlock& data,
                     bool success,
                     Cycles initialRequestTime,
                     Cycles forwardRequestTime,
                     Cycles firstResponseTime,
                     bool isRegion);
    void recordMissLatency(CoalescedRequest* crequest,
                           MachineType mach,
                           Cycles initialRequestTime,
                           Cycles forwardRequestTime,
                           Cycles firstResponseTime,
                           bool success, bool isRegion);
    void completeHitCallback(std::vector<PacketPtr> & mylist);


    virtual RubyRequestType getRequestType(PacketPtr pkt);

    // Attempt to remove a packet from the uncoalescedTable and coalesce
    // with a previous request from the same instruction. If there is no
    // previous instruction and the max number of outstanding requests has
    // not be reached, a new coalesced request is created and added to the
    // "target" list of the coalescedTable.
    bool coalescePacket(PacketPtr pkt);

    EventFunctionWrapper issueEvent;


  // Changed to protected to enable inheritance by VIPER Coalescer
  protected:
    int m_max_outstanding_requests;
    Cycles m_deadlock_threshold;

    CacheMemory* m_dataCache_ptr;
    CacheMemory* m_instCache_ptr;

    // coalescingWindow is the maximum number of instructions that are
    // allowed to be coalesced in a single cycle.
    int coalescingWindow;

    // The uncoalescedTable contains several "columns" which hold memory
    // request packets for an instruction. The maximum size is the number of
    // columns * the wavefront size.
    UncoalescedTable uncoalescedTable;

    // An MSHR-like struct for holding coalesced requests. The requests in
    // this table may or may not be outstanding in the memory hierarchy. The
    // maximum size is equal to the maximum outstanding requests for a CU
    // (typically the number of blocks in TCP). If there are duplicates of
    // an address, the are serviced in age order.
    std::map<Addr, std::deque<CoalescedRequest*>> coalescedTable;

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
    // Token port is used to send/receive tokens to/from GPU's global memory
    // pipeline across the port boundary. There is one per <wave size> data
    // ports in the CU.
    GMTokenPort gmTokenPort;

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
