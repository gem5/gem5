/*
 * Copyright (c) 2017,2019-2023 ARM Limited
 * All rights reserved.
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
 * Copyright (c) 2009-2014 Mark D. Hill and David A. Wood
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

#ifndef __MEM_RUBY_SLICC_INTERFACE_ABSTRACTCONTROLLER_HH__
#define __MEM_RUBY_SLICC_INTERFACE_ABSTRACTCONTROLLER_HH__

#include <exception>
#include <iostream>
#include <string>
#include <unordered_map>

#include "base/addr_range.hh"
#include "base/addr_range_map.hh"
#include "base/callback.hh"
#include "mem/packet.hh"
#include "mem/qport.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/common/DataBlock.hh"
#include "mem/ruby/common/Histogram.hh"
#include "mem/ruby/common/MachineID.hh"
#include "mem/ruby/network/MessageBuffer.hh"
#include "mem/ruby/protocol/AccessPermission.hh"
#include "mem/ruby/system/CacheRecorder.hh"
#include "params/RubyController.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

namespace gem5
{

namespace ruby
{

class Network;
class GPUCoalescer;
class DMASequencer;

// used to communicate that an in_port peeked the wrong message type
class RejectException : public std::exception
{
    virtual const char *
    what() const throw()
    {
        return "Port rejected message based on type";
    }
};

class AbstractController : public ClockedObject, public Consumer
{
  public:
    PARAMS(RubyController);
    AbstractController(const Params &p);
    void init();

    NodeID
    getVersion() const
    {
        return m_machineID.getNum();
    }

    MachineType
    getType() const
    {
        return m_machineID.getType();
    }

    void
    initNetworkPtr(Network *net_ptr)
    {
        m_net_ptr = net_ptr;
    }

    // return instance name
    void blockOnQueue(Addr, MessageBuffer *);
    bool isBlocked(Addr) const;
    void unblock(Addr);
    bool isBlocked(Addr);

    virtual MessageBuffer *getMandatoryQueue() const = 0;
    virtual MessageBuffer *getMemReqQueue() const = 0;
    virtual MessageBuffer *getMemRespQueue() const = 0;

    // That function must be called by controller when dequeuing mem resp queue
    // for memory controller to receive the retry request in time
    void memRespQueueDequeued();
    // Or that function can be called to perform both dequeue and notification
    // at once.
    void dequeueMemRespQueue();

    virtual AccessPermission getAccessPermission(const Addr &addr) = 0;

    virtual void print(std::ostream &out) const = 0;
    virtual void wakeup() = 0;
    virtual void resetStats() = 0;
    virtual void regStats();

    virtual void recordCacheTrace(int cntrl, CacheRecorder *tr) = 0;
    virtual Sequencer *getCPUSequencer() const = 0;
    virtual DMASequencer *getDMASequencer() const = 0;
    virtual GPUCoalescer *getGPUCoalescer() const = 0;

    // This latency is used by the sequencer when enqueueing requests.
    // Different latencies may be used depending on the request type.
    // This is the hit latency unless the top-level cache controller
    // introduces additional cycles in the response path.
    virtual Cycles
    mandatoryQueueLatency(const RubyRequestType &param_type)
    {
        return m_mandatory_queue_latency;
    }

    //! These functions are used by ruby system to read/write the data blocks
    //! that exist with in the controller.
    virtual bool functionalReadBuffers(PacketPtr &) = 0;

    virtual void
    functionalRead(const Addr &addr, PacketPtr)
    {
        panic("functionalRead(Addr,PacketPtr) not implemented");
    }

    //! Functional read that reads only blocks not present in the mask.
    //! Return number of bytes read.
    virtual bool functionalReadBuffers(PacketPtr &, WriteMask &mask) = 0;

    virtual void
    functionalRead(const Addr &addr, PacketPtr pkt, WriteMask &mask)
    {
        panic("functionalRead(Addr,PacketPtr,WriteMask) not implemented");
    }

    void functionalMemoryRead(PacketPtr);
    //! The return value indicates the number of messages written with the
    //! data from the packet.
    virtual int functionalWriteBuffers(PacketPtr &) = 0;
    virtual int functionalWrite(const Addr &addr, PacketPtr) = 0;
    int functionalMemoryWrite(PacketPtr);

    //! Function for enqueuing a prefetch request
    virtual void
    enqueuePrefetch(const Addr &, const RubyRequestType &)
    {
        fatal("Prefetches not implemented!");
    }

    //! Notifies controller of a request coalesced at the sequencer.
    //! By default, it does nothing. Behavior is protocol-specific
    virtual void
    notifyCoalesced(const Addr &addr, const RubyRequestType &type,
                    const RequestPtr &req, const DataBlock &data_blk,
                    const bool &was_miss)
    {}

    //! Function for collating statistics from all the controllers of this
    //! particular type. This function should only be called from the
    //! version 0 of this controller type.
    virtual void
    collateStats()
    {
        fatal("collateStats() should be overridden!");
    }

    //! Initialize the message buffers.
    virtual void initNetQueues() = 0;

    /** A function used to return the port associated with this bus object. */
    Port &getPort(const std::string &if_name, PortID idx = InvalidPortID);

    bool recvTimingResp(PacketPtr pkt);
    Tick recvAtomic(PacketPtr pkt);

    const AddrRangeList &
    getAddrRanges() const
    {
        return addrRanges;
    }

  public:
    MachineID
    getMachineID() const
    {
        return m_machineID;
    }

    RequestorID
    getRequestorId() const
    {
        return m_id;
    }

    statistics::Histogram &
    getDelayHist()
    {
        return stats.delayHistogram;
    }

    statistics::Histogram &
    getDelayVCHist(uint32_t index)
    {
        return *(stats.delayVCHistogram[index]);
    }

    bool
    respondsTo(Addr addr)
    {
        for (auto &range : addrRanges)
            if (range.contains(addr))
                return true;
        return false;
    }

    /**
     * Map an address to the correct MachineID
     *
     * This function querries the network for the NodeID of the
     * destination for a given request using its address and the type
     * of the destination. For example for a request with a given
     * address to a directory it will return the MachineID of the
     * authorative directory.
     *
     * @param the destination address
     * @param the type of the destination
     * @return the MachineID of the destination
     */
    MachineID mapAddressToMachine(Addr addr, MachineType mtype) const;

    /**
     * Maps an address to the correct dowstream MachineID (i.e. the component
     * in the next level of the cache hierarchy towards memory)
     *
     * This function uses the local list of possible destinations instead of
     * querying the network.
     *
     * @param the destination address
     * @param the type of the destination (optional)
     * @return the MachineID of the destination
     */
    MachineID
    mapAddressToDownstreamMachine(Addr addr,
                                  MachineType mtype = MachineType_NUM) const;

    /** List of downstream destinations (towards memory) */
    const NetDest &
    allDownstreamDest() const
    {
        return downstreamDestinations;
    }

    /** List of upstream destinations (towards the CPU) */
    const NetDest &
    allUpstreamDest() const
    {
        return upstreamDestinations;
    }

  protected:
    //! Profiles original cache requests including PUTs
    void profileRequest(const std::string &request);
    //! Profiles the delay associated with messages.
    void profileMsgDelay(uint32_t virtualNetwork, Cycles delay);

    // Tracks outstanding transactions for latency profiling
    struct TransMapPair
    {
        unsigned transaction;
        unsigned state;
        Tick time;
    };

    std::unordered_map<Addr, TransMapPair> m_inTransAddressed;
    std::unordered_map<Addr, TransMapPair> m_outTransAddressed;

    std::unordered_map<Addr, TransMapPair> m_inTransUnaddressed;
    std::unordered_map<Addr, TransMapPair> m_outTransUnaddressed;

    /**
     * Profiles an event that initiates a protocol transactions for a specific
     * line (e.g. events triggered by incoming request messages).
     * A histogram with the latency of the transactions is generated for
     * all combinations of trigger event, initial state, and final state.
     * This function also supports "unaddressed" transactions,
     * those not associated with an address in memory but
     * instead associated with a unique ID.
     *
     * @param addr address of the line, or unique transaction ID
     * @param type event that started the transaction
     * @param initialState state of the line before the transaction
     * @param isAddressed is addr a line address or a unique ID
     */
    template <typename EventType, typename StateType>
    void
    incomingTransactionStart(Addr addr, EventType type, StateType initialState,
                             bool retried, bool isAddressed = true)
    {
        auto &m_inTrans =
            isAddressed ? m_inTransAddressed : m_inTransUnaddressed;
        assert(m_inTrans.find(addr) == m_inTrans.end());
        m_inTrans[addr] = { type, initialState, curTick() };
        if (retried)
            ++(*stats.inTransRetryCnt[type]);
    }

    /**
     * Profiles an event that ends a transaction.
     * This function also supports "unaddressed" transactions,
     * those not associated with an address in memory but
     * instead associated with a unique ID.
     *
     * @param addr address or unique ID with an outstanding transaction
     * @param finalState state of the line after the transaction
     * @param isAddressed is addr a line address or a unique ID
     */
    template <typename StateType>
    void
    incomingTransactionEnd(Addr addr, StateType finalState,
                           bool isAddressed = true)
    {
        auto &m_inTrans =
            isAddressed ? m_inTransAddressed : m_inTransUnaddressed;
        auto iter = m_inTrans.find(addr);
        assert(iter != m_inTrans.end());
        auto &trans = iter->second;

        auto stat_iter_ev = stats.inTransStateChanges.find(trans.transaction);
        gem5_assert(stat_iter_ev != stats.inTransStateChanges.end(),
                    "%s: event type=%d not marked as in_trans in SLICC",
                    name(), trans.transaction);

        auto stat_iter_state = stat_iter_ev->second.find(trans.state);
        gem5_assert(stat_iter_state != stat_iter_ev->second.end(),
                    "%s: event type=%d has no transition from state=%d",
                    name(), trans.transaction, trans.state);

        ++(*stat_iter_state->second[(unsigned)finalState]);

        stats.inTransLatHist[iter->second.transaction]->sample(
            ticksToCycles(curTick() - trans.time));

        m_inTrans.erase(iter);
    }

    /**
     * Profiles an event that initiates a transaction in a peer controller
     * (e.g. an event that sends a request message)
     * This function also supports "unaddressed" transactions,
     * those not associated with an address in memory but
     * instead associated with a unique ID.
     *
     * @param addr address of the line or a unique transaction ID
     * @param type event that started the transaction
     * @param isAddressed is addr a line address or a unique ID
     */
    template <typename EventType>
    void
    outgoingTransactionStart(Addr addr, EventType type,
                             bool isAddressed = true)
    {
        auto &m_outTrans =
            isAddressed ? m_outTransAddressed : m_outTransUnaddressed;
        assert(m_outTrans.find(addr) == m_outTrans.end());
        m_outTrans[addr] = { type, 0, curTick() };
    }

    /**
     * Profiles the end of an outgoing transaction.
     * (e.g. receiving the response for a requests)
     * This function also supports "unaddressed" transactions,
     * those not associated with an address in memory but
     * instead associated with a unique ID.
     *
     * @param addr address of the line with an outstanding transaction
     * @param isAddressed is addr a line address or a unique ID
     */
    void
    outgoingTransactionEnd(Addr addr, bool retried, bool isAddressed = true)
    {
        auto &m_outTrans =
            isAddressed ? m_outTransAddressed : m_outTransUnaddressed;
        auto iter = m_outTrans.find(addr);
        assert(iter != m_outTrans.end());
        auto &trans = iter->second;

        auto stat_iter = stats.outTransLatHist.find(trans.transaction);
        gem5_assert(stat_iter != stats.outTransLatHist.end(),
                    "%s: event type=%d not marked as out_trans in SLICC",
                    name(), trans.transaction);

        stat_iter->second->sample(ticksToCycles(curTick() - trans.time));
        if (retried)
            ++(*stats.outTransRetryCnt[trans.transaction]);
        m_outTrans.erase(iter);
    }

    void stallBuffer(MessageBuffer *buf, Addr addr);
    void wakeUpBuffer(MessageBuffer *buf, Addr addr);
    void wakeUpBuffers(Addr addr);
    void wakeUpAllBuffers(Addr addr);
    void wakeUpAllBuffers();
    bool serviceMemoryQueue();

    /**
     * Functions needed by CacheAccessor. These are implemented in SLICC,
     * thus the const& for all args to match the generated code.
     */
    virtual bool
    inCache(const Addr &addr, const bool &is_secure)
    {
        fatal("inCache: prefetching not supported");
        return false;
    }

    virtual bool
    hasBeenPrefetched(const Addr &addr, const bool &is_secure)
    {
        fatal("hasBeenPrefetched: prefetching not supported");
        return false;
    }

    virtual bool
    hasBeenPrefetched(const Addr &addr, const bool &is_secure,
                      const RequestorID &requestor)
    {
        fatal("hasBeenPrefetched: prefetching not supported");
        return false;
    }

    virtual bool
    inMissQueue(const Addr &addr, const bool &is_secure)
    {
        fatal("inMissQueue: prefetching not supported");
        return false;
    }

    virtual bool
    coalesce()
    {
        fatal("coalesce: prefetching not supported");
        return false;
    }

    friend class RubyPrefetcherProxy;

  protected:
    const NodeID m_version;
    MachineID m_machineID;
    const NodeID m_clusterID;

    // RequestorID used by some components of gem5.
    const RequestorID m_id;

    Network *m_net_ptr;
    bool m_is_blocking;
    std::map<Addr, MessageBuffer *> m_block_map;

    typedef std::vector<MessageBuffer *> MsgVecType;
    typedef std::set<MessageBuffer *> MsgBufType;
    typedef std::map<Addr, MsgVecType *> WaitingBufType;
    WaitingBufType m_waiting_buffers;

    unsigned int m_in_ports;
    unsigned int m_cur_in_port;
    const int m_number_of_TBEs;
    const int m_transitions_per_cycle;
    const unsigned int m_buffer_size;
    Cycles m_recycle_latency;
    const Cycles m_mandatory_queue_latency;
    bool m_waiting_mem_retry;
    bool m_mem_ctrl_waiting_retry;

    /**
     * Port that forwards requests and receives responses from the
     * memory controller.
     */
    class MemoryPort : public RequestPort
    {
      private:
        // Controller that operates this port.
        AbstractController *controller;

      public:
        MemoryPort(const std::string &_name, AbstractController *_controller,
                   PortID id = InvalidPortID);

      protected:
        // Function for receiving a timing response from the peer port.
        // Currently the pkt is handed to the coherence controller
        // associated with this port.
        bool recvTimingResp(PacketPtr pkt);

        void recvReqRetry();
    };

    /* Request port to the memory controller. */
    MemoryPort memoryPort;

    // State that is stored in packets sent to the memory controller.
    struct SenderState : public Packet::SenderState
    {
        // Id of the machine from which the request originated.
        MachineID id;

        SenderState(MachineID _id) : id(_id) {}
    };

  private:
    /** The address range to which the controller responds on the CPU side. */
    const AddrRangeList addrRanges;

    std::unordered_map<MachineType, AddrRangeMap<MachineID, 3>>
        downstreamAddrMap;

    NetDest downstreamDestinations;
    NetDest upstreamDestinations;

    void sendRetryRespToMem();
    MemberEventWrapper<&AbstractController::sendRetryRespToMem>
        mRetryRespEvent;

  public:
    struct ControllerStats : public statistics::Group
    {
        ControllerStats(statistics::Group *parent);

        // Initialized by the SLICC compiler for all events with the
        // "in_trans" property.
        // Only histograms with samples will appear in the stats
        std::unordered_map<unsigned, statistics::Histogram *> inTransLatHist;
        std::unordered_map<unsigned, statistics::Scalar *> inTransRetryCnt;
        // Initialized by the SLICC compiler for all combinations of events
        // with the "in_trans" property, potential initial states, and
        // potential final states. Potential initial states are states that
        // appear in transitions triggered by that event. Currently all states
        // are considered as potential final states.
        std::unordered_map<
            unsigned,
            std::unordered_map<unsigned, std::vector<statistics::Scalar *>>>
            inTransStateChanges;

        // Initialized by the SLICC compiler for all events with the
        // "out_trans" property.
        // Only histograms with samples will appear in the stats.
        std::unordered_map<unsigned, statistics::Histogram *> outTransLatHist;
        std::unordered_map<unsigned, statistics::Scalar *> outTransRetryCnt;

        //! Counter for the number of cycles when the transitions carried out
        //! were equal to the maximum allowed
        statistics::Scalar fullyBusyCycles;

        //! Histogram for profiling delay for the messages this controller
        //! cares for
        statistics::Histogram delayHistogram;
        std::vector<statistics::Histogram *> delayVCHistogram;
    } stats;
};

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_SLICC_INTERFACE_ABSTRACTCONTROLLER_HH__
