/*
 * Copyright (c) 2017,2019 ARM Limited
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

#include "base/addr_range.hh"
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

class Network;
class GPUCoalescer;
class DMASequencer;

// used to communicate that an in_port peeked the wrong message type
class RejectException: public std::exception
{
    virtual const char* what() const throw()
    { return "Port rejected message based on type"; }
};

class AbstractController : public ClockedObject, public Consumer
{
  public:
    typedef RubyControllerParams Params;
    AbstractController(const Params *p);
    void init();
    const Params *params() const { return (const Params *)_params; }

    NodeID getVersion() const { return m_machineID.getNum(); }
    MachineType getType() const { return m_machineID.getType(); }

    void initNetworkPtr(Network* net_ptr) { m_net_ptr = net_ptr; }

    // return instance name
    void blockOnQueue(Addr, MessageBuffer*);
    bool isBlocked(Addr) const;
    void unblock(Addr);
    bool isBlocked(Addr);

    virtual MessageBuffer* getMandatoryQueue() const = 0;
    virtual MessageBuffer* getMemReqQueue() const = 0;
    virtual MessageBuffer* getMemRespQueue() const = 0;
    virtual AccessPermission getAccessPermission(const Addr &addr) = 0;

    virtual void print(std::ostream & out) const = 0;
    virtual void wakeup() = 0;
    virtual void resetStats() = 0;
    virtual void regStats();

    virtual void recordCacheTrace(int cntrl, CacheRecorder* tr) = 0;
    virtual Sequencer* getCPUSequencer() const = 0;
    virtual DMASequencer* getDMASequencer() const = 0;
    virtual GPUCoalescer* getGPUCoalescer() const = 0;

    // This latency is used by the sequencer when enqueueing requests.
    // Different latencies may be used depending on the request type.
    // This is the hit latency unless the top-level cache controller
    // introduces additional cycles in the response path.
    virtual Cycles mandatoryQueueLatency(const RubyRequestType& param_type)
    { return m_mandatory_queue_latency; }

    //! These functions are used by ruby system to read/write the data blocks
    //! that exist with in the controller.
    virtual bool functionalReadBuffers(PacketPtr&) = 0;
    virtual void functionalRead(const Addr &addr, PacketPtr) = 0;
    void functionalMemoryRead(PacketPtr);
    //! The return value indicates the number of messages written with the
    //! data from the packet.
    virtual int functionalWriteBuffers(PacketPtr&) = 0;
    virtual int functionalWrite(const Addr &addr, PacketPtr) = 0;
    int functionalMemoryWrite(PacketPtr);

    //! Function for enqueuing a prefetch request
    virtual void enqueuePrefetch(const Addr &, const RubyRequestType&)
    { fatal("Prefetches not implemented!");}

    //! Function for collating statistics from all the controllers of this
    //! particular type. This function should only be called from the
    //! version 0 of this controller type.
    virtual void collateStats()
    {fatal("collateStats() should be overridden!");}

    //! Initialize the message buffers.
    virtual void initNetQueues() = 0;

    /** A function used to return the port associated with this bus object. */
    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID);

    void recvTimingResp(PacketPtr pkt);
    Tick recvAtomic(PacketPtr pkt);

    const AddrRangeList &getAddrRanges() const { return addrRanges; }

  public:
    MachineID getMachineID() const { return m_machineID; }
    RequestorID getRequestorId() const { return m_id; }

    Stats::Histogram& getDelayHist() { return m_delayHistogram; }
    Stats::Histogram& getDelayVCHist(uint32_t index)
    { return *(m_delayVCHistogram[index]); }

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

  protected:
    //! Profiles original cache requests including PUTs
    void profileRequest(const std::string &request);
    //! Profiles the delay associated with messages.
    void profileMsgDelay(uint32_t virtualNetwork, Cycles delay);

    void stallBuffer(MessageBuffer* buf, Addr addr);
    void wakeUpBuffers(Addr addr);
    void wakeUpAllBuffers(Addr addr);
    void wakeUpAllBuffers();
    bool serviceMemoryQueue();

  protected:
    const NodeID m_version;
    MachineID m_machineID;
    const NodeID m_clusterID;

    // RequestorID used by some components of gem5.
    const RequestorID m_id;

    Network *m_net_ptr;
    bool m_is_blocking;
    std::map<Addr, MessageBuffer*> m_block_map;

    typedef std::vector<MessageBuffer*> MsgVecType;
    typedef std::set<MessageBuffer*> MsgBufType;
    typedef std::map<Addr, MsgVecType* > WaitingBufType;
    WaitingBufType m_waiting_buffers;

    unsigned int m_in_ports;
    unsigned int m_cur_in_port;
    const int m_number_of_TBEs;
    const int m_transitions_per_cycle;
    const unsigned int m_buffer_size;
    Cycles m_recycle_latency;
    const Cycles m_mandatory_queue_latency;

    //! Counter for the number of cycles when the transitions carried out
    //! were equal to the maximum allowed
    Stats::Scalar m_fully_busy_cycles;

    //! Histogram for profiling delay for the messages this controller
    //! cares for
    Stats::Histogram m_delayHistogram;
    std::vector<Stats::Histogram *> m_delayVCHistogram;

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

        SenderState(MachineID _id) : id(_id)
        {}
    };

  private:
    /** The address range to which the controller responds on the CPU side. */
    const AddrRangeList addrRanges;
};

#endif // __MEM_RUBY_SLICC_INTERFACE_ABSTRACTCONTROLLER_HH__
