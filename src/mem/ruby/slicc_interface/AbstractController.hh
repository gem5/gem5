/*
 * Copyright (c) 2009 Mark D. Hill and David A. Wood
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

#include <iostream>
#include <string>

#include "base/callback.hh"
#include "mem/protocol/AccessPermission.hh"
#include "mem/ruby/buffers/MessageBuffer.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/common/DataBlock.hh"
#include "mem/ruby/common/Histogram.hh"
#include "mem/ruby/network/Network.hh"
#include "mem/ruby/recorder/CacheRecorder.hh"
#include "mem/ruby/system/MachineID.hh"
#include "mem/packet.hh"
#include "params/RubyController.hh"
#include "sim/clocked_object.hh"

class Network;

class AbstractController : public ClockedObject, public Consumer
{
  public:
    typedef RubyControllerParams Params;
    AbstractController(const Params *p);
    void init();
    const Params *params() const { return (const Params *)_params; }

    const NodeID getVersion() const { return m_machineID.getNum(); }
    const MachineType getType() const { return m_machineID.getType(); }

    void initNetworkPtr(Network* net_ptr) { m_net_ptr = net_ptr; }

    // return instance name
    void blockOnQueue(Address, MessageBuffer*);
    void unblock(Address);

    virtual MessageBuffer* getMandatoryQueue() const = 0;
    virtual const std::string toString() const = 0;  // returns text version of
                                                     // controller type
    virtual AccessPermission getAccessPermission(const Address& addr) = 0;
    virtual DataBlock& getDataBlock(const Address& addr) = 0;

    virtual void print(std::ostream & out) const = 0;
    virtual void wakeup() = 0;
    virtual void resetStats() = 0;
    virtual void regStats();

    virtual void recordCacheTrace(int cntrl, CacheRecorder* tr) = 0;
    virtual Sequencer* getSequencer() const = 0;

    //! These functions are used by ruby system to read/write the message
    //! queues that exist with in the controller.
    //! The boolean return value indicates if the read was performed
    //! successfully.
    virtual bool functionalReadBuffers(PacketPtr&) = 0;
    //! The return value indicates the number of messages written with the
    //! data from the packet.
    virtual uint32_t functionalWriteBuffers(PacketPtr&) = 0;

    //! Function for enqueuing a prefetch request
    virtual void enqueuePrefetch(const Address&, const RubyRequestType&)
    { fatal("Prefetches not implemented!");}

    //! Function for collating statistics from all the controllers of this
    //! particular type. This function should only be called from the
    //! version 0 of this controller type.
    virtual void collateStats()
    {fatal("collateStats() should be overridden!");}

  public:
    MachineID getMachineID() const { return m_machineID; }

    Stats::Histogram& getDelayHist() { return m_delayHistogram; }
    Stats::Histogram& getDelayVCHist(uint32_t index)
    { return *(m_delayVCHistogram[index]); }

    MessageBuffer *getPeerQueue(uint32_t pid)
    {
        std::map<uint32_t, MessageBuffer *>::iterator it =
                                        peerQueueMap.find(pid);
        assert(it != peerQueueMap.end());
        return (*it).second;
    }

  protected:
    //! Profiles original cache requests including PUTs
    void profileRequest(const std::string &request);
    //! Profiles the delay associated with messages.
    void profileMsgDelay(uint32_t virtualNetwork, Cycles delay);

    //! Function for connecting peer controllers
    void connectWithPeer(AbstractController *);
    virtual void getQueuesFromPeer(AbstractController *)
    { fatal("getQueuesFromPeer() should be called only if implemented!"); }

    void stallBuffer(MessageBuffer* buf, Address addr);
    void wakeUpBuffers(Address addr);
    void wakeUpAllBuffers(Address addr);
    void wakeUpAllBuffers();

  protected:
    NodeID m_version;
    MachineID m_machineID;
    NodeID m_clusterID;

    Network* m_net_ptr;
    bool m_is_blocking;
    std::map<Address, MessageBuffer*> m_block_map;

    typedef std::vector<MessageBuffer*> MsgVecType;
    typedef std::map< Address, MsgVecType* > WaitingBufType;
    WaitingBufType m_waiting_buffers;

    unsigned int m_in_ports;
    unsigned int m_cur_in_port;
    int m_number_of_TBEs;
    int m_transitions_per_cycle;
    unsigned int m_buffer_size;
    Cycles m_recycle_latency;

    //! Map from physical network number to the Message Buffer.
    std::map<uint32_t, MessageBuffer*> peerQueueMap;

    //! Counter for the number of cycles when the transitions carried out
    //! were equal to the maximum allowed
    Stats::Scalar m_fully_busy_cycles;

    //! Histogram for profiling delay for the messages this controller
    //! cares for
    Stats::Histogram m_delayHistogram;
    std::vector<Stats::Histogram *> m_delayVCHistogram;

    //! Callback class used for collating statistics from all the
    //! controller of this type.
    class StatsCallback : public Callback
    {
      private:
        AbstractController *ctr;

      public:
        virtual ~StatsCallback() {}
        StatsCallback(AbstractController *_ctr) : ctr(_ctr) {}
        void process() {ctr->collateStats();}
    };
};

#endif // __MEM_RUBY_SLICC_INTERFACE_ABSTRACTCONTROLLER_HH__
