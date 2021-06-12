/*
 * Copyright (c) 2018-2020 ARM Limited
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


#ifndef __MEM_QOS_MEM_SINK_HH__
#define __MEM_QOS_MEM_SINK_HH__

#include <cstdint>
#include <deque>
#include <vector>

#include "base/compiler.hh"
#include "base/types.hh"
#include "mem/abstract_mem.hh"
#include "mem/qos/mem_ctrl.hh"
#include "mem/qport.hh"
#include "params/QoSMemSinkCtrl.hh"
#include "sim/eventq.hh"

namespace gem5
{

struct QoSMemSinkInterfaceParams;

namespace memory
{

GEM5_DEPRECATED_NAMESPACE(QoS, qos);
namespace qos
{

class MemSinkInterface;

/**
 * QoS Memory Sink
 *
 * The QoS Memory Sink is a lightweight memory controller with QoS
 * support. It is meant to provide a QoS aware simple memory system
 * without the need of using a complex DRAM memory controller
 */
class MemSinkCtrl : public MemCtrl
{
  protected:
    /**
     * The Request packets are store in a multiple dequeue structure,
     * based on their QoS priority
     */
    using PacketQueue = std::deque<PacketPtr>;

  private:
    class MemoryPort : public QueuedResponsePort
    {
      private:
        /** reference to parent memory object */
        MemSinkCtrl& mem;

        /** Outgoing packet responses queue */
        RespPacketQueue queue;

      public:
       /**
        * Constructor
        *
        * @param n port name
        * @param m reference to ProfileGen parent object
        */
        MemoryPort(const std::string&, MemSinkCtrl&);

      protected:
       /**
        * Receive a Packet in Atomic mode
        *
        * @param pkt pointer to memory packet
        * @return packet access latency in ticks
        */
        Tick recvAtomic(PacketPtr pkt);

        /**
        * Receive a Packet in Functional mode
        *
        * @param pkt pointer to memory packet
        */
        void recvFunctional(PacketPtr pkt);

        /**
        * Receive a Packet in Timing mode
        *
        * @param pkt pointer to memory packet
        * @return true if the request was accepted
        */
        bool recvTimingReq(PacketPtr pkt);

        /**
         * Gets the configured address ranges for this port
         * @return the configured address ranges for this port
         */
        AddrRangeList getAddrRanges() const;

    };

  public:
    /**
     * QoS Memory Sink Constructor
     *
     * @param p QoS Memory Sink configuration parameters
     */
    MemSinkCtrl(const QoSMemSinkCtrlParams &);

    virtual ~MemSinkCtrl();

    /**
     * Checks and return the Drain state of this SimObject
     * @return current Drain state
     */
    DrainState drain() override;

    /**
     * Getter method to access this memory's response port
     *
     * @param if_name interface name
     * @param idx port ID number
     * @return reference to this memory's response port
     */
    Port &getPort(const std::string &if_name, PortID=InvalidPortID) override;

    /**
     * Initializes this object
     */
    void init() override;

  protected:
    /** Memory between requests latency (ticks) */
    const Tick requestLatency;

    /** Memory response latency (ticks) */
    const Tick responseLatency;

    /** Memory packet size in bytes */
    const uint64_t memoryPacketSize;

    /** Read request packets queue buffer size in #packets */
    const uint64_t readBufferSize;

    /** Write request packets queue buffer size in #packets */
    const uint64_t writeBufferSize;

    /** Memory response port */
    MemoryPort port;

    /**
     * Create pointer to interface of actual media
     */
    MemSinkInterface* const interface;

    /** Read request pending */
    bool retryRdReq;

    /** Write request pending */
    bool retryWrReq;

    /** Next request service time */
    Tick nextRequest;

    struct MemSinkCtrlStats : public statistics::Group
    {
        MemSinkCtrlStats(statistics::Group *parent);

        /** Count the number of read retries */
        statistics::Scalar numReadRetries;

        /** Count the number of write retries */
        statistics::Scalar numWriteRetries;
    };

    /**
     * QoS-aware (per priority) incoming read requests packets queue
     */
    std::vector<PacketQueue> readQueue;

    /**
     * QoS-aware (per priority) incoming read requests packets queue
     */
    std::vector<PacketQueue> writeQueue;

    /**
     * Processes the next Request event according to configured
     * request latency
     */
    void processNextReqEvent();

    /** Event wrapper to schedule next request handler function */
    EventWrapper<
        MemSinkCtrl,
        &MemSinkCtrl::processNextReqEvent> nextReqEvent;

    /**
     * Check if the read queue has room for more entries
     *
     * @param packets The number of entries needed in the read queue
     * @return true if read queue is full, false otherwise
     */
    inline bool readQueueFull(const uint64_t packets) const;

    /**
     * Check if the write queue has room for more entries
     *
     * @param packets  The number of entries needed in the write queue
     * @return true if write queue is full, false otherwise
     */
    inline bool writeQueueFull(const uint64_t packets) const;

    /**
     * Receive a Packet in Atomic mode
     *
     * @param pkt pointer to memory packet
     * @return packet access latency in ticks
     */
    Tick recvAtomic(PacketPtr pkt);

    /**
     * Receive a Packet in Functional mode
     *
     * @param pkt pointer to memory packet
     */
    void recvFunctional(PacketPtr pkt);

   /**
    * Receive a Packet in Timing mode
    *
    * @param pkt pointer to memory packet
    * @return true if the request was accepted
    */
    bool recvTimingReq(PacketPtr pkt);

    MemSinkCtrlStats stats;
};

class MemSinkInterface : public AbstractMemory
{
  public:
    /** Setting a pointer to the interface */
    void setMemCtrl(MemSinkCtrl* _ctrl) { ctrl = _ctrl; };

    /** Pointer to the controller */
    MemSinkCtrl* ctrl;

    MemSinkInterface(const QoSMemSinkInterfaceParams &_p);
};

} // namespace qos
} // namespace memory
} // namespace gem5

#endif // __MEM_QOS_MEM_SINK_HH__
