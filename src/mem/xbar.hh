/*
 * Copyright (c) 2011-2015 ARM Limited
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
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 *
 * Authors: Ron Dreslinski
 *          Ali Saidi
 *          Andreas Hansson
 *          William Wang
 */

/**
 * @file
 * Declaration of an abstract crossbar base class.
 */

#ifndef __MEM_XBAR_HH__
#define __MEM_XBAR_HH__

#include <deque>
#include <unordered_map>

#include "base/addr_range_map.hh"
#include "base/types.hh"
#include "mem/mem_object.hh"
#include "mem/qport.hh"
#include "params/BaseXBar.hh"
#include "sim/stats.hh"

/**
 * The base crossbar contains the common elements of the non-coherent
 * and coherent crossbar. It is an abstract class that does not have
 * any of the functionality relating to the actual reception and
 * transmission of packets, as this is left for the subclasses.
 *
 * The BaseXBar is responsible for the basic flow control (busy or
 * not), the administration of retries, and the address decoding.
 */
class BaseXBar : public MemObject
{

  protected:

    /**
     * A layer is an internal crossbar arbitration point with its own
     * flow control. Each layer is a converging multiplexer tree. By
     * instantiating one layer per destination port (and per packet
     * type, i.e. request, response, snoop request and snoop
     * response), we model full crossbar structures like AXI, ACE,
     * PCIe, etc.
     *
     * The template parameter, PortClass, indicates the destination
     * port type for the layer. The retry list holds either master
     * ports or slave ports, depending on the direction of the
     * layer. Thus, a request layer has a retry list containing slave
     * ports, whereas a response layer holds master ports.
     */
    template <typename SrcType, typename DstType>
    class Layer : public Drainable
    {

      public:

        /**
         * Create a layer and give it a name. The layer uses
         * the crossbar an event manager.
         *
         * @param _port destination port the layer converges at
         * @param _xbar the crossbar this layer belongs to
         * @param _name the layer's name
         */
        Layer(DstType& _port, BaseXBar& _xbar, const std::string& _name);

        /**
         * Drain according to the normal semantics, so that the crossbar
         * can tell the layer to drain, and pass an event to signal
         * back when drained.
         *
         * @param de drain event to call once drained
         *
         * @return 1 if busy or waiting to retry, or 0 if idle
         */
        DrainState drain() override;

        /**
         * Get the crossbar layer's name
         */
        const std::string name() const { return xbar.name() + _name; }


        /**
         * Determine if the layer accepts a packet from a specific
         * port. If not, the port in question is also added to the
         * retry list. In either case the state of the layer is
         * updated accordingly.
         *
         * @param port Source port presenting the packet
         *
         * @return True if the layer accepts the packet
         */
        bool tryTiming(SrcType* src_port);

        /**
         * Deal with a destination port accepting a packet by potentially
         * removing the source port from the retry list (if retrying) and
         * occupying the layer accordingly.
         *
         * @param busy_time Time to spend as a result of a successful send
         */
        void succeededTiming(Tick busy_time);

        /**
         * Deal with a destination port not accepting a packet by
         * potentially adding the source port to the retry list (if
         * not already at the front) and occupying the layer
         * accordingly.
         *
         * @param src_port Source port
         * @param busy_time Time to spend as a result of a failed send
         */
        void failedTiming(SrcType* src_port, Tick busy_time);

        /** Occupy the layer until until */
        void occupyLayer(Tick until);

        /**
         * Send a retry to the port at the head of waitingForLayer. The
         * caller must ensure that the list is not empty.
         */
        void retryWaiting();

        /**
         * Handle a retry from a neighbouring module. This wraps
         * retryWaiting by verifying that there are ports waiting
         * before calling retryWaiting.
         */
        void recvRetry();

        /**
         * Register stats for the layer
         */
        void regStats();

      protected:

        /**
         * Sending the actual retry, in a manner specific to the
         * individual layers. Note that for a MasterPort, there is
         * both a RequestLayer and a SnoopResponseLayer using the same
         * port, but using different functions for the flow control.
         */
        virtual void sendRetry(SrcType* retry_port) = 0;

      private:

        /** The destination port this layer converges at. */
        DstType& port;

        /** The crossbar this layer is a part of. */
        BaseXBar& xbar;

        /** A name for this layer. */
        std::string _name;

        /**
         * We declare an enum to track the state of the layer. The
         * starting point is an idle state where the layer is waiting
         * for a packet to arrive. Upon arrival, the layer
         * transitions to the busy state, where it remains either
         * until the packet transfer is done, or the header time is
         * spent. Once the layer leaves the busy state, it can
         * either go back to idle, if no packets have arrived while it
         * was busy, or the layer goes on to retry the first port
         * in waitingForLayer. A similar transition takes place from
         * idle to retry if the layer receives a retry from one of
         * its connected ports. The retry state lasts until the port
         * in questions calls sendTiming and returns control to the
         * layer, or goes to a busy state if the port does not
         * immediately react to the retry by calling sendTiming.
         */
        enum State { IDLE, BUSY, RETRY };

        /** track the state of the layer */
        State state;

        /**
         * A deque of ports that retry should be called on because
         * the original send was delayed due to a busy layer.
         */
        std::deque<SrcType*> waitingForLayer;

        /**
         * Track who is waiting for the retry when receiving it from a
         * peer. If no port is waiting NULL is stored.
         */
        SrcType* waitingForPeer;

        /**
         * Release the layer after being occupied and return to an
         * idle state where we proceed to send a retry to any
         * potential waiting port, or drain if asked to do so.
         */
        void releaseLayer();

        /** event used to schedule a release of the layer */
        EventFunctionWrapper releaseEvent;

        /**
         * Stats for occupancy and utilization. These stats capture
         * the time the layer spends in the busy state and are thus only
         * relevant when the memory system is in timing mode.
         */
        Stats::Scalar occupancy;
        Stats::Formula utilization;

    };

    class ReqLayer : public Layer<SlavePort,MasterPort>
    {
      public:
        /**
         * Create a request layer and give it a name.
         *
         * @param _port destination port the layer converges at
         * @param _xbar the crossbar this layer belongs to
         * @param _name the layer's name
         */
        ReqLayer(MasterPort& _port, BaseXBar& _xbar, const std::string& _name) :
            Layer(_port, _xbar, _name) {}

      protected:

        void sendRetry(SlavePort* retry_port)
        { retry_port->sendRetryReq(); }
    };

    class RespLayer : public Layer<MasterPort,SlavePort>
    {
      public:
        /**
         * Create a response layer and give it a name.
         *
         * @param _port destination port the layer converges at
         * @param _xbar the crossbar this layer belongs to
         * @param _name the layer's name
         */
        RespLayer(SlavePort& _port, BaseXBar& _xbar, const std::string& _name) :
            Layer(_port, _xbar, _name) {}

      protected:

        void sendRetry(MasterPort* retry_port)
        { retry_port->sendRetryResp(); }
    };

    class SnoopRespLayer : public Layer<SlavePort,MasterPort>
    {
      public:
        /**
         * Create a snoop response layer and give it a name.
         *
         * @param _port destination port the layer converges at
         * @param _xbar the crossbar this layer belongs to
         * @param _name the layer's name
         */
        SnoopRespLayer(MasterPort& _port, BaseXBar& _xbar,
                       const std::string& _name) :
            Layer(_port, _xbar, _name) {}

      protected:

        void sendRetry(SlavePort* retry_port)
        { retry_port->sendRetrySnoopResp(); }
    };

    /**
     * Cycles of front-end pipeline including the delay to accept the request
     * and to decode the address.
     */
    const Cycles frontendLatency;
    /** Cycles of forward latency */
    const Cycles forwardLatency;
    /** Cycles of response latency */
    const Cycles responseLatency;
    /** the width of the xbar in bytes */
    const uint32_t width;

    AddrRangeMap<PortID> portMap;

    /**
     * Remember where request packets came from so that we can route
     * responses to the appropriate port. This relies on the fact that
     * the underlying Request pointer inside the Packet stays
     * constant.
     */
    std::unordered_map<RequestPtr, PortID> routeTo;

    /** all contigous ranges seen by this crossbar */
    AddrRangeList xbarRanges;

    AddrRange defaultRange;

    /**
     * Function called by the port when the crossbar is recieving a
     * range change.
     *
     * @param master_port_id id of the port that received the change
     */
    virtual void recvRangeChange(PortID master_port_id);

    /** Find which port connected to this crossbar (if any) should be
     * given a packet with this address.
     *
     * @param addr Address to find port for.
     * @return id of port that the packet should be sent out of.
     */
    PortID findPort(Addr addr);

    // Cache for the findPort function storing recently used ports from portMap
    struct PortCache {
        bool valid;
        PortID id;
        AddrRange range;
    };

    PortCache portCache[3];

    // Checks the cache and returns the id of the port that has the requested
    // address within its range
    inline PortID checkPortCache(Addr addr) const {
        if (portCache[0].valid && portCache[0].range.contains(addr)) {
            return portCache[0].id;
        }
        if (portCache[1].valid && portCache[1].range.contains(addr)) {
            return portCache[1].id;
        }
        if (portCache[2].valid && portCache[2].range.contains(addr)) {
            return portCache[2].id;
        }

        return InvalidPortID;
    }

    // Clears the earliest entry of the cache and inserts a new port entry
    inline void updatePortCache(short id, const AddrRange& range) {
        portCache[2].valid = portCache[1].valid;
        portCache[2].id    = portCache[1].id;
        portCache[2].range = portCache[1].range;

        portCache[1].valid = portCache[0].valid;
        portCache[1].id    = portCache[0].id;
        portCache[1].range = portCache[0].range;

        portCache[0].valid = true;
        portCache[0].id    = id;
        portCache[0].range = range;
    }

    // Clears the cache. Needs to be called in constructor.
    inline void clearPortCache() {
        portCache[2].valid = false;
        portCache[1].valid = false;
        portCache[0].valid = false;
    }

    /**
     * Return the address ranges the crossbar is responsible for.
     *
     * @return a list of non-overlapping address ranges
     */
    AddrRangeList getAddrRanges() const;

    /**
     * Calculate the timing parameters for the packet. Updates the
     * headerDelay and payloadDelay fields of the packet
     * object with the relative number of ticks required to transmit
     * the header and the payload, respectively.
     *
     * @param pkt Packet to populate with timings
     * @param header_delay Header delay to be added
     */
    void calcPacketTiming(PacketPtr pkt, Tick header_delay);

    /**
     * Remember for each of the master ports of the crossbar if we got
     * an address range from the connected slave. For convenience,
     * also keep track of if we got ranges from all the slave modules
     * or not.
     */
    std::vector<bool> gotAddrRanges;
    bool gotAllAddrRanges;

    /** The master and slave ports of the crossbar */
    std::vector<QueuedSlavePort*> slavePorts;
    std::vector<MasterPort*> masterPorts;

    /** Port that handles requests that don't match any of the interfaces.*/
    PortID defaultPortID;

    /** If true, use address range provided by default device.  Any
       address not handled by another port and not in default device's
       range will cause a fatal error.  If false, just send all
       addresses not handled by another port to default device. */
    const bool useDefaultRange;

    BaseXBar(const BaseXBarParams *p);

    /**
     * Stats for transaction distribution and data passing through the
     * crossbar. The transaction distribution is globally counting
     * different types of commands. The packet count and total packet
     * size are two-dimensional vectors that are indexed by the
     * slave port and master port id (thus the neighbouring master and
     * neighbouring slave), summing up both directions (request and
     * response).
     */
    Stats::Vector transDist;
    Stats::Vector2d pktCount;
    Stats::Vector2d pktSize;

  public:

    virtual ~BaseXBar();

    virtual void init();

    /** A function used to return the port associated with this object. */
    BaseMasterPort& getMasterPort(const std::string& if_name,
                                  PortID idx = InvalidPortID);
    BaseSlavePort& getSlavePort(const std::string& if_name,
                                PortID idx = InvalidPortID);

    virtual void regStats();

};

#endif //__MEM_XBAR_HH__
