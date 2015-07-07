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
 * Declaration of a non-coherent crossbar.
 */

#ifndef __MEM_NONCOHERENT_XBAR_HH__
#define __MEM_NONCOHERENT_XBAR_HH__

#include "mem/xbar.hh"
#include "params/NoncoherentXBar.hh"

/**
 * A non-coherent crossbar connects a number of non-snooping masters
 * and slaves, and routes the request and response packets based on
 * the address. The request packets issued by the master connected to
 * a non-coherent crossbar could still snoop in caches attached to a
 * coherent crossbar, as is the case with the I/O bus and memory bus
 * in most system configurations. No snoops will, however, reach any
 * master on the non-coherent crossbar itself.
 *
 * The non-coherent crossbar can be used as a template for modelling
 * PCIe, and non-coherent AMBA and OCP buses, and is typically used
 * for the I/O buses.
 */
class NoncoherentXBar : public BaseXBar
{

  protected:

    /**
     * Declare the layers of this crossbar, one vector for requests
     * and one for responses.
     */
    std::vector<ReqLayer*> reqLayers;
    std::vector<RespLayer*> respLayers;

    /**
     * Declaration of the non-coherent crossbar slave port type, one
     * will be instantiated for each of the master ports connecting to
     * the crossbar.
     */
    class NoncoherentXBarSlavePort : public QueuedSlavePort
    {
      private:

        /** A reference to the crossbar to which this port belongs. */
        NoncoherentXBar &xbar;

        /** A normal packet queue used to store responses. */
        RespPacketQueue queue;

      public:

        NoncoherentXBarSlavePort(const std::string &_name,
                                NoncoherentXBar &_xbar, PortID _id)
            : QueuedSlavePort(_name, &_xbar, queue, _id), xbar(_xbar),
              queue(_xbar, *this)
        { }

      protected:

        /**
         * When receiving a timing request, pass it to the crossbar.
         */
        virtual bool recvTimingReq(PacketPtr pkt)
        { return xbar.recvTimingReq(pkt, id); }

        /**
         * When receiving an atomic request, pass it to the crossbar.
         */
        virtual Tick recvAtomic(PacketPtr pkt)
        { return xbar.recvAtomic(pkt, id); }

        /**
         * When receiving a functional request, pass it to the crossbar.
         */
        virtual void recvFunctional(PacketPtr pkt)
        { xbar.recvFunctional(pkt, id); }

        /**
         * Return the union of all adress ranges seen by this crossbar.
         */
        virtual AddrRangeList getAddrRanges() const
        { return xbar.getAddrRanges(); }

    };

    /**
     * Declaration of the crossbar master port type, one will be
     * instantiated for each of the slave ports connecting to the
     * crossbar.
     */
    class NoncoherentXBarMasterPort : public MasterPort
    {
      private:

        /** A reference to the crossbar to which this port belongs. */
        NoncoherentXBar &xbar;

      public:

        NoncoherentXBarMasterPort(const std::string &_name,
                                 NoncoherentXBar &_xbar, PortID _id)
            : MasterPort(_name, &_xbar, _id), xbar(_xbar)
        { }

      protected:

        /**
         * When receiving a timing response, pass it to the crossbar.
         */
        virtual bool recvTimingResp(PacketPtr pkt)
        { return xbar.recvTimingResp(pkt, id); }

        /** When reciving a range change from the peer port (at id),
            pass it to the crossbar. */
        virtual void recvRangeChange()
        { xbar.recvRangeChange(id); }

        /** When reciving a retry from the peer port (at id),
            pass it to the crossbar. */
        virtual void recvReqRetry()
        { xbar.recvReqRetry(id); }

    };

    /** Function called by the port when the crossbar is recieving a Timing
      request packet.*/
    virtual bool recvTimingReq(PacketPtr pkt, PortID slave_port_id);

    /** Function called by the port when the crossbar is recieving a Timing
      response packet.*/
    virtual bool recvTimingResp(PacketPtr pkt, PortID master_port_id);

    /** Timing function called by port when it is once again able to process
     * requests. */
    void recvReqRetry(PortID master_port_id);

    /** Function called by the port when the crossbar is recieving a Atomic
      transaction.*/
    Tick recvAtomic(PacketPtr pkt, PortID slave_port_id);

    /** Function called by the port when the crossbar is recieving a Functional
        transaction.*/
    void recvFunctional(PacketPtr pkt, PortID slave_port_id);

  public:

    NoncoherentXBar(const NoncoherentXBarParams *p);

    virtual ~NoncoherentXBar();

    /**
     * stats
     */
    virtual void regStats();
    Stats::Scalar totPktSize;
};

#endif //__MEM_NONCOHERENT_XBAR_HH__
