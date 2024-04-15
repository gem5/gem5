/*
 * Copyright (c) 2011-2015, 2019 ARM Limited
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
 */

/**
 * @file
 * Declaration of a non-coherent crossbar.
 */

#ifndef __MEM_NONCOHERENT_XBAR_HH__
#define __MEM_NONCOHERENT_XBAR_HH__

#include "mem/xbar.hh"
#include "params/NoncoherentXBar.hh"

namespace gem5
{

/**
 * A non-coherent crossbar connects a number of non-snooping memory-side ports
 * and cpu_sides, and routes the request and response packets based on
 * the address. The request packets issued by the memory-side port connected to
 * a non-coherent crossbar could still snoop in caches attached to a
 * coherent crossbar, as is the case with the I/O bus and memory bus
 * in most system configurations. No snoops will, however, reach any
 * memory-side port on the non-coherent crossbar itself.
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
    std::vector<ReqLayer *> reqLayers;
    std::vector<RespLayer *> respLayers;

    /**
     * Declaration of the non-coherent crossbar CPU-side port type, one
     * will be instantiated for each of the memory-side ports connecting to
     * the crossbar.
     */
    class NoncoherentXBarResponsePort : public QueuedResponsePort
    {
      private:
        /** A reference to the crossbar to which this port belongs. */
        NoncoherentXBar &xbar;

        /** A normal packet queue used to store responses. */
        RespPacketQueue queue;

      public:
        NoncoherentXBarResponsePort(const std::string &_name,
                                    NoncoherentXBar &_xbar, PortID _id)
            : QueuedResponsePort(_name, queue, _id),
              xbar(_xbar),
              queue(_xbar, *this)
        {}

      protected:
        bool
        recvTimingReq(PacketPtr pkt) override
        {
            return xbar.recvTimingReq(pkt, id);
        }

        Tick
        recvAtomic(PacketPtr pkt) override
        {
            return xbar.recvAtomicBackdoor(pkt, id);
        }

        Tick
        recvAtomicBackdoor(PacketPtr pkt, MemBackdoorPtr &backdoor) override
        {
            return xbar.recvAtomicBackdoor(pkt, id, &backdoor);
        }

        void
        recvFunctional(PacketPtr pkt) override
        {
            xbar.recvFunctional(pkt, id);
        }

        void
        recvMemBackdoorReq(const MemBackdoorReq &req,
                           MemBackdoorPtr &backdoor) override
        {
            xbar.recvMemBackdoorReq(req, backdoor);
        }

        AddrRangeList
        getAddrRanges() const override
        {
            return xbar.getAddrRanges();
        }
    };

    /**
     * Declaration of the crossbar memory-side port type, one will be
     * instantiated for each of the CPU-side ports connecting to the
     * crossbar.
     */
    class NoncoherentXBarRequestPort : public RequestPort
    {
      private:
        /** A reference to the crossbar to which this port belongs. */
        NoncoherentXBar &xbar;

      public:
        NoncoherentXBarRequestPort(const std::string &_name,
                                   NoncoherentXBar &_xbar, PortID _id)
            : RequestPort(_name, _id), xbar(_xbar)
        {}

      protected:
        bool
        recvTimingResp(PacketPtr pkt) override
        {
            return xbar.recvTimingResp(pkt, id);
        }

        void
        recvRangeChange() override
        {
            xbar.recvRangeChange(id);
        }

        void
        recvReqRetry() override
        {
            xbar.recvReqRetry(id);
        }
    };

    virtual bool recvTimingReq(PacketPtr pkt, PortID cpu_side_port_id);
    virtual bool recvTimingResp(PacketPtr pkt, PortID mem_side_port_id);
    void recvReqRetry(PortID mem_side_port_id);
    Tick recvAtomicBackdoor(PacketPtr pkt, PortID cpu_side_port_id,
                            MemBackdoorPtr *backdoor = nullptr);
    void recvFunctional(PacketPtr pkt, PortID cpu_side_port_id);
    void recvMemBackdoorReq(const MemBackdoorReq &req,
                            MemBackdoorPtr &backdoor);

  public:
    NoncoherentXBar(const NoncoherentXBarParams &p);

    virtual ~NoncoherentXBar();
};

} // namespace gem5

#endif //__MEM_NONCOHERENT_XBAR_HH__
