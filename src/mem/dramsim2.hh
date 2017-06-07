/*
 * Copyright (c) 2013 ARM Limited
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
 *
 * Authors: Andreas Hansson
 */

/**
 * @file
 * DRAMSim2
 */
#ifndef __MEM_DRAMSIM2_HH__
#define __MEM_DRAMSIM2_HH__

#include <queue>
#include <unordered_map>

#include "mem/abstract_mem.hh"
#include "mem/dramsim2_wrapper.hh"
#include "mem/qport.hh"
#include "params/DRAMSim2.hh"

class DRAMSim2 : public AbstractMemory
{
  private:

    /**
     * The memory port has to deal with its own flow control to avoid
     * having unbounded storage that is implicitly created in the port
     * itself.
     */
    class MemoryPort : public SlavePort
    {

      private:

        DRAMSim2& memory;

      public:

        MemoryPort(const std::string& _name, DRAMSim2& _memory);

      protected:

        Tick recvAtomic(PacketPtr pkt);

        void recvFunctional(PacketPtr pkt);

        bool recvTimingReq(PacketPtr pkt);

        void recvRespRetry();

        AddrRangeList getAddrRanges() const;

    };

    MemoryPort port;

    /**
     * The actual DRAMSim2 wrapper
     */
    DRAMSim2Wrapper wrapper;

    /**
     * Is the connected port waiting for a retry from us
     */
    bool retryReq;

    /**
     * Are we waiting for a retry for sending a response.
     */
    bool retryResp;

    /**
     * Keep track of when the wrapper is started.
     */
    Tick startTick;

    /**
     * Keep track of what packets are outstanding per
     * address, and do so separately for reads and writes. This is
     * done so that we can return the right packet on completion from
     * DRAMSim.
     */
    std::unordered_map<Addr, std::queue<PacketPtr> > outstandingReads;
    std::unordered_map<Addr, std::queue<PacketPtr> > outstandingWrites;

    /**
     * Count the number of outstanding transactions so that we can
     * block any further requests until there is space in DRAMSim2 and
     * the sending queue we need to buffer the response packets.
     */
    unsigned int nbrOutstandingReads;
    unsigned int nbrOutstandingWrites;

    /**
     * Queue to hold response packets until we can send them
     * back. This is needed as DRAMSim2 unconditionally passes
     * responses back without any flow control.
     */
    std::deque<PacketPtr> responseQueue;

    unsigned int nbrOutstanding() const;

    /**
     * When a packet is ready, use the "access()" method in
     * AbstractMemory to actually create the response packet, and send
     * it back to the outside world requestor.
     *
     * @param pkt The packet from the outside world
     */
    void accessAndRespond(PacketPtr pkt);

    void sendResponse();

    /**
     * Event to schedule sending of responses
     */
    EventFunctionWrapper sendResponseEvent;

    /**
     * Progress the controller one clock cycle.
     */
    void tick();

    /**
     * Event to schedule clock ticks
     */
    EventFunctionWrapper tickEvent;

    /**
     * Upstream caches need this packet until true is returned, so
     * hold it for deletion until a subsequent call
     */
    std::unique_ptr<Packet> pendingDelete;

  public:

    typedef DRAMSim2Params Params;
    DRAMSim2(const Params *p);

    /**
     * Read completion callback.
     *
     * @param id Channel id of the responder
     * @param addr Address of the request
     * @param cycle Internal cycle count of DRAMSim2
     */
    void readComplete(unsigned id, uint64_t addr, uint64_t cycle);

    /**
     * Write completion callback.
     *
     * @param id Channel id of the responder
     * @param addr Address of the request
     * @param cycle Internal cycle count of DRAMSim2
     */
    void writeComplete(unsigned id, uint64_t addr, uint64_t cycle);

    DrainState drain() override;

    virtual BaseSlavePort& getSlavePort(const std::string& if_name,
                                        PortID idx = InvalidPortID) override;

    void init() override;
    void startup() override;

  protected:

    Tick recvAtomic(PacketPtr pkt);
    void recvFunctional(PacketPtr pkt);
    bool recvTimingReq(PacketPtr pkt);
    void recvRespRetry();

};

#endif // __MEM_DRAMSIM2_HH__
