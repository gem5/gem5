/*
 * Copyright (c) 2018 ARM Limited
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

#ifndef __MEM_MEM_DELAY_HH__
#define __MEM_MEM_DELAY_HH__

#include "mem/qport.hh"
#include "sim/clocked_object.hh"

struct MemDelayParams;
struct SimpleMemDelayParams;

/**
 * This abstract component provides a mechanism to delay
 * packets. It can be spliced between arbitrary ports of the memory
 * system and delays packets that pass through it.
 *
 * Specialisations of this abstract class should override at least one
 * of delayReq, delayResp, deleySnoopReq, delaySnoopResp. These
 * methods receive a PacketPtr as their argument and return a delay in
 * Ticks. The base class implements an infinite buffer to hold delayed
 * packets until they are ready. The intention is to use this
 * component for rapid prototyping of other memory system components
 * that introduce a packet processing delays.
 *
 * NOTE: Packets may be reordered if the delays aren't constant.
 */
class MemDelay : public ClockedObject
{

  public:
    MemDelay(const MemDelayParams *params);

    void init() override;

  protected: // Port interface
    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;

    class MasterPort : public QueuedMasterPort
    {
      public:
        MasterPort(const std::string &_name, MemDelay &_parent);

      protected:
        bool recvTimingResp(PacketPtr pkt) override;

        void recvFunctionalSnoop(PacketPtr pkt) override;

        Tick recvAtomicSnoop(PacketPtr pkt) override;

        void recvTimingSnoopReq(PacketPtr pkt) override;

        void recvRangeChange() override {
            parent.slavePort.sendRangeChange();
        }

        bool isSnooping() const override {
            return parent.slavePort.isSnooping();
        }

      private:
        MemDelay& parent;
    };

    class SlavePort : public QueuedSlavePort
    {
      public:
        SlavePort(const std::string &_name, MemDelay &_parent);

      protected:
        Tick recvAtomic(PacketPtr pkt) override;
        bool recvTimingReq(PacketPtr pkt) override;
        void recvFunctional(PacketPtr pkt) override;
        bool recvTimingSnoopResp(PacketPtr pkt) override;

        AddrRangeList getAddrRanges() const override {
            return parent.masterPort.getAddrRanges();
        }

        bool tryTiming(PacketPtr pkt) override { return true; }

      private:

        MemDelay& parent;

    };

    bool trySatisfyFunctional(PacketPtr pkt);

    MasterPort masterPort;
    SlavePort slavePort;

    ReqPacketQueue reqQueue;
    RespPacketQueue respQueue;
    SnoopRespPacketQueue snoopRespQueue;

  protected:
    /**
     * Delay a request by some number of ticks.
     *
     * @return Ticks to delay packet.
     */
    virtual Tick delayReq(PacketPtr pkt) { return 0; }

    /**
     * Delay a response by some number of ticks.
     *
     * @return Ticks to delay packet.
     */
    virtual Tick delayResp(PacketPtr pkt) { return 0; }

    /**
     * Delay a snoop response by some number of ticks.
     *
     * @return Ticks to delay packet.
     */
    virtual Tick delaySnoopResp(PacketPtr pkt) { return 0; }
};

/**
 * Delay packets by a constant time. Delays can be specified
 * separately for read requests, read responses, write requests, and
 * write responses.
 *
 * This class does not delay snoops or requests/responses that are
 * neither reads or writes.
 */
class SimpleMemDelay : public MemDelay
{
  public:
    SimpleMemDelay(const SimpleMemDelayParams *params);

  protected:
    Tick delayReq(PacketPtr pkt) override;
    Tick delayResp(PacketPtr pkt) override;

  protected: // Params
    const Tick readReqDelay;
    const Tick readRespDelay;

    const Tick writeReqDelay;
    const Tick writeRespDelay;
};

#endif //__MEM_MEM_DELAY_HH__
