/*
 * Copyright 2021 Google, Inc.
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

#ifndef __MEM_SYS_BRIDGE_HH__
#define __MEM_SYS_BRIDGE_HH__

#include "base/trace.hh"
#include "base/types.hh"
#include "debug/SysBridge.hh"
#include "mem/port.hh"
#include "params/SysBridge.hh"
#include "sim/sim_object.hh"

namespace gem5
{

/**
 * Each System object in gem5 is responsible for a set of RequestorIDs which
 * identify different sources for memory requests within that System. Each
 * object within the memory system is responsible for requesting an ID if it
 * needs one, and then using that ID in the requests it sends out.
 *
 * When a simulation has multiple System objects within it, components
 * registered with different Systems may be able to interact through the memory
 * system. If an object uses a RequestorID it got from its parent System, and
 * that ends up being handled by an object which is using a different parent,
 * the target object may misinterpret or misattribute that ID, or if the number
 * of IDs in the two systems are different, it might even index an array based
 * on the requestor out of bounds.
 *
 * This SysBridge object helps handle that situation by translating requests
 * going across it in either direction, upstream or downstream, so that they
 * always have a RequestorID which is valid in the current System. They
 * register themselves as a Requestor in each System, and use that ID in the
 * foreign System, restoring the original ID as the request makes its way back
 * to its source.
 *
 * Example:
 * # One System with a CPU in it.
 * sys1 = System(...)
 * sys1.cpu = CPU(...)
 *
 * # One System with memory in it.
 * sys2 = System(...)
 * sys2.memory = Memory(...)
 *
 * # A SysBridge for crossing from sys1 to sys2.
 * sys2.sys_bridge = SysBridge(
 *         source=sys1,
 *         target=sys2,
 *         target_port=sys2.memory.port,
 *         source_port=sys1.cpu.port)
 */

class SysBridge : public SimObject
{
  private:
    class SysBridgeTargetPort;
    class SysBridgeSourcePort;

    // A structure for whatever we need to keep when bridging a packet.
    struct PacketData
    {
        RequestPtr req;
    };

    class SysBridgeSenderState : public Packet::SenderState
    {
      private:
        PacketData pData;

      public:
        SysBridgeSenderState(const PacketData &data) : pData(data) {}

        const PacketData &data() const { return pData; }
    };

    class BridgingPort
    {
      protected:
        RequestorID id;

        // Replace the requestor ID in pkt, and return any scratch data we'll
        // need going back the other way.
        PacketData replaceReqID(PacketPtr pkt);
        // Restore pkt to use its original requestor ID.
        static void
        restoreReqID(PacketPtr pkt, const PacketData &data)
        {
            pkt->req = data.req;
        }

        static void
        restoreReqID(PacketPtr pkt, const PacketData &data, PacketData &backup)
        {
            backup.req = pkt->req;
            restoreReqID(pkt, data);
        }

        BridgingPort(RequestorID _id) : id(_id) {}
    };

    class SysBridgeTargetPort : public RequestPort, public BridgingPort
    {
      private:
        SysBridgeSourcePort *sourcePort;

      public:
        SysBridgeTargetPort(const std::string &_name, SimObject *owner,
                SysBridgeSourcePort *source_port, RequestorID _id) :
            RequestPort(_name, owner), BridgingPort(_id),
            sourcePort(source_port)
        {
            DPRINTF(SysBridge, "Target side requestor ID = %s.\n", _id);
        }

      private:
        void
        recvRangeChange() override
        {
            sourcePort->sendRangeChange();
        }

        Tick
        recvAtomicSnoop(PacketPtr pkt) override
        {
            DPRINTF(SysBridge, "recvAtomicSnoop incoming ID %d.\n",
                    pkt->requestorId());
            auto data = replaceReqID(pkt);
            DPRINTF(SysBridge, "recvAtomicSnoop outgoing ID %d.\n",
                    pkt->requestorId());
            Tick tick = sourcePort->sendAtomicSnoop(pkt);
            restoreReqID(pkt, data);
            DPRINTF(SysBridge, "recvAtomicSnoop restored ID %d.\n",
                    pkt->requestorId());
            return tick;
        }

        bool
        recvTimingResp(PacketPtr pkt) override
        {
            auto *state = dynamic_cast<SysBridgeSenderState *>(
                    pkt->popSenderState());
            PacketData backup;
            DPRINTF(SysBridge, "recvTimingResp incoming ID %d.\n",
                    pkt->requestorId());
            restoreReqID(pkt, state->data(), backup);
            DPRINTF(SysBridge, "recvTimingResp restored ID %d.\n",
                    pkt->requestorId());
            if (!sourcePort->sendTimingResp(pkt)) {
                restoreReqID(pkt, backup);
                DPRINTF(SysBridge, "recvTimingResp un-restored ID %d.\n",
                        pkt->requestorId());
                pkt->pushSenderState(state);
                return false;
            } else {
                delete state;
                return true;
            }
        }

        void
        recvTimingSnoopReq(PacketPtr pkt) override
        {
            DPRINTF(SysBridge, "recvTimingSnoopReq incoming ID %d.\n",
                    pkt->requestorId());
            auto *state = new SysBridgeSenderState(replaceReqID(pkt));
            pkt->pushSenderState(state);
            DPRINTF(SysBridge, "recvTimingSnoopReq outgoing ID %d.\n",
                    pkt->requestorId());
            sourcePort->sendTimingSnoopReq(pkt);
        }

        void recvReqRetry() override { sourcePort->sendRetryReq(); }
        void
        recvRetrySnoopResp() override
        {
            sourcePort->sendRetrySnoopResp();
        }

        void
        recvFunctionalSnoop(PacketPtr pkt) override
        {
            DPRINTF(SysBridge, "recvFunctionalSnoop incoming ID %d.\n",
                    pkt->requestorId());
            auto data = replaceReqID(pkt);
            DPRINTF(SysBridge, "recvFunctionalSnoop outgoing ID %d.\n",
                    pkt->requestorId());
            sourcePort->sendFunctionalSnoop(pkt);
            restoreReqID(pkt, data);
            DPRINTF(SysBridge, "recvFunctionalSnoop restored ID %d.\n",
                    pkt->requestorId());
        }
    };

    class SysBridgeSourcePort : public ResponsePort, public BridgingPort
    {
      private:
        SysBridgeTargetPort *targetPort;

      public:
        SysBridgeSourcePort(const std::string &_name, SimObject *owner,
                SysBridgeTargetPort *target_port, RequestorID _id) :
            ResponsePort(_name, owner), BridgingPort(_id),
            targetPort(target_port)
        {
            DPRINTF(SysBridge, "Source side requestor ID = %s.\n", _id);
        }

      private:
        bool
        recvTimingReq(PacketPtr pkt) override
        {
            DPRINTF(SysBridge, "recvTimingReq incoming ID %d.\n",
                    pkt->requestorId());
            auto *state = new SysBridgeSenderState(replaceReqID(pkt));
            pkt->pushSenderState(state);
            DPRINTF(SysBridge, "recvTimingReq outgoing ID %d.\n",
                    pkt->requestorId());
            if (!targetPort->sendTimingReq(pkt)) {
                restoreReqID(pkt, state->data());
                DPRINTF(SysBridge, "recvTimingReq restored ID %d.\n",
                        pkt->requestorId());
                pkt->popSenderState();
                delete state;
                return false;
            } else {
                return true;
            }
        }

        bool
        tryTiming(PacketPtr pkt) override
        {
            // Since tryTiming shouldn't actually send the packet, we should
            // be able to clean up inline like we would for atomic methods.
            // This may not actually be necessary at all, but it's a little
            // safer.
            DPRINTF(SysBridge, "tryTiming incoming ID %d.\n",
                    pkt->requestorId());
            auto data = replaceReqID(pkt);
            DPRINTF(SysBridge, "tryTiming outgoing ID %d.\n",
                    pkt->requestorId());
            bool ret = targetPort->tryTiming(pkt);
            restoreReqID(pkt, data);
            DPRINTF(SysBridge, "tryTiming restored ID %d.\n",
                    pkt->requestorId());
            return ret;
        }

        bool
        recvTimingSnoopResp(PacketPtr pkt) override
        {
            auto *state = dynamic_cast<SysBridgeSenderState *>(
                    pkt->popSenderState());
            DPRINTF(SysBridge, "recvTimingSnoopResp incoming ID %d.\n",
                    pkt->requestorId());
            restoreReqID(pkt, state->data());
            DPRINTF(SysBridge, "recvTimingSnoopResp restored ID %d.\n",
                    pkt->requestorId());
            return targetPort->sendTimingSnoopResp(pkt);
        }

        void recvRespRetry() override { targetPort->sendRetryResp(); }

        Tick
        recvAtomic(PacketPtr pkt) override
        {
            DPRINTF(SysBridge, "recvAtomic incoming ID %d.\n",
                    pkt->requestorId());
            auto data = replaceReqID(pkt);
            DPRINTF(SysBridge, "recvAtomic outgoing ID %d.\n",
                    pkt->requestorId());
            Tick tick = targetPort->sendAtomic(pkt);
            restoreReqID(pkt, data);
            DPRINTF(SysBridge, "recvAtomic restored ID %d.\n",
                    pkt->requestorId());
            return tick;
        }

        Tick
        recvAtomicBackdoor(PacketPtr pkt, MemBackdoorPtr &backdoor) override
        {
            DPRINTF(SysBridge, "recvAtomicBackdoor incoming ID %d.\n",
                    pkt->requestorId());
            auto data = replaceReqID(pkt);
            DPRINTF(SysBridge, "recvAtomicBackdoor outgoing ID %d.\n",
                    pkt->requestorId());
            Tick tick = targetPort->sendAtomicBackdoor(pkt, backdoor);
            restoreReqID(pkt, data);
            DPRINTF(SysBridge, "recvAtomicBackdoor restored ID %d.\n",
                    pkt->requestorId());
            return tick;
        }

        void
        recvFunctional(PacketPtr pkt) override
        {
            DPRINTF(SysBridge, "recvFunctional incoming ID %d.\n",
                    pkt->requestorId());
            auto data = replaceReqID(pkt);
            DPRINTF(SysBridge, "recvFunctional outgoing ID %d.\n",
                    pkt->requestorId());
            targetPort->sendFunctional(pkt);
            restoreReqID(pkt, data);
            DPRINTF(SysBridge, "recvFunctional restored ID %d.\n",
                    pkt->requestorId());
        }

        AddrRangeList
        getAddrRanges() const override
        {
            return targetPort->getAddrRanges();
        }
    };

    SysBridgeSourcePort sourcePort;
    SysBridgeTargetPort targetPort;

  public:
    Port &getPort(const std::string &if_name,
            PortID idx=InvalidPortID) override;

    SysBridge(const SysBridgeParams &p);
};

} // namespace gem5

#endif //__MEM_SYS_BRIDGE_HH__
