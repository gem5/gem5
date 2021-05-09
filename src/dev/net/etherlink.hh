/*
 * Copyright (c) 2015 ARM Limited
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

/* @file
 * Device module for modelling a fixed bandwidth full duplex ethernet link
 */

#ifndef __DEV_NET_ETHERLINK_HH__
#define __DEV_NET_ETHERLINK_HH__

#include <queue>
#include <utility>

#include "base/types.hh"
#include "dev/net/etherint.hh"
#include "dev/net/etherpkt.hh"
#include "params/EtherLink.hh"
#include "sim/eventq.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class EtherDump;
class Checkpoint;
/*
 * Model for a fixed bandwidth full duplex ethernet link
 */
class EtherLink : public SimObject
{
  protected:
    class Interface;

    /*
     * Model for a single uni-directional link
     */
    class Link
    {
      protected:
        const std::string objName;

        EtherLink *const parent;
        const int number;

        Interface *txint;
        Interface *rxint;

        const double ticksPerByte;
        const Tick linkDelay;
        const Tick delayVar;
        EtherDump *const dump;

      protected:
        /*
         * Transfer is complete
         */
        EthPacketPtr packet;
        void txDone();
        EventFunctionWrapper doneEvent;

        /**
         * Maintain a queue of in-flight packets. Assume that the
         * delay is non-zero and constant (i.e., at most one packet
         * per tick).
         */
        std::deque<std::pair<Tick, EthPacketPtr>> txQueue;

        void processTxQueue();
        EventFunctionWrapper txQueueEvent;

        void txComplete(EthPacketPtr packet);

      public:
        Link(const std::string &name, EtherLink *p, int num,
             double rate, Tick delay, Tick delay_var, EtherDump *dump);
        ~Link() {}

        const std::string name() const { return objName; }

        bool busy() const { return (bool)packet; }
        bool transmit(EthPacketPtr packet);

        void setTxInt(Interface *i) { assert(!txint); txint = i; }
        void setRxInt(Interface *i) { assert(!rxint); rxint = i; }

        void serialize(const std::string &base, CheckpointOut &cp) const;
        void unserialize(const std::string &base, CheckpointIn &cp);
    };

    /*
     * Interface at each end of the link
     */
    class Interface : public EtherInt
    {
      private:
        Link *txlink;

      public:
        Interface(const std::string &name, Link *txlink, Link *rxlink);
        bool recvPacket(EthPacketPtr packet) { return txlink->transmit(packet); }
        void sendDone() { peer->sendDone(); }
        bool isBusy() { return txlink->busy(); }
    };

    Link *link[2];
    Interface *interface[2];

  public:
    using Params = EtherLinkParams;
    EtherLink(const Params &p);
    virtual ~EtherLink();

    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

};

} // namespace gem5

#endif // __DEV_NET_ETHERLINK_HH__
