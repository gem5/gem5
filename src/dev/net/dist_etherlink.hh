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
 * Device module for a full duplex ethernet link for dist gem5 simulations.
 *
 * See comments in dev/net/dist_iface.hh for a generic description of dist
 * gem5 simulations.
 *
 * This class is meant to be a drop in replacement for the EtherLink class for
 * dist gem5 runs.
 *
 */
#ifndef __DEV_DIST_ETHERLINK_HH__
#define __DEV_DIST_ETHERLINK_HH__

#include <cassert>
#include <iostream>

#include "base/types.hh"
#include "dev/net/etherlink.hh"
#include "params/DistEtherLink.hh"
#include "sim/serialize.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class DistIface;
class EthPacketData;

/**
 * Model for a fixed bandwidth full duplex ethernet link.
 */
class DistEtherLink : public SimObject
{
  protected:
    class LocalIface;

    /**
     * Model base class for a single uni-directional link.
     *
     * The link will encapsulate and transfer Ethernet packets to/from
     * the message server.
     */
    class Link : public Serializable
    {
      protected:
        std::string objName;
        DistEtherLink *parent;
        LocalIface *localIface;
        EtherDump *dump;
        DistIface *distIface;
        Event *event;
        EthPacketPtr packet;

      public:
        Link(const std::string &name, DistEtherLink *p,
             EtherDump *d, Event *e) :
            objName(name), parent(p), localIface(nullptr), dump(d),
            distIface(nullptr), event(e) {}

        ~Link() {}

        const std::string name() const { return objName; }
        bool busy() const { return (bool)packet; }
        void setLocalInt(LocalIface *i) { assert(!localIface); localIface=i; }

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;
    };

    /**
     * Model for a send link.
     */
    class TxLink : public Link
    {
      protected:
        /**
         * Per byte send delay
         */
        double ticksPerByte;
        /**
         * Random component of the send delay
         */
        Tick delayVar;

        /**
         * Send done callback. Called from doneEvent.
         */
        void txDone();
        EventFunctionWrapper doneEvent;

      public:
        TxLink(const std::string &name, DistEtherLink *p,
               double invBW, Tick delay_var, EtherDump *d) :
            Link(name, p, d, &doneEvent), ticksPerByte(invBW),
            delayVar(delay_var), doneEvent([this]{ txDone(); }, name) {}
        ~TxLink() {}

        /**
         * Register the dist interface to be used to talk to the
         * peer gem5 processes.
         */
        void setDistInt(DistIface *m) { assert(!distIface); distIface=m; }

        /**
         * Initiate sending of a packet via this link.
         *
         * @param packet Ethernet packet to send
         */
        bool transmit(EthPacketPtr packet);
    };

    /**
     * Model for a receive link.
     */
    class RxLink : public Link
    {
      protected:

        /**
         * Transmission delay for the simulated Ethernet link.
         */
        Tick linkDelay;

        /**
         * Receive done callback method. Called from doneEvent.
         */
        void rxDone();
        EventFunctionWrapper _doneEvent;

      public:

        RxLink(const std::string &name, DistEtherLink *p,
               Tick delay, EtherDump *d) :
            Link(name, p, d, &_doneEvent), linkDelay(delay),
            _doneEvent([this]{ rxDone(); }, name) {}
        ~RxLink() {}

        /**
         * Register our dist interface to talk to the peer gem5 processes.
         */
        void setDistInt(DistIface *m);
        /**
         * Done events will be scheduled by DistIface (so we need the accessor)
         */
        const EventFunctionWrapper *doneEvent() const { return &_doneEvent; }
    };

    /**
     * Interface to the local simulated system
     */
    class LocalIface : public EtherInt
    {
      private:
        TxLink *txLink;

      public:
        LocalIface(const std::string &name, TxLink *tx, RxLink *rx,
                   DistIface *m);

        bool recvPacket(EthPacketPtr pkt) { return txLink->transmit(pkt); }
        void sendDone() { peer->sendDone(); }
        bool isBusy() { return txLink->busy(); }
    };


  protected:
    /**
     * Interface to talk to the peer gem5 processes.
     */
    DistIface *distIface;
    /**
     * Send link
     */
    TxLink *txLink;
    /**
     * Receive link
     */
    RxLink *rxLink;
    LocalIface *localIface;

    Tick linkDelay;

  public:
    using Params = DistEtherLinkParams;
    DistEtherLink(const Params &p);
    ~DistEtherLink();

    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;

    virtual void init() override;
    virtual void startup() override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

} // namespace gem5

#endif // __DEV_DIST_ETHERLINK_HH__
