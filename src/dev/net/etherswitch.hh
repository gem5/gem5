/*
 * Copyright (c) 2014 The Regents of The University of Michigan
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
 * Authors: Anthony Gutierrez
 *          Mohammad Alian
 */

/* @file
 * Device model for an ethernet switch
 */

#ifndef __DEV_ETHERSWITCH_HH__
#define __DEV_ETHERSWITCH_HH__

#include <unordered_map>

#include "base/inet.hh"
#include "dev/net/etherint.hh"
#include "dev/net/etherlink.hh"
#include "dev/net/etherobject.hh"
#include "dev/net/etherpkt.hh"
#include "dev/net/pktfifo.hh"
#include "params/EtherSwitch.hh"
#include "sim/eventq.hh"

class EtherSwitch : public EtherObject
{
  public:
    typedef EtherSwitchParams Params;

    EtherSwitch(const Params *p);
    ~EtherSwitch();

    const Params * params() const
    {
        return dynamic_cast<const Params*>(_params);
    }

    EtherInt *getEthPort(const std::string &if_name, int idx) override;

  protected:
    /**
     * Model for an Ethernet switch port
     */
    class Interface : public EtherInt
    {
      public:
        Interface(const std::string &name, EtherSwitch *_etherSwitch,
                  uint64_t outputBufferSize, Tick delay, Tick delay_var,
                  double rate);
        /**
         * When a packet is received from a device, route it
         * through an (several) output queue(s)
         */
        bool recvPacket(EthPacketPtr packet);
        /**
         * enqueue packet to the outputFifo
         */
        void enqueue(EthPacketPtr packet);
        void sendDone() {}
        Tick switchingDelay();

        Interface* lookupDestPort(Net::EthAddr destAddr);
        void learnSenderAddr(Net::EthAddr srcMacAddr, Interface *sender);

        void serialize(const std::string &base, CheckpointOut &cp) const;
        void unserialize(const std::string &base, CheckpointIn &cp);

      private:
        const double ticksPerByte;
        const Tick switchDelay;
        const Tick delayVar;
        EtherSwitch *parent;
        /**
         * output fifo at each interface
         */
        PacketFifo outputFifo;
        void transmit();
        EventWrapper<Interface, &Interface::transmit> txEvent;
    };

    struct SwitchTableEntry {
            Interface *interface;
            Tick lastUseTime;
        };

  private:
    // time to live for MAC address mappings
    const double ttl;
    // all interfaces of the switch
    std::vector<Interface*> interfaces;
    // table that maps MAC address to interfaces
    std::map<uint64_t, SwitchTableEntry> forwardingTable;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

#endif // __DEV_ETHERSWITCH_HH__
