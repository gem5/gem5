/*
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
 * Authors: Nathan Binkert
 */

/* @file
 * Device module for modelling an ethernet hub
 */

#ifndef __DEV_NET_ETHERBUS_HH__
#define __DEV_NET_ETHERBUS_HH__

#include "dev/net/etherobject.hh"
#include "dev/net/etherpkt.hh"
#include "params/EtherBus.hh"
#include "sim/eventq.hh"
#include "sim/sim_object.hh"

class EtherDump;
class EtherInt;
class EtherBus : public SimObject, public EtherObject
{
  protected:
    typedef std::list<EtherInt *> devlist_t;
    devlist_t devlist;
    double ticksPerByte;
    bool loopback;

  protected:
    EventFunctionWrapper event;
    EthPacketPtr packet;
    EtherInt *sender;
    EtherDump *dump;

  public:
    typedef EtherBusParams Params;
    EtherBus(const Params *p);
    virtual ~EtherBus() {}

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    void txDone();
    void reg(EtherInt *dev);
    bool busy() const { return (bool)packet; }
    bool send(EtherInt *sender, EthPacketPtr &packet);
    EtherInt *getEthPort(const std::string &if_name, int idx) override;
};

#endif // __DEV_NET_ETHERBUS_HH__
