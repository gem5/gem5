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
 */

/* @file
 * Class representing the actual interface between two ethernet
 * components.
 */

#ifndef __DEV_NET_ETHERINT_HH__
#define __DEV_NET_ETHERINT_HH__

#include <string>

#include "dev/net/etherpkt.hh"
#include "mem/port.hh"

namespace gem5
{

/*
 * Class representing the actual interface between two ethernet
 * components.  These components are intended to attach to another
 * ethernet interface on one side and whatever device on the other.
 */
class EtherInt : public Port
{
  protected:
    mutable std::string portName;
    EtherInt *peer;

  public:
    EtherInt(const std::string &name, int idx=InvalidPortID)
        : Port(name, idx), portName(name), peer(NULL) {}
    virtual ~EtherInt() {}

    /** Return port name (for DPRINTF). */
    const std::string &name() const { return portName; }

    void bind(Port &peer) override;
    void unbind() override;

    void setPeer(EtherInt *p);
    EtherInt* getPeer() { return peer; }

    void recvDone() { peer->sendDone(); }
    virtual void sendDone() = 0;

    bool sendPacket(EthPacketPtr packet)
    { return peer ? peer->recvPacket(packet) : true; }
    virtual bool recvPacket(EthPacketPtr packet) = 0;

    bool askBusy() {return peer->isBusy(); }
    virtual bool isBusy() { return false; }
};

} // namespace gem5

#endif // __DEV_NET_ETHERINT_HH__
