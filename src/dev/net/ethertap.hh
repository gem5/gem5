/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 * Interface to connect a simulated ethernet device to the real world
 */

#ifndef __DEV_NET_ETHERTAP_HH__
#define __DEV_NET_ETHERTAP_HH__

#include <queue>
#include <string>

#include "base/pollevent.hh"
#include "config/have_tuntap.hh"
#include "dev/net/etherint.hh"
#include "dev/net/etherpkt.hh"

#if HAVE_TUNTAP
#include "params/EtherTap.hh"

#endif

#include "params/EtherTapStub.hh"
#include "sim/eventq.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class TapEvent;
class EtherTapInt;

class EtherTapBase : public SimObject
{
  public:
    using Params = EtherTapBaseParams;
    EtherTapBase(const Params &p);
    virtual ~EtherTapBase();

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  protected:
    uint8_t *buffer;
    int buflen;

    EtherDump *dump;

    /*
     * Interface to the real network.
     */
  protected:
    friend class TapEvent;
    TapEvent *event;
    void pollFd(int fd);
    void stopPolling();

    // Receive data from the real network.
    virtual void recvReal(int revent) = 0;
    // Prepare and send data out to the real network.
    virtual bool sendReal(const void *data, size_t len) = 0;

    /*
     * Interface to the simulated network.
     */
  protected:
    EtherTapInt *interface;

  public:
    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    bool recvSimulated(EthPacketPtr packet);
    void sendSimulated(void *data, size_t len);

  protected:
    std::queue<EthPacketPtr> packetBuffer;
    void retransmit();
    EventFunctionWrapper txEvent;
};

class EtherTapInt : public EtherInt
{
  private:
    EtherTapBase *tap;

  public:
    EtherTapInt(const std::string &name, EtherTapBase *t)
        : EtherInt(name), tap(t)
    {}

    bool
    recvPacket(EthPacketPtr pkt) override
    {
        return tap->recvSimulated(pkt);
    }

    void
    sendDone() override
    {}
};

class TapListener;

/*
 * Interface to connect a simulated ethernet device to the real world. An
 * external helper program bridges between this object's TCP port and a
 * source/sink for Ethernet frames. Each frame going in either direction is
 * prepended with the frame's length in a 32 bit integer in network byte order.
 */
class EtherTapStub : public EtherTapBase
{
  public:
    using Params = EtherTapStubParams;
    EtherTapStub(const Params &p);
    ~EtherTapStub();

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  protected:
    friend class TapListener;
    TapListener *listener;

    int socket;

    void attach(int fd);
    void detach();

    uint32_t buffer_used;
    uint32_t frame_len;

    void recvReal(int revent) override;
    bool sendReal(const void *data, size_t len) override;
};

#if HAVE_TUNTAP
class EtherTap : public EtherTapBase
{
  public:
    using Params = EtherTapParams;
    EtherTap(const Params &p);
    ~EtherTap();

  protected:
    int tap;

    void recvReal(int revent) override;
    bool sendReal(const void *data, size_t len) override;
};
#endif

} // namespace gem5

#endif // __DEV_NET_ETHERTAP_HH__
