/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#if defined(__OpenBSD__)
#include <sys/param.h>
#endif
#include <netinet/in.h>

#include <unistd.h>

#include <deque>
#include <string>

#include "dev/etherdump.hh"
#include "dev/etherint.hh"
#include "dev/etherpkt.hh"
#include "dev/ethertap.hh"
#include "base/pollevent.hh"
#include "base/socket.hh"
#include "base/trace.hh"
#include "base/misc.hh"

using namespace std;

/**
 */
class TapListener
{
  protected:
    /**
     */
    class Event : public PollEvent
    {
      protected:
        TapListener *listener;

      public:
        Event(TapListener *l, int fd, int e)
            : PollEvent(fd, e), listener(l) {}

        virtual void process(int revent) { listener->accept(); }
    };

    friend class Event;
    Event *event;

  protected:
    ListenSocket listener;
    EtherTap *tap;
    int port;

  public:
    TapListener(EtherTap *t, int p)
        : event(NULL), tap(t), port(p) {}
    ~TapListener() { if (event) delete event; }

    void accept();
    void listen();
};

void
TapListener::listen()
{
    while (!listener.listen(port, true)) {
        DPRINTF(Ethernet, "TapListener(listen): Can't bind port %d\n", port);
        port++;
    }

    ccprintf(cerr, "Listening for tap connection on port %d\n", port);
    event = new Event(this, listener.getfd(), POLLIN|POLLERR);
    pollQueue.schedule(event);
}

void
TapListener::accept()
{
    if (!listener.islistening())
        panic("TapListener(accept): cannot accept if we're not listening!");

    int sfd = listener.accept(true);
    if (sfd != -1)
        tap->attach(sfd);
}

/**
 */
class TapEvent : public PollEvent
{
  protected:
    EtherTap *tap;

  public:
    TapEvent(EtherTap *_tap, int fd, int e)
        : PollEvent(fd, e), tap(_tap) {}
    virtual void process(int revent) { tap->process(revent); }
};

EtherTap::EtherTap(const string &name, EtherDump *d, int port, int bufsz)
    : EtherInt(name), event(NULL), socket(-1), buflen(bufsz), dump(d),
      txEvent(this)
{
    buffer = new char[buflen];
    listener = new TapListener(this, port);
    listener->listen();
}

EtherTap::~EtherTap()
{
    if (event)
        delete event;
    if (buffer)
        delete [] buffer;

    delete listener;
}

void
EtherTap::attach(int fd)
{
    if (socket != -1)
        close(fd);

    buffer_offset = 0;
    data_len = 0;
    socket = fd;
    DPRINTF(Ethernet, "EtherTap attached\n");
    event = new TapEvent(this, socket, POLLIN|POLLERR);
    pollQueue.schedule(event);
}

void
EtherTap::detach()
{
    DPRINTF(Ethernet, "EtherTap detached\n");
    delete event;
    close(socket);
    socket = -1;
}

bool
EtherTap::recvPacket(PacketPtr packet)
{
    if (dump)
        dump->dump(packet);

    DPRINTF(Ethernet, "EtherTap output len=%d\n", packet->length);
    DDUMP(EthernetData, packet->data, packet->length);
    u_int32_t len = htonl(packet->length);
    write(socket, &len, sizeof(len));
    write(socket, packet->data, packet->length);

    recvDone();

    return true;
}

void
EtherTap::sendDone()
{}

void
EtherTap::process(int revent)
{
    if (revent & POLLERR) {
        detach();
        return;
    }

    char *data = buffer + sizeof(u_int32_t);
    if (!(revent & POLLIN))
        return;

    if (buffer_offset < data_len + sizeof(u_int32_t)) {
        int len = read(socket, buffer + buffer_offset, buflen - buffer_offset);
        if (len == 0) {
            detach();
            return;
        }

        buffer_offset += len;

        if (data_len == 0)
            data_len = ntohl(*(u_int32_t *)buffer);

        DPRINTF(Ethernet, "Received data from peer: len=%d buffer_offset=%d "
                "data_len=%d\n", len, buffer_offset, data_len);
    }

    while (data_len != 0 && buffer_offset >= data_len + sizeof(u_int32_t)) {
        PacketPtr packet;
        packet = new EtherPacket;
        packet->data = new uint8_t[data_len];
        packet->length = data_len;
        memcpy(packet->data, data, data_len);

        buffer_offset -= data_len + sizeof(u_int32_t);
        assert(buffer_offset >= 0);
        if (buffer_offset > 0) {
            memmove(buffer, data + data_len, buffer_offset);
            data_len = ntohl(*(u_int32_t *)buffer);
        } else
            data_len = 0;

        DPRINTF(Ethernet, "EtherTap input len=%d\n", packet->length);
        DDUMP(EthernetData, packet->data, packet->length);
        if (!sendPacket(packet)) {
            DPRINTF(Ethernet, "bus busy...buffer for retransmission\n");
            packetBuffer.push(packet);
            if (!txEvent.scheduled())
                txEvent.schedule(curTick + 1000);
        } else if (dump)
            dump->dump(packet);
    }
}

void
EtherTap::retransmit()
{
    if (packetBuffer.empty())
        return;

    PacketPtr packet = packetBuffer.front();
    if (sendPacket(packet)) {
        if (dump)
            dump->dump(packet);
        DPRINTF(Ethernet, "EtherTap retransmit\n");
        packetBuffer.front() = NULL;
        packetBuffer.pop();
    }

    if (!packetBuffer.empty() && !txEvent.scheduled())
        txEvent.schedule(curTick + 1000);
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(EtherTap)

    SimObjectParam<EtherInt *> peer;
    SimObjectParam<EtherDump *> packet_dump;
    Param<uint16_t> port;
    Param<uint16_t> bufsz;

END_DECLARE_SIM_OBJECT_PARAMS(EtherTap)

BEGIN_INIT_SIM_OBJECT_PARAMS(EtherTap)

    INIT_PARAM_DFLT(peer, "peer interface", NULL),
    INIT_PARAM_DFLT(packet_dump, "object to dump network packets to", NULL),
    INIT_PARAM_DFLT(port, "tap port", 3500),
    INIT_PARAM_DFLT(bufsz, "tap buffer size", 10000)

END_INIT_SIM_OBJECT_PARAMS(EtherTap)


CREATE_SIM_OBJECT(EtherTap)
{
    EtherTap *tap = new EtherTap(getInstanceName(), packet_dump, port, bufsz);

    if (peer) {
        tap->setPeer(peer);
        peer->setPeer(tap);
    }

    return tap;
}

REGISTER_SIM_OBJECT("EtherTap", EtherTap)
