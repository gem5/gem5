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
 *
 * Authors: Nathan Binkert
 */

/* @file
 * Interface to connect a simulated ethernet device to the real world
 */

#include "dev/net/ethertap.hh"

#if defined(__OpenBSD__) || defined(__APPLE__)
#include <sys/param.h>

#endif
#include <netinet/in.h>
#include <unistd.h>

#include <deque>
#include <string>

#include "base/misc.hh"
#include "base/pollevent.hh"
#include "base/socket.hh"
#include "base/trace.hh"
#include "debug/Ethernet.hh"
#include "debug/EthernetData.hh"
#include "dev/net/etherdump.hh"
#include "dev/net/etherint.hh"
#include "dev/net/etherpkt.hh"

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
    // As a consequence of being called from the PollQueue, we might
    // have been called from a different thread. Migrate to "our"
    // thread.
    EventQueue::ScopedMigration migrate(tap->eventQueue());

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

EtherTap::EtherTap(const Params *p)
    : EtherObject(p), event(NULL), socket(-1), buflen(p->bufsz), dump(p->dump),
      interface(NULL), txEvent(this)
{
    if (ListenSocket::allDisabled())
        fatal("All listeners are disabled! EtherTap can't work!");

    buffer = new char[buflen];
    listener = new TapListener(this, p->port);
    listener->listen();
    interface = new EtherTapInt(name() + ".interface", this);
}

EtherTap::~EtherTap()
{
    if (event)
        delete event;
    if (buffer)
        delete [] buffer;

    delete interface;
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
    event = 0;
    close(socket);
    socket = -1;
}

bool
EtherTap::recvPacket(EthPacketPtr packet)
{
    if (dump)
        dump->dump(packet);

    DPRINTF(Ethernet, "EtherTap output len=%d\n", packet->length);
    DDUMP(EthernetData, packet->data, packet->length);
    uint32_t len = htonl(packet->length);
    ssize_t ret = write(socket, &len, sizeof(len));
    if (ret != sizeof(len))
        return false;
    ret = write(socket, packet->data, packet->length);
    if (ret != packet->length)
        return false;

    interface->recvDone();

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

    char *data = buffer + sizeof(uint32_t);
    if (!(revent & POLLIN))
        return;

    if (buffer_offset < data_len + sizeof(uint32_t)) {
        int len = read(socket, buffer + buffer_offset, buflen - buffer_offset);
        if (len == 0) {
            detach();
            return;
        }

        buffer_offset += len;

        if (data_len == 0)
            data_len = ntohl(*(uint32_t *)buffer);

        DPRINTF(Ethernet, "Received data from peer: len=%d buffer_offset=%d "
                "data_len=%d\n", len, buffer_offset, data_len);
    }

    while (data_len != 0 && buffer_offset >= data_len + sizeof(uint32_t)) {
        EthPacketPtr packet;
        packet = make_shared<EthPacketData>(data_len);
        packet->length = data_len;
        packet->simLength = data_len;
        memcpy(packet->data, data, data_len);

        assert(buffer_offset >= data_len + sizeof(uint32_t));
        buffer_offset -= data_len + sizeof(uint32_t);
        if (buffer_offset > 0) {
            memmove(buffer, data + data_len, buffer_offset);
            data_len = ntohl(*(uint32_t *)buffer);
        } else
            data_len = 0;

        DPRINTF(Ethernet, "EtherTap input len=%d\n", packet->length);
        DDUMP(EthernetData, packet->data, packet->length);
        if (!interface->sendPacket(packet)) {
            DPRINTF(Ethernet, "bus busy...buffer for retransmission\n");
            packetBuffer.push(packet);
            if (!txEvent.scheduled())
                schedule(txEvent, curTick() + retryTime);
        } else if (dump) {
            dump->dump(packet);
        }
    }
}

void
EtherTap::retransmit()
{
    if (packetBuffer.empty())
        return;

    EthPacketPtr packet = packetBuffer.front();
    if (interface->sendPacket(packet)) {
        if (dump)
            dump->dump(packet);
        DPRINTF(Ethernet, "EtherTap retransmit\n");
        packetBuffer.front() = NULL;
        packetBuffer.pop();
    }

    if (!packetBuffer.empty() && !txEvent.scheduled())
        schedule(txEvent, curTick() + retryTime);
}

EtherInt*
EtherTap::getEthPort(const std::string &if_name, int idx)
{
    if (if_name == "tap") {
        if (interface->getPeer())
            panic("Interface already connected to\n");
        return interface;
    }
    return NULL;
}


//=====================================================================

void
EtherTap::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(socket);
    SERIALIZE_SCALAR(buflen);
    uint8_t *buffer = (uint8_t *)this->buffer;
    SERIALIZE_ARRAY(buffer, buflen);
    SERIALIZE_SCALAR(buffer_offset);
    SERIALIZE_SCALAR(data_len);

    bool tapevent_present = false;
    if (event) {
        tapevent_present = true;
        SERIALIZE_SCALAR(tapevent_present);
        event->serialize(cp);
    }
    else {
        SERIALIZE_SCALAR(tapevent_present);
    }
}

void
EtherTap::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(socket);
    UNSERIALIZE_SCALAR(buflen);
    uint8_t *buffer = (uint8_t *)this->buffer;
    UNSERIALIZE_ARRAY(buffer, buflen);
    UNSERIALIZE_SCALAR(buffer_offset);
    UNSERIALIZE_SCALAR(data_len);

    bool tapevent_present;
    UNSERIALIZE_SCALAR(tapevent_present);
    if (tapevent_present) {
        event = new TapEvent(this, socket, POLLIN|POLLERR);

        event->unserialize(cp);

        if (event->queued()) {
            pollQueue.schedule(event);
        }
    }
}

//=====================================================================

EtherTap *
EtherTapParams::create()
{
    return new EtherTap(this);
}
