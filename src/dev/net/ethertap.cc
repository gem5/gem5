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

#if USE_TUNTAP && defined(__linux__)
#if 1 // Hide from the style checker since these have to be out of order.
#include <sys/socket.h> // Has to be included before if.h for some reason.

#endif

#include <linux/if.h>
#include <linux/if_tun.h>

#endif

#include <fcntl.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstring>
#include <deque>
#include <string>

#include "base/logging.hh"
#include "base/pollevent.hh"
#include "base/socket.hh"
#include "base/trace.hh"
#include "debug/Ethernet.hh"
#include "debug/EthernetData.hh"
#include "dev/net/etherdump.hh"
#include "dev/net/etherint.hh"
#include "dev/net/etherpkt.hh"

using namespace std;

class TapEvent : public PollEvent
{
  protected:
    EtherTapBase *tap;

  public:
    TapEvent(EtherTapBase *_tap, int fd, int e)
        : PollEvent(fd, e), tap(_tap) {}
    virtual void process(int revent) { tap->recvReal(revent); }
};

EtherTapBase::EtherTapBase(const Params *p)
    : EtherObject(p), buflen(p->bufsz), dump(p->dump), event(NULL),
      interface(NULL),
      txEvent([this]{ retransmit(); }, "EtherTapBase retransmit")
{
    buffer = new uint8_t[buflen];
    interface = new EtherTapInt(name() + ".interface", this);
}

EtherTapBase::~EtherTapBase()
{
    delete buffer;
    delete event;
    delete interface;
}

void
EtherTapBase::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(buflen);
    uint8_t *buffer = (uint8_t *)this->buffer;
    SERIALIZE_ARRAY(buffer, buflen);

    bool tapevent_present = false;
    if (event) {
        tapevent_present = true;
        SERIALIZE_SCALAR(tapevent_present);
        event->serialize(cp);
    } else {
        SERIALIZE_SCALAR(tapevent_present);
    }
}

void
EtherTapBase::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(buflen);
    uint8_t *buffer = (uint8_t *)this->buffer;
    UNSERIALIZE_ARRAY(buffer, buflen);

    bool tapevent_present;
    UNSERIALIZE_SCALAR(tapevent_present);
    if (tapevent_present) {
        event = new TapEvent(this, 0, 0);
        event->unserialize(cp);
        if (event->queued())
            pollQueue.schedule(event);
    }
}


void
EtherTapBase::pollFd(int fd)
{
    assert(!event);
    event = new TapEvent(this, fd, POLLIN|POLLERR);
    pollQueue.schedule(event);
}

void
EtherTapBase::stopPolling()
{
    assert(event);
    delete event;
    event = NULL;
}


EtherInt*
EtherTapBase::getEthPort(const std::string &if_name, int idx)
{
    if (if_name == "tap") {
        if (interface->getPeer())
            panic("Interface already connected to\n");
        return interface;
    }
    return NULL;
}

bool
EtherTapBase::recvSimulated(EthPacketPtr packet)
{
    if (dump)
        dump->dump(packet);

    DPRINTF(Ethernet, "EtherTap sim->real len=%d\n", packet->length);
    DDUMP(EthernetData, packet->data, packet->length);

    bool success = sendReal(packet->data, packet->length);

    interface->recvDone();

    return success;
}

void
EtherTapBase::sendSimulated(void *data, size_t len)
{
    EthPacketPtr packet;
    packet = make_shared<EthPacketData>(len);
    packet->length = len;
    packet->simLength = len;
    memcpy(packet->data, data, len);

    DPRINTF(Ethernet, "EtherTap real->sim len=%d\n", packet->length);
    DDUMP(EthernetData, packet->data, packet->length);
    if (!packetBuffer.empty() || !interface->sendPacket(packet)) {
        DPRINTF(Ethernet, "bus busy...buffer for retransmission\n");
        packetBuffer.push(packet);
        if (!txEvent.scheduled())
            schedule(txEvent, curTick() + retryTime);
    } else if (dump) {
        dump->dump(packet);
    }
}

void
EtherTapBase::retransmit()
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


class TapListener
{
  protected:
    class Event : public PollEvent
    {
      protected:
        TapListener *listener;

      public:
        Event(TapListener *l, int fd, int e) : PollEvent(fd, e), listener(l) {}

        void process(int revent) override { listener->accept(); }
    };

    friend class Event;
    Event *event;

    void accept();

  protected:
    ListenSocket listener;
    EtherTapStub *tap;
    int port;

  public:
    TapListener(EtherTapStub *t, int p) : event(NULL), tap(t), port(p) {}
    ~TapListener() { delete event; }

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


EtherTapStub::EtherTapStub(const Params *p) : EtherTapBase(p), socket(-1)
{
    if (ListenSocket::allDisabled())
        fatal("All listeners are disabled! EtherTapStub can't work!");

    listener = new TapListener(this, p->port);
    listener->listen();
}

EtherTapStub::~EtherTapStub()
{
    delete listener;
}

void
EtherTapStub::serialize(CheckpointOut &cp) const
{
    EtherTapBase::serialize(cp);

    SERIALIZE_SCALAR(socket);
    SERIALIZE_SCALAR(buffer_used);
    SERIALIZE_SCALAR(frame_len);
}

void
EtherTapStub::unserialize(CheckpointIn &cp)
{
    EtherTapBase::unserialize(cp);

    UNSERIALIZE_SCALAR(socket);
    UNSERIALIZE_SCALAR(buffer_used);
    UNSERIALIZE_SCALAR(frame_len);
}


void
EtherTapStub::attach(int fd)
{
    if (socket != -1)
        close(fd);

    buffer_used = 0;
    frame_len = 0;
    socket = fd;
    DPRINTF(Ethernet, "EtherTapStub attached\n");
    pollFd(socket);
}

void
EtherTapStub::detach()
{
    DPRINTF(Ethernet, "EtherTapStub detached\n");
    stopPolling();
    close(socket);
    socket = -1;
}

void
EtherTapStub::recvReal(int revent)
{
    if (revent & POLLERR) {
        detach();
        return;
    }

    if (!(revent & POLLIN))
        return;

    // Read in as much of the new data as we can.
    int len = read(socket, buffer + buffer_used, buflen - buffer_used);
    if (len == 0) {
        detach();
        return;
    }
    buffer_used += len;

    // If there's not enough data for the frame length, wait for more.
    if (buffer_used < sizeof(uint32_t))
        return;

    if (frame_len == 0)
        frame_len = ntohl(*(uint32_t *)buffer);

    DPRINTF(Ethernet, "Received data from peer: len=%d buffer_used=%d "
            "frame_len=%d\n", len, buffer_used, frame_len);

    uint8_t *frame_start = &buffer[sizeof(uint32_t)];
    while (frame_len != 0 && buffer_used >= frame_len + sizeof(uint32_t)) {
        sendSimulated(frame_start, frame_len);

        // Bookkeeping.
        buffer_used -= frame_len + sizeof(uint32_t);
        if (buffer_used > 0) {
            // If there's still any data left, move it into position.
            memmove(buffer, frame_start + frame_len, buffer_used);
        }
        frame_len = 0;

        if (buffer_used >= sizeof(uint32_t))
            frame_len = ntohl(*(uint32_t *)buffer);
    }
}

bool
EtherTapStub::sendReal(const void *data, size_t len)
{
    uint32_t frame_len = htonl(len);
    ssize_t ret = write(socket, &frame_len, sizeof(frame_len));
    if (ret != sizeof(frame_len))
        return false;
    return write(socket, data, len) == len;
}


#if USE_TUNTAP

EtherTap::EtherTap(const Params *p) : EtherTapBase(p)
{
    int fd = open(p->tun_clone_device.c_str(), O_RDWR);
    if (fd < 0)
        panic("Couldn't open %s.\n", p->tun_clone_device);

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    ifr.ifr_flags = IFF_TAP | IFF_NO_PI;
    strncpy(ifr.ifr_name, p->tap_device_name.c_str(), IFNAMSIZ - 1);

    if (ioctl(fd, TUNSETIFF, (void *)&ifr) < 0)
        panic("Failed to access tap device %s.\n", ifr.ifr_name);
    // fd now refers to the tap device.
    tap = fd;
    pollFd(tap);
}

EtherTap::~EtherTap()
{
    stopPolling();
    close(tap);
    tap = -1;
}

void
EtherTap::recvReal(int revent)
{
    if (revent & POLLERR)
        panic("Error polling for tap data.\n");

    if (!(revent & POLLIN))
        return;

    ssize_t ret = read(tap, buffer, buflen);
    if (ret < 0)
        panic("Failed to read from tap device.\n");

    sendSimulated(buffer, ret);
}

bool
EtherTap::sendReal(const void *data, size_t len)
{
    if (write(tap, data, len) != len)
        panic("Failed to write data to tap device.\n");
    return true;
}

EtherTap *
EtherTapParams::create()
{
    return new EtherTap(this);
}

#endif

EtherTapStub *
EtherTapStubParams::create()
{
    return new EtherTapStub(this);
}
