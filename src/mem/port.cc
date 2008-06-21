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
 * Authors: Steve Reinhardt
 */

/**
 * @file
 * Port object definitions.
 */
#include <cstring>

#include "base/chunk_generator.hh"
#include "base/trace.hh"
#include "mem/mem_object.hh"
#include "mem/port.hh"

/**
 * Special class for port objects that are used as peers for
 * unconnected ports.  Assigning instances of this class to newly
 * allocated ports allows us to guarantee that every port has a peer
 * object (so there's no need to check for null peer pointers), while
 * catching uses of unconnected ports.
 */
class DefaultPeerPort : public Port
{
  protected:
    void blowUp()
    {
        Port *peer = getPeer();
        fatal("unconnected port: %s", peer ? peer->name() : "<unknown>");
    }

  public:
    DefaultPeerPort(Port *_peer)
        : Port("default_port", NULL, _peer)
    { }

    bool recvTiming(PacketPtr)
    {
        blowUp();
        return false;
    }

    Tick recvAtomic(PacketPtr)
    {
        blowUp();
        return 0;
    }

    void recvFunctional(PacketPtr)
    {
        blowUp();
    }

    void recvStatusChange(Status)
    {
        blowUp();
    }

    int deviceBlockSize()
    {
        blowUp();
        return 0;
    }

    void getDeviceAddressRanges(AddrRangeList &, bool &)
    {
        blowUp();
    }

    bool isDefaultPort() const { return true; }
};


Port::Port(const std::string &_name, MemObject *_owner, Port *_peer) :
    portName(_name),
    peer(_peer ? _peer : new DefaultPeerPort(this)),
    owner(_owner)
{
}

Port::~Port()
{
    disconnectFromPeer();
}

void
Port::disconnectFromPeer()
{
    if (peer) {
        assert(peer->getPeer() == this);
        peer->disconnect();
    }
}

void
Port::disconnect()
{
    // This notification should come only from our peer, so we must
    // have one,
    assert(peer != NULL);
    // We must clear 'peer' here, else if owner->deletePort() calls
    // delete on us then we'll recurse infinitely through the Port
    // destructor.
    peer = NULL;
    // If owner->deletePort() returns true, then we've been deleted,
    // so don't do anything but get out of here.  If not, reset peer
    // pointer to a DefaultPeerPort.
    if (!(owner && owner->deletePort(this)))
        peer = new DefaultPeerPort(this);
}

void
Port::setPeer(Port *port)
{
    DPRINTF(Config, "setting peer to %s, old peer %s\n",
            port->name(), peer ? peer->name() : "<null>");
    
    // You'd think we'd want to disconnect from the previous peer
    // here, but it turns out that with some functional ports the old
    // peer keeps using the connection, and it works because
    // functional ports are unidirectional.
    //
    // disconnectFromPeer();

    peer = port;
}

void
Port::blobHelper(Addr addr, uint8_t *p, int size, MemCmd cmd)
{
    Request req;

    for (ChunkGenerator gen(addr, size, peerBlockSize());
         !gen.done(); gen.next()) {
        req.setPhys(gen.addr(), gen.size(), 0);
        Packet pkt(&req, cmd, Packet::Broadcast);
        pkt.dataStatic(p);
        sendFunctional(&pkt);
        p += gen.size();
    }
}

void
Port::writeBlob(Addr addr, uint8_t *p, int size)
{
    blobHelper(addr, p, size, MemCmd::WriteReq);
}

void
Port::readBlob(Addr addr, uint8_t *p, int size)
{
    blobHelper(addr, p, size, MemCmd::ReadReq);
}

void
Port::memsetBlob(Addr addr, uint8_t val, int size)
{
    // quick and dirty...
    uint8_t *buf = new uint8_t[size];

    std::memset(buf, val, size);
    blobHelper(addr, buf, size, MemCmd::WriteReq);

    delete [] buf;
}


void
Port::printAddr(Addr a)
{
    Request req(a, 1, 0);
    Packet pkt(&req, MemCmd::PrintReq, Packet::Broadcast);
    Packet::PrintReqState prs(std::cerr);
    pkt.senderState = &prs;

    sendFunctional(&pkt);
}
