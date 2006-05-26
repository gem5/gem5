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

/**
 * @file Port object definitions.
 */

#include "base/chunk_generator.hh"
#include "mem/packet_impl.hh"
#include "mem/port.hh"

void
Port::blobHelper(Addr addr, uint8_t *p, int size, Packet::Command cmd)
{
    Request req(false);
    Packet pkt(&req, cmd, Packet::Broadcast);

    for (ChunkGenerator gen(addr, size, peerBlockSize());
         !gen.done(); gen.next()) {
        req.setPaddr(gen.addr());
        req.setSize(gen.size());
        pkt.reinitFromRequest();
        pkt.dataStatic(p);
        sendFunctional(&pkt);
        p += gen.size();
    }
}

void
Port::writeBlob(Addr addr, uint8_t *p, int size)
{
    blobHelper(addr, p, size, Packet::WriteReq);
}

void
Port::readBlob(Addr addr, uint8_t *p, int size)
{
    blobHelper(addr, p, size, Packet::ReadReq);
}

void
Port::memsetBlob(Addr addr, uint8_t val, int size)
{
    // quick and dirty...
    uint8_t *buf = new uint8_t[size];

    memset(buf, val, size);
    blobHelper(addr, buf, size, Packet::WriteReq);

    delete [] buf;
}
