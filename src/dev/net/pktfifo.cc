/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

#include "dev/net/pktfifo.hh"

#include "base/logging.hh"

using namespace std;

bool
PacketFifo::copyout(void *dest, unsigned offset, unsigned len)
{
    char *data = (char *)dest;
    if (offset + len >= size())
        return false;

    iterator i = fifo.begin();
    iterator end = fifo.end();
    while (len > 0) {
        EthPacketPtr &pkt = i->packet;
        while (offset >= pkt->length) {
            offset -= pkt->length;
            ++i;
        }

        if (i == end)
            panic("invalid fifo");

        unsigned size = min(pkt->length - offset, len);
        memcpy(data, pkt->data, size);
        offset = 0;
        len -= size;
        data += size;
        ++i;
    }

    return true;
}


void
PacketFifoEntry::serialize(const string &base, CheckpointOut &cp) const
{
    packet->serialize(base + ".packet", cp);
    paramOut(cp, base + ".slack", slack);
    paramOut(cp, base + ".number", number);
    paramOut(cp, base + ".priv", priv);
}

void
PacketFifoEntry::unserialize(const string &base, CheckpointIn &cp)
{
    packet = make_shared<EthPacketData>();
    packet->unserialize(base + ".packet", cp);
    paramIn(cp, base + ".slack", slack);
    paramIn(cp, base + ".number", number);
    paramIn(cp, base + ".priv", priv);
}

void
PacketFifo::serialize(const string &base, CheckpointOut &cp) const
{
    paramOut(cp, base + ".size", _size);
    paramOut(cp, base + ".maxsize", _maxsize);
    paramOut(cp, base + ".reserved", _reserved);
    paramOut(cp, base + ".packets", fifo.size());

    int i = 0;
    for (const auto &entry : fifo)
        entry.serialize(csprintf("%s.entry%d", base, i++), cp);
}

void
PacketFifo::unserialize(const string &base, CheckpointIn &cp)
{
    paramIn(cp, base + ".size", _size);
//  paramIn(cp, base + ".maxsize", _maxsize);
    paramIn(cp, base + ".reserved", _reserved);
    int fifosize;
    paramIn(cp, base + ".packets", fifosize);

    fifo.clear();

    for (int i = 0; i < fifosize; ++i) {
        PacketFifoEntry entry;
        entry.unserialize(csprintf("%s.entry%d", base, i), cp);
        fifo.push_back(entry);
    }
}
