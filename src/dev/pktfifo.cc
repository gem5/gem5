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

#include "base/misc.hh"
#include "dev/pktfifo.hh"

using namespace std;

bool
PacketFifo::copyout(void *dest, int offset, int len)
{
    char *data = (char *)dest;
    if (offset + len >= size())
        return false;

    list<EthPacketPtr>::iterator p = fifo.begin();
    list<EthPacketPtr>::iterator end = fifo.end();
    while (len > 0) {
        while (offset >= (*p)->length) {
            offset -= (*p)->length;
            ++p;
        }

        if (p == end)
            panic("invalid fifo");

        int size = min((*p)->length - offset, len);
        memcpy(data, (*p)->data, size);
        offset = 0;
        len -= size;
        data += size;
        ++p;
    }

    return true;
}


void
PacketFifo::serialize(const string &base, ostream &os)
{
    paramOut(os, base + ".size", _size);
    paramOut(os, base + ".maxsize", _maxsize);
    paramOut(os, base + ".reserved", _reserved);
    paramOut(os, base + ".packets", fifo.size());

    int i = 0;
    list<EthPacketPtr>::iterator p = fifo.begin();
    list<EthPacketPtr>::iterator end = fifo.end();
    while (p != end) {
        (*p)->serialize(csprintf("%s.packet%d", base, i), os);
        ++p;
        ++i;
    }
}

void
PacketFifo::unserialize(const string &base, Checkpoint *cp,
                        const string &section)
{
    paramIn(cp, section, base + ".size", _size);
//  paramIn(cp, section, base + ".maxsize", _maxsize);
    paramIn(cp, section, base + ".reserved", _reserved);
    int fifosize;
    paramIn(cp, section, base + ".packets", fifosize);

    fifo.clear();

    for (int i = 0; i < fifosize; ++i) {
        EthPacketPtr p = new EthPacketData(16384);
        p->unserialize(csprintf("%s.packet%d", base, i), cp, section);
        fifo.push_back(p);
    }
}
