/*
 * Copyright (c) 2012, 2018 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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

#include "mem/port_proxy.hh"

#include "base/chunk_generator.hh"
#include "cpu/thread_context.hh"
#include "mem/port.hh"

namespace gem5
{

PortProxy::PortProxy(ThreadContext *tc, unsigned int cache_line_size) :
    PortProxy([tc](PacketPtr pkt)->void { tc->sendFunctional(pkt); },
        cache_line_size)
{}

PortProxy::PortProxy(const RequestPort &port, unsigned int cache_line_size) :
    PortProxy([&port](PacketPtr pkt)->void { port.sendFunctional(pkt); },
        cache_line_size)
{}

void
PortProxy::readBlobPhys(Addr addr, Request::Flags flags,
                        void *p, uint64_t size) const
{
    for (ChunkGenerator gen(addr, size, _cacheLineSize); !gen.done();
         gen.next()) {

        auto req = std::make_shared<Request>(
            gen.addr(), gen.size(), flags, Request::funcRequestorId);

        Packet pkt(req, MemCmd::ReadReq);
        pkt.dataStatic(static_cast<uint8_t *>(p));
        sendFunctional(&pkt);
        p = static_cast<uint8_t *>(p) + gen.size();
    }
}

void
PortProxy::writeBlobPhys(Addr addr, Request::Flags flags,
                         const void *p, uint64_t size) const
{
    for (ChunkGenerator gen(addr, size, _cacheLineSize); !gen.done();
         gen.next()) {

        auto req = std::make_shared<Request>(
            gen.addr(), gen.size(), flags, Request::funcRequestorId);

        Packet pkt(req, MemCmd::WriteReq);
        pkt.dataStaticConst(static_cast<const uint8_t *>(p));
        sendFunctional(&pkt);
        p = static_cast<const uint8_t *>(p) + gen.size();
    }
}

void
PortProxy::memsetBlobPhys(Addr addr, Request::Flags flags,
                          uint8_t v, uint64_t size) const
{
    // quick and dirty...
    uint8_t *buf = new uint8_t[size];

    std::memset(buf, v, size);
    PortProxy::writeBlobPhys(addr, flags, buf, size);

    delete [] buf;
}

bool
PortProxy::tryWriteString(Addr addr, const char *str) const
{
    do {
        if (!tryWriteBlob(addr++, str, 1))
            return false;
    } while (*str++);
    return true;
}

bool
PortProxy::tryReadString(std::string &str, Addr addr) const
{
    while (true) {
        uint8_t c;
        if (!tryReadBlob(addr++, &c, 1))
            return false;
        if (!c)
            return true;
        str += c;
    }
}

bool
PortProxy::tryReadString(char *str, Addr addr, size_t maxlen) const
{
    assert(maxlen);
    while (maxlen--) {
        if (!tryReadBlob(addr++, str, 1))
            return false;
        if (!*str++)
            return true;
    }
    // We ran out of room, so back up and add a terminator.
    *--str = '\0';
    return true;
}

} // namespace gem5
