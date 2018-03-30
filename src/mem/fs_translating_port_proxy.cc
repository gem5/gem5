/*
 * Copyright (c) 2011,2013 ARM Limited
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
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 * Authors: Ali Saidi
 *          Andreas Hansson
 */

/**
 * @file
 * Port object definitions.
 */

#include "mem/fs_translating_port_proxy.hh"

#include "arch/vtophys.hh"
#include "base/chunk_generator.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "sim/system.hh"

FSTranslatingPortProxy::FSTranslatingPortProxy(ThreadContext *tc)
    : PortProxy(tc->getCpuPtr()->getDataPort(),
                tc->getSystemPtr()->cacheLineSize()), _tc(tc)
{
}

FSTranslatingPortProxy::FSTranslatingPortProxy(MasterPort &port,
                                               unsigned int cacheLineSize)
    : PortProxy(port, cacheLineSize), _tc(NULL)
{
}

FSTranslatingPortProxy::~FSTranslatingPortProxy()
{
}

void
FSTranslatingPortProxy::readBlob(Addr addr, uint8_t *p, int size) const
{
    Addr paddr;
    for (ChunkGenerator gen(addr, size, TheISA::PageBytes); !gen.done();
         gen.next())
    {
        if (_tc)
            paddr = TheISA::vtophys(_tc,gen.addr());
        else
            paddr = TheISA::vtophys(gen.addr());

        PortProxy::readBlobPhys(paddr, 0, p, gen.size());
        p += gen.size();
    }
}

void
FSTranslatingPortProxy::writeBlob(Addr addr, const uint8_t *p, int size) const
{
    Addr paddr;
    for (ChunkGenerator gen(addr, size, TheISA::PageBytes); !gen.done();
         gen.next())
    {
        if (_tc)
            paddr = TheISA::vtophys(_tc,gen.addr());
        else
            paddr = TheISA::vtophys(gen.addr());

        PortProxy::writeBlobPhys(paddr, 0, p, gen.size());
        p += gen.size();
    }
}

void
FSTranslatingPortProxy::memsetBlob(Addr address, uint8_t v, int size) const
{
    Addr paddr;
    for (ChunkGenerator gen(address, size, TheISA::PageBytes); !gen.done();
         gen.next())
    {
        if (_tc)
            paddr = TheISA::vtophys(_tc,gen.addr());
        else
            paddr = TheISA::vtophys(gen.addr());

        PortProxy::memsetBlobPhys(paddr, 0, v, gen.size());
    }
}

void
CopyOut(ThreadContext *tc, void *dest, Addr src, size_t cplen)
{
    uint8_t *dst = (uint8_t *)dest;
    tc->getVirtProxy().readBlob(src, dst, cplen);
}

void
CopyIn(ThreadContext *tc, Addr dest, const void *source, size_t cplen)
{
    uint8_t *src = (uint8_t *)source;
    tc->getVirtProxy().writeBlob(dest, src, cplen);
}

void
CopyStringOut(ThreadContext *tc, char *dst, Addr vaddr, size_t maxlen)
{
    char *start = dst;
    FSTranslatingPortProxy &vp = tc->getVirtProxy();

    bool foundNull = false;
    while ((dst - start + 1) < maxlen && !foundNull) {
        vp.readBlob(vaddr++, (uint8_t*)dst, 1);
        if (*dst == '\0')
            foundNull = true;
        dst++;
    }

    if (!foundNull)
        *dst = '\0';
}

void
CopyStringIn(ThreadContext *tc, const char *src, Addr vaddr)
{
    FSTranslatingPortProxy &vp = tc->getVirtProxy();
    for (ChunkGenerator gen(vaddr, strlen(src), TheISA::PageBytes); !gen.done();
         gen.next())
    {
        vp.writeBlob(gen.addr(), (uint8_t*)src, gen.size());
        src += gen.size();
    }
}
