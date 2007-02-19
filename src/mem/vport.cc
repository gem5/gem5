/*
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
 */

/**
 * @file
 * Port object definitions.
 */

#include "base/chunk_generator.hh"
#include "cpu/thread_context.hh"
#include "mem/vport.hh"

void
VirtualPort::readBlob(Addr addr, uint8_t *p, int size)
{
    Addr paddr;
    for (ChunkGenerator gen(addr, size, TheISA::PageBytes); !gen.done();
            gen.next())
    {
        if (tc)
            paddr = TheISA::vtophys(tc,gen.addr());
        else
            paddr = TheISA::vtophys(gen.addr());

        FunctionalPort::readBlob(paddr, p, gen.size());
        p += gen.size();
    }
}

void
VirtualPort::writeBlob(Addr addr, uint8_t *p, int size)
{
    Addr paddr;
    for (ChunkGenerator gen(addr, size, TheISA::PageBytes); !gen.done();
            gen.next())
    {
        if (tc)
            paddr = TheISA::vtophys(tc,gen.addr());
        else
            paddr = TheISA::vtophys(gen.addr());

        FunctionalPort::writeBlob(paddr, p, gen.size());
        p += gen.size();
    }
}

void
CopyOut(ThreadContext *tc, void *dest, Addr src, size_t cplen)
{
    uint8_t *dst = (uint8_t *)dest;
    VirtualPort *vp = tc->getVirtPort(tc);

    vp->readBlob(src, dst, cplen);

    tc->delVirtPort(vp);

}

void
CopyIn(ThreadContext *tc, Addr dest, void *source, size_t cplen)
{
    uint8_t *src = (uint8_t *)source;
    VirtualPort *vp = tc->getVirtPort(tc);

    vp->writeBlob(dest, src, cplen);

    tc->delVirtPort(vp);
}

void
CopyStringOut(ThreadContext *tc, char *dst, Addr vaddr, size_t maxlen)
{
    int len = 0;
    char *start = dst;
    VirtualPort *vp = tc->getVirtPort(tc);

    do {
        vp->readBlob(vaddr++, (uint8_t*)dst++, 1);
    } while (len < maxlen && start[len++] != 0 );

    tc->delVirtPort(vp);
    dst[len] = 0;
}

void
CopyStringIn(ThreadContext *tc, char *src, Addr vaddr)
{
    VirtualPort *vp = tc->getVirtPort(tc);
    for (ChunkGenerator gen(vaddr, strlen(src), TheISA::PageBytes); !gen.done();
            gen.next())
    {
        vp->writeBlob(gen.addr(), (uint8_t*)src, gen.size());
        src += gen.size();
    }
    tc->delVirtPort(vp);
}
