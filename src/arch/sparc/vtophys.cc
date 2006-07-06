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
 * Authors: Nathan Binkert
 *          Steve Reinhardt
 *          Ali Saidi
 */

#include <string>

#include "arch/alpha/ev5.hh"
#include "arch/alpha/vtophys.hh"
#include "base/chunk_generator.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "mem/vport.hh"

using namespace std;
using namespace AlphaISA;

AlphaISA::PageTableEntry
AlphaISA::kernel_pte_lookup(FunctionalPort *mem, Addr ptbr, AlphaISA::VAddr vaddr)
{
    Addr level1_pte = ptbr + vaddr.level1();
    AlphaISA::PageTableEntry level1 = mem->read<uint64_t>(level1_pte);
    if (!level1.valid()) {
        DPRINTF(VtoPhys, "level 1 PTE not valid, va = %#\n", vaddr);
        return 0;
    }

    Addr level2_pte = level1.paddr() + vaddr.level2();
    AlphaISA::PageTableEntry level2 = mem->read<uint64_t>(level2_pte);
    if (!level2.valid()) {
        DPRINTF(VtoPhys, "level 2 PTE not valid, va = %#x\n", vaddr);
        return 0;
    }

    Addr level3_pte = level2.paddr() + vaddr.level3();
    AlphaISA::PageTableEntry level3 = mem->read<uint64_t>(level3_pte);
    if (!level3.valid()) {
        DPRINTF(VtoPhys, "level 3 PTE not valid, va = %#x\n", vaddr);
        return 0;
    }
    return level3;
}

Addr
AlphaISA::vtophys(Addr vaddr)
{
    Addr paddr = 0;
    if (AlphaISA::IsUSeg(vaddr))
        DPRINTF(VtoPhys, "vtophys: invalid vaddr %#x", vaddr);
    else if (AlphaISA::IsK0Seg(vaddr))
        paddr = AlphaISA::K0Seg2Phys(vaddr);
    else
        panic("vtophys: ptbr is not set on virtual lookup");

    DPRINTF(VtoPhys, "vtophys(%#x) -> %#x\n", vaddr, paddr);

    return paddr;
}

Addr
AlphaISA::vtophys(ThreadContext *tc, Addr addr)
{
    AlphaISA::VAddr vaddr = addr;
    Addr ptbr = tc->readMiscReg(AlphaISA::IPR_PALtemp20);
    Addr paddr = 0;
    //@todo Andrew couldn't remember why he commented some of this code
    //so I put it back in. Perhaps something to do with gdb debugging?
    if (AlphaISA::PcPAL(vaddr) && (vaddr < EV5::PalMax)) {
        paddr = vaddr & ~ULL(1);
    } else {
        if (AlphaISA::IsK0Seg(vaddr)) {
            paddr = AlphaISA::K0Seg2Phys(vaddr);
        } else if (!ptbr) {
            paddr = vaddr;
        } else {
            AlphaISA::PageTableEntry pte =
                kernel_pte_lookup(tc->getPhysPort(), ptbr, vaddr);
            if (pte.valid())
                paddr = pte.paddr() | vaddr.offset();
        }
    }


    DPRINTF(VtoPhys, "vtophys(%#x) -> %#x\n", vaddr, paddr);

    return paddr;
}


void
AlphaISA::CopyOut(ThreadContext *tc, void *dest, Addr src, size_t cplen)
{
    uint8_t *dst = (uint8_t *)dest;
    VirtualPort *vp = tc->getVirtPort(tc);

    vp->readBlob(src, dst, cplen);

    tc->delVirtPort(vp);

}

void
AlphaISA::CopyIn(ThreadContext *tc, Addr dest, void *source, size_t cplen)
{
    uint8_t *src = (uint8_t *)source;
    VirtualPort *vp = tc->getVirtPort(tc);

    vp->writeBlob(dest, src, cplen);

    tc->delVirtPort(vp);
}

void
AlphaISA::CopyStringOut(ThreadContext *tc, char *dst, Addr vaddr, size_t maxlen)
{
    int len = 0;
    VirtualPort *vp = tc->getVirtPort(tc);

    do {
        vp->readBlob(vaddr++, (uint8_t*)dst++, 1);
        len++;
    } while (len < maxlen && dst[len] != 0 );

    tc->delVirtPort(vp);
    dst[len] = 0;
}

void
AlphaISA::CopyStringIn(ThreadContext *tc, char *src, Addr vaddr)
{
    VirtualPort *vp = tc->getVirtPort(tc);
    for (ChunkGenerator gen(vaddr, strlen(src), AlphaISA::PageBytes); !gen.done();
            gen.next())
    {
        vp->writeBlob(gen.addr(), (uint8_t*)src, gen.size());
        src += gen.size();
    }
    tc->delVirtPort(vp);
}
