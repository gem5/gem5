/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#include <string>

#include "targetarch/pmap.h"

#include "cpu/exec_context.hh"
#include "mem/functional_mem/physical_memory.hh"
#include "base/trace.hh"
#include "targetarch/vtophys.hh"

using namespace std;

inline Addr
level3_index(Addr vaddr)
{ return (vaddr >> ALPHA_PGSHIFT) & PTEMASK; }

inline Addr
level2_index(Addr vaddr)
{ return (vaddr >> (ALPHA_PGSHIFT + NPTEPG_SHIFT)) & PTEMASK; }

inline Addr
level1_index(Addr vaddr)
{ return (vaddr >> (ALPHA_PGSHIFT + 2 * NPTEPG_SHIFT)) & PTEMASK; }

Addr
kernel_pte_lookup(PhysicalMemory *pmem, Addr ptbr, Addr vaddr)
{
    uint64_t level1_map = ptbr;
    Addr level1_pte = level1_map + (level1_index(vaddr) << PTESHIFT);

    uint64_t level1 = pmem->phys_read_qword(level1_pte);
    if (!entry_valid(level1)) {
        DPRINTF(VtoPhys, "level 1 PTE not valid, va = %#\n", vaddr);
        return 0;
    }

    uint64_t level2_map = PMAP_PTE_PA(level1);
    Addr level2_pte = level2_map + (level2_index(vaddr) << PTESHIFT);
    uint64_t level2 = pmem->phys_read_qword(level2_pte);
    if (!entry_valid(level2)) {
        DPRINTF(VtoPhys, "level 2 PTE not valid, va = %#x\n", vaddr);
        return 0;
    }

    uint64_t level3_map = PMAP_PTE_PA(level2);
    Addr level3_pte = level3_map + (level3_index(vaddr) << PTESHIFT);

    return level3_pte;
}

Addr
vtophys(PhysicalMemory *xc, Addr vaddr)
{
    Addr paddr = 0;
    if (vaddr < ALPHA_K0SEG_BASE)
        DPRINTF(VtoPhys, "vtophys: invalid vaddr %#x", vaddr);
    else if (vaddr < ALPHA_K1SEG_BASE)
        paddr = ALPHA_K0SEG_TO_PHYS(vaddr);
    else
        panic("vtophys: ptbr is not set on virtual lookup");

    DPRINTF(VtoPhys, "vtophys(%#x) -> %#x\n", vaddr, paddr);

    return paddr;
}

Addr
vtophys(ExecContext *xc, Addr vaddr)
{
    Addr ptbr = xc->regs.ipr[AlphaISA::IPR_PALtemp20];
    Addr paddr = 0;
    if (vaddr < ALPHA_K0SEG_BASE) {
        DPRINTF(VtoPhys, "vtophys: invalid vaddr %#x", vaddr);
    } else if (vaddr < ALPHA_K1SEG_BASE) {
        paddr = ALPHA_K0SEG_TO_PHYS(vaddr);
    } else {
        if (!ptbr)
            panic("vtophys: ptbr is not set on virtual lookup");

        Addr pte = kernel_pte_lookup(xc->physmem, ptbr, vaddr);
        uint64_t entry = xc->physmem->phys_read_qword(pte);
        if (pte && entry_valid(entry))
            paddr = PMAP_PTE_PA(entry) | (vaddr & PGOFSET);
    }

    DPRINTF(VtoPhys, "vtophys(%#x) -> %#x\n", vaddr, paddr);

    return paddr;
}

uint8_t *
vtomem(ExecContext *xc, Addr vaddr, size_t len)
{
    Addr paddr = vtophys(xc, vaddr);
    return xc->physmem->dma_addr(paddr, len);
}
