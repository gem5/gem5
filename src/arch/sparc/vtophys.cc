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
 * Authors: Ali Saidi
 */

#include <string>

#include "arch/sparc/tlb.hh"
#include "arch/sparc/vtophys.hh"
#include "base/chunk_generator.hh"
#include "base/compiler.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "debug/VtoPhys.hh"
#include "mem/port_proxy.hh"

using namespace std;

namespace SparcISA {

Addr
vtophys(Addr vaddr)
{
    // In SPARC it's almost always impossible to turn a VA->PA w/o a
    // context The only times we can kinda do it are if we have a
    // SegKPM mapping and can find the real address in the tlb or we
    // have a physical adddress already (beacuse we are looking at the
    // hypervisor) Either case is rare, so we'll just panic.

    panic("vtophys() without context on SPARC largly worthless\n");
    M5_DUMMY_RETURN;
}

Addr
vtophys(ThreadContext *tc, Addr addr)
{
    // Here we have many options and are really implementing something like
    // a fill handler to find the address since there isn't a multilevel
    // table for us to walk around.
    //
    // 1. We are currently hyperpriv, return the address unmodified
    // 2. The mmu is off return(ra->pa)
    // 3. We are currently priv, use ctx0* tsbs to find the page
    // 4. We are not priv, use ctxN0* tsbs to find the page
    // For all accesses we check the tlbs first since it's possible that
    // long standing pages (e.g. locked kernel mappings) won't be in the tsb
    uint64_t tlbdata = tc->readMiscRegNoEffect(MISCREG_TLB_DATA);

    bool hpriv = bits(tlbdata,0,0);
    // bool priv = bits(tlbdata,2,2);
    bool addr_mask = bits(tlbdata,3,3);
    bool data_real = !bits(tlbdata,5,5);
    bool inst_real = !bits(tlbdata,4,4);
    bool ctx_zero  = bits(tlbdata,18,16) > 0;
    int part_id = bits(tlbdata,15,8);
    int pri_context = bits(tlbdata,47,32);
    // int sec_context = bits(tlbdata,63,48);

    PortProxy &mem = tc->getPhysProxy();
    TLB* itb = tc->getITBPtr();
    TLB* dtb = tc->getDTBPtr();
    TlbEntry* tbe;
    PageTableEntry pte;
    Addr tsbs[4];
    Addr va_tag;
    TteTag ttetag;

    if (hpriv)
        return addr;

    if (addr_mask)
        addr = addr & VAddrAMask;

    tbe = dtb->lookup(addr, part_id, data_real, ctx_zero ? 0 : pri_context ,
                      false);
    if (tbe)
        goto foundtbe;

    tbe = itb->lookup(addr, part_id, inst_real, ctx_zero ? 0 : pri_context,
                      false);
    if (tbe)
        goto foundtbe;

    // We didn't find it in the tlbs, so lets look at the TSBs
    dtb->GetTsbPtr(tc, addr, ctx_zero ? 0 : pri_context, tsbs);
    va_tag = bits(addr, 63, 22);
    for (int x = 0; x < 4; x++) {
        ttetag = betoh(mem.read<uint64_t>(tsbs[x]));
        if (ttetag.valid() && ttetag.va() == va_tag) {
            uint64_t entry = mem.read<uint64_t>(tsbs[x]) + sizeof(uint64_t);
            // I think it's sun4v at least!
            pte.populate(betoh(entry), PageTableEntry::sun4v);
            DPRINTF(VtoPhys, "Virtual(%#x)->Physical(%#x) found in TTE\n",
                    addr, pte.translate(addr));
            goto foundpte;
        }
    }
    panic("couldn't translate %#x\n", addr);

  foundtbe:
    pte = tbe->pte;
    DPRINTF(VtoPhys, "Virtual(%#x)->Physical(%#x) found in TLB\n", addr,
            pte.translate(addr));
  foundpte:
    return pte.translate(addr);
}

} // namespace SparcISA
