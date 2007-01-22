/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
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

#include "arch/sparc/asi.hh"
#include "arch/sparc/miscregfile.hh"
#include "arch/sparc/tlb.hh"
#include "base/bitfield.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "cpu/base.hh"
#include "mem/packet_access.hh"
#include "mem/request.hh"
#include "sim/builder.hh"

/* @todo remove some of the magic constants.  -- ali
 * */
namespace SparcISA
{

TLB::TLB(const std::string &name, int s)
    : SimObject(name), size(s), usedEntries(0), lastReplaced(0),
      cacheValid(false)
{
    // To make this work you'll have to change the hypervisor and OS
    if (size > 64)
        fatal("SPARC T1 TLB registers don't support more than 64 TLB entries.");

    tlb = new TlbEntry[size];
    memset(tlb, 0, sizeof(TlbEntry) * size);

    for (int x = 0; x < size; x++)
        freeList.push_back(&tlb[x]);
}

void
TLB::clearUsedBits()
{
    MapIter i;
    for (i = lookupTable.begin(); i != lookupTable.end(); i++) {
        TlbEntry *t = i->second;
        if (!t->pte.locked()) {
            t->used = false;
            usedEntries--;
        }
    }
}


void
TLB::insert(Addr va, int partition_id, int context_id, bool real,
        const PageTableEntry& PTE, int entry)
{


    MapIter i;
    TlbEntry *new_entry = NULL;
//    TlbRange tr;
    int x;

    cacheValid = false;
    va &= ~(PTE.size()-1);
 /*   tr.va = va;
    tr.size = PTE.size() - 1;
    tr.contextId = context_id;
    tr.partitionId = partition_id;
    tr.real = real;
*/

    DPRINTF(TLB, "TLB: Inserting TLB Entry; va=%#x pa=%#x pid=%d cid=%d r=%d entryid=%d\n",
            va, PTE.paddr(), partition_id, context_id, (int)real, entry);

    // Demap any entry that conflicts
    for (x = 0; x < size; x++) {
        if (tlb[x].range.real == real &&
            tlb[x].range.partitionId == partition_id &&
            tlb[x].range.va < va + PTE.size() - 1 &&
            tlb[x].range.va + tlb[x].range.size >= va &&
            (real || tlb[x].range.contextId == context_id ))
        {
            if (tlb[x].valid) {
                freeList.push_front(&tlb[x]);
                DPRINTF(TLB, "TLB: Conflicting entry %#X , deleting it\n", x);

                tlb[x].valid = false;
                if (tlb[x].used) {
                    tlb[x].used = false;
                    usedEntries--;
                }
                lookupTable.erase(tlb[x].range);
            }
        }
    }


/*
    i = lookupTable.find(tr);
    if (i != lookupTable.end()) {
        i->second->valid = false;
        if (i->second->used) {
            i->second->used = false;
            usedEntries--;
        }
        freeList.push_front(i->second);
        DPRINTF(TLB, "TLB: Found conflicting entry %#X , deleting it\n",
                i->second);
        lookupTable.erase(i);
    }
*/

    if (entry != -1) {
        assert(entry < size && entry >= 0);
        new_entry = &tlb[entry];
    } else {
        if (!freeList.empty()) {
            new_entry = freeList.front();
        } else {
            x = lastReplaced;
            do {
                ++x;
                if (x == size)
                    x = 0;
                if (x == lastReplaced)
                    goto insertAllLocked;
            } while (tlb[x].pte.locked());
            lastReplaced = x;
            new_entry = &tlb[x];
        }
        /*
        for (x = 0; x < size; x++) {
            if (!tlb[x].valid || !tlb[x].used)  {
                new_entry = &tlb[x];
                break;
            }
        }*/
    }

insertAllLocked:
    // Update the last ently if their all locked
    if (!new_entry) {
        new_entry = &tlb[size-1];
    }

    freeList.remove(new_entry);
    if (new_entry->valid && new_entry->used)
        usedEntries--;

    lookupTable.erase(new_entry->range);


    DPRINTF(TLB, "Using entry: %#X\n", new_entry);

    assert(PTE.valid());
    new_entry->range.va = va;
    new_entry->range.size = PTE.size() - 1;
    new_entry->range.partitionId = partition_id;
    new_entry->range.contextId = context_id;
    new_entry->range.real = real;
    new_entry->pte = PTE;
    new_entry->used = true;;
    new_entry->valid = true;
    usedEntries++;



    i = lookupTable.insert(new_entry->range, new_entry);
    assert(i != lookupTable.end());

    // If all entries have there used bit set, clear it on them all, but the
    // one we just inserted
    if (usedEntries == size) {
        clearUsedBits();
        new_entry->used = true;
        usedEntries++;
    }

}


TlbEntry*
TLB::lookup(Addr va, int partition_id, bool real, int context_id)
{
    MapIter i;
    TlbRange tr;
    TlbEntry *t;

    DPRINTF(TLB, "TLB: Looking up entry va=%#x pid=%d cid=%d r=%d\n",
            va, partition_id, context_id, real);
    // Assemble full address structure
    tr.va = va;
    tr.size = MachineBytes;
    tr.contextId = context_id;
    tr.partitionId = partition_id;
    tr.real = real;

    // Try to find the entry
    i = lookupTable.find(tr);
    if (i == lookupTable.end()) {
        DPRINTF(TLB, "TLB: No valid entry found\n");
        return NULL;
    }

    // Mark the entries used bit and clear other used bits in needed
    t = i->second;
    DPRINTF(TLB, "TLB: Valid entry found pa: %#x size: %#x\n", t->pte.paddr(),
            t->pte.size());
    if (!t->used) {
        t->used = true;
        usedEntries++;
        if (usedEntries == size) {
            clearUsedBits();
            t->used = true;
            usedEntries++;
        }
    }

    return t;
}

void
TLB::dumpAll()
{
    MapIter i;
    for (int x = 0; x < size; x++) {
        if (tlb[x].valid) {
           DPRINTFN("%4d:  %#2x:%#2x %c %#4x %#8x %#8x %#16x\n",
                   x, tlb[x].range.partitionId, tlb[x].range.contextId,
                   tlb[x].range.real ? 'R' : ' ', tlb[x].range.size,
                   tlb[x].range.va, tlb[x].pte.paddr(), tlb[x].pte());
        }
    }
}

void
TLB::demapPage(Addr va, int partition_id, bool real, int context_id)
{
    TlbRange tr;
    MapIter i;

    DPRINTF(IPR, "TLB: Demapping Page va=%#x pid=%#d cid=%d r=%d\n",
            va, partition_id, context_id, real);

    cacheValid = false;

    // Assemble full address structure
    tr.va = va;
    tr.size = MachineBytes;
    tr.contextId = context_id;
    tr.partitionId = partition_id;
    tr.real = real;

    // Demap any entry that conflicts
    i = lookupTable.find(tr);
    if (i != lookupTable.end()) {
        DPRINTF(IPR, "TLB: Demapped page\n");
        i->second->valid = false;
        if (i->second->used) {
            i->second->used = false;
            usedEntries--;
        }
        freeList.push_front(i->second);
        DPRINTF(TLB, "Freeing TLB entry : %#X\n", i->second);
        lookupTable.erase(i);
    }
}

void
TLB::demapContext(int partition_id, int context_id)
{
    int x;
    DPRINTF(IPR, "TLB: Demapping Context pid=%#d cid=%d\n",
            partition_id, context_id);
    cacheValid = false;
    for (x = 0; x < size; x++) {
        if (tlb[x].range.contextId == context_id &&
            tlb[x].range.partitionId == partition_id) {
            if (tlb[x].valid == true) {
                freeList.push_front(&tlb[x]);
                DPRINTF(TLB, "Freeing TLB entry : %#X\n", &tlb[x]);
            }
            tlb[x].valid = false;
            if (tlb[x].used) {
                tlb[x].used = false;
                usedEntries--;
            }
            lookupTable.erase(tlb[x].range);
        }
    }
}

void
TLB::demapAll(int partition_id)
{
    int x;
    DPRINTF(TLB, "TLB: Demapping All pid=%#d\n", partition_id);
    cacheValid = false;
    for (x = 0; x < size; x++) {
        if (!tlb[x].pte.locked() && tlb[x].range.partitionId == partition_id) {
            if (tlb[x].valid == true){
                freeList.push_front(&tlb[x]);
                DPRINTF(TLB, "Freeing TLB entry : %#X\n", &tlb[x]);
            }
            tlb[x].valid = false;
            if (tlb[x].used) {
                tlb[x].used = false;
                usedEntries--;
            }
            lookupTable.erase(tlb[x].range);
        }
    }
}

void
TLB::invalidateAll()
{
    int x;
    cacheValid = false;

    freeList.clear();
    lookupTable.clear();
    for (x = 0; x < size; x++) {
        if (tlb[x].valid == true)
            freeList.push_back(&tlb[x]);
        tlb[x].valid = false;
        tlb[x].used = false;
    }
    usedEntries = 0;
}

uint64_t
TLB::TteRead(int entry) {
    if (entry >= size)
        panic("entry: %d\n", entry);

    assert(entry < size);
    if (tlb[entry].valid)
        return tlb[entry].pte();
    else
        return (uint64_t)-1ll;
}

uint64_t
TLB::TagRead(int entry) {
    assert(entry < size);
    uint64_t tag;
    if (!tlb[entry].valid)
        return (uint64_t)-1ll;

    tag = tlb[entry].range.contextId;
    tag |= tlb[entry].range.va;
    tag |= (uint64_t)tlb[entry].range.partitionId << 61;
    tag |= tlb[entry].range.real ? ULL(1) << 60 : 0;
    tag |= (uint64_t)~tlb[entry].pte._size() << 56;
    return tag;
}

bool
TLB::validVirtualAddress(Addr va, bool am)
{
    if (am)
        return true;
    if (va >= StartVAddrHole && va <= EndVAddrHole)
        return false;
    return true;
}

void
TLB::writeSfsr(ThreadContext *tc, int reg,  bool write, ContextType ct,
        bool se, FaultTypes ft, int asi)
{
    uint64_t sfsr;
    sfsr = tc->readMiscReg(reg);

    if (sfsr & 0x1)
        sfsr = 0x3;
    else
        sfsr = 1;

    if (write)
        sfsr |= 1 << 2;
    sfsr |= ct << 4;
    if (se)
        sfsr |= 1 << 6;
    sfsr |= ft << 7;
    sfsr |= asi << 16;
    tc->setMiscRegWithEffect(reg, sfsr);
}

void
TLB::writeTagAccess(ThreadContext *tc, int reg, Addr va, int context)
{
    tc->setMiscRegWithEffect(reg, mbits(va, 63,13) | mbits(context,12,0));
}

void
ITB::writeSfsr(ThreadContext *tc, bool write, ContextType ct,
        bool se, FaultTypes ft, int asi)
{
    DPRINTF(TLB, "TLB: ITB Fault:  w=%d ct=%d ft=%d asi=%d\n",
             (int)write, ct, ft, asi);
    TLB::writeSfsr(tc, MISCREG_MMU_ITLB_SFSR, write, ct, se, ft, asi);
}

void
ITB::writeTagAccess(ThreadContext *tc, Addr va, int context)
{
    TLB::writeTagAccess(tc, MISCREG_MMU_ITLB_TAG_ACCESS, va, context);
}

void
DTB::writeSfr(ThreadContext *tc, Addr a, bool write, ContextType ct,
        bool se, FaultTypes ft, int asi)
{
    DPRINTF(TLB, "TLB: DTB Fault: A=%#x w=%d ct=%d ft=%d asi=%d\n",
            a, (int)write, ct, ft, asi);
    TLB::writeSfsr(tc, MISCREG_MMU_DTLB_SFSR, write, ct, se, ft, asi);
    tc->setMiscRegWithEffect(MISCREG_MMU_DTLB_SFAR, a);
}

void
DTB::writeTagAccess(ThreadContext *tc, Addr va, int context)
{
    TLB::writeTagAccess(tc, MISCREG_MMU_DTLB_TAG_ACCESS, va, context);
}



Fault
ITB::translate(RequestPtr &req, ThreadContext *tc)
{
    uint64_t tlbdata = tc->readMiscReg(MISCREG_TLB_DATA);

    Addr vaddr = req->getVaddr();
    TlbEntry *e;

    assert(req->getAsi() == ASI_IMPLICIT);

    DPRINTF(TLB, "TLB: ITB Request to translate va=%#x size=%d\n",
            vaddr, req->getSize());

    // Be fast if we can!
    if (cacheValid && cacheState == tlbdata) {
        if (cacheEntry) {
            if (cacheEntry->range.va < vaddr + sizeof(MachInst) &&
                cacheEntry->range.va + cacheEntry->range.size >= vaddr) {
                    req->setPaddr(cacheEntry->pte.paddr() & ~(cacheEntry->pte.size()-1) |
                                  vaddr & cacheEntry->pte.size()-1 );
                    return NoFault;
            }
        } else {
            req->setPaddr(vaddr & PAddrImplMask);
            return NoFault;
        }
    }

    bool hpriv = bits(tlbdata,0,0);
    bool red = bits(tlbdata,1,1);
    bool priv = bits(tlbdata,2,2);
    bool addr_mask = bits(tlbdata,3,3);
    bool lsu_im = bits(tlbdata,4,4);

    int part_id = bits(tlbdata,15,8);
    int tl = bits(tlbdata,18,16);
    int pri_context = bits(tlbdata,47,32);
    int context;
    ContextType ct;
    int asi;
    bool real = false;

    DPRINTF(TLB, "TLB: priv:%d hpriv:%d red:%d lsuim:%d part_id: %#X\n",
           priv, hpriv, red, lsu_im, part_id);

    if (tl > 0) {
        asi = ASI_N;
        ct = Nucleus;
        context = 0;
    } else {
        asi = ASI_P;
        ct = Primary;
        context = pri_context;
    }

    if ( hpriv || red ) {
        cacheValid = true;
        cacheState = tlbdata;
        cacheEntry = NULL;
        req->setPaddr(vaddr & PAddrImplMask);
        return NoFault;
    }

    // If the access is unaligned trap
    if (vaddr & 0x3) {
        writeSfsr(tc, false, ct, false, OtherFault, asi);
        return new MemAddressNotAligned;
    }

    if (addr_mask)
        vaddr = vaddr & VAddrAMask;

    if (!validVirtualAddress(vaddr, addr_mask)) {
        writeSfsr(tc, false, ct, false, VaOutOfRange, asi);
        return new InstructionAccessException;
    }

    if (!lsu_im) {
        e = lookup(vaddr, part_id, true);
        real = true;
        context = 0;
    } else {
        e = lookup(vaddr, part_id, false, context);
    }

    if (e == NULL || !e->valid) {
        tc->setMiscReg(MISCREG_MMU_ITLB_TAG_ACCESS,
                vaddr & ~BytesInPageMask | context);
        if (real)
            return new InstructionRealTranslationMiss;
        else
            return new FastInstructionAccessMMUMiss;
    }

    // were not priviledged accesing priv page
    if (!priv && e->pte.priv()) {
        writeSfsr(tc, false, ct, false, PrivViolation, asi);
        return new InstructionAccessException;
    }

    // cache translation date for next translation
    cacheValid = true;
    cacheState = tlbdata;
    cacheEntry = e;

    req->setPaddr(e->pte.paddr() & ~(e->pte.size()-1) |
                  vaddr & e->pte.size()-1 );
    DPRINTF(TLB, "TLB: %#X -> %#X\n", vaddr, req->getPaddr());
    return NoFault;
}



Fault
DTB::translate(RequestPtr &req, ThreadContext *tc, bool write)
{
    /* @todo this could really use some profiling and fixing to make it faster! */
    uint64_t tlbdata = tc->readMiscReg(MISCREG_TLB_DATA);
    Addr vaddr = req->getVaddr();
    Addr size = req->getSize();
    ASI asi;
    asi = (ASI)req->getAsi();
    bool implicit = false;
    bool hpriv = bits(tlbdata,0,0);

    DPRINTF(TLB, "TLB: DTB Request to translate va=%#x size=%d asi=%#x\n",
            vaddr, size, asi);

    if (asi == ASI_IMPLICIT)
        implicit = true;

    if (hpriv && implicit) {
        req->setPaddr(vaddr & PAddrImplMask);
        return NoFault;
    }

    // Be fast if we can!
    if (cacheValid &&  cacheState == tlbdata) {
        if (cacheEntry[0] && cacheAsi[0] == asi && cacheEntry[0]->range.va < vaddr + size &&
            cacheEntry[0]->range.va + cacheEntry[0]->range.size > vaddr) {
                req->setPaddr(cacheEntry[0]->pte.paddr() & ~(cacheEntry[0]->pte.size()-1) |
                              vaddr & cacheEntry[0]->pte.size()-1 );
                return NoFault;
        }
        if (cacheEntry[1] && cacheAsi[1] == asi && cacheEntry[1]->range.va < vaddr + size &&
            cacheEntry[1]->range.va + cacheEntry[1]->range.size > vaddr) {
                req->setPaddr(cacheEntry[1]->pte.paddr() & ~(cacheEntry[1]->pte.size()-1) |
                              vaddr & cacheEntry[1]->pte.size()-1 );
                return NoFault;
        }
    }

    bool red = bits(tlbdata,1,1);
    bool priv = bits(tlbdata,2,2);
    bool addr_mask = bits(tlbdata,3,3);
    bool lsu_dm = bits(tlbdata,5,5);

    int part_id = bits(tlbdata,15,8);
    int tl = bits(tlbdata,18,16);
    int pri_context = bits(tlbdata,47,32);
    int sec_context = bits(tlbdata,47,32);

    bool real = false;
    ContextType ct = Primary;
    int context = 0;

    TlbEntry *e;

    DPRINTF(TLB, "TLB: priv:%d hpriv:%d red:%d lsudm:%d part_id: %#X\n",
           priv, hpriv, red, lsu_dm, part_id);

    if (implicit) {
        if (tl > 0) {
            asi = ASI_N;
            ct = Nucleus;
            context = 0;
        } else {
            asi = ASI_P;
            ct = Primary;
            context = pri_context;
        }
    } else {
        // We need to check for priv level/asi priv
        if (!priv && !hpriv && !AsiIsUnPriv(asi)) {
            // It appears that context should be Nucleus in these cases?
            writeSfr(tc, vaddr, write, Nucleus, false, IllegalAsi, asi);
            return new PrivilegedAction;
        }

        if (!hpriv && AsiIsHPriv(asi)) {
            writeSfr(tc, vaddr, write, Nucleus, false, IllegalAsi, asi);
            return new DataAccessException;
        }

        if (AsiIsPrimary(asi)) {
            context = pri_context;
            ct = Primary;
        } else if (AsiIsSecondary(asi)) {
            context = sec_context;
            ct = Secondary;
        } else if (AsiIsNucleus(asi)) {
            ct = Nucleus;
            context = 0;
        } else {  // ????
            ct = Primary;
            context = pri_context;
        }
    }

    if (!implicit) {
        if (AsiIsLittle(asi))
            panic("Little Endian ASIs not supported\n");
        if (AsiIsBlock(asi))
            panic("Block ASIs not supported\n");
        if (AsiIsNoFault(asi))
            panic("No Fault ASIs not supported\n");

        // These twin ASIs are OK
        if (asi == ASI_P || asi == ASI_LDTX_P)
            goto continueDtbFlow;
        if (!write && (asi == ASI_QUAD_LDD || asi == ASI_LDTX_REAL))
            goto continueDtbFlow;

        if (AsiIsTwin(asi))
            panic("Twin ASIs not supported\n");
        if (AsiIsPartialStore(asi))
            panic("Partial Store ASIs not supported\n");
        if (AsiIsInterrupt(asi))
            panic("Interrupt ASIs not supported\n");

        if (AsiIsMmu(asi))
            goto handleMmuRegAccess;
        if (AsiIsScratchPad(asi))
            goto handleScratchRegAccess;
        if (AsiIsQueue(asi))
            goto handleQueueRegAccess;
        if (AsiIsSparcError(asi))
            goto handleSparcErrorRegAccess;

        if (!AsiIsReal(asi) && !AsiIsNucleus(asi) && !AsiIsAsIfUser(asi))
            panic("Accessing ASI %#X. Should we?\n", asi);
    }

continueDtbFlow:
    // If the asi is unaligned trap
    if (vaddr & size-1) {
        writeSfr(tc, vaddr, false, ct, false, OtherFault, asi);
        return new MemAddressNotAligned;
    }

    if (addr_mask)
        vaddr = vaddr & VAddrAMask;

    if (!validVirtualAddress(vaddr, addr_mask)) {
        writeSfr(tc, vaddr, false, ct, true, VaOutOfRange, asi);
        return new DataAccessException;
    }


    if ((!lsu_dm && !hpriv && !red) || AsiIsReal(asi)) {
        real = true;
        context = 0;
    };

    if (hpriv && (implicit || (!AsiIsAsIfUser(asi) && !AsiIsReal(asi)))) {
        req->setPaddr(vaddr & PAddrImplMask);
        return NoFault;
    }

    e = lookup(vaddr, part_id, real, context);

    if (e == NULL || !e->valid) {
        tc->setMiscReg(MISCREG_MMU_DTLB_TAG_ACCESS,
                vaddr & ~BytesInPageMask | context);
        DPRINTF(TLB, "TLB: DTB Failed to find matching TLB entry\n");
        if (real)
            return new DataRealTranslationMiss;
        else
            return new FastDataAccessMMUMiss;

    }


    if (write && !e->pte.writable()) {
        writeSfr(tc, vaddr, write, ct, e->pte.sideffect(), OtherFault, asi);
        return new FastDataAccessProtection;
    }

    if (e->pte.nofault() && !AsiIsNoFault(asi)) {
        writeSfr(tc, vaddr, write, ct, e->pte.sideffect(), LoadFromNfo, asi);
        return new DataAccessException;
    }

    if (e->pte.sideffect())
        req->setFlags(req->getFlags() | UNCACHEABLE);


    if (!priv && e->pte.priv()) {
        writeSfr(tc, vaddr, write, ct, e->pte.sideffect(), PrivViolation, asi);
        return new DataAccessException;
    }

    // cache translation date for next translation
    cacheState = tlbdata;
    if (!cacheValid) {
        cacheEntry[1] = NULL;
        cacheEntry[0] = NULL;
    }

    if (cacheEntry[0] != e && cacheEntry[1] != e) {
        cacheEntry[1] = cacheEntry[0];
        cacheEntry[0] = e;
        cacheAsi[1] = cacheAsi[0];
        cacheAsi[0] = asi;
        if (implicit)
            cacheAsi[0] = (ASI)0;
    }
    cacheValid = true;
    req->setPaddr(e->pte.paddr() & ~(e->pte.size()-1) |
                  vaddr & e->pte.size()-1);
    DPRINTF(TLB, "TLB: %#X -> %#X\n", vaddr, req->getPaddr());
    return NoFault;
    /** Normal flow ends here. */

handleScratchRegAccess:
    if (vaddr > 0x38 || (vaddr >= 0x20 && vaddr < 0x30 && !hpriv)) {
        writeSfr(tc, vaddr, write, Primary, true, IllegalAsi, asi);
        return new DataAccessException;
    }
    goto regAccessOk;

handleQueueRegAccess:
    if (!priv  && !hpriv) {
        writeSfr(tc, vaddr, write, Primary, true, IllegalAsi, asi);
        return new PrivilegedAction;
    }
    if (!hpriv && vaddr & 0xF || vaddr > 0x3f8 || vaddr < 0x3c0) {
        writeSfr(tc, vaddr, write, Primary, true, IllegalAsi, asi);
        return new DataAccessException;
    }
    goto regAccessOk;

handleSparcErrorRegAccess:
    if (!hpriv) {
        if (priv) {
            writeSfr(tc, vaddr, write, Primary, true, IllegalAsi, asi);
            return new DataAccessException;
        } else {
            writeSfr(tc, vaddr, write, Primary, true, IllegalAsi, asi);
            return new PrivilegedAction;
        }
    }
    goto regAccessOk;


regAccessOk:
handleMmuRegAccess:
    DPRINTF(TLB, "TLB: DTB Translating MM IPR access\n");
    req->setMmapedIpr(true);
    req->setPaddr(req->getVaddr());
    return NoFault;
};

Tick
DTB::doMmuRegRead(ThreadContext *tc, Packet *pkt)
{
    Addr va = pkt->getAddr();
    ASI asi = (ASI)pkt->req->getAsi();
    uint64_t temp, data;
    uint64_t tsbtemp, cnftemp;

    DPRINTF(IPR, "Memory Mapped IPR Read: asi=%#X a=%#x\n",
         (uint32_t)pkt->req->getAsi(), pkt->getAddr());

    switch (asi) {
      case ASI_LSU_CONTROL_REG:
        assert(va == 0);
        pkt->set(tc->readMiscRegWithEffect(MISCREG_MMU_LSU_CTRL));
        break;
      case ASI_MMU:
        switch (va) {
          case 0x8:
            pkt->set(tc->readMiscRegWithEffect(MISCREG_MMU_P_CONTEXT));
            break;
          case 0x10:
            pkt->set(tc->readMiscRegWithEffect(MISCREG_MMU_S_CONTEXT));
            break;
          default:
            goto doMmuReadError;
        }
        break;
      case ASI_QUEUE:
        pkt->set(tc->readMiscRegWithEffect(MISCREG_QUEUE_CPU_MONDO_HEAD +
                    (va >> 4) - 0x3c));
        break;
      case ASI_DMMU_CTXT_ZERO_TSB_BASE_PS0:
        assert(va == 0);
        pkt->set(tc->readMiscRegWithEffect(MISCREG_MMU_DTLB_C0_TSB_PS0));
        break;
      case ASI_DMMU_CTXT_ZERO_TSB_BASE_PS1:
        assert(va == 0);
        pkt->set(tc->readMiscRegWithEffect(MISCREG_MMU_DTLB_C0_TSB_PS1));
        break;
      case ASI_DMMU_CTXT_ZERO_CONFIG:
        assert(va == 0);
        pkt->set(tc->readMiscRegWithEffect(MISCREG_MMU_DTLB_C0_CONFIG));
        break;
      case ASI_IMMU_CTXT_ZERO_TSB_BASE_PS0:
        assert(va == 0);
        pkt->set(tc->readMiscRegWithEffect(MISCREG_MMU_ITLB_C0_TSB_PS0));
        break;
      case ASI_IMMU_CTXT_ZERO_TSB_BASE_PS1:
        assert(va == 0);
        pkt->set(tc->readMiscRegWithEffect(MISCREG_MMU_ITLB_C0_TSB_PS1));
        break;
      case ASI_IMMU_CTXT_ZERO_CONFIG:
        assert(va == 0);
        pkt->set(tc->readMiscRegWithEffect(MISCREG_MMU_ITLB_C0_CONFIG));
        break;
      case ASI_DMMU_CTXT_NONZERO_TSB_BASE_PS0:
        assert(va == 0);
        pkt->set(tc->readMiscRegWithEffect(MISCREG_MMU_DTLB_CX_TSB_PS0));
        break;
      case ASI_DMMU_CTXT_NONZERO_TSB_BASE_PS1:
        assert(va == 0);
        pkt->set(tc->readMiscRegWithEffect(MISCREG_MMU_DTLB_CX_TSB_PS1));
        break;
      case ASI_DMMU_CTXT_NONZERO_CONFIG:
        assert(va == 0);
        pkt->set(tc->readMiscRegWithEffect(MISCREG_MMU_DTLB_CX_CONFIG));
        break;
      case ASI_IMMU_CTXT_NONZERO_TSB_BASE_PS0:
        assert(va == 0);
        pkt->set(tc->readMiscRegWithEffect(MISCREG_MMU_ITLB_CX_TSB_PS0));
        break;
      case ASI_IMMU_CTXT_NONZERO_TSB_BASE_PS1:
        assert(va == 0);
        pkt->set(tc->readMiscRegWithEffect(MISCREG_MMU_ITLB_CX_TSB_PS1));
        break;
      case ASI_IMMU_CTXT_NONZERO_CONFIG:
        assert(va == 0);
        pkt->set(tc->readMiscRegWithEffect(MISCREG_MMU_ITLB_CX_CONFIG));
        break;
      case ASI_SPARC_ERROR_STATUS_REG:
        warn("returning 0 for  SPARC ERROR regsiter read\n");
        pkt->set((uint64_t)0);
        break;
      case ASI_HYP_SCRATCHPAD:
      case ASI_SCRATCHPAD:
        pkt->set(tc->readMiscRegWithEffect(MISCREG_SCRATCHPAD_R0 + (va >> 3)));
        break;
      case ASI_IMMU:
        switch (va) {
          case 0x0:
            temp = tc->readMiscRegWithEffect(MISCREG_MMU_ITLB_TAG_ACCESS);
            pkt->set(bits(temp,63,22) | bits(temp,12,0) << 48);
            break;
          case 0x18:
            pkt->set(tc->readMiscRegWithEffect(MISCREG_MMU_ITLB_SFSR));
            break;
          case 0x30:
            pkt->set(tc->readMiscRegWithEffect(MISCREG_MMU_ITLB_TAG_ACCESS));
            break;
          default:
            goto doMmuReadError;
        }
        break;
      case ASI_DMMU:
        switch (va) {
          case 0x0:
            temp = tc->readMiscRegWithEffect(MISCREG_MMU_DTLB_TAG_ACCESS);
            pkt->set(bits(temp,63,22) | bits(temp,12,0) << 48);
            break;
          case 0x18:
            pkt->set(tc->readMiscRegWithEffect(MISCREG_MMU_DTLB_SFSR));
            break;
          case 0x20:
            pkt->set(tc->readMiscRegWithEffect(MISCREG_MMU_DTLB_SFAR));
            break;
          case 0x30:
            pkt->set(tc->readMiscRegWithEffect(MISCREG_MMU_DTLB_TAG_ACCESS));
            break;
          case 0x80:
            pkt->set(tc->readMiscRegWithEffect(MISCREG_MMU_PART_ID));
            break;
          default:
                goto doMmuReadError;
        }
        break;
      case ASI_DMMU_TSB_PS0_PTR_REG:
        temp = tc->readMiscRegWithEffect(MISCREG_MMU_DTLB_TAG_ACCESS);
        if (bits(temp,12,0) == 0) {
            tsbtemp = tc->readMiscRegWithEffect(MISCREG_MMU_DTLB_C0_TSB_PS0);
            cnftemp = tc->readMiscRegWithEffect(MISCREG_MMU_DTLB_C0_CONFIG);
        } else {
            tsbtemp = tc->readMiscRegWithEffect(MISCREG_MMU_DTLB_CX_TSB_PS0);
            cnftemp = tc->readMiscRegWithEffect(MISCREG_MMU_DTLB_CX_CONFIG);
        }
        data = mbits(tsbtemp,63,13);
        data |= temp >> (9 + bits(cnftemp,2,0) * 3) &
            mbits((uint64_t)-1ll,12+bits(tsbtemp,3,0), 4);
        pkt->set(data);
        break;
      case ASI_DMMU_TSB_PS1_PTR_REG:
        temp = tc->readMiscRegWithEffect(MISCREG_MMU_DTLB_TAG_ACCESS);
        if (bits(temp,12,0) == 0) {
            tsbtemp = tc->readMiscRegWithEffect(MISCREG_MMU_DTLB_C0_TSB_PS1);
            cnftemp = tc->readMiscRegWithEffect(MISCREG_MMU_DTLB_C0_CONFIG);
        } else {
            tsbtemp = tc->readMiscRegWithEffect(MISCREG_MMU_DTLB_CX_TSB_PS1);
            cnftemp = tc->readMiscRegWithEffect(MISCREG_MMU_DTLB_CX_CONFIG);
        }
        data = mbits(tsbtemp,63,13);
        if (bits(tsbtemp,12,12))
            data |= ULL(1) << (13+bits(tsbtemp,3,0));
        data |= temp >> (9 + bits(cnftemp,10,8) * 3) &
            mbits((uint64_t)-1ll,12+bits(tsbtemp,3,0), 4);
        pkt->set(data);
        break;
      case ASI_IMMU_TSB_PS0_PTR_REG:
        temp = tc->readMiscRegWithEffect(MISCREG_MMU_ITLB_TAG_ACCESS);
        if (bits(temp,12,0) == 0) {
            tsbtemp = tc->readMiscRegWithEffect(MISCREG_MMU_ITLB_C0_TSB_PS0);
            cnftemp = tc->readMiscRegWithEffect(MISCREG_MMU_ITLB_C0_CONFIG);
        } else {
            tsbtemp = tc->readMiscRegWithEffect(MISCREG_MMU_ITLB_CX_TSB_PS0);
            cnftemp = tc->readMiscRegWithEffect(MISCREG_MMU_ITLB_CX_CONFIG);
        }
        data = mbits(tsbtemp,63,13);
        data |= temp >> (9 + bits(cnftemp,2,0) * 3) &
            mbits((uint64_t)-1ll,12+bits(tsbtemp,3,0), 4);
        pkt->set(data);
        break;
      case ASI_IMMU_TSB_PS1_PTR_REG:
        temp = tc->readMiscRegWithEffect(MISCREG_MMU_ITLB_TAG_ACCESS);
        if (bits(temp,12,0) == 0) {
            tsbtemp = tc->readMiscRegWithEffect(MISCREG_MMU_ITLB_C0_TSB_PS1);
            cnftemp = tc->readMiscRegWithEffect(MISCREG_MMU_ITLB_C0_CONFIG);
        } else {
            tsbtemp = tc->readMiscRegWithEffect(MISCREG_MMU_ITLB_CX_TSB_PS1);
            cnftemp = tc->readMiscRegWithEffect(MISCREG_MMU_ITLB_CX_CONFIG);
        }
        data = mbits(tsbtemp,63,13);
        if (bits(tsbtemp,12,12))
            data |= ULL(1) << (13+bits(tsbtemp,3,0));
        data |= temp >> (9 + bits(cnftemp,10,8) * 3) &
            mbits((uint64_t)-1ll,12+bits(tsbtemp,3,0), 4);
        pkt->set(data);
        break;

      default:
doMmuReadError:
        panic("need to impl DTB::doMmuRegRead() got asi=%#x, va=%#x\n",
            (uint32_t)asi, va);
    }
    pkt->result = Packet::Success;
    return tc->getCpuPtr()->cycles(1);
}

Tick
DTB::doMmuRegWrite(ThreadContext *tc, Packet *pkt)
{
    uint64_t data = gtoh(pkt->get<uint64_t>());
    Addr va = pkt->getAddr();
    ASI asi = (ASI)pkt->req->getAsi();

    Addr ta_insert;
    Addr va_insert;
    Addr ct_insert;
    int part_insert;
    int entry_insert = -1;
    bool real_insert;
    bool ignore;
    int part_id;
    int ctx_id;
    PageTableEntry pte;

    DPRINTF(IPR, "Memory Mapped IPR Write: asi=%#X a=%#x d=%#X\n",
         (uint32_t)asi, va, data);

    switch (asi) {
      case ASI_LSU_CONTROL_REG:
        assert(va == 0);
        tc->setMiscRegWithEffect(MISCREG_MMU_LSU_CTRL, data);
        break;
      case ASI_MMU:
        switch (va) {
          case 0x8:
            tc->setMiscRegWithEffect(MISCREG_MMU_P_CONTEXT, data);
            break;
          case 0x10:
            tc->setMiscRegWithEffect(MISCREG_MMU_S_CONTEXT, data);
            break;
          default:
            goto doMmuWriteError;
        }
        break;
      case ASI_QUEUE:
        assert(mbits(data,13,6) == data);
        tc->setMiscRegWithEffect(MISCREG_QUEUE_CPU_MONDO_HEAD +
                    (va >> 4) - 0x3c, data);
        break;
      case ASI_DMMU_CTXT_ZERO_TSB_BASE_PS0:
        assert(va == 0);
        tc->setMiscRegWithEffect(MISCREG_MMU_DTLB_C0_TSB_PS0, data);
        break;
      case ASI_DMMU_CTXT_ZERO_TSB_BASE_PS1:
        assert(va == 0);
        tc->setMiscRegWithEffect(MISCREG_MMU_DTLB_C0_TSB_PS1, data);
        break;
      case ASI_DMMU_CTXT_ZERO_CONFIG:
        assert(va == 0);
        tc->setMiscRegWithEffect(MISCREG_MMU_DTLB_C0_CONFIG, data);
        break;
      case ASI_IMMU_CTXT_ZERO_TSB_BASE_PS0:
        assert(va == 0);
        tc->setMiscRegWithEffect(MISCREG_MMU_ITLB_C0_TSB_PS0, data);
        break;
      case ASI_IMMU_CTXT_ZERO_TSB_BASE_PS1:
        assert(va == 0);
        tc->setMiscRegWithEffect(MISCREG_MMU_ITLB_C0_TSB_PS1, data);
        break;
      case ASI_IMMU_CTXT_ZERO_CONFIG:
        assert(va == 0);
        tc->setMiscRegWithEffect(MISCREG_MMU_ITLB_C0_CONFIG, data);
        break;
      case ASI_DMMU_CTXT_NONZERO_TSB_BASE_PS0:
        assert(va == 0);
        tc->setMiscRegWithEffect(MISCREG_MMU_DTLB_CX_TSB_PS0, data);
        break;
      case ASI_DMMU_CTXT_NONZERO_TSB_BASE_PS1:
        assert(va == 0);
        tc->setMiscRegWithEffect(MISCREG_MMU_DTLB_CX_TSB_PS1, data);
        break;
      case ASI_DMMU_CTXT_NONZERO_CONFIG:
        assert(va == 0);
        tc->setMiscRegWithEffect(MISCREG_MMU_DTLB_CX_CONFIG, data);
        break;
      case ASI_IMMU_CTXT_NONZERO_TSB_BASE_PS0:
        assert(va == 0);
        tc->setMiscRegWithEffect(MISCREG_MMU_ITLB_CX_TSB_PS0, data);
        break;
      case ASI_IMMU_CTXT_NONZERO_TSB_BASE_PS1:
        assert(va == 0);
        tc->setMiscRegWithEffect(MISCREG_MMU_ITLB_CX_TSB_PS1, data);
        break;
      case ASI_IMMU_CTXT_NONZERO_CONFIG:
        assert(va == 0);
        tc->setMiscRegWithEffect(MISCREG_MMU_ITLB_CX_CONFIG, data);
        break;
      case ASI_SPARC_ERROR_EN_REG:
      case ASI_SPARC_ERROR_STATUS_REG:
        warn("Ignoring write to SPARC ERROR regsiter\n");
        break;
      case ASI_HYP_SCRATCHPAD:
      case ASI_SCRATCHPAD:
        tc->setMiscRegWithEffect(MISCREG_SCRATCHPAD_R0 + (va >> 3), data);
        break;
      case ASI_IMMU:
        switch (va) {
          case 0x18:
            tc->setMiscRegWithEffect(MISCREG_MMU_ITLB_SFSR, data);
            break;
          case 0x30:
            tc->setMiscRegWithEffect(MISCREG_MMU_ITLB_TAG_ACCESS, data);
            break;
          default:
            goto doMmuWriteError;
        }
        break;
      case ASI_ITLB_DATA_ACCESS_REG:
        entry_insert = bits(va, 8,3);
      case ASI_ITLB_DATA_IN_REG:
        assert(entry_insert != -1 || mbits(va,10,9) == va);
        ta_insert = tc->readMiscRegWithEffect(MISCREG_MMU_ITLB_TAG_ACCESS);
        va_insert = mbits(ta_insert, 63,13);
        ct_insert = mbits(ta_insert, 12,0);
        part_insert = tc->readMiscRegWithEffect(MISCREG_MMU_PART_ID);
        real_insert = bits(va, 9,9);
        pte.populate(data, bits(va,10,10) ? PageTableEntry::sun4v :
                PageTableEntry::sun4u);
        tc->getITBPtr()->insert(va_insert, part_insert, ct_insert, real_insert,
                pte, entry_insert);
        break;
      case ASI_DTLB_DATA_ACCESS_REG:
        entry_insert = bits(va, 8,3);
      case ASI_DTLB_DATA_IN_REG:
        assert(entry_insert != -1 || mbits(va,10,9) == va);
        ta_insert = tc->readMiscRegWithEffect(MISCREG_MMU_DTLB_TAG_ACCESS);
        va_insert = mbits(ta_insert, 63,13);
        ct_insert = mbits(ta_insert, 12,0);
        part_insert = tc->readMiscRegWithEffect(MISCREG_MMU_PART_ID);
        real_insert = bits(va, 9,9);
        pte.populate(data, bits(va,10,10) ? PageTableEntry::sun4v :
                PageTableEntry::sun4u);
        insert(va_insert, part_insert, ct_insert, real_insert, pte, entry_insert);
        break;
      case ASI_IMMU_DEMAP:
        ignore = false;
        ctx_id = -1;
        part_id =  tc->readMiscRegWithEffect(MISCREG_MMU_PART_ID);
        switch (bits(va,5,4)) {
          case 0:
            ctx_id = tc->readMiscRegWithEffect(MISCREG_MMU_P_CONTEXT);
            break;
          case 1:
            ignore = true;
            break;
          case 3:
            ctx_id = 0;
            break;
          default:
            ignore = true;
        }

        switch(bits(va,7,6)) {
          case 0: // demap page
            if (!ignore)
                tc->getITBPtr()->demapPage(mbits(va,63,13), part_id,
                        bits(va,9,9), ctx_id);
            break;
          case 1: //demap context
            if (!ignore)
                tc->getITBPtr()->demapContext(part_id, ctx_id);
            break;
          case 2:
            tc->getITBPtr()->demapAll(part_id);
            break;
          default:
            panic("Invalid type for IMMU demap\n");
        }
        break;
      case ASI_DMMU:
        switch (va) {
          case 0x18:
            tc->setMiscRegWithEffect(MISCREG_MMU_DTLB_SFSR, data);
            break;
          case 0x30:
            tc->setMiscRegWithEffect(MISCREG_MMU_DTLB_TAG_ACCESS, data);
            break;
          case 0x80:
            tc->setMiscRegWithEffect(MISCREG_MMU_PART_ID, data);
            break;
          default:
            goto doMmuWriteError;
        }
        break;
      case ASI_DMMU_DEMAP:
        ignore = false;
        ctx_id = -1;
        part_id =  tc->readMiscRegWithEffect(MISCREG_MMU_PART_ID);
        switch (bits(va,5,4)) {
          case 0:
            ctx_id = tc->readMiscRegWithEffect(MISCREG_MMU_P_CONTEXT);
            break;
          case 1:
            ctx_id = tc->readMiscRegWithEffect(MISCREG_MMU_S_CONTEXT);
            break;
          case 3:
            ctx_id = 0;
            break;
          default:
            ignore = true;
        }

        switch(bits(va,7,6)) {
          case 0: // demap page
            if (!ignore)
                demapPage(mbits(va,63,13), part_id, bits(va,9,9), ctx_id);
            break;
          case 1: //demap context
            if (!ignore)
                demapContext(part_id, ctx_id);
            break;
          case 2:
            demapAll(part_id);
            break;
          default:
            panic("Invalid type for IMMU demap\n");
        }
        break;
      default:
doMmuWriteError:
        panic("need to impl DTB::doMmuRegWrite() got asi=%#x, va=%#x d=%#x\n",
            (uint32_t)pkt->req->getAsi(), pkt->getAddr(), data);
    }
    pkt->result = Packet::Success;
    return tc->getCpuPtr()->cycles(1);
}

void
TLB::serialize(std::ostream &os)
{
    panic("Need to implement serialize tlb for SPARC\n");
}

void
TLB::unserialize(Checkpoint *cp, const std::string &section)
{
    panic("Need to implement unserialize tlb for SPARC\n");
}


DEFINE_SIM_OBJECT_CLASS_NAME("SparcTLB", TLB)

BEGIN_DECLARE_SIM_OBJECT_PARAMS(ITB)

    Param<int> size;

END_DECLARE_SIM_OBJECT_PARAMS(ITB)

BEGIN_INIT_SIM_OBJECT_PARAMS(ITB)

    INIT_PARAM_DFLT(size, "TLB size", 48)

END_INIT_SIM_OBJECT_PARAMS(ITB)


CREATE_SIM_OBJECT(ITB)
{
    return new ITB(getInstanceName(), size);
}

REGISTER_SIM_OBJECT("SparcITB", ITB)

BEGIN_DECLARE_SIM_OBJECT_PARAMS(DTB)

    Param<int> size;

END_DECLARE_SIM_OBJECT_PARAMS(DTB)

BEGIN_INIT_SIM_OBJECT_PARAMS(DTB)

    INIT_PARAM_DFLT(size, "TLB size", 64)

END_INIT_SIM_OBJECT_PARAMS(DTB)


CREATE_SIM_OBJECT(DTB)
{
    return new DTB(getInstanceName(), size);
}

REGISTER_SIM_OBJECT("SparcDTB", DTB)
}
