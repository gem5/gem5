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
 */

#include "arch/sparc/tlb.hh"

#include <cstring>

#include "arch/sparc/asi.hh"
#include "arch/sparc/faults.hh"
#include "arch/sparc/interrupts.hh"
#include "arch/sparc/mmu.hh"
#include "arch/sparc/regs/misc.hh"
#include "arch/sparc/types.hh"
#include "base/bitfield.hh"
#include "base/compiler.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/IPR.hh"
#include "debug/TLB.hh"
#include "mem/packet_access.hh"
#include "mem/page_table.hh"
#include "mem/request.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"
#include "sim/system.hh"

namespace gem5
{

/* @todo remove some of the magic constants.  -- ali
 * */
namespace SparcISA
{

TLB::TLB(const Params &p)
    : BaseTLB(p),
      size(p.size),
      usedEntries(0),
      lastReplaced(0),
      cacheState(0),
      cacheValid(false)
{
    // To make this work you'll have to change the hypervisor and OS
    if (size > 64)
        fatal("SPARC T1 TLB registers don't support more than 64 TLB entries");

    tlb = new TlbEntry[size];
    std::memset((void *)tlb, 0, sizeof(TlbEntry) * size);

    for (int x = 0; x < size; x++)
        freeList.push_back(&tlb[x]);

    c0_tsb_ps0 = 0;
    c0_tsb_ps1 = 0;
    c0_config = 0;
    cx_tsb_ps0 = 0;
    cx_tsb_ps1 = 0;
    cx_config = 0;
    sfsr = 0;
    tag_access = 0;
    sfar = 0;
    cacheEntry[0] = NULL;
    cacheEntry[1] = NULL;
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
            const PageTableEntry &PTE, int entry)
{
    MapIter i;
    TlbEntry *new_entry = NULL;
    int x;

    cacheValid = false;
    va &= ~(PTE.size() - 1);

    DPRINTF(
        TLB,
        "TLB: Inserting Entry; va=%#x pa=%#x pid=%d cid=%d r=%d entryid=%d\n",
        va, PTE.paddr(), partition_id, context_id, (int)real, entry);

    // Demap any entry that conflicts
    for (x = 0; x < size; x++) {
        if (tlb[x].range.real == real &&
            tlb[x].range.partitionId == partition_id &&
            tlb[x].range.va < va + PTE.size() - 1 &&
            tlb[x].range.va + tlb[x].range.size >= va &&
            (real || tlb[x].range.contextId == context_id)) {
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
    }

insertAllLocked:
    // Update the last ently if their all locked
    if (!new_entry) {
        new_entry = &tlb[size - 1];
    }

    freeList.remove(new_entry);
    if (new_entry->valid && new_entry->used)
        usedEntries--;
    if (new_entry->valid)
        lookupTable.erase(new_entry->range);

    assert(PTE.valid());
    new_entry->range.va = va;
    new_entry->range.size = PTE.size() - 1;
    new_entry->range.partitionId = partition_id;
    new_entry->range.contextId = context_id;
    new_entry->range.real = real;
    new_entry->pte = PTE;
    new_entry->used = true;
    ;
    new_entry->valid = true;
    usedEntries++;

    i = lookupTable.insert(new_entry->range, new_entry);
    assert(i != lookupTable.end());

    // If all entries have their used bit set, clear it on them all,
    // but the one we just inserted
    if (usedEntries == size) {
        clearUsedBits();
        new_entry->used = true;
        usedEntries++;
    }
}

TlbEntry *
TLB::lookup(Addr va, int partition_id, bool real, int context_id,
            bool update_used)
{
    MapIter i;
    TlbRange tr;
    TlbEntry *t;

    DPRINTF(TLB, "TLB: Looking up entry va=%#x pid=%d cid=%d r=%d\n", va,
            partition_id, context_id, real);
    // Assemble full address structure
    tr.va = va;
    tr.size = 1;
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

    // Update the used bits only if this is a real access (not a fake
    // one from virttophys()
    if (!t->used && update_used) {
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
            DPRINTFN("%4d:  %#2x:%#2x %c %#4x %#8x %#8x %#16x\n", x,
                     tlb[x].range.partitionId, tlb[x].range.contextId,
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

    DPRINTF(IPR, "TLB: Demapping Page va=%#x pid=%#d cid=%d r=%d\n", va,
            partition_id, context_id, real);

    cacheValid = false;

    // Assemble full address structure
    tr.va = va;
    tr.size = 1;
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
        lookupTable.erase(i);
    }
}

void
TLB::demapContext(int partition_id, int context_id)
{
    DPRINTF(IPR, "TLB: Demapping Context pid=%#d cid=%d\n", partition_id,
            context_id);
    cacheValid = false;
    for (int x = 0; x < size; x++) {
        if (tlb[x].range.contextId == context_id &&
            tlb[x].range.partitionId == partition_id) {
            if (tlb[x].valid) {
                freeList.push_front(&tlb[x]);
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
    DPRINTF(TLB, "TLB: Demapping All pid=%#d\n", partition_id);
    cacheValid = false;
    for (int x = 0; x < size; x++) {
        if (tlb[x].valid && !tlb[x].pte.locked() &&
            tlb[x].range.partitionId == partition_id) {
            freeList.push_front(&tlb[x]);
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
TLB::flushAll()
{
    cacheValid = false;
    lookupTable.clear();

    for (int x = 0; x < size; x++) {
        if (tlb[x].valid)
            freeList.push_back(&tlb[x]);
        tlb[x].valid = false;
        tlb[x].used = false;
    }
    usedEntries = 0;
}

uint64_t
TLB::TteRead(int entry)
{
    if (entry >= size)
        panic("entry: %d\n", entry);

    assert(entry < size);
    if (tlb[entry].valid)
        return tlb[entry].pte();
    else
        return (uint64_t)-1ll;
}

uint64_t
TLB::TagRead(int entry)
{
    assert(entry < size);
    uint64_t tag;
    if (!tlb[entry].valid)
        return (uint64_t)-1ll;

    tag = tlb[entry].range.contextId;
    tag |= tlb[entry].range.va;
    tag |= (uint64_t)tlb[entry].range.partitionId << 61;
    tag |= tlb[entry].range.real ? 1ULL << 60 : 0;
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
TLB::writeSfsr(bool write, ContextType ct, bool se, FaultTypes ft, int asi)
{
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
}

void
TLB::writeTagAccess(Addr va, int context)
{
    DPRINTF(TLB, "TLB: Writing Tag Access: va: %#X ctx: %#X value: %#X\n", va,
            context, mbits(va, 63, 13) | mbits(context, 12, 0));

    tag_access = mbits(va, 63, 13) | mbits(context, 12, 0);
}

void
TLB::writeSfsr(Addr a, bool write, ContextType ct, bool se, FaultTypes ft,
               int asi)
{
    DPRINTF(TLB, "TLB: Fault: A=%#x w=%d ct=%d ft=%d asi=%d\n", a, (int)write,
            ct, ft, asi);
    TLB::writeSfsr(write, ct, se, ft, asi);
    sfar = a;
}

Fault
TLB::translateInst(const RequestPtr &req, ThreadContext *tc)
{
    uint64_t tlbdata = tc->readMiscRegNoEffect(MISCREG_TLB_DATA);

    Addr vaddr = req->getVaddr();
    TlbEntry *e;

    assert(req->getArchFlags() == ASI_IMPLICIT);

    DPRINTF(TLB, "TLB: ITB Request to translate va=%#x size=%d\n", vaddr,
            req->getSize());

    // Be fast if we can!
    if (cacheValid && cacheState == tlbdata) {
        if (cacheEntry[0]) {
            if (cacheEntry[0]->range.va < vaddr + sizeof(MachInst) &&
                cacheEntry[0]->range.va + cacheEntry[0]->range.size >= vaddr) {
                req->setPaddr(cacheEntry[0]->pte.translate(vaddr));
                return NoFault;
            }
        } else {
            req->setPaddr(vaddr & PAddrImplMask);
            return NoFault;
        }
    }

    bool hpriv = bits(tlbdata, 0, 0);
    bool red = bits(tlbdata, 1, 1);
    bool priv = bits(tlbdata, 2, 2);
    bool addr_mask = bits(tlbdata, 3, 3);
    bool lsu_im = bits(tlbdata, 4, 4);

    int part_id = bits(tlbdata, 15, 8);
    int tl = bits(tlbdata, 18, 16);
    int pri_context = bits(tlbdata, 47, 32);
    int context;
    ContextType ct;
    int asi;
    bool real = false;

    DPRINTF(TLB, "TLB: priv:%d hpriv:%d red:%d lsuim:%d part_id: %#X\n", priv,
            hpriv, red, lsu_im, part_id);

    if (tl > 0) {
        asi = ASI_N;
        ct = Nucleus;
        context = 0;
    } else {
        asi = ASI_P;
        ct = Primary;
        context = pri_context;
    }

    if (hpriv || red) {
        cacheValid = true;
        cacheState = tlbdata;
        cacheEntry[0] = NULL;
        req->setPaddr(vaddr & PAddrImplMask);
        return NoFault;
    }

    // If the access is unaligned trap
    if (vaddr & 0x3) {
        writeSfsr(false, ct, false, OtherFault, asi);
        return std::make_shared<MemAddressNotAligned>();
    }

    if (addr_mask)
        vaddr = vaddr & VAddrAMask;

    if (!validVirtualAddress(vaddr, addr_mask)) {
        writeSfsr(false, ct, false, VaOutOfRange, asi);
        return std::make_shared<InstructionAccessException>();
    }

    if (!lsu_im) {
        e = lookup(vaddr, part_id, true);
        real = true;
        context = 0;
    } else {
        e = lookup(vaddr, part_id, false, context);
    }

    if (e == NULL || !e->valid) {
        writeTagAccess(vaddr, context);
        if (real) {
            return std::make_shared<InstructionRealTranslationMiss>();
        } else {
            if (FullSystem)
                return std::make_shared<FastInstructionAccessMMUMiss>();
            else
                return std::make_shared<FastInstructionAccessMMUMiss>(
                    req->getVaddr());
        }
    }

    // were not priviledged accesing priv page
    if (!priv && e->pte.priv()) {
        writeTagAccess(vaddr, context);
        writeSfsr(false, ct, false, PrivViolation, asi);
        return std::make_shared<InstructionAccessException>();
    }

    // cache translation date for next translation
    cacheValid = true;
    cacheState = tlbdata;
    cacheEntry[0] = e;

    req->setPaddr(e->pte.translate(vaddr));
    DPRINTF(TLB, "TLB: %#X -> %#X\n", vaddr, req->getPaddr());
    return NoFault;
}

Fault
TLB::translateData(const RequestPtr &req, ThreadContext *tc, bool write)
{
    /*
     * @todo this could really use some profiling and fixing to make
     * it faster!
     */
    uint64_t tlbdata = tc->readMiscRegNoEffect(MISCREG_TLB_DATA);
    Addr vaddr = req->getVaddr();
    Addr size = req->getSize();
    ASI asi;
    asi = (ASI)req->getArchFlags();
    bool implicit = false;
    bool hpriv = bits(tlbdata, 0, 0);
    bool unaligned = vaddr & (size - 1);

    DPRINTF(TLB, "TLB: DTB Request to translate va=%#x size=%d asi=%#x\n",
            vaddr, size, asi);

    if (lookupTable.size() != 64 - freeList.size())
        panic("Lookup table size: %d tlb size: %d\n", lookupTable.size(),
              freeList.size());
    if (asi == ASI_IMPLICIT)
        implicit = true;

    // Only use the fast path here if there doesn't need to be an unaligned
    // trap later
    if (!unaligned) {
        if (hpriv && implicit) {
            req->setPaddr(vaddr & PAddrImplMask);
            return NoFault;
        }

        // Be fast if we can!
        if (cacheValid && cacheState == tlbdata) {
            if (cacheEntry[0]) {
                TlbEntry *ce = cacheEntry[0];
                Addr ce_va = ce->range.va;
                if (cacheAsi[0] == asi && ce_va < vaddr + size &&
                    ce_va + ce->range.size > vaddr &&
                    (!write || ce->pte.writable())) {
                    req->setPaddr(ce->pte.translate(vaddr));
                    if (ce->pte.sideffect() || (ce->pte.paddr() >> 39) & 1) {
                        req->setFlags(Request::UNCACHEABLE |
                                      Request::STRICT_ORDER);
                    }
                    DPRINTF(TLB, "TLB: %#X -> %#X\n", vaddr, req->getPaddr());
                    return NoFault;
                } // if matched
            }     // if cache entry valid
            if (cacheEntry[1]) {
                TlbEntry *ce = cacheEntry[1];
                Addr ce_va = ce->range.va;
                if (cacheAsi[1] == asi && ce_va < vaddr + size &&
                    ce_va + ce->range.size > vaddr &&
                    (!write || ce->pte.writable())) {
                    req->setPaddr(ce->pte.translate(vaddr));
                    if (ce->pte.sideffect() || (ce->pte.paddr() >> 39) & 1) {
                        req->setFlags(Request::UNCACHEABLE |
                                      Request::STRICT_ORDER);
                    }
                    DPRINTF(TLB, "TLB: %#X -> %#X\n", vaddr, req->getPaddr());
                    return NoFault;
                } // if matched
            }     // if cache entry valid
        }
    }

    bool red = bits(tlbdata, 1, 1);
    bool priv = bits(tlbdata, 2, 2);
    bool addr_mask = bits(tlbdata, 3, 3);
    bool lsu_dm = bits(tlbdata, 5, 5);

    int part_id = bits(tlbdata, 15, 8);
    int tl = bits(tlbdata, 18, 16);
    int pri_context = bits(tlbdata, 47, 32);
    int sec_context = bits(tlbdata, 63, 48);

    bool real = false;
    ContextType ct = Primary;
    int context = 0;

    TlbEntry *e;

    DPRINTF(TLB, "TLB: priv:%d hpriv:%d red:%d lsudm:%d part_id: %#X\n", priv,
            hpriv, red, lsu_dm, part_id);

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
        if (!priv && !hpriv && !asiIsUnPriv(asi)) {
            // It appears that context should be Nucleus in these cases?
            writeSfsr(vaddr, write, Nucleus, false, IllegalAsi, asi);
            return std::make_shared<PrivilegedAction>();
        }

        if (!hpriv && asiIsHPriv(asi)) {
            writeSfsr(vaddr, write, Nucleus, false, IllegalAsi, asi);
            return std::make_shared<DataAccessException>();
        }

        if (asiIsPrimary(asi)) {
            context = pri_context;
            ct = Primary;
        } else if (asiIsSecondary(asi)) {
            context = sec_context;
            ct = Secondary;
        } else if (asiIsNucleus(asi)) {
            ct = Nucleus;
            context = 0;
        } else { // ????
            ct = Primary;
            context = pri_context;
        }
    }

    if (!implicit && asi != ASI_P && asi != ASI_S) {
        if (asiIsLittle(asi))
            panic("Little Endian ASIs not supported\n");

        if (asiIsPartialStore(asi))
            panic("Partial Store ASIs not supported\n");

        if (asiIsCmt(asi))
            panic("Cmt ASI registers not implmented\n");

        if (asiIsInterrupt(asi))
            goto handleIntRegAccess;
        if (asiIsMmu(asi))
            goto handleMmuRegAccess;
        if (asiIsScratchPad(asi))
            goto handleScratchRegAccess;
        if (asiIsQueue(asi))
            goto handleQueueRegAccess;
        if (asiIsSparcError(asi))
            goto handleSparcErrorRegAccess;

        if (!asiIsReal(asi) && !asiIsNucleus(asi) && !asiIsAsIfUser(asi) &&
            !asiIsTwin(asi) && !asiIsBlock(asi) && !asiIsNoFault(asi))
            panic("Accessing ASI %#X. Should we?\n", asi);
    }

    // If the asi is unaligned trap
    if (unaligned) {
        writeSfsr(vaddr, false, ct, false, OtherFault, asi);
        return std::make_shared<MemAddressNotAligned>();
    }

    if (addr_mask)
        vaddr = vaddr & VAddrAMask;

    if (!validVirtualAddress(vaddr, addr_mask)) {
        writeSfsr(vaddr, false, ct, true, VaOutOfRange, asi);
        return std::make_shared<DataAccessException>();
    }

    if ((!lsu_dm && !hpriv && !red) || asiIsReal(asi)) {
        real = true;
        context = 0;
    }

    if (hpriv && (implicit || (!asiIsAsIfUser(asi) && !asiIsReal(asi)))) {
        req->setPaddr(vaddr & PAddrImplMask);
        return NoFault;
    }

    e = lookup(vaddr, part_id, real, context);

    if (e == NULL || !e->valid) {
        writeTagAccess(vaddr, context);
        DPRINTF(TLB, "TLB: DTB Failed to find matching TLB entry\n");
        if (real) {
            return std::make_shared<DataRealTranslationMiss>();
        } else {
            if (FullSystem)
                return std::make_shared<FastDataAccessMMUMiss>();
            else
                return std::make_shared<FastDataAccessMMUMiss>(
                    req->getVaddr());
        }
    }

    if (!priv && e->pte.priv()) {
        writeTagAccess(vaddr, context);
        writeSfsr(vaddr, write, ct, e->pte.sideffect(), PrivViolation, asi);
        return std::make_shared<DataAccessException>();
    }

    if (write && !e->pte.writable()) {
        writeTagAccess(vaddr, context);
        writeSfsr(vaddr, write, ct, e->pte.sideffect(), OtherFault, asi);
        return std::make_shared<FastDataAccessProtection>();
    }

    if (e->pte.nofault() && !asiIsNoFault(asi)) {
        writeTagAccess(vaddr, context);
        writeSfsr(vaddr, write, ct, e->pte.sideffect(), LoadFromNfo, asi);
        return std::make_shared<DataAccessException>();
    }

    if (e->pte.sideffect() && asiIsNoFault(asi)) {
        writeTagAccess(vaddr, context);
        writeSfsr(vaddr, write, ct, e->pte.sideffect(), SideEffect, asi);
        return std::make_shared<DataAccessException>();
    }

    if (e->pte.sideffect() || (e->pte.paddr() >> 39) & 1)
        req->setFlags(Request::UNCACHEABLE | Request::STRICT_ORDER);

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
    req->setPaddr(e->pte.translate(vaddr));
    DPRINTF(TLB, "TLB: %#X -> %#X\n", vaddr, req->getPaddr());
    return NoFault;

    /** Normal flow ends here. */
handleIntRegAccess:
    if (!hpriv) {
        writeSfsr(vaddr, write, Primary, true, IllegalAsi, asi);
        if (priv)
            return std::make_shared<DataAccessException>();
        else
            return std::make_shared<PrivilegedAction>();
    }

    if ((asi == ASI_SWVR_UDB_INTR_W && !write) ||
        (asi == ASI_SWVR_UDB_INTR_R && write)) {
        writeSfsr(vaddr, write, Primary, true, IllegalAsi, asi);
        return std::make_shared<DataAccessException>();
    }

    goto regAccessOk;

handleScratchRegAccess:
    if (vaddr > 0x38 || (vaddr >= 0x20 && vaddr < 0x30 && !hpriv)) {
        writeSfsr(vaddr, write, Primary, true, IllegalAsi, asi);
        return std::make_shared<DataAccessException>();
    }
    goto regAccessOk;

handleQueueRegAccess:
    if (!priv && !hpriv) {
        writeSfsr(vaddr, write, Primary, true, IllegalAsi, asi);
        return std::make_shared<PrivilegedAction>();
    }
    if ((!hpriv && vaddr & 0xF) || vaddr > 0x3f8 || vaddr < 0x3c0) {
        writeSfsr(vaddr, write, Primary, true, IllegalAsi, asi);
        return std::make_shared<DataAccessException>();
    }
    goto regAccessOk;

handleSparcErrorRegAccess:
    if (!hpriv) {
        writeSfsr(vaddr, write, Primary, true, IllegalAsi, asi);
        if (priv)
            return std::make_shared<DataAccessException>();
        else
            return std::make_shared<PrivilegedAction>();
    }
    goto regAccessOk;

regAccessOk:
handleMmuRegAccess:
    DPRINTF(TLB, "TLB: DTB Translating local access\n");
    req->setLocalAccessor(
        [this, write](ThreadContext *tc, PacketPtr pkt) -> Cycles {
            return write ? doMmuRegWrite(tc, pkt) : doMmuRegRead(tc, pkt);
        });
    req->setPaddr(req->getVaddr());
    return NoFault;
};

Fault
TLB::translateAtomic(const RequestPtr &req, ThreadContext *tc,
                     BaseMMU::Mode mode)
{
    if (mode == BaseMMU::Execute)
        return translateInst(req, tc);
    else
        return translateData(req, tc, mode == BaseMMU::Write);
}

Fault
TLB::translateFunctional(const RequestPtr &req, ThreadContext *tc,
                         BaseMMU::Mode mode)
{
    Addr vaddr = req->getVaddr();

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

    bool hpriv = bits(tlbdata, 0, 0);
    // bool priv = bits(tlbdata,2,2);
    bool addr_mask = bits(tlbdata, 3, 3);
    bool data_real = !bits(tlbdata, 5, 5);
    bool inst_real = !bits(tlbdata, 4, 4);
    bool ctx_zero = bits(tlbdata, 18, 16) > 0;
    int part_id = bits(tlbdata, 15, 8);
    int pri_context = bits(tlbdata, 47, 32);
    // int sec_context = bits(tlbdata,63,48);

    bool real = (mode == BaseMMU::Execute) ? inst_real : data_real;

    TlbEntry *tbe;
    PageTableEntry pte;
    Addr tsbs[4];
    Addr va_tag;
    TteTag ttetag;

    if (hpriv) {
        req->setPaddr(vaddr);
        return NoFault;
    }

    if (addr_mask)
        vaddr = vaddr & VAddrAMask;

    if (!validVirtualAddress(vaddr, addr_mask)) {
        if (mode == BaseMMU::Execute)
            return std::make_shared<InstructionAccessException>();
        else
            return std::make_shared<DataAccessException>();
    }

    tbe = lookup(vaddr, part_id, real, ctx_zero ? 0 : pri_context, false);
    if (tbe) {
        pte = tbe->pte;
        DPRINTF(TLB, "Virtual(%#x)->Physical(%#x) found in TLB\n", vaddr,
                pte.translate(vaddr));
        req->setPaddr(pte.translate(vaddr));
        return NoFault;
    }

    if (!FullSystem)
        return tc->getProcessPtr()->pTable->translate(req);

    PortProxy mem(tc, tc->getSystemPtr()->cacheLineSize());
    // We didn't find it in the tlbs, so lets look at the TSBs
    GetTsbPtr(tc, vaddr, ctx_zero ? 0 : pri_context, tsbs);
    va_tag = bits(vaddr, 63, 22);
    for (int x = 0; x < 4; x++) {
        ttetag = betoh(mem.read<uint64_t>(tsbs[x]));
        if (ttetag.valid() && ttetag.va() == va_tag) {
            uint64_t entry = mem.read<uint64_t>(tsbs[x]) + sizeof(uint64_t);
            // I think it's sun4v at least!
            pte.populate(betoh(entry), PageTableEntry::sun4v);
            DPRINTF(TLB, "Virtual(%#x)->Physical(%#x) found in TTE\n", vaddr,
                    pte.translate(vaddr));
            req->setPaddr(pte.translate(vaddr));
            return NoFault;
        }
    }

    if (mode == BaseMMU::Execute) {
        if (real)
            return std::make_shared<InstructionRealTranslationMiss>();
        else if (FullSystem)
            return std::make_shared<FastInstructionAccessMMUMiss>();
        else
            return std::make_shared<FastInstructionAccessMMUMiss>(vaddr);
    } else {
        if (real)
            return std::make_shared<DataRealTranslationMiss>();
        else if (FullSystem)
            return std::make_shared<FastDataAccessMMUMiss>();
        else
            return std::make_shared<FastDataAccessMMUMiss>(vaddr);
    }
}

void
TLB::translateTiming(const RequestPtr &req, ThreadContext *tc,
                     BaseMMU::Translation *translation, BaseMMU::Mode mode)
{
    assert(translation);
    translation->finish(translateAtomic(req, tc, mode), req, tc, mode);
}

Fault
TLB::finalizePhysical(const RequestPtr &req, ThreadContext *tc,
                      BaseMMU::Mode mode) const
{
    return NoFault;
}

Cycles
TLB::doMmuRegRead(ThreadContext *tc, Packet *pkt)
{
    Addr va = pkt->getAddr();
    ASI asi = (ASI)pkt->req->getArchFlags();
    uint64_t temp;

    DPRINTF(IPR, "Memory Mapped IPR Read: asi=%#X a=%#x\n",
            (uint32_t)pkt->req->getArchFlags(), pkt->getAddr());

    TLB *itb = static_cast<TLB *>(tc->getMMUPtr()->itb);

    switch (asi) {
    case ASI_LSU_CONTROL_REG:
        assert(va == 0);
        pkt->setBE(tc->readMiscReg(MISCREG_MMU_LSU_CTRL));
        break;
    case ASI_MMU:
        switch (va) {
        case 0x8:
            pkt->setBE(tc->readMiscReg(MISCREG_MMU_P_CONTEXT));
            break;
        case 0x10:
            pkt->setBE(tc->readMiscReg(MISCREG_MMU_S_CONTEXT));
            break;
        default:
            goto doMmuReadError;
        }
        break;
    case ASI_QUEUE:
        pkt->setBE(
            tc->readMiscReg(MISCREG_QUEUE_CPU_MONDO_HEAD + (va >> 4) - 0x3c));
        break;
    case ASI_DMMU_CTXT_ZERO_TSB_BASE_PS0:
        assert(va == 0);
        pkt->setBE(c0_tsb_ps0);
        break;
    case ASI_DMMU_CTXT_ZERO_TSB_BASE_PS1:
        assert(va == 0);
        pkt->setBE(c0_tsb_ps1);
        break;
    case ASI_DMMU_CTXT_ZERO_CONFIG:
        assert(va == 0);
        pkt->setBE(c0_config);
        break;
    case ASI_IMMU_CTXT_ZERO_TSB_BASE_PS0:
        assert(va == 0);
        pkt->setBE(itb->c0_tsb_ps0);
        break;
    case ASI_IMMU_CTXT_ZERO_TSB_BASE_PS1:
        assert(va == 0);
        pkt->setBE(itb->c0_tsb_ps1);
        break;
    case ASI_IMMU_CTXT_ZERO_CONFIG:
        assert(va == 0);
        pkt->setBE(itb->c0_config);
        break;
    case ASI_DMMU_CTXT_NONZERO_TSB_BASE_PS0:
        assert(va == 0);
        pkt->setBE(cx_tsb_ps0);
        break;
    case ASI_DMMU_CTXT_NONZERO_TSB_BASE_PS1:
        assert(va == 0);
        pkt->setBE(cx_tsb_ps1);
        break;
    case ASI_DMMU_CTXT_NONZERO_CONFIG:
        assert(va == 0);
        pkt->setBE(cx_config);
        break;
    case ASI_IMMU_CTXT_NONZERO_TSB_BASE_PS0:
        assert(va == 0);
        pkt->setBE(itb->cx_tsb_ps0);
        break;
    case ASI_IMMU_CTXT_NONZERO_TSB_BASE_PS1:
        assert(va == 0);
        pkt->setBE(itb->cx_tsb_ps1);
        break;
    case ASI_IMMU_CTXT_NONZERO_CONFIG:
        assert(va == 0);
        pkt->setBE(itb->cx_config);
        break;
    case ASI_SPARC_ERROR_STATUS_REG:
        pkt->setBE((uint64_t)0);
        break;
    case ASI_HYP_SCRATCHPAD:
    case ASI_SCRATCHPAD:
        pkt->setBE(tc->readMiscReg(MISCREG_SCRATCHPAD_R0 + (va >> 3)));
        break;
    case ASI_IMMU:
        switch (va) {
        case 0x0:
            temp = itb->tag_access;
            pkt->setBE(bits(temp, 63, 22) | bits(temp, 12, 0) << 48);
            break;
        case 0x18:
            pkt->setBE(itb->sfsr);
            break;
        case 0x30:
            pkt->setBE(itb->tag_access);
            break;
        default:
            goto doMmuReadError;
        }
        break;
    case ASI_DMMU:
        switch (va) {
        case 0x0:
            temp = tag_access;
            pkt->setBE(bits(temp, 63, 22) | bits(temp, 12, 0) << 48);
            break;
        case 0x18:
            pkt->setBE(sfsr);
            break;
        case 0x20:
            pkt->setBE(sfar);
            break;
        case 0x30:
            pkt->setBE(tag_access);
            break;
        case 0x80:
            pkt->setBE(tc->readMiscReg(MISCREG_MMU_PART_ID));
            break;
        default:
            goto doMmuReadError;
        }
        break;
    case ASI_DMMU_TSB_PS0_PTR_REG:
        pkt->setBE(MakeTsbPtr(Ps0, tag_access, c0_tsb_ps0, c0_config,
                              cx_tsb_ps0, cx_config));
        break;
    case ASI_DMMU_TSB_PS1_PTR_REG:
        pkt->setBE(MakeTsbPtr(Ps1, tag_access, c0_tsb_ps1, c0_config,
                              cx_tsb_ps1, cx_config));
        break;
    case ASI_IMMU_TSB_PS0_PTR_REG:
        pkt->setBE(MakeTsbPtr(Ps0, itb->tag_access, itb->c0_tsb_ps0,
                              itb->c0_config, itb->cx_tsb_ps0,
                              itb->cx_config));
        break;
    case ASI_IMMU_TSB_PS1_PTR_REG:
        pkt->setBE(MakeTsbPtr(Ps1, itb->tag_access, itb->c0_tsb_ps1,
                              itb->c0_config, itb->cx_tsb_ps1,
                              itb->cx_config));
        break;
    case ASI_SWVR_INTR_RECEIVE: {
        SparcISA::Interrupts *interrupts =
            dynamic_cast<SparcISA::Interrupts *>(
                tc->getCpuPtr()->getInterruptController(0));
        pkt->setBE(interrupts->get_vec(IT_INT_VEC));
    } break;
    case ASI_SWVR_UDB_INTR_R: {
        SparcISA::Interrupts *interrupts =
            dynamic_cast<SparcISA::Interrupts *>(
                tc->getCpuPtr()->getInterruptController(0));
        temp = findMsbSet(interrupts->get_vec(IT_INT_VEC));
        tc->getCpuPtr()->clearInterrupt(0, IT_INT_VEC, temp);
        pkt->setBE(temp);
    } break;
    default:
    doMmuReadError:
        panic("need to impl DTB::doMmuRegRead() got asi=%#x, va=%#x\n",
              (uint32_t)asi, va);
    }
    pkt->makeAtomicResponse();
    return Cycles(1);
}

Cycles
TLB::doMmuRegWrite(ThreadContext *tc, Packet *pkt)
{
    uint64_t data = pkt->getBE<uint64_t>();
    Addr va = pkt->getAddr();
    ASI asi = (ASI)pkt->req->getArchFlags();

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

    TLB *itb = static_cast<TLB *>(tc->getMMUPtr()->itb);

    switch (asi) {
    case ASI_LSU_CONTROL_REG:
        assert(va == 0);
        tc->setMiscReg(MISCREG_MMU_LSU_CTRL, data);
        break;
    case ASI_MMU:
        switch (va) {
        case 0x8:
            tc->setMiscReg(MISCREG_MMU_P_CONTEXT, data);
            break;
        case 0x10:
            tc->setMiscReg(MISCREG_MMU_S_CONTEXT, data);
            break;
        default:
            goto doMmuWriteError;
        }
        break;
    case ASI_QUEUE:
        assert(mbits(data, 13, 6) == data);
        tc->setMiscReg(MISCREG_QUEUE_CPU_MONDO_HEAD + (va >> 4) - 0x3c, data);
        break;
    case ASI_DMMU_CTXT_ZERO_TSB_BASE_PS0:
        assert(va == 0);
        c0_tsb_ps0 = data;
        break;
    case ASI_DMMU_CTXT_ZERO_TSB_BASE_PS1:
        assert(va == 0);
        c0_tsb_ps1 = data;
        break;
    case ASI_DMMU_CTXT_ZERO_CONFIG:
        assert(va == 0);
        c0_config = data;
        break;
    case ASI_IMMU_CTXT_ZERO_TSB_BASE_PS0:
        assert(va == 0);
        itb->c0_tsb_ps0 = data;
        break;
    case ASI_IMMU_CTXT_ZERO_TSB_BASE_PS1:
        assert(va == 0);
        itb->c0_tsb_ps1 = data;
        break;
    case ASI_IMMU_CTXT_ZERO_CONFIG:
        assert(va == 0);
        itb->c0_config = data;
        break;
    case ASI_DMMU_CTXT_NONZERO_TSB_BASE_PS0:
        assert(va == 0);
        cx_tsb_ps0 = data;
        break;
    case ASI_DMMU_CTXT_NONZERO_TSB_BASE_PS1:
        assert(va == 0);
        cx_tsb_ps1 = data;
        break;
    case ASI_DMMU_CTXT_NONZERO_CONFIG:
        assert(va == 0);
        cx_config = data;
        break;
    case ASI_IMMU_CTXT_NONZERO_TSB_BASE_PS0:
        assert(va == 0);
        itb->cx_tsb_ps0 = data;
        break;
    case ASI_IMMU_CTXT_NONZERO_TSB_BASE_PS1:
        assert(va == 0);
        itb->cx_tsb_ps1 = data;
        break;
    case ASI_IMMU_CTXT_NONZERO_CONFIG:
        assert(va == 0);
        itb->cx_config = data;
        break;
    case ASI_SPARC_ERROR_EN_REG:
    case ASI_SPARC_ERROR_STATUS_REG:
        inform("Ignoring write to SPARC ERROR regsiter\n");
        break;
    case ASI_HYP_SCRATCHPAD:
    case ASI_SCRATCHPAD:
        tc->setMiscReg(MISCREG_SCRATCHPAD_R0 + (va >> 3), data);
        break;
    case ASI_IMMU:
        switch (va) {
        case 0x18:
            itb->sfsr = data;
            break;
        case 0x30:
            itb->tag_access = szext<60>(data);
            break;
        default:
            goto doMmuWriteError;
        }
        break;
    case ASI_ITLB_DATA_ACCESS_REG:
        entry_insert = bits(va, 8, 3);
        [[fallthrough]];
    case ASI_ITLB_DATA_IN_REG:
        assert(entry_insert != -1 || mbits(va, 10, 9) == va);
        ta_insert = itb->tag_access;
        va_insert = mbits(ta_insert, 63, 13);
        ct_insert = mbits(ta_insert, 12, 0);
        part_insert = tc->readMiscReg(MISCREG_MMU_PART_ID);
        real_insert = bits(va, 9, 9);
        pte.populate(data, bits(va, 10, 10) ? PageTableEntry::sun4v :
                                              PageTableEntry::sun4u);
        itb->insert(va_insert, part_insert, ct_insert, real_insert, pte,
                    entry_insert);
        break;
    case ASI_DTLB_DATA_ACCESS_REG:
        entry_insert = bits(va, 8, 3);
        [[fallthrough]];
    case ASI_DTLB_DATA_IN_REG:
        assert(entry_insert != -1 || mbits(va, 10, 9) == va);
        ta_insert = tag_access;
        va_insert = mbits(ta_insert, 63, 13);
        ct_insert = mbits(ta_insert, 12, 0);
        part_insert = tc->readMiscReg(MISCREG_MMU_PART_ID);
        real_insert = bits(va, 9, 9);
        pte.populate(data, bits(va, 10, 10) ? PageTableEntry::sun4v :
                                              PageTableEntry::sun4u);
        insert(va_insert, part_insert, ct_insert, real_insert, pte,
               entry_insert);
        break;
    case ASI_IMMU_DEMAP:
        ignore = false;
        ctx_id = -1;
        part_id = tc->readMiscReg(MISCREG_MMU_PART_ID);
        switch (bits(va, 5, 4)) {
        case 0:
            ctx_id = tc->readMiscReg(MISCREG_MMU_P_CONTEXT);
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

        switch (bits(va, 7, 6)) {
        case 0: // demap page
            if (!ignore)
                itb->demapPage(mbits(va, 63, 13), part_id, bits(va, 9, 9),
                               ctx_id);
            break;
        case 1: // demap context
            if (!ignore)
                itb->demapContext(part_id, ctx_id);
            break;
        case 2:
            itb->demapAll(part_id);
            break;
        default:
            panic("Invalid type for IMMU demap\n");
        }
        break;
    case ASI_DMMU:
        switch (va) {
        case 0x18:
            sfsr = data;
            break;
        case 0x30:
            tag_access = szext<60>(data);
            break;
        case 0x80:
            tc->setMiscReg(MISCREG_MMU_PART_ID, data);
            break;
        default:
            goto doMmuWriteError;
        }
        break;
    case ASI_DMMU_DEMAP:
        ignore = false;
        ctx_id = -1;
        part_id = tc->readMiscReg(MISCREG_MMU_PART_ID);
        switch (bits(va, 5, 4)) {
        case 0:
            ctx_id = tc->readMiscReg(MISCREG_MMU_P_CONTEXT);
            break;
        case 1:
            ctx_id = tc->readMiscReg(MISCREG_MMU_S_CONTEXT);
            break;
        case 3:
            ctx_id = 0;
            break;
        default:
            ignore = true;
        }

        switch (bits(va, 7, 6)) {
        case 0: // demap page
            if (!ignore)
                demapPage(mbits(va, 63, 13), part_id, bits(va, 9, 9), ctx_id);
            break;
        case 1: // demap context
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
    case ASI_SWVR_INTR_RECEIVE: {
        int msb;
        // clear all the interrupts that aren't set in the write
        SparcISA::Interrupts *interrupts =
            dynamic_cast<SparcISA::Interrupts *>(
                tc->getCpuPtr()->getInterruptController(0));
        while (interrupts->get_vec(IT_INT_VEC) & data) {
            msb = findMsbSet(interrupts->get_vec(IT_INT_VEC) & data);
            tc->getCpuPtr()->clearInterrupt(0, IT_INT_VEC, msb);
        }
    } break;
    case ASI_SWVR_UDB_INTR_W:
        tc->getSystemPtr()
            ->threads[bits(data, 12, 8)]
            ->getCpuPtr()
            ->postInterrupt(0, bits(data, 5, 0), 0);
        break;
    default:
    doMmuWriteError:
        panic("need to impl DTB::doMmuRegWrite() got asi=%#x, va=%#x d=%#x\n",
              (uint32_t)pkt->req->getArchFlags(), pkt->getAddr(), data);
    }
    pkt->makeAtomicResponse();
    return Cycles(1);
}

void
TLB::GetTsbPtr(ThreadContext *tc, Addr addr, int ctx, Addr *ptrs)
{
    uint64_t tag_access = mbits(addr, 63, 13) | mbits(ctx, 12, 0);
    TLB *itb = static_cast<TLB *>(tc->getMMUPtr()->itb);
    ptrs[0] = MakeTsbPtr(Ps0, tag_access, c0_tsb_ps0, c0_config, cx_tsb_ps0,
                         cx_config);
    ptrs[1] = MakeTsbPtr(Ps1, tag_access, c0_tsb_ps1, c0_config, cx_tsb_ps1,
                         cx_config);
    ptrs[2] = MakeTsbPtr(Ps0, tag_access, itb->c0_tsb_ps0, itb->c0_config,
                         itb->cx_tsb_ps0, itb->cx_config);
    ptrs[3] = MakeTsbPtr(Ps1, tag_access, itb->c0_tsb_ps1, itb->c0_config,
                         itb->cx_tsb_ps1, itb->cx_config);
}

uint64_t
TLB::MakeTsbPtr(TsbPageSize ps, uint64_t tag_access, uint64_t c0_tsb,
                uint64_t c0_config, uint64_t cX_tsb, uint64_t cX_config)
{
    uint64_t tsb;
    uint64_t config;

    if (bits(tag_access, 12, 0) == 0) {
        tsb = c0_tsb;
        config = c0_config;
    } else {
        tsb = cX_tsb;
        config = cX_config;
    }

    uint64_t ptr = mbits(tsb, 63, 13);
    bool split = bits(tsb, 12, 12);
    int tsb_size = bits(tsb, 3, 0);
    int page_size = (ps == Ps0) ? bits(config, 2, 0) : bits(config, 10, 8);

    if (ps == Ps1 && split)
        ptr |= 1ULL << (13 + tsb_size);
    ptr |= (tag_access >> (9 + page_size * 3)) & mask(12 + tsb_size, 4);

    return ptr;
}

void
TLB::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(size);
    SERIALIZE_SCALAR(usedEntries);
    SERIALIZE_SCALAR(lastReplaced);

    // convert the pointer based free list into an index based one
    std::vector<int> free_list;
    for (const TlbEntry *entry : freeList)
        free_list.push_back(entry - tlb);

    SERIALIZE_CONTAINER(free_list);

    SERIALIZE_SCALAR(c0_tsb_ps0);
    SERIALIZE_SCALAR(c0_tsb_ps1);
    SERIALIZE_SCALAR(c0_config);
    SERIALIZE_SCALAR(cx_tsb_ps0);
    SERIALIZE_SCALAR(cx_tsb_ps1);
    SERIALIZE_SCALAR(cx_config);
    SERIALIZE_SCALAR(sfsr);
    SERIALIZE_SCALAR(tag_access);
    SERIALIZE_SCALAR(sfar);

    for (int x = 0; x < size; x++) {
        ScopedCheckpointSection sec(cp, csprintf("PTE%d", x));
        tlb[x].serialize(cp);
    }
}

void
TLB::unserialize(CheckpointIn &cp)
{
    int oldSize;

    paramIn(cp, "size", oldSize);
    if (oldSize != size)
        panic("Don't support unserializing different sized TLBs\n");
    UNSERIALIZE_SCALAR(usedEntries);
    UNSERIALIZE_SCALAR(lastReplaced);

    std::vector<int> free_list;
    UNSERIALIZE_CONTAINER(free_list);
    freeList.clear();
    for (int idx : free_list)
        freeList.push_back(&tlb[idx]);

    UNSERIALIZE_SCALAR(c0_tsb_ps0);
    UNSERIALIZE_SCALAR(c0_tsb_ps1);
    UNSERIALIZE_SCALAR(c0_config);
    UNSERIALIZE_SCALAR(cx_tsb_ps0);
    UNSERIALIZE_SCALAR(cx_tsb_ps1);
    UNSERIALIZE_SCALAR(cx_config);
    UNSERIALIZE_SCALAR(sfsr);
    UNSERIALIZE_SCALAR(tag_access);

    lookupTable.clear();
    for (int x = 0; x < size; x++) {
        ScopedCheckpointSection sec(cp, csprintf("PTE%d", x));
        tlb[x].unserialize(cp);
        if (tlb[x].valid)
            lookupTable.insert(tlb[x].range, &tlb[x]);
    }
    UNSERIALIZE_SCALAR(sfar);
}

} // namespace SparcISA
} // namespace gem5
