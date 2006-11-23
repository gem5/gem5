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
#include "arch/sparc/tlb.hh"
#include "sim/builder.hh"
#include "arch/sparc/miscregfile.hh"
#include "cpu/thread_context.hh"

/* @todo remove some of the magic constants.  -- ali
 * */
namespace SparcISA
{

TLB::TLB(const std::string &name, int s)
    : SimObject(name), size(s)
{
    // To make this work you'll have to change the hypervisor and OS
    if (size > 64)
        fatal("SPARC T1 TLB registers don't support more than 64 TLB entries.");

    tlb = new TlbEntry[size];
    memset(tlb, 0, sizeof(TlbEntry) * size);
}

void
TLB::clearUsedBits()
{
    MapIter i;
    for (i = lookupTable.begin(); i != lookupTable.end();) {
        TlbEntry *t = i->second;
        if (!t->pte.locked()) {
            t->used = false;
            usedEntries--;
        }
    }
}


void
TLB::insert(Addr va, int partition_id, int context_id, bool real,
        const PageTableEntry& PTE)
{


    MapIter i;
    TlbEntry *new_entry;
    int x = -1;
    for (x = 0; x < size; x++) {
        if (!tlb[x].valid || !tlb[x].used)  {
            new_entry = &tlb[x];
            break;
        }
    }

    // Update the last ently if their all locked
    if (x == -1)
       x = size - 1;

    assert(PTE.valid());
    new_entry->range.va = va;
    new_entry->range.size = PTE.size();
    new_entry->range.partitionId = partition_id;
    new_entry->range.contextId = context_id;
    new_entry->range.real = real;
    new_entry->pte = PTE;
    new_entry->used = true;;
    new_entry->valid = true;
    usedEntries++;


    // Demap any entry that conflicts
    i = lookupTable.find(new_entry->range);
    if (i != lookupTable.end()) {
        i->second->valid = false;
        if (i->second->used) {
            i->second->used = false;
            usedEntries--;
        }
        lookupTable.erase(i);
    }

    lookupTable.insert(new_entry->range, new_entry);;

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

    // Assemble full address structure
    tr.va = va;
    tr.size = va + MachineBytes;
    tr.contextId = context_id;
    tr.partitionId = partition_id;
    tr.real = real;

    // Try to find the entry
    i = lookupTable.find(tr);
    if (i == lookupTable.end()) {
        return NULL;
    }

    // Mark the entries used bit and clear other used bits in needed
    t = i->second;
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
TLB::demapPage(Addr va, int partition_id, bool real, int context_id)
{
    TlbRange tr;
    MapIter i;

    // Assemble full address structure
    tr.va = va;
    tr.size = va + MachineBytes;
    tr.contextId = context_id;
    tr.partitionId = partition_id;
    tr.real = real;

    // Demap any entry that conflicts
    i = lookupTable.find(tr);
    if (i != lookupTable.end()) {
        i->second->valid = false;
        if (i->second->used) {
            i->second->used = false;
            usedEntries--;
        }
        lookupTable.erase(i);
    }
}

void
TLB::demapContext(int partition_id, int context_id)
{
    int x;
    for (x = 0; x < size; x++) {
        if (tlb[x].range.contextId == context_id &&
            tlb[x].range.partitionId == partition_id) {
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
    for (x = 0; x < size; x++) {
        if (!tlb[x].pte.locked() && tlb[x].range.partitionId == partition_id) {
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
    for (x = 0; x < size; x++) {
        tlb[x].valid = false;
    }
    usedEntries = 0;
}

uint64_t
TLB::TteRead(int entry) {
    assert(entry < size);
    return tlb[entry].pte();
}

uint64_t
TLB::TagRead(int entry) {
    assert(entry < size);
    uint64_t tag;

    tag = tlb[entry].range.contextId | tlb[entry].range.va |
          (uint64_t)tlb[entry].range.partitionId << 61;
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
    tc->setMiscReg(reg, sfsr);
}


void
ITB::writeSfsr(ThreadContext *tc, bool write, ContextType ct,
        bool se, FaultTypes ft, int asi)
{
    TLB::writeSfsr(tc, MISCREG_MMU_ITLB_SFSR, write, ct, se, ft, asi);
}

void
DTB::writeSfr(ThreadContext *tc, Addr a, bool write, ContextType ct,
        bool se, FaultTypes ft, int asi)
{
    TLB::writeSfsr(tc, MISCREG_MMU_DTLB_SFSR, write, ct, se, ft, asi);
    tc->setMiscReg(MISCREG_MMU_DTLB_SFAR, a);
}


Fault
ITB::translate(RequestPtr &req, ThreadContext *tc)
{
    uint64_t hpstate = tc->readMiscReg(MISCREG_HPSTATE);
    uint64_t pstate = tc->readMiscReg(MISCREG_PSTATE);
    bool lsuIm = tc->readMiscReg(MISCREG_MMU_LSU_CTRL) >> 2 & 0x1;
    uint64_t tl = tc->readMiscReg(MISCREG_TL);
    uint64_t part_id = tc->readMiscReg(MISCREG_MMU_PART_ID);
    bool addr_mask = pstate >> 3 & 0x1;
    bool priv = pstate >> 2 & 0x1;
    Addr vaddr = req->getVaddr();
    int context;
    ContextType ct;
    int asi;
    bool real = false;
    TlbEntry *e;

    assert(req->getAsi() == ASI_IMPLICIT);

    if (tl > 0) {
        asi = ASI_N;
        ct = Nucleus;
        context = 0;
    } else {
        asi = ASI_P;
        ct = Primary;
        context = tc->readMiscReg(MISCREG_MMU_P_CONTEXT);
    }

    if ( hpstate >> 2 & 0x1 || hpstate >> 5 & 0x1 ) {
        req->setPaddr(req->getVaddr() & PAddrImplMask);
        return NoFault;
    }

    // If the asi is unaligned trap
    if (vaddr & 0x7) {
        writeSfsr(tc, false, ct, false, OtherFault, asi);
        return new MemAddressNotAligned;
    }

    if (addr_mask)
        vaddr = vaddr & VAddrAMask;

    if (!validVirtualAddress(vaddr, addr_mask)) {
        writeSfsr(tc, false, ct, false, VaOutOfRange, asi);
        return new InstructionAccessException;
    }

    if (lsuIm) {
        e = lookup(req->getVaddr(), part_id, true);
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

    req->setPaddr(e->pte.paddr() & ~e->pte.size() |
                  req->getVaddr() & e->pte.size());
    return NoFault;
}



Fault
DTB::translate(RequestPtr &req, ThreadContext *tc, bool write)
{
    /* @todo this could really use some profiling and fixing to make it faster! */
    uint64_t hpstate = tc->readMiscReg(MISCREG_HPSTATE);
    uint64_t pstate = tc->readMiscReg(MISCREG_PSTATE);
    bool lsuDm = tc->readMiscReg(MISCREG_MMU_LSU_CTRL) >> 3 & 0x1;
    uint64_t tl = tc->readMiscReg(MISCREG_TL);
    uint64_t part_id = tc->readMiscReg(MISCREG_MMU_PART_ID);
    bool hpriv = hpstate >> 2 & 0x1;
    bool red = hpstate >> 5 >> 0x1;
    bool addr_mask = pstate >> 3 & 0x1;
    bool priv = pstate >> 2 & 0x1;
    bool implicit = false;
    bool real = false;
    Addr vaddr = req->getVaddr();
    ContextType ct;
    int context;
    ASI asi;

    TlbEntry *e;


    asi = (ASI)req->getAsi();
    if (asi == ASI_IMPLICIT)
        implicit = true;

    if (implicit) {
        if (tl > 0) {
            asi = ASI_N;
            ct = Nucleus;
            context = 0;
        } else {
            asi = ASI_P;
            ct = Primary;
            context = tc->readMiscReg(MISCREG_MMU_P_CONTEXT);
        }
    } else if (!hpriv && !red) {
        if (tl > 0) {
            ct = Nucleus;
            context = 0;
        } else if (AsiIsSecondary(asi)) {
            ct = Secondary;
            context = tc->readMiscReg(MISCREG_MMU_S_CONTEXT);
        } else {
            context = tc->readMiscReg(MISCREG_MMU_P_CONTEXT);
            ct = Primary; //???
        }

        // We need to check for priv level/asi priv
        if (!priv && !AsiIsUnPriv(asi)) {
            // It appears that context should be Nucleus in these cases?
            writeSfr(tc, vaddr, write, Nucleus, false, IllegalAsi, asi);
            return new PrivilegedAction;
        }
        if (priv && AsiIsHPriv(asi)) {
            writeSfr(tc, vaddr, write, Nucleus, false, IllegalAsi, asi);
            return new DataAccessException;
        }

    }

    // If the asi is unaligned trap
    if (AsiIsBlock(asi) && vaddr & 0x3f || vaddr & 0x7) {
        writeSfr(tc, vaddr, false, ct, false, OtherFault, asi);
        return new MemAddressNotAligned;
    }

    if (addr_mask)
        vaddr = vaddr & VAddrAMask;

    if (!validVirtualAddress(vaddr, addr_mask)) {
        writeSfr(tc, vaddr, false, ct, true, VaOutOfRange, asi);
        return new DataAccessException;
    }

    if (!implicit) {
        if (AsiIsLittle(asi))
            panic("Little Endian ASIs not supported\n");
        if (AsiIsBlock(asi))
            panic("Block ASIs not supported\n");
        if (AsiIsNoFault(asi))
            panic("No Fault ASIs not supported\n");
        if (AsiIsTwin(asi))
            panic("Twin ASIs not supported\n");
        if (AsiIsPartialStore(asi))
            panic("Partial Store ASIs not supported\n");
        if (AsiIsMmu(asi))
            goto handleMmuRegAccess;

        if (AsiIsScratchPad(asi))
            goto handleScratchRegAccess;
    }

    if ((!lsuDm && !hpriv) || AsiIsReal(asi)) {
        real = true;
        context = 0;
    };

    if (hpriv && (implicit || (!AsiIsAsIfUser(asi) && !AsiIsReal(asi)))) {
        req->setPaddr(req->getVaddr() & PAddrImplMask);
        return NoFault;
    }

    e = lookup(req->getVaddr(), part_id, real, context);

    if (e == NULL || !e->valid) {
        tc->setMiscReg(MISCREG_MMU_DTLB_TAG_ACCESS,
                vaddr & ~BytesInPageMask | context);
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

    req->setPaddr(e->pte.paddr() & ~e->pte.size() |
                  req->getVaddr() & e->pte.size());
    return NoFault;
    /*** End of normal Path ***/

handleMmuRegAccess:
handleScratchRegAccess:
    panic("How are we ever going to deal with this?\n");
};

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
