/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
 * Copyright (c) 2007 MIPS Technologies, Inc.
 * Copyright (c) 2020 Barkhausen Institut
 * Copyright (c) 2021 Huawei International
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

#include "arch/riscv/tlb.hh"

#include <string>
#include <vector>

#include "arch/riscv/faults.hh"
#include "arch/riscv/mmu.hh"
#include "arch/riscv/pagetable.hh"
#include "arch/riscv/pagetable_walker.hh"
#include "arch/riscv/pma_checker.hh"
#include "arch/riscv/pmp.hh"
#include "arch/riscv/pra_constants.hh"
#include "arch/riscv/utility.hh"
#include "base/inifile.hh"
#include "base/str.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "debug/TLB.hh"
#include "debug/TLBVerbose.hh"
#include "mem/page_table.hh"
#include "params/RiscvTLB.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"
#include "sim/system.hh"

namespace gem5
{

using namespace RiscvISA;

///////////////////////////////////////////////////////////////////////
//
//  RISC-V TLB
//

static Addr
buildKey(Addr vpn, uint16_t asid)
{
    // Note ASID is 16 bits
    // The VPN in sv39 is up to 39-12=27 bits
    // The VPN in sv48 is up to 48-12=36 bits
    // The VPN in sv57 is up to 57-12=45 bits
    // So, shifting the ASID into the top 16 bits is safe.
    assert(bits(vpn, 63, 48) == 0);
    return (static_cast<Addr>(asid) << 48) | vpn;
}

TLB::TLB(const Params &p) :
    BaseTLB(p), size(p.size), tlb(size),
    lruSeq(0), stats(this), pma(p.pma_checker),
    pmp(p.pmp)
{
    for (size_t x = 0; x < size; x++) {
        tlb[x].trieHandle = NULL;
        freeList.push_back(&tlb[x]);
    }

    walker = p.walker;
    walker->setTLB(this);
}

Walker *
TLB::getWalker()
{
    return walker;
}

void
TLB::evictLRU()
{
    // Find the entry with the lowest (and hence least recently updated)
    // sequence number.

    size_t lru = 0;
    for (size_t i = 1; i < size; i++) {
        if (tlb[i].lruSeq < tlb[lru].lruSeq)
            lru = i;
    }

    remove(lru);
}

TlbEntry *
TLB::lookup(Addr vpn, uint16_t asid, BaseMMU::Mode mode, bool hidden)
{
    TlbEntry *entry = trie.lookup(buildKey(vpn, asid));

    DPRINTF(TLBVerbose, "lookup(vpn=%#x, asid=%#x, key=%#x): "
                        "%s ppn=%#x (%#x) %s\n",
            vpn, asid, buildKey(vpn, asid), entry ? "hit" : "miss",
            entry ? entry->paddr : 0, entry ? entry->size() : 0,
            hidden ? "hidden" : "");

    if (!hidden) {
        if (entry)
            entry->lruSeq = nextSeq();

        if (mode == BaseMMU::Write)
            stats.writeAccesses++;
        else
            stats.readAccesses++;

        if (!entry) {
            if (mode == BaseMMU::Write)
                stats.writeMisses++;
            else
                stats.readMisses++;
        }
        else {
            if (mode == BaseMMU::Write)
                stats.writeHits++;
            else
                stats.readHits++;
        }
    }

    return entry;
}

TlbEntry *
TLB::insert(Addr vpn, const TlbEntry &entry)
{
    DPRINTF(TLB, "insert(vpn=%#x, asid=%#x, key=%#x): "
                 "vaddr=%#x paddr=%#x pte=%#x size=%#x\n",
        vpn, entry.asid, buildKey(vpn, entry.asid), entry.vaddr, entry.paddr,
        entry.pte, entry.size());

    // If somebody beat us to it, just use that existing entry.
    TlbEntry *newEntry = lookup(vpn, entry.asid, BaseMMU::Read, true);
    if (newEntry) {
        // update PTE flags (maybe we set the dirty/writable flag)
        newEntry->pte = entry.pte;
        assert(newEntry->vaddr == entry.vaddr);
        assert(newEntry->asid == entry.asid);
        assert(newEntry->logBytes == entry.logBytes);
        return newEntry;
    }

    if (freeList.empty())
        evictLRU();

    newEntry = freeList.front();
    freeList.pop_front();

    Addr key = buildKey(vpn, entry.asid);
    *newEntry = entry;
    newEntry->lruSeq = nextSeq();
    newEntry->trieHandle = trie.insert(
        key, TlbEntryTrie::MaxBits - entry.logBytes + PageShift, newEntry
    );
    return newEntry;
}

void
TLB::demapPage(Addr vaddr, uint64_t asid)
{
    // Note: vaddr is Reg[rs1] and asid is Reg[rs2]
    // The definition of this instruction is
    // if vaddr=x0 and asid=x0, then flush all
    // if vaddr=x0 and asid!=x0 then flush all with matching asid
    // if vaddr!=x0 and asid=x0 then flush all leaf PTEs that match vaddr
    // if vaddr!=x0 and asid!=x0 then flush the leaf PTE that matches vaddr
    //    in the given asid.
    // No effect if vaddr is not valid
    // Currently, we assume if the values of the registers are 0 then it was
    // referencing x0.

    asid &= 0xFFFF;

    DPRINTF(TLB, "flush(vaddr=%#x, asid=%#x)\n", vaddr, asid);
    if (vaddr == 0 && asid == 0) {
        DPRINTF(TLB, "Flushing all TLB entries\n");
        flushAll();
    } else {
        if (vaddr != 0 && asid != 0) {
            // TODO: When supporting other address translation modes, fix this
            Addr vpn = getVPNFromVAddr(vaddr, AddrXlateMode::SV39);
            TlbEntry *entry = lookup(vpn, asid, BaseMMU::Read, true);
            if (entry) {
                remove(entry - tlb.data());
            }
        }
        else {
            for (size_t i = 0; i < size; i++) {
                if (tlb[i].trieHandle) {
                    Addr mask = ~(tlb[i].size() - 1);
                    if ((vaddr == 0 || (vaddr & mask) == tlb[i].vaddr) &&
                        (asid == 0 || tlb[i].asid == asid))
                        remove(i);
                }
            }
        }
    }
}

void
TLB::flushAll()
{
    DPRINTF(TLB, "flushAll()\n");
    for (size_t i = 0; i < size; i++) {
        if (tlb[i].trieHandle)
            remove(i);
    }
}

void
TLB::remove(size_t idx)
{
    DPRINTF(TLB, "remove(vpn=%#x, asid=%#x): ppn=%#x pte=%#x size=%#x\n",
        tlb[idx].vaddr, tlb[idx].asid, tlb[idx].paddr, tlb[idx].pte,
        tlb[idx].size());

    assert(tlb[idx].trieHandle);
    trie.remove(tlb[idx].trieHandle);
    tlb[idx].trieHandle = NULL;
    freeList.push_back(&tlb[idx]);
}

Fault
TLB::checkPermissions(STATUS status, PrivilegeMode pmode, Addr vaddr,
                      BaseMMU::Mode mode, PTESv39 pte)
{
    Fault fault = NoFault;

    if (mode == BaseMMU::Read && !pte.r) {
        DPRINTF(TLB, "PTE has no read perm, raising PF\n");
        fault = createPagefault(vaddr, mode);
    }
    else if (mode == BaseMMU::Write && !pte.w) {
        DPRINTF(TLB, "PTE has no write perm, raising PF\n");
        fault = createPagefault(vaddr, mode);
    }
    else if (mode == BaseMMU::Execute && !pte.x) {
        DPRINTF(TLB, "PTE has no exec perm, raising PF\n");
        fault = createPagefault(vaddr, mode);
    }

    if (fault == NoFault) {
        // check pte.u
        if (pmode == PrivilegeMode::PRV_U && !pte.u) {
            DPRINTF(TLB, "PTE is not user accessible, raising PF\n");
            fault = createPagefault(vaddr, mode);
        }
        else if (pmode == PrivilegeMode::PRV_S && pte.u && status.sum == 0) {
            DPRINTF(TLB, "PTE is only user accessible, raising PF\n");
            fault = createPagefault(vaddr, mode);
        }
    }

    return fault;
}

Fault
TLB::createPagefault(Addr vaddr, BaseMMU::Mode mode)
{
    ExceptionCode code;
    if (mode == BaseMMU::Read)
        code = ExceptionCode::LOAD_PAGE;
    else if (mode == BaseMMU::Write)
        code = ExceptionCode::STORE_PAGE;
    else
        code = ExceptionCode::INST_PAGE;
    return std::make_shared<AddressFault>(vaddr, code);
}

Addr
TLB::translateWithTLB(Addr vaddr, uint16_t asid, Addr xmode,
                      BaseMMU::Mode mode)
{
    TlbEntry *e = lookup(getVPNFromVAddr(vaddr, xmode), asid, mode, false);
    assert(e != nullptr);
    return e->paddr << PageShift | (vaddr & mask(e->logBytes));
}

Fault
TLB::doTranslate(const RequestPtr &req, ThreadContext *tc,
                 BaseMMU::Translation *translation, BaseMMU::Mode mode,
                 bool &delayed)
{
    delayed = false;

    Addr vaddr = Addr(sext<VADDR_BITS>(req->getVaddr()));
    SATP satp = tc->readMiscReg(MISCREG_SATP);

    Addr vpn = getVPNFromVAddr(vaddr, satp.mode);
    TlbEntry *e = lookup(vpn, satp.asid, mode, false);
    if (!e) {
        Fault fault = walker->start(tc, translation, req, mode);
        if (translation != nullptr || fault != NoFault) {
            // This gets ignored in atomic mode.
            delayed = true;
            return fault;
        }
        e = lookup(vpn, satp.asid, mode, true);
        assert(e != nullptr);
    }

    STATUS status = tc->readMiscReg(MISCREG_STATUS);
    PrivilegeMode pmode = getMemPriv(tc, mode);
    Fault fault = checkPermissions(status, pmode, vaddr, mode, e->pte);
    if (fault != NoFault) {
        // if we want to write and it isn't writable, do a page table walk
        // again to update the dirty flag.
        if (mode == BaseMMU::Write && !e->pte.w) {
            DPRINTF(TLB, "Dirty bit not set, repeating PT walk\n");
            fault = walker->start(tc, translation, req, mode);
            if (translation != nullptr || fault != NoFault) {
                delayed = true;
                return fault;
            }
        }
        if (fault != NoFault)
            return fault;
    }

    Addr paddr = e->paddr << PageShift | (vaddr & mask(e->logBytes));
    DPRINTF(TLBVerbose, "translate(vaddr=%#x, vpn=%#x, asid=%#x): %#x\n",
            vaddr, vpn, satp.asid, paddr);
    req->setPaddr(paddr);

    if(pma->isUncacheable(paddr, req->getSize())){
        req->setFlags(Request::UNCACHEABLE);
    }

    return NoFault;
}

PrivilegeMode
TLB::getMemPriv(ThreadContext *tc, BaseMMU::Mode mode)
{
    STATUS status = (STATUS)tc->readMiscReg(MISCREG_STATUS);
    PrivilegeMode pmode = (PrivilegeMode)tc->readMiscReg(MISCREG_PRV);
    if (mode != BaseMMU::Execute && status.mprv == 1)
        pmode = (PrivilegeMode)(RegVal)status.mpp;
    return pmode;
}

Fault
TLB::translate(const RequestPtr &req, ThreadContext *tc,
               BaseMMU::Translation *translation, BaseMMU::Mode mode,
               bool &delayed)
{
    delayed = false;

    if (FullSystem) {
        PrivilegeMode pmode = getMemPriv(tc, mode);
        MISA misa = tc->readMiscRegNoEffect(MISCREG_ISA);
        SATP satp = tc->readMiscReg(MISCREG_SATP);
        Fault fault = NoFault;

        fault = pma->checkVAddrAlignment(req, mode);

        if (!misa.rvs || pmode == PrivilegeMode::PRV_M ||
            satp.mode == AddrXlateMode::BARE) {
            req->setFlags(Request::PHYSICAL);
        }

        if (fault == NoFault) {
            if (req->getFlags() & Request::PHYSICAL) {
                /**
                 * we simply set the virtual address to physical address
                 */
                req->setPaddr(req->getVaddr());
            } else {
                fault = doTranslate(req, tc, translation, mode, delayed);
            }
        }

        if (!delayed && fault == NoFault) {
            // do pmp check if any checking condition is met.
            // timingFault will be NoFault if pmp checks are
            // passed, otherwise an address fault will be returned.
            fault = pmp->pmpCheck(req, mode, pmode, tc);
        }

        if (!delayed && fault == NoFault) {
            fault = pma->check(req, mode);
        }
        return fault;
    } else {
        // In the O3 CPU model, sometimes a memory access will be speculatively
        // executed along a branch that will end up not being taken where the
        // address is invalid.  In that case, return a fault rather than trying
        // to translate it (which will cause a panic).  Since RISC-V allows
        // unaligned memory accesses, this should only happen if the request's
        // length is long enough to wrap around from the end of the memory to
        // the start.
        assert(req->getSize() > 0);
        if (req->getVaddr() + req->getSize() - 1 < req->getVaddr())
            return std::make_shared<GenericPageTableFault>(req->getVaddr());

        Process * p = tc->getProcessPtr();

        Fault fault = p->pTable->translate(req);
        if (fault != NoFault)
            return fault;

        Addr paddr = req->getPaddr();
        if(pma->isUncacheable(paddr, req->getSize())){
            req->setFlags(Request::UNCACHEABLE);
        }

        return NoFault;
    }
}

Fault
TLB::translateAtomic(const RequestPtr &req, ThreadContext *tc,
                     BaseMMU::Mode mode)
{
    bool delayed;
    return translate(req, tc, nullptr, mode, delayed);
}

void
TLB::translateTiming(const RequestPtr &req, ThreadContext *tc,
                     BaseMMU::Translation *translation, BaseMMU::Mode mode)
{
    bool delayed;
    assert(translation);
    Fault fault = translate(req, tc, translation, mode, delayed);
    if (!delayed)
        translation->finish(fault, req, tc, mode);
    else
        translation->markDelayed();
}

Fault
TLB::translateFunctional(const RequestPtr &req, ThreadContext *tc,
                         BaseMMU::Mode mode)
{
    const Addr vaddr = req->getVaddr();
    Addr paddr = vaddr;

    if (FullSystem) {
        MMU *mmu = static_cast<MMU *>(tc->getMMUPtr());

        PrivilegeMode pmode = mmu->getMemPriv(tc, mode);
        MISA misa = tc->readMiscRegNoEffect(MISCREG_ISA);
        SATP satp = tc->readMiscReg(MISCREG_SATP);
        if (misa.rvs && pmode != PrivilegeMode::PRV_M &&
            satp.mode != AddrXlateMode::BARE) {
            Walker *walker = mmu->getDataWalker();
            unsigned logBytes;
            Fault fault = walker->startFunctional(
                    tc, paddr, logBytes, mode);
            if (fault != NoFault)
                return fault;

            Addr masked_addr = vaddr & mask(logBytes);
            paddr |= masked_addr;
        }
    }
    else {
        Process *process = tc->getProcessPtr();
        const auto *pte = process->pTable->lookup(vaddr);

        if (!pte && mode != BaseMMU::Execute) {
            // Check if we just need to grow the stack.
            if (process->fixupFault(vaddr)) {
                // If we did, lookup the entry for the new page.
                pte = process->pTable->lookup(vaddr);
            }
        }

        if (!pte)
            return std::make_shared<GenericPageTableFault>(req->getVaddr());

        paddr = pte->paddr | process->pTable->pageOffset(vaddr);
    }

    DPRINTF(TLB, "Translated (functional) %#x -> %#x.\n", vaddr, paddr);
    req->setPaddr(paddr);

    if(pma->isUncacheable(paddr, req->getSize())){
        req->setFlags(Request::UNCACHEABLE);
    }

    return NoFault;
}

Fault
TLB::finalizePhysical(const RequestPtr &req,
                      ThreadContext *tc, BaseMMU::Mode mode) const
{
    return NoFault;
}

void
TLB::serialize(CheckpointOut &cp) const
{
    // Only store the entries in use.
    uint32_t _size = size - freeList.size();
    SERIALIZE_SCALAR(_size);
    SERIALIZE_SCALAR(lruSeq);

    uint32_t _count = 0;
    for (uint32_t x = 0; x < size; x++) {
        if (tlb[x].trieHandle != NULL)
            tlb[x].serializeSection(cp, csprintf("Entry%d", _count++));
    }
}

void
TLB::unserialize(CheckpointIn &cp)
{
    // Do not allow to restore with a smaller tlb.
    uint32_t _size;
    UNSERIALIZE_SCALAR(_size);
    if (_size > size) {
        fatal("TLB size less than the one in checkpoint!");
    }

    UNSERIALIZE_SCALAR(lruSeq);

    for (uint32_t x = 0; x < _size; x++) {
        TlbEntry *newEntry = freeList.front();
        freeList.pop_front();

        newEntry->unserializeSection(cp, csprintf("Entry%d", x));
        // TODO: When supporting other addressing modes fix this
        Addr vpn = getVPNFromVAddr(newEntry->vaddr, AddrXlateMode::SV39);
        Addr key = buildKey(vpn, newEntry->asid);
        newEntry->trieHandle = trie.insert(key,
            TlbEntryTrie::MaxBits - newEntry->logBytes + PageShift, newEntry);
    }
}

TLB::TlbStats::TlbStats(statistics::Group *parent)
  : statistics::Group(parent),
    ADD_STAT(readHits, statistics::units::Count::get(), "read hits"),
    ADD_STAT(readMisses, statistics::units::Count::get(), "read misses"),
    ADD_STAT(readAccesses, statistics::units::Count::get(), "read accesses"),
    ADD_STAT(writeHits, statistics::units::Count::get(), "write hits"),
    ADD_STAT(writeMisses, statistics::units::Count::get(), "write misses"),
    ADD_STAT(writeAccesses, statistics::units::Count::get(), "write accesses"),
    ADD_STAT(hits, statistics::units::Count::get(),
             "Total TLB (read and write) hits", readHits + writeHits),
    ADD_STAT(misses, statistics::units::Count::get(),
             "Total TLB (read and write) misses", readMisses + writeMisses),
    ADD_STAT(accesses, statistics::units::Count::get(),
             "Total TLB (read and write) accesses",
             readAccesses + writeAccesses)
{
}

Port *
TLB::getTableWalkerPort()
{
    return &walker->getPort("port");
}

} // namespace gem5
