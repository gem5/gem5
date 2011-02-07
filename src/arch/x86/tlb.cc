/*
 * Copyright (c) 2007-2008 The Hewlett-Packard Development Company
 * All rights reserved.
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
 *
 * Authors: Gabe Black
 */

#include <cstring>

#include "config/full_system.hh"

#include "arch/x86/faults.hh"
#include "arch/x86/insts/microldstop.hh"
#include "arch/x86/pagetable.hh"
#include "arch/x86/regs/misc.hh"
#include "arch/x86/tlb.hh"
#include "arch/x86/x86_traits.hh"
#include "base/bitfield.hh"
#include "base/trace.hh"
#include "config/full_system.hh"
#include "cpu/thread_context.hh"
#include "cpu/base.hh"
#include "mem/packet_access.hh"
#include "mem/request.hh"

#if FULL_SYSTEM
#include "arch/x86/pagetable_walker.hh"
#else
#include "mem/page_table.hh"
#include "sim/process.hh"
#endif

namespace X86ISA {

TLB::TLB(const Params *p) : BaseTLB(p), configAddress(0), size(p->size)
{
    tlb = new TlbEntry[size];
    std::memset(tlb, 0, sizeof(TlbEntry) * size);

    for (int x = 0; x < size; x++)
        freeList.push_back(&tlb[x]);

#if FULL_SYSTEM
    walker = p->walker;
    walker->setTLB(this);
#endif
}

TlbEntry *
TLB::insert(Addr vpn, TlbEntry &entry)
{
    //TODO Deal with conflicting entries

    TlbEntry *newEntry = NULL;
    if (!freeList.empty()) {
        newEntry = freeList.front();
        freeList.pop_front();
    } else {
        newEntry = entryList.back();
        entryList.pop_back();
    }
    *newEntry = entry;
    newEntry->vaddr = vpn;
    entryList.push_front(newEntry);
    return newEntry;
}

TLB::EntryList::iterator
TLB::lookupIt(Addr va, bool update_lru)
{
    //TODO make this smarter at some point
    EntryList::iterator entry;
    for (entry = entryList.begin(); entry != entryList.end(); entry++) {
        if ((*entry)->vaddr <= va && (*entry)->vaddr + (*entry)->size > va) {
            DPRINTF(TLB, "Matched vaddr %#x to entry starting at %#x "
                    "with size %#x.\n", va, (*entry)->vaddr, (*entry)->size);
            if (update_lru) {
                entryList.push_front(*entry);
                entryList.erase(entry);
                entry = entryList.begin();
            }
            break;
        }
    }
    return entry;
}

TlbEntry *
TLB::lookup(Addr va, bool update_lru)
{
    EntryList::iterator entry = lookupIt(va, update_lru);
    if (entry == entryList.end())
        return NULL;
    else
        return *entry;
}

void
TLB::invalidateAll()
{
    DPRINTF(TLB, "Invalidating all entries.\n");
    while (!entryList.empty()) {
        TlbEntry *entry = entryList.front();
        entryList.pop_front();
        freeList.push_back(entry);
    }
}

void
TLB::setConfigAddress(uint32_t addr)
{
    configAddress = addr;
}

void
TLB::invalidateNonGlobal()
{
    DPRINTF(TLB, "Invalidating all non global entries.\n");
    EntryList::iterator entryIt;
    for (entryIt = entryList.begin(); entryIt != entryList.end();) {
        if (!(*entryIt)->global) {
            freeList.push_back(*entryIt);
            entryList.erase(entryIt++);
        } else {
            entryIt++;
        }
    }
}

void
TLB::demapPage(Addr va, uint64_t asn)
{
    EntryList::iterator entry = lookupIt(va, false);
    if (entry != entryList.end()) {
        freeList.push_back(*entry);
        entryList.erase(entry);
    }
}

Fault
TLB::translateInt(RequestPtr req, ThreadContext *tc)
{
    DPRINTF(TLB, "Addresses references internal memory.\n");
    Addr vaddr = req->getVaddr();
    Addr prefix = (vaddr >> 3) & IntAddrPrefixMask;
    if (prefix == IntAddrPrefixCPUID) {
        panic("CPUID memory space not yet implemented!\n");
    } else if (prefix == IntAddrPrefixMSR) {
        vaddr = vaddr >> 3;
        req->setFlags(Request::MMAPED_IPR);
        Addr regNum = 0;
        switch (vaddr & ~IntAddrPrefixMask) {
          case 0x10:
            regNum = MISCREG_TSC;
            break;
          case 0x1B:
            regNum = MISCREG_APIC_BASE;
            break;
          case 0xFE:
            regNum = MISCREG_MTRRCAP;
            break;
          case 0x174:
            regNum = MISCREG_SYSENTER_CS;
            break;
          case 0x175:
            regNum = MISCREG_SYSENTER_ESP;
            break;
          case 0x176:
            regNum = MISCREG_SYSENTER_EIP;
            break;
          case 0x179:
            regNum = MISCREG_MCG_CAP;
            break;
          case 0x17A:
            regNum = MISCREG_MCG_STATUS;
            break;
          case 0x17B:
            regNum = MISCREG_MCG_CTL;
            break;
          case 0x1D9:
            regNum = MISCREG_DEBUG_CTL_MSR;
            break;
          case 0x1DB:
            regNum = MISCREG_LAST_BRANCH_FROM_IP;
            break;
          case 0x1DC:
            regNum = MISCREG_LAST_BRANCH_TO_IP;
            break;
          case 0x1DD:
            regNum = MISCREG_LAST_EXCEPTION_FROM_IP;
            break;
          case 0x1DE:
            regNum = MISCREG_LAST_EXCEPTION_TO_IP;
            break;
          case 0x200:
            regNum = MISCREG_MTRR_PHYS_BASE_0;
            break;
          case 0x201:
            regNum = MISCREG_MTRR_PHYS_MASK_0;
            break;
          case 0x202:
            regNum = MISCREG_MTRR_PHYS_BASE_1;
            break;
          case 0x203:
            regNum = MISCREG_MTRR_PHYS_MASK_1;
            break;
          case 0x204:
            regNum = MISCREG_MTRR_PHYS_BASE_2;
            break;
          case 0x205:
            regNum = MISCREG_MTRR_PHYS_MASK_2;
            break;
          case 0x206:
            regNum = MISCREG_MTRR_PHYS_BASE_3;
            break;
          case 0x207:
            regNum = MISCREG_MTRR_PHYS_MASK_3;
            break;
          case 0x208:
            regNum = MISCREG_MTRR_PHYS_BASE_4;
            break;
          case 0x209:
            regNum = MISCREG_MTRR_PHYS_MASK_4;
            break;
          case 0x20A:
            regNum = MISCREG_MTRR_PHYS_BASE_5;
            break;
          case 0x20B:
            regNum = MISCREG_MTRR_PHYS_MASK_5;
            break;
          case 0x20C:
            regNum = MISCREG_MTRR_PHYS_BASE_6;
            break;
          case 0x20D:
            regNum = MISCREG_MTRR_PHYS_MASK_6;
            break;
          case 0x20E:
            regNum = MISCREG_MTRR_PHYS_BASE_7;
            break;
          case 0x20F:
            regNum = MISCREG_MTRR_PHYS_MASK_7;
            break;
          case 0x250:
            regNum = MISCREG_MTRR_FIX_64K_00000;
            break;
          case 0x258:
            regNum = MISCREG_MTRR_FIX_16K_80000;
            break;
          case 0x259:
            regNum = MISCREG_MTRR_FIX_16K_A0000;
            break;
          case 0x268:
            regNum = MISCREG_MTRR_FIX_4K_C0000;
            break;
          case 0x269:
            regNum = MISCREG_MTRR_FIX_4K_C8000;
            break;
          case 0x26A:
            regNum = MISCREG_MTRR_FIX_4K_D0000;
            break;
          case 0x26B:
            regNum = MISCREG_MTRR_FIX_4K_D8000;
            break;
          case 0x26C:
            regNum = MISCREG_MTRR_FIX_4K_E0000;
            break;
          case 0x26D:
            regNum = MISCREG_MTRR_FIX_4K_E8000;
            break;
          case 0x26E:
            regNum = MISCREG_MTRR_FIX_4K_F0000;
            break;
          case 0x26F:
            regNum = MISCREG_MTRR_FIX_4K_F8000;
            break;
          case 0x277:
            regNum = MISCREG_PAT;
            break;
          case 0x2FF:
            regNum = MISCREG_DEF_TYPE;
            break;
          case 0x400:
            regNum = MISCREG_MC0_CTL;
            break;
          case 0x404:
            regNum = MISCREG_MC1_CTL;
            break;
          case 0x408:
            regNum = MISCREG_MC2_CTL;
            break;
          case 0x40C:
            regNum = MISCREG_MC3_CTL;
            break;
          case 0x410:
            regNum = MISCREG_MC4_CTL;
            break;
          case 0x414:
            regNum = MISCREG_MC5_CTL;
            break;
          case 0x418:
            regNum = MISCREG_MC6_CTL;
            break;
          case 0x41C:
            regNum = MISCREG_MC7_CTL;
            break;
          case 0x401:
            regNum = MISCREG_MC0_STATUS;
            break;
          case 0x405:
            regNum = MISCREG_MC1_STATUS;
            break;
          case 0x409:
            regNum = MISCREG_MC2_STATUS;
            break;
          case 0x40D:
            regNum = MISCREG_MC3_STATUS;
            break;
          case 0x411:
            regNum = MISCREG_MC4_STATUS;
            break;
          case 0x415:
            regNum = MISCREG_MC5_STATUS;
            break;
          case 0x419:
            regNum = MISCREG_MC6_STATUS;
            break;
          case 0x41D:
            regNum = MISCREG_MC7_STATUS;
            break;
          case 0x402:
            regNum = MISCREG_MC0_ADDR;
            break;
          case 0x406:
            regNum = MISCREG_MC1_ADDR;
            break;
          case 0x40A:
            regNum = MISCREG_MC2_ADDR;
            break;
          case 0x40E:
            regNum = MISCREG_MC3_ADDR;
            break;
          case 0x412:
            regNum = MISCREG_MC4_ADDR;
            break;
          case 0x416:
            regNum = MISCREG_MC5_ADDR;
            break;
          case 0x41A:
            regNum = MISCREG_MC6_ADDR;
            break;
          case 0x41E:
            regNum = MISCREG_MC7_ADDR;
            break;
          case 0x403:
            regNum = MISCREG_MC0_MISC;
            break;
          case 0x407:
            regNum = MISCREG_MC1_MISC;
            break;
          case 0x40B:
            regNum = MISCREG_MC2_MISC;
            break;
          case 0x40F:
            regNum = MISCREG_MC3_MISC;
            break;
          case 0x413:
            regNum = MISCREG_MC4_MISC;
            break;
          case 0x417:
            regNum = MISCREG_MC5_MISC;
            break;
          case 0x41B:
            regNum = MISCREG_MC6_MISC;
            break;
          case 0x41F:
            regNum = MISCREG_MC7_MISC;
            break;
          case 0xC0000080:
            regNum = MISCREG_EFER;
            break;
          case 0xC0000081:
            regNum = MISCREG_STAR;
            break;
          case 0xC0000082:
            regNum = MISCREG_LSTAR;
            break;
          case 0xC0000083:
            regNum = MISCREG_CSTAR;
            break;
          case 0xC0000084:
            regNum = MISCREG_SF_MASK;
            break;
          case 0xC0000100:
            regNum = MISCREG_FS_BASE;
            break;
          case 0xC0000101:
            regNum = MISCREG_GS_BASE;
            break;
          case 0xC0000102:
            regNum = MISCREG_KERNEL_GS_BASE;
            break;
          case 0xC0000103:
            regNum = MISCREG_TSC_AUX;
            break;
          case 0xC0010000:
            regNum = MISCREG_PERF_EVT_SEL0;
            break;
          case 0xC0010001:
            regNum = MISCREG_PERF_EVT_SEL1;
            break;
          case 0xC0010002:
            regNum = MISCREG_PERF_EVT_SEL2;
            break;
          case 0xC0010003:
            regNum = MISCREG_PERF_EVT_SEL3;
            break;
          case 0xC0010004:
            regNum = MISCREG_PERF_EVT_CTR0;
            break;
          case 0xC0010005:
            regNum = MISCREG_PERF_EVT_CTR1;
            break;
          case 0xC0010006:
            regNum = MISCREG_PERF_EVT_CTR2;
            break;
          case 0xC0010007:
            regNum = MISCREG_PERF_EVT_CTR3;
            break;
          case 0xC0010010:
            regNum = MISCREG_SYSCFG;
            break;
          case 0xC0010016:
            regNum = MISCREG_IORR_BASE0;
            break;
          case 0xC0010017:
            regNum = MISCREG_IORR_BASE1;
            break;
          case 0xC0010018:
            regNum = MISCREG_IORR_MASK0;
            break;
          case 0xC0010019:
            regNum = MISCREG_IORR_MASK1;
            break;
          case 0xC001001A:
            regNum = MISCREG_TOP_MEM;
            break;
          case 0xC001001D:
            regNum = MISCREG_TOP_MEM2;
            break;
          case 0xC0010114:
            regNum = MISCREG_VM_CR;
            break;
          case 0xC0010115:
            regNum = MISCREG_IGNNE;
            break;
          case 0xC0010116:
            regNum = MISCREG_SMM_CTL;
            break;
          case 0xC0010117:
            regNum = MISCREG_VM_HSAVE_PA;
            break;
          default:
            return new GeneralProtection(0);
        }
        //The index is multiplied by the size of a MiscReg so that
        //any memory dependence calculations will not see these as
        //overlapping.
        req->setPaddr(regNum * sizeof(MiscReg));
        return NoFault;
    } else if (prefix == IntAddrPrefixIO) {
        // TODO If CPL > IOPL or in virtual mode, check the I/O permission
        // bitmap in the TSS.

        Addr IOPort = vaddr & ~IntAddrPrefixMask;
        // Make sure the address fits in the expected 16 bit IO address
        // space.
        assert(!(IOPort & ~0xFFFF));
        if (IOPort == 0xCF8 && req->getSize() == 4) {
            req->setFlags(Request::MMAPED_IPR);
            req->setPaddr(MISCREG_PCI_CONFIG_ADDRESS * sizeof(MiscReg));
        } else if ((IOPort & ~mask(2)) == 0xCFC) {
            req->setFlags(Request::UNCACHEABLE);
            Addr configAddress =
                tc->readMiscRegNoEffect(MISCREG_PCI_CONFIG_ADDRESS);
            if (bits(configAddress, 31, 31)) {
                req->setPaddr(PhysAddrPrefixPciConfig |
                        mbits(configAddress, 30, 2) |
                        (IOPort & mask(2)));
            }
        } else {
            req->setFlags(Request::UNCACHEABLE);
            req->setPaddr(PhysAddrPrefixIO | IOPort);
        }
        return NoFault;
    } else {
        panic("Access to unrecognized internal address space %#x.\n",
                prefix);
    }
}

Fault
TLB::translate(RequestPtr req, ThreadContext *tc, Translation *translation,
        Mode mode, bool &delayedResponse, bool timing)
{
    uint32_t flags = req->getFlags();
    int seg = flags & SegmentFlagMask;
    bool storeCheck = flags & (StoreCheck << FlagShift);

    // If this is true, we're dealing with a request to a non-memory address
    // space.
    if (seg == SEGMENT_REG_MS) {
        return translateInt(req, tc);
    }

    delayedResponse = false;
    Addr vaddr = req->getVaddr();
    DPRINTF(TLB, "Translating vaddr %#x.\n", vaddr);

    HandyM5Reg m5Reg = tc->readMiscRegNoEffect(MISCREG_M5_REG);

    // If protected mode has been enabled...
    if (m5Reg.prot) {
        DPRINTF(TLB, "In protected mode.\n");
        // If we're not in 64-bit mode, do protection/limit checks
        if (m5Reg.mode != LongMode) {
            DPRINTF(TLB, "Not in long mode. Checking segment protection.\n");
            // Check for a NULL segment selector.
            if (!(seg == SEGMENT_REG_TSG || seg == SYS_SEGMENT_REG_IDTR ||
                        seg == SEGMENT_REG_HS || seg == SEGMENT_REG_LS)
                    && !tc->readMiscRegNoEffect(MISCREG_SEG_SEL(seg)))
                return new GeneralProtection(0);
            bool expandDown = false;
            SegAttr attr = tc->readMiscRegNoEffect(MISCREG_SEG_ATTR(seg));
            if (seg >= SEGMENT_REG_ES && seg <= SEGMENT_REG_HS) {
                if (!attr.writable && (mode == Write || storeCheck))
                    return new GeneralProtection(0);
                if (!attr.readable && mode == Read)
                    return new GeneralProtection(0);
                expandDown = attr.expandDown;

            }
            Addr base = tc->readMiscRegNoEffect(MISCREG_SEG_BASE(seg));
            Addr limit = tc->readMiscRegNoEffect(MISCREG_SEG_LIMIT(seg));
            // This assumes we're not in 64 bit mode. If we were, the default
            // address size is 64 bits, overridable to 32.
            int size = 32;
            bool sizeOverride = (flags & (AddrSizeFlagBit << FlagShift));
            SegAttr csAttr = tc->readMiscRegNoEffect(MISCREG_CS_ATTR);
            if ((csAttr.defaultSize && sizeOverride) ||
                    (!csAttr.defaultSize && !sizeOverride))
                size = 16;
            Addr offset = bits(vaddr - base, size-1, 0);
            Addr endOffset = offset + req->getSize() - 1;
            if (expandDown) {
                DPRINTF(TLB, "Checking an expand down segment.\n");
                warn_once("Expand down segments are untested.\n");
                if (offset <= limit || endOffset <= limit)
                    return new GeneralProtection(0);
            } else {
                if (offset > limit || endOffset > limit)
                    return new GeneralProtection(0);
            }
        }
        // If paging is enabled, do the translation.
        if (m5Reg.paging) {
            DPRINTF(TLB, "Paging enabled.\n");
            // The vaddr already has the segment base applied.
            TlbEntry *entry = lookup(vaddr);
            if (!entry) {
#if FULL_SYSTEM
                Fault fault = walker->start(tc, translation, req, mode);
                if (timing || fault != NoFault) {
                    // This gets ignored in atomic mode.
                    delayedResponse = true;
                    return fault;
                }
                entry = lookup(vaddr);
                assert(entry);
#else
                DPRINTF(TLB, "Handling a TLB miss for "
                        "address %#x at pc %#x.\n",
                        vaddr, tc->instAddr());

                Process *p = tc->getProcessPtr();
                TlbEntry newEntry;
                bool success = p->pTable->lookup(vaddr, newEntry);
                if (!success && mode != Execute) {
                    p->checkAndAllocNextPage(vaddr);
                    success = p->pTable->lookup(vaddr, newEntry);
                }
                if (!success) {
                    return new PageFault(vaddr, true, mode, true, false);
                } else {
                    Addr alignedVaddr = p->pTable->pageAlign(vaddr);
                    DPRINTF(TLB, "Mapping %#x to %#x\n", alignedVaddr,
                            newEntry.pageStart());
                    entry = insert(alignedVaddr, newEntry);
                }
                DPRINTF(TLB, "Miss was serviced.\n");
#endif
            }
            // Do paging protection checks.
            bool inUser = (m5Reg.cpl == 3 &&
                    !(flags & (CPL0FlagBit << FlagShift)));
            CR0 cr0 = tc->readMiscRegNoEffect(MISCREG_CR0);
            bool badWrite = (!entry->writable && (inUser || cr0.wp));
            if ((inUser && !entry->user) || (mode == Write && badWrite)) {
                // The page must have been present to get into the TLB in
                // the first place. We'll assume the reserved bits are
                // fine even though we're not checking them.
                return new PageFault(vaddr, true, mode, inUser, false);
            }
            if (storeCheck && badWrite) {
                // This would fault if this were a write, so return a page
                // fault that reflects that happening.
                return new PageFault(vaddr, true, Write, inUser, false);
            }


            DPRINTF(TLB, "Entry found with paddr %#x, "
                    "doing protection checks.\n", entry->paddr);
            Addr paddr = entry->paddr | (vaddr & (entry->size-1));
            DPRINTF(TLB, "Translated %#x -> %#x.\n", vaddr, paddr);
            req->setPaddr(paddr);
            if (entry->uncacheable)
                req->setFlags(Request::UNCACHEABLE);
        } else {
            //Use the address which already has segmentation applied.
            DPRINTF(TLB, "Paging disabled.\n");
            DPRINTF(TLB, "Translated %#x -> %#x.\n", vaddr, vaddr);
            req->setPaddr(vaddr);
        }
    } else {
        // Real mode
        DPRINTF(TLB, "In real mode.\n");
        DPRINTF(TLB, "Translated %#x -> %#x.\n", vaddr, vaddr);
        req->setPaddr(vaddr);
    }
    // Check for an access to the local APIC
#if FULL_SYSTEM
    LocalApicBase localApicBase = tc->readMiscRegNoEffect(MISCREG_APIC_BASE);
    Addr baseAddr = localApicBase.base * PageBytes;
    Addr paddr = req->getPaddr();
    if (baseAddr <= paddr && baseAddr + PageBytes > paddr) {
        // The Intel developer's manuals say the below restrictions apply,
        // but the linux kernel, because of a compiler optimization, breaks
        // them.
        /*
        // Check alignment
        if (paddr & ((32/8) - 1))
            return new GeneralProtection(0);
        // Check access size
        if (req->getSize() != (32/8))
            return new GeneralProtection(0);
        */
        // Force the access to be uncacheable.
        req->setFlags(Request::UNCACHEABLE);
        req->setPaddr(x86LocalAPICAddress(tc->contextId(), paddr - baseAddr));
    }
#endif
    return NoFault;
};

Fault
TLB::translateAtomic(RequestPtr req, ThreadContext *tc, Mode mode)
{
    bool delayedResponse;
    return TLB::translate(req, tc, NULL, mode, delayedResponse, false);
}

void
TLB::translateTiming(RequestPtr req, ThreadContext *tc,
        Translation *translation, Mode mode)
{
    bool delayedResponse;
    assert(translation);
    Fault fault =
        TLB::translate(req, tc, translation, mode, delayedResponse, true);
    if (!delayedResponse)
        translation->finish(fault, req, tc, mode);
}

#if FULL_SYSTEM

Tick
TLB::doMmuRegRead(ThreadContext *tc, Packet *pkt)
{
    return tc->getCpuPtr()->ticks(1);
}

Tick
TLB::doMmuRegWrite(ThreadContext *tc, Packet *pkt)
{
    return tc->getCpuPtr()->ticks(1);
}

Walker *
TLB::getWalker()
{
    return walker;
}

#endif

void
TLB::serialize(std::ostream &os)
{
}

void
TLB::unserialize(Checkpoint *cp, const std::string &section)
{
}

} // namespace X86ISA

X86ISA::TLB *
X86TLBParams::create()
{
    return new X86ISA::TLB(this);
}
