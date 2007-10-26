/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * Redistribution and use of this software in source and binary forms,
 * with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * The software must be used only for Non-Commercial Use which means any
 * use which is NOT directed to receiving any direct monetary
 * compensation for, or commercial advantage from such use.  Illustrative
 * examples of non-commercial use are academic research, personal study,
 * teaching, education and corporate research & development.
 * Illustrative examples of commercial use are distributing products for
 * commercial advantage and providing services using the software for
 * commercial advantage.
 *
 * If you wish to use this software or functionality therein that may be
 * covered by patents for commercial use, please contact:
 *     Director of Intellectual Property Licensing
 *     Office of Strategy and Technology
 *     Hewlett-Packard Company
 *     1501 Page Mill Road
 *     Palo Alto, California  94304
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.  Redistributions
 * in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.  Neither the name of
 * the COPYRIGHT HOLDER(s), HEWLETT-PACKARD COMPANY, nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.  No right of
 * sublicense is granted herewith.  Derivatives of the software and
 * output created using the software may be prepared, but only for
 * Non-Commercial Uses.  Derivatives of the software may be shared with
 * others provided: (i) the others agree to abide by the list of
 * conditions herein which includes the Non-Commercial Use restrictions;
 * and (ii) such Derivatives of the software include the above copyright
 * notice to acknowledge the contribution from this software where
 * applicable, this list of conditions and the disclaimer below.
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

#include "arch/x86/pagetable.hh"
#include "arch/x86/tlb.hh"
#include "arch/x86/x86_traits.hh"
#include "base/bitfield.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "cpu/base.hh"
#include "mem/packet_access.hh"
#include "mem/request.hh"
#include "sim/system.hh"

namespace X86ISA {

TLB::TLB(const Params *p) : SimObject(p), size(p->size)
{
    tlb = new TlbEntry[size];
    std::memset(tlb, 0, sizeof(TlbEntry) * size);

    for (int x = 0; x < size; x++)
        freeList.push_back(&tlb[x]);
}

void
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
}

TlbEntry *
TLB::lookup(Addr va, bool update_lru)
{
    //TODO make this smarter at some point
    EntryList::iterator entry;
    for (entry = entryList.begin(); entry != entryList.end(); entry++) {
        if ((*entry)->vaddr <= va && (*entry)->vaddr + (*entry)->size > va) {
            DPRINTF(TLB, "Matched vaddr %#x to entry starting at %#x "
                    "with size %#x.\n", va, (*entry)->vaddr, (*entry)->size);
            TlbEntry *e = *entry;
            if (update_lru) {
                entryList.erase(entry);
                entryList.push_front(e);
            }
            return e;
        }
    }
    return NULL;
}

void
TLB::invalidateAll()
{
}

void
TLB::invalidateNonGlobal()
{
}

void
TLB::demapPage(Addr va)
{
}

template<class TlbFault>
Fault
TLB::translate(RequestPtr &req, ThreadContext *tc, bool write, bool execute)
{
    Addr vaddr = req->getVaddr();
    DPRINTF(TLB, "Translating vaddr %#x.\n", vaddr);
    uint32_t flags = req->getFlags();
    bool storeCheck = flags & StoreCheck;

    int seg = flags & mask(3);

    //XXX Junk code to surpress the warning
    if (storeCheck);

    // If this is true, we're dealing with a request to read an internal
    // value.
    if (seg == NUM_SEGMENTREGS) {
        Addr prefix = vaddr & IntAddrPrefixMask;
        if (prefix == IntAddrPrefixCPUID) {
            panic("CPUID memory space not yet implemented!\n");
        } else if (prefix == IntAddrPrefixMSR) {
            req->setMmapedIpr(true);
            Addr regNum = 0;
            switch (vaddr & ~IntAddrPrefixMask) {
              case 0x10:
                regNum = MISCREG_TSC;
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
        } else {
            panic("Access to unrecognized internal address space %#x.\n",
                    prefix);
        }
    }

    // Get cr0. This will tell us how to do translation. We'll assume it was
    // verified to be correct and consistent when set.
    CR0 cr0 = tc->readMiscRegNoEffect(MISCREG_CR0);

    // If protected mode has been enabled...
    if (cr0.pe) {
        Efer efer = tc->readMiscRegNoEffect(MISCREG_EFER);
        SegAttr csAttr = tc->readMiscRegNoEffect(MISCREG_CS_ATTR);
        // If we're not in 64-bit mode, do protection/limit checks
        if (!efer.lma || !csAttr.longMode) {
            SegAttr attr = tc->readMiscRegNoEffect(MISCREG_SEG_ATTR(seg));
            if (!attr.writable && write)
                return new GeneralProtection(0);
            if (!attr.readable && !write && !execute)
                return new GeneralProtection(0);
            Addr base = tc->readMiscRegNoEffect(MISCREG_SEG_BASE(seg));
            Addr limit = tc->readMiscRegNoEffect(MISCREG_SEG_LIMIT(seg));
            if (!attr.expandDown) {
                // We don't have to worry about the access going around the
                // end of memory because accesses will be broken up into
                // pieces at boundaries aligned on sizes smaller than an
                // entire address space. We do have to worry about the limit
                // being less than the base.
                if (limit < base) {
                    if (limit < vaddr + req->getSize() && vaddr < base)
                        return new GeneralProtection(0);
                } else {
                    if (limit < vaddr + req->getSize())
                        return new GeneralProtection(0);
                }
            } else {
                if (limit < base) {
                    if (vaddr <= limit || vaddr + req->getSize() >= base)
                        return new GeneralProtection(0);
                } else {
                    if (vaddr <= limit && vaddr + req->getSize() >= base)
                        return new GeneralProtection(0);
                }
            }
        }
        // If paging is enabled, do the translation.
        if (cr0.pg) {
            // The vaddr already has the segment base applied.
            TlbEntry *entry = lookup(vaddr);
            if (!entry) {
#if FULL_SYSTEM
                return new TlbFault();
#else
                return new TlbFault(vaddr);
#endif
            } else {
                // Do paging protection checks.
                Addr paddr = entry->paddr | (vaddr & mask(12));
                req->setPaddr(paddr);
            }
        } else {
            //Use the address which already has segmentation applied.
            req->setPaddr(vaddr);
        }
    } else {
        // Real mode
        req->setPaddr(vaddr);
    }
    return NoFault;
};

Fault
DTB::translate(RequestPtr &req, ThreadContext *tc, bool write)
{
    return TLB::translate<FakeDTLBFault>(req, tc, write, false);
}

Fault
ITB::translate(RequestPtr &req, ThreadContext *tc)
{
    return TLB::translate<FakeITLBFault>(req, tc, false, true);
}

#if FULL_SYSTEM

Tick
DTB::doMmuRegRead(ThreadContext *tc, Packet *pkt)
{
    return tc->getCpuPtr()->ticks(1);
}

Tick
DTB::doMmuRegWrite(ThreadContext *tc, Packet *pkt)
{
    return tc->getCpuPtr()->ticks(1);
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

void
DTB::serialize(std::ostream &os)
{
    TLB::serialize(os);
}

void
DTB::unserialize(Checkpoint *cp, const std::string &section)
{
    TLB::unserialize(cp, section);
}

/* end namespace X86ISA */ }

X86ISA::ITB *
X86ITBParams::create()
{
    return new X86ISA::ITB(this);
}

X86ISA::DTB *
X86DTBParams::create()
{
    return new X86ISA::DTB(this);
}
