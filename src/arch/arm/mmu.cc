/*
 * Copyright (c) 2010-2013, 2016-2023 Arm Limited
 * All rights reserved
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

#include "arch/arm/mmu.hh"

#include "arch/arm/isa.hh"
#include "arch/arm/reg_abi.hh"
#include "arch/arm/stage2_lookup.hh"
#include "arch/arm/table_walker.hh"
#include "arch/arm/tlbi_op.hh"
#include "debug/TLB.hh"
#include "debug/TLBVerbose.hh"
#include "mem/packet_access.hh"
#include "sim/pseudo_inst.hh"
#include "sim/process.hh"

namespace gem5
{

using namespace ArmISA;

MMU::MMU(const ArmMMUParams &p)
  : BaseMMU(p),
    itbStage2(p.stage2_itb), dtbStage2(p.stage2_dtb),
    itbWalker(p.itb_walker), dtbWalker(p.dtb_walker),
    itbStage2Walker(p.stage2_itb_walker),
    dtbStage2Walker(p.stage2_dtb_walker),
    test(nullptr),
    miscRegContext(0),
    s1State(this, false), s2State(this, true),
    _attr(0),
    _release(nullptr),
    _hasWalkCache(false),
    stats(this)
{
    // Cache system-level properties
    if (FullSystem) {
        ArmSystem *arm_sys = dynamic_cast<ArmSystem *>(p.sys);
        assert(arm_sys);
        haveLargeAsid64 = arm_sys->haveLargeAsid64();
        physAddrRange = arm_sys->physAddrRange();

        _release = arm_sys->releaseFS();
    } else {
        haveLargeAsid64 = false;
        physAddrRange = 48;

        _release = p.release_se;
    }

    m5opRange = p.sys->m5opRange();
}

void
MMU::init()
{
    itbWalker->setMmu(this);
    dtbWalker->setMmu(this);
    itbStage2Walker->setMmu(this);
    dtbStage2Walker->setMmu(this);

    itbStage2->setTableWalker(itbStage2Walker);
    dtbStage2->setTableWalker(dtbStage2Walker);

    getITBPtr()->setTableWalker(itbWalker);
    getDTBPtr()->setTableWalker(dtbWalker);

    BaseMMU::init();

    _hasWalkCache = checkWalkCache();
}

bool
MMU::checkWalkCache() const
{
    for (auto tlb : instruction) {
        if (static_cast<TLB*>(tlb)->walkCache())
            return true;
    }
    for (auto tlb : data) {
        if (static_cast<TLB*>(tlb)->walkCache())
            return true;
    }
    for (auto tlb : unified) {
        if (static_cast<TLB*>(tlb)->walkCache())
            return true;
    }

    return false;
}

void
MMU::drainResume()
{
    s1State.miscRegValid = false;
    s2State.miscRegValid = false;
}

TLB *
MMU::getTlb(BaseMMU::Mode mode, bool stage2) const
{
    if (mode == BaseMMU::Execute) {
        if (stage2)
            return itbStage2;
        else
            return getITBPtr();
    } else {
        if (stage2)
            return dtbStage2;
        else
            return getDTBPtr();
    }
}

TableWalker *
MMU::getTableWalker(BaseMMU::Mode mode, bool stage2) const
{
    if (mode == BaseMMU::Execute) {
        if (stage2)
            return itbStage2Walker;
        else
            return itbWalker;
    } else {
        if (stage2)
            return dtbStage2Walker;
        else
            return dtbWalker;
    }
}

bool
MMU::translateFunctional(ThreadContext *tc, Addr va, Addr &pa)
{
    CachedState& state = updateMiscReg(tc, NormalTran, false);

    auto tlb = getTlb(BaseMMU::Read, state.directToStage2);

    TlbEntry::Lookup lookup_data;

    lookup_data.va = va;
    lookup_data.asn = state.asid;
    lookup_data.ignoreAsn = false;
    lookup_data.vmid = state.vmid;
    lookup_data.secure = state.isSecure;
    lookup_data.functional = true;
    lookup_data.targetEL = state.aarch64EL;
    lookup_data.inHost = false;
    lookup_data.mode = BaseMMU::Read;

    TlbEntry *e = tlb->multiLookup(lookup_data);

    if (!e)
        return false;
    pa = e->pAddr(va);
    return true;
}

void
MMU::invalidateMiscReg()
{
    s1State.miscRegValid = false;
    s1State.computeAddrTop.flush();
    s2State.computeAddrTop.flush();
}

Fault
MMU::finalizePhysical(const RequestPtr &req,
                      ThreadContext *tc, Mode mode) const
{
    const Addr paddr = req->getPaddr();

    if (m5opRange.contains(paddr)) {
        uint8_t func;
        pseudo_inst::decodeAddrOffset(paddr - m5opRange.start(), func);
        req->setLocalAccessor(
            [func, mode](ThreadContext *tc, PacketPtr pkt) -> Cycles
            {
                uint64_t ret;
                if (inAArch64(tc))
                    pseudo_inst::pseudoInst<RegABI64>(tc, func, ret);
                else
                    pseudo_inst::pseudoInst<RegABI32>(tc, func, ret);

                if (mode == Read)
                    pkt->setLE(ret);

                return Cycles(1);
            }
        );
    }

    return NoFault;
}


Fault
MMU::translateSe(const RequestPtr &req, ThreadContext *tc, Mode mode,
                 Translation *translation, bool &delay, bool timing,
                 CachedState &state)
{
    updateMiscReg(tc, NormalTran, state.isStage2);
    Addr vaddr_tainted = req->getVaddr();
    Addr vaddr = 0;
    if (state.aarch64) {
        vaddr = purifyTaggedAddr(vaddr_tainted, tc, state.aarch64EL,
            static_cast<TCR>(state.ttbcr), mode==Execute, state);
    } else {
        vaddr = vaddr_tainted;
    }
    Request::Flags flags = req->getFlags();

    bool is_fetch = (mode == Execute);
    bool is_write = (mode == Write);

    if (!is_fetch) {
        if (state.sctlr.a || !(flags & AllowUnaligned)) {
            if (vaddr & mask(flags & AlignmentMask)) {
                // LPAE is always disabled in SE mode
                return std::make_shared<DataAbort>(
                    vaddr_tainted,
                    TlbEntry::DomainType::NoAccess, is_write,
                    ArmFault::AlignmentFault, state.isStage2,
                    ArmFault::VmsaTran);
            }
        }
    }

    Process *p = tc->getProcessPtr();
    if (const auto pte = p->pTable->lookup(vaddr); !pte) {
        return std::make_shared<GenericPageTableFault>(vaddr_tainted);
    } else {
        req->setPaddr(pte->paddr + p->pTable->pageOffset(vaddr));

        if (pte->flags & EmulationPageTable::Uncacheable)
            req->setFlags(Request::UNCACHEABLE);

        return finalizePhysical(req, tc, mode);
    }
}

Fault
MMU::checkPermissions(TlbEntry *te, const RequestPtr &req, Mode mode,
                      bool stage2)
{
    return checkPermissions(te, req, mode, stage2 ? s2State : s1State);
}

Fault
MMU::checkPermissions(TlbEntry *te, const RequestPtr &req, Mode mode,
                      CachedState &state)
{
    // a data cache maintenance instruction that operates by MVA does
    // not generate a Data Abort exeception due to a Permission fault
    if (req->isCacheMaintenance()) {
        return NoFault;
    }

    Addr vaddr = req->getVaddr(); // 32-bit don't have to purify
    Request::Flags flags = req->getFlags();
    bool is_fetch  = (mode == Execute);
    bool is_write  = (mode == Write);
    bool is_priv   = state.isPriv && !(flags & UserMode);

    // Get the translation type from the actuall table entry
    ArmFault::TranMethod tranMethod = te->longDescFormat ? ArmFault::LpaeTran
                                                         : ArmFault::VmsaTran;

    // If this is the second stage of translation and the request is for a
    // stage 1 page table walk then we need to check the HCR.PTW bit. This
    // allows us to generate a fault if the request targets an area marked
    // as a device or strongly ordered.
    if (state.isStage2 && req->isPTWalk() && state.hcr.ptw &&
        (te->mtype != TlbEntry::MemoryType::Normal)) {
        return std::make_shared<DataAbort>(
            vaddr, te->domain, is_write,
            ArmFault::PermissionLL + te->lookupLevel,
            state.isStage2, tranMethod);
    }

    // Generate an alignment fault for unaligned data accesses to device or
    // strongly ordered memory
    if (!is_fetch) {
        if (te->mtype != TlbEntry::MemoryType::Normal) {
            if (vaddr & mask(flags & AlignmentMask)) {
                stats.alignFaults++;
                return std::make_shared<DataAbort>(
                    vaddr, TlbEntry::DomainType::NoAccess, is_write,
                    ArmFault::AlignmentFault, state.isStage2,
                    tranMethod);
            }
        }
    }

    if (te->nonCacheable) {
        // Prevent prefetching from I/O devices.
        if (req->isPrefetch()) {
            // Here we can safely use the fault status for the short
            // desc. format in all cases
            return std::make_shared<PrefetchAbort>(
                vaddr, ArmFault::PrefetchUncacheable,
                state.isStage2, tranMethod);
        }
    }

    if (!te->longDescFormat) {
        switch ((state.dacr >> (static_cast<uint8_t>(te->domain) * 2)) & 0x3) {
          case 0:
            stats.domainFaults++;
            DPRINTF(TLB, "TLB Fault: Data abort on domain. DACR: %#x"
                    " domain: %#x write:%d\n", state.dacr,
                    static_cast<uint8_t>(te->domain), is_write);
            if (is_fetch) {
                // Use PC value instead of vaddr because vaddr might
                // be aligned to cache line and should not be the
                // address reported in FAR
                return std::make_shared<PrefetchAbort>(
                    req->getPC(),
                    ArmFault::DomainLL + te->lookupLevel,
                    state.isStage2, tranMethod);
            } else
                return std::make_shared<DataAbort>(
                    vaddr, te->domain, is_write,
                    ArmFault::DomainLL + te->lookupLevel,
                    state.isStage2, tranMethod);
          case 1:
            // Continue with permissions check
            break;
          case 2:
            panic("UNPRED domain\n");
          case 3:
            return NoFault;
        }
    }

    // The 'ap' variable is AP[2:0] or {AP[2,1],1b'0}, i.e. always three bits
    uint8_t ap  = te->longDescFormat ? te->ap << 1 : te->ap;
    uint8_t hap = te->hap;

    if (state.sctlr.afe == 1 || te->longDescFormat)
        ap |= 1;

    bool abt;
    bool isWritable = true;
    // If this is a stage 2 access (eg for reading stage 1 page table entries)
    // then don't perform the AP permissions check, we stil do the HAP check
    // below.
    if (state.isStage2) {
        abt = false;
    } else {
        switch (ap) {
          case 0:
            DPRINTF(TLB, "Access permissions 0, checking rs:%#x\n",
                    (int)state.sctlr.rs);
            if (!state.sctlr.xp) {
                switch ((int)state.sctlr.rs) {
                  case 2:
                    abt = is_write;
                    break;
                  case 1:
                    abt = is_write || !is_priv;
                    break;
                  case 0:
                  case 3:
                  default:
                    abt = true;
                    break;
                }
            } else {
                abt = true;
            }
            break;
          case 1:
            abt = !is_priv;
            break;
          case 2:
            abt = !is_priv && is_write;
            isWritable = is_priv;
            break;
          case 3:
            abt = false;
            break;
          case 4:
            panic("UNPRED premissions\n");
          case 5:
            abt = !is_priv || is_write;
            isWritable = false;
            break;
          case 6:
          case 7:
            abt        = is_write;
            isWritable = false;
            break;
          default:
            panic("Unknown permissions %#x\n", ap);
        }
    }

    bool hapAbt = is_write ? !(hap & 2) : !(hap & 1);
    bool xn     = te->xn || (isWritable && state.sctlr.wxn) ||
                            (ap == 3    && state.sctlr.uwxn && is_priv);
    if (is_fetch && (abt || xn ||
                     (te->longDescFormat && te->pxn && is_priv) ||
                     (state.isSecure && te->ns && state.scr.sif))) {
        stats.permsFaults++;
        DPRINTF(TLB, "TLB Fault: Prefetch abort on permission check. AP:%d "
                     "priv:%d write:%d ns:%d sif:%d sctlr.afe: %d \n",
                     ap, is_priv, is_write, te->ns,
                     state.scr.sif, state.sctlr.afe);
        // Use PC value instead of vaddr because vaddr might be aligned to
        // cache line and should not be the address reported in FAR
        return std::make_shared<PrefetchAbort>(
            req->getPC(),
            ArmFault::PermissionLL + te->lookupLevel,
            state.isStage2, tranMethod);
    } else if (abt | hapAbt) {
        stats.permsFaults++;
        DPRINTF(TLB, "TLB Fault: Data abort on permission check. AP:%d priv:%d"
               " write:%d\n", ap, is_priv, is_write);
        return std::make_shared<DataAbort>(
            vaddr, te->domain, is_write,
            ArmFault::PermissionLL + te->lookupLevel,
            state.isStage2 | !abt, tranMethod);
    }
    return NoFault;
}

Fault
MMU::checkPermissions64(TlbEntry *te, const RequestPtr &req, Mode mode,
                        ThreadContext *tc, bool stage2)
{
    return checkPermissions64(te, req, mode, tc, stage2 ? s2State : s1State);
}

Fault
MMU::checkPermissions64(TlbEntry *te, const RequestPtr &req, Mode mode,
                        ThreadContext *tc, CachedState &state)
{
    assert(state.aarch64);

    // A data cache maintenance instruction that operates by VA does
    // not generate a Permission fault unless:
    // * It is a data cache invalidate (dc ivac) which requires write
    //   permissions to the VA, or
    // * It is executed from EL0
    if (req->isCacheClean() && state.aarch64EL != EL0 && !state.isStage2) {
        return NoFault;
    }

    Addr vaddr_tainted = req->getVaddr();
    Addr vaddr = purifyTaggedAddr(vaddr_tainted, tc, state.aarch64EL,
        static_cast<TCR>(state.ttbcr), mode==Execute, state);

    Request::Flags flags = req->getFlags();
    bool is_fetch  = (mode == Execute);
    // Cache clean operations require read permissions to the specified VA
    bool is_write = !req->isCacheClean() && mode == Write;
    bool is_atomic = req->isAtomic();

    updateMiscReg(tc, state.curTranType, state.isStage2);

    // If this is the second stage of translation and the request is for a
    // stage 1 page table walk then we need to check the HCR.PTW bit. This
    // allows us to generate a fault if the request targets an area marked
    // as a device or strongly ordered.
    if (state.isStage2 && req->isPTWalk() && state.hcr.ptw &&
        (te->mtype != TlbEntry::MemoryType::Normal)) {
        return std::make_shared<DataAbort>(
            vaddr_tainted, te->domain, is_write,
            ArmFault::PermissionLL + te->lookupLevel,
            state.isStage2, ArmFault::LpaeTran);
    }

    // Generate an alignment fault for unaligned accesses to device or
    // strongly ordered memory
    if (!is_fetch) {
        if (te->mtype != TlbEntry::MemoryType::Normal) {
            if (vaddr & mask(flags & AlignmentMask)) {
                stats.alignFaults++;
                return std::make_shared<DataAbort>(
                    vaddr_tainted,
                    TlbEntry::DomainType::NoAccess,
                    is_atomic ? false : is_write,
                    ArmFault::AlignmentFault, state.isStage2,
                    ArmFault::LpaeTran);
            }
        }
    }

    if (te->nonCacheable) {
        // Prevent prefetching from I/O devices.
        if (req->isPrefetch()) {
            // Here we can safely use the fault status for the short
            // desc. format in all cases
            return std::make_shared<PrefetchAbort>(
                vaddr_tainted,
                ArmFault::PrefetchUncacheable,
                state.isStage2, ArmFault::LpaeTran);
        }
    }

    bool grant = false;
    // grant_read is used for faults from an atomic instruction that
    // both reads and writes from a memory location. From a ISS point
    // of view they count as read if a read to that address would have
    // generated the fault; they count as writes otherwise
    bool grant_read = true;

    if (state.isStage2) {
        std::tie(grant, grant_read) = s2PermBits64(te, req, mode, tc, state,
            (!is_write && !is_fetch), is_write, is_fetch);
    } else {
        std::tie(grant, grant_read) = s1PermBits64(te, req, mode, tc, state,
            (!is_write && !is_fetch), is_write, is_fetch);
    }

    if (!grant) {
        if (is_fetch) {
            stats.permsFaults++;
            DPRINTF(TLB, "TLB Fault: Prefetch abort on permission check. "
                    "ns:%d scr.sif:%d sctlr.afe: %d\n",
                    te->ns, state.scr.sif, state.sctlr.afe);
            // Use PC value instead of vaddr because vaddr might be aligned to
            // cache line and should not be the address reported in FAR
            return std::make_shared<PrefetchAbort>(
                req->getPC(),
                ArmFault::PermissionLL + te->lookupLevel,
                state.isStage2, ArmFault::LpaeTran);
        } else {
            stats.permsFaults++;
            DPRINTF(TLB, "TLB Fault: Data abort on permission check."
                    "ns:%d", te->ns);
            return std::make_shared<DataAbort>(
                vaddr_tainted, te->domain,
                (is_atomic && !grant_read) ? false : is_write,
                ArmFault::PermissionLL + te->lookupLevel,
                state.isStage2, ArmFault::LpaeTran);
        }
    }

    return NoFault;
}

std::pair<bool, bool>
MMU::s2PermBits64(TlbEntry *te, const RequestPtr &req, Mode mode,
                  ThreadContext *tc, CachedState &state, bool r, bool w, bool x)
{
    assert(ArmSystem::haveEL(tc, EL2) && state.aarch64EL != EL2);

    // In stage 2 we use the hypervisor access permission bits.
    // The following permissions are described in ARM DDI 0487A.f
    // D4-1802
    bool grant = false;
    bool grant_read = te->hap & 0b01;
    bool grant_write = te->hap & 0b10;

    uint8_t xn =  te->xn;
    uint8_t pxn = te->pxn;

    if (ArmSystem::haveEL(tc, EL3) && state.isSecure &&
        te->ns && state.scr.sif) {
        xn = true;
    }

    DPRINTF(TLBVerbose,
            "Checking S2 permissions: hap:%d, xn:%d, pxn:%d, r:%d, "
            "w:%d, x:%d\n", te->hap, xn, pxn, r, w, x);

    if (x) {
        grant = grant_read && !xn;
    } else if (req->isAtomic()) {
        grant = grant_read || grant_write;
    } else if (w) {
        grant = grant_write;
    } else if (r) {
        grant = grant_read;
    } else {
        panic("Invalid Operation\n");
    }

    return std::make_pair(grant, grant_read);
}

std::pair<bool, bool>
MMU::s1PermBits64(TlbEntry *te, const RequestPtr &req, Mode mode,
                  ThreadContext *tc, CachedState &state, bool r, bool w, bool x)
{
    bool grant = false, grant_read = true, grant_write = true, grant_exec = true;

    const uint8_t ap  = te->ap & 0b11;  // 2-bit access protection field
    const bool is_priv = state.isPriv && !(req->getFlags() & UserMode);

    bool wxn = state.sctlr.wxn;
    uint8_t xn =  te->xn;
    uint8_t pxn = te->pxn;

    DPRINTF(TLBVerbose, "Checking S1 permissions: ap:%d, xn:%d, pxn:%d, r:%d, "
                        "w:%d, x:%d, is_priv: %d, wxn: %d\n", ap, xn,
                        pxn, r, w, x, is_priv, wxn);

    if (faultPAN(tc, ap, req, mode, is_priv, state)) {
        return std::make_pair(false, false);
    }

    ExceptionLevel regime = !is_priv ? EL0 : state.aarch64EL;
    if (hasUnprivRegime(regime, state)) {
        bool pr = false;
        bool pw = false;
        bool ur = false;
        bool uw = false;
        // Apply leaf permissions
        switch (ap) {
          case 0b00: // Privileged access
            pr = 1; pw = 1; ur = 0; uw = 0;
            break;
          case 0b01: // No effect
            pr = 1; pw = 1; ur = 1; uw = 1;
            break;
          case 0b10: // Read-only, privileged access
            pr = 1; pw = 0; ur = 0; uw = 0;
            break;
          case 0b11: // Read-only
            pr = 1; pw = 0; ur = 1; uw = 0;
            break;
        }

        // Locations writable by unprivileged cannot be executed by privileged
        const bool px = !(pxn || uw);
        const bool ux = !xn;

        grant_read = is_priv ? pr : ur;
        grant_write = is_priv ? pw : uw;
        grant_exec = is_priv ? px : ux;
    } else {
        switch (bits(ap, 1)) {
          case 0b0: // No effect
            grant_read = 1; grant_write = 1;
            break;
          case 0b1: // Read-Only
            grant_read = 1; grant_write = 0;
            break;
        }
        grant_exec = !xn;
    }

    // Do not allow execution from writable location
    // if wxn is set
    grant_exec = grant_exec && !(wxn && grant_write);

    if (ArmSystem::haveEL(tc, EL3) && state.isSecure && te->ns) {
        grant_exec = grant_exec && !state.scr.sif;
    }

    if (x) {
        grant = grant_exec;
    } else if (req->isAtomic()) {
        grant = grant_read && grant_write;
    } else if (w) {
        grant = grant_write;
    } else {
        grant = grant_read;
    }

    return std::make_pair(grant, grant_read);
}

bool
MMU::hasUnprivRegime(ExceptionLevel el, bool e2h)
{
    switch (el) {
      case EL0:
      case EL1:
        // EL1&0
        return true;
      case EL2:
        // EL2&0 or EL2
        return e2h;
      case EL3:
      default:
        return false;
    }
}

bool
MMU::hasUnprivRegime(ExceptionLevel el, CachedState &state)
{
    return hasUnprivRegime(el, state.hcr.e2h);
}

bool
MMU::faultPAN(ThreadContext *tc, uint8_t ap, const RequestPtr &req, Mode mode,
              const bool is_priv, CachedState &state)
{
    bool exception = false;
    switch (state.aarch64EL) {
      case EL0:
        break;
      case EL1:
        if (checkPAN(tc, ap, req, mode, is_priv, state)) {
            exception = true;;
        }
        break;
      case EL2:
        if (state.hcr.e2h && checkPAN(tc, ap, req, mode, is_priv, state)) {
            exception = true;;
        }
        break;
      case EL3:
        break;
    }

    return exception;
}

bool
MMU::checkPAN(ThreadContext *tc, uint8_t ap, const RequestPtr &req, Mode mode,
              const bool is_priv, CachedState &state)
{
    // The PAN bit has no effect on:
    // 1) Instruction accesses.
    // 2) Data Cache instructions other than DC ZVA
    // 3) Address translation instructions, other than ATS1E1RP and
    // ATS1E1WP when ARMv8.2-ATS1E1 is implemented. (Unimplemented in
    // gem5)
    // 4) Instructions to be treated as unprivileged, unless
    // HCR_EL2.{E2H, TGE} == {1, 0}
    if (HaveExt(tc, ArmExtension::FEAT_PAN) && state.cpsr.pan && (ap & 0x1) &&
        mode != BaseMMU::Execute) {

        if (req->isCacheMaintenance() &&
            !(req->getFlags() & Request::CACHE_BLOCK_ZERO)) {
            // Cache maintenance other than DC ZVA
            return false;
        } else if (!is_priv && !(state.hcr.e2h && !state.hcr.tge)) {
            // Treated as unprivileged unless HCR_EL2.{E2H, TGE} == {1, 0}
            return false;
        }
        return true;
    }

    return false;
}

Addr
MMU::purifyTaggedAddr(Addr vaddr_tainted, ThreadContext *tc, ExceptionLevel el,
                      TCR tcr, bool is_inst, CachedState& state)
{
    const bool selbit = bits(vaddr_tainted, 55);

    // Call the memoized version of computeAddrTop
    const auto topbit = state.computeAddrTop(tc, selbit, is_inst, tcr, el);

    return maskTaggedAddr(vaddr_tainted, tc, el, topbit);
}

Fault
MMU::translateMmuOff(ThreadContext *tc, const RequestPtr &req, Mode mode,
        ArmTranslationType tran_type, Addr vaddr, bool long_desc_format,
        CachedState &state)
{
    bool is_fetch  = (mode == Execute);
    bool is_atomic = req->isAtomic();
    req->setPaddr(vaddr);
    // When the MMU is off the security attribute corresponds to the
    // security state of the processor
    if (state.isSecure)
        req->setFlags(Request::SECURE);

    if (state.aarch64) {
        bool selbit = bits(vaddr, 55);
        TCR tcr1 = tc->readMiscReg(MISCREG_TCR_EL1);
        int topbit = computeAddrTop(tc, selbit, is_fetch, tcr1, currEL(tc));
        int addr_sz = bits(vaddr, topbit, physAddrRange);
        if (addr_sz != 0){
            Fault f;
            if (is_fetch)
                f = std::make_shared<PrefetchAbort>(vaddr,
                    ArmFault::AddressSizeLL, state.isStage2,
                    ArmFault::LpaeTran);
            else
                f = std::make_shared<DataAbort>( vaddr,
                    TlbEntry::DomainType::NoAccess,
                    is_atomic ? false : mode==Write,
                    ArmFault::AddressSizeLL, state.isStage2,
                    ArmFault::LpaeTran);
            return f;
        }
    }

    // @todo: double check this (ARM ARM issue C B3.2.1)
    if (long_desc_format || state.sctlr.tre == 0 || state.nmrr.ir0 == 0 ||
        state.nmrr.or0 == 0 || state.prrr.tr0 != 0x2) {
        if (!req->isCacheMaintenance()) {
            req->setFlags(Request::UNCACHEABLE);
        }
        req->setFlags(Request::STRICT_ORDER);
    }

    // Set memory attributes
    TlbEntry temp_te;
    temp_te.ns = !state.isSecure;
    bool dc = (HaveExt(tc, ArmExtension::FEAT_VHE) &&
               state.hcr.e2h == 1 && state.hcr.tge == 1) ? 0: state.hcr.dc;
    bool i_cacheability = state.sctlr.i && !state.sctlr.m;
    if (state.isStage2 || !dc || state.aarch64EL == EL2) {
        temp_te.mtype      = is_fetch ? TlbEntry::MemoryType::Normal
                                      : TlbEntry::MemoryType::StronglyOrdered;
        temp_te.innerAttrs = i_cacheability? 0x2: 0x0;
        temp_te.outerAttrs = i_cacheability? 0x2: 0x0;
        temp_te.shareable  = true;
        temp_te.outerShareable = true;
    } else {
        temp_te.mtype      = TlbEntry::MemoryType::Normal;
        temp_te.innerAttrs = 0x3;
        temp_te.outerAttrs = 0x3;
        temp_te.shareable  = false;
        temp_te.outerShareable = false;
    }
    temp_te.setAttributes(long_desc_format);
    DPRINTF(TLBVerbose, "(No MMU) setting memory attributes: shareable: "
            "%d, innerAttrs: %d, outerAttrs: %d, stage2: %d\n",
            temp_te.shareable, temp_te.innerAttrs, temp_te.outerAttrs,
            state.isStage2);
    setAttr(temp_te.attributes);

    return testTranslation(req, mode, TlbEntry::DomainType::NoAccess, state);
}

Fault
MMU::translateMmuOn(ThreadContext* tc, const RequestPtr &req, Mode mode,
                    Translation *translation, bool &delay, bool timing,
                    bool functional, Addr vaddr,
                    ArmFault::TranMethod tranMethod, CachedState &state)
{
    TlbEntry *te = NULL;
    bool is_fetch  = (mode == Execute);
    TlbEntry mergeTe;

    Request::Flags flags = req->getFlags();
    Addr vaddr_tainted = req->getVaddr();

    Fault fault = getResultTe(&te, req, tc, mode, translation, timing,
                              functional, &mergeTe, state);
    // only proceed if we have a valid table entry
    if (!isCompleteTranslation(te) && (fault == NoFault)) delay = true;

    // If we have the table entry transfer some of the attributes to the
    // request that triggered the translation
    if (isCompleteTranslation(te)) {
        // Set memory attributes
        DPRINTF(TLBVerbose,
                "Setting memory attributes: shareable: %d, innerAttrs: %d, "
                "outerAttrs: %d, mtype: %d, stage2: %d\n",
                te->shareable, te->innerAttrs, te->outerAttrs,
                static_cast<uint8_t>(te->mtype), state.isStage2);
        setAttr(te->attributes);

        if (te->nonCacheable && !req->isCacheMaintenance())
            req->setFlags(Request::UNCACHEABLE);

        // Require requests to be ordered if the request goes to
        // strongly ordered or device memory (i.e., anything other
        // than normal memory requires strict order).
        if (te->mtype != TlbEntry::MemoryType::Normal)
            req->setFlags(Request::STRICT_ORDER);

        Addr pa = te->pAddr(vaddr);
        req->setPaddr(pa);

        if (state.isSecure && !te->ns) {
            req->setFlags(Request::SECURE);
        }
        if (!is_fetch && fault == NoFault &&
            (vaddr & mask(flags & AlignmentMask)) &&
            (te->mtype != TlbEntry::MemoryType::Normal)) {
                // Unaligned accesses to Device memory should always cause an
                // abort regardless of sctlr.a
                stats.alignFaults++;
                bool is_write  = (mode == Write);
                return std::make_shared<DataAbort>(
                    vaddr_tainted,
                    TlbEntry::DomainType::NoAccess, is_write,
                    ArmFault::AlignmentFault, state.isStage2,
                    tranMethod);
        }

        // Check for a trickbox generated address fault
        if (fault == NoFault)
            fault = testTranslation(req, mode, te->domain, state);
    }

    if (fault == NoFault) {
        // Don't try to finalize a physical address unless the
        // translation has completed (i.e., there is a table entry).
        return te ? finalizePhysical(req, tc, mode) : NoFault;
    } else {
        return fault;
    }
}

Fault
MMU::translateFs(const RequestPtr &req, ThreadContext *tc, Mode mode,
        Translation *translation, bool &delay, bool timing,
        ArmTranslationType tran_type, bool functional,
        CachedState &state)
{
    // No such thing as a functional timing access
    assert(!(timing && functional));

    Addr vaddr_tainted = req->getVaddr();
    Addr vaddr = 0;
    if (state.aarch64) {
        vaddr = purifyTaggedAddr(vaddr_tainted, tc, state.aarch64EL,
            static_cast<TCR>(state.ttbcr), mode==Execute, state);
    } else {
        vaddr = vaddr_tainted;
    }
    Request::Flags flags = req->getFlags();

    bool is_fetch  = (mode == Execute);
    bool is_write  = (mode == Write);
    bool long_desc_format = state.aarch64 || longDescFormatInUse(tc);
    ArmFault::TranMethod tranMethod = long_desc_format ? ArmFault::LpaeTran
                                                       : ArmFault::VmsaTran;

    DPRINTF(TLBVerbose,
            "CPSR is priv:%d UserMode:%d secure:%d S1S2NsTran:%d\n",
            state.isPriv, flags & UserMode, state.isSecure,
            tran_type & S1S2NsTran);

    DPRINTF(TLB, "translateFs addr %#x, mode %d, st2 %d, scr %#x sctlr %#x "
                 "flags %#lx tranType 0x%x\n", vaddr_tainted, mode,
                 state.isStage2, state.scr, state.sctlr, flags, tran_type);

    if (!state.isStage2) {
        if ((req->isInstFetch() && (!state.sctlr.i)) ||
            ((!req->isInstFetch()) && (!state.sctlr.c))){
            if (!req->isCacheMaintenance()) {
                req->setFlags(Request::UNCACHEABLE);
            }
            req->setFlags(Request::STRICT_ORDER);
        }
    }
    if (!is_fetch) {
        if (state.sctlr.a || !(flags & AllowUnaligned)) {
            if (vaddr & mask(flags & AlignmentMask)) {
                stats.alignFaults++;
                return std::make_shared<DataAbort>(
                    vaddr_tainted,
                    TlbEntry::DomainType::NoAccess, is_write,
                    ArmFault::AlignmentFault, state.isStage2,
                    tranMethod);
            }
        }
    }

    bool vm = state.hcr.vm;
    if (HaveExt(tc, ArmExtension::FEAT_VHE) &&
        state.hcr.e2h == 1 && state.hcr.tge == 1)
        vm = 0;
    else if (state.hcr.dc == 1)
        vm = 1;

    Fault fault = NoFault;
    // If guest MMU is off or hcr.vm=0 go straight to stage2
    if ((state.isStage2 && !vm) || (!state.isStage2 && !state.sctlr.m)) {
        fault = translateMmuOff(tc, req, mode, tran_type, vaddr,
                                long_desc_format, state);
    } else {
        DPRINTF(TLBVerbose, "Translating %s=%#x context=%d\n",
                state.isStage2 ? "IPA" : "VA", vaddr_tainted, state.asid);
        // Translation enabled
        fault = translateMmuOn(tc, req, mode, translation, delay, timing,
                               functional, vaddr, tranMethod, state);
    }

    // Check for Debug Exceptions
    SelfDebug *sd = ArmISA::ISA::getSelfDebug(tc);

    if (sd->enabled() && fault == NoFault) {
        fault = sd->testDebug(tc, req, mode);
    }

    return fault;
}

Fault
MMU::translateAtomic(const RequestPtr &req, ThreadContext *tc, Mode mode,
    ArmTranslationType tran_type)
{
    return translateAtomic(req, tc, mode, tran_type, false);
}

Fault
MMU::translateAtomic(const RequestPtr &req, ThreadContext *tc, Mode mode,
    ArmTranslationType tran_type, bool stage2)
{
    auto& state = updateMiscReg(tc, tran_type, stage2);

    bool delay = false;
    Fault fault;
    if (FullSystem)
        fault = translateFs(req, tc, mode, NULL, delay, false,
            tran_type, false, state);
    else
        fault = translateSe(req, tc, mode, NULL, delay, false, state);
    assert(!delay);
    return fault;
}

Fault
MMU::translateFunctional(const RequestPtr &req, ThreadContext *tc, Mode mode)
{
    return translateFunctional(req, tc, mode, NormalTran, false);
}

Fault
MMU::translateFunctional(const RequestPtr &req, ThreadContext *tc, Mode mode,
    ArmTranslationType tran_type)
{
    return translateFunctional(req, tc, mode, tran_type, false);
}

Fault
MMU::translateFunctional(const RequestPtr &req, ThreadContext *tc, Mode mode,
    ArmTranslationType tran_type, bool stage2)
{
    auto& state = updateMiscReg(tc, tran_type, stage2);

    bool delay = false;
    Fault fault;
    if (FullSystem)
        fault = translateFs(req, tc, mode, NULL, delay, false,
            tran_type, true, state);
   else
        fault = translateSe(req, tc, mode, NULL, delay, false, state);
    assert(!delay);
    return fault;
}

void
MMU::translateTiming(const RequestPtr &req, ThreadContext *tc,
    Translation *translation, Mode mode, ArmTranslationType tran_type,
    bool stage2)
{
    auto& state = updateMiscReg(tc, tran_type, stage2);

    assert(translation);

    translateComplete(req, tc, translation, mode, tran_type,
        stage2, state);
}

Fault
MMU::translateComplete(const RequestPtr &req, ThreadContext *tc,
        Translation *translation, Mode mode, ArmTranslationType tran_type,
        bool call_from_s2)
{
    return translateComplete(req, tc, translation, mode, tran_type,
        call_from_s2, s1State);
}

Fault
MMU::translateComplete(const RequestPtr &req, ThreadContext *tc,
        Translation *translation, Mode mode, ArmTranslationType tran_type,
        bool call_from_s2, CachedState &state)
{
    bool delay = false;
    Fault fault;
    if (FullSystem)
        fault = translateFs(req, tc, mode, translation, delay, true, tran_type,
            false, state);
    else
        fault = translateSe(req, tc, mode, translation, delay, true, state);

    DPRINTF(TLBVerbose, "Translation returning delay=%d fault=%d\n", delay,
            fault != NoFault);
    // If we have a translation, and we're not in the middle of doing a stage
    // 2 translation tell the translation that we've either finished or its
    // going to take a while. By not doing this when we're in the middle of a
    // stage 2 translation we prevent marking the translation as delayed twice,
    // one when the translation starts and again when the stage 1 translation
    // completes.

    if (translation && (call_from_s2 || !state.stage2Req || req->hasPaddr() ||
        fault != NoFault)) {
        if (!delay)
            translation->finish(fault, req, tc, mode);
        else
            translation->markDelayed();
    }
    return fault;
}

vmid_t
MMU::CachedState::getVMID(ThreadContext *tc) const
{
    AA64MMFR1 mmfr1 = tc->readMiscReg(MISCREG_ID_AA64MMFR1_EL1);
    VTCR_t vtcr = tc->readMiscReg(MISCREG_VTCR_EL2);
    vmid_t vmid = 0;

    switch (mmfr1.vmidbits) {
      case 0b0000:
        // 8 bits
        vmid = bits(tc->readMiscReg(MISCREG_VTTBR_EL2), 55, 48);
        break;
      case 0b0010:
        if (vtcr.vs && ELIs64(tc, EL2)) {
            // 16 bits
            vmid = bits(tc->readMiscReg(MISCREG_VTTBR_EL2), 63, 48);
        } else {
            // 8 bits
            vmid = bits(tc->readMiscReg(MISCREG_VTTBR_EL2), 55, 48);
        }
        break;
      default:
        panic("Reserved ID_AA64MMFR1_EL1.VMIDBits value: %#x",
              mmfr1.vmidbits);
    }

    return vmid;
}

MMU::CachedState&
MMU::updateMiscReg(ThreadContext *tc,
    ArmTranslationType tran_type, bool stage2)
{
    // check if the regs have changed, or the translation mode is different.
    // NOTE: the tran type doesn't affect stage 2 TLB's as they only handle
    // one type of translation anyway

    auto& state = stage2 ? s2State : s1State;
    if (state.miscRegValid && miscRegContext == tc->contextId() &&
        ((tran_type == state.curTranType) || stage2)) {

    } else {
        DPRINTF(TLBVerbose, "TLB variables changed!\n");
        state.updateMiscReg(tc, tran_type);

        itbStage2->setVMID(state.vmid);
        dtbStage2->setVMID(state.vmid);

        for (auto tlb : instruction) {
            static_cast<TLB*>(tlb)->setVMID(state.vmid);
        }
        for (auto tlb : data) {
            static_cast<TLB*>(tlb)->setVMID(state.vmid);
        }
        for (auto tlb : unified) {
            static_cast<TLB*>(tlb)->setVMID(state.vmid);
        }

        miscRegContext = tc->contextId();
    }

    if (state.directToStage2) {
        s2State.updateMiscReg(tc, tran_type);
        return s2State;
    } else {
        return state;
    }
}

void
MMU::CachedState::updateMiscReg(ThreadContext *tc,
    ArmTranslationType tran_type)
{
    cpsr = tc->readMiscReg(MISCREG_CPSR);
    hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    scr = tc->readMiscReg(MISCREG_SCR_EL3);

    // Dependencies: SCR/SCR_EL3, CPSR
    isSecure = ArmISA::isSecure(tc) &&
        !(tran_type & HypMode) && !(tran_type & S1S2NsTran);

    aarch64EL = tranTypeEL(cpsr, scr, tran_type);
    aarch64 = isStage2 ?
        ELIs64(tc, EL2) :
        ELIs64(tc, aarch64EL == EL0 ? EL1 : aarch64EL);

    if (aarch64) {  // AArch64
        // determine EL we need to translate in
        switch (aarch64EL) {
          case EL0:
            if (HaveExt(tc, ArmExtension::FEAT_VHE) &&
                hcr.tge == 1 && hcr.e2h == 1) {
                // VHE code for EL2&0 regime
                sctlr = tc->readMiscReg(MISCREG_SCTLR_EL2);
                ttbcr = tc->readMiscReg(MISCREG_TCR_EL2);
                uint64_t ttbr_asid = ttbcr.a1 ?
                    tc->readMiscReg(MISCREG_TTBR1_EL2) :
                    tc->readMiscReg(MISCREG_TTBR0_EL2);
                asid = bits(ttbr_asid,
                            (mmu->haveLargeAsid64 && ttbcr.as) ? 63 : 55, 48);

            } else {
                sctlr = tc->readMiscReg(MISCREG_SCTLR_EL1);
                ttbcr = tc->readMiscReg(MISCREG_TCR_EL1);
                uint64_t ttbr_asid = ttbcr.a1 ?
                    tc->readMiscReg(MISCREG_TTBR1_EL1) :
                    tc->readMiscReg(MISCREG_TTBR0_EL1);
                asid = bits(ttbr_asid,
                            (mmu->haveLargeAsid64 && ttbcr.as) ? 63 : 55, 48);

            }
            break;
          case EL1:
            {
                sctlr = tc->readMiscReg(MISCREG_SCTLR_EL1);
                ttbcr = tc->readMiscReg(MISCREG_TCR_EL1);
                uint64_t ttbr_asid = ttbcr.a1 ?
                    tc->readMiscReg(MISCREG_TTBR1_EL1) :
                    tc->readMiscReg(MISCREG_TTBR0_EL1);
                asid = bits(ttbr_asid,
                            (mmu->haveLargeAsid64 && ttbcr.as) ? 63 : 55, 48);
            }
            break;
          case EL2:
            sctlr = tc->readMiscReg(MISCREG_SCTLR_EL2);
            ttbcr = tc->readMiscReg(MISCREG_TCR_EL2);
            if (hcr.e2h == 1) {
                // VHE code for EL2&0 regime
                uint64_t ttbr_asid = ttbcr.a1 ?
                    tc->readMiscReg(MISCREG_TTBR1_EL2) :
                    tc->readMiscReg(MISCREG_TTBR0_EL2);
                asid = bits(ttbr_asid,
                            (mmu->haveLargeAsid64 && ttbcr.as) ? 63 : 55, 48);
            } else {
                asid = -1;
            }
            break;
          case EL3:
            sctlr = tc->readMiscReg(MISCREG_SCTLR_EL3);
            ttbcr = tc->readMiscReg(MISCREG_TCR_EL3);
            asid = -1;
            break;
        }

        isPriv = aarch64EL != EL0;
        if (mmu->release()->has(ArmExtension::VIRTUALIZATION)) {
            vmid = getVMID(tc);
            bool vm = hcr.vm;
            if (HaveExt(tc, ArmExtension::FEAT_VHE) &&
                hcr.e2h == 1 && hcr.tge ==1) {
                vm = 0;
            }

            if (hcr.e2h == 1 && (aarch64EL == EL2
                                  || (hcr.tge ==1 && aarch64EL == EL0))) {
                directToStage2 = false;
                stage2Req      = false;
                stage2DescReq  = false;
            } else {
            // Work out if we should skip the first stage of translation and go
            // directly to stage 2. This value is cached so we don't have to
            // compute it for every translation.
                const bool el2_enabled = EL2Enabled(tc);
                stage2Req = isStage2 ||
                    (vm && aarch64EL < EL2 && el2_enabled &&
                        !(tran_type & S1CTran) &&
                        !(tran_type & S1E1Tran)); // <--- FIX THIS HACK
                stage2DescReq = isStage2 ||
                    (vm && aarch64EL < EL2 && el2_enabled);
                directToStage2 = !isStage2 && stage2Req && !sctlr.m;
            }
        } else {
            vmid           = 0;
            directToStage2 = false;
            stage2Req      = false;
            stage2DescReq  = false;
        }
    } else {  // AArch32
        sctlr  = tc->readMiscReg(snsBankedIndex(MISCREG_SCTLR, tc,
                                 !isSecure));
        ttbcr  = tc->readMiscReg(snsBankedIndex(MISCREG_TTBCR, tc,
                                 !isSecure));
        isPriv = cpsr.mode != MODE_USER;
        if (longDescFormatInUse(tc)) {
            uint64_t ttbr_asid = tc->readMiscReg(
                snsBankedIndex(ttbcr.a1 ? MISCREG_TTBR1 :
                                          MISCREG_TTBR0,
                                       tc, !isSecure));
            asid = bits(ttbr_asid, 55, 48);
        } else { // Short-descriptor translation table format in use
            CONTEXTIDR context_id = tc->readMiscReg(snsBankedIndex(
                MISCREG_CONTEXTIDR, tc,!isSecure));
            asid = context_id.asid;
        }
        prrr = tc->readMiscReg(snsBankedIndex(MISCREG_PRRR, tc,
                               !isSecure));
        nmrr = tc->readMiscReg(snsBankedIndex(MISCREG_NMRR, tc,
                               !isSecure));
        dacr = tc->readMiscReg(snsBankedIndex(MISCREG_DACR, tc,
                               !isSecure));

        if (mmu->release()->has(ArmExtension::VIRTUALIZATION)) {
            vmid   = bits(tc->readMiscReg(MISCREG_VTTBR), 55, 48);
            if (aarch64EL == EL2) {
                sctlr = tc->readMiscReg(MISCREG_HSCTLR);
            }
            // Work out if we should skip the first stage of translation and go
            // directly to stage 2. This value is cached so we don't have to
            // compute it for every translation.
            const bool el2_enabled = EL2Enabled(tc);
            stage2Req = isStage2 ||
                (hcr.vm && aarch64EL < EL2 && el2_enabled &&
                    !(tran_type & S1CTran));
            stage2DescReq  = isStage2 ||
                (hcr.vm && aarch64EL < EL2 && el2_enabled);
            directToStage2 = !isStage2 && stage2Req && !sctlr.m;
        } else {
            vmid           = 0;
            stage2Req      = false;
            directToStage2 = false;
            stage2DescReq  = false;
        }
    }
    miscRegValid = true;
    curTranType  = tran_type;
}

ExceptionLevel
MMU::tranTypeEL(CPSR cpsr, SCR scr, ArmTranslationType type)
{
    switch (type) {
      case S1E0Tran:
      case S12E0Tran:
        return EL0;

      case S1E1Tran:
      case S12E1Tran:
      case S1S2NsTran:
        return EL1;

      case S1E2Tran:
      case HypMode:
        return EL2;

      case S1E3Tran:
        return EL3;

      case S1CTran:
        return currEL(cpsr) == EL3 && scr.ns == 0 ?
           EL3 : EL1;

      case NormalTran:
        return currEL(cpsr);

      default:
        panic("Unknown translation mode!\n");
    }
}

Fault
MMU::getTE(TlbEntry **te, const RequestPtr &req, ThreadContext *tc, Mode mode,
        Translation *translation, bool timing, bool functional,
        bool is_secure, ArmTranslationType tran_type,
        bool stage2)
{
    return getTE(te, req, tc, mode, translation, timing, functional,
        is_secure, tran_type, stage2 ? s2State : s1State);
}

TlbEntry*
MMU::lookup(Addr va, uint16_t asid, vmid_t vmid, bool secure,
            bool functional, bool ignore_asn, ExceptionLevel target_el,
            bool in_host, bool stage2, BaseMMU::Mode mode)
{
    TLB *tlb = getTlb(mode, stage2);

    TlbEntry::Lookup lookup_data;

    lookup_data.va = va;
    lookup_data.asn = asid;
    lookup_data.ignoreAsn = ignore_asn;
    lookup_data.vmid = vmid;
    lookup_data.secure = secure;
    lookup_data.functional = functional;
    lookup_data.targetEL = target_el;
    lookup_data.inHost = in_host;
    lookup_data.mode = mode;

    return tlb->multiLookup(lookup_data);
}

Fault
MMU::getTE(TlbEntry **te, const RequestPtr &req, ThreadContext *tc, Mode mode,
        Translation *translation, bool timing, bool functional,
        bool is_secure, ArmTranslationType tran_type,
        CachedState& state)
{
    // In a 2-stage system, the IPA->PA translation can be started via this
    // call so make sure the miscRegs are correct.
    if (state.isStage2) {
        updateMiscReg(tc, tran_type, true);
    }

    Addr vaddr_tainted = req->getVaddr();
    Addr vaddr = 0;
    ExceptionLevel target_el = state.aarch64EL;
    if (state.aarch64) {
        vaddr = purifyTaggedAddr(vaddr_tainted, tc, target_el,
            static_cast<TCR>(state.ttbcr), mode==Execute, state);
    } else {
        vaddr = vaddr_tainted;
    }

    *te = lookup(vaddr, state.asid, state.vmid, is_secure, false,
                 false, target_el, false, state.isStage2, mode);

    if (!isCompleteTranslation(*te)) {
        if (req->isPrefetch()) {
            // if the request is a prefetch don't attempt to fill the TLB or go
            // any further with the memory access (here we can safely use the
            // fault status for the short desc. format in all cases)
           stats.prefetchFaults++;
           return std::make_shared<PrefetchAbort>(
               vaddr_tainted, ArmFault::PrefetchTLBMiss, state.isStage2);
        }

        // start translation table walk, pass variables rather than
        // re-retreaving in table walker for speed
        DPRINTF(TLB,
                "TLB Miss: Starting hardware table walker for %#x(%d:%d)\n",
                vaddr_tainted, state.asid, state.vmid);

        Fault fault;
        fault = getTableWalker(mode, state.isStage2)->walk(
            req, tc, state.asid, state.vmid, mode,
            translation, timing, functional, is_secure,
            tran_type, state.stage2DescReq, *te);

        // for timing mode, return and wait for table walk,
        if (timing || fault != NoFault) {
            return fault;
        }

        *te = lookup(vaddr, state.asid, state.vmid, is_secure,
                     true, false, target_el, false, state.isStage2, mode);
        assert(*te);
    }
    return NoFault;
}

Fault
MMU::getResultTe(TlbEntry **te, const RequestPtr &req,
        ThreadContext *tc, Mode mode,
        Translation *translation, bool timing, bool functional,
        TlbEntry *mergeTe, CachedState &state)
{
    Fault fault;

    if (state.isStage2) {
        // We are already in the stage 2 TLB. Grab the table entry for stage
        // 2 only. We are here because stage 1 translation is disabled.
        TlbEntry *s2_te = nullptr;
        // Get the stage 2 table entry
        fault = getTE(&s2_te, req, tc, mode, translation, timing, functional,
                      state.isSecure, state.curTranType, state);
        // Check permissions of stage 2
        if (isCompleteTranslation(s2_te) && (fault == NoFault)) {
            if (state.aarch64)
                fault = checkPermissions64(s2_te, req, mode, tc, state);
            else
                fault = checkPermissions(s2_te, req, mode, state);
        }
        *te = s2_te;
        return fault;
    }

    TlbEntry *s1_te = nullptr;

    Addr vaddr_tainted = req->getVaddr();

    // Get the stage 1 table entry
    fault = getTE(&s1_te, req, tc, mode, translation, timing, functional,
                  state.isSecure, state.curTranType, state);
    // only proceed if we have a valid table entry
    if (isCompleteTranslation(s1_te) && (fault == NoFault)) {
        // Check stage 1 permissions before checking stage 2
        if (state.aarch64)
            fault = checkPermissions64(s1_te, req, mode, tc, state);
        else
            fault = checkPermissions(s1_te, req, mode, state);
        if (state.stage2Req & (fault == NoFault)) {
            Stage2LookUp *s2_lookup = new Stage2LookUp(this, *s1_te,
                req, translation, mode, timing, functional, state.isSecure,
                state.curTranType);
            fault = s2_lookup->getTe(tc, mergeTe);
            if (s2_lookup->isComplete()) {
                *te = mergeTe;
                // We've finished with the lookup so delete it
                delete s2_lookup;
            } else {
                // The lookup hasn't completed, so we can't delete it now. We
                // get round this by asking the object to self delete when the
                // translation is complete.
                s2_lookup->setSelfDelete();
            }
        } else {
            // This case deals with an S1 hit (or bypass), followed by
            // an S2 hit-but-perms issue
            if (state.isStage2) {
                DPRINTF(TLBVerbose, "s2TLB: reqVa %#x, reqPa %#x, fault %p\n",
                        vaddr_tainted, req->hasPaddr() ? req->getPaddr() : ~0,
                        fault);
                if (fault != NoFault) {
                    auto arm_fault = reinterpret_cast<ArmFault*>(fault.get());
                    arm_fault->annotate(ArmFault::S1PTW, false);
                    arm_fault->annotate(ArmFault::OVA, vaddr_tainted);
                }
            }
            *te = s1_te;
        }
    }
    return fault;
}

bool
MMU::isCompleteTranslation(TlbEntry *entry) const
{
    return entry && !entry->partial;
}

void
MMU::takeOverFrom(BaseMMU *old_mmu)
{
    BaseMMU::takeOverFrom(old_mmu);

    auto *ommu = dynamic_cast<MMU*>(old_mmu);
    assert(ommu);

    _attr = ommu->_attr;

    s1State = ommu->s1State;
    s2State = ommu->s2State;
}

void
MMU::setTestInterface(SimObject *_ti)
{
    if (!_ti) {
        test = nullptr;
    } else {
        TlbTestInterface *ti(dynamic_cast<TlbTestInterface *>(_ti));
        fatal_if(!ti, "%s is not a valid ARM TLB tester\n", _ti->name());
        test = ti;
    }
}

Fault
MMU::testTranslation(const RequestPtr &req, Mode mode,
                     TlbEntry::DomainType domain, CachedState &state)
{
    if (!test || !req->hasSize() || req->getSize() == 0 ||
        req->isCacheMaintenance()) {
        return NoFault;
    } else {
        return test->translationCheck(req, state.isPriv, mode, domain);
    }
}

Fault
MMU::testWalk(Addr pa, Addr size, Addr va, bool is_secure, Mode mode,
              TlbEntry::DomainType domain, LookupLevel lookup_level,
              bool stage2)
{
    return testWalk(pa, size, va, is_secure, mode, domain, lookup_level,
        stage2 ? s2State : s1State);
}

Fault
MMU::testWalk(Addr pa, Addr size, Addr va, bool is_secure, Mode mode,
              TlbEntry::DomainType domain, LookupLevel lookup_level,
              CachedState &state)
{
    if (!test) {
        return NoFault;
    } else {
        return test->walkCheck(pa, size, va, is_secure, state.isPriv, mode,
                               domain, lookup_level);
    }
}

MMU::Stats::Stats(statistics::Group *parent)
  : statistics::Group(parent),
    ADD_STAT(alignFaults, statistics::units::Count::get(),
             "Number of MMU faults due to alignment restrictions"),
    ADD_STAT(prefetchFaults, statistics::units::Count::get(),
             "Number of MMU faults due to prefetch"),
    ADD_STAT(domainFaults, statistics::units::Count::get(),
             "Number of MMU faults due to domain restrictions"),
    ADD_STAT(permsFaults, statistics::units::Count::get(),
             "Number of MMU faults due to permissions restrictions")
{
}

} // namespace gem5
