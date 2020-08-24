/*
 * Copyright (c) 2010-2013, 2016-2019 ARM Limited
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

#include "arch/arm/tlb.hh"

#include <memory>
#include <string>
#include <vector>

#include "arch/arm/faults.hh"
#include "arch/arm/isa.hh"
#include "arch/arm/pagetable.hh"
#include "arch/arm/self_debug.hh"
#include "arch/arm/stage2_lookup.hh"
#include "arch/arm/stage2_mmu.hh"
#include "arch/arm/system.hh"
#include "arch/arm/table_walker.hh"
#include "arch/arm/utility.hh"
#include "base/inifile.hh"
#include "base/str.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/Checkpoint.hh"
#include "debug/TLB.hh"
#include "debug/TLBVerbose.hh"
#include "mem/packet_access.hh"
#include "mem/page_table.hh"
#include "mem/request.hh"
#include "params/ArmTLB.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"
#include "sim/pseudo_inst.hh"

using namespace std;
using namespace ArmISA;

TLB::TLB(const ArmTLBParams *p)
    : BaseTLB(p), table(new TlbEntry[p->size]), size(p->size),
      isStage2(p->is_stage2), stage2Req(false), stage2DescReq(false), _attr(0),
      directToStage2(false), tableWalker(p->walker), stage2Tlb(NULL),
      stage2Mmu(NULL), test(nullptr), stats(this),  rangeMRU(1),
      aarch64(false), aarch64EL(EL0), isPriv(false), isSecure(false),
      isHyp(false), asid(0), vmid(0), hcr(0), dacr(0),
      miscRegValid(false), miscRegContext(0), curTranType(NormalTran)
{
    const ArmSystem *sys = dynamic_cast<const ArmSystem *>(p->sys);

    tableWalker->setTlb(this);

    // Cache system-level properties
    haveLPAE = tableWalker->haveLPAE();
    haveVirtualization = tableWalker->haveVirtualization();
    haveLargeAsid64 = tableWalker->haveLargeAsid64();

    if (sys)
        m5opRange = sys->m5opRange();
}

TLB::~TLB()
{
    delete[] table;
}

void
TLB::init()
{
    if (stage2Mmu && !isStage2)
        stage2Tlb = stage2Mmu->stage2Tlb();
}

void
TLB::setMMU(Stage2MMU *m, RequestorID requestor_id)
{
    stage2Mmu = m;
    tableWalker->setMMU(m, requestor_id);
}

bool
TLB::translateFunctional(ThreadContext *tc, Addr va, Addr &pa)
{
    updateMiscReg(tc);

    if (directToStage2) {
        assert(stage2Tlb);
        return stage2Tlb->translateFunctional(tc, va, pa);
    }

    TlbEntry *e = lookup(va, asid, vmid, isHyp, isSecure, true, false,
                         aarch64 ? aarch64EL : EL1, false);
    if (!e)
        return false;
    pa = e->pAddr(va);
    return true;
}

Fault
TLB::finalizePhysical(const RequestPtr &req,
                      ThreadContext *tc, Mode mode) const
{
    const Addr paddr = req->getPaddr();

    if (m5opRange.contains(paddr)) {
        uint8_t func;
        PseudoInst::decodeAddrOffset(paddr - m5opRange.start(), func);
        req->setLocalAccessor(
            [func, mode](ThreadContext *tc, PacketPtr pkt) -> Cycles
            {
                uint64_t ret;
                PseudoInst::pseudoInst<PseudoInstABI>(tc, func, ret);
                if (mode == Read)
                    pkt->setLE(ret);
                return Cycles(1);
            }
        );
    }

    return NoFault;
}

TlbEntry*
TLB::lookup(Addr va, uint16_t asn, uint8_t vmid, bool hyp, bool secure,
            bool functional, bool ignore_asn, ExceptionLevel target_el,
            bool in_host)
{

    TlbEntry *retval = NULL;

    // Maintaining LRU array
    int x = 0;
    while (retval == NULL && x < size) {
        if ((!ignore_asn && table[x].match(va, asn, vmid, hyp, secure, false,
             target_el, in_host)) ||
            (ignore_asn && table[x].match(va, vmid, hyp, secure, target_el,
             in_host))) {
            // We only move the hit entry ahead when the position is higher
            // than rangeMRU
            if (x > rangeMRU && !functional) {
                TlbEntry tmp_entry = table[x];
                for (int i = x; i > 0; i--)
                    table[i] = table[i - 1];
                table[0] = tmp_entry;
                retval = &table[0];
            } else {
                retval = &table[x];
            }
            break;
        }
        ++x;
    }

    DPRINTF(TLBVerbose, "Lookup %#x, asn %#x -> %s vmn 0x%x hyp %d secure %d "
            "ppn %#x size: %#x pa: %#x ap:%d ns:%d nstid:%d g:%d asid: %d "
            "el: %d\n",
            va, asn, retval ? "hit" : "miss", vmid, hyp, secure,
            retval ? retval->pfn       : 0, retval ? retval->size  : 0,
            retval ? retval->pAddr(va) : 0, retval ? retval->ap    : 0,
            retval ? retval->ns        : 0, retval ? retval->nstid : 0,
            retval ? retval->global    : 0, retval ? retval->asid  : 0,
            retval ? retval->el        : 0);

    return retval;
}

// insert a new TLB entry
void
TLB::insert(Addr addr, TlbEntry &entry)
{
    DPRINTF(TLB, "Inserting entry into TLB with pfn:%#x size:%#x vpn: %#x"
            " asid:%d vmid:%d N:%d global:%d valid:%d nc:%d xn:%d"
            " ap:%#x domain:%#x ns:%d nstid:%d isHyp:%d\n", entry.pfn,
            entry.size, entry.vpn, entry.asid, entry.vmid, entry.N,
            entry.global, entry.valid, entry.nonCacheable, entry.xn,
            entry.ap, static_cast<uint8_t>(entry.domain), entry.ns, entry.nstid,
            entry.isHyp);

    if (table[size - 1].valid)
        DPRINTF(TLB, " - Replacing Valid entry %#x, asn %d vmn %d ppn %#x "
                "size: %#x ap:%d ns:%d nstid:%d g:%d isHyp:%d el: %d\n",
                table[size-1].vpn << table[size-1].N, table[size-1].asid,
                table[size-1].vmid, table[size-1].pfn << table[size-1].N,
                table[size-1].size, table[size-1].ap, table[size-1].ns,
                table[size-1].nstid, table[size-1].global, table[size-1].isHyp,
                table[size-1].el);

    //inserting to MRU position and evicting the LRU one

    for (int i = size - 1; i > 0; --i)
        table[i] = table[i-1];
    table[0] = entry;

    stats.inserts++;
    ppRefills->notify(1);
}

void
TLB::printTlb() const
{
    int x = 0;
    TlbEntry *te;
    DPRINTF(TLB, "Current TLB contents:\n");
    while (x < size) {
        te = &table[x];
        if (te->valid)
            DPRINTF(TLB, " *  %s\n", te->print());
        ++x;
    }
}

void
TLB::flushAllSecurity(bool secure_lookup, ExceptionLevel target_el,
                      bool ignore_el, bool in_host)
{
    DPRINTF(TLB, "Flushing all TLB entries (%s lookup)\n",
            (secure_lookup ? "secure" : "non-secure"));
    int x = 0;
    TlbEntry *te;
    while (x < size) {
        te = &table[x];
        const bool el_match = ignore_el ?
            true : te->checkELMatch(target_el, in_host);
        if (te->valid && secure_lookup == !te->nstid &&
            (te->vmid == vmid || secure_lookup) && el_match) {

            DPRINTF(TLB, " -  %s\n", te->print());
            te->valid = false;
            stats.flushedEntries++;
        }
        ++x;
    }

    stats.flushTlb++;

    // If there's a second stage TLB (and we're not it) then flush it as well
    // if we're currently in hyp mode
    if (!isStage2 && isHyp) {
        stage2Tlb->flushAllSecurity(secure_lookup, EL1, true, false);
    }
}

void
TLB::flushAllNs(ExceptionLevel target_el, bool ignore_el)
{
    bool hyp = target_el == EL2;

    DPRINTF(TLB, "Flushing all NS TLB entries (%s lookup)\n",
            (hyp ? "hyp" : "non-hyp"));
    int x = 0;
    TlbEntry *te;
    while (x < size) {
        te = &table[x];
        const bool el_match = ignore_el ?
            true : te->checkELMatch(target_el, false);

        if (te->valid && te->nstid && te->isHyp == hyp && el_match) {

            DPRINTF(TLB, " -  %s\n", te->print());
            stats.flushedEntries++;
            te->valid = false;
        }
        ++x;
    }

    stats.flushTlb++;

    // If there's a second stage TLB (and we're not it) then flush it as well
    if (!isStage2 && !hyp) {
        stage2Tlb->flushAllNs(EL1, true);
    }
}

void
TLB::flushMvaAsid(Addr mva, uint64_t asn, bool secure_lookup,
                  ExceptionLevel target_el, bool in_host)
{
    DPRINTF(TLB, "Flushing TLB entries with mva: %#x, asid: %#x "
            "(%s lookup)\n", mva, asn, (secure_lookup ?
            "secure" : "non-secure"));
    _flushMva(mva, asn, secure_lookup, false, target_el, in_host);
    stats.flushTlbMvaAsid++;
}

void
TLB::flushAsid(uint64_t asn, bool secure_lookup, ExceptionLevel target_el,
               bool in_host)
{
    DPRINTF(TLB, "Flushing TLB entries with asid: %#x (%s lookup)\n", asn,
            (secure_lookup ? "secure" : "non-secure"));

    int x = 0 ;
    TlbEntry *te;

    while (x < size) {
        te = &table[x];
        if (te->valid && te->asid == asn && secure_lookup == !te->nstid &&
            (te->vmid == vmid || secure_lookup) &&
            te->checkELMatch(target_el, in_host)) {

            te->valid = false;
            DPRINTF(TLB, " -  %s\n", te->print());
            stats.flushedEntries++;
        }
        ++x;
    }
    stats.flushTlbAsid++;
}

void
TLB::flushMva(Addr mva, bool secure_lookup, ExceptionLevel target_el,
              bool in_host) {

    DPRINTF(TLB, "Flushing TLB entries with mva: %#x (%s lookup)\n", mva,
            (secure_lookup ? "secure" : "non-secure"));
    _flushMva(mva, 0xbeef, secure_lookup, true, target_el, in_host);
    stats.flushTlbMva++;
}

void
TLB::_flushMva(Addr mva, uint64_t asn, bool secure_lookup,
               bool ignore_asn, ExceptionLevel target_el, bool in_host)
{
    TlbEntry *te;
    // D5.7.2: Sign-extend address to 64 bits
    mva = sext<56>(mva);

    bool hyp = target_el == EL2;

    te = lookup(mva, asn, vmid, hyp, secure_lookup, false, ignore_asn,
                target_el, in_host);
    while (te != NULL) {
        if (secure_lookup == !te->nstid) {
            DPRINTF(TLB, " -  %s\n", te->print());
            te->valid = false;
            stats.flushedEntries++;
        }
        te = lookup(mva, asn, vmid, hyp, secure_lookup, false, ignore_asn,
                    target_el, in_host);
    }
}

void
TLB::flushIpaVmid(Addr ipa, bool secure_lookup, ExceptionLevel target_el)
{
    assert(!isStage2);
    stage2Tlb->_flushMva(ipa, 0xbeef, secure_lookup, true, target_el, false);
}

void
TLB::drainResume()
{
    // We might have unserialized something or switched CPUs, so make
    // sure to re-read the misc regs.
    miscRegValid = false;
}

void
TLB::takeOverFrom(BaseTLB *_otlb)
{
    TLB *otlb = dynamic_cast<TLB*>(_otlb);
    /* Make sure we actually have a valid type */
    if (otlb) {
        _attr = otlb->_attr;
        haveLPAE = otlb->haveLPAE;
        directToStage2 = otlb->directToStage2;
        stage2Req = otlb->stage2Req;
        stage2DescReq = otlb->stage2DescReq;

        /* Sync the stage2 MMU if they exist in both
         * the old CPU and the new
         */
        if (!isStage2 &&
            stage2Tlb && otlb->stage2Tlb) {
            stage2Tlb->takeOverFrom(otlb->stage2Tlb);
        }
    } else {
        panic("Incompatible TLB type!");
    }
}

TLB::TlbStats::TlbStats(Stats::Group *parent)
  : Stats::Group(parent),
    ADD_STAT(instHits,"ITB inst hits"),
    ADD_STAT(instMisses, "ITB inst misses"),
    ADD_STAT(readHits, "DTB read hits"),
    ADD_STAT(readMisses, "DTB read misses"),
    ADD_STAT(writeHits, "DTB write hits"),
    ADD_STAT(writeMisses, "DTB write misses"),
    ADD_STAT(inserts, "Number of times an entry is inserted into the TLB"),
    ADD_STAT(flushTlb, "Number of times complete TLB was flushed"),
    ADD_STAT(flushTlbMva, "Number of times TLB was flushed by MVA"),
    ADD_STAT(flushTlbMvaAsid, "Number of times TLB was flushed by MVA & ASID"),
    ADD_STAT(flushTlbAsid, "Number of times TLB was flushed by ASID"),
    ADD_STAT(flushedEntries, "Number of entries that have been flushed"
        " from TLB"),
    ADD_STAT(alignFaults, "Number of TLB faults due to alignment"
        " restrictions"),
    ADD_STAT(prefetchFaults, "Number of TLB faults due to prefetch"),
    ADD_STAT(domainFaults, "Number of TLB faults due to domain restrictions"),
    ADD_STAT(permsFaults, "Number of TLB faults due to permissions"
        " restrictions"),
    ADD_STAT(readAccesses, "DTB read accesses", readHits + readMisses),
    ADD_STAT(writeAccesses, "DTB write accesses", writeHits + writeMisses),
    ADD_STAT(instAccesses, "ITB inst accesses", instHits + instMisses),
    ADD_STAT(hits, "Total TLB (inst and data) hits",
        readHits + writeHits + instHits),
    ADD_STAT(misses, "Total TLB (inst and data) misses",
        readMisses + writeMisses + instMisses),
    ADD_STAT(accesses, "Total TLB (inst and data) accesses",
        readAccesses + writeAccesses + instAccesses)
{
}

void
TLB::regProbePoints()
{
    ppRefills.reset(new ProbePoints::PMU(getProbeManager(), "Refills"));
}

Fault
TLB::translateSe(const RequestPtr &req, ThreadContext *tc, Mode mode,
                 Translation *translation, bool &delay, bool timing)
{
    updateMiscReg(tc);
    Addr vaddr_tainted = req->getVaddr();
    Addr vaddr = 0;
    if (aarch64)
        vaddr = purifyTaggedAddr(vaddr_tainted, tc, aarch64EL, (TCR)ttbcr,
                                 mode==Execute);
    else
        vaddr = vaddr_tainted;
    Request::Flags flags = req->getFlags();

    bool is_fetch = (mode == Execute);
    bool is_write = (mode == Write);

    if (!is_fetch) {
        if (sctlr.a || !(flags & AllowUnaligned)) {
            if (vaddr & mask(flags & AlignmentMask)) {
                // LPAE is always disabled in SE mode
                return std::make_shared<DataAbort>(
                    vaddr_tainted,
                    TlbEntry::DomainType::NoAccess, is_write,
                    ArmFault::AlignmentFault, isStage2,
                    ArmFault::VmsaTran);
            }
        }
    }

    Addr paddr;
    Process *p = tc->getProcessPtr();

    if (!p->pTable->translate(vaddr, paddr))
        return std::make_shared<GenericPageTableFault>(vaddr_tainted);
    req->setPaddr(paddr);

    return finalizePhysical(req, tc, mode);
}

Fault
TLB::checkPermissions(TlbEntry *te, const RequestPtr &req, Mode mode)
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
    bool is_priv   = isPriv && !(flags & UserMode);

    // Get the translation type from the actuall table entry
    ArmFault::TranMethod tranMethod = te->longDescFormat ? ArmFault::LpaeTran
                                                         : ArmFault::VmsaTran;

    // If this is the second stage of translation and the request is for a
    // stage 1 page table walk then we need to check the HCR.PTW bit. This
    // allows us to generate a fault if the request targets an area marked
    // as a device or strongly ordered.
    if (isStage2 && req->isPTWalk() && hcr.ptw &&
        (te->mtype != TlbEntry::MemoryType::Normal)) {
        return std::make_shared<DataAbort>(
            vaddr, te->domain, is_write,
            ArmFault::PermissionLL + te->lookupLevel,
            isStage2, tranMethod);
    }

    // Generate an alignment fault for unaligned data accesses to device or
    // strongly ordered memory
    if (!is_fetch) {
        if (te->mtype != TlbEntry::MemoryType::Normal) {
            if (vaddr & mask(flags & AlignmentMask)) {
                stats.alignFaults++;
                return std::make_shared<DataAbort>(
                    vaddr, TlbEntry::DomainType::NoAccess, is_write,
                    ArmFault::AlignmentFault, isStage2,
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
                isStage2, tranMethod);
        }
    }

    if (!te->longDescFormat) {
        switch ((dacr >> (static_cast<uint8_t>(te->domain) * 2)) & 0x3) {
          case 0:
            stats.domainFaults++;
            DPRINTF(TLB, "TLB Fault: Data abort on domain. DACR: %#x"
                    " domain: %#x write:%d\n", dacr,
                    static_cast<uint8_t>(te->domain), is_write);
            if (is_fetch) {
                // Use PC value instead of vaddr because vaddr might
                // be aligned to cache line and should not be the
                // address reported in FAR
                return std::make_shared<PrefetchAbort>(
                    req->getPC(),
                    ArmFault::DomainLL + te->lookupLevel,
                    isStage2, tranMethod);
            } else
                return std::make_shared<DataAbort>(
                    vaddr, te->domain, is_write,
                    ArmFault::DomainLL + te->lookupLevel,
                    isStage2, tranMethod);
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

    if (sctlr.afe == 1 || te->longDescFormat)
        ap |= 1;

    bool abt;
    bool isWritable = true;
    // If this is a stage 2 access (eg for reading stage 1 page table entries)
    // then don't perform the AP permissions check, we stil do the HAP check
    // below.
    if (isStage2) {
        abt = false;
    } else {
        switch (ap) {
          case 0:
            DPRINTF(TLB, "Access permissions 0, checking rs:%#x\n",
                    (int)sctlr.rs);
            if (!sctlr.xp) {
                switch ((int)sctlr.rs) {
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
    bool xn     = te->xn || (isWritable && sctlr.wxn) ||
                            (ap == 3    && sctlr.uwxn && is_priv);
    if (is_fetch && (abt || xn ||
                     (te->longDescFormat && te->pxn && is_priv) ||
                     (isSecure && te->ns && scr.sif))) {
        stats.permsFaults++;
        DPRINTF(TLB, "TLB Fault: Prefetch abort on permission check. AP:%d "
                     "priv:%d write:%d ns:%d sif:%d sctlr.afe: %d \n",
                     ap, is_priv, is_write, te->ns, scr.sif,sctlr.afe);
        // Use PC value instead of vaddr because vaddr might be aligned to
        // cache line and should not be the address reported in FAR
        return std::make_shared<PrefetchAbort>(
            req->getPC(),
            ArmFault::PermissionLL + te->lookupLevel,
            isStage2, tranMethod);
    } else if (abt | hapAbt) {
        stats.permsFaults++;
        DPRINTF(TLB, "TLB Fault: Data abort on permission check. AP:%d priv:%d"
               " write:%d\n", ap, is_priv, is_write);
        return std::make_shared<DataAbort>(
            vaddr, te->domain, is_write,
            ArmFault::PermissionLL + te->lookupLevel,
            isStage2 | !abt, tranMethod);
    }
    return NoFault;
}


Fault
TLB::checkPermissions64(TlbEntry *te, const RequestPtr &req, Mode mode,
                        ThreadContext *tc)
{
    assert(aarch64);

    // A data cache maintenance instruction that operates by VA does
    // not generate a Permission fault unless:
    // * It is a data cache invalidate (dc ivac) which requires write
    //   permissions to the VA, or
    // * It is executed from EL0
    if (req->isCacheClean() && aarch64EL != EL0 && !isStage2) {
        return NoFault;
    }

    Addr vaddr_tainted = req->getVaddr();
    Addr vaddr = purifyTaggedAddr(vaddr_tainted, tc, aarch64EL, (TCR)ttbcr,
                                  mode==Execute);

    Request::Flags flags = req->getFlags();
    bool is_fetch  = (mode == Execute);
    // Cache clean operations require read permissions to the specified VA
    bool is_write = !req->isCacheClean() && mode == Write;
    bool is_atomic = req->isAtomic();
    bool is_priv M5_VAR_USED  = isPriv && !(flags & UserMode);

    updateMiscReg(tc, curTranType);

    // If this is the second stage of translation and the request is for a
    // stage 1 page table walk then we need to check the HCR.PTW bit. This
    // allows us to generate a fault if the request targets an area marked
    // as a device or strongly ordered.
    if (isStage2 && req->isPTWalk() && hcr.ptw &&
        (te->mtype != TlbEntry::MemoryType::Normal)) {
        return std::make_shared<DataAbort>(
            vaddr_tainted, te->domain, is_write,
            ArmFault::PermissionLL + te->lookupLevel,
            isStage2, ArmFault::LpaeTran);
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
                    ArmFault::AlignmentFault, isStage2,
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
                isStage2, ArmFault::LpaeTran);
        }
    }

    uint8_t ap  = 0x3 & (te->ap);  // 2-bit access protection field
    bool grant = false;

    bool wxn = sctlr.wxn;
    uint8_t xn =  te->xn;
    uint8_t pxn = te->pxn;
    bool r = (!is_write && !is_fetch);
    bool w = is_write;
    bool x = is_fetch;

    if (ArmSystem::haveEL(tc, EL3) && isSecure && te->ns && scr.sif)
        xn = true;

    // grant_read is used for faults from an atomic instruction that
    // both reads and writes from a memory location. From a ISS point
    // of view they count as read if a read to that address would have
    // generated the fault; they count as writes otherwise
    bool grant_read = true;
    DPRINTF(TLBVerbose, "Checking permissions: ap:%d, xn:%d, pxn:%d, r:%d, "
                        "w:%d, x:%d, is_priv: %d, wxn: %d\n", ap, xn,
                        pxn, r, w, x, is_priv, wxn);

    if (isStage2) {
        assert(ArmSystem::haveVirtualization(tc) && aarch64EL != EL2);
        // In stage 2 we use the hypervisor access permission bits.
        // The following permissions are described in ARM DDI 0487A.f
        // D4-1802
        uint8_t hap = 0x3 & te->hap;
        grant_read = hap & 0x1;
        if (is_fetch) {
            // sctlr.wxn overrides the xn bit
            grant = !wxn && !xn;
        } else if (is_atomic) {
            grant = r && w;
            grant_read = r;
        } else if (is_write) {
            grant = hap & 0x2;
        } else { // is_read
            grant = grant_read;
        }
    } else {
        switch (aarch64EL) {
          case EL0:
            {
                grant_read = ap & 0x1;
                uint8_t perm = (ap << 2)  | (xn << 1) | pxn;
                switch (perm) {
                  case 0:
                  case 1:
                  case 8:
                  case 9:
                    grant = x;
                    break;
                  case 4:
                  case 5:
                    grant = r || w || (x && !wxn);
                    break;
                  case 6:
                  case 7:
                    grant = r || w;
                    break;
                  case 12:
                  case 13:
                    grant = r || x;
                    break;
                  case 14:
                  case 15:
                    grant = r;
                    break;
                  default:
                    grant = false;
                }
            }
            break;
          case EL1:
            {
                if (checkPAN(tc, ap, req, mode)) {
                    grant = false;
                    grant_read = false;
                    break;
                }

                uint8_t perm = (ap << 2)  | (xn << 1) | pxn;
                switch (perm) {
                  case 0:
                  case 2:
                    grant = r || w || (x && !wxn);
                    break;
                  case 1:
                  case 3:
                  case 4:
                  case 5:
                  case 6:
                  case 7:
                    // regions that are writeable at EL0 should not be
                    // executable at EL1
                    grant = r || w;
                    break;
                  case 8:
                  case 10:
                  case 12:
                  case 14:
                    grant = r || x;
                    break;
                  case 9:
                  case 11:
                  case 13:
                  case 15:
                    grant = r;
                    break;
                  default:
                    grant = false;
                }
            }
            break;
          case EL2:
            if (hcr.e2h && checkPAN(tc, ap, req, mode)) {
                grant = false;
                grant_read = false;
                break;
            }
            M5_FALLTHROUGH;
          case EL3:
            {
                uint8_t perm = (ap & 0x2) | xn;
                switch (perm) {
                  case 0:
                    grant = r || w || (x && !wxn);
                    break;
                  case 1:
                    grant = r || w;
                    break;
                  case 2:
                    grant = r || x;
                    break;
                  case 3:
                    grant = r;
                    break;
                  default:
                    grant = false;
                }
            }
            break;
        }
    }

    if (!grant) {
        if (is_fetch) {
            stats.permsFaults++;
            DPRINTF(TLB, "TLB Fault: Prefetch abort on permission check. "
                    "AP:%d priv:%d write:%d ns:%d sif:%d "
                    "sctlr.afe: %d\n",
                    ap, is_priv, is_write, te->ns, scr.sif, sctlr.afe);
            // Use PC value instead of vaddr because vaddr might be aligned to
            // cache line and should not be the address reported in FAR
            return std::make_shared<PrefetchAbort>(
                req->getPC(),
                ArmFault::PermissionLL + te->lookupLevel,
                isStage2, ArmFault::LpaeTran);
        } else {
            stats.permsFaults++;
            DPRINTF(TLB, "TLB Fault: Data abort on permission check. AP:%d "
                    "priv:%d write:%d\n", ap, is_priv, is_write);
            return std::make_shared<DataAbort>(
                vaddr_tainted, te->domain,
                (is_atomic && !grant_read) ? false : is_write,
                ArmFault::PermissionLL + te->lookupLevel,
                isStage2, ArmFault::LpaeTran);
        }
    }

    return NoFault;
}

bool
TLB::checkPAN(ThreadContext *tc, uint8_t ap, const RequestPtr &req, Mode mode)
{
    // The PAN bit has no effect on:
    // 1) Instruction accesses.
    // 2) Data Cache instructions other than DC ZVA
    // 3) Address translation instructions, other than ATS1E1RP and
    // ATS1E1WP when ARMv8.2-ATS1E1 is implemented. (Unimplemented in
    // gem5)
    // 4) Unprivileged instructions (Unimplemented in gem5)
    AA64MMFR1 mmfr1 = tc->readMiscReg(MISCREG_ID_AA64MMFR1_EL1);
    if (mmfr1.pan && cpsr.pan && (ap & 0x1) && mode != Execute &&
        (!req->isCacheMaintenance() ||
            (req->getFlags() & Request::CACHE_BLOCK_ZERO))) {
        return true;
    } else {
        return false;
    }
}

Fault
TLB::translateMmuOff(ThreadContext *tc, const RequestPtr &req, Mode mode,
        TLB::ArmTranslationType tranType, Addr vaddr, bool long_desc_format)
{
    bool is_fetch  = (mode == Execute);
    bool is_atomic = req->isAtomic();
    req->setPaddr(vaddr);
    // When the MMU is off the security attribute corresponds to the
    // security state of the processor
    if (isSecure)
        req->setFlags(Request::SECURE);

    if (aarch64) {
        bool selbit = bits(vaddr, 55);
        TCR tcr1 = tc->readMiscReg(MISCREG_TCR_EL1);
        int topbit = computeAddrTop(tc, selbit, is_fetch, tcr1, currEL(tc));
        int addr_sz = bits(vaddr, topbit, MaxPhysAddrRange);
        if (addr_sz != 0){
            Fault f;
            if (is_fetch)
                f = std::make_shared<PrefetchAbort>(vaddr,
                    ArmFault::AddressSizeLL, isStage2, ArmFault::LpaeTran);
            else
                f = std::make_shared<DataAbort>( vaddr,
                    TlbEntry::DomainType::NoAccess,
                    is_atomic ? false : mode==Write,
                    ArmFault::AddressSizeLL, isStage2, ArmFault::LpaeTran);
            return f;
        }
    }

    // @todo: double check this (ARM ARM issue C B3.2.1)
    if (long_desc_format || sctlr.tre == 0 || nmrr.ir0 == 0 ||
        nmrr.or0 == 0 || prrr.tr0 != 0x2) {
        if (!req->isCacheMaintenance()) {
            req->setFlags(Request::UNCACHEABLE);
        }
        req->setFlags(Request::STRICT_ORDER);
    }

    // Set memory attributes
    TlbEntry temp_te;
    temp_te.ns = !isSecure;
    bool dc = (HaveVirtHostExt(tc)
               && hcr.e2h == 1 && hcr.tge == 1) ? 0: hcr.dc;
    bool i_cacheability = sctlr.i && !sctlr.m;
    if (isStage2 || !dc || isSecure ||
       (isHyp && !(tranType & S1CTran))) {

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
            "%d, innerAttrs: %d, outerAttrs: %d, isStage2: %d\n",
            temp_te.shareable, temp_te.innerAttrs, temp_te.outerAttrs,
            isStage2);
    setAttr(temp_te.attributes);

    return testTranslation(req, mode, TlbEntry::DomainType::NoAccess);
}

Fault
TLB::translateMmuOn(ThreadContext* tc, const RequestPtr &req, Mode mode,
                    Translation *translation, bool &delay, bool timing,
                    bool functional, Addr vaddr,
                    ArmFault::TranMethod tranMethod)
{
    TlbEntry *te = NULL;
    bool is_fetch  = (mode == Execute);
    TlbEntry mergeTe;

    Request::Flags flags = req->getFlags();
    Addr vaddr_tainted = req->getVaddr();

    Fault fault = getResultTe(&te, req, tc, mode, translation, timing,
                              functional, &mergeTe);
    // only proceed if we have a valid table entry
    if ((te == NULL) && (fault == NoFault)) delay = true;

    // If we have the table entry transfer some of the attributes to the
    // request that triggered the translation
    if (te != NULL) {
        // Set memory attributes
        DPRINTF(TLBVerbose,
                "Setting memory attributes: shareable: %d, innerAttrs: %d, "
                "outerAttrs: %d, mtype: %d, isStage2: %d\n",
                te->shareable, te->innerAttrs, te->outerAttrs,
                static_cast<uint8_t>(te->mtype), isStage2);
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

        if (isSecure && !te->ns) {
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
                    ArmFault::AlignmentFault, isStage2,
                    tranMethod);
        }

        // Check for a trickbox generated address fault
        if (fault == NoFault)
            fault = testTranslation(req, mode, te->domain);
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
TLB::translateFs(const RequestPtr &req, ThreadContext *tc, Mode mode,
        Translation *translation, bool &delay, bool timing,
        TLB::ArmTranslationType tranType, bool functional)
{
    // No such thing as a functional timing access
    assert(!(timing && functional));

    updateMiscReg(tc, tranType);

    Addr vaddr_tainted = req->getVaddr();
    Addr vaddr = 0;
    if (aarch64)
        vaddr = purifyTaggedAddr(vaddr_tainted, tc, aarch64EL, (TCR)ttbcr,
                                 mode==Execute);
    else
        vaddr = vaddr_tainted;
    Request::Flags flags = req->getFlags();

    bool is_fetch  = (mode == Execute);
    bool is_write  = (mode == Write);
    bool long_desc_format = aarch64 || longDescFormatInUse(tc);
    ArmFault::TranMethod tranMethod = long_desc_format ? ArmFault::LpaeTran
                                                       : ArmFault::VmsaTran;

    DPRINTF(TLBVerbose,
            "CPSR is priv:%d UserMode:%d secure:%d S1S2NsTran:%d\n",
            isPriv, flags & UserMode, isSecure, tranType & S1S2NsTran);

    DPRINTF(TLB, "translateFs addr %#x, mode %d, st2 %d, scr %#x sctlr %#x "
                 "flags %#lx tranType 0x%x\n", vaddr_tainted, mode, isStage2,
                 scr, sctlr, flags, tranType);

    if ((req->isInstFetch() && (!sctlr.i)) ||
        ((!req->isInstFetch()) && (!sctlr.c))){
        if (!req->isCacheMaintenance()) {
            req->setFlags(Request::UNCACHEABLE);
        }
        req->setFlags(Request::STRICT_ORDER);
    }
    if (!is_fetch) {
        if (sctlr.a || !(flags & AllowUnaligned)) {
            if (vaddr & mask(flags & AlignmentMask)) {
                stats.alignFaults++;
                return std::make_shared<DataAbort>(
                    vaddr_tainted,
                    TlbEntry::DomainType::NoAccess, is_write,
                    ArmFault::AlignmentFault, isStage2,
                    tranMethod);
            }
        }
    }

    bool vm = hcr.vm;
    if (HaveVirtHostExt(tc) && hcr.e2h == 1 && hcr.tge ==1)
        vm = 0;
    else if (hcr.dc == 1)
        vm = 1;

    Fault fault = NoFault;
    // If guest MMU is off or hcr.vm=0 go straight to stage2
    if ((isStage2 && !vm) || (!isStage2 && !sctlr.m)) {
        fault = translateMmuOff(tc, req, mode, tranType, vaddr,
                                long_desc_format);
    } else {
        DPRINTF(TLBVerbose, "Translating %s=%#x context=%d\n",
                isStage2 ? "IPA" : "VA", vaddr_tainted, asid);
        // Translation enabled
        fault = translateMmuOn(tc, req, mode, translation, delay, timing,
                               functional, vaddr, tranMethod);
    }

    // Check for Debug Exceptions
    SelfDebug *sd = ArmISA::ISA::getSelfDebug(tc);

    if (sd->enabled() && fault == NoFault) {
        fault = sd->testDebug(tc, req, mode);
    }

    return fault;
}

Fault
TLB::translateAtomic(const RequestPtr &req, ThreadContext *tc, Mode mode,
    TLB::ArmTranslationType tranType)
{
    updateMiscReg(tc, tranType);

    if (directToStage2) {
        assert(stage2Tlb);
        return stage2Tlb->translateAtomic(req, tc, mode, tranType);
    }

    bool delay = false;
    Fault fault;
    if (FullSystem)
        fault = translateFs(req, tc, mode, NULL, delay, false, tranType);
    else
        fault = translateSe(req, tc, mode, NULL, delay, false);
    assert(!delay);
    return fault;
}

Fault
TLB::translateFunctional(const RequestPtr &req, ThreadContext *tc, Mode mode,
    TLB::ArmTranslationType tranType)
{
    updateMiscReg(tc, tranType);

    if (directToStage2) {
        assert(stage2Tlb);
        return stage2Tlb->translateFunctional(req, tc, mode, tranType);
    }

    bool delay = false;
    Fault fault;
    if (FullSystem)
        fault = translateFs(req, tc, mode, NULL, delay, false, tranType, true);
   else
        fault = translateSe(req, tc, mode, NULL, delay, false);
    assert(!delay);
    return fault;
}

void
TLB::translateTiming(const RequestPtr &req, ThreadContext *tc,
    Translation *translation, Mode mode, TLB::ArmTranslationType tranType)
{
    updateMiscReg(tc, tranType);

    if (directToStage2) {
        assert(stage2Tlb);
        stage2Tlb->translateTiming(req, tc, translation, mode, tranType);
        return;
    }

    assert(translation);

    translateComplete(req, tc, translation, mode, tranType, isStage2);
}

Fault
TLB::translateComplete(const RequestPtr &req, ThreadContext *tc,
        Translation *translation, Mode mode, TLB::ArmTranslationType tranType,
        bool callFromS2)
{
    bool delay = false;
    Fault fault;
    if (FullSystem)
        fault = translateFs(req, tc, mode, translation, delay, true, tranType);
    else
        fault = translateSe(req, tc, mode, translation, delay, true);
    DPRINTF(TLBVerbose, "Translation returning delay=%d fault=%d\n", delay, fault !=
            NoFault);
    // If we have a translation, and we're not in the middle of doing a stage
    // 2 translation tell the translation that we've either finished or its
    // going to take a while. By not doing this when we're in the middle of a
    // stage 2 translation we prevent marking the translation as delayed twice,
    // one when the translation starts and again when the stage 1 translation
    // completes.

    if (translation && (callFromS2 || !stage2Req || req->hasPaddr() ||
        fault != NoFault)) {
        if (!delay)
            translation->finish(fault, req, tc, mode);
        else
            translation->markDelayed();
    }
    return fault;
}

Port *
TLB::getTableWalkerPort()
{
    return &stage2Mmu->getDMAPort();
}

void
TLB::updateMiscReg(ThreadContext *tc, ArmTranslationType tranType)
{
    // check if the regs have changed, or the translation mode is different.
    // NOTE: the tran type doesn't affect stage 2 TLB's as they only handle
    // one type of translation anyway
    if (miscRegValid && miscRegContext == tc->contextId() &&
            ((tranType == curTranType) || isStage2)) {
        return;
    }

    DPRINTF(TLBVerbose, "TLB variables changed!\n");
    cpsr = tc->readMiscReg(MISCREG_CPSR);

    // Dependencies: SCR/SCR_EL3, CPSR
    isSecure = ArmISA::isSecure(tc) &&
        !(tranType & HypMode) && !(tranType & S1S2NsTran);

    aarch64EL = tranTypeEL(cpsr, tranType);
    aarch64 = isStage2 ?
        ELIs64(tc, EL2) :
        ELIs64(tc, aarch64EL == EL0 ? EL1 : aarch64EL);

    hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    if (aarch64) {  // AArch64
        // determine EL we need to translate in
        switch (aarch64EL) {
          case EL0:
            if (HaveVirtHostExt(tc) && hcr.tge == 1 && hcr.e2h == 1) {
                // VHE code for EL2&0 regime
                sctlr = tc->readMiscReg(MISCREG_SCTLR_EL2);
                ttbcr = tc->readMiscReg(MISCREG_TCR_EL2);
                uint64_t ttbr_asid = ttbcr.a1 ?
                    tc->readMiscReg(MISCREG_TTBR1_EL2) :
                    tc->readMiscReg(MISCREG_TTBR0_EL2);
                asid = bits(ttbr_asid,
                            (haveLargeAsid64 && ttbcr.as) ? 63 : 55, 48);

            } else {
                sctlr = tc->readMiscReg(MISCREG_SCTLR_EL1);
                ttbcr = tc->readMiscReg(MISCREG_TCR_EL1);
                uint64_t ttbr_asid = ttbcr.a1 ?
                    tc->readMiscReg(MISCREG_TTBR1_EL1) :
                    tc->readMiscReg(MISCREG_TTBR0_EL1);
                asid = bits(ttbr_asid,
                            (haveLargeAsid64 && ttbcr.as) ? 63 : 55, 48);

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
                            (haveLargeAsid64 && ttbcr.as) ? 63 : 55, 48);
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
                            (haveLargeAsid64 && ttbcr.as) ? 63 : 55, 48);
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

        scr = tc->readMiscReg(MISCREG_SCR_EL3);
        isPriv = aarch64EL != EL0;
        if (haveVirtualization) {
            vmid = bits(tc->readMiscReg(MISCREG_VTTBR_EL2), 55, 48);
            isHyp = aarch64EL == EL2;
            isHyp |= tranType & HypMode;
            isHyp &= (tranType & S1S2NsTran) == 0;
            isHyp &= (tranType & S1CTran)    == 0;
            bool vm = hcr.vm;
            if (HaveVirtHostExt(tc) && hcr.e2h == 1 && hcr.tge ==1) {
                vm = 0;
            }

            if (hcr.e2h == 1 && (aarch64EL == EL2
                                  || (hcr.tge ==1 && aarch64EL == EL0))) {
                isHyp = true;
                directToStage2 = false;
                stage2Req      = false;
                stage2DescReq  = false;
            } else {
            // Work out if we should skip the first stage of translation and go
            // directly to stage 2. This value is cached so we don't have to
            // compute it for every translation.
                bool sec = !isSecure || (isSecure && IsSecureEL2Enabled(tc));
                stage2Req = isStage2 ||
                            (vm && !isHyp && sec &&
                             !(tranType & S1CTran) && (aarch64EL < EL2) &&
                             !(tranType & S1E1Tran)); // <--- FIX THIS HACK
                stage2DescReq = isStage2 ||  (vm && !isHyp && sec &&
                                (aarch64EL < EL2));
                directToStage2 = !isStage2 && stage2Req && !sctlr.m;
            }
        } else {
            vmid           = 0;
            isHyp          = false;
            directToStage2 = false;
            stage2Req      = false;
            stage2DescReq  = false;
        }
    } else {  // AArch32
        sctlr  = tc->readMiscReg(snsBankedIndex(MISCREG_SCTLR, tc,
                                 !isSecure));
        ttbcr  = tc->readMiscReg(snsBankedIndex(MISCREG_TTBCR, tc,
                                 !isSecure));
        scr    = tc->readMiscReg(MISCREG_SCR);
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
        hcr  = tc->readMiscReg(MISCREG_HCR);

        if (haveVirtualization) {
            vmid   = bits(tc->readMiscReg(MISCREG_VTTBR), 55, 48);
            isHyp  = cpsr.mode == MODE_HYP;
            isHyp |=  tranType & HypMode;
            isHyp &= (tranType & S1S2NsTran) == 0;
            isHyp &= (tranType & S1CTran)    == 0;
            if (isHyp) {
                sctlr = tc->readMiscReg(MISCREG_HSCTLR);
            }
            // Work out if we should skip the first stage of translation and go
            // directly to stage 2. This value is cached so we don't have to
            // compute it for every translation.
            bool sec = !isSecure || (isSecure && IsSecureEL2Enabled(tc));
            stage2Req      = hcr.vm && !isStage2 && !isHyp && sec &&
                             !(tranType & S1CTran);
            stage2DescReq  = hcr.vm && !isStage2 && !isHyp && sec;
            directToStage2 = stage2Req && !sctlr.m;
        } else {
            vmid           = 0;
            stage2Req      = false;
            isHyp          = false;
            directToStage2 = false;
            stage2DescReq  = false;
        }
    }
    miscRegValid = true;
    miscRegContext = tc->contextId();
    curTranType  = tranType;
}

ExceptionLevel
TLB::tranTypeEL(CPSR cpsr, ArmTranslationType type)
{
    switch (type) {
      case S1E0Tran:
      case S12E0Tran:
        return EL0;

      case S1E1Tran:
      case S12E1Tran:
        return EL1;

      case S1E2Tran:
        return EL2;

      case S1E3Tran:
        return EL3;

      case NormalTran:
      case S1CTran:
      case S1S2NsTran:
      case HypMode:
        return currEL(cpsr);

      default:
        panic("Unknown translation mode!\n");
    }
}

Fault
TLB::getTE(TlbEntry **te, const RequestPtr &req, ThreadContext *tc, Mode mode,
        Translation *translation, bool timing, bool functional,
        bool is_secure, TLB::ArmTranslationType tranType)
{
    // In a 2-stage system, the IPA->PA translation can be started via this
    // call so make sure the miscRegs are correct.
    if (isStage2) {
        updateMiscReg(tc, tranType);
    }
    bool is_fetch = (mode == Execute);
    bool is_write = (mode == Write);

    Addr vaddr_tainted = req->getVaddr();
    Addr vaddr = 0;
    ExceptionLevel target_el = aarch64 ? aarch64EL : EL1;
    if (aarch64) {
        vaddr = purifyTaggedAddr(vaddr_tainted, tc, target_el, (TCR)ttbcr,
                                 mode==Execute);
    } else {
        vaddr = vaddr_tainted;
    }
    *te = lookup(vaddr, asid, vmid, isHyp, is_secure, false, false, target_el,
                 false);
    if (*te == NULL) {
        if (req->isPrefetch()) {
            // if the request is a prefetch don't attempt to fill the TLB or go
            // any further with the memory access (here we can safely use the
            // fault status for the short desc. format in all cases)
           stats.prefetchFaults++;
           return std::make_shared<PrefetchAbort>(
               vaddr_tainted, ArmFault::PrefetchTLBMiss, isStage2);
        }

        if (is_fetch)
            stats.instMisses++;
        else if (is_write)
            stats.writeMisses++;
        else
            stats.readMisses++;

        // start translation table walk, pass variables rather than
        // re-retreaving in table walker for speed
        DPRINTF(TLB, "TLB Miss: Starting hardware table walker for %#x(%d:%d)\n",
                vaddr_tainted, asid, vmid);
        Fault fault;
        fault = tableWalker->walk(req, tc, asid, vmid, isHyp, mode,
                                  translation, timing, functional, is_secure,
                                  tranType, stage2DescReq);
        // for timing mode, return and wait for table walk,
        if (timing || fault != NoFault) {
            return fault;
        }

        *te = lookup(vaddr, asid, vmid, isHyp, is_secure, false, false,
                     target_el, false);
        if (!*te)
            printTlb();
        assert(*te);
    } else {
        if (is_fetch)
            stats.instHits++;
        else if (is_write)
           stats.writeHits++;
        else
            stats.readHits++;
    }
    return NoFault;
}

Fault
TLB::getResultTe(TlbEntry **te, const RequestPtr &req,
        ThreadContext *tc, Mode mode,
        Translation *translation, bool timing, bool functional,
        TlbEntry *mergeTe)
{
    Fault fault;

    if (isStage2) {
        // We are already in the stage 2 TLB. Grab the table entry for stage
        // 2 only. We are here because stage 1 translation is disabled.
        TlbEntry *s2Te = NULL;
        // Get the stage 2 table entry
        fault = getTE(&s2Te, req, tc, mode, translation, timing, functional,
                      isSecure, curTranType);
        // Check permissions of stage 2
        if ((s2Te != NULL) && (fault == NoFault)) {
            if (aarch64)
                fault = checkPermissions64(s2Te, req, mode, tc);
            else
                fault = checkPermissions(s2Te, req, mode);
        }
        *te = s2Te;
        return fault;
    }

    TlbEntry *s1Te = NULL;

    Addr vaddr_tainted = req->getVaddr();

    // Get the stage 1 table entry
    fault = getTE(&s1Te, req, tc, mode, translation, timing, functional,
                  isSecure, curTranType);
    // only proceed if we have a valid table entry
    if ((s1Te != NULL) && (fault == NoFault)) {
        // Check stage 1 permissions before checking stage 2
        if (aarch64)
            fault = checkPermissions64(s1Te, req, mode, tc);
        else
            fault = checkPermissions(s1Te, req, mode);
        if (stage2Req & (fault == NoFault)) {
            Stage2LookUp *s2Lookup = new Stage2LookUp(this, stage2Tlb, *s1Te,
                req, translation, mode, timing, functional, isSecure,
                curTranType);
            fault = s2Lookup->getTe(tc, mergeTe);
            if (s2Lookup->isComplete()) {
                *te = mergeTe;
                // We've finished with the lookup so delete it
                delete s2Lookup;
            } else {
                // The lookup hasn't completed, so we can't delete it now. We
                // get round this by asking the object to self delete when the
                // translation is complete.
                s2Lookup->setSelfDelete();
            }
        } else {
            // This case deals with an S1 hit (or bypass), followed by
            // an S2 hit-but-perms issue
            if (isStage2) {
                DPRINTF(TLBVerbose, "s2TLB: reqVa %#x, reqPa %#x, fault %p\n",
                        vaddr_tainted, req->hasPaddr() ? req->getPaddr() : ~0, fault);
                if (fault != NoFault) {
                    ArmFault *armFault = reinterpret_cast<ArmFault *>(fault.get());
                    armFault->annotate(ArmFault::S1PTW, false);
                    armFault->annotate(ArmFault::OVA, vaddr_tainted);
                }
            }
            *te = s1Te;
        }
    }
    return fault;
}

void
TLB::setTestInterface(SimObject *_ti)
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
TLB::testTranslation(const RequestPtr &req, Mode mode,
                     TlbEntry::DomainType domain)
{
    if (!test || !req->hasSize() || req->getSize() == 0 ||
        req->isCacheMaintenance()) {
        return NoFault;
    } else {
        return test->translationCheck(req, isPriv, mode, domain);
    }
}

Fault
TLB::testWalk(Addr pa, Addr size, Addr va, bool is_secure, Mode mode,
              TlbEntry::DomainType domain, LookupLevel lookup_level)
{
    if (!test) {
        return NoFault;
    } else {
        return test->walkCheck(pa, size, va, is_secure, isPriv, mode,
                               domain, lookup_level);
    }
}


ArmISA::TLB *
ArmTLBParams::create()
{
    return new ArmISA::TLB(this);
}
