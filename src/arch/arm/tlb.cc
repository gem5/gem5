/*
 * Copyright (c) 2010-2012 ARM Limited
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
 *
 * Authors: Ali Saidi
 *          Nathan Binkert
 *          Steve Reinhardt
 */

#include <string>
#include <vector>

#include "arch/arm/faults.hh"
#include "arch/arm/pagetable.hh"
#include "arch/arm/system.hh"
#include "arch/arm/table_walker.hh"
#include "arch/arm/tlb.hh"
#include "arch/arm/utility.hh"
#include "base/inifile.hh"
#include "base/str.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/Checkpoint.hh"
#include "debug/TLB.hh"
#include "debug/TLBVerbose.hh"
#include "mem/page_table.hh"
#include "params/ArmTLB.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"

using namespace std;
using namespace ArmISA;

TLB::TLB(const Params *p)
    : BaseTLB(p), size(p->size) , tableWalker(p->walker),
    rangeMRU(1), bootUncacheability(false), miscRegValid(false)
{
    table = new TlbEntry[size];
    memset(table, 0, sizeof(TlbEntry) * size);

    tableWalker->setTlb(this);
}

TLB::~TLB()
{
    if (table)
        delete [] table;
}

bool
TLB::translateFunctional(ThreadContext *tc, Addr va, Addr &pa)
{
    if (!miscRegValid)
        updateMiscReg(tc);
    TlbEntry *e = lookup(va, contextId, true);
    if (!e)
        return false;
    pa = e->pAddr(va);
    return true;
}

Fault
TLB::finalizePhysical(RequestPtr req, ThreadContext *tc, Mode mode) const
{
    return NoFault;
}

TlbEntry*
TLB::lookup(Addr va, uint8_t cid, bool functional)
{

    TlbEntry *retval = NULL;

    // Maitaining LRU array

    int x = 0;
    while (retval == NULL && x < size) {
        if (table[x].match(va, cid)) {

            // We only move the hit entry ahead when the position is higher than rangeMRU
            if (x > rangeMRU && !functional) {
                TlbEntry tmp_entry = table[x];
                for(int i = x; i > 0; i--)
                    table[i] = table[i-1];
                table[0] = tmp_entry;
                retval = &table[0];
            } else {
                retval = &table[x];
            }
            break;
        }
        x++;
    }

    DPRINTF(TLBVerbose, "Lookup %#x, cid %#x -> %s ppn %#x size: %#x pa: %#x ap:%d\n",
            va, cid, retval ? "hit" : "miss", retval ? retval->pfn : 0,
            retval ? retval->size : 0, retval ? retval->pAddr(va) : 0,
            retval ? retval->ap : 0);
    ;
    return retval;
}

// insert a new TLB entry
void
TLB::insert(Addr addr, TlbEntry &entry)
{
    DPRINTF(TLB, "Inserting entry into TLB with pfn:%#x size:%#x vpn: %#x"
            " asid:%d N:%d global:%d valid:%d nc:%d sNp:%d xn:%d ap:%#x"
            " domain:%#x\n", entry.pfn, entry.size, entry.vpn, entry.asid,
            entry.N, entry.global, entry.valid, entry.nonCacheable, entry.sNp,
            entry.xn, entry.ap, entry.domain);

    if (table[size-1].valid)
        DPRINTF(TLB, " - Replacing Valid entry %#x, asn %d ppn %#x size: %#x ap:%d\n",
                table[size-1].vpn << table[size-1].N, table[size-1].asid,
                table[size-1].pfn << table[size-1].N, table[size-1].size,
                table[size-1].ap);

    //inserting to MRU position and evicting the LRU one

    for(int i = size-1; i > 0; i--)
      table[i] = table[i-1];
    table[0] = entry;

    inserts++;
}

void
TLB::printTlb()
{
    int x = 0;
    TlbEntry *te;
    DPRINTF(TLB, "Current TLB contents:\n");
    while (x < size) {
       te = &table[x];
       if (te->valid)
           DPRINTF(TLB, " *  %#x, asn %d ppn %#x size: %#x ap:%d\n",
                te->vpn << te->N, te->asid, te->pfn << te->N, te->size, te->ap);
       x++;
    }
}


void
TLB::flushAll()
{
    DPRINTF(TLB, "Flushing all TLB entries\n");
    int x = 0;
    TlbEntry *te;
    while (x < size) {
       te = &table[x];
       if (te->valid) {
           DPRINTF(TLB, " -  %#x, asn %d ppn %#x size: %#x ap:%d\n",
                te->vpn << te->N, te->asid, te->pfn << te->N, te->size, te->ap);
           flushedEntries++;
       }
       x++;
    }

    memset(table, 0, sizeof(TlbEntry) * size);

    flushTlb++;
}


void
TLB::flushMvaAsid(Addr mva, uint64_t asn)
{
    DPRINTF(TLB, "Flushing mva %#x asid: %#x\n", mva, asn);
    TlbEntry *te;

    te = lookup(mva, asn);
    while (te != NULL) {
     DPRINTF(TLB, " -  %#x, asn %d ppn %#x size: %#x ap:%d\n",
            te->vpn << te->N, te->asid, te->pfn << te->N, te->size, te->ap);
        te->valid = false;
        flushedEntries++;
        te = lookup(mva,asn);
    }
    flushTlbMvaAsid++;
}

void
TLB::flushAsid(uint64_t asn)
{
    DPRINTF(TLB, "Flushing all entries with asid: %#x\n", asn);

    int x = 0;
    TlbEntry *te;

    while (x < size) {
        te = &table[x];
        if (te->asid == asn) {
            te->valid = false;
            DPRINTF(TLB, " -  %#x, asn %d ppn %#x size: %#x ap:%d\n",
                te->vpn << te->N, te->asid, te->pfn << te->N, te->size, te->ap);
            flushedEntries++;
        }
        x++;
    }
    flushTlbAsid++;
}

void
TLB::flushMva(Addr mva)
{
    DPRINTF(TLB, "Flushing all entries with mva: %#x\n", mva);

    int x = 0;
    TlbEntry *te;

    while (x < size) {
        te = &table[x];
        Addr v = te->vpn << te->N;
        if (mva >= v && mva < v + te->size) {
            te->valid = false;
            DPRINTF(TLB, " -  %#x, asn %d ppn %#x size: %#x ap:%d\n",
                te->vpn << te->N, te->asid, te->pfn << te->N, te->size, te->ap);
            flushedEntries++;
        }
        x++;
    }
    flushTlbMva++;
}

void
TLB::drainResume()
{
    // We might have unserialized something or switched CPUs, so make
    // sure to re-read the misc regs.
    miscRegValid = false;
}

void
TLB::serialize(ostream &os)
{
    DPRINTF(Checkpoint, "Serializing Arm TLB\n");

    SERIALIZE_SCALAR(_attr);

    int num_entries = size;
    SERIALIZE_SCALAR(num_entries);
    for(int i = 0; i < size; i++){
        nameOut(os, csprintf("%s.TlbEntry%d", name(), i));
        table[i].serialize(os);
    }
}

void
TLB::unserialize(Checkpoint *cp, const string &section)
{
    DPRINTF(Checkpoint, "Unserializing Arm TLB\n");

    UNSERIALIZE_SCALAR(_attr);
    int num_entries;
    UNSERIALIZE_SCALAR(num_entries);
    for(int i = 0; i < min(size, num_entries); i++){
        table[i].unserialize(cp, csprintf("%s.TlbEntry%d", section, i));
    }
}

void
TLB::regStats()
{
    instHits
        .name(name() + ".inst_hits")
        .desc("ITB inst hits")
        ;

    instMisses
        .name(name() + ".inst_misses")
        .desc("ITB inst misses")
        ;

    instAccesses
        .name(name() + ".inst_accesses")
        .desc("ITB inst accesses")
        ;

    readHits
        .name(name() + ".read_hits")
        .desc("DTB read hits")
        ;

    readMisses
        .name(name() + ".read_misses")
        .desc("DTB read misses")
        ;

    readAccesses
        .name(name() + ".read_accesses")
        .desc("DTB read accesses")
        ;

    writeHits
        .name(name() + ".write_hits")
        .desc("DTB write hits")
        ;

    writeMisses
        .name(name() + ".write_misses")
        .desc("DTB write misses")
        ;

    writeAccesses
        .name(name() + ".write_accesses")
        .desc("DTB write accesses")
        ;

    hits
        .name(name() + ".hits")
        .desc("DTB hits")
        ;

    misses
        .name(name() + ".misses")
        .desc("DTB misses")
        ;

    accesses
        .name(name() + ".accesses")
        .desc("DTB accesses")
        ;

    flushTlb
        .name(name() + ".flush_tlb")
        .desc("Number of times complete TLB was flushed")
        ;

    flushTlbMva
        .name(name() + ".flush_tlb_mva")
        .desc("Number of times TLB was flushed by MVA")
        ;

    flushTlbMvaAsid
        .name(name() + ".flush_tlb_mva_asid")
        .desc("Number of times TLB was flushed by MVA & ASID")
        ;

    flushTlbAsid
        .name(name() + ".flush_tlb_asid")
        .desc("Number of times TLB was flushed by ASID")
        ;

    flushedEntries
        .name(name() + ".flush_entries")
        .desc("Number of entries that have been flushed from TLB")
        ;

    alignFaults
        .name(name() + ".align_faults")
        .desc("Number of TLB faults due to alignment restrictions")
        ;

    prefetchFaults
        .name(name() + ".prefetch_faults")
        .desc("Number of TLB faults due to prefetch")
        ;

    domainFaults
        .name(name() + ".domain_faults")
        .desc("Number of TLB faults due to domain restrictions")
        ;

    permsFaults
        .name(name() + ".perms_faults")
        .desc("Number of TLB faults due to permissions restrictions")
        ;

    instAccesses = instHits + instMisses;
    readAccesses = readHits + readMisses;
    writeAccesses = writeHits + writeMisses;
    hits = readHits + writeHits + instHits;
    misses = readMisses + writeMisses + instMisses;
    accesses = readAccesses + writeAccesses + instAccesses;
}

Fault
TLB::translateSe(RequestPtr req, ThreadContext *tc, Mode mode,
        Translation *translation, bool &delay, bool timing)
{
    if (!miscRegValid)
        updateMiscReg(tc);
    Addr vaddr = req->getVaddr();
    uint32_t flags = req->getFlags();

    bool is_fetch = (mode == Execute);
    bool is_write = (mode == Write);

    if (!is_fetch) {
        assert(flags & MustBeOne);
        if (sctlr.a || !(flags & AllowUnaligned)) {
            if (vaddr & flags & AlignmentMask) {
                return new DataAbort(vaddr, 0, is_write, ArmFault::AlignmentFault);
            }
        }
    }

    Addr paddr;
    Process *p = tc->getProcessPtr();

    if (!p->pTable->translate(vaddr, paddr))
        return Fault(new GenericPageTableFault(vaddr));
    req->setPaddr(paddr);

    return NoFault;
}

Fault
TLB::trickBoxCheck(RequestPtr req, Mode mode, uint8_t domain, bool sNp)
{
    return NoFault;
}

Fault
TLB::walkTrickBoxCheck(Addr pa, Addr va, Addr sz, bool is_exec,
        bool is_write, uint8_t domain, bool sNp)
{
    return NoFault;
}

Fault
TLB::translateFs(RequestPtr req, ThreadContext *tc, Mode mode,
        Translation *translation, bool &delay, bool timing, bool functional)
{
    // No such thing as a functional timing access
    assert(!(timing && functional));

    if (!miscRegValid) {
        updateMiscReg(tc);
        DPRINTF(TLBVerbose, "TLB variables changed!\n");
    }

    Addr vaddr = req->getVaddr();
    uint32_t flags = req->getFlags();

    bool is_fetch = (mode == Execute);
    bool is_write = (mode == Write);
    bool is_priv = isPriv && !(flags & UserMode);

    req->setAsid(contextId.asid);
    if (is_priv)
        req->setFlags(Request::PRIVILEGED);

    req->taskId(tc->getCpuPtr()->taskId());

    DPRINTF(TLBVerbose, "CPSR is priv:%d UserMode:%d\n",
            isPriv, flags & UserMode);
    // If this is a clrex instruction, provide a PA of 0 with no fault
    // This will force the monitor to set the tracked address to 0
    // a bit of a hack but this effectively clrears this processors monitor
    if (flags & Request::CLEAR_LL){
       req->setPaddr(0);
       req->setFlags(Request::UNCACHEABLE);
       req->setFlags(Request::CLEAR_LL);
       return NoFault;
    }
    if ((req->isInstFetch() && (!sctlr.i)) ||
        ((!req->isInstFetch()) && (!sctlr.c))){
       req->setFlags(Request::UNCACHEABLE);
    }
    if (!is_fetch) {
        assert(flags & MustBeOne);
        if (sctlr.a || !(flags & AllowUnaligned)) {
            if (vaddr & flags & AlignmentMask) {
                alignFaults++;
                return new DataAbort(vaddr, 0, is_write, ArmFault::AlignmentFault);
            }
        }
    }

    Fault fault;

    if (!sctlr.m) {
        req->setPaddr(vaddr);
        if (sctlr.tre == 0) {
            req->setFlags(Request::UNCACHEABLE);
        } else {
            if (nmrr.ir0 == 0 || nmrr.or0 == 0 || prrr.tr0 != 0x2)
               req->setFlags(Request::UNCACHEABLE);
        }

        // Set memory attributes
        TlbEntry temp_te;
        tableWalker->memAttrs(tc, temp_te, sctlr, 0, 1);
        temp_te.shareable = true;
        DPRINTF(TLBVerbose, "(No MMU) setting memory attributes: shareable:\
                %d, innerAttrs: %d, outerAttrs: %d\n", temp_te.shareable,
                temp_te.innerAttrs, temp_te.outerAttrs);
        setAttr(temp_te.attributes);

        return trickBoxCheck(req, mode, 0, false);
    }

    DPRINTF(TLBVerbose, "Translating vaddr=%#x context=%d\n", vaddr, contextId);
    // Translation enabled

    TlbEntry *te = lookup(vaddr, contextId);
    if (te == NULL) {
        if (req->isPrefetch()){
           //if the request is a prefetch don't attempt to fill the TLB
           //or go any further with the memory access
           prefetchFaults++;
           return new PrefetchAbort(vaddr, ArmFault::PrefetchTLBMiss);
        }

        if (is_fetch)
            instMisses++;
        else if (is_write)
            writeMisses++;
        else
            readMisses++;

        // start translation table walk, pass variables rather than
        // re-retreaving in table walker for speed
        DPRINTF(TLB, "TLB Miss: Starting hardware table walker for %#x(%d)\n",
                vaddr, contextId);
        fault = tableWalker->walk(req, tc, contextId, mode, translation,
                                  timing, functional);
        if (timing && fault == NoFault) {
            delay = true;
            // for timing mode, return and wait for table walk
            return fault;
        }
        if (fault)
            return fault;

        te = lookup(vaddr, contextId);
        if (!te)
            printTlb();
        assert(te);
    } else {
        if (is_fetch)
            instHits++;
        else if (is_write)
            writeHits++;
        else
            readHits++;
    }

    // Set memory attributes
    DPRINTF(TLBVerbose,
            "Setting memory attributes: shareable: %d, innerAttrs: %d, \
            outerAttrs: %d\n",
            te->shareable, te->innerAttrs, te->outerAttrs);
    setAttr(te->attributes);
    if (te->nonCacheable) {
        req->setFlags(Request::UNCACHEABLE);

        // Prevent prefetching from I/O devices.
        if (req->isPrefetch()) {
            return new PrefetchAbort(vaddr, ArmFault::PrefetchUncacheable);
        }
    }

    if (!bootUncacheability &&
            ((ArmSystem*)tc->getSystemPtr())->adderBootUncacheable(vaddr))
        req->setFlags(Request::UNCACHEABLE);

    switch ( (dacr >> (te->domain * 2)) & 0x3) {
      case 0:
        domainFaults++;
        DPRINTF(TLB, "TLB Fault: Data abort on domain. DACR: %#x domain: %#x"
               " write:%d sNp:%d\n", dacr, te->domain, is_write, te->sNp);
        if (is_fetch)
            return new PrefetchAbort(vaddr,
                (te->sNp ? ArmFault::Domain0 : ArmFault::Domain1));
        else
            return new DataAbort(vaddr, te->domain, is_write,
                (te->sNp ? ArmFault::Domain0 : ArmFault::Domain1));
      case 1:
        // Continue with permissions check
        break;
      case 2:
        panic("UNPRED domain\n");
      case 3:
        req->setPaddr(te->pAddr(vaddr));
        fault = trickBoxCheck(req, mode, te->domain, te->sNp);
        if (fault)
            return fault;
        return NoFault;
    }

    uint8_t ap = te->ap;

    if (sctlr.afe == 1)
        ap |= 1;

    bool abt;

   /* if (!sctlr.xp)
        ap &= 0x3;
*/
    switch (ap) {
      case 0:
        DPRINTF(TLB, "Access permissions 0, checking rs:%#x\n", (int)sctlr.rs);
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
        break;
      case 3:
        abt = false;
        break;
      case 4:
        panic("UNPRED premissions\n");
      case 5:
        abt = !is_priv || is_write;
        break;
      case 6:
      case 7:
        abt = is_write;
        break;
      default:
        panic("Unknown permissions\n");
    }
    if ((is_fetch) && (abt || te->xn)) {
        permsFaults++;
        DPRINTF(TLB, "TLB Fault: Prefetch abort on permission check. AP:%d priv:%d"
               " write:%d sNp:%d\n", ap, is_priv, is_write, te->sNp);
        return new PrefetchAbort(vaddr,
                (te->sNp ? ArmFault::Permission0 :
                 ArmFault::Permission1));
    } else if (abt) {
        permsFaults++;
        DPRINTF(TLB, "TLB Fault: Data abort on permission check. AP:%d priv:%d"
               " write:%d sNp:%d\n", ap, is_priv, is_write, te->sNp);
        return new DataAbort(vaddr, te->domain, is_write,
                (te->sNp ? ArmFault::Permission0 :
                 ArmFault::Permission1));
    }

    req->setPaddr(te->pAddr(vaddr));
    // Check for a trickbox generated address fault
    fault = trickBoxCheck(req, mode, te->domain, te->sNp);
    if (fault)
        return fault;

    return NoFault;
}

Fault
TLB::translateAtomic(RequestPtr req, ThreadContext *tc, Mode mode)
{
    bool delay = false;
    Fault fault;
    if (FullSystem)
        fault = translateFs(req, tc, mode, NULL, delay, false);
    else
        fault = translateSe(req, tc, mode, NULL, delay, false);
    assert(!delay);
    return fault;
}

Fault
TLB::translateFunctional(RequestPtr req, ThreadContext *tc, Mode mode)
{
    bool delay = false;
    Fault fault;
    if (FullSystem)
        fault = translateFs(req, tc, mode, NULL, delay, false, true);
    else
        fault = translateSe(req, tc, mode, NULL, delay, false);
    assert(!delay);
    return fault;
}

Fault
TLB::translateTiming(RequestPtr req, ThreadContext *tc,
        Translation *translation, Mode mode)
{
    assert(translation);
    bool delay = false;
    Fault fault;
    if (FullSystem)
        fault = translateFs(req, tc, mode, translation, delay, true);
    else
        fault = translateSe(req, tc, mode, translation, delay, true);
    DPRINTF(TLBVerbose, "Translation returning delay=%d fault=%d\n", delay, fault !=
            NoFault);
    if (!delay)
        translation->finish(fault, req, tc, mode);
    else
        translation->markDelayed();
    return fault;
}

BaseMasterPort*
TLB::getMasterPort()
{
    return &tableWalker->getMasterPort("port");
}



ArmISA::TLB *
ArmTLBParams::create()
{
    return new ArmISA::TLB(this);
}
