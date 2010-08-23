/*
 * Copyright (c) 2010 ARM Limited
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
#include "arch/arm/tlb.hh"
#include "arch/arm/utility.hh"
#include "base/inifile.hh"
#include "base/str.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "mem/page_table.hh"
#include "params/ArmTLB.hh"
#include "sim/process.hh"

#if FULL_SYSTEM
#include "arch/arm/table_walker.hh"
#endif

using namespace std;
using namespace ArmISA;

TLB::TLB(const Params *p)
    : BaseTLB(p), size(p->size), nlu(0)
#if FULL_SYSTEM
      , tableWalker(p->walker)
#endif
{
    table = new TlbEntry[size];
    memset(table, 0, sizeof(TlbEntry[size]));

#if FULL_SYSTEM
    tableWalker->setTlb(this);
#endif
}

TLB::~TLB()
{
    if (table)
        delete [] table;
}

TlbEntry*
TLB::lookup(Addr va, uint8_t cid)
{
    // XXX This should either turn into a TlbMap or add caching

    TlbEntry *retval = NULL;

    // Do some kind of caching, fast indexing, anything

    int x = 0;
    while (retval == NULL && x < size) {
        if (table[x].match(va, cid)) {
            retval = &table[x];
            if (x == nlu)
                nextnlu();

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

    if (table[nlu].valid)
        DPRINTF(TLB, " - Replacing Valid entry %#x, asn %d ppn %#x size: %#x ap:%d\n",
            table[nlu].vpn << table[nlu].N, table[nlu].asid, table[nlu].pfn << table[nlu].N,
            table[nlu].size, table[nlu].ap);

    // XXX Update caching, lookup table etc
    table[nlu] = entry;

    // XXX Figure out how entries are generally inserted in ARM
    nextnlu();
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
       if (te->valid)
           DPRINTF(TLB, " -  %#x, asn %d ppn %#x size: %#x ap:%d\n",
                te->vpn << te->N, te->asid, te->pfn << te->N, te->size, te->ap);
       x++;
    }

    memset(table, 0, sizeof(TlbEntry[size]));
    nlu = 0;
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
        te = lookup(mva,asn);
    }
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
        }
        x++;
    }
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
        }
        x++;
    }
}

void
TLB::serialize(ostream &os)
{
    panic("Implement Serialize\n");
}

void
TLB::unserialize(Checkpoint *cp, const string &section)
{

    panic("Need to properly unserialize TLB\n");
}

void
TLB::regStats()
{
    read_hits
        .name(name() + ".read_hits")
        .desc("DTB read hits")
        ;

    read_misses
        .name(name() + ".read_misses")
        .desc("DTB read misses")
        ;


    read_accesses
        .name(name() + ".read_accesses")
        .desc("DTB read accesses")
        ;

    write_hits
        .name(name() + ".write_hits")
        .desc("DTB write hits")
        ;

    write_misses
        .name(name() + ".write_misses")
        .desc("DTB write misses")
        ;


    write_accesses
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

    hits = read_hits + write_hits;
    misses = read_misses + write_misses;
    accesses = read_accesses + write_accesses;
}

#if !FULL_SYSTEM
Fault
TLB::translateSe(RequestPtr req, ThreadContext *tc, Mode mode,
        Translation *translation, bool &delay, bool timing)
{
    // XXX Cache misc registers and have miscreg write function inv cache
    Addr vaddr = req->getVaddr() & ~PcModeMask;
    SCTLR sctlr = tc->readMiscReg(MISCREG_SCTLR);
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

#else // FULL_SYSTEM

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
        Translation *translation, bool &delay, bool timing)
{
    // XXX Cache misc registers and have miscreg write function inv cache
    Addr vaddr = req->getVaddr() & ~PcModeMask;
    SCTLR sctlr = tc->readMiscReg(MISCREG_SCTLR);
    CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);
    uint32_t flags = req->getFlags();

    bool is_fetch = (mode == Execute);
    bool is_write = (mode == Write);
    bool is_priv = (cpsr.mode != MODE_USER) && !(flags & UserMode);

    DPRINTF(TLBVerbose, "CPSR is user:%d UserMode:%d\n", cpsr.mode == MODE_USER, flags
            & UserMode);
    // If this is a clrex instruction, provide a PA of 0 with no fault
    // This will force the monitor to set the tracked address to 0
    // a bit of a hack but this effectively clrears this processors monitor
    if (flags & Clrex){
       req->setPaddr(0);
       req->setFlags(Request::UNCACHEABLE);
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
                return new DataAbort(vaddr, 0, is_write, ArmFault::AlignmentFault);
            }
        }
    }

    uint32_t context_id = tc->readMiscReg(MISCREG_CONTEXTIDR);
    Fault fault;


    if (!sctlr.m) {
        req->setPaddr(vaddr);
        if (sctlr.tre == 0) {
            req->setFlags(Request::UNCACHEABLE);
        } else {
            PRRR prrr = tc->readMiscReg(MISCREG_PRRR);
            NMRR nmrr = tc->readMiscReg(MISCREG_NMRR);

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

    DPRINTF(TLBVerbose, "Translating vaddr=%#x context=%d\n", vaddr, context_id);
    // Translation enabled

    TlbEntry *te = lookup(vaddr, context_id);
    if (te == NULL) {
        // start translation table walk, pass variables rather than
        // re-retreaving in table walker for speed
        DPRINTF(TLB, "TLB Miss: Starting hardware table walker for %#x(%d)\n",
                vaddr, context_id);
        fault = tableWalker->walk(req, tc, context_id, mode, translation,
                timing);
        if (timing) {
            delay = true;
            // for timing mode, return and wait for table walk
            return fault;
        }
        if (fault)
            return fault;

        te = lookup(vaddr, context_id);
        if (!te)
            printTlb();
        assert(te);
    }

    // Set memory attributes
    DPRINTF(TLBVerbose,
            "Setting memory attributes: shareable: %d, innerAttrs: %d, \
            outerAttrs: %d\n",
            te->shareable, te->innerAttrs, te->outerAttrs);
    setAttr(te->attributes);
    if (te->nonCacheable)
        req->setFlags(Request::UNCACHEABLE);
    uint32_t dacr = tc->readMiscReg(MISCREG_DACR);
    switch ( (dacr >> (te->domain * 2)) & 0x3) {
      case 0:
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
        DPRINTF(TLB, "TLB Fault: Prefetch abort on permission check. AP:%d priv:%d"
               " write:%d sNp:%d\n", ap, is_priv, is_write, te->sNp);
        return new PrefetchAbort(vaddr,
                (te->sNp ? ArmFault::Permission0 :
                 ArmFault::Permission1));
    } else if (abt) {
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

#endif

Fault
TLB::translateAtomic(RequestPtr req, ThreadContext *tc, Mode mode)
{
    bool delay = false;
    Fault fault;
#if FULL_SYSTEM
    fault = translateFs(req, tc, mode, NULL, delay, false);
#else
    fault = translateSe(req, tc, mode, NULL, delay, false);
#endif
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
#if FULL_SYSTEM
    fault = translateFs(req, tc, mode, translation, delay, true);
#else
    fault = translateSe(req, tc, mode, translation, delay, true);
#endif
    if (!delay)
        translation->finish(fault, req, tc, mode);
    return fault;
}

ArmISA::TLB *
ArmTLBParams::create()
{
    return new ArmISA::TLB(this);
}
