/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
 * Copyright (c) 2007 MIPS Technologies, Inc.
 * Copyright (c) 2007-2008 The Florida State University
 * Copyright (c) 2009 The University of Edinburgh
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

#ifndef __ARCH_POWER_TLB_HH__
#define __ARCH_POWER_TLB_HH__

#include <map>

#include "arch/generic/tlb.hh"
#include "arch/power/isa_traits.hh"
#include "arch/power/pagetable.hh"
#include "arch/power/utility.hh"
#include "base/statistics.hh"
#include "mem/request.hh"
#include "params/PowerTLB.hh"

class ThreadContext;

namespace PowerISA {

// This is copied from the ARM ISA and has not been checked against the
// Power at all.
struct TlbEntry
{
    Addr _pageStart;

    TlbEntry()
    {
    }

    TlbEntry(Addr asn, Addr vaddr, Addr paddr,
             bool uncacheable, bool read_only)
        : _pageStart(paddr)
    {
        if (uncacheable || read_only)
            warn("Power TlbEntry does not support uncacheable"
                 " or read-only mappings\n");
    }

    void
    updateVaddr(Addr new_vaddr)
    {
        panic("unimplemented");
    }

    Addr
    pageStart()
    {
        return _pageStart;
    }

    void
    serialize(CheckpointOut &cp) const
    {
        SERIALIZE_SCALAR(_pageStart);
    }

    void
    unserialize(CheckpointIn &cp)
    {
        UNSERIALIZE_SCALAR(_pageStart);
    }
};

class TLB : public BaseTLB
{
  protected:
    typedef std::multimap<Addr, int> PageTable;
    PageTable lookupTable;      // Quick lookup into page table

    PowerISA::PTE *table;       // the Page Table
    int size;                   // TLB Size
    int nlu;                    // not last used entry (for replacement)

    void
    nextnlu()
    {
        if (++nlu >= size) {
            nlu = 0;
        }
    }

    PowerISA::PTE *lookup(Addr vpn, uint8_t asn) const;

  public:
    typedef PowerTLBParams Params;
    TLB(const Params *p);
    virtual ~TLB();

    void takeOverFrom(BaseTLB *otlb) override {}

    int probeEntry(Addr vpn,uint8_t) const;
    PowerISA::PTE *getEntry(unsigned) const;

    int smallPages;

    int
    getsize() const
    {
        return size;
    }

    PowerISA::PTE &index(bool advance = true);
    void insert(Addr vaddr, PowerISA::PTE &pte);
    void insertAt(PowerISA::PTE &pte, unsigned Index, int _smallPages);
    void flushAll() override;

    void
    demapPage(Addr vaddr, uint64_t asn) override
    {
        panic("demapPage unimplemented.\n");
    }

    // static helper functions... really
    static bool validVirtualAddress(Addr vaddr);
    static Fault checkCacheability(const RequestPtr &req);
    Fault translateInst(const RequestPtr &req, ThreadContext *tc);
    Fault translateData(const RequestPtr &req, ThreadContext *tc, bool write);
    Fault translateAtomic(
            const RequestPtr &req, ThreadContext *tc, Mode mode) override;
    void translateTiming(
            const RequestPtr &req, ThreadContext *tc,
            Translation *translation, Mode mode) override;
    Fault translateFunctional(
            const RequestPtr &req, ThreadContext *tc, Mode mode) override;
    Fault finalizePhysical(
            const RequestPtr &req,
            ThreadContext *tc, Mode mode) const override;

    // Checkpointing
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

} // namespace PowerISA

#endif // __ARCH_POWER_TLB_HH__
