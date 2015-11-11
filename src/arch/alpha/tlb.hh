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
 * Authors: Nathan Binkert
 *          Steve Reinhardt
 */

#ifndef __ARCH_ALPHA_TLB_HH__
#define __ARCH_ALPHA_TLB_HH__

#include <map>

#include "arch/alpha/ev5.hh"
#include "arch/alpha/isa_traits.hh"
#include "arch/alpha/pagetable.hh"
#include "arch/alpha/utility.hh"
#include "arch/alpha/vtophys.hh"
#include "arch/generic/tlb.hh"
#include "base/statistics.hh"
#include "mem/request.hh"
#include "params/AlphaTLB.hh"

class ThreadContext;

namespace AlphaISA {

struct TlbEntry;

class TLB : public BaseTLB
{
  protected:
    mutable Stats::Scalar fetch_hits;
    mutable Stats::Scalar fetch_misses;
    mutable Stats::Scalar fetch_acv;
    mutable Stats::Formula fetch_accesses;
    mutable Stats::Scalar read_hits;
    mutable Stats::Scalar read_misses;
    mutable Stats::Scalar read_acv;
    mutable Stats::Scalar read_accesses;
    mutable Stats::Scalar write_hits;
    mutable Stats::Scalar write_misses;
    mutable Stats::Scalar write_acv;
    mutable Stats::Scalar write_accesses;
    Stats::Formula data_hits;
    Stats::Formula data_misses;
    Stats::Formula data_acv;
    Stats::Formula data_accesses;


    typedef std::multimap<Addr, int> PageTable;
    PageTable lookupTable;  // Quick lookup into page table

    std::vector<TlbEntry> table; // the Page Table
    int nlu;                // not last used entry (for replacement)

    void nextnlu() { if (++nlu >= table.size()) nlu = 0; }
    TlbEntry *lookup(Addr vpn, uint8_t asn);

  public:
    typedef AlphaTLBParams Params;
    TLB(const Params *p);
    virtual ~TLB();

    void takeOverFrom(BaseTLB *otlb) override {}

    void regStats() override;

    int getsize() const { return table.size(); }

    TlbEntry &index(bool advance = true);
    void insert(Addr vaddr, TlbEntry &entry);

    void flushAll() override;
    void flushProcesses();
    void flushAddr(Addr addr, uint8_t asn);

    void
    demapPage(Addr vaddr, uint64_t asn) override
    {
        assert(asn < (1 << 8));
        flushAddr(vaddr, asn);
    }

    // static helper functions... really EV5 VM traits
    static bool
    validVirtualAddress(Addr vaddr)
    {
        // unimplemented bits must be all 0 or all 1
        Addr unimplBits = vaddr & VAddrUnImplMask;
        return unimplBits == 0 || unimplBits == VAddrUnImplMask;
    }

    static Fault checkCacheability(RequestPtr &req, bool itb = false);

    // Checkpointing
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    // Most recently used page table entries
    TlbEntry *EntryCache[3];
    inline void
    flushCache()
    {
        memset(EntryCache, 0, 3 * sizeof(TlbEntry*));
    }

    inline TlbEntry *
    updateCache(TlbEntry *entry) {
        EntryCache[2] = EntryCache[1];
        EntryCache[1] = EntryCache[0];
        EntryCache[0] = entry;
        return entry;
    }

  protected:
    Fault translateData(RequestPtr req, ThreadContext *tc, bool write);
    Fault translateInst(RequestPtr req, ThreadContext *tc);

  public:
    Fault translateAtomic(RequestPtr req, ThreadContext *tc, Mode mode);
    void translateTiming(RequestPtr req, ThreadContext *tc,
                         Translation *translation, Mode mode);
    /**
     * translateFunctional stub function for future CheckerCPU support
     */
    Fault translateFunctional(RequestPtr req, ThreadContext *tc, Mode mode);
    Fault finalizePhysical(RequestPtr req, ThreadContext *tc, Mode mode) const;
};

} // namespace AlphaISA

#endif // __ARCH_ALPHA_TLB_HH__
