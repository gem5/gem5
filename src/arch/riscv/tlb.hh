/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
 * Copyright (c) 2007 MIPS Technologies, Inc.
 * Copyright (c) 2020 Barkhausen Institut
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

#ifndef __ARCH_RISCV_TLB_HH__
#define __ARCH_RISCV_TLB_HH__

#include <list>

#include "arch/generic/tlb.hh"
#include "arch/riscv/isa.hh"
#include "arch/riscv/isa_traits.hh"
#include "arch/riscv/pagetable.hh"
#include "arch/riscv/utility.hh"
#include "base/statistics.hh"
#include "mem/request.hh"
#include "params/RiscvTLB.hh"
#include "sim/sim_object.hh"

class ThreadContext;

/* To maintain compatibility with other architectures, we'll
   simply create an ITLB and DTLB that will point to the real TLB */
namespace RiscvISA {

class Walker;

class TLB : public BaseTLB
{
    typedef std::list<TlbEntry *> EntryList;

  protected:
    size_t size;
    std::vector<TlbEntry> tlb;  // our TLB
    TlbEntryTrie trie;          // for quick access
    EntryList freeList;         // free entries
    uint64_t lruSeq;

    Walker *walker;

    mutable Stats::Scalar read_hits;
    mutable Stats::Scalar read_misses;
    mutable Stats::Scalar read_acv;
    mutable Stats::Scalar read_accesses;
    mutable Stats::Scalar write_hits;
    mutable Stats::Scalar write_misses;
    mutable Stats::Scalar write_acv;
    mutable Stats::Scalar write_accesses;
    Stats::Formula hits;
    Stats::Formula misses;
    Stats::Formula accesses;

  public:
    typedef RiscvTLBParams Params;
    TLB(const Params *p);

    Walker *getWalker();

    void takeOverFrom(BaseTLB *otlb) override {}

    TlbEntry *insert(Addr vpn, const TlbEntry &entry);
    void flushAll() override;
    void demapPage(Addr vaddr, uint64_t asn) override;

    Fault checkPermissions(STATUS status, PrivilegeMode pmode, Addr vaddr,
                           Mode mode, PTESv39 pte);
    Fault createPagefault(Addr vaddr, Mode mode);

    PrivilegeMode getMemPriv(ThreadContext *tc, Mode mode);

    // Checkpointing
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    void regStats() override;

    Addr translateWithTLB(Addr vaddr, uint16_t asid, Mode mode);

    Fault translateAtomic(const RequestPtr &req,
                          ThreadContext *tc, Mode mode) override;
    void translateTiming(const RequestPtr &req, ThreadContext *tc,
                         Translation *translation, Mode mode) override;
    Fault translateFunctional(const RequestPtr &req,
                              ThreadContext *tc, Mode mode) override;
    Fault finalizePhysical(const RequestPtr &req,
                           ThreadContext *tc, Mode mode) const override;

  private:
    uint64_t nextSeq() { return ++lruSeq; }

    TlbEntry *lookup(Addr vpn, uint16_t asid, Mode mode, bool hidden);

    void evictLRU();
    void remove(size_t idx);

    Fault translate(const RequestPtr &req, ThreadContext *tc,
                    Translation *translation, Mode mode, bool &delayed);
    Fault doTranslate(const RequestPtr &req, ThreadContext *tc,
                      Translation *translation, Mode mode, bool &delayed);
};

}

#endif // __RISCV_MEMORY_HH__
