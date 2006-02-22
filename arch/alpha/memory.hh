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
 */

#ifndef __ALPHA_MEMORY_HH__
#define __ALPHA_MEMORY_HH__

#include <map>

#include "arch/alpha/isa_traits.hh"
#include "arch/alpha/faults.hh"
#include "base/statistics.hh"
#include "mem/mem_req.hh"
#include "sim/sim_object.hh"

class ExecContext;

class AlphaTLB : public SimObject
{
  protected:
    typedef std::multimap<Addr, int> PageTable;
    PageTable lookupTable;	// Quick lookup into page table

    AlphaISA::PTE *table;	// the Page Table
    int size;			// TLB Size
    int nlu;			// not last used entry (for replacement)

    void nextnlu() { if (++nlu >= size) nlu = 0; }
    AlphaISA::PTE *lookup(Addr vpn, uint8_t asn) const;

  public:
    AlphaTLB(const std::string &name, int size);
    virtual ~AlphaTLB();

    int getsize() const { return size; }

    AlphaISA::PTE &index(bool advance = true);
    void insert(Addr vaddr, AlphaISA::PTE &pte);

    void flushAll();
    void flushProcesses();
    void flushAddr(Addr addr, uint8_t asn);

    // static helper functions... really EV5 VM traits
    static bool validVirtualAddress(Addr vaddr) {
        // unimplemented bits must be all 0 or all 1
        Addr unimplBits = vaddr & EV5::VAddrUnImplMask;
        return (unimplBits == 0) || (unimplBits == EV5::VAddrUnImplMask);
    }

    static void checkCacheability(MemReqPtr &req);

    // Checkpointing
    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);
};

class AlphaITB : public AlphaTLB
{
  protected:
    mutable Stats::Scalar<> hits;
    mutable Stats::Scalar<> misses;
    mutable Stats::Scalar<> acv;
    mutable Stats::Formula accesses;

  protected:
    void fault(Addr pc, ExecContext *xc) const;

  public:
    AlphaITB(const std::string &name, int size);
    virtual void regStats();

    Fault * translate(MemReqPtr &req) const;
};

class AlphaDTB : public AlphaTLB
{
  protected:
    mutable Stats::Scalar<> read_hits;
    mutable Stats::Scalar<> read_misses;
    mutable Stats::Scalar<> read_acv;
    mutable Stats::Scalar<> read_accesses;
    mutable Stats::Scalar<> write_hits;
    mutable Stats::Scalar<> write_misses;
    mutable Stats::Scalar<> write_acv;
    mutable Stats::Scalar<> write_accesses;
    Stats::Formula hits;
    Stats::Formula misses;
    Stats::Formula acv;
    Stats::Formula accesses;

  protected:
    void fault(MemReqPtr &req, uint64_t flags) const;

  public:
    AlphaDTB(const std::string &name, int size);
    virtual void regStats();

    Fault * translate(MemReqPtr &req, bool write) const;
};

#endif // __ALPHA_MEMORY_HH__
