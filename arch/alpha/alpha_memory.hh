/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#include "targetarch/mem_req.hh"
#include "sim/sim_object.hh"
#include "base/statistics.hh"

class ExecContext;

class AlphaTlb : public SimObject
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
    AlphaTlb(const std::string &name, int size);
    virtual ~AlphaTlb();

    int getsize() const { return size; }

    AlphaISA::PTE &index();
    void insert(Addr vaddr, AlphaISA::PTE &pte);

    void flushAll();
    void flushProcesses();
    void flushAddr(Addr addr, uint8_t asn);

    // static helper functions... really EV5 VM traits
    static bool validVirtualAddress(Addr vaddr) {
        // unimplemented bits must be all 0 or all 1
        Addr unimplBits = vaddr & VA_UNIMPL_MASK;
        return (unimplBits == 0) || (unimplBits == VA_UNIMPL_MASK);
    }

    static void checkCacheability(MemReqPtr req);

    // Checkpointing
    virtual void serialize();
    virtual void unserialize(IniFile &db, const std::string &category,
                             ConfigNode *node);

};

class AlphaItb : public AlphaTlb
{
  protected:
    mutable Statistics::Scalar<> hits;
    mutable Statistics::Scalar<> misses;
    mutable Statistics::Scalar<> acv;
    mutable Statistics::Formula accesses;

  protected:
    void fault(Addr pc, ExecContext *xc) const;

  public:
    AlphaItb(const std::string &name, int size);
    virtual void regStats();

    Fault translate(MemReqPtr req) const;
};

class AlphaDtb : public AlphaTlb
{
  protected:
    mutable Statistics::Scalar<> read_hits;
    mutable Statistics::Scalar<> read_misses;
    mutable Statistics::Scalar<> read_acv;
    mutable Statistics::Scalar<> read_accesses;
    mutable Statistics::Scalar<> write_hits;
    mutable Statistics::Scalar<> write_misses;
    mutable Statistics::Scalar<> write_acv;
    mutable Statistics::Scalar<> write_accesses;
    Statistics::Formula hits;
    Statistics::Formula misses;
    Statistics::Formula acv;
    Statistics::Formula accesses;

  protected:
    void fault(Addr pc, uint64_t flags, ExecContext *xc) const;

  public:
    AlphaDtb(const std::string &name, int size);
    virtual void regStats();

    Fault translate(MemReqPtr req, bool write) const;
};

#endif // __ALPHA_MEMORY_HH__
