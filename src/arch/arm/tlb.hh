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
 */

#ifndef __ARCH_ARM_TLB_HH__
#define __ARCH_ARM_TLB_HH__

#include <map>

#include "arch/arm/isa_traits.hh"
#include "arch/arm/utility.hh"
#include "arch/arm/vtophys.hh"
#include "arch/arm/pagetable.hh"
#include "base/statistics.hh"
#include "mem/request.hh"
#include "params/ArmTLB.hh"
#include "sim/faults.hh"
#include "sim/tlb.hh"

class ThreadContext;

namespace ArmISA {

class TLB : public BaseTLB
{
  public:
    enum ArmFlags {
        AlignmentMask = 0x7,

        AlignByte = 0x0,
        AlignHalfWord = 0x1,
        AlignWord = 0x3,
        AlignDoubleWord = 0x7,

        AllowUnaligned = 0x8,
        // Because zero otherwise looks like a valid setting and may be used
        // accidentally, this bit must be non-zero to show it was used on
        // purpose.
        MustBeOne = 0x10
    };
  protected:
    typedef std::multimap<Addr, int> PageTable;
    PageTable lookupTable;	// Quick lookup into page table

    ArmISA::PTE *table;	// the Page Table
    int size;			// TLB Size
    int nlu;			// not last used entry (for replacement)

    void nextnlu() { if (++nlu >= size) nlu = 0; }
    ArmISA::PTE *lookup(Addr vpn, uint8_t asn) const;

    // Access Stats
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
    Stats::Formula invalids;
    Stats::Formula accesses;

  public:
    typedef ArmTLBParams Params;
    TLB(const Params *p);

    virtual ~TLB();
    int getsize() const { return size; }

    void insert(Addr vaddr, ArmISA::PTE &pte);
    void flushAll();
    void demapPage(Addr vaddr, uint64_t asn)
    {
        panic("demapPage unimplemented.\n");
    }

    static bool validVirtualAddress(Addr vaddr);

    Fault translateAtomic(RequestPtr req, ThreadContext *tc, Mode mode);
    void translateTiming(RequestPtr req, ThreadContext *tc,
            Translation *translation, Mode mode);

    // Checkpointing
    void serialize(std::ostream &os);
    void unserialize(Checkpoint *cp, const std::string &section);

    void regStats();
};

/* namespace ArmISA */ }

#endif // __ARCH_ARM_TLB_HH__
