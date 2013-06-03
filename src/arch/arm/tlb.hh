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
 */

#ifndef __ARCH_ARM_TLB_HH__
#define __ARCH_ARM_TLB_HH__

#include <map>

#include "arch/arm/isa_traits.hh"
#include "arch/arm/pagetable.hh"
#include "arch/arm/utility.hh"
#include "arch/arm/vtophys.hh"
#include "base/statistics.hh"
#include "mem/request.hh"
#include "params/ArmTLB.hh"
#include "sim/fault_fwd.hh"
#include "sim/tlb.hh"

class ThreadContext;

namespace ArmISA {

class TableWalker;

class TLB : public BaseTLB
{
  public:
    enum ArmFlags {
        AlignmentMask = 0x1f,

        AlignByte = 0x0,
        AlignHalfWord = 0x1,
        AlignWord = 0x3,
        AlignDoubleWord = 0x7,
        AlignQuadWord = 0xf,
        AlignOctWord = 0x1f,

        AllowUnaligned = 0x20,
        // Priv code operating as if it wasn't
        UserMode = 0x40,
        // Because zero otherwise looks like a valid setting and may be used
        // accidentally, this bit must be non-zero to show it was used on
        // purpose.
        MustBeOne = 0x80
    };
  protected:

    TlbEntry *table;    // the Page Table
    int size;           // TLB Size

    uint32_t _attr;     // Memory attributes for last accessed TLB entry

    TableWalker *tableWalker;

    // Access Stats
    mutable Stats::Scalar instHits;
    mutable Stats::Scalar instMisses;
    mutable Stats::Scalar readHits;
    mutable Stats::Scalar readMisses;
    mutable Stats::Scalar writeHits;
    mutable Stats::Scalar writeMisses;
    mutable Stats::Scalar inserts;
    mutable Stats::Scalar flushTlb;
    mutable Stats::Scalar flushTlbMva;
    mutable Stats::Scalar flushTlbMvaAsid;
    mutable Stats::Scalar flushTlbAsid;
    mutable Stats::Scalar flushedEntries;
    mutable Stats::Scalar alignFaults;
    mutable Stats::Scalar prefetchFaults;
    mutable Stats::Scalar domainFaults;
    mutable Stats::Scalar permsFaults;

    Stats::Formula readAccesses;
    Stats::Formula writeAccesses;
    Stats::Formula instAccesses;
    Stats::Formula hits;
    Stats::Formula misses;
    Stats::Formula accesses;

    int rangeMRU; //On lookup, only move entries ahead when outside rangeMRU

    bool bootUncacheability;

  public:
    typedef ArmTLBParams Params;
    TLB(const Params *p);

    /** Lookup an entry in the TLB
     * @param vpn virtual address
     * @param asn context id/address space id to use
     * @param functional if the lookup should modify state
     * @return pointer to TLB entrry if it exists
     */
    TlbEntry *lookup(Addr vpn, uint8_t asn, bool functional = false);

    virtual ~TLB();
    int getsize() const { return size; }

    void insert(Addr vaddr, TlbEntry &pte);

    /** Reset the entire TLB */
    void flushAll();

    /** Remove any entries that match both a va and asn
     * @param mva virtual address to flush
     * @param asn contextid/asn to flush on match
     */
    void flushMvaAsid(Addr mva, uint64_t asn);

    /** Remove any entries that match the asn
     * @param asn contextid/asn to flush on match
     */
    void flushAsid(uint64_t asn);

    /** Remove all entries that match the va regardless of asn
     * @param mva address to flush from cache
     */
    void flushMva(Addr mva);

    Fault trickBoxCheck(RequestPtr req, Mode mode, uint8_t domain, bool sNp);
    Fault walkTrickBoxCheck(Addr pa, Addr va, Addr sz, bool is_exec,
            bool is_write, uint8_t domain, bool sNp);

    void printTlb();

    void allCpusCaching() { bootUncacheability = true; }
    void demapPage(Addr vaddr, uint64_t asn)
    {
        flushMvaAsid(vaddr, asn);
    }

    static bool validVirtualAddress(Addr vaddr);

    /**
     * Do a functional lookup on the TLB (for debugging)
     * and don't modify any internal state
     * @param tc thread context to get the context id from
     * @param vaddr virtual address to translate
     * @param pa returned physical address
     * @return if the translation was successful
     */
    bool translateFunctional(ThreadContext *tc, Addr vaddr, Addr &paddr);

    /**
     * Do a functional lookup on the TLB (for checker cpu) that
     * behaves like a normal lookup without modifying any page table state.
     */
    Fault translateFunctional(RequestPtr req, ThreadContext *tc, Mode mode);

    /** Accessor functions for memory attributes for last accessed TLB entry
     */
    void
    setAttr(uint32_t attr)
    {
        _attr = attr;
    }
    uint32_t
    getAttr() const
    {
        return _attr;
    }

    Fault translateFs(RequestPtr req, ThreadContext *tc, Mode mode,
            Translation *translation, bool &delay,
            bool timing, bool functional = false);
    Fault translateSe(RequestPtr req, ThreadContext *tc, Mode mode,
            Translation *translation, bool &delay, bool timing);
    Fault translateAtomic(RequestPtr req, ThreadContext *tc, Mode mode);
    Fault translateTiming(RequestPtr req, ThreadContext *tc,
            Translation *translation, Mode mode);
    Fault finalizePhysical(RequestPtr req, ThreadContext *tc, Mode mode) const;

    void drainResume();

    // Checkpointing
    void serialize(std::ostream &os);
    void unserialize(Checkpoint *cp, const std::string &section);

    void regStats();

    /**
     * Get the table walker master port. This is used for migrating
     * port connections during a CPU takeOverFrom() call. For
     * architectures that do not have a table walker, NULL is
     * returned, hence the use of a pointer rather than a
     * reference. For ARM this method will always return a valid port
     * pointer.
     *
     * @return A pointer to the walker master port
     */
    virtual BaseMasterPort* getMasterPort();

    // Caching misc register values here.
    // Writing to misc registers needs to invalidate them.
    // translateFunctional/translateSe/translateFs checks if they are
    // invalid and call updateMiscReg if necessary.
protected:
    SCTLR sctlr;
    bool isPriv;
    CONTEXTIDR contextId;
    PRRR prrr;
    NMRR nmrr;
    uint32_t dacr;
    bool miscRegValid;
    void updateMiscReg(ThreadContext *tc)
    {
        sctlr = tc->readMiscReg(MISCREG_SCTLR);
        CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);
        isPriv = cpsr.mode != MODE_USER;
        contextId = tc->readMiscReg(MISCREG_CONTEXTIDR);
        prrr = tc->readMiscReg(MISCREG_PRRR);
        nmrr = tc->readMiscReg(MISCREG_NMRR);
        dacr = tc->readMiscReg(MISCREG_DACR);
        miscRegValid = true;
    }
public:
    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }
    inline void invalidateMiscReg() { miscRegValid = false; }
};

} // namespace ArmISA

#endif // __ARCH_ARM_TLB_HH__
