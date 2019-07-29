/*
 * Copyright (c) 2010-2013, 2016, 2019 ARM Limited
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


#include "arch/arm/isa_traits.hh"
#include "arch/arm/pagetable.hh"
#include "arch/arm/utility.hh"
#include "arch/arm/vtophys.hh"
#include "arch/generic/tlb.hh"
#include "base/statistics.hh"
#include "mem/request.hh"
#include "params/ArmTLB.hh"
#include "sim/probe/pmu.hh"

class ThreadContext;

namespace ArmISA {

class TableWalker;
class Stage2LookUp;
class Stage2MMU;
class TLB;

class TlbTestInterface
{
  public:
    TlbTestInterface() {}
    virtual ~TlbTestInterface() {}

    /**
     * Check if a TLB translation should be forced to fail.
     *
     * @param req Request requiring a translation.
     * @param is_priv Access from a privileged mode (i.e., not EL0)
     * @param mode Access type
     * @param domain Domain type
     */
    virtual Fault translationCheck(const RequestPtr &req, bool is_priv,
                                   BaseTLB::Mode mode,
                                   TlbEntry::DomainType domain) = 0;

    /**
     * Check if a page table walker access should be forced to fail.
     *
     * @param pa Physical address the walker is accessing
     * @param size Walker access size
     * @param va Virtual address that initiated the walk
     * @param is_secure Access from secure state
     * @param is_priv Access from a privileged mode (i.e., not EL0)
     * @param mode Access type
     * @param domain Domain type
     * @param lookup_level Page table walker level
     */
    virtual Fault walkCheck(Addr pa, Addr size, Addr va, bool is_secure,
                            Addr is_priv, BaseTLB::Mode mode,
                            TlbEntry::DomainType domain,
                            LookupLevel lookup_level) = 0;
};

class TLB : public BaseTLB
{
  public:
    enum ArmFlags {
        AlignmentMask = 0x7,

        AlignByte = 0x0,
        AlignHalfWord = 0x1,
        AlignWord = 0x2,
        AlignDoubleWord = 0x3,
        AlignQuadWord = 0x4,
        AlignOctWord = 0x5,

        AllowUnaligned = 0x8,
        // Priv code operating as if it wasn't
        UserMode = 0x10,
        // Because zero otherwise looks like a valid setting and may be used
        // accidentally, this bit must be non-zero to show it was used on
        // purpose.
        MustBeOne = 0x40
    };

    enum ArmTranslationType {
        NormalTran = 0,
        S1CTran = 0x1,
        HypMode = 0x2,
        // Secure code operating as if it wasn't (required by some Address
        // Translate operations)
        S1S2NsTran = 0x4,
        // Address translation instructions (eg AT S1E0R_Xt) need to be handled
        // in special ways during translation because they could need to act
        // like a different EL than the current EL. The following flags are
        // for these instructions
        S1E0Tran = 0x8,
        S1E1Tran = 0x10,
        S1E2Tran = 0x20,
        S1E3Tran = 0x40,
        S12E0Tran = 0x80,
        S12E1Tran = 0x100
    };

    /**
     * Determine the EL to use for the purpose of a translation given
     * a specific translation type. If the translation type doesn't
     * specify an EL, we use the current EL.
     */
    static ExceptionLevel tranTypeEL(CPSR cpsr, ArmTranslationType type);

  protected:
    TlbEntry* table;     // the Page Table
    int size;            // TLB Size
    bool isStage2;       // Indicates this TLB is part of the second stage MMU
    bool stage2Req;      // Indicates whether a stage 2 lookup is also required
    // Indicates whether a stage 2 lookup of the table descriptors is required.
    // Certain address translation instructions will intercept the IPA but the
    // table descriptors still need to be translated by the stage2.
    bool stage2DescReq;
    uint64_t _attr;      // Memory attributes for last accessed TLB entry
    bool directToStage2; // Indicates whether all translation requests should
                         // be routed directly to the stage 2 TLB

    TableWalker *tableWalker;
    TLB *stage2Tlb;
    Stage2MMU *stage2Mmu;

    TlbTestInterface *test;

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

    /** PMU probe for TLB refills */
    ProbePoints::PMUUPtr ppRefills;

    int rangeMRU; //On lookup, only move entries ahead when outside rangeMRU

  public:
    TLB(const ArmTLBParams *p);
    TLB(const Params *p, int _size, TableWalker *_walker);

    /** Lookup an entry in the TLB
     * @param vpn virtual address
     * @param asn context id/address space id to use
     * @param vmid The virtual machine ID used for stage 2 translation
     * @param secure if the lookup is secure
     * @param hyp if the lookup is done from hyp mode
     * @param functional if the lookup should modify state
     * @param ignore_asn if on lookup asn should be ignored
     * @return pointer to TLB entry if it exists
     */
    TlbEntry *lookup(Addr vpn, uint16_t asn, uint8_t vmid, bool hyp,
                     bool secure, bool functional,
                     bool ignore_asn, ExceptionLevel target_el);

    virtual ~TLB();

    void takeOverFrom(BaseTLB *otlb) override;

    /// setup all the back pointers
    void init() override;

    void setTestInterface(SimObject *ti);

    TableWalker *getTableWalker() { return tableWalker; }

    void setMMU(Stage2MMU *m, MasterID master_id);

    int getsize() const { return size; }

    void insert(Addr vaddr, TlbEntry &pte);

    Fault getTE(TlbEntry **te, const RequestPtr &req,
                ThreadContext *tc, Mode mode,
                Translation *translation, bool timing, bool functional,
                bool is_secure, ArmTranslationType tranType);

    Fault getResultTe(TlbEntry **te, const RequestPtr &req,
                      ThreadContext *tc, Mode mode,
                      Translation *translation, bool timing,
                      bool functional, TlbEntry *mergeTe);

    Fault checkPermissions(TlbEntry *te, const RequestPtr &req, Mode mode);
    Fault checkPermissions64(TlbEntry *te, const RequestPtr &req, Mode mode,
                             ThreadContext *tc);
    bool checkPAN(ThreadContext *tc, uint8_t ap, const RequestPtr &req,
                  Mode mode);


    /** Reset the entire TLB
     * @param secure_lookup if the operation affects the secure world
     */
    void flushAllSecurity(bool secure_lookup, ExceptionLevel target_el,
                          bool ignore_el = false);

    /** Remove all entries in the non secure world, depending on whether they
     *  were allocated in hyp mode or not
     */
    void flushAllNs(ExceptionLevel target_el, bool ignore_el = false);


    /** Reset the entire TLB. Used for CPU switching to prevent stale
     * translations after multiple switches
     */
    void flushAll() override
    {
        flushAllSecurity(false, EL0, true);
        flushAllSecurity(true, EL0, true);
    }

    /** Remove any entries that match both a va and asn
     * @param mva virtual address to flush
     * @param asn contextid/asn to flush on match
     * @param secure_lookup if the operation affects the secure world
     */
    void flushMvaAsid(Addr mva, uint64_t asn, bool secure_lookup,
                      ExceptionLevel target_el);

    /** Remove any entries that match the asn
     * @param asn contextid/asn to flush on match
     * @param secure_lookup if the operation affects the secure world
     */
    void flushAsid(uint64_t asn, bool secure_lookup,
                   ExceptionLevel target_el);

    /** Remove all entries that match the va regardless of asn
     * @param mva address to flush from cache
     * @param secure_lookup if the operation affects the secure world
     */
    void flushMva(Addr mva, bool secure_lookup, ExceptionLevel target_el);

    /**
     * Invalidate all entries in the stage 2 TLB that match the given ipa
     * and the current VMID
     * @param ipa the address to invalidate
     * @param secure_lookup if the operation affects the secure world
     */
    void flushIpaVmid(Addr ipa, bool secure_lookup, ExceptionLevel target_el);

    Fault trickBoxCheck(const RequestPtr &req, Mode mode,
                        TlbEntry::DomainType domain);

    Fault walkTrickBoxCheck(Addr pa, bool is_secure, Addr va, Addr sz,
                            bool is_exec, bool is_write,
                            TlbEntry::DomainType domain,
                            LookupLevel lookup_level);

    void printTlb() const;

    void demapPage(Addr vaddr, uint64_t asn) override
    {
        // needed for x86 only
        panic("demapPage() is not implemented.\n");
    }

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
    Fault translateFunctional(const RequestPtr &req, ThreadContext *tc,
            Mode mode, ArmTranslationType tranType);
    Fault
    translateFunctional(const RequestPtr &req,
                        ThreadContext *tc, Mode mode) override
    {
        return translateFunctional(req, tc, mode, NormalTran);
    }

    /** Accessor functions for memory attributes for last accessed TLB entry
     */
    void
    setAttr(uint64_t attr)
    {
        _attr = attr;
    }

    uint64_t
    getAttr() const
    {
        return _attr;
    }

    Fault translateFs(const RequestPtr &req, ThreadContext *tc, Mode mode,
            Translation *translation, bool &delay,
            bool timing, ArmTranslationType tranType, bool functional = false);
    Fault translateSe(const RequestPtr &req, ThreadContext *tc, Mode mode,
            Translation *translation, bool &delay, bool timing);
    Fault translateAtomic(const RequestPtr &req, ThreadContext *tc, Mode mode,
            ArmTranslationType tranType);
    Fault
    translateAtomic(const RequestPtr &req,
                    ThreadContext *tc, Mode mode) override
    {
        return translateAtomic(req, tc, mode, NormalTran);
    }
    void translateTiming(
            const RequestPtr &req, ThreadContext *tc,
            Translation *translation, Mode mode,
            ArmTranslationType tranType);
    void
    translateTiming(const RequestPtr &req, ThreadContext *tc,
                    Translation *translation, Mode mode) override
    {
        translateTiming(req, tc, translation, mode, NormalTran);
    }
    Fault translateComplete(const RequestPtr &req, ThreadContext *tc,
            Translation *translation, Mode mode, ArmTranslationType tranType,
            bool callFromS2);
    Fault finalizePhysical(
            const RequestPtr &req,
            ThreadContext *tc, Mode mode) const override;

    void drainResume() override;

    // Checkpointing
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    void regStats() override;

    void regProbePoints() override;

    /**
     * Get the table walker port. This is used for migrating
     * port connections during a CPU takeOverFrom() call. For
     * architectures that do not have a table walker, NULL is
     * returned, hence the use of a pointer rather than a
     * reference. For ARM this method will always return a valid port
     * pointer.
     *
     * @return A pointer to the walker master port
     */
    Port *getTableWalkerPort() override;

    // Caching misc register values here.
    // Writing to misc registers needs to invalidate them.
    // translateFunctional/translateSe/translateFs checks if they are
    // invalid and call updateMiscReg if necessary.
protected:
    CPSR cpsr;
    bool aarch64;
    ExceptionLevel aarch64EL;
    SCTLR sctlr;
    SCR scr;
    bool isPriv;
    bool isSecure;
    bool isHyp;
    TTBCR ttbcr;
    uint16_t asid;
    uint8_t vmid;
    PRRR prrr;
    NMRR nmrr;
    HCR hcr;
    uint32_t dacr;
    bool miscRegValid;
    ContextID miscRegContext;
    ArmTranslationType curTranType;

    // Cached copies of system-level properties
    bool haveLPAE;
    bool haveVirtualization;
    bool haveLargeAsid64;

    AddrRange m5opRange;

    void updateMiscReg(ThreadContext *tc,
                       ArmTranslationType tranType = NormalTran);

public:
    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }
    inline void invalidateMiscReg() { miscRegValid = false; }

private:
    /** Remove any entries that match both a va and asn
     * @param mva virtual address to flush
     * @param asn contextid/asn to flush on match
     * @param secure_lookup if the operation affects the secure world
     * @param ignore_asn if the flush should ignore the asn
     */
    void _flushMva(Addr mva, uint64_t asn, bool secure_lookup,
                   bool ignore_asn, ExceptionLevel target_el);

  public: /* Testing */
    Fault testTranslation(const RequestPtr &req, Mode mode,
                          TlbEntry::DomainType domain);
    Fault testWalk(Addr pa, Addr size, Addr va, bool is_secure, Mode mode,
                   TlbEntry::DomainType domain,
                   LookupLevel lookup_level);
};

template<typename T>
TLB *
getITBPtr(T *tc)
{
    auto tlb = static_cast<TLB *>(tc->getITBPtr());
    assert(tlb);
    return tlb;
}

template<typename T>
TLB *
getDTBPtr(T *tc)
{
    auto tlb = static_cast<TLB *>(tc->getDTBPtr());
    assert(tlb);
    return tlb;
}

} // namespace ArmISA

#endif // __ARCH_ARM_TLB_HH__
