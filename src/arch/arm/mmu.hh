/*
 * Copyright (c) 2010-2013, 2016, 2019-2021 Arm Limited
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
 */

#ifndef __ARCH_ARM_MMU_HH__
#define __ARCH_ARM_MMU_HH__

#include "arch/arm/page_size.hh"
#include "arch/arm/tlb.hh"
#include "arch/generic/mmu.hh"

#include "enums/ArmLookupLevel.hh"

#include "params/ArmMMU.hh"

namespace gem5
{

namespace ArmISA {

class TableWalker;

class MMU : public BaseMMU
{
  protected:
    using LookupLevel = enums::ArmLookupLevel;

    ArmISA::TLB *
    getDTBPtr() const
    {
        return static_cast<ArmISA::TLB *>(dtb);
    }

    ArmISA::TLB *
    getITBPtr() const
    {
        return static_cast<ArmISA::TLB *>(itb);
    }

    TLB * getTlb(BaseMMU::Mode mode, bool stage2) const;
    TableWalker * getTableWalker(BaseMMU::Mode mode, bool stage2) const;

  protected:
    TLB *itbStage2;
    TLB *dtbStage2;

    TableWalker *itbWalker;
    TableWalker *dtbWalker;
    TableWalker *itbStage2Walker;
    TableWalker *dtbStage2Walker;

  public:
    TranslationGenPtr
    translateFunctional(Addr start, Addr size, ThreadContext *tc,
            Mode mode, Request::Flags flags) override
    {
        return TranslationGenPtr(new MMUTranslationGen(
                PageBytes, start, size, tc, this, mode, flags));
    }

    enum ArmFlags
    {
        AlignmentMask = 0x7,

        AlignByte = 0x0,
        AlignHalfWord = 0x1,
        AlignWord = 0x2,
        AlignDoubleWord = 0x3,
        AlignQuadWord = 0x4,
        AlignOctWord = 0x5,

        AllowUnaligned = 0x8,
        // Priv code operating as if it wasn't
        UserMode = 0x10
    };

    enum ArmTranslationType
    {
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

    struct CachedState {
        explicit CachedState(MMU *_mmu, bool stage2)
          : mmu(_mmu), isStage2(stage2)
        {}

        void updateMiscReg(ThreadContext *tc, ArmTranslationType tran_type);

        /** Returns the current VMID
         * (information stored in the VTTBR_EL2 register) */
        vmid_t getVMID(ThreadContext *tc) const;

        MMU *mmu;
        bool isStage2 = false;
        CPSR cpsr = 0;
        bool aarch64 = false;
        ExceptionLevel aarch64EL = EL0;
        SCTLR sctlr = 0;
        SCR scr = 0;
        bool isPriv = false;
        bool isSecure = false;
        bool isHyp = false;
        TTBCR ttbcr = 0;
        uint16_t asid = 0;
        vmid_t vmid = 0;
        PRRR prrr = 0;
        NMRR nmrr = 0;
        HCR hcr = 0;
        uint32_t dacr = 0;
        bool miscRegValid = false;
        ArmTranslationType curTranType = NormalTran;

        // Indicates whether a stage 2 lookup is also required
        bool stage2Req = false;

        // Indicates whether a stage 2 lookup of the table descriptors is
        // required.  Certain address translation instructions will
        // intercept the IPA but the table descriptors still need to be
        // translated by the stage2.
        bool stage2DescReq = false;

        // Indicates whether all translation requests should
        // be routed directly to the stage 2 TLB
        bool directToStage2 = false;
    };

    MMU(const ArmMMUParams &p);

    void init() override;

    using BaseMMU::translateFunctional;

    /**
     * Do a functional lookup on the TLB (for debugging)
     * and don't modify any internal state
     * @param tc thread context to get the context id from
     * @param vaddr virtual address to translate
     * @param pa returned physical address
     * @return if the translation was successful
     */
    bool translateFunctional(ThreadContext *tc, Addr vaddr, Addr &paddr);

    Fault translateFunctional(const RequestPtr &req, ThreadContext *tc,
        BaseMMU::Mode mode) override;

    /**
     * Do a functional lookup on the TLB (for checker cpu) that
     * behaves like a normal lookup without modifying any page table state.
     */
    Fault translateFunctional(const RequestPtr &req, ThreadContext *tc,
        BaseMMU::Mode mode, ArmTranslationType tran_type);

    Fault translateFunctional(const RequestPtr &req, ThreadContext *tc,
        BaseMMU::Mode mode, ArmTranslationType tran_type, bool stage2);

    Fault
    translateAtomic(const RequestPtr &req,
                    ThreadContext *tc, Mode mode) override
    {
        return translateAtomic(req, tc, mode, NormalTran);
    }
    Fault translateAtomic(const RequestPtr &req, ThreadContext *tc,
        BaseMMU::Mode mode, ArmTranslationType tran_type, bool stage2);
    Fault translateAtomic(const RequestPtr &req, ThreadContext *tc, Mode mode,
        ArmTranslationType tran_type);

    void
    translateTiming(const RequestPtr &req, ThreadContext *tc,
                    Translation *translation, Mode mode) override
    {
        translateTiming(req, tc, translation, mode, NormalTran, false);
    }
    void translateTiming(const RequestPtr &req, ThreadContext *tc,
        BaseMMU::Translation *translation, BaseMMU::Mode mode, bool stage2);
    void translateTiming(
            const RequestPtr &req, ThreadContext *tc,
            Translation *translation, Mode mode,
            ArmTranslationType tran_type, bool stage2);

    Fault translateMmuOff(ThreadContext *tc, const RequestPtr &req, Mode mode,
        ArmTranslationType tran_type, Addr vaddr, bool long_desc_format,
        CachedState &state);
    Fault translateMmuOn(ThreadContext *tc, const RequestPtr &req, Mode mode,
        Translation *translation, bool &delay, bool timing, bool functional,
        Addr vaddr, ArmFault::TranMethod tranMethod,
        CachedState &state);

    Fault translateFs(const RequestPtr &req, ThreadContext *tc, Mode mode,
            Translation *translation, bool &delay,
            bool timing, ArmTranslationType tran_type, bool functional,
            CachedState &state);
    Fault translateSe(const RequestPtr &req, ThreadContext *tc, Mode mode,
            Translation *translation, bool &delay, bool timing,
            CachedState &state);

    Fault translateComplete(const RequestPtr &req, ThreadContext *tc,
            Translation *translation, Mode mode, ArmTranslationType tran_type,
            bool call_from_s2);
    Fault translateComplete(const RequestPtr &req, ThreadContext *tc,
            Translation *translation, Mode mode, ArmTranslationType tran_type,
            bool call_from_s2, CachedState &state);
    Fault finalizePhysical(
            const RequestPtr &req,
            ThreadContext *tc, Mode mode) const override;

    void drainResume() override;

    void takeOverFrom(BaseMMU *old_mmu) override;

    void invalidateMiscReg();

    template <typename OP>
    void
    flush(const OP &tlbi_op)
    {
        if (tlbi_op.stage1Flush()) {
            flushStage1(tlbi_op);
        }

        if (tlbi_op.stage2Flush()) {
            flushStage2(tlbi_op.makeStage2());
        }
    }

    template <typename OP>
    void
    flushStage1(const OP &tlbi_op)
    {
        for (auto tlb : instruction) {
            static_cast<TLB*>(tlb)->flush(tlbi_op);
        }
        for (auto tlb : data) {
            static_cast<TLB*>(tlb)->flush(tlbi_op);
        }
        for (auto tlb : unified) {
            static_cast<TLB*>(tlb)->flush(tlbi_op);
        }
    }

    template <typename OP>
    void
    flushStage2(const OP &tlbi_op)
    {
        itbStage2->flush(tlbi_op);
        dtbStage2->flush(tlbi_op);
    }

    template <typename OP>
    void
    iflush(const OP &tlbi_op)
    {
        for (auto tlb : instruction) {
            static_cast<TLB*>(tlb)->flush(tlbi_op);
        }
        for (auto tlb : unified) {
            static_cast<TLB*>(tlb)->flush(tlbi_op);
        }
    }

    template <typename OP>
    void
    dflush(const OP &tlbi_op)
    {
        for (auto tlb : data) {
            static_cast<TLB*>(tlb)->flush(tlbi_op);
        }
        for (auto tlb : unified) {
            static_cast<TLB*>(tlb)->flush(tlbi_op);
        }
    }

    void
    flushAll() override
    {
        BaseMMU::flushAll();
        itbStage2->flushAll();
        dtbStage2->flushAll();
    }

    uint64_t
    getAttr() const
    {
        return _attr;
    }

    /** Accessor functions for memory attributes for last accessed TLB entry
     */
    void
    setAttr(uint64_t attr)
    {
        _attr = attr;
    }

    /**
     * Determine the EL to use for the purpose of a translation given
     * a specific translation type. If the translation type doesn't
     * specify an EL, we use the current EL.
     */
    static ExceptionLevel tranTypeEL(CPSR cpsr, ArmTranslationType type);

  public:
    /** Lookup an entry in the TLB
     * @param vpn virtual address
     * @param asn context id/address space id to use
     * @param vmid The virtual machine ID used for stage 2 translation
     * @param secure if the lookup is secure
     * @param hyp if the lookup is done from hyp mode
     * @param functional if the lookup should modify state
     * @param ignore_asn if on lookup asn should be ignored
     * @param target_el selecting the translation regime
     * @param in_host if we are in host (EL2&0 regime)
     * @param mode to differentiate between read/writes/fetches.
     * @return pointer to TLB entry if it exists
     */
    TlbEntry *lookup(Addr vpn, uint16_t asn, vmid_t vmid, bool hyp,
                     bool secure, bool functional,
                     bool ignore_asn, ExceptionLevel target_el,
                     bool in_host, bool stage2, BaseMMU::Mode mode);

    Fault getTE(TlbEntry **te, const RequestPtr &req,
                ThreadContext *tc, Mode mode,
                Translation *translation, bool timing, bool functional,
                bool is_secure, ArmTranslationType tran_type,
                bool stage2);
    Fault getTE(TlbEntry **te, const RequestPtr &req,
                ThreadContext *tc, Mode mode,
                Translation *translation, bool timing, bool functional,
                bool is_secure, ArmTranslationType tran_type,
                CachedState &state);

    Fault getResultTe(TlbEntry **te, const RequestPtr &req,
                      ThreadContext *tc, Mode mode,
                      Translation *translation, bool timing,
                      bool functional, TlbEntry *mergeTe,
                      CachedState &state);

    Fault checkPermissions(TlbEntry *te, const RequestPtr &req, Mode mode,
                           bool stage2);
    Fault checkPermissions(TlbEntry *te, const RequestPtr &req, Mode mode,
                           CachedState &state);
    Fault checkPermissions64(TlbEntry *te, const RequestPtr &req, Mode mode,
                             ThreadContext *tc, bool stage2);
    Fault checkPermissions64(TlbEntry *te, const RequestPtr &req, Mode mode,
                             ThreadContext *tc, CachedState &state);
  protected:
    bool checkPAN(ThreadContext *tc, uint8_t ap, const RequestPtr &req,
                  Mode mode, const bool is_priv, CachedState &state);

    bool faultPAN(ThreadContext *tc, uint8_t ap, const RequestPtr &req,
                  Mode mode, const bool is_priv, CachedState &state);

    std::pair<bool, bool> s1PermBits64(
        TlbEntry *te, const RequestPtr &req, Mode mode,
        ThreadContext *tc, CachedState &state, bool r, bool w, bool x);

    std::pair<bool, bool> s2PermBits64(
        TlbEntry *te, const RequestPtr &req, Mode mode,
        ThreadContext *tc, CachedState &state, bool r, bool w, bool x);

  public: /* Testing */
    TlbTestInterface *test;

    void setTestInterface(SimObject *ti);

    Fault testTranslation(const RequestPtr &req, Mode mode,
                          TlbEntry::DomainType domain, CachedState &state);
    Fault testWalk(Addr pa, Addr size, Addr va, bool is_secure, Mode mode,
                   TlbEntry::DomainType domain,
                   LookupLevel lookup_level, bool stage2);
    Fault testWalk(Addr pa, Addr size, Addr va, bool is_secure, Mode mode,
                   TlbEntry::DomainType domain,
                   LookupLevel lookup_level, CachedState &state);

  protected:
    ContextID miscRegContext;

  public:
    CachedState s1State, s2State;

  protected:
    uint64_t _attr;      // Memory attributes for last accessed TLB entry

    // Cached copies of system-level properties
    bool haveLPAE;
    bool haveVirtualization;
    bool haveLargeAsid64;
    uint8_t physAddrRange;

    AddrRange m5opRange;

    CachedState& updateMiscReg(
        ThreadContext *tc, ArmTranslationType tran_type,
        bool stage2);

    struct Stats : public statistics::Group
    {
        Stats(statistics::Group *parent);
        // Access Stats
        mutable statistics::Scalar alignFaults;
        mutable statistics::Scalar prefetchFaults;
        mutable statistics::Scalar domainFaults;
        mutable statistics::Scalar permsFaults;
    } stats;

};

template<typename T>
MMU *
getMMUPtr(T *tc)
{
    auto mmu = static_cast<MMU *>(tc->getMMUPtr());
    assert(mmu);
    return mmu;
}

} // namespace ArmISA
} // namespace gem5

#endif // __ARCH_ARM_MMU_HH__
