/*
 * Copyright (c) 2018-2020, 2022-2024 Arm Limited
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

#ifndef __ARCH_ARM_TLBI_HH__
#define __ARCH_ARM_TLBI_HH__

#include "arch/arm/system.hh"
#include "arch/arm/tlb.hh"
#include "cpu/thread_context.hh"

/**
 * @file
 * The file contains the definition of a set of TLB Invalidate
 * Instructions. Those are the ISA interface for TLB flushing
 * operations.
 */

namespace gem5
{

namespace ArmISA {

class TLBIOp
{
  public:
    TLBIOp(TranslationRegime _target_regime, bool _secure)
      : secureLookup(_secure), targetRegime(_target_regime)
    {}

    virtual ~TLBIOp() {}
    virtual void operator()(ThreadContext* tc) {}

    /**
     * Broadcast the TLB Invalidate operation to all
     * TLBs in the Arm system.
     * @param tc Thread Context
     */
    void
    broadcast(ThreadContext *tc)
    {
        for (auto *oc: tc->getSystemPtr()->threads)
            (*this)(oc);
    }

    virtual bool match(TlbEntry *entry, vmid_t curr_vmid) const = 0;

    /**
     * Return true if the TLBI op needs to flush stage1
     * entries, Defaulting to true in the TLBIOp abstract
     * class
     */
    virtual bool
    stage1Flush() const
    {
        return true;
    }

    /**
     * Return true if the TLBI op needs to flush stage2
     * entries, Defaulting to false in the TLBIOp abstract
     * class
     */
    virtual bool
    stage2Flush() const
    {
        return false;
    }

    bool secureLookup;
    TranslationRegime targetRegime;
};

/** TLB Invalidate All */
class TLBIALL : public TLBIOp
{
  public:
    TLBIALL(TranslationRegime _target_regime, bool _secure)
      : TLBIOp(_target_regime, _secure), el2Enabled(false),
        currentEL(EL0)
    {}

    void operator()(ThreadContext* tc) override;

    bool match(TlbEntry *entry, vmid_t curr_vmid) const override;

    bool
    stage2Flush() const override
    {
        // TLBIALL (AArch32) flushing stage2 entries if we're currently
        // in hyp mode
        return currentEL == EL2;
    }

    TLBIALL
    makeStage2() const
    {
        return TLBIALL(targetRegime, secureLookup);
    }

    bool el2Enabled;
    ExceptionLevel currentEL;
};

/** Instruction TLB Invalidate All */
class ITLBIALL : public TLBIALL
{
  public:
    ITLBIALL(TranslationRegime _target_regime, bool _secure)
      : TLBIALL(_target_regime, _secure)
    {}

    void operator()(ThreadContext* tc) override;

    bool match(TlbEntry *entry, vmid_t curr_vmid) const override;
};

/** Data TLB Invalidate All */
class DTLBIALL : public TLBIALL
{
  public:
    DTLBIALL(TranslationRegime _target_regime, bool _secure)
      : TLBIALL(_target_regime, _secure)
    {}

    void operator()(ThreadContext* tc) override;

    bool match(TlbEntry *entry, vmid_t curr_vmid) const override;
};

/** Implementaton of AArch64 TLBI ALLE(1,2,3)(IS) instructions */
class TLBIALLEL : public TLBIOp
{
  public:
    TLBIALLEL(TranslationRegime _target_regime, bool _secure)
      : TLBIOp(_target_regime, _secure)
    {}

    void operator()(ThreadContext* tc) override;

    bool match(TlbEntry *entry, vmid_t curr_vmid) const override;

    bool
    stage2Flush() const override
    {
        // If we're targeting EL1 then flush stage2 as well
        return targetRegime == TranslationRegime::EL10 ||
               targetRegime == TranslationRegime::EL20;
    }

    TLBIALLEL
    makeStage2() const
    {
        return TLBIALLEL(targetRegime, secureLookup);
    }

};

/** Implementaton of AArch64 TLBI VMALLE1(IS)/VMALLS112E1(IS) instructions */
class TLBIVMALL : public TLBIOp
{
  public:
    TLBIVMALL(TranslationRegime _target_regime, bool _secure, bool _stage2)
      : TLBIOp(_target_regime, _secure), el2Enabled(false),
        stage2(_stage2)
    {}

    void operator()(ThreadContext* tc) override;

    bool match(TlbEntry *entry, vmid_t curr_vmid) const override;

    bool
    stage2Flush() const override
    {
        return stage2;
    }

    TLBIVMALL
    makeStage2() const
    {
        return TLBIVMALL(targetRegime, secureLookup, false);
    }

    bool el2Enabled;
    bool stage2;
};

/** TLB Invalidate by ASID match */
class TLBIASID : public TLBIOp
{
  public:
    TLBIASID(TranslationRegime _target_regime, bool _secure, uint16_t _asid)
      : TLBIOp(_target_regime, _secure), asid(_asid),
        el2Enabled(false)
    {}

    void operator()(ThreadContext* tc) override;

    bool match(TlbEntry *entry, vmid_t curr_vmid) const override;

    uint16_t asid;
    bool el2Enabled;
};

/** Instruction TLB Invalidate by ASID match */
class ITLBIASID : public TLBIASID
{
  public:
    ITLBIASID(TranslationRegime _target_regime, bool _secure, uint16_t _asid)
      : TLBIASID(_target_regime, _secure, _asid)
    {}

    void operator()(ThreadContext* tc) override;

    bool match(TlbEntry *entry, vmid_t curr_vmid) const override;
};

/** Data TLB Invalidate by ASID match */
class DTLBIASID : public TLBIASID
{
  public:
    DTLBIASID(TranslationRegime _target_regime, bool _secure, uint16_t _asid)
      : TLBIASID(_target_regime, _secure, _asid)
    {}

    void operator()(ThreadContext* tc) override;

    bool match(TlbEntry *entry, vmid_t curr_vmid) const override;
};

/** TLB Invalidate All, Non-Secure */
class TLBIALLN : public TLBIOp
{
  public:
    TLBIALLN(TranslationRegime _target_regime)
      : TLBIOp(_target_regime, false)
    {}

    void operator()(ThreadContext* tc) override;

    bool match(TlbEntry *entry, vmid_t curr_vmid) const override;

    bool
    stage2Flush() const override
    {
        return targetRegime != TranslationRegime::EL2;
    }

    TLBIALLN
    makeStage2() const
    {
        return TLBIALLN(targetRegime);
    }
};

/** TLB Invalidate by VA, All ASID */
class TLBIMVAA : public TLBIOp
{
  protected:
    TlbEntry::Lookup lookupGen(vmid_t vmid) const;
  public:
    TLBIMVAA(TranslationRegime _target_regime, bool _secure,
             Addr _addr, bool last_level)
      : TLBIOp(_target_regime, _secure), addr(_addr),
        lastLevel(last_level)
    {}

    void operator()(ThreadContext* tc) override;

    bool match(TlbEntry *entry, vmid_t curr_vmid) const override;

    Addr addr;
    bool lastLevel;
};

/** TLB Invalidate by VA */
class TLBIMVA : public TLBIOp
{
  protected:
    TlbEntry::Lookup lookupGen(vmid_t vmid) const;

  public:
    TLBIMVA(TranslationRegime _target_regime, bool _secure,
            Addr _addr, uint16_t _asid, bool last_level)
      : TLBIOp(_target_regime, _secure), addr(_addr), asid(_asid),
        lastLevel(last_level)
    {}

    void operator()(ThreadContext* tc) override;

    bool match(TlbEntry *entry, vmid_t curr_vmid) const override;

    Addr addr;
    uint16_t asid;
    bool lastLevel;
};

/** Instruction TLB Invalidate by VA */
class ITLBIMVA : public TLBIMVA
{
  public:
    ITLBIMVA(TranslationRegime _target_regime, bool _secure,
             Addr _addr, uint16_t _asid)
      : TLBIMVA(_target_regime, _secure, _addr, _asid, false)
    {}

    void operator()(ThreadContext* tc) override;

    bool match(TlbEntry *entry, vmid_t curr_vmid) const override;
};

/** Data TLB Invalidate by VA */
class DTLBIMVA : public TLBIMVA
{
  public:
    DTLBIMVA(TranslationRegime _target_regime, bool _secure,
             Addr _addr, uint16_t _asid)
      : TLBIMVA(_target_regime, _secure, _addr, _asid, false)
    {}

    void operator()(ThreadContext* tc) override;

    bool match(TlbEntry *entry, vmid_t curr_vmid) const override;
};

class TLBIRange
{
  public:
    /**
     * Is the range valid? This mainly depends on the specified
     * translation granule.
     */
    bool valid() const { return granule != ReservedGrain; }

  protected:
    BitUnion64(RangeData)
        Bitfield<47, 46> tg;
        Bitfield<45, 44> scale;
        Bitfield<43, 39> num;
        Bitfield<38, 37> ttl;
        Bitfield<36, 0> baseAddr;
    EndBitUnion(RangeData)

    static constexpr std::array<GrainSize, 4> tgMap = {
        ReservedGrain,
        Grain4KB,
        Grain16KB,
        Grain64KB
    };

    TLBIRange(RegVal val)
      : rangeData(val), granule(tgMap[rangeData.tg])
    {}

    Addr
    startAddress() const
    {
        return sext<37>(rangeData.baseAddr) << granule;
    }

    Addr
    rangeSize() const
    {
        return (rangeData.num + 1) << (5 * rangeData.scale + 1 + granule);
    }

    bool
    resTLBIttl(uint8_t tg, uint8_t ttl) const
    {
        switch (ttl) {
          case 0: return true;
          case 1: return tgMap[tg] == Grain16KB;
          default: return false;
        }
    }

    RangeData rangeData;
    GrainSize granule;
};

/** TLB Invalidate by Intermediate Physical Address */
class TLBIIPA : public TLBIOp
{
  public:
    TLBIIPA(TranslationRegime _target_regime, bool _secure, Addr _addr,
            bool last_level)
      : TLBIOp(_target_regime, _secure), addr(_addr), lastLevel(last_level)
    {}

    void operator()(ThreadContext* tc) override;

    bool
    match(TlbEntry *entry, vmid_t curr_vmid) const override
    {
        panic("This shouldn't be called\n");
    }

    bool
    stage1Flush() const override
    {
        return false;
    }

    /** TLBIIPA is basically a TLBIMVAA for stage2 TLBs */
    virtual TLBIMVAA
    makeStage2() const
    {
        return TLBIMVAA(targetRegime, secureLookup, addr, lastLevel);
    }

    Addr addr;
    bool lastLevel;
};

/** TLB Range Invalidate by VA */
class TLBIRMVA : public TLBIRange, public TLBIMVA
{
  public:
    TLBIRMVA(TranslationRegime _target_regime, bool _secure,
             RegVal val, uint16_t _asid, bool last_level)
      : TLBIRange(val),
        TLBIMVA(_target_regime, _secure, startAddress(), _asid, last_level)
    {}

    bool match(TlbEntry *entry, vmid_t curr_vmid) const override;
};

/** TLB Range Invalidate by VA, All ASIDs */
class TLBIRMVAA : public TLBIRange, public TLBIMVAA
{
  public:
    TLBIRMVAA(TranslationRegime _target_regime, bool _secure,
              RegVal val, bool last_level)
      : TLBIRange(val),
        TLBIMVAA(_target_regime, _secure, startAddress(), last_level)
    {}

    bool match(TlbEntry *entry, vmid_t curr_vmid) const override;
};

/** TLB Range Invalidate by VA, All ASIDs */
class TLBIRIPA : public TLBIRange, public TLBIIPA
{
  public:
    TLBIRIPA(TranslationRegime _target_regime, bool _secure,
             RegVal val, bool last_level)
      : TLBIRange(val),
        TLBIIPA(_target_regime, _secure, startAddress(), last_level)
    {}

    virtual TLBIMVAA
    makeStage2() const
    {
        return TLBIRMVAA(targetRegime, secureLookup, rangeData, lastLevel);
    }
};

} // namespace ArmISA
} // namespace gem5

#endif //__ARCH_ARM_TLBI_HH__
