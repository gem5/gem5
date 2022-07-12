/*
 * Copyright (c) 2018-2020, 2022 Arm Limited
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
    TLBIOp(ExceptionLevel _targetEL, bool _secure)
      : secureLookup(_secure), targetEL(_targetEL)
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
    ExceptionLevel targetEL;
};

/** TLB Invalidate All */
class TLBIALL : public TLBIOp
{
  public:
    TLBIALL(ExceptionLevel _targetEL, bool _secure)
      : TLBIOp(_targetEL, _secure), inHost(false), el2Enabled(false),
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
        return TLBIALL(EL1, secureLookup);
    }

    bool inHost;
    bool el2Enabled;
    ExceptionLevel currentEL;
};

/** Instruction TLB Invalidate All */
class ITLBIALL : public TLBIALL
{
  public:
    ITLBIALL(ExceptionLevel _targetEL, bool _secure)
      : TLBIALL(_targetEL, _secure)
    {}

    void broadcast(ThreadContext *tc) = delete;

    void operator()(ThreadContext* tc) override;

    bool match(TlbEntry *entry, vmid_t curr_vmid) const override;
};

/** Data TLB Invalidate All */
class DTLBIALL : public TLBIALL
{
  public:
    DTLBIALL(ExceptionLevel _targetEL, bool _secure)
      : TLBIALL(_targetEL, _secure)
    {}

    void broadcast(ThreadContext *tc) = delete;

    void operator()(ThreadContext* tc) override;

    bool match(TlbEntry *entry, vmid_t curr_vmid) const override;
};

/** Implementaton of AArch64 TLBI ALLE(1,2,3)(IS) instructions */
class TLBIALLEL : public TLBIOp
{
  public:
    TLBIALLEL(ExceptionLevel _targetEL, bool _secure)
      : TLBIOp(_targetEL, _secure), inHost(false)
    {}

    void operator()(ThreadContext* tc) override;

    bool match(TlbEntry *entry, vmid_t curr_vmid) const override;

    bool
    stage2Flush() const override
    {
        // If we're targeting EL1 then flush stage2 as well
        return targetEL == EL1;
    }

    TLBIALLEL
    makeStage2() const
    {
        return TLBIALLEL(EL1, secureLookup);
    }

    bool inHost;
};

/** Implementaton of AArch64 TLBI VMALLE1(IS)/VMALLS112E1(IS) instructions */
class TLBIVMALL : public TLBIOp
{
  public:
    TLBIVMALL(ExceptionLevel _targetEL, bool _secure, bool _stage2)
      : TLBIOp(_targetEL, _secure), inHost(false), el2Enabled(false),
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
        return TLBIVMALL(EL1, secureLookup, false);
    }

    bool inHost;
    bool el2Enabled;
    bool stage2;
};

/** TLB Invalidate by ASID match */
class TLBIASID : public TLBIOp
{
  public:
    TLBIASID(ExceptionLevel _targetEL, bool _secure, uint16_t _asid)
      : TLBIOp(_targetEL, _secure), asid(_asid), inHost(false),
        el2Enabled(false)
    {}

    void operator()(ThreadContext* tc) override;

    bool match(TlbEntry *entry, vmid_t curr_vmid) const override;

    uint16_t asid;
    bool inHost;
    bool el2Enabled;
};

/** Instruction TLB Invalidate by ASID match */
class ITLBIASID : public TLBIASID
{
  public:
    ITLBIASID(ExceptionLevel _targetEL, bool _secure, uint16_t _asid)
      : TLBIASID(_targetEL, _secure, _asid)
    {}

    void broadcast(ThreadContext *tc) = delete;

    void operator()(ThreadContext* tc) override;

    bool match(TlbEntry *entry, vmid_t curr_vmid) const override;
};

/** Data TLB Invalidate by ASID match */
class DTLBIASID : public TLBIASID
{
  public:
    DTLBIASID(ExceptionLevel _targetEL, bool _secure, uint16_t _asid)
      : TLBIASID(_targetEL, _secure, _asid)
    {}

    void broadcast(ThreadContext *tc) = delete;

    void operator()(ThreadContext* tc) override;

    bool match(TlbEntry *entry, vmid_t curr_vmid) const override;
};

/** TLB Invalidate All, Non-Secure */
class TLBIALLN : public TLBIOp
{
  public:
    TLBIALLN(ExceptionLevel _targetEL)
      : TLBIOp(_targetEL, false)
    {}

    void operator()(ThreadContext* tc) override;

    bool match(TlbEntry *entry, vmid_t curr_vmid) const override;

    bool
    stage2Flush() const override
    {
        return targetEL != EL2;
    }

    TLBIALLN
    makeStage2() const
    {
        return TLBIALLN(EL1);
    }
};

/** TLB Invalidate by VA, All ASID */
class TLBIMVAA : public TLBIOp
{
  public:
    TLBIMVAA(ExceptionLevel _targetEL, bool _secure,
             Addr _addr)
      : TLBIOp(_targetEL, _secure), addr(_addr), inHost(false)
    {}

    void operator()(ThreadContext* tc) override;

    bool match(TlbEntry *entry, vmid_t curr_vmid) const override;

    Addr addr;
    bool inHost;
};

/** TLB Invalidate by VA */
class TLBIMVA : public TLBIOp
{
  public:
    TLBIMVA(ExceptionLevel _targetEL, bool _secure,
            Addr _addr, uint16_t _asid)
      : TLBIOp(_targetEL, _secure), addr(_addr), asid(_asid),
        inHost(false)
    {}

    void operator()(ThreadContext* tc) override;

    bool match(TlbEntry *entry, vmid_t curr_vmid) const override;

    Addr addr;
    uint16_t asid;
    bool inHost;
};

/** Instruction TLB Invalidate by VA */
class ITLBIMVA : public TLBIMVA
{
  public:
    ITLBIMVA(ExceptionLevel _targetEL, bool _secure,
             Addr _addr, uint16_t _asid)
      : TLBIMVA(_targetEL, _secure, _addr, _asid)
    {}

    void broadcast(ThreadContext *tc) = delete;

    void operator()(ThreadContext* tc) override;

    bool match(TlbEntry *entry, vmid_t curr_vmid) const override;
};

/** Data TLB Invalidate by VA */
class DTLBIMVA : public TLBIMVA
{
  public:
    DTLBIMVA(ExceptionLevel _targetEL, bool _secure,
             Addr _addr, uint16_t _asid)
      : TLBIMVA(_targetEL, _secure, _addr, _asid)
    {}

    void broadcast(ThreadContext *tc) = delete;

    void operator()(ThreadContext* tc) override;

    bool match(TlbEntry *entry, vmid_t curr_vmid) const override;
};

/** TLB Invalidate by Intermediate Physical Address */
class TLBIIPA : public TLBIOp
{
  public:
    TLBIIPA(ExceptionLevel _targetEL, bool _secure, Addr _addr)
      : TLBIOp(_targetEL, _secure), addr(_addr)
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
    TLBIMVAA
    makeStage2() const
    {
        return TLBIMVAA(EL1, secureLookup, addr);
    }

    Addr addr;
};

} // namespace ArmISA
} // namespace gem5

#endif //__ARCH_ARM_TLBI_HH__
