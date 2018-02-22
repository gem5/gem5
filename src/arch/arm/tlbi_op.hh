/*
 * Copyright (c) 2018 ARM Limited
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
 *
 * Authors: Giacomo Travaglini
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
        System *sys = tc->getSystemPtr();
        for (int x = 0; x < sys->numContexts(); x++) {
            ThreadContext *oc = sys->getThreadContext(x);
            (*this)(oc);
        }
    }

  protected:
    bool secureLookup;
    ExceptionLevel targetEL;
};

/** TLB Invalidate All */
class TLBIALL : public TLBIOp
{
  public:
    TLBIALL(ExceptionLevel _targetEL, bool _secure)
      : TLBIOp(_targetEL, _secure)
    {}

    void operator()(ThreadContext* tc) override;
};

/** Instruction TLB Invalidate All */
class ITLBIALL : public TLBIOp
{
  public:
    ITLBIALL(ExceptionLevel _targetEL, bool _secure)
      : TLBIOp(_targetEL, _secure)
    {}

    void broadcast(ThreadContext *tc) = delete;

    void operator()(ThreadContext* tc) override;
};

/** Data TLB Invalidate All */
class DTLBIALL : public TLBIOp
{
  public:
    DTLBIALL(ExceptionLevel _targetEL, bool _secure)
      : TLBIOp(_targetEL, _secure)
    {}

    void broadcast(ThreadContext *tc) = delete;

    void operator()(ThreadContext* tc) override;
};

/** TLB Invalidate by ASID match */
class TLBIASID : public TLBIOp
{
  public:
    TLBIASID(ExceptionLevel _targetEL, bool _secure, uint16_t _asid)
      : TLBIOp(_targetEL, _secure), asid(_asid)
    {}

    void operator()(ThreadContext* tc) override;

  protected:
    uint16_t asid;
};

/** Instruction TLB Invalidate by ASID match */
class ITLBIASID : public TLBIOp
{
  public:
    ITLBIASID(ExceptionLevel _targetEL, bool _secure, uint16_t _asid)
      : TLBIOp(_targetEL, _secure), asid(_asid)
    {}

    void broadcast(ThreadContext *tc) = delete;

    void operator()(ThreadContext* tc) override;

  protected:
    uint16_t asid;
};

/** Data TLB Invalidate by ASID match */
class DTLBIASID : public TLBIOp
{
  public:
    DTLBIASID(ExceptionLevel _targetEL, bool _secure, uint16_t _asid)
      : TLBIOp(_targetEL, _secure), asid(_asid)
    {}

    void broadcast(ThreadContext *tc) = delete;

    void operator()(ThreadContext* tc) override;

  protected:
    uint16_t asid;
};

/** TLB Invalidate All, Non-Secure */
class TLBIALLN : public TLBIOp
{
  public:
    TLBIALLN(ExceptionLevel _targetEL, bool _hyp)
      : TLBIOp(_targetEL, false), hyp(_hyp)
    {}

    void operator()(ThreadContext* tc) override;

  protected:
    bool hyp;
};

/** TLB Invalidate by VA, All ASID */
class TLBIMVAA : public TLBIOp
{
  public:
    TLBIMVAA(ExceptionLevel _targetEL, bool _secure,
             Addr _addr, bool _hyp)
      : TLBIOp(_targetEL, _secure), addr(_addr), hyp(_hyp)
    {}

    void operator()(ThreadContext* tc) override;

  protected:
    Addr addr;
    bool hyp;
};

/** TLB Invalidate by VA */
class TLBIMVA : public TLBIOp
{
  public:
    TLBIMVA(ExceptionLevel _targetEL, bool _secure,
            Addr _addr, uint16_t _asid)
      : TLBIOp(_targetEL, _secure), addr(_addr), asid(_asid)
    {}

    void operator()(ThreadContext* tc) override;

  protected:
    Addr addr;
    uint16_t asid;
};

/** Instruction TLB Invalidate by VA */
class ITLBIMVA : public TLBIOp
{
  public:
    ITLBIMVA(ExceptionLevel _targetEL, bool _secure,
             Addr _addr, uint16_t _asid)
      : TLBIOp(_targetEL, _secure), addr(_addr), asid(_asid)
    {}

    void broadcast(ThreadContext *tc) = delete;

    void operator()(ThreadContext* tc) override;

  protected:
    Addr addr;
    uint16_t asid;
};

/** Data TLB Invalidate by VA */
class DTLBIMVA : public TLBIOp
{
  public:
    DTLBIMVA(ExceptionLevel _targetEL, bool _secure,
             Addr _addr, uint16_t _asid)
      : TLBIOp(_targetEL, _secure), addr(_addr), asid(_asid)
    {}

    void broadcast(ThreadContext *tc) = delete;

    void operator()(ThreadContext* tc) override;

  protected:
    Addr addr;
    uint16_t asid;
};

/** TLB Invalidate by Intermediate Physical Address */
class TLBIIPA : public TLBIOp
{
  public:
    TLBIIPA(ExceptionLevel _targetEL, bool _secure, Addr _addr)
      : TLBIOp(_targetEL, _secure), addr(_addr)
    {}

    void operator()(ThreadContext* tc) override;

  protected:
    Addr addr;
};

} // namespace ArmISA

#endif //__ARCH_ARM_TLBI_HH__
