/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
 * Copyright (c) 2007 MIPS Technologies, Inc.
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
 * Authors: Gabe Black
 *          Korey Sewell
 *          Jaidev Patwardhan
 */

#ifndef __MIPS_FAULTS_HH__
#define __MIPS_FAULTS_HH__

#include "sim/faults.hh"

namespace MipsISA
{

typedef const Addr FaultVect;

class MipsFaultBase : public FaultBase
{
  protected:
    virtual bool skipFaultingInstruction() {return false;}
    virtual bool setRestartAddress() {return true;}
  public:
    struct FaultVals
    {
        const FaultName name;
        const FaultVect vect;
    };

    Addr badVAddr;
    Addr entryHiAsid;
    Addr entryHiVPN2;
    Addr entryHiVPN2X;
    Addr contextBadVPN2;
#if FULL_SYSTEM
    void invoke(ThreadContext * tc,
            StaticInst::StaticInstPtr inst = StaticInst::nullStaticInstPtr)
    {}
    void setExceptionState(ThreadContext *, uint8_t);
    void setHandlerPC(Addr, ThreadContext *);
#endif
};

template <typename T>
class MipsFault : public MipsFaultBase
{
  protected:
    static FaultVals vals;
  public:
    FaultName name() const { return vals.name; }
    FaultVect vect() const { return vals.vect; }
};

class MachineCheckFault : public MipsFault<MachineCheckFault>
{
  public:
    bool isMachineCheckFault() {return true;}
};

static inline Fault genMachineCheckFault()
{
    return new MachineCheckFault;
}

class NonMaskableInterrupt : public MipsFault<NonMaskableInterrupt>
{
  public:
    bool isNonMaskableInterrupt() {return true;}
};

class AddressErrorFault : public MipsFault<AddressErrorFault>
{
  protected:
    Addr vaddr;
    bool store;
  public:
    AddressErrorFault(Addr _vaddr, bool _store) : vaddr(_vaddr), store(_store)
    {}
#if FULL_SYSTEM
    void invoke(ThreadContext * tc,
            StaticInstPtr inst = StaticInst::nullStaticInstPtr);
#endif

};

class ResetFault : public MipsFault<ResetFault>
{
  public:
    void invoke(ThreadContext * tc,
            StaticInstPtr inst = StaticInst::nullStaticInstPtr);

};

class SystemCallFault : public MipsFault<SystemCallFault>
{
  public:
#if FULL_SYSTEM
    void invoke(ThreadContext * tc,
            StaticInstPtr inst = StaticInst::nullStaticInstPtr);
#endif
};

class SoftResetFault : public MipsFault<SoftResetFault>
{
  public:
    void invoke(ThreadContext * tc,
            StaticInstPtr inst = StaticInst::nullStaticInstPtr);
};

class CoprocessorUnusableFault : public MipsFault<CoprocessorUnusableFault>
{
  protected:
    int coProcID;
  public:
    CoprocessorUnusableFault(int _procid) : coProcID(_procid)
    {}

    void invoke(ThreadContext * tc,
            StaticInstPtr inst = StaticInst::nullStaticInstPtr);
};

class ReservedInstructionFault : public MipsFault<ReservedInstructionFault>
{
  public:
    void invoke(ThreadContext * tc,
            StaticInstPtr inst = StaticInst::nullStaticInstPtr);
};

class ThreadFault : public MipsFault<ThreadFault>
{
  public:
    void invoke(ThreadContext * tc,
            StaticInstPtr inst = StaticInst::nullStaticInstPtr);
};

class IntegerOverflowFault : public MipsFault<IntegerOverflowFault>
{
  protected:
    bool skipFaultingInstruction() {return true;}
  public:
#if FULL_SYSTEM
    void invoke(ThreadContext * tc,
            StaticInstPtr inst = StaticInst::nullStaticInstPtr);
#endif
};

class InterruptFault : public MipsFault<InterruptFault>
{
  protected:
    bool setRestartAddress() {return false;}
  public:
#if FULL_SYSTEM
    void invoke(ThreadContext * tc,
            StaticInstPtr inst = StaticInst::nullStaticInstPtr);
#endif
};

class TrapFault : public MipsFault<TrapFault>
{
  public:
#if FULL_SYSTEM
    void invoke(ThreadContext * tc,
            StaticInstPtr inst = StaticInst::nullStaticInstPtr);
#endif
};

class BreakpointFault : public MipsFault<BreakpointFault>
{
  public:
#if FULL_SYSTEM
    void invoke(ThreadContext * tc,
            StaticInstPtr inst = StaticInst::nullStaticInstPtr);
#endif
};

class TlbRefillFault : public MipsFault<TlbRefillFault>
{
  protected:
    bool store;
  public:
    TlbRefillFault(Addr asid, Addr vaddr, Addr vpn, bool _store) :
        store(_store)
    {
        entryHiAsid = asid;
        entryHiVPN2 = vpn >> 2;
        entryHiVPN2X = vpn & 0x3;
        badVAddr = vaddr;
        contextBadVPN2 = vpn >> 2;
    }
#if FULL_SYSTEM
    void invoke(ThreadContext * tc,
            StaticInstPtr inst = StaticInst::nullStaticInstPtr);
#endif
};

class TlbInvalidFault : public MipsFault<TlbInvalidFault>
{
  protected:
    bool store;
  public:
    TlbInvalidFault(Addr asid, Addr vaddr, Addr vpn, bool _store) :
        store(_store)
    {
        entryHiAsid = asid;
        entryHiVPN2 = vpn >> 2;
        entryHiVPN2X = vpn & 0x3;
        badVAddr = vaddr;
        contextBadVPN2 = vpn >> 2;
    }
#if FULL_SYSTEM
    void invoke(ThreadContext * tc,
            StaticInstPtr inst = StaticInst::nullStaticInstPtr);
#endif
};

class TLBModifiedFault : public MipsFault<TLBModifiedFault>
{
  public:
    TLBModifiedFault(Addr asid, Addr vaddr, Addr vpn)
    {
        entryHiAsid = asid;
        entryHiVPN2 = vpn >> 2;
        entryHiVPN2X = vpn & 0x3;
        badVAddr = vaddr;
        contextBadVPN2 = vpn >> 2;
    }
#if FULL_SYSTEM
    void invoke(ThreadContext * tc,
            StaticInstPtr inst = StaticInst::nullStaticInstPtr);
#endif
};

class DspStateDisabledFault : public MipsFault<DspStateDisabledFault>
{
  public:
    void invoke(ThreadContext * tc,
            StaticInstPtr inst = StaticInst::nullStaticInstPtr);
};

} // namespace MipsISA

#endif // __MIPS_FAULTS_HH__
