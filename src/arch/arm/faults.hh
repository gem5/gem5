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
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
 * Copyright (c) 2007-2008 The Florida State University
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
 *          Gabe Black
 */

#ifndef __ARM_FAULTS_HH__
#define __ARM_FAULTS_HH__

#include "arch/arm/miscregs.hh"
#include "arch/arm/types.hh"
#include "base/misc.hh"
#include "sim/faults.hh"
#include "sim/full_system.hh"

// The design of the "name" and "vect" functions is in sim/faults.hh

namespace ArmISA
{
typedef const Addr FaultOffset;

class ArmFault : public FaultBase
{
  protected:
    Addr getVector(ThreadContext *tc);

  public:
    enum StatusEncoding
    {
        // Fault Status register encodings
        // ARM ARM B3.9.4
        AlignmentFault = 0x1,
        DebugEvent = 0x2,
        AccessFlag0 = 0x3,
        InstructionCacheMaintenance = 0x4,
        Translation0 = 0x5,
        AccessFlag1 = 0x6,
        Translation1 = 0x7,
        SynchronousExternalAbort0 = 0x8,
        Domain0 = 0x9,
        SynchronousExternalAbort1 = 0x8,
        Domain1 = 0xb,
        TranslationTableWalkExtAbt0 = 0xc,
        Permission0 = 0xd,
        TranslationTableWalkExtAbt1 = 0xe,
        Permission1 = 0xf,
        AsynchronousExternalAbort = 0x16,
        MemoryAccessAsynchronousParityError = 0x18,
        MemoryAccessSynchronousParityError = 0x19,
        TranslationTableWalkPrtyErr0 = 0x1c,
        TranslationTableWalkPrtyErr1 = 0x1e,

        // not a real fault. This is a status code
        // to allow the translation function to inform
        // the memory access function not to proceed
        // for a Prefetch that misses in the TLB.
        PrefetchTLBMiss = 0x1f,
        PrefetchUncacheable = 0x20
    };

    struct FaultVals
    {
        const FaultName name;
        const FaultOffset offset;
        const OperatingMode nextMode;
        const uint8_t armPcOffset;
        const uint8_t thumbPcOffset;
        const bool abortDisable;
        const bool fiqDisable;
        FaultStat count;
    };

    void invoke(ThreadContext *tc,
            StaticInstPtr inst = StaticInst::nullStaticInstPtr);
    virtual FaultStat& countStat() = 0;
    virtual FaultOffset offset() = 0;
    virtual OperatingMode nextMode() = 0;
    virtual uint8_t armPcOffset() = 0;
    virtual uint8_t thumbPcOffset() = 0;
    virtual bool abortDisable() = 0;
    virtual bool fiqDisable() = 0;
};

template<typename T>
class ArmFaultVals : public ArmFault
{
  protected:
    static FaultVals vals;

  public:
    FaultName name() const { return vals.name; }
    FaultStat & countStat() {return vals.count;}
    FaultOffset offset() { return vals.offset; }
    OperatingMode nextMode() { return vals.nextMode; }
    uint8_t armPcOffset() { return vals.armPcOffset; }
    uint8_t thumbPcOffset() { return vals.thumbPcOffset; }
    bool abortDisable() { return vals.abortDisable; }
    bool fiqDisable() { return vals.fiqDisable; }
};

class Reset : public ArmFaultVals<Reset>
{
  public:
    void invoke(ThreadContext *tc,
            StaticInstPtr inst = StaticInst::nullStaticInstPtr);
};

class UndefinedInstruction : public ArmFaultVals<UndefinedInstruction>
{
  protected:
    ExtMachInst machInst;
    bool unknown;
    const char *mnemonic;
    bool disabled;

  public:
    UndefinedInstruction(ExtMachInst _machInst,
                         bool _unknown,
                         const char *_mnemonic = NULL,
                         bool _disabled = false) :
        machInst(_machInst), unknown(_unknown),
        mnemonic(_mnemonic), disabled(_disabled)
    {
    }
    UndefinedInstruction() :
        machInst(0), unknown(false), mnemonic("undefined"), disabled(false)
    {}

    void invoke(ThreadContext *tc,
            StaticInstPtr inst = StaticInst::nullStaticInstPtr);
};

class SupervisorCall : public ArmFaultVals<SupervisorCall>
{
  protected:
    ExtMachInst machInst;

  public:
    SupervisorCall(ExtMachInst _machInst) : machInst(_machInst)
    {}
    SupervisorCall() : machInst(0)
    {}

    void invoke(ThreadContext *tc,
            StaticInstPtr inst = StaticInst::nullStaticInstPtr);
};

template <class T>
class AbortFault : public ArmFaultVals<T>
{
  protected:
    Addr faultAddr;
    bool write;
    uint8_t domain;
    uint8_t status;

  public:
    AbortFault(Addr _faultAddr, bool _write,
            uint8_t _domain, uint8_t _status) :
        faultAddr(_faultAddr), write(_write),
        domain(_domain), status(_status)
    {}

    void invoke(ThreadContext *tc,
            StaticInstPtr inst = StaticInst::nullStaticInstPtr);
};

class PrefetchAbort : public AbortFault<PrefetchAbort>
{
  public:
    static const MiscRegIndex FsrIndex = MISCREG_IFSR;
    static const MiscRegIndex FarIndex = MISCREG_IFAR;

    PrefetchAbort(Addr _addr, uint8_t _status) :
        AbortFault<PrefetchAbort>(_addr, false, 0, _status)
    {}
};

class DataAbort : public AbortFault<DataAbort>
{
  public:
    static const MiscRegIndex FsrIndex = MISCREG_DFSR;
    static const MiscRegIndex FarIndex = MISCREG_DFAR;

    DataAbort(Addr _addr, uint8_t _domain, bool _write, uint8_t _status) :
        AbortFault<DataAbort>(_addr, _write, _domain, _status)
    {}
};

class Interrupt : public ArmFaultVals<Interrupt> {};
class FastInterrupt : public ArmFaultVals<FastInterrupt> {};

// A fault that flushes the pipe, excluding the faulting instructions
class FlushPipe : public ArmFaultVals<FlushPipe>
{
  public:
    FlushPipe() {}
    void invoke(ThreadContext *tc,
            StaticInstPtr inst = StaticInst::nullStaticInstPtr);
};

// A fault that flushes the pipe, excluding the faulting instructions
class ArmSev : public ArmFaultVals<ArmSev>
{
  public:
    ArmSev () {}
    void invoke(ThreadContext *tc,
            StaticInstPtr inst = StaticInst::nullStaticInstPtr);
};

} // namespace ArmISA

#endif // __ARM_FAULTS_HH__
