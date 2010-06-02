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

#include "arch/arm/types.hh"
#include "config/full_system.hh"
#include "sim/faults.hh"

// The design of the "name" and "vect" functions is in sim/faults.hh

namespace ArmISA
{
typedef const Addr FaultOffset;

class ArmFaultBase : public FaultBase
{
  protected:
    Addr getVector(ThreadContext *tc);

  public:
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

#if FULL_SYSTEM
    void invoke(ThreadContext *tc);
#endif
    virtual FaultStat& countStat() = 0;
    virtual FaultOffset offset() = 0;
    virtual OperatingMode nextMode() = 0;
    virtual uint8_t armPcOffset() = 0;
    virtual uint8_t thumbPcOffset() = 0;
    virtual bool abortDisable() = 0;
    virtual bool fiqDisable() = 0;
};

template<typename T>
class ArmFault : public ArmFaultBase
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


class Reset                : public ArmFault<Reset> {};

class UndefinedInstruction : public ArmFault<UndefinedInstruction>
{
#if !FULL_SYSTEM
  protected:
    ExtMachInst machInst;
    bool unknown;
    const char *mnemonic;

  public:
    UndefinedInstruction(ExtMachInst _machInst,
                         bool _unknown,
                         const char *_mnemonic = NULL) :
        machInst(_machInst), unknown(_unknown), mnemonic(_mnemonic)
    {
    }

    void invoke(ThreadContext *tc);
#endif
};

class SupervisorCall       : public ArmFault<SupervisorCall>
{
#if !FULL_SYSTEM
  protected:
    ExtMachInst machInst;

  public:
    SupervisorCall(ExtMachInst _machInst) : machInst(_machInst)
    {}

    void invoke(ThreadContext *tc);
#endif
};
class PrefetchAbort        : public ArmFault<PrefetchAbort> {};
class DataAbort            : public ArmFault<DataAbort> {};
class Interrupt            : public ArmFault<Interrupt> {};
class FastInterrupt        : public ArmFault<FastInterrupt> {};


} // ArmISA namespace

#endif // __ARM_FAULTS_HH__
