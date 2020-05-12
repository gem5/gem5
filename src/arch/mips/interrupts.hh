/*
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
 */

#ifndef __ARCH_MIPS_INTERRUPT_HH__
#define __ARCH_MIPS_INTERRUPT_HH__

#include <string>

#include "arch/generic/interrupts.hh"
#include "arch/mips/faults.hh"
#include "base/compiler.hh"
#include "base/logging.hh"
#include "params/MipsInterrupts.hh"
#include "sim/serialize.hh"

class BaseCPU;
class Checkpoint;

namespace MipsISA
{

class Interrupts : public BaseInterrupts
{
  public:
    typedef MipsInterruptsParams Params;

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    Interrupts(Params * p) : BaseInterrupts(p) {}

    //  post(int int_num, int index) is responsible
    //  for posting an interrupt. It sets a bit
    //  in intstatus corresponding to Cause IP*. The
    //  MIPS register Cause is updated by updateIntrInfo
    //  which is called by checkInterrupts
    //
    void post(int int_num);
    void post(int int_num, int index) override;

    // clear(int int_num, int index) is responsible
    //  for clearing an interrupt. It clear a bit
    //  in intstatus corresponding to Cause IP*. The
    //  MIPS register Cause is updated by updateIntrInfo
    //  which is called by checkInterrupts
    //
    void clear(int int_num);
    void clear(int int_num, int index) override;

    //  clearAll() is responsible
    //  for clearing all interrupts. It clears all bits
    //  in intstatus corresponding to Cause IP*. The
    //  MIPS register Cause is updated by updateIntrInfo
    //  which is called by checkInterrupts
    //
    void clearAll() override;

    // getInterrupt() checks if an interrupt
    //  should be returned. It ands the interrupt mask and
    //  and interrupt pending bits to see if one exists. It
    //  also makes sure interrupts are enabled (IE) and
    //  that ERL and ERX are not set
    //
    Fault getInterrupt() override;

    // updateIntrInfo() const syncs the
    //  MIPS cause register with the instatus variable. instatus
    //  is essentially a copy of the MIPS cause[IP7:IP0]
    //
    void updateIntrInfo() override;
    bool interruptsPending() const;
    bool onCpuTimerInterrupt() const;
    bool checkInterrupts() const override;

    void
    serialize(CheckpointOut &cp) const override
    {
        fatal("Serialization of Interrupts Unimplemented for MIPS");
    }

    void
    unserialize(CheckpointIn &cp) override
    {
        fatal("Unserialization of Interrupts Unimplemented for MIPS");
    }
};

}

#endif

