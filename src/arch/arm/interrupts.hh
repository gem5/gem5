/*
 * Copyright (c) 2006 The Regents of The University of Michigan
 * Copyright (c) 2009 ARM Limited
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

#ifndef __ARCH_ARM_INTERRUPT_HH__
#define __ARCH_ARM_INTERRUPT_HH__

#include "arch/arm/faults.hh"
#include "arch/arm/isa_traits.hh"
#include "arch/arm/registers.hh"
#include "cpu/thread_context.hh"
#include "params/ArmInterrupts.hh"
#include "sim/sim_object.hh"

namespace ArmISA
{

class Interrupts : public SimObject
{
  private:
    BaseCPU * cpu;

    uint64_t intStatus;

  public:

    void
    setCPU(BaseCPU * _cpu)
    {
        cpu = _cpu;
    }

    typedef ArmInterruptsParams Params;

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    Interrupts(Params * p) : SimObject(p), cpu(NULL)
    {
        clearAll();
    }


    void
    post(int int_num, int index)
    {
    }

    void
    clear(int int_num, int index)
    {
    }

    void
    clearAll()
    {
        intStatus = 0;
    }

    bool
    checkInterrupts(ThreadContext *tc) const
    {
        return intStatus;
    }

    Fault
    getInterrupt(ThreadContext *tc)
    {
        warn_once("ARM  Interrupts not handled\n");
        return NoFault;
    }

    void
    updateIntrInfo(ThreadContext *tc)
    {

    }

    void
    serialize(std::ostream &os)
    {
    }

    void
    unserialize(Checkpoint *cp, const std::string &section)
    {
    }
};
} // namespace ARM_ISA

#endif // __ARCH_ARM_INTERRUPT_HH__
