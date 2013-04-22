/*
 * Copyright (c) 2010,2012 ARM Limited
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
 * Copyright (c) 2006 The Regents of The University of Michigan
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
#include "arch/arm/miscregs.hh"
#include "arch/arm/registers.hh"
#include "cpu/thread_context.hh"
#include "debug/Interrupt.hh"
#include "params/ArmInterrupts.hh"
#include "sim/sim_object.hh"

namespace ArmISA
{

class Interrupts : public SimObject
{
  private:
    BaseCPU * cpu;

    bool interrupts[NumInterruptTypes];
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
        DPRINTF(Interrupt, "Interrupt %d:%d posted\n", int_num, index);

        if (int_num < 0 || int_num >= NumInterruptTypes)
            panic("int_num out of bounds\n");

        if (index != 0)
            panic("No support for other interrupt indexes\n");

        interrupts[int_num] = true;
        intStatus |= ULL(1) << int_num;
    }

    void
    clear(int int_num, int index)
    {
        DPRINTF(Interrupt, "Interrupt %d:%d cleared\n", int_num, index);

        if (int_num < 0 || int_num >= NumInterruptTypes)
            panic("int_num out of bounds\n");

        if (index != 0)
            panic("No support for other interrupt indexes\n");

        interrupts[int_num] = false;
        intStatus &= ~(ULL(1) << int_num);
    }

    void
    clearAll()
    {
        DPRINTF(Interrupt, "Interrupts all cleared\n");
        intStatus = 0;
        memset(interrupts, 0, sizeof(interrupts));
    }

    bool
    checkInterrupts(ThreadContext *tc) const
    {
        if (!intStatus)
            return false;

        CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);

        return ((interrupts[INT_IRQ] && !cpsr.i) ||
                (interrupts[INT_FIQ] && !cpsr.f) ||
                (interrupts[INT_ABT] && !cpsr.a) ||
                (interrupts[INT_RST]) ||
                (interrupts[INT_SEV]));
    }

    /**
     * Check the raw interrupt state.
     * This function is used to check if a wfi operation should sleep. If there
     * is an interrupt pending, even if it's masked, wfi doesn't sleep.
     * @return any interrupts pending
     */
    bool
    checkRaw() const
    {
        return intStatus;
    }

    /**
     * Check the state of a particular interrupt, ignoring CPSR masks.
     *
     * This method is primarily used when running the target CPU in a
     * hardware VM (e.g., KVM) to check if interrupts should be
     * delivered upon guest entry.
     *
     * @param interrupt Interrupt type to check the state of.
     * @return true if the interrupt is asserted, false otherwise.
     */
    bool
    checkRaw(InterruptTypes interrupt) const
    {
        if (interrupt >= NumInterruptTypes)
            panic("Interrupt number out of range.\n");

        return interrupts[interrupt];
    }

    Fault
    getInterrupt(ThreadContext *tc)
    {
        if (!intStatus)
            return NoFault;

        CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);

        if (interrupts[INT_IRQ] && !cpsr.i)
            return new Interrupt;
        if (interrupts[INT_FIQ] && !cpsr.f)
            return new FastInterrupt;
        if (interrupts[INT_ABT] && !cpsr.a)
            return new DataAbort(0, false, 0,
                    ArmFault::AsynchronousExternalAbort);
        if (interrupts[INT_RST])
           return new Reset;
        if (interrupts[INT_SEV])
           return new ArmSev;

        panic("intStatus and interrupts not in sync\n");
    }

    void
    updateIntrInfo(ThreadContext *tc)
    {
        ; // nothing to do
    }

    void
    serialize(std::ostream &os)
    {
        SERIALIZE_ARRAY(interrupts, NumInterruptTypes);
        SERIALIZE_SCALAR(intStatus);
    }

    void
    unserialize(Checkpoint *cp, const std::string &section)
    {
        UNSERIALIZE_ARRAY(interrupts, NumInterruptTypes);
        UNSERIALIZE_SCALAR(intStatus);
    }
};
} // namespace ARM_ISA

#endif // __ARCH_ARM_INTERRUPT_HH__
