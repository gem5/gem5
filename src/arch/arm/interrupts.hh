/*
 * Copyright (c) 2010, 2012-2013 ARM Limited
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
#include "arch/arm/utility.hh"
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

    enum InterruptMask {
        INT_MASK_M, // masked (subject to PSTATE.{A,I,F} mask bit
        INT_MASK_T, // taken regardless of mask
        INT_MASK_P  // pending
    };

    bool takeInt(ThreadContext *tc, InterruptTypes int_type) const;

    bool
    checkInterrupts(ThreadContext *tc) const
    {
        HCR  hcr  = tc->readMiscReg(MISCREG_HCR);

        if (!(intStatus || hcr.va || hcr.vi || hcr.vf))
            return false;

        CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);
        SCR  scr  = tc->readMiscReg(MISCREG_SCR);

        bool isHypMode   = cpsr.mode == MODE_HYP;
        bool isSecure    = inSecureState(scr, cpsr);
        bool allowVIrq   = !cpsr.i && hcr.imo && !isSecure && !isHypMode;
        bool allowVFiq   = !cpsr.f && hcr.fmo && !isSecure && !isHypMode;
        bool allowVAbort = !cpsr.a && hcr.amo && !isSecure && !isHypMode;

        bool take_irq = takeInt(tc, INT_IRQ);
        bool take_fiq = takeInt(tc, INT_FIQ);
        bool take_ea =  takeInt(tc, INT_ABT);

        return ((interrupts[INT_IRQ] && take_irq)                   ||
                (interrupts[INT_FIQ] && take_fiq)                   ||
                (interrupts[INT_ABT] && take_ea)                    ||
                ((interrupts[INT_VIRT_IRQ] || hcr.vi) && allowVIrq) ||
                ((interrupts[INT_VIRT_FIQ] || hcr.vf) && allowVFiq) ||
                (hcr.va && allowVAbort)                             ||
                (interrupts[INT_RST])                               ||
                (interrupts[INT_SEV])
               );
    }

    /**
     * This function is used to check if a wfi operation should sleep. If there
     * is an interrupt pending, even if it's masked, wfi doesn't sleep.
     * @return any interrupts pending
     */
    bool
    checkWfiWake(HCR hcr, CPSR cpsr, SCR scr) const
    {
        uint64_t maskedIntStatus;
        bool     virtWake;

        maskedIntStatus = intStatus & ~((1 << INT_VIRT_IRQ) |
                                        (1 << INT_VIRT_FIQ));
        virtWake  = (hcr.vi || interrupts[INT_VIRT_IRQ]) && hcr.imo;
        virtWake |= (hcr.vf || interrupts[INT_VIRT_FIQ]) && hcr.fmo;
        virtWake |=  hcr.va                              && hcr.amo;
        virtWake &= (cpsr.mode != MODE_HYP) && !inSecureState(scr, cpsr);
        return maskedIntStatus || virtWake;
    }

    uint32_t
    getISR(HCR hcr, CPSR cpsr, SCR scr)
    {
        bool useHcrMux;
        CPSR isr = 0; // ARM ARM states ISR reg uses same bit possitions as CPSR

        useHcrMux = (cpsr.mode != MODE_HYP) && !inSecureState(scr, cpsr);
        isr.i = (useHcrMux & hcr.imo) ? (interrupts[INT_VIRT_IRQ] || hcr.vi)
                                      :  interrupts[INT_IRQ];
        isr.f = (useHcrMux & hcr.fmo) ? (interrupts[INT_VIRT_FIQ] || hcr.vf)
                                      :  interrupts[INT_FIQ];
        isr.a = (useHcrMux & hcr.amo) ?  hcr.va : interrupts[INT_ABT];
        return isr;
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
        HCR  hcr  = tc->readMiscReg(MISCREG_HCR);
        CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);
        SCR  scr  = tc->readMiscReg(MISCREG_SCR);

        // Calculate a few temp vars so we can work out if there's a pending
        // virtual interrupt, and if its allowed to happen
        // ARM ARM Issue C section B1.9.9, B1.9.11, and B1.9.13
        bool isHypMode   = cpsr.mode == MODE_HYP;
        bool isSecure    = inSecureState(scr, cpsr);
        bool allowVIrq   = !cpsr.i && hcr.imo && !isSecure && !isHypMode;
        bool allowVFiq   = !cpsr.f && hcr.fmo && !isSecure && !isHypMode;
        bool allowVAbort = !cpsr.a && hcr.amo && !isSecure && !isHypMode;

        if ( !(intStatus || (hcr.vi && allowVIrq) || (hcr.vf && allowVFiq) ||
               (hcr.va && allowVAbort)) )
            return NoFault;

        bool take_irq = takeInt(tc, INT_IRQ);
        bool take_fiq = takeInt(tc, INT_FIQ);
        bool take_ea =  takeInt(tc, INT_ABT);


        if (interrupts[INT_IRQ] && take_irq)
            return new Interrupt;
        if ((interrupts[INT_VIRT_IRQ] || hcr.vi) && allowVIrq)
            return new VirtualInterrupt;
        if (interrupts[INT_FIQ] && take_fiq)
            return new FastInterrupt;
        if ((interrupts[INT_VIRT_FIQ] || hcr.vf) && allowVFiq)
            return new VirtualFastInterrupt;
        if (interrupts[INT_ABT] && take_ea)
            return new SystemError;
        if (hcr.va && allowVAbort)
            return new VirtualDataAbort(0, TlbEntry::DomainType::NoAccess, false,
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
