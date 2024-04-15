/*
 * Copyright 2019 Google Inc.
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

#include "arch/arm/fastmodel/iris/interrupts.hh"

#include "arch/arm/fastmodel/iris/thread_context.hh"
#include "arch/arm/interrupts.hh"
#include "arch/arm/regs/misc.hh"
#include "arch/arm/regs/misc_types.hh"
#include "arch/arm/types.hh"
#include "params/IrisInterrupts.hh"

namespace gem5
{

void
Iris::Interrupts::serialize(CheckpointOut &cp) const
{
    using namespace ArmISA;

    CPSR cpsr = tc->readMiscRegNoEffect(MISCREG_CPSR);
    CPSR orig_cpsr = cpsr;
    SCR scr = tc->readMiscRegNoEffect(MISCREG_SCR_EL3);
    SCR orig_scr = scr;
    HCR hcr = tc->readMiscRegNoEffect(MISCREG_HCR_EL2);
    HCR orig_hcr = hcr;

    // Set up state so we can get either physical or virtual interrupt bits.
    cpsr.mode = 0;
    cpsr.width = 0;
    cpsr.el = EL1;
    tc->setMiscReg(MISCREG_CPSR, cpsr);
    scr.eel2 = 1;
    tc->setMiscReg(MISCREG_SCR, scr);

    // Get the virtual bits.
    hcr.imo = 1;
    hcr.fmo = 1;
    hcr.amo = 1;
    tc->setMiscReg(MISCREG_HCR_EL2, hcr);

    RegVal isr_el1 = tc->readMiscRegNoEffect(MISCREG_ISR_EL1);
    // There is also a virtual abort, but it's not used by gem5.
    bool virt_irq = bits(7, isr_el1);
    bool virt_fiq = bits(6, isr_el1);

    // Get the physical bits.
    hcr.imo = 0;
    hcr.fmo = 0;
    hcr.amo = 0;
    tc->setMiscReg(MISCREG_HCR_EL2, hcr);

    isr_el1 = tc->readMiscRegNoEffect(MISCREG_ISR_EL1);
    bool phys_abort = bits(8, isr_el1);
    bool phys_irq = bits(7, isr_el1);
    bool phys_fiq = bits(6, isr_el1);

    tc->setMiscReg(MISCREG_CPSR, orig_cpsr);
    tc->setMiscReg(MISCREG_SCR_EL3, orig_scr);
    tc->setMiscReg(MISCREG_HCR_EL2, orig_hcr);

    bool interrupts[ArmISA::NumInterruptTypes];
    uint64_t intStatus = 0;

    for (bool &i : interrupts)
        i = false;

    interrupts[ArmISA::INT_ABT] = phys_abort;
    interrupts[ArmISA::INT_IRQ] = phys_irq;
    interrupts[ArmISA::INT_FIQ] = phys_fiq;
    interrupts[ArmISA::INT_SEV] = tc->readMiscReg(MISCREG_SEV_MAILBOX);
    interrupts[ArmISA::INT_VIRT_IRQ] = virt_irq;
    interrupts[ArmISA::INT_VIRT_FIQ] = virt_fiq;

    for (int i = 0; i < NumInterruptTypes; i++) {
        if (interrupts[i])
            intStatus |= (0x1ULL << i);
    }

    SERIALIZE_ARRAY(interrupts, NumInterruptTypes);
    SERIALIZE_SCALAR(intStatus);
}

void
Iris::Interrupts::unserialize(CheckpointIn &cp)
{}

} // namespace gem5
