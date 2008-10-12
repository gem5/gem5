/*
 * Copyright (c) 2008 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * Redistribution and use of this software in source and binary forms,
 * with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * The software must be used only for Non-Commercial Use which means any
 * use which is NOT directed to receiving any direct monetary
 * compensation for, or commercial advantage from such use.  Illustrative
 * examples of non-commercial use are academic research, personal study,
 * teaching, education and corporate research & development.
 * Illustrative examples of commercial use are distributing products for
 * commercial advantage and providing services using the software for
 * commercial advantage.
 *
 * If you wish to use this software or functionality therein that may be
 * covered by patents for commercial use, please contact:
 *     Director of Intellectual Property Licensing
 *     Office of Strategy and Technology
 *     Hewlett-Packard Company
 *     1501 Page Mill Road
 *     Palo Alto, California  94304
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.  Redistributions
 * in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.  Neither the name of
 * the COPYRIGHT HOLDER(s), HEWLETT-PACKARD COMPANY, nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.  No right of
 * sublicense is granted herewith.  Derivatives of the software and
 * output created using the software may be prepared, but only for
 * Non-Commercial Uses.  Derivatives of the software may be shared with
 * others provided: (i) the others agree to abide by the list of
 * conditions herein which includes the Non-Commercial Use restrictions;
 * and (ii) such Derivatives of the software include the above copyright
 * notice to acknowledge the contribution from this software where
 * applicable, this list of conditions and the disclaimer below.
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
 */

#include "arch/x86/interrupts.hh"
#include "cpu/base.hh"

int divideFromConf(uint32_t conf)
{
    // This figures out what division we want from the division configuration
    // register in the local APIC. The encoding is a little odd but it can
    // be deciphered fairly easily.
    int shift = ((conf & 0x8) >> 1) | (conf & 0x3);
    shift = (shift + 1) % 8;
    return 1 << shift;
}

uint32_t
X86ISA::Interrupts::readRegNoEffect(ApicRegIndex reg)
{
    return regs[reg];
}

uint32_t
X86ISA::Interrupts::readReg(ApicRegIndex reg, ThreadContext * tc)
{
    if (reg >= APIC_TRIGGER_MODE(0) &&
            reg <= APIC_TRIGGER_MODE(15)) {
        panic("Local APIC Trigger Mode registers are unimplemented.\n");
    }
    switch (reg) {
      case APIC_ARBITRATION_PRIORITY:
        panic("Local APIC Arbitration Priority register unimplemented.\n");
        break;
      case APIC_PROCESSOR_PRIORITY:
        panic("Local APIC Processor Priority register unimplemented.\n");
        break;
      case APIC_EOI:
        panic("Local APIC EOI register unimplemented.\n");
        break;
      case APIC_ERROR_STATUS:
        regs[APIC_INTERNAL_STATE] &= ~ULL(0x1);
        break;
      case APIC_INTERRUPT_COMMAND_LOW:
        panic("Local APIC Interrupt Command low"
                " register unimplemented.\n");
        break;
      case APIC_INTERRUPT_COMMAND_HIGH:
        panic("Local APIC Interrupt Command high"
                " register unimplemented.\n");
        break;
      case APIC_CURRENT_COUNT:
        {
            uint32_t val = regs[reg] - tc->getCpuPtr()->curCycle();
            val /= (16 * divideFromConf(regs[APIC_DIVIDE_CONFIGURATION]));
            return val;
        }
      default:
        break;
    }
    return readRegNoEffect(reg);
}

void
X86ISA::Interrupts::setRegNoEffect(ApicRegIndex reg, uint32_t val)
{
    regs[reg] = val;
}

void
X86ISA::Interrupts::setReg(ApicRegIndex reg, uint32_t val, ThreadContext *tc)
{
    uint32_t newVal = val;
    if (reg >= APIC_IN_SERVICE(0) &&
            reg <= APIC_IN_SERVICE(15)) {
        panic("Local APIC In-Service registers are unimplemented.\n");
    }
    if (reg >= APIC_TRIGGER_MODE(0) &&
            reg <= APIC_TRIGGER_MODE(15)) {
        panic("Local APIC Trigger Mode registers are unimplemented.\n");
    }
    if (reg >= APIC_INTERRUPT_REQUEST(0) &&
            reg <= APIC_INTERRUPT_REQUEST(15)) {
        panic("Local APIC Interrupt Request registers "
                "are unimplemented.\n");
    }
    switch (reg) {
      case APIC_ID:
        newVal = val & 0xFF;
        break;
      case APIC_VERSION:
        // The Local APIC Version register is read only.
        return;
      case APIC_TASK_PRIORITY:
        newVal = val & 0xFF;
        break;
      case APIC_ARBITRATION_PRIORITY:
        panic("Local APIC Arbitration Priority register unimplemented.\n");
        break;
      case APIC_PROCESSOR_PRIORITY:
        panic("Local APIC Processor Priority register unimplemented.\n");
        break;
      case APIC_EOI:
        panic("Local APIC EOI register unimplemented.\n");
        break;
      case APIC_LOGICAL_DESTINATION:
        newVal = val & 0xFF000000;
        break;
      case APIC_DESTINATION_FORMAT:
        newVal = val | 0x0FFFFFFF;
        break;
      case APIC_SPURIOUS_INTERRUPT_VECTOR:
        regs[APIC_INTERNAL_STATE] &= ~ULL(1 << 1);
        regs[APIC_INTERNAL_STATE] |= val & (1 << 8);
        if (val & (1 << 9))
            warn("Focus processor checking not implemented.\n");
        break;
      case APIC_ERROR_STATUS:
        {
            if (regs[APIC_INTERNAL_STATE] & 0x1) {
                regs[APIC_INTERNAL_STATE] &= ~ULL(0x1);
                newVal = 0;
            } else {
                regs[APIC_INTERNAL_STATE] |= ULL(0x1);
                return;
            }

        }
        break;
      case APIC_INTERRUPT_COMMAND_LOW:
        panic("Local APIC Interrupt Command low"
                " register unimplemented.\n");
        break;
      case APIC_INTERRUPT_COMMAND_HIGH:
        panic("Local APIC Interrupt Command high"
                " register unimplemented.\n");
        break;
      case APIC_LVT_TIMER:
      case APIC_LVT_THERMAL_SENSOR:
      case APIC_LVT_PERFORMANCE_MONITORING_COUNTERS:
      case APIC_LVT_LINT0:
      case APIC_LVT_LINT1:
      case APIC_LVT_ERROR:
        {
            uint64_t readOnlyMask = (1 << 12) | (1 << 14);
            newVal = (val & ~readOnlyMask) |
                     (regs[reg] & readOnlyMask);
        }
        break;
      case APIC_INITIAL_COUNT:
        newVal = bits(val, 31, 0);
        regs[APIC_CURRENT_COUNT] =
            tc->getCpuPtr()->curCycle() +
            (16 * divideFromConf(regs[APIC_DIVIDE_CONFIGURATION])) * newVal;
        //FIXME This should schedule the timer event.
        break;
      case APIC_CURRENT_COUNT:
        //Local APIC Current Count register is read only.
        return;
      case APIC_DIVIDE_CONFIGURATION:
        newVal = val & 0xB;
        break;
      default:
        break;
    }
    setRegNoEffect(reg, newVal);
    return;
}

X86ISA::Interrupts *
X86LocalApicParams::create()
{
    return new X86ISA::Interrupts(this);
}
