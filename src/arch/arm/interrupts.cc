/*
 * Copyright (c) 2009, 2012-2013, 2016, 2019, 2023 Arm Limited
 * All rights reserved.
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
 */

#include "arch/arm/interrupts.hh"

#include "arch/arm/system.hh"

namespace gem5
{

bool
ArmISA::Interrupts::takeInt32(InterruptTypes int_type) const
{
    InterruptMask mask;
    CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);
    SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);;
    HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    ExceptionLevel el = currEL(tc);
    bool cpsr_mask_bit, scr_routing_bit, scr_fwaw_bit, hcr_mask_override_bit;
    bool is_secure = isSecure(tc);

    switch(int_type) {
      case INT_FIQ:
        cpsr_mask_bit = cpsr.f;
        scr_routing_bit = scr.fiq;
        scr_fwaw_bit = scr.fw;
        hcr_mask_override_bit = hcr.fmo;
        break;
      case INT_IRQ:
        cpsr_mask_bit = cpsr.i;
        scr_routing_bit = scr.irq;
        scr_fwaw_bit = 1;
        hcr_mask_override_bit = hcr.imo;
        break;
      case INT_ABT:
        cpsr_mask_bit = cpsr.a;
        scr_routing_bit = scr.ea;
        scr_fwaw_bit = scr.aw;
        hcr_mask_override_bit = hcr.amo;
        break;
      default:
        panic("Unhandled interrupt type!");
    }

    if (hcr.tge)
        hcr_mask_override_bit = 1;

    if (!scr_routing_bit) {
        // SCR IRQ == 0
        if (!hcr_mask_override_bit)
            mask = INT_MASK_M;
        else {
            if (!is_secure && (el == EL0 || el == EL1))
                mask = INT_MASK_T;
            else
                mask = INT_MASK_M;
        }
    } else {
        // SCR IRQ == 1
        if ((!is_secure) &&
            (hcr_mask_override_bit ||
                (!scr_fwaw_bit && !hcr_mask_override_bit)))
            mask = INT_MASK_T;
        else
            mask = INT_MASK_M;
    }
    return ((mask == INT_MASK_T) ||
            ((mask == INT_MASK_M) && !cpsr_mask_bit)) &&
            (mask != INT_MASK_P);
}


bool
ArmISA::Interrupts::takeInt64(InterruptTypes int_type) const
{
    InterruptMask mask;
    CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);
    SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);;
    HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    ExceptionLevel el = currEL(tc);
    bool cpsr_mask_bit, scr_routing_bit, hcr_mask_override_bit;
    bool is_secure = isSecureBelowEL3(tc);

    switch(int_type) {
      case INT_FIQ:
        cpsr_mask_bit = cpsr.f;
        scr_routing_bit = scr.fiq;
        hcr_mask_override_bit = hcr.fmo;
        break;
      case INT_IRQ:
        cpsr_mask_bit = cpsr.i;
        scr_routing_bit = scr.irq;
        hcr_mask_override_bit = hcr.imo;
        break;
      case INT_ABT:
        cpsr_mask_bit = cpsr.a;
        scr_routing_bit = scr.ea;
        hcr_mask_override_bit = hcr.amo;
        break;
      default:
        panic("Unhandled interrupt type!");
    }

    if (is_secure) {
        if (!scr.eel2) {
            if (!scr_routing_bit) {
                // NS=0,EEL2=0,EAI/IRQ/FIQ=0
                if (el == EL3)
                    mask = INT_MASK_P;
                else
                    mask = INT_MASK_M;
            } else {
                // NS=0,EEL2=0,EAI/IRQ/FIQ=1
                if (el == EL3)
                    mask = INT_MASK_M;
                else
                    mask = INT_MASK_T;
            }
        } else {
            if (!scr_routing_bit) {
                if (!hcr.tge) {
                    if (!hcr_mask_override_bit) {
                        // NS=0,EEL2=1,EAI/IRQ/FIQ=0,TGE=0,AMO/IMO/FMO=0
                        if (el == EL3 || el == EL2)
                            mask = INT_MASK_P;
                        else
                            mask = INT_MASK_M;
                    } else {
                        // NS=0,EEL2=1,EAI/IRQ/FIQ=0,TGE=0,AMO/IMO/FMO=1
                        if (el == EL3)
                            mask = INT_MASK_P;
                        else if (el == EL2)
                            mask = INT_MASK_M;
                        else
                            mask = INT_MASK_T;
                    }
                } else {
                    if (!hcr.e2h) {
                        // NS=0,EEL2=1,EAI/IRQ/FIQ=0,TGE=1,E2H=0
                        if (el == EL3)
                            mask = INT_MASK_P;
                        else if (el == EL2)
                            mask = INT_MASK_M;
                        else
                            mask = INT_MASK_T;
                    } else {
                        // NS=0,EEL2=1,EAI/IRQ/FIQ=0,TGE=1,E2H=1
                        if (el == EL3)
                            mask = INT_MASK_P;
                        else
                            mask = INT_MASK_M;
                    }
                }
            } else {
                if (!hcr.tge) {
                    // NS=0,EEL2=1,EAI/IRQ/FIQ=1,TGE=0
                    if (el == EL3)
                        mask = INT_MASK_M;
                    else
                        mask = INT_MASK_T;
                } else {
                    // NS=0,EEL2=1,EAI/IRQ/FIQ=1,TGE=1
                    if (el == EL3)
                        mask = INT_MASK_M;
                    else
                        mask = INT_MASK_T;
                }
            }
        }
    } else {
        if (!scr_routing_bit) {
            if (!scr.rw) {
                if (!hcr.tge) {
                    if (!hcr_mask_override_bit) {
                        // NS=1,EAI/IRQ/FIQ=0,RW=0,TGE=0,AMO/IMO?/FMO=0
                        if (el == EL3)
                            mask = INT_MASK_P;
                        else
                            mask = INT_MASK_M;
                    } else {
                        // NS=1,EAI/IRQ/FIQ=0,RW=0,TGE=0,AMO/IMO?/FMO=1
                        if (el == EL3)
                            mask = INT_MASK_P;
                        else if (el == EL2)
                            mask = INT_MASK_M;
                        else
                            mask = INT_MASK_T;
                    }
                } else {
                    // NS=1,EAI/IRQ/FIQ=0,RW=0,TGE=1
                    if (el == EL3)
                        mask = INT_MASK_P;
                    else if (el == EL2)
                        mask = INT_MASK_M;
                    else
                        mask = INT_MASK_T;
                }
            } else {
                if (!hcr.tge) {
                    if (!hcr_mask_override_bit) {
                        // NS=1,EAI/IRQ/FIQ=0,RW=1,TGE=0,AMO/IMO/FMO=0
                        if (el == EL3 || el == EL2)
                            mask = INT_MASK_P;
                        else
                            mask = INT_MASK_M;
                    } else {
                        // NS=1,EAI/IRQ/FIQ=0,RW=1,TGE=0,AMO/IMO/FMO=1
                        if (el == EL3)
                            mask = INT_MASK_P;
                        else if (el == EL2)
                            mask = INT_MASK_M;
                        else
                            mask = INT_MASK_T;
                    }
                } else {
                    if (!hcr.e2h) {
                        // NS=1,EAI/IRQ/FIQ=0,RW=1,TGE=1,E2H=0
                        if (el == EL3)
                            mask = INT_MASK_P;
                        else if (el == EL2)
                            mask = INT_MASK_M;
                        else
                            mask = INT_MASK_T;
                    } else {
                        // NS=1,EAI/IRQ/FIQ=0,RW=1,TGE=1,E2H=1
                        if (el == EL3)
                            mask = INT_MASK_P;
                        else
                            mask = INT_MASK_M;
                    }
                }
            }
        } else {
            if (el == EL3)
                mask = INT_MASK_M;
            else
                mask = INT_MASK_T;
        }
    }
    return ((mask == INT_MASK_T) ||
            ((mask == INT_MASK_M) && !cpsr_mask_bit)) &&
            (mask != INT_MASK_P);
}

bool
ArmISA::Interrupts::takeInt(InterruptTypes int_type) const
{
    // Table G1-17~19 of ARM V8 ARM
    return ArmSystem::highestELIs64(tc) ? takeInt64(int_type) :
                                          takeInt32(int_type);

}

} // namespace gem5
