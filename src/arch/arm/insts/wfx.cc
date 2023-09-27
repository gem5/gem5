/*
 * Copyright (c) 2010-2014, 2016-2020,2022, 2026 Arm Limited
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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
 */

#include "arch/arm/insts/wfx.hh"

#include "arch/arm/faults.hh"
#include "arch/arm/insts/pred_inst.hh"
#include "arch/arm/insts/static_inst.hh"
#include "arch/arm/system.hh"
#include "cpu/thread_context.hh"

namespace gem5
{

using namespace ArmISA;

inline bool
WFxOpBase::isWFxTrapping(ThreadContext *tc, ExceptionLevel tgtEl,
                         bool isWfe) const
{
    bool trap = false;
    SCTLR sctlr = ((SCTLR)tc->readMiscReg(MISCREG_SCTLR_EL1));
    HCR hcr = ((HCR)tc->readMiscReg(MISCREG_HCR_EL2));
    SCR scr = ((SCR)tc->readMiscReg(MISCREG_SCR_EL3));

    switch (tgtEl) {
        case EL1:
            trap = isWfe ? !sctlr.ntwe : !sctlr.ntwi;
            break;
        case EL2:
            trap = isWfe ? hcr.twe : hcr.twi;
            break;
        case EL3:
            trap = isWfe ? scr.twe : scr.twi;
            break;
        default:
            break;
    }

    return trap;
}

Fault
WFxOpBase::checkForWFxTrap32(ThreadContext *tc, ExceptionLevel targetEL,
                             const ArmISA::ArmStaticInst &inst,
                             bool isWfe) const
{
    // Check if target exception level is implemented.
    assert(ArmSystem::haveEL(tc, targetEL));

    // Check for routing to AArch64: this happens if the
    // target exception level (where the trap will be handled)
    // is using aarch64
    if (ELIs64(tc, targetEL)) {
        return checkForWFxTrap64(tc, targetEL, inst, isWfe);
    }

    // Check if processor needs to trap at selected exception level
    bool trap = isWFxTrapping(tc, targetEL, isWfe);

    if (trap) {
        uint32_t iss = isWfe ? 0x1E00001 : /* WFE Instruction syndrome */
                           0x1E00000;      /* WFI Instruction syndrome */
        return inst.generateTrap(targetEL, ExceptionClass::TRAPPED_WFI_WFE,
                                 iss);
    } else {
        return NoFault;
    }
}

Fault
WFxOpBase::checkForWFxTrap64(ThreadContext *tc, ExceptionLevel targetEL,
                             const ArmISA::ArmStaticInst &inst,
                             bool isWfe) const
{
    // Check if target exception level is implemented.
    assert(ArmSystem::haveEL(tc, targetEL));

    // Check if processor needs to trap at selected exception level
    bool trap = isWFxTrapping(tc, targetEL, isWfe);

    if (trap) {
        uint32_t iss = isWfe ? 0x1E00001 : /* WFE Instruction syndrome */
                           0x1E00000;      /* WFI Instruction syndrome */
        return inst.generateTrap(targetEL, ExceptionClass::TRAPPED_WFI_WFE,
                                 iss);
    } else {
        return NoFault;
    }
}

Fault
WFxOpBase::trapWFx(ThreadContext *tc, CPSR cpsr, SCR scr,
                   const ArmISA::ArmStaticInst &inst, bool isWfe) const
{
    Fault fault = NoFault;
    ExceptionLevel curr_el = currEL(tc);

    if (curr_el == EL0) {
        fault = checkForWFxTrap32(tc, EL1, inst, isWfe);
    }

    if ((fault == NoFault) && EL2Enabled(tc) &&
        ((curr_el == EL0 && !ELIsInHost(tc, curr_el)) || (curr_el == EL1))) {

        fault = checkForWFxTrap32(tc, EL2, inst, isWfe);
    }

    if ((fault == NoFault) && ArmSystem::haveEL(tc, EL3) && curr_el != EL3) {
        fault = checkForWFxTrap32(tc, EL3, inst, isWfe);
    }

    return fault;
}

} // namespace gem5
