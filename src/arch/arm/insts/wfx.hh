/*
 * Copyright (c) 2010-2013,2016-2018, 2022, 2025-2026 Arm Limited
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

#ifndef __ARCH_ARM_INSTS_WFX_HH__
#define __ARCH_ARM_INSTS_WFX_HH__

#include "arch/arm/insts/static_inst.hh"
#include "arch/arm/regs/misc_types.hh"
#include "arch/arm/types.hh"

namespace gem5
{

class ThreadContext;

class WFxOpBase
{
  protected:
    enum class Type
    {
        Wfe,
        Wfi
    };

    // Utility function used by checkForWFxTrap32 and checkForWFxTrap64
    // Returns true if processor has to trap a WFI/WFE instruction.
    bool isWFxTrapping(ThreadContext *tc, ArmISA::ExceptionLevel target_el,
                       Type wfx_type) const;

    /**
     * Check if WFE/WFI instruction execution in aarch32 should be trapped.
     *
     * See aarch32/exceptions/traps/AArch32.checkForWFxTrap in the
     * ARM ARM psueodcode library.
     */
    Fault checkForWFxTrap32(ThreadContext *tc,
                            ArmISA::ExceptionLevel target_el,
                            const ArmISA::ArmStaticInst &inst,
                            Type wfx_type) const;

    /**
     * Check if WFE/WFI instruction execution in aarch64 should be trapped.
     *
     * See aarch64/exceptions/traps/AArch64.checkForWFxTrap in the
     * ARM ARM psueodcode library.
     */
    Fault checkForWFxTrap64(ThreadContext *tc,
                            ArmISA::ExceptionLevel target_el,
                            const ArmISA::ArmStaticInst &inst,
                            Type wfx_type) const;

    /**
     * WFE/WFI trapping helper function.
     */
    Fault trapWFx(ThreadContext *tc, ArmISA::CPSR cpsr, ArmISA::SCR scr,
                  const ArmISA::ArmStaticInst &inst, Type wfx_type) const;
};

} // namespace gem5

#endif // __ARCH_ARM_INSTS_WFX_HH__
