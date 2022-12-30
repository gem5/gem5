/*
 * Copyright (c) 2010-2014 ARM Limited
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
 * Copyright (c) 2009 The Regents of The University of Michigan
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

#include "arch/arm/regs/int.hh"

#include "arch/arm/isa.hh"
#include "arch/arm/regs/misc.hh"
#include "arch/arm/utility.hh"
#include "base/logging.hh"

namespace gem5
{

namespace ArmISA
{

RegId
IntRegClassOps::flatten(const BaseISA &isa, const RegId &id) const
{
    const RegIndex reg_idx = id.index();

    auto &arm_isa = static_cast<const ArmISA::ISA &>(isa);

    if (reg_idx < int_reg::NumArchRegs) {
        return {flatIntRegClass, arm_isa.mapIntRegId(reg_idx)};
    } else if (reg_idx < int_reg::NumRegs) {
        return {flatIntRegClass, id};
    } else if (reg_idx == int_reg::Spx) {
        auto &arm_isa = static_cast<const ArmISA::ISA &>(isa);
        CPSR cpsr = arm_isa.readMiscRegNoEffect(MISCREG_CPSR);
        ExceptionLevel el = opModeToEL((OperatingMode)(uint8_t)cpsr.mode);

        if (!cpsr.sp && el != EL0)
            return {flatIntRegClass, int_reg::Sp0};

        switch (el) {
          case EL3:
            return {flatIntRegClass, int_reg::Sp3};
          case EL2:
            return {flatIntRegClass, int_reg::Sp2};
          case EL1:
            return {flatIntRegClass, int_reg::Sp1};
          case EL0:
            return {flatIntRegClass, int_reg::Sp0};
          default:
            panic("Invalid exception level");
        }
    } else {
        return {flatIntRegClass, flattenIntRegModeIndex(reg_idx)};
    }
}

} // namespace ArmISA
} // namespace gem5
