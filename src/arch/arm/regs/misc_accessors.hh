/*
 * Copyright (c) 2024 Arm Limited
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

#ifndef __ARCH_ARM_REGS_MISC_ACCESSORS_HH__
#define __ARCH_ARM_REGS_MISC_ACCESSORS_HH__

#include "arch/arm/regs/misc_types.hh"
#include "cpu/thread_context.hh"

namespace gem5
{

namespace ArmISA
{

namespace misc_regs
{

struct FarAccessor
{
    using type = RegVal;
    static const MiscRegIndex el0 = NUM_MISCREGS;
    static const MiscRegIndex el1 = MISCREG_FAR_EL1;
    static const MiscRegIndex el2 = MISCREG_FAR_EL2;
    static const MiscRegIndex el3 = MISCREG_FAR_EL3;
};

struct MpamAccessor
{
    using type = MPAM;
    static const MiscRegIndex el0 = MISCREG_MPAM0_EL1;
    static const MiscRegIndex el1 = MISCREG_MPAM1_EL1;
    static const MiscRegIndex el2 = MISCREG_MPAM2_EL2;
    static const MiscRegIndex el3 = MISCREG_MPAM3_EL3;
};

template <typename RegAccessor>
MiscRegIndex
getRegVersion(ExceptionLevel el)
{
    switch (el) {
      case EL0:
        return RegAccessor::el0;
      case EL1:
        return RegAccessor::el1;
      case EL2:
        return RegAccessor::el2;
      case EL3:
        return RegAccessor::el3;
      default:
        panic("Invalid EL\n");
    }
}

template <typename RegAccessor>
typename RegAccessor::type
readRegister(ThreadContext *tc, ExceptionLevel el)
{
    return tc->readMiscReg(getRegVersion<RegAccessor>(el));
}

template <typename RegAccessor>
typename RegAccessor::type
readRegisterNoEffect(ThreadContext *tc, ExceptionLevel el)
{
    return tc->readMiscRegNoEffect(getRegVersion<RegAccessor>(el));
}

template <typename RegAccessor>
void
writeRegister(ThreadContext *tc, RegVal val, ExceptionLevel el)
{
    tc->setMiscReg(getRegVersion<RegAccessor>(el), val);
}

} // namespace misc_regs
} // namespace ArmISA
} // namespace gem5

#endif // __ARCH_ARM_REGS_MISC_ACCESSORS_HH__
