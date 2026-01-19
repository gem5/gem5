/*
 * Copyright (c) 2010-2013, 2015-2025 Arm Limited
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

#include "arch/arm/regs/misc_info.hh"

#include "arch/arm/insts/misc64.hh"
#include "arch/arm/regs/misc.hh"

namespace gem5
{

namespace ArmISA
{

std::vector<struct MiscRegLUTEntry> lookUpMiscReg(NUM_MISCREGS);

MiscRegLUTEntryInitializer::chain
MiscRegLUTEntryInitializer::highest(ArmSystem *const sys) const
{
    switch (FullSystem ? sys->highestEL() : EL1) {
        case EL0:
        case EL1:
            priv();
            break;
        case EL2:
            hyp();
            break;
        case EL3:
            mon();
            break;
    }
    return *this;
}

Fault
MiscRegLUTEntry::checkFault(ThreadContext *tc, const MiscRegOp64 &inst,
                            ExceptionLevel el)
{
    return !inst.miscRead() ? faultWrite[el](*this, tc, inst)
                            : faultRead[el](*this, tc, inst);
}

template <MiscRegInfo Sec, MiscRegInfo NonSec>
Fault
MiscRegLUTEntry::defaultFault(const MiscRegLUTEntry &entry, ThreadContext *tc,
                              const MiscRegOp64 &inst)
{
    if (isSecureBelowEL3(tc) ? entry.info[Sec] : entry.info[NonSec]) {
        return NoFault;
    } else {
        return inst.undefined();
    }
}

} // namespace ArmISA

} // namespace gem5
