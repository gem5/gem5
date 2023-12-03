/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
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

#include "arch/x86/insts/microop.hh"

#include "arch/x86/regs/misc.hh"

namespace gem5
{

namespace X86ISA
{

bool
X86MicroopBase::checkCondition(uint64_t flags, int condition) const
{
    CCFlagBits ccflags = flags;
    switch (condition) {
    case condition_tests::True:
        return true;
    case condition_tests::ECF:
        return ccflags.ecf;
    case condition_tests::EZF:
        return ccflags.ezf;
    case condition_tests::SZnZF:
        return !(!ccflags.ezf && ccflags.zf);
    case condition_tests::MSTRZ:
        panic("This condition is not implemented!");
    case condition_tests::STRZ:
        panic("This condition is not implemented!");
    case condition_tests::MSTRC:
        panic("This condition is not implemented!");
    case condition_tests::STRZnEZF:
        return !ccflags.ezf && ccflags.zf;
        // And no interrupts or debug traps are waiting
    case condition_tests::OF:
        return ccflags.of;
    case condition_tests::CF:
        return ccflags.cf;
    case condition_tests::ZF:
        return ccflags.zf;
    case condition_tests::CvZF:
        return ccflags.cf | ccflags.zf;
    case condition_tests::SF:
        return ccflags.sf;
    case condition_tests::PF:
        return ccflags.pf;
    case condition_tests::SxOF:
        return ccflags.sf ^ ccflags.of;
    case condition_tests::SxOvZF:
        return (ccflags.sf ^ ccflags.of) | ccflags.zf;
    case condition_tests::False:
        return false;
    case condition_tests::NotECF:
        return !ccflags.ecf;
    case condition_tests::NotEZF:
        return !ccflags.ezf;
    case condition_tests::NotSZnZF:
        return !ccflags.ezf && ccflags.zf;
    case condition_tests::NotMSTRZ:
        panic("This condition is not implemented!");
    case condition_tests::NotSTRZ:
        panic("This condition is not implemented!");
    case condition_tests::NotMSTRC:
        panic("This condition is not implemented!");
    case condition_tests::STRnZnEZF:
        return !ccflags.ezf && !ccflags.zf;
        // And no interrupts or debug traps are waiting
    case condition_tests::NotOF:
        return !ccflags.of;
    case condition_tests::NotCF:
        return !ccflags.cf;
    case condition_tests::NotZF:
        return !ccflags.zf;
    case condition_tests::NotCvZF:
        return !(ccflags.cf | ccflags.zf);
    case condition_tests::NotSF:
        return !ccflags.sf;
    case condition_tests::NotPF:
        return !ccflags.pf;
    case condition_tests::NotSxOF:
        return !(ccflags.sf ^ ccflags.of);
    case condition_tests::NotSxOvZF:
        return !((ccflags.sf ^ ccflags.of) | ccflags.zf);
    }
    panic("Unknown condition: %d\n", condition);
    return true;
}

std::unique_ptr<PCStateBase>
X86MicroopBase::branchTarget(const PCStateBase &branch_pc) const
{
    PCStateBase *pcs = branch_pc.clone();
    DPRINTF(X86, "branchTarget PC info: %s, Immediate: %lx\n", *pcs,
            (int64_t)machInst.immediate);
    auto &xpc = pcs->as<PCState>();
    xpc.npc(xpc.npc() + (int64_t)machInst.immediate);
    xpc.uEnd();
    return std::unique_ptr<PCStateBase>{ pcs };
}

} // namespace X86ISA
} // namespace gem5
