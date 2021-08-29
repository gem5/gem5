/*
 * Copyright (c) 2012 ARM Limited
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

#include "arch/power/isa.hh"

#include "arch/power/regs/float.hh"
#include "arch/power/regs/int.hh"
#include "arch/power/regs/misc.hh"
#include "cpu/thread_context.hh"
#include "debug/FloatRegs.hh"
#include "debug/IntRegs.hh"
#include "debug/MiscRegs.hh"
#include "params/PowerISA.hh"

namespace gem5
{

namespace PowerISA
{

ISA::ISA(const Params &p) : BaseISA(p)
{
    _regClasses.emplace_back(int_reg::NumRegs, debug::IntRegs);
    _regClasses.emplace_back(float_reg::NumRegs, debug::FloatRegs);
    _regClasses.emplace_back(1, debug::IntRegs);
    _regClasses.emplace_back(2, debug::IntRegs);
    _regClasses.emplace_back(1, debug::IntRegs);
    _regClasses.emplace_back(0, debug::IntRegs);
    _regClasses.emplace_back(NUM_MISCREGS, debug::MiscRegs);
    clear();
}

void
ISA::copyRegsFrom(ThreadContext *src)
{
    // First loop through the integer registers.
    for (int i = 0; i < int_reg::NumRegs; ++i) {
        RegId reg(IntRegClass, i);
        tc->setReg(reg, src->getReg(reg));
    }

    // Then loop through the floating point registers.
    for (int i = 0; i < float_reg::NumRegs; ++i) {
        RegId reg(FloatRegClass, i);
        tc->setReg(reg, src->getReg(reg));
    }

    //TODO Copy misc. registers

    // Lastly copy PC/NPC
    tc->pcState(src->pcState());
}

} // namespace PowerISA
} // namespace gem5
