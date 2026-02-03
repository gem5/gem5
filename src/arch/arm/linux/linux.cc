/*
 * Copyright 2025 Google Inc.
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

#include "arch/arm/linux/linux.hh"

#include "arch/arm/pcstate.hh"
#include "arch/arm/regs/misc.hh"
#include "arch/arm/utility.hh"
#include "base/types.hh"
#include "cpu/thread_context.hh"
#include "mem/se_translating_port_proxy.hh"

namespace gem5
{

void
ArmLinux64::archSigreturn(ThreadContext *ctc)
{
    Addr sp = ctc->getReg(ArmISA::int_reg::Sp0);
    SETranslatingPortProxy proxy(ctc);

    tgt_sigframe frame;
    proxy.readBlob(sp, &frame, sizeof(frame));

    RegVal new_sp = frame.uc.uc_mcontext.sp;
    RegVal new_pc = frame.uc.uc_mcontext.pc;
    RegVal _newpsr = frame.uc.uc_mcontext.pstate;

    for (int i = 0; i < 31; ++i) {
        ctc->setReg(ArmISA::int_reg::x(i), frame.uc.uc_mcontext.regs[i]);
    }
    auto pc_state = ctc->pcState().as<ArmISA::PCState>();
    pc_state.set(new_pc);
    ctc->pcState(pc_state);
    ctc->setMiscRegNoEffect(ArmISA::MISCREG_CPSR, _newpsr);
    // Update the stack pointer. This should be done after
    // updating CPSR/PSTATE since that might affect how SPX gets
    // mapped.
    ctc->setReg(ArmISA::int_reg::Spx, new_sp);
}

} // namespace gem5
