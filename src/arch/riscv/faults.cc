/*
 * Copyright (c) 2016 RISC-V Foundation
 * Copyright (c) 2016 The University of Virginia
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
 *
 * Authors: Alec Roelke
 */
#include "arch/riscv/faults.hh"

#include "arch/riscv/utility.hh"
#include "cpu/thread_context.hh"
#include "sim/debug.hh"
#include "sim/full_system.hh"

using namespace RiscvISA;

void
RiscvFault::invoke_se(ThreadContext *tc, const StaticInstPtr &inst)
{
    panic("Fault %s encountered at pc 0x%016llx.", name(), tc->pcState().pc());
}

void
RiscvFault::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    if (FullSystem) {
        panic("Full system mode not supported for RISC-V.");
    } else {
        invoke_se(tc, inst);
        PCState pcState = tc->pcState();
        advancePC(pcState, inst);
        tc->pcState(pcState);
    }
}

void
UnknownInstFault::invoke_se(ThreadContext *tc, const StaticInstPtr &inst)
{
    panic("Unknown instruction 0x%08x at pc 0x%016llx", inst->machInst,
        tc->pcState().pc());
}

void
IllegalInstFault::invoke_se(ThreadContext *tc, const StaticInstPtr &inst)
{
    panic("Illegal instruction 0x%08x at pc 0x%016llx: %s", inst->machInst,
        tc->pcState().pc(), reason.c_str());
}

void
UnimplementedFault::invoke_se(ThreadContext *tc,
        const StaticInstPtr &inst)
{
    panic("Unimplemented instruction %s at pc 0x%016llx", instName,
        tc->pcState().pc());
}

void
IllegalFrmFault::invoke_se(ThreadContext *tc, const StaticInstPtr &inst)
{
    panic("Illegal floating-point rounding mode 0x%x at pc 0x%016llx.",
            frm, tc->pcState().pc());
}

void
BreakpointFault::invoke_se(ThreadContext *tc, const StaticInstPtr &inst)
{
    schedRelBreak(0);
}

void
SyscallFault::invoke_se(ThreadContext *tc, const StaticInstPtr &inst)
{
    Fault *fault = NoFault;
    tc->syscall(tc->readIntReg(SyscallNumReg), fault);
}
