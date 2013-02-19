/*
 * Copyright (c) 2010 ARM Limited
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
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 *
 * Authors: Ali Saidi
 *          Gabe Black
 */

#include "arch/arm/faults.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/Faults.hh"
#include "sim/full_system.hh"

namespace ArmISA
{

template<> ArmFault::FaultVals ArmFaultVals<Reset>::vals =
{"reset", 0x00, MODE_SVC, 0, 0, true, true, FaultStat()};

template<> ArmFault::FaultVals ArmFaultVals<UndefinedInstruction>::vals =
{"Undefined Instruction", 0x04, MODE_UNDEFINED, 4 ,2, false, false,
 FaultStat()} ;

template<> ArmFault::FaultVals ArmFaultVals<SupervisorCall>::vals =
{"Supervisor Call", 0x08, MODE_SVC, 4, 2, false, false, FaultStat()};

template<> ArmFault::FaultVals ArmFaultVals<PrefetchAbort>::vals =
{"Prefetch Abort", 0x0C, MODE_ABORT, 4, 4, true, false, FaultStat()};

template<> ArmFault::FaultVals ArmFaultVals<DataAbort>::vals =
{"Data Abort", 0x10, MODE_ABORT, 8, 8, true, false, FaultStat()};

template<> ArmFault::FaultVals ArmFaultVals<Interrupt>::vals =
{"IRQ", 0x18, MODE_IRQ, 4, 4, true, false, FaultStat()};

template<> ArmFault::FaultVals ArmFaultVals<FastInterrupt>::vals =
{"FIQ", 0x1C, MODE_FIQ, 4, 4, true, true, FaultStat()};

template<> ArmFault::FaultVals ArmFaultVals<FlushPipe>::vals =
{"Pipe Flush", 0x00, MODE_SVC, 0, 0, true, true, FaultStat()}; // dummy values

template<> ArmFault::FaultVals ArmFaultVals<ArmSev>::vals =
{"ArmSev Flush", 0x00, MODE_SVC, 0, 0, true, true, FaultStat()}; // dummy values
Addr 
ArmFault::getVector(ThreadContext *tc)
{
    // ARM ARM B1-3

    SCTLR sctlr = tc->readMiscReg(MISCREG_SCTLR);

    // panic if SCTLR.VE because I have no idea what to do with vectored
    // interrupts
    assert(!sctlr.ve);

    if (!sctlr.v)
        return offset();
    return offset() + HighVecs;

}

void 
ArmFault::invoke(ThreadContext *tc, StaticInstPtr inst)
{
    // ARM ARM B1.6.3
    FaultBase::invoke(tc);
    if (!FullSystem)
        return;
    countStat()++;

    SCTLR sctlr = tc->readMiscReg(MISCREG_SCTLR);
    CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);
    CPSR saved_cpsr = tc->readMiscReg(MISCREG_CPSR);
    saved_cpsr.nz = tc->readIntReg(INTREG_CONDCODES_NZ);
    saved_cpsr.c = tc->readIntReg(INTREG_CONDCODES_C);
    saved_cpsr.v = tc->readIntReg(INTREG_CONDCODES_V);
    saved_cpsr.ge = tc->readIntReg(INTREG_CONDCODES_GE);

    Addr curPc M5_VAR_USED = tc->pcState().pc();
    ITSTATE it = tc->pcState().itstate();
    saved_cpsr.it2 = it.top6;
    saved_cpsr.it1 = it.bottom2;

    cpsr.mode = nextMode();
    cpsr.it1 = cpsr.it2 = 0;
    cpsr.j = 0;
   
    cpsr.t = sctlr.te;
    cpsr.a = cpsr.a | abortDisable();
    cpsr.f = cpsr.f | fiqDisable();
    cpsr.i = 1;
    cpsr.e = sctlr.ee;
    tc->setMiscReg(MISCREG_CPSR, cpsr);
    // Make sure mailbox sets to one always
    tc->setMiscReg(MISCREG_SEV_MAILBOX, 1);
    tc->setIntReg(INTREG_LR, curPc +
            (saved_cpsr.t ? thumbPcOffset() : armPcOffset()));

    switch (nextMode()) {
      case MODE_FIQ:
        tc->setMiscReg(MISCREG_SPSR_FIQ, saved_cpsr);
        break;
      case MODE_IRQ:
        tc->setMiscReg(MISCREG_SPSR_IRQ, saved_cpsr);
        break;
      case MODE_SVC:
        tc->setMiscReg(MISCREG_SPSR_SVC, saved_cpsr);
        break;
      case MODE_UNDEFINED:
        tc->setMiscReg(MISCREG_SPSR_UND, saved_cpsr);
        break;
      case MODE_ABORT:
        tc->setMiscReg(MISCREG_SPSR_ABT, saved_cpsr);
        break;
      default:
        panic("unknown Mode\n");
    }

    Addr newPc = getVector(tc);
    DPRINTF(Faults, "Invoking Fault:%s cpsr:%#x PC:%#x lr:%#x newVec: %#x\n",
            name(), cpsr, curPc, tc->readIntReg(INTREG_LR), newPc);
    PCState pc(newPc);
    pc.thumb(cpsr.t);
    pc.nextThumb(pc.thumb());
    pc.jazelle(cpsr.j);
    pc.nextJazelle(pc.jazelle());
    tc->pcState(pc);
}

void
Reset::invoke(ThreadContext *tc, StaticInstPtr inst)
{
    if (FullSystem) {
        tc->getCpuPtr()->clearInterrupts();
        tc->clearArchRegs();
    }
    ArmFault::invoke(tc, inst);
}

void
UndefinedInstruction::invoke(ThreadContext *tc, StaticInstPtr inst)
{
    if (FullSystem) {
        ArmFault::invoke(tc, inst);
        return;
    }

    // If the mnemonic isn't defined this has to be an unknown instruction.
    assert(unknown || mnemonic != NULL);
    if (disabled) {
        panic("Attempted to execute disabled instruction "
                "'%s' (inst 0x%08x)", mnemonic, machInst);
    } else if (unknown) {
        panic("Attempted to execute unknown instruction (inst 0x%08x)",
              machInst);
    } else {
        panic("Attempted to execute unimplemented instruction "
                "'%s' (inst 0x%08x)", mnemonic, machInst);
    }
}

void
SupervisorCall::invoke(ThreadContext *tc, StaticInstPtr inst)
{
    if (FullSystem) {
        ArmFault::invoke(tc, inst);
        return;
    }

    // As of now, there isn't a 32 bit thumb version of this instruction.
    assert(!machInst.bigThumb);
    uint32_t callNum;
    callNum = tc->readIntReg(INTREG_R7);
    tc->syscall(callNum);

    // Advance the PC since that won't happen automatically.
    PCState pc = tc->pcState();
    assert(inst);
    inst->advancePC(pc);
    tc->pcState(pc);
}

template<class T>
void
AbortFault<T>::invoke(ThreadContext *tc, StaticInstPtr inst)
{
    ArmFaultVals<T>::invoke(tc, inst);
    FSR fsr = 0;
    fsr.fsLow = bits(status, 3, 0);
    fsr.fsHigh = bits(status, 4);
    fsr.domain = domain;
    fsr.wnr = (write ? 1 : 0);
    fsr.ext = 0;
    tc->setMiscReg(T::FsrIndex, fsr);
    tc->setMiscReg(T::FarIndex, faultAddr);

    DPRINTF(Faults, "Abort Fault fsr=%#x faultAddr=%#x\n", fsr, faultAddr);
}

void
FlushPipe::invoke(ThreadContext *tc, StaticInstPtr inst) {
    DPRINTF(Faults, "Invoking FlushPipe Fault\n");

    // Set the PC to the next instruction of the faulting instruction.
    // Net effect is simply squashing all instructions behind and
    // start refetching from the next instruction.
    PCState pc = tc->pcState();
    assert(inst);
    inst->advancePC(pc);
    tc->pcState(pc);
}

template void AbortFault<PrefetchAbort>::invoke(ThreadContext *tc,
                                                StaticInstPtr inst);
template void AbortFault<DataAbort>::invoke(ThreadContext *tc,
                                            StaticInstPtr inst);

void
ArmSev::invoke(ThreadContext *tc, StaticInstPtr inst) {
    DPRINTF(Faults, "Invoking ArmSev Fault\n");
    if (!FullSystem)
        return;

    // Set sev_mailbox to 1, clear the pending interrupt from remote
    // SEV execution and let pipeline continue as pcState is still
    // valid.
    tc->setMiscReg(MISCREG_SEV_MAILBOX, 1);
    tc->getCpuPtr()->clearInterrupt(INT_SEV, 0);
}

// return via SUBS pc, lr, xxx; rfe, movs, ldm

} // namespace ArmISA
