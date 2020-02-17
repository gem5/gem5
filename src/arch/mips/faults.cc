/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
 * Copyright (c) 2007 MIPS Technologies, Inc.
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

#include "arch/mips/faults.hh"

#include "arch/mips/pra_constants.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/MipsPRA.hh"
#include "mem/page_table.hh"
#include "sim/process.hh"

namespace MipsISA
{

typedef MipsFaultBase::FaultVals FaultVals;

template <> FaultVals MipsFault<SystemCallFault>::vals =
    { "Syscall", 0x180, ExcCodeSys };

template <> FaultVals MipsFault<ReservedInstructionFault>::vals =
    { "Reserved Instruction Fault", 0x180, ExcCodeRI };

template <> FaultVals MipsFault<ThreadFault>::vals =
    { "Thread Fault", 0x180, ExcCodeDummy };

template <> FaultVals MipsFault<IntegerOverflowFault>::vals =
    { "Integer Overflow Exception", 0x180, ExcCodeOv };

template <> FaultVals MipsFault<TrapFault>::vals =
    { "Trap", 0x180, ExcCodeTr };

template <> FaultVals MipsFault<BreakpointFault>::vals =
    { "Breakpoint", 0x180, ExcCodeBp };

template <> FaultVals MipsFault<DspStateDisabledFault>::vals =
    { "DSP Disabled Fault", 0x180, ExcCodeDummy };

template <> FaultVals MipsFault<MachineCheckFault>::vals =
    { "Machine Check", 0x180, ExcCodeMCheck };

template <> FaultVals MipsFault<ResetFault>::vals =
    { "Reset Fault", 0x000, ExcCodeDummy };

template <> FaultVals MipsFault<SoftResetFault>::vals =
    { "Soft Reset Fault", 0x000, ExcCodeDummy };

template <> FaultVals MipsFault<NonMaskableInterrupt>::vals =
    { "Non Maskable Interrupt", 0x000, ExcCodeDummy };

template <> FaultVals MipsFault<CoprocessorUnusableFault>::vals =
    { "Coprocessor Unusable Fault", 0x180, ExcCodeCpU };

template <> FaultVals MipsFault<InterruptFault>::vals =
    { "Interrupt", 0x000, ExcCodeInt };

template <> FaultVals MipsFault<AddressErrorFault>::vals =
    { "Address Error", 0x180, ExcCodeDummy };

template <> FaultVals MipsFault<TlbInvalidFault>::vals =
    { "Invalid TLB Entry Exception", 0x180, ExcCodeDummy };

template <> FaultVals MipsFault<TlbRefillFault>::vals =
    { "TLB Refill Exception", 0x180, ExcCodeDummy };

template <> MipsFaultBase::FaultVals MipsFault<TlbModifiedFault>::vals =
    { "TLB Modified Exception", 0x180, ExcCodeMod };

void
MipsFaultBase::setExceptionState(ThreadContext *tc, uint8_t excCode)
{
    // modify SRS Ctl - Save CSS, put ESS into CSS
    StatusReg status = tc->readMiscReg(MISCREG_STATUS);
    if (status.exl != 1 && status.bev != 1) {
        // SRS Ctl is modified only if Status_EXL and Status_BEV are not set
        SRSCtlReg srsCtl = tc->readMiscReg(MISCREG_SRSCTL);
        srsCtl.pss = srsCtl.css;
        srsCtl.css = srsCtl.ess;
        tc->setMiscRegNoEffect(MISCREG_SRSCTL, srsCtl);
    }

    // set EXL bit (don't care if it is already set!)
    status.exl = 1;
    tc->setMiscRegNoEffect(MISCREG_STATUS, status);

    // write EPC
    PCState pc = tc->pcState();
    DPRINTF(MipsPRA, "PC: %s\n", pc);
    bool delay_slot = pc.pc() + sizeof(MachInst) != pc.npc();
    tc->setMiscRegNoEffect(MISCREG_EPC,
            pc.pc() - (delay_slot ? sizeof(MachInst) : 0));

    // Set Cause_EXCCODE field
    CauseReg cause = tc->readMiscReg(MISCREG_CAUSE);
    cause.excCode = excCode;
    cause.bd = delay_slot ? 1 : 0;
    cause.ce = 0;
    tc->setMiscRegNoEffect(MISCREG_CAUSE, cause);
}

void
MipsFaultBase::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    if (FullSystem) {
        DPRINTF(MipsPRA, "Fault %s encountered.\n", name());
        setExceptionState(tc, code());
        tc->pcState(vect(tc));
    } else {
        panic("Fault %s encountered.\n", name());
    }
}

void
ResetFault::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    if (FullSystem) {
        DPRINTF(MipsPRA, "%s encountered.\n", name());
        /* All reset activity must be invoked from here */
        Addr handler = vect(tc);
        tc->pcState(handler);
        DPRINTF(MipsPRA, "ResetFault::invoke : PC set to %x", handler);
    }

    // Set Coprocessor 1 (Floating Point) To Usable
    StatusReg status = tc->readMiscRegNoEffect(MISCREG_STATUS);
    status.cu.cu1 = 1;
    tc->setMiscReg(MISCREG_STATUS, status);
}

void
SoftResetFault::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    panic("Soft reset not implemented.\n");
}

void
NonMaskableInterrupt::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    panic("Non maskable interrupt not implemented.\n");
}

} // namespace MipsISA

