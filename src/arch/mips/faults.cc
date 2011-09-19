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
 *
 * Authors: Gabe Black
 *          Korey Sewell
 *          Jaidev Patwardhan
 */

#include "arch/mips/faults.hh"
#include "arch/mips/pra_constants.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/MipsPRA.hh"

#if !FULL_SYSTEM
#include "mem/page_table.hh"
#include "sim/process.hh"
#endif

namespace MipsISA
{

typedef MipsFaultBase::FaultVals FaultVals;

template <> FaultVals MipsFault<MachineCheckFault>::vals =
    { "Machine Check", 0x0401 };

template <> FaultVals MipsFault<ResetFault>::vals =
#if  FULL_SYSTEM
    { "Reset Fault", 0xBFC00000};
#else
    { "Reset Fault", 0x001};
#endif

template <> FaultVals MipsFault<AddressErrorFault>::vals =
    { "Address Error", 0x0180 };

template <> FaultVals MipsFault<SystemCallFault>::vals =
    { "Syscall", 0x0180 };

template <> FaultVals MipsFault<CoprocessorUnusableFault>::vals =
    { "Coprocessor Unusable Fault", 0x180 };

template <> FaultVals MipsFault<ReservedInstructionFault>::vals =
    { "Reserved Instruction Fault", 0x0180 };

template <> FaultVals MipsFault<ThreadFault>::vals =
    { "Thread Fault", 0x00F1 };

template <> FaultVals MipsFault<IntegerOverflowFault>::vals =
    { "Integer Overflow Exception", 0x180 };

template <> FaultVals MipsFault<InterruptFault>::vals =
    { "interrupt", 0x0180 };

template <> FaultVals MipsFault<TrapFault>::vals =
    { "Trap", 0x0180 };

template <> FaultVals MipsFault<BreakpointFault>::vals =
    { "Breakpoint", 0x0180 };

template <> FaultVals MipsFault<TlbInvalidFault>::vals =
    { "Invalid TLB Entry Exception", 0x0180 };

template <> FaultVals MipsFault<TlbRefillFault>::vals =
    { "TLB Refill Exception", 0x0180 };

template <> FaultVals MipsFault<TlbModifiedFault>::vals =
    { "TLB Modified Exception", 0x0180 };

template <> FaultVals MipsFault<DspStateDisabledFault>::vals =
    { "DSP Disabled Fault", 0x001a };

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
            pc.pc() - delay_slot ? sizeof(MachInst) : 0);

    // Set Cause_EXCCODE field
    CauseReg cause = tc->readMiscReg(MISCREG_CAUSE);
    cause.excCode = excCode;
    cause.bd = delay_slot ? 1 : 0;
    cause.ce = 0;
    tc->setMiscRegNoEffect(MISCREG_CAUSE, cause);
}

#if FULL_SYSTEM

void
IntegerOverflowFault::invoke(ThreadContext *tc, StaticInstPtr inst)
{
    DPRINTF(MipsPRA, "%s encountered.\n", name());
    setExceptionState(tc, 0xC);

    // Set new PC
    StatusReg status = tc->readMiscReg(MISCREG_STATUS);
    if (!status.bev) {
        // See MIPS ARM Vol 3, Revision 2, Page 38
        tc->pcState(vect() + tc->readMiscReg(MISCREG_EBASE));
    } else {
        tc->pcState(0xBFC00200);
    }
}

void
TrapFault::invoke(ThreadContext *tc, StaticInstPtr inst)
{
    DPRINTF(MipsPRA, "%s encountered.\n", name());
    setExceptionState(tc, 0xD);

    tc->pcState(vect() + tc->readMiscReg(MISCREG_EBASE));
}

void
BreakpointFault::invoke(ThreadContext *tc, StaticInstPtr inst)
{
    setExceptionState(tc, 0x9);

    tc->pcState(vect() + tc->readMiscReg(MISCREG_EBASE));
}

void
AddressErrorFault::invoke(ThreadContext *tc, StaticInstPtr inst)
{
    DPRINTF(MipsPRA, "%s encountered.\n", name());
    setExceptionState(tc, store ? 0x5 : 0x4);
    tc->setMiscRegNoEffect(MISCREG_BADVADDR, vaddr);

    tc->pcState(vect() + tc->readMiscReg(MISCREG_EBASE));
}

void
TlbInvalidFault::invoke(ThreadContext *tc, StaticInstPtr inst)
{
    setTlbExceptionState(tc, store ? 0x3 : 0x2);
    tc->pcState(vect() + tc->readMiscReg(MISCREG_EBASE));
}

void
TlbRefillFault::invoke(ThreadContext *tc, StaticInstPtr inst)
{
    // Since handler depends on EXL bit, must check EXL bit before setting it!!
    StatusReg status = tc->readMiscReg(MISCREG_STATUS);

    setTlbExceptionState(tc, store ? 0x3 : 0x2);

    // See MIPS ARM Vol 3, Revision 2, Page 38
    if (status.exl == 1) {
        tc->pcState(vect() + tc->readMiscReg(MISCREG_EBASE));
    } else {
        tc->pcState(tc->readMiscReg(MISCREG_EBASE));
    }
}

void
TlbModifiedFault::invoke(ThreadContext *tc, StaticInstPtr inst)
{
    setTlbExceptionState(tc, 0x1);

    tc->pcState(vect() + tc->readMiscReg(MISCREG_EBASE));
}

void
SystemCallFault::invoke(ThreadContext *tc, StaticInstPtr inst)
{
    DPRINTF(MipsPRA, "%s encountered.\n", name());
    setExceptionState(tc, 0x8);

    tc->pcState(vect() + tc->readMiscReg(MISCREG_EBASE));
}

void
InterruptFault::invoke(ThreadContext *tc, StaticInstPtr inst)
{
    DPRINTF(MipsPRA, "%s encountered.\n", name());
    setExceptionState(tc, 0x0A);

    CauseReg cause = tc->readMiscRegNoEffect(MISCREG_CAUSE);
    if (cause.iv) {
        // Offset 200 for release 2
        tc->pcState(0x20 + vect() + tc->readMiscRegNoEffect(MISCREG_EBASE));
    } else {
        //Ofset at 180 for release 1
        tc->pcState(vect() + tc->readMiscRegNoEffect(MISCREG_EBASE));
    }
}

#endif // FULL_SYSTEM

void
ResetFault::invoke(ThreadContext *tc, StaticInstPtr inst)
{
#if FULL_SYSTEM
    DPRINTF(MipsPRA, "%s encountered.\n", name());
    /* All reset activity must be invoked from here */
    tc->pcState(vect());
    DPRINTF(MipsPRA, "ResetFault::invoke : PC set to %x", tc->readPC());
#endif

    // Set Coprocessor 1 (Floating Point) To Usable
    StatusReg status = tc->readMiscRegNoEffect(MISCREG_STATUS);
    status.cu.cu1 = 1;
    tc->setMiscReg(MISCREG_STATUS, status);
}

void
ReservedInstructionFault::invoke(ThreadContext *tc, StaticInstPtr inst)
{
#if  FULL_SYSTEM
    DPRINTF(MipsPRA, "%s encountered.\n", name());
    setExceptionState(tc, 0x0A);
    tc->pcState(vect() + tc->readMiscRegNoEffect(MISCREG_EBASE));
#else
    panic("%s encountered.\n", name());
#endif
}

void
ThreadFault::invoke(ThreadContext *tc, StaticInstPtr inst)
{
    DPRINTF(MipsPRA, "%s encountered.\n", name());
    panic("%s encountered.\n", name());
}

void
DspStateDisabledFault::invoke(ThreadContext *tc, StaticInstPtr inst)
{
    DPRINTF(MipsPRA, "%s encountered.\n", name());
    panic("%s encountered.\n", name());
}

void
CoprocessorUnusableFault::invoke(ThreadContext *tc, StaticInstPtr inst)
{
#if FULL_SYSTEM
    DPRINTF(MipsPRA, "%s encountered.\n", name());
    setExceptionState(tc, 0xb);
    // The ID of the coprocessor causing the exception is stored in
    // CoprocessorUnusableFault::coProcID
    CauseReg cause = tc->readMiscReg(MISCREG_CAUSE);
    cause.ce = coProcID;
    tc->setMiscRegNoEffect(MISCREG_CAUSE, cause);
    tc->pcState(vect() + tc->readMiscReg(MISCREG_EBASE));
#else
    warn("%s (CP%d) encountered.\n", name(), coProcID);
#endif
}

} // namespace MipsISA

