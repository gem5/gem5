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

#if !FULL_SYSTEM
#include "mem/page_table.hh"
#include "sim/process.hh"
#endif

namespace MipsISA
{

FaultName MachineCheckFault::_name = "Machine Check";
FaultVect MachineCheckFault::_vect = 0x0401;
FaultStat MachineCheckFault::_count;

FaultName AlignmentFault::_name = "Alignment";
FaultVect AlignmentFault::_vect = 0x0301;
FaultStat AlignmentFault::_count;

FaultName ResetFault::_name = "Reset Fault";
#if  FULL_SYSTEM
FaultVect ResetFault::_vect = 0xBFC00000;
#else
FaultVect ResetFault::_vect = 0x001;
#endif
FaultStat ResetFault::_count;

FaultName AddressErrorFault::_name = "Address Error";
FaultVect AddressErrorFault::_vect = 0x0180;
FaultStat AddressErrorFault::_count;

FaultName StoreAddressErrorFault::_name = "Store Address Error";
FaultVect StoreAddressErrorFault::_vect = 0x0180;
FaultStat StoreAddressErrorFault::_count;


FaultName SystemCallFault::_name = "Syscall";
FaultVect SystemCallFault::_vect = 0x0180;
FaultStat SystemCallFault::_count;

FaultName CoprocessorUnusableFault::_name = "Coprocessor Unusable Fault";
FaultVect CoprocessorUnusableFault::_vect = 0x180;
FaultStat CoprocessorUnusableFault::_count;

FaultName ReservedInstructionFault::_name = "Reserved Instruction Fault";
FaultVect ReservedInstructionFault::_vect = 0x0180;
FaultStat ReservedInstructionFault::_count;

FaultName ThreadFault::_name = "Thread Fault";
FaultVect ThreadFault::_vect = 0x00F1;
FaultStat ThreadFault::_count;

FaultName ArithmeticFault::_name = "Arithmetic Overflow Exception";
FaultVect ArithmeticFault::_vect = 0x180;
FaultStat ArithmeticFault::_count;

FaultName UnimplementedOpcodeFault::_name = "opdec";
FaultVect UnimplementedOpcodeFault::_vect = 0x0481;
FaultStat UnimplementedOpcodeFault::_count;

FaultName InterruptFault::_name = "interrupt";
FaultVect InterruptFault::_vect = 0x0180;
FaultStat InterruptFault::_count;

FaultName TrapFault::_name = "Trap";
FaultVect TrapFault::_vect = 0x0180;
FaultStat TrapFault::_count;

FaultName BreakpointFault::_name = "Breakpoint";
FaultVect BreakpointFault::_vect = 0x0180;
FaultStat BreakpointFault::_count;

FaultName ItbInvalidFault::_name = "Invalid TLB Entry Exception (I-Fetch/LW)";
FaultVect ItbInvalidFault::_vect = 0x0180;
FaultStat ItbInvalidFault::_count;

FaultName ItbPageFault::_name = "itbmiss";
FaultVect ItbPageFault::_vect = 0x0181;
FaultStat ItbPageFault::_count;

FaultName ItbMissFault::_name = "itbmiss";
FaultVect ItbMissFault::_vect = 0x0181;
FaultStat ItbMissFault::_count;

FaultName ItbAcvFault::_name = "iaccvio";
FaultVect ItbAcvFault::_vect = 0x0081;
FaultStat ItbAcvFault::_count;

FaultName ItbRefillFault::_name = "TLB Refill Exception (I-Fetch/LW)";
FaultVect ItbRefillFault::_vect = 0x0180;
FaultStat ItbRefillFault::_count;

FaultName NDtbMissFault::_name = "dtb_miss_single";
FaultVect NDtbMissFault::_vect = 0x0201;
FaultStat NDtbMissFault::_count;

FaultName PDtbMissFault::_name = "dtb_miss_double";
FaultVect PDtbMissFault::_vect = 0x0281;
FaultStat PDtbMissFault::_count;

FaultName DtbPageFault::_name = "dfault";
FaultVect DtbPageFault::_vect = 0x0381;
FaultStat DtbPageFault::_count;

FaultName DtbAcvFault::_name = "dfault";
FaultVect DtbAcvFault::_vect = 0x0381;
FaultStat DtbAcvFault::_count;

FaultName DtbInvalidFault::_name = "Invalid TLB Entry Exception (Store)";
FaultVect DtbInvalidFault::_vect = 0x0180;
FaultStat DtbInvalidFault::_count;

FaultName DtbRefillFault::_name = "TLB Refill Exception (Store)";
FaultVect DtbRefillFault::_vect = 0x0180;
FaultStat DtbRefillFault::_count;

FaultName TLBModifiedFault::_name = "TLB Modified Exception";
FaultVect TLBModifiedFault::_vect = 0x0180;
FaultStat TLBModifiedFault::_count;

FaultName FloatEnableFault::_name = "float_enable_fault";
FaultVect FloatEnableFault::_vect = 0x0581;
FaultStat FloatEnableFault::_count;

FaultName IntegerOverflowFault::_name = "Integer Overflow Fault";
FaultVect IntegerOverflowFault::_vect = 0x0501;
FaultStat IntegerOverflowFault::_count;

FaultName DspStateDisabledFault::_name = "DSP Disabled Fault";
FaultVect DspStateDisabledFault::_vect = 0x001a;
FaultStat DspStateDisabledFault::_count;

#if FULL_SYSTEM
void
MipsFault::setHandlerPC(Addr HandlerBase, ThreadContext *tc)
{
    tc->setPC(HandlerBase);
    tc->setNextPC(HandlerBase + sizeof(MachInst));
    tc->setNextNPC(HandlerBase + 2 * sizeof(MachInst));
}

void
MipsFault::setExceptionState(ThreadContext *tc, uint8_t excCode)
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
    // CHECK ME  or FIXME or FIX ME or POSSIBLE HACK
    // Check to see if the exception occurred in the branch delay slot
    DPRINTF(MipsPRA, "PC: %x, NextPC: %x, NNPC: %x\n",
            tc->readPC(), tc->readNextPC(), tc->readNextNPC());
    int bd = 0;
    if (tc->readPC() + sizeof(MachInst) != tc->readNextPC()) {
        tc->setMiscRegNoEffect(MISCREG_EPC, tc->readPC() - sizeof(MachInst));
        // In the branch delay slot? set CAUSE_31
        bd = 1;
    } else {
        tc->setMiscRegNoEffect(MISCREG_EPC, tc->readPC());
        // In the branch delay slot? reset CAUSE_31
        bd = 0;
    }

    // Set Cause_EXCCODE field
    CauseReg cause = tc->readMiscReg(MISCREG_CAUSE);
    cause.excCode = excCode;
    cause.bd = bd;
    cause.ce = 0;
    tc->setMiscRegNoEffect(MISCREG_CAUSE, cause);
}

void
ArithmeticFault::invoke(ThreadContext *tc, StaticInstPtr inst)
{
    DPRINTF(MipsPRA, "%s encountered.\n", name());
    setExceptionState(tc, 0xC);

    // Set new PC
    Addr HandlerBase;
    StatusReg status = tc->readMiscReg(MISCREG_STATUS);
    // Here, the handler is dependent on BEV, which is not modified by
    // setExceptionState()
    if (!status.bev) {
        // See MIPS ARM Vol 3, Revision 2, Page 38
        HandlerBase = vect() + tc->readMiscReg(MISCREG_EBASE);
    } else {
        HandlerBase = 0xBFC00200;
    }
    setHandlerPC(HandlerBase, tc);
}

void
StoreAddressErrorFault::invoke(ThreadContext *tc, StaticInstPtr inst)
{
    DPRINTF(MipsPRA, "%s encountered.\n", name());
    setExceptionState(tc, 0x5);
    tc->setMiscRegNoEffect(MISCREG_BADVADDR, badVAddr);

    // Set new PC
    Addr HandlerBase;
    // Offset 0x180 - General Exception Vector
    HandlerBase = vect() + tc->readMiscReg(MISCREG_EBASE);
    setHandlerPC(HandlerBase, tc);
}

void
TrapFault::invoke(ThreadContext *tc, StaticInstPtr inst)
{
    DPRINTF(MipsPRA, "%s encountered.\n", name());
    setExceptionState(tc, 0xD);

    // Set new PC
    Addr HandlerBase;
    // Offset 0x180 - General Exception Vector
    HandlerBase = vect() + tc->readMiscReg(MISCREG_EBASE);
    setHandlerPC(HandlerBase, tc);
}

void
BreakpointFault::invoke(ThreadContext *tc, StaticInstPtr inst)
{
    setExceptionState(tc, 0x9);

    // Set new PC
    Addr HandlerBase;
    // Offset 0x180 - General Exception Vector
    HandlerBase = vect() + tc->readMiscReg(MISCREG_EBASE);
    setHandlerPC(HandlerBase, tc);
}

void
DtbInvalidFault::invoke(ThreadContext *tc, StaticInstPtr inst)
{
    DPRINTF(MipsPRA, "%s encountered.\n", name());

    tc->setMiscRegNoEffect(MISCREG_BADVADDR, badVAddr);
    EntryHiReg entryHi = tc->readMiscReg(MISCREG_ENTRYHI);
    entryHi.asid = entryHiAsid;
    entryHi.vpn2 = entryHiVPN2;
    entryHi.vpn2x = entryHiVPN2X;
    tc->setMiscRegNoEffect(MISCREG_ENTRYHI, entryHi);

    ContextReg context = tc->readMiscReg(MISCREG_CONTEXT);
    context.badVPN2 = contextBadVPN2;
    tc->setMiscRegNoEffect(MISCREG_CONTEXT, context);
    setExceptionState(tc, 0x3);


    // Set new PC
    Addr HandlerBase;
    // Offset 0x180 - General Exception Vector
    HandlerBase = vect() + tc->readMiscReg(MISCREG_EBASE);
    setHandlerPC(HandlerBase, tc);
}

void
AddressErrorFault::invoke(ThreadContext *tc, StaticInstPtr inst)
{
    DPRINTF(MipsPRA, "%s encountered.\n", name());
    setExceptionState(tc, 0x4);
    tc->setMiscRegNoEffect(MISCREG_BADVADDR, badVAddr);

    // Set new PC
    Addr HandlerBase;
    // Offset 0x180 - General Exception Vector
    HandlerBase = vect() + tc->readMiscReg(MISCREG_EBASE);
    setHandlerPC(HandlerBase, tc);
}

void
ItbInvalidFault::invoke(ThreadContext *tc, StaticInstPtr inst)
{
    DPRINTF(MipsPRA, "%s encountered.\n", name());
    setExceptionState(tc, 0x2);
    tc->setMiscRegNoEffect(MISCREG_BADVADDR, badVAddr);
    EntryHiReg entryHi = tc->readMiscReg(MISCREG_ENTRYHI);
    entryHi.asid = entryHiAsid;
    entryHi.vpn2 = entryHiVPN2;
    entryHi.vpn2x = entryHiVPN2X;
    tc->setMiscRegNoEffect(MISCREG_ENTRYHI, entryHi);

    ContextReg context = tc->readMiscReg(MISCREG_CONTEXT);
    context.badVPN2 = contextBadVPN2;
    tc->setMiscRegNoEffect(MISCREG_CONTEXT, context);


    // Set new PC
    Addr HandlerBase;
    // Offset 0x180 - General Exception Vector
    HandlerBase = vect() + tc->readMiscReg(MISCREG_EBASE);
    setHandlerPC(HandlerBase,tc);
    DPRINTF(MipsPRA, "Exception Handler At: %x , EPC set to %x\n",
            HandlerBase, tc->readMiscReg(MISCREG_EPC));
}

void
ItbRefillFault::invoke(ThreadContext *tc, StaticInstPtr inst)
{
    DPRINTF(MipsPRA, "%s encountered (%x).\n", name(), MISCREG_BADVADDR);
    Addr HandlerBase;
    tc->setMiscRegNoEffect(MISCREG_BADVADDR, badVAddr);
    EntryHiReg entryHi = tc->readMiscReg(MISCREG_ENTRYHI);
    entryHi.asid = entryHiAsid;
    entryHi.vpn2 = entryHiVPN2;
    entryHi.vpn2x = entryHiVPN2X;
    tc->setMiscRegNoEffect(MISCREG_ENTRYHI, entryHi);
    ContextReg context = tc->readMiscReg(MISCREG_CONTEXT);
    context.badVPN2 = contextBadVPN2;
    tc->setMiscRegNoEffect(MISCREG_CONTEXT, context);

    StatusReg status = tc->readMiscReg(MISCREG_STATUS);
    // Since handler depends on EXL bit, must check EXL bit before setting it!!
    // See MIPS ARM Vol 3, Revision 2, Page 38
    if (status.exl == 1) {
        // Offset 0x180 - General Exception Vector
        HandlerBase = vect() + tc->readMiscReg(MISCREG_EBASE);
    } else {
        // Offset 0x000
        HandlerBase = tc->readMiscReg(MISCREG_EBASE);
    }

    setExceptionState(tc, 0x2);
    setHandlerPC(HandlerBase, tc);
}

void
DtbRefillFault::invoke(ThreadContext *tc, StaticInstPtr inst)
{
    // Set new PC
    DPRINTF(MipsPRA, "%s encountered.\n", name());
    Addr HandlerBase;
    tc->setMiscRegNoEffect(MISCREG_BADVADDR, badVAddr);
    EntryHiReg entryHi = tc->readMiscReg(MISCREG_ENTRYHI);
    entryHi.asid = entryHiAsid;
    entryHi.vpn2 = entryHiVPN2;
    entryHi.vpn2x = entryHiVPN2X;
    tc->setMiscRegNoEffect(MISCREG_ENTRYHI, entryHi);

    ContextReg context = tc->readMiscReg(MISCREG_CONTEXT);
    context.badVPN2 = contextBadVPN2;
    tc->setMiscRegNoEffect(MISCREG_CONTEXT, context);

    StatusReg status = tc->readMiscReg(MISCREG_STATUS);
    // Since handler depends on EXL bit, must check EXL bit before setting it!!
    // See MIPS ARM Vol 3, Revision 2, Page 38
    if (status.exl) {
        // Offset 0x180 - General Exception Vector
        HandlerBase = vect() + tc->readMiscReg(MISCREG_EBASE);
    } else {
        // Offset 0x000
        HandlerBase = tc->readMiscReg(MISCREG_EBASE);
    }

    setExceptionState(tc, 0x3);

    setHandlerPC(HandlerBase, tc);
}

void
TLBModifiedFault::invoke(ThreadContext *tc, StaticInstPtr inst)
{
    DPRINTF(MipsPRA, "%s encountered.\n", name());
    tc->setMiscRegNoEffect(MISCREG_BADVADDR, badVAddr);
    EntryHiReg entryHi = tc->readMiscReg(MISCREG_ENTRYHI);
    entryHi.asid = entryHiAsid;
    entryHi.vpn2 = entryHiVPN2;
    entryHi.vpn2x = entryHiVPN2X;
    tc->setMiscRegNoEffect(MISCREG_ENTRYHI, entryHi);

    ContextReg context = tc->readMiscReg(MISCREG_CONTEXT);
    context.badVPN2 = contextBadVPN2;
    tc->setMiscRegNoEffect(MISCREG_CONTEXT, context);

    // Set new PC
    Addr HandlerBase;
    // Offset 0x180 - General Exception Vector
    HandlerBase = vect() + tc->readMiscReg(MISCREG_EBASE);
    setExceptionState(tc, 0x1);
    setHandlerPC(HandlerBase, tc);

}

void
SystemCallFault::invoke(ThreadContext *tc, StaticInstPtr inst)
{
    DPRINTF(MipsPRA, "%s encountered.\n", name());
    setExceptionState(tc, 0x8);

    // Set new PC
    Addr HandlerBase;
    // Offset 0x180 - General Exception Vector
    HandlerBase = vect() + tc->readMiscReg(MISCREG_EBASE);
    setHandlerPC(HandlerBase, tc);
}

void
InterruptFault::invoke(ThreadContext *tc, StaticInstPtr inst)
{
#if  FULL_SYSTEM
    DPRINTF(MipsPRA, "%s encountered.\n", name());
    setExceptionState(tc, 0x0A);
    Addr HandlerBase;

    CauseReg cause = tc->readMiscRegNoEffect(MISCREG_CAUSE);
    if (cause.iv) {
        // Offset 200 for release 2
        HandlerBase = 0x20 + vect() + tc->readMiscRegNoEffect(MISCREG_EBASE);
    } else {
        //Ofset at 180 for release 1
        HandlerBase = vect() + tc->readMiscRegNoEffect(MISCREG_EBASE);
    }

    setHandlerPC(HandlerBase, tc);
#endif
}

#endif // FULL_SYSTEM

void
ResetFault::invoke(ThreadContext *tc, StaticInstPtr inst)
{
#if FULL_SYSTEM
    DPRINTF(MipsPRA, "%s encountered.\n", name());
    /* All reset activity must be invoked from here */
    tc->setPC(vect());
    tc->setNextPC(vect() + sizeof(MachInst));
    tc->setNextNPC(vect() + sizeof(MachInst) + sizeof(MachInst));
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
    Addr HandlerBase;
    // Offset 0x180 - General Exception Vector
    HandlerBase = vect() + tc->readMiscRegNoEffect(MISCREG_EBASE);
    setHandlerPC(HandlerBase, tc);
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

    Addr HandlerBase;
    // Offset 0x180 - General Exception Vector
    HandlerBase = vect() + tc->readMiscReg(MISCREG_EBASE);
    setHandlerPC(HandlerBase, tc);

#else
    warn("%s (CP%d) encountered.\n", name(), coProcID);
#endif
}

} // namespace MipsISA

