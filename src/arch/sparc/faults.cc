/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 *          Kevin Lim
 */

#include <algorithm>

#include "arch/sparc/faults.hh"
#include "arch/sparc/isa_traits.hh"
#include "base/bitfield.hh"
#include "base/trace.hh"
#include "config/full_system.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#if !FULL_SYSTEM
#include "arch/sparc/process.hh"
#include "mem/page_table.hh"
#include "sim/process.hh"
#endif

using namespace std;

namespace SparcISA
{

FaultName     InternalProcessorError::_name = "intprocerr";
TrapType      InternalProcessorError::_trapType = 0x029;
FaultPriority InternalProcessorError::_priority = 4;
FaultStat     InternalProcessorError::_count;

FaultName     MemAddressNotAligned::_name = "unalign";
TrapType      MemAddressNotAligned::_trapType = 0x034;
FaultPriority MemAddressNotAligned::_priority = 10;
FaultStat     MemAddressNotAligned::_count;

FaultName     PowerOnReset::_name = "pow_reset";
TrapType      PowerOnReset::_trapType = 0x001;
FaultPriority PowerOnReset::_priority = 0;
FaultStat     PowerOnReset::_count;

FaultName     WatchDogReset::_name = "watch_dog_reset";
TrapType      WatchDogReset::_trapType = 0x002;
FaultPriority WatchDogReset::_priority = 1;
FaultStat     WatchDogReset::_count;

FaultName     ExternallyInitiatedReset::_name = "extern_reset";
TrapType      ExternallyInitiatedReset::_trapType = 0x003;
FaultPriority ExternallyInitiatedReset::_priority = 1;
FaultStat     ExternallyInitiatedReset::_count;

FaultName     SoftwareInitiatedReset::_name = "software_reset";
TrapType      SoftwareInitiatedReset::_trapType = 0x004;
FaultPriority SoftwareInitiatedReset::_priority = 1;
FaultStat     SoftwareInitiatedReset::_count;

FaultName     REDStateException::_name = "red_counte";
TrapType      REDStateException::_trapType = 0x005;
FaultPriority REDStateException::_priority = 1;
FaultStat     REDStateException::_count;

FaultName     InstructionAccessException::_name = "inst_access";
TrapType      InstructionAccessException::_trapType = 0x008;
FaultPriority InstructionAccessException::_priority = 5;
FaultStat     InstructionAccessException::_count;

FaultName     InstructionAccessMMUMiss::_name = "inst_mmu";
TrapType      InstructionAccessMMUMiss::_trapType = 0x009;
FaultPriority InstructionAccessMMUMiss::_priority = 2;
FaultStat     InstructionAccessMMUMiss::_count;

FaultName     InstructionAccessError::_name = "inst_error";
TrapType      InstructionAccessError::_trapType = 0x00A;
FaultPriority InstructionAccessError::_priority = 3;
FaultStat     InstructionAccessError::_count;

FaultName     IllegalInstruction::_name = "illegal_inst";
TrapType      IllegalInstruction::_trapType = 0x010;
FaultPriority IllegalInstruction::_priority = 7;
FaultStat     IllegalInstruction::_count;

FaultName     PrivilegedOpcode::_name = "priv_opcode";
TrapType      PrivilegedOpcode::_trapType = 0x011;
FaultPriority PrivilegedOpcode::_priority = 6;
FaultStat     PrivilegedOpcode::_count;

FaultName     UnimplementedLDD::_name = "unimp_ldd";
TrapType      UnimplementedLDD::_trapType = 0x012;
FaultPriority UnimplementedLDD::_priority = 6;
FaultStat     UnimplementedLDD::_count;

FaultName     UnimplementedSTD::_name = "unimp_std";
TrapType      UnimplementedSTD::_trapType = 0x013;
FaultPriority UnimplementedSTD::_priority = 6;
FaultStat     UnimplementedSTD::_count;

FaultName     FpDisabled::_name = "fp_disabled";
TrapType      FpDisabled::_trapType = 0x020;
FaultPriority FpDisabled::_priority = 8;
FaultStat     FpDisabled::_count;

FaultName     FpExceptionIEEE754::_name = "fp_754";
TrapType      FpExceptionIEEE754::_trapType = 0x021;
FaultPriority FpExceptionIEEE754::_priority = 11;
FaultStat     FpExceptionIEEE754::_count;

FaultName     FpExceptionOther::_name = "fp_other";
TrapType      FpExceptionOther::_trapType = 0x022;
FaultPriority FpExceptionOther::_priority = 11;
FaultStat     FpExceptionOther::_count;

FaultName     TagOverflow::_name = "tag_overflow";
TrapType      TagOverflow::_trapType = 0x023;
FaultPriority TagOverflow::_priority = 14;
FaultStat     TagOverflow::_count;

FaultName     DivisionByZero::_name = "div_by_zero";
TrapType      DivisionByZero::_trapType = 0x028;
FaultPriority DivisionByZero::_priority = 15;
FaultStat     DivisionByZero::_count;

FaultName     DataAccessException::_name = "data_access";
TrapType      DataAccessException::_trapType = 0x030;
FaultPriority DataAccessException::_priority = 12;
FaultStat     DataAccessException::_count;

FaultName     DataAccessMMUMiss::_name = "data_mmu";
TrapType      DataAccessMMUMiss::_trapType = 0x031;
FaultPriority DataAccessMMUMiss::_priority = 12;
FaultStat     DataAccessMMUMiss::_count;

FaultName     DataAccessError::_name = "data_error";
TrapType      DataAccessError::_trapType = 0x032;
FaultPriority DataAccessError::_priority = 12;
FaultStat     DataAccessError::_count;

FaultName     DataAccessProtection::_name = "data_protection";
TrapType      DataAccessProtection::_trapType = 0x033;
FaultPriority DataAccessProtection::_priority = 12;
FaultStat     DataAccessProtection::_count;

FaultName     LDDFMemAddressNotAligned::_name = "unalign_lddf";
TrapType      LDDFMemAddressNotAligned::_trapType = 0x035;
FaultPriority LDDFMemAddressNotAligned::_priority = 10;
FaultStat     LDDFMemAddressNotAligned::_count;

FaultName     STDFMemAddressNotAligned::_name = "unalign_stdf";
TrapType      STDFMemAddressNotAligned::_trapType = 0x036;
FaultPriority STDFMemAddressNotAligned::_priority = 10;
FaultStat     STDFMemAddressNotAligned::_count;

FaultName     PrivilegedAction::_name = "priv_action";
TrapType      PrivilegedAction::_trapType = 0x037;
FaultPriority PrivilegedAction::_priority = 11;
FaultStat     PrivilegedAction::_count;

FaultName     LDQFMemAddressNotAligned::_name = "unalign_ldqf";
TrapType      LDQFMemAddressNotAligned::_trapType = 0x038;
FaultPriority LDQFMemAddressNotAligned::_priority = 10;
FaultStat     LDQFMemAddressNotAligned::_count;

FaultName     STQFMemAddressNotAligned::_name = "unalign_stqf";
TrapType      STQFMemAddressNotAligned::_trapType = 0x039;
FaultPriority STQFMemAddressNotAligned::_priority = 10;
FaultStat     STQFMemAddressNotAligned::_count;

FaultName     AsyncDataError::_name = "async_data";
TrapType      AsyncDataError::_trapType = 0x040;
FaultPriority AsyncDataError::_priority = 2;
FaultStat     AsyncDataError::_count;

FaultName     CleanWindow::_name = "clean_win";
TrapType      CleanWindow::_trapType = 0x024;
FaultPriority CleanWindow::_priority = 10;
FaultStat     CleanWindow::_count;

//The enumerated faults

FaultName     InterruptLevelN::_name = "interrupt_n";
TrapType      InterruptLevelN::_baseTrapType = 0x041;
FaultStat     InterruptLevelN::_count;

FaultName     SpillNNormal::_name = "spill_n_normal";
TrapType      SpillNNormal::_baseTrapType = 0x080;
FaultPriority SpillNNormal::_priority = 9;
FaultStat     SpillNNormal::_count;

FaultName     SpillNOther::_name = "spill_n_other";
TrapType      SpillNOther::_baseTrapType = 0x0A0;
FaultPriority SpillNOther::_priority = 9;
FaultStat     SpillNOther::_count;

FaultName     FillNNormal::_name = "fill_n_normal";
TrapType      FillNNormal::_baseTrapType = 0x0C0;
FaultPriority FillNNormal::_priority = 9;
FaultStat     FillNNormal::_count;

FaultName     FillNOther::_name = "fill_n_other";
TrapType      FillNOther::_baseTrapType = 0x0E0;
FaultPriority FillNOther::_priority = 9;
FaultStat     FillNOther::_count;

FaultName     TrapInstruction::_name = "trap_inst_n";
TrapType      TrapInstruction::_baseTrapType = 0x100;
FaultPriority TrapInstruction::_priority = 16;
FaultStat     TrapInstruction::_count;

#if !FULL_SYSTEM
FaultName PageTableFault::_name = "page_table_fault";
TrapType PageTableFault::_trapType = 0x0000;
FaultPriority PageTableFault::_priority = 0;
FaultStat PageTableFault::_count;
#endif

/**
 * This sets everything up for a normal trap except for actually jumping to
 * the handler. It will need to be expanded to include the state machine in
 * the manual. Right now it assumes that traps will always be to the
 * privileged level.
 */

void doNormalFault(ThreadContext *tc, TrapType tt)
{
    uint64_t TL = tc->readMiscReg(MISCREG_TL);
    uint64_t TSTATE = tc->readMiscReg(MISCREG_TSTATE);
    uint64_t PSTATE = tc->readMiscReg(MISCREG_PSTATE);
    uint64_t HPSTATE = tc->readMiscReg(MISCREG_HPSTATE);
    uint64_t CCR = tc->readMiscReg(MISCREG_CCR);
    uint64_t ASI = tc->readMiscReg(MISCREG_ASI);
    uint64_t CWP = tc->readMiscReg(MISCREG_CWP);
    uint64_t CANSAVE = tc->readMiscReg(MISCREG_CANSAVE);
    uint64_t GL = tc->readMiscReg(MISCREG_GL);
    uint64_t PC = tc->readPC();
    uint64_t NPC = tc->readNextPC();

    //Increment the trap level
    TL++;
    tc->setMiscReg(MISCREG_TL, TL);

    //Save off state

    //set TSTATE.gl to gl
    replaceBits(TSTATE, 42, 40, GL);
    //set TSTATE.ccr to ccr
    replaceBits(TSTATE, 39, 32, CCR);
    //set TSTATE.asi to asi
    replaceBits(TSTATE, 31, 24, ASI);
    //set TSTATE.pstate to pstate
    replaceBits(TSTATE, 20, 8, PSTATE);
    //set TSTATE.cwp to cwp
    replaceBits(TSTATE, 4, 0, CWP);

    //Write back TSTATE
    tc->setMiscReg(MISCREG_TSTATE, TSTATE);

    //set TPC to PC
    tc->setMiscReg(MISCREG_TPC, PC);
    //set TNPC to NPC
    tc->setMiscReg(MISCREG_TNPC, NPC);

    //set HTSTATE.hpstate to hpstate
    tc->setMiscReg(MISCREG_HTSTATE, HPSTATE);

    //TT = trap type;
    tc->setMiscReg(MISCREG_TT, tt);

    //Update the global register level
    if(1/*We're delivering the trap in priveleged mode*/)
        tc->setMiscReg(MISCREG_GL, min<int>(GL+1, MaxGL));
    else
        tc->setMiscReg(MISCREG_GL, min<int>(GL+1, MaxPGL));

    //PSTATE.mm is unchanged
    //PSTATE.pef = whether or not an fpu is present
    //XXX We'll say there's one present, even though there aren't
    //implementations for a decent number of the instructions
    PSTATE |= (1 << 4);
    //PSTATE.am = 0
    PSTATE &= ~(1 << 3);
    if(1/*We're delivering the trap in priveleged mode*/)
    {
        //PSTATE.priv = 1
        PSTATE |= (1 << 2);
        //PSTATE.cle = PSTATE.tle
        replaceBits(PSTATE, 9, 9, PSTATE >> 8);
    }
    else
    {
        //PSTATE.priv = 0
        PSTATE &= ~(1 << 2);
        //PSTATE.cle = 0
        PSTATE &= ~(1 << 9);
    }
    //PSTATE.ie = 0
    PSTATE &= ~(1 << 1);
    //PSTATE.tle is unchanged
    //PSTATE.tct = 0
    //XXX Where exactly is this field?
    tc->setMiscReg(MISCREG_PSTATE, PSTATE);

    if(0/*We're delivering the trap in hyperprivileged mode*/)
    {
        //HPSTATE.red = 0
        HPSTATE &= ~(1 << 5);
        //HPSTATE.hpriv = 1
        HPSTATE |= (1 << 2);
        //HPSTATE.ibe = 0
        HPSTATE &= ~(1 << 10);
        //HPSTATE.tlz is unchanged
        tc->setMiscReg(MISCREG_HPSTATE, HPSTATE);
    }

    bool changedCWP = true;
    if(tt == 0x24)
        CWP++;
    else if(0x80 <= tt && tt <= 0xbf)
        CWP += (CANSAVE + 2);
    else if(0xc0 <= tt && tt <= 0xff)
        CWP--;
    else
        changedCWP = false;

    if(changedCWP)
    {
        CWP = (CWP + NWindows) % NWindows;
        tc->setMiscRegWithEffect(MISCREG_CWP, CWP);
    }
}

#if FULL_SYSTEM

void SparcFault::invoke(ThreadContext * tc)
{
    FaultBase::invoke(tc);
    countStat()++;

    //Use the SPARC trap state machine
}

void PowerOnReset::invoke(ThreadContext * tc)
{
    //For SPARC, when a system is first started, there is a power
    //on reset Trap which sets the processor into the following state.
    //Bits that aren't set aren't defined on startup.
    /*
    tl = MaxTL;
    gl = MaxGL;

    tickFields.counter = 0; //The TICK register is unreadable bya
    tickFields.npt = 1; //The TICK register is unreadable by by !priv

    softint = 0; // Clear all the soft interrupt bits
    tick_cmprFields.int_dis = 1; // disable timer compare interrupts
    tick_cmprFields.tick_cmpr = 0; // Reset to 0 for pretty printing
    stickFields.npt = 1; //The TICK register is unreadable by by !priv
    stick_cmprFields.int_dis = 1; // disable timer compare interrupts
    stick_cmprFields.tick_cmpr = 0; // Reset to 0 for pretty printing

    tt[tl] = _trapType;
    pstate = 0; // fields 0 but pef
    pstateFields.pef = 1;

    hpstate = 0;
    hpstateFields.red = 1;
    hpstateFields.hpriv = 1;
    hpstateFields.tlz = 0; // this is a guess
    hintp = 0; // no interrupts pending
    hstick_cmprFields.int_dis = 1; // disable timer compare interrupts
    hstick_cmprFields.tick_cmpr = 0; // Reset to 0 for pretty printing
    */
}

#endif

#if !FULL_SYSTEM

void SpillNNormal::invoke(ThreadContext *tc)
{
    doNormalFault(tc, trapType());

    Process *p = tc->getProcessPtr();

    //This will only work in faults from a SparcLiveProcess
    SparcLiveProcess *lp = dynamic_cast<SparcLiveProcess *>(p);
    assert(lp);

    //Then adjust the PC and NPC
    Addr spillStart = lp->readSpillStart();
    tc->setPC(spillStart);
    tc->setNextPC(spillStart + sizeof(MachInst));
    tc->setNextNPC(spillStart + 2*sizeof(MachInst));
}

void FillNNormal::invoke(ThreadContext *tc)
{
    doNormalFault(tc, trapType());

    Process * p = tc->getProcessPtr();

    //This will only work in faults from a SparcLiveProcess
    SparcLiveProcess *lp = dynamic_cast<SparcLiveProcess *>(p);
    assert(lp);

    //The adjust the PC and NPC
    Addr fillStart = lp->readFillStart();
    tc->setPC(fillStart);
    tc->setNextPC(fillStart + sizeof(MachInst));
    tc->setNextNPC(fillStart + 2*sizeof(MachInst));
}

void PageTableFault::invoke(ThreadContext *tc)
{
    Process *p = tc->getProcessPtr();

    // address is higher than the stack region or in the current stack region
    if (vaddr > p->stack_base || vaddr > p->stack_min)
        FaultBase::invoke(tc);

    // We've accessed the next page
    if (vaddr > p->stack_min - PageBytes) {
        p->stack_min -= PageBytes;
        if (p->stack_base - p->stack_min > 8*1024*1024)
            fatal("Over max stack size for one thread\n");
        p->pTable->allocate(p->stack_min, PageBytes);
        warn("Increasing stack size by one page.");
    } else {
        FaultBase::invoke(tc);
    }
}

#endif

} // namespace SparcISA

