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

template<> SparcFaultBase::FaultVals
    SparcFault<PowerOnReset>::vals =
    {"power_on_reset", 0x001, 0, {H, H, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<WatchDogReset>::vals =
    {"watch_dog_reset", 0x002, 120, {H, H, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<ExternallyInitiatedReset>::vals =
    {"externally_initiated_reset", 0x003, 110, {H, H, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<SoftwareInitiatedReset>::vals =
    {"software_initiated_reset", 0x004, 130, {SH, SH, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<REDStateException>::vals =
    {"RED_state_exception", 0x005, 1, {H, H, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<StoreError>::vals =
    {"store_error", 0x007, 201, {H, H, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<InstructionAccessException>::vals =
    {"instruction_access_exception", 0x008, 300, {H, H, H}};

//XXX This trap is apparently dropped from ua2005
/*template<> SparcFaultBase::FaultVals
    SparcFault<InstructionAccessMMUMiss>::vals =
    {"inst_mmu", 0x009, 2, {H, H, H}};*/

template<> SparcFaultBase::FaultVals
    SparcFault<InstructionAccessError>::vals =
    {"instruction_access_error", 0x00A, 400, {H, H, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<IllegalInstruction>::vals =
    {"illegal_instruction", 0x010, 620, {H, H, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<PrivilegedOpcode>::vals =
    {"privileged_opcode", 0x011, 700, {P, SH, SH}};

//XXX This trap is apparently dropped from ua2005
/*template<> SparcFaultBase::FaultVals
    SparcFault<UnimplementedLDD>::vals =
    {"unimp_ldd", 0x012, 6, {H, H, H}};*/

//XXX This trap is apparently dropped from ua2005
/*template<> SparcFaultBase::FaultVals
    SparcFault<UnimplementedSTD>::vals =
    {"unimp_std", 0x013, 6, {H, H, H}};*/

template<> SparcFaultBase::FaultVals
    SparcFault<FpDisabled>::vals =
    {"fp_disabled", 0x020, 800, {P, P, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<FpExceptionIEEE754>::vals =
    {"fp_exception_ieee_754", 0x021, 1110, {P, P, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<FpExceptionOther>::vals =
    {"fp_exception_other", 0x022, 1110, {P, P, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<TagOverflow>::vals =
    {"tag_overflow", 0x023, 1400, {P, P, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<CleanWindow>::vals =
    {"clean_window", 0x024, 1010, {P, P, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<DivisionByZero>::vals =
    {"division_by_zero", 0x028, 1500, {P, P, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<InternalProcessorError>::vals =
    {"internal_processor_error", 0x029, 4, {H, H, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<InstructionInvalidTSBEntry>::vals =
    {"instruction_invalid_tsb_entry", 0x02A, 210, {H, H, SH}};

template<> SparcFaultBase::FaultVals
    SparcFault<DataInvalidTSBEntry>::vals =
    {"data_invalid_tsb_entry", 0x02B, 1203, {H, H, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<DataAccessException>::vals =
    {"data_access_exception", 0x030, 1201, {H, H, H}};

//XXX This trap is apparently dropped from ua2005
/*template<> SparcFaultBase::FaultVals
    SparcFault<DataAccessMMUMiss>::vals =
    {"data_mmu", 0x031, 12, {H, H, H}};*/

template<> SparcFaultBase::FaultVals
    SparcFault<DataAccessError>::vals =
    {"data_access_error", 0x032, 1210, {H, H, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<DataAccessProtection>::vals =
    {"data_access_protection", 0x033, 1207, {H, H, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<MemAddressNotAligned>::vals =
    {"mem_address_not_aligned", 0x034, 1020, {H, H, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<LDDFMemAddressNotAligned>::vals =
    {"LDDF_mem_address_not_aligned", 0x035, 1010, {H, H, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<STDFMemAddressNotAligned>::vals =
    {"STDF_mem_address_not_aligned", 0x036, 1010, {H, H, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<PrivilegedAction>::vals =
    {"privileged_action", 0x037, 1110, {H, H, SH}};

template<> SparcFaultBase::FaultVals
    SparcFault<LDQFMemAddressNotAligned>::vals =
    {"LDQF_mem_address_not_aligned", 0x038, 1010, {H, H, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<STQFMemAddressNotAligned>::vals =
    {"STQF_mem_address_not_aligned", 0x039, 1010, {H, H, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<InstructionRealTranslationMiss>::vals =
    {"instruction_real_translation_miss", 0x03E, 208, {H, H, SH}};

template<> SparcFaultBase::FaultVals
    SparcFault<DataRealTranslationMiss>::vals =
    {"data_real_translation_miss", 0x03F, 1203, {H, H, H}};

//XXX This trap is apparently dropped from ua2005
/*template<> SparcFaultBase::FaultVals
    SparcFault<AsyncDataError>::vals =
    {"async_data", 0x040, 2, {H, H, H}};*/

template<> SparcFaultBase::FaultVals
    SparcFault<InterruptLevelN>::vals =
    {"interrupt_level_n", 0x041, 0, {P, P, SH}};

template<> SparcFaultBase::FaultVals
    SparcFault<HstickMatch>::vals =
    {"hstick_match", 0x05E, 1601, {H, H, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<TrapLevelZero>::vals =
    {"trap_level_zero", 0x05F, 202, {H, H, SH}};

template<> SparcFaultBase::FaultVals
    SparcFault<PAWatchpoint>::vals =
    {"PA_watchpoint", 0x061, 1209, {H, H, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<VAWatchpoint>::vals =
    {"VA_watchpoint", 0x062, 1120, {P, P, SH}};

template<> SparcFaultBase::FaultVals
    SparcFault<FastInstructionAccessMMUMiss>::vals =
    {"fast_instruction_access_MMU_miss", 0x064, 208, {H, H, SH}};

template<> SparcFaultBase::FaultVals
    SparcFault<FastDataAccessMMUMiss>::vals =
    {"fast_data_access_MMU_miss", 0x068, 1203, {H, H, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<FastDataAccessProtection>::vals =
    {"fast_data_access_protection", 0x06C, 1207, {H, H, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<InstructionBreakpoint>::vals =
    {"instruction_break", 0x076, 610, {H, H, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<CpuMondo>::vals =
    {"cpu_mondo", 0x07C, 1608, {P, P, SH}};

template<> SparcFaultBase::FaultVals
    SparcFault<DevMondo>::vals =
    {"dev_mondo", 0x07D, 1611, {P, P, SH}};

template<> SparcFaultBase::FaultVals
    SparcFault<ResumeableError>::vals =
    {"resume_error", 0x07E, 3330, {P, P, SH}};

template<> SparcFaultBase::FaultVals
    SparcFault<SpillNNormal>::vals =
    {"spill_n_normal", 0x080, 900, {P, P, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<SpillNOther>::vals =
    {"spill_n_other", 0x0A0, 900, {P, P, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<FillNNormal>::vals =
    {"fill_n_normal", 0x0C0, 900, {P, P, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<FillNOther>::vals =
    {"fill_n_other", 0x0E0, 900, {P, P, H}};

template<> SparcFaultBase::FaultVals
    SparcFault<TrapInstruction>::vals =
    {"trap_instruction", 0x100, 1602, {P, P, H}};

#if !FULL_SYSTEM
template<> SparcFaultBase::FaultVals
    SparcFault<PageTableFault>::vals =
    {"page_table_fault", 0x0000, 0, {SH, SH, SH}};
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

void SparcFaultBase::invoke(ThreadContext * tc)
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

