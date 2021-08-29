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
 */

#include "arch/sparc/faults.hh"

#include <algorithm>

#include "arch/sparc/mmu.hh"
#include "arch/sparc/process.hh"
#include "arch/sparc/se_workload.hh"
#include "arch/sparc/sparc_traits.hh"
#include "arch/sparc/types.hh"
#include "base/bitfield.hh"
#include "base/compiler.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "mem/page_table.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"

namespace gem5
{

namespace SparcISA
{

template<> SparcFaultBase::FaultVals
    SparcFault<PowerOnReset>::vals
("power_on_reset", 0x001, 0, {{H, H, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<WatchDogReset>::vals
("watch_dog_reset", 0x002, 120, {{H, H, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<ExternallyInitiatedReset>::vals
("externally_initiated_reset", 0x003, 110, {{H, H, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<SoftwareInitiatedReset>::vals
("software_initiated_reset", 0x004, 130, {{SH, SH, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<REDStateException>::vals
("RED_state_exception", 0x005, 1, {{H, H, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<StoreError>::vals
("store_error", 0x007, 201, {{H, H, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<InstructionAccessException>::vals
("instruction_access_exception", 0x008, 300, {{H, H, H}});

//XXX This trap is apparently dropped from ua2005
/*template<> SparcFaultBase::FaultVals
    SparcFault<InstructionAccessMMUMiss>::vals
    ("inst_mmu", 0x009, 2, {{H, H, H}});*/

template<> SparcFaultBase::FaultVals
    SparcFault<InstructionAccessError>::vals
("instruction_access_error", 0x00A, 400, {{H, H, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<IllegalInstruction>::vals
("illegal_instruction", 0x010, 620, {{H, H, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<PrivilegedOpcode>::vals
("privileged_opcode", 0x011, 700, {{P, SH, SH}});

//XXX This trap is apparently dropped from ua2005
/*template<> SparcFaultBase::FaultVals
    SparcFault<UnimplementedLDD>::vals
    ("unimp_ldd", 0x012, 6, {{H, H, H}});*/

//XXX This trap is apparently dropped from ua2005
/*template<> SparcFaultBase::FaultVals
    SparcFault<UnimplementedSTD>::vals
    ("unimp_std", 0x013, 6, {{H, H, H}});*/

template<> SparcFaultBase::FaultVals
    SparcFault<FpDisabled>::vals
("fp_disabled", 0x020, 800, {{P, P, H}});

/* SPARCv8 and SPARCv9 define just fp_disabled trap. SIMD is not contemplated
 * as a separate part. Therefore, we use the same code and TT */
template<> SparcFaultBase::FaultVals
    SparcFault<VecDisabled>::vals
("fp_disabled", 0x020, 800, {{P, P, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<FpExceptionIEEE754>::vals
("fp_exception_ieee_754", 0x021, 1110, {{P, P, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<FpExceptionOther>::vals
("fp_exception_other", 0x022, 1110, {{P, P, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<TagOverflow>::vals
("tag_overflow", 0x023, 1400, {{P, P, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<CleanWindow>::vals
("clean_window", 0x024, 1010, {{P, P, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<DivisionByZero>::vals
("division_by_zero", 0x028, 1500, {{P, P, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<InternalProcessorError>::vals
("internal_processor_error", 0x029, 4, {{H, H, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<InstructionInvalidTSBEntry>::vals
("instruction_invalid_tsb_entry", 0x02A, 210, {{H, H, SH}});

template<> SparcFaultBase::FaultVals
    SparcFault<DataInvalidTSBEntry>::vals
("data_invalid_tsb_entry", 0x02B, 1203, {{H, H, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<DataAccessException>::vals
("data_access_exception", 0x030, 1201, {{H, H, H}});

//XXX This trap is apparently dropped from ua2005
/*template<> SparcFaultBase::FaultVals
    SparcFault<DataAccessMMUMiss>::vals
    ("data_mmu", 0x031, 12, {{H, H, H}});*/

template<> SparcFaultBase::FaultVals
    SparcFault<DataAccessError>::vals
("data_access_error", 0x032, 1210, {{H, H, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<DataAccessProtection>::vals
("data_access_protection", 0x033, 1207, {{H, H, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<MemAddressNotAligned>::vals
("mem_address_not_aligned", 0x034, 1020, {{H, H, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<LDDFMemAddressNotAligned>::vals
("LDDF_mem_address_not_aligned", 0x035, 1010, {{H, H, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<STDFMemAddressNotAligned>::vals
("STDF_mem_address_not_aligned", 0x036, 1010, {{H, H, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<PrivilegedAction>::vals
("privileged_action", 0x037, 1110, {{H, H, SH}});

template<> SparcFaultBase::FaultVals
    SparcFault<LDQFMemAddressNotAligned>::vals
("LDQF_mem_address_not_aligned", 0x038, 1010, {{H, H, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<STQFMemAddressNotAligned>::vals
("STQF_mem_address_not_aligned", 0x039, 1010, {{H, H, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<InstructionRealTranslationMiss>::vals
("instruction_real_translation_miss", 0x03E, 208, {{H, H, SH}});

template<> SparcFaultBase::FaultVals
    SparcFault<DataRealTranslationMiss>::vals
("data_real_translation_miss", 0x03F, 1203, {{H, H, H}});

//XXX This trap is apparently dropped from ua2005
/*template<> SparcFaultBase::FaultVals
    SparcFault<AsyncDataError>::vals
    ("async_data", 0x040, 2, {{H, H, H}});*/

template<> SparcFaultBase::FaultVals
    SparcFault<InterruptLevelN>::vals
("interrupt_level_n", 0x040, 0, {{P, P, SH}});

template<> SparcFaultBase::FaultVals
    SparcFault<HstickMatch>::vals
("hstick_match", 0x05E, 1601, {{H, H, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<TrapLevelZero>::vals
("trap_level_zero", 0x05F, 202, {{H, H, SH}});

template<> SparcFaultBase::FaultVals
    SparcFault<InterruptVector>::vals
("interrupt_vector", 0x060, 2630, {{H, H, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<PAWatchpoint>::vals
("PA_watchpoint", 0x061, 1209, {{H, H, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<VAWatchpoint>::vals
("VA_watchpoint", 0x062, 1120, {{P, P, SH}});

template<> SparcFaultBase::FaultVals
    SparcFault<FastInstructionAccessMMUMiss>::vals
("fast_instruction_access_MMU_miss", 0x064, 208, {{H, H, SH}});

template<> SparcFaultBase::FaultVals
    SparcFault<FastDataAccessMMUMiss>::vals
("fast_data_access_MMU_miss", 0x068, 1203, {{H, H, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<FastDataAccessProtection>::vals
("fast_data_access_protection", 0x06C, 1207, {{H, H, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<InstructionBreakpoint>::vals
("instruction_break", 0x076, 610, {{H, H, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<CpuMondo>::vals
("cpu_mondo", 0x07C, 1608, {{P, P, SH}});

template<> SparcFaultBase::FaultVals
    SparcFault<DevMondo>::vals
("dev_mondo", 0x07D, 1611, {{P, P, SH}});

template<> SparcFaultBase::FaultVals
    SparcFault<ResumableError>::vals
("resume_error", 0x07E, 3330, {{P, P, SH}});

template<> SparcFaultBase::FaultVals
    SparcFault<SpillNNormal>::vals
("spill_n_normal", 0x080, 900, {{P, P, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<SpillNOther>::vals
("spill_n_other", 0x0A0, 900, {{P, P, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<FillNNormal>::vals
("fill_n_normal", 0x0C0, 900, {{P, P, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<FillNOther>::vals
("fill_n_other", 0x0E0, 900, {{P, P, H}});

template<> SparcFaultBase::FaultVals
    SparcFault<TrapInstruction>::vals
("trap_instruction", 0x100, 1602, {{P, P, H}});

/**
 * This causes the thread context to enter RED state. This causes the side
 * effects which go with entering RED state because of a trap.
 */

void
enterREDState(ThreadContext *tc)
{
    //@todo Disable the mmu?
    //@todo Disable watchpoints?
    HPSTATE hpstate= tc->readMiscRegNoEffect(MISCREG_HPSTATE);
    hpstate.red = 1;
    hpstate.hpriv = 1;
    tc->setMiscReg(MISCREG_HPSTATE, hpstate);
    // PSTATE.priv is set to 1 here. The manual says it should be 0, but
    // Legion sets it to 1.
    PSTATE pstate = tc->readMiscRegNoEffect(MISCREG_PSTATE);
    pstate.priv = 1;
    tc->setMiscReg(MISCREG_PSTATE, pstate);
}

/**
 * This sets everything up for a RED state trap except for actually jumping to
 * the handler.
 */

void
doREDFault(ThreadContext *tc, TrapType tt)
{
    RegVal TL = tc->readMiscRegNoEffect(MISCREG_TL);
    RegVal TSTATE = tc->readMiscRegNoEffect(MISCREG_TSTATE);
    PSTATE pstate = tc->readMiscRegNoEffect(MISCREG_PSTATE);
    HPSTATE hpstate = tc->readMiscRegNoEffect(MISCREG_HPSTATE);
    CCR ccr = tc->getReg(int_reg::Ccr);
    RegVal ASI = tc->readMiscRegNoEffect(MISCREG_ASI);
    RegVal CWP = tc->readMiscRegNoEffect(MISCREG_CWP);
    RegVal CANSAVE = tc->getReg(int_reg::Cansave);
    RegVal GL = tc->readMiscRegNoEffect(MISCREG_GL);
    auto &pc = tc->pcState().as<PCState>();

    TL++;

    Addr pcMask = pstate.am ? mask(32) : mask(64);

    // set TSTATE.gl to gl
    replaceBits(TSTATE, 42, 40, GL);
    // set TSTATE.ccr to ccr
    replaceBits(TSTATE, 39, 32, ccr);
    // set TSTATE.asi to asi
    replaceBits(TSTATE, 31, 24, ASI);
    // set TSTATE.pstate to pstate
    replaceBits(TSTATE, 20, 8, pstate);
    // set TSTATE.cwp to cwp
    replaceBits(TSTATE, 4, 0, CWP);

    // Write back TSTATE
    tc->setMiscRegNoEffect(MISCREG_TSTATE, TSTATE);

    // set TPC to PC
    tc->setMiscRegNoEffect(MISCREG_TPC, pc.pc() & pcMask);
    // set TNPC to NPC
    tc->setMiscRegNoEffect(MISCREG_TNPC, pc.npc() & pcMask);

    // set HTSTATE.hpstate to hpstate
    tc->setMiscRegNoEffect(MISCREG_HTSTATE, hpstate);

    // TT = trap type;
    tc->setMiscRegNoEffect(MISCREG_TT, tt);

    // Update GL
    tc->setMiscReg(MISCREG_GL, std::min<int>(GL+1, MaxGL));

    bool priv = pstate.priv; // just save the priv bit
    pstate = 0;
    pstate.priv = priv;
    pstate.pef = 1;
    tc->setMiscRegNoEffect(MISCREG_PSTATE, pstate);

    hpstate.red = 1;
    hpstate.hpriv = 1;
    hpstate.ibe = 0;
    hpstate.tlz = 0;
    tc->setMiscRegNoEffect(MISCREG_HPSTATE, hpstate);

    bool changedCWP = true;
    if (tt == 0x24)
        CWP++;
    else if (0x80 <= tt && tt <= 0xbf)
        CWP += (CANSAVE + 2);
    else if (0xc0 <= tt && tt <= 0xff)
        CWP--;
    else
        changedCWP = false;

    if (changedCWP) {
        CWP = (CWP + NWindows) % NWindows;
        tc->setMiscReg(MISCREG_CWP, CWP);
    }
}

/**
 * This sets everything up for a normal trap except for actually jumping to
 * the handler.
 */

void
doNormalFault(ThreadContext *tc, TrapType tt, bool gotoHpriv)
{
    RegVal TL = tc->readMiscRegNoEffect(MISCREG_TL);
    RegVal TSTATE = tc->readMiscRegNoEffect(MISCREG_TSTATE);
    PSTATE pstate = tc->readMiscRegNoEffect(MISCREG_PSTATE);
    HPSTATE hpstate = tc->readMiscRegNoEffect(MISCREG_HPSTATE);
    CCR ccr = tc->getReg(int_reg::Ccr);
    RegVal ASI = tc->readMiscRegNoEffect(MISCREG_ASI);
    RegVal CWP = tc->readMiscRegNoEffect(MISCREG_CWP);
    RegVal CANSAVE = tc->getReg(int_reg::Cansave);
    RegVal GL = tc->readMiscRegNoEffect(MISCREG_GL);
    auto &pc = tc->pcState().as<PCState>();

    // Increment the trap level
    TL++;
    tc->setMiscRegNoEffect(MISCREG_TL, TL);

    Addr pcMask = pstate.am ? mask(32) : mask(64);

    // Save off state

    // set TSTATE.gl to gl
    replaceBits(TSTATE, 42, 40, GL);
    // set TSTATE.ccr to ccr
    replaceBits(TSTATE, 39, 32, ccr);
    // set TSTATE.asi to asi
    replaceBits(TSTATE, 31, 24, ASI);
    // set TSTATE.pstate to pstate
    replaceBits(TSTATE, 20, 8, pstate);
    // set TSTATE.cwp to cwp
    replaceBits(TSTATE, 4, 0, CWP);

    // Write back TSTATE
    tc->setMiscRegNoEffect(MISCREG_TSTATE, TSTATE);

    // set TPC to PC
    tc->setMiscRegNoEffect(MISCREG_TPC, pc.pc() & pcMask);
    // set TNPC to NPC
    tc->setMiscRegNoEffect(MISCREG_TNPC, pc.npc() & pcMask);

    // set HTSTATE.hpstate to hpstate
    tc->setMiscRegNoEffect(MISCREG_HTSTATE, hpstate);

    // TT = trap type;
    tc->setMiscRegNoEffect(MISCREG_TT, tt);

    // Update the global register level
    if (!gotoHpriv)
        tc->setMiscReg(MISCREG_GL, std::min<int>(GL + 1, MaxPGL));
    else
        tc->setMiscReg(MISCREG_GL, std::min<int>(GL + 1, MaxGL));

    // pstate.mm is unchanged
    pstate.pef = 1; // PSTATE.pef = whether or not an fpu is present
    pstate.am = 0;
    pstate.ie = 0;
    // pstate.tle is unchanged
    // pstate.tct = 0

    if (gotoHpriv) {
        pstate.cle = 0;
        // The manual says PSTATE.priv should be 0, but Legion leaves it alone
        hpstate.red = 0;
        hpstate.hpriv = 1;
        hpstate.ibe = 0;
        // hpstate.tlz is unchanged
        tc->setMiscRegNoEffect(MISCREG_HPSTATE, hpstate);
    } else { // we are going to priv
        pstate.priv = 1;
        pstate.cle = pstate.tle;
    }
    tc->setMiscRegNoEffect(MISCREG_PSTATE, pstate);


    bool changedCWP = true;
    if (tt == 0x24)
        CWP++;
    else if (0x80 <= tt && tt <= 0xbf)
        CWP += (CANSAVE + 2);
    else if (0xc0 <= tt && tt <= 0xff)
        CWP--;
    else
        changedCWP = false;

    if (changedCWP) {
        CWP = (CWP + NWindows) % NWindows;
        tc->setMiscReg(MISCREG_CWP, CWP);
    }
}

void
getREDVector(RegVal TT, Addr &PC, Addr &NPC)
{
    //XXX The following constant might belong in a header file.
    const Addr RSTVAddr = 0xFFF0000000ULL;
    PC = RSTVAddr | ((TT << 5) & 0xFF);
    NPC = PC + sizeof(MachInst);
}

void
getHyperVector(ThreadContext * tc, Addr &PC, Addr &NPC, RegVal TT)
{
    Addr HTBA = tc->readMiscRegNoEffect(MISCREG_HTBA);
    PC = (HTBA & ~mask(14)) | ((TT << 5) & mask(14));
    NPC = PC + sizeof(MachInst);
}

void
getPrivVector(ThreadContext *tc, Addr &PC, Addr &NPC, RegVal TT, RegVal TL)
{
    Addr TBA = tc->readMiscRegNoEffect(MISCREG_TBA);
    PC = (TBA & ~mask(15)) |
        (TL > 1 ? (1 << 14) : 0) |
        ((TT << 5) & mask(14));
    NPC = PC + sizeof(MachInst);
}

void
SparcFaultBase::invoke(ThreadContext * tc, const StaticInstPtr &inst)
{
    FaultBase::invoke(tc);
    if (!FullSystem)
        return;


    // We can refer to this to see what the trap level -was-, but something
    // in the middle could change it in the regfile out from under us.
    RegVal tl = tc->readMiscRegNoEffect(MISCREG_TL);
    RegVal tt = tc->readMiscRegNoEffect(MISCREG_TT);
    PSTATE pstate = tc->readMiscRegNoEffect(MISCREG_PSTATE);
    HPSTATE hpstate = tc->readMiscRegNoEffect(MISCREG_HPSTATE);

    Addr PC, NPC;

    PrivilegeLevel current;
    if (hpstate.hpriv)
        current = Hyperprivileged;
    else if (pstate.priv)
        current = Privileged;
    else
        current = User;

    PrivilegeLevel level = getNextLevel(current);

    if (hpstate.red || (tl == MaxTL - 1)) {
        getREDVector(5, PC, NPC);
        doREDFault(tc, tt);
        // This changes the hpstate and pstate, so we need to make sure we
        // save the old version on the trap stack in doREDFault.
        enterREDState(tc);
    } else if (tl == MaxTL) {
        panic("Should go to error state here.. crap\n");
        // Do error_state somehow?
        // Probably inject a WDR fault using the interrupt mechanism.
        // What should the PC and NPC be set to?
    } else if (tl > MaxPTL && level == Privileged) {
        // guest_watchdog fault
        doNormalFault(tc, trapType(), true);
        getHyperVector(tc, PC, NPC, 2);
    } else if (level == Hyperprivileged ||
               (level == Privileged && trapType() >= 384)) {
        doNormalFault(tc, trapType(), true);
        getHyperVector(tc, PC, NPC, trapType());
    } else {
        doNormalFault(tc, trapType(), false);
        getPrivVector(tc, PC, NPC, trapType(), tl + 1);
    }

    PCState pc;
    pc.pc(PC);
    pc.npc(NPC);
    pc.nnpc(NPC + sizeof(MachInst));
    pc.upc(0);
    pc.nupc(1);
    tc->pcState(pc);
}

void
PowerOnReset::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    // For SPARC, when a system is first started, there is a power
    // on reset Trap which sets the processor into the following state.
    // Bits that aren't set aren't defined on startup.

    tc->setMiscRegNoEffect(MISCREG_TL, MaxTL);
    tc->setMiscRegNoEffect(MISCREG_TT, trapType());
    tc->setMiscReg(MISCREG_GL, MaxGL);

    PSTATE pstate = 0;
    pstate.pef = 1;
    pstate.priv = 1;
    tc->setMiscRegNoEffect(MISCREG_PSTATE, pstate);

    // Turn on red and hpriv, set everything else to 0
    HPSTATE hpstate = tc->readMiscRegNoEffect(MISCREG_HPSTATE);
    hpstate.red = 1;
    hpstate.hpriv = 1;
    hpstate.ibe = 0;
    hpstate.tlz = 0;
    tc->setMiscRegNoEffect(MISCREG_HPSTATE, hpstate);

    // The tick register is unreadable by nonprivileged software
    tc->setMiscRegNoEffect(MISCREG_TICK, 1ULL << 63);

    // Enter RED state. We do this last so that the actual state preserved in
    // the trap stack is the state from before this fault.
    enterREDState(tc);

    Addr PC, NPC;
    getREDVector(trapType(), PC, NPC);

    PCState pc;
    pc.pc(PC);
    pc.npc(NPC);
    pc.nnpc(NPC + sizeof(MachInst));
    pc.upc(0);
    pc.nupc(1);
    tc->pcState(pc);

    // These registers are specified as "undefined" after a POR, and they
    // should have reasonable values after the miscregfile is reset
    /*
    // Clear all the soft interrupt bits
    softint = 0;
    // disable timer compare interrupts, reset tick_cmpr
    tc->setMiscRegNoEffect(MISCREG_
    tick_cmprFields.int_dis = 1;
    tick_cmprFields.tick_cmpr = 0; // Reset to 0 for pretty printing
    stickFields.npt = 1; // The TICK register is unreadable by by !priv
    stick_cmprFields.int_dis = 1; // disable timer compare interrupts
    stick_cmprFields.tick_cmpr = 0; // Reset to 0 for pretty printing

    tt[tl] = _trapType;

    hintp = 0; // no interrupts pending
    hstick_cmprFields.int_dis = 1; // disable timer compare interrupts
    hstick_cmprFields.tick_cmpr = 0; // Reset to 0 for pretty printing
    */
}

void
FastInstructionAccessMMUMiss::invoke(ThreadContext *tc,
                                     const StaticInstPtr &inst)
{
    if (FullSystem) {
        SparcFaultBase::invoke(tc, inst);
        return;
    }

    Process *p = tc->getProcessPtr();
    const EmulationPageTable::Entry *pte = p->pTable->lookup(vaddr);
    panic_if(!pte, "Tried to execute unmapped address %#x.\n", vaddr);

    Addr alignedvaddr = p->pTable->pageAlign(vaddr);

    // Grab fields used during instruction translation to figure out
    // which context to use.
    uint64_t tlbdata = tc->readMiscRegNoEffect(MISCREG_TLB_DATA);

    // Inside a VM, a real address is the address that guest OS would
    // interpret to be a physical address. To map to the physical address,
    // it still needs to undergo a translation. The instruction
    // translation code in the SPARC ITLB code assumes that the context is
    // zero (kernel-level) if real addressing is being used.
    bool is_real_address = !bits(tlbdata, 4);

    // The SPARC ITLB code assumes that traps are executed in context
    // zero so we carry that assumption through here.
    bool trapped = bits(tlbdata, 18, 16) > 0;

    // The primary context acts as a PASID. It allows the MMU to
    // distinguish between virtual addresses that would alias to the
    // same physical address (if two or more processes shared the same
    // virtual address mapping).
    int primary_context = bits(tlbdata, 47, 32);

    // The partition id distinguishes between virtualized environments.
    int const partition_id = 0;

    // Given the assumptions in the translateInst code in the SPARC ITLB,
    // the logic works out to the following for the context.
    int context_id = (is_real_address || trapped) ? 0 : primary_context;

    TlbEntry entry(p->pTable->pid(), alignedvaddr, pte->paddr,
                   pte->flags & EmulationPageTable::Uncacheable,
                   pte->flags & EmulationPageTable::ReadOnly);

    // Insert the TLB entry.
    // The entry specifying whether the address is "real" is set to
    // false for syscall emulation mode regardless of whether the
    // address is real in preceding code. Not sure sure that this is
    // correct, but also not sure if it matters at all.
    static_cast<MMU *>(tc->getMMUPtr())->insertItlbEntry(
        alignedvaddr, partition_id, context_id,
        false, entry.pte);
}

void
FastDataAccessMMUMiss::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    if (FullSystem) {
        SparcFaultBase::invoke(tc, inst);
        return;
    }

    Process *p = tc->getProcessPtr();
    const EmulationPageTable::Entry *pte = p->pTable->lookup(vaddr);
    if (!pte && p->fixupFault(vaddr))
        pte = p->pTable->lookup(vaddr);
    panic_if(!pte, "Tried to access unmapped address %#x.\n", vaddr);

    Addr alignedvaddr = p->pTable->pageAlign(vaddr);

    // Grab fields used during data translation to figure out
    // which context to use.
    uint64_t tlbdata = tc->readMiscRegNoEffect(MISCREG_TLB_DATA);

    // The primary context acts as a PASID. It allows the MMU to
    // distinguish between virtual addresses that would alias to the
    // same physical address (if two or more processes shared the same
    // virtual address mapping). There's a secondary context used in the
    // DTLB translation code, but it should __probably__ be zero for
    // syscall emulation code. (The secondary context is used by Solaris
    // to allow kernel privilege code to access user space code:
    // [ISBN 0-13-022496-0]:PG199.)
    int primary_context = bits(tlbdata, 47, 32);

    // "Hyper-Privileged Mode" is in use. There are three main modes of
    // operation for Sparc: Hyper-Privileged Mode, Privileged Mode, and
    // User Mode.
    int hpriv = bits(tlbdata, 0);

    // Reset, Error and Debug state is in use. Something horrible has
    // happened or the system is operating in Reset Mode.
    int red = bits(tlbdata, 1);

    // Inside a VM, a real address is the address that guest OS would
    // interpret to be a physical address. To map to the physical address,
    // it still needs to undergo a translation. The instruction
    // translation code in the SPARC ITLB code assumes that the context is
    // zero (kernel-level) if real addressing is being used.
    int is_real_address = !bits(tlbdata, 5);

    // Grab the address space identifier register from the thread context.
    // XXX: Inspecting how setMiscReg and setMiscRegNoEffect behave for
    // MISCREG_ASI causes me to think that the ASI register implementation
    // might be bugged. The NoEffect variant changes the ASI register
    // value in the architectural state while the normal variant changes
    // the context field in the thread context's currently decoded request
    // but does not directly affect the ASI register value in the
    // architectural state. The ASI values and the context field in the
    // request packet seem to have completely different uses.
    RegVal reg_asi = tc->readMiscRegNoEffect(MISCREG_ASI);
    ASI asi = static_cast<ASI>(reg_asi);

    // The SPARC DTLB code assumes that traps are executed in context
    // zero if the asi value is ASI_IMPLICIT (which is 0x0). There's also
    // an assumption that the nucleus address space is being used, but
    // the context is the relevant issue since we need to pass it to TLB.
    bool trapped = bits(tlbdata, 18, 16) > 0;

    // Given the assumptions in the translateData code in the SPARC DTLB,
    // the logic works out to the following for the context.
    int context_id = ((!hpriv && !red && is_real_address) ||
                      asiIsReal(asi) ||
                      (trapped && asi == ASI_IMPLICIT))
                     ? 0 : primary_context;

    // The partition id distinguishes between virtualized environments.
    int const partition_id = 0;

    TlbEntry entry(p->pTable->pid(), alignedvaddr, pte->paddr,
                   pte->flags & EmulationPageTable::Uncacheable,
                   pte->flags & EmulationPageTable::ReadOnly);

    // Insert the TLB entry.
    // The entry specifying whether the address is "real" is set to
    // false for syscall emulation mode regardless of whether the
    // address is real in preceding code. Not sure sure that this is
    // correct, but also not sure if it matters at all.
    static_cast<MMU *>(tc->getMMUPtr())->insertDtlbEntry(
        alignedvaddr, partition_id, context_id,
        false, entry.pte);
}

void
SpillNNormal::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    if (FullSystem) {
        SparcFaultBase::invoke(tc, inst);
        return;
    }

    doNormalFault(tc, trapType(), false);

    Process *p = tc->getProcessPtr();

    SparcProcess *sp = dynamic_cast<SparcProcess *>(p);
    assert(sp);

    // Then adjust the PC and NPC
    tc->pcState(sp->readSpillStart());
}

void
FillNNormal::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    if (FullSystem) {
        SparcFaultBase::invoke(tc, inst);
        return;
    }

    doNormalFault(tc, trapType(), false);

    Process *p = tc->getProcessPtr();

    SparcProcess *sp = dynamic_cast<SparcProcess *>(p);
    assert(sp);

    // Then adjust the PC and NPC
    tc->pcState(sp->readFillStart());
}

void
TrapInstruction::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    if (FullSystem) {
        SparcFaultBase::invoke(tc, inst);
        return;
    }

    // In SE, this mechanism is how the process requests a service from
    // the operating system. We'll get the process object from the thread
    // context and let it service the request.

    Process *p = tc->getProcessPtr();

    [[maybe_unused]] SparcProcess *sp = dynamic_cast<SparcProcess *>(p);
    assert(sp);

    auto *workload = dynamic_cast<SEWorkload *>(tc->getSystemPtr()->workload);
    workload->handleTrap(tc, _n);

    // We need to explicitly advance the pc, since that's not done for us
    // on a faulting instruction
    PCState pc = tc->pcState().as<PCState>();
    pc.advance();
    tc->pcState(pc);
}

} // namespace SparcISA
} // namespace gem5
