/*
 * Copyright (c) 2010, 2012-2014, 2016-2019 ARM Limited
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
 */

#include "arch/arm/faults.hh"

#include "arch/arm/insts/static_inst.hh"
#include "arch/arm/interrupts.hh"
#include "arch/arm/isa.hh"
#include "arch/arm/self_debug.hh"
#include "arch/arm/system.hh"
#include "arch/arm/utility.hh"
#include "base/compiler.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/Faults.hh"
#include "sim/full_system.hh"

namespace ArmISA
{

const uint32_t HighVecs = 0xFFFF0000;

uint8_t ArmFault::shortDescFaultSources[] = {
    0x01,  // AlignmentFault
    0x04,  // InstructionCacheMaintenance
    0xff,  // SynchExtAbtOnTranslTableWalkL0 (INVALID)
    0x0c,  // SynchExtAbtOnTranslTableWalkL1
    0x0e,  // SynchExtAbtOnTranslTableWalkL2
    0xff,  // SynchExtAbtOnTranslTableWalkL3 (INVALID)
    0xff,  // SynchPtyErrOnTranslTableWalkL0 (INVALID)
    0x1c,  // SynchPtyErrOnTranslTableWalkL1
    0x1e,  // SynchPtyErrOnTranslTableWalkL2
    0xff,  // SynchPtyErrOnTranslTableWalkL3 (INVALID)
    0xff,  // TranslationL0 (INVALID)
    0x05,  // TranslationL1
    0x07,  // TranslationL2
    0xff,  // TranslationL3 (INVALID)
    0xff,  // AccessFlagL0 (INVALID)
    0x03,  // AccessFlagL1
    0x06,  // AccessFlagL2
    0xff,  // AccessFlagL3 (INVALID)
    0xff,  // DomainL0 (INVALID)
    0x09,  // DomainL1
    0x0b,  // DomainL2
    0xff,  // DomainL3 (INVALID)
    0xff,  // PermissionL0 (INVALID)
    0x0d,  // PermissionL1
    0x0f,  // PermissionL2
    0xff,  // PermissionL3 (INVALID)
    0x02,  // DebugEvent
    0x08,  // SynchronousExternalAbort
    0x10,  // TLBConflictAbort
    0x19,  // SynchPtyErrOnMemoryAccess
    0x16,  // AsynchronousExternalAbort
    0x18,  // AsynchPtyErrOnMemoryAccess
    0xff,  // AddressSizeL0 (INVALID)
    0xff,  // AddressSizeL1 (INVALID)
    0xff,  // AddressSizeL2 (INVALID)
    0xff,  // AddressSizeL3 (INVALID)
    0x40,  // PrefetchTLBMiss
    0x80   // PrefetchUncacheable
};

static_assert(sizeof(ArmFault::shortDescFaultSources) ==
              ArmFault::NumFaultSources,
              "Invalid size of ArmFault::shortDescFaultSources[]");

uint8_t ArmFault::longDescFaultSources[] = {
    0x21,  // AlignmentFault
    0xff,  // InstructionCacheMaintenance (INVALID)
    0xff,  // SynchExtAbtOnTranslTableWalkL0 (INVALID)
    0x15,  // SynchExtAbtOnTranslTableWalkL1
    0x16,  // SynchExtAbtOnTranslTableWalkL2
    0x17,  // SynchExtAbtOnTranslTableWalkL3
    0xff,  // SynchPtyErrOnTranslTableWalkL0 (INVALID)
    0x1d,  // SynchPtyErrOnTranslTableWalkL1
    0x1e,  // SynchPtyErrOnTranslTableWalkL2
    0x1f,  // SynchPtyErrOnTranslTableWalkL3
    0xff,  // TranslationL0 (INVALID)
    0x05,  // TranslationL1
    0x06,  // TranslationL2
    0x07,  // TranslationL3
    0xff,  // AccessFlagL0 (INVALID)
    0x09,  // AccessFlagL1
    0x0a,  // AccessFlagL2
    0x0b,  // AccessFlagL3
    0xff,  // DomainL0 (INVALID)
    0x3d,  // DomainL1
    0x3e,  // DomainL2
    0xff,  // DomainL3 (RESERVED)
    0xff,  // PermissionL0 (INVALID)
    0x0d,  // PermissionL1
    0x0e,  // PermissionL2
    0x0f,  // PermissionL3
    0x22,  // DebugEvent
    0x10,  // SynchronousExternalAbort
    0x30,  // TLBConflictAbort
    0x18,  // SynchPtyErrOnMemoryAccess
    0x11,  // AsynchronousExternalAbort
    0x19,  // AsynchPtyErrOnMemoryAccess
    0xff,  // AddressSizeL0 (INVALID)
    0xff,  // AddressSizeL1 (INVALID)
    0xff,  // AddressSizeL2 (INVALID)
    0xff,  // AddressSizeL3 (INVALID)
    0x40,  // PrefetchTLBMiss
    0x80   // PrefetchUncacheable
};

static_assert(sizeof(ArmFault::longDescFaultSources) ==
              ArmFault::NumFaultSources,
              "Invalid size of ArmFault::longDescFaultSources[]");

uint8_t ArmFault::aarch64FaultSources[] = {
    0x21,  // AlignmentFault
    0xff,  // InstructionCacheMaintenance (INVALID)
    0x14,  // SynchExtAbtOnTranslTableWalkL0
    0x15,  // SynchExtAbtOnTranslTableWalkL1
    0x16,  // SynchExtAbtOnTranslTableWalkL2
    0x17,  // SynchExtAbtOnTranslTableWalkL3
    0x1c,  // SynchPtyErrOnTranslTableWalkL0
    0x1d,  // SynchPtyErrOnTranslTableWalkL1
    0x1e,  // SynchPtyErrOnTranslTableWalkL2
    0x1f,  // SynchPtyErrOnTranslTableWalkL3
    0x04,  // TranslationL0
    0x05,  // TranslationL1
    0x06,  // TranslationL2
    0x07,  // TranslationL3
    0x08,  // AccessFlagL0
    0x09,  // AccessFlagL1
    0x0a,  // AccessFlagL2
    0x0b,  // AccessFlagL3
    // @todo: Section & Page Domain Fault in AArch64?
    0xff,  // DomainL0 (INVALID)
    0xff,  // DomainL1 (INVALID)
    0xff,  // DomainL2 (INVALID)
    0xff,  // DomainL3 (INVALID)
    0x0c,  // PermissionL0
    0x0d,  // PermissionL1
    0x0e,  // PermissionL2
    0x0f,  // PermissionL3
    0x22,  // DebugEvent
    0x10,  // SynchronousExternalAbort
    0x30,  // TLBConflictAbort
    0x18,  // SynchPtyErrOnMemoryAccess
    0xff,  // AsynchronousExternalAbort (INVALID)
    0xff,  // AsynchPtyErrOnMemoryAccess (INVALID)
    0x00,  // AddressSizeL0
    0x01,  // AddressSizeL1
    0x02,  // AddressSizeL2
    0x03,  // AddressSizeL3
    0x40,  // PrefetchTLBMiss
    0x80   // PrefetchUncacheable
};

static_assert(sizeof(ArmFault::aarch64FaultSources) ==
              ArmFault::NumFaultSources,
              "Invalid size of ArmFault::aarch64FaultSources[]");

// Fields: name, offset, cur{ELT,ELH}Offset, lowerEL{64,32}Offset, next mode,
//         {ARM, Thumb, ARM_ELR, Thumb_ELR} PC offset, hyp trap,
//         {A, F} disable, class, stat
template<> ArmFault::FaultVals ArmFaultVals<Reset>::vals(
    // Some dummy values (the reset vector has an IMPLEMENTATION DEFINED
    // location in AArch64)
    "Reset",                 0x000, 0x000, 0x000, 0x000, 0x000, MODE_SVC,
    0, 0, 0, 0, false, true,  true,  EC_UNKNOWN
);
template<> ArmFault::FaultVals ArmFaultVals<UndefinedInstruction>::vals(
    "Undefined Instruction", 0x004, 0x000, 0x200, 0x400, 0x600, MODE_UNDEFINED,
    4, 2, 0, 0, true,  false, false, EC_UNKNOWN
);
template<> ArmFault::FaultVals ArmFaultVals<SupervisorCall>::vals(
    "Supervisor Call",       0x008, 0x000, 0x200, 0x400, 0x600, MODE_SVC,
    4, 2, 4, 2, true,  false, false, EC_SVC_TO_HYP
);
template<> ArmFault::FaultVals ArmFaultVals<SecureMonitorCall>::vals(
    "Secure Monitor Call",   0x008, 0x000, 0x200, 0x400, 0x600, MODE_MON,
    4, 4, 4, 4, false, true,  true,  EC_SMC_TO_HYP
);
template<> ArmFault::FaultVals ArmFaultVals<HypervisorCall>::vals(
    "Hypervisor Call",       0x008, 0x000, 0x200, 0x400, 0x600, MODE_HYP,
    4, 4, 4, 4, true,  false, false, EC_HVC
);
template<> ArmFault::FaultVals ArmFaultVals<PrefetchAbort>::vals(
    "Prefetch Abort",        0x00C, 0x000, 0x200, 0x400, 0x600, MODE_ABORT,
    4, 4, 0, 0, true,  true,  false, EC_PREFETCH_ABORT_TO_HYP
);
template<> ArmFault::FaultVals ArmFaultVals<DataAbort>::vals(
    "Data Abort",            0x010, 0x000, 0x200, 0x400, 0x600, MODE_ABORT,
    8, 8, 0, 0, true,  true,  false, EC_DATA_ABORT_TO_HYP
);
template<> ArmFault::FaultVals ArmFaultVals<VirtualDataAbort>::vals(
    "Virtual Data Abort",    0x010, 0x000, 0x200, 0x400, 0x600, MODE_ABORT,
    8, 8, 0, 0, true,  true,  false, EC_INVALID
);
template<> ArmFault::FaultVals ArmFaultVals<HypervisorTrap>::vals(
    // @todo: double check these values
    "Hypervisor Trap",       0x014, 0x000, 0x200, 0x400, 0x600, MODE_HYP,
    0, 0, 0, 0, false, false, false, EC_UNKNOWN
);
template<> ArmFault::FaultVals ArmFaultVals<SecureMonitorTrap>::vals(
    "Secure Monitor Trap",   0x004, 0x000, 0x200, 0x400, 0x600, MODE_MON,
    4, 2, 0, 0, false, false, false, EC_UNKNOWN
);
template<> ArmFault::FaultVals ArmFaultVals<Interrupt>::vals(
    "IRQ",                   0x018, 0x080, 0x280, 0x480, 0x680, MODE_IRQ,
    4, 4, 0, 0, false, true,  false, EC_UNKNOWN
);
template<> ArmFault::FaultVals ArmFaultVals<VirtualInterrupt>::vals(
    "Virtual IRQ",           0x018, 0x080, 0x280, 0x480, 0x680, MODE_IRQ,
    4, 4, 0, 0, false, true,  false, EC_INVALID
);
template<> ArmFault::FaultVals ArmFaultVals<FastInterrupt>::vals(
    "FIQ",                   0x01C, 0x100, 0x300, 0x500, 0x700, MODE_FIQ,
    4, 4, 0, 0, false, true,  true,  EC_UNKNOWN
);
template<> ArmFault::FaultVals ArmFaultVals<VirtualFastInterrupt>::vals(
    "Virtual FIQ",           0x01C, 0x100, 0x300, 0x500, 0x700, MODE_FIQ,
    4, 4, 0, 0, false, true,  true,  EC_INVALID
);
template<> ArmFault::FaultVals ArmFaultVals<IllegalInstSetStateFault>::vals(
    "Illegal Inst Set State Fault",   0x004, 0x000, 0x200, 0x400, 0x600, MODE_UNDEFINED,
    4, 2, 0, 0, true, false, false, EC_ILLEGAL_INST
);
template<> ArmFault::FaultVals ArmFaultVals<SupervisorTrap>::vals(
    // Some dummy values (SupervisorTrap is AArch64-only)
    "Supervisor Trap",   0x014, 0x000, 0x200, 0x400, 0x600, MODE_SVC,
    0, 0, 0, 0, false, false, false, EC_UNKNOWN
);
template<> ArmFault::FaultVals ArmFaultVals<PCAlignmentFault>::vals(
    // Some dummy values (PCAlignmentFault is AArch64-only)
    "PC Alignment Fault",   0x000, 0x000, 0x200, 0x400, 0x600, MODE_SVC,
    0, 0, 0, 0, true, false, false, EC_PC_ALIGNMENT
);
template<> ArmFault::FaultVals ArmFaultVals<SPAlignmentFault>::vals(
    // Some dummy values (SPAlignmentFault is AArch64-only)
    "SP Alignment Fault",   0x000, 0x000, 0x200, 0x400, 0x600, MODE_SVC,
    0, 0, 0, 0, true, false, false, EC_STACK_PTR_ALIGNMENT
);
template<> ArmFault::FaultVals ArmFaultVals<SystemError>::vals(
    // Some dummy values (SError is AArch64-only)
    "SError",                0x000, 0x180, 0x380, 0x580, 0x780, MODE_SVC,
    0, 0, 0, 0, false, true,  true,  EC_SERROR
);
template<> ArmFault::FaultVals ArmFaultVals<SoftwareBreakpoint>::vals(
    // Some dummy values (SoftwareBreakpoint is AArch64-only)
    "Software Breakpoint",   0x000, 0x000, 0x200, 0x400, 0x600, MODE_SVC,
    0, 0, 0, 0, true, false, false,  EC_SOFTWARE_BREAKPOINT
);
template<> ArmFault::FaultVals ArmFaultVals<HardwareBreakpoint>::vals(
    "Hardware Breakpoint",   0x000, 0x000, 0x200, 0x400, 0x600, MODE_SVC,
    0, 0, 0, 0, true, false, false,  EC_HW_BREAKPOINT
);
template<> ArmFault::FaultVals ArmFaultVals<Watchpoint>::vals(
    "Watchpoint",   0x000, 0x000, 0x200, 0x400, 0x600, MODE_SVC,
    0, 0, 0, 0, true, false, false,  EC_WATCHPOINT
);
template<> ArmFault::FaultVals ArmFaultVals<SoftwareStepFault>::vals(
    "SoftwareStep",   0x000, 0x000, 0x200, 0x400, 0x600, MODE_SVC,
    0, 0, 0, 0, true, false, false,  EC_SOFTWARE_STEP
);
template<> ArmFault::FaultVals ArmFaultVals<ArmSev>::vals(
    // Some dummy values
    "ArmSev Flush",          0x000, 0x000, 0x000, 0x000, 0x000, MODE_SVC,
    0, 0, 0, 0, false, true,  true,  EC_UNKNOWN
);

Addr
ArmFault::getVector(ThreadContext *tc)
{
    Addr base;

    // Check for invalid modes
    CPSR cpsr = tc->readMiscRegNoEffect(MISCREG_CPSR);
    assert(ArmSystem::haveSecurity(tc) || cpsr.mode != MODE_MON);
    assert(ArmSystem::haveVirtualization(tc) || cpsr.mode != MODE_HYP);

    switch (cpsr.mode)
    {
      case MODE_MON:
        base = tc->readMiscReg(MISCREG_MVBAR);
        break;
      case MODE_HYP:
        base = tc->readMiscReg(MISCREG_HVBAR);
        break;
      default:
        SCTLR sctlr = tc->readMiscReg(MISCREG_SCTLR);
        if (sctlr.v) {
            base = HighVecs;
        } else {
            base = ArmSystem::haveSecurity(tc) ?
                tc->readMiscReg(MISCREG_VBAR) : 0;
        }
        break;
    }

    return base + offset(tc);
}

Addr
ArmFault::getVector64(ThreadContext *tc)
{
    Addr vbar;
    switch (toEL) {
      case EL3:
        assert(ArmSystem::haveSecurity(tc));
        vbar = tc->readMiscReg(MISCREG_VBAR_EL3);
        break;
      case EL2:
        assert(ArmSystem::haveVirtualization(tc));
        vbar = tc->readMiscReg(MISCREG_VBAR_EL2);
        break;
      case EL1:
        vbar = tc->readMiscReg(MISCREG_VBAR_EL1);
        break;
      default:
        panic("Invalid target exception level");
        break;
    }
    return vbar + offset64(tc);
}

MiscRegIndex
ArmFault::getSyndromeReg64() const
{
    switch (toEL) {
      case EL1:
        return MISCREG_ESR_EL1;
      case EL2:
        return MISCREG_ESR_EL2;
      case EL3:
        return MISCREG_ESR_EL3;
      default:
        panic("Invalid exception level");
        break;
    }
}

MiscRegIndex
ArmFault::getFaultAddrReg64() const
{
    switch (toEL) {
      case EL1:
        return MISCREG_FAR_EL1;
      case EL2:
        return MISCREG_FAR_EL2;
      case EL3:
        return MISCREG_FAR_EL3;
      default:
        panic("Invalid exception level");
        break;
    }
}

void
ArmFault::setSyndrome(ThreadContext *tc, MiscRegIndex syndrome_reg)
{
    uint32_t value;
    uint32_t exc_class = (uint32_t) ec(tc);
    uint32_t issVal = iss();

    assert(!from64 || ArmSystem::highestELIs64(tc));

    value = exc_class << 26;

    // HSR.IL not valid for Prefetch Aborts (0x20, 0x21) and Data Aborts (0x24,
    // 0x25) for which the ISS information is not valid (ARMv7).
    // @todo: ARMv8 revises AArch32 functionality: when HSR.IL is not
    // valid it is treated as RES1.
    if (to64) {
        value |= 1 << 25;
    } else if ((bits(exc_class, 5, 3) != 4) ||
               (bits(exc_class, 2) && bits(issVal, 24))) {
        if (!machInst.thumb || machInst.bigThumb)
            value |= 1 << 25;
    }
    // Condition code valid for EC[5:4] nonzero
    if (!from64 && ((bits(exc_class, 5, 4) == 0) &&
                    (bits(exc_class, 3, 0) != 0))) {
        if (!machInst.thumb) {
            uint32_t      cond;
            ConditionCode condCode = (ConditionCode) (uint32_t) machInst.condCode;
            // If its on unconditional instruction report with a cond code of
            // 0xE, ie the unconditional code
            cond  = (condCode == COND_UC) ? COND_AL : condCode;
            value |= cond << 20;
            value |= 1    << 24;
        }
        value |= bits(issVal, 19, 0);
    } else {
        value |= issVal;
    }
    tc->setMiscReg(syndrome_reg, value);
}

void
ArmFault::update(ThreadContext *tc)
{
    CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);

    // Determine source exception level and mode
    fromMode = (OperatingMode) (uint8_t) cpsr.mode;
    fromEL = opModeToEL(fromMode);
    if (opModeIs64(fromMode))
        from64 = true;

    // Determine target exception level (aarch64) or target execution
    // mode (aarch32).
    if (ArmSystem::haveSecurity(tc) && routeToMonitor(tc)) {
        toMode = MODE_MON;
        toEL = EL3;
    } else if (ArmSystem::haveVirtualization(tc) && routeToHyp(tc)) {
        toMode = MODE_HYP;
        toEL = EL2;
        hypRouted = true;
    } else {
        toMode = nextMode();
        toEL = opModeToEL(toMode);
    }

    if (fromEL > toEL)
        toEL = fromEL;

    // Check for Set Priviledge Access Never, if PAN is supported
    AA64MMFR1 mmfr1 = tc->readMiscReg(MISCREG_ID_AA64MMFR1_EL1);
    if (mmfr1.pan) {
        if (toEL == EL1) {
            const SCTLR sctlr = tc->readMiscReg(MISCREG_SCTLR_EL1);
            span = !sctlr.span;
        }

        const HCR hcr = tc->readMiscRegNoEffect(MISCREG_HCR_EL2);
        if (toEL == EL2 && hcr.e2h && hcr.tge) {
            const SCTLR sctlr = tc->readMiscReg(MISCREG_SCTLR_EL2);
            span = !sctlr.span;
        }
    }

    to64 = ELIs64(tc, toEL);

    // The fault specific informations have been updated; it is
    // now possible to use them inside the fault.
    faultUpdated = true;
}

void
ArmFault::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    // Update fault state informations, like the starting mode (aarch32)
    // or EL (aarch64) and the ending mode or EL.
    // From the update function we are also evaluating if the fault must
    // be handled in AArch64 mode (to64).
    update(tc);

    if (to64) {
        // Invoke exception handler in AArch64 state
        invoke64(tc, inst);
        return;
    }

    if (vectorCatch(tc, inst))
        return;

    // ARMv7 (ARM ARM issue C B1.9)

    bool have_security       = ArmSystem::haveSecurity(tc);

    FaultBase::invoke(tc);
    if (!FullSystem)
        return;
    countStat()++;

    SCTLR sctlr = tc->readMiscReg(MISCREG_SCTLR);
    SCR scr = tc->readMiscReg(MISCREG_SCR);
    CPSR saved_cpsr = tc->readMiscReg(MISCREG_CPSR);
    saved_cpsr.nz = tc->readCCReg(CCREG_NZ);
    saved_cpsr.c = tc->readCCReg(CCREG_C);
    saved_cpsr.v = tc->readCCReg(CCREG_V);
    saved_cpsr.ge = tc->readCCReg(CCREG_GE);

    Addr curPc M5_VAR_USED = tc->pcState().pc();
    ITSTATE it = tc->pcState().itstate();
    saved_cpsr.it2 = it.top6;
    saved_cpsr.it1 = it.bottom2;

    // if we have a valid instruction then use it to annotate this fault with
    // extra information. This is used to generate the correct fault syndrome
    // information
    ArmStaticInst *arm_inst M5_VAR_USED = instrAnnotate(inst);

    // Ensure Secure state if initially in Monitor mode
    if (have_security && saved_cpsr.mode == MODE_MON) {
        SCR scr = tc->readMiscRegNoEffect(MISCREG_SCR);
        if (scr.ns) {
            scr.ns = 0;
            tc->setMiscRegNoEffect(MISCREG_SCR, scr);
        }
    }

    CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);
    cpsr.mode = toMode;

    // some bits are set differently if we have been routed to hyp mode
    if (cpsr.mode == MODE_HYP) {
        SCTLR hsctlr = tc->readMiscReg(MISCREG_HSCTLR);
        cpsr.t = hsctlr.te;
        cpsr.e = hsctlr.ee;
        if (!scr.ea)  {cpsr.a = 1;}
        if (!scr.fiq) {cpsr.f = 1;}
        if (!scr.irq) {cpsr.i = 1;}
    } else if (cpsr.mode == MODE_MON) {
        // Special case handling when entering monitor mode
        cpsr.t = sctlr.te;
        cpsr.e = sctlr.ee;
        cpsr.a = 1;
        cpsr.f = 1;
        cpsr.i = 1;
    } else {
        cpsr.t = sctlr.te;
        cpsr.e = sctlr.ee;

        // The *Disable functions are virtual and different per fault
        cpsr.a = cpsr.a | abortDisable(tc);
        cpsr.f = cpsr.f | fiqDisable(tc);
        cpsr.i = 1;
    }
    cpsr.it1 = cpsr.it2 = 0;
    cpsr.j = 0;
    cpsr.pan = span ? 1 : saved_cpsr.pan;
    tc->setMiscReg(MISCREG_CPSR, cpsr);

    // Make sure mailbox sets to one always
    tc->setMiscReg(MISCREG_SEV_MAILBOX, 1);

    // Clear the exclusive monitor
    tc->setMiscReg(MISCREG_LOCKFLAG, 0);

    if (cpsr.mode == MODE_HYP) {
        tc->setMiscReg(MISCREG_ELR_HYP, curPc +
                (saved_cpsr.t ? thumbPcOffset(true)  : armPcOffset(true)));
    } else {
        tc->setIntReg(INTREG_LR, curPc +
                (saved_cpsr.t ? thumbPcOffset(false) : armPcOffset(false)));
    }

    switch (cpsr.mode) {
      case MODE_FIQ:
        tc->setMiscReg(MISCREG_SPSR_FIQ, saved_cpsr);
        break;
      case MODE_IRQ:
        tc->setMiscReg(MISCREG_SPSR_IRQ, saved_cpsr);
        break;
      case MODE_SVC:
        tc->setMiscReg(MISCREG_SPSR_SVC, saved_cpsr);
        break;
      case MODE_MON:
        assert(have_security);
        tc->setMiscReg(MISCREG_SPSR_MON, saved_cpsr);
        break;
      case MODE_ABORT:
        tc->setMiscReg(MISCREG_SPSR_ABT, saved_cpsr);
        break;
      case MODE_UNDEFINED:
        tc->setMiscReg(MISCREG_SPSR_UND, saved_cpsr);
        if (ec(tc) != EC_UNKNOWN)
            setSyndrome(tc, MISCREG_HSR);
        break;
      case MODE_HYP:
        assert(ArmSystem::haveVirtualization(tc));
        tc->setMiscReg(MISCREG_SPSR_HYP, saved_cpsr);
        setSyndrome(tc, MISCREG_HSR);
        break;
      default:
        panic("unknown Mode\n");
    }

    Addr newPc = getVector(tc);
    DPRINTF(Faults, "Invoking Fault:%s cpsr:%#x PC:%#x lr:%#x newVec: %#x "
            "%s\n", name(), cpsr, curPc, tc->readIntReg(INTREG_LR),
            newPc, arm_inst ? csprintf("inst: %#x", arm_inst->encoding()) :
            std::string());
    PCState pc(newPc);
    pc.thumb(cpsr.t);
    pc.nextThumb(pc.thumb());
    pc.jazelle(cpsr.j);
    pc.nextJazelle(pc.jazelle());
    pc.aarch64(!cpsr.width);
    pc.nextAArch64(!cpsr.width);
    pc.illegalExec(false);
    tc->pcState(pc);
}

void
ArmFault::invoke64(ThreadContext *tc, const StaticInstPtr &inst)
{
    // Determine actual misc. register indices for ELR_ELx and SPSR_ELx
    MiscRegIndex elr_idx, spsr_idx;
    switch (toEL) {
      case EL1:
        elr_idx = MISCREG_ELR_EL1;
        spsr_idx = MISCREG_SPSR_EL1;
        break;
      case EL2:
        assert(ArmSystem::haveVirtualization(tc));
        elr_idx = MISCREG_ELR_EL2;
        spsr_idx = MISCREG_SPSR_EL2;
        break;
      case EL3:
        assert(ArmSystem::haveSecurity(tc));
        elr_idx = MISCREG_ELR_EL3;
        spsr_idx = MISCREG_SPSR_EL3;
        break;
      default:
        panic("Invalid target exception level");
        break;
    }

    // Save process state into SPSR_ELx
    CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);
    CPSR spsr = cpsr;
    spsr.nz = tc->readCCReg(CCREG_NZ);
    spsr.c = tc->readCCReg(CCREG_C);
    spsr.v = tc->readCCReg(CCREG_V);
    spsr.ss = isResetSPSR() ? 0: cpsr.ss;
    if (from64) {
        // Force some bitfields to 0
        spsr.q = 0;
        spsr.it1 = 0;
        spsr.j = 0;
        spsr.ge = 0;
        spsr.it2 = 0;
        spsr.t = 0;
    } else {
        spsr.ge = tc->readCCReg(CCREG_GE);
        ITSTATE it = tc->pcState().itstate();
        spsr.it2 = it.top6;
        spsr.it1 = it.bottom2;
    }
    tc->setMiscReg(spsr_idx, spsr);

    // Save preferred return address into ELR_ELx
    Addr curr_pc = tc->pcState().pc();
    Addr ret_addr = curr_pc;
    if (from64)
        ret_addr += armPcElrOffset();
    else
        ret_addr += spsr.t ? thumbPcElrOffset() : armPcElrOffset();
    tc->setMiscReg(elr_idx, ret_addr);

    Addr vec_address = getVector64(tc);

    // Update process state
    OperatingMode64 mode = 0;
    mode.spX = 1;
    mode.el = toEL;
    mode.width = 0;
    cpsr.mode = mode;
    cpsr.daif = 0xf;
    cpsr.il = 0;
    cpsr.ss = 0;
    cpsr.pan = span ? 1 : spsr.pan;
    tc->setMiscReg(MISCREG_CPSR, cpsr);

    // If we have a valid instruction then use it to annotate this fault with
    // extra information. This is used to generate the correct fault syndrome
    // information
    ArmStaticInst *arm_inst M5_VAR_USED = instrAnnotate(inst);

    // Set PC to start of exception handler
    Addr new_pc = purifyTaggedAddr(vec_address, tc, toEL, true);
    DPRINTF(Faults, "Invoking Fault (AArch64 target EL):%s cpsr:%#x PC:%#x "
            "elr:%#x newVec: %#x %s\n", name(), cpsr, curr_pc, ret_addr,
            new_pc, arm_inst ? csprintf("inst: %#x", arm_inst->encoding()) :
            std::string());
    PCState pc(new_pc);
    pc.aarch64(!cpsr.width);
    pc.nextAArch64(!cpsr.width);
    pc.illegalExec(false);
    pc.stepped(false);
    tc->pcState(pc);

    // Save exception syndrome
    if ((nextMode() != MODE_IRQ) && (nextMode() != MODE_FIQ))
        setSyndrome(tc, getSyndromeReg64());
}

bool
ArmFault::vectorCatch(ThreadContext *tc, const StaticInstPtr &inst)
{
    SelfDebug *sd = ArmISA::ISA::getSelfDebug(tc);
    VectorCatch* vc = sd->getVectorCatch(tc);
    if (!vc->isVCMatch()) {
        Fault fault = sd->testVectorCatch(tc, 0x0, this);
        if (fault != NoFault)
            fault->invoke(tc, inst);
        return true;
    }
    return false;
}

ArmStaticInst *
ArmFault::instrAnnotate(const StaticInstPtr &inst)
{
    if (inst) {
        auto arm_inst = static_cast<ArmStaticInst *>(inst.get());
        arm_inst->annotateFault(this);
        return arm_inst;
    } else {
        return nullptr;
    }
}

Addr
Reset::getVector(ThreadContext *tc)
{
    Addr base;

    // Check for invalid modes
    CPSR M5_VAR_USED cpsr = tc->readMiscRegNoEffect(MISCREG_CPSR);
    assert(ArmSystem::haveSecurity(tc) || cpsr.mode != MODE_MON);
    assert(ArmSystem::haveVirtualization(tc) || cpsr.mode != MODE_HYP);

    // RVBAR is aliased (implemented as) MVBAR in gem5, since the two
    // are mutually exclusive; there is no need to check here for
    // which register to use since they hold the same value
    base = tc->readMiscReg(MISCREG_MVBAR);

    return base + offset(tc);
}

void
Reset::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    if (FullSystem) {
        tc->getCpuPtr()->clearInterrupts(tc->threadId());
        tc->clearArchRegs();
    }
    if (!ArmSystem::highestELIs64(tc)) {
        ArmFault::invoke(tc, inst);
        tc->setMiscReg(MISCREG_VMPIDR,
                       getMPIDR(dynamic_cast<ArmSystem*>(tc->getSystemPtr()), tc));

        // Unless we have SMC code to get us there, boot in HYP!
        if (ArmSystem::haveVirtualization(tc) &&
            !ArmSystem::haveSecurity(tc)) {
            CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);
            cpsr.mode = MODE_HYP;
            tc->setMiscReg(MISCREG_CPSR, cpsr);
        }
    } else {
        // Advance the PC to the IMPLEMENTATION DEFINED reset value
        PCState pc = ArmSystem::resetAddr(tc);
        pc.aarch64(true);
        pc.nextAArch64(true);
        tc->pcState(pc);
    }
}

void
UndefinedInstruction::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    if (FullSystem) {
        ArmFault::invoke(tc, inst);
        return;
    }

    // If the mnemonic isn't defined this has to be an unknown instruction.
    assert(unknown || mnemonic != NULL);
    auto arm_inst = static_cast<ArmStaticInst *>(inst.get());
    if (disabled) {
        panic("Attempted to execute disabled instruction "
                "'%s' (inst 0x%08x)", mnemonic, arm_inst->encoding());
    } else if (unknown) {
        panic("Attempted to execute unknown instruction (inst 0x%08x)",
              arm_inst->encoding());
    } else {
        panic("Attempted to execute unimplemented instruction "
                "'%s' (inst 0x%08x)", mnemonic, arm_inst->encoding());
    }
}

bool
UndefinedInstruction::routeToHyp(ThreadContext *tc) const
{
    HCR  hcr  = tc->readMiscRegNoEffect(MISCREG_HCR);
    return fromEL == EL2 ||
           (EL2Enabled(tc) && (fromEL == EL0) && hcr.tge);
}

uint32_t
UndefinedInstruction::iss() const
{

    // If UndefinedInstruction is routed to hypervisor, iss field is 0.
    if (hypRouted) {
        return 0;
    }

    if (overrideEc == EC_INVALID)
        return issRaw;

    uint32_t new_iss = 0;
    uint32_t op0, op1, op2, CRn, CRm, Rt, dir;

    dir = bits(machInst, 21, 21);
    op0 = bits(machInst, 20, 19);
    op1 = bits(machInst, 18, 16);
    CRn = bits(machInst, 15, 12);
    CRm = bits(machInst, 11, 8);
    op2 = bits(machInst, 7, 5);
    Rt = bits(machInst, 4, 0);

    new_iss = op0 << 20 | op2 << 17 | op1 << 14 | CRn << 10 |
            Rt << 5 | CRm << 1 | dir;

    return new_iss;
}

void
SupervisorCall::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    if (FullSystem) {
        ArmFault::invoke(tc, inst);
        return;
    }

    // As of now, there isn't a 32 bit thumb version of this instruction.
    assert(!machInst.bigThumb);
    tc->syscall();

    // Advance the PC since that won't happen automatically.
    PCState pc = tc->pcState();
    assert(inst);
    inst->advancePC(pc);
    tc->pcState(pc);
}

bool
SupervisorCall::routeToHyp(ThreadContext *tc) const
{
    HCR  hcr  = tc->readMiscRegNoEffect(MISCREG_HCR);
    return fromEL == EL2 ||
           (EL2Enabled(tc) && fromEL == EL0 && hcr.tge);
}

ExceptionClass
SupervisorCall::ec(ThreadContext *tc) const
{
    return (overrideEc != EC_INVALID) ? overrideEc :
        (from64 ? EC_SVC_64 : vals.ec);
}

uint32_t
SupervisorCall::iss() const
{
    // Even if we have a 24 bit imm from an arm32 instruction then we only use
    // the bottom 16 bits for the ISS value (it doesn't hurt for AArch64 SVC).
    return issRaw & 0xFFFF;
}

uint32_t
SecureMonitorCall::iss() const
{
    if (from64)
        return bits(machInst, 20, 5);
    return 0;
}

ExceptionClass
UndefinedInstruction::ec(ThreadContext *tc) const
{
    // If UndefinedInstruction is routed to hypervisor,
    // HSR.EC field is 0.
    if (hypRouted)
        return EC_UNKNOWN;
    else
        return (overrideEc != EC_INVALID) ? overrideEc : vals.ec;
}


HypervisorCall::HypervisorCall(ExtMachInst _machInst, uint32_t _imm) :
        ArmFaultVals<HypervisorCall>(_machInst, _imm)
{
    bStep = true;
}

bool
HypervisorCall::routeToMonitor(ThreadContext *tc) const
{
    return from64 && fromEL == EL3;
}

bool
HypervisorCall::routeToHyp(ThreadContext *tc) const
{
    return !from64 || fromEL != EL3;
}

ExceptionClass
HypervisorCall::ec(ThreadContext *tc) const
{
    return from64 ? EC_HVC_64 : vals.ec;
}

ExceptionClass
HypervisorTrap::ec(ThreadContext *tc) const
{
    return (overrideEc != EC_INVALID) ? overrideEc : vals.ec;
}

template<class T>
FaultOffset
ArmFaultVals<T>::offset(ThreadContext *tc)
{
    bool isHypTrap = false;

    // Normally we just use the exception vector from the table at the top if
    // this file, however if this exception has caused a transition to hype
    // mode, and its an exception type that would only do this if it has been
    // trapped then we use the hyp trap vector instead of the normal vector
    if (vals.hypTrappable) {
        CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);
        if (cpsr.mode == MODE_HYP) {
            CPSR spsr = tc->readMiscReg(MISCREG_SPSR_HYP);
            isHypTrap = spsr.mode != MODE_HYP;
        }
    }
    return isHypTrap ? 0x14 : vals.offset;
}

template<class T>
FaultOffset
ArmFaultVals<T>::offset64(ThreadContext *tc)
{
    if (toEL == fromEL) {
        if (opModeIsT(fromMode))
            return vals.currELTOffset;
        return vals.currELHOffset;
    } else {
        bool lower_32 = false;
        if (toEL == EL3) {
            if (EL2Enabled(tc))
                lower_32 = ELIs32(tc, EL2);
            else
                lower_32 = ELIs32(tc, EL1);
        } else if (ELIsInHost(tc, fromEL) && fromEL == EL0 && toEL == EL2) {
            lower_32 = ELIs32(tc, EL0);
        } else {
            lower_32 = ELIs32(tc, static_cast<ExceptionLevel>(toEL - 1));
        }

        if (lower_32)
            return vals.lowerEL32Offset;
        return vals.lowerEL64Offset;
    }
}

// void
// SupervisorCall::setSyndrome64(ThreadContext *tc, MiscRegIndex esr_idx)
// {
//     ESR esr = 0;
//     esr.ec = machInst.aarch64 ? SvcAArch64 : SvcAArch32;
//     esr.il = !machInst.thumb;
//     if (machInst.aarch64)
//         esr.imm16 = bits(machInst.instBits, 20, 5);
//     else if (machInst.thumb)
//         esr.imm16 = bits(machInst.instBits, 7, 0);
//     else
//         esr.imm16 = bits(machInst.instBits, 15, 0);
//     tc->setMiscReg(esr_idx, esr);
// }

void
SecureMonitorCall::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    if (FullSystem) {
        ArmFault::invoke(tc, inst);
        return;
    }
}

ExceptionClass
SecureMonitorCall::ec(ThreadContext *tc) const
{
    return (from64 ? EC_SMC_64 : vals.ec);
}

bool
SupervisorTrap::routeToHyp(ThreadContext *tc) const
{
    HCR  hcr  = tc->readMiscRegNoEffect(MISCREG_HCR_EL2);
    return EL2Enabled(tc) && currEL(tc) <= EL1 && hcr.tge;
}

uint32_t
SupervisorTrap::iss() const
{
    // If SupervisorTrap is routed to hypervisor, iss field is 0.
    if (hypRouted) {
        return 0;
    }
    return issRaw;
}

ExceptionClass
SupervisorTrap::ec(ThreadContext *tc) const
{
    if (hypRouted)
        return EC_UNKNOWN;
    else
        return (overrideEc != EC_INVALID) ? overrideEc : vals.ec;
}

ExceptionClass
SecureMonitorTrap::ec(ThreadContext *tc) const
{
    return (overrideEc != EC_INVALID) ? overrideEc :
        (from64 ? EC_SMC_64 : vals.ec);
}

template<class T>
void
AbortFault<T>::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    if (tranMethod == ArmFault::UnknownTran) {
        tranMethod = longDescFormatInUse(tc) ? ArmFault::LpaeTran
                                             : ArmFault::VmsaTran;

        if ((tranMethod == ArmFault::VmsaTran) && this->routeToMonitor(tc)) {
            // See ARM ARM B3-1416
            bool override_LPAE = false;
            TTBCR ttbcr_s = tc->readMiscReg(MISCREG_TTBCR_S);
            TTBCR M5_VAR_USED ttbcr_ns = tc->readMiscReg(MISCREG_TTBCR_NS);
            if (ttbcr_s.eae) {
                override_LPAE = true;
            } else {
                // Unimplemented code option, not seen in testing.  May need
                // extension according to the manual exceprt above.
                DPRINTF(Faults, "Warning: Incomplete translation method "
                        "override detected.\n");
            }
            if (override_LPAE)
                tranMethod = ArmFault::LpaeTran;
        }
    }

    if (source == ArmFault::AsynchronousExternalAbort) {
        tc->getCpuPtr()->clearInterrupt(tc->threadId(), INT_ABT, 0);
    }
    // Get effective fault source encoding
    CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);

    // source must be determined BEFORE invoking generic routines which will
    // try to set hsr etc. and are based upon source!
    ArmFaultVals<T>::invoke(tc, inst);

    if (!this->to64) {  // AArch32
        FSR  fsr  = getFsr(tc);
        if (cpsr.mode == MODE_HYP) {
            tc->setMiscReg(T::HFarIndex, faultAddr);
        } else if (stage2) {
            tc->setMiscReg(MISCREG_HPFAR, (faultAddr >> 8) & ~0xf);
            tc->setMiscReg(T::HFarIndex,  OVAddr);
        } else if (debug > ArmFault::NODEBUG) {
            DBGDS32 Rext =  tc->readMiscReg(MISCREG_DBGDSCRext);
            tc->setMiscReg(T::FarIndex, faultAddr);
            if (debug == ArmFault::BRKPOINT){
                Rext.moe = 0x1;
            } else if (debug == ArmFault::VECTORCATCH){
                Rext.moe = 0x5;
            } else if (debug > ArmFault::VECTORCATCH) {
                Rext.moe = 0xa;
                fsr.cm = (debug == ArmFault::WPOINT_CM)? 1 : 0;
            }

            tc->setMiscReg(T::FsrIndex, fsr);
            tc->setMiscReg(MISCREG_DBGDSCRext, Rext);

        } else {
            tc->setMiscReg(T::FsrIndex, fsr);
            tc->setMiscReg(T::FarIndex, faultAddr);
        }
        DPRINTF(Faults, "Abort Fault source=%#x fsr=%#x faultAddr=%#x "\
                "tranMethod=%#x\n", source, fsr, faultAddr, tranMethod);
    } else {  // AArch64
        // Set the FAR register.  Nothing else to do if we are in AArch64 state
        // because the syndrome register has already been set inside invoke64()
        if (stage2) {
            // stage 2 fault, set HPFAR_EL2 to the faulting IPA
            // and FAR_EL2 to the Original VA
            tc->setMiscReg(AbortFault<T>::getFaultAddrReg64(), OVAddr);
            tc->setMiscReg(MISCREG_HPFAR_EL2, bits(faultAddr, 47, 12) << 4);

            DPRINTF(Faults, "Abort Fault (Stage 2) VA: 0x%x IPA: 0x%x\n",
                    OVAddr, faultAddr);
        } else {
            tc->setMiscReg(AbortFault<T>::getFaultAddrReg64(), faultAddr);
        }
    }
}

template<class T>
void
AbortFault<T>::setSyndrome(ThreadContext *tc, MiscRegIndex syndrome_reg)
{
    srcEncoded = getFaultStatusCode(tc);
    if (srcEncoded == ArmFault::FaultSourceInvalid) {
        panic("Invalid fault source\n");
    }
    ArmFault::setSyndrome(tc, syndrome_reg);
}

template<class T>
uint8_t
AbortFault<T>::getFaultStatusCode(ThreadContext *tc) const
{

    panic_if(!this->faultUpdated,
             "Trying to use un-updated ArmFault internal variables\n");

    uint8_t fsc = 0;

    if (!this->to64) {
        // AArch32
        assert(tranMethod != ArmFault::UnknownTran);
        if (tranMethod == ArmFault::LpaeTran) {
            fsc = ArmFault::longDescFaultSources[source];
        } else {
            fsc = ArmFault::shortDescFaultSources[source];
        }
    } else {
        // AArch64
        fsc = ArmFault::aarch64FaultSources[source];
    }

    return fsc;
}

template<class T>
FSR
AbortFault<T>::getFsr(ThreadContext *tc) const
{
    FSR fsr = 0;

    auto fsc = getFaultStatusCode(tc);

    // AArch32
    assert(tranMethod != ArmFault::UnknownTran);
    if (tranMethod == ArmFault::LpaeTran) {
        fsr.status = fsc;
        fsr.lpae   = 1;
    } else {
        fsr.fsLow  = bits(fsc, 3, 0);
        fsr.fsHigh = bits(fsc, 4);
        fsr.domain = static_cast<uint8_t>(domain);
    }

    fsr.wnr = (write ? 1 : 0);
    fsr.ext = 0;

    return fsr;
}

template<class T>
bool
AbortFault<T>::abortDisable(ThreadContext *tc)
{
    if (ArmSystem::haveSecurity(tc)) {
        SCR scr = tc->readMiscRegNoEffect(MISCREG_SCR);
        return (!scr.ns || scr.aw);
    }
    return true;
}

template<class T>
void
AbortFault<T>::annotate(ArmFault::AnnotationIDs id, uint64_t val)
{
    switch (id)
    {
      case ArmFault::S1PTW:
        s1ptw = val;
        break;
      case ArmFault::OVA:
        OVAddr = val;
        break;

      // Just ignore unknown ID's
      default:
        break;
    }
}

template<class T>
uint32_t
AbortFault<T>::iss() const
{
    uint32_t val;

    val  = srcEncoded & 0x3F;
    val |= write << 6;
    val |= s1ptw << 7;
    return (val);
}

template<class T>
bool
AbortFault<T>::isMMUFault() const
{
    // NOTE: Not relying on LL information being aligned to lowest bits here
    return
         (source == ArmFault::AlignmentFault)     ||
        ((source >= ArmFault::TranslationLL) &&
         (source <  ArmFault::TranslationLL + 4)) ||
        ((source >= ArmFault::AccessFlagLL) &&
         (source <  ArmFault::AccessFlagLL + 4))  ||
        ((source >= ArmFault::DomainLL) &&
         (source <  ArmFault::DomainLL + 4))      ||
        ((source >= ArmFault::PermissionLL) &&
         (source <  ArmFault::PermissionLL + 4));
}

template<class T>
bool
AbortFault<T>::getFaultVAddr(Addr &va) const
{
    va = (stage2 ?  OVAddr : faultAddr);
    return true;
}

ExceptionClass
PrefetchAbort::ec(ThreadContext *tc) const
{
    if (to64) {
        // AArch64
        if (toEL == fromEL)
            return EC_PREFETCH_ABORT_CURR_EL;
        else
            return EC_PREFETCH_ABORT_LOWER_EL;
    } else {
        // AArch32
        // Abort faults have different EC codes depending on whether
        // the fault originated within HYP mode, or not. So override
        // the method and add the extra adjustment of the EC value.

        ExceptionClass ec = ArmFaultVals<PrefetchAbort>::vals.ec;

        CPSR spsr = tc->readMiscReg(MISCREG_SPSR_HYP);
        if (spsr.mode == MODE_HYP) {
            ec = ((ExceptionClass) (((uint32_t) ec) + 1));
        }
        return ec;
    }
}

bool
PrefetchAbort::routeToMonitor(ThreadContext *tc) const
{
    SCR scr = 0;
    if (from64)
        scr = tc->readMiscRegNoEffect(MISCREG_SCR_EL3);
    else
        scr = tc->readMiscRegNoEffect(MISCREG_SCR);

    return scr.ea && !isMMUFault();
}

bool
PrefetchAbort::routeToHyp(ThreadContext *tc) const
{
    bool toHyp;

    HCR  hcr  = tc->readMiscRegNoEffect(MISCREG_HCR);
    HDCR hdcr = tc->readMiscRegNoEffect(MISCREG_HDCR);

    toHyp = fromEL == EL2;
    toHyp |=  ArmSystem::haveEL(tc, EL2) && !isSecure(tc) &&
        currEL(tc) <= EL1 && (hcr.tge || stage2 ||
                              (source == DebugEvent && hdcr.tde));
     return toHyp;
}

ExceptionClass
DataAbort::ec(ThreadContext *tc) const
{
    if (to64) {
        // AArch64
        if (source == ArmFault::AsynchronousExternalAbort) {
            panic("Asynchronous External Abort should be handled with "
                    "SystemErrors (SErrors)!");
        }
        if (toEL == fromEL)
            return EC_DATA_ABORT_CURR_EL;
        else
            return EC_DATA_ABORT_LOWER_EL;
    } else {
        // AArch32
        // Abort faults have different EC codes depending on whether
        // the fault originated within HYP mode, or not. So override
        // the method and add the extra adjustment of the EC value.

        ExceptionClass ec = ArmFaultVals<DataAbort>::vals.ec;

        CPSR spsr = tc->readMiscReg(MISCREG_SPSR_HYP);
        if (spsr.mode == MODE_HYP) {
            ec = ((ExceptionClass) (((uint32_t) ec) + 1));
        }
        return ec;
    }
}

bool
DataAbort::routeToMonitor(ThreadContext *tc) const
{
    SCR scr = 0;
    if (from64)
        scr = tc->readMiscRegNoEffect(MISCREG_SCR_EL3);
    else
        scr = tc->readMiscRegNoEffect(MISCREG_SCR);

    return scr.ea && !isMMUFault();
}

bool
DataAbort::routeToHyp(ThreadContext *tc) const
{
    bool toHyp;

    HCR  hcr  = tc->readMiscRegNoEffect(MISCREG_HCR);
    HDCR hdcr = tc->readMiscRegNoEffect(MISCREG_HDCR);

    bool amo = hcr.amo;
    if (hcr.tge == 1)
        amo =  (!HaveVirtHostExt(tc) || hcr.e2h == 0);

    // if in Hyp mode then stay in Hyp mode
    toHyp = fromEL == EL2 ||
            (EL2Enabled(tc) && fromEL <= EL1
                && (hcr.tge || stage2 ||
                    ((source == AsynchronousExternalAbort) && amo) ||
                    ((fromEL == EL0) && hcr.tge &&
                     ((source == AlignmentFault) ||
                      (source == SynchronousExternalAbort))) ||
                    ((source == DebugEvent) && (hdcr.tde || hcr.tge))));
    return toHyp;
}

uint32_t
DataAbort::iss() const
{
    uint32_t val;

    // Add on the data abort specific fields to the generic abort ISS value
    val  = AbortFault<DataAbort>::iss();

    val |= cm << 8;

    // ISS is valid if not caused by a stage 1 page table walk, and when taken
    // to AArch64 only when directed to EL2
    if (!s1ptw && stage2 && (!to64 || toEL == EL2)) {
        val |= isv << 24;
        if (isv) {
            val |= sas << 22;
            val |= sse << 21;
            val |= srt << 16;
            // AArch64 only. These assignments are safe on AArch32 as well
            // because these vars are initialized to false
            val |= sf << 15;
            val |= ar << 14;
        }
    }
    return (val);
}

void
DataAbort::annotate(AnnotationIDs id, uint64_t val)
{
    AbortFault<DataAbort>::annotate(id, val);
    switch (id)
    {
      case SAS:
        isv = true;
        sas = val;
        break;
      case SSE:
        isv = true;
        sse = val;
        break;
      case SRT:
        isv = true;
        srt = val;
        break;
      case SF:
        isv = true;
        sf  = val;
        break;
      case AR:
        isv = true;
        ar  = val;
        break;
      case CM:
        cm  = val;
        break;
      case OFA:
        faultAddr  = val;
        break;
      // Just ignore unknown ID's
      default:
        break;
    }
}

void
VirtualDataAbort::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    AbortFault<VirtualDataAbort>::invoke(tc, inst);
    HCR hcr = tc->readMiscRegNoEffect(MISCREG_HCR);
    hcr.va = 0;
    tc->setMiscRegNoEffect(MISCREG_HCR, hcr);
}

bool
Interrupt::routeToMonitor(ThreadContext *tc) const
{
    assert(ArmSystem::haveSecurity(tc));
    SCR scr = 0;
    if (from64)
        scr = tc->readMiscRegNoEffect(MISCREG_SCR_EL3);
    else
        scr = tc->readMiscRegNoEffect(MISCREG_SCR);
    return scr.irq;
}

bool
Interrupt::routeToHyp(ThreadContext *tc) const
{
    HCR  hcr  = tc->readMiscRegNoEffect(MISCREG_HCR);
    return fromEL == EL2 ||
           (EL2Enabled(tc) && fromEL <= EL1 && (hcr.tge || hcr.imo));
}

bool
Interrupt::abortDisable(ThreadContext *tc)
{
    if (ArmSystem::haveSecurity(tc)) {
        SCR scr = tc->readMiscRegNoEffect(MISCREG_SCR);
        return (!scr.ns || scr.aw);
    }
    return true;
}

VirtualInterrupt::VirtualInterrupt()
{}

bool
FastInterrupt::routeToMonitor(ThreadContext *tc) const
{
    assert(ArmSystem::haveSecurity(tc));
    SCR scr = 0;
    if (from64)
        scr = tc->readMiscRegNoEffect(MISCREG_SCR_EL3);
    else
        scr = tc->readMiscRegNoEffect(MISCREG_SCR);
    return scr.fiq;
}

bool
FastInterrupt::routeToHyp(ThreadContext *tc) const
{
    HCR  hcr  = tc->readMiscRegNoEffect(MISCREG_HCR);
    return fromEL == EL2 ||
           (EL2Enabled(tc) && fromEL <= EL1 && (hcr.tge || hcr.fmo));
}

bool
FastInterrupt::abortDisable(ThreadContext *tc)
{
    if (ArmSystem::haveSecurity(tc)) {
        SCR scr = tc->readMiscRegNoEffect(MISCREG_SCR);
        return (!scr.ns || scr.aw);
    }
    return true;
}

bool
FastInterrupt::fiqDisable(ThreadContext *tc)
{
    if (ArmSystem::haveVirtualization(tc)) {
        return true;
    } else if (ArmSystem::haveSecurity(tc)) {
        SCR scr = tc->readMiscRegNoEffect(MISCREG_SCR);
        return (!scr.ns || scr.fw);
    }
    return true;
}

VirtualFastInterrupt::VirtualFastInterrupt()
{}

void
PCAlignmentFault::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    ArmFaultVals<PCAlignmentFault>::invoke(tc, inst);
    assert(from64);
    // Set the FAR
    tc->setMiscReg(getFaultAddrReg64(), faultPC);
}

bool
PCAlignmentFault::routeToHyp(ThreadContext *tc) const
{
    HCR  hcr  = tc->readMiscRegNoEffect(MISCREG_HCR_EL2);
    return fromEL == EL2 || (EL2Enabled(tc) && fromEL <= EL1 && hcr.tge);
}

SPAlignmentFault::SPAlignmentFault()
{}

bool
SPAlignmentFault::routeToHyp(ThreadContext *tc) const
{
    assert(from64);
    HCR hcr  = tc->readMiscRegNoEffect(MISCREG_HCR_EL2);
    return EL2Enabled(tc) && currEL(tc) <= EL1 && hcr.tge == 1;
}

SystemError::SystemError()
{}

void
SystemError::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    tc->getCpuPtr()->clearInterrupt(tc->threadId(), INT_ABT, 0);
    ArmFault::invoke(tc, inst);
}

bool
SystemError::routeToMonitor(ThreadContext *tc) const
{
    assert(ArmSystem::haveSecurity(tc));
    assert(from64);
    SCR scr = tc->readMiscRegNoEffect(MISCREG_SCR_EL3);
    return scr.ea || fromEL == EL3;
}

bool
SystemError::routeToHyp(ThreadContext *tc) const
{
    assert(from64);

    HCR hcr  = tc->readMiscRegNoEffect(MISCREG_HCR_EL2);

    return fromEL == EL2 ||
           (EL2Enabled(tc) && fromEL <= EL1 && (hcr.tge || hcr.amo));
}


SoftwareBreakpoint::SoftwareBreakpoint(ExtMachInst _mach_inst, uint32_t _iss)
    : ArmFaultVals<SoftwareBreakpoint>(_mach_inst, _iss)
{}

bool
SoftwareBreakpoint::routeToHyp(ThreadContext *tc) const
{
    const HCR hcr  = tc->readMiscRegNoEffect(MISCREG_HCR_EL2);
    const HDCR mdcr  = tc->readMiscRegNoEffect(MISCREG_MDCR_EL2);

    return fromEL == EL2 ||
           (EL2Enabled(tc) && fromEL <= EL1 && (hcr.tge || mdcr.tde));
}

ExceptionClass
SoftwareBreakpoint::ec(ThreadContext *tc) const
{
    return from64 ? EC_SOFTWARE_BREAKPOINT_64 : vals.ec;
}

HardwareBreakpoint::HardwareBreakpoint(Addr _vaddr,  uint32_t _iss)
    : ArmFaultVals<HardwareBreakpoint>(0x0, _iss), vAddr(_vaddr)
{}

bool
HardwareBreakpoint::routeToHyp(ThreadContext *tc) const
{
    const HCR hcr  = tc->readMiscRegNoEffect(MISCREG_HCR_EL2);
    const HDCR mdcr  = tc->readMiscRegNoEffect(MISCREG_MDCR_EL2);

    return EL2Enabled(tc) && fromEL <= EL1 && (hcr.tge || mdcr.tde);
}

ExceptionClass
HardwareBreakpoint::ec(ThreadContext *tc) const
{
        // AArch64
    if (toEL == fromEL)
        return EC_HW_BREAKPOINT_CURR_EL;
    else
        return EC_HW_BREAKPOINT_LOWER_EL;
}

void
HardwareBreakpoint::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{

    ArmFaultVals<HardwareBreakpoint>::invoke(tc, inst);
    MiscRegIndex elr_idx;
    switch (toEL) {
      case EL1:
        elr_idx = MISCREG_ELR_EL1;
        break;
      case EL2:
        assert(ArmSystem::haveVirtualization(tc));
        elr_idx = MISCREG_ELR_EL2;
        break;
      case EL3:
        assert(ArmSystem::haveSecurity(tc));
        elr_idx = MISCREG_ELR_EL3;
        break;
      default:
        panic("Invalid target exception level");
        break;
    }

    tc->setMiscReg(elr_idx, vAddr);

}

Watchpoint::Watchpoint(ExtMachInst _mach_inst, Addr _vaddr,
                       bool _write, bool _cm)
    : ArmFaultVals<Watchpoint>(_mach_inst), vAddr(_vaddr),
      write(_write), cm(_cm)
{}

uint32_t
Watchpoint::iss() const
{
    uint32_t iss = 0x0022;
// NV
//    if (toEL == EL2)
//        iss |= 0x02000;
    if (cm)
        iss |= 0x00100;
    if (write)
        iss |= 0x00040;
    return iss;
}

void
Watchpoint::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    ArmFaultVals<Watchpoint>::invoke(tc, inst);
    // Set the FAR
    tc->setMiscReg(getFaultAddrReg64(), vAddr);

}

bool
Watchpoint::routeToHyp(ThreadContext *tc) const
{
    const HCR hcr  = tc->readMiscRegNoEffect(MISCREG_HCR_EL2);
    const HDCR mdcr  = tc->readMiscRegNoEffect(MISCREG_MDCR_EL2);

    return fromEL == EL2 ||
           (EL2Enabled(tc) && fromEL <= EL1 && (hcr.tge || mdcr.tde));
}

void
Watchpoint::annotate(AnnotationIDs id, uint64_t val)
{
    ArmFaultVals<Watchpoint>::annotate(id, val);
    switch (id)
    {
      case OFA:
        vAddr  = val;
        break;
      // Just ignore unknown ID's
      default:
        break;
    }
}

ExceptionClass
Watchpoint::ec(ThreadContext *tc) const
{
        // AArch64
        if (toEL == fromEL)
            return EC_WATCHPOINT_CURR_EL;
        else
            return EC_WATCHPOINT_LOWER_EL;
}

SoftwareStepFault::SoftwareStepFault(ExtMachInst _mach_inst, bool is_ldx,
                                     bool _stepped)
    : ArmFaultVals<SoftwareStepFault>(_mach_inst), isldx(is_ldx),
                                      stepped(_stepped)
{
    bStep = true;
}

bool
SoftwareStepFault::routeToHyp(ThreadContext *tc) const
{
    const HCR hcr  = tc->readMiscRegNoEffect(MISCREG_HCR_EL2);
    const HDCR mdcr  = tc->readMiscRegNoEffect(MISCREG_MDCR_EL2);

    return fromEL == EL2 ||
           (EL2Enabled(tc) && fromEL <= EL1 && (hcr.tge || mdcr.tde));
}

ExceptionClass
SoftwareStepFault::ec(ThreadContext *tc) const
{
    // AArch64
    if (toEL == fromEL)
        return EC_SOFTWARE_STEP_CURR_EL;
    else
        return EC_SOFTWARE_STEP_LOWER_EL;
}

uint32_t
SoftwareStepFault::iss() const
{
    uint32_t iss= 0x0022;
    if (stepped) {
        iss |= 0x1000000;
    }

    if (isldx) {
        iss |= 0x40;
    }

    return iss;

}

void
ArmSev::invoke(ThreadContext *tc, const StaticInstPtr &inst) {
    DPRINTF(Faults, "Invoking ArmSev Fault\n");
    if (!FullSystem)
        return;

    // Set sev_mailbox to 1, clear the pending interrupt from remote
    // SEV execution and let pipeline continue as pcState is still
    // valid.
    tc->setMiscReg(MISCREG_SEV_MAILBOX, 1);
    tc->getCpuPtr()->clearInterrupt(tc->threadId(), INT_SEV, 0);
}

// Instantiate all the templates to make the linker happy
template class ArmFaultVals<Reset>;
template class ArmFaultVals<UndefinedInstruction>;
template class ArmFaultVals<SupervisorCall>;
template class ArmFaultVals<SecureMonitorCall>;
template class ArmFaultVals<HypervisorCall>;
template class ArmFaultVals<PrefetchAbort>;
template class ArmFaultVals<DataAbort>;
template class ArmFaultVals<VirtualDataAbort>;
template class ArmFaultVals<HypervisorTrap>;
template class ArmFaultVals<Interrupt>;
template class ArmFaultVals<VirtualInterrupt>;
template class ArmFaultVals<FastInterrupt>;
template class ArmFaultVals<VirtualFastInterrupt>;
template class ArmFaultVals<SupervisorTrap>;
template class ArmFaultVals<SecureMonitorTrap>;
template class ArmFaultVals<PCAlignmentFault>;
template class ArmFaultVals<SPAlignmentFault>;
template class ArmFaultVals<SystemError>;
template class ArmFaultVals<SoftwareBreakpoint>;
template class ArmFaultVals<HardwareBreakpoint>;
template class ArmFaultVals<Watchpoint>;
template class ArmFaultVals<SoftwareStepFault>;
template class ArmFaultVals<ArmSev>;
template class AbortFault<PrefetchAbort>;
template class AbortFault<DataAbort>;
template class AbortFault<VirtualDataAbort>;


IllegalInstSetStateFault::IllegalInstSetStateFault()
{}

bool
IllegalInstSetStateFault::routeToHyp(ThreadContext *tc) const
{
    const HCR hcr  = tc->readMiscRegNoEffect(MISCREG_HCR_EL2);
    return EL2Enabled(tc) && fromEL == EL0 && hcr.tge;
}

bool
getFaultVAddr(Fault fault, Addr &va)
{
    auto arm_fault = dynamic_cast<ArmFault *>(fault.get());

    if (arm_fault) {
        return arm_fault->getFaultVAddr(va);
    } else {
        auto pgt_fault = dynamic_cast<GenericPageTableFault *>(fault.get());
        if (pgt_fault) {
            va = pgt_fault->getFaultVAddr();
            return true;
        }

        auto align_fault = dynamic_cast<GenericAlignmentFault *>(fault.get());
        if (align_fault) {
            va = align_fault->getFaultVAddr();
            return true;
        }

        // Return false since it's not an address triggered exception
        return false;
    }
}

} // namespace ArmISA
