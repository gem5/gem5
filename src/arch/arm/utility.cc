/*
 * Copyright (c) 2009-2014, 2016-2020, 2022 Arm Limited
 * All rights reserved.
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

#include "arch/arm/utility.hh"

#include <memory>

#include "arch/arm/faults.hh"
#include "arch/arm/interrupts.hh"
#include "arch/arm/isa.hh"
#include "arch/arm/mmu.hh"
#include "arch/arm/page_size.hh"
#include "arch/arm/regs/cc.hh"
#include "arch/arm/regs/int.hh"
#include "arch/arm/regs/vec.hh"
#include "arch/arm/system.hh"
#include "base/compiler.hh"
#include "cpu/base.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/thread_context.hh"
#include "mem/port_proxy.hh"
#include "sim/full_system.hh"

namespace gem5
{

namespace ArmISA
{

void
sendEvent(ThreadContext *tc)
{
    if (tc->readMiscReg(MISCREG_SEV_MAILBOX) == 0) {
        // Post Interrupt and wake cpu if needed
        tc->getCpuPtr()->postInterrupt(tc->threadId(), INT_SEV, 0);
    }
}

bool
isSecure(ThreadContext *tc)
{
    CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);
    if (ArmSystem::haveEL(tc, EL3) && !cpsr.width && currEL(tc) == EL3)
        return true;
    if (ArmSystem::haveEL(tc, EL3) && cpsr.width  && cpsr.mode == MODE_MON)
        return true;
    else
        return isSecureBelowEL3(tc);
}

bool
isSecureBelowEL3(ThreadContext *tc)
{
    return ArmSystem::haveEL(tc, EL3) &&
        static_cast<SCR>(tc->readMiscRegNoEffect(MISCREG_SCR_EL3)).ns == 0;
}

ExceptionLevel
debugTargetFrom(ThreadContext *tc, bool secure)
{
    bool route_to_el2;
    if (ArmSystem::haveEL(tc, EL2) &&
        (!secure || HaveExt(tc, ArmExtension::FEAT_SEL2))) {
        const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
        const HDCR mdcr = tc->readMiscRegNoEffect(MISCREG_MDCR_EL2);
        route_to_el2 = (mdcr.tde == 1 || hcr.tge == 1);
    } else {
        route_to_el2 = false;
    }
    ExceptionLevel target;
    if (route_to_el2) {
        target = EL2;
    } else if (ArmSystem::haveEL(tc, EL3) && !ArmSystem::highestELIs64(tc)
              && secure) {
        target = EL3;
    } else {
        target = EL1;
    }
    return target;
}

bool
inAArch64(ThreadContext *tc)
{
    CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);
    return opModeIs64((OperatingMode) (uint8_t) cpsr.mode);
}

ExceptionLevel
currEL(const ThreadContext *tc)
{
    CPSR cpsr = tc->readMiscRegNoEffect(MISCREG_CPSR);
    return opModeToEL((OperatingMode)(uint8_t)cpsr.mode);
}

bool
longDescFormatInUse(ThreadContext *tc)
{
    TTBCR ttbcr = tc->readMiscReg(MISCREG_TTBCR);
    return ArmSystem::has(ArmExtension::LPAE, tc) && ttbcr.eae;
}

RegVal
readMPIDR(ArmSystem *arm_sys, ThreadContext *tc)
{
    const ExceptionLevel current_el = currEL(tc);

    const bool is_secure = isSecureBelowEL3(tc);

    switch (current_el) {
      case EL0:
        // Note: in MsrMrs instruction we read the register value before
        // checking access permissions. This means that EL0 entry must
        // be part of the table even if MPIDR is not accessible in user
        // mode.
        warn_once("Trying to read MPIDR at EL0\n");
        [[fallthrough]];
      case EL1:
        if (ArmSystem::haveEL(tc, EL2) && !is_secure)
            return tc->readMiscReg(MISCREG_VMPIDR_EL2);
        else
            return getMPIDR(arm_sys, tc);
      case EL2:
      case EL3:
        return getMPIDR(arm_sys, tc);
      default:
        panic("Invalid EL for reading MPIDR register\n");
    }
}

RegVal
getMPIDR(ArmSystem *arm_sys, ThreadContext *tc)
{
    // Multiprocessor Affinity Register MPIDR from Cortex(tm)-A15 Technical
    // Reference Manual
    //
    // bit   31 - Multi-processor extensions available
    // bit   30 - Uni-processor system
    // bit   24 - Multi-threaded cores
    // bit 11-8 - Cluster ID
    // bit  1-0 - CPU ID
    //
    // We deliberately extend both the Cluster ID and CPU ID fields to allow
    // for simulation of larger systems
    assert((0 <= tc->cpuId()) && (tc->cpuId() < 256));
    assert(tc->socketId() < 65536);

    RegVal mpidr = 0x80000000;

    if (!arm_sys->multiProc)
        replaceBits(mpidr, 30, 1);

    if (arm_sys->multiThread)
        replaceBits(mpidr, 24, 1);

    // Get Affinity numbers
    mpidr |= getAffinity(arm_sys, tc);
    return mpidr;
}

static RegVal
getAff2(ArmSystem *arm_sys, ThreadContext *tc)
{
    return arm_sys->multiThread ? tc->socketId() : 0;
}

static RegVal
getAff1(ArmSystem *arm_sys, ThreadContext *tc)
{
    return arm_sys->multiThread ? tc->cpuId() : tc->socketId();
}

static RegVal
getAff0(ArmSystem *arm_sys, ThreadContext *tc)
{
    return arm_sys->multiThread ? tc->threadId() : tc->cpuId();
}

Affinity
getAffinity(ArmSystem *arm_sys, ThreadContext *tc)
{
    Affinity aff = 0;
    aff.aff0 = getAff0(arm_sys, tc);
    aff.aff1 = getAff1(arm_sys, tc);
    aff.aff2 = getAff2(arm_sys, tc);
    return aff;
}

bool
HaveExt(ThreadContext* tc, ArmExtension ext)
{
    auto *isa = static_cast<ArmISA::ISA *>(tc->getIsaPtr());
    return isa->getRelease()->has(ext);
}

ExceptionLevel
s1TranslationRegime(ThreadContext* tc, ExceptionLevel el)
{
    if (el != EL0)
        return el;
    else if (ArmSystem::haveEL(tc, EL3) && ELIs32(tc, EL3) &&
             static_cast<SCR>(
                tc->readMiscRegNoEffect(MISCREG_SCR_EL3)).ns == 0)
        return EL3;
    else if (HaveExt(tc, ArmExtension::FEAT_VHE) && ELIsInHost(tc, el))
        return EL2;
    else
        return EL1;
}

bool
IsSecureEL2Enabled(ThreadContext *tc)
{
    if (ArmSystem::haveEL(tc, EL2) && HaveExt(tc, ArmExtension::FEAT_SEL2) &&
        !ELIs32(tc, EL2)) {
        if (ArmSystem::haveEL(tc, EL3))
            return !ELIs32(tc, EL3) && static_cast<SCR>(
                tc->readMiscRegNoEffect(MISCREG_SCR_EL3)).eel2;
        else
            return isSecure(tc);
    }
    return false;
}

bool
EL2Enabled(ThreadContext *tc)
{
    return ArmSystem::haveEL(tc, EL2) &&
           (!ArmSystem::haveEL(tc, EL3) || static_cast<SCR>(
                tc->readMiscRegNoEffect(MISCREG_SCR_EL3)).ns ||
            IsSecureEL2Enabled(tc));
}

bool
ELIs64(ThreadContext *tc, ExceptionLevel el)
{
    return !ELIs32(tc, el);
}

bool
ELIs32(ThreadContext *tc, ExceptionLevel el)
{
    auto [known, aarch32] = ELUsingAArch32K(tc, el);
    panic_if(!known, "EL state is UNKNOWN");
    return aarch32;
}

bool
ELIsInHost(ThreadContext *tc, ExceptionLevel el)
{
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    return (ArmSystem::haveEL(tc, EL2) &&
            (IsSecureEL2Enabled(tc) || !isSecureBelowEL3(tc)) &&
            HaveExt(tc, ArmExtension::FEAT_VHE) &&
            !ELIs32(tc, EL2) && hcr.e2h == 1 &&
            (el == EL2 || (el == EL0 && hcr.tge == 1)));
}

std::pair<bool, bool>
ELUsingAArch32K(ThreadContext *tc, ExceptionLevel el)
{
    bool secure = isSecureBelowEL3(tc);
    return ELStateUsingAArch32K(tc, el, secure);
}

bool
haveAArch32EL(ThreadContext *tc, ExceptionLevel el)
{
    if (!ArmSystem::haveEL(tc, el))
        return false;
    else if (!ArmSystem::highestELIs64(tc))
        return true;
    else if (ArmSystem::highestEL(tc) == el)
        return false;
    else if (el == EL0)
        return true;
    return true;
}

std::pair<bool, bool>
ELStateUsingAArch32K(ThreadContext *tc, ExceptionLevel el, bool secure)
{
    // Return true if the specified EL is in aarch32 state.
    const bool have_el3 = ArmSystem::haveEL(tc, EL3);
    const bool have_el2 = ArmSystem::haveEL(tc, EL2);

    panic_if(el == EL2 && !have_el2, "Asking for EL2 when it doesn't exist");
    panic_if(el == EL3 && !have_el3, "Asking for EL3 when it doesn't exist");

    bool known, aarch32;
    known = aarch32 = false;
    if (!haveAArch32EL(tc, el)) {
        // Target EL is the highest one in a system where
        // the highest is using AArch64.
        known = true; aarch32 = false;
    } else if (secure && el == EL2) {
        known = true; aarch32 = false;
    } else if (!ArmSystem::highestELIs64(tc)) {
        // All ELs are using AArch32:
        known = true; aarch32 = true;
    } else if (ArmSystem::highestEL(tc) == el) {
        known = true; aarch32 = false;
    } else {
        SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
        bool aarch32_below_el3 = have_el3 && scr.rw == 0 &&
            (!secure || !HaveExt(tc, ArmExtension::FEAT_SEL2) || !scr.eel2);

        HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
        bool sec_el2 = HaveExt(tc, ArmExtension::FEAT_SEL2) && scr.eel2;
        bool aarch32_at_el1 = (aarch32_below_el3 ||
                               (have_el2 && (sec_el2 || !secure) &&
                                hcr.rw == 0 &&
                                !(hcr.e2h && hcr.tge &&
                                 HaveExt(tc, ArmExtension::FEAT_VHE))));

        // Only know if EL0 using AArch32 from PSTATE
        if (el == EL0 && !aarch32_at_el1) {
            // EL0 controlled by PSTATE
            CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);
            known = (currEL(tc) == EL0);
            aarch32 = (cpsr.width == 1);
        } else {
            known = true;
            aarch32 = (aarch32_below_el3 && el != EL3) ||
                (aarch32_at_el1 && (el == EL0 || el == EL1) );
        }
    }

    return std::make_pair(known, aarch32);
}

bool
ELStateUsingAArch32(ThreadContext *tc, ExceptionLevel el, bool secure)
{
    auto [known, aarch32] = ELStateUsingAArch32K(tc, el, secure);
    panic_if(!known, "EL state is UNKNOWN");
    return aarch32;
}

bool
isBigEndian64(const ThreadContext *tc)
{
    switch (currEL(tc)) {
      case EL3:
        return ((SCTLR) tc->readMiscRegNoEffect(MISCREG_SCTLR_EL3)).ee;
      case EL2:
        return ((SCTLR) tc->readMiscRegNoEffect(MISCREG_SCTLR_EL2)).ee;
      case EL1:
        return ((SCTLR) tc->readMiscRegNoEffect(MISCREG_SCTLR_EL1)).ee;
      case EL0:
        return ((SCTLR) tc->readMiscRegNoEffect(MISCREG_SCTLR_EL1)).e0e;
      default:
        panic("Invalid exception level");
        break;
    }
}

bool
badMode32(ThreadContext *tc, OperatingMode mode)
{
    return unknownMode32(mode) || !ArmSystem::haveEL(tc, opModeToEL(mode));
}

bool
badMode(ThreadContext *tc, OperatingMode mode)
{
    return unknownMode(mode) || !ArmSystem::haveEL(tc, opModeToEL(mode));
}

int
computeAddrTop(ThreadContext *tc, bool selbit, bool is_instr,
               TCR tcr, ExceptionLevel el)
{
    bool tbi = false;
    bool tbid = false;
    ExceptionLevel regime = s1TranslationRegime(tc, el);
    if (ELIs32(tc, regime)) {
        return 31;
    } else {
        switch (regime) {
          case EL1:
          {
            //TCR tcr = tc->readMiscReg(MISCREG_TCR_EL1);
            tbi = selbit? tcr.tbi1 : tcr.tbi0;
            tbid = selbit? tcr.tbid1 : tcr.tbid0;
            break;
          }
          case EL2:
          {
            TCR tcr = tc->readMiscReg(MISCREG_TCR_EL2);
            if (HaveExt(tc, ArmExtension::FEAT_VHE) && ELIsInHost(tc, el)) {
                tbi = selbit? tcr.tbi1 : tcr.tbi0;
                tbid = selbit? tcr.tbid1 : tcr.tbid0;
            } else {
                tbi = tcr.tbi;
                tbid = tcr.tbid;
            }
            break;
          }
          case EL3:
          {
            TCR tcr = tc->readMiscReg(MISCREG_TCR_EL3);
            tbi = tcr.tbi;
            tbid = tcr.tbid;
            break;
          }
          default:
            break;
        }

    }
    int res = (tbi && (!tbid || !is_instr))? 55: 63;
    return res;
}

Addr
maskTaggedAddr(Addr addr, ThreadContext *tc, ExceptionLevel el,
               int topbit)
{
    if (topbit == 63) {
        return addr;
    } else if (bits(addr,55) && (el <= EL1 || ELIsInHost(tc, el))) {
        uint64_t mask = ((uint64_t)0x1 << topbit) -1;
        addr = addr | ~mask;
    } else {
        addr = bits(addr, topbit, 0);
    }
    return addr;  // Nothing to do if this is not a tagged address
}

Addr
purifyTaggedAddr(Addr addr, ThreadContext *tc, ExceptionLevel el,
                 TCR tcr, bool is_instr)
{
    bool selbit = bits(addr, 55);
    int topbit = computeAddrTop(tc, selbit, is_instr, tcr, el);

    return maskTaggedAddr(addr, tc, el, topbit);
}

Addr
purifyTaggedAddr(Addr addr, ThreadContext *tc, ExceptionLevel el,
                 bool is_instr)
{

    TCR tcr = tc->readMiscReg(MISCREG_TCR_EL1);
    return purifyTaggedAddr(addr, tc, el, tcr, is_instr);
}

Addr
truncPage(Addr addr)
{
    return addr & ~(PageBytes - 1);
}

Addr
roundPage(Addr addr)
{
    return (addr + PageBytes - 1) & ~(PageBytes - 1);
}

Fault
mcrMrc15Trap(const MiscRegIndex misc_reg, ExtMachInst mach_inst,
             ThreadContext *tc, uint32_t imm)
{
    ExceptionClass ec = ExceptionClass::TRAPPED_CP15_MCR_MRC;
    if (mcrMrc15TrapToHyp(misc_reg, tc, imm, &ec))
        return std::make_shared<HypervisorTrap>(mach_inst, imm, ec);
    return AArch64AArch32SystemAccessTrap(misc_reg, mach_inst, tc, imm, ec);
}

bool
mcrMrc15TrapToHyp(const MiscRegIndex misc_reg, ThreadContext *tc, uint32_t iss,
                  ExceptionClass *ec)
{
    bool is_read;
    uint32_t crm;
    RegIndex rt;
    uint32_t crn;
    uint32_t opc1;
    uint32_t opc2;
    bool trap_to_hyp = false;

    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    const HDCR hdcr = tc->readMiscReg(MISCREG_HDCR);
    const HSTR hstr = tc->readMiscReg(MISCREG_HSTR);
    const HCPTR hcptr = tc->readMiscReg(MISCREG_HCPTR);

    if (EL2Enabled(tc) && (currEL(tc) < EL2)) {
        mcrMrcIssExtract(iss, is_read, crm, rt, crn, opc1, opc2);
        trap_to_hyp  = ((uint32_t) hstr) & (1 << crn);
        trap_to_hyp |= hdcr.tpm  && (crn == 9) && (crm >= 12);
        trap_to_hyp |= hcr.tidcp && (
            ((crn ==  9) && ((crm <= 2) || ((crm >= 5) && (crm <= 8)))) ||
            ((crn == 10) && ((crm <= 1) || (crm == 4) || (crm == 8))) ||
            ((crn == 11) && ((crm <= 8) || (crm == 15))));

        if (!trap_to_hyp) {
            switch (unflattenMiscReg(misc_reg)) {
              case MISCREG_CPACR:
                trap_to_hyp = hcptr.tcpac;
                break;
              case MISCREG_REVIDR:
              case MISCREG_TCMTR:
              case MISCREG_TLBTR:
              case MISCREG_AIDR:
                trap_to_hyp = hcr.tid1;
                break;
              case MISCREG_CTR:
              case MISCREG_CCSIDR:
              case MISCREG_CLIDR:
              case MISCREG_CSSELR:
                trap_to_hyp = hcr.tid2;
                break;
              case MISCREG_ID_PFR0:
              case MISCREG_ID_PFR1:
              case MISCREG_ID_DFR0:
              case MISCREG_ID_AFR0:
              case MISCREG_ID_MMFR0:
              case MISCREG_ID_MMFR1:
              case MISCREG_ID_MMFR2:
              case MISCREG_ID_MMFR3:
              case MISCREG_ID_MMFR4:
              case MISCREG_ID_ISAR0:
              case MISCREG_ID_ISAR1:
              case MISCREG_ID_ISAR2:
              case MISCREG_ID_ISAR3:
              case MISCREG_ID_ISAR4:
              case MISCREG_ID_ISAR5:
              case MISCREG_ID_ISAR6:
                trap_to_hyp = hcr.tid3;
                break;
              case MISCREG_DCISW:
              case MISCREG_DCCSW:
              case MISCREG_DCCISW:
                trap_to_hyp = hcr.tsw;
                break;
              case MISCREG_DCIMVAC:
              case MISCREG_DCCIMVAC:
              case MISCREG_DCCMVAC:
                trap_to_hyp = hcr.tpc;
                break;
              case MISCREG_ICIMVAU:
              case MISCREG_ICIALLU:
              case MISCREG_ICIALLUIS:
              case MISCREG_DCCMVAU:
                trap_to_hyp = hcr.tpu;
                break;
              case MISCREG_TLBIALLIS:
              case MISCREG_TLBIMVAIS:
              case MISCREG_TLBIASIDIS:
              case MISCREG_TLBIMVAAIS:
              case MISCREG_TLBIMVALIS:
              case MISCREG_TLBIMVAALIS:
              case MISCREG_DTLBIALL:
              case MISCREG_ITLBIALL:
              case MISCREG_DTLBIMVA:
              case MISCREG_ITLBIMVA:
              case MISCREG_DTLBIASID:
              case MISCREG_ITLBIASID:
              case MISCREG_TLBIMVAA:
              case MISCREG_TLBIALL:
              case MISCREG_TLBIMVA:
              case MISCREG_TLBIMVAL:
              case MISCREG_TLBIMVAAL:
              case MISCREG_TLBIASID:
                trap_to_hyp = hcr.ttlb;
                break;
              case MISCREG_ACTLR:
                trap_to_hyp = hcr.tac;
                break;
              case MISCREG_SCTLR:
              case MISCREG_TTBR0:
              case MISCREG_TTBR1:
              case MISCREG_TTBCR:
              case MISCREG_DACR:
              case MISCREG_DFSR:
              case MISCREG_IFSR:
              case MISCREG_DFAR:
              case MISCREG_IFAR:
              case MISCREG_ADFSR:
              case MISCREG_AIFSR:
              case MISCREG_PRRR:
              case MISCREG_NMRR:
              case MISCREG_MAIR0:
              case MISCREG_MAIR1:
              case MISCREG_CONTEXTIDR:
                trap_to_hyp = hcr.tvm & !is_read;
                break;
              case MISCREG_PMCR:
                trap_to_hyp = hdcr.tpmcr;
                break;
              // GICv3 regs
              case MISCREG_ICC_SGI0R:
                trap_to_hyp = hcr.fmo;
                break;
              case MISCREG_ICC_SGI1R:
              case MISCREG_ICC_ASGI1R:
                trap_to_hyp = hcr.imo;
                break;
              case MISCREG_CNTFRQ ... MISCREG_CNTV_TVAL:
                // CNTFRQ may be trapped only on reads
                // CNTPCT and CNTVCT are read-only
                if (MISCREG_CNTFRQ <= misc_reg && misc_reg <= MISCREG_CNTVCT &&
                    !is_read)
                    break;
                trap_to_hyp = isGenericTimerHypTrap(misc_reg, tc, ec);
                break;
              // No default action needed
              default:
                break;
            }
        }
    }
    return trap_to_hyp;
}


bool
mcrMrc14TrapToHyp(const MiscRegIndex misc_reg, ThreadContext *tc, uint32_t iss)
{
    bool is_read;
    uint32_t crm;
    RegIndex rt;
    uint32_t crn;
    uint32_t opc1;
    uint32_t opc2;

    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    const HDCR hdcr = tc->readMiscReg(MISCREG_HDCR);
    const HSTR hstr = tc->readMiscReg(MISCREG_HSTR);
    const HCPTR hcptr = tc->readMiscReg(MISCREG_HCPTR);

    bool trap_to_hyp = false;

    if (EL2Enabled(tc) && (currEL(tc) < EL2)) {
        mcrMrcIssExtract(iss, is_read, crm, rt, crn, opc1, opc2);
        inform("trap check M:%x N:%x 1:%x 2:%x hdcr %x, hcptr %x, hstr %x\n",
                crm, crn, opc1, opc2, hdcr, hcptr, hstr);
        trap_to_hyp  = hdcr.tda  && (opc1 == 0);
        trap_to_hyp |= hcptr.tta && (opc1 == 1);
        if (!trap_to_hyp) {
            switch (unflattenMiscReg(misc_reg)) {
              case MISCREG_DBGOSLSR:
              case MISCREG_DBGOSLAR:
              case MISCREG_DBGOSDLR:
              case MISCREG_DBGPRCR:
                trap_to_hyp = hdcr.tdosa;
                break;
              case MISCREG_DBGDRAR:
              case MISCREG_DBGDSAR:
                trap_to_hyp = hdcr.tdra;
                break;
              case MISCREG_JIDR:
                trap_to_hyp = hcr.tid0;
                break;
              case MISCREG_JOSCR:
              case MISCREG_JMCR:
                trap_to_hyp = hstr.tjdbx;
                break;
              case MISCREG_TEECR:
              case MISCREG_TEEHBR:
                trap_to_hyp = hstr.ttee;
                break;
              // No default action needed
              default:
                break;
            }
        }
    }
    return trap_to_hyp;
}

Fault
mcrrMrrc15Trap(const MiscRegIndex misc_reg, ExtMachInst mach_inst,
               ThreadContext *tc, uint32_t imm)
{
    ExceptionClass ec = ExceptionClass::TRAPPED_CP15_MCRR_MRRC;
    if (mcrrMrrc15TrapToHyp(misc_reg, tc, imm, &ec))
        return std::make_shared<HypervisorTrap>(mach_inst, imm, ec);
    return AArch64AArch32SystemAccessTrap(misc_reg, mach_inst, tc, imm, ec);
}

bool
mcrrMrrc15TrapToHyp(const MiscRegIndex misc_reg, ThreadContext *tc,
                    uint32_t iss, ExceptionClass *ec)
{
    uint32_t crm;
    RegIndex rt;
    uint32_t crn;
    uint32_t opc1;
    uint32_t opc2;
    bool is_read;
    bool trap_to_hyp = false;

    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    const HSTR hstr = tc->readMiscReg(MISCREG_HSTR);

    if (EL2Enabled(tc) && (currEL(tc) < EL2)) {
        // This is technically the wrong function, but we can re-use it for
        // the moment because we only need one field, which overlaps with the
        // mcrmrc layout
        mcrMrcIssExtract(iss, is_read, crm, rt, crn, opc1, opc2);
        trap_to_hyp = ((uint32_t)hstr) & (1 << crm);

        if (!trap_to_hyp) {
            switch (unflattenMiscReg(misc_reg)) {
              case MISCREG_SCTLR:
              case MISCREG_TTBR0:
              case MISCREG_TTBR1:
              case MISCREG_TTBCR:
              case MISCREG_DACR:
              case MISCREG_DFSR:
              case MISCREG_IFSR:
              case MISCREG_DFAR:
              case MISCREG_IFAR:
              case MISCREG_ADFSR:
              case MISCREG_AIFSR:
              case MISCREG_PRRR:
              case MISCREG_NMRR:
              case MISCREG_MAIR0:
              case MISCREG_MAIR1:
              case MISCREG_CONTEXTIDR:
                trap_to_hyp = hcr.tvm & !is_read;
                break;
              case MISCREG_CNTFRQ ... MISCREG_CNTV_TVAL:
                // CNTFRQ may be trapped only on reads
                // CNTPCT and CNTVCT are read-only
                if (MISCREG_CNTFRQ <= misc_reg && misc_reg <= MISCREG_CNTVCT &&
                    !is_read) {
                    break;
                }
                trap_to_hyp = isGenericTimerHypTrap(misc_reg, tc, ec);
                break;
              // No default action needed
              default:
                break;
            }
        }
    }
    return trap_to_hyp;
}

Fault
AArch64AArch32SystemAccessTrap(const MiscRegIndex misc_reg,
                               ExtMachInst mach_inst, ThreadContext *tc,
                               uint32_t imm, ExceptionClass ec)
{
    if (currEL(tc) <= EL1 && !ELIs32(tc, EL1) &&
        isAArch64AArch32SystemAccessTrapEL1(misc_reg, tc))
        return std::make_shared<SupervisorTrap>(mach_inst, imm, ec);
    if (currEL(tc) <= EL2 && EL2Enabled(tc) && !ELIs32(tc, EL2) &&
        isAArch64AArch32SystemAccessTrapEL2(misc_reg, tc))
        return std::make_shared<HypervisorTrap>(mach_inst, imm, ec);
    return NoFault;
}

bool
isAArch64AArch32SystemAccessTrapEL1(const MiscRegIndex misc_reg,
                                    ThreadContext *tc)
{
    switch (misc_reg) {
      case MISCREG_CNTFRQ ... MISCREG_CNTVOFF:
        return currEL(tc) == EL0 &&
               isGenericTimerSystemAccessTrapEL1(misc_reg, tc);
      default:
        break;
    }
    return false;
}

bool
isGenericTimerHypTrap(const MiscRegIndex misc_reg, ThreadContext *tc,
                      ExceptionClass *ec)
{
    if (currEL(tc) <= EL2 && EL2Enabled(tc) && ELIs32(tc, EL2)) {
        switch (misc_reg) {
          case MISCREG_CNTFRQ ... MISCREG_CNTV_TVAL:
            if (currEL(tc) == EL0 &&
                isGenericTimerCommonEL0HypTrap(misc_reg, tc, ec))
                return true;
            switch (misc_reg) {
              case MISCREG_CNTPCT:
              case MISCREG_CNTP_CTL ... MISCREG_CNTP_TVAL_S:
                return currEL(tc) <= EL1 &&
                       isGenericTimerPhysHypTrap(misc_reg, tc, ec);
              default:
                break;
            }
            break;
          default:
            break;
        }
    }
    return false;
}

bool
isGenericTimerCommonEL0HypTrap(const MiscRegIndex misc_reg, ThreadContext *tc,
                               ExceptionClass *ec)
{
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    bool trap_cond = condGenericTimerSystemAccessTrapEL1(misc_reg, tc);
    if (ELIs32(tc, EL1) && trap_cond && hcr.tge) {
        // As per the architecture, this hyp trap should have uncategorized
        // exception class
        if (ec)
            *ec = ExceptionClass::UNKNOWN;
        return true;
    }
    return false;
}

bool
isGenericTimerPhysHypTrap(const MiscRegIndex misc_reg, ThreadContext *tc,
                          ExceptionClass *ec)
{
    return condGenericTimerPhysHypTrap(misc_reg, tc);
}

bool
condGenericTimerPhysHypTrap(const MiscRegIndex misc_reg, ThreadContext *tc)
{
    const CNTHCTL cnthctl = tc->readMiscReg(MISCREG_CNTHCTL_EL2);
    switch (misc_reg) {
      case MISCREG_CNTPCT:
        return !cnthctl.el1pcten;
      case MISCREG_CNTP_CTL ... MISCREG_CNTP_TVAL_S:
        return !cnthctl.el1pcen;
      default:
        break;
    }
    return false;
}

bool
isGenericTimerSystemAccessTrapEL1(const MiscRegIndex misc_reg,
                                  ThreadContext *tc)
{
    switch (misc_reg) {
      case MISCREG_CNTFRQ ... MISCREG_CNTV_TVAL:
      case MISCREG_CNTFRQ_EL0 ... MISCREG_CNTV_TVAL_EL0:
      {
        const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
        bool trap_cond = condGenericTimerSystemAccessTrapEL1(misc_reg, tc);
        return !(EL2Enabled(tc) && hcr.e2h && hcr.tge) && trap_cond &&
               !(EL2Enabled(tc) && !ELIs32(tc, EL2) && hcr.tge);
      }
      default:
        break;
    }
    return false;
}

bool
condGenericTimerSystemAccessTrapEL1(const MiscRegIndex misc_reg,
                                    ThreadContext *tc)
{
    const CNTKCTL cntkctl = tc->readMiscReg(MISCREG_CNTKCTL_EL1);
    switch (misc_reg) {
      case MISCREG_CNTFRQ:
      case MISCREG_CNTFRQ_EL0:
        return !cntkctl.el0pcten && !cntkctl.el0vcten;
      case MISCREG_CNTPCT:
      case MISCREG_CNTPCT_EL0:
        return !cntkctl.el0pcten;
      case MISCREG_CNTVCT:
      case MISCREG_CNTVCT_EL0:
        return !cntkctl.el0vcten;
      case MISCREG_CNTP_CTL ... MISCREG_CNTP_TVAL_S:
      case MISCREG_CNTP_CTL_EL0 ... MISCREG_CNTP_TVAL_EL0:
        return !cntkctl.el0pten;
      case MISCREG_CNTV_CTL ... MISCREG_CNTV_TVAL:
      case MISCREG_CNTV_CTL_EL0 ... MISCREG_CNTV_TVAL_EL0:
        return !cntkctl.el0vten;
      default:
        break;
    }
    return false;
}

bool
isAArch64AArch32SystemAccessTrapEL2(const MiscRegIndex misc_reg,
                                    ThreadContext *tc)
{
    switch (misc_reg) {
      case MISCREG_CNTFRQ ... MISCREG_CNTVOFF:
        return currEL(tc) <= EL1 &&
               isGenericTimerSystemAccessTrapEL2(misc_reg, tc);
      default:
        break;
    }
    return false;
}

bool
isGenericTimerSystemAccessTrapEL2(const MiscRegIndex misc_reg,
                                  ThreadContext *tc)
{
    switch (misc_reg) {
      case MISCREG_CNTFRQ ... MISCREG_CNTV_TVAL:
      case MISCREG_CNTFRQ_EL0 ... MISCREG_CNTV_TVAL_EL0:
        if (currEL(tc) == EL0 &&
            isGenericTimerCommonEL0SystemAccessTrapEL2(misc_reg, tc))
            return true;
        switch (misc_reg) {
          case MISCREG_CNTPCT:
          case MISCREG_CNTPCT_EL0:
          case MISCREG_CNTP_CTL ... MISCREG_CNTP_TVAL_S:
          case MISCREG_CNTP_CTL_EL0 ... MISCREG_CNTP_TVAL_EL0:
            return (currEL(tc) == EL0 &&
                    isGenericTimerPhysEL0SystemAccessTrapEL2(misc_reg, tc)) ||
                   (currEL(tc) == EL1 &&
                    isGenericTimerPhysEL1SystemAccessTrapEL2(misc_reg, tc));
          case MISCREG_CNTVCT:
          case MISCREG_CNTVCT_EL0:
          case MISCREG_CNTV_CTL ... MISCREG_CNTV_TVAL:
          case MISCREG_CNTV_CTL_EL0 ... MISCREG_CNTV_TVAL_EL0:
            return isGenericTimerVirtSystemAccessTrapEL2(misc_reg, tc);
          default:
            break;
        }
        break;
      default:
        break;
    }
    return false;
}

bool
isGenericTimerCommonEL0SystemAccessTrapEL2(const MiscRegIndex misc_reg,
                                           ThreadContext *tc)
{
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    bool trap_cond_el1 = condGenericTimerSystemAccessTrapEL1(misc_reg, tc);
    bool trap_cond_el2 = condGenericTimerCommonEL0SystemAccessTrapEL2(misc_reg,
                                                                      tc);
    return (!ELIs32(tc, EL1) && !hcr.e2h && trap_cond_el1 && hcr.tge) ||
           (ELIs32(tc, EL1) && trap_cond_el1 && hcr.tge) ||
           (hcr.e2h && hcr.tge && trap_cond_el2);
}

bool
isGenericTimerPhysEL0SystemAccessTrapEL2(const MiscRegIndex misc_reg,
                                         ThreadContext *tc)
{
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    bool trap_cond_0 = condGenericTimerPhysEL1SystemAccessTrapEL2(
        misc_reg, tc);
    bool trap_cond_1 = condGenericTimerCommonEL1SystemAccessTrapEL2(
        misc_reg, tc);

    switch (misc_reg) {
      case MISCREG_CNTPCT:
      case MISCREG_CNTPCT_EL0:
        return !hcr.e2h && trap_cond_1;
      case MISCREG_CNTP_CTL ... MISCREG_CNTP_TVAL_S:
      case MISCREG_CNTP_CTL_EL0 ... MISCREG_CNTP_TVAL_EL0:
        return (!hcr.e2h && trap_cond_0) ||
               (hcr.e2h && !hcr.tge && trap_cond_1);
      default:
        break;
    }

    return false;
}

bool
isGenericTimerPhysEL1SystemAccessTrapEL2(const MiscRegIndex misc_reg,
                                         ThreadContext *tc)
{
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    bool trap_cond_0 = condGenericTimerPhysEL1SystemAccessTrapEL2(
        misc_reg, tc);
    bool trap_cond_1 = condGenericTimerCommonEL1SystemAccessTrapEL2(
        misc_reg, tc);

    switch (misc_reg) {
      case MISCREG_CNTPCT:
      case MISCREG_CNTPCT_EL0:
        return trap_cond_1;
      case MISCREG_CNTP_CTL ... MISCREG_CNTP_TVAL_S:
      case MISCREG_CNTP_CTL_EL0 ... MISCREG_CNTP_TVAL_EL0:
        return (!hcr.e2h && trap_cond_0) ||
               (hcr.e2h && trap_cond_1);
      default:
        break;
    }
    return false;
}

bool
isGenericTimerVirtSystemAccessTrapEL2(const MiscRegIndex misc_reg,
                                      ThreadContext *tc)
{
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    bool trap_cond = condGenericTimerCommonEL1SystemAccessTrapEL2(
        misc_reg, tc);
    return !ELIs32(tc, EL1) && !(hcr.e2h && hcr.tge) && trap_cond;
}

bool
condGenericTimerCommonEL0SystemAccessTrapEL2(const MiscRegIndex misc_reg,
                                             ThreadContext *tc)
{
    const CNTHCTL_E2H cnthctl = tc->readMiscReg(MISCREG_CNTHCTL_EL2);
    switch (misc_reg) {
      case MISCREG_CNTFRQ:
      case MISCREG_CNTFRQ_EL0:
        return !cnthctl.el0pcten && !cnthctl.el0vcten;
      case MISCREG_CNTPCT:
      case MISCREG_CNTPCT_EL0:
        return !cnthctl.el0pcten;
      case MISCREG_CNTVCT:
      case MISCREG_CNTVCT_EL0:
        return !cnthctl.el0vcten;
      case MISCREG_CNTP_CTL ... MISCREG_CNTP_TVAL_S:
      case MISCREG_CNTP_CTL_EL0 ... MISCREG_CNTP_TVAL_EL0:
        return !cnthctl.el0pten;
      case MISCREG_CNTV_CTL ... MISCREG_CNTV_TVAL:
      case MISCREG_CNTV_CTL_EL0 ... MISCREG_CNTV_TVAL_EL0:
        return !cnthctl.el0vten;
      default:
        break;
    }
    return false;
}

bool
condGenericTimerCommonEL1SystemAccessTrapEL2(const MiscRegIndex misc_reg,
                                             ThreadContext *tc)
{
    const AA64MMFR0 mmfr0 = tc->readMiscRegNoEffect(MISCREG_ID_AA64MMFR0_EL1);
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    const RegVal cnthctl_val = tc->readMiscReg(MISCREG_CNTHCTL_EL2);
    const CNTHCTL cnthctl = cnthctl_val;
    const CNTHCTL_E2H cnthctl_e2h = cnthctl_val;
    switch (misc_reg) {
      case MISCREG_CNTPCT:
      case MISCREG_CNTPCT_EL0:
        return hcr.e2h ? !cnthctl_e2h.el1pcten : !cnthctl.el1pcten;
      case MISCREG_CNTVCT:
      case MISCREG_CNTVCT_EL0:
        if (!mmfr0.ecv)
            return false;
        else
            return hcr.e2h ? cnthctl_e2h.el1tvct : cnthctl.el1tvct;
      case MISCREG_CNTP_CTL ... MISCREG_CNTP_TVAL_S:
      case MISCREG_CNTP_CTL_EL0 ... MISCREG_CNTP_TVAL_EL0:
        return hcr.e2h ? !cnthctl_e2h.el1pten : false;
      case MISCREG_CNTV_CTL ... MISCREG_CNTV_TVAL:
      case MISCREG_CNTV_CTL_EL0 ... MISCREG_CNTV_TVAL_EL0:
        if (!mmfr0.ecv)
            return false;
        else
            return hcr.e2h ? cnthctl_e2h.el1tvt : cnthctl.el1tvt;
      default:
        break;
    }
    return false;
}

bool
condGenericTimerPhysEL1SystemAccessTrapEL2(const MiscRegIndex misc_reg,
                                           ThreadContext *tc)
{
    const CNTHCTL cnthctl = tc->readMiscReg(MISCREG_CNTHCTL_EL2);
    return !cnthctl.el1pcen;
}

bool
isGenericTimerSystemAccessTrapEL3(const MiscRegIndex misc_reg,
                                  ThreadContext *tc)
{
    switch (misc_reg) {
      case MISCREG_CNTPS_CTL_EL1 ... MISCREG_CNTPS_TVAL_EL1:
      {
        const SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
        return currEL(tc) == EL1 && !scr.ns && !scr.st;
      }
      default:
        break;
    }
    return false;
}

bool
decodeMrsMsrBankedReg(uint8_t sysM, bool r, bool &isIntReg, int &regIdx,
                      CPSR cpsr, SCR scr, NSACR nsacr, bool checkSecurity)
{
    OperatingMode mode = MODE_UNDEFINED;
    bool          ok = true;

    // R mostly indicates if its a int register or a misc reg, we override
    // below if the few corner cases
    isIntReg = !r;
    // Loosely based on ARM ARM issue C section B9.3.10
    if (r) {
        switch (sysM) {
          case 0xE:
            regIdx = MISCREG_SPSR_FIQ;
            mode = MODE_FIQ;
            break;
          case 0x10:
            regIdx = MISCREG_SPSR_IRQ;
            mode = MODE_IRQ;
            break;
          case 0x12:
            regIdx = MISCREG_SPSR_SVC;
            mode = MODE_SVC;
            break;
          case 0x14:
            regIdx = MISCREG_SPSR_ABT;
            mode = MODE_ABORT;
            break;
          case 0x16:
            regIdx = MISCREG_SPSR_UND;
            mode = MODE_UNDEFINED;
            break;
          case 0x1C:
            regIdx = MISCREG_SPSR_MON;
            mode = MODE_MON;
            break;
          case 0x1E:
            regIdx = MISCREG_SPSR_HYP;
            mode = MODE_HYP;
            break;
          default:
            ok = false;
            break;
        }
    } else {
        int sysM4To3 = bits(sysM, 4, 3);

        if (sysM4To3 == 0) {
            mode = MODE_USER;
            regIdx = int_reg::regInMode(mode, bits(sysM, 2, 0) + 8);
        } else if (sysM4To3 == 1) {
            mode = MODE_FIQ;
            regIdx = int_reg::regInMode(mode, bits(sysM, 2, 0) + 8);
        } else if (sysM4To3 == 3) {
            if (bits(sysM, 1) == 0) {
                mode = MODE_MON;
                regIdx = int_reg::regInMode(mode, 14 - bits(sysM, 0));
            } else {
                mode = MODE_HYP;
                if (bits(sysM, 0) == 1) {
                    regIdx = int_reg::regInMode(mode, 13); // R13 in HYP
                } else {
                    isIntReg = false;
                    regIdx = MISCREG_ELR_HYP;
                }
            }
        } else { // Other Banked registers
            int sysM2 = bits(sysM, 2);
            int sysM1 = bits(sysM, 1);

            mode  = (OperatingMode)(((sysM2 || sysM1) << 0) |
                                    (1 << 1) |
                                    ((sysM2 && !sysM1) << 2) |
                                    ((sysM2 && sysM1) << 3) |
                                    (1 << 4));
            regIdx = int_reg::regInMode(mode, 14 - bits(sysM, 0));
            // Don't flatten the register here. This is going to go through
            // setReg() which will do the flattening
            ok &= mode != cpsr.mode;
        }
    }

    // Check that the requested register is accessable from the current mode
    if (ok && checkSecurity && mode != cpsr.mode) {
        switch (cpsr.mode) {
          case MODE_USER:
            ok = false;
            break;
          case MODE_FIQ:
            ok &= mode != MODE_HYP;
            ok &= (mode != MODE_MON) || !scr.ns;
            break;
          case MODE_HYP:
            ok &= mode != MODE_MON;
            ok &= (mode != MODE_FIQ) || !nsacr.rfr;
            break;
          case MODE_IRQ:
          case MODE_SVC:
          case MODE_ABORT:
          case MODE_UNDEFINED:
          case MODE_SYSTEM:
            ok &= mode != MODE_HYP;
            ok &= (mode != MODE_MON) || !scr.ns;
            ok &= (mode != MODE_FIQ) || !nsacr.rfr;
            break;
          // can access everything, no further checks required
          case MODE_MON:
            break;
          default:
            panic("unknown Mode 0x%x\n", cpsr.mode);
            break;
        }
    }
    return ok;
}

bool
isUnpriviledgeAccess(ThreadContext *tc)
{
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    const CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);

    // NV Extension not implemented yet
    bool have_nv_ext = false;
    bool unpriv_el1 = currEL(tc) == EL1 &&
        !(ArmSystem::haveEL(tc, EL2) &&
            have_nv_ext && hcr.nv == 1 && hcr.nv1 == 1);
    bool unpriv_el2 = ArmSystem::haveEL(tc, EL2) &&
                      HaveExt(tc, ArmExtension::FEAT_VHE) &&
                      currEL(tc) == EL2 && hcr.e2h == 1 && hcr.tge == 1;

    return (unpriv_el1 || unpriv_el2) && !cpsr.uao;
}

bool
SPAlignmentCheckEnabled(ThreadContext *tc)
{
    ExceptionLevel regime = s1TranslationRegime(tc, currEL(tc));

    switch (currEL(tc)) {
      case EL3:
        return ((SCTLR)tc->readMiscReg(MISCREG_SCTLR_EL3)).sa;
      case EL2:
        return ((SCTLR)tc->readMiscReg(MISCREG_SCTLR_EL2)).sa;
      case EL1:
        return ((SCTLR)tc->readMiscReg(MISCREG_SCTLR_EL1)).sa;
      case EL0:
        {
          SCTLR sc = (regime == EL2) ? tc->readMiscReg(MISCREG_SCTLR_EL2):
                                       tc->readMiscReg(MISCREG_SCTLR_EL1);
          return sc.sa0;
        }
      default:
        panic("Invalid exception level");
        break;
    }
}

int
decodePhysAddrRange64(uint8_t pa_enc)
{
    switch (pa_enc) {
      case 0x0:
        return 32;
      case 0x1:
        return 36;
      case 0x2:
        return 40;
      case 0x3:
        return 42;
      case 0x4:
        return 44;
      case 0x5:
        return 48;
      case 0x6:
        return 52;
      default:
        panic("Invalid phys. address range encoding");
    }
}

uint8_t
encodePhysAddrRange64(int pa_size)
{
    switch (pa_size) {
      case 32:
        return 0x0;
      case 36:
        return 0x1;
      case 40:
        return 0x2;
      case 42:
        return 0x3;
      case 44:
        return 0x4;
      case 48:
        return 0x5;
      case 52:
        return 0x6;
      default:
        panic("Invalid phys. address range");
    }
}

void
syncVecRegsToElems(ThreadContext *tc)
{
    int ei = 0;
    for (int ri = 0; ri < NumVecRegs; ri++) {
        VecRegContainer reg;
        tc->getReg(vecRegClass[ri], &reg);
        for (int j = 0; j < NumVecElemPerVecReg; j++, ei++)
            tc->setReg(vecElemClass[ei], reg.as<VecElem>()[j]);
    }
}

void
syncVecElemsToRegs(ThreadContext *tc)
{
    int ei = 0;
    for (int ri = 0; ri < NumVecRegs; ri++) {
        VecRegContainer reg;
        for (int j = 0; j < NumVecElemPerVecReg; j++, ei++) {
            RegId elem_id = vecElemClass[ei];
            reg.as<VecElem>()[j] = tc->getReg(elem_id);
        }
        tc->setReg(vecRegClass[ri], &reg);
    }
}

} // namespace ArmISA
} // namespace gem5
