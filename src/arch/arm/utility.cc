/*
 * Copyright (c) 2009-2014, 2016-2020, 2022-2025 Arm Limited
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

SecurityState
securityStateAtEL(ThreadContext *tc, ExceptionLevel el)
{
    if (ArmSystem::haveEL(tc, EL3) && el == EL3)
        return SecurityState::Secure;
    else
        return isSecureBelowEL3(tc) ? SecurityState::Secure :
                                      SecurityState::NonSecure;
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

    switch (current_el) {
      case EL0:
        // Note: in MsrMrs instruction we read the register value before
        // checking access permissions. This means that EL0 entry must
        // be part of the table even if MPIDR is not accessible in user
        // mode.
        warn_once("Trying to read MPIDR at EL0\n");
        [[fallthrough]];
      case EL1:
        if (EL2Enabled(tc))
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

unsigned
addrAlignmentFlags(int memsize, unsigned memAccessFlags)
{
    unsigned flags = memAccessFlags & (~(unsigned)MMU::AlignmentMask);
    switch (memsize) {
        case 1:
            flags = flags | MMU::AlignByte;
            break;
        case 2:
            flags = flags | MMU::AlignHalfWord;
            break;
        case 4:
            flags = flags | MMU::AlignWord;
            break;
        case 8:
            flags = flags | MMU::AlignDoubleWord;
            break;
        case 16:
            flags = flags | MMU::AlignQuadWord;
            break;
        case 32:
            flags = flags | MMU::AlignOctWord;
            break;
        default:
            flags = flags | MMU::AllowUnaligned;
            break;
    }
    return flags;
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

bool
fgtEnabled(ThreadContext *tc)
{
    return EL2Enabled(tc) && HaveExt(tc, ArmExtension::FEAT_FGT) &&
        (!ArmSystem::haveEL(tc, EL3) ||
            static_cast<SCR>(tc->readMiscReg(MISCREG_SCR_EL3)).fgten);
}

bool
isHcrxEL2Enabled(ThreadContext *tc)
{
    if (!ArmSystem::has(ArmExtension::FEAT_HCX, tc))
        return false;
    if (ArmSystem::haveEL(tc, EL3) &&
        !static_cast<SCR>(tc->readMiscReg(MISCREG_SCR_EL3)).hxen)
        return false;
    return EL2Enabled(tc);
}

TranslationRegime
translationRegime(ThreadContext *tc, ExceptionLevel el)
{
    switch (el) {
      case EL3:
        return TranslationRegime::EL3;
      case EL2:
        return ELIsInHost(tc, EL2) ?
            TranslationRegime::EL20 : TranslationRegime::EL2;
      case EL1:
        return TranslationRegime::EL10;
      case EL0:
        return ELIsInHost(tc, EL0) ?
            TranslationRegime::EL20 : TranslationRegime::EL10;
      default:
        panic("Invalid ExceptionLevel\n");
    }
}

ExceptionLevel
translationEl(TranslationRegime regime)
{
    switch (regime) {
      case TranslationRegime::EL10:
        return EL1;
      case TranslationRegime::EL20:
      case TranslationRegime::EL2:
        return EL2;
      case TranslationRegime::EL3:
        return EL3;
      default:
        return EL1;
    }
}

} // namespace ArmISA
} // namespace gem5
