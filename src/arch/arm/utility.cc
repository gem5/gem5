/*
 * Copyright (c) 2009-2014, 2016-2018 ARM Limited
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
 *
 * Authors: Ali Saidi
 */

#include "arch/arm/utility.hh"

#include <memory>

#include "arch/arm/faults.hh"
#include "arch/arm/isa_traits.hh"
#include "arch/arm/system.hh"
#include "arch/arm/tlb.hh"
#include "arch/arm/vtophys.hh"
#include "cpu/base.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/thread_context.hh"
#include "mem/fs_translating_port_proxy.hh"
#include "sim/full_system.hh"

namespace ArmISA {

void
initCPU(ThreadContext *tc, int cpuId)
{
    // Reset CP15?? What does that mean -- ali

    // FPEXC.EN = 0

    static Fault reset = std::make_shared<Reset>();
    reset->invoke(tc);
}

uint64_t
getArgument(ThreadContext *tc, int &number, uint16_t size, bool fp)
{
    if (!FullSystem) {
        panic("getArgument() only implemented for full system mode.\n");
        M5_DUMMY_RETURN
    }

    if (fp)
        panic("getArgument(): Floating point arguments not implemented\n");

    if (inAArch64(tc)) {
        if (size == (uint16_t)(-1))
            size = sizeof(uint64_t);

        if (number < 8 /*NumArgumentRegs64*/) {
               return tc->readIntReg(number);
        } else {
            panic("getArgument(): No support reading stack args for AArch64\n");
        }
    } else {
        if (size == (uint16_t)(-1))
            // todo: should this not be sizeof(uint32_t) rather?
            size = ArmISA::MachineBytes;

        if (number < NumArgumentRegs) {
            // If the argument is 64 bits, it must be in an even regiser
            // number. Increment the number here if it isn't even.
            if (size == sizeof(uint64_t)) {
                if ((number % 2) != 0)
                    number++;
                // Read the two halves of the data. Number is inc here to
                // get the second half of the 64 bit reg.
                uint64_t tmp;
                tmp = tc->readIntReg(number++);
                tmp |= tc->readIntReg(number) << 32;
                return tmp;
            } else {
               return tc->readIntReg(number);
            }
        } else {
            Addr sp = tc->readIntReg(StackPointerReg);
            FSTranslatingPortProxy &vp = tc->getVirtProxy();
            uint64_t arg;
            if (size == sizeof(uint64_t)) {
                // If the argument is even it must be aligned
                if ((number % 2) != 0)
                    number++;
                arg = vp.read<uint64_t>(sp +
                        (number-NumArgumentRegs) * sizeof(uint32_t));
                // since two 32 bit args == 1 64 bit arg, increment number
                number++;
            } else {
                arg = vp.read<uint32_t>(sp +
                               (number-NumArgumentRegs) * sizeof(uint32_t));
            }
            return arg;
        }
    }
    panic("getArgument() should always return\n");
}

void
skipFunction(ThreadContext *tc)
{
    PCState newPC = tc->pcState();
    if (inAArch64(tc)) {
        newPC.set(tc->readIntReg(INTREG_X30));
    } else {
        newPC.set(tc->readIntReg(ReturnAddressReg) & ~ULL(1));
    }

    CheckerCPU *checker = tc->getCheckerCpuPtr();
    if (checker) {
        tc->pcStateNoRecord(newPC);
    } else {
        tc->pcState(newPC);
    }
}

static void
copyVecRegs(ThreadContext *src, ThreadContext *dest)
{
    auto src_mode = RenameMode<ArmISA::ISA>::mode(src->pcState());

    // The way vector registers are copied (VecReg vs VecElem) is relevant
    // in the O3 model only.
    if (src_mode == Enums::Full) {
        for (auto idx = 0; idx < NumVecRegs; idx++)
            dest->setVecRegFlat(idx, src->readVecRegFlat(idx));
    } else {
        for (auto idx = 0; idx < NumVecRegs; idx++)
            for (auto elem_idx = 0; elem_idx < NumVecElemPerVecReg; elem_idx++)
                dest->setVecElemFlat(
                    idx, elem_idx, src->readVecElemFlat(idx, elem_idx));
    }
}

void
copyRegs(ThreadContext *src, ThreadContext *dest)
{
    for (int i = 0; i < NumIntRegs; i++)
        dest->setIntRegFlat(i, src->readIntRegFlat(i));

    for (int i = 0; i < NumFloatRegs; i++)
        dest->setFloatRegFlat(i, src->readFloatRegFlat(i));

    for (int i = 0; i < NumCCRegs; i++)
        dest->setCCReg(i, src->readCCReg(i));

    for (int i = 0; i < NumMiscRegs; i++)
        dest->setMiscRegNoEffect(i, src->readMiscRegNoEffect(i));

    copyVecRegs(src, dest);

    // setMiscReg "with effect" will set the misc register mapping correctly.
    // e.g. updateRegMap(val)
    dest->setMiscReg(MISCREG_CPSR, src->readMiscRegNoEffect(MISCREG_CPSR));

    // Copy over the PC State
    dest->pcState(src->pcState());

    // Invalidate the tlb misc register cache
    dynamic_cast<TLB *>(dest->getITBPtr())->invalidateMiscReg();
    dynamic_cast<TLB *>(dest->getDTBPtr())->invalidateMiscReg();
}

bool
inSecureState(ThreadContext *tc)
{
    SCR scr = inAArch64(tc) ? tc->readMiscReg(MISCREG_SCR_EL3) :
        tc->readMiscReg(MISCREG_SCR);
    return ArmSystem::haveSecurity(tc) && inSecureState(
        scr, tc->readMiscReg(MISCREG_CPSR));
}

inline bool
isSecureBelowEL3(ThreadContext *tc)
{
    SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
    return ArmSystem::haveEL(tc, EL3) && scr.ns == 0;
}

bool
inAArch64(ThreadContext *tc)
{
    CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);
    return opModeIs64((OperatingMode) (uint8_t) cpsr.mode);
}

bool
longDescFormatInUse(ThreadContext *tc)
{
    TTBCR ttbcr = tc->readMiscReg(MISCREG_TTBCR);
    return ArmSystem::haveLPAE(tc) && ttbcr.eae;
}

RegVal
readMPIDR(ArmSystem *arm_sys, ThreadContext *tc)
{
    CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);
    const ExceptionLevel current_el =
        opModeToEL((OperatingMode) (uint8_t) cpsr.mode);

    const bool is_secure = isSecureBelowEL3(tc);

    switch (current_el) {
      case EL0:
        // Note: in MsrMrs instruction we read the register value before
        // checking access permissions. This means that EL0 entry must
        // be part of the table even if MPIDR is not accessible in user
        // mode.
        warn_once("Trying to read MPIDR at EL0\n");
        M5_FALLTHROUGH;
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
    if (arm_sys->multiThread) {
       return 0x80000000 | // multiprocessor extensions available
              0x01000000 | // multi-threaded cores
              tc->contextId();
    } else if (arm_sys->multiProc) {
       return 0x80000000 | // multiprocessor extensions available
              tc->cpuId() | tc->socketId() << 8;
    } else {
       return 0x80000000 |  // multiprocessor extensions available
              0x40000000 |  // in up system
              tc->cpuId() | tc->socketId() << 8;
    }
}

bool
ELIs64(ThreadContext *tc, ExceptionLevel el)
{
    return !ELIs32(tc, el);
}

bool
ELIs32(ThreadContext *tc, ExceptionLevel el)
{
    bool known, aarch32;
    std::tie(known, aarch32) = ELUsingAArch32K(tc, el);
    panic_if(!known, "EL state is UNKNOWN");
    return aarch32;
}

bool
ELIsInHost(ThreadContext *tc, ExceptionLevel el)
{
    if (!ArmSystem::haveVirtualization(tc)) {
        return false;
    }
    HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    return (!isSecureBelowEL3(tc) && !ELIs32(tc, EL2) && hcr.e2h == 1 &&
            (el == EL2 || (el == EL0 && hcr.tge == 1)));
}

std::pair<bool, bool>
ELUsingAArch32K(ThreadContext *tc, ExceptionLevel el)
{
    // Return true if the specified EL is in aarch32 state.
    const bool have_el3 = ArmSystem::haveSecurity(tc);
    const bool have_el2 = ArmSystem::haveVirtualization(tc);

    panic_if(el == EL2 && !have_el2, "Asking for EL2 when it doesn't exist");
    panic_if(el == EL3 && !have_el3, "Asking for EL3 when it doesn't exist");

    bool known, aarch32;
    known = aarch32 = false;
    if (ArmSystem::highestELIs64(tc) && ArmSystem::highestEL(tc) == el) {
        // Target EL is the highest one in a system where
        // the highest is using AArch64.
        known = true; aarch32 = false;
    } else if (!ArmSystem::highestELIs64(tc)) {
        // All ELs are using AArch32:
        known = true; aarch32 = true;
    } else {
        SCR scr = tc->readMiscReg(MISCREG_SCR_EL3);
        bool aarch32_below_el3 = (have_el3 && scr.rw == 0);

        HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
        bool aarch32_at_el1 = (aarch32_below_el3
                               || (have_el2
                               && !isSecureBelowEL3(tc) && hcr.rw == 0));

        // Only know if EL0 using AArch32 from PSTATE
        if (el == EL0 && !aarch32_at_el1) {
            // EL0 controlled by PSTATE
            CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);

            known = (cpsr.el == EL0);
            aarch32 = (cpsr.width == 1);
        } else {
            known = true;
            aarch32 = (aarch32_below_el3 && el != EL3)
                      || (aarch32_at_el1 && (el == EL0 || el == EL1) );
        }
    }

    return std::make_pair(known, aarch32);
}

bool
isBigEndian64(ThreadContext *tc)
{
    switch (opModeToEL(currOpMode(tc))) {
      case EL3:
        return ((SCTLR) tc->readMiscReg(MISCREG_SCTLR_EL3)).ee;
      case EL2:
        return ((SCTLR) tc->readMiscReg(MISCREG_SCTLR_EL2)).ee;
      case EL1:
        return ((SCTLR) tc->readMiscReg(MISCREG_SCTLR_EL1)).ee;
      case EL0:
        return ((SCTLR) tc->readMiscReg(MISCREG_SCTLR_EL1)).e0e;
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

Addr
purifyTaggedAddr(Addr addr, ThreadContext *tc, ExceptionLevel el,
                 TTBCR tcr)
{
    switch (el) {
      case EL0:
      case EL1:
        if (bits(addr, 55, 48) == 0xFF && tcr.tbi1)
            return addr | mask(63, 55);
        else if (!bits(addr, 55, 48) && tcr.tbi0)
            return bits(addr,55, 0);
        break;
      case EL2:
        assert(ArmSystem::haveVirtualization(tc));
        tcr = tc->readMiscReg(MISCREG_TCR_EL2);
        if (tcr.tbi)
            return addr & mask(56);
        break;
      case EL3:
        assert(ArmSystem::haveSecurity(tc));
        if (tcr.tbi)
            return addr & mask(56);
        break;
      default:
        panic("Invalid exception level");
        break;
    }

    return addr;  // Nothing to do if this is not a tagged address
}

Addr
purifyTaggedAddr(Addr addr, ThreadContext *tc, ExceptionLevel el)
{
    TTBCR tcr;

    switch (el) {
      case EL0:
      case EL1:
        tcr = tc->readMiscReg(MISCREG_TCR_EL1);
        if (bits(addr, 55, 48) == 0xFF && tcr.tbi1)
            return addr | mask(63, 55);
        else if (!bits(addr, 55, 48) && tcr.tbi0)
            return bits(addr,55, 0);
        break;
      case EL2:
        assert(ArmSystem::haveVirtualization(tc));
        tcr = tc->readMiscReg(MISCREG_TCR_EL2);
        if (tcr.tbi)
            return addr & mask(56);
        break;
      case EL3:
        assert(ArmSystem::haveSecurity(tc));
        tcr = tc->readMiscReg(MISCREG_TCR_EL3);
        if (tcr.tbi)
            return addr & mask(56);
        break;
      default:
        panic("Invalid exception level");
        break;
    }

    return addr;  // Nothing to do if this is not a tagged address
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
mcrMrc15TrapToHyp(const MiscRegIndex miscReg, HCR hcr, CPSR cpsr, SCR scr,
                  HDCR hdcr, HSTR hstr, HCPTR hcptr, uint32_t iss)
{
    bool        isRead;
    uint32_t    crm;
    IntRegIndex rt;
    uint32_t    crn;
    uint32_t    opc1;
    uint32_t    opc2;
    bool        trapToHype = false;


    if (!inSecureState(scr, cpsr) && (cpsr.mode != MODE_HYP)) {
        mcrMrcIssExtract(iss, isRead, crm, rt, crn, opc1, opc2);
        trapToHype  = ((uint32_t) hstr) & (1 << crn);
        trapToHype |= hdcr.tpm  && (crn == 9) && (crm >= 12);
        trapToHype |= hcr.tidcp && (
            ((crn ==  9) && ((crm <= 2) || ((crm >= 5) && (crm <= 8)))) ||
            ((crn == 10) && ((crm <= 1) ||  (crm == 4) || (crm == 8)))  ||
            ((crn == 11) && ((crm <= 8) ||  (crm == 15)))               );

        if (!trapToHype) {
            switch (unflattenMiscReg(miscReg)) {
              case MISCREG_CPACR:
                trapToHype = hcptr.tcpac;
                break;
              case MISCREG_REVIDR:
              case MISCREG_TCMTR:
              case MISCREG_TLBTR:
              case MISCREG_AIDR:
                trapToHype = hcr.tid1;
                break;
              case MISCREG_CTR:
              case MISCREG_CCSIDR:
              case MISCREG_CLIDR:
              case MISCREG_CSSELR:
                trapToHype = hcr.tid2;
                break;
              case MISCREG_ID_PFR0:
              case MISCREG_ID_PFR1:
              case MISCREG_ID_DFR0:
              case MISCREG_ID_AFR0:
              case MISCREG_ID_MMFR0:
              case MISCREG_ID_MMFR1:
              case MISCREG_ID_MMFR2:
              case MISCREG_ID_MMFR3:
              case MISCREG_ID_ISAR0:
              case MISCREG_ID_ISAR1:
              case MISCREG_ID_ISAR2:
              case MISCREG_ID_ISAR3:
              case MISCREG_ID_ISAR4:
              case MISCREG_ID_ISAR5:
                trapToHype = hcr.tid3;
                break;
              case MISCREG_DCISW:
              case MISCREG_DCCSW:
              case MISCREG_DCCISW:
                trapToHype = hcr.tsw;
                break;
              case MISCREG_DCIMVAC:
              case MISCREG_DCCIMVAC:
              case MISCREG_DCCMVAC:
                trapToHype = hcr.tpc;
                break;
              case MISCREG_ICIMVAU:
              case MISCREG_ICIALLU:
              case MISCREG_ICIALLUIS:
              case MISCREG_DCCMVAU:
                trapToHype = hcr.tpu;
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
                trapToHype = hcr.ttlb;
                break;
              case MISCREG_ACTLR:
                trapToHype = hcr.tac;
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
                trapToHype = hcr.tvm & !isRead;
                break;
              case MISCREG_PMCR:
                trapToHype = hdcr.tpmcr;
                break;
              // No default action needed
              default:
                break;
            }
        }
    }
    return trapToHype;
}


bool
mcrMrc14TrapToHyp(const MiscRegIndex miscReg, HCR hcr, CPSR cpsr, SCR scr,
                  HDCR hdcr, HSTR hstr, HCPTR hcptr, uint32_t iss)
{
    bool        isRead;
    uint32_t    crm;
    IntRegIndex rt;
    uint32_t    crn;
    uint32_t    opc1;
    uint32_t    opc2;
    bool        trapToHype = false;

    if (!inSecureState(scr, cpsr) && (cpsr.mode != MODE_HYP)) {
        mcrMrcIssExtract(iss, isRead, crm, rt, crn, opc1, opc2);
        inform("trap check M:%x N:%x 1:%x 2:%x hdcr %x, hcptr %x, hstr %x\n",
                crm, crn, opc1, opc2, hdcr, hcptr, hstr);
        trapToHype  = hdcr.tda  && (opc1 == 0);
        trapToHype |= hcptr.tta && (opc1 == 1);
        if (!trapToHype) {
            switch (unflattenMiscReg(miscReg)) {
              case MISCREG_DBGOSLSR:
              case MISCREG_DBGOSLAR:
              case MISCREG_DBGOSDLR:
              case MISCREG_DBGPRCR:
                trapToHype = hdcr.tdosa;
                break;
              case MISCREG_DBGDRAR:
              case MISCREG_DBGDSAR:
                trapToHype = hdcr.tdra;
                break;
              case MISCREG_JIDR:
                trapToHype = hcr.tid0;
                break;
              case MISCREG_JOSCR:
              case MISCREG_JMCR:
                trapToHype = hstr.tjdbx;
                break;
              case MISCREG_TEECR:
              case MISCREG_TEEHBR:
                trapToHype = hstr.ttee;
                break;
              // No default action needed
              default:
                break;
            }
        }
    }
    return trapToHype;
}

bool
mcrrMrrc15TrapToHyp(const MiscRegIndex miscReg, CPSR cpsr, SCR scr, HSTR hstr,
                    HCR hcr, uint32_t iss)
{
    uint32_t    crm;
    IntRegIndex rt;
    uint32_t    crn;
    uint32_t    opc1;
    uint32_t    opc2;
    bool        isRead;
    bool        trapToHype = false;

    if (!inSecureState(scr, cpsr) && (cpsr.mode != MODE_HYP)) {
        // This is technically the wrong function, but we can re-use it for
        // the moment because we only need one field, which overlaps with the
        // mcrmrc layout
        mcrMrcIssExtract(iss, isRead, crm, rt, crn, opc1, opc2);
        trapToHype = ((uint32_t) hstr) & (1 << crm);

        if (!trapToHype) {
            switch (unflattenMiscReg(miscReg)) {
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
                trapToHype = hcr.tvm & !isRead;
                break;
              // No default action needed
              default:
                break;
            }
        }
    }
    return trapToHype;
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
        switch (sysM)
        {
          case 0xE:
            regIdx = MISCREG_SPSR_FIQ;
            mode   = MODE_FIQ;
            break;
          case 0x10:
            regIdx = MISCREG_SPSR_IRQ;
            mode   = MODE_IRQ;
            break;
          case 0x12:
            regIdx = MISCREG_SPSR_SVC;
            mode   = MODE_SVC;
            break;
          case 0x14:
            regIdx = MISCREG_SPSR_ABT;
            mode   = MODE_ABORT;
            break;
          case 0x16:
            regIdx = MISCREG_SPSR_UND;
            mode   = MODE_UNDEFINED;
            break;
          case 0x1C:
            regIdx = MISCREG_SPSR_MON;
            mode   = MODE_MON;
            break;
          case 0x1E:
            regIdx = MISCREG_SPSR_HYP;
            mode   = MODE_HYP;
            break;
          default:
            ok = false;
            break;
        }
    } else {
        int sysM4To3 = bits(sysM, 4, 3);

        if (sysM4To3 == 0) {
            mode = MODE_USER;
            regIdx = intRegInMode(mode, bits(sysM, 2, 0) + 8);
        } else if (sysM4To3 == 1) {
            mode = MODE_FIQ;
            regIdx = intRegInMode(mode, bits(sysM, 2, 0) + 8);
        } else if (sysM4To3 == 3) {
            if (bits(sysM, 1) == 0) {
                mode = MODE_MON;
                regIdx = intRegInMode(mode, 14 - bits(sysM, 0));
            } else {
                mode = MODE_HYP;
                if (bits(sysM, 0) == 1) {
                    regIdx = intRegInMode(mode, 13); // R13 in HYP
                } else {
                    isIntReg = false;
                    regIdx   = MISCREG_ELR_HYP;
                }
            }
        } else { // Other Banked registers
            int sysM2 = bits(sysM, 2);
            int sysM1 = bits(sysM, 1);

            mode  = (OperatingMode) ( ((sysM2 ||  sysM1) << 0) |
                                      (1                 << 1) |
                                      ((sysM2 && !sysM1) << 2) |
                                      ((sysM2 &&  sysM1) << 3) |
                                      (1                 << 4) );
            regIdx = intRegInMode(mode, 14 - bits(sysM, 0));
            // Don't flatten the register here. This is going to go through
            // setIntReg() which will do the flattening
            ok &= mode != cpsr.mode;
        }
    }

    // Check that the requested register is accessable from the current mode
    if (ok && checkSecurity && mode != cpsr.mode) {
        switch (cpsr.mode)
        {
          case MODE_USER:
            ok = false;
            break;
          case MODE_FIQ:
            ok &=  mode != MODE_HYP;
            ok &= (mode != MODE_MON) || !scr.ns;
            break;
          case MODE_HYP:
            ok &=  mode != MODE_MON;
            ok &= (mode != MODE_FIQ) || !nsacr.rfr;
            break;
          case MODE_IRQ:
          case MODE_SVC:
          case MODE_ABORT:
          case MODE_UNDEFINED:
          case MODE_SYSTEM:
            ok &=  mode != MODE_HYP;
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
    return (ok);
}

bool
SPAlignmentCheckEnabled(ThreadContext* tc)
{
    switch (opModeToEL(currOpMode(tc))) {
      case EL3:
        return ((SCTLR) tc->readMiscReg(MISCREG_SCTLR_EL3)).sa;
      case EL2:
        return ((SCTLR) tc->readMiscReg(MISCREG_SCTLR_EL2)).sa;
      case EL1:
        return ((SCTLR) tc->readMiscReg(MISCREG_SCTLR_EL1)).sa;
      case EL0:
        return ((SCTLR) tc->readMiscReg(MISCREG_SCTLR_EL1)).sa0;
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
      case 0x6:
      case 0x7:
        return 48;
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
      default:
        panic("Invalid phys. address range");
    }
}

} // namespace ArmISA
