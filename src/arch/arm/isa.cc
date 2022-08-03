/*
 * Copyright (c) 2010-2022 Arm Limited
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

#include "arch/arm/isa.hh"

#include "arch/arm/decoder.hh"
#include "arch/arm/faults.hh"
#include "arch/arm/htm.hh"
#include "arch/arm/interrupts.hh"
#include "arch/arm/mmu.hh"
#include "arch/arm/pmu.hh"
#include "arch/arm/regs/misc.hh"
#include "arch/arm/self_debug.hh"
#include "arch/arm/system.hh"
#include "arch/arm/utility.hh"
#include "arch/generic/decoder.hh"
#include "base/cprintf.hh"
#include "cpu/base.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/reg_class.hh"
#include "debug/Arm.hh"
#include "debug/LLSC.hh"
#include "debug/MatRegs.hh"
#include "debug/VecPredRegs.hh"
#include "debug/VecRegs.hh"
#include "dev/arm/generic_timer.hh"
#include "dev/arm/gic_v3.hh"
#include "dev/arm/gic_v3_cpu_interface.hh"
#include "params/ArmISA.hh"
#include "sim/faults.hh"
#include "sim/stat_control.hh"
#include "sim/system.hh"

namespace gem5
{

namespace ArmISA
{

namespace
{

/* Not applicable to ARM */
RegClass floatRegClass(FloatRegClass, FloatRegClassName, 0, debug::FloatRegs);

} // anonymous namespace

ISA::ISA(const Params &p) : BaseISA(p), system(NULL),
    _decoderFlavor(p.decoderFlavor), pmu(p.pmu), impdefAsNop(p.impdef_nop)
{
    _regClasses.push_back(&flatIntRegClass);
    _regClasses.push_back(&floatRegClass);
    _regClasses.push_back(&vecRegClass);
    _regClasses.push_back(&vecElemClass);
    _regClasses.push_back(&vecPredRegClass);
    _regClasses.push_back(&matRegClass);
    _regClasses.push_back(&ccRegClass);
    _regClasses.push_back(&miscRegClass);

    miscRegs[MISCREG_SCTLR_RST] = 0;

    // Hook up a dummy device if we haven't been configured with a
    // real PMU. By using a dummy device, we don't need to check that
    // the PMU exist every time we try to access a PMU register.
    if (!pmu)
        pmu = &dummyDevice;

    // Give all ISA devices a pointer to this ISA
    pmu->setISA(this);

    system = dynamic_cast<ArmSystem *>(p.system);

    // Cache system-level properties
    if (FullSystem && system) {
        highestELIs64 = system->highestELIs64();
        haveLargeAsid64 = system->haveLargeAsid64();
        physAddrRange = system->physAddrRange();
        sveVL = system->sveVL();
        smeVL = system->smeVL();

        release = system->releaseFS();
    } else {
        highestELIs64 = true; // ArmSystem::highestELIs64 does the same
        haveLargeAsid64 = false;
        physAddrRange = 32;  // dummy value
        sveVL = p.sve_vl_se;
        smeVL = p.sme_vl_se;

        release = p.release_se;
    }

    selfDebug = new SelfDebug();
    initializeMiscRegMetadata();
    preUnflattenMiscReg();

    clear();
}

void
ISA::clear()
{
    const Params &p(params());

    // Invalidate cached copies of miscregs in the TLBs
    if (tc) {
        getMMUPtr(tc)->invalidateMiscReg();
    }

    SCTLR sctlr_rst = miscRegs[MISCREG_SCTLR_RST];
    memset(miscRegs, 0, sizeof(miscRegs));

    initID32(p);

    // We always initialize AArch64 ID registers even
    // if we are in AArch32. This is done since if we
    // are in SE mode we don't know if our ArmProcess is
    // AArch32 or AArch64
    initID64(p);

    // Start with an event in the mailbox
    miscRegs[MISCREG_SEV_MAILBOX] = 1;

    // Separate Instruction and Data TLBs
    miscRegs[MISCREG_TLBTR] = 1;

    MVFR0 mvfr0 = 0;
    mvfr0.advSimdRegisters = 2;
    mvfr0.singlePrecision = 2;
    mvfr0.doublePrecision = 2;
    mvfr0.vfpExceptionTrapping = 0;
    mvfr0.divide = 1;
    mvfr0.squareRoot = 1;
    mvfr0.shortVectors = 1;
    mvfr0.roundingModes = 1;
    miscRegs[MISCREG_MVFR0] = mvfr0;

    MVFR1 mvfr1 = 0;
    mvfr1.flushToZero = 1;
    mvfr1.defaultNaN = 1;
    mvfr1.advSimdLoadStore = 1;
    mvfr1.advSimdInteger = 1;
    mvfr1.advSimdSinglePrecision = 1;
    mvfr1.advSimdHalfPrecision = 1;
    mvfr1.vfpHalfPrecision = 1;
    miscRegs[MISCREG_MVFR1] = mvfr1;

    // Reset values of PRRR and NMRR are implementation dependent

    // @todo: PRRR and NMRR in secure state?
    miscRegs[MISCREG_PRRR_NS] =
        (1 << 19) | // 19
        (0 << 18) | // 18
        (0 << 17) | // 17
        (1 << 16) | // 16
        (2 << 14) | // 15:14
        (0 << 12) | // 13:12
        (2 << 10) | // 11:10
        (2 << 8)  | // 9:8
        (2 << 6)  | // 7:6
        (2 << 4)  | // 5:4
        (1 << 2)  | // 3:2
        0;          // 1:0

    miscRegs[MISCREG_NMRR_NS] =
        (1 << 30) | // 31:30
        (0 << 26) | // 27:26
        (0 << 24) | // 25:24
        (3 << 22) | // 23:22
        (2 << 20) | // 21:20
        (0 << 18) | // 19:18
        (0 << 16) | // 17:16
        (1 << 14) | // 15:14
        (0 << 12) | // 13:12
        (2 << 10) | // 11:10
        (0 << 8)  | // 9:8
        (3 << 6)  | // 7:6
        (2 << 4)  | // 5:4
        (0 << 2)  | // 3:2
        0;          // 1:0

    if (FullSystem && system->highestELIs64()) {
        // Initialize AArch64 state
        clear64(p);
        return;
    }

    // Initialize AArch32 state...
    clear32(p, sctlr_rst);
}

void
ISA::clear32(const ArmISAParams &p, const SCTLR &sctlr_rst)
{
    CPSR cpsr = 0;
    cpsr.mode = MODE_USER;

    if (FullSystem) {
        miscRegs[MISCREG_MVBAR] = system->resetAddr();
    }

    miscRegs[MISCREG_CPSR] = cpsr;
    updateRegMap(cpsr);

    SCTLR sctlr = 0;
    sctlr.te = (bool) sctlr_rst.te;
    sctlr.nmfi = (bool) sctlr_rst.nmfi;
    sctlr.v = (bool) sctlr_rst.v;
    sctlr.u = 1;
    sctlr.xp = 1;
    sctlr.rao2 = 1;
    sctlr.rao3 = 1;
    sctlr.rao4 = 0xf;  // SCTLR[6:3]
    sctlr.uci = 1;
    sctlr.dze = 1;
    miscRegs[MISCREG_SCTLR_NS] = sctlr;
    miscRegs[MISCREG_SCTLR_RST] = sctlr_rst;
    miscRegs[MISCREG_HCPTR] = 0;

    miscRegs[MISCREG_CPACR] = 0;

    miscRegs[MISCREG_FPSID] = p.fpsid;

    if (release->has(ArmExtension::LPAE)) {
        TTBCR ttbcr = miscRegs[MISCREG_TTBCR_NS];
        ttbcr.eae = 0;
        miscRegs[MISCREG_TTBCR_NS] = ttbcr;
        // Enforce consistency with system-level settings
        miscRegs[MISCREG_ID_MMFR0] = (miscRegs[MISCREG_ID_MMFR0] & ~0xf) | 0x5;
    }

    if (release->has(ArmExtension::SECURITY)) {
        miscRegs[MISCREG_SCTLR_S] = sctlr;
        miscRegs[MISCREG_SCR] = 0;
        miscRegs[MISCREG_VBAR_S] = 0;
    } else {
        // we're always non-secure
        miscRegs[MISCREG_SCR] = 1;
    }

    //XXX We need to initialize the rest of the state.
}

void
ISA::clear64(const ArmISAParams &p)
{
    CPSR cpsr = 0;
    Addr rvbar = system->resetAddr();
    switch (system->highestEL()) {
        // Set initial EL to highest implemented EL using associated stack
        // pointer (SP_ELx); set RVBAR_ELx to implementation defined reset
        // value
      case EL3:
        cpsr.mode = MODE_EL3H;
        miscRegs[MISCREG_RVBAR_EL3] = rvbar;
        break;
      case EL2:
        cpsr.mode = MODE_EL2H;
        miscRegs[MISCREG_RVBAR_EL2] = rvbar;
        break;
      case EL1:
        cpsr.mode = MODE_EL1H;
        miscRegs[MISCREG_RVBAR_EL1] = rvbar;
        break;
      default:
        panic("Invalid highest implemented exception level");
        break;
    }

    // Initialize rest of CPSR
    cpsr.daif = 0xf;  // Mask all interrupts
    cpsr.ss = 0;
    cpsr.il = 0;
    miscRegs[MISCREG_CPSR] = cpsr;
    updateRegMap(cpsr);

    // Initialize other control registers
    miscRegs[MISCREG_MPIDR_EL1] = 0x80000000;
    if (release->has(ArmExtension::SECURITY)) {
        miscRegs[MISCREG_SCTLR_EL3] = 0x30c50830;
        miscRegs[MISCREG_SCR_EL3]   = 0x00000030;  // RES1 fields
    } else if (release->has(ArmExtension::VIRTUALIZATION)) {
        // also  MISCREG_SCTLR_EL2 (by mapping)
        miscRegs[MISCREG_HSCTLR] = 0x30c50830;
    } else {
        // also  MISCREG_SCTLR_EL1 (by mapping)
        miscRegs[MISCREG_SCTLR_NS] = 0x30d00800 | 0x00050030; // RES1 | init
        // Always non-secure
        miscRegs[MISCREG_SCR_EL3] = 1;
    }
}

void
ISA::initID32(const ArmISAParams &p)
{
    // Initialize configurable default values

    uint32_t midr;
    if (p.midr != 0x0)
        midr = p.midr;
    else if (highestELIs64)
        // Cortex-A57 TRM r0p0 MIDR
        midr = 0x410fd070;
    else
        // Cortex-A15 TRM r0p0 MIDR
        midr = 0x410fc0f0;

    miscRegs[MISCREG_MIDR] = midr;
    miscRegs[MISCREG_MIDR_EL1] = midr;
    miscRegs[MISCREG_VPIDR] = midr;

    miscRegs[MISCREG_ID_ISAR0] = p.id_isar0;
    miscRegs[MISCREG_ID_ISAR1] = p.id_isar1;
    miscRegs[MISCREG_ID_ISAR2] = p.id_isar2;
    miscRegs[MISCREG_ID_ISAR3] = p.id_isar3;
    miscRegs[MISCREG_ID_ISAR4] = p.id_isar4;
    miscRegs[MISCREG_ID_ISAR5] = p.id_isar5;
    miscRegs[MISCREG_ID_ISAR6] = p.id_isar6;

    miscRegs[MISCREG_ID_MMFR0] = p.id_mmfr0;
    miscRegs[MISCREG_ID_MMFR1] = p.id_mmfr1;
    miscRegs[MISCREG_ID_MMFR2] = p.id_mmfr2;
    miscRegs[MISCREG_ID_MMFR3] = p.id_mmfr3;
    miscRegs[MISCREG_ID_MMFR4] = p.id_mmfr4;

    /** MISCREG_ID_ISAR5 */
    // Crypto
    miscRegs[MISCREG_ID_ISAR5] = insertBits(
        miscRegs[MISCREG_ID_ISAR5], 19, 4,
        release->has(ArmExtension::CRYPTO) ? 0x1112 : 0x0);
    // RDM
    miscRegs[MISCREG_ID_ISAR5] = insertBits(
        miscRegs[MISCREG_ID_ISAR5], 27, 24,
        release->has(ArmExtension::FEAT_RDM) ? 0x1 : 0x0);
    // FCMA
    miscRegs[MISCREG_ID_ISAR5] = insertBits(
        miscRegs[MISCREG_ID_ISAR5], 31, 28,
        release->has(ArmExtension::FEAT_FCMA) ? 0x1 : 0x0);

    /** ID_ISAR6 */
    miscRegs[MISCREG_ID_ISAR6] = insertBits(
        miscRegs[MISCREG_ID_ISAR6], 3, 0,
        release->has(ArmExtension::FEAT_JSCVT) ? 0x1 : 0x0);
}

void
ISA::initID64(const ArmISAParams &p)
{
    // Initialize configurable id registers
    miscRegs[MISCREG_ID_AA64AFR0_EL1] = p.id_aa64afr0_el1;
    miscRegs[MISCREG_ID_AA64AFR1_EL1] = p.id_aa64afr1_el1;
    miscRegs[MISCREG_ID_AA64DFR0_EL1] =
        (p.id_aa64dfr0_el1 & 0xfffffffffffff0ffULL) |
        (p.pmu ?             0x0000000000000100ULL : 0); // Enable PMUv3

    miscRegs[MISCREG_ID_AA64DFR1_EL1] = p.id_aa64dfr1_el1;
    miscRegs[MISCREG_ID_AA64ISAR0_EL1] = p.id_aa64isar0_el1;
    miscRegs[MISCREG_ID_AA64ISAR1_EL1] = p.id_aa64isar1_el1;
    miscRegs[MISCREG_ID_AA64MMFR0_EL1] = p.id_aa64mmfr0_el1;
    miscRegs[MISCREG_ID_AA64MMFR1_EL1] = p.id_aa64mmfr1_el1;
    miscRegs[MISCREG_ID_AA64MMFR2_EL1] = p.id_aa64mmfr2_el1;

    miscRegs[MISCREG_ID_DFR0_EL1] =
        (p.pmu ? 0x03000000ULL : 0); // Enable PMUv3

    miscRegs[MISCREG_ID_DFR0] = miscRegs[MISCREG_ID_DFR0_EL1];

    // SVE
    miscRegs[MISCREG_ID_AA64ZFR0_EL1] = 0;  // SVEver 0
    if (release->has(ArmExtension::SECURITY)) {
        miscRegs[MISCREG_ZCR_EL3] = sveVL - 1;
    } else if (release->has(ArmExtension::VIRTUALIZATION)) {
        miscRegs[MISCREG_ZCR_EL2] = sveVL - 1;
    } else {
        miscRegs[MISCREG_ZCR_EL1] = sveVL - 1;
    }

    // SME

    // Set up the SME SMIDR
    // [63:32] RES0
    // [31:24] Implementer - default this to Arm Limited
    // [23:16] SMCU Revision - set to 0 as we don't model an SMCU
    // [15]    SMPS - We don't do priorities in gem5, so disable
    // [14:12] RES0
    // [11:0]  Affinity - we implement per-CPU SME, so set to 0 (no SMCU)
    miscRegs[MISCREG_SMIDR_EL1] = 0 | // Affinity
        0 << 15 |                     // SMPS
        0x41 << 24;                   // Implementer

    miscRegs[MISCREG_ID_AA64SMFR0_EL1] = 0;
    miscRegs[MISCREG_ID_AA64SMFR0_EL1] |= 0x1UL << 32; // F32F32
    // The following BF16F32 is actually not implemented due to a lack
    // of BF16 support in gem5's fplib. However, as per the SME spec the
    // _only_ allowed value is 0x1.
    miscRegs[MISCREG_ID_AA64SMFR0_EL1] |= 0x1UL << 34; // BF16F32
    miscRegs[MISCREG_ID_AA64SMFR0_EL1] |= 0x1UL << 35; // F16F32
    miscRegs[MISCREG_ID_AA64SMFR0_EL1] |= 0xFUL << 36; // I8I32
    miscRegs[MISCREG_ID_AA64SMFR0_EL1] |= 0x1UL << 48; // F64F64
    miscRegs[MISCREG_ID_AA64SMFR0_EL1] |= 0xFUL << 52; // I16I64
    miscRegs[MISCREG_ID_AA64SMFR0_EL1] |= 0x0UL << 56; // SMEver
    miscRegs[MISCREG_ID_AA64SMFR0_EL1] |= 0x1UL << 32; // FA64

    // We want to support FEAT_SME_FA64. Therefore, we enable it in all
    // SMCR_ELx registers by default. Runtime software might change this
    // later, but given that gem5 doesn't disable instructions based on
    // this flag we default to the most representative value.
    miscRegs[MISCREG_SMCR_EL3] = 0x1 << 31;
    miscRegs[MISCREG_SMCR_EL2] = 0x1 << 31;
    miscRegs[MISCREG_SMCR_EL1] = 0x1 << 31;

    // Set the vector default vector length
    if (release->has(ArmExtension::SECURITY)) {
        miscRegs[MISCREG_SMCR_EL3] |= ((smeVL - 1) & 0xF);
    } else if (release->has(ArmExtension::VIRTUALIZATION)) {
        miscRegs[MISCREG_SMCR_EL2] |= ((smeVL - 1) & 0xF);
    } else {
        miscRegs[MISCREG_SMCR_EL1] |= ((smeVL - 1) & 0xF);
    }

    // Enforce consistency with system-level settings...

    // EL3
    miscRegs[MISCREG_ID_AA64PFR0_EL1] = insertBits(
        miscRegs[MISCREG_ID_AA64PFR0_EL1], 15, 12,
        release->has(ArmExtension::SECURITY) ? 0x2 : 0x0);
    // EL2
    miscRegs[MISCREG_ID_AA64PFR0_EL1] = insertBits(
        miscRegs[MISCREG_ID_AA64PFR0_EL1], 11, 8,
        release->has(ArmExtension::VIRTUALIZATION) ? 0x2 : 0x0);
    // SVE
    miscRegs[MISCREG_ID_AA64PFR0_EL1] = insertBits(
        miscRegs[MISCREG_ID_AA64PFR0_EL1], 35, 32,
        release->has(ArmExtension::FEAT_SVE) ? 0x1 : 0x0);
    // SME
    miscRegs[MISCREG_ID_AA64PFR1_EL1] = insertBits(
        miscRegs[MISCREG_ID_AA64PFR1_EL1], 27, 24,
        release->has(ArmExtension::FEAT_SME) ? 0x1 : 0x0);
    // SecEL2
    miscRegs[MISCREG_ID_AA64PFR0_EL1] = insertBits(
        miscRegs[MISCREG_ID_AA64PFR0_EL1], 39, 36,
        release->has(ArmExtension::FEAT_SEL2) ? 0x1 : 0x0);

    // Large ASID support
    miscRegs[MISCREG_ID_AA64MMFR0_EL1] = insertBits(
        miscRegs[MISCREG_ID_AA64MMFR0_EL1], 7, 4,
        haveLargeAsid64 ? 0x2 : 0x0);
    // Physical address size
    miscRegs[MISCREG_ID_AA64MMFR0_EL1] = insertBits(
        miscRegs[MISCREG_ID_AA64MMFR0_EL1], 3, 0,
        encodePhysAddrRange64(physAddrRange));

    /** MISCREG_ID_AA64ISAR0_EL1 */
    // Crypto
    miscRegs[MISCREG_ID_AA64ISAR0_EL1] = insertBits(
        miscRegs[MISCREG_ID_AA64ISAR0_EL1], 19, 4,
        release->has(ArmExtension::CRYPTO) ? 0x1112 : 0x0);
    // LSE
    miscRegs[MISCREG_ID_AA64ISAR0_EL1] = insertBits(
        miscRegs[MISCREG_ID_AA64ISAR0_EL1], 23, 20,
        release->has(ArmExtension::FEAT_LSE) ? 0x2 : 0x0);
    // RDM
    miscRegs[MISCREG_ID_AA64ISAR0_EL1] = insertBits(
        miscRegs[MISCREG_ID_AA64ISAR0_EL1], 31, 28,
        release->has(ArmExtension::FEAT_RDM) ? 0x1 : 0x0);

    /** MISCREG_ID_AA64ISAR1_EL1 */
    // PAuth, APA
    miscRegs[MISCREG_ID_AA64ISAR1_EL1] = insertBits(
        miscRegs[MISCREG_ID_AA64ISAR1_EL1], 7, 4,
        release->has(ArmExtension::FEAT_PAuth) ? 0x1 : 0x0);
    // JSCVT
    miscRegs[MISCREG_ID_AA64ISAR1_EL1] = insertBits(
        miscRegs[MISCREG_ID_AA64ISAR1_EL1], 15, 12,
        release->has(ArmExtension::FEAT_JSCVT) ? 0x1 : 0x0);
    // FCMA
    miscRegs[MISCREG_ID_AA64ISAR1_EL1] = insertBits(
        miscRegs[MISCREG_ID_AA64ISAR1_EL1], 19, 16,
        release->has(ArmExtension::FEAT_FCMA) ? 0x1 : 0x0);
    // PAuth, GPA
    miscRegs[MISCREG_ID_AA64ISAR1_EL1] = insertBits(
        miscRegs[MISCREG_ID_AA64ISAR1_EL1], 27, 24,
        release->has(ArmExtension::FEAT_PAuth) ? 0x1 : 0x0);

    /** MISCREG_ID_AA64MMFR1_EL1 */
    // VMID16
    miscRegs[MISCREG_ID_AA64MMFR1_EL1] = insertBits(
        miscRegs[MISCREG_ID_AA64MMFR1_EL1], 7, 4,
        release->has(ArmExtension::FEAT_VMID16) ? 0x2 : 0x0);
    // VHE
    miscRegs[MISCREG_ID_AA64MMFR1_EL1] = insertBits(
        miscRegs[MISCREG_ID_AA64MMFR1_EL1], 11, 8,
        release->has(ArmExtension::FEAT_VHE) ? 0x1 : 0x0);
    // HPDS
    miscRegs[MISCREG_ID_AA64MMFR1_EL1] = insertBits(
        miscRegs[MISCREG_ID_AA64MMFR1_EL1], 15, 12,
        release->has(ArmExtension::FEAT_HPDS) ? 0x1 : 0x0);
    // PAN
    miscRegs[MISCREG_ID_AA64MMFR1_EL1] = insertBits(
        miscRegs[MISCREG_ID_AA64MMFR1_EL1], 23, 20,
        release->has(ArmExtension::FEAT_PAN) ? 0x1 : 0x0);

    /** MISCREG_ID_AA64MMFR2_EL1 */
    // UAO
    miscRegs[MISCREG_ID_AA64MMFR2_EL1] = insertBits(
        miscRegs[MISCREG_ID_AA64MMFR2_EL1], 7, 4,
        release->has(ArmExtension::FEAT_UAO) ? 0x1 : 0x0);
    // LVA
    miscRegs[MISCREG_ID_AA64MMFR2_EL1] = insertBits(
        miscRegs[MISCREG_ID_AA64MMFR2_EL1], 19, 16,
        release->has(ArmExtension::FEAT_LVA) ? 0x1 : 0x0);


    // TME
    miscRegs[MISCREG_ID_AA64ISAR0_EL1] = insertBits(
        miscRegs[MISCREG_ID_AA64ISAR0_EL1], 27, 24,
        release->has(ArmExtension::TME) ? 0x1 : 0x0);
}

void
ISA::startup()
{
    BaseISA::startup();

    if (tc) {
        setupThreadContext();

        if (release->has(ArmExtension::TME)) {
            std::unique_ptr<BaseHTMCheckpoint> cpt(new HTMCheckpoint());
            tc->setHtmCheckpointPtr(std::move(cpt));
        }
    }
}

void
ISA::setupThreadContext()
{
    pmu->setThreadContext(tc);

    if (!system)
        return;

    selfDebug->init(tc);

    if (auto gicv3_ifc = getGICv3CPUInterface(tc); gicv3_ifc) {
        gicv3_ifc->setISA(this);
        gicv3_ifc->setThreadContext(tc);
    }
}

void
ISA::takeOverFrom(ThreadContext *new_tc, ThreadContext *old_tc)
{
    tc = new_tc;
    setupThreadContext();
}

void
ISA::copyRegsFrom(ThreadContext *src)
{
    for (auto &id: flatIntRegClass)
        tc->setReg(id, src->getReg(id));

    for (auto &id: ccRegClass)
        tc->setReg(id, src->getReg(id));

    for (int i = 0; i < NUM_MISCREGS; i++)
        tc->setMiscRegNoEffect(i, src->readMiscRegNoEffect(i));

    ArmISA::VecRegContainer vc;
    for (auto &id: vecRegClass) {
        src->getReg(id, &vc);
        tc->setReg(id, &vc);
    }

    for (auto &id: vecElemClass)
        tc->setReg(id, src->getReg(id));

    ArmISA::MatRegContainer mc;
    for (auto &id: matRegClass) {
        src->getReg(id, &mc);
        tc->setReg(id, &mc);
    }

    // setMiscReg "with effect" will set the misc register mapping correctly.
    // e.g. updateRegMap(val)
    tc->setMiscReg(MISCREG_CPSR, src->readMiscRegNoEffect(MISCREG_CPSR));

    // Copy over the PC State
    tc->pcState(src->pcState());

    // Invalidate the tlb misc register cache
    static_cast<MMU *>(tc->getMMUPtr())->invalidateMiscReg();
}

/**
 * Returns the enconcing equivalent when VHE is implemented and
 * HCR_EL2.E2H is enabled and executing at EL2
 */
int
ISA::redirectRegVHE(int misc_reg)
{
    const HCR hcr = readMiscRegNoEffect(MISCREG_HCR_EL2);
    if (hcr.e2h == 0x0)
        return misc_reg;
    SCR scr = readMiscRegNoEffect(MISCREG_SCR_EL3);
    bool sec_el2 = scr.eel2 && release->has(ArmExtension::FEAT_SEL2);
    switch(misc_reg) {
      case MISCREG_SPSR_EL1:
        return currEL() == EL2 ? MISCREG_SPSR_EL2 : misc_reg;
      case MISCREG_ELR_EL1:
        return currEL() == EL2 ? MISCREG_ELR_EL2 : misc_reg;
      case MISCREG_SCTLR_EL1:
        return currEL() == EL2 ? MISCREG_SCTLR_EL2 : misc_reg;
      case MISCREG_CPACR_EL1:
        return currEL() == EL2 ? MISCREG_CPTR_EL2 : misc_reg;
//    case MISCREG_TRFCR_EL1:
//      return currEL() == EL2 ? MISCREG_TRFCR_EL2 : misc_reg;
      case MISCREG_TTBR0_EL1:
        return currEL() == EL2 ? MISCREG_TTBR0_EL2 : misc_reg;
      case MISCREG_TTBR1_EL1:
        return currEL() == EL2 ? MISCREG_TTBR1_EL2 : misc_reg;
      case MISCREG_TCR_EL1:
        return currEL() == EL2 ? MISCREG_TCR_EL2 : misc_reg;
      case MISCREG_AFSR0_EL1:
        return currEL() == EL2 ? MISCREG_AFSR0_EL2 : misc_reg;
      case MISCREG_AFSR1_EL1:
        return currEL() == EL2 ? MISCREG_AFSR1_EL2 : misc_reg;
      case MISCREG_ESR_EL1:
        return currEL() == EL2 ? MISCREG_ESR_EL2 : misc_reg;
      case MISCREG_FAR_EL1:
        return currEL() == EL2 ? MISCREG_FAR_EL2 : misc_reg;
      case MISCREG_MAIR_EL1:
        return currEL() == EL2 ? MISCREG_MAIR_EL2 : misc_reg;
      case MISCREG_AMAIR_EL1:
        return currEL() == EL2 ? MISCREG_AMAIR_EL2 : misc_reg;
      case MISCREG_VBAR_EL1:
        return currEL() == EL2 ? MISCREG_VBAR_EL2 : misc_reg;
      case MISCREG_CONTEXTIDR_EL1:
        return currEL() == EL2 ? MISCREG_CONTEXTIDR_EL2 : misc_reg;
      case MISCREG_CNTKCTL_EL1:
        return currEL() == EL2 ? MISCREG_CNTHCTL_EL2 : misc_reg;
      case MISCREG_CNTP_TVAL:
      case MISCREG_CNTP_TVAL_EL0:
        if (ELIsInHost(tc, currEL())) {
            return sec_el2 && !scr.ns ? MISCREG_CNTHPS_TVAL_EL2:
                                        MISCREG_CNTHP_TVAL_EL2;
        } else {
            return misc_reg;
        }
      case MISCREG_CNTP_CTL:
      case MISCREG_CNTP_CTL_EL0:
        if (ELIsInHost(tc, currEL())) {
            return sec_el2 && !scr.ns ? MISCREG_CNTHPS_CTL_EL2:
                                        MISCREG_CNTHP_CTL_EL2;
        } else {
            return misc_reg;
        }
      case MISCREG_CNTP_CVAL:
      case MISCREG_CNTP_CVAL_EL0:
        if (ELIsInHost(tc, currEL())) {
            return sec_el2 && !scr.ns ? MISCREG_CNTHPS_CVAL_EL2:
                                        MISCREG_CNTHP_CVAL_EL2;
        } else {
            return misc_reg;
        }
      case MISCREG_CNTV_TVAL:
      case MISCREG_CNTV_TVAL_EL0:
        if (ELIsInHost(tc, currEL())) {
            return sec_el2 && !scr.ns ? MISCREG_CNTHVS_TVAL_EL2:
                                        MISCREG_CNTHV_TVAL_EL2;
        } else {
            return misc_reg;
        }
      case MISCREG_CNTV_CTL:
      case MISCREG_CNTV_CTL_EL0:
        if (ELIsInHost(tc, currEL())) {
            return sec_el2 && !scr.ns ? MISCREG_CNTHVS_CTL_EL2:
                                        MISCREG_CNTHV_CTL_EL2;
        } else {
            return misc_reg;
        }
      case MISCREG_CNTV_CVAL:
      case MISCREG_CNTV_CVAL_EL0:
        if (ELIsInHost(tc, currEL())) {
            return sec_el2 && !scr.ns ? MISCREG_CNTHVS_CVAL_EL2:
                                        MISCREG_CNTHV_CVAL_EL2;
        } else {
            return misc_reg;
        }
      case MISCREG_CNTVCT:
      case MISCREG_CNTVCT_EL0:
        return ELIsInHost(tc, currEL()) ? MISCREG_CNTPCT_EL0 : misc_reg;
      case MISCREG_SCTLR_EL12:
        return MISCREG_SCTLR_EL1;
      case MISCREG_CPACR_EL12:
        return MISCREG_CPACR_EL1;
      case MISCREG_ZCR_EL12:
        return MISCREG_ZCR_EL1;
      case MISCREG_TTBR0_EL12:
        return MISCREG_TTBR0_EL1;
      case MISCREG_TTBR1_EL12:
        return MISCREG_TTBR1_EL1;
      case MISCREG_TCR_EL12:
        return MISCREG_TCR_EL1;
      case MISCREG_SPSR_EL12:
        return MISCREG_SPSR_EL1;
      case MISCREG_ELR_EL12:
        return MISCREG_ELR_EL1;
      case MISCREG_AFSR0_EL12:
        return MISCREG_AFSR0_EL1;
      case MISCREG_AFSR1_EL12:
        return MISCREG_AFSR1_EL1;
      case MISCREG_ESR_EL12:
        return MISCREG_ESR_EL1;
      case MISCREG_FAR_EL12:
        return MISCREG_FAR_EL1;
      case MISCREG_MAIR_EL12:
        return MISCREG_MAIR_EL1;
      case MISCREG_AMAIR_EL12:
        return MISCREG_AMAIR_EL1;
      case MISCREG_VBAR_EL12:
        return MISCREG_VBAR_EL1;
      case MISCREG_CONTEXTIDR_EL12:
        return MISCREG_CONTEXTIDR_EL1;
      case MISCREG_CNTKCTL_EL12:
        return MISCREG_CNTKCTL_EL1;
      // _EL02 registers
      case MISCREG_CNTP_TVAL_EL02:
        return MISCREG_CNTP_TVAL_EL0;
      case MISCREG_CNTP_CTL_EL02:
        return MISCREG_CNTP_CTL_EL0;
      case MISCREG_CNTP_CVAL_EL02:
        return MISCREG_CNTP_CVAL_EL0;
      case MISCREG_CNTV_TVAL_EL02:
        return MISCREG_CNTV_TVAL_EL0;
      case MISCREG_CNTV_CTL_EL02:
        return MISCREG_CNTV_CTL_EL0;
      case MISCREG_CNTV_CVAL_EL02:
        return MISCREG_CNTV_CVAL_EL0;
      default:
        return misc_reg;
    }
}

RegVal
ISA::readMiscRegNoEffect(RegIndex idx) const
{
    assert(idx < NUM_MISCREGS);

    const auto &reg = lookUpMiscReg[idx]; // bit masks
    const auto &map = getMiscIndices(idx);
    int lower = map.first, upper = map.second;
    // NB!: apply architectural masks according to desired register,
    // despite possibly getting value from different (mapped) register.
    auto val = !upper ? miscRegs[lower] : ((miscRegs[lower] & mask(32))
                                          |(miscRegs[upper] << 32));
    if (val & reg.res0()) {
        DPRINTF(MiscRegs, "Reading MiscReg %s with set res0 bits: %#x\n",
                miscRegName[idx], val & reg.res0());
    }
    if ((val & reg.res1()) != reg.res1()) {
        DPRINTF(MiscRegs, "Reading MiscReg %s with clear res1 bits: %#x\n",
                miscRegName[idx], (val & reg.res1()) ^ reg.res1());
    }
    return (val & ~reg.raz()) | reg.rao(); // enforce raz/rao
}


RegVal
ISA::readMiscReg(RegIndex idx)
{
    CPSR cpsr = 0;
    SCR scr = 0;

    if (idx == MISCREG_CPSR) {
        cpsr = miscRegs[idx];
        auto pc = tc->pcState().as<PCState>();
        cpsr.j = pc.jazelle() ? 1 : 0;
        cpsr.t = pc.thumb() ? 1 : 0;
        return cpsr;
    }

#ifndef NDEBUG
    auto& miscreg_info = lookUpMiscReg[idx].info;
    if (!miscreg_info[MISCREG_IMPLEMENTED]) {
        if (miscreg_info[MISCREG_WARN_NOT_FAIL])
            warn("Unimplemented system register %s read.\n",
                 miscRegName[idx]);
        else
            panic("Unimplemented system register %s read.\n",
                  miscRegName[idx]);
    }
#endif
    idx = redirectRegVHE(idx);

    switch (unflattenMiscReg(idx)) {
      case MISCREG_HCR:
      case MISCREG_HCR2:
            if (!release->has(ArmExtension::VIRTUALIZATION))
                return 0;
            break;
      case MISCREG_CPACR:
        {
            const uint32_t ones = (uint32_t)(-1);
            CPACR cpacrMask = 0;
            // Only cp10, cp11, and ase are implemented, nothing else should
            // be readable? (straight copy from the write code)
            cpacrMask.cp10 = ones;
            cpacrMask.cp11 = ones;
            cpacrMask.asedis = ones;

            // Security Extensions may limit the readability of CPACR
            if (release->has(ArmExtension::SECURITY)) {
                scr = readMiscRegNoEffect(MISCREG_SCR_EL3);
                cpsr = readMiscRegNoEffect(MISCREG_CPSR);
                if (scr.ns && (cpsr.mode != MODE_MON) && ELIs32(tc, EL3)) {
                    NSACR nsacr = readMiscRegNoEffect(MISCREG_NSACR);
                    // NB: Skipping the full loop, here
                    if (!nsacr.cp10) cpacrMask.cp10 = 0;
                    if (!nsacr.cp11) cpacrMask.cp11 = 0;
                }
            }
            RegVal val = readMiscRegNoEffect(MISCREG_CPACR);
            val &= cpacrMask;
            DPRINTF(MiscRegs, "Reading misc reg %s: %#x\n",
                    miscRegName[idx], val);
            return val;
        }
      case MISCREG_MPIDR:
      case MISCREG_MPIDR_EL1:
        return readMPIDR(system, tc);
      case MISCREG_VMPIDR:
      case MISCREG_VMPIDR_EL2:
        // top bit defined as RES1
        return readMiscRegNoEffect(idx) | 0x80000000;
      case MISCREG_ID_AFR0: // not implemented, so alias MIDR
      case MISCREG_REVIDR:  // not implemented, so alias MIDR
      case MISCREG_MIDR:
        cpsr = readMiscRegNoEffect(MISCREG_CPSR);
        scr  = readMiscRegNoEffect(MISCREG_SCR_EL3);
        if ((cpsr.mode == MODE_HYP) || isSecure(tc)) {
            return readMiscRegNoEffect(idx);
        } else {
            return readMiscRegNoEffect(MISCREG_VPIDR);
        }
        break;
      case MISCREG_JOSCR: // Jazelle trivial implementation, RAZ/WI
      case MISCREG_JMCR:  // Jazelle trivial implementation, RAZ/WI
      case MISCREG_JIDR:  // Jazelle trivial implementation, RAZ/WI
      case MISCREG_AIDR:  // AUX ID set to 0
      case MISCREG_TCMTR: // No TCM's
        return 0;

      case MISCREG_CLIDR:
        warn_once("The clidr register always reports 0 caches.\n");
        warn_once("clidr LoUIS field of 0b001 to match current "
                  "ARM implementations.\n");
        return 0x00200000;
      case MISCREG_CCSIDR:
        warn_once("The ccsidr register isn't implemented and "
                "always reads as 0.\n");
        break;
      case MISCREG_CTR:                 // AArch32, ARMv7, top bit set
      case MISCREG_CTR_EL0:             // AArch64
        {
            //all caches have the same line size in gem5
            //4 byte words in ARM
            unsigned lineSizeWords =
                tc->getSystemPtr()->cacheLineSize() / 4;
            unsigned log2LineSizeWords = 0;

            while (lineSizeWords >>= 1) {
                ++log2LineSizeWords;
            }

            CTR ctr = 0;
            //log2 of minimun i-cache line size (words)
            ctr.iCacheLineSize = log2LineSizeWords;
            //b11 - gem5 uses pipt
            ctr.l1IndexPolicy = 0x3;
            //log2 of minimum d-cache line size (words)
            ctr.dCacheLineSize = log2LineSizeWords;
            //log2 of max reservation size (words)
            ctr.erg = log2LineSizeWords;
            //log2 of max writeback size (words)
            ctr.cwg = log2LineSizeWords;
            //b100 - gem5 format is ARMv7
            ctr.format = 0x4;

            return ctr;
        }
      case MISCREG_ACTLR:
        warn("Not doing anything for miscreg ACTLR\n");
        break;

      case MISCREG_PMXEVTYPER_PMCCFILTR:
      case MISCREG_PMINTENSET_EL1 ... MISCREG_PMOVSSET_EL0:
      case MISCREG_PMEVCNTR0_EL0 ... MISCREG_PMEVTYPER5_EL0:
      case MISCREG_PMCR ... MISCREG_PMOVSSET:
        return pmu->readMiscReg(idx);

      case MISCREG_CPSR_Q:
        panic("shouldn't be reading this register seperately\n");
      case MISCREG_FPSCR_QC:
        return readMiscRegNoEffect(MISCREG_FPSCR) & ~FpscrQcMask;
      case MISCREG_FPSCR_EXC:
        return readMiscRegNoEffect(MISCREG_FPSCR) & ~FpscrExcMask;
      case MISCREG_FPSR:
        {
            const uint32_t ones = (uint32_t)(-1);
            FPSCR fpscrMask = 0;
            fpscrMask.ioc = ones;
            fpscrMask.dzc = ones;
            fpscrMask.ofc = ones;
            fpscrMask.ufc = ones;
            fpscrMask.ixc = ones;
            fpscrMask.idc = ones;
            fpscrMask.qc = ones;
            fpscrMask.v = ones;
            fpscrMask.c = ones;
            fpscrMask.z = ones;
            fpscrMask.n = ones;
            return readMiscRegNoEffect(MISCREG_FPSCR) & (uint32_t)fpscrMask;
        }
      case MISCREG_FPCR:
        {
            const uint32_t ones = (uint32_t)(-1);
            FPSCR fpscrMask  = 0;
            fpscrMask.len    = ones;
            fpscrMask.fz16   = ones;
            fpscrMask.stride = ones;
            fpscrMask.rMode  = ones;
            fpscrMask.fz     = ones;
            fpscrMask.dn     = ones;
            fpscrMask.ahp    = ones;
            return readMiscRegNoEffect(MISCREG_FPSCR) & (uint32_t)fpscrMask;
        }
      case MISCREG_NZCV:
        {
            CPSR cpsr = 0;
            cpsr.nz   = tc->getReg(cc_reg::Nz);
            cpsr.c    = tc->getReg(cc_reg::C);
            cpsr.v    = tc->getReg(cc_reg::V);
            return cpsr;
        }
      case MISCREG_DAIF:
        {
            CPSR cpsr = 0;
            cpsr.daif = (uint8_t) ((CPSR) miscRegs[MISCREG_CPSR]).daif;
            return cpsr;
        }
      case MISCREG_SP_EL0:
        {
            return tc->getReg(int_reg::Sp0);
        }
      case MISCREG_SP_EL1:
        {
            return tc->getReg(int_reg::Sp1);
        }
      case MISCREG_SP_EL2:
        {
            return tc->getReg(int_reg::Sp2);
        }
      case MISCREG_SPSEL:
        {
            return miscRegs[MISCREG_CPSR] & 0x1;
        }
      case MISCREG_CURRENTEL:
        {
            return miscRegs[MISCREG_CPSR] & 0xc;
        }
      case MISCREG_PAN:
        {
            return miscRegs[MISCREG_CPSR] & 0x400000;
        }
      case MISCREG_UAO:
        {
            return miscRegs[MISCREG_CPSR] & 0x800000;
        }
      case MISCREG_SVCR:
        {
            return miscRegs[MISCREG_SVCR];
        }
      case MISCREG_L2CTLR:
        {
            // mostly unimplemented, just set NumCPUs field from sim and return
            L2CTLR l2ctlr = 0;
            // b00:1CPU to b11:4CPUs
            l2ctlr.numCPUs = tc->getSystemPtr()->threads.size() - 1;
            return l2ctlr;
        }
      case MISCREG_DBGDIDR:
        /* For now just implement the version number.
         * ARMv7, v7.1 Debug architecture (0b0101 --> 0x5)
         */
        return 0x5 << 16;
      case MISCREG_DBGDSCRint:
        return readMiscRegNoEffect(MISCREG_DBGDSCRint);
      case MISCREG_ISR:
      case MISCREG_ISR_EL1:
        {
            auto ic = dynamic_cast<ArmISA::Interrupts *>(
                    tc->getCpuPtr()->getInterruptController(tc->threadId()));
            return ic->getISR(
                readMiscRegNoEffect(MISCREG_HCR_EL2),
                readMiscRegNoEffect(MISCREG_CPSR),
                readMiscRegNoEffect(MISCREG_SCR_EL3));
        }
      case MISCREG_DCZID_EL0:
        return 0x04;  // DC ZVA clear 64-byte chunks
      case MISCREG_HCPTR:
        {
            RegVal val = readMiscRegNoEffect(idx);
            // The trap bit associated with CP14 is defined as RAZ
            val &= ~(1 << 14);
            // If a CP bit in NSACR is 0 then the corresponding bit in
            // HCPTR is RAO/WI
            bool secure_lookup = release->has(ArmExtension::SECURITY) &&
                isSecure(tc);
            if (!secure_lookup) {
                RegVal mask = readMiscRegNoEffect(MISCREG_NSACR);
                val |= (mask ^ 0x7FFF) & 0xBFFF;
            }
            // Set the bits for unimplemented coprocessors to RAO/WI
            val |= 0x33FF;
            return (val);
        }
      case MISCREG_HDFAR: // alias for secure DFAR
        return readMiscRegNoEffect(MISCREG_DFAR_S);
      case MISCREG_HIFAR: // alias for secure IFAR
        return readMiscRegNoEffect(MISCREG_IFAR_S);

      case MISCREG_ID_PFR0:
        // !ThumbEE | !Jazelle | Thumb | ARM
        return 0x00000031;
      case MISCREG_ID_PFR1:
        {   // Timer | Virti | !M Profile | TrustZone | ARMv4
            bool have_timer = (system->getGenericTimer() != nullptr);
            return 0x00000001 |
                (release->has(ArmExtension::SECURITY) ?
                    0x00000010 : 0x0) |
                (release->has(ArmExtension::VIRTUALIZATION) ?
                    0x00001000 : 0x0) |
                (have_timer ? 0x00010000 : 0x0);
        }
      case MISCREG_ID_AA64PFR0_EL1:
        return 0x0000000000000002 | // AArch{64,32} supported at EL0
               0x0000000000000020 | // EL1
               (release->has(ArmExtension::VIRTUALIZATION) ?
                    0x0000000000000200 : 0) | // EL2
               (release->has(ArmExtension::SECURITY) ?
                    0x0000000000002000 : 0) | // EL3
               (release->has(ArmExtension::FEAT_SVE) ?
                    0x0000000100000000 : 0) | // SVE
               (release->has(ArmExtension::FEAT_SEL2) ?
                    0x0000001000000000 : 0) | // SecEL2
               (gicv3CpuInterface     ? 0x0000000001000000 : 0);
      case MISCREG_ID_AA64PFR1_EL1:
        return 0x0 |
               (release->has(ArmExtension::FEAT_SME) ?
                    0x1 << 24 : 0); // SME

      // Generic Timer registers
      case MISCREG_CNTFRQ ... MISCREG_CNTVOFF:
      case MISCREG_CNTFRQ_EL0 ... MISCREG_CNTVOFF_EL2:
        return getGenericTimer().readMiscReg(idx);

      case MISCREG_ICC_AP0R0 ... MISCREG_ICH_LRC15:
      case MISCREG_ICC_PMR_EL1 ... MISCREG_ICC_IGRPEN1_EL3:
      case MISCREG_ICH_AP0R0_EL2 ... MISCREG_ICH_LR15_EL2:
        return getGICv3CPUInterface().readMiscReg(idx);

      default:
        break;

    }
    return readMiscRegNoEffect(idx);
}

void
ISA::setMiscRegNoEffect(RegIndex idx, RegVal val)
{
    assert(idx < NUM_MISCREGS);

    const auto &reg = lookUpMiscReg[idx]; // bit masks
    const auto &map = getMiscIndices(idx);
    int lower = map.first, upper = map.second;

    auto v = (val & ~reg.wi()) | reg.rao();
    if (upper > 0) {
        miscRegs[lower] = bits(v, 31, 0);
        miscRegs[upper] = bits(v, 63, 32);
        DPRINTF(MiscRegs, "Writing MiscReg %s (%d %d:%d) : %#x\n",
                miscRegName[idx], idx, lower, upper, v);
    } else {
        miscRegs[lower] = v;
        DPRINTF(MiscRegs, "Writing MiscReg %s (%d %d) : %#x\n",
                miscRegName[idx], idx, lower, v);
    }
}

void
ISA::setMiscReg(RegIndex idx, RegVal val)
{

    RegVal newVal = val;
    bool secure_lookup;
    SCR scr;

    if (idx == MISCREG_CPSR) {
        updateRegMap(val);


        CPSR old_cpsr = miscRegs[MISCREG_CPSR];
        int old_mode = old_cpsr.mode;
        CPSR cpsr = val;
        if (cpsr.pan != old_cpsr.pan || cpsr.il != old_cpsr.il) {
            getMMUPtr(tc)->invalidateMiscReg();
        }

        DPRINTF(Arm, "Updating CPSR from %#x to %#x f:%d i:%d a:%d mode:%#x\n",
                miscRegs[idx], cpsr, cpsr.f, cpsr.i, cpsr.a, cpsr.mode);
        PCState pc = tc->pcState().as<PCState>();
        pc.nextThumb(cpsr.t);
        pc.nextJazelle(cpsr.j);
        pc.illegalExec(cpsr.il == 1);
        selfDebug->setDebugMask(cpsr.d == 1);

        tc->getDecoderPtr()->as<Decoder>().setSveLen(
                (getCurSveVecLenInBits() >> 7) - 1);

        // Follow slightly different semantics if a CheckerCPU object
        // is connected
        CheckerCPU *checker = tc->getCheckerCpuPtr();
        if (checker) {
            tc->pcStateNoRecord(pc);
        } else {
            tc->pcState(pc);
        }

        setMiscRegNoEffect(idx, newVal);

        if (old_mode != cpsr.mode) {
            getMMUPtr(tc)->invalidateMiscReg();
            if (gicv3CpuInterface) {
                // The assertion and de-assertion of IRQs and FIQs are
                // affected by the current Exception level and Security
                // state of the PE. As part of the Context
                // Synchronization that occurs as the result of taking
                // or returning from an exception, the CPU interface
                // ensures that IRQ and FIQ are both appropriately
                // asserted or deasserted for the Exception level and
                // Security state that the PE is entering.
                static_cast<Gicv3CPUInterface&>(
                    getGICv3CPUInterface()).update();
            }
        }
    } else {
#ifndef NDEBUG
        auto& miscreg_info = lookUpMiscReg[idx].info;
        if (!miscreg_info[MISCREG_IMPLEMENTED]) {
            if (miscreg_info[MISCREG_WARN_NOT_FAIL])
                warn("Unimplemented system register %s write with %#x.\n",
                    miscRegName[idx], val);
            else
                panic("Unimplemented system register %s write with %#x.\n",
                    miscRegName[idx], val);
        }
#endif
        idx = redirectRegVHE(idx);

        switch (unflattenMiscReg(idx)) {
          case MISCREG_CPACR:
            {

                const uint32_t ones = (uint32_t)(-1);
                CPACR cpacrMask = 0;
                // Only cp10, cp11, and ase are implemented, nothing else should
                // be writable
                cpacrMask.cp10 = ones;
                cpacrMask.cp11 = ones;
                cpacrMask.asedis = ones;

                // Security Extensions may limit the writability of CPACR
                if (release->has(ArmExtension::SECURITY)) {
                    scr = readMiscRegNoEffect(MISCREG_SCR_EL3);
                    CPSR cpsr = readMiscRegNoEffect(MISCREG_CPSR);
                    if (scr.ns && (cpsr.mode != MODE_MON) && ELIs32(tc, EL3)) {
                        NSACR nsacr = readMiscRegNoEffect(MISCREG_NSACR);
                        // NB: Skipping the full loop, here
                        if (!nsacr.cp10) cpacrMask.cp10 = 0;
                        if (!nsacr.cp11) cpacrMask.cp11 = 0;
                    }
                }

                RegVal old_val = readMiscRegNoEffect(MISCREG_CPACR);
                newVal &= cpacrMask;
                newVal |= old_val & ~cpacrMask;
                DPRINTF(MiscRegs, "Writing misc reg %s: %#x\n",
                        miscRegName[idx], newVal);
            }
            break;
          case MISCREG_CPACR_EL1:
            {
                const uint32_t ones = (uint32_t)(-1);
                CPACR cpacrMask = 0;
                cpacrMask.tta = ones;
                cpacrMask.fpen = ones;
                if (release->has(ArmExtension::FEAT_SVE)) {
                    cpacrMask.zen = ones;
                }
                if (release->has(ArmExtension::FEAT_SME)) {
                    cpacrMask.smen = ones;
                }
                newVal &= cpacrMask;
                DPRINTF(MiscRegs, "Writing misc reg %s: %#x\n",
                        miscRegName[idx], newVal);
            }
            break;
          case MISCREG_CPTR_EL2:
            {
                const HCR hcr = readMiscRegNoEffect(MISCREG_HCR_EL2);
                const uint32_t ones = (uint32_t)(-1);
                CPTR cptrMask = 0;
                cptrMask.tcpac = ones;
                cptrMask.tta = ones;
                cptrMask.tfp = ones;
                if (release->has(ArmExtension::FEAT_SVE)) {
                    cptrMask.tz = ones;
                    cptrMask.zen = hcr.e2h ? ones : 0;
                }
                if (release->has(ArmExtension::FEAT_SME)) {
                    cptrMask.tsm = ones;
                    cptrMask.smen = hcr.e2h ? ones : 0;
                }
                cptrMask.fpen = hcr.e2h ? ones : 0;
                newVal &= cptrMask;
                cptrMask = 0;
                cptrMask.res1_13_el2 = ones;
                cptrMask.res1_7_0_el2 = ones;
                if (!release->has(ArmExtension::FEAT_SVE)) {
                    cptrMask.res1_8_el2 = ones;
                }
                if (!release->has(ArmExtension::FEAT_SME)) {
                    cptrMask.res1_12_el2 = ones;
                }
                cptrMask.res1_9_el2 = ones;
                newVal |= cptrMask;
                DPRINTF(MiscRegs, "Writing misc reg %s: %#x\n",
                        miscRegName[idx], newVal);
            }
            break;
          case MISCREG_CPTR_EL3:
            {
                const uint32_t ones = (uint32_t)(-1);
                CPTR cptrMask = 0;
                cptrMask.tcpac = ones;
                cptrMask.tta = ones;
                cptrMask.tfp = ones;
                if (release->has(ArmExtension::FEAT_SVE)) {
                    cptrMask.ez = ones;
                }
                if (release->has(ArmExtension::FEAT_SME)) {
                    cptrMask.esm = ones;
                }
                newVal &= cptrMask;
                DPRINTF(MiscRegs, "Writing misc reg %s: %#x\n",
                        miscRegName[idx], newVal);
            }
            break;
          case MISCREG_CSSELR:
            warn_once("The csselr register isn't implemented.\n");
            return;

          case MISCREG_DC_ZVA_Xt:
            warn("Calling DC ZVA! Not Implemeted! Expect WEIRD results\n");
            return;

          case MISCREG_FPSCR:
            {
                const uint32_t ones = (uint32_t)(-1);
                FPSCR fpscrMask = 0;
                fpscrMask.ioc = ones;
                fpscrMask.dzc = ones;
                fpscrMask.ofc = ones;
                fpscrMask.ufc = ones;
                fpscrMask.ixc = ones;
                fpscrMask.idc = ones;
                fpscrMask.ioe = ones;
                fpscrMask.dze = ones;
                fpscrMask.ofe = ones;
                fpscrMask.ufe = ones;
                fpscrMask.ixe = ones;
                fpscrMask.ide = ones;
                fpscrMask.len = ones;
                fpscrMask.fz16 = ones;
                fpscrMask.stride = ones;
                fpscrMask.rMode = ones;
                fpscrMask.fz = ones;
                fpscrMask.dn = ones;
                fpscrMask.ahp = ones;
                fpscrMask.qc = ones;
                fpscrMask.v = ones;
                fpscrMask.c = ones;
                fpscrMask.z = ones;
                fpscrMask.n = ones;
                newVal = (newVal & (uint32_t)fpscrMask) |
                         (readMiscRegNoEffect(MISCREG_FPSCR) &
                          ~(uint32_t)fpscrMask);
                tc->getDecoderPtr()->as<Decoder>().setContext(newVal);
            }
            break;
          case MISCREG_FPSR:
            {
                const uint32_t ones = (uint32_t)(-1);
                FPSCR fpscrMask = 0;
                fpscrMask.ioc = ones;
                fpscrMask.dzc = ones;
                fpscrMask.ofc = ones;
                fpscrMask.ufc = ones;
                fpscrMask.ixc = ones;
                fpscrMask.idc = ones;
                fpscrMask.qc = ones;
                fpscrMask.v = ones;
                fpscrMask.c = ones;
                fpscrMask.z = ones;
                fpscrMask.n = ones;
                newVal = (newVal & (uint32_t)fpscrMask) |
                         (readMiscRegNoEffect(MISCREG_FPSCR) &
                          ~(uint32_t)fpscrMask);
                idx = MISCREG_FPSCR;
            }
            break;
          case MISCREG_FPCR:
            {
                const uint32_t ones = (uint32_t)(-1);
                FPSCR fpscrMask  = 0;
                fpscrMask.len    = ones;
                fpscrMask.fz16   = ones;
                fpscrMask.stride = ones;
                fpscrMask.rMode  = ones;
                fpscrMask.fz     = ones;
                fpscrMask.dn     = ones;
                fpscrMask.ahp    = ones;
                newVal = (newVal & (uint32_t)fpscrMask) |
                         (readMiscRegNoEffect(MISCREG_FPSCR) &
                          ~(uint32_t)fpscrMask);
                idx = MISCREG_FPSCR;
            }
            break;
          case MISCREG_CPSR_Q:
            {
                assert(!(newVal & ~CpsrMaskQ));
                newVal = readMiscRegNoEffect(MISCREG_CPSR) | newVal;
                idx = MISCREG_CPSR;
            }
            break;
          case MISCREG_FPSCR_QC:
            {
                newVal = readMiscRegNoEffect(MISCREG_FPSCR) |
                         (newVal & FpscrQcMask);
                idx = MISCREG_FPSCR;
            }
            break;
          case MISCREG_FPSCR_EXC:
            {
                newVal = readMiscRegNoEffect(MISCREG_FPSCR) |
                         (newVal & FpscrExcMask);
                idx = MISCREG_FPSCR;
            }
            break;
          case MISCREG_FPEXC:
            {
                // vfpv3 architecture, section B.6.1 of DDI04068
                // bit 29 - valid only if fpexc[31] is 0
                const uint32_t fpexcMask = 0x60000000;
                newVal = (newVal & fpexcMask) |
                         (readMiscRegNoEffect(MISCREG_FPEXC) & ~fpexcMask);
            }
            break;
          case MISCREG_HCR2:
                if (!release->has(ArmExtension::VIRTUALIZATION))
                    return;
                break;
          case MISCREG_HCR:
            {
                const HDCR mdcr  = tc->readMiscRegNoEffect(MISCREG_MDCR_EL2);
                selfDebug->setenableTDETGE((HCR)val, mdcr);
                if (!release->has(ArmExtension::VIRTUALIZATION))
                    return;
            }
            break;

          case MISCREG_HDCR:
            {
                const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
                selfDebug->setenableTDETGE(hcr, (HDCR)val);
            }
            break;
          case MISCREG_DBGOSLAR:
            {
                OSL r = tc->readMiscReg(MISCREG_DBGOSLSR);
                const uint32_t temp = (val == 0xC5ACCE55)? 0x1 : 0x0;
                selfDebug->updateOSLock((RegVal) temp);
                r.oslk = bits(temp,0);
                tc->setMiscReg(MISCREG_DBGOSLSR, r);
            }
            break;
          case MISCREG_DBGBCR0:
            selfDebug->updateDBGBCR(0, val);
            break;
          case MISCREG_DBGBCR1:
            selfDebug->updateDBGBCR(1, val);
            break;
          case MISCREG_DBGBCR2:
            selfDebug->updateDBGBCR(2, val);
            break;
          case MISCREG_DBGBCR3:
            selfDebug->updateDBGBCR(3, val);
            break;
          case MISCREG_DBGBCR4:
            selfDebug->updateDBGBCR(4, val);
            break;
          case MISCREG_DBGBCR5:
            selfDebug->updateDBGBCR(5, val);
            break;
          case MISCREG_DBGBCR6:
            selfDebug->updateDBGBCR(6, val);
            break;
          case MISCREG_DBGBCR7:
            selfDebug->updateDBGBCR(7, val);
            break;
          case MISCREG_DBGBCR8:
            selfDebug->updateDBGBCR(8, val);
            break;
          case MISCREG_DBGBCR9:
            selfDebug->updateDBGBCR(9, val);
            break;
          case MISCREG_DBGBCR10:
            selfDebug->updateDBGBCR(10, val);
            break;
          case MISCREG_DBGBCR11:
            selfDebug->updateDBGBCR(11, val);
            break;
          case MISCREG_DBGBCR12:
            selfDebug->updateDBGBCR(12, val);
            break;
          case MISCREG_DBGBCR13:
            selfDebug->updateDBGBCR(13, val);
            break;
          case MISCREG_DBGBCR14:
            selfDebug->updateDBGBCR(14, val);
            break;
          case MISCREG_DBGBCR15:
            selfDebug->updateDBGBCR(15, val);
            break;
          case MISCREG_DBGWCR0:
            selfDebug->updateDBGWCR(0, val);
            break;
          case MISCREG_DBGWCR1:
            selfDebug->updateDBGWCR(1, val);
            break;
          case MISCREG_DBGWCR2:
            selfDebug->updateDBGWCR(2, val);
            break;
          case MISCREG_DBGWCR3:
            selfDebug->updateDBGWCR(3, val);
            break;
          case MISCREG_DBGWCR4:
            selfDebug->updateDBGWCR(4, val);
            break;
          case MISCREG_DBGWCR5:
            selfDebug->updateDBGWCR(5, val);
            break;
          case MISCREG_DBGWCR6:
            selfDebug->updateDBGWCR(6, val);
            break;
          case MISCREG_DBGWCR7:
            selfDebug->updateDBGWCR(7, val);
            break;
          case MISCREG_DBGWCR8:
            selfDebug->updateDBGWCR(8, val);
            break;
          case MISCREG_DBGWCR9:
            selfDebug->updateDBGWCR(9, val);
            break;
          case MISCREG_DBGWCR10:
            selfDebug->updateDBGWCR(10, val);
            break;
          case MISCREG_DBGWCR11:
            selfDebug->updateDBGWCR(11, val);
            break;
          case MISCREG_DBGWCR12:
            selfDebug->updateDBGWCR(12, val);
            break;
          case MISCREG_DBGWCR13:
            selfDebug->updateDBGWCR(13, val);
            break;
          case MISCREG_DBGWCR14:
            selfDebug->updateDBGWCR(14, val);
            break;
          case MISCREG_DBGWCR15:
            selfDebug->updateDBGWCR(15, val);
            break;

          case MISCREG_MDCR_EL2:
            {
                const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
                selfDebug->setenableTDETGE(hcr, (HDCR)val);
            }
            break;
          case MISCREG_SDCR:
          case MISCREG_MDCR_EL3:
            {
                selfDebug->setbSDD(val);
            }
            break;
          case MISCREG_DBGDSCRext:
            {
                selfDebug->setMDBGen(val);
                DBGDS32 r = tc->readMiscReg(MISCREG_DBGDSCRint);
                DBGDS32 v = val;
                r.moe = v.moe;
                r.udccdis = v.udccdis;
                r.mdbgen = v.mdbgen;
                tc->setMiscReg(MISCREG_DBGDSCRint, r);
                r = tc->readMiscReg(MISCREG_DBGDSCRint);
            }

            break;
          case MISCREG_MDSCR_EL1:
            {
                selfDebug->setMDSCRvals(val);
            }
            break;

          case MISCREG_OSLAR_EL1:
            {
                selfDebug->updateOSLock(val);
                OSL r = tc->readMiscReg(MISCREG_OSLSR_EL1);
                r.oslk = bits(val, 0);
                r.oslm_3 = 1;
                tc->setMiscReg(MISCREG_OSLSR_EL1, r);
            }
            break;

          case MISCREG_DBGBCR0_EL1:
            selfDebug->updateDBGBCR(0, val);
            break;
          case MISCREG_DBGBCR1_EL1:
            selfDebug->updateDBGBCR(1, val);
            break;
          case MISCREG_DBGBCR2_EL1:
            selfDebug->updateDBGBCR(2, val);
            break;
          case MISCREG_DBGBCR3_EL1:
            selfDebug->updateDBGBCR(3, val);
            break;
          case MISCREG_DBGBCR4_EL1:
            selfDebug->updateDBGBCR(4, val);
            break;
          case MISCREG_DBGBCR5_EL1:
            selfDebug->updateDBGBCR(5, val);
            break;
          case MISCREG_DBGBCR6_EL1:
            selfDebug->updateDBGBCR(6, val);
            break;
          case MISCREG_DBGBCR7_EL1:
            selfDebug->updateDBGBCR(7, val);
            break;
          case MISCREG_DBGBCR8_EL1:
            selfDebug->updateDBGBCR(8, val);
            break;
          case MISCREG_DBGBCR9_EL1:
            selfDebug->updateDBGBCR(9, val);
            break;
          case MISCREG_DBGBCR10_EL1:
            selfDebug->updateDBGBCR(10, val);
            break;
          case MISCREG_DBGBCR11_EL1:
            selfDebug->updateDBGBCR(11, val);
            break;
          case MISCREG_DBGBCR12_EL1:
            selfDebug->updateDBGBCR(12, val);
            break;
          case MISCREG_DBGBCR13_EL1:
            selfDebug->updateDBGBCR(13, val);
            break;
          case MISCREG_DBGBCR14_EL1:
            selfDebug->updateDBGBCR(14, val);
            break;
          case MISCREG_DBGBCR15_EL1:
            selfDebug->updateDBGBCR(15, val);
            break;
          case MISCREG_DBGWCR0_EL1:
            selfDebug->updateDBGWCR(0, val);
            break;
          case MISCREG_DBGWCR1_EL1:
            selfDebug->updateDBGWCR(1, val);
            break;
          case MISCREG_DBGWCR2_EL1:
            selfDebug->updateDBGWCR(2, val);
            break;
          case MISCREG_DBGWCR3_EL1:
            selfDebug->updateDBGWCR(3, val);
            break;
          case MISCREG_DBGWCR4_EL1:
            selfDebug->updateDBGWCR(4, val);
            break;
          case MISCREG_DBGWCR5_EL1:
            selfDebug->updateDBGWCR(5, val);
            break;
          case MISCREG_DBGWCR6_EL1:
            selfDebug->updateDBGWCR(6, val);
            break;
          case MISCREG_DBGWCR7_EL1:
            selfDebug->updateDBGWCR(7, val);
            break;
          case MISCREG_DBGWCR8_EL1:
            selfDebug->updateDBGWCR(8, val);
            break;
          case MISCREG_DBGWCR9_EL1:
            selfDebug->updateDBGWCR(9, val);
            break;
          case MISCREG_DBGWCR10_EL1:
            selfDebug->updateDBGWCR(10, val);
            break;
          case MISCREG_DBGWCR11_EL1:
            selfDebug->updateDBGWCR(11, val);
            break;
          case MISCREG_DBGWCR12_EL1:
            selfDebug->updateDBGWCR(12, val);
            break;
          case MISCREG_DBGWCR13_EL1:
            selfDebug->updateDBGWCR(13, val);
            break;
          case MISCREG_DBGWCR14_EL1:
            selfDebug->updateDBGWCR(14, val);
            break;
          case MISCREG_DBGWCR15_EL1:
            selfDebug->updateDBGWCR(15, val);
            break;
          case MISCREG_IFSR:
            {
                // ARM ARM (ARM DDI 0406C.b) B4.1.96
                const uint32_t ifsrMask =
                    mask(31, 13) | mask(11, 11) | mask(8, 6);
                newVal = newVal & ~ifsrMask;
            }
            break;
          case MISCREG_DFSR:
            {
                // ARM ARM (ARM DDI 0406C.b) B4.1.52
                const uint32_t dfsrMask = mask(31, 14) | mask(8, 8);
                newVal = newVal & ~dfsrMask;
            }
            break;
          case MISCREG_AMAIR0:
          case MISCREG_AMAIR1:
            {
                // ARM ARM (ARM DDI 0406C.b) B4.1.5
                // Valid only with LPAE
                if (!release->has(ArmExtension::LPAE))
                    return;
                DPRINTF(MiscRegs, "Writing AMAIR: %#x\n", newVal);
            }
            break;
          case MISCREG_SCR:
            getMMUPtr(tc)->invalidateMiscReg();
            break;
          case MISCREG_SCTLR:
            {
                DPRINTF(MiscRegs, "Writing SCTLR: %#x\n", newVal);
                scr = readMiscRegNoEffect(MISCREG_SCR_EL3);

                MiscRegIndex sctlr_idx;
                if (release->has(ArmExtension::SECURITY) &&
                    !highestELIs64 && !scr.ns) {
                    sctlr_idx = MISCREG_SCTLR_S;
                } else {
                    sctlr_idx =  MISCREG_SCTLR_NS;
                }

                SCTLR sctlr = miscRegs[sctlr_idx];
                SCTLR new_sctlr = newVal;
                new_sctlr.nmfi =  ((bool)sctlr.nmfi) &&
                    !release->has(ArmExtension::VIRTUALIZATION);
                miscRegs[sctlr_idx] = (RegVal)new_sctlr;
                getMMUPtr(tc)->invalidateMiscReg();
            }
          case MISCREG_MIDR:
          case MISCREG_ID_PFR0:
          case MISCREG_ID_PFR1:
          case MISCREG_ID_DFR0:
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

          case MISCREG_MPIDR:
          case MISCREG_FPSID:
          case MISCREG_TLBTR:
          case MISCREG_MVFR0:
          case MISCREG_MVFR1:

          case MISCREG_ID_AA64AFR0_EL1:
          case MISCREG_ID_AA64AFR1_EL1:
          case MISCREG_ID_AA64DFR0_EL1:
          case MISCREG_ID_AA64DFR1_EL1:
          case MISCREG_ID_AA64ISAR0_EL1:
          case MISCREG_ID_AA64ISAR1_EL1:
          case MISCREG_ID_AA64MMFR0_EL1:
          case MISCREG_ID_AA64MMFR1_EL1:
          case MISCREG_ID_AA64MMFR2_EL1:
          case MISCREG_ID_AA64PFR0_EL1:
          case MISCREG_ID_AA64PFR1_EL1:
            // ID registers are constants.
            return;

          // TLB Invalidate All
          case MISCREG_ACTLR:
            warn("Not doing anything for write of miscreg ACTLR\n");
            break;

          case MISCREG_PMXEVTYPER_PMCCFILTR:
          case MISCREG_PMINTENSET_EL1 ... MISCREG_PMOVSSET_EL0:
          case MISCREG_PMEVCNTR0_EL0 ... MISCREG_PMEVTYPER5_EL0:
          case MISCREG_PMCR ... MISCREG_PMOVSSET:
            pmu->setMiscReg(idx, newVal);
            break;


          case MISCREG_HSTR: // TJDBX, now redifined to be RES0
            {
                HSTR hstrMask = 0;
                hstrMask.tjdbx = 1;
                newVal &= ~((uint32_t) hstrMask);
                break;
            }
          case MISCREG_HCPTR:
            {
                // If a CP bit in NSACR is 0 then the corresponding bit in
                // HCPTR is RAO/WI. Same applies to NSASEDIS
                secure_lookup = release->has(ArmExtension::SECURITY) &&
                    isSecure(tc);
                if (!secure_lookup) {
                    RegVal oldValue = readMiscRegNoEffect(MISCREG_HCPTR);
                    RegVal mask =
                        (readMiscRegNoEffect(MISCREG_NSACR) ^ 0x7FFF) & 0xBFFF;
                    newVal = (newVal & ~mask) | (oldValue & mask);
                }
                break;
            }
          case MISCREG_HDFAR: // alias for secure DFAR
            idx = MISCREG_DFAR_S;
            break;
          case MISCREG_HIFAR: // alias for secure IFAR
            idx = MISCREG_IFAR_S;
            break;
          case MISCREG_ATS1CPR:
            addressTranslation(MMU::S1CTran, BaseMMU::Read, 0, val);
            return;
          case MISCREG_ATS1CPW:
            addressTranslation(MMU::S1CTran, BaseMMU::Write, 0, val);
            return;
          case MISCREG_ATS1CUR:
            addressTranslation(MMU::S1CTran, BaseMMU::Read,
                MMU::UserMode, val);
            return;
          case MISCREG_ATS1CUW:
            addressTranslation(MMU::S1CTran, BaseMMU::Write,
                MMU::UserMode, val);
            return;
          case MISCREG_ATS12NSOPR:
            if (!release->has(ArmExtension::SECURITY))
                panic("Security Extensions required for ATS12NSOPR");
            addressTranslation(MMU::S1S2NsTran, BaseMMU::Read, 0, val);
            return;
          case MISCREG_ATS12NSOPW:
            if (!release->has(ArmExtension::SECURITY))
                panic("Security Extensions required for ATS12NSOPW");
            addressTranslation(MMU::S1S2NsTran, BaseMMU::Write, 0, val);
            return;
          case MISCREG_ATS12NSOUR:
            if (!release->has(ArmExtension::SECURITY))
                panic("Security Extensions required for ATS12NSOUR");
            addressTranslation(MMU::S1S2NsTran, BaseMMU::Read,
                MMU::UserMode, val);
            return;
          case MISCREG_ATS12NSOUW:
            if (!release->has(ArmExtension::SECURITY))
                panic("Security Extensions required for ATS12NSOUW");
            addressTranslation(MMU::S1S2NsTran, BaseMMU::Write,
                MMU::UserMode, val);
            return;
          case MISCREG_ATS1HR:
            addressTranslation(MMU::HypMode, BaseMMU::Read, 0, val);
            return;
          case MISCREG_ATS1HW:
            addressTranslation(MMU::HypMode, BaseMMU::Write, 0, val);
            return;
          case MISCREG_TTBCR:
            {
                TTBCR ttbcr = readMiscRegNoEffect(MISCREG_TTBCR);
                const uint32_t ones = (uint32_t)(-1);
                TTBCR ttbcrMask = 0;
                TTBCR ttbcrNew = newVal;

                // ARM DDI 0406C.b, ARMv7-32
                ttbcrMask.n = ones; // T0SZ
                if (release->has(ArmExtension::SECURITY)) {
                    ttbcrMask.pd0 = ones;
                    ttbcrMask.pd1 = ones;
                }
                ttbcrMask.epd0 = ones;
                ttbcrMask.irgn0 = ones;
                ttbcrMask.orgn0 = ones;
                ttbcrMask.sh0 = ones;
                ttbcrMask.ps = ones; // T1SZ
                ttbcrMask.a1 = ones;
                ttbcrMask.epd1 = ones;
                ttbcrMask.irgn1 = ones;
                ttbcrMask.orgn1 = ones;
                ttbcrMask.sh1 = ones;
                if (release->has(ArmExtension::LPAE))
                    ttbcrMask.eae = ones;

                if (release->has(ArmExtension::LPAE) && ttbcrNew.eae) {
                    newVal = newVal & ttbcrMask;
                } else {
                    newVal = (newVal & ttbcrMask) | (ttbcr & (~ttbcrMask));
                }
                // Invalidate TLB MiscReg
                getMMUPtr(tc)->invalidateMiscReg();
                break;
            }
          case MISCREG_TTBR0:
          case MISCREG_TTBR1:
            {
                TTBCR ttbcr = readMiscRegNoEffect(MISCREG_TTBCR);
                if (release->has(ArmExtension::LPAE)) {
                    if (ttbcr.eae) {
                        // ARMv7 bit 63-56, 47-40 reserved, UNK/SBZP
                        // ARMv8 AArch32 bit 63-56 only
                        uint64_t ttbrMask = mask(63,56) | mask(47,40);
                        newVal = (newVal & (~ttbrMask));
                    }
                }
                // Invalidate TLB MiscReg
                getMMUPtr(tc)->invalidateMiscReg();
                break;
            }
          case MISCREG_SCTLR_EL1:
          case MISCREG_CONTEXTIDR:
          case MISCREG_PRRR:
          case MISCREG_NMRR:
          case MISCREG_MAIR0:
          case MISCREG_MAIR1:
          case MISCREG_DACR:
          case MISCREG_VTTBR:
          case MISCREG_SCR_EL3:
          case MISCREG_TCR_EL1:
          case MISCREG_TCR_EL2:
          case MISCREG_TCR_EL3:
          case MISCREG_VTCR_EL2:
          case MISCREG_SCTLR_EL2:
          case MISCREG_SCTLR_EL3:
          case MISCREG_HSCTLR:
          case MISCREG_TTBR0_EL1:
          case MISCREG_TTBR1_EL1:
          case MISCREG_TTBR0_EL2:
          case MISCREG_TTBR1_EL2:
          case MISCREG_TTBR0_EL3:
            getMMUPtr(tc)->invalidateMiscReg();
            break;
          case MISCREG_HCR_EL2:
            {
                const HDCR mdcr  = tc->readMiscRegNoEffect(MISCREG_MDCR_EL2);
                selfDebug->setenableTDETGE((HCR)val, mdcr);
                getMMUPtr(tc)->invalidateMiscReg();
            }
            break;
          case MISCREG_NZCV:
            {
                CPSR cpsr = val;

                tc->setReg(cc_reg::Nz, cpsr.nz);
                tc->setReg(cc_reg::C,  cpsr.c);
                tc->setReg(cc_reg::V,  cpsr.v);
            }
            break;
          case MISCREG_DAIF:
            {
                CPSR cpsr = miscRegs[MISCREG_CPSR];
                cpsr.daif = (uint8_t) ((CPSR) newVal).daif;
                newVal = cpsr;
                idx = MISCREG_CPSR;
            }
            break;
          case MISCREG_SP_EL0:
            tc->setReg(int_reg::Sp0, newVal);
            break;
          case MISCREG_SP_EL1:
            tc->setReg(int_reg::Sp1, newVal);
            break;
          case MISCREG_SP_EL2:
            tc->setReg(int_reg::Sp2, newVal);
            break;
          case MISCREG_SPSEL:
            {
                CPSR cpsr = miscRegs[MISCREG_CPSR];
                cpsr.sp = (uint8_t) ((CPSR) newVal).sp;
                newVal = cpsr;
                idx = MISCREG_CPSR;
            }
            break;
          case MISCREG_CURRENTEL:
            {
                CPSR cpsr = miscRegs[MISCREG_CPSR];
                cpsr.el = (uint8_t) ((CPSR) newVal).el;
                newVal = cpsr;
                idx = MISCREG_CPSR;
            }
            break;
          case MISCREG_PAN:
            {
                // PAN is affecting data accesses
                getMMUPtr(tc)->invalidateMiscReg();

                CPSR cpsr = miscRegs[MISCREG_CPSR];
                cpsr.pan = (uint8_t) ((CPSR) newVal).pan;
                newVal = cpsr;
                idx = MISCREG_CPSR;
            }
            break;
          case MISCREG_UAO:
            {
                // UAO is affecting data accesses
                getMMUPtr(tc)->invalidateMiscReg();

                CPSR cpsr = miscRegs[MISCREG_CPSR];
                cpsr.uao = (uint8_t) ((CPSR) newVal).uao;
                newVal = cpsr;
                idx = MISCREG_CPSR;
            }
            break;
          case MISCREG_SVCR:
            {
                SVCR svcr = miscRegs[MISCREG_SVCR];
                SVCR newSvcr = newVal;

                // Don't allow other bits to be set
                svcr.sm = newSvcr.sm;
                svcr.za = newSvcr.za;
                newVal = svcr;
            }
            break;
          case MISCREG_SMPRI_EL1:
            // Only the bottom 4 bits are settable
            newVal = newVal & 0xF;
            break;
          case MISCREG_AT_S1E1R_Xt:
            addressTranslation64(MMU::S1E1Tran, BaseMMU::Read, 0, val);
            return;
          case MISCREG_AT_S1E1W_Xt:
            addressTranslation64(MMU::S1E1Tran, BaseMMU::Write, 0, val);
            return;
          case MISCREG_AT_S1E0R_Xt:
            addressTranslation64(MMU::S1E0Tran, BaseMMU::Read,
                MMU::UserMode, val);
            return;
          case MISCREG_AT_S1E0W_Xt:
            addressTranslation64(MMU::S1E0Tran, BaseMMU::Write,
                MMU::UserMode, val);
            return;
          case MISCREG_AT_S1E2R_Xt:
            addressTranslation64(MMU::S1E2Tran, BaseMMU::Read, 0, val);
            return;
          case MISCREG_AT_S1E2W_Xt:
            addressTranslation64(MMU::S1E2Tran, BaseMMU::Write, 0, val);
            return;
          case MISCREG_AT_S12E1R_Xt:
            addressTranslation64(MMU::S12E1Tran, BaseMMU::Read, 0, val);
            return;
          case MISCREG_AT_S12E1W_Xt:
            addressTranslation64(MMU::S12E1Tran, BaseMMU::Write, 0, val);
            return;
          case MISCREG_AT_S12E0R_Xt:
            addressTranslation64(MMU::S12E0Tran, BaseMMU::Read,
                MMU::UserMode, val);
            return;
          case MISCREG_AT_S12E0W_Xt:
            addressTranslation64(MMU::S12E0Tran, BaseMMU::Write,
                MMU::UserMode, val);
            return;
          case MISCREG_AT_S1E3R_Xt:
            addressTranslation64(MMU::S1E3Tran, BaseMMU::Read, 0, val);
            return;
          case MISCREG_AT_S1E3W_Xt:
            addressTranslation64(MMU::S1E3Tran, BaseMMU::Write, 0, val);
            return;
          case MISCREG_L2CTLR:
            warn("miscreg L2CTLR (%s) written with %#x. ignored...\n",
                 miscRegName[idx], uint32_t(val));
            break;

          // Generic Timer registers
          case MISCREG_CNTFRQ ... MISCREG_CNTVOFF:
          case MISCREG_CNTFRQ_EL0 ... MISCREG_CNTVOFF_EL2:
            getGenericTimer().setMiscReg(idx, newVal);
            break;
          case MISCREG_ICC_AP0R0 ... MISCREG_ICH_LRC15:
          case MISCREG_ICC_PMR_EL1 ... MISCREG_ICC_IGRPEN1_EL3:
          case MISCREG_ICH_AP0R0_EL2 ... MISCREG_ICH_LR15_EL2:
            getGICv3CPUInterface().setMiscReg(idx, newVal);
            return;
          case MISCREG_ZCR_EL3:
          case MISCREG_ZCR_EL2:
          case MISCREG_ZCR_EL1:
            // Set the value here as we need to update the regs before
            // reading them back in getCurSveVecLenInBits to avoid
            // setting stale vector lengths in the decoder.
            setMiscRegNoEffect(idx, newVal);
            tc->getDecoderPtr()->as<Decoder>().setSveLen(
                    (getCurSveVecLenInBits() >> 7) - 1);
            return;
          case MISCREG_SMCR_EL3:
          case MISCREG_SMCR_EL2:
          case MISCREG_SMCR_EL1:
            // Set the value here as we need to update the regs before
            // reading them back in getCurSmeVecLenInBits (not
            // implemented yet) to avoid setting stale vector lengths in
            // the decoder.
            setMiscRegNoEffect(idx, newVal);
            // TODO: set the SME vector length
            return;
        }
        setMiscRegNoEffect(idx, newVal);
    }
}

BaseISADevice &
ISA::getGenericTimer()
{
    // We only need to create an ISA interface the first time we try
    // to access the timer.
    if (timer)
        return *timer.get();

    assert(system);
    GenericTimer *generic_timer(system->getGenericTimer());
    if (!generic_timer) {
        panic("Trying to get a generic timer from a system that hasn't "
              "been configured to use a generic timer.\n");
    }

    timer.reset(new GenericTimerISA(*generic_timer, tc->contextId()));
    timer->setThreadContext(tc);

    return *timer.get();
}

BaseISADevice &
ISA::getGICv3CPUInterface()
{
    if (gicv3CpuInterface)
        return *gicv3CpuInterface.get();

    auto gicv3_ifc = getGICv3CPUInterface(tc);
    panic_if(!gicv3_ifc, "The system does not have a GICv3 irq controller\n");
    gicv3CpuInterface.reset(gicv3_ifc);

    return *gicv3CpuInterface.get();
}

BaseISADevice*
ISA::getGICv3CPUInterface(ThreadContext *tc)
{
    assert(system);
    Gicv3 *gicv3 = dynamic_cast<Gicv3 *>(system->getGIC());
    if (gicv3) {
        return gicv3->getCPUInterface(tc->contextId());
    } else {
        return nullptr;
    }
}

bool
ISA::inSecureState() const
{
    if (!release->has(ArmExtension::SECURITY)) {
        return false;
    }

    SCR scr = miscRegs[MISCREG_SCR];
    CPSR cpsr = miscRegs[MISCREG_CPSR];

    switch ((OperatingMode) (uint8_t) cpsr.mode) {
      case MODE_MON:
      case MODE_EL3T:
      case MODE_EL3H:
        return true;
      case MODE_HYP:
      case MODE_EL2T:
      case MODE_EL2H:
        return false;
      default:
        return !scr.ns;
    }
}

ExceptionLevel
ISA::currEL() const
{
    CPSR cpsr = readMiscRegNoEffect(MISCREG_CPSR);

    return opModeToEL((OperatingMode)(uint8_t)cpsr.mode);
}

unsigned
ISA::getCurSveVecLenInBits() const
{
    if (!FullSystem) {
        return sveVL * 128;
    }

    panic_if(!tc,
             "A ThreadContext is needed to determine the SVE vector length "
             "in full-system mode");

    CPSR cpsr = miscRegs[MISCREG_CPSR];
    ExceptionLevel el = (ExceptionLevel) (uint8_t) cpsr.el;

    unsigned len = 0;

    if (el == EL1 || (el == EL0 && !ELIsInHost(tc, el))) {
        len = static_cast<ZCR>(miscRegs[MISCREG_ZCR_EL1]).len;
    }

    if (el == EL2 || (el == EL0 && ELIsInHost(tc, el))) {
        len = static_cast<ZCR>(miscRegs[MISCREG_ZCR_EL2]).len;
    } else if (release->has(ArmExtension::VIRTUALIZATION) && !isSecure(tc) &&
               (el == EL0 || el == EL1)) {
        len = std::min(
            len,
            static_cast<unsigned>(
                static_cast<ZCR>(miscRegs[MISCREG_ZCR_EL2]).len));
    }

    if (el == EL3) {
        len = static_cast<ZCR>(miscRegs[MISCREG_ZCR_EL3]).len;
    } else if (release->has(ArmExtension::SECURITY)) {
        len = std::min(
            len,
            static_cast<unsigned>(
                static_cast<ZCR>(miscRegs[MISCREG_ZCR_EL3]).len));
    }

    len = std::min(len, sveVL - 1);

    return (len + 1) * 128;
}

void
ISA::serialize(CheckpointOut &cp) const
{
    DPRINTF(Checkpoint, "Serializing Arm Misc Registers\n");
    SERIALIZE_MAPPING(miscRegs, miscRegName, NUM_PHYS_MISCREGS);
}

void
ISA::unserialize(CheckpointIn &cp)
{
    DPRINTF(Checkpoint, "Unserializing Arm Misc Registers\n");
    UNSERIALIZE_MAPPING(miscRegs, miscRegName, NUM_PHYS_MISCREGS);
    CPSR tmp_cpsr = miscRegs[MISCREG_CPSR];
    updateRegMap(tmp_cpsr);
}

void
ISA::addressTranslation64(MMU::ArmTranslationType tran_type,
    BaseMMU::Mode mode, Request::Flags flags, RegVal val)
{
    // If we're in timing mode then doing the translation in
    // functional mode then we're slightly distorting performance
    // results obtained from simulations. The translation should be
    // done in the same mode the core is running in. NOTE: This
    // can't be an atomic translation because that causes problems
    // with unexpected atomic snoop requests.
    warn_once("Doing AT (address translation) in functional mode! Fix Me!\n");

    auto req = std::make_shared<Request>(
        val, 0, flags,  Request::funcRequestorId,
        tc->pcState().instAddr(), tc->contextId());

    Fault fault = getMMUPtr(tc)->translateFunctional(
        req, tc, mode, tran_type);

    PAR par = 0;
    if (fault == NoFault) {
        Addr paddr = req->getPaddr();
        uint64_t attr = getMMUPtr(tc)->getAttr();
        uint64_t attr1 = attr >> 56;
        if (!attr1 || attr1 ==0x44) {
            attr |= 0x100;
            attr &= ~ uint64_t(0x80);
        }
        par = (paddr & mask(47, 12)) | attr;
        DPRINTF(MiscRegs,
              "MISCREG: Translated addr %#x: PAR_EL1: %#xx\n",
              val, par);
    } else {
        ArmFault *arm_fault = static_cast<ArmFault *>(fault.get());
        arm_fault->update(tc);
        // Set fault bit and FSR
        FSR fsr = arm_fault->getFsr(tc);

        par.f = 1; // F bit
        par.fst = fsr.status; // FST
        par.ptw = (arm_fault->iss() >> 7) & 0x1; // S1PTW
        par.s = arm_fault->isStage2() ? 1 : 0; // S

        DPRINTF(MiscRegs,
                "MISCREG: Translated addr %#x fault fsr %#x: PAR: %#x\n",
                val, fsr, par);
    }
    setMiscRegNoEffect(MISCREG_PAR_EL1, par);
    return;
}

void
ISA::addressTranslation(MMU::ArmTranslationType tran_type,
    BaseMMU::Mode mode, Request::Flags flags, RegVal val)
{
    // If we're in timing mode then doing the translation in
    // functional mode then we're slightly distorting performance
    // results obtained from simulations. The translation should be
    // done in the same mode the core is running in. NOTE: This
    // can't be an atomic translation because that causes problems
    // with unexpected atomic snoop requests.
    warn_once("Doing AT (address translation) in functional mode! Fix Me!\n");

    auto req = std::make_shared<Request>(
        val, 0, flags,  Request::funcRequestorId,
        tc->pcState().instAddr(), tc->contextId());

    Fault fault = getMMUPtr(tc)->translateFunctional(
        req, tc, mode, tran_type);

    PAR par = 0;
    if (fault == NoFault) {
        Addr paddr = req->getPaddr();
        TTBCR ttbcr = readMiscRegNoEffect(MISCREG_TTBCR);
        HCR hcr = readMiscRegNoEffect(MISCREG_HCR_EL2);

        uint8_t max_paddr_bit = 0;
        if (release->has(ArmExtension::LPAE) &&
            (ttbcr.eae || tran_type & MMU::HypMode ||
            ((tran_type & MMU::S1S2NsTran) && hcr.vm) )) {

            max_paddr_bit = 39;
        } else {
            max_paddr_bit = 31;
        }

        par = (paddr & mask(max_paddr_bit, 12)) |
            (getMMUPtr(tc)->getAttr());

        DPRINTF(MiscRegs,
               "MISCREG: Translated addr 0x%08x: PAR: 0x%08x\n",
               val, par);
    } else {
        ArmFault *arm_fault = static_cast<ArmFault *>(fault.get());
        arm_fault->update(tc);
        // Set fault bit and FSR
        FSR fsr = arm_fault->getFsr(tc);

        par.f = 0x1; // F bit
        par.lpae = fsr.lpae;
        par.ptw = (arm_fault->iss() >> 7) & 0x1;
        par.s = arm_fault->isStage2() ? 1 : 0;

        if (par.lpae) {
            // LPAE - rearange fault status
            par.fst = fsr.status;
        } else {
            // VMSA - rearange fault status
            par.fs4_0 = fsr.fsLow | (fsr.fsHigh << 5);
            par.fs5 = fsr.ext;
        }
        DPRINTF(MiscRegs,
               "MISCREG: Translated addr 0x%08x fault fsr %#x: PAR: 0x%08x\n",
               val, fsr, par);
    }
    setMiscRegNoEffect(MISCREG_PAR, par);
    return;
}

template <class XC>
static inline void
lockedSnoopHandler(ThreadContext *tc, XC *xc, PacketPtr pkt,
        Addr cacheBlockMask)
{
    // Should only every see invalidations / direct writes
    assert(pkt->isInvalidate() || pkt->isWrite());

    DPRINTF(LLSC, "%s:  handling snoop for address: %#x locked: %d\n",
            tc->getCpuPtr()->name(), pkt->getAddr(),
            xc->readMiscReg(MISCREG_LOCKFLAG));
    if (!xc->readMiscReg(MISCREG_LOCKFLAG))
        return;

    Addr locked_addr = xc->readMiscReg(MISCREG_LOCKADDR) & cacheBlockMask;
    // If no caches are attached, the snoop address always needs to be masked
    Addr snoop_addr = pkt->getAddr() & cacheBlockMask;

    DPRINTF(LLSC, "%s:  handling snoop for address: %#x locked addr: %#x\n",
            tc->getCpuPtr()->name(), snoop_addr, locked_addr);
    if (locked_addr == snoop_addr) {
        DPRINTF(LLSC, "%s: address match, clearing lock and signaling sev\n",
                tc->getCpuPtr()->name());
        xc->setMiscReg(MISCREG_LOCKFLAG, false);
        // Implement ARMv8 WFE/SEV semantics
        sendEvent(tc);
        xc->setMiscReg(MISCREG_SEV_MAILBOX, true);
    }
}

void
ISA::handleLockedSnoop(PacketPtr pkt, Addr cacheBlockMask)
{
    lockedSnoopHandler(tc, tc, pkt, cacheBlockMask);
}

void
ISA::handleLockedSnoop(ExecContext *xc, PacketPtr pkt, Addr cacheBlockMask)
{
    lockedSnoopHandler(xc->tcBase(), xc, pkt, cacheBlockMask);
}

void
ISA::handleLockedRead(const RequestPtr &req)
{
    tc->setMiscReg(MISCREG_LOCKADDR, req->getPaddr());
    tc->setMiscReg(MISCREG_LOCKFLAG, true);
    DPRINTF(LLSC, "%s: Placing address %#x in monitor\n",
            tc->getCpuPtr()->name(), req->getPaddr());
}

void
ISA::handleLockedRead(ExecContext *xc, const RequestPtr &req)
{
    xc->setMiscReg(MISCREG_LOCKADDR, req->getPaddr());
    xc->setMiscReg(MISCREG_LOCKFLAG, true);
    DPRINTF(LLSC, "%s: Placing address %#x in monitor\n",
            xc->tcBase()->getCpuPtr()->name(), req->getPaddr());
}

void
ISA::handleLockedSnoopHit()
{
    DPRINTF(LLSC, "%s:  handling snoop lock hit address: %#x\n",
            tc->getCpuPtr()->name(), tc->readMiscReg(MISCREG_LOCKADDR));
    tc->setMiscReg(MISCREG_LOCKFLAG, false);
    tc->setMiscReg(MISCREG_SEV_MAILBOX, true);
}

void
ISA::handleLockedSnoopHit(ExecContext *xc)
{
    DPRINTF(LLSC, "%s:  handling snoop lock hit address: %#x\n",
            xc->tcBase()->getCpuPtr()->name(),
            xc->readMiscReg(MISCREG_LOCKADDR));
    xc->setMiscReg(MISCREG_LOCKFLAG, false);
    xc->setMiscReg(MISCREG_SEV_MAILBOX, true);
}

template <class XC>
static inline bool
lockedWriteHandler(ThreadContext *tc, XC *xc, const RequestPtr &req,
        Addr cacheBlockMask)
{
    if (req->isSwap())
        return true;

    DPRINTF(LLSC, "Handling locked write for address %#x in monitor.\n",
            req->getPaddr());
    // Verify that the lock flag is still set and the address
    // is correct
    bool lock_flag = xc->readMiscReg(MISCREG_LOCKFLAG);
    Addr lock_addr = xc->readMiscReg(MISCREG_LOCKADDR) & cacheBlockMask;
    if (!lock_flag || (req->getPaddr() & cacheBlockMask) != lock_addr) {
        // Lock flag not set or addr mismatch in CPU;
        // don't even bother sending to memory system
        req->setExtraData(0);
        xc->setMiscReg(MISCREG_LOCKFLAG, false);
        DPRINTF(LLSC, "clearing lock flag in handle locked write\n",
                tc->getCpuPtr()->name());
        // the rest of this code is not architectural;
        // it's just a debugging aid to help detect
        // livelock by warning on long sequences of failed
        // store conditionals
        int stCondFailures = xc->readStCondFailures();
        stCondFailures++;
        xc->setStCondFailures(stCondFailures);
        if (stCondFailures % 100000 == 0) {
            warn("context %d: %d consecutive "
                 "store conditional failures\n",
                 tc->contextId(), stCondFailures);
        }

        // store conditional failed already, so don't issue it to mem
        return false;
    }
    return true;
}

bool
ISA::handleLockedWrite(const RequestPtr &req, Addr cacheBlockMask)
{
    return lockedWriteHandler(tc, tc, req, cacheBlockMask);
}

bool
ISA::handleLockedWrite(ExecContext *xc, const RequestPtr &req,
        Addr cacheBlockMask)
{
    return lockedWriteHandler(xc->tcBase(), xc, req, cacheBlockMask);
}

void
ISA::globalClearExclusive()
{
    // A spinlock would typically include a Wait For Event (WFE) to
    // conserve energy. The ARMv8 architecture specifies that an event
    // is automatically generated when clearing the exclusive monitor
    // to wake up the processor in WFE.
    DPRINTF(LLSC, "Clearing lock and signaling sev\n");
    tc->setMiscReg(MISCREG_LOCKFLAG, false);
    // Implement ARMv8 WFE/SEV semantics
    sendEvent(tc);
    tc->setMiscReg(MISCREG_SEV_MAILBOX, true);
}

void
ISA::globalClearExclusive(ExecContext *xc)
{
    // A spinlock would typically include a Wait For Event (WFE) to
    // conserve energy. The ARMv8 architecture specifies that an event
    // is automatically generated when clearing the exclusive monitor
    // to wake up the processor in WFE.
    DPRINTF(LLSC, "Clearing lock and signaling sev\n");
    xc->setMiscReg(MISCREG_LOCKFLAG, false);
    // Implement ARMv8 WFE/SEV semantics
    sendEvent(xc->tcBase());
    xc->setMiscReg(MISCREG_SEV_MAILBOX, true);
}

} // namespace ArmISA
} // namespace gem5
