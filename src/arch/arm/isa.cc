/*
 * Copyright (c) 2010-2016 ARM Limited
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
 *
 * Authors: Gabe Black
 *          Ali Saidi
 */

#include "arch/arm/isa.hh"
#include "arch/arm/pmu.hh"
#include "arch/arm/system.hh"
#include "arch/arm/tlb.hh"
#include "arch/arm/tlbi_op.hh"
#include "cpu/base.hh"
#include "cpu/checker/cpu.hh"
#include "debug/Arm.hh"
#include "debug/MiscRegs.hh"
#include "dev/arm/generic_timer.hh"
#include "params/ArmISA.hh"
#include "sim/faults.hh"
#include "sim/stat_control.hh"
#include "sim/system.hh"

namespace ArmISA
{

ISA::ISA(Params *p)
    : SimObject(p),
      system(NULL),
      _decoderFlavour(p->decoderFlavour),
      _vecRegRenameMode(p->vecRegRenameMode),
      pmu(p->pmu)
{
    miscRegs[MISCREG_SCTLR_RST] = 0;

    // Hook up a dummy device if we haven't been configured with a
    // real PMU. By using a dummy device, we don't need to check that
    // the PMU exist every time we try to access a PMU register.
    if (!pmu)
        pmu = &dummyDevice;

    // Give all ISA devices a pointer to this ISA
    pmu->setISA(this);

    system = dynamic_cast<ArmSystem *>(p->system);

    // Cache system-level properties
    if (FullSystem && system) {
        highestELIs64 = system->highestELIs64();
        haveSecurity = system->haveSecurity();
        haveLPAE = system->haveLPAE();
        haveVirtualization = system->haveVirtualization();
        haveLargeAsid64 = system->haveLargeAsid64();
        physAddrRange64 = system->physAddrRange64();
    } else {
        highestELIs64 = true; // ArmSystem::highestELIs64 does the same
        haveSecurity = haveLPAE = haveVirtualization = false;
        haveLargeAsid64 = false;
        physAddrRange64 = 32;  // dummy value
    }

    initializeMiscRegMetadata();
    preUnflattenMiscReg();

    clear();
}

std::vector<struct ISA::MiscRegLUTEntry> ISA::lookUpMiscReg(NUM_MISCREGS);

const ArmISAParams *
ISA::params() const
{
    return dynamic_cast<const Params *>(_params);
}

void
ISA::clear()
{
    const Params *p(params());

    SCTLR sctlr_rst = miscRegs[MISCREG_SCTLR_RST];
    memset(miscRegs, 0, sizeof(miscRegs));

    // Initialize configurable default values
    miscRegs[MISCREG_MIDR] = p->midr;
    miscRegs[MISCREG_MIDR_EL1] = p->midr;
    miscRegs[MISCREG_VPIDR] = p->midr;

    if (FullSystem && system->highestELIs64()) {
        // Initialize AArch64 state
        clear64(p);
        return;
    }

    // Initialize AArch32 state...

    CPSR cpsr = 0;
    cpsr.mode = MODE_USER;
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

    miscRegs[MISCREG_CPACR] = 0;

    miscRegs[MISCREG_ID_MMFR0] = p->id_mmfr0;
    miscRegs[MISCREG_ID_MMFR1] = p->id_mmfr1;
    miscRegs[MISCREG_ID_MMFR2] = p->id_mmfr2;
    miscRegs[MISCREG_ID_MMFR3] = p->id_mmfr3;

    miscRegs[MISCREG_ID_ISAR0] = p->id_isar0;
    miscRegs[MISCREG_ID_ISAR1] = p->id_isar1;
    miscRegs[MISCREG_ID_ISAR2] = p->id_isar2;
    miscRegs[MISCREG_ID_ISAR3] = p->id_isar3;
    miscRegs[MISCREG_ID_ISAR4] = p->id_isar4;
    miscRegs[MISCREG_ID_ISAR5] = p->id_isar5;

    miscRegs[MISCREG_FPSID] = p->fpsid;

    if (haveLPAE) {
        TTBCR ttbcr = miscRegs[MISCREG_TTBCR_NS];
        ttbcr.eae = 0;
        miscRegs[MISCREG_TTBCR_NS] = ttbcr;
        // Enforce consistency with system-level settings
        miscRegs[MISCREG_ID_MMFR0] = (miscRegs[MISCREG_ID_MMFR0] & ~0xf) | 0x5;
    }

    if (haveSecurity) {
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
ISA::clear64(const ArmISAParams *p)
{
    CPSR cpsr = 0;
    Addr rvbar = system->resetAddr64();
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
    if (haveSecurity) {
        miscRegs[MISCREG_SCTLR_EL3] = 0x30c50830;
        miscRegs[MISCREG_SCR_EL3]   = 0x00000030;  // RES1 fields
    } else if (haveVirtualization) {
        // also  MISCREG_SCTLR_EL2 (by mapping)
        miscRegs[MISCREG_HSCTLR] = 0x30c50830;
    } else {
        // also  MISCREG_SCTLR_EL1 (by mapping)
        miscRegs[MISCREG_SCTLR_NS] = 0x30d00800 | 0x00050030; // RES1 | init
        // Always non-secure
        miscRegs[MISCREG_SCR_EL3] = 1;
    }

    // Initialize configurable id registers
    miscRegs[MISCREG_ID_AA64AFR0_EL1] = p->id_aa64afr0_el1;
    miscRegs[MISCREG_ID_AA64AFR1_EL1] = p->id_aa64afr1_el1;
    miscRegs[MISCREG_ID_AA64DFR0_EL1] =
        (p->id_aa64dfr0_el1 & 0xfffffffffffff0ffULL) |
        (p->pmu ?             0x0000000000000100ULL : 0); // Enable PMUv3

    miscRegs[MISCREG_ID_AA64DFR1_EL1] = p->id_aa64dfr1_el1;
    miscRegs[MISCREG_ID_AA64ISAR0_EL1] = p->id_aa64isar0_el1;
    miscRegs[MISCREG_ID_AA64ISAR1_EL1] = p->id_aa64isar1_el1;
    miscRegs[MISCREG_ID_AA64MMFR0_EL1] = p->id_aa64mmfr0_el1;
    miscRegs[MISCREG_ID_AA64MMFR1_EL1] = p->id_aa64mmfr1_el1;

    miscRegs[MISCREG_ID_DFR0_EL1] =
        (p->pmu ? 0x03000000ULL : 0); // Enable PMUv3

    miscRegs[MISCREG_ID_DFR0] = miscRegs[MISCREG_ID_DFR0_EL1];

    // Enforce consistency with system-level settings...

    // EL3
    miscRegs[MISCREG_ID_AA64PFR0_EL1] = insertBits(
        miscRegs[MISCREG_ID_AA64PFR0_EL1], 15, 12,
        haveSecurity ? 0x2 : 0x0);
    // EL2
    miscRegs[MISCREG_ID_AA64PFR0_EL1] = insertBits(
        miscRegs[MISCREG_ID_AA64PFR0_EL1], 11, 8,
        haveVirtualization ? 0x2 : 0x0);
    // Large ASID support
    miscRegs[MISCREG_ID_AA64MMFR0_EL1] = insertBits(
        miscRegs[MISCREG_ID_AA64MMFR0_EL1], 7, 4,
        haveLargeAsid64 ? 0x2 : 0x0);
    // Physical address size
    miscRegs[MISCREG_ID_AA64MMFR0_EL1] = insertBits(
        miscRegs[MISCREG_ID_AA64MMFR0_EL1], 3, 0,
        encodePhysAddrRange64(physAddrRange64));
}

MiscReg
ISA::readMiscRegNoEffect(int misc_reg) const
{
    assert(misc_reg < NumMiscRegs);

    const auto &reg = lookUpMiscReg[misc_reg]; // bit masks
    const auto &map = getMiscIndices(misc_reg);
    int lower = map.first, upper = map.second;
    // NB!: apply architectural masks according to desired register,
    // despite possibly getting value from different (mapped) register.
    auto val = !upper ? miscRegs[lower] : ((miscRegs[lower] & mask(32))
                                          |(miscRegs[upper] << 32));
    if (val & reg.res0()) {
        DPRINTF(MiscRegs, "Reading MiscReg %s with set res0 bits: %#x\n",
                miscRegName[misc_reg], val & reg.res0());
    }
    if ((val & reg.res1()) != reg.res1()) {
        DPRINTF(MiscRegs, "Reading MiscReg %s with clear res1 bits: %#x\n",
                miscRegName[misc_reg], (val & reg.res1()) ^ reg.res1());
    }
    return (val & ~reg.raz()) | reg.rao(); // enforce raz/rao
}


MiscReg
ISA::readMiscReg(int misc_reg, ThreadContext *tc)
{
    CPSR cpsr = 0;
    PCState pc = 0;
    SCR scr = 0;

    if (misc_reg == MISCREG_CPSR) {
        cpsr = miscRegs[misc_reg];
        pc = tc->pcState();
        cpsr.j = pc.jazelle() ? 1 : 0;
        cpsr.t = pc.thumb() ? 1 : 0;
        return cpsr;
    }

#ifndef NDEBUG
    if (!miscRegInfo[misc_reg][MISCREG_IMPLEMENTED]) {
        if (miscRegInfo[misc_reg][MISCREG_WARN_NOT_FAIL])
            warn("Unimplemented system register %s read.\n",
                 miscRegName[misc_reg]);
        else
            panic("Unimplemented system register %s read.\n",
                  miscRegName[misc_reg]);
    }
#endif

    switch (unflattenMiscReg(misc_reg)) {
      case MISCREG_HCR:
        {
            if (!haveVirtualization)
                return 0;
            else
                return readMiscRegNoEffect(MISCREG_HCR);
        }
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
            if (haveSecurity) {
                scr = readMiscRegNoEffect(MISCREG_SCR);
                cpsr = readMiscRegNoEffect(MISCREG_CPSR);
                if (scr.ns && (cpsr.mode != MODE_MON)) {
                    NSACR nsacr = readMiscRegNoEffect(MISCREG_NSACR);
                    // NB: Skipping the full loop, here
                    if (!nsacr.cp10) cpacrMask.cp10 = 0;
                    if (!nsacr.cp11) cpacrMask.cp11 = 0;
                }
            }
            MiscReg val = readMiscRegNoEffect(MISCREG_CPACR);
            val &= cpacrMask;
            DPRINTF(MiscRegs, "Reading misc reg %s: %#x\n",
                    miscRegName[misc_reg], val);
            return val;
        }
      case MISCREG_MPIDR:
        cpsr = readMiscRegNoEffect(MISCREG_CPSR);
        scr  = readMiscRegNoEffect(MISCREG_SCR);
        if ((cpsr.mode == MODE_HYP) || inSecureState(scr, cpsr)) {
            return getMPIDR(system, tc);
        } else {
            return readMiscReg(MISCREG_VMPIDR, tc);
        }
            break;
      case MISCREG_MPIDR_EL1:
        // @todo in the absence of v8 virtualization support just return MPIDR_EL1
        return getMPIDR(system, tc) & 0xffffffff;
      case MISCREG_VMPIDR:
        // top bit defined as RES1
        return readMiscRegNoEffect(misc_reg) | 0x80000000;
      case MISCREG_ID_AFR0: // not implemented, so alias MIDR
      case MISCREG_REVIDR:  // not implemented, so alias MIDR
      case MISCREG_MIDR:
        cpsr = readMiscRegNoEffect(MISCREG_CPSR);
        scr  = readMiscRegNoEffect(MISCREG_SCR);
        if ((cpsr.mode == MODE_HYP) || inSecureState(scr, cpsr)) {
            return readMiscRegNoEffect(misc_reg);
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
        return pmu->readMiscReg(misc_reg);

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
            fpscrMask.ioe = ones;
            fpscrMask.dze = ones;
            fpscrMask.ofe = ones;
            fpscrMask.ufe = ones;
            fpscrMask.ixe = ones;
            fpscrMask.ide = ones;
            fpscrMask.len    = ones;
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
            cpsr.nz   = tc->readCCReg(CCREG_NZ);
            cpsr.c    = tc->readCCReg(CCREG_C);
            cpsr.v    = tc->readCCReg(CCREG_V);
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
            return tc->readIntReg(INTREG_SP0);
        }
      case MISCREG_SP_EL1:
        {
            return tc->readIntReg(INTREG_SP1);
        }
      case MISCREG_SP_EL2:
        {
            return tc->readIntReg(INTREG_SP2);
        }
      case MISCREG_SPSEL:
        {
            return miscRegs[MISCREG_CPSR] & 0x1;
        }
      case MISCREG_CURRENTEL:
        {
            return miscRegs[MISCREG_CPSR] & 0xc;
        }
      case MISCREG_L2CTLR:
        {
            // mostly unimplemented, just set NumCPUs field from sim and return
            L2CTLR l2ctlr = 0;
            // b00:1CPU to b11:4CPUs
            l2ctlr.numCPUs = tc->getSystemPtr()->numContexts() - 1;
            return l2ctlr;
        }
      case MISCREG_DBGDIDR:
        /* For now just implement the version number.
         * ARMv7, v7.1 Debug architecture (0b0101 --> 0x5)
         */
        return 0x5 << 16;
      case MISCREG_DBGDSCRint:
        return 0;
      case MISCREG_ISR:
        return tc->getCpuPtr()->getInterruptController(tc->threadId())->getISR(
            readMiscRegNoEffect(MISCREG_HCR),
            readMiscRegNoEffect(MISCREG_CPSR),
            readMiscRegNoEffect(MISCREG_SCR));
      case MISCREG_ISR_EL1:
        return tc->getCpuPtr()->getInterruptController(tc->threadId())->getISR(
            readMiscRegNoEffect(MISCREG_HCR_EL2),
            readMiscRegNoEffect(MISCREG_CPSR),
            readMiscRegNoEffect(MISCREG_SCR_EL3));
      case MISCREG_DCZID_EL0:
        return 0x04;  // DC ZVA clear 64-byte chunks
      case MISCREG_HCPTR:
        {
            MiscReg val = readMiscRegNoEffect(misc_reg);
            // The trap bit associated with CP14 is defined as RAZ
            val &= ~(1 << 14);
            // If a CP bit in NSACR is 0 then the corresponding bit in
            // HCPTR is RAO/WI
            bool secure_lookup = haveSecurity &&
                inSecureState(readMiscRegNoEffect(MISCREG_SCR),
                              readMiscRegNoEffect(MISCREG_CPSR));
            if (!secure_lookup) {
                MiscReg mask = readMiscRegNoEffect(MISCREG_NSACR);
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
      case MISCREG_HVBAR: // bottom bits reserved
        return readMiscRegNoEffect(MISCREG_HVBAR) & 0xFFFFFFE0;
      case MISCREG_SCTLR:
        return (readMiscRegNoEffect(misc_reg) & 0x72DD39FF) | 0x00C00818;
      case MISCREG_SCTLR_EL1:
        return (readMiscRegNoEffect(misc_reg) & 0x37DDDBBF) | 0x30D00800;
      case MISCREG_SCTLR_EL2:
      case MISCREG_SCTLR_EL3:
      case MISCREG_HSCTLR:
        return (readMiscRegNoEffect(misc_reg) & 0x32CD183F) | 0x30C50830;

      case MISCREG_ID_PFR0:
        // !ThumbEE | !Jazelle | Thumb | ARM
        return 0x00000031;
      case MISCREG_ID_PFR1:
        {   // Timer | Virti | !M Profile | TrustZone | ARMv4
            bool haveTimer = (system->getGenericTimer() != NULL);
            return 0x00000001
                 | (haveSecurity       ? 0x00000010 : 0x0)
                 | (haveVirtualization ? 0x00001000 : 0x0)
                 | (haveTimer          ? 0x00010000 : 0x0);
        }
      case MISCREG_ID_AA64PFR0_EL1:
        return 0x0000000000000002   // AArch{64,32} supported at EL0
             | 0x0000000000000020                             // EL1
             | (haveVirtualization ? 0x0000000000000200 : 0)  // EL2
             | (haveSecurity       ? 0x0000000000002000 : 0); // EL3
      case MISCREG_ID_AA64PFR1_EL1:
        return 0; // bits [63:0] RES0 (reserved for future use)

      // Generic Timer registers
      case MISCREG_CNTFRQ ... MISCREG_CNTHP_CTL:
      case MISCREG_CNTPCT ... MISCREG_CNTHP_CVAL:
      case MISCREG_CNTKCTL_EL1 ... MISCREG_CNTV_CVAL_EL0:
      case MISCREG_CNTVOFF_EL2 ... MISCREG_CNTPS_CVAL_EL1:
        return getGenericTimer(tc).readMiscReg(misc_reg);

      default:
        break;

    }
    return readMiscRegNoEffect(misc_reg);
}

void
ISA::setMiscRegNoEffect(int misc_reg, const MiscReg &val)
{
    assert(misc_reg < NumMiscRegs);

    const auto &reg = lookUpMiscReg[misc_reg]; // bit masks
    const auto &map = getMiscIndices(misc_reg);
    int lower = map.first, upper = map.second;

    auto v = (val & ~reg.wi()) | reg.rao();
    if (upper > 0) {
        miscRegs[lower] = bits(v, 31, 0);
        miscRegs[upper] = bits(v, 63, 32);
        DPRINTF(MiscRegs, "Writing to misc reg %d (%d:%d) : %#x\n",
                misc_reg, lower, upper, v);
    } else {
        miscRegs[lower] = v;
        DPRINTF(MiscRegs, "Writing to misc reg %d (%d) : %#x\n",
                misc_reg, lower, v);
    }
}

void
ISA::setMiscReg(int misc_reg, const MiscReg &val, ThreadContext *tc)
{

    MiscReg newVal = val;
    bool secure_lookup;
    SCR scr;

    if (misc_reg == MISCREG_CPSR) {
        updateRegMap(val);


        CPSR old_cpsr = miscRegs[MISCREG_CPSR];
        int old_mode = old_cpsr.mode;
        CPSR cpsr = val;
        if (old_mode != cpsr.mode || cpsr.il != old_cpsr.il) {
            getITBPtr(tc)->invalidateMiscReg();
            getDTBPtr(tc)->invalidateMiscReg();
        }

        DPRINTF(Arm, "Updating CPSR from %#x to %#x f:%d i:%d a:%d mode:%#x\n",
                miscRegs[misc_reg], cpsr, cpsr.f, cpsr.i, cpsr.a, cpsr.mode);
        PCState pc = tc->pcState();
        pc.nextThumb(cpsr.t);
        pc.nextJazelle(cpsr.j);

        // Follow slightly different semantics if a CheckerCPU object
        // is connected
        CheckerCPU *checker = tc->getCheckerCpuPtr();
        if (checker) {
            tc->pcStateNoRecord(pc);
        } else {
            tc->pcState(pc);
        }
    } else {
#ifndef NDEBUG
        if (!miscRegInfo[misc_reg][MISCREG_IMPLEMENTED]) {
            if (miscRegInfo[misc_reg][MISCREG_WARN_NOT_FAIL])
                warn("Unimplemented system register %s write with %#x.\n",
                    miscRegName[misc_reg], val);
            else
                panic("Unimplemented system register %s write with %#x.\n",
                    miscRegName[misc_reg], val);
        }
#endif
        switch (unflattenMiscReg(misc_reg)) {
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
                if (haveSecurity) {
                    scr = readMiscRegNoEffect(MISCREG_SCR);
                    CPSR cpsr = readMiscRegNoEffect(MISCREG_CPSR);
                    if (scr.ns && (cpsr.mode != MODE_MON)) {
                        NSACR nsacr = readMiscRegNoEffect(MISCREG_NSACR);
                        // NB: Skipping the full loop, here
                        if (!nsacr.cp10) cpacrMask.cp10 = 0;
                        if (!nsacr.cp11) cpacrMask.cp11 = 0;
                    }
                }

                MiscReg old_val = readMiscRegNoEffect(MISCREG_CPACR);
                newVal &= cpacrMask;
                newVal |= old_val & ~cpacrMask;
                DPRINTF(MiscRegs, "Writing misc reg %s: %#x\n",
                        miscRegName[misc_reg], newVal);
            }
            break;
          case MISCREG_CPACR_EL1:
            {
                const uint32_t ones = (uint32_t)(-1);
                CPACR cpacrMask = 0;
                cpacrMask.tta = ones;
                cpacrMask.fpen = ones;
                newVal &= cpacrMask;
                DPRINTF(MiscRegs, "Writing misc reg %s: %#x\n",
                        miscRegName[misc_reg], newVal);
            }
            break;
          case MISCREG_CPTR_EL2:
            {
                const uint32_t ones = (uint32_t)(-1);
                CPTR cptrMask = 0;
                cptrMask.tcpac = ones;
                cptrMask.tta = ones;
                cptrMask.tfp = ones;
                newVal &= cptrMask;
                cptrMask = 0;
                cptrMask.res1_13_12_el2 = ones;
                cptrMask.res1_9_0_el2 = ones;
                newVal |= cptrMask;
                DPRINTF(MiscRegs, "Writing misc reg %s: %#x\n",
                        miscRegName[misc_reg], newVal);
            }
            break;
          case MISCREG_CPTR_EL3:
            {
                const uint32_t ones = (uint32_t)(-1);
                CPTR cptrMask = 0;
                cptrMask.tcpac = ones;
                cptrMask.tta = ones;
                cptrMask.tfp = ones;
                newVal &= cptrMask;
                DPRINTF(MiscRegs, "Writing misc reg %s: %#x\n",
                        miscRegName[misc_reg], newVal);
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
                tc->getDecoderPtr()->setContext(newVal);
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
                misc_reg = MISCREG_FPSCR;
            }
            break;
          case MISCREG_FPCR:
            {
                const uint32_t ones = (uint32_t)(-1);
                FPSCR fpscrMask  = 0;
                fpscrMask.ioe = ones;
                fpscrMask.dze = ones;
                fpscrMask.ofe = ones;
                fpscrMask.ufe = ones;
                fpscrMask.ixe = ones;
                fpscrMask.ide = ones;
                fpscrMask.len    = ones;
                fpscrMask.stride = ones;
                fpscrMask.rMode  = ones;
                fpscrMask.fz     = ones;
                fpscrMask.dn     = ones;
                fpscrMask.ahp    = ones;
                newVal = (newVal & (uint32_t)fpscrMask) |
                         (readMiscRegNoEffect(MISCREG_FPSCR) &
                          ~(uint32_t)fpscrMask);
                misc_reg = MISCREG_FPSCR;
            }
            break;
          case MISCREG_CPSR_Q:
            {
                assert(!(newVal & ~CpsrMaskQ));
                newVal = readMiscRegNoEffect(MISCREG_CPSR) | newVal;
                misc_reg = MISCREG_CPSR;
            }
            break;
          case MISCREG_FPSCR_QC:
            {
                newVal = readMiscRegNoEffect(MISCREG_FPSCR) |
                         (newVal & FpscrQcMask);
                misc_reg = MISCREG_FPSCR;
            }
            break;
          case MISCREG_FPSCR_EXC:
            {
                newVal = readMiscRegNoEffect(MISCREG_FPSCR) |
                         (newVal & FpscrExcMask);
                misc_reg = MISCREG_FPSCR;
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
          case MISCREG_HCR:
            {
                if (!haveVirtualization)
                    return;
            }
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
                if (!haveLPAE)
                    return;
                DPRINTF(MiscRegs, "Writing AMAIR: %#x\n", newVal);
            }
            break;
          case MISCREG_SCR:
            getITBPtr(tc)->invalidateMiscReg();
            getDTBPtr(tc)->invalidateMiscReg();
            break;
          case MISCREG_SCTLR:
            {
                DPRINTF(MiscRegs, "Writing SCTLR: %#x\n", newVal);
                scr = readMiscRegNoEffect(MISCREG_SCR);
                MiscRegIndex sctlr_idx = (haveSecurity && !scr.ns)
                                         ? MISCREG_SCTLR_S : MISCREG_SCTLR_NS;
                SCTLR sctlr = miscRegs[sctlr_idx];
                SCTLR new_sctlr = newVal;
                new_sctlr.nmfi =  ((bool)sctlr.nmfi) && !haveVirtualization;
                miscRegs[sctlr_idx] = (MiscReg)new_sctlr;
                getITBPtr(tc)->invalidateMiscReg();
                getDTBPtr(tc)->invalidateMiscReg();
            }
          case MISCREG_MIDR:
          case MISCREG_ID_PFR0:
          case MISCREG_ID_PFR1:
          case MISCREG_ID_DFR0:
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
          case MISCREG_ID_AA64PFR0_EL1:
          case MISCREG_ID_AA64PFR1_EL1:
            // ID registers are constants.
            return;

          // TLB Invalidate All
          case MISCREG_TLBIALL: // TLBI all entries, EL0&1,
            {
                assert32(tc);
                scr = readMiscReg(MISCREG_SCR, tc);

                TLBIALL tlbiOp(EL1, haveSecurity && !scr.ns);
                tlbiOp(tc);
                return;
            }
          // TLB Invalidate All, Inner Shareable
          case MISCREG_TLBIALLIS:
            {
                assert32(tc);
                scr = readMiscReg(MISCREG_SCR, tc);

                TLBIALL tlbiOp(EL1, haveSecurity && !scr.ns);
                tlbiOp.broadcast(tc);
                return;
            }
          // Instruction TLB Invalidate All
          case MISCREG_ITLBIALL:
            {
                assert32(tc);
                scr = readMiscReg(MISCREG_SCR, tc);

                ITLBIALL tlbiOp(EL1, haveSecurity && !scr.ns);
                tlbiOp(tc);
                return;
            }
          // Data TLB Invalidate All
          case MISCREG_DTLBIALL:
            {
                assert32(tc);
                scr = readMiscReg(MISCREG_SCR, tc);

                DTLBIALL tlbiOp(EL1, haveSecurity && !scr.ns);
                tlbiOp(tc);
                return;
            }
          // TLB Invalidate by VA
          // mcr tlbimval(is) is invalidating all matching entries
          // regardless of the level of lookup, since in gem5 we cache
          // in the tlb the last level of lookup only.
          case MISCREG_TLBIMVA:
          case MISCREG_TLBIMVAL:
            {
                assert32(tc);
                scr = readMiscReg(MISCREG_SCR, tc);

                TLBIMVA tlbiOp(EL1,
                               haveSecurity && !scr.ns,
                               mbits(newVal, 31, 12),
                               bits(newVal, 7,0));

                tlbiOp(tc);
                return;
            }
          // TLB Invalidate by VA, Inner Shareable
          case MISCREG_TLBIMVAIS:
          case MISCREG_TLBIMVALIS:
            {
                assert32(tc);
                scr = readMiscReg(MISCREG_SCR, tc);

                TLBIMVA tlbiOp(EL1,
                               haveSecurity && !scr.ns,
                               mbits(newVal, 31, 12),
                               bits(newVal, 7,0));

                tlbiOp.broadcast(tc);
                return;
            }
          // TLB Invalidate by ASID match
          case MISCREG_TLBIASID:
            {
                assert32(tc);
                scr = readMiscReg(MISCREG_SCR, tc);

                TLBIASID tlbiOp(EL1,
                                haveSecurity && !scr.ns,
                                bits(newVal, 7,0));

                tlbiOp(tc);
                return;
            }
          // TLB Invalidate by ASID match, Inner Shareable
          case MISCREG_TLBIASIDIS:
            {
                assert32(tc);
                scr = readMiscReg(MISCREG_SCR, tc);

                TLBIASID tlbiOp(EL1,
                                haveSecurity && !scr.ns,
                                bits(newVal, 7,0));

                tlbiOp.broadcast(tc);
                return;
            }
          // mcr tlbimvaal(is) is invalidating all matching entries
          // regardless of the level of lookup, since in gem5 we cache
          // in the tlb the last level of lookup only.
          // TLB Invalidate by VA, All ASID
          case MISCREG_TLBIMVAA:
          case MISCREG_TLBIMVAAL:
            {
                assert32(tc);
                scr = readMiscReg(MISCREG_SCR, tc);

                TLBIMVAA tlbiOp(EL1, haveSecurity && !scr.ns,
                                mbits(newVal, 31,12), false);

                tlbiOp(tc);
                return;
            }
          // TLB Invalidate by VA, All ASID, Inner Shareable
          case MISCREG_TLBIMVAAIS:
          case MISCREG_TLBIMVAALIS:
            {
                assert32(tc);
                scr = readMiscReg(MISCREG_SCR, tc);

                TLBIMVAA tlbiOp(EL1, haveSecurity && !scr.ns,
                                mbits(newVal, 31,12), false);

                tlbiOp.broadcast(tc);
                return;
            }
          // mcr tlbimvalh(is) is invalidating all matching entries
          // regardless of the level of lookup, since in gem5 we cache
          // in the tlb the last level of lookup only.
          // TLB Invalidate by VA, Hyp mode
          case MISCREG_TLBIMVAH:
          case MISCREG_TLBIMVALH:
            {
                assert32(tc);
                scr = readMiscReg(MISCREG_SCR, tc);

                TLBIMVAA tlbiOp(EL1, haveSecurity && !scr.ns,
                                mbits(newVal, 31,12), true);

                tlbiOp(tc);
                return;
            }
          // TLB Invalidate by VA, Hyp mode, Inner Shareable
          case MISCREG_TLBIMVAHIS:
          case MISCREG_TLBIMVALHIS:
            {
                assert32(tc);
                scr = readMiscReg(MISCREG_SCR, tc);

                TLBIMVAA tlbiOp(EL1, haveSecurity && !scr.ns,
                                mbits(newVal, 31,12), true);

                tlbiOp.broadcast(tc);
                return;
            }
          // mcr tlbiipas2l(is) is invalidating all matching entries
          // regardless of the level of lookup, since in gem5 we cache
          // in the tlb the last level of lookup only.
          // TLB Invalidate by Intermediate Physical Address, Stage 2
          case MISCREG_TLBIIPAS2:
          case MISCREG_TLBIIPAS2L:
            {
                assert32(tc);
                scr = readMiscReg(MISCREG_SCR, tc);

                TLBIIPA tlbiOp(EL1,
                               haveSecurity && !scr.ns,
                               static_cast<Addr>(bits(newVal, 35, 0)) << 12);

                tlbiOp(tc);
                return;
            }
          // TLB Invalidate by Intermediate Physical Address, Stage 2,
          // Inner Shareable
          case MISCREG_TLBIIPAS2IS:
          case MISCREG_TLBIIPAS2LIS:
            {
                assert32(tc);
                scr = readMiscReg(MISCREG_SCR, tc);

                TLBIIPA tlbiOp(EL1,
                               haveSecurity && !scr.ns,
                               static_cast<Addr>(bits(newVal, 35, 0)) << 12);

                tlbiOp.broadcast(tc);
                return;
            }
          // Instruction TLB Invalidate by VA
          case MISCREG_ITLBIMVA:
            {
                assert32(tc);
                scr = readMiscReg(MISCREG_SCR, tc);

                ITLBIMVA tlbiOp(EL1,
                                haveSecurity && !scr.ns,
                                mbits(newVal, 31, 12),
                                bits(newVal, 7,0));

                tlbiOp(tc);
                return;
            }
          // Data TLB Invalidate by VA
          case MISCREG_DTLBIMVA:
            {
                assert32(tc);
                scr = readMiscReg(MISCREG_SCR, tc);

                DTLBIMVA tlbiOp(EL1,
                                haveSecurity && !scr.ns,
                                mbits(newVal, 31, 12),
                                bits(newVal, 7,0));

                tlbiOp(tc);
                return;
            }
          // Instruction TLB Invalidate by ASID match
          case MISCREG_ITLBIASID:
            {
                assert32(tc);
                scr = readMiscReg(MISCREG_SCR, tc);

                ITLBIASID tlbiOp(EL1,
                                 haveSecurity && !scr.ns,
                                 bits(newVal, 7,0));

                tlbiOp(tc);
                return;
            }
          // Data TLB Invalidate by ASID match
          case MISCREG_DTLBIASID:
            {
                assert32(tc);
                scr = readMiscReg(MISCREG_SCR, tc);

                DTLBIASID tlbiOp(EL1,
                                 haveSecurity && !scr.ns,
                                 bits(newVal, 7,0));

                tlbiOp(tc);
                return;
            }
          // TLB Invalidate All, Non-Secure Non-Hyp
          case MISCREG_TLBIALLNSNH:
            {
                assert32(tc);

                TLBIALLN tlbiOp(EL1, false);
                tlbiOp(tc);
                return;
            }
          // TLB Invalidate All, Non-Secure Non-Hyp, Inner Shareable
          case MISCREG_TLBIALLNSNHIS:
            {
                assert32(tc);

                TLBIALLN tlbiOp(EL1, false);
                tlbiOp.broadcast(tc);
                return;
            }
          // TLB Invalidate All, Hyp mode
          case MISCREG_TLBIALLH:
            {
                assert32(tc);

                TLBIALLN tlbiOp(EL1, true);
                tlbiOp(tc);
                return;
            }
          // TLB Invalidate All, Hyp mode, Inner Shareable
          case MISCREG_TLBIALLHIS:
            {
                assert32(tc);

                TLBIALLN tlbiOp(EL1, true);
                tlbiOp.broadcast(tc);
                return;
            }
          // AArch64 TLB Invalidate All, EL3
          case MISCREG_TLBI_ALLE3:
            {
                assert64(tc);

                TLBIALL tlbiOp(EL3, true);
                tlbiOp(tc);
                return;
            }
          // AArch64 TLB Invalidate All, EL3, Inner Shareable
          case MISCREG_TLBI_ALLE3IS:
            {
                assert64(tc);

                TLBIALL tlbiOp(EL3, true);
                tlbiOp.broadcast(tc);
                return;
            }
          // @todo: uncomment this to enable Virtualization
          // case MISCREG_TLBI_ALLE2IS:
          // case MISCREG_TLBI_ALLE2:
          // AArch64 TLB Invalidate All, EL1
          case MISCREG_TLBI_ALLE1:
          case MISCREG_TLBI_VMALLE1:
          case MISCREG_TLBI_VMALLS12E1:
            // @todo: handle VMID and stage 2 to enable Virtualization
            {
                assert64(tc);
                scr = readMiscReg(MISCREG_SCR, tc);

                TLBIALL tlbiOp(EL1, haveSecurity && !scr.ns);
                tlbiOp(tc);
                return;
            }
          // AArch64 TLB Invalidate All, EL1, Inner Shareable
          case MISCREG_TLBI_ALLE1IS:
          case MISCREG_TLBI_VMALLE1IS:
          case MISCREG_TLBI_VMALLS12E1IS:
            // @todo: handle VMID and stage 2 to enable Virtualization
            {
                assert64(tc);
                scr = readMiscReg(MISCREG_SCR, tc);

                TLBIALL tlbiOp(EL1, haveSecurity && !scr.ns);
                tlbiOp.broadcast(tc);
                return;
            }
          // VAEx(IS) and VALEx(IS) are the same because TLBs
          // only store entries
          // from the last level of translation table walks
          // @todo: handle VMID to enable Virtualization
          // AArch64 TLB Invalidate by VA, EL3
          case MISCREG_TLBI_VAE3_Xt:
          case MISCREG_TLBI_VALE3_Xt:
            {
                assert64(tc);

                TLBIMVA tlbiOp(EL3, true,
                               static_cast<Addr>(bits(newVal, 43, 0)) << 12,
                               0xbeef);
                tlbiOp(tc);
                return;
            }
          // AArch64 TLB Invalidate by VA, EL3, Inner Shareable
          case MISCREG_TLBI_VAE3IS_Xt:
          case MISCREG_TLBI_VALE3IS_Xt:
            {
                assert64(tc);

                TLBIMVA tlbiOp(EL3, true,
                               static_cast<Addr>(bits(newVal, 43, 0)) << 12,
                               0xbeef);

                tlbiOp.broadcast(tc);
                return;
            }
          // AArch64 TLB Invalidate by VA, EL2
          case MISCREG_TLBI_VAE2_Xt:
          case MISCREG_TLBI_VALE2_Xt:
            {
                assert64(tc);
                scr = readMiscReg(MISCREG_SCR, tc);

                TLBIMVA tlbiOp(EL2, haveSecurity && !scr.ns,
                               static_cast<Addr>(bits(newVal, 43, 0)) << 12,
                               0xbeef);
                tlbiOp(tc);
                return;
            }
          // AArch64 TLB Invalidate by VA, EL2, Inner Shareable
          case MISCREG_TLBI_VAE2IS_Xt:
          case MISCREG_TLBI_VALE2IS_Xt:
            {
                assert64(tc);
                scr = readMiscReg(MISCREG_SCR, tc);

                TLBIMVA tlbiOp(EL2, haveSecurity && !scr.ns,
                               static_cast<Addr>(bits(newVal, 43, 0)) << 12,
                               0xbeef);

                tlbiOp.broadcast(tc);
                return;
            }
          // AArch64 TLB Invalidate by VA, EL1
          case MISCREG_TLBI_VAE1_Xt:
          case MISCREG_TLBI_VALE1_Xt:
            {
                assert64(tc);
                scr = readMiscReg(MISCREG_SCR, tc);
                auto asid = haveLargeAsid64 ? bits(newVal, 63, 48) :
                                              bits(newVal, 55, 48);

                TLBIMVA tlbiOp(EL1, haveSecurity && !scr.ns,
                               static_cast<Addr>(bits(newVal, 43, 0)) << 12,
                               asid);

                tlbiOp(tc);
                return;
            }
          // AArch64 TLB Invalidate by VA, EL1, Inner Shareable
          case MISCREG_TLBI_VAE1IS_Xt:
          case MISCREG_TLBI_VALE1IS_Xt:
            {
                assert64(tc);
                scr = readMiscReg(MISCREG_SCR, tc);
                auto asid = haveLargeAsid64 ? bits(newVal, 63, 48) :
                                              bits(newVal, 55, 48);

                TLBIMVA tlbiOp(EL1, haveSecurity && !scr.ns,
                               static_cast<Addr>(bits(newVal, 43, 0)) << 12,
                               asid);

                tlbiOp.broadcast(tc);
                return;
            }
          // AArch64 TLB Invalidate by ASID, EL1
          // @todo: handle VMID to enable Virtualization
          case MISCREG_TLBI_ASIDE1_Xt:
            {
                assert64(tc);
                scr = readMiscReg(MISCREG_SCR, tc);
                auto asid = haveLargeAsid64 ? bits(newVal, 63, 48) :
                                              bits(newVal, 55, 48);

                TLBIASID tlbiOp(EL1, haveSecurity && !scr.ns, asid);
                tlbiOp(tc);
                return;
            }
          // AArch64 TLB Invalidate by ASID, EL1, Inner Shareable
          case MISCREG_TLBI_ASIDE1IS_Xt:
            {
                assert64(tc);
                scr = readMiscReg(MISCREG_SCR, tc);
                auto asid = haveLargeAsid64 ? bits(newVal, 63, 48) :
                                              bits(newVal, 55, 48);

                TLBIASID tlbiOp(EL1, haveSecurity && !scr.ns, asid);
                tlbiOp.broadcast(tc);
                return;
            }
          // VAAE1(IS) and VAALE1(IS) are the same because TLBs only store
          // entries from the last level of translation table walks
          // AArch64 TLB Invalidate by VA, All ASID, EL1
          case MISCREG_TLBI_VAAE1_Xt:
          case MISCREG_TLBI_VAALE1_Xt:
            {
                assert64(tc);
                scr = readMiscReg(MISCREG_SCR, tc);

                TLBIMVAA tlbiOp(EL1, haveSecurity && !scr.ns,
                    static_cast<Addr>(bits(newVal, 43, 0)) << 12, false);

                tlbiOp(tc);
                return;
            }
          // AArch64 TLB Invalidate by VA, All ASID, EL1, Inner Shareable
          case MISCREG_TLBI_VAAE1IS_Xt:
          case MISCREG_TLBI_VAALE1IS_Xt:
            {
                assert64(tc);
                scr = readMiscReg(MISCREG_SCR, tc);

                TLBIMVAA tlbiOp(EL1, haveSecurity && !scr.ns,
                    static_cast<Addr>(bits(newVal, 43, 0)) << 12, false);

                tlbiOp.broadcast(tc);
                return;
            }
          // AArch64 TLB Invalidate by Intermediate Physical Address,
          // Stage 2, EL1
          case MISCREG_TLBI_IPAS2E1_Xt:
          case MISCREG_TLBI_IPAS2LE1_Xt:
            {
                assert64(tc);
                scr = readMiscReg(MISCREG_SCR, tc);

                TLBIIPA tlbiOp(EL1, haveSecurity && !scr.ns,
                               static_cast<Addr>(bits(newVal, 35, 0)) << 12);

                tlbiOp(tc);
                return;
            }
          // AArch64 TLB Invalidate by Intermediate Physical Address,
          // Stage 2, EL1, Inner Shareable
          case MISCREG_TLBI_IPAS2E1IS_Xt:
          case MISCREG_TLBI_IPAS2LE1IS_Xt:
            {
                assert64(tc);
                scr = readMiscReg(MISCREG_SCR, tc);

                TLBIIPA tlbiOp(EL1, haveSecurity && !scr.ns,
                               static_cast<Addr>(bits(newVal, 35, 0)) << 12);

                tlbiOp.broadcast(tc);
                return;
            }
          case MISCREG_ACTLR:
            warn("Not doing anything for write of miscreg ACTLR\n");
            break;

          case MISCREG_PMXEVTYPER_PMCCFILTR:
          case MISCREG_PMINTENSET_EL1 ... MISCREG_PMOVSSET_EL0:
          case MISCREG_PMEVCNTR0_EL0 ... MISCREG_PMEVTYPER5_EL0:
          case MISCREG_PMCR ... MISCREG_PMOVSSET:
            pmu->setMiscReg(misc_reg, newVal);
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
                secure_lookup = haveSecurity &&
                    inSecureState(readMiscRegNoEffect(MISCREG_SCR),
                                  readMiscRegNoEffect(MISCREG_CPSR));
                if (!secure_lookup) {
                    MiscReg oldValue = readMiscRegNoEffect(MISCREG_HCPTR);
                    MiscReg mask = (readMiscRegNoEffect(MISCREG_NSACR) ^ 0x7FFF) & 0xBFFF;
                    newVal = (newVal & ~mask) | (oldValue & mask);
                }
                break;
            }
          case MISCREG_HDFAR: // alias for secure DFAR
            misc_reg = MISCREG_DFAR_S;
            break;
          case MISCREG_HIFAR: // alias for secure IFAR
            misc_reg = MISCREG_IFAR_S;
            break;
          case MISCREG_ATS1CPR:
          case MISCREG_ATS1CPW:
          case MISCREG_ATS1CUR:
          case MISCREG_ATS1CUW:
          case MISCREG_ATS12NSOPR:
          case MISCREG_ATS12NSOPW:
          case MISCREG_ATS12NSOUR:
          case MISCREG_ATS12NSOUW:
          case MISCREG_ATS1HR:
          case MISCREG_ATS1HW:
            {
              Request::Flags flags = 0;
              BaseTLB::Mode mode = BaseTLB::Read;
              TLB::ArmTranslationType tranType = TLB::NormalTran;
              Fault fault;
              switch(misc_reg) {
                case MISCREG_ATS1CPR:
                  flags    = TLB::MustBeOne;
                  tranType = TLB::S1CTran;
                  mode     = BaseTLB::Read;
                  break;
                case MISCREG_ATS1CPW:
                  flags    = TLB::MustBeOne;
                  tranType = TLB::S1CTran;
                  mode     = BaseTLB::Write;
                  break;
                case MISCREG_ATS1CUR:
                  flags    = TLB::MustBeOne | TLB::UserMode;
                  tranType = TLB::S1CTran;
                  mode     = BaseTLB::Read;
                  break;
                case MISCREG_ATS1CUW:
                  flags    = TLB::MustBeOne | TLB::UserMode;
                  tranType = TLB::S1CTran;
                  mode     = BaseTLB::Write;
                  break;
                case MISCREG_ATS12NSOPR:
                  if (!haveSecurity)
                      panic("Security Extensions required for ATS12NSOPR");
                  flags    = TLB::MustBeOne;
                  tranType = TLB::S1S2NsTran;
                  mode     = BaseTLB::Read;
                  break;
                case MISCREG_ATS12NSOPW:
                  if (!haveSecurity)
                      panic("Security Extensions required for ATS12NSOPW");
                  flags    = TLB::MustBeOne;
                  tranType = TLB::S1S2NsTran;
                  mode     = BaseTLB::Write;
                  break;
                case MISCREG_ATS12NSOUR:
                  if (!haveSecurity)
                      panic("Security Extensions required for ATS12NSOUR");
                  flags    = TLB::MustBeOne | TLB::UserMode;
                  tranType = TLB::S1S2NsTran;
                  mode     = BaseTLB::Read;
                  break;
                case MISCREG_ATS12NSOUW:
                  if (!haveSecurity)
                      panic("Security Extensions required for ATS12NSOUW");
                  flags    = TLB::MustBeOne | TLB::UserMode;
                  tranType = TLB::S1S2NsTran;
                  mode     = BaseTLB::Write;
                  break;
                case MISCREG_ATS1HR: // only really useful from secure mode.
                  flags    = TLB::MustBeOne;
                  tranType = TLB::HypMode;
                  mode     = BaseTLB::Read;
                  break;
                case MISCREG_ATS1HW:
                  flags    = TLB::MustBeOne;
                  tranType = TLB::HypMode;
                  mode     = BaseTLB::Write;
                  break;
              }
              // If we're in timing mode then doing the translation in
              // functional mode then we're slightly distorting performance
              // results obtained from simulations. The translation should be
              // done in the same mode the core is running in. NOTE: This
              // can't be an atomic translation because that causes problems
              // with unexpected atomic snoop requests.
              warn("Translating via MISCREG(%d) in functional mode! Fix Me!\n", misc_reg);
              Request req(0, val, 0, flags,  Request::funcMasterId,
                          tc->pcState().pc(), tc->contextId());
              fault = getDTBPtr(tc)->translateFunctional(
                      &req, tc, mode, tranType);
              TTBCR ttbcr = readMiscRegNoEffect(MISCREG_TTBCR);
              HCR   hcr   = readMiscRegNoEffect(MISCREG_HCR);

              MiscReg newVal;
              if (fault == NoFault) {
                  Addr paddr = req.getPaddr();
                  if (haveLPAE && (ttbcr.eae || tranType & TLB::HypMode ||
                     ((tranType & TLB::S1S2NsTran) && hcr.vm) )) {
                      newVal = (paddr & mask(39, 12)) |
                               (getDTBPtr(tc)->getAttr());
                  } else {
                      newVal = (paddr & 0xfffff000) |
                               (getDTBPtr(tc)->getAttr());
                  }
                  DPRINTF(MiscRegs,
                          "MISCREG: Translated addr 0x%08x: PAR: 0x%08x\n",
                          val, newVal);
              } else {
                  ArmFault *armFault = static_cast<ArmFault *>(fault.get());
                  armFault->update(tc);
                  // Set fault bit and FSR
                  FSR fsr = armFault->getFsr(tc);

                  newVal = ((fsr >> 9) & 1) << 11;
                  if (newVal) {
                    // LPAE - rearange fault status
                    newVal |= ((fsr >>  0) & 0x3f) << 1;
                  } else {
                    // VMSA - rearange fault status
                    newVal |= ((fsr >>  0) & 0xf) << 1;
                    newVal |= ((fsr >> 10) & 0x1) << 5;
                    newVal |= ((fsr >> 12) & 0x1) << 6;
                  }
                  newVal |= 0x1; // F bit
                  newVal |= ((armFault->iss() >> 7) & 0x1) << 8;
                  newVal |= armFault->isStage2() ? 0x200 : 0;
                  DPRINTF(MiscRegs,
                          "MISCREG: Translated addr 0x%08x fault fsr %#x: PAR: 0x%08x\n",
                          val, fsr, newVal);
              }
              setMiscRegNoEffect(MISCREG_PAR, newVal);
              return;
            }
          case MISCREG_TTBCR:
            {
                TTBCR ttbcr = readMiscRegNoEffect(MISCREG_TTBCR);
                const uint32_t ones = (uint32_t)(-1);
                TTBCR ttbcrMask = 0;
                TTBCR ttbcrNew = newVal;

                // ARM DDI 0406C.b, ARMv7-32
                ttbcrMask.n = ones; // T0SZ
                if (haveSecurity) {
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
                if (haveLPAE)
                    ttbcrMask.eae = ones;

                if (haveLPAE && ttbcrNew.eae) {
                    newVal = newVal & ttbcrMask;
                } else {
                    newVal = (newVal & ttbcrMask) | (ttbcr & (~ttbcrMask));
                }
            }
            M5_FALLTHROUGH;
          case MISCREG_TTBR0:
          case MISCREG_TTBR1:
            {
                TTBCR ttbcr = readMiscRegNoEffect(MISCREG_TTBCR);
                if (haveLPAE) {
                    if (ttbcr.eae) {
                        // ARMv7 bit 63-56, 47-40 reserved, UNK/SBZP
                        // ARMv8 AArch32 bit 63-56 only
                        uint64_t ttbrMask = mask(63,56) | mask(47,40);
                        newVal = (newVal & (~ttbrMask));
                    }
                }
            }
            M5_FALLTHROUGH;
          case MISCREG_SCTLR_EL1:
            {
                getITBPtr(tc)->invalidateMiscReg();
                getDTBPtr(tc)->invalidateMiscReg();
                setMiscRegNoEffect(misc_reg, newVal);
            }
            M5_FALLTHROUGH;
          case MISCREG_CONTEXTIDR:
          case MISCREG_PRRR:
          case MISCREG_NMRR:
          case MISCREG_MAIR0:
          case MISCREG_MAIR1:
          case MISCREG_DACR:
          case MISCREG_VTTBR:
          case MISCREG_SCR_EL3:
          case MISCREG_HCR_EL2:
          case MISCREG_TCR_EL1:
          case MISCREG_TCR_EL2:
          case MISCREG_TCR_EL3:
          case MISCREG_SCTLR_EL2:
          case MISCREG_SCTLR_EL3:
          case MISCREG_HSCTLR:
          case MISCREG_TTBR0_EL1:
          case MISCREG_TTBR1_EL1:
          case MISCREG_TTBR0_EL2:
          case MISCREG_TTBR0_EL3:
            getITBPtr(tc)->invalidateMiscReg();
            getDTBPtr(tc)->invalidateMiscReg();
            break;
          case MISCREG_NZCV:
            {
                CPSR cpsr = val;

                tc->setCCReg(CCREG_NZ, cpsr.nz);
                tc->setCCReg(CCREG_C,  cpsr.c);
                tc->setCCReg(CCREG_V,  cpsr.v);
            }
            break;
          case MISCREG_DAIF:
            {
                CPSR cpsr = miscRegs[MISCREG_CPSR];
                cpsr.daif = (uint8_t) ((CPSR) newVal).daif;
                newVal = cpsr;
                misc_reg = MISCREG_CPSR;
            }
            break;
          case MISCREG_SP_EL0:
            tc->setIntReg(INTREG_SP0, newVal);
            break;
          case MISCREG_SP_EL1:
            tc->setIntReg(INTREG_SP1, newVal);
            break;
          case MISCREG_SP_EL2:
            tc->setIntReg(INTREG_SP2, newVal);
            break;
          case MISCREG_SPSEL:
            {
                CPSR cpsr = miscRegs[MISCREG_CPSR];
                cpsr.sp = (uint8_t) ((CPSR) newVal).sp;
                newVal = cpsr;
                misc_reg = MISCREG_CPSR;
            }
            break;
          case MISCREG_CURRENTEL:
            {
                CPSR cpsr = miscRegs[MISCREG_CPSR];
                cpsr.el = (uint8_t) ((CPSR) newVal).el;
                newVal = cpsr;
                misc_reg = MISCREG_CPSR;
            }
            break;
          case MISCREG_AT_S1E1R_Xt:
          case MISCREG_AT_S1E1W_Xt:
          case MISCREG_AT_S1E0R_Xt:
          case MISCREG_AT_S1E0W_Xt:
          case MISCREG_AT_S1E2R_Xt:
          case MISCREG_AT_S1E2W_Xt:
          case MISCREG_AT_S12E1R_Xt:
          case MISCREG_AT_S12E1W_Xt:
          case MISCREG_AT_S12E0R_Xt:
          case MISCREG_AT_S12E0W_Xt:
          case MISCREG_AT_S1E3R_Xt:
          case MISCREG_AT_S1E3W_Xt:
            {
                RequestPtr req = new Request;
                Request::Flags flags = 0;
                BaseTLB::Mode mode = BaseTLB::Read;
                TLB::ArmTranslationType tranType = TLB::NormalTran;
                Fault fault;
                switch(misc_reg) {
                  case MISCREG_AT_S1E1R_Xt:
                    flags    = TLB::MustBeOne;
                    tranType = TLB::S1E1Tran;
                    mode     = BaseTLB::Read;
                    break;
                  case MISCREG_AT_S1E1W_Xt:
                    flags    = TLB::MustBeOne;
                    tranType = TLB::S1E1Tran;
                    mode     = BaseTLB::Write;
                    break;
                  case MISCREG_AT_S1E0R_Xt:
                    flags    = TLB::MustBeOne | TLB::UserMode;
                    tranType = TLB::S1E0Tran;
                    mode     = BaseTLB::Read;
                    break;
                  case MISCREG_AT_S1E0W_Xt:
                    flags    = TLB::MustBeOne | TLB::UserMode;
                    tranType = TLB::S1E0Tran;
                    mode     = BaseTLB::Write;
                    break;
                  case MISCREG_AT_S1E2R_Xt:
                    flags    = TLB::MustBeOne;
                    tranType = TLB::S1E2Tran;
                    mode     = BaseTLB::Read;
                    break;
                  case MISCREG_AT_S1E2W_Xt:
                    flags    = TLB::MustBeOne;
                    tranType = TLB::S1E2Tran;
                    mode     = BaseTLB::Write;
                    break;
                  case MISCREG_AT_S12E0R_Xt:
                    flags    = TLB::MustBeOne | TLB::UserMode;
                    tranType = TLB::S12E0Tran;
                    mode     = BaseTLB::Read;
                    break;
                  case MISCREG_AT_S12E0W_Xt:
                    flags    = TLB::MustBeOne | TLB::UserMode;
                    tranType = TLB::S12E0Tran;
                    mode     = BaseTLB::Write;
                    break;
                  case MISCREG_AT_S12E1R_Xt:
                    flags    = TLB::MustBeOne;
                    tranType = TLB::S12E1Tran;
                    mode     = BaseTLB::Read;
                    break;
                  case MISCREG_AT_S12E1W_Xt:
                    flags    = TLB::MustBeOne;
                    tranType = TLB::S12E1Tran;
                    mode     = BaseTLB::Write;
                    break;
                  case MISCREG_AT_S1E3R_Xt:
                    flags    = TLB::MustBeOne;
                    tranType = TLB::S1E3Tran;
                    mode     = BaseTLB::Read;
                    break;
                  case MISCREG_AT_S1E3W_Xt:
                    flags    = TLB::MustBeOne;
                    tranType = TLB::S1E3Tran;
                    mode     = BaseTLB::Write;
                    break;
                }
                // If we're in timing mode then doing the translation in
                // functional mode then we're slightly distorting performance
                // results obtained from simulations. The translation should be
                // done in the same mode the core is running in. NOTE: This
                // can't be an atomic translation because that causes problems
                // with unexpected atomic snoop requests.
                warn("Translating via MISCREG(%d) in functional mode! Fix Me!\n", misc_reg);
                req->setVirt(0, val, 0, flags,  Request::funcMasterId,
                               tc->pcState().pc());
                req->setContext(tc->contextId());
                fault = getDTBPtr(tc)->translateFunctional(req, tc, mode,
                                                           tranType);

                MiscReg newVal;
                if (fault == NoFault) {
                    Addr paddr = req->getPaddr();
                    uint64_t attr = getDTBPtr(tc)->getAttr();
                    uint64_t attr1 = attr >> 56;
                    if (!attr1 || attr1 ==0x44) {
                        attr |= 0x100;
                        attr &= ~ uint64_t(0x80);
                    }
                    newVal = (paddr & mask(47, 12)) | attr;
                    DPRINTF(MiscRegs,
                          "MISCREG: Translated addr %#x: PAR_EL1: %#xx\n",
                          val, newVal);
                } else {
                    ArmFault *armFault = static_cast<ArmFault *>(fault.get());
                    armFault->update(tc);
                    // Set fault bit and FSR
                    FSR fsr = armFault->getFsr(tc);

                    CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);
                    if (cpsr.width) { // AArch32
                        newVal = ((fsr >> 9) & 1) << 11;
                        // rearrange fault status
                        newVal |= ((fsr >>  0) & 0x3f) << 1;
                        newVal |= 0x1; // F bit
                        newVal |= ((armFault->iss() >> 7) & 0x1) << 8;
                        newVal |= armFault->isStage2() ? 0x200 : 0;
                    } else { // AArch64
                        newVal = 1; // F bit
                        newVal |= fsr << 1; // FST
                        // TODO: DDI 0487A.f D7-2083, AbortFault's s1ptw bit.
                        newVal |= armFault->isStage2() ? 1 << 8 : 0; // PTW
                        newVal |= armFault->isStage2() ? 1 << 9 : 0; // S
                        newVal |= 1 << 11; // RES1
                    }
                    DPRINTF(MiscRegs,
                            "MISCREG: Translated addr %#x fault fsr %#x: PAR: %#x\n",
                            val, fsr, newVal);
                }
                delete req;
                setMiscRegNoEffect(MISCREG_PAR_EL1, newVal);
                return;
            }
          case MISCREG_SPSR_EL3:
          case MISCREG_SPSR_EL2:
          case MISCREG_SPSR_EL1:
            // Force bits 23:21 to 0
            newVal = val & ~(0x7 << 21);
            break;
          case MISCREG_L2CTLR:
            warn("miscreg L2CTLR (%s) written with %#x. ignored...\n",
                 miscRegName[misc_reg], uint32_t(val));
            break;

          // Generic Timer registers
          case MISCREG_CNTFRQ ... MISCREG_CNTHP_CTL:
          case MISCREG_CNTPCT ... MISCREG_CNTHP_CVAL:
          case MISCREG_CNTKCTL_EL1 ... MISCREG_CNTV_CVAL_EL0:
          case MISCREG_CNTVOFF_EL2 ... MISCREG_CNTPS_CVAL_EL1:
            getGenericTimer(tc).setMiscReg(misc_reg, newVal);
            break;
        }
    }
    setMiscRegNoEffect(misc_reg, newVal);
}

BaseISADevice &
ISA::getGenericTimer(ThreadContext *tc)
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
    return *timer.get();
}

}

ArmISA::ISA *
ArmISAParams::create()
{
    return new ArmISA::ISA(this);
}
