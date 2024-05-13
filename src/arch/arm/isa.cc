/*
 * Copyright (c) 2010-2024 Arm Limited
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
#include "arch/arm/regs/misc_accessors.hh"
#include "arch/arm/self_debug.hh"
#include "arch/arm/system.hh"
#include "arch/arm/utility.hh"
#include "arch/generic/decoder.hh"
#include "base/cprintf.hh"
#include "base/random.hh"
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

using namespace misc_regs;

namespace
{

/* Not applicable to ARM */
RegClass floatRegClass(FloatRegClass, FloatRegClassName, 0, debug::FloatRegs);

} // anonymous namespace

ISA::ISA(const Params &p) : BaseISA(p, "arm"), system(NULL),
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
        highestEL = system->highestEL();
        haveLargeAsid64 = system->haveLargeAsid64();
        physAddrRange = system->physAddrRange();
        sveVL = system->sveVL();
        smeVL = system->smeVL();

        release = system->releaseFS();
    } else {
        highestELIs64 = true; // ArmSystem::highestELIs64 does the same
        highestEL = EL1; // ArmSystem::highestEL does the same
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
    // Invalidate cached copies of miscregs in the TLBs
    if (tc) {
        getMMUPtr(tc)->invalidateMiscReg();
    }

    for (auto idx = 0; idx < NUM_MISCREGS; idx++) {
        miscRegs[idx] = lookUpMiscReg[idx].reset();
    }

    updateRegMap(miscRegs[MISCREG_CPSR]);
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
      case MISCREG_MPAM1_EL1:
        return currEL() == EL2 ? MISCREG_MPAM2_EL2 : misc_reg;
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
      case MISCREG_SCTLR2_EL12:
        return MISCREG_SCTLR2_EL1;
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
      case MISCREG_TCR2_EL12:
        return MISCREG_TCR2_EL1;
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
      case MISCREG_MPAM1_EL12:
        return MISCREG_MPAM1_EL1;
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
      case MISCREG_ID_AFR0: // not implemented, so alias MIDR
      case MISCREG_REVIDR:  // not implemented, so alias MIDR
      case MISCREG_MIDR:
      case MISCREG_MIDR_EL1:
        if (currEL() == EL1 && EL2Enabled(tc)) {
            return readMiscRegNoEffect(MISCREG_VPIDR);
        } else {
            return readMiscRegNoEffect(idx);
        }
        break;

      case MISCREG_CLIDR:
        warn_once("The clidr register always reports 0 caches.\n");
        warn_once("clidr LoUIS field of 0b001 to match current "
                  "ARM implementations.\n");
        return 0x00200000;
      case MISCREG_CCSIDR:
        warn_once("The ccsidr register isn't implemented and "
                "always reads as 0.\n");
        break;
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
      case MISCREG_L2CTLR:
        {
            // mostly unimplemented, just set NumCPUs field from sim and return
            L2CTLR l2ctlr = 0;
            // b00:1CPU to b11:4CPUs
            l2ctlr.numCPUs = tc->getSystemPtr()->threads.size() - 1;
            return l2ctlr;
        }
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
      case MISCREG_HCPTR:
        {
            HCPTR val = readMiscRegNoEffect(idx);
            bool secure_lookup = release->has(ArmExtension::SECURITY) &&
                isSecure(tc);
            if (!secure_lookup) {
                NSACR nsacr = readMiscRegNoEffect(MISCREG_NSACR);
                if (!nsacr.cp10) {
                    val.tcp10 = 1;
                    val.tcp11 = 1;
                }
            }
            return val;
        }
      case MISCREG_HDFAR: // alias for secure DFAR
        return readMiscRegNoEffect(MISCREG_DFAR_S);
      case MISCREG_HIFAR: // alias for secure IFAR
        return readMiscRegNoEffect(MISCREG_IFAR_S);

      case MISCREG_MPAM1_EL1:
        {
            MPAM mpam1 = readMiscRegNoEffect(MISCREG_MPAM1_EL1);
            mpam1.mpamEn = readRegisterNoEffect<MpamAccessor>(
                tc, highestEL).mpamEn;
            mpam1.el1.forcedNs = isSecure(tc) ?
                readRegisterNoEffect<MpamAccessor>(tc, EL3).el3.forceNs : 0;
            return mpam1;
        }
      case MISCREG_MPAM2_EL2:
        {
            MPAM mpam2 = readMiscRegNoEffect(MISCREG_MPAM2_EL2);
            mpam2.mpamEn = readRegisterNoEffect<MpamAccessor>(
                tc, highestEL).mpamEn;
            return mpam2;
        }

      case MISCREG_RNDR:
        tc->setReg(cc_reg::Nz, (RegVal)0);
        tc->setReg(cc_reg::C, (RegVal)0);
        tc->setReg(cc_reg::V, (RegVal)0);
        return random_mt.random<RegVal>();
      case MISCREG_RNDRRS:
        tc->setReg(cc_reg::Nz, (RegVal)0);
        tc->setReg(cc_reg::C, (RegVal)0);
        tc->setReg(cc_reg::V, (RegVal)0);
        // Note: we are not reseeding
        // The random number generator already has an hardcoded
        // seed for the sake of determinism. There is no point
        // in simulating non-determinism here
        return random_mt.random<RegVal>();

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
        pc.illegalExec(cpsr.il == 1);
        selfDebug->setDebugMask(cpsr.d == 1);

        tc->getDecoderPtr()->as<Decoder>().setSveLen(
                (getCurSveVecLenInBits() >> 7) - 1);
        tc->getDecoderPtr()->as<Decoder>().setSmeLen(
                (getCurSmeVecLenInBits() >> 7) - 1);

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
                // Only cp10, cp11, and ase are implemented
                // nothing else should be writable
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
            tc->getDecoderPtr()->as<Decoder>().setContext(newVal);
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
          case MISCREG_HCR:
            {
                const HDCR mdcr  = tc->readMiscRegNoEffect(MISCREG_MDCR_EL2);
                selfDebug->setenableTDETGE((HCR)val, mdcr);
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
          case MISCREG_DBGBCR0 ... MISCREG_DBGBCR15:
            selfDebug->updateDBGBCR(idx - MISCREG_DBGBCR0, val);
            break;
          case MISCREG_DBGWCR0 ... MISCREG_DBGWCR15:
            selfDebug->updateDBGWCR(idx - MISCREG_DBGWCR0, val);
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

          case MISCREG_DBGBCR0_EL1 ... MISCREG_DBGBCR15_EL1:
            selfDebug->updateDBGBCR(idx - MISCREG_DBGBCR0_EL1, val);
            break;
          case MISCREG_DBGWCR0_EL1 ... MISCREG_DBGWCR15_EL1:
            selfDebug->updateDBGWCR(idx - MISCREG_DBGWCR0_EL1, val);
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
            // reading them back in getCurSmeVecLenInBits to avoid
            // setting stale vector lengths in the decoder.
            setMiscRegNoEffect(idx, newVal);
            tc->getDecoderPtr()->as<Decoder>().setSmeLen(
                    (getCurSmeVecLenInBits() >> 7) - 1);
            return;
        }
        setMiscRegNoEffect(idx, newVal);
    }
}

RegVal
ISA::readMiscRegReset(RegIndex idx) const
{
    int flat_idx = flattenMiscIndex(idx);
    return lookUpMiscReg[flat_idx].reset();
}

void
ISA::setMiscRegReset(RegIndex idx, RegVal val)
{
    int flat_idx = flattenMiscIndex(idx);
    InitReg(flat_idx).reset(val);
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
    SVCR svcr = miscRegs[MISCREG_SVCR];
    // If we are in Streaming Mode, we should return the Streaming Mode vector
    // length instead.
    if (svcr.sm) {
        return getCurSmeVecLenInBits();
    }

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

unsigned
ISA::getCurSmeVecLenInBits() const
{
    if (!FullSystem) {
        return smeVL * 128;
    }

    panic_if(!tc,
             "A ThreadContext is needed to determine the SME vector length "
             "in full-system mode");

    CPSR cpsr = miscRegs[MISCREG_CPSR];
    ExceptionLevel el = (ExceptionLevel) (uint8_t) cpsr.el;

    unsigned len = 0;

    if (el == EL1 || (el == EL0 && !ELIsInHost(tc, el))) {
        len = static_cast<SMCR>(miscRegs[MISCREG_SMCR_EL1]).len;
    }

    if (el == EL2 || (el == EL0 && ELIsInHost(tc, el))) {
        len = static_cast<SMCR>(miscRegs[MISCREG_SMCR_EL2]).len;
    } else if (release->has(ArmExtension::VIRTUALIZATION) && !isSecure(tc) &&
               (el == EL0 || el == EL1)) {
        len = std::min(
            len,
            static_cast<unsigned>(
                static_cast<SMCR>(miscRegs[MISCREG_SMCR_EL2]).len));
    }

    if (el == EL3) {
        len = static_cast<SMCR>(miscRegs[MISCREG_SMCR_EL3]).len;
    } else if (release->has(ArmExtension::SECURITY)) {
        len = std::min(
            len,
            static_cast<unsigned>(
                static_cast<SMCR>(miscRegs[MISCREG_SMCR_EL3]).len));
    }

    len = std::min(len, smeVL - 1);

    // len + 1 must be a power of 2! Round down to the nearest whole power of
    // two.
    static const unsigned LUT[16] = {0, 1, 1, 3, 3, 3, 3, 7,
                                     7, 7, 7, 7, 7, 7, 7, 15};
    len = LUT[len];

    return (len + 1) * 128;
}

void
ISA::serialize(CheckpointOut &cp) const
{
    BaseISA::serialize(cp);

    DPRINTF(Checkpoint, "Serializing Arm Misc Registers\n");
    SERIALIZE_MAPPING(miscRegs, miscRegName, NUM_PHYS_MISCREGS);
}

void
ISA::unserialize(CheckpointIn &cp)
{
    DPRINTF(Checkpoint, "Unserializing Arm Misc Registers\n");
    UNSERIALIZE_MAPPING(miscRegs, miscRegName, NUM_PHYS_MISCREGS);

    for (auto idx = 0; idx < NUM_MISCREGS; idx++) {
        if (!lookUpMiscReg[idx].info[MISCREG_UNSERIALIZE] &&
            miscRegs[idx] != lookUpMiscReg[idx].reset()) {
            warn("Checkpoint value for register %s does not match "
                 "current configuration (checkpointed: %#x, current: %#x)",
                 miscRegName[idx], miscRegs[idx],
                 lookUpMiscReg[idx].reset());
            miscRegs[idx] = lookUpMiscReg[idx].reset();
        }
    }

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
