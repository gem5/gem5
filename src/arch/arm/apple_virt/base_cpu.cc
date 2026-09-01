/*
 * Copyright (c) 2026 The Regents of The University of California
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

#include "arch/arm/apple_virt/base_cpu.hh"

#include <algorithm>
#include <cstring>

#include "arch/arm/apple_virt/exit_decode.hh"
#include "arch/arm/interrupts.hh"
#include "arch/arm/pcstate.hh"
#include "arch/arm/regs/cc.hh"
#include "arch/arm/regs/int.hh"
#include "arch/arm/regs/misc.hh"
#include "arch/arm/regs/misc_types.hh"
#include "arch/arm/regs/vec.hh"
#include "arch/arm/utility.hh"
#include "base/bitfield.hh"
#include "cpu/thread_context.hh"
#include "debug/AppleVirtRun.hh"
#include "params/BaseArmAppleVirtCPU.hh"

namespace gem5
{

namespace
{

struct SysRegMap
{
    hv_sys_reg_t hv;
    ArmISA::MiscRegIndex gem5;
    bool effectful = false;
};

const SysRegMap sysRegMap[] = {
    {HV_SYS_REG_SCTLR_EL1, ArmISA::MISCREG_SCTLR_EL1},
    {HV_SYS_REG_CPACR_EL1, ArmISA::MISCREG_CPACR_EL1},
    {HV_SYS_REG_TTBR0_EL1, ArmISA::MISCREG_TTBR0_EL1},
    {HV_SYS_REG_TTBR1_EL1, ArmISA::MISCREG_TTBR1_EL1},
    {HV_SYS_REG_TCR_EL1, ArmISA::MISCREG_TCR_EL1},
    {HV_SYS_REG_APIAKEYLO_EL1, ArmISA::MISCREG_APIAKeyLo_EL1},
    {HV_SYS_REG_APIAKEYHI_EL1, ArmISA::MISCREG_APIAKeyHi_EL1},
    {HV_SYS_REG_APIBKEYLO_EL1, ArmISA::MISCREG_APIBKeyLo_EL1},
    {HV_SYS_REG_APIBKEYHI_EL1, ArmISA::MISCREG_APIBKeyHi_EL1},
    {HV_SYS_REG_APDAKEYLO_EL1, ArmISA::MISCREG_APDAKeyLo_EL1},
    {HV_SYS_REG_APDAKEYHI_EL1, ArmISA::MISCREG_APDAKeyHi_EL1},
    {HV_SYS_REG_APDBKEYLO_EL1, ArmISA::MISCREG_APDBKeyLo_EL1},
    {HV_SYS_REG_APDBKEYHI_EL1, ArmISA::MISCREG_APDBKeyHi_EL1},
    {HV_SYS_REG_APGAKEYLO_EL1, ArmISA::MISCREG_APGAKeyLo_EL1},
    {HV_SYS_REG_APGAKEYHI_EL1, ArmISA::MISCREG_APGAKeyHi_EL1},
    {HV_SYS_REG_SPSR_EL1, ArmISA::MISCREG_SPSR_EL1},
    {HV_SYS_REG_ELR_EL1, ArmISA::MISCREG_ELR_EL1},
    {HV_SYS_REG_AFSR0_EL1, ArmISA::MISCREG_AFSR0_EL1},
    {HV_SYS_REG_AFSR1_EL1, ArmISA::MISCREG_AFSR1_EL1},
    {HV_SYS_REG_ESR_EL1, ArmISA::MISCREG_ESR_EL1},
    {HV_SYS_REG_FAR_EL1, ArmISA::MISCREG_FAR_EL1},
    {HV_SYS_REG_PAR_EL1, ArmISA::MISCREG_PAR_EL1},
    {HV_SYS_REG_MAIR_EL1, ArmISA::MISCREG_MAIR_EL1},
    {HV_SYS_REG_AMAIR_EL1, ArmISA::MISCREG_AMAIR_EL1},
    {HV_SYS_REG_VBAR_EL1, ArmISA::MISCREG_VBAR_EL1},
    {HV_SYS_REG_CONTEXTIDR_EL1, ArmISA::MISCREG_CONTEXTIDR_EL1},
    {HV_SYS_REG_TPIDR_EL1, ArmISA::MISCREG_TPIDR_EL1},
    {HV_SYS_REG_CSSELR_EL1, ArmISA::MISCREG_CSSELR_EL1},
    {HV_SYS_REG_TPIDR_EL0, ArmISA::MISCREG_TPIDR_EL0},
    {HV_SYS_REG_TPIDRRO_EL0, ArmISA::MISCREG_TPIDRRO_EL0},
    {HV_SYS_REG_CNTKCTL_EL1, ArmISA::MISCREG_CNTKCTL_EL1, true},
};

} // anonymous namespace

BaseArmAppleVirtCPU::BaseArmAppleVirtCPU(
    const BaseArmAppleVirtCPUParams &params)
    : BaseAppleVirtCPU(params)
{}

void
BaseArmAppleVirtCPU::startup()
{ BaseAppleVirtCPU::startup(); }

void
BaseArmAppleVirtCPU::syncThreadToHV()
{
    ThreadContext *tc = threadContext;
    fatal_if(!tc, "AppleVirtCPU has no thread context to sync");

    const auto pc = tc->pcState().instAddr();
    hv_return_t hv_err = hv_vcpu_set_reg(hvVCPU, HV_REG_PC, pc);
    fatal_if(hv_err != HV_SUCCESS,
             "Failed to set HVF PC register (pc=%#llx err=%d)",
             static_cast<unsigned long long>(pc), hv_err);

    for (int idx = 0; idx < 31; ++idx) {
        const hv_reg_t hv_reg = static_cast<hv_reg_t>(HV_REG_X0 + idx);
        hv_err = hv_vcpu_set_reg(hvVCPU, hv_reg,
                                 tc->getReg(ArmISA::int_reg::x(idx)));
        fatal_if(hv_err != HV_SUCCESS,
                 "Failed to set HVF X%d register (err=%d)", idx, hv_err);
    }

    fatal_if(!ArmISA::inAArch64(tc),
             "AppleVirtCPU supports AArch64 guest state only");
    ArmISA::CPSR pstate(tc->readMiscRegNoEffect(ArmISA::MISCREG_CPSR));
    fatal_if(pstate.el > 1,
             "AppleVirtCPU supports guest exception levels EL0 and EL1");
    pstate.nz = tc->getReg(ArmISA::cc_reg::Nz);
    pstate.c = tc->getReg(ArmISA::cc_reg::C);
    pstate.v = tc->getReg(ArmISA::cc_reg::V);

    hv_err = hv_vcpu_set_reg(hvVCPU, HV_REG_CPSR, static_cast<RegVal>(pstate));
    fatal_if(hv_err != HV_SUCCESS,
             "Failed to set HVF PSTATE register (err=%d)", hv_err);

    hv_err = hv_vcpu_set_sys_reg(
        hvVCPU, HV_SYS_REG_SP_EL0,
        tc->getReg(ArmISA::intRegClass[ArmISA::int_reg::Sp0]));
    fatal_if(hv_err != HV_SUCCESS,
             "Failed to set HVF SP_EL0 register (err=%d)", hv_err);

    hv_err = hv_vcpu_set_sys_reg(
        hvVCPU, HV_SYS_REG_SP_EL1,
        tc->getReg(ArmISA::intRegClass[ArmISA::int_reg::Sp1]));
    fatal_if(hv_err != HV_SUCCESS,
             "Failed to set HVF SP_EL1 register (err=%d)", hv_err);

    for (const auto &mapping : sysRegMap) {
        RegVal value;
        if (mapping.effectful) {
            EventQueue::ScopedMigration migrate(vm->eventQueue());
            value = tc->readMiscReg(mapping.gem5);
        } else {
            value = tc->readMiscRegNoEffect(mapping.gem5);
        }
        hv_err = hv_vcpu_set_sys_reg(hvVCPU, mapping.hv, value);
        fatal_if(hv_err != HV_SUCCESS,
                 "Failed to set HVF system register %#x (err=%d)", mapping.hv,
                 hv_err);
    }

    for (int idx = 0; idx < ArmISA::NumVecV8ArchRegs; ++idx) {
        ArmISA::VecRegContainer container;
        tc->getReg(ArmISA::vecRegClass[idx], &container);
        hv_simd_fp_uchar16_t value{};
        const auto bytes = container.as<uint8_t>();
        std::memcpy(&value, bytes, sizeof(value));
        hv_err = hv_vcpu_set_simd_fp_reg(
            hvVCPU, static_cast<hv_simd_fp_reg_t>(HV_SIMD_FP_REG_Q0 + idx),
            value);
        fatal_if(hv_err != HV_SUCCESS,
                 "Failed to set HVF Q%d register (err=%d)", idx, hv_err);
    }

    hv_err = hv_vcpu_set_reg(hvVCPU, HV_REG_FPCR,
                             tc->readMiscRegNoEffect(ArmISA::MISCREG_FPCR));
    fatal_if(hv_err != HV_SUCCESS, "Failed to set HVF FPCR register (err=%d)",
             hv_err);
    hv_err = hv_vcpu_set_reg(hvVCPU, HV_REG_FPSR,
                             tc->readMiscRegNoEffect(ArmISA::MISCREG_FPSR));
    fatal_if(hv_err != HV_SUCCESS, "Failed to set HVF FPSR register (err=%d)",
             hv_err);
}

void
BaseArmAppleVirtCPU::syncHVToThread()
{
    ThreadContext *tc = threadContext;
    fatal_if(!tc, "AppleVirtCPU has no thread context to sync");

    RegVal pc = 0;
    hv_return_t hv_err = hv_vcpu_get_reg(hvVCPU, HV_REG_PC, &pc);
    fatal_if(hv_err != HV_SUCCESS, "Failed to read HVF PC register (err=%d)",
             hv_err);
    tc->pcState(static_cast<Addr>(pc));

    for (int idx = 0; idx < 31; ++idx) {
        const hv_reg_t hv_reg = static_cast<hv_reg_t>(HV_REG_X0 + idx);
        RegVal value = 0;
        hv_err = hv_vcpu_get_reg(hvVCPU, hv_reg, &value);
        fatal_if(hv_err != HV_SUCCESS,
                 "Failed to read HVF X%d register (err=%d)", idx, hv_err);
        tc->setReg(ArmISA::int_reg::x(idx), value);
    }

    RegVal pstate = 0;
    hv_err = hv_vcpu_get_reg(hvVCPU, HV_REG_CPSR, &pstate);
    fatal_if(hv_err != HV_SUCCESS,
             "Failed to read HVF PSTATE register (err=%d)", hv_err);
    const ArmISA::CPSR cpsr(pstate);
    tc->setMiscRegNoEffect(ArmISA::MISCREG_CPSR, cpsr);
    tc->setReg(ArmISA::cc_reg::Nz, cpsr.nz);
    tc->setReg(ArmISA::cc_reg::C, cpsr.c);
    tc->setReg(ArmISA::cc_reg::V, cpsr.v);

    RegVal sp = 0;
    hv_err = hv_vcpu_get_sys_reg(hvVCPU, HV_SYS_REG_SP_EL0, &sp);
    fatal_if(hv_err != HV_SUCCESS,
             "Failed to read HVF SP_EL0 register (err=%d)", hv_err);
    tc->setReg(ArmISA::intRegClass[ArmISA::int_reg::Sp0], sp);

    hv_err = hv_vcpu_get_sys_reg(hvVCPU, HV_SYS_REG_SP_EL1, &sp);
    fatal_if(hv_err != HV_SUCCESS,
             "Failed to read HVF SP_EL1 register (err=%d)", hv_err);
    tc->setReg(ArmISA::intRegClass[ArmISA::int_reg::Sp1], sp);

    for (const auto &mapping : sysRegMap) {
        RegVal value = 0;
        hv_err = hv_vcpu_get_sys_reg(hvVCPU, mapping.hv, &value);
        fatal_if(hv_err != HV_SUCCESS,
                 "Failed to read HVF system register %#x (err=%d)", mapping.hv,
                 hv_err);
        if (mapping.effectful) {
            EventQueue::ScopedMigration migrate(vm->eventQueue());
            tc->setMiscReg(mapping.gem5, value);
        } else {
            tc->setMiscRegNoEffect(mapping.gem5, value);
        }
    }

    for (int idx = 0; idx < ArmISA::NumVecV8ArchRegs; ++idx) {
        hv_simd_fp_uchar16_t value{};
        hv_err = hv_vcpu_get_simd_fp_reg(
            hvVCPU, static_cast<hv_simd_fp_reg_t>(HV_SIMD_FP_REG_Q0 + idx),
            &value);
        fatal_if(hv_err != HV_SUCCESS,
                 "Failed to read HVF Q%d register (err=%d)", idx, hv_err);
        auto *container = static_cast<ArmISA::VecRegContainer *>(
            tc->getWritableReg(ArmISA::vecRegClass[idx]));
        auto bytes = container->as<uint8_t>();
        std::memcpy(bytes, &value, sizeof(value));
    }

    RegVal value = 0;
    hv_err = hv_vcpu_get_reg(hvVCPU, HV_REG_FPCR, &value);
    fatal_if(hv_err != HV_SUCCESS, "Failed to read HVF FPCR register (err=%d)",
             hv_err);
    tc->setMiscRegNoEffect(ArmISA::MISCREG_FPCR, value);
    hv_err = hv_vcpu_get_reg(hvVCPU, HV_REG_FPSR, &value);
    fatal_if(hv_err != HV_SUCCESS, "Failed to read HVF FPSR register (err=%d)",
             hv_err);
    tc->setMiscRegNoEffect(ArmISA::MISCREG_FPSR, value);

    ArmISA::PCState pc_state = tc->pcState().as<ArmISA::PCState>();
    pc_state.aarch64(true);
    pc_state.nextAArch64(true);
    pc_state.thumb(false);
    pc_state.nextThumb(false);
    tc->pcState(pc_state);
}

void
BaseArmAppleVirtCPU::advancePC()
{
    uint64_t pc = 0;
    hv_return_t hv_err = hv_vcpu_get_reg(hvVCPU, HV_REG_PC, &pc);
    fatal_if(hv_err != HV_SUCCESS,
             "Failed to read HVF PC while completing an exit (err=%d)",
             hv_err);
    pc += sizeof(uint32_t);
    hv_err = hv_vcpu_set_reg(hvVCPU, HV_REG_PC, pc);
    fatal_if(hv_err != HV_SUCCESS,
             "Failed to advance HVF PC while completing an exit (err=%d)",
             hv_err);
    threadContext->pcState(static_cast<Addr>(pc));
}

void
BaseArmAppleVirtCPU::updateInterrupts()
{
    auto *interrupt = static_cast<ArmISA::Interrupts *>(interrupts[0]);
    const bool irq = interrupt->checkRaw(ArmISA::INT_IRQ);
    const bool fiq = interrupt->checkRaw(ArmISA::INT_FIQ);

    hv_return_t hv_err =
        hv_vcpu_set_pending_interrupt(hvVCPU, HV_INTERRUPT_TYPE_IRQ, irq);
    fatal_if(hv_err != HV_SUCCESS, "Failed to update HVF IRQ state (err=%d)",
             hv_err);
    hv_err = hv_vcpu_set_pending_interrupt(hvVCPU, HV_INTERRUPT_TYPE_FIQ, fiq);
    fatal_if(hv_err != HV_SUCCESS, "Failed to update HVF FIQ state (err=%d)",
             hv_err);
}

Tick
BaseArmAppleVirtCPU::handleException(const hv_vcpu_exit_exception_t &exception)
{
    const auto decoded = ArmISA::AppleVirt::decodeExit(exception.syndrome);
    using ArmISA::AppleVirt::ExitKind;

    DPRINTF(AppleVirtRun,
            "exception PC=%#x ESR=%#llx EC=%#x ISS=%#x FAR=%#llx "
            "IPA=%#llx\n",
            threadContext->pcState().instAddr(),
            static_cast<unsigned long long>(exception.syndrome), decoded.ec,
            decoded.iss,
            static_cast<unsigned long long>(exception.virtual_address),
            static_cast<unsigned long long>(exception.physical_address));

    if (decoded.kind == ExitKind::Wfi || decoded.kind == ExitKind::Wfe) {
        advancePC();
        return 0;
    }

    if (decoded.kind == ExitKind::SystemRegister) {
        const auto &sys_reg = decoded.systemRegister;
        const ArmISA::MiscRegIndex misc_reg = ArmISA::decodeAArch64SysReg(
            sys_reg.op0, sys_reg.op1, sys_reg.crn, sys_reg.crm, sys_reg.op2);
        fatal_if(misc_reg == ArmISA::MISCREG_UNKNOWN ||
                     misc_reg == ArmISA::MISCREG_IMPDEF_UNIMPL,
                 "Unsupported AppleVirtCPU system register access: "
                 "S%u_%u_C%u_C%u_%u ESR=%#llx",
                 sys_reg.op0, sys_reg.op1, sys_reg.crn, sys_reg.crm,
                 sys_reg.op2,
                 static_cast<unsigned long long>(exception.syndrome));

        if (sys_reg.read) {
            const RegVal value = threadContext->readMiscReg(misc_reg);
            if (sys_reg.targetRegister != 31) {
                threadContext->setReg(
                    ArmISA::int_reg::x(sys_reg.targetRegister), value);
                const hv_return_t hv_err = hv_vcpu_set_reg(
                    hvVCPU,
                    static_cast<hv_reg_t>(HV_REG_X0 + sys_reg.targetRegister),
                    value);
                fatal_if(hv_err != HV_SUCCESS,
                         "Failed to complete AppleVirtCPU system register "
                         "read (err=%d)",
                         hv_err);
            }
        } else {
            const RegVal value =
                sys_reg.targetRegister == 31
                    ? 0
                    : threadContext->getReg(
                          ArmISA::int_reg::x(sys_reg.targetRegister));
            threadContext->setMiscReg(misc_reg, value);
        }

        advancePC();
        return 0;
    }

    if (decoded.kind != ExitKind::DataAbort) {
        fatal("Unsupported AppleVirtCPU exception: ESR=%#llx EC=%#x "
              "ISS=%#x FAR=%#llx IPA=%#llx",
              static_cast<unsigned long long>(exception.syndrome), decoded.ec,
              decoded.iss,
              static_cast<unsigned long long>(exception.virtual_address),
              static_cast<unsigned long long>(exception.physical_address));
    }

    const auto &abort = decoded.dataAbort;
    fatal_if(!abort.valid || abort.s1ptw || abort.cacheMaintenance,
             "Unsupported AppleVirtCPU data abort: ESR=%#llx FAR=%#llx "
             "IPA=%#llx",
             static_cast<unsigned long long>(exception.syndrome),
             static_cast<unsigned long long>(exception.virtual_address),
             static_cast<unsigned long long>(exception.physical_address));

    RegVal value = 0;
    if (abort.write && abort.targetRegister != 31) {
        value =
            threadContext->getReg(ArmISA::int_reg::x(abort.targetRegister));
    }

    const Tick delay = doMMIOAccess(exception.physical_address, &value,
                                    abort.size, abort.write);

    if (!abort.write && abort.targetRegister != 31) {
        const unsigned bits = abort.size * 8;
        value &= mask(bits);
        if (abort.signExtend) {
            const RegVal sign_bit = RegVal(1) << (bits - 1);
            value = (value ^ sign_bit) - sign_bit;
        }
        if (!abort.sf) {
            value = static_cast<uint32_t>(value);
        }
        threadContext->setReg(ArmISA::int_reg::x(abort.targetRegister), value);
        hv_return_t hv_err = hv_vcpu_set_reg(
            hvVCPU, static_cast<hv_reg_t>(HV_REG_X0 + abort.targetRegister),
            value);
        fatal_if(hv_err != HV_SUCCESS,
                 "Failed to complete AppleVirtCPU MMIO read (err=%d)", hv_err);
    }

    advancePC();
    return delay;
}

void
BaseArmAppleVirtCPU::handleVTimerActivated()
{
    fatal("AppleVirtCPU does not support the architectural virtual timer; "
          "use a gem5-simulated timer");
}

} // namespace gem5
