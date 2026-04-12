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

#include "arch/arm/regs/int.hh"
#include "arch/arm/regs/misc.hh"
#include "arch/arm/regs/misc_types.hh"
#include "cpu/thread_context.hh"
#include "debug/AppleVirtRun.hh"
#include "params/BaseArmAppleVirtCPU.hh"

namespace gem5
{

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
    BaseAppleVirtCPU::syncThreadToHV();

    ThreadContext *tc = threadContext;
    fatal_if(!tc, "AppleVirtCPU has no thread context to sync");

    ArmISA::CPSR pstate(tc->readMiscRegNoEffect(ArmISA::MISCREG_CPSR));

    if (pstate.width) {
        warn_once("AppleVirtCPU forcing AArch64 guest state for HVF run");
        pstate.width = 0;
        pstate.el = 0;
        pstate.sp = 0;
        pstate.t = 0;
    }

    hv_return_t hv_err =
        hv_vcpu_set_reg(hvVCPU, HV_REG_CPSR, static_cast<RegVal>(pstate));
    fatal_if(hv_err != HV_SUCCESS,
             "Failed to set HVF PSTATE register (err=%d)", hv_err);

    const RegVal sp = tc->getReg(ArmISA::int_reg::Spx);
    hv_err = hv_vcpu_set_sys_reg(hvVCPU, HV_SYS_REG_SP_EL0, sp);
    fatal_if(hv_err != HV_SUCCESS,
             "Failed to set HVF SP_EL0 register (err=%d)", hv_err);

    hv_err = hv_vcpu_set_sys_reg(hvVCPU, HV_SYS_REG_SP_EL1, sp);
    fatal_if(hv_err != HV_SUCCESS,
             "Failed to set HVF SP_EL1 register (err=%d)", hv_err);
}

void
BaseArmAppleVirtCPU::syncHVToThread()
{
    BaseAppleVirtCPU::syncHVToThread();

    ThreadContext *tc = threadContext;
    fatal_if(!tc, "AppleVirtCPU has no thread context to sync");

    RegVal pstate = 0;
    hv_return_t hv_err = hv_vcpu_get_reg(hvVCPU, HV_REG_CPSR, &pstate);
    fatal_if(hv_err != HV_SUCCESS,
             "Failed to read HVF PSTATE register (err=%d)", hv_err);
    tc->setMiscRegNoEffect(ArmISA::MISCREG_CPSR, pstate);

    RegVal sp = 0;
    hv_err = hv_vcpu_get_sys_reg(hvVCPU, HV_SYS_REG_SP_EL0, &sp);
    fatal_if(hv_err != HV_SUCCESS,
             "Failed to read HVF SP_EL0 register (err=%d)", hv_err);
    tc->setReg(ArmISA::int_reg::Spx, sp);
}

} // namespace gem5
