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

#include "cpu/apple_virt/base.hh"

#include <Hypervisor/hv_vcpu.h>

#include "arch/arm/regs/int.hh"
#include "base/logging.hh"
#include "cpu/base.hh"
#include "cpu/simple_thread.hh"
#include "cpu/thread_context.hh"
#include "debug/AppleVirtRun.hh"
#include "params/BaseAppleVirtCPU.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"
#include "sim/system.hh"

namespace gem5
{

bool
BaseAppleVirtCPU::AppleVirtCPUPort::recvTimingResp(PacketPtr)
{
    panic("%s does not support timing responses", name());
    return false;
}

void
BaseAppleVirtCPU::AppleVirtCPUPort::recvReqRetry()
{ panic("%s does not support request retries", name()); }

BaseAppleVirtCPU::BaseAppleVirtCPU(const BaseAppleVirtCPUParams &params)
    : BaseCPU(params),
      vm(params.vm),
      thread(nullptr),
      threadContext(nullptr),
      hvVCPU(static_cast<hv_vcpu_t>(~0U)),
      hvExit(nullptr),
      hvReady(false),
      runEvent([this]() { runOnce(); }, name() + ".run"),
      runPeriod(params.run_period),
      dataPort(name() + ".dcache_port"),
      instPort(name() + ".icache_port")
{
    fatal_if(!vm, "BaseAppleVirtCPU requires an AppleVirtVM instance");

    if (FullSystem) {
        thread = new SimpleThread(this, 0, params.system, params.mmu,
                                  params.isa[0], params.decoder[0]);
    } else {
        fatal_if(params.workload.empty(),
                 "AppleVirtCPU requires exactly one SE workload");
        thread =
            new SimpleThread(this, 0, params.system, params.workload[0],
                             params.mmu, params.isa[0], params.decoder[0]);
    }

    thread->setStatus(ThreadContext::Halted);
    threadContext = thread->getTC();
    threadContexts.push_back(threadContext);
}

BaseAppleVirtCPU::~BaseAppleVirtCPU()
{
    if (hvReady) {
        hv_return_t hv_err = hv_vcpu_destroy(hvVCPU);
        if (hv_err != HV_SUCCESS) {
            warn("hv_vcpu_destroy failed (err=%d)", hv_err);
        }
    }

    delete thread;
}

void
BaseAppleVirtCPU::init()
{
    BaseCPU::init();

    fatal_if(numThreads != 1,
             "AppleVirtCPU currently supports only a single thread");
}

void
BaseAppleVirtCPU::startup()
{
    BaseCPU::startup();
    vm->ensureInitialized(*system);
    schedule(runEvent, clockEdge(runPeriod));
}

void
BaseAppleVirtCPU::wakeup(ThreadID tid)
{
    if (tid != 0) {
        return;
    }

    if (!runEvent.scheduled()) {
        schedule(runEvent, clockEdge(Cycles(0)));
    }
}

void
BaseAppleVirtCPU::runOnce()
{
    if (!hvReady) {
        hv_return_t hv_err = hv_vcpu_create(&hvVCPU, &hvExit, nullptr);
        fatal_if(hv_err != HV_SUCCESS,
                 "hv_vcpu_create failed for AppleVirtCPU (err=%d)", hv_err);
        hvReady = true;
    }

    syncThreadToHV();

    hv_return_t hv_err = hv_vcpu_run(hvVCPU);
    fatal_if(hv_err != HV_SUCCESS,
             "hv_vcpu_run failed for AppleVirtCPU (err=%d)", hv_err);

    if (hvExit) {
        DPRINTF(AppleVirtRun, "vCPU exited with reason %u\n", hvExit->reason);
        if (hvExit->reason == HV_EXIT_REASON_EXCEPTION) {
            warn("AppleVirtCPU exception exit: syndrome=%#x far=%#llx",
                 hvExit->exception.syndrome,
                 static_cast<unsigned long long>(
                     hvExit->exception.virtual_address));
            return;
        }
    }

    syncHVToThread();

    schedule(runEvent, clockEdge(runPeriod));
}

void
BaseAppleVirtCPU::syncThreadToHV()
{
    ThreadContext *tc = threadContext;
    fatal_if(!tc, "AppleVirtCPU has no thread context to sync");

    const auto pc = tc->pcState().instAddr();
    hv_return_t hv_err = hv_vcpu_set_reg(hvVCPU, HV_REG_PC, pc);
    fatal_if(hv_err != HV_SUCCESS,
             "Failed to set HVF PC register (pc=%#llx err=%d)",
             static_cast<unsigned long long>(pc), hv_err);

    for (int idx = 0; idx < 31; ++idx) {
        hv_reg_t hv_reg = static_cast<hv_reg_t>(HV_REG_X0 + idx);
        hv_err = hv_vcpu_set_reg(hvVCPU, hv_reg,
                                 tc->getReg(ArmISA::int_reg::x(idx)));
        fatal_if(hv_err != HV_SUCCESS,
                 "Failed to set HVF X%d register (err=%d)", idx, hv_err);
    }

    // TODO: sync SP once an HVF mapping is confirmed.
}

void
BaseAppleVirtCPU::syncHVToThread()
{
    ThreadContext *tc = threadContext;
    fatal_if(!tc, "AppleVirtCPU has no thread context to sync");

    RegVal pc = 0;
    hv_return_t hv_err = hv_vcpu_get_reg(hvVCPU, HV_REG_PC, &pc);
    fatal_if(hv_err != HV_SUCCESS, "Failed to read HVF PC register (err=%d)",
             hv_err);
    tc->pcState(static_cast<Addr>(pc));

    for (int idx = 0; idx < 31; ++idx) {
        hv_reg_t hv_reg = static_cast<hv_reg_t>(HV_REG_X0 + idx);
        RegVal value = 0;
        hv_err = hv_vcpu_get_reg(hvVCPU, hv_reg, &value);
        fatal_if(hv_err != HV_SUCCESS,
                 "Failed to read HVF X%d register (err=%d)", idx, hv_err);
        tc->setReg(ArmISA::int_reg::x(idx), value);
    }

    // TODO: sync SP once an HVF mapping is confirmed.
}

} // namespace gem5
