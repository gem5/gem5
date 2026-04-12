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

#ifndef __CPU_APPLE_VIRT_BASE_HH__
#define __CPU_APPLE_VIRT_BASE_HH__

#include <Hypervisor/hv_vcpu.h>

#include <memory>

#include "cpu/apple_virt/vm.hh"
#include "cpu/base.hh"
#include "cpu/simple_thread.hh"
#include "mem/port.hh"
#include "sim/eventq.hh"

namespace gem5
{

struct BaseAppleVirtCPUParams;

/**
 * Skeleton CPU model that will eventually offload execution to the Apple
 * Hypervisor framework.  At the moment it only allocates the shared VM and
 * prepares per-CPU bookkeeping; the run loop will be fleshed out as the
 * backend implementation matures.
 */
class BaseAppleVirtCPU : public BaseCPU
{
  public:
    BaseAppleVirtCPU(const BaseAppleVirtCPUParams &params);
    ~BaseAppleVirtCPU() override;

    void init() override;
    void startup() override;

    Port &
    getDataPort() override
    { return dataPort; }
    Port &
    getInstPort() override
    { return instPort; }
    void wakeup(ThreadID tid = 0) override;

    ThreadContext *
    getContext(int tid) override
    { return threadContext; }

    Counter
    totalInsts() const override
    { return 0; }
    Counter
    totalOps() const override
    { return 0; }

  protected:
    /** Shared virtual machine front-end. */
    AppleVirtVM *vm;

    /** Cached gem5 thread representation. */
    SimpleThread *thread;

    /** ThreadContext view to expose to the rest of gem5. */
    ThreadContext *threadContext;

    /** HVF handle for this vCPU. */
    hv_vcpu_t hvVCPU;
    hv_vcpu_exit_t *hvExit;

    class AppleVirtCPUPort : public RequestPort
    {
      public:
        explicit AppleVirtCPUPort(const std::string &name) : RequestPort(name)
        {}

      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
    };

    bool hvReady;
    EventFunctionWrapper runEvent;
    Cycles runPeriod;

    AppleVirtCPUPort dataPort;
    AppleVirtCPUPort instPort;

    void runOnce();
    virtual void syncThreadToHV();
    virtual void syncHVToThread();
};

} // namespace gem5

#endif
