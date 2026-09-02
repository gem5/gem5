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

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include "cpu/apple_virt/vm.hh"
#include "cpu/base.hh"
#include "cpu/simple_thread.hh"
#include "mem/port.hh"
#include "sim/eventq.hh"

namespace gem5
{

struct BaseAppleVirtCPUParams;

/**
 * Base class for CPUs accelerated by Apple's Hypervisor framework.
 *
 * The initial implementation intentionally supports one full-system CPU in
 * atomic non-caching mode. A host-side watchdog bounds each blocking
 * hv_vcpu_run() call so gem5's event queue continues to make progress.
 * Hypervisor.framework does not expose a retired-instruction counter, so
 * instruction-count events and instruction statistics are unsupported.
 */
class BaseAppleVirtCPU : public BaseCPU
{
  public:
    BaseAppleVirtCPU(const BaseAppleVirtCPUParams &params);
    ~BaseAppleVirtCPU() override;

    void init() override;
    void startup() override;
    DrainState drain() override;
    void drainResume() override;
    void switchOut() override;
    void takeOverFrom(BaseCPU *cpu) override;
    void serializeThread(CheckpointOut &cp, ThreadID tid) const override;
    void unserializeThread(CheckpointIn &cp, ThreadID tid) override;
    void verifyMemoryMode() const override;

    Port &
    getDataPort() override
    {
        return dataPort;
    }
    Port &
    getInstPort() override
    {
        return instPort;
    }
    void wakeup(ThreadID tid = 0) override;
    bool
    wakeupOnInterrupt(ThreadID tid) const override
    {
        return true;
    }
    void activateContext(ThreadID thread_num) override;
    void suspendContext(ThreadID thread_num) override;
    void haltContext(ThreadID thread_num) override;

    ThreadContext *getContext(int tid) override;

    Counter
    totalInsts() const override
    {
        return 0;
    }
    Counter
    totalOps() const override
    {
        return 0;
    }

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

    enum class Status
    {
        Idle,
        Running,
    };

    Status status;

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
    std::chrono::microseconds hostRunTime;

    AppleVirtCPUPort dataPort;
    AppleVirtCPUPort instPort;

    void runOnce();
    void createVCPU();
    void destroyVCPU();
    void startWatchdog();
    void stopWatchdog();
    void armWatchdog();
    void disarmWatchdog();
    void watchdogLoop();

    Tick doMMIOAccess(Addr paddr, void *data, unsigned size, bool write);

    virtual void advancePC() = 0;
    virtual void syncThreadToHV() = 0;
    virtual void syncHVToThread() = 0;
    virtual void updateInterrupts() = 0;
    virtual Tick
    handleException(const hv_vcpu_exit_exception_t &exception) = 0;
    virtual void handleVTimerActivated() = 0;

    std::thread::id ownerThread;
    std::thread watchdogThread;
    std::mutex watchdogMutex;
    std::condition_variable watchdogCV;
    bool watchdogStop;
    bool watchdogArmed;
    uint64_t watchdogGeneration;
};

} // namespace gem5

#endif
