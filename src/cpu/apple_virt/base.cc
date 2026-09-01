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

#include <algorithm>
#include <cassert>

#include "arch/generic/mmu.hh"
#include "base/logging.hh"
#include "cpu/base.hh"
#include "cpu/simple_thread.hh"
#include "cpu/thread_context.hh"
#include "debug/AppleVirtRun.hh"
#include "params/BaseAppleVirtCPU.hh"
#include "sim/core.hh"
#include "sim/faults.hh"
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
      status(Status::Idle),
      hvReady(false),
      runEvent([this]() { runOnce(); }, name() + ".run"),
      hostRunTime(params.host_run_time_us),
      dataPort(name() + ".dcache_port"),
      instPort(name() + ".icache_port"),
      watchdogStop(false),
      watchdogArmed(false),
      watchdogGeneration(0)
{
    fatal_if(!vm, "BaseAppleVirtCPU requires an AppleVirtVM instance");
    fatal_if(!FullSystem,
             "AppleVirtCPU currently supports full-system mode only");

    thread = new SimpleThread(this, 0, params.system, params.mmu,
                              params.isa[0], params.decoder[0]);

    thread->setStatus(ThreadContext::Halted);
    threadContext = thread->getTC();
    threadContexts.push_back(threadContext);
}

BaseAppleVirtCPU::~BaseAppleVirtCPU()
{
    stopWatchdog();
    destroyVCPU();
    delete thread;
}

void
BaseAppleVirtCPU::init()
{
    BaseCPU::init();

    fatal_if(numThreads != 1,
             "AppleVirtCPU currently supports only a single thread");
    fatal_if(!thread->comInstEventQueue.empty(),
             "AppleVirtCPU does not support instruction-count events");
    verifyMemoryMode();
}

void
BaseAppleVirtCPU::startup()
{
    BaseCPU::startup();
    vm->ensureInitialized(*system);
    createVCPU();
    startWatchdog();

    if (thread->status() == ThreadContext::Active) {
        status = Status::Running;
        if (!runEvent.scheduled()) {
            schedule(runEvent, clockEdge(Cycles(0)));
        }
    }
}

void
BaseAppleVirtCPU::wakeup(ThreadID tid)
{
    if (tid != 0) {
        return;
    }

    if (hvReady) {
        hv_vcpu_t vcpu = hvVCPU;
        hv_vcpus_exit(&vcpu, 1);
    }

    if (thread->status() == ThreadContext::Suspended) {
        thread->activate();
    }
}

void
BaseAppleVirtCPU::activateContext(ThreadID thread_num)
{
    assert(thread_num == 0);
    if (status == Status::Running) {
        return;
    }

    status = Status::Running;
    if (!runEvent.scheduled()) {
        schedule(runEvent, clockEdge(Cycles(0)));
    }
}

void
BaseAppleVirtCPU::suspendContext(ThreadID thread_num)
{
    assert(thread_num == 0);
    if (runEvent.scheduled()) {
        deschedule(runEvent);
    }
    status = Status::Idle;
}

void
BaseAppleVirtCPU::haltContext(ThreadID thread_num)
{
    suspendContext(thread_num);
    updateCycleCounters(BaseCPU::CPU_STATE_SLEEP);
}

ThreadContext *
BaseAppleVirtCPU::getContext(int tid)
{
    assert(tid == 0);
    if (hvReady && std::this_thread::get_id() == ownerThread) {
        syncHVToThread();
    }
    return threadContext;
}

DrainState
BaseAppleVirtCPU::drain()
{
    if (runEvent.scheduled()) {
        deschedule(runEvent);
    }
    status = Status::Idle;
    if (hvReady && std::this_thread::get_id() == ownerThread) {
        syncHVToThread();
    }
    return DrainState::Drained;
}

void
BaseAppleVirtCPU::drainResume()
{
    assert(!runEvent.scheduled());
    if (switchedOut()) {
        return;
    }

    verifyMemoryMode();
    if (thread->status() == ThreadContext::Active) {
        status = Status::Running;
        schedule(runEvent, clockEdge(Cycles(0)));
    }
}

void
BaseAppleVirtCPU::serializeThread(CheckpointOut &cp, ThreadID tid) const
{
    assert(tid == 0);
    assert(status == Status::Idle);
    thread->serialize(cp);
}

void
BaseAppleVirtCPU::unserializeThread(CheckpointIn &cp, ThreadID tid)
{
    assert(tid == 0);
    assert(status == Status::Idle);
    thread->unserialize(cp);
}

void
BaseAppleVirtCPU::verifyMemoryMode() const
{
    fatal_if(!system->bypassCaches(),
             "AppleVirtCPU requires atomic non-caching memory mode");
}

void
BaseAppleVirtCPU::createVCPU()
{
    assert(!hvReady);
    ownerThread = std::this_thread::get_id();
    vm->registerCPU();

    hv_return_t hv_err = hv_vcpu_create(&hvVCPU, &hvExit, nullptr);
    fatal_if(hv_err != HV_SUCCESS,
             "hv_vcpu_create failed for AppleVirtCPU (err=%d)", hv_err);
    hvReady = true;
}

void
BaseAppleVirtCPU::destroyVCPU()
{
    if (!hvReady) {
        return;
    }

    fatal_if(std::this_thread::get_id() != ownerThread,
             "AppleVirtCPU vCPU must be destroyed by its owner thread");
    hv_return_t hv_err = hv_vcpu_destroy(hvVCPU);
    warn_if(hv_err != HV_SUCCESS, "hv_vcpu_destroy failed (err=%d)", hv_err);
    if (hv_err == HV_SUCCESS) {
        hvReady = false;
        vm->unregisterCPU();
    }
}

void
BaseAppleVirtCPU::runOnce()
{
    assert(status == Status::Running);
    fatal_if(std::this_thread::get_id() != ownerThread,
             "AppleVirtCPU must run on the thread which created its vCPU");
    fatal_if(!thread->comInstEventQueue.empty(),
             "AppleVirtCPU does not support instruction-count events");

    syncThreadToHV();
    updateInterrupts();
    armWatchdog();

    const auto host_start = std::chrono::steady_clock::now();
    hv_return_t hv_err = hv_vcpu_run(hvVCPU);
    const auto host_end = std::chrono::steady_clock::now();
    disarmWatchdog();
    fatal_if(hv_err != HV_SUCCESS,
             "hv_vcpu_run failed for AppleVirtCPU (err=%d)", hv_err);

    syncHVToThread();
    fatal_if(!hvExit, "AppleVirtCPU run completed without exit information");

    DPRINTF(AppleVirtRun, "vCPU exited with reason %u\n", hvExit->reason);
    Tick delay = 0;
    switch (hvExit->reason) {
        case HV_EXIT_REASON_CANCELED:
            break;
        case HV_EXIT_REASON_EXCEPTION:
            delay = handleException(hvExit->exception);
            break;
        case HV_EXIT_REASON_VTIMER_ACTIVATED:
            handleVTimerActivated();
            break;
        case HV_EXIT_REASON_UNKNOWN:
        default:
            fatal("AppleVirtCPU exited for an unknown reason");
    }

    if (status == Status::Running) {
        const auto host_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(host_end -
                                                                 host_start)
                .count();
        const Tick execution_delay =
            std::max<Tick>(1, host_ns * sim_clock::as_int::ns);
        baseStats.numCycles += ticksToCycles(execution_delay);
        Cycles next_cycles = ticksToCycles(execution_delay + delay);
        next_cycles = std::max(next_cycles, Cycles(1));
        schedule(runEvent, clockEdge(next_cycles));
    }
}

void
BaseAppleVirtCPU::startWatchdog()
{
    watchdogThread = std::thread([this]() { watchdogLoop(); });
}

void
BaseAppleVirtCPU::stopWatchdog()
{
    if (!watchdogThread.joinable()) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(watchdogMutex);
        watchdogStop = true;
        watchdogArmed = false;
        ++watchdogGeneration;
    }
    watchdogCV.notify_all();
    watchdogThread.join();
}

void
BaseAppleVirtCPU::armWatchdog()
{
    {
        std::lock_guard<std::mutex> lock(watchdogMutex);
        watchdogArmed = true;
        ++watchdogGeneration;
    }
    watchdogCV.notify_all();
}

void
BaseAppleVirtCPU::disarmWatchdog()
{
    {
        std::lock_guard<std::mutex> lock(watchdogMutex);
        watchdogArmed = false;
        ++watchdogGeneration;
    }
    watchdogCV.notify_all();
}

void
BaseAppleVirtCPU::watchdogLoop()
{
    std::unique_lock<std::mutex> lock(watchdogMutex);
    while (!watchdogStop) {
        watchdogCV.wait(lock,
                        [this]() { return watchdogStop || watchdogArmed; });
        if (watchdogStop) {
            break;
        }

        const uint64_t generation = watchdogGeneration;
        if (watchdogCV.wait_for(lock, hostRunTime, [this, generation]() {
                return watchdogStop || !watchdogArmed ||
                       watchdogGeneration != generation;
            })) {
            continue;
        }

        hv_vcpu_t vcpu = hvVCPU;
        watchdogArmed = false;
        lock.unlock();
        hv_return_t hv_err = hv_vcpus_exit(&vcpu, 1);
        warn_if(hv_err != HV_SUCCESS,
                "hv_vcpus_exit failed in AppleVirtCPU watchdog (err=%d)",
                hv_err);
        lock.lock();
    }
}

Tick
BaseAppleVirtCPU::doMMIOAccess(Addr paddr, void *data, unsigned size,
                               bool write)
{
    RequestPtr request = std::make_shared<Request>(
        paddr, size, Request::UNCACHEABLE, dataRequestorId());
    request->setContext(threadContext->contextId());

    const BaseMMU::Mode mode = write ? BaseMMU::Write : BaseMMU::Read;
    Fault fault = threadContext->getMMUPtr()->finalizePhysical(
        request, threadContext, mode);
    if (fault != NoFault) {
        warn("Finalizing AppleVirtCPU MMIO failed: %s", fault->name());
    }

    PacketPtr packet =
        new Packet(request, write ? MemCmd::WriteReq : MemCmd::ReadReq);
    packet->dataStatic(data);

    if (request->isLocalAccess()) {
        const Cycles local_delay =
            request->localAccessor(threadContext, packet);
        delete packet;
        return clockPeriod() * local_delay;
    }

    EventQueue::ScopedMigration migrate(vm->eventQueue());
    Tick delay = dataPort.sendAtomic(packet);
    delete packet;
    return delay;
}

} // namespace gem5
