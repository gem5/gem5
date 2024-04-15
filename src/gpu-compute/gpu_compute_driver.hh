/*
 * Copyright (c) 2015-2018 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 * The GPUComputeDriver implements an HSADriver for an HSA AMD GPU
 * agent. Other GPU devices, or other HSA agents, should not derive
 * from this class. Instead device-specific implementations of an
 * HSADriver should be provided for each unique device.
 */

#ifndef __GPU_COMPUTE_GPU_COMPUTE_DRIVER_HH__
#define __GPU_COMPUTE_GPU_COMPUTE_DRIVER_HH__

#include <cassert>
#include <cstdint>
#include <set>
#include <unordered_map>

#include "base/addr_range_map.hh"
#include "base/types.hh"
#include "enums/GfxVersion.hh"
#include "mem/request.hh"
#include "sim/emul_driver.hh"

namespace gem5
{

struct GPUComputeDriverParams;
class GPUCommandProcessor;
class PortProxy;
class ThreadContext;

class GPUComputeDriver final : public EmulatedDriver
{
  public:
    typedef GPUComputeDriverParams Params;
    GPUComputeDriver(const Params &p);
    int ioctl(ThreadContext *tc, unsigned req, Addr ioc_buf) override;

    int open(ThreadContext *tc, int mode, int flags) override;
    Addr mmap(ThreadContext *tc, Addr start, uint64_t length, int prot,
              int tgt_flags, int tgt_fd, off_t offset) override;
    virtual void signalWakeupEvent(uint32_t event_id);
    void sleepCPU(ThreadContext *tc, uint32_t milliSecTimeout);
    /**
     * Called by the compute units right before a request is issued to ruby.
     * This uses our VMAs to correctly set the MTYPE on a per-request basis.
     * In real hardware, this is actually done through PTE bits in GPUVM.
     * Since we are running a single VM (x86 PT) system, the MTYPE bits aren't
     * available.  Adding GPUVM specific bits to x86 page tables probably
     * isn't the best way to proceed.  For now we just have the driver set
     * these until we implement a proper dual PT system.
     */
    void setMtype(RequestPtr req);

    int
    doorbellSize()
    {
        switch (gfxVersion) {
        case GfxVersion::gfx902:
            return 4;
        case GfxVersion::gfx900:
            // gfx900 supports large BAR, so it has a larger doorbell
            return 8;
        default:
            fatal("Invalid GPU type\n");
        }
        return 4;
    }

    class DriverWakeupEvent : public Event
    {
      public:
        DriverWakeupEvent(GPUComputeDriver *gpu_driver,
                          ThreadContext *thrd_cntxt)
            : driver(gpu_driver), tc(thrd_cntxt)
        {}

        void process() override;
        const char *description() const override;
        void scheduleWakeup(Tick wakeup_delay);

      private:
        GPUComputeDriver *driver;
        ThreadContext *tc;
    };

    class EventTableEntry
    {
      public:
        EventTableEntry()
            : mailBoxPtr(0), tc(nullptr), threadWaiting(false), setEvent(false)
        {}

        // Mail box pointer for this address. Current implementation does not
        // use this mailBoxPtr to notify events but directly calls
        // signalWakeupEvent from dispatcher (GPU) to notifiy events. So,
        // currently this mailBoxPtr is not used. But a future implementation
        // may communicate to the driver using mailBoxPtr.
        Addr mailBoxPtr;
        // Thread context waiting on this even. We do not support multiple
        // threads waiting on an event currently.
        ThreadContext *tc;
        // threadWaiting = true, if some thread context is waiting on this
        // event. A thread context waiting on this event is put to sleep.
        bool threadWaiting;
        // setEvent = true, if this event is triggered but when this event
        // triggered, no thread context was waiting on it. In the future, some
        // thread context will try to wait on this event but since event has
        // already happened, we will not allow that thread context to go to
        // sleep. The above mentioned scneario can happen when the waiting
        // thread and wakeup thread race on this event and the wakeup thread
        // beat the waiting thread at the driver.
        bool setEvent;
    };
    typedef class EventTableEntry ETEntry;

    GfxVersion
    getGfxVersion() const
    {
        return gfxVersion;
    }

  private:
    /**
     * GPU that is controlled by this driver.
     */
    GPUCommandProcessor *device;
    uint32_t queueId;
    bool isdGPU;
    GfxVersion gfxVersion;
    int dGPUPoolID;
    Addr eventPage;
    uint32_t eventSlotIndex;
    // Event table that keeps track of events. It is indexed with event ID.
    std::unordered_map<uint32_t, ETEntry> ETable;

    /**
     * VMA structures for GPUVM memory.
     */
    AddrRangeMap<Request::CacheCoherenceFlags, 1> gpuVmas;

    /**
     * Mtype bits {Cached, Read Write, Shared} for caches
     */
    enum MtypeFlags
    {
        SHARED = 0,
        READ_WRITE = 1,
        CACHED = 2,
        NUM_MTYPE_BITS
    };

    Request::CacheCoherenceFlags defaultMtype;

    // TCEvents map keeps trak of the events that can wakeup this thread. When
    // multiple events can wake up this thread, this data structure helps to
    // reset all events when one of those events wake up this thread. the
    // signal events that can wake up this thread are stored in signalEvents
    // whereas the timer wakeup event is stored in timerEvent.
    class EventList
    {
      public:
        EventList() : driver(nullptr), timerEvent(nullptr, nullptr) {}

        EventList(GPUComputeDriver *gpu_driver, ThreadContext *thrd_cntxt)
            : driver(gpu_driver), timerEvent(gpu_driver, thrd_cntxt)
        {}

        void
        clearEvents()
        {
            assert(driver);
            for (auto event : signalEvents) {
                assert(event < driver->eventSlotIndex);
                driver->ETable[event].tc = nullptr;
                driver->ETable[event].threadWaiting = false;
            }
            signalEvents.clear();
            if (timerEvent.scheduled()) {
                driver->deschedule(timerEvent);
            }
        }

        GPUComputeDriver *driver;
        DriverWakeupEvent timerEvent;
        // The set of events that can wake up the same thread.
        std::set<uint32_t> signalEvents;
    };

    std::unordered_map<ThreadContext *, EventList> TCEvents;

    /**
     * Register a region of host memory as uncacheable from the perspective
     * of the dGPU.
     */
    void registerUncacheableMemory(Addr start, Addr length);

    /**
     * The aperture (APE) base/limit pairs are set
     * statically at startup by the real KFD. AMD
     * x86_64 CPUs only use the areas in the 64b
     * address space where VA[63:47] == 0x1ffff or
     * VA[63:47] = 0. These methods generate the APE
     * base/limit pairs in exactly the same way as
     * the real KFD does, which ensures these APEs do
     * not fall into the CPU's address space
     *
     * see the macros in the KFD driver in the ROCm
     * Linux kernel source:
     *
     * drivers/gpu/drm/amd/amdkfd/kfd_flat_memory.c
     */
    Addr gpuVmApeBase(int gpuNum) const;
    Addr gpuVmApeLimit(Addr apeBase) const;
    Addr scratchApeBase(int gpuNum) const;
    Addr scratchApeBaseV9() const;
    Addr scratchApeLimit(Addr apeBase) const;
    Addr ldsApeBase(int gpuNum) const;
    Addr ldsApeBaseV9() const;
    Addr ldsApeLimit(Addr apeBase) const;

    /**
     * Allocate/deallocate GPUVM VMAs for tracking virtual address allocations
     * and properties on DGPUs.  For now, we use these to track MTYPE and to
     * be able to select which pages to unmap when the user provides us with
     * a handle during the free ioctl.
     */
    void allocateGpuVma(Request::CacheCoherenceFlags mtype, Addr start,
                        Addr length);
    Addr deallocateGpuVma(Addr start);

    void allocateQueue(PortProxy &mem_proxy, Addr ioc_buf_addr);
};

} // namespace gem5

#endif // __GPU_COMPUTE_GPU_COMPUTE_DRIVER_HH__
