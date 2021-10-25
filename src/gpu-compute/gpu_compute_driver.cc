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

#include "gpu-compute/gpu_compute_driver.hh"

#include <memory>

#include "arch/x86/page_size.hh"
#include "base/compiler.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "debug/GPUDriver.hh"
#include "debug/GPUShader.hh"
#include "dev/hsa/hsa_packet_processor.hh"
#include "dev/hsa/kfd_event_defines.h"
#include "dev/hsa/kfd_ioctl.h"
#include "gpu-compute/gpu_command_processor.hh"
#include "gpu-compute/shader.hh"
#include "mem/port_proxy.hh"
#include "mem/se_translating_port_proxy.hh"
#include "mem/translating_port_proxy.hh"
#include "params/GPUComputeDriver.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"
#include "sim/se_workload.hh"
#include "sim/syscall_emul_buf.hh"

namespace gem5
{

GPUComputeDriver::GPUComputeDriver(const Params &p)
    : EmulatedDriver(p), device(p.device), queueId(0),
      isdGPU(p.isdGPU), gfxVersion(p.gfxVersion), dGPUPoolID(p.dGPUPoolID),
      eventPage(0), eventSlotIndex(0)
{
    device->attachDriver(this);
    DPRINTF(GPUDriver, "Constructing KFD: device\n");

    // Convert the 3 bit mtype specified in Shader.py to the proper type
    // used for requests.
    std::bitset<MtypeFlags::NUM_MTYPE_BITS> mtype(p.m_type);
    if (mtype.test(MtypeFlags::SHARED)) {
        defaultMtype.set(Request::SHARED);
    }

    if (mtype.test(MtypeFlags::READ_WRITE)) {
        defaultMtype.set(Request::READ_WRITE);
    }

    if (mtype.test(MtypeFlags::CACHED)) {
        defaultMtype.set(Request::CACHED);
    }
}

const char*
GPUComputeDriver::DriverWakeupEvent::description() const
{
    return "DriverWakeupEvent";
}

/**
 * Create an FD entry for the KFD inside of the owning process.
 */
int
GPUComputeDriver::open(ThreadContext *tc, int mode, int flags)
{
    DPRINTF(GPUDriver, "Opened %s\n", filename);
    auto process = tc->getProcessPtr();
    auto device_fd_entry = std::make_shared<DeviceFDEntry>(this, filename);
    int tgt_fd = process->fds->allocFD(device_fd_entry);
    return tgt_fd;
}

/**
 * Currently, mmap() will simply setup a mapping for the associated
 * device's packet processor's doorbells and creates the event page.
 */
Addr
GPUComputeDriver::mmap(ThreadContext *tc, Addr start, uint64_t length,
                       int prot, int tgt_flags, int tgt_fd, off_t offset)
{
    auto process = tc->getProcessPtr();
    auto mem_state = process->memState;

    Addr pg_off = offset >> PAGE_SHIFT;
    Addr mmap_type = pg_off & KFD_MMAP_TYPE_MASK;
    DPRINTF(GPUDriver, "amdkfd mmap (start: %p, length: 0x%x,"
            "offset: 0x%x)\n", start, length, offset);

    switch(mmap_type) {
        case KFD_MMAP_TYPE_DOORBELL:
            DPRINTF(GPUDriver, "amdkfd mmap type DOORBELL offset\n");
            start = mem_state->extendMmap(length);
            process->pTable->map(start, device->hsaPacketProc().pioAddr,
                    length, false);
            break;
        case KFD_MMAP_TYPE_EVENTS:
            DPRINTF(GPUDriver, "amdkfd mmap type EVENTS offset\n");
            panic_if(start != 0,
                     "Start address should be provided by KFD\n");
            panic_if(length != 8 * KFD_SIGNAL_EVENT_LIMIT,
                     "Requested length %d, expected length %d; length "
                     "mismatch\n", length, 8* KFD_SIGNAL_EVENT_LIMIT);
            /**
             * We don't actually access these pages.  We just need to reserve
             * some VA space.  See commit id 5ce8abce for details on how
             * events are currently implemented.
             */
            if (!eventPage) {
                eventPage = mem_state->extendMmap(length);
                start = eventPage;
            }
            break;
        default:
            warn_once("Unrecognized kfd mmap type %llx\n", mmap_type);
            break;
    }

    return start;
}

/**
 * Forward relevant parameters to packet processor; queueId
 * is used to link doorbell. The queueIDs are not re-used
 * in current implementation, and we allocate only one page
 * (4096 bytes) for doorbells, so check if this queueID can
 * be mapped into that page.
 */
void
GPUComputeDriver::allocateQueue(PortProxy &mem_proxy, Addr ioc_buf)
{
    TypedBufferArg<kfd_ioctl_create_queue_args> args(ioc_buf);
    args.copyIn(mem_proxy);

    if ((doorbellSize() * queueId) > 4096) {
        fatal("%s: Exceeded maximum number of HSA queues allowed\n", name());
    }

    args->doorbell_offset = (KFD_MMAP_TYPE_DOORBELL |
        KFD_MMAP_GPU_ID(args->gpu_id)) << PAGE_SHIFT;

    // for vega offset needs to include exact value of doorbell
    if (doorbellSize())
        args->doorbell_offset += queueId * doorbellSize();

    args->queue_id = queueId++;
    auto &hsa_pp = device->hsaPacketProc();
    hsa_pp.setDeviceQueueDesc(args->read_pointer_address,
                              args->ring_base_address, args->queue_id,
                              args->ring_size, doorbellSize(), gfxVersion);
    args.copyOut(mem_proxy);
}

void
GPUComputeDriver::DriverWakeupEvent::scheduleWakeup(Tick wakeup_delay)
{
    assert(driver);
    driver->schedule(this, curTick() + wakeup_delay);
}

void
GPUComputeDriver::signalWakeupEvent(uint32_t event_id)
{
    panic_if(event_id >= eventSlotIndex,
        "Trying wakeup on an event that is not yet created\n");
    if (ETable[event_id].threadWaiting) {
        panic_if(!ETable[event_id].tc,
                 "No thread context to wake up\n");
        ThreadContext *tc = ETable[event_id].tc;
        DPRINTF(GPUDriver,
                "Signal event: Waking up CPU %d\n", tc->cpuId());
        // Remove events that can wakeup this thread
        TCEvents[tc].clearEvents();
        // Now wakeup this thread
        tc->activate();
    } else {
       // This may be a race condition between an ioctl call asking to wait on
       // this event and this signalWakeupEvent. Taking care of this race
       // condition here by setting the event here. The ioctl call should take
       // the necessary action when waiting on an already set event.  However,
       // this may be a genuine instance in which the runtime has decided not
       // to wait on this event. But since we cannot distinguish this case with
       // the race condition, we are any way setting the event.
       ETable[event_id].setEvent = true;
    }
}

void
GPUComputeDriver::DriverWakeupEvent::process()
{
    DPRINTF(GPUDriver,
            "Timer event: Waking up CPU %d\n", tc->cpuId());
    // Remove events that can wakeup this thread
    driver->TCEvents[tc].clearEvents();
    // Now wakeup this thread
    tc->activate();
}

int
GPUComputeDriver::ioctl(ThreadContext *tc, unsigned req, Addr ioc_buf)
{
    TranslatingPortProxy fs_proxy(tc);
    SETranslatingPortProxy se_proxy(tc);
    PortProxy &virt_proxy = FullSystem ? fs_proxy : se_proxy;
    auto process = tc->getProcessPtr();
    auto mem_state = process->memState;

    switch (req) {
        case AMDKFD_IOC_GET_VERSION:
          {
            DPRINTF(GPUDriver, "ioctl: AMDKFD_IOC_GET_VERSION\n");

            TypedBufferArg<kfd_ioctl_get_version_args> args(ioc_buf);
            args->major_version = KFD_IOCTL_MAJOR_VERSION;
            args->minor_version = KFD_IOCTL_MINOR_VERSION;

            args.copyOut(virt_proxy);
          }
          break;
        case AMDKFD_IOC_CREATE_QUEUE:
          {
            DPRINTF(GPUDriver, "ioctl: AMDKFD_IOC_CREATE_QUEUE\n");

            allocateQueue(virt_proxy, ioc_buf);

            DPRINTF(GPUDriver, "Creating queue %d\n", queueId);
          }
          break;
        case AMDKFD_IOC_DESTROY_QUEUE:
          {
            TypedBufferArg<kfd_ioctl_destroy_queue_args> args(ioc_buf);
            args.copyIn(virt_proxy);
            DPRINTF(GPUDriver, "ioctl: AMDKFD_IOC_DESTROY_QUEUE;" \
                    "queue offset %d\n", args->queue_id);
            device->hsaPacketProc().unsetDeviceQueueDesc(args->queue_id,
                                                         doorbellSize());
          }
          break;
        case AMDKFD_IOC_SET_MEMORY_POLICY:
          {
            /**
             * This is where the runtime requests MTYPE from an aperture.
             * Basically, the globally memory aperture is divided up into
             * a default aperture and an alternate aperture each of which have
             * their own MTYPE policies.  This is done to mark a small piece
             * of the global memory as uncacheable.  Host memory mappings will
             * be carved out of this uncacheable aperture, which is how they
             * implement 'coherent' host/device memory on dGPUs.
             *
             * TODO: Need to reflect per-aperture MTYPE policies based on this
             * call.
             *
             */
            warn("unimplemented ioctl: AMDKFD_IOC_SET_MEMORY_POLICY\n");
          }
          break;
        case AMDKFD_IOC_GET_CLOCK_COUNTERS:
          {
            DPRINTF(GPUDriver, "ioctl: AMDKFD_IOC_GET_CLOCK_COUNTERS\n");

            TypedBufferArg<kfd_ioctl_get_clock_counters_args> args(ioc_buf);
            args.copyIn(virt_proxy);

            // Set nanosecond resolution
            args->system_clock_freq = 1000000000;

            /**
             * Derive all clock counters based on the tick. All
             * device clocks are identical and perfectly in sync.
             */
            uint64_t elapsed_nsec = curTick() / sim_clock::as_int::ns;
            args->gpu_clock_counter = elapsed_nsec;
            args->cpu_clock_counter = elapsed_nsec;
            args->system_clock_counter = elapsed_nsec;

            args.copyOut(virt_proxy);
          }
          break;
        case AMDKFD_IOC_GET_PROCESS_APERTURES:
          {
            DPRINTF(GPUDriver, "ioctl: AMDKFD_IOC_GET_PROCESS_APERTURES\n");

            TypedBufferArg<kfd_ioctl_get_process_apertures_args> args(ioc_buf);
            args->num_of_nodes = 1;

            /**
             * Set the GPUVM/LDS/Scratch APEs exactly as they
             * are in the real driver, see the KFD driver
             * in the ROCm Linux kernel source:
             * drivers/gpu/drm/amd/amdkfd/kfd_flat_memory.c
             */
            for (int i = 0; i < args->num_of_nodes; ++i) {
                /**
                 * While the GPU node numbers start at 0, we add 1
                 * to force the count to start at 1. This is to
                 * ensure that the base/limit addresses are
                 * calculated correctly.
                 */

                switch (gfxVersion) {
                  case GfxVersion::gfx801:
                  case GfxVersion::gfx803:
                    args->process_apertures[i].scratch_base =
                        scratchApeBase(i + 1);
                    args->process_apertures[i].lds_base =
                        ldsApeBase(i + 1);
                    break;
                  case GfxVersion::gfx900:
                  case GfxVersion::gfx902:
                    args->process_apertures[i].scratch_base =
                        scratchApeBaseV9();
                    args->process_apertures[i].lds_base =
                        ldsApeBaseV9();
                    break;
                  default:
                    fatal("Invalid gfx version\n");
                }

                // GFX8 and GFX9 set lds and scratch limits the same way
                args->process_apertures[i].scratch_limit =
                    scratchApeLimit(args->process_apertures[i].scratch_base);

                args->process_apertures[i].lds_limit =
                    ldsApeLimit(args->process_apertures[i].lds_base);

                switch (gfxVersion) {
                  case GfxVersion::gfx801:
                    args->process_apertures[i].gpuvm_base =
                        gpuVmApeBase(i + 1);
                    args->process_apertures[i].gpuvm_limit =
                        gpuVmApeLimit(args->process_apertures[i].gpuvm_base);
                    break;
                  case GfxVersion::gfx803:
                  case GfxVersion::gfx900:
                  case GfxVersion::gfx902:
                    // Taken from SVM_USE_BASE in Linux kernel
                    args->process_apertures[i].gpuvm_base = 0x1000000ull;
                    // Taken from AMDGPU_GMC_HOLE_START in Linux kernel
                    args->process_apertures[i].gpuvm_limit =
                        0x0000800000000000ULL - 1;
                    break;
                  default:
                    fatal("Invalid gfx version");
                }

                // NOTE: Must match ID populated by hsaTopology.py
                //
                // https://github.com/RadeonOpenCompute/ROCK-Kernel-Driver/
                // blob/6a986c0943e9acd8c4c0cf2a9d510ff42167b43f/include/uapi/
                // linux/kfd_ioctl.h#L564
                //
                // The gpu_id is a device identifier used by the driver for
                // ioctls that allocate arguments. Each device has an unique
                // id composed out of a non-zero base and an offset.
                if (isdGPU) {
                    switch (gfxVersion) {
                      case GfxVersion::gfx803:
                        args->process_apertures[i].gpu_id = 50156;
                        break;
                      case GfxVersion::gfx900:
                        args->process_apertures[i].gpu_id = 22124;
                        break;
                      default:
                        fatal("Invalid gfx version for dGPU\n");
                    }
                } else {
                    switch (gfxVersion) {
                      case GfxVersion::gfx801:
                      case GfxVersion::gfx902:
                        args->process_apertures[i].gpu_id = 2765;
                        break;
                      default:
                        fatal("Invalid gfx version for APU\n");
                    }
                }

                DPRINTF(GPUDriver, "GPUVM base for node[%i] = %#x\n", i,
                        args->process_apertures[i].gpuvm_base);
                DPRINTF(GPUDriver, "GPUVM limit for node[%i] = %#x\n", i,
                        args->process_apertures[i].gpuvm_limit);

                DPRINTF(GPUDriver, "LDS base for node[%i] = %#x\n", i,
                        args->process_apertures[i].lds_base);
                DPRINTF(GPUDriver, "LDS limit for node[%i] = %#x\n", i,
                        args->process_apertures[i].lds_limit);

                DPRINTF(GPUDriver, "Scratch base for node[%i] = %#x\n", i,
                        args->process_apertures[i].scratch_base);
                DPRINTF(GPUDriver, "Scratch limit for node[%i] = %#x\n", i,
                        args->process_apertures[i].scratch_limit);

                /**
                 * The CPU's 64b address space can only use the
                 * areas with VA[63:47] == 0x1ffff or VA[63:47] == 0,
                 * therefore we must ensure that the apertures do not
                 * fall in the CPU's address space.
                 */
                assert(bits<Addr>(args->process_apertures[i].scratch_base, 63,
                       47) != 0x1ffff);
                assert(bits<Addr>(args->process_apertures[i].scratch_base, 63,
                       47) != 0);
                assert(bits<Addr>(args->process_apertures[i].scratch_limit, 63,
                       47) != 0x1ffff);
                assert(bits<Addr>(args->process_apertures[i].scratch_limit, 63,
                       47) != 0);
                assert(bits<Addr>(args->process_apertures[i].lds_base, 63,
                       47) != 0x1ffff);
                assert(bits<Addr>(args->process_apertures[i].lds_base, 63,
                       47) != 0);
                assert(bits<Addr>(args->process_apertures[i].lds_limit, 63,
                       47) != 0x1ffff);
                assert(bits<Addr>(args->process_apertures[i].lds_limit, 63,
                       47) != 0);
            }

            args.copyOut(virt_proxy);
          }
          break;
        case AMDKFD_IOC_UPDATE_QUEUE:
          {
            warn("unimplemented ioctl: AMDKFD_IOC_UPDATE_QUEUE\n");
          }
          break;
        case AMDKFD_IOC_CREATE_EVENT:
          {
            DPRINTF(GPUDriver, "ioctl: AMDKFD_IOC_CREATE_EVENT\n");

            TypedBufferArg<kfd_ioctl_create_event_args> args(ioc_buf);
            args.copyIn(virt_proxy);
            if (args->event_type != KFD_IOC_EVENT_SIGNAL) {
                warn("Signal events are only supported currently\n");
            } else if (eventSlotIndex == SLOTS_PER_PAGE) {
                fatal("Signal event wasn't created; signal limit reached\n");
            }
            // Currently, we allocate only one signal_page for events.
            // Note that this signal page is of size 8 * KFD_SIGNAL_EVENT_LIMIT
            uint64_t page_index = 0;
            args->event_page_offset = (page_index | KFD_MMAP_TYPE_EVENTS);
            args->event_page_offset <<= PAGE_SHIFT;
            // TODO: Currently we support only signal events, hence using
            // the same ID for both signal slot and event slot
            args->event_slot_index = eventSlotIndex;
            args->event_id = eventSlotIndex++;
            args->event_trigger_data = args->event_id;
            DPRINTF(GPUDriver, "amdkfd create events"
                    "(event_id: 0x%x, offset: 0x%x)\n",
                    args->event_id, args->event_page_offset);
            // Since eventSlotIndex is increased everytime a new event is
            // created ETable at eventSlotIndex(event_id) is guaranteed to be
            // empty. In a future implementation that reuses deleted event_ids,
            // we should check if event table at this
            // eventSlotIndex(event_id) is empty before inserting a new event
            // table entry
            ETable.emplace(std::pair<uint32_t, ETEntry>(args->event_id, {}));
            args.copyOut(virt_proxy);
          }
          break;
        case AMDKFD_IOC_DESTROY_EVENT:
          {
            DPRINTF(GPUDriver, "ioctl: AMDKFD_IOC_DESTROY_EVENT\n");
            TypedBufferArg<kfd_ioctl_destroy_event_args> args(ioc_buf);
            args.copyIn(virt_proxy);
            DPRINTF(GPUDriver, "amdkfd destroying event %d\n", args->event_id);
            fatal_if(ETable.count(args->event_id) == 0,
                     "Event ID invalid, cannot destroy this event\n");
            ETable.erase(args->event_id);
          }
          break;
        case AMDKFD_IOC_SET_EVENT:
          {
            DPRINTF(GPUDriver, "ioctl: AMDKFD_IOC_SET_EVENTS\n");
            TypedBufferArg<kfd_ioctl_set_event_args> args(ioc_buf);
            args.copyIn(virt_proxy);
            DPRINTF(GPUDriver, "amdkfd set event %d\n", args->event_id);
            fatal_if(ETable.count(args->event_id) == 0,
                     "Event ID invlaid, cannot set this event\n");
            ETable[args->event_id].setEvent = true;
            signalWakeupEvent(args->event_id);
          }
          break;
        case AMDKFD_IOC_RESET_EVENT:
          {
            warn("unimplemented ioctl: AMDKFD_IOC_RESET_EVENT\n");
          }
          break;
        case AMDKFD_IOC_WAIT_EVENTS:
          {
            DPRINTF(GPUDriver, "ioctl: AMDKFD_IOC_WAIT_EVENTS\n");
            TypedBufferArg<kfd_ioctl_wait_events_args> args(ioc_buf);
            args.copyIn(virt_proxy);
            kfd_event_data *events =
                (kfd_event_data *)args->events_ptr;
            DPRINTF(GPUDriver, "amdkfd wait for events"
                    "(wait on all: %d, timeout : %d, num_events: %s)\n",
                    args->wait_for_all, args->timeout, args->num_events);
            panic_if(args->wait_for_all != 0 && args->num_events > 1,
                    "Wait for all events not supported\n");
            bool should_sleep = true;
            if (TCEvents.count(tc) == 0) {
                // This thread context trying to wait on an event for the first
                // time, initialize it.
                TCEvents.emplace(std::piecewise_construct, std::make_tuple(tc),
                                 std::make_tuple(this, tc));
                DPRINTF(GPUDriver, "\tamdkfd creating event list"
                        " for thread  %d\n", tc->cpuId());
            }
            panic_if(TCEvents[tc].signalEvents.size() != 0,
                     "There are %d events that put this thread to sleep,"
                     " this thread should not be running\n",
                     TCEvents[tc].signalEvents.size());
            for (int i = 0; i < args->num_events; i++) {
                panic_if(!events,
                         "Event pointer invalid\n");
                Addr eventDataAddr = (Addr)(events + i);
                TypedBufferArg<kfd_event_data> EventData(
                    eventDataAddr, sizeof(kfd_event_data));
                EventData.copyIn(virt_proxy);
                DPRINTF(GPUDriver,
                        "\tamdkfd wait for event %d\n", EventData->event_id);
                panic_if(ETable.count(EventData->event_id) == 0,
                         "Event ID invalid, cannot set this event\n");
                if (ETable[EventData->event_id].threadWaiting)
                         warn("Multiple threads waiting on the same event\n");
                if (ETable[EventData->event_id].setEvent) {
                    // If event is already set, the event has already happened.
                    // Just unset the event and dont put this thread to sleep.
                    ETable[EventData->event_id].setEvent = false;
                    should_sleep = false;
                }
                if (should_sleep) {
                    // Put this thread to sleep
                    ETable[EventData->event_id].threadWaiting = true;
                    ETable[EventData->event_id].tc = tc;
                    TCEvents[tc].signalEvents.insert(EventData->event_id);
                }
            }

            // TODO: Return the correct wait_result back. Currently, returning
            // success for both KFD_WAIT_TIMEOUT and KFD_WAIT_COMPLETE.
            // Ideally, this needs to be done after the event is triggered and
            // after the thread is woken up.
            args->wait_result = 0;
            args.copyOut(virt_proxy);
            if (should_sleep) {
                // Put this thread to sleep
                sleepCPU(tc, args->timeout);
            } else {
                // Remove events that tried to put this thread to sleep
                TCEvents[tc].clearEvents();
            }
          }
          break;
        case AMDKFD_IOC_DBG_REGISTER:
          {
            warn("unimplemented ioctl: AMDKFD_IOC_DBG_REGISTER\n");
          }
          break;
        case AMDKFD_IOC_DBG_UNREGISTER:
          {
            warn("unimplemented ioctl: AMDKFD_IOC_DBG_UNREGISTER\n");
          }
          break;
        case AMDKFD_IOC_DBG_ADDRESS_WATCH:
          {
            warn("unimplemented ioctl: AMDKFD_IOC_DBG_ADDRESS_WATCH\n");
          }
          break;
        case AMDKFD_IOC_DBG_WAVE_CONTROL:
          {
            warn("unimplemented ioctl: AMDKFD_IOC_DBG_WAVE_CONTROL\n");
          }
          break;
        case AMDKFD_IOC_SET_SCRATCH_BACKING_VA:
          {
            warn("unimplemented ioctl: AMDKFD_IOC_SET_SCRATCH_BACKING_VA\n");
          }
          break;
        case AMDKFD_IOC_GET_TILE_CONFIG:
          {
            warn("unimplemented ioctl: AMDKFD_IOC_GET_TILE_CONFIG\n");
          }
          break;
        case AMDKFD_IOC_SET_TRAP_HANDLER:
          {
            warn("unimplemented ioctl: AMDKFD_IOC_SET_TRAP_HANDLER\n");
          }
          break;
        case AMDKFD_IOC_GET_PROCESS_APERTURES_NEW:
          {
            DPRINTF(GPUDriver,
                    "ioctl: AMDKFD_IOC_GET_PROCESS_APERTURES_NEW\n");

            TypedBufferArg<kfd_ioctl_get_process_apertures_new_args>
                ioc_args(ioc_buf);

            ioc_args.copyIn(virt_proxy);
            ioc_args->num_of_nodes = 1;

            for (int i = 0; i < ioc_args->num_of_nodes; ++i) {
                TypedBufferArg<kfd_process_device_apertures> ape_args
                    (ioc_args->kfd_process_device_apertures_ptr);

                switch (gfxVersion) {
                  case GfxVersion::gfx801:
                  case GfxVersion::gfx803:
                    ape_args->scratch_base = scratchApeBase(i + 1);
                    ape_args->lds_base = ldsApeBase(i + 1);
                    break;
                  case GfxVersion::gfx900:
                  case GfxVersion::gfx902:
                    ape_args->scratch_base = scratchApeBaseV9();
                    ape_args->lds_base = ldsApeBaseV9();
                    break;
                  default:
                    fatal("Invalid gfx version\n");
                }

                // GFX8 and GFX9 set lds and scratch limits the same way
                ape_args->scratch_limit =
                    scratchApeLimit(ape_args->scratch_base);
                ape_args->lds_limit = ldsApeLimit(ape_args->lds_base);

                switch (gfxVersion) {
                  case GfxVersion::gfx801:
                    ape_args->gpuvm_base = gpuVmApeBase(i + 1);
                    ape_args->gpuvm_limit =
                        gpuVmApeLimit(ape_args->gpuvm_base);
                    break;
                  case GfxVersion::gfx803:
                  case GfxVersion::gfx900:
                  case GfxVersion::gfx902:
                    // Taken from SVM_USE_BASE in Linux kernel
                    ape_args->gpuvm_base = 0x1000000ull;
                    // Taken from AMDGPU_GMC_HOLE_START in Linux kernel
                    ape_args->gpuvm_limit = 0x0000800000000000ULL - 1;
                    break;
                  default:
                    fatal("Invalid gfx version\n");
                }

                // NOTE: Must match ID populated by hsaTopology.py
                if (isdGPU) {
                    switch (gfxVersion) {
                      case GfxVersion::gfx803:
                        ape_args->gpu_id = 50156;
                        break;
                      case GfxVersion::gfx900:
                        ape_args->gpu_id = 22124;
                        break;
                      default:
                        fatal("Invalid gfx version for dGPU\n");
                    }
                } else {
                    switch (gfxVersion) {
                      case GfxVersion::gfx801:
                      case GfxVersion::gfx902:
                        ape_args->gpu_id = 2765;
                        break;
                      default:
                        fatal("Invalid gfx version for APU\n");
                    }
                }

                assert(bits<Addr>(ape_args->scratch_base, 63, 47) != 0x1ffff);
                assert(bits<Addr>(ape_args->scratch_base, 63, 47) != 0);
                assert(bits<Addr>(ape_args->scratch_limit, 63, 47) != 0x1ffff);
                assert(bits<Addr>(ape_args->scratch_limit, 63, 47) != 0);
                assert(bits<Addr>(ape_args->lds_base, 63, 47) != 0x1ffff);
                assert(bits<Addr>(ape_args->lds_base, 63, 47) != 0);
                assert(bits<Addr>(ape_args->lds_limit, 63, 47) != 0x1ffff);
                assert(bits<Addr>(ape_args->lds_limit, 63, 47) != 0);

                ape_args.copyOut(virt_proxy);
            }

            ioc_args.copyOut(virt_proxy);
          }
          break;
        case AMDKFD_IOC_ACQUIRE_VM:
          {
            warn("unimplemented ioctl: AMDKFD_IOC_ACQUIRE_VM\n");
          }
          break;
         /**
         * In real hardware, this IOCTL maps host memory, dGPU memory, or dGPU
         * doorbells into GPUVM space. Essentially, ROCm implements SVM by
         * carving out a region of free VA space that both the host and GPUVM
         * can agree upon.  The entire GPU VA space is reserved on the host
         * using a fixed mmap at a low VA range that is also directly
         * accessable by the GPU's limited number of VA bits.  When we actually
         * call memory allocation later in the program, this IOCTL is invoked
         * to create BOs/VMAs in the driver and bind them to physical
         * memory/doorbells.
         *
         * For gem5, we don't need to carve out any GPUVM space here (we don't
         * support GPUVM and use host page tables on the GPU directly). We can
         * can just use the existing host SVM region. We comment on each memory
         * type seperately.
         */
        case AMDKFD_IOC_ALLOC_MEMORY_OF_GPU:
          {
            DPRINTF(GPUDriver, "ioctl: AMDKFD_IOC_ALLOC_MEMORY_OF_GPU\n");
            TypedBufferArg<kfd_ioctl_alloc_memory_of_gpu_args> args(ioc_buf);
            args.copyIn(virt_proxy);

            assert(isdGPU || gfxVersion == GfxVersion::gfx902);
            assert((args->va_addr % X86ISA::PageBytes) == 0);
            [[maybe_unused]] Addr mmap_offset = 0;

            Request::CacheCoherenceFlags mtype = defaultMtype;
            Addr pa_addr = 0;

            int npages = divCeil(args->size, (int64_t)X86ISA::PageBytes);
            bool cacheable = true;

            if (KFD_IOC_ALLOC_MEM_FLAGS_VRAM & args->flags) {
                DPRINTF(GPUDriver, "amdkfd allocation type: VRAM\n");
                args->mmap_offset = args->va_addr;
                // VRAM allocations are device memory mapped into GPUVM
                // space.
                //
                // We can't rely on the lazy host allocator (fixupFault) to
                // handle this mapping since it needs to be placed in dGPU
                // framebuffer memory.  The lazy allocator will try to place
                // this in host memory.
                //
                // TODO: We don't have the appropriate bifurcation of the
                // physical address space with different memory controllers
                // yet.  This is where we will explicitly add the PT maps to
                // dGPU memory in the future.
                //
                // Bind the VA space to the dGPU physical memory pool.  Mark
                // this region as Uncacheable.  The Uncacheable flag is only
                // really used by the CPU and is ignored by the GPU. We mark
                // this as uncacheable from the CPU so that we can implement
                // direct CPU framebuffer access similar to what we currently
                // offer in real HW through the so-called Large BAR feature.
                pa_addr = process->seWorkload->allocPhysPages(
                        npages, dGPUPoolID);
                //
                // TODO: Uncacheable accesses need to be supported by the
                // CPU-side protocol for this to work correctly.  I believe
                // it only works right now if the physical memory is MMIO
                cacheable = false;

                DPRINTF(GPUDriver, "Mapping VA %p to framebuffer PA %p size "
                        "%d\n", args->va_addr, pa_addr, args->size);

            } else if (KFD_IOC_ALLOC_MEM_FLAGS_USERPTR & args->flags) {
                DPRINTF(GPUDriver, "amdkfd allocation type: USERPTR\n");
                mmap_offset = args->mmap_offset;
                // USERPTR allocations are system memory mapped into GPUVM
                // space.  The user provides the driver with the pointer.
                pa_addr = process->seWorkload->allocPhysPages(npages);

                DPRINTF(GPUDriver, "Mapping VA %p to framebuffer PA %p size "
                        "%d\n", args->va_addr, pa_addr, args->size);

                // If the HSA runtime requests system coherent memory, than we
                // need to explicity mark this region as uncacheable from the
                // perspective of the GPU.
                if (args->flags & KFD_IOC_ALLOC_MEM_FLAGS_COHERENT)
                    mtype.clear();

            } else if (KFD_IOC_ALLOC_MEM_FLAGS_GTT & args->flags) {
                DPRINTF(GPUDriver, "amdkfd allocation type: GTT\n");
                args->mmap_offset = args->va_addr;
                // GTT allocations are system memory mapped into GPUVM space.
                // It's different than a USERPTR allocation since the driver
                // itself allocates the physical memory on the host.
                //
                // We will lazily map it into host memory on first touch.  The
                // fixupFault will find the original SVM aperture mapped to the
                // host.
                pa_addr = process->seWorkload->allocPhysPages(npages);

                DPRINTF(GPUDriver, "Mapping VA %p to framebuffer PA %p size "
                        "%d\n", args->va_addr, pa_addr, args->size);

                // If the HSA runtime requests system coherent memory, than we
                // need to explicity mark this region as uncacheable from the
                // perspective of the GPU.
                if (args->flags & KFD_IOC_ALLOC_MEM_FLAGS_COHERENT)
                    mtype.clear();

                // Note that for GTT the thunk layer needs to call mmap on the
                // driver FD later if it wants the host to have access to this
                // memory (which it probably does).  This will be ignored.
            } else if (KFD_IOC_ALLOC_MEM_FLAGS_DOORBELL & args->flags) {
                DPRINTF(GPUDriver, "amdkfd allocation type: DOORBELL\n");
                // DOORBELL allocations are the queue doorbells that are
                // memory mapped into GPUVM space.
                //
                // Explicitly map this virtual address to our PIO doorbell
                // interface in the page tables (non-cacheable)
                pa_addr = device->hsaPacketProc().pioAddr;
                cacheable = false;
            }

            DPRINTF(GPUDriver, "amdkfd allocation arguments: va_addr %p "
                    "size %lu, mmap_offset %p, gpu_id %d\n",
                    args->va_addr, args->size, mmap_offset, args->gpu_id);

            // Bind selected physical memory to provided virtual address range
            // in X86 page tables.
            process->pTable->map(args->va_addr, pa_addr, args->size,
                cacheable);

            // We keep track of allocated regions of GPU mapped memory,
            // just like the driver would.  This allows us to provide the
            // user with a unique handle for a given allocation.  The user
            // will only provide us with a handle after allocation and expect
            // us to be able to use said handle to extract all the properties
            // of the region.
            //
            // This is a simplified version of regular system VMAs, but for
            // GPUVM space (none of the clobber/remap nonsense we find in real
            // OS managed memory).
            allocateGpuVma(mtype, args->va_addr, args->size);

            // Used by the runtime to uniquely identify this allocation.
            // We can just use the starting address of the VMA region.
            args->handle= args->va_addr;
            args.copyOut(virt_proxy);
          }
          break;
        case AMDKFD_IOC_FREE_MEMORY_OF_GPU:
          {
            DPRINTF(GPUDriver, "ioctl: AMDKFD_IOC_FREE_MEMORY_OF_GPU\n");
            TypedBufferArg<kfd_ioctl_free_memory_of_gpu_args> args(ioc_buf);
            args.copyIn(virt_proxy);

            assert(isdGPU);
            DPRINTF(GPUDriver, "amdkfd free arguments: handle %p ",
                    args->handle);

            // We don't recycle physical pages in SE mode
            Addr size = deallocateGpuVma(args->handle);
            process->pTable->unmap(args->handle, size);

            // TODO: IOMMU and GPUTLBs do not seem to correctly support
            // shootdown.  This is also a potential issue for APU systems
            // that perform unmap or remap with system memory.
            tc->getMMUPtr()->flushAll();

            args.copyOut(virt_proxy);
          }
          break;
        /**
         * Called to map an already allocated region of memory to this GPU's
         * GPUVM VA space.  We don't need to implement this in the simulator
         * since we only have a single VM system.  If the region has already
         * been allocated somewhere like the CPU, then it's already visible
         * to the device.
         */
        case AMDKFD_IOC_MAP_MEMORY_TO_GPU:
          {
            warn("unimplemented ioctl: AMDKFD_IOC_MAP_MEMORY_TO_GPU\n");
          }
          break;
        case AMDKFD_IOC_UNMAP_MEMORY_FROM_GPU:
          {
            warn("unimplemented ioctl: AMDKFD_IOC_UNMAP_MEMORY_FROM_GPU\n");
          }
          break;
        case AMDKFD_IOC_SET_CU_MASK:
          {
            warn("unimplemented ioctl: AMDKFD_IOC_SET_CU_MASK\n");
          }
          break;
        case AMDKFD_IOC_GET_QUEUE_WAVE_STATE:
          {
            warn("unimplemented ioctl: AMDKFD_IOC_GET_QUEUE_WAVE_STATE\n");
          }
          break;
        case AMDKFD_IOC_GET_DMABUF_INFO:
          {
            warn("unimplemented ioctl: AMDKFD_IOC_GET_DMABUF_INFO\n");
          }
          break;
        case AMDKFD_IOC_IMPORT_DMABUF:
          {
            warn("unimplemented ioctl: AMDKFD_IOC_IMPORT_DMABUF\n");
          }
          break;
        case AMDKFD_IOC_ALLOC_QUEUE_GWS:
          {
            warn("unimplemented ioctl: AMDKFD_IOC_ALLOC_QUEUE_GWS\n");
          }
          break;
        case AMDKFD_IOC_SMI_EVENTS:
          {
            warn("unimplemented ioctl: AMDKFD_IOC_SMI_EVENTS\n");
          }
          break;
        default:
          fatal("%s: bad ioctl %d\n", req);
          break;
    }
    return 0;
}

void
GPUComputeDriver::sleepCPU(ThreadContext *tc, uint32_t milliSecTimeout)
{
    // Convert millisecs to ticks
    Tick wakeup_delay((uint64_t)milliSecTimeout * 1000000000);
    assert(TCEvents.count(tc) == 1);
    TCEvents[tc].timerEvent.scheduleWakeup(wakeup_delay);
    tc->suspend();
    DPRINTF(GPUDriver,
            "CPU %d is put to sleep\n", tc->cpuId());
}

Addr
GPUComputeDriver::gpuVmApeBase(int gpuNum) const
{
    return ((Addr)gpuNum << 61) + 0x1000000000000L;
}

Addr
GPUComputeDriver::gpuVmApeLimit(Addr apeBase) const
{
    return (apeBase & 0xFFFFFF0000000000UL) | 0xFFFFFFFFFFL;
}

Addr
GPUComputeDriver::scratchApeBase(int gpuNum) const
{
    return ((Addr)gpuNum << 61) + 0x100000000L;
}

// Used for GFX9 devices
// From drivers/gpu/drm/amd/amdkfd/kfd_flat_memory.c in the Linux kernel
Addr
GPUComputeDriver::scratchApeBaseV9() const
{
    return ((Addr)0x1 << 48);
}

Addr
GPUComputeDriver::scratchApeLimit(Addr apeBase) const
{
    return (apeBase & 0xFFFFFFFF00000000UL) | 0xFFFFFFFF;
}

Addr
GPUComputeDriver::ldsApeBase(int gpuNum) const
{
    return ((Addr)gpuNum << 61) + 0x0;
}

//Used for GFX9 devices
// From drivers/gpu/drm/amd/amdkfd/kfd_flat_memory.c in the Linux kernel
Addr
GPUComputeDriver::ldsApeBaseV9() const
{
    return ((Addr)0x2 << 48);
}

Addr
GPUComputeDriver::ldsApeLimit(Addr apeBase) const
{
    return (apeBase & 0xFFFFFFFF00000000UL) | 0xFFFFFFFF;
}

void
GPUComputeDriver::allocateGpuVma(Request::CacheCoherenceFlags mtype,
                                 Addr start, Addr length)
{
    AddrRange range = AddrRange(start, start + length);
    DPRINTF(GPUDriver, "Registering [%p - %p] with MTYPE %d\n",
            range.start(), range.end(), mtype);
    fatal_if(gpuVmas.insert(range, mtype) == gpuVmas.end(),
             "Attempted to double register Mtypes for [%p - %p]\n",
             range.start(), range.end());
}

Addr
GPUComputeDriver::deallocateGpuVma(Addr start)
{
    auto vma = gpuVmas.contains(start);
    assert(vma != gpuVmas.end());
    assert((vma->first.start() == start));
    Addr size = vma->first.size();
    DPRINTF(GPUDriver, "Unregistering [%p - %p]\n", vma->first.start(),
            vma->first.end());
    gpuVmas.erase(vma);
    return size;
}

void
GPUComputeDriver::setMtype(RequestPtr req)
{
    // If we are a dGPU then set the MTYPE from our VMAs.
    if (isdGPU) {
        assert(!FullSystem);
        AddrRange range = RangeSize(req->getVaddr(), req->getSize());
        auto vma = gpuVmas.contains(range);
        assert(vma != gpuVmas.end());
        DPRINTF(GPUShader, "Setting req from [%p - %p] MTYPE %d\n"
                "%d\n", range.start(), range.end(), vma->second);
        req->setCacheCoherenceFlags(vma->second);
    // APUs always get the default MTYPE
    } else {
        req->setCacheCoherenceFlags(defaultMtype);
    }
}

} // namespace gem5
