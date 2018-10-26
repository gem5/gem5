/*
 * Copyright (c) 2015-2018 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
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

#include "dev/hsa/hsa_driver.hh"

#include "base/trace.hh"
#include "debug/HSADriver.hh"
#include "dev/hsa/hsa_device.hh"
#include "dev/hsa/hsa_packet_processor.hh"
#include "dev/hsa/kfd_event_defines.h"
#include "dev/hsa/kfd_ioctl.h"
#include "params/HSADriver.hh"
#include "sim/process.hh"
#include "sim/proxy_ptr.hh"
#include "sim/syscall_emul_buf.hh"

HSADriver::HSADriver(const HSADriverParams &p)
    : EmulatedDriver(p), device(p.device), queueId(0)
{
}

/**
 * Create an FD entry for the KFD inside of the owning process.
 */
int
HSADriver::open(ThreadContext *tc, int mode, int flags)
{
    DPRINTF(HSADriver, "Opened %s\n", filename);
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
HSADriver::mmap(ThreadContext *tc, Addr start, uint64_t length, int prot,
                int tgt_flags, int tgt_fd, off_t offset)
{
    auto process = tc->getProcessPtr();
    auto mem_state = process->memState;

    Addr pg_off = offset >> PAGE_SHIFT;
    Addr mmap_type = pg_off & KFD_MMAP_TYPE_MASK;
    DPRINTF(HSADriver, "amdkfd mmap (start: %p, length: 0x%x,"
            "offset: 0x%x)\n", start, length, offset);

    switch (mmap_type) {
        case KFD_MMAP_TYPE_DOORBELL:
            DPRINTF(HSADriver, "amdkfd mmap type DOORBELL offset\n");
            start = mem_state->extendMmap(length);
            process->pTable->map(start, device->hsaPacketProc().pioAddr,
                    length, false);
            break;
        case KFD_MMAP_TYPE_EVENTS:
            DPRINTF(HSADriver, "amdkfd mmap type EVENTS offset\n");
            panic_if(start != 0,
                     "Start address should be provided by KFD\n");
            panic_if(length != 8 * KFD_SIGNAL_EVENT_LIMIT,
                     "Requested length %d, expected length %d; length "
                     "mismatch\n", length, 8 * KFD_SIGNAL_EVENT_LIMIT);
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
 * Forward relevant parameters to packet processor; queueID
 * is used to link doorbell. The queueIDs are not re-used
 * in current implementation, and we allocate only one page
 * (4096 bytes) for doorbells, so check if this queue ID can
 * be mapped into that page.
 */
void
HSADriver::allocateQueue(ThreadContext *tc, Addr ioc_buf)
{
    VPtr<kfd_ioctl_create_queue_args> args(ioc_buf, tc);

    if (queueId >= 0x1000) {
        fatal("%s: Exceeded maximum number of HSA queues allowed\n", name());
    }

    args->doorbell_offset = (KFD_MMAP_TYPE_DOORBELL |
        KFD_MMAP_GPU_ID(args->gpu_id)) << PAGE_SHIFT;

    args->queue_id = queueId++;
    auto &hsa_pp = device->hsaPacketProc();
    hsa_pp.setDeviceQueueDesc(args->read_pointer_address,
                              args->ring_base_address, args->queue_id,
                              args->ring_size);
}

const char*
HSADriver::DriverWakeupEvent::description() const
{
    return "DriverWakeupEvent";
}

void
HSADriver::DriverWakeupEvent::scheduleWakeup(Tick wakeup_delay)
{
    assert(driver);
    driver->schedule(this, curTick() + wakeup_delay);
}

void
HSADriver::signalWakeupEvent(uint32_t event_id)
{
    panic_if(event_id >= eventSlotIndex,
        "Trying wakeup on an event that is not yet created\n");
    if (ETable[event_id].threadWaiting) {
        panic_if(!ETable[event_id].tc,
                 "No thread context to wake up\n");
        ThreadContext *tc = ETable[event_id].tc;
        DPRINTF(HSADriver,
                "Signal event: Waking up CPU %d\n", tc->cpuId());
        // Wake up this thread
        tc->activate();
        // Remove events that can wake up this thread
        TCEvents[tc].clearEvents();
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
HSADriver::DriverWakeupEvent::process()
{
    DPRINTF(HSADriver,
            "Timer event: Waking up CPU %d\n", tc->cpuId());
    // Wake up this thread
    tc->activate();
    // Remove events that can wake up this thread
    driver->TCEvents[tc].clearEvents();
}
