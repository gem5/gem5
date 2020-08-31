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
 *
 * Authors: Anthony Gutierrez
 *          Eric van Tassell
 */

#include "dev/hsa/hsa_driver.hh"

#include "cpu/thread_context.hh"
#include "debug/HSADriver.hh"
#include "dev/hsa/hsa_device.hh"
#include "dev/hsa/kfd_ioctl.h"
#include "params/HSADriver.hh"
#include "sim/process.hh"
#include "sim/proxy_ptr.hh"
#include "sim/syscall_emul_buf.hh"

HSADriver::HSADriver(HSADriverParams *p)
    : EmulatedDriver(p), device(p->device), queueId(0)
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
 * device's packet processor's doorbells.
 */
Addr
HSADriver::mmap(ThreadContext *tc, Addr start, uint64_t length, int prot,
                int tgt_flags, int tgt_fd, int offset)
{
    DPRINTF(HSADriver, "amdkfd doorbell mmap (start: %p, length: 0x%x,"
            "offset: 0x%x)\n", start, length, offset);

    auto process = tc->getProcessPtr();
    auto mem_state = process->memState;

    // Extend global mmap region if necessary.
    if (start == 0) {
        // Assume mmap grows down, as in x86 Linux.
        start = mem_state->getMmapEnd() - length;
        mem_state->setMmapEnd(start);
    }

    /**
     * Now map this virtual address to our PIO doorbell interface
     * in the page tables (non-cacheable).
     */
    process->pTable->map(start, device->hsaPacketProc().pioAddr,
                         length, false);
    DPRINTF(HSADriver, "amdkfd doorbell mapped to %xp\n", start);
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

    args->queue_id = queueId++;
    auto &hsa_pp = device->hsaPacketProc();
    hsa_pp.setDeviceQueueDesc(args->read_pointer_address,
                              args->ring_base_address, args->queue_id,
                              args->ring_size);
}
