/*
 * Copyright (c) 2021 Kyle Roarty
 * All Rights Reserved.
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

#include "gpu-compute/gpu_render_driver.hh"

#include "params/GPURenderDriver.hh"
#include "sim/fd_entry.hh"

namespace gem5
{

GPURenderDriver::GPURenderDriver(const GPURenderDriverParams &p)
    : EmulatedDriver(p)
{
}

/* ROCm 4 utilizes the render driver located at /dev/dri/renderDXXX. This
 * patch implements a very simple driver that just returns a file
 * descriptor when opened.
 */
int
GPURenderDriver::open(ThreadContext *tc, int mode, int flags)
{
    auto process = tc->getProcessPtr();
    auto device_fd_entry = std::make_shared<DeviceFDEntry>(this, filename);
    int tgt_fd = process->fds->allocFD(device_fd_entry);
    return tgt_fd;
}

/* DGPUs try to mmap the driver file. It doesn't appear they do anything
 * with it, so we just return the address that's provided
 */
Addr GPURenderDriver::mmap(ThreadContext *tc, Addr start, uint64_t length,
                           int prot, int tgt_flags, int tgt_fd, off_t offset)
{
    warn_once("GPURenderDriver::mmap returning start address %#x", start);
    return start;
}

} // namespace gem5
