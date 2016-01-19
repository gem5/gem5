/*
 * Copyright (c) 2012-2015 Advanced Micro Devices, Inc.
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
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
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
 * Author: Anthony Gutierrez
 */

#ifndef __CL_DRIVER_HH__
#define __CL_DRIVER_HH__

#include <vector>

#include "gpu-compute/hsa_kernel_info.hh"
#include "sim/emul_driver.hh"

class GpuDispatcher;
class HsaCode;
class LiveProcess;
class ThreadContext;

struct ClDriverParams;

class ClDriver final : public EmulatedDriver
{
  public:
    ClDriver(ClDriverParams *p);
    void handshake(GpuDispatcher *_dispatcher);
    int open(LiveProcess *p, ThreadContext *tc, int mode, int flags);
    int ioctl(LiveProcess *p, ThreadContext *tc, unsigned req);
    const char* codeOffToKernelName(uint64_t code_ptr);

  private:
    GpuDispatcher *dispatcher;

    std::vector<const std::string*> codeFiles;

    // All the kernels we know about
    std::vector<HsaCode*> kernels;
    std::vector<HsaCode*> functions;

    std::vector<HsaKernelInfo> kernelInfo;

    // maximum size necessary for function arguments
    int maxFuncArgsSize;
    // The host virtual address for the kernel code
    uint64_t hsaCode;
};

#endif // __CL_DRIVER_HH__
