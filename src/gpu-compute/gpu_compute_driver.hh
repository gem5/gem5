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

/**
 * @file
 * The GPUComputeDriver implements an HSADriver for an HSA AMD GPU
 * agent. Other GPU devices, or other HSA agents, should not derive
 * from this class. Instead device-specific implementations of an
 * HSADriver should be provided for each unique device.
 */

#ifndef __GPU_COMPUTE_GPU_COMPUTE_DRIVER_HH__
#define __GPU_COMPUTE_GPU_COMPUTE_DRIVER_HH__

#include "base/addr_range_map.hh"
#include "dev/hsa/hsa_driver.hh"
#include "mem/request.hh"

struct GPUComputeDriverParams;

class GPUComputeDriver final : public HSADriver
{
  public:
    typedef GPUComputeDriverParams Params;
    GPUComputeDriver(const Params &p);
    int ioctl(ThreadContext *tc, unsigned req, Addr ioc_buf) override;
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

  private:
    bool isdGPU;
    int dGPUPoolID;

    /**
     * VMA structures for GPUVM memory.
     */
    AddrRangeMap<Request::CacheCoherenceFlags, 1> gpuVmas;

    /**
     * Mtype bits {Cached, Read Write, Shared} for caches
     */
    enum MtypeFlags
    {
        SHARED                  = 0,
        READ_WRITE              = 1,
        CACHED                  = 2
    };

    Request::CacheCoherenceFlags defaultMtype;

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
    Addr scratchApeLimit(Addr apeBase) const;
    Addr ldsApeBase(int gpuNum) const;
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
};

#endif // __GPU_COMPUTE_GPU_COMPUTE_DRIVER_HH__
