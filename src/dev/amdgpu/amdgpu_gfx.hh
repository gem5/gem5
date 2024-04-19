/*
 * Copyright (c) 2023 Advanced Micro Devices, Inc.
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

#ifndef __DEV_AMDGPU_AMDGPU_GFX_HH__
#define __DEV_AMDGPU_AMDGPU_GFX_HH__

#include "base/types.hh"
#include "mem/packet.hh"

/**
 * MMIO offsets for GFX. This class handles MMIO reads/writes to the GFX_BASE
 * aperture which are generally read/written by the gfx driver source here:
 *
 *      drivers/gpu/drm/amd/amdgpu/amdgpu_discovery.c
 * https://github.com/RadeonOpenCompute/ROCK-Kernel-Driver/blob/master/
 *      drivers/gpu/drm/amd/amdgpu/gfx_v9_0.c
 *
 * The MMIO addresses in the file are dword addresses. Here they are converted
 * to byte addresses so gem5 does not need to shift the values.
 */

// Registers used to read GPU clock count used in profiling
#define AMDGPU_MM_RLC_GPU_CLOCK_COUNT_LSB                 0x13090
#define AMDGPU_MM_RLC_GPU_CLOCK_COUNT_MSB                 0x13094
#define AMDGPU_MM_RLC_CAPTURE_GPU_CLOCK_COUNT             0x13098

// Scratch registers used for GPU post
#define AMDGPU_MM_SCRATCH_REG0                            0x08100

namespace gem5
{

class AMDGPUGfx
{
  public:
    AMDGPUGfx();

    void readMMIO(PacketPtr pkt, Addr offset);
    void writeMMIO(PacketPtr pkt, Addr offset);

  private:
    /*
     * GPU clock count at the time capture MMIO is received.
     */
    uint64_t captured_clock_count = 1;

    /*
     * Scratch registers.
     */
    static constexpr int SCRATCH_REGS = 8;
    std::array<uint32_t, SCRATCH_REGS> scratchRegs;
};

} // namespace gem5

#endif // __DEV_AMDGPU_AMDGPU_GFX_HH__
