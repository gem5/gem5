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

#include "dev/amdgpu/amdgpu_gfx.hh"

#include "mem/packet_access.hh"
#include "sim/core.hh"

namespace gem5
{

AMDGPUGfx::AMDGPUGfx()
{
    for (int i = 0; i < SCRATCH_REGS; ++i) {
        scratchRegs[i] = 0;
    }
}

void
AMDGPUGfx::readMMIO(PacketPtr pkt, Addr offset)
{
    switch (offset) {
    case AMDGPU_MM_RLC_GPU_CLOCK_COUNT_LSB:
        pkt->setLE<uint32_t>(captured_clock_count);
        break;
    case AMDGPU_MM_RLC_GPU_CLOCK_COUNT_MSB:
        pkt->setLE<uint32_t>(captured_clock_count >> 32);
        break;
    case AMDGPU_MM_SCRATCH_REG0:
        pkt->setLE<uint32_t>(scratchRegs[0]);
        break;
    default:
        break;
    }
}

void
AMDGPUGfx::writeMMIO(PacketPtr pkt, Addr offset)
{
    switch (offset) {
    case AMDGPU_MM_RLC_CAPTURE_GPU_CLOCK_COUNT:
        // Use gem5 Ticks in nanoseconds are the counter. The first capture
        // is expected to return zero.
        if (captured_clock_count == 1) {
            captured_clock_count = 0;
        } else {
            captured_clock_count = curTick() / sim_clock::as_int::ns;
        }
        break;
    case AMDGPU_MM_SCRATCH_REG0:
        scratchRegs[0] = pkt->getLE<uint32_t>();
        break;
    default:
        break;
    }
}

} // namespace gem5
