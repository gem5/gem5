/*
 * Copyright (c) 2021 Advanced Micro Devices, Inc.
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

#ifndef __DEV_AMDGPU_AMDGPU_DEFINES_HH__
#define __DEV_AMDGPU_AMDGPU_DEFINES_HH__

#include "base/types.hh"

namespace gem5
{

/* Types of queues supported by device */
enum QueueType
{
    Compute,
    Gfx,
    SDMAGfx,
    SDMAPage,
    ComputeAQL,
    InterruptHandler,
    RLC
};

// AMD GPUs support 16 different virtual address spaces
static constexpr int AMDGPU_VM_COUNT = 16;

/* Names of BARs used by the device. */
constexpr int FRAMEBUFFER_BAR = 0;
constexpr int DOORBELL_BAR = 2;
constexpr int MMIO_BAR = 5;

/* By default the X86 kernel expects the vga ROM at 0xc0000. */
constexpr uint32_t VGA_ROM_DEFAULT = 0xc0000;
constexpr uint32_t ROM_SIZE = 0x20000;        // 128kB

/* SDMA base, size, mmio offset shift. */
static constexpr uint32_t SDMA0_BASE  = 0x4980;
static constexpr uint32_t SDMA1_BASE  = 0x5180;
static constexpr uint32_t SDMA_SIZE  = 0x800;
static constexpr uint32_t SDMA_OFFSET_SHIFT  = 2;

/* Interrupt handler base, size, mmio offset shift. */
static constexpr uint32_t IH_BASE = 0x4280;
static constexpr uint32_t IH_SIZE = 0x700;
static constexpr uint32_t IH_OFFSET_SHIFT = 2;

/* Graphics register bus manager base, size, mmio offset shift. */
static constexpr uint32_t GRBM_BASE  = 0x8000;
static constexpr uint32_t GRBM_SIZE  = 0x5000;
static constexpr uint32_t GRBM_OFFSET_SHIFT  = 2;

/* GFX base, size, mmio offset shift. */
static constexpr uint32_t GFX_BASE  = 0x28000;
static constexpr uint32_t GFX_SIZE  = 0x17000;
static constexpr uint32_t GFX_OFFSET_SHIFT  = 2;

/* MMHUB base, size, mmio offset shift. */
static constexpr uint32_t MMHUB_BASE = 0x68000;
static constexpr uint32_t MMHUB_SIZE = 0x2120;
static constexpr uint32_t MMHUB_OFFSET_SHIFT = 2;

/* NBIO base and size. */
static constexpr uint32_t NBIO_BASE = 0x0;
static constexpr uint32_t NBIO_SIZE = 0x4280;

} // namespace gem5

#endif // __DEV_AMDGPU_AMDGPU_DEFINES_HH__
