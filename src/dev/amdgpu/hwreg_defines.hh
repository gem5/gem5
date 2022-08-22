/*
 * Copyright (c) 2022 Advanced Micro Devices, Inc.
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
 *
 */

#ifndef __DEV_GPU_HWREG_DEFINES_H__
#define __DEV_GPU_HWREG_DEFINES_H__
/*
 * This enum is adapted from the offsets seen by LLVM:
 *
 * https://github.com/llvm/llvm-project/blob/release/14.x/llvm/lib/
 *         Target/AMDGPU/Utils/AMDGPUAsmUtils.cpp#L58
 */

namespace gem5
{

/*
 * Further descriptions can be found in the "Hardware Register Values" table
 * in any of the GCN3, Vega, CDNA1, CNDA2, or RDNA ISA manuals.
 */
enum amdgpu_hwreg
{
    HW_REG_MODE = 0x1,
    HW_REG_STATUS = 0x2,
    HW_REG_TRAPSTS = 0x3,
    HW_REG_HW_ID = 0x4,
    HW_REG_GPR_ALLOC = 0x5,
    HW_REG_LDS_ALLOC = 0x6,
    HW_REG_IB_STS = 0x7,
    HW_REG_SH_MEM_BASES = 0xf,
    HW_REG_TBA_LO = 0x10,
    HW_REG_TBA_HI = 0x11,
    HW_REG_TMA_LO = 0x12,
    HW_REG_TMA_HI = 0x13,
    HW_REG_FLAT_SCR_LO = 0x14,
    HW_REG_FLAT_SCR_HI = 0x15,
    HW_REG_XNACK_MASK = 0x16,
    HW_REG_HW_ID1 = 0x17,
    HW_REG_HW_ID2 = 0x18,
    HW_REG_POPS_PACKER = 0x19,
    HW_REG_SHADER_CYCLES = 0x1d,
};

} // namespace gem5

#endif // __DEV_GPU_HWREG_DEFINES_H__
