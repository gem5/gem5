/*
 * Copyright (c) 2016-2021 Advanced Micro Devices, Inc.
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

#include "arch/amdgpu/vega/gpu_isa.hh"

#include <numeric>

#include "gpu-compute/gpu_static_inst.hh"
#include "gpu-compute/wavefront.hh"

namespace gem5
{

namespace VegaISA
{
GPUISA::GPUISA(Wavefront &wf) : wavefront(wf), m0(0) {}

ScalarRegU32
GPUISA::readMiscReg(int opIdx) const
{
    switch (opIdx) {
    case REG_M0:
        return m0;
    case REG_ZERO:
        return 0;
    case REG_SCC:
        return statusReg.SCC;
    default:
        fatal("attempting to read from unsupported or non-readable "
              "register. selector val: %i\n",
              opIdx);
        return 0;
    }
}

void
GPUISA::writeMiscReg(int opIdx, ScalarRegU32 operandVal)
{
    switch (opIdx) {
    case REG_M0:
        m0 = operandVal;
        break;
    case REG_SCC:
        statusReg.SCC = operandVal ? 1 : 0;
        break;
    default:
        fatal("attempting to write to an unsupported or non-writable "
              "register. selector val: %i\n",
              opIdx);
        break;
    }
}

void
GPUISA::advancePC(GPUDynInstPtr gpuDynInst)
{
    wavefront.pc(wavefront.pc() + gpuDynInst->staticInstruction()->instSize());
}

const std::array<const ScalarRegU32, NumPosConstRegs> GPUISA::posConstRegs = {
    { 1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16,
      17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32,
      33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48,
      49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64 }
};

const std::array<const ScalarRegI32, NumNegConstRegs> GPUISA::negConstRegs = {
    { -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -12, -13, -14, -15, -16 }
};
} // namespace VegaISA
} // namespace gem5
