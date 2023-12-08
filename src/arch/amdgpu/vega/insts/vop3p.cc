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

#include "arch/amdgpu/vega/insts/vop3p.hh"

#include "arch/arm/insts/fplib.hh"

namespace gem5
{

namespace VegaISA
{

using half = uint16_t;

// Helper functions
int16_t
clampI16(int32_t value, bool clamp)
{
    if (!clamp) {
        return static_cast<int16_t>(value);
    }

    return std::clamp(value,
            static_cast<int32_t>(std::numeric_limits<int16_t>::min()),
            static_cast<int32_t>(std::numeric_limits<int16_t>::max()));
}

uint16_t
clampU16(uint32_t value, bool clamp)
{
    if (!clamp) {
        return static_cast<uint16_t>(value);
    }

    return std::clamp(value,
            static_cast<uint32_t>(std::numeric_limits<uint16_t>::min()),
            static_cast<uint32_t>(std::numeric_limits<uint16_t>::max()));
}

uint16_t
clampF16(uint16_t value, bool clamp)
{
    if (!clamp) {
        return value;
    }

    // Values of one and zero in fp16.
    constexpr uint16_t one = 0x3c00;
    constexpr uint16_t zero = 0x0;
    ArmISA::FPSCR fpscr1, fpscr2;

    // If value > one, set to one, then if value < zero set to zero.
    uint16_t imm = fplibMin(value, one, fpscr1);
    return fplibMax(imm, zero, fpscr2);
}




// Begin instruction execute definitions
void Inst_VOP3P__V_PK_MAD_I16::execute(GPUDynInstPtr gpuDynInst)
{
    auto opImpl =
        [](int16_t S0, int16_t S1, int16_t S2, bool clamp) -> int16_t
    {
        return clampI16(S0 * S1 + S2, clamp);
    };

    vop3pHelper<int16_t>(gpuDynInst, opImpl);
}

void
Inst_VOP3P__V_PK_MUL_LO_U16::execute(GPUDynInstPtr gpuDynInst)
{
    auto opImpl = [](uint16_t S0, uint16_t S1, bool) -> uint16_t
    {
        // Only return lower 16 bits of result - This operation cannot clamp.
        uint32_t D = S0 * S1;
        uint16_t Dh = D & 0xFFFF;
        return Dh;
    };

    vop3pHelper<uint16_t>(gpuDynInst, opImpl);
}

void Inst_VOP3P__V_PK_ADD_I16::execute(GPUDynInstPtr gpuDynInst)
{
    auto opImpl = [](int16_t S0, int16_t S1, bool clamp) -> int16_t
    {
        return clampI16(S0 + S1, clamp);
    };

    vop3pHelper<int16_t>(gpuDynInst, opImpl);
}

void Inst_VOP3P__V_PK_SUB_I16::execute(GPUDynInstPtr gpuDynInst)
{
    auto opImpl = [](int16_t S0, int16_t S1, bool clamp) -> int16_t
    {
        return clampI16(S0 - S1, clamp);
    };

    vop3pHelper<int16_t>(gpuDynInst, opImpl);
}

void Inst_VOP3P__V_PK_LSHLREV_B16::execute(GPUDynInstPtr gpuDynInst)
{
    auto opImpl = [](uint16_t S0, uint16_t S1, bool) -> uint16_t
    {
        unsigned shift_val = bits(S0, 3, 0);

        // Shift does not clamp
        return S1 << shift_val;
    };

    vop3pHelper<uint16_t>(gpuDynInst, opImpl);
}

void Inst_VOP3P__V_PK_LSHRREV_B16::execute(GPUDynInstPtr gpuDynInst)
{
    auto opImpl = [](uint16_t S0, uint16_t S1, bool) -> uint16_t
    {
        unsigned shift_val = bits(S0, 3, 0);

        return S1 >> shift_val;
    };

    vop3pHelper<uint16_t>(gpuDynInst, opImpl);
}

void Inst_VOP3P__V_PK_ASHRREV_B16::execute(GPUDynInstPtr gpuDynInst)
{
    auto opImpl = [](int16_t S0, int16_t S1, bool clamp) -> int16_t
    {
        // Sign extend to larger type to ensure we don't lose sign bits when
        // shifting.
        int32_t S1e = S1;
        unsigned shift_val = bits(S0, 3, 0);

        return S1e >> shift_val;
    };

    vop3pHelper<int16_t>(gpuDynInst, opImpl);
}

void Inst_VOP3P__V_PK_MAX_I16::execute(GPUDynInstPtr gpuDynInst)
{
    auto opImpl = [](int16_t S0, int16_t S1, bool clamp) -> int16_t
    {
        return clampI16((S0 >= S1) ? S0 : S1, clamp);
    };

    vop3pHelper<int16_t>(gpuDynInst, opImpl);
}

void Inst_VOP3P__V_PK_MIN_I16::execute(GPUDynInstPtr gpuDynInst)
{
    auto opImpl = [](int16_t S0, int16_t S1, bool clamp) -> int16_t
    {
        return clampI16((S0 < S1) ? S0 : S1, clamp);
    };

    vop3pHelper<int16_t>(gpuDynInst, opImpl);
}

void Inst_VOP3P__V_PK_MAD_U16::execute(GPUDynInstPtr gpuDynInst)
{
    auto opImpl =
        [](uint16_t S0, uint16_t S1, uint16_t S2, bool clamp) -> uint16_t
    {
        return clampU16(S0 * S1 + S2, clamp);
    };

    vop3pHelper<uint16_t>(gpuDynInst, opImpl);
}

void Inst_VOP3P__V_PK_ADD_U16::execute(GPUDynInstPtr gpuDynInst)
{
    auto opImpl = [](uint16_t S0, uint16_t S1, bool clamp) -> uint16_t
    {
        return clampU16(S0 + S1, clamp);
    };

    vop3pHelper<uint16_t>(gpuDynInst, opImpl);
}

void Inst_VOP3P__V_PK_SUB_U16::execute(GPUDynInstPtr gpuDynInst)
{
    auto opImpl = [](uint16_t S0, uint16_t S1, bool clamp) -> uint16_t
    {
        return clampU16(S0 - S1, clamp);
    };

    vop3pHelper<uint16_t>(gpuDynInst, opImpl);
}

void Inst_VOP3P__V_PK_MAX_U16::execute(GPUDynInstPtr gpuDynInst)
{
    auto opImpl = [](uint16_t S0, uint16_t S1, bool clamp) -> uint16_t
    {
        return clampU16((S0 >= S1) ? S0 : S1, clamp);
    };

    vop3pHelper<uint16_t>(gpuDynInst, opImpl);
}

void Inst_VOP3P__V_PK_MIN_U16::execute(GPUDynInstPtr gpuDynInst)
{
    auto opImpl = [](uint16_t S0, uint16_t S1, bool clamp) -> uint16_t
    {
        return clampU16((S0 < S1) ? S0 : S1, clamp);
    };

    vop3pHelper<uint16_t>(gpuDynInst, opImpl);
}

void Inst_VOP3P__V_PK_FMA_F16::execute(GPUDynInstPtr gpuDynInst)
{
    auto opImpl = [](half S0, half S1, half S2, bool clamp) -> half
    {
        ArmISA::FPSCR fpscr;
        return clampF16(fplibMulAdd(S2, S0, S1, fpscr), clamp);
    };

    vop3pHelper<half>(gpuDynInst, opImpl);
}

void Inst_VOP3P__V_PK_ADD_F16::execute(GPUDynInstPtr gpuDynInst)
{
    auto opImpl = [](half S0, half S1, bool clamp) -> half
    {
        ArmISA::FPSCR fpscr;
        return clampF16(fplibAdd(S0, S1, fpscr), clamp);
    };

    vop3pHelper<half>(gpuDynInst, opImpl);
}

void Inst_VOP3P__V_PK_MUL_F16::execute(GPUDynInstPtr gpuDynInst)
{
    auto opImpl = [](half S0, half S1, bool clamp) -> half
    {
        ArmISA::FPSCR fpscr;
        return clampF16(fplibMul(S0, S1, fpscr), clamp);
    };

    vop3pHelper<half>(gpuDynInst, opImpl);
}

void Inst_VOP3P__V_PK_MIN_F16::execute(GPUDynInstPtr gpuDynInst)
{
    auto opImpl = [](half S0, half S1, bool clamp) -> half
    {
        ArmISA::FPSCR fpscr;
        return clampF16(fplibMin(S0, S1, fpscr), clamp);
    };

    vop3pHelper<half>(gpuDynInst, opImpl);
}

void Inst_VOP3P__V_PK_MAX_F16::execute(GPUDynInstPtr gpuDynInst)
{
    auto opImpl = [](half S0, half S1, bool clamp) -> half
    {
        ArmISA::FPSCR fpscr;
        return clampF16(fplibMax(S0, S1, fpscr), clamp);
    };

    vop3pHelper<half>(gpuDynInst, opImpl);
}

} // namespace VegaISA
} // namespace gem5
