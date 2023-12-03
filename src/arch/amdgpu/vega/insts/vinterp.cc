/*
 * Copyright (c) 2024 Advanced Micro Devices, Inc.
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

#include "arch/amdgpu/vega/insts/instructions.hh"

namespace gem5
{

namespace VegaISA
{
// --- Inst_VINTRP__V_INTERP_P1_F32 class methods ---

Inst_VINTRP__V_INTERP_P1_F32::Inst_VINTRP__V_INTERP_P1_F32(InFmt_VINTRP *iFmt)
    : Inst_VINTRP(iFmt, "v_interp_p1_f32")
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VINTRP__V_INTERP_P1_F32

Inst_VINTRP__V_INTERP_P1_F32::~Inst_VINTRP__V_INTERP_P1_F32() {
} // ~Inst_VINTRP__V_INTERP_P1_F32

// --- description from .arch file ---
// D.f = P10 * S.f + P0; parameter interpolation (SQ translates to
// V_MAD_F32 for SP).
// CAUTION: when in HALF_LDS mode, D must not be the same GPR as S;
// if D == S then data corruption will occur.
// NOTE: In textual representations the I/J VGPR is the first source and
// the attribute is the second source; however in the VOP3 encoding the
// attribute is stored in the src0 field and the VGPR is stored in the
// src1 field.
void
Inst_VINTRP__V_INTERP_P1_F32::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VINTRP__V_INTERP_P2_F32 class methods ---

Inst_VINTRP__V_INTERP_P2_F32::Inst_VINTRP__V_INTERP_P2_F32(InFmt_VINTRP *iFmt)
    : Inst_VINTRP(iFmt, "v_interp_p2_f32")
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VINTRP__V_INTERP_P2_F32

Inst_VINTRP__V_INTERP_P2_F32::~Inst_VINTRP__V_INTERP_P2_F32() {
} // ~Inst_VINTRP__V_INTERP_P2_F32

// --- description from .arch file ---
// D.f = P20 * S.f + D.f; parameter interpolation (SQ translates to
// V_MAD_F32 for SP).
// NOTE: In textual representations the I/J VGPR is the first source and
// the attribute is the second source; however in the VOP3 encoding the
// attribute is stored in the src0 field and the VGPR is stored in the
// src1 field.
void
Inst_VINTRP__V_INTERP_P2_F32::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VINTRP__V_INTERP_MOV_F32 class methods ---

Inst_VINTRP__V_INTERP_MOV_F32::Inst_VINTRP__V_INTERP_MOV_F32(
    InFmt_VINTRP *iFmt)
    : Inst_VINTRP(iFmt, "v_interp_mov_f32")
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VINTRP__V_INTERP_MOV_F32

Inst_VINTRP__V_INTERP_MOV_F32::~Inst_VINTRP__V_INTERP_MOV_F32() {
} // ~Inst_VINTRP__V_INTERP_MOV_F32

// --- description from .arch file ---
// D.f = {P10,P20,P0}[S.u]; parameter load.
void
Inst_VINTRP__V_INTERP_MOV_F32::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute
} // namespace VegaISA
} // namespace gem5
