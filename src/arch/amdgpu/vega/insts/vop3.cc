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

#include "arch/amdgpu/vega/insts/inst_util.hh"
#include "arch/amdgpu/vega/insts/instructions.hh"

namespace gem5
{

namespace VegaISA
{
// --- Inst_VOP3__V_CNDMASK_B32 class methods ---

Inst_VOP3__V_CNDMASK_B32::Inst_VOP3__V_CNDMASK_B32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cndmask_b32", false)
{
    setFlag(ALU);
    setFlag(ReadsVCC);
} // Inst_VOP3__V_CNDMASK_B32

Inst_VOP3__V_CNDMASK_B32::~Inst_VOP3__V_CNDMASK_B32() {
} // ~Inst_VOP3__V_CNDMASK_B32

// --- description from .arch file ---
// D.u = (VCC[i] ? S1.u : S0.u) (i = threadID in wave); VOP3: specify VCC
// as a scalar GPR in S2.
void
Inst_VOP3__V_CNDMASK_B32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ConstScalarOperandU64 vcc(gpuDynInst, extData.SRC2);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    vcc.read();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = bits(vcc.rawData(), lane) ? src1[lane] : src0[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_ADD_F32 class methods ---

Inst_VOP3__V_ADD_F32::Inst_VOP3__V_ADD_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_add_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_ADD_F32

Inst_VOP3__V_ADD_F32::~Inst_VOP3__V_ADD_F32() {} // ~Inst_VOP3__V_ADD_F32

// --- description from .arch file ---
// D.f = S0.f + S1.f.
void
Inst_VOP3__V_ADD_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (instData.ABS & 0x2) {
        src1.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src0[lane] + src1[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_SUB_F32 class methods ---

Inst_VOP3__V_SUB_F32::Inst_VOP3__V_SUB_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_sub_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_SUB_F32

Inst_VOP3__V_SUB_F32::~Inst_VOP3__V_SUB_F32() {} // ~Inst_VOP3__V_SUB_F32

// --- description from .arch file ---
// D.f = S0.f - S1.f.
// SQ translates to V_ADD_F32.
void
Inst_VOP3__V_SUB_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (instData.ABS & 0x2) {
        src1.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src0[lane] - src1[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_SUBREV_F32 class methods ---

Inst_VOP3__V_SUBREV_F32::Inst_VOP3__V_SUBREV_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_subrev_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_SUBREV_F32

Inst_VOP3__V_SUBREV_F32::~Inst_VOP3__V_SUBREV_F32() {
} // ~Inst_VOP3__V_SUBREV_F32

// --- description from .arch file ---
// D.f = S1.f - S0.f.
// SQ translates to V_ADD_F32.
void
Inst_VOP3__V_SUBREV_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (instData.ABS & 0x2) {
        src1.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src1[lane] - src0[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MUL_LEGACY_F32 class methods ---

Inst_VOP3__V_MUL_LEGACY_F32::Inst_VOP3__V_MUL_LEGACY_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_mul_legacy_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_MUL_LEGACY_F32

Inst_VOP3__V_MUL_LEGACY_F32::~Inst_VOP3__V_MUL_LEGACY_F32() {
} // ~Inst_VOP3__V_MUL_LEGACY_F32

// --- description from .arch file ---
// D.f = S0.f * S1.f (DX9 rules, 0.0*x = 0.0).
void
Inst_VOP3__V_MUL_LEGACY_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (instData.ABS & 0x2) {
        src1.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            if (std::isnan(src0[lane]) || std::isnan(src1[lane])) {
                vdst[lane] = NAN;
            } else if ((std::fpclassify(src0[lane]) == FP_SUBNORMAL ||
                        std::fpclassify(src0[lane]) == FP_ZERO) &&
                       !std::signbit(src0[lane])) {
                if (std::isinf(src1[lane])) {
                    vdst[lane] = NAN;
                } else if (!std::signbit(src1[lane])) {
                    vdst[lane] = +0.0;
                } else {
                    vdst[lane] = -0.0;
                }
            } else if ((std::fpclassify(src0[lane]) == FP_SUBNORMAL ||
                        std::fpclassify(src0[lane]) == FP_ZERO) &&
                       std::signbit(src0[lane])) {
                if (std::isinf(src1[lane])) {
                    vdst[lane] = NAN;
                } else if (std::signbit(src1[lane])) {
                    vdst[lane] = +0.0;
                } else {
                    vdst[lane] = -0.0;
                }
            } else if (std::isinf(src0[lane]) && !std::signbit(src0[lane])) {
                if (std::fpclassify(src1[lane]) == FP_SUBNORMAL ||
                    std::fpclassify(src1[lane]) == FP_ZERO) {
                    vdst[lane] = NAN;
                } else if (!std::signbit(src1[lane])) {
                    vdst[lane] = +INFINITY;
                } else {
                    vdst[lane] = -INFINITY;
                }
            } else if (std::isinf(src0[lane]) && std::signbit(src0[lane])) {
                if (std::fpclassify(src1[lane]) == FP_SUBNORMAL ||
                    std::fpclassify(src1[lane]) == FP_ZERO) {
                    vdst[lane] = NAN;
                } else if (std::signbit(src1[lane])) {
                    vdst[lane] = +INFINITY;
                } else {
                    vdst[lane] = -INFINITY;
                }
            } else {
                vdst[lane] = src0[lane] * src1[lane];
            }
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MUL_F32 class methods ---

Inst_VOP3__V_MUL_F32::Inst_VOP3__V_MUL_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_mul_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_MUL_F32

Inst_VOP3__V_MUL_F32::~Inst_VOP3__V_MUL_F32() {} // ~Inst_VOP3__V_MUL_F32

// --- description from .arch file ---
// D.f = S0.f * S1.f.
void
Inst_VOP3__V_MUL_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (instData.ABS & 0x2) {
        src1.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            if (std::isnan(src0[lane]) || std::isnan(src1[lane])) {
                vdst[lane] = NAN;
            } else if ((std::fpclassify(src0[lane]) == FP_SUBNORMAL ||
                        std::fpclassify(src0[lane]) == FP_ZERO) &&
                       !std::signbit(src0[lane])) {
                if (std::isinf(src1[lane])) {
                    vdst[lane] = NAN;
                } else if (!std::signbit(src1[lane])) {
                    vdst[lane] = +0.0;
                } else {
                    vdst[lane] = -0.0;
                }
            } else if ((std::fpclassify(src0[lane]) == FP_SUBNORMAL ||
                        std::fpclassify(src0[lane]) == FP_ZERO) &&
                       std::signbit(src0[lane])) {
                if (std::isinf(src1[lane])) {
                    vdst[lane] = NAN;
                } else if (std::signbit(src1[lane])) {
                    vdst[lane] = +0.0;
                } else {
                    vdst[lane] = -0.0;
                }
            } else if (std::isinf(src0[lane]) && !std::signbit(src0[lane])) {
                if (std::fpclassify(src1[lane]) == FP_SUBNORMAL ||
                    std::fpclassify(src1[lane]) == FP_ZERO) {
                    vdst[lane] = NAN;
                } else if (!std::signbit(src1[lane])) {
                    vdst[lane] = +INFINITY;
                } else {
                    vdst[lane] = -INFINITY;
                }
            } else if (std::isinf(src0[lane]) && std::signbit(src0[lane])) {
                if (std::fpclassify(src1[lane]) == FP_SUBNORMAL ||
                    std::fpclassify(src1[lane]) == FP_ZERO) {
                    vdst[lane] = NAN;
                } else if (std::signbit(src1[lane])) {
                    vdst[lane] = +INFINITY;
                } else {
                    vdst[lane] = -INFINITY;
                }
            } else {
                vdst[lane] = src0[lane] * src1[lane];
            }
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MUL_I32_I24 class methods ---

Inst_VOP3__V_MUL_I32_I24::Inst_VOP3__V_MUL_I32_I24(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_mul_i32_i24", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_MUL_I32_I24

Inst_VOP3__V_MUL_I32_I24::~Inst_VOP3__V_MUL_I32_I24() {
} // ~Inst_VOP3__V_MUL_I32_I24

// --- description from .arch file ---
// D.i = S0.i[23:0] * S1.i[23:0].
void
Inst_VOP3__V_MUL_I32_I24::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1);
    VecOperandI32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = sext<24>(bits(src0[lane], 23, 0)) *
                         sext<24>(bits(src1[lane], 23, 0));
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MUL_HI_I32_I24 class methods ---

Inst_VOP3__V_MUL_HI_I32_I24::Inst_VOP3__V_MUL_HI_I32_I24(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_mul_hi_i32_i24", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_MUL_HI_I32_I24

Inst_VOP3__V_MUL_HI_I32_I24::~Inst_VOP3__V_MUL_HI_I32_I24() {
} // ~Inst_VOP3__V_MUL_HI_I32_I24

// --- description from .arch file ---
// D.i = (S0.i[23:0] * S1.i[23:0])>>32.
void
Inst_VOP3__V_MUL_HI_I32_I24::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1);
    VecOperandI32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            VecElemI64 tmp_src0 =
                (VecElemI64)sext<24>(bits(src0[lane], 23, 0));
            VecElemI64 tmp_src1 =
                (VecElemI64)sext<24>(bits(src1[lane], 23, 0));

            vdst[lane] = (VecElemI32)((tmp_src0 * tmp_src1) >> 32);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MUL_U32_U24 class methods ---

Inst_VOP3__V_MUL_U32_U24::Inst_VOP3__V_MUL_U32_U24(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_mul_u32_u24", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_MUL_U32_U24

Inst_VOP3__V_MUL_U32_U24::~Inst_VOP3__V_MUL_U32_U24() {
} // ~Inst_VOP3__V_MUL_U32_U24

// --- description from .arch file ---
// D.u = S0.u[23:0] * S1.u[23:0].
void
Inst_VOP3__V_MUL_U32_U24::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = bits(src0[lane], 23, 0) * bits(src1[lane], 23, 0);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MUL_HI_U32_U24 class methods ---

Inst_VOP3__V_MUL_HI_U32_U24::Inst_VOP3__V_MUL_HI_U32_U24(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_mul_hi_u32_u24", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_MUL_HI_U32_U24

Inst_VOP3__V_MUL_HI_U32_U24::~Inst_VOP3__V_MUL_HI_U32_U24() {
} // ~Inst_VOP3__V_MUL_HI_U32_U24

// --- description from .arch file ---
// D.i = (S0.u[23:0] * S1.u[23:0])>>32.
void
Inst_VOP3__V_MUL_HI_U32_U24::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            VecElemU64 tmp_src0 = (VecElemU64)bits(src0[lane], 23, 0);
            VecElemU64 tmp_src1 = (VecElemU64)bits(src1[lane], 23, 0);
            vdst[lane] = (VecElemU32)((tmp_src0 * tmp_src1) >> 32);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MIN_F32 class methods ---

Inst_VOP3__V_MIN_F32::Inst_VOP3__V_MIN_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_min_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_MIN_F32

Inst_VOP3__V_MIN_F32::~Inst_VOP3__V_MIN_F32() {} // ~Inst_VOP3__V_MIN_F32

// --- description from .arch file ---
// D.f = (S0.f < S1.f ? S0.f : S1.f).
void
Inst_VOP3__V_MIN_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (instData.ABS & 0x2) {
        src1.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::fmin(src0[lane], src1[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MAX_F32 class methods ---

Inst_VOP3__V_MAX_F32::Inst_VOP3__V_MAX_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_max_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_MAX_F32

Inst_VOP3__V_MAX_F32::~Inst_VOP3__V_MAX_F32() {} // ~Inst_VOP3__V_MAX_F32

// --- description from .arch file ---
// D.f = (S0.f >= S1.f ? S0.f : S1.f).
void
Inst_VOP3__V_MAX_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (instData.ABS & 0x2) {
        src1.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::fmax(src0[lane], src1[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MIN_I32 class methods ---

Inst_VOP3__V_MIN_I32::Inst_VOP3__V_MIN_I32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_min_i32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_MIN_I32

Inst_VOP3__V_MIN_I32::~Inst_VOP3__V_MIN_I32() {} // ~Inst_VOP3__V_MIN_I32

// --- description from .arch file ---
// D.i = min(S0.i, S1.i).
void
Inst_VOP3__V_MIN_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1);
    VecOperandI32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::min(src0[lane], src1[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MAX_I32 class methods ---

Inst_VOP3__V_MAX_I32::Inst_VOP3__V_MAX_I32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_max_i32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_MAX_I32

Inst_VOP3__V_MAX_I32::~Inst_VOP3__V_MAX_I32() {} // ~Inst_VOP3__V_MAX_I32

// --- description from .arch file ---
// D.i = max(S0.i, S1.i).
void
Inst_VOP3__V_MAX_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1);
    VecOperandI32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::max(src0[lane], src1[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MIN_U32 class methods ---

Inst_VOP3__V_MIN_U32::Inst_VOP3__V_MIN_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_min_u32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_MIN_U32

Inst_VOP3__V_MIN_U32::~Inst_VOP3__V_MIN_U32() {} // ~Inst_VOP3__V_MIN_U32

// --- description from .arch file ---
// D.u = min(S0.u, S1.u).
void
Inst_VOP3__V_MIN_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::min(src0[lane], src1[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MAX_U32 class methods ---

Inst_VOP3__V_MAX_U32::Inst_VOP3__V_MAX_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_max_u32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_MAX_U32

Inst_VOP3__V_MAX_U32::~Inst_VOP3__V_MAX_U32() {} // ~Inst_VOP3__V_MAX_U32

// --- description from .arch file ---
// D.u = max(S0.u, S1.u).
void
Inst_VOP3__V_MAX_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::max(src0[lane], src1[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_LSHRREV_B32 class methods ---

Inst_VOP3__V_LSHRREV_B32::Inst_VOP3__V_LSHRREV_B32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_lshrrev_b32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_LSHRREV_B32

Inst_VOP3__V_LSHRREV_B32::~Inst_VOP3__V_LSHRREV_B32() {
} // ~Inst_VOP3__V_LSHRREV_B32

// --- description from .arch file ---
// D.u = S1.u >> S0.u[4:0].
// The vacated bits are set to zero.
// SQ translates this to an internal SP opcode.
void
Inst_VOP3__V_LSHRREV_B32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src1[lane] >> bits(src0[lane], 4, 0);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_ASHRREV_I32 class methods ---

Inst_VOP3__V_ASHRREV_I32::Inst_VOP3__V_ASHRREV_I32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_ashrrev_i32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_ASHRREV_I32

Inst_VOP3__V_ASHRREV_I32::~Inst_VOP3__V_ASHRREV_I32() {
} // ~Inst_VOP3__V_ASHRREV_I32

// --- description from .arch file ---
// D.i = signext(S1.i) >> S0.i[4:0].
// The vacated bits are set to the sign bit of the input value.
// SQ translates this to an internal SP opcode.
void
Inst_VOP3__V_ASHRREV_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1);
    VecOperandI32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src1[lane] >> bits(src0[lane], 4, 0);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_LSHLREV_B32 class methods ---

Inst_VOP3__V_LSHLREV_B32::Inst_VOP3__V_LSHLREV_B32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_lshlrev_b32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_LSHLREV_B32

Inst_VOP3__V_LSHLREV_B32::~Inst_VOP3__V_LSHLREV_B32() {
} // ~Inst_VOP3__V_LSHLREV_B32

// --- description from .arch file ---
// D.u = S1.u << S0.u[4:0].
// SQ translates this to an internal SP opcode.
void
Inst_VOP3__V_LSHLREV_B32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src1[lane] << bits(src0[lane], 4, 0);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_AND_B32 class methods ---

Inst_VOP3__V_AND_B32::Inst_VOP3__V_AND_B32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_and_b32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_AND_B32

Inst_VOP3__V_AND_B32::~Inst_VOP3__V_AND_B32() {} // ~Inst_VOP3__V_AND_B32

// --- description from .arch file ---
// D.u = S0.u & S1.u.
// Input and output modifiers not supported.
void
Inst_VOP3__V_AND_B32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src0[lane] & src1[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_OR_B32 class methods ---

Inst_VOP3__V_OR_B32::Inst_VOP3__V_OR_B32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_or_b32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_OR_B32

Inst_VOP3__V_OR_B32::~Inst_VOP3__V_OR_B32() {} // ~Inst_VOP3__V_OR_B32

// --- description from .arch file ---
// D.u = S0.u | S1.u.
// Input and output modifiers not supported.
void
Inst_VOP3__V_OR_B32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src0[lane] | src1[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_OR3_B32 class methods ---

Inst_VOP3__V_OR3_B32::Inst_VOP3__V_OR3_B32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_or3_b32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_OR3_B32

Inst_VOP3__V_OR3_B32::~Inst_VOP3__V_OR3_B32() {} // ~Inst_VOP3__V_OR3_B32

// --- description from .arch file ---
// D.u = S0.u | S1.u | S2.u.
// Input and output modifiers not supported.
void
Inst_VOP3__V_OR3_B32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU32 src2(gpuDynInst, extData.SRC2);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src0[lane] | src1[lane] | src2[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_XOR_B32 class methods ---

Inst_VOP3__V_XOR_B32::Inst_VOP3__V_XOR_B32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_xor_b32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_XOR_B32

Inst_VOP3__V_XOR_B32::~Inst_VOP3__V_XOR_B32() {} // ~Inst_VOP3__V_XOR_B32

// --- description from .arch file ---
// D.u = S0.u ^ S1.u.
// Input and output modifiers not supported.
void
Inst_VOP3__V_XOR_B32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src0[lane] ^ src1[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MAC_F32 class methods ---

Inst_VOP3__V_MAC_F32::Inst_VOP3__V_MAC_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_mac_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
    setFlag(MAC);
} // Inst_VOP3__V_MAC_F32

Inst_VOP3__V_MAC_F32::~Inst_VOP3__V_MAC_F32() {} // ~Inst_VOP3__V_MAC_F32

// --- description from .arch file ---
// D.f = S0.f * S1.f + D.f.
// SQ translates to V_MAD_F32.
void
Inst_VOP3__V_MAC_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    vdst.read();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (instData.ABS & 0x2) {
        src1.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::fma(src0[lane], src1[lane], vdst[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_ADD_CO_U32 class methods ---

Inst_VOP3__V_ADD_CO_U32::Inst_VOP3__V_ADD_CO_U32(InFmt_VOP3B *iFmt)
    : Inst_VOP3B(iFmt, "v_add_co_u32")
{
    setFlag(ALU);
    setFlag(WritesVCC);
} // Inst_VOP3__V_ADD_CO_U32

Inst_VOP3__V_ADD_CO_U32::~Inst_VOP3__V_ADD_CO_U32() {
} // ~Inst_VOP3__V_ADD_CO_U32

// --- description from .arch file ---
// D.u = S0.u + S1.u;
// VCC[threadId] = (S0.u + S1.u >= 0x800000000ULL ? 1 : 0) is an UNSIGNED
// ---  overflow or carry-out for V_ADDC_U32.
// In VOP3 the VCC destination may be an arbitrary SGPR-pair.
void
Inst_VOP3__V_ADD_CO_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);
    ScalarOperandU64 vcc(gpuDynInst, instData.SDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src0[lane] + src1[lane];
            vcc.setBit(lane, ((VecElemU64)src0[lane] +
                              (VecElemU64)src1[lane]) >= 0x100000000ULL ?
                                 1 :
                                 0);
        }
    }

    vdst.write();
    vcc.write();
} // execute

// --- Inst_VOP3__V_SUB_CO_U32 class methods ---

Inst_VOP3__V_SUB_CO_U32::Inst_VOP3__V_SUB_CO_U32(InFmt_VOP3B *iFmt)
    : Inst_VOP3B(iFmt, "v_sub_co_u32")
{
    setFlag(ALU);
    setFlag(WritesVCC);
} // Inst_VOP3__V_SUB_CO_U32

Inst_VOP3__V_SUB_CO_U32::~Inst_VOP3__V_SUB_CO_U32() {
} // ~Inst_VOP3__V_SUB_CO_U32

// --- description from .arch file ---
// D.u = S0.u - S1.u;
// VCC[threadId] = (S1.u > S0.u ? 1 : 0) is an UNSIGNED overflow or
// carry-out for V_SUBB_U32.
// In VOP3 the VCC destination may be an arbitrary SGPR-pair.
void
Inst_VOP3__V_SUB_CO_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);
    ScalarOperandU64 vcc(gpuDynInst, instData.SDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src0[lane] - src1[lane];
            vcc.setBit(lane, src1[lane] > src0[lane] ? 1 : 0);
        }
    }

    vdst.write();
    vcc.write();
} // execute

// --- Inst_VOP3__V_SUBREV_CO_U32 class methods ---

Inst_VOP3__V_SUBREV_CO_U32::Inst_VOP3__V_SUBREV_CO_U32(InFmt_VOP3B *iFmt)
    : Inst_VOP3B(iFmt, "v_subrev_co_u32")
{
    setFlag(ALU);
    setFlag(WritesVCC);
} // Inst_VOP3__V_SUBREV_CO_U32

Inst_VOP3__V_SUBREV_CO_U32::~Inst_VOP3__V_SUBREV_CO_U32() {
} // ~Inst_VOP3__V_SUBREV_CO_U32

// --- description from .arch file ---
// D.u = S1.u - S0.u;
// VCC[threadId] = (S0.u > S1.u ? 1 : 0) is an UNSIGNED overflow or
// carry-out for V_SUBB_U32.
// In VOP3 the VCC destination may be an arbitrary SGPR-pair.
// SQ translates this to V_SUB_U32 with reversed operands.
void
Inst_VOP3__V_SUBREV_CO_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);
    ScalarOperandU64 vcc(gpuDynInst, instData.SDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src1[lane] - src0[lane];
            vcc.setBit(lane, src0[lane] > src1[lane] ? 1 : 0);
        }
    }

    vdst.write();
    vcc.write();
} // execute

// --- Inst_VOP3__V_ADDC_CO_U32 class methods ---

Inst_VOP3__V_ADDC_CO_U32::Inst_VOP3__V_ADDC_CO_U32(InFmt_VOP3B *iFmt)
    : Inst_VOP3B(iFmt, "v_addc_co_u32")
{
    setFlag(ALU);
    setFlag(WritesVCC);
    setFlag(ReadsVCC);
} // Inst_VOP3__V_ADDC_CO_U32

Inst_VOP3__V_ADDC_CO_U32::~Inst_VOP3__V_ADDC_CO_U32() {
} // ~Inst_VOP3__V_ADDC_CO_U32

// --- description from .arch file ---
// D.u = S0.u + S1.u + VCC[threadId];
// VCC[threadId] = (S0.u + S1.u + VCC[threadId] >= 0x800000000ULL ? 1 : 0)
// is an UNSIGNED overflow.
// In VOP3 the VCC destination may be an arbitrary SGPR-pair, and the VCC
// source comes from the SGPR-pair at S2.u.
void
Inst_VOP3__V_ADDC_CO_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ConstScalarOperandU64 vcc(gpuDynInst, extData.SRC2);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);
    ScalarOperandU64 sdst(gpuDynInst, instData.SDST);

    src0.readSrc();
    src1.readSrc();
    vcc.read();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src0[lane] + src1[lane] + bits(vcc.rawData(), lane);
            sdst.setBit(
                lane, ((VecElemU64)src0[lane] + (VecElemU64)src1[lane] +
                       (VecElemU64)bits(vcc.rawData(), lane)) >= 0x100000000 ?
                          1 :
                          0);
        }
    }

    vdst.write();
    sdst.write();
} // execute

// --- Inst_VOP3__V_SUBB_CO_U32 class methods ---

Inst_VOP3__V_SUBB_CO_U32::Inst_VOP3__V_SUBB_CO_U32(InFmt_VOP3B *iFmt)
    : Inst_VOP3B(iFmt, "v_subb_co_u32")
{
    setFlag(ALU);
    setFlag(WritesVCC);
    setFlag(ReadsVCC);
} // Inst_VOP3__V_SUBB_CO_U32

Inst_VOP3__V_SUBB_CO_U32::~Inst_VOP3__V_SUBB_CO_U32() {
} // ~Inst_VOP3__V_SUBB_CO_U32

// --- description from .arch file ---
// D.u = S0.u - S1.u - VCC[threadId];
// VCC[threadId] = (S1.u + VCC[threadId] > S0.u ? 1 : 0) is an UNSIGNED
// ---  overflow.
// In VOP3 the VCC destination may be an arbitrary SGPR-pair, and the VCC
// ---  source comes from the SGPR-pair at S2.u.
void
Inst_VOP3__V_SUBB_CO_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ConstScalarOperandU64 vcc(gpuDynInst, extData.SRC2);
    ScalarOperandU64 sdst(gpuDynInst, instData.SDST);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    vcc.read();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src0[lane] - src1[lane] - bits(vcc.rawData(), lane);
            sdst.setBit(
                lane,
                (src1[lane] + bits(vcc.rawData(), lane)) > src0[lane] ? 1 : 0);
        }
    }

    vdst.write();
    sdst.write();
} // execute

// --- Inst_VOP3__V_SUBBREV_CO_U32 class methods ---

Inst_VOP3__V_SUBBREV_CO_U32::Inst_VOP3__V_SUBBREV_CO_U32(InFmt_VOP3B *iFmt)
    : Inst_VOP3B(iFmt, "v_subbrev_co_u32")
{
    setFlag(ALU);
    setFlag(WritesVCC);
    setFlag(ReadsVCC);
} // Inst_VOP3__V_SUBBREV_CO_U32

Inst_VOP3__V_SUBBREV_CO_U32::~Inst_VOP3__V_SUBBREV_CO_U32() {
} // ~Inst_VOP3__V_SUBBREV_CO_U32

// --- description from .arch file ---
// D.u = S1.u - S0.u - VCC[threadId];
// VCC[threadId] = (S1.u + VCC[threadId] > S0.u ? 1 : 0) is an UNSIGNED
// overflow.
// In VOP3 the VCC destination may be an arbitrary SGPR-pair, and the VCC
// source comes from the SGPR-pair at S2.u. SQ translates to V_SUBB_U32.
void
Inst_VOP3__V_SUBBREV_CO_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstScalarOperandU64 sdst(gpuDynInst, instData.SDST);
    ScalarOperandU64 vcc(gpuDynInst, extData.SRC2);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    vcc.read();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src1[lane] - src0[lane] - bits(vcc.rawData(), lane);
            sdst.setBit(
                lane,
                (src1[lane] + bits(vcc.rawData(), lane)) > src0[lane] ? 1 : 0);
        }
    }

    vdst.write();
    sdst.write();
} // execute

// --- Inst_VOP3__V_ADD_F16 class methods ---

Inst_VOP3__V_ADD_F16::Inst_VOP3__V_ADD_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_add_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_ADD_F16

Inst_VOP3__V_ADD_F16::~Inst_VOP3__V_ADD_F16() {} // ~Inst_VOP3__V_ADD_F16

// --- description from .arch file ---
// D.f16 = S0.f16 + S1.f16.
// Supports denormals, round mode, exception flags, saturation.
void
Inst_VOP3__V_ADD_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_SUB_F16 class methods ---

Inst_VOP3__V_SUB_F16::Inst_VOP3__V_SUB_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_sub_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_SUB_F16

Inst_VOP3__V_SUB_F16::~Inst_VOP3__V_SUB_F16() {} // ~Inst_VOP3__V_SUB_F16

// --- description from .arch file ---
// D.f16 = S0.f16 - S1.f16.
// Supports denormals, round mode, exception flags, saturation.
// SQ translates to V_ADD_F16.
void
Inst_VOP3__V_SUB_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_SUBREV_F16 class methods ---

Inst_VOP3__V_SUBREV_F16::Inst_VOP3__V_SUBREV_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_subrev_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_SUBREV_F16

Inst_VOP3__V_SUBREV_F16::~Inst_VOP3__V_SUBREV_F16() {
} // ~Inst_VOP3__V_SUBREV_F16

// --- description from .arch file ---
// D.f16 = S1.f16 - S0.f16.
// Supports denormals, round mode, exception flags, saturation.
// SQ translates to V_ADD_F16.
void
Inst_VOP3__V_SUBREV_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_MUL_F16 class methods ---

Inst_VOP3__V_MUL_F16::Inst_VOP3__V_MUL_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_mul_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_MUL_F16

Inst_VOP3__V_MUL_F16::~Inst_VOP3__V_MUL_F16() {} // ~Inst_VOP3__V_MUL_F16

// --- description from .arch file ---
// D.f16 = S0.f16 * S1.f16.
// Supports denormals, round mode, exception flags, saturation.
void
Inst_VOP3__V_MUL_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_MAC_F16 class methods ---

Inst_VOP3__V_MAC_F16::Inst_VOP3__V_MAC_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_mac_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
    setFlag(MAC);
} // Inst_VOP3__V_MAC_F16

Inst_VOP3__V_MAC_F16::~Inst_VOP3__V_MAC_F16() {} // ~Inst_VOP3__V_MAC_F16

// --- description from .arch file ---
// D.f16 = S0.f16 * S1.f16 + D.f16.
// Supports round mode, exception flags, saturation.
// SQ translates this to V_MAD_F16.
void
Inst_VOP3__V_MAC_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_ADD_U16 class methods ---

Inst_VOP3__V_ADD_U16::Inst_VOP3__V_ADD_U16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_add_u16", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_ADD_U16

Inst_VOP3__V_ADD_U16::~Inst_VOP3__V_ADD_U16() {} // ~Inst_VOP3__V_ADD_U16

// --- description from .arch file ---
// D.u16 = S0.u16 + S1.u16.
// Supports saturation (unsigned 16-bit integer domain).
void
Inst_VOP3__V_ADD_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU16 src1(gpuDynInst, extData.SRC1);
    VecOperandU16 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src0[lane] + src1[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_SUB_U16 class methods ---

Inst_VOP3__V_SUB_U16::Inst_VOP3__V_SUB_U16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_sub_u16", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_SUB_U16

Inst_VOP3__V_SUB_U16::~Inst_VOP3__V_SUB_U16() {} // ~Inst_VOP3__V_SUB_U16

// --- description from .arch file ---
// D.u16 = S0.u16 - S1.u16.
// Supports saturation (unsigned 16-bit integer domain).
void
Inst_VOP3__V_SUB_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU16 src1(gpuDynInst, extData.SRC1);
    VecOperandU16 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src0[lane] - src1[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_SUBREV_U16 class methods ---

Inst_VOP3__V_SUBREV_U16::Inst_VOP3__V_SUBREV_U16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_subrev_u16", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_SUBREV_U16

Inst_VOP3__V_SUBREV_U16::~Inst_VOP3__V_SUBREV_U16() {
} // ~Inst_VOP3__V_SUBREV_U16

// --- description from .arch file ---
// D.u16 = S1.u16 - S0.u16.
// Supports saturation (unsigned 16-bit integer domain).
// SQ translates this to V_SUB_U16 with reversed operands.
void
Inst_VOP3__V_SUBREV_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU16 src1(gpuDynInst, extData.SRC1);
    VecOperandU16 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src1[lane] - src0[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MUL_LO_U16 class methods ---

Inst_VOP3__V_MUL_LO_U16::Inst_VOP3__V_MUL_LO_U16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_mul_lo_u16", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_MUL_LO_U16

Inst_VOP3__V_MUL_LO_U16::~Inst_VOP3__V_MUL_LO_U16() {
} // ~Inst_VOP3__V_MUL_LO_U16

// --- description from .arch file ---
// D.u16 = S0.u16 * S1.u16.
// Supports saturation (unsigned 16-bit integer domain).
void
Inst_VOP3__V_MUL_LO_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU16 src1(gpuDynInst, extData.SRC1);
    VecOperandU16 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src0[lane] * src1[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_LSHLREV_B16 class methods ---

Inst_VOP3__V_LSHLREV_B16::Inst_VOP3__V_LSHLREV_B16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_lshlrev_b16", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_LSHLREV_B16

Inst_VOP3__V_LSHLREV_B16::~Inst_VOP3__V_LSHLREV_B16() {
} // ~Inst_VOP3__V_LSHLREV_B16

// --- description from .arch file ---
// D.u[15:0] = S1.u[15:0] << S0.u[3:0].
// SQ translates this to an internal SP opcode.
void
Inst_VOP3__V_LSHLREV_B16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU16 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU16 src0(gpuDynInst, extData.SRC0);
    VecOperandU16 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src1[lane] << bits(src0[lane], 3, 0);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_LSHRREV_B16 class methods ---

Inst_VOP3__V_LSHRREV_B16::Inst_VOP3__V_LSHRREV_B16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_lshrrev_b16", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_LSHRREV_B16

Inst_VOP3__V_LSHRREV_B16::~Inst_VOP3__V_LSHRREV_B16() {
} // ~Inst_VOP3__V_LSHRREV_B16

// --- description from .arch file ---
// D.u[15:0] = S1.u[15:0] >> S0.u[3:0].
// The vacated bits are set to zero.
// SQ translates this to an internal SP opcode.
void
Inst_VOP3__V_LSHRREV_B16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU16 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU16 src0(gpuDynInst, extData.SRC0);
    VecOperandU16 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (instData.ABS & 0x2) {
        src1.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src1[lane] >> bits(src0[lane], 3, 0);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_ASHRREV_I16 class methods ---

Inst_VOP3__V_ASHRREV_I16::Inst_VOP3__V_ASHRREV_I16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_ashrrev_i16", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_ASHRREV_I16

Inst_VOP3__V_ASHRREV_I16::~Inst_VOP3__V_ASHRREV_I16() {
} // ~Inst_VOP3__V_ASHRREV_I16

// --- description from .arch file ---
// D.i[15:0] = signext(S1.i[15:0]) >> S0.i[3:0].
// The vacated bits are set to the sign bit of the input value.
// SQ translates this to an internal SP opcode.
void
Inst_VOP3__V_ASHRREV_I16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI16 src1(gpuDynInst, extData.SRC1);
    VecOperandI16 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src1[lane] >> bits(src0[lane], 3, 0);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MAX_F16 class methods ---

Inst_VOP3__V_MAX_F16::Inst_VOP3__V_MAX_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_max_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_MAX_F16

Inst_VOP3__V_MAX_F16::~Inst_VOP3__V_MAX_F16() {} // ~Inst_VOP3__V_MAX_F16

// --- description from .arch file ---
// D.f16 = max(S0.f16, S1.f16).
// IEEE compliant. Supports denormals, round mode, exception flags,
// saturation.
void
Inst_VOP3__V_MAX_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_MIN_F16 class methods ---

Inst_VOP3__V_MIN_F16::Inst_VOP3__V_MIN_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_min_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_MIN_F16

Inst_VOP3__V_MIN_F16::~Inst_VOP3__V_MIN_F16() {} // ~Inst_VOP3__V_MIN_F16

// --- description from .arch file ---
// D.f16 = min(S0.f16, S1.f16).
// IEEE compliant. Supports denormals, round mode, exception flags,
// saturation.
void
Inst_VOP3__V_MIN_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_MAX_U16 class methods ---

Inst_VOP3__V_MAX_U16::Inst_VOP3__V_MAX_U16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_max_u16", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_MAX_U16

Inst_VOP3__V_MAX_U16::~Inst_VOP3__V_MAX_U16() {} // ~Inst_VOP3__V_MAX_U16

// --- description from .arch file ---
// D.u[15:0] = max(S0.u[15:0], S1.u[15:0]).
void
Inst_VOP3__V_MAX_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU16 src1(gpuDynInst, extData.SRC1);
    VecOperandU16 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (instData.ABS & 0x2) {
        src1.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::max(src0[lane], src1[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MAX_I16 class methods ---

Inst_VOP3__V_MAX_I16::Inst_VOP3__V_MAX_I16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_max_i16", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_MAX_I16

Inst_VOP3__V_MAX_I16::~Inst_VOP3__V_MAX_I16() {} // ~Inst_VOP3__V_MAX_I16

// --- description from .arch file ---
// D.i[15:0] = max(S0.i[15:0], S1.i[15:0]).
void
Inst_VOP3__V_MAX_I16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI16 src1(gpuDynInst, extData.SRC1);
    VecOperandI16 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (instData.ABS & 0x2) {
        src1.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::max(src0[lane], src1[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MIN_U16 class methods ---

Inst_VOP3__V_MIN_U16::Inst_VOP3__V_MIN_U16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_min_u16", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_MIN_U16

Inst_VOP3__V_MIN_U16::~Inst_VOP3__V_MIN_U16() {} // ~Inst_VOP3__V_MIN_U16

// --- description from .arch file ---
// D.u[15:0] = min(S0.u[15:0], S1.u[15:0]).
void
Inst_VOP3__V_MIN_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU16 src1(gpuDynInst, extData.SRC1);
    VecOperandU16 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (instData.ABS & 0x2) {
        src1.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::min(src0[lane], src1[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MIN_I16 class methods ---

Inst_VOP3__V_MIN_I16::Inst_VOP3__V_MIN_I16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_min_i16", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_MIN_I16

Inst_VOP3__V_MIN_I16::~Inst_VOP3__V_MIN_I16() {} // ~Inst_VOP3__V_MIN_I16

// --- description from .arch file ---
// D.i[15:0] = min(S0.i[15:0], S1.i[15:0]).
void
Inst_VOP3__V_MIN_I16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI16 src1(gpuDynInst, extData.SRC1);
    VecOperandI16 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (instData.ABS & 0x2) {
        src1.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::min(src0[lane], src1[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_LDEXP_F16 class methods ---

Inst_VOP3__V_LDEXP_F16::Inst_VOP3__V_LDEXP_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_ldexp_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_LDEXP_F16

Inst_VOP3__V_LDEXP_F16::~Inst_VOP3__V_LDEXP_F16() {} // ~Inst_VOP3__V_LDEXP_F16

// --- description from .arch file ---
// D.f16 = S0.f16 * (2 ** S1.i16).
void
Inst_VOP3__V_LDEXP_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_ADD_U32 class methods ---

Inst_VOP3__V_ADD_U32::Inst_VOP3__V_ADD_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_add_u32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_ADD_U32

Inst_VOP3__V_ADD_U32::~Inst_VOP3__V_ADD_U32() {} // ~Inst_VOP3__V_ADD_U32

// --- description from .arch file ---
// D.u32 = S0.u32 + S1.u32.
void
Inst_VOP3__V_ADD_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src0[lane] + src1[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_SUB_U32 class methods ---

Inst_VOP3__V_SUB_U32::Inst_VOP3__V_SUB_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_sub_u32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_SUB_U32

Inst_VOP3__V_SUB_U32::~Inst_VOP3__V_SUB_U32() {} // ~Inst_VOP3__V_SUB_U32

// --- description from .arch file ---
// D.u32 = S0.u32 - S1.u32.
void
Inst_VOP3__V_SUB_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src0[lane] - src1[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_SUBREV_U32 class methods ---

Inst_VOP3__V_SUBREV_U32::Inst_VOP3__V_SUBREV_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_subrev_u32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_SUBREV_U32

Inst_VOP3__V_SUBREV_U32::~Inst_VOP3__V_SUBREV_U32() {
} // ~Inst_VOP3__V_SUBREV_U32

// --- description from .arch file ---
// D.u32 = S1.u32 - S0.u32.
void
Inst_VOP3__V_SUBREV_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src1[lane] - src0[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_NOP class methods ---

Inst_VOP3__V_NOP::Inst_VOP3__V_NOP(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_nop", false)
{
    setFlag(Nop);
    setFlag(ALU);
} // Inst_VOP3__V_NOP

Inst_VOP3__V_NOP::~Inst_VOP3__V_NOP() {} // ~Inst_VOP3__V_NOP

// --- description from .arch file ---
// Do nothing.
void
Inst_VOP3__V_NOP::execute(GPUDynInstPtr gpuDynInst)
{} // execute

// --- Inst_VOP3__V_MOV_B32 class methods ---

Inst_VOP3__V_MOV_B32::Inst_VOP3__V_MOV_B32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_mov_b32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_MOV_B32

Inst_VOP3__V_MOV_B32::~Inst_VOP3__V_MOV_B32() {} // ~Inst_VOP3__V_MOV_B32

// --- description from .arch file ---
// D.u = S0.u.
// Input and output modifiers not supported; this is an untyped operation.
void
Inst_VOP3__V_MOV_B32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src(gpuDynInst, extData.SRC0);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_CVT_I32_F64 class methods ---

Inst_VOP3__V_CVT_I32_F64::Inst_VOP3__V_CVT_I32_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_i32_f64", false)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_CVT_I32_F64

Inst_VOP3__V_CVT_I32_F64::~Inst_VOP3__V_CVT_I32_F64() {
} // ~Inst_VOP3__V_CVT_I32_F64

// --- description from .arch file ---
// D.i = (int)S0.d.
// Out-of-range floating point values (including infinity) saturate. NaN is
// ---  converted to 0.
void
Inst_VOP3__V_CVT_I32_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src(gpuDynInst, extData.SRC0);
    VecOperandI32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            int exp;
            std::frexp(src[lane], &exp);
            if (std::isnan(src[lane])) {
                vdst[lane] = 0;
            } else if (std::isinf(src[lane]) || exp > 30) {
                if (std::signbit(src[lane])) {
                    vdst[lane] = INT_MIN;
                } else {
                    vdst[lane] = INT_MAX;
                }
            } else {
                vdst[lane] = (VecElemI32)src[lane];
            }
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_CVT_F64_I32 class methods ---

Inst_VOP3__V_CVT_F64_I32::Inst_VOP3__V_CVT_F64_I32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_f64_i32", false)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_CVT_F64_I32

Inst_VOP3__V_CVT_F64_I32::~Inst_VOP3__V_CVT_F64_I32() {
} // ~Inst_VOP3__V_CVT_F64_I32

// --- description from .arch file ---
// D.d = (double)S0.i.
void
Inst_VOP3__V_CVT_F64_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src(gpuDynInst, extData.SRC0);
    VecOperandF64 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = (VecElemF64)src[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_CVT_F32_I32 class methods ---

Inst_VOP3__V_CVT_F32_I32::Inst_VOP3__V_CVT_F32_I32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_f32_i32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CVT_F32_I32

Inst_VOP3__V_CVT_F32_I32::~Inst_VOP3__V_CVT_F32_I32() {
} // ~Inst_VOP3__V_CVT_F32_I32

// --- description from .arch file ---
// D.f = (float)S0.i.
void
Inst_VOP3__V_CVT_F32_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    VecOperandI32 src(gpuDynInst, extData.SRC0);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = (VecElemF32)src[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_CVT_F32_U32 class methods ---

Inst_VOP3__V_CVT_F32_U32::Inst_VOP3__V_CVT_F32_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_f32_u32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CVT_F32_U32

Inst_VOP3__V_CVT_F32_U32::~Inst_VOP3__V_CVT_F32_U32() {
} // ~Inst_VOP3__V_CVT_F32_U32

// --- description from .arch file ---
// D.f = (float)S0.u.
void
Inst_VOP3__V_CVT_F32_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src(gpuDynInst, extData.SRC0);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = (VecElemF32)src[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_CVT_U32_F32 class methods ---

Inst_VOP3__V_CVT_U32_F32::Inst_VOP3__V_CVT_U32_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_u32_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CVT_U32_F32

Inst_VOP3__V_CVT_U32_F32::~Inst_VOP3__V_CVT_U32_F32() {
} // ~Inst_VOP3__V_CVT_U32_F32

// --- description from .arch file ---
// D.u = (unsigned)S0.f.
// Out-of-range floating point values (including infinity) saturate. NaN is
// ---  converted to 0.
void
Inst_VOP3__V_CVT_U32_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src(gpuDynInst, extData.SRC0);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            int exp;
            std::frexp(src[lane], &exp);
            if (std::isnan(src[lane])) {
                vdst[lane] = 0;
            } else if (std::isinf(src[lane])) {
                if (std::signbit(src[lane])) {
                    vdst[lane] = 0;
                } else {
                    vdst[lane] = UINT_MAX;
                }
            } else if (exp > 31) {
                vdst[lane] = UINT_MAX;
            } else {
                vdst[lane] = (VecElemU32)src[lane];
            }
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_CVT_I32_F32 class methods ---

Inst_VOP3__V_CVT_I32_F32::Inst_VOP3__V_CVT_I32_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_i32_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CVT_I32_F32

Inst_VOP3__V_CVT_I32_F32::~Inst_VOP3__V_CVT_I32_F32() {
} // ~Inst_VOP3__V_CVT_I32_F32

// --- description from .arch file ---
// D.i = (int)S0.f.
// Out-of-range floating point values (including infinity) saturate. NaN is
// ---  converted to 0.
void
Inst_VOP3__V_CVT_I32_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src(gpuDynInst, extData.SRC0);
    VecOperandI32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            int exp;
            std::frexp(src[lane], &exp);
            if (std::isnan(src[lane])) {
                vdst[lane] = 0;
            } else if (std::isinf(src[lane]) || exp > 30) {
                if (std::signbit(src[lane])) {
                    vdst[lane] = INT_MIN;
                } else {
                    vdst[lane] = INT_MAX;
                }
            } else {
                vdst[lane] = (VecElemI32)src[lane];
            }
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MOV_FED_B32 class methods ---

Inst_VOP3__V_MOV_FED_B32::Inst_VOP3__V_MOV_FED_B32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_mov_fed_b32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_MOV_FED_B32

Inst_VOP3__V_MOV_FED_B32::~Inst_VOP3__V_MOV_FED_B32() {
} // ~Inst_VOP3__V_MOV_FED_B32

// --- description from .arch file ---
// D.u = S0.u;
// Introduce EDC double error upon write to dest vgpr without causing an
// ---  exception.
// Input and output modifiers not supported; this is an untyped operation.
void
Inst_VOP3__V_MOV_FED_B32::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CVT_F16_F32 class methods ---

Inst_VOP3__V_CVT_F16_F32::Inst_VOP3__V_CVT_F16_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_f16_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CVT_F16_F32

Inst_VOP3__V_CVT_F16_F32::~Inst_VOP3__V_CVT_F16_F32() {
} // ~Inst_VOP3__V_CVT_F16_F32

// --- description from .arch file ---
// D.f16 = flt32_to_flt16(S0.f).
// Supports input modifiers and creates FP16 denormals when appropriate.
void
Inst_VOP3__V_CVT_F16_F32::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CVT_F32_F16 class methods ---

Inst_VOP3__V_CVT_F32_F16::Inst_VOP3__V_CVT_F32_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_f32_f16", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CVT_F32_F16

Inst_VOP3__V_CVT_F32_F16::~Inst_VOP3__V_CVT_F32_F16() {
} // ~Inst_VOP3__V_CVT_F32_F16

// --- description from .arch file ---
// D.f = flt16_to_flt32(S0.f16).
// FP16 denormal inputs are always accepted.
void
Inst_VOP3__V_CVT_F32_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CVT_RPI_I32_F32 class methods ---

Inst_VOP3__V_CVT_RPI_I32_F32::Inst_VOP3__V_CVT_RPI_I32_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_rpi_i32_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CVT_RPI_I32_F32

Inst_VOP3__V_CVT_RPI_I32_F32::~Inst_VOP3__V_CVT_RPI_I32_F32() {
} // ~Inst_VOP3__V_CVT_RPI_I32_F32

// --- description from .arch file ---
// D.i = (int)floor(S0.f + 0.5).
void
Inst_VOP3__V_CVT_RPI_I32_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src(gpuDynInst, extData.SRC0);
    VecOperandI32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = (VecElemI32)std::floor(src[lane] + 0.5);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_CVT_FLR_I32_F32 class methods ---

Inst_VOP3__V_CVT_FLR_I32_F32::Inst_VOP3__V_CVT_FLR_I32_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_flr_i32_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CVT_FLR_I32_F32

Inst_VOP3__V_CVT_FLR_I32_F32::~Inst_VOP3__V_CVT_FLR_I32_F32() {
} // ~Inst_VOP3__V_CVT_FLR_I32_F32

// --- description from .arch file ---
// D.i = (int)floor(S0.f).
void
Inst_VOP3__V_CVT_FLR_I32_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src(gpuDynInst, extData.SRC0);
    VecOperandI32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = (VecElemI32)std::floor(src[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_CVT_OFF_F32_I4 class methods ---

Inst_VOP3__V_CVT_OFF_F32_I4::Inst_VOP3__V_CVT_OFF_F32_I4(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_off_f32_i4", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CVT_OFF_F32_I4

Inst_VOP3__V_CVT_OFF_F32_I4::~Inst_VOP3__V_CVT_OFF_F32_I4() {
} // ~Inst_VOP3__V_CVT_OFF_F32_I4

// --- description from .arch file ---
// 4-bit signed int to 32-bit float. Used for interpolation in shader.
void
Inst_VOP3__V_CVT_OFF_F32_I4::execute(GPUDynInstPtr gpuDynInst)
{
    // Could not parse sq_uc.arch desc field
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CVT_F32_F64 class methods ---

Inst_VOP3__V_CVT_F32_F64::Inst_VOP3__V_CVT_F32_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_f32_f64", false)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_CVT_F32_F64

Inst_VOP3__V_CVT_F32_F64::~Inst_VOP3__V_CVT_F32_F64() {
} // ~Inst_VOP3__V_CVT_F32_F64

// --- description from .arch file ---
// D.f = (float)S0.d.
void
Inst_VOP3__V_CVT_F32_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src(gpuDynInst, extData.SRC0);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = (VecElemF32)src[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_CVT_F64_F32 class methods ---

Inst_VOP3__V_CVT_F64_F32::Inst_VOP3__V_CVT_F64_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_f64_f32", false)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_CVT_F64_F32

Inst_VOP3__V_CVT_F64_F32::~Inst_VOP3__V_CVT_F64_F32() {
} // ~Inst_VOP3__V_CVT_F64_F32

// --- description from .arch file ---
// D.d = (double)S0.f.
void
Inst_VOP3__V_CVT_F64_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src(gpuDynInst, extData.SRC0);
    VecOperandF64 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = (VecElemF64)src[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_CVT_F32_UBYTE0 class methods ---

Inst_VOP3__V_CVT_F32_UBYTE0::Inst_VOP3__V_CVT_F32_UBYTE0(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_f32_ubyte0", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CVT_F32_UBYTE0

Inst_VOP3__V_CVT_F32_UBYTE0::~Inst_VOP3__V_CVT_F32_UBYTE0() {
} // ~Inst_VOP3__V_CVT_F32_UBYTE0

// --- description from .arch file ---
// D.f = (float)(S0.u[7:0]).
void
Inst_VOP3__V_CVT_F32_UBYTE0::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src(gpuDynInst, extData.SRC0);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = (VecElemF32)bits(src[lane], 7, 0);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_CVT_F32_UBYTE1 class methods ---

Inst_VOP3__V_CVT_F32_UBYTE1::Inst_VOP3__V_CVT_F32_UBYTE1(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_f32_ubyte1", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CVT_F32_UBYTE1

Inst_VOP3__V_CVT_F32_UBYTE1::~Inst_VOP3__V_CVT_F32_UBYTE1() {
} // ~Inst_VOP3__V_CVT_F32_UBYTE1

// --- description from .arch file ---
// D.f = (float)(S0.u[15:8]).
void
Inst_VOP3__V_CVT_F32_UBYTE1::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src(gpuDynInst, extData.SRC0);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = (VecElemF32)bits(src[lane], 15, 8);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_CVT_F32_UBYTE2 class methods ---

Inst_VOP3__V_CVT_F32_UBYTE2::Inst_VOP3__V_CVT_F32_UBYTE2(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_f32_ubyte2", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CVT_F32_UBYTE2

Inst_VOP3__V_CVT_F32_UBYTE2::~Inst_VOP3__V_CVT_F32_UBYTE2() {
} // ~Inst_VOP3__V_CVT_F32_UBYTE2

// --- description from .arch file ---
// D.f = (float)(S0.u[23:16]).
void
Inst_VOP3__V_CVT_F32_UBYTE2::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src(gpuDynInst, extData.SRC0);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = (VecElemF32)bits(src[lane], 23, 16);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_CVT_F32_UBYTE3 class methods ---

Inst_VOP3__V_CVT_F32_UBYTE3::Inst_VOP3__V_CVT_F32_UBYTE3(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_f32_ubyte3", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CVT_F32_UBYTE3

Inst_VOP3__V_CVT_F32_UBYTE3::~Inst_VOP3__V_CVT_F32_UBYTE3() {
} // ~Inst_VOP3__V_CVT_F32_UBYTE3

// --- description from .arch file ---
// D.f = (float)(S0.u[31:24]).
void
Inst_VOP3__V_CVT_F32_UBYTE3::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src(gpuDynInst, extData.SRC0);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = (VecElemF32)bits(src[lane], 31, 24);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_CVT_U32_F64 class methods ---

Inst_VOP3__V_CVT_U32_F64::Inst_VOP3__V_CVT_U32_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_u32_f64", false)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_CVT_U32_F64

Inst_VOP3__V_CVT_U32_F64::~Inst_VOP3__V_CVT_U32_F64() {
} // ~Inst_VOP3__V_CVT_U32_F64

// --- description from .arch file ---
// D.u = (unsigned)S0.d.
// Out-of-range floating point values (including infinity) saturate. NaN is
// ---  converted to 0.
void
Inst_VOP3__V_CVT_U32_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src(gpuDynInst, extData.SRC0);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            int exp;
            std::frexp(src[lane], &exp);
            if (std::isnan(src[lane])) {
                vdst[lane] = 0;
            } else if (std::isinf(src[lane])) {
                if (std::signbit(src[lane])) {
                    vdst[lane] = 0;
                } else {
                    vdst[lane] = UINT_MAX;
                }
            } else if (exp > 31) {
                vdst[lane] = UINT_MAX;
            } else {
                vdst[lane] = (VecElemU32)src[lane];
            }
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_CVT_F64_U32 class methods ---

Inst_VOP3__V_CVT_F64_U32::Inst_VOP3__V_CVT_F64_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_f64_u32", false)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_CVT_F64_U32

Inst_VOP3__V_CVT_F64_U32::~Inst_VOP3__V_CVT_F64_U32() {
} // ~Inst_VOP3__V_CVT_F64_U32

// --- description from .arch file ---
// D.d = (double)S0.u.
void
Inst_VOP3__V_CVT_F64_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src(gpuDynInst, extData.SRC0);
    VecOperandF64 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = (VecElemF64)src[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_TRUNC_F64 class methods ---

Inst_VOP3__V_TRUNC_F64::Inst_VOP3__V_TRUNC_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_trunc_f64", false)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_TRUNC_F64

Inst_VOP3__V_TRUNC_F64::~Inst_VOP3__V_TRUNC_F64() {} // ~Inst_VOP3__V_TRUNC_F64

// --- description from .arch file ---
// D.d = trunc(S0.d), return integer part of S0.d.
void
Inst_VOP3__V_TRUNC_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src(gpuDynInst, extData.SRC0);
    VecOperandF64 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::trunc(src[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_CEIL_F64 class methods ---

Inst_VOP3__V_CEIL_F64::Inst_VOP3__V_CEIL_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_ceil_f64", false)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_CEIL_F64

Inst_VOP3__V_CEIL_F64::~Inst_VOP3__V_CEIL_F64() {} // ~Inst_VOP3__V_CEIL_F64

// --- description from .arch file ---
// D.d = trunc(S0.d);
// if (S0.d > 0.0 && S0.d != D.d) then D.d += 1.0.
void
Inst_VOP3__V_CEIL_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src(gpuDynInst, extData.SRC0);
    VecOperandF64 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::ceil(src[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_RNDNE_F64 class methods ---

Inst_VOP3__V_RNDNE_F64::Inst_VOP3__V_RNDNE_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_rndne_f64", false)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_RNDNE_F64

Inst_VOP3__V_RNDNE_F64::~Inst_VOP3__V_RNDNE_F64() {} // ~Inst_VOP3__V_RNDNE_F64

// --- description from .arch file ---
// D.d = round_nearest_even(S0.d).
void
Inst_VOP3__V_RNDNE_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src(gpuDynInst, extData.SRC0);
    VecOperandF64 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = roundNearestEven(src[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_FLOOR_F64 class methods ---

Inst_VOP3__V_FLOOR_F64::Inst_VOP3__V_FLOOR_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_floor_f64", false)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_FLOOR_F64

Inst_VOP3__V_FLOOR_F64::~Inst_VOP3__V_FLOOR_F64() {} // ~Inst_VOP3__V_FLOOR_F64

// --- description from .arch file ---
// D.d = trunc(S0.d);
// if (S0.d < 0.0 && S0.d != D.d) then D.d += -1.0.
void
Inst_VOP3__V_FLOOR_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src(gpuDynInst, extData.SRC0);
    VecOperandF64 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::floor(src[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_FRACT_F32 class methods ---

Inst_VOP3__V_FRACT_F32::Inst_VOP3__V_FRACT_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_fract_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_FRACT_F32

Inst_VOP3__V_FRACT_F32::~Inst_VOP3__V_FRACT_F32() {} // ~Inst_VOP3__V_FRACT_F32

// --- description from .arch file ---
// D.f = S0.f - floor(S0.f).
void
Inst_VOP3__V_FRACT_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src(gpuDynInst, extData.SRC0);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            VecElemF32 int_part(0.0);
            vdst[lane] = std::modf(src[lane], &int_part);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_TRUNC_F32 class methods ---

Inst_VOP3__V_TRUNC_F32::Inst_VOP3__V_TRUNC_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_trunc_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_TRUNC_F32

Inst_VOP3__V_TRUNC_F32::~Inst_VOP3__V_TRUNC_F32() {} // ~Inst_VOP3__V_TRUNC_F32

// --- description from .arch file ---
// D.f = trunc(S0.f), return integer part of S0.f.
void
Inst_VOP3__V_TRUNC_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src(gpuDynInst, extData.SRC0);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::trunc(src[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_CEIL_F32 class methods ---

Inst_VOP3__V_CEIL_F32::Inst_VOP3__V_CEIL_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_ceil_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CEIL_F32

Inst_VOP3__V_CEIL_F32::~Inst_VOP3__V_CEIL_F32() {} // ~Inst_VOP3__V_CEIL_F32

// --- description from .arch file ---
// D.f = trunc(S0.f);
// if (S0.f > 0.0 && S0.f != D.f) then D.f += 1.0.
void
Inst_VOP3__V_CEIL_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src(gpuDynInst, extData.SRC0);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::ceil(src[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_RNDNE_F32 class methods ---

Inst_VOP3__V_RNDNE_F32::Inst_VOP3__V_RNDNE_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_rndne_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_RNDNE_F32

Inst_VOP3__V_RNDNE_F32::~Inst_VOP3__V_RNDNE_F32() {} // ~Inst_VOP3__V_RNDNE_F32

// --- description from .arch file ---
// D.f = round_nearest_even(S0.f).
void
Inst_VOP3__V_RNDNE_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src(gpuDynInst, extData.SRC0);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = roundNearestEven(src[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_FLOOR_F32 class methods ---

Inst_VOP3__V_FLOOR_F32::Inst_VOP3__V_FLOOR_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_floor_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_FLOOR_F32

Inst_VOP3__V_FLOOR_F32::~Inst_VOP3__V_FLOOR_F32() {} // ~Inst_VOP3__V_FLOOR_F32

// --- description from .arch file ---
// D.f = trunc(S0.f);
// if (S0.f < 0.0 && S0.f != D.f) then D.f += -1.0.
void
Inst_VOP3__V_FLOOR_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src(gpuDynInst, extData.SRC0);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::floor(src[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_EXP_F32 class methods ---

Inst_VOP3__V_EXP_F32::Inst_VOP3__V_EXP_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_exp_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_EXP_F32

Inst_VOP3__V_EXP_F32::~Inst_VOP3__V_EXP_F32() {} // ~Inst_VOP3__V_EXP_F32

// --- description from .arch file ---
// D.f = pow(2.0, S0.f).
void
Inst_VOP3__V_EXP_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src(gpuDynInst, extData.SRC0);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::pow(2.0, src[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_LOG_F32 class methods ---

Inst_VOP3__V_LOG_F32::Inst_VOP3__V_LOG_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_log_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_LOG_F32

Inst_VOP3__V_LOG_F32::~Inst_VOP3__V_LOG_F32() {} // ~Inst_VOP3__V_LOG_F32

// --- description from .arch file ---
// D.f = log2(S0.f). Base 2 logarithm.
void
Inst_VOP3__V_LOG_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src(gpuDynInst, extData.SRC0);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::log2(src[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_RCP_F32 class methods ---

Inst_VOP3__V_RCP_F32::Inst_VOP3__V_RCP_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_rcp_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_RCP_F32

Inst_VOP3__V_RCP_F32::~Inst_VOP3__V_RCP_F32() {} // ~Inst_VOP3__V_RCP_F32

// --- description from .arch file ---
// D.f = 1.0 / S0.f. Reciprocal with IEEE rules and < 1ulp error.
void
Inst_VOP3__V_RCP_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src(gpuDynInst, extData.SRC0);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = 1.0 / src[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_RCP_IFLAG_F32 class methods ---

Inst_VOP3__V_RCP_IFLAG_F32::Inst_VOP3__V_RCP_IFLAG_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_rcp_iflag_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_RCP_IFLAG_F32

Inst_VOP3__V_RCP_IFLAG_F32::~Inst_VOP3__V_RCP_IFLAG_F32() {
} // ~Inst_VOP3__V_RCP_IFLAG_F32

// --- description from .arch file ---
// D.f = 1.0 / S0.f. Reciprocal intended for integer division, can raise
// ---  integer DIV_BY_ZERO exception but cannot raise floating-point
// ---  exceptions.
void
Inst_VOP3__V_RCP_IFLAG_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src(gpuDynInst, extData.SRC0);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = 1.0 / src[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_RSQ_F32 class methods ---

Inst_VOP3__V_RSQ_F32::Inst_VOP3__V_RSQ_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_rsq_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_RSQ_F32

Inst_VOP3__V_RSQ_F32::~Inst_VOP3__V_RSQ_F32() {} // ~Inst_VOP3__V_RSQ_F32

// --- description from .arch file ---
// D.f = 1.0 / sqrt(S0.f). Reciprocal square root with IEEE rules.
void
Inst_VOP3__V_RSQ_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src(gpuDynInst, extData.SRC0);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = 1.0 / std::sqrt(src[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_RCP_F64 class methods ---

Inst_VOP3__V_RCP_F64::Inst_VOP3__V_RCP_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_rcp_f64", false)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_RCP_F64

Inst_VOP3__V_RCP_F64::~Inst_VOP3__V_RCP_F64() {} // ~Inst_VOP3__V_RCP_F64

// --- description from .arch file ---
// D.d = 1.0 / S0.d.
void
Inst_VOP3__V_RCP_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src(gpuDynInst, extData.SRC0);
    VecOperandF64 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            if (std::fpclassify(src[lane]) == FP_ZERO) {
                vdst[lane] = +INFINITY;
            } else if (std::isnan(src[lane])) {
                vdst[lane] = NAN;
            } else if (std::isinf(src[lane])) {
                if (std::signbit(src[lane])) {
                    vdst[lane] = -0.0;
                } else {
                    vdst[lane] = 0.0;
                }
            } else {
                vdst[lane] = 1.0 / src[lane];
            }
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_RSQ_F64 class methods ---

Inst_VOP3__V_RSQ_F64::Inst_VOP3__V_RSQ_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_rsq_f64", false)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_RSQ_F64

Inst_VOP3__V_RSQ_F64::~Inst_VOP3__V_RSQ_F64() {} // ~Inst_VOP3__V_RSQ_F64

// --- description from .arch file ---
// D.d = 1.0 / sqrt(S0.d). See V_RSQ_F32.
void
Inst_VOP3__V_RSQ_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src(gpuDynInst, extData.SRC0);
    VecOperandF64 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            if (std::fpclassify(src[lane]) == FP_ZERO) {
                vdst[lane] = +INFINITY;
            } else if (std::isnan(src[lane])) {
                vdst[lane] = NAN;
            } else if (std::isinf(src[lane]) && !std::signbit(src[lane])) {
                vdst[lane] = 0.0;
            } else if (std::signbit(src[lane])) {
                vdst[lane] = NAN;
            } else {
                vdst[lane] = 1.0 / std::sqrt(src[lane]);
            }
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_SQRT_F32 class methods ---

Inst_VOP3__V_SQRT_F32::Inst_VOP3__V_SQRT_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_sqrt_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_SQRT_F32

Inst_VOP3__V_SQRT_F32::~Inst_VOP3__V_SQRT_F32() {} // ~Inst_VOP3__V_SQRT_F32

// --- description from .arch file ---
// D.f = sqrt(S0.f).
void
Inst_VOP3__V_SQRT_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src(gpuDynInst, extData.SRC0);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::sqrt(src[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_SQRT_F64 class methods ---

Inst_VOP3__V_SQRT_F64::Inst_VOP3__V_SQRT_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_sqrt_f64", false)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_SQRT_F64

Inst_VOP3__V_SQRT_F64::~Inst_VOP3__V_SQRT_F64() {} // ~Inst_VOP3__V_SQRT_F64

// --- description from .arch file ---
// D.d = sqrt(S0.d).
void
Inst_VOP3__V_SQRT_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src(gpuDynInst, extData.SRC0);
    VecOperandF64 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::sqrt(src[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_SIN_F32 class methods ---

Inst_VOP3__V_SIN_F32::Inst_VOP3__V_SIN_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_sin_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_SIN_F32

Inst_VOP3__V_SIN_F32::~Inst_VOP3__V_SIN_F32() {} // ~Inst_VOP3__V_SIN_F32

// --- description from .arch file ---
// D.f = sin(S0.f * 2 * PI).
// Valid range of S0.f is [-256.0, +256.0]. Out of range input results in
// float 0.0.
void
Inst_VOP3__V_SIN_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src(gpuDynInst, extData.SRC0);
    ConstScalarOperandF32 pi(gpuDynInst, REG_PI);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();
    pi.read();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::sin(src[lane] * 2 * pi.rawData());
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_COS_F32 class methods ---

Inst_VOP3__V_COS_F32::Inst_VOP3__V_COS_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cos_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_COS_F32

Inst_VOP3__V_COS_F32::~Inst_VOP3__V_COS_F32() {} // ~Inst_VOP3__V_COS_F32

// --- description from .arch file ---
// D.f = cos(S0.f * 2 * PI).
// Valid range of S0.f is [-256.0, +256.0]. Out of range input results in
// float 1.0.
void
Inst_VOP3__V_COS_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src(gpuDynInst, extData.SRC0);
    ConstScalarOperandF32 pi(gpuDynInst, REG_PI);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();
    pi.read();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::cos(src[lane] * 2 * pi.rawData());
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_NOT_B32 class methods ---

Inst_VOP3__V_NOT_B32::Inst_VOP3__V_NOT_B32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_not_b32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_NOT_B32

Inst_VOP3__V_NOT_B32::~Inst_VOP3__V_NOT_B32() {} // ~Inst_VOP3__V_NOT_B32

// --- description from .arch file ---
// D.u = ~S0.u.
// Input and output modifiers not supported.
void
Inst_VOP3__V_NOT_B32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src(gpuDynInst, extData.SRC0);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = ~src[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_BFREV_B32 class methods ---

Inst_VOP3__V_BFREV_B32::Inst_VOP3__V_BFREV_B32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_bfrev_b32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_BFREV_B32

Inst_VOP3__V_BFREV_B32::~Inst_VOP3__V_BFREV_B32() {} // ~Inst_VOP3__V_BFREV_B32

// --- description from .arch file ---
// D.u[31:0] = S0.u[0:31], bitfield reverse.
// Input and output modifiers not supported.
void
Inst_VOP3__V_BFREV_B32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src(gpuDynInst, extData.SRC0);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = reverseBits(src[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_FFBH_U32 class methods ---

Inst_VOP3__V_FFBH_U32::Inst_VOP3__V_FFBH_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_ffbh_u32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_FFBH_U32

Inst_VOP3__V_FFBH_U32::~Inst_VOP3__V_FFBH_U32() {} // ~Inst_VOP3__V_FFBH_U32

// --- description from .arch file ---
// D.u = position of first 1 in S0.u from MSB;
// D.u = 0xffffffff if S0.u == 0.
void
Inst_VOP3__V_FFBH_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src(gpuDynInst, extData.SRC0);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = findFirstOneMsb(src[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_FFBL_B32 class methods ---

Inst_VOP3__V_FFBL_B32::Inst_VOP3__V_FFBL_B32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_ffbl_b32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_FFBL_B32

Inst_VOP3__V_FFBL_B32::~Inst_VOP3__V_FFBL_B32() {} // ~Inst_VOP3__V_FFBL_B32

// --- description from .arch file ---
// D.u = position of first 1 in S0.u from LSB;
// D.u = 0xffffffff if S0.u == 0.
void
Inst_VOP3__V_FFBL_B32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src(gpuDynInst, extData.SRC0);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = findFirstOne(src[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_FFBH_I32 class methods ---

Inst_VOP3__V_FFBH_I32::Inst_VOP3__V_FFBH_I32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_ffbh_i32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_FFBH_I32

Inst_VOP3__V_FFBH_I32::~Inst_VOP3__V_FFBH_I32() {} // ~Inst_VOP3__V_FFBH_I32

// --- description from .arch file ---
// D.u = position of first bit different from sign bit in S0.i from MSB;
// D.u = 0xffffffff if S0.i == 0 or S0.i == 0xffffffff.
void
Inst_VOP3__V_FFBH_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src(gpuDynInst, extData.SRC0);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = firstOppositeSignBit(src[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_FREXP_EXP_I32_F64 class methods ---

Inst_VOP3__V_FREXP_EXP_I32_F64::Inst_VOP3__V_FREXP_EXP_I32_F64(
    InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_frexp_exp_i32_f64", false)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_FREXP_EXP_I32_F64

Inst_VOP3__V_FREXP_EXP_I32_F64::~Inst_VOP3__V_FREXP_EXP_I32_F64() {
} // ~Inst_VOP3__V_FREXP_EXP_I32_F64

// --- description from .arch file ---
// See V_FREXP_EXP_I32_F32.
void
Inst_VOP3__V_FREXP_EXP_I32_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src(gpuDynInst, extData.SRC0);
    VecOperandI32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            if (std::isinf(src[lane]) || std::isnan(src[lane])) {
                vdst[lane] = 0;
            } else {
                VecElemI32 exp(0);
                std::frexp(src[lane], &exp);
                vdst[lane] = exp;
            }
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_FREXP_MANT_F64 class methods ---

Inst_VOP3__V_FREXP_MANT_F64::Inst_VOP3__V_FREXP_MANT_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_frexp_mant_f64", false)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_FREXP_MANT_F64

Inst_VOP3__V_FREXP_MANT_F64::~Inst_VOP3__V_FREXP_MANT_F64() {
} // ~Inst_VOP3__V_FREXP_MANT_F64

// --- description from .arch file ---
// See V_FREXP_MANT_F32.
void
Inst_VOP3__V_FREXP_MANT_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src(gpuDynInst, extData.SRC0);
    VecOperandF64 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            VecElemI32 exp(0);
            vdst[lane] = std::frexp(src[lane], &exp);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_FRACT_F64 class methods ---

Inst_VOP3__V_FRACT_F64::Inst_VOP3__V_FRACT_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_fract_f64", false)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_FRACT_F64

Inst_VOP3__V_FRACT_F64::~Inst_VOP3__V_FRACT_F64() {} // ~Inst_VOP3__V_FRACT_F64

// --- description from .arch file ---
// See V_FRACT_F32.
void
Inst_VOP3__V_FRACT_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src(gpuDynInst, extData.SRC0);
    VecOperandF64 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            VecElemF32 int_part(0.0);
            vdst[lane] = std::modf(src[lane], &int_part);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_FREXP_EXP_I32_F32 class methods ---

Inst_VOP3__V_FREXP_EXP_I32_F32::Inst_VOP3__V_FREXP_EXP_I32_F32(
    InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_frexp_exp_i32_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_FREXP_EXP_I32_F32

Inst_VOP3__V_FREXP_EXP_I32_F32::~Inst_VOP3__V_FREXP_EXP_I32_F32() {
} // ~Inst_VOP3__V_FREXP_EXP_I32_F32

// --- description from .arch file ---
// if (S0.f == INF || S0.f == NAN) then D.i = 0;
// else D.i = TwosComplement(Exponent(S0.f) - 127 + 1).
// Returns exponent of single precision float input, such that S0.f =
// significand * (2 ** exponent). See also FREXP_MANT_F32, which returns
// the significand.
void
Inst_VOP3__V_FREXP_EXP_I32_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src(gpuDynInst, extData.SRC0);
    VecOperandI32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            if (std::isinf(src[lane]) || std::isnan(src[lane])) {
                vdst[lane] = 0;
            } else {
                VecElemI32 exp(0);
                std::frexp(src[lane], &exp);
                vdst[lane] = exp;
            }
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_FREXP_MANT_F32 class methods ---

Inst_VOP3__V_FREXP_MANT_F32::Inst_VOP3__V_FREXP_MANT_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_frexp_mant_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_FREXP_MANT_F32

Inst_VOP3__V_FREXP_MANT_F32::~Inst_VOP3__V_FREXP_MANT_F32() {
} // ~Inst_VOP3__V_FREXP_MANT_F32

// --- description from .arch file ---
// if (S0.f == INF || S0.f == NAN) then D.f = S0.f;
// else D.f = Mantissa(S0.f).
// Result range is in (-1.0,-0.5][0.5,1.0) in normal cases. Returns binary
// ---  significand of single precision float input, such that S0.f =
// ---  significand * (2 ** exponent). See also FREXP_EXP_I32_F32, which
// ---  returns integer exponent.
void
Inst_VOP3__V_FREXP_MANT_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src(gpuDynInst, extData.SRC0);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            if (std::isinf(src[lane]) || std::isnan(src[lane])) {
                vdst[lane] = src[lane];
            } else {
                VecElemI32 exp(0);
                vdst[lane] = std::frexp(src[lane], &exp);
            }
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_CLREXCP class methods ---

Inst_VOP3__V_CLREXCP::Inst_VOP3__V_CLREXCP(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_clrexcp", false)
{} // Inst_VOP3__V_CLREXCP

Inst_VOP3__V_CLREXCP::~Inst_VOP3__V_CLREXCP() {} // ~Inst_VOP3__V_CLREXCP

// --- description from .arch file ---
// Clear wave's exception state in SIMD (SP).
void
Inst_VOP3__V_CLREXCP::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CVT_F16_U16 class methods ---

Inst_VOP3__V_CVT_F16_U16::Inst_VOP3__V_CVT_F16_U16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_f16_u16", false)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_CVT_F16_U16

Inst_VOP3__V_CVT_F16_U16::~Inst_VOP3__V_CVT_F16_U16() {
} // ~Inst_VOP3__V_CVT_F16_U16

// --- description from .arch file ---
// D.f16 = uint16_to_flt16(S.u16).
// Supports denormals, rounding, exception flags and saturation.
void
Inst_VOP3__V_CVT_F16_U16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CVT_F16_I16 class methods ---

Inst_VOP3__V_CVT_F16_I16::Inst_VOP3__V_CVT_F16_I16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_f16_i16", false)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_CVT_F16_I16

Inst_VOP3__V_CVT_F16_I16::~Inst_VOP3__V_CVT_F16_I16() {
} // ~Inst_VOP3__V_CVT_F16_I16

// --- description from .arch file ---
// D.f16 = int16_to_flt16(S.i16).
// Supports denormals, rounding, exception flags and saturation.
void
Inst_VOP3__V_CVT_F16_I16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CVT_U16_F16 class methods ---

Inst_VOP3__V_CVT_U16_F16::Inst_VOP3__V_CVT_U16_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_u16_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_CVT_U16_F16

Inst_VOP3__V_CVT_U16_F16::~Inst_VOP3__V_CVT_U16_F16() {
} // ~Inst_VOP3__V_CVT_U16_F16

// --- description from .arch file ---
// D.u16 = flt16_to_uint16(S.f16).
// Supports rounding, exception flags and saturation.
void
Inst_VOP3__V_CVT_U16_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CVT_I16_F16 class methods ---

Inst_VOP3__V_CVT_I16_F16::Inst_VOP3__V_CVT_I16_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_i16_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_CVT_I16_F16

Inst_VOP3__V_CVT_I16_F16::~Inst_VOP3__V_CVT_I16_F16() {
} // ~Inst_VOP3__V_CVT_I16_F16

// --- description from .arch file ---
// D.i16 = flt16_to_int16(S.f16).
// Supports rounding, exception flags and saturation.
void
Inst_VOP3__V_CVT_I16_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_RCP_F16 class methods ---

Inst_VOP3__V_RCP_F16::Inst_VOP3__V_RCP_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_rcp_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_RCP_F16

Inst_VOP3__V_RCP_F16::~Inst_VOP3__V_RCP_F16() {} // ~Inst_VOP3__V_RCP_F16

// --- description from .arch file ---
// if (S0.f16 == 1.0f)
//     D.f16 = 1.0f;
// else
//     D.f16 = ApproximateRecip(S0.f16).
void
Inst_VOP3__V_RCP_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_SQRT_F16 class methods ---

Inst_VOP3__V_SQRT_F16::Inst_VOP3__V_SQRT_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_sqrt_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_SQRT_F16

Inst_VOP3__V_SQRT_F16::~Inst_VOP3__V_SQRT_F16() {} // ~Inst_VOP3__V_SQRT_F16

// --- description from .arch file ---
// if (S0.f16 == 1.0f)
//     D.f16 = 1.0f;
// else
//     D.f16 = ApproximateSqrt(S0.f16).
void
Inst_VOP3__V_SQRT_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_RSQ_F16 class methods ---

Inst_VOP3__V_RSQ_F16::Inst_VOP3__V_RSQ_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_rsq_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_RSQ_F16

Inst_VOP3__V_RSQ_F16::~Inst_VOP3__V_RSQ_F16() {} // ~Inst_VOP3__V_RSQ_F16

// --- description from .arch file ---
// if (S0.f16 == 1.0f)
//     D.f16 = 1.0f;
// else
//     D.f16 = ApproximateRecipSqrt(S0.f16).
void
Inst_VOP3__V_RSQ_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_LOG_F16 class methods ---

Inst_VOP3__V_LOG_F16::Inst_VOP3__V_LOG_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_log_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_LOG_F16

Inst_VOP3__V_LOG_F16::~Inst_VOP3__V_LOG_F16() {} // ~Inst_VOP3__V_LOG_F16

// --- description from .arch file ---
// if (S0.f16 == 1.0f)
//     D.f16 = 0.0f;
// else
//     D.f16 = ApproximateLog2(S0.f16).
void
Inst_VOP3__V_LOG_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_EXP_F16 class methods ---

Inst_VOP3__V_EXP_F16::Inst_VOP3__V_EXP_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_exp_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_EXP_F16

Inst_VOP3__V_EXP_F16::~Inst_VOP3__V_EXP_F16() {} // ~Inst_VOP3__V_EXP_F16

// --- description from .arch file ---
// if (S0.f16 == 0.0f)
//     D.f16 = 1.0f;
// else
//     D.f16 = Approximate2ToX(S0.f16).
void
Inst_VOP3__V_EXP_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_FREXP_MANT_F16 class methods ---

Inst_VOP3__V_FREXP_MANT_F16::Inst_VOP3__V_FREXP_MANT_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_frexp_mant_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_FREXP_MANT_F16

Inst_VOP3__V_FREXP_MANT_F16::~Inst_VOP3__V_FREXP_MANT_F16() {
} // ~Inst_VOP3__V_FREXP_MANT_F16

// --- description from .arch file ---
// if (S0.f16 == +-INF || S0.f16 == NAN)
//     D.f16 = S0.f16;
// else
//     D.f16 = mantissa(S0.f16).
// Result range is (-1.0,-0.5][0.5,1.0).
// C math library frexp function.
// Returns binary significand of half precision float input, such that the
// original single float = significand * (2 ** exponent).
void
Inst_VOP3__V_FREXP_MANT_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_FREXP_EXP_I16_F16 class methods ---

Inst_VOP3__V_FREXP_EXP_I16_F16::Inst_VOP3__V_FREXP_EXP_I16_F16(
    InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_frexp_exp_i16_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_FREXP_EXP_I16_F16

Inst_VOP3__V_FREXP_EXP_I16_F16::~Inst_VOP3__V_FREXP_EXP_I16_F16() {
} // ~Inst_VOP3__V_FREXP_EXP_I16_F16

// --- description from .arch file ---
// if (S0.f16 == +-INF || S0.f16 == NAN)
//     D.i16 = 0;
// else
//     D.i16 = 2s_complement(exponent(S0.f16) - 15 + 1).
// C math library frexp function.
// Returns exponent of half precision float input, such that the
// original single float = significand * (2 ** exponent).
void
Inst_VOP3__V_FREXP_EXP_I16_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_FLOOR_F16 class methods ---

Inst_VOP3__V_FLOOR_F16::Inst_VOP3__V_FLOOR_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_floor_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_FLOOR_F16

Inst_VOP3__V_FLOOR_F16::~Inst_VOP3__V_FLOOR_F16() {} // ~Inst_VOP3__V_FLOOR_F16

// --- description from .arch file ---
// D.f16 = trunc(S0.f16);
// if (S0.f16 < 0.0f && S0.f16 != D.f16) then D.f16 -= 1.0f.
void
Inst_VOP3__V_FLOOR_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CEIL_F16 class methods ---

Inst_VOP3__V_CEIL_F16::Inst_VOP3__V_CEIL_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_ceil_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_CEIL_F16

Inst_VOP3__V_CEIL_F16::~Inst_VOP3__V_CEIL_F16() {} // ~Inst_VOP3__V_CEIL_F16

// --- description from .arch file ---
// D.f16 = trunc(S0.f16);
// if (S0.f16 > 0.0f && S0.f16 != D.f16) then D.f16 += 1.0f.
void
Inst_VOP3__V_CEIL_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_TRUNC_F16 class methods ---

Inst_VOP3__V_TRUNC_F16::Inst_VOP3__V_TRUNC_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_trunc_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_TRUNC_F16

Inst_VOP3__V_TRUNC_F16::~Inst_VOP3__V_TRUNC_F16() {} // ~Inst_VOP3__V_TRUNC_F16

// --- description from .arch file ---
// D.f16 = trunc(S0.f16).
// Round-to-zero semantics.
void
Inst_VOP3__V_TRUNC_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_RNDNE_F16 class methods ---

Inst_VOP3__V_RNDNE_F16::Inst_VOP3__V_RNDNE_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_rndne_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_RNDNE_F16

Inst_VOP3__V_RNDNE_F16::~Inst_VOP3__V_RNDNE_F16() {} // ~Inst_VOP3__V_RNDNE_F16

// --- description from .arch file ---
// D.f16 = FLOOR(S0.f16 + 0.5f);
// if (floor(S0.f16) is even && fract(S0.f16) == 0.5f) then D.f16 -= 1.0f.
// Round-to-nearest-even semantics.
void
Inst_VOP3__V_RNDNE_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_FRACT_F16 class methods ---

Inst_VOP3__V_FRACT_F16::Inst_VOP3__V_FRACT_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_fract_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_FRACT_F16

Inst_VOP3__V_FRACT_F16::~Inst_VOP3__V_FRACT_F16() {} // ~Inst_VOP3__V_FRACT_F16

// --- description from .arch file ---
// D.f16 = S0.f16 + -floor(S0.f16).
void
Inst_VOP3__V_FRACT_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_SIN_F16 class methods ---

Inst_VOP3__V_SIN_F16::Inst_VOP3__V_SIN_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_sin_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_SIN_F16

Inst_VOP3__V_SIN_F16::~Inst_VOP3__V_SIN_F16() {} // ~Inst_VOP3__V_SIN_F16

// --- description from .arch file ---
// D.f16 = sin(S0.f16 * 2 * PI).
void
Inst_VOP3__V_SIN_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_COS_F16 class methods ---

Inst_VOP3__V_COS_F16::Inst_VOP3__V_COS_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cos_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_COS_F16

Inst_VOP3__V_COS_F16::~Inst_VOP3__V_COS_F16() {} // ~Inst_VOP3__V_COS_F16

// --- description from .arch file ---
// D.f16 = cos(S0.f16 * 2 * PI).
void
Inst_VOP3__V_COS_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_EXP_LEGACY_F32 class methods ---

Inst_VOP3__V_EXP_LEGACY_F32::Inst_VOP3__V_EXP_LEGACY_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_exp_legacy_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_EXP_LEGACY_F32

Inst_VOP3__V_EXP_LEGACY_F32::~Inst_VOP3__V_EXP_LEGACY_F32() {
} // ~Inst_VOP3__V_EXP_LEGACY_F32

// --- description from .arch file ---
// D.f = pow(2.0, S0.f) with legacy semantics.
void
Inst_VOP3__V_EXP_LEGACY_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src(gpuDynInst, extData.SRC0);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    if (instData.ABS & 0x1) {
        src.absModifier();
    }

    if (extData.NEG & 0x1) {
        src.negModifier();
    }

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::pow(2.0, src[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_LOG_LEGACY_F32 class methods ---

Inst_VOP3__V_LOG_LEGACY_F32::Inst_VOP3__V_LOG_LEGACY_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_log_legacy_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_LOG_LEGACY_F32

Inst_VOP3__V_LOG_LEGACY_F32::~Inst_VOP3__V_LOG_LEGACY_F32() {
} // ~Inst_VOP3__V_LOG_LEGACY_F32

// --- description from .arch file ---
// D.f = log2(S0.f). Base 2 logarithm with legacy semantics.
void
Inst_VOP3__V_LOG_LEGACY_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src(gpuDynInst, extData.SRC0);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::log2(src[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MAD_LEGACY_F32 class methods ---

Inst_VOP3__V_MAD_LEGACY_F32::Inst_VOP3__V_MAD_LEGACY_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_mad_legacy_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
    setFlag(MAD);
} // Inst_VOP3__V_MAD_LEGACY_F32

Inst_VOP3__V_MAD_LEGACY_F32::~Inst_VOP3__V_MAD_LEGACY_F32() {
} // ~Inst_VOP3__V_MAD_LEGACY_F32

// --- description from .arch file ---
// D.f = S0.f * S1.f + S2.f (DX9 rules, 0.0 * x = 0.0).
void
Inst_VOP3__V_MAD_LEGACY_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandF32 src2(gpuDynInst, extData.SRC2);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (instData.ABS & 0x2) {
        src1.absModifier();
    }

    if (instData.ABS & 0x4) {
        src2.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    if (extData.NEG & 0x4) {
        src2.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::fma(src0[lane], src1[lane], src2[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MAD_F32 class methods ---

Inst_VOP3__V_MAD_F32::Inst_VOP3__V_MAD_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_mad_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
    setFlag(MAD);
} // Inst_VOP3__V_MAD_F32

Inst_VOP3__V_MAD_F32::~Inst_VOP3__V_MAD_F32() {} // ~Inst_VOP3__V_MAD_F32

// --- description from .arch file ---
// D.f = S0.f * S1.f + S2.f.
void
Inst_VOP3__V_MAD_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandF32 src2(gpuDynInst, extData.SRC2);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (instData.ABS & 0x2) {
        src1.absModifier();
    }

    if (instData.ABS & 0x4) {
        src2.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    if (extData.NEG & 0x4) {
        src2.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::fma(src0[lane], src1[lane], src2[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MAD_I32_I24 class methods ---

Inst_VOP3__V_MAD_I32_I24::Inst_VOP3__V_MAD_I32_I24(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_mad_i32_i24", false)
{
    setFlag(ALU);
    setFlag(MAD);
} // Inst_VOP3__V_MAD_I32_I24

Inst_VOP3__V_MAD_I32_I24::~Inst_VOP3__V_MAD_I32_I24() {
} // ~Inst_VOP3__V_MAD_I32_I24

// --- description from .arch file ---
// D.i = S0.i[23:0] * S1.i[23:0] + S2.i.
void
Inst_VOP3__V_MAD_I32_I24::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandI32 src2(gpuDynInst, extData.SRC2);
    VecOperandI32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = sext<24>(bits(src0[lane], 23, 0)) *
                             sext<24>(bits(src1[lane], 23, 0)) +
                         src2[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MAD_U32_U24 class methods ---

Inst_VOP3__V_MAD_U32_U24::Inst_VOP3__V_MAD_U32_U24(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_mad_u32_u24", false)
{
    setFlag(ALU);
    setFlag(MAD);
} // Inst_VOP3__V_MAD_U32_U24

Inst_VOP3__V_MAD_U32_U24::~Inst_VOP3__V_MAD_U32_U24() {
} // ~Inst_VOP3__V_MAD_U32_U24

// --- description from .arch file ---
// D.u = S0.u[23:0] * S1.u[23:0] + S2.u.
void
Inst_VOP3__V_MAD_U32_U24::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU32 src2(gpuDynInst, extData.SRC2);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] =
                bits(src0[lane], 23, 0) * bits(src1[lane], 23, 0) + src2[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_CUBEID_F32 class methods ---

Inst_VOP3__V_CUBEID_F32::Inst_VOP3__V_CUBEID_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cubeid_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CUBEID_F32

Inst_VOP3__V_CUBEID_F32::~Inst_VOP3__V_CUBEID_F32() {
} // ~Inst_VOP3__V_CUBEID_F32

// --- description from .arch file ---
// D.f = cubemap face ID ({0.0, 1.0, ..., 5.0}). XYZ coordinate is given in
// ---  (S0.f, S1.f, S2.f).
void
Inst_VOP3__V_CUBEID_F32::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CUBESC_F32 class methods ---

Inst_VOP3__V_CUBESC_F32::Inst_VOP3__V_CUBESC_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cubesc_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CUBESC_F32

Inst_VOP3__V_CUBESC_F32::~Inst_VOP3__V_CUBESC_F32() {
} // ~Inst_VOP3__V_CUBESC_F32

// --- description from .arch file ---
// D.f = cubemap S coordinate. XYZ coordinate is given in (S0.f, S1.f,
// S2.f).
void
Inst_VOP3__V_CUBESC_F32::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CUBETC_F32 class methods ---

Inst_VOP3__V_CUBETC_F32::Inst_VOP3__V_CUBETC_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cubetc_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CUBETC_F32

Inst_VOP3__V_CUBETC_F32::~Inst_VOP3__V_CUBETC_F32() {
} // ~Inst_VOP3__V_CUBETC_F32

// --- description from .arch file ---
// D.f = cubemap T coordinate. XYZ coordinate is given in (S0.f, S1.f,
// S2.f).
void
Inst_VOP3__V_CUBETC_F32::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CUBEMA_F32 class methods ---

Inst_VOP3__V_CUBEMA_F32::Inst_VOP3__V_CUBEMA_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cubema_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CUBEMA_F32

Inst_VOP3__V_CUBEMA_F32::~Inst_VOP3__V_CUBEMA_F32() {
} // ~Inst_VOP3__V_CUBEMA_F32

// --- description from .arch file ---
// D.f = 2.0 * cubemap major axis. XYZ coordinate is given in (S0.f, S1.f,
// ---  S2.f).
void
Inst_VOP3__V_CUBEMA_F32::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_BFE_U32 class methods ---

Inst_VOP3__V_BFE_U32::Inst_VOP3__V_BFE_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_bfe_u32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_BFE_U32

Inst_VOP3__V_BFE_U32::~Inst_VOP3__V_BFE_U32() {} // ~Inst_VOP3__V_BFE_U32

// --- description from .arch file ---
// D.u = (S0.u>>S1.u[4:0]) & ((1<<S2.u[4:0])-1).
// Bitfield extract with S0 = data, S1 = field_offset, S2 = field_width.
void
Inst_VOP3__V_BFE_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU32 src2(gpuDynInst, extData.SRC2);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = (src0[lane] >> bits(src1[lane], 4, 0)) &
                         ((1 << bits(src2[lane], 4, 0)) - 1);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_BFE_I32 class methods ---

Inst_VOP3__V_BFE_I32::Inst_VOP3__V_BFE_I32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_bfe_i32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_BFE_I32

Inst_VOP3__V_BFE_I32::~Inst_VOP3__V_BFE_I32() {} // ~Inst_VOP3__V_BFE_I32

// --- description from .arch file ---
// D.i = (S0.i>>S1.u[4:0]) & ((1<<S2.u[4:0])-1).
// Bitfield extract with S0 = data, S1 = field_offset, S2 = field_width.
void
Inst_VOP3__V_BFE_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU32 src2(gpuDynInst, extData.SRC2);
    VecOperandI32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = (src0[lane] >> bits(src1[lane], 4, 0)) &
                         ((1 << bits(src2[lane], 4, 0)) - 1);

            // Above extracted a signed int of size src2 bits which needs
            // to be signed-extended. Check if the MSB of our src2-bit
            // integer is 1, and sign extend it is.
            if (vdst[lane] >> (bits(src2[lane], 4, 0) - 1)) {
                vdst[lane] |= 0xffffffff << bits(src2[lane], 4, 0);
            }
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_BFI_B32 class methods ---

Inst_VOP3__V_BFI_B32::Inst_VOP3__V_BFI_B32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_bfi_b32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_BFI_B32

Inst_VOP3__V_BFI_B32::~Inst_VOP3__V_BFI_B32() {} // ~Inst_VOP3__V_BFI_B32

// --- description from .arch file ---
// D.u = (S0.u & S1.u) | (~S0.u & S2.u); bitfield insert.
void
Inst_VOP3__V_BFI_B32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU32 src2(gpuDynInst, extData.SRC2);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] =
                (src0[lane] & src1[lane]) | (~src0[lane] & src2[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_FMA_F32 class methods ---

Inst_VOP3__V_FMA_F32::Inst_VOP3__V_FMA_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_fma_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
    setFlag(FMA);
} // Inst_VOP3__V_FMA_F32

Inst_VOP3__V_FMA_F32::~Inst_VOP3__V_FMA_F32() {} // ~Inst_VOP3__V_FMA_F32

// --- description from .arch file ---
// D.f = S0.f * S1.f + S2.f.
void
Inst_VOP3__V_FMA_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandF32 src2(gpuDynInst, extData.SRC2);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (instData.ABS & 0x2) {
        src1.absModifier();
    }

    if (instData.ABS & 0x4) {
        src2.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    if (extData.NEG & 0x4) {
        src2.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::fma(src0[lane], src1[lane], src2[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_FMA_F64 class methods ---

Inst_VOP3__V_FMA_F64::Inst_VOP3__V_FMA_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_fma_f64", false)
{
    setFlag(ALU);
    setFlag(F64);
    setFlag(FMA);
} // Inst_VOP3__V_FMA_F64

Inst_VOP3__V_FMA_F64::~Inst_VOP3__V_FMA_F64() {} // ~Inst_VOP3__V_FMA_F64

// --- description from .arch file ---
// D.d = S0.d * S1.d + S2.d.
void
Inst_VOP3__V_FMA_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandF64 src2(gpuDynInst, extData.SRC2);
    VecOperandF64 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (instData.ABS & 0x2) {
        src1.absModifier();
    }

    if (instData.ABS & 0x4) {
        src2.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    if (extData.NEG & 0x4) {
        src2.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::fma(src0[lane], src1[lane], src2[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_LERP_U8 class methods ---

Inst_VOP3__V_LERP_U8::Inst_VOP3__V_LERP_U8(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_lerp_u8", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_LERP_U8

Inst_VOP3__V_LERP_U8::~Inst_VOP3__V_LERP_U8() {} // ~Inst_VOP3__V_LERP_U8

// --- description from .arch file ---
// D.u = ((S0.u[31:24] + S1.u[31:24] + S2.u[24]) >> 1) << 24
// D.u += ((S0.u[23:16] + S1.u[23:16] + S2.u[16]) >> 1) << 16;
// D.u += ((S0.u[15:8] + S1.u[15:8] + S2.u[8]) >> 1) << 8;
// D.u += ((S0.u[7:0] + S1.u[7:0] + S2.u[0]) >> 1).
// Unsigned 8-bit pixel average on packed unsigned bytes (linear
// ---  interpolation). S2 acts as a round mode; if set, 0.5 rounds up,
// ---  otherwise 0.5 truncates.
void
Inst_VOP3__V_LERP_U8::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU32 src2(gpuDynInst, extData.SRC2);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = ((bits(src0[lane], 31, 24) +
                           bits(src1[lane], 31, 24) + bits(src2[lane], 24)) >>
                          1)
                         << 24;
            vdst[lane] += ((bits(src0[lane], 23, 16) +
                            bits(src1[lane], 23, 16) + bits(src2[lane], 16)) >>
                           1)
                          << 16;
            vdst[lane] += ((bits(src0[lane], 15, 8) + bits(src1[lane], 15, 8) +
                            bits(src2[lane], 8)) >>
                           1)
                          << 8;
            vdst[lane] += ((bits(src0[lane], 7, 0) + bits(src1[lane], 7, 0) +
                            bits(src2[lane], 0)) >>
                           1);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_ALIGNBIT_B32 class methods ---

Inst_VOP3__V_ALIGNBIT_B32::Inst_VOP3__V_ALIGNBIT_B32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_alignbit_b32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_ALIGNBIT_B32

Inst_VOP3__V_ALIGNBIT_B32::~Inst_VOP3__V_ALIGNBIT_B32() {
} // ~Inst_VOP3__V_ALIGNBIT_B32

// --- description from .arch file ---
// D.u = ({S0,S1} >> S2.u[4:0]) & 0xffffffff.
void
Inst_VOP3__V_ALIGNBIT_B32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU32 src2(gpuDynInst, extData.SRC2);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            VecElemU64 src_0_1 =
                (((VecElemU64)src0[lane] << 32) | (VecElemU64)src1[lane]);
            vdst[lane] =
                (VecElemU32)((src_0_1 >> (VecElemU64)bits(src2[lane], 4, 0)) &
                             0xffffffff);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_ALIGNBYTE_B32 class methods ---

Inst_VOP3__V_ALIGNBYTE_B32::Inst_VOP3__V_ALIGNBYTE_B32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_alignbyte_b32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_ALIGNBYTE_B32

Inst_VOP3__V_ALIGNBYTE_B32::~Inst_VOP3__V_ALIGNBYTE_B32() {
} // ~Inst_VOP3__V_ALIGNBYTE_B32

// --- description from .arch file ---
// D.u = ({S0,S1} >> (8*S2.u[4:0])) & 0xffffffff.
void
Inst_VOP3__V_ALIGNBYTE_B32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU32 src2(gpuDynInst, extData.SRC2);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            VecElemU64 src_0_1 =
                (((VecElemU64)src0[lane] << 32) | (VecElemU64)src1[lane]);
            vdst[lane] =
                (VecElemU32)((src_0_1 >>
                              (8ULL * (VecElemU64)bits(src2[lane], 4, 0))) &
                             0xffffffff);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MIN3_F32 class methods ---

Inst_VOP3__V_MIN3_F32::Inst_VOP3__V_MIN3_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_min3_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_MIN3_F32

Inst_VOP3__V_MIN3_F32::~Inst_VOP3__V_MIN3_F32() {} // ~Inst_VOP3__V_MIN3_F32

// --- description from .arch file ---
// D.f = min(S0.f, S1.f, S2.f).
void
Inst_VOP3__V_MIN3_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandF32 src2(gpuDynInst, extData.SRC2);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (instData.ABS & 0x2) {
        src1.absModifier();
    }

    if (instData.ABS & 0x4) {
        src2.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    if (extData.NEG & 0x4) {
        src2.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            VecElemF32 min_0_1 = std::fmin(src0[lane], src1[lane]);
            vdst[lane] = std::fmin(min_0_1, src2[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MIN3_I32 class methods ---

Inst_VOP3__V_MIN3_I32::Inst_VOP3__V_MIN3_I32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_min3_i32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_MIN3_I32

Inst_VOP3__V_MIN3_I32::~Inst_VOP3__V_MIN3_I32() {} // ~Inst_VOP3__V_MIN3_I32

// --- description from .arch file ---
// D.i = min(S0.i, S1.i, S2.i).
void
Inst_VOP3__V_MIN3_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandI32 src2(gpuDynInst, extData.SRC2);
    VecOperandI32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            VecElemI32 min_0_1 = std::min(src0[lane], src1[lane]);
            vdst[lane] = std::min(min_0_1, src2[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MIN3_U32 class methods ---

Inst_VOP3__V_MIN3_U32::Inst_VOP3__V_MIN3_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_min3_u32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_MIN3_U32

Inst_VOP3__V_MIN3_U32::~Inst_VOP3__V_MIN3_U32() {} // ~Inst_VOP3__V_MIN3_U32

// --- description from .arch file ---
// D.u = min(S0.u, S1.u, S2.u).
void
Inst_VOP3__V_MIN3_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU32 src2(gpuDynInst, extData.SRC2);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            VecElemU32 min_0_1 = std::min(src0[lane], src1[lane]);
            vdst[lane] = std::min(min_0_1, src2[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MAX3_F32 class methods ---

Inst_VOP3__V_MAX3_F32::Inst_VOP3__V_MAX3_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_max3_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_MAX3_F32

Inst_VOP3__V_MAX3_F32::~Inst_VOP3__V_MAX3_F32() {} // ~Inst_VOP3__V_MAX3_F32

// --- description from .arch file ---
// D.f = max(S0.f, S1.f, S2.f).
void
Inst_VOP3__V_MAX3_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandF32 src2(gpuDynInst, extData.SRC2);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (instData.ABS & 0x2) {
        src1.absModifier();
    }

    if (instData.ABS & 0x4) {
        src2.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    if (extData.NEG & 0x4) {
        src2.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            VecElemF32 max_0_1 = std::fmax(src0[lane], src1[lane]);
            vdst[lane] = std::fmax(max_0_1, src2[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MAX3_I32 class methods ---

Inst_VOP3__V_MAX3_I32::Inst_VOP3__V_MAX3_I32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_max3_i32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_MAX3_I32

Inst_VOP3__V_MAX3_I32::~Inst_VOP3__V_MAX3_I32() {} // ~Inst_VOP3__V_MAX3_I32

// --- description from .arch file ---
// D.i = max(S0.i, S1.i, S2.i).
void
Inst_VOP3__V_MAX3_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandI32 src2(gpuDynInst, extData.SRC2);
    VecOperandI32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            VecElemI32 max_0_1 = std::max(src0[lane], src1[lane]);
            vdst[lane] = std::max(max_0_1, src2[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MAX3_U32 class methods ---

Inst_VOP3__V_MAX3_U32::Inst_VOP3__V_MAX3_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_max3_u32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_MAX3_U32

Inst_VOP3__V_MAX3_U32::~Inst_VOP3__V_MAX3_U32() {} // ~Inst_VOP3__V_MAX3_U32

// --- description from .arch file ---
// D.u = max(S0.u, S1.u, S2.u).
void
Inst_VOP3__V_MAX3_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU32 src2(gpuDynInst, extData.SRC2);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            VecElemU32 max_0_1 = std::max(src0[lane], src1[lane]);
            vdst[lane] = std::max(max_0_1, src2[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MED3_F32 class methods ---

Inst_VOP3__V_MED3_F32::Inst_VOP3__V_MED3_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_med3_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_MED3_F32

Inst_VOP3__V_MED3_F32::~Inst_VOP3__V_MED3_F32() {} // ~Inst_VOP3__V_MED3_F32

// --- description from .arch file ---
// D.f = median(S0.f, S1.f, S2.f).
void
Inst_VOP3__V_MED3_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandF32 src2(gpuDynInst, extData.SRC2);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (instData.ABS & 0x2) {
        src1.absModifier();
    }

    if (instData.ABS & 0x4) {
        src2.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    if (extData.NEG & 0x4) {
        src2.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = median(src0[lane], src1[lane], src2[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MED3_I32 class methods ---

Inst_VOP3__V_MED3_I32::Inst_VOP3__V_MED3_I32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_med3_i32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_MED3_I32

Inst_VOP3__V_MED3_I32::~Inst_VOP3__V_MED3_I32() {} // ~Inst_VOP3__V_MED3_I32

// --- description from .arch file ---
// D.i = median(S0.i, S1.i, S2.i).
void
Inst_VOP3__V_MED3_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandI32 src2(gpuDynInst, extData.SRC2);
    VecOperandI32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = median(src0[lane], src1[lane], src2[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MED3_U32 class methods ---

Inst_VOP3__V_MED3_U32::Inst_VOP3__V_MED3_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_med3_u32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_MED3_U32

Inst_VOP3__V_MED3_U32::~Inst_VOP3__V_MED3_U32() {} // ~Inst_VOP3__V_MED3_U32

// --- description from .arch file ---
// D.u = median(S0.u, S1.u, S2.u).
void
Inst_VOP3__V_MED3_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU32 src2(gpuDynInst, extData.SRC2);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = median(src0[lane], src1[lane], src2[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_SAD_U8 class methods ---

Inst_VOP3__V_SAD_U8::Inst_VOP3__V_SAD_U8(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_sad_u8", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_SAD_U8

Inst_VOP3__V_SAD_U8::~Inst_VOP3__V_SAD_U8() {} // ~Inst_VOP3__V_SAD_U8

// --- description from .arch file ---
// D.u = abs(S0.i[31:24] - S1.i[31:24]) + abs(S0.i[23:16] - S1.i[23:16]) +
// abs(S0.i[15:8] - S1.i[15:8]) + abs(S0.i[7:0] - S1.i[7:0]) + S2.u.
// Sum of absolute differences with accumulation, overflow into upper bits
// is allowed.
void
Inst_VOP3__V_SAD_U8::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU32 src2(gpuDynInst, extData.SRC2);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] =
                std::abs(bits(src0[lane], 31, 24) - bits(src1[lane], 31, 24)) +
                std::abs(bits(src0[lane], 23, 16) - bits(src1[lane], 23, 16)) +
                std::abs(bits(src0[lane], 15, 8) - bits(src1[lane], 15, 8)) +
                std::abs(bits(src0[lane], 7, 0) - bits(src1[lane], 7, 0)) +
                src2[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_SAD_HI_U8 class methods ---

Inst_VOP3__V_SAD_HI_U8::Inst_VOP3__V_SAD_HI_U8(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_sad_hi_u8", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_SAD_HI_U8

Inst_VOP3__V_SAD_HI_U8::~Inst_VOP3__V_SAD_HI_U8() {} // ~Inst_VOP3__V_SAD_HI_U8

// --- description from .arch file ---
// D.u = (SAD_U8(S0, S1, 0) << 16) + S2.u.
// Sum of absolute differences with accumulation, overflow is lost.
void
Inst_VOP3__V_SAD_HI_U8::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU32 src2(gpuDynInst, extData.SRC2);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] =
                (((bits(src0[lane], 31, 24) - bits(src1[lane], 31, 24)) +
                  (bits(src0[lane], 23, 16) - bits(src1[lane], 23, 16)) +
                  (bits(src0[lane], 15, 8) - bits(src1[lane], 15, 8)) +
                  (bits(src0[lane], 7, 0) - bits(src1[lane], 7, 0)))
                 << 16) +
                src2[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_SAD_U16 class methods ---

Inst_VOP3__V_SAD_U16::Inst_VOP3__V_SAD_U16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_sad_u16", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_SAD_U16

Inst_VOP3__V_SAD_U16::~Inst_VOP3__V_SAD_U16() {} // ~Inst_VOP3__V_SAD_U16

// --- description from .arch file ---
// D.u = abs(S0.i[31:16] - S1.i[31:16]) + abs(S0.i[15:0] - S1.i[15:0])
// + S2.u.
// Word SAD with accumulation.
void
Inst_VOP3__V_SAD_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU32 src2(gpuDynInst, extData.SRC2);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] =
                std::abs(bits(src0[lane], 31, 16) - bits(src1[lane], 31, 16)) +
                std::abs(bits(src0[lane], 15, 0) - bits(src1[lane], 15, 0)) +
                src2[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_SAD_U32 class methods ---

Inst_VOP3__V_SAD_U32::Inst_VOP3__V_SAD_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_sad_u32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_SAD_U32

Inst_VOP3__V_SAD_U32::~Inst_VOP3__V_SAD_U32() {} // ~Inst_VOP3__V_SAD_U32

// --- description from .arch file ---
// D.u = abs(S0.i - S1.i) + S2.u.
// Dword SAD with accumulation.
void
Inst_VOP3__V_SAD_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU32 src2(gpuDynInst, extData.SRC2);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::abs(src0[lane] - src1[lane]) + src2[lane];
        } // if
    }     // for

    vdst.write();
} // execute

// --- Inst_VOP3__V_CVT_PK_U8_F32 class methods ---

Inst_VOP3__V_CVT_PK_U8_F32::Inst_VOP3__V_CVT_PK_U8_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_pk_u8_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CVT_PK_U8_F32

Inst_VOP3__V_CVT_PK_U8_F32::~Inst_VOP3__V_CVT_PK_U8_F32() {
} // ~Inst_VOP3__V_CVT_PK_U8_F32

// --- description from .arch file ---
// D.u = ((flt32_to_uint8(S0.f) & 0xff) << (8 * S1.u[1:0]))
// | (S2.u & ~(0xff << (8 * S1.u[1:0]))).
// Convert floating point value S0 to 8-bit unsigned integer and pack the
// result into byte S1 of dword S2.
void
Inst_VOP3__V_CVT_PK_U8_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU32 src2(gpuDynInst, extData.SRC2);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] =
                (((VecElemU8)src0[lane] & 0xff)
                 << (8 * bits(src1[lane], 1, 0))) |
                (src2[lane] & ~(0xff << (8 * bits(src1[lane], 1, 0))));
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_DIV_FIXUP_F32 class methods ---

Inst_VOP3__V_DIV_FIXUP_F32::Inst_VOP3__V_DIV_FIXUP_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_div_fixup_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_DIV_FIXUP_F32

Inst_VOP3__V_DIV_FIXUP_F32::~Inst_VOP3__V_DIV_FIXUP_F32() {
} // ~Inst_VOP3__V_DIV_FIXUP_F32

// --- description from .arch file ---
// D.f = Divide fixup and flags -- s0.f = Quotient, s1.f = Denominator,
// s2.f = Numerator. This opcode generates exceptions resulting from the
// division operation.
void
Inst_VOP3__V_DIV_FIXUP_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandF32 src2(gpuDynInst, extData.SRC2);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (instData.ABS & 0x2) {
        src1.absModifier();
    }

    if (instData.ABS & 0x4) {
        src2.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    if (extData.NEG & 0x4) {
        src2.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            if (std::fpclassify(src1[lane]) == FP_ZERO) {
                if (std::signbit(src1[lane])) {
                    vdst[lane] = -INFINITY;
                } else {
                    vdst[lane] = +INFINITY;
                }
            } else if (std::isnan(src2[lane]) || std::isnan(src1[lane])) {
                vdst[lane] = NAN;
            } else if (std::isinf(src1[lane])) {
                if (std::signbit(src1[lane])) {
                    vdst[lane] = -INFINITY;
                } else {
                    vdst[lane] = +INFINITY;
                }
            } else {
                vdst[lane] = src2[lane] / src1[lane];
            }
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_DIV_FIXUP_F64 class methods ---

Inst_VOP3__V_DIV_FIXUP_F64::Inst_VOP3__V_DIV_FIXUP_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_div_fixup_f64", false)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_DIV_FIXUP_F64

Inst_VOP3__V_DIV_FIXUP_F64::~Inst_VOP3__V_DIV_FIXUP_F64() {
} // ~Inst_VOP3__V_DIV_FIXUP_F64

// --- description from .arch file ---
// D.d = Divide fixup and flags -- s0.d = Quotient, s1.d = Denominator,
// s2.d = Numerator. This opcode generates exceptions resulting from the
// division operation.
void
Inst_VOP3__V_DIV_FIXUP_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandF64 src2(gpuDynInst, extData.SRC2);
    VecOperandF64 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (instData.ABS & 0x2) {
        src1.absModifier();
    }

    if (instData.ABS & 0x4) {
        src2.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    if (extData.NEG & 0x4) {
        src2.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            int sign_out = std::signbit(src1[lane]) ^ std::signbit(src2[lane]);
            int exp1(0);
            int exp2(0);
            std::frexp(src1[lane], &exp1);
            std::frexp(src2[lane], &exp2);

            if (std::isnan(src1[lane]) || std::isnan(src2[lane])) {
                vdst[lane] = std::numeric_limits<VecElemF64>::quiet_NaN();
            } else if (std::fpclassify(src1[lane]) == FP_ZERO &&
                       std::fpclassify(src2[lane]) == FP_ZERO) {
                vdst[lane] = std::numeric_limits<VecElemF64>::signaling_NaN();
            } else if (std::isinf(src1[lane]) && std::isinf(src2[lane])) {
                vdst[lane] = std::numeric_limits<VecElemF64>::signaling_NaN();
            } else if (std::fpclassify(src1[lane]) == FP_ZERO ||
                       std::isinf(src2[lane])) {
                vdst[lane] = sign_out ? -INFINITY : +INFINITY;
            } else if (std::isinf(src1[lane]) ||
                       std::fpclassify(src2[lane]) == FP_ZERO) {
                vdst[lane] = sign_out ? -0.0 : +0.0;
            } else if (exp2 - exp1 < -1075) {
                vdst[lane] = src0[lane];
            } else if (exp1 == 2047) {
                vdst[lane] = src0[lane];
            } else {
                vdst[lane] =
                    sign_out ? -std::fabs(src0[lane]) : std::fabs(src0[lane]);
            }
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_DIV_SCALE_F32 class methods ---

Inst_VOP3__V_DIV_SCALE_F32::Inst_VOP3__V_DIV_SCALE_F32(InFmt_VOP3B *iFmt)
    : Inst_VOP3B(iFmt, "v_div_scale_f32")
{
    setFlag(ALU);
    setFlag(WritesVCC);
    setFlag(F32);
} // Inst_VOP3__V_DIV_SCALE_F32

Inst_VOP3__V_DIV_SCALE_F32::~Inst_VOP3__V_DIV_SCALE_F32() {
} // ~Inst_VOP3__V_DIV_SCALE_F32

// --- description from .arch file ---
// {vcc,D.f} = Divide preop and flags -- s0.f = Quotient, s1.f =
// Denominator, s2.f = Numerator -- s0 must equal s1 or s2. Given a
// numerator and denominator, this opcode will appropriately scale inputs
// for division to avoid subnormal terms during Newton-Raphson correction
// algorithm. This opcode producses a VCC flag for post-scale of quotient.
void
Inst_VOP3__V_DIV_SCALE_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandF32 src2(gpuDynInst, extData.SRC2);
    ScalarOperandU64 vcc(gpuDynInst, instData.SDST);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    if (extData.NEG & 0x4) {
        src2.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src0[lane];
            vcc.setBit(lane, 0);
        }
    }

    vcc.write();
    vdst.write();
} // execute

// --- Inst_VOP3__V_DIV_SCALE_F64 class methods ---

Inst_VOP3__V_DIV_SCALE_F64::Inst_VOP3__V_DIV_SCALE_F64(InFmt_VOP3B *iFmt)
    : Inst_VOP3B(iFmt, "v_div_scale_f64")
{
    setFlag(ALU);
    setFlag(WritesVCC);
    setFlag(F64);
} // Inst_VOP3__V_DIV_SCALE_F64

Inst_VOP3__V_DIV_SCALE_F64::~Inst_VOP3__V_DIV_SCALE_F64() {
} // ~Inst_VOP3__V_DIV_SCALE_F64

// --- description from .arch file ---
// {vcc,D.d} = Divide preop and flags -- s0.d = Quotient, s1.d =
// Denominator, s2.d = Numerator -- s0 must equal s1 or s2. Given a
// numerator and denominator, this opcode will appropriately scale inputs
// for division to avoid subnormal terms during Newton-Raphson correction
// algorithm. This opcode producses a VCC flag for post-scale of quotient.
void
Inst_VOP3__V_DIV_SCALE_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandF64 src2(gpuDynInst, extData.SRC2);
    ScalarOperandU64 vcc(gpuDynInst, instData.SDST);
    VecOperandF64 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    if (extData.NEG & 0x4) {
        src2.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            int exp1(0);
            int exp2(0);
            std::frexp(src1[lane], &exp1);
            std::frexp(src2[lane], &exp2);
            vcc.setBit(lane, 0);

            if (std::fpclassify(src1[lane]) == FP_ZERO ||
                std::fpclassify(src2[lane]) == FP_ZERO) {
                vdst[lane] = NAN;
            } else if (exp2 - exp1 >= 768) {
                vcc.setBit(lane, 1);
                if (src0[lane] == src1[lane]) {
                    vdst[lane] = std::ldexp(src0[lane], 128);
                }
            } else if (std::fpclassify(src1[lane]) == FP_SUBNORMAL) {
                vdst[lane] = std::ldexp(src0[lane], 128);
            } else if (std::fpclassify(1.0 / src1[lane]) == FP_SUBNORMAL &&
                       std::fpclassify(src2[lane] / src1[lane]) ==
                           FP_SUBNORMAL) {
                vcc.setBit(lane, 1);
                if (src0[lane] == src1[lane]) {
                    vdst[lane] = std::ldexp(src0[lane], 128);
                }
            } else if (std::fpclassify(1.0 / src1[lane]) == FP_SUBNORMAL) {
                vdst[lane] = std::ldexp(src0[lane], -128);
            } else if (std::fpclassify(src2[lane] / src1[lane]) ==
                       FP_SUBNORMAL) {
                vcc.setBit(lane, 1);
                if (src0[lane] == src2[lane]) {
                    vdst[lane] = std::ldexp(src0[lane], 128);
                }
            } else if (exp2 <= 53) {
                vdst[lane] = std::ldexp(src0[lane], 128);
            }
        }
    }

    vcc.write();
    vdst.write();
} // execute

// --- Inst_VOP3__V_DIV_FMAS_F32 class methods ---

Inst_VOP3__V_DIV_FMAS_F32::Inst_VOP3__V_DIV_FMAS_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_div_fmas_f32", false)
{
    setFlag(ALU);
    setFlag(ReadsVCC);
    setFlag(F32);
    setFlag(FMA);
} // Inst_VOP3__V_DIV_FMAS_F32

Inst_VOP3__V_DIV_FMAS_F32::~Inst_VOP3__V_DIV_FMAS_F32() {
} // ~Inst_VOP3__V_DIV_FMAS_F32

// --- description from .arch file ---
// D.f = Special case divide FMA with scale and flags(s0.f = Quotient,
// s1.f = Denominator, s2.f = Numerator)
void
Inst_VOP3__V_DIV_FMAS_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandF32 src2(gpuDynInst, extData.SRC2);
    VecOperandF64 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (instData.ABS & 0x2) {
        src1.absModifier();
    }

    if (instData.ABS & 0x4) {
        src2.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    if (extData.NEG & 0x4) {
        src2.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::fma(src0[lane], src1[lane], src2[lane]);
        }
    }

    // vdst.write();
} // execute

// --- Inst_VOP3__V_DIV_FMAS_F64 class methods ---

Inst_VOP3__V_DIV_FMAS_F64::Inst_VOP3__V_DIV_FMAS_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_div_fmas_f64", false)
{
    setFlag(ALU);
    setFlag(ReadsVCC);
    setFlag(F64);
    setFlag(FMA);
} // Inst_VOP3__V_DIV_FMAS_F64

Inst_VOP3__V_DIV_FMAS_F64::~Inst_VOP3__V_DIV_FMAS_F64() {
} // ~Inst_VOP3__V_DIV_FMAS_F64

// --- description from .arch file ---
// D.d = Special case divide FMA with scale and flags(s0.d = Quotient,
// s1.d = Denominator, s2.d = Numerator)
void
Inst_VOP3__V_DIV_FMAS_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandF64 src2(gpuDynInst, extData.SRC2);
    VecOperandF64 vdst(gpuDynInst, instData.VDST);
    ConstScalarOperandU64 vcc(gpuDynInst, REG_VCC_LO);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();
    vcc.read();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (instData.ABS & 0x2) {
        src1.absModifier();
    }

    if (instData.ABS & 0x4) {
        src2.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    if (extData.NEG & 0x4) {
        src2.negModifier();
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            if (bits(vcc.rawData(), lane)) {
                vdst[lane] = std::pow(2, 64) *
                             std::fma(src0[lane], src1[lane], src2[lane]);
            } else {
                vdst[lane] = std::fma(src0[lane], src1[lane], src2[lane]);
            }
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MSAD_U8 class methods ---

Inst_VOP3__V_MSAD_U8::Inst_VOP3__V_MSAD_U8(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_msad_u8", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_MSAD_U8

Inst_VOP3__V_MSAD_U8::~Inst_VOP3__V_MSAD_U8() {} // ~Inst_VOP3__V_MSAD_U8

// --- description from .arch file ---
// D.u = Masked Byte SAD with accum_lo(S0.u, S1.u, S2.u).
void
Inst_VOP3__V_MSAD_U8::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_QSAD_PK_U16_U8 class methods ---

Inst_VOP3__V_QSAD_PK_U16_U8::Inst_VOP3__V_QSAD_PK_U16_U8(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_qsad_pk_u16_u8", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_QSAD_PK_U16_U8

Inst_VOP3__V_QSAD_PK_U16_U8::~Inst_VOP3__V_QSAD_PK_U16_U8() {
} // ~Inst_VOP3__V_QSAD_PK_U16_U8

// --- description from .arch file ---
// D.u = Quad-Byte SAD with 16-bit packed accum_lo/hi(S0.u[63:0],
// S1.u[31:0], S2.u[63:0])
void
Inst_VOP3__V_QSAD_PK_U16_U8::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_MQSAD_PK_U16_U8 class methods ---

Inst_VOP3__V_MQSAD_PK_U16_U8::Inst_VOP3__V_MQSAD_PK_U16_U8(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_mqsad_pk_u16_u8", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_MQSAD_PK_U16_U8

Inst_VOP3__V_MQSAD_PK_U16_U8::~Inst_VOP3__V_MQSAD_PK_U16_U8() {
} // ~Inst_VOP3__V_MQSAD_PK_U16_U8

// --- description from .arch file ---
// D.u = Masked Quad-Byte SAD with 16-bit packed accum_lo/hi(S0.u[63:0],
// ---  S1.u[31:0], S2.u[63:0])
void
Inst_VOP3__V_MQSAD_PK_U16_U8::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_MQSAD_U32_U8 class methods ---

Inst_VOP3__V_MQSAD_U32_U8::Inst_VOP3__V_MQSAD_U32_U8(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_mqsad_u32_u8", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_MQSAD_U32_U8

Inst_VOP3__V_MQSAD_U32_U8::~Inst_VOP3__V_MQSAD_U32_U8() {
} // ~Inst_VOP3__V_MQSAD_U32_U8

// --- description from .arch file ---
// D.u128 = Masked Quad-Byte SAD with 32-bit accum_lo/hi(S0.u[63:0],
// ---  S1.u[31:0], S2.u[127:0])
void
Inst_VOP3__V_MQSAD_U32_U8::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_MAD_U64_U32 class methods ---

Inst_VOP3__V_MAD_U64_U32::Inst_VOP3__V_MAD_U64_U32(InFmt_VOP3B *iFmt)
    : Inst_VOP3B(iFmt, "v_mad_u64_u32")
{
    setFlag(ALU);
    setFlag(WritesVCC);
    setFlag(MAD);
} // Inst_VOP3__V_MAD_U64_U32

Inst_VOP3__V_MAD_U64_U32::~Inst_VOP3__V_MAD_U64_U32() {
} // ~Inst_VOP3__V_MAD_U64_U32

// --- description from .arch file ---
// {vcc_out,D.u64} = S0.u32 * S1.u32 + S2.u64.
void
Inst_VOP3__V_MAD_U64_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU64 src2(gpuDynInst, extData.SRC2);
    ScalarOperandU64 vcc(gpuDynInst, instData.SDST);
    VecOperandU64 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();
    vdst.read();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vcc.setBit(lane,
                       muladd(vdst[lane], src0[lane], src1[lane], src2[lane]));
        }
    }

    vcc.write();
    vdst.write();
} // execute

// --- Inst_VOP3__V_MAD_I64_I32 class methods ---

Inst_VOP3__V_MAD_I64_I32::Inst_VOP3__V_MAD_I64_I32(InFmt_VOP3B *iFmt)
    : Inst_VOP3B(iFmt, "v_mad_i64_i32")
{
    setFlag(ALU);
    setFlag(WritesVCC);
    setFlag(MAD);
} // Inst_VOP3__V_MAD_I64_I32

Inst_VOP3__V_MAD_I64_I32::~Inst_VOP3__V_MAD_I64_I32() {
} // ~Inst_VOP3__V_MAD_I64_I32

// --- description from .arch file ---
// {vcc_out,D.i64} = S0.i32 * S1.i32 + S2.i64.
void
Inst_VOP3__V_MAD_I64_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandI64 src2(gpuDynInst, extData.SRC2);
    ScalarOperandU64 vcc(gpuDynInst, instData.SDST);
    VecOperandI64 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vcc.setBit(lane,
                       muladd(vdst[lane], src0[lane], src1[lane], src2[lane]));
        }
    }

    vcc.write();
    vdst.write();
} // execute

// --- Inst_VOP3__V_XAD_U32 class methods ---

Inst_VOP3__V_XAD_U32::Inst_VOP3__V_XAD_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_xad_u32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_XAD_U32

Inst_VOP3__V_XAD_U32::~Inst_VOP3__V_XAD_U32() {} // ~Inst_VOP3__V_XAD_U32

// --- description from .arch file ---
// D.u32 = (S0.u32 ^ S1.u32) + S2.u32.
void
Inst_VOP3__V_XAD_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU32 src2(gpuDynInst, extData.SRC2);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = (src0[lane] ^ src1[lane]) + src2[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_LSHL_ADD_U32 class methods ---

Inst_VOP3__V_LSHL_ADD_U32::Inst_VOP3__V_LSHL_ADD_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_lshl_add_u32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_LSHL_ADD_U32

Inst_VOP3__V_LSHL_ADD_U32::~Inst_VOP3__V_LSHL_ADD_U32() {
} // ~Inst_VOP3__V_LSHL_ADD_U32

// --- description from .arch file ---
// D.u = (S0.u << S1.u[4:0]) + S2.u.
void
Inst_VOP3__V_LSHL_ADD_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU32 src2(gpuDynInst, extData.SRC2);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = (src0[lane] << bits(src1[lane], 4, 0)) + src2[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_ADD_LSHL_U32 class methods ---

Inst_VOP3__V_ADD_LSHL_U32::Inst_VOP3__V_ADD_LSHL_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_add_lshl_u32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_ADD_LSHL_U32

Inst_VOP3__V_ADD_LSHL_U32::~Inst_VOP3__V_ADD_LSHL_U32() {
} // ~Inst_VOP3__V_ADD_LSHL_U32

// --- description from .arch file ---
// D.u = (S0.u + S1.u) << S2.u[4:0].
void
Inst_VOP3__V_ADD_LSHL_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU32 src2(gpuDynInst, extData.SRC2);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = (src0[lane] + src1[lane]) << bits(src2[lane], 4, 0);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_ADD3_U32 class methods ---

Inst_VOP3__V_ADD3_U32::Inst_VOP3__V_ADD3_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_add3_u32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_ADD3_U32

Inst_VOP3__V_ADD3_U32::~Inst_VOP3__V_ADD3_U32() {} // ~Inst_VOP3__V_ADD3_U32

// --- description from .arch file ---
// D.u = S0.u + S1.u + S2.u.
void
Inst_VOP3__V_ADD3_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU32 src2(gpuDynInst, extData.SRC2);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src0[lane] + src1[lane] + src2[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_LSHL_OR_B32 class methods ---

Inst_VOP3__V_LSHL_OR_B32::Inst_VOP3__V_LSHL_OR_B32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_lshl_or_b32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_LSHL_OR_B32

Inst_VOP3__V_LSHL_OR_B32::~Inst_VOP3__V_LSHL_OR_B32() {
} // ~Inst_VOP3__V_LSHL_OR_B32

// --- description from .arch file ---
// D.u = (S0.u << S1.u[4:0]) | S2.u.
void
Inst_VOP3__V_LSHL_OR_B32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU32 src2(gpuDynInst, extData.SRC2);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = (src0[lane] << bits(src1[lane], 4, 0)) | src2[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_AND_OR_B32 class methods ---

Inst_VOP3__V_AND_OR_B32::Inst_VOP3__V_AND_OR_B32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_and_or_b32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_AND_OR_B32

Inst_VOP3__V_AND_OR_B32::~Inst_VOP3__V_AND_OR_B32() {
} // ~Inst_VOP3__V_AND_OR_B32

// --- description from .arch file ---
// D.u = (S0.u & S1.u) | S2.u.
// Input and output modifiers not supported.
void
Inst_VOP3__V_AND_OR_B32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU32 src2(gpuDynInst, extData.SRC2);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = (src0[lane] & src1[lane]) | src2[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MAD_F16 class methods ---

Inst_VOP3__V_MAD_F16::Inst_VOP3__V_MAD_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_mad_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
    setFlag(MAD);
} // Inst_VOP3__V_MAD_F16

Inst_VOP3__V_MAD_F16::~Inst_VOP3__V_MAD_F16() {} // ~Inst_VOP3__V_MAD_F16

// --- description from .arch file ---
// D.f16 = S0.f16 * S1.f16 + S2.f16.
// Supports round mode, exception flags, saturation.
void
Inst_VOP3__V_MAD_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_MAD_U16 class methods ---

Inst_VOP3__V_MAD_U16::Inst_VOP3__V_MAD_U16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_mad_u16", false)
{
    setFlag(ALU);
    setFlag(MAD);
} // Inst_VOP3__V_MAD_U16

Inst_VOP3__V_MAD_U16::~Inst_VOP3__V_MAD_U16() {} // ~Inst_VOP3__V_MAD_U16

// --- description from .arch file ---
// D.u16 = S0.u16 * S1.u16 + S2.u16.
// Supports saturation (unsigned 16-bit integer domain).
void
Inst_VOP3__V_MAD_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU16 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU16 src2(gpuDynInst, extData.SRC2);
    VecOperandU16 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src0[lane] * src1[lane] + src2[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MAD_I16 class methods ---

Inst_VOP3__V_MAD_I16::Inst_VOP3__V_MAD_I16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_mad_i16", false)
{
    setFlag(ALU);
    setFlag(MAD);
} // Inst_VOP3__V_MAD_I16

Inst_VOP3__V_MAD_I16::~Inst_VOP3__V_MAD_I16() {} // ~Inst_VOP3__V_MAD_I16

// --- description from .arch file ---
// D.i16 = S0.i16 * S1.i16 + S2.i16.
// Supports saturation (signed 16-bit integer domain).
void
Inst_VOP3__V_MAD_I16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI16 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandI16 src2(gpuDynInst, extData.SRC2);
    VecOperandI16 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src0[lane] * src1[lane] + src2[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_PERM_B32 class methods ---

Inst_VOP3__V_PERM_B32::Inst_VOP3__V_PERM_B32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_perm_b32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_PERM_B32

Inst_VOP3__V_PERM_B32::~Inst_VOP3__V_PERM_B32() {} // ~Inst_VOP3__V_PERM_B32

// --- description from .arch file ---
// D.u[31:24] = permute({S0.u, S1.u}, S2.u[31:24]);
// D.u[23:16] = permute({S0.u, S1.u}, S2.u[23:16]);
// D.u[15:8] = permute({S0.u, S1.u}, S2.u[15:8]);
// D.u[7:0] = permute({S0.u, S1.u}, S2.u[7:0]);
// byte permute(byte in[8], byte sel) {
//     if (sel>=13) then return 0xff;
//     elsif(sel==12) then return 0x00;
//     elsif(sel==11) then return in[7][7] * 0xff;
//     elsif(sel==10) then return in[5][7] * 0xff;
//     elsif(sel==9) then return in[3][7] * 0xff;
//     elsif(sel==8) then return in[1][7] * 0xff;
//     else return in[sel];
//     }
// Byte permute.
void
Inst_VOP3__V_PERM_B32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU32 src2(gpuDynInst, extData.SRC2);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            VecElemU64 selector = (VecElemU64)src0[lane];
            selector = (selector << 32) | (VecElemU64)src1[lane];
            vdst[lane] = 0;

            DPRINTF(VEGA,
                    "Executing v_perm_b32 src_0 0x%08x, src_1 "
                    "0x%08x, src_2 0x%08x, vdst 0x%08x\n",
                    src0[lane], src1[lane], src2[lane], vdst[lane]);
            DPRINTF(VEGA, "Selector: 0x%08x \n", selector);

            for (int i = 0; i < 4; ++i) {
                VecElemU32 permuted_val = permute(
                    selector, 0xFF & ((VecElemU32)src2[lane] >> (8 * i)));
                vdst[lane] |= (permuted_val << (8 * i));
            }

            DPRINTF(VEGA, "v_perm result: 0x%08x\n", vdst[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_FMA_F16 class methods ---

Inst_VOP3__V_FMA_F16::Inst_VOP3__V_FMA_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_fma_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
    setFlag(FMA);
} // Inst_VOP3__V_FMA_F16

Inst_VOP3__V_FMA_F16::~Inst_VOP3__V_FMA_F16() {} // ~Inst_VOP3__V_FMA_F16

// --- description from .arch file ---
// D.f16 = S0.f16 * S1.f16 + S2.f16.
// Fused half precision multiply add.
void
Inst_VOP3__V_FMA_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_DIV_FIXUP_F16 class methods ---

Inst_VOP3__V_DIV_FIXUP_F16::Inst_VOP3__V_DIV_FIXUP_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_div_fixup_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_DIV_FIXUP_F16

Inst_VOP3__V_DIV_FIXUP_F16::~Inst_VOP3__V_DIV_FIXUP_F16() {
} // ~Inst_VOP3__V_DIV_FIXUP_F16

// --- description from .arch file ---
// sign_out =  sign(S1.f16)^sign(S2.f16);
// if (S2.f16 == NAN)
//     D.f16 = Quiet(S2.f16);
// else if (S1.f16 == NAN)
//     D.f16 = Quiet(S1.f16);
// else if (S1.f16 == S2.f16 == 0)
//     # 0/0
//     D.f16 = pele_nan(0xfe00);
// else if (abs(S1.f16) == abs(S2.f16) == +-INF)
//     # inf/inf
//     D.f16 = pele_nan(0xfe00);
// else if (S1.f16 ==0 || abs(S2.f16) == +-INF)
//     # x/0, or inf/y
//     D.f16 = sign_out ? -INF : INF;
// else if (abs(S1.f16) == +-INF || S2.f16 == 0)
//     # x/inf, 0/y
//     D.f16 = sign_out ? -0 : 0;
// else if ((exp(S2.f16) - exp(S1.f16)) < -150)
//     D.f16 = sign_out ? -underflow : underflow;
// else if (exp(S1.f16) == 255)
//     D.f16 = sign_out ? -overflow : overflow;
// else
//     D.f16 = sign_out ? -abs(S0.f16) : abs(S0.f16).
// Half precision division fixup.
// S0 = Quotient, S1 = Denominator, S3 = Numerator.
// Given a numerator, denominator, and quotient from a divide, this opcode
// will detect and apply special case numerics, touching up the quotient if
// necessary. This opcode also generates invalid, denorm and divide by
// zero exceptions caused by the division.
void
Inst_VOP3__V_DIV_FIXUP_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_LSHL_ADD_U64 class methods ---

Inst_VOP3__V_LSHL_ADD_U64::Inst_VOP3__V_LSHL_ADD_U64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_lshl_add_u64", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_LSHL_ADD_U64

Inst_VOP3__V_LSHL_ADD_U64::~Inst_VOP3__V_LSHL_ADD_U64() {
} // ~Inst_VOP3__V_LSHL_ADD_U64

// --- description from .arch file ---
// D.u = (S0.u << S1.u[4:0]) + S2.u.
void
Inst_VOP3__V_LSHL_ADD_U64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandU64 src2(gpuDynInst, extData.SRC2);
    VecOperandU64 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            int shift_amount = bits(src1[lane], 2, 0);
            shift_amount = shift_amount > 4 ? 0 : shift_amount;
            vdst[lane] = (src0[lane] << shift_amount) + src2[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_CVT_PKACCUM_U8_F32 class methods ---

Inst_VOP3__V_CVT_PKACCUM_U8_F32::Inst_VOP3__V_CVT_PKACCUM_U8_F32(
    InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_pkaccum_u8_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CVT_PKACCUM_U8_F32

Inst_VOP3__V_CVT_PKACCUM_U8_F32::~Inst_VOP3__V_CVT_PKACCUM_U8_F32() {
} // ~Inst_VOP3__V_CVT_PKACCUM_U8_F32

// --- description from .arch file ---
// byte = S1.u[1:0]; bit = byte * 8;
// D.u[bit+7:bit] = flt32_to_uint8(S0.f);
// Pack converted value of S0.f into byte S1 of the destination.
// SQ translates to V_CVT_PK_U8_F32.
// Note: this opcode uses src_c to pass destination in as a source.
void
Inst_VOP3__V_CVT_PKACCUM_U8_F32::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_INTERP_P1_F32 class methods ---

Inst_VOP3__V_INTERP_P1_F32::Inst_VOP3__V_INTERP_P1_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_interp_p1_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_INTERP_P1_F32

Inst_VOP3__V_INTERP_P1_F32::~Inst_VOP3__V_INTERP_P1_F32() {
} // ~Inst_VOP3__V_INTERP_P1_F32

// --- description from .arch file ---
// D.f = P10 * S.f + P0; parameter interpolation (SQ translates to
// V_MAD_F32 for SP).
// CAUTION: when in HALF_LDS mode, D must not be the same GPR as S; if
// D == S then data corruption will occur.
// NOTE: In textual representations the I/J VGPR is the first source and
// the attribute is the second source; however in the VOP3 encoding the
// attribute is stored in the src0 field and the VGPR is stored in the
// src1 field.
void
Inst_VOP3__V_INTERP_P1_F32::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_INTERP_P2_F32 class methods ---

Inst_VOP3__V_INTERP_P2_F32::Inst_VOP3__V_INTERP_P2_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_interp_p2_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_INTERP_P2_F32

Inst_VOP3__V_INTERP_P2_F32::~Inst_VOP3__V_INTERP_P2_F32() {
} // ~Inst_VOP3__V_INTERP_P2_F32

// --- description from .arch file ---
// D.f = P20 * S.f + D.f; parameter interpolation (SQ translates to
// V_MAD_F32 for SP).
// NOTE: In textual representations the I/J VGPR is the first source and
// the attribute is the second source; however in the VOP3 encoding the
// attribute is stored in the src0 field and the VGPR is stored in the
// src1 field.
void
Inst_VOP3__V_INTERP_P2_F32::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_INTERP_MOV_F32 class methods ---

Inst_VOP3__V_INTERP_MOV_F32::Inst_VOP3__V_INTERP_MOV_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_interp_mov_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_INTERP_MOV_F32

Inst_VOP3__V_INTERP_MOV_F32::~Inst_VOP3__V_INTERP_MOV_F32() {
} // ~Inst_VOP3__V_INTERP_MOV_F32

// --- description from .arch file ---
// D.f = {P10,P20,P0}[S.u]; parameter load.
void
Inst_VOP3__V_INTERP_MOV_F32::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_INTERP_P1LL_F16 class methods ---

Inst_VOP3__V_INTERP_P1LL_F16::Inst_VOP3__V_INTERP_P1LL_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_interp_p1ll_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_INTERP_P1LL_F16

Inst_VOP3__V_INTERP_P1LL_F16::~Inst_VOP3__V_INTERP_P1LL_F16() {
} // ~Inst_VOP3__V_INTERP_P1LL_F16

// --- description from .arch file ---
// D.f32 = P10.f16 * S0.f32 + P0.f16.
// 'LL' stands for 'two LDS arguments'.
// attr_word selects the high or low half 16 bits of each LDS dword
// accessed.
// This opcode is available for 32-bank LDS only.
// NOTE: In textual representations the I/J VGPR is the first source and
// the attribute is the second source; however in the VOP3 encoding the
// attribute is stored in the src0 field and the VGPR is stored in the
// src1 field.
void
Inst_VOP3__V_INTERP_P1LL_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_INTERP_P1LV_F16 class methods ---

Inst_VOP3__V_INTERP_P1LV_F16::Inst_VOP3__V_INTERP_P1LV_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_interp_p1lv_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_INTERP_P1LV_F16

Inst_VOP3__V_INTERP_P1LV_F16::~Inst_VOP3__V_INTERP_P1LV_F16() {
} // ~Inst_VOP3__V_INTERP_P1LV_F16

// --- description from .arch file ---
// D.f32 = P10.f16 * S0.f32 + (S2.u32 >> (attr_word * 16)).f16.
// 'LV' stands for 'One LDS and one VGPR argument'.
// S2 holds two parameters, attr_word selects the high or low word of the
// VGPR for this calculation, as well as the high or low half of the LDS
// data.
// Meant for use with 16-bank LDS.
// NOTE: In textual representations the I/J VGPR is the first source and
// the attribute is the second source; however in the VOP3 encoding the
// attribute is stored in the src0 field and the VGPR is stored in the
// src1 field.
void
Inst_VOP3__V_INTERP_P1LV_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_INTERP_P2_F16 class methods ---

Inst_VOP3__V_INTERP_P2_F16::Inst_VOP3__V_INTERP_P2_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_interp_p2_f16", false)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_INTERP_P2_F16

Inst_VOP3__V_INTERP_P2_F16::~Inst_VOP3__V_INTERP_P2_F16() {
} // ~Inst_VOP3__V_INTERP_P2_F16

// --- description from .arch file ---
// D.f16 = P20.f16 * S0.f32 + S2.f32.
// Final computation. attr_word selects LDS high or low 16bits. Used for
// both 16- and 32-bank LDS.
// Result is always written to the 16 LSBs of the destination VGPR.
// NOTE: In textual representations the I/J VGPR is the first source and
// the attribute is the second source; however in the VOP3 encoding the
// attribute is stored in the src0 field and the VGPR is stored in the
// src1 field.
void
Inst_VOP3__V_INTERP_P2_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_ADD_F64 class methods ---

Inst_VOP3__V_ADD_F64::Inst_VOP3__V_ADD_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_add_f64", false)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_ADD_F64

Inst_VOP3__V_ADD_F64::~Inst_VOP3__V_ADD_F64() {} // ~Inst_VOP3__V_ADD_F64

// --- description from .arch file ---
// D.d = S0.d + S1.d.
void
Inst_VOP3__V_ADD_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    VecOperandF64 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (instData.ABS & 0x2) {
        src1.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            if (std::isnan(src0[lane]) || std::isnan(src1[lane])) {
                vdst[lane] = NAN;
            } else if (std::isinf(src0[lane]) && std::isinf(src1[lane])) {
                if (std::signbit(src0[lane]) != std::signbit(src1[lane])) {
                    vdst[lane] = NAN;
                } else {
                    vdst[lane] = src0[lane];
                }
            } else if (std::isinf(src0[lane])) {
                vdst[lane] = src0[lane];
            } else if (std::isinf(src1[lane])) {
                vdst[lane] = src1[lane];
            } else if (std::fpclassify(src0[lane]) == FP_SUBNORMAL ||
                       std::fpclassify(src0[lane]) == FP_ZERO) {
                if (std::fpclassify(src1[lane]) == FP_SUBNORMAL ||
                    std::fpclassify(src1[lane]) == FP_ZERO) {
                    if (std::signbit(src0[lane]) && std::signbit(src1[lane])) {
                        vdst[lane] = -0.0;
                    } else {
                        vdst[lane] = 0.0;
                    }
                } else {
                    vdst[lane] = src1[lane];
                }
            } else if (std::fpclassify(src1[lane]) == FP_SUBNORMAL ||
                       std::fpclassify(src1[lane]) == FP_ZERO) {
                if (std::fpclassify(src0[lane]) == FP_SUBNORMAL ||
                    std::fpclassify(src0[lane]) == FP_ZERO) {
                    if (std::signbit(src0[lane]) && std::signbit(src1[lane])) {
                        vdst[lane] = -0.0;
                    } else {
                        vdst[lane] = 0.0;
                    }
                } else {
                    vdst[lane] = src0[lane];
                }
            } else {
                vdst[lane] = src0[lane] + src1[lane];
            }
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MUL_F64 class methods ---

Inst_VOP3__V_MUL_F64::Inst_VOP3__V_MUL_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_mul_f64", false)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_MUL_F64

Inst_VOP3__V_MUL_F64::~Inst_VOP3__V_MUL_F64() {} // ~Inst_VOP3__V_MUL_F64

// --- description from .arch file ---
// D.d = S0.d * S1.d.
void
Inst_VOP3__V_MUL_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    VecOperandF64 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (instData.ABS & 0x2) {
        src1.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            if (std::isnan(src0[lane]) || std::isnan(src1[lane])) {
                vdst[lane] = NAN;
            } else if ((std::fpclassify(src0[lane]) == FP_SUBNORMAL ||
                        std::fpclassify(src0[lane]) == FP_ZERO) &&
                       !std::signbit(src0[lane])) {
                if (std::isinf(src1[lane])) {
                    vdst[lane] = NAN;
                } else if (!std::signbit(src1[lane])) {
                    vdst[lane] = +0.0;
                } else {
                    vdst[lane] = -0.0;
                }
            } else if ((std::fpclassify(src0[lane]) == FP_SUBNORMAL ||
                        std::fpclassify(src0[lane]) == FP_ZERO) &&
                       std::signbit(src0[lane])) {
                if (std::isinf(src1[lane])) {
                    vdst[lane] = NAN;
                } else if (std::signbit(src1[lane])) {
                    vdst[lane] = +0.0;
                } else {
                    vdst[lane] = -0.0;
                }
            } else if (std::isinf(src0[lane]) && !std::signbit(src0[lane])) {
                if (std::fpclassify(src1[lane]) == FP_SUBNORMAL ||
                    std::fpclassify(src1[lane]) == FP_ZERO) {
                    vdst[lane] = NAN;
                } else if (!std::signbit(src1[lane])) {
                    vdst[lane] = +INFINITY;
                } else {
                    vdst[lane] = -INFINITY;
                }
            } else if (std::isinf(src0[lane]) && std::signbit(src0[lane])) {
                if (std::fpclassify(src1[lane]) == FP_SUBNORMAL ||
                    std::fpclassify(src1[lane]) == FP_ZERO) {
                    vdst[lane] = NAN;
                } else if (std::signbit(src1[lane])) {
                    vdst[lane] = +INFINITY;
                } else {
                    vdst[lane] = -INFINITY;
                }
            } else {
                vdst[lane] = src0[lane] * src1[lane];
            }
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MIN_F64 class methods ---

Inst_VOP3__V_MIN_F64::Inst_VOP3__V_MIN_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_min_f64", false)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_MIN_F64

Inst_VOP3__V_MIN_F64::~Inst_VOP3__V_MIN_F64() {} // ~Inst_VOP3__V_MIN_F64

// --- description from .arch file ---
// D.d = min(S0.d, S1.d).
void
Inst_VOP3__V_MIN_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    VecOperandF64 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (instData.ABS & 0x2) {
        src1.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::fmin(src0[lane], src1[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MAX_F64 class methods ---

Inst_VOP3__V_MAX_F64::Inst_VOP3__V_MAX_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_max_f64", false)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_MAX_F64

Inst_VOP3__V_MAX_F64::~Inst_VOP3__V_MAX_F64() {} // ~Inst_VOP3__V_MAX_F64

// --- description from .arch file ---
// D.d = max(S0.d, S1.d).
void
Inst_VOP3__V_MAX_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    VecOperandF64 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (instData.ABS & 0x2) {
        src1.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    if (extData.NEG & 0x2) {
        src1.negModifier();
    }

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::fmax(src0[lane], src1[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_LDEXP_F64 class methods ---

Inst_VOP3__V_LDEXP_F64::Inst_VOP3__V_LDEXP_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_ldexp_f64", false)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_LDEXP_F64

Inst_VOP3__V_LDEXP_F64::~Inst_VOP3__V_LDEXP_F64() {} // ~Inst_VOP3__V_LDEXP_F64

// --- description from .arch file ---
// D.d = pow(S0.d, S1.i[31:0]).
void
Inst_VOP3__V_LDEXP_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    VecOperandF64 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    if (instData.ABS & 0x1) {
        src0.absModifier();
    }

    if (extData.NEG & 0x1) {
        src0.negModifier();
    }

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            if (std::isnan(src0[lane]) || std::isinf(src0[lane])) {
                vdst[lane] = src0[lane];
            } else if (std::fpclassify(src0[lane]) == FP_SUBNORMAL ||
                       std::fpclassify(src0[lane]) == FP_ZERO) {
                if (std::signbit(src0[lane])) {
                    vdst[lane] = -0.0;
                } else {
                    vdst[lane] = +0.0;
                }
            } else {
                vdst[lane] = std::ldexp(src0[lane], src1[lane]);
            }
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MUL_LO_U32 class methods ---

Inst_VOP3__V_MUL_LO_U32::Inst_VOP3__V_MUL_LO_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_mul_lo_u32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_MUL_LO_U32

Inst_VOP3__V_MUL_LO_U32::~Inst_VOP3__V_MUL_LO_U32() {
} // ~Inst_VOP3__V_MUL_LO_U32

// --- description from .arch file ---
// D.u = S0.u * S1.u.
void
Inst_VOP3__V_MUL_LO_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            VecElemI64 s0 = (VecElemI64)src0[lane];
            VecElemI64 s1 = (VecElemI64)src1[lane];
            vdst[lane] = (VecElemU32)((s0 * s1) & 0xffffffffLL);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MUL_HI_U32 class methods ---

Inst_VOP3__V_MUL_HI_U32::Inst_VOP3__V_MUL_HI_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_mul_hi_u32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_MUL_HI_U32

Inst_VOP3__V_MUL_HI_U32::~Inst_VOP3__V_MUL_HI_U32() {
} // ~Inst_VOP3__V_MUL_HI_U32

// --- description from .arch file ---
// D.u = (S0.u * S1.u) >> 32.
void
Inst_VOP3__V_MUL_HI_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            VecElemI64 s0 = (VecElemI64)src0[lane];
            VecElemI64 s1 = (VecElemI64)src1[lane];
            vdst[lane] = (VecElemU32)(((s0 * s1) >> 32) & 0xffffffffLL);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MUL_HI_I32 class methods ---

Inst_VOP3__V_MUL_HI_I32::Inst_VOP3__V_MUL_HI_I32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_mul_hi_i32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_MUL_HI_I32

Inst_VOP3__V_MUL_HI_I32::~Inst_VOP3__V_MUL_HI_I32() {
} // ~Inst_VOP3__V_MUL_HI_I32

// --- description from .arch file ---
// D.i = (S0.i * S1.i) >> 32.
void
Inst_VOP3__V_MUL_HI_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1);
    VecOperandI32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            VecElemI64 s0 = (VecElemI64)src0[lane];
            VecElemI64 s1 = (VecElemI64)src1[lane];
            vdst[lane] = (VecElemI32)(((s0 * s1) >> 32LL) & 0xffffffffLL);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_LDEXP_F32 class methods ---

Inst_VOP3__V_LDEXP_F32::Inst_VOP3__V_LDEXP_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_ldexp_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_LDEXP_F32

Inst_VOP3__V_LDEXP_F32::~Inst_VOP3__V_LDEXP_F32() {} // ~Inst_VOP3__V_LDEXP_F32

// --- description from .arch file ---
// D.f = pow(S0.f, S1.i)
void
Inst_VOP3__V_LDEXP_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::ldexp(src0[lane], src1[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_READLANE_B32 class methods ---

Inst_VOP3__V_READLANE_B32::Inst_VOP3__V_READLANE_B32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_readlane_b32", true)
{
    setFlag(ALU);
    setFlag(IgnoreExec);
} // Inst_VOP3__V_READLANE_B32

Inst_VOP3__V_READLANE_B32::~Inst_VOP3__V_READLANE_B32() {
} // ~Inst_VOP3__V_READLANE_B32

// --- description from .arch file ---
// Copy one VGPR value to one SGPR. D = SGPR-dest, S0 = Source Data (VGPR#
// or M0(lds-direct)), S1 = Lane Select (SGPR or M0). Ignores exec mask.
// Input and output modifiers not supported; this is an untyped operation.
void
Inst_VOP3__V_READLANE_B32::execute(GPUDynInstPtr gpuDynInst)
{
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstScalarOperandU32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU32 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    sdst = src0[src1.rawData() & 0x3f];

    sdst.write();
} // execute

// --- Inst_VOP3__V_WRITELANE_B32 class methods ---

Inst_VOP3__V_WRITELANE_B32::Inst_VOP3__V_WRITELANE_B32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_writelane_b32", false)
{
    setFlag(ALU);
    setFlag(IgnoreExec);
} // Inst_VOP3__V_WRITELANE_B32

Inst_VOP3__V_WRITELANE_B32::~Inst_VOP3__V_WRITELANE_B32() {
} // ~Inst_VOP3__V_WRITELANE_B32

// --- description from .arch file ---
// Write value into one VGPR in one lane. D = VGPR-dest, S0 = Source Data
// (sgpr, m0, exec or constants), S1 = Lane Select (SGPR or M0). Ignores
// exec mask.
// Input and output modifiers not supported; this is an untyped operation.
// SQ translates to V_MOV_B32.
void
Inst_VOP3__V_WRITELANE_B32::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstScalarOperandU32 src1(gpuDynInst, extData.SRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.read();
    src1.read();
    vdst.read();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    vdst[src1.rawData() & 0x3f] = src0.rawData();

    vdst.write();
} // execute

// --- Inst_VOP3__V_BCNT_U32_B32 class methods ---

Inst_VOP3__V_BCNT_U32_B32::Inst_VOP3__V_BCNT_U32_B32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_bcnt_u32_b32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_BCNT_U32_B32

Inst_VOP3__V_BCNT_U32_B32::~Inst_VOP3__V_BCNT_U32_B32() {
} // ~Inst_VOP3__V_BCNT_U32_B32

// --- description from .arch file ---
// D.u = CountOneBits(S0.u) + S1.u. Bit count.
void
Inst_VOP3__V_BCNT_U32_B32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = popCount(src0[lane]) + src1[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MBCNT_LO_U32_B32 class methods ---

Inst_VOP3__V_MBCNT_LO_U32_B32::Inst_VOP3__V_MBCNT_LO_U32_B32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_mbcnt_lo_u32_b32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_MBCNT_LO_U32_B32

Inst_VOP3__V_MBCNT_LO_U32_B32::~Inst_VOP3__V_MBCNT_LO_U32_B32() {
} // ~Inst_VOP3__V_MBCNT_LO_U32_B32

// --- description from .arch file ---
// ThreadMask = (1 << ThreadPosition) - 1;
// D.u = CountOneBits(S0.u & ThreadMask[31:0]) + S1.u.
// Masked bit count, ThreadPosition is the position of this thread in the
// ---  wavefront (in 0..63).
void
Inst_VOP3__V_MBCNT_LO_U32_B32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);
    uint64_t threadMask = 0;

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            threadMask = ((1LL << lane) - 1LL);
            vdst[lane] =
                popCount(src0[lane] & bits(threadMask, 31, 0)) + src1[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_MBCNT_HI_U32_B32 class methods ---

Inst_VOP3__V_MBCNT_HI_U32_B32::Inst_VOP3__V_MBCNT_HI_U32_B32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_mbcnt_hi_u32_b32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_MBCNT_HI_U32_B32

Inst_VOP3__V_MBCNT_HI_U32_B32::~Inst_VOP3__V_MBCNT_HI_U32_B32() {
} // ~Inst_VOP3__V_MBCNT_HI_U32_B32

// --- description from .arch file ---
// ThreadMask = (1 << ThreadPosition) - 1;
// D.u = CountOneBits(S0.u & ThreadMask[63:32]) + S1.u.
// Masked bit count, ThreadPosition is the position of this thread in the
// ---  wavefront (in 0..63).
void
Inst_VOP3__V_MBCNT_HI_U32_B32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);
    uint64_t threadMask = 0;

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            threadMask = ((1LL << lane) - 1LL);
            vdst[lane] =
                popCount(src0[lane] & bits(threadMask, 63, 32)) + src1[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_LSHLREV_B64 class methods ---

Inst_VOP3__V_LSHLREV_B64::Inst_VOP3__V_LSHLREV_B64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_lshlrev_b64", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_LSHLREV_B64

Inst_VOP3__V_LSHLREV_B64::~Inst_VOP3__V_LSHLREV_B64() {
} // ~Inst_VOP3__V_LSHLREV_B64

// --- description from .arch file ---
// D.u64 = S1.u64 << S0.u[5:0].
// SQ translates this to an internal SP opcode.
void
Inst_VOP3__V_LSHLREV_B64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU64 src1(gpuDynInst, extData.SRC1);
    VecOperandU64 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src1[lane] << bits(src0[lane], 5, 0);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_LSHRREV_B64 class methods ---

Inst_VOP3__V_LSHRREV_B64::Inst_VOP3__V_LSHRREV_B64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_lshrrev_b64", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_LSHRREV_B64

Inst_VOP3__V_LSHRREV_B64::~Inst_VOP3__V_LSHRREV_B64() {
} // ~Inst_VOP3__V_LSHRREV_B64

// --- description from .arch file ---
// D.u64 = S1.u64 >> S0.u[5:0].
// The vacated bits are set to zero.
// SQ translates this to an internal SP opcode.
void
Inst_VOP3__V_LSHRREV_B64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU64 src1(gpuDynInst, extData.SRC1);
    VecOperandU64 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src1[lane] >> bits(src0[lane], 5, 0);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_ASHRREV_I64 class methods ---

Inst_VOP3__V_ASHRREV_I64::Inst_VOP3__V_ASHRREV_I64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_ashrrev_i64", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_ASHRREV_I64

Inst_VOP3__V_ASHRREV_I64::~Inst_VOP3__V_ASHRREV_I64() {
} // ~Inst_VOP3__V_ASHRREV_I64

// --- description from .arch file ---
// D.u64 = signext(S1.u64) >> S0.u[5:0].
// The vacated bits are set to the sign bit of the input value.
// SQ translates this to an internal SP opcode.
void
Inst_VOP3__V_ASHRREV_I64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI64 src1(gpuDynInst, extData.SRC1);
    VecOperandU64 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src1[lane] >> bits(src0[lane], 5, 0);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_TRIG_PREOP_F64 class methods ---

Inst_VOP3__V_TRIG_PREOP_F64::Inst_VOP3__V_TRIG_PREOP_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_trig_preop_f64", false)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_TRIG_PREOP_F64

Inst_VOP3__V_TRIG_PREOP_F64::~Inst_VOP3__V_TRIG_PREOP_F64() {
} // ~Inst_VOP3__V_TRIG_PREOP_F64

// --- description from .arch file ---
// D.d = Look Up 2/PI (S0.d) with segment select S1.u[4:0]. This operation
// returns an aligned, double precision segment of 2/PI needed to do range
// reduction on S0.d (double-precision value). Multiple segments can be
// specified through S1.u[4:0]. Rounding is always round-to-zero. Large
// inputs (exp > 1968) are scaled to avoid loss of precision through
// denormalization.
void
Inst_VOP3__V_TRIG_PREOP_F64::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_BFM_B32 class methods ---

Inst_VOP3__V_BFM_B32::Inst_VOP3__V_BFM_B32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_bfm_b32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_BFM_B32

Inst_VOP3__V_BFM_B32::~Inst_VOP3__V_BFM_B32() {} // ~Inst_VOP3__V_BFM_B32

// --- description from .arch file ---
// D.u = ((1<<S0.u[4:0])-1) << S1.u[4:0]; S0 is the bitfield width and S1
// is the bitfield offset.
void
Inst_VOP3__V_BFM_B32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    /**
     * input modifiers are supported by FP operations only
     */
    assert(!(instData.ABS & 0x1));
    assert(!(instData.ABS & 0x2));
    assert(!(instData.ABS & 0x4));
    assert(!(extData.NEG & 0x1));
    assert(!(extData.NEG & 0x2));
    assert(!(extData.NEG & 0x4));

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = ((1 << bits(src0[lane], 4, 0)) - 1)
                         << bits(src1[lane], 4, 0);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP3__V_CVT_PKNORM_I16_F32 class methods ---

Inst_VOP3__V_CVT_PKNORM_I16_F32::Inst_VOP3__V_CVT_PKNORM_I16_F32(
    InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_pknorm_i16_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CVT_PKNORM_I16_F32

Inst_VOP3__V_CVT_PKNORM_I16_F32::~Inst_VOP3__V_CVT_PKNORM_I16_F32() {
} // ~Inst_VOP3__V_CVT_PKNORM_I16_F32

// --- description from .arch file ---
// D = {(snorm)S1.f, (snorm)S0.f}.
void
Inst_VOP3__V_CVT_PKNORM_I16_F32::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CVT_PKNORM_U16_F32 class methods ---

Inst_VOP3__V_CVT_PKNORM_U16_F32::Inst_VOP3__V_CVT_PKNORM_U16_F32(
    InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_pknorm_u16_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CVT_PKNORM_U16_F32

Inst_VOP3__V_CVT_PKNORM_U16_F32::~Inst_VOP3__V_CVT_PKNORM_U16_F32() {
} // ~Inst_VOP3__V_CVT_PKNORM_U16_F32

// --- description from .arch file ---
// D = {(unorm)S1.f, (unorm)S0.f}.
void
Inst_VOP3__V_CVT_PKNORM_U16_F32::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CVT_PKRTZ_F16_F32 class methods ---

Inst_VOP3__V_CVT_PKRTZ_F16_F32::Inst_VOP3__V_CVT_PKRTZ_F16_F32(
    InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_pkrtz_f16_f32", false)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CVT_PKRTZ_F16_F32

Inst_VOP3__V_CVT_PKRTZ_F16_F32::~Inst_VOP3__V_CVT_PKRTZ_F16_F32() {
} // ~Inst_VOP3__V_CVT_PKRTZ_F16_F32

// --- description from .arch file ---
// D = {flt32_to_flt16(S1.f),flt32_to_flt16(S0.f)}, with round-toward-zero
// ---  regardless of current round mode setting in hardware.
// This opcode is intended for use with 16-bit compressed exports.
// See V_CVT_F16_F32 for a version that respects the current rounding mode.
void
Inst_VOP3__V_CVT_PKRTZ_F16_F32::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CVT_PK_U16_U32 class methods ---

Inst_VOP3__V_CVT_PK_U16_U32::Inst_VOP3__V_CVT_PK_U16_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_pk_u16_u32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_CVT_PK_U16_U32

Inst_VOP3__V_CVT_PK_U16_U32::~Inst_VOP3__V_CVT_PK_U16_U32() {
} // ~Inst_VOP3__V_CVT_PK_U16_U32

// --- description from .arch file ---
// D = {uint32_to_uint16(S1.u), uint32_to_uint16(S0.u)}.
void
Inst_VOP3__V_CVT_PK_U16_U32::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CVT_PK_I16_I32 class methods ---

Inst_VOP3__V_CVT_PK_I16_I32::Inst_VOP3__V_CVT_PK_I16_I32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cvt_pk_i16_i32", false)
{
    setFlag(ALU);
} // Inst_VOP3__V_CVT_PK_I16_I32

Inst_VOP3__V_CVT_PK_I16_I32::~Inst_VOP3__V_CVT_PK_I16_I32() {
} // ~Inst_VOP3__V_CVT_PK_I16_I32

// --- description from .arch file ---
// D = {int32_to_int16(S1.i), int32_to_int16(S0.i)}.
void
Inst_VOP3__V_CVT_PK_I16_I32::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute
} // namespace VegaISA
} // namespace gem5
