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
#include "debug/VEGA.hh"

namespace gem5
{

namespace VegaISA
{
// --- Inst_VOP2__V_CNDMASK_B32 class methods ---

Inst_VOP2__V_CNDMASK_B32::Inst_VOP2__V_CNDMASK_B32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_cndmask_b32")
{
    setFlag(ALU);
    setFlag(ReadsVCC);
} // Inst_VOP2__V_CNDMASK_B32

Inst_VOP2__V_CNDMASK_B32::~Inst_VOP2__V_CNDMASK_B32() {
} // ~Inst_VOP2__V_CNDMASK_B32

// --- description from .arch file ---
// D.u = (VCC[i] ? S1.u : S0.u) (i = threadID in wave); VOP3: specify VCC
// as a scalar GPR in S2.
void
Inst_VOP2__V_CNDMASK_B32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, instData.VSRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);
    ConstScalarOperandU64 vcc(gpuDynInst, REG_VCC_LO);

    src0.readSrc();
    src1.read();
    vcc.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = bits(vcc.rawData(), lane) ? src1[lane] : src0[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_ADD_F32 class methods ---

Inst_VOP2__V_ADD_F32::Inst_VOP2__V_ADD_F32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_add_f32")
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP2__V_ADD_F32

Inst_VOP2__V_ADD_F32::~Inst_VOP2__V_ADD_F32() {} // ~Inst_VOP2__V_ADD_F32

// --- description from .arch file ---
// D.f = S0.f + S1.f.
void
Inst_VOP2__V_ADD_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, instData.SRC0);
    VecOperandF32 src1(gpuDynInst, instData.VSRC1);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    if (isDPPInst()) {
        VecOperandF32 src0_dpp(gpuDynInst, extData.iFmt_VOP_DPP.SRC0);
        src0_dpp.read();

        DPRINTF(VEGA,
                "Handling V_ADD_F32 SRC DPP. SRC0: register v[%d], "
                "DPP_CTRL: 0x%#x, SRC0_ABS: %d, SRC0_NEG: %d, "
                "SRC1_ABS: %d, SRC1_NEG: %d, BC: %d, "
                "BANK_MASK: %d, ROW_MASK: %d\n",
                extData.iFmt_VOP_DPP.SRC0, extData.iFmt_VOP_DPP.DPP_CTRL,
                extData.iFmt_VOP_DPP.SRC0_ABS, extData.iFmt_VOP_DPP.SRC0_NEG,
                extData.iFmt_VOP_DPP.SRC1_ABS, extData.iFmt_VOP_DPP.SRC1_NEG,
                extData.iFmt_VOP_DPP.BC, extData.iFmt_VOP_DPP.BANK_MASK,
                extData.iFmt_VOP_DPP.ROW_MASK);

        processDPP(gpuDynInst, extData.iFmt_VOP_DPP, src0_dpp, src1);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = src0_dpp[lane] + src1[lane];
            }
        }
    } else {
        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = src0[lane] + src1[lane];
            }
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_SUB_F32 class methods ---

Inst_VOP2__V_SUB_F32::Inst_VOP2__V_SUB_F32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_sub_f32")
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP2__V_SUB_F32

Inst_VOP2__V_SUB_F32::~Inst_VOP2__V_SUB_F32() {} // ~Inst_VOP2__V_SUB_F32

// --- description from .arch file ---
// D.f = S0.f - S1.f.
// SQ translates to V_ADD_F32.
void
Inst_VOP2__V_SUB_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, instData.VSRC1);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src0[lane] - src1[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_SUBREV_F32 class methods ---

Inst_VOP2__V_SUBREV_F32::Inst_VOP2__V_SUBREV_F32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_subrev_f32")
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP2__V_SUBREV_F32

Inst_VOP2__V_SUBREV_F32::~Inst_VOP2__V_SUBREV_F32() {
} // ~Inst_VOP2__V_SUBREV_F32

// --- description from .arch file ---
// D.f = S1.f - S0.f.
// SQ translates to V_ADD_F32.
void
Inst_VOP2__V_SUBREV_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, instData.VSRC1);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src1[lane] - src0[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_MUL_LEGACY_F32 class methods ---

Inst_VOP2__V_MUL_LEGACY_F32::Inst_VOP2__V_MUL_LEGACY_F32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_mul_legacy_f32")
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP2__V_MUL_LEGACY_F32

Inst_VOP2__V_MUL_LEGACY_F32::~Inst_VOP2__V_MUL_LEGACY_F32() {
} // ~Inst_VOP2__V_MUL_LEGACY_F32

// --- description from .arch file ---
// D.f = S0.f * S1.f (DX9 rules, 0.0*x = 0.0).
void
Inst_VOP2__V_MUL_LEGACY_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, instData.VSRC1);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src0[lane] * src1[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_MUL_F32 class methods ---

Inst_VOP2__V_MUL_F32::Inst_VOP2__V_MUL_F32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_mul_f32")
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP2__V_MUL_F32

Inst_VOP2__V_MUL_F32::~Inst_VOP2__V_MUL_F32() {} // ~Inst_VOP2__V_MUL_F32

// --- description from .arch file ---
// D.f = S0.f * S1.f.
void
Inst_VOP2__V_MUL_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, instData.VSRC1);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

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

// --- Inst_VOP2__V_MUL_I32_I24 class methods ---

Inst_VOP2__V_MUL_I32_I24::Inst_VOP2__V_MUL_I32_I24(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_mul_i32_i24")
{
    setFlag(ALU);
} // Inst_VOP2__V_MUL_I32_I24

Inst_VOP2__V_MUL_I32_I24::~Inst_VOP2__V_MUL_I32_I24() {
} // ~Inst_VOP2__V_MUL_I32_I24

// --- description from .arch file ---
// D.i = S0.i[23:0] * S1.i[23:0].
void
Inst_VOP2__V_MUL_I32_I24::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, instData.VSRC1);
    VecOperandI32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = sext<24>(bits(src0[lane], 23, 0)) *
                         sext<24>(bits(src1[lane], 23, 0));
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_MUL_HI_I32_I24 class methods ---

Inst_VOP2__V_MUL_HI_I32_I24::Inst_VOP2__V_MUL_HI_I32_I24(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_mul_hi_i32_i24")
{
    setFlag(ALU);
} // Inst_VOP2__V_MUL_HI_I32_I24

Inst_VOP2__V_MUL_HI_I32_I24::~Inst_VOP2__V_MUL_HI_I32_I24() {
} // ~Inst_VOP2__V_MUL_HI_I32_I24

// --- description from .arch file ---
// D.i = (S0.i[23:0] * S1.i[23:0])>>32.
void
Inst_VOP2__V_MUL_HI_I32_I24::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, instData.VSRC1);
    VecOperandI32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

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

// --- Inst_VOP2__V_MUL_U32_U24 class methods ---

Inst_VOP2__V_MUL_U32_U24::Inst_VOP2__V_MUL_U32_U24(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_mul_u32_u24")
{
    setFlag(ALU);
} // Inst_VOP2__V_MUL_U32_U24

Inst_VOP2__V_MUL_U32_U24::~Inst_VOP2__V_MUL_U32_U24() {
} // ~Inst_VOP2__V_MUL_U32_U24

// --- description from .arch file ---
// D.u = S0.u[23:0] * S1.u[23:0].
void
Inst_VOP2__V_MUL_U32_U24::execute(GPUDynInstPtr gpuDynInst)
{
    auto opImpl = [](VecOperandU32 &src0, VecOperandU32 &src1,
                     VecOperandU32 &vdst, Wavefront *wf) {
        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = bits(src0[lane], 23, 0) * bits(src1[lane], 23, 0);
            }
        }
    };

    vop2Helper<ConstVecOperandU32, VecOperandU32>(gpuDynInst, opImpl);
} // execute

// --- Inst_VOP2__V_MUL_HI_U32_U24 class methods ---

Inst_VOP2__V_MUL_HI_U32_U24::Inst_VOP2__V_MUL_HI_U32_U24(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_mul_hi_u32_u24")
{
    setFlag(ALU);
} // Inst_VOP2__V_MUL_HI_U32_U24

Inst_VOP2__V_MUL_HI_U32_U24::~Inst_VOP2__V_MUL_HI_U32_U24() {
} // ~Inst_VOP2__V_MUL_HI_U32_U24

// --- description from .arch file ---
// D.i = (S0.u[23:0] * S1.u[23:0])>>32.
void
Inst_VOP2__V_MUL_HI_U32_U24::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, instData.VSRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            VecElemU64 tmp_src0 = (VecElemU64)bits(src0[lane], 23, 0);
            VecElemU64 tmp_src1 = (VecElemU64)bits(src1[lane], 23, 0);
            vdst[lane] = (VecElemU32)((tmp_src0 * tmp_src1) >> 32);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_MIN_F32 class methods ---

Inst_VOP2__V_MIN_F32::Inst_VOP2__V_MIN_F32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_min_f32")
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP2__V_MIN_F32

Inst_VOP2__V_MIN_F32::~Inst_VOP2__V_MIN_F32() {} // ~Inst_VOP2__V_MIN_F32

// --- description from .arch file ---
// D.f = (S0.f < S1.f ? S0.f : S1.f).
void
Inst_VOP2__V_MIN_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, instData.VSRC1);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::fmin(src0[lane], src1[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_MAX_F32 class methods ---

Inst_VOP2__V_MAX_F32::Inst_VOP2__V_MAX_F32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_max_f32")
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP2__V_MAX_F32

Inst_VOP2__V_MAX_F32::~Inst_VOP2__V_MAX_F32() {} // ~Inst_VOP2__V_MAX_F32

// --- description from .arch file ---
// D.f = (S0.f >= S1.f ? S0.f : S1.f).
void
Inst_VOP2__V_MAX_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, instData.VSRC1);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::fmax(src0[lane], src1[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_MIN_I32 class methods ---

Inst_VOP2__V_MIN_I32::Inst_VOP2__V_MIN_I32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_min_i32")
{
    setFlag(ALU);
} // Inst_VOP2__V_MIN_I32

Inst_VOP2__V_MIN_I32::~Inst_VOP2__V_MIN_I32() {} // ~Inst_VOP2__V_MIN_I32

// --- description from .arch file ---
// D.i = min(S0.i, S1.i).
void
Inst_VOP2__V_MIN_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, instData.VSRC1);
    VecOperandI32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::min(src0[lane], src1[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_MAX_I32 class methods ---

Inst_VOP2__V_MAX_I32::Inst_VOP2__V_MAX_I32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_max_i32")
{
    setFlag(ALU);
} // Inst_VOP2__V_MAX_I32

Inst_VOP2__V_MAX_I32::~Inst_VOP2__V_MAX_I32() {} // ~Inst_VOP2__V_MAX_I32

// --- description from .arch file ---
// D.i = max(S0.i, S1.i).
void
Inst_VOP2__V_MAX_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, instData.VSRC1);
    VecOperandI32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::max(src0[lane], src1[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_MIN_U32 class methods ---

Inst_VOP2__V_MIN_U32::Inst_VOP2__V_MIN_U32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_min_u32")
{
    setFlag(ALU);
} // Inst_VOP2__V_MIN_U32

Inst_VOP2__V_MIN_U32::~Inst_VOP2__V_MIN_U32() {} // ~Inst_VOP2__V_MIN_U32

// --- description from .arch file ---
// D.u = min(S0.u, S1.u).
void
Inst_VOP2__V_MIN_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, instData.VSRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::min(src0[lane], src1[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_MAX_U32 class methods ---

Inst_VOP2__V_MAX_U32::Inst_VOP2__V_MAX_U32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_max_u32")
{
    setFlag(ALU);
} // Inst_VOP2__V_MAX_U32

Inst_VOP2__V_MAX_U32::~Inst_VOP2__V_MAX_U32() {} // ~Inst_VOP2__V_MAX_U32

// --- description from .arch file ---
// D.u = max(S0.u, S1.u).
void
Inst_VOP2__V_MAX_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, instData.VSRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::max(src0[lane], src1[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_LSHRREV_B32 class methods ---

Inst_VOP2__V_LSHRREV_B32::Inst_VOP2__V_LSHRREV_B32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_lshrrev_b32")
{
    setFlag(ALU);
} // Inst_VOP2__V_LSHRREV_B32

Inst_VOP2__V_LSHRREV_B32::~Inst_VOP2__V_LSHRREV_B32() {
} // ~Inst_VOP2__V_LSHRREV_B32

// --- description from .arch file ---
// D.u = S1.u >> S0.u[4:0].
// The vacated bits are set to zero.
// SQ translates this to an internal SP opcode.
void
Inst_VOP2__V_LSHRREV_B32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, instData.VSRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src1[lane] >> bits(src0[lane], 4, 0);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_ASHRREV_I32 class methods ---

Inst_VOP2__V_ASHRREV_I32::Inst_VOP2__V_ASHRREV_I32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_ashrrev_i32")
{
    setFlag(ALU);
} // Inst_VOP2__V_ASHRREV_I32

Inst_VOP2__V_ASHRREV_I32::~Inst_VOP2__V_ASHRREV_I32() {
} // ~Inst_VOP2__V_ASHRREV_I32

// --- description from .arch file ---
// D.i = signext(S1.i) >> S0.i[4:0].
// The vacated bits are set to the sign bit of the input value.
// SQ translates this to an internal SP opcode.
void
Inst_VOP2__V_ASHRREV_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, instData.VSRC1);
    VecOperandI32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src1[lane] >> bits(src0[lane], 4, 0);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_LSHLREV_B32 class methods ---

Inst_VOP2__V_LSHLREV_B32::Inst_VOP2__V_LSHLREV_B32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_lshlrev_b32")
{
    setFlag(ALU);
} // Inst_VOP2__V_LSHLREV_B32

Inst_VOP2__V_LSHLREV_B32::~Inst_VOP2__V_LSHLREV_B32() {
} // ~Inst_VOP2__V_LSHLREV_B32

// --- description from .arch file ---
// D.u = S1.u << S0.u[4:0].
// SQ translates this to an internal SP opcode.
void
Inst_VOP2__V_LSHLREV_B32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, instData.SRC0);
    VecOperandU32 src1(gpuDynInst, instData.VSRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    if (isSDWAInst()) {
        VecOperandU32 src0_sdwa(gpuDynInst, extData.iFmt_VOP_SDWA.SRC0);
        // use copies of original src0, src1, and vdst during selecting
        VecOperandU32 origSrc0_sdwa(gpuDynInst, extData.iFmt_VOP_SDWA.SRC0);
        VecOperandU32 origSrc1(gpuDynInst, instData.VSRC1);
        VecOperandU32 origVdst(gpuDynInst, instData.VDST);

        src0_sdwa.read();
        origSrc0_sdwa.read();
        origSrc1.read();

        DPRINTF(
            VEGA,
            "Handling V_LSHLREV_B32 SRC SDWA. SRC0: register "
            "v[%d], DST_SEL: %d, DST_U: %d, CLMP: %d, SRC0_SEL: "
            "%d, SRC0_SEXT: %d, SRC0_NEG: %d, SRC0_ABS: %d, SRC1_SEL: "
            "%d, SRC1_SEXT: %d, SRC1_NEG: %d, SRC1_ABS: %d\n",
            extData.iFmt_VOP_SDWA.SRC0, extData.iFmt_VOP_SDWA.DST_SEL,
            extData.iFmt_VOP_SDWA.DST_U, extData.iFmt_VOP_SDWA.CLMP,
            extData.iFmt_VOP_SDWA.SRC0_SEL, extData.iFmt_VOP_SDWA.SRC0_SEXT,
            extData.iFmt_VOP_SDWA.SRC0_NEG, extData.iFmt_VOP_SDWA.SRC0_ABS,
            extData.iFmt_VOP_SDWA.SRC1_SEL, extData.iFmt_VOP_SDWA.SRC1_SEXT,
            extData.iFmt_VOP_SDWA.SRC1_NEG, extData.iFmt_VOP_SDWA.SRC1_ABS);

        processSDWA_src(extData.iFmt_VOP_SDWA, src0_sdwa, origSrc0_sdwa, src1,
                        origSrc1);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = src1[lane] << bits(src0_sdwa[lane], 4, 0);
                origVdst[lane] = vdst[lane]; // keep copy consistent
            }
        }

        processSDWA_dst(extData.iFmt_VOP_SDWA, vdst, origVdst);
    } else {
        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = src1[lane] << bits(src0[lane], 4, 0);
            }
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_AND_B32 class methods ---

Inst_VOP2__V_AND_B32::Inst_VOP2__V_AND_B32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_and_b32")
{
    setFlag(ALU);
} // Inst_VOP2__V_AND_B32

Inst_VOP2__V_AND_B32::~Inst_VOP2__V_AND_B32() {} // ~Inst_VOP2__V_AND_B32

// --- description from .arch file ---
// D.u = S0.u & S1.u.
// Input and output modifiers not supported.
void
Inst_VOP2__V_AND_B32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, instData.SRC0);
    VecOperandU32 src1(gpuDynInst, instData.VSRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    if (isDPPInst()) {
        VecOperandU32 src0_dpp(gpuDynInst, extData.iFmt_VOP_DPP.SRC0);
        src0_dpp.read();

        DPRINTF(VEGA,
                "Handling V_AND_B32 SRC DPP. SRC0: register v[%d], "
                "DPP_CTRL: 0x%#x, SRC0_ABS: %d, SRC0_NEG: %d, "
                "SRC1_ABS: %d, SRC1_NEG: %d, BC: %d, "
                "BANK_MASK: %d, ROW_MASK: %d\n",
                extData.iFmt_VOP_DPP.SRC0, extData.iFmt_VOP_DPP.DPP_CTRL,
                extData.iFmt_VOP_DPP.SRC0_ABS, extData.iFmt_VOP_DPP.SRC0_NEG,
                extData.iFmt_VOP_DPP.SRC1_ABS, extData.iFmt_VOP_DPP.SRC1_NEG,
                extData.iFmt_VOP_DPP.BC, extData.iFmt_VOP_DPP.BANK_MASK,
                extData.iFmt_VOP_DPP.ROW_MASK);

        processDPP(gpuDynInst, extData.iFmt_VOP_DPP, src0_dpp, src1);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = src0_dpp[lane] & src1[lane];
            }
        }
    } else {
        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = src0[lane] & src1[lane];
            }
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_OR_B32 class methods ---

Inst_VOP2__V_OR_B32::Inst_VOP2__V_OR_B32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_or_b32")
{
    setFlag(ALU);
} // Inst_VOP2__V_OR_B32

Inst_VOP2__V_OR_B32::~Inst_VOP2__V_OR_B32() {} // ~Inst_VOP2__V_OR_B32

// --- description from .arch file ---
// D.u = S0.u | S1.u.
// Input and output modifiers not supported.
void
Inst_VOP2__V_OR_B32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, instData.SRC0);
    VecOperandU32 src1(gpuDynInst, instData.VSRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    if (isSDWAInst()) {
        VecOperandU32 src0_sdwa(gpuDynInst, extData.iFmt_VOP_SDWA.SRC0);
        // use copies of original src0, src1, and dest during selecting
        VecOperandU32 origSrc0_sdwa(gpuDynInst, extData.iFmt_VOP_SDWA.SRC0);
        VecOperandU32 origSrc1(gpuDynInst, instData.VSRC1);
        VecOperandU32 origVdst(gpuDynInst, instData.VDST);

        src0_sdwa.read();
        origSrc0_sdwa.read();
        origSrc1.read();

        DPRINTF(
            VEGA,
            "Handling V_OR_B32 SRC SDWA. SRC0: register v[%d], "
            "DST_SEL: %d, DST_U: %d, CLMP: %d, SRC0_SEL: %d, "
            "SRC0_SEXT: %d, SRC0_NEG: %d, SRC0_ABS: %d, SRC1_SEL: %d, "
            "SRC1_SEXT: %d, SRC1_NEG: %d, SRC1_ABS: %d\n",
            extData.iFmt_VOP_SDWA.SRC0, extData.iFmt_VOP_SDWA.DST_SEL,
            extData.iFmt_VOP_SDWA.DST_U, extData.iFmt_VOP_SDWA.CLMP,
            extData.iFmt_VOP_SDWA.SRC0_SEL, extData.iFmt_VOP_SDWA.SRC0_SEXT,
            extData.iFmt_VOP_SDWA.SRC0_NEG, extData.iFmt_VOP_SDWA.SRC0_ABS,
            extData.iFmt_VOP_SDWA.SRC1_SEL, extData.iFmt_VOP_SDWA.SRC1_SEXT,
            extData.iFmt_VOP_SDWA.SRC1_NEG, extData.iFmt_VOP_SDWA.SRC1_ABS);

        processSDWA_src(extData.iFmt_VOP_SDWA, src0_sdwa, origSrc0_sdwa, src1,
                        origSrc1);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = src0_sdwa[lane] | src1[lane];
                origVdst[lane] = vdst[lane]; // keep copy consistent
            }
        }

        processSDWA_dst(extData.iFmt_VOP_SDWA, vdst, origVdst);
    } else {
        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = src0[lane] | src1[lane];
            }
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_XOR_B32 class methods ---

Inst_VOP2__V_XOR_B32::Inst_VOP2__V_XOR_B32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_xor_b32")
{
    setFlag(ALU);
} // Inst_VOP2__V_XOR_B32

Inst_VOP2__V_XOR_B32::~Inst_VOP2__V_XOR_B32() {} // ~Inst_VOP2__V_XOR_B32

// --- description from .arch file ---
// D.u = S0.u ^ S1.u.
// Input and output modifiers not supported.
void
Inst_VOP2__V_XOR_B32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, instData.VSRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src0[lane] ^ src1[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_MAC_F32 class methods ---

Inst_VOP2__V_MAC_F32::Inst_VOP2__V_MAC_F32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_mac_f32")
{
    setFlag(ALU);
    setFlag(F32);
    setFlag(MAC);
} // Inst_VOP2__V_MAC_F32

Inst_VOP2__V_MAC_F32::~Inst_VOP2__V_MAC_F32() {} // ~Inst_VOP2__V_MAC_F32

// --- description from .arch file ---
// D.f = S0.f * S1.f + D.f.
// SQ translates to V_MAD_F32.
void
Inst_VOP2__V_MAC_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, instData.SRC0);
    VecOperandF32 src1(gpuDynInst, instData.VSRC1);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();
    vdst.read();

    if (isDPPInst()) {
        VecOperandF32 src0_dpp(gpuDynInst, extData.iFmt_VOP_DPP.SRC0);
        src0_dpp.read();

        DPRINTF(VEGA,
                "Handling V_MAC_F32 SRC DPP. SRC0: register v[%d], "
                "DPP_CTRL: 0x%#x, SRC0_ABS: %d, SRC0_NEG: %d, "
                "SRC1_ABS: %d, SRC1_NEG: %d, BC: %d, "
                "BANK_MASK: %d, ROW_MASK: %d\n",
                extData.iFmt_VOP_DPP.SRC0, extData.iFmt_VOP_DPP.DPP_CTRL,
                extData.iFmt_VOP_DPP.SRC0_ABS, extData.iFmt_VOP_DPP.SRC0_NEG,
                extData.iFmt_VOP_DPP.SRC1_ABS, extData.iFmt_VOP_DPP.SRC1_NEG,
                extData.iFmt_VOP_DPP.BC, extData.iFmt_VOP_DPP.BANK_MASK,
                extData.iFmt_VOP_DPP.ROW_MASK);

        processDPP(gpuDynInst, extData.iFmt_VOP_DPP, src0_dpp, src1);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = std::fma(src0_dpp[lane], src1[lane], vdst[lane]);
            }
        }
    } else {
        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = std::fma(src0[lane], src1[lane], vdst[lane]);
            }
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_MADMK_F32 class methods ---

Inst_VOP2__V_MADMK_F32::Inst_VOP2__V_MADMK_F32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_madmk_f32")
{
    setFlag(ALU);
    setFlag(F32);
    setFlag(MAD);
} // Inst_VOP2__V_MADMK_F32

Inst_VOP2__V_MADMK_F32::~Inst_VOP2__V_MADMK_F32() {} // ~Inst_VOP2__V_MADMK_F32

// --- description from .arch file ---
// D.f = S0.f * K + S1.f; K is a 32-bit inline constant.
// This opcode cannot use the VOP3 encoding and cannot use input/output
// ---  modifiers.
// SQ translates to V_MAD_F32.
void
Inst_VOP2__V_MADMK_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, instData.VSRC1);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);
    VecElemF32 k = extData.imm_f32;

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::fma(src0[lane], k, src1[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_MADAK_F32 class methods ---

Inst_VOP2__V_MADAK_F32::Inst_VOP2__V_MADAK_F32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_madak_f32")
{
    setFlag(ALU);
    setFlag(F32);
    setFlag(MAD);
} // Inst_VOP2__V_MADAK_F32

Inst_VOP2__V_MADAK_F32::~Inst_VOP2__V_MADAK_F32() {} // ~Inst_VOP2__V_MADAK_F32

// --- description from .arch file ---
// D.f = S0.f * S1.f + K; K is a 32-bit inline constant.
// This opcode cannot use the VOP3 encoding and cannot use input/output
// ---  modifiers.
// SQ translates to V_MAD_F32.
void
Inst_VOP2__V_MADAK_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, instData.VSRC1);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);
    VecElemF32 k = extData.imm_f32;

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::fma(src0[lane], src1[lane], k);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_ADD_CO_U32 class methods ---

Inst_VOP2__V_ADD_CO_U32::Inst_VOP2__V_ADD_CO_U32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_add_co_u32")
{
    setFlag(ALU);
    setFlag(WritesVCC);
} // Inst_VOP2__V_ADD_CO_U32

Inst_VOP2__V_ADD_CO_U32::~Inst_VOP2__V_ADD_CO_U32() {
} // ~Inst_VOP2__V_ADD_CO_U32

// --- description from .arch file ---
// D.u = S0.u + S1.u;
// VCC[threadId] = (S0.u + S1.u >= 0x800000000ULL ? 1 : 0) is an UNSIGNED
// ---  overflow or carry-out for V_ADDC_U32.
// In VOP3 the VCC destination may be an arbitrary SGPR-pair.
void
Inst_VOP2__V_ADD_CO_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, instData.SRC0);
    VecOperandU32 src1(gpuDynInst, instData.VSRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);
    ScalarOperandU64 vcc(gpuDynInst, REG_VCC_LO);

    src0.readSrc();
    src1.read();

    if (isSDWAInst()) {
        VecOperandU32 src0_sdwa(gpuDynInst, extData.iFmt_VOP_SDWA.SRC0);
        // use copies of original src0, src1, and dest during selecting
        VecOperandU32 origSrc0_sdwa(gpuDynInst, extData.iFmt_VOP_SDWA.SRC0);
        VecOperandU32 origSrc1(gpuDynInst, instData.VSRC1);
        VecOperandU32 origVdst(gpuDynInst, instData.VDST);

        src0_sdwa.read();
        origSrc0_sdwa.read();
        origSrc1.read();

        DPRINTF(
            VEGA,
            "Handling V_ADD_CO_U32 SRC SDWA. SRC0: register "
            "v[%d], DST_SEL: %d, DST_U: %d, CLMP: %d, SRC0_SEL: %d, "
            "SRC0_SEXT: %d, SRC0_NEG: %d, SRC0_ABS: %d, SRC1_SEL: %d, "
            "SRC1_SEXT: %d, SRC1_NEG: %d, SRC1_ABS: %d\n",
            extData.iFmt_VOP_SDWA.SRC0, extData.iFmt_VOP_SDWA.DST_SEL,
            extData.iFmt_VOP_SDWA.DST_U, extData.iFmt_VOP_SDWA.CLMP,
            extData.iFmt_VOP_SDWA.SRC0_SEL, extData.iFmt_VOP_SDWA.SRC0_SEXT,
            extData.iFmt_VOP_SDWA.SRC0_NEG, extData.iFmt_VOP_SDWA.SRC0_ABS,
            extData.iFmt_VOP_SDWA.SRC1_SEL, extData.iFmt_VOP_SDWA.SRC1_SEXT,
            extData.iFmt_VOP_SDWA.SRC1_NEG, extData.iFmt_VOP_SDWA.SRC1_ABS);

        processSDWA_src(extData.iFmt_VOP_SDWA, src0_sdwa, origSrc0_sdwa, src1,
                        origSrc1);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = src0_sdwa[lane] + src1[lane];
                origVdst[lane] = vdst[lane]; // keep copy consistent
                vcc.setBit(lane, ((VecElemU64)src0_sdwa[lane] +
                                      (VecElemU64)src1[lane] >=
                                  0x100000000ULL) ?
                                     1 :
                                     0);
            }
        }

        processSDWA_dst(extData.iFmt_VOP_SDWA, vdst, origVdst);
    } else {
        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = src0[lane] + src1[lane];
                vcc.setBit(lane,
                           ((VecElemU64)src0[lane] + (VecElemU64)src1[lane] >=
                            0x100000000ULL) ?
                               1 :
                               0);
            }
        }
    }

    vcc.write();
    vdst.write();
} // execute

// --- Inst_VOP2__V_SUB_CO_U32 class methods ---

Inst_VOP2__V_SUB_CO_U32::Inst_VOP2__V_SUB_CO_U32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_sub_co_u32")
{
    setFlag(ALU);
    setFlag(WritesVCC);
} // Inst_VOP2__V_SUB_CO_U32

Inst_VOP2__V_SUB_CO_U32::~Inst_VOP2__V_SUB_CO_U32() {
} // ~Inst_VOP2__V_SUB_CO_U32

// --- description from .arch file ---
// D.u = S0.u - S1.u;
// VCC[threadId] = (S1.u > S0.u ? 1 : 0) is an UNSIGNED overflow or
// carry-out for V_SUBB_U32.
// In VOP3 the VCC destination may be an arbitrary SGPR-pair.
void
Inst_VOP2__V_SUB_CO_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, instData.VSRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);
    ScalarOperandU64 vcc(gpuDynInst, REG_VCC_LO);

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src0[lane] - src1[lane];
            vcc.setBit(lane, src1[lane] > src0[lane] ? 1 : 0);
        }
    }

    vdst.write();
    vcc.write();
} // execute

// --- Inst_VOP2__V_SUBREV_CO_U32 class methods ---

Inst_VOP2__V_SUBREV_CO_U32::Inst_VOP2__V_SUBREV_CO_U32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_subrev_co_u32")
{
    setFlag(ALU);
    setFlag(WritesVCC);
} // Inst_VOP2__V_SUBREV_CO_U32

Inst_VOP2__V_SUBREV_CO_U32::~Inst_VOP2__V_SUBREV_CO_U32() {
} // ~Inst_VOP2__V_SUBREV_CO_U32

// --- description from .arch file ---
// D.u = S1.u - S0.u;
// VCC[threadId] = (S0.u > S1.u ? 1 : 0) is an UNSIGNED overflow or
// carry-out for V_SUBB_U32.
// In VOP3 the VCC destination may be an arbitrary SGPR-pair.
void
Inst_VOP2__V_SUBREV_CO_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, instData.VSRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);
    ScalarOperandU64 vcc(gpuDynInst, REG_VCC_LO);

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src1[lane] - src0[lane];
            vcc.setBit(lane, src0[lane] > src1[lane] ? 1 : 0);
        }
    }

    vdst.write();
    vcc.write();
} // execute

// --- Inst_VOP2__V_ADDC_CO_U32 class methods ---

Inst_VOP2__V_ADDC_CO_U32::Inst_VOP2__V_ADDC_CO_U32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_addc_co_u32")
{
    setFlag(ALU);
    setFlag(WritesVCC);
    setFlag(ReadsVCC);
} // Inst_VOP2__V_ADDC_CO_U32

Inst_VOP2__V_ADDC_CO_U32::~Inst_VOP2__V_ADDC_CO_U32() {
} // ~Inst_VOP2__V_ADDC_CO_U32

// --- description from .arch file ---
// D.u = S0.u + S1.u + VCC[threadId];
// VCC[threadId] = (S0.u + S1.u + VCC[threadId] >= 0x800000000ULL ? 1 : 0)
// is an UNSIGNED overflow.
// In VOP3 the VCC destination may be an arbitrary SGPR-pair, and the VCC
// source comes from the SGPR-pair at S2.u.
void
Inst_VOP2__V_ADDC_CO_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, instData.VSRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);
    ScalarOperandU64 vcc(gpuDynInst, REG_VCC_LO);

    src0.readSrc();
    src1.read();
    vcc.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src0[lane] + src1[lane] + bits(vcc.rawData(), lane);
            vcc.setBit(lane, ((VecElemU64)src0[lane] + (VecElemU64)src1[lane] +
                              (VecElemU64)bits(vcc.rawData(), lane, lane)) >=
                                     0x100000000 ?
                                 1 :
                                 0);
        }
    }

    vdst.write();
    vcc.write();
} // execute

// --- Inst_VOP2__V_SUBB_CO_U32 class methods ---

Inst_VOP2__V_SUBB_CO_U32::Inst_VOP2__V_SUBB_CO_U32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_subb_co_u32")
{
    setFlag(ALU);
    setFlag(WritesVCC);
    setFlag(ReadsVCC);
} // Inst_VOP2__V_SUBB_CO_U32

Inst_VOP2__V_SUBB_CO_U32::~Inst_VOP2__V_SUBB_CO_U32() {
} // ~Inst_VOP2__V_SUBB_CO_U32

// --- description from .arch file ---
// D.u = S0.u - S1.u - VCC[threadId];
// VCC[threadId] = (S1.u + VCC[threadId] > S0.u ? 1 : 0) is an UNSIGNED
// ---  overflow.
// In VOP3 the VCC destination may be an arbitrary SGPR-pair, and the VCC
// ---  source comes from the SGPR-pair at S2.u.
void
Inst_VOP2__V_SUBB_CO_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, instData.VSRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);
    ScalarOperandU64 vcc(gpuDynInst, REG_VCC_LO);

    src0.readSrc();
    src1.read();
    vcc.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src0[lane] - src1[lane] - bits(vcc.rawData(), lane);
            vcc.setBit(
                lane,
                (src1[lane] + bits(vcc.rawData(), lane)) > src0[lane] ? 1 : 0);
        }
    }

    vdst.write();
    vcc.write();
} // execute

// --- Inst_VOP2__V_SUBBREV_CO_U32 class methods ---

Inst_VOP2__V_SUBBREV_CO_U32::Inst_VOP2__V_SUBBREV_CO_U32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_subbrev_co_u32")
{
    setFlag(ALU);
    setFlag(WritesVCC);
    setFlag(ReadsVCC);
} // Inst_VOP2__V_SUBBREV_CO_U32

Inst_VOP2__V_SUBBREV_CO_U32::~Inst_VOP2__V_SUBBREV_CO_U32() {
} // ~Inst_VOP2__V_SUBBREV_CO_U32

// --- description from .arch file ---
// D.u = S1.u - S0.u - VCC[threadId];
// VCC[threadId] = (S1.u + VCC[threadId] > S0.u ? 1 : 0) is an UNSIGNED
// overflow.
// In VOP3 the VCC destination may be an arbitrary SGPR-pair, and the VCC
// source comes from the SGPR-pair at S2.u. SQ translates to V_SUBB_U32.
// SQ translates this to V_SUBREV_U32 with reversed operands.
void
Inst_VOP2__V_SUBBREV_CO_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, instData.VSRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);
    ScalarOperandU64 vcc(gpuDynInst, REG_VCC_LO);

    src0.readSrc();
    src1.read();
    vcc.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src1[lane] - src0[lane] - bits(vcc.rawData(), lane);
            vcc.setBit(
                lane,
                (src0[lane] + bits(vcc.rawData(), lane)) > src1[lane] ? 1 : 0);
        }
    }

    vdst.write();
    vcc.write();
} // execute

// --- Inst_VOP2__V_ADD_F16 class methods ---

Inst_VOP2__V_ADD_F16::Inst_VOP2__V_ADD_F16(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_add_f16")
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP2__V_ADD_F16

Inst_VOP2__V_ADD_F16::~Inst_VOP2__V_ADD_F16() {} // ~Inst_VOP2__V_ADD_F16

// --- description from .arch file ---
// D.f16 = S0.f16 + S1.f16.
// Supports denormals, round mode, exception flags, saturation.
void
Inst_VOP2__V_ADD_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP2__V_SUB_F16 class methods ---

Inst_VOP2__V_SUB_F16::Inst_VOP2__V_SUB_F16(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_sub_f16")
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP2__V_SUB_F16

Inst_VOP2__V_SUB_F16::~Inst_VOP2__V_SUB_F16() {} // ~Inst_VOP2__V_SUB_F16

// --- description from .arch file ---
// D.f16 = S0.f16 - S1.f16.
// Supports denormals, round mode, exception flags, saturation.
// SQ translates to V_ADD_F16.
void
Inst_VOP2__V_SUB_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP2__V_SUBREV_F16 class methods ---

Inst_VOP2__V_SUBREV_F16::Inst_VOP2__V_SUBREV_F16(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_subrev_f16")
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP2__V_SUBREV_F16

Inst_VOP2__V_SUBREV_F16::~Inst_VOP2__V_SUBREV_F16() {
} // ~Inst_VOP2__V_SUBREV_F16

// --- description from .arch file ---
// D.f16 = S1.f16 - S0.f16.
// Supports denormals, round mode, exception flags, saturation.
// SQ translates to V_ADD_F16.
void
Inst_VOP2__V_SUBREV_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP2__V_MUL_F16 class methods ---

Inst_VOP2__V_MUL_F16::Inst_VOP2__V_MUL_F16(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_mul_f16")
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP2__V_MUL_F16

Inst_VOP2__V_MUL_F16::~Inst_VOP2__V_MUL_F16() {} // ~Inst_VOP2__V_MUL_F16

// --- description from .arch file ---
// D.f16 = S0.f16 * S1.f16.
// Supports denormals, round mode, exception flags, saturation.
void
Inst_VOP2__V_MUL_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP2__V_MAC_F16 class methods ---

Inst_VOP2__V_MAC_F16::Inst_VOP2__V_MAC_F16(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_mac_f16")
{
    setFlag(ALU);
    setFlag(F16);
    setFlag(MAC);
} // Inst_VOP2__V_MAC_F16

Inst_VOP2__V_MAC_F16::~Inst_VOP2__V_MAC_F16() {} // ~Inst_VOP2__V_MAC_F16

// --- description from .arch file ---
// D.f16 = S0.f16 * S1.f16 + D.f16.
// Supports round mode, exception flags, saturation.
// SQ translates this to V_MAD_F16.
void
Inst_VOP2__V_MAC_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP2__V_MADMK_F16 class methods ---

Inst_VOP2__V_MADMK_F16::Inst_VOP2__V_MADMK_F16(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_madmk_f16")
{
    setFlag(ALU);
    setFlag(F16);
    setFlag(MAD);
} // Inst_VOP2__V_MADMK_F16

Inst_VOP2__V_MADMK_F16::~Inst_VOP2__V_MADMK_F16() {} // ~Inst_VOP2__V_MADMK_F16

// --- description from .arch file ---
// D.f16 = S0.f16 * K.f16 + S1.f16; K is a 16-bit inline constant stored
// in the following literal DWORD.
// This opcode cannot use the VOP3 encoding and cannot use input/output
// modifiers. Supports round mode, exception flags, saturation.
// SQ translates this to V_MAD_F16.
void
Inst_VOP2__V_MADMK_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP2__V_MADAK_F16 class methods ---

Inst_VOP2__V_MADAK_F16::Inst_VOP2__V_MADAK_F16(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_madak_f16")
{
    setFlag(ALU);
    setFlag(F16);
    setFlag(MAD);
} // Inst_VOP2__V_MADAK_F16

Inst_VOP2__V_MADAK_F16::~Inst_VOP2__V_MADAK_F16() {} // ~Inst_VOP2__V_MADAK_F16

// --- description from .arch file ---
// D.f16 = S0.f16 * S1.f16 + K.f16; K is a 16-bit inline constant stored
// in the following literal DWORD.
// This opcode cannot use the VOP3 encoding and cannot use input/output
// modifiers. Supports round mode, exception flags, saturation.
// SQ translates this to V_MAD_F16.
void
Inst_VOP2__V_MADAK_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP2__V_ADD_U16 class methods ---

Inst_VOP2__V_ADD_U16::Inst_VOP2__V_ADD_U16(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_add_u16")
{
    setFlag(ALU);
} // Inst_VOP2__V_ADD_U16

Inst_VOP2__V_ADD_U16::~Inst_VOP2__V_ADD_U16() {} // ~Inst_VOP2__V_ADD_U16

// --- description from .arch file ---
// D.u16 = S0.u16 + S1.u16.
// Supports saturation (unsigned 16-bit integer domain).
void
Inst_VOP2__V_ADD_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU16 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandU16 src1(gpuDynInst, instData.VSRC1);
    VecOperandU16 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src0[lane] + src1[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_SUB_U16 class methods ---

Inst_VOP2__V_SUB_U16::Inst_VOP2__V_SUB_U16(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_sub_u16")
{
    setFlag(ALU);
} // Inst_VOP2__V_SUB_U16

Inst_VOP2__V_SUB_U16::~Inst_VOP2__V_SUB_U16() {} // ~Inst_VOP2__V_SUB_U16

// --- description from .arch file ---
// D.u16 = S0.u16 - S1.u16.
// Supports saturation (unsigned 16-bit integer domain).
void
Inst_VOP2__V_SUB_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU16 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandU16 src1(gpuDynInst, instData.VSRC1);
    VecOperandU16 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src0[lane] - src1[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_SUBREV_U16 class methods ---

Inst_VOP2__V_SUBREV_U16::Inst_VOP2__V_SUBREV_U16(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_subrev_u16")
{
    setFlag(ALU);
} // Inst_VOP2__V_SUBREV_U16

Inst_VOP2__V_SUBREV_U16::~Inst_VOP2__V_SUBREV_U16() {
} // ~Inst_VOP2__V_SUBREV_U16

// --- description from .arch file ---
// D.u16 = S1.u16 - S0.u16.
// Supports saturation (unsigned 16-bit integer domain).
// SQ translates this to V_SUB_U16 with reversed operands.
void
Inst_VOP2__V_SUBREV_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU16 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandU16 src1(gpuDynInst, instData.VSRC1);
    VecOperandU16 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src1[lane] - src0[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_MUL_LO_U16 class methods ---

Inst_VOP2__V_MUL_LO_U16::Inst_VOP2__V_MUL_LO_U16(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_mul_lo_u16")
{
    setFlag(ALU);
} // Inst_VOP2__V_MUL_LO_U16

Inst_VOP2__V_MUL_LO_U16::~Inst_VOP2__V_MUL_LO_U16() {
} // ~Inst_VOP2__V_MUL_LO_U16

// --- description from .arch file ---
// D.u16 = S0.u16 * S1.u16.
// Supports saturation (unsigned 16-bit integer domain).
void
Inst_VOP2__V_MUL_LO_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU16 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandU16 src1(gpuDynInst, instData.VSRC1);
    VecOperandU16 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src0[lane] * src1[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_LSHLREV_B16 class methods ---

Inst_VOP2__V_LSHLREV_B16::Inst_VOP2__V_LSHLREV_B16(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_lshlrev_b16")
{
    setFlag(ALU);
} // Inst_VOP2__V_LSHLREV_B16

Inst_VOP2__V_LSHLREV_B16::~Inst_VOP2__V_LSHLREV_B16() {
} // ~Inst_VOP2__V_LSHLREV_B16

// --- description from .arch file ---
// D.u[15:0] = S1.u[15:0] << S0.u[3:0].
// SQ translates this to an internal SP opcode.
void
Inst_VOP2__V_LSHLREV_B16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU16 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandU16 src1(gpuDynInst, instData.VSRC1);
    VecOperandU16 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src1[lane] << bits(src0[lane], 3, 0);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_LSHRREV_B16 class methods ---

Inst_VOP2__V_LSHRREV_B16::Inst_VOP2__V_LSHRREV_B16(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_lshrrev_b16")
{
    setFlag(ALU);
} // Inst_VOP2__V_LSHRREV_B16

Inst_VOP2__V_LSHRREV_B16::~Inst_VOP2__V_LSHRREV_B16() {
} // ~Inst_VOP2__V_LSHRREV_B16

// --- description from .arch file ---
// D.u[15:0] = S1.u[15:0] >> S0.u[3:0].
// The vacated bits are set to zero.
// SQ translates this to an internal SP opcode.
void
Inst_VOP2__V_LSHRREV_B16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU16 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandU16 src1(gpuDynInst, instData.VSRC1);
    VecOperandU16 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src1[lane] >> src0[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_ASHRREV_I16 class methods ---

Inst_VOP2__V_ASHRREV_I16::Inst_VOP2__V_ASHRREV_I16(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_ashrrev_i16")
{
    setFlag(ALU);
} // Inst_VOP2__V_ASHRREV_I16

Inst_VOP2__V_ASHRREV_I16::~Inst_VOP2__V_ASHRREV_I16() {
} // ~Inst_VOP2__V_ASHRREV_I16

// --- description from .arch file ---
// D.i[15:0] = signext(S1.i[15:0]) >> S0.i[3:0].
// The vacated bits are set to the sign bit of the input value.
// SQ translates this to an internal SP opcode.
void
Inst_VOP2__V_ASHRREV_I16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU16 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandI16 src1(gpuDynInst, instData.VSRC1);
    VecOperandI16 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src1[lane] >> src0[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_MAX_F16 class methods ---

Inst_VOP2__V_MAX_F16::Inst_VOP2__V_MAX_F16(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_max_f16")
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP2__V_MAX_F16

Inst_VOP2__V_MAX_F16::~Inst_VOP2__V_MAX_F16() {} // ~Inst_VOP2__V_MAX_F16

// --- description from .arch file ---
// D.f16 = max(S0.f16, S1.f16).
// IEEE compliant. Supports denormals, round mode, exception flags,
// saturation.
void
Inst_VOP2__V_MAX_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP2__V_MIN_F16 class methods ---

Inst_VOP2__V_MIN_F16::Inst_VOP2__V_MIN_F16(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_min_f16")
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP2__V_MIN_F16

Inst_VOP2__V_MIN_F16::~Inst_VOP2__V_MIN_F16() {} // ~Inst_VOP2__V_MIN_F16

// --- description from .arch file ---
// D.f16 = min(S0.f16, S1.f16).
// IEEE compliant. Supports denormals, round mode, exception flags,
// saturation.
void
Inst_VOP2__V_MIN_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP2__V_MAX_U16 class methods ---

Inst_VOP2__V_MAX_U16::Inst_VOP2__V_MAX_U16(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_max_u16")
{
    setFlag(ALU);
} // Inst_VOP2__V_MAX_U16

Inst_VOP2__V_MAX_U16::~Inst_VOP2__V_MAX_U16() {} // ~Inst_VOP2__V_MAX_U16

// --- description from .arch file ---
// D.u[15:0] = max(S0.u[15:0], S1.u[15:0]).
void
Inst_VOP2__V_MAX_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU16 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandU16 src1(gpuDynInst, instData.VSRC1);
    VecOperandU16 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::max(src0[lane], src1[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_MAX_I16 class methods ---

Inst_VOP2__V_MAX_I16::Inst_VOP2__V_MAX_I16(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_max_i16")
{
    setFlag(ALU);
} // Inst_VOP2__V_MAX_I16

Inst_VOP2__V_MAX_I16::~Inst_VOP2__V_MAX_I16() {} // ~Inst_VOP2__V_MAX_I16

// --- description from .arch file ---
// D.i[15:0] = max(S0.i[15:0], S1.i[15:0]).
void
Inst_VOP2__V_MAX_I16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI16 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandI16 src1(gpuDynInst, instData.VSRC1);
    VecOperandI16 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::max(src0[lane], src1[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_MIN_U16 class methods ---

Inst_VOP2__V_MIN_U16::Inst_VOP2__V_MIN_U16(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_min_u16")
{
    setFlag(ALU);
} // Inst_VOP2__V_MIN_U16

Inst_VOP2__V_MIN_U16::~Inst_VOP2__V_MIN_U16() {} // ~Inst_VOP2__V_MIN_U16

// --- description from .arch file ---
// D.u[15:0] = min(S0.u[15:0], S1.u[15:0]).
void
Inst_VOP2__V_MIN_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU16 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandU16 src1(gpuDynInst, instData.VSRC1);
    VecOperandU16 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::min(src0[lane], src1[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_MIN_I16 class methods ---

Inst_VOP2__V_MIN_I16::Inst_VOP2__V_MIN_I16(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_min_i16")
{
    setFlag(ALU);
} // Inst_VOP2__V_MIN_I16

Inst_VOP2__V_MIN_I16::~Inst_VOP2__V_MIN_I16() {} // ~Inst_VOP2__V_MIN_I16

// --- description from .arch file ---
// D.i[15:0] = min(S0.i[15:0], S1.i[15:0]).
void
Inst_VOP2__V_MIN_I16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI16 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandI16 src1(gpuDynInst, instData.VSRC1);
    VecOperandI16 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::min(src0[lane], src1[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_LDEXP_F16 class methods ---

Inst_VOP2__V_LDEXP_F16::Inst_VOP2__V_LDEXP_F16(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_ldexp_f16")
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP2__V_LDEXP_F16

Inst_VOP2__V_LDEXP_F16::~Inst_VOP2__V_LDEXP_F16() {} // ~Inst_VOP2__V_LDEXP_F16

// --- description from .arch file ---
// D.f16 = S0.f16 * (2 ** S1.i16).
void
Inst_VOP2__V_LDEXP_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP2__V_ADD_U32 class methods ---

Inst_VOP2__V_ADD_U32::Inst_VOP2__V_ADD_U32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_add_u32")
{
    setFlag(ALU);
} // Inst_VOP2__V_ADD_U32

Inst_VOP2__V_ADD_U32::~Inst_VOP2__V_ADD_U32() {} // ~Inst_VOP2__V_ADD_U32

// --- description from .arch file ---
// D.u = S0.u + S1.u;
void
Inst_VOP2__V_ADD_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, instData.SRC0);
    VecOperandU32 src1(gpuDynInst, instData.VSRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    if (isSDWAInst()) {
        VecOperandU32 src0_sdwa(gpuDynInst, extData.iFmt_VOP_SDWA.SRC0);
        // use copies of original src0, src1, and dest during selecting
        VecOperandU32 origSrc0_sdwa(gpuDynInst, extData.iFmt_VOP_SDWA.SRC0);
        VecOperandU32 origSrc1(gpuDynInst, instData.VSRC1);
        VecOperandU32 origVdst(gpuDynInst, instData.VDST);

        src0_sdwa.read();
        origSrc0_sdwa.read();
        origSrc1.read();

        DPRINTF(
            VEGA,
            "Handling V_ADD_U32 SRC SDWA. SRC0: register v[%d], "
            "DST_SEL: %d, DST_U: %d, CLMP: %d, SRC0_SEL: %d, "
            "SRC0_SEXT: %d, SRC0_NEG: %d, SRC0_ABS: %d, SRC1_SEL: %d, "
            "SRC1_SEXT: %d, SRC1_NEG: %d, SRC1_ABS: %d\n",
            extData.iFmt_VOP_SDWA.SRC0, extData.iFmt_VOP_SDWA.DST_SEL,
            extData.iFmt_VOP_SDWA.DST_U, extData.iFmt_VOP_SDWA.CLMP,
            extData.iFmt_VOP_SDWA.SRC0_SEL, extData.iFmt_VOP_SDWA.SRC0_SEXT,
            extData.iFmt_VOP_SDWA.SRC0_NEG, extData.iFmt_VOP_SDWA.SRC0_ABS,
            extData.iFmt_VOP_SDWA.SRC1_SEL, extData.iFmt_VOP_SDWA.SRC1_SEXT,
            extData.iFmt_VOP_SDWA.SRC1_NEG, extData.iFmt_VOP_SDWA.SRC1_ABS);

        processSDWA_src(extData.iFmt_VOP_SDWA, src0_sdwa, origSrc0_sdwa, src1,
                        origSrc1);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = src0_sdwa[lane] + src1[lane];
                origVdst[lane] = vdst[lane]; // keep copy consistent
            }
        }

        processSDWA_dst(extData.iFmt_VOP_SDWA, vdst, origVdst);
    } else {
        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = src0[lane] + src1[lane];
            }
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_SUB_U32 class methods ---

Inst_VOP2__V_SUB_U32::Inst_VOP2__V_SUB_U32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_sub_u32")
{
    setFlag(ALU);
} // Inst_VOP2__V_SUB_U32

Inst_VOP2__V_SUB_U32::~Inst_VOP2__V_SUB_U32() {} // ~Inst_VOP2__V_SUB_U32

// --- description from .arch file ---
// D.u = S0.u - S1.u;
void
Inst_VOP2__V_SUB_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, instData.VSRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src0[lane] - src1[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_SUBREV_U32 class methods ---

Inst_VOP2__V_SUBREV_U32::Inst_VOP2__V_SUBREV_U32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_subrev_u32")
{
    setFlag(ALU);
} // Inst_VOP2__V_SUBREV_U32

Inst_VOP2__V_SUBREV_U32::~Inst_VOP2__V_SUBREV_U32() {
} // ~Inst_VOP2__V_SUBREV_U32

// --- description from .arch file ---
// D.u = S1.u - S0.u;
void
Inst_VOP2__V_SUBREV_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, instData.VSRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = src1[lane] - src0[lane];
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_FMAC_F32 class methods ---

Inst_VOP2__V_FMAC_F32::Inst_VOP2__V_FMAC_F32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_fmac_f32")
{
    setFlag(ALU);
} // Inst_VOP2__V_FMAC_F32

Inst_VOP2__V_FMAC_F32::~Inst_VOP2__V_FMAC_F32() {} // ~Inst_VOP2__V_FMAC_F32

// --- description from .arch file ---
// D.u = S1.u - S0.u;
void
Inst_VOP2__V_FMAC_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, instData.VSRC1);
    VecOperandF32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();
    vdst.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = std::fma(src0[lane], src1[lane], vdst[lane]);
        }
    }

    vdst.write();
} // execute

// --- Inst_VOP2__V_XNOR_B32 class methods ---

Inst_VOP2__V_XNOR_B32::Inst_VOP2__V_XNOR_B32(InFmt_VOP2 *iFmt)
    : Inst_VOP2(iFmt, "v_xnor_b32")
{
    setFlag(ALU);
} // Inst_VOP2__V_XNOR_B32

Inst_VOP2__V_XNOR_B32::~Inst_VOP2__V_XNOR_B32() {} // ~Inst_VOP2__V_XNOR_B32

// --- description from .arch file ---
// D.u = S1.u - S0.u;
void
Inst_VOP2__V_XNOR_B32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, instData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, instData.VSRC1);
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.read();
    vdst.read();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            vdst[lane] = ~(src0[lane] ^ src1[lane]);
        }
    }

    vdst.write();
} // execute
} // namespace VegaISA
} // namespace gem5
