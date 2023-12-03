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
// --- Inst_VOP3__V_CMP_CLASS_F32 class methods ---

Inst_VOP3__V_CMP_CLASS_F32::Inst_VOP3__V_CMP_CLASS_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_class_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CMP_CLASS_F32

Inst_VOP3__V_CMP_CLASS_F32::~Inst_VOP3__V_CMP_CLASS_F32() {
} // ~Inst_VOP3__V_CMP_CLASS_F32

// --- description from .arch file ---
// VCC = IEEE numeric class function specified in S1.u, performed on S0.f
// The function reports true if the floating point value is *any* of the
// ---  numeric types selected in S1.u according to the following list:
// S1.u[0] -- value is a signaling NaN.
// S1.u[1] -- value is a quiet NaN.
// S1.u[2] -- value is negative infinity.
// S1.u[3] -- value is a negative normal value.
// S1.u[4] -- value is a negative denormal value.
// S1.u[5] -- value is negative zero.
// S1.u[6] -- value is positive zero.
// S1.u[7] -- value is a positive denormal value.
// S1.u[8] -- value is a positive normal value.
// S1.u[9] -- value is positive infinity.
void
Inst_VOP3__V_CMP_CLASS_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            if (bits(src1[lane], 0) || bits(src1[lane], 1)) {
                // is NaN
                if (std::isnan(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 2)) {
                // is -infinity
                if (std::isinf(src0[lane]) && std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 3)) {
                // is -normal
                if (std::isnormal(src0[lane]) && std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 4)) {
                // is -denormal
                if (std::fpclassify(src0[lane]) == FP_SUBNORMAL &&
                    std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 5)) {
                // is -zero
                if (std::fpclassify(src0[lane]) == FP_ZERO &&
                    std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 6)) {
                // is +zero
                if (std::fpclassify(src0[lane]) == FP_ZERO &&
                    !std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 7)) {
                // is +denormal
                if (std::fpclassify(src0[lane]) == FP_SUBNORMAL &&
                    !std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 8)) {
                // is +normal
                if (std::isnormal(src0[lane]) && !std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 9)) {
                // is +infinity
                if (std::isinf(src0[lane]) && !std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_CLASS_F32 class methods ---

Inst_VOP3__V_CMPX_CLASS_F32::Inst_VOP3__V_CMPX_CLASS_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_class_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_CLASS_F32

Inst_VOP3__V_CMPX_CLASS_F32::~Inst_VOP3__V_CMPX_CLASS_F32() {
} // ~Inst_VOP3__V_CMPX_CLASS_F32

// --- description from .arch file ---
// EXEC, VCC = IEEE numeric class function specified in S1.u, performed on
// S0.f
// The function reports true if the floating point value is *any* of the
// numeric types selected in S1.u according to the following list:
// S1.u[0] -- value is a signaling NaN.
// S1.u[1] -- value is a quiet NaN.
// S1.u[2] -- value is negative infinity.
// S1.u[3] -- value is a negative normal value.
// S1.u[4] -- value is a negative denormal value.
// S1.u[5] -- value is negative zero.
// S1.u[6] -- value is positive zero.
// S1.u[7] -- value is a positive denormal value.
// S1.u[8] -- value is a positive normal value.
// S1.u[9] -- value is positive infinity.
void
Inst_VOP3__V_CMPX_CLASS_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            if (bits(src1[lane], 0) || bits(src1[lane], 1)) {
                // is NaN
                if (std::isnan(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 2)) {
                // is -infinity
                if (std::isinf(src0[lane]) && std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 3)) {
                // is -normal
                if (std::isnormal(src0[lane]) && std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 4)) {
                // is -denormal
                if (std::fpclassify(src0[lane]) == FP_SUBNORMAL &&
                    std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 5)) {
                // is -zero
                if (std::fpclassify(src0[lane]) == FP_ZERO &&
                    std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 6)) {
                // is +zero
                if (std::fpclassify(src0[lane]) == FP_ZERO &&
                    !std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 7)) {
                // is +denormal
                if (std::fpclassify(src0[lane]) == FP_SUBNORMAL &&
                    !std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 8)) {
                // is +normal
                if (std::isnormal(src0[lane]) && !std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 9)) {
                // is +infinity
                if (std::isinf(src0[lane]) && !std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_CLASS_F64 class methods ---

Inst_VOP3__V_CMP_CLASS_F64::Inst_VOP3__V_CMP_CLASS_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_class_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_CMP_CLASS_F64

Inst_VOP3__V_CMP_CLASS_F64::~Inst_VOP3__V_CMP_CLASS_F64() {
} // ~Inst_VOP3__V_CMP_CLASS_F64

// --- description from .arch file ---
// VCC = IEEE numeric class function specified in S1.u, performed on S0.d
// The function reports true if the floating point value is *any* of the
// ---  numeric types selected in S1.u according to the following list:
// S1.u[0] -- value is a signaling NaN.
// S1.u[1] -- value is a quiet NaN.
// S1.u[2] -- value is negative infinity.
// S1.u[3] -- value is a negative normal value.
// S1.u[4] -- value is a negative denormal value.
// S1.u[5] -- value is negative zero.
// S1.u[6] -- value is positive zero.
// S1.u[7] -- value is a positive denormal value.
// S1.u[8] -- value is a positive normal value.
// S1.u[9] -- value is positive infinity.
void
Inst_VOP3__V_CMP_CLASS_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            if (bits(src1[lane], 0) || bits(src1[lane], 1)) {
                // is NaN
                if (std::isnan(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 2)) {
                // is -infinity
                if (std::isinf(src0[lane]) && std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 3)) {
                // is -normal
                if (std::isnormal(src0[lane]) && std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 4)) {
                // is -denormal
                if (std::fpclassify(src0[lane]) == FP_SUBNORMAL &&
                    std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 5)) {
                // is -zero
                if (std::fpclassify(src0[lane]) == FP_ZERO &&
                    std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 6)) {
                // is +zero
                if (std::fpclassify(src0[lane]) == FP_ZERO &&
                    !std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 7)) {
                // is +denormal
                if (std::fpclassify(src0[lane]) == FP_SUBNORMAL &&
                    !std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 8)) {
                // is +normal
                if (std::isnormal(src0[lane]) && !std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 9)) {
                // is +infinity
                if (std::isinf(src0[lane]) && !std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_CLASS_F64 class methods ---

Inst_VOP3__V_CMPX_CLASS_F64::Inst_VOP3__V_CMPX_CLASS_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_class_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_CLASS_F64

Inst_VOP3__V_CMPX_CLASS_F64::~Inst_VOP3__V_CMPX_CLASS_F64() {
} // ~Inst_VOP3__V_CMPX_CLASS_F64

// --- description from .arch file ---
// EXEC, VCC = IEEE numeric class function specified in S1.u, performed on
// S0.d
// The function reports true if the floating point value is *any* of the
// numeric types selected in S1.u according to the following list:
// S1.u[0] -- value is a signaling NaN.
// S1.u[1] -- value is a quiet NaN.
// S1.u[2] -- value is negative infinity.
// S1.u[3] -- value is a negative normal value.
// S1.u[4] -- value is a negative denormal value.
// S1.u[5] -- value is negative zero.
// S1.u[6] -- value is positive zero.
// S1.u[7] -- value is a positive denormal value.
// S1.u[8] -- value is a positive normal value.
// S1.u[9] -- value is positive infinity.
void
Inst_VOP3__V_CMPX_CLASS_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            if (bits(src1[lane], 0) || bits(src1[lane], 1)) {
                // is NaN
                if (std::isnan(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 2)) {
                // is -infinity
                if (std::isinf(src0[lane]) && std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 3)) {
                // is -normal
                if (std::isnormal(src0[lane]) && std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 4)) {
                // is -denormal
                if (std::fpclassify(src0[lane]) == FP_SUBNORMAL &&
                    std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 5)) {
                // is -zero
                if (std::fpclassify(src0[lane]) == FP_ZERO &&
                    std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 6)) {
                // is +zero
                if (std::fpclassify(src0[lane]) == FP_ZERO &&
                    !std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 7)) {
                // is +denormal
                if (std::fpclassify(src0[lane]) == FP_SUBNORMAL &&
                    !std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 8)) {
                // is +normal
                if (std::isnormal(src0[lane]) && !std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
            if (bits(src1[lane], 9)) {
                // is +infinity
                if (std::isinf(src0[lane]) && !std::signbit(src0[lane])) {
                    sdst.setBit(lane, 1);
                    continue;
                }
            }
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_CLASS_F16 class methods ---

Inst_VOP3__V_CMP_CLASS_F16::Inst_VOP3__V_CMP_CLASS_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_class_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_CMP_CLASS_F16

Inst_VOP3__V_CMP_CLASS_F16::~Inst_VOP3__V_CMP_CLASS_F16() {
} // ~Inst_VOP3__V_CMP_CLASS_F16

// --- description from .arch file ---
// VCC = IEEE numeric class function specified in S1.u, performed on S0.f16
// The function reports true if the floating point value is *any* of the
// ---  numeric types selected in S1.u according to the following list:
// S1.u[0] -- value is a signaling NaN.
// S1.u[1] -- value is a quiet NaN.
// S1.u[2] -- value is negative infinity.
// S1.u[3] -- value is a negative normal value.
// S1.u[4] -- value is a negative denormal value.
// S1.u[5] -- value is negative zero.
// S1.u[6] -- value is positive zero.
// S1.u[7] -- value is a positive denormal value.
// S1.u[8] -- value is a positive normal value.
// S1.u[9] -- value is positive infinity.
void
Inst_VOP3__V_CMP_CLASS_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMPX_CLASS_F16 class methods ---

Inst_VOP3__V_CMPX_CLASS_F16::Inst_VOP3__V_CMPX_CLASS_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_class_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_CLASS_F16

Inst_VOP3__V_CMPX_CLASS_F16::~Inst_VOP3__V_CMPX_CLASS_F16() {
} // ~Inst_VOP3__V_CMPX_CLASS_F16

// --- description from .arch file ---
// EXEC, VCC = IEEE numeric class function specified in S1.u, performed on
// ---  S0.f16
// The function reports true if the floating point value is *any* of the
// ---  numeric types selected in S1.u according to the following list:
// S1.u[0] -- value is a signaling NaN.
// S1.u[1] -- value is a quiet NaN.
// S1.u[2] -- value is negative infinity.
// S1.u[3] -- value is a negative normal value.
// S1.u[4] -- value is a negative denormal value.
// S1.u[5] -- value is negative zero.
// S1.u[6] -- value is positive zero.
// S1.u[7] -- value is a positive denormal value.
// S1.u[8] -- value is a positive normal value.
// S1.u[9] -- value is positive infinity.
void
Inst_VOP3__V_CMPX_CLASS_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMP_F_F16 class methods ---

Inst_VOP3__V_CMP_F_F16::Inst_VOP3__V_CMP_F_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_f_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_CMP_F_F16

Inst_VOP3__V_CMP_F_F16::~Inst_VOP3__V_CMP_F_F16() {} // ~Inst_VOP3__V_CMP_F_F16

// --- description from .arch file ---
// D.u64[threadID] = 0; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_F_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMP_LT_F16 class methods ---

Inst_VOP3__V_CMP_LT_F16::Inst_VOP3__V_CMP_LT_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_lt_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_CMP_LT_F16

Inst_VOP3__V_CMP_LT_F16::~Inst_VOP3__V_CMP_LT_F16() {
} // ~Inst_VOP3__V_CMP_LT_F16

// --- description from .arch file ---
// D.u64[threadID] = (S0 < S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_LT_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMP_EQ_F16 class methods ---

Inst_VOP3__V_CMP_EQ_F16::Inst_VOP3__V_CMP_EQ_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_eq_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_CMP_EQ_F16

Inst_VOP3__V_CMP_EQ_F16::~Inst_VOP3__V_CMP_EQ_F16() {
} // ~Inst_VOP3__V_CMP_EQ_F16

// --- description from .arch file ---
// D.u64[threadID] = (S0 == S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_EQ_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMP_LE_F16 class methods ---

Inst_VOP3__V_CMP_LE_F16::Inst_VOP3__V_CMP_LE_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_le_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_CMP_LE_F16

Inst_VOP3__V_CMP_LE_F16::~Inst_VOP3__V_CMP_LE_F16() {
} // ~Inst_VOP3__V_CMP_LE_F16

// --- description from .arch file ---
// D.u64[threadID] = (S0 <= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_LE_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMP_GT_F16 class methods ---

Inst_VOP3__V_CMP_GT_F16::Inst_VOP3__V_CMP_GT_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_gt_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_CMP_GT_F16

Inst_VOP3__V_CMP_GT_F16::~Inst_VOP3__V_CMP_GT_F16() {
} // ~Inst_VOP3__V_CMP_GT_F16

// --- description from .arch file ---
// D.u64[threadID] = (S0 > S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_GT_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMP_LG_F16 class methods ---

Inst_VOP3__V_CMP_LG_F16::Inst_VOP3__V_CMP_LG_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_lg_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_CMP_LG_F16

Inst_VOP3__V_CMP_LG_F16::~Inst_VOP3__V_CMP_LG_F16() {
} // ~Inst_VOP3__V_CMP_LG_F16

// --- description from .arch file ---
// D.u64[threadID] = (S0 <> S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_LG_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMP_GE_F16 class methods ---

Inst_VOP3__V_CMP_GE_F16::Inst_VOP3__V_CMP_GE_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_ge_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_CMP_GE_F16

Inst_VOP3__V_CMP_GE_F16::~Inst_VOP3__V_CMP_GE_F16() {
} // ~Inst_VOP3__V_CMP_GE_F16

// --- description from .arch file ---
// D.u64[threadID] = (S0 >= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_GE_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMP_O_F16 class methods ---

Inst_VOP3__V_CMP_O_F16::Inst_VOP3__V_CMP_O_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_o_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_CMP_O_F16

Inst_VOP3__V_CMP_O_F16::~Inst_VOP3__V_CMP_O_F16() {} // ~Inst_VOP3__V_CMP_O_F16

// --- description from .arch file ---
// D.u64[threadID] = (!isNan(S0) && !isNan(S1)); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_O_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMP_U_F16 class methods ---

Inst_VOP3__V_CMP_U_F16::Inst_VOP3__V_CMP_U_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_u_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_CMP_U_F16

Inst_VOP3__V_CMP_U_F16::~Inst_VOP3__V_CMP_U_F16() {} // ~Inst_VOP3__V_CMP_U_F16

// --- description from .arch file ---
// D.u64[threadID] = (isNan(S0)  ||  isNan(S1)); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_U_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMP_NGE_F16 class methods ---

Inst_VOP3__V_CMP_NGE_F16::Inst_VOP3__V_CMP_NGE_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_nge_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_CMP_NGE_F16

Inst_VOP3__V_CMP_NGE_F16::~Inst_VOP3__V_CMP_NGE_F16() {
} // ~Inst_VOP3__V_CMP_NGE_F16

// --- description from .arch file ---
// D.u64[threadID] = !(S0 >= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_NGE_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMP_NLG_F16 class methods ---

Inst_VOP3__V_CMP_NLG_F16::Inst_VOP3__V_CMP_NLG_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_nlg_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_CMP_NLG_F16

Inst_VOP3__V_CMP_NLG_F16::~Inst_VOP3__V_CMP_NLG_F16() {
} // ~Inst_VOP3__V_CMP_NLG_F16

// --- description from .arch file ---
// D.u64[threadID] = !(S0 <> S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_NLG_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMP_NGT_F16 class methods ---

Inst_VOP3__V_CMP_NGT_F16::Inst_VOP3__V_CMP_NGT_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_ngt_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_CMP_NGT_F16

Inst_VOP3__V_CMP_NGT_F16::~Inst_VOP3__V_CMP_NGT_F16() {
} // ~Inst_VOP3__V_CMP_NGT_F16

// --- description from .arch file ---
// D.u64[threadID] = !(S0 > S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_NGT_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMP_NLE_F16 class methods ---

Inst_VOP3__V_CMP_NLE_F16::Inst_VOP3__V_CMP_NLE_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_nle_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_CMP_NLE_F16

Inst_VOP3__V_CMP_NLE_F16::~Inst_VOP3__V_CMP_NLE_F16() {
} // ~Inst_VOP3__V_CMP_NLE_F16

// --- description from .arch file ---
// D.u64[threadID] = !(S0 <= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_NLE_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMP_NEQ_F16 class methods ---

Inst_VOP3__V_CMP_NEQ_F16::Inst_VOP3__V_CMP_NEQ_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_neq_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_CMP_NEQ_F16

Inst_VOP3__V_CMP_NEQ_F16::~Inst_VOP3__V_CMP_NEQ_F16() {
} // ~Inst_VOP3__V_CMP_NEQ_F16

// --- description from .arch file ---
// D.u64[threadID] = !(S0 == S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_NEQ_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMP_NLT_F16 class methods ---

Inst_VOP3__V_CMP_NLT_F16::Inst_VOP3__V_CMP_NLT_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_nlt_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_CMP_NLT_F16

Inst_VOP3__V_CMP_NLT_F16::~Inst_VOP3__V_CMP_NLT_F16() {
} // ~Inst_VOP3__V_CMP_NLT_F16

// --- description from .arch file ---
// D.u64[threadID] = !(S0 < S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_NLT_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMP_TRU_F16 class methods ---

Inst_VOP3__V_CMP_TRU_F16::Inst_VOP3__V_CMP_TRU_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_tru_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
} // Inst_VOP3__V_CMP_TRU_F16

Inst_VOP3__V_CMP_TRU_F16::~Inst_VOP3__V_CMP_TRU_F16() {
} // ~Inst_VOP3__V_CMP_TRU_F16

// --- description from .arch file ---
// D.u64[threadID] = 1; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_TRU_F16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 1);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_F_F16 class methods ---

Inst_VOP3__V_CMPX_F_F16::Inst_VOP3__V_CMPX_F_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_f_f16", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_F_F16

Inst_VOP3__V_CMPX_F_F16::~Inst_VOP3__V_CMPX_F_F16() {
} // ~Inst_VOP3__V_CMPX_F_F16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = 0; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_F_F16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_LT_F16 class methods ---

Inst_VOP3__V_CMPX_LT_F16::Inst_VOP3__V_CMPX_LT_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_lt_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_LT_F16

Inst_VOP3__V_CMPX_LT_F16::~Inst_VOP3__V_CMPX_LT_F16() {
} // ~Inst_VOP3__V_CMPX_LT_F16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 < S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_LT_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMPX_EQ_F16 class methods ---

Inst_VOP3__V_CMPX_EQ_F16::Inst_VOP3__V_CMPX_EQ_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_eq_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_EQ_F16

Inst_VOP3__V_CMPX_EQ_F16::~Inst_VOP3__V_CMPX_EQ_F16() {
} // ~Inst_VOP3__V_CMPX_EQ_F16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 == S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_EQ_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMPX_LE_F16 class methods ---

Inst_VOP3__V_CMPX_LE_F16::Inst_VOP3__V_CMPX_LE_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_le_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_LE_F16

Inst_VOP3__V_CMPX_LE_F16::~Inst_VOP3__V_CMPX_LE_F16() {
} // ~Inst_VOP3__V_CMPX_LE_F16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 <= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_LE_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMPX_GT_F16 class methods ---

Inst_VOP3__V_CMPX_GT_F16::Inst_VOP3__V_CMPX_GT_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_gt_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_GT_F16

Inst_VOP3__V_CMPX_GT_F16::~Inst_VOP3__V_CMPX_GT_F16() {
} // ~Inst_VOP3__V_CMPX_GT_F16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 > S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_GT_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMPX_LG_F16 class methods ---

Inst_VOP3__V_CMPX_LG_F16::Inst_VOP3__V_CMPX_LG_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_lg_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_LG_F16

Inst_VOP3__V_CMPX_LG_F16::~Inst_VOP3__V_CMPX_LG_F16() {
} // ~Inst_VOP3__V_CMPX_LG_F16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 <> S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_LG_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMPX_GE_F16 class methods ---

Inst_VOP3__V_CMPX_GE_F16::Inst_VOP3__V_CMPX_GE_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_ge_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_GE_F16

Inst_VOP3__V_CMPX_GE_F16::~Inst_VOP3__V_CMPX_GE_F16() {
} // ~Inst_VOP3__V_CMPX_GE_F16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 >= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_GE_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMPX_O_F16 class methods ---

Inst_VOP3__V_CMPX_O_F16::Inst_VOP3__V_CMPX_O_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_o_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_O_F16

Inst_VOP3__V_CMPX_O_F16::~Inst_VOP3__V_CMPX_O_F16() {
} // ~Inst_VOP3__V_CMPX_O_F16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (!isNan(S0) && !isNan(S1)); D = VCC in VOPC
// encoding.
void
Inst_VOP3__V_CMPX_O_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMPX_U_F16 class methods ---

Inst_VOP3__V_CMPX_U_F16::Inst_VOP3__V_CMPX_U_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_u_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_U_F16

Inst_VOP3__V_CMPX_U_F16::~Inst_VOP3__V_CMPX_U_F16() {
} // ~Inst_VOP3__V_CMPX_U_F16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (isNan(S0)  ||  isNan(S1)); D = VCC in VOPC
// encoding.
void
Inst_VOP3__V_CMPX_U_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMPX_NGE_F16 class methods ---

Inst_VOP3__V_CMPX_NGE_F16::Inst_VOP3__V_CMPX_NGE_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_nge_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_NGE_F16

Inst_VOP3__V_CMPX_NGE_F16::~Inst_VOP3__V_CMPX_NGE_F16() {
} // ~Inst_VOP3__V_CMPX_NGE_F16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = !(S0 >= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_NGE_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMPX_NLG_F16 class methods ---

Inst_VOP3__V_CMPX_NLG_F16::Inst_VOP3__V_CMPX_NLG_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_nlg_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_NLG_F16

Inst_VOP3__V_CMPX_NLG_F16::~Inst_VOP3__V_CMPX_NLG_F16() {
} // ~Inst_VOP3__V_CMPX_NLG_F16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = !(S0 <> S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_NLG_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMPX_NGT_F16 class methods ---

Inst_VOP3__V_CMPX_NGT_F16::Inst_VOP3__V_CMPX_NGT_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_ngt_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_NGT_F16

Inst_VOP3__V_CMPX_NGT_F16::~Inst_VOP3__V_CMPX_NGT_F16() {
} // ~Inst_VOP3__V_CMPX_NGT_F16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = !(S0 > S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_NGT_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMPX_NLE_F16 class methods ---

Inst_VOP3__V_CMPX_NLE_F16::Inst_VOP3__V_CMPX_NLE_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_nle_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_NLE_F16

Inst_VOP3__V_CMPX_NLE_F16::~Inst_VOP3__V_CMPX_NLE_F16() {
} // ~Inst_VOP3__V_CMPX_NLE_F16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = !(S0 <= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_NLE_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMPX_NEQ_F16 class methods ---

Inst_VOP3__V_CMPX_NEQ_F16::Inst_VOP3__V_CMPX_NEQ_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_neq_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_NEQ_F16

Inst_VOP3__V_CMPX_NEQ_F16::~Inst_VOP3__V_CMPX_NEQ_F16() {
} // ~Inst_VOP3__V_CMPX_NEQ_F16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = !(S0 == S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_NEQ_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMPX_NLT_F16 class methods ---

Inst_VOP3__V_CMPX_NLT_F16::Inst_VOP3__V_CMPX_NLT_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_nlt_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_NLT_F16

Inst_VOP3__V_CMPX_NLT_F16::~Inst_VOP3__V_CMPX_NLT_F16() {
} // ~Inst_VOP3__V_CMPX_NLT_F16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = !(S0 < S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_NLT_F16::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_VOP3__V_CMPX_TRU_F16 class methods ---

Inst_VOP3__V_CMPX_TRU_F16::Inst_VOP3__V_CMPX_TRU_F16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_tru_f16", true)
{
    setFlag(ALU);
    setFlag(F16);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_TRU_F16

Inst_VOP3__V_CMPX_TRU_F16::~Inst_VOP3__V_CMPX_TRU_F16() {
} // ~Inst_VOP3__V_CMPX_TRU_F16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = 1; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_TRU_F16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 1);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_F_F32 class methods ---

Inst_VOP3__V_CMP_F_F32::Inst_VOP3__V_CMP_F_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_f_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CMP_F_F32

Inst_VOP3__V_CMP_F_F32::~Inst_VOP3__V_CMP_F_F32() {} // ~Inst_VOP3__V_CMP_F_F32

// --- description from .arch file ---
// D.u64[threadID] = 0; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_F_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_LT_F32 class methods ---

Inst_VOP3__V_CMP_LT_F32::Inst_VOP3__V_CMP_LT_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_lt_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CMP_LT_F32

Inst_VOP3__V_CMP_LT_F32::~Inst_VOP3__V_CMP_LT_F32() {
} // ~Inst_VOP3__V_CMP_LT_F32

// --- description from .arch file ---
// D.u64[threadID] = (S0 < S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_LT_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, src0[lane] < src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_EQ_F32 class methods ---

Inst_VOP3__V_CMP_EQ_F32::Inst_VOP3__V_CMP_EQ_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_eq_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CMP_EQ_F32

Inst_VOP3__V_CMP_EQ_F32::~Inst_VOP3__V_CMP_EQ_F32() {
} // ~Inst_VOP3__V_CMP_EQ_F32

// --- description from .arch file ---
// D.u64[threadID] = (S0 == S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_EQ_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, src0[lane] == src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_LE_F32 class methods ---

Inst_VOP3__V_CMP_LE_F32::Inst_VOP3__V_CMP_LE_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_le_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CMP_LE_F32

Inst_VOP3__V_CMP_LE_F32::~Inst_VOP3__V_CMP_LE_F32() {
} // ~Inst_VOP3__V_CMP_LE_F32

// --- description from .arch file ---
// D.u64[threadID] = (S0 <= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_LE_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, src0[lane] <= src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_GT_F32 class methods ---

Inst_VOP3__V_CMP_GT_F32::Inst_VOP3__V_CMP_GT_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_gt_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CMP_GT_F32

Inst_VOP3__V_CMP_GT_F32::~Inst_VOP3__V_CMP_GT_F32() {
} // ~Inst_VOP3__V_CMP_GT_F32

// --- description from .arch file ---
// D.u64[threadID] = (S0 > S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_GT_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, src0[lane] > src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_LG_F32 class methods ---

Inst_VOP3__V_CMP_LG_F32::Inst_VOP3__V_CMP_LG_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_lg_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CMP_LG_F32

Inst_VOP3__V_CMP_LG_F32::~Inst_VOP3__V_CMP_LG_F32() {
} // ~Inst_VOP3__V_CMP_LG_F32

// --- description from .arch file ---
// D.u64[threadID] = (S0 <> S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_LG_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, src0[lane] != src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_GE_F32 class methods ---

Inst_VOP3__V_CMP_GE_F32::Inst_VOP3__V_CMP_GE_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_ge_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CMP_GE_F32

Inst_VOP3__V_CMP_GE_F32::~Inst_VOP3__V_CMP_GE_F32() {
} // ~Inst_VOP3__V_CMP_GE_F32

// --- description from .arch file ---
// D.u64[threadID] = (S0 >= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_GE_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, src0[lane] >= src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_O_F32 class methods ---

Inst_VOP3__V_CMP_O_F32::Inst_VOP3__V_CMP_O_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_o_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CMP_O_F32

Inst_VOP3__V_CMP_O_F32::~Inst_VOP3__V_CMP_O_F32() {} // ~Inst_VOP3__V_CMP_O_F32

// --- description from .arch file ---
// D.u64[threadID] = (!isNan(S0) && !isNan(S1)); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_O_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(
                lane,
                (!std::isnan(src0[lane]) && !std::isnan(src1[lane])) ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_U_F32 class methods ---

Inst_VOP3__V_CMP_U_F32::Inst_VOP3__V_CMP_U_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_u_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CMP_U_F32

Inst_VOP3__V_CMP_U_F32::~Inst_VOP3__V_CMP_U_F32() {} // ~Inst_VOP3__V_CMP_U_F32

// --- description from .arch file ---
// D.u64[threadID] = (isNan(S0)  ||  isNan(S1)); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_U_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(
                lane,
                (std::isnan(src0[lane]) || std::isnan(src1[lane])) ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_NGE_F32 class methods ---

Inst_VOP3__V_CMP_NGE_F32::Inst_VOP3__V_CMP_NGE_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_nge_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CMP_NGE_F32

Inst_VOP3__V_CMP_NGE_F32::~Inst_VOP3__V_CMP_NGE_F32() {
} // ~Inst_VOP3__V_CMP_NGE_F32

// --- description from .arch file ---
// D.u64[threadID] = !(S0 >= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_NGE_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, !(src0[lane] >= src1[lane]) ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_NLG_F32 class methods ---

Inst_VOP3__V_CMP_NLG_F32::Inst_VOP3__V_CMP_NLG_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_nlg_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CMP_NLG_F32

Inst_VOP3__V_CMP_NLG_F32::~Inst_VOP3__V_CMP_NLG_F32() {
} // ~Inst_VOP3__V_CMP_NLG_F32

// --- description from .arch file ---
// D.u64[threadID] = !(S0 <> S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_NLG_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(
                lane,
                !(src0[lane] < src1[lane] || src0[lane] > src1[lane]) ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_NGT_F32 class methods ---

Inst_VOP3__V_CMP_NGT_F32::Inst_VOP3__V_CMP_NGT_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_ngt_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CMP_NGT_F32

Inst_VOP3__V_CMP_NGT_F32::~Inst_VOP3__V_CMP_NGT_F32() {
} // ~Inst_VOP3__V_CMP_NGT_F32

// --- description from .arch file ---
// D.u64[threadID] = !(S0 > S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_NGT_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, !(src0[lane] > src1[lane]) ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_NLE_F32 class methods ---

Inst_VOP3__V_CMP_NLE_F32::Inst_VOP3__V_CMP_NLE_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_nle_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CMP_NLE_F32

Inst_VOP3__V_CMP_NLE_F32::~Inst_VOP3__V_CMP_NLE_F32() {
} // ~Inst_VOP3__V_CMP_NLE_F32

// --- description from .arch file ---
// D.u64[threadID] = !(S0 <= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_NLE_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, !(src0[lane] <= src1[lane]) ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_NEQ_F32 class methods ---

Inst_VOP3__V_CMP_NEQ_F32::Inst_VOP3__V_CMP_NEQ_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_neq_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CMP_NEQ_F32

Inst_VOP3__V_CMP_NEQ_F32::~Inst_VOP3__V_CMP_NEQ_F32() {
} // ~Inst_VOP3__V_CMP_NEQ_F32

// --- description from .arch file ---
// D.u64[threadID] = !(S0 == S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_NEQ_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, src0[lane] != src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_NLT_F32 class methods ---

Inst_VOP3__V_CMP_NLT_F32::Inst_VOP3__V_CMP_NLT_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_nlt_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CMP_NLT_F32

Inst_VOP3__V_CMP_NLT_F32::~Inst_VOP3__V_CMP_NLT_F32() {
} // ~Inst_VOP3__V_CMP_NLT_F32

// --- description from .arch file ---
// D.u64[threadID] = !(S0 < S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_NLT_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, !(src0[lane] < src1[lane]) ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_TRU_F32 class methods ---

Inst_VOP3__V_CMP_TRU_F32::Inst_VOP3__V_CMP_TRU_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_tru_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
} // Inst_VOP3__V_CMP_TRU_F32

Inst_VOP3__V_CMP_TRU_F32::~Inst_VOP3__V_CMP_TRU_F32() {
} // ~Inst_VOP3__V_CMP_TRU_F32

// --- description from .arch file ---
// D.u64[threadID] = 1; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_TRU_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 1);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_F_F32 class methods ---

Inst_VOP3__V_CMPX_F_F32::Inst_VOP3__V_CMPX_F_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_f_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_F_F32

Inst_VOP3__V_CMPX_F_F32::~Inst_VOP3__V_CMPX_F_F32() {
} // ~Inst_VOP3__V_CMPX_F_F32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = 0; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_F_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_LT_F32 class methods ---

Inst_VOP3__V_CMPX_LT_F32::Inst_VOP3__V_CMPX_LT_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_lt_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_LT_F32

Inst_VOP3__V_CMPX_LT_F32::~Inst_VOP3__V_CMPX_LT_F32() {
} // ~Inst_VOP3__V_CMPX_LT_F32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 < S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_LT_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, src0[lane] < src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_EQ_F32 class methods ---

Inst_VOP3__V_CMPX_EQ_F32::Inst_VOP3__V_CMPX_EQ_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_eq_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_EQ_F32

Inst_VOP3__V_CMPX_EQ_F32::~Inst_VOP3__V_CMPX_EQ_F32() {
} // ~Inst_VOP3__V_CMPX_EQ_F32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 == S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_EQ_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, src0[lane] == src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_LE_F32 class methods ---

Inst_VOP3__V_CMPX_LE_F32::Inst_VOP3__V_CMPX_LE_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_le_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_LE_F32

Inst_VOP3__V_CMPX_LE_F32::~Inst_VOP3__V_CMPX_LE_F32() {
} // ~Inst_VOP3__V_CMPX_LE_F32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 <= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_LE_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, src0[lane] <= src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_GT_F32 class methods ---

Inst_VOP3__V_CMPX_GT_F32::Inst_VOP3__V_CMPX_GT_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_gt_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_GT_F32

Inst_VOP3__V_CMPX_GT_F32::~Inst_VOP3__V_CMPX_GT_F32() {
} // ~Inst_VOP3__V_CMPX_GT_F32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 > S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_GT_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, src0[lane] > src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_LG_F32 class methods ---

Inst_VOP3__V_CMPX_LG_F32::Inst_VOP3__V_CMPX_LG_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_lg_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_LG_F32

Inst_VOP3__V_CMPX_LG_F32::~Inst_VOP3__V_CMPX_LG_F32() {
} // ~Inst_VOP3__V_CMPX_LG_F32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 <> S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_LG_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(
                lane,
                (src0[lane] < src1[lane] || src0[lane] > src1[lane]) ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_GE_F32 class methods ---

Inst_VOP3__V_CMPX_GE_F32::Inst_VOP3__V_CMPX_GE_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_ge_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_GE_F32

Inst_VOP3__V_CMPX_GE_F32::~Inst_VOP3__V_CMPX_GE_F32() {
} // ~Inst_VOP3__V_CMPX_GE_F32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 >= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_GE_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, src0[lane] >= src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_O_F32 class methods ---

Inst_VOP3__V_CMPX_O_F32::Inst_VOP3__V_CMPX_O_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_o_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_O_F32

Inst_VOP3__V_CMPX_O_F32::~Inst_VOP3__V_CMPX_O_F32() {
} // ~Inst_VOP3__V_CMPX_O_F32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (!isNan(S0) && !isNan(S1)); D = VCC in VOPC
// encoding.
void
Inst_VOP3__V_CMPX_O_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(
                lane,
                (!std::isnan(src0[lane]) && !std::isnan(src1[lane])) ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_U_F32 class methods ---

Inst_VOP3__V_CMPX_U_F32::Inst_VOP3__V_CMPX_U_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_u_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_U_F32

Inst_VOP3__V_CMPX_U_F32::~Inst_VOP3__V_CMPX_U_F32() {
} // ~Inst_VOP3__V_CMPX_U_F32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (isNan(S0)  ||  isNan(S1)); D = VCC in VOPC
// encoding.
void
Inst_VOP3__V_CMPX_U_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(
                lane,
                (std::isnan(src0[lane]) || std::isnan(src1[lane])) ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_NGE_F32 class methods ---

Inst_VOP3__V_CMPX_NGE_F32::Inst_VOP3__V_CMPX_NGE_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_nge_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_NGE_F32

Inst_VOP3__V_CMPX_NGE_F32::~Inst_VOP3__V_CMPX_NGE_F32() {
} // ~Inst_VOP3__V_CMPX_NGE_F32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = !(S0 >= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_NGE_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, !(src0[lane] >= src1[lane]) ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_NLG_F32 class methods ---

Inst_VOP3__V_CMPX_NLG_F32::Inst_VOP3__V_CMPX_NLG_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_nlg_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_NLG_F32

Inst_VOP3__V_CMPX_NLG_F32::~Inst_VOP3__V_CMPX_NLG_F32() {
} // ~Inst_VOP3__V_CMPX_NLG_F32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = !(S0 <> S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_NLG_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(
                lane,
                !(src0[lane] < src1[lane] || src0[lane] > src1[lane]) ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_NGT_F32 class methods ---

Inst_VOP3__V_CMPX_NGT_F32::Inst_VOP3__V_CMPX_NGT_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_ngt_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_NGT_F32

Inst_VOP3__V_CMPX_NGT_F32::~Inst_VOP3__V_CMPX_NGT_F32() {
} // ~Inst_VOP3__V_CMPX_NGT_F32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = !(S0 > S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_NGT_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, !(src0[lane] > src1[lane]) ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_NLE_F32 class methods ---

Inst_VOP3__V_CMPX_NLE_F32::Inst_VOP3__V_CMPX_NLE_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_nle_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_NLE_F32

Inst_VOP3__V_CMPX_NLE_F32::~Inst_VOP3__V_CMPX_NLE_F32() {
} // ~Inst_VOP3__V_CMPX_NLE_F32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = !(S0 <= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_NLE_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, !(src0[lane] <= src1[lane]) ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_NEQ_F32 class methods ---

Inst_VOP3__V_CMPX_NEQ_F32::Inst_VOP3__V_CMPX_NEQ_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_neq_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_NEQ_F32

Inst_VOP3__V_CMPX_NEQ_F32::~Inst_VOP3__V_CMPX_NEQ_F32() {
} // ~Inst_VOP3__V_CMPX_NEQ_F32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = !(S0 == S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_NEQ_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, src0[lane] != src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_NLT_F32 class methods ---

Inst_VOP3__V_CMPX_NLT_F32::Inst_VOP3__V_CMPX_NLT_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_nlt_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_NLT_F32

Inst_VOP3__V_CMPX_NLT_F32::~Inst_VOP3__V_CMPX_NLT_F32() {
} // ~Inst_VOP3__V_CMPX_NLT_F32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = !(S0 < S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_NLT_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, !(src0[lane] < src1[lane]) ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_TRU_F32 class methods ---

Inst_VOP3__V_CMPX_TRU_F32::Inst_VOP3__V_CMPX_TRU_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_tru_f32", true)
{
    setFlag(ALU);
    setFlag(F32);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_TRU_F32

Inst_VOP3__V_CMPX_TRU_F32::~Inst_VOP3__V_CMPX_TRU_F32() {
} // ~Inst_VOP3__V_CMPX_TRU_F32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = 1; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_TRU_F32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 1);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_F_F64 class methods ---

Inst_VOP3__V_CMP_F_F64::Inst_VOP3__V_CMP_F_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_f_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_CMP_F_F64

Inst_VOP3__V_CMP_F_F64::~Inst_VOP3__V_CMP_F_F64() {} // ~Inst_VOP3__V_CMP_F_F64

// --- description from .arch file ---
// D.u64[threadID] = 0; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_F_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_LT_F64 class methods ---

Inst_VOP3__V_CMP_LT_F64::Inst_VOP3__V_CMP_LT_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_lt_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_CMP_LT_F64

Inst_VOP3__V_CMP_LT_F64::~Inst_VOP3__V_CMP_LT_F64() {
} // ~Inst_VOP3__V_CMP_LT_F64

// --- description from .arch file ---
// D.u64[threadID] = (S0 < S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_LT_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] < src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_EQ_F64 class methods ---

Inst_VOP3__V_CMP_EQ_F64::Inst_VOP3__V_CMP_EQ_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_eq_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_CMP_EQ_F64

Inst_VOP3__V_CMP_EQ_F64::~Inst_VOP3__V_CMP_EQ_F64() {
} // ~Inst_VOP3__V_CMP_EQ_F64

// --- description from .arch file ---
// D.u64[threadID] = (S0 == S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_EQ_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] == src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_LE_F64 class methods ---

Inst_VOP3__V_CMP_LE_F64::Inst_VOP3__V_CMP_LE_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_le_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_CMP_LE_F64

Inst_VOP3__V_CMP_LE_F64::~Inst_VOP3__V_CMP_LE_F64() {
} // ~Inst_VOP3__V_CMP_LE_F64

// --- description from .arch file ---
// D.u64[threadID] = (S0 <= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_LE_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] <= src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_GT_F64 class methods ---

Inst_VOP3__V_CMP_GT_F64::Inst_VOP3__V_CMP_GT_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_gt_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_CMP_GT_F64

Inst_VOP3__V_CMP_GT_F64::~Inst_VOP3__V_CMP_GT_F64() {
} // ~Inst_VOP3__V_CMP_GT_F64

// --- description from .arch file ---
// D.u64[threadID] = (S0 > S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_GT_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] > src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_LG_F64 class methods ---

Inst_VOP3__V_CMP_LG_F64::Inst_VOP3__V_CMP_LG_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_lg_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_CMP_LG_F64

Inst_VOP3__V_CMP_LG_F64::~Inst_VOP3__V_CMP_LG_F64() {
} // ~Inst_VOP3__V_CMP_LG_F64

// --- description from .arch file ---
// D.u64[threadID] = (S0 <> S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_LG_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(
                lane,
                (src0[lane] < src1[lane] || src0[lane] > src1[lane]) ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_GE_F64 class methods ---

Inst_VOP3__V_CMP_GE_F64::Inst_VOP3__V_CMP_GE_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_ge_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_CMP_GE_F64

Inst_VOP3__V_CMP_GE_F64::~Inst_VOP3__V_CMP_GE_F64() {
} // ~Inst_VOP3__V_CMP_GE_F64

// --- description from .arch file ---
// D.u64[threadID] = (S0 >= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_GE_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] >= src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_O_F64 class methods ---

Inst_VOP3__V_CMP_O_F64::Inst_VOP3__V_CMP_O_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_o_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_CMP_O_F64

Inst_VOP3__V_CMP_O_F64::~Inst_VOP3__V_CMP_O_F64() {} // ~Inst_VOP3__V_CMP_O_F64

// --- description from .arch file ---
// D.u64[threadID] = (!isNan(S0) && !isNan(S1)); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_O_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(
                lane,
                (!std::isnan(src0[lane]) && !std::isnan(src1[lane])) ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_U_F64 class methods ---

Inst_VOP3__V_CMP_U_F64::Inst_VOP3__V_CMP_U_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_u_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_CMP_U_F64

Inst_VOP3__V_CMP_U_F64::~Inst_VOP3__V_CMP_U_F64() {} // ~Inst_VOP3__V_CMP_U_F64

// --- description from .arch file ---
// D.u64[threadID] = (isNan(S0)  ||  isNan(S1)); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_U_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(
                lane,
                (std::isnan(src0[lane]) || std::isnan(src1[lane])) ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_NGE_F64 class methods ---

Inst_VOP3__V_CMP_NGE_F64::Inst_VOP3__V_CMP_NGE_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_nge_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_CMP_NGE_F64

Inst_VOP3__V_CMP_NGE_F64::~Inst_VOP3__V_CMP_NGE_F64() {
} // ~Inst_VOP3__V_CMP_NGE_F64

// --- description from .arch file ---
// D.u64[threadID] = !(S0 >= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_NGE_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, !(src0[lane] >= src1[lane]) ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_NLG_F64 class methods ---

Inst_VOP3__V_CMP_NLG_F64::Inst_VOP3__V_CMP_NLG_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_nlg_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_CMP_NLG_F64

Inst_VOP3__V_CMP_NLG_F64::~Inst_VOP3__V_CMP_NLG_F64() {
} // ~Inst_VOP3__V_CMP_NLG_F64

// --- description from .arch file ---
// D.u64[threadID] = !(S0 <> S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_NLG_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(
                lane,
                !(src0[lane] < src1[lane] || src0[lane] > src1[lane]) ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_NGT_F64 class methods ---

Inst_VOP3__V_CMP_NGT_F64::Inst_VOP3__V_CMP_NGT_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_ngt_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_CMP_NGT_F64

Inst_VOP3__V_CMP_NGT_F64::~Inst_VOP3__V_CMP_NGT_F64() {
} // ~Inst_VOP3__V_CMP_NGT_F64

// --- description from .arch file ---
// D.u64[threadID] = !(S0 > S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_NGT_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, !(src0[lane] > src1[lane]) ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_NLE_F64 class methods ---

Inst_VOP3__V_CMP_NLE_F64::Inst_VOP3__V_CMP_NLE_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_nle_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_CMP_NLE_F64

Inst_VOP3__V_CMP_NLE_F64::~Inst_VOP3__V_CMP_NLE_F64() {
} // ~Inst_VOP3__V_CMP_NLE_F64

// --- description from .arch file ---
// D.u64[threadID] = !(S0 <= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_NLE_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, !(src0[lane] <= src1[lane]) ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_NEQ_F64 class methods ---

Inst_VOP3__V_CMP_NEQ_F64::Inst_VOP3__V_CMP_NEQ_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_neq_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_CMP_NEQ_F64

Inst_VOP3__V_CMP_NEQ_F64::~Inst_VOP3__V_CMP_NEQ_F64() {
} // ~Inst_VOP3__V_CMP_NEQ_F64

// --- description from .arch file ---
// D.u64[threadID] = !(S0 == S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_NEQ_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] != src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_NLT_F64 class methods ---

Inst_VOP3__V_CMP_NLT_F64::Inst_VOP3__V_CMP_NLT_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_nlt_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_CMP_NLT_F64

Inst_VOP3__V_CMP_NLT_F64::~Inst_VOP3__V_CMP_NLT_F64() {
} // ~Inst_VOP3__V_CMP_NLT_F64

// --- description from .arch file ---
// D.u64[threadID] = !(S0 < S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_NLT_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, !(src0[lane] < src1[lane]) ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_TRU_F64 class methods ---

Inst_VOP3__V_CMP_TRU_F64::Inst_VOP3__V_CMP_TRU_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_tru_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
} // Inst_VOP3__V_CMP_TRU_F64

Inst_VOP3__V_CMP_TRU_F64::~Inst_VOP3__V_CMP_TRU_F64() {
} // ~Inst_VOP3__V_CMP_TRU_F64

// --- description from .arch file ---
// D.u64[threadID] = 1; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_TRU_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 1);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_F_F64 class methods ---

Inst_VOP3__V_CMPX_F_F64::Inst_VOP3__V_CMPX_F_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_f_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_F_F64

Inst_VOP3__V_CMPX_F_F64::~Inst_VOP3__V_CMPX_F_F64() {
} // ~Inst_VOP3__V_CMPX_F_F64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = 0; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_F_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_LT_F64 class methods ---

Inst_VOP3__V_CMPX_LT_F64::Inst_VOP3__V_CMPX_LT_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_lt_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_LT_F64

Inst_VOP3__V_CMPX_LT_F64::~Inst_VOP3__V_CMPX_LT_F64() {
} // ~Inst_VOP3__V_CMPX_LT_F64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 < S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_LT_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] < src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_EQ_F64 class methods ---

Inst_VOP3__V_CMPX_EQ_F64::Inst_VOP3__V_CMPX_EQ_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_eq_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_EQ_F64

Inst_VOP3__V_CMPX_EQ_F64::~Inst_VOP3__V_CMPX_EQ_F64() {
} // ~Inst_VOP3__V_CMPX_EQ_F64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 == S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_EQ_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] == src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_LE_F64 class methods ---

Inst_VOP3__V_CMPX_LE_F64::Inst_VOP3__V_CMPX_LE_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_le_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_LE_F64

Inst_VOP3__V_CMPX_LE_F64::~Inst_VOP3__V_CMPX_LE_F64() {
} // ~Inst_VOP3__V_CMPX_LE_F64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 <= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_LE_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] <= src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_GT_F64 class methods ---

Inst_VOP3__V_CMPX_GT_F64::Inst_VOP3__V_CMPX_GT_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_gt_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_GT_F64

Inst_VOP3__V_CMPX_GT_F64::~Inst_VOP3__V_CMPX_GT_F64() {
} // ~Inst_VOP3__V_CMPX_GT_F64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 > S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_GT_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] > src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_LG_F64 class methods ---

Inst_VOP3__V_CMPX_LG_F64::Inst_VOP3__V_CMPX_LG_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_lg_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_LG_F64

Inst_VOP3__V_CMPX_LG_F64::~Inst_VOP3__V_CMPX_LG_F64() {
} // ~Inst_VOP3__V_CMPX_LG_F64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 <> S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_LG_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(
                lane,
                (src0[lane] < src1[lane] || src0[lane] > src1[lane]) ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_GE_F64 class methods ---

Inst_VOP3__V_CMPX_GE_F64::Inst_VOP3__V_CMPX_GE_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_ge_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_GE_F64

Inst_VOP3__V_CMPX_GE_F64::~Inst_VOP3__V_CMPX_GE_F64() {
} // ~Inst_VOP3__V_CMPX_GE_F64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 >= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_GE_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] >= src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_O_F64 class methods ---

Inst_VOP3__V_CMPX_O_F64::Inst_VOP3__V_CMPX_O_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_o_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_O_F64

Inst_VOP3__V_CMPX_O_F64::~Inst_VOP3__V_CMPX_O_F64() {
} // ~Inst_VOP3__V_CMPX_O_F64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (!isNan(S0) && !isNan(S1)); D = VCC in VOPC
// encoding.
void
Inst_VOP3__V_CMPX_O_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(
                lane,
                (!std::isnan(src0[lane]) && !std::isnan(src1[lane])) ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_U_F64 class methods ---

Inst_VOP3__V_CMPX_U_F64::Inst_VOP3__V_CMPX_U_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_u_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_U_F64

Inst_VOP3__V_CMPX_U_F64::~Inst_VOP3__V_CMPX_U_F64() {
} // ~Inst_VOP3__V_CMPX_U_F64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (isNan(S0)  ||  isNan(S1)); D = VCC in VOPC
// encoding.
void
Inst_VOP3__V_CMPX_U_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(
                lane,
                (std::isnan(src0[lane]) || std::isnan(src1[lane])) ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_NGE_F64 class methods ---

Inst_VOP3__V_CMPX_NGE_F64::Inst_VOP3__V_CMPX_NGE_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_nge_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_NGE_F64

Inst_VOP3__V_CMPX_NGE_F64::~Inst_VOP3__V_CMPX_NGE_F64() {
} // ~Inst_VOP3__V_CMPX_NGE_F64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = !(S0 >= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_NGE_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, !(src0[lane] >= src1[lane]) ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_NLG_F64 class methods ---

Inst_VOP3__V_CMPX_NLG_F64::Inst_VOP3__V_CMPX_NLG_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_nlg_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_NLG_F64

Inst_VOP3__V_CMPX_NLG_F64::~Inst_VOP3__V_CMPX_NLG_F64() {
} // ~Inst_VOP3__V_CMPX_NLG_F64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = !(S0 <> S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_NLG_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(
                lane,
                !(src0[lane] < src1[lane] || src0[lane] > src1[lane]) ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_NGT_F64 class methods ---

Inst_VOP3__V_CMPX_NGT_F64::Inst_VOP3__V_CMPX_NGT_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_ngt_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_NGT_F64

Inst_VOP3__V_CMPX_NGT_F64::~Inst_VOP3__V_CMPX_NGT_F64() {
} // ~Inst_VOP3__V_CMPX_NGT_F64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = !(S0 > S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_NGT_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, !(src0[lane] > src1[lane]) ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_NLE_F64 class methods ---

Inst_VOP3__V_CMPX_NLE_F64::Inst_VOP3__V_CMPX_NLE_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_nle_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_NLE_F64

Inst_VOP3__V_CMPX_NLE_F64::~Inst_VOP3__V_CMPX_NLE_F64() {
} // ~Inst_VOP3__V_CMPX_NLE_F64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = !(S0 <= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_NLE_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, !(src0[lane] <= src1[lane]) ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_NEQ_F64 class methods ---

Inst_VOP3__V_CMPX_NEQ_F64::Inst_VOP3__V_CMPX_NEQ_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_neq_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_NEQ_F64

Inst_VOP3__V_CMPX_NEQ_F64::~Inst_VOP3__V_CMPX_NEQ_F64() {
} // ~Inst_VOP3__V_CMPX_NEQ_F64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = !(S0 == S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_NEQ_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] != src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_NLT_F64 class methods ---

Inst_VOP3__V_CMPX_NLT_F64::Inst_VOP3__V_CMPX_NLT_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_nlt_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_NLT_F64

Inst_VOP3__V_CMPX_NLT_F64::~Inst_VOP3__V_CMPX_NLT_F64() {
} // ~Inst_VOP3__V_CMPX_NLT_F64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = !(S0 < S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_NLT_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandF64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, !(src0[lane] < src1[lane]) ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_TRU_F64 class methods ---

Inst_VOP3__V_CMPX_TRU_F64::Inst_VOP3__V_CMPX_TRU_F64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_tru_f64", true)
{
    setFlag(ALU);
    setFlag(F64);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_TRU_F64

Inst_VOP3__V_CMPX_TRU_F64::~Inst_VOP3__V_CMPX_TRU_F64() {
} // ~Inst_VOP3__V_CMPX_TRU_F64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = 1; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_TRU_F64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 1);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_F_I16 class methods ---

Inst_VOP3__V_CMP_F_I16::Inst_VOP3__V_CMP_F_I16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_f_i16", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_F_I16

Inst_VOP3__V_CMP_F_I16::~Inst_VOP3__V_CMP_F_I16() {} // ~Inst_VOP3__V_CMP_F_I16

// --- description from .arch file ---
// D.u64[threadID] = 0; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_F_I16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_LT_I16 class methods ---

Inst_VOP3__V_CMP_LT_I16::Inst_VOP3__V_CMP_LT_I16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_lt_i16", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_LT_I16

Inst_VOP3__V_CMP_LT_I16::~Inst_VOP3__V_CMP_LT_I16() {
} // ~Inst_VOP3__V_CMP_LT_I16

// --- description from .arch file ---
// D.u64[threadID] = (S0 < S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_LT_I16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI16 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] < src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_EQ_I16 class methods ---

Inst_VOP3__V_CMP_EQ_I16::Inst_VOP3__V_CMP_EQ_I16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_eq_i16", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_EQ_I16

Inst_VOP3__V_CMP_EQ_I16::~Inst_VOP3__V_CMP_EQ_I16() {
} // ~Inst_VOP3__V_CMP_EQ_I16

// --- description from .arch file ---
// D.u64[threadID] = (S0 == S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_EQ_I16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI16 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] == src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_LE_I16 class methods ---

Inst_VOP3__V_CMP_LE_I16::Inst_VOP3__V_CMP_LE_I16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_le_i16", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_LE_I16

Inst_VOP3__V_CMP_LE_I16::~Inst_VOP3__V_CMP_LE_I16() {
} // ~Inst_VOP3__V_CMP_LE_I16

// --- description from .arch file ---
// D.u64[threadID] = (S0 <= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_LE_I16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI16 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] <= src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_GT_I16 class methods ---

Inst_VOP3__V_CMP_GT_I16::Inst_VOP3__V_CMP_GT_I16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_gt_i16", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_GT_I16

Inst_VOP3__V_CMP_GT_I16::~Inst_VOP3__V_CMP_GT_I16() {
} // ~Inst_VOP3__V_CMP_GT_I16

// --- description from .arch file ---
// D.u64[threadID] = (S0 > S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_GT_I16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI16 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] > src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_NE_I16 class methods ---

Inst_VOP3__V_CMP_NE_I16::Inst_VOP3__V_CMP_NE_I16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_ne_i16", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_NE_I16

Inst_VOP3__V_CMP_NE_I16::~Inst_VOP3__V_CMP_NE_I16() {
} // ~Inst_VOP3__V_CMP_NE_I16

// --- description from .arch file ---
// D.u64[threadID] = (S0 <> S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_NE_I16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI16 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] != src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_GE_I16 class methods ---

Inst_VOP3__V_CMP_GE_I16::Inst_VOP3__V_CMP_GE_I16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_ge_i16", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_GE_I16

Inst_VOP3__V_CMP_GE_I16::~Inst_VOP3__V_CMP_GE_I16() {
} // ~Inst_VOP3__V_CMP_GE_I16

// --- description from .arch file ---
// D.u64[threadID] = (S0 >= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_GE_I16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI16 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] >= src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_T_I16 class methods ---

Inst_VOP3__V_CMP_T_I16::Inst_VOP3__V_CMP_T_I16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_t_i16", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_T_I16

Inst_VOP3__V_CMP_T_I16::~Inst_VOP3__V_CMP_T_I16() {} // ~Inst_VOP3__V_CMP_T_I16

// --- description from .arch file ---
// D.u64[threadID] = 1; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_T_I16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 1);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_F_U16 class methods ---

Inst_VOP3__V_CMP_F_U16::Inst_VOP3__V_CMP_F_U16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_f_u16", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_F_U16

Inst_VOP3__V_CMP_F_U16::~Inst_VOP3__V_CMP_F_U16() {} // ~Inst_VOP3__V_CMP_F_U16

// --- description from .arch file ---
// D.u64[threadID] = 0; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_F_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_LT_U16 class methods ---

Inst_VOP3__V_CMP_LT_U16::Inst_VOP3__V_CMP_LT_U16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_lt_u16", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_LT_U16

Inst_VOP3__V_CMP_LT_U16::~Inst_VOP3__V_CMP_LT_U16() {
} // ~Inst_VOP3__V_CMP_LT_U16

// --- description from .arch file ---
// D.u64[threadID] = (S0 < S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_LT_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU16 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] < src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_EQ_U16 class methods ---

Inst_VOP3__V_CMP_EQ_U16::Inst_VOP3__V_CMP_EQ_U16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_eq_u16", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_EQ_U16

Inst_VOP3__V_CMP_EQ_U16::~Inst_VOP3__V_CMP_EQ_U16() {
} // ~Inst_VOP3__V_CMP_EQ_U16

// --- description from .arch file ---
// D.u64[threadID] = (S0 == S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_EQ_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU16 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] == src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_LE_U16 class methods ---

Inst_VOP3__V_CMP_LE_U16::Inst_VOP3__V_CMP_LE_U16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_le_u16", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_LE_U16

Inst_VOP3__V_CMP_LE_U16::~Inst_VOP3__V_CMP_LE_U16() {
} // ~Inst_VOP3__V_CMP_LE_U16

// --- description from .arch file ---
// D.u64[threadID] = (S0 <= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_LE_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU16 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] <= src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_GT_U16 class methods ---

Inst_VOP3__V_CMP_GT_U16::Inst_VOP3__V_CMP_GT_U16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_gt_u16", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_GT_U16

Inst_VOP3__V_CMP_GT_U16::~Inst_VOP3__V_CMP_GT_U16() {
} // ~Inst_VOP3__V_CMP_GT_U16

// --- description from .arch file ---
// D.u64[threadID] = (S0 > S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_GT_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU16 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] > src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_NE_U16 class methods ---

Inst_VOP3__V_CMP_NE_U16::Inst_VOP3__V_CMP_NE_U16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_ne_u16", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_NE_U16

Inst_VOP3__V_CMP_NE_U16::~Inst_VOP3__V_CMP_NE_U16() {
} // ~Inst_VOP3__V_CMP_NE_U16

// --- description from .arch file ---
// D.u64[threadID] = (S0 <> S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_NE_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU16 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] != src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_GE_U16 class methods ---

Inst_VOP3__V_CMP_GE_U16::Inst_VOP3__V_CMP_GE_U16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_ge_u16", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_GE_U16

Inst_VOP3__V_CMP_GE_U16::~Inst_VOP3__V_CMP_GE_U16() {
} // ~Inst_VOP3__V_CMP_GE_U16

// --- description from .arch file ---
// D.u64[threadID] = (S0 >= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_GE_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU16 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] >= src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_T_U16 class methods ---

Inst_VOP3__V_CMP_T_U16::Inst_VOP3__V_CMP_T_U16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_t_u16", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_T_U16

Inst_VOP3__V_CMP_T_U16::~Inst_VOP3__V_CMP_T_U16() {} // ~Inst_VOP3__V_CMP_T_U16

// --- description from .arch file ---
// D.u64[threadID] = 1; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_T_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 1);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_F_I16 class methods ---

Inst_VOP3__V_CMPX_F_I16::Inst_VOP3__V_CMPX_F_I16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_f_i16", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_F_I16

Inst_VOP3__V_CMPX_F_I16::~Inst_VOP3__V_CMPX_F_I16() {
} // ~Inst_VOP3__V_CMPX_F_I16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = 0; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_F_I16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_LT_I16 class methods ---

Inst_VOP3__V_CMPX_LT_I16::Inst_VOP3__V_CMPX_LT_I16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_lt_i16", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_LT_I16

Inst_VOP3__V_CMPX_LT_I16::~Inst_VOP3__V_CMPX_LT_I16() {
} // ~Inst_VOP3__V_CMPX_LT_I16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 < S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_LT_I16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI16 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] < src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_EQ_I16 class methods ---

Inst_VOP3__V_CMPX_EQ_I16::Inst_VOP3__V_CMPX_EQ_I16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_eq_i16", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_EQ_I16

Inst_VOP3__V_CMPX_EQ_I16::~Inst_VOP3__V_CMPX_EQ_I16() {
} // ~Inst_VOP3__V_CMPX_EQ_I16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 == S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_EQ_I16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI16 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] == src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_LE_I16 class methods ---

Inst_VOP3__V_CMPX_LE_I16::Inst_VOP3__V_CMPX_LE_I16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_le_i16", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_LE_I16

Inst_VOP3__V_CMPX_LE_I16::~Inst_VOP3__V_CMPX_LE_I16() {
} // ~Inst_VOP3__V_CMPX_LE_I16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 <= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_LE_I16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI16 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] <= src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_GT_I16 class methods ---

Inst_VOP3__V_CMPX_GT_I16::Inst_VOP3__V_CMPX_GT_I16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_gt_i16", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_GT_I16

Inst_VOP3__V_CMPX_GT_I16::~Inst_VOP3__V_CMPX_GT_I16() {
} // ~Inst_VOP3__V_CMPX_GT_I16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 > S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_GT_I16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI16 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] > src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_NE_I16 class methods ---

Inst_VOP3__V_CMPX_NE_I16::Inst_VOP3__V_CMPX_NE_I16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_ne_i16", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_NE_I16

Inst_VOP3__V_CMPX_NE_I16::~Inst_VOP3__V_CMPX_NE_I16() {
} // ~Inst_VOP3__V_CMPX_NE_I16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 <> S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_NE_I16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI16 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] != src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_GE_I16 class methods ---

Inst_VOP3__V_CMPX_GE_I16::Inst_VOP3__V_CMPX_GE_I16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_ge_i16", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_GE_I16

Inst_VOP3__V_CMPX_GE_I16::~Inst_VOP3__V_CMPX_GE_I16() {
} // ~Inst_VOP3__V_CMPX_GE_I16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 >= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_GE_I16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI16 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] >= src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_T_I16 class methods ---

Inst_VOP3__V_CMPX_T_I16::Inst_VOP3__V_CMPX_T_I16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_t_i16", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_T_I16

Inst_VOP3__V_CMPX_T_I16::~Inst_VOP3__V_CMPX_T_I16() {
} // ~Inst_VOP3__V_CMPX_T_I16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = 1; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_T_I16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 1);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_F_U16 class methods ---

Inst_VOP3__V_CMPX_F_U16::Inst_VOP3__V_CMPX_F_U16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_f_u16", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_F_U16

Inst_VOP3__V_CMPX_F_U16::~Inst_VOP3__V_CMPX_F_U16() {
} // ~Inst_VOP3__V_CMPX_F_U16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = 0; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_F_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_LT_U16 class methods ---

Inst_VOP3__V_CMPX_LT_U16::Inst_VOP3__V_CMPX_LT_U16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_lt_u16", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_LT_U16

Inst_VOP3__V_CMPX_LT_U16::~Inst_VOP3__V_CMPX_LT_U16() {
} // ~Inst_VOP3__V_CMPX_LT_U16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 < S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_LT_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU16 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] < src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_EQ_U16 class methods ---

Inst_VOP3__V_CMPX_EQ_U16::Inst_VOP3__V_CMPX_EQ_U16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_eq_u16", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_EQ_U16

Inst_VOP3__V_CMPX_EQ_U16::~Inst_VOP3__V_CMPX_EQ_U16() {
} // ~Inst_VOP3__V_CMPX_EQ_U16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 == S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_EQ_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI16 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] == src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_LE_U16 class methods ---

Inst_VOP3__V_CMPX_LE_U16::Inst_VOP3__V_CMPX_LE_U16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_le_u16", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_LE_U16

Inst_VOP3__V_CMPX_LE_U16::~Inst_VOP3__V_CMPX_LE_U16() {
} // ~Inst_VOP3__V_CMPX_LE_U16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 <= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_LE_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI16 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] <= src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_GT_U16 class methods ---

Inst_VOP3__V_CMPX_GT_U16::Inst_VOP3__V_CMPX_GT_U16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_gt_u16", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_GT_U16

Inst_VOP3__V_CMPX_GT_U16::~Inst_VOP3__V_CMPX_GT_U16() {
} // ~Inst_VOP3__V_CMPX_GT_U16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 > S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_GT_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI16 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] > src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_NE_U16 class methods ---

Inst_VOP3__V_CMPX_NE_U16::Inst_VOP3__V_CMPX_NE_U16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_ne_u16", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_NE_U16

Inst_VOP3__V_CMPX_NE_U16::~Inst_VOP3__V_CMPX_NE_U16() {
} // ~Inst_VOP3__V_CMPX_NE_U16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 <> S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_NE_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI16 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] != src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_GE_U16 class methods ---

Inst_VOP3__V_CMPX_GE_U16::Inst_VOP3__V_CMPX_GE_U16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_ge_u16", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_GE_U16

Inst_VOP3__V_CMPX_GE_U16::~Inst_VOP3__V_CMPX_GE_U16() {
} // ~Inst_VOP3__V_CMPX_GE_U16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 >= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_GE_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI16 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI16 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] >= src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_T_U16 class methods ---

Inst_VOP3__V_CMPX_T_U16::Inst_VOP3__V_CMPX_T_U16(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_t_u16", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_T_U16

Inst_VOP3__V_CMPX_T_U16::~Inst_VOP3__V_CMPX_T_U16() {
} // ~Inst_VOP3__V_CMPX_T_U16

// --- description from .arch file ---
// EXEC,D.u64[threadID] = 1; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_T_U16::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 1);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_F_I32 class methods ---

Inst_VOP3__V_CMP_F_I32::Inst_VOP3__V_CMP_F_I32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_f_i32", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_F_I32

Inst_VOP3__V_CMP_F_I32::~Inst_VOP3__V_CMP_F_I32() {} // ~Inst_VOP3__V_CMP_F_I32

// --- description from .arch file ---
// D.u64[threadID] = 0; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_F_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_LT_I32 class methods ---

Inst_VOP3__V_CMP_LT_I32::Inst_VOP3__V_CMP_LT_I32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_lt_i32", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_LT_I32

Inst_VOP3__V_CMP_LT_I32::~Inst_VOP3__V_CMP_LT_I32() {
} // ~Inst_VOP3__V_CMP_LT_I32

// --- description from .arch file ---
// D.u64[threadID] = (S0 < S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_LT_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] < src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_EQ_I32 class methods ---

Inst_VOP3__V_CMP_EQ_I32::Inst_VOP3__V_CMP_EQ_I32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_eq_i32", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_EQ_I32

Inst_VOP3__V_CMP_EQ_I32::~Inst_VOP3__V_CMP_EQ_I32() {
} // ~Inst_VOP3__V_CMP_EQ_I32

// --- description from .arch file ---
// D.u64[threadID] = (S0 == S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_EQ_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] == src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_LE_I32 class methods ---

Inst_VOP3__V_CMP_LE_I32::Inst_VOP3__V_CMP_LE_I32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_le_i32", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_LE_I32

Inst_VOP3__V_CMP_LE_I32::~Inst_VOP3__V_CMP_LE_I32() {
} // ~Inst_VOP3__V_CMP_LE_I32

// --- description from .arch file ---
// D.u64[threadID] = (S0 <= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_LE_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] <= src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_GT_I32 class methods ---

Inst_VOP3__V_CMP_GT_I32::Inst_VOP3__V_CMP_GT_I32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_gt_i32", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_GT_I32

Inst_VOP3__V_CMP_GT_I32::~Inst_VOP3__V_CMP_GT_I32() {
} // ~Inst_VOP3__V_CMP_GT_I32

// --- description from .arch file ---
// D.u64[threadID] = (S0 > S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_GT_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] > src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_NE_I32 class methods ---

Inst_VOP3__V_CMP_NE_I32::Inst_VOP3__V_CMP_NE_I32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_ne_i32", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_NE_I32

Inst_VOP3__V_CMP_NE_I32::~Inst_VOP3__V_CMP_NE_I32() {
} // ~Inst_VOP3__V_CMP_NE_I32

// --- description from .arch file ---
// D.u64[threadID] = (S0 <> S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_NE_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] != src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_GE_I32 class methods ---

Inst_VOP3__V_CMP_GE_I32::Inst_VOP3__V_CMP_GE_I32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_ge_i32", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_GE_I32

Inst_VOP3__V_CMP_GE_I32::~Inst_VOP3__V_CMP_GE_I32() {
} // ~Inst_VOP3__V_CMP_GE_I32

// --- description from .arch file ---
// D.u64[threadID] = (S0 >= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_GE_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] >= src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_T_I32 class methods ---

Inst_VOP3__V_CMP_T_I32::Inst_VOP3__V_CMP_T_I32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_t_i32", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_T_I32

Inst_VOP3__V_CMP_T_I32::~Inst_VOP3__V_CMP_T_I32() {} // ~Inst_VOP3__V_CMP_T_I32

// --- description from .arch file ---
// D.u64[threadID] = 1; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_T_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 1);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_F_U32 class methods ---

Inst_VOP3__V_CMP_F_U32::Inst_VOP3__V_CMP_F_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_f_u32", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_F_U32

Inst_VOP3__V_CMP_F_U32::~Inst_VOP3__V_CMP_F_U32() {} // ~Inst_VOP3__V_CMP_F_U32

// --- description from .arch file ---
// D.u64[threadID] = 0; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_F_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_LT_U32 class methods ---

Inst_VOP3__V_CMP_LT_U32::Inst_VOP3__V_CMP_LT_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_lt_u32", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_LT_U32

Inst_VOP3__V_CMP_LT_U32::~Inst_VOP3__V_CMP_LT_U32() {
} // ~Inst_VOP3__V_CMP_LT_U32

// --- description from .arch file ---
// D.u64[threadID] = (S0 < S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_LT_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] < src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_EQ_U32 class methods ---

Inst_VOP3__V_CMP_EQ_U32::Inst_VOP3__V_CMP_EQ_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_eq_u32", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_EQ_U32

Inst_VOP3__V_CMP_EQ_U32::~Inst_VOP3__V_CMP_EQ_U32() {
} // ~Inst_VOP3__V_CMP_EQ_U32

// --- description from .arch file ---
// D.u64[threadID] = (S0 == S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_EQ_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] == src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_LE_U32 class methods ---

Inst_VOP3__V_CMP_LE_U32::Inst_VOP3__V_CMP_LE_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_le_u32", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_LE_U32

Inst_VOP3__V_CMP_LE_U32::~Inst_VOP3__V_CMP_LE_U32() {
} // ~Inst_VOP3__V_CMP_LE_U32

// --- description from .arch file ---
// D.u64[threadID] = (S0 <= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_LE_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] <= src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_GT_U32 class methods ---

Inst_VOP3__V_CMP_GT_U32::Inst_VOP3__V_CMP_GT_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_gt_u32", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_GT_U32

Inst_VOP3__V_CMP_GT_U32::~Inst_VOP3__V_CMP_GT_U32() {
} // ~Inst_VOP3__V_CMP_GT_U32

// --- description from .arch file ---
// D.u64[threadID] = (S0 > S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_GT_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] > src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_NE_U32 class methods ---

Inst_VOP3__V_CMP_NE_U32::Inst_VOP3__V_CMP_NE_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_ne_u32", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_NE_U32

Inst_VOP3__V_CMP_NE_U32::~Inst_VOP3__V_CMP_NE_U32() {
} // ~Inst_VOP3__V_CMP_NE_U32

// --- description from .arch file ---
// D.u64[threadID] = (S0 <> S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_NE_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] != src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_GE_U32 class methods ---

Inst_VOP3__V_CMP_GE_U32::Inst_VOP3__V_CMP_GE_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_ge_u32", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_GE_U32

Inst_VOP3__V_CMP_GE_U32::~Inst_VOP3__V_CMP_GE_U32() {
} // ~Inst_VOP3__V_CMP_GE_U32

// --- description from .arch file ---
// D.u64[threadID] = (S0 >= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_GE_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] >= src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_T_U32 class methods ---

Inst_VOP3__V_CMP_T_U32::Inst_VOP3__V_CMP_T_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_t_u32", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_T_U32

Inst_VOP3__V_CMP_T_U32::~Inst_VOP3__V_CMP_T_U32() {} // ~Inst_VOP3__V_CMP_T_U32

// --- description from .arch file ---
// D.u64[threadID] = 1; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_T_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 1);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_F_I32 class methods ---

Inst_VOP3__V_CMPX_F_I32::Inst_VOP3__V_CMPX_F_I32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_f_i32", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_F_I32

Inst_VOP3__V_CMPX_F_I32::~Inst_VOP3__V_CMPX_F_I32() {
} // ~Inst_VOP3__V_CMPX_F_I32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = 0; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_F_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_LT_I32 class methods ---

Inst_VOP3__V_CMPX_LT_I32::Inst_VOP3__V_CMPX_LT_I32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_lt_i32", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_LT_I32

Inst_VOP3__V_CMPX_LT_I32::~Inst_VOP3__V_CMPX_LT_I32() {
} // ~Inst_VOP3__V_CMPX_LT_I32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 < S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_LT_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] < src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_EQ_I32 class methods ---

Inst_VOP3__V_CMPX_EQ_I32::Inst_VOP3__V_CMPX_EQ_I32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_eq_i32", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_EQ_I32

Inst_VOP3__V_CMPX_EQ_I32::~Inst_VOP3__V_CMPX_EQ_I32() {
} // ~Inst_VOP3__V_CMPX_EQ_I32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 == S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_EQ_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] == src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_LE_I32 class methods ---

Inst_VOP3__V_CMPX_LE_I32::Inst_VOP3__V_CMPX_LE_I32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_le_i32", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_LE_I32

Inst_VOP3__V_CMPX_LE_I32::~Inst_VOP3__V_CMPX_LE_I32() {
} // ~Inst_VOP3__V_CMPX_LE_I32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 <= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_LE_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] <= src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_GT_I32 class methods ---

Inst_VOP3__V_CMPX_GT_I32::Inst_VOP3__V_CMPX_GT_I32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_gt_i32", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_GT_I32

Inst_VOP3__V_CMPX_GT_I32::~Inst_VOP3__V_CMPX_GT_I32() {
} // ~Inst_VOP3__V_CMPX_GT_I32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 > S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_GT_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] > src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_NE_I32 class methods ---

Inst_VOP3__V_CMPX_NE_I32::Inst_VOP3__V_CMPX_NE_I32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_ne_i32", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_NE_I32

Inst_VOP3__V_CMPX_NE_I32::~Inst_VOP3__V_CMPX_NE_I32() {
} // ~Inst_VOP3__V_CMPX_NE_I32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 <> S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_NE_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] != src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_GE_I32 class methods ---

Inst_VOP3__V_CMPX_GE_I32::Inst_VOP3__V_CMPX_GE_I32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_ge_i32", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_GE_I32

Inst_VOP3__V_CMPX_GE_I32::~Inst_VOP3__V_CMPX_GE_I32() {
} // ~Inst_VOP3__V_CMPX_GE_I32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 >= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_GE_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] >= src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_T_I32 class methods ---

Inst_VOP3__V_CMPX_T_I32::Inst_VOP3__V_CMPX_T_I32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_t_i32", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_T_I32

Inst_VOP3__V_CMPX_T_I32::~Inst_VOP3__V_CMPX_T_I32() {
} // ~Inst_VOP3__V_CMPX_T_I32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = 1; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_T_I32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 1);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_F_U32 class methods ---

Inst_VOP3__V_CMPX_F_U32::Inst_VOP3__V_CMPX_F_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_f_u32", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_F_U32

Inst_VOP3__V_CMPX_F_U32::~Inst_VOP3__V_CMPX_F_U32() {
} // ~Inst_VOP3__V_CMPX_F_U32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = 0; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_F_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_LT_U32 class methods ---

Inst_VOP3__V_CMPX_LT_U32::Inst_VOP3__V_CMPX_LT_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_lt_u32", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_LT_U32

Inst_VOP3__V_CMPX_LT_U32::~Inst_VOP3__V_CMPX_LT_U32() {
} // ~Inst_VOP3__V_CMPX_LT_U32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 < S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_LT_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] < src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_EQ_U32 class methods ---

Inst_VOP3__V_CMPX_EQ_U32::Inst_VOP3__V_CMPX_EQ_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_eq_u32", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_EQ_U32

Inst_VOP3__V_CMPX_EQ_U32::~Inst_VOP3__V_CMPX_EQ_U32() {
} // ~Inst_VOP3__V_CMPX_EQ_U32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 == S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_EQ_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] == src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_LE_U32 class methods ---

Inst_VOP3__V_CMPX_LE_U32::Inst_VOP3__V_CMPX_LE_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_le_u32", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_LE_U32

Inst_VOP3__V_CMPX_LE_U32::~Inst_VOP3__V_CMPX_LE_U32() {
} // ~Inst_VOP3__V_CMPX_LE_U32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 <= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_LE_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] <= src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_GT_U32 class methods ---

Inst_VOP3__V_CMPX_GT_U32::Inst_VOP3__V_CMPX_GT_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_gt_u32", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_GT_U32

Inst_VOP3__V_CMPX_GT_U32::~Inst_VOP3__V_CMPX_GT_U32() {
} // ~Inst_VOP3__V_CMPX_GT_U32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 > S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_GT_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] > src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_NE_U32 class methods ---

Inst_VOP3__V_CMPX_NE_U32::Inst_VOP3__V_CMPX_NE_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_ne_u32", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_NE_U32

Inst_VOP3__V_CMPX_NE_U32::~Inst_VOP3__V_CMPX_NE_U32() {
} // ~Inst_VOP3__V_CMPX_NE_U32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 <> S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_NE_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] != src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_GE_U32 class methods ---

Inst_VOP3__V_CMPX_GE_U32::Inst_VOP3__V_CMPX_GE_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_ge_u32", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_GE_U32

Inst_VOP3__V_CMPX_GE_U32::~Inst_VOP3__V_CMPX_GE_U32() {
} // ~Inst_VOP3__V_CMPX_GE_U32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 >= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_GE_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] >= src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_T_U32 class methods ---

Inst_VOP3__V_CMPX_T_U32::Inst_VOP3__V_CMPX_T_U32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_t_u32", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_T_U32

Inst_VOP3__V_CMPX_T_U32::~Inst_VOP3__V_CMPX_T_U32() {
} // ~Inst_VOP3__V_CMPX_T_U32

// --- description from .arch file ---
// EXEC,D.u64[threadID] = 1; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_T_U32::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 1);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_F_I64 class methods ---

Inst_VOP3__V_CMP_F_I64::Inst_VOP3__V_CMP_F_I64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_f_i64", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_F_I64

Inst_VOP3__V_CMP_F_I64::~Inst_VOP3__V_CMP_F_I64() {} // ~Inst_VOP3__V_CMP_F_I64

// --- description from .arch file ---
// D.u64[threadID] = 0; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_F_I64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_LT_I64 class methods ---

Inst_VOP3__V_CMP_LT_I64::Inst_VOP3__V_CMP_LT_I64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_lt_i64", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_LT_I64

Inst_VOP3__V_CMP_LT_I64::~Inst_VOP3__V_CMP_LT_I64() {
} // ~Inst_VOP3__V_CMP_LT_I64

// --- description from .arch file ---
// D.u64[threadID] = (S0 < S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_LT_I64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] < src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_EQ_I64 class methods ---

Inst_VOP3__V_CMP_EQ_I64::Inst_VOP3__V_CMP_EQ_I64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_eq_i64", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_EQ_I64

Inst_VOP3__V_CMP_EQ_I64::~Inst_VOP3__V_CMP_EQ_I64() {
} // ~Inst_VOP3__V_CMP_EQ_I64

// --- description from .arch file ---
// D.u64[threadID] = (S0 == S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_EQ_I64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] == src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_LE_I64 class methods ---

Inst_VOP3__V_CMP_LE_I64::Inst_VOP3__V_CMP_LE_I64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_le_i64", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_LE_I64

Inst_VOP3__V_CMP_LE_I64::~Inst_VOP3__V_CMP_LE_I64() {
} // ~Inst_VOP3__V_CMP_LE_I64

// --- description from .arch file ---
// D.u64[threadID] = (S0 <= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_LE_I64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] <= src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_GT_I64 class methods ---

Inst_VOP3__V_CMP_GT_I64::Inst_VOP3__V_CMP_GT_I64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_gt_i64", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_GT_I64

Inst_VOP3__V_CMP_GT_I64::~Inst_VOP3__V_CMP_GT_I64() {
} // ~Inst_VOP3__V_CMP_GT_I64

// --- description from .arch file ---
// D.u64[threadID] = (S0 > S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_GT_I64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] > src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_NE_I64 class methods ---

Inst_VOP3__V_CMP_NE_I64::Inst_VOP3__V_CMP_NE_I64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_ne_i64", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_NE_I64

Inst_VOP3__V_CMP_NE_I64::~Inst_VOP3__V_CMP_NE_I64() {
} // ~Inst_VOP3__V_CMP_NE_I64

// --- description from .arch file ---
// D.u64[threadID] = (S0 <> S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_NE_I64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] != src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_GE_I64 class methods ---

Inst_VOP3__V_CMP_GE_I64::Inst_VOP3__V_CMP_GE_I64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_ge_i64", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_GE_I64

Inst_VOP3__V_CMP_GE_I64::~Inst_VOP3__V_CMP_GE_I64() {
} // ~Inst_VOP3__V_CMP_GE_I64

// --- description from .arch file ---
// D.u64[threadID] = (S0 >= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_GE_I64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] >= src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_T_I64 class methods ---

Inst_VOP3__V_CMP_T_I64::Inst_VOP3__V_CMP_T_I64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_t_i64", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_T_I64

Inst_VOP3__V_CMP_T_I64::~Inst_VOP3__V_CMP_T_I64() {} // ~Inst_VOP3__V_CMP_T_I64

// --- description from .arch file ---
// D.u64[threadID] = 1; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_T_I64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 1);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_F_U64 class methods ---

Inst_VOP3__V_CMP_F_U64::Inst_VOP3__V_CMP_F_U64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_f_u64", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_F_U64

Inst_VOP3__V_CMP_F_U64::~Inst_VOP3__V_CMP_F_U64() {} // ~Inst_VOP3__V_CMP_F_U64

// --- description from .arch file ---
// D.u64[threadID] = 0; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_F_U64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_LT_U64 class methods ---

Inst_VOP3__V_CMP_LT_U64::Inst_VOP3__V_CMP_LT_U64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_lt_u64", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_LT_U64

Inst_VOP3__V_CMP_LT_U64::~Inst_VOP3__V_CMP_LT_U64() {
} // ~Inst_VOP3__V_CMP_LT_U64

// --- description from .arch file ---
// D.u64[threadID] = (S0 < S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_LT_U64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] < src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_EQ_U64 class methods ---

Inst_VOP3__V_CMP_EQ_U64::Inst_VOP3__V_CMP_EQ_U64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_eq_u64", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_EQ_U64

Inst_VOP3__V_CMP_EQ_U64::~Inst_VOP3__V_CMP_EQ_U64() {
} // ~Inst_VOP3__V_CMP_EQ_U64

// --- description from .arch file ---
// D.u64[threadID] = (S0 == S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_EQ_U64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] == src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_LE_U64 class methods ---

Inst_VOP3__V_CMP_LE_U64::Inst_VOP3__V_CMP_LE_U64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_le_u64", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_LE_U64

Inst_VOP3__V_CMP_LE_U64::~Inst_VOP3__V_CMP_LE_U64() {
} // ~Inst_VOP3__V_CMP_LE_U64

// --- description from .arch file ---
// D.u64[threadID] = (S0 <= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_LE_U64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] <= src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_GT_U64 class methods ---

Inst_VOP3__V_CMP_GT_U64::Inst_VOP3__V_CMP_GT_U64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_gt_u64", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_GT_U64

Inst_VOP3__V_CMP_GT_U64::~Inst_VOP3__V_CMP_GT_U64() {
} // ~Inst_VOP3__V_CMP_GT_U64

// --- description from .arch file ---
// D.u64[threadID] = (S0 > S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_GT_U64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] > src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_NE_U64 class methods ---

Inst_VOP3__V_CMP_NE_U64::Inst_VOP3__V_CMP_NE_U64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_ne_u64", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_NE_U64

Inst_VOP3__V_CMP_NE_U64::~Inst_VOP3__V_CMP_NE_U64() {
} // ~Inst_VOP3__V_CMP_NE_U64

// --- description from .arch file ---
// D.u64[threadID] = (S0 <> S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_NE_U64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] != src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_GE_U64 class methods ---

Inst_VOP3__V_CMP_GE_U64::Inst_VOP3__V_CMP_GE_U64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_ge_u64", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_GE_U64

Inst_VOP3__V_CMP_GE_U64::~Inst_VOP3__V_CMP_GE_U64() {
} // ~Inst_VOP3__V_CMP_GE_U64

// --- description from .arch file ---
// D.u64[threadID] = (S0 >= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_GE_U64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] >= src1[lane] ? 1 : 0);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMP_T_U64 class methods ---

Inst_VOP3__V_CMP_T_U64::Inst_VOP3__V_CMP_T_U64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmp_t_u64", true)
{
    setFlag(ALU);
} // Inst_VOP3__V_CMP_T_U64

Inst_VOP3__V_CMP_T_U64::~Inst_VOP3__V_CMP_T_U64() {} // ~Inst_VOP3__V_CMP_T_U64

// --- description from .arch file ---
// D.u64[threadID] = 1; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMP_T_U64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 1);
        }
    }

    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_F_I64 class methods ---

Inst_VOP3__V_CMPX_F_I64::Inst_VOP3__V_CMPX_F_I64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_f_i64", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_F_I64

Inst_VOP3__V_CMPX_F_I64::~Inst_VOP3__V_CMPX_F_I64() {
} // ~Inst_VOP3__V_CMPX_F_I64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = 0; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_F_I64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_LT_I64 class methods ---

Inst_VOP3__V_CMPX_LT_I64::Inst_VOP3__V_CMPX_LT_I64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_lt_i64", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_LT_I64

Inst_VOP3__V_CMPX_LT_I64::~Inst_VOP3__V_CMPX_LT_I64() {
} // ~Inst_VOP3__V_CMPX_LT_I64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 < S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_LT_I64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] < src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_EQ_I64 class methods ---

Inst_VOP3__V_CMPX_EQ_I64::Inst_VOP3__V_CMPX_EQ_I64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_eq_i64", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_EQ_I64

Inst_VOP3__V_CMPX_EQ_I64::~Inst_VOP3__V_CMPX_EQ_I64() {
} // ~Inst_VOP3__V_CMPX_EQ_I64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 == S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_EQ_I64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] == src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_LE_I64 class methods ---

Inst_VOP3__V_CMPX_LE_I64::Inst_VOP3__V_CMPX_LE_I64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_le_i64", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_LE_I64

Inst_VOP3__V_CMPX_LE_I64::~Inst_VOP3__V_CMPX_LE_I64() {
} // ~Inst_VOP3__V_CMPX_LE_I64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 <= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_LE_I64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] <= src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_GT_I64 class methods ---

Inst_VOP3__V_CMPX_GT_I64::Inst_VOP3__V_CMPX_GT_I64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_gt_i64", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_GT_I64

Inst_VOP3__V_CMPX_GT_I64::~Inst_VOP3__V_CMPX_GT_I64() {
} // ~Inst_VOP3__V_CMPX_GT_I64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 > S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_GT_I64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] > src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_NE_I64 class methods ---

Inst_VOP3__V_CMPX_NE_I64::Inst_VOP3__V_CMPX_NE_I64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_ne_i64", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_NE_I64

Inst_VOP3__V_CMPX_NE_I64::~Inst_VOP3__V_CMPX_NE_I64() {
} // ~Inst_VOP3__V_CMPX_NE_I64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 <> S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_NE_I64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] != src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_GE_I64 class methods ---

Inst_VOP3__V_CMPX_GE_I64::Inst_VOP3__V_CMPX_GE_I64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_ge_i64", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_GE_I64

Inst_VOP3__V_CMPX_GE_I64::~Inst_VOP3__V_CMPX_GE_I64() {
} // ~Inst_VOP3__V_CMPX_GE_I64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 >= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_GE_I64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandI64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandI64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] >= src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_T_I64 class methods ---

Inst_VOP3__V_CMPX_T_I64::Inst_VOP3__V_CMPX_T_I64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_t_i64", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_T_I64

Inst_VOP3__V_CMPX_T_I64::~Inst_VOP3__V_CMPX_T_I64() {
} // ~Inst_VOP3__V_CMPX_T_I64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = 1; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_T_I64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 1);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_F_U64 class methods ---

Inst_VOP3__V_CMPX_F_U64::Inst_VOP3__V_CMPX_F_U64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_f_u64", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_F_U64

Inst_VOP3__V_CMPX_F_U64::~Inst_VOP3__V_CMPX_F_U64() {
} // ~Inst_VOP3__V_CMPX_F_U64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = 0; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_F_U64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sdst.setBit(lane, 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_LT_U64 class methods ---

Inst_VOP3__V_CMPX_LT_U64::Inst_VOP3__V_CMPX_LT_U64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_lt_u64", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_LT_U64

Inst_VOP3__V_CMPX_LT_U64::~Inst_VOP3__V_CMPX_LT_U64() {
} // ~Inst_VOP3__V_CMPX_LT_U64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 < S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_LT_U64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] < src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_EQ_U64 class methods ---

Inst_VOP3__V_CMPX_EQ_U64::Inst_VOP3__V_CMPX_EQ_U64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_eq_u64", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_EQ_U64

Inst_VOP3__V_CMPX_EQ_U64::~Inst_VOP3__V_CMPX_EQ_U64() {
} // ~Inst_VOP3__V_CMPX_EQ_U64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 == S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_EQ_U64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] == src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_LE_U64 class methods ---

Inst_VOP3__V_CMPX_LE_U64::Inst_VOP3__V_CMPX_LE_U64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_le_u64", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_LE_U64

Inst_VOP3__V_CMPX_LE_U64::~Inst_VOP3__V_CMPX_LE_U64() {
} // ~Inst_VOP3__V_CMPX_LE_U64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 <= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_LE_U64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] <= src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_GT_U64 class methods ---

Inst_VOP3__V_CMPX_GT_U64::Inst_VOP3__V_CMPX_GT_U64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_gt_u64", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_GT_U64

Inst_VOP3__V_CMPX_GT_U64::~Inst_VOP3__V_CMPX_GT_U64() {
} // ~Inst_VOP3__V_CMPX_GT_U64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 > S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_GT_U64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] > src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_NE_U64 class methods ---

Inst_VOP3__V_CMPX_NE_U64::Inst_VOP3__V_CMPX_NE_U64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_ne_u64", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_NE_U64

Inst_VOP3__V_CMPX_NE_U64::~Inst_VOP3__V_CMPX_NE_U64() {
} // ~Inst_VOP3__V_CMPX_NE_U64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 <> S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_NE_U64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] != src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_GE_U64 class methods ---

Inst_VOP3__V_CMPX_GE_U64::Inst_VOP3__V_CMPX_GE_U64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_ge_u64", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_GE_U64

Inst_VOP3__V_CMPX_GE_U64::~Inst_VOP3__V_CMPX_GE_U64() {
} // ~Inst_VOP3__V_CMPX_GE_U64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = (S0 >= S1); D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_GE_U64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstVecOperandU64 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU64 src1(gpuDynInst, extData.SRC1);
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, src0[lane] >= src1[lane] ? 1 : 0);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute

// --- Inst_VOP3__V_CMPX_T_U64 class methods ---

Inst_VOP3__V_CMPX_T_U64::Inst_VOP3__V_CMPX_T_U64(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, "v_cmpx_t_u64", true)
{
    setFlag(ALU);
    setFlag(WritesEXEC);
} // Inst_VOP3__V_CMPX_T_U64

Inst_VOP3__V_CMPX_T_U64::~Inst_VOP3__V_CMPX_T_U64() {
} // ~Inst_VOP3__V_CMPX_T_U64

// --- description from .arch file ---
// EXEC,D.u64[threadID] = 1; D = VCC in VOPC encoding.
void
Inst_VOP3__V_CMPX_T_U64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ScalarOperandU64 sdst(gpuDynInst, instData.VDST);

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
            sdst.setBit(lane, 1);
        }
    }

    wf->execMask() = sdst.rawData();
    sdst.write();
} // execute
} // namespace VegaISA
} // namespace gem5
