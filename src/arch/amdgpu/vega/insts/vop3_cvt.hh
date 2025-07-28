/*
 * Copyright (c) 2025 Advanced Micro Devices, Inc.
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

#ifndef __ARCH_AMDGPU_VEGA_INSTS_VOP3_CVT_HH__
#define __ARCH_AMDGPU_VEGA_INSTS_VOP3_CVT_HH__

#include <vector>

#include "arch/amdgpu/common/dtype/mxfp_types.hh"
#include "arch/amdgpu/vega/insts/inst_util.hh"

namespace gem5
{

namespace VegaISA
{

/**
 * Base class for all V_CVT_SCALEF32_PK* instructions in MI355X. Only requires
 * destination and source format plus the mnemonic name for the instruction.
 */
template<typename dFMT, typename sFMT, const char **MNEM>
class Inst_VOP3__V_CVT_SCALE_PK : public Inst_VOP3A
{
public:
Inst_VOP3__V_CVT_SCALE_PK(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, *MNEM, false)
{
    setFlag(ALU);
}

~Inst_VOP3__V_CVT_SCALE_PK() { }

void
execute(GPUDynInstPtr gpuDynInst) override
{
    static_assert(dFMT::size() == 32 || dFMT::size() == 16 ||
                  dFMT::size() == 8  || dFMT::size() == 4);
    static_assert(sFMT::size() == 32 || sFMT::size() == 16 ||
                  sFMT::size() == 8  || sFMT::size() == 4);

    Wavefront *wf = gpuDynInst->wavefront();

    // For the operands, there might be an easier way to type these based
    // on dFMT/sFMT. Here we define the possibilities and only read/write
    // the valid ones in an if constexpr conditional.
    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandU32 src1(gpuDynInst, extData.SRC1);
    ConstVecOperandF32 src2(gpuDynInst, extData.SRC2);

    VecOperandU64 vdst64(gpuDynInst, instData.VDST);
    VecOperandU32 vdst32(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    if constexpr (sFMT::size() == 32) {
        src2.readSrc();
    }

    // These are read in the case of dFMT < 32 bit. In those cases the dest
    // register is updated without clobbering the unwritten bits.
    if constexpr (dFMT::size() == 32) {
        vdst64.read();
    } else {
        vdst32.read();
    }

    panic_if(isSDWAInst(), "SDWA not supported for %s", _opcode);
    panic_if(isDPPInst(), "DPP not supported for %s", _opcode);
    panic_if(extData.OMOD, "OMOD not supported for %s", _opcode);
    panic_if(instData.ABS, "ABS not supported for %s", _opcode);
    panic_if(extData.NEG, "NEG not supported for %s", _opcode);

    // For 16 bit source format this is unused. For 8 bit only bit
    // 0 is valid. For 4 bit only bits 0 and 1 are valid.
    [[maybe_unused]] int in_opsel = 0;
    if constexpr (sFMT::size() == 8) {
        in_opsel = instData.OPSEL & 1;
    } else if (sFMT::size() == 4) {
        in_opsel = instData.OPSEL & 3;
    } else {
        in_opsel = 0;
    }

    // If the destination size is 8 bits select the word in vdst using
    // bit 3. If the size is 4 bits select the word using bits 3 and 2
    int out_opsel = 0;
    if constexpr (dFMT::size() == 8) {
        out_opsel = (instData.OPSEL >> 3) & 1;
    } else if (dFMT::size() == 4) {
        out_opsel = (instData.OPSEL >> 2) & 3;
    } else {
        out_opsel = 0;
    }

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            dFMT cvt1, cvt2;

            // When downcasting, scale before conversion otherwise scale
            // after conversion. Read the scale value first in either case.
            float scale_val = 1.0f;
            if constexpr (sFMT::size() == 32) {
                scale_val = src2[lane];
            } else {
                VecElemU32 tmp = src1[lane];
                scale_val = *reinterpret_cast<float*>(&(tmp));
            }

            if constexpr (sFMT::size() == 32) {
                sFMT tmp1(src0[lane]);
                sFMT tmp2(src1[lane]);

                static_assert(dFMT::size() < sFMT::size());
                tmp1.scaleDiv(scale_val);
                tmp2.scaleDiv(scale_val);

                // Implicit convert here
                cvt1 = tmp1;
                cvt2 = tmp2;
            } else {
                auto in = unpackMXOperands<sFMT>(src0[lane], in_opsel);

                if (dFMT::size() < sFMT::size()) {
                    in.first.scaleDiv(scale_val);
                    in.second.scaleDiv(scale_val);
                }

                cvt1 = in.first;
                cvt2 = in.second;
            }

            // Upcasting. Scale after conversion from above.
            if (dFMT::size() >= sFMT::size()) {
                cvt1.scaleMul(scale_val);
                cvt2.scaleMul(scale_val);
            }

            if (instData.CLAMP) {
                cvt1 = std::clamp(float(cvt1), 0.0f, 1.0f);
                cvt2 = std::clamp(float(cvt2), 0.0f, 1.0f);
            }

            if constexpr (dFMT::size() == 32) {
                vdst64[lane] = packMXOperands64(cvt2, cvt1);
            } else if (dFMT::size() == 16) {
                vdst32[lane] = packMXOperands32(cvt2, cvt1);
            } else if (dFMT::size() == 8) {
                uint16_t packed_data = packMXOperands32(cvt2, cvt1);
                vdst32[lane] = insertBits(vdst32[lane], 16 * out_opsel + 15,
                                          16 * out_opsel, packed_data);
            } else {
                uint8_t packed_data = packMXOperands32(cvt2, cvt1);
                vdst32[lane] = insertBits(vdst32[lane], 8 * out_opsel + 7,
                                          8 * out_opsel, packed_data);
            }
        }
    }

    if constexpr (dFMT::size() == 32) {
        vdst64.write();
    } else {
        vdst32.write();
    }
}

int
getNumOperands() override
{
    return numDstRegOperands() + numSrcRegOperands();
}

int
numDstRegOperands() override
{
    return 1;
}

int
numSrcRegOperands() override
{
    if constexpr (dFMT::size() == 32) {
        return 3;
    }

    return 2;
}

int
getOperandSize(int opIdx) override
{
    if constexpr (dFMT::size() == 32) {
        switch (opIdx) {
          case 0: //src_0
            return 4;
          case 1: //src_1
            return 4;
          case 2: //src_2
            return 4;
          case 3: //vdst
            return 8;
          default:
            fatal("op idx %i out of bounds\n", opIdx);
            return -1;
        }
    } else {
        switch (opIdx) {
          case 0: //src_0
            return 4;
          case 1: //src_1
            return 4;
          case 2: //vdst
            return 4;
          default:
            fatal("op idx %i out of bounds\n", opIdx);
            return -1;
        }
    }

    fatal("op idx %i out of bounds\n", opIdx);
    return -1;
}

};

static const char* MNEM__V_CVT_SCALEF32_PK_FP8_F32 =
    "v_cvt_scalef32_pk_fp8_f32";
using Inst_VOP3__V_CVT_SCALEF32_PK_FP8_F32 =
    Inst_VOP3__V_CVT_SCALE_PK<AMDGPU::mxfloat8, AMDGPU::mxfloat32,
                              &MNEM__V_CVT_SCALEF32_PK_FP8_F32>;

static const char* MNEM__V_CVT_SCALEF32_PK_BF8_F32 =
    "v_cvt_scalef32_pk_bf8_f32";
using Inst_VOP3__V_CVT_SCALEF32_PK_BF8_F32 =
    Inst_VOP3__V_CVT_SCALE_PK<AMDGPU::mxbfloat8, AMDGPU::mxfloat32,
                              &MNEM__V_CVT_SCALEF32_PK_BF8_F32>;

static const char* MNEM__V_CVT_SCALEF32_PK_F32_FP8 =
    "v_cvt_scalef32_pk_f32_fp8";
using Inst_VOP3__V_CVT_SCALEF32_PK_F32_FP8 =
    Inst_VOP3__V_CVT_SCALE_PK<AMDGPU::mxfloat32, AMDGPU::mxfloat8,
                              &MNEM__V_CVT_SCALEF32_PK_F32_FP8>;

static const char* MNEM__V_CVT_SCALEF32_PK_F32_BF8 =
    "v_cvt_scalef32_pk_f32_bf8";
using Inst_VOP3__V_CVT_SCALEF32_PK_F32_BF8 =
    Inst_VOP3__V_CVT_SCALE_PK<AMDGPU::mxfloat32, AMDGPU::mxbfloat8,
                              &MNEM__V_CVT_SCALEF32_PK_F32_BF8>;

static const char* MNEM__V_CVT_SCALEF32_PK_FP4_F32 =
    "v_cvt_scalef32_pk_fp4_f32";
using Inst_VOP3__V_CVT_SCALEF32_PK_FP4_F32 =
    Inst_VOP3__V_CVT_SCALE_PK<AMDGPU::mxfp4, AMDGPU::mxfloat32,
                              &MNEM__V_CVT_SCALEF32_PK_FP4_F32>;

static const char* MNEM__V_CVT_SCALEF32_PK_F32_FP4 =
    "v_cvt_scalef32_pk_f32_fp4";
using Inst_VOP3__V_CVT_SCALEF32_PK_F32_FP4 =
    Inst_VOP3__V_CVT_SCALE_PK<AMDGPU::mxfloat32, AMDGPU::mxfp4,
                              &MNEM__V_CVT_SCALEF32_PK_F32_FP4>;

static const char* MNEM__V_CVT_SCALEF32_PK_FP8_F16 =
    "v_cvt_scalef32_pk_fp8_f16";
using Inst_VOP3__V_CVT_SCALEF32_PK_FP8_F16 =
    Inst_VOP3__V_CVT_SCALE_PK<AMDGPU::mxfloat8, AMDGPU::mxfloat16,
                              &MNEM__V_CVT_SCALEF32_PK_FP8_F16>;

static const char* MNEM__V_CVT_SCALEF32_PK_BF8_F16 =
    "v_cvt_scalef32_pk_bf8_f16";
using Inst_VOP3__V_CVT_SCALEF32_PK_BF8_F16 =
    Inst_VOP3__V_CVT_SCALE_PK<AMDGPU::mxbfloat8, AMDGPU::mxfloat16,
                              &MNEM__V_CVT_SCALEF32_PK_BF8_F16>;

static const char* MNEM__V_CVT_SCALEF32_PK_FP8_BF16 =
    "v_cvt_scalef32_pk_fp8_bf16";
using Inst_VOP3__V_CVT_SCALEF32_PK_FP8_BF16 =
    Inst_VOP3__V_CVT_SCALE_PK<AMDGPU::mxfloat8, AMDGPU::mxbfloat16,
                              &MNEM__V_CVT_SCALEF32_PK_FP8_BF16>;

static const char* MNEM__V_CVT_SCALEF32_PK_BF8_BF16 =
    "v_cvt_scalef32_pk_bf8_bf16";
using Inst_VOP3__V_CVT_SCALEF32_PK_BF8_BF16 =
    Inst_VOP3__V_CVT_SCALE_PK<AMDGPU::mxbfloat8, AMDGPU::mxbfloat16,
                              &MNEM__V_CVT_SCALEF32_PK_BF8_BF16>;

static const char* MNEM__V_CVT_SCALEF32_PK_F16_FP8 =
    "v_cvt_scalef32_pk_f16_fp8";
using Inst_VOP3__V_CVT_SCALEF32_PK_F16_FP8 =
    Inst_VOP3__V_CVT_SCALE_PK<AMDGPU::mxfloat16, AMDGPU::mxfloat8,
                              &MNEM__V_CVT_SCALEF32_PK_F16_FP8>;

static const char* MNEM__V_CVT_SCALEF32_PK_F16_BF8 =
    "v_cvt_scalef32_pk_f16_bf8";
using Inst_VOP3__V_CVT_SCALEF32_PK_F16_BF8 =
    Inst_VOP3__V_CVT_SCALE_PK<AMDGPU::mxfloat16, AMDGPU::mxbfloat8,
                              &MNEM__V_CVT_SCALEF32_PK_F16_BF8>;

static const char* MNEM__V_CVT_SCALEF32_PK_FP4_F16 =
    "v_cvt_scalef32_pk_fp4_f16";
using Inst_VOP3__V_CVT_SCALEF32_PK_FP4_F16 =
    Inst_VOP3__V_CVT_SCALE_PK<AMDGPU::mxfp4, AMDGPU::mxfloat16,
                              &MNEM__V_CVT_SCALEF32_PK_FP4_F16>;

static const char* MNEM__V_CVT_SCALEF32_PK_FP4_BF16 =
    "v_cvt_scalef32_pk_fp4_bf16";
using Inst_VOP3__V_CVT_SCALEF32_PK_FP4_BF16 =
    Inst_VOP3__V_CVT_SCALE_PK<AMDGPU::mxfp4, AMDGPU::mxbfloat16,
                              &MNEM__V_CVT_SCALEF32_PK_FP4_BF16>;

static const char* MNEM__V_CVT_SCALEF32_PK_F16_FP4 =
    "v_cvt_scalef32_pk_f16_fp4";
using Inst_VOP3__V_CVT_SCALEF32_PK_F16_FP4 =
    Inst_VOP3__V_CVT_SCALE_PK<AMDGPU::mxfloat16, AMDGPU::mxfp4,
                              &MNEM__V_CVT_SCALEF32_PK_F16_FP4>;

static const char* MNEM__V_CVT_SCALEF32_PK_BF16_FP4 =
    "v_cvt_scalef32_pk_bf16_fp4";
using Inst_VOP3__V_CVT_SCALEF32_PK_BF16_FP4 =
    Inst_VOP3__V_CVT_SCALE_PK<AMDGPU::mxbfloat16, AMDGPU::mxfp4,
                              &MNEM__V_CVT_SCALEF32_PK_BF16_FP4>;

static const char* MNEM__V_CVT_SCALEF32_PK_BF16_FP8 =
    "v_cvt_scalef32_pk_bf16_fp8";
using Inst_VOP3__V_CVT_SCALEF32_PK_BF16_FP8 =
    Inst_VOP3__V_CVT_SCALE_PK<AMDGPU::mxbfloat16, AMDGPU::mxfloat8,
                              &MNEM__V_CVT_SCALEF32_PK_BF16_FP8>;

static const char* MNEM__V_CVT_SCALEF32_PK_BF16_BF8 =
    "v_cvt_scalef32_pk_bf16_bf8";
using Inst_VOP3__V_CVT_SCALEF32_PK_BF16_BF8 =
    Inst_VOP3__V_CVT_SCALE_PK<AMDGPU::mxbfloat16, AMDGPU::mxfloat8,
                              &MNEM__V_CVT_SCALEF32_PK_BF16_BF8>;



/**
 * Base class for all V_CVT_SCALEF32* instructions in MI355X which are NOT
 * packed. Only requires destination and source format plus the mnemonic name
 * for the instruction.
 */
template<typename dFMT, typename sFMT, const char **MNEM>
class Inst_VOP3__V_CVT_SCALE : public Inst_VOP3A
{
public:
Inst_VOP3__V_CVT_SCALE(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, *MNEM, false)
{
    setFlag(ALU);
}

~Inst_VOP3__V_CVT_SCALE() { }

dFMT
omodModifier(dFMT val, unsigned omod)
{
    // These implicitly convert to F32 first. However that is always larger
    // than the largest source format so there should be not precision loss.
    assert(omod < 4);

    if (omod == 1) return val * 2.0f;
    if (omod == 2) return val * 4.0f;
    if (omod == 3) return val / 2.0f;

    return val;
}

void
execute(GPUDynInstPtr gpuDynInst) override
{
    // Currently only 4 conversions
    static_assert(dFMT::size() == 32 || dFMT::size() == 16);
    static_assert(sFMT::size() == 8);

    Wavefront *wf = gpuDynInst->wavefront();

    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0);
    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);

    // The instruction spec does not mention existing bits in the dest be
    // preserved, so we do not read this before modifying and clobber it.
    VecOperandU32 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();

    panic_if(isSDWAInst(), "SDWA not supported for %s", _opcode);
    panic_if(isDPPInst(), "DPP not supported for %s", _opcode);
    panic_if(instData.ABS, "ABS not supported for %s", _opcode);
    panic_if(extData.NEG, "NEG not supported for %s", _opcode);

    // Two bits to select the byte in the dword. No output opsel bit is
    // mentioned in the spec.
    int in_opsel = instData.OPSEL & 3;

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sFMT in(bits(src0[lane], 8 * in_opsel + 7, 8 * in_opsel));
            dFMT cvt;

            // Implicit convert
            cvt = in;

            float scale_val = src1[lane];

            // Upcast only in this template. Apply after converting.
            cvt.scaleMul(scale_val);

            // Not marked OPF_NOOMOD, apply output modifiers before clamp.
            cvt = omodModifier(cvt, extData.OMOD);

            if (instData.CLAMP) {
                cvt = std::clamp(float(cvt), 0.0f, 1.0f);
            }

            // Write raw data back to register
            vdst[lane] = cvt.data >> (32 - dFMT::size());
        }
    }

    vdst.write();
}

int
getNumOperands() override
{
    return numDstRegOperands() + numSrcRegOperands();
}

int
numDstRegOperands() override
{
    return 1;
}

int
numSrcRegOperands() override
{
    return 2;
}

int
getOperandSize(int opIdx) override
{
    switch (opIdx) {
      case 0: //src_0
        return 4;
      case 1: //src_1
        return 4;
      case 2: //vdst
        return 4;
      default:
        fatal("op idx %i out of bounds\n", opIdx);
        return -1;
    }
}

};

static const char* MNEM__V_CVT_SCALEF32_F16_BF8 =
    "v_cvt_scalef32_f16_bf8";
using Inst_VOP3__V_CVT_SCALEF32_F16_BF8 =
    Inst_VOP3__V_CVT_SCALE<AMDGPU::mxfloat16, AMDGPU::mxbfloat8,
                              &MNEM__V_CVT_SCALEF32_F16_BF8>;

static const char* MNEM__V_CVT_SCALEF32_F16_FP8 =
    "v_cvt_scalef32_f16_fp8";
using Inst_VOP3__V_CVT_SCALEF32_F16_FP8 =
    Inst_VOP3__V_CVT_SCALE<AMDGPU::mxfloat16, AMDGPU::mxfloat8,
                              &MNEM__V_CVT_SCALEF32_F16_FP8>;

static const char* MNEM__V_CVT_SCALEF32_F32_BF8 =
    "v_cvt_scalef32_f32_bf8";
using Inst_VOP3__V_CVT_SCALEF32_F32_BF8 =
    Inst_VOP3__V_CVT_SCALE<AMDGPU::mxfloat32, AMDGPU::mxbfloat8,
                              &MNEM__V_CVT_SCALEF32_F32_BF8>;

static const char* MNEM__V_CVT_SCALEF32_F32_FP8 =
    "v_cvt_scalef32_f32_fp8";
using Inst_VOP3__V_CVT_SCALEF32_F32_FP8 =
    Inst_VOP3__V_CVT_SCALE<AMDGPU::mxfloat32, AMDGPU::mxfloat8,
                              &MNEM__V_CVT_SCALEF32_F32_FP8>;




/**
 * Base class for all V_CVT_SCALEF32_PK32* MI355X instructions (except with F32
 * inputs). This basically handles special case of F6 types which do not divide
 * 32-bits. Rather than waste 2-bits per-GPR, there are 32 values packed across
 * 6 VGPRs. Only requires destination and source format plus the mnemonic name
 * for the instruction.
 */
template<typename dFMT, typename sFMT, const char **MNEM>
class Inst_VOP3__V_CVT_SCALE_PK32 : public Inst_VOP3A
{
public:
Inst_VOP3__V_CVT_SCALE_PK32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, *MNEM, false)
{
    setFlag(ALU);
}

~Inst_VOP3__V_CVT_SCALE_PK32() { }

void
execute(GPUDynInstPtr gpuDynInst) override
{
    static_assert(dFMT::size() == 32 ||
            dFMT::size() == 16 || dFMT::size() == 6);
    static_assert(sFMT::size() == 16 || sFMT::size() == 6);

    // There are 32 values packed into a huge operand. These are called
    // components in the spec.
    constexpr const int components = sFMT::size() == 32 ? 16 : 32;
    size_t input_regs = getOperandSize(0) / 4;
    size_t output_regs = getOperandSize(2) / 4;

    Wavefront *wf = gpuDynInst->wavefront();

    // The gem5 operand types are really only handy up to 64 bits. For BF
    // operand sizes such as in these instructions, just create an array of
    // 32-bit registers to use.
    std::vector<ConstVecOperandU32> src0;
    src0.reserve(input_regs);
    for (int reg = 0; reg < input_regs; ++reg) {
        src0.emplace_back(gpuDynInst, extData.SRC0 + reg);
        src0[reg].readSrc();
    }

    ConstVecOperandF32 src1(gpuDynInst, extData.SRC1);
    src1.readSrc();

    std::vector<typename std::aligned_storage<sizeof(VecOperandU32),
                                              alignof(VecOperandU32)>::type>
        _vdst(output_regs);
    VecOperandU32* vdst =
        std::launder(reinterpret_cast<VecOperandU32*>(_vdst.data()));
    for (int reg = 0; reg < output_regs; ++reg) {
        new (&vdst[reg]) VecOperandU32(gpuDynInst, instData.VDST + reg);
    }

    panic_if(isSDWAInst(), "SDWA not supported for %s", _opcode);
    panic_if(isDPPInst(), "DPP not supported for %s", _opcode);
    panic_if(instData.CLAMP, "CLAMP not supported for %s", _opcode);
    panic_if(extData.OMOD, "OMOD not supported for %s", _opcode);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            PackedReg<sFMT::size() * components, sFMT::size()> in_reg;
            PackedReg<dFMT::size() * components, dFMT::size()> out_reg;

            for (int reg = 0; reg < input_regs; ++reg) {
                in_reg.setDword(reg, src0[reg][lane]);
            }

            for (int pass = 0; pass < components; ++pass) {
                sFMT in;
                dFMT out;

                // When downcasting, scale before conversion otherwise scale
                // after conversion. Read the scale value first in either case.
                float scale_val = src1[lane];
                if (instData.ABS & 2) {
                    scale_val = std::fabs(scale_val);
                }
                if (extData.NEG & 2) {
                    scale_val = -scale_val;
                }

                // Note: Due to the union of a signed int and bitfield struct,
                // the data is [31:(32 - sFMT::size())], so we must align this
                // otherwise the conversions will result in a zero value.
                in.data = in_reg.getElem(pass) << (32 - sFMT::size());

                // Apply ABS, NEG
                if (instData.ABS & 1 && float(in) < 0.0f) {
                    in = -in;
                }
                if (extData.NEG & 1) {
                    in = -in;
                }

                // Downcasting. Apply scale before converting.
                if constexpr (dFMT::size() < sFMT::size()) {
                    out.scaleDiv(scale_val);
                }

                out = in; // Implicit conversion happens here.

                // Upcasting. Apply scale after converting.
                if constexpr (dFMT::size() >= sFMT::size()) {
                    out.scaleMul(scale_val);
                }

                // Apply ABS, NEG
                if (instData.ABS & 8 && float(out) < 0.0f) {
                    out = -out;
                }
                if (extData.NEG & 8) {
                    out = -out;
                }

                out_reg.setElem(pass, out.data >> (32 - dFMT::size()));
            }

            for (int reg = 0; reg < output_regs; ++reg) {
                vdst[reg][lane] = out_reg.getDword(reg);
            }
        }
    }

    for (int reg = 0; reg < output_regs; ++reg) {
        vdst[reg].write();
    }
}

int
getNumOperands() override
{
    return numDstRegOperands() + numSrcRegOperands();
}

int
numDstRegOperands() override
{
    return 1;
}

int
numSrcRegOperands() override
{
    return 2;
}

int
getOperandSize(int opIdx) override
{
    switch (opIdx) {
      case 0: //src_0
        if constexpr (sFMT::size() == 32) {
            return sFMT::size() * 2;
        } else {
            return sFMT::size() * 4;
        }
      case 1: //src_1
        return 4;
      case 2: //vdst
        return dFMT::size() * 4;
      default:
        fatal("op idx %i out of bounds\n", opIdx);
        return -1;
    }
}

};

static const char* MNEM__V_CVT_SCALEF32_PK32_BF16_BF6 =
    "v_cvt_scalef32_pk32_bf16_bf6";
using Inst_VOP3__V_CVT_SCALEF32_PK32_BF16_BF6 =
    Inst_VOP3__V_CVT_SCALE_PK32<AMDGPU::mxbfloat16,
                                AMDGPU::mxbf6,
                                &MNEM__V_CVT_SCALEF32_PK32_BF16_BF6>;

static const char* MNEM__V_CVT_SCALEF32_PK32_BF16_FP6 =
    "v_cvt_scalef32_pk32_bf16_fp6";
using Inst_VOP3__V_CVT_SCALEF32_PK32_BF16_FP6 =
    Inst_VOP3__V_CVT_SCALE_PK32<AMDGPU::mxbfloat16,
                                AMDGPU::mxfp6,
                                &MNEM__V_CVT_SCALEF32_PK32_BF16_FP6>;

static const char* MNEM__V_CVT_SCALEF32_PK32_BF6_BF16 =
    "v_cvt_scalef32_pk32_bf6_bf16";
using Inst_VOP3__V_CVT_SCALEF32_PK32_BF6_BF16 =
    Inst_VOP3__V_CVT_SCALE_PK32<AMDGPU::mxbf6,
                                AMDGPU::mxbfloat16,
                                &MNEM__V_CVT_SCALEF32_PK32_BF6_BF16>;

static const char* MNEM__V_CVT_SCALEF32_PK32_BF6_F16 =
    "v_cvt_scalef32_pk32_bf6_f16";
using Inst_VOP3__V_CVT_SCALEF32_PK32_BF6_F16 =
    Inst_VOP3__V_CVT_SCALE_PK32<AMDGPU::mxbf6,
                                AMDGPU::mxfloat16,
                                &MNEM__V_CVT_SCALEF32_PK32_BF6_F16>;

static const char* MNEM__V_CVT_SCALEF32_PK32_F16_BF6 =
    "v_cvt_scalef32_pk32_f16_bf6";
using Inst_VOP3__V_CVT_SCALEF32_PK32_F16_BF6 =
    Inst_VOP3__V_CVT_SCALE_PK32<AMDGPU::mxfloat16,
                                AMDGPU::mxbf6,
                                &MNEM__V_CVT_SCALEF32_PK32_F16_BF6>;

static const char* MNEM__V_CVT_SCALEF32_PK32_F16_FP6 =
    "v_cvt_scalef32_pk32_f16_fp6";
using Inst_VOP3__V_CVT_SCALEF32_PK32_F16_FP6 =
    Inst_VOP3__V_CVT_SCALE_PK32<AMDGPU::mxfloat16,
                                AMDGPU::mxfp6,
                                &MNEM__V_CVT_SCALEF32_PK32_F16_FP6>;

static const char* MNEM__V_CVT_SCALEF32_PK32_F32_BF6 =
    "v_cvt_scalef32_pk32_f32_bf6";
using Inst_VOP3__V_CVT_SCALEF32_PK32_F32_BF6 =
    Inst_VOP3__V_CVT_SCALE_PK32<AMDGPU::mxfloat32,
                                AMDGPU::mxbf6,
                                &MNEM__V_CVT_SCALEF32_PK32_F32_BF6>;

static const char* MNEM__V_CVT_SCALEF32_PK32_F32_FP6 =
    "v_cvt_scalef32_pk32_f32_fp6";
using Inst_VOP3__V_CVT_SCALEF32_PK32_F32_FP6 =
    Inst_VOP3__V_CVT_SCALE_PK32<AMDGPU::mxfloat32,
                                AMDGPU::mxfp6,
                                &MNEM__V_CVT_SCALEF32_PK32_F32_FP6>;

static const char* MNEM__V_CVT_SCALEF32_PK32_FP6_BF16 =
    "v_cvt_scalef32_pk32_fp6_bf16";
using Inst_VOP3__V_CVT_SCALEF32_PK32_FP6_BF16 =
    Inst_VOP3__V_CVT_SCALE_PK32<AMDGPU::mxfp6,
                                AMDGPU::mxbfloat16,
                                &MNEM__V_CVT_SCALEF32_PK32_FP6_BF16>;

static const char* MNEM__V_CVT_SCALEF32_PK32_FP6_F16 =
    "v_cvt_scalef32_pk32_fp6_f16";
using Inst_VOP3__V_CVT_SCALEF32_PK32_FP6_F16 =
    Inst_VOP3__V_CVT_SCALE_PK32<AMDGPU::mxfp6,
                                AMDGPU::mxfloat16,
                                &MNEM__V_CVT_SCALEF32_PK32_FP6_F16>;


/**
 * Base class for all V_CVT_SCALEF32_PK32* MI355X instructions which have F32
 * inputs. Only requires destination and source format plus the mnemonic name
 * for the instruction.
 */
template<typename dFMT, typename sFMT, const char **MNEM>
class Inst_VOP3__V_CVT_SCALEF32_2XPK16_F32 : public Inst_VOP3A
{
public:
Inst_VOP3__V_CVT_SCALEF32_2XPK16_F32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, *MNEM, false)
{
    setFlag(ALU);
}

~Inst_VOP3__V_CVT_SCALEF32_2XPK16_F32() { }

void
execute(GPUDynInstPtr gpuDynInst) override
{
    static_assert(dFMT::size() == 6);
    static_assert(sFMT::size() == 32);

    // There are 32 values over two source operands which have 16 values.
    // These are called components in the spec.
    constexpr const int components = 32;
    size_t input_regs = getOperandSize(0) / 4;
    size_t output_regs = getOperandSize(3) / 4;

    Wavefront *wf = gpuDynInst->wavefront();

    // The gem5 operand types are really only handy up to 64 bits. For BF
    // operand sizes such as in these instructions, just create an array of
    // 32-bit registers to use.
    std::vector<typename std::aligned_storage<
        sizeof(ConstVecOperandU32), alignof(ConstVecOperandU32)>::type>
        _src0(input_regs);
    ConstVecOperandU32* src0 =
        std::launder(reinterpret_cast<ConstVecOperandU32*>(_src0.data()));
    for (int reg = 0; reg < input_regs; ++reg) {
        new (&src0[reg]) ConstVecOperandU32(gpuDynInst, extData.SRC0 + reg);
        src0[reg].readSrc();
    }

    std::vector<typename std::aligned_storage<
        sizeof(ConstVecOperandU32), alignof(ConstVecOperandU32)>::type>
        _src1(input_regs);
    ConstVecOperandU32* src1 =
        std::launder(reinterpret_cast<ConstVecOperandU32*>(_src1.data()));
    for (int reg = 0; reg < input_regs; ++reg) {
        new (&src1[reg]) ConstVecOperandU32(gpuDynInst, extData.SRC1 + reg);
        src1[reg].readSrc();
    }

    ConstVecOperandF32 src2(gpuDynInst, extData.SRC2);
    src2.readSrc();

    std::vector<typename std::aligned_storage<sizeof(VecOperandU32),
                                              alignof(VecOperandU32)>::type>
        _vdst(output_regs);
    VecOperandU32* vdst =
        std::launder(reinterpret_cast<VecOperandU32*>(_vdst.data()));
    for (int reg = 0; reg < output_regs; ++reg) {
        new (&vdst[reg]) VecOperandU32(gpuDynInst, instData.VDST + reg);
    }

    panic_if(isSDWAInst(), "SDWA not supported for %s", _opcode);
    panic_if(isDPPInst(), "DPP not supported for %s", _opcode);
    panic_if(instData.CLAMP, "CLAMP not supported for %s", _opcode);
    panic_if(extData.OMOD, "OMOD not supported for %s", _opcode);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            PackedReg<sFMT::size() * components, sFMT::size()> in_reg;
            PackedReg<dFMT::size() * components, dFMT::size()> out_reg;

            for (int reg = 0; reg < input_regs; ++reg) {
                in_reg.setDword(reg * 2, src0[reg][lane]);
            }

            for (int reg = 0; reg < input_regs; ++reg) {
                in_reg.setDword(reg * 2 + 1, src1[reg][lane]);
            }

            for (int pass = 0; pass < components; ++pass) {
                sFMT in;
                dFMT out;

                // Note: Due to the union of a signed int and bitfield struct,
                // the data is [31:(32 - sFMT::size())], so we must align this
                // otherwise the conversions will result in a zero value.
                in.data = in_reg.getElem(pass) << (32 - sFMT::size());

                // Apply ABS, NEG
                if (instData.ABS & 1 && float(in) < 0.0f) {
                    in = -in;
                }
                if (extData.NEG & 1) {
                    in = -in;
                }

                // Only downcasts in this template. Scale before converting.
                float scale_val = src2[lane];
                if (instData.ABS & 2) {
                    scale_val = std::fabs(scale_val);
                }
                if (extData.NEG & 2) {
                    scale_val = -scale_val;
                }

                in.scaleDiv(scale_val);

                out = in; // Implicit conversion happens here.

                // Apply ABS, NEG
                if (instData.ABS & 8 && float(out) < 0.0f) {
                    out = -out;
                }
                if (extData.NEG & 8) {
                    out = -out;
                }

                out_reg.setElem(pass, out.data >> (32 - dFMT::size()));
            }

            for (int reg = 0; reg < output_regs; ++reg) {
                vdst[reg][lane] = out_reg.getDword(reg);
            }
        }
    }

    for (int reg = 0; reg < output_regs; ++reg) {
        vdst[reg].write();
    }
}

int
getNumOperands() override
{
    return numDstRegOperands() + numSrcRegOperands();
}

int
numDstRegOperands() override
{
    return 1;
}

int
numSrcRegOperands() override
{
    return 3;
}

int
getOperandSize(int opIdx) override
{
    switch (opIdx) {
      case 0: //src_0
        return sFMT::size() * 2;
      case 1: //src_1
        return sFMT::size() * 2;
      case 2: //src_2
        return 4;
      case 3: //vdst
        return dFMT::size() * 4;
      default:
        fatal("op idx %i out of bounds\n", opIdx);
        return -1;
    }
}

};

static const char* MNEM__V_CVT_SCALEF32_PK32_BF6_F32 =
    "v_cvt_scalef32_2xpk16_bf6_f32";
using Inst_VOP3__V_CVT_SCALEF32_2XPK16_BF6_F32 =
    Inst_VOP3__V_CVT_SCALEF32_2XPK16_F32<AMDGPU::mxbf6,
                                         AMDGPU::mxfloat32,
                                         &MNEM__V_CVT_SCALEF32_PK32_BF6_F32>;

static const char* MNEM__V_CVT_SCALEF32_PK32_FP6_F32 =
    "v_cvt_scalef32_2xpk16_fp6_f32";
using Inst_VOP3__V_CVT_SCALEF32_2XPK16_FP6_F32 =
    Inst_VOP3__V_CVT_SCALEF32_2XPK16_F32<AMDGPU::mxfp6,
                                         AMDGPU::mxfloat32,
                                         &MNEM__V_CVT_SCALEF32_PK32_FP6_F32>;


/**
 * Base class for all V_CVT_SCALEF32_SR_* instructions in MI355X which are NOT
 * packed. Only requires destination and source format plus the mnemonic name
 * for the instruction.
 */
template<typename dFMT, typename sFMT, const char **MNEM>
class Inst_VOP3__V_CVT_SCALEF32_SR : public Inst_VOP3A
{
public:
Inst_VOP3__V_CVT_SCALEF32_SR(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, *MNEM, false)
{
    setFlag(ALU);
}

~Inst_VOP3__V_CVT_SCALEF32_SR() { }

void
execute(GPUDynInstPtr gpuDynInst) override
{
    // Currently only 5 conversions
    static_assert(dFMT::size() == 8);
    static_assert(sFMT::size() == 32 || sFMT::size() == 16);

    Wavefront *wf = gpuDynInst->wavefront();

    ConstVecOperandU32 src0(gpuDynInst, extData.SRC0); // input
    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1); // seed
    ConstVecOperandF32 src2(gpuDynInst, extData.SRC2); // scale
    VecOperandU64 vdst(gpuDynInst, instData.VDST);

    src0.readSrc();
    src1.readSrc();
    src2.readSrc();
    vdst.read();

    panic_if(isSDWAInst(), "SDWA not supported for %s", _opcode);
    panic_if(isDPPInst(), "DPP not supported for %s", _opcode);
    panic_if(extData.OMOD, "OMOD not supported for %s", _opcode);
    panic_if(instData.CLAMP, "CLAMP not supported for %s", _opcode);

    // Two bits [3:2] select the byte in the output dword. No input opsel bit
    // is mentioned in the spec.
    int out_byte = bits(instData.OPSEL, 3, 2);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sFMT in(src0[lane]);
            dFMT cvt;

            if (instData.ABS & 1 && float(in) < 0.0f) {
                in = -in;
            }
            if (extData.NEG & 1) {
                in = -in;
            }

            VecElemI32 seed_val = src1[lane];

            if (instData.ABS & 2) {
                seed_val = std::abs(seed_val);
            }
            if (extData.NEG & 2) {
                seed_val = -seed_val;
            }

            float scale_val = src2[lane];
            if (instData.ABS & 4) {
                scale_val = std::fabs(scale_val);
            }
            if (extData.NEG & 4) {
                scale_val = -scale_val;
            }

            // Only downcasts in this template. Apply scale before converting.
            in.scaleDiv(scale_val);

            using sInfo = decltype(in.getFmt());
            using dInfo = decltype(cvt.getFmt());
            dInfo cvt_info = AMDGPU::convertMXFP<dInfo, sInfo>(
                in.getFmt(), AMDGPU::roundStochastic, seed_val
            );
            cvt.setFmt(cvt_info);

            if (instData.ABS & 8 && float(cvt) < 0.0f) {
                cvt = -cvt;
            }
            if (extData.NEG & 8) {
                cvt = -cvt;
            }

            // Write raw data back to register
            vdst[lane] = insertBits(vdst[lane], out_byte * 8 + 7, out_byte * 8,
                                    bits(cvt.data, 31, 32 - dFMT::size()));
        }
    }

    vdst.write();
}

int
getNumOperands() override
{
    return numDstRegOperands() + numSrcRegOperands();
}

int
numDstRegOperands() override
{
    return 1;
}

int
numSrcRegOperands() override
{
    return 3;
}

int
getOperandSize(int opIdx) override
{
    switch (opIdx) {
      case 0: //src_0
        return 4;
      case 1: //src_1
        return 4;
      case 2: //src_2
        return 4;
      case 3: //vdst
        return 4;
      default:
        fatal("op idx %i out of bounds\n", opIdx);
        return -1;
    }
}

};

static const char* MNEM__V_CVT_SCALEF32_SR_BF8_F16 =
    "v_cvt_scale_sr_bf8_f16";
using Inst_VOP3__V_CVT_SCALEF32_SR_BF8_F16 =
    Inst_VOP3__V_CVT_SCALEF32_SR<AMDGPU::mxbfloat8,
                                 AMDGPU::mxfloat16,
                                 &MNEM__V_CVT_SCALEF32_SR_BF8_F16>;

static const char* MNEM__V_CVT_SCALEF32_SR_BF8_F32 =
    "v_cvt_scale_sr_bf8_f32";
using Inst_VOP3__V_CVT_SCALEF32_SR_BF8_F32 =
    Inst_VOP3__V_CVT_SCALEF32_SR<AMDGPU::mxbfloat8,
                                 AMDGPU::mxfloat32,
                                 &MNEM__V_CVT_SCALEF32_SR_BF8_F32>;

static const char* MNEM__V_CVT_SCALEF32_SR_FP8_BF16 =
    "v_cvt_scale_sr_fp8_bf16";
using Inst_VOP3__V_CVT_SCALEF32_SR_FP8_BF16 =
    Inst_VOP3__V_CVT_SCALEF32_SR<AMDGPU::mxfloat8,
                                 AMDGPU::mxbfloat16,
                                 &MNEM__V_CVT_SCALEF32_SR_FP8_BF16>;

static const char* MNEM__V_CVT_SCALEF32_SR_BF8_BF16 =
    "v_cvt_scale_sr_bf8_bf16";
using Inst_VOP3__V_CVT_SCALEF32_SR_BF8_BF16 =
    Inst_VOP3__V_CVT_SCALEF32_SR<AMDGPU::mxbfloat8,
                                 AMDGPU::mxbfloat16,
                                 &MNEM__V_CVT_SCALEF32_SR_BF8_BF16>;

static const char* MNEM__V_CVT_SCALEF32_SR_FP8_F16 =
    "v_cvt_scale_sr_fp8_f16";
using Inst_VOP3__V_CVT_SCALEF32_SR_FP8_F16 =
    Inst_VOP3__V_CVT_SCALEF32_SR<AMDGPU::mxfloat8,
                                 AMDGPU::mxfloat16,
                                 &MNEM__V_CVT_SCALEF32_SR_FP8_F16>;

static const char* MNEM__V_CVT_SCALEF32_SR_FP8_F32 =
    "v_cvt_scale_sr_fp8_f32";
using Inst_VOP3__V_CVT_SCALEF32_SR_FP8_F32 =
    Inst_VOP3__V_CVT_SCALEF32_SR<AMDGPU::mxfloat8,
                                 AMDGPU::mxfloat32,
                                 &MNEM__V_CVT_SCALEF32_SR_FP8_F32>;


/**
 * Base class for all V_CVT_SCALEF32_SR_PK32* MI355X instructions. Only
 * requires destination and source format plus the mnemonic name for the
 * instruction.
 */
template<typename dFMT, typename sFMT, const char **MNEM>
class Inst_VOP3__V_CVT_SCALE_SR_PK32 : public Inst_VOP3A
{
public:
Inst_VOP3__V_CVT_SCALE_SR_PK32(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, *MNEM, false)
{
    setFlag(ALU);
}

~Inst_VOP3__V_CVT_SCALE_SR_PK32() { }

void
execute(GPUDynInstPtr gpuDynInst) override
{
    static_assert(dFMT::size() == 6);
    static_assert(sFMT::size() == 32 || sFMT::size() == 16);

    // There are 32 values in all cases.
    constexpr const int components = 32;
    size_t input_regs = getOperandSize(0) / 4;
    size_t output_regs = getOperandSize(3) / 4;

    Wavefront *wf = gpuDynInst->wavefront();

    // The gem5 operand types are really only handy up to 64 bits. For BF
    // operand sizes such as in these instructions, just create an array of
    // 32-bit registers to use.
    std::vector<typename std::aligned_storage<
        sizeof(ConstVecOperandU32), alignof(ConstVecOperandU32)>::type>
        _src0(input_regs);
    ConstVecOperandU32* src0 =
        std::launder(reinterpret_cast<ConstVecOperandU32*>(_src0.data()));
    for (int reg = 0; reg < input_regs; ++reg) {
        new (&src0[reg]) ConstVecOperandU32(gpuDynInst, extData.SRC0 + reg);
        src0[reg].readSrc();
    }

    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1); // seed
    ConstVecOperandF32 src2(gpuDynInst, extData.SRC2); // scale
    src1.readSrc();
    src2.readSrc();

    std::vector<typename std::aligned_storage<sizeof(VecOperandU32),
                                              alignof(VecOperandU32)>::type>
        _vdst(output_regs);
    VecOperandU32* vdst =
        std::launder(reinterpret_cast<VecOperandU32*>(_vdst.data()));
    for (int reg = 0; reg < output_regs; ++reg) {
        new (&vdst[reg]) VecOperandU32(gpuDynInst, instData.VDST + reg);
    }

    panic_if(isSDWAInst(), "SDWA not supported for %s", _opcode);
    panic_if(isDPPInst(), "DPP not supported for %s", _opcode);
    panic_if(instData.CLAMP, "CLAMP not supported for %s", _opcode);
    panic_if(extData.OMOD, "OMOD not supported for %s", _opcode);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            PackedReg<sFMT::size() * components, sFMT::size()> in_reg;
            PackedReg<dFMT::size() * components, dFMT::size()> out_reg;

            for (int reg = 0; reg < input_regs; ++reg) {
                in_reg.setDword(reg, src0[reg][lane]);
            }

            for (int pass = 0; pass < components; ++pass) {
                sFMT in;
                dFMT out;

                // Note: Due to the union of a signed int and bitfield struct,
                // the data is [31:(32 - sFMT::size())], so we must align this
                // otherwise the conversions will result in a zero value.
                in.data = in_reg.getElem(pass) << (32 - sFMT::size());

                // Apply ABS, NEG, and scale
                if (instData.ABS & 1 && float(in) < 0.0f) {
                    in = -in;
                }
                if (extData.NEG & 1) {
                    in = -in;
                }

                VecElemI32 seed_val = src1[lane];

                if (instData.ABS & 2) {
                    seed_val = std::fabs(seed_val);
                }
                if (extData.NEG & 2) {
                    seed_val = -seed_val;
                }

                float scale_val = src2[lane];
                if (instData.ABS & 4) {
                    scale_val = std::fabs(scale_val);
                }
                if (extData.NEG & 4) {
                    scale_val = -scale_val;
                }

                // Only downcasts in this template. Scale before converting.
                in.scaleDiv(scale_val);

                using sInfo = decltype(in.getFmt());
                using dInfo = decltype(out.getFmt());
                dInfo cvt_info = AMDGPU::convertMXFP<dInfo, sInfo>(
                    in.getFmt(), AMDGPU::roundStochastic, seed_val
                );
                out.setFmt(cvt_info);

                if (instData.ABS & 8 && float(out) < 0.0f) {
                    out = -out;
                }
                if (extData.NEG & 8) {
                    out = -out;
                }

                out_reg.setElem(pass, out.data >> (32 - dFMT::size()));
            }

            for (int reg = 0; reg < output_regs; ++reg) {
                vdst[reg][lane] = out_reg.getDword(reg);
            }
        }
    }

    for (int reg = 0; reg < output_regs; ++reg) {
        vdst[reg].write();
    }
}

int
getNumOperands() override
{
    return numDstRegOperands() + numSrcRegOperands();
}

int
numDstRegOperands() override
{
    return 1;
}

int
numSrcRegOperands() override
{
    return 3;
}

int
getOperandSize(int opIdx) override
{
    switch (opIdx) {
      case 0: //src_0
        return sFMT::size() * 4;
      case 1: //src_1
        return 4;
      case 2: //src_2
        return 4;
      case 3: //vdst
        // Always 6 dwords
        return 6 * 4;
      default:
        fatal("op idx %i out of bounds\n", opIdx);
        return -1;
    }
}

};

static const char* MNEM__V_CVT_SCALE_SR_PK_BF6_BF16 =
    "v_cvt_scale_sr_pk_bf6_bf16";
using Inst_VOP3__V_CVT_SCALE_SR_PK_BF6_BF16 =
    Inst_VOP3__V_CVT_SCALE_SR_PK32<AMDGPU::mxbf6,
                                   AMDGPU::mxbfloat16,
                                   &MNEM__V_CVT_SCALE_SR_PK_BF6_BF16>;

static const char* MNEM__V_CVT_SCALE_SR_PK_BF6_F16 =
    "v_cvt_scale_sr_pk_bf6_f16";
using Inst_VOP3__V_CVT_SCALE_SR_PK_BF6_F16 =
    Inst_VOP3__V_CVT_SCALE_SR_PK32<AMDGPU::mxbf6,
                                   AMDGPU::mxfloat16,
                                   &MNEM__V_CVT_SCALE_SR_PK_BF6_F16>;

static const char* MNEM__V_CVT_SCALE_SR_PK_BF6_F32 =
    "v_cvt_scale_sr_pk_bf6_f32";
using Inst_VOP3__V_CVT_SCALE_SR_PK_BF6_F32 =
    Inst_VOP3__V_CVT_SCALE_SR_PK32<AMDGPU::mxbf6,
                                   AMDGPU::mxfloat32,
                                   &MNEM__V_CVT_SCALE_SR_PK_BF6_F32>;

static const char* MNEM__V_CVT_SCALE_SR_PK_FP6_BF16 =
    "v_cvt_scale_sr_pk_fp6_bf16";
using Inst_VOP3__V_CVT_SCALE_SR_PK_FP6_BF16 =
    Inst_VOP3__V_CVT_SCALE_SR_PK32<AMDGPU::mxfp6,
                                   AMDGPU::mxbfloat16,
                                   &MNEM__V_CVT_SCALE_SR_PK_FP6_BF16>;

static const char* MNEM__V_CVT_SCALE_SR_PK_FP6_F16 =
    "v_cvt_scale_sr_pk_fp6_f16";
using Inst_VOP3__V_CVT_SCALE_SR_PK_FP6_F16 =
    Inst_VOP3__V_CVT_SCALE_SR_PK32<AMDGPU::mxfp6,
                                   AMDGPU::mxfloat16,
                                   &MNEM__V_CVT_SCALE_SR_PK_FP6_F16>;

static const char* MNEM__V_CVT_SCALE_SR_PK_FP6_F32 =
    "v_cvt_scale_sr_pk_fp6_f32";
using Inst_VOP3__V_CVT_SCALE_SR_PK_FP6_F32 =
    Inst_VOP3__V_CVT_SCALE_SR_PK32<AMDGPU::mxfp6,
                                   AMDGPU::mxfloat32,
                                   &MNEM__V_CVT_SCALE_SR_PK_FP6_F32>;


/**
 * Base class for all V_CVT_SCALEF32_SR_PK_FP4* MI355X instructions. Only
 * requires destination and source format plus the mnemonic name for the
 * instruction.
 */
template<typename dFMT, typename sFMT, const char **MNEM>
class Inst_VOP3__V_CVT_SCALE_SR_PK_FP4 : public Inst_VOP3A
{
public:
Inst_VOP3__V_CVT_SCALE_SR_PK_FP4(InFmt_VOP3A *iFmt)
    : Inst_VOP3A(iFmt, *MNEM, false)
{
    setFlag(ALU);
}

~Inst_VOP3__V_CVT_SCALE_SR_PK_FP4() { }

void
execute(GPUDynInstPtr gpuDynInst) override
{
    static_assert(dFMT::size() == 4);
    static_assert(sFMT::size() == 32 || sFMT::size() == 16);

    Wavefront *wf = gpuDynInst->wavefront();

    // There are either one or two dwords read depending on input type. To
    // simplify things, just declare two here and don't read the second
    // dword if it is not used.
    ConstVecOperandU32 src0[2] = { // input
        ConstVecOperandU32(gpuDynInst, extData.SRC0 + 0),
        ConstVecOperandU32(gpuDynInst, extData.SRC0 + 1)
    };
    ConstVecOperandI32 src1(gpuDynInst, extData.SRC1); // seed
    ConstVecOperandF32 src2(gpuDynInst, extData.SRC2); // scale
    VecOperandU32 vdst(gpuDynInst, instData.VDST); // output

    src0[0].readSrc();
    if constexpr (sFMT::size() == 32) {
        src0[1].readSrc();
    }
    src1.readSrc();
    src2.readSrc();

    // We want to replace the bits at the OPSEL location and not clobber
    // the rest of the register, therefore need to read modify and write.
    vdst.read();

    panic_if(isSDWAInst(), "SDWA not supported for %s", _opcode);
    panic_if(isDPPInst(), "DPP not supported for %s", _opcode);
    panic_if(instData.CLAMP, "CLAMP not supported for %s", _opcode);
    panic_if(extData.OMOD, "OMOD not supported for %s", _opcode);

    // Output byte. Input is always either 2x 16-bit values or 2x 32-bit
    // values. Therefore there is no input opsel.
    int out_opsel = bits(instData.OPSEL, 3, 2);

    for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
        if (wf->execMask(lane)) {
            sFMT in[2];
            dFMT out[2]; // Always FP4 but may as well keep it templated.

            if constexpr (sFMT::size() == 32) {
                in[0].data = src0[0][lane];
                in[1].data = src0[1][lane];
            } else {
                assert(sFMT::size() == 16);
                in[0].data = bits(src0[0][lane], 15, 0)  << 16;
                in[1].data = bits(src0[0][lane], 31, 15) << 16;
            }

            // Apply ABS, NEG, and scale - Assume these apply to both packed
            // values.
            if (instData.ABS & 1) {
                if (float(in[0]) < 0.0f) {
                    in[0] = -in[0];
                }
                if (float(in[1]) < 0.0f) {
                    in[1] = -in[1];
                }
            }
            if (extData.NEG & 1) {
                in[0] = -in[0];
                in[1] = -in[1];
            }

            VecElemI32 seed_val = src1[lane];

            if (instData.ABS & 2) {
                seed_val = std::fabs(seed_val);
            }
            if (extData.NEG & 2) {
                seed_val = -seed_val;
            }

            // Only downcasts in this template. Apply scale before converting.
            float scale_val = src2[lane];
            if (instData.ABS & 4) {
                scale_val = std::fabs(scale_val);
            }
            if (extData.NEG & 4) {
                scale_val = -scale_val;
            }

            in[0].scaleDiv(scale_val);
            in[1].scaleDiv(scale_val);

            using sInfo = decltype(in[0].getFmt());
            using dInfo = decltype(out[0].getFmt());
            dInfo cvt_info = AMDGPU::convertMXFP<dInfo, sInfo>(
                in[0].getFmt(), AMDGPU::roundStochastic, seed_val
            );
            out[0].setFmt(cvt_info);
            cvt_info = AMDGPU::convertMXFP<dInfo, sInfo>(
                in[1].getFmt(), AMDGPU::roundStochastic, seed_val
            );
            out[1].setFmt(cvt_info);

            if (instData.ABS & 8) {
                if (float(out[0]) < 0.0f) {
                    out[0] = -out[0];
                }
                if (float(out[1]) < 0.0f) {
                    out[1] = -out[1];
                }
            }

            // The bits of the mxfp type are aligned to the left of the dword,
            // so bits [31:28] are the relevant bits.
            uint8_t packed_output = (bits(out[1].data, 31, 28) << 4)
                                  | bits(out[0].data, 31, 28);
            vdst[lane] = insertBits(vdst[lane], 8 * out_opsel + 7,
                                    8 * out_opsel, packed_output);
        }
    }

    vdst.write();
}

int
getNumOperands() override
{
    return numDstRegOperands() + numSrcRegOperands();
}

int
numDstRegOperands() override
{
    return 1;
}

int
numSrcRegOperands() override
{
    return 3;
}

int
getOperandSize(int opIdx) override
{
    switch (opIdx) {
      case 0: //src_0
        return sFMT::size() / 4;
      case 1: //src_1
        return 4;
      case 2: //src_2
        return 4;
      case 3: //vdst
        return 4;
      default:
        fatal("op idx %i out of bounds\n", opIdx);
        return -1;
    }
}

};

static const char* MNEM__V_CVT_SCALE_SR_PK_FP4_BF16 =
    "v_cvt_scale_sr_pk_fp4_bf16";
using Inst_VOP3__V_CVT_SCALE_SR_PK_FP4_BF16 =
    Inst_VOP3__V_CVT_SCALE_SR_PK_FP4<AMDGPU::mxfp4,
                                     AMDGPU::mxbfloat16,
                                     &MNEM__V_CVT_SCALE_SR_PK_FP4_BF16>;

static const char* MNEM__V_CVT_SCALE_SR_PK_FP4_F16 =
    "v_cvt_scale_sr_pk_fp4_f16";
using Inst_VOP3__V_CVT_SCALE_SR_PK_FP4_F16 =
    Inst_VOP3__V_CVT_SCALE_SR_PK_FP4<AMDGPU::mxfp4,
                                     AMDGPU::mxfloat16,
                                     &MNEM__V_CVT_SCALE_SR_PK_FP4_F16>;

static const char* MNEM__V_CVT_SCALE_SR_PK_FP4_F32 =
    "v_cvt_scale_sr_pk_fp4_f32";
using Inst_VOP3__V_CVT_SCALE_SR_PK_FP4_F32 =
    Inst_VOP3__V_CVT_SCALE_SR_PK_FP4<AMDGPU::mxfp4,
                                     AMDGPU::mxfloat32,
                                     &MNEM__V_CVT_SCALE_SR_PK_FP4_F32>;


}
}

#endif // __ARCH_AMDGPU_VEGA_INSTS_VOP3_CVT_HH__
