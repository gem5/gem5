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
    // --- Inst_VOP1__V_NOP class methods ---

    Inst_VOP1__V_NOP::Inst_VOP1__V_NOP(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_nop")
    {
        setFlag(Nop);
        setFlag(ALU);
    } // Inst_VOP1__V_NOP

    Inst_VOP1__V_NOP::~Inst_VOP1__V_NOP()
    {
    } // ~Inst_VOP1__V_NOP

    // --- description from .arch file ---
    // Do nothing.
    void
    Inst_VOP1__V_NOP::execute(GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_VOP1__V_MOV_B32 class methods ---

    Inst_VOP1__V_MOV_B32::Inst_VOP1__V_MOV_B32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_mov_b32")
    {
        setFlag(ALU);
    } // Inst_VOP1__V_MOV_B32

    Inst_VOP1__V_MOV_B32::~Inst_VOP1__V_MOV_B32()
    {
    } // ~Inst_VOP1__V_MOV_B32

    // --- description from .arch file ---
    // D.u = S0.u.
    // Input and output modifiers not supported; this is an untyped operation.
    void
    Inst_VOP1__V_MOV_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandU32 src(gpuDynInst, instData.SRC0);
        VecOperandU32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);

        if (isDPPInst()) {
            VecOperandU32 src_dpp(gpuDynInst, extData.iFmt_VOP_DPP.SRC0);
            src_dpp.read();

            DPRINTF(VEGA, "Handling V_MOV_B32 SRC DPP. SRC0: register v[%d], "
                    "DPP_CTRL: 0x%#x, SRC0_ABS: %d, SRC0_NEG: %d, "
                    "SRC1_ABS: %d, SRC1_NEG: %d, BC: %d, "
                    "BANK_MASK: %d, ROW_MASK: %d\n", extData.iFmt_VOP_DPP.SRC0,
                    extData.iFmt_VOP_DPP.DPP_CTRL,
                    extData.iFmt_VOP_DPP.SRC0_ABS,
                    extData.iFmt_VOP_DPP.SRC0_NEG,
                    extData.iFmt_VOP_DPP.SRC1_ABS,
                    extData.iFmt_VOP_DPP.SRC1_NEG,
                    extData.iFmt_VOP_DPP.BC,
                    extData.iFmt_VOP_DPP.BANK_MASK,
                    extData.iFmt_VOP_DPP.ROW_MASK);

            // NOTE: For VOP1, there is no SRC1, so make sure we're not trying
            // to negate it or take the absolute value of it
            assert(!extData.iFmt_VOP_DPP.SRC1_ABS);
            assert(!extData.iFmt_VOP_DPP.SRC1_NEG);
            processDPP(gpuDynInst, extData.iFmt_VOP_DPP, src_dpp);

            for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                if (wf->execMask(lane)) {
                    vdst[lane] = src_dpp[lane];
                }
            }
        } else {
            for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                if (wf->execMask(lane)) {
                    vdst[lane] = src[lane];
                }
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_READFIRSTLANE_B32 class methods ---

    Inst_VOP1__V_READFIRSTLANE_B32::Inst_VOP1__V_READFIRSTLANE_B32(
          InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_readfirstlane_b32")
    {
        setFlag(ALU);
    } // Inst_VOP1__V_READFIRSTLANE_B32

    Inst_VOP1__V_READFIRSTLANE_B32::~Inst_VOP1__V_READFIRSTLANE_B32()
    {
    } // ~Inst_VOP1__V_READFIRSTLANE_B32

    // --- description from .arch file ---
    // Copy one VGPR value to one SGPR. D = SGPR destination, S0 = source data
    // (VGPR# or M0 for lds direct access), Lane# = FindFirst1fromLSB(exec)
    // (Lane# = 0 if exec is zero). Ignores exec mask for the access. SQ
    // translates to V_READLANE_B32.
    // Input and output modifiers not supported; this is an untyped operation.
    void
    Inst_VOP1__V_READFIRSTLANE_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ScalarRegI32 src_lane(0);
        ScalarRegU64 exec_mask = wf->execMask().to_ullong();
        ConstVecOperandU32 src(gpuDynInst, instData.SRC0);
        ScalarOperandU32 sdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not supported for %s", _opcode);
        panic_if(isDPPInst(), "DPP not supported for %s", _opcode);

        if (exec_mask) {
            src_lane = findLsbSet(exec_mask);
        }

        sdst = src[src_lane];

        sdst.write();
    } // execute
    // --- Inst_VOP1__V_CVT_I32_F64 class methods ---

    Inst_VOP1__V_CVT_I32_F64::Inst_VOP1__V_CVT_I32_F64(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_cvt_i32_f64")
    {
        setFlag(ALU);
        setFlag(F64);
    } // Inst_VOP1__V_CVT_I32_F64

    Inst_VOP1__V_CVT_I32_F64::~Inst_VOP1__V_CVT_I32_F64()
    {
    } // ~Inst_VOP1__V_CVT_I32_F64

    // --- description from .arch file ---
    // D.i = (int)S0.d.
    // Out-of-range floating point values (including infinity) saturate. NaN is
    // ---  converted to 0.
    void
    Inst_VOP1__V_CVT_I32_F64::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF64 src(gpuDynInst, instData.SRC0);
        VecOperandI32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not supported for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                int exp;
                std::frexp(src[lane],&exp);
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
    // --- Inst_VOP1__V_CVT_F64_I32 class methods ---

    Inst_VOP1__V_CVT_F64_I32::Inst_VOP1__V_CVT_F64_I32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_cvt_f64_i32")
    {
        setFlag(ALU);
        setFlag(F64);
    } // Inst_VOP1__V_CVT_F64_I32

    Inst_VOP1__V_CVT_F64_I32::~Inst_VOP1__V_CVT_F64_I32()
    {
    } // ~Inst_VOP1__V_CVT_F64_I32

    // --- description from .arch file ---
    // D.d = (double)S0.i.
    void
    Inst_VOP1__V_CVT_F64_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandI32 src(gpuDynInst, instData.SRC0);
        VecOperandF64 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not supported for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = (VecElemF64)src[lane];
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_CVT_F32_I32 class methods ---

    Inst_VOP1__V_CVT_F32_I32::Inst_VOP1__V_CVT_F32_I32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_cvt_f32_i32")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_CVT_F32_I32

    Inst_VOP1__V_CVT_F32_I32::~Inst_VOP1__V_CVT_F32_I32()
    {
    } // ~Inst_VOP1__V_CVT_F32_I32

    // --- description from .arch file ---
    // D.f = (float)S0.i.
    void
    Inst_VOP1__V_CVT_F32_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandI32 src(gpuDynInst, instData.SRC0);
        VecOperandF32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = (VecElemF32)src[lane];
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_CVT_F32_U32 class methods ---

    Inst_VOP1__V_CVT_F32_U32::Inst_VOP1__V_CVT_F32_U32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_cvt_f32_u32")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_CVT_F32_U32

    Inst_VOP1__V_CVT_F32_U32::~Inst_VOP1__V_CVT_F32_U32()
    {
    } // ~Inst_VOP1__V_CVT_F32_U32

    // --- description from .arch file ---
    // D.f = (float)S0.u.
    void
    Inst_VOP1__V_CVT_F32_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandU32 src(gpuDynInst, instData.SRC0);
        VecOperandF32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = (VecElemF32)src[lane];
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_CVT_U32_F32 class methods ---

    Inst_VOP1__V_CVT_U32_F32::Inst_VOP1__V_CVT_U32_F32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_cvt_u32_f32")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_CVT_U32_F32

    Inst_VOP1__V_CVT_U32_F32::~Inst_VOP1__V_CVT_U32_F32()
    {
    } // ~Inst_VOP1__V_CVT_U32_F32

    // --- description from .arch file ---
    // D.u = (unsigned)S0.f.
    // Out-of-range floating point values (including infinity) saturate. NaN is
    // ---  converted to 0.
    void
    Inst_VOP1__V_CVT_U32_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF32 src(gpuDynInst, instData.SRC0);
        VecOperandU32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                int exp;
                std::frexp(src[lane],&exp);
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
    // --- Inst_VOP1__V_CVT_I32_F32 class methods ---

    Inst_VOP1__V_CVT_I32_F32::Inst_VOP1__V_CVT_I32_F32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_cvt_i32_f32")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_CVT_I32_F32

    Inst_VOP1__V_CVT_I32_F32::~Inst_VOP1__V_CVT_I32_F32()
    {
    } // ~Inst_VOP1__V_CVT_I32_F32

    // --- description from .arch file ---
    // D.i = (int)S0.f.
    // Out-of-range floating point values (including infinity) saturate. NaN is
    // ---  converted to 0.
    void
    Inst_VOP1__V_CVT_I32_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF32 src(gpuDynInst, instData.SRC0);
        VecOperandI32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                int exp;
                std::frexp(src[lane],&exp);
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
    // --- Inst_VOP1__V_MOV_FED_B32 class methods ---

    Inst_VOP1__V_MOV_FED_B32::Inst_VOP1__V_MOV_FED_B32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_mov_fed_b32")
    {
        setFlag(ALU);
    } // Inst_VOP1__V_MOV_FED_B32

    Inst_VOP1__V_MOV_FED_B32::~Inst_VOP1__V_MOV_FED_B32()
    {
    } // ~Inst_VOP1__V_MOV_FED_B32

    // --- description from .arch file ---
    // D.u = S0.u;
    // Introduce EDC double error upon write to dest vgpr without causing an
    // ---  exception.
    // Input and output modifiers not supported; this is an untyped operation.
    void
    Inst_VOP1__V_MOV_FED_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_VOP1__V_CVT_F16_F32 class methods ---

    Inst_VOP1__V_CVT_F16_F32::Inst_VOP1__V_CVT_F16_F32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_cvt_f16_f32")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_CVT_F16_F32

    Inst_VOP1__V_CVT_F16_F32::~Inst_VOP1__V_CVT_F16_F32()
    {
    } // ~Inst_VOP1__V_CVT_F16_F32

    // --- description from .arch file ---
    // D.f16 = flt32_to_flt16(S0.f).
    // Supports input modifiers and creates FP16 denormals when appropriate.
    void
    Inst_VOP1__V_CVT_F16_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF32 src(gpuDynInst, instData.SRC0);
        VecOperandU32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                float tmp = src[lane];
                AMDGPU::mxfloat16 out(tmp);

                vdst[lane] = (out.data >> 16);
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_CVT_F32_F16 class methods ---

    Inst_VOP1__V_CVT_F32_F16::Inst_VOP1__V_CVT_F32_F16(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_cvt_f32_f16")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_CVT_F32_F16

    Inst_VOP1__V_CVT_F32_F16::~Inst_VOP1__V_CVT_F32_F16()
    {
    } // ~Inst_VOP1__V_CVT_F32_F16

    // --- description from .arch file ---
    // D.f = flt16_to_flt32(S0.f16).
    // FP16 denormal inputs are always accepted.
    void
    Inst_VOP1__V_CVT_F32_F16::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandU32 src(gpuDynInst, instData.SRC0);
        VecOperandF32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                AMDGPU::mxfloat16 tmp(src[lane]);
                vdst[lane] = float(tmp);
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_CVT_RPI_I32_F32 class methods ---

    Inst_VOP1__V_CVT_RPI_I32_F32::Inst_VOP1__V_CVT_RPI_I32_F32(
          InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_cvt_rpi_i32_f32")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_CVT_RPI_I32_F32

    Inst_VOP1__V_CVT_RPI_I32_F32::~Inst_VOP1__V_CVT_RPI_I32_F32()
    {
    } // ~Inst_VOP1__V_CVT_RPI_I32_F32

    // --- description from .arch file ---
    // D.i = (int)floor(S0.f + 0.5).
    void
    Inst_VOP1__V_CVT_RPI_I32_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF32 src(gpuDynInst, instData.SRC0);
        VecOperandI32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = (VecElemI32)std::floor(src[lane] + 0.5);
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_CVT_FLR_I32_F32 class methods ---

    Inst_VOP1__V_CVT_FLR_I32_F32::Inst_VOP1__V_CVT_FLR_I32_F32(
          InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_cvt_flr_i32_f32")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_CVT_FLR_I32_F32

    Inst_VOP1__V_CVT_FLR_I32_F32::~Inst_VOP1__V_CVT_FLR_I32_F32()
    {
    } // ~Inst_VOP1__V_CVT_FLR_I32_F32

    // --- description from .arch file ---
    // D.i = (int)floor(S0.f).
    void
    Inst_VOP1__V_CVT_FLR_I32_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF32 src(gpuDynInst, instData.SRC0);
        VecOperandI32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = (VecElemI32)std::floor(src[lane]);
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_CVT_OFF_F32_I4 class methods ---

    Inst_VOP1__V_CVT_OFF_F32_I4::Inst_VOP1__V_CVT_OFF_F32_I4(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_cvt_off_f32_i4")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_CVT_OFF_F32_I4

    Inst_VOP1__V_CVT_OFF_F32_I4::~Inst_VOP1__V_CVT_OFF_F32_I4()
    {
    } // ~Inst_VOP1__V_CVT_OFF_F32_I4

    // --- description from .arch file ---
    // 4-bit signed int to 32-bit float. Used for interpolation in shader.
    void
    Inst_VOP1__V_CVT_OFF_F32_I4::execute(GPUDynInstPtr gpuDynInst)
    {
        // Could not parse sq_uc.arch desc field
        panicUnimplemented();
    } // execute
    // --- Inst_VOP1__V_CVT_F32_F64 class methods ---

    Inst_VOP1__V_CVT_F32_F64::Inst_VOP1__V_CVT_F32_F64(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_cvt_f32_f64")
    {
        setFlag(ALU);
        setFlag(F64);
    } // Inst_VOP1__V_CVT_F32_F64

    Inst_VOP1__V_CVT_F32_F64::~Inst_VOP1__V_CVT_F32_F64()
    {
    } // ~Inst_VOP1__V_CVT_F32_F64

    // --- description from .arch file ---
    // D.f = (float)S0.d.
    void
    Inst_VOP1__V_CVT_F32_F64::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF64 src(gpuDynInst, instData.SRC0);
        VecOperandF32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not supported for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = (VecElemF32)src[lane];
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_CVT_F64_F32 class methods ---

    Inst_VOP1__V_CVT_F64_F32::Inst_VOP1__V_CVT_F64_F32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_cvt_f64_f32")
    {
        setFlag(ALU);
        setFlag(F64);
    } // Inst_VOP1__V_CVT_F64_F32

    Inst_VOP1__V_CVT_F64_F32::~Inst_VOP1__V_CVT_F64_F32()
    {
    } // ~Inst_VOP1__V_CVT_F64_F32

    // --- description from .arch file ---
    // D.d = (double)S0.f.
    void
    Inst_VOP1__V_CVT_F64_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF32 src(gpuDynInst, instData.SRC0);
        VecOperandF64 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not supported for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = (VecElemF64)src[lane];
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_CVT_F32_UBYTE0 class methods ---

    Inst_VOP1__V_CVT_F32_UBYTE0::Inst_VOP1__V_CVT_F32_UBYTE0(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_cvt_f32_ubyte0")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_CVT_F32_UBYTE0

    Inst_VOP1__V_CVT_F32_UBYTE0::~Inst_VOP1__V_CVT_F32_UBYTE0()
    {
    } // ~Inst_VOP1__V_CVT_F32_UBYTE0

    // --- description from .arch file ---
    // D.f = (float)(S0.u[7:0]).
    void
    Inst_VOP1__V_CVT_F32_UBYTE0::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandU32 src(gpuDynInst, instData.SRC0);
        VecOperandF32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = (VecElemF32)(bits(src[lane], 7, 0));
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_CVT_F32_UBYTE1 class methods ---

    Inst_VOP1__V_CVT_F32_UBYTE1::Inst_VOP1__V_CVT_F32_UBYTE1(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_cvt_f32_ubyte1")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_CVT_F32_UBYTE1

    Inst_VOP1__V_CVT_F32_UBYTE1::~Inst_VOP1__V_CVT_F32_UBYTE1()
    {
    } // ~Inst_VOP1__V_CVT_F32_UBYTE1

    // --- description from .arch file ---
    // D.f = (float)(S0.u[15:8]).
    void
    Inst_VOP1__V_CVT_F32_UBYTE1::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandU32 src(gpuDynInst, instData.SRC0);
        VecOperandF32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = (VecElemF32)(bits(src[lane], 15, 8));
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_CVT_F32_UBYTE2 class methods ---

    Inst_VOP1__V_CVT_F32_UBYTE2::Inst_VOP1__V_CVT_F32_UBYTE2(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_cvt_f32_ubyte2")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_CVT_F32_UBYTE2

    Inst_VOP1__V_CVT_F32_UBYTE2::~Inst_VOP1__V_CVT_F32_UBYTE2()
    {
    } // ~Inst_VOP1__V_CVT_F32_UBYTE2

    // --- description from .arch file ---
    // D.f = (float)(S0.u[23:16]).
    void
    Inst_VOP1__V_CVT_F32_UBYTE2::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandU32 src(gpuDynInst, instData.SRC0);
        VecOperandF32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = (VecElemF32)(bits(src[lane], 23, 16));
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_CVT_F32_UBYTE3 class methods ---

    Inst_VOP1__V_CVT_F32_UBYTE3::Inst_VOP1__V_CVT_F32_UBYTE3(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_cvt_f32_ubyte3")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_CVT_F32_UBYTE3

    Inst_VOP1__V_CVT_F32_UBYTE3::~Inst_VOP1__V_CVT_F32_UBYTE3()
    {
    } // ~Inst_VOP1__V_CVT_F32_UBYTE3

    // --- description from .arch file ---
    // D.f = (float)(S0.u[31:24]).
    void
    Inst_VOP1__V_CVT_F32_UBYTE3::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandU32 src(gpuDynInst, instData.SRC0);
        VecOperandF32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = (VecElemF32)(bits(src[lane], 31, 24));
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_CVT_U32_F64 class methods ---

    Inst_VOP1__V_CVT_U32_F64::Inst_VOP1__V_CVT_U32_F64(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_cvt_u32_f64")
    {
        setFlag(ALU);
        setFlag(F64);
    } // Inst_VOP1__V_CVT_U32_F64

    Inst_VOP1__V_CVT_U32_F64::~Inst_VOP1__V_CVT_U32_F64()
    {
    } // ~Inst_VOP1__V_CVT_U32_F64

    // --- description from .arch file ---
    // D.u = (unsigned)S0.d.
    // Out-of-range floating point values (including infinity) saturate. NaN is
    // ---  converted to 0.
    void
    Inst_VOP1__V_CVT_U32_F64::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF64 src(gpuDynInst, instData.SRC0);
        VecOperandU32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not supported for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                int exp;
                std::frexp(src[lane],&exp);
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
    // --- Inst_VOP1__V_CVT_F64_U32 class methods ---

    Inst_VOP1__V_CVT_F64_U32::Inst_VOP1__V_CVT_F64_U32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_cvt_f64_u32")
    {
        setFlag(ALU);
        setFlag(F64);
    } // Inst_VOP1__V_CVT_F64_U32

    Inst_VOP1__V_CVT_F64_U32::~Inst_VOP1__V_CVT_F64_U32()
    {
    } // ~Inst_VOP1__V_CVT_F64_U32

    // --- description from .arch file ---
    // D.d = (double)S0.u.
    void
    Inst_VOP1__V_CVT_F64_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandU32 src(gpuDynInst, instData.SRC0);
        VecOperandF64 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not supported for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = (VecElemF64)src[lane];
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_TRUNC_F64 class methods ---

    Inst_VOP1__V_TRUNC_F64::Inst_VOP1__V_TRUNC_F64(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_trunc_f64")
    {
        setFlag(ALU);
        setFlag(F64);
    } // Inst_VOP1__V_TRUNC_F64

    Inst_VOP1__V_TRUNC_F64::~Inst_VOP1__V_TRUNC_F64()
    {
    } // ~Inst_VOP1__V_TRUNC_F64

    // --- description from .arch file ---
    // D.d = trunc(S0.d), return integer part of S0.d.
    void
    Inst_VOP1__V_TRUNC_F64::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF64 src(gpuDynInst, instData.SRC0);
        VecOperandF64 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not supported for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = std::trunc(src[lane]);
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_CEIL_F64 class methods ---

    Inst_VOP1__V_CEIL_F64::Inst_VOP1__V_CEIL_F64(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_ceil_f64")
    {
        setFlag(ALU);
        setFlag(F64);
    } // Inst_VOP1__V_CEIL_F64

    Inst_VOP1__V_CEIL_F64::~Inst_VOP1__V_CEIL_F64()
    {
    } // ~Inst_VOP1__V_CEIL_F64

    // --- description from .arch file ---
    // D.d = trunc(S0.d);
    // if (S0.d > 0.0 && S0.d != D.d) then D.d += 1.0.
    void
    Inst_VOP1__V_CEIL_F64::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF64 src(gpuDynInst, instData.SRC0);
        VecOperandF64 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not supported for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = std::ceil(src[lane]);
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_RNDNE_F64 class methods ---

    Inst_VOP1__V_RNDNE_F64::Inst_VOP1__V_RNDNE_F64(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_rndne_f64")
    {
        setFlag(ALU);
        setFlag(F64);
    } // Inst_VOP1__V_RNDNE_F64

    Inst_VOP1__V_RNDNE_F64::~Inst_VOP1__V_RNDNE_F64()
    {
    } // ~Inst_VOP1__V_RNDNE_F64

    // --- description from .arch file ---
    // D.d = round_nearest_even(S0.d).
    void
    Inst_VOP1__V_RNDNE_F64::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF64 src(gpuDynInst, instData.SRC0);
        VecOperandF64 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not supported for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = roundNearestEven(src[lane]);
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_FLOOR_F64 class methods ---

    Inst_VOP1__V_FLOOR_F64::Inst_VOP1__V_FLOOR_F64(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_floor_f64")
    {
        setFlag(ALU);
        setFlag(F64);
    } // Inst_VOP1__V_FLOOR_F64

    Inst_VOP1__V_FLOOR_F64::~Inst_VOP1__V_FLOOR_F64()
    {
    } // ~Inst_VOP1__V_FLOOR_F64

    // --- description from .arch file ---
    // D.d = trunc(S0.d);
    // if (S0.d < 0.0 && S0.d != D.d) then D.d += -1.0.
    void
    Inst_VOP1__V_FLOOR_F64::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF64 src(gpuDynInst, instData.SRC0);
        VecOperandF64 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not supported for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = std::floor(src[lane]);
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_FRACT_F32 class methods ---

    Inst_VOP1__V_FRACT_F32::Inst_VOP1__V_FRACT_F32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_fract_f32")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_FRACT_F32

    Inst_VOP1__V_FRACT_F32::~Inst_VOP1__V_FRACT_F32()
    {
    } // ~Inst_VOP1__V_FRACT_F32

    // --- description from .arch file ---
    // D.f = S0.f - floor(S0.f).
    void
    Inst_VOP1__V_FRACT_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF32 src(gpuDynInst, instData.SRC0);
        VecOperandF32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                VecElemF32 int_part(0.0);
                vdst[lane] = std::modf(src[lane], &int_part);
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_TRUNC_F32 class methods ---

    Inst_VOP1__V_TRUNC_F32::Inst_VOP1__V_TRUNC_F32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_trunc_f32")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_TRUNC_F32

    Inst_VOP1__V_TRUNC_F32::~Inst_VOP1__V_TRUNC_F32()
    {
    } // ~Inst_VOP1__V_TRUNC_F32

    // --- description from .arch file ---
    // D.f = trunc(S0.f), return integer part of S0.f.
    void
    Inst_VOP1__V_TRUNC_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF32 src(gpuDynInst, instData.SRC0);
        VecOperandF32 vdst (gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = std::trunc(src[lane]);
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_CEIL_F32 class methods ---

    Inst_VOP1__V_CEIL_F32::Inst_VOP1__V_CEIL_F32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_ceil_f32")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_CEIL_F32

    Inst_VOP1__V_CEIL_F32::~Inst_VOP1__V_CEIL_F32()
    {
    } // ~Inst_VOP1__V_CEIL_F32

    // --- description from .arch file ---
    // D.f = trunc(S0.f);
    // if (S0.f > 0.0 && S0.f != D.f) then D.f += 1.0.
    void
    Inst_VOP1__V_CEIL_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF32 src(gpuDynInst, instData.SRC0);
        VecOperandF32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = std::ceil(src[lane]);
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_RNDNE_F32 class methods ---

    Inst_VOP1__V_RNDNE_F32::Inst_VOP1__V_RNDNE_F32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_rndne_f32")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_RNDNE_F32

    Inst_VOP1__V_RNDNE_F32::~Inst_VOP1__V_RNDNE_F32()
    {
    } // ~Inst_VOP1__V_RNDNE_F32

    // --- description from .arch file ---
    // D.f = round_nearest_even(S0.f).
    void
    Inst_VOP1__V_RNDNE_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF32 src(gpuDynInst, instData.SRC0);
        VecOperandF32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = roundNearestEven(src[lane]);
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_FLOOR_F32 class methods ---

    Inst_VOP1__V_FLOOR_F32::Inst_VOP1__V_FLOOR_F32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_floor_f32")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_FLOOR_F32

    Inst_VOP1__V_FLOOR_F32::~Inst_VOP1__V_FLOOR_F32()
    {
    } // ~Inst_VOP1__V_FLOOR_F32

    // --- description from .arch file ---
    // D.f = trunc(S0.f);
    // if (S0.f < 0.0 && S0.f != D.f) then D.f += -1.0.
    void
    Inst_VOP1__V_FLOOR_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF32 src(gpuDynInst, instData.SRC0);
        VecOperandF32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = std::floor(src[lane]);
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_EXP_F32 class methods ---

    Inst_VOP1__V_EXP_F32::Inst_VOP1__V_EXP_F32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_exp_f32")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_EXP_F32

    Inst_VOP1__V_EXP_F32::~Inst_VOP1__V_EXP_F32()
    {
    } // ~Inst_VOP1__V_EXP_F32

    // --- description from .arch file ---
    // D.f = pow(2.0, S0.f).
    void
    Inst_VOP1__V_EXP_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF32 src(gpuDynInst, instData.SRC0);
        VecOperandF32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = std::pow(2.0, src[lane]);
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_LOG_F32 class methods ---

    Inst_VOP1__V_LOG_F32::Inst_VOP1__V_LOG_F32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_log_f32")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_LOG_F32

    Inst_VOP1__V_LOG_F32::~Inst_VOP1__V_LOG_F32()
    {
    } // ~Inst_VOP1__V_LOG_F32

    // --- description from .arch file ---
    // D.f = log2(S0.f). Base 2 logarithm.
    void
    Inst_VOP1__V_LOG_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF32 src(gpuDynInst, instData.SRC0);
        VecOperandF32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = std::log2(src[lane]);
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_RCP_F32 class methods ---

    Inst_VOP1__V_RCP_F32::Inst_VOP1__V_RCP_F32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_rcp_f32")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_RCP_F32

    Inst_VOP1__V_RCP_F32::~Inst_VOP1__V_RCP_F32()
    {
    } // ~Inst_VOP1__V_RCP_F32

    // --- description from .arch file ---
    // D.f = 1.0 / S0.f. Reciprocal with IEEE rules and < 1ulp error.
    void
    Inst_VOP1__V_RCP_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF32 src(gpuDynInst, instData.SRC0);
        VecOperandF32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = 1.0 / src[lane];
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_RCP_IFLAG_F32 class methods ---

    Inst_VOP1__V_RCP_IFLAG_F32::Inst_VOP1__V_RCP_IFLAG_F32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_rcp_iflag_f32")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_RCP_IFLAG_F32

    Inst_VOP1__V_RCP_IFLAG_F32::~Inst_VOP1__V_RCP_IFLAG_F32()
    {
    } // ~Inst_VOP1__V_RCP_IFLAG_F32

    // --- description from .arch file ---
    // D.f = 1.0 / S0.f. Reciprocal intended for integer division, can raise
    // ---  integer DIV_BY_ZERO exception but cannot raise floating-point
    // ---  exceptions.
    void
    Inst_VOP1__V_RCP_IFLAG_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF32 src(gpuDynInst, instData.SRC0);
        VecOperandF32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = 1.0 / src[lane];
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_RSQ_F32 class methods ---

    Inst_VOP1__V_RSQ_F32::Inst_VOP1__V_RSQ_F32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_rsq_f32")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_RSQ_F32

    Inst_VOP1__V_RSQ_F32::~Inst_VOP1__V_RSQ_F32()
    {
    } // ~Inst_VOP1__V_RSQ_F32

    // --- description from .arch file ---
    // D.f = 1.0 / sqrt(S0.f). Reciprocal square root with IEEE rules.
    void
    Inst_VOP1__V_RSQ_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF32 src(gpuDynInst, instData.SRC0);
        VecOperandF32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = 1.0 / std::sqrt(src[lane]);
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_RCP_F64 class methods ---

    Inst_VOP1__V_RCP_F64::Inst_VOP1__V_RCP_F64(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_rcp_f64")
    {
        setFlag(ALU);
        setFlag(F64);
    } // Inst_VOP1__V_RCP_F64

    Inst_VOP1__V_RCP_F64::~Inst_VOP1__V_RCP_F64()
    {
    } // ~Inst_VOP1__V_RCP_F64

    // --- description from .arch file ---
    // D.d = 1.0 / S0.d.
    void
    Inst_VOP1__V_RCP_F64::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF64 src(gpuDynInst, instData.SRC0);
        VecOperandF64 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not supported for %s", _opcode);

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
    // --- Inst_VOP1__V_RSQ_F64 class methods ---

    Inst_VOP1__V_RSQ_F64::Inst_VOP1__V_RSQ_F64(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_rsq_f64")
    {
        setFlag(ALU);
        setFlag(F64);
    } // Inst_VOP1__V_RSQ_F64

    Inst_VOP1__V_RSQ_F64::~Inst_VOP1__V_RSQ_F64()
    {
    } // ~Inst_VOP1__V_RSQ_F64

    // --- description from .arch file ---
    // D.d = 1.0 / sqrt(S0.d). See V_RSQ_F32.
    void
    Inst_VOP1__V_RSQ_F64::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF64 src(gpuDynInst, instData.SRC0);
        VecOperandF64 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not supported for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                if (std::fpclassify(src[lane]) == FP_ZERO) {
                    vdst[lane] = +INFINITY;
                } else if (std::isnan(src[lane])) {
                    vdst[lane] = NAN;
                } else if (std::isinf(src[lane])
                           && !std::signbit(src[lane])) {
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
    // --- Inst_VOP1__V_SQRT_F32 class methods ---

    Inst_VOP1__V_SQRT_F32::Inst_VOP1__V_SQRT_F32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_sqrt_f32")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_SQRT_F32

    Inst_VOP1__V_SQRT_F32::~Inst_VOP1__V_SQRT_F32()
    {
    } // ~Inst_VOP1__V_SQRT_F32

    // --- description from .arch file ---
    // D.f = sqrt(S0.f).
    void
    Inst_VOP1__V_SQRT_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF32 src(gpuDynInst, instData.SRC0);
        VecOperandF32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = std::sqrt(src[lane]);
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_SQRT_F64 class methods ---

    Inst_VOP1__V_SQRT_F64::Inst_VOP1__V_SQRT_F64(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_sqrt_f64")
    {
        setFlag(ALU);
        setFlag(F64);
    } // Inst_VOP1__V_SQRT_F64

    Inst_VOP1__V_SQRT_F64::~Inst_VOP1__V_SQRT_F64()
    {
    } // ~Inst_VOP1__V_SQRT_F64

    // --- description from .arch file ---
    // D.d = sqrt(S0.d).
    void
    Inst_VOP1__V_SQRT_F64::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF64 src(gpuDynInst, instData.SRC0);
        VecOperandF64 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not supported for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = std::sqrt(src[lane]);
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_SIN_F32 class methods ---

    Inst_VOP1__V_SIN_F32::Inst_VOP1__V_SIN_F32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_sin_f32")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_SIN_F32

    Inst_VOP1__V_SIN_F32::~Inst_VOP1__V_SIN_F32()
    {
    } // ~Inst_VOP1__V_SIN_F32

    // --- description from .arch file ---
    // D.f = sin(S0.f * 2 * PI).
    // Valid range of S0.f is [-256.0, +256.0]. Out of range input results in
    // float 0.0.
    void
    Inst_VOP1__V_SIN_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF32 src(gpuDynInst, instData.SRC0);
        ConstScalarOperandF32 pi(gpuDynInst, REG_PI);
        VecOperandF32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();
        pi.read();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                if (src[lane] < -256.0 || src[lane] > 256.0) {
                    vdst[lane] = 0.0;
                } else {
                    vdst[lane] = std::sin(src[lane] * 2.0 * pi.rawData());
                }
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_COS_F32 class methods ---

    Inst_VOP1__V_COS_F32::Inst_VOP1__V_COS_F32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_cos_f32")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_COS_F32

    Inst_VOP1__V_COS_F32::~Inst_VOP1__V_COS_F32()
    {
    } // ~Inst_VOP1__V_COS_F32

    // --- description from .arch file ---
    // D.f = cos(S0.f * 2 * PI).
    // Valid range of S0.f is [-256.0, +256.0]. Out of range input results in
    // float 1.0.
    void
    Inst_VOP1__V_COS_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF32 src(gpuDynInst, instData.SRC0);
        ConstScalarOperandF32 pi(gpuDynInst, REG_PI);
        VecOperandF32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();
        pi.read();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                if (src[lane] < -256.0 || src[lane] > 256.0) {
                    vdst[lane] = 0.0;
                } else {
                    vdst[lane] = std::cos(src[lane] * 2.0 * pi.rawData());
                }
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_NOT_B32 class methods ---

    Inst_VOP1__V_NOT_B32::Inst_VOP1__V_NOT_B32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_not_b32")
    {
        setFlag(ALU);
    } // Inst_VOP1__V_NOT_B32

    Inst_VOP1__V_NOT_B32::~Inst_VOP1__V_NOT_B32()
    {
    } // ~Inst_VOP1__V_NOT_B32

    // --- description from .arch file ---
    // D.u = ~S0.u.
    // Input and output modifiers not supported.
    void
    Inst_VOP1__V_NOT_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandU32 src(gpuDynInst, instData.SRC0);
        VecOperandU32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = ~src[lane];
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_BFREV_B32 class methods ---

    Inst_VOP1__V_BFREV_B32::Inst_VOP1__V_BFREV_B32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_bfrev_b32")
    {
        setFlag(ALU);
    } // Inst_VOP1__V_BFREV_B32

    Inst_VOP1__V_BFREV_B32::~Inst_VOP1__V_BFREV_B32()
    {
    } // ~Inst_VOP1__V_BFREV_B32

    // --- description from .arch file ---
    // D.u[31:0] = S0.u[0:31], bitfield reverse.
    // Input and output modifiers not supported.
    void
    Inst_VOP1__V_BFREV_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandU32 src(gpuDynInst, instData.SRC0);
        VecOperandU32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = reverseBits(src[lane]);
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_FFBH_U32 class methods ---

    Inst_VOP1__V_FFBH_U32::Inst_VOP1__V_FFBH_U32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_ffbh_u32")
    {
        setFlag(ALU);
    } // Inst_VOP1__V_FFBH_U32

    Inst_VOP1__V_FFBH_U32::~Inst_VOP1__V_FFBH_U32()
    {
    } // ~Inst_VOP1__V_FFBH_U32

    // --- description from .arch file ---
    // D.u = position of first 1 in S0.u from MSB;
    // D.u = 0xffffffff if S0.u == 0.
    void
    Inst_VOP1__V_FFBH_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandU32 src(gpuDynInst, instData.SRC0);
        VecOperandU32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = findFirstOneMsb(src[lane]);
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_FFBL_B32 class methods ---

    Inst_VOP1__V_FFBL_B32::Inst_VOP1__V_FFBL_B32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_ffbl_b32")
    {
        setFlag(ALU);
    } // Inst_VOP1__V_FFBL_B32

    Inst_VOP1__V_FFBL_B32::~Inst_VOP1__V_FFBL_B32()
    {
    } // ~Inst_VOP1__V_FFBL_B32

    // --- description from .arch file ---
    // D.u = position of first 1 in S0.u from LSB;
    // D.u = 0xffffffff if S0.u == 0.
    void
    Inst_VOP1__V_FFBL_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandU32 src(gpuDynInst, instData.SRC0);
        VecOperandU32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = findFirstOne(src[lane]);
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_FFBH_I32 class methods ---

    Inst_VOP1__V_FFBH_I32::Inst_VOP1__V_FFBH_I32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_ffbh_i32")
    {
        setFlag(ALU);
    } // Inst_VOP1__V_FFBH_I32

    Inst_VOP1__V_FFBH_I32::~Inst_VOP1__V_FFBH_I32()
    {
    } // ~Inst_VOP1__V_FFBH_I32

    // --- description from .arch file ---
    // D.u = position of first bit different from sign bit in S0.i from MSB;
    // D.u = 0xffffffff if S0.i == 0 or S0.i == 0xffffffff.
    void
    Inst_VOP1__V_FFBH_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandI32 src(gpuDynInst, instData.SRC0);
        VecOperandU32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = firstOppositeSignBit(src[lane]);
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_FREXP_EXP_I32_F64 class methods ---

    Inst_VOP1__V_FREXP_EXP_I32_F64::Inst_VOP1__V_FREXP_EXP_I32_F64(
          InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_frexp_exp_i32_f64")
    {
        setFlag(ALU);
        setFlag(F64);
    } // Inst_VOP1__V_FREXP_EXP_I32_F64

    Inst_VOP1__V_FREXP_EXP_I32_F64::~Inst_VOP1__V_FREXP_EXP_I32_F64()
    {
    } // ~Inst_VOP1__V_FREXP_EXP_I32_F64

    // --- description from .arch file ---
    // See V_FREXP_EXP_I32_F32.
    void
    Inst_VOP1__V_FREXP_EXP_I32_F64::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF64 src(gpuDynInst, instData.SRC0);
        VecOperandI32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not supported for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                if (std::isinf(src[lane]) || std::isnan(src[lane])) {
                    vdst[lane] = 0;
                } else {
                    VecElemI32 exp = 0;
                    std::frexp(src[lane], &exp);
                    vdst[lane] = exp;
                }
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_FREXP_MANT_F64 class methods ---

    Inst_VOP1__V_FREXP_MANT_F64::Inst_VOP1__V_FREXP_MANT_F64(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_frexp_mant_f64")
    {
        setFlag(ALU);
        setFlag(F64);
    } // Inst_VOP1__V_FREXP_MANT_F64

    Inst_VOP1__V_FREXP_MANT_F64::~Inst_VOP1__V_FREXP_MANT_F64()
    {
    } // ~Inst_VOP1__V_FREXP_MANT_F64

    // --- description from .arch file ---
    // See V_FREXP_MANT_F32.
    void
    Inst_VOP1__V_FREXP_MANT_F64::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF64 src(gpuDynInst, instData.SRC0);
        VecOperandF64 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not supported for %s", _opcode);

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
    // --- Inst_VOP1__V_FRACT_F64 class methods ---

    Inst_VOP1__V_FRACT_F64::Inst_VOP1__V_FRACT_F64(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_fract_f64")
    {
        setFlag(ALU);
        setFlag(F64);
    } // Inst_VOP1__V_FRACT_F64

    Inst_VOP1__V_FRACT_F64::~Inst_VOP1__V_FRACT_F64()
    {
    } // ~Inst_VOP1__V_FRACT_F64

    // --- description from .arch file ---
    // See V_FRACT_F32.
    void
    Inst_VOP1__V_FRACT_F64::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF64 src(gpuDynInst, instData.SRC0);
        VecOperandF64 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not supported for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                VecElemF64 int_part(0.0);
                vdst[lane] = std::modf(src[lane], &int_part);
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_FREXP_EXP_I32_F32 class methods ---

    Inst_VOP1__V_FREXP_EXP_I32_F32::Inst_VOP1__V_FREXP_EXP_I32_F32(
          InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_frexp_exp_i32_f32")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_FREXP_EXP_I32_F32

    Inst_VOP1__V_FREXP_EXP_I32_F32::~Inst_VOP1__V_FREXP_EXP_I32_F32()
    {
    } // ~Inst_VOP1__V_FREXP_EXP_I32_F32

    // --- description from .arch file ---
    // if (S0.f == INF || S0.f == NAN) then D.i = 0;
    // else D.i = TwosComplement(Exponent(S0.f) - 127 + 1).
    // Returns exponent of single precision float input, such that S0.f =
    // significand * (2 ** exponent). See also FREXP_MANT_F32, which returns
    // the significand.
    void
    Inst_VOP1__V_FREXP_EXP_I32_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF32 src(gpuDynInst, instData.SRC0);
        VecOperandI32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

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
    // --- Inst_VOP1__V_FREXP_MANT_F32 class methods ---

    Inst_VOP1__V_FREXP_MANT_F32::Inst_VOP1__V_FREXP_MANT_F32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_frexp_mant_f32")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_FREXP_MANT_F32

    Inst_VOP1__V_FREXP_MANT_F32::~Inst_VOP1__V_FREXP_MANT_F32()
    {
    } // ~Inst_VOP1__V_FREXP_MANT_F32

    // --- description from .arch file ---
    // if (S0.f == INF || S0.f == NAN) then D.f = S0.f;
    // else D.f = Mantissa(S0.f).
    // Result range is in (-1.0,-0.5][0.5,1.0) in normal cases. Returns binary
    // ---  significand of single precision float input, such that S0.f =
    // ---  significand * (2 ** exponent). See also FREXP_EXP_I32_F32, which
    // ---  returns integer exponent.
    void
    Inst_VOP1__V_FREXP_MANT_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF32 src(gpuDynInst, instData.SRC0);
        VecOperandF32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

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
    // --- Inst_VOP1__V_CLREXCP class methods ---

    Inst_VOP1__V_CLREXCP::Inst_VOP1__V_CLREXCP(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_clrexcp")
    {
        setFlag(ALU);
    } // Inst_VOP1__V_CLREXCP

    Inst_VOP1__V_CLREXCP::~Inst_VOP1__V_CLREXCP()
    {
    } // ~Inst_VOP1__V_CLREXCP

    // --- description from .arch file ---
    // Clear wave's exception state in SIMD (SP).
    void
    Inst_VOP1__V_CLREXCP::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_VOP1__V_MOV_B64 class methods ---

    Inst_VOP1__V_MOV_B64::Inst_VOP1__V_MOV_B64(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_mov_b64")
    {
        setFlag(ALU);
    } // Inst_VOP1__V_MOV_B64

    Inst_VOP1__V_MOV_B64::~Inst_VOP1__V_MOV_B64()
    {
    } // ~Inst_VOP1__V_MOV_B64

    // --- description from .arch file ---
    // D.u = S0.u.
    // Input and output modifiers not supported; this is an untyped operation.
    void
    Inst_VOP1__V_MOV_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandU64 src(gpuDynInst, instData.SRC0);
        VecOperandU64 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = src[lane];
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_CVT_F16_U16 class methods ---

    Inst_VOP1__V_CVT_F16_U16::Inst_VOP1__V_CVT_F16_U16(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_cvt_f16_u16")
    {
        setFlag(ALU);
        setFlag(F16);
    } // Inst_VOP1__V_CVT_F16_U16

    Inst_VOP1__V_CVT_F16_U16::~Inst_VOP1__V_CVT_F16_U16()
    {
    } // ~Inst_VOP1__V_CVT_F16_U16

    // --- description from .arch file ---
    // D.f16 = uint16_to_flt16(S.u16).
    // Supports denormals, rounding, exception flags and saturation.
    void
    Inst_VOP1__V_CVT_F16_U16::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_VOP1__V_CVT_F16_I16 class methods ---

    Inst_VOP1__V_CVT_F16_I16::Inst_VOP1__V_CVT_F16_I16(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_cvt_f16_i16")
    {
        setFlag(ALU);
        setFlag(F16);
    } // Inst_VOP1__V_CVT_F16_I16

    Inst_VOP1__V_CVT_F16_I16::~Inst_VOP1__V_CVT_F16_I16()
    {
    } // ~Inst_VOP1__V_CVT_F16_I16

    // --- description from .arch file ---
    // D.f16 = int16_to_flt16(S.i16).
    // Supports denormals, rounding, exception flags and saturation.
    void
    Inst_VOP1__V_CVT_F16_I16::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_VOP1__V_CVT_U16_F16 class methods ---

    Inst_VOP1__V_CVT_U16_F16::Inst_VOP1__V_CVT_U16_F16(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_cvt_u16_f16")
    {
        setFlag(ALU);
        setFlag(F16);
    } // Inst_VOP1__V_CVT_U16_F16

    Inst_VOP1__V_CVT_U16_F16::~Inst_VOP1__V_CVT_U16_F16()
    {
    } // ~Inst_VOP1__V_CVT_U16_F16

    // --- description from .arch file ---
    // D.u16 = flt16_to_uint16(S.f16).
    // Supports rounding, exception flags and saturation.
    void
    Inst_VOP1__V_CVT_U16_F16::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_VOP1__V_CVT_I16_F16 class methods ---

    Inst_VOP1__V_CVT_I16_F16::Inst_VOP1__V_CVT_I16_F16(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_cvt_i16_f16")
    {
        setFlag(ALU);
        setFlag(F16);
    } // Inst_VOP1__V_CVT_I16_F16

    Inst_VOP1__V_CVT_I16_F16::~Inst_VOP1__V_CVT_I16_F16()
    {
    } // ~Inst_VOP1__V_CVT_I16_F16

    // --- description from .arch file ---
    // D.i16 = flt16_to_int16(S.f16).
    // Supports rounding, exception flags and saturation.
    void
    Inst_VOP1__V_CVT_I16_F16::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_VOP1__V_RCP_F16 class methods ---

    Inst_VOP1__V_RCP_F16::Inst_VOP1__V_RCP_F16(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_rcp_f16")
    {
        setFlag(ALU);
        setFlag(F16);
    } // Inst_VOP1__V_RCP_F16

    Inst_VOP1__V_RCP_F16::~Inst_VOP1__V_RCP_F16()
    {
    } // ~Inst_VOP1__V_RCP_F16

    // --- description from .arch file ---
    // if (S0.f16 == 1.0f)
    //     D.f16 = 1.0f;
    // else
    //     D.f16 = ApproximateRecip(S0.f16).
    void
    Inst_VOP1__V_RCP_F16::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_VOP1__V_SQRT_F16 class methods ---

    Inst_VOP1__V_SQRT_F16::Inst_VOP1__V_SQRT_F16(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_sqrt_f16")
    {
        setFlag(ALU);
        setFlag(F16);
    } // Inst_VOP1__V_SQRT_F16

    Inst_VOP1__V_SQRT_F16::~Inst_VOP1__V_SQRT_F16()
    {
    } // ~Inst_VOP1__V_SQRT_F16

    // --- description from .arch file ---
    // if (S0.f16 == 1.0f)
    //     D.f16 = 1.0f;
    // else
    //     D.f16 = ApproximateSqrt(S0.f16).
    void
    Inst_VOP1__V_SQRT_F16::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_VOP1__V_RSQ_F16 class methods ---

    Inst_VOP1__V_RSQ_F16::Inst_VOP1__V_RSQ_F16(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_rsq_f16")
    {
        setFlag(ALU);
        setFlag(F16);
    } // Inst_VOP1__V_RSQ_F16

    Inst_VOP1__V_RSQ_F16::~Inst_VOP1__V_RSQ_F16()
    {
    } // ~Inst_VOP1__V_RSQ_F16

    // --- description from .arch file ---
    // if (S0.f16 == 1.0f)
    //     D.f16 = 1.0f;
    // else
    //     D.f16 = ApproximateRecipSqrt(S0.f16).
    void
    Inst_VOP1__V_RSQ_F16::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_VOP1__V_LOG_F16 class methods ---

    Inst_VOP1__V_LOG_F16::Inst_VOP1__V_LOG_F16(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_log_f16")
    {
        setFlag(ALU);
        setFlag(F16);
    } // Inst_VOP1__V_LOG_F16

    Inst_VOP1__V_LOG_F16::~Inst_VOP1__V_LOG_F16()
    {
    } // ~Inst_VOP1__V_LOG_F16

    // --- description from .arch file ---
    // if (S0.f16 == 1.0f)
    //     D.f16 = 0.0f;
    // else
    //     D.f16 = ApproximateLog2(S0.f16).
    void
    Inst_VOP1__V_LOG_F16::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_VOP1__V_EXP_F16 class methods ---

    Inst_VOP1__V_EXP_F16::Inst_VOP1__V_EXP_F16(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_exp_f16")
    {
        setFlag(ALU);
        setFlag(F16);
    } // Inst_VOP1__V_EXP_F16

    Inst_VOP1__V_EXP_F16::~Inst_VOP1__V_EXP_F16()
    {
    } // ~Inst_VOP1__V_EXP_F16

    // --- description from .arch file ---
    // if (S0.f16 == 0.0f)
    //     D.f16 = 1.0f;
    // else
    //     D.f16 = Approximate2ToX(S0.f16).
    void
    Inst_VOP1__V_EXP_F16::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_VOP1__V_FREXP_MANT_F16 class methods ---

    Inst_VOP1__V_FREXP_MANT_F16::Inst_VOP1__V_FREXP_MANT_F16(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_frexp_mant_f16")
    {
        setFlag(ALU);
        setFlag(F16);
    } // Inst_VOP1__V_FREXP_MANT_F16

    Inst_VOP1__V_FREXP_MANT_F16::~Inst_VOP1__V_FREXP_MANT_F16()
    {
    } // ~Inst_VOP1__V_FREXP_MANT_F16

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
    Inst_VOP1__V_FREXP_MANT_F16::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_VOP1__V_FREXP_EXP_I16_F16 class methods ---

    Inst_VOP1__V_FREXP_EXP_I16_F16::Inst_VOP1__V_FREXP_EXP_I16_F16(
          InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_frexp_exp_i16_f16")
    {
        setFlag(ALU);
        setFlag(F16);
    } // Inst_VOP1__V_FREXP_EXP_I16_F16

    Inst_VOP1__V_FREXP_EXP_I16_F16::~Inst_VOP1__V_FREXP_EXP_I16_F16()
    {
    } // ~Inst_VOP1__V_FREXP_EXP_I16_F16

    // --- description from .arch file ---
    // if (S0.f16 == +-INF || S0.f16 == NAN)
    //     D.i16 = 0;
    // else
    //     D.i16 = 2s_complement(exponent(S0.f16) - 15 + 1).
    // C math library frexp function.
    // Returns exponent of half precision float input, such that the
    // original single float = significand * (2 ** exponent).
    void
    Inst_VOP1__V_FREXP_EXP_I16_F16::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_VOP1__V_FLOOR_F16 class methods ---

    Inst_VOP1__V_FLOOR_F16::Inst_VOP1__V_FLOOR_F16(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_floor_f16")
    {
        setFlag(ALU);
        setFlag(F16);
    } // Inst_VOP1__V_FLOOR_F16

    Inst_VOP1__V_FLOOR_F16::~Inst_VOP1__V_FLOOR_F16()
    {
    } // ~Inst_VOP1__V_FLOOR_F16

    // --- description from .arch file ---
    // D.f16 = trunc(S0.f16);
    // if (S0.f16 < 0.0f && S0.f16 != D.f16) then D.f16 -= 1.0f.
    void
    Inst_VOP1__V_FLOOR_F16::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_VOP1__V_CEIL_F16 class methods ---

    Inst_VOP1__V_CEIL_F16::Inst_VOP1__V_CEIL_F16(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_ceil_f16")
    {
        setFlag(ALU);
        setFlag(F16);
    } // Inst_VOP1__V_CEIL_F16

    Inst_VOP1__V_CEIL_F16::~Inst_VOP1__V_CEIL_F16()
    {
    } // ~Inst_VOP1__V_CEIL_F16

    // --- description from .arch file ---
    // D.f16 = trunc(S0.f16);
    // if (S0.f16 > 0.0f && S0.f16 != D.f16) then D.f16 += 1.0f.
    void
    Inst_VOP1__V_CEIL_F16::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_VOP1__V_TRUNC_F16 class methods ---

    Inst_VOP1__V_TRUNC_F16::Inst_VOP1__V_TRUNC_F16(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_trunc_f16")
    {
        setFlag(ALU);
        setFlag(F16);
    } // Inst_VOP1__V_TRUNC_F16

    Inst_VOP1__V_TRUNC_F16::~Inst_VOP1__V_TRUNC_F16()
    {
    } // ~Inst_VOP1__V_TRUNC_F16

    // --- description from .arch file ---
    // D.f16 = trunc(S0.f16).
    // Round-to-zero semantics.
    void
    Inst_VOP1__V_TRUNC_F16::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_VOP1__V_RNDNE_F16 class methods ---

    Inst_VOP1__V_RNDNE_F16::Inst_VOP1__V_RNDNE_F16(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_rndne_f16")
    {
        setFlag(ALU);
        setFlag(F16);
    } // Inst_VOP1__V_RNDNE_F16

    Inst_VOP1__V_RNDNE_F16::~Inst_VOP1__V_RNDNE_F16()
    {
    } // ~Inst_VOP1__V_RNDNE_F16

    // --- description from .arch file ---
    // D.f16 = FLOOR(S0.f16 + 0.5f);
    // if (floor(S0.f16) is even && fract(S0.f16) == 0.5f) then D.f16 -= 1.0f.
    // Round-to-nearest-even semantics.
    void
    Inst_VOP1__V_RNDNE_F16::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_VOP1__V_FRACT_F16 class methods ---

    Inst_VOP1__V_FRACT_F16::Inst_VOP1__V_FRACT_F16(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_fract_f16")
    {
        setFlag(ALU);
        setFlag(F16);
    } // Inst_VOP1__V_FRACT_F16

    Inst_VOP1__V_FRACT_F16::~Inst_VOP1__V_FRACT_F16()
    {
    } // ~Inst_VOP1__V_FRACT_F16

    // --- description from .arch file ---
    // D.f16 = S0.f16 + -floor(S0.f16).
    void
    Inst_VOP1__V_FRACT_F16::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_VOP1__V_SIN_F16 class methods ---

    Inst_VOP1__V_SIN_F16::Inst_VOP1__V_SIN_F16(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_sin_f16")
    {
        setFlag(ALU);
        setFlag(F16);
    } // Inst_VOP1__V_SIN_F16

    Inst_VOP1__V_SIN_F16::~Inst_VOP1__V_SIN_F16()
    {
    } // ~Inst_VOP1__V_SIN_F16

    // --- description from .arch file ---
    // D.f16 = sin(S0.f16 * 2 * PI).
    void
    Inst_VOP1__V_SIN_F16::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_VOP1__V_COS_F16 class methods ---

    Inst_VOP1__V_COS_F16::Inst_VOP1__V_COS_F16(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_cos_f16")
    {
        setFlag(ALU);
        setFlag(F16);
    } // Inst_VOP1__V_COS_F16

    Inst_VOP1__V_COS_F16::~Inst_VOP1__V_COS_F16()
    {
    } // ~Inst_VOP1__V_COS_F16

    // --- description from .arch file ---
    // D.f16 = cos(S0.f16 * 2 * PI).
    void
    Inst_VOP1__V_COS_F16::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_VOP1__V_EXP_LEGACY_F32 class methods ---

    Inst_VOP1__V_EXP_LEGACY_F32::Inst_VOP1__V_EXP_LEGACY_F32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_exp_legacy_f32")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_EXP_LEGACY_F32

    Inst_VOP1__V_EXP_LEGACY_F32::~Inst_VOP1__V_EXP_LEGACY_F32()
    {
    } // ~Inst_VOP1__V_EXP_LEGACY_F32

    // --- description from .arch file ---
    // D.f = pow(2.0, S0.f) with legacy semantics.
    void
    Inst_VOP1__V_EXP_LEGACY_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF32 src(gpuDynInst, instData.SRC0);
        VecOperandF32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = std::pow(2.0, src[lane]);
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_LOG_LEGACY_F32 class methods ---

    Inst_VOP1__V_LOG_LEGACY_F32::Inst_VOP1__V_LOG_LEGACY_F32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_log_legacy_f32")
    {
        setFlag(ALU);
        setFlag(F32);
    } // Inst_VOP1__V_LOG_LEGACY_F32

    Inst_VOP1__V_LOG_LEGACY_F32::~Inst_VOP1__V_LOG_LEGACY_F32()
    {
    } // ~Inst_VOP1__V_LOG_LEGACY_F32

    // --- description from .arch file ---
    // D.f = log2(S0.f). Base 2 logarithm with legacy semantics.
    void
    Inst_VOP1__V_LOG_LEGACY_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstVecOperandF32 src(gpuDynInst, instData.SRC0);
        VecOperandF32 vdst(gpuDynInst, instData.VDST);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = std::log2(src[lane]);
            }
        }

        vdst.write();
    } // execute
    // --- Inst_VOP1__V_ACCVGPR_MOV_B32 class methods ---

    Inst_VOP1__V_ACCVGPR_MOV_B32::
        Inst_VOP1__V_ACCVGPR_MOV_B32(InFmt_VOP1 *iFmt)
        : Inst_VOP1(iFmt, "v_accvgpr_mov_b32")
    {
        setFlag(ALU);
    } // Inst_VOP1__V_ACCVGPR_MOV_B32

    Inst_VOP1__V_ACCVGPR_MOV_B32::~Inst_VOP1__V_ACCVGPR_MOV_B32()
    {
    } // ~Inst_VOP1__V_ACCVGPR_MOV_B32

    void
    Inst_VOP1__V_ACCVGPR_MOV_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        unsigned accum_offset = wf->accumOffset;

        ConstVecOperandU32 src(gpuDynInst, instData.SRC0+accum_offset);
        VecOperandU32 vdst(gpuDynInst, instData.VDST+accum_offset);

        src.readSrc();

        panic_if(isSDWAInst(), "SDWA not implemented for %s", _opcode);
        panic_if(isDPPInst(), "DPP not implemented for %s", _opcode);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                vdst[lane] = src[lane];
            }
        }

        vdst.write();
    } // execute
} // namespace VegaISA
} // namespace gem5
