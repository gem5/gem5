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
#include "arch/amdgpu/vega/insts/vop3p.hh"

namespace gem5
{

namespace VegaISA
{
    // --- Inst_VOP3P_MAI__V_MFMA_I32_16X16X16I8 class methods ---

    Inst_VOP3P_MAI__V_MFMA_I32_16X16X16I8::
        Inst_VOP3P_MAI__V_MFMA_I32_16X16X16I8(InFmt_VOP3P_MAI *iFmt)
        : Inst_VOP3P_MAI(iFmt, "v_mfma_i32_16x16x16i8")
    {
        setFlag(ALU);
    } // Inst_VOP3P_MAI__V_MFMA_I32_16X16X16I8

    Inst_VOP3P_MAI__V_MFMA_I32_16X16X16I8::
        ~Inst_VOP3P_MAI__V_MFMA_I32_16X16X16I8()
    {
    } // ~Inst_VOP3P_MAI__V_MFMA_I32_16X16X16I8

    // D(16x16I32) = A(16x16I8) x B(16x16I8) + C(16x16I32), 1 Blocks, 8
    // pass, srcA/srcB 1 archVgpr, srcC/D 4 accVGPR
    void
    Inst_VOP3P_MAI__V_MFMA_I32_16X16X16I8::execute(GPUDynInstPtr gpuDynInst)
    {
        // Accumulation register offsets for A, B, and C/D matrix.
        int a_offset = 0;
        int b_offset = 0;
        int cd_offset = 0;
        if (instData.ACC_CD) {
            cd_offset = gpuDynInst->wavefront()->accumOffset;
        }
        if (extData.ACC) {
            if (extData.ACC & 0x1) {
                a_offset = gpuDynInst->wavefront()->accumOffset;
            } else if (extData.ACC & 0x2) {
                b_offset = gpuDynInst->wavefront()->accumOffset;
            }
        }

        // int8 size allows for 4 elements per lane. At 16x16 this means 4
        // lanes per column (A matrix) / (B matrix). This whole matrix fits
        // in one VGPR. The C matrix with size int32 requires 4 VGPRs.
        // Handle the C matrix by using a delta. This is set to 1 normally to
        // move to the next VGPR (1 dword away) and 0 if the input is a scalar
        // reg (e.g., a constant).
        int delta = isVectorReg(extData.SRC2) ? 1 : 0;

        // VecOperandI8 will read 8 bits and sign extend, so used U32 to read
        // as "untyped" 32-bit values.
        ConstVecOperandU32 src0(gpuDynInst, extData.SRC0+a_offset);
        ConstVecOperandU32 src1(gpuDynInst, extData.SRC1+b_offset);
        ConstVecOperandI32 src2[4] = {
            ConstVecOperandI32(gpuDynInst, extData.SRC2+cd_offset),
            ConstVecOperandI32(gpuDynInst, extData.SRC2+cd_offset+1*delta),
            ConstVecOperandI32(gpuDynInst, extData.SRC2+cd_offset+2*delta),
            ConstVecOperandI32(gpuDynInst, extData.SRC2+cd_offset+3*delta),
        };

        VecOperandI32 vdst[4] = {
            VecOperandI32(gpuDynInst, instData.VDST+cd_offset),
            VecOperandI32(gpuDynInst, instData.VDST+cd_offset+1),
            VecOperandI32(gpuDynInst, instData.VDST+cd_offset+2),
            VecOperandI32(gpuDynInst, instData.VDST+cd_offset+3),
        };

        src0.readSrc();
        src1.readSrc();
        for (int i = 0; i < 4; ++i) {
            src2[i].readSrc();
        }

        int32_t A[16][16];
        for (int i = 0; i < 64; ++i) {
            // src0[0:15] contains columns 1 - 4 packed for rows 0 - 15,
            // src0[16:31] contains columns 5 - 8 packed for rows 0 - 15,
            // src0[32:47] contains columns 9 - 12 packed for rows 0 - 15,
            // src0[48:63] contains columns 13 - 16 packed for rows 0 - 15,
            int row = i % 16;
            int start_col = (i / 16) * 4;

            A[row][start_col+0] = sext<8>(bits(src0[i], 7, 0));
            A[row][start_col+1] = sext<8>(bits(src0[i], 15, 8));
            A[row][start_col+2] = sext<8>(bits(src0[i], 23, 16));
            A[row][start_col+3] = sext<8>(bits(src0[i], 31, 24));
        }

        int32_t B[16][16];
        for (int i = 0; i < 64; ++i) {
            // src1[0:15] contains rows 1 - 4 packed for columns 0 - 15
            // src1[16:31] contains rows 5 - 8 packed for columns 0 - 15
            // src1[32:47] contains rows 9 - 12 packed for columns 0 - 15
            // src1[48:63] contains rows 13 - 16 packed for columns 0 - 15
            int start_row = (i / 16) * 4;
            int col = i % 16;

            B[start_row+0][col] = sext<8>(bits(src1[i], 7, 0));
            B[start_row+1][col] = sext<8>(bits(src1[i], 15, 8));
            B[start_row+2][col] = sext<8>(bits(src1[i], 23, 16));
            B[start_row+3][col] = sext<8>(bits(src1[i], 31, 24));
        }

        int32_t result[16][16];

        // Load accumulation matrix C into result
        for (int i = 0; i < 64; ++i) {
            // src2[0] contains rows 0, 4, 8, 12
            result[(i/16)*4][(i%16)] = src2[0][i];
            // src2[1] contains rows 1, 5, 9, 13
            result[(i/16)*4+1][(i%16)] = src2[1][i];
            // src2[2] contains rows 2, 6, 10, 14
            result[(i/16)*4+2][(i%16)] = src2[2][i];
            // src2[3] contains rows 3, 7, 11, 15
            result[(i/16)*4+3][(i%16)] = src2[3][i];
        }

        // Compute new result - This is (obviously) not optimized
        for (int i = 0; i < 16; ++i) {
            for (int j = 0; j < 16; ++j) {
                for (int k = 0; k < 16; ++k) {
                    result[i][j] += A[i][k] * B[k][j];
                }
            }
        }

        // Put result in dest VGPRs
        for (int i = 0; i < 64; ++i) {
            // vdst[0] contains rows 0, 4, 8, 12
            vdst[0][i] = result[(i/16)*4][(i%16)];
            // vdst[1] contains rows 1, 5, 9, 13
            vdst[1][i] = result[(i/16)*4+1][(i%16)];
            // vdst[2] contains rows 2, 6, 10, 14
            vdst[2][i] = result[(i/16)*4+2][(i%16)];
            // vdst[3] contains rows 3, 7, 11, 15
            vdst[3][i] = result[(i/16)*4+3][(i%16)];
        }

        for (int i = 0; i < 4; ++i) {
            vdst[i].write();
        }
    } // execute
    // --- Inst_VOP3P_MAI__V_MFMA_F64_16X16X4F64 class methods ---

    Inst_VOP3P_MAI__V_MFMA_F64_16X16X4F64::
        Inst_VOP3P_MAI__V_MFMA_F64_16X16X4F64(InFmt_VOP3P_MAI *iFmt)
        : Inst_VOP3P_MAI(iFmt, "v_mfma_f64_16x16x4f64")
    {
        setFlag(ALU);
    } // Inst_VOP3P_MAI__V_MFMA_F64_16X16X4F64

    Inst_VOP3P_MAI__V_MFMA_F64_16X16X4F64::
        ~Inst_VOP3P_MAI__V_MFMA_F64_16X16X4F64()
    {
    } // ~Inst_VOP3P_MAI__V_MFMA_F64_16X16X4F64

    // D(16x16F64) = A(16x4F64) x B(4x16F64) + C(16x16F64), 1 Blocks, 8
    // pass, srcA/srcB 2 VGPR, srcC/D 8 VGPR
    void
    Inst_VOP3P_MAI__V_MFMA_F64_16X16X4F64::execute(GPUDynInstPtr gpuDynInst)
    {
        // Accumulation register offsets for A, B, and C/D matrix.
        int a_offset = 0;
        int b_offset = 0;
        int cd_offset = 0;
        if (instData.ACC_CD) {
            cd_offset = gpuDynInst->wavefront()->accumOffset;
        }
        if (extData.ACC) {
            if (extData.ACC & 0x1) {
                a_offset = gpuDynInst->wavefront()->accumOffset;
            } else if (extData.ACC & 0x2) {
                b_offset = gpuDynInst->wavefront()->accumOffset;
            }
        }

        // Handling of src2 is a bit tricky. The operator[] overload cannot
        // be used for dword count > 2, and the dword count here is 8. Usually
        // src2 is a VGPR/AccGPR, but it might also be constant. In order to
        // use operator[] and handle constants, check for VGPR here and set
        // a delta for each of the pairs of src2 GPRs.
        int delta = isVectorReg(extData.SRC2) ? 2 : 0;

        ConstVecOperandF64 src0(gpuDynInst, extData.SRC0+a_offset);
        ConstVecOperandF64 src1(gpuDynInst, extData.SRC1+b_offset);
        ConstVecOperandF64 src2[4] = {
            ConstVecOperandF64(gpuDynInst, extData.SRC2+cd_offset),
            ConstVecOperandF64(gpuDynInst, extData.SRC2+cd_offset+1*delta),
            ConstVecOperandF64(gpuDynInst, extData.SRC2+cd_offset+2*delta),
            ConstVecOperandF64(gpuDynInst, extData.SRC2+cd_offset+3*delta),
        };

        VecOperandF64 vdst[4] = {
            VecOperandF64(gpuDynInst, instData.VDST+cd_offset),
            VecOperandF64(gpuDynInst, instData.VDST+cd_offset+2),
            VecOperandF64(gpuDynInst, instData.VDST+cd_offset+4),
            VecOperandF64(gpuDynInst, instData.VDST+cd_offset+6),
        };

        src0.readSrc();
        src1.readSrc();

        for (int i = 0; i < 4; ++i) {
            src2[i].readSrc();
        }

        double result[16][16];

        // Load src2 into result. src2 is row major
        for (int i = 0; i < 64; ++i) {
            // src2[0] contains rows 0 - 3
            result[(i/16)][(i%16)] = src2[0][i];
            // src2[1] contains rows 4 - 7
            result[(i/16)+4][(i%16)] = src2[1][i];
            // src2[2] contains rows 8 - 11
            result[(i/16)+8][(i%16)] = src2[2][i];
            // src2[3] contains rows 12 - 15
            result[(i/16)+12][(i%16)] = src2[3][i];
        }

        // Compute new result
        for (int i = 0; i < 16; ++i) {
            for (int j = 0; j < 16; ++j) {
                for (int k = 0; k < 4; ++k) {
                    // src0 is column major, src1 is row major
                    int lane_A = 16*k + i;
                    int lane_B = 16*k + j;
                    result[i][j] += src0[lane_A] * src1[lane_B];
                }
            }
        }

        // Put result in dest VGPRs
        for (int i = 0; i < 64; ++i) {
            // vdst[0] contains rows 0 - 3
            vdst[0][i] = result[(i/16)][(i%16)];
            // src2[1] contains rows 4 - 7
            vdst[1][i] = result[(i/16)+4][(i%16)];
            // src2[2] contains rows 8 - 11
            vdst[2][i] = result[(i/16)+8][(i%16)];
            // src2[3] contains rows 12 - 15
            vdst[3][i] = result[(i/16)+12][(i%16)];
        }

        for (int i = 0; i < 4; ++i) {
            vdst[i].write();
        }
    } // execute
} // namespace VegaISA
} // namespace gem5
