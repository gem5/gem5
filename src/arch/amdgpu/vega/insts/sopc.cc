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
    // --- Inst_SOPC__S_CMP_EQ_I32 class methods ---

    Inst_SOPC__S_CMP_EQ_I32::Inst_SOPC__S_CMP_EQ_I32(InFmt_SOPC *iFmt)
        : Inst_SOPC(iFmt, "s_cmp_eq_i32")
    {
        setFlag(ALU);
    } // Inst_SOPC__S_CMP_EQ_I32

    Inst_SOPC__S_CMP_EQ_I32::~Inst_SOPC__S_CMP_EQ_I32()
    {
    } // ~Inst_SOPC__S_CMP_EQ_I32

    // --- description from .arch file ---
    // SCC = (S0.i == S1.i).
    void
    Inst_SOPC__S_CMP_EQ_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandI32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandI32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        scc = (src0.rawData() == src1.rawData()) ? 1 : 0;

        scc.write();
    } // execute
    // --- Inst_SOPC__S_CMP_LG_I32 class methods ---

    Inst_SOPC__S_CMP_LG_I32::Inst_SOPC__S_CMP_LG_I32(InFmt_SOPC *iFmt)
        : Inst_SOPC(iFmt, "s_cmp_lg_i32")
    {
        setFlag(ALU);
    } // Inst_SOPC__S_CMP_LG_I32

    Inst_SOPC__S_CMP_LG_I32::~Inst_SOPC__S_CMP_LG_I32()
    {
    } // ~Inst_SOPC__S_CMP_LG_I32

    // --- description from .arch file ---
    // SCC = (S0.i != S1.i).
    void
    Inst_SOPC__S_CMP_LG_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandI32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandI32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        scc = (src0.rawData() != src1.rawData()) ? 1 : 0;

        scc.write();
    } // execute
    // --- Inst_SOPC__S_CMP_GT_I32 class methods ---

    Inst_SOPC__S_CMP_GT_I32::Inst_SOPC__S_CMP_GT_I32(InFmt_SOPC *iFmt)
        : Inst_SOPC(iFmt, "s_cmp_gt_i32")
    {
        setFlag(ALU);
    } // Inst_SOPC__S_CMP_GT_I32

    Inst_SOPC__S_CMP_GT_I32::~Inst_SOPC__S_CMP_GT_I32()
    {
    } // ~Inst_SOPC__S_CMP_GT_I32

    // --- description from .arch file ---
    // SCC = (S0.i > S1.i).
    void
    Inst_SOPC__S_CMP_GT_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandI32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandI32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        scc = (src0.rawData() > src1.rawData()) ? 1 : 0;

        scc.write();
    } // execute
    // --- Inst_SOPC__S_CMP_GE_I32 class methods ---

    Inst_SOPC__S_CMP_GE_I32::Inst_SOPC__S_CMP_GE_I32(InFmt_SOPC *iFmt)
        : Inst_SOPC(iFmt, "s_cmp_ge_i32")
    {
        setFlag(ALU);
    } // Inst_SOPC__S_CMP_GE_I32

    Inst_SOPC__S_CMP_GE_I32::~Inst_SOPC__S_CMP_GE_I32()
    {
    } // ~Inst_SOPC__S_CMP_GE_I32

    // --- description from .arch file ---
    // SCC = (S0.i >= S1.i).
    void
    Inst_SOPC__S_CMP_GE_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandI32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandI32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        scc = (src0.rawData() >= src1.rawData()) ? 1 : 0;

        scc.write();
    } // execute
    // --- Inst_SOPC__S_CMP_LT_I32 class methods ---

    Inst_SOPC__S_CMP_LT_I32::Inst_SOPC__S_CMP_LT_I32(InFmt_SOPC *iFmt)
        : Inst_SOPC(iFmt, "s_cmp_lt_i32")
    {
        setFlag(ALU);
    } // Inst_SOPC__S_CMP_LT_I32

    Inst_SOPC__S_CMP_LT_I32::~Inst_SOPC__S_CMP_LT_I32()
    {
    } // ~Inst_SOPC__S_CMP_LT_I32

    // --- description from .arch file ---
    // SCC = (S0.i < S1.i).
    void
    Inst_SOPC__S_CMP_LT_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandI32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandI32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        scc = (src0.rawData() < src1.rawData()) ? 1 : 0;

        scc.write();
    } // execute
    // --- Inst_SOPC__S_CMP_LE_I32 class methods ---

    Inst_SOPC__S_CMP_LE_I32::Inst_SOPC__S_CMP_LE_I32(InFmt_SOPC *iFmt)
        : Inst_SOPC(iFmt, "s_cmp_le_i32")
    {
        setFlag(ALU);
    } // Inst_SOPC__S_CMP_LE_I32

    Inst_SOPC__S_CMP_LE_I32::~Inst_SOPC__S_CMP_LE_I32()
    {
    } // ~Inst_SOPC__S_CMP_LE_I32

    // --- description from .arch file ---
    // SCC = (S0.i <= S1.i).
    void
    Inst_SOPC__S_CMP_LE_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandI32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandI32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        scc = (src0.rawData() <= src1.rawData()) ? 1 : 0;

        scc.write();
    } // execute
    // --- Inst_SOPC__S_CMP_EQ_U32 class methods ---

    Inst_SOPC__S_CMP_EQ_U32::Inst_SOPC__S_CMP_EQ_U32(InFmt_SOPC *iFmt)
        : Inst_SOPC(iFmt, "s_cmp_eq_u32")
    {
        setFlag(ALU);
    } // Inst_SOPC__S_CMP_EQ_U32

    Inst_SOPC__S_CMP_EQ_U32::~Inst_SOPC__S_CMP_EQ_U32()
    {
    } // ~Inst_SOPC__S_CMP_EQ_U32

    // --- description from .arch file ---
    // SCC = (S0.u == S1.u).
    void
    Inst_SOPC__S_CMP_EQ_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        scc = (src0.rawData() == src1.rawData()) ? 1 : 0;

        scc.write();
    } // execute
    // --- Inst_SOPC__S_CMP_LG_U32 class methods ---

    Inst_SOPC__S_CMP_LG_U32::Inst_SOPC__S_CMP_LG_U32(InFmt_SOPC *iFmt)
        : Inst_SOPC(iFmt, "s_cmp_lg_u32")
    {
        setFlag(ALU);
    } // Inst_SOPC__S_CMP_LG_U32

    Inst_SOPC__S_CMP_LG_U32::~Inst_SOPC__S_CMP_LG_U32()
    {
    } // ~Inst_SOPC__S_CMP_LG_U32

    // --- description from .arch file ---
    // SCC = (S0.u != S1.u).
    void
    Inst_SOPC__S_CMP_LG_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        scc = (src0.rawData() != src1.rawData()) ? 1 : 0;

        scc.write();
    } // execute
    // --- Inst_SOPC__S_CMP_GT_U32 class methods ---

    Inst_SOPC__S_CMP_GT_U32::Inst_SOPC__S_CMP_GT_U32(InFmt_SOPC *iFmt)
        : Inst_SOPC(iFmt, "s_cmp_gt_u32")
    {
        setFlag(ALU);
    } // Inst_SOPC__S_CMP_GT_U32

    Inst_SOPC__S_CMP_GT_U32::~Inst_SOPC__S_CMP_GT_U32()
    {
    } // ~Inst_SOPC__S_CMP_GT_U32

    // --- description from .arch file ---
    // SCC = (S0.u > S1.u).
    void
    Inst_SOPC__S_CMP_GT_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        scc = (src0.rawData() > src1.rawData()) ? 1 : 0;

        scc.write();
    } // execute
    // --- Inst_SOPC__S_CMP_GE_U32 class methods ---

    Inst_SOPC__S_CMP_GE_U32::Inst_SOPC__S_CMP_GE_U32(InFmt_SOPC *iFmt)
        : Inst_SOPC(iFmt, "s_cmp_ge_u32")
    {
        setFlag(ALU);
    } // Inst_SOPC__S_CMP_GE_U32

    Inst_SOPC__S_CMP_GE_U32::~Inst_SOPC__S_CMP_GE_U32()
    {
    } // ~Inst_SOPC__S_CMP_GE_U32

    // --- description from .arch file ---
    // SCC = (S0.u >= S1.u).
    void
    Inst_SOPC__S_CMP_GE_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        scc = (src0.rawData() >= src1.rawData()) ? 1 : 0;

        scc.write();
    } // execute
    // --- Inst_SOPC__S_CMP_LT_U32 class methods ---

    Inst_SOPC__S_CMP_LT_U32::Inst_SOPC__S_CMP_LT_U32(InFmt_SOPC *iFmt)
        : Inst_SOPC(iFmt, "s_cmp_lt_u32")
    {
        setFlag(ALU);
    } // Inst_SOPC__S_CMP_LT_U32

    Inst_SOPC__S_CMP_LT_U32::~Inst_SOPC__S_CMP_LT_U32()
    {
    } // ~Inst_SOPC__S_CMP_LT_U32

    // --- description from .arch file ---
    // SCC = (S0.u < S1.u).
    void
    Inst_SOPC__S_CMP_LT_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        scc = (src0.rawData() < src1.rawData()) ? 1 : 0;

        scc.write();
    } // execute
    // --- Inst_SOPC__S_CMP_LE_U32 class methods ---

    Inst_SOPC__S_CMP_LE_U32::Inst_SOPC__S_CMP_LE_U32(InFmt_SOPC *iFmt)
        : Inst_SOPC(iFmt, "s_cmp_le_u32")
    {
        setFlag(ALU);
    } // Inst_SOPC__S_CMP_LE_U32

    Inst_SOPC__S_CMP_LE_U32::~Inst_SOPC__S_CMP_LE_U32()
    {
    } // ~Inst_SOPC__S_CMP_LE_U32

    // --- description from .arch file ---
    // SCC = (S0.u <= S1.u).
    void
    Inst_SOPC__S_CMP_LE_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        scc = (src0.rawData() <= src1.rawData()) ? 1 : 0;

        scc.write();
    } // execute
    // --- Inst_SOPC__S_BITCMP0_B32 class methods ---

    Inst_SOPC__S_BITCMP0_B32::Inst_SOPC__S_BITCMP0_B32(InFmt_SOPC *iFmt)
        : Inst_SOPC(iFmt, "s_bitcmp0_b32")
    {
        setFlag(ALU);
    } // Inst_SOPC__S_BITCMP0_B32

    Inst_SOPC__S_BITCMP0_B32::~Inst_SOPC__S_BITCMP0_B32()
    {
    } // ~Inst_SOPC__S_BITCMP0_B32

    // --- description from .arch file ---
    // SCC = (S0.u[S1.u[4:0]] == 0).
    void
    Inst_SOPC__S_BITCMP0_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        scc = !bits(src0.rawData(), bits(src1.rawData(), 4, 0)) ? 1 : 0;

        scc.write();
    } // execute
    // --- Inst_SOPC__S_BITCMP1_B32 class methods ---

    Inst_SOPC__S_BITCMP1_B32::Inst_SOPC__S_BITCMP1_B32(InFmt_SOPC *iFmt)
        : Inst_SOPC(iFmt, "s_bitcmp1_b32")
    {
        setFlag(ALU);
    } // Inst_SOPC__S_BITCMP1_B32

    Inst_SOPC__S_BITCMP1_B32::~Inst_SOPC__S_BITCMP1_B32()
    {
    } // ~Inst_SOPC__S_BITCMP1_B32

    // --- description from .arch file ---
    // SCC = (S0.u[S1.u[4:0]] == 1).
    void
    Inst_SOPC__S_BITCMP1_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        scc = bits(src0.rawData(), bits(src1.rawData(), 4, 0)) ? 1 : 0;

        scc.write();
    } // execute
    // --- Inst_SOPC__S_BITCMP0_B64 class methods ---

    Inst_SOPC__S_BITCMP0_B64::Inst_SOPC__S_BITCMP0_B64(InFmt_SOPC *iFmt)
        : Inst_SOPC(iFmt, "s_bitcmp0_b64")
    {
        setFlag(ALU);
    } // Inst_SOPC__S_BITCMP0_B64

    Inst_SOPC__S_BITCMP0_B64::~Inst_SOPC__S_BITCMP0_B64()
    {
    } // ~Inst_SOPC__S_BITCMP0_B64

    // --- description from .arch file ---
    // SCC = (S0.u64[S1.u[5:0]] == 0).
    void
    Inst_SOPC__S_BITCMP0_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU64 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        scc = !bits(src0.rawData(), bits(src1.rawData(), 5, 0)) ? 1 : 0;

        scc.write();
    } // execute
    // --- Inst_SOPC__S_BITCMP1_B64 class methods ---

    Inst_SOPC__S_BITCMP1_B64::Inst_SOPC__S_BITCMP1_B64(InFmt_SOPC *iFmt)
        : Inst_SOPC(iFmt, "s_bitcmp1_b64")
    {
        setFlag(ALU);
    } // Inst_SOPC__S_BITCMP1_B64

    Inst_SOPC__S_BITCMP1_B64::~Inst_SOPC__S_BITCMP1_B64()
    {
    } // ~Inst_SOPC__S_BITCMP1_B64

    // --- description from .arch file ---
    // SCC = (S0.u64[S1.u[5:0]] == 1).
    void
    Inst_SOPC__S_BITCMP1_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU64 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        scc = bits(src0.rawData(), bits(src1.rawData(), 5, 0)) ? 1 : 0;

        scc.write();
    } // execute
    // --- Inst_SOPC__S_SETVSKIP class methods ---

    Inst_SOPC__S_SETVSKIP::Inst_SOPC__S_SETVSKIP(InFmt_SOPC *iFmt)
        : Inst_SOPC(iFmt, "s_setvskip")
    {
    } // Inst_SOPC__S_SETVSKIP

    Inst_SOPC__S_SETVSKIP::~Inst_SOPC__S_SETVSKIP()
    {
    } // ~Inst_SOPC__S_SETVSKIP

    // --- description from .arch file ---
    // VSKIP = S0.u[S1.u[4:0]].
    // Enables and disables VSKIP mode.
    // When VSKIP is enabled, no VOP*/M*BUF/MIMG/DS/FLAT/EXP instuctions are
    // issued.
    // If any vector operations are outstanding, S_WAITCNT must be issued
    // before executing.
    // This instruction requires one waitstate after executing (e.g. S_NOP 0).
    // Example:
    //     s_waitcnt 0
    //     s_setvskip 1, 0  // Enable vskip mode.
    //     s_nop 1
    void
    Inst_SOPC__S_SETVSKIP::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_SOPC__S_SET_GPR_IDX_ON class methods ---

    Inst_SOPC__S_SET_GPR_IDX_ON::Inst_SOPC__S_SET_GPR_IDX_ON(InFmt_SOPC *iFmt)
        : Inst_SOPC(iFmt, "s_set_gpr_idx_on")
    {
    } // Inst_SOPC__S_SET_GPR_IDX_ON

    Inst_SOPC__S_SET_GPR_IDX_ON::~Inst_SOPC__S_SET_GPR_IDX_ON()
    {
    } // ~Inst_SOPC__S_SET_GPR_IDX_ON

    // --- description from .arch file ---
    // MODE.gpr_idx_en = 1;
    // M0[7:0] = S0.u[7:0];
    // M0[15:12] = SIMM4 (direct contents of S1 field);
    // // Remaining bits of M0 are unmodified.
    // Enable GPR indexing mode. Vector operations after this will perform
    // relative GPR addressing based on the contents of M0. The structure
    // SQ_M0_GPR_IDX_WORD may be used to decode M0.
    // The raw contents of the S1 field are read and used to set the enable
    // bits. S1[0] = VSRC0_REL, S1[1] = VSRC1_REL, S1[2] = VSRC2_REL and
    // S1[3] = VDST_REL.
    void
    Inst_SOPC__S_SET_GPR_IDX_ON::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_SOPC__S_CMP_EQ_U64 class methods ---

    Inst_SOPC__S_CMP_EQ_U64::Inst_SOPC__S_CMP_EQ_U64(InFmt_SOPC *iFmt)
        : Inst_SOPC(iFmt, "s_cmp_eq_u64")
    {
        setFlag(ALU);
    } // Inst_SOPC__S_CMP_EQ_U64

    Inst_SOPC__S_CMP_EQ_U64::~Inst_SOPC__S_CMP_EQ_U64()
    {
    } // ~Inst_SOPC__S_CMP_EQ_U64

    // --- description from .arch file ---
    // SCC = (S0.i64 == S1.i64).
    void
    Inst_SOPC__S_CMP_EQ_U64::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU64 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU64 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        scc = (src0.rawData() == src1.rawData()) ? 1 : 0;

        scc.write();
    } // execute
    // --- Inst_SOPC__S_CMP_LG_U64 class methods ---

    Inst_SOPC__S_CMP_LG_U64::Inst_SOPC__S_CMP_LG_U64(InFmt_SOPC *iFmt)
        : Inst_SOPC(iFmt, "s_cmp_lg_u64")
    {
        setFlag(ALU);
    } // Inst_SOPC__S_CMP_LG_U64

    Inst_SOPC__S_CMP_LG_U64::~Inst_SOPC__S_CMP_LG_U64()
    {
    } // ~Inst_SOPC__S_CMP_LG_U64

    // --- description from .arch file ---
    // SCC = (S0.i64 != S1.i64).
    void
    Inst_SOPC__S_CMP_LG_U64::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU64 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU64 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        scc = (src0.rawData() != src1.rawData()) ? 1 : 0;

        scc.write();
    } // execute
} // namespace VegaISA
} // namespace gem5
