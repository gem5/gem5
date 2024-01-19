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
#include "dev/amdgpu/hwreg_defines.hh"
#include "gpu-compute/shader.hh"

namespace gem5
{

namespace VegaISA
{
    // --- Inst_SOPK__S_MOVK_I32 class methods ---

    Inst_SOPK__S_MOVK_I32::Inst_SOPK__S_MOVK_I32(InFmt_SOPK *iFmt)
        : Inst_SOPK(iFmt, "s_movk_i32")
    {
        setFlag(ALU);
    } // Inst_SOPK__S_MOVK_I32

    Inst_SOPK__S_MOVK_I32::~Inst_SOPK__S_MOVK_I32()
    {
    } // ~Inst_SOPK__S_MOVK_I32

    // --- description from .arch file ---
    // D.i = signext(SIMM16) (sign extension).
    void
    Inst_SOPK__S_MOVK_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        ScalarRegI32 simm16 = (ScalarRegI32)sext<16>(instData.SIMM16);
        ScalarOperandI32 sdst(gpuDynInst, instData.SDST);

        sdst = simm16;

        sdst.write();
    } // execute
    // --- Inst_SOPK__S_CMOVK_I32 class methods ---

    Inst_SOPK__S_CMOVK_I32::Inst_SOPK__S_CMOVK_I32(InFmt_SOPK *iFmt)
        : Inst_SOPK(iFmt, "s_cmovk_i32")
    {
        setFlag(ALU);
    } // Inst_SOPK__S_CMOVK_I32

    Inst_SOPK__S_CMOVK_I32::~Inst_SOPK__S_CMOVK_I32()
    {
    } // ~Inst_SOPK__S_CMOVK_I32

    // --- description from .arch file ---
    // if (SCC) then D.i = signext(SIMM16);
    // else NOP.
    // Conditional move with sign extension.
    void
    Inst_SOPK__S_CMOVK_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        ScalarRegI32 simm16 = (ScalarRegI32)sext<16>(instData.SIMM16);
        ScalarOperandI32 sdst(gpuDynInst, instData.SDST);
        ConstScalarOperandU32 scc(gpuDynInst, REG_SCC);

        scc.read();

        if (scc.rawData()) {
            sdst = simm16;
            sdst.write();
        }
    } // execute
    // --- Inst_SOPK__S_CMPK_EQ_I32 class methods ---

    Inst_SOPK__S_CMPK_EQ_I32::Inst_SOPK__S_CMPK_EQ_I32(InFmt_SOPK *iFmt)
        : Inst_SOPK(iFmt, "s_cmpk_eq_i32")
    {
        setFlag(ALU);
    } // Inst_SOPK__S_CMPK_EQ_I32

    Inst_SOPK__S_CMPK_EQ_I32::~Inst_SOPK__S_CMPK_EQ_I32()
    {
    } // ~Inst_SOPK__S_CMPK_EQ_I32

    // --- description from .arch file ---
    // SCC = (S0.i == signext(SIMM16)).
    void
    Inst_SOPK__S_CMPK_EQ_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        ScalarRegI32 simm16 = (ScalarRegI32)sext<16>(instData.SIMM16);
        ConstScalarOperandI32 src(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src.read();

        scc = (src.rawData() == simm16) ? 1 : 0;

        scc.write();
    } // execute
    // --- Inst_SOPK__S_CMPK_LG_I32 class methods ---

    Inst_SOPK__S_CMPK_LG_I32::Inst_SOPK__S_CMPK_LG_I32(InFmt_SOPK *iFmt)
        : Inst_SOPK(iFmt, "s_cmpk_lg_i32")
    {
        setFlag(ALU);
    } // Inst_SOPK__S_CMPK_LG_I32

    Inst_SOPK__S_CMPK_LG_I32::~Inst_SOPK__S_CMPK_LG_I32()
    {
    } // ~Inst_SOPK__S_CMPK_LG_I32

    // --- description from .arch file ---
    // SCC = (S0.i != signext(SIMM16)).
    void
    Inst_SOPK__S_CMPK_LG_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        ScalarRegI32 simm16 = (ScalarRegI32)sext<16>(instData.SIMM16);
        ConstScalarOperandI32 src(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src.read();

        scc = (src.rawData() != simm16) ? 1 : 0;

        scc.write();
    } // execute
    // --- Inst_SOPK__S_CMPK_GT_I32 class methods ---

    Inst_SOPK__S_CMPK_GT_I32::Inst_SOPK__S_CMPK_GT_I32(InFmt_SOPK *iFmt)
        : Inst_SOPK(iFmt, "s_cmpk_gt_i32")
    {
        setFlag(ALU);
    } // Inst_SOPK__S_CMPK_GT_I32

    Inst_SOPK__S_CMPK_GT_I32::~Inst_SOPK__S_CMPK_GT_I32()
    {
    } // ~Inst_SOPK__S_CMPK_GT_I32

    // --- description from .arch file ---
    // SCC = (S0.i > signext(SIMM16)).
    void
    Inst_SOPK__S_CMPK_GT_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        ScalarRegI32 simm16 = (ScalarRegI32)sext<16>(instData.SIMM16);
        ConstScalarOperandI32 src(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src.read();

        scc = (src.rawData() > simm16) ? 1 : 0;

        scc.write();
    } // execute
    // --- Inst_SOPK__S_CMPK_GE_I32 class methods ---

    Inst_SOPK__S_CMPK_GE_I32::Inst_SOPK__S_CMPK_GE_I32(InFmt_SOPK *iFmt)
        : Inst_SOPK(iFmt, "s_cmpk_ge_i32")
    {
        setFlag(ALU);
    } // Inst_SOPK__S_CMPK_GE_I32

    Inst_SOPK__S_CMPK_GE_I32::~Inst_SOPK__S_CMPK_GE_I32()
    {
    } // ~Inst_SOPK__S_CMPK_GE_I32

    // --- description from .arch file ---
    // SCC = (S0.i >= signext(SIMM16)).
    void
    Inst_SOPK__S_CMPK_GE_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        ScalarRegI32 simm16 = (ScalarRegI32)sext<16>(instData.SIMM16);
        ConstScalarOperandI32 src(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src.read();

        scc = (src.rawData() >= simm16) ? 1 : 0;

        scc.write();
    } // execute
    // --- Inst_SOPK__S_CMPK_LT_I32 class methods ---

    Inst_SOPK__S_CMPK_LT_I32::Inst_SOPK__S_CMPK_LT_I32(InFmt_SOPK *iFmt)
        : Inst_SOPK(iFmt, "s_cmpk_lt_i32")
    {
        setFlag(ALU);
    } // Inst_SOPK__S_CMPK_LT_I32

    Inst_SOPK__S_CMPK_LT_I32::~Inst_SOPK__S_CMPK_LT_I32()
    {
    } // ~Inst_SOPK__S_CMPK_LT_I32

    // --- description from .arch file ---
    // SCC = (S0.i < signext(SIMM16)).
    void
    Inst_SOPK__S_CMPK_LT_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        ScalarRegI32 simm16 = (ScalarRegI32)sext<16>(instData.SIMM16);
        ConstScalarOperandI32 src(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src.read();

        scc = (src.rawData() < simm16) ? 1 : 0;

        scc.write();
    } // execute
    // --- Inst_SOPK__S_CMPK_LE_I32 class methods ---

    Inst_SOPK__S_CMPK_LE_I32::Inst_SOPK__S_CMPK_LE_I32(InFmt_SOPK *iFmt)
        : Inst_SOPK(iFmt, "s_cmpk_le_i32")
    {
        setFlag(ALU);
    } // Inst_SOPK__S_CMPK_LE_I32

    Inst_SOPK__S_CMPK_LE_I32::~Inst_SOPK__S_CMPK_LE_I32()
    {
    } // ~Inst_SOPK__S_CMPK_LE_I32

    // --- description from .arch file ---
    // SCC = (S0.i <= signext(SIMM16)).
    void
    Inst_SOPK__S_CMPK_LE_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        ScalarRegI32 simm16 = (ScalarRegI32)sext<16>(instData.SIMM16);
        ConstScalarOperandI32 src(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src.read();

        scc = (src.rawData() <= simm16) ? 1 : 0;

        scc.write();
    } // execute
    // --- Inst_SOPK__S_CMPK_EQ_U32 class methods ---

    Inst_SOPK__S_CMPK_EQ_U32::Inst_SOPK__S_CMPK_EQ_U32(InFmt_SOPK *iFmt)
        : Inst_SOPK(iFmt, "s_cmpk_eq_u32")
    {
        setFlag(ALU);
    } // Inst_SOPK__S_CMPK_EQ_U32

    Inst_SOPK__S_CMPK_EQ_U32::~Inst_SOPK__S_CMPK_EQ_U32()
    {
    } // ~Inst_SOPK__S_CMPK_EQ_U32

    // --- description from .arch file ---
    // SCC = (S0.u == SIMM16).
    void
    Inst_SOPK__S_CMPK_EQ_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        ScalarRegU32 simm16 = (ScalarRegU32)instData.SIMM16;
        ConstScalarOperandU32 src(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src.read();

        scc = (src.rawData() == simm16) ? 1 : 0;

        scc.write();
    } // execute
    // --- Inst_SOPK__S_CMPK_LG_U32 class methods ---

    Inst_SOPK__S_CMPK_LG_U32::Inst_SOPK__S_CMPK_LG_U32(InFmt_SOPK *iFmt)
        : Inst_SOPK(iFmt, "s_cmpk_lg_u32")
    {
        setFlag(ALU);
    } // Inst_SOPK__S_CMPK_LG_U32

    Inst_SOPK__S_CMPK_LG_U32::~Inst_SOPK__S_CMPK_LG_U32()
    {
    } // ~Inst_SOPK__S_CMPK_LG_U32

    // --- description from .arch file ---
    // SCC = (S0.u != SIMM16).
    void
    Inst_SOPK__S_CMPK_LG_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        ScalarRegU32 simm16 = (ScalarRegU32)instData.SIMM16;
        ConstScalarOperandU32 src(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src.read();

        scc = (src.rawData() != simm16) ? 1 : 0;

        scc.write();
    } // execute
    // --- Inst_SOPK__S_CMPK_GT_U32 class methods ---

    Inst_SOPK__S_CMPK_GT_U32::Inst_SOPK__S_CMPK_GT_U32(InFmt_SOPK *iFmt)
        : Inst_SOPK(iFmt, "s_cmpk_gt_u32")
    {
        setFlag(ALU);
    } // Inst_SOPK__S_CMPK_GT_U32

    Inst_SOPK__S_CMPK_GT_U32::~Inst_SOPK__S_CMPK_GT_U32()
    {
    } // ~Inst_SOPK__S_CMPK_GT_U32

    // --- description from .arch file ---
    // SCC = (S0.u > SIMM16).
    void
    Inst_SOPK__S_CMPK_GT_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        ScalarRegU32 simm16 = (ScalarRegU32)instData.SIMM16;
        ConstScalarOperandU32 src(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src.read();

        scc = (src.rawData() > simm16) ? 1 : 0;

        scc.write();
    } // execute
    // --- Inst_SOPK__S_CMPK_GE_U32 class methods ---

    Inst_SOPK__S_CMPK_GE_U32::Inst_SOPK__S_CMPK_GE_U32(InFmt_SOPK *iFmt)
        : Inst_SOPK(iFmt, "s_cmpk_ge_u32")
    {
        setFlag(ALU);
    } // Inst_SOPK__S_CMPK_GE_U32

    Inst_SOPK__S_CMPK_GE_U32::~Inst_SOPK__S_CMPK_GE_U32()
    {
    } // ~Inst_SOPK__S_CMPK_GE_U32

    // --- description from .arch file ---
    // SCC = (S0.u >= SIMM16).
    void
    Inst_SOPK__S_CMPK_GE_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        ScalarRegU32 simm16 = (ScalarRegU32)instData.SIMM16;
        ConstScalarOperandU32 src(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src.read();

        scc = (src.rawData() >= simm16) ? 1 : 0;

        scc.write();
    } // execute
    // --- Inst_SOPK__S_CMPK_LT_U32 class methods ---

    Inst_SOPK__S_CMPK_LT_U32::Inst_SOPK__S_CMPK_LT_U32(InFmt_SOPK *iFmt)
        : Inst_SOPK(iFmt, "s_cmpk_lt_u32")
    {
        setFlag(ALU);
    } // Inst_SOPK__S_CMPK_LT_U32

    Inst_SOPK__S_CMPK_LT_U32::~Inst_SOPK__S_CMPK_LT_U32()
    {
    } // ~Inst_SOPK__S_CMPK_LT_U32

    // --- description from .arch file ---
    // SCC = (S0.u < SIMM16).
    void
    Inst_SOPK__S_CMPK_LT_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        ScalarRegU32 simm16 = (ScalarRegU32)instData.SIMM16;
        ConstScalarOperandU32 src(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src.read();

        scc = (src.rawData() < simm16) ? 1 : 0;

        scc.write();
    } // execute
    // --- Inst_SOPK__S_CMPK_LE_U32 class methods ---

    Inst_SOPK__S_CMPK_LE_U32::Inst_SOPK__S_CMPK_LE_U32(InFmt_SOPK *iFmt)
        : Inst_SOPK(iFmt, "s_cmpk_le_u32")
    {
        setFlag(ALU);
    } // Inst_SOPK__S_CMPK_LE_U32

    Inst_SOPK__S_CMPK_LE_U32::~Inst_SOPK__S_CMPK_LE_U32()
    {
    } // ~Inst_SOPK__S_CMPK_LE_U32

    // --- description from .arch file ---
    // SCC = (S0.u <= SIMM16).
    void
    Inst_SOPK__S_CMPK_LE_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        ScalarRegU32 simm16 = (ScalarRegU32)instData.SIMM16;
        ConstScalarOperandU32 src(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src.read();

        scc = (src.rawData() <= simm16) ? 1 : 0;

        scc.write();
    } // execute
    // --- Inst_SOPK__S_ADDK_I32 class methods ---

    Inst_SOPK__S_ADDK_I32::Inst_SOPK__S_ADDK_I32(InFmt_SOPK *iFmt)
        : Inst_SOPK(iFmt, "s_addk_i32")
    {
        setFlag(ALU);
    } // Inst_SOPK__S_ADDK_I32

    Inst_SOPK__S_ADDK_I32::~Inst_SOPK__S_ADDK_I32()
    {
    } // ~Inst_SOPK__S_ADDK_I32

    // --- description from .arch file ---
    // D.i = D.i + signext(SIMM16);
    // SCC = overflow.
    void
    Inst_SOPK__S_ADDK_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        ScalarRegI16 simm16 = instData.SIMM16;
        ConstScalarOperandI32 src(gpuDynInst, instData.SDST);
        ScalarOperandI32 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src.read();

        sdst = src.rawData() + (ScalarRegI32)sext<16>(simm16);
        scc = (bits(src.rawData(), 31) == bits(simm16, 15)
            && bits(src.rawData(), 31) != bits(sdst.rawData(), 31)) ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOPK__S_MULK_I32 class methods ---

    Inst_SOPK__S_MULK_I32::Inst_SOPK__S_MULK_I32(InFmt_SOPK *iFmt)
        : Inst_SOPK(iFmt, "s_mulk_i32")
    {
        setFlag(ALU);
    } // Inst_SOPK__S_MULK_I32

    Inst_SOPK__S_MULK_I32::~Inst_SOPK__S_MULK_I32()
    {
    } // ~Inst_SOPK__S_MULK_I32

    // --- description from .arch file ---
    // D.i = D.i * signext(SIMM16).
    void
    Inst_SOPK__S_MULK_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        ScalarRegI16 simm16 = instData.SIMM16;
        ConstScalarOperandI32 src(gpuDynInst, instData.SDST);
        ScalarOperandI32 sdst(gpuDynInst, instData.SDST);

        src.read();

        sdst = src.rawData() * (ScalarRegI32)sext<16>(simm16);

        sdst.write();
    } // execute
    // --- Inst_SOPK__S_CBRANCH_I_FORK class methods ---

    Inst_SOPK__S_CBRANCH_I_FORK::Inst_SOPK__S_CBRANCH_I_FORK(InFmt_SOPK *iFmt)
        : Inst_SOPK(iFmt, "s_cbranch_i_fork")
    {
        setFlag(Branch);
    } // Inst_SOPK__S_CBRANCH_I_FORK

    Inst_SOPK__S_CBRANCH_I_FORK::~Inst_SOPK__S_CBRANCH_I_FORK()
    {
    } // ~Inst_SOPK__S_CBRANCH_I_FORK

    // --- description from .arch file ---
    // mask_pass = S0.u64 & EXEC;
    // mask_fail = ~S0.u64 & EXEC;
    // target_addr = PC + signext(SIMM16 * 4) + 4;
    // if (mask_pass == EXEC)
    //     PC = target_addr;
    // elsif (mask_fail == EXEC)
    //     PC += 4;
    // elsif (bitcount(mask_fail) < bitcount(mask_pass))
    //     EXEC = mask_fail;
    //     SGPR[CSP*4] = { target_addr, mask_pass };
    //     CSP++;
    //     PC += 4;
    // else
    //     EXEC = mask_pass;
    //     SGPR[CSP*4] = { PC + 4, mask_fail };
    //     CSP++;
    //     PC = target_addr;
    // end.
    // Conditional branch using branch-stack.
    // S0 = compare mask(vcc or any sgpr), and
    // SIMM16 = signed DWORD branch offset relative to next instruction.
    // See also S_CBRANCH_JOIN.
    void
    Inst_SOPK__S_CBRANCH_I_FORK::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_SOPK__S_GETREG_B32 class methods ---

    Inst_SOPK__S_GETREG_B32::Inst_SOPK__S_GETREG_B32(InFmt_SOPK *iFmt)
        : Inst_SOPK(iFmt, "s_getreg_b32")
    {
        setFlag(ALU);
    } // Inst_SOPK__S_GETREG_B32

    Inst_SOPK__S_GETREG_B32::~Inst_SOPK__S_GETREG_B32()
    {
    } // ~Inst_SOPK__S_GETREG_B32

    // --- description from .arch file ---
    // D.u = hardware-reg. Read some or all of a hardware register into the
    // LSBs of D.
    // SIMM16 = {size[4:0], offset[4:0], hwRegId[5:0]}; offset is 0..31, size
    // is 1..32.
    void
    Inst_SOPK__S_GETREG_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        ScalarRegI16 simm16 = instData.SIMM16;
        ScalarRegU32 hwregId = simm16 & 0x3f;
        ScalarRegU32 offset = (simm16 >> 6) & 31;
        ScalarRegU32 size = ((simm16 >> 11) & 31) + 1;

        ScalarRegU32 hwreg =
            gpuDynInst->computeUnit()->shader->getHwReg(hwregId);
        ScalarOperandU32 sdst(gpuDynInst, instData.SDST);
        sdst.read();

        // Store value from hardware to part of the SDST.
        ScalarRegU32 mask = (((1U << size) - 1U) << offset);
        sdst = (hwreg & mask) >> offset;
        sdst.write();
    } // execute
    // --- Inst_SOPK__S_SETREG_B32 class methods ---

    Inst_SOPK__S_SETREG_B32::Inst_SOPK__S_SETREG_B32(InFmt_SOPK *iFmt)
        : Inst_SOPK(iFmt, "s_setreg_b32")
    {
        setFlag(ALU);
    } // Inst_SOPK__S_SETREG_B32

    Inst_SOPK__S_SETREG_B32::~Inst_SOPK__S_SETREG_B32()
    {
    } // ~Inst_SOPK__S_SETREG_B32

    // --- description from .arch file ---
    // hardware-reg = S0.u. Write some or all of the LSBs of D into a hardware
    // register.
    // SIMM16 = {size[4:0], offset[4:0], hwRegId[5:0]}; offset is 0..31, size
    // is 1..32.
    void
    Inst_SOPK__S_SETREG_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        ScalarRegI16 simm16 = instData.SIMM16;
        ScalarRegU32 hwregId = simm16 & 0x3f;
        ScalarRegU32 offset = (simm16 >> 6) & 31;
        ScalarRegU32 size = ((simm16 >> 11) & 31) + 1;

        ScalarRegU32 hwreg =
            gpuDynInst->computeUnit()->shader->getHwReg(hwregId);
        ScalarOperandU32 sdst(gpuDynInst, instData.SDST);
        sdst.read();

        // Store value from SDST to part of the hardware register.
        ScalarRegU32 mask = (((1U << size) - 1U) << offset);
        hwreg = ((hwreg & ~mask) | ((sdst.rawData() << offset) & mask));
        gpuDynInst->computeUnit()->shader->setHwReg(hwregId, hwreg);

        // set MODE register to control the behavior of single precision
        // floating-point numbers: denormal mode or round mode
        if (hwregId==1 && size==2
                        && (offset==4 || offset==0)) {
            warn_once("Be cautious that s_setreg_b32 has no real effect "
                            "on FP modes: %s\n", gpuDynInst->disassemble());
            return;
        }

        // panic if not changing MODE of floating-point numbers
        panicUnimplemented();
    } // execute
    // --- Inst_SOPK__S_SETREG_IMM32_B32 class methods ---

    Inst_SOPK__S_SETREG_IMM32_B32::Inst_SOPK__S_SETREG_IMM32_B32(
          InFmt_SOPK *iFmt)
        : Inst_SOPK(iFmt, "s_setreg_imm32_b32")
    {
        setFlag(ALU);
    } // Inst_SOPK__S_SETREG_IMM32_B32

    Inst_SOPK__S_SETREG_IMM32_B32::~Inst_SOPK__S_SETREG_IMM32_B32()
    {
    } // ~Inst_SOPK__S_SETREG_IMM32_B32

    // --- description from .arch file ---
    // Write some or all of the LSBs of IMM32 into a hardware register; this
    // ---  instruction requires a 32-bit literal constant.
    // SIMM16 = {size[4:0], offset[4:0], hwRegId[5:0]}; offset is 0..31, size
    // is 1..32.
    void
    Inst_SOPK__S_SETREG_IMM32_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        ScalarRegI16 simm16 = instData.SIMM16;
        ScalarRegU32 hwregId = simm16 & 0x3f;
        ScalarRegU32 offset = (simm16 >> 6) & 31;
        ScalarRegU32 size = ((simm16 >> 11) & 31) + 1;

        ScalarRegU32 hwreg =
            gpuDynInst->computeUnit()->shader->getHwReg(hwregId);
        ScalarRegI32 simm32 = extData.imm_u32;

        // Store value from SIMM32 to part of the hardware register.
        ScalarRegU32 mask = (((1U << size) - 1U) << offset);
        hwreg = ((hwreg & ~mask) | ((simm32 << offset) & mask));
        gpuDynInst->computeUnit()->shader->setHwReg(hwregId, hwreg);

        // set MODE register to control the behavior of single precision
        // floating-point numbers: denormal mode or round mode
        if (hwregId==HW_REG_MODE && size==2
                        && (offset==4 || offset==0)) {
            warn_once("Be cautious that s_setreg_imm32_b32 has no real effect "
                            "on FP modes: %s\n", gpuDynInst->disassemble());
            return;
        }

        // panic if not changing modes of single-precision FPs
        panicUnimplemented();
    } // execute
} // namespace VegaISA
} // namespace gem5
