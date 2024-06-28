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
    // --- Inst_SOP2__S_ADD_U32 class methods ---

    Inst_SOP2__S_ADD_U32::Inst_SOP2__S_ADD_U32(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_add_u32")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_ADD_U32

    Inst_SOP2__S_ADD_U32::~Inst_SOP2__S_ADD_U32()
    {
    } // ~Inst_SOP2__S_ADD_U32

    // --- description from .arch file ---
    // D.u = S0.u + S1.u;
    // SCC = (S0.u + S1.u >= 0x800000000ULL ? 1 : 0) is an unsigned
    // ---  overflow/carry-out for S_ADDC_U32.
    void
    Inst_SOP2__S_ADD_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = src0.rawData() + src1.rawData();
        scc = ((ScalarRegU64)src0.rawData() + (ScalarRegU64)src1.rawData())
            >= 0x100000000ULL ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_SUB_U32 class methods ---

    Inst_SOP2__S_SUB_U32::Inst_SOP2__S_SUB_U32(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_sub_u32")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_SUB_U32

    Inst_SOP2__S_SUB_U32::~Inst_SOP2__S_SUB_U32()
    {
    } // ~Inst_SOP2__S_SUB_U32

    // --- description from .arch file ---
    // D.u = S0.u - S1.u;
    // SCC = (S1.u > S0.u ? 1 : 0) is an unsigned overflow or carry-out for
    // ---  S_SUBB_U32.
    void
    Inst_SOP2__S_SUB_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = src0.rawData() - src1.rawData();
        scc = (src1.rawData() > src0.rawData()) ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_ADD_I32 class methods ---

    Inst_SOP2__S_ADD_I32::Inst_SOP2__S_ADD_I32(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_add_i32")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_ADD_I32

    Inst_SOP2__S_ADD_I32::~Inst_SOP2__S_ADD_I32()
    {
    } // ~Inst_SOP2__S_ADD_I32

    // --- description from .arch file ---
    // D.i = S0.i + S1.i;
    // SCC = (S0.u[31] == S1.u[31] && S0.u[31] != D.u[31]) is a signed
    // overflow.
    // This opcode is not suitable for use with S_ADDC_U32 for implementing
    // 64-bit operations.
    void
    Inst_SOP2__S_ADD_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandI32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandI32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandI32 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = src0.rawData() + src1.rawData();
        scc = (bits(src0.rawData(), 31) == bits(src1.rawData(), 31)
            && bits(src0.rawData(), 31) != bits(sdst.rawData(), 31))
            ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_SUB_I32 class methods ---

    Inst_SOP2__S_SUB_I32::Inst_SOP2__S_SUB_I32(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_sub_i32")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_SUB_I32

    Inst_SOP2__S_SUB_I32::~Inst_SOP2__S_SUB_I32()
    {
    } // ~Inst_SOP2__S_SUB_I32

    // --- description from .arch file ---
    // D.i = S0.i - S1.i;
    // SCC = (S0.u[31] != S1.u[31] && S0.u[31] != D.u[31]) is a signed
    // overflow.
    // CAUTION: The condition code behaviour for this opcode is inconsistent
    // with V_SUB_I32; see V_SUB_I32 for further details.
    // This opcode is not suitable for use with S_SUBB_U32 for implementing
    // 64-bit operations.
    void
    Inst_SOP2__S_SUB_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandI32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandI32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandI32 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = src0.rawData() - src1.rawData();
        scc = (bits(src0.rawData(), 31) != bits(src1.rawData(), 31)
            && bits(src0.rawData(), 31) != bits(sdst.rawData(), 31)) ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_ADDC_U32 class methods ---

    Inst_SOP2__S_ADDC_U32::Inst_SOP2__S_ADDC_U32(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_addc_u32")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_ADDC_U32

    Inst_SOP2__S_ADDC_U32::~Inst_SOP2__S_ADDC_U32()
    {
    } // ~Inst_SOP2__S_ADDC_U32

    // --- description from .arch file ---
    // D.u = S0.u + S1.u + SCC;
    // SCC = (S0.u + S1.u + SCC >= 0x800000000ULL ? 1 : 0) is an unsigned
    // overflow.
    void
    Inst_SOP2__S_ADDC_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();
        scc.read();

        sdst = src0.rawData() + src1.rawData() + scc.rawData();
        scc = ((ScalarRegU64)src0.rawData() + (ScalarRegU64)src1.rawData()
            + (ScalarRegU64)scc.rawData()) >= 0x100000000ULL ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_SUBB_U32 class methods ---

    Inst_SOP2__S_SUBB_U32::Inst_SOP2__S_SUBB_U32(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_subb_u32")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_SUBB_U32

    Inst_SOP2__S_SUBB_U32::~Inst_SOP2__S_SUBB_U32()
    {
    } // ~Inst_SOP2__S_SUBB_U32

    // --- description from .arch file ---
    // D.u = S0.u - S1.u - SCC;
    // SCC = (S1.u + SCC > S0.u ? 1 : 0) is an unsigned overflow.
    void
    Inst_SOP2__S_SUBB_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();
        scc.read();

        sdst = src0.rawData() - src1.rawData() - scc.rawData();
        scc = (src1.rawData() + scc.rawData()) > src0.rawData() ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_MIN_I32 class methods ---

    Inst_SOP2__S_MIN_I32::Inst_SOP2__S_MIN_I32(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_min_i32")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_MIN_I32

    Inst_SOP2__S_MIN_I32::~Inst_SOP2__S_MIN_I32()
    {
    } // ~Inst_SOP2__S_MIN_I32

    // --- description from .arch file ---
    // D.i = (S0.i < S1.i) ? S0.i : S1.i;
    // SCC = 1 if S0 is chosen as the minimum value.
    void
    Inst_SOP2__S_MIN_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandI32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandI32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandI32 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = std::min(src0.rawData(), src1.rawData());
        scc = (src0.rawData() < src1.rawData()) ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_MIN_U32 class methods ---

    Inst_SOP2__S_MIN_U32::Inst_SOP2__S_MIN_U32(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_min_u32")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_MIN_U32

    Inst_SOP2__S_MIN_U32::~Inst_SOP2__S_MIN_U32()
    {
    } // ~Inst_SOP2__S_MIN_U32

    // --- description from .arch file ---
    // D.u = (S0.u < S1.u) ? S0.u : S1.u;
    // SCC = 1 if S0 is chosen as the minimum value.
    void
    Inst_SOP2__S_MIN_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = std::min(src0.rawData(), src1.rawData());
        scc = (src0.rawData() < src1.rawData()) ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_MAX_I32 class methods ---

    Inst_SOP2__S_MAX_I32::Inst_SOP2__S_MAX_I32(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_max_i32")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_MAX_I32

    Inst_SOP2__S_MAX_I32::~Inst_SOP2__S_MAX_I32()
    {
    } // ~Inst_SOP2__S_MAX_I32

    // --- description from .arch file ---
    // D.i = (S0.i > S1.i) ? S0.i : S1.i;
    // SCC = 1 if S0 is chosen as the maximum value.
    void
    Inst_SOP2__S_MAX_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandI32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandI32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandI32 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = std::max(src0.rawData(), src1.rawData());
        scc = (src0.rawData() > src1.rawData()) ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_MAX_U32 class methods ---

    Inst_SOP2__S_MAX_U32::Inst_SOP2__S_MAX_U32(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_max_u32")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_MAX_U32

    Inst_SOP2__S_MAX_U32::~Inst_SOP2__S_MAX_U32()
    {
    } // ~Inst_SOP2__S_MAX_U32

    // --- description from .arch file ---
    // D.u = (S0.u > S1.u) ? S0.u : S1.u;
    // SCC = 1 if S0 is chosen as the maximum value.
    void
    Inst_SOP2__S_MAX_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = std::max(src0.rawData(), src1.rawData());
        scc = (src0.rawData() > src1.rawData()) ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_CSELECT_B32 class methods ---

    Inst_SOP2__S_CSELECT_B32::Inst_SOP2__S_CSELECT_B32(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_cselect_b32")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_CSELECT_B32

    Inst_SOP2__S_CSELECT_B32::~Inst_SOP2__S_CSELECT_B32()
    {
    } // ~Inst_SOP2__S_CSELECT_B32

    // --- description from .arch file ---
    // D.u = SCC ? S0.u : S1.u (conditional select).
    void
    Inst_SOP2__S_CSELECT_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 sdst(gpuDynInst, instData.SDST);
        ConstScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();
        scc.read();

        sdst = scc.rawData() ? src0.rawData() : src1.rawData();

        sdst.write();
    } // execute
    // --- Inst_SOP2__S_CSELECT_B64 class methods ---

    Inst_SOP2__S_CSELECT_B64::Inst_SOP2__S_CSELECT_B64(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_cselect_b64")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_CSELECT_B64

    Inst_SOP2__S_CSELECT_B64::~Inst_SOP2__S_CSELECT_B64()
    {
    } // ~Inst_SOP2__S_CSELECT_B64

    // --- description from .arch file ---
    // D.u64 = SCC ? S0.u64 : S1.u64 (conditional select).
    void
    Inst_SOP2__S_CSELECT_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU64 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU64 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU64 sdst(gpuDynInst, instData.SDST);
        ConstScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();
        scc.read();

        sdst = scc.rawData() ? src0.rawData() : src1.rawData();

        sdst.write();
    } // execute
    // --- Inst_SOP2__S_AND_B32 class methods ---

    Inst_SOP2__S_AND_B32::Inst_SOP2__S_AND_B32(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_and_b32")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_AND_B32

    Inst_SOP2__S_AND_B32::~Inst_SOP2__S_AND_B32()
    {
    } // ~Inst_SOP2__S_AND_B32

    // --- description from .arch file ---
    // D.u = S0.u & S1.u;
    // SCC = 1 if result is non-zero.
    void
    Inst_SOP2__S_AND_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = src0.rawData() & src1.rawData();
        scc = sdst.rawData() ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_AND_B64 class methods ---

    Inst_SOP2__S_AND_B64::Inst_SOP2__S_AND_B64(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_and_b64")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_AND_B64

    Inst_SOP2__S_AND_B64::~Inst_SOP2__S_AND_B64()
    {
    } // ~Inst_SOP2__S_AND_B64

    // --- description from .arch file ---
    // D.u64 = S0.u64 & S1.u64;
    // SCC = 1 if result is non-zero.
    void
    Inst_SOP2__S_AND_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU64 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU64 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU64 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = src0.rawData() & src1.rawData();
        scc = sdst.rawData() ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_OR_B32 class methods ---

    Inst_SOP2__S_OR_B32::Inst_SOP2__S_OR_B32(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_or_b32")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_OR_B32

    Inst_SOP2__S_OR_B32::~Inst_SOP2__S_OR_B32()
    {
    } // ~Inst_SOP2__S_OR_B32

    // --- description from .arch file ---
    // D.u = S0.u | S1.u;
    // SCC = 1 if result is non-zero.
    void
    Inst_SOP2__S_OR_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = src0.rawData() | src1.rawData();
        scc = sdst.rawData() ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_OR_B64 class methods ---

    Inst_SOP2__S_OR_B64::Inst_SOP2__S_OR_B64(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_or_b64")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_OR_B64

    Inst_SOP2__S_OR_B64::~Inst_SOP2__S_OR_B64()
    {
    } // ~Inst_SOP2__S_OR_B64

    // --- description from .arch file ---
    // D.u64 = S0.u64 | S1.u64;
    // SCC = 1 if result is non-zero.
    void
    Inst_SOP2__S_OR_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU64 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU64 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU64 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = src0.rawData() | src1.rawData();
        scc = sdst.rawData() ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_XOR_B32 class methods ---

    Inst_SOP2__S_XOR_B32::Inst_SOP2__S_XOR_B32(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_xor_b32")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_XOR_B32

    Inst_SOP2__S_XOR_B32::~Inst_SOP2__S_XOR_B32()
    {
    } // ~Inst_SOP2__S_XOR_B32

    // --- description from .arch file ---
    // D.u = S0.u ^ S1.u;
    // SCC = 1 if result is non-zero.
    void
    Inst_SOP2__S_XOR_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = src0.rawData() ^ src1.rawData();
        scc = sdst.rawData() ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_XOR_B64 class methods ---

    Inst_SOP2__S_XOR_B64::Inst_SOP2__S_XOR_B64(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_xor_b64")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_XOR_B64

    Inst_SOP2__S_XOR_B64::~Inst_SOP2__S_XOR_B64()
    {
    } // ~Inst_SOP2__S_XOR_B64

    // --- description from .arch file ---
    // D.u64 = S0.u64 ^ S1.u64;
    // SCC = 1 if result is non-zero.
    void
    Inst_SOP2__S_XOR_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU64 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU64 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU64 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = src0.rawData() ^ src1.rawData();
        scc = sdst.rawData() ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_ANDN2_B32 class methods ---

    Inst_SOP2__S_ANDN2_B32::Inst_SOP2__S_ANDN2_B32(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_andn2_b32")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_ANDN2_B32

    Inst_SOP2__S_ANDN2_B32::~Inst_SOP2__S_ANDN2_B32()
    {
    } // ~Inst_SOP2__S_ANDN2_B32

    // --- description from .arch file ---
    // D.u = S0.u & ~S1.u;
    // SCC = 1 if result is non-zero.
    void
    Inst_SOP2__S_ANDN2_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = src0.rawData() &~ src1.rawData();
        scc = sdst.rawData() ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_ANDN2_B64 class methods ---

    Inst_SOP2__S_ANDN2_B64::Inst_SOP2__S_ANDN2_B64(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_andn2_b64")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_ANDN2_B64

    Inst_SOP2__S_ANDN2_B64::~Inst_SOP2__S_ANDN2_B64()
    {
    } // ~Inst_SOP2__S_ANDN2_B64

    // --- description from .arch file ---
    // D.u64 = S0.u64 & ~S1.u64;
    // SCC = 1 if result is non-zero.
    void
    Inst_SOP2__S_ANDN2_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU64 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU64 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU64 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = src0.rawData() &~ src1.rawData();
        scc = sdst.rawData() ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_ORN2_B32 class methods ---

    Inst_SOP2__S_ORN2_B32::Inst_SOP2__S_ORN2_B32(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_orn2_b32")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_ORN2_B32

    Inst_SOP2__S_ORN2_B32::~Inst_SOP2__S_ORN2_B32()
    {
    } // ~Inst_SOP2__S_ORN2_B32

    // --- description from .arch file ---
    // D.u = S0.u | ~S1.u;
    // SCC = 1 if result is non-zero.
    void
    Inst_SOP2__S_ORN2_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = src0.rawData() |~ src1.rawData();
        scc = sdst.rawData() ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_ORN2_B64 class methods ---

    Inst_SOP2__S_ORN2_B64::Inst_SOP2__S_ORN2_B64(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_orn2_b64")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_ORN2_B64

    Inst_SOP2__S_ORN2_B64::~Inst_SOP2__S_ORN2_B64()
    {
    } // ~Inst_SOP2__S_ORN2_B64

    // --- description from .arch file ---
    // D.u64 = S0.u64 | ~S1.u64;
    // SCC = 1 if result is non-zero.
    void
    Inst_SOP2__S_ORN2_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU64 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU64 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU64 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = src0.rawData() |~ src1.rawData();
        scc = sdst.rawData() ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_NAND_B32 class methods ---

    Inst_SOP2__S_NAND_B32::Inst_SOP2__S_NAND_B32(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_nand_b32")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_NAND_B32

    Inst_SOP2__S_NAND_B32::~Inst_SOP2__S_NAND_B32()
    {
    } // ~Inst_SOP2__S_NAND_B32

    // --- description from .arch file ---
    // D.u = ~(S0.u & S1.u);
    // SCC = 1 if result is non-zero.
    void
    Inst_SOP2__S_NAND_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = ~(src0.rawData() & src1.rawData());
        scc = sdst.rawData() ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_NAND_B64 class methods ---

    Inst_SOP2__S_NAND_B64::Inst_SOP2__S_NAND_B64(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_nand_b64")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_NAND_B64

    Inst_SOP2__S_NAND_B64::~Inst_SOP2__S_NAND_B64()
    {
    } // ~Inst_SOP2__S_NAND_B64

    // --- description from .arch file ---
    // D.u64 = ~(S0.u64 & S1.u64);
    // SCC = 1 if result is non-zero.
    void
    Inst_SOP2__S_NAND_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU64 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU64 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU64 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = ~(src0.rawData() & src1.rawData());
        scc = sdst.rawData() ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_NOR_B32 class methods ---

    Inst_SOP2__S_NOR_B32::Inst_SOP2__S_NOR_B32(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_nor_b32")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_NOR_B32

    Inst_SOP2__S_NOR_B32::~Inst_SOP2__S_NOR_B32()
    {
    } // ~Inst_SOP2__S_NOR_B32

    // --- description from .arch file ---
    // D.u = ~(S0.u | S1.u);
    // SCC = 1 if result is non-zero.
    void
    Inst_SOP2__S_NOR_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = ~(src0.rawData() | src1.rawData());
        scc = sdst.rawData() ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_NOR_B64 class methods ---

    Inst_SOP2__S_NOR_B64::Inst_SOP2__S_NOR_B64(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_nor_b64")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_NOR_B64

    Inst_SOP2__S_NOR_B64::~Inst_SOP2__S_NOR_B64()
    {
    } // ~Inst_SOP2__S_NOR_B64

    // --- description from .arch file ---
    // D.u64 = ~(S0.u64 | S1.u64);
    // SCC = 1 if result is non-zero.
    void
    Inst_SOP2__S_NOR_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU64 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU64 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU64 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = ~(src0.rawData() | src1.rawData());
        scc = sdst.rawData() ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_XNOR_B32 class methods ---

    Inst_SOP2__S_XNOR_B32::Inst_SOP2__S_XNOR_B32(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_xnor_b32")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_XNOR_B32

    Inst_SOP2__S_XNOR_B32::~Inst_SOP2__S_XNOR_B32()
    {
    } // ~Inst_SOP2__S_XNOR_B32

    // --- description from .arch file ---
    // D.u = ~(S0.u ^ S1.u);
    // SCC = 1 if result is non-zero.
    void
    Inst_SOP2__S_XNOR_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = ~(src0.rawData() ^ src1.rawData());
        scc = sdst.rawData() ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_XNOR_B64 class methods ---

    Inst_SOP2__S_XNOR_B64::Inst_SOP2__S_XNOR_B64(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_xnor_b64")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_XNOR_B64

    Inst_SOP2__S_XNOR_B64::~Inst_SOP2__S_XNOR_B64()
    {
    } // ~Inst_SOP2__S_XNOR_B64

    // --- description from .arch file ---
    // D.u64 = ~(S0.u64 ^ S1.u64);
    // SCC = 1 if result is non-zero.
    void
    Inst_SOP2__S_XNOR_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU64 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU64 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU64 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = ~(src0.rawData() ^ src1.rawData());
        scc = sdst.rawData() ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_LSHL_B32 class methods ---

    Inst_SOP2__S_LSHL_B32::Inst_SOP2__S_LSHL_B32(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_lshl_b32")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_LSHL_B32

    Inst_SOP2__S_LSHL_B32::~Inst_SOP2__S_LSHL_B32()
    {
    } // ~Inst_SOP2__S_LSHL_B32

    // --- description from .arch file ---
    // D.u = S0.u << S1.u[4:0];
    // SCC = 1 if result is non-zero.
    void
    Inst_SOP2__S_LSHL_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = (src0.rawData() << bits(src1.rawData(), 4, 0));
        scc = sdst.rawData() ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_LSHL_B64 class methods ---

    Inst_SOP2__S_LSHL_B64::Inst_SOP2__S_LSHL_B64(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_lshl_b64")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_LSHL_B64

    Inst_SOP2__S_LSHL_B64::~Inst_SOP2__S_LSHL_B64()
    {
    } // ~Inst_SOP2__S_LSHL_B64

    // --- description from .arch file ---
    // D.u64 = S0.u64 << S1.u[5:0];
    // SCC = 1 if result is non-zero.
    void
    Inst_SOP2__S_LSHL_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU64 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU64 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = (src0.rawData() << bits(src1.rawData(), 5, 0));
        scc = sdst.rawData() ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_LSHR_B32 class methods ---

    Inst_SOP2__S_LSHR_B32::Inst_SOP2__S_LSHR_B32(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_lshr_b32")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_LSHR_B32

    Inst_SOP2__S_LSHR_B32::~Inst_SOP2__S_LSHR_B32()
    {
    } // ~Inst_SOP2__S_LSHR_B32

    // --- description from .arch file ---
    // D.u = S0.u >> S1.u[4:0];
    // SCC = 1 if result is non-zero.
    // The vacated bits are set to zero.
    void
    Inst_SOP2__S_LSHR_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = (src0.rawData() >> bits(src1.rawData(), 4, 0));
        scc = sdst.rawData() ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_LSHR_B64 class methods ---

    Inst_SOP2__S_LSHR_B64::Inst_SOP2__S_LSHR_B64(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_lshr_b64")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_LSHR_B64

    Inst_SOP2__S_LSHR_B64::~Inst_SOP2__S_LSHR_B64()
    {
    } // ~Inst_SOP2__S_LSHR_B64

    // --- description from .arch file ---
    // D.u64 = S0.u64 >> S1.u[5:0];
    // SCC = 1 if result is non-zero.
    // The vacated bits are set to zero.
    void
    Inst_SOP2__S_LSHR_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU64 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU64 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = (src0.rawData() >> bits(src1.rawData(), 5, 0));
        scc = sdst.rawData() ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_ASHR_I32 class methods ---

    Inst_SOP2__S_ASHR_I32::Inst_SOP2__S_ASHR_I32(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_ashr_i32")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_ASHR_I32

    Inst_SOP2__S_ASHR_I32::~Inst_SOP2__S_ASHR_I32()
    {
    } // ~Inst_SOP2__S_ASHR_I32

    // --- description from .arch file ---
    // D.i = signext(S0.i) >> S1.u[4:0];
    // SCC = 1 if result is non-zero.
    // The vacated bits are set to the sign bit of the input value.
    void
    Inst_SOP2__S_ASHR_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandI32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandI32 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = (src0.rawData() >> bits(src1.rawData(), 4, 0));
        scc = sdst.rawData() ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_ASHR_I64 class methods ---

    Inst_SOP2__S_ASHR_I64::Inst_SOP2__S_ASHR_I64(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_ashr_i64")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_ASHR_I64

    Inst_SOP2__S_ASHR_I64::~Inst_SOP2__S_ASHR_I64()
    {
    } // ~Inst_SOP2__S_ASHR_I64

    // --- description from .arch file ---
    // D.i64 = signext(S0.i64) >> S1.u[5:0];
    // SCC = 1 if result is non-zero.
    // The vacated bits are set to the sign bit of the input value.
    void
    Inst_SOP2__S_ASHR_I64::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandI64 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandI64 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = (src0.rawData() >> bits(src1.rawData(), 5, 0));
        scc = sdst.rawData() ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_BFM_B32 class methods ---

    Inst_SOP2__S_BFM_B32::Inst_SOP2__S_BFM_B32(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_bfm_b32")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_BFM_B32

    Inst_SOP2__S_BFM_B32::~Inst_SOP2__S_BFM_B32()
    {
    } // ~Inst_SOP2__S_BFM_B32

    // --- description from .arch file ---
    // D.u = ((1 << S0.u[4:0]) - 1) << S1.u[4:0] (bitfield mask).
    void
    Inst_SOP2__S_BFM_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 sdst(gpuDynInst, instData.SDST);

        src0.read();
        src1.read();

        sdst = ((1 << bits(src0.rawData(), 4, 0)) - 1)
            << bits(src1.rawData(), 4, 0);

        sdst.write();
    } // execute
    // --- Inst_SOP2__S_BFM_B64 class methods ---

    Inst_SOP2__S_BFM_B64::Inst_SOP2__S_BFM_B64(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_bfm_b64")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_BFM_B64

    Inst_SOP2__S_BFM_B64::~Inst_SOP2__S_BFM_B64()
    {
    } // ~Inst_SOP2__S_BFM_B64

    // --- description from .arch file ---
    // D.u64 = ((1ULL << S0.u[5:0]) - 1) << S1.u[5:0] (bitfield mask).
    void
    Inst_SOP2__S_BFM_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU64 sdst(gpuDynInst, instData.SDST);

        src0.read();
        src1.read();

        sdst = ((1ULL << bits(src0.rawData(), 5, 0)) - 1)
            << bits(src1.rawData(), 5, 0);

        sdst.write();
    } // execute
    // --- Inst_SOP2__S_MUL_I32 class methods ---

    Inst_SOP2__S_MUL_I32::Inst_SOP2__S_MUL_I32(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_mul_i32")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_MUL_I32

    Inst_SOP2__S_MUL_I32::~Inst_SOP2__S_MUL_I32()
    {
    } // ~Inst_SOP2__S_MUL_I32

    // --- description from .arch file ---
    // D.i = S0.i * S1.i.
    void
    Inst_SOP2__S_MUL_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandI32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandI32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandI32 sdst(gpuDynInst, instData.SDST);

        src0.read();
        src1.read();

        ScalarRegI64 tmp = src0.rawData() * src1.rawData();
        sdst = tmp & mask(32);

        sdst.write();
    } // execute
    // --- Inst_SOP2__S_BFE_U32 class methods ---

    Inst_SOP2__S_BFE_U32::Inst_SOP2__S_BFE_U32(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_bfe_u32")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_BFE_U32

    Inst_SOP2__S_BFE_U32::~Inst_SOP2__S_BFE_U32()
    {
    } // ~Inst_SOP2__S_BFE_U32

    // --- description from .arch file ---
    // Bit field extract. S0 is Data, S1[4:0] is field offset, S1[22:16] is
    // field width.
    // D.u = (S0.u>>S1.u[4:0]) & ((1<<S1.u[22:16])-1);
    // SCC = 1 if result is non-zero.
    void
    Inst_SOP2__S_BFE_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = (src0.rawData() >> bits(src1.rawData(), 4, 0))
            & ((1 << bits(src1.rawData(), 22, 16)) - 1);
        scc = sdst.rawData() ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_BFE_I32 class methods ---

    Inst_SOP2__S_BFE_I32::Inst_SOP2__S_BFE_I32(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_bfe_i32")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_BFE_I32

    Inst_SOP2__S_BFE_I32::~Inst_SOP2__S_BFE_I32()
    {
    } // ~Inst_SOP2__S_BFE_I32

    // --- description from .arch file ---
    // Bit field extract. S0 is Data, S1[4:0] is field offset, S1[22:16] is
    // field width.
    // D.i = (S0.i>>S1.u[4:0]) & ((1<<S1.u[22:16])-1);
    // Sign-extend the result;
    // SCC = 1 if result is non-zero.
    void
    Inst_SOP2__S_BFE_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandI32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandI32 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = (src0.rawData() >> bits(src1.rawData(), 4, 0))
            & ((1 << bits(src1.rawData(), 22, 16)) - 1);

        // Above extracted a signed int of size src1[22:16] bits which needs
        // to be signed-extended. Check if the MSB of our src1[22:16]-bit
        // integer is 1, and sign extend it is.
        //
        // Note: The description in the Vega ISA manual does not mention to
        // sign-extend the result. An update description can be found in the
        // more recent RDNA3 manual here:
        // https://developer.amd.com/wp-content/resources/
        //      RDNA3_Shader_ISA_December2022.pdf
        if (sdst.rawData() >> (bits(src1.rawData(), 22, 16) - 1)) {
            sdst = sdst.rawData()
                 | (0xffffffff << bits(src1.rawData(), 22, 16));
        }

        scc = sdst.rawData() ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_BFE_U64 class methods ---

    Inst_SOP2__S_BFE_U64::Inst_SOP2__S_BFE_U64(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_bfe_u64")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_BFE_U64

    Inst_SOP2__S_BFE_U64::~Inst_SOP2__S_BFE_U64()
    {
    } // ~Inst_SOP2__S_BFE_U64

    // --- description from .arch file ---
    // Bit field extract. S0 is Data, S1[5:0] is field offset, S1[22:16] is
    // field width.
    // D.u64 = (S0.u64>>S1.u[5:0]) & ((1<<S1.u[22:16])-1);
    // SCC = 1 if result is non-zero.
    void
    Inst_SOP2__S_BFE_U64::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU64 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU64 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = (src0.rawData() >> bits(src1.rawData(), 5, 0))
            & ((1 << bits(src1.rawData(), 22, 16)) - 1);
        scc = sdst.rawData() ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_BFE_I64 class methods ---

    Inst_SOP2__S_BFE_I64::Inst_SOP2__S_BFE_I64(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_bfe_i64")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_BFE_I64

    Inst_SOP2__S_BFE_I64::~Inst_SOP2__S_BFE_I64()
    {
    } // ~Inst_SOP2__S_BFE_I64

    // --- description from .arch file ---
    // Bit field extract. S0 is Data, S1[5:0] is field offset, S1[22:16] is
    // field width.
    // D.i64 = (S0.i64>>S1.u[5:0]) & ((1<<S1.u[22:16])-1);
    // Sign-extend result;
    // SCC = 1 if result is non-zero.
    void
    Inst_SOP2__S_BFE_I64::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandI64 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandI64 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        src0.read();
        src1.read();

        sdst = (src0.rawData() >> bits(src1.rawData(), 5, 0))
            & ((1 << bits(src1.rawData(), 22, 16)) - 1);

        // Above extracted a signed int of size src1[22:16] bits which needs
        // to be signed-extended. Check if the MSB of our src1[22:16]-bit
        // integer is 1, and sign extend it is.
        if (sdst.rawData() >> (bits(src1.rawData(), 22, 16) - 1)) {
            sdst = sdst.rawData()
                 | 0xffffffffffffffff << bits(src1.rawData(), 22, 16);
        }
        scc = sdst.rawData() ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_CBRANCH_G_FORK class methods ---

    Inst_SOP2__S_CBRANCH_G_FORK::Inst_SOP2__S_CBRANCH_G_FORK(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_cbranch_g_fork")
    {
        setFlag(Branch);
    } // Inst_SOP2__S_CBRANCH_G_FORK

    Inst_SOP2__S_CBRANCH_G_FORK::~Inst_SOP2__S_CBRANCH_G_FORK()
    {
    } // ~Inst_SOP2__S_CBRANCH_G_FORK

    // --- description from .arch file ---
    // mask_pass = S0.u64 & EXEC;
    // mask_fail = ~S0.u64 & EXEC;
    // if (mask_pass == EXEC)
    //     PC = S1.u64;
    // elsif (mask_fail == EXEC)
    //     PC += 4;
    // elsif (bitcount(mask_fail) < bitcount(mask_pass))
    //     EXEC = mask_fail;
    //     SGPR[CSP*4] = { S1.u64, mask_pass };
    //     CSP++;
    //     PC += 4;
    // else
    //     EXEC = mask_pass;
    //     SGPR[CSP*4] = { PC + 4, mask_fail };
    //     CSP++;
    //     PC = S1.u64;
    // end.
    // Conditional branch using branch-stack.
    // S0 = compare mask(vcc or any sgpr) and
    // S1 = 64-bit byte address of target instruction.
    // See also S_CBRANCH_JOIN.
    void
    Inst_SOP2__S_CBRANCH_G_FORK::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_SOP2__S_ABSDIFF_I32 class methods ---

    Inst_SOP2__S_ABSDIFF_I32::Inst_SOP2__S_ABSDIFF_I32(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_absdiff_i32")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_ABSDIFF_I32

    Inst_SOP2__S_ABSDIFF_I32::~Inst_SOP2__S_ABSDIFF_I32()
    {
    } // ~Inst_SOP2__S_ABSDIFF_I32

    // --- description from .arch file ---
    // D.i = S0.i - S1.i;
    // if (D.i < 0) then D.i = -D.i;
    // SCC = 1 if result is non-zero.
    // Compute the absolute value of difference between two values.
    void
    Inst_SOP2__S_ABSDIFF_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandI32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandI32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandI32 sdst(gpuDynInst, instData.SDST);
        ScalarOperandU32 scc(gpuDynInst, REG_SCC);

        sdst = std::abs(src0.rawData() - src1.rawData());
        scc = sdst.rawData() ? 1 : 0;

        sdst.write();
        scc.write();
    } // execute
    // --- Inst_SOP2__S_RFE_RESTORE_B64 class methods ---

    Inst_SOP2__S_RFE_RESTORE_B64::Inst_SOP2__S_RFE_RESTORE_B64(
          InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_rfe_restore_b64")
    {
    } // Inst_SOP2__S_RFE_RESTORE_B64

    Inst_SOP2__S_RFE_RESTORE_B64::~Inst_SOP2__S_RFE_RESTORE_B64()
    {
    } // ~Inst_SOP2__S_RFE_RESTORE_B64

    // --- description from .arch file ---
    // PRIV = 0;
    // PC = S0.u64;
    // INST_ATC = S1.u32[0].
    // Return from exception handler and continue, possibly changing the
    // ---  instruction ATC mode.
    // This instruction may only be used within a trap handler.
    // Use this instruction when the main program may be in a different memory
    // ---  space than the trap handler.
    void
    Inst_SOP2__S_RFE_RESTORE_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_SOP2__S_MUL_HI_U32 class methods ---

    Inst_SOP2__S_MUL_HI_U32::Inst_SOP2__S_MUL_HI_U32(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_mul_hi_u32")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_MUL_HI_U32

    Inst_SOP2__S_MUL_HI_U32::~Inst_SOP2__S_MUL_HI_U32()
    {
    } // ~Inst_SOP2__S_MUL_HI_U32

    // --- description from .arch file ---
    // D.u = (S0.u * S1.u) >> 32;
    void
    Inst_SOP2__S_MUL_HI_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandU32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandU32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandU32 sdst(gpuDynInst, instData.SDST);

        src0.read();
        src1.read();

        VecElemU64 tmp_dst =
            ((VecElemU64)src0.rawData() * (VecElemU64)src1.rawData());
        sdst = (tmp_dst >> 32);

        sdst.write();
    } // execute
    // --- Inst_SOP2__S_MUL_HI_I32 class methods ---

    Inst_SOP2__S_MUL_HI_I32::Inst_SOP2__S_MUL_HI_I32(InFmt_SOP2 *iFmt)
        : Inst_SOP2(iFmt, "s_mul_hi_i32")
    {
        setFlag(ALU);
    } // Inst_SOP2__S_MUL_HI_I32

    Inst_SOP2__S_MUL_HI_I32::~Inst_SOP2__S_MUL_HI_I32()
    {
    } // ~Inst_SOP2__S_MUL_HI_I32

    // --- description from .arch file ---
    // D.u = (S0.u * S1.u) >> 32;
    void
    Inst_SOP2__S_MUL_HI_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        ConstScalarOperandI32 src0(gpuDynInst, instData.SSRC0);
        ConstScalarOperandI32 src1(gpuDynInst, instData.SSRC1);
        ScalarOperandI32 sdst(gpuDynInst, instData.SDST);

        src0.read();
        src1.read();

        VecElemI64 tmp_src0 =
            sext<std::numeric_limits<VecElemI64>::digits>(src0.rawData());
        VecElemI64 tmp_src1 =
            sext<std::numeric_limits<VecElemI64>::digits>(src1.rawData());
        sdst = (VecElemI32)((tmp_src0 * tmp_src1) >> 32);

        sdst.write();
    } // execute
} // namespace VegaISA
} // namespace gem5
