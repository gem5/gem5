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
// --- Inst_SOP1__S_MOV_B32 class methods ---

Inst_SOP1__S_MOV_B32::Inst_SOP1__S_MOV_B32(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_mov_b32")
{
    setFlag(ALU);
} // Inst_SOP1__S_MOV_B32

Inst_SOP1__S_MOV_B32::~Inst_SOP1__S_MOV_B32() {} // ~Inst_SOP1__S_MOV_B32

// --- description from .arch file ---
// D.u = S0.u.
void
Inst_SOP1__S_MOV_B32::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU32 src(gpuDynInst, instData.SSRC0);
    ScalarOperandU32 sdst(gpuDynInst, instData.SDST);

    src.read();

    sdst = src.rawData();

    sdst.write();
} // execute

// --- Inst_SOP1__S_MOV_B64 class methods ---

Inst_SOP1__S_MOV_B64::Inst_SOP1__S_MOV_B64(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_mov_b64")
{
    setFlag(ALU);
} // Inst_SOP1__S_MOV_B64

Inst_SOP1__S_MOV_B64::~Inst_SOP1__S_MOV_B64() {} // ~Inst_SOP1__S_MOV_B64

// --- description from .arch file ---
// D.u64 = S0.u64.
void
Inst_SOP1__S_MOV_B64::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU64 src(gpuDynInst, instData.SSRC0);
    ScalarOperandU64 sdst(gpuDynInst, instData.SDST);

    src.read();

    sdst = src.rawData();

    sdst.write();
} // execute

// --- Inst_SOP1__S_CMOV_B32 class methods ---

Inst_SOP1__S_CMOV_B32::Inst_SOP1__S_CMOV_B32(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_cmov_b32")
{
    setFlag(ALU);
} // Inst_SOP1__S_CMOV_B32

Inst_SOP1__S_CMOV_B32::~Inst_SOP1__S_CMOV_B32() {} // ~Inst_SOP1__S_CMOV_B32

// --- description from .arch file ---
// (SCC) then D.u = S0.u;
// else NOP.
// Conditional move.
void
Inst_SOP1__S_CMOV_B32::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU32 src(gpuDynInst, instData.SSRC0);
    ScalarOperandU32 sdst(gpuDynInst, instData.SDST);
    ScalarOperandU32 scc(gpuDynInst, REG_SCC);

    src.read();
    scc.read();

    if (scc.rawData()) {
        sdst = src.rawData();
        sdst.write();
    }
} // execute

// --- Inst_SOP1__S_CMOV_B64 class methods ---

Inst_SOP1__S_CMOV_B64::Inst_SOP1__S_CMOV_B64(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_cmov_b64")
{
    setFlag(ALU);
} // Inst_SOP1__S_CMOV_B64

Inst_SOP1__S_CMOV_B64::~Inst_SOP1__S_CMOV_B64() {} // ~Inst_SOP1__S_CMOV_B64

// --- description from .arch file ---
// if (SCC) then D.u64 = S0.u64;
// else NOP.
// Conditional move.
void
Inst_SOP1__S_CMOV_B64::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU64 src(gpuDynInst, instData.SSRC0);
    ScalarOperandU64 sdst(gpuDynInst, instData.SDST);
    ScalarOperandU32 scc(gpuDynInst, REG_SCC);

    src.read();
    scc.read();

    if (scc.rawData()) {
        sdst = src.rawData();
        sdst.write();
    }
} // execute

// --- Inst_SOP1__S_NOT_B32 class methods ---

Inst_SOP1__S_NOT_B32::Inst_SOP1__S_NOT_B32(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_not_b32")
{
    setFlag(ALU);
} // Inst_SOP1__S_NOT_B32

Inst_SOP1__S_NOT_B32::~Inst_SOP1__S_NOT_B32() {} // ~Inst_SOP1__S_NOT_B32

// --- description from .arch file ---
// D.u = ~S0.u;
// SCC = 1 if result is non-zero.
// Bitwise negation.
void
Inst_SOP1__S_NOT_B32::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU32 src(gpuDynInst, instData.SSRC0);
    ScalarOperandU32 sdst(gpuDynInst, instData.SDST);
    ScalarOperandU32 scc(gpuDynInst, REG_SCC);

    src.read();

    sdst = ~src.rawData();

    scc = sdst.rawData() ? 1 : 0;

    sdst.write();
    scc.write();
} // execute

// --- Inst_SOP1__S_NOT_B64 class methods ---

Inst_SOP1__S_NOT_B64::Inst_SOP1__S_NOT_B64(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_not_b64")
{
    setFlag(ALU);
} // Inst_SOP1__S_NOT_B64

Inst_SOP1__S_NOT_B64::~Inst_SOP1__S_NOT_B64() {} // ~Inst_SOP1__S_NOT_B64

// --- description from .arch file ---
// D.u64 = ~S0.u64;
// SCC = 1 if result is non-zero.
// Bitwise negation.
void
Inst_SOP1__S_NOT_B64::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU64 src(gpuDynInst, instData.SSRC0);
    ScalarOperandU64 sdst(gpuDynInst, instData.SDST);
    ScalarOperandU32 scc(gpuDynInst, REG_SCC);

    src.read();

    sdst = ~src.rawData();
    scc = sdst.rawData() ? 1 : 0;

    sdst.write();
    scc.write();
} // execute

// --- Inst_SOP1__S_WQM_B32 class methods ---

Inst_SOP1__S_WQM_B32::Inst_SOP1__S_WQM_B32(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_wqm_b32")
{
    setFlag(ALU);
} // Inst_SOP1__S_WQM_B32

Inst_SOP1__S_WQM_B32::~Inst_SOP1__S_WQM_B32() {} // ~Inst_SOP1__S_WQM_B32

// --- description from .arch file ---
// D[i] = (S0[(i & ~3):(i | 3)] != 0);
// Computes whole quad mode for an active/valid mask.
// SCC = 1 if result is non-zero.
void
Inst_SOP1__S_WQM_B32::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU32 src(gpuDynInst, instData.SSRC0);
    ScalarOperandU32 sdst(gpuDynInst, instData.SDST);
    ScalarOperandU32 scc(gpuDynInst, REG_SCC);

    src.read();

    sdst = wholeQuadMode(src.rawData());
    scc = sdst.rawData() ? 1 : 0;

    sdst.write();
    scc.write();
} // execute

// --- Inst_SOP1__S_WQM_B64 class methods ---

Inst_SOP1__S_WQM_B64::Inst_SOP1__S_WQM_B64(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_wqm_b64")
{
    setFlag(ALU);
} // Inst_SOP1__S_WQM_B64

Inst_SOP1__S_WQM_B64::~Inst_SOP1__S_WQM_B64() {} // ~Inst_SOP1__S_WQM_B64

// --- description from .arch file ---
// D[i] = (S0[(i & ~3):(i | 3)] != 0);
// Computes whole quad mode for an active/valid mask.
// SCC = 1 if result is non-zero.
void
Inst_SOP1__S_WQM_B64::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU64 src(gpuDynInst, instData.SSRC0);
    ScalarOperandU64 sdst(gpuDynInst, instData.SDST);
    ScalarOperandU32 scc(gpuDynInst, REG_SCC);

    src.read();

    sdst = wholeQuadMode(src.rawData());
    scc = sdst.rawData() ? 1 : 0;

    sdst.write();
    scc.write();
} // execute

// --- Inst_SOP1__S_BREV_B32 class methods ---

Inst_SOP1__S_BREV_B32::Inst_SOP1__S_BREV_B32(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_brev_b32")
{
    setFlag(ALU);
} // Inst_SOP1__S_BREV_B32

Inst_SOP1__S_BREV_B32::~Inst_SOP1__S_BREV_B32() {} // ~Inst_SOP1__S_BREV_B32

// --- description from .arch file ---
// D.u[31:0] = S0.u[0:31] (reverse bits).
void
Inst_SOP1__S_BREV_B32::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU32 src(gpuDynInst, instData.SSRC0);
    ScalarOperandU32 sdst(gpuDynInst, instData.SDST);

    src.read();

    sdst = reverseBits(src.rawData());

    sdst.write();
} // execute

// --- Inst_SOP1__S_BREV_B64 class methods ---

Inst_SOP1__S_BREV_B64::Inst_SOP1__S_BREV_B64(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_brev_b64")
{
    setFlag(ALU);
} // Inst_SOP1__S_BREV_B64

Inst_SOP1__S_BREV_B64::~Inst_SOP1__S_BREV_B64() {} // ~Inst_SOP1__S_BREV_B64

// --- description from .arch file ---
// D.u64[63:0] = S0.u64[0:63] (reverse bits).
void
Inst_SOP1__S_BREV_B64::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU64 src(gpuDynInst, instData.SSRC0);
    ScalarOperandU64 sdst(gpuDynInst, instData.SDST);

    src.read();

    sdst = reverseBits(src.rawData());

    sdst.write();
} // execute

// --- Inst_SOP1__S_BCNT0_I32_B32 class methods ---

Inst_SOP1__S_BCNT0_I32_B32::Inst_SOP1__S_BCNT0_I32_B32(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_bcnt0_i32_b32")
{
    setFlag(ALU);
} // Inst_SOP1__S_BCNT0_I32_B32

Inst_SOP1__S_BCNT0_I32_B32::~Inst_SOP1__S_BCNT0_I32_B32() {
} // ~Inst_SOP1__S_BCNT0_I32_B32

// --- description from .arch file ---
// D.i = CountZeroBits(S0.u);
// SCC = 1 if result is non-zero.
void
Inst_SOP1__S_BCNT0_I32_B32::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU32 src(gpuDynInst, instData.SSRC0);
    ScalarOperandI32 sdst(gpuDynInst, instData.SDST);
    ScalarOperandU32 scc(gpuDynInst, REG_SCC);

    src.read();

    sdst = countZeroBits(src.rawData());
    scc = sdst.rawData() ? 1 : 0;

    sdst.write();
    scc.write();
} // execute

// --- Inst_SOP1__S_BCNT0_I32_B64 class methods ---

Inst_SOP1__S_BCNT0_I32_B64::Inst_SOP1__S_BCNT0_I32_B64(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_bcnt0_i32_b64")
{
    setFlag(ALU);
} // Inst_SOP1__S_BCNT0_I32_B64

Inst_SOP1__S_BCNT0_I32_B64::~Inst_SOP1__S_BCNT0_I32_B64() {
} // ~Inst_SOP1__S_BCNT0_I32_B64

// --- description from .arch file ---
// D.i = CountZeroBits(S0.u64);
// SCC = 1 if result is non-zero.
void
Inst_SOP1__S_BCNT0_I32_B64::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU64 src(gpuDynInst, instData.SSRC0);
    ScalarOperandI32 sdst(gpuDynInst, instData.SDST);
    ScalarOperandU32 scc(gpuDynInst, REG_SCC);

    src.read();

    sdst = countZeroBits(src.rawData());
    scc = sdst.rawData() ? 1 : 0;

    sdst.write();
    scc.write();
} // execute

// --- Inst_SOP1__S_BCNT1_I32_B32 class methods ---

Inst_SOP1__S_BCNT1_I32_B32::Inst_SOP1__S_BCNT1_I32_B32(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_bcnt1_i32_b32")
{
    setFlag(ALU);
} // Inst_SOP1__S_BCNT1_I32_B32

Inst_SOP1__S_BCNT1_I32_B32::~Inst_SOP1__S_BCNT1_I32_B32() {
} // ~Inst_SOP1__S_BCNT1_I32_B32

// --- description from .arch file ---
// D.i = CountOneBits(S0.u);
// SCC = 1 if result is non-zero.
void
Inst_SOP1__S_BCNT1_I32_B32::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU32 src(gpuDynInst, instData.SSRC0);
    ScalarOperandI32 sdst(gpuDynInst, instData.SDST);
    ScalarOperandU32 scc(gpuDynInst, REG_SCC);

    src.read();

    sdst = popCount(src.rawData());
    scc = sdst.rawData() ? 1 : 0;

    sdst.write();
    scc.write();
} // execute

// --- Inst_SOP1__S_BCNT1_I32_B64 class methods ---

Inst_SOP1__S_BCNT1_I32_B64::Inst_SOP1__S_BCNT1_I32_B64(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_bcnt1_i32_b64")
{
    setFlag(ALU);
} // Inst_SOP1__S_BCNT1_I32_B64

Inst_SOP1__S_BCNT1_I32_B64::~Inst_SOP1__S_BCNT1_I32_B64() {
} // ~Inst_SOP1__S_BCNT1_I32_B64

// --- description from .arch file ---
// D.i = CountOneBits(S0.u64);
// SCC = 1 if result is non-zero.
void
Inst_SOP1__S_BCNT1_I32_B64::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU64 src(gpuDynInst, instData.SSRC0);
    ScalarOperandI32 sdst(gpuDynInst, instData.SDST);
    ScalarOperandU32 scc(gpuDynInst, REG_SCC);

    src.read();

    sdst = popCount(src.rawData());
    scc = sdst.rawData() ? 1 : 0;

    sdst.write();
    scc.write();
} // execute

// --- Inst_SOP1__S_FF0_I32_B32 class methods ---

Inst_SOP1__S_FF0_I32_B32::Inst_SOP1__S_FF0_I32_B32(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_ff0_i32_b32")
{
    setFlag(ALU);
} // Inst_SOP1__S_FF0_I32_B32

Inst_SOP1__S_FF0_I32_B32::~Inst_SOP1__S_FF0_I32_B32() {
} // ~Inst_SOP1__S_FF0_I32_B32

// --- description from .arch file ---
// D.i = FindFirstZero(S0.u);
// If no zeros are found, return -1.
// Returns the bit position of the first zero from the LSB.
void
Inst_SOP1__S_FF0_I32_B32::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU32 src(gpuDynInst, instData.SSRC0);
    ScalarOperandI32 sdst(gpuDynInst, instData.SDST);

    src.read();

    sdst = findFirstZero(src.rawData());

    sdst.write();
} // execute

// --- Inst_SOP1__S_FF0_I32_B64 class methods ---

Inst_SOP1__S_FF0_I32_B64::Inst_SOP1__S_FF0_I32_B64(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_ff0_i32_b64")
{
    setFlag(ALU);
} // Inst_SOP1__S_FF0_I32_B64

Inst_SOP1__S_FF0_I32_B64::~Inst_SOP1__S_FF0_I32_B64() {
} // ~Inst_SOP1__S_FF0_I32_B64

// --- description from .arch file ---
// D.i = FindFirstZero(S0.u64);
// If no zeros are found, return -1.
// Returns the bit position of the first zero from the LSB.
void
Inst_SOP1__S_FF0_I32_B64::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU64 src(gpuDynInst, instData.SSRC0);
    ScalarOperandI32 sdst(gpuDynInst, instData.SDST);

    src.read();

    sdst = findFirstZero(src.rawData());

    sdst.write();
} // execute

// --- Inst_SOP1__S_FF1_I32_B32 class methods ---

Inst_SOP1__S_FF1_I32_B32::Inst_SOP1__S_FF1_I32_B32(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_ff1_i32_b32")
{
    setFlag(ALU);
} // Inst_SOP1__S_FF1_I32_B32

Inst_SOP1__S_FF1_I32_B32::~Inst_SOP1__S_FF1_I32_B32() {
} // ~Inst_SOP1__S_FF1_I32_B32

// --- description from .arch file ---
// D.i = FindFirstOne(S0.u);
// If no ones are found, return -1.
// Returns the bit position of the first one from the LSB.
void
Inst_SOP1__S_FF1_I32_B32::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU32 src(gpuDynInst, instData.SSRC0);
    ScalarOperandI32 sdst(gpuDynInst, instData.SDST);

    src.read();

    sdst = findFirstOne(src.rawData());

    sdst.write();
} // execute

// --- Inst_SOP1__S_FF1_I32_B64 class methods ---

Inst_SOP1__S_FF1_I32_B64::Inst_SOP1__S_FF1_I32_B64(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_ff1_i32_b64")
{
    setFlag(ALU);
} // Inst_SOP1__S_FF1_I32_B64

Inst_SOP1__S_FF1_I32_B64::~Inst_SOP1__S_FF1_I32_B64() {
} // ~Inst_SOP1__S_FF1_I32_B64

// --- description from .arch file ---
// D.i = FindFirstOne(S0.u64);
// If no ones are found, return -1.
// Returns the bit position of the first one from the LSB.
void
Inst_SOP1__S_FF1_I32_B64::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU64 src(gpuDynInst, instData.SSRC0);
    ScalarOperandI32 sdst(gpuDynInst, instData.SDST);

    src.read();

    sdst = findFirstOne(src.rawData());

    sdst.write();
} // execute

// --- Inst_SOP1__S_FLBIT_I32_B32 class methods ---

Inst_SOP1__S_FLBIT_I32_B32::Inst_SOP1__S_FLBIT_I32_B32(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_flbit_i32_b32")
{
    setFlag(ALU);
} // Inst_SOP1__S_FLBIT_I32_B32

Inst_SOP1__S_FLBIT_I32_B32::~Inst_SOP1__S_FLBIT_I32_B32() {
} // ~Inst_SOP1__S_FLBIT_I32_B32

// --- description from .arch file ---
// D.i = FindFirstOne(S0.u);
// If no ones are found, return -1.
// Counts how many zeros before the first one starting from the MSB.
void
Inst_SOP1__S_FLBIT_I32_B32::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU32 src(gpuDynInst, instData.SSRC0);
    ScalarOperandI32 sdst(gpuDynInst, instData.SDST);

    src.read();

    sdst = countZeroBitsMsb(src.rawData());

    sdst.write();
} // execute

// --- Inst_SOP1__S_FLBIT_I32_B64 class methods ---

Inst_SOP1__S_FLBIT_I32_B64::Inst_SOP1__S_FLBIT_I32_B64(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_flbit_i32_b64")
{
    setFlag(ALU);
} // Inst_SOP1__S_FLBIT_I32_B64

Inst_SOP1__S_FLBIT_I32_B64::~Inst_SOP1__S_FLBIT_I32_B64() {
} // ~Inst_SOP1__S_FLBIT_I32_B64

// --- description from .arch file ---
// D.i = FindFirstOne(S0.u64);
// If no ones are found, return -1.
// Counts how many zeros before the first one starting from the MSB.
void
Inst_SOP1__S_FLBIT_I32_B64::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU64 src(gpuDynInst, instData.SSRC0);
    ScalarOperandI32 sdst(gpuDynInst, instData.SDST);

    src.read();

    sdst = countZeroBitsMsb(src.rawData());

    sdst.write();
} // execute

// --- Inst_SOP1__S_FLBIT_I32 class methods ---

Inst_SOP1__S_FLBIT_I32::Inst_SOP1__S_FLBIT_I32(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_flbit_i32")
{
    setFlag(ALU);
} // Inst_SOP1__S_FLBIT_I32

Inst_SOP1__S_FLBIT_I32::~Inst_SOP1__S_FLBIT_I32() {} // ~Inst_SOP1__S_FLBIT_I32

// --- description from .arch file ---
// D.i = FirstOppositeSignBit(S0.i);
// If S0.i == 0 or S0.i == -1 (all bits are the same), return -1.
// Counts how many bits in a row (from MSB to LSB) are the same as the
// sign bit.
void
Inst_SOP1__S_FLBIT_I32::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandI32 src(gpuDynInst, instData.SSRC0);
    ScalarOperandI32 sdst(gpuDynInst, instData.SDST);

    src.read();

    sdst = firstOppositeSignBit(src.rawData());

    sdst.write();
} // execute

// --- Inst_SOP1__S_FLBIT_I32_I64 class methods ---

Inst_SOP1__S_FLBIT_I32_I64::Inst_SOP1__S_FLBIT_I32_I64(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_flbit_i32_i64")
{
    setFlag(ALU);
} // Inst_SOP1__S_FLBIT_I32_I64

Inst_SOP1__S_FLBIT_I32_I64::~Inst_SOP1__S_FLBIT_I32_I64() {
} // ~Inst_SOP1__S_FLBIT_I32_I64

// --- description from .arch file ---
// D.i = FirstOppositeSignBit(S0.i64);
// If S0.i == 0 or S0.i == -1 (all bits are the same), return -1.
// Counts how many bits in a row (from MSB to LSB) are the same as the
// sign bit.
void
Inst_SOP1__S_FLBIT_I32_I64::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandI64 src(gpuDynInst, instData.SSRC0);
    ScalarOperandI32 sdst(gpuDynInst, instData.SDST);

    src.read();

    sdst = firstOppositeSignBit(src.rawData());

    sdst.write();
} // execute

// --- Inst_SOP1__S_SEXT_I32_I8 class methods ---

Inst_SOP1__S_SEXT_I32_I8::Inst_SOP1__S_SEXT_I32_I8(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_sext_i32_i8")
{
    setFlag(ALU);
} // Inst_SOP1__S_SEXT_I32_I8

Inst_SOP1__S_SEXT_I32_I8::~Inst_SOP1__S_SEXT_I32_I8() {
} // ~Inst_SOP1__S_SEXT_I32_I8

// --- description from .arch file ---
// D.i = signext(S0.i[7:0]) (sign extension).
void
Inst_SOP1__S_SEXT_I32_I8::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandI32 src(gpuDynInst, instData.SSRC0);
    ScalarOperandI32 sdst(gpuDynInst, instData.SDST);

    src.read();

    sdst = sext<std::numeric_limits<ScalarRegI8>::digits>(
        bits(src.rawData(), 7, 0));

    sdst.write();
} // execute

// --- Inst_SOP1__S_SEXT_I32_I16 class methods ---

Inst_SOP1__S_SEXT_I32_I16::Inst_SOP1__S_SEXT_I32_I16(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_sext_i32_i16")
{
    setFlag(ALU);
} // Inst_SOP1__S_SEXT_I32_I16

Inst_SOP1__S_SEXT_I32_I16::~Inst_SOP1__S_SEXT_I32_I16() {
} // ~Inst_SOP1__S_SEXT_I32_I16

// --- description from .arch file ---
// D.i = signext(S0.i[15:0]) (sign extension).
void
Inst_SOP1__S_SEXT_I32_I16::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandI32 src(gpuDynInst, instData.SSRC0);
    ScalarOperandI32 sdst(gpuDynInst, instData.SDST);

    src.read();

    sdst = sext<std::numeric_limits<ScalarRegI16>::digits>(
        bits(src.rawData(), 15, 0));

    sdst.write();
} // execute

// --- Inst_SOP1__S_BITSET0_B32 class methods ---

Inst_SOP1__S_BITSET0_B32::Inst_SOP1__S_BITSET0_B32(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_bitset0_b32")
{
    setFlag(ALU);
} // Inst_SOP1__S_BITSET0_B32

Inst_SOP1__S_BITSET0_B32::~Inst_SOP1__S_BITSET0_B32() {
} // ~Inst_SOP1__S_BITSET0_B32

// --- description from .arch file ---
// D.u[S0.u[4:0]] = 0.
void
Inst_SOP1__S_BITSET0_B32::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU32 src(gpuDynInst, instData.SSRC0);
    ScalarOperandU32 sdst(gpuDynInst, instData.SDST);

    src.read();

    sdst.setBit(bits(src.rawData(), 4, 0), 0);

    sdst.write();
} // execute

// --- Inst_SOP1__S_BITSET0_B64 class methods ---

Inst_SOP1__S_BITSET0_B64::Inst_SOP1__S_BITSET0_B64(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_bitset0_b64")
{
    setFlag(ALU);
} // Inst_SOP1__S_BITSET0_B64

Inst_SOP1__S_BITSET0_B64::~Inst_SOP1__S_BITSET0_B64() {
} // ~Inst_SOP1__S_BITSET0_B64

// --- description from .arch file ---
// D.u64[S0.u[5:0]] = 0.
void
Inst_SOP1__S_BITSET0_B64::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU32 src(gpuDynInst, instData.SSRC0);
    ScalarOperandU64 sdst(gpuDynInst, instData.SDST);

    src.read();

    sdst.setBit(bits(src.rawData(), 5, 0), 0);

    sdst.write();
} // execute

// --- Inst_SOP1__S_BITSET1_B32 class methods ---

Inst_SOP1__S_BITSET1_B32::Inst_SOP1__S_BITSET1_B32(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_bitset1_b32")
{
    setFlag(ALU);
} // Inst_SOP1__S_BITSET1_B32

Inst_SOP1__S_BITSET1_B32::~Inst_SOP1__S_BITSET1_B32() {
} // ~Inst_SOP1__S_BITSET1_B32

// --- description from .arch file ---
// D.u[S0.u[4:0]] = 1.
void
Inst_SOP1__S_BITSET1_B32::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU32 src(gpuDynInst, instData.SSRC0);
    ScalarOperandU32 sdst(gpuDynInst, instData.SDST);

    src.read();

    sdst.setBit(bits(src.rawData(), 4, 0), 1);

    sdst.write();
} // execute

// --- Inst_SOP1__S_BITSET1_B64 class methods ---

Inst_SOP1__S_BITSET1_B64::Inst_SOP1__S_BITSET1_B64(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_bitset1_b64")
{
    setFlag(ALU);
} // Inst_SOP1__S_BITSET1_B64

Inst_SOP1__S_BITSET1_B64::~Inst_SOP1__S_BITSET1_B64() {
} // ~Inst_SOP1__S_BITSET1_B64

// --- description from .arch file ---
// D.u64[S0.u[5:0]] = 1.
void
Inst_SOP1__S_BITSET1_B64::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU32 src(gpuDynInst, instData.SSRC0);
    ScalarOperandU64 sdst(gpuDynInst, instData.SDST);

    src.read();

    sdst.setBit(bits(src.rawData(), 5, 0), 1);

    sdst.write();
} // execute

// --- Inst_SOP1__S_GETPC_B64 class methods ---

Inst_SOP1__S_GETPC_B64::Inst_SOP1__S_GETPC_B64(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_getpc_b64")
{
    setFlag(ALU);
} // Inst_SOP1__S_GETPC_B64

Inst_SOP1__S_GETPC_B64::~Inst_SOP1__S_GETPC_B64() {} // ~Inst_SOP1__S_GETPC_B64

// --- description from .arch file ---
// D.u64 = PC + 4.
// Destination receives the byte address of the next instruction.
void
Inst_SOP1__S_GETPC_B64::execute(GPUDynInstPtr gpuDynInst)
{
    Addr pc = gpuDynInst->pc();
    ScalarOperandU64 sdst(gpuDynInst, instData.SDST);

    sdst = pc + 4;

    sdst.write();
} // execute

// --- Inst_SOP1__S_SETPC_B64 class methods ---

Inst_SOP1__S_SETPC_B64::Inst_SOP1__S_SETPC_B64(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_setpc_b64")
{
    setFlag(ALU);
} // Inst_SOP1__S_SETPC_B64

Inst_SOP1__S_SETPC_B64::~Inst_SOP1__S_SETPC_B64() {} // ~Inst_SOP1__S_SETPC_B64

// --- description from .arch file ---
// PC = S0.u64.
// S0.u64 is a byte address of the instruction to jump to.
void
Inst_SOP1__S_SETPC_B64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstScalarOperandU64 src(gpuDynInst, instData.SSRC0);

    src.read();

    wf->pc(src.rawData());
} // execute

// --- Inst_SOP1__S_SWAPPC_B64 class methods ---

Inst_SOP1__S_SWAPPC_B64::Inst_SOP1__S_SWAPPC_B64(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_swappc_b64")
{
    setFlag(ALU);
} // Inst_SOP1__S_SWAPPC_B64

Inst_SOP1__S_SWAPPC_B64::~Inst_SOP1__S_SWAPPC_B64() {
} // ~Inst_SOP1__S_SWAPPC_B64

// --- description from .arch file ---
// D.u64 = PC + 4; PC = S0.u64.
// S0.u64 is a byte address of the instruction to jump to.
void
Inst_SOP1__S_SWAPPC_B64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    Addr pc = gpuDynInst->pc();
    ConstScalarOperandU64 src(gpuDynInst, instData.SSRC0);
    ScalarOperandU64 sdst(gpuDynInst, instData.SDST);

    src.read();

    sdst = pc + 4;

    wf->pc(src.rawData());
    sdst.write();
} // execute

// --- Inst_SOP1__S_RFE_B64 class methods ---

Inst_SOP1__S_RFE_B64::Inst_SOP1__S_RFE_B64(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_rfe_b64")
{} // Inst_SOP1__S_RFE_B64

Inst_SOP1__S_RFE_B64::~Inst_SOP1__S_RFE_B64() {} // ~Inst_SOP1__S_RFE_B64

// --- description from .arch file ---
// PRIV = 0;
// PC = S0.u64.
// Return from exception handler and continue.
// This instruction may only be used within a trap handler.
void
Inst_SOP1__S_RFE_B64::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_SOP1__S_AND_SAVEEXEC_B64 class methods ---

Inst_SOP1__S_AND_SAVEEXEC_B64::Inst_SOP1__S_AND_SAVEEXEC_B64(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_and_saveexec_b64")
{
    setFlag(ALU);
    setFlag(ReadsEXEC);
    setFlag(WritesEXEC);
} // Inst_SOP1__S_AND_SAVEEXEC_B64

Inst_SOP1__S_AND_SAVEEXEC_B64::~Inst_SOP1__S_AND_SAVEEXEC_B64() {
} // ~Inst_SOP1__S_AND_SAVEEXEC_B64

// --- description from .arch file ---
// D.u64 = EXEC;
// EXEC = S0.u64 & EXEC;
// SCC = 1 if the new value of EXEC is non-zero.
void
Inst_SOP1__S_AND_SAVEEXEC_B64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstScalarOperandU64 src(gpuDynInst, instData.SSRC0);
    ScalarOperandU64 sdst(gpuDynInst, instData.SDST);
    ScalarOperandU32 scc(gpuDynInst, REG_SCC);

    src.read();

    sdst = wf->execMask().to_ullong();
    wf->execMask() = src.rawData() & wf->execMask().to_ullong();
    scc = wf->execMask().any() ? 1 : 0;

    sdst.write();
    scc.write();
} // execute

// --- Inst_SOP1__S_OR_SAVEEXEC_B64 class methods ---

Inst_SOP1__S_OR_SAVEEXEC_B64::Inst_SOP1__S_OR_SAVEEXEC_B64(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_or_saveexec_b64")
{
    setFlag(ALU);
    setFlag(ReadsEXEC);
    setFlag(WritesEXEC);
} // Inst_SOP1__S_OR_SAVEEXEC_B64

Inst_SOP1__S_OR_SAVEEXEC_B64::~Inst_SOP1__S_OR_SAVEEXEC_B64() {
} // ~Inst_SOP1__S_OR_SAVEEXEC_B64

// --- description from .arch file ---
// D.u64 = EXEC;
// EXEC = S0.u64 | EXEC;
// SCC = 1 if the new value of EXEC is non-zero.
void
Inst_SOP1__S_OR_SAVEEXEC_B64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstScalarOperandU64 src(gpuDynInst, instData.SSRC0);
    ScalarOperandU64 sdst(gpuDynInst, instData.SDST);
    ScalarOperandU32 scc(gpuDynInst, REG_SCC);

    src.read();

    sdst = wf->execMask().to_ullong();
    wf->execMask() = src.rawData() | wf->execMask().to_ullong();
    scc = wf->execMask().any() ? 1 : 0;

    sdst.write();
    scc.write();
} // execute

// --- Inst_SOP1__S_XOR_SAVEEXEC_B64 class methods ---

Inst_SOP1__S_XOR_SAVEEXEC_B64::Inst_SOP1__S_XOR_SAVEEXEC_B64(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_xor_saveexec_b64")
{
    setFlag(ALU);
    setFlag(ReadsEXEC);
    setFlag(WritesEXEC);
} // Inst_SOP1__S_XOR_SAVEEXEC_B64

Inst_SOP1__S_XOR_SAVEEXEC_B64::~Inst_SOP1__S_XOR_SAVEEXEC_B64() {
} // ~Inst_SOP1__S_XOR_SAVEEXEC_B64

// --- description from .arch file ---
// D.u64 = EXEC;
// EXEC = S0.u64 ^ EXEC;
// SCC = 1 if the new value of EXEC is non-zero.
void
Inst_SOP1__S_XOR_SAVEEXEC_B64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstScalarOperandU64 src(gpuDynInst, instData.SSRC0);
    ScalarOperandU64 sdst(gpuDynInst, instData.SDST);
    ScalarOperandU32 scc(gpuDynInst, REG_SCC);

    src.read();

    sdst = wf->execMask().to_ullong();
    wf->execMask() = src.rawData() ^ wf->execMask().to_ullong();
    scc = wf->execMask().any() ? 1 : 0;

    sdst.write();
    scc.write();
} // execute

// --- Inst_SOP1__S_ANDN2_SAVEEXEC_B64 class methods ---

Inst_SOP1__S_ANDN2_SAVEEXEC_B64::Inst_SOP1__S_ANDN2_SAVEEXEC_B64(
    InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_andn2_saveexec_b64")
{
    setFlag(ALU);
    setFlag(ReadsEXEC);
    setFlag(WritesEXEC);
} // Inst_SOP1__S_ANDN2_SAVEEXEC_B64

Inst_SOP1__S_ANDN2_SAVEEXEC_B64::~Inst_SOP1__S_ANDN2_SAVEEXEC_B64() {
} // ~Inst_SOP1__S_ANDN2_SAVEEXEC_B64

// --- description from .arch file ---
// D.u64 = EXEC;
// EXEC = S0.u64 & ~EXEC;
// SCC = 1 if the new value of EXEC is non-zero.
void
Inst_SOP1__S_ANDN2_SAVEEXEC_B64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstScalarOperandU64 src(gpuDynInst, instData.SSRC0);
    ScalarOperandU64 sdst(gpuDynInst, instData.SDST);
    ScalarOperandU32 scc(gpuDynInst, REG_SCC);

    src.read();

    sdst = wf->execMask().to_ullong();
    wf->execMask() = src.rawData() & ~wf->execMask().to_ullong();
    scc = wf->execMask().any() ? 1 : 0;

    sdst.write();
    scc.write();
} // execute

// --- Inst_SOP1__S_ORN2_SAVEEXEC_B64 class methods ---

Inst_SOP1__S_ORN2_SAVEEXEC_B64::Inst_SOP1__S_ORN2_SAVEEXEC_B64(
    InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_orn2_saveexec_b64")
{
    setFlag(ALU);
    setFlag(ReadsEXEC);
    setFlag(WritesEXEC);
} // Inst_SOP1__S_ORN2_SAVEEXEC_B64

Inst_SOP1__S_ORN2_SAVEEXEC_B64::~Inst_SOP1__S_ORN2_SAVEEXEC_B64() {
} // ~Inst_SOP1__S_ORN2_SAVEEXEC_B64

// --- description from .arch file ---
// D.u64 = EXEC;
// EXEC = S0.u64 | ~EXEC;
// SCC = 1 if the new value of EXEC is non-zero.
void
Inst_SOP1__S_ORN2_SAVEEXEC_B64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstScalarOperandU64 src(gpuDynInst, instData.SSRC0);
    ScalarOperandU64 sdst(gpuDynInst, instData.SDST);
    ScalarOperandU32 scc(gpuDynInst, REG_SCC);

    src.read();

    sdst = wf->execMask().to_ullong();
    wf->execMask() = src.rawData() | ~wf->execMask().to_ullong();
    scc = wf->execMask().any() ? 1 : 0;

    sdst.write();
    scc.write();
} // execute

// --- Inst_SOP1__S_NAND_SAVEEXEC_B64 class methods ---

Inst_SOP1__S_NAND_SAVEEXEC_B64::Inst_SOP1__S_NAND_SAVEEXEC_B64(
    InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_nand_saveexec_b64")
{
    setFlag(ALU);
    setFlag(ReadsEXEC);
    setFlag(WritesEXEC);
} // Inst_SOP1__S_NAND_SAVEEXEC_B64

Inst_SOP1__S_NAND_SAVEEXEC_B64::~Inst_SOP1__S_NAND_SAVEEXEC_B64() {
} // ~Inst_SOP1__S_NAND_SAVEEXEC_B64

// --- description from .arch file ---
// D.u64 = EXEC;
// EXEC = ~(S0.u64 & EXEC);
// SCC = 1 if the new value of EXEC is non-zero.
void
Inst_SOP1__S_NAND_SAVEEXEC_B64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstScalarOperandU64 src(gpuDynInst, instData.SSRC0);
    ScalarOperandU64 sdst(gpuDynInst, instData.SDST);
    ScalarOperandU32 scc(gpuDynInst, REG_SCC);

    src.read();

    sdst = wf->execMask().to_ullong();
    wf->execMask() = ~(src.rawData() & wf->execMask().to_ullong());
    scc = wf->execMask().any() ? 1 : 0;

    sdst.write();
    scc.write();
} // execute

// --- Inst_SOP1__S_NOR_SAVEEXEC_B64 class methods ---

Inst_SOP1__S_NOR_SAVEEXEC_B64::Inst_SOP1__S_NOR_SAVEEXEC_B64(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_nor_saveexec_b64")
{
    setFlag(ALU);
    setFlag(ReadsEXEC);
    setFlag(WritesEXEC);
} // Inst_SOP1__S_NOR_SAVEEXEC_B64

Inst_SOP1__S_NOR_SAVEEXEC_B64::~Inst_SOP1__S_NOR_SAVEEXEC_B64() {
} // ~Inst_SOP1__S_NOR_SAVEEXEC_B64

// --- description from .arch file ---
// D.u64 = EXEC;
// EXEC = ~(S0.u64 | EXEC);
// SCC = 1 if the new value of EXEC is non-zero.
void
Inst_SOP1__S_NOR_SAVEEXEC_B64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstScalarOperandU64 src(gpuDynInst, instData.SSRC0);
    ScalarOperandU64 sdst(gpuDynInst, instData.SDST);
    ScalarOperandU32 scc(gpuDynInst, REG_SCC);

    src.read();

    sdst = wf->execMask().to_ullong();
    wf->execMask() = ~(src.rawData() | wf->execMask().to_ullong());
    scc = wf->execMask().any() ? 1 : 0;

    sdst.write();
    scc.write();
} // execute

// --- Inst_SOP1__S_XNOR_SAVEEXEC_B64 class methods ---

Inst_SOP1__S_XNOR_SAVEEXEC_B64::Inst_SOP1__S_XNOR_SAVEEXEC_B64(
    InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_xnor_saveexec_b64")
{
    setFlag(ALU);
    setFlag(ReadsEXEC);
    setFlag(WritesEXEC);
} // Inst_SOP1__S_XNOR_SAVEEXEC_B64

Inst_SOP1__S_XNOR_SAVEEXEC_B64::~Inst_SOP1__S_XNOR_SAVEEXEC_B64() {
} // ~Inst_SOP1__S_XNOR_SAVEEXEC_B64

// --- description from .arch file ---
// D.u64 = EXEC;
// EXEC = ~(S0.u64 ^ EXEC);
// SCC = 1 if the new value of EXEC is non-zero.
void
Inst_SOP1__S_XNOR_SAVEEXEC_B64::execute(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    ConstScalarOperandU64 src(gpuDynInst, instData.SSRC0);
    ScalarOperandU64 sdst(gpuDynInst, instData.SDST);
    ScalarOperandU32 scc(gpuDynInst, REG_SCC);

    src.read();

    sdst = wf->execMask().to_ullong();
    wf->execMask() = ~(src.rawData() ^ wf->execMask().to_ullong());
    scc = wf->execMask().any() ? 1 : 0;

    sdst.write();
    scc.write();
} // execute

// --- Inst_SOP1__S_QUADMASK_B32 class methods ---

Inst_SOP1__S_QUADMASK_B32::Inst_SOP1__S_QUADMASK_B32(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_quadmask_b32")
{
    setFlag(ALU);
} // Inst_SOP1__S_QUADMASK_B32

Inst_SOP1__S_QUADMASK_B32::~Inst_SOP1__S_QUADMASK_B32() {
} // ~Inst_SOP1__S_QUADMASK_B32

// --- description from .arch file ---
// D.u = QuadMask(S0.u):
// D[0] = OR(S0[3:0]), D[1] = OR(S0[7:4]) ... D[31:8] = 0;
// SCC = 1 if result is non-zero.
void
Inst_SOP1__S_QUADMASK_B32::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU32 src(gpuDynInst, instData.SSRC0);
    ScalarOperandU32 sdst(gpuDynInst, instData.SDST);
    ScalarOperandU32 scc(gpuDynInst, REG_SCC);

    src.read();

    sdst = quadMask(src.rawData());
    scc = sdst.rawData() ? 1 : 0;

    sdst.write();
    scc.write();
} // execute

// --- Inst_SOP1__S_QUADMASK_B64 class methods ---

Inst_SOP1__S_QUADMASK_B64::Inst_SOP1__S_QUADMASK_B64(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_quadmask_b64")
{
    setFlag(ALU);
} // Inst_SOP1__S_QUADMASK_B64

Inst_SOP1__S_QUADMASK_B64::~Inst_SOP1__S_QUADMASK_B64() {
} // ~Inst_SOP1__S_QUADMASK_B64

// --- description from .arch file ---
// D.u64 = QuadMask(S0.u64):
// D[0] = OR(S0[3:0]), D[1] = OR(S0[7:4]) ... D[63:16] = 0;
// SCC = 1 if result is non-zero.
void
Inst_SOP1__S_QUADMASK_B64::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU64 src(gpuDynInst, instData.SSRC0);
    ScalarOperandU64 sdst(gpuDynInst, instData.SDST);
    ScalarOperandU32 scc(gpuDynInst, REG_SCC);

    src.read();

    sdst = quadMask(src.rawData());
    scc = sdst.rawData() ? 1 : 0;

    sdst.write();
    scc.write();
} // execute

// --- Inst_SOP1__S_MOVRELS_B32 class methods ---

Inst_SOP1__S_MOVRELS_B32::Inst_SOP1__S_MOVRELS_B32(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_movrels_b32")
{
    setFlag(ALU);
} // Inst_SOP1__S_MOVRELS_B32

Inst_SOP1__S_MOVRELS_B32::~Inst_SOP1__S_MOVRELS_B32() {
} // ~Inst_SOP1__S_MOVRELS_B32

// --- description from .arch file ---
// D.u = SGPR[S0.u + M0.u].u (move from relative source).
void
Inst_SOP1__S_MOVRELS_B32::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU32 m0(gpuDynInst, REG_M0);
    m0.read();
    ConstScalarOperandU32 src(gpuDynInst, instData.SSRC0 + m0.rawData());
    ScalarOperandU32 sdst(gpuDynInst, instData.SDST);

    src.read();

    sdst = src.rawData();

    sdst.write();
} // execute

// --- Inst_SOP1__S_MOVRELS_B64 class methods ---

Inst_SOP1__S_MOVRELS_B64::Inst_SOP1__S_MOVRELS_B64(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_movrels_b64")
{
    setFlag(ALU);
} // Inst_SOP1__S_MOVRELS_B64

Inst_SOP1__S_MOVRELS_B64::~Inst_SOP1__S_MOVRELS_B64() {
} // ~Inst_SOP1__S_MOVRELS_B64

// --- description from .arch file ---
// D.u64 = SGPR[S0.u + M0.u].u64 (move from relative source).
// The index in M0.u must be even for this operation.
void
Inst_SOP1__S_MOVRELS_B64::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU32 m0(gpuDynInst, REG_M0);
    m0.read();
    ConstScalarOperandU64 src(gpuDynInst, instData.SSRC0 + m0.rawData());
    ScalarOperandU64 sdst(gpuDynInst, instData.SDST);

    src.read();

    sdst = src.rawData();

    sdst.write();
} // execute

// --- Inst_SOP1__S_MOVRELD_B32 class methods ---

Inst_SOP1__S_MOVRELD_B32::Inst_SOP1__S_MOVRELD_B32(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_movreld_b32")
{
    setFlag(ALU);
} // Inst_SOP1__S_MOVRELD_B32

Inst_SOP1__S_MOVRELD_B32::~Inst_SOP1__S_MOVRELD_B32() {
} // ~Inst_SOP1__S_MOVRELD_B32

// --- description from .arch file ---
// SGPR[D.u + M0.u].u = S0.u (move to relative destination).
void
Inst_SOP1__S_MOVRELD_B32::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU32 m0(gpuDynInst, REG_M0);
    m0.read();
    ConstScalarOperandU32 src(gpuDynInst, instData.SSRC0);
    ScalarOperandU32 sdst(gpuDynInst, instData.SDST + m0.rawData());

    src.read();

    sdst = src.rawData();

    sdst.write();
} // execute

// --- Inst_SOP1__S_MOVRELD_B64 class methods ---

Inst_SOP1__S_MOVRELD_B64::Inst_SOP1__S_MOVRELD_B64(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_movreld_b64")
{
    setFlag(ALU);
} // Inst_SOP1__S_MOVRELD_B64

Inst_SOP1__S_MOVRELD_B64::~Inst_SOP1__S_MOVRELD_B64() {
} // ~Inst_SOP1__S_MOVRELD_B64

// --- description from .arch file ---
// SGPR[D.u + M0.u].u64 = S0.u64 (move to relative destination).
// The index in M0.u must be even for this operation.
void
Inst_SOP1__S_MOVRELD_B64::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandU32 m0(gpuDynInst, REG_M0);
    m0.read();
    ConstScalarOperandU64 src(gpuDynInst, instData.SSRC0);
    ScalarOperandU64 sdst(gpuDynInst, instData.SDST + m0.rawData());

    src.read();

    sdst = src.rawData();

    sdst.write();
} // execute

// --- Inst_SOP1__S_CBRANCH_JOIN class methods ---

Inst_SOP1__S_CBRANCH_JOIN::Inst_SOP1__S_CBRANCH_JOIN(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_cbranch_join")
{
    setFlag(Branch);
    setFlag(WritesEXEC);
} // Inst_SOP1__S_CBRANCH_JOIN

Inst_SOP1__S_CBRANCH_JOIN::~Inst_SOP1__S_CBRANCH_JOIN() {
} // ~Inst_SOP1__S_CBRANCH_JOIN

// --- description from .arch file ---
// saved_csp = S0.u;
// if (CSP == saved_csp) then
//     PC += 4; // Second time to JOIN: continue with program.
// else
//     CSP -= 1; // First time to JOIN; jump to other FORK path.
//     {PC, EXEC} = SGPR[CSP * 4]; // Read 128 bits from 4 consecutive
//     SGPRs.
// end
// Conditional branch join point (end of conditional branch block). S0 is
// saved CSP value.
// See S_CBRANCH_G_FORK and S_CBRANCH_I_FORK for related instructions.
void
Inst_SOP1__S_CBRANCH_JOIN::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_SOP1__S_ABS_I32 class methods ---

Inst_SOP1__S_ABS_I32::Inst_SOP1__S_ABS_I32(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_abs_i32")
{
    setFlag(ALU);
} // Inst_SOP1__S_ABS_I32

Inst_SOP1__S_ABS_I32::~Inst_SOP1__S_ABS_I32() {} // ~Inst_SOP1__S_ABS_I32

// --- description from .arch file ---
// if (S.i < 0) then D.i = -S.i;
// else D.i = S.i;
// SCC = 1 if result is non-zero.
// Integer absolute value.
void
Inst_SOP1__S_ABS_I32::execute(GPUDynInstPtr gpuDynInst)
{
    ConstScalarOperandI32 src(gpuDynInst, instData.SSRC0);
    ScalarOperandI32 sdst(gpuDynInst, instData.SDST);
    ScalarOperandU32 scc(gpuDynInst, REG_SCC);

    src.read();

    sdst = std::abs(src.rawData());

    scc = sdst.rawData() ? 1 : 0;

    sdst.write();
    scc.write();
} // execute

// --- Inst_SOP1__S_MOV_FED_B32 class methods ---

Inst_SOP1__S_MOV_FED_B32::Inst_SOP1__S_MOV_FED_B32(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_mov_fed_b32")
{
    setFlag(ALU);
} // Inst_SOP1__S_MOV_FED_B32

Inst_SOP1__S_MOV_FED_B32::~Inst_SOP1__S_MOV_FED_B32() {
} // ~Inst_SOP1__S_MOV_FED_B32

// --- description from .arch file ---
// D.u = S0.u. Introduce an EDC double-detect error on write to the
// destination SGPR.
void
Inst_SOP1__S_MOV_FED_B32::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_SOP1__S_SET_GPR_IDX_IDX class methods ---

Inst_SOP1__S_SET_GPR_IDX_IDX::Inst_SOP1__S_SET_GPR_IDX_IDX(InFmt_SOP1 *iFmt)
    : Inst_SOP1(iFmt, "s_set_gpr_idx_idx")
{} // Inst_SOP1__S_SET_GPR_IDX_IDX

Inst_SOP1__S_SET_GPR_IDX_IDX::~Inst_SOP1__S_SET_GPR_IDX_IDX() {
} // ~Inst_SOP1__S_SET_GPR_IDX_IDX

// --- description from .arch file ---
// M0[7:0] = S0.u[7:0].
// Modify the index used in vector GPR indexing.
void
Inst_SOP1__S_SET_GPR_IDX_IDX::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute
} // namespace VegaISA
} // namespace gem5
