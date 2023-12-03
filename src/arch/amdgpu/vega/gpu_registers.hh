/*
 * Copyright (c) 2015-2021 Advanced Micro Devices, Inc.
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

#ifndef __ARCH_VEGA_REGISTERS_HH__
#define __ARCH_VEGA_REGISTERS_HH__

#include <array>
#include <cstdint>
#include <string>

#include "arch/generic/vec_reg.hh"
#include "base/intmath.hh"
#include "base/logging.hh"

namespace gem5
{

namespace VegaISA
{
enum OpSelector : int
{
    REG_SGPR_MIN = 0,
    REG_SGPR_MAX = 101,
    REG_FLAT_SCRATCH_LO = 102,
    REG_FLAT_SCRATCH_HI = 103,
    REG_XNACK_MASK_LO = 104,
    REG_XNACK_MASK_HI = 105,
    REG_VCC_LO = 106,
    REG_VCC_HI = 107,
    REG_TBA_LO = 108,
    REG_TBA_HI = 109,
    REG_TMA_LO = 110,
    REG_TMA_HI = 111,
    REG_TTMP_0 = 112,
    REG_TTMP_1 = 113,
    REG_TTMP_2 = 114,
    REG_TTMP_3 = 115,
    REG_TTMP_4 = 116,
    REG_TTMP_5 = 117,
    REG_TTMP_6 = 118,
    REG_TTMP_7 = 119,
    REG_TTMP_8 = 120,
    REG_TTMP_9 = 121,
    REG_TTMP_10 = 122,
    REG_TTMP_11 = 123,
    REG_M0 = 124,
    REG_RESERVED_1 = 125,
    REG_EXEC_LO = 126,
    REG_EXEC_HI = 127,
    REG_ZERO = 128,
    REG_INT_CONST_POS_MIN = 129,
    REG_INT_CONST_POS_MAX = 192,
    REG_INT_CONST_NEG_MIN = 193,
    REG_INT_CONST_NEG_MAX = 208,
    REG_RESERVED_2 = 209,
    REG_RESERVED_3 = 210,
    REG_RESERVED_4 = 211,
    REG_RESERVED_5 = 212,
    REG_RESERVED_6 = 213,
    REG_RESERVED_7 = 214,
    REG_RESERVED_8 = 215,
    REG_RESERVED_9 = 216,
    REG_RESERVED_10 = 217,
    REG_RESERVED_11 = 218,
    REG_RESERVED_12 = 219,
    REG_RESERVED_13 = 220,
    REG_RESERVED_14 = 221,
    REG_RESERVED_15 = 222,
    REG_RESERVED_16 = 223,
    REG_RESERVED_17 = 224,
    REG_RESERVED_18 = 225,
    REG_RESERVED_19 = 226,
    REG_RESERVED_20 = 227,
    REG_RESERVED_21 = 228,
    REG_RESERVED_22 = 229,
    REG_RESERVED_23 = 230,
    REG_RESERVED_24 = 231,
    REG_RESERVED_25 = 232,
    REG_RESERVED_26 = 233,
    REG_RESERVED_27 = 234,
    REG_SHARED_BASE = 235,
    REG_SHARED_LIMIT = 236,
    REG_PRIVATE_BASE = 237,
    REG_PRIVATE_LIMIT = 238,
    REG_RESERVED_32 = 239,
    REG_POS_HALF = 240,
    REG_NEG_HALF = 241,
    REG_POS_ONE = 242,
    REG_NEG_ONE = 243,
    REG_POS_TWO = 244,
    REG_NEG_TWO = 245,
    REG_POS_FOUR = 246,
    REG_NEG_FOUR = 247,
    REG_PI = 248,
    /* NOTE: SDWA and SWDA both refer to sub d-word addressing */
    REG_SRC_SWDA = 249,
    REG_SRC_DPP = 250,
    REG_VCCZ = 251,
    REG_EXECZ = 252,
    REG_SCC = 253,
    REG_LDS_DIRECT = 254,
    REG_SRC_LITERAL = 255,
    REG_VGPR_MIN = 256,
    REG_VGPR_MAX = 767
};

constexpr size_t MaxOperandDwords(16);
const int NumVecElemPerVecReg(64);
// op selector values 129 - 192 correspond to const values 1 - 64
const int NumPosConstRegs = REG_INT_CONST_POS_MAX - REG_INT_CONST_POS_MIN + 1;
// op selector values 193 - 208 correspond to const values -1 - 16
const int NumNegConstRegs = REG_INT_CONST_NEG_MAX - REG_INT_CONST_NEG_MIN + 1;
const int BITS_PER_BYTE = 8;
const int BITS_PER_WORD = 16;
const int MSB_PER_BYTE = (BITS_PER_BYTE - 1);
const int MSB_PER_WORD = (BITS_PER_WORD - 1);

// typedefs for the various sizes/types of scalar regs
typedef uint8_t ScalarRegU8;
typedef int8_t ScalarRegI8;
typedef uint16_t ScalarRegU16;
typedef int16_t ScalarRegI16;
typedef uint32_t ScalarRegU32;
typedef int32_t ScalarRegI32;
typedef float ScalarRegF32;
typedef uint64_t ScalarRegU64;
typedef int64_t ScalarRegI64;
typedef double ScalarRegF64;

// typedefs for the various sizes/types of vector reg elements
typedef uint8_t VecElemU8;
typedef int8_t VecElemI8;
typedef uint16_t VecElemU16;
typedef int16_t VecElemI16;
typedef uint32_t VecElemU32;
typedef int32_t VecElemI32;
typedef float VecElemF32;
typedef uint64_t VecElemU64;
typedef int64_t VecElemI64;
typedef double VecElemF64;

const int DWordSize = sizeof(VecElemU32);
/**
 * Size of a single-precision register in DWords.
 */
const int RegSizeDWords = sizeof(VecElemU32) / DWordSize;

using VecRegContainerU32 =
    VecRegContainer<sizeof(VecElemU32) * NumVecElemPerVecReg>;
using VecRegContainerU64 =
    VecRegContainer<sizeof(VecElemU64) * NumVecElemPerVecReg>;

struct StatusReg
{
    StatusReg()
        : SCC(0),
          SPI_PRIO(0),
          USER_PRIO(0),
          PRIV(0),
          TRAP_EN(0),
          TTRACE_EN(0),
          EXPORT_RDY(0),
          EXECZ(0),
          VCCZ(0),
          IN_TG(0),
          IN_BARRIER(0),
          HALT(0),
          TRAP(0),
          TTRACE_CU_EN(0),
          VALID(0),
          ECC_ERR(0),
          SKIP_EXPORT(0),
          PERF_EN(0),
          COND_DBG_USER(0),
          COND_DBG_SYS(0),
          ALLOW_REPLAY(0),
          INSTRUCTION_ATC(0),
          RESERVED(0),
          MUST_EXPORT(0),
          RESERVED_1(0)
    {}

    uint32_t SCC : 1;
    uint32_t SPI_PRIO : 2;
    uint32_t USER_PRIO : 2;
    uint32_t PRIV : 1;
    uint32_t TRAP_EN : 1;
    uint32_t TTRACE_EN : 1;
    uint32_t EXPORT_RDY : 1;
    uint32_t EXECZ : 1;
    uint32_t VCCZ : 1;
    uint32_t IN_TG : 1;
    uint32_t IN_BARRIER : 1;
    uint32_t HALT : 1;
    uint32_t TRAP : 1;
    uint32_t TTRACE_CU_EN : 1;
    uint32_t VALID : 1;
    uint32_t ECC_ERR : 1;
    uint32_t SKIP_EXPORT : 1;
    uint32_t PERF_EN : 1;
    uint32_t COND_DBG_USER : 1;
    uint32_t COND_DBG_SYS : 1;
    uint32_t ALLOW_REPLAY : 1;
    uint32_t INSTRUCTION_ATC : 1;
    uint32_t RESERVED : 3;
    uint32_t MUST_EXPORT : 1;
    uint32_t RESERVED_1 : 4;
};

std::string opSelectorToRegSym(int opIdx, int numRegs = 0);
int opSelectorToRegIdx(int opIdx, int numScalarRegs);
bool isPosConstVal(int opIdx);
bool isNegConstVal(int opIdx);
bool isConstVal(int opIdx);
bool isLiteral(int opIdx);
bool isScalarReg(int opIdx);
bool isVectorReg(int opIdx);
bool isFlatScratchReg(int opIdx);
bool isExecMask(int opIdx);
bool isVccReg(int opIdx);
} // namespace VegaISA
} // namespace gem5

#endif // __ARCH_VEGA_REGISTERS_HH__
