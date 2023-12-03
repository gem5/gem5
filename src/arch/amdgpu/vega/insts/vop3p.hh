/*
 * Copyright (c) 2023 Advanced Micro Devices, Inc.
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

#ifndef __ARCH_VEGA_INSTS_VOP3P_HH__
#define __ARCH_VEGA_INSTS_VOP3P_HH__

#include "arch/amdgpu/vega/gpu_decoder.hh"
#include "arch/amdgpu/vega/insts/gpu_static_inst.hh"
#include "arch/amdgpu/vega/insts/op_encodings.hh"
#include "debug/VEGA.hh"

namespace gem5
{

namespace VegaISA
{
// One source operand
class Inst_VOP3P__1OP : public Inst_VOP3P
{
  public:
    Inst_VOP3P__1OP(InFmt_VOP3P *iFmt, const std::string &name)
        : Inst_VOP3P(iFmt, name)
    {
        setFlag(ALU);
    }

    int
    getNumOperands() override
    {
        return numDstRegOperands() + numSrcRegOperands();
    } // getNumOperands

    int
    numDstRegOperands() override
    {
        return 1;
    }

    int
    numSrcRegOperands() override
    {
        return 1;
    }

    int
    getOperandSize(int opIdx) override
    {
        switch (opIdx) {
        case 0: // src
            return 4;
        case 1: // dst
            return 4;
        default:
            fatal("op idx %i out of bounds\n", opIdx);
            return -1;
        }
    }
};

// Two source operands with two 16-bit values in a dword
class Inst_VOP3P__2OP_X16 : public Inst_VOP3P
{
  public:
    Inst_VOP3P__2OP_X16(InFmt_VOP3P *iFmt, const std::string &name)
        : Inst_VOP3P(iFmt, name)
    {
        setFlag(ALU);
    }

    int
    getNumOperands() override
    {
        return numDstRegOperands() + numSrcRegOperands();
    } // getNumOperands

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
        case 0: // src0
            return 4;
        case 1: // src1
            return 4;
        case 2: // dst
            return 4;
        default:
            fatal("op idx %i out of bounds\n", opIdx);
            return -1;
        }
    }
};

// Three source operands with two 16-bit values in a dword
class Inst_VOP3P__3OP_X16 : public Inst_VOP3P
{
  public:
    Inst_VOP3P__3OP_X16(InFmt_VOP3P *iFmt, const std::string &name)
        : Inst_VOP3P(iFmt, name)
    {
        setFlag(ALU);
    }

    int
    getNumOperands() override
    {
        return numDstRegOperands() + numSrcRegOperands();
    } // getNumOperands

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
        case 0: // src0
            return 4;
        case 1: // src1
            return 4;
        case 2: // src2
            return 4;
        case 3: // dst
            return 4;
        default:
            fatal("op idx %i out of bounds\n", opIdx);
            return -1;
        }
    }
};

// Begin instruction implementations
class Inst_VOP3P__V_PK_MAD_I16 : public Inst_VOP3P__3OP_X16
{
  public:
    Inst_VOP3P__V_PK_MAD_I16(InFmt_VOP3P *iFmt)
        : Inst_VOP3P__3OP_X16(iFmt, "v_pk_mad_i16")
    {}

    void execute(GPUDynInstPtr gpuDynInst) override;
};

class Inst_VOP3P__V_PK_MUL_LO_U16 : public Inst_VOP3P__2OP_X16
{
  public:
    Inst_VOP3P__V_PK_MUL_LO_U16(InFmt_VOP3P *iFmt)
        : Inst_VOP3P__2OP_X16(iFmt, "v_pk_mul_lo_u16")
    {}

    void execute(GPUDynInstPtr gpuDynInst) override;
};

class Inst_VOP3P__V_PK_ADD_I16 : public Inst_VOP3P__2OP_X16
{
  public:
    Inst_VOP3P__V_PK_ADD_I16(InFmt_VOP3P *iFmt)
        : Inst_VOP3P__2OP_X16(iFmt, "v_pk_add_i16")
    {}

    void execute(GPUDynInstPtr gpuDynInst) override;
};

class Inst_VOP3P__V_PK_SUB_I16 : public Inst_VOP3P__2OP_X16
{
  public:
    Inst_VOP3P__V_PK_SUB_I16(InFmt_VOP3P *iFmt)
        : Inst_VOP3P__2OP_X16(iFmt, "v_pk_sub_i16")
    {}

    void execute(GPUDynInstPtr gpuDynInst) override;
};

class Inst_VOP3P__V_PK_LSHLREV_B16 : public Inst_VOP3P__2OP_X16
{
  public:
    Inst_VOP3P__V_PK_LSHLREV_B16(InFmt_VOP3P *iFmt)
        : Inst_VOP3P__2OP_X16(iFmt, "v_pk_lshlrev_b16")
    {}

    void execute(GPUDynInstPtr gpuDynInst) override;
};

class Inst_VOP3P__V_PK_LSHRREV_B16 : public Inst_VOP3P__2OP_X16
{
  public:
    Inst_VOP3P__V_PK_LSHRREV_B16(InFmt_VOP3P *iFmt)
        : Inst_VOP3P__2OP_X16(iFmt, "v_pk_lshrrev_b16")
    {}

    void execute(GPUDynInstPtr gpuDynInst) override;
};

class Inst_VOP3P__V_PK_ASHRREV_B16 : public Inst_VOP3P__2OP_X16
{
  public:
    Inst_VOP3P__V_PK_ASHRREV_B16(InFmt_VOP3P *iFmt)
        : Inst_VOP3P__2OP_X16(iFmt, "v_pk_ashrrev_b16")
    {}

    void execute(GPUDynInstPtr gpuDynInst) override;
};

class Inst_VOP3P__V_PK_MAX_I16 : public Inst_VOP3P__2OP_X16
{
  public:
    Inst_VOP3P__V_PK_MAX_I16(InFmt_VOP3P *iFmt)
        : Inst_VOP3P__2OP_X16(iFmt, "v_pk_max_i16")
    {}

    void execute(GPUDynInstPtr gpuDynInst) override;
};

class Inst_VOP3P__V_PK_MIN_I16 : public Inst_VOP3P__2OP_X16
{
  public:
    Inst_VOP3P__V_PK_MIN_I16(InFmt_VOP3P *iFmt)
        : Inst_VOP3P__2OP_X16(iFmt, "v_pk_min_i16")
    {}

    void execute(GPUDynInstPtr gpuDynInst) override;
};

class Inst_VOP3P__V_PK_MAD_U16 : public Inst_VOP3P__3OP_X16
{
  public:
    Inst_VOP3P__V_PK_MAD_U16(InFmt_VOP3P *iFmt)
        : Inst_VOP3P__3OP_X16(iFmt, "v_pk_mad_u16")
    {}

    void execute(GPUDynInstPtr gpuDynInst) override;
};

class Inst_VOP3P__V_PK_ADD_U16 : public Inst_VOP3P__2OP_X16
{
  public:
    Inst_VOP3P__V_PK_ADD_U16(InFmt_VOP3P *iFmt)
        : Inst_VOP3P__2OP_X16(iFmt, "v_pk_add_u16")
    {}

    void execute(GPUDynInstPtr gpuDynInst) override;
};

class Inst_VOP3P__V_PK_SUB_U16 : public Inst_VOP3P__2OP_X16
{
  public:
    Inst_VOP3P__V_PK_SUB_U16(InFmt_VOP3P *iFmt)
        : Inst_VOP3P__2OP_X16(iFmt, "v_pk_sub_u16")
    {}

    void execute(GPUDynInstPtr gpuDynInst) override;
};

class Inst_VOP3P__V_PK_MAX_U16 : public Inst_VOP3P__2OP_X16
{
  public:
    Inst_VOP3P__V_PK_MAX_U16(InFmt_VOP3P *iFmt)
        : Inst_VOP3P__2OP_X16(iFmt, "v_pk_max_u16")
    {}

    void execute(GPUDynInstPtr gpuDynInst) override;
};

class Inst_VOP3P__V_PK_MIN_U16 : public Inst_VOP3P__2OP_X16
{
  public:
    Inst_VOP3P__V_PK_MIN_U16(InFmt_VOP3P *iFmt)
        : Inst_VOP3P__2OP_X16(iFmt, "v_pk_min_u16")
    {}

    void execute(GPUDynInstPtr gpuDynInst) override;
};

class Inst_VOP3P__V_PK_FMA_F16 : public Inst_VOP3P__3OP_X16
{
  public:
    Inst_VOP3P__V_PK_FMA_F16(InFmt_VOP3P *iFmt)
        : Inst_VOP3P__3OP_X16(iFmt, "v_pk_fma_f16")
    {}

    void execute(GPUDynInstPtr gpuDynInst) override;
};

class Inst_VOP3P__V_PK_ADD_F16 : public Inst_VOP3P__2OP_X16
{
  public:
    Inst_VOP3P__V_PK_ADD_F16(InFmt_VOP3P *iFmt)
        : Inst_VOP3P__2OP_X16(iFmt, "v_pk_add_f16")
    {}

    void execute(GPUDynInstPtr gpuDynInst) override;
};

class Inst_VOP3P__V_PK_MUL_F16 : public Inst_VOP3P__2OP_X16
{
  public:
    Inst_VOP3P__V_PK_MUL_F16(InFmt_VOP3P *iFmt)
        : Inst_VOP3P__2OP_X16(iFmt, "v_pk_mul_f16")
    {}

    void execute(GPUDynInstPtr gpuDynInst) override;
};

class Inst_VOP3P__V_PK_MIN_F16 : public Inst_VOP3P__2OP_X16
{
  public:
    Inst_VOP3P__V_PK_MIN_F16(InFmt_VOP3P *iFmt)
        : Inst_VOP3P__2OP_X16(iFmt, "v_pk_min_f16")
    {}

    void execute(GPUDynInstPtr gpuDynInst) override;
};

class Inst_VOP3P__V_PK_MAX_F16 : public Inst_VOP3P__2OP_X16
{
  public:
    Inst_VOP3P__V_PK_MAX_F16(InFmt_VOP3P *iFmt)
        : Inst_VOP3P__2OP_X16(iFmt, "v_pk_max_f16")
    {}

    void execute(GPUDynInstPtr gpuDynInst) override;
};

class Inst_VOP3P__V_DOT2_F32_F16 : public Inst_VOP3P__3OP_X16
{
  public:
    Inst_VOP3P__V_DOT2_F32_F16(InFmt_VOP3P *iFmt)
        : Inst_VOP3P__3OP_X16(iFmt, "v_dot2_f32_f16")
    {}

    void execute(GPUDynInstPtr gpuDynInst) override;
};

class Inst_VOP3P__V_DOT2_I32_I16 : public Inst_VOP3P__3OP_X16
{
  public:
    Inst_VOP3P__V_DOT2_I32_I16(InFmt_VOP3P *iFmt)
        : Inst_VOP3P__3OP_X16(iFmt, "v_dot2_i32_i16")
    {}

    void execute(GPUDynInstPtr gpuDynInst) override;
};

class Inst_VOP3P__V_DOT2_U32_U16 : public Inst_VOP3P__3OP_X16
{
  public:
    Inst_VOP3P__V_DOT2_U32_U16(InFmt_VOP3P *iFmt)
        : Inst_VOP3P__3OP_X16(iFmt, "v_dot2_u32_u16")
    {}

    void execute(GPUDynInstPtr gpuDynInst) override;
};

class Inst_VOP3P__V_DOT4_I32_I8 : public Inst_VOP3P__3OP_X16
{
  public:
    Inst_VOP3P__V_DOT4_I32_I8(InFmt_VOP3P *iFmt)
        : Inst_VOP3P__3OP_X16(iFmt, "v_dot4_i32_i8")
    {}

    void execute(GPUDynInstPtr gpuDynInst) override;
};

class Inst_VOP3P__V_DOT4_U32_U8 : public Inst_VOP3P__3OP_X16
{
  public:
    Inst_VOP3P__V_DOT4_U32_U8(InFmt_VOP3P *iFmt)
        : Inst_VOP3P__3OP_X16(iFmt, "v_dot4_u32_u8")
    {}

    void execute(GPUDynInstPtr gpuDynInst) override;
};

class Inst_VOP3P__V_DOT8_I32_I4 : public Inst_VOP3P__3OP_X16
{
  public:
    Inst_VOP3P__V_DOT8_I32_I4(InFmt_VOP3P *iFmt)
        : Inst_VOP3P__3OP_X16(iFmt, "v_dot8_i32_i4")
    {}

    void execute(GPUDynInstPtr gpuDynInst) override;
};

class Inst_VOP3P__V_DOT8_U32_U4 : public Inst_VOP3P__3OP_X16
{
  public:
    Inst_VOP3P__V_DOT8_U32_U4(InFmt_VOP3P *iFmt)
        : Inst_VOP3P__3OP_X16(iFmt, "v_dot8_u32_u4")
    {}

    void execute(GPUDynInstPtr gpuDynInst) override;
};

class Inst_VOP3P__V_ACCVGPR_READ : public Inst_VOP3P__1OP
{
  public:
    Inst_VOP3P__V_ACCVGPR_READ(InFmt_VOP3P *iFmt)
        : Inst_VOP3P__1OP(iFmt, "v_accvgpr_read")
    {}

    void execute(GPUDynInstPtr gpuDynInst) override;
};

class Inst_VOP3P__V_ACCVGPR_WRITE : public Inst_VOP3P__1OP
{
  public:
    Inst_VOP3P__V_ACCVGPR_WRITE(InFmt_VOP3P *iFmt)
        : Inst_VOP3P__1OP(iFmt, "v_accvgpr_write")
    {}

    void execute(GPUDynInstPtr gpuDynInst) override;
};
} // namespace VegaISA
} // namespace gem5

#endif // __ARCH_VEGA_INSTS_VOP3P_HH__
