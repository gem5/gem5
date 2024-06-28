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

#include "arch/amdgpu/vega/gpu_decoder.hh"

#include <vector>

#include "arch/amdgpu/vega/insts/gpu_static_inst.hh"
#include "arch/amdgpu/vega/insts/instructions.hh"
#include "arch/amdgpu/vega/insts/vop3p.hh"

namespace gem5
{

namespace VegaISA
{
    Decoder::Decoder()
    {
    } // Decoder

    Decoder::~Decoder()
    {
    } // ~Decoder

    /*
     * These will probably have to be updated according to the Vega ISA manual:
     * https://developer.amd.com/wp-content/resources/
     * Vega_Shader_ISA_28July2017.pdf
     */
    IsaDecodeMethod Decoder::tableDecodePrimary[] = {
        &Decoder::decode_OP_VOP2__V_CNDMASK_B32,
        &Decoder::decode_OP_VOP2__V_CNDMASK_B32,
        &Decoder::decode_OP_VOP2__V_CNDMASK_B32,
        &Decoder::decode_OP_VOP2__V_CNDMASK_B32,
        &Decoder::decode_OP_VOP2__V_ADD_F32,
        &Decoder::decode_OP_VOP2__V_ADD_F32,
        &Decoder::decode_OP_VOP2__V_ADD_F32,
        &Decoder::decode_OP_VOP2__V_ADD_F32,
        &Decoder::decode_OP_VOP2__V_SUB_F32,
        &Decoder::decode_OP_VOP2__V_SUB_F32,
        &Decoder::decode_OP_VOP2__V_SUB_F32,
        &Decoder::decode_OP_VOP2__V_SUB_F32,
        &Decoder::decode_OP_VOP2__V_SUBREV_F32,
        &Decoder::decode_OP_VOP2__V_SUBREV_F32,
        &Decoder::decode_OP_VOP2__V_SUBREV_F32,
        &Decoder::decode_OP_VOP2__V_SUBREV_F32,
        &Decoder::decode_OP_VOP2__V_MUL_LEGACY_F32,
        &Decoder::decode_OP_VOP2__V_MUL_LEGACY_F32,
        &Decoder::decode_OP_VOP2__V_MUL_LEGACY_F32,
        &Decoder::decode_OP_VOP2__V_MUL_LEGACY_F32,
        &Decoder::decode_OP_VOP2__V_MUL_F32,
        &Decoder::decode_OP_VOP2__V_MUL_F32,
        &Decoder::decode_OP_VOP2__V_MUL_F32,
        &Decoder::decode_OP_VOP2__V_MUL_F32,
        &Decoder::decode_OP_VOP2__V_MUL_I32_I24,
        &Decoder::decode_OP_VOP2__V_MUL_I32_I24,
        &Decoder::decode_OP_VOP2__V_MUL_I32_I24,
        &Decoder::decode_OP_VOP2__V_MUL_I32_I24,
        &Decoder::decode_OP_VOP2__V_MUL_HI_I32_I24,
        &Decoder::decode_OP_VOP2__V_MUL_HI_I32_I24,
        &Decoder::decode_OP_VOP2__V_MUL_HI_I32_I24,
        &Decoder::decode_OP_VOP2__V_MUL_HI_I32_I24,
        &Decoder::decode_OP_VOP2__V_MUL_U32_U24,
        &Decoder::decode_OP_VOP2__V_MUL_U32_U24,
        &Decoder::decode_OP_VOP2__V_MUL_U32_U24,
        &Decoder::decode_OP_VOP2__V_MUL_U32_U24,
        &Decoder::decode_OP_VOP2__V_MUL_HI_U32_U24,
        &Decoder::decode_OP_VOP2__V_MUL_HI_U32_U24,
        &Decoder::decode_OP_VOP2__V_MUL_HI_U32_U24,
        &Decoder::decode_OP_VOP2__V_MUL_HI_U32_U24,
        &Decoder::decode_OP_VOP2__V_MIN_F32,
        &Decoder::decode_OP_VOP2__V_MIN_F32,
        &Decoder::decode_OP_VOP2__V_MIN_F32,
        &Decoder::decode_OP_VOP2__V_MIN_F32,
        &Decoder::decode_OP_VOP2__V_MAX_F32,
        &Decoder::decode_OP_VOP2__V_MAX_F32,
        &Decoder::decode_OP_VOP2__V_MAX_F32,
        &Decoder::decode_OP_VOP2__V_MAX_F32,
        &Decoder::decode_OP_VOP2__V_MIN_I32,
        &Decoder::decode_OP_VOP2__V_MIN_I32,
        &Decoder::decode_OP_VOP2__V_MIN_I32,
        &Decoder::decode_OP_VOP2__V_MIN_I32,
        &Decoder::decode_OP_VOP2__V_MAX_I32,
        &Decoder::decode_OP_VOP2__V_MAX_I32,
        &Decoder::decode_OP_VOP2__V_MAX_I32,
        &Decoder::decode_OP_VOP2__V_MAX_I32,
        &Decoder::decode_OP_VOP2__V_MIN_U32,
        &Decoder::decode_OP_VOP2__V_MIN_U32,
        &Decoder::decode_OP_VOP2__V_MIN_U32,
        &Decoder::decode_OP_VOP2__V_MIN_U32,
        &Decoder::decode_OP_VOP2__V_MAX_U32,
        &Decoder::decode_OP_VOP2__V_MAX_U32,
        &Decoder::decode_OP_VOP2__V_MAX_U32,
        &Decoder::decode_OP_VOP2__V_MAX_U32,
        &Decoder::decode_OP_VOP2__V_LSHRREV_B32,
        &Decoder::decode_OP_VOP2__V_LSHRREV_B32,
        &Decoder::decode_OP_VOP2__V_LSHRREV_B32,
        &Decoder::decode_OP_VOP2__V_LSHRREV_B32,
        &Decoder::decode_OP_VOP2__V_ASHRREV_I32,
        &Decoder::decode_OP_VOP2__V_ASHRREV_I32,
        &Decoder::decode_OP_VOP2__V_ASHRREV_I32,
        &Decoder::decode_OP_VOP2__V_ASHRREV_I32,
        &Decoder::decode_OP_VOP2__V_LSHLREV_B32,
        &Decoder::decode_OP_VOP2__V_LSHLREV_B32,
        &Decoder::decode_OP_VOP2__V_LSHLREV_B32,
        &Decoder::decode_OP_VOP2__V_LSHLREV_B32,
        &Decoder::decode_OP_VOP2__V_AND_B32,
        &Decoder::decode_OP_VOP2__V_AND_B32,
        &Decoder::decode_OP_VOP2__V_AND_B32,
        &Decoder::decode_OP_VOP2__V_AND_B32,
        &Decoder::decode_OP_VOP2__V_OR_B32,
        &Decoder::decode_OP_VOP2__V_OR_B32,
        &Decoder::decode_OP_VOP2__V_OR_B32,
        &Decoder::decode_OP_VOP2__V_OR_B32,
        &Decoder::decode_OP_VOP2__V_XOR_B32,
        &Decoder::decode_OP_VOP2__V_XOR_B32,
        &Decoder::decode_OP_VOP2__V_XOR_B32,
        &Decoder::decode_OP_VOP2__V_XOR_B32,
        &Decoder::decode_OP_VOP2__V_MAC_F32,
        &Decoder::decode_OP_VOP2__V_MAC_F32,
        &Decoder::decode_OP_VOP2__V_MAC_F32,
        &Decoder::decode_OP_VOP2__V_MAC_F32,
        &Decoder::decode_OP_VOP2__V_MADMK_F32,
        &Decoder::decode_OP_VOP2__V_MADMK_F32,
        &Decoder::decode_OP_VOP2__V_MADMK_F32,
        &Decoder::decode_OP_VOP2__V_MADMK_F32,
        &Decoder::decode_OP_VOP2__V_MADAK_F32,
        &Decoder::decode_OP_VOP2__V_MADAK_F32,
        &Decoder::decode_OP_VOP2__V_MADAK_F32,
        &Decoder::decode_OP_VOP2__V_MADAK_F32,
        &Decoder::decode_OP_VOP2__V_ADD_CO_U32,
        &Decoder::decode_OP_VOP2__V_ADD_CO_U32,
        &Decoder::decode_OP_VOP2__V_ADD_CO_U32,
        &Decoder::decode_OP_VOP2__V_ADD_CO_U32,
        &Decoder::decode_OP_VOP2__V_SUB_CO_U32,
        &Decoder::decode_OP_VOP2__V_SUB_CO_U32,
        &Decoder::decode_OP_VOP2__V_SUB_CO_U32,
        &Decoder::decode_OP_VOP2__V_SUB_CO_U32,
        &Decoder::decode_OP_VOP2__V_SUBREV_CO_U32,
        &Decoder::decode_OP_VOP2__V_SUBREV_CO_U32,
        &Decoder::decode_OP_VOP2__V_SUBREV_CO_U32,
        &Decoder::decode_OP_VOP2__V_SUBREV_CO_U32,
        &Decoder::decode_OP_VOP2__V_ADDC_CO_U32,
        &Decoder::decode_OP_VOP2__V_ADDC_CO_U32,
        &Decoder::decode_OP_VOP2__V_ADDC_CO_U32,
        &Decoder::decode_OP_VOP2__V_ADDC_CO_U32,
        &Decoder::decode_OP_VOP2__V_SUBB_CO_U32,
        &Decoder::decode_OP_VOP2__V_SUBB_CO_U32,
        &Decoder::decode_OP_VOP2__V_SUBB_CO_U32,
        &Decoder::decode_OP_VOP2__V_SUBB_CO_U32,
        &Decoder::decode_OP_VOP2__V_SUBBREV_CO_U32,
        &Decoder::decode_OP_VOP2__V_SUBBREV_CO_U32,
        &Decoder::decode_OP_VOP2__V_SUBBREV_CO_U32,
        &Decoder::decode_OP_VOP2__V_SUBBREV_CO_U32,
        &Decoder::decode_OP_VOP2__V_ADD_F16,
        &Decoder::decode_OP_VOP2__V_ADD_F16,
        &Decoder::decode_OP_VOP2__V_ADD_F16,
        &Decoder::decode_OP_VOP2__V_ADD_F16,
        &Decoder::decode_OP_VOP2__V_SUB_F16,
        &Decoder::decode_OP_VOP2__V_SUB_F16,
        &Decoder::decode_OP_VOP2__V_SUB_F16,
        &Decoder::decode_OP_VOP2__V_SUB_F16,
        &Decoder::decode_OP_VOP2__V_SUBREV_F16,
        &Decoder::decode_OP_VOP2__V_SUBREV_F16,
        &Decoder::decode_OP_VOP2__V_SUBREV_F16,
        &Decoder::decode_OP_VOP2__V_SUBREV_F16,
        &Decoder::decode_OP_VOP2__V_MUL_F16,
        &Decoder::decode_OP_VOP2__V_MUL_F16,
        &Decoder::decode_OP_VOP2__V_MUL_F16,
        &Decoder::decode_OP_VOP2__V_MUL_F16,
        &Decoder::decode_OP_VOP2__V_MAC_F16,
        &Decoder::decode_OP_VOP2__V_MAC_F16,
        &Decoder::decode_OP_VOP2__V_MAC_F16,
        &Decoder::decode_OP_VOP2__V_MAC_F16,
        &Decoder::decode_OP_VOP2__V_MADMK_F16,
        &Decoder::decode_OP_VOP2__V_MADMK_F16,
        &Decoder::decode_OP_VOP2__V_MADMK_F16,
        &Decoder::decode_OP_VOP2__V_MADMK_F16,
        &Decoder::decode_OP_VOP2__V_MADAK_F16,
        &Decoder::decode_OP_VOP2__V_MADAK_F16,
        &Decoder::decode_OP_VOP2__V_MADAK_F16,
        &Decoder::decode_OP_VOP2__V_MADAK_F16,
        &Decoder::decode_OP_VOP2__V_ADD_U16,
        &Decoder::decode_OP_VOP2__V_ADD_U16,
        &Decoder::decode_OP_VOP2__V_ADD_U16,
        &Decoder::decode_OP_VOP2__V_ADD_U16,
        &Decoder::decode_OP_VOP2__V_SUB_U16,
        &Decoder::decode_OP_VOP2__V_SUB_U16,
        &Decoder::decode_OP_VOP2__V_SUB_U16,
        &Decoder::decode_OP_VOP2__V_SUB_U16,
        &Decoder::decode_OP_VOP2__V_SUBREV_U16,
        &Decoder::decode_OP_VOP2__V_SUBREV_U16,
        &Decoder::decode_OP_VOP2__V_SUBREV_U16,
        &Decoder::decode_OP_VOP2__V_SUBREV_U16,
        &Decoder::decode_OP_VOP2__V_MUL_LO_U16,
        &Decoder::decode_OP_VOP2__V_MUL_LO_U16,
        &Decoder::decode_OP_VOP2__V_MUL_LO_U16,
        &Decoder::decode_OP_VOP2__V_MUL_LO_U16,
        &Decoder::decode_OP_VOP2__V_LSHLREV_B16,
        &Decoder::decode_OP_VOP2__V_LSHLREV_B16,
        &Decoder::decode_OP_VOP2__V_LSHLREV_B16,
        &Decoder::decode_OP_VOP2__V_LSHLREV_B16,
        &Decoder::decode_OP_VOP2__V_LSHRREV_B16,
        &Decoder::decode_OP_VOP2__V_LSHRREV_B16,
        &Decoder::decode_OP_VOP2__V_LSHRREV_B16,
        &Decoder::decode_OP_VOP2__V_LSHRREV_B16,
        &Decoder::decode_OP_VOP2__V_ASHRREV_I16,
        &Decoder::decode_OP_VOP2__V_ASHRREV_I16,
        &Decoder::decode_OP_VOP2__V_ASHRREV_I16,
        &Decoder::decode_OP_VOP2__V_ASHRREV_I16,
        &Decoder::decode_OP_VOP2__V_MAX_F16,
        &Decoder::decode_OP_VOP2__V_MAX_F16,
        &Decoder::decode_OP_VOP2__V_MAX_F16,
        &Decoder::decode_OP_VOP2__V_MAX_F16,
        &Decoder::decode_OP_VOP2__V_MIN_F16,
        &Decoder::decode_OP_VOP2__V_MIN_F16,
        &Decoder::decode_OP_VOP2__V_MIN_F16,
        &Decoder::decode_OP_VOP2__V_MIN_F16,
        &Decoder::decode_OP_VOP2__V_MAX_U16,
        &Decoder::decode_OP_VOP2__V_MAX_U16,
        &Decoder::decode_OP_VOP2__V_MAX_U16,
        &Decoder::decode_OP_VOP2__V_MAX_U16,
        &Decoder::decode_OP_VOP2__V_MAX_I16,
        &Decoder::decode_OP_VOP2__V_MAX_I16,
        &Decoder::decode_OP_VOP2__V_MAX_I16,
        &Decoder::decode_OP_VOP2__V_MAX_I16,
        &Decoder::decode_OP_VOP2__V_MIN_U16,
        &Decoder::decode_OP_VOP2__V_MIN_U16,
        &Decoder::decode_OP_VOP2__V_MIN_U16,
        &Decoder::decode_OP_VOP2__V_MIN_U16,
        &Decoder::decode_OP_VOP2__V_MIN_I16,
        &Decoder::decode_OP_VOP2__V_MIN_I16,
        &Decoder::decode_OP_VOP2__V_MIN_I16,
        &Decoder::decode_OP_VOP2__V_MIN_I16,
        &Decoder::decode_OP_VOP2__V_LDEXP_F16,
        &Decoder::decode_OP_VOP2__V_LDEXP_F16,
        &Decoder::decode_OP_VOP2__V_LDEXP_F16,
        &Decoder::decode_OP_VOP2__V_LDEXP_F16,
        &Decoder::decode_OP_VOP2__V_ADD_U32,
        &Decoder::decode_OP_VOP2__V_ADD_U32,
        &Decoder::decode_OP_VOP2__V_ADD_U32,
        &Decoder::decode_OP_VOP2__V_ADD_U32,
        &Decoder::decode_OP_VOP2__V_SUB_U32,
        &Decoder::decode_OP_VOP2__V_SUB_U32,
        &Decoder::decode_OP_VOP2__V_SUB_U32,
        &Decoder::decode_OP_VOP2__V_SUB_U32,
        &Decoder::decode_OP_VOP2__V_SUBREV_U32,
        &Decoder::decode_OP_VOP2__V_SUBREV_U32,
        &Decoder::decode_OP_VOP2__V_SUBREV_U32,
        &Decoder::decode_OP_VOP2__V_SUBREV_U32,
        &Decoder::decode_OP_VOP2__V_DOT2C_F32_F16,
        &Decoder::decode_OP_VOP2__V_DOT2C_F32_F16,
        &Decoder::decode_OP_VOP2__V_DOT2C_F32_F16,
        &Decoder::decode_OP_VOP2__V_DOT2C_F32_F16,
        &Decoder::decode_OP_VOP2__V_DOT2C_I32_I16,
        &Decoder::decode_OP_VOP2__V_DOT2C_I32_I16,
        &Decoder::decode_OP_VOP2__V_DOT2C_I32_I16,
        &Decoder::decode_OP_VOP2__V_DOT2C_I32_I16,
        &Decoder::decode_OP_VOP2__V_DOT4C_I32_I8,
        &Decoder::decode_OP_VOP2__V_DOT4C_I32_I8,
        &Decoder::decode_OP_VOP2__V_DOT4C_I32_I8,
        &Decoder::decode_OP_VOP2__V_DOT4C_I32_I8,
        &Decoder::decode_OP_VOP2__V_DOT8C_I32_I4,
        &Decoder::decode_OP_VOP2__V_DOT8C_I32_I4,
        &Decoder::decode_OP_VOP2__V_DOT8C_I32_I4,
        &Decoder::decode_OP_VOP2__V_DOT8C_I32_I4,
        &Decoder::decode_OP_VOP2__V_FMAC_F32,
        &Decoder::decode_OP_VOP2__V_FMAC_F32,
        &Decoder::decode_OP_VOP2__V_FMAC_F32,
        &Decoder::decode_OP_VOP2__V_FMAC_F32,
        &Decoder::decode_OP_VOP2__V_PK_FMAC_F16,
        &Decoder::decode_OP_VOP2__V_PK_FMAC_F16,
        &Decoder::decode_OP_VOP2__V_PK_FMAC_F16,
        &Decoder::decode_OP_VOP2__V_PK_FMAC_F16,
        &Decoder::decode_OP_VOP2__V_XNOR_B32,
        &Decoder::decode_OP_VOP2__V_XNOR_B32,
        &Decoder::decode_OP_VOP2__V_XNOR_B32,
        &Decoder::decode_OP_VOP2__V_XNOR_B32,
        &Decoder::subDecode_OP_VOPC,
        &Decoder::subDecode_OP_VOPC,
        &Decoder::subDecode_OP_VOPC,
        &Decoder::subDecode_OP_VOPC,
        &Decoder::subDecode_OP_VOP1,
        &Decoder::subDecode_OP_VOP1,
        &Decoder::subDecode_OP_VOP1,
        &Decoder::subDecode_OP_VOP1,
        &Decoder::decode_OP_SOP2__S_ADD_U32,
        &Decoder::decode_OP_SOP2__S_SUB_U32,
        &Decoder::decode_OP_SOP2__S_ADD_I32,
        &Decoder::decode_OP_SOP2__S_SUB_I32,
        &Decoder::decode_OP_SOP2__S_ADDC_U32,
        &Decoder::decode_OP_SOP2__S_SUBB_U32,
        &Decoder::decode_OP_SOP2__S_MIN_I32,
        &Decoder::decode_OP_SOP2__S_MIN_U32,
        &Decoder::decode_OP_SOP2__S_MAX_I32,
        &Decoder::decode_OP_SOP2__S_MAX_U32,
        &Decoder::decode_OP_SOP2__S_CSELECT_B32,
        &Decoder::decode_OP_SOP2__S_CSELECT_B64,
        &Decoder::decode_OP_SOP2__S_AND_B32,
        &Decoder::decode_OP_SOP2__S_AND_B64,
        &Decoder::decode_OP_SOP2__S_OR_B32,
        &Decoder::decode_OP_SOP2__S_OR_B64,
        &Decoder::decode_OP_SOP2__S_XOR_B32,
        &Decoder::decode_OP_SOP2__S_XOR_B64,
        &Decoder::decode_OP_SOP2__S_ANDN2_B32,
        &Decoder::decode_OP_SOP2__S_ANDN2_B64,
        &Decoder::decode_OP_SOP2__S_ORN2_B32,
        &Decoder::decode_OP_SOP2__S_ORN2_B64,
        &Decoder::decode_OP_SOP2__S_NAND_B32,
        &Decoder::decode_OP_SOP2__S_NAND_B64,
        &Decoder::decode_OP_SOP2__S_NOR_B32,
        &Decoder::decode_OP_SOP2__S_NOR_B64,
        &Decoder::decode_OP_SOP2__S_XNOR_B32,
        &Decoder::decode_OP_SOP2__S_XNOR_B64,
        &Decoder::decode_OP_SOP2__S_LSHL_B32,
        &Decoder::decode_OP_SOP2__S_LSHL_B64,
        &Decoder::decode_OP_SOP2__S_LSHR_B32,
        &Decoder::decode_OP_SOP2__S_LSHR_B64,
        &Decoder::decode_OP_SOP2__S_ASHR_I32,
        &Decoder::decode_OP_SOP2__S_ASHR_I64,
        &Decoder::decode_OP_SOP2__S_BFM_B32,
        &Decoder::decode_OP_SOP2__S_BFM_B64,
        &Decoder::decode_OP_SOP2__S_MUL_I32,
        &Decoder::decode_OP_SOP2__S_BFE_U32,
        &Decoder::decode_OP_SOP2__S_BFE_I32,
        &Decoder::decode_OP_SOP2__S_BFE_U64,
        &Decoder::decode_OP_SOP2__S_BFE_I64,
        &Decoder::decode_OP_SOP2__S_CBRANCH_G_FORK,
        &Decoder::decode_OP_SOP2__S_ABSDIFF_I32,
        &Decoder::decode_OP_SOP2__S_RFE_RESTORE_B64,
        &Decoder::decode_OP_SOP2__S_MUL_HI_U32,
        &Decoder::decode_OP_SOP2__S_MUL_HI_I32,
        &Decoder::decode_OP_SOP2__S_LSHL1_ADD_U32,
        &Decoder::decode_OP_SOP2__S_LSHL2_ADD_U32,
        &Decoder::decode_OP_SOP2__S_LSHL3_ADD_U32,
        &Decoder::decode_OP_SOP2__S_LSHL4_ADD_U32,
        &Decoder::decode_OP_SOP2__S_PACK_LL_B32_B16,
        &Decoder::decode_OP_SOP2__S_PACK_LH_B32_B16,
        &Decoder::decode_OP_SOP2__S_HH_B32_B16,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_SOPK__S_MOVK_I32,
        &Decoder::decode_OP_SOPK__S_CMOVK_I32,
        &Decoder::decode_OP_SOPK__S_CMPK_EQ_I32,
        &Decoder::decode_OP_SOPK__S_CMPK_LG_I32,
        &Decoder::decode_OP_SOPK__S_CMPK_GT_I32,
        &Decoder::decode_OP_SOPK__S_CMPK_GE_I32,
        &Decoder::decode_OP_SOPK__S_CMPK_LT_I32,
        &Decoder::decode_OP_SOPK__S_CMPK_LE_I32,
        &Decoder::decode_OP_SOPK__S_CMPK_EQ_U32,
        &Decoder::decode_OP_SOPK__S_CMPK_LG_U32,
        &Decoder::decode_OP_SOPK__S_CMPK_GT_U32,
        &Decoder::decode_OP_SOPK__S_CMPK_GE_U32,
        &Decoder::decode_OP_SOPK__S_CMPK_LT_U32,
        &Decoder::decode_OP_SOPK__S_CMPK_LE_U32,
        &Decoder::decode_OP_SOPK__S_ADDK_I32,
        &Decoder::decode_OP_SOPK__S_MULK_I32,
        &Decoder::decode_OP_SOPK__S_CBRANCH_I_FORK,
        &Decoder::decode_OP_SOPK__S_GETREG_B32,
        &Decoder::decode_OP_SOPK__S_SETREG_B32,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_SOPK__S_SETREG_IMM32_B32,
        &Decoder::decode_OP_SOPK__S_CALL_B64,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::subDecode_OP_SOP1,
        &Decoder::subDecode_OP_SOPC,
        &Decoder::subDecode_OP_SOPP,
        &Decoder::subDecode_OP_SMEM,
        &Decoder::subDecode_OP_SMEM,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_EXP,
        &Decoder::decode_OP_EXP,
        &Decoder::decode_OP_EXP,
        &Decoder::decode_OP_EXP,
        &Decoder::decode_OP_EXP,
        &Decoder::decode_OP_EXP,
        &Decoder::decode_OP_EXP,
        &Decoder::decode_OP_EXP,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::subDecode_OPU_VOP3,
        &Decoder::subDecode_OPU_VOP3,
        &Decoder::subDecode_OPU_VOP3,
        &Decoder::subDecode_OPU_VOP3,
        &Decoder::subDecode_OPU_VOP3,
        &Decoder::subDecode_OPU_VOP3,
        &Decoder::decode_invalid,
        &Decoder::subDecode_OP_VOP3P,
        &Decoder::subDecode_OP_VINTRP,
        &Decoder::subDecode_OP_VINTRP,
        &Decoder::subDecode_OP_VINTRP,
        &Decoder::subDecode_OP_VINTRP,
        &Decoder::subDecode_OP_VINTRP,
        &Decoder::subDecode_OP_VINTRP,
        &Decoder::subDecode_OP_VINTRP,
        &Decoder::subDecode_OP_VINTRP,
        &Decoder::subDecode_OP_DS,
        &Decoder::subDecode_OP_DS,
        &Decoder::subDecode_OP_DS,
        &Decoder::subDecode_OP_DS,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::subDecode_OP_FLAT,
        &Decoder::subDecode_OP_FLAT,
        &Decoder::subDecode_OP_FLAT,
        &Decoder::subDecode_OP_FLAT,
        &Decoder::subDecode_OP_FLAT,
        &Decoder::subDecode_OP_FLAT,
        &Decoder::subDecode_OP_FLAT,
        &Decoder::subDecode_OP_FLAT,
        &Decoder::subDecode_OP_MUBUF,
        &Decoder::subDecode_OP_MUBUF,
        &Decoder::subDecode_OP_MUBUF,
        &Decoder::subDecode_OP_MUBUF,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::subDecode_OP_MTBUF,
        &Decoder::subDecode_OP_MTBUF,
        &Decoder::subDecode_OP_MTBUF,
        &Decoder::subDecode_OP_MTBUF,
        &Decoder::subDecode_OP_MTBUF,
        &Decoder::subDecode_OP_MTBUF,
        &Decoder::subDecode_OP_MTBUF,
        &Decoder::subDecode_OP_MTBUF,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::subDecode_OP_MIMG,
        &Decoder::subDecode_OP_MIMG,
        &Decoder::subDecode_OP_MIMG,
        &Decoder::subDecode_OP_MIMG,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid
    };

    IsaDecodeMethod Decoder::tableSubDecode_OPU_VOP3[] = {
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OPU_VOP3__V_CMP_CLASS_F32,
        &Decoder::decode_OPU_VOP3__V_CMPX_CLASS_F32,
        &Decoder::decode_OPU_VOP3__V_CMP_CLASS_F64,
        &Decoder::decode_OPU_VOP3__V_CMPX_CLASS_F64,
        &Decoder::decode_OPU_VOP3__V_CMP_CLASS_F16,
        &Decoder::decode_OPU_VOP3__V_CMPX_CLASS_F16,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OPU_VOP3__V_CMP_F_F16,
        &Decoder::decode_OPU_VOP3__V_CMP_LT_F16,
        &Decoder::decode_OPU_VOP3__V_CMP_EQ_F16,
        &Decoder::decode_OPU_VOP3__V_CMP_LE_F16,
        &Decoder::decode_OPU_VOP3__V_CMP_GT_F16,
        &Decoder::decode_OPU_VOP3__V_CMP_LG_F16,
        &Decoder::decode_OPU_VOP3__V_CMP_GE_F16,
        &Decoder::decode_OPU_VOP3__V_CMP_O_F16,
        &Decoder::decode_OPU_VOP3__V_CMP_U_F16,
        &Decoder::decode_OPU_VOP3__V_CMP_NGE_F16,
        &Decoder::decode_OPU_VOP3__V_CMP_NLG_F16,
        &Decoder::decode_OPU_VOP3__V_CMP_NGT_F16,
        &Decoder::decode_OPU_VOP3__V_CMP_NLE_F16,
        &Decoder::decode_OPU_VOP3__V_CMP_NEQ_F16,
        &Decoder::decode_OPU_VOP3__V_CMP_NLT_F16,
        &Decoder::decode_OPU_VOP3__V_CMP_TRU_F16,
        &Decoder::decode_OPU_VOP3__V_CMPX_F_F16,
        &Decoder::decode_OPU_VOP3__V_CMPX_LT_F16,
        &Decoder::decode_OPU_VOP3__V_CMPX_EQ_F16,
        &Decoder::decode_OPU_VOP3__V_CMPX_LE_F16,
        &Decoder::decode_OPU_VOP3__V_CMPX_GT_F16,
        &Decoder::decode_OPU_VOP3__V_CMPX_LG_F16,
        &Decoder::decode_OPU_VOP3__V_CMPX_GE_F16,
        &Decoder::decode_OPU_VOP3__V_CMPX_O_F16,
        &Decoder::decode_OPU_VOP3__V_CMPX_U_F16,
        &Decoder::decode_OPU_VOP3__V_CMPX_NGE_F16,
        &Decoder::decode_OPU_VOP3__V_CMPX_NLG_F16,
        &Decoder::decode_OPU_VOP3__V_CMPX_NGT_F16,
        &Decoder::decode_OPU_VOP3__V_CMPX_NLE_F16,
        &Decoder::decode_OPU_VOP3__V_CMPX_NEQ_F16,
        &Decoder::decode_OPU_VOP3__V_CMPX_NLT_F16,
        &Decoder::decode_OPU_VOP3__V_CMPX_TRU_F16,
        &Decoder::decode_OPU_VOP3__V_CMP_F_F32,
        &Decoder::decode_OPU_VOP3__V_CMP_LT_F32,
        &Decoder::decode_OPU_VOP3__V_CMP_EQ_F32,
        &Decoder::decode_OPU_VOP3__V_CMP_LE_F32,
        &Decoder::decode_OPU_VOP3__V_CMP_GT_F32,
        &Decoder::decode_OPU_VOP3__V_CMP_LG_F32,
        &Decoder::decode_OPU_VOP3__V_CMP_GE_F32,
        &Decoder::decode_OPU_VOP3__V_CMP_O_F32,
        &Decoder::decode_OPU_VOP3__V_CMP_U_F32,
        &Decoder::decode_OPU_VOP3__V_CMP_NGE_F32,
        &Decoder::decode_OPU_VOP3__V_CMP_NLG_F32,
        &Decoder::decode_OPU_VOP3__V_CMP_NGT_F32,
        &Decoder::decode_OPU_VOP3__V_CMP_NLE_F32,
        &Decoder::decode_OPU_VOP3__V_CMP_NEQ_F32,
        &Decoder::decode_OPU_VOP3__V_CMP_NLT_F32,
        &Decoder::decode_OPU_VOP3__V_CMP_TRU_F32,
        &Decoder::decode_OPU_VOP3__V_CMPX_F_F32,
        &Decoder::decode_OPU_VOP3__V_CMPX_LT_F32,
        &Decoder::decode_OPU_VOP3__V_CMPX_EQ_F32,
        &Decoder::decode_OPU_VOP3__V_CMPX_LE_F32,
        &Decoder::decode_OPU_VOP3__V_CMPX_GT_F32,
        &Decoder::decode_OPU_VOP3__V_CMPX_LG_F32,
        &Decoder::decode_OPU_VOP3__V_CMPX_GE_F32,
        &Decoder::decode_OPU_VOP3__V_CMPX_O_F32,
        &Decoder::decode_OPU_VOP3__V_CMPX_U_F32,
        &Decoder::decode_OPU_VOP3__V_CMPX_NGE_F32,
        &Decoder::decode_OPU_VOP3__V_CMPX_NLG_F32,
        &Decoder::decode_OPU_VOP3__V_CMPX_NGT_F32,
        &Decoder::decode_OPU_VOP3__V_CMPX_NLE_F32,
        &Decoder::decode_OPU_VOP3__V_CMPX_NEQ_F32,
        &Decoder::decode_OPU_VOP3__V_CMPX_NLT_F32,
        &Decoder::decode_OPU_VOP3__V_CMPX_TRU_F32,
        &Decoder::decode_OPU_VOP3__V_CMP_F_F64,
        &Decoder::decode_OPU_VOP3__V_CMP_LT_F64,
        &Decoder::decode_OPU_VOP3__V_CMP_EQ_F64,
        &Decoder::decode_OPU_VOP3__V_CMP_LE_F64,
        &Decoder::decode_OPU_VOP3__V_CMP_GT_F64,
        &Decoder::decode_OPU_VOP3__V_CMP_LG_F64,
        &Decoder::decode_OPU_VOP3__V_CMP_GE_F64,
        &Decoder::decode_OPU_VOP3__V_CMP_O_F64,
        &Decoder::decode_OPU_VOP3__V_CMP_U_F64,
        &Decoder::decode_OPU_VOP3__V_CMP_NGE_F64,
        &Decoder::decode_OPU_VOP3__V_CMP_NLG_F64,
        &Decoder::decode_OPU_VOP3__V_CMP_NGT_F64,
        &Decoder::decode_OPU_VOP3__V_CMP_NLE_F64,
        &Decoder::decode_OPU_VOP3__V_CMP_NEQ_F64,
        &Decoder::decode_OPU_VOP3__V_CMP_NLT_F64,
        &Decoder::decode_OPU_VOP3__V_CMP_TRU_F64,
        &Decoder::decode_OPU_VOP3__V_CMPX_F_F64,
        &Decoder::decode_OPU_VOP3__V_CMPX_LT_F64,
        &Decoder::decode_OPU_VOP3__V_CMPX_EQ_F64,
        &Decoder::decode_OPU_VOP3__V_CMPX_LE_F64,
        &Decoder::decode_OPU_VOP3__V_CMPX_GT_F64,
        &Decoder::decode_OPU_VOP3__V_CMPX_LG_F64,
        &Decoder::decode_OPU_VOP3__V_CMPX_GE_F64,
        &Decoder::decode_OPU_VOP3__V_CMPX_O_F64,
        &Decoder::decode_OPU_VOP3__V_CMPX_U_F64,
        &Decoder::decode_OPU_VOP3__V_CMPX_NGE_F64,
        &Decoder::decode_OPU_VOP3__V_CMPX_NLG_F64,
        &Decoder::decode_OPU_VOP3__V_CMPX_NGT_F64,
        &Decoder::decode_OPU_VOP3__V_CMPX_NLE_F64,
        &Decoder::decode_OPU_VOP3__V_CMPX_NEQ_F64,
        &Decoder::decode_OPU_VOP3__V_CMPX_NLT_F64,
        &Decoder::decode_OPU_VOP3__V_CMPX_TRU_F64,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OPU_VOP3__V_CMP_F_I16,
        &Decoder::decode_OPU_VOP3__V_CMP_LT_I16,
        &Decoder::decode_OPU_VOP3__V_CMP_EQ_I16,
        &Decoder::decode_OPU_VOP3__V_CMP_LE_I16,
        &Decoder::decode_OPU_VOP3__V_CMP_GT_I16,
        &Decoder::decode_OPU_VOP3__V_CMP_NE_I16,
        &Decoder::decode_OPU_VOP3__V_CMP_GE_I16,
        &Decoder::decode_OPU_VOP3__V_CMP_T_I16,
        &Decoder::decode_OPU_VOP3__V_CMP_F_U16,
        &Decoder::decode_OPU_VOP3__V_CMP_LT_U16,
        &Decoder::decode_OPU_VOP3__V_CMP_EQ_U16,
        &Decoder::decode_OPU_VOP3__V_CMP_LE_U16,
        &Decoder::decode_OPU_VOP3__V_CMP_GT_U16,
        &Decoder::decode_OPU_VOP3__V_CMP_NE_U16,
        &Decoder::decode_OPU_VOP3__V_CMP_GE_U16,
        &Decoder::decode_OPU_VOP3__V_CMP_T_U16,
        &Decoder::decode_OPU_VOP3__V_CMPX_F_I16,
        &Decoder::decode_OPU_VOP3__V_CMPX_LT_I16,
        &Decoder::decode_OPU_VOP3__V_CMPX_EQ_I16,
        &Decoder::decode_OPU_VOP3__V_CMPX_LE_I16,
        &Decoder::decode_OPU_VOP3__V_CMPX_GT_I16,
        &Decoder::decode_OPU_VOP3__V_CMPX_NE_I16,
        &Decoder::decode_OPU_VOP3__V_CMPX_GE_I16,
        &Decoder::decode_OPU_VOP3__V_CMPX_T_I16,
        &Decoder::decode_OPU_VOP3__V_CMPX_F_U16,
        &Decoder::decode_OPU_VOP3__V_CMPX_LT_U16,
        &Decoder::decode_OPU_VOP3__V_CMPX_EQ_U16,
        &Decoder::decode_OPU_VOP3__V_CMPX_LE_U16,
        &Decoder::decode_OPU_VOP3__V_CMPX_GT_U16,
        &Decoder::decode_OPU_VOP3__V_CMPX_NE_U16,
        &Decoder::decode_OPU_VOP3__V_CMPX_GE_U16,
        &Decoder::decode_OPU_VOP3__V_CMPX_T_U16,
        &Decoder::decode_OPU_VOP3__V_CMP_F_I32,
        &Decoder::decode_OPU_VOP3__V_CMP_LT_I32,
        &Decoder::decode_OPU_VOP3__V_CMP_EQ_I32,
        &Decoder::decode_OPU_VOP3__V_CMP_LE_I32,
        &Decoder::decode_OPU_VOP3__V_CMP_GT_I32,
        &Decoder::decode_OPU_VOP3__V_CMP_NE_I32,
        &Decoder::decode_OPU_VOP3__V_CMP_GE_I32,
        &Decoder::decode_OPU_VOP3__V_CMP_T_I32,
        &Decoder::decode_OPU_VOP3__V_CMP_F_U32,
        &Decoder::decode_OPU_VOP3__V_CMP_LT_U32,
        &Decoder::decode_OPU_VOP3__V_CMP_EQ_U32,
        &Decoder::decode_OPU_VOP3__V_CMP_LE_U32,
        &Decoder::decode_OPU_VOP3__V_CMP_GT_U32,
        &Decoder::decode_OPU_VOP3__V_CMP_NE_U32,
        &Decoder::decode_OPU_VOP3__V_CMP_GE_U32,
        &Decoder::decode_OPU_VOP3__V_CMP_T_U32,
        &Decoder::decode_OPU_VOP3__V_CMPX_F_I32,
        &Decoder::decode_OPU_VOP3__V_CMPX_LT_I32,
        &Decoder::decode_OPU_VOP3__V_CMPX_EQ_I32,
        &Decoder::decode_OPU_VOP3__V_CMPX_LE_I32,
        &Decoder::decode_OPU_VOP3__V_CMPX_GT_I32,
        &Decoder::decode_OPU_VOP3__V_CMPX_NE_I32,
        &Decoder::decode_OPU_VOP3__V_CMPX_GE_I32,
        &Decoder::decode_OPU_VOP3__V_CMPX_T_I32,
        &Decoder::decode_OPU_VOP3__V_CMPX_F_U32,
        &Decoder::decode_OPU_VOP3__V_CMPX_LT_U32,
        &Decoder::decode_OPU_VOP3__V_CMPX_EQ_U32,
        &Decoder::decode_OPU_VOP3__V_CMPX_LE_U32,
        &Decoder::decode_OPU_VOP3__V_CMPX_GT_U32,
        &Decoder::decode_OPU_VOP3__V_CMPX_NE_U32,
        &Decoder::decode_OPU_VOP3__V_CMPX_GE_U32,
        &Decoder::decode_OPU_VOP3__V_CMPX_T_U32,
        &Decoder::decode_OPU_VOP3__V_CMP_F_I64,
        &Decoder::decode_OPU_VOP3__V_CMP_LT_I64,
        &Decoder::decode_OPU_VOP3__V_CMP_EQ_I64,
        &Decoder::decode_OPU_VOP3__V_CMP_LE_I64,
        &Decoder::decode_OPU_VOP3__V_CMP_GT_I64,
        &Decoder::decode_OPU_VOP3__V_CMP_NE_I64,
        &Decoder::decode_OPU_VOP3__V_CMP_GE_I64,
        &Decoder::decode_OPU_VOP3__V_CMP_T_I64,
        &Decoder::decode_OPU_VOP3__V_CMP_F_U64,
        &Decoder::decode_OPU_VOP3__V_CMP_LT_U64,
        &Decoder::decode_OPU_VOP3__V_CMP_EQ_U64,
        &Decoder::decode_OPU_VOP3__V_CMP_LE_U64,
        &Decoder::decode_OPU_VOP3__V_CMP_GT_U64,
        &Decoder::decode_OPU_VOP3__V_CMP_NE_U64,
        &Decoder::decode_OPU_VOP3__V_CMP_GE_U64,
        &Decoder::decode_OPU_VOP3__V_CMP_T_U64,
        &Decoder::decode_OPU_VOP3__V_CMPX_F_I64,
        &Decoder::decode_OPU_VOP3__V_CMPX_LT_I64,
        &Decoder::decode_OPU_VOP3__V_CMPX_EQ_I64,
        &Decoder::decode_OPU_VOP3__V_CMPX_LE_I64,
        &Decoder::decode_OPU_VOP3__V_CMPX_GT_I64,
        &Decoder::decode_OPU_VOP3__V_CMPX_NE_I64,
        &Decoder::decode_OPU_VOP3__V_CMPX_GE_I64,
        &Decoder::decode_OPU_VOP3__V_CMPX_T_I64,
        &Decoder::decode_OPU_VOP3__V_CMPX_F_U64,
        &Decoder::decode_OPU_VOP3__V_CMPX_LT_U64,
        &Decoder::decode_OPU_VOP3__V_CMPX_EQ_U64,
        &Decoder::decode_OPU_VOP3__V_CMPX_LE_U64,
        &Decoder::decode_OPU_VOP3__V_CMPX_GT_U64,
        &Decoder::decode_OPU_VOP3__V_CMPX_NE_U64,
        &Decoder::decode_OPU_VOP3__V_CMPX_GE_U64,
        &Decoder::decode_OPU_VOP3__V_CMPX_T_U64,
        &Decoder::decode_OPU_VOP3__V_CNDMASK_B32,
        &Decoder::decode_OPU_VOP3__V_ADD_F32,
        &Decoder::decode_OPU_VOP3__V_SUB_F32,
        &Decoder::decode_OPU_VOP3__V_SUBREV_F32,
        &Decoder::decode_OPU_VOP3__V_MUL_LEGACY_F32,
        &Decoder::decode_OPU_VOP3__V_MUL_F32,
        &Decoder::decode_OPU_VOP3__V_MUL_I32_I24,
        &Decoder::decode_OPU_VOP3__V_MUL_HI_I32_I24,
        &Decoder::decode_OPU_VOP3__V_MUL_U32_U24,
        &Decoder::decode_OPU_VOP3__V_MUL_HI_U32_U24,
        &Decoder::decode_OPU_VOP3__V_MIN_F32,
        &Decoder::decode_OPU_VOP3__V_MAX_F32,
        &Decoder::decode_OPU_VOP3__V_MIN_I32,
        &Decoder::decode_OPU_VOP3__V_MAX_I32,
        &Decoder::decode_OPU_VOP3__V_MIN_U32,
        &Decoder::decode_OPU_VOP3__V_MAX_U32,
        &Decoder::decode_OPU_VOP3__V_LSHRREV_B32,
        &Decoder::decode_OPU_VOP3__V_ASHRREV_I32,
        &Decoder::decode_OPU_VOP3__V_LSHLREV_B32,
        &Decoder::decode_OPU_VOP3__V_AND_B32,
        &Decoder::decode_OPU_VOP3__V_OR_B32,
        &Decoder::decode_OPU_VOP3__V_XOR_B32,
        &Decoder::decode_OPU_VOP3__V_MAC_F32,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OPU_VOP3__V_ADD_CO_U32,
        &Decoder::decode_OPU_VOP3__V_SUB_CO_U32,
        &Decoder::decode_OPU_VOP3__V_SUBREV_CO_U32,
        &Decoder::decode_OPU_VOP3__V_ADDC_CO_U32,
        &Decoder::decode_OPU_VOP3__V_SUBB_CO_U32,
        &Decoder::decode_OPU_VOP3__V_SUBBREV_CO_U32,
        &Decoder::decode_OPU_VOP3__V_ADD_F16,
        &Decoder::decode_OPU_VOP3__V_SUB_F16,
        &Decoder::decode_OPU_VOP3__V_SUBREV_F16,
        &Decoder::decode_OPU_VOP3__V_MUL_F16,
        &Decoder::decode_OPU_VOP3__V_MAC_F16,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OPU_VOP3__V_ADD_U16,
        &Decoder::decode_OPU_VOP3__V_SUB_U16,
        &Decoder::decode_OPU_VOP3__V_SUBREV_U16,
        &Decoder::decode_OPU_VOP3__V_MUL_LO_U16,
        &Decoder::decode_OPU_VOP3__V_LSHLREV_B16,
        &Decoder::decode_OPU_VOP3__V_LSHRREV_B16,
        &Decoder::decode_OPU_VOP3__V_ASHRREV_I16,
        &Decoder::decode_OPU_VOP3__V_MAX_F16,
        &Decoder::decode_OPU_VOP3__V_MIN_F16,
        &Decoder::decode_OPU_VOP3__V_MAX_U16,
        &Decoder::decode_OPU_VOP3__V_MAX_I16,
        &Decoder::decode_OPU_VOP3__V_MIN_U16,
        &Decoder::decode_OPU_VOP3__V_MIN_I16,
        &Decoder::decode_OPU_VOP3__V_LDEXP_F16,
        &Decoder::decode_OPU_VOP3__V_ADD_U32,
        &Decoder::decode_OPU_VOP3__V_SUB_U32,
        &Decoder::decode_OPU_VOP3__V_SUBREV_U32,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OPU_VOP3__V_FMAC_F32,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OPU_VOP3__V_NOP,
        &Decoder::decode_OPU_VOP3__V_MOV_B32,
        &Decoder::decode_invalid,
        &Decoder::decode_OPU_VOP3__V_CVT_I32_F64,
        &Decoder::decode_OPU_VOP3__V_CVT_F64_I32,
        &Decoder::decode_OPU_VOP3__V_CVT_F32_I32,
        &Decoder::decode_OPU_VOP3__V_CVT_F32_U32,
        &Decoder::decode_OPU_VOP3__V_CVT_U32_F32,
        &Decoder::decode_OPU_VOP3__V_CVT_I32_F32,
        &Decoder::decode_OPU_VOP3__V_MOV_FED_B32,
        &Decoder::decode_OPU_VOP3__V_CVT_F16_F32,
        &Decoder::decode_OPU_VOP3__V_CVT_F32_F16,
        &Decoder::decode_OPU_VOP3__V_CVT_RPI_I32_F32,
        &Decoder::decode_OPU_VOP3__V_CVT_FLR_I32_F32,
        &Decoder::decode_OPU_VOP3__V_CVT_OFF_F32_I4,
        &Decoder::decode_OPU_VOP3__V_CVT_F32_F64,
        &Decoder::decode_OPU_VOP3__V_CVT_F64_F32,
        &Decoder::decode_OPU_VOP3__V_CVT_F32_UBYTE0,
        &Decoder::decode_OPU_VOP3__V_CVT_F32_UBYTE1,
        &Decoder::decode_OPU_VOP3__V_CVT_F32_UBYTE2,
        &Decoder::decode_OPU_VOP3__V_CVT_F32_UBYTE3,
        &Decoder::decode_OPU_VOP3__V_CVT_U32_F64,
        &Decoder::decode_OPU_VOP3__V_CVT_F64_U32,
        &Decoder::decode_OPU_VOP3__V_TRUNC_F64,
        &Decoder::decode_OPU_VOP3__V_CEIL_F64,
        &Decoder::decode_OPU_VOP3__V_RNDNE_F64,
        &Decoder::decode_OPU_VOP3__V_FLOOR_F64,
        &Decoder::decode_OPU_VOP3__V_FRACT_F32,
        &Decoder::decode_OPU_VOP3__V_TRUNC_F32,
        &Decoder::decode_OPU_VOP3__V_CEIL_F32,
        &Decoder::decode_OPU_VOP3__V_RNDNE_F32,
        &Decoder::decode_OPU_VOP3__V_FLOOR_F32,
        &Decoder::decode_OPU_VOP3__V_EXP_F32,
        &Decoder::decode_OPU_VOP3__V_LOG_F32,
        &Decoder::decode_OPU_VOP3__V_RCP_F32,
        &Decoder::decode_OPU_VOP3__V_RCP_IFLAG_F32,
        &Decoder::decode_OPU_VOP3__V_RSQ_F32,
        &Decoder::decode_OPU_VOP3__V_RCP_F64,
        &Decoder::decode_OPU_VOP3__V_RSQ_F64,
        &Decoder::decode_OPU_VOP3__V_SQRT_F32,
        &Decoder::decode_OPU_VOP3__V_SQRT_F64,
        &Decoder::decode_OPU_VOP3__V_SIN_F32,
        &Decoder::decode_OPU_VOP3__V_COS_F32,
        &Decoder::decode_OPU_VOP3__V_NOT_B32,
        &Decoder::decode_OPU_VOP3__V_BFREV_B32,
        &Decoder::decode_OPU_VOP3__V_FFBH_U32,
        &Decoder::decode_OPU_VOP3__V_FFBL_B32,
        &Decoder::decode_OPU_VOP3__V_FFBH_I32,
        &Decoder::decode_OPU_VOP3__V_FREXP_EXP_I32_F64,
        &Decoder::decode_OPU_VOP3__V_FREXP_MANT_F64,
        &Decoder::decode_OPU_VOP3__V_FRACT_F64,
        &Decoder::decode_OPU_VOP3__V_FREXP_EXP_I32_F32,
        &Decoder::decode_OPU_VOP3__V_FREXP_MANT_F32,
        &Decoder::decode_OPU_VOP3__V_CLREXCP,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OPU_VOP3__V_CVT_F16_U16,
        &Decoder::decode_OPU_VOP3__V_CVT_F16_I16,
        &Decoder::decode_OPU_VOP3__V_CVT_U16_F16,
        &Decoder::decode_OPU_VOP3__V_CVT_I16_F16,
        &Decoder::decode_OPU_VOP3__V_RCP_F16,
        &Decoder::decode_OPU_VOP3__V_SQRT_F16,
        &Decoder::decode_OPU_VOP3__V_RSQ_F16,
        &Decoder::decode_OPU_VOP3__V_LOG_F16,
        &Decoder::decode_OPU_VOP3__V_EXP_F16,
        &Decoder::decode_OPU_VOP3__V_FREXP_MANT_F16,
        &Decoder::decode_OPU_VOP3__V_FREXP_EXP_I16_F16,
        &Decoder::decode_OPU_VOP3__V_FLOOR_F16,
        &Decoder::decode_OPU_VOP3__V_CEIL_F16,
        &Decoder::decode_OPU_VOP3__V_TRUNC_F16,
        &Decoder::decode_OPU_VOP3__V_RNDNE_F16,
        &Decoder::decode_OPU_VOP3__V_FRACT_F16,
        &Decoder::decode_OPU_VOP3__V_SIN_F16,
        &Decoder::decode_OPU_VOP3__V_COS_F16,
        &Decoder::decode_OPU_VOP3__V_EXP_LEGACY_F32,
        &Decoder::decode_OPU_VOP3__V_LOG_LEGACY_F32,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OPU_VOP3__V_MAD_LEGACY_F32,
        &Decoder::decode_OPU_VOP3__V_MAD_F32,
        &Decoder::decode_OPU_VOP3__V_MAD_I32_I24,
        &Decoder::decode_OPU_VOP3__V_MAD_U32_U24,
        &Decoder::decode_OPU_VOP3__V_CUBEID_F32,
        &Decoder::decode_OPU_VOP3__V_CUBESC_F32,
        &Decoder::decode_OPU_VOP3__V_CUBETC_F32,
        &Decoder::decode_OPU_VOP3__V_CUBEMA_F32,
        &Decoder::decode_OPU_VOP3__V_BFE_U32,
        &Decoder::decode_OPU_VOP3__V_BFE_I32,
        &Decoder::decode_OPU_VOP3__V_BFI_B32,
        &Decoder::decode_OPU_VOP3__V_FMA_F32,
        &Decoder::decode_OPU_VOP3__V_FMA_F64,
        &Decoder::decode_OPU_VOP3__V_LERP_U8,
        &Decoder::decode_OPU_VOP3__V_ALIGNBIT_B32,
        &Decoder::decode_OPU_VOP3__V_ALIGNBYTE_B32,
        &Decoder::decode_OPU_VOP3__V_MIN3_F32,
        &Decoder::decode_OPU_VOP3__V_MIN3_I32,
        &Decoder::decode_OPU_VOP3__V_MIN3_U32,
        &Decoder::decode_OPU_VOP3__V_MAX3_F32,
        &Decoder::decode_OPU_VOP3__V_MAX3_I32,
        &Decoder::decode_OPU_VOP3__V_MAX3_U32,
        &Decoder::decode_OPU_VOP3__V_MED3_F32,
        &Decoder::decode_OPU_VOP3__V_MED3_I32,
        &Decoder::decode_OPU_VOP3__V_MED3_U32,
        &Decoder::decode_OPU_VOP3__V_SAD_U8,
        &Decoder::decode_OPU_VOP3__V_SAD_HI_U8,
        &Decoder::decode_OPU_VOP3__V_SAD_U16,
        &Decoder::decode_OPU_VOP3__V_SAD_U32,
        &Decoder::decode_OPU_VOP3__V_CVT_PK_U8_F32,
        &Decoder::decode_OPU_VOP3__V_DIV_FIXUP_F32,
        &Decoder::decode_OPU_VOP3__V_DIV_FIXUP_F64,
        &Decoder::decode_OPU_VOP3__V_DIV_SCALE_F32,
        &Decoder::decode_OPU_VOP3__V_DIV_SCALE_F64,
        &Decoder::decode_OPU_VOP3__V_DIV_FMAS_F32,
        &Decoder::decode_OPU_VOP3__V_DIV_FMAS_F64,
        &Decoder::decode_OPU_VOP3__V_MSAD_U8,
        &Decoder::decode_OPU_VOP3__V_QSAD_PK_U16_U8,
        &Decoder::decode_OPU_VOP3__V_MQSAD_PK_U16_U8,
        &Decoder::decode_OPU_VOP3__V_MQSAD_U32_U8,
        &Decoder::decode_OPU_VOP3__V_MAD_U64_U32,
        &Decoder::decode_OPU_VOP3__V_MAD_I64_I32,
        &Decoder::decode_OPU_VOP3__V_MAD_F16,
        &Decoder::decode_OPU_VOP3__V_MAD_U16,
        &Decoder::decode_OPU_VOP3__V_MAD_I16,
        &Decoder::decode_OPU_VOP3__V_PERM_B32,
        &Decoder::decode_OPU_VOP3__V_FMA_F16,
        &Decoder::decode_OPU_VOP3__V_DIV_FIXUP_F16,
        &Decoder::decode_OPU_VOP3__V_CVT_PKACCUM_U8_F32,
        &Decoder::decode_OPU_VOP3__V_MAD_U32_U16,
        &Decoder::decode_OPU_VOP3__V_MAD_I32_I16,
        &Decoder::decode_OPU_VOP3__V_XAD_U32,
        &Decoder::decode_OPU_VOP3__V_MIN3_F16,
        &Decoder::decode_OPU_VOP3__V_MIN3_I16,
        &Decoder::decode_OPU_VOP3__V_MIN3_U16,
        &Decoder::decode_OPU_VOP3__V_MAX3_F16,
        &Decoder::decode_OPU_VOP3__V_MAX3_I16,
        &Decoder::decode_OPU_VOP3__V_MAX3_U16,
        &Decoder::decode_OPU_VOP3__V_MED3_F16,
        &Decoder::decode_OPU_VOP3__V_MED3_I16,
        &Decoder::decode_OPU_VOP3__V_MED3_U16,
        &Decoder::decode_OPU_VOP3__V_LSHL_ADD_U32,
        &Decoder::decode_OPU_VOP3__V_ADD_LSHL_U32,
        &Decoder::decode_OPU_VOP3__V_ADD3_U32,
        &Decoder::decode_OPU_VOP3__V_LSHL_OR_B32,
        &Decoder::decode_OPU_VOP3__V_AND_OR_B32,
        &Decoder::decode_OPU_VOP3__V_OR3_B32,
        &Decoder::decode_OPU_VOP3__V_MAD_F16,
        &Decoder::decode_OPU_VOP3__V_MAD_U16,
        &Decoder::decode_OPU_VOP3__V_MAD_I16,
        &Decoder::decode_OPU_VOP3__V_FMA_F16,
        &Decoder::decode_OPU_VOP3__V_DIV_FIXUP_F16,
        &Decoder::decode_OPU_VOP3__V_LSHL_ADD_U64,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OPU_VOP3__V_INTERP_P1_F32,
        &Decoder::decode_OPU_VOP3__V_INTERP_P2_F32,
        &Decoder::decode_OPU_VOP3__V_INTERP_MOV_F32,
        &Decoder::decode_invalid,
        &Decoder::decode_OPU_VOP3__V_INTERP_P1LL_F16,
        &Decoder::decode_OPU_VOP3__V_INTERP_P1LV_F16,
        &Decoder::decode_OPU_VOP3__V_INTERP_P2_LEGACY_F16,
        &Decoder::decode_OPU_VOP3__V_INTERP_P2_F16,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OPU_VOP3__V_ADD_F64,
        &Decoder::decode_OPU_VOP3__V_MUL_F64,
        &Decoder::decode_OPU_VOP3__V_MIN_F64,
        &Decoder::decode_OPU_VOP3__V_MAX_F64,
        &Decoder::decode_OPU_VOP3__V_LDEXP_F64,
        &Decoder::decode_OPU_VOP3__V_MUL_LO_U32,
        &Decoder::decode_OPU_VOP3__V_MUL_HI_U32,
        &Decoder::decode_OPU_VOP3__V_MUL_HI_I32,
        &Decoder::decode_OPU_VOP3__V_LDEXP_F32,
        &Decoder::decode_OPU_VOP3__V_READLANE_B32,
        &Decoder::decode_OPU_VOP3__V_WRITELANE_B32,
        &Decoder::decode_OPU_VOP3__V_BCNT_U32_B32,
        &Decoder::decode_OPU_VOP3__V_MBCNT_LO_U32_B32,
        &Decoder::decode_OPU_VOP3__V_MBCNT_HI_U32_B32,
        &Decoder::decode_invalid,
        &Decoder::decode_OPU_VOP3__V_LSHLREV_B64,
        &Decoder::decode_OPU_VOP3__V_LSHRREV_B64,
        &Decoder::decode_OPU_VOP3__V_ASHRREV_I64,
        &Decoder::decode_OPU_VOP3__V_TRIG_PREOP_F64,
        &Decoder::decode_OPU_VOP3__V_BFM_B32,
        &Decoder::decode_OPU_VOP3__V_CVT_PKNORM_I16_F32,
        &Decoder::decode_OPU_VOP3__V_CVT_PKNORM_U16_F32,
        &Decoder::decode_OPU_VOP3__V_CVT_PKRTZ_F16_F32,
        &Decoder::decode_OPU_VOP3__V_CVT_PK_U16_U32,
        &Decoder::decode_OPU_VOP3__V_CVT_PK_I16_I32,
        &Decoder::decode_OPU_VOP3__V_PKNORM_I16_F16,
        &Decoder::decode_OPU_VOP3__V_PKNORM_U16_F16,
        &Decoder::decode_invalid,
        &Decoder::decode_OPU_VOP3__V_ADD_I32,
        &Decoder::decode_OPU_VOP3__V_SUB_I32,
        &Decoder::decode_OPU_VOP3__V_ADD_I16,
        &Decoder::decode_OPU_VOP3__V_SUB_I16,
        &Decoder::decode_OPU_VOP3__V_PACK_B32_F16,
        &Decoder::decode_invalid,
        &Decoder::decode_OPU_VOP3__V_CVT_PK_FP8_F32,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid
    };

    IsaDecodeMethod Decoder::tableSubDecode_OP_DS[] = {
        &Decoder::decode_OP_DS__DS_ADD_U32,
        &Decoder::decode_OP_DS__DS_SUB_U32,
        &Decoder::decode_OP_DS__DS_RSUB_U32,
        &Decoder::decode_OP_DS__DS_INC_U32,
        &Decoder::decode_OP_DS__DS_DEC_U32,
        &Decoder::decode_OP_DS__DS_MIN_I32,
        &Decoder::decode_OP_DS__DS_MAX_I32,
        &Decoder::decode_OP_DS__DS_MIN_U32,
        &Decoder::decode_OP_DS__DS_MAX_U32,
        &Decoder::decode_OP_DS__DS_AND_B32,
        &Decoder::decode_OP_DS__DS_OR_B32,
        &Decoder::decode_OP_DS__DS_XOR_B32,
        &Decoder::decode_OP_DS__DS_MSKOR_B32,
        &Decoder::decode_OP_DS__DS_WRITE_B32,
        &Decoder::decode_OP_DS__DS_WRITE2_B32,
        &Decoder::decode_OP_DS__DS_WRITE2ST64_B32,
        &Decoder::decode_OP_DS__DS_CMPST_B32,
        &Decoder::decode_OP_DS__DS_CMPST_F32,
        &Decoder::decode_OP_DS__DS_MIN_F32,
        &Decoder::decode_OP_DS__DS_MAX_F32,
        &Decoder::decode_OP_DS__DS_NOP,
        &Decoder::decode_OP_DS__DS_ADD_F32,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_DS__DS_WRITE_ADDTID_B32,
        &Decoder::decode_OP_DS__DS_WRITE_B8,
        &Decoder::decode_OP_DS__DS_WRITE_B16,
        &Decoder::decode_OP_DS__DS_ADD_RTN_U32,
        &Decoder::decode_OP_DS__DS_SUB_RTN_U32,
        &Decoder::decode_OP_DS__DS_RSUB_RTN_U32,
        &Decoder::decode_OP_DS__DS_INC_RTN_U32,
        &Decoder::decode_OP_DS__DS_DEC_RTN_U32,
        &Decoder::decode_OP_DS__DS_MIN_RTN_I32,
        &Decoder::decode_OP_DS__DS_MAX_RTN_I32,
        &Decoder::decode_OP_DS__DS_MIN_RTN_U32,
        &Decoder::decode_OP_DS__DS_MAX_RTN_U32,
        &Decoder::decode_OP_DS__DS_AND_RTN_B32,
        &Decoder::decode_OP_DS__DS_OR_RTN_B32,
        &Decoder::decode_OP_DS__DS_XOR_RTN_B32,
        &Decoder::decode_OP_DS__DS_MSKOR_RTN_B32,
        &Decoder::decode_OP_DS__DS_WRXCHG_RTN_B32,
        &Decoder::decode_OP_DS__DS_WRXCHG2_RTN_B32,
        &Decoder::decode_OP_DS__DS_WRXCHG2ST64_RTN_B32,
        &Decoder::decode_OP_DS__DS_CMPST_RTN_B32,
        &Decoder::decode_OP_DS__DS_CMPST_RTN_F32,
        &Decoder::decode_OP_DS__DS_MIN_RTN_F32,
        &Decoder::decode_OP_DS__DS_MAX_RTN_F32,
        &Decoder::decode_OP_DS__DS_WRAP_RTN_B32,
        &Decoder::decode_OP_DS__DS_ADD_RTN_F32,
        &Decoder::decode_OP_DS__DS_READ_B32,
        &Decoder::decode_OP_DS__DS_READ2_B32,
        &Decoder::decode_OP_DS__DS_READ2ST64_B32,
        &Decoder::decode_OP_DS__DS_READ_I8,
        &Decoder::decode_OP_DS__DS_READ_U8,
        &Decoder::decode_OP_DS__DS_READ_I16,
        &Decoder::decode_OP_DS__DS_READ_U16,
        &Decoder::decode_OP_DS__DS_SWIZZLE_B32,
        &Decoder::decode_OP_DS__DS_PERMUTE_B32,
        &Decoder::decode_OP_DS__DS_BPERMUTE_B32,
        &Decoder::decode_OP_DS__DS_ADD_U64,
        &Decoder::decode_OP_DS__DS_SUB_U64,
        &Decoder::decode_OP_DS__DS_RSUB_U64,
        &Decoder::decode_OP_DS__DS_INC_U64,
        &Decoder::decode_OP_DS__DS_DEC_U64,
        &Decoder::decode_OP_DS__DS_MIN_I64,
        &Decoder::decode_OP_DS__DS_MAX_I64,
        &Decoder::decode_OP_DS__DS_MIN_U64,
        &Decoder::decode_OP_DS__DS_MAX_U64,
        &Decoder::decode_OP_DS__DS_AND_B64,
        &Decoder::decode_OP_DS__DS_OR_B64,
        &Decoder::decode_OP_DS__DS_XOR_B64,
        &Decoder::decode_OP_DS__DS_MSKOR_B64,
        &Decoder::decode_OP_DS__DS_WRITE_B64,
        &Decoder::decode_OP_DS__DS_WRITE2_B64,
        &Decoder::decode_OP_DS__DS_WRITE2ST64_B64,
        &Decoder::decode_OP_DS__DS_CMPST_B64,
        &Decoder::decode_OP_DS__DS_CMPST_F64,
        &Decoder::decode_OP_DS__DS_MIN_F64,
        &Decoder::decode_OP_DS__DS_MAX_F64,
        &Decoder::decode_OP_DS__DS_WRITE_B8_D16_HI,
        &Decoder::decode_OP_DS__DS_WRITE_B16_D16_HI,
        &Decoder::decode_OP_DS__DS_READ_U8_D16,
        &Decoder::decode_OP_DS__DS_READ_U8_D16_HI,
        &Decoder::decode_OP_DS__DS_READ_I8_D16,
        &Decoder::decode_OP_DS__DS_READ_I8_D16_HI,
        &Decoder::decode_OP_DS__DS_READ_U16_D16,
        &Decoder::decode_OP_DS__DS_READ_U16_D16_HI,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_DS__DS_ADD_RTN_U64,
        &Decoder::decode_OP_DS__DS_SUB_RTN_U64,
        &Decoder::decode_OP_DS__DS_RSUB_RTN_U64,
        &Decoder::decode_OP_DS__DS_INC_RTN_U64,
        &Decoder::decode_OP_DS__DS_DEC_RTN_U64,
        &Decoder::decode_OP_DS__DS_MIN_RTN_I64,
        &Decoder::decode_OP_DS__DS_MAX_RTN_I64,
        &Decoder::decode_OP_DS__DS_MIN_RTN_U64,
        &Decoder::decode_OP_DS__DS_MAX_RTN_U64,
        &Decoder::decode_OP_DS__DS_AND_RTN_B64,
        &Decoder::decode_OP_DS__DS_OR_RTN_B64,
        &Decoder::decode_OP_DS__DS_XOR_RTN_B64,
        &Decoder::decode_OP_DS__DS_MSKOR_RTN_B64,
        &Decoder::decode_OP_DS__DS_WRXCHG_RTN_B64,
        &Decoder::decode_OP_DS__DS_WRXCHG2_RTN_B64,
        &Decoder::decode_OP_DS__DS_WRXCHG2ST64_RTN_B64,
        &Decoder::decode_OP_DS__DS_CMPST_RTN_B64,
        &Decoder::decode_OP_DS__DS_CMPST_RTN_F64,
        &Decoder::decode_OP_DS__DS_MIN_RTN_F64,
        &Decoder::decode_OP_DS__DS_MAX_RTN_F64,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_DS__DS_READ_B64,
        &Decoder::decode_OP_DS__DS_READ2_B64,
        &Decoder::decode_OP_DS__DS_READ2ST64_B64,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_DS__DS_CONDXCHG32_RTN_B64,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_DS__DS_ADD_SRC2_U32,
        &Decoder::decode_OP_DS__DS_SUB_SRC2_U32,
        &Decoder::decode_OP_DS__DS_RSUB_SRC2_U32,
        &Decoder::decode_OP_DS__DS_INC_SRC2_U32,
        &Decoder::decode_OP_DS__DS_DEC_SRC2_U32,
        &Decoder::decode_OP_DS__DS_MIN_SRC2_I32,
        &Decoder::decode_OP_DS__DS_MAX_SRC2_I32,
        &Decoder::decode_OP_DS__DS_MIN_SRC2_U32,
        &Decoder::decode_OP_DS__DS_MAX_SRC2_U32,
        &Decoder::decode_OP_DS__DS_AND_SRC2_B32,
        &Decoder::decode_OP_DS__DS_OR_SRC2_B32,
        &Decoder::decode_OP_DS__DS_XOR_SRC2_B32,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_DS__DS_WRITE_SRC2_B32,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_DS__DS_MIN_SRC2_F32,
        &Decoder::decode_OP_DS__DS_MAX_SRC2_F32,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_DS__DS_ADD_SRC2_F32,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_DS__DS_GWS_SEMA_RELEASE_ALL,
        &Decoder::decode_OP_DS__DS_GWS_INIT,
        &Decoder::decode_OP_DS__DS_GWS_SEMA_V,
        &Decoder::decode_OP_DS__DS_GWS_SEMA_BR,
        &Decoder::decode_OP_DS__DS_GWS_SEMA_P,
        &Decoder::decode_OP_DS__DS_GWS_BARRIER,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_DS__DS_READ_ADDTID_B32,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_DS__DS_CONSUME,
        &Decoder::decode_OP_DS__DS_APPEND,
        &Decoder::decode_OP_DS__DS_ORDERED_COUNT,
        &Decoder::decode_OP_DS__DS_ADD_SRC2_U64,
        &Decoder::decode_OP_DS__DS_SUB_SRC2_U64,
        &Decoder::decode_OP_DS__DS_RSUB_SRC2_U64,
        &Decoder::decode_OP_DS__DS_INC_SRC2_U64,
        &Decoder::decode_OP_DS__DS_DEC_SRC2_U64,
        &Decoder::decode_OP_DS__DS_MIN_SRC2_I64,
        &Decoder::decode_OP_DS__DS_MAX_SRC2_I64,
        &Decoder::decode_OP_DS__DS_MIN_SRC2_U64,
        &Decoder::decode_OP_DS__DS_MAX_SRC2_U64,
        &Decoder::decode_OP_DS__DS_AND_SRC2_B64,
        &Decoder::decode_OP_DS__DS_OR_SRC2_B64,
        &Decoder::decode_OP_DS__DS_XOR_SRC2_B64,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_DS__DS_WRITE_SRC2_B64,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_DS__DS_MIN_SRC2_F64,
        &Decoder::decode_OP_DS__DS_MAX_SRC2_F64,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_DS__DS_WRITE_B96,
        &Decoder::decode_OP_DS__DS_WRITE_B128,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_DS__DS_READ_B96,
        &Decoder::decode_OP_DS__DS_READ_B128
    };

    IsaDecodeMethod Decoder::tableSubDecode_OP_FLAT[] = {
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_FLAT__FLAT_LOAD_UBYTE,
        &Decoder::decode_OP_FLAT__FLAT_LOAD_SBYTE,
        &Decoder::decode_OP_FLAT__FLAT_LOAD_USHORT,
        &Decoder::decode_OP_FLAT__FLAT_LOAD_SSHORT,
        &Decoder::decode_OP_FLAT__FLAT_LOAD_DWORD,
        &Decoder::decode_OP_FLAT__FLAT_LOAD_DWORDX2,
        &Decoder::decode_OP_FLAT__FLAT_LOAD_DWORDX3,
        &Decoder::decode_OP_FLAT__FLAT_LOAD_DWORDX4,
        &Decoder::decode_OP_FLAT__FLAT_STORE_BYTE,
        &Decoder::decode_OP_FLAT__FLAT_STORE_BYTE_D16_HI,
        &Decoder::decode_OP_FLAT__FLAT_STORE_SHORT,
        &Decoder::decode_OP_FLAT__FLAT_STORE_SHORT_D16_HI,
        &Decoder::decode_OP_FLAT__FLAT_STORE_DWORD,
        &Decoder::decode_OP_FLAT__FLAT_STORE_DWORDX2,
        &Decoder::decode_OP_FLAT__FLAT_STORE_DWORDX3,
        &Decoder::decode_OP_FLAT__FLAT_STORE_DWORDX4,
        &Decoder::decode_OP_FLAT__FLAT_LOAD_UBYTE_D16,
        &Decoder::decode_OP_FLAT__FLAT_LOAD_UBYTE_D16_HI,
        &Decoder::decode_OP_FLAT__FLAT_LOAD_SBYTE_D16,
        &Decoder::decode_OP_FLAT__FLAT_LOAD_SBYTE_D16_HI,
        &Decoder::decode_OP_FLAT__FLAT_LOAD_SHORT_D16,
        &Decoder::decode_OP_FLAT__FLAT_LOAD_SHORT_D16_HI,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_FLAT__FLAT_ATOMIC_SWAP,
        &Decoder::decode_OP_FLAT__FLAT_ATOMIC_CMPSWAP,
        &Decoder::decode_OP_FLAT__FLAT_ATOMIC_ADD,
        &Decoder::decode_OP_FLAT__FLAT_ATOMIC_SUB,
        &Decoder::decode_OP_FLAT__FLAT_ATOMIC_SMIN,
        &Decoder::decode_OP_FLAT__FLAT_ATOMIC_UMIN,
        &Decoder::decode_OP_FLAT__FLAT_ATOMIC_SMAX,
        &Decoder::decode_OP_FLAT__FLAT_ATOMIC_UMAX,
        &Decoder::decode_OP_FLAT__FLAT_ATOMIC_AND,
        &Decoder::decode_OP_FLAT__FLAT_ATOMIC_OR,
        &Decoder::decode_OP_FLAT__FLAT_ATOMIC_XOR,
        &Decoder::decode_OP_FLAT__FLAT_ATOMIC_INC,
        &Decoder::decode_OP_FLAT__FLAT_ATOMIC_DEC,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_FLAT__FLAT_ATOMIC_ADD_F64,
        &Decoder::decode_OP_FLAT__FLAT_ATOMIC_MIN_F64,
        &Decoder::decode_OP_FLAT__FLAT_ATOMIC_MAX_F64,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_FLAT__FLAT_ATOMIC_SWAP_X2,
        &Decoder::decode_OP_FLAT__FLAT_ATOMIC_CMPSWAP_X2,
        &Decoder::decode_OP_FLAT__FLAT_ATOMIC_ADD_X2,
        &Decoder::decode_OP_FLAT__FLAT_ATOMIC_SUB_X2,
        &Decoder::decode_OP_FLAT__FLAT_ATOMIC_SMIN_X2,
        &Decoder::decode_OP_FLAT__FLAT_ATOMIC_UMIN_X2,
        &Decoder::decode_OP_FLAT__FLAT_ATOMIC_SMAX_X2,
        &Decoder::decode_OP_FLAT__FLAT_ATOMIC_UMAX_X2,
        &Decoder::decode_OP_FLAT__FLAT_ATOMIC_AND_X2,
        &Decoder::decode_OP_FLAT__FLAT_ATOMIC_OR_X2,
        &Decoder::decode_OP_FLAT__FLAT_ATOMIC_XOR_X2,
        &Decoder::decode_OP_FLAT__FLAT_ATOMIC_INC_X2,
        &Decoder::decode_OP_FLAT__FLAT_ATOMIC_DEC_X2,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid
    };

    IsaDecodeMethod Decoder::tableSubDecode_OP_GLOBAL[] = {
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_GLOBAL__GLOBAL_LOAD_UBYTE,
        &Decoder::decode_OP_GLOBAL__GLOBAL_LOAD_SBYTE,
        &Decoder::decode_OP_GLOBAL__GLOBAL_LOAD_USHORT,
        &Decoder::decode_OP_GLOBAL__GLOBAL_LOAD_SSHORT,
        &Decoder::decode_OP_GLOBAL__GLOBAL_LOAD_DWORD,
        &Decoder::decode_OP_GLOBAL__GLOBAL_LOAD_DWORDX2,
        &Decoder::decode_OP_GLOBAL__GLOBAL_LOAD_DWORDX3,
        &Decoder::decode_OP_GLOBAL__GLOBAL_LOAD_DWORDX4,
        &Decoder::decode_OP_GLOBAL__GLOBAL_STORE_BYTE,
        &Decoder::decode_OP_GLOBAL__GLOBAL_STORE_BYTE_D16_HI,
        &Decoder::decode_OP_GLOBAL__GLOBAL_STORE_SHORT,
        &Decoder::decode_OP_GLOBAL__GLOBAL_STORE_SHORT_D16_HI,
        &Decoder::decode_OP_GLOBAL__GLOBAL_STORE_DWORD,
        &Decoder::decode_OP_GLOBAL__GLOBAL_STORE_DWORDX2,
        &Decoder::decode_OP_GLOBAL__GLOBAL_STORE_DWORDX3,
        &Decoder::decode_OP_GLOBAL__GLOBAL_STORE_DWORDX4,
        &Decoder::decode_OP_GLOBAL__GLOBAL_LOAD_UBYTE_D16,
        &Decoder::decode_OP_GLOBAL__GLOBAL_LOAD_UBYTE_D16_HI,
        &Decoder::decode_OP_GLOBAL__GLOBAL_LOAD_SBYTE_D16,
        &Decoder::decode_OP_GLOBAL__GLOBAL_LOAD_SBYTE_D16_HI,
        &Decoder::decode_OP_GLOBAL__GLOBAL_LOAD_SHORT_D16,
        &Decoder::decode_OP_GLOBAL__GLOBAL_LOAD_SHORT_D16_HI,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_SWAP,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_CMPSWAP,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_ADD,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_SUB,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_SMIN,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_UMIN,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_SMAX,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_UMAX,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_AND,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_OR,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_XOR,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_INC,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_DEC,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_ADD_F32,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_ADD_F64,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_MIN_F64,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_MAX_F64,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_SWAP_X2,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_CMPSWAP_X2,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_ADD_X2,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_SUB_X2,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_SMIN_X2,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_UMIN_X2,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_SMAX_X2,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_UMAX_X2,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_AND_X2,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_OR_X2,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_XOR_X2,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_INC_X2,
        &Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_DEC_X2,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid
    };

    IsaDecodeMethod Decoder::tableSubDecode_OP_MIMG[] = {
        &Decoder::decode_OP_MIMG__IMAGE_LOAD,
        &Decoder::decode_OP_MIMG__IMAGE_LOAD_MIP,
        &Decoder::decode_OP_MIMG__IMAGE_LOAD_PCK,
        &Decoder::decode_OP_MIMG__IMAGE_LOAD_PCK_SGN,
        &Decoder::decode_OP_MIMG__IMAGE_LOAD_MIP_PCK,
        &Decoder::decode_OP_MIMG__IMAGE_LOAD_MIP_PCK_SGN,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_MIMG__IMAGE_STORE,
        &Decoder::decode_OP_MIMG__IMAGE_STORE_MIP,
        &Decoder::decode_OP_MIMG__IMAGE_STORE_PCK,
        &Decoder::decode_OP_MIMG__IMAGE_STORE_MIP_PCK,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_MIMG__IMAGE_GET_RESINFO,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_MIMG__IMAGE_ATOMIC_SWAP,
        &Decoder::decode_OP_MIMG__IMAGE_ATOMIC_CMPSWAP,
        &Decoder::decode_OP_MIMG__IMAGE_ATOMIC_ADD,
        &Decoder::decode_OP_MIMG__IMAGE_ATOMIC_SUB,
        &Decoder::decode_OP_MIMG__IMAGE_ATOMIC_SMIN,
        &Decoder::decode_OP_MIMG__IMAGE_ATOMIC_UMIN,
        &Decoder::decode_OP_MIMG__IMAGE_ATOMIC_SMAX,
        &Decoder::decode_OP_MIMG__IMAGE_ATOMIC_UMAX,
        &Decoder::decode_OP_MIMG__IMAGE_ATOMIC_AND,
        &Decoder::decode_OP_MIMG__IMAGE_ATOMIC_OR,
        &Decoder::decode_OP_MIMG__IMAGE_ATOMIC_XOR,
        &Decoder::decode_OP_MIMG__IMAGE_ATOMIC_INC,
        &Decoder::decode_OP_MIMG__IMAGE_ATOMIC_DEC,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_CL,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_D,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_D_CL,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_L,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_B,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_B_CL,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_LZ,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_CL,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_D,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_D_CL,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_L,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_B,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_B_CL,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_LZ,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_O,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_CL_O,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_D_O,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_D_CL_O,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_L_O,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_B_O,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_B_CL_O,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_LZ_O,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_O,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_CL_O,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_D_O,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_D_CL_O,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_L_O,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_B_O,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_B_CL_O,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_LZ_O,
        &Decoder::decode_OP_MIMG__IMAGE_GATHER4,
        &Decoder::decode_OP_MIMG__IMAGE_GATHER4_CL,
        &Decoder::decode_OP_MIMG__IMAGE_GATHER4H,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_MIMG__IMAGE_GATHER4_L,
        &Decoder::decode_OP_MIMG__IMAGE_GATHER4_B,
        &Decoder::decode_OP_MIMG__IMAGE_GATHER4_B_CL,
        &Decoder::decode_OP_MIMG__IMAGE_GATHER4_LZ,
        &Decoder::decode_OP_MIMG__IMAGE_GATHER4_C,
        &Decoder::decode_OP_MIMG__IMAGE_GATHER4_C_CL,
        &Decoder::decode_OP_MIMG__IMAGE_GATHER4H_PCK,
        &Decoder::decode_OP_MIMG__IMAGE_GATHER8H_PCK,
        &Decoder::decode_OP_MIMG__IMAGE_GATHER4_C_L,
        &Decoder::decode_OP_MIMG__IMAGE_GATHER4_C_B,
        &Decoder::decode_OP_MIMG__IMAGE_GATHER4_C_B_CL,
        &Decoder::decode_OP_MIMG__IMAGE_GATHER4_C_LZ,
        &Decoder::decode_OP_MIMG__IMAGE_GATHER4_O,
        &Decoder::decode_OP_MIMG__IMAGE_GATHER4_CL_O,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_MIMG__IMAGE_GATHER4_L_O,
        &Decoder::decode_OP_MIMG__IMAGE_GATHER4_B_O,
        &Decoder::decode_OP_MIMG__IMAGE_GATHER4_B_CL_O,
        &Decoder::decode_OP_MIMG__IMAGE_GATHER4_LZ_O,
        &Decoder::decode_OP_MIMG__IMAGE_GATHER4_C_O,
        &Decoder::decode_OP_MIMG__IMAGE_GATHER4_C_CL_O,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_MIMG__IMAGE_GATHER4_C_L_O,
        &Decoder::decode_OP_MIMG__IMAGE_GATHER4_C_B_O,
        &Decoder::decode_OP_MIMG__IMAGE_GATHER4_C_B_CL_O,
        &Decoder::decode_OP_MIMG__IMAGE_GATHER4_C_LZ_O,
        &Decoder::decode_OP_MIMG__IMAGE_GET_LOD,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_CD,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_CD_CL,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_CD,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_CD_CL,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_CD_O,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_CD_CL_O,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_CD_O,
        &Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_CD_CL_O,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid
    };

    IsaDecodeMethod Decoder::tableSubDecode_OP_MTBUF[] = {
        &Decoder::decode_OP_MTBUF__TBUFFER_LOAD_FORMAT_X,
        &Decoder::decode_OP_MTBUF__TBUFFER_LOAD_FORMAT_XY,
        &Decoder::decode_OP_MTBUF__TBUFFER_LOAD_FORMAT_XYZ,
        &Decoder::decode_OP_MTBUF__TBUFFER_LOAD_FORMAT_XYZW,
        &Decoder::decode_OP_MTBUF__TBUFFER_STORE_FORMAT_X,
        &Decoder::decode_OP_MTBUF__TBUFFER_STORE_FORMAT_XY,
        &Decoder::decode_OP_MTBUF__TBUFFER_STORE_FORMAT_XYZ,
        &Decoder::decode_OP_MTBUF__TBUFFER_STORE_FORMAT_XYZW,
        &Decoder::decode_OP_MTBUF__TBUFFER_LOAD_FORMAT_D16_X,
        &Decoder::decode_OP_MTBUF__TBUFFER_LOAD_FORMAT_D16_XY,
        &Decoder::decode_OP_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZ,
        &Decoder::decode_OP_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZW,
        &Decoder::decode_OP_MTBUF__TBUFFER_STORE_FORMAT_D16_X,
        &Decoder::decode_OP_MTBUF__TBUFFER_STORE_FORMAT_D16_XY,
        &Decoder::decode_OP_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZ,
        &Decoder::decode_OP_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZW
    };

    IsaDecodeMethod Decoder::tableSubDecode_OP_MUBUF[] = {
        &Decoder::decode_OP_MUBUF__BUFFER_LOAD_FORMAT_X,
        &Decoder::decode_OP_MUBUF__BUFFER_LOAD_FORMAT_XY,
        &Decoder::decode_OP_MUBUF__BUFFER_LOAD_FORMAT_XYZ,
        &Decoder::decode_OP_MUBUF__BUFFER_LOAD_FORMAT_XYZW,
        &Decoder::decode_OP_MUBUF__BUFFER_STORE_FORMAT_X,
        &Decoder::decode_OP_MUBUF__BUFFER_STORE_FORMAT_XY,
        &Decoder::decode_OP_MUBUF__BUFFER_STORE_FORMAT_XYZ,
        &Decoder::decode_OP_MUBUF__BUFFER_STORE_FORMAT_XYZW,
        &Decoder::decode_OP_MUBUF__BUFFER_LOAD_FORMAT_D16_X,
        &Decoder::decode_OP_MUBUF__BUFFER_LOAD_FORMAT_D16_XY,
        &Decoder::decode_OP_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZ,
        &Decoder::decode_OP_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZW,
        &Decoder::decode_OP_MUBUF__BUFFER_STORE_FORMAT_D16_X,
        &Decoder::decode_OP_MUBUF__BUFFER_STORE_FORMAT_D16_XY,
        &Decoder::decode_OP_MUBUF__BUFFER_STORE_FORMAT_D16_XYZ,
        &Decoder::decode_OP_MUBUF__BUFFER_STORE_FORMAT_D16_XYZW,
        &Decoder::decode_OP_MUBUF__BUFFER_LOAD_UBYTE,
        &Decoder::decode_OP_MUBUF__BUFFER_LOAD_SBYTE,
        &Decoder::decode_OP_MUBUF__BUFFER_LOAD_USHORT,
        &Decoder::decode_OP_MUBUF__BUFFER_LOAD_SSHORT,
        &Decoder::decode_OP_MUBUF__BUFFER_LOAD_DWORD,
        &Decoder::decode_OP_MUBUF__BUFFER_LOAD_DWORDX2,
        &Decoder::decode_OP_MUBUF__BUFFER_LOAD_DWORDX3,
        &Decoder::decode_OP_MUBUF__BUFFER_LOAD_DWORDX4,
        &Decoder::decode_OP_MUBUF__BUFFER_STORE_BYTE,
        &Decoder::decode_OP_MUBUF__BUFFER_STORE_BYTE_D16_HI,
        &Decoder::decode_OP_MUBUF__BUFFER_STORE_SHORT,
        &Decoder::decode_OP_MUBUF__BUFFER_STORE_SHORT_D16_HI,
        &Decoder::decode_OP_MUBUF__BUFFER_STORE_DWORD,
        &Decoder::decode_OP_MUBUF__BUFFER_STORE_DWORDX2,
        &Decoder::decode_OP_MUBUF__BUFFER_STORE_DWORDX3,
        &Decoder::decode_OP_MUBUF__BUFFER_STORE_DWORDX4,
        &Decoder::decode_OP_MUBUF__BUFFER_LOAD_UBYTE_D16,
        &Decoder::decode_OP_MUBUF__BUFFER_LOAD_UBYTE_D16_HI,
        &Decoder::decode_OP_MUBUF__BUFFER_LOAD_SBYTE_D16,
        &Decoder::decode_OP_MUBUF__BUFFER_LOAD_SBYTE_D16_HI,
        &Decoder::decode_OP_MUBUF__BUFFER_LOAD_SHORT_D16,
        &Decoder::decode_OP_MUBUF__BUFFER_LOAD_SHORT_D16_HI,
        &Decoder::decode_OP_MUBUF__BUFFER_LOAD_FORMAT_D16_HI_X,
        &Decoder::decode_OP_MUBUF__BUFFER_STORE_FORMAT_D16_HI_X,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_MUBUF__BUFFER_STORE_LDS_DWORD,
        &Decoder::decode_OP_MUBUF__BUFFER_WBINVL1,
        &Decoder::decode_OP_MUBUF__BUFFER_WBINVL1_VOL,
        &Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_SWAP,
        &Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_CMPSWAP,
        &Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_ADD,
        &Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_SUB,
        &Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_SMIN,
        &Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_UMIN,
        &Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_SMAX,
        &Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_UMAX,
        &Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_AND,
        &Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_OR,
        &Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_XOR,
        &Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_INC,
        &Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_DEC,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_SWAP_X2,
        &Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_CMPSWAP_X2,
        &Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_ADD_X2,
        &Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_SUB_X2,
        &Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_SMIN_X2,
        &Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_UMIN_X2,
        &Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_SMAX_X2,
        &Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_UMAX_X2,
        &Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_AND_X2,
        &Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_OR_X2,
        &Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_XOR_X2,
        &Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_INC_X2,
        &Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_DEC_X2,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid
    };

    IsaDecodeMethod Decoder::tableSubDecode_OP_SCRATCH[] = {
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_SCRATCH__SCRATCH_LOAD_UBYTE,
        &Decoder::decode_OP_SCRATCH__SCRATCH_LOAD_SBYTE,
        &Decoder::decode_OP_SCRATCH__SCRATCH_LOAD_USHORT,
        &Decoder::decode_OP_SCRATCH__SCRATCH_LOAD_SSHORT,
        &Decoder::decode_OP_SCRATCH__SCRATCH_LOAD_DWORD,
        &Decoder::decode_OP_SCRATCH__SCRATCH_LOAD_DWORDX2,
        &Decoder::decode_OP_SCRATCH__SCRATCH_LOAD_DWORDX3,
        &Decoder::decode_OP_SCRATCH__SCRATCH_LOAD_DWORDX4,
        &Decoder::decode_OP_SCRATCH__SCRATCH_STORE_BYTE,
        &Decoder::decode_OP_SCRATCH__SCRATCH_STORE_BYTE_D16_HI,
        &Decoder::decode_OP_SCRATCH__SCRATCH_STORE_SHORT,
        &Decoder::decode_OP_SCRATCH__SCRATCH_STORE_SHORT_D16_HI,
        &Decoder::decode_OP_SCRATCH__SCRATCH_STORE_DWORD,
        &Decoder::decode_OP_SCRATCH__SCRATCH_STORE_DWORDX2,
        &Decoder::decode_OP_SCRATCH__SCRATCH_STORE_DWORDX3,
        &Decoder::decode_OP_SCRATCH__SCRATCH_STORE_DWORDX4,
        &Decoder::decode_OP_SCRATCH__SCRATCH_LOAD_UBYTE_D16,
        &Decoder::decode_OP_SCRATCH__SCRATCH_LOAD_UBYTE_D16_HI,
        &Decoder::decode_OP_SCRATCH__SCRATCH_LOAD_SBYTE_D16,
        &Decoder::decode_OP_SCRATCH__SCRATCH_LOAD_SBYTE_D16_HI,
        &Decoder::decode_OP_SCRATCH__SCRATCH_LOAD_SHORT_D16,
        &Decoder::decode_OP_SCRATCH__SCRATCH_LOAD_SHORT_D16_HI,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
    };

    IsaDecodeMethod Decoder::tableSubDecode_OP_SMEM[] = {
        &Decoder::decode_OP_SMEM__S_LOAD_DWORD,
        &Decoder::decode_OP_SMEM__S_LOAD_DWORDX2,
        &Decoder::decode_OP_SMEM__S_LOAD_DWORDX4,
        &Decoder::decode_OP_SMEM__S_LOAD_DWORDX8,
        &Decoder::decode_OP_SMEM__S_LOAD_DWORDX16,
        &Decoder::decode_OP_SMEM__S_SCRATCH_LOAD_DWORD,
        &Decoder::decode_OP_SMEM__S_SCRATCH_LOAD_DWORDX2,
        &Decoder::decode_OP_SMEM__S_SCRATCH_LOAD_DWORDX4,
        &Decoder::decode_OP_SMEM__S_BUFFER_LOAD_DWORD,
        &Decoder::decode_OP_SMEM__S_BUFFER_LOAD_DWORDX2,
        &Decoder::decode_OP_SMEM__S_BUFFER_LOAD_DWORDX4,
        &Decoder::decode_OP_SMEM__S_BUFFER_LOAD_DWORDX8,
        &Decoder::decode_OP_SMEM__S_BUFFER_LOAD_DWORDX16,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_SMEM__S_STORE_DWORD,
        &Decoder::decode_OP_SMEM__S_STORE_DWORDX2,
        &Decoder::decode_OP_SMEM__S_STORE_DWORDX4,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_SMEM__S_SCRATCH_STORE_DWORD,
        &Decoder::decode_OP_SMEM__S_SCRATCH_STORE_DWORDX2,
        &Decoder::decode_OP_SMEM__S_SCRATCH_STORE_DWORDX4,
        &Decoder::decode_OP_SMEM__S_BUFFER_STORE_DWORD,
        &Decoder::decode_OP_SMEM__S_BUFFER_STORE_DWORDX2,
        &Decoder::decode_OP_SMEM__S_BUFFER_STORE_DWORDX4,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_SMEM__S_DCACHE_INV,
        &Decoder::decode_OP_SMEM__S_DCACHE_WB,
        &Decoder::decode_OP_SMEM__S_DCACHE_INV_VOL,
        &Decoder::decode_OP_SMEM__S_DCACHE_WB_VOL,
        &Decoder::decode_OP_SMEM__S_MEMTIME,
        &Decoder::decode_OP_SMEM__S_MEMREALTIME,
        &Decoder::decode_OP_SMEM__S_ATC_PROBE,
        &Decoder::decode_OP_SMEM__S_ATC_PROBE_BUFFER,
        &Decoder::decode_OP_SMEM__S_DCACHE_DISCARD,
        &Decoder::decode_OP_SMEM__S_DCACHE_DISCARD_X2,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_SWAP,
        &Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_CMPSWAP,
        &Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_ADD,
        &Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_SUB,
        &Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_SMIN,
        &Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_UMIN,
        &Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_SMAX,
        &Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_UMAX,
        &Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_AND,
        &Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_OR,
        &Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_XOR,
        &Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_INC,
        &Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_DEC,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_SWAP_X2,
        &Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_CMPSWAP_X2,
        &Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_ADD_X2,
        &Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_SUB_X2,
        &Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_SMIN_X2,
        &Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_UMIN_X2,
        &Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_SMAX_X2,
        &Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_UMAX_X2,
        &Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_AND_X2,
        &Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_OR_X2,
        &Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_XOR_X2,
        &Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_INC_X2,
        &Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_DEC_X2,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_SMEM__S_ATOMIC_SWAP,
        &Decoder::decode_OP_SMEM__S_ATOMIC_CMPSWAP,
        &Decoder::decode_OP_SMEM__S_ATOMIC_ADD,
        &Decoder::decode_OP_SMEM__S_ATOMIC_SUB,
        &Decoder::decode_OP_SMEM__S_ATOMIC_SMIN,
        &Decoder::decode_OP_SMEM__S_ATOMIC_UMIN,
        &Decoder::decode_OP_SMEM__S_ATOMIC_SMAX,
        &Decoder::decode_OP_SMEM__S_ATOMIC_UMAX,
        &Decoder::decode_OP_SMEM__S_ATOMIC_AND,
        &Decoder::decode_OP_SMEM__S_ATOMIC_OR,
        &Decoder::decode_OP_SMEM__S_ATOMIC_XOR,
        &Decoder::decode_OP_SMEM__S_ATOMIC_INC,
        &Decoder::decode_OP_SMEM__S_ATOMIC_DEC,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_SMEM__S_ATOMIC_SWAP_X2,
        &Decoder::decode_OP_SMEM__S_ATOMIC_CMPSWAP_X2,
        &Decoder::decode_OP_SMEM__S_ATOMIC_ADD_X2,
        &Decoder::decode_OP_SMEM__S_ATOMIC_SUB_X2,
        &Decoder::decode_OP_SMEM__S_ATOMIC_SMIN_X2,
        &Decoder::decode_OP_SMEM__S_ATOMIC_UMIN_X2,
        &Decoder::decode_OP_SMEM__S_ATOMIC_SMAX_X2,
        &Decoder::decode_OP_SMEM__S_ATOMIC_UMAX_X2,
        &Decoder::decode_OP_SMEM__S_ATOMIC_AND_X2,
        &Decoder::decode_OP_SMEM__S_ATOMIC_OR_X2,
        &Decoder::decode_OP_SMEM__S_ATOMIC_XOR_X2,
        &Decoder::decode_OP_SMEM__S_ATOMIC_INC_X2,
        &Decoder::decode_OP_SMEM__S_ATOMIC_DEC_X2,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
    };

    IsaDecodeMethod Decoder::tableSubDecode_OP_SOP1[] = {
        &Decoder::decode_OP_SOP1__S_MOV_B32,
        &Decoder::decode_OP_SOP1__S_MOV_B64,
        &Decoder::decode_OP_SOP1__S_CMOV_B32,
        &Decoder::decode_OP_SOP1__S_CMOV_B64,
        &Decoder::decode_OP_SOP1__S_NOT_B32,
        &Decoder::decode_OP_SOP1__S_NOT_B64,
        &Decoder::decode_OP_SOP1__S_WQM_B32,
        &Decoder::decode_OP_SOP1__S_WQM_B64,
        &Decoder::decode_OP_SOP1__S_BREV_B32,
        &Decoder::decode_OP_SOP1__S_BREV_B64,
        &Decoder::decode_OP_SOP1__S_BCNT0_I32_B32,
        &Decoder::decode_OP_SOP1__S_BCNT0_I32_B64,
        &Decoder::decode_OP_SOP1__S_BCNT1_I32_B32,
        &Decoder::decode_OP_SOP1__S_BCNT1_I32_B64,
        &Decoder::decode_OP_SOP1__S_FF0_I32_B32,
        &Decoder::decode_OP_SOP1__S_FF0_I32_B64,
        &Decoder::decode_OP_SOP1__S_FF1_I32_B32,
        &Decoder::decode_OP_SOP1__S_FF1_I32_B64,
        &Decoder::decode_OP_SOP1__S_FLBIT_I32_B32,
        &Decoder::decode_OP_SOP1__S_FLBIT_I32_B64,
        &Decoder::decode_OP_SOP1__S_FLBIT_I32,
        &Decoder::decode_OP_SOP1__S_FLBIT_I32_I64,
        &Decoder::decode_OP_SOP1__S_SEXT_I32_I8,
        &Decoder::decode_OP_SOP1__S_SEXT_I32_I16,
        &Decoder::decode_OP_SOP1__S_BITSET0_B32,
        &Decoder::decode_OP_SOP1__S_BITSET0_B64,
        &Decoder::decode_OP_SOP1__S_BITSET1_B32,
        &Decoder::decode_OP_SOP1__S_BITSET1_B64,
        &Decoder::decode_OP_SOP1__S_GETPC_B64,
        &Decoder::decode_OP_SOP1__S_SETPC_B64,
        &Decoder::decode_OP_SOP1__S_SWAPPC_B64,
        &Decoder::decode_OP_SOP1__S_RFE_B64,
        &Decoder::decode_OP_SOP1__S_AND_SAVEEXEC_B64,
        &Decoder::decode_OP_SOP1__S_OR_SAVEEXEC_B64,
        &Decoder::decode_OP_SOP1__S_XOR_SAVEEXEC_B64,
        &Decoder::decode_OP_SOP1__S_ANDN2_SAVEEXEC_B64,
        &Decoder::decode_OP_SOP1__S_ORN2_SAVEEXEC_B64,
        &Decoder::decode_OP_SOP1__S_NAND_SAVEEXEC_B64,
        &Decoder::decode_OP_SOP1__S_NOR_SAVEEXEC_B64,
        &Decoder::decode_OP_SOP1__S_XNOR_SAVEEXEC_B64,
        &Decoder::decode_OP_SOP1__S_QUADMASK_B32,
        &Decoder::decode_OP_SOP1__S_QUADMASK_B64,
        &Decoder::decode_OP_SOP1__S_MOVRELS_B32,
        &Decoder::decode_OP_SOP1__S_MOVRELS_B64,
        &Decoder::decode_OP_SOP1__S_MOVRELD_B32,
        &Decoder::decode_OP_SOP1__S_MOVRELD_B64,
        &Decoder::decode_OP_SOP1__S_CBRANCH_JOIN,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_SOP1__S_ABS_I32,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_SOP1__S_SET_GPR_IDX_IDX,
        &Decoder::decode_OP_SOP1__S_ANDN1_SAVEEXEC_B64,
        &Decoder::decode_OP_SOP1__S_ORN1_SAVEEXEC_B64,
        &Decoder::decode_OP_SOP1__S_ANDN1_WREXEC_B64,
        &Decoder::decode_OP_SOP1__S_ANDN2_WREXEC_B64,
        &Decoder::decode_OP_SOP1__S_BITREPLICATE_B64_B32,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid
    };

    IsaDecodeMethod Decoder::tableSubDecode_OP_SOPC[] = {
        &Decoder::decode_OP_SOPC__S_CMP_EQ_I32,
        &Decoder::decode_OP_SOPC__S_CMP_LG_I32,
        &Decoder::decode_OP_SOPC__S_CMP_GT_I32,
        &Decoder::decode_OP_SOPC__S_CMP_GE_I32,
        &Decoder::decode_OP_SOPC__S_CMP_LT_I32,
        &Decoder::decode_OP_SOPC__S_CMP_LE_I32,
        &Decoder::decode_OP_SOPC__S_CMP_EQ_U32,
        &Decoder::decode_OP_SOPC__S_CMP_LG_U32,
        &Decoder::decode_OP_SOPC__S_CMP_GT_U32,
        &Decoder::decode_OP_SOPC__S_CMP_GE_U32,
        &Decoder::decode_OP_SOPC__S_CMP_LT_U32,
        &Decoder::decode_OP_SOPC__S_CMP_LE_U32,
        &Decoder::decode_OP_SOPC__S_BITCMP0_B32,
        &Decoder::decode_OP_SOPC__S_BITCMP1_B32,
        &Decoder::decode_OP_SOPC__S_BITCMP0_B64,
        &Decoder::decode_OP_SOPC__S_BITCMP1_B64,
        &Decoder::decode_OP_SOPC__S_SETVSKIP,
        &Decoder::decode_OP_SOPC__S_SET_GPR_IDX_ON,
        &Decoder::decode_OP_SOPC__S_CMP_EQ_U64,
        &Decoder::decode_OP_SOPC__S_CMP_LG_U64,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid
    };

    IsaDecodeMethod Decoder::tableSubDecode_OP_SOPP[] = {
        &Decoder::decode_OP_SOPP__S_NOP,
        &Decoder::decode_OP_SOPP__S_ENDPGM,
        &Decoder::decode_OP_SOPP__S_BRANCH,
        &Decoder::decode_OP_SOPP__S_WAKEUP,
        &Decoder::decode_OP_SOPP__S_CBRANCH_SCC0,
        &Decoder::decode_OP_SOPP__S_CBRANCH_SCC1,
        &Decoder::decode_OP_SOPP__S_CBRANCH_VCCZ,
        &Decoder::decode_OP_SOPP__S_CBRANCH_VCCNZ,
        &Decoder::decode_OP_SOPP__S_CBRANCH_EXECZ,
        &Decoder::decode_OP_SOPP__S_CBRANCH_EXECNZ,
        &Decoder::decode_OP_SOPP__S_BARRIER,
        &Decoder::decode_OP_SOPP__S_SETKILL,
        &Decoder::decode_OP_SOPP__S_WAITCNT,
        &Decoder::decode_OP_SOPP__S_SETHALT,
        &Decoder::decode_OP_SOPP__S_SLEEP,
        &Decoder::decode_OP_SOPP__S_SETPRIO,
        &Decoder::decode_OP_SOPP__S_SENDMSG,
        &Decoder::decode_OP_SOPP__S_SENDMSGHALT,
        &Decoder::decode_OP_SOPP__S_TRAP,
        &Decoder::decode_OP_SOPP__S_ICACHE_INV,
        &Decoder::decode_OP_SOPP__S_INCPERFLEVEL,
        &Decoder::decode_OP_SOPP__S_DECPERFLEVEL,
        &Decoder::decode_OP_SOPP__S_TTRACEDATA,
        &Decoder::decode_OP_SOPP__S_CBRANCH_CDBGSYS,
        &Decoder::decode_OP_SOPP__S_CBRANCH_CDBGUSER,
        &Decoder::decode_OP_SOPP__S_CBRANCH_CDBGSYS_OR_USER,
        &Decoder::decode_OP_SOPP__S_CBRANCH_CDBGSYS_AND_USER,
        &Decoder::decode_OP_SOPP__S_ENDPGM_SAVED,
        &Decoder::decode_OP_SOPP__S_SET_GPR_IDX_OFF,
        &Decoder::decode_OP_SOPP__S_SET_GPR_IDX_MODE,
        &Decoder::decode_OP_SOPP__S_ENDPGM_ORDERED_PS_DONE,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid
    };

    IsaDecodeMethod Decoder::tableSubDecode_OP_VINTRP[] = {
        &Decoder::decode_OP_VINTRP__V_INTERP_P1_F32,
        &Decoder::decode_OP_VINTRP__V_INTERP_P2_F32,
        &Decoder::decode_OP_VINTRP__V_INTERP_MOV_F32,
        &Decoder::decode_invalid
    };

    IsaDecodeMethod Decoder::tableSubDecode_OP_VOP1[] = {
        &Decoder::decode_OP_VOP1__V_NOP,
        &Decoder::decode_OP_VOP1__V_MOV_B32,
        &Decoder::decode_OP_VOP1__V_READFIRSTLANE_B32,
        &Decoder::decode_OP_VOP1__V_CVT_I32_F64,
        &Decoder::decode_OP_VOP1__V_CVT_F64_I32,
        &Decoder::decode_OP_VOP1__V_CVT_F32_I32,
        &Decoder::decode_OP_VOP1__V_CVT_F32_U32,
        &Decoder::decode_OP_VOP1__V_CVT_U32_F32,
        &Decoder::decode_OP_VOP1__V_CVT_I32_F32,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_VOP1__V_CVT_F16_F32,
        &Decoder::decode_OP_VOP1__V_CVT_F32_F16,
        &Decoder::decode_OP_VOP1__V_CVT_RPI_I32_F32,
        &Decoder::decode_OP_VOP1__V_CVT_FLR_I32_F32,
        &Decoder::decode_OP_VOP1__V_CVT_OFF_F32_I4,
        &Decoder::decode_OP_VOP1__V_CVT_F32_F64,
        &Decoder::decode_OP_VOP1__V_CVT_F64_F32,
        &Decoder::decode_OP_VOP1__V_CVT_F32_UBYTE0,
        &Decoder::decode_OP_VOP1__V_CVT_F32_UBYTE1,
        &Decoder::decode_OP_VOP1__V_CVT_F32_UBYTE2,
        &Decoder::decode_OP_VOP1__V_CVT_F32_UBYTE3,
        &Decoder::decode_OP_VOP1__V_CVT_U32_F64,
        &Decoder::decode_OP_VOP1__V_CVT_F64_U32,
        &Decoder::decode_OP_VOP1__V_TRUNC_F64,
        &Decoder::decode_OP_VOP1__V_CEIL_F64,
        &Decoder::decode_OP_VOP1__V_RNDNE_F64,
        &Decoder::decode_OP_VOP1__V_FLOOR_F64,
        &Decoder::decode_OP_VOP1__V_FRACT_F32,
        &Decoder::decode_OP_VOP1__V_TRUNC_F32,
        &Decoder::decode_OP_VOP1__V_CEIL_F32,
        &Decoder::decode_OP_VOP1__V_RNDNE_F32,
        &Decoder::decode_OP_VOP1__V_FLOOR_F32,
        &Decoder::decode_OP_VOP1__V_EXP_F32,
        &Decoder::decode_OP_VOP1__V_LOG_F32,
        &Decoder::decode_OP_VOP1__V_RCP_F32,
        &Decoder::decode_OP_VOP1__V_RCP_IFLAG_F32,
        &Decoder::decode_OP_VOP1__V_RSQ_F32,
        &Decoder::decode_OP_VOP1__V_RCP_F64,
        &Decoder::decode_OP_VOP1__V_RSQ_F64,
        &Decoder::decode_OP_VOP1__V_SQRT_F32,
        &Decoder::decode_OP_VOP1__V_SQRT_F64,
        &Decoder::decode_OP_VOP1__V_SIN_F32,
        &Decoder::decode_OP_VOP1__V_COS_F32,
        &Decoder::decode_OP_VOP1__V_NOT_B32,
        &Decoder::decode_OP_VOP1__V_BFREV_B32,
        &Decoder::decode_OP_VOP1__V_FFBH_U32,
        &Decoder::decode_OP_VOP1__V_FFBL_B32,
        &Decoder::decode_OP_VOP1__V_FFBH_I32,
        &Decoder::decode_OP_VOP1__V_FREXP_EXP_I32_F64,
        &Decoder::decode_OP_VOP1__V_FREXP_MANT_F64,
        &Decoder::decode_OP_VOP1__V_FRACT_F64,
        &Decoder::decode_OP_VOP1__V_FREXP_EXP_I32_F32,
        &Decoder::decode_OP_VOP1__V_FREXP_MANT_F32,
        &Decoder::decode_OP_VOP1__V_CLREXCP,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_VOP1__V_SCREEN_PARTITION_4SE_B32,
        &Decoder::decode_OP_VOP1__V_MOV_B64,
        &Decoder::decode_OP_VOP1__V_CVT_F16_U16,
        &Decoder::decode_OP_VOP1__V_CVT_F16_I16,
        &Decoder::decode_OP_VOP1__V_CVT_U16_F16,
        &Decoder::decode_OP_VOP1__V_CVT_I16_F16,
        &Decoder::decode_OP_VOP1__V_RCP_F16,
        &Decoder::decode_OP_VOP1__V_SQRT_F16,
        &Decoder::decode_OP_VOP1__V_RSQ_F16,
        &Decoder::decode_OP_VOP1__V_LOG_F16,
        &Decoder::decode_OP_VOP1__V_EXP_F16,
        &Decoder::decode_OP_VOP1__V_FREXP_MANT_F16,
        &Decoder::decode_OP_VOP1__V_FREXP_EXP_I16_F16,
        &Decoder::decode_OP_VOP1__V_FLOOR_F16,
        &Decoder::decode_OP_VOP1__V_CEIL_F16,
        &Decoder::decode_OP_VOP1__V_TRUNC_F16,
        &Decoder::decode_OP_VOP1__V_RNDNE_F16,
        &Decoder::decode_OP_VOP1__V_FRACT_F16,
        &Decoder::decode_OP_VOP1__V_SIN_F16,
        &Decoder::decode_OP_VOP1__V_COS_F16,
        &Decoder::decode_OP_VOP1__V_EXP_LEGACY_F32,
        &Decoder::decode_OP_VOP1__V_LOG_LEGACY_F32,
        &Decoder::decode_OP_VOP1__V_CVT_NORM_I16_F16,
        &Decoder::decode_OP_VOP1__V_CVT_NORM_U16_F16,
        &Decoder::decode_OP_VOP1__V_SAT_PK_U8_I16,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_VOP1__V_SWAP_B32,
        &Decoder::decode_OP_VOP1__V_ACCVGPR_MOV_B32,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid
    };

    IsaDecodeMethod Decoder::tableSubDecode_OP_VOPC[] = {
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_VOPC__V_CMP_CLASS_F32,
        &Decoder::decode_OP_VOPC__V_CMPX_CLASS_F32,
        &Decoder::decode_OP_VOPC__V_CMP_CLASS_F64,
        &Decoder::decode_OP_VOPC__V_CMPX_CLASS_F64,
        &Decoder::decode_OP_VOPC__V_CMP_CLASS_F16,
        &Decoder::decode_OP_VOPC__V_CMPX_CLASS_F16,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_VOPC__V_CMP_F_F16,
        &Decoder::decode_OP_VOPC__V_CMP_LT_F16,
        &Decoder::decode_OP_VOPC__V_CMP_EQ_F16,
        &Decoder::decode_OP_VOPC__V_CMP_LE_F16,
        &Decoder::decode_OP_VOPC__V_CMP_GT_F16,
        &Decoder::decode_OP_VOPC__V_CMP_LG_F16,
        &Decoder::decode_OP_VOPC__V_CMP_GE_F16,
        &Decoder::decode_OP_VOPC__V_CMP_O_F16,
        &Decoder::decode_OP_VOPC__V_CMP_U_F16,
        &Decoder::decode_OP_VOPC__V_CMP_NGE_F16,
        &Decoder::decode_OP_VOPC__V_CMP_NLG_F16,
        &Decoder::decode_OP_VOPC__V_CMP_NGT_F16,
        &Decoder::decode_OP_VOPC__V_CMP_NLE_F16,
        &Decoder::decode_OP_VOPC__V_CMP_NEQ_F16,
        &Decoder::decode_OP_VOPC__V_CMP_NLT_F16,
        &Decoder::decode_OP_VOPC__V_CMP_TRU_F16,
        &Decoder::decode_OP_VOPC__V_CMPX_F_F16,
        &Decoder::decode_OP_VOPC__V_CMPX_LT_F16,
        &Decoder::decode_OP_VOPC__V_CMPX_EQ_F16,
        &Decoder::decode_OP_VOPC__V_CMPX_LE_F16,
        &Decoder::decode_OP_VOPC__V_CMPX_GT_F16,
        &Decoder::decode_OP_VOPC__V_CMPX_LG_F16,
        &Decoder::decode_OP_VOPC__V_CMPX_GE_F16,
        &Decoder::decode_OP_VOPC__V_CMPX_O_F16,
        &Decoder::decode_OP_VOPC__V_CMPX_U_F16,
        &Decoder::decode_OP_VOPC__V_CMPX_NGE_F16,
        &Decoder::decode_OP_VOPC__V_CMPX_NLG_F16,
        &Decoder::decode_OP_VOPC__V_CMPX_NGT_F16,
        &Decoder::decode_OP_VOPC__V_CMPX_NLE_F16,
        &Decoder::decode_OP_VOPC__V_CMPX_NEQ_F16,
        &Decoder::decode_OP_VOPC__V_CMPX_NLT_F16,
        &Decoder::decode_OP_VOPC__V_CMPX_TRU_F16,
        &Decoder::decode_OP_VOPC__V_CMP_F_F32,
        &Decoder::decode_OP_VOPC__V_CMP_LT_F32,
        &Decoder::decode_OP_VOPC__V_CMP_EQ_F32,
        &Decoder::decode_OP_VOPC__V_CMP_LE_F32,
        &Decoder::decode_OP_VOPC__V_CMP_GT_F32,
        &Decoder::decode_OP_VOPC__V_CMP_LG_F32,
        &Decoder::decode_OP_VOPC__V_CMP_GE_F32,
        &Decoder::decode_OP_VOPC__V_CMP_O_F32,
        &Decoder::decode_OP_VOPC__V_CMP_U_F32,
        &Decoder::decode_OP_VOPC__V_CMP_NGE_F32,
        &Decoder::decode_OP_VOPC__V_CMP_NLG_F32,
        &Decoder::decode_OP_VOPC__V_CMP_NGT_F32,
        &Decoder::decode_OP_VOPC__V_CMP_NLE_F32,
        &Decoder::decode_OP_VOPC__V_CMP_NEQ_F32,
        &Decoder::decode_OP_VOPC__V_CMP_NLT_F32,
        &Decoder::decode_OP_VOPC__V_CMP_TRU_F32,
        &Decoder::decode_OP_VOPC__V_CMPX_F_F32,
        &Decoder::decode_OP_VOPC__V_CMPX_LT_F32,
        &Decoder::decode_OP_VOPC__V_CMPX_EQ_F32,
        &Decoder::decode_OP_VOPC__V_CMPX_LE_F32,
        &Decoder::decode_OP_VOPC__V_CMPX_GT_F32,
        &Decoder::decode_OP_VOPC__V_CMPX_LG_F32,
        &Decoder::decode_OP_VOPC__V_CMPX_GE_F32,
        &Decoder::decode_OP_VOPC__V_CMPX_O_F32,
        &Decoder::decode_OP_VOPC__V_CMPX_U_F32,
        &Decoder::decode_OP_VOPC__V_CMPX_NGE_F32,
        &Decoder::decode_OP_VOPC__V_CMPX_NLG_F32,
        &Decoder::decode_OP_VOPC__V_CMPX_NGT_F32,
        &Decoder::decode_OP_VOPC__V_CMPX_NLE_F32,
        &Decoder::decode_OP_VOPC__V_CMPX_NEQ_F32,
        &Decoder::decode_OP_VOPC__V_CMPX_NLT_F32,
        &Decoder::decode_OP_VOPC__V_CMPX_TRU_F32,
        &Decoder::decode_OP_VOPC__V_CMP_F_F64,
        &Decoder::decode_OP_VOPC__V_CMP_LT_F64,
        &Decoder::decode_OP_VOPC__V_CMP_EQ_F64,
        &Decoder::decode_OP_VOPC__V_CMP_LE_F64,
        &Decoder::decode_OP_VOPC__V_CMP_GT_F64,
        &Decoder::decode_OP_VOPC__V_CMP_LG_F64,
        &Decoder::decode_OP_VOPC__V_CMP_GE_F64,
        &Decoder::decode_OP_VOPC__V_CMP_O_F64,
        &Decoder::decode_OP_VOPC__V_CMP_U_F64,
        &Decoder::decode_OP_VOPC__V_CMP_NGE_F64,
        &Decoder::decode_OP_VOPC__V_CMP_NLG_F64,
        &Decoder::decode_OP_VOPC__V_CMP_NGT_F64,
        &Decoder::decode_OP_VOPC__V_CMP_NLE_F64,
        &Decoder::decode_OP_VOPC__V_CMP_NEQ_F64,
        &Decoder::decode_OP_VOPC__V_CMP_NLT_F64,
        &Decoder::decode_OP_VOPC__V_CMP_TRU_F64,
        &Decoder::decode_OP_VOPC__V_CMPX_F_F64,
        &Decoder::decode_OP_VOPC__V_CMPX_LT_F64,
        &Decoder::decode_OP_VOPC__V_CMPX_EQ_F64,
        &Decoder::decode_OP_VOPC__V_CMPX_LE_F64,
        &Decoder::decode_OP_VOPC__V_CMPX_GT_F64,
        &Decoder::decode_OP_VOPC__V_CMPX_LG_F64,
        &Decoder::decode_OP_VOPC__V_CMPX_GE_F64,
        &Decoder::decode_OP_VOPC__V_CMPX_O_F64,
        &Decoder::decode_OP_VOPC__V_CMPX_U_F64,
        &Decoder::decode_OP_VOPC__V_CMPX_NGE_F64,
        &Decoder::decode_OP_VOPC__V_CMPX_NLG_F64,
        &Decoder::decode_OP_VOPC__V_CMPX_NGT_F64,
        &Decoder::decode_OP_VOPC__V_CMPX_NLE_F64,
        &Decoder::decode_OP_VOPC__V_CMPX_NEQ_F64,
        &Decoder::decode_OP_VOPC__V_CMPX_NLT_F64,
        &Decoder::decode_OP_VOPC__V_CMPX_TRU_F64,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_VOPC__V_CMP_F_I16,
        &Decoder::decode_OP_VOPC__V_CMP_LT_I16,
        &Decoder::decode_OP_VOPC__V_CMP_EQ_I16,
        &Decoder::decode_OP_VOPC__V_CMP_LE_I16,
        &Decoder::decode_OP_VOPC__V_CMP_GT_I16,
        &Decoder::decode_OP_VOPC__V_CMP_NE_I16,
        &Decoder::decode_OP_VOPC__V_CMP_GE_I16,
        &Decoder::decode_OP_VOPC__V_CMP_T_I16,
        &Decoder::decode_OP_VOPC__V_CMP_F_U16,
        &Decoder::decode_OP_VOPC__V_CMP_LT_U16,
        &Decoder::decode_OP_VOPC__V_CMP_EQ_U16,
        &Decoder::decode_OP_VOPC__V_CMP_LE_U16,
        &Decoder::decode_OP_VOPC__V_CMP_GT_U16,
        &Decoder::decode_OP_VOPC__V_CMP_NE_U16,
        &Decoder::decode_OP_VOPC__V_CMP_GE_U16,
        &Decoder::decode_OP_VOPC__V_CMP_T_U16,
        &Decoder::decode_OP_VOPC__V_CMPX_F_I16,
        &Decoder::decode_OP_VOPC__V_CMPX_LT_I16,
        &Decoder::decode_OP_VOPC__V_CMPX_EQ_I16,
        &Decoder::decode_OP_VOPC__V_CMPX_LE_I16,
        &Decoder::decode_OP_VOPC__V_CMPX_GT_I16,
        &Decoder::decode_OP_VOPC__V_CMPX_NE_I16,
        &Decoder::decode_OP_VOPC__V_CMPX_GE_I16,
        &Decoder::decode_OP_VOPC__V_CMPX_T_I16,
        &Decoder::decode_OP_VOPC__V_CMPX_F_U16,
        &Decoder::decode_OP_VOPC__V_CMPX_LT_U16,
        &Decoder::decode_OP_VOPC__V_CMPX_EQ_U16,
        &Decoder::decode_OP_VOPC__V_CMPX_LE_U16,
        &Decoder::decode_OP_VOPC__V_CMPX_GT_U16,
        &Decoder::decode_OP_VOPC__V_CMPX_NE_U16,
        &Decoder::decode_OP_VOPC__V_CMPX_GE_U16,
        &Decoder::decode_OP_VOPC__V_CMPX_T_U16,
        &Decoder::decode_OP_VOPC__V_CMP_F_I32,
        &Decoder::decode_OP_VOPC__V_CMP_LT_I32,
        &Decoder::decode_OP_VOPC__V_CMP_EQ_I32,
        &Decoder::decode_OP_VOPC__V_CMP_LE_I32,
        &Decoder::decode_OP_VOPC__V_CMP_GT_I32,
        &Decoder::decode_OP_VOPC__V_CMP_NE_I32,
        &Decoder::decode_OP_VOPC__V_CMP_GE_I32,
        &Decoder::decode_OP_VOPC__V_CMP_T_I32,
        &Decoder::decode_OP_VOPC__V_CMP_F_U32,
        &Decoder::decode_OP_VOPC__V_CMP_LT_U32,
        &Decoder::decode_OP_VOPC__V_CMP_EQ_U32,
        &Decoder::decode_OP_VOPC__V_CMP_LE_U32,
        &Decoder::decode_OP_VOPC__V_CMP_GT_U32,
        &Decoder::decode_OP_VOPC__V_CMP_NE_U32,
        &Decoder::decode_OP_VOPC__V_CMP_GE_U32,
        &Decoder::decode_OP_VOPC__V_CMP_T_U32,
        &Decoder::decode_OP_VOPC__V_CMPX_F_I32,
        &Decoder::decode_OP_VOPC__V_CMPX_LT_I32,
        &Decoder::decode_OP_VOPC__V_CMPX_EQ_I32,
        &Decoder::decode_OP_VOPC__V_CMPX_LE_I32,
        &Decoder::decode_OP_VOPC__V_CMPX_GT_I32,
        &Decoder::decode_OP_VOPC__V_CMPX_NE_I32,
        &Decoder::decode_OP_VOPC__V_CMPX_GE_I32,
        &Decoder::decode_OP_VOPC__V_CMPX_T_I32,
        &Decoder::decode_OP_VOPC__V_CMPX_F_U32,
        &Decoder::decode_OP_VOPC__V_CMPX_LT_U32,
        &Decoder::decode_OP_VOPC__V_CMPX_EQ_U32,
        &Decoder::decode_OP_VOPC__V_CMPX_LE_U32,
        &Decoder::decode_OP_VOPC__V_CMPX_GT_U32,
        &Decoder::decode_OP_VOPC__V_CMPX_NE_U32,
        &Decoder::decode_OP_VOPC__V_CMPX_GE_U32,
        &Decoder::decode_OP_VOPC__V_CMPX_T_U32,
        &Decoder::decode_OP_VOPC__V_CMP_F_I64,
        &Decoder::decode_OP_VOPC__V_CMP_LT_I64,
        &Decoder::decode_OP_VOPC__V_CMP_EQ_I64,
        &Decoder::decode_OP_VOPC__V_CMP_LE_I64,
        &Decoder::decode_OP_VOPC__V_CMP_GT_I64,
        &Decoder::decode_OP_VOPC__V_CMP_NE_I64,
        &Decoder::decode_OP_VOPC__V_CMP_GE_I64,
        &Decoder::decode_OP_VOPC__V_CMP_T_I64,
        &Decoder::decode_OP_VOPC__V_CMP_F_U64,
        &Decoder::decode_OP_VOPC__V_CMP_LT_U64,
        &Decoder::decode_OP_VOPC__V_CMP_EQ_U64,
        &Decoder::decode_OP_VOPC__V_CMP_LE_U64,
        &Decoder::decode_OP_VOPC__V_CMP_GT_U64,
        &Decoder::decode_OP_VOPC__V_CMP_NE_U64,
        &Decoder::decode_OP_VOPC__V_CMP_GE_U64,
        &Decoder::decode_OP_VOPC__V_CMP_T_U64,
        &Decoder::decode_OP_VOPC__V_CMPX_F_I64,
        &Decoder::decode_OP_VOPC__V_CMPX_LT_I64,
        &Decoder::decode_OP_VOPC__V_CMPX_EQ_I64,
        &Decoder::decode_OP_VOPC__V_CMPX_LE_I64,
        &Decoder::decode_OP_VOPC__V_CMPX_GT_I64,
        &Decoder::decode_OP_VOPC__V_CMPX_NE_I64,
        &Decoder::decode_OP_VOPC__V_CMPX_GE_I64,
        &Decoder::decode_OP_VOPC__V_CMPX_T_I64,
        &Decoder::decode_OP_VOPC__V_CMPX_F_U64,
        &Decoder::decode_OP_VOPC__V_CMPX_LT_U64,
        &Decoder::decode_OP_VOPC__V_CMPX_EQ_U64,
        &Decoder::decode_OP_VOPC__V_CMPX_LE_U64,
        &Decoder::decode_OP_VOPC__V_CMPX_GT_U64,
        &Decoder::decode_OP_VOPC__V_CMPX_NE_U64,
        &Decoder::decode_OP_VOPC__V_CMPX_GE_U64,
        &Decoder::decode_OP_VOPC__V_CMPX_T_U64,
    };

    IsaDecodeMethod Decoder::tableSubDecode_OP_VOP3P[] = {
        &Decoder::decode_OP_VOP3P__V_PK_MAD_I16,
        &Decoder::decode_OP_VOP3P__V_PK_MUL_LO_U16,
        &Decoder::decode_OP_VOP3P__V_PK_ADD_I16,
        &Decoder::decode_OP_VOP3P__V_PK_SUB_I16,
        &Decoder::decode_OP_VOP3P__V_PK_LSHLREV_B16,
        &Decoder::decode_OP_VOP3P__V_PK_LSHRREV_B16,
        &Decoder::decode_OP_VOP3P__V_PK_ASHRREV_I16,
        &Decoder::decode_OP_VOP3P__V_PK_MAX_I16,
        &Decoder::decode_OP_VOP3P__V_PK_MIN_I16,
        &Decoder::decode_OP_VOP3P__V_PK_MAD_U16,
        &Decoder::decode_OP_VOP3P__V_PK_ADD_U16,
        &Decoder::decode_OP_VOP3P__V_PK_SUB_U16,
        &Decoder::decode_OP_VOP3P__V_PK_MAX_U16,
        &Decoder::decode_OP_VOP3P__V_PK_MIN_U16,
        &Decoder::decode_OP_VOP3P__V_PK_FMA_F16,
        &Decoder::decode_OP_VOP3P__V_PK_ADD_F16,
        &Decoder::decode_OP_VOP3P__V_PK_MUL_F16,
        &Decoder::decode_OP_VOP3P__V_PK_MIN_F16,
        &Decoder::decode_OP_VOP3P__V_PK_MAX_F16,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_VOP3P__V_MAD_MIX_F32,
        &Decoder::decode_OP_VOP3P__V_MAD_MIXLO_F16,
        &Decoder::decode_OP_VOP3P__V_MAD_MIXHI_F16,
        &Decoder::decode_OP_VOP3P__V_DOT2_F32_F16,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_VOP3P__V_DOT2_I32_I16,
        &Decoder::decode_OP_VOP3P__V_DOT2_U32_U16,
        &Decoder::decode_OP_VOP3P__V_DOT4_I32_I8,
        &Decoder::decode_OP_VOP3P__V_DOT4_U32_U8,
        &Decoder::decode_OP_VOP3P__V_DOT8_I32_I4,
        &Decoder::decode_OP_VOP3P__V_DOT8_U32_U4,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_VOP3P__V_PK_FMA_F32,
        &Decoder::decode_OP_VOP3P__V_PK_MUL_F32,
        &Decoder::decode_OP_VOP3P__V_PK_ADD_F32,
        &Decoder::decode_OP_VOP3P__V_PK_MOV_B32,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_VOP3P__V_MFMA_F32_32X32X1_2B_F32,
        &Decoder::decode_OP_VOP3P__V_MFMA_F32_16X16X1_4B_F32,
        &Decoder::decode_OP_VOP3P__V_MFMA_F32_4X4X1_16B_F32,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_VOP3P__V_MFMA_F32_32X32X2_F32,
        &Decoder::decode_OP_VOP3P__V_MFMA_F32_16X16X4_F32,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_VOP3P__V_MFMA_F32_32X32X4_2B_F16,
        &Decoder::decode_OP_VOP3P__V_MFMA_F32_16X16X4_4B_F16,
        &Decoder::decode_OP_VOP3P__V_MFMA_F32_4X4X4_16B_F16,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_VOP3P__V_MFMA_F32_32X32X8_F16,
        &Decoder::decode_OP_VOP3P__V_MFMA_F32_16X16X16_F16,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_VOP3P__V_MFMA_I32_32X32X4_2B_I8,
        &Decoder::decode_OP_VOP3P__V_MFMA_I32_16X16X4_4B_I8,
        &Decoder::decode_OP_VOP3P__V_MFMA_I32_4X4X4_16B_I8,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_VOP3P__V_MFMA_I32_32X32X8_I8,
        &Decoder::decode_OP_VOP3P__V_MFMA_I32_16X16X16_I8,
        &Decoder::decode_OP_VOP3P__V_MFMA_I32_32X32X16_I8,
        &Decoder::decode_OP_VOP3P__V_MFMA_I32_16X16X32_I8,
        &Decoder::decode_OP_VOP3P__V_ACCVGPR_READ,
        &Decoder::decode_OP_VOP3P__V_ACCVGPR_WRITE,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_VOP3P__V_MFMA_F32_32X32X4_2B_BF16,
        &Decoder::decode_OP_VOP3P__V_MFMA_F32_16X16X4_4B_BF16,
        &Decoder::decode_OP_VOP3P__V_MFMA_F32_4X4X4_16B_BF16,
        &Decoder::decode_OP_VOP3P__V_MFMA_F32_32X32X8_BF16,
        &Decoder::decode_OP_VOP3P__V_MFMA_F32_16X16X16_BF16,
        &Decoder::decode_OP_VOP3P__V_SMFMAC_F32_16X16X32_F16,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_VOP3P__V_SMFMAC_F32_32X32X16_F16,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_VOP3P__V_SMFMAC_F32_16X16X32_BF16,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_VOP3P__V_SMFMAC_F32_32X32X16_BF16,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_VOP3P__V_SMFMAC_I32_16X16X64_I8,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_VOP3P__V_SMFMAC_I32_32X32X32_I8,
        &Decoder::decode_invalid,
        &Decoder::decode_OP_VOP3P__V_MFMA_F64_16X16X4_F64,
        &Decoder::decode_OP_VOP3P__V_MFMA_F64_4X4X4_4B_F64,
        &Decoder::decode_OP_VOP3P__V_MFMA_F32_16X16X32_BF8_BF8,
        &Decoder::decode_OP_VOP3P__V_MFMA_F32_16X16X32_BF8_FP8,
        &Decoder::decode_OP_VOP3P__V_MFMA_F32_16X16X32_FP8_BF8,
        &Decoder::decode_OP_VOP3P__V_MFMA_F32_16X16X32_FP8_FP8,
        &Decoder::decode_OP_VOP3P__V_MFMA_F32_32X32X16_BF8_BF8,
        &Decoder::decode_OP_VOP3P__V_MFMA_F32_32X32X16_BF8_FP8,
        &Decoder::decode_OP_VOP3P__V_MFMA_F32_32X32X16_FP8_BF8,
        &Decoder::decode_OP_VOP3P__V_MFMA_F32_32X32X16_FP8_FP8,
        &Decoder::decode_OP_VOP3P__V_SMFMAC_F32_16X16X64_BF8_BF8,
        &Decoder::decode_OP_VOP3P__V_SMFMAC_F32_16X16X64_BF8_FP8,
        &Decoder::decode_OP_VOP3P__V_SMFMAC_F32_16X16X64_FP8_BF8,
        &Decoder::decode_OP_VOP3P__V_SMFMAC_F32_16X16X64_FP8_FP8,
        &Decoder::decode_OP_VOP3P__V_SMFMAC_F32_32X32X32_BF8_BF8,
        &Decoder::decode_OP_VOP3P__V_SMFMAC_F32_32X32X32_BF8_FP8,
        &Decoder::decode_OP_VOP3P__V_SMFMAC_F32_32X32X32_FP8_BF8,
        &Decoder::decode_OP_VOP3P__V_SMFMAC_F32_32X32X32_FP8_FP8,
    };

    GPUStaticInst*
    Decoder::decode(MachInst mach_inst)
    {
        InFmt_SOP1 *enc = &mach_inst->iFmt_SOP1;
        IsaDecodeMethod method = tableDecodePrimary[enc->ENCODING];
        return (this->*method)(mach_inst);
    } // decode

    GPUStaticInst*
    Decoder::subDecode_OP_VOPC(MachInst iFmt)
    {
        InFmt_VOPC *enc = &iFmt->iFmt_VOPC;
        IsaDecodeMethod method = tableSubDecode_OP_VOPC[enc->OP];
        return (this->*method)(iFmt);
    } // subDecode_OP_VOPC

    GPUStaticInst*
    Decoder::subDecode_OP_VOP3P(MachInst iFmt)
    {
        InFmt_VOP3P *enc = &iFmt->iFmt_VOP3P;
        IsaDecodeMethod method = tableSubDecode_OP_VOP3P[enc->OP];
        return (this->*method)(iFmt);
    } // subDecode_OP_VOP3P

    GPUStaticInst*
    Decoder::subDecode_OP_VOP1(MachInst iFmt)
    {
        InFmt_VOP1 *enc = &iFmt->iFmt_VOP1;
        IsaDecodeMethod method = tableSubDecode_OP_VOP1[enc->OP];
        return (this->*method)(iFmt);
    } // subDecode_OP_VOP1

    GPUStaticInst*
    Decoder::subDecode_OP_SOP1(MachInst iFmt)
    {
        InFmt_SOP1 *enc = &iFmt->iFmt_SOP1;
        IsaDecodeMethod method = tableSubDecode_OP_SOP1[enc->OP];
        return (this->*method)(iFmt);
    } // subDecode_OP_SOP1

    GPUStaticInst*
    Decoder::subDecode_OP_SOPC(MachInst iFmt)
    {
        InFmt_SOPC *enc = &iFmt->iFmt_SOPC;
        IsaDecodeMethod method = tableSubDecode_OP_SOPC[enc->OP];
        return (this->*method)(iFmt);
    } // subDecode_OP_SOPC

    GPUStaticInst*
    Decoder::subDecode_OP_SOPP(MachInst iFmt)
    {
        InFmt_SOPP *enc = &iFmt->iFmt_SOPP;
        IsaDecodeMethod method = tableSubDecode_OP_SOPP[enc->OP];
        return (this->*method)(iFmt);
    } // subDecode_OP_SOPP

    GPUStaticInst*
    Decoder::subDecode_OP_SMEM(MachInst iFmt)
    {
        InFmt_SMEM *enc = &iFmt->iFmt_SMEM;
        IsaDecodeMethod method = tableSubDecode_OP_SMEM[enc->OP];
        return (this->*method)(iFmt);
    } // subDecode_OP_SMEM

    GPUStaticInst*
    Decoder::subDecode_OPU_VOP3(MachInst iFmt)
    {
        InFmt_VOP3A *enc = &iFmt->iFmt_VOP3A;
        IsaDecodeMethod method = tableSubDecode_OPU_VOP3[enc->OP];
        return (this->*method)(iFmt);
    } // subDecode_OPU_VOP3

    GPUStaticInst*
    Decoder::subDecode_OP_VINTRP(MachInst iFmt)
    {
        InFmt_VINTRP *enc = &iFmt->iFmt_VINTRP;
        IsaDecodeMethod method = tableSubDecode_OP_VINTRP[enc->OP];
        return (this->*method)(iFmt);
    } // subDecode_OP_VINTRP

    GPUStaticInst*
    Decoder::subDecode_OP_DS(MachInst iFmt)
    {
        InFmt_DS *enc = &iFmt->iFmt_DS;
        IsaDecodeMethod method = tableSubDecode_OP_DS[enc->OP];
        return (this->*method)(iFmt);
    } // subDecode_OP_DS

    GPUStaticInst*
    Decoder::subDecode_OP_FLAT(MachInst iFmt)
    {
        InFmt_FLAT *enc = &iFmt->iFmt_FLAT;
        IsaDecodeMethod method;
        switch (enc->SEG) {
            case 0:
                method = tableSubDecode_OP_FLAT[enc->OP];
                break;
            case 1:
                method = tableSubDecode_OP_SCRATCH[enc->OP];
                break;
            case 2:
                method = tableSubDecode_OP_GLOBAL[enc->OP];
                break;
            default:
                fatal("Invalid SEG for FLAT encoding: %d\n", enc->SEG);
        }
        return (this->*method)(iFmt);
    } // subDecode_OP_FLAT

    GPUStaticInst*
    Decoder::subDecode_OP_MUBUF(MachInst iFmt)
    {
        InFmt_MUBUF *enc = &iFmt->iFmt_MUBUF;
        IsaDecodeMethod method = tableSubDecode_OP_MUBUF[enc->OP];
        return (this->*method)(iFmt);
    } // subDecode_OP_MUBUF

    GPUStaticInst*
    Decoder::subDecode_OP_MTBUF(MachInst iFmt)
    {
        InFmt_MTBUF *enc = &iFmt->iFmt_MTBUF;
        IsaDecodeMethod method = tableSubDecode_OP_MTBUF[enc->OP];
        return (this->*method)(iFmt);
    } // subDecode_OP_MTBUF

    GPUStaticInst*
    Decoder::subDecode_OP_MIMG(MachInst iFmt)
    {
        InFmt_MIMG *enc = &iFmt->iFmt_MIMG;
        IsaDecodeMethod method = tableSubDecode_OP_MIMG[enc->OP];
        return (this->*method)(iFmt);
    } // subDecode_OP_MIMG

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_CNDMASK_B32(MachInst iFmt)
    {
        return new Inst_VOP2__V_CNDMASK_B32(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_CNDMASK_B32

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_ADD_F32(MachInst iFmt)
    {
        return new Inst_VOP2__V_ADD_F32(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_ADD_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_SUB_F32(MachInst iFmt)
    {
        return new Inst_VOP2__V_SUB_F32(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_SUB_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_SUBREV_F32(MachInst iFmt)
    {
        return new Inst_VOP2__V_SUBREV_F32(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_SUBREV_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_MUL_LEGACY_F32(MachInst iFmt)
    {
        return new Inst_VOP2__V_MUL_LEGACY_F32(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_MUL_LEGACY_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_MUL_F32(MachInst iFmt)
    {
        return new Inst_VOP2__V_MUL_F32(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_MUL_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_MUL_I32_I24(MachInst iFmt)
    {
        return new Inst_VOP2__V_MUL_I32_I24(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_MUL_I32_I24

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_MUL_HI_I32_I24(MachInst iFmt)
    {
        return new Inst_VOP2__V_MUL_HI_I32_I24(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_MUL_HI_I32_I24

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_MUL_U32_U24(MachInst iFmt)
    {
        return new Inst_VOP2__V_MUL_U32_U24(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_MUL_U32_U24

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_MUL_HI_U32_U24(MachInst iFmt)
    {
        return new Inst_VOP2__V_MUL_HI_U32_U24(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_MUL_HI_U32_U24

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_MIN_F32(MachInst iFmt)
    {
        return new Inst_VOP2__V_MIN_F32(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_MIN_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_MAX_F32(MachInst iFmt)
    {
        return new Inst_VOP2__V_MAX_F32(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_MAX_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_MIN_I32(MachInst iFmt)
    {
        return new Inst_VOP2__V_MIN_I32(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_MIN_I32

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_MAX_I32(MachInst iFmt)
    {
        return new Inst_VOP2__V_MAX_I32(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_MAX_I32

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_MIN_U32(MachInst iFmt)
    {
        return new Inst_VOP2__V_MIN_U32(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_MIN_U32

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_MAX_U32(MachInst iFmt)
    {
        return new Inst_VOP2__V_MAX_U32(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_MAX_U32

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_LSHRREV_B32(MachInst iFmt)
    {
        return new Inst_VOP2__V_LSHRREV_B32(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_LSHRREV_B32

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_ASHRREV_I32(MachInst iFmt)
    {
        return new Inst_VOP2__V_ASHRREV_I32(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_ASHRREV_I32

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_LSHLREV_B32(MachInst iFmt)
    {
        return new Inst_VOP2__V_LSHLREV_B32(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_LSHLREV_B32

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_AND_B32(MachInst iFmt)
    {
        return new Inst_VOP2__V_AND_B32(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_AND_B32

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_OR_B32(MachInst iFmt)
    {
        return new Inst_VOP2__V_OR_B32(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_OR_B32

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_XOR_B32(MachInst iFmt)
    {
        return new Inst_VOP2__V_XOR_B32(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_XOR_B32

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_MAC_F32(MachInst iFmt)
    {
        return new Inst_VOP2__V_MAC_F32(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_MAC_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_MADMK_F32(MachInst iFmt)
    {
        return new Inst_VOP2__V_MADMK_F32(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_MADMK_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_MADAK_F32(MachInst iFmt)
    {
        return new Inst_VOP2__V_MADAK_F32(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_MADAK_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_ADD_CO_U32(MachInst iFmt)
    {
        return new Inst_VOP2__V_ADD_CO_U32(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_ADD_CO_U32

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_SUB_CO_U32(MachInst iFmt)
    {
        return new Inst_VOP2__V_SUB_CO_U32(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_SUB_CO_U32

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_SUBREV_CO_U32(MachInst iFmt)
    {
        return new Inst_VOP2__V_SUBREV_CO_U32(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_SUBREV_CO_U32

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_ADDC_CO_U32(MachInst iFmt)
    {
        return new Inst_VOP2__V_ADDC_CO_U32(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_ADDC_CO_U32

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_SUBB_CO_U32(MachInst iFmt)
    {
        return new Inst_VOP2__V_SUBB_CO_U32(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_SUBB_CO_U32

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_SUBBREV_CO_U32(MachInst iFmt)
    {
        return new Inst_VOP2__V_SUBBREV_CO_U32(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_SUBBREV_CO_U32

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_ADD_F16(MachInst iFmt)
    {
        return new Inst_VOP2__V_ADD_F16(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_ADD_F16

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_SUB_F16(MachInst iFmt)
    {
        return new Inst_VOP2__V_SUB_F16(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_SUB_F16

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_SUBREV_F16(MachInst iFmt)
    {
        return new Inst_VOP2__V_SUBREV_F16(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_SUBREV_F16

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_MUL_F16(MachInst iFmt)
    {
        return new Inst_VOP2__V_MUL_F16(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_MUL_F16

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_MAC_F16(MachInst iFmt)
    {
        return new Inst_VOP2__V_MAC_F16(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_MAC_F16

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_MADMK_F16(MachInst iFmt)
    {
        return new Inst_VOP2__V_MADMK_F16(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_MADMK_F16

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_MADAK_F16(MachInst iFmt)
    {
        return new Inst_VOP2__V_MADAK_F16(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_MADAK_F16

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_ADD_U16(MachInst iFmt)
    {
        return new Inst_VOP2__V_ADD_U16(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_ADD_U16

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_SUB_U16(MachInst iFmt)
    {
        return new Inst_VOP2__V_SUB_U16(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_SUB_U16

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_SUBREV_U16(MachInst iFmt)
    {
        return new Inst_VOP2__V_SUBREV_U16(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_SUBREV_U16

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_MUL_LO_U16(MachInst iFmt)
    {
        return new Inst_VOP2__V_MUL_LO_U16(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_MUL_LO_U16

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_LSHLREV_B16(MachInst iFmt)
    {
        return new Inst_VOP2__V_LSHLREV_B16(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_LSHLREV_B16

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_LSHRREV_B16(MachInst iFmt)
    {
        return new Inst_VOP2__V_LSHRREV_B16(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_LSHRREV_B16

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_ASHRREV_I16(MachInst iFmt)
    {
        return new Inst_VOP2__V_ASHRREV_I16(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_ASHRREV_I16

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_MAX_F16(MachInst iFmt)
    {
        return new Inst_VOP2__V_MAX_F16(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_MAX_F16

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_MIN_F16(MachInst iFmt)
    {
        return new Inst_VOP2__V_MIN_F16(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_MIN_F16

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_MAX_U16(MachInst iFmt)
    {
        return new Inst_VOP2__V_MAX_U16(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_MAX_U16

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_MAX_I16(MachInst iFmt)
    {
        return new Inst_VOP2__V_MAX_I16(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_MAX_I16

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_MIN_U16(MachInst iFmt)
    {
        return new Inst_VOP2__V_MIN_U16(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_MIN_U16

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_MIN_I16(MachInst iFmt)
    {
        return new Inst_VOP2__V_MIN_I16(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_MIN_I16

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_LDEXP_F16(MachInst iFmt)
    {
        return new Inst_VOP2__V_LDEXP_F16(&iFmt->iFmt_VOP2);
    } // decode_OP_VOP2__V_LDEXP_F16

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_ADD_U32(MachInst iFmt)
    {
        return new Inst_VOP2__V_ADD_U32(&iFmt->iFmt_VOP2);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_SUB_U32(MachInst iFmt)
    {
        return new Inst_VOP2__V_SUB_U32(&iFmt->iFmt_VOP2);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_SUBREV_U32(MachInst iFmt)
    {
        return new Inst_VOP2__V_SUBREV_U32(&iFmt->iFmt_VOP2);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_DOT2C_F32_F16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_DOT2C_I32_I16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_DOT4C_I32_I8(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_DOT8C_I32_I4(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_FMAC_F32(MachInst iFmt)
    {
        return new Inst_VOP2__V_FMAC_F32(&iFmt->iFmt_VOP2);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_PK_FMAC_F16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP2__V_XNOR_B32(MachInst iFmt)
    {
        return new Inst_VOP2__V_XNOR_B32(&iFmt->iFmt_VOP2);
    }

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_ADD_U32(MachInst iFmt)
    {
        return new Inst_SOP2__S_ADD_U32(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_ADD_U32

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_SUB_U32(MachInst iFmt)
    {
        return new Inst_SOP2__S_SUB_U32(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_SUB_U32

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_ADD_I32(MachInst iFmt)
    {
        return new Inst_SOP2__S_ADD_I32(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_ADD_I32

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_SUB_I32(MachInst iFmt)
    {
        return new Inst_SOP2__S_SUB_I32(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_SUB_I32

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_ADDC_U32(MachInst iFmt)
    {
        return new Inst_SOP2__S_ADDC_U32(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_ADDC_U32

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_SUBB_U32(MachInst iFmt)
    {
        return new Inst_SOP2__S_SUBB_U32(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_SUBB_U32

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_MIN_I32(MachInst iFmt)
    {
        return new Inst_SOP2__S_MIN_I32(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_MIN_I32

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_MIN_U32(MachInst iFmt)
    {
        return new Inst_SOP2__S_MIN_U32(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_MIN_U32

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_MAX_I32(MachInst iFmt)
    {
        return new Inst_SOP2__S_MAX_I32(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_MAX_I32

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_MAX_U32(MachInst iFmt)
    {
        return new Inst_SOP2__S_MAX_U32(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_MAX_U32

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_CSELECT_B32(MachInst iFmt)
    {
        return new Inst_SOP2__S_CSELECT_B32(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_CSELECT_B32

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_CSELECT_B64(MachInst iFmt)
    {
        return new Inst_SOP2__S_CSELECT_B64(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_CSELECT_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_AND_B32(MachInst iFmt)
    {
        return new Inst_SOP2__S_AND_B32(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_AND_B32

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_AND_B64(MachInst iFmt)
    {
        return new Inst_SOP2__S_AND_B64(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_AND_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_OR_B32(MachInst iFmt)
    {
        return new Inst_SOP2__S_OR_B32(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_OR_B32

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_OR_B64(MachInst iFmt)
    {
        return new Inst_SOP2__S_OR_B64(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_OR_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_XOR_B32(MachInst iFmt)
    {
        return new Inst_SOP2__S_XOR_B32(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_XOR_B32

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_XOR_B64(MachInst iFmt)
    {
        return new Inst_SOP2__S_XOR_B64(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_XOR_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_ANDN2_B32(MachInst iFmt)
    {
        return new Inst_SOP2__S_ANDN2_B32(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_ANDN2_B32

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_ANDN2_B64(MachInst iFmt)
    {
        return new Inst_SOP2__S_ANDN2_B64(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_ANDN2_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_ORN2_B32(MachInst iFmt)
    {
        return new Inst_SOP2__S_ORN2_B32(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_ORN2_B32

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_ORN2_B64(MachInst iFmt)
    {
        return new Inst_SOP2__S_ORN2_B64(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_ORN2_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_NAND_B32(MachInst iFmt)
    {
        return new Inst_SOP2__S_NAND_B32(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_NAND_B32

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_NAND_B64(MachInst iFmt)
    {
        return new Inst_SOP2__S_NAND_B64(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_NAND_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_NOR_B32(MachInst iFmt)
    {
        return new Inst_SOP2__S_NOR_B32(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_NOR_B32

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_NOR_B64(MachInst iFmt)
    {
        return new Inst_SOP2__S_NOR_B64(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_NOR_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_XNOR_B32(MachInst iFmt)
    {
        return new Inst_SOP2__S_XNOR_B32(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_XNOR_B32

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_XNOR_B64(MachInst iFmt)
    {
        return new Inst_SOP2__S_XNOR_B64(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_XNOR_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_LSHL_B32(MachInst iFmt)
    {
        return new Inst_SOP2__S_LSHL_B32(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_LSHL_B32

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_LSHL_B64(MachInst iFmt)
    {
        return new Inst_SOP2__S_LSHL_B64(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_LSHL_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_LSHR_B32(MachInst iFmt)
    {
        return new Inst_SOP2__S_LSHR_B32(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_LSHR_B32

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_LSHR_B64(MachInst iFmt)
    {
        return new Inst_SOP2__S_LSHR_B64(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_LSHR_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_ASHR_I32(MachInst iFmt)
    {
        return new Inst_SOP2__S_ASHR_I32(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_ASHR_I32

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_ASHR_I64(MachInst iFmt)
    {
        return new Inst_SOP2__S_ASHR_I64(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_ASHR_I64

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_BFM_B32(MachInst iFmt)
    {
        return new Inst_SOP2__S_BFM_B32(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_BFM_B32

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_BFM_B64(MachInst iFmt)
    {
        return new Inst_SOP2__S_BFM_B64(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_BFM_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_MUL_I32(MachInst iFmt)
    {
        return new Inst_SOP2__S_MUL_I32(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_MUL_I32

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_BFE_U32(MachInst iFmt)
    {
        return new Inst_SOP2__S_BFE_U32(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_BFE_U32

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_BFE_I32(MachInst iFmt)
    {
        return new Inst_SOP2__S_BFE_I32(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_BFE_I32

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_BFE_U64(MachInst iFmt)
    {
        return new Inst_SOP2__S_BFE_U64(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_BFE_U64

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_BFE_I64(MachInst iFmt)
    {
        return new Inst_SOP2__S_BFE_I64(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_BFE_I64

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_CBRANCH_G_FORK(MachInst iFmt)
    {
        return new Inst_SOP2__S_CBRANCH_G_FORK(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_CBRANCH_G_FORK

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_ABSDIFF_I32(MachInst iFmt)
    {
        return new Inst_SOP2__S_ABSDIFF_I32(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_ABSDIFF_I32

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_RFE_RESTORE_B64(MachInst iFmt)
    {
        return new Inst_SOP2__S_RFE_RESTORE_B64(&iFmt->iFmt_SOP2);
    } // decode_OP_SOP2__S_RFE_RESTORE_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_MUL_HI_U32(MachInst iFmt)
    {
        return new Inst_SOP2__S_MUL_HI_U32(&iFmt->iFmt_SOP2);
    }

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_MUL_HI_I32(MachInst iFmt)
    {
        return new Inst_SOP2__S_MUL_HI_I32(&iFmt->iFmt_SOP2);
    }

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_LSHL1_ADD_U32(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_LSHL2_ADD_U32(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_LSHL3_ADD_U32(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_LSHL4_ADD_U32(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_PACK_LL_B32_B16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_PACK_LH_B32_B16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SOP2__S_HH_B32_B16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SOPK__S_MOVK_I32(MachInst iFmt)
    {
        return new Inst_SOPK__S_MOVK_I32(&iFmt->iFmt_SOPK);
    } // decode_OP_SOPK__S_MOVK_I32

    GPUStaticInst*
    Decoder::decode_OP_SOPK__S_CMOVK_I32(MachInst iFmt)
    {
        return new Inst_SOPK__S_CMOVK_I32(&iFmt->iFmt_SOPK);
    } // decode_OP_SOPK__S_CMOVK_I32

    GPUStaticInst*
    Decoder::decode_OP_SOPK__S_CMPK_EQ_I32(MachInst iFmt)
    {
        return new Inst_SOPK__S_CMPK_EQ_I32(&iFmt->iFmt_SOPK);
    } // decode_OP_SOPK__S_CMPK_EQ_I32

    GPUStaticInst*
    Decoder::decode_OP_SOPK__S_CMPK_LG_I32(MachInst iFmt)
    {
        return new Inst_SOPK__S_CMPK_LG_I32(&iFmt->iFmt_SOPK);
    } // decode_OP_SOPK__S_CMPK_LG_I32

    GPUStaticInst*
    Decoder::decode_OP_SOPK__S_CMPK_GT_I32(MachInst iFmt)
    {
        return new Inst_SOPK__S_CMPK_GT_I32(&iFmt->iFmt_SOPK);
    } // decode_OP_SOPK__S_CMPK_GT_I32

    GPUStaticInst*
    Decoder::decode_OP_SOPK__S_CMPK_GE_I32(MachInst iFmt)
    {
        return new Inst_SOPK__S_CMPK_GE_I32(&iFmt->iFmt_SOPK);
    } // decode_OP_SOPK__S_CMPK_GE_I32

    GPUStaticInst*
    Decoder::decode_OP_SOPK__S_CMPK_LT_I32(MachInst iFmt)
    {
        return new Inst_SOPK__S_CMPK_LT_I32(&iFmt->iFmt_SOPK);
    } // decode_OP_SOPK__S_CMPK_LT_I32

    GPUStaticInst*
    Decoder::decode_OP_SOPK__S_CMPK_LE_I32(MachInst iFmt)
    {
        return new Inst_SOPK__S_CMPK_LE_I32(&iFmt->iFmt_SOPK);
    } // decode_OP_SOPK__S_CMPK_LE_I32

    GPUStaticInst*
    Decoder::decode_OP_SOPK__S_CMPK_EQ_U32(MachInst iFmt)
    {
        return new Inst_SOPK__S_CMPK_EQ_U32(&iFmt->iFmt_SOPK);
    } // decode_OP_SOPK__S_CMPK_EQ_U32

    GPUStaticInst*
    Decoder::decode_OP_SOPK__S_CMPK_LG_U32(MachInst iFmt)
    {
        return new Inst_SOPK__S_CMPK_LG_U32(&iFmt->iFmt_SOPK);
    } // decode_OP_SOPK__S_CMPK_LG_U32

    GPUStaticInst*
    Decoder::decode_OP_SOPK__S_CMPK_GT_U32(MachInst iFmt)
    {
        return new Inst_SOPK__S_CMPK_GT_U32(&iFmt->iFmt_SOPK);
    } // decode_OP_SOPK__S_CMPK_GT_U32

    GPUStaticInst*
    Decoder::decode_OP_SOPK__S_CMPK_GE_U32(MachInst iFmt)
    {
        return new Inst_SOPK__S_CMPK_GE_U32(&iFmt->iFmt_SOPK);
    } // decode_OP_SOPK__S_CMPK_GE_U32

    GPUStaticInst*
    Decoder::decode_OP_SOPK__S_CMPK_LT_U32(MachInst iFmt)
    {
        return new Inst_SOPK__S_CMPK_LT_U32(&iFmt->iFmt_SOPK);
    } // decode_OP_SOPK__S_CMPK_LT_U32

    GPUStaticInst*
    Decoder::decode_OP_SOPK__S_CMPK_LE_U32(MachInst iFmt)
    {
        return new Inst_SOPK__S_CMPK_LE_U32(&iFmt->iFmt_SOPK);
    } // decode_OP_SOPK__S_CMPK_LE_U32

    GPUStaticInst*
    Decoder::decode_OP_SOPK__S_ADDK_I32(MachInst iFmt)
    {
        return new Inst_SOPK__S_ADDK_I32(&iFmt->iFmt_SOPK);
    } // decode_OP_SOPK__S_ADDK_I32

    GPUStaticInst*
    Decoder::decode_OP_SOPK__S_MULK_I32(MachInst iFmt)
    {
        return new Inst_SOPK__S_MULK_I32(&iFmt->iFmt_SOPK);
    } // decode_OP_SOPK__S_MULK_I32

    GPUStaticInst*
    Decoder::decode_OP_SOPK__S_CBRANCH_I_FORK(MachInst iFmt)
    {
        return new Inst_SOPK__S_CBRANCH_I_FORK(&iFmt->iFmt_SOPK);
    } // decode_OP_SOPK__S_CBRANCH_I_FORK

    GPUStaticInst*
    Decoder::decode_OP_SOPK__S_GETREG_B32(MachInst iFmt)
    {
        return new Inst_SOPK__S_GETREG_B32(&iFmt->iFmt_SOPK);
    } // decode_OP_SOPK__S_GETREG_B32

    GPUStaticInst*
    Decoder::decode_OP_SOPK__S_SETREG_B32(MachInst iFmt)
    {
        return new Inst_SOPK__S_SETREG_B32(&iFmt->iFmt_SOPK);
    } // decode_OP_SOPK__S_SETREG_B32

    GPUStaticInst*
    Decoder::decode_OP_SOPK__S_SETREG_IMM32_B32(MachInst iFmt)
    {
        return new Inst_SOPK__S_SETREG_IMM32_B32(&iFmt->iFmt_SOPK);
    } // decode_OP_SOPK__S_SETREG_IMM32_B32

    GPUStaticInst*
    Decoder::decode_OP_SOPK__S_CALL_B64(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_EXP(MachInst iFmt)
    {
        return new Inst_EXP__EXP(&iFmt->iFmt_EXP);
    } // decode_OP_EXP

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_CLASS_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_CLASS_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_CLASS_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_CLASS_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_CLASS_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_CLASS_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_CLASS_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_CLASS_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_CLASS_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_CLASS_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_CLASS_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_CLASS_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_CLASS_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_CLASS_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_CLASS_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_CLASS_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_CLASS_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_CLASS_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_F_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_F_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_F_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_LT_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_LT_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_LT_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_EQ_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_EQ_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_EQ_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_LE_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_LE_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_LE_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_GT_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_GT_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_GT_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_LG_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_LG_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_LG_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_GE_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_GE_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_GE_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_O_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_O_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_O_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_U_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_U_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_U_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_NGE_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_NGE_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_NGE_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_NLG_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_NLG_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_NLG_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_NGT_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_NGT_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_NGT_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_NLE_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_NLE_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_NLE_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_NEQ_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_NEQ_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_NEQ_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_NLT_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_NLT_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_NLT_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_TRU_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_TRU_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_TRU_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_F_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_F_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_F_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_LT_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_LT_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_LT_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_EQ_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_EQ_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_EQ_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_LE_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_LE_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_LE_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_GT_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_GT_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_GT_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_LG_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_LG_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_LG_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_GE_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_GE_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_GE_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_O_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_O_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_O_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_U_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_U_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_U_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_NGE_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_NGE_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_NGE_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_NLG_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_NLG_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_NLG_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_NGT_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_NGT_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_NGT_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_NLE_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_NLE_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_NLE_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_NEQ_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_NEQ_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_NEQ_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_NLT_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_NLT_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_NLT_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_TRU_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_TRU_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_TRU_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_F_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_F_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_F_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_LT_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_LT_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_LT_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_EQ_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_EQ_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_EQ_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_LE_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_LE_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_LE_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_GT_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_GT_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_GT_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_LG_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_LG_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_LG_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_GE_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_GE_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_GE_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_O_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_O_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_O_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_U_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_U_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_U_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_NGE_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_NGE_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_NGE_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_NLG_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_NLG_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_NLG_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_NGT_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_NGT_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_NGT_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_NLE_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_NLE_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_NLE_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_NEQ_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_NEQ_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_NEQ_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_NLT_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_NLT_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_NLT_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_TRU_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_TRU_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_TRU_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_F_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_F_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_F_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_LT_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_LT_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_LT_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_EQ_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_EQ_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_EQ_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_LE_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_LE_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_LE_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_GT_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_GT_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_GT_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_LG_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_LG_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_LG_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_GE_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_GE_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_GE_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_O_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_O_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_O_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_U_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_U_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_U_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_NGE_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_NGE_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_NGE_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_NLG_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_NLG_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_NLG_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_NGT_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_NGT_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_NGT_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_NLE_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_NLE_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_NLE_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_NEQ_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_NEQ_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_NEQ_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_NLT_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_NLT_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_NLT_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_TRU_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_TRU_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_TRU_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_F_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_F_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_F_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_LT_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_LT_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_LT_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_EQ_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_EQ_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_EQ_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_LE_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_LE_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_LE_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_GT_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_GT_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_GT_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_LG_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_LG_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_LG_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_GE_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_GE_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_GE_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_O_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_O_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_O_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_U_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_U_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_U_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_NGE_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_NGE_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_NGE_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_NLG_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_NLG_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_NLG_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_NGT_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_NGT_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_NGT_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_NLE_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_NLE_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_NLE_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_NEQ_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_NEQ_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_NEQ_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_NLT_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_NLT_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_NLT_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_TRU_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_TRU_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_TRU_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_F_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_F_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_F_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_LT_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_LT_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_LT_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_EQ_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_EQ_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_EQ_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_LE_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_LE_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_LE_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_GT_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_GT_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_GT_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_LG_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_LG_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_LG_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_GE_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_GE_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_GE_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_O_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_O_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_O_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_U_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_U_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_U_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_NGE_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_NGE_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_NGE_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_NLG_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_NLG_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_NLG_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_NGT_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_NGT_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_NGT_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_NLE_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_NLE_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_NLE_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_NEQ_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_NEQ_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_NEQ_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_NLT_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_NLT_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_NLT_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_TRU_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_TRU_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_TRU_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_F_I16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_F_I16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_F_I16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_LT_I16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_LT_I16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_LT_I16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_EQ_I16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_EQ_I16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_EQ_I16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_LE_I16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_LE_I16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_LE_I16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_GT_I16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_GT_I16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_GT_I16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_NE_I16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_NE_I16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_NE_I16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_GE_I16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_GE_I16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_GE_I16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_T_I16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_T_I16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_T_I16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_F_U16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_F_U16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_F_U16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_LT_U16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_LT_U16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_LT_U16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_EQ_U16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_EQ_U16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_EQ_U16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_LE_U16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_LE_U16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_LE_U16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_GT_U16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_GT_U16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_GT_U16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_NE_U16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_NE_U16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_NE_U16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_GE_U16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_GE_U16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_GE_U16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_T_U16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_T_U16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_T_U16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_F_I16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_F_I16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_F_I16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_LT_I16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_LT_I16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_LT_I16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_EQ_I16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_EQ_I16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_EQ_I16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_LE_I16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_LE_I16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_LE_I16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_GT_I16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_GT_I16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_GT_I16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_NE_I16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_NE_I16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_NE_I16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_GE_I16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_GE_I16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_GE_I16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_T_I16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_T_I16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_T_I16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_F_U16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_F_U16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_F_U16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_LT_U16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_LT_U16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_LT_U16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_EQ_U16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_EQ_U16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_EQ_U16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_LE_U16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_LE_U16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_LE_U16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_GT_U16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_GT_U16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_GT_U16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_NE_U16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_NE_U16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_NE_U16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_GE_U16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_GE_U16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_GE_U16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_T_U16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_T_U16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_T_U16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_F_I32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_F_I32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_F_I32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_LT_I32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_LT_I32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_LT_I32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_EQ_I32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_EQ_I32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_EQ_I32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_LE_I32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_LE_I32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_LE_I32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_GT_I32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_GT_I32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_GT_I32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_NE_I32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_NE_I32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_NE_I32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_GE_I32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_GE_I32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_GE_I32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_T_I32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_T_I32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_T_I32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_F_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_F_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_F_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_LT_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_LT_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_LT_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_EQ_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_EQ_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_EQ_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_LE_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_LE_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_LE_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_GT_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_GT_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_GT_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_NE_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_NE_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_NE_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_GE_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_GE_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_GE_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_T_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_T_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_T_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_F_I32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_F_I32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_F_I32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_LT_I32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_LT_I32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_LT_I32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_EQ_I32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_EQ_I32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_EQ_I32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_LE_I32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_LE_I32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_LE_I32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_GT_I32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_GT_I32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_GT_I32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_NE_I32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_NE_I32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_NE_I32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_GE_I32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_GE_I32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_GE_I32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_T_I32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_T_I32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_T_I32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_F_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_F_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_F_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_LT_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_LT_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_LT_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_EQ_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_EQ_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_EQ_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_LE_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_LE_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_LE_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_GT_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_GT_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_GT_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_NE_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_NE_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_NE_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_GE_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_GE_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_GE_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_T_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_T_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_T_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_F_I64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_F_I64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_F_I64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_LT_I64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_LT_I64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_LT_I64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_EQ_I64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_EQ_I64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_EQ_I64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_LE_I64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_LE_I64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_LE_I64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_GT_I64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_GT_I64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_GT_I64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_NE_I64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_NE_I64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_NE_I64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_GE_I64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_GE_I64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_GE_I64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_T_I64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_T_I64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_T_I64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_F_U64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_F_U64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_F_U64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_LT_U64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_LT_U64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_LT_U64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_EQ_U64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_EQ_U64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_EQ_U64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_LE_U64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_LE_U64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_LE_U64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_GT_U64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_GT_U64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_GT_U64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_NE_U64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_NE_U64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_NE_U64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_GE_U64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_GE_U64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_GE_U64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMP_T_U64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMP_T_U64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMP_T_U64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_F_I64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_F_I64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_F_I64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_LT_I64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_LT_I64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_LT_I64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_EQ_I64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_EQ_I64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_EQ_I64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_LE_I64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_LE_I64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_LE_I64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_GT_I64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_GT_I64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_GT_I64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_NE_I64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_NE_I64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_NE_I64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_GE_I64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_GE_I64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_GE_I64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_T_I64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_T_I64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_T_I64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_F_U64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_F_U64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_F_U64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_LT_U64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_LT_U64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_LT_U64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_EQ_U64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_EQ_U64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_EQ_U64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_LE_U64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_LE_U64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_LE_U64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_GT_U64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_GT_U64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_GT_U64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_NE_U64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_NE_U64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_NE_U64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_GE_U64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_GE_U64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_GE_U64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CMPX_T_U64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CMPX_T_U64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CMPX_T_U64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CNDMASK_B32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CNDMASK_B32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CNDMASK_B32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_ADD_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_ADD_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_ADD_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_SUB_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_SUB_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_SUB_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_SUBREV_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_SUBREV_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_SUBREV_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MUL_LEGACY_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_MUL_LEGACY_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MUL_LEGACY_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MUL_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_MUL_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MUL_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MUL_I32_I24(MachInst iFmt)
    {
        return new Inst_VOP3__V_MUL_I32_I24(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MUL_I32_I24

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MUL_HI_I32_I24(MachInst iFmt)
    {
        return new Inst_VOP3__V_MUL_HI_I32_I24(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MUL_HI_I32_I24

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MUL_U32_U24(MachInst iFmt)
    {
        return new Inst_VOP3__V_MUL_U32_U24(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MUL_U32_U24

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MUL_HI_U32_U24(MachInst iFmt)
    {
        return new Inst_VOP3__V_MUL_HI_U32_U24(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MUL_HI_U32_U24

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MIN_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_MIN_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MIN_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MAX_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_MAX_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MAX_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MIN_I32(MachInst iFmt)
    {
        return new Inst_VOP3__V_MIN_I32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MIN_I32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MAX_I32(MachInst iFmt)
    {
        return new Inst_VOP3__V_MAX_I32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MAX_I32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MIN_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_MIN_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MIN_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MAX_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_MAX_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MAX_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_LSHRREV_B32(MachInst iFmt)
    {
        return new Inst_VOP3__V_LSHRREV_B32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_LSHRREV_B32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_ASHRREV_I32(MachInst iFmt)
    {
        return new Inst_VOP3__V_ASHRREV_I32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_ASHRREV_I32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_LSHLREV_B32(MachInst iFmt)
    {
        return new Inst_VOP3__V_LSHLREV_B32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_LSHLREV_B32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_AND_B32(MachInst iFmt)
    {
        return new Inst_VOP3__V_AND_B32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_AND_B32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_OR_B32(MachInst iFmt)
    {
        return new Inst_VOP3__V_OR_B32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_OR_B32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_XOR_B32(MachInst iFmt)
    {
        return new Inst_VOP3__V_XOR_B32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_XOR_B32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MAC_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_MAC_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MAC_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_ADD_CO_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_ADD_CO_U32(&iFmt->iFmt_VOP3B);
    } // decode_OPU_VOP3__V_ADD_CO_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_SUB_CO_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_SUB_CO_U32(&iFmt->iFmt_VOP3B);
    } // decode_OPU_VOP3__V_SUB_CO_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_SUBREV_CO_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_SUBREV_CO_U32(&iFmt->iFmt_VOP3B);
    } // decode_OPU_VOP3__V_SUBREV_CO_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_ADDC_CO_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_ADDC_CO_U32(&iFmt->iFmt_VOP3B);
    } // decode_OPU_VOP3__V_ADDC_CO_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_SUBB_CO_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_SUBB_CO_U32(&iFmt->iFmt_VOP3B);
    } // decode_OPU_VOP3__V_SUBB_CO_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_SUBBREV_CO_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_SUBBREV_CO_U32(&iFmt->iFmt_VOP3B);
    } // decode_OPU_VOP3__V_SUBBREV_CO_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_ADD_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_ADD_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_ADD_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_SUB_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_SUB_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_SUB_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_SUBREV_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_SUBREV_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_SUBREV_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MUL_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_MUL_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MUL_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MAC_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_MAC_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MAC_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_ADD_U16(MachInst iFmt)
    {
        return new Inst_VOP3__V_ADD_U16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_ADD_U16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_SUB_U16(MachInst iFmt)
    {
        return new Inst_VOP3__V_SUB_U16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_SUB_U16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_SUBREV_U16(MachInst iFmt)
    {
        return new Inst_VOP3__V_SUBREV_U16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_SUBREV_U16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MUL_LO_U16(MachInst iFmt)
    {
        return new Inst_VOP3__V_MUL_LO_U16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MUL_LO_U16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_LSHLREV_B16(MachInst iFmt)
    {
        return new Inst_VOP3__V_LSHLREV_B16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_LSHLREV_B16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_LSHRREV_B16(MachInst iFmt)
    {
        return new Inst_VOP3__V_LSHRREV_B16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_LSHRREV_B16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_ASHRREV_I16(MachInst iFmt)
    {
        return new Inst_VOP3__V_ASHRREV_I16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_ASHRREV_I16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MAX_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_MAX_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MAX_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MIN_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_MIN_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MIN_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MAX_U16(MachInst iFmt)
    {
        return new Inst_VOP3__V_MAX_U16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MAX_U16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MAX_I16(MachInst iFmt)
    {
        return new Inst_VOP3__V_MAX_I16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MAX_I16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MIN_U16(MachInst iFmt)
    {
        return new Inst_VOP3__V_MIN_U16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MIN_U16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MIN_I16(MachInst iFmt)
    {
        return new Inst_VOP3__V_MIN_I16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MIN_I16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_LDEXP_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_LDEXP_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_LDEXP_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_ADD_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_ADD_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_ADD_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_SUB_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_SUB_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_SUB_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_SUBREV_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_SUBREV_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_SUBREV_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_FMAC_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_FMAC_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_FMAC_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_NOP(MachInst iFmt)
    {
        return new Inst_VOP3__V_NOP(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_NOP

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MOV_B32(MachInst iFmt)
    {
        return new Inst_VOP3__V_MOV_B32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MOV_B32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_I32_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_I32_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_I32_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_F64_I32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_F64_I32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_F64_I32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_F32_I32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_F32_I32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_F32_I32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_F32_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_F32_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_F32_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_U32_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_U32_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_U32_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_I32_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_I32_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_I32_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MOV_FED_B32(MachInst iFmt)
    {
        return new Inst_VOP3__V_MOV_FED_B32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MOV_FED_B32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_F16_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_F16_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_F16_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_F32_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_F32_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_F32_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_RPI_I32_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_RPI_I32_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_RPI_I32_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_FLR_I32_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_FLR_I32_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_FLR_I32_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_OFF_F32_I4(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_OFF_F32_I4(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_OFF_F32_I4

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_F32_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_F32_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_F32_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_F64_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_F64_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_F64_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_F32_UBYTE0(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_F32_UBYTE0(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_F32_UBYTE0

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_F32_UBYTE1(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_F32_UBYTE1(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_F32_UBYTE1

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_F32_UBYTE2(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_F32_UBYTE2(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_F32_UBYTE2

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_F32_UBYTE3(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_F32_UBYTE3(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_F32_UBYTE3

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_U32_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_U32_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_U32_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_F64_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_F64_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_F64_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_TRUNC_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_TRUNC_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_TRUNC_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CEIL_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_CEIL_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CEIL_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_RNDNE_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_RNDNE_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_RNDNE_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_FLOOR_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_FLOOR_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_FLOOR_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_FRACT_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_FRACT_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_FRACT_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_TRUNC_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_TRUNC_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_TRUNC_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CEIL_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CEIL_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CEIL_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_RNDNE_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_RNDNE_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_RNDNE_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_FLOOR_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_FLOOR_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_FLOOR_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_EXP_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_EXP_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_EXP_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_LOG_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_LOG_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_LOG_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_RCP_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_RCP_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_RCP_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_RCP_IFLAG_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_RCP_IFLAG_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_RCP_IFLAG_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_RSQ_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_RSQ_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_RSQ_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_RCP_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_RCP_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_RCP_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_RSQ_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_RSQ_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_RSQ_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_SQRT_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_SQRT_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_SQRT_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_SQRT_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_SQRT_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_SQRT_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_SIN_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_SIN_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_SIN_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_COS_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_COS_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_COS_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_NOT_B32(MachInst iFmt)
    {
        return new Inst_VOP3__V_NOT_B32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_NOT_B32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_BFREV_B32(MachInst iFmt)
    {
        return new Inst_VOP3__V_BFREV_B32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_BFREV_B32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_FFBH_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_FFBH_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_FFBH_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_FFBL_B32(MachInst iFmt)
    {
        return new Inst_VOP3__V_FFBL_B32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_FFBL_B32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_FFBH_I32(MachInst iFmt)
    {
        return new Inst_VOP3__V_FFBH_I32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_FFBH_I32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_FREXP_EXP_I32_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_FREXP_EXP_I32_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_FREXP_EXP_I32_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_FREXP_MANT_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_FREXP_MANT_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_FREXP_MANT_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_FRACT_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_FRACT_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_FRACT_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_FREXP_EXP_I32_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_FREXP_EXP_I32_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_FREXP_EXP_I32_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_FREXP_MANT_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_FREXP_MANT_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_FREXP_MANT_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CLREXCP(MachInst iFmt)
    {
        return new Inst_VOP3__V_CLREXCP(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CLREXCP

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_F16_U16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_F16_U16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_F16_U16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_F16_I16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_F16_I16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_F16_I16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_U16_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_U16_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_U16_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_I16_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_I16_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_I16_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_RCP_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_RCP_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_RCP_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_SQRT_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_SQRT_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_SQRT_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_RSQ_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_RSQ_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_RSQ_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_LOG_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_LOG_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_LOG_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_EXP_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_EXP_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_EXP_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_FREXP_MANT_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_FREXP_MANT_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_FREXP_MANT_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_FREXP_EXP_I16_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_FREXP_EXP_I16_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_FREXP_EXP_I16_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_FLOOR_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_FLOOR_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_FLOOR_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CEIL_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_CEIL_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CEIL_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_TRUNC_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_TRUNC_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_TRUNC_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_RNDNE_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_RNDNE_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_RNDNE_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_FRACT_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_FRACT_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_FRACT_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_SIN_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_SIN_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_SIN_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_COS_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_COS_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_COS_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_EXP_LEGACY_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_EXP_LEGACY_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_EXP_LEGACY_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_LOG_LEGACY_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_LOG_LEGACY_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_LOG_LEGACY_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MAD_LEGACY_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_MAD_LEGACY_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MAD_LEGACY_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MAD_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_MAD_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MAD_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MAD_I32_I24(MachInst iFmt)
    {
        return new Inst_VOP3__V_MAD_I32_I24(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MAD_I32_I24

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MAD_U32_U24(MachInst iFmt)
    {
        return new Inst_VOP3__V_MAD_U32_U24(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MAD_U32_U24

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CUBEID_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CUBEID_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CUBEID_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CUBESC_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CUBESC_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CUBESC_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CUBETC_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CUBETC_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CUBETC_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CUBEMA_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CUBEMA_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CUBEMA_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_BFE_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_BFE_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_BFE_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_BFE_I32(MachInst iFmt)
    {
        return new Inst_VOP3__V_BFE_I32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_BFE_I32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_BFI_B32(MachInst iFmt)
    {
        return new Inst_VOP3__V_BFI_B32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_BFI_B32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_FMA_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_FMA_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_FMA_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_FMA_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_FMA_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_FMA_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_LERP_U8(MachInst iFmt)
    {
        return new Inst_VOP3__V_LERP_U8(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_LERP_U8

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_ALIGNBIT_B32(MachInst iFmt)
    {
        return new Inst_VOP3__V_ALIGNBIT_B32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_ALIGNBIT_B32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_ALIGNBYTE_B32(MachInst iFmt)
    {
        return new Inst_VOP3__V_ALIGNBYTE_B32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_ALIGNBYTE_B32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MIN3_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_MIN3_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MIN3_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MIN3_I32(MachInst iFmt)
    {
        return new Inst_VOP3__V_MIN3_I32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MIN3_I32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MIN3_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_MIN3_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MIN3_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MAX3_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_MAX3_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MAX3_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MAX3_I32(MachInst iFmt)
    {
        return new Inst_VOP3__V_MAX3_I32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MAX3_I32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MAX3_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_MAX3_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MAX3_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MED3_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_MED3_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MED3_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MED3_I32(MachInst iFmt)
    {
        return new Inst_VOP3__V_MED3_I32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MED3_I32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MED3_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_MED3_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MED3_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_SAD_U8(MachInst iFmt)
    {
        return new Inst_VOP3__V_SAD_U8(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_SAD_U8

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_SAD_HI_U8(MachInst iFmt)
    {
        return new Inst_VOP3__V_SAD_HI_U8(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_SAD_HI_U8

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_SAD_U16(MachInst iFmt)
    {
        return new Inst_VOP3__V_SAD_U16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_SAD_U16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_SAD_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_SAD_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_SAD_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_PK_U8_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_PK_U8_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_PK_U8_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_DIV_FIXUP_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_DIV_FIXUP_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_DIV_FIXUP_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_DIV_FIXUP_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_DIV_FIXUP_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_DIV_FIXUP_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_DIV_SCALE_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_DIV_SCALE_F32(&iFmt->iFmt_VOP3B);
    } // decode_OPU_VOP3__V_DIV_SCALE_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_DIV_SCALE_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_DIV_SCALE_F64(&iFmt->iFmt_VOP3B);
    } // decode_OPU_VOP3__V_DIV_SCALE_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_DIV_FMAS_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_DIV_FMAS_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_DIV_FMAS_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_DIV_FMAS_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_DIV_FMAS_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_DIV_FMAS_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MSAD_U8(MachInst iFmt)
    {
        return new Inst_VOP3__V_MSAD_U8(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MSAD_U8

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_QSAD_PK_U16_U8(MachInst iFmt)
    {
        return new Inst_VOP3__V_QSAD_PK_U16_U8(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_QSAD_PK_U16_U8

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MQSAD_PK_U16_U8(MachInst iFmt)
    {
        return new Inst_VOP3__V_MQSAD_PK_U16_U8(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MQSAD_PK_U16_U8

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MQSAD_U32_U8(MachInst iFmt)
    {
        return new Inst_VOP3__V_MQSAD_U32_U8(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MQSAD_U32_U8

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MAD_U64_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_MAD_U64_U32(&iFmt->iFmt_VOP3B);
    } // decode_OPU_VOP3__V_MAD_U64_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MAD_I64_I32(MachInst iFmt)
    {
        return new Inst_VOP3__V_MAD_I64_I32(&iFmt->iFmt_VOP3B);
    } // decode_OPU_VOP3__V_MAD_I64_I32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MAD_LEGACY_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_MAD_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MAD_LEGACY_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MAD_LEGACY_U16(MachInst iFmt)
    {
        return new Inst_VOP3__V_MAD_U16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MAD_LEGACY_U16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MAD_LEGACY_I16(MachInst iFmt)
    {
        return new Inst_VOP3__V_MAD_I16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MAD_LEGACY_I16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_PERM_B32(MachInst iFmt)
    {
        return new Inst_VOP3__V_PERM_B32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_PERM_B32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_FMA_LEGACY_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_FMA_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_FMA_LEGACY_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_DIV_FIXUP_LEGACY_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_DIV_FIXUP_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_DIV_FIXUP_LEGACY_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_PKACCUM_U8_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_PKACCUM_U8_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_PKACCUM_U8_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MAD_U32_U16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MAD_I32_I16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_XAD_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_XAD_U32(&iFmt->iFmt_VOP3A);
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MIN3_F16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MIN3_I16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MIN3_U16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MAX3_F16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MAX3_I16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MAX3_U16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MED3_F16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MED3_I16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MED3_U16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_LSHL_ADD_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_LSHL_ADD_U32(&iFmt->iFmt_VOP3A);
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_ADD_LSHL_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_ADD_LSHL_U32(&iFmt->iFmt_VOP3A);
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_ADD3_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_ADD3_U32(&iFmt->iFmt_VOP3A);
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_LSHL_OR_B32(MachInst iFmt)
    {
        return new Inst_VOP3__V_LSHL_OR_B32(&iFmt->iFmt_VOP3A);
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_AND_OR_B32(MachInst iFmt)
    {
        return new Inst_VOP3__V_AND_OR_B32(&iFmt->iFmt_VOP3A);
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_OR3_B32(MachInst iFmt)
    {
        return new Inst_VOP3__V_OR3_B32(&iFmt->iFmt_VOP3A);
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MAD_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_MAD_F16(&iFmt->iFmt_VOP3A);
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MAD_U16(MachInst iFmt)
    {
        return new Inst_VOP3__V_MAD_U16(&iFmt->iFmt_VOP3A);
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MAD_I16(MachInst iFmt)
    {
        return new Inst_VOP3__V_MAD_I16(&iFmt->iFmt_VOP3A);
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_FMA_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_FMA_F16(&iFmt->iFmt_VOP3A);
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_DIV_FIXUP_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_DIV_FIXUP_F16(&iFmt->iFmt_VOP3A);
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_LSHL_ADD_U64(MachInst iFmt)
    {
        return new Inst_VOP3__V_LSHL_ADD_U64(&iFmt->iFmt_VOP3A);
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_INTERP_P1_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_INTERP_P1_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_INTERP_P1_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_INTERP_P2_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_INTERP_P2_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_INTERP_P2_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_INTERP_MOV_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_INTERP_MOV_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_INTERP_MOV_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_INTERP_P1LL_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_INTERP_P1LL_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_INTERP_P1LL_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_INTERP_P1LV_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_INTERP_P1LV_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_INTERP_P1LV_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_INTERP_P2_LEGACY_F16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_INTERP_P2_F16(MachInst iFmt)
    {
        return new Inst_VOP3__V_INTERP_P2_F16(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_INTERP_P2_F16

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_ADD_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_ADD_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_ADD_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MUL_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_MUL_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MUL_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MIN_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_MIN_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MIN_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MAX_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_MAX_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MAX_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_LDEXP_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_LDEXP_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_LDEXP_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MUL_LO_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_MUL_LO_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MUL_LO_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MUL_HI_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_MUL_HI_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MUL_HI_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MUL_HI_I32(MachInst iFmt)
    {
        return new Inst_VOP3__V_MUL_HI_I32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MUL_HI_I32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_LDEXP_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_LDEXP_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_LDEXP_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_READLANE_B32(MachInst iFmt)
    {
        return new Inst_VOP3__V_READLANE_B32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_READLANE_B32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_WRITELANE_B32(MachInst iFmt)
    {
        return new Inst_VOP3__V_WRITELANE_B32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_WRITELANE_B32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_BCNT_U32_B32(MachInst iFmt)
    {
        return new Inst_VOP3__V_BCNT_U32_B32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_BCNT_U32_B32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MBCNT_LO_U32_B32(MachInst iFmt)
    {
        return new Inst_VOP3__V_MBCNT_LO_U32_B32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MBCNT_LO_U32_B32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_MBCNT_HI_U32_B32(MachInst iFmt)
    {
        return new Inst_VOP3__V_MBCNT_HI_U32_B32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_MBCNT_HI_U32_B32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_LSHLREV_B64(MachInst iFmt)
    {
        return new Inst_VOP3__V_LSHLREV_B64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_LSHLREV_B64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_LSHRREV_B64(MachInst iFmt)
    {
        return new Inst_VOP3__V_LSHRREV_B64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_LSHRREV_B64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_ASHRREV_I64(MachInst iFmt)
    {
        return new Inst_VOP3__V_ASHRREV_I64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_ASHRREV_I64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_TRIG_PREOP_F64(MachInst iFmt)
    {
        return new Inst_VOP3__V_TRIG_PREOP_F64(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_TRIG_PREOP_F64

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_BFM_B32(MachInst iFmt)
    {
        return new Inst_VOP3__V_BFM_B32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_BFM_B32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_PKNORM_I16_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_PKNORM_I16_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_PKNORM_I16_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_PKNORM_U16_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_PKNORM_U16_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_PKNORM_U16_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_PKRTZ_F16_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_PKRTZ_F16_F32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_PKRTZ_F16_F32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_PK_U16_U32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_PK_U16_U32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_PK_U16_U32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_PK_I16_I32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_PK_I16_I32(&iFmt->iFmt_VOP3A);
    } // decode_OPU_VOP3__V_CVT_PK_I16_I32

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_PKNORM_I16_F16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_PKNORM_U16_F16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_ADD_I32(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_SUB_I32(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_ADD_I16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_SUB_I16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_PACK_B32_F16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OPU_VOP3__V_CVT_PK_FP8_F32(MachInst iFmt)
    {
        return new Inst_VOP3__V_CVT_PK_FP8_F32(&iFmt->iFmt_VOP3A);
    }

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_ADD_U32(MachInst iFmt)
    {
        return new Inst_DS__DS_ADD_U32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_ADD_U32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_SUB_U32(MachInst iFmt)
    {
        return new Inst_DS__DS_SUB_U32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_SUB_U32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_RSUB_U32(MachInst iFmt)
    {
        return new Inst_DS__DS_RSUB_U32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_RSUB_U32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_INC_U32(MachInst iFmt)
    {
        return new Inst_DS__DS_INC_U32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_INC_U32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_DEC_U32(MachInst iFmt)
    {
        return new Inst_DS__DS_DEC_U32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_DEC_U32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MIN_I32(MachInst iFmt)
    {
        return new Inst_DS__DS_MIN_I32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MIN_I32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MAX_I32(MachInst iFmt)
    {
        return new Inst_DS__DS_MAX_I32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MAX_I32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MIN_U32(MachInst iFmt)
    {
        return new Inst_DS__DS_MIN_U32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MIN_U32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MAX_U32(MachInst iFmt)
    {
        return new Inst_DS__DS_MAX_U32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MAX_U32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_AND_B32(MachInst iFmt)
    {
        return new Inst_DS__DS_AND_B32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_AND_B32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_OR_B32(MachInst iFmt)
    {
        return new Inst_DS__DS_OR_B32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_OR_B32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_XOR_B32(MachInst iFmt)
    {
        return new Inst_DS__DS_XOR_B32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_XOR_B32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MSKOR_B32(MachInst iFmt)
    {
        return new Inst_DS__DS_MSKOR_B32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MSKOR_B32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_WRITE_B32(MachInst iFmt)
    {
        return new Inst_DS__DS_WRITE_B32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_WRITE_B32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_WRITE2_B32(MachInst iFmt)
    {
        return new Inst_DS__DS_WRITE2_B32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_WRITE2_B32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_WRITE2ST64_B32(MachInst iFmt)
    {
        return new Inst_DS__DS_WRITE2ST64_B32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_WRITE2ST64_B32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_CMPST_B32(MachInst iFmt)
    {
        return new Inst_DS__DS_CMPST_B32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_CMPST_B32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_CMPST_F32(MachInst iFmt)
    {
        return new Inst_DS__DS_CMPST_F32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_CMPST_F32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MIN_F32(MachInst iFmt)
    {
        return new Inst_DS__DS_MIN_F32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MIN_F32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MAX_F32(MachInst iFmt)
    {
        return new Inst_DS__DS_MAX_F32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MAX_F32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_NOP(MachInst iFmt)
    {
        return new Inst_DS__DS_NOP(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_NOP

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_ADD_F32(MachInst iFmt)
    {
        return new Inst_DS__DS_ADD_F32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_ADD_F32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_WRITE_ADDTID_B32(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_WRITE_B8(MachInst iFmt)
    {
        return new Inst_DS__DS_WRITE_B8(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_WRITE_B8

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_WRITE_B16(MachInst iFmt)
    {
        return new Inst_DS__DS_WRITE_B16(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_WRITE_B16

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_ADD_RTN_U32(MachInst iFmt)
    {
        return new Inst_DS__DS_ADD_RTN_U32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_ADD_RTN_U32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_SUB_RTN_U32(MachInst iFmt)
    {
        return new Inst_DS__DS_SUB_RTN_U32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_SUB_RTN_U32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_RSUB_RTN_U32(MachInst iFmt)
    {
        return new Inst_DS__DS_RSUB_RTN_U32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_RSUB_RTN_U32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_INC_RTN_U32(MachInst iFmt)
    {
        return new Inst_DS__DS_INC_RTN_U32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_INC_RTN_U32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_DEC_RTN_U32(MachInst iFmt)
    {
        return new Inst_DS__DS_DEC_RTN_U32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_DEC_RTN_U32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MIN_RTN_I32(MachInst iFmt)
    {
        return new Inst_DS__DS_MIN_RTN_I32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MIN_RTN_I32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MAX_RTN_I32(MachInst iFmt)
    {
        return new Inst_DS__DS_MAX_RTN_I32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MAX_RTN_I32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MIN_RTN_U32(MachInst iFmt)
    {
        return new Inst_DS__DS_MIN_RTN_U32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MIN_RTN_U32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MAX_RTN_U32(MachInst iFmt)
    {
        return new Inst_DS__DS_MAX_RTN_U32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MAX_RTN_U32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_AND_RTN_B32(MachInst iFmt)
    {
        return new Inst_DS__DS_AND_RTN_B32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_AND_RTN_B32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_OR_RTN_B32(MachInst iFmt)
    {
        return new Inst_DS__DS_OR_RTN_B32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_OR_RTN_B32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_XOR_RTN_B32(MachInst iFmt)
    {
        return new Inst_DS__DS_XOR_RTN_B32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_XOR_RTN_B32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MSKOR_RTN_B32(MachInst iFmt)
    {
        return new Inst_DS__DS_MSKOR_RTN_B32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MSKOR_RTN_B32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_WRXCHG_RTN_B32(MachInst iFmt)
    {
        return new Inst_DS__DS_WRXCHG_RTN_B32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_WRXCHG_RTN_B32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_WRXCHG2_RTN_B32(MachInst iFmt)
    {
        return new Inst_DS__DS_WRXCHG2_RTN_B32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_WRXCHG2_RTN_B32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_WRXCHG2ST64_RTN_B32(MachInst iFmt)
    {
        return new Inst_DS__DS_WRXCHG2ST64_RTN_B32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_WRXCHG2ST64_RTN_B32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_CMPST_RTN_B32(MachInst iFmt)
    {
        return new Inst_DS__DS_CMPST_RTN_B32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_CMPST_RTN_B32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_CMPST_RTN_F32(MachInst iFmt)
    {
        return new Inst_DS__DS_CMPST_RTN_F32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_CMPST_RTN_F32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MIN_RTN_F32(MachInst iFmt)
    {
        return new Inst_DS__DS_MIN_RTN_F32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MIN_RTN_F32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MAX_RTN_F32(MachInst iFmt)
    {
        return new Inst_DS__DS_MAX_RTN_F32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MAX_RTN_F32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_WRAP_RTN_B32(MachInst iFmt)
    {
        return new Inst_DS__DS_WRAP_RTN_B32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_WRAP_RTN_B32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_ADD_RTN_F32(MachInst iFmt)
    {
        return new Inst_DS__DS_ADD_RTN_F32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_ADD_RTN_F32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_READ_B32(MachInst iFmt)
    {
        return new Inst_DS__DS_READ_B32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_READ_B32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_READ2_B32(MachInst iFmt)
    {
        return new Inst_DS__DS_READ2_B32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_READ2_B32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_READ2ST64_B32(MachInst iFmt)
    {
        return new Inst_DS__DS_READ2ST64_B32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_READ2ST64_B32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_READ_I8(MachInst iFmt)
    {
        return new Inst_DS__DS_READ_I8(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_READ_I8

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_READ_U8(MachInst iFmt)
    {
        return new Inst_DS__DS_READ_U8(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_READ_U8

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_READ_I16(MachInst iFmt)
    {
        return new Inst_DS__DS_READ_I16(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_READ_I16

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_READ_U16(MachInst iFmt)
    {
        return new Inst_DS__DS_READ_U16(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_READ_U16

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_SWIZZLE_B32(MachInst iFmt)
    {
        return new Inst_DS__DS_SWIZZLE_B32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_SWIZZLE_B32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_PERMUTE_B32(MachInst iFmt)
    {
        return new Inst_DS__DS_PERMUTE_B32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_PERMUTE_B32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_BPERMUTE_B32(MachInst iFmt)
    {
        return new Inst_DS__DS_BPERMUTE_B32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_BPERMUTE_B32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_ADD_U64(MachInst iFmt)
    {
        return new Inst_DS__DS_ADD_U64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_ADD_U64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_SUB_U64(MachInst iFmt)
    {
        return new Inst_DS__DS_SUB_U64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_SUB_U64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_RSUB_U64(MachInst iFmt)
    {
        return new Inst_DS__DS_RSUB_U64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_RSUB_U64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_INC_U64(MachInst iFmt)
    {
        return new Inst_DS__DS_INC_U64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_INC_U64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_DEC_U64(MachInst iFmt)
    {
        return new Inst_DS__DS_DEC_U64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_DEC_U64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MIN_I64(MachInst iFmt)
    {
        return new Inst_DS__DS_MIN_I64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MIN_I64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MAX_I64(MachInst iFmt)
    {
        return new Inst_DS__DS_MAX_I64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MAX_I64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MIN_U64(MachInst iFmt)
    {
        return new Inst_DS__DS_MIN_U64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MIN_U64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MAX_U64(MachInst iFmt)
    {
        return new Inst_DS__DS_MAX_U64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MAX_U64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_AND_B64(MachInst iFmt)
    {
        return new Inst_DS__DS_AND_B64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_AND_B64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_OR_B64(MachInst iFmt)
    {
        return new Inst_DS__DS_OR_B64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_OR_B64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_XOR_B64(MachInst iFmt)
    {
        return new Inst_DS__DS_XOR_B64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_XOR_B64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MSKOR_B64(MachInst iFmt)
    {
        return new Inst_DS__DS_MSKOR_B64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MSKOR_B64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_WRITE_B64(MachInst iFmt)
    {
        return new Inst_DS__DS_WRITE_B64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_WRITE_B64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_WRITE2_B64(MachInst iFmt)
    {
        return new Inst_DS__DS_WRITE2_B64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_WRITE2_B64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_WRITE2ST64_B64(MachInst iFmt)
    {
        return new Inst_DS__DS_WRITE2ST64_B64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_WRITE2ST64_B64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_CMPST_B64(MachInst iFmt)
    {
        return new Inst_DS__DS_CMPST_B64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_CMPST_B64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_CMPST_F64(MachInst iFmt)
    {
        return new Inst_DS__DS_CMPST_F64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_CMPST_F64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MIN_F64(MachInst iFmt)
    {
        return new Inst_DS__DS_MIN_F64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MIN_F64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MAX_F64(MachInst iFmt)
    {
        return new Inst_DS__DS_MAX_F64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MAX_F64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_WRITE_B8_D16_HI(MachInst iFmt)
    {
        return new Inst_DS__DS_WRITE_B8_D16_HI(&iFmt->iFmt_DS);
    }

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_WRITE_B16_D16_HI(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_READ_U8_D16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_READ_U8_D16_HI(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_READ_I8_D16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_READ_I8_D16_HI(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_READ_U16_D16(MachInst iFmt)
    {
        return new Inst_DS__DS_READ_U16_D16(&iFmt->iFmt_DS);
    }

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_READ_U16_D16_HI(MachInst iFmt)
    {
        return new Inst_DS__DS_READ_U16_D16_HI(&iFmt->iFmt_DS);
    }

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_ADD_RTN_U64(MachInst iFmt)
    {
        return new Inst_DS__DS_ADD_RTN_U64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_ADD_RTN_U64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_SUB_RTN_U64(MachInst iFmt)
    {
        return new Inst_DS__DS_SUB_RTN_U64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_SUB_RTN_U64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_RSUB_RTN_U64(MachInst iFmt)
    {
        return new Inst_DS__DS_RSUB_RTN_U64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_RSUB_RTN_U64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_INC_RTN_U64(MachInst iFmt)
    {
        return new Inst_DS__DS_INC_RTN_U64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_INC_RTN_U64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_DEC_RTN_U64(MachInst iFmt)
    {
        return new Inst_DS__DS_DEC_RTN_U64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_DEC_RTN_U64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MIN_RTN_I64(MachInst iFmt)
    {
        return new Inst_DS__DS_MIN_RTN_I64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MIN_RTN_I64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MAX_RTN_I64(MachInst iFmt)
    {
        return new Inst_DS__DS_MAX_RTN_I64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MAX_RTN_I64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MIN_RTN_U64(MachInst iFmt)
    {
        return new Inst_DS__DS_MIN_RTN_U64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MIN_RTN_U64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MAX_RTN_U64(MachInst iFmt)
    {
        return new Inst_DS__DS_MAX_RTN_U64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MAX_RTN_U64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_AND_RTN_B64(MachInst iFmt)
    {
        return new Inst_DS__DS_AND_RTN_B64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_AND_RTN_B64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_OR_RTN_B64(MachInst iFmt)
    {
        return new Inst_DS__DS_OR_RTN_B64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_OR_RTN_B64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_XOR_RTN_B64(MachInst iFmt)
    {
        return new Inst_DS__DS_XOR_RTN_B64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_XOR_RTN_B64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MSKOR_RTN_B64(MachInst iFmt)
    {
        return new Inst_DS__DS_MSKOR_RTN_B64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MSKOR_RTN_B64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_WRXCHG_RTN_B64(MachInst iFmt)
    {
        return new Inst_DS__DS_WRXCHG_RTN_B64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_WRXCHG_RTN_B64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_WRXCHG2_RTN_B64(MachInst iFmt)
    {
        return new Inst_DS__DS_WRXCHG2_RTN_B64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_WRXCHG2_RTN_B64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_WRXCHG2ST64_RTN_B64(MachInst iFmt)
    {
        return new Inst_DS__DS_WRXCHG2ST64_RTN_B64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_WRXCHG2ST64_RTN_B64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_CMPST_RTN_B64(MachInst iFmt)
    {
        return new Inst_DS__DS_CMPST_RTN_B64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_CMPST_RTN_B64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_CMPST_RTN_F64(MachInst iFmt)
    {
        return new Inst_DS__DS_CMPST_RTN_F64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_CMPST_RTN_F64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MIN_RTN_F64(MachInst iFmt)
    {
        return new Inst_DS__DS_MIN_RTN_F64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MIN_RTN_F64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MAX_RTN_F64(MachInst iFmt)
    {
        return new Inst_DS__DS_MAX_RTN_F64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MAX_RTN_F64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_READ_B64(MachInst iFmt)
    {
        return new Inst_DS__DS_READ_B64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_READ_B64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_READ2_B64(MachInst iFmt)
    {
        return new Inst_DS__DS_READ2_B64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_READ2_B64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_READ2ST64_B64(MachInst iFmt)
    {
        return new Inst_DS__DS_READ2ST64_B64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_READ2ST64_B64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_CONDXCHG32_RTN_B64(MachInst iFmt)
    {
        return new Inst_DS__DS_CONDXCHG32_RTN_B64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_CONDXCHG32_RTN_B64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_ADD_SRC2_U32(MachInst iFmt)
    {
        return new Inst_DS__DS_ADD_SRC2_U32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_ADD_SRC2_U32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_SUB_SRC2_U32(MachInst iFmt)
    {
        return new Inst_DS__DS_SUB_SRC2_U32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_SUB_SRC2_U32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_RSUB_SRC2_U32(MachInst iFmt)
    {
        return new Inst_DS__DS_RSUB_SRC2_U32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_RSUB_SRC2_U32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_INC_SRC2_U32(MachInst iFmt)
    {
        return new Inst_DS__DS_INC_SRC2_U32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_INC_SRC2_U32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_DEC_SRC2_U32(MachInst iFmt)
    {
        return new Inst_DS__DS_DEC_SRC2_U32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_DEC_SRC2_U32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MIN_SRC2_I32(MachInst iFmt)
    {
        return new Inst_DS__DS_MIN_SRC2_I32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MIN_SRC2_I32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MAX_SRC2_I32(MachInst iFmt)
    {
        return new Inst_DS__DS_MAX_SRC2_I32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MAX_SRC2_I32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MIN_SRC2_U32(MachInst iFmt)
    {
        return new Inst_DS__DS_MIN_SRC2_U32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MIN_SRC2_U32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MAX_SRC2_U32(MachInst iFmt)
    {
        return new Inst_DS__DS_MAX_SRC2_U32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MAX_SRC2_U32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_AND_SRC2_B32(MachInst iFmt)
    {
        return new Inst_DS__DS_AND_SRC2_B32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_AND_SRC2_B32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_OR_SRC2_B32(MachInst iFmt)
    {
        return new Inst_DS__DS_OR_SRC2_B32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_OR_SRC2_B32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_XOR_SRC2_B32(MachInst iFmt)
    {
        return new Inst_DS__DS_XOR_SRC2_B32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_XOR_SRC2_B32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_WRITE_SRC2_B32(MachInst iFmt)
    {
        return new Inst_DS__DS_WRITE_SRC2_B32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_WRITE_SRC2_B32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MIN_SRC2_F32(MachInst iFmt)
    {
        return new Inst_DS__DS_MIN_SRC2_F32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MIN_SRC2_F32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MAX_SRC2_F32(MachInst iFmt)
    {
        return new Inst_DS__DS_MAX_SRC2_F32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MAX_SRC2_F32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_ADD_SRC2_F32(MachInst iFmt)
    {
        return new Inst_DS__DS_ADD_SRC2_F32(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_ADD_SRC2_F32

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_GWS_SEMA_RELEASE_ALL(MachInst iFmt)
    {
        return new Inst_DS__DS_GWS_SEMA_RELEASE_ALL(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_GWS_SEMA_RELEASE_ALL

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_GWS_INIT(MachInst iFmt)
    {
        return new Inst_DS__DS_GWS_INIT(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_GWS_INIT

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_GWS_SEMA_V(MachInst iFmt)
    {
        return new Inst_DS__DS_GWS_SEMA_V(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_GWS_SEMA_V

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_GWS_SEMA_BR(MachInst iFmt)
    {
        return new Inst_DS__DS_GWS_SEMA_BR(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_GWS_SEMA_BR

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_GWS_SEMA_P(MachInst iFmt)
    {
        return new Inst_DS__DS_GWS_SEMA_P(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_GWS_SEMA_P

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_GWS_BARRIER(MachInst iFmt)
    {
        return new Inst_DS__DS_GWS_BARRIER(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_GWS_BARRIER

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_READ_ADDTID_B32(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_CONSUME(MachInst iFmt)
    {
        return new Inst_DS__DS_CONSUME(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_CONSUME

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_APPEND(MachInst iFmt)
    {
        return new Inst_DS__DS_APPEND(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_APPEND

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_ORDERED_COUNT(MachInst iFmt)
    {
        return new Inst_DS__DS_ORDERED_COUNT(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_ORDERED_COUNT

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_ADD_SRC2_U64(MachInst iFmt)
    {
        return new Inst_DS__DS_ADD_SRC2_U64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_ADD_SRC2_U64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_SUB_SRC2_U64(MachInst iFmt)
    {
        return new Inst_DS__DS_SUB_SRC2_U64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_SUB_SRC2_U64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_RSUB_SRC2_U64(MachInst iFmt)
    {
        return new Inst_DS__DS_RSUB_SRC2_U64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_RSUB_SRC2_U64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_INC_SRC2_U64(MachInst iFmt)
    {
        return new Inst_DS__DS_INC_SRC2_U64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_INC_SRC2_U64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_DEC_SRC2_U64(MachInst iFmt)
    {
        return new Inst_DS__DS_DEC_SRC2_U64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_DEC_SRC2_U64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MIN_SRC2_I64(MachInst iFmt)
    {
        return new Inst_DS__DS_MIN_SRC2_I64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MIN_SRC2_I64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MAX_SRC2_I64(MachInst iFmt)
    {
        return new Inst_DS__DS_MAX_SRC2_I64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MAX_SRC2_I64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MIN_SRC2_U64(MachInst iFmt)
    {
        return new Inst_DS__DS_MIN_SRC2_U64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MIN_SRC2_U64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MAX_SRC2_U64(MachInst iFmt)
    {
        return new Inst_DS__DS_MAX_SRC2_U64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MAX_SRC2_U64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_AND_SRC2_B64(MachInst iFmt)
    {
        return new Inst_DS__DS_AND_SRC2_B64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_AND_SRC2_B64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_OR_SRC2_B64(MachInst iFmt)
    {
        return new Inst_DS__DS_OR_SRC2_B64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_OR_SRC2_B64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_XOR_SRC2_B64(MachInst iFmt)
    {
        return new Inst_DS__DS_XOR_SRC2_B64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_XOR_SRC2_B64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_WRITE_SRC2_B64(MachInst iFmt)
    {
        return new Inst_DS__DS_WRITE_SRC2_B64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_WRITE_SRC2_B64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MIN_SRC2_F64(MachInst iFmt)
    {
        return new Inst_DS__DS_MIN_SRC2_F64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MIN_SRC2_F64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_MAX_SRC2_F64(MachInst iFmt)
    {
        return new Inst_DS__DS_MAX_SRC2_F64(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_MAX_SRC2_F64

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_WRITE_B96(MachInst iFmt)
    {
        return new Inst_DS__DS_WRITE_B96(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_WRITE_B96

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_WRITE_B128(MachInst iFmt)
    {
        return new Inst_DS__DS_WRITE_B128(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_WRITE_B128

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_READ_B96(MachInst iFmt)
    {
        return new Inst_DS__DS_READ_B96(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_READ_B96

    GPUStaticInst*
    Decoder::decode_OP_DS__DS_READ_B128(MachInst iFmt)
    {
        return new Inst_DS__DS_READ_B128(&iFmt->iFmt_DS);
    } // decode_OP_DS__DS_READ_B128

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_LOAD_UBYTE(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_LOAD_UBYTE(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_LOAD_UBYTE

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_LOAD_SBYTE(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_LOAD_SBYTE(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_LOAD_SBYTE

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_LOAD_USHORT(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_LOAD_USHORT(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_LOAD_USHORT

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_LOAD_SSHORT(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_LOAD_SSHORT(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_LOAD_SSHORT

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_LOAD_DWORD(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_LOAD_DWORD(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_LOAD_DWORD

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_LOAD_DWORDX2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_LOAD_DWORDX2(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_LOAD_DWORDX2

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_LOAD_DWORDX3(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_LOAD_DWORDX3(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_LOAD_DWORDX3

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_LOAD_DWORDX4(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_LOAD_DWORDX4(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_LOAD_DWORDX4

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_STORE_BYTE(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_STORE_BYTE(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_STORE_BYTE

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_STORE_BYTE_D16_HI(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_STORE_SHORT(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_STORE_SHORT(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_STORE_SHORT

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_STORE_SHORT_D16_HI(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_STORE_SHORT_D16_HI(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_STORE_DWORD(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_STORE_DWORD(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_STORE_DWORD

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_STORE_DWORDX2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_STORE_DWORDX2(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_STORE_DWORDX2

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_STORE_DWORDX3(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_STORE_DWORDX3(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_STORE_DWORDX3

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_STORE_DWORDX4(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_STORE_DWORDX4(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_STORE_DWORDX4

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_LOAD_UBYTE_D16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_LOAD_UBYTE_D16_HI(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_LOAD_SBYTE_D16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_LOAD_SBYTE_D16_HI(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_LOAD_SHORT_D16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_LOAD_SHORT_D16_HI(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_ATOMIC_SWAP(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_SWAP(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_ATOMIC_SWAP

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_ATOMIC_CMPSWAP(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_CMPSWAP(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_ATOMIC_CMPSWAP

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_ATOMIC_ADD(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_ADD(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_ATOMIC_ADD

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_ATOMIC_SUB(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_SUB(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_ATOMIC_SUB

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_ATOMIC_SMIN(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_SMIN(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_ATOMIC_SMIN

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_ATOMIC_UMIN(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_UMIN(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_ATOMIC_UMIN

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_ATOMIC_SMAX(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_SMAX(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_ATOMIC_SMAX

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_ATOMIC_UMAX(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_UMAX(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_ATOMIC_UMAX

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_ATOMIC_AND(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_AND(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_ATOMIC_AND

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_ATOMIC_OR(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_OR(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_ATOMIC_OR

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_ATOMIC_XOR(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_XOR(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_ATOMIC_XOR

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_ATOMIC_INC(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_INC(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_ATOMIC_INC

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_ATOMIC_DEC(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_DEC(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_ATOMIC_DEC

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_ATOMIC_ADD_F64(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_ADD_F64(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_ATOMIC_ADD_F64

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_ATOMIC_MIN_F64(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_MIN_F64(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_ATOMIC_MIN_F64

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_ATOMIC_MAX_F64(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_MAX_F64(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_ATOMIC_MAX_F64

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_ATOMIC_SWAP_X2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_SWAP_X2(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_ATOMIC_SWAP_X2

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_ATOMIC_CMPSWAP_X2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_CMPSWAP_X2(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_ATOMIC_CMPSWAP_X2

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_ATOMIC_ADD_X2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_ADD_X2(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_ATOMIC_ADD_X2

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_ATOMIC_SUB_X2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_SUB_X2(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_ATOMIC_SUB_X2

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_ATOMIC_SMIN_X2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_SMIN_X2(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_ATOMIC_SMIN_X2

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_ATOMIC_UMIN_X2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_UMIN_X2(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_ATOMIC_UMIN_X2

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_ATOMIC_SMAX_X2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_SMAX_X2(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_ATOMIC_SMAX_X2

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_ATOMIC_UMAX_X2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_UMAX_X2(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_ATOMIC_UMAX_X2

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_ATOMIC_AND_X2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_AND_X2(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_ATOMIC_AND_X2

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_ATOMIC_OR_X2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_OR_X2(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_ATOMIC_OR_X2

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_ATOMIC_XOR_X2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_XOR_X2(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_ATOMIC_XOR_X2

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_ATOMIC_INC_X2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_INC_X2(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_ATOMIC_INC_X2

    GPUStaticInst*
    Decoder::decode_OP_FLAT__FLAT_ATOMIC_DEC_X2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_DEC_X2(&iFmt->iFmt_FLAT);
    } // decode_OP_FLAT__FLAT_ATOMIC_DEC_X2

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_LOAD_UBYTE(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_LOAD_UBYTE(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_LOAD_SBYTE(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_LOAD_SBYTE(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_LOAD_USHORT(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_LOAD_USHORT(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_LOAD_SSHORT(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_LOAD_SSHORT(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_LOAD_DWORD(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_LOAD_DWORD(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_LOAD_DWORDX2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_LOAD_DWORDX2(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_LOAD_DWORDX3(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_LOAD_DWORDX3(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_LOAD_DWORDX4(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_LOAD_DWORDX4(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_STORE_BYTE(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_STORE_BYTE(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_STORE_BYTE_D16_HI(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_STORE_SHORT(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_STORE_SHORT(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_STORE_SHORT_D16_HI(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_STORE_SHORT_D16_HI(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_STORE_DWORD(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_STORE_DWORD(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_STORE_DWORDX2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_STORE_DWORDX2(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_STORE_DWORDX3(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_STORE_DWORDX3(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_STORE_DWORDX4(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_STORE_DWORDX4(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_LOAD_UBYTE_D16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_LOAD_UBYTE_D16_HI(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_LOAD_SBYTE_D16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_LOAD_SBYTE_D16_HI(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_LOAD_SHORT_D16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_LOAD_SHORT_D16_HI(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_SWAP(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_SWAP(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_CMPSWAP(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_CMPSWAP(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_ADD(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_ADD(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_SUB(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_SUB(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_SMIN(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_SMIN(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_UMIN(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_UMIN(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_SMAX(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_SMAX(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_UMAX(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_UMAX(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_AND(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_AND(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_OR(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_OR(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_XOR(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_XOR(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_INC(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_INC(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_DEC(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_DEC(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_ADD_F32(MachInst iFmt)
    {
        // Note: There is no flat_atomic_add_f32 as of MI200. However, gem5
        // impelements all global and scratch instructions as Inst_FLAT.
        return new Inst_FLAT__FLAT_ATOMIC_ADD_F32(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_PK_ADD_F16(MachInst iFmt)
    {
        // Note: There is no flat_atomic_pk_add_f16 as of MI200. However, gem5
        // impelements all global and scratch instructions as Inst_FLAT.
        return new Inst_FLAT__FLAT_ATOMIC_PK_ADD_F16(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_ADD_F64(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_ADD_F64(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_MIN_F64(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_MIN_F64(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_MAX_F64(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_MAX_F64(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_SWAP_X2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_SWAP_X2(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_CMPSWAP_X2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_CMPSWAP_X2(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_ADD_X2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_ADD_X2(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_SUB_X2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_SUB_X2(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_SMIN_X2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_SMIN_X2(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_UMIN_X2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_UMIN_X2(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_SMAX_X2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_SMAX_X2(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_UMAX_X2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_UMAX_X2(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_AND_X2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_AND_X2(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_OR_X2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_OR_X2(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_XOR_X2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_XOR_X2(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_INC_X2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_INC_X2(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_GLOBAL__GLOBAL_ATOMIC_DEC_X2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_ATOMIC_DEC_X2(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_LOAD(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_LOAD(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_LOAD

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_LOAD_MIP(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_LOAD_MIP(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_LOAD_MIP

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_LOAD_PCK(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_LOAD_PCK(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_LOAD_PCK

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_LOAD_PCK_SGN(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_LOAD_PCK_SGN(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_LOAD_PCK_SGN

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_LOAD_MIP_PCK(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_LOAD_MIP_PCK(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_LOAD_MIP_PCK

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_LOAD_MIP_PCK_SGN(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_LOAD_MIP_PCK_SGN(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_LOAD_MIP_PCK_SGN

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_STORE(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_STORE(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_STORE

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_STORE_MIP(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_STORE_MIP(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_STORE_MIP

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_STORE_PCK(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_STORE_PCK(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_STORE_PCK

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_STORE_MIP_PCK(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_STORE_MIP_PCK(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_STORE_MIP_PCK

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_GET_RESINFO(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_GET_RESINFO(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_GET_RESINFO

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_ATOMIC_SWAP(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_ATOMIC_SWAP(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_ATOMIC_SWAP

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_ATOMIC_CMPSWAP(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_ATOMIC_CMPSWAP(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_ATOMIC_CMPSWAP

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_ATOMIC_ADD(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_ATOMIC_ADD(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_ATOMIC_ADD

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_ATOMIC_SUB(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_ATOMIC_SUB(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_ATOMIC_SUB

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_ATOMIC_SMIN(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_ATOMIC_SMIN(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_ATOMIC_SMIN

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_ATOMIC_UMIN(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_ATOMIC_UMIN(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_ATOMIC_UMIN

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_ATOMIC_SMAX(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_ATOMIC_SMAX(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_ATOMIC_SMAX

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_ATOMIC_UMAX(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_ATOMIC_UMAX(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_ATOMIC_UMAX

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_ATOMIC_AND(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_ATOMIC_AND(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_ATOMIC_AND

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_ATOMIC_OR(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_ATOMIC_OR(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_ATOMIC_OR

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_ATOMIC_XOR(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_ATOMIC_XOR(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_ATOMIC_XOR

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_ATOMIC_INC(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_ATOMIC_INC(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_ATOMIC_INC

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_ATOMIC_DEC(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_ATOMIC_DEC(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_ATOMIC_DEC

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_CL(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_CL(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_CL

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_D(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_D(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_D

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_D_CL(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_D_CL(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_D_CL

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_L(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_L(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_L

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_B(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_B(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_B

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_B_CL(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_B_CL(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_B_CL

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_LZ(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_LZ(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_LZ

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_C(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_C

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_CL(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_C_CL(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_C_CL

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_D(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_C_D(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_C_D

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_D_CL(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_C_D_CL(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_C_D_CL

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_L(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_C_L(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_C_L

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_B(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_C_B(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_C_B

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_B_CL(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_C_B_CL(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_C_B_CL

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_LZ(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_C_LZ(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_C_LZ

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_CL_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_CL_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_CL_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_D_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_D_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_D_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_D_CL_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_D_CL_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_D_CL_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_L_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_L_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_L_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_B_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_B_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_B_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_B_CL_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_B_CL_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_B_CL_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_LZ_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_LZ_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_LZ_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_C_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_C_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_CL_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_C_CL_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_C_CL_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_D_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_C_D_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_C_D_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_D_CL_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_C_D_CL_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_C_D_CL_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_L_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_C_L_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_C_L_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_B_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_C_B_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_C_B_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_B_CL_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_C_B_CL_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_C_B_CL_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_LZ_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_C_LZ_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_C_LZ_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_GATHER4(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_GATHER4(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_GATHER4

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_GATHER4_CL(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_GATHER4_CL(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_GATHER4_CL

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_GATHER4H(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_GATHER4_L(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_GATHER4_L(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_GATHER4_L

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_GATHER4_B(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_GATHER4_B(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_GATHER4_B

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_GATHER4_B_CL(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_GATHER4_B_CL(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_GATHER4_B_CL

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_GATHER4_LZ(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_GATHER4_LZ(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_GATHER4_LZ

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_GATHER4_C(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_GATHER4_C(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_GATHER4_C

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_GATHER4_C_CL(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_GATHER4_C_CL(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_GATHER4_C_CL

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_GATHER4H_PCK(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_GATHER8H_PCK(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_GATHER4_C_L(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_GATHER4_C_L(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_GATHER4_C_L

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_GATHER4_C_B(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_GATHER4_C_B(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_GATHER4_C_B

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_GATHER4_C_B_CL(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_GATHER4_C_B_CL(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_GATHER4_C_B_CL

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_GATHER4_C_LZ(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_GATHER4_C_LZ(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_GATHER4_C_LZ

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_GATHER4_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_GATHER4_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_GATHER4_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_GATHER4_CL_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_GATHER4_CL_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_GATHER4_CL_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_GATHER4_L_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_GATHER4_L_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_GATHER4_L_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_GATHER4_B_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_GATHER4_B_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_GATHER4_B_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_GATHER4_B_CL_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_GATHER4_B_CL_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_GATHER4_B_CL_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_GATHER4_LZ_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_GATHER4_LZ_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_GATHER4_LZ_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_GATHER4_C_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_GATHER4_C_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_GATHER4_C_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_GATHER4_C_CL_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_GATHER4_C_CL_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_GATHER4_C_CL_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_GATHER4_C_L_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_GATHER4_C_L_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_GATHER4_C_L_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_GATHER4_C_B_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_GATHER4_C_B_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_GATHER4_C_B_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_GATHER4_C_B_CL_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_GATHER4_C_B_CL_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_GATHER4_C_B_CL_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_GATHER4_C_LZ_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_GATHER4_C_LZ_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_GATHER4_C_LZ_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_GET_LOD(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_GET_LOD(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_GET_LOD

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_CD(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_CD(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_CD

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_CD_CL(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_CD_CL(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_CD_CL

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_CD(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_C_CD(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_C_CD

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_CD_CL(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_C_CD_CL(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_C_CD_CL

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_CD_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_CD_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_CD_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_CD_CL_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_CD_CL_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_CD_CL_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_CD_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_C_CD_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_C_CD_O

    GPUStaticInst*
    Decoder::decode_OP_MIMG__IMAGE_SAMPLE_C_CD_CL_O(MachInst iFmt)
    {
        return new Inst_MIMG__IMAGE_SAMPLE_C_CD_CL_O(&iFmt->iFmt_MIMG);
    } // decode_OP_MIMG__IMAGE_SAMPLE_C_CD_CL_O

    GPUStaticInst*
    Decoder::decode_OP_MTBUF__TBUFFER_LOAD_FORMAT_X(MachInst iFmt)
    {
        return new Inst_MTBUF__TBUFFER_LOAD_FORMAT_X(&iFmt->iFmt_MTBUF);
    } // decode_OP_MTBUF__TBUFFER_LOAD_FORMAT_X

    GPUStaticInst*
    Decoder::decode_OP_MTBUF__TBUFFER_LOAD_FORMAT_XY(MachInst iFmt)
    {
        return new Inst_MTBUF__TBUFFER_LOAD_FORMAT_XY(&iFmt->iFmt_MTBUF);
    } // decode_OP_MTBUF__TBUFFER_LOAD_FORMAT_XY

    GPUStaticInst*
    Decoder::decode_OP_MTBUF__TBUFFER_LOAD_FORMAT_XYZ(MachInst iFmt)
    {
        return new Inst_MTBUF__TBUFFER_LOAD_FORMAT_XYZ(&iFmt->iFmt_MTBUF);
    } // decode_OP_MTBUF__TBUFFER_LOAD_FORMAT_XYZ

    GPUStaticInst*
    Decoder::decode_OP_MTBUF__TBUFFER_LOAD_FORMAT_XYZW(MachInst iFmt)
    {
        return new Inst_MTBUF__TBUFFER_LOAD_FORMAT_XYZW(&iFmt->iFmt_MTBUF);
    } // decode_OP_MTBUF__TBUFFER_LOAD_FORMAT_XYZW

    GPUStaticInst*
    Decoder::decode_OP_MTBUF__TBUFFER_STORE_FORMAT_X(MachInst iFmt)
    {
        return new Inst_MTBUF__TBUFFER_STORE_FORMAT_X(&iFmt->iFmt_MTBUF);
    } // decode_OP_MTBUF__TBUFFER_STORE_FORMAT_X

    GPUStaticInst*
    Decoder::decode_OP_MTBUF__TBUFFER_STORE_FORMAT_XY(MachInst iFmt)
    {
        return new Inst_MTBUF__TBUFFER_STORE_FORMAT_XY(&iFmt->iFmt_MTBUF);
    } // decode_OP_MTBUF__TBUFFER_STORE_FORMAT_XY

    GPUStaticInst*
    Decoder::decode_OP_MTBUF__TBUFFER_STORE_FORMAT_XYZ(MachInst iFmt)
    {
        return new Inst_MTBUF__TBUFFER_STORE_FORMAT_XYZ(&iFmt->iFmt_MTBUF);
    } // decode_OP_MTBUF__TBUFFER_STORE_FORMAT_XYZ

    GPUStaticInst*
    Decoder::decode_OP_MTBUF__TBUFFER_STORE_FORMAT_XYZW(MachInst iFmt)
    {
        return new Inst_MTBUF__TBUFFER_STORE_FORMAT_XYZW(&iFmt->iFmt_MTBUF);
    } // decode_OP_MTBUF__TBUFFER_STORE_FORMAT_XYZW

    GPUStaticInst*
    Decoder::decode_OP_MTBUF__TBUFFER_LOAD_FORMAT_D16_X(MachInst iFmt)
    {
        return new Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_X(&iFmt->iFmt_MTBUF);
    } // decode_OP_MTBUF__TBUFFER_LOAD_FORMAT_D16_X

    GPUStaticInst*
    Decoder::decode_OP_MTBUF__TBUFFER_LOAD_FORMAT_D16_XY(MachInst iFmt)
    {
        return new Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XY(&iFmt->iFmt_MTBUF);
    } // decode_OP_MTBUF__TBUFFER_LOAD_FORMAT_D16_XY

    GPUStaticInst*
    Decoder::decode_OP_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZ(MachInst iFmt)
    {
        return new Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZ(&iFmt->iFmt_MTBUF);
    } // decode_OP_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZ

    GPUStaticInst*
    Decoder::decode_OP_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZW(MachInst iFmt)
    {
        return new Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZW(&iFmt->iFmt_MTBUF);
    } // decode_OP_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZW

    GPUStaticInst*
    Decoder::decode_OP_MTBUF__TBUFFER_STORE_FORMAT_D16_X(MachInst iFmt)
    {
        return new Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_X(&iFmt->iFmt_MTBUF);
    } // decode_OP_MTBUF__TBUFFER_STORE_FORMAT_D16_X

    GPUStaticInst*
    Decoder::decode_OP_MTBUF__TBUFFER_STORE_FORMAT_D16_XY(MachInst iFmt)
    {
        return new Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XY(&iFmt->iFmt_MTBUF);
    } // decode_OP_MTBUF__TBUFFER_STORE_FORMAT_D16_XY

    GPUStaticInst*
    Decoder::decode_OP_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZ(MachInst iFmt)
    {
        return new Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZ(&iFmt->iFmt_MTBUF);
    } // decode_OP_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZ

    GPUStaticInst*
    Decoder::decode_OP_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZW(MachInst iFmt)
    {
        return new
            Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZW(&iFmt->iFmt_MTBUF);
    } // decode_OP_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZW

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_LOAD_FORMAT_X(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_LOAD_FORMAT_X(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_LOAD_FORMAT_X

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_LOAD_FORMAT_XY(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_LOAD_FORMAT_XY(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_LOAD_FORMAT_XY

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_LOAD_FORMAT_XYZ(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_LOAD_FORMAT_XYZ(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_LOAD_FORMAT_XYZ

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_LOAD_FORMAT_XYZW(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_LOAD_FORMAT_XYZW(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_LOAD_FORMAT_XYZW

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_STORE_FORMAT_X(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_STORE_FORMAT_X(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_STORE_FORMAT_X

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_STORE_FORMAT_XY(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_STORE_FORMAT_XY(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_STORE_FORMAT_XY

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_STORE_FORMAT_XYZ(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_STORE_FORMAT_XYZ(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_STORE_FORMAT_XYZ

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_STORE_FORMAT_XYZW(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_STORE_FORMAT_XYZW(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_STORE_FORMAT_XYZW

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_LOAD_FORMAT_D16_X(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_X(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_LOAD_FORMAT_D16_X

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_LOAD_FORMAT_D16_XY(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XY(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_LOAD_FORMAT_D16_XY

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZ(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZ(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZ

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZW(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZW(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZW

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_STORE_FORMAT_D16_X(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_STORE_FORMAT_D16_X(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_STORE_FORMAT_D16_X

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_STORE_FORMAT_D16_XY(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XY(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_STORE_FORMAT_D16_XY

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_STORE_FORMAT_D16_XYZ(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XYZ(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_STORE_FORMAT_D16_XYZ

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_STORE_FORMAT_D16_XYZW(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XYZW(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_STORE_FORMAT_D16_XYZW

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_LOAD_UBYTE(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_LOAD_UBYTE(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_LOAD_UBYTE

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_LOAD_SBYTE(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_LOAD_SBYTE(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_LOAD_SBYTE

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_LOAD_USHORT(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_LOAD_USHORT(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_LOAD_USHORT

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_LOAD_SSHORT(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_LOAD_SSHORT(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_LOAD_SSHORT

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_LOAD_DWORD(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_LOAD_DWORD(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_LOAD_DWORD

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_LOAD_DWORDX2(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_LOAD_DWORDX2(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_LOAD_DWORDX2

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_LOAD_DWORDX3(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_LOAD_DWORDX3(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_LOAD_DWORDX3

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_LOAD_DWORDX4(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_LOAD_DWORDX4(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_LOAD_DWORDX4

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_STORE_BYTE(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_STORE_BYTE(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_STORE_BYTE

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_STORE_BYTE_D16_HI(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_STORE_SHORT(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_STORE_SHORT(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_STORE_SHORT

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_STORE_SHORT_D16_HI(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_STORE_DWORD(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_STORE_DWORD(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_STORE_DWORD

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_STORE_DWORDX2(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_STORE_DWORDX2(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_STORE_DWORDX2

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_STORE_DWORDX3(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_STORE_DWORDX3(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_STORE_DWORDX3

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_STORE_DWORDX4(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_STORE_DWORDX4(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_STORE_DWORDX4

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_STORE_LDS_DWORD(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_STORE_LDS_DWORD(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_STORE_LDS_DWORD

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_WBINVL1(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_WBINVL1(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_WBINVL1

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_WBINVL1_VOL(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_WBINVL1_VOL(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_WBINVL1_VOL

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_SWAP(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_ATOMIC_SWAP(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_ATOMIC_SWAP

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_CMPSWAP(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_ATOMIC_CMPSWAP(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_ATOMIC_CMPSWAP

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_ADD(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_ATOMIC_ADD(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_ATOMIC_ADD

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_SUB(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_ATOMIC_SUB(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_ATOMIC_SUB

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_SMIN(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_ATOMIC_SMIN(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_ATOMIC_SMIN

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_UMIN(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_ATOMIC_UMIN(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_ATOMIC_UMIN

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_SMAX(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_ATOMIC_SMAX(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_ATOMIC_SMAX

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_UMAX(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_ATOMIC_UMAX(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_ATOMIC_UMAX

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_AND(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_ATOMIC_AND(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_ATOMIC_AND

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_OR(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_ATOMIC_OR(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_ATOMIC_OR

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_XOR(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_ATOMIC_XOR(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_ATOMIC_XOR

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_INC(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_ATOMIC_INC(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_ATOMIC_INC

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_DEC(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_ATOMIC_DEC(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_ATOMIC_DEC

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_SWAP_X2(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_ATOMIC_SWAP_X2(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_ATOMIC_SWAP_X2

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_CMPSWAP_X2(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_ATOMIC_CMPSWAP_X2(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_ATOMIC_CMPSWAP_X2

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_ADD_X2(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_ATOMIC_ADD_X2(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_ATOMIC_ADD_X2

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_SUB_X2(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_ATOMIC_SUB_X2(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_ATOMIC_SUB_X2

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_SMIN_X2(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_ATOMIC_SMIN_X2(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_ATOMIC_SMIN_X2

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_UMIN_X2(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_ATOMIC_UMIN_X2(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_ATOMIC_UMIN_X2

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_SMAX_X2(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_ATOMIC_SMAX_X2(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_ATOMIC_SMAX_X2

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_UMAX_X2(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_ATOMIC_UMAX_X2(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_ATOMIC_UMAX_X2

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_AND_X2(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_ATOMIC_AND_X2(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_ATOMIC_AND_X2

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_OR_X2(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_ATOMIC_OR_X2(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_ATOMIC_OR_X2

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_XOR_X2(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_ATOMIC_XOR_X2(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_ATOMIC_XOR_X2

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_INC_X2(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_ATOMIC_INC_X2(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_ATOMIC_INC_X2

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_ATOMIC_DEC_X2(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_ATOMIC_DEC_X2(&iFmt->iFmt_MUBUF);
    } // decode_OP_MUBUF__BUFFER_ATOMIC_DEC_X2

    GPUStaticInst*
    Decoder::decode_OP_SCRATCH__SCRATCH_LOAD_UBYTE(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_LOAD_UBYTE(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_SCRATCH__SCRATCH_LOAD_SBYTE(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_LOAD_SBYTE(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_SCRATCH__SCRATCH_LOAD_USHORT(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_LOAD_USHORT(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_SCRATCH__SCRATCH_LOAD_SSHORT(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_LOAD_SSHORT(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_SCRATCH__SCRATCH_LOAD_DWORD(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_LOAD_DWORD(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_SCRATCH__SCRATCH_LOAD_DWORDX2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_LOAD_DWORDX2(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_SCRATCH__SCRATCH_LOAD_DWORDX3(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_LOAD_DWORDX3(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_SCRATCH__SCRATCH_LOAD_DWORDX4(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_LOAD_DWORDX4(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_SCRATCH__SCRATCH_STORE_BYTE(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_STORE_BYTE(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_SCRATCH__SCRATCH_STORE_BYTE_D16_HI(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SCRATCH__SCRATCH_STORE_SHORT(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_STORE_SHORT(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_SCRATCH__SCRATCH_STORE_SHORT_D16_HI(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_STORE_SHORT_D16_HI(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_SCRATCH__SCRATCH_STORE_DWORD(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_STORE_DWORD(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_SCRATCH__SCRATCH_STORE_DWORDX2(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_STORE_DWORDX2(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_SCRATCH__SCRATCH_STORE_DWORDX3(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_STORE_DWORDX3(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_SCRATCH__SCRATCH_STORE_DWORDX4(MachInst iFmt)
    {
        return new Inst_FLAT__FLAT_STORE_DWORDX4(&iFmt->iFmt_FLAT);
    }

    GPUStaticInst*
    Decoder::decode_OP_SCRATCH__SCRATCH_LOAD_UBYTE_D16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SCRATCH__SCRATCH_LOAD_UBYTE_D16_HI(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SCRATCH__SCRATCH_LOAD_SBYTE_D16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SCRATCH__SCRATCH_LOAD_SBYTE_D16_HI(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SCRATCH__SCRATCH_LOAD_SHORT_D16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SCRATCH__SCRATCH_LOAD_SHORT_D16_HI(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_LOAD_DWORD(MachInst iFmt)
    {
        return new Inst_SMEM__S_LOAD_DWORD(&iFmt->iFmt_SMEM);
    } // decode_OP_SMEM__S_LOAD_DWORD

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_LOAD_DWORDX2(MachInst iFmt)
    {
        return new Inst_SMEM__S_LOAD_DWORDX2(&iFmt->iFmt_SMEM);
    } // decode_OP_SMEM__S_LOAD_DWORDX2

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_LOAD_DWORDX4(MachInst iFmt)
    {
        return new Inst_SMEM__S_LOAD_DWORDX4(&iFmt->iFmt_SMEM);
    } // decode_OP_SMEM__S_LOAD_DWORDX4

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_LOAD_DWORDX8(MachInst iFmt)
    {
        return new Inst_SMEM__S_LOAD_DWORDX8(&iFmt->iFmt_SMEM);
    } // decode_OP_SMEM__S_LOAD_DWORDX8

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_LOAD_DWORDX16(MachInst iFmt)
    {
        return new Inst_SMEM__S_LOAD_DWORDX16(&iFmt->iFmt_SMEM);
    } // decode_OP_SMEM__S_LOAD_DWORDX16

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_SCRATCH_LOAD_DWORD(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_SCRATCH_LOAD_DWORDX2(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_SCRATCH_LOAD_DWORDX4(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_LOAD_DWORD(MachInst iFmt)
    {
        return new Inst_SMEM__S_BUFFER_LOAD_DWORD(&iFmt->iFmt_SMEM);
    } // decode_OP_SMEM__S_BUFFER_LOAD_DWORD

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_LOAD_DWORDX2(MachInst iFmt)
    {
        return new Inst_SMEM__S_BUFFER_LOAD_DWORDX2(&iFmt->iFmt_SMEM);
    } // decode_OP_SMEM__S_BUFFER_LOAD_DWORDX2

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_LOAD_DWORDX4(MachInst iFmt)
    {
        return new Inst_SMEM__S_BUFFER_LOAD_DWORDX4(&iFmt->iFmt_SMEM);
    } // decode_OP_SMEM__S_BUFFER_LOAD_DWORDX4

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_LOAD_DWORDX8(MachInst iFmt)
    {
        return new Inst_SMEM__S_BUFFER_LOAD_DWORDX8(&iFmt->iFmt_SMEM);
    } // decode_OP_SMEM__S_BUFFER_LOAD_DWORDX8

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_LOAD_DWORDX16(MachInst iFmt)
    {
        return new Inst_SMEM__S_BUFFER_LOAD_DWORDX16(&iFmt->iFmt_SMEM);
    } // decode_OP_SMEM__S_BUFFER_LOAD_DWORDX16

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_STORE_DWORD(MachInst iFmt)
    {
        return new Inst_SMEM__S_STORE_DWORD(&iFmt->iFmt_SMEM);
    } // decode_OP_SMEM__S_STORE_DWORD

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_STORE_DWORDX2(MachInst iFmt)
    {
        return new Inst_SMEM__S_STORE_DWORDX2(&iFmt->iFmt_SMEM);
    } // decode_OP_SMEM__S_STORE_DWORDX2

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_STORE_DWORDX4(MachInst iFmt)
    {
        return new Inst_SMEM__S_STORE_DWORDX4(&iFmt->iFmt_SMEM);
    } // decode_OP_SMEM__S_STORE_DWORDX4

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_SCRATCH_STORE_DWORD(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_SCRATCH_STORE_DWORDX2(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_SCRATCH_STORE_DWORDX4(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_STORE_DWORD(MachInst iFmt)
    {
        return new Inst_SMEM__S_BUFFER_STORE_DWORD(&iFmt->iFmt_SMEM);
    } // decode_OP_SMEM__S_BUFFER_STORE_DWORD

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_STORE_DWORDX2(MachInst iFmt)
    {
        return new Inst_SMEM__S_BUFFER_STORE_DWORDX2(&iFmt->iFmt_SMEM);
    } // decode_OP_SMEM__S_BUFFER_STORE_DWORDX2

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_STORE_DWORDX4(MachInst iFmt)
    {
        return new Inst_SMEM__S_BUFFER_STORE_DWORDX4(&iFmt->iFmt_SMEM);
    } // decode_OP_SMEM__S_BUFFER_STORE_DWORDX4

    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_LOAD_UBYTE_D16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }
    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_LOAD_UBYTE_D16_HI(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }
    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_LOAD_SBYTE_D16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }
    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_LOAD_SBYTE_D16_HI(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }
    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_LOAD_SHORT_D16(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_LOAD_SHORT_D16(&iFmt->iFmt_MUBUF);
    }
    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_LOAD_SHORT_D16_HI(MachInst iFmt)
    {
        return new Inst_MUBUF__BUFFER_LOAD_SHORT_D16_HI(&iFmt->iFmt_MUBUF);
    }
    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_LOAD_FORMAT_D16_HI_X(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }
    GPUStaticInst*
    Decoder::decode_OP_MUBUF__BUFFER_STORE_FORMAT_D16_HI_X(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_DCACHE_INV(MachInst iFmt)
    {
        return new Inst_SMEM__S_DCACHE_INV(&iFmt->iFmt_SMEM);
    } // decode_OP_SMEM__S_DCACHE_INV

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_DCACHE_WB(MachInst iFmt)
    {
        return new Inst_SMEM__S_DCACHE_WB(&iFmt->iFmt_SMEM);
    } // decode_OP_SMEM__S_DCACHE_WB

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_DCACHE_INV_VOL(MachInst iFmt)
    {
        return new Inst_SMEM__S_DCACHE_INV_VOL(&iFmt->iFmt_SMEM);
    } // decode_OP_SMEM__S_DCACHE_INV_VOL

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_DCACHE_WB_VOL(MachInst iFmt)
    {
        return new Inst_SMEM__S_DCACHE_WB_VOL(&iFmt->iFmt_SMEM);
    } // decode_OP_SMEM__S_DCACHE_WB_VOL

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_MEMTIME(MachInst iFmt)
    {
        return new Inst_SMEM__S_MEMTIME(&iFmt->iFmt_SMEM);
    } // decode_OP_SMEM__S_MEMTIME

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_MEMREALTIME(MachInst iFmt)
    {
        return new Inst_SMEM__S_MEMREALTIME(&iFmt->iFmt_SMEM);
    } // decode_OP_SMEM__S_MEMREALTIME

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_ATC_PROBE(MachInst iFmt)
    {
        return new Inst_SMEM__S_ATC_PROBE(&iFmt->iFmt_SMEM);
    } // decode_OP_SMEM__S_ATC_PROBE

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_ATC_PROBE_BUFFER(MachInst iFmt)
    {
        return new Inst_SMEM__S_ATC_PROBE_BUFFER(&iFmt->iFmt_SMEM);
    } // decode_OP_SMEM__S_ATC_PROBE_BUFFER

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_DCACHE_DISCARD(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_DCACHE_DISCARD_X2(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_SWAP(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_CMPSWAP(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_ADD(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_SUB(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_SMIN(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_UMIN(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_SMAX(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_UMAX(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_AND(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_OR(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_XOR(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_INC(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_DEC(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_SWAP_X2(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_CMPSWAP_X2(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_ADD_X2(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_SUB_X2(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_SMIN_X2(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_UMIN_X2(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_SMAX_X2(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_UMAX_X2(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_AND_X2(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_OR_X2(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_XOR_X2(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_INC_X2(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_BUFFER_ATOMIC_DEC_X2(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_ATOMIC_SWAP(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_ATOMIC_CMPSWAP(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_ATOMIC_ADD(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_ATOMIC_SUB(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_ATOMIC_SMIN(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_ATOMIC_UMIN(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_ATOMIC_SMAX(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_ATOMIC_UMAX(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_ATOMIC_AND(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_ATOMIC_OR(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_ATOMIC_XOR(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_ATOMIC_INC(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_ATOMIC_DEC(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_ATOMIC_SWAP_X2(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_ATOMIC_CMPSWAP_X2(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_ATOMIC_ADD_X2(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_ATOMIC_SUB_X2(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_ATOMIC_SMIN_X2(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_ATOMIC_UMIN_X2(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_ATOMIC_SMAX_X2(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_ATOMIC_UMAX_X2(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_ATOMIC_AND_X2(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_ATOMIC_OR_X2(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_ATOMIC_XOR_X2(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_ATOMIC_INC_X2(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SMEM__S_ATOMIC_DEC_X2(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_MOV_B32(MachInst iFmt)
    {
        return new Inst_SOP1__S_MOV_B32(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_MOV_B32

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_MOV_B64(MachInst iFmt)
    {
        return new Inst_SOP1__S_MOV_B64(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_MOV_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_CMOV_B32(MachInst iFmt)
    {
        return new Inst_SOP1__S_CMOV_B32(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_CMOV_B32

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_CMOV_B64(MachInst iFmt)
    {
        return new Inst_SOP1__S_CMOV_B64(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_CMOV_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_NOT_B32(MachInst iFmt)
    {
        return new Inst_SOP1__S_NOT_B32(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_NOT_B32

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_NOT_B64(MachInst iFmt)
    {
        return new Inst_SOP1__S_NOT_B64(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_NOT_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_WQM_B32(MachInst iFmt)
    {
        return new Inst_SOP1__S_WQM_B32(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_WQM_B32

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_WQM_B64(MachInst iFmt)
    {
        return new Inst_SOP1__S_WQM_B64(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_WQM_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_BREV_B32(MachInst iFmt)
    {
        return new Inst_SOP1__S_BREV_B32(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_BREV_B32

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_BREV_B64(MachInst iFmt)
    {
        return new Inst_SOP1__S_BREV_B64(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_BREV_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_BCNT0_I32_B32(MachInst iFmt)
    {
        return new Inst_SOP1__S_BCNT0_I32_B32(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_BCNT0_I32_B32

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_BCNT0_I32_B64(MachInst iFmt)
    {
        return new Inst_SOP1__S_BCNT0_I32_B64(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_BCNT0_I32_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_BCNT1_I32_B32(MachInst iFmt)
    {
        return new Inst_SOP1__S_BCNT1_I32_B32(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_BCNT1_I32_B32

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_BCNT1_I32_B64(MachInst iFmt)
    {
        return new Inst_SOP1__S_BCNT1_I32_B64(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_BCNT1_I32_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_FF0_I32_B32(MachInst iFmt)
    {
        return new Inst_SOP1__S_FF0_I32_B32(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_FF0_I32_B32

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_FF0_I32_B64(MachInst iFmt)
    {
        return new Inst_SOP1__S_FF0_I32_B64(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_FF0_I32_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_FF1_I32_B32(MachInst iFmt)
    {
        return new Inst_SOP1__S_FF1_I32_B32(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_FF1_I32_B32

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_FF1_I32_B64(MachInst iFmt)
    {
        return new Inst_SOP1__S_FF1_I32_B64(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_FF1_I32_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_FLBIT_I32_B32(MachInst iFmt)
    {
        return new Inst_SOP1__S_FLBIT_I32_B32(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_FLBIT_I32_B32

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_FLBIT_I32_B64(MachInst iFmt)
    {
        return new Inst_SOP1__S_FLBIT_I32_B64(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_FLBIT_I32_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_FLBIT_I32(MachInst iFmt)
    {
        return new Inst_SOP1__S_FLBIT_I32(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_FLBIT_I32

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_FLBIT_I32_I64(MachInst iFmt)
    {
        return new Inst_SOP1__S_FLBIT_I32_I64(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_FLBIT_I32_I64

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_SEXT_I32_I8(MachInst iFmt)
    {
        return new Inst_SOP1__S_SEXT_I32_I8(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_SEXT_I32_I8

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_SEXT_I32_I16(MachInst iFmt)
    {
        return new Inst_SOP1__S_SEXT_I32_I16(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_SEXT_I32_I16

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_BITSET0_B32(MachInst iFmt)
    {
        return new Inst_SOP1__S_BITSET0_B32(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_BITSET0_B32

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_BITSET0_B64(MachInst iFmt)
    {
        return new Inst_SOP1__S_BITSET0_B64(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_BITSET0_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_BITSET1_B32(MachInst iFmt)
    {
        return new Inst_SOP1__S_BITSET1_B32(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_BITSET1_B32

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_BITSET1_B64(MachInst iFmt)
    {
        return new Inst_SOP1__S_BITSET1_B64(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_BITSET1_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_GETPC_B64(MachInst iFmt)
    {
        return new Inst_SOP1__S_GETPC_B64(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_GETPC_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_SETPC_B64(MachInst iFmt)
    {
        return new Inst_SOP1__S_SETPC_B64(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_SETPC_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_SWAPPC_B64(MachInst iFmt)
    {
        return new Inst_SOP1__S_SWAPPC_B64(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_SWAPPC_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_RFE_B64(MachInst iFmt)
    {
        return new Inst_SOP1__S_RFE_B64(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_RFE_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_AND_SAVEEXEC_B64(MachInst iFmt)
    {
        return new Inst_SOP1__S_AND_SAVEEXEC_B64(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_AND_SAVEEXEC_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_OR_SAVEEXEC_B64(MachInst iFmt)
    {
        return new Inst_SOP1__S_OR_SAVEEXEC_B64(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_OR_SAVEEXEC_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_XOR_SAVEEXEC_B64(MachInst iFmt)
    {
        return new Inst_SOP1__S_XOR_SAVEEXEC_B64(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_XOR_SAVEEXEC_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_ANDN2_SAVEEXEC_B64(MachInst iFmt)
    {
        return new Inst_SOP1__S_ANDN2_SAVEEXEC_B64(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_ANDN2_SAVEEXEC_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_ORN2_SAVEEXEC_B64(MachInst iFmt)
    {
        return new Inst_SOP1__S_ORN2_SAVEEXEC_B64(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_ORN2_SAVEEXEC_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_NAND_SAVEEXEC_B64(MachInst iFmt)
    {
        return new Inst_SOP1__S_NAND_SAVEEXEC_B64(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_NAND_SAVEEXEC_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_NOR_SAVEEXEC_B64(MachInst iFmt)
    {
        return new Inst_SOP1__S_NOR_SAVEEXEC_B64(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_NOR_SAVEEXEC_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_XNOR_SAVEEXEC_B64(MachInst iFmt)
    {
        return new Inst_SOP1__S_XNOR_SAVEEXEC_B64(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_XNOR_SAVEEXEC_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_QUADMASK_B32(MachInst iFmt)
    {
        return new Inst_SOP1__S_QUADMASK_B32(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_QUADMASK_B32

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_QUADMASK_B64(MachInst iFmt)
    {
        return new Inst_SOP1__S_QUADMASK_B64(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_QUADMASK_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_MOVRELS_B32(MachInst iFmt)
    {
        return new Inst_SOP1__S_MOVRELS_B32(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_MOVRELS_B32

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_MOVRELS_B64(MachInst iFmt)
    {
        return new Inst_SOP1__S_MOVRELS_B64(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_MOVRELS_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_MOVRELD_B32(MachInst iFmt)
    {
        return new Inst_SOP1__S_MOVRELD_B32(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_MOVRELD_B32

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_MOVRELD_B64(MachInst iFmt)
    {
        return new Inst_SOP1__S_MOVRELD_B64(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_MOVRELD_B64

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_CBRANCH_JOIN(MachInst iFmt)
    {
        return new Inst_SOP1__S_CBRANCH_JOIN(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_CBRANCH_JOIN

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_ABS_I32(MachInst iFmt)
    {
        return new Inst_SOP1__S_ABS_I32(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_ABS_I32

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_SET_GPR_IDX_IDX(MachInst iFmt)
    {
        return new Inst_SOP1__S_SET_GPR_IDX_IDX(&iFmt->iFmt_SOP1);
    } // decode_OP_SOP1__S_SET_GPR_IDX_IDX

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_ANDN1_SAVEEXEC_B64(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_ORN1_SAVEEXEC_B64(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_ANDN1_WREXEC_B64(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_ANDN2_WREXEC_B64(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SOP1__S_BITREPLICATE_B64_B32(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_SOPC__S_CMP_EQ_I32(MachInst iFmt)
    {
        return new Inst_SOPC__S_CMP_EQ_I32(&iFmt->iFmt_SOPC);
    } // decode_OP_SOPC__S_CMP_EQ_I32

    GPUStaticInst*
    Decoder::decode_OP_SOPC__S_CMP_LG_I32(MachInst iFmt)
    {
        return new Inst_SOPC__S_CMP_LG_I32(&iFmt->iFmt_SOPC);
    } // decode_OP_SOPC__S_CMP_LG_I32

    GPUStaticInst*
    Decoder::decode_OP_SOPC__S_CMP_GT_I32(MachInst iFmt)
    {
        return new Inst_SOPC__S_CMP_GT_I32(&iFmt->iFmt_SOPC);
    } // decode_OP_SOPC__S_CMP_GT_I32

    GPUStaticInst*
    Decoder::decode_OP_SOPC__S_CMP_GE_I32(MachInst iFmt)
    {
        return new Inst_SOPC__S_CMP_GE_I32(&iFmt->iFmt_SOPC);
    } // decode_OP_SOPC__S_CMP_GE_I32

    GPUStaticInst*
    Decoder::decode_OP_SOPC__S_CMP_LT_I32(MachInst iFmt)
    {
        return new Inst_SOPC__S_CMP_LT_I32(&iFmt->iFmt_SOPC);
    } // decode_OP_SOPC__S_CMP_LT_I32

    GPUStaticInst*
    Decoder::decode_OP_SOPC__S_CMP_LE_I32(MachInst iFmt)
    {
        return new Inst_SOPC__S_CMP_LE_I32(&iFmt->iFmt_SOPC);
    } // decode_OP_SOPC__S_CMP_LE_I32

    GPUStaticInst*
    Decoder::decode_OP_SOPC__S_CMP_EQ_U32(MachInst iFmt)
    {
        return new Inst_SOPC__S_CMP_EQ_U32(&iFmt->iFmt_SOPC);
    } // decode_OP_SOPC__S_CMP_EQ_U32

    GPUStaticInst*
    Decoder::decode_OP_SOPC__S_CMP_LG_U32(MachInst iFmt)
    {
        return new Inst_SOPC__S_CMP_LG_U32(&iFmt->iFmt_SOPC);
    } // decode_OP_SOPC__S_CMP_LG_U32

    GPUStaticInst*
    Decoder::decode_OP_SOPC__S_CMP_GT_U32(MachInst iFmt)
    {
        return new Inst_SOPC__S_CMP_GT_U32(&iFmt->iFmt_SOPC);
    } // decode_OP_SOPC__S_CMP_GT_U32

    GPUStaticInst*
    Decoder::decode_OP_SOPC__S_CMP_GE_U32(MachInst iFmt)
    {
        return new Inst_SOPC__S_CMP_GE_U32(&iFmt->iFmt_SOPC);
    } // decode_OP_SOPC__S_CMP_GE_U32

    GPUStaticInst*
    Decoder::decode_OP_SOPC__S_CMP_LT_U32(MachInst iFmt)
    {
        return new Inst_SOPC__S_CMP_LT_U32(&iFmt->iFmt_SOPC);
    } // decode_OP_SOPC__S_CMP_LT_U32

    GPUStaticInst*
    Decoder::decode_OP_SOPC__S_CMP_LE_U32(MachInst iFmt)
    {
        return new Inst_SOPC__S_CMP_LE_U32(&iFmt->iFmt_SOPC);
    } // decode_OP_SOPC__S_CMP_LE_U32

    GPUStaticInst*
    Decoder::decode_OP_SOPC__S_BITCMP0_B32(MachInst iFmt)
    {
        return new Inst_SOPC__S_BITCMP0_B32(&iFmt->iFmt_SOPC);
    } // decode_OP_SOPC__S_BITCMP0_B32

    GPUStaticInst*
    Decoder::decode_OP_SOPC__S_BITCMP1_B32(MachInst iFmt)
    {
        return new Inst_SOPC__S_BITCMP1_B32(&iFmt->iFmt_SOPC);
    } // decode_OP_SOPC__S_BITCMP1_B32

    GPUStaticInst*
    Decoder::decode_OP_SOPC__S_BITCMP0_B64(MachInst iFmt)
    {
        return new Inst_SOPC__S_BITCMP0_B64(&iFmt->iFmt_SOPC);
    } // decode_OP_SOPC__S_BITCMP0_B64

    GPUStaticInst*
    Decoder::decode_OP_SOPC__S_BITCMP1_B64(MachInst iFmt)
    {
        return new Inst_SOPC__S_BITCMP1_B64(&iFmt->iFmt_SOPC);
    } // decode_OP_SOPC__S_BITCMP1_B64

    GPUStaticInst*
    Decoder::decode_OP_SOPC__S_SETVSKIP(MachInst iFmt)
    {
        return new Inst_SOPC__S_SETVSKIP(&iFmt->iFmt_SOPC);
    } // decode_OP_SOPC__S_SETVSKIP

    GPUStaticInst*
    Decoder::decode_OP_SOPC__S_SET_GPR_IDX_ON(MachInst iFmt)
    {
        return new Inst_SOPC__S_SET_GPR_IDX_ON(&iFmt->iFmt_SOPC);
    } // decode_OP_SOPC__S_SET_GPR_IDX_ON

    GPUStaticInst*
    Decoder::decode_OP_SOPC__S_CMP_EQ_U64(MachInst iFmt)
    {
        return new Inst_SOPC__S_CMP_EQ_U64(&iFmt->iFmt_SOPC);
    } // decode_OP_SOPC__S_CMP_EQ_U64

    GPUStaticInst*
    Decoder::decode_OP_SOPC__S_CMP_LG_U64(MachInst iFmt)
    {
        return new Inst_SOPC__S_CMP_LG_U64(&iFmt->iFmt_SOPC);
    } // decode_OP_SOPC__S_CMP_LG_U64

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_NOP(MachInst iFmt)
    {
        return new Inst_SOPP__S_NOP(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_NOP

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_ENDPGM(MachInst iFmt)
    {
        return new Inst_SOPP__S_ENDPGM(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_ENDPGM

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_BRANCH(MachInst iFmt)
    {
        return new Inst_SOPP__S_BRANCH(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_BRANCH

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_WAKEUP(MachInst iFmt)
    {
        return new Inst_SOPP__S_WAKEUP(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_WAKEUP

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_CBRANCH_SCC0(MachInst iFmt)
    {
        return new Inst_SOPP__S_CBRANCH_SCC0(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_CBRANCH_SCC0

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_CBRANCH_SCC1(MachInst iFmt)
    {
        return new Inst_SOPP__S_CBRANCH_SCC1(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_CBRANCH_SCC1

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_CBRANCH_VCCZ(MachInst iFmt)
    {
        return new Inst_SOPP__S_CBRANCH_VCCZ(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_CBRANCH_VCCZ

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_CBRANCH_VCCNZ(MachInst iFmt)
    {
        return new Inst_SOPP__S_CBRANCH_VCCNZ(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_CBRANCH_VCCNZ

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_CBRANCH_EXECZ(MachInst iFmt)
    {
        return new Inst_SOPP__S_CBRANCH_EXECZ(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_CBRANCH_EXECZ

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_CBRANCH_EXECNZ(MachInst iFmt)
    {
        return new Inst_SOPP__S_CBRANCH_EXECNZ(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_CBRANCH_EXECNZ

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_BARRIER(MachInst iFmt)
    {
        return new Inst_SOPP__S_BARRIER(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_BARRIER

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_SETKILL(MachInst iFmt)
    {
        return new Inst_SOPP__S_SETKILL(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_SETKILL

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_WAITCNT(MachInst iFmt)
    {
        return new Inst_SOPP__S_WAITCNT(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_WAITCNT

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_SETHALT(MachInst iFmt)
    {
        return new Inst_SOPP__S_SETHALT(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_SETHALT

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_SLEEP(MachInst iFmt)
    {
        return new Inst_SOPP__S_SLEEP(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_SLEEP

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_SETPRIO(MachInst iFmt)
    {
        return new Inst_SOPP__S_SETPRIO(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_SETPRIO

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_SENDMSG(MachInst iFmt)
    {
        return new Inst_SOPP__S_SENDMSG(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_SENDMSG

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_SENDMSGHALT(MachInst iFmt)
    {
        return new Inst_SOPP__S_SENDMSGHALT(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_SENDMSGHALT

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_TRAP(MachInst iFmt)
    {
        return new Inst_SOPP__S_TRAP(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_TRAP

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_ICACHE_INV(MachInst iFmt)
    {
        return new Inst_SOPP__S_ICACHE_INV(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_ICACHE_INV

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_INCPERFLEVEL(MachInst iFmt)
    {
        return new Inst_SOPP__S_INCPERFLEVEL(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_INCPERFLEVEL

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_DECPERFLEVEL(MachInst iFmt)
    {
        return new Inst_SOPP__S_DECPERFLEVEL(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_DECPERFLEVEL

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_TTRACEDATA(MachInst iFmt)
    {
        return new Inst_SOPP__S_TTRACEDATA(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_TTRACEDATA

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_CBRANCH_CDBGSYS(MachInst iFmt)
    {
        return new Inst_SOPP__S_CBRANCH_CDBGSYS(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_CBRANCH_CDBGSYS

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_CBRANCH_CDBGUSER(MachInst iFmt)
    {
        return new Inst_SOPP__S_CBRANCH_CDBGUSER(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_CBRANCH_CDBGUSER

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_CBRANCH_CDBGSYS_OR_USER(MachInst iFmt)
    {
        return new Inst_SOPP__S_CBRANCH_CDBGSYS_OR_USER(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_CBRANCH_CDBGSYS_OR_USER

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_CBRANCH_CDBGSYS_AND_USER(MachInst iFmt)
    {
        return new Inst_SOPP__S_CBRANCH_CDBGSYS_AND_USER(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_CBRANCH_CDBGSYS_AND_USER

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_ENDPGM_SAVED(MachInst iFmt)
    {
        return new Inst_SOPP__S_ENDPGM_SAVED(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_ENDPGM_SAVED

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_SET_GPR_IDX_OFF(MachInst iFmt)
    {
        return new Inst_SOPP__S_SET_GPR_IDX_OFF(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_SET_GPR_IDX_OFF

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_SET_GPR_IDX_MODE(MachInst iFmt)
    {
        return new Inst_SOPP__S_SET_GPR_IDX_MODE(&iFmt->iFmt_SOPP);
    } // decode_OP_SOPP__S_SET_GPR_IDX_MODE

    GPUStaticInst*
    Decoder::decode_OP_SOPP__S_ENDPGM_ORDERED_PS_DONE(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VINTRP__V_INTERP_P1_F32(MachInst iFmt)
    {
        return new Inst_VINTRP__V_INTERP_P1_F32(&iFmt->iFmt_VINTRP);
    } // decode_OP_VINTRP__V_INTERP_P1_F32

    GPUStaticInst*
    Decoder::decode_OP_VINTRP__V_INTERP_P2_F32(MachInst iFmt)
    {
        return new Inst_VINTRP__V_INTERP_P2_F32(&iFmt->iFmt_VINTRP);
    } // decode_OP_VINTRP__V_INTERP_P2_F32

    GPUStaticInst*
    Decoder::decode_OP_VINTRP__V_INTERP_MOV_F32(MachInst iFmt)
    {
        return new Inst_VINTRP__V_INTERP_MOV_F32(&iFmt->iFmt_VINTRP);
    } // decode_OP_VINTRP__V_INTERP_MOV_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_NOP(MachInst iFmt)
    {
        return new Inst_VOP1__V_NOP(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_NOP

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_MOV_B32(MachInst iFmt)
    {
        return new Inst_VOP1__V_MOV_B32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_MOV_B32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_READFIRSTLANE_B32(MachInst iFmt)
    {
        return new Inst_VOP1__V_READFIRSTLANE_B32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_READFIRSTLANE_B32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_CVT_I32_F64(MachInst iFmt)
    {
        return new Inst_VOP1__V_CVT_I32_F64(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_CVT_I32_F64

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_CVT_F64_I32(MachInst iFmt)
    {
        return new Inst_VOP1__V_CVT_F64_I32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_CVT_F64_I32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_CVT_F32_I32(MachInst iFmt)
    {
        return new Inst_VOP1__V_CVT_F32_I32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_CVT_F32_I32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_CVT_F32_U32(MachInst iFmt)
    {
        return new Inst_VOP1__V_CVT_F32_U32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_CVT_F32_U32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_CVT_U32_F32(MachInst iFmt)
    {
        return new Inst_VOP1__V_CVT_U32_F32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_CVT_U32_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_CVT_I32_F32(MachInst iFmt)
    {
        return new Inst_VOP1__V_CVT_I32_F32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_CVT_I32_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_CVT_F16_F32(MachInst iFmt)
    {
        return new Inst_VOP1__V_CVT_F16_F32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_CVT_F16_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_CVT_F32_F16(MachInst iFmt)
    {
        return new Inst_VOP1__V_CVT_F32_F16(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_CVT_F32_F16

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_CVT_RPI_I32_F32(MachInst iFmt)
    {
        return new Inst_VOP1__V_CVT_RPI_I32_F32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_CVT_RPI_I32_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_CVT_FLR_I32_F32(MachInst iFmt)
    {
        return new Inst_VOP1__V_CVT_FLR_I32_F32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_CVT_FLR_I32_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_CVT_OFF_F32_I4(MachInst iFmt)
    {
        return new Inst_VOP1__V_CVT_OFF_F32_I4(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_CVT_OFF_F32_I4

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_CVT_F32_F64(MachInst iFmt)
    {
        return new Inst_VOP1__V_CVT_F32_F64(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_CVT_F32_F64

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_CVT_F64_F32(MachInst iFmt)
    {
        return new Inst_VOP1__V_CVT_F64_F32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_CVT_F64_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_CVT_F32_UBYTE0(MachInst iFmt)
    {
        return new Inst_VOP1__V_CVT_F32_UBYTE0(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_CVT_F32_UBYTE0

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_CVT_F32_UBYTE1(MachInst iFmt)
    {
        return new Inst_VOP1__V_CVT_F32_UBYTE1(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_CVT_F32_UBYTE1

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_CVT_F32_UBYTE2(MachInst iFmt)
    {
        return new Inst_VOP1__V_CVT_F32_UBYTE2(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_CVT_F32_UBYTE2

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_CVT_F32_UBYTE3(MachInst iFmt)
    {
        return new Inst_VOP1__V_CVT_F32_UBYTE3(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_CVT_F32_UBYTE3

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_CVT_U32_F64(MachInst iFmt)
    {
        return new Inst_VOP1__V_CVT_U32_F64(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_CVT_U32_F64

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_CVT_F64_U32(MachInst iFmt)
    {
        return new Inst_VOP1__V_CVT_F64_U32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_CVT_F64_U32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_TRUNC_F64(MachInst iFmt)
    {
        return new Inst_VOP1__V_TRUNC_F64(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_TRUNC_F64

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_CEIL_F64(MachInst iFmt)
    {
        return new Inst_VOP1__V_CEIL_F64(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_CEIL_F64

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_RNDNE_F64(MachInst iFmt)
    {
        return new Inst_VOP1__V_RNDNE_F64(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_RNDNE_F64

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_FLOOR_F64(MachInst iFmt)
    {
        return new Inst_VOP1__V_FLOOR_F64(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_FLOOR_F64

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_FRACT_F32(MachInst iFmt)
    {
        return new Inst_VOP1__V_FRACT_F32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_FRACT_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_TRUNC_F32(MachInst iFmt)
    {
        return new Inst_VOP1__V_TRUNC_F32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_TRUNC_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_CEIL_F32(MachInst iFmt)
    {
        return new Inst_VOP1__V_CEIL_F32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_CEIL_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_RNDNE_F32(MachInst iFmt)
    {
        return new Inst_VOP1__V_RNDNE_F32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_RNDNE_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_FLOOR_F32(MachInst iFmt)
    {
        return new Inst_VOP1__V_FLOOR_F32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_FLOOR_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_EXP_F32(MachInst iFmt)
    {
        return new Inst_VOP1__V_EXP_F32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_EXP_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_LOG_F32(MachInst iFmt)
    {
        return new Inst_VOP1__V_LOG_F32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_LOG_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_RCP_F32(MachInst iFmt)
    {
        return new Inst_VOP1__V_RCP_F32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_RCP_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_RCP_IFLAG_F32(MachInst iFmt)
    {
        return new Inst_VOP1__V_RCP_IFLAG_F32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_RCP_IFLAG_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_RSQ_F32(MachInst iFmt)
    {
        return new Inst_VOP1__V_RSQ_F32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_RSQ_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_RCP_F64(MachInst iFmt)
    {
        return new Inst_VOP1__V_RCP_F64(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_RCP_F64

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_RSQ_F64(MachInst iFmt)
    {
        return new Inst_VOP1__V_RSQ_F64(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_RSQ_F64

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_SQRT_F32(MachInst iFmt)
    {
        return new Inst_VOP1__V_SQRT_F32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_SQRT_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_SQRT_F64(MachInst iFmt)
    {
        return new Inst_VOP1__V_SQRT_F64(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_SQRT_F64

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_SIN_F32(MachInst iFmt)
    {
        return new Inst_VOP1__V_SIN_F32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_SIN_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_COS_F32(MachInst iFmt)
    {
        return new Inst_VOP1__V_COS_F32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_COS_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_NOT_B32(MachInst iFmt)
    {
        return new Inst_VOP1__V_NOT_B32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_NOT_B32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_BFREV_B32(MachInst iFmt)
    {
        return new Inst_VOP1__V_BFREV_B32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_BFREV_B32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_FFBH_U32(MachInst iFmt)
    {
        return new Inst_VOP1__V_FFBH_U32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_FFBH_U32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_FFBL_B32(MachInst iFmt)
    {
        return new Inst_VOP1__V_FFBL_B32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_FFBL_B32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_FFBH_I32(MachInst iFmt)
    {
        return new Inst_VOP1__V_FFBH_I32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_FFBH_I32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_FREXP_EXP_I32_F64(MachInst iFmt)
    {
        return new Inst_VOP1__V_FREXP_EXP_I32_F64(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_FREXP_EXP_I32_F64

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_FREXP_MANT_F64(MachInst iFmt)
    {
        return new Inst_VOP1__V_FREXP_MANT_F64(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_FREXP_MANT_F64

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_FRACT_F64(MachInst iFmt)
    {
        return new Inst_VOP1__V_FRACT_F64(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_FRACT_F64

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_FREXP_EXP_I32_F32(MachInst iFmt)
    {
        return new Inst_VOP1__V_FREXP_EXP_I32_F32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_FREXP_EXP_I32_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_FREXP_MANT_F32(MachInst iFmt)
    {
        return new Inst_VOP1__V_FREXP_MANT_F32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_FREXP_MANT_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_CLREXCP(MachInst iFmt)
    {
        return new Inst_VOP1__V_CLREXCP(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_CLREXCP

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_SCREEN_PARTITION_4SE_B32(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_MOV_B64(MachInst iFmt)
    {
        return new Inst_VOP1__V_MOV_B64(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_MOV_B64

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_CVT_F16_U16(MachInst iFmt)
    {
        return new Inst_VOP1__V_CVT_F16_U16(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_CVT_F16_U16

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_CVT_F16_I16(MachInst iFmt)
    {
        return new Inst_VOP1__V_CVT_F16_I16(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_CVT_F16_I16

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_CVT_U16_F16(MachInst iFmt)
    {
        return new Inst_VOP1__V_CVT_U16_F16(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_CVT_U16_F16

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_CVT_I16_F16(MachInst iFmt)
    {
        return new Inst_VOP1__V_CVT_I16_F16(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_CVT_I16_F16

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_RCP_F16(MachInst iFmt)
    {
        return new Inst_VOP1__V_RCP_F16(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_RCP_F16

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_SQRT_F16(MachInst iFmt)
    {
        return new Inst_VOP1__V_SQRT_F16(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_SQRT_F16

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_RSQ_F16(MachInst iFmt)
    {
        return new Inst_VOP1__V_RSQ_F16(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_RSQ_F16

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_LOG_F16(MachInst iFmt)
    {
        return new Inst_VOP1__V_LOG_F16(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_LOG_F16

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_EXP_F16(MachInst iFmt)
    {
        return new Inst_VOP1__V_EXP_F16(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_EXP_F16

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_FREXP_MANT_F16(MachInst iFmt)
    {
        return new Inst_VOP1__V_FREXP_MANT_F16(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_FREXP_MANT_F16

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_FREXP_EXP_I16_F16(MachInst iFmt)
    {
        return new Inst_VOP1__V_FREXP_EXP_I16_F16(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_FREXP_EXP_I16_F16

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_FLOOR_F16(MachInst iFmt)
    {
        return new Inst_VOP1__V_FLOOR_F16(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_FLOOR_F16

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_CEIL_F16(MachInst iFmt)
    {
        return new Inst_VOP1__V_CEIL_F16(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_CEIL_F16

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_TRUNC_F16(MachInst iFmt)
    {
        return new Inst_VOP1__V_TRUNC_F16(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_TRUNC_F16

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_RNDNE_F16(MachInst iFmt)
    {
        return new Inst_VOP1__V_RNDNE_F16(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_RNDNE_F16

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_FRACT_F16(MachInst iFmt)
    {
        return new Inst_VOP1__V_FRACT_F16(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_FRACT_F16

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_SIN_F16(MachInst iFmt)
    {
        return new Inst_VOP1__V_SIN_F16(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_SIN_F16

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_COS_F16(MachInst iFmt)
    {
        return new Inst_VOP1__V_COS_F16(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_COS_F16

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_EXP_LEGACY_F32(MachInst iFmt)
    {
        return new Inst_VOP1__V_EXP_LEGACY_F32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_EXP_LEGACY_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_LOG_LEGACY_F32(MachInst iFmt)
    {
        return new Inst_VOP1__V_LOG_LEGACY_F32(&iFmt->iFmt_VOP1);
    } // decode_OP_VOP1__V_LOG_LEGACY_F32

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_CVT_NORM_I16_F16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_CVT_NORM_U16_F16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_SAT_PK_U8_I16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_SWAP_B32(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP1__V_ACCVGPR_MOV_B32(MachInst iFmt)
    {
        return new Inst_VOP1__V_ACCVGPR_MOV_B32(&iFmt->iFmt_VOP1);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_CLASS_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_CLASS_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_CLASS_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_CLASS_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_CLASS_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_CLASS_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_CLASS_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_CLASS_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_CLASS_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_CLASS_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_CLASS_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_CLASS_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_CLASS_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_CLASS_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_CLASS_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_CLASS_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_CLASS_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_CLASS_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_F_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_F_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_F_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_LT_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_LT_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_LT_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_EQ_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_EQ_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_EQ_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_LE_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_LE_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_LE_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_GT_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_GT_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_GT_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_LG_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_LG_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_LG_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_GE_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_GE_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_GE_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_O_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_O_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_O_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_U_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_U_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_U_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_NGE_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_NGE_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_NGE_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_NLG_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_NLG_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_NLG_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_NGT_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_NGT_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_NGT_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_NLE_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_NLE_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_NLE_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_NEQ_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_NEQ_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_NEQ_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_NLT_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_NLT_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_NLT_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_TRU_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_TRU_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_TRU_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_F_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_F_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_F_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_LT_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_LT_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_LT_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_EQ_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_EQ_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_EQ_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_LE_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_LE_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_LE_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_GT_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_GT_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_GT_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_LG_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_LG_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_LG_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_GE_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_GE_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_GE_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_O_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_O_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_O_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_U_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_U_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_U_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_NGE_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_NGE_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_NGE_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_NLG_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_NLG_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_NLG_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_NGT_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_NGT_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_NGT_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_NLE_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_NLE_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_NLE_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_NEQ_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_NEQ_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_NEQ_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_NLT_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_NLT_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_NLT_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_TRU_F16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_TRU_F16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_TRU_F16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_F_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_F_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_F_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_LT_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_LT_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_LT_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_EQ_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_EQ_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_EQ_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_LE_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_LE_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_LE_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_GT_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_GT_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_GT_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_LG_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_LG_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_LG_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_GE_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_GE_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_GE_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_O_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_O_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_O_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_U_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_U_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_U_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_NGE_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_NGE_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_NGE_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_NLG_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_NLG_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_NLG_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_NGT_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_NGT_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_NGT_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_NLE_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_NLE_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_NLE_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_NEQ_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_NEQ_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_NEQ_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_NLT_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_NLT_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_NLT_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_TRU_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_TRU_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_TRU_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_F_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_F_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_F_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_LT_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_LT_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_LT_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_EQ_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_EQ_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_EQ_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_LE_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_LE_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_LE_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_GT_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_GT_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_GT_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_LG_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_LG_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_LG_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_GE_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_GE_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_GE_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_O_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_O_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_O_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_U_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_U_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_U_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_NGE_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_NGE_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_NGE_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_NLG_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_NLG_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_NLG_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_NGT_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_NGT_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_NGT_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_NLE_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_NLE_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_NLE_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_NEQ_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_NEQ_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_NEQ_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_NLT_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_NLT_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_NLT_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_TRU_F32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_TRU_F32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_TRU_F32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_F_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_F_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_F_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_LT_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_LT_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_LT_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_EQ_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_EQ_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_EQ_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_LE_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_LE_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_LE_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_GT_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_GT_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_GT_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_LG_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_LG_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_LG_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_GE_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_GE_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_GE_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_O_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_O_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_O_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_U_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_U_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_U_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_NGE_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_NGE_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_NGE_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_NLG_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_NLG_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_NLG_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_NGT_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_NGT_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_NGT_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_NLE_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_NLE_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_NLE_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_NEQ_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_NEQ_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_NEQ_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_NLT_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_NLT_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_NLT_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_TRU_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_TRU_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_TRU_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_F_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_F_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_F_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_LT_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_LT_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_LT_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_EQ_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_EQ_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_EQ_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_LE_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_LE_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_LE_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_GT_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_GT_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_GT_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_LG_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_LG_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_LG_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_GE_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_GE_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_GE_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_O_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_O_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_O_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_U_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_U_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_U_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_NGE_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_NGE_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_NGE_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_NLG_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_NLG_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_NLG_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_NGT_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_NGT_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_NGT_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_NLE_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_NLE_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_NLE_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_NEQ_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_NEQ_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_NEQ_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_NLT_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_NLT_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_NLT_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_TRU_F64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_TRU_F64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_TRU_F64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_F_I16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_F_I16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_F_I16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_LT_I16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_LT_I16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_LT_I16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_EQ_I16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_EQ_I16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_EQ_I16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_LE_I16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_LE_I16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_LE_I16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_GT_I16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_GT_I16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_GT_I16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_NE_I16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_NE_I16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_NE_I16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_GE_I16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_GE_I16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_GE_I16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_T_I16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_T_I16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_T_I16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_F_U16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_F_U16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_F_U16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_LT_U16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_LT_U16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_LT_U16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_EQ_U16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_EQ_U16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_EQ_U16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_LE_U16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_LE_U16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_LE_U16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_GT_U16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_GT_U16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_GT_U16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_NE_U16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_NE_U16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_NE_U16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_GE_U16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_GE_U16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_GE_U16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_T_U16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_T_U16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_T_U16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_F_I16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_F_I16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_F_I16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_LT_I16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_LT_I16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_LT_I16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_EQ_I16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_EQ_I16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_EQ_I16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_LE_I16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_LE_I16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_LE_I16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_GT_I16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_GT_I16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_GT_I16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_NE_I16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_NE_I16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_NE_I16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_GE_I16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_GE_I16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_GE_I16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_T_I16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_T_I16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_T_I16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_F_U16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_F_U16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_F_U16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_LT_U16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_LT_U16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_LT_U16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_EQ_U16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_EQ_U16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_EQ_U16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_LE_U16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_LE_U16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_LE_U16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_GT_U16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_GT_U16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_GT_U16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_NE_U16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_NE_U16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_NE_U16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_GE_U16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_GE_U16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_GE_U16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_T_U16(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_T_U16(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_T_U16

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_F_I32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_F_I32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_F_I32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_LT_I32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_LT_I32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_LT_I32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_EQ_I32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_EQ_I32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_EQ_I32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_LE_I32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_LE_I32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_LE_I32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_GT_I32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_GT_I32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_GT_I32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_NE_I32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_NE_I32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_NE_I32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_GE_I32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_GE_I32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_GE_I32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_T_I32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_T_I32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_T_I32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_F_U32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_F_U32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_F_U32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_LT_U32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_LT_U32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_LT_U32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_EQ_U32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_EQ_U32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_EQ_U32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_LE_U32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_LE_U32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_LE_U32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_GT_U32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_GT_U32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_GT_U32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_NE_U32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_NE_U32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_NE_U32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_GE_U32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_GE_U32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_GE_U32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_T_U32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_T_U32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_T_U32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_F_I32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_F_I32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_F_I32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_LT_I32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_LT_I32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_LT_I32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_EQ_I32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_EQ_I32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_EQ_I32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_LE_I32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_LE_I32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_LE_I32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_GT_I32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_GT_I32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_GT_I32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_NE_I32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_NE_I32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_NE_I32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_GE_I32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_GE_I32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_GE_I32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_T_I32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_T_I32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_T_I32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_F_U32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_F_U32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_F_U32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_LT_U32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_LT_U32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_LT_U32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_EQ_U32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_EQ_U32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_EQ_U32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_LE_U32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_LE_U32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_LE_U32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_GT_U32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_GT_U32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_GT_U32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_NE_U32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_NE_U32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_NE_U32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_GE_U32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_GE_U32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_GE_U32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_T_U32(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_T_U32(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_T_U32

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_F_I64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_F_I64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_F_I64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_LT_I64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_LT_I64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_LT_I64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_EQ_I64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_EQ_I64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_EQ_I64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_LE_I64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_LE_I64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_LE_I64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_GT_I64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_GT_I64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_GT_I64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_NE_I64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_NE_I64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_NE_I64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_GE_I64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_GE_I64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_GE_I64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_T_I64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_T_I64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_T_I64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_F_U64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_F_U64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_F_U64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_LT_U64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_LT_U64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_LT_U64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_EQ_U64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_EQ_U64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_EQ_U64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_LE_U64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_LE_U64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_LE_U64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_GT_U64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_GT_U64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_GT_U64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_NE_U64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_NE_U64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_NE_U64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_GE_U64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_GE_U64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_GE_U64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMP_T_U64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMP_T_U64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMP_T_U64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_F_I64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_F_I64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_F_I64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_LT_I64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_LT_I64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_LT_I64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_EQ_I64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_EQ_I64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_EQ_I64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_LE_I64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_LE_I64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_LE_I64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_GT_I64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_GT_I64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_GT_I64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_NE_I64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_NE_I64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_NE_I64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_GE_I64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_GE_I64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_GE_I64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_T_I64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_T_I64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_T_I64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_F_U64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_F_U64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_F_U64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_LT_U64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_LT_U64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_LT_U64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_EQ_U64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_EQ_U64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_EQ_U64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_LE_U64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_LE_U64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_LE_U64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_GT_U64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_GT_U64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_GT_U64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_NE_U64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_NE_U64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_NE_U64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_GE_U64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_GE_U64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_GE_U64

    GPUStaticInst*
    Decoder::decode_OP_VOPC__V_CMPX_T_U64(MachInst iFmt)
    {
        return new Inst_VOPC__V_CMPX_T_U64(&iFmt->iFmt_VOPC);
    } // decode_OP_VOPC__V_CMPX_T_U64

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_PK_MAD_I16(MachInst iFmt)
    {
        return new Inst_VOP3P__V_PK_MAD_I16(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_PK_MUL_LO_U16(MachInst iFmt)
    {
        return new Inst_VOP3P__V_PK_MUL_LO_U16(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_PK_ADD_I16(MachInst iFmt)
    {
        return new Inst_VOP3P__V_PK_ADD_I16(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_PK_SUB_I16(MachInst iFmt)
    {
        return new Inst_VOP3P__V_PK_SUB_I16(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_PK_LSHLREV_B16(MachInst iFmt)
    {
        return new Inst_VOP3P__V_PK_LSHLREV_B16(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_PK_LSHRREV_B16(MachInst iFmt)
    {
        return new Inst_VOP3P__V_PK_LSHRREV_B16(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_PK_ASHRREV_I16(MachInst iFmt)
    {
        return new Inst_VOP3P__V_PK_ASHRREV_B16(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_PK_MAX_I16(MachInst iFmt)
    {
        return new Inst_VOP3P__V_PK_MAX_I16(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_PK_MIN_I16(MachInst iFmt)
    {
        return new Inst_VOP3P__V_PK_MIN_I16(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_PK_MAD_U16(MachInst iFmt)
    {
        return new Inst_VOP3P__V_PK_MAD_U16(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_PK_ADD_U16(MachInst iFmt)
    {
        return new Inst_VOP3P__V_PK_ADD_U16(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_PK_SUB_U16(MachInst iFmt)
    {
        return new Inst_VOP3P__V_PK_SUB_U16(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_PK_MAX_U16(MachInst iFmt)
    {
        return new Inst_VOP3P__V_PK_MAX_U16(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_PK_MIN_U16(MachInst iFmt)
    {
        return new Inst_VOP3P__V_PK_MIN_U16(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_PK_FMA_F16(MachInst iFmt)
    {
        return new Inst_VOP3P__V_PK_FMA_F16(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_PK_ADD_F16(MachInst iFmt)
    {
        return new Inst_VOP3P__V_PK_ADD_F16(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_PK_MUL_F16(MachInst iFmt)
    {
        return new Inst_VOP3P__V_PK_MUL_F16(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_PK_MIN_F16(MachInst iFmt)
    {
        return new Inst_VOP3P__V_PK_MIN_F16(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_PK_MAX_F16(MachInst iFmt)
    {
        return new Inst_VOP3P__V_PK_MAX_F16(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MAD_MIX_F32(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MAD_MIXLO_F16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MAD_MIXHI_F16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_PK_FMA_F32(MachInst iFmt)
    {
        return new Inst_VOP3P__V_PK_FMA_F32(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_PK_MUL_F32(MachInst iFmt)
    {
        return new Inst_VOP3P__V_PK_MUL_F32(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_PK_ADD_F32(MachInst iFmt)
    {
        return new Inst_VOP3P__V_PK_ADD_F32(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_PK_MOV_B32(MachInst iFmt)
    {
        return new Inst_VOP3P__V_PK_MOV_B32(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_DOT2_F32_F16(MachInst iFmt)
    {
        return new Inst_VOP3P__V_DOT2_F32_F16(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_DOT2_I32_I16(MachInst iFmt)
    {
        return new Inst_VOP3P__V_DOT2_I32_I16(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_DOT2_U32_U16(MachInst iFmt)
    {
        return new Inst_VOP3P__V_DOT2_U32_U16(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_DOT4_I32_I8(MachInst iFmt)
    {
        return new Inst_VOP3P__V_DOT4_I32_I8(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_DOT4_U32_U8(MachInst iFmt)
    {
        return new Inst_VOP3P__V_DOT4_U32_U8(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_DOT8_I32_I4(MachInst iFmt)
    {
        return new Inst_VOP3P__V_DOT8_I32_I4(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_DOT8_U32_U4(MachInst iFmt)
    {
        return new Inst_VOP3P__V_DOT8_U32_U4(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_F32_32X32X1_2B_F32(MachInst iFmt)
    {
        return new Inst_VOP3P_MAI__V_MFMA_F32_32X32X1_2B_F32(
                &iFmt->iFmt_VOP3P_MAI);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_F32_16X16X1_4B_F32(MachInst iFmt)
    {
        return new Inst_VOP3P_MAI__V_MFMA_F32_16X16X1_4B_F32(
                &iFmt->iFmt_VOP3P_MAI);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_F32_4X4X1_16B_F32(MachInst iFmt)
    {
        return new Inst_VOP3P_MAI__V_MFMA_F32_4X4X1_16B_F32(
                &iFmt->iFmt_VOP3P_MAI);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_F32_32X32X2_F32(MachInst iFmt)
    {
        return new Inst_VOP3P_MAI__V_MFMA_F32_32X32X2_F32(
                &iFmt->iFmt_VOP3P_MAI);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_F32_16X16X4_F32(MachInst iFmt)
    {
        return new Inst_VOP3P_MAI__V_MFMA_F32_16X16X4_F32(
                &iFmt->iFmt_VOP3P_MAI);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_F32_32X32X4_2B_F16(MachInst iFmt)
    {
        return new Inst_VOP3P_MAI__V_MFMA_F32_32X32X4_2B_F16(
                &iFmt->iFmt_VOP3P_MAI);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_F32_16X16X4_4B_F16(MachInst iFmt)
    {
        return new Inst_VOP3P_MAI__V_MFMA_F32_16X16X4_4B_F16(
                &iFmt->iFmt_VOP3P_MAI);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_F32_4X4X4_16B_F16(MachInst iFmt)
    {
        return new Inst_VOP3P_MAI__V_MFMA_F32_4X4X4_16B_F16(
                &iFmt->iFmt_VOP3P_MAI);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_F32_32X32X8_F16(MachInst iFmt)
    {
        return new Inst_VOP3P_MAI__V_MFMA_F32_32X32X8_F16(
                &iFmt->iFmt_VOP3P_MAI);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_F32_16X16X16_F16(MachInst iFmt)
    {
        return new Inst_VOP3P_MAI__V_MFMA_F32_16X16X16_F16(
                &iFmt->iFmt_VOP3P_MAI);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_I32_32X32X4_2B_I8(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_I32_16X16X4_4B_I8(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_I32_4X4X4_16B_I8(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_I32_16X16X16_I8(MachInst iFmt)
    {
        return new Inst_VOP3P_MAI__V_MFMA_I32_16X16X16_I8(
                &iFmt->iFmt_VOP3P_MAI);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_I32_32X32X8_I8(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_I32_32X32X16_I8(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_I32_16X16X32_I8(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_F32_32X32X4_2B_BF16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_F32_16X16X4_4B_BF16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_F32_4X4X4_16B_BF16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_F32_32X32X8_BF16(MachInst iFmt)
    {
        return new Inst_VOP3P_MAI__V_MFMA_F32_32X32X8_BF16(
                &iFmt->iFmt_VOP3P_MAI);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_F32_16X16X16_BF16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_SMFMAC_F32_16X16X32_F16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_SMFMAC_F32_32X32X16_F16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_SMFMAC_F32_16X16X32_BF16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_SMFMAC_F32_32X32X16_BF16(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_SMFMAC_I32_16X16X64_I8(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_SMFMAC_I32_32X32X32_I8(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_F64_4X4X4_4B_F64(MachInst iFmt)
    {
        return new Inst_VOP3P_MAI__V_MFMA_F64_4X4X4_4B_F64(
                &iFmt->iFmt_VOP3P_MAI);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_F32_16X16X32_BF8_BF8(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_F32_16X16X32_BF8_FP8(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_F32_16X16X32_FP8_BF8(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_F32_16X16X32_FP8_FP8(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_F32_32X32X16_BF8_BF8(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_F32_32X32X16_BF8_FP8(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_F32_32X32X16_FP8_BF8(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_F32_32X32X16_FP8_FP8(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_SMFMAC_F32_16X16X64_BF8_BF8(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_SMFMAC_F32_16X16X64_BF8_FP8(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_SMFMAC_F32_16X16X64_FP8_BF8(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_SMFMAC_F32_16X16X64_FP8_FP8(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_SMFMAC_F32_32X32X32_BF8_BF8(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_SMFMAC_F32_32X32X32_BF8_FP8(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_SMFMAC_F32_32X32X32_FP8_BF8(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_SMFMAC_F32_32X32X32_FP8_FP8(MachInst iFmt)
    {
        fatal("Trying to decode instruction without a class\n");
        return nullptr;
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_MFMA_F64_16X16X4_F64(MachInst iFmt)
    {
        return new Inst_VOP3P_MAI__V_MFMA_F64_16X16X4_F64(
                &iFmt->iFmt_VOP3P_MAI);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_ACCVGPR_READ(MachInst iFmt)
    {
        return new Inst_VOP3P__V_ACCVGPR_READ(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_OP_VOP3P__V_ACCVGPR_WRITE(MachInst iFmt)
    {
        return new Inst_VOP3P__V_ACCVGPR_WRITE(&iFmt->iFmt_VOP3P);
    }

    GPUStaticInst*
    Decoder::decode_invalid(MachInst iFmt)
    {
        fatal("Invalid opcode encountered: %#x\n", iFmt->imm_u32);

        return nullptr;
    }
} // namespace VegaISA
} // namespace gem5
