/*
 * Copyright (c) 2006 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __ARCH_SPARC_ASI_HH__
#define __ARCH_SPARC_ASI_HH__

namespace gem5
{

namespace SparcISA
{

enum ASI
{
    ASI_IMPLICIT = 0x00,
    /* Priveleged ASIs */
    // 0x00-0x03 implementation dependent
    ASI_NUCLEUS = 0x4,
    ASI_N = 0x4,
    // 0x05-0x0B implementation dependent
    ASI_NL = 0xC,
    ASI_NUCLEUS_LITTLE = ASI_NL,
    // 0x0D-0x0F implementation dependent
    ASI_AIUP = 0x10,
    ASI_AS_IF_USER_PRIMARY = ASI_AIUP,
    ASI_AIUS = 0x11,
    ASI_AS_IF_USER_SECONDARY = ASI_AIUS,
    // 0x12-0x13 implementation dependent
    ASI_REAL = 0x14,
    ASI_REAL_IO = 0x15,
    ASI_BLK_AIUP = 0x16,
    ASI_BLOCK_AS_IF_USER_PRIMARY = ASI_BLK_AIUP,
    ASI_BLK_AIUS = 0x17,
    ASI_BLOCK_AS_IF_USER_SECONDARY = ASI_BLK_AIUS,
    ASI_AIUP_L = 0x18,
    ASI_AS_IF_USER_PRIMARY_LITTLE = ASI_AIUP_L,
    ASI_AIUS_L = 0x19,
    ASI_AS_IF_USER_SECONDARY_LITTLE = ASI_AIUS_L,
    // 0x1A-0x1B implementation dependent
    ASI_REAL_L = 0x1C,
    ASI_REAL_LITTLE = ASI_REAL_L,
    ASI_REAL_IO_L = 0x1D,
    ASI_REAL_IO_LITTLE = ASI_REAL_IO_L,
    ASI_BLK_AIUP_L = 0x1E,
    ASI_BLOCK_AS_IF_USER_PRIMARY_LITTLE = ASI_BLK_AIUP_L,
    ASI_BLK_AIUS_L = 0x1F,
    ASI_BLOCK_AS_IF_USER_SECONDARY_LITTLE = ASI_BLK_AIUS_L,
    ASI_SCRATCHPAD = 0x20,
    ASI_MMU = 0x21,
    ASI_LDTX_AIUP = 0x22,
    ASI_LD_TWINX_AS_IF_USER_PRIMARY = ASI_LDTX_AIUP,
    ASI_LDTX_AIUS = 0x23,
    ASI_LD_TWINX_AS_IF_USER_SECONDARY = ASI_LDTX_AIUS,
    ASI_QUAD_LDD = 0x24,
    ASI_QUEUE = 0x25,
    ASI_QUAD_LDD_REAL = 0x26,
    ASI_LDTX_REAL = ASI_QUAD_LDD_REAL,
    ASI_LDTX_N = 0x27,
    ASI_LD_TWINX_NUCLEUS = ASI_LDTX_N,
    ASI_ST_BLKINIT_NUCLEUS = ASI_LDTX_N,
    ASI_STBI_N = ASI_LDTX_N,
    // 0x28-0x29 implementation dependent
    ASI_LDTX_AIUP_L = 0x2A,
    ASI_TWINX_AS_IF_USER_PRIMARY_LITTLE = ASI_LDTX_AIUP_L,
    ASI_ST_BLKINIT_AS_IF_USER_PRIMARY_LITTLE = ASI_LDTX_AIUP_L,
    ASI_STBI_AIUP_L = ASI_LDTX_AIUP_L,
    ASI_LDTX_AIUS_L = 0x2B,
    ASI_LD_TWINX_AS_IF_USER_SECONDARY_LITTLE = ASI_LDTX_AIUS_L,
    ASI_ST_BLKINIT_AS_IF_USER_SECONDARY_LITTLE = ASI_LDTX_AIUS_L,
    ASI_STBI_AIUS_L = ASI_LDTX_AIUS_L,
    ASI_LTX_L = 0x2C,
    ASI_TWINX_LITTLE = ASI_LTX_L,
    // 0x2D implementation dependent
    ASI_LDTX_REAL_L = 0x2E,
    ASI_LD_TWINX_REAL_LITTLE = ASI_LDTX_REAL_L,
    ASI_LDTX_NL = 0x2F,
    ASI_LD_TWINX_NUCLEUS_LITTLE = ASI_LDTX_NL,
    // 0x20 implementation dependent
    ASI_DMMU_CTXT_ZERO_TSB_BASE_PS0 = 0x31,
    ASI_DMMU_CTXT_ZERO_TSB_BASE_PS1 = 0x32,
    ASI_DMMU_CTXT_ZERO_CONFIG = 0x33,
    // 0x34 implementation dependent
    ASI_IMMU_CTXT_ZERO_TSB_BASE_PS0 = 0x35,
    ASI_IMMU_CTXT_ZERO_TSB_BASE_PS1 = 0x36,
    ASI_IMMU_CTXT_ZERO_CONFIG = 0x37,
    // 0x38 implementation dependent
    ASI_DMMU_CTXT_NONZERO_TSB_BASE_PS0 = 0x39,
    ASI_DMMU_CTXT_NONZERO_TSB_BASE_PS1 = 0x3A,
    ASI_DMMU_CTXT_NONZERO_CONFIG = 0x3B,
    // 0x3C implementation dependent
    ASI_IMMU_CTXT_NONZERO_TSB_BASE_PS0 = 0x3D,
    ASI_IMMU_CTXT_NONZERO_TSB_BASE_PS1 = 0x3E,
    ASI_IMMU_CTXT_NONZERO_CONFIG = 0x3F,
    ASI_STREAM_MA = 0x40,
    ASI_CMT_SHARED = 0x41,
    // 0x41 implementation dependent
    ASI_SPARC_BIST_CONTROL = 0x42,
    ASI_INST_MASK_REG = 0x42,
    ASI_LSU_DIAG_REG = 0x42,
    // 0x43 implementation dependent
    ASI_STM_CTL_REG = 0x44,
    ASI_LSU_CONTROL_REG = 0x45,
    ASI_DCACHE_DATA = 0x46,
    ASI_DCACHE_TAG = 0x47,
    ASI_INTR_DISPATCH_STATUS = 0x48,
    ASI_INTR_RECEIVE = 0x49,
    ASI_UPA_CONFIG_REGISTER = 0x4A,
    ASI_SPARC_ERROR_EN_REG = 0x4B,
    ASI_SPARC_ERROR_STATUS_REG = 0x4C,
    ASI_SPARC_ERROR_ADDRESS_REG = 0x4D,
    ASI_ECACHE_TAG_DATA = 0x4E,
    ASI_HYP_SCRATCHPAD = 0x4F,
    ASI_IMMU = 0x50,
    ASI_IMMU_TSB_PS0_PTR_REG = 0x51,
    ASI_IMMU_TSB_PS1_PTR_REG = 0x52,
    // 0x53 implementation dependent
    ASI_ITLB_DATA_IN_REG = 0x54,
    ASI_ITLB_DATA_ACCESS_REG = 0x55,
    ASI_ITLB_TAG_READ_REG = 0x56,
    ASI_IMMU_DEMAP = 0x57,
    ASI_DMMU = 0x58,
    ASI_DMMU_TSB_PS0_PTR_REG = 0x59,
    ASI_DMMU_TSB_PS1_PTR_REG = 0x5A,
    ASI_DMMU_TSB_DIRECT_PTR_REG = 0x5B,
    ASI_DTLB_DATA_IN_REG = 0x5C,
    ASI_DTLB_DATA_ACCESS_REG = 0x5D,
    ASI_DTLB_TAG_READ_REG = 0x5E,
    ASI_DMMU_DEMAP = 0x5F,
    ASI_TLB_INVALIDATE_ALL = 0x60,
    // 0x61-0x62 implementation dependent
    ASI_CMT_PER_STRAND = 0x63,
    // 0x64-0x65 implementation dependent
    ASI_ICACHE_INSTR = 0x66,
    ASI_ICACHE_TAG = 0x67,
    // 0x68-0x71 implementation dependent
    ASI_SWVR_INTR_RECEIVE = 0x72,
    ASI_SWVR_UDB_INTR_W = 0x73,
    ASI_SWVR_UDB_INTR_R = 0x74,
    // 0x74-0x7F reserved
    /* Unpriveleged ASIs */
    ASI_P = 0x80,
    ASI_PRIMARY = ASI_P,
    ASI_S = 0x81,
    ASI_SECONDARY = ASI_S,
    ASI_PNF = 0x82,
    ASI_PRIMARY_NO_FAULT = ASI_PNF,
    ASI_SNF = 0x83,
    ASI_SECONDARY_NO_FAULT = ASI_SNF,
    // 0x84-0x87 reserved
    ASI_PL = 0x88,
    ASI_PRIMARY_LITTLE = ASI_PL,
    ASI_SL = 0x89,
    ASI_SECONDARY_LITTLE = ASI_SL,
    ASI_PNFL = 0x8A,
    ASI_PRIMARY_NO_FAULT_LITTLE = ASI_PNFL,
    ASI_SNFL = 0x8B,
    ASI_SECONDARY_NO_FAULT_LITTLE = ASI_SNFL,
    // 0x8C-0xBF reserved
    ASI_PST8_P = 0xC0,
    ASI_PST8_PRIMARY = ASI_PST8_P,
    ASI_PST8_S = 0xC1,
    ASI_PST8_SECONDARY = ASI_PST8_S,
    ASI_PST16_P = 0xC2,
    ASI_PST16_PRIMARY = ASI_PST16_P,
    ASI_PST16_S = 0xC3,
    ASI_PST16_SECONDARY = ASI_PST16_S,
    ASI_PST32_P = 0xC4,
    ASI_PST32_PRIMARY = ASI_PST32_P,
    ASI_PST32_S = 0xC5,
    ASI_PST32_SECONDARY = ASI_PST32_S,
    // 0xC6-0xC7 implementation dependent
    ASI_PST8_PL = 0xC8,
    ASI_PST8_PRIMARY_LITTLE = ASI_PST8_PL,
    ASI_PST8_SL = 0xC9,
    ASI_PST8_SECONDARY_LITTLE = ASI_PST8_SL,
    ASI_PST16_PL = 0xCA,
    ASI_PST16_PRIMARY_LITTLE = ASI_PST16_PL,
    ASI_PST16_SL = 0xCB,
    ASI_PST16_SECONDARY_LITTLE = ASI_PST16_SL,
    ASI_PST32_PL = 0xCC,
    ASI_PST32_PRIMARY_LITTLE = ASI_PST32_PL,
    ASI_PST32_SL = 0xCD,
    ASI_PST32_SECONDARY_LITTLE = ASI_PST32_SL,
    // 0xCE-0xCF implementation dependent
    ASI_FL8_P = 0xD0,
    ASI_FL8_PRIMARY = ASI_FL8_P,
    ASI_FL8_S = 0xD1,
    ASI_FL8_SECONDARY = ASI_FL8_S,
    ASI_FL16_P = 0xD2,
    ASI_FL16_PRIMARY = ASI_FL16_P,
    ASI_FL16_S = 0xD3,
    ASI_FL16_SECONDARY = ASI_FL16_S,
    // 0xD4-0xD7 implementation dependent
    ASI_FL8_PL = 0xD8,
    ASI_FL8_PRIMARY_LITTLE = ASI_FL8_PL,
    ASI_FL8_SL = 0xD9,
    ASI_FL8_SECONDARY_LITTLE = ASI_FL8_SL,
    ASI_FL16_PL = 0xDA,
    ASI_FL16_PRIMARY_LITTLE = ASI_FL16_PL,
    ASI_FL16_SL = 0xDB,
    ASI_FL16_SECONDARY_LITTLE = ASI_FL16_SL,
    // 0xDC-0xDF implementation dependent
    // 0xE0-0xE1 reserved
    ASI_LDTX_P = 0xE2,
    ASI_LD_TWINX_PRIMARY = ASI_LDTX_P,
    ASI_LDTX_S = 0xE3,
    ASI_LD_TWINX_SECONDARY = ASI_LDTX_S,
    // 0xE4-0xE9 implementation dependent
    ASI_LDTX_PL = 0xEA,
    ASI_LD_TWINX_PRIMARY_LITTLE = ASI_LDTX_PL,
    ASI_LDTX_SL = 0xEB,
    ASI_LD_TWINX_SECONDARY_LITTLE = ASI_LDTX_SL,
    // 0xEC-0xEF implementation dependent
    ASI_BLK_P = 0xF0,
    ASI_BLOCK_PRIMARY = ASI_BLK_P,
    ASI_BLK_S = 0xF1,
    ASI_BLOCK_SECONDARY = ASI_BLK_S,
    // 0xF2-0xF7 implementation dependent
    ASI_BLK_PL = 0xF8,
    ASI_BLOCK_PRIMARY_LITTLE = ASI_BLK_PL,
    ASI_BLK_SL = 0xF9,
    ASI_BLOCK_SECONDARY_LITTLE = ASI_BLK_SL,
    // 0xFA-0xFF implementation dependent
    MAX_ASI = 0xFF
};

// Functions that classify an asi
bool asiIsBlock(ASI);
bool asiIsPrimary(ASI);
bool asiIsSecondary(ASI);
bool asiIsNucleus(ASI);
bool asiIsAsIfUser(ASI);
bool asiIsIO(ASI);
bool asiIsReal(ASI);
bool asiIsLittle(ASI);
bool asiIsTwin(ASI);
bool asiIsPartialStore(ASI);
bool asiIsFloatingLoad(ASI);
bool asiIsNoFault(ASI);
bool asiIsScratchPad(ASI);
bool asiIsCmt(ASI);
bool asiIsQueue(ASI);
bool asiIsDtlb(ASI);
bool asiIsMmu(ASI);
bool asiIsUnPriv(ASI);
bool asiIsPriv(ASI);
bool asiIsHPriv(ASI);
bool asiIsReg(ASI);
bool asiIsInterrupt(ASI);
bool asiIsSparcError(ASI);
}; // namespace SparcISA

} // namespace gem5

#endif // __ARCH_SPARC_ASI_HH__
