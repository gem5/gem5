/*
 *  Copyright (c) 2021, Markku-Juhani O. Saarinen <mjos@pqshield.com>
 *  All rights reserved.
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

#ifndef __ARCH_RISCV_RVK_HH__
#define __ARCH_RISCV_RVK_HH__

#include <cstdint>

// Standard scalar cryptography extension

namespace gem5
{

namespace RiscvISA
{

/**
 * Ref:
 * https://github.com/rvkrypto/rvkrypto-fips
 */

const uint8_t _rvk_emu_aes_fwd_sbox[256] = {
    0x63, 0x7C, 0x77, 0x7B, 0xF2, 0x6B, 0x6F, 0xC5, 0x30, 0x01, 0x67, 0x2B,
    0xFE, 0xD7, 0xAB, 0x76, 0xCA, 0x82, 0xC9, 0x7D, 0xFA, 0x59, 0x47, 0xF0,
    0xAD, 0xD4, 0xA2, 0xAF, 0x9C, 0xA4, 0x72, 0xC0, 0xB7, 0xFD, 0x93, 0x26,
    0x36, 0x3F, 0xF7, 0xCC, 0x34, 0xA5, 0xE5, 0xF1, 0x71, 0xD8, 0x31, 0x15,
    0x04, 0xC7, 0x23, 0xC3, 0x18, 0x96, 0x05, 0x9A, 0x07, 0x12, 0x80, 0xE2,
    0xEB, 0x27, 0xB2, 0x75, 0x09, 0x83, 0x2C, 0x1A, 0x1B, 0x6E, 0x5A, 0xA0,
    0x52, 0x3B, 0xD6, 0xB3, 0x29, 0xE3, 0x2F, 0x84, 0x53, 0xD1, 0x00, 0xED,
    0x20, 0xFC, 0xB1, 0x5B, 0x6A, 0xCB, 0xBE, 0x39, 0x4A, 0x4C, 0x58, 0xCF,
    0xD0, 0xEF, 0xAA, 0xFB, 0x43, 0x4D, 0x33, 0x85, 0x45, 0xF9, 0x02, 0x7F,
    0x50, 0x3C, 0x9F, 0xA8, 0x51, 0xA3, 0x40, 0x8F, 0x92, 0x9D, 0x38, 0xF5,
    0xBC, 0xB6, 0xDA, 0x21, 0x10, 0xFF, 0xF3, 0xD2, 0xCD, 0x0C, 0x13, 0xEC,
    0x5F, 0x97, 0x44, 0x17, 0xC4, 0xA7, 0x7E, 0x3D, 0x64, 0x5D, 0x19, 0x73,
    0x60, 0x81, 0x4F, 0xDC, 0x22, 0x2A, 0x90, 0x88, 0x46, 0xEE, 0xB8, 0x14,
    0xDE, 0x5E, 0x0B, 0xDB, 0xE0, 0x32, 0x3A, 0x0A, 0x49, 0x06, 0x24, 0x5C,
    0xC2, 0xD3, 0xAC, 0x62, 0x91, 0x95, 0xE4, 0x79, 0xE7, 0xC8, 0x37, 0x6D,
    0x8D, 0xD5, 0x4E, 0xA9, 0x6C, 0x56, 0xF4, 0xEA, 0x65, 0x7A, 0xAE, 0x08,
    0xBA, 0x78, 0x25, 0x2E, 0x1C, 0xA6, 0xB4, 0xC6, 0xE8, 0xDD, 0x74, 0x1F,
    0x4B, 0xBD, 0x8B, 0x8A, 0x70, 0x3E, 0xB5, 0x66, 0x48, 0x03, 0xF6, 0x0E,
    0x61, 0x35, 0x57, 0xB9, 0x86, 0xC1, 0x1D, 0x9E, 0xE1, 0xF8, 0x98, 0x11,
    0x69, 0xD9, 0x8E, 0x94, 0x9B, 0x1E, 0x87, 0xE9, 0xCE, 0x55, 0x28, 0xDF,
    0x8C, 0xA1, 0x89, 0x0D, 0xBF, 0xE6, 0x42, 0x68, 0x41, 0x99, 0x2D, 0x0F,
    0xB0, 0x54, 0xBB, 0x16
};

// AES Inverse S-Box
const uint8_t _rvk_emu_aes_inv_sbox[256] = {
    0x52, 0x09, 0x6A, 0xD5, 0x30, 0x36, 0xA5, 0x38, 0xBF, 0x40, 0xA3, 0x9E,
    0x81, 0xF3, 0xD7, 0xFB, 0x7C, 0xE3, 0x39, 0x82, 0x9B, 0x2F, 0xFF, 0x87,
    0x34, 0x8E, 0x43, 0x44, 0xC4, 0xDE, 0xE9, 0xCB, 0x54, 0x7B, 0x94, 0x32,
    0xA6, 0xC2, 0x23, 0x3D, 0xEE, 0x4C, 0x95, 0x0B, 0x42, 0xFA, 0xC3, 0x4E,
    0x08, 0x2E, 0xA1, 0x66, 0x28, 0xD9, 0x24, 0xB2, 0x76, 0x5B, 0xA2, 0x49,
    0x6D, 0x8B, 0xD1, 0x25, 0x72, 0xF8, 0xF6, 0x64, 0x86, 0x68, 0x98, 0x16,
    0xD4, 0xA4, 0x5C, 0xCC, 0x5D, 0x65, 0xB6, 0x92, 0x6C, 0x70, 0x48, 0x50,
    0xFD, 0xED, 0xB9, 0xDA, 0x5E, 0x15, 0x46, 0x57, 0xA7, 0x8D, 0x9D, 0x84,
    0x90, 0xD8, 0xAB, 0x00, 0x8C, 0xBC, 0xD3, 0x0A, 0xF7, 0xE4, 0x58, 0x05,
    0xB8, 0xB3, 0x45, 0x06, 0xD0, 0x2C, 0x1E, 0x8F, 0xCA, 0x3F, 0x0F, 0x02,
    0xC1, 0xAF, 0xBD, 0x03, 0x01, 0x13, 0x8A, 0x6B, 0x3A, 0x91, 0x11, 0x41,
    0x4F, 0x67, 0xDC, 0xEA, 0x97, 0xF2, 0xCF, 0xCE, 0xF0, 0xB4, 0xE6, 0x73,
    0x96, 0xAC, 0x74, 0x22, 0xE7, 0xAD, 0x35, 0x85, 0xE2, 0xF9, 0x37, 0xE8,
    0x1C, 0x75, 0xDF, 0x6E, 0x47, 0xF1, 0x1A, 0x71, 0x1D, 0x29, 0xC5, 0x89,
    0x6F, 0xB7, 0x62, 0x0E, 0xAA, 0x18, 0xBE, 0x1B, 0xFC, 0x56, 0x3E, 0x4B,
    0xC6, 0xD2, 0x79, 0x20, 0x9A, 0xDB, 0xC0, 0xFE, 0x78, 0xCD, 0x5A, 0xF4,
    0x1F, 0xDD, 0xA8, 0x33, 0x88, 0x07, 0xC7, 0x31, 0xB1, 0x12, 0x10, 0x59,
    0x27, 0x80, 0xEC, 0x5F, 0x60, 0x51, 0x7F, 0xA9, 0x19, 0xB5, 0x4A, 0x0D,
    0x2D, 0xE5, 0x7A, 0x9F, 0x93, 0xC9, 0x9C, 0xEF, 0xA0, 0xE0, 0x3B, 0x4D,
    0xAE, 0x2A, 0xF5, 0xB0, 0xC8, 0xEB, 0xBB, 0x3C, 0x83, 0x53, 0x99, 0x61,
    0x17, 0x2B, 0x04, 0x7E, 0xBA, 0x77, 0xD6, 0x26, 0xE1, 0x69, 0x14, 0x63,
    0x55, 0x21, 0x0C, 0x7D
};

// SM4 Forward S-Box (there is no need for an inverse S-Box)
const uint8_t _rvk_emu_sm4_sbox[256] = {
    0xD6, 0x90, 0xE9, 0xFE, 0xCC, 0xE1, 0x3D, 0xB7, 0x16, 0xB6, 0x14, 0xC2,
    0x28, 0xFB, 0x2C, 0x05, 0x2B, 0x67, 0x9A, 0x76, 0x2A, 0xBE, 0x04, 0xC3,
    0xAA, 0x44, 0x13, 0x26, 0x49, 0x86, 0x06, 0x99, 0x9C, 0x42, 0x50, 0xF4,
    0x91, 0xEF, 0x98, 0x7A, 0x33, 0x54, 0x0B, 0x43, 0xED, 0xCF, 0xAC, 0x62,
    0xE4, 0xB3, 0x1C, 0xA9, 0xC9, 0x08, 0xE8, 0x95, 0x80, 0xDF, 0x94, 0xFA,
    0x75, 0x8F, 0x3F, 0xA6, 0x47, 0x07, 0xA7, 0xFC, 0xF3, 0x73, 0x17, 0xBA,
    0x83, 0x59, 0x3C, 0x19, 0xE6, 0x85, 0x4F, 0xA8, 0x68, 0x6B, 0x81, 0xB2,
    0x71, 0x64, 0xDA, 0x8B, 0xF8, 0xEB, 0x0F, 0x4B, 0x70, 0x56, 0x9D, 0x35,
    0x1E, 0x24, 0x0E, 0x5E, 0x63, 0x58, 0xD1, 0xA2, 0x25, 0x22, 0x7C, 0x3B,
    0x01, 0x21, 0x78, 0x87, 0xD4, 0x00, 0x46, 0x57, 0x9F, 0xD3, 0x27, 0x52,
    0x4C, 0x36, 0x02, 0xE7, 0xA0, 0xC4, 0xC8, 0x9E, 0xEA, 0xBF, 0x8A, 0xD2,
    0x40, 0xC7, 0x38, 0xB5, 0xA3, 0xF7, 0xF2, 0xCE, 0xF9, 0x61, 0x15, 0xA1,
    0xE0, 0xAE, 0x5D, 0xA4, 0x9B, 0x34, 0x1A, 0x55, 0xAD, 0x93, 0x32, 0x30,
    0xF5, 0x8C, 0xB1, 0xE3, 0x1D, 0xF6, 0xE2, 0x2E, 0x82, 0x66, 0xCA, 0x60,
    0xC0, 0x29, 0x23, 0xAB, 0x0D, 0x53, 0x4E, 0x6F, 0xD5, 0xDB, 0x37, 0x45,
    0xDE, 0xFD, 0x8E, 0x2F, 0x03, 0xFF, 0x6A, 0x72, 0x6D, 0x6C, 0x5B, 0x51,
    0x8D, 0x1B, 0xAF, 0x92, 0xBB, 0xDD, 0xBC, 0x7F, 0x11, 0xD9, 0x5C, 0x41,
    0x1F, 0x10, 0x5A, 0xD8, 0x0A, 0xC1, 0x31, 0x88, 0xA5, 0xCD, 0x7B, 0xBD,
    0x2D, 0x74, 0xD0, 0x12, 0xB8, 0xE5, 0xB4, 0xB0, 0x89, 0x69, 0x97, 0x4A,
    0x0C, 0x96, 0x77, 0x7E, 0x65, 0xB9, 0xF1, 0x09, 0xC5, 0x6E, 0xC6, 0x84,
    0x18, 0xF0, 0x7D, 0xEC, 0x3A, 0xDC, 0x4D, 0x20, 0x79, 0xEE, 0x5F, 0x3E,
    0xD7, 0xCB, 0x39, 0x48
};

inline int32_t _rvk_emu_sll_32(int32_t rs1, int32_t rs2)
    { return rs1 << (rs2 & 31); }
inline int32_t _rvk_emu_srl_32(int32_t rs1, int32_t rs2)
    { return (uint32_t)rs1 >> (rs2 & 31); }
inline int64_t _rvk_emu_sll_64(int64_t rs1, int64_t rs2)
    { return rs1 << (rs2 & 63); }
inline int64_t _rvk_emu_srl_64(int64_t rs1, int64_t rs2)
    { return (uint64_t)rs1 >> (rs2 & 63); }

// rotate (a part of the extension). no separate intrinsic for rori
inline int32_t _rvk_emu_rol_32(int32_t rs1, int32_t rs2)
    { return _rvk_emu_sll_32(rs1, rs2) | _rvk_emu_srl_32(rs1, -rs2); }
inline int32_t _rvk_emu_ror_32(int32_t rs1, int32_t rs2)
    { return _rvk_emu_srl_32(rs1, rs2) | _rvk_emu_sll_32(rs1, -rs2); }

inline int64_t _rvk_emu_rol_64(int64_t rs1, int64_t rs2)
    { return _rvk_emu_sll_64(rs1, rs2) | _rvk_emu_srl_64(rs1, -rs2); }
inline int64_t _rvk_emu_ror_64(int64_t rs1, int64_t rs2)
    { return _rvk_emu_srl_64(rs1, rs2) | _rvk_emu_sll_64(rs1, -rs2); }

// brev8, rev8
inline int32_t _rvk_emu_grev_32(int32_t rs1, int32_t rs2)
{
    uint32_t x = rs1;
    int shamt = rs2 & 31;
    if (shamt &  1) x = ((x & 0x55555555) <<  1) | ((x & 0xAAAAAAAA) >>  1);
    if (shamt &  2) x = ((x & 0x33333333) <<  2) | ((x & 0xCCCCCCCC) >>  2);
    if (shamt &  4) x = ((x & 0x0F0F0F0F) <<  4) | ((x & 0xF0F0F0F0) >>  4);
    if (shamt &  8) x = ((x & 0x00FF00FF) <<  8) | ((x & 0xFF00FF00) >>  8);
    if (shamt & 16) x = ((x & 0x0000FFFF) << 16) | ((x & 0xFFFF0000) >> 16);
    return x;
}

inline int64_t _rvk_emu_grev_64(int64_t rs1, int64_t rs2)
{
    uint64_t x = rs1;
    int shamt = rs2 & 63;
    if (shamt &  1)
        x = ((x & 0x5555555555555555LL) << 1) |
            ((x & 0xAAAAAAAAAAAAAAAALL) >> 1);
    if (shamt &  2)
        x = ((x & 0x3333333333333333LL) << 2) |
            ((x & 0xCCCCCCCCCCCCCCCCLL) >> 2);
    if (shamt &  4)
        x = ((x & 0x0F0F0F0F0F0F0F0FLL) << 4) |
            ((x & 0xF0F0F0F0F0F0F0F0LL) >> 4);
    if (shamt &  8)
        x = ((x & 0x00FF00FF00FF00FFLL) << 8) |
            ((x & 0xFF00FF00FF00FF00LL) >> 8);
    if (shamt & 16)
        x = ((x & 0x0000FFFF0000FFFFLL) << 16) |
            ((x & 0xFFFF0000FFFF0000LL) >> 16);
    if (shamt & 32)
        x = ((x & 0x00000000FFFFFFFFLL) << 32) |
            ((x & 0xFFFFFFFF00000000LL) >> 32);
    return x;
}

inline int32_t _rvk_emu_brev8_32(int32_t rs1)
    { return _rvk_emu_grev_32(rs1, 7); }

inline int64_t _rvk_emu_brev8_64(int64_t rs1)
    { return _rvk_emu_grev_64(rs1, 7); }

inline uint32_t _rvk_emu_shuffle32_stage(uint32_t src,
    uint32_t maskL, uint32_t maskR, int N)
{
    uint32_t x = src & ~(maskL | maskR);
    x |= ((src <<  N) & maskL) | ((src >>  N) & maskR);
    return x;
}

inline int32_t _rvk_emu_shfl_32(int32_t rs1, int32_t rs2)
{
    uint32_t x = rs1;
    int shamt = rs2 & 15;

    if (shamt & 8) x = _rvk_emu_shuffle32_stage(x, 0x00ff0000, 0x0000ff00, 8);
    if (shamt & 4) x = _rvk_emu_shuffle32_stage(x, 0x0f000f00, 0x00f000f0, 4);
    if (shamt & 2) x = _rvk_emu_shuffle32_stage(x, 0x30303030, 0x0c0c0c0c, 2);
    if (shamt & 1) x = _rvk_emu_shuffle32_stage(x, 0x44444444, 0x22222222, 1);

    return x;
}

inline int32_t _rvk_emu_unshfl_32(int32_t rs1, int32_t rs2)
{
    uint32_t x = rs1;
    int shamt = rs2 & 15;

    if (shamt & 1) x = _rvk_emu_shuffle32_stage(x, 0x44444444, 0x22222222, 1);
    if (shamt & 2) x = _rvk_emu_shuffle32_stage(x, 0x30303030, 0x0c0c0c0c, 2);
    if (shamt & 4) x = _rvk_emu_shuffle32_stage(x, 0x0f000f00, 0x00f000f0, 4);
    if (shamt & 8) x = _rvk_emu_shuffle32_stage(x, 0x00ff0000, 0x0000ff00, 8);

    return x;
}

inline int32_t _rvk_emu_zip_32(int32_t rs1)
    { return _rvk_emu_shfl_32(rs1, 15); }

inline int32_t _rvk_emu_unzip_32(int32_t rs1)
    { return _rvk_emu_unshfl_32(rs1, 15); }

// Zbkc: Carry-less multiply instructions
inline int32_t _rvk_emu_clmul_32(int32_t rs1, int32_t rs2)
{
    uint32_t a = rs1, b = rs2, x = 0;
    for (int i = 0; i < 32; i++) {
        if ((b >> i) & 1)
            x ^= a << i;
    }
    return x;
}

inline int32_t _rvk_emu_clmulh_32(int32_t rs1, int32_t rs2)
{
    uint32_t a = rs1, b = rs2, x = 0;
    for (int i = 1; i < 32; i++) {
        if ((b >> i) & 1)
            x ^= a >> (32-i);
    }
    return x;
}

inline int64_t _rvk_emu_clmul_64(int64_t rs1, int64_t rs2)
{
    uint64_t a = rs1, b = rs2, x = 0;

    for (int i = 0; i < 64; i++) {
        if ((b >> i) & 1)
            x ^= a << i;
    }
    return x;
}

inline int64_t _rvk_emu_clmulh_64(int64_t rs1, int64_t rs2)
{
    uint64_t a = rs1, b = rs2, x = 0;

    for (int i = 1; i < 64; i++) {
        if ((b >> i) & 1)
            x ^= a >> (64-i);
    }
    return x;
}

// Zbkx: Crossbar permutation instructions
inline uint32_t _rvk_emu_xperm32(uint32_t rs1, uint32_t rs2, int sz_log2)
{
    uint32_t r = 0;
    uint32_t sz = 1LL << sz_log2;
    uint32_t mask = (1LL << sz) - 1;
    for (int i = 0; i < 32; i += sz) {
        uint32_t pos = ((rs2 >> i) & mask) << sz_log2;
        if (pos < 32)
            r |= ((rs1 >> pos) & mask) << i;
    }
    return r;
}

inline int32_t _rvk_emu_xperm4_32(int32_t rs1, int32_t rs2)
    { return _rvk_emu_xperm32(rs1, rs2, 2); }

inline int32_t _rvk_emu_xperm8_32(int32_t rs1, int32_t rs2)
    { return _rvk_emu_xperm32(rs1, rs2, 3); }

inline uint64_t _rvk_emu_xperm64(uint64_t rs1, uint64_t rs2, int sz_log2)
{
    uint64_t r = 0;
    uint64_t sz = 1LL << sz_log2;
    uint64_t mask = (1LL << sz) - 1;
    for (int i = 0; i < 64; i += sz) {
        uint64_t pos = ((rs2 >> i) & mask) << sz_log2;
        if (pos < 64)
            r |= ((rs1 >> pos) & mask) << i;
    }
    return r;
}

inline int64_t _rvk_emu_xperm4_64(int64_t rs1, int64_t rs2)
    { return _rvk_emu_xperm64(rs1, rs2, 2); }

inline int64_t _rvk_emu_xperm8_64(int64_t rs1, int64_t rs2)
    { return _rvk_emu_xperm64(rs1, rs2, 3); }

// rvk_emu internal: multiply by 0x02 in AES's GF(256) - LFSR style.
inline uint8_t _rvk_emu_aes_xtime(uint8_t x)
{
    return (x << 1) ^ ((x & 0x80) ? 0x11B : 0x00);
}

// rvk_emu internal: AES forward MixColumns 8->32 bits
inline uint32_t _rvk_emu_aes_fwd_mc_8(uint32_t x)
{
    uint32_t x2;
    x2 = _rvk_emu_aes_xtime(x);
    x = ((x ^ x2) << 24) | (x << 16) |(x << 8) | x2;
    return x;
}

// rvk_emu internal: AES forward MixColumns 32->32 bits
inline uint32_t _rvk_emu_aes_fwd_mc_32(uint32_t x)
{
    return _rvk_emu_aes_fwd_mc_8(x & 0xFF) ^
        _rvk_emu_rol_32(_rvk_emu_aes_fwd_mc_8((x >>  8) & 0xFF),  8) ^
        _rvk_emu_rol_32(_rvk_emu_aes_fwd_mc_8((x >> 16) & 0xFF), 16) ^
        _rvk_emu_rol_32(_rvk_emu_aes_fwd_mc_8((x >> 24) & 0xFF), 24);
}

// rvk_emu internal: AES inverse MixColumns 8->32 bits
inline uint32_t _rvk_emu_aes_inv_mc_8(uint32_t x)
{
    uint32_t x2, x4, x8;

    x2 = _rvk_emu_aes_xtime(x);
    x4 = _rvk_emu_aes_xtime(x2);
    x8 = _rvk_emu_aes_xtime(x4);

    x = ((x ^ x2 ^ x8) << 24) |
    ((x ^ x4 ^ x8) << 16) |
    ((x ^ x8) << 8) |
    (x2 ^ x4 ^ x8);

    return x;
}

// rvk_emu internal: AES inverse MixColumns 32->32 bits
inline uint32_t _rvk_emu_aes_inv_mc_32(uint32_t x)
{
    return _rvk_emu_aes_inv_mc_8(x & 0xFF) ^
        _rvk_emu_rol_32(_rvk_emu_aes_inv_mc_8((x >>  8) & 0xFF),  8) ^
        _rvk_emu_rol_32(_rvk_emu_aes_inv_mc_8((x >> 16) & 0xFF), 16) ^
        _rvk_emu_rol_32(_rvk_emu_aes_inv_mc_8((x >> 24) & 0xFF), 24);
}

// Zknd: NIST Suite: AES Decryption
inline int32_t _rvk_emu_aes32dsi(int32_t rs1, int32_t rs2, uint8_t bs)
{
    int32_t x;

    bs = (bs & 3) << 3;
    x = (rs2 >> bs) & 0xFF;
    x = _rvk_emu_aes_inv_sbox[x];

    return rs1 ^ _rvk_emu_rol_32(x, bs);
}

inline int32_t _rvk_emu_aes32dsmi(int32_t rs1, int32_t rs2, uint8_t bs)
{
    int32_t x;

    bs = (bs & 3) << 3;
    x = (rs2 >> bs) & 0xFF;
    x = _rvk_emu_aes_inv_sbox[x];
    x = _rvk_emu_aes_inv_mc_8(x);

    return rs1 ^ _rvk_emu_rol_32(x, bs);
}

inline int64_t _rvk_emu_aes64ds(int64_t rs1, int64_t rs2)
{
    return ((int64_t) _rvk_emu_aes_inv_sbox[rs1 & 0xFF]) |
        (((int64_t) _rvk_emu_aes_inv_sbox[(rs2 >> 40) & 0xFF]) <<  8) |
        (((int64_t) _rvk_emu_aes_inv_sbox[(rs2 >> 16) & 0xFF]) << 16) |
        (((int64_t) _rvk_emu_aes_inv_sbox[(rs1 >> 56) & 0xFF]) << 24) |
        (((int64_t) _rvk_emu_aes_inv_sbox[(rs1 >> 32) & 0xFF]) << 32) |
        (((int64_t) _rvk_emu_aes_inv_sbox[(rs1 >>  8) & 0xFF]) << 40) |
        (((int64_t) _rvk_emu_aes_inv_sbox[(rs2 >> 48) & 0xFF]) << 48) |
        (((int64_t) _rvk_emu_aes_inv_sbox[(rs2 >> 24) & 0xFF]) << 56);
}

inline int64_t _rvk_emu_aes64im(int64_t rs1)
{
    return ((int64_t) _rvk_emu_aes_inv_mc_32(rs1)) |
        (((int64_t) _rvk_emu_aes_inv_mc_32(rs1 >> 32)) << 32);
}

inline int64_t _rvk_emu_aes64dsm(int64_t rs1, int64_t rs2)
{
    int64_t x;

    x = _rvk_emu_aes64ds(rs1, rs2);
    x = _rvk_emu_aes64im(x);
    return x;
}

inline int64_t _rvk_emu_aes64ks1i(int64_t rs1, int rnum)
{
    const uint8_t aes_rcon[] = {
        0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1B, 0x36
    };

    uint32_t t, rc;

    t = rs1 >> 32;
    rc = 0;

    if (rnum < 10) {
        t = _rvk_emu_ror_32(t, 8);
        rc = aes_rcon[rnum];
    }

    t = ((uint32_t) _rvk_emu_aes_fwd_sbox[t & 0xFF]) |
        (((uint32_t) _rvk_emu_aes_fwd_sbox[(t >>  8) & 0xFF]) <<  8) |
        (((uint32_t) _rvk_emu_aes_fwd_sbox[(t >> 16) & 0xFF]) << 16) |
        (((uint32_t) _rvk_emu_aes_fwd_sbox[(t >> 24) & 0xFF]) << 24);

    t ^= rc;

    return ((int64_t) t) | (((int64_t) t) << 32);
}

inline int64_t _rvk_emu_aes64ks2(int64_t rs1, int64_t rs2)
{
    uint32_t t;

    t = (rs1 >> 32) ^ (rs2 & 0xFFFFFFFF);

    return ((int64_t) t) ^
        (((int64_t) t) << 32) ^ (rs2 & 0xFFFFFFFF00000000ULL);
}

inline int32_t _rvk_emu_aes32esi(int32_t rs1, int32_t rs2, uint8_t bs)
{
    int32_t x;

    bs = (bs & 3) << 3;
    x = (rs2 >> bs) & 0xFF;
    x = _rvk_emu_aes_fwd_sbox[x];

    return rs1 ^ _rvk_emu_rol_32(x, bs);
}

inline int32_t _rvk_emu_aes32esmi(int32_t rs1, int32_t rs2, uint8_t bs)
{
    uint32_t x;

    bs = (bs & 3) << 3;
    x = (rs2 >> bs) & 0xFF;
    x = _rvk_emu_aes_fwd_sbox[x];
    x = _rvk_emu_aes_fwd_mc_8(x);

    return rs1 ^ _rvk_emu_rol_32(x, bs);
}

inline int64_t _rvk_emu_aes64es(int64_t rs1, int64_t rs2)
{
    return ((int64_t) _rvk_emu_aes_fwd_sbox[rs1 & 0xFF]) |
        (((int64_t) _rvk_emu_aes_fwd_sbox[(rs1 >> 40) & 0xFF]) <<  8) |
        (((int64_t) _rvk_emu_aes_fwd_sbox[(rs2 >> 16) & 0xFF]) << 16) |
        (((int64_t) _rvk_emu_aes_fwd_sbox[(rs2 >> 56) & 0xFF]) << 24) |
        (((int64_t) _rvk_emu_aes_fwd_sbox[(rs1 >> 32) & 0xFF]) << 32) |
        (((int64_t) _rvk_emu_aes_fwd_sbox[(rs2 >>  8) & 0xFF]) << 40) |
        (((int64_t) _rvk_emu_aes_fwd_sbox[(rs2 >> 48) & 0xFF]) << 48) |
        (((int64_t) _rvk_emu_aes_fwd_sbox[(rs1 >> 24) & 0xFF]) << 56);
}

inline int64_t _rvk_emu_aes64esm(int64_t rs1, int64_t rs2)
{
    int64_t x;

    x = _rvk_emu_aes64es(rs1, rs2);
    x = ((int64_t) _rvk_emu_aes_fwd_mc_32(x)) |
        (((int64_t) _rvk_emu_aes_fwd_mc_32(x >> 32)) << 32);
    return x;
}

inline int32_t _rvk_emu_sha256sig0(int32_t rs1)
{
    int32_t x;

    x = _rvk_emu_ror_32(rs1, 7) ^ _rvk_emu_ror_32(rs1, 18) ^
        _rvk_emu_srl_32(rs1, 3);
    return (int32_t) x;
}

inline int32_t _rvk_emu_sha256sig1(int32_t rs1)
{
    int32_t x;

    x = _rvk_emu_ror_32(rs1, 17) ^ _rvk_emu_ror_32(rs1, 19) ^
        _rvk_emu_srl_32(rs1, 10);
    return (int32_t) x;
}

inline int32_t _rvk_emu_sha256sum0(int32_t rs1)
{
    int32_t x;

    x = _rvk_emu_ror_32(rs1, 2) ^ _rvk_emu_ror_32(rs1, 13) ^
        _rvk_emu_ror_32(rs1, 22);
    return (int32_t) x;
}

inline int32_t _rvk_emu_sha256sum1(int32_t rs1)
{
    int32_t x;

    x = _rvk_emu_ror_32(rs1, 6) ^ _rvk_emu_ror_32(rs1, 11) ^
        _rvk_emu_ror_32(rs1, 25);
    return (int32_t) x;
}

static inline int32_t  _rvk_emu_sha512sig0h(int32_t rs1, int32_t rs2)
{
    return  _rvk_emu_srl_32(rs1, 1) ^ _rvk_emu_srl_32(rs1, 7) ^
            _rvk_emu_srl_32(rs1, 8) ^ _rvk_emu_sll_32(rs2, 31) ^
            _rvk_emu_sll_32(rs2, 24);
}

static inline int32_t  _rvk_emu_sha512sig0l(int32_t rs1, int32_t rs2)
{
    return  _rvk_emu_srl_32(rs1, 1) ^ _rvk_emu_srl_32(rs1, 7) ^
            _rvk_emu_srl_32(rs1, 8) ^ _rvk_emu_sll_32(rs2, 31) ^
            _rvk_emu_sll_32(rs2, 25) ^ _rvk_emu_sll_32(rs2, 24);
}

static inline int32_t  _rvk_emu_sha512sig1h(int32_t rs1, int32_t rs2)
{
    return  _rvk_emu_sll_32(rs1, 3) ^ _rvk_emu_srl_32(rs1, 6) ^
            _rvk_emu_srl_32(rs1, 19) ^ _rvk_emu_srl_32(rs2, 29) ^
            _rvk_emu_sll_32(rs2, 13);
}

static inline int32_t  _rvk_emu_sha512sig1l(int32_t rs1, int32_t rs2)
{
    return  _rvk_emu_sll_32(rs1, 3) ^ _rvk_emu_srl_32(rs1, 6) ^
            _rvk_emu_srl_32(rs1,19) ^ _rvk_emu_srl_32(rs2, 29) ^
            _rvk_emu_sll_32(rs2, 26) ^ _rvk_emu_sll_32(rs2, 13);
}

static inline int32_t  _rvk_emu_sha512sum0r(int32_t rs1, int32_t rs2)
{
    return  _rvk_emu_sll_32(rs1, 25) ^ _rvk_emu_sll_32(rs1, 30) ^
            _rvk_emu_srl_32(rs1, 28) ^ _rvk_emu_srl_32(rs2, 7) ^
            _rvk_emu_srl_32(rs2, 2) ^ _rvk_emu_sll_32(rs2, 4);
}

static inline int32_t  _rvk_emu_sha512sum1r(int32_t rs1, int32_t rs2)
{
    return  _rvk_emu_sll_32(rs1, 23) ^ _rvk_emu_srl_32(rs1,14) ^
            _rvk_emu_srl_32(rs1, 18) ^ _rvk_emu_srl_32(rs2, 9) ^
            _rvk_emu_sll_32(rs2, 18) ^ _rvk_emu_sll_32(rs2, 14);
}

inline int64_t  _rvk_emu_sha512sig0(int64_t rs1)
{
    return _rvk_emu_ror_64(rs1, 1) ^ _rvk_emu_ror_64(rs1, 8) ^
            _rvk_emu_srl_64(rs1,7);
}

inline int64_t  _rvk_emu_sha512sig1(int64_t rs1)
{
    return _rvk_emu_ror_64(rs1, 19) ^ _rvk_emu_ror_64(rs1, 61) ^
            _rvk_emu_srl_64(rs1, 6);
}

inline int64_t  _rvk_emu_sha512sum0(int64_t rs1)
{
    return _rvk_emu_ror_64(rs1, 28) ^ _rvk_emu_ror_64(rs1, 34) ^
            _rvk_emu_ror_64(rs1, 39);
}

inline int64_t  _rvk_emu_sha512sum1(int64_t rs1)
{
    return _rvk_emu_ror_64(rs1, 14) ^ _rvk_emu_ror_64(rs1, 18) ^
            _rvk_emu_ror_64(rs1, 41);
}

// Zksed: ShangMi Suite: SM4 Block Cipher Instructions
inline int32_t _rvk_emu_sm4ed(int32_t rs1, int32_t rs2, uint8_t bs)
{
    int32_t x;

    bs = (bs & 3) << 3;
    x = (rs2 >> bs) & 0xFF;
    x = _rvk_emu_sm4_sbox[x];

    x = x ^ (x << 8) ^ (x << 2) ^ (x << 18) ^
            ((x & 0x3F) << 26) ^ ((x & 0xC0) << 10);
    x = rs1 ^ _rvk_emu_rol_32(x, bs);
    return (int32_t) x;
}

inline int32_t _rvk_emu_sm4ks(int32_t rs1, int32_t rs2, uint8_t bs)
{
    int32_t x;

    bs = (bs & 3) << 3;
    x = (rs2 >> bs) & 0xFF;
    x = _rvk_emu_sm4_sbox[x];

    x = x ^ ((x & 0x07) << 29) ^ ((x & 0xFE) << 7) ^
        ((x & 1) << 23) ^ ((x & 0xF8) << 13);
    x = rs1 ^ _rvk_emu_rol_32(x, bs);
    return (int32_t) x;
}

// Zksh: ShangMi Suite: SM3 Hash Function Instructions
inline int32_t  _rvk_emu_sm3p0(int32_t rs1)
{
    int32_t x;

    x = rs1 ^ _rvk_emu_rol_32(rs1, 9) ^ _rvk_emu_rol_32(rs1, 17);
    return (int32_t) x;
}

inline int32_t  _rvk_emu_sm3p1(int32_t rs1)
{
    int32_t x;

    x = rs1 ^ _rvk_emu_rol_32(rs1, 15) ^ _rvk_emu_rol_32(rs1, 23);
    return (int32_t) x;
}

} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_UTILITY_HH__
