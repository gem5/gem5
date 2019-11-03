/*
 * Copyright (c) 2007 MIPS Technologies, Inc.
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
 *
 * Authors: Brett Miller
 */

#ifndef __ARCH_MIPS_DSP_HH__
#define __ARCH_MIPS_DSP_HH__

#include "arch/mips/isa_traits.hh"
#include "arch/mips/types.hh"
#include "base/logging.hh"
#include "base/types.hh"

class ThreadContext;

namespace MipsISA {

// SIMD formats
enum {
    SIMD_FMT_L,    // long word
    SIMD_FMT_W,    // word
    SIMD_FMT_PH,   // paired halfword
    SIMD_FMT_QB,   // quad byte
    SIMD_NUM_FMTS
};

// DSPControl Fields
enum {
    DSP_POS,       // insertion bitfield position
    DSP_SCOUNT,    // insertion bitfield size
    DSP_C,         // carry bit
    DSP_OUFLAG,    // overflow-underflow flag
    DSP_CCOND,     // condition code
    DSP_EFI,       // extract fail indicator bit
    DSP_NUM_FIELDS
};

// compare instruction operations
enum {
    CMP_EQ,        // equal
    CMP_LT,        // less than
    CMP_LE         // less than or equal
};

// SIMD operation order modes
enum {
    MODE_L,        // left
    MODE_R,        // right
    MODE_LA,       // left-alternate
    MODE_RA,       // right-alternate
    MODE_X         // cross
};

// dsp operation parameters
enum { UNSIGNED, SIGNED };
enum { NOSATURATE, SATURATE };
enum { NOROUND, ROUND };

// DSPControl field positions and masks
const uint32_t DSP_CTL_POS[DSP_NUM_FIELDS] = { 0, 7, 13, 16, 24, 14 };
const uint32_t DSP_CTL_MASK[DSP_NUM_FIELDS] =
{ 0x0000003f, 0x00001f80, 0x00002000,
  0x00ff0000, 0x0f000000, 0x00004000 };

/*
 * SIMD format constants
 */

// maximum values per register
const uint32_t SIMD_MAX_VALS = 4;
// number of values in fmt
const uint32_t SIMD_NVALS[SIMD_NUM_FMTS] = { 1, 1, 2, 4 };
// number of bits per value
const uint32_t SIMD_NBITS[SIMD_NUM_FMTS] = { 64, 32, 16, 8 };
// log2(bits per value)
const uint32_t SIMD_LOG2N[SIMD_NUM_FMTS] = { 6, 5, 4, 3 };


// DSP maximum values
const uint64_t FIXED_L_SMAX = ULL(0x7fffffffffffffff);
const uint64_t FIXED_W_SMAX = ULL(0x000000007fffffff);
const uint64_t FIXED_H_SMAX = ULL(0x0000000000007fff);
const uint64_t FIXED_B_SMAX = ULL(0x000000000000007f);
const uint64_t FIXED_L_UMAX = ULL(0xffffffffffffffff);
const uint64_t FIXED_W_UMAX = ULL(0x00000000ffffffff);
const uint64_t FIXED_H_UMAX = ULL(0x000000000000ffff);
const uint64_t FIXED_B_UMAX = ULL(0x00000000000000ff);
const uint64_t FIXED_SMAX[SIMD_NUM_FMTS] =
{ FIXED_L_SMAX, FIXED_W_SMAX, FIXED_H_SMAX, FIXED_B_SMAX };
const uint64_t FIXED_UMAX[SIMD_NUM_FMTS] =
{ FIXED_L_UMAX, FIXED_W_UMAX, FIXED_H_UMAX, FIXED_B_UMAX };

// DSP minimum values
const uint64_t FIXED_L_SMIN = ULL(0x8000000000000000);
const uint64_t FIXED_W_SMIN = ULL(0xffffffff80000000);
const uint64_t FIXED_H_SMIN = ULL(0xffffffffffff8000);
const uint64_t FIXED_B_SMIN = ULL(0xffffffffffffff80);
const uint64_t FIXED_L_UMIN = ULL(0x0000000000000000);
const uint64_t FIXED_W_UMIN = ULL(0x0000000000000000);
const uint64_t FIXED_H_UMIN = ULL(0x0000000000000000);
const uint64_t FIXED_B_UMIN = ULL(0x0000000000000000);
const uint64_t FIXED_SMIN[SIMD_NUM_FMTS] =
{ FIXED_L_SMIN, FIXED_W_SMIN, FIXED_H_SMIN, FIXED_B_SMIN };
const uint64_t FIXED_UMIN[SIMD_NUM_FMTS] =
{ FIXED_L_UMIN, FIXED_W_UMIN, FIXED_H_UMIN, FIXED_B_UMIN };

// DSP utility functions
int32_t bitrev(int32_t value);
uint64_t dspSaturate(uint64_t value, int32_t fmt, int32_t sign,
                     uint32_t *overflow);
uint64_t checkOverflow(uint64_t value, int32_t fmt, int32_t sign,
                       uint32_t *overflow);
uint64_t signExtend(uint64_t value, int32_t signpos);
uint64_t addHalfLsb(uint64_t value, int32_t lsbpos);
int32_t dspAbs(int32_t a, int32_t fmt, uint32_t *dspctl);
int32_t dspAdd(int32_t a, int32_t b, int32_t fmt, int32_t saturate,
               int32_t sign, uint32_t *dspctl);
int32_t dspAddh(int32_t a, int32_t b, int32_t fmt, int32_t round,
                int32_t sign);
int32_t dspSub(int32_t a, int32_t b, int32_t fmt, int32_t saturate,
               int32_t sign, uint32_t *dspctl);
int32_t dspSubh(int32_t a, int32_t b, int32_t fmt, int32_t round,
                int32_t sign);
int32_t dspShll(int32_t a, uint32_t sa, int32_t fmt, int32_t saturate,
                int32_t sign, uint32_t *dspctl);
int32_t dspShrl(int32_t a, uint32_t sa, int32_t fmt, int32_t sign);
int32_t dspShra(int32_t a, uint32_t sa, int32_t fmt, int32_t round,
                int32_t sign, uint32_t *dspctl);
int32_t dspMul(int32_t a, int32_t b, int32_t fmt, int32_t saturate,
               uint32_t *dspctl);
int32_t dspMulq(int32_t a, int32_t b, int32_t fmt, int32_t saturate,
                int32_t round, uint32_t *dspctl);
int32_t dspMuleu(int32_t a, int32_t b, int32_t mode, uint32_t *dspctl);
int32_t dspMuleq(int32_t a, int32_t b, int32_t mode, uint32_t *dspctl);
int64_t dspDpaq(int64_t dspac, int32_t a, int32_t b, int32_t ac,
                int32_t infmt, int32_t outfmt, int32_t postsat, int32_t mode,
                uint32_t *dspctl);
int64_t dspDpsq(int64_t dspac, int32_t a, int32_t b, int32_t ac,
                int32_t infmt, int32_t outfmt, int32_t postsat, int32_t mode,
                uint32_t *dspctl);
int64_t dspDpa(int64_t dspac, int32_t a, int32_t b, int32_t ac, int32_t fmt,
               int32_t sign, int32_t mode);
int64_t dspDps(int64_t dspac, int32_t a, int32_t b, int32_t ac, int32_t fmt,
               int32_t sign, int32_t mode);
int64_t dspMaq(int64_t dspac, int32_t a, int32_t b, int32_t ac,
               int32_t fmt, int32_t mode, int32_t saturate, uint32_t *dspctl);
int64_t dspMulsa(int64_t dspac, int32_t a, int32_t b, int32_t ac, int32_t fmt);
int64_t dspMulsaq(int64_t dspac, int32_t a, int32_t b, int32_t ac, int32_t fmt,
                  uint32_t *dspctl);
void dspCmp(int32_t a, int32_t b, int32_t fmt, int32_t sign, int32_t op,
            uint32_t *dspctl);
int32_t dspCmpg(int32_t a, int32_t b, int32_t fmt, int32_t sign, int32_t op);
int32_t dspCmpgd(int32_t a, int32_t b, int32_t fmt, int32_t sign, int32_t op,
                 uint32_t *dspctl);
int32_t dspPrece(int32_t a, int32_t infmt, int32_t insign, int32_t outfmt,
                 int32_t outsign, int32_t mode);
int32_t dspPrecrqu(int32_t a, int32_t b, uint32_t *dspctl);
int32_t dspPrecrq(int32_t a, int32_t b, int32_t fmt, uint32_t *dspctl);
int32_t dspPrecrSra(int32_t a, int32_t b, int32_t sa, int32_t fmt,
                    int32_t round);
int32_t dspPick(int32_t a, int32_t b, int32_t fmt, uint32_t *dspctl);
int32_t dspPack(int32_t a, int32_t b, int32_t fmt);
int32_t dspExtr(int64_t dspac, int32_t fmt, int32_t sa, int32_t round,
                int32_t saturate, uint32_t *dspctl);
int32_t dspExtp(int64_t dspac, int32_t size, uint32_t *dspctl);
int32_t dspExtpd(int64_t dspac, int32_t size, uint32_t *dspctl);

// SIMD pack/unpack utility functions
void simdPack(uint64_t *values_ptr, int32_t *reg, int32_t fmt);
void simdUnpack(int32_t reg, uint64_t *values_ptr, int32_t fmt, int32_t sign);

// DSPControl r/w utility functions
void writeDSPControl(uint32_t *dspctl, uint32_t value, uint32_t mask);
uint32_t readDSPControl(uint32_t *dspctl, uint32_t mask);

} // namespace MipsISA

#endif // __ARCH_MIPS_DSP_HH__
