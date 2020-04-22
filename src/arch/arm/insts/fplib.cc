/*
* Copyright (c) 2012-2013, 2017-2018, 2020 ARM Limited
* Copyright (c) 2020 Metempsy Technology Consulting
* All rights reserved
*
* The license below extends only to copyright in the software and shall
* not be construed as granting a license to any other intellectual
* property including but not limited to intellectual property relating
* to a hardware implementation of the functionality of the software
* licensed hereunder.  You may use the software subject to the license
* terms below provided that you ensure that this notice is replicated
* unmodified and in its entirety in all distributions of the software,
* modified or unmodified, in source code or in binary form.
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

#include <stdint.h>

#include <cassert>
#include <cmath>

#include "base/logging.hh"
#include "fplib.hh"

namespace ArmISA
{

#define FPLIB_RN 0
#define FPLIB_RP 1
#define FPLIB_RM 2
#define FPLIB_RZ 3
#define FPLIB_FZ 4
#define FPLIB_DN 8
#define FPLIB_AHP 16
#define FPLIB_FZ16 32

#define FPLIB_IDC 128 // Input Denormal
#define FPLIB_IXC 16  // Inexact
#define FPLIB_UFC 8   // Underflow
#define FPLIB_OFC 4   // Overflow
#define FPLIB_DZC 2   // Division by Zero
#define FPLIB_IOC 1   // Invalid Operation

#define FP16_BITS 16
#define FP32_BITS 32
#define FP64_BITS 64

#define FP16_EXP_BITS 5
#define FP32_EXP_BITS 8
#define FP64_EXP_BITS 11

#define FP16_EXP_BIAS 15
#define FP32_EXP_BIAS 127
#define FP64_EXP_BIAS 1023

#define FP16_EXP_INF ((1ULL << FP16_EXP_BITS) - 1)
#define FP32_EXP_INF ((1ULL << FP32_EXP_BITS) - 1)
#define FP64_EXP_INF ((1ULL << FP64_EXP_BITS) - 1)

#define FP16_MANT_BITS (FP16_BITS - FP16_EXP_BITS - 1)
#define FP32_MANT_BITS (FP32_BITS - FP32_EXP_BITS - 1)
#define FP64_MANT_BITS (FP64_BITS - FP64_EXP_BITS - 1)

#define FP16_EXP(x) ((x) >> FP16_MANT_BITS & ((1ULL << FP16_EXP_BITS) - 1))
#define FP32_EXP(x) ((x) >> FP32_MANT_BITS & ((1ULL << FP32_EXP_BITS) - 1))
#define FP64_EXP(x) ((x) >> FP64_MANT_BITS & ((1ULL << FP64_EXP_BITS) - 1))

#define FP16_MANT(x) ((x) & ((1ULL << FP16_MANT_BITS) - 1))
#define FP32_MANT(x) ((x) & ((1ULL << FP32_MANT_BITS) - 1))
#define FP64_MANT(x) ((x) & ((1ULL << FP64_MANT_BITS) - 1))

static inline uint16_t
lsl16(uint16_t x, uint32_t shift)
{
    return shift < 16 ? x << shift : 0;
}

static inline uint16_t
lsr16(uint16_t x, uint32_t shift)
{
    return shift < 16 ? x >> shift : 0;
}

static inline uint32_t
lsl32(uint32_t x, uint32_t shift)
{
    return shift < 32 ? x << shift : 0;
}

static inline uint32_t
lsr32(uint32_t x, uint32_t shift)
{
    return shift < 32 ? x >> shift : 0;
}

static inline uint64_t
lsl64(uint64_t x, uint32_t shift)
{
    return shift < 64 ? x << shift : 0;
}

static inline uint64_t
lsr64(uint64_t x, uint32_t shift)
{
    return shift < 64 ? x >> shift : 0;
}

static inline void
lsl128(uint64_t *r0, uint64_t *r1, uint64_t x0, uint64_t x1, uint32_t shift)
{
    if (shift == 0) {
        *r1 = x1;
        *r0 = x0;
    } else if (shift < 64) {
        *r1 = x1 << shift | x0 >> (64 - shift);
        *r0 = x0 << shift;
    } else if (shift < 128) {
        *r1 = x0 << (shift - 64);
        *r0 = 0;
    } else {
        *r1 = 0;
        *r0 = 0;
    }
}

static inline void
lsr128(uint64_t *r0, uint64_t *r1, uint64_t x0, uint64_t x1, uint32_t shift)
{
    if (shift == 0) {
        *r1 = x1;
        *r0 = x0;
    } else if (shift < 64) {
        *r0 = x0 >> shift | x1 << (64 - shift);
        *r1 = x1 >> shift;
    } else if (shift < 128) {
        *r0 = x1 >> (shift - 64);
        *r1 = 0;
    } else {
        *r0 = 0;
        *r1 = 0;
    }
}

static inline void
mul62x62(uint64_t *x0, uint64_t *x1, uint64_t a, uint64_t b)
{
    uint32_t mask = ((uint32_t)1 << 31) - 1;
    uint64_t a0 = a & mask;
    uint64_t a1 = a >> 31 & mask;
    uint64_t b0 = b & mask;
    uint64_t b1 = b >> 31 & mask;
    uint64_t p0 = a0 * b0;
    uint64_t p2 = a1 * b1;
    uint64_t p1 = (a0 + a1) * (b0 + b1) - p0 - p2;
    uint64_t s0 = p0;
    uint64_t s1 = (s0 >> 31) + p1;
    uint64_t s2 = (s1 >> 31) + p2;
    *x0 = (s0 & mask) | (s1 & mask) << 31 | s2 << 62;
    *x1 = s2 >> 2;
}

static inline
void mul64x32(uint64_t *x0, uint64_t *x1, uint64_t a, uint32_t b)
{
    uint64_t t0 = (uint64_t)(uint32_t)a * b;
    uint64_t t1 = (t0 >> 32) + (a >> 32) * b;
    *x0 = t1 << 32 | (uint32_t)t0;
    *x1 = t1 >> 32;
}

static inline void
add128(uint64_t *x0, uint64_t *x1, uint64_t a0, uint64_t a1, uint64_t b0,
       uint64_t b1)
{
    *x0 = a0 + b0;
    *x1 = a1 + b1 + (*x0 < a0);
}

static inline void
sub128(uint64_t *x0, uint64_t *x1, uint64_t a0, uint64_t a1, uint64_t b0,
       uint64_t b1)
{
    *x0 = a0 - b0;
    *x1 = a1 - b1 - (*x0 > a0);
}

static inline int
cmp128(uint64_t a0, uint64_t a1, uint64_t b0, uint64_t b1)
{
    return (a1 < b1 ? -1 : a1 > b1 ? 1 : a0 < b0 ? -1 : a0 > b0 ? 1 : 0);
}

static inline uint16_t
fp16_normalise(uint16_t mnt, int *exp)
{
    int shift;

    if (!mnt) {
        return 0;
    }

    for (shift = 8; shift; shift >>= 1) {
        if (!(mnt >> (16 - shift))) {
            mnt <<= shift;
            *exp -= shift;
        }
    }
    return mnt;
}

static inline uint32_t
fp32_normalise(uint32_t mnt, int *exp)
{
    int shift;

    if (!mnt) {
        return 0;
    }

    for (shift = 16; shift; shift >>= 1) {
        if (!(mnt >> (32 - shift))) {
            mnt <<= shift;
            *exp -= shift;
        }
    }
    return mnt;
}

static inline uint64_t
fp64_normalise(uint64_t mnt, int *exp)
{
    int shift;

    if (!mnt) {
        return 0;
    }

    for (shift = 32; shift; shift >>= 1) {
        if (!(mnt >> (64 - shift))) {
            mnt <<= shift;
            *exp -= shift;
        }
    }
    return mnt;
}

static inline void
fp128_normalise(uint64_t *mnt0, uint64_t *mnt1, int *exp)
{
    uint64_t x0 = *mnt0;
    uint64_t x1 = *mnt1;
    int shift;

    if (!x0 && !x1) {
        return;
    }

    if (!x1) {
        x1 = x0;
        x0 = 0;
        *exp -= 64;
    }

    for (shift = 32; shift; shift >>= 1) {
        if (!(x1 >> (64 - shift))) {
            x1 = x1 << shift | x0 >> (64 - shift);
            x0 <<= shift;
            *exp -= shift;
        }
    }

    *mnt0 = x0;
    *mnt1 = x1;
}

static inline uint16_t
fp16_pack(uint16_t sgn, uint16_t exp, uint16_t mnt)
{
    return sgn << (FP16_BITS - 1) | exp << FP16_MANT_BITS | FP16_MANT(mnt);
}

static inline uint32_t
fp32_pack(uint32_t sgn, uint32_t exp, uint32_t mnt)
{
    return sgn << (FP32_BITS - 1) | exp << FP32_MANT_BITS | FP32_MANT(mnt);
}

static inline uint64_t
fp64_pack(uint64_t sgn, uint64_t exp, uint64_t mnt)
{
    return sgn << (FP64_BITS - 1) | exp << FP64_MANT_BITS | FP64_MANT(mnt);
}

static inline uint16_t
fp16_zero(int sgn)
{
    return fp16_pack(sgn, 0, 0);
}

static inline uint32_t
fp32_zero(int sgn)
{
    return fp32_pack(sgn, 0, 0);
}

static inline uint64_t
fp64_zero(int sgn)
{
    return fp64_pack(sgn, 0, 0);
}

static inline uint16_t
fp16_max_normal(int sgn)
{
    return fp16_pack(sgn, FP16_EXP_INF - 1, -1);
}

static inline uint32_t
fp32_max_normal(int sgn)
{
    return fp32_pack(sgn, FP32_EXP_INF - 1, -1);
}

static inline uint64_t
fp64_max_normal(int sgn)
{
    return fp64_pack(sgn, FP64_EXP_INF - 1, -1);
}

static inline uint16_t
fp16_infinity(int sgn)
{
    return fp16_pack(sgn, FP16_EXP_INF, 0);
}

static inline uint32_t
fp32_infinity(int sgn)
{
    return fp32_pack(sgn, FP32_EXP_INF, 0);
}

static inline uint64_t
fp64_infinity(int sgn)
{
    return fp64_pack(sgn, FP64_EXP_INF, 0);
}

static inline uint16_t
fp16_defaultNaN()
{
    return fp16_pack(0, FP16_EXP_INF, 1ULL << (FP16_MANT_BITS - 1));
}

static inline uint32_t
fp32_defaultNaN()
{
    return fp32_pack(0, FP32_EXP_INF, 1ULL << (FP32_MANT_BITS - 1));
}

static inline uint64_t
fp64_defaultNaN()
{
    return fp64_pack(0, FP64_EXP_INF, 1ULL << (FP64_MANT_BITS - 1));
}

static inline void
fp16_unpack(int *sgn, int *exp, uint16_t *mnt, uint16_t x, int mode,
            int *flags)
{
    *sgn = x >> (FP16_BITS - 1);
    *exp = FP16_EXP(x);
    *mnt = FP16_MANT(x);

    if (*exp) {
        *mnt |= 1ULL << FP16_MANT_BITS;
    } else {
        // Handle subnormals:
        // IDC (Input Denormal) is not set in this case.
        if (*mnt) {
            if (mode & FPLIB_FZ16) {
                *mnt = 0;
            } else {
                ++*exp;
            }
        }
    }
}

static inline void
fp32_unpack(int *sgn, int *exp, uint32_t *mnt, uint32_t x, int mode,
            int *flags)
{
    *sgn = x >> (FP32_BITS - 1);
    *exp = FP32_EXP(x);
    *mnt = FP32_MANT(x);

    if (*exp) {
        *mnt |= 1ULL << FP32_MANT_BITS;
    } else {
        // Handle subnormals:
        if (*mnt) {
            if (mode & FPLIB_FZ) {
                *flags |= FPLIB_IDC;
                *mnt = 0;
            } else {
                ++*exp;
            }
        }
    }
}

static inline void
fp64_unpack(int *sgn, int *exp, uint64_t *mnt, uint64_t x, int mode,
            int *flags)
{


    *sgn = x >> (FP64_BITS - 1);
    *exp = FP64_EXP(x);
    *mnt = FP64_MANT(x);

    if (*exp) {
        *mnt |= 1ULL << FP64_MANT_BITS;
    } else {
        // Handle subnormals:
        if (*mnt) {
            if (mode & FPLIB_FZ) {
                *flags |= FPLIB_IDC;
                *mnt = 0;
            } else {
                ++*exp;
            }
        }
    }
}

static inline int
fp16_is_NaN(int exp, uint16_t mnt)
{
    return exp == FP16_EXP_INF && FP16_MANT(mnt);
}

static inline int
fp32_is_NaN(int exp, uint32_t mnt)
{
    return exp == FP32_EXP_INF && FP32_MANT(mnt);
}

static inline int
fp64_is_NaN(int exp, uint64_t mnt)
{
    return exp == FP64_EXP_INF && FP64_MANT(mnt);
}

static inline int
fp16_is_signalling_NaN(int exp, uint16_t mnt)
{
    return fp16_is_NaN(exp, mnt) && !(mnt >> (FP16_MANT_BITS - 1) & 1);
}

static inline int
fp32_is_signalling_NaN(int exp, uint32_t mnt)
{
    return fp32_is_NaN(exp, mnt) && !(mnt >> (FP32_MANT_BITS - 1) & 1);
}

static inline int
fp64_is_signalling_NaN(int exp, uint64_t mnt)
{
    return fp64_is_NaN(exp, mnt) && !(mnt >> (FP64_MANT_BITS - 1) & 1);
}

static inline int
fp16_is_quiet_NaN(int exp, uint16_t mnt)
{
    return exp == FP16_EXP_INF && (mnt >> (FP16_MANT_BITS - 1) & 1);
}

static inline int
fp32_is_quiet_NaN(int exp, uint32_t mnt)
{
    return exp == FP32_EXP_INF && (mnt >> (FP32_MANT_BITS - 1) & 1);
}

static inline int
fp64_is_quiet_NaN(int exp, uint64_t mnt)
{
    return exp == FP64_EXP_INF && (mnt >> (FP64_MANT_BITS - 1) & 1);
}

static inline int
fp16_is_infinity(int exp, uint16_t mnt)
{
    return exp == FP16_EXP_INF && !FP16_MANT(mnt);
}

static inline int
fp32_is_infinity(int exp, uint32_t mnt)
{
    return exp == FP32_EXP_INF && !FP32_MANT(mnt);
}

static inline int
fp64_is_infinity(int exp, uint64_t mnt)
{
    return exp == FP64_EXP_INF && !FP64_MANT(mnt);
}

static inline uint16_t
fp16_process_NaN(uint16_t a, int mode, int *flags)
{
    if (!(a >> (FP16_MANT_BITS - 1) & 1)) {
        *flags |= FPLIB_IOC;
        a |= 1ULL << (FP16_MANT_BITS - 1);
    }
    return mode & FPLIB_DN ? fp16_defaultNaN() : a;
}

static inline uint32_t
fp32_process_NaN(uint32_t a, int mode, int *flags)
{
    if (!(a >> (FP32_MANT_BITS - 1) & 1)) {
        *flags |= FPLIB_IOC;
        a |= 1ULL << (FP32_MANT_BITS - 1);
    }
    return mode & FPLIB_DN ? fp32_defaultNaN() : a;
}

static inline uint64_t
fp64_process_NaN(uint64_t a, int mode, int *flags)
{
    if (!(a >> (FP64_MANT_BITS - 1) & 1)) {
        *flags |= FPLIB_IOC;
        a |= 1ULL << (FP64_MANT_BITS - 1);
    }
    return mode & FPLIB_DN ? fp64_defaultNaN() : a;
}

static uint16_t
fp16_process_NaNs(uint16_t a, uint16_t b, int mode, int *flags)
{
    int a_exp = FP16_EXP(a);
    uint16_t a_mnt = FP16_MANT(a);
    int b_exp = FP16_EXP(b);
    uint16_t b_mnt = FP16_MANT(b);

    // Handle signalling NaNs:
    if (fp16_is_signalling_NaN(a_exp, a_mnt))
        return fp16_process_NaN(a, mode, flags);
    if (fp16_is_signalling_NaN(b_exp, b_mnt))
        return fp16_process_NaN(b, mode, flags);

    // Handle quiet NaNs:
    if (fp16_is_NaN(a_exp, a_mnt))
        return fp16_process_NaN(a, mode, flags);
    if (fp16_is_NaN(b_exp, b_mnt))
        return fp16_process_NaN(b, mode, flags);

    return 0;
}

static uint32_t
fp32_process_NaNs(uint32_t a, uint32_t b, int mode, int *flags)
{
    int a_exp = FP32_EXP(a);
    uint32_t a_mnt = FP32_MANT(a);
    int b_exp = FP32_EXP(b);
    uint32_t b_mnt = FP32_MANT(b);

    // Handle signalling NaNs:
    if (fp32_is_signalling_NaN(a_exp, a_mnt))
        return fp32_process_NaN(a, mode, flags);
    if (fp32_is_signalling_NaN(b_exp, b_mnt))
        return fp32_process_NaN(b, mode, flags);

    // Handle quiet NaNs:
    if (fp32_is_NaN(a_exp, a_mnt))
        return fp32_process_NaN(a, mode, flags);
    if (fp32_is_NaN(b_exp, b_mnt))
        return fp32_process_NaN(b, mode, flags);

    return 0;
}

static uint64_t
fp64_process_NaNs(uint64_t a, uint64_t b, int mode, int *flags)
{
    int a_exp = FP64_EXP(a);
    uint64_t a_mnt = FP64_MANT(a);
    int b_exp = FP64_EXP(b);
    uint64_t b_mnt = FP64_MANT(b);

    // Handle signalling NaNs:
    if (fp64_is_signalling_NaN(a_exp, a_mnt))
        return fp64_process_NaN(a, mode, flags);
    if (fp64_is_signalling_NaN(b_exp, b_mnt))
        return fp64_process_NaN(b, mode, flags);

    // Handle quiet NaNs:
    if (fp64_is_NaN(a_exp, a_mnt))
        return fp64_process_NaN(a, mode, flags);
    if (fp64_is_NaN(b_exp, b_mnt))
        return fp64_process_NaN(b, mode, flags);

    return 0;
}

static uint16_t
fp16_process_NaNs3(uint16_t a, uint16_t b, uint16_t c, int mode, int *flags)
{
    int a_exp = FP16_EXP(a);
    uint16_t a_mnt = FP16_MANT(a);
    int b_exp = FP16_EXP(b);
    uint16_t b_mnt = FP16_MANT(b);
    int c_exp = FP16_EXP(c);
    uint16_t c_mnt = FP16_MANT(c);

    // Handle signalling NaNs:
    if (fp16_is_signalling_NaN(a_exp, a_mnt))
        return fp16_process_NaN(a, mode, flags);
    if (fp16_is_signalling_NaN(b_exp, b_mnt))
        return fp16_process_NaN(b, mode, flags);
    if (fp16_is_signalling_NaN(c_exp, c_mnt))
        return fp16_process_NaN(c, mode, flags);

    // Handle quiet NaNs:
    if (fp16_is_NaN(a_exp, a_mnt))
        return fp16_process_NaN(a, mode, flags);
    if (fp16_is_NaN(b_exp, b_mnt))
        return fp16_process_NaN(b, mode, flags);
    if (fp16_is_NaN(c_exp, c_mnt))
        return fp16_process_NaN(c, mode, flags);

    return 0;
}

static uint32_t
fp32_process_NaNs3(uint32_t a, uint32_t b, uint32_t c, int mode, int *flags)
{
    int a_exp = FP32_EXP(a);
    uint32_t a_mnt = FP32_MANT(a);
    int b_exp = FP32_EXP(b);
    uint32_t b_mnt = FP32_MANT(b);
    int c_exp = FP32_EXP(c);
    uint32_t c_mnt = FP32_MANT(c);

    // Handle signalling NaNs:
    if (fp32_is_signalling_NaN(a_exp, a_mnt))
        return fp32_process_NaN(a, mode, flags);
    if (fp32_is_signalling_NaN(b_exp, b_mnt))
        return fp32_process_NaN(b, mode, flags);
    if (fp32_is_signalling_NaN(c_exp, c_mnt))
        return fp32_process_NaN(c, mode, flags);

    // Handle quiet NaNs:
    if (fp32_is_NaN(a_exp, a_mnt))
        return fp32_process_NaN(a, mode, flags);
    if (fp32_is_NaN(b_exp, b_mnt))
        return fp32_process_NaN(b, mode, flags);
    if (fp32_is_NaN(c_exp, c_mnt))
        return fp32_process_NaN(c, mode, flags);

    return 0;
}

static uint64_t
fp64_process_NaNs3(uint64_t a, uint64_t b, uint64_t c, int mode, int *flags)
{
    int a_exp = FP64_EXP(a);
    uint64_t a_mnt = FP64_MANT(a);
    int b_exp = FP64_EXP(b);
    uint64_t b_mnt = FP64_MANT(b);
    int c_exp = FP64_EXP(c);
    uint64_t c_mnt = FP64_MANT(c);

    // Handle signalling NaNs:
    if (fp64_is_signalling_NaN(a_exp, a_mnt))
        return fp64_process_NaN(a, mode, flags);
    if (fp64_is_signalling_NaN(b_exp, b_mnt))
        return fp64_process_NaN(b, mode, flags);
    if (fp64_is_signalling_NaN(c_exp, c_mnt))
        return fp64_process_NaN(c, mode, flags);

    // Handle quiet NaNs:
    if (fp64_is_NaN(a_exp, a_mnt))
        return fp64_process_NaN(a, mode, flags);
    if (fp64_is_NaN(b_exp, b_mnt))
        return fp64_process_NaN(b, mode, flags);
    if (fp64_is_NaN(c_exp, c_mnt))
        return fp64_process_NaN(c, mode, flags);

    return 0;
}

static uint16_t
fp16_round_(int sgn, int exp, uint16_t mnt, int rm, int mode, int *flags)
{
    int biased_exp; // non-negative exponent value for result
    uint16_t int_mant; // mantissa for result, less than (2 << FP16_MANT_BITS)
    int error; // 0, 1, 2 or 3, where 2 means int_mant is wrong by exactly 0.5

    assert(rm != FPRounding_TIEAWAY);

    // Flush to zero:
    if ((mode & FPLIB_FZ16) && exp < 1) {
        *flags |= FPLIB_UFC;
        return fp16_zero(sgn);
    }

    // The bottom FP16_EXP_BITS bits of mnt are orred together:
    mnt = (4ULL << FP16_MANT_BITS | mnt >> (FP16_EXP_BITS - 1) |
           ((mnt & ((1ULL << FP16_EXP_BITS) - 1)) != 0));

    if (exp > 0) {
        biased_exp = exp;
        int_mant = mnt >> 2;
        error = mnt & 3;
    } else {
        biased_exp = 0;
        int_mant = lsr16(mnt, 3 - exp);
        error = (lsr16(mnt, 1 - exp) & 3) | !!(mnt & (lsl16(1, 1 - exp) - 1));
    }

    if (!biased_exp && error) { // xx should also check fpscr_val<11>
        *flags |= FPLIB_UFC;
    }

    // Round up:
    if ((rm == FPLIB_RN && (error == 3 ||
                            (error == 2 && (int_mant & 1)))) ||
        (((rm == FPLIB_RP && !sgn) || (rm == FPLIB_RM && sgn)) && error)) {
        ++int_mant;
        if (int_mant == 1ULL << FP16_MANT_BITS) {
            // Rounded up from denormalized to normalized
            biased_exp = 1;
        }
        if (int_mant == 2ULL << FP16_MANT_BITS) {
            // Rounded up to next exponent
            ++biased_exp;
            int_mant >>= 1;
        }
    }

    // Handle rounding to odd aka Von Neumann rounding:
    if (error && rm == FPRounding_ODD)
        int_mant |= 1;

    // Handle overflow:
    if (!(mode & FPLIB_AHP)) {
        if (biased_exp >= (int)FP16_EXP_INF) {
            *flags |= FPLIB_OFC | FPLIB_IXC;
            if (rm == FPLIB_RN || (rm == FPLIB_RP && !sgn) ||
                (rm == FPLIB_RM && sgn)) {
                return fp16_infinity(sgn);
            } else {
                return fp16_max_normal(sgn);
            }
        }
    } else {
        if (biased_exp >= (int)FP16_EXP_INF + 1) {
            *flags |= FPLIB_IOC;
            return fp16_pack(sgn, FP16_EXP_INF, -1);
        }
    }

    if (error) {
        *flags |= FPLIB_IXC;
    }

    return fp16_pack(sgn, biased_exp, int_mant);
}

static uint16_t
fp16_round(int sgn, int exp, uint16_t mnt, int mode, int *flags)
{
    return fp16_round_(sgn, exp, mnt, mode & 3, mode, flags);
}

static uint32_t
fp32_round_(int sgn, int exp, uint32_t mnt, int rm, int mode, int *flags)
{
    int biased_exp; // non-negative exponent value for result
    uint32_t int_mant; // mantissa for result, less than (2 << FP32_MANT_BITS)
    int error; // 0, 1, 2 or 3, where 2 means int_mant is wrong by exactly 0.5

    assert(rm != FPRounding_TIEAWAY);

    // Flush to zero:
    if ((mode & FPLIB_FZ) && exp < 1) {
        *flags |= FPLIB_UFC;
        return fp32_zero(sgn);
    }

    // The bottom FP32_EXP_BITS bits of mnt are orred together:
    mnt = (4ULL << FP32_MANT_BITS | mnt >> (FP32_EXP_BITS - 1) |
           ((mnt & ((1ULL << FP32_EXP_BITS) - 1)) != 0));

    if (exp > 0) {
        biased_exp = exp;
        int_mant = mnt >> 2;
        error = mnt & 3;
    } else {
        biased_exp = 0;
        int_mant = lsr32(mnt, 3 - exp);
        error = (lsr32(mnt, 1 - exp) & 3) | !!(mnt & (lsl32(1, 1 - exp) - 1));
    }

    if (!biased_exp && error) { // xx should also check fpscr_val<11>
        *flags |= FPLIB_UFC;
    }

    // Round up:
    if ((rm == FPLIB_RN && (error == 3 ||
                            (error == 2 && (int_mant & 1)))) ||
        (((rm == FPLIB_RP && !sgn) || (rm == FPLIB_RM && sgn)) && error)) {
        ++int_mant;
        if (int_mant == 1ULL << FP32_MANT_BITS) {
            // Rounded up from denormalized to normalized
            biased_exp = 1;
        }
        if (int_mant == 2ULL << FP32_MANT_BITS) {
            // Rounded up to next exponent
            ++biased_exp;
            int_mant >>= 1;
        }
    }

    // Handle rounding to odd aka Von Neumann rounding:
    if (error && rm == FPRounding_ODD)
        int_mant |= 1;

    // Handle overflow:
    if (biased_exp >= (int)FP32_EXP_INF) {
        *flags |= FPLIB_OFC | FPLIB_IXC;
        if (rm == FPLIB_RN || (rm == FPLIB_RP && !sgn) ||
            (rm == FPLIB_RM && sgn)) {
            return fp32_infinity(sgn);
        } else {
            return fp32_max_normal(sgn);
        }
    }

    if (error) {
        *flags |= FPLIB_IXC;
    }

    return fp32_pack(sgn, biased_exp, int_mant);
}

static uint32_t
fp32_round(int sgn, int exp, uint32_t mnt, int mode, int *flags)
{
    return fp32_round_(sgn, exp, mnt, mode & 3, mode, flags);
}

static uint64_t
fp64_round_(int sgn, int exp, uint64_t mnt, int rm, int mode, int *flags)
{
    int biased_exp; // non-negative exponent value for result
    uint64_t int_mant; // mantissa for result, less than (2 << FP64_MANT_BITS)
    int error; // 0, 1, 2 or 3, where 2 means int_mant is wrong by exactly 0.5

    assert(rm != FPRounding_TIEAWAY);

    // Flush to zero:
    if ((mode & FPLIB_FZ) && exp < 1) {
        *flags |= FPLIB_UFC;
        return fp64_zero(sgn);
    }

    // The bottom FP64_EXP_BITS bits of mnt are orred together:
    mnt = (4ULL << FP64_MANT_BITS | mnt >> (FP64_EXP_BITS - 1) |
           ((mnt & ((1ULL << FP64_EXP_BITS) - 1)) != 0));

    if (exp > 0) {
        biased_exp = exp;
        int_mant = mnt >> 2;
        error = mnt & 3;
    } else {
        biased_exp = 0;
        int_mant = lsr64(mnt, 3 - exp);
        error = (lsr64(mnt, 1 - exp) & 3) | !!(mnt & (lsl64(1, 1 - exp) - 1));
    }

    if (!biased_exp && error) { // xx should also check fpscr_val<11>
        *flags |= FPLIB_UFC;
    }

    // Round up:
    if ((rm == FPLIB_RN && (error == 3 ||
                            (error == 2 && (int_mant & 1)))) ||
        (((rm == FPLIB_RP && !sgn) || (rm == FPLIB_RM && sgn)) && error)) {
        ++int_mant;
        if (int_mant == 1ULL << FP64_MANT_BITS) {
            // Rounded up from denormalized to normalized
            biased_exp = 1;
        }
        if (int_mant == 2ULL << FP64_MANT_BITS) {
            // Rounded up to next exponent
            ++biased_exp;
            int_mant >>= 1;
        }
    }

    // Handle rounding to odd aka Von Neumann rounding:
    if (error && rm == FPRounding_ODD)
        int_mant |= 1;

    // Handle overflow:
    if (biased_exp >= (int)FP64_EXP_INF) {
        *flags |= FPLIB_OFC | FPLIB_IXC;
        if (rm == FPLIB_RN || (rm == FPLIB_RP && !sgn) ||
            (rm == FPLIB_RM && sgn)) {
            return fp64_infinity(sgn);
        } else {
            return fp64_max_normal(sgn);
        }
    }

    if (error) {
        *flags |= FPLIB_IXC;
    }

    return fp64_pack(sgn, biased_exp, int_mant);
}

static uint64_t
fp64_round(int sgn, int exp, uint64_t mnt, int mode, int *flags)
{
    return fp64_round_(sgn, exp, mnt, mode & 3, mode, flags);
}

static int
fp16_compare_eq(uint16_t a, uint16_t b, int mode, int *flags)
{
    int a_sgn, a_exp, b_sgn, b_exp;
    uint16_t a_mnt, b_mnt;

    fp16_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);
    fp16_unpack(&b_sgn, &b_exp, &b_mnt, b, mode, flags);

    if (fp16_is_NaN(a_exp, a_mnt) ||
        fp16_is_NaN(b_exp, b_mnt)) {
        if (fp16_is_signalling_NaN(a_exp, a_mnt) ||
            fp16_is_signalling_NaN(b_exp, b_mnt))
            *flags |= FPLIB_IOC;
        return 0;
    }
    return a == b || (!a_mnt && !b_mnt);
}

static int
fp16_compare_ge(uint16_t a, uint16_t b, int mode, int *flags)
{
    int a_sgn, a_exp, b_sgn, b_exp;
    uint16_t a_mnt, b_mnt;

    fp16_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);
    fp16_unpack(&b_sgn, &b_exp, &b_mnt, b, mode, flags);

    if (fp16_is_NaN(a_exp, a_mnt) ||
        fp16_is_NaN(b_exp, b_mnt)) {
        *flags |= FPLIB_IOC;
        return 0;
    }
    if (!a_mnt && !b_mnt)
        return 1;
    if (a_sgn != b_sgn)
        return b_sgn;
    if (a_exp != b_exp)
        return a_sgn ^ (a_exp > b_exp);
    if (a_mnt != b_mnt)
        return a_sgn ^ (a_mnt > b_mnt);
    return 1;
}

static int
fp16_compare_gt(uint16_t a, uint16_t b, int mode, int *flags)
{
    int a_sgn, a_exp, b_sgn, b_exp;
    uint16_t a_mnt, b_mnt;

    fp16_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);
    fp16_unpack(&b_sgn, &b_exp, &b_mnt, b, mode, flags);

    if (fp16_is_NaN(a_exp, a_mnt) ||
        fp16_is_NaN(b_exp, b_mnt)) {
        *flags |= FPLIB_IOC;
        return 0;
    }
    if (!a_mnt && !b_mnt)
        return 0;
    if (a_sgn != b_sgn)
        return b_sgn;
    if (a_exp != b_exp)
        return a_sgn ^ (a_exp > b_exp);
    if (a_mnt != b_mnt)
        return a_sgn ^ (a_mnt > b_mnt);
    return 0;
}

static int
fp16_compare_un(uint16_t a, uint16_t b, int mode, int *flags)
{
    int a_sgn, a_exp, b_sgn, b_exp;
    uint16_t a_mnt, b_mnt;

    fp16_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);
    fp16_unpack(&b_sgn, &b_exp, &b_mnt, b, mode, flags);

    if (fp16_is_NaN(a_exp, a_mnt) ||
        fp16_is_NaN(b_exp, b_mnt)) {
        if (fp16_is_signalling_NaN(a_exp, a_mnt) ||
            fp16_is_signalling_NaN(b_exp, b_mnt))
            *flags |= FPLIB_IOC;
        return 1;
    }
    return 0;
}

static int
fp32_compare_eq(uint32_t a, uint32_t b, int mode, int *flags)
{
    int a_sgn, a_exp, b_sgn, b_exp;
    uint32_t a_mnt, b_mnt;

    fp32_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);
    fp32_unpack(&b_sgn, &b_exp, &b_mnt, b, mode, flags);

    if (fp32_is_NaN(a_exp, a_mnt) ||
        fp32_is_NaN(b_exp, b_mnt)) {
        if (fp32_is_signalling_NaN(a_exp, a_mnt) ||
            fp32_is_signalling_NaN(b_exp, b_mnt))
            *flags |= FPLIB_IOC;
        return 0;
    }
    return a == b || (!a_mnt && !b_mnt);
}

static int
fp32_compare_ge(uint32_t a, uint32_t b, int mode, int *flags)
{
    int a_sgn, a_exp, b_sgn, b_exp;
    uint32_t a_mnt, b_mnt;

    fp32_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);
    fp32_unpack(&b_sgn, &b_exp, &b_mnt, b, mode, flags);

    if (fp32_is_NaN(a_exp, a_mnt) ||
        fp32_is_NaN(b_exp, b_mnt)) {
        *flags |= FPLIB_IOC;
        return 0;
    }
    if (!a_mnt && !b_mnt)
        return 1;
    if (a_sgn != b_sgn)
        return b_sgn;
    if (a_exp != b_exp)
        return a_sgn ^ (a_exp > b_exp);
    if (a_mnt != b_mnt)
        return a_sgn ^ (a_mnt > b_mnt);
    return 1;
}

static int
fp32_compare_gt(uint32_t a, uint32_t b, int mode, int *flags)
{
    int a_sgn, a_exp, b_sgn, b_exp;
    uint32_t a_mnt, b_mnt;

    fp32_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);
    fp32_unpack(&b_sgn, &b_exp, &b_mnt, b, mode, flags);

    if (fp32_is_NaN(a_exp, a_mnt) ||
        fp32_is_NaN(b_exp, b_mnt)) {
        *flags |= FPLIB_IOC;
        return 0;
    }
    if (!a_mnt && !b_mnt)
        return 0;
    if (a_sgn != b_sgn)
        return b_sgn;
    if (a_exp != b_exp)
        return a_sgn ^ (a_exp > b_exp);
    if (a_mnt != b_mnt)
        return a_sgn ^ (a_mnt > b_mnt);
    return 0;
}

static int
fp32_compare_un(uint32_t a, uint32_t b, int mode, int *flags)
{
    int a_sgn, a_exp, b_sgn, b_exp;
    uint32_t a_mnt, b_mnt;

    fp32_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);
    fp32_unpack(&b_sgn, &b_exp, &b_mnt, b, mode, flags);

    if (fp32_is_NaN(a_exp, a_mnt) ||
        fp32_is_NaN(b_exp, b_mnt)) {
        if (fp32_is_signalling_NaN(a_exp, a_mnt) ||
            fp32_is_signalling_NaN(b_exp, b_mnt))
            *flags |= FPLIB_IOC;
        return 1;
    }
    return 0;
}

static int
fp64_compare_eq(uint64_t a, uint64_t b, int mode, int *flags)
{
    int a_sgn, a_exp, b_sgn, b_exp;
    uint64_t a_mnt, b_mnt;

    fp64_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);
    fp64_unpack(&b_sgn, &b_exp, &b_mnt, b, mode, flags);

    if (fp64_is_NaN(a_exp, a_mnt) ||
        fp64_is_NaN(b_exp, b_mnt)) {
        if (fp64_is_signalling_NaN(a_exp, a_mnt) ||
            fp64_is_signalling_NaN(b_exp, b_mnt))
            *flags |= FPLIB_IOC;
        return 0;
    }
    return a == b || (!a_mnt && !b_mnt);
}

static int
fp64_compare_ge(uint64_t a, uint64_t b, int mode, int *flags)
{
    int a_sgn, a_exp, b_sgn, b_exp;
    uint64_t a_mnt, b_mnt;

    fp64_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);
    fp64_unpack(&b_sgn, &b_exp, &b_mnt, b, mode, flags);

    if (fp64_is_NaN(a_exp, a_mnt) ||
        fp64_is_NaN(b_exp, b_mnt)) {
        *flags |= FPLIB_IOC;
        return 0;
    }
    if (!a_mnt && !b_mnt)
        return 1;
    if (a_sgn != b_sgn)
        return b_sgn;
    if (a_exp != b_exp)
        return a_sgn ^ (a_exp > b_exp);
    if (a_mnt != b_mnt)
        return a_sgn ^ (a_mnt > b_mnt);
    return 1;
}

static int
fp64_compare_gt(uint64_t a, uint64_t b, int mode, int *flags)
{
    int a_sgn, a_exp, b_sgn, b_exp;
    uint64_t a_mnt, b_mnt;

    fp64_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);
    fp64_unpack(&b_sgn, &b_exp, &b_mnt, b, mode, flags);

    if (fp64_is_NaN(a_exp, a_mnt) ||
        fp64_is_NaN(b_exp, b_mnt)) {
        *flags |= FPLIB_IOC;
        return 0;
    }
    if (!a_mnt && !b_mnt)
        return 0;
    if (a_sgn != b_sgn)
        return b_sgn;
    if (a_exp != b_exp)
        return a_sgn ^ (a_exp > b_exp);
    if (a_mnt != b_mnt)
        return a_sgn ^ (a_mnt > b_mnt);
    return 0;
}

static int
fp64_compare_un(uint64_t a, uint64_t b, int mode, int *flags)
{
    int a_sgn, a_exp, b_sgn, b_exp;
    uint64_t a_mnt, b_mnt;

    fp64_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);
    fp64_unpack(&b_sgn, &b_exp, &b_mnt, b, mode, flags);

    if (fp64_is_NaN(a_exp, a_mnt) ||
        fp64_is_NaN(b_exp, b_mnt)) {
        if (fp64_is_signalling_NaN(a_exp, a_mnt) ||
            fp64_is_signalling_NaN(b_exp, b_mnt))
            *flags |= FPLIB_IOC;
        return 1;
    }
    return 0;
}

static uint16_t
fp16_add(uint16_t a, uint16_t b, int neg, int mode, int *flags)
{
    int a_sgn, a_exp, b_sgn, b_exp, x_sgn, x_exp;
    uint16_t a_mnt, b_mnt, x, x_mnt;

    fp16_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);
    fp16_unpack(&b_sgn, &b_exp, &b_mnt, b, mode, flags);

    if ((x = fp16_process_NaNs(a, b, mode, flags))) {
        return x;
    }

    b_sgn ^= neg;

    // Handle infinities and zeroes:
    if (a_exp == FP16_EXP_INF && b_exp == FP16_EXP_INF && a_sgn != b_sgn) {
        *flags |= FPLIB_IOC;
        return fp16_defaultNaN();
    } else if (a_exp == FP16_EXP_INF) {
        return fp16_infinity(a_sgn);
    } else if (b_exp == FP16_EXP_INF) {
        return fp16_infinity(b_sgn);
    } else if (!a_mnt && !b_mnt && a_sgn == b_sgn) {
        return fp16_zero(a_sgn);
    }

    a_mnt <<= 3;
    b_mnt <<= 3;
    if (a_exp >= b_exp) {
        b_mnt = (lsr16(b_mnt, a_exp - b_exp) |
                 !!(b_mnt & (lsl16(1, a_exp - b_exp) - 1)));
        b_exp = a_exp;
    } else {
        a_mnt = (lsr16(a_mnt, b_exp - a_exp) |
                 !!(a_mnt & (lsl16(1, b_exp - a_exp) - 1)));
        a_exp = b_exp;
    }
    x_sgn = a_sgn;
    x_exp = a_exp;
    if (a_sgn == b_sgn) {
        x_mnt = a_mnt + b_mnt;
    } else if (a_mnt >= b_mnt) {
        x_mnt = a_mnt - b_mnt;
    } else {
        x_sgn ^= 1;
        x_mnt = b_mnt - a_mnt;
    }

    if (!x_mnt) {
        // Sign of exact zero result depends on rounding mode
        return fp16_zero((mode & 3) == 2);
    }

    x_mnt = fp16_normalise(x_mnt, &x_exp);

    return fp16_round(x_sgn, x_exp + FP16_EXP_BITS - 3, x_mnt << 1,
                      mode, flags);
}

static uint32_t
fp32_add(uint32_t a, uint32_t b, int neg, int mode, int *flags)
{
    int a_sgn, a_exp, b_sgn, b_exp, x_sgn, x_exp;
    uint32_t a_mnt, b_mnt, x, x_mnt;

    fp32_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);
    fp32_unpack(&b_sgn, &b_exp, &b_mnt, b, mode, flags);

    if ((x = fp32_process_NaNs(a, b, mode, flags))) {
        return x;
    }

    b_sgn ^= neg;

    // Handle infinities and zeroes:
    if (a_exp == FP32_EXP_INF && b_exp == FP32_EXP_INF && a_sgn != b_sgn) {
        *flags |= FPLIB_IOC;
        return fp32_defaultNaN();
    } else if (a_exp == FP32_EXP_INF) {
        return fp32_infinity(a_sgn);
    } else if (b_exp == FP32_EXP_INF) {
        return fp32_infinity(b_sgn);
    } else if (!a_mnt && !b_mnt && a_sgn == b_sgn) {
        return fp32_zero(a_sgn);
    }

    a_mnt <<= 3;
    b_mnt <<= 3;
    if (a_exp >= b_exp) {
        b_mnt = (lsr32(b_mnt, a_exp - b_exp) |
                 !!(b_mnt & (lsl32(1, a_exp - b_exp) - 1)));
        b_exp = a_exp;
    } else {
        a_mnt = (lsr32(a_mnt, b_exp - a_exp) |
                 !!(a_mnt & (lsl32(1, b_exp - a_exp) - 1)));
        a_exp = b_exp;
    }
    x_sgn = a_sgn;
    x_exp = a_exp;
    if (a_sgn == b_sgn) {
        x_mnt = a_mnt + b_mnt;
    } else if (a_mnt >= b_mnt) {
        x_mnt = a_mnt - b_mnt;
    } else {
        x_sgn ^= 1;
        x_mnt = b_mnt - a_mnt;
    }

    if (!x_mnt) {
        // Sign of exact zero result depends on rounding mode
        return fp32_zero((mode & 3) == 2);
    }

    x_mnt = fp32_normalise(x_mnt, &x_exp);

    return fp32_round(x_sgn, x_exp + FP32_EXP_BITS - 3, x_mnt << 1,
                      mode, flags);
}

static uint64_t
fp64_add(uint64_t a, uint64_t b, int neg, int mode, int *flags)
{
    int a_sgn, a_exp, b_sgn, b_exp, x_sgn, x_exp;
    uint64_t a_mnt, b_mnt, x, x_mnt;

    fp64_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);
    fp64_unpack(&b_sgn, &b_exp, &b_mnt, b, mode, flags);

    if ((x = fp64_process_NaNs(a, b, mode, flags))) {
        return x;
    }

    b_sgn ^= neg;

    // Handle infinities and zeroes:
    if (a_exp == FP64_EXP_INF && b_exp == FP64_EXP_INF && a_sgn != b_sgn) {
        *flags |= FPLIB_IOC;
        return fp64_defaultNaN();
    } else if (a_exp == FP64_EXP_INF) {
        return fp64_infinity(a_sgn);
    } else if (b_exp == FP64_EXP_INF) {
        return fp64_infinity(b_sgn);
    } else if (!a_mnt && !b_mnt && a_sgn == b_sgn) {
        return fp64_zero(a_sgn);
    }

    a_mnt <<= 3;
    b_mnt <<= 3;
    if (a_exp >= b_exp) {
        b_mnt = (lsr64(b_mnt, a_exp - b_exp) |
                 !!(b_mnt & (lsl64(1, a_exp - b_exp) - 1)));
        b_exp = a_exp;
    } else {
        a_mnt = (lsr64(a_mnt, b_exp - a_exp) |
                 !!(a_mnt & (lsl64(1, b_exp - a_exp) - 1)));
        a_exp = b_exp;
    }
    x_sgn = a_sgn;
    x_exp = a_exp;
    if (a_sgn == b_sgn) {
        x_mnt = a_mnt + b_mnt;
    } else if (a_mnt >= b_mnt) {
        x_mnt = a_mnt - b_mnt;
    } else {
        x_sgn ^= 1;
        x_mnt = b_mnt - a_mnt;
    }

    if (!x_mnt) {
        // Sign of exact zero result depends on rounding mode
        return fp64_zero((mode & 3) == 2);
    }

    x_mnt = fp64_normalise(x_mnt, &x_exp);

    return fp64_round(x_sgn, x_exp + FP64_EXP_BITS - 3, x_mnt << 1,
                      mode, flags);
}

static uint16_t
fp16_mul(uint16_t a, uint16_t b, int mode, int *flags)
{
    int a_sgn, a_exp, b_sgn, b_exp, x_sgn, x_exp;
    uint16_t a_mnt, b_mnt, x;
    uint32_t x_mnt;

    fp16_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);
    fp16_unpack(&b_sgn, &b_exp, &b_mnt, b, mode, flags);

    if ((x = fp16_process_NaNs(a, b, mode, flags))) {
        return x;
    }

    // Handle infinities and zeroes:
    if ((a_exp == FP16_EXP_INF && !b_mnt) ||
        (b_exp == FP16_EXP_INF && !a_mnt)) {
        *flags |= FPLIB_IOC;
        return fp16_defaultNaN();
    } else if (a_exp == FP16_EXP_INF || b_exp == FP16_EXP_INF) {
        return fp16_infinity(a_sgn ^ b_sgn);
    } else if (!a_mnt || !b_mnt) {
        return fp16_zero(a_sgn ^ b_sgn);
    }

    // Multiply and normalise:
    x_sgn = a_sgn ^ b_sgn;
    x_exp = a_exp + b_exp - FP16_EXP_BIAS + 2 * FP16_EXP_BITS + 1;
    x_mnt = (uint32_t)a_mnt * b_mnt;
    x_mnt = fp32_normalise(x_mnt, &x_exp);

    // Convert to FP16_BITS bits, collapsing error into bottom bit:
    x_mnt = lsr32(x_mnt, FP16_BITS - 1) | !!lsl32(x_mnt, FP16_BITS + 1);

    return fp16_round(x_sgn, x_exp, x_mnt, mode, flags);
}

static uint32_t
fp32_mul(uint32_t a, uint32_t b, int mode, int *flags)
{
    int a_sgn, a_exp, b_sgn, b_exp, x_sgn, x_exp;
    uint32_t a_mnt, b_mnt, x;
    uint64_t x_mnt;

    fp32_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);
    fp32_unpack(&b_sgn, &b_exp, &b_mnt, b, mode, flags);

    if ((x = fp32_process_NaNs(a, b, mode, flags))) {
        return x;
    }

    // Handle infinities and zeroes:
    if ((a_exp == FP32_EXP_INF && !b_mnt) ||
        (b_exp == FP32_EXP_INF && !a_mnt)) {
        *flags |= FPLIB_IOC;
        return fp32_defaultNaN();
    } else if (a_exp == FP32_EXP_INF || b_exp == FP32_EXP_INF) {
        return fp32_infinity(a_sgn ^ b_sgn);
    } else if (!a_mnt || !b_mnt) {
        return fp32_zero(a_sgn ^ b_sgn);
    }

    // Multiply and normalise:
    x_sgn = a_sgn ^ b_sgn;
    x_exp = a_exp + b_exp - FP32_EXP_BIAS + 2 * FP32_EXP_BITS + 1;
    x_mnt = (uint64_t)a_mnt * b_mnt;
    x_mnt = fp64_normalise(x_mnt, &x_exp);

    // Convert to FP32_BITS bits, collapsing error into bottom bit:
    x_mnt = lsr64(x_mnt, FP32_BITS - 1) | !!lsl64(x_mnt, FP32_BITS + 1);

    return fp32_round(x_sgn, x_exp, x_mnt, mode, flags);
}

static uint64_t
fp64_mul(uint64_t a, uint64_t b, int mode, int *flags)
{
    int a_sgn, a_exp, b_sgn, b_exp, x_sgn, x_exp;
    uint64_t a_mnt, b_mnt, x;
    uint64_t x0_mnt, x1_mnt;

    fp64_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);
    fp64_unpack(&b_sgn, &b_exp, &b_mnt, b, mode, flags);

    if ((x = fp64_process_NaNs(a, b, mode, flags))) {
        return x;
    }

    // Handle infinities and zeroes:
    if ((a_exp == FP64_EXP_INF && !b_mnt) ||
        (b_exp == FP64_EXP_INF && !a_mnt)) {
        *flags |= FPLIB_IOC;
        return fp64_defaultNaN();
    } else if (a_exp == FP64_EXP_INF || b_exp == FP64_EXP_INF) {
        return fp64_infinity(a_sgn ^ b_sgn);
    } else if (!a_mnt || !b_mnt) {
        return fp64_zero(a_sgn ^ b_sgn);
    }

    // Multiply and normalise:
    x_sgn = a_sgn ^ b_sgn;
    x_exp = a_exp + b_exp - FP64_EXP_BIAS + 2 * FP64_EXP_BITS + 1;
    mul62x62(&x0_mnt, &x1_mnt, a_mnt, b_mnt);
    fp128_normalise(&x0_mnt, &x1_mnt, &x_exp);

    // Convert to FP64_BITS bits, collapsing error into bottom bit:
    x0_mnt = x1_mnt << 1 | !!x0_mnt;

    return fp64_round(x_sgn, x_exp, x0_mnt, mode, flags);
}

static uint16_t
fp16_muladd(uint16_t a, uint16_t b, uint16_t c, int scale,
            int mode, int *flags)
{
    int a_sgn, a_exp, b_sgn, b_exp, c_sgn, c_exp, x_sgn, x_exp, y_sgn, y_exp;
    uint16_t a_mnt, b_mnt, c_mnt, x;
    uint32_t x_mnt, y_mnt;

    fp16_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);
    fp16_unpack(&b_sgn, &b_exp, &b_mnt, b, mode, flags);
    fp16_unpack(&c_sgn, &c_exp, &c_mnt, c, mode, flags);

    x = fp16_process_NaNs3(a, b, c, mode, flags);

    // Quiet NaN added to product of zero and infinity:
    if (fp16_is_quiet_NaN(a_exp, a_mnt) &&
        ((!b_mnt && fp16_is_infinity(c_exp, c_mnt)) ||
         (!c_mnt && fp16_is_infinity(b_exp, b_mnt)))) {
        x = fp16_defaultNaN();
        *flags |= FPLIB_IOC;
    }

    if (x) {
        return x;
    }

    // Handle infinities and zeroes:
    if ((b_exp == FP16_EXP_INF && !c_mnt) ||
        (c_exp == FP16_EXP_INF && !b_mnt) ||
        (a_exp == FP16_EXP_INF &&
         (b_exp == FP16_EXP_INF || c_exp == FP16_EXP_INF) &&
         (a_sgn != (b_sgn ^ c_sgn)))) {
        *flags |= FPLIB_IOC;
        return fp16_defaultNaN();
    }
    if (a_exp == FP16_EXP_INF)
        return fp16_infinity(a_sgn);
    if (b_exp == FP16_EXP_INF || c_exp == FP16_EXP_INF)
        return fp16_infinity(b_sgn ^ c_sgn);
    if (!a_mnt && (!b_mnt || !c_mnt) && a_sgn == (b_sgn ^ c_sgn))
        return fp16_zero(a_sgn);

    x_sgn = a_sgn;
    x_exp = a_exp + 2 * FP16_EXP_BITS - 3;
    x_mnt = (uint32_t)a_mnt << (FP16_MANT_BITS + 4);

    // Multiply:
    y_sgn = b_sgn ^ c_sgn;
    y_exp = b_exp + c_exp - FP16_EXP_BIAS + 2 * FP16_EXP_BITS + 1 - 3;
    y_mnt = (uint32_t)b_mnt * c_mnt << 3;
    if (!y_mnt) {
        y_exp = x_exp;
    }

    // Add:
    if (x_exp >= y_exp) {
        y_mnt = (lsr32(y_mnt, x_exp - y_exp) |
                 !!(y_mnt & (lsl32(1, x_exp - y_exp) - 1)));
        y_exp = x_exp;
    } else {
        x_mnt = (lsr32(x_mnt, y_exp - x_exp) |
                 !!(x_mnt & (lsl32(1, y_exp - x_exp) - 1)));
        x_exp = y_exp;
    }
    if (x_sgn == y_sgn) {
        x_mnt = x_mnt + y_mnt;
    } else if (x_mnt >= y_mnt) {
        x_mnt = x_mnt - y_mnt;
    } else {
        x_sgn ^= 1;
        x_mnt = y_mnt - x_mnt;
    }

    if (!x_mnt) {
        // Sign of exact zero result depends on rounding mode
        return fp16_zero((mode & 3) == 2);
    }

    // Normalise into FP16_BITS bits, collapsing error into bottom bit:
    x_mnt = fp32_normalise(x_mnt, &x_exp);
    x_mnt = x_mnt >> (FP16_BITS - 1) | !!(uint16_t)(x_mnt << 1);

    return fp16_round(x_sgn, x_exp + scale, x_mnt, mode, flags);
}

static uint32_t
fp32_muladd(uint32_t a, uint32_t b, uint32_t c, int scale,
            int mode, int *flags)
{
    int a_sgn, a_exp, b_sgn, b_exp, c_sgn, c_exp, x_sgn, x_exp, y_sgn, y_exp;
    uint32_t a_mnt, b_mnt, c_mnt, x;
    uint64_t x_mnt, y_mnt;

    fp32_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);
    fp32_unpack(&b_sgn, &b_exp, &b_mnt, b, mode, flags);
    fp32_unpack(&c_sgn, &c_exp, &c_mnt, c, mode, flags);

    x = fp32_process_NaNs3(a, b, c, mode, flags);

    // Quiet NaN added to product of zero and infinity:
    if (fp32_is_quiet_NaN(a_exp, a_mnt) &&
        ((!b_mnt && fp32_is_infinity(c_exp, c_mnt)) ||
         (!c_mnt && fp32_is_infinity(b_exp, b_mnt)))) {
        x = fp32_defaultNaN();
        *flags |= FPLIB_IOC;
    }

    if (x) {
        return x;
    }

    // Handle infinities and zeroes:
    if ((b_exp == FP32_EXP_INF && !c_mnt) ||
        (c_exp == FP32_EXP_INF && !b_mnt) ||
        (a_exp == FP32_EXP_INF &&
         (b_exp == FP32_EXP_INF || c_exp == FP32_EXP_INF) &&
         (a_sgn != (b_sgn ^ c_sgn)))) {
        *flags |= FPLIB_IOC;
        return fp32_defaultNaN();
    }
    if (a_exp == FP32_EXP_INF)
        return fp32_infinity(a_sgn);
    if (b_exp == FP32_EXP_INF || c_exp == FP32_EXP_INF)
        return fp32_infinity(b_sgn ^ c_sgn);
    if (!a_mnt && (!b_mnt || !c_mnt) && a_sgn == (b_sgn ^ c_sgn))
        return fp32_zero(a_sgn);

    x_sgn = a_sgn;
    x_exp = a_exp + 2 * FP32_EXP_BITS - 3;
    x_mnt = (uint64_t)a_mnt << (FP32_MANT_BITS + 4);

    // Multiply:
    y_sgn = b_sgn ^ c_sgn;
    y_exp = b_exp + c_exp - FP32_EXP_BIAS + 2 * FP32_EXP_BITS + 1 - 3;
    y_mnt = (uint64_t)b_mnt * c_mnt << 3;
    if (!y_mnt) {
        y_exp = x_exp;
    }

    // Add:
    if (x_exp >= y_exp) {
        y_mnt = (lsr64(y_mnt, x_exp - y_exp) |
                 !!(y_mnt & (lsl64(1, x_exp - y_exp) - 1)));
        y_exp = x_exp;
    } else {
        x_mnt = (lsr64(x_mnt, y_exp - x_exp) |
                 !!(x_mnt & (lsl64(1, y_exp - x_exp) - 1)));
        x_exp = y_exp;
    }
    if (x_sgn == y_sgn) {
        x_mnt = x_mnt + y_mnt;
    } else if (x_mnt >= y_mnt) {
        x_mnt = x_mnt - y_mnt;
    } else {
        x_sgn ^= 1;
        x_mnt = y_mnt - x_mnt;
    }

    if (!x_mnt) {
        // Sign of exact zero result depends on rounding mode
        return fp32_zero((mode & 3) == 2);
    }

    // Normalise into FP32_BITS bits, collapsing error into bottom bit:
    x_mnt = fp64_normalise(x_mnt, &x_exp);
    x_mnt = x_mnt >> (FP32_BITS - 1) | !!(uint32_t)(x_mnt << 1);

    return fp32_round(x_sgn, x_exp + scale, x_mnt, mode, flags);
}

static uint64_t
fp64_muladd(uint64_t a, uint64_t b, uint64_t c, int scale,
            int mode, int *flags)
{
    int a_sgn, a_exp, b_sgn, b_exp, c_sgn, c_exp, x_sgn, x_exp, y_sgn, y_exp;
    uint64_t a_mnt, b_mnt, c_mnt, x;
    uint64_t x0_mnt, x1_mnt, y0_mnt, y1_mnt;

    fp64_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);
    fp64_unpack(&b_sgn, &b_exp, &b_mnt, b, mode, flags);
    fp64_unpack(&c_sgn, &c_exp, &c_mnt, c, mode, flags);

    x = fp64_process_NaNs3(a, b, c, mode, flags);

    // Quiet NaN added to product of zero and infinity:
    if (fp64_is_quiet_NaN(a_exp, a_mnt) &&
        ((!b_mnt && fp64_is_infinity(c_exp, c_mnt)) ||
         (!c_mnt && fp64_is_infinity(b_exp, b_mnt)))) {
        x = fp64_defaultNaN();
        *flags |= FPLIB_IOC;
    }

    if (x) {
        return x;
    }

    // Handle infinities and zeroes:
    if ((b_exp == FP64_EXP_INF && !c_mnt) ||
        (c_exp == FP64_EXP_INF && !b_mnt) ||
        (a_exp == FP64_EXP_INF &&
         (b_exp == FP64_EXP_INF || c_exp == FP64_EXP_INF) &&
         (a_sgn != (b_sgn ^ c_sgn)))) {
        *flags |= FPLIB_IOC;
        return fp64_defaultNaN();
    }
    if (a_exp == FP64_EXP_INF)
        return fp64_infinity(a_sgn);
    if (b_exp == FP64_EXP_INF || c_exp == FP64_EXP_INF)
        return fp64_infinity(b_sgn ^ c_sgn);
    if (!a_mnt && (!b_mnt || !c_mnt) && a_sgn == (b_sgn ^ c_sgn))
        return fp64_zero(a_sgn);

    x_sgn = a_sgn;
    x_exp = a_exp + FP64_EXP_BITS;
    x0_mnt = 0;
    x1_mnt = a_mnt;

    // Multiply:
    y_sgn = b_sgn ^ c_sgn;
    y_exp = b_exp + c_exp - FP64_EXP_BIAS + 2 * FP64_EXP_BITS + 1 - 3;
    mul62x62(&y0_mnt, &y1_mnt, b_mnt, c_mnt << 3);
    if (!y0_mnt && !y1_mnt) {
        y_exp = x_exp;
    }

    // Add:
    if (x_exp >= y_exp) {
        uint64_t t0, t1;
        lsl128(&t0, &t1, y0_mnt, y1_mnt,
               x_exp - y_exp < 128 ? 128 - (x_exp - y_exp) : 0);
        lsr128(&y0_mnt, &y1_mnt, y0_mnt, y1_mnt, x_exp - y_exp);
        y0_mnt |= !!(t0 | t1);
        y_exp = x_exp;
    } else {
        uint64_t t0, t1;
        lsl128(&t0, &t1, x0_mnt, x1_mnt,
               y_exp - x_exp < 128 ? 128 - (y_exp - x_exp) : 0);
        lsr128(&x0_mnt, &x1_mnt, x0_mnt, x1_mnt, y_exp - x_exp);
        x0_mnt |= !!(t0 | t1);
        x_exp = y_exp;
    }
    if (x_sgn == y_sgn) {
        add128(&x0_mnt, &x1_mnt, x0_mnt, x1_mnt, y0_mnt, y1_mnt);
    } else if (cmp128(x0_mnt, x1_mnt, y0_mnt, y1_mnt) >= 0) {
        sub128(&x0_mnt, &x1_mnt, x0_mnt, x1_mnt, y0_mnt, y1_mnt);
    } else {
        x_sgn ^= 1;
        sub128(&x0_mnt, &x1_mnt, y0_mnt, y1_mnt, x0_mnt, x1_mnt);
    }

    if (!x0_mnt && !x1_mnt) {
        // Sign of exact zero result depends on rounding mode
        return fp64_zero((mode & 3) == 2);
    }

    // Normalise into FP64_BITS bits, collapsing error into bottom bit:
    fp128_normalise(&x0_mnt, &x1_mnt, &x_exp);
    x0_mnt = x1_mnt << 1 | !!x0_mnt;

    return fp64_round(x_sgn, x_exp + scale, x0_mnt, mode, flags);
}

static uint16_t
fp16_div(uint16_t a, uint16_t b, int mode, int *flags)
{
    int a_sgn, a_exp, b_sgn, b_exp, x_sgn, x_exp;
    uint16_t a_mnt, b_mnt, x;
    uint32_t x_mnt;

    fp16_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);
    fp16_unpack(&b_sgn, &b_exp, &b_mnt, b, mode, flags);

    if ((x = fp16_process_NaNs(a, b, mode, flags)))
        return x;

    // Handle infinities and zeroes:
    if ((a_exp == FP16_EXP_INF && b_exp == FP16_EXP_INF) ||
        (!a_mnt && !b_mnt)) {
        *flags |= FPLIB_IOC;
        return fp16_defaultNaN();
    }
    if (a_exp == FP16_EXP_INF || !b_mnt) {
        if (a_exp != FP16_EXP_INF)
            *flags |= FPLIB_DZC;
        return fp16_infinity(a_sgn ^ b_sgn);
    }
    if (!a_mnt || b_exp == FP16_EXP_INF)
        return fp16_zero(a_sgn ^ b_sgn);

    // Divide, setting bottom bit if inexact:
    a_mnt = fp16_normalise(a_mnt, &a_exp);
    x_sgn = a_sgn ^ b_sgn;
    x_exp = a_exp - b_exp + (FP16_EXP_BIAS + FP16_BITS + 2 * FP16_EXP_BITS - 3);
    x_mnt = ((uint32_t)a_mnt << (FP16_MANT_BITS - FP16_EXP_BITS + 3)) / b_mnt;
    x_mnt |= (x_mnt * b_mnt !=
              (uint32_t)a_mnt << (FP16_MANT_BITS - FP16_EXP_BITS + 3));

    // Normalise into FP16_BITS bits, collapsing error into bottom bit:
    x_mnt = fp32_normalise(x_mnt, &x_exp);
    x_mnt = x_mnt >> (FP16_BITS - 1) | !!(uint16_t)(x_mnt << 1);

    return fp16_round(x_sgn, x_exp, x_mnt, mode, flags);
}

static uint32_t
fp32_div(uint32_t a, uint32_t b, int mode, int *flags)
{
    int a_sgn, a_exp, b_sgn, b_exp, x_sgn, x_exp;
    uint32_t a_mnt, b_mnt, x;
    uint64_t x_mnt;

    fp32_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);
    fp32_unpack(&b_sgn, &b_exp, &b_mnt, b, mode, flags);

    if ((x = fp32_process_NaNs(a, b, mode, flags)))
        return x;

    // Handle infinities and zeroes:
    if ((a_exp == FP32_EXP_INF && b_exp == FP32_EXP_INF) ||
        (!a_mnt && !b_mnt)) {
        *flags |= FPLIB_IOC;
        return fp32_defaultNaN();
    }
    if (a_exp == FP32_EXP_INF || !b_mnt) {
        if (a_exp != FP32_EXP_INF)
            *flags |= FPLIB_DZC;
        return fp32_infinity(a_sgn ^ b_sgn);
    }
    if (!a_mnt || b_exp == FP32_EXP_INF)
        return fp32_zero(a_sgn ^ b_sgn);

    // Divide, setting bottom bit if inexact:
    a_mnt = fp32_normalise(a_mnt, &a_exp);
    x_sgn = a_sgn ^ b_sgn;
    x_exp = a_exp - b_exp + (FP32_EXP_BIAS + FP32_BITS + 2 * FP32_EXP_BITS - 3);
    x_mnt = ((uint64_t)a_mnt << (FP32_MANT_BITS - FP32_EXP_BITS + 3)) / b_mnt;
    x_mnt |= (x_mnt * b_mnt !=
              (uint64_t)a_mnt << (FP32_MANT_BITS - FP32_EXP_BITS + 3));

    // Normalise into FP32_BITS bits, collapsing error into bottom bit:
    x_mnt = fp64_normalise(x_mnt, &x_exp);
    x_mnt = x_mnt >> (FP32_BITS - 1) | !!(uint32_t)(x_mnt << 1);

    return fp32_round(x_sgn, x_exp, x_mnt, mode, flags);
}

static uint64_t
fp64_div(uint64_t a, uint64_t b, int mode, int *flags)
{
    int a_sgn, a_exp, b_sgn, b_exp, x_sgn, x_exp, c;
    uint64_t a_mnt, b_mnt, x, x_mnt, x0_mnt, x1_mnt;

    fp64_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);
    fp64_unpack(&b_sgn, &b_exp, &b_mnt, b, mode, flags);

    if ((x = fp64_process_NaNs(a, b, mode, flags)))
        return x;

    // Handle infinities and zeroes:
    if ((a_exp == FP64_EXP_INF && b_exp == FP64_EXP_INF) ||
        (!a_mnt && !b_mnt)) {
        *flags |= FPLIB_IOC;
        return fp64_defaultNaN();
    }
    if (a_exp == FP64_EXP_INF || !b_mnt) {
        if (a_exp != FP64_EXP_INF)
            *flags |= FPLIB_DZC;
        return fp64_infinity(a_sgn ^ b_sgn);
    }
    if (!a_mnt || b_exp == FP64_EXP_INF)
        return fp64_zero(a_sgn ^ b_sgn);

    // Find reciprocal of divisor with Newton-Raphson:
    a_mnt = fp64_normalise(a_mnt, &a_exp);
    b_mnt = fp64_normalise(b_mnt, &b_exp);
    x_mnt = ~(uint64_t)0 / (b_mnt >> 31);
    mul64x32(&x0_mnt, &x1_mnt, b_mnt, x_mnt);
    sub128(&x0_mnt, &x1_mnt, 0, (uint64_t)1 << 32, x0_mnt, x1_mnt);
    lsr128(&x0_mnt, &x1_mnt, x0_mnt, x1_mnt, 32);
    mul64x32(&x0_mnt, &x1_mnt, x0_mnt, x_mnt);
    lsr128(&x0_mnt, &x1_mnt, x0_mnt, x1_mnt, 33);

    // Multiply by dividend:
    x_sgn = a_sgn ^ b_sgn;
    x_exp = a_exp - b_exp + FP64_EXP_BIAS + 8;
    mul62x62(&x0_mnt, &x1_mnt, x0_mnt, a_mnt >> 2);
    lsr128(&x0_mnt, &x1_mnt, x0_mnt, x1_mnt, 4);
    x_mnt = x1_mnt;

    // This is an underestimate, so try adding one:
    mul62x62(&x0_mnt, &x1_mnt, b_mnt >> 2, x_mnt + 1);
    c = cmp128(x0_mnt, x1_mnt, 0, a_mnt >> 11);
    if (c <= 0) {
        ++x_mnt;
    }

    x_mnt = fp64_normalise(x_mnt, &x_exp);

    return fp64_round(x_sgn, x_exp, x_mnt << 1 | !!c, mode, flags);
}

static void
set_fpscr0(FPSCR &fpscr, int flags)
{
    if (flags & FPLIB_IDC) {
        fpscr.idc = 1;
    }
    if (flags & FPLIB_IOC) {
        fpscr.ioc = 1;
    }
    if (flags & FPLIB_DZC) {
        fpscr.dzc = 1;
    }
    if (flags & FPLIB_OFC) {
        fpscr.ofc = 1;
    }
    if (flags & FPLIB_UFC) {
        fpscr.ufc = 1;
    }
    if (flags & FPLIB_IXC) {
        fpscr.ixc = 1;
    }
}

static uint16_t
fp16_scale(uint16_t a, int16_t b, int mode, int *flags)
{
    int a_sgn, a_exp;
    uint16_t a_mnt;

    fp16_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);

    // Handle NaNs:
    if (fp16_is_NaN(a_exp, a_mnt)) {
        return fp16_process_NaN(a, mode, flags);
    }

    // Handle zeroes:
    if (!a_mnt) {
        return fp16_zero(a_sgn);
    }

    // Handle infinities:
    if (a_exp == FP16_EXP_INF) {
        return fp16_infinity(a_sgn);
    }

    b = b < -300 ? -300 : b;
    b = b >  300 ?  300 : b;
    a_exp += b;
    a_mnt <<= 3;

    a_mnt = fp16_normalise(a_mnt, &a_exp);

    return fp16_round(a_sgn, a_exp + FP16_EXP_BITS - 3, a_mnt << 1,
                      mode, flags);
}

static uint32_t
fp32_scale(uint32_t a, int32_t b, int mode, int *flags)
{
    int a_sgn, a_exp;
    uint32_t a_mnt;

    fp32_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);

    // Handle NaNs:
    if (fp32_is_NaN(a_exp, a_mnt)) {
        return fp32_process_NaN(a, mode, flags);
    }

    // Handle zeroes:
    if (!a_mnt) {
        return fp32_zero(a_sgn);
    }

    // Handle infinities:
    if (a_exp == FP32_EXP_INF) {
        return fp32_infinity(a_sgn);
    }

    b = b < -300 ? -300 : b;
    b = b >  300 ?  300 : b;
    a_exp += b;
    a_mnt <<= 3;

    a_mnt = fp32_normalise(a_mnt, &a_exp);

    return fp32_round(a_sgn, a_exp + FP32_EXP_BITS - 3, a_mnt << 1,
                      mode, flags);
}

static uint64_t
fp64_scale(uint64_t a, int64_t b, int mode, int *flags)
{
    int a_sgn, a_exp;
    uint64_t a_mnt;

    fp64_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);

    // Handle NaNs:
    if (fp64_is_NaN(a_exp, a_mnt)) {
        return fp64_process_NaN(a, mode, flags);
    }

    // Handle zeroes:
    if (!a_mnt) {
        return fp64_zero(a_sgn);
    }

    // Handle infinities:
    if (a_exp == FP64_EXP_INF) {
        return fp64_infinity(a_sgn);
    }

    b = b < -3000 ? -3000 : b;
    b = b >  3000 ?  3000 : b;
    a_exp += b;
    a_mnt <<= 3;

    a_mnt = fp64_normalise(a_mnt, &a_exp);

    return fp64_round(a_sgn, a_exp + FP64_EXP_BITS - 3, a_mnt << 1,
                      mode, flags);
}

static uint16_t
fp16_sqrt(uint16_t a, int mode, int *flags)
{
    int a_sgn, a_exp, x_sgn, x_exp;
    uint16_t a_mnt, x_mnt;
    uint32_t x, t0, t1;

    fp16_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);

    // Handle NaNs:
    if (fp16_is_NaN(a_exp, a_mnt))
        return fp16_process_NaN(a, mode, flags);

    // Handle infinities and zeroes:
    if (!a_mnt)
        return fp16_zero(a_sgn);
    if (a_exp == FP16_EXP_INF && !a_sgn)
        return fp16_infinity(a_sgn);
    if (a_sgn) {
        *flags |= FPLIB_IOC;
        return fp16_defaultNaN();
    }

    a_mnt = fp16_normalise(a_mnt, &a_exp);
    if (a_exp & 1) {
        ++a_exp;
        a_mnt >>= 1;
    }

    // x = (a * 3 + 5) / 8
    x = ((uint32_t)a_mnt << 14) + ((uint32_t)a_mnt << 13) + ((uint32_t)5 << 28);

    // x = (a / x + x) / 2; // 8-bit accuracy
    x = (((uint32_t)a_mnt << 16) / (x >> 15) + (x >> 16)) << 15;

    // x = (a / x + x) / 2; // 16-bit accuracy
    x = (((uint32_t)a_mnt << 16) / (x >> 15) + (x >> 16)) << 15;

    x_sgn = 0;
    x_exp = (a_exp + 27) >> 1;
    x_mnt = ((x - (1 << 18)) >> 19) + 1;
    t1 = (uint32_t)x_mnt * x_mnt;
    t0 = (uint32_t)a_mnt << 9;
    if (t1 > t0) {
        --x_mnt;
    }

    x_mnt = fp16_normalise(x_mnt, &x_exp);

    return fp16_round(x_sgn, x_exp, x_mnt << 1 | (t1 != t0), mode, flags);
}

static uint32_t
fp32_sqrt(uint32_t a, int mode, int *flags)
{
    int a_sgn, a_exp, x_sgn, x_exp;
    uint32_t a_mnt, x, x_mnt;
    uint64_t t0, t1;

    fp32_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);

    // Handle NaNs:
    if (fp32_is_NaN(a_exp, a_mnt))
        return fp32_process_NaN(a, mode, flags);

    // Handle infinities and zeroes:
    if (!a_mnt)
        return fp32_zero(a_sgn);
    if (a_exp == FP32_EXP_INF && !a_sgn)
        return fp32_infinity(a_sgn);
    if (a_sgn) {
        *flags |= FPLIB_IOC;
        return fp32_defaultNaN();
    }

    a_mnt = fp32_normalise(a_mnt, &a_exp);
    if (!(a_exp & 1)) {
        ++a_exp;
        a_mnt >>= 1;
    }

    // x = (a * 3 + 5) / 8
    x = (a_mnt >> 2) + (a_mnt >> 3) + ((uint32_t)5 << 28);

    // x = (a / x + x) / 2; // 8-bit accuracy
    x = (a_mnt / (x >> 15) + (x >> 16)) << 15;

    // x = (a / x + x) / 2; // 16-bit accuracy
    x = (a_mnt / (x >> 15) + (x >> 16)) << 15;

    // x = (a / x + x) / 2; // 32-bit accuracy
    x = ((((uint64_t)a_mnt << 32) / x) >> 2) + (x >> 1);

    x_sgn = 0;
    x_exp = (a_exp + 147) >> 1;
    x_mnt = ((x - (1 << 5)) >> 6) + 1;
    t1 = (uint64_t)x_mnt * x_mnt;
    t0 = (uint64_t)a_mnt << 19;
    if (t1 > t0) {
        --x_mnt;
    }

    x_mnt = fp32_normalise(x_mnt, &x_exp);

    return fp32_round(x_sgn, x_exp, x_mnt << 1 | (t1 != t0), mode, flags);
}

static uint64_t
fp64_sqrt(uint64_t a, int mode, int *flags)
{
    int a_sgn, a_exp, x_sgn, x_exp, c;
    uint64_t a_mnt, x_mnt, r, x0, x1;
    uint32_t x;

    fp64_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);

    // Handle NaNs:
    if (fp64_is_NaN(a_exp, a_mnt))
        return fp64_process_NaN(a, mode, flags);

    // Handle infinities and zeroes:
    if (!a_mnt)
        return fp64_zero(a_sgn);
    if (a_exp == FP64_EXP_INF && !a_sgn)
        return fp64_infinity(a_sgn);
    if (a_sgn) {
        *flags |= FPLIB_IOC;
        return fp64_defaultNaN();
    }

    a_mnt = fp64_normalise(a_mnt, &a_exp);
    if (a_exp & 1) {
        ++a_exp;
        a_mnt >>= 1;
    }

    // x = (a * 3 + 5) / 8
    x = (a_mnt >> 34) + (a_mnt >> 35) + ((uint32_t)5 << 28);

    // x = (a / x + x) / 2; // 8-bit accuracy
    x = ((a_mnt >> 32) / (x >> 15) + (x >> 16)) << 15;

    // x = (a / x + x) / 2; // 16-bit accuracy
    x = ((a_mnt >> 32) / (x >> 15) + (x >> 16)) << 15;

    // x = (a / x + x) / 2; // 32-bit accuracy
    x = ((a_mnt / x) >> 2) + (x >> 1);

    // r = 1 / x; // 32-bit accuracy
    r = ((uint64_t)1 << 62) / x;

    // r = r * (2 - x * r); // 64-bit accuracy
    mul64x32(&x0, &x1, -(uint64_t)x * r << 1, r);
    lsr128(&x0, &x1, x0, x1, 31);

    // x = (x + a * r) / 2; // 64-bit accuracy
    mul62x62(&x0, &x1, a_mnt >> 10, x0 >> 2);
    lsl128(&x0, &x1, x0, x1, 5);
    lsr128(&x0, &x1, x0, x1, 56);

    x0 = ((uint64_t)x << 31) + (x0 >> 1);

    x_sgn = 0;
    x_exp = (a_exp + 1053) >> 1;
    x_mnt = x0;
    x_mnt = ((x_mnt - (1 << 8)) >> 9) + 1;
    mul62x62(&x0, &x1, x_mnt, x_mnt);
    lsl128(&x0, &x1, x0, x1, 19);
    c = cmp128(x0, x1, 0, a_mnt);
    if (c > 0)
        --x_mnt;

    x_mnt = fp64_normalise(x_mnt, &x_exp);

    return fp64_round(x_sgn, x_exp, x_mnt << 1 | !!c, mode, flags);
}

static int
modeConv(FPSCR fpscr)
{
    uint32_t x = (uint32_t)fpscr;
    return (x >> 22 & 0xf) | (x >> 19 & 1 ? FPLIB_FZ16 : 0);
    // AHP bit is ignored. Only fplibConvert uses AHP.
}

static void
set_fpscr(FPSCR &fpscr, int flags)
{
    // translate back to FPSCR
    bool underflow = false;
    if (flags & FPLIB_IDC) {
        fpscr.idc = 1;
    }
    if (flags & FPLIB_IOC) {
        fpscr.ioc = 1;
    }
    if (flags & FPLIB_DZC) {
        fpscr.dzc = 1;
    }
    if (flags & FPLIB_OFC) {
        fpscr.ofc = 1;
    }
    if (flags & FPLIB_UFC) {
        underflow = true; //xx Why is this required?
        fpscr.ufc = 1;
    }
    if ((flags & FPLIB_IXC) && !(underflow && fpscr.fz)) {
        fpscr.ixc = 1;
    }
}

template <>
bool
fplibCompareEQ(uint16_t a, uint16_t b, FPSCR &fpscr)
{
    int flags = 0;
    int x = fp16_compare_eq(a, b, modeConv(fpscr), &flags);
    set_fpscr(fpscr, flags);
    return x;
}

template <>
bool
fplibCompareGE(uint16_t a, uint16_t b, FPSCR &fpscr)
{
    int flags = 0;
    int x = fp16_compare_ge(a, b, modeConv(fpscr), &flags);
    set_fpscr(fpscr, flags);
    return x;
}

template <>
bool
fplibCompareGT(uint16_t a, uint16_t b, FPSCR &fpscr)
{
    int flags = 0;
    int x = fp16_compare_gt(a, b, modeConv(fpscr), &flags);
    set_fpscr(fpscr, flags);
    return x;
}

template <>
bool
fplibCompareUN(uint16_t a, uint16_t b, FPSCR &fpscr)
{
    int flags = 0;
    int x = fp16_compare_un(a, b, modeConv(fpscr), &flags);
    set_fpscr(fpscr, flags);
    return x;
}

template <>
bool
fplibCompareEQ(uint32_t a, uint32_t b, FPSCR &fpscr)
{
    int flags = 0;
    int x = fp32_compare_eq(a, b, modeConv(fpscr), &flags);
    set_fpscr(fpscr, flags);
    return x;
}

template <>
bool
fplibCompareGE(uint32_t a, uint32_t b, FPSCR &fpscr)
{
    int flags = 0;
    int x = fp32_compare_ge(a, b, modeConv(fpscr), &flags);
    set_fpscr(fpscr, flags);
    return x;
}

template <>
bool
fplibCompareGT(uint32_t a, uint32_t b, FPSCR &fpscr)
{
    int flags = 0;
    int x = fp32_compare_gt(a, b, modeConv(fpscr), &flags);
    set_fpscr(fpscr, flags);
    return x;
}

template <>
bool
fplibCompareUN(uint32_t a, uint32_t b, FPSCR &fpscr)
{
    int flags = 0;
    int x = fp32_compare_un(a, b, modeConv(fpscr), &flags);
    set_fpscr(fpscr, flags);
    return x;
}

template <>
bool
fplibCompareEQ(uint64_t a, uint64_t b, FPSCR &fpscr)
{
    int flags = 0;
    int x = fp64_compare_eq(a, b, modeConv(fpscr), &flags);
    set_fpscr(fpscr, flags);
    return x;
}

template <>
bool
fplibCompareGE(uint64_t a, uint64_t b, FPSCR &fpscr)
{
    int flags = 0;
    int x = fp64_compare_ge(a, b, modeConv(fpscr), &flags);
    set_fpscr(fpscr, flags);
    return x;
}

template <>
bool
fplibCompareGT(uint64_t a, uint64_t b, FPSCR &fpscr)
{
    int flags = 0;
    int x = fp64_compare_gt(a, b, modeConv(fpscr), &flags);
    set_fpscr(fpscr, flags);
    return x;
}

template <>
bool
fplibCompareUN(uint64_t a, uint64_t b, FPSCR &fpscr)
{
    int flags = 0;
    int x = fp64_compare_un(a, b, modeConv(fpscr), &flags);
    set_fpscr(fpscr, flags);
    return x;
}

template <>
uint16_t
fplibAbs(uint16_t op)
{
    return op & ~(1ULL << (FP16_BITS - 1));
}

template <>
uint32_t
fplibAbs(uint32_t op)
{
    return op & ~(1ULL << (FP32_BITS - 1));
}

template <>
uint64_t
fplibAbs(uint64_t op)
{
    return op & ~(1ULL << (FP64_BITS - 1));
}

template <>
uint16_t
fplibAdd(uint16_t op1, uint16_t op2, FPSCR &fpscr)
{
    int flags = 0;
    uint16_t result = fp16_add(op1, op2, 0, modeConv(fpscr), &flags);
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint32_t
fplibAdd(uint32_t op1, uint32_t op2, FPSCR &fpscr)
{
    int flags = 0;
    uint32_t result = fp32_add(op1, op2, 0, modeConv(fpscr), &flags);
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint64_t
fplibAdd(uint64_t op1, uint64_t op2, FPSCR &fpscr)
{
    int flags = 0;
    uint64_t result = fp64_add(op1, op2, 0, modeConv(fpscr), &flags);
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
int
fplibCompare(uint16_t op1, uint16_t op2, bool signal_nans, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn1, exp1, sgn2, exp2, result;
    uint16_t mnt1, mnt2;

    fp16_unpack(&sgn1, &exp1, &mnt1, op1, mode, &flags);
    fp16_unpack(&sgn2, &exp2, &mnt2, op2, mode, &flags);

    if (fp16_is_NaN(exp1, mnt1) || fp16_is_NaN(exp2, mnt2)) {
        result = 3;
        if (fp16_is_signalling_NaN(exp1, mnt1) ||
            fp16_is_signalling_NaN(exp2, mnt2) || signal_nans)
            flags |= FPLIB_IOC;
    } else {
        if (op1 == op2 || (!mnt1 && !mnt2)) {
            result = 6;
        } else if (sgn1 != sgn2) {
            result = sgn1 ? 8 : 2;
        } else if (exp1 != exp2) {
            result = sgn1 ^ (exp1 < exp2) ? 8 : 2;
        } else {
            result = sgn1 ^ (mnt1 < mnt2) ? 8 : 2;
        }
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
int
fplibCompare(uint32_t op1, uint32_t op2, bool signal_nans, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn1, exp1, sgn2, exp2, result;
    uint32_t mnt1, mnt2;

    fp32_unpack(&sgn1, &exp1, &mnt1, op1, mode, &flags);
    fp32_unpack(&sgn2, &exp2, &mnt2, op2, mode, &flags);

    if (fp32_is_NaN(exp1, mnt1) || fp32_is_NaN(exp2, mnt2)) {
        result = 3;
        if (fp32_is_signalling_NaN(exp1, mnt1) ||
            fp32_is_signalling_NaN(exp2, mnt2) || signal_nans)
            flags |= FPLIB_IOC;
    } else {
        if (op1 == op2 || (!mnt1 && !mnt2)) {
            result = 6;
        } else if (sgn1 != sgn2) {
            result = sgn1 ? 8 : 2;
        } else if (exp1 != exp2) {
            result = sgn1 ^ (exp1 < exp2) ? 8 : 2;
        } else {
            result = sgn1 ^ (mnt1 < mnt2) ? 8 : 2;
        }
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
int
fplibCompare(uint64_t op1, uint64_t op2, bool signal_nans, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn1, exp1, sgn2, exp2, result;
    uint64_t mnt1, mnt2;

    fp64_unpack(&sgn1, &exp1, &mnt1, op1, mode, &flags);
    fp64_unpack(&sgn2, &exp2, &mnt2, op2, mode, &flags);

    if (fp64_is_NaN(exp1, mnt1) || fp64_is_NaN(exp2, mnt2)) {
        result = 3;
        if (fp64_is_signalling_NaN(exp1, mnt1) ||
            fp64_is_signalling_NaN(exp2, mnt2) || signal_nans)
            flags |= FPLIB_IOC;
    } else {
        if (op1 == op2 || (!mnt1 && !mnt2)) {
            result = 6;
        } else if (sgn1 != sgn2) {
            result = sgn1 ? 8 : 2;
        } else if (exp1 != exp2) {
            result = sgn1 ^ (exp1 < exp2) ? 8 : 2;
        } else {
            result = sgn1 ^ (mnt1 < mnt2) ? 8 : 2;
        }
    }

    set_fpscr0(fpscr, flags);

    return result;
}

static uint16_t
fp16_FPConvertNaN_32(uint32_t op)
{
    return fp16_pack(op >> (FP32_BITS - 1), FP16_EXP_INF,
                     1ULL << (FP16_MANT_BITS - 1) |
                     op >> (FP32_MANT_BITS - FP16_MANT_BITS));
}

static uint16_t
fp16_FPConvertNaN_64(uint64_t op)
{
    return fp16_pack(op >> (FP64_BITS - 1), FP16_EXP_INF,
                     1ULL << (FP16_MANT_BITS - 1) |
                     op >> (FP64_MANT_BITS - FP16_MANT_BITS));
}

static uint32_t
fp32_FPConvertNaN_16(uint16_t op)
{
    return fp32_pack(op >> (FP16_BITS - 1), FP32_EXP_INF,
                     1ULL << (FP32_MANT_BITS - 1) |
                     (uint32_t)op << (FP32_MANT_BITS - FP16_MANT_BITS));
}

static uint32_t
fp32_FPConvertNaN_64(uint64_t op)
{
    return fp32_pack(op >> (FP64_BITS - 1), FP32_EXP_INF,
                     1ULL << (FP32_MANT_BITS - 1) |
                     op >> (FP64_MANT_BITS - FP32_MANT_BITS));
}

static uint64_t
fp64_FPConvertNaN_16(uint16_t op)
{
    return fp64_pack(op >> (FP16_BITS - 1), FP64_EXP_INF,
                     1ULL << (FP64_MANT_BITS - 1) |
                     (uint64_t)op << (FP64_MANT_BITS - FP16_MANT_BITS));
}

static uint64_t
fp64_FPConvertNaN_32(uint32_t op)
{
    return fp64_pack(op >> (FP32_BITS - 1), FP64_EXP_INF,
                     1ULL << (FP64_MANT_BITS - 1) |
                     (uint64_t)op << (FP64_MANT_BITS - FP32_MANT_BITS));
}

static uint16_t
fp16_FPOnePointFive(int sgn)
{
    return fp16_pack(sgn, FP16_EXP_BIAS, 1ULL << (FP16_MANT_BITS - 1));
}

static uint32_t
fp32_FPOnePointFive(int sgn)
{
    return fp32_pack(sgn, FP32_EXP_BIAS, 1ULL << (FP32_MANT_BITS - 1));
}

static uint64_t
fp64_FPOnePointFive(int sgn)
{
    return fp64_pack(sgn, FP64_EXP_BIAS, 1ULL << (FP64_MANT_BITS - 1));
}

static uint16_t
fp16_FPThree(int sgn)
{
    return fp16_pack(sgn, FP16_EXP_BIAS + 1, 1ULL << (FP16_MANT_BITS - 1));
}

static uint32_t
fp32_FPThree(int sgn)
{
    return fp32_pack(sgn, FP32_EXP_BIAS + 1, 1ULL << (FP32_MANT_BITS - 1));
}

static uint64_t
fp64_FPThree(int sgn)
{
    return fp64_pack(sgn, FP64_EXP_BIAS + 1, 1ULL << (FP64_MANT_BITS - 1));
}

static uint16_t
fp16_FPTwo(int sgn)
{
    return fp16_pack(sgn, FP16_EXP_BIAS + 1, 0);
}

static uint32_t
fp32_FPTwo(int sgn)
{
    return fp32_pack(sgn, FP32_EXP_BIAS + 1, 0);
}

static uint64_t
fp64_FPTwo(int sgn)
{
    return fp64_pack(sgn, FP64_EXP_BIAS + 1, 0);
}

template <>
uint16_t
fplibConvert(uint32_t op, FPRounding rounding, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn, exp;
    uint32_t mnt;
    uint16_t result;

    // Unpack floating-point operand optionally with flush-to-zero:
    fp32_unpack(&sgn, &exp, &mnt, op, mode, &flags);

    bool alt_hp = fpscr.ahp;

    if (fp32_is_NaN(exp, mnt)) {
        if (alt_hp) {
            result = fp16_zero(sgn);
        } else if (fpscr.dn) {
            result = fp16_defaultNaN();
        } else {
            result = fp16_FPConvertNaN_32(op);
        }
        if (!(mnt >> (FP32_MANT_BITS - 1) & 1) || alt_hp) {
            flags |= FPLIB_IOC;
        }
    } else if (exp == FP32_EXP_INF) {
        if (alt_hp) {
            result = ((uint16_t)sgn << (FP16_BITS - 1) |
                      ((1ULL << (FP16_BITS - 1)) - 1));
            flags |= FPLIB_IOC;
        } else {
            result = fp16_infinity(sgn);
        }
    } else if (!mnt) {
        result = fp16_zero(sgn);
    } else {
        result =
            fp16_round_(sgn, exp - FP32_EXP_BIAS + FP16_EXP_BIAS,
                        mnt >> (FP32_MANT_BITS - FP16_BITS) |
                        !!(mnt & ((1ULL << (FP32_MANT_BITS - FP16_BITS)) - 1)),
                        rounding, (mode & 0xf) | alt_hp << 4, &flags);
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint16_t
fplibConvert(uint64_t op, FPRounding rounding, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn, exp;
    uint64_t mnt;
    uint16_t result;

    // Unpack floating-point operand optionally with flush-to-zero:
    fp64_unpack(&sgn, &exp, &mnt, op, mode, &flags);

    bool alt_hp = fpscr.ahp;

    if (fp64_is_NaN(exp, mnt)) {
        if (alt_hp) {
            result = fp16_zero(sgn);
        } else if (fpscr.dn) {
            result = fp16_defaultNaN();
        } else {
            result = fp16_FPConvertNaN_64(op);
        }
        if (!(mnt >> (FP64_MANT_BITS - 1) & 1) || alt_hp) {
            flags |= FPLIB_IOC;
        }
    } else if (exp == FP64_EXP_INF) {
        if (alt_hp) {
            result = ((uint16_t)sgn << (FP16_BITS - 1) |
                      ((1ULL << (FP16_BITS - 1)) - 1));
            flags |= FPLIB_IOC;
        } else {
            result = fp16_infinity(sgn);
        }
    } else if (!mnt) {
        result = fp16_zero(sgn);
    } else {
        result =
            fp16_round_(sgn, exp - FP64_EXP_BIAS + FP16_EXP_BIAS,
                        mnt >> (FP64_MANT_BITS - FP16_BITS) |
                        !!(mnt & ((1ULL << (FP64_MANT_BITS - FP16_BITS)) - 1)),
                        rounding, (mode & 0xf) | alt_hp << 4, &flags);
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint32_t
fplibConvert(uint16_t op, FPRounding rounding, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn, exp;
    uint16_t mnt;
    uint32_t result;

    // Unpack floating-point operand optionally with flush-to-zero:
    fp16_unpack(&sgn, &exp, &mnt, op, mode & 0xf, &flags);

    if (fp16_is_NaN(exp, mnt) && !fpscr.ahp) {
        if (fpscr.dn) {
            result = fp32_defaultNaN();
        } else {
            result = fp32_FPConvertNaN_16(op);
        }
        if (!(mnt >> (FP16_MANT_BITS - 1) & 1)) {
            flags |= FPLIB_IOC;
        }
    } else if (exp == FP16_EXP_INF && !fpscr.ahp) {
        result = fp32_infinity(sgn);
    } else if (!mnt) {
        result = fp32_zero(sgn);
    } else {
        mnt = fp16_normalise(mnt, &exp);
        result = fp32_pack(sgn, (exp - FP16_EXP_BIAS +
                                 FP32_EXP_BIAS + FP16_EXP_BITS),
                           (uint32_t)mnt << (FP32_MANT_BITS - FP16_BITS + 1));
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint32_t
fplibConvert(uint64_t op, FPRounding rounding, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn, exp;
    uint64_t mnt;
    uint32_t result;

    // Unpack floating-point operand optionally with flush-to-zero:
    fp64_unpack(&sgn, &exp, &mnt, op, mode, &flags);

    if (fp64_is_NaN(exp, mnt)) {
        if (fpscr.dn) {
            result = fp32_defaultNaN();
        } else {
            result = fp32_FPConvertNaN_64(op);
        }
        if (!(mnt >> (FP64_MANT_BITS - 1) & 1)) {
            flags |= FPLIB_IOC;
        }
    } else if (exp == FP64_EXP_INF) {
        result = fp32_infinity(sgn);
    } else if (!mnt) {
        result = fp32_zero(sgn);
    } else {
        result =
            fp32_round_(sgn, exp - FP64_EXP_BIAS + FP32_EXP_BIAS,
                        mnt >> (FP64_MANT_BITS - FP32_BITS) |
                        !!(mnt & ((1ULL << (FP64_MANT_BITS - FP32_BITS)) - 1)),
                        rounding, mode, &flags);
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint64_t
fplibConvert(uint16_t op, FPRounding rounding, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn, exp;
    uint16_t mnt;
    uint64_t result;

    // Unpack floating-point operand optionally with flush-to-zero:
    fp16_unpack(&sgn, &exp, &mnt, op, mode & 0xf, &flags);

    if (fp16_is_NaN(exp, mnt) && !fpscr.ahp) {
        if (fpscr.dn) {
            result = fp64_defaultNaN();
        } else {
            result = fp64_FPConvertNaN_16(op);
        }
        if (!(mnt >> (FP16_MANT_BITS - 1) & 1)) {
            flags |= FPLIB_IOC;
        }
    } else if (exp == FP16_EXP_INF && !fpscr.ahp) {
        result = fp64_infinity(sgn);
    } else if (!mnt) {
        result = fp64_zero(sgn);
    } else {
        mnt = fp16_normalise(mnt, &exp);
        result = fp64_pack(sgn, (exp - FP16_EXP_BIAS +
                                 FP64_EXP_BIAS + FP16_EXP_BITS),
                           (uint64_t)mnt << (FP64_MANT_BITS - FP16_BITS + 1));
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint64_t
fplibConvert(uint32_t op, FPRounding rounding, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn, exp;
    uint32_t mnt;
    uint64_t result;

    // Unpack floating-point operand optionally with flush-to-zero:
    fp32_unpack(&sgn, &exp, &mnt, op, mode, &flags);

    if (fp32_is_NaN(exp, mnt)) {
        if (fpscr.dn) {
            result = fp64_defaultNaN();
        } else {
            result = fp64_FPConvertNaN_32(op);
        }
        if (!(mnt >> (FP32_MANT_BITS - 1) & 1)) {
            flags |= FPLIB_IOC;
        }
    } else if (exp == FP32_EXP_INF) {
        result = fp64_infinity(sgn);
    } else if (!mnt) {
        result = fp64_zero(sgn);
    } else {
        mnt = fp32_normalise(mnt, &exp);
        result = fp64_pack(sgn, (exp - FP32_EXP_BIAS +
                                 FP64_EXP_BIAS + FP32_EXP_BITS),
                           (uint64_t)mnt << (FP64_MANT_BITS - FP32_BITS + 1));
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint16_t
fplibMulAdd(uint16_t addend, uint16_t op1, uint16_t op2, FPSCR &fpscr)
{
    int flags = 0;
    uint16_t result = fp16_muladd(addend, op1, op2, 0, modeConv(fpscr), &flags);
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint32_t
fplibMulAdd(uint32_t addend, uint32_t op1, uint32_t op2, FPSCR &fpscr)
{
    int flags = 0;
    uint32_t result = fp32_muladd(addend, op1, op2, 0, modeConv(fpscr), &flags);
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint64_t
fplibMulAdd(uint64_t addend, uint64_t op1, uint64_t op2, FPSCR &fpscr)
{
    int flags = 0;
    uint64_t result = fp64_muladd(addend, op1, op2, 0, modeConv(fpscr), &flags);
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint16_t
fplibDiv(uint16_t op1, uint16_t op2, FPSCR &fpscr)
{
    int flags = 0;
    uint16_t result = fp16_div(op1, op2, modeConv(fpscr), &flags);
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint32_t
fplibDiv(uint32_t op1, uint32_t op2, FPSCR &fpscr)
{
    int flags = 0;
    uint32_t result = fp32_div(op1, op2, modeConv(fpscr), &flags);
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint64_t
fplibDiv(uint64_t op1, uint64_t op2, FPSCR &fpscr)
{
    int flags = 0;
    uint64_t result = fp64_div(op1, op2, modeConv(fpscr), &flags);
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint16_t
fplibExpA(uint16_t op)
{
    static uint16_t coeff[32] = {
        0x0000,
        0x0016,
        0x002d,
        0x0045,
        0x005d,
        0x0075,
        0x008e,
        0x00a8,
        0x00c2,
        0x00dc,
        0x00f8,
        0x0114,
        0x0130,
        0x014d,
        0x016b,
        0x0189,
        0x01a8,
        0x01c8,
        0x01e8,
        0x0209,
        0x022b,
        0x024e,
        0x0271,
        0x0295,
        0x02ba,
        0x02e0,
        0x0306,
        0x032e,
        0x0356,
        0x037f,
        0x03a9,
        0x03d4
    };
    return ((((op >> 5) & ((1 << FP16_EXP_BITS) - 1)) << FP16_MANT_BITS) |
            coeff[op & ((1 << 5) - 1)]);
}

template <>
uint32_t
fplibExpA(uint32_t op)
{
    static uint32_t coeff[64] = {
        0x000000,
        0x0164d2,
        0x02cd87,
        0x043a29,
        0x05aac3,
        0x071f62,
        0x08980f,
        0x0a14d5,
        0x0b95c2,
        0x0d1adf,
        0x0ea43a,
        0x1031dc,
        0x11c3d3,
        0x135a2b,
        0x14f4f0,
        0x16942d,
        0x1837f0,
        0x19e046,
        0x1b8d3a,
        0x1d3eda,
        0x1ef532,
        0x20b051,
        0x227043,
        0x243516,
        0x25fed7,
        0x27cd94,
        0x29a15b,
        0x2b7a3a,
        0x2d583f,
        0x2f3b79,
        0x3123f6,
        0x3311c4,
        0x3504f3,
        0x36fd92,
        0x38fbaf,
        0x3aff5b,
        0x3d08a4,
        0x3f179a,
        0x412c4d,
        0x4346cd,
        0x45672a,
        0x478d75,
        0x49b9be,
        0x4bec15,
        0x4e248c,
        0x506334,
        0x52a81e,
        0x54f35b,
        0x5744fd,
        0x599d16,
        0x5bfbb8,
        0x5e60f5,
        0x60ccdf,
        0x633f89,
        0x65b907,
        0x68396a,
        0x6ac0c7,
        0x6d4f30,
        0x6fe4ba,
        0x728177,
        0x75257d,
        0x77d0df,
        0x7a83b3,
        0x7d3e0c
    };
    return ((((op >> 6) & ((1 << FP32_EXP_BITS) - 1)) << FP32_MANT_BITS) |
            coeff[op & ((1 << 6) - 1)]);
}

template <>
uint64_t
fplibExpA(uint64_t op)
{
    static uint64_t coeff[64] = {
        0x0000000000000ULL,
        0x02c9a3e778061ULL,
        0x059b0d3158574ULL,
        0x0874518759bc8ULL,
        0x0b5586cf9890fULL,
        0x0e3ec32d3d1a2ULL,
        0x11301d0125b51ULL,
        0x1429aaea92de0ULL,
        0x172b83c7d517bULL,
        0x1a35beb6fcb75ULL,
        0x1d4873168b9aaULL,
        0x2063b88628cd6ULL,
        0x2387a6e756238ULL,
        0x26b4565e27cddULL,
        0x29e9df51fdee1ULL,
        0x2d285a6e4030bULL,
        0x306fe0a31b715ULL,
        0x33c08b26416ffULL,
        0x371a7373aa9cbULL,
        0x3a7db34e59ff7ULL,
        0x3dea64c123422ULL,
        0x4160a21f72e2aULL,
        0x44e086061892dULL,
        0x486a2b5c13cd0ULL,
        0x4bfdad5362a27ULL,
        0x4f9b2769d2ca7ULL,
        0x5342b569d4f82ULL,
        0x56f4736b527daULL,
        0x5ab07dd485429ULL,
        0x5e76f15ad2148ULL,
        0x6247eb03a5585ULL,
        0x6623882552225ULL,
        0x6a09e667f3bcdULL,
        0x6dfb23c651a2fULL,
        0x71f75e8ec5f74ULL,
        0x75feb564267c9ULL,
        0x7a11473eb0187ULL,
        0x7e2f336cf4e62ULL,
        0x82589994cce13ULL,
        0x868d99b4492edULL,
        0x8ace5422aa0dbULL,
        0x8f1ae99157736ULL,
        0x93737b0cdc5e5ULL,
        0x97d829fde4e50ULL,
        0x9c49182a3f090ULL,
        0xa0c667b5de565ULL,
        0xa5503b23e255dULL,
        0xa9e6b5579fdbfULL,
        0xae89f995ad3adULL,
        0xb33a2b84f15fbULL,
        0xb7f76f2fb5e47ULL,
        0xbcc1e904bc1d2ULL,
        0xc199bdd85529cULL,
        0xc67f12e57d14bULL,
        0xcb720dcef9069ULL,
        0xd072d4a07897cULL,
        0xd5818dcfba487ULL,
        0xda9e603db3285ULL,
        0xdfc97337b9b5fULL,
        0xe502ee78b3ff6ULL,
        0xea4afa2a490daULL,
        0xefa1bee615a27ULL,
        0xf50765b6e4540ULL,
        0xfa7c1819e90d8ULL
    };
    return ((((op >> 6) & ((1 << FP64_EXP_BITS) - 1)) << FP64_MANT_BITS) |
            coeff[op & ((1 << 6) - 1)]);
}

static uint16_t
fp16_repack(int sgn, int exp, uint16_t mnt)
{
    return fp16_pack(sgn, mnt >> FP16_MANT_BITS ? exp : 0, mnt);
}

static uint32_t
fp32_repack(int sgn, int exp, uint32_t mnt)
{
    return fp32_pack(sgn, mnt >> FP32_MANT_BITS ? exp : 0, mnt);
}

static uint64_t
fp64_repack(int sgn, int exp, uint64_t mnt)
{
    return fp64_pack(sgn, mnt >> FP64_MANT_BITS ? exp : 0, mnt);
}

static void
fp16_minmaxnum(uint16_t *op1, uint16_t *op2, int sgn)
{
    // Treat a single quiet-NaN as +Infinity/-Infinity
    if (!((uint16_t)~(*op1 << 1) >> FP16_MANT_BITS) &&
        (uint16_t)~(*op2 << 1) >> FP16_MANT_BITS)
        *op1 = fp16_infinity(sgn);
    if (!((uint16_t)~(*op2 << 1) >> FP16_MANT_BITS) &&
        (uint16_t)~(*op1 << 1) >> FP16_MANT_BITS)
        *op2 = fp16_infinity(sgn);
}

static void
fp32_minmaxnum(uint32_t *op1, uint32_t *op2, int sgn)
{
    // Treat a single quiet-NaN as +Infinity/-Infinity
    if (!((uint32_t)~(*op1 << 1) >> FP32_MANT_BITS) &&
        (uint32_t)~(*op2 << 1) >> FP32_MANT_BITS)
        *op1 = fp32_infinity(sgn);
    if (!((uint32_t)~(*op2 << 1) >> FP32_MANT_BITS) &&
        (uint32_t)~(*op1 << 1) >> FP32_MANT_BITS)
        *op2 = fp32_infinity(sgn);
}

static void
fp64_minmaxnum(uint64_t *op1, uint64_t *op2, int sgn)
{
    // Treat a single quiet-NaN as +Infinity/-Infinity
    if (!((uint64_t)~(*op1 << 1) >> FP64_MANT_BITS) &&
        (uint64_t)~(*op2 << 1) >> FP64_MANT_BITS)
        *op1 = fp64_infinity(sgn);
    if (!((uint64_t)~(*op2 << 1) >> FP64_MANT_BITS) &&
        (uint64_t)~(*op1 << 1) >> FP64_MANT_BITS)
        *op2 = fp64_infinity(sgn);
}

template <>
uint16_t
fplibMax(uint16_t op1, uint16_t op2, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn1, exp1, sgn2, exp2;
    uint16_t mnt1, mnt2, x, result;

    fp16_unpack(&sgn1, &exp1, &mnt1, op1, mode, &flags);
    fp16_unpack(&sgn2, &exp2, &mnt2, op2, mode, &flags);

    if ((x = fp16_process_NaNs(op1, op2, mode, &flags))) {
        result = x;
    } else {
        result = ((sgn1 != sgn2 ? sgn2 : sgn1 ^ (op1 > op2)) ?
                  fp16_repack(sgn1, exp1, mnt1) :
                  fp16_repack(sgn2, exp2, mnt2));
    }
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint32_t
fplibMax(uint32_t op1, uint32_t op2, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn1, exp1, sgn2, exp2;
    uint32_t mnt1, mnt2, x, result;

    fp32_unpack(&sgn1, &exp1, &mnt1, op1, mode, &flags);
    fp32_unpack(&sgn2, &exp2, &mnt2, op2, mode, &flags);

    if ((x = fp32_process_NaNs(op1, op2, mode, &flags))) {
        result = x;
    } else {
        result = ((sgn1 != sgn2 ? sgn2 : sgn1 ^ (op1 > op2)) ?
                  fp32_repack(sgn1, exp1, mnt1) :
                  fp32_repack(sgn2, exp2, mnt2));
    }
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint64_t
fplibMax(uint64_t op1, uint64_t op2, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn1, exp1, sgn2, exp2;
    uint64_t mnt1, mnt2, x, result;

    fp64_unpack(&sgn1, &exp1, &mnt1, op1, mode, &flags);
    fp64_unpack(&sgn2, &exp2, &mnt2, op2, mode, &flags);

    if ((x = fp64_process_NaNs(op1, op2, mode, &flags))) {
        result = x;
    } else {
        result = ((sgn1 != sgn2 ? sgn2 : sgn1 ^ (op1 > op2)) ?
                  fp64_repack(sgn1, exp1, mnt1) :
                  fp64_repack(sgn2, exp2, mnt2));
    }
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint16_t
fplibMaxNum(uint16_t op1, uint16_t op2, FPSCR &fpscr)
{
    fp16_minmaxnum(&op1, &op2, 1);
    return fplibMax<uint16_t>(op1, op2, fpscr);
}

template <>
uint32_t
fplibMaxNum(uint32_t op1, uint32_t op2, FPSCR &fpscr)
{
    fp32_minmaxnum(&op1, &op2, 1);
    return fplibMax<uint32_t>(op1, op2, fpscr);
}

template <>
uint64_t
fplibMaxNum(uint64_t op1, uint64_t op2, FPSCR &fpscr)
{
    fp64_minmaxnum(&op1, &op2, 1);
    return fplibMax<uint64_t>(op1, op2, fpscr);
}

template <>
uint16_t
fplibMin(uint16_t op1, uint16_t op2, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn1, exp1, sgn2, exp2;
    uint16_t mnt1, mnt2, x, result;

    fp16_unpack(&sgn1, &exp1, &mnt1, op1, mode, &flags);
    fp16_unpack(&sgn2, &exp2, &mnt2, op2, mode, &flags);

    if ((x = fp16_process_NaNs(op1, op2, mode, &flags))) {
        result = x;
    } else {
        result = ((sgn1 != sgn2 ? sgn1 : sgn1 ^ (op1 < op2)) ?
                  fp16_repack(sgn1, exp1, mnt1) :
                  fp16_repack(sgn2, exp2, mnt2));
    }
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint32_t
fplibMin(uint32_t op1, uint32_t op2, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn1, exp1, sgn2, exp2;
    uint32_t mnt1, mnt2, x, result;

    fp32_unpack(&sgn1, &exp1, &mnt1, op1, mode, &flags);
    fp32_unpack(&sgn2, &exp2, &mnt2, op2, mode, &flags);

    if ((x = fp32_process_NaNs(op1, op2, mode, &flags))) {
        result = x;
    } else {
        result = ((sgn1 != sgn2 ? sgn1 : sgn1 ^ (op1 < op2)) ?
                  fp32_repack(sgn1, exp1, mnt1) :
                  fp32_repack(sgn2, exp2, mnt2));
    }
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint64_t
fplibMin(uint64_t op1, uint64_t op2, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn1, exp1, sgn2, exp2;
    uint64_t mnt1, mnt2, x, result;

    fp64_unpack(&sgn1, &exp1, &mnt1, op1, mode, &flags);
    fp64_unpack(&sgn2, &exp2, &mnt2, op2, mode, &flags);

    if ((x = fp64_process_NaNs(op1, op2, mode, &flags))) {
        result = x;
    } else {
        result = ((sgn1 != sgn2 ? sgn1 : sgn1 ^ (op1 < op2)) ?
                  fp64_repack(sgn1, exp1, mnt1) :
                  fp64_repack(sgn2, exp2, mnt2));
    }
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint16_t
fplibMinNum(uint16_t op1, uint16_t op2, FPSCR &fpscr)
{
    fp16_minmaxnum(&op1, &op2, 0);
    return fplibMin<uint16_t>(op1, op2, fpscr);
}

template <>
uint32_t
fplibMinNum(uint32_t op1, uint32_t op2, FPSCR &fpscr)
{
    fp32_minmaxnum(&op1, &op2, 0);
    return fplibMin<uint32_t>(op1, op2, fpscr);
}

template <>
uint64_t
fplibMinNum(uint64_t op1, uint64_t op2, FPSCR &fpscr)
{
    fp64_minmaxnum(&op1, &op2, 0);
    return fplibMin<uint64_t>(op1, op2, fpscr);
}

template <>
uint16_t
fplibMul(uint16_t op1, uint16_t op2, FPSCR &fpscr)
{
    int flags = 0;
    uint16_t result = fp16_mul(op1, op2, modeConv(fpscr), &flags);
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint32_t
fplibMul(uint32_t op1, uint32_t op2, FPSCR &fpscr)
{
    int flags = 0;
    uint32_t result = fp32_mul(op1, op2, modeConv(fpscr), &flags);
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint64_t
fplibMul(uint64_t op1, uint64_t op2, FPSCR &fpscr)
{
    int flags = 0;
    uint64_t result = fp64_mul(op1, op2, modeConv(fpscr), &flags);
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint16_t
fplibMulX(uint16_t op1, uint16_t op2, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn1, exp1, sgn2, exp2;
    uint16_t mnt1, mnt2, result;

    fp16_unpack(&sgn1, &exp1, &mnt1, op1, mode, &flags);
    fp16_unpack(&sgn2, &exp2, &mnt2, op2, mode, &flags);

    result = fp16_process_NaNs(op1, op2, mode, &flags);
    if (!result) {
        if ((exp1 == FP16_EXP_INF && !mnt2) ||
            (exp2 == FP16_EXP_INF && !mnt1)) {
            result = fp16_FPTwo(sgn1 ^ sgn2);
        } else if (exp1 == FP16_EXP_INF || exp2 == FP16_EXP_INF) {
            result = fp16_infinity(sgn1 ^ sgn2);
        } else if (!mnt1 || !mnt2) {
            result = fp16_zero(sgn1 ^ sgn2);
        } else {
            result = fp16_mul(op1, op2, mode, &flags);
        }
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint32_t
fplibMulX(uint32_t op1, uint32_t op2, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn1, exp1, sgn2, exp2;
    uint32_t mnt1, mnt2, result;

    fp32_unpack(&sgn1, &exp1, &mnt1, op1, mode, &flags);
    fp32_unpack(&sgn2, &exp2, &mnt2, op2, mode, &flags);

    result = fp32_process_NaNs(op1, op2, mode, &flags);
    if (!result) {
        if ((exp1 == FP32_EXP_INF && !mnt2) ||
            (exp2 == FP32_EXP_INF && !mnt1)) {
            result = fp32_FPTwo(sgn1 ^ sgn2);
        } else if (exp1 == FP32_EXP_INF || exp2 == FP32_EXP_INF) {
            result = fp32_infinity(sgn1 ^ sgn2);
        } else if (!mnt1 || !mnt2) {
            result = fp32_zero(sgn1 ^ sgn2);
        } else {
            result = fp32_mul(op1, op2, mode, &flags);
        }
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint64_t
fplibMulX(uint64_t op1, uint64_t op2, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn1, exp1, sgn2, exp2;
    uint64_t mnt1, mnt2, result;

    fp64_unpack(&sgn1, &exp1, &mnt1, op1, mode, &flags);
    fp64_unpack(&sgn2, &exp2, &mnt2, op2, mode, &flags);

    result = fp64_process_NaNs(op1, op2, mode, &flags);
    if (!result) {
        if ((exp1 == FP64_EXP_INF && !mnt2) ||
            (exp2 == FP64_EXP_INF && !mnt1)) {
            result = fp64_FPTwo(sgn1 ^ sgn2);
        } else if (exp1 == FP64_EXP_INF || exp2 == FP64_EXP_INF) {
            result = fp64_infinity(sgn1 ^ sgn2);
        } else if (!mnt1 || !mnt2) {
            result = fp64_zero(sgn1 ^ sgn2);
        } else {
            result = fp64_mul(op1, op2, mode, &flags);
        }
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint16_t
fplibNeg(uint16_t op)
{
    return op ^ 1ULL << (FP16_BITS - 1);
}

template <>
uint32_t
fplibNeg(uint32_t op)
{
    return op ^ 1ULL << (FP32_BITS - 1);
}

template <>
uint64_t
fplibNeg(uint64_t op)
{
    return op ^ 1ULL << (FP64_BITS - 1);
}

static const uint8_t recip_sqrt_estimate[256] = {
    255, 253, 251, 249, 247, 245, 243, 242, 240, 238, 236, 234, 233, 231, 229, 228,
    226, 224, 223, 221, 219, 218, 216, 215, 213, 212, 210, 209, 207, 206, 204, 203,
    201, 200, 198, 197, 196, 194, 193, 192, 190, 189, 188, 186, 185, 184, 183, 181,
    180, 179, 178, 176, 175, 174, 173, 172, 170, 169, 168, 167, 166, 165, 164, 163,
    162, 160, 159, 158, 157, 156, 155, 154, 153, 152, 151, 150, 149, 148, 147, 146,
    145, 144, 143, 142, 141, 140, 140, 139, 138, 137, 136, 135, 134, 133, 132, 131,
    131, 130, 129, 128, 127, 126, 126, 125, 124, 123, 122, 121, 121, 120, 119, 118,
    118, 117, 116, 115, 114, 114, 113, 112, 111, 111, 110, 109, 109, 108, 107, 106,
    105, 104, 103, 101, 100,  99,  97,  96,  95,  93,  92,  91,  90,  88,  87,  86,
    85,  84,  82,  81,  80,  79,  78,  77,  76,  75,  74,  72,  71,  70,  69,  68,
    67,  66,  65,  64,  63,  62,  61,  60,  60,  59,  58,  57,  56,  55,  54,  53,
    52,  51,  51,  50,  49,  48,  47,  46,  46,  45,  44,  43,  42,  42,  41,  40,
    39,  38,  38,  37,  36,  35,  35,  34,  33,  33,  32,  31,  30,  30,  29,  28,
    28,  27,  26,  26,  25,  24,  24,  23,  22,  22,  21,  20,  20,  19,  19,  18,
    17,  17,  16,  16,  15,  14,  14,  13,  13,  12,  11,  11,  10,  10,   9,   9,
    8,   8,   7,   6,   6,   5,   5,   4,   4,   3,   3,   2,   2,   1,   1,   0
};

template <>
uint16_t
fplibRSqrtEstimate(uint16_t op, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn, exp;
    uint16_t mnt, result;

    fp16_unpack(&sgn, &exp, &mnt, op, mode, &flags);

    if (fp16_is_NaN(exp, mnt)) {
        result = fp16_process_NaN(op, mode, &flags);
    } else if (!mnt) {
        result = fp16_infinity(sgn);
        flags |= FPLIB_DZC;
    } else if (sgn) {
        result = fp16_defaultNaN();
        flags |= FPLIB_IOC;
    } else if (exp == FP16_EXP_INF) {
        result = fp16_zero(0);
    } else {
        exp += FP16_EXP_BITS;
        mnt = fp16_normalise(mnt, &exp);
        mnt = recip_sqrt_estimate[(~exp & 1) << 7 |
                                  (mnt >> (FP16_BITS - 8) & 127)];
        result = fp16_pack(0, (3 * FP16_EXP_BIAS - exp - 1) >> 1,
                           mnt << (FP16_MANT_BITS - 8));
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint32_t
fplibRSqrtEstimate(uint32_t op, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn, exp;
    uint32_t mnt, result;

    fp32_unpack(&sgn, &exp, &mnt, op, mode, &flags);

    if (fp32_is_NaN(exp, mnt)) {
        result = fp32_process_NaN(op, mode, &flags);
    } else if (!mnt) {
        result = fp32_infinity(sgn);
        flags |= FPLIB_DZC;
    } else if (sgn) {
        result = fp32_defaultNaN();
        flags |= FPLIB_IOC;
    } else if (exp == FP32_EXP_INF) {
        result = fp32_zero(0);
    } else {
        exp += FP32_EXP_BITS;
        mnt = fp32_normalise(mnt, &exp);
        mnt = recip_sqrt_estimate[(~exp & 1) << 7 |
                                  (mnt >> (FP32_BITS - 8) & 127)];
        result = fp32_pack(0, (3 * FP32_EXP_BIAS - exp - 1) >> 1,
                           mnt << (FP32_MANT_BITS - 8));
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint64_t
fplibRSqrtEstimate(uint64_t op, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn, exp;
    uint64_t mnt, result;

    fp64_unpack(&sgn, &exp, &mnt, op, mode, &flags);

    if (fp64_is_NaN(exp, mnt)) {
        result = fp64_process_NaN(op, mode, &flags);
    } else if (!mnt) {
        result = fp64_infinity(sgn);
        flags |= FPLIB_DZC;
    } else if (sgn) {
        result = fp64_defaultNaN();
        flags |= FPLIB_IOC;
    } else if (exp == FP64_EXP_INF) {
        result = fp32_zero(0);
    } else {
        exp += FP64_EXP_BITS;
        mnt = fp64_normalise(mnt, &exp);
        mnt = recip_sqrt_estimate[(~exp & 1) << 7 |
                                  (mnt >> (FP64_BITS - 8) & 127)];
        result = fp64_pack(0, (3 * FP64_EXP_BIAS - exp - 1) >> 1,
                           mnt << (FP64_MANT_BITS - 8));
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint16_t
fplibRSqrtStepFused(uint16_t op1, uint16_t op2, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn1, exp1, sgn2, exp2;
    uint16_t mnt1, mnt2, result;

    op1 = fplibNeg<uint16_t>(op1);
    fp16_unpack(&sgn1, &exp1, &mnt1, op1, mode, &flags);
    fp16_unpack(&sgn2, &exp2, &mnt2, op2, mode, &flags);

    result = fp16_process_NaNs(op1, op2, mode, &flags);
    if (!result) {
        if ((exp1 == FP16_EXP_INF && !mnt2) ||
            (exp2 == FP16_EXP_INF && !mnt1)) {
            result = fp16_FPOnePointFive(0);
        } else if (exp1 == FP16_EXP_INF || exp2 == FP16_EXP_INF) {
            result = fp16_infinity(sgn1 ^ sgn2);
        } else {
            result = fp16_muladd(fp16_FPThree(0), op1, op2, -1, mode, &flags);
        }
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint32_t
fplibRSqrtStepFused(uint32_t op1, uint32_t op2, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn1, exp1, sgn2, exp2;
    uint32_t mnt1, mnt2, result;

    op1 = fplibNeg<uint32_t>(op1);
    fp32_unpack(&sgn1, &exp1, &mnt1, op1, mode, &flags);
    fp32_unpack(&sgn2, &exp2, &mnt2, op2, mode, &flags);

    result = fp32_process_NaNs(op1, op2, mode, &flags);
    if (!result) {
        if ((exp1 == FP32_EXP_INF && !mnt2) ||
            (exp2 == FP32_EXP_INF && !mnt1)) {
            result = fp32_FPOnePointFive(0);
        } else if (exp1 == FP32_EXP_INF || exp2 == FP32_EXP_INF) {
            result = fp32_infinity(sgn1 ^ sgn2);
        } else {
            result = fp32_muladd(fp32_FPThree(0), op1, op2, -1, mode, &flags);
        }
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint64_t
fplibRSqrtStepFused(uint64_t op1, uint64_t op2, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn1, exp1, sgn2, exp2;
    uint64_t mnt1, mnt2, result;

    op1 = fplibNeg<uint64_t>(op1);
    fp64_unpack(&sgn1, &exp1, &mnt1, op1, mode, &flags);
    fp64_unpack(&sgn2, &exp2, &mnt2, op2, mode, &flags);

    result = fp64_process_NaNs(op1, op2, mode, &flags);
    if (!result) {
        if ((exp1 == FP64_EXP_INF && !mnt2) ||
            (exp2 == FP64_EXP_INF && !mnt1)) {
            result = fp64_FPOnePointFive(0);
        } else if (exp1 == FP64_EXP_INF || exp2 == FP64_EXP_INF) {
            result = fp64_infinity(sgn1 ^ sgn2);
        } else {
            result = fp64_muladd(fp64_FPThree(0), op1, op2, -1, mode, &flags);
        }
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint16_t
fplibRecipEstimate(uint16_t op, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn, exp;
    uint16_t mnt, result;

    fp16_unpack(&sgn, &exp, &mnt, op, mode, &flags);

    if (fp16_is_NaN(exp, mnt)) {
        result = fp16_process_NaN(op, mode, &flags);
    } else if (exp == FP16_EXP_INF) {
        result = fp16_zero(sgn);
    } else if (!mnt) {
        result = fp16_infinity(sgn);
        flags |= FPLIB_DZC;
    } else if (!((uint16_t)(op << 1) >> (FP16_MANT_BITS - 1))) {
        bool overflow_to_inf = false;
        switch (FPCRRounding(fpscr)) {
          case FPRounding_TIEEVEN:
            overflow_to_inf = true;
            break;
          case FPRounding_POSINF:
            overflow_to_inf = !sgn;
            break;
          case FPRounding_NEGINF:
            overflow_to_inf = sgn;
            break;
          case FPRounding_ZERO:
            overflow_to_inf = false;
            break;
          default:
            panic("Unrecognized FP rounding mode");
        }
        result = overflow_to_inf ? fp16_infinity(sgn) : fp16_max_normal(sgn);
        flags |= FPLIB_OFC | FPLIB_IXC;
    } else if (fpscr.fz16 && exp >= 2 * FP16_EXP_BIAS - 1) {
        result = fp16_zero(sgn);
        flags |= FPLIB_UFC;
    } else {
        exp += FP16_EXP_BITS;
        mnt = fp16_normalise(mnt, &exp);
        int result_exp = 2 * FP16_EXP_BIAS - 1 - exp;
        uint16_t fraction = (((uint32_t)1 << 19) /
                             (mnt >> (FP16_BITS - 10) | 1) + 1) >> 1;
        fraction <<= FP16_MANT_BITS - 8;
        if (result_exp == 0) {
            fraction >>= 1;
        } else if (result_exp == -1) {
            fraction >>= 2;
            result_exp = 0;
        }
        result = fp16_pack(sgn, result_exp, fraction);
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint32_t
fplibRecipEstimate(uint32_t op, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn, exp;
    uint32_t mnt, result;

    fp32_unpack(&sgn, &exp, &mnt, op, mode, &flags);

    if (fp32_is_NaN(exp, mnt)) {
        result = fp32_process_NaN(op, mode, &flags);
    } else if (exp == FP32_EXP_INF) {
        result = fp32_zero(sgn);
    } else if (!mnt) {
        result = fp32_infinity(sgn);
        flags |= FPLIB_DZC;
    } else if (!((uint32_t)(op << 1) >> (FP32_MANT_BITS - 1))) {
        bool overflow_to_inf = false;
        switch (FPCRRounding(fpscr)) {
          case FPRounding_TIEEVEN:
            overflow_to_inf = true;
            break;
          case FPRounding_POSINF:
            overflow_to_inf = !sgn;
            break;
          case FPRounding_NEGINF:
            overflow_to_inf = sgn;
            break;
          case FPRounding_ZERO:
            overflow_to_inf = false;
            break;
          default:
            panic("Unrecognized FP rounding mode");
        }
        result = overflow_to_inf ? fp32_infinity(sgn) : fp32_max_normal(sgn);
        flags |= FPLIB_OFC | FPLIB_IXC;
    } else if (fpscr.fz && exp >= 2 * FP32_EXP_BIAS - 1) {
        result = fp32_zero(sgn);
        flags |= FPLIB_UFC;
    } else {
        exp += FP32_EXP_BITS;
        mnt = fp32_normalise(mnt, &exp);
        int result_exp = 2 * FP32_EXP_BIAS - 1 - exp;
        uint32_t fraction = (((uint32_t)1 << 19) /
                             (mnt >> (FP32_BITS - 10) | 1) + 1) >> 1;
        fraction <<= FP32_MANT_BITS - 8;
        if (result_exp == 0) {
            fraction >>= 1;
        } else if (result_exp == -1) {
            fraction >>= 2;
            result_exp = 0;
        }
        result = fp32_pack(sgn, result_exp, fraction);
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint64_t
fplibRecipEstimate(uint64_t op, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn, exp;
    uint64_t mnt, result;

    fp64_unpack(&sgn, &exp, &mnt, op, mode, &flags);

    if (fp64_is_NaN(exp, mnt)) {
        result = fp64_process_NaN(op, mode, &flags);
    } else if (exp == FP64_EXP_INF) {
        result = fp64_zero(sgn);
    } else if (!mnt) {
        result = fp64_infinity(sgn);
        flags |= FPLIB_DZC;
    } else if (!((uint64_t)(op << 1) >> (FP64_MANT_BITS - 1))) {
        bool overflow_to_inf = false;
        switch (FPCRRounding(fpscr)) {
          case FPRounding_TIEEVEN:
            overflow_to_inf = true;
            break;
          case FPRounding_POSINF:
            overflow_to_inf = !sgn;
            break;
          case FPRounding_NEGINF:
            overflow_to_inf = sgn;
            break;
          case FPRounding_ZERO:
            overflow_to_inf = false;
            break;
          default:
            panic("Unrecognized FP rounding mode");
        }
        result = overflow_to_inf ? fp64_infinity(sgn) : fp64_max_normal(sgn);
        flags |= FPLIB_OFC | FPLIB_IXC;
    } else if (fpscr.fz && exp >= 2 * FP64_EXP_BIAS - 1) {
        result = fp64_zero(sgn);
        flags |= FPLIB_UFC;
    } else {
        exp += FP64_EXP_BITS;
        mnt = fp64_normalise(mnt, &exp);
        int result_exp = 2 * FP64_EXP_BIAS - 1 - exp;
        uint64_t fraction = (((uint32_t)1 << 19) /
                             (mnt >> (FP64_BITS - 10) | 1) + 1) >> 1;
        fraction <<= FP64_MANT_BITS - 8;
        if (result_exp == 0) {
            fraction >>= 1;
        } else if (result_exp == -1) {
            fraction >>= 2;
            result_exp = 0;
        }
        result = fp64_pack(sgn, result_exp, fraction);
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint16_t
fplibRecipStepFused(uint16_t op1, uint16_t op2, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn1, exp1, sgn2, exp2;
    uint16_t mnt1, mnt2, result;

    op1 = fplibNeg<uint16_t>(op1);
    fp16_unpack(&sgn1, &exp1, &mnt1, op1, mode, &flags);
    fp16_unpack(&sgn2, &exp2, &mnt2, op2, mode, &flags);

    result = fp16_process_NaNs(op1, op2, mode, &flags);
    if (!result) {
        if ((exp1 == FP16_EXP_INF && !mnt2) ||
            (exp2 == FP16_EXP_INF && !mnt1)) {
            result = fp16_FPTwo(0);
        } else if (exp1 == FP16_EXP_INF || exp2 == FP16_EXP_INF) {
            result = fp16_infinity(sgn1 ^ sgn2);
        } else {
            result = fp16_muladd(fp16_FPTwo(0), op1, op2, 0, mode, &flags);
        }
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint32_t
fplibRecipStepFused(uint32_t op1, uint32_t op2, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn1, exp1, sgn2, exp2;
    uint32_t mnt1, mnt2, result;

    op1 = fplibNeg<uint32_t>(op1);
    fp32_unpack(&sgn1, &exp1, &mnt1, op1, mode, &flags);
    fp32_unpack(&sgn2, &exp2, &mnt2, op2, mode, &flags);

    result = fp32_process_NaNs(op1, op2, mode, &flags);
    if (!result) {
        if ((exp1 == FP32_EXP_INF && !mnt2) ||
            (exp2 == FP32_EXP_INF && !mnt1)) {
            result = fp32_FPTwo(0);
        } else if (exp1 == FP32_EXP_INF || exp2 == FP32_EXP_INF) {
            result = fp32_infinity(sgn1 ^ sgn2);
        } else {
            result = fp32_muladd(fp32_FPTwo(0), op1, op2, 0, mode, &flags);
        }
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint64_t
fplibRecipStepFused(uint64_t op1, uint64_t op2, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn1, exp1, sgn2, exp2;
    uint64_t mnt1, mnt2, result;

    op1 = fplibNeg<uint64_t>(op1);
    fp64_unpack(&sgn1, &exp1, &mnt1, op1, mode, &flags);
    fp64_unpack(&sgn2, &exp2, &mnt2, op2, mode, &flags);

    result = fp64_process_NaNs(op1, op2, mode, &flags);
    if (!result) {
        if ((exp1 == FP64_EXP_INF && !mnt2) ||
            (exp2 == FP64_EXP_INF && !mnt1)) {
            result = fp64_FPTwo(0);
        } else if (exp1 == FP64_EXP_INF || exp2 == FP64_EXP_INF) {
            result = fp64_infinity(sgn1 ^ sgn2);
        } else {
            result = fp64_muladd(fp64_FPTwo(0), op1, op2, 0, mode, &flags);
        }
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint16_t
fplibRecpX(uint16_t op, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn, exp;
    uint16_t mnt, result;

    fp16_unpack(&sgn, &exp, &mnt, op, mode, &flags);

    if (fp16_is_NaN(exp, mnt)) {
        result = fp16_process_NaN(op, mode, &flags);
    }
    else {
        if (!mnt) { // Zero and denormals
            result = fp16_pack(sgn, FP16_EXP_INF - 1, 0);
        } else { // Infinities and normals
            result = fp16_pack(sgn, exp ^ FP16_EXP_INF, 0);
        }
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint32_t
fplibRecpX(uint32_t op, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn, exp;
    uint32_t mnt, result;

    fp32_unpack(&sgn, &exp, &mnt, op, mode, &flags);

    if (fp32_is_NaN(exp, mnt)) {
        result = fp32_process_NaN(op, mode, &flags);
    }
    else {
        if (!mnt) { // Zero and denormals
            result = fp32_pack(sgn, FP32_EXP_INF - 1, 0);
        } else { // Infinities and normals
            result = fp32_pack(sgn, exp ^ FP32_EXP_INF, 0);
        }
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint64_t
fplibRecpX(uint64_t op, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn, exp;
    uint64_t mnt, result;

    fp64_unpack(&sgn, &exp, &mnt, op, mode, &flags);

    if (fp64_is_NaN(exp, mnt)) {
        result = fp64_process_NaN(op, mode, &flags);
    }
    else {
        if (!mnt) { // Zero and denormals
            result = fp64_pack(sgn, FP64_EXP_INF - 1, 0);
        } else { // Infinities and normals
            result = fp64_pack(sgn, exp ^ FP64_EXP_INF, 0);
        }
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint16_t
fplibRoundInt(uint16_t op, FPRounding rounding, bool exact, FPSCR &fpscr)
{
    int expint = FP16_EXP_BIAS + FP16_MANT_BITS;
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn, exp;
    uint16_t mnt, result;

    // Unpack using FPCR to determine if subnormals are flushed-to-zero:
    fp16_unpack(&sgn, &exp, &mnt, op, mode, &flags);

    // Handle NaNs, infinities and zeroes:
    if (fp16_is_NaN(exp, mnt)) {
        result = fp16_process_NaN(op, mode, &flags);
    } else if (exp == FP16_EXP_INF) {
        result = fp16_infinity(sgn);
    } else if (!mnt) {
        result = fp16_zero(sgn);
    } else if (exp >= expint) {
        // There are no fractional bits
        result = op;
    } else {
        // Truncate towards zero:
        uint16_t x = expint - exp >= FP16_BITS ? 0 : mnt >> (expint - exp);
        int err = exp < expint - FP16_BITS ? 1 :
            ((mnt << 1 >> (expint - exp - 1) & 3) |
             ((uint16_t)(mnt << 2 << (FP16_BITS + exp - expint)) != 0));
        switch (rounding) {
          case FPRounding_TIEEVEN:
            x += (err == 3 || (err == 2 && (x & 1)));
            break;
          case FPRounding_POSINF:
            x += err && !sgn;
            break;
          case FPRounding_NEGINF:
            x += err && sgn;
            break;
          case FPRounding_ZERO:
            break;
          case FPRounding_TIEAWAY:
            x += err >> 1;
            break;
          default:
            panic("Unrecognized FP rounding mode");
        }

        if (x == 0) {
            result = fp16_zero(sgn);
        } else {
            exp = expint;
            mnt = fp16_normalise(x, &exp);
            result = fp16_pack(sgn, exp + FP16_EXP_BITS, mnt >> FP16_EXP_BITS);
        }

        if (err && exact)
            flags |= FPLIB_IXC;
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint32_t
fplibRoundInt(uint32_t op, FPRounding rounding, bool exact, FPSCR &fpscr)
{
    int expint = FP32_EXP_BIAS + FP32_MANT_BITS;
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn, exp;
    uint32_t mnt, result;

    // Unpack using FPCR to determine if subnormals are flushed-to-zero:
    fp32_unpack(&sgn, &exp, &mnt, op, mode, &flags);

    // Handle NaNs, infinities and zeroes:
    if (fp32_is_NaN(exp, mnt)) {
        result = fp32_process_NaN(op, mode, &flags);
    } else if (exp == FP32_EXP_INF) {
        result = fp32_infinity(sgn);
    } else if (!mnt) {
        result = fp32_zero(sgn);
    } else if (exp >= expint) {
        // There are no fractional bits
        result = op;
    } else {
        // Truncate towards zero:
        uint32_t x = expint - exp >= FP32_BITS ? 0 : mnt >> (expint - exp);
        int err = exp < expint - FP32_BITS ? 1 :
            ((mnt << 1 >> (expint - exp - 1) & 3) |
             ((uint32_t)(mnt << 2 << (FP32_BITS + exp - expint)) != 0));
        switch (rounding) {
          case FPRounding_TIEEVEN:
            x += (err == 3 || (err == 2 && (x & 1)));
            break;
          case FPRounding_POSINF:
            x += err && !sgn;
            break;
          case FPRounding_NEGINF:
            x += err && sgn;
            break;
          case FPRounding_ZERO:
            break;
          case FPRounding_TIEAWAY:
            x += err >> 1;
            break;
          default:
            panic("Unrecognized FP rounding mode");
        }

        if (x == 0) {
            result = fp32_zero(sgn);
        } else {
            exp = expint;
            mnt = fp32_normalise(x, &exp);
            result = fp32_pack(sgn, exp + FP32_EXP_BITS, mnt >> FP32_EXP_BITS);
        }

        if (err && exact)
            flags |= FPLIB_IXC;
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint64_t
fplibRoundInt(uint64_t op, FPRounding rounding, bool exact, FPSCR &fpscr)
{
    int expint = FP64_EXP_BIAS + FP64_MANT_BITS;
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn, exp;
    uint64_t mnt, result;

    // Unpack using FPCR to determine if subnormals are flushed-to-zero:
    fp64_unpack(&sgn, &exp, &mnt, op, mode, &flags);

    // Handle NaNs, infinities and zeroes:
    if (fp64_is_NaN(exp, mnt)) {
        result = fp64_process_NaN(op, mode, &flags);
    } else if (exp == FP64_EXP_INF) {
        result = fp64_infinity(sgn);
    } else if (!mnt) {
        result = fp64_zero(sgn);
    } else if (exp >= expint) {
        // There are no fractional bits
        result = op;
    } else {
        // Truncate towards zero:
        uint64_t x = expint - exp >= FP64_BITS ? 0 : mnt >> (expint - exp);
        int err = exp < expint - FP64_BITS ? 1 :
            ((mnt << 1 >> (expint - exp - 1) & 3) |
             ((uint64_t)(mnt << 2 << (FP64_BITS + exp - expint)) != 0));
        switch (rounding) {
          case FPRounding_TIEEVEN:
            x += (err == 3 || (err == 2 && (x & 1)));
            break;
          case FPRounding_POSINF:
            x += err && !sgn;
            break;
          case FPRounding_NEGINF:
            x += err && sgn;
            break;
          case FPRounding_ZERO:
            break;
          case FPRounding_TIEAWAY:
            x += err >> 1;
            break;
          default:
            panic("Unrecognized FP rounding mode");
        }

        if (x == 0) {
            result = fp64_zero(sgn);
        } else {
            exp = expint;
            mnt = fp64_normalise(x, &exp);
            result = fp64_pack(sgn, exp + FP64_EXP_BITS, mnt >> FP64_EXP_BITS);
        }

        if (err && exact)
            flags |= FPLIB_IXC;
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint16_t
fplibScale(uint16_t op1, uint16_t op2, FPSCR &fpscr)
{
    int flags = 0;
    uint16_t result = fp16_scale(op1, (int16_t)op2, modeConv(fpscr), &flags);
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint32_t
fplibScale(uint32_t op1, uint32_t op2, FPSCR &fpscr)
{
    int flags = 0;
    uint32_t result = fp32_scale(op1, (int32_t)op2, modeConv(fpscr), &flags);
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint64_t
fplibScale(uint64_t op1, uint64_t op2, FPSCR &fpscr)
{
    int flags = 0;
    uint64_t result = fp64_scale(op1, (int64_t)op2, modeConv(fpscr), &flags);
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint16_t
fplibSqrt(uint16_t op, FPSCR &fpscr)
{
    int flags = 0;
    uint16_t result = fp16_sqrt(op, modeConv(fpscr), &flags);
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint32_t
fplibSqrt(uint32_t op, FPSCR &fpscr)
{
    int flags = 0;
    uint32_t result = fp32_sqrt(op, modeConv(fpscr), &flags);
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint64_t
fplibSqrt(uint64_t op, FPSCR &fpscr)
{
    int flags = 0;
    uint64_t result = fp64_sqrt(op, modeConv(fpscr), &flags);
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint16_t
fplibSub(uint16_t op1, uint16_t op2, FPSCR &fpscr)
{
    int flags = 0;
    uint16_t result = fp16_add(op1, op2, 1, modeConv(fpscr), &flags);
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint32_t
fplibSub(uint32_t op1, uint32_t op2, FPSCR &fpscr)
{
    int flags = 0;
    uint32_t result = fp32_add(op1, op2, 1, modeConv(fpscr), &flags);
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint64_t
fplibSub(uint64_t op1, uint64_t op2, FPSCR &fpscr)
{
    int flags = 0;
    uint64_t result = fp64_add(op1, op2, 1, modeConv(fpscr), &flags);
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint16_t
fplibTrigMulAdd(uint8_t coeff_index, uint16_t op1, uint16_t op2, FPSCR &fpscr)
{
    static uint16_t coeff[2][8] = {
        {
            0x3c00,
            0xb155,
            0x2030,
            0x0000,
            0x0000,
            0x0000,
            0x0000,
            0x0000,
        },
        {
            0x3c00,
            0xb800,
            0x293a,
            0x0000,
            0x0000,
            0x0000,
            0x0000,
            0x0000
        }
    };
    int flags = 0;
    uint16_t result =
        fp16_muladd(coeff[op2 >> (FP16_BITS - 1)][coeff_index], op1,
                    fplibAbs(op2), 0, modeConv(fpscr), &flags);
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint32_t
fplibTrigMulAdd(uint8_t coeff_index, uint32_t op1, uint32_t op2, FPSCR &fpscr)
{
    static uint32_t coeff[2][8] = {
        {
            0x3f800000,
            0xbe2aaaab,
            0x3c088886,
            0xb95008b9,
            0x36369d6d,
            0x00000000,
            0x00000000,
            0x00000000
        },
        {
            0x3f800000,
            0xbf000000,
            0x3d2aaaa6,
            0xbab60705,
            0x37cd37cc,
            0x00000000,
            0x00000000,
            0x00000000
        }
    };
    int flags = 0;
    uint32_t result =
        fp32_muladd(coeff[op2 >> (FP32_BITS - 1)][coeff_index], op1,
                    fplibAbs(op2), 0, modeConv(fpscr), &flags);
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint64_t
fplibTrigMulAdd(uint8_t coeff_index, uint64_t op1, uint64_t op2, FPSCR &fpscr)
{
    static uint64_t coeff[2][8] = {
        {
            0x3ff0000000000000ULL,
            0xbfc5555555555543ULL,
            0x3f8111111110f30cULL,
            0xbf2a01a019b92fc6ULL,
            0x3ec71de351f3d22bULL,
            0xbe5ae5e2b60f7b91ULL,
            0x3de5d8408868552fULL,
            0x0000000000000000ULL
        },
        {
            0x3ff0000000000000ULL,
            0xbfe0000000000000ULL,
            0x3fa5555555555536ULL,
            0xbf56c16c16c13a0bULL,
            0x3efa01a019b1e8d8ULL,
            0xbe927e4f7282f468ULL,
            0x3e21ee96d2641b13ULL,
            0xbda8f76380fbb401ULL
        }
    };
    int flags = 0;
    uint64_t result =
        fp64_muladd(coeff[op2 >> (FP64_BITS - 1)][coeff_index], op1,
                    fplibAbs(op2), 0, modeConv(fpscr), &flags);
    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint16_t
fplibTrigSMul(uint16_t op1, uint16_t op2, FPSCR &fpscr)
{
    int flags = 0;
    int sgn, exp;
    uint16_t mnt;

    int mode = modeConv(fpscr);
    uint16_t result = fp16_mul(op1, op1, mode, &flags);
    set_fpscr0(fpscr, flags);

    fp16_unpack(&sgn, &exp, &mnt, result, mode, &flags);
    if (!fp16_is_NaN(exp, mnt)) {
        result = (result & ~(1ULL << (FP16_BITS - 1))) |
            op2 << (FP16_BITS - 1);
    }
    return result;
}

template <>
uint32_t
fplibTrigSMul(uint32_t op1, uint32_t op2, FPSCR &fpscr)
{
    int flags = 0;
    int sgn, exp;
    uint32_t mnt;

    int mode = modeConv(fpscr);
    uint32_t result = fp32_mul(op1, op1, mode, &flags);
    set_fpscr0(fpscr, flags);

    fp32_unpack(&sgn, &exp, &mnt, result, mode, &flags);
    if (!fp32_is_NaN(exp, mnt)) {
        result = (result & ~(1ULL << (FP32_BITS - 1))) | op2 << (FP32_BITS - 1);
    }
    return result;
}

template <>
uint64_t
fplibTrigSMul(uint64_t op1, uint64_t op2, FPSCR &fpscr)
{
    int flags = 0;
    int sgn, exp;
    uint64_t mnt;

    int mode = modeConv(fpscr);
    uint64_t result = fp64_mul(op1, op1, mode, &flags);
    set_fpscr0(fpscr, flags);

    fp64_unpack(&sgn, &exp, &mnt, result, mode, &flags);
    if (!fp64_is_NaN(exp, mnt)) {
        result = (result & ~(1ULL << (FP64_BITS - 1))) | op2 << (FP64_BITS - 1);
    }
    return result;
}

template <>
uint16_t
fplibTrigSSel(uint16_t op1, uint16_t op2, FPSCR &fpscr)
{
    static constexpr uint16_t fpOne =
        (uint16_t)FP16_EXP_BIAS << FP16_MANT_BITS; // 1.0
    if (op2 & 1)
        op1 = fpOne;
    return op1 ^ ((op2 >> 1) << (FP16_BITS - 1));
}

template <>
uint32_t
fplibTrigSSel(uint32_t op1, uint32_t op2, FPSCR &fpscr)
{
    static constexpr uint32_t fpOne =
        (uint32_t)FP32_EXP_BIAS << FP32_MANT_BITS; // 1.0
    if (op2 & 1)
        op1 = fpOne;
    return op1 ^ ((op2 >> 1) << (FP32_BITS - 1));
}

template <>
uint64_t
fplibTrigSSel(uint64_t op1, uint64_t op2, FPSCR &fpscr)
{
    static constexpr uint64_t fpOne =
        (uint64_t)FP64_EXP_BIAS << FP64_MANT_BITS; // 1.0
    if (op2 & 1)
        op1 = fpOne;
    return op1 ^ ((op2 >> 1) << (FP64_BITS - 1));
}

static uint64_t
FPToFixed_64(int sgn, int exp, uint64_t mnt, bool u, FPRounding rounding,
             int *flags)
{
    int expmax = FP64_EXP_BIAS + FP64_BITS - 1;
    uint64_t x;
    int err;

    if (exp > expmax) {
        *flags = FPLIB_IOC;
        return ((uint64_t)!u << (FP64_BITS - 1)) - !sgn;
    }

    x = lsr64(mnt << FP64_EXP_BITS, expmax - exp);
    err = (exp > expmax - 2 ? 0 :
           (lsr64(mnt << FP64_EXP_BITS, expmax - 2 - exp) & 3) |
           !!(mnt << FP64_EXP_BITS & (lsl64(1, expmax - 2 - exp) - 1)));

    switch (rounding) {
      case FPRounding_TIEEVEN:
        x += (err == 3 || (err == 2 && (x & 1)));
        break;
      case FPRounding_POSINF:
        x += err && !sgn;
        break;
      case FPRounding_NEGINF:
        x += err && sgn;
        break;
      case FPRounding_ZERO:
        break;
      case FPRounding_TIEAWAY:
        x += err >> 1;
        break;
      default:
        panic("Unrecognized FP rounding mode");
    }

    if (u ? sgn && x : x > (1ULL << (FP64_BITS - 1)) - !sgn) {
        *flags = FPLIB_IOC;
        return ((uint64_t)!u << (FP64_BITS - 1)) - !sgn;
    }

    if (err) {
        *flags = FPLIB_IXC;
    }

    return sgn ? -x : x;
}

static uint32_t
FPToFixed_32(int sgn, int exp, uint64_t mnt, bool u, FPRounding rounding,
             int *flags)
{
    uint64_t x = FPToFixed_64(sgn, exp, mnt, u, rounding, flags);
    if (u ? x >= 1ULL << FP32_BITS :
        !(x < 1ULL << (FP32_BITS - 1) ||
          (uint64_t)-x <= (uint64_t)1 << (FP32_BITS - 1))) {
        *flags = FPLIB_IOC;
        x = ((uint32_t)!u << (FP32_BITS - 1)) - !sgn;
    }
    return x;
}

static uint16_t
FPToFixed_16(int sgn, int exp, uint64_t mnt, bool u, FPRounding rounding,
             int *flags)
{
    uint64_t x = FPToFixed_64(sgn, exp, mnt, u, rounding, flags);
    if (u ? x >= 1ULL << FP16_BITS :
        !(x < 1ULL << (FP16_BITS - 1) ||
          (uint64_t)-x <= (uint64_t)1 << (FP16_BITS - 1))) {
        *flags = FPLIB_IOC;
        x = ((uint16_t)!u << (FP16_BITS - 1)) - !sgn;
    }
    return x;
}

template <>
uint16_t
fplibFPToFixed(uint16_t op, int fbits, bool u, FPRounding rounding,
               FPSCR &fpscr)
{
    int flags = 0;
    int sgn, exp;
    uint16_t mnt, result;

    // Unpack using FPCR to determine if subnormals are flushed-to-zero:
    fp16_unpack(&sgn, &exp, &mnt, op, modeConv(fpscr), &flags);

    // If NaN, set cumulative flag or take exception:
    if (fp16_is_NaN(exp, mnt)) {
        flags = FPLIB_IOC;
        result = 0;
    } else {
        assert(fbits >= 0);
        // Infinity is treated as an ordinary normalised number that saturates.
        result =
            FPToFixed_16(sgn, exp + FP64_EXP_BIAS - FP16_EXP_BIAS + fbits,
                         (uint64_t)mnt << (FP64_MANT_BITS - FP16_MANT_BITS),
                         u, rounding, &flags);
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint32_t
fplibFPToFixed(uint16_t op, int fbits, bool u, FPRounding rounding,
               FPSCR &fpscr)
{
    int flags = 0;
    int sgn, exp;
    uint16_t mnt;
    uint32_t result;

    // Unpack using FPCR to determine if subnormals are flushed-to-zero:
    fp16_unpack(&sgn, &exp, &mnt, op, modeConv(fpscr), &flags);

    // If NaN, set cumulative flag or take exception:
    if (fp16_is_NaN(exp, mnt)) {
        flags = FPLIB_IOC;
        result = 0;
    } else {
        assert(fbits >= 0);
        if (exp == FP16_EXP_INF)
            exp = 255; // infinity: make it big enough to saturate
        result =
            FPToFixed_32(sgn, exp + FP64_EXP_BIAS - FP16_EXP_BIAS + fbits,
                         (uint64_t)mnt << (FP64_MANT_BITS - FP16_MANT_BITS),
                         u, rounding, &flags);
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint32_t
fplibFPToFixed(uint32_t op, int fbits, bool u, FPRounding rounding, FPSCR &fpscr)
{
    int flags = 0;
    int sgn, exp;
    uint32_t mnt, result;

    // Unpack using FPCR to determine if subnormals are flushed-to-zero:
    fp32_unpack(&sgn, &exp, &mnt, op, modeConv(fpscr), &flags);

    // If NaN, set cumulative flag or take exception:
    if (fp32_is_NaN(exp, mnt)) {
        flags = FPLIB_IOC;
        result = 0;
    } else {
        assert(fbits >= 0);
        // Infinity is treated as an ordinary normalised number that saturates.
        result =
            FPToFixed_32(sgn, exp + FP64_EXP_BIAS - FP32_EXP_BIAS + fbits,
                         (uint64_t)mnt << (FP64_MANT_BITS - FP32_MANT_BITS),
                         u, rounding, &flags);
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint32_t
fplibFPToFixed(uint64_t op, int fbits, bool u, FPRounding rounding, FPSCR &fpscr)
{
    int flags = 0;
    int sgn, exp;
    uint64_t mnt;
    uint32_t result;

    // Unpack using FPCR to determine if subnormals are flushed-to-zero:
    fp64_unpack(&sgn, &exp, &mnt, op, modeConv(fpscr), &flags);

    // If NaN, set cumulative flag or take exception:
    if (fp64_is_NaN(exp, mnt)) {
        flags = FPLIB_IOC;
        result = 0;
    } else {
        assert(fbits >= 0);
        // Infinity is treated as an ordinary normalised number that saturates.
        result = FPToFixed_32(sgn, exp + fbits, mnt, u, rounding, &flags);
    }

    set_fpscr0(fpscr, flags);

    return result;
}

uint32_t
fplibFPToFixedJS(uint64_t op, FPSCR &fpscr, bool is64, uint8_t& nz)
{
    int flags = 0;
    uint32_t result;
    bool Z = true;

    uint32_t sgn = bits(op, 63);
    int32_t exp  = bits(op, 62, 52);
    uint64_t mnt = bits(op, 51, 0);

    if (exp == 0) {
        if (mnt != 0) {
           if (fpscr.fz) {
                flags |= FPLIB_IDC;
            } else {
                flags |= FPLIB_IXC;
                Z = 0;
            }
        }
        result = 0;
    } else if (exp == 0x7ff) {
        flags |= FPLIB_IOC;
        result = 0;
        Z = 0;
    } else {
        mnt |= 1ULL << FP64_MANT_BITS;
        int mnt_shft = exp - FP64_EXP_BIAS - 52;
        bool err = true;

        if (abs(mnt_shft) >= FP64_BITS) {
            result = 0;
            Z = 0;
        } else if (mnt_shft >= 0) {
            result = lsl64(mnt, mnt_shft);
        } else if (mnt_shft < 0) {
            err = lsl64(mnt, mnt_shft+FP64_BITS) != 0;
            result = lsr64(mnt, abs(mnt_shft));
        }
        uint64_t max_result = (1UL << (FP32_BITS - 1)) -!sgn;
        if ((exp - FP64_EXP_BIAS) > 31 || result > max_result) {
                flags |= FPLIB_IOC;
                Z = false;
        } else if (err) {
                flags |= FPLIB_IXC;
                Z = false;
        }
        result =  sgn ? -result : result;
    }
    if (sgn == 1 && result == 0)
        Z = false;

    if (is64) {
        nz = Z? 0x1: 0x0;
    } else {
        fpscr.n = 0;
        fpscr.z = (int)Z;
        fpscr.c = 0;
        fpscr.v = 0;
    }

    set_fpscr0(fpscr, flags);
    return result;
}

template <>
uint64_t
fplibFPToFixed(uint16_t op, int fbits, bool u, FPRounding rounding,
               FPSCR &fpscr)
{
    int flags = 0;
    int sgn, exp;
    uint16_t mnt;
    uint64_t result;

    // Unpack using FPCR to determine if subnormals are flushed-to-zero:
    fp16_unpack(&sgn, &exp, &mnt, op, modeConv(fpscr), &flags);

    // If NaN, set cumulative flag or take exception:
    if (fp16_is_NaN(exp, mnt)) {
        flags = FPLIB_IOC;
        result = 0;
    } else {
        assert(fbits >= 0);
        if (exp == FP16_EXP_INF)
            exp = 255; // infinity: make it big enough to saturate
        result =
            FPToFixed_64(sgn, exp + FP64_EXP_BIAS - FP16_EXP_BIAS + fbits,
                         (uint64_t)mnt << (FP64_MANT_BITS - FP16_MANT_BITS),
                         u, rounding, &flags);
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint64_t
fplibFPToFixed(uint32_t op, int fbits, bool u, FPRounding rounding, FPSCR &fpscr)
{
    int flags = 0;
    int sgn, exp;
    uint32_t mnt;
    uint64_t result;

    // Unpack using FPCR to determine if subnormals are flushed-to-zero:
    fp32_unpack(&sgn, &exp, &mnt, op, modeConv(fpscr), &flags);

    // If NaN, set cumulative flag or take exception:
    if (fp32_is_NaN(exp, mnt)) {
        flags = FPLIB_IOC;
        result = 0;
    } else {
        assert(fbits >= 0);
        // Infinity is treated as an ordinary normalised number that saturates.
        result =
            FPToFixed_64(sgn, exp + FP64_EXP_BIAS - FP32_EXP_BIAS + fbits,
                         (uint64_t)mnt << (FP64_MANT_BITS - FP32_MANT_BITS),
                         u, rounding, &flags);
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint64_t
fplibFPToFixed(uint64_t op, int fbits, bool u, FPRounding rounding, FPSCR &fpscr)
{
    int flags = 0;
    int sgn, exp;
    uint64_t mnt, result;

    // Unpack using FPCR to determine if subnormals are flushed-to-zero:
    fp64_unpack(&sgn, &exp, &mnt, op, modeConv(fpscr), &flags);

    // If NaN, set cumulative flag or take exception:
    if (fp64_is_NaN(exp, mnt)) {
        flags = FPLIB_IOC;
        result = 0;
    } else {
        assert(fbits >= 0);
        // Infinity is treated as an ordinary normalised number that saturates.
        result = FPToFixed_64(sgn, exp + fbits, mnt, u, rounding, &flags);
    }

    set_fpscr0(fpscr, flags);

    return result;
}

static uint16_t
fp16_cvtf(uint64_t a, int fbits, int u, int mode, int *flags)
{
    int x_sgn = !u && a >> (FP64_BITS - 1);
    int x_exp = FP16_EXP_BIAS + FP64_BITS - 1 - fbits;
    uint64_t x_mnt = x_sgn ? -a : a;

    // Handle zero:
    if (!x_mnt) {
        return fp16_zero(0);
    }

    // Normalise into FP16_BITS bits, collapsing error into bottom bit:
    x_mnt = fp64_normalise(x_mnt, &x_exp);
    x_mnt = (x_mnt >> (FP64_BITS - FP16_BITS - 1) |
             !!(x_mnt & ((1ULL << (FP64_BITS - FP16_BITS - 1)) - 1)));

    return fp16_round(x_sgn, x_exp, x_mnt, mode, flags);
}

static uint32_t
fp32_cvtf(uint64_t a, int fbits, int u, int mode, int *flags)
{
    int x_sgn = !u && a >> (FP64_BITS - 1);
    int x_exp = FP32_EXP_BIAS + FP64_BITS - 1 - fbits;
    uint64_t x_mnt = x_sgn ? -a : a;

    // Handle zero:
    if (!x_mnt) {
        return fp32_zero(0);
    }

    // Normalise into FP32_BITS bits, collapsing error into bottom bit:
    x_mnt = fp64_normalise(x_mnt, &x_exp);
    x_mnt = (x_mnt >> (FP64_BITS - FP32_BITS - 1) |
             !!(x_mnt & ((1ULL << (FP64_BITS - FP32_BITS - 1)) - 1)));

    return fp32_round(x_sgn, x_exp, x_mnt, mode, flags);
}

static uint64_t
fp64_cvtf(uint64_t a, int fbits, int u, int mode, int *flags)
{
    int x_sgn = !u && a >> (FP64_BITS - 1);
    int x_exp = FP64_EXP_BIAS + FP64_BITS - 1 - fbits;
    uint64_t x_mnt = x_sgn ? -a : a;

    // Handle zero:
    if (!x_mnt) {
        return fp64_zero(0);
    }

    x_mnt = fp64_normalise(x_mnt, &x_exp);

    return fp64_round(x_sgn, x_exp, x_mnt << 1, mode, flags);
}

template <>
uint16_t
fplibFixedToFP(uint64_t op, int fbits, bool u, FPRounding rounding,
               FPSCR &fpscr)
{
    int flags = 0;
    uint16_t res = fp16_cvtf(op, fbits, u,
                             (int)rounding | ((uint32_t)fpscr >> 22 & 12),
                             &flags);
    set_fpscr0(fpscr, flags);
    return res;
}

template <>
uint32_t
fplibFixedToFP(uint64_t op, int fbits, bool u, FPRounding rounding, FPSCR &fpscr)
{
    int flags = 0;
    uint32_t res = fp32_cvtf(op, fbits, u,
                             (int)rounding | ((uint32_t)fpscr >> 22 & 12),
                             &flags);
    set_fpscr0(fpscr, flags);
    return res;
}

template <>
uint64_t
fplibFixedToFP(uint64_t op, int fbits, bool u, FPRounding rounding, FPSCR &fpscr)
{
    int flags = 0;
    uint64_t res = fp64_cvtf(op, fbits, u,
                             (int)rounding | ((uint32_t)fpscr >> 22 & 12),
                             &flags);
    set_fpscr0(fpscr, flags);
    return res;
}

template <>
uint16_t
fplibInfinity(int sgn)
{
    return fp16_infinity(sgn);
}

template <>
uint32_t
fplibInfinity(int sgn)
{
    return fp32_infinity(sgn);
}

template <>
uint64_t
fplibInfinity(int sgn)
{
    return fp64_infinity(sgn);
}

template <>
uint16_t
fplibDefaultNaN()
{
    return fp16_defaultNaN();
}

template <>
uint32_t
fplibDefaultNaN()
{
    return fp32_defaultNaN();
}

template <>
uint64_t
fplibDefaultNaN()
{
    return fp64_defaultNaN();
}

}
