/*
* Copyright (c) 2012-2013 ARM Limited
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
*
* Authors: Edmund Grimley Evans
*          Thomas Grocutt
*/

#include <stdint.h>

#include <cassert>

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

#define FPLIB_IDC 128 // Input Denormal
#define FPLIB_IXC 16  // Inexact
#define FPLIB_UFC 8   // Underflow
#define FPLIB_OFC 4   // Overflow
#define FPLIB_DZC 2   // Division by Zero
#define FPLIB_IOC 1   // Invalid Operation

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
    if (shift < 64) {
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
    if (shift < 64) {
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
mul64x64(uint64_t *x0, uint64_t *x1, uint64_t a, uint64_t b)
{
    uint64_t a0 = (uint32_t)a;
    uint64_t a1 = a >> 32;
    uint64_t b0 = (uint32_t)b;
    uint64_t b1 = b >> 32;
    uint64_t t1 = (a0 * b0 >> 32) + a1 * b0;
    uint64_t t2 = a0 * b1;
    uint64_t x = ((uint64_t)(uint32_t)t1 + (uint32_t)t2) >> 32;
    x += t1 >> 32;
    x += t2 >> 32;
    x += a1 * b1;
    *x0 = a * b;
    *x1 = x;
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
    return sgn << 15 | exp << 10 | (mnt & (((uint16_t)1 << 10) - 1));
}

static inline uint32_t
fp32_pack(uint32_t sgn, uint32_t exp, uint32_t mnt)
{
    return sgn << 31 | exp << 23 | (mnt & (((uint32_t)1 << 23) - 1));
}

static inline uint64_t
fp64_pack(uint64_t sgn, uint64_t exp, uint64_t mnt)
{
    return (uint64_t)sgn << 63 | exp << 52 | (mnt & (((uint64_t)1 << 52) - 1));
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
    return fp16_pack(sgn, 30, -1);
}

static inline uint32_t
fp32_max_normal(int sgn)
{
    return fp32_pack(sgn, 254, -1);
}

static inline uint64_t
fp64_max_normal(int sgn)
{
    return fp64_pack(sgn, 2046, -1);
}

static inline uint16_t
fp16_infinity(int sgn)
{
    return fp16_pack(sgn, 31, 0);
}

static inline uint32_t
fp32_infinity(int sgn)
{
    return fp32_pack(sgn, 255, 0);
}

static inline uint64_t
fp64_infinity(int sgn)
{
    return fp64_pack(sgn, 2047, 0);
}

static inline uint16_t
fp16_defaultNaN()
{
    return fp16_pack(0, 31, (uint16_t)1 << 9);
}

static inline uint32_t
fp32_defaultNaN()
{
    return fp32_pack(0, 255, (uint32_t)1 << 22);
}

static inline uint64_t
fp64_defaultNaN()
{
    return fp64_pack(0, 2047, (uint64_t)1 << 51);
}

static inline void
fp16_unpack(int *sgn, int *exp, uint16_t *mnt, uint16_t x, int mode,
            int *flags)
{
    *sgn = x >> 15;
    *exp = x >> 10 & 31;
    *mnt = x & (((uint16_t)1 << 10) - 1);

    // Handle subnormals:
    if (*exp) {
        *mnt |= (uint16_t)1 << 10;
    } else {
        ++*exp;
        // There is no flush to zero in this case!
    }
}

static inline void
fp32_unpack(int *sgn, int *exp, uint32_t *mnt, uint32_t x, int mode,
            int *flags)
{
    *sgn = x >> 31;
    *exp = x >> 23 & 255;
    *mnt = x & (((uint32_t)1 << 23) - 1);

    // Handle subnormals:
    if (*exp) {
        *mnt |= (uint32_t)1 << 23;
    } else {
        ++*exp;
        if ((mode & FPLIB_FZ) && *mnt) {
            *flags |= FPLIB_IDC;
            *mnt = 0;
        }
    }
}

static inline void
fp64_unpack(int *sgn, int *exp, uint64_t *mnt, uint64_t x, int mode,
            int *flags)
{
    *sgn = x >> 63;
    *exp = x >> 52 & 2047;
    *mnt = x & (((uint64_t)1 << 52) - 1);

    // Handle subnormals:
    if (*exp) {
        *mnt |= (uint64_t)1 << 52;
    } else {
        ++*exp;
        if ((mode & FPLIB_FZ) && *mnt) {
            *flags |= FPLIB_IDC;
            *mnt = 0;
        }
    }
}

static inline uint32_t
fp32_process_NaN(uint32_t a, int mode, int *flags)
{
    if (!(a >> 22 & 1)) {
        *flags |= FPLIB_IOC;
        a |= (uint32_t)1 << 22;
    }
    return mode & FPLIB_DN ? fp32_defaultNaN() : a;
}

static inline uint64_t
fp64_process_NaN(uint64_t a, int mode, int *flags)
{
    if (!(a >> 51 & 1)) {
        *flags |= FPLIB_IOC;
        a |= (uint64_t)1 << 51;
    }
    return mode & FPLIB_DN ? fp64_defaultNaN() : a;
}

static uint32_t
fp32_process_NaNs(uint32_t a, uint32_t b, int mode, int *flags)
{
    int a_exp = a >> 23 & 255;
    uint32_t a_mnt = a & (((uint32_t)1 << 23) - 1);
    int b_exp = b >> 23 & 255;
    uint32_t b_mnt = b & (((uint32_t)1 << 23) - 1);

    // Handle signalling NaNs:
    if (a_exp == 255 && a_mnt && !(a_mnt >> 22 & 1))
        return fp32_process_NaN(a, mode, flags);
    if (b_exp == 255 && b_mnt && !(b_mnt >> 22 & 1))
        return fp32_process_NaN(b, mode, flags);

    // Handle quiet NaNs:
    if (a_exp == 255 && a_mnt)
        return fp32_process_NaN(a, mode, flags);
    if (b_exp == 255 && b_mnt)
        return fp32_process_NaN(b, mode, flags);

    return 0;
}

static uint64_t
fp64_process_NaNs(uint64_t a, uint64_t b, int mode, int *flags)
{
    int a_exp = a >> 52 & 2047;
    uint64_t a_mnt = a & (((uint64_t)1 << 52) - 1);
    int b_exp = b >> 52 & 2047;
    uint64_t b_mnt = b & (((uint64_t)1 << 52) - 1);

    // Handle signalling NaNs:
    if (a_exp == 2047 && a_mnt && !(a_mnt >> 51 & 1))
        return fp64_process_NaN(a, mode, flags);
    if (b_exp == 2047 && b_mnt && !(b_mnt >> 51 & 1))
        return fp64_process_NaN(b, mode, flags);

    // Handle quiet NaNs:
    if (a_exp == 2047 && a_mnt)
        return fp64_process_NaN(a, mode, flags);
    if (b_exp == 2047 && b_mnt)
        return fp64_process_NaN(b, mode, flags);

    return 0;
}

static uint32_t
fp32_process_NaNs3(uint32_t a, uint32_t b, uint32_t c, int mode, int *flags)
{
    int a_exp = a >> 23 & 255;
    uint32_t a_mnt = a & (((uint32_t)1 << 23) - 1);
    int b_exp = b >> 23 & 255;
    uint32_t b_mnt = b & (((uint32_t)1 << 23) - 1);
    int c_exp = c >> 23 & 255;
    uint32_t c_mnt = c & (((uint32_t)1 << 23) - 1);

    // Handle signalling NaNs:
    if (a_exp == 255 && a_mnt && !(a_mnt >> 22 & 1))
        return fp32_process_NaN(a, mode, flags);
    if (b_exp == 255 && b_mnt && !(b_mnt >> 22 & 1))
        return fp32_process_NaN(b, mode, flags);
    if (c_exp == 255 && c_mnt && !(c_mnt >> 22 & 1))
        return fp32_process_NaN(c, mode, flags);

    // Handle quiet NaNs:
    if (a_exp == 255 && a_mnt)
        return fp32_process_NaN(a, mode, flags);
    if (b_exp == 255 && b_mnt)
        return fp32_process_NaN(b, mode, flags);
    if (c_exp == 255 && c_mnt)
        return fp32_process_NaN(c, mode, flags);

    return 0;
}

static uint64_t
fp64_process_NaNs3(uint64_t a, uint64_t b, uint64_t c, int mode, int *flags)
{
    int a_exp = a >> 52 & 2047;
    uint64_t a_mnt = a & (((uint64_t)1 << 52) - 1);
    int b_exp = b >> 52 & 2047;
    uint64_t b_mnt = b & (((uint64_t)1 << 52) - 1);
    int c_exp = c >> 52 & 2047;
    uint64_t c_mnt = c & (((uint64_t)1 << 52) - 1);

    // Handle signalling NaNs:
    if (a_exp == 2047 && a_mnt && !(a_mnt >> 51 & 1))
        return fp64_process_NaN(a, mode, flags);
    if (b_exp == 2047 && b_mnt && !(b_mnt >> 51 & 1))
        return fp64_process_NaN(b, mode, flags);
    if (c_exp == 2047 && c_mnt && !(c_mnt >> 51 & 1))
        return fp64_process_NaN(c, mode, flags);

    // Handle quiet NaNs:
    if (a_exp == 2047 && a_mnt)
        return fp64_process_NaN(a, mode, flags);
    if (b_exp == 2047 && b_mnt)
        return fp64_process_NaN(b, mode, flags);
    if (c_exp == 2047 && c_mnt)
        return fp64_process_NaN(c, mode, flags);

    return 0;
}

static uint16_t
fp16_round_(int sgn, int exp, uint16_t mnt, int rm, int mode, int *flags)
{
    int biased_exp; // non-negative exponent value for result
    uint16_t int_mant; // mantissa for result, less than (1 << 11)
    int error; // 0, 1, 2 or 3, where 2 means int_mant is wrong by exactly 0.5

    assert(rm != FPRounding_TIEAWAY);

    // There is no flush to zero in this case!

    // The bottom 5 bits of mnt are orred together:
    mnt = (uint16_t)1 << 12 | mnt >> 4 | ((mnt & 31) != 0);

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
        if (int_mant == (uint32_t)1 << 10) {
            // Rounded up from denormalized to normalized
            biased_exp = 1;
        }
        if (int_mant == (uint32_t)1 << 11) {
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
        if (biased_exp >= 31) {
            *flags |= FPLIB_OFC | FPLIB_IXC;
            if (rm == FPLIB_RN || (rm == FPLIB_RP && !sgn) ||
                (rm == FPLIB_RM && sgn)) {
                return fp16_infinity(sgn);
            } else {
                return fp16_max_normal(sgn);
            }
        }
    } else {
        if (biased_exp >= 32) {
            *flags |= FPLIB_IOC;
            return fp16_pack(sgn, 31, -1);
        }
    }

    if (error) {
        *flags |= FPLIB_IXC;
    }

    return fp16_pack(sgn, biased_exp, int_mant);
}

static uint32_t
fp32_round_(int sgn, int exp, uint32_t mnt, int rm, int mode, int *flags)
{
    int biased_exp; // non-negative exponent value for result
    uint32_t int_mant; // mantissa for result, less than (1 << 24)
    int error; // 0, 1, 2 or 3, where 2 means int_mant is wrong by exactly 0.5

    assert(rm != FPRounding_TIEAWAY);

    // Flush to zero:
    if ((mode & FPLIB_FZ) && exp < 1) {
        *flags |= FPLIB_UFC;
        return fp32_zero(sgn);
    }

    // The bottom 8 bits of mnt are orred together:
    mnt = (uint32_t)1 << 25 | mnt >> 7 | ((mnt & 255) != 0);

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
        if (int_mant == (uint32_t)1 << 23) {
            // Rounded up from denormalized to normalized
            biased_exp = 1;
        }
        if (int_mant == (uint32_t)1 << 24) {
            // Rounded up to next exponent
            ++biased_exp;
            int_mant >>= 1;
        }
    }

    // Handle rounding to odd aka Von Neumann rounding:
    if (error && rm == FPRounding_ODD)
        int_mant |= 1;

    // Handle overflow:
    if (biased_exp >= 255) {
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
    uint64_t int_mant; // mantissa for result, less than (1 << 52)
    int error; // 0, 1, 2 or 3, where 2 means int_mant is wrong by exactly 0.5

    assert(rm != FPRounding_TIEAWAY);

    // Flush to zero:
    if ((mode & FPLIB_FZ) && exp < 1) {
        *flags |= FPLIB_UFC;
        return fp64_zero(sgn);
    }

    // The bottom 11 bits of mnt are orred together:
    mnt = (uint64_t)1 << 54 | mnt >> 10 | ((mnt & 0x3ff) != 0);

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
        if (int_mant == (uint64_t)1 << 52) {
            // Rounded up from denormalized to normalized
            biased_exp = 1;
        }
        if (int_mant == (uint64_t)1 << 53) {
            // Rounded up to next exponent
            ++biased_exp;
            int_mant >>= 1;
        }
    }

    // Handle rounding to odd aka Von Neumann rounding:
    if (error && rm == FPRounding_ODD)
        int_mant |= 1;

    // Handle overflow:
    if (biased_exp >= 2047) {
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
fp32_compare_eq(uint32_t a, uint32_t b, int mode, int *flags)
{
    int a_sgn, a_exp, b_sgn, b_exp;
    uint32_t a_mnt, b_mnt;

    fp32_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);
    fp32_unpack(&b_sgn, &b_exp, &b_mnt, b, mode, flags);

    if ((a_exp == 255 && (uint32_t)(a_mnt << 9)) ||
        (b_exp == 255 && (uint32_t)(b_mnt << 9))) {
        if ((a_exp == 255 && (uint32_t)(a_mnt << 9) && !(a >> 22 & 1)) ||
            (b_exp == 255 && (uint32_t)(b_mnt << 9) && !(b >> 22 & 1)))
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

    if ((a_exp == 255 && (uint32_t)(a_mnt << 9)) ||
        (b_exp == 255 && (uint32_t)(b_mnt << 9))) {
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

    if ((a_exp == 255 && (uint32_t)(a_mnt << 9)) ||
        (b_exp == 255 && (uint32_t)(b_mnt << 9))) {
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
fp64_compare_eq(uint64_t a, uint64_t b, int mode, int *flags)
{
    int a_sgn, a_exp, b_sgn, b_exp;
    uint64_t a_mnt, b_mnt;

    fp64_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);
    fp64_unpack(&b_sgn, &b_exp, &b_mnt, b, mode, flags);

    if ((a_exp == 2047 && (uint64_t)(a_mnt << 12)) ||
        (b_exp == 2047 && (uint64_t)(b_mnt << 12))) {
        if ((a_exp == 2047 && (uint64_t)(a_mnt << 12) && !(a >> 51 & 1)) ||
            (b_exp == 2047 && (uint64_t)(b_mnt << 12) && !(b >> 51 & 1)))
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

    if ((a_exp == 2047 && (uint64_t)(a_mnt << 12)) ||
        (b_exp == 2047 && (uint64_t)(b_mnt << 12))) {
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

    if ((a_exp == 2047 && (uint64_t)(a_mnt << 12)) ||
        (b_exp == 2047 && (uint64_t)(b_mnt << 12))) {
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
    if (a_exp == 255 && b_exp == 255 && a_sgn != b_sgn) {
        *flags |= FPLIB_IOC;
        return fp32_defaultNaN();
    } else if (a_exp == 255) {
        return fp32_infinity(a_sgn);
    } else if (b_exp == 255) {
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

    return fp32_round(x_sgn, x_exp + 5, x_mnt << 1, mode, flags);
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
    if (a_exp == 2047 && b_exp == 2047 && a_sgn != b_sgn) {
        *flags |= FPLIB_IOC;
        return fp64_defaultNaN();
    } else if (a_exp == 2047) {
        return fp64_infinity(a_sgn);
    } else if (b_exp == 2047) {
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

    return fp64_round(x_sgn, x_exp + 8, x_mnt << 1, mode, flags);
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
    if ((a_exp == 255 && !b_mnt) || (b_exp == 255 && !a_mnt)) {
        *flags |= FPLIB_IOC;
        return fp32_defaultNaN();
    } else if (a_exp == 255 || b_exp == 255) {
        return fp32_infinity(a_sgn ^ b_sgn);
    } else if (!a_mnt || !b_mnt) {
        return fp32_zero(a_sgn ^ b_sgn);
    }

    // Multiply and normalise:
    x_sgn = a_sgn ^ b_sgn;
    x_exp = a_exp + b_exp - 110;
    x_mnt = (uint64_t)a_mnt * b_mnt;
    x_mnt = fp64_normalise(x_mnt, &x_exp);

    // Convert to 32 bits, collapsing error into bottom bit:
    x_mnt = lsr64(x_mnt, 31) | !!lsl64(x_mnt, 33);

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
    if ((a_exp == 2047 && !b_mnt) || (b_exp == 2047 && !a_mnt)) {
        *flags |= FPLIB_IOC;
        return fp64_defaultNaN();
    } else if (a_exp == 2047 || b_exp == 2047) {
        return fp64_infinity(a_sgn ^ b_sgn);
    } else if (!a_mnt || !b_mnt) {
        return fp64_zero(a_sgn ^ b_sgn);
    }

    // Multiply and normalise:
    x_sgn = a_sgn ^ b_sgn;
    x_exp = a_exp + b_exp - 1000;
    mul62x62(&x0_mnt, &x1_mnt, a_mnt, b_mnt);
    fp128_normalise(&x0_mnt, &x1_mnt, &x_exp);

    // Convert to 64 bits, collapsing error into bottom bit:
    x0_mnt = x1_mnt << 1 | !!x0_mnt;

    return fp64_round(x_sgn, x_exp, x0_mnt, mode, flags);
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
    if (a_exp == 255 && (a_mnt >> 22 & 1) &&
        ((!b_mnt && c_exp == 255 && !(uint32_t)(c_mnt << 9)) ||
         (!c_mnt && b_exp == 255 && !(uint32_t)(b_mnt << 9)))) {
        x = fp32_defaultNaN();
        *flags |= FPLIB_IOC;
    }

    if (x) {
        return x;
    }

    // Handle infinities and zeroes:
    if ((b_exp == 255 && !c_mnt) ||
        (c_exp == 255 && !b_mnt) ||
        (a_exp == 255 && (b_exp == 255 || c_exp == 255) &&
         (a_sgn != (b_sgn ^ c_sgn)))) {
        *flags |= FPLIB_IOC;
        return fp32_defaultNaN();
    }
    if (a_exp == 255)
        return fp32_infinity(a_sgn);
    if (b_exp == 255 || c_exp == 255)
        return fp32_infinity(b_sgn ^ c_sgn);
    if (!a_mnt && (!b_mnt || !c_mnt) && a_sgn == (b_sgn ^ c_sgn))
        return fp32_zero(a_sgn);

    x_sgn = a_sgn;
    x_exp = a_exp + 13;
    x_mnt = (uint64_t)a_mnt << 27;

    // Multiply:
    y_sgn = b_sgn ^ c_sgn;
    y_exp = b_exp + c_exp - 113;
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

    // Normalise and convert to 32 bits, collapsing error into bottom bit:
    x_mnt = fp64_normalise(x_mnt, &x_exp);
    x_mnt = x_mnt >> 31 | !!(uint32_t)(x_mnt << 1);

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
    if (a_exp == 2047 && (a_mnt >> 51 & 1) &&
        ((!b_mnt && c_exp == 2047 && !(uint64_t)(c_mnt << 12)) ||
         (!c_mnt && b_exp == 2047 && !(uint64_t)(b_mnt << 12)))) {
        x = fp64_defaultNaN();
        *flags |= FPLIB_IOC;
    }

    if (x) {
        return x;
    }

    // Handle infinities and zeroes:
    if ((b_exp == 2047 && !c_mnt) ||
        (c_exp == 2047 && !b_mnt) ||
        (a_exp == 2047 && (b_exp == 2047 || c_exp == 2047) &&
         (a_sgn != (b_sgn ^ c_sgn)))) {
        *flags |= FPLIB_IOC;
        return fp64_defaultNaN();
    }
    if (a_exp == 2047)
        return fp64_infinity(a_sgn);
    if (b_exp == 2047 || c_exp == 2047)
        return fp64_infinity(b_sgn ^ c_sgn);
    if (!a_mnt && (!b_mnt || !c_mnt) && a_sgn == (b_sgn ^ c_sgn))
        return fp64_zero(a_sgn);

    x_sgn = a_sgn;
    x_exp = a_exp + 11;
    x0_mnt = 0;
    x1_mnt = a_mnt;

    // Multiply:
    y_sgn = b_sgn ^ c_sgn;
    y_exp = b_exp + c_exp - 1003;
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

    // Normalise and convert to 64 bits, collapsing error into bottom bit:
    fp128_normalise(&x0_mnt, &x1_mnt, &x_exp);
    x0_mnt = x1_mnt << 1 | !!x0_mnt;

    return fp64_round(x_sgn, x_exp + scale, x0_mnt, mode, flags);
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
    if ((a_exp == 255 && b_exp == 255) || (!a_mnt && !b_mnt)) {
        *flags |= FPLIB_IOC;
        return fp32_defaultNaN();
    }
    if (a_exp == 255 || !b_mnt) {
        if (a_exp != 255)
            *flags |= FPLIB_DZC;
        return fp32_infinity(a_sgn ^ b_sgn);
    }
    if (!a_mnt || b_exp == 255)
        return fp32_zero(a_sgn ^ b_sgn);

    // Divide, setting bottom bit if inexact:
    a_mnt = fp32_normalise(a_mnt, &a_exp);
    x_sgn = a_sgn ^ b_sgn;
    x_exp = a_exp - b_exp + 172;
    x_mnt = ((uint64_t)a_mnt << 18) / b_mnt;
    x_mnt |= (x_mnt * b_mnt != (uint64_t)a_mnt << 18);

    // Normalise and convert to 32 bits, collapsing error into bottom bit:
    x_mnt = fp64_normalise(x_mnt, &x_exp);
    x_mnt = x_mnt >> 31 | !!(uint32_t)(x_mnt << 1);

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
    if ((a_exp == 2047 && b_exp == 2047) || (!a_mnt && !b_mnt)) {
        *flags |= FPLIB_IOC;
        return fp64_defaultNaN();
    }
    if (a_exp == 2047 || !b_mnt) {
        if (a_exp != 2047)
            *flags |= FPLIB_DZC;
        return fp64_infinity(a_sgn ^ b_sgn);
    }
    if (!a_mnt || b_exp == 2047)
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
    x_exp = a_exp - b_exp + 1031;
    mul62x62(&x0_mnt, &x1_mnt, x0_mnt, a_mnt >> 2); // xx 62x62 is enough
    lsr128(&x0_mnt, &x1_mnt, x0_mnt, x1_mnt, 4);
    x_mnt = x1_mnt;

    // This is an underestimate, so try adding one:
    mul62x62(&x0_mnt, &x1_mnt, b_mnt >> 2, x_mnt + 1); // xx 62x62 is enough
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

static uint32_t
fp32_sqrt(uint32_t a, int mode, int *flags)
{
    int a_sgn, a_exp, x_sgn, x_exp;
    uint32_t a_mnt, x, x_mnt;
    uint64_t t0, t1;

    fp32_unpack(&a_sgn, &a_exp, &a_mnt, a, mode, flags);

    // Handle NaNs:
    if (a_exp == 255 && (uint32_t)(a_mnt << 9))
        return fp32_process_NaN(a, mode, flags);

    // Handle infinities and zeroes:
    if (!a_mnt) {
        return fp32_zero(a_sgn);
    }
    if (a_exp == 255 && !a_sgn) {
        return fp32_infinity(a_sgn);
    }
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
    x = (a_mnt >> 2) + (a_mnt >> 3) + (5 << 28);

    // x = (a / x + x) / 2; // 16-bit accuracy
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
    if (a_exp == 2047 && (uint64_t)(a_mnt << 12)) {
        return fp64_process_NaN(a, mode, flags);
    }

    // Handle infinities and zeroes:
    if (!a_mnt)
        return fp64_zero(a_sgn);
    if (a_exp == 2047 && !a_sgn)
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
    x = (a_mnt >> 34) + (a_mnt >> 35) + (5 << 28);

    // x = (a / x + x) / 2; // 16-bit accuracy
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
    return (((int) fpscr) >> 22) & 0xF;
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
uint32_t
fplibAbs(uint32_t op)
{
    return op & ~((uint32_t)1 << 31);
}

template <>
uint64_t
fplibAbs(uint64_t op)
{
    return op & ~((uint64_t)1 << 63);
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
fplibCompare(uint32_t op1, uint32_t op2, bool signal_nans, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn1, exp1, sgn2, exp2, result;
    uint32_t mnt1, mnt2;

    fp32_unpack(&sgn1, &exp1, &mnt1, op1, mode, &flags);
    fp32_unpack(&sgn2, &exp2, &mnt2, op2, mode, &flags);

    if ((exp1 == 255 && (uint32_t)(mnt1 << 9)) ||
        (exp2 == 255 && (uint32_t)(mnt2 << 9))) {
        result = 3;
        if ((exp1 == 255 && (uint32_t)(mnt1 << 9) && !(mnt1 >> 22 & 1)) ||
            (exp2 == 255 && (uint32_t)(mnt2 << 9) && !(mnt2 >> 22 & 1)) ||
            signal_nans)
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

    if ((exp1 == 2047 && (uint64_t)(mnt1 << 12)) ||
        (exp2 == 2047 && (uint64_t)(mnt2 << 12))) {
        result = 3;
        if ((exp1 == 2047 && (uint64_t)(mnt1 << 12) && !(mnt1 >> 51 & 1)) ||
            (exp2 == 2047 && (uint64_t)(mnt2 << 12) && !(mnt2 >> 51 & 1)) ||
            signal_nans)
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
    return fp16_pack(op >> 31, 31, (uint16_t)1 << 9 | op >> 13);
}

static uint16_t
fp16_FPConvertNaN_64(uint64_t op)
{
    return fp16_pack(op >> 63, 31, (uint16_t)1 << 9 | op >> 42);
}

static uint32_t
fp32_FPConvertNaN_16(uint16_t op)
{
    return fp32_pack(op >> 15, 255, (uint32_t)1 << 22 | (uint32_t)op << 13);
}

static uint32_t
fp32_FPConvertNaN_64(uint64_t op)
{
    return fp32_pack(op >> 63, 255, (uint32_t)1 << 22 | op >> 29);
}

static uint64_t
fp64_FPConvertNaN_16(uint16_t op)
{
    return fp64_pack(op >> 15, 2047, (uint64_t)1 << 51 | (uint64_t)op << 42);
}

static uint64_t
fp64_FPConvertNaN_32(uint32_t op)
{
    return fp64_pack(op >> 31, 2047, (uint64_t)1 << 51 | (uint64_t)op << 29);
}

static uint32_t
fp32_FPOnePointFive(int sgn)
{
    return fp32_pack(sgn, 127, (uint64_t)1 << 22);
}

static uint64_t
fp64_FPOnePointFive(int sgn)
{
    return fp64_pack(sgn, 1023, (uint64_t)1 << 51);
}

static uint32_t
fp32_FPThree(int sgn)
{
    return fp32_pack(sgn, 128, (uint64_t)1 << 22);
}

static uint64_t
fp64_FPThree(int sgn)
{
    return fp64_pack(sgn, 1024, (uint64_t)1 << 51);
}

static uint32_t
fp32_FPTwo(int sgn)
{
    return fp32_pack(sgn, 128, 0);
}

static uint64_t
fp64_FPTwo(int sgn)
{
    return fp64_pack(sgn, 1024, 0);
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

    if (exp == 255 && (uint32_t)(mnt << 9)) {
        if (alt_hp) {
            result = fp16_zero(sgn);
        } else if (fpscr.dn) {
            result = fp16_defaultNaN();
        } else {
            result = fp16_FPConvertNaN_32(op);
        }
        if (!(mnt >> 22 & 1) || alt_hp) {
            flags |= FPLIB_IOC;
        }
    } else if (exp == 255) {
        if (alt_hp) {
            result = sgn << 15 | (uint16_t)0x7fff;
            flags |= FPLIB_IOC;
        } else {
            result = fp16_infinity(sgn);
        }
    } else if (!mnt) {
        result = fp16_zero(sgn);
    } else {
        result = fp16_round_(sgn, exp - 127 + 15,
                             mnt >> 7 | !!(uint32_t)(mnt << 25),
                             rounding, mode | alt_hp << 4, &flags);
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

    if (exp == 2047 && (uint64_t)(mnt << 12)) {
        if (alt_hp) {
            result = fp16_zero(sgn);
        } else if (fpscr.dn) {
            result = fp16_defaultNaN();
        } else {
            result = fp16_FPConvertNaN_64(op);
        }
        if (!(mnt >> 51 & 1) || alt_hp) {
            flags |= FPLIB_IOC;
        }
    } else if (exp == 2047) {
        if (alt_hp) {
            result = sgn << 15 | (uint16_t)0x7fff;
            flags |= FPLIB_IOC;
        } else {
            result = fp16_infinity(sgn);
        }
    } else if (!mnt) {
        result = fp16_zero(sgn);
    } else {
        result = fp16_round_(sgn, exp - 1023 + 15,
                             mnt >> 36 | !!(uint64_t)(mnt << 28),
                             rounding, mode | alt_hp << 4, &flags);
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
    fp16_unpack(&sgn, &exp, &mnt, op, mode, &flags);

    if (exp == 31 && !fpscr.ahp && (uint16_t)(mnt << 6)) {
        if (fpscr.dn) {
            result = fp32_defaultNaN();
        } else {
            result = fp32_FPConvertNaN_16(op);
        }
        if (!(mnt >> 9 & 1)) {
            flags |= FPLIB_IOC;
        }
    } else if (exp == 31 && !fpscr.ahp) {
        result = fp32_infinity(sgn);
    } else if (!mnt) {
        result = fp32_zero(sgn);
    } else {
        mnt = fp16_normalise(mnt, &exp);
        result = fp32_pack(sgn, exp - 15 + 127 + 5, (uint32_t)mnt << 8);
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

    if (exp == 2047 && (uint64_t)(mnt << 12)) {
        if (fpscr.dn) {
            result = fp32_defaultNaN();
        } else {
            result = fp32_FPConvertNaN_64(op);
        }
        if (!(mnt >> 51 & 1)) {
            flags |= FPLIB_IOC;
        }
    } else if (exp == 2047) {
        result = fp32_infinity(sgn);
    } else if (!mnt) {
        result = fp32_zero(sgn);
    } else {
        result = fp32_round_(sgn, exp - 1023 + 127,
                             mnt >> 20 | !!(uint64_t)(mnt << 44),
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
    fp16_unpack(&sgn, &exp, &mnt, op, mode, &flags);

    if (exp == 31 && !fpscr.ahp && (uint16_t)(mnt << 6)) {
        if (fpscr.dn) {
            result = fp64_defaultNaN();
        } else {
            result = fp64_FPConvertNaN_16(op);
        }
        if (!(mnt >> 9 & 1)) {
            flags |= FPLIB_IOC;
        }
    } else if (exp == 31 && !fpscr.ahp) {
        result = fp64_infinity(sgn);
    } else if (!mnt) {
        result = fp64_zero(sgn);
    } else {
        mnt = fp16_normalise(mnt, &exp);
        result = fp64_pack(sgn, exp - 15 + 1023 + 5, (uint64_t)mnt << 37);
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

    if (exp == 255 && (uint32_t)(mnt << 9)) {
        if (fpscr.dn) {
            result = fp64_defaultNaN();
        } else {
            result = fp64_FPConvertNaN_32(op);
        }
        if (!(mnt >> 22 & 1)) {
            flags |= FPLIB_IOC;
        }
    } else if (exp == 255) {
        result = fp64_infinity(sgn);
    } else if (!mnt) {
        result = fp64_zero(sgn);
    } else {
        mnt = fp32_normalise(mnt, &exp);
        result = fp64_pack(sgn, exp - 127 + 1023 + 8, (uint64_t)mnt << 21);
    }

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

static uint32_t
fp32_repack(int sgn, int exp, uint32_t mnt)
{
    return fp32_pack(sgn, mnt >> 23 ? exp : 0, mnt);
}

static uint64_t
fp64_repack(int sgn, int exp, uint64_t mnt)
{
    return fp64_pack(sgn, mnt >> 52 ? exp : 0, mnt);
}

static void
fp32_minmaxnum(uint32_t *op1, uint32_t *op2, int sgn)
{
    // Treat a single quiet-NaN as +Infinity/-Infinity
    if (!((uint32_t)~(*op1 << 1) >> 23) && (uint32_t)~(*op2 << 1) >> 23)
        *op1 = fp32_infinity(sgn);
    if (!((uint32_t)~(*op2 << 1) >> 23) && (uint32_t)~(*op1 << 1) >> 23)
        *op2 = fp32_infinity(sgn);
}

static void
fp64_minmaxnum(uint64_t *op1, uint64_t *op2, int sgn)
{
    // Treat a single quiet-NaN as +Infinity/-Infinity
    if (!((uint64_t)~(*op1 << 1) >> 52) && (uint64_t)~(*op2 << 1) >> 52)
        *op1 = fp64_infinity(sgn);
    if (!((uint64_t)~(*op2 << 1) >> 52) && (uint64_t)~(*op1 << 1) >> 52)
        *op2 = fp64_infinity(sgn);
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
        if ((exp1 == 255 && !mnt2) || (exp2 == 255 && !mnt1)) {
            result = fp32_FPTwo(sgn1 ^ sgn2);
        } else if (exp1 == 255 || exp2 == 255) {
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
        if ((exp1 == 2047 && !mnt2) || (exp2 == 2047 && !mnt1)) {
            result = fp64_FPTwo(sgn1 ^ sgn2);
        } else if (exp1 == 2047 || exp2 == 2047) {
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
uint32_t
fplibNeg(uint32_t op)
{
    return op ^ (uint32_t)1 << 31;
}

template <>
uint64_t
fplibNeg(uint64_t op)
{
    return op ^ (uint64_t)1 << 63;
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
uint32_t
fplibRSqrtEstimate(uint32_t op, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn, exp;
    uint32_t mnt, result;

    fp32_unpack(&sgn, &exp, &mnt, op, mode, &flags);

    if (exp == 255 && (uint32_t)(mnt << 9)) {
        result = fp32_process_NaN(op, mode, &flags);
    } else if (!mnt) {
        result = fp32_infinity(sgn);
        flags |= FPLIB_DZC;
    } else if (sgn) {
        result = fp32_defaultNaN();
        flags |= FPLIB_IOC;
    } else if (exp == 255) {
        result = fp32_zero(0);
    } else {
        exp += 8;
        mnt = fp32_normalise(mnt, &exp);
        mnt = recip_sqrt_estimate[(~exp & 1) << 7 | (mnt >> 24 & 127)];
        result = fp32_pack(0, (380 - exp) >> 1, mnt << 15);
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

    if (exp == 2047 && (uint64_t)(mnt << 12)) {
        result = fp64_process_NaN(op, mode, &flags);
    } else if (!mnt) {
        result = fp64_infinity(sgn);
        flags |= FPLIB_DZC;
    } else if (sgn) {
        result = fp64_defaultNaN();
        flags |= FPLIB_IOC;
    } else if (exp == 2047) {
        result = fp32_zero(0);
    } else {
        exp += 11;
        mnt = fp64_normalise(mnt, &exp);
        mnt = recip_sqrt_estimate[(~exp & 1) << 7 | (mnt >> 56 & 127)];
        result = fp64_pack(0, (3068 - exp) >> 1, mnt << 44);
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
        if ((exp1 == 255 && !mnt2) || (exp2 == 255 && !mnt1)) {
            result = fp32_FPOnePointFive(0);
        } else if (exp1 == 255 || exp2 == 255) {
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
        if ((exp1 == 2047 && !mnt2) || (exp2 == 2047 && !mnt1)) {
            result = fp64_FPOnePointFive(0);
        } else if (exp1 == 2047 || exp2 == 2047) {
            result = fp64_infinity(sgn1 ^ sgn2);
        } else {
            result = fp64_muladd(fp64_FPThree(0), op1, op2, -1, mode, &flags);
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
        if ((exp1 == 255 && !mnt2) || (exp2 == 255 && !mnt1)) {
            result = fp32_FPTwo(0);
        } else if (exp1 == 255 || exp2 == 255) {
            result = fp32_infinity(sgn1 ^ sgn2);
        } else {
            result = fp32_muladd(fp32_FPTwo(0), op1, op2, 0, mode, &flags);
        }
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

    if (exp == 255 && (uint32_t)(mnt << 9)) {
        result = fp32_process_NaN(op, mode, &flags);
    } else if (exp == 255) {
        result = fp32_zero(sgn);
    } else if (!mnt) {
        result = fp32_infinity(sgn);
        flags |= FPLIB_DZC;
    } else if (!((uint32_t)(op << 1) >> 22)) {
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
            assert(0);
        }
        result = overflow_to_inf ? fp32_infinity(sgn) : fp32_max_normal(sgn);
        flags |= FPLIB_OFC | FPLIB_IXC;
    } else if (fpscr.fz && exp >= 253) {
        result = fp32_zero(sgn);
        flags |= FPLIB_UFC;
    } else {
        exp += 8;
        mnt = fp32_normalise(mnt, &exp);
        int result_exp = 253 - exp;
        uint32_t fraction = (((uint32_t)1 << 19) / (mnt >> 22 | 1) + 1) >> 1;
        fraction <<= 15;
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

    if (exp == 2047 && (uint64_t)(mnt << 12)) {
        result = fp64_process_NaN(op, mode, &flags);
    } else if (exp == 2047) {
        result = fp64_zero(sgn);
    } else if (!mnt) {
        result = fp64_infinity(sgn);
        flags |= FPLIB_DZC;
    } else if (!((uint64_t)(op << 1) >> 51)) {
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
            assert(0);
        }
        result = overflow_to_inf ? fp64_infinity(sgn) : fp64_max_normal(sgn);
        flags |= FPLIB_OFC | FPLIB_IXC;
    } else if (fpscr.fz && exp >= 2045) {
        result = fp64_zero(sgn);
        flags |= FPLIB_UFC;
    } else {
        exp += 11;
        mnt = fp64_normalise(mnt, &exp);
        int result_exp = 2045 - exp;
        uint64_t fraction = (((uint32_t)1 << 19) / (mnt >> 54 | 1) + 1) >> 1;
        fraction <<= 44;
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
        if ((exp1 == 2047 && !mnt2) || (exp2 == 2047 && !mnt1)) {
            result = fp64_FPTwo(0);
        } else if (exp1 == 2047 || exp2 == 2047) {
            result = fp64_infinity(sgn1 ^ sgn2);
        } else {
            result = fp64_muladd(fp64_FPTwo(0), op1, op2, 0, mode, &flags);
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

    if (exp == 255 && (uint32_t)(mnt << 9)) {
        result = fp32_process_NaN(op, mode, &flags);
    }
    else {
        if (!mnt) { // Zero and denormals
            result = fp32_pack(sgn, 254, 0);
        } else { // Infinities and normals
            result = fp32_pack(sgn, exp ^ 255, 0);
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

    if (exp == 2047 && (uint64_t)(mnt << 12)) {
        result = fp64_process_NaN(op, mode, &flags);
    }
    else {
        if (!mnt) { // Zero and denormals
            result = fp64_pack(sgn, 2046, 0);
        } else { // Infinities and normals
            result = fp64_pack(sgn, exp ^ 2047, 0);
        }
    }

    set_fpscr0(fpscr, flags);

    return result;
}

template <>
uint32_t
fplibRoundInt(uint32_t op, FPRounding rounding, bool exact, FPSCR &fpscr)
{
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn, exp;
    uint32_t mnt, result;

    // Unpack using FPCR to determine if subnormals are flushed-to-zero:
    fp32_unpack(&sgn, &exp, &mnt, op, mode, &flags);

    // Handle NaNs, infinities and zeroes:
    if (exp == 255 && (uint32_t)(mnt << 9)) {
        result = fp32_process_NaN(op, mode, &flags);
    } else if (exp == 255) {
        result = fp32_infinity(sgn);
    } else if (!mnt) {
        result = fp32_zero(sgn);
    } else if (exp >= 150) {
        // There are no fractional bits
        result = op;
    } else {
        // Truncate towards zero:
        uint32_t x = 150 - exp >= 32 ? 0 : mnt >> (150 - exp);
        int err = exp < 118 ? 1 :
            (mnt << 1 >> (149 - exp) & 3) | (mnt << 2 << (exp - 118) != 0);
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
            assert(0);
        }

        if (x == 0) {
            result = fp32_zero(sgn);
        } else {
            exp = 150;
            mnt = fp32_normalise(x, &exp);
            result = fp32_pack(sgn, exp + 8, mnt >> 8);
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
    int mode = modeConv(fpscr);
    int flags = 0;
    int sgn, exp;
    uint64_t mnt, result;

    // Unpack using FPCR to determine if subnormals are flushed-to-zero:
    fp64_unpack(&sgn, &exp, &mnt, op, mode, &flags);

    // Handle NaNs, infinities and zeroes:
    if (exp == 2047 && (uint64_t)(mnt << 12)) {
        result = fp64_process_NaN(op, mode, &flags);
    } else if (exp == 2047) {
        result = fp64_infinity(sgn);
    } else if (!mnt) {
        result = fp64_zero(sgn);
    } else if (exp >= 1075) {
        // There are no fractional bits
        result = op;
    } else {
        // Truncate towards zero:
        uint64_t x = 1075 - exp >= 64 ? 0 : mnt >> (1075 - exp);
        int err = exp < 1011 ? 1 :
            (mnt << 1 >> (1074 - exp) & 3) | (mnt << 2 << (exp - 1011) != 0);
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
            assert(0);
        }

        if (x == 0) {
            result = fp64_zero(sgn);
        } else {
            exp = 1075;
            mnt = fp64_normalise(x, &exp);
            result = fp64_pack(sgn, exp + 11, mnt >> 11);
        }

        if (err && exact)
            flags |= FPLIB_IXC;
    }

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

static uint64_t
FPToFixed_64(int sgn, int exp, uint64_t mnt, bool u, FPRounding rounding,
             int *flags)
{
    uint64_t x;
    int err;

    if (exp > 1023 + 63) {
        *flags = FPLIB_IOC;
        return ((uint64_t)!u << 63) - !sgn;
    }

    x = lsr64(mnt << 11, 1023 + 63 - exp);
    err = (exp > 1023 + 63 - 2 ? 0 :
           (lsr64(mnt << 11, 1023 + 63 - 2 - exp) & 3) |
           !!(mnt << 11 & (lsl64(1, 1023 + 63 - 2 - exp) - 1)));

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
        assert(0);
    }

    if (u ? sgn && x : x > ((uint64_t)1 << 63) - !sgn) {
        *flags = FPLIB_IOC;
        return ((uint64_t)!u << 63) - !sgn;
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
    if (u ? x >= (uint64_t)1 << 32 :
        !(x < (uint64_t)1 << 31 ||
          (uint64_t)-x <= (uint64_t)1 << 31)) {
        *flags = FPLIB_IOC;
        x = ((uint32_t)!u << 31) - !sgn;
    }
    return x;
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
    if (exp == 255 && (uint32_t)(mnt << 9)) {
        flags = FPLIB_IOC;
        result = 0;
    } else {
        result = FPToFixed_32(sgn, exp + 1023 - 127 + fbits,
                              (uint64_t)mnt << (52 - 23), u, rounding, &flags);
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
    if (exp == 2047 && (uint64_t)(mnt << 12)) {
        flags = FPLIB_IOC;
        result = 0;
    } else {
        result = FPToFixed_32(sgn, exp + fbits, mnt, u, rounding, &flags);
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
    if (exp == 255 && (uint32_t)(mnt << 9)) {
        flags = FPLIB_IOC;
        result = 0;
    } else {
        result = FPToFixed_64(sgn, exp + 1023 - 127 + fbits,
                              (uint64_t)mnt << (52 - 23), u, rounding, &flags);
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
    if (exp == 2047 && (uint64_t)(mnt << 12)) {
        flags = FPLIB_IOC;
        result = 0;
    } else {
        result = FPToFixed_64(sgn, exp + fbits, mnt, u, rounding, &flags);
    }

    set_fpscr0(fpscr, flags);

    return result;
}

static uint32_t
fp32_cvtf(uint64_t a, int fbits, int u, int mode, int *flags)
{
    int x_sgn = !u && a >> 63;
    int x_exp = 190 - fbits;
    uint64_t x_mnt = x_sgn ? -a : a;

    // Handle zero:
    if (!x_mnt) {
        return fp32_zero(0);
    }

    // Normalise and convert to 32 bits, collapsing error into bottom bit:
    x_mnt = fp64_normalise(x_mnt, &x_exp);
    x_mnt = x_mnt >> 31 | !!(uint32_t)(x_mnt << 1);

    return fp32_round(x_sgn, x_exp, x_mnt, mode, flags);
}

static uint64_t
fp64_cvtf(uint64_t a, int fbits, int u, int mode, int *flags)
{
    int x_sgn = !u && a >> 63;
    int x_exp = 1024 + 62 - fbits;
    uint64_t x_mnt = x_sgn ? -a : a;

    // Handle zero:
    if (!x_mnt) {
        return fp64_zero(0);
    }

    x_mnt = fp64_normalise(x_mnt, &x_exp);

    return fp64_round(x_sgn, x_exp, x_mnt << 1, mode, flags);
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

}
