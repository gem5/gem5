/*
 * Copyright (c) 2013, Andreas Sandberg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _FPBITS_H
#define _FPBITS_H

#include <stdint.h>

#define FP80_FRAC_BITS 63
#define FP80_INT_BIT  0x8000000000000000ULL
#define FP80_QNAN_BIT  0x4000000000000000ULL
#define FP80_FRAC_MASK  0x7fffffffffffffffULL
#define FP80_EXP_MASK 0x7fff
#define FP80_SIGN_BIT 0x8000
#define FP80_EXP_BIAS 0x3fff

#define FP80_EXP_SPECIAL 0x7fff
#define FP80_FRAC_SNAN  0x3fffffffffffffffULL
#define FP80_FRAC_QNAN  0x7fffffffffffffffULL
#define FP80_FRAC_QNANI 0x4000000000000000ULL


#define FP64_EXP_SHIFT 52
#define FP64_FRAC_BITS 52
#define FP64_SIGN_BIT  0x8000000000000000ULL
#define FP64_EXP_MASK  0x7ff0000000000000ULL
#define FP64_FRAC_MASK 0x000fffffffffffffULL
#define FP64_EXP_BIAS 0x3ff

#define FP64_EXP_SPECIAL 0x7ff
#define FP64_FRAC_SNAN  0x0007ffffffffffffULL
#define FP64_FRAC_QNAN  0x000fffffffffffffULL
#define FP64_FRAC_QNANI 0x0008000000000000ULL

#define BUILD_IFP64(sign, frac, exp)                            \
    ((sign) ? FP64_SIGN_BIT : 0) |                              \
    (((uint64_t)(exp) << FP64_EXP_SHIFT) & FP64_EXP_MASK) |     \
    ((frac) & FP64_FRAC_MASK)

#define BUILD_FP64(sign, frac, exp)                             \
    { .bits = BUILD_IFP64(sign, frac, exp) }

static inline fp64_t
build_fp64(int sign, uint64_t frac, int exp)
{
    const fp64_t f = BUILD_FP64(sign, frac, exp);

    return f;
}

#define BUILD_FP80_SE(sign, exp)                                \
    ((sign) ? FP80_SIGN_BIT : 0) |                              \
    ((exp) & FP80_EXP_MASK)

#define BUILD_FP80_FI(frac, exp)                                \
    ((exp) ? FP80_INT_BIT : 0) |                                \
    ((frac) & FP80_FRAC_MASK)

#define BUILD_FP80(sign, frac, exp)                             \
    {                                                           \
        .repr.pad = { 0 },                                      \
        .repr.se = BUILD_FP80_SE(sign, exp),                    \
        .repr.fi = BUILD_FP80_FI(frac, exp)                     \
    }

static inline fp80_t
build_fp80(int sign, uint64_t frac, int exp)
{
    const fp80_t f = BUILD_FP80(sign, frac, exp);

    return f;
}

#define FP80_FRAC(fp80)                                         \
    (fp80.repr.fi & FP80_FRAC_MASK)

#define FP80_EXP(fp80)                                          \
    (fp80.repr.se & FP80_EXP_MASK)

#define FP64_FRAC(fp64)                                         \
    (fp64.bits & FP64_FRAC_MASK)

#define FP64_EXP(fp80)                                          \
    ((fp64.bits & FP64_EXP_MASK) >> FP64_EXP_SHIFT)


#endif
