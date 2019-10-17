/*
 * Copyright (c) 2013 Andreas Sandberg
 * All rights reserved
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
 * Authors: Andreas Sandberg
 */

#include <fputils/fp80.h>
#include <fputils/fp64.h>
#include "fpbits.h"

#include <assert.h>
#include <stdint.h>

#include <stdio.h>


const fp80_t fp80_pinf  = BUILD_FP80(0, 0,               FP80_EXP_SPECIAL);
const fp80_t fp80_ninf  = BUILD_FP80(1, 0,               FP80_EXP_SPECIAL);
const fp80_t fp80_qnan  = BUILD_FP80(0, FP80_FRAC_QNAN,  FP80_EXP_SPECIAL);
const fp80_t fp80_qnani = BUILD_FP80(1, FP80_FRAC_QNANI, FP80_EXP_SPECIAL);
const fp80_t fp80_snan  = BUILD_FP80(0, FP80_FRAC_SNAN,  FP80_EXP_SPECIAL);
const fp80_t fp80_nan   = BUILD_FP80(0, FP80_FRAC_QNAN,  FP80_EXP_SPECIAL);

int
fp80_sgn(fp80_t fp80)
{
    return (fp80.repr.se & FP80_SIGN_BIT) ? -1 : 1;
}

int
fp80_isspecial(fp80_t fp80)
{
    const int exp = FP80_EXP(fp80);

    return exp == FP80_EXP_SPECIAL;
}

int
fp80_isinf(fp80_t fp80)
{
    const uint64_t frac = FP80_FRAC(fp80);

    return fp80_isspecial(fp80) && frac == 0 ? fp80_sgn(fp80) : 0;
}


int
fp80_isqnan(fp80_t fp80)
{
    const uint64_t frac = FP80_FRAC(fp80);

    return fp80_isspecial(fp80) && (frac & FP80_QNAN_BIT);
}

int
fp80_isqnani(fp80_t fp80)
{
    const uint64_t frac_low = fp80.repr.fi & (FP80_FRAC_MASK >> 1);

    return fp80_isqnan(fp80) && (fp80.repr.se & FP80_SIGN_BIT) && !frac_low;
}

int
fp80_issnan(fp80_t fp80)
{
    const uint64_t frac = FP80_FRAC(fp80);

    return fp80_isspecial(fp80) && !(frac & FP80_QNAN_BIT) && frac;
}

int
fp80_isfinite(fp80_t fp80)
{
    return !fp80_isnan(fp80) && !fp80_isinf(fp80);
}

int
fp80_isnan(fp80_t fp80)
{
    return fp80_issnan(fp80) || fp80_isqnan(fp80) ? fp80_sgn(fp80) : 0;
}

int
fp80_iszero(fp80_t fp80)
{
    return fp80.repr.fi == 0 && FP80_EXP(fp80) == 0 ? fp80_sgn(fp80) : 0;
}

int
fp80_isnormal(fp80_t fp80)
{
    return FP80_EXP(fp80) != 0 && !fp80_isspecial(fp80) ?
        fp80_sgn(fp80) : 0;
}

int
fp80_issubnormal(fp80_t fp80)
{
    return FP80_FRAC(fp80) && FP80_EXP(fp80) == 0 ? fp80_sgn(fp80) : 0;
}

int
fp80_classify(fp80_t fp80)
{
    if (fp80_issubnormal(fp80)) {
        return FP_SUBNORMAL;
    } else if (fp80_iszero(fp80)) {
        return FP_ZERO;
    } else if (fp80_isinf(fp80)) {
        return FP_INFINITE;
    } else if (fp80_isnan(fp80)) {
        return FP_NAN;
    } else {
        assert(fp80_isfinite(fp80));
        return FP_NORMAL;
    }
}

double
fp80_cvtd(fp80_t fp80)
{
    return fp80_cvtfp64(fp80).value;
}

fp64_t
fp80_cvtfp64(fp80_t fp80)
{
    const int sign = fp80.repr.se & FP80_SIGN_BIT;

    if (!fp80_isspecial(fp80)) {
        const uint64_t frac = fp80.repr.fi;
        const int unb_exp = FP80_EXP(fp80) - FP80_EXP_BIAS;
        const int fp64_exp = unb_exp + FP64_EXP_BIAS;
        const uint64_t fp64_frac = frac >> (FP80_FRAC_BITS - FP64_FRAC_BITS);

        if (fp64_exp > 0 && fp64_exp < FP64_EXP_SPECIAL) {
            /* These numbers fall in the range of what we can express
             * as normals */
            return build_fp64(sign, fp64_frac, fp64_exp);
        } else if (fp64_exp <= 0) {
            uint64_t fp64_denormal_frac = -64 < fp64_exp
                // -64 < fp_exp <= 0, so safe to bitshift by -fp_exp
                ? fp64_frac >> (-fp64_exp)
                : 0;
            /* Generate a denormal or zero */
            return build_fp64(sign, fp64_denormal_frac, 0);
        } else {
            /* Infinity */
            return build_fp64(sign, 0, FP64_EXP_SPECIAL);
        }
    } else {
        if (fp80_isinf(fp80)) {
            return build_fp64(sign, 0, FP64_EXP_SPECIAL);
        } else if (fp80_issnan(fp80)) {
            return fp80_sgn(fp80) > 0 ? fp64_snan : fp64_nsnan;
        } else if (fp80_isqnani(fp80)) {
            return fp64_qnani;
        } else {
            assert(fp80_isqnan(fp80));
            return fp80_sgn(fp80) > 0 ? fp64_qnan : fp64_nqnan;
        }
    }
}

fp80_t
fp80_cvfd(double value)
{
    const fp64_t fp64 = { .value = value };

    return fp80_cvffp64(fp64);
}

fp80_t
fp80_cvffp64(fp64_t fp64)
{
    const uint64_t frac = FP64_FRAC(fp64);
    const unsigned exp = FP64_EXP(fp64);
    const int unb_exp = exp - FP64_EXP_BIAS;
    const uint64_t fp80_frac = frac << (FP80_FRAC_BITS - FP64_FRAC_BITS);

    if (exp != 0) {
        // Normal, inf, nan
        const unsigned fp80_exp = exp == FP64_EXP_SPECIAL ?
            FP80_EXP_SPECIAL : (unb_exp + FP80_EXP_BIAS);
        const fp80_t fp80 = BUILD_FP80(fp64.bits & FP64_SIGN_BIT,
                                       fp80_frac, fp80_exp);
        return fp80;
    } else if (exp == 0 && frac == 0) {
        // Zero
        const fp80_t fp80 = BUILD_FP80(fp64.bits & FP64_SIGN_BIT, 0, 0);
        return fp80;
    } else {
        // Denormal
        uint64_t fp80_fi = fp80_frac;
        int shift_amt = 0;
        while (!(fp80_fi & FP80_INT_BIT)) {
            fp80_fi <<= 1;
            ++shift_amt;
        }
        const unsigned fp80_exp = (unb_exp - shift_amt) + FP80_EXP_BIAS;
        const fp80_t fp80 = BUILD_FP80(fp64.bits & FP64_SIGN_BIT,
                                       fp80_fi, fp80_exp);
        return fp80;
    }
}

void
fp80_debug_dump(FILE *fout, fp80_t fp80)
{
    fprintf(fout, "sgn: %i, int: %i, frac: 0x%llx, exp: 0x%x (%i)\n",
            fp80_sgn(fp80), !!(fp80.repr.fi & FP80_INT_BIT), FP80_FRAC(fp80),
            FP80_EXP(fp80), FP80_EXP(fp80) - FP80_EXP_BIAS);
}
