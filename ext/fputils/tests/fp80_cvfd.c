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

#include "test_helper.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static void
test_cvfd_class(const char *name, double fin)
{
    test_diag("converting '%e' -> fp80...", fin);
    fp80_t v = fp80_cvfd(fin);
    int class_ok;
    int fp64_class = fpclassify(fin);
    int class = fp64_class == FP_SUBNORMAL ? FP_NORMAL : fp64_class;
    int fp80_class = fp80_classify(v);

    printf("# ");
    fp80_debug_dump(stdout, v);
    test_diag("isnan: %i, isinf: %i, iszero: %i, isnormal: %i, issubnormal: %i",
              fp80_isnan(v), fp80_isinf(v), fp80_iszero(v),
              fp80_isnormal(v), fp80_issubnormal(v));
    test_diag("class(fp64): %i, expected class: %i, class(fp80): %i",
              fp64_class, class, fp80_classify(v));
    switch (class) {
    case FP_NAN:
        class_ok = fp80_isnan(v) && !fp80_isinf(v) && !fp80_iszero(v) &&
            !fp80_isnormal(v) && !fp80_issubnormal(v);
        break;

    case FP_INFINITE:
        class_ok = !fp80_isnan(v) && fp80_isinf(v) && !fp80_iszero(v) &&
            !fp80_isnormal(v) && !fp80_issubnormal(v);
        break;
    case FP_ZERO:
        class_ok = !fp80_isnan(v) && !fp80_isinf(v) && fp80_iszero(v) &&
            !fp80_isnormal(v) && !fp80_issubnormal(v);
        break;

    case FP_SUBNORMAL:
        class_ok = !fp80_isnan(v) && !fp80_isinf(v) && !fp80_iszero(v) &&
            !fp80_isnormal(v) && fp80_issubnormal(v);
        break;

    case FP_NORMAL:
        class_ok = !fp80_isnan(v) && !fp80_isinf(v) && !fp80_iszero(v) &&
            fp80_isnormal(v) && !fp80_issubnormal(v);
        break;

    default:
        test_bail("unexpected FP class (%i)", class);
    }

    if (!class_ok) {
        test_diag("inconsistent classification");
        test_fail(name);
    } else if (fp80_class != class) {
        test_diag("class mismatch");
        test_fail(name);
    } else {
        test_ok(name);
    }
}

static void
test_cvfd_class_exp(const char *name, double x, int exp)
{
    double val = ldexp(x, exp);
    test_cvfd_class(name, val);
}

int
main(int argc, char *argv[])
{
    test_init(9);

    test_cvfd_class("double->fp80 +inf", -INFINITY);
    test_cvfd_class("double->fp80 -inf", INFINITY);
    test_cvfd_class("double->fp80 +nan", NAN);
    test_cvfd_class("double->fp80 -nan", -NAN);
    test_cvfd_class("double->fp80 +0", 0);
    test_cvfd_class("double->fp80 PI", M_PI);

    test_cvfd_class_exp("double->fp80 smallest normal", 1.0, -1022);
    test_cvfd_class_exp("double->fp80 denormal1", 0.5, -1022);
    test_cvfd_class_exp("double->fp80 denormal2", 0.25, -1022);

    test_exit();
}
