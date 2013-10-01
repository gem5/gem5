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

#include <fputils/fp80.h>

#include "test_helper.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* We provide our own version of isinf_sgn since the C99 standard
 * doesn't guarantee that isinf() returns the sign of the infinity
 * (most implementations do). */
static inline int
isinf_sgn(double x)
{
    return isinf(x) ? (signbit(x) ? -1 : 1) : 0;
}

static void
test_fp80_cvtd_class(const char *name, fp80_t fin, int class)
{
    double d = fp80_cvtd(fin);
    if (fpclassify(d) != class) {
        test_diag("wrong class");
        test_fail(name);
    } else {
        test_ok(name);
    }
}

static void
test_fp80_cvtd_inf(const char *name, fp80_t fin, int expected_inf_class)
{
    double d = fp80_cvtd(fin);
    if (isinf_sgn(d) != expected_inf_class) {
        test_diag("wrong infinity type");
        test_fail(name);
    } else {
        test_ok(name);
    }
}

int
main(int argc, char *argv[])
{
    test_init(6);

    test_fp80_cvtd_inf("fp80->double +inf", fp80_pinf, 1);
    test_fp80_cvtd_inf("fp80->double -inf", fp80_ninf, -1);
    test_fp80_cvtd_class("fp80->double qnan", fp80_qnan, FP_NAN);
    test_fp80_cvtd_class("fp80->double qnani", fp80_qnani, FP_NAN);
    test_fp80_cvtd_class("fp80->double snan", fp80_snan, FP_NAN);
    test_fp80_cvtd_class("fp80->double nan", fp80_nan, FP_NAN);

    test_exit();
}
