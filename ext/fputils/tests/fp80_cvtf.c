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

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static void
test_cvtf(const char *name, double fin)
{
    fp80_t v80 = fp80_cvfd(fin);
    double v64 = fp80_cvtd(v80);

    test_diag("Conversion '%e' -> fp80 -> double: %e", fin, v64);

    printf("# ");
    fp80_debug_dump(stdout, v80);
    if (v64 == fin ||
        (isnan(fin) && isnan(v64) && signbit(fin) == signbit(v64))) {
        test_ok(name);
    } else {
        test_diag("MISMATCH: %e != %e", fin, v64);
        test_fail(name);
    }
}

static void
test_cvtf_exp(const char *name, double x, int exp)
{
    double val = ldexp(x, exp);
    test_cvtf(name, val);
}

int
main(int argc, char *argv[])
{
    test_init(9);

    test_cvtf("+inf", -INFINITY);
    test_cvtf("-inf", INFINITY);
    test_cvtf("+nan", NAN);
    test_cvtf("-nan", -NAN);
    test_cvtf("+0", 0);
    test_cvtf("PI", M_PI);

    test_cvtf_exp("smallest normal", 1.0, -1022);
    test_cvtf_exp("denormal1", 0.5, -1022);
    test_cvtf_exp("denormal2", 0.25, -1022);

    test_exit();
}
