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

#include "test_helper.h"

#include <assert.h>
#include <stdarg.h>
#include <stdlib.h>

unsigned test_current = 0;
unsigned test_count = 0;
unsigned test_fail_count = 0;

void
test_init(unsigned no_tests)
{
    assert(test_count == 0 && test_current == 0);

    test_count = no_tests;
    test_current = 1;
    test_fail_count = 0;

    printf("1..%u\n", no_tests);
}

void
test_exit()
{
    if (test_fail_count)
        exit(EXIT_FAILURE);
    else
        exit(EXIT_SUCCESS);
}

void
test_bail(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);

    printf("Bail out! ");
    vprintf(fmt, ap);
    printf("\n");

    va_end(ap);

    exit(EXIT_FAILURE);
}

void
test_diag(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);

    printf("# ");
    vprintf(fmt, ap);
    printf("\n");

    va_end(ap);
}

static void
test_vstatus(const char *status, const char *test,
             const char *directive,
             const char *fmt_why, va_list ap)
{
    printf("%s %i", status, test_current);

    if (test && test[0] != '\0')
        printf(" - %s", test);

    if (directive && directive[0] != '\0') {
        printf(" # %s ", directive);
        if (fmt_why && fmt_why[0] != '\0')
            vprintf(fmt_why, ap);
    }
    printf("\n");

    ++test_current;
}

static void __attribute__((format (printf, 4, 5)))
test_status(const char *status, const char *test,
            const char *directive,
            const char *fmt_why, ...)
{
    va_list ap;
    va_start(ap, fmt_why);

    test_vstatus(status, test, directive, fmt_why, ap);

    va_end(ap);
}

void
test_ok(const char *test)
{
    test_status("ok", test, NULL, NULL);
}

void
test_fail(const char *test)
{
    test_status("not ok", test, NULL, NULL);
    ++test_fail_count;
}

void
test_skip(const char *test, const char *fmt_why, ...)
{
    va_list ap;
    va_start(ap, fmt_why);

    test_vstatus("ok", test, "SKIP", fmt_why, ap);

    va_end(ap);
}

void
test_todo(const char *test, const char *fmt_why, ...)
{
    va_list ap;
    va_start(ap, fmt_why);

    test_vstatus("not ok", test, "TODO", fmt_why, ap);

    va_end(ap);

    ++test_fail_count;
}
