/*
 * Copyright (c) 2013 Andreas Sandberg
 * All rights reserved
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Andreas Sandberg
 */

#include "test_helpers.h"

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
