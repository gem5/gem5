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

#ifndef _TEST_HELPERS
#define _TEST_HELPERS 1

#include <stdio.h>

#ifdef __cplusplus
#extern "C" {
#endif

extern unsigned test_current;
extern unsigned test_count;
extern unsigned test_fail_count;

void test_init(unsigned no_tests);
void test_exit()
    __attribute__((noreturn));

void test_bail(const char *fmt, ...)
    __attribute__((format (printf, 1, 2), noreturn));

void test_diag(const char *fmt, ...)
    __attribute__((format (printf, 1, 2)));

void test_ok(const char *test);

void test_fail(const char *test);

void test_skip(const char *test, const char *fmt_why, ...)
    __attribute__((format (printf, 2, 3)));

void test_todo(const char *test, const char *fmt_why, ...)
    __attribute__((format (printf, 2, 3)));

#ifdef __cplusplus
}
#endif

#endif
