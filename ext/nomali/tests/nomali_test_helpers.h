/*
 * Copyright (c) 2014-2016 ARM Limited
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

#ifndef _TESTS_NOMALI_TEST_HELPERS_H
#define _TESTS_NOMALI_TEST_HELPERS_H

#include <libnomali/nomali.h>
#include "test_helpers.h"

#define E_NOMALI_BAIL(c)                                \
    do {                                                \
        nomali_error_t error;                           \
        if ((error = (c)) != NOMALI_E_OK) {             \
            test_bail(# c " failed: %s (%i)",           \
                      nomali_errstr(error), error);     \
        }                                               \
    } while (0)

#define E_NOMALI_TEST(t, c)                             \
    do {                                                \
        if ((error = (c)) != NOMALI_E_OK) {             \
            test_diag(# c " failed: %s (%i)",           \
                      nomali_errstr(error), error);     \
            test_fail(t);                               \
        } else {                                        \
            test_ok(t);                                 \
        }                                               \
    } while (0)

#define NOMALI_TEST_REG(t, handle, reg, test)                           \
    do {                                                                \
        uint32_t value;                                                 \
        E_NOMALI_BAIL(                                                  \
            nomali_reg_read(handle, &value, (reg)));                    \
        if (!(test)) {                                                  \
            test_fail(t);                                               \
        } else {                                                        \
            test_ok(t);                                                 \
        }                                                               \
    } while (0)



#endif /* _TESTS_NOMALI_TEST_HELPERS_H */
