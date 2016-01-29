/*
 * Copyright (c) 2016 ARM Limited
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

#include <libnomali/nomali.h>
#include <inttypes.h>

#include "nomali_test_helpers.h"
#include "../lib/mali_midg_regmap.h"

#define TEST_ASn_STATUS(n)                                              \
    NOMALI_TEST_REG("AS" # n "_STATUS", h, MMU_AS_REG(n, ASn_STATUS),   \
                    value == 0)

int
main(int argc, char **argv)
{
    const nomali_config_t cfg = {
        .type = NOMALI_GPU_T60X,
        .ver_maj = 0,
        .ver_min = 1,
        .ver_status = 0,
    };
    nomali_handle_t h;
    uint32_t value;

    E_NOMALI_BAIL(nomali_create(&h, &cfg));

    TEST_ASn_STATUS(0);
    TEST_ASn_STATUS(1);
    TEST_ASn_STATUS(2);
    TEST_ASn_STATUS(3);
    TEST_ASn_STATUS(4);
    TEST_ASn_STATUS(5);
    TEST_ASn_STATUS(6);
    TEST_ASn_STATUS(7);
    TEST_ASn_STATUS(8);
    TEST_ASn_STATUS(9);
    TEST_ASn_STATUS(10);
    TEST_ASn_STATUS(11);
    TEST_ASn_STATUS(12);
    TEST_ASn_STATUS(13);
    TEST_ASn_STATUS(14);
    TEST_ASn_STATUS(15);

    E_NOMALI_BAIL(nomali_destroy(h));


    return 0;
}
