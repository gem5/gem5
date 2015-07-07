/*
 * Copyright (c) 2014-2015 ARM Limited
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
    nomali_error_t error = NOMALI_E_OK;
    uint32_t value;

    E_NOMALI_TEST("nomali_create", nomali_create(&h, &cfg));
    if (error != NOMALI_E_OK)
        test_bail("Failed to create NoMail instance!");

    E_NOMALI_TEST("reg_read(GPU_ID)",
                  nomali_reg_read(h, &value, GPU_CONTROL_REG(GPU_ID)));
    if (value != ((GPU_ID_PI_T60X << 16) | 0x10)) {
        test_fail("GPU_ID");
    } else
        test_ok("GPU_ID");

    E_NOMALI_TEST("nomali_destroy", nomali_destroy(h));

    return 0;
}
