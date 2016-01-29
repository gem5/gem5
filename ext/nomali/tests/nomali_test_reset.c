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
    E_NOMALI_TEST_REG("AS" # n "_STATUS", h, MMU_AS_REG(n, ASn_STATUS), \
                      value == 0)

static void
on_reset(nomali_handle_t h, void *usr)
{
    E_NOMALI_BAIL(nomali_reg_write_raw(h, 0x0000, 0xC0FFEE));
}

int
main(int argc, char **argv)
{
    const nomali_config_t cfg = {
        .type = NOMALI_GPU_T60X,
        .ver_maj = 0,
        .ver_min = 1,
        .ver_status = 0,
    };

    nomali_callback_t reset_callback = {
        .type = NOMALI_CALLBACK_RESET,
        .usr = NULL,
        .func.reset = &on_reset,
    };

    nomali_handle_t h;
    nomali_error_t error;

    E_NOMALI_BAIL(nomali_create(&h, &cfg));

    NOMALI_TEST_REG("gpu_id", h, 0x0000, value == 0x69560010);

    E_NOMALI_TEST("reg_callback", nomali_set_callback(h, &reset_callback));
    E_NOMALI_BAIL(nomali_reset(h));

    NOMALI_TEST_REG("custom_gpu_id", h, 0x0000, value == 0xC0FFEE);

    E_NOMALI_BAIL(nomali_destroy(h));


    return 0;
}
