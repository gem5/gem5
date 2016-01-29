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

static void
on_int(nomali_handle_t h, void *usr, nomali_int_t intno, int set)
{
    test_diag("on_int: intno: %i, set: %i", intno, set);
    *(int*)usr = !!set;
}

static void
test_gpu_int(nomali_handle_t h)
{
    int int_triggered = 0;
    nomali_callback_t int_callback = {
        .type = NOMALI_CALLBACK_INT,
        .usr = &int_triggered,
        .func.interrupt = on_int,
    };

    nomali_callback_t int_null_callback = {
        .type = NOMALI_CALLBACK_INT,
        .usr = NULL,
        .func.interrupt = NULL,
    };

    /*
     * Raise an interrupt without callbacks
     */
    E_NOMALI_BAIL(nomali_reg_write(h,
                                   GPU_CONTROL_REG(GPU_IRQ_CLEAR),
                                   GPU_IRQ_REG_ALL));

    E_NOMALI_BAIL(nomali_reg_write(h, GPU_CONTROL_REG(GPU_IRQ_MASK),
                                   GPU_FAULT));

    E_NOMALI_BAIL(nomali_reg_write(h, GPU_CONTROL_REG(GPU_IRQ_RAWSTAT),
                                   GPU_FAULT));

    E_NOMALI_BAIL(nomali_reg_write(h,
                                   GPU_CONTROL_REG(GPU_IRQ_CLEAR),
                                   GPU_IRQ_REG_ALL));

    /*
     * Register callbacks and raise interrupt again.
     */
    E_NOMALI_BAIL(nomali_set_callback(h, &int_callback));
    if (int_triggered != 0) {
        test_diag("Got spurious interrupt\n");
        test_fail("gpu_int");
    }

    E_NOMALI_BAIL(nomali_reg_write(h, GPU_CONTROL_REG(GPU_IRQ_RAWSTAT),
                                   GPU_FAULT));
    if (int_triggered == 1) {
        test_ok("gpu_int");
    } else {
        test_fail("gpu_int");
    }
    int_triggered = 0;


    /*
     * Register mask interrupts and raise interrupt again.
     */
    E_NOMALI_BAIL(nomali_reg_write(h,
                                   GPU_CONTROL_REG(GPU_IRQ_CLEAR),
                                   GPU_IRQ_REG_ALL));
    E_NOMALI_BAIL(nomali_reg_write(h, GPU_CONTROL_REG(GPU_IRQ_MASK),
                                   0));
    E_NOMALI_BAIL(nomali_reg_write(h, GPU_CONTROL_REG(GPU_IRQ_RAWSTAT),
                                   GPU_FAULT));
    if (int_triggered == 0) {
        test_ok("gpu_int_masked");
    } else {
        test_fail("gpu_int_maked");
    }
    E_NOMALI_BAIL(nomali_reg_write(h,
                                   GPU_CONTROL_REG(GPU_IRQ_CLEAR),
                                   GPU_IRQ_REG_ALL));
    E_NOMALI_BAIL(nomali_set_callback(h, &int_null_callback));
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

    nomali_handle_t h;

    E_NOMALI_BAIL(nomali_create(&h, &cfg));

    test_gpu_int(h);

    E_NOMALI_BAIL(nomali_destroy(h));

    return 0;
}
