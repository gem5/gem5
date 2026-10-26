/*
 * Copyright (c) 2026 Akanksha Chaudhari
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Host program for the SALAM kernel harness. inputs.h is written by the
// test and defines N_IN, N_OUT, N_RUNS and inputs[].

#include <stdint.h>
#include <stdio.h>

#include <gem5/m5ops.h>

#include "../harness_clstr_hw_defines.h"
#include "bench.h"
#include "inputs.h"

volatile uint8_t *top = (uint8_t *)KERNEL;
volatile int stage = 0;

int
main(void)
{
    volatile int32_t *in = (volatile int32_t *)IN;
    volatile int32_t *out = (volatile int32_t *)OUT;

    for (int i = 0; i < N_IN; i++) {
        in[i] = inputs[i];
    }
    // RegisterBank writes become visible after its delta time; wait
    // for the inputs before starting the kernel.
    for (int i = 0; i < N_IN; i++) {
        while (in[i] != inputs[i])
            ;
    }
    for (int run = 0; run < N_RUNS; run++) {
        stage = 0;
        *top = 0x01;
        while (stage < 1)
            ;
        for (int i = 0; i < N_OUT; i++) {
            printf("RUN[%d] OUT[%d]=%d 0x%08x\n", run, i, (int)out[i],
                   (unsigned)out[i]);
        }
    }
    m5_exit(0);
    return 0;
}
