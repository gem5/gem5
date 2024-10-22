// SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-Only
/*
 * Copyright (c) 2024 The Regents of the University of California
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

#include <linux/kernel.h>

#include "gem5_bridge.h"
#include "gem5_poke.h"

u64 gem5_poke_opcode;
u64 gem5_poke_retval;

u64 noinline gem5_poke_func(u64 a, u64 b, u64 c, u64 d, u64 e, u64 f)
{
#if defined(__x86_64__)
    /* 64-bit x86 */
    asm volatile(
        /* Assembly to build and access poke addr, extracting retval */
        "movq %[mmio], %%r10 \n"
        "movq %[opcode], %%r11 \n"
        "shl  $8, %%r11 \n"
        "movq (%%r10, %%r11, 1), %%rax \n"
        "movq %%rax, %[retval] \n"
        /* Output variables */
          : [retval] "=m" (gem5_poke_retval)
        /* Input variables */
          : [mmio] "m" (gem5_bridge_mmio), [opcode] "m" (gem5_poke_opcode)
        /* Clobbers from this asm fragment */
          : "%rax", "%r10", "%r11",
        /* Clobbers to prevent changing calling-convention arg registers */
            "%rdi", "%rsi", "%rdx", "%rcx", "%r8", "%r9"
    );
#elif defined(__aarch64__)
    /* 64-bit ARM */
    asm volatile(
        /* Assembly to build and access poke addr, extracting retval */
        "ldr x10, %[mmio] \n"
        "ldr x11, %[opcode] \n"
        "lsl x11, x11, #8 \n"
        "ldr x0, [x10, x11] \n"
        "str x0, %[retval] \n"
        /* Output variables */
          : [retval] "=m" (gem5_poke_retval)
        /* Input variables */
          : [mmio] "m" (gem5_bridge_mmio), [opcode] "m" (gem5_poke_opcode)
        /* Clobbers from this asm fragment */
          : "x0", "x10", "x11",
        /* Clobbers to prevent changing calling-convention arg registers */
            /* "x0", */ "x1", "x2", "x3", "x4", "x5"
    );
#else
    pr_err("%s: unsupported architecture\n", __func__);
#endif

    return gem5_poke_retval;
}
