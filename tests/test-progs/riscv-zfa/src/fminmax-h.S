/*
 * Copyright (c) 2026 Donmakino LLC
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* Rebuild from the repository root:
 * riscv64-unknown-elf-gcc -march=rv64gc_zfh -mabi=lp64d -nostdlib -static \
 *     -Wl,--build-id=none,-s -o \
 *     tests/test-progs/riscv-zfa/bin/riscv/linux/fminmax-h \
 *     tests/test-progs/riscv-zfa/src/fminmax-h.S
 */

.option norelax
.text
.globl _start
_start:
    csrw fcsr, zero
    li t0, 0x3c00                 /* 1.0h */
    fmv.h.x f0, t0
    li t1, 0x4000                 /* 2.0h */
    fmv.h.x f1, t1

    /* Zfa 1.0 sets bit 13 of FMIN.H/FMAX.H, giving funct3=2/3. */
    .word 0x2c103153              /* fmaxm.h f2, f0, f1 */
    fmv.x.h t2, f2
    bne t2, t1, fail
    .word 0x2c102153              /* fminm.h f2, f0, f1 */
    fmv.x.h t2, f2
    bne t2, t0, fail
    csrr t2, fflags
    bnez t2, fail

    li a0, 0
    j exit
fail:
    li a0, 1
exit:
    li a7, 93
    ecall
