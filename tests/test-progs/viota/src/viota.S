/*
 * Copyright (c) 2026 Donmakino LLC
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Build the checked-in binary with:
 * From tests/test-progs/viota:
 * riscv64-unknown-elf-gcc -march=rv64gcv -mabi=lp64d -c \
 *     -o viota.o src/viota.S
 * riscv64-unknown-elf-gcc -march=rv64gcv -mabi=lp64d -nostdlib -static \
 *     -Wl,--build-id=none -o bin/riscv/linux/viota viota.o
 */
.option norelax
.text
.globl _start
_start:
    vsetivli zero, 16, e32, m2, tu, mu
    vmset.m v0
    vmv.v.i v16, 0
    li t0, 0x3f800000
    fmv.w.x f1, t0
    li t0, 0x40400000
    fmv.w.x f2, t0
    # Delay only the first destination register so piece 1 can execute first.
    fdiv.s f0, f1, f2
    vfmv.s.f v16, f0
    viota.m v16, v0
    la a1, result
    vse32.v v16, (a1)
    la a2, unmasked_expected
    jal ra, check

    # Only enabled source bits contribute, even before a piece's start.
    vmv.v.i v16, -1
    li t0, 0x6db7
    vmv.s.x v2, t0
    li t0, 0x5555
    vmv.s.x v0, t0
    vsetivli zero, 13, e32, m2, tu, mu
    fdiv.s f0, f1, f2
    vfmv.s.f v16, f0
    viota.m v16, v2, v0.t
    vsetivli zero, 16, e32, m2, tu, mu
    la a1, result
    vse32.v v16, (a1)
    la a2, masked_expected
    jal ra, check
    li a0, 1
    la a1, passed
    li a2, 12
    li a7, 64
    ecall
    li a0, 0
exit:
    li a7, 93
    ecall
check:
    li t0, 16
1:
    lw t1, 0(a1)
    lw t2, 0(a2)
    bne t1, t2, fail
    addi a1, a1, 4
    addi a2, a2, 4
    addi t0, t0, -1
    bnez t0, 1b
    ret
fail:
    li a0, 1
    j exit
.section .rodata
passed:
    .ascii "viota: pass\n"
.balign 4
unmasked_expected:
    .word 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15
masked_expected:
    .word 0, -1, 1, -1, 2, -1, 3, -1, 3, -1, 4, -1, 5, -1, -1, -1
.bss
.balign 8
result:
    .zero 64
