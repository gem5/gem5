.include "ai_ext_macros.inc"

.section .text
.globl _start

_start:
    li a0, 5
    li a1, -3
    li a2, 4
    AI_MAC 10, 11, 12
    li t0, -7
    bne a0, t0, fail

    li a1, 7
    li a2, 8
    AI_MAC 0, 11, 12
    addi t0, x0, 0
    bnez t0, fail

    li a0, 10
    li a1, 0x04030201
    li a2, 0xfe02ff01
    AI_DOT4_ACC 10, 11, 12
    li t0, 7
    bne a0, t0, fail

    li a1, -9
    AI_RELU 10, 11
    bnez a0, fail

    li a1, 12
    AI_RELU 10, 11
    li t0, 12
    bne a0, t0, fail

    AI_CLAMP 10, 0, 9
    bnez a0, fail

    li a1, 20
    AI_CLAMP 10, 11, 9
    li t0, 9
    bne a0, t0, fail

    li a1, 7
    AI_CLAMP 10, 11, 9
    li t0, 7
    bne a0, t0, fail

pass:
    li a0, 1
    la a1, pass_msg
    la a2, pass_msg_end
    sub a2, a2, a1
    li a7, 64
    ecall

    li a0, 0
    li a7, 93
    ecall

fail:
    li a0, 1
    la a1, fail_msg
    la a2, fail_msg_end
    sub a2, a2, a1
    li a7, 64
    ecall

    li a0, 1
    li a7, 93
    ecall

.section .rodata
pass_msg:
    .ascii "ai_ops_smoke: PASS\n"
pass_msg_end:

fail_msg:
    .ascii "ai_ops_smoke: FAIL\n"
fail_msg_end:
