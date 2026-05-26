.include "ai_ext_macros.inc"

.section .text
.globl _start

_start:
    li s0, 0
    li s1, 0
    li t0, 3
    AI_LP_SETUP 5, 3
    addi s0, s0, 2
    addi s1, s1, 1

    li t1, 6
    bne s0, t1, fail
    li t1, 3
    bne s1, t1, fail

    li s2, 0
    li s3, 0
    AI_LP_SETUP 0, 3
    addi s2, s2, 5
    addi s3, s3, 1

    li t1, 5
    bne s2, t1, fail
    li t1, 1
    bne s3, t1, fail

    li s4, 0
    li t0, 2
    AI_LP_SETUP 5, 2
    addi s4, s4, 1

    li t1, 2
    bne s4, t1, fail

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
    .ascii "hwloop_smoke: PASS\n"
pass_msg_end:

fail_msg:
    .ascii "hwloop_smoke: FAIL\n"
fail_msg_end:
