.include "ai_ext_macros.inc"

.section .text
.globl _start

_start:
    la s0, data_words

    AI_P_LW 10, 8, 4
    li t0, 0x11223344
    bne a0, t0, fail
    la t1, data_words
    addi t1, t1, 4
    bne s0, t1, fail

    AI_P_LW 11, 8, 4
    li t0, 0x55667788
    bne a1, t0, fail
    la t1, data_words
    addi t1, t1, 8
    bne s0, t1, fail

    AI_P_LW 0, 8, 4
    addi t0, x0, 0
    bnez t0, fail
    la t1, data_words
    addi t1, t1, 12
    bne s0, t1, fail

    AI_P_LW 12, 8, -4
    li t0, 0x12345678
    bne a2, t0, fail
    la t1, data_words
    addi t1, t1, 8
    bne s0, t1, fail

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
    .ascii "p_lw_smoke: PASS\n"
pass_msg_end:

fail_msg:
    .ascii "p_lw_smoke: FAIL\n"
fail_msg_end:

.align 2
data_words:
    .word 0x11223344
    .word 0x55667788
    .word 0x99aabbcc
    .word 0x12345678
