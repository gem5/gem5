/*
 * Copyright (c) 2026 Jaeuk Lee
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

    .section .text.init
    .globl _start
_start:
    li      t0, 4096
1:
    addi    t0, t0, -1
    bnez    t0, 1b

    csrw    mcycle, zero
    csrw    minstret, zero

    nop
    nop
    nop
    nop

    csrr    a0, mcycle
    csrr    a1, minstret

    li      a2, 1000
    bgeu    a0, a2, fail
    bgeu    a1, a2, fail

pass:
    li      a0, 0x20
    la      a1, pass_args
    j       semihost_exit

fail:
    li      a0, 0x20
    la      a1, fail_args

semihost_exit:
    .option push
    .option norvc
    slli    zero, zero, 0x1f
    ebreak
    srai    zero, zero, 7
    .option pop

2:
    j       2b

    .section .rodata
    .align 3
pass_args:
    .dword  0x20026
    .dword  0
fail_args:
    .dword  0x20026
    .dword  1
