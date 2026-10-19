# Copyright (c) 2026 Donmakino LLC
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Build the checked-in binary with:
# riscv64-unknown-elf-gcc -march=rv64gcv -mabi=lp64d -nostdlib -static \
#     -Wl,--build-id=none -Wl,--strip-all \
#     -o ../bin/riscv/linux/vfredosum-rounding vfredosum-rounding.S

.option norelax
.text
.global _start
_start:
    # Prime SoftFloat with RDN before changing the architectural mode.
    csrwi frm, 2
    fmv.w.x f0, zero
    fadd.s f0, f0, f0, dyn

    vsetivli zero, 1, e32, m1, tu, mu
    li t0, 0x33000000 # 2^-25, one quarter of an ulp at 1.0f
    vmv.v.x v8, t0
    li t0, 0x3f800000 # 1.0f
    vmv.v.x v16, t0

    # Each reduction must use the current frm, set NX, and preserve frm.
    .macro check_sum mode, expected
    csrwi frm, \mode
    csrwi fflags, 0
    vfredosum.vs v24, v8, v16
    vmv.x.s t1, v24
    li t0, \expected
    bne t1, t0, fail
    csrr t1, fflags
    li t0, 1
    bne t1, t0, fail
    csrr t1, frm
    li t0, \mode
    bne t1, t0, fail
    .endm

    check_sum 3, 0x3f800001 # RUP
    check_sum 2, 0x3f800000 # RDN, after the RUP reduction

    li a0, 0
    j exit
fail:
    li a0, 1
exit:
    li a7, 93
    ecall
