/*
 * Copyright (c) 2026 Nathanon Chirattithiphan
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

#include <gem5/asm/generic/m5ops.h>

.global _start
.section .text
_start:
#if !defined(UNARMED)
    lea monitored_line(%rip), %rax
    xor %ecx, %ecx
    xor %edx, %edx
#endif

#if defined(ARMED)
    monitor
    xor %eax, %eax
    xor %ecx, %ecx
    mwait
#elif defined(WRONG_MONITOR)
    # A cold load delays a taken branch. MONITOR is on the predicted
    # fall-through path. The MWAIT at the target should see no monitor.
    cmpq $0, branch_condition(%rip)
    je right_path
    monitor
    .rept 32
    nop
    .endr
right_path:
    xor %eax, %eax
    xor %ecx, %ecx
    cpuid
    xor %eax, %eax
    xor %ecx, %ecx
    mwait
#elif defined(WRONG_MWAIT)
    # Arm the monitor and warm its cache line before the branch. This
    # lets a speculative MWAIT complete before the cold load resolves.
    monitor
    xor %eax, %eax
    cpuid
    mov monitored_line(%rip), %rax
    xor %eax, %eax
    cpuid
    cmpq $0, branch_condition(%rip)
    je right_path
    xor %eax, %eax
    xor %ecx, %ecx
    mwait
    .rept 32
    nop
    .endr
right_path:
#elif defined(UNARMED)
    xor %eax, %eax
    xor %ecx, %ecx
    mwait
#else
#error Unknown MONITOR/MWAIT test case
#endif

    # m5_exit(0).
    xor %edi, %edi
    .byte 0x0f, 0x04
    .word M5OP_EXIT
1:  jmp 1b

#if defined(ARMED)
    # The configuration places .data at the writer's first target line.
    .section .data
    .balign 64
monitored_line:
    .zero 64
    .zero 64
#elif defined(WRONG_MONITOR) || defined(WRONG_MWAIT)
    .balign 64
monitored_line:
    .quad 0
    .balign 4096
branch_condition:
    .quad 0
#endif

.section .note.GNU-stack,"",@progbits
