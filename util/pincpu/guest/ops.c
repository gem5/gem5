/*
 * Copyright (c) 2026 The Board of Trustees of the Leland Stanford
 * Junior University
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

#include "ops.h"

// TODO: macro for defining these, since they are all basically the same.

void __attribute__((naked))
pinop_get_reqpath(char *data, size_t size)
{
    asm volatile(
        "movb $0, (%0)\nret\n" ::"r"(pinops_addr_base + OP_GET_REQPATH));
}

void __attribute__((naked))
pinop_get_resppath(char *data, size_t size)
{
    asm volatile(
        "movb $0, (%0)\nret\n" ::"r"(pinops_addr_base + OP_GET_RESPPATH));
}

void __attribute__((naked))
pinop_get_mempath(char *data, size_t size)
{
    asm volatile(
        "movb $0, (%0)\nret\n" ::"r"(pinops_addr_base + OP_GET_MEMPATH));
}

void __attribute__((naked))
pinop_exit(int code)
{ asm volatile("movb $0, (%0)\nret\n" ::"r"(pinops_addr_base + OP_EXIT)); }

void __attribute__((naked)) (pinop_abort)(const char *msg, size_t line)
{
    asm volatile("movq (%%rsp), %%rdx\n"
                 "movb $0, (%0)\nret\n" ::"r"(pinops_addr_base + OP_ABORT));
}

void __attribute__((naked))
pinop_resetuser()
{
    asm volatile(
        "movb $0, (%0)\nret\n" ::"r"(pinops_addr_base + OP_RESETUSER));
}

void __attribute__((naked))
pinop_run(struct RunResult *result)
{ asm volatile("movb $0, (%0)\nret\n" ::"r"(pinops_addr_base + OP_RUN)); }

void __attribute__((naked))
pinop_set_vsyscall_base(void *virt, void *phys)
{
    asm volatile(
        "movb $0, (%0)\nret\n" ::"r"(pinops_addr_base + OP_SET_VSYSCALL_BASE));
}

uint64_t __attribute__((naked))
pinop_get_instcount(void)
{
    asm volatile(
        "movb $0, (%0)\nret\n" ::"r"(pinops_addr_base + OP_GET_INSTCOUNT));
}

void __attribute__((naked))
pinop_set_regs(const struct PinRegFile *regfile)
{ asm volatile("movb $0, (%0)\nret\n" ::"r"(pinops_addr_base + OP_SET_REGS)); }

void __attribute__((naked))
pinop_get_regs(struct PinRegFile *regfile)
{ asm volatile("movb $0, (%0)\nret\n" ::"r"(pinops_addr_base + OP_GET_REGS)); }

void __attribute__((naked))
pinop_add_symbol(const char *name, void *vaddr)
{
    asm volatile(
        "movb $0, (%0)\nret\n" ::"r"(pinops_addr_base + OP_ADD_SYMBOL));
}

size_t __attribute__((naked))
pinop_exec_command(const char *cmd)
{
    asm volatile(
        "movb $0, (%0)\nret\n" ::"r"(pinops_addr_base + OP_EXEC_COMMAND));
}

void __attribute__((naked))
pinop_read_command_result(char *buf, size_t idx, size_t size)
{
    asm volatile("movb $0, (%0)\nret\n" ::"r"(pinops_addr_base +
                                              OP_READ_COMMAND_RESULT));
}
