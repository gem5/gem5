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

#ifndef __UTIL_PINCPU_OPS_H__
#define __UTIL_PINCPU_OPS_H__

#ifdef __cplusplus
#include <cstdint>

#else
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#endif

enum PinOp
{
    OP_GET_REQPATH,
    OP_GET_RESPPATH,
    OP_GET_MEMPATH,
    OP_ABORT,
    OP_EXIT,
    OP_RUN,
    OP_RESETUSER,
    OP_SET_VSYSCALL_BASE,
    OP_GET_INSTCOUNT,
    OP_SET_REGS,
    OP_GET_REGS,
    OP_ADD_SYMBOL,
    OP_EXEC_COMMAND,
    OP_READ_COMMAND_RESULT,
    OP_COUNT,
};

#define pinops_addr_base ((uint64_t)0xbaddecaf << 12)
#define pinops_addr_end (pinops_addr_base + OP_COUNT)

static inline bool
is_pinop_addr(void *p)
{
    const uintptr_t s = (uintptr_t)p;
    return pinops_addr_base <= s && s < pinops_addr_end;
}

// TODO: Need utility functions for setting this from pintool.
struct RunResult
{
    enum RunResultType
    {
        RUNRESULT_PAGEFAULT,
        RUNRESULT_SYSCALL,
        RUNRESULT_CPUID,
        RUNRESULT_BREAK,
        RUNRESULT_INSTCOUNT,
    } result;
    union
    {
        uint64_t addr; // RUNRESULT_PAGEFAULT
    };
};

// TODO: Only declare these in guest, not pintool.
void pinop_get_reqpath(char *data, size_t size);
void pinop_get_resppath(char *data, size_t size);
void pinop_get_mempath(char *data, size_t size);
void pinop_exit(int code);
void pinop_abort(const char *msg, size_t line);
void pinop_resetuser(void);
void pinop_run(struct RunResult *result);
void pinop_set_vsyscall_base(void *virt, void *phys);
uint64_t pinop_get_instcount(void);

struct PinRegFile;
void pinop_set_regs(const struct PinRegFile *regfile);
void pinop_get_regs(struct PinRegFile *regfile);

void pinop_add_symbol(const char *name, void *vaddr);

/// Returns the number of bytes in the command result.
size_t pinop_exec_command(const char *s);
void pinop_read_command_result(char *buf, size_t idx, size_t size);

#define pinop_abort() (pinop_abort)(__FILE__, __LINE__)

#endif // __UTIL_PINCPU_OPS_H__
