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

#include <stdbool.h>
#include <stdint.h>
#include <sys/syscall.h>

#include "syscall.h"

#ifdef errno
#undef errno
#endif

int errno;

static inline int64_t
set_errno_int(int64_t result)
{
    if (result >= -4096 && result < 0) {
        errno = -result;
        return -1;
    } else {
        return result;
    }
}

static inline void *
set_errno_ptr(void *value)
{ return (void *)set_errno_int((intptr_t)value); }

#define set_errno(result)                                                     \
    _Generic((result), void *: set_errno_ptr, default: set_errno_int)(result)

void
exit(int code)
{
    asm volatile("movl %0, %%eax\n"
                 "movl %1, %%edi\n"
                 "syscall\n"
                 "ud2\n" ::"i"(SYS_exit),
                 "r"(code));
    __builtin_unreachable();
}

ssize_t
write(int fd, const void *data, size_t size)
{
    ssize_t result;
    asm volatile("movl %1, %%eax\n"
                 "movl %2, %%edi\n"
                 "movq %3, %%rsi\n"
                 "movq %4, %%rdx\n"
                 "syscall\n"
                 : "=a"(result)
                 : "i"(SYS_write), "r"(fd), "r"(data), "r"(size));
    return set_errno(result);
}

ssize_t
read(int fd, void *data, size_t size)
{
    ssize_t result;
    asm volatile("movl %1, %%eax\n"
                 "movl %2, %%edi\n"
                 "movq %3, %%rsi\n"
                 "movq %4, %%rdx\n"
                 "syscall\n"
                 : "=a"(result)
                 : "i"(SYS_read), "r"(fd), "r"(data), "r"(size));
    return set_errno(result);
}

int
open(const char *path, int flags, ...)
{
    int result;
    asm volatile("movl %1, %%eax\n"
                 "movq %2, %%rdi\n"
                 "movl %3, %%esi\n"
                 "syscall\n"
                 : "=a"(result)
                 : "i"(SYS_open), "r"(path), "r"(flags));
    return set_errno(result);
}

void *
mmap(void *addr, size_t length, int prot, int flags, int fd, off_t offset)
{
    void *result;
    asm volatile("movl %1, %%eax\n"
                 "movq %2, %%rdi\n"
                 "movq %3, %%rsi\n"
                 "movl %4, %%edx\n"
                 "movl %5, %%r10d\n"
                 "movl %6, %%r8d\n"
                 "movq %7, %%r9\n"
                 "syscall\n"
                 : "=a"(result)
                 : "i"(SYS_mmap), "r"(addr), "r"(length), "r"(prot),
                   "r"(flags), "r"(fd), "r"(offset));
    return set_errno(result);
}

int
munmap(void *addr, size_t length)
{
    int result;
    asm volatile("movl %1, %%eax\n"
                 "movq %2, %%rdi\n"
                 "movq %3, %%rsi\n"
                 "syscall\n"
                 : "=a"(result)
                 : "i"(SYS_munmap), "r"(addr), "r"(length));
    return set_errno(result);
}

int
arch_prctl(int code, unsigned long addr)
{
    int result;
    asm volatile("movl %1, %%eax\n"
                 "movl %2, %%edi\n"
                 "movq %3, %%rsi\n"
                 : "=a"(result)
                 : "i"(SYS_arch_prctl), "r"(code), "r"(addr));
    return set_errno(result);
}
