/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * (University of Wisconsin-Madison)
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

int
_read_r(struct _reent *r, int file, char *ptr, int len)
{
    r = r;
    file = file;
    ptr = ptr;
    len = len;

    errno = EINVAL;
    return -1;
}

int
_lseek_r(struct _reent *r, int file, int ptr, int dir)
{
    r = r;
    file = file;
    ptr = ptr;
    dir = dir;

    return 0;
}

// The following two functions implement the basis for printf and the UART
// component in gem5.
volatile unsigned int *const UART0DR = (unsigned int *)0x1c090000;

void
gem5_print(const char *s)
{
    while (*s != '\0') {
        *UART0DR = (unsigned int)(*s);
        s++;
    }
}

int
_write_r(struct _reent *r, int file, char *ptr, int len)
{
    r = r;
    file = file;
    ptr = ptr;

    gem5_print(ptr);

    return len;
}

int
_close_r(struct _reent *r, int file)
{
    return 0;
}

register char *stack_ptr __asm("sp");

caddr_t
_sbrk_r(struct _reent *r, int incr)
{
    extern char end __asm("end");
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == NULL) {
        heap_end = &end;
    }

    prev_heap_end = heap_end;

    if (heap_end + incr > stack_ptr) {
        errno = ENOMEM;
        return (caddr_t)-1;
    }

    heap_end += incr;
    return (caddr_t)prev_heap_end;
}

int
_fstat_r(struct _reent *r, int file, struct stat *st)
{
    r = r;
    file = file;
    memset(st, 0, sizeof(*st));
    st->st_mode = S_IFCHR;
    return 0;
}

int
_isatty_r(struct _reent *r, int fd)
{
    r = r;
    fd = fd;
    return 1;
}

int
_getpid(int n)
{
    return 1;
    n = n;
}

int
_kill(int pid, int sig)
{
    asm("swi %a0" ::"i"(11));
    return -1; // Never gets here
}

void
_exit(int sig)
{
    _kill(sig, -1);
    while (1)
        ; // Infinite loop to halt execution
    __builtin_unreachable();
}
