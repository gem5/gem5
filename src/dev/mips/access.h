/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 *
 * Authors: Nathan Binkert
 */

#ifndef __MIPS_ACCESS_H__
#define __MIPS_ACCESS_H__

/** @file
 * System Console Memory Mapped Register Definition
 */

#define MIPS_ACCESS_VERSION (1305)
#define CONSOLE_START_ADDRESS 0xBFD00F00
#define REG_OFFSET 1
#define UART8250_BASE 0xBFD003F8
#define UART8250_END 7*REG_OFFSET
#ifdef CONSOLE
typedef unsigned uint32_t;
typedef unsigned long uint64_t;
#endif

// This structure hacked up from simos
struct MipsAccess
{
    uint32_t    inputChar;              // 00: Placeholder for input
    uint32_t    last_offset;            // 04: must be first field
    uint32_t    version;                // 08:
    uint32_t    numCPUs;                // 0C:
    uint32_t    intrClockFrequency;     // 10: Hz

    // Loaded kernel
    uint32_t    kernStart;              // 14:
    uint32_t    kernEnd;                // 18:
    uint32_t    entryPoint;             // 1c:

                // console simple output stuff
                uint32_t        outputChar;             // 20: Placeholder for output

    // console disk stuff
    uint32_t    diskUnit;               // 24:
    uint32_t    diskCount;              // 28:
    uint32_t    diskPAddr;              // 2c:
    uint32_t    diskBlock;              // 30:
    uint32_t    diskOperation;          // 34:

                // MP boot
    uint32_t    cpuStack[64];           // 70:

    /* XXX There appears to be a problem in accessing
     * unit64_t in the console.c file. They are treated
     * like uint32_int and result in the wrong address for
     * everything below. This problem should be investigated.
     */
    uint64_t    cpuClock;               // 38: MHz
    uint64_t    mem_size;               // 40:
};

#endif // __MIPS_ACCESS_H__
