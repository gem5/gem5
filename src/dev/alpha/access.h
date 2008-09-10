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

#ifndef __ALPHA_ACCESS_H__
#define __ALPHA_ACCESS_H__

/** @file
 * System Console Memory Mapped Register Definition
 */

#define ALPHA_ACCESS_VERSION (1305)

#ifdef CONSOLE
typedef unsigned uint32_t;
typedef unsigned long uint64_t;
#endif

// This structure hacked up from simos
struct AlphaAccess
{
    uint32_t    last_offset;            // 00: must be first field
    uint32_t    version;                // 04:
    uint32_t    numCPUs;                // 08:
    uint32_t    intrClockFrequency;     // 0C: Hz
    uint64_t    cpuClock;               // 10: MHz
    uint64_t    mem_size;               // 18:

    // Loaded kernel
    uint64_t    kernStart;              // 20:
    uint64_t    kernEnd;                // 28:
    uint64_t    entryPoint;             // 30:

    // console disk stuff
    uint64_t    diskUnit;               // 38:
    uint64_t    diskCount;              // 40:
    uint64_t    diskPAddr;              // 48:
    uint64_t    diskBlock;              // 50:
    uint64_t    diskOperation;          // 58:

    // console simple output stuff
    uint64_t    outputChar;             // 60: Placeholder for output
    uint64_t    inputChar;              // 68: Placeholder for input

    // MP boot
    uint64_t    cpuStack[64];           // 70:
};

#endif // __ALPHA_ACCESS_H__
