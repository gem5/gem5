/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#ifndef __ALPHA_ACCESS_H__
#define __ALPHA_ACCESS_H__

/* @file
 * System Console Memory Mapped Register Definition
 */

#define ALPHA_ACCESS_VERSION (1291+1) /* CH++*/

#ifdef CONSOLE
typedef uint32 UINT32;
typedef uint64 UINT64;
#else
typedef uint32_t UINT32;
typedef uint64_t UINT64;

#include <ostream>
#include <string>
class IniFile;

#endif

// This structure hacked up from simos
struct AlphaAccess
{
    UINT32	last_offset;		// 00: must be first field
    UINT32	version;		// 04:
    UINT32	numCPUs;		// 08:
    UINT32	align0;			// 0C: Placeholder for alignment
    UINT64	mem_size;		// 10:
    UINT64	cpuClock;		// 18: MHz
    UINT32	intrClockFrequency;	// 20: Hz
    UINT32	align1;			// 24: Placeholder for alignment

    // Loaded kernel
    UINT64	kernStart;		// 28:
    UINT64	kernEnd;		// 30:
    UINT64	entryPoint;		// 38:

    // console disk stuff
    UINT64	diskUnit;		// 40:
    UINT64	diskCount;		// 48:
    UINT64	diskPAddr;		// 50:
    UINT64	diskBlock;		// 58:
    UINT64	diskOperation;		// 60:

    // console simple output stuff
    UINT64	outputChar;		// 68:

    // MP boot
    UINT64	bootStrapImpure;	// 70:
    UINT32	bootStrapCPU;		// 78:
    UINT32	align2;			// 7C: Dummy placeholder for alignment

#ifndef CONSOLE
    void serialize(std::ostream &os);
    void unserialize(const IniFile *db, const std::string &section);
#endif
};

#endif // __ALPHA_ACCESS_H__
