/*
 * Copyright 2014 Google, Inc.
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
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
 *
 * Authors: Gabe Black
 */

#ifndef __ARCH_X86_REMOTEGDB_HH__
#define __ARCH_X86_REMOTEGDB_HH__

#include <algorithm>

#include "arch/x86/types.hh"
#include "base/remote_gdb.hh"

class System;
class ThreadContext;

namespace X86ISA
{
class RemoteGDB : public BaseRemoteGDB
{
  public:
    enum
    {
        GDB32_EAX,
        GDB32_ECX,
        GDB32_EDX,
        GDB32_EBX,
        GDB32_ESP,
        GDB32_EBP,
        GDB32_ESI,
        GDB32_EDI,
        GDB32_EIP,
        GDB32_EFLAGS,
        GDB32_CS,
        GDB32_SS,
        GDB32_DS,
        GDB32_ES,
        GDB32_FS,
        GDB32_GS,

        GDB32_NUMREGS
    };

    enum
    {
        GDB64_RAX,
        GDB64_RBX,
        GDB64_RCX,
        GDB64_RDX,
        GDB64_RSI,
        GDB64_RDI,
        GDB64_RBP,
        GDB64_RSP,
        GDB64_R8,
        GDB64_R9,
        GDB64_R10,
        GDB64_R11,
        GDB64_R12,
        GDB64_R13,
        GDB64_R14,
        GDB64_R15,
        GDB64_RIP,
        // These indices index into the reg cache treated as an array of 32
        // bit integers. The next index is one beyond the previous, and then
        // scaled up from an index into an array of 64 bit integers.
        GDB64_RFLAGS_32 = (GDB64_RIP + 1) * 2,
        GDB64_CS_32,
        GDB64_SS_32,
        GDB64_DS_32,
        GDB64_ES_32,
        GDB64_FS_32,
        GDB64_GS_32,

        // Scale the end index count back down (rounded up) to be for an
        // array of 64 bit integers.
        GDB64_NUMREGS = (GDB64_GS_32 + 1) / 2 + 1
    };

    RemoteGDB(System *system, ThreadContext *context);

    bool acc(Addr addr, size_t len);

  protected:
    void getregs();
    void setregs();

    bool checkBpLen(size_t len) { return len == 1; }
};

const int GDB_REG_BYTES M5_VAR_USED =
    std::max(RemoteGDB::GDB32_NUMREGS * sizeof(uint32_t),
             RemoteGDB::GDB64_NUMREGS * sizeof(uint64_t));

}

#endif // __ARCH_X86_REMOTEGDB_HH__
