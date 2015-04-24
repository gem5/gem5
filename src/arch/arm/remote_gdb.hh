/*
 * Copyright 2014 Google, Inc.
 * Copyright (c) 2013 ARM Limited
 * All rights reserved
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
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * Copyright (c) 2007-2008 The Florida State University
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
 *          Stephen Hines
 */

#ifndef __ARCH_ARM_REMOTE_GDB_HH__
#define __ARCH_ARM_REMOTE_GDB_HH__

#include <algorithm>

#include "base/remote_gdb.hh"

class System;
class ThreadContext;

namespace ArmISA
{

// AArch32 registers with vfpv3/neon
enum {
    GDB32_R0 = 0,
    GDB32_CPSR = 16,
    GDB32_F0 = 17,
    GDB32_FPSCR = 81,
    GDB32_NUMREGS = 82
};

// AArch64 registers
enum {
    GDB64_X0 = 0,
    GDB64_SPX = 31,
    GDB64_PC = 32,
    GDB64_CPSR = 33,
    GDB64_V0 = 34,
    GDB64_V0_32 = 2 * GDB64_V0,
    GDB64_NUMREGS = 98
};

const int GDB_REG_BYTES M5_VAR_USED =
    std::max(GDB64_NUMREGS * sizeof(uint64_t),
             GDB32_NUMREGS * sizeof(uint32_t));

class RemoteGDB : public BaseRemoteGDB
{
  protected:
    bool acc(Addr addr, size_t len);
    bool write(Addr addr, size_t size, const char *data);

    void getregs();
    void setregs();

  public:
    RemoteGDB(System *_system, ThreadContext *tc);
};
} // namespace ArmISA

#endif /* __ARCH_ARM_REMOTE_GDB_H__ */
