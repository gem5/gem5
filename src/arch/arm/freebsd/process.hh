/*
 * Copyright (c) 2015 Ruslan Bukin <br@bsdpad.com>
 * All rights reserved.
 *
 * This software was developed by the University of Cambridge Computer
 * Laboratory as part of the CTSRD Project, with support from the UK Higher
 * Education Innovation Fund (HEIF).
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

#ifndef __ARCH_ARM_FREEBSD_PROCESS_HH__
#define __ARCH_ARM_FREEBSD_PROCESS_HH__

#include <vector>

#include "arch/arm/process.hh"

class ArmFreebsdProcessBits
{
  protected:
    SyscallDesc* getFreebsdDesc(int callnum);

    struct SyscallTable
    {
        int base;
        SyscallDesc *descs;
        int size;

        SyscallDesc *getDesc(int offset) const;
    };

    std::vector<SyscallTable> syscallTables;
};

/// A process with emulated Arm/Freebsd syscalls.
class ArmFreebsdProcess32 : public ArmProcess32, public ArmFreebsdProcessBits
{
  public:
    ArmFreebsdProcess32(ProcessParams * params, ObjectFile *objFile,
                        ObjectFile::Arch _arch);

    void initState();

    /// Explicitly import the otherwise hidden getSyscallArg
    using ArmProcess::getSyscallArg;

    /// A page to hold "kernel" provided functions. The name might be wrong.
    static const Addr commPage;

    SyscallDesc* getDesc(int callnum);
};

/// A process with emulated Arm/Freebsd syscalls.
class ArmFreebsdProcess64 : public ArmProcess64, public ArmFreebsdProcessBits
{
  public:
    ArmFreebsdProcess64(ProcessParams * params, ObjectFile *objFile,
                        ObjectFile::Arch _arch);

    void initState();
    SyscallDesc* getDesc(int callnum);
};

#endif // __ARCH_ARM_FREEBSD_PROCESS_HH__
