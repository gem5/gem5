/*
 * Copyright (c) 2003-2004 The Regents of The University of Michigan
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
 * Authors: Steve Reinhardt
 */

#ifndef __SPARC_LINUX_PROCESS_HH__
#define __SPARC_LINUX_PROCESS_HH__

#include "arch/sparc/linux/linux.hh"
#include "arch/sparc/process.hh"
#include "sim/process.hh"

namespace SparcISA {

// This contains all of the common elements of a SPARC Linux process which
// are not shared by other operating systems. The rest come from the common
// SPARC process class.
class SparcLinuxProcess
{
  public:
     /// Array of syscall descriptors, indexed by call number.
    static SyscallDesc syscallDescs[];

     /// Array of 32 bit compatibility syscall descriptors,
     /// indexed by call number.
    static SyscallDesc syscall32Descs[];

    SyscallDesc* getDesc(int callnum);
    SyscallDesc* getDesc32(int callnum);

    static const int Num_Syscall_Descs;
    static const int Num_Syscall32_Descs;
};

/// A process with emulated SPARC/Linux syscalls.
class Sparc32LinuxProcess : public SparcLinuxProcess, public Sparc32LiveProcess
{
  public:
    /// Constructor.
    Sparc32LinuxProcess(LiveProcessParams * params, ObjectFile *objFile);

    SyscallDesc*
    getDesc(int callnum)
    {
        return SparcLinuxProcess::getDesc32(callnum);
    }

    void handleTrap(int trapNum, ThreadContext *tc);
};

/// A process with emulated 32 bit SPARC/Linux syscalls.
class Sparc64LinuxProcess : public SparcLinuxProcess, public Sparc64LiveProcess
{
  public:
    /// Constructor.
    Sparc64LinuxProcess(LiveProcessParams * params, ObjectFile *objFile);

    SyscallDesc*
    getDesc(int callnum)
    {
        return SparcLinuxProcess::getDesc(callnum);
    }

    void handleTrap(int trapNum, ThreadContext *tc);
};

SyscallReturn getresuidFunc(SyscallDesc *desc, int num,
                                 LiveProcess *p, ThreadContext *tc);

} // namespace SparcISA
#endif // __SPARC_LINUX_PROCESS_HH__
