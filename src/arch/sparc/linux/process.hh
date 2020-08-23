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
 */

#ifndef __SPARC_LINUX_PROCESS_HH__
#define __SPARC_LINUX_PROCESS_HH__

#include "arch/sparc/linux/linux.hh"
#include "arch/sparc/process.hh"
#include "sim/process.hh"
#include "sim/syscall_desc.hh"

namespace SparcISA {

// This contains all of the common elements of a SPARC Linux process which
// are not shared by other operating systems. The rest come from the common
// SPARC process class.
class SparcLinuxProcess
{
  public:
    /// 64 bit syscall descriptors, indexed by call number.
    static SyscallDescTable<Sparc64Process::SyscallABI> syscallDescs;

    /// 32 bit compatibility syscall descriptors, indexed by call number.
    static SyscallDescTable<Sparc32Process::SyscallABI> syscall32Descs;
};

/// A process with emulated SPARC/Linux syscalls.
class Sparc32LinuxProcess : public SparcLinuxProcess, public Sparc32Process
{
  public:
    /// Constructor.
    Sparc32LinuxProcess(ProcessParams * params, ::Loader::ObjectFile *objFile);

    void syscall(ThreadContext *tc) override;

    void handleTrap(int trapNum, ThreadContext *tc) override;
};

/// A process with emulated 32 bit SPARC/Linux syscalls.
class Sparc64LinuxProcess : public SparcLinuxProcess, public Sparc64Process
{
  public:
    /// Constructor.
    Sparc64LinuxProcess(ProcessParams * params, ::Loader::ObjectFile *objFile);

    void syscall(ThreadContext *tc) override;

    void getContext(ThreadContext *tc);
    void setContext(ThreadContext *tc);

    void handleTrap(int trapNum, ThreadContext *tc) override;
};

SyscallReturn getresuidFunc(SyscallDesc *desc, ThreadContext *tc,
                            Addr ruid, Addr euid, Addr suid);

} // namespace SparcISA
#endif // __SPARC_LINUX_PROCESS_HH__
