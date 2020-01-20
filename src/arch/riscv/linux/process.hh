/*
 * Copyright (c) 2004 The Regents of The University of Michigan
 * Copyright (c) 2016 The University of Virginia
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

#ifndef __RISCV_LINUX_PROCESS_HH__
#define __RISCV_LINUX_PROCESS_HH__

#include <map>

#include "arch/riscv/linux/linux.hh"
#include "arch/riscv/process.hh"
#include "sim/eventq.hh"
#include "sim/syscall_desc.hh"

/// A process with emulated Riscv/Linux syscalls.
class RiscvLinuxProcess64 : public RiscvProcess64
{
  public:
    /// Constructor.
    RiscvLinuxProcess64(ProcessParams * params, ::Loader::ObjectFile *objFile);

    /// The target system's hostname.
    static const char *hostname;

    /// ID of the thread group leader for the process
    uint64_t __tgid;

    void syscall(ThreadContext *tc, Fault *fault) override;

    /// Syscall descriptors, indexed by call number.
    static SyscallDescTable<SyscallABI> syscallDescs;
};

class RiscvLinuxProcess32 : public RiscvProcess32
{
  public:
    /// Constructor.
    RiscvLinuxProcess32(ProcessParams * params, ::Loader::ObjectFile *objFile);

    /// The target system's hostname.
    static const char *hostname;

    /// ID of the thread group leader for the process
    uint64_t __tgid;

    void syscall(ThreadContext *tc, Fault *fault) override;

    /// Array of syscall descriptors, indexed by call number.
    static SyscallDescTable<SyscallABI> syscallDescs;
};

#endif // __RISCV_LINUX_PROCESS_HH__
