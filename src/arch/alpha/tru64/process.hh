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

#ifndef __ARCH_ALPHA_TRU64_PROCESS_HH__
#define __ARCH_ALPHA_TRU64_PROCESS_HH__

#include "arch/alpha/process.hh"

namespace AlphaISA {

/// A process with emulated Alpha Tru64 syscalls.
class AlphaTru64Process : public AlphaLiveProcess
{
  public:
    /// Constructor.
    AlphaTru64Process(LiveProcessParams * params,
                      ObjectFile *objFile);

    /// Array of syscall descriptors, indexed by call number.
    static SyscallDesc syscallDescs[];

    /// Array of mach syscall descriptors, indexed by call number.
    static SyscallDesc machSyscallDescs[];

    const int Num_Syscall_Descs;
    const int Num_Mach_Syscall_Descs;

    virtual SyscallDesc *getDesc(int callnum);
};

} // namespace AlphaISA

#endif // __ARCH_ALPHA_TRU64_PROCESS_HH__
