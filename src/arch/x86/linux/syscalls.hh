/*
 * Copyright 2020 Google Inc.
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

#ifndef __ARCH_X86_LINUX_SYSCALLS_HH__
#define __ARCH_X86_LINUX_SYSCALLS_HH__

#include "arch/x86/linux/linux.hh"
#include "base/bitunion.hh"
#include "sim/se_workload.hh"
#include "sim/syscall_emul.hh"

namespace gem5
{

namespace X86ISA
{

SyscallReturn unameFunc(SyscallDesc *desc, ThreadContext *tc,
                        VPtr<Linux::utsname> name);

SyscallReturn archPrctlFunc(SyscallDesc *desc, ThreadContext *tc, int code,
                            uint64_t addr);

BitUnion32(UserDescFlags)
    Bitfield<0> seg_32bit;
    Bitfield<2, 1> contents;
    Bitfield<3> read_exec_only;
    Bitfield<4> limit_in_pages;
    Bitfield<5> seg_not_present;
    Bitfield<6> useable;
EndBitUnion(UserDescFlags)

struct UserDesc32
{
    uint32_t entry_number;
    uint32_t base_addr;
    uint32_t limit;
    uint32_t flags;
};

SyscallReturn setThreadArea32Func(SyscallDesc *desc, ThreadContext *tc,
                                  VPtr<UserDesc32> userDesc);

} // namespace X86ISA
} // namespace gem5

#endif // __ARCH_X86_LINUX_SYSCALLS_HH__
