/*
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
 */

#ifndef __X86_LINUX_PROCESS_HH__
#define __X86_LINUX_PROCESS_HH__

#include "arch/x86/linux/linux.hh"
#include "arch/x86/process.hh"
#include "sim/process.hh"
#include "sim/syscall_abi.hh"

struct ProcessParams;
struct ThreadContext;

namespace X86ISA
{

class X86_64LinuxProcess : public X86_64Process
{
  public:
    using X86_64Process::X86_64Process;
    void syscall(ThreadContext *tc) override;
    void clone(ThreadContext *old_tc, ThreadContext *new_tc, Process *process,
               RegVal flags) override;

    struct SyscallABI : public GenericSyscallABI64, public X86Linux::SyscallABI
    {
        static const std::vector<IntRegIndex> ArgumentRegs;
    };
};

class I386LinuxProcess : public I386Process
{
  public:
    using I386Process::I386Process;
    void syscall(ThreadContext *tc) override;
    void clone(ThreadContext *old_tc, ThreadContext *new_tc, Process *process,
               RegVal flags) override;

    struct SyscallABI : public GenericSyscallABI32, public X86Linux::SyscallABI
    {
        static const std::vector<IntRegIndex> ArgumentRegs;
    };
};

} // namespace X86ISA

namespace GuestABI
{

template <typename Arg>
struct Argument<X86ISA::I386LinuxProcess::SyscallABI, Arg,
    typename std::enable_if<
        X86ISA::I386LinuxProcess::SyscallABI::IsWide<Arg>::value>::type>
{
    using ABI = X86ISA::I386LinuxProcess::SyscallABI;

    static Arg
    get(ThreadContext *tc, typename ABI::State &state)
    {
        panic_if(state + 1 >= ABI::ArgumentRegs.size(),
                "Ran out of syscall argument registers.");
        auto low = ABI::ArgumentRegs[state++];
        auto high = ABI::ArgumentRegs[state++];
        return (Arg)ABI::mergeRegs(tc, low, high);
    }
};

} // namespace GuestABI

#endif // __X86_LINUX_PROCESS_HH__
