/*
 * Copyright (c) 2012-2013, 2015 ARM Limited
 * Copyright (c) 2015-2016 Advanced Micro Devices, Inc.
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
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

#ifndef __SIM_SYSCALL_DESC_HH__
#define __SIM_SYSCALL_DESC_HH__

#include <functional>
#include <string>

#include "base/types.hh"
#include "cpu/thread_context.hh"
#include "sim/guest_abi.hh"
#include "sim/process.hh"
#include "sim/syscall_return.hh"

class SyscallDesc;

SyscallReturn unimplementedFunc(SyscallDesc *desc, int num,
                                ThreadContext *tc);

/**
 * This class provides the wrapper interface for the system call
 * implementations which are defined in the sim/syscall_emul files and
 * bound to the ISAs in the architecture specific code
 * (i.e. arch/X86/linux/process.cc).
 */
class SyscallDesc {
  public:
    using SyscallExecutor =
        std::function<SyscallReturn(SyscallDesc *, int num, ThreadContext *)>;

    SyscallDesc(const char *name, SyscallExecutor sys_exec=unimplementedFunc)
        : _name(name), executor(sys_exec)
    {}

    /**
     * Interface for invoking the system call funcion pointer. Note that
     * this acts as a gateway for all system calls and serves a good point
     * to add filters for behaviors or apply checks for all system calls.
     * @param callnum Number associated with call (by operating system)
     * @param proc Handle for the owning Process to pass information
     * @param tc Handle for owning ThreadContext to pass information
     */
    void doSyscall(int callnum, ThreadContext *tc, Fault *fault);

    std::string name() { return _name; }

  private:
    /** System call name (e.g., open, mmap, clone, socket, etc.) */
    std::string _name;

    /** Mechanism for ISAs to connect to the emul function definitions */
    SyscallExecutor executor;
};

/*
 * This SyscallDesc subclass template adapts a given syscall implementation so
 * that some arguments can come from the simulator (desc, num and tc) while the
 * rest can come from the guest using the GuestABI mechanism.
 */
template <typename ABI>
class SyscallDescABI : public SyscallDesc
{
  private:
    // Aliases to make the code below a little more concise.
    template <typename ...Args>
    using SyscallABIExecutor =
        std::function<SyscallReturn(SyscallDesc *, int,
                                    ThreadContext *, Args...)>;

    template <typename ...Args>
    using SyscallABIExecutorPtr =
        SyscallReturn (*)(SyscallDesc *, int, ThreadContext *, Args...);


    // Wrap an executor with guest arguments with a normal executor that gets
    // those additional arguments from the guest context.
    template <typename ...Args>
    static inline SyscallExecutor
    buildExecutor(SyscallABIExecutor<Args...> target)
    {
        return [target](SyscallDesc *desc, int num,
                        ThreadContext *tc) -> SyscallReturn {
            // Create a partial function which will stick desc and num to the
            // front of the parameter list.
            auto partial = [target,desc,num](
                    ThreadContext *tc, Args... args) -> SyscallReturn {
                return target(desc, num, tc, args...);
            };

            // Use invokeSimcall to gather the other arguments based on the
            // given ABI and pass them to the syscall implementation.
            return invokeSimcall<ABI, SyscallReturn, Args...>(tc,
                    std::function<SyscallReturn(ThreadContext *, Args...)>(
                        partial));
        };
    }


  public:
    // Constructors which plumb in buildExecutor.
    template <typename ...Args>
    SyscallDescABI(const char *name, SyscallABIExecutor<Args...> target) :
        SyscallDesc(name, buildExecutor<Args...>(target))
    {}

    template <typename ...Args>
    SyscallDescABI(const char *name, SyscallABIExecutorPtr<Args...> target) :
        SyscallDescABI(name, SyscallABIExecutor<Args...>(target))
    {}

    using SyscallDesc::SyscallDesc;
};

struct DefaultSyscallABI
{
    using Position = int;
};

namespace GuestABI
{

template <>
struct Result<DefaultSyscallABI, SyscallReturn>
{
    static void
    store(ThreadContext *tc, const SyscallReturn &ret)
    {
        auto *process = tc->getProcessPtr();
        process->setSyscallReturn(tc, ret);
    }
};

template <typename Arg>
struct Argument<DefaultSyscallABI, Arg,
    typename std::enable_if<std::is_integral<Arg>::value>::type>
{
    static Arg
    get(ThreadContext *tc, DefaultSyscallABI::Position &position)
    {
        auto *process = tc->getProcessPtr();
        return process->getSyscallArg(tc, position);
    }
};

template <typename Arg>
struct Argument<DefaultSyscallABI, Arg,
    typename std::enable_if<std::is_pointer<Arg>::value>::type>
{
    static Arg
    get(ThreadContext *tc, DefaultSyscallABI::Position &position)
    {
        auto *process = tc->getProcessPtr();
        RegVal reg = process->getSyscallArg(tc, position);
        return (Arg)(uintptr_t)(reg);
    }
};

} // namespace GuestABI

#endif // __SIM_SYSCALL_DESC_HH__
