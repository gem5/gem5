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
#include <map>
#include <string>

#include "base/logging.hh"
#include "base/types.hh"
#include "cpu/thread_context.hh"
#include "sim/guest_abi.hh"
#include "sim/process.hh"
#include "sim/syscall_return.hh"

namespace gem5
{

class SyscallDesc;

SyscallReturn unimplementedFunc(SyscallDesc *desc, ThreadContext *tc);

/**
 * This class provides the wrapper interface for the system call
 * implementations which are defined in the sim/syscall_emul files and
 * bound to the ISAs in the architecture specific code
 * (i.e. arch/X86/linux/process.cc).
 */
class SyscallDesc
{
  public:
    /**
     * Interface for invoking the system call funcion pointer. Note that
     * this acts as a gateway for all system calls and serves a good point
     * to add filters for behaviors or apply checks for all system calls.
     * @param tc Handle for owning ThreadContext to pass information
     */
    void doSyscall(ThreadContext *tc);

    std::string name() const { return _name; }
    int num() const { return _num; }

    /**
     * For use within the system call executor if new threads are created and
     * need something returned into them.
     */
    virtual void returnInto(ThreadContext *tc, const SyscallReturn &ret) = 0;

  protected:
    using Executor =
        std::function<SyscallReturn(SyscallDesc *, ThreadContext *)>;
    using Dumper = std::function<std::string(std::string, ThreadContext *)>;

    SyscallDesc(int num, const char *name, Executor exec, Dumper dump) :
        _name(name), _num(num), executor(exec), dumper(dump)
    {}

    void retrySyscall(ThreadContext *tc);

  private:
    /** System call name (e.g., open, mmap, clone, socket, etc.) */
    std::string _name;
    int _num;

    void setupRetry(ThreadContext *tc);
    void handleReturn(ThreadContext *tc, const SyscallReturn &ret);

    /** Mechanism for ISAs to connect to the emul function definitions */
    Executor executor;
    Dumper dumper;
};

/*
 * This SyscallDesc subclass template adapts a given syscall implementation so
 * that some arguments can come from the simulator (desc, num and tc) while the
 * rest can come from the guest using the guest_abi mechanism.
 */
template <typename ABI>
class SyscallDescABI : public SyscallDesc
{
  private:
    // Aliases to make the code below a little more concise.
    template <typename ...Args>
    using ABIExecutor =
        std::function<SyscallReturn(SyscallDesc *, ThreadContext *, Args...)>;

    template <typename ...Args>
    using ABIExecutorPtr =
        SyscallReturn (*)(SyscallDesc *, ThreadContext *, Args...);


    // Wrap an executor with guest arguments with a normal executor that gets
    // those additional arguments from the guest context.
    template <typename ...Args>
    static inline Executor
    buildExecutor(ABIExecutor<Args...> target)
    {
        return [target](SyscallDesc *desc,
                        ThreadContext *tc) -> SyscallReturn {
            // Create a partial function which will stick desc to the front of
            // the parameter list.
            auto partial = [target,desc](
                    ThreadContext *tc, Args... args) -> SyscallReturn {
                return target(desc, tc, args...);
            };

            // Use invokeSimcall to gather the other arguments based on the
            // given ABI and pass them to the syscall implementation.
            return invokeSimcall<ABI, false, SyscallReturn, Args...>(tc,
                    std::function<SyscallReturn(ThreadContext *, Args...)>(
                        partial));
        };
    }

    template <typename ...Args>
    static inline Dumper
    buildDumper()
    {
        return [](std::string name, ThreadContext *tc) -> std::string {
            return dumpSimcall<ABI, SyscallReturn, Args...>(name, tc);
        };
    }

  public:
    // Constructors which plumb in buildExecutor.
    template <typename ...Args>
    SyscallDescABI(int num, const char *name, ABIExecutor<Args...> target) :
        SyscallDesc(num, name, buildExecutor<Args...>(target),
                               buildDumper<Args...>())
    {}

    template <typename ...Args>
    SyscallDescABI(int num, const char *name, ABIExecutorPtr<Args...> target) :
        SyscallDescABI(num, name, ABIExecutor<Args...>(target))
    {}

    SyscallDescABI(int num, const char *name) :
        SyscallDescABI(num, name, ABIExecutor<>(unimplementedFunc))
    {}

    void
    returnInto(ThreadContext *tc, const SyscallReturn &ret) override
    {
        guest_abi::Result<ABI, SyscallReturn>::store(tc, ret);
    }
};

template <typename ABI>
class SyscallDescTable
{
  private:
    std::map<int, SyscallDescABI<ABI>> _descs;

  public:
    SyscallDescTable(std::initializer_list<SyscallDescABI<ABI>> descs)
    {
        for (auto &desc: descs) {
            auto res = _descs.insert({desc.num(), desc});
            panic_if(!res.second, "Failed to insert desc %s", desc.name());
        }
    }

    SyscallDesc
    *get(int num, bool fatal_if_missing=true)
    {
        auto it = _descs.find(num);
        if (it == _descs.end()) {
            if (fatal_if_missing)
                fatal("Syscall %d out of range", num);
            else
                return nullptr;
        }
        return &it->second;
    }
};

} // namespace gem5

#endif // __SIM_SYSCALL_DESC_HH__
