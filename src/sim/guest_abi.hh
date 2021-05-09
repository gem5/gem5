/*
 * Copyright 2019 Google Inc.
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

#ifndef __SIM_GUEST_ABI_HH__
#define __SIM_GUEST_ABI_HH__

#include <functional>

#include "sim/guest_abi/definition.hh"
#include "sim/guest_abi/dispatch.hh"
#include "sim/guest_abi/layout.hh"
#include "sim/guest_abi/varargs.hh"

namespace gem5
{

class ThreadContext;

// These functions wrap a simulator level function with the given signature.
// The wrapper takes one argument, a thread context to extract arguments from
// and write a result (if any) back to. For convenience, the wrapper also
// returns the result of the wrapped function.

template <typename ABI, bool store_ret, typename Ret, typename ...Args>
Ret
invokeSimcall(ThreadContext *tc,
              std::function<Ret(ThreadContext *, Args...)> target)
{
    // Default construct a State to track consumed resources. Built in
    // types will be zero initialized.
    auto state = guest_abi::initializeState<ABI>(tc);
    guest_abi::prepareForFunction<ABI, Ret, Args...>(tc, state);
    return guest_abi::callFrom<ABI, Ret, store_ret, Args...>(tc, state,
        target);
}

template <typename ABI, typename Ret, typename ...Args>
Ret
invokeSimcall(ThreadContext *tc,
              std::function<Ret(ThreadContext *, Args...)> target)
{
    return invokeSimcall<ABI, true>(tc, target);
}

template <typename ABI, bool store_ret, typename Ret, typename ...Args>
Ret
invokeSimcall(ThreadContext *tc, Ret (*target)(ThreadContext *, Args...))
{
    return invokeSimcall<ABI, store_ret>(
            tc, std::function<Ret(ThreadContext *, Args...)>(target));
}

template <typename ABI, typename Ret, typename ...Args>
Ret
invokeSimcall(ThreadContext *tc, Ret (*target)(ThreadContext *, Args...))
{
    return invokeSimcall<ABI, true>(tc, target);
}

template <typename ABI, typename ...Args>
void
invokeSimcall(ThreadContext *tc,
              std::function<void(ThreadContext *, Args...)> target)
{
    // Default construct a State to track consumed resources. Built in
    // types will be zero initialized.
    auto state = guest_abi::initializeState<ABI>(tc);
    guest_abi::prepareForArguments<ABI, Args...>(tc, state);
    guest_abi::callFrom<ABI, void, false, Args...>(tc, state, target);
}

template <typename ABI, typename ...Args>
void
invokeSimcall(ThreadContext *tc, void (*target)(ThreadContext *, Args...))
{
    invokeSimcall<ABI>(
            tc, std::function<void(ThreadContext *, Args...)>(target));
}


// These functions also wrap a simulator level function. Instead of running the
// function, they return a string which shows what arguments the function would
// be invoked with if it were called from the given context.

template <typename ABI, typename Ret, typename ...Args>
std::string
dumpSimcall(std::string name, ThreadContext *tc,
            std::function<Ret(ThreadContext *, Args...)> target=
            std::function<Ret(ThreadContext *, Args...)>())
{
    auto state = guest_abi::initializeState<ABI>(tc);
    std::ostringstream ss;

    guest_abi::prepareForFunction<ABI, Ret, Args...>(tc, state);
    ss << name;
    guest_abi::dumpArgsFrom<ABI, Ret, Args...>(ss, tc, state);
    return ss.str();
}

template <typename ABI, typename Ret, typename ...Args>
std::string
dumpSimcall(std::string name, ThreadContext *tc,
            Ret (*target)(ThreadContext *, Args...))
{
    return dumpSimcall<ABI>(
            name, tc, std::function<Ret(ThreadContext *, Args...)>(target));
}

} // namespace gem5

#endif // __SIM_GUEST_ABI_HH__
