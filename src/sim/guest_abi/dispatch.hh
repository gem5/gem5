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

#ifndef __SIM_GUEST_ABI_DISPATCH_HH__
#define __SIM_GUEST_ABI_DISPATCH_HH__

#include <functional>
#include <sstream>
#include <tuple>
#include <type_traits>
#include <utility>

#include "base/compiler.hh"
#include "sim/guest_abi/definition.hh"
#include "sim/guest_abi/layout.hh"

namespace gem5
{

class ThreadContext;

namespace guest_abi
{

/*
 * These functions will likely be common among all ABIs and implement the
 * mechanism of gathering arguments, calling the target function, and then
 * storing the result. They might need to be overridden if, for instance,
 * the location of arguments need to be determined in a different order.
 * For example, there might be an ABI which gathers arguments starting
 * from the last in the list instead of the first. This is unlikely but
 * still possible to support by redefining these functions..
 */

template <typename ABI, typename Ret, bool store_ret, typename Target,
          typename State, typename Args, std::size_t... I>
static inline typename std::enable_if_t<!store_ret, Ret>
callFromHelper(Target &target, ThreadContext *tc, State &state, Args &&args,
               std::index_sequence<I...>)
{
    return target(tc, std::get<I>(args)...);
}

template <typename ABI, typename Ret, bool store_ret, typename Target,
          typename State, typename Args, std::size_t... I>
static inline typename std::enable_if_t<store_ret, Ret>
callFromHelper(Target &target, ThreadContext *tc, State &state, Args &&args,
               std::index_sequence<I...>)
{
    Ret ret = target(tc, std::get<I>(args)...);
    storeResult<ABI, Ret>(tc, ret, state);
    return ret;
}

template <typename ABI, typename Ret, bool store_ret, typename... Args>
static inline Ret
callFrom(ThreadContext *tc, typename ABI::State &state,
         std::function<Ret(ThreadContext *, Args...)> target)
{
    // Extract all the arguments from the thread context. Braced initializers
    // are evaluated from left to right.
    auto args = std::tuple<Args...>{ getArgument<ABI, Args>(tc, state)... };

    // Call the wrapper which will call target.
    return callFromHelper<ABI, Ret, store_ret>(
        target, tc, state, std::move(args),
        std::make_index_sequence<sizeof...(Args)>{});
}

/*
 * This function is like the ones above, except it prints the arguments
 * a target function would be called with instead of actually calling it.
 */

template <typename ABI, typename Ret, typename... Args>
static void
dumpArgsFrom(std::ostream &os, [[maybe_unused]] ThreadContext *tc,
             typename ABI::State &state)
{
    int count = 0;
    // Extract all the arguments from the thread context and print them,
    // prefixed with either a ( or a , as appropriate.
    GEM5_FOR_EACH_IN_PACK (os << (count++ ? ", " : "("),
                           os << getArgument<ABI, Args>(tc, state))
        ;
    os << ")";
}

} // namespace guest_abi
} // namespace gem5

#endif // __SIM_GUEST_ABI_DISPATCH_HH__
