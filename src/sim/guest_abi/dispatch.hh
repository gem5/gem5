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
#include <type_traits>

#include "sim/guest_abi/definition.hh"
#include "sim/guest_abi/layout.hh"

class ThreadContext;

namespace GuestABI
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

// With no arguments to gather, call the target function and store the
// result.
template <typename ABI, bool store_ret, typename Ret>
static typename std::enable_if<!std::is_void<Ret>::value && store_ret,
                Ret>::type
callFrom(ThreadContext *tc, typename ABI::State &state,
        std::function<Ret(ThreadContext *)> target)
{
    Ret ret = target(tc);
    storeResult<ABI, Ret>(tc, ret, state);
    return ret;
}

template <typename ABI, bool store_ret, typename Ret>
static typename std::enable_if<!std::is_void<Ret>::value && !store_ret,
                Ret>::type
callFrom(ThreadContext *tc, typename ABI::State &state,
        std::function<Ret(ThreadContext *)> target)
{
    return target(tc);
}

// With no arguments to gather and nothing to return, call the target function.
template <typename ABI>
static void
callFrom(ThreadContext *tc, typename ABI::State &state,
        std::function<void(ThreadContext *)> target)
{
    target(tc);
}

// Recursively gather arguments for target from tc until we get to the base
// case above.
template <typename ABI, bool store_ret, typename Ret,
          typename NextArg, typename ...Args>
static typename std::enable_if<!std::is_void<Ret>::value, Ret>::type
callFrom(ThreadContext *tc, typename ABI::State &state,
        std::function<Ret(ThreadContext *, NextArg, Args...)> target)
{
    // Extract the next argument from the thread context.
    NextArg next = getArgument<ABI, NextArg>(tc, state);

    // Build a partial function which adds the next argument to the call.
    std::function<Ret(ThreadContext *, Args...)> partial =
        [target,next](ThreadContext *_tc, Args... args) {
            return target(_tc, next, args...);
        };

    // Recursively handle any remaining arguments.
    return callFrom<ABI, store_ret, Ret, Args...>(tc, state, partial);
}

// Recursively gather arguments for target from tc until we get to the base
// case above. This version is for functions that don't return anything.
template <typename ABI, typename NextArg, typename ...Args>
static void
callFrom(ThreadContext *tc, typename ABI::State &state,
        std::function<void(ThreadContext *, NextArg, Args...)> target)
{
    // Extract the next argument from the thread context.
    NextArg next = getArgument<ABI, NextArg>(tc, state);

    // Build a partial function which adds the next argument to the call.
    std::function<void(ThreadContext *, Args...)> partial =
        [target,next](ThreadContext *_tc, Args... args) {
            target(_tc, next, args...);
        };

    // Recursively handle any remaining arguments.
    callFrom<ABI, Args...>(tc, state, partial);
}



/*
 * These functions are like the ones above, except they print the arguments
 * a target function would be called with instead of actually calling it.
 */

// With no arguments to print, add the closing parenthesis and return.
template <typename ABI, typename Ret>
static void
dumpArgsFrom(int count, std::ostream &os, ThreadContext *tc,
             typename ABI::State &state)
{
    os << ")";
}

// Recursively gather arguments for target from tc until we get to the base
// case above, and append those arguments to the string stream being
// constructed.
template <typename ABI, typename Ret, typename NextArg, typename ...Args>
static void
dumpArgsFrom(int count, std::ostream &os, ThreadContext *tc,
             typename ABI::State &state)
{
    // Either open the parenthesis or add a comma, depending on where we are
    // in the argument list.
    os << (count ? ", " : "(");

    // Extract the next argument from the thread context.
    NextArg next = getArgument<ABI, NextArg>(tc, state);

    // Add this argument to the list.
    os << next;

    // Recursively handle any remaining arguments.
    dumpArgsFrom<ABI, Ret, Args...>(count + 1, os, tc, state);
}

} // namespace GuestABI

#endif // __SIM_GUEST_ABI_DISPATCH_HH__
