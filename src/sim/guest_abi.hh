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
 *
 * Authors: Gabe Black
 */

#ifndef __SIM_GUEST_ABI_HH__
#define __SIM_GUEST_ABI_HH__

#include <functional>
#include <type_traits>

class ThreadContext;

namespace GuestABI
{

/*
 * To implement an ABI, a subclass needs to implement a system of
 * specializations of these two templates Result and Argument, and define a
 * "Position" type.
 *
 * The Position type carries information about, for instance, how many
 * integer registers have been consumed gathering earlier arguments. It
 * may contain multiple elements if there are multiple dimensions to track,
 * for instance the number of integer and floating point registers used so far.
 *
 * Result and Argument are class templates instead of function templates so
 * that they can be partially specialized if necessary. C++ doesn't let you
 * partially specialize function templates because that conflicts with
 * template resolution using the function's arguments. Since we already know
 * what type we want and we don't need argument based resolution, we can just
 * wrap the desired functionality in classes and sidestep the problem.
 *
 * Also note that these templates have an "Enabled" parameter to support
 * std::enable_if style conditional specializations.
 */

template <typename ABI, typename Ret, typename Enabled=void>
struct Result
{
  private:
    /*
     * Store result "ret" into the state accessible through tc.
     *
     * Note that the declaration below is only to document the expected
     * signature and is private so it won't be used by accident.
     * Specializations of this Result class should define their own version
     * of this method which actually does something and is public.
     */
    static void store(ThreadContext *tc, const Ret &ret);
};

template <typename ABI, typename Arg, typename Enabled=void>
struct Argument
{
    /*
     * Retrieve an argument of type Arg from the state accessible through tc,
     * assuming the state represented by "position" has already been used.
     * Also update position to account for this argument as well.
     *
     * Like Result::store above, the declaration below is only to document
     * the expected method signature.
     */
    static Arg get(ThreadContext *tc, typename ABI::Position &position);
};


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
template <typename ABI, typename Ret>
static typename std::enable_if<!std::is_void<Ret>::value, Ret>::type
callFrom(ThreadContext *tc, typename ABI::Position &position,
        std::function<Ret(ThreadContext *)> target)
{
    Ret ret = target(tc);
    Result<ABI, Ret>::store(tc, ret);
    return ret;
}

// With no arguments to gather and nothing to return, call the target function.
template <typename ABI>
static void
callFrom(ThreadContext *tc, typename ABI::Position &position,
        std::function<void(ThreadContext *)> target)
{
    target(tc);
}

// Recursively gather arguments for target from tc until we get to the base
// case above.
template <typename ABI, typename Ret, typename NextArg, typename ...Args>
static typename std::enable_if<!std::is_void<Ret>::value, Ret>::type
callFrom(ThreadContext *tc, typename ABI::Position &position,
        std::function<Ret(ThreadContext *, NextArg, Args...)> target)
{
    // Extract the next argument from the thread context.
    NextArg next = Argument<ABI, NextArg>::get(tc, position);

    // Build a partial function which adds the next argument to the call.
    std::function<Ret(ThreadContext *, Args...)> partial =
        [target,next](ThreadContext *_tc, Args... args) {
            return target(_tc, next, args...);
        };

    // Recursively handle any remaining arguments.
    return callFrom<ABI, Ret, Args...>(tc, position, partial);
}

// Recursively gather arguments for target from tc until we get to the base
// case above. This version is for functions that don't return anything.
template <typename ABI, typename NextArg, typename ...Args>
static void
callFrom(ThreadContext *tc, typename ABI::Position &position,
        std::function<void(ThreadContext *, NextArg, Args...)> target)
{
    // Extract the next argument from the thread context.
    NextArg next = Argument<ABI, NextArg>::get(tc, position);

    // Build a partial function which adds the next argument to the call.
    std::function<void(ThreadContext *, Args...)> partial =
        [target,next](ThreadContext *_tc, Args... args) {
            target(_tc, next, args...);
        };

    // Recursively handle any remaining arguments.
    callFrom<ABI, Args...>(tc, position, partial);
}

} // namespace GuestABI


// These functions wrap a simulator level function with the given signature.
// The wrapper takes one argument, a thread context to extract arguments from
// and write a result (if any) back to. For convenience, the wrapper also
// returns the result of the wrapped function.

template <typename ABI, typename Ret, typename ...Args>
Ret
invokeSimcall(ThreadContext *tc,
              std::function<Ret(ThreadContext *, Args...)> target)
{
    // Default construct a Position to track consumed resources. Built in
    // types will be zero initialized.
    auto position = typename ABI::Position();
    return GuestABI::callFrom<ABI, Ret, Args...>(tc, position, target);
}

template <typename ABI, typename Ret, typename ...Args>
Ret
invokeSimcall(ThreadContext *tc, Ret (*target)(ThreadContext *, Args...))
{
    return invokeSimcall<ABI>(
            tc, std::function<Ret(ThreadContext *, Args...)>(target));
}

template <typename ABI, typename ...Args>
void
invokeSimcall(ThreadContext *tc,
              std::function<void(ThreadContext *, Args...)> target)
{
    // Default construct a Position to track consumed resources. Built in
    // types will be zero initialized.
    auto position = typename ABI::Position();
    GuestABI::callFrom<ABI, Args...>(tc, position, target);
}

template <typename ABI, typename ...Args>
void
invokeSimcall(ThreadContext *tc, void (*target)(ThreadContext *, Args...))
{
    invokeSimcall<ABI>(
            tc, std::function<void(ThreadContext *, Args...)>(target));
}

#endif // __SIM_GUEST_ABI_HH__
