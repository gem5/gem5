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

#ifndef __SIM_GUEST_ABI_LAYOUT_HH__
#define __SIM_GUEST_ABI_LAYOUT_HH__

#include <type_traits>

#include "base/compiler.hh"
#include "sim/guest_abi/definition.hh"

namespace gem5
{

class ThreadContext;

namespace guest_abi
{

/*
 * State may need to be initialized based on the ThreadContext, for instance
 * to find out where the stack pointer is initially.
 */
template <typename ABI, typename Enabled=void>
struct StateInitializer
{
    static typename ABI::State
    init(const ThreadContext *tc)
    {
        return typename ABI::State();
    }
};

template <typename ABI>
struct StateInitializer<ABI, typename std::enable_if_t<
    std::is_constructible_v<typename ABI::State, const ThreadContext *>>>
{
    static typename ABI::State
    init(const ThreadContext *tc)
    {
        return typename ABI::State(tc);
    }
};

template <typename ABI>
static typename ABI::State
initializeState(const ThreadContext *tc)
{
    return StateInitializer<ABI>::init(tc);
}

/*
 * This struct template provides a default prepare() method in case the
 * Result or Argument template doesn't provide one. This is the default in
 * cases where the return or argument type doesn't affect where things are
 * stored.
 */
template <typename ABI, template <class ...> class Role,
          typename Type, typename Enabled=void>
struct Preparer
{
    static void
    prepare(ThreadContext *tc, typename ABI::State &state)
    {}
};

/*
 * If the return or argument type isn't void and does affect where things
 * are stored, the ABI can implement a prepare() method for the various
 * argument and/or return types, and this specialization will call into it.
 */
template <typename ABI, template <class ...> class Role, typename Type>
struct Preparer<ABI, Role, Type, decltype((void)&Role<ABI, Type>::prepare)>
{
    static void
    prepare(ThreadContext *tc, typename ABI::State &state)
    {
        Role<ABI, Type>::prepare(tc, state);
    }
};

template <typename ABI, typename Ret, typename Enabled=void>
static inline void
prepareForResult(ThreadContext *tc, typename ABI::State &state)
{
    Preparer<ABI, Result, Ret>::prepare(tc, state);
}

template <typename ABI, typename ...Args>
static inline void
prepareForArguments([[maybe_unused]] ThreadContext *tc,
        typename ABI::State &state)
{
    GEM5_FOR_EACH_IN_PACK(Preparer<ABI, Argument, Args>::prepare(tc, state));
}

template <typename ABI, typename Ret, typename ...Args>
static inline void
prepareForFunction(ThreadContext *tc, typename ABI::State &state)
{
    prepareForResult<ABI, Ret>(tc, state);
    prepareForArguments<ABI, Args...>(tc, state);
}

/*
 * This struct template provides a way to call the Result store method and
 * optionally pass it the state.
 */

template <typename ABI, typename Ret, typename Enabled=void>
struct ResultStorer
{
    static void
    store(ThreadContext *tc, const Ret &ret, typename ABI::State &state)
    {
        Result<ABI, Ret>::store(tc, ret);
    }
};

template <typename ABI, typename Ret>
struct ResultStorer<ABI, Ret, typename std::enable_if_t<
    std::is_same_v<void (*)(ThreadContext *, const Ret &,
                            typename ABI::State &),
                 decltype(&Result<ABI, Ret>::store)>>>
{
    static void
    store(ThreadContext *tc, const Ret &ret, typename ABI::State &state)
    {
        Result<ABI, Ret>::store(tc, ret, state);
    }
};

/*
 * Function templates to wrap the Result::store and Argument::get methods.
 */

template <typename ABI, typename Ret>
static void
storeResult(ThreadContext *tc, const Ret &ret, typename ABI::State &state)
{
    ResultStorer<ABI, Ret>::store(tc, ret, state);
}

template <typename ABI, typename Arg>
static Arg
getArgument(ThreadContext *tc, typename ABI::State &state)
{
    auto arg = Argument<ABI, Arg>::get(tc, state);
    return *reinterpret_cast<Arg*>(&arg);
}

} // namespace guest_abi
} // namespace gem5

#endif // __SIM_GUEST_ABI_LAYOUT_HH__
