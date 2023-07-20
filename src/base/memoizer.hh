/*
 * Copyright (c) 2022 Arm Limited
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

#ifndef __BASE_MEMOIZER_HH__
#define __BASE_MEMOIZER_HH__

#include <functional>
#include <map>
#include <type_traits>
#include <utility>

namespace gem5
{

/**
 * This class takes a function as a constructor argument and memoizes
 * it: every time the function gets invoked through the Memoizer object
 * (see operator()), the result gets saved in the internal cache, ready
 * to be retrieved next time an invokation is made with the same
 * arguments.
 *
 * Example usage:
 *
 * int fibonacci(int n);
 *
 * Memoizer fibonacci_memo(fibonacci);
 * fibonacci_memo(5);
 *
 * Not every function is eligible for memoization. We explicitly
 * validate the memoizer at compile time and produce an error if the
 * function signature contains a reference.
 *
 * There are two ways to discard a memoization
 *
 * 1) Delete the Memoizer object
 * 2) Use the Memoizer::flush method
 *
 * In some cases there is little or no reason to discard a memoization
 * (like in the fibonacci example, where fibonacci(k) always returns
 * the same value for the same input k)
 * The memoizer could be used in more complex cases, where a change in
 * the global state affects the output of the function, which
 * effectively invalidates the cached results.
 * It is up to the client to understand when memoization is no longer
 * valid and to flush the result cache as a consequence.
 */
template <typename Ret, typename... Args>
class Memoizer
{
  public:
    using ret_type = Ret;
    using args_type = std::tuple<Args...>;

    constexpr Memoizer(Ret (*_func)(Args...))
     : func(_func)
    {
        validateMemoizer();
    }

    Memoizer() = delete;
    Memoizer(const Memoizer &rhs) = delete;
    Memoizer& operator=(const Memoizer &rhs) = delete;

    auto
    operator()(Args... args) const
    {
        auto key = std::make_tuple(args...);
        if (auto it = cache.find(key); it != cache.end()) {
            // Return the cached result
            return it->second;
        }
        auto emplaced = cache.emplace(key, func(args...));
        return emplaced.first->second;
    };

    /** Clear the memoization cache */
    void flush() { cache.clear(); }

  protected:
    /** True if the passed value is cached, false otherwise */
    bool
    cached(args_type args) const
    {
        return cache.find(args) != cache.end();
    }

    /** Return the size of the memoizer cache */
    size_t
    cacheSize() const
    {
        return cache.size();
    }

  private:
    /**
     * Validate the memoizer and produce an error if the
     * function signature contains a reference.
     */
    constexpr void
    validateMemoizer()
    {
        constexpr size_t num_args = std::tuple_size_v<std::decay_t<args_type>>;
        iterateTupleArgs<size_t(0), num_args, size_t(1)>([&](auto i) {
            static_assert(!std::is_reference_v<
                typename std::tuple_element<
                    i.value, args_type>::type>);
        });
    }

    template <size_t Start, size_t End, size_t Inc, class F>
    constexpr void
    iterateTupleArgs(F&& func)
    {
        if constexpr (Start < End) {
            func(std::integral_constant<decltype(Start), Start>());
            iterateTupleArgs<Start + Inc, End, Inc>(func);
        }
    }

  private:
    /** Memoized function */
    const std::function<Ret(Args...)> func;
    /** Result cache */
    mutable std::map<args_type, ret_type> cache;

};

} // namespace gem5

#endif // __BASE_MEMOIZER_HH__
