/*
 * Copyright (c) 2023 Arteris, Inc. and its applicable licensors and
 * affiliates.
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef BASE_STL_HELPERS_HASH_HELPERS_HH
#define BASE_STL_HELPERS_HASH_HELPERS_HH

#include <functional>
#include <numeric>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "base/type_traits.hh"

namespace gem5::stl_helpers
{

namespace hash_impl
{
// The math in hash_combine and hash_refine functions are inspired from Jon
// Maiga's work hosted at https://github.com/jonmaiga/mx3 under the CC0
// license. It makes use of two components: a stream mixer for combination and
// a scalar mixer for refinement.
// The stream mixer is a lighter weight function with lower entropy used to
// combine hash values while the scalar mixer is a high entropy function that
// increases the overall hashing quality.
// The tradeoff of not using hash_refine has not been thoroughtly tested and is
// only done based on Maiga's return on exprerience.
static constexpr uint64_t C = 0xbea225f9eb34556d;
template<typename... T>
constexpr size_t hash_combine(T... hashes) {
    // gcc reports unused variable if T is the empty pack
    [[maybe_unused]] auto combine = [](uint64_t a, uint64_t b) {
        b *= C;
        b ^= b >> 39;
        a += b * C;
        a *= C;
        return a;
    };
    // The following couple of expressions is equivalent to a hypothetical
    // functional "acc = hashes.fold_left(0, combine)". The comma operator
    // effectively repeats the expression in the second level parenthesis for
    // each argument in the parameter pack hashes, in order. Thus, final value
    // of acc is the recursive combination of all hashes.
    uint64_t acc{0};
    ((acc = combine(acc, static_cast<uint64_t>(hashes))), ...);
    return static_cast<size_t>(acc);
}

constexpr size_t hash_refine(size_t x) {
    x ^= x >> 32;
    x *= C;
    x ^= x >> 29;
    x *= C;
    x ^= x >> 32;
    x *= C;
    x ^= x >> 29;
    return static_cast<size_t>(x);
}

// SFINAE-enabled hash functor
template<typename T, typename = void>
struct hash;

// Reuse std::hash whenever possible
template<typename T>
struct hash<T, std::enable_if_t<is_std_hash_enabled_v<T>>>: std::hash<T>
{};

// Enable type deduction for hash object construction
template<typename T>
constexpr auto make_hash_for(const T&) {
    return hash<T>();
}

// Compute a hash without the hassle of constructing a hash functor
template<typename T>
constexpr auto hash_value(const T& v) {
    return make_hash_for(v)(v);
}

// Hash for tuple
template<typename... T>
struct hash<std::tuple<T...>>
{
    constexpr size_t operator()(const std::tuple<T...>& t) const {
        if constexpr (sizeof...(T) == 0) {
            return 0;
        } else {
            return std::apply([](const auto&... e){
               return hash_refine(hash_combine(hash_value(e)...));
            }, t);
        }
    }
};

// Hash for pairs (based on hash for 2-uple)
template<typename T, typename U>
struct hash<std::pair<T, U>>
{
    constexpr size_t operator()(const std::pair<T, U>& p) const {
        return hash_value(std::tie(p.first, p.second));
    }
};

// Hash for any iterable of stl_helpers::hash-enabled types.
template<typename T>
struct hash<T, std::enable_if_t<
    !is_std_hash_enabled_v<T> && is_iterable_v<T>>>
{
    constexpr size_t operator()(const T& t) const {
        auto b = begin(t);
        auto e = end(t);
        if (b == e) return 0;
        // Equivalent to hypothetical functional style
        // return t.map(hash_value).reduce(hash_combine)
        auto h = std::accumulate(next(b), e, hash_value(*b),
            [](const auto& acc, const auto& val) {
                return hash_combine(acc, hash_value(val));
            });
        return hash_refine(h);
    }
};

template<typename, typename = void>
constexpr bool is_hash_enabled = false;

template <typename T>
constexpr bool is_hash_enabled<T,
    std::void_t<decltype(hash<T>()(std::declval<T>()))>> = true;

} // namespace hash_impl

// Export useful hash_impl functions
using hash_impl::hash;
using hash_impl::make_hash_for;
using hash_impl::hash_value;
using hash_impl::is_hash_enabled;

} // namespace gem5::stl_helpers

#endif // BASE_STL_HELPERS_HASH_HELPERS_HH
