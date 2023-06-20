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

#ifndef BASE_STL_HELPERS_OSTREAM_HELPERS_HH
#define BASE_STL_HELPERS_OSTREAM_HELPERS_HH

#include <iostream>
#include <memory>
#include <tuple>
#include <utility>

#include "base/type_traits.hh"
#include "magic_enum/magic_enum.hh"

namespace gem5::stl_helpers
{

/*
 * Wrap any object in a Printer object to force using a opExtract_impl printing
 * function. This is not required for types that do not already enable
 * operator<< in another namespace. However, to enable the special printing
 * function for, e.g., raw pointers, those must be wrapped in a Printer.
 */
template<typename T>
struct Printer
{
    Printer(const T& value): value{value} {}
    const T& value;
};

namespace opExtract_impl
{

/*
 * In order to provide a specialization for operator<< with stl_helpers-enabled
 * types
 * without loosing the hability to use it with other types, a dual-dispatch
 * mechanism is used. The only entry point in the system is through a primary
 * dispatch function that won't resolve for non-helped types. Then, recursive
 * calls go through the secondary dispatch interface that sort between helped
 * and non-helped types. Helped typed will enter the system back through the
 * primary dispatch interface while other types will look for operator<<
 * through regular lookup, especially ADL.
 */

template<typename T>
std::ostream&
opExtractSecDisp(std::ostream& os, const T& v);

template <typename E>
std::enable_if_t<std::is_enum_v<E>,
std::ostream&>
opExtractPrimDisp(std::ostream& os, const E& e)
{
    return os << magic_enum::enum_name(e);
}

template <typename... T>
std::ostream&
opExtractPrimDisp(std::ostream& os, const std::tuple<T...>& p)
{
    std::apply([&](auto&&... e) {
        std::size_t n{0};
        os << '(';
        ((opExtractSecDisp(os, e) << (++n != sizeof...(T) ? ", " : "")), ...);
        os << ')';
    }, p);
    return os;
}

template <typename T, typename U>
std::ostream&
opExtractPrimDisp(std::ostream& os, const std::pair<T, U>& p)
{
    return opExtractPrimDisp(os, std::tie(p.first, p.second));
}

template <typename T>
std::enable_if_t<is_iterable_v<T>, std::ostream&>
opExtractPrimDisp(std::ostream& os, const T& v)
{
    os << "[ ";
    for (auto& e: v) {
        opExtractSecDisp(os, e) << ", ";
    }
    return os << ']';
}

template <typename T>
std::ostream&
opExtractPrimDisp(std::ostream& os, const std::optional<T>& o)
{
    if (o) {
        return opExtractSecDisp(os, *o);
    } else {
        return os << '-';
    }
}

template <typename T>
std::ostream&
opExtractPrimDisp(std::ostream& os, T* p);

template <typename T>
std::ostream&
opExtractPrimDisp(std::ostream& os, const std::shared_ptr<T>& p)
{
    return opExtractPrimDisp(os, p.get());
}

template <typename T>
std::ostream&
opExtractPrimDisp(std::ostream& os, const std::unique_ptr<T>& p)
{
    return opExtractPrimDisp(os, p.get());
}

template <typename T>
std::ostream&
opExtractPrimDisp(std::ostream& os, const Printer<T>& p);

template <typename, typename = void>
constexpr bool isOpExtractNativelySupported = false;

template <typename T>
constexpr bool isOpExtractNativelySupported<T,
    std::void_t<decltype(
        std::declval<std::ostream&>() << std::declval<T>())>> = true;

template <typename, typename = void>
constexpr bool isOpExtractHelped = false;

template <typename T>
constexpr bool isOpExtractHelped<T,
    std::void_t<decltype(
        opExtractPrimDisp(std::declval<std::ostream&>(),
                          std::declval<T>()))>>
    = true;

template <typename T>
constexpr bool needsDispatch =
    isOpExtractHelped<T> && !isOpExtractNativelySupported<T>;

template <typename T>
std::ostream&
opExtractPrimDisp(std::ostream& os, T* p)
{
    if (!p) {
        return os << "nullptr";
    }
    if constexpr (isOpExtractHelped<T> || isOpExtractNativelySupported<T>) {
        os << '(' << p << ": ";
        opExtractSecDisp(os, *p);
        return os << ')';
    } else {
        return os << p;
    }
}

template <typename T>
std::ostream&
opExtractPrimDisp(std::ostream& os, const Printer<T>& p)
{
    if constexpr (isOpExtractHelped<T>) {
        return opExtractPrimDisp(os, p.value);
    } else {
        return os << p.value;
    }
}


template<typename T>
std::ostream&
opExtractSecDisp(std::ostream& os, const T& v)
{
    if constexpr (needsDispatch<T>) {
        return opExtractPrimDisp(os, v);
    } else {
        return os << v;
    }
}

} // namespace opExtract_impl

// use the Printer wrapper or add "using stl_helpers::operator<<" in the scope
// where you want to use that operator<<.
template<typename T>
std::enable_if_t<opExtract_impl::needsDispatch<T>, std::ostream&>
operator<<(std::ostream& os, const T& v)
{
    return opExtract_impl::opExtractPrimDisp(os, v);
}

} // namespace gem5::stl_helpers

#endif // BASE_STL_HELPERS_OSTREAM_HELPERS_HH
