/*
 * Copyright (c) 2022 Arteris, Inc. and its applicable licensors and
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

#ifndef __BASE_TYPETRAITS_HH__
#define __BASE_TYPETRAITS_HH__

#include <tuple>
#include <type_traits>

namespace gem5
{

/*
 * Type traits that enable inspecting the signature of a member function based
 * on a pointer to that function. Specifically, these type traits provide a
 * class_t, a return_t and  a argsTuple_t alias that correspond respectively to
 * the class that the function is a member of, the return type of the member
 * function and the list of parameters types packed in a tuple. Convenience
 * Convenience template aliases are also provided.
 *
 * Example, assuming "struct Struct {void foo(int, bool);};":
 *    - MemberFunctionClass_t<&Struct::foo> is Struct.
 *    - MemberFunctionReturn_t<&Struct::foo> is void.
 *    - MemberFunctionArgsTuple_t<&Struct::foo> is std::tuple<int, bool>.
 */

template<typename F>
struct MemberFunctionSignature;
template<typename C, typename R, class... A>
struct MemberFunctionSignature<R(C::*)(A...)>
{
    using class_t = C;
    using return_t = R;
    using argsTuple_t = std::tuple<A...>;
};
template<typename C, typename R, class... A>
struct MemberFunctionSignature<R(C::*)(A...) const>
{
    using class_t = std::add_const_t<C>;
    using return_t = R;
    using argsTuple_t = std::tuple<A...>;
};
template<typename C, typename R, class... A>
struct MemberFunctionSignature<R(C::*)(A...) volatile>
{
    using class_t = std::add_volatile_t<C>;
    using return_t = R;
    using argsTuple_t = std::tuple<A...>;
};
template<typename C, typename R, class... A>
struct MemberFunctionSignature<R(C::*)(A...) const volatile>
{
    using class_t = std::add_cv_t<C>;
    using return_t = R;
    using argsTuple_t = std::tuple<A...>;
};
template<auto F>
using MemberFunctionClass_t =
    typename MemberFunctionSignature<decltype(F)>::class_t;

template<auto F>
using MemberFunctionReturn_t =
    typename MemberFunctionSignature<decltype(F)>::return_t;

template<auto F>
using MemberFunctionArgsTuple_t =
    typename MemberFunctionSignature<decltype(F)>::argsTuple_t;

} // namespace gem5

#endif // __BASE_TYPETRAITS_HH__
