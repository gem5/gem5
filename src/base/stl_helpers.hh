/*
 * Copyright (c) 2010 The Hewlett-Packard Development Company
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

#ifndef __BASE_STL_HELPERS_HH__
#define __BASE_STL_HELPERS_HH__

#include <algorithm>
#include <iostream>
#include <type_traits>
#include <vector>

#include "base/compiler.hh"

namespace gem5
{

namespace stl_helpers
{

template <typename T, typename Enabled=void>
struct IsHelpedContainer : public std::false_type {};

template <typename ...Types>
struct IsHelpedContainer<std::vector<Types...>> : public std::true_type {};

template <typename ...Types>
constexpr bool IsHelpedContainerV = IsHelpedContainer<Types...>::value;

/**
 * Write out all elements in an stl container as a space separated
 * list enclosed in square brackets
 *
 * @ingroup api_base_utils
 */

template <typename T>
std::enable_if_t<IsHelpedContainerV<T>, std::ostream &>
operator<<(std::ostream& out, const T &t)
{
    out << "[ ";
    bool first = true;
    auto printer = [&first, &out](const auto &elem) {
        if (first)
            out << elem;
        else
            out << " " << elem;
    };
    std::for_each(t.begin(), t.end(), printer);
    out << " ]";
    out << std::flush;
    return out;
}

} // namespace stl_helpers
} // namespace gem5

#endif // __BASE_STL_HELPERS_HH__
