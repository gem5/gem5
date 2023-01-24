/*
 * Copyright (c) 2008 The Hewlett-Packard Development Company
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

#ifndef __BASE_CAST_HH__
#define __BASE_CAST_HH__

#include <cassert>
#include <type_traits>
#include "base/logging.hh"

namespace gem5
{

// This is designed for situations where we have a pointer to a base
// type, but in all cases when we cast it to a derived type, we know
// by construction that it should work correctly.

#if defined(DEBUG)

// In debug builds, do the dynamic cast and assert the result is good

template <class T, class U>
inline T
safe_cast(U&& ref_or_ptr)
{
    /*
     * srd::forward used in conjunction with forwarding references (template T
     * + T&&) ensures that dynamic_cast will see the exact same type that was
     * passed to safe_cast (a.k.a., perfect forwarding).
     *
     * Not using std::forward would make safe_cast compile with references to
     * temporary objects and thus return a dangling reference.
     */
    T ret = dynamic_cast<T>(std::forward<U>(ref_or_ptr));
    if constexpr (std::is_pointer_v<T>) {
        gem5_assert(ret);
    }
    return ret;
}

#else

// In non debug builds statically cast the result to the pointer we
// want to use.  This is technically unsafe, but this is only for
// cases where we know that this should work by construction.

template <class T, class U>
inline T
safe_cast(U&& ref_or_ptr)
{
    /*
     * safe_cast should be reserved to polymorphic types while static_cast is
     * also allowed for non-polymorphic types. It could make safe_cast able to
     * compile in a non-debug build and fail in a debug build.
     */
    static_assert(std::is_polymorphic_v<
        std::remove_pointer_t<
        std::remove_reference_t<
        U>>
    >);
    return static_cast<T>(std::forward<U>(ref_or_ptr));
}

#endif

} // namespace gem5

#endif // __BASE_CAST_HH__
