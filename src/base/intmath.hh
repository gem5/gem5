/*
 * Copyright (c) 2001, 2003-2005 The Regents of The University of Michigan
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

#ifndef __BASE_INTMATH_HH__
#define __BASE_INTMATH_HH__

#include <cassert>
#include <cstdint>
#include <type_traits>

#include "base/logging.hh"
#include "base/types.hh"

/**
 * @ingroup api_base_utils
 */
inline uint64_t
power(uint32_t n, uint32_t e)
{
    uint64_t result = 1;
    uint64_t component = n;
    while (e) {
        uint64_t last = result;
        if (e & 0x1)
            result *= component;
        warn_if(result < last, "power() overflowed!");
        e >>= 1;
        component *= component;
    }
    return result;
}

/**
 * @ingroup api_base_utils
 */
template <class T>
inline typename std::enable_if<std::is_integral<T>::value, int>::type
floorLog2(T x)
{
    assert(x > 0);

    // A guaranteed unsigned version of x.
    uint64_t ux = (typename std::make_unsigned<T>::type)x;

    int y = 0;
    constexpr auto ts = sizeof(T);

    if (ts >= 8 && (ux & ULL(0xffffffff00000000))) { y += 32; ux >>= 32; }
    if (ts >= 4 && (ux & ULL(0x00000000ffff0000))) { y += 16; ux >>= 16; }
    if (ts >= 2 && (ux & ULL(0x000000000000ff00))) { y +=  8; ux >>=  8; }
    if (ux & ULL(0x00000000000000f0)) { y +=  4; ux >>=  4; }
    if (ux & ULL(0x000000000000000c)) { y +=  2; ux >>=  2; }
    if (ux & ULL(0x0000000000000002)) { y +=  1; }

    return y;
}

/**
 * @ingroup api_base_utils
 */
template <class T>
inline int
ceilLog2(const T& n)
{
    assert(n > 0);
    if (n == 1)
        return 0;

    return floorLog2(n - (T)1) + 1;
}

/**
 * @ingroup api_base_utils
 */
template <class T>
inline bool
isPowerOf2(const T& n)
{
    // If n is non-zero, and subtracting one borrows all the way to the MSB
    // and flips all bits, then this is a power of 2.
    return n && !(n & (n - 1));
}

/**
 * @ingroup api_base_utils
 */
template <class T, class U>
inline T
divCeil(const T& a, const U& b)
{
    return (a + b - 1) / b;
}

/**
 * This function is used to align addresses in memory.
 *
 * @param val is the address to be aligned.
 * @param align is the alignment. Can only be a power of 2.
 * @return The aligned address. The smallest number divisible
 * by @param align which is greater than or equal to @param val.
 *
 * @ingroup api_base_utils
 */
template <class T, class U>
inline T
roundUp(const T& val, const U& align)
{
    assert(isPowerOf2(align));
    T mask = (T)align - 1;
    return (val + mask) & ~mask;
}

/**
 * This function is used to align addresses in memory.
 *
 * @param val is the address to be aligned.
 * @param align is the alignment. Can only be a power of 2.
 * @return The aligned address. The biggest number divisible
 * by @param align which is less than or equal to @param val.
 *
 * @ingroup api_base_utils
 */
template <class T, class U>
inline T
roundDown(const T& val, const U& align)
{
    assert(isPowerOf2(align));
    T mask = (T)align - 1;
    return val & ~mask;
}

#endif // __BASE_INTMATH_HH__
