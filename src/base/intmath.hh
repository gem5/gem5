/*
 * Copyright (c) 2021 ARM Limited
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
#include <utility>

#include "base/bitfield.hh"

namespace gem5
{

/**
 * @ingroup api_base_utils
 */
template <class T>
static constexpr std::enable_if_t<std::is_integral_v<T>, int>
floorLog2(T x)
{
    assert(x > 0);

    // A guaranteed unsigned version of x.
    uint64_t ux = (typename std::make_unsigned<T>::type)x;

    int y = 0;
    constexpr auto ts = sizeof(T);

    if (ts >= 8 && (ux & 0xffffffff00000000ULL)) { y += 32; ux >>= 32; }
    if (ts >= 4 && (ux & 0x00000000ffff0000ULL)) { y += 16; ux >>= 16; }
    if (ts >= 2 && (ux & 0x000000000000ff00ULL)) { y +=  8; ux >>=  8; }
    if (ux & 0x00000000000000f0ULL) { y +=  4; ux >>=  4; }
    if (ux & 0x000000000000000cULL) { y +=  2; ux >>=  2; }
    if (ux & 0x0000000000000002ULL) { y +=  1; }

    return y;
}

/**
 * @ingroup api_base_utils
 */
template <class T>
static constexpr int
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
static constexpr bool
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
static constexpr T
divCeil(const T& a, const U& b)
{
    return (a + b - 1) / b;
}

/**
 * @ingroup api_base_utils
 */
template <typename T>
static constexpr std::enable_if_t<sizeof(T) <= sizeof(uint32_t)>
mulUnsigned(std::make_unsigned_t<T> &high, std::make_unsigned_t<T> &low,
            std::make_unsigned_t<T> val_a, std::make_unsigned_t<T> val_b)
{
    uint64_t product = (uint64_t)val_a * (uint64_t)val_b;
    low = product;
    high = (product >> (sizeof(low) * 8));
};

/**
 * @ingroup api_base_utils
 */
template <typename T>
static constexpr std::enable_if_t<sizeof(T) <= sizeof(uint32_t)>
mulSigned(std::make_signed_t<T> &high, std::make_signed_t<T> &low,
          std::make_signed_t<T> val_a, std::make_signed_t<T> val_b)
{
    uint64_t product = (int64_t)val_a * (int64_t)val_b;
    low = product;
    high = (product >> (sizeof(low) * 8));
};

/**
 * Multiply two values with place value p.
 *
 *  (A * p + a) * (B * p + b) =
 *  (A * B) * p^2 + (a * B + A * b) * p + (a * b)
 *
 *  low result = (a * B + A * b) * p + (a * b)
 *  high result = (A * B) + carry out from low result.
 *
 * As long as p is at most half the capacity of the underlying type, no
 * individual multiplication will overflow. We just have to carefully manage
 * carries to avoid losing any during the addition steps.
 */
template <typename T>
static constexpr std::enable_if_t<sizeof(T) == sizeof(uint64_t)>
mulUnsignedManual(std::make_unsigned_t<T> &high, std::make_unsigned_t<T> &low,
                  std::make_unsigned_t<T> val_a, std::make_unsigned_t<T> val_b)
{
    low = val_a * val_b;

    uint64_t A = (uint32_t)(val_a >> 32);
    uint64_t a = (uint32_t)val_a;
    uint64_t B = (uint32_t)(val_b >> 32);
    uint64_t b = (uint32_t)val_b;

    uint64_t c1 = 0, c2 = 0; // Carry between place values.
    uint64_t ab = a * b, Ab = A * b, aB = a * B, AB = A * B;

    c1 = (uint32_t)(ab >> 32);

    // Be careful to avoid overflow.
    c2 = (c1 >> 1) + (Ab >> 1) + (aB >> 1);
    c2 += ((c1 & 0x1) + (Ab & 0x1) + (aB & 0x1)) >> 1;
    c2 >>= 31;

    high = AB + c2;
}

/**
 * @ingroup api_base_utils
 */
template <typename T>
static constexpr std::enable_if_t<sizeof(T) == sizeof(uint64_t)>
mulUnsigned(std::make_unsigned_t<T> &high, std::make_unsigned_t<T> &low,
            std::make_unsigned_t<T> val_a, std::make_unsigned_t<T> val_b)
{
#ifdef __SIZEOF_INT128__
    __uint128_t val = (__uint128_t)val_a * (__uint128_t)val_b;
    low = val;
    high = (val >> 64);
#else
    mulUnsignedManual<T>(high, low, val_a, val_b);
#endif
}

template <typename T>
static constexpr std::enable_if_t<sizeof(T) == sizeof(uint64_t)>
mulSignedManual(std::make_signed_t<T> &high, std::make_signed_t<T> &low,
                std::make_signed_t<T> val_a, std::make_signed_t<T> val_b)
{
    uint64_t u_high = 0, u_low = 0;
    mulUnsigned<T>(u_high, u_low, val_a, val_b);

    if (val_a < 0)
        u_high -= val_b;
    if (val_b < 0)
        u_high -= val_a;

    high = u_high;
    low = u_low;
}

/**
 * @ingroup api_base_utils
 */
template <typename T>
static constexpr std::enable_if_t<sizeof(T) == sizeof(uint64_t)>
mulSigned(std::make_signed_t<T> &high, std::make_signed_t<T> &low,
          std::make_signed_t<T> val_a, std::make_signed_t<T> val_b)
{
#ifdef __SIZEOF_INT128__
    __int128_t val = (__int128_t)val_a * (__int128_t)val_b;
    low = val;
    high = (val >> 64);
#else
    mulSignedManual<T>(high, low, val_a, val_b);
#endif
}

template <typename T>
static constexpr std::pair<std::make_unsigned_t<T>, std::make_unsigned_t<T>>
mulUnsigned(std::make_unsigned_t<T> val_a, std::make_unsigned_t<T> val_b)
{
    std::make_unsigned_t<T> hi{}, low{};
    mulUnsigned<T>(hi, low, val_a, val_b);
    return {hi, low};
};

template <typename T>
static constexpr std::pair<std::make_signed_t<T>, std::make_signed_t<T>>
mulSigned(std::make_signed_t<T> val_a, std::make_signed_t<T> val_b)
{
    std::make_signed_t<T> hi{}, low{};
    mulSigned<T>(hi, low, val_a, val_b);
    return {hi, low};
};

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
static constexpr T
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
static constexpr T
roundDown(const T& val, const U& align)
{
    assert(isPowerOf2(align));
    T mask = (T)align - 1;
    return val & ~mask;
}

/**
 * Calculate the log2 of a power of 2 integer
 *
 * @param An input value
 * @return The base 2 log of value
 *
 * @ingroup api_base_utils
 */
static constexpr int
log2i(int value)
{
    assert(isPowerOf2(value) && value > 0);
    return ctz32(value);
}

} // namespace gem5

#endif // __BASE_INTMATH_HH__
