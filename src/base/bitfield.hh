/*
 * Copyright (c) 2017, 2019 ARM Limited
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
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

#ifndef __BASE_BITFIELD_HH__
#define __BASE_BITFIELD_HH__

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace gem5
{

extern const uint8_t reverseBitsLookUpTable[];

/**
 * Generate a 64-bit mask of 'nbits' 1s, right justified. If a number of bits
 * greater than 64 is given, it is truncated to 64.
 *
 * @param nbits The number of bits set in the mask.
 *
 * @ingroup api_bitfield
 */
constexpr uint64_t
mask(unsigned nbits)
{
    return (nbits >= 64) ? (uint64_t)-1LL : (1ULL << nbits) - 1;
}

/**
 * Extract the bitfield from position 'first' to 'last' (inclusive)
 * from 'val' and right justify it.  MSB is numbered 63, LSB is 0.
 *
 * @ingroup api_bitfield
 */
template <class T>
constexpr T
bits(T val, unsigned first, unsigned last)
{
    assert(first >= last);
    int nbits = first - last + 1;
    return (val >> last) & mask(nbits);
}

/**
 * Extract the bit from this position from 'val' and right justify it.
 *
 * @ingroup api_bitfield
 */
template <class T>
constexpr T
bits(T val, unsigned bit)
{
    return bits(val, bit, bit);
}

/**
 * Mask off the given bits in place like bits() but without shifting.
 * msb = 63, lsb = 0
 *
 * @ingroup api_bitfield
 */
template <class T>
constexpr T
mbits(T val, unsigned first, unsigned last)
{
    return val & (mask(first + 1) & ~mask(last));
}

/**
 * @ingroup api_bitfield
 */
constexpr uint64_t
mask(unsigned first, unsigned last)
{
    return mbits((uint64_t)-1LL, first, last);
}

/**
 * Sign-extend an N-bit value to 64 bits. Assumes all bits past the sign are
 * currently zero. For true sign extension regardless of the value of the sign
 * bit, see szext.
 *
 * @ingroup api_bitfield
 */
template <int N>
constexpr uint64_t
sext(uint64_t val)
{
    bool sign_bit = bits(val, N - 1);
    if (sign_bit)
        val |= ~mask(N);
    return val;
}

/**
 * Sign-extend an N-bit value to 64 bits. Zero any bits past the sign if
 * necessary.
 *
 * @ingroup api_bitfield
 */
template <int N>
constexpr uint64_t
szext(uint64_t val)
{
    bool sign_bit = bits(val, N - 1);
    if (sign_bit)
        val |= ~mask(N);
    else
        val &= mask(N);
    return val;
}

/**
 * Returns val with bits first to last set to the LSBs of bit_val
 *
 * E.g.:
 * first: 7
 * last:  4
 * val:      0xFFFF
 * bit_val:  0x0000
 * returned: 0xFF0F
 *
 * @ingroup api_bitfield
 */
template <class T, class B>
constexpr T
insertBits(T val, unsigned first, unsigned last, B bit_val)
{
    assert(first >= last);
    T bmask = mask(first, last);
    val &= ~bmask;
    val |= ((T)bit_val << last) & bmask;
    return val;
}

/**
 * Overloaded for access to only one bit in value
 *
 * @ingroup api_bitfield
 */
template <class T, class B>
constexpr T
insertBits(T val, unsigned bit, B bit_val)
{
    return insertBits(val, bit, bit, bit_val);
}

/**
 * A convenience function to replace bits first to last of val with bit_val
 * in place. It is functionally equivalent to insertBits.
 *
 * \note "first" is the MSB and "last" is the LSB. "first" >= "last"
 *
 * @ingroup api_bitfield
 */
template <class T, class B>
constexpr void
replaceBits(T& val, unsigned first, unsigned last, B bit_val)
{
    val = insertBits(val, first, last, bit_val);
}

/**
 * Overloaded function to allow to access only 1 bit
 *
 * @ingroup api_bitfield
 */
template <class T, class B>
constexpr void
replaceBits(T& val, unsigned bit, B bit_val)
{
    val = insertBits(val, bit, bit, bit_val);
}

/**
 * Takes a value and returns the bit reversed version.
 *
 * E.g.:
 * val:      0x0303
 * returned: 0xc0c0
 *
 * val:      0x0303
 * size:     1
 * returned: 0x03c0
 *
 * Algorithm from:
 * http://graphics.stanford.edu/~seander/bithacks.html#ReverseBitsByLookupTable
 *
 * @param val: variable length value
 * @param size: number of bytes to mirror
 * @return reversed value
 *
 * @ingroup api_bitfield
 */
template <class T>
std::enable_if_t<std::is_integral_v<T>, T>
reverseBits(T val, size_t size=sizeof(T))
{
    assert(size <= sizeof(T));

    if constexpr (sizeof(T) == 1) {
        return reverseBitsLookUpTable[val];
    } else {
        T output = {};

        for (size_t byte = 0; byte < size; byte++) {
            output = (output << 8) | reverseBitsLookUpTable[val & mask(8)];
            val >>= 8;
        }

        return output;
    }
}

/**
 * Returns the bit position of the MSB that is set in the input
 *
 * @ingroup api_bitfield
 */
constexpr int
findMsbSet(uint64_t val)
{
    int msb = 0;
    if (!val)
        return 0;
    if (bits(val, 63, 32)) {
        msb += 32;
        val >>= 32;
    }
    if (bits(val, 31, 16)) {
        msb += 16;
        val >>= 16;
    }
    if (bits(val, 15, 8)) {
        msb += 8;
        val >>= 8;
    }
    if (bits(val, 7, 4)) {
        msb += 4;
        val >>= 4;
    }
    if (bits(val, 3, 2)) {
        msb += 2;
        val >>= 2;
    }
    if (bits(val, 1, 1))
        msb += 1;
    return msb;
}

/**
 * Returns the bit position of the LSB that is set in the input
 *
 * @ingroup api_bitfield
 */
constexpr int
findLsbSet(uint64_t val)
{
    int lsb = 0;
    if (!val)
        return sizeof(val) * 8;
    if (!bits(val, 31, 0)) {
        lsb += 32;
        val >>= 32;
    }
    if (!bits(val, 15, 0)) {
        lsb += 16;
        val >>= 16;
    }
    if (!bits(val, 7, 0)) {
        lsb += 8;
        val >>= 8;
    }
    if (!bits(val, 3, 0)) {
        lsb += 4;
        val >>= 4;
    }
    if (!bits(val, 1, 0)) {
        lsb += 2;
        val >>= 2;
    }
    if (!bits(val, 0, 0))
        lsb += 1;
    return lsb;
}

/**
 * Returns the number of set ones in the provided value.
 * PD algorithm from
 * http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
 *
 * @ingroup api_bitfield
 */
constexpr int
popCount(uint64_t val)
{
#ifndef __has_builtin
#   define __has_builtin(foo) 0
#endif
#if defined(__GNUC__) || \
    (defined(__clang__) && __has_builtin(__builtin_popcountl))
    return __builtin_popcountl(val);
#else
    const uint64_t m1 = 0x5555555555555555ULL;  // ..010101b
    const uint64_t m2 = 0x3333333333333333ULL;  // ..110011b
    const uint64_t m4 = 0x0f0f0f0f0f0f0f0fULL;  // ..001111b
    const uint64_t sum = 0x0101010101010101ULL;

    val -= (val >> 1) & m1;               // 2 bits count -> 2 bits
    val = (val & m2) + ((val >> 2) & m2); // 4 bits count -> 4 bits
    val = (val + (val >> 4)) & m4;        // 8 bits count -> 8 bits
    return (val * sum) >> 56;             // horizontal sum
#endif // defined(__GNUC__) ||
    //(defined(__clang__) && __has_builtin(__builtin_popcountl))
}

/**
 * Align to the next highest power of two.
 *
 * The number passed in is aligned to the next highest power of two,
 * if it is not already a power of two. Please note that if 0 is
 * passed in, 0 is returned.
 *
 * This code has been modified from the following:
 * http://graphics.stanford.edu/~seander/bithacks.html#RoundUpPowerOf2
 *
 * @ingroup api_bitfield
 */
constexpr uint64_t
alignToPowerOfTwo(uint64_t val)
{
    val--;
    val |= val >> 1;
    val |= val >> 2;
    val |= val >> 4;
    val |= val >> 8;
    val |= val >> 16;
    val |= val >> 32;
    val++;

    return val;
};

/**
 * Count trailing zeros in a 32-bit value.
 *
 * @param An input value
 * @return The number of trailing zeros or 32 if the value is zero.
 *
 * @ingroup api_bitfield
 */
constexpr int
ctz32(uint32_t value)
{
    return value ? __builtin_ctzl(value) : 32;
}

/**
 * Count trailing zeros in a 64-bit value.
 *
 * @param An input value
 * @return The number of trailing zeros or 64 if the value is zero.
 *
 * @ingroup api_bitfield
 */
constexpr int
ctz64(uint64_t value)
{
    return value ? __builtin_ctzll(value) : 64;
}

} // namespace gem5

#endif // __BASE_BITFIELD_HH__
