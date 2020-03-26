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

#include <inttypes.h>
#include <cassert>
#include <cstddef>
#include <type_traits>

/** Lookup table used for High Speed bit reversing */
extern const uint8_t reverseLookUpTable[];

/**
 * Generate a 64-bit mask of 'nbits' 1s, right justified. If a number of bits
 * greater than 64 is given, it is truncated to 64.
 *
 * @param nbits The number of bits set in the mask.
 */
inline uint64_t
mask(int nbits)
{
    return (nbits >= 64) ? (uint64_t)-1LL : (1ULL << nbits) - 1;
}

/**
 * Extract the bitfield from position 'first' to 'last' (inclusive)
 * from 'val' and right justify it.  MSB is numbered 63, LSB is 0.
 */
template <class T>
inline
T
bits(T val, int first, int last)
{
    int nbits = first - last + 1;
    return (val >> last) & mask(nbits);
}

/**
 * Extract the bit from this position from 'val' and right justify it.
 */
template <class T>
inline
T
bits(T val, int bit)
{
    return bits(val, bit, bit);
}

/**
 * Mask off the given bits in place like bits() but without shifting.
 * msb = 63, lsb = 0
 */
template <class T>
inline
T
mbits(T val, int first, int last)
{
    return val & (mask(first+1) & ~mask(last));
}

inline uint64_t
mask(int first, int last)
{
    return mbits((uint64_t)-1LL, first, last);
}

/**
 * Sign-extend an N-bit value to 64 bits.
 */
template <int N>
inline
uint64_t
sext(uint64_t val)
{
    int sign_bit = bits(val, N-1, N-1);
    return sign_bit ? (val | ~mask(N)) : val;
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
 */
template <class T, class B>
inline
T
insertBits(T val, int first, int last, B bit_val)
{
    T t_bit_val = bit_val;
    T bmask = mask(first - last + 1) << last;
    return ((t_bit_val << last) & bmask) | (val & ~bmask);
}

/**
 * Overloaded for access to only one bit in value
 */
template <class T, class B>
inline
T
insertBits(T val, int bit, B bit_val)
{
    return insertBits(val, bit, bit, bit_val);
}

/**
 * A convenience function to replace bits first to last of val with bit_val
 * in place.
 */
template <class T, class B>
inline
void
replaceBits(T& val, int first, int last, B bit_val)
{
    val = insertBits(val, first, last, bit_val);
}

/** Overloaded function to allow to access only 1 bit*/
template <class T, class B>
inline
void
replaceBits(T& val, int bit, B bit_val)
{
    val = insertBits(val, bit, bit, bit_val);
}

/**
 * Takes a variable lenght word and returns the mirrored version
 * (Bit by bit, LSB=>MSB).
 *
 * algorithm from
 * http://graphics.stanford.edu/~seander/bithacks.html
 * #ReverseBitsByLookupTable
 *
 * @param val: variable lenght word
 * @param size: number of bytes to mirror
 * @return mirrored word
 */
template <class T>
T
reverseBits(T val, std::size_t size = sizeof(T))
{
    static_assert(std::is_integral<T>::value, "Expecting an integer type");

    assert(size <= sizeof(T));

    T output = 0;
    for (auto byte = 0; byte < size; byte++, val = static_cast<T>(val >> 8)) {
        output = (output << 8) | reverseLookUpTable[val & 0xFF];
    }

    return output;
}

/**
 * Returns the bit position of the MSB that is set in the input
 */
inline
int
findMsbSet(uint64_t val) {
    int msb = 0;
    if (!val)
        return 0;
    if (bits(val, 63,32)) { msb += 32; val >>= 32; }
    if (bits(val, 31,16)) { msb += 16; val >>= 16; }
    if (bits(val, 15,8))  { msb += 8;  val >>= 8;  }
    if (bits(val, 7,4))   { msb += 4;  val >>= 4;  }
    if (bits(val, 3,2))   { msb += 2;  val >>= 2;  }
    if (bits(val, 1,1))   { msb += 1; }
    return msb;
}

/**
 * Returns the bit position of the LSB that is set in the input
 */
inline int
findLsbSet(uint64_t val) {
    int lsb = 0;
    if (!val)
        return sizeof(val) * 8;
    if (!bits(val, 31,0)) { lsb += 32; val >>= 32; }
    if (!bits(val, 15,0)) { lsb += 16; val >>= 16; }
    if (!bits(val, 7,0))  { lsb += 8;  val >>= 8;  }
    if (!bits(val, 3,0))  { lsb += 4;  val >>= 4;  }
    if (!bits(val, 1,0))  { lsb += 2;  val >>= 2;  }
    if (!bits(val, 0,0))  { lsb += 1; }
    return lsb;
}

/**
 * Checks if a number is a power of two, or zero.
 */
template <class T>
inline bool
isPow2(T v) {
   return (v & (v - 1)) == (T)0;
}

/**
 * Returns the number of set ones in the provided value.
 * PD algorithm from
 * http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
 */
inline int
popCount(uint64_t val) {
#ifndef __has_builtin
    #define __has_builtin(foo) 0
#endif
#if defined(__GNUC__) || (defined(__clang__) && __has_builtin(__builtin_popcountl))
    return __builtin_popcountl(val);
#else
    const uint64_t m1 = 0x5555555555555555;  // ..010101b
    const uint64_t m2 = 0x3333333333333333;  // ..110011b
    const uint64_t m4 = 0x0f0f0f0f0f0f0f0f;  // ..001111b
    const uint64_t sum = 0x0101010101010101;

    val -= (val >> 1) & m1;               // 2 bits count -> 2 bits
    val = (val & m2) + ((val >> 2) & m2); // 4 bits count -> 4 bits
    val = (val + (val >> 4)) & m4;        // 8 bits count -> 8 bits
    return (val * sum) >> 56;             // horizontal sum
#endif // defined(__GNUC__) || (defined(__clang__) && __has_builtin(__builtin_popcountl))
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
 */
inline uint64_t alignToPowerOfTwo(uint64_t val)
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
 */
inline int ctz32(uint32_t value)
{
    return value ? __builtin_ctzl(value) : 32;
}

/**
 * Count trailing zeros in a 64-bit value.
 *
 * @param An input value
 * @return The number of trailing zeros or 64 if the value is zero.
 */
inline int ctz64(uint64_t value)
{
    return value ? __builtin_ctzll(value) : 64;
}

#endif // __BASE_BITFIELD_HH__
