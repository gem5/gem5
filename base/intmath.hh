/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#ifndef __INTMATH_HH__
#define __INTMATH_HH__

#include <assert.h>

#include "sim/host.hh"

// Returns the prime number one less than n.
int PrevPrime(int n);

// Determine if a number is prime
template <class T>
inline bool
IsPrime(T n)
{
    T i;

    if (n == 2 || n == 3)
        return true;

    // Don't try every odd number to prove if it is a prime.
    // Toggle between every 2nd and 4th number.
    // (This is because every 6th odd number is divisible by 3.)
    for (i = 5; i*i <= n; i += 6) {
        if (((n % i) == 0 ) || ((n % (i + 2)) == 0) ) {
            return false;
        }
    }

    return true;
}

template <class T>
inline T
LeastSigBit(T n)
{
    return n & ~(n - 1);
}

template <class T>
inline bool
IsPowerOf2(T n)
{
    return n != 0 && LeastSigBit(n) == n;
}

inline int
FloorLog2(uint32_t x)
{
    assert(x > 0);

    int y = 0;

    if (x & 0xffff0000) { y += 16; x >>= 16; }
    if (x & 0x0000ff00) { y +=  8; x >>=  8; }
    if (x & 0x000000f0) { y +=  4; x >>=  4; }
    if (x & 0x0000000c) { y +=  2; x >>=  2; }
    if (x & 0x00000002) { y +=  1; }

    return y;
}

inline int
FloorLog2(uint64_t x)
{
    assert(x > 0);

    int y = 0;

    if (x & ULL(0xffffffff00000000)) { y += 32; x >>= 32; }
    if (x & ULL(0x00000000ffff0000)) { y += 16; x >>= 16; }
    if (x & ULL(0x000000000000ff00)) { y +=  8; x >>=  8; }
    if (x & ULL(0x00000000000000f0)) { y +=  4; x >>=  4; }
    if (x & ULL(0x000000000000000c)) { y +=  2; x >>=  2; }
    if (x & ULL(0x0000000000000002)) { y +=  1; }

    return y;
}

inline int
FloorLog2(int32_t x)
{
    assert(x > 0);
    return FloorLog2(x);
}

inline int
FloorLog2(int64_t x)
{
    assert(x > 0);
    return FloorLog2(x);
}

template <class T>
inline int
CeilLog2(T n)
{
    if (n == 1)
        return 0;

    return FloorLog2(n - (T)1) + 1;
}

template <class T>
inline T
FloorPow2(T n)
{
    return (T)1 << FloorLog2(n);
}

template <class T>
inline T
CeilPow2(T n)
{
    return (T)1 << CeilLog2(n);
}

template <class T>
inline T
DivCeil(T a, T b)
{
    return (a + b - 1) / b;
}

template <class T>
inline T
RoundUp(T val, T align)
{
    T mask = align - 1;
    return (val + mask) & ~mask;
}

template <class T>
inline T
RoundDown(T val, T align)
{
    T mask = align - 1;
    return val & ~mask;
}

inline bool
IsHex(char c)
{
    return c >= '0' && c <= '9' ||
        c >= 'A' && c <= 'F' ||
        c >= 'a' && c <= 'f';
}

inline bool
IsOct(char c)
{
    return c >= '0' && c <= '7';
}

inline bool
IsDec(char c)
{
    return c >= '0' && c <= '9';
}

inline int
Hex2Int(char c)
{
  if (c >= '0' && c <= '9')
    return (c - '0');

  if(c >= 'A' && c <= 'F')
    return (c - 'A') + 10;

  if (c >= 'a' && c <= 'f')
    return (c - 'a') + 10;

  return 0;
}

#endif // __INTMATH_HH__
