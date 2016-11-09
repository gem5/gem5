/*
 * Copyright (c) 2004 The Regents of The University of Michigan
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
 *
 * Authors: Gabe Black
 *          Ali Saidi
 *          Nathan Binkert
 */

//The purpose of this file is to provide endainness conversion utility
//functions. Depending on the endianness of the guest system, either
//the LittleEndianGuest or BigEndianGuest namespace is used.

#ifndef __SIM_BYTE_SWAP_HH__
#define __SIM_BYTE_SWAP_HH__

#include "base/bigint.hh"
#include "base/types.hh"

// This lets us figure out what the byte order of the host system is
#if defined(__linux__)
#include <endian.h>
// If this is a linux system, lets used the optimized definitions if they exist.
// If one doesn't exist, we pretty much get what is listed below, so it all
// works out
#include <byteswap.h>
#elif defined (__sun)
#include <sys/isa_defs.h>
#else
#include <machine/endian.h>
#endif

#if defined(__APPLE__)
#include <libkern/OSByteOrder.h>
#endif

//These functions actually perform the swapping for parameters
//of various bit lengths
inline uint64_t
swap_byte64(uint64_t x)
{
#if defined(__linux__)
    return bswap_64(x);
#elif defined(__APPLE__)
    return OSSwapInt64(x);
#else
    return  (uint64_t)((((uint64_t)(x) & 0xff) << 56) |
            ((uint64_t)(x) & 0xff00ULL) << 40 |
            ((uint64_t)(x) & 0xff0000ULL) << 24 |
            ((uint64_t)(x) & 0xff000000ULL) << 8 |
            ((uint64_t)(x) & 0xff00000000ULL) >> 8 |
            ((uint64_t)(x) & 0xff0000000000ULL) >> 24 |
            ((uint64_t)(x) & 0xff000000000000ULL) >> 40 |
            ((uint64_t)(x) & 0xff00000000000000ULL) >> 56) ;
#endif
}

inline uint32_t
swap_byte32(uint32_t x)
{
#if defined(__linux__)
    return bswap_32(x);
#elif defined(__APPLE__)
    return OSSwapInt32(x);
#else
    return  (uint32_t)(((uint32_t)(x) & 0xff) << 24 |
            ((uint32_t)(x) & 0xff00) << 8 | ((uint32_t)(x) & 0xff0000) >> 8 |
            ((uint32_t)(x) & 0xff000000) >> 24);
#endif
}

inline uint16_t
swap_byte16(uint16_t x)
{
#if defined(__linux__)
    return bswap_16(x);
#elif defined(__APPLE__)
    return OSSwapInt16(x);
#else
    return (uint16_t)(((uint16_t)(x) & 0xff) << 8 |
                      ((uint16_t)(x) & 0xff00) >> 8);
#endif
}

// This function lets the compiler figure out how to call the
// swap_byte functions above for different data types.  Since the
// sizeof() values are known at compile time, it should inline to a
// direct call to the right swap_byteNN() function.
template <typename T>
inline T swap_byte(T x) {
    if (sizeof(T) == 8)
        return swap_byte64((uint64_t)x);
    else if (sizeof(T) == 4)
        return swap_byte32((uint32_t)x);
    else if (sizeof(T) == 2)
        return swap_byte16((uint16_t)x);
    else if (sizeof(T) == 1)
        return x;
    else
        panic("Can't byte-swap values larger than 64 bits");
}

template<>
inline Twin64_t swap_byte<Twin64_t>(Twin64_t x)
{
    x.a = swap_byte(x.a);
    x.b = swap_byte(x.b);
    return x;
}

template<>
inline Twin32_t swap_byte<Twin32_t>(Twin32_t x)
{
    x.a = swap_byte(x.a);
    x.b = swap_byte(x.b);
    return x;
}

//The conversion functions with fixed endianness on both ends don't need to
//be in a namespace
template <typename T> inline T betole(T value) {return swap_byte(value);}
template <typename T> inline T letobe(T value) {return swap_byte(value);}

//For conversions not involving the guest system, we can define the functions
//conditionally based on the BYTE_ORDER macro and outside of the namespaces
#if (defined(_BIG_ENDIAN) || !defined(_LITTLE_ENDIAN)) && BYTE_ORDER == BIG_ENDIAN
const ByteOrder HostByteOrder = BigEndianByteOrder;
template <typename T> inline T htole(T value) {return swap_byte(value);}
template <typename T> inline T letoh(T value) {return swap_byte(value);}
template <typename T> inline T htobe(T value) {return value;}
template <typename T> inline T betoh(T value) {return value;}
#elif defined(_LITTLE_ENDIAN) || BYTE_ORDER == LITTLE_ENDIAN
const ByteOrder HostByteOrder = LittleEndianByteOrder;
template <typename T> inline T htole(T value) {return value;}
template <typename T> inline T letoh(T value) {return value;}
template <typename T> inline T htobe(T value) {return swap_byte(value);}
template <typename T> inline T betoh(T value) {return swap_byte(value);}
#else
        #error Invalid Endianess
#endif

namespace BigEndianGuest
{
    const ByteOrder GuestByteOrder = BigEndianByteOrder;
    template <typename T>
    inline T gtole(T value) {return betole(value);}
    template <typename T>
    inline T letog(T value) {return letobe(value);}
    template <typename T>
    inline T gtobe(T value) {return value;}
    template <typename T>
    inline T betog(T value) {return value;}
    template <typename T>
    inline T htog(T value) {return htobe(value);}
    template <typename T>
    inline T gtoh(T value) {return betoh(value);}
}

namespace LittleEndianGuest
{
    const ByteOrder GuestByteOrder = LittleEndianByteOrder;
    template <typename T>
    inline T gtole(T value) {return value;}
    template <typename T>
    inline T letog(T value) {return value;}
    template <typename T>
    inline T gtobe(T value) {return letobe(value);}
    template <typename T>
    inline T betog(T value) {return betole(value);}
    template <typename T>
    inline T htog(T value) {return htole(value);}
    template <typename T>
    inline T gtoh(T value) {return letoh(value);}
}
#endif // __SIM_BYTE_SWAP_HH__
