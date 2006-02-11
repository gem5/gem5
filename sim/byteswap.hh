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
 */

//The purpose of this file is to provide endainness conversion utility
//functions. Depending on the endianness of the guest system, either
//the LittleEndianGuest or BigEndianGuest namespace is used.

#ifndef __SIM_BYTE_SWAP_HH__
#define __SIM_BYTE_SWAP_HH__

#include "sim/host.hh"

// This lets us figure out what the byte order of the host system is
#if defined(linux)
#include <endian.h>
#else
#include <machine/endian.h>
#endif

//These functions actually perform the swapping for parameters
//of various bit lengths
static inline uint64_t
swap_byte64(uint64_t x)
{
    return  (uint64_t)((((uint64_t)(x) & 0xff) << 56) |
            ((uint64_t)(x) & 0xff00ULL) << 40 |
            ((uint64_t)(x) & 0xff0000ULL) << 24 |
            ((uint64_t)(x) & 0xff000000ULL) << 8 |
            ((uint64_t)(x) & 0xff00000000ULL) >> 8 |
            ((uint64_t)(x) & 0xff0000000000ULL) >> 24 |
            ((uint64_t)(x) & 0xff000000000000ULL) >> 40 |
            ((uint64_t)(x) & 0xff00000000000000ULL) >> 56) ;
}

static inline uint32_t
swap_byte32(uint32_t x)
{
    return  (uint32_t)(((uint32_t)(x) & 0xff) << 24 |
            ((uint32_t)(x) & 0xff00) << 8 | ((uint32_t)(x) & 0xff0000) >> 8 |
            ((uint32_t)(x) & 0xff000000) >> 24);

}

static inline uint16_t
swap_byte16(uint16_t x)
{
    return (uint16_t)(((uint16_t)(x) & 0xff) << 8 |
                      ((uint16_t)(x) & 0xff00) >> 8);
}

//This lets the compiler figure out how to call the swap_byte functions above
//for different data types.
static inline uint64_t swap_byte(uint64_t x) {return swap_byte64(x);}
static inline int64_t swap_byte(int64_t x) {return swap_byte64((uint64_t)x);}
static inline uint32_t swap_byte(uint32_t x) {return swap_byte32(x);}
static inline int32_t swap_byte(int32_t x) {return swap_byte32((uint32_t)x);}
static inline int32_t swap_byte(long x) {return swap_byte32((long)x);}
static inline uint16_t swap_byte(uint16_t x) {return swap_byte32(x);}
static inline int16_t swap_byte(int16_t x) {return swap_byte16((uint16_t)x);}
static inline uint8_t swap_byte(uint8_t x) {return x;}
static inline int8_t swap_byte(int8_t x) {return x;}
static inline double swap_byte(double x) {return swap_byte64((uint64_t)x);}
static inline float swap_byte(float x) {return swap_byte32((uint32_t)x);}

//The conversion functions with fixed endianness on both ends don't need to
//be in a namespace
template <typename T> static inline T betole(T value) {return swap_byte(value);}
template <typename T> static inline T letobe(T value) {return swap_byte(value);}

//For conversions not involving the guest system, we can define the functions
//conditionally based on the BYTE_ORDER macro and outside of the namespaces
#if BYTE_ORDER == BIG_ENDIAN
template <typename T> static inline T htole(T value) {return swap_byte(value);}
template <typename T> static inline T letoh(T value) {return swap_byte(value);}
template <typename T> static inline T htobe(T value) {return value;}
template <typename T> static inline T betoh(T value) {return value;}
#elif BYTE_ORDER == LITTLE_ENDIAN
template <typename T> static inline T htole(T value) {return value;}
template <typename T> static inline T letoh(T value) {return value;}
template <typename T> static inline T htobe(T value) {return swap_byte(value);}
template <typename T> static inline T betoh(T value) {return swap_byte(value);}
#else
        #error Invalid Endianess
#endif

namespace BigEndianGuest
{
        template <typename T>
        static inline T gtole(T value) {return betole(value);}
        template <typename T>
        static inline T letog(T value) {return letobe(value);}
        template <typename T>
        static inline T gtobe(T value) {return value;}
        template <typename T>
        static inline T betog(T value) {return value;}
        template <typename T>
        static inline T htog(T value) {return htobe(value);}
        template <typename T>
        static inline T gtoh(T value) {return betoh(value);}
}

namespace LittleEndianGuest
{
        template <typename T>
        static inline T gtole(T value) {return value;}
        template <typename T>
        static inline T letog(T value) {return value;}
        template <typename T>
        static inline T gtobe(T value) {return letobe(value);}
        template <typename T>
        static inline T betog(T value) {return betole(value);}
        template <typename T>
        static inline T htog(T value) {return htole(value);}
        template <typename T>
        static inline T gtoh(T value) {return letoh(value);}
}
#endif // __SIM_BYTE_SWAP_HH__
