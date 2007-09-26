/*
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
 *
 * Authors: Gabe Black
 */

#ifndef __BASE_CONDCODE_HH__
#define __BASE_CONDCODE_HH__

#include "base/bitfield.hh"
#include "base/trace.hh"

/**
 * Calculate the carry flag from an addition. This should work even when
 * a carry value is also added in.
 */
inline
bool
findCarry(int width, uint64_t dest, uint64_t src1, uint64_t src2) {
    int shift = width - 1;
    return ((~(dest >> shift) & 1) +
            ((src1 >> shift) & 1) +
            ((src2 >> shift) & 1)) & 0x2;
}

/**
 * Calculate the overflow flag from an addition.
 */
inline
bool
findOverflow(int width, uint64_t dest, uint64_t src1, uint64_t src2) {
    int shift = width - 1;
    return ((src1 ^ ~src2) & (src1 ^ dest)) & (1ULL << shift);
}

/**
 * Calculate the parity of a value. 1 is for odd parity and 0 is for even.
 */
inline
bool
findParity(int width, uint64_t dest) {
    dest &= mask(width);
    dest ^= (dest >> 32);
    dest ^= (dest >> 16);
    dest ^= (dest >> 8);
    dest ^= (dest >> 4);
    dest ^= (dest >> 2);
    dest ^= (dest >> 1);
    return dest & 1;
}

/**
 * Calculate the negative flag.
 */
inline
bool
findNegative(int width, uint64_t dest) {
    return bits(dest, width - 1, width - 1);
}

/**
 * Calculate the zero flag.
 */
inline
bool
findZero(int width, uint64_t dest) {
    return !(dest & mask(width));
}

#endif // __BASE_CONDCODE_HH__
