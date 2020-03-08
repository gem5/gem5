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
 */

#ifndef __BASE_CONDCODE_HH__
#define __BASE_CONDCODE_HH__

#include "base/bitfield.hh"

/**
 * Calculate the carry flag from an addition. This should work even when
 * a carry value is also added in.
 *
 * Parameters:
 *   dest: The result value of the addition.
 *   src1: One of the addends that was added.
 *   src2: The other addend that was added in.
 *
 * Rationale:
 *   This code analyzes the most sig. bits of the source addends and result,
 *   and deduces the carry out flag from them without needing the carry in bit.
 *
 *   Observe that we have four cases after an addition regarding the carry
 *   in and carry out bits:
 *
 *   If we have no carry in but a carry out:
 *     src1 and src2 must both be 1, with the result bit being 0. Hence,
 *     ~0 + 1 + 1 => 11, which has a high second bit. We return true.
 *
 *   If we have a carry in and a carry out:
 *     src1 and src2 can either be 1 and 0, or vice versa. In this case,
 *     the addition with the carry in gives a result bit of 0 but a carry out.
 *     Hence,
 *     ~0 + 1 + 0 => 10, or ~0 + 0 + 1 => 10. We return true.
 *
 *     Or, src1 and src2 can both be one. Along with the carry, this gives
 *     a result of 1 and a carry out of 1. Hence,
 *     ~1 + 1 + 1 => 10. We return true.
 *
 *   If we have no carry in and no carry out:
 *     src1 and src2 can either be 1 and 0, 0 and 1, or 0 and 0.
 *     In the first two cases the result bit is 1, which when negated does not
 *     contribute to the sum algorithm at all. In the last case the result bit
 *     is zero, but neither src1 nor src2 contribute to the sum either. Hence,
 *     ~1 + 1 + 0 => 1,
 *     ~1 + 0 + 1 => 1,
 *     ~0 + 0 + 0 => 1.
 *     So we return false for all of these cases.
 *
 *   If we have a carry in, but no carry out:
 *     src1 and src2 can neither be 1. So the overall result bit is 1. Hence:
 *     ~1 + 0 + 0 => 0. We return false.
 */
static inline bool
findCarry(int width, uint64_t dest, uint64_t src1, uint64_t src2)
{
    int shift = width - 1;
    return ((~(dest >> shift) & 1) +
            ((src1 >> shift) & 1) +
            ((src2 >> shift) & 1)) & 0x2;
}

/**
 * Calculate the overflow flag from an addition.
 */
static inline bool
findOverflow(int width, uint64_t dest, uint64_t src1, uint64_t src2)
{
    int shift = width - 1;
    return ((src1 ^ ~src2) & (src1 ^ dest)) & (1ULL << shift);
}

/**
 * Calculate the parity of a value. 1 is for odd parity and 0 is for even.
 *
 * Parameters:
 *   dest: a value to be tested.
 *
 * Rationale:
 *   findParity simply performs bitwise XOR operations on each "pair" of bits
 *   in the dest parameter; the procedure being that a pair of ones will be
 *   XOR'ed out of the intermediate value.
 *
 *   This process is repeated until one last pair of bits are XOR'ed together.
 *   If the intermediate is still one, then there is exactly one high bit
 *   which does not have a corresponding high bit. Therefore, the value must
 *   have odd parity, and we return 1 accordingly. Otherwise we return 0.
 */
static inline bool
findParity(int width, uint64_t dest)
{
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
static inline bool
findNegative(int width, uint64_t dest)
{
    return bits(dest, width - 1);
}

/**
 * Calculate the zero flag.
 */
static inline bool
findZero(int width, uint64_t dest)
{
    return !(dest & mask(width));
}

#endif // __BASE_CONDCODE_HH__
