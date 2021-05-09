/*
 * Copyright (c) 2012-2013 ARM Limited
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

/// @file
/// Utility functions and datatypes used by AArch64 NEON memory instructions.

#ifndef __ARCH_ARM_INSTS_NEON64_MEM_HH__
#define __ARCH_ARM_INSTS_NEON64_MEM_HH__

#include <cassert>
#include <cstdint>

namespace gem5
{

namespace ArmISA
{

typedef uint64_t XReg;

/// 128-bit NEON vector register.
struct VReg
{
    XReg hi;
    XReg lo;
};

/// Write a single NEON vector element leaving the others untouched.
inline void
writeVecElem(VReg *dest, XReg src, int index, int eSize)
{
    // eSize must be less than 4:
    // 0 -> 8-bit elems,
    // 1 -> 16-bit elems,
    // 2 -> 32-bit elems,
    // 3 -> 64-bit elems
    assert(eSize <= 3);

    int eBits = 8 << eSize;
    int lsbPos = index * eBits;
    assert(lsbPos < 128);
    int shiftAmt = lsbPos % 64;

    XReg maskBits = -1;
    if (eBits == 64) {
        maskBits = 0;
    } else {
        maskBits = maskBits << eBits;
    }
    maskBits = ~maskBits;

    XReg sMask = maskBits;
    maskBits = sMask << shiftAmt;

    if (lsbPos < 64) {
        dest->lo = (dest->lo & (~maskBits)) | ((src & sMask) << shiftAmt);
    } else {
        dest->hi = (dest->hi & (~maskBits)) | ((src & sMask) << shiftAmt);
    }
}

/// Read a single NEON vector element.
inline XReg
readVecElem(VReg src, int index, int eSize)
{
    // eSize must be less than 4:
    // 0 -> 8-bit elems,
    // 1 -> 16-bit elems,
    // 2 -> 32-bit elems,
    // 3 -> 64-bit elems
    assert(eSize <= 3);

    XReg data;

    int eBits = 8 << eSize;
    int lsbPos = index * eBits;
    assert(lsbPos < 128);
    int shiftAmt = lsbPos % 64;

    XReg maskBits = -1;
    if (eBits == 64) {
        maskBits = 0;
    } else {
        maskBits = maskBits << eBits;
    }
    maskBits = ~maskBits;

    if (lsbPos < 64) {
        data = (src.lo >> shiftAmt) & maskBits;
    } else {
        data = (src.hi >> shiftAmt) & maskBits;
    }
    return data;
}

} // namespace ArmISA
} // namespace gem5

#endif  // __ARCH_ARM_INSTS_NEON64_MEM_HH__
