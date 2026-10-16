/*
 * Copyright (c) 2026 Mao Weiming
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

#ifndef __CPU_O3_VEC_MEM_PACK_HH__
#define __CPU_O3_VEC_MEM_PACK_HH__

#include "cpu/op_class.hh"

namespace gem5
{
namespace o3
{

/** Contiguous unit-stride / mask / whole-register vector memops. */
inline bool
isVecMemPackable(OpClass op)
{
    switch (op) {
      case SimdUnitStrideLoadOp:
      case SimdUnitStrideStoreOp:
      case SimdUnitStrideMaskLoadOp:
      case SimdUnitStrideMaskStoreOp:
      case SimdWholeRegisterLoadOp:
      case SimdWholeRegisterStoreOp:
        return true;
      default:
        return false;
    }
}

/**
 * Split grain for SplitDataRequest / transferNeedsBurst.
 * Non-packable ops use cache_line; packable ops use min(pack_width, cache_line).
 * pack_width is the resolved grain (LSQ maps an unspecified param to the
 * cache-line size); 0 is not a default sentinel.
 */
inline unsigned
vecMemSplitGrain(unsigned pack_width, unsigned cache_line, OpClass op)
{
    if (!isVecMemPackable(op)) {
        return cache_line;
    }
    return pack_width < cache_line ? pack_width : cache_line;
}

} // namespace o3
} // namespace gem5

#endif // __CPU_O3_VEC_MEM_PACK_HH__
