/*
 * Copyright (c) 2026
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
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
 * Returns cache_line when pack_width is 0 or the op is not packable,
 * otherwise min(pack_width, cache_line).
 */
inline unsigned
vecMemSplitGrain(unsigned pack_width, unsigned cache_line, OpClass op)
{
    if (pack_width == 0 || cache_line == 0 || !isVecMemPackable(op)) {
        return cache_line;
    }
    return pack_width < cache_line ? pack_width : cache_line;
}

} // namespace o3
} // namespace gem5

#endif // __CPU_O3_VEC_MEM_PACK_HH__
