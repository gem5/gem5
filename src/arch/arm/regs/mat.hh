/*
 * Copyright (c) 2022 Arm Limited
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

#ifndef __ARCH_ARM_REGS_MAT_HH__
#define __ARCH_ARM_REGS_MAT_HH__

#include "arch/arm/types.hh"
#include "arch/arm/matrix.hh"
#include "cpu/reg_class.hh"
#include "debug/MatRegs.hh"

namespace gem5
{

namespace ArmISA
{

/*
 * We do the same as is done for vector registers when creating the
 * matricies. One of the things to note is that this allocates the
 * largest architecturally possible matrix - this is a bit inefficient
 * from a memory point of view, but at this point we do not know which
 * vector length will be chosen (and this can potentially vary during
 * runtime).
 */
using MatRegContainer =
    gem5::MatStore<MaxSmeVecLenInBytes, MaxSmeVecLenInBytes>;

template <typename ElemType>
using MatTile = gem5::Tile<ElemType, MatRegContainer>;

template <typename ElemType>
using MatTileRow = gem5::HorizontalSlice<ElemType, MatRegContainer, true>;

template <typename ElemType>
using MatTileCol = gem5::VerticalSlice<ElemType, MatRegContainer, true>;

template <typename ElemType>
using MatRow = gem5::HorizontalSlice<ElemType, MatRegContainer, false>;

template <typename ElemType>
using MatCol = gem5::VerticalSlice<ElemType, MatRegContainer, false>;

// SME ZA tile, i.e. matrix
const int NumMatrixRegs = 1;

static inline TypedRegClassOps<ArmISA::MatRegContainer> matRegClassOps;

inline constexpr RegClass matRegClass =
    RegClass(MatRegClass, MatRegClassName, NumMatrixRegs, debug::MatRegs)
        .ops(matRegClassOps)
        .regType<MatRegContainer>();

/*
 * Helpers for providing access to the different views of a matrix
 * register. Intended to be called from the instruction implementations
 * themselves.
 */
template <typename ElemType>
MatTile<ElemType>
getTile(MatRegContainer &reg, uint8_t tile_idx)
{
    return reg.asTile<ElemType>(tile_idx);
}

template <typename ElemType>
MatTileRow<ElemType>
getTileHSlice(MatRegContainer &reg, uint8_t tile_idx, uint8_t row_idx)
{
    return reg.asTile<ElemType>(tile_idx).asHSlice(row_idx);
}

template <typename ElemType>
MatTileCol<ElemType>
getTileVSlice(MatRegContainer &reg, uint8_t tile_idx, uint8_t col_idx)
{
    return reg.asTile<ElemType>(tile_idx).asVSlice(col_idx);
}

template <typename ElemType>
MatRow<ElemType>
getHSlice(MatRegContainer &reg, uint8_t row_idx)
{
    return reg.asHSlice<ElemType>(row_idx);
}

template <typename ElemType>
MatCol<ElemType>
getVSlice(MatRegContainer &reg, uint8_t col_idx)
{
    return reg.asVSlice<ElemType>(col_idx);
}

} // namespace ArmISA
} // namespace gem5

#endif
