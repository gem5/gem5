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

#include <gtest/gtest.h>

#include "cpu/o3/vec_mem_pack.hh"

using gem5::o3::isVecMemPackable;
using gem5::o3::vecMemSplitGrain;
using gem5::SimdIndexedLoadOp;
using gem5::SimdIndexedStoreOp;
using gem5::SimdStridedLoadOp;
using gem5::SimdUnitStrideFaultOnlyFirstLoadOp;
using gem5::SimdUnitStrideLoadOp;
using gem5::SimdUnitStrideMaskLoadOp;
using gem5::SimdUnitStrideMaskStoreOp;
using gem5::SimdUnitStrideSegmentedLoadOp;
using gem5::SimdUnitStrideStoreOp;
using gem5::SimdWholeRegisterLoadOp;
using gem5::SimdWholeRegisterStoreOp;

TEST(VecMemPack, PackableOps)
{
    EXPECT_TRUE(isVecMemPackable(SimdUnitStrideLoadOp));
    EXPECT_TRUE(isVecMemPackable(SimdUnitStrideStoreOp));
    EXPECT_TRUE(isVecMemPackable(SimdUnitStrideMaskLoadOp));
    EXPECT_TRUE(isVecMemPackable(SimdUnitStrideMaskStoreOp));
    EXPECT_TRUE(isVecMemPackable(SimdWholeRegisterLoadOp));
    EXPECT_TRUE(isVecMemPackable(SimdWholeRegisterStoreOp));
    EXPECT_FALSE(isVecMemPackable(SimdUnitStrideFaultOnlyFirstLoadOp));
    EXPECT_FALSE(isVecMemPackable(SimdUnitStrideSegmentedLoadOp));
    EXPECT_FALSE(isVecMemPackable(SimdIndexedLoadOp));
    EXPECT_FALSE(isVecMemPackable(SimdIndexedStoreOp));
    EXPECT_FALSE(isVecMemPackable(SimdStridedLoadOp));
}

TEST(VecMemPack, SplitGrain)
{
    EXPECT_EQ(vecMemSplitGrain(16, 64, SimdIndexedLoadOp), 64u);
    EXPECT_EQ(vecMemSplitGrain(16, 64, SimdUnitStrideLoadOp), 16u);
    EXPECT_EQ(vecMemSplitGrain(16, 16, SimdUnitStrideLoadOp), 16u);
    EXPECT_EQ(vecMemSplitGrain(64, 64, SimdUnitStrideLoadOp), 64u);
    EXPECT_EQ(vecMemSplitGrain(128, 64, SimdUnitStrideLoadOp), 64u);
    EXPECT_EQ(vecMemSplitGrain(16, 0, SimdUnitStrideLoadOp), 0u);
}
