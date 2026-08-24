/*
 * Copyright (c) 2026
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
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
    EXPECT_EQ(vecMemSplitGrain(0, 64, SimdUnitStrideLoadOp), 64u);
    EXPECT_EQ(vecMemSplitGrain(16, 64, SimdIndexedLoadOp), 64u);
    EXPECT_EQ(vecMemSplitGrain(16, 64, SimdUnitStrideLoadOp), 16u);
    EXPECT_EQ(vecMemSplitGrain(16, 16, SimdUnitStrideLoadOp), 16u);
    EXPECT_EQ(vecMemSplitGrain(128, 64, SimdUnitStrideLoadOp), 64u);
    EXPECT_EQ(vecMemSplitGrain(16, 0, SimdUnitStrideLoadOp), 0u);
}
