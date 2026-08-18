/*
 * Copyright (c) 2026 Arm Limited
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
 * Copyright (c) 2020 Daniel R. Carvalho
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

#include <cstdint>
#include <limits>

#include "base/flags.hh"

using namespace gem5;

/** Test default zero-initialized constructor. */
TEST(FlagsTest, ConstructorZero)
{
    const Flags<uint32_t> flags;
    ASSERT_EQ(uint32_t(0), uint32_t(flags));
}

/** Test constructor with a single-bit initial value. */
TEST(FlagsTest, ConstructorSingle)
{
    const uint32_t value = (1 << 3);
    const Flags<uint32_t> flags(value);
    ASSERT_EQ(value, uint32_t(flags));
}

/** Test constructor with an initial multi-bit value. */
TEST(FlagsTest, ConstructorMulti)
{
    const uint32_t value = (1 << 3) | (1 << 5) | (1 << 9);
    const Flags<uint32_t> flags(value);
    ASSERT_EQ(value, uint32_t(flags));
}

/** Test assignment of variable of underlying type. */
TEST(FlagsTest, TypeAssignment)
{
    const uint32_t value = (1 << 3) | (1 << 5) | (1 << 9);
    Flags<uint32_t> flags;
    flags = value;
    ASSERT_EQ(value, uint32_t(flags));
}

/**
 * Test assignment of variable of underlying type, overwriting an initial
 * value.
 */
TEST(FlagsTest, TypeAssignmentOverwrite)
{
    const uint32_t init_value = (1 << 5) | (1 << 6) ;
    const uint32_t value = (1 << 3) | (1 << 5) | (1 << 9);
    Flags<uint32_t> flags(init_value);
    flags = value;
    ASSERT_EQ(value, uint32_t(flags));
}

/** Test assignment of other Flags. */
TEST(FlagsTest, FlagsAssignment)
{
    const uint32_t value = (1 << 3) | (1 << 5) | (1 << 9);
    Flags<uint32_t> flags_a;
    Flags<uint32_t> flags_b(value);
    flags_a = flags_b;
    ASSERT_EQ(uint32_t(flags_a), uint32_t(flags_b));
}

/** Test assignment of other Flags, overwriting an initial value. */
TEST(FlagsTest, FlagsAssignmentOverwrite)
{
    const uint32_t init_value = (1 << 5) | (1 << 6);
    const uint32_t value = (1 << 3) | (1 << 5) | (1 << 9);
    Flags<uint32_t> flags_a(init_value);
    Flags<uint32_t> flags_b(value);
    flags_a = flags_b;
    ASSERT_EQ(uint32_t(flags_a), uint32_t(flags_b));
}

/** Test isSet for multiple bits set. */
TEST(FlagsTest, IsSetValue)
{
    const uint32_t value_a = (1 << 3);
    const uint32_t value_b = (1 << 5);
    const Flags<uint32_t> flags(value_a | value_b);
    ASSERT_TRUE(flags.isSet(value_a));
    ASSERT_FALSE(flags.isSet(value_a << 1));
    ASSERT_TRUE(flags.isSet(value_b));
}

/** Test isSet comparing against another flag. */
TEST(FlagsTest, IsSetType)
{
    const uint32_t value_a = (1 << 5) | (1 << 6);
    const uint32_t value_b = (1 << 3) | (1 << 5) | (1 << 9);
    const uint32_t value_c = (1 << 4) | (1 << 8);
    const Flags<uint32_t> flags(value_a);
    ASSERT_TRUE(flags.isSet(value_b));
    ASSERT_FALSE(flags.isSet(value_c));
}

/** Test allSet comparing against another flag. */
TEST(FlagsTest, AllSetMatch)
{
    const uint32_t value_a = (1 << 5) | (1 << 6);
    const uint32_t value_b = (1 << 3) | (1 << 5) | (1 << 9);
    const Flags<uint32_t> flags(value_a);
    ASSERT_TRUE(flags.allSet(value_a));
    ASSERT_FALSE(flags.allSet(value_b));
}

/** Test noneSet comparing against another flag. */
TEST(FlagsTest, NoneSetMatch)
{
    const uint32_t value_a = (1 << 5) | (1 << 6);
    const uint32_t value_b = (1 << 3) | (1 << 6);
    const uint32_t value_c = (1 << 3) | (1 << 4) | (1 << 9);
    const Flags<uint32_t> flags(value_a);
    ASSERT_FALSE(flags.noneSet(value_a));
    ASSERT_FALSE(flags.noneSet(value_b));
    ASSERT_TRUE(flags.noneSet(value_c));
}

/** Test if no bits are set after a full clear. */
TEST(FlagsTest, Clear)
{
    const uint32_t value = (1 << 5) | (1 << 6);
    Flags<uint32_t> flags(value);
    flags.clear();
    ASSERT_EQ(0, uint32_t(flags));
}

/** Test clearing specific bits. */
TEST(FlagsTest, ClearMatch)
{
    const uint32_t value_a = (1 << 5) | (1 << 6);
    const uint32_t value_b = (1 << 3) | (1 << 5) | (1 << 9);
    Flags<uint32_t> flags(value_a);
    flags.clear(value_b);
    ASSERT_FALSE(flags.isSet(value_a & value_b));
    ASSERT_TRUE(flags.isSet(value_a ^ (value_a & value_b)));
}

/** Test setting with a few overlapping bits. */
TEST(FlagsTest, SetOverlapping)
{
    const uint32_t value_a = (1 << 5) | (1 << 6);
    const uint32_t value_b = (1 << 3) | (1 << 5) | (1 << 9);
    Flags<uint32_t> flags(value_a);
    flags.set(value_b);
    ASSERT_EQ(value_a | value_b, uint32_t(flags));
}

/**
 * Test conditional set. If true the selected bits are set; otherwise, they
 * are cleared.
 */
TEST(FlagsTest, ConditionalSet)
{
    const uint32_t value_a = (1 << 5) | (1 << 6);
    const uint32_t value_b = (1 << 3) | (1 << 5) | (1 << 9);

    Flags<uint32_t> flags_true(value_a);
    flags_true.set(value_b, true);
    ASSERT_EQ(value_a | value_b, uint32_t(flags_true));

    Flags<uint32_t> flags_false(value_a);
    flags_false.set(value_b, false);
    ASSERT_EQ(value_a & ~value_b, uint32_t(flags_false));
}

/**
 * Test replacing a masked selection of bits. This means that bits of the
 * original value that match the mask will be replaced by the bits of
 * the new value that match the mask.
 */
TEST(FlagsTest, ReplaceOverlapping)
{
    const uint32_t value_a = (1 << 4) | (1 << 5) | (1 << 6);
    const uint32_t value_b = (1 << 3) | (1 << 5) | (1 << 9);
    const uint32_t mask = (1 << 4) | (1 << 5) | (1 << 9) | (1 << 10);
    // (1 << 4) is set in value_a, but is not set in value_b, so it is cleared
    // (1 << 5) is set in both values, so it remains set
    // (1 << 9) is not set in value_a, but it is in value_b, so it is set
    // (1 << 10) is not set in both values, so it remains not set
    const uint32_t result = (1 << 5) | (1 << 6) | (1 << 9);
    Flags<uint32_t> flags(value_a);
    flags.replace(value_b, mask);
    ASSERT_EQ(result, uint32_t(flags));
}

enum class TestFlag : uint8_t
{
    FlagA,
    FlagB,
    FlagC,
    NumFlags
};

using TestFlags =
    BitsetFlag<TestFlag, static_cast<size_t>(TestFlag::NumFlags)>;

/** Test default zero-initialized constructor for bitset-backed flags. */
TEST(BitsetFlagTest, ConstructorZero)
{
    const TestFlags flags;
    ASSERT_TRUE(flags.noneSet(TestFlag::FlagA));
}

/** Test construction and queries with scoped enum flags. */
TEST(BitsetFlagTest, ScopedEnum)
{
    TestFlags flags(TestFlag::FlagA);
    flags.set(TestFlag::FlagC);

    ASSERT_TRUE(flags.isSet(TestFlag::FlagA));
    ASSERT_FALSE(flags.isSet(TestFlag::FlagB));
    ASSERT_TRUE(flags.isSet(TestFlag::FlagC));
}

/** Test operator composition with scoped enum flags. */
TEST(BitsetFlagTest, OperatorOr)
{
    const TestFlags flags = TestFlags(TestFlag::FlagA) | TestFlag::FlagB;
    ASSERT_TRUE(flags.isSet(TestFlag::FlagA));
    ASSERT_TRUE(flags.isSet(TestFlag::FlagB));
    ASSERT_FALSE(flags.isSet(TestFlag::FlagC));
}

/** Test allSet when the rhs is a BitsetFlag itself */
TEST(BitsetFlagTest, BitsetAllSet)
{
    const TestFlags flags = TestFlags(TestFlag::FlagA) | TestFlag::FlagB;
    const TestFlags compare_flags =
        TestFlags(TestFlag::FlagA) | TestFlag::FlagB;

    ASSERT_TRUE(flags.allSet(compare_flags));
}

/** Test isSet when the rhs is a BitsetFlag itself */
TEST(BitsetFlagTest, BitsetIsSet)
{
    const TestFlags flags = TestFlags(TestFlag::FlagA) | TestFlag::FlagB;
    const TestFlags compare_flags =
        TestFlags(TestFlag::FlagC) | TestFlag::FlagB;

    ASSERT_TRUE(flags.isSet(compare_flags));
}

/** Test clearing all bits except a selected flag set. */
TEST(BitsetFlagTest, ClearAllExcept)
{
    TestFlags flags =
        TestFlags(TestFlag::FlagA) | TestFlag::FlagB | TestFlag::FlagC;
    flags.clearAllExcept(TestFlags(TestFlag::FlagB));
    ASSERT_TRUE(flags.noneSet(TestFlag::FlagA));
    ASSERT_TRUE(flags.isSet(TestFlag::FlagB));
    ASSERT_TRUE(flags.noneSet(TestFlag::FlagC));
}

/** Test replacing a masked selection of flags. */
TEST(BitsetFlagTest, Replace)
{
    TestFlags flags = TestFlags(TestFlag::FlagA) | TestFlag::FlagB;
    const TestFlags replacement = TestFlags(TestFlag::FlagC);
    const TestFlags mask = TestFlags(TestFlag::FlagA) | TestFlag::FlagC;

    flags.replace(replacement, mask);

    ASSERT_TRUE(flags.noneSet(TestFlag::FlagA));
    ASSERT_TRUE(flags.isSet(TestFlag::FlagB));
    ASSERT_TRUE(flags.isSet(TestFlag::FlagC));
}
