/*
 * Copyright (c) 2019 The Regents of the University of California
 * All rights reserved
 *
 * Redistribution and use in source and binary forms,  with or without
 * modification,  are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice,  this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice,  this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,  INCLUDING,  BUT NOT
 * LIMITED TO,  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,  INDIRECT,  INCIDENTAL,
 * SPECIAL,  EXEMPLARY,  OR CONSEQUENTIAL DAMAGES (INCLUDING,  BUT NOT
 * LIMITED TO,  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA,  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY,  WHETHER IN CONTRACT,  STRICT LIABILITY,  OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE,  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>

#include "base/condcodes.hh"

using namespace gem5;

/*
 * Add 0x80 + 0x80 to get 0x100. findCarry should report a carry flag after
 * this operation.
 */
TEST(CondCodes, FindCarryWithNoCarryIn8Bit)
{
    EXPECT_TRUE(findCarry(8, 0x100, 0x80, 0x80));
}

/*
 * Add 0xf0 + 0x0f to get 0xff. findCarry should not report a carry flag after
 * this operation.
 */
TEST(CondCodes, FindNoCarryWithNoCarryIn8Bit)
{
    EXPECT_FALSE(findCarry(8, 0xff, 0xf0, 0x0f));
}

/*
 * Add 0x7f + 0x80 + 0x01 to get 0x100. findCarry should report a carry flag
 * after this operation.
 */
TEST(CondCodes, FindCarryWithCarryIn8Bit)
{
    EXPECT_TRUE(findCarry(8, 0x100, 0x80, 0x7f));
}

/*
 * Add 0x80 + 0x7e + 0x01 to get 0xff. findCarry should not report a carry
 * flag after this operation.
 */
TEST(CondCodes, FindNoCarryWithCarryIn8Bit)
{
    EXPECT_FALSE(findCarry(8, 0xff, 0x80, 0x7e));
}

/*
 * Add 0x80000000 + 0x80000000 to get 0x100000000. findCarry should report a
 * carry flag after this operation.
 */
TEST(CondCodes, FindCarryWithNoCarryIn32Bit)
{
    EXPECT_TRUE(findCarry(32, 0x100000000, 0x80000000, 0x80000000));
}

/*
 * Add 0xffff0000 + 0x0000ffff to get 0xffffffff. findCarry should not report a
 * carry flag after this operation.
 */
TEST(CondCodes, FindNoCarryWithNoCarryIn32Bit)
{
    EXPECT_FALSE(findCarry(32, 0xffffffff, 0xffff0000, 0x0000ffff));
}

TEST(CondCodes, FindCarryWithCarryIn32Bit)
{
    /*
     * Add 0x80000000 + 0x7fffffff + 0x00000001 to get 0x100000000,
     * resulting in a carry
     */
    EXPECT_TRUE(findCarry(32, 0x100000000, 0x80000000, 0x7fffffff));
    // Add 0x80000000 + 0x7ffffffe + 0x00000001 to get 0xffffffff,
    // resulting in no carry
    EXPECT_FALSE(findCarry(32, 0xffffffff, 0x80000000, 0x7ffffffe));
    // Add 0xffffffff + 0x00000000 + 0x00000001 to get 0x100000000,
    // resulting in a carry
    EXPECT_TRUE(findCarry(32, 0x100000000, 0xffffffff, 0x00000000));
}

TEST(CondCodes, FindCarryWithNoCarryIn64Bit)
{
    // Add 0x8000000000000000 + 0x8000000000000000 to get 0x10000000000000000,
    // (unrepresentable with uint64_t),  resulting in a carry
    EXPECT_TRUE(findCarry(64, 0x0000000000000000,
                              0x8000000000000000,
                              0x8000000000000000));
    /*
     * Add 0x0000000000000000 + 0x0000000000000000 to get 0x0000000000000000
     * resulting in no carry
     * We get the same sum as above case due to unrepresentability,  but we
     * should still expect no carry
     */
    EXPECT_FALSE(findCarry(64, 0x0000000000000000,
                               0x0000000000000000,
                               0x0000000000000000));
    /*
     * Add 0x8000000000000000 + 0x7fffffffffffffff to get 0xffffffffffffffff,
     * resulting in no carry
     */
    EXPECT_FALSE(findCarry(64, 0xffffffffffffffff,
                               0x8000000000000000,
                               0x7fffffffffffffff));
    /*
     * Add 0xffffffff00000000 + 0x00000000ffffffff to get 0xffffffffffffffff,
     * resulting in no carry
     */
    EXPECT_FALSE(findCarry(64, 0xffffffffffffffff,
                               0xffffffff00000000,
                               0x00000000ffffffff));
}

TEST(CondCodes, FindCarryWithCarryIn64Bit)
{
    /* Add 0x8000000000000000 + 0x8000000000000000 + 0x0000000000000001
     * to get 0x1 000000000000001 (unrepresentable with uint64_t),
     * resulting in a carry
     */
    EXPECT_TRUE(findCarry(64, 0x0000000000000000,
                              0x8000000000000000,
                              0x7fffffffffffffff));
    /*
     * Add 0x0000000000000000 + 0x0000000000000000 + 0x0000000000000001
     * resulting in no carry
     * We get the same sum as the above case due to unrepresentability, but we
     * should still expect no carry
     */
    EXPECT_FALSE(findCarry(64, 0x0000000000000001,
                               0x0000000000000000,
                               0x0000000000000000));
    /*
     * Add 0x8000000000000000 + 0x7fffffffffffffff + 0x0000000000000001
     * to get 0x1 0000000000000000 (unrepresentable with uint64_t),
     * resulting in a carry
     */
    EXPECT_TRUE(findCarry(64, 0x0000000000000000,
                              0x8000000000000000,
                              0x7fffffffffffffff));
    /*
     * Add 0xffffffff00000000 + 0x000000000000000 + 0x0000000000000001
     * to get 0x1 0000000000000000 (unrepresentable with uint64_t),
     * resulting in a carry
     */
    EXPECT_TRUE(findCarry(64, 0x0000000000000000,
                              0xffffffffffffffff,
                              0x0000000000000001));
}

TEST(CondCodes, FindOverflow8Bit)
{
    /*
     * Addition of 127 + 1 = 128 or -128 as signed two's complement.
     * Overflow occurs in this case
     */
    EXPECT_TRUE(findOverflow(8, 0x80, 0x7f, 0x01));
    /*
     * Addition of 64 + 63 = 127,  or 127 as signed two's complement.
     * No overflow occurs
     */
    EXPECT_FALSE(findOverflow(8, 0x7f, 0x40, 0x3f));
}

TEST(CondCodes, FindOverflow32Bit)
{
    /*
     * Addition of 2,147,483,647 + 1 = 2,147,483,648, or -2,147,483,648 as
     * signed two's complement. Overflow occurs in this case
     */
    EXPECT_TRUE(findOverflow(32, 0x80000000, 0x7fffffff, 0x00000001));
    /*
     * Addition of 1,073,741,824 + 1,073,741,823 = 2,147,483,647, or
     * 2,147,483,647 as signed two's complement. No overflow occurs
     */
    EXPECT_FALSE(findOverflow(32, 0x7fffffff, 0x40000000, 0x3fffffff));
}

TEST(CondCodes, FindOverflow64Bit)
{
    /*
     * Addition of 0x7fffffffffffffff + 0x0000000000000001 =
     * 0x8000000000000000, or -9,223,372,036,854,775,808 as signed two's
     * complement. Overflow occurs in this case
     */
    EXPECT_TRUE(findOverflow(64, 0x8000000000000000,
                                 0x7fffffffffffffff,
                                 0x0000000000000001));
    /* Addition of 0x4000000000000000 + 0x3fffffffffffffff =
     * 0x7fffffffffffffff, or 9,223,372,036,854,775,807 as signed two's
     * complement. No overflow occurs
     */
    EXPECT_FALSE(findOverflow(64, 0x7fffffffffffffff,
                                  0x4000000000000000,
                                  0x3fffffffffffffff));
}

TEST(CondCodes, OddParity)
{
    EXPECT_EQ(1, findParity(8, 1));
}

TEST(CondCodes, EvenParity)
{
    EXPECT_EQ(0, findParity(8, 3));
}

TEST(CondCodes, OddParityOverflow)
{
    EXPECT_EQ(1, findParity(8, 0x102));
}

TEST(CondCodes, EvenParityOverflow)
{
    EXPECT_EQ(0, findParity(4,0x43));
}

TEST(CondCodes, IsNegative)
{
    EXPECT_EQ(1, findNegative(8, 128));
}

TEST(CondCodes, IsNotNegative)
{
    EXPECT_EQ(0, findNegative(8, 127));
}

TEST(CondCodes, IsZero)
{
    EXPECT_EQ(1, findZero(8, 0));
}

TEST(CondCodes, IsNotZero)
{
    EXPECT_EQ(0, findZero(8, 1));
}

TEST(CondCodes, IsZeroOverflow)
{
    EXPECT_EQ(1, findZero(8,0x100));
}
