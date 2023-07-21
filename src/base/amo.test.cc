/*
 * Copyright (c) 2020 The Regents of the University of California
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
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

#include <string>
#include <array>

#include "base/amo.hh"

using namespace gem5;

void
multiply2Op(int *b, int a)
{
    *b *= a;
}

void
multiply3Op(int *b, int a, int c)
{
    *b *= a * c;
}

void
addSubColumns(int *b, const std::array<int, 2>& a, const std::array<int, 2>& c)
{
    *b += a[0] + c[0];
    *b -= a[1] + c[1];
}

TEST(AmoTest, AtomicOpMin)
{
    // test with ints and strings
    int test_int_smaller = 5;
    int test_int_bigger = 15;
    std::string test_string_smaller = "apple";
    std::string test_string_bigger = "cat";

    std::unique_ptr<TypedAtomicOpFunctor<int>> amo_op_int =
        std::make_unique<AtomicOpMin<int>>(10);
    std::unique_ptr<TypedAtomicOpFunctor<std::string>> amo_op_string =
        std::make_unique<AtomicOpMin<std::string>>("base");
    amo_op_int->execute(&test_int_smaller);
    amo_op_int->execute(&test_int_bigger);
    amo_op_string->execute(&test_string_smaller);
    amo_op_string->execute(&test_string_bigger);

    EXPECT_EQ(test_int_smaller, 5);
    EXPECT_EQ(test_int_bigger, 10);
    EXPECT_EQ(test_string_smaller, "apple");
    EXPECT_EQ(test_string_bigger, "base");
}

TEST(AmoTest, AtomicOpMax)
{
    int test_int_smaller = 5;
    int test_int_bigger = 15;
    std::string test_string_smaller = "apple";
    std::string test_string_bigger = "cat";

    std::unique_ptr<TypedAtomicOpFunctor<int>> amo_op_int =
        std::make_unique<AtomicOpMax<int>>(10);
    std::unique_ptr<TypedAtomicOpFunctor<std::string>> amo_op_string =
        std::make_unique<AtomicOpMax<std::string>>("base");
    amo_op_int->execute(&test_int_smaller);
    amo_op_int->execute(&test_int_bigger);
    amo_op_string->execute(&test_string_smaller);
    amo_op_string->execute(&test_string_bigger);

    EXPECT_EQ(test_int_smaller, 10);
    EXPECT_EQ(test_int_bigger, 15);
    EXPECT_EQ(test_string_smaller, "base");
    EXPECT_EQ(test_string_bigger, "cat");
}

TEST(AmoTest, AtomicOpDec)
{
    int test_int = 10;
    char test_char = 'c';

    std::unique_ptr<TypedAtomicOpFunctor<int>> amo_op_int =
        std::make_unique<AtomicOpDec<int>>();
    std::unique_ptr<TypedAtomicOpFunctor<char>> amo_op_char =
        std::make_unique<AtomicOpDec<char>>();
    amo_op_int->execute(&test_int);
    amo_op_char->execute(&test_char);

    EXPECT_EQ(test_int, 9);
    EXPECT_EQ(test_char, 'b');
}

TEST(AmoTest, AtomicOpInc)
{
    int test_int = 10;
    char test_char = 'c';

    std::unique_ptr<TypedAtomicOpFunctor<int>> amo_op_int =
        std::make_unique<AtomicOpInc<int>>();
    std::unique_ptr<TypedAtomicOpFunctor<char>> amo_op_char =
        std::make_unique<AtomicOpInc<char>>();
    amo_op_int->execute(&test_int);
    amo_op_char->execute(&test_char);

    EXPECT_EQ(test_int, 11);
    EXPECT_EQ(test_char, 'd');
}

TEST(AmoTest, AtomicOpSub)
{
    int test_int = 10;
    char test_char = 'c';

    std::unique_ptr<TypedAtomicOpFunctor<int>> amo_op_int =
        std::make_unique<AtomicOpSub<int>>(2);
    std::unique_ptr<TypedAtomicOpFunctor<char>> amo_op_char =
        std::make_unique<AtomicOpSub<char>>('a');
    amo_op_int->execute(&test_int);
    amo_op_char->execute(&test_char);

    EXPECT_EQ(test_int, 8);
    EXPECT_EQ(test_char, 2);
}

TEST(AmoTest, AtomicOpAdd)
{
    int test_int = 10;
    char test_char = 'c';

    std::unique_ptr<TypedAtomicOpFunctor<int>> amo_op_int =
        std::make_unique<AtomicOpAdd<int>>(2);
    std::unique_ptr<TypedAtomicOpFunctor<char>> amo_op_char =
        std::make_unique<AtomicOpAdd<char>>(2);
    amo_op_int->execute(&test_int);
    amo_op_char->execute(&test_char);

    EXPECT_EQ(test_int, 12);
    EXPECT_EQ(test_char, 'e');
}

TEST(AmoTest, AtomicOpExch)
{
    int test_int = 10;
    char test_char = 'c';

    std::unique_ptr<TypedAtomicOpFunctor<int>> amo_op_int =
        std::make_unique<AtomicOpExch<int>>(2);
    std::unique_ptr<TypedAtomicOpFunctor<char>> amo_op_char =
        std::make_unique<AtomicOpExch<char>>('a');
    amo_op_int->execute(&test_int);
    amo_op_char->execute(&test_char);

    EXPECT_EQ(test_int, 2);
    EXPECT_EQ(test_char, 'a');
}

TEST(AmoTest, AtomicOpXor)
{
    int test_int = 10;
    char test_char = 'c';

    std::unique_ptr<TypedAtomicOpFunctor<int>> amo_op_int =
        std::make_unique<AtomicOpXor<int>>(2);
    std::unique_ptr<TypedAtomicOpFunctor<char>> amo_op_char =
        std::make_unique<AtomicOpXor<char>>('a');
    amo_op_int->execute(&test_int);
    amo_op_char->execute(&test_char);

    EXPECT_EQ(test_int, 8); // 1010 ^ 0010 = 1000
    EXPECT_EQ(test_char, 2); // 99 ^ 97 = 2
}

TEST(AmoTest, AtomicOpOr)
{
    int test_int = 8;
    bool test_bool = true;

    std::unique_ptr<TypedAtomicOpFunctor<int>> amo_op_int =
        std::make_unique<AtomicOpOr<int>>(2);
    std::unique_ptr<TypedAtomicOpFunctor<bool>> amo_op_bool =
        std::make_unique<AtomicOpOr<bool>>(false);
    amo_op_int->execute(&test_int);
    amo_op_bool->execute(&test_bool);

    EXPECT_EQ(test_int, 10);
    EXPECT_EQ(test_bool, true);
}

TEST(AmoTest, AtomicOpAnd)
{
    int test_int = 10;
    char test_char = 'c';

    std::unique_ptr<TypedAtomicOpFunctor<int>> amo_op_int =
        std::make_unique<AtomicOpAnd<int>>(6);
    std::unique_ptr<TypedAtomicOpFunctor<char>> amo_op_char =
        std::make_unique<AtomicOpAnd<char>>('a');
    amo_op_int->execute(&test_int);
    amo_op_char->execute(&test_char);

    EXPECT_EQ(test_int, 2);
    EXPECT_EQ(test_char, 'a');
}

TEST(AmoTest, AtomicGeneric2Op)
{
    int test_int = 9;

    std::unique_ptr<TypedAtomicOpFunctor<int>> amo_op_int =
        std::make_unique<AtomicGeneric2Op<int>>(9, multiply2Op);
    amo_op_int->execute(&test_int);

    EXPECT_EQ(test_int, 81);
}

TEST(AmoTest, AtomicGeneric3Op)
{
    int test_int = 2;

    std::unique_ptr<TypedAtomicOpFunctor<int>> amo_op_int =
        std::make_unique<AtomicGeneric3Op<int>>(4, 3, multiply3Op);
    amo_op_int->execute(&test_int);

    EXPECT_EQ(test_int, 24);
}

TEST(AmoTest, AtomicGenericPair3Op)
{
    int test_int = 5;

    std::array<int, 2> a = {6, 3};
    std::array<int, 2> c = {10, 8};
    std::unique_ptr<TypedAtomicOpFunctor<int>> amo_op_int =
            std::make_unique<AtomicGenericPair3Op<int>>(a, c, addSubColumns);
    amo_op_int->execute(&test_int);

    EXPECT_EQ(test_int, 10);
}
