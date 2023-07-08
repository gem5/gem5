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

#include <gtest/gtest.h>

#include "arch/arm/matrix.hh"

using namespace gem5;

TEST(Matrix, Size)
{
    {
        // Minimum size
        MatStore<1, 1> mat;
        ASSERT_EQ(1, mat.linearSize());
    }

    {
        // Medium size
        constexpr size_t x_size = MaxMatRegRowLenInBytes / 2;
        constexpr size_t y_size = MaxMatRegRows / 2;
        MatStore<x_size, y_size> mat;
        ASSERT_EQ(x_size * y_size, mat.linearSize());
    }

    {
        // Maximum size
        MatStore<MaxMatRegRowLenInBytes, MaxMatRegRows> mat;
        ASSERT_EQ(MaxMatRegRowLenInBytes * MaxMatRegRows, mat.linearSize());
    }
}

TEST(Matrix, Zero)
{
    constexpr size_t size = 16;
    MatStore<size, size> mat;
    auto tile = mat.asTile<uint8_t>(0);

    // Initializing with non-zero value
    for (auto i = 0; i < size; i++) {
        for (auto j = 0; j < size; j++) {
            tile[i][j] = 0xAA;
        }
    }

    // zeroing the matrix
    mat.zero();

    // checking if every matrix element is set to zero
    for (auto i = 0; i < size; i++) {
        for (auto j = 0; j < size; j++) {
            ASSERT_EQ(tile[i][j], 0);
        }
    }
}

TEST(Matrix, ZeroTiles)
{
    constexpr size_t size = 16;
    MatStore<size, size> mat;
    auto byte_tile = mat.asTile<uint8_t>(0);

    // Initializing the whole tile with non-zero value
    for (auto i = 0; i < size; i++) {
        for (auto j = 0; j < size; j++) {
            byte_tile[i][j] = 0xAA;
        }
    }

    // zeroing the half-word tile 0 of matrix
    auto half_word_tile = mat.asTile<uint16_t>(0);
    half_word_tile.zero();

    // Check that every element of half-word tile 0 is zero
    for (auto i = 0; i < size / 2; i++) {
        for (auto j = 0; j < size / 2; j++) {
            ASSERT_EQ(half_word_tile[i][j], 0);
        }
    }

    // Check that every element of half-word tile 1 is 0xAAAA (note the
    // double width of the element)
    half_word_tile = mat.asTile<uint16_t>(1);
    for (auto i = 0; i < size / 2; i++) {
        for (auto j = 0; j < size / 2; j++) {
            ASSERT_EQ(half_word_tile[i][j], 0xAAAA);
        }
    }

    // Check if every matrix element on an even row is set to zero
    for (auto i = 0; i < size; i += 2) {
        for (auto j = 0; j < size; j++) {
            ASSERT_EQ(byte_tile[i][j], 0);
        }
    }

    // Check if every matrix element on an odd row is set to 0xAA
    for (auto i = 1; i < size; i += 2) {
        for (auto j = 0; j < size; j++) {
            ASSERT_EQ(byte_tile[i][j], 0xAA);
        }
    }
}

TEST(Matrix, ZeroTileHSlice)
{
    constexpr size_t size = 16;
    MatStore<size, size> mat;
    auto byte_tile = mat.asTile<uint8_t>(0);

    // Initializing the whole tile with non-zero value
    for (auto i = 0; i < size; i++) {
        for (auto j = 0; j < size; j++) {
            byte_tile[i][j] = 0xAA;
        }
    }

    // zeroing the 0th row of half-word tile 0
    auto half_word_tile = mat.asTile<uint16_t>(0);
    auto row = half_word_tile.asHSlice(0);
    row.zero();

    // Check that every element of the row is zero
    for (auto i = 0; i < size / 2; i++) {
        ASSERT_EQ(row[i], 0);
    }

    // Check that every element of row 1 is 0xAAAA
    row = half_word_tile.asHSlice(1);
    for (auto i = 0; i < size / 2; i++) {
        ASSERT_EQ(row[i], 0xAAAA);
    }

    // Check that row 0 of the byte tile is zero, and that all remaining
    // rows are unaffected
    for (auto i = 0; i < size; i++) {
        for (auto j = 0; j < size; j++) {
            if (i == 0) {
                ASSERT_EQ(byte_tile[i][j], 0);
            } else {
                ASSERT_EQ(byte_tile[i][j], 0xAA);
            }
        }
    }
}

TEST(Matrix, ZeroTileVSlice)
{
    constexpr size_t size = 16;
    MatStore<size, size> mat;
    auto byte_tile = mat.asTile<uint8_t>(0);

    // Initializing the whole tile with non-zero value
    for (auto i = 0; i < size; i++) {
        for (auto j = 0; j < size; j++) {
            byte_tile[i][j] = 0xAA;
        }
    }

    // zeroing the 0th column of half-word tile 0
    auto half_word_tile = mat.asTile<uint16_t>(0);
    auto col = half_word_tile.asVSlice(0);
    col.zero();

    // Check that every element of the column is zero
    for (auto i = 0; i < size / 2; i++) {
        ASSERT_EQ(col[i], 0);
    }

    // Check that every element of column 1 is 0xAAAA
    col = half_word_tile.asVSlice(1);
    for (auto i = 0; i < size / 2; i++) {
        ASSERT_EQ(col[i], 0xAAAA);
    }

    // Check that elements 0 & 1 of the byte tile are zero for even rows,
    // and that all remaining elements are unaffected
    for (auto i = 0; i < size; i++) {
        for (auto j = 0; j < size; j++) {
            if (i % 2 == 0 && (j == 0 || j == 1)) {
                ASSERT_EQ(byte_tile[i][j], 0);
            } else {
                ASSERT_EQ(byte_tile[i][j], 0xAA);
            }
        }
    }
}

TEST(Matrix, ZeroHSlice)
{
    constexpr size_t size = 16;
    MatStore<size, size> mat;
    auto byte_tile = mat.asTile<uint8_t>(0);

    // Initializing the whole tile with non-zero value
    for (auto i = 0; i < size; i++) {
        for (auto j = 0; j < size; j++) {
            byte_tile[i][j] = 0xAA;
        }
    }

    // Now we get a row directly from the matrix (as words, because it
    // should make no difference), zero it
    auto row = mat.asHSlice<uint32_t>(4);
    row.zero();

    // Check that every element of the row is zero
    for (auto i = 0; i < size / 4; i++) {
        ASSERT_EQ(row[i], 0);
    }

    // Check that row 4 of the byte tile is zero, and that all remaining
    // rows are unaffected
    for (auto i = 0; i < size; i++) {
        for (auto j = 0; j < size; j++) {
            if (i == 4) {
                ASSERT_EQ(byte_tile[i][j], 0);
            } else {
                ASSERT_EQ(byte_tile[i][j], 0xAA);
            }
        }
    }
}

TEST(Matrix, ZeroVSlice)
{
    constexpr size_t size = 16;
    MatStore<size, size> mat;
    auto byte_tile = mat.asTile<uint8_t>(0);

    // Initializing the whole tile with non-zero value
    for (auto i = 0; i < size; i++) {
        for (auto j = 0; j < size; j++) {
            byte_tile[i][j] = 0xAA;
        }
    }

    // Now we get a column directly from the matrix, zero it
    auto col = mat.asVSlice<uint8_t>(4);
    col.zero();

    // Check that every element of the column is zero
    for (auto i = 0; i < size; i++) {
        ASSERT_EQ(col[i], 0);
    }

    // Check that col 4 of the byte tile is zero, and that all remaining
    // rows are unaffected
    for (auto i = 0; i < size; i++) {
        for (auto j = 0; j < size; j++) {
            if (j == 4) {
                ASSERT_EQ(byte_tile[i][j], 0);
            } else {
                ASSERT_EQ(byte_tile[i][j], 0xAA);
            }
        }
    }

    // Now we repeat with a wider element type too. Reinitializing the
    // whole tile with non-zero value
    for (auto i = 0; i < size; i++) {
        for (auto j = 0; j < size; j++) {
            byte_tile[i][j] = 0xAA;
        }
    }

    // Now we get a word-wide column directly from the matrix, zero it
    auto wide_col = mat.asVSlice<uint32_t>(1);
    wide_col.zero();

    // Check that every element of the column is zero
    for (auto i = 0; i < size; i++) {
        ASSERT_EQ(wide_col[i], 0);
    }

    // Check that cols 4-7 of the byte tile are zero, and that all
    // remaining rows are unaffected
    for (auto i = 0; i < size; i++) {
        for (auto j = 0; j < size; j++) {
            if (j >= 4 && j <= 7) {
                ASSERT_EQ(byte_tile[i][j], 0);
            } else {
                ASSERT_EQ(byte_tile[i][j], 0xAA);
            }
        }
    }
}

class TwoDifferentMatRegs : public testing::Test
{
  protected:
    static constexpr size_t size = 4;

    MatStore<size, size> mat1;
    MatStore<size, size> mat2;

    void
    SetUp() override
    {
        auto tile1 = mat1.asTile<uint8_t>(0);
        auto tile2 = mat2.asTile<uint8_t>(0);

        // Initializing with non-zero value for matrix 1
        for (auto i = 0; i < size; i++) {
            for (auto j = 0; j < size; j++) {
                tile1[i][j] = 0xAA;
            }
        }

        // Initializing with zero value for matrix 2
        for (auto i = 0; i < size; i++) {
            for (auto j = 0; j < size; j++) {
                tile2[i][j] = 0x0;
            }
        }
    }
};

// Testing operator=
TEST_F(TwoDifferentMatRegs, Assignment)
{
    // Copying the matrix
    mat2 = mat1;

    auto tile2 = mat2.asTile<uint8_t>(0);

    // Checking if matrix 2 elements are 0xAA
    for (auto i = 0; i < size; i++) {
        for (auto j = 0; j < size; j++) {
            ASSERT_EQ(tile2[i][j], 0xAA);
        }
    }
}

// Testing operator==
TEST_F(TwoDifferentMatRegs, Equality)
{
    // Equality check
    ASSERT_TRUE(mat1 == mat1);
    ASSERT_TRUE(mat2 == mat2);
    ASSERT_FALSE(mat1 == mat2);
}

// Testing operator!=
TEST_F(TwoDifferentMatRegs, Inequality)
{
    // Inequality check
    ASSERT_FALSE(mat1 != mat1);
    ASSERT_FALSE(mat2 != mat2);
    ASSERT_TRUE(mat1 != mat2);
}

// Testing operator<<
TEST_F(TwoDifferentMatRegs, Printing)
{
    {
        std::ostringstream stream;
        stream << mat1;
        ASSERT_EQ(stream.str(), "[aaaaaaaa_aaaaaaaa_aaaaaaaa_aaaaaaaa]");
    }

    {
        std::ostringstream stream;
        stream << mat2;
        ASSERT_EQ(stream.str(), "[00000000_00000000_00000000_00000000]");
    }
}

// Testing ParseParam
TEST_F(TwoDifferentMatRegs, ParseParam)
{
    ParseParam<decltype(mat1)> parser;

    parser.parse("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb", mat1);
    parser.parse("cccccccccccccccccccccccccccccccc", mat2);

    for (auto i = 0; i < size; i++) {
        for (auto j = 0; j < size; j++) {
            ASSERT_EQ(mat1.asTile<uint8_t>(0)[i][j], 0xbb);
            ASSERT_EQ(mat2.asTile<uint8_t>(0)[i][j], 0xcc);
        }
    }
}

// Testing ParseParam Underflow
TEST_F(TwoDifferentMatRegs, ParseParamUnderflow)
{
    ParseParam<decltype(mat1)> parser;

    // We should trigger a fatal() here.
    EXPECT_ANY_THROW(parser.parse("b", mat1));
}

// Testing ParseParam Overflow
TEST_F(TwoDifferentMatRegs, ParseParamOverflow)
{
    ParseParam<decltype(mat1)> parser;

    // We should trigger a fatal() here.
    EXPECT_ANY_THROW(parser.parse("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb", mat1));
}

// Testing ShowParam
TEST_F(TwoDifferentMatRegs, ShowParam)
{
    ShowParam<decltype(mat1)> parser;

    {
        std::stringstream ss;
        parser.show(ss, mat1);
        ASSERT_EQ(ss.str(), "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
    }

    {
        std::stringstream ss;
        parser.show(ss, mat2);
        ASSERT_EQ(ss.str(), "00000000000000000000000000000000");
    }
}
