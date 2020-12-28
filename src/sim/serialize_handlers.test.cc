/*
 * Copyright 2021 Daniel R. Carvalho
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
#include <string>

#include "sim/serialize_handlers.hh"

using namespace gem5;

TEST(SerializeTest, ParseParamInt8)
{
    ParseParam<int8_t> parser;
    int8_t value(0);

    // Zero
    EXPECT_TRUE(parser.parse("0", value));
    EXPECT_EQ(0, value);

    // Booleans
    EXPECT_FALSE(parser.parse("true", value));
    EXPECT_FALSE(parser.parse("false", value));

    // 8-bit values
    EXPECT_FALSE(parser.parse("255", value));
    EXPECT_TRUE(parser.parse("-128", value));
    EXPECT_EQ(-128, value);

    // 16-bit values
    EXPECT_FALSE(parser.parse("1000", value));
    EXPECT_FALSE(parser.parse("-1000", value));

    // 32-bit values
    EXPECT_FALSE(parser.parse("2147483648", value));
    EXPECT_FALSE(parser.parse("-1073741824", value));

    // Doubles (scientific numbers should not be converted to integers
    // correctly)
    EXPECT_FALSE(parser.parse("123456.789", value));
    EXPECT_FALSE(parser.parse("-123456.789", value));
    EXPECT_FALSE(parser.parse("9.87654e+06", value));

    // Characters
    EXPECT_TRUE(parser.parse("69", value));
    EXPECT_EQ(69, value);
    EXPECT_TRUE(parser.parse("97", value));
    EXPECT_EQ(97, value);

    // Strings
    EXPECT_FALSE(parser.parse("Test", value));
}

TEST(SerializeTest, ParseParamUint32)
{
    ParseParam<uint32_t> parser;
    uint32_t value(0);

    // Zero
    EXPECT_TRUE(parser.parse("0", value));
    EXPECT_EQ(0, value);

    // Booleans
    EXPECT_FALSE(parser.parse("true", value));
    EXPECT_FALSE(parser.parse("false", value));

    // 8-bit values
    EXPECT_TRUE(parser.parse("255", value));
    EXPECT_EQ(255, value);
    EXPECT_FALSE(parser.parse("-128", value));

    // 16-bit values
    EXPECT_TRUE(parser.parse("1000", value));
    EXPECT_EQ(1000, value);
    EXPECT_FALSE(parser.parse("-1000", value));

    // 32-bit values
    EXPECT_TRUE(parser.parse("2147483648", value));
    EXPECT_EQ(2147483648, value);
    EXPECT_FALSE(parser.parse("-1073741824", value));

    // Doubles (scientific numbers should not be converted to integers
    // correctly)
    EXPECT_TRUE(parser.parse("123456.789", value));
    EXPECT_EQ(123456, value);
    EXPECT_FALSE(parser.parse("-123456.789", value));
    EXPECT_FALSE(parser.parse("9.87654e+06", value));

    // Characters
    EXPECT_TRUE(parser.parse("69", value));
    EXPECT_EQ(69, value);
    EXPECT_TRUE(parser.parse("97", value));
    EXPECT_EQ(97, value);

    // Strings
    EXPECT_FALSE(parser.parse("Test", value));
}

TEST(SerializeTest, ParseParamDouble)
{
    ParseParam<double> parser;
    double value(0.0);

    // Zero
    EXPECT_TRUE(parser.parse("0", value));
    EXPECT_EQ(0.0, value);

    // Booleans
    EXPECT_FALSE(parser.parse("true", value));
    EXPECT_FALSE(parser.parse("false", value));

    // 8-bit values
    EXPECT_TRUE(parser.parse("255", value));
    EXPECT_EQ(255.0, value);
    EXPECT_TRUE(parser.parse("-128", value));
    EXPECT_EQ(-128.0, value);

    // 16-bit values
    EXPECT_TRUE(parser.parse("1000", value));
    EXPECT_EQ(1000.0, value);
    EXPECT_TRUE(parser.parse("-1000", value));
    EXPECT_EQ(-1000.0, value);

    // 32-bit values
    EXPECT_TRUE(parser.parse("2147483648", value));
    EXPECT_EQ(2147483648.0, value);
    EXPECT_TRUE(parser.parse("-1073741824", value));
    EXPECT_EQ(-1073741824.0, value);

    // Doubles
    EXPECT_TRUE(parser.parse("123456.789", value));
    EXPECT_EQ(123456.789, value);
    EXPECT_TRUE(parser.parse("-123456.789", value));
    EXPECT_EQ(-123456.789, value);
    EXPECT_TRUE(parser.parse("9.87654e+06", value));
    EXPECT_EQ(9.87654e+06, value);

    // Characters
    EXPECT_TRUE(parser.parse("69", value));
    EXPECT_EQ(69.0, value);
    EXPECT_TRUE(parser.parse("97", value));
    EXPECT_EQ(97.0, value);

    // Strings
    EXPECT_FALSE(parser.parse("Test", value));
}

TEST(SerializeTest, ParseParamBool)
{
    ParseParam<bool> parser;
    bool value(false);

    // Zero
    EXPECT_FALSE(parser.parse("0", value));

    // Booleans
    EXPECT_TRUE(parser.parse("true", value));
    EXPECT_EQ(true, value);
    EXPECT_TRUE(parser.parse("false", value));
    EXPECT_EQ(false, value);

    // 8-bit values
    EXPECT_FALSE(parser.parse("255", value));
    EXPECT_FALSE(parser.parse("-128", value));

    // 16-bit values
    EXPECT_FALSE(parser.parse("1000", value));
    EXPECT_FALSE(parser.parse("-1000", value));

    // 32-bit values
    EXPECT_FALSE(parser.parse("2147483648", value));
    EXPECT_FALSE(parser.parse("-1073741824", value));

    // Doubles
    EXPECT_FALSE(parser.parse("123456.789", value));
    EXPECT_FALSE(parser.parse("-123456.789", value));
    EXPECT_FALSE(parser.parse("9.87654e+06", value));

    // Characters
    EXPECT_FALSE(parser.parse("69", value));
    EXPECT_FALSE(parser.parse("97", value));

    // Strings
    EXPECT_FALSE(parser.parse("Test", value));
}

/** Characters are parsed as integers. */
TEST(SerializeTest, ParseParamChar)
{
    ParseParam<char> parser;
    char value;

    // Zero
    EXPECT_TRUE(parser.parse("48", value));
    EXPECT_EQ('0', value);

    // Booleans
    EXPECT_FALSE(parser.parse("true", value));
    EXPECT_FALSE(parser.parse("false", value));

    // 8-bit values
    EXPECT_FALSE(parser.parse("255", value));
    EXPECT_TRUE(parser.parse("-128", value));
    EXPECT_EQ(char(-128), value);

    // 16-bit values
    EXPECT_FALSE(parser.parse("1000", value));
    EXPECT_FALSE(parser.parse("-1000", value));

    // 32-bit values
    EXPECT_FALSE(parser.parse("2147483648", value));
    EXPECT_FALSE(parser.parse("-1073741824", value));

    // Doubles
    EXPECT_FALSE(parser.parse("123456.789", value));
    EXPECT_FALSE(parser.parse("-123456.789", value));
    EXPECT_FALSE(parser.parse("9.87654e+06", value));

    // Characters
    EXPECT_TRUE(parser.parse("69", value));
    EXPECT_EQ('E', value);
    EXPECT_TRUE(parser.parse("97", value));
    EXPECT_EQ('a', value);

    // Strings
    EXPECT_FALSE(parser.parse("Test", value));
}

TEST(SerializeTest, ParseParamString)
{
    ParseParam<std::string> parser;
    std::string value("");

    // Zero
    EXPECT_TRUE(parser.parse("0", value));
    EXPECT_EQ("0", value);

    // Booleans
    EXPECT_TRUE(parser.parse("true", value));
    EXPECT_EQ("true", value);
    EXPECT_TRUE(parser.parse("false", value));
    EXPECT_EQ("false", value);

    // 8-bit values
    EXPECT_TRUE(parser.parse("255", value));
    EXPECT_EQ("255", value);
    EXPECT_TRUE(parser.parse("-128", value));
    EXPECT_EQ("-128", value);

    // 16-bit values
    EXPECT_TRUE(parser.parse("1000", value));
    EXPECT_EQ("1000", value);
    EXPECT_TRUE(parser.parse("-1000", value));
    EXPECT_EQ("-1000", value);

    // 32-bit values
    EXPECT_TRUE(parser.parse("2147483648", value));
    EXPECT_EQ("2147483648", value);
    EXPECT_TRUE(parser.parse("-1073741824", value));
    EXPECT_EQ("-1073741824", value);

    // Doubles
    EXPECT_TRUE(parser.parse("123456.789", value));
    EXPECT_EQ("123456.789", value);
    EXPECT_TRUE(parser.parse("-123456.789", value));
    EXPECT_EQ("-123456.789", value);
    EXPECT_TRUE(parser.parse("9.87654e+06", value));
    EXPECT_EQ("9.87654e+06", value);

    // Characters
    EXPECT_TRUE(parser.parse("69", value));
    EXPECT_EQ("69", value);
    EXPECT_TRUE(parser.parse("97", value));
    EXPECT_EQ("97", value);

    // Strings
    EXPECT_TRUE(parser.parse("Test", value));
    EXPECT_EQ("Test", value);
}

TEST(SerializeTest, ShowParamInt8)
{
    ShowParam<int8_t> parser;
    std::stringstream ss;

    parser.show(ss, 0);
    EXPECT_EQ("0", ss.str());
    ss.str("");
    parser.show(ss, 127);
    EXPECT_EQ("127", ss.str());
    ss.str("");
    parser.show(ss, -128);
    EXPECT_EQ("-128", ss.str());
    ss.str("");
}

TEST(SerializeTest, ShowParamUint32)
{
    ShowParam<uint32_t> parser;
    std::stringstream ss;

    parser.show(ss, 0);
    EXPECT_EQ("0", ss.str());
    ss.str("");
    parser.show(ss, 255);
    EXPECT_EQ("255", ss.str());
    ss.str("");
    parser.show(ss, 1000);
    EXPECT_EQ("1000", ss.str());
    ss.str("");
    parser.show(ss, 2147483648);
    EXPECT_EQ("2147483648", ss.str());
    ss.str("");
    parser.show(ss, (double)123456.789);
    EXPECT_EQ("123456", ss.str());
    ss.str("");
    parser.show(ss, 9.87654e+06);
    EXPECT_EQ("9876540", ss.str());
    ss.str("");
}

/**
 * Test converting doubles to strings. Floating numbers are expected to
 * have 6-digit precision.
 */
TEST(SerializeTest, ShowParamDouble)
{
    ShowParam<double> parser;
    std::stringstream ss;

    parser.show(ss, 0);
    EXPECT_EQ("0", ss.str());
    ss.str("");
    parser.show(ss, 255);
    EXPECT_EQ("255", ss.str());
    ss.str("");
    parser.show(ss, -1000);
    EXPECT_EQ("-1000", ss.str());
    ss.str("");
    parser.show(ss, 123456.789);
    EXPECT_EQ("123457", ss.str());
    ss.str("");
    parser.show(ss, -123456.789);
    EXPECT_EQ("-123457", ss.str());
    ss.str("");
    parser.show(ss, 1234567.89);
    EXPECT_EQ("1.23457e+06", ss.str());
    ss.str("");
    parser.show(ss, -1234567.89);
    EXPECT_EQ("-1.23457e+06", ss.str());
    ss.str("");
    parser.show(ss, 9.87654e+06);
    EXPECT_EQ("9.87654e+06", ss.str());
    ss.str("");
}

TEST(SerializeTest, ShowParamBool)
{
    ShowParam<bool> parser;
    std::stringstream ss;

    parser.show(ss, true);
    EXPECT_EQ("true", ss.str());
    ss.str("");
    parser.show(ss, false);
    EXPECT_EQ("false", ss.str());
    ss.str("");
}

TEST(SerializeTest, ShowParamChar)
{
    ShowParam<char> parser;
    std::stringstream ss;

    parser.show(ss, 'E');
    EXPECT_EQ("69", ss.str()); // int('E')=69
    ss.str("");
    parser.show(ss, 'a');
    EXPECT_EQ("97", ss.str()); // int('a')=97
    ss.str("");
}

TEST(SerializeTest, ShowParamString)
{
    ShowParam<std::string> parser;
    std::stringstream ss;

    parser.show(ss, "test");
    EXPECT_EQ("test", ss.str());
    ss.str("");
    parser.show(ss, "tEsT");
    EXPECT_EQ("tEsT", ss.str());
    ss.str("");
}
