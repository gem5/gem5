/*
 * Copyright (c) 2019 The Regents of the University of California
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

#include <cstdint>

#include "base/str.hh"

/*
 * str.cc has "eat_lead_white", "eat_end_white", and "eat_white" fucntions to
 * remove leading and trailing whitespace. The following tests verify this
 * behavior.
 */
TEST(StrTest, EatLeadWhite)
{
    std::string val = "    hello there    ";
    eat_lead_white(val);
    EXPECT_EQ("hello there    ", val);
}

TEST(StrTest, EatLeadWhiteNoLeadingWhitespace)
{
    std::string val = "hello there    ";
    eat_lead_white(val);
    EXPECT_EQ("hello there    ", val);
}

TEST(StrTest, EatEndWhite)
{
    std::string val = "    hello there    ";
    eat_end_white(val);
    EXPECT_EQ("    hello there", val);
}

TEST(StrTest, EatEndWhiteNoTrailingWhitespace)
{
    std::string val = "    hello there";
    eat_end_white(val);
    EXPECT_EQ("    hello there", val);
}

TEST(StrTest, EatWhite)
{
    std::string val = "    hello there    ";
    eat_white(val);
    EXPECT_EQ("hello there", val);
}

TEST(StrTest, EatWhiteNoWhitespace)
{
    std::string val = "hello there";
    eat_lead_white(val);
    EXPECT_EQ("hello there", val);
}

/*
 * This tests checks that str.cc's "to_lower" function converts a string to
 * lowercase.
 */
TEST(StrTest, ToLower)
{
    std::string val = "gOoDbYe FOO@barr!";
    EXPECT_EQ("goodbye foo@barr!", to_lower(val));
}

/*
 * str.cc's "split_first" and "split_last" fucntions split a string on a
 * character into two parts. "split_first" splits on the first instance of
 * this character and "split_last" splits on the last instance of this
 * character. The character itself is not included in either of the output
 * right-hand side and left-hand side strings. If the character cannot be
 * found in the string then the left-hand side string is equal to the input
 * string and the right-hand side string is empty.
 */
TEST(StrTest, SplitFirst)
{
    std::string val = "abcdefg abcdefg";
    std::string lhs;
    std::string rhs;

    split_first(val , lhs, rhs, 'f');
    EXPECT_EQ("abcdefg abcdefg", val);
    EXPECT_EQ("abcde", lhs);
    EXPECT_EQ("g abcdefg", rhs);
}

TEST(StrTest, SplitFirstNoChar)
{
    std::string val = "abcdefg abcdefg";
    std::string lhs;
    std::string rhs;

    split_first(val , lhs, rhs, 'h');
    EXPECT_EQ("abcdefg abcdefg", val);
    EXPECT_EQ("abcdefg abcdefg", lhs);
    EXPECT_EQ("", rhs);
}

TEST(StrTest, SplitFirstOnFirstChar)
{
    std::string val = "abcdefg abcdefg";
    std::string lhs;
    std::string rhs;

    split_first(val , lhs, rhs, 'a');
    EXPECT_EQ("abcdefg abcdefg", val);
    EXPECT_EQ("", lhs);
    EXPECT_EQ("bcdefg abcdefg", rhs);
}

TEST(StrTest, SplitLast)
{
    std::string val = "abcdefg abcdefg";
    std::string lhs;
    std::string rhs;

    split_last(val , lhs, rhs, 'f');
    EXPECT_EQ("abcdefg abcdefg", val);
    EXPECT_EQ("abcdefg abcde", lhs);
    EXPECT_EQ("g", rhs);
}

TEST(StrTest, SplitLastNoChar)
{
    std::string val = "abcdefg abcdefg";
    std::string lhs;
    std::string rhs;

    split_last(val , lhs, rhs, 'h');
    EXPECT_EQ("abcdefg abcdefg", val);
    EXPECT_EQ("abcdefg abcdefg", lhs);
    EXPECT_EQ("", rhs);
}

TEST(StrTest, SplitLastOnLastChar)
{
    std::string val = "abcdefg abcdefg";
    std::string lhs;
    std::string rhs;

    split_last(val , lhs, rhs, 'g');
    EXPECT_EQ("abcdefg abcdefg", val);
    EXPECT_EQ("abcdefg abcdef", lhs);
    EXPECT_EQ("", rhs);
}


/*
 * str.cc's "tokenize" function splits a string into its constituent tokens.
 * It splits based on an input character.
 */
TEST(StrTest, TokenizeOnSpace)
{
    /*
     * val has a double space between each token with trailing and leading
     * whitespace.
     */
    std::string val = " Hello,  this  is  a  sentence. ";
    std::vector<std::string> tokens;

    /*
     * By default 'ign' is true. This means empty tokens are not included in
     * the output list.
     */
    tokenize(tokens, val, ' ');
    EXPECT_EQ(" Hello,  this  is  a  sentence. ", val);
    EXPECT_EQ(5, tokens.size());
    EXPECT_EQ("Hello,", tokens[0]);
    EXPECT_EQ("this", tokens[1]);
    EXPECT_EQ("is", tokens[2]);
    EXPECT_EQ("a", tokens[3]);
    EXPECT_EQ("sentence.", tokens[4]);
}

TEST(StrTest, TokenizeOnSpaceIgnFalse)
{
    /*
     * val has a double space between each token with trailing and leading
     * whitespace.
     */
    std::string val = " Hello,  this  is  a  sentence. ";
    std::vector<std::string> tokens;

    tokenize(tokens, val, ' ', false);
    EXPECT_EQ(" Hello,  this  is  a  sentence. ", val);
    EXPECT_EQ(11, tokens.size());
    EXPECT_EQ("", tokens[0]);
    EXPECT_EQ("Hello,", tokens[1]);
    EXPECT_EQ("", tokens[2]);
    EXPECT_EQ("this", tokens[3]);
    EXPECT_EQ("", tokens[4]);
    EXPECT_EQ("is", tokens[5]);
    EXPECT_EQ("", tokens[6]);
    EXPECT_EQ("a", tokens[7]);
    EXPECT_EQ("", tokens[8]);
    EXPECT_EQ("sentence.", tokens[9]);
    EXPECT_EQ("", tokens[10]);
}

TEST(StrTest, TokenizedTokenDoesNotExist)
{
    std::string val = "abcdefg";
    std::vector<std::string> tokens;

    tokenize(tokens, val, 'h');
    EXPECT_EQ("abcdefg", val);
    EXPECT_EQ(1, tokens.size());
    EXPECT_EQ("abcdefg", tokens[0]);
}

/*
 * str.cc's "to_number" function converts a string to a number. The function
 * will return false if this is not possible either because the string
 * represents a number out-of-range, or because the string cannot be parsed.
 */
TEST(StrTest, ToNumber8BitInt)
{
    int8_t output;
    std::string input = "-128";
    EXPECT_TRUE(to_number(input, output));
    EXPECT_EQ(-128, output);
}

TEST(StrTest, ToNumber8BitIntStringOutOfRange)
{
    int8_t output;
    std::string input = "-129";
    EXPECT_FALSE(to_number(input, output));
}

TEST(StrTest, ToNumber8BitIntInvalidString)
{
    int8_t output;
    std::string input = "onetwoeight";
    EXPECT_FALSE(to_number(input, output));
}

TEST(StrTest, ToNumberUnsigned8BitInt)
{
    uint8_t output;
    std::string input = "255";
    EXPECT_TRUE(to_number(input, output));
    EXPECT_EQ(255, output);
}

TEST(StrTest, ToNumberUnsigned8BitIntNegative)
{
    uint8_t output;
    std::string input = "-1";
    EXPECT_FALSE(to_number(input, output));
}

TEST(StrTest, ToNumber64BitInt)
{
    int64_t output;
    int64_t input_number = 0xFFFFFFFFFFFFFFFF;
    std::string input = std::to_string(input_number);
    EXPECT_TRUE(to_number(input, output));
    EXPECT_EQ(input_number, output);
}

TEST(StrTest, ToNumber64BitIntInvalidString)
{
    int64_t output;
    std::string input = " ";
    EXPECT_FALSE(to_number(input, output));
}

TEST(StrTest, ToNumberFloat)
{
    float output;
    std::string input = "0.1";
    float expected_output = 0.1;
    EXPECT_TRUE(to_number(input, output));
    EXPECT_EQ(expected_output, output);
}

TEST(StrTest, ToNumberFloatIntegerString)
{
    float output;
    std::string input = "10";
    float expected_output = 10.0;
    EXPECT_TRUE(to_number(input, output));
    EXPECT_EQ(expected_output, output);
}

TEST(StrTest, ToNumberFloatNegative)
{
    float output;
    std::string input = "-0.1";
    float expected_output = -0.1;
    EXPECT_TRUE(to_number(input, output));
    EXPECT_EQ(expected_output, output);
}

TEST(StrTest, ToNumberDouble)
{
    double output;
    std::string input = "0.0001";
    double expected_output = 0.0001;
    EXPECT_TRUE(to_number(input, output));
    EXPECT_EQ(expected_output, output);
}

TEST(StrTest, ToNumberDoubleIntegerString)
{
    double output;
    std::string input = "12345";
    double expected_output = 12345.0;
    EXPECT_TRUE(to_number(input, output));
    EXPECT_EQ(expected_output, output);
}

TEST(StrTest, ToNumberDoubleNegative)
{
    double output;
    std::string input = "-1.2345";
    double expected_output = -1.2345;
    EXPECT_TRUE(to_number(input, output));
    EXPECT_EQ(expected_output, output);
}

/*
 * The "to_bool" function takes a string, "true" or "false"
 * (case-insenstive), and sets the second argument to the bool equivilent.
 * The function will return false if it cannot parse the string.
 */
TEST(StrTest, ToBoolTrue)
{
    bool output = false;
    EXPECT_TRUE(to_bool("TrUe", output));
    EXPECT_TRUE(output);
}

TEST(StrTest, ToBoolFalse){
    bool output = true;
    EXPECT_TRUE(to_bool("fAlSe", output));
    EXPECT_FALSE(output);
}

TEST(StrTest, ToBoolInvalidInput)
{
    bool output;
    EXPECT_FALSE(to_bool("falsify", output));
}

/*
 * The "quote" function take a string and returns that string quoted (i.e.,
 * between double-quotes) if the string contains a space.
 */
TEST(StrTest, QuoteStringNoSpace)
{
    EXPECT_EQ("hello", quote("hello"));
}

TEST(StrTest, QuoteStringWithSpace)
{
    EXPECT_EQ("\"hello world\"", quote("hello world"));
}

TEST(StrTest, QuoteQuotedString)
{
    /*
     * At present, a quoted string can be quoted again.
     */
    EXPECT_EQ("\"\"hello world\"\"", quote("\"hello world\""));
}

TEST(StrTest, QuoteStringWithTab)
{
    /*
     * The "quote" function only works with standard space, not any
     * whitepsace.
     */
    EXPECT_EQ("hello\tworld", quote("hello\tworld"));
}

/*
 * str.hh has three implementations of "startswith"; a function that takes
 * string and a prefix and returns true if the string starts with the prefix.
 * One implementation takes two strings, another takes two char*, and the
 * third takes a string and a char* as a prefix.
 */
TEST(StrTest, StartswithDoubleStringDoesStartWith)
{
    std::string s = "Hello, how are you?";
    std::string prefix = "Hello";
    EXPECT_TRUE(startswith(s, prefix));
}

TEST(StrTest, StartswithDoubleStringDoesNotStartWith)
{
    std::string s = "Hello, how are you?";
    std::string prefix = "ello";
    EXPECT_FALSE(startswith(s, prefix));
}

TEST(StrTest, StartswithDoubleCharArrayDoesStartWith)
{
    const char* s = "abcdefg";
    const char* prefix = "ab";
    EXPECT_TRUE(startswith(s, prefix));
}

TEST(StrTest, StartswithDoubleCharArrayDoesNotStartWith)
{
    const char* s = " abcdefg";
    const char* prefix = "a";
    EXPECT_FALSE(startswith(s, prefix));
}

TEST(StrTest, StartswithStringCharArrayDoesStartWith)
{
    std::string s = "foobarr";
    const char* prefix = "f";
    EXPECT_TRUE(startswith(s, prefix));
}

TEST(StrTest, StartswithStringCharArrayDoesNotStartWith)
{
    std::string s = "foobarr";
    const char* prefix = "barr";
    EXPECT_FALSE(startswith(s, prefix));
}
