/*
 * Copyright (c) 2011 Advanced Micro Devices, Inc.
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
 *
 * Authors: Gabe Black
 */

/**
 * @file This file defines functions and macros for use in unit tests.
 */

#ifndef __UNITTEST_UNITTEST_HH__
#define __UNITTEST_UNITTEST_HH__

namespace UnitTest {

/**
 * Function that actually handles checking whether an EXPECT_* passed. This
 * should be used through the EXPECT macros below and not called directly.
 * @param file The name of the file this check is in.
 * @param line The line number this check is on.
 * @param test Text specifying what check is being performed.
 * @param result Whether the check passed.
 */
void checkVal(const char *file, const unsigned line,
              const char *test, const bool result);

/**
 * Print on pass is a switch that specifies whether to print a message even
 * when a check passes. It's default value is whether or not "PRINT_ON_PASS"
 * is set in the calling environment. What it's actually set to is ignored.
 */

/**
 * Function for retrieving the current setting for print on pass.
 * @return The current setting.
 */
bool printOnPass();

/**
 * Function for setting print on pass.
 * @param newVal The new setting.
 */
void printOnPass(bool newVal);

/**
 * Function that returns the current number of passed checks.
 * @return Number of checks that have passed so far.
 */
unsigned passes();

/**
 * Function that returns the current number of failed checks.
 * @return Number of checks that have failed so far.
 */
unsigned failures();

/**
 * Function to call at the end of a test that prints an overall result and a
 * summary of how many checks passed and failed. main() should return the
 * return value of this function which is the number of failed checks.
 * @return Number of failed checks.
 */
unsigned printResults();

/// Zero the number of passes and failures so far.
void reset();

/**
 * Sets the current test case. Test cases are used to group checks together and
 * describe what that group is doing. Setting a new case defines the start of
 * a new group and the end of the previous one. The case string is used in
 * place and not copied, so don't modify or invalidate it until a new case
 * label is installed.
 * @param newCase The name of the new test case.
 */
void setCase(const char *newCase);

} // namespace UnitTest

/// A macro which verifies that expr evaluates to true.
#define EXPECT_TRUE(expr) \
    UnitTest::checkVal(__FILE__, __LINE__, "EXPECT_TRUE(" #expr ")", (expr))
/// A macro which verifies that expr evaluates to false.
#define EXPECT_FALSE(expr) \
    UnitTest::checkVal(__FILE__, __LINE__, \
            "EXPECT_FALSE(" #expr ")", (expr) == false)
/// A macro which verifies that lhs and rhs are equal to each other.
#define EXPECT_EQ(lhs, rhs) \
    UnitTest::checkVal(__FILE__, __LINE__, \
            "EXPECT_EQ(" #lhs ", " #rhs ")", (lhs) == (rhs));

#endif
