/*
 * Copyright 2020 Google Inc.
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

// For EXPECT_THAT and HasSubstr
#include <gmock/gmock.h>

#include "call_type.hh"

// Simple substitute definitions for Args and DispatchTable, with an int so
// we can tell instances apart.
class Args { public: int i; };
class DispatchTable { public: int i; };

class TestCallType : public CallType
{
  protected:
    bool isDefault() const override { return testIsDefault; }
    bool checkArgs(Args &args) override { return args.i == testAcceptArgs; }
    void
    init() override
    {
        testInitHappened = true;
        CallType::init();
    }

    void printBrief(std::ostream &os) const override { os << testBrief; }
    void printDesc(std::ostream &os) const override { os << testDesc; }
  public:
    static std::vector<CallType *> &testGetAllTypes() { return allTypes(); }

    // Usage strings to return.
    std::string testBrief;
    std::string testDesc;

    // Return this dispatch table when requested.
    DispatchTable testDt = { 0 };
    const DispatchTable &getDispatch() const override { return testDt; }

    // If an Args has this value, accept it.
    int testAcceptArgs = 0;

    // Whether this call type should be considered default.
    bool testIsDefault = false;

    // Whether init has been called.
    bool testInitHappened = false;
};

TEST(CallTypeTest, Constructor)
{
    auto &all_types = TestCallType::testGetAllTypes();

    // There should be no call types yet.
    EXPECT_EQ(all_types.size(), 0);

    // Create one.
    TestCallType test_ct;

    // Set the dispatch table to something we'll recognize.
    test_ct.testDt.i = 0xaa55;

    // Verify that the list of all call types has one in it now.
    EXPECT_EQ(all_types.size(), 1);

    // Verify that that was our call type by verifying that the dispatch table
    // has our signature.
    EXPECT_EQ(all_types[0]->getDispatch().i, 0xaa55);

    // Clear all_types, since call types don't clean up after themselves as
    // they go away.
    all_types.clear();
}

TEST(CallTypeTest, DetectOne)
{
    auto &all_types = TestCallType::testGetAllTypes();

    // One option selected.
    TestCallType option1;
    option1.testAcceptArgs = 1;
    option1.testIsDefault = true;
    option1.testDt.i = 1;

    EXPECT_EQ(all_types.size(), 1);

    Args args;
    args.i = 1;

    EXPECT_FALSE(option1.testInitHappened);

    auto &ct = CallType::detect(args);

    // Verify that we selected the only option.
    EXPECT_TRUE(option1.testInitHappened);
    EXPECT_EQ(ct.getDispatch().i, 1);

    // One option, selecting the default.
    option1.testInitHappened = false;

    // Args will not match.
    args.i = 2;

    auto &def_ct = CallType::detect(args);

    // Verify that the one option was defaulted to.
    EXPECT_TRUE(option1.testInitHappened);
    EXPECT_EQ(def_ct.getDispatch().i, 1);

    // Reset all_types.
    all_types.clear();
}

TEST(CallTypeTest, DetectTwo)
{
    auto &all_types = TestCallType::testGetAllTypes();

    // One of two options selected.
    TestCallType option1;
    option1.testAcceptArgs = 1;
    option1.testIsDefault = true;
    option1.testDt.i = 1;

    TestCallType option2;
    option2.testAcceptArgs = 2;
    option2.testIsDefault = false;
    option2.testDt.i = 2;

    EXPECT_EQ(all_types.size(), 2);

    // Select the first option.
    Args args;
    args.i = 1;

    EXPECT_FALSE(option1.testInitHappened);
    EXPECT_FALSE(option2.testInitHappened);

    auto &ct1 = CallType::detect(args);

    // Verify that we selected the first option.
    EXPECT_TRUE(option1.testInitHappened);
    EXPECT_FALSE(option2.testInitHappened);
    EXPECT_EQ(ct1.getDispatch().i, 1);

    option1.testInitHappened = false;
    option2.testInitHappened = false;

    // Select the second option.
    args.i = 2;

    auto &ct2 = CallType::detect(args);

    // Verify that we selected the second option.
    EXPECT_FALSE(option1.testInitHappened);
    EXPECT_TRUE(option2.testInitHappened);
    EXPECT_EQ(ct2.getDispatch().i, 2);

    option1.testInitHappened = false;
    option2.testInitHappened = false;

    // Default to the first option.
    args.i = 3;

    auto &def_ct1 = CallType::detect(args);

    // Verify that we selected the first option.
    EXPECT_TRUE(option1.testInitHappened);
    EXPECT_FALSE(option2.testInitHappened);
    EXPECT_EQ(ct1.getDispatch().i, 1);

    option1.testInitHappened = false;
    option2.testInitHappened = false;

    // Default to the second option.
    option1.testIsDefault = false;
    option2.testIsDefault = true;

    auto &def_ct2 = CallType::detect(args);

    // Verify that we selected the second option.
    EXPECT_FALSE(option1.testInitHappened);
    EXPECT_TRUE(option2.testInitHappened);
    EXPECT_EQ(ct2.getDispatch().i, 2);

    option1.testInitHappened = false;
    option2.testInitHappened = false;

    // Reset all_types.
    all_types.clear();
}

TEST(CallTypeTest, Usage)
{
    auto &all_types = TestCallType::testGetAllTypes();

    TestCallType ct1;
    ct1.testBrief = "brief 1";
    ct1.testDesc = "A longer description of call type 1, which is long.";

    TestCallType ct2;
    ct2.testBrief = "short 2";
    ct2.testDesc = "Very verbose text saying what call type 2 is, "
        "and is different from 1.";

    EXPECT_EQ(all_types.size(), 2);

    auto summary = CallType::usageSummary();

    // For now, just expect that the brief and full descriptive text shows up
    // in the summary somewhere.
    //
    // More strict checks might test that things were placed in the right
    // order, were on their own lines when appropriate, etc.
    EXPECT_THAT(summary, testing::HasSubstr(ct1.testBrief));
    EXPECT_THAT(summary, testing::HasSubstr(ct1.testDesc));
    EXPECT_THAT(summary, testing::HasSubstr(ct2.testBrief));
    EXPECT_THAT(summary, testing::HasSubstr(ct2.testDesc));
    EXPECT_THAT(summary, testing::HasSubstr(ct2.testDesc));

    // Reset all_types.
    all_types.clear();
}
