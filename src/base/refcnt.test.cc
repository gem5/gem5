/*
 * Copyright (c) 2019 The Regents of The University of California
 * All rights resvered.
 *
 * Copyright (c) 2010 The Regents of The University of Michigan
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

#include <list>

#include "base/refcnt.hh"

using namespace gem5;

namespace {

class TestRC;
typedef std::list<TestRC *> LiveList;
LiveList liveList;

int
liveListSize()
{
    return liveList.size();
}

class TestRC : public RefCounted
{
  protected:
    LiveList::iterator liveIt;

  public:
    TestRC()
    {
        liveList.push_front(this);
        liveIt = liveList.begin();
    }

    ~TestRC()
    {
        liveList.erase(liveIt);
    }

    int testVal;
};
typedef RefCountingPtr<TestRC> Ptr;

} // anonymous namespace

TEST(RefcntTest, NullPointerCheck)
{
    // Create an empty Ptr and verify it's data pointer is NULL.
    Ptr nullCheck;
    EXPECT_EQ(NULL, nullCheck.get());
    EXPECT_EQ(0, liveListSize());
}

TEST(RefcntTest, ConstructionFromPointer)
{
    // Construct a Ptr from a TestRC pointer.
    Ptr constFromPointer = new TestRC();
    EXPECT_EQ(1, liveListSize());
}

TEST(RefcntTest, ConstructionFromExistingPointer)
{
    // Construct a Ptr from an existing Ptr.
    Ptr constFromPointer1 = new TestRC();
    Ptr constFromPointer2 = constFromPointer1;

    EXPECT_EQ(1, liveListSize());
}

TEST(RefcntTest, DestroyPointer)
{
    // Test a Ptr being destroyed.
    Ptr *ptrPtr = new Ptr(new TestRC());
    EXPECT_EQ(1, liveListSize());
    delete ptrPtr;
    EXPECT_EQ(0, liveListSize());
}

TEST(RefcntTest, AssignmentFromAPointerFromAPointer)
{
    // Test assignment from a pointer and from a Ptr.
    Ptr assignmentTarget;
    TestRC *assignmentSourcePointer = new TestRC();
    EXPECT_EQ(liveListSize(), 1);
    assignmentTarget = assignmentSourcePointer;
    EXPECT_EQ(liveListSize(), 1);
    assignmentTarget = NULL;
    EXPECT_EQ(liveListSize(), 0);
    Ptr assignmentSourcePtr(new TestRC());
    EXPECT_EQ(liveListSize(), 1);
    assignmentTarget = assignmentSourcePtr;
    EXPECT_EQ(liveListSize(), 1);
    assignmentSourcePtr = NULL;
    EXPECT_EQ(liveListSize(), 1);
    assignmentTarget = NULL;
    EXPECT_EQ(liveListSize(), 0);
}

TEST(RefcntTest, AccessToClassPointers)
{
    // Test access to members of the pointed to class and dereferencing.
    TestRC *accessTest = new TestRC();
    Ptr accessTestPtr = accessTest;
    accessTest->testVal = 1;
    EXPECT_EQ(1, accessTestPtr->testVal);
    EXPECT_EQ(1, (*accessTestPtr).testVal);
    accessTest->testVal = 2;
    EXPECT_EQ(2, accessTestPtr->testVal);
    EXPECT_EQ(2, (*accessTestPtr).testVal);
    accessTestPtr->testVal = 3;
    EXPECT_EQ(3, accessTest->testVal);
    (*accessTestPtr).testVal = 4;
    EXPECT_EQ(4, accessTest->testVal);
    accessTestPtr = NULL;
    accessTest = NULL;
    EXPECT_EQ(0, liveListSize());
}

TEST(RefcntTest, BoolAndLogicalNotOperatorOverloads)
{
    // Test bool and ! operator overloads.
    Ptr boolTest = new TestRC();
    EXPECT_EQ(boolTest, true);
    EXPECT_EQ(!boolTest, false);
    boolTest = NULL;
    EXPECT_FALSE(boolTest);
    EXPECT_TRUE(!boolTest);
    EXPECT_EQ(0, liveListSize());
}

TEST(RefcntTest, EqualityOperators)
{
    // Test the equality operators.
    TestRC *equalTestA = new TestRC();
    Ptr equalTestAPtr = equalTestA;
    Ptr equalTestAPtr2 = equalTestA;
    TestRC *equalTestB = new TestRC();
    Ptr equalTestBPtr = equalTestB;
    EXPECT_TRUE(equalTestA == equalTestAPtr);
    EXPECT_TRUE(equalTestAPtr == equalTestA);
    EXPECT_TRUE(equalTestAPtr == equalTestAPtr2);
    EXPECT_TRUE(equalTestA != equalTestBPtr);
    EXPECT_TRUE(equalTestAPtr != equalTestB);
    EXPECT_TRUE(equalTestAPtr != equalTestBPtr);
}
