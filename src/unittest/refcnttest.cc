/*
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
 *
 * Authors: Gabe Black
 */

#include <cassert>
#include <iostream>
#include <list>

#include "base/cprintf.hh"
#include "base/refcnt.hh"
#include "unittest/unittest.hh"

using namespace std;
using UnitTest::setCase;

namespace {

bool printNewDel = false;

class TestRC;
typedef list<TestRC *> LiveList;
LiveList liveList;

int
live()
{
    return liveList.size();
}

int
liveChange()
{
    static int oldLive = 0;
    int newLive = live();
    int diff =  newLive - oldLive;
    oldLive = newLive;
    return diff;
}

class TestRC : public RefCounted
{
  protected:
    const char *_tag;
    LiveList::iterator liveIt;

  public:
    TestRC(const char *newTag) : _tag(newTag)
    {
        if (printNewDel)
            cprintf("  Creating object \"%s\"\n", _tag);
        liveList.push_front(this);
        liveIt = liveList.begin();
    }

    ~TestRC()
    {
        if (printNewDel)
            cprintf("  Destroying object \"%s\"\n", _tag);
        liveList.erase(liveIt);
    }

    const char *
    tag()
    {
        return _tag;
    }

    int testVal;
};

typedef RefCountingPtr<TestRC> Ptr;

} // anonymous namespace

int
main()
{
    assert(live() == 0);
    assert(liveChange() == 0);

    // Create an empty Ptr and verify it's data pointer is NULL.
    setCase("NULL check");
    Ptr nullCheck;
    EXPECT_EQ(nullCheck.get(), NULL);

    EXPECT_EQ(liveChange(), 0);

    // Construct a Ptr from a TestRC pointer.
    setCase("construction from pointer");
    Ptr constFromPointer = new TestRC("construction from pointer");

    EXPECT_EQ(liveChange(), 1);

    // Construct a Ptr from an existing Ptr.
    setCase("construction from a Ptr");
    Ptr constFromPtr = constFromPointer;

    EXPECT_EQ(liveChange(), 0);

    // Test a Ptr being destroyed.
    setCase("destroying a Ptr");
    Ptr *ptrPtr = new Ptr(new TestRC("destroying a ptr"));
    EXPECT_EQ(liveChange(), 1);
    delete ptrPtr;
    EXPECT_EQ(liveChange(), -1);

    // Test assignment from a pointer and from a Ptr.
    setCase("assignment operators");
    Ptr assignmentTarget;
    TestRC *assignmentSourcePointer = new TestRC("assignment source 1");
    EXPECT_EQ(liveChange(), 1);
    assignmentTarget = assignmentSourcePointer;
    EXPECT_EQ(liveChange(), 0);
    assignmentTarget = NULL;
    EXPECT_EQ(liveChange(), -1);
    Ptr assignmentSourcePtr(new TestRC("assignment source 2"));
    EXPECT_EQ(liveChange(), 1);
    assignmentTarget = assignmentSourcePtr;
    EXPECT_EQ(liveChange(), 0);
    assignmentSourcePtr = NULL;
    EXPECT_EQ(liveChange(), 0);
    assignmentTarget = NULL;
    EXPECT_EQ(liveChange(), -1);

    // Test access to members of the pointed to class and dereferencing.
    setCase("access to members");
    TestRC *accessTest = new TestRC("access test");
    Ptr accessTestPtr = accessTest;
    accessTest->testVal = 1;
    EXPECT_EQ(accessTestPtr->testVal, 1);
    EXPECT_EQ((*accessTestPtr).testVal, 1);
    accessTest->testVal = 2;
    EXPECT_EQ(accessTestPtr->testVal, 2);
    EXPECT_EQ((*accessTestPtr).testVal, 2);
    accessTestPtr->testVal = 3;
    EXPECT_EQ(accessTest->testVal, 3);
    (*accessTestPtr).testVal = 4;
    EXPECT_EQ(accessTest->testVal, 4);
    accessTestPtr = NULL;
    accessTest = NULL;
    EXPECT_EQ(liveChange(), 0);

    // Test bool and ! operator overloads.
    setCase("conversion to bool and ! overload");
    Ptr boolTest = new TestRC("bool test");
    EXPECT_EQ(boolTest, true);
    EXPECT_EQ(!boolTest, false);
    boolTest = NULL;
    EXPECT_EQ(boolTest, false);
    EXPECT_EQ(!boolTest, true);
    EXPECT_EQ(liveChange(), 0);

    // Test the equality operators.
    setCase("equality operators");
    TestRC *equalTestA = new TestRC("equal test a");
    Ptr equalTestAPtr = equalTestA;
    Ptr equalTestAPtr2 = equalTestA;
    TestRC *equalTestB = new TestRC("equal test b");
    Ptr equalTestBPtr = equalTestB;
    EXPECT_TRUE(equalTestA == equalTestAPtr);
    EXPECT_TRUE(equalTestAPtr == equalTestA);
    EXPECT_TRUE(equalTestAPtr == equalTestAPtr2);
    EXPECT_TRUE(equalTestA != equalTestBPtr);
    EXPECT_TRUE(equalTestAPtr != equalTestB);
    EXPECT_TRUE(equalTestAPtr != equalTestBPtr);

    return UnitTest::printResults();
}
