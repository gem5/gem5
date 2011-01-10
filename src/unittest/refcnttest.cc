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

#include "base/refcnt.hh"

using namespace std;

namespace {

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
        cout << "  Creating object \"" << _tag << "\"\n";
        liveList.push_front(this);
        liveIt = liveList.begin();
    }

    ~TestRC()
    {
        cout << "  Destroying object \"" << _tag << "\"\n";
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

}

int
main()
{
    assert(live() == 0);
    assert(liveChange() == 0);

    // Create an empty Ptr and verify it's data pointer is NULL.
    cout << "NULL check.\n";
    Ptr nullCheck;
    assert(nullCheck.get() == NULL);

    assert(liveChange() == 0);

    // Construct a Ptr from a TestRC pointer.
    cout << "Construction from pointer.\n";
    Ptr constFromPointer = new TestRC("construction from pointer");

    assert(liveChange() == 1);

    // Construct a Ptr from an existing Ptr.
    cout << "Construction from a Ptr.\n";
    Ptr constFromPtr = constFromPointer;

    assert(liveChange() == 0);

    // Test a Ptr being destroyed.
    cout << "Destroying a Ptr.\n";
    Ptr *ptrPtr = new Ptr(new TestRC("destroying a ptr"));
    assert(liveChange() == 1);
    delete ptrPtr;
    assert(liveChange() == -1);

    // Test assignment from a pointer and from a Ptr.
    cout << "Assignment operators.\n";
    Ptr assignmentTarget;
    TestRC *assignmentSourcePointer = new TestRC("assignment source 1");
    assert(liveChange() == 1);
    assignmentTarget = assignmentSourcePointer;
    assert(liveChange() == 0);
    assignmentTarget = NULL;
    assert(liveChange() == -1);
    Ptr assignmentSourcePtr(new TestRC("assignment source 2"));
    assert(liveChange() == 1);
    assignmentTarget = assignmentSourcePtr;
    assert(liveChange() == 0);
    assignmentSourcePtr = NULL;
    assert(liveChange() == 0);
    assignmentTarget = NULL;
    assert(liveChange() == -1);

    // Test access to members of the pointed to class and dereferencing.
    cout << "Access to members.\n";
    TestRC *accessTest = new TestRC("access test");
    Ptr accessTestPtr = accessTest;
    accessTest->testVal = 1;
    assert(accessTestPtr->testVal == 1);
    assert((*accessTestPtr).testVal == 1);
    accessTest->testVal = 2;
    assert(accessTestPtr->testVal == 2);
    assert((*accessTestPtr).testVal == 2);
    accessTestPtr->testVal = 3;
    assert(accessTest->testVal == 3);
    (*accessTestPtr).testVal = 4;
    assert(accessTest->testVal == 4);
    accessTestPtr = NULL;
    accessTest = NULL;
    assert(liveChange() == 0);

    // Test bool and ! operator overloads.
    cout << "Conversion to bool and ! overload.\n";
    Ptr boolTest = new TestRC("bool test");
    assert(boolTest == true);
    assert(!boolTest == false);
    boolTest = NULL;
    assert(boolTest == false);
    assert(!boolTest == true);
    assert(liveChange() == 0);

    // Test the equality operators.
    cout << "Equality operators.\n";
    TestRC *equalTestA = new TestRC("equal test a");
    Ptr equalTestAPtr = equalTestA;
    Ptr equalTestAPtr2 = equalTestA;
    TestRC *equalTestB = new TestRC("equal test b");
    Ptr equalTestBPtr = equalTestB;
    assert(equalTestA == equalTestAPtr);
    assert(equalTestAPtr == equalTestA);
    assert(equalTestAPtr == equalTestAPtr2);
    assert(equalTestA != equalTestBPtr);
    assert(equalTestAPtr != equalTestB);
    assert(equalTestAPtr != equalTestBPtr);

    cout << flush;
}
