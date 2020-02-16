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

#include "base/callback.hh"

class CallbackImpl : public Callback
{
    public:
        bool processed = false;
        void process()
        {
            processed = true;
        }
};

class MockClass
{
    public:
        bool methodExecuted = false;
        void method()
        {
            methodExecuted = true;
        }
};

TEST(CallbackQueueTest, GetName)
{
    CallbackQueue callbackQueue;
    EXPECT_EQ("CallbackQueue", callbackQueue.name());
}

TEST(CallbackQueueTest, IsEmpty)
{
    CallbackQueue callbackQueue;
    EXPECT_TRUE(callbackQueue.empty());
}

TEST(CallbackQueueTest, IsNotEmpty)
{
    CallbackQueue callbackQueue;
    CallbackImpl impl;
    callbackQueue.add(&impl);
    EXPECT_FALSE(callbackQueue.empty());
}

TEST(CallbackQueueTest, AddOneAndProcess)
{
    CallbackQueue callbackQueue;
    CallbackImpl impl;
    callbackQueue.add(&impl);
    EXPECT_FALSE(impl.processed);
    callbackQueue.process();
    EXPECT_TRUE(impl.processed);
    // Processing a queue does not clear it.
    EXPECT_FALSE(callbackQueue.empty());
}

TEST(CallbackQueueTest, AddManyAndProcess)
{
    CallbackQueue callbackQueue;
    CallbackImpl impl1;
    CallbackImpl impl2;
    CallbackImpl impl3;
    CallbackImpl impl4;
    callbackQueue.add(&impl1);
    callbackQueue.add(&impl2);
    callbackQueue.add(&impl3);
    callbackQueue.add(&impl4);
    EXPECT_FALSE(impl1.processed);
    EXPECT_FALSE(impl2.processed);
    EXPECT_FALSE(impl3.processed);
    EXPECT_FALSE(impl4.processed);
    callbackQueue.process();
    EXPECT_TRUE(impl1.processed);
    EXPECT_TRUE(impl2.processed);
    EXPECT_TRUE(impl3.processed);
    EXPECT_TRUE(impl4.processed);
    EXPECT_FALSE(callbackQueue.empty());
}

TEST(CallbackQueueTest, ClearQueue)
{
    CallbackQueue callbackQueue;
    CallbackImpl callbackImpl;
    callbackQueue.add(&callbackImpl);
    EXPECT_FALSE(callbackQueue.empty());
    callbackQueue.clear();
    EXPECT_TRUE(callbackQueue.empty());
}

TEST(CallbackQueueTest, MakeCallbackAddByReference)
{
    CallbackQueue callbackQueue;
    MockClass mockClass;
    EXPECT_FALSE(mockClass.methodExecuted);
    callbackQueue.add<MockClass, &MockClass::method>(mockClass);
    callbackQueue.process();
    EXPECT_TRUE(mockClass.methodExecuted);
    EXPECT_FALSE(callbackQueue.empty());
}

TEST(CallbackQueueTest, MakeCallbackAddByPointer)
{
    CallbackQueue callbackQueue;
    MockClass mockClass;
    EXPECT_FALSE(mockClass.methodExecuted);
    callbackQueue.add<MockClass, &MockClass::method>(&mockClass);
    callbackQueue.process();
    EXPECT_TRUE(mockClass.methodExecuted);
    EXPECT_FALSE(callbackQueue.empty());
}
