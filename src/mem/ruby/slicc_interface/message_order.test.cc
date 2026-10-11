/*
 * Copyright (c) 2026 Arm Limited
 * All rights reserved.
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

#include "mem/ruby/slicc_interface/Message.hh"

class TestMessage : public gem5::ruby::Message
{
  public:
    TestMessage(gem5::Tick cur_time, int block_size)
        : gem5::ruby::Message(cur_time, block_size, nullptr)
    {}

    gem5::ruby::MsgPtr
    clone() const override
    {
        return gem5::ruby::MsgPtr(new TestMessage(*this));
    }

    void
    print(std::ostream &out) const override
    {
        out << "TestMessage";
    }
};

TEST(MessageOrdering, TransactionTimeOverridesCounter)
{
    gem5::ruby::MsgPtr a(new TestMessage(0, 64));
    gem5::ruby::MsgPtr b(new TestMessage(0, 64));

    a->setLastEnqueueTime(100);
    b->setLastEnqueueTime(100);

    a->setTransactionTime(10);
    b->setTransactionTime(20);

    a->setMsgCounter(999);
    b->setMsgCounter(1);

    EXPECT_FALSE(a > b);
    EXPECT_TRUE(b > a);
}

TEST(MessageOrdering, MsgCounterStillBreaksTransactionTie)
{
    gem5::ruby::MsgPtr a(new TestMessage(0, 64));
    gem5::ruby::MsgPtr b(new TestMessage(0, 64));

    a->setLastEnqueueTime(100);
    b->setLastEnqueueTime(100);

    a->setTransactionTime(10);
    b->setTransactionTime(10);

    a->setMsgCounter(999);
    b->setMsgCounter(1);

    EXPECT_TRUE(a > b);
    EXPECT_FALSE(b > a);
}
