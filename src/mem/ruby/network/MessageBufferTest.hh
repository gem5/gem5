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

#ifndef __MEM_RUBY_NETWORK_MESSAGEBUFFERTEST_HH__
#define __MEM_RUBY_NETWORK_MESSAGEBUFFERTEST_HH__

#include "base/random.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/network/MessageBuffer.hh"
#include "params/MessageBufferConsumerTest.hh"
#include "params/MessageBufferProducerTest.hh"
#include "sim/sim_exit.hh"
#include "sim/stats.hh"

namespace gem5
{

namespace ruby
{

class MessageBufferConsumerTest : public ClockedObject, Consumer
{
  public:
    MessageBuffer *buffer;
    int num_msgs;
    const int max_msgs;
    const int rate;
    const float stall_prob;

    typedef MessageBufferConsumerTestParams Params;
    MessageBufferConsumerTest(const Params &p)
        : ClockedObject(p),
          Consumer(this),
          buffer(p.buffer),
          num_msgs(0),
          max_msgs(p.num_msgs),
          rate(p.rate),
          stall_prob(p.stall_prob)
    {
        buffer->setConsumer(this);
    }

    statistics::Scalar msgsTotal;
    statistics::Formula msgsPerCy;

    void
    regStats() override
    {
        ClockedObject::regStats();

        msgsTotal.name(name() + ".msgsTotal").desc("Messages");
        msgsPerCy.name(name() + ".msgsPerCy").desc("Messages per cycle");
        msgsPerCy = msgsTotal / (simTicks / cyclesToTicks(Cycles(1)));
    }

    void
    wakeup() override
    {
        static Random::RandomPtr rng(Random::genRandom());
        if (rng->random<float>() >= stall_prob) {
            for (int i = 0; (i < rate) && buffer->isReady(curTick()); ++i) {
                buffer->dequeue(curTick(), true);
                ++msgsTotal;
                ++num_msgs;
            }
        }
        if (buffer->isReady(curTick())) {
            scheduleEvent(Cycles(1));
        }
        if (num_msgs >= max_msgs) {
            exitSimulationLoopClassic("max number of operations reached");
        }
    }

    void
    print(std::ostream &out) const override
    {
        out << name();
    };
};

class MessageBufferProducerTest : public ClockedObject
{
  public:
    MessageBuffer *buffer;
    const int rate;
    const int max_msgs;
    const Cycles latency;
    int num_msgs;
    int pending_msgs;
    EventFunctionWrapper prod_event;
    EventFunctionWrapper enq_event;

    typedef MessageBufferProducerTestParams Params;
    MessageBufferProducerTest(const Params &p)
        : ClockedObject(p),
          buffer(p.buffer),
          rate(p.rate),
          max_msgs(p.num_msgs),
          latency(p.latency),
          num_msgs(0),
          pending_msgs(0),
          prod_event([this] { produce(); }, "Produce Event"),
          enq_event([this] { enqueue(); }, "Enqueue Event")
    {}

    void
    scheduleProduce()
    {
        assert(rate != 0);
        if (rate > 0) {
            schedule(&prod_event, clockEdge(Cycles(1)));
        } else {
            schedule(&prod_event, clockEdge(Cycles(rate * -1)));
        }
    }

    void
    scheduleEnqueue(Tick tick)
    {
        if (pending_msgs > 0) {
            if (enq_event.scheduled()) {
                reschedule(&enq_event, tick, true);
            } else {
                schedule(&enq_event, tick);
            }
        }
    }

    void
    init() override
    {
        ClockedObject::init();
        scheduleProduce();
    }

    statistics::Scalar msgsTotal;
    statistics::Formula msgsPerCy;

    void
    regStats() override
    {
        ClockedObject::regStats();

        msgsTotal.name(name() + ".msgsTotal").desc("Messages");
        msgsPerCy.name(name() + ".msgsPerCy").desc("Messages per cycle");
        msgsPerCy = msgsTotal / (simTicks / cyclesToTicks(Cycles(1)));
    }

    void
    produce()
    {
        if (num_msgs >= max_msgs) {
            return;
        }
        int prod_msgs = 0;
        if (rate >= 1) {
            prod_msgs += rate;
        } else {
            prod_msgs += 1;
        }
        prod_msgs = std::min(prod_msgs, max_msgs - num_msgs);
        pending_msgs += prod_msgs;
        num_msgs += prod_msgs;
        scheduleEnqueue(curTick());
        scheduleProduce();
    }

    struct DummyMessage : Message
    {
        DummyMessage(Tick curTick) : Message(curTick, 64, nullptr) {}
        MsgPtr
        clone() const override
        {
            return std::make_shared<DummyMessage>(getTime());
        }
        void print(std::ostream &out) const override {};
    };

    void
    enqueue()
    {
        while (pending_msgs > 0) {
            if (buffer->areNSlotsAvailable(1, curTick())) {
                buffer->enqueue(std::make_shared<DummyMessage>(curTick()),
                                curTick(), cyclesToTicks(latency), false,
                                false);
                ++msgsTotal;
                --pending_msgs;
            } else {
                scheduleEnqueue(clockEdge(Cycles(1)));
                break;
            }
        }
    }
};

} // namespace ruby
} // namespace gem5

#endif //__MEM_RUBY_NETWORK_MESSAGEBUFFER_HH__
