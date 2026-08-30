/*
 * Copyright (c) 2026 The Regents of The University of California
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

#include <chrono>
#include <future>
#include <thread>

#include "sim/eventq.hh"
#include "sim/global_event.hh"

namespace gem5
{
namespace
{

using namespace std::chrono_literals;

class LockCheckingGlobalEvent : public GlobalEvent
{
  public:
    LockCheckingGlobalEvent()
        : GlobalEvent(Default_Pri, 0),
          contenderIsReady(contenderReady.get_future()),
          contenderStarted(contenderStart.get_future()),
          contenderIsAttempting(contenderAttempt.get_future()),
          contenderAcquired(contenderAcquire.get_future())
    {}

    void
    process() override
    {
        contenderStart.set_value();
        contenderIsAttempting.wait();
        lockWasAvailable =
            contenderAcquired.wait_for(1s) == std::future_status::ready;
    }

    const char *
    description() const override
    { return "lock checking event"; }

    std::promise<void> contenderReady;
    std::future<void> contenderIsReady;
    std::promise<void> contenderStart;
    std::future<void> contenderStarted;
    std::promise<void> contenderAttempt;
    std::future<void> contenderIsAttempting;
    std::promise<void> contenderAcquire;
    std::future<void> contenderAcquired;
    bool lockWasAvailable = false;
};

TEST(GlobalEventTest, ProcessRunsWithEventQueueLocked)
{
    EventQueue *queue = getEventQueue(0);
    curEventQueue(queue);

    LockCheckingGlobalEvent event;
    GlobalEvent::BarrierEvent barrierEvent(&event, Event::Default_Pri, 0);

    std::thread contender([&event, queue] {
        event.contenderReady.set_value();
        event.contenderStarted.wait();
        event.contenderAttempt.set_value();
        std::lock_guard<EventQueue> lock(*queue);
        event.contenderAcquire.set_value();
    });
    event.contenderIsReady.wait();

    {
        std::lock_guard<EventQueue> lock(*queue);
        barrierEvent.process();
    }
    contender.join();

    EXPECT_FALSE(event.lockWasAvailable);
}

} // anonymous namespace
} // namespace gem5
