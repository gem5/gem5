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

#include <cstdlib>
#include <functional>

#include "dev/dma_device.hh"
#include "sim/drain.hh"
#include "sim/eventq.hh"

namespace gem5
{
namespace
{

class TestDmaCallback : public DmaCallback
{
  private:
    std::function<void()> callback;

    void
    process() override
    { callback(); }

  public:
    explicit TestDmaCallback(std::function<void()> callback)
        : callback(std::move(callback))
    {}
};

TEST(DmaCallbackTest, ChainedCallbackCompletesDuringDrain)
{
    EXPECT_EXIT(
        {
            EventQueue event_queue("DmaCallbackTest Queue");
            curEventQueue(&event_queue);

            Event *second_event = nullptr;
            auto *first_callback = new TestDmaCallback([&second_event] {
                auto *second_callback = new TestDmaCallback([] {});
                second_event = second_callback->getChunkEvent();
            });
            Event *first_event = first_callback->getChunkEvent();

            // This pass counts first_callback, which returns Draining.
            if (DrainManager::instance().tryDrain()) {
                std::exit(1);
            }

            // The first callback creates second_callback before it signals
            // completion. The second callback therefore did not contribute
            // to the active drain count. The first callback then decrements
            // that count to zero.
            first_event->process();

            // With the old constructor, second_callback inherited Draining.
            // Completing it called signalDrainDone() and tried to decrement
            // the already-zero count. A callback not counted by the active
            // pass must instead remain Running, making this signal a no-op.
            second_event->process();

            // A surviving callback would be visited and counted here.
            if (!DrainManager::instance().tryDrain()) {
                std::exit(2);
            }

            std::exit(0);
        },
        ::testing::ExitedWithCode(0), "");
}

} // anonymous namespace
} // namespace gem5
