#include <gtest/gtest.h>

#include <memory>

#include "mem/cache/replacement_policies/spaib_rp.hh"
#include "params/SPAIBRP.hh"

namespace
{

gem5::EventQueue eventQueue("SPAIBRPTest Queue");

class SPAIBRPTestF : public ::testing::Test
{
  protected:
    std::shared_ptr<gem5::replacement_policy::SPAIB>
    makePolicy(bool enable_bypass, int bypass_threshold, int dead_threshold,
               unsigned history_length)
    {
        gem5::SPAIBRPParams params;
        params.eventq_index = 0;
        params.enable_bypass = enable_bypass;
        params.bypass_threshold = bypass_threshold;
        params.dead_threshold = dead_threshold;
        params.history_length = history_length;
        return std::make_shared<gem5::replacement_policy::SPAIB>(params);
    }

    gem5::PacketPtr
    fakePacket()
    {
        return reinterpret_cast<gem5::PacketPtr>(0x1);
    }

    void
    tick()
    {
        eventQueue.setCurTick(gem5::curTick() + 1);
    }

    void
    recordDeadLine(const std::shared_ptr<gem5::replacement_policy::SPAIB>& rp)
    {
        auto repl_data = rp->instantiateEntry();
        const auto pkt = fakePacket();
        rp->reset(repl_data, pkt);
        tick();
        rp->invalidate(repl_data);
        tick();
    }

    void
    recordReusedLine(const std::shared_ptr<gem5::replacement_policy::SPAIB>& rp)
    {
        auto repl_data = rp->instantiateEntry();
        const auto pkt = fakePacket();
        rp->reset(repl_data, pkt);
        tick();
        rp->touch(repl_data, pkt);
        tick();
        rp->invalidate(repl_data);
        tick();
    }

    void SetUp() override
    {
        gem5::curEventQueue(&eventQueue);
        eventQueue.setCurTick(1);
    }
};

TEST_F(SPAIBRPTestF, BypassEnabledAfterDeadStreamingPhase)
{
    auto rp = makePolicy(true, 75, 50, 4);
    const auto pkt = fakePacket();

    for (int i = 0; i < 4; ++i) {
        recordDeadLine(rp);
    }

    ASSERT_TRUE(rp->shouldBypass(pkt));
}

TEST_F(SPAIBRPTestF, BypassClearsAfterReusePhase)
{
    auto rp = makePolicy(true, 75, 50, 4);
    const auto pkt = fakePacket();

    for (int i = 0; i < 4; ++i) {
        recordDeadLine(rp);
    }
    ASSERT_TRUE(rp->shouldBypass(pkt));

    for (int i = 0; i < 4; ++i) {
        recordReusedLine(rp);
    }

    ASSERT_FALSE(rp->shouldBypass(pkt));
}

TEST_F(SPAIBRPTestF, StreamingInsertionMovesNewLineTowardLRU)
{
    auto rp = makePolicy(false, 90, 50, 4);

    for (int i = 0; i < 4; ++i) {
        recordDeadLine(rp);
    }

    gem5::ReplaceableEntry older_entry;
    gem5::ReplaceableEntry new_entry;
    older_entry.replacementData = rp->instantiateEntry();
    new_entry.replacementData = rp->instantiateEntry();

    const auto pkt = fakePacket();

    rp->reset(older_entry.replacementData, pkt);
    tick();
    rp->touch(older_entry.replacementData, pkt);
    tick();

    rp->reset(new_entry.replacementData, pkt);

    gem5::ReplacementCandidates candidates;
    candidates.push_back(&older_entry);
    candidates.push_back(&new_entry);

    ASSERT_EQ(rp->getVictim(candidates), &new_entry);
}

TEST_F(SPAIBRPTestF, NonStreamingInsertionKeepsNewLineAwayFromLRU)
{
    auto rp = makePolicy(false, 90, 75, 4);

    gem5::ReplaceableEntry older_entry;
    gem5::ReplaceableEntry new_entry;
    older_entry.replacementData = rp->instantiateEntry();
    new_entry.replacementData = rp->instantiateEntry();

    const auto pkt = fakePacket();

    rp->reset(older_entry.replacementData, pkt);
    tick();
    rp->touch(older_entry.replacementData, pkt);
    tick();

    rp->reset(new_entry.replacementData, pkt);

    gem5::ReplacementCandidates candidates;
    candidates.push_back(&older_entry);
    candidates.push_back(&new_entry);

    ASSERT_EQ(rp->getVictim(candidates), &older_entry);
}

} // anonymous namespace
