#include <gtest/gtest.h>

#include <memory>

#include "mem/cache/replacement_policies/spaib_mpki_rp.hh"
#include "mem/request.hh"
#include "params/SPAIBMPKIRP.hh"

namespace
{

gem5::EventQueue eventQueue("SPAIBMPKITest Queue");

class SPAIBMPKIRPTestF : public ::testing::Test
{
  protected:
    std::shared_ptr<gem5::replacement_policy::SPAIBMPKI>
    makePolicy(bool enable_bypass, double mpki_threshold,
               uint64_t instruction_window)
    {
        gem5::SPAIBMPKIRPParams params;
        params.eventq_index = 0;
        params.enable_bypass = enable_bypass;
        params.mpki_threshold = mpki_threshold;
        params.instruction_window = instruction_window;
        return std::make_shared<gem5::replacement_policy::SPAIBMPKI>(params);
    }

    std::unique_ptr<gem5::Packet>
    makePacket(gem5::Addr addr, uint64_t inst_count)
    {
        auto req = std::make_shared<gem5::Request>(addr, 64, 0, 0);
        req->setInstCount(inst_count);
        return std::make_unique<gem5::Packet>(req, gem5::MemCmd::ReadReq);
    }

    void
    tick()
    {
        eventQueue.setCurTick(gem5::curTick() + 1);
    }

    void SetUp() override
    {
        gem5::curEventQueue(&eventQueue);
        eventQueue.setCurTick(1);
    }
};

TEST_F(SPAIBMPKIRPTestF, BypassEnabledAfterHighMpki)
{
    auto rp = makePolicy(true, 10.0, 100);

    auto pkt1 = makePacket(0x1000, 100);
    auto pkt2 = makePacket(0x2000, 200);

    ASSERT_FALSE(rp->shouldBypass(pkt1.get()));
    ASSERT_TRUE(rp->shouldBypass(pkt2.get()));
}

TEST_F(SPAIBMPKIRPTestF, BypassClearsAfterLowMpki)
{
    auto rp = makePolicy(true, 10.0, 100);

    auto pkt1 = makePacket(0x1000, 100);
    auto pkt2 = makePacket(0x2000, 200);
    auto pkt3 = makePacket(0x3000, 1200);

    ASSERT_FALSE(rp->shouldBypass(pkt1.get()));
    ASSERT_TRUE(rp->shouldBypass(pkt2.get()));
    ASSERT_FALSE(rp->shouldBypass(pkt3.get()));
}

TEST_F(SPAIBMPKIRPTestF, StreamingInsertionMovesNewLineTowardLRU)
{
    auto rp = makePolicy(false, 10.0, 100);

    auto pkt1 = makePacket(0x1000, 100);
    auto pkt2 = makePacket(0x2000, 200);

    ASSERT_FALSE(rp->shouldBypass(pkt1.get()));
    ASSERT_FALSE(rp->shouldBypass(pkt2.get()));

    gem5::ReplaceableEntry older_entry;
    gem5::ReplaceableEntry new_entry;
    older_entry.replacementData = rp->instantiateEntry();
    new_entry.replacementData = rp->instantiateEntry();

    rp->reset(older_entry.replacementData, pkt2.get());
    tick();
    rp->touch(older_entry.replacementData);
    tick();
    rp->reset(new_entry.replacementData, pkt2.get());

    gem5::ReplacementCandidates candidates;
    candidates.push_back(&older_entry);
    candidates.push_back(&new_entry);

    ASSERT_EQ(rp->getVictim(candidates), &new_entry);
}

TEST_F(SPAIBMPKIRPTestF, NonStreamingInsertionKeepsNewLineAwayFromLRU)
{
    auto rp = makePolicy(false, 10.0, 100);

    auto pkt1 = makePacket(0x1000, 100);
    auto pkt2 = makePacket(0x2000, 2100);

    ASSERT_FALSE(rp->shouldBypass(pkt1.get()));
    ASSERT_FALSE(rp->shouldBypass(pkt2.get()));

    gem5::ReplaceableEntry older_entry;
    gem5::ReplaceableEntry new_entry;
    older_entry.replacementData = rp->instantiateEntry();
    new_entry.replacementData = rp->instantiateEntry();

    rp->reset(older_entry.replacementData, pkt2.get());
    tick();
    rp->touch(older_entry.replacementData);
    tick();
    rp->reset(new_entry.replacementData, pkt2.get());

    gem5::ReplacementCandidates candidates;
    candidates.push_back(&older_entry);
    candidates.push_back(&new_entry);

    ASSERT_EQ(rp->getVictim(candidates), &older_entry);
}

} // anonymous namespace
