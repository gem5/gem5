#include "mem/cache/replacement_policies/spaib_rp.hh"

#include <memory>

#include "base/logging.hh"
#include "params/SPAIBRP.hh"
#include "sim/cur_tick.hh"

namespace gem5
{

namespace replacement_policy
{

SPAIB::SPAIB(const Params &p)
  : LRU(p), historyLength(p.history_length),
    deadThreshold(p.dead_threshold / 100.0), recentOutcomes(),
    deadCount(0), streamingPhase(false)
{
    fatal_if(historyLength == 0,
        "SPAIB requires history_length to be greater than zero.");
}

void
SPAIB::recordOutcome(bool dead) const
{
    recentOutcomes.push_back(dead);
    deadCount += dead ? 1 : 0;

    if (recentOutcomes.size() > historyLength) {
        deadCount -= recentOutcomes.front() ? 1 : 0;
        recentOutcomes.pop_front();
    }

    if (recentOutcomes.size() == historyLength) {
        const double deadRate =
            static_cast<double>(deadCount) / recentOutcomes.size();
        streamingPhase = deadRate >= deadThreshold;
    }
}

bool
SPAIB::shouldInsertNearLRU() const
{
    return streamingPhase && recentOutcomes.size() == historyLength;
}

void
SPAIB::invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
{
    auto data = std::static_pointer_cast<SPAIBReplData>(replacement_data);

    if (data->lastTouchTick != Tick(0) && !data->reused) {
        recordOutcome(true);
    }

    data->reused = false;
    LRU::invalidate(replacement_data);
}

void
SPAIB::touch(const std::shared_ptr<ReplacementData>& replacement_data,
    const PacketPtr pkt)
{
    auto data = std::static_pointer_cast<SPAIBReplData>(replacement_data);

    if (!data->reused) {
        recordOutcome(false);
        data->reused = true;
    }

    LRU::touch(replacement_data);
}

void
SPAIB::touch(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    auto data = std::static_pointer_cast<SPAIBReplData>(replacement_data);

    if (!data->reused) {
        recordOutcome(false);
        data->reused = true;
    }

    LRU::touch(replacement_data);
}

void
SPAIB::reset(const std::shared_ptr<ReplacementData>& replacement_data,
    const PacketPtr pkt)
{
    auto data = std::static_pointer_cast<SPAIBReplData>(replacement_data);
    panic_if(!pkt, "SPAIB requires packet-aware reset for insertion.");

    data->reused = false;
    if (shouldInsertNearLRU()) {
        data->lastTouchTick = Tick(1);
    } else {
        data->lastTouchTick = curTick();
    }
}

void
SPAIB::reset(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    auto data = std::static_pointer_cast<SPAIBReplData>(replacement_data);

    data->reused = false;
    if (shouldInsertNearLRU()) {
        data->lastTouchTick = Tick(1);
    } else {
        data->lastTouchTick = curTick();
    }
}

std::shared_ptr<ReplacementData>
SPAIB::instantiateEntry()
{
    return std::shared_ptr<ReplacementData>(new SPAIBReplData());
}

} // namespace replacement_policy
} // namespace gem5
