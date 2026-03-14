#include "mem/cache/replacement_policies/spaib_mpki_rp.hh"

#include <memory>

#include "base/logging.hh"
#include "params/SPAIBMPKIRP.hh"
#include "sim/cur_tick.hh"

namespace gem5
{

namespace replacement_policy
{

SPAIBMPKI::SPAIBMPKI(const Params &p)
  : LRU(p), enableBypass(p.enable_bypass),
    mpkiThreshold(p.mpki_threshold),
    instructionWindow(p.instruction_window),
    missCount(0), baseInstCount(0), hasBaseInstCount(false),
    currentMpki(0.0), streamingPhase(false)
{
    fatal_if(instructionWindow == 0,
        "SPAIBMPKI requires instruction_window to be greater than zero.");
}

void
SPAIBMPKI::recordMiss(const PacketPtr pkt) const
{
    if (!pkt || !pkt->req || !pkt->req->hasInstCount()) {
        return;
    }

    const uint64_t instCount = pkt->req->getInstCount();

    if (!hasBaseInstCount) {
        baseInstCount = instCount;
        hasBaseInstCount = true;
    }

    missCount++;

    if (instCount < baseInstCount) {
        baseInstCount = instCount;
        missCount = 1;
        return;
    }

    const uint64_t instDelta = instCount - baseInstCount;

    if (instDelta >= instructionWindow && instDelta > 0) {
        currentMpki = (static_cast<double>(missCount) * 1000.0) /
            static_cast<double>(instDelta);
        streamingPhase = currentMpki >= mpkiThreshold;

        baseInstCount = instCount;
        missCount = 0;
    }
}

bool
SPAIBMPKI::shouldInsertNearLRU() const
{
    return streamingPhase && !shouldBypassFill();
}

bool
SPAIBMPKI::shouldBypassFill() const
{
    return enableBypass && streamingPhase;
}

bool
SPAIBMPKI::shouldBypass(const PacketPtr pkt) const
{
    recordMiss(pkt);
    return shouldBypassFill();
}

void
SPAIBMPKI::reset(const std::shared_ptr<ReplacementData>& replacement_data,
    const PacketPtr pkt)
{
    panic_if(!pkt, "SPAIBMPKI requires packet-aware reset for insertion.");

    auto data = std::static_pointer_cast<LRUReplData>(replacement_data);
    if (shouldInsertNearLRU()) {
        data->lastTouchTick = Tick(1);
    } else {
        data->lastTouchTick = curTick();
    }
}

void
SPAIBMPKI::reset(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    auto data = std::static_pointer_cast<LRUReplData>(replacement_data);
    if (shouldInsertNearLRU()) {
        data->lastTouchTick = Tick(1);
    } else {
        data->lastTouchTick = curTick();
    }
}

} // namespace replacement_policy
} // namespace gem5
