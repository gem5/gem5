#ifndef __MEM_CACHE_REPLACEMENT_POLICIES_SPAIB_RP_HH__
#define __MEM_CACHE_REPLACEMENT_POLICIES_SPAIB_RP_HH__

#include <deque>

#include "mem/cache/replacement_policies/lru_rp.hh"
#include "mem/packet.hh"

namespace gem5
{

struct SPAIBRPParams;

namespace replacement_policy
{

class SPAIB : public LRU
{
  protected:
    struct SPAIBReplData : LRUReplData
    {
        bool reused;

        SPAIBReplData() : LRUReplData(), reused(false) {}
    };

    const unsigned historyLength;
    const double deadThreshold;
    mutable std::deque<bool> recentOutcomes;
    mutable unsigned deadCount;
    mutable bool streamingPhase;

    void recordOutcome(bool dead) const;
    bool shouldInsertNearLRU() const;

  public:
    typedef SPAIBRPParams Params;
    SPAIB(const Params &p);
    ~SPAIB() = default;

    void invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
        override;

    void touch(const std::shared_ptr<ReplacementData>& replacement_data,
        const PacketPtr pkt) override;
    void touch(const std::shared_ptr<ReplacementData>& replacement_data) const
        override;

    void reset(const std::shared_ptr<ReplacementData>& replacement_data,
        const PacketPtr pkt) override;
    void reset(const std::shared_ptr<ReplacementData>& replacement_data) const
        override;

    std::shared_ptr<ReplacementData> instantiateEntry() override;
};

} // namespace replacement_policy
} // namespace gem5

#endif // __MEM_CACHE_REPLACEMENT_POLICIES_SPAIB_RP_HH__
