#ifndef __MEM_CACHE_REPLACEMENT_POLICIES_SPAIB_MPKI_RP_HH__
#define __MEM_CACHE_REPLACEMENT_POLICIES_SPAIB_MPKI_RP_HH__

#include "mem/cache/replacement_policies/lru_rp.hh"
#include "mem/packet.hh"

namespace gem5
{

struct SPAIBMPKIRPParams;

namespace replacement_policy
{

class SPAIBMPKI : public LRU
{
  protected:
    const bool enableBypass;
    const double mpkiThreshold;
    const uint64_t instructionWindow;

    mutable uint64_t missCount;
    mutable uint64_t baseInstCount;
    mutable bool hasBaseInstCount;
    mutable double currentMpki;
    mutable bool streamingPhase;

    void recordMiss(const PacketPtr pkt) const;
    bool shouldInsertNearLRU() const;
    bool shouldBypassFill() const;

  public:
    typedef SPAIBMPKIRPParams Params;
    SPAIBMPKI(const Params &p);
    ~SPAIBMPKI() = default;

    void reset(const std::shared_ptr<ReplacementData>& replacement_data,
        const PacketPtr pkt) override;
    void reset(const std::shared_ptr<ReplacementData>& replacement_data) const
        override;

    bool shouldBypass(const PacketPtr pkt) const override;
};

} // namespace replacement_policy
} // namespace gem5

#endif // __MEM_CACHE_REPLACEMENT_POLICIES_SPAIB_MPKI_RP_HH__
