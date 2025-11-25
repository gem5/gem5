// Simple Way Guard Table (DAWG) for gem5
#ifndef __MEM_CACHE_DAWG_HH__
#define __MEM_CACHE_DAWG_HH__

#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

#include "sim/sim_object.hh"

#if __has_include("params/WayGuardTable.hh")
#include "params/WayGuardTable.hh"

#endif

namespace gem5 {

class WayGuardTable : public SimObject
{
  public:

  typedef WayGuardTableParams Params;
    WayGuardTable(const Params &p);

    // return the mask for a given set and domain: bit i corresponds to way i
    uint32_t getMask(uint32_t set, uint32_t domain) const;

    // set the mask for a given set and domain
    void setMask(uint32_t set, uint32_t domain, uint32_t mask);

    // set all masks for a domain to the provided mask value
    void setAllMasks(uint32_t domain, uint32_t mask);

  // return list of domains that currently have masks configured. */
  std::vector<uint32_t> getDomains() const;

  // register callback ( receives set,domain,mask ) to be invoked when a mask changes.
  typedef std::function<void(uint32_t, uint32_t, uint32_t)> MaskChangeCallback;
  void registerChangeCallback(MaskChangeCallback cb);

  private:
    const uint32_t numSets;
    const uint32_t numWays;

    // per-domain map to vector of masks, one entry per set
    std::unordered_map<uint32_t, std::vector<uint32_t>> masks;
    // registered callbacks invoked when masks change
    std::vector<MaskChangeCallback> callbacks;
};

} // namespace gem5

#endif // __MEM_CACHE_DAWG_HH__
