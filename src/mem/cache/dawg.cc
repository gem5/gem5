#include "mem/cache/dawg.hh"

#include "base/logging.hh"
#include "params/WayGuardTable.hh"

namespace gem5
{

    WayGuardTable::WayGuardTable(const Params &p)
        : SimObject(p), numSets(p.num_sets), numWays(p.num_ways)
    {
        // initialize default domain 0 with all-ones mask
        std::vector<uint32_t> default_masks(numSets, (numWays >= 32) ? 0xFFFFFFFFu : ((1u << numWays) - 1u));
        masks[0] = std::move(default_masks);

        // apply any initial masks provided via the Params struct.

        // set via python Param: 'VectorParam.String' with entires in the format "domain:mask"
        // mask is hex with delimiter '0x'

        try {
            for (const auto &s : p.initial_masks) {
                std::string entry = s;
                size_t col1 = entry.find(':');
                if (col1 == std::string::npos) {
                    warn("WayGuardTable: ignoring malformed initial_masks entry '%s'",
                        entry);
                    continue;
                }
                size_t col2 = entry.find(':', col1 + 1);
                if (col2 == std::string::npos) {
                    // format: domain:mask
                    std::string dom_str = entry.substr(0, col1);
                    std::string mask_str = entry.substr(col1 + 1);
                    uint32_t domain = static_cast<uint32_t>(std::stoul(dom_str, nullptr, 0));
                    uint32_t mask = static_cast<uint32_t>(std::stoul(mask_str, nullptr, 0));
                    setAllMasks(domain, mask);
                } else {
                    // format: set:domain:mask
                    std::string set_str = entry.substr(0, col1);
                    std::string dom_str = entry.substr(col1 + 1, col2 - (col1 + 1));
                    std::string mask_str = entry.substr(col2 + 1);
                    uint32_t set = static_cast<uint32_t>(std::stoul(set_str, nullptr, 0));
                    uint32_t domain = static_cast<uint32_t>(std::stoul(dom_str, nullptr, 0));
                    uint32_t mask = static_cast<uint32_t>(std::stoul(mask_str, nullptr, 0));
                    if (set >= numSets) {
                        warn("WayGuardTable: initial_masks set %u out of range (numSets=%u)", set, numSets);
                        continue;
                    }
                    setMask(set, domain, mask);
                }
            }
        } catch (...) {
            // if initial_masks isn't present or parsing failed, just do nothing to preserve compatibility.
        }
    }

    uint32_t WayGuardTable::getMask(uint32_t set, uint32_t domain) const
    {
        if (set >= numSets) {
            warn("WayGuardTable: set %u out of range (numSets=%u)", set, numSets);
            return (numWays >= 32) ? 0xFFFFFFFFu : ((1u << numWays) - 1u);
        }

        auto it = masks.find(domain);
        if (it == masks.end()) {
            // no mask for this domain; return all-ones mask (allow all ways)
            return (numWays >= 32) ? 0xFFFFFFFFu : ((1u << numWays) - 1u);
        }

        return it->second[set];
    }

    void WayGuardTable::setMask(uint32_t set, uint32_t domain, uint32_t mask)
    {
        if (set >= numSets) {
            panic("WayGuardTable::setMask: set %u out of range (numSets=%u)", set, numSets);
        }

        auto &vec = masks[domain];
        if (vec.empty())
            vec.assign(numSets, (numWays >= 32) ? 0xFFFFFFFFu : ((1u << numWays) - 1u));

        vec[set] = mask;
        if (mask == 0) {
            warn("WayGuardTable: set %u domain %u mask is zero (no allowed ways)", set, domain);
        }

        // notify callbacks
        for (auto &cb : callbacks) {
            cb(set, domain, mask);
        }
    }

    void WayGuardTable::setAllMasks(uint32_t domain, uint32_t mask)
    {
        auto &vec = masks[domain];
        vec.assign(numSets, mask);

        // notify callbacks for each set
        for (uint32_t set = 0; set < numSets; ++set) {
            for (auto &cb : callbacks) {
                cb(set, domain, mask);
            }
        }
    }

    std::vector<uint32_t> WayGuardTable::getDomains() const
    {
        std::vector<uint32_t> domains;
        domains.reserve(masks.size());
        for (const auto &kv : masks) {
            domains.push_back(kv.first);
        }
        return domains;
    }

    void WayGuardTable::registerChangeCallback(MaskChangeCallback cb)
    {
        callbacks.push_back(std::move(cb));
    }

} // namespace gem5
