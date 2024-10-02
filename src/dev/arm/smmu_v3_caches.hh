/*
 * Copyright (c) 2014, 2018-2019, 2021 Arm Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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

#ifndef __DEV_ARM_SMMU_V3_CACHES_HH__
#define __DEV_ARM_SMMU_V3_CACHES_HH__

#include <stdint.h>

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "base/random.hh"
#include "base/statistics.hh"
#include "base/types.hh"

#define WALK_CACHE_LEVELS 4

namespace gem5
{

enum
{
    SMMU_CACHE_REPL_ROUND_ROBIN,
    SMMU_CACHE_REPL_RANDOM,
    SMMU_CACHE_REPL_LRU,
};

class SMMUv3BaseCache
{
  protected:
    int replacementPolicy;
    size_t nextToReplace;
    Random::RandomPtr random;
    uint32_t useStamp;

    struct SMMUv3BaseCacheStats : public statistics::Group
    {
        SMMUv3BaseCacheStats(statistics::Group *parent,
            const std::string &name);

        statistics::Formula averageLookups;
        statistics::Scalar totalLookups;

        statistics::Formula averageMisses;
        statistics::Scalar totalMisses;

        statistics::Formula averageUpdates;
        statistics::Scalar totalUpdates;

        statistics::Formula averageHitRate;

        statistics::Scalar insertions;
    } baseCacheStats;

    static int decodePolicyName(const std::string &policy_name);

  public:
    SMMUv3BaseCache(const std::string &policy_name, uint32_t seed,
                    statistics::Group *parent, const std::string &name);
    virtual ~SMMUv3BaseCache() {}
};

class SMMUTLB : public SMMUv3BaseCache
{
  public:
    enum AllocPolicy
    {
        ALLOC_ANY_WAY,
        ALLOC_ANY_BUT_LAST_WAY,
        ALLOC_LAST_WAY,
    };

    struct Entry
    {
        bool valid;
        bool prefetched;
        mutable uint32_t lastUsed;

        // TAGS
        uint32_t sid;
        uint32_t ssid;
        Addr va;
        Addr vaMask;

        // EXTRA TAGS
        uint16_t asid;
        uint16_t vmid;

        // OUTPUTS
        Addr pa;
        uint8_t permissions;
    };

    SMMUTLB(unsigned numEntries, unsigned _associativity,
            const std::string &policy, statistics::Group *parent,
            const std::string &name);
    SMMUTLB(const SMMUTLB& tlb) = delete;
    virtual ~SMMUTLB() {}

    const Entry *lookup(uint32_t sid, uint32_t ssid, Addr va,
                        bool updStats=true);
    const Entry *lookupAnyVA(uint32_t sid, uint32_t ssid,
                             bool updStats=true);
    void store(const Entry &incoming, AllocPolicy alloc);

    void invalidateSSID(uint32_t sid, uint32_t ssid);
    void invalidateSID(uint32_t sid);
    void invalidateVA(Addr va, uint16_t asid, uint16_t vmid);
    void invalidateVAA(Addr va, uint16_t vmid);
    void invalidateASID(uint16_t asid, uint16_t vmid);
    void invalidateVMID(uint16_t vmid);
    void invalidateAll();

  private:
    typedef std::vector<Entry> Set;
    std::vector<Set> sets;

    size_t associativity;

    size_t pickSetIdx(uint32_t sid, uint32_t ssid) const;
    size_t pickSetIdx(Addr va) const;
    size_t pickEntryIdxToReplace(const Set &set, AllocPolicy alloc);
};

class ARMArchTLB : public SMMUv3BaseCache
{
  public:
    struct Entry
    {
        bool valid;
        mutable uint32_t lastUsed;

        // TAGS
        Addr va;
        Addr vaMask;
        uint16_t asid;
        uint16_t vmid;

        // OUTPUTS
        Addr pa;
        uint8_t permissions;
    };

    ARMArchTLB(unsigned numEntries, unsigned _associativity,
               const std::string &policy, statistics::Group *parent);
    virtual ~ARMArchTLB() {}

    const Entry *lookup(Addr va, uint16_t asid, uint16_t vmid,
                        bool updStats=true);

    void store(const Entry &incoming);

    void invalidateVA(Addr va, uint16_t asid, uint16_t vmid);
    void invalidateVAA(Addr va, uint16_t vmid);
    void invalidateASID(uint16_t asid, uint16_t vmid);
    void invalidateVMID(uint16_t vmid);
    void invalidateAll();

  private:
    typedef std::vector<Entry> Set;
    std::vector<Set> sets;

    size_t associativity;

    size_t pickSetIdx(Addr va, uint16_t asid, uint16_t vmid) const;
    size_t pickEntryIdxToReplace(const Set &set);
};

class IPACache : public SMMUv3BaseCache
{
  public:
    struct Entry
    {
        bool valid;
        mutable uint32_t lastUsed;

        // TAGS
        Addr ipa;
        Addr ipaMask;
        uint16_t vmid;

        // OUTPUTS
        Addr pa;
        uint8_t permissions;
    };

    IPACache(unsigned numEntries, unsigned _associativity,
             const std::string &policy, statistics::Group *parent);
    virtual ~IPACache() {}

    const Entry *lookup(Addr ipa, uint16_t vmid, bool updStats=true);
    void store(const Entry &incoming);

    void invalidateIPA(Addr ipa, uint16_t vmid);
    void invalidateIPAA(Addr ipa);
    void invalidateVMID(uint16_t vmid);
    void invalidateAll();

  private:
    typedef std::vector<Entry> Set;
    std::vector<Set> sets;

    size_t associativity;

    size_t pickSetIdx(Addr ipa, uint16_t vmid) const;
    size_t pickEntryIdxToReplace(const Set &set);
};

class ConfigCache : public SMMUv3BaseCache
{
  public:
    struct Entry
    {
        bool valid;
        mutable uint32_t lastUsed;

        // TAGS
        uint32_t sid;
        uint32_t ssid;

        // OUTPUTS
        bool stage1_en;
        bool stage2_en;
        Addr ttb0;
        Addr ttb1;
        Addr httb;
        uint16_t asid;
        uint16_t vmid;
        uint8_t stage1_tg;
        uint8_t stage2_tg;
        uint8_t t0sz;
        uint8_t s2t0sz;
    };

    ConfigCache(unsigned numEntries, unsigned _associativity,
                const std::string &policy, statistics::Group *parent);
    virtual ~ConfigCache() {}

    const Entry *lookup(uint32_t sid, uint32_t ssid, bool updStats=true);
    void store(const Entry &incoming);

    void invalidateSSID(uint32_t sid, uint32_t ssid);
    void invalidateSID(uint32_t sid);
    void invalidateAll();

  private:
    typedef std::vector<Entry> Set;
    std::vector<Set> sets;

    size_t associativity;

    size_t pickSetIdx(uint32_t sid, uint32_t ssid) const;
    size_t pickEntryIdxToReplace(const Set &set);
};

class WalkCache : public SMMUv3BaseCache
{
  public:
    struct Entry
    {
        bool valid;
        mutable uint32_t lastUsed;

        // TAGS
        Addr va;
        Addr vaMask;
        uint16_t asid;
        uint16_t vmid;
        unsigned stage;
        unsigned level;

        // OUTPUTS
        bool leaf;
        Addr pa;
        uint8_t permissions;
    };

    WalkCache(const std::array<unsigned, 2*WALK_CACHE_LEVELS> &_sizes,
              unsigned _associativity, const std::string &policy,
              statistics::Group *parent);
    virtual ~WalkCache() {}

    const Entry *lookup(Addr va, Addr vaMask, uint16_t asid, uint16_t vmid,
                        unsigned stage, unsigned level, bool updStats=true);
    void store(const Entry &incoming);

    void invalidateVA(Addr va, uint16_t asid, uint16_t vmid,
                      const bool leaf_only);
    void invalidateVAA(Addr va, uint16_t vmid, const bool leaf_only);
    void invalidateASID(uint16_t asid, uint16_t vmid);
    void invalidateVMID(uint16_t vmid);
    void invalidateAll();

  protected:
    struct WalkCacheStats : public statistics::Group
    {
        WalkCacheStats(statistics::Group *parent);
        ~WalkCacheStats();

        std::vector<statistics::Formula*> averageLookupsByStageLevel;
        statistics::Vector2d totalLookupsByStageLevel;

        std::vector<statistics::Formula*> averageMissesByStageLevel;
        statistics::Vector2d totalMissesByStageLevel;

        std::vector<statistics::Formula*> averageUpdatesByStageLevel;
        statistics::Vector2d totalUpdatesByStageLevel;

        std::vector<statistics::Formula*> averageHitRateByStageLevel;

        statistics::Vector2d insertionsByStageLevel;
    } walkCacheStats;
  private:
    typedef std::vector<Entry> Set;
    std::vector<Set> sets;

    size_t associativity;
    std::array<unsigned, 2*WALK_CACHE_LEVELS> sizes;
    std::array<unsigned, 2*WALK_CACHE_LEVELS> offsets;

    size_t pickSetIdx(Addr va, Addr vaMask,
                      unsigned stage, unsigned level) const;

    size_t pickEntryIdxToReplace(const Set &set,
                                 unsigned stage, unsigned level);
};

} // namespace gem5

#endif /* __DEV_ARM_SMMU_V3_CACHES_HH__ */
