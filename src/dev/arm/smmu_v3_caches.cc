/*
 * Copyright (c) 2014, 2018-2019 ARM Limited
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
 *
 * Authors: Stan Czerniawski
 *          Damian Richardson
 */

#include "dev/arm/smmu_v3_caches.hh"

#include <numeric>

#include "base/bitfield.hh"
#include "base/intmath.hh"
#include "base/logging.hh"
#include "sim/stats.hh"


// taken from hex expansion of pi
#define SMMUTLB_SEED     0xEA752DFE
#define ARMARCHTLB_SEED  0x8B021FA1
#define IPACACHE_SEED    0xE5A0CC0F
#define CONFIGCACHE_SEED 0xB56F74E8
#define WALKCACHE_SEED   0x18ACF3D6

/*
 * BaseCache
 *
 * TODO: move more code into this base class to reduce duplication.
 */

SMMUv3BaseCache::SMMUv3BaseCache(const std::string &policy_name, uint32_t seed) :
    replacementPolicy(decodePolicyName(policy_name)),
    nextToReplace(0),
    random(seed),
    useStamp(0)
{}

int
SMMUv3BaseCache::decodePolicyName(const std::string &policy_name)
{
    if (policy_name == "rr") {
        return SMMU_CACHE_REPL_ROUND_ROBIN;
    } else if (policy_name == "rand") {
        return SMMU_CACHE_REPL_RANDOM;
    } else if (policy_name == "lru") {
        return SMMU_CACHE_REPL_LRU;
    } else {
        panic("Unknown cache replacement policy '%s'\n", policy_name);
    }
}

void
SMMUv3BaseCache::regStats(const std::string &name)
{
    using namespace Stats;


    averageLookups
        .name(name + ".averageLookups")
        .desc("Average number lookups per second")
        .flags(pdf);

    totalLookups
        .name(name + ".totalLookups")
        .desc("Total number of lookups")
        .flags(pdf);

    averageLookups = totalLookups / simSeconds;


    averageMisses
        .name(name + ".averageMisses")
        .desc("Average number misses per second")
        .flags(pdf);

    totalMisses
        .name(name + ".totalMisses")
        .desc("Total number of misses")
        .flags(pdf);

    averageMisses = totalMisses / simSeconds;


    averageUpdates
        .name(name + ".averageUpdates")
        .desc("Average number updates per second")
        .flags(pdf);

    totalUpdates
        .name(name + ".totalUpdates")
        .desc("Total number of updates")
        .flags(pdf);

    averageUpdates = totalUpdates / simSeconds;


    averageHitRate
        .name(name + ".averageHitRate")
        .desc("Average hit rate")
        .flags(pdf);

    averageHitRate = (totalLookups - totalMisses) / totalLookups;

    insertions
        .name(name + ".insertions")
        .desc("Number of insertions (not replacements)")
        .flags(pdf);
}



/*
 * SMMUTLB
 */

SMMUTLB::SMMUTLB(unsigned numEntries, unsigned _associativity,
                 const std::string &policy)
:
    SMMUv3BaseCache(policy, SMMUTLB_SEED),
    associativity(_associativity)
{
    if (associativity == 0)
        associativity = numEntries; // fully associative

    if (numEntries == 0)
        fatal("SMMUTLB must have at least one entry\n");

    if (associativity > numEntries)
        fatal("SMMUTLB associativity cannot be higher than "
              "its number of entries\n");

    unsigned num_sets = numEntries / associativity;

    if (num_sets*associativity != numEntries)
        fatal("Number of SMMUTLB entries must be divisible "
              "by its associativity\n");

    Entry e;
    e.valid = false;

    Set set(associativity, e);
    sets.resize(num_sets, set);
}

const SMMUTLB::Entry*
SMMUTLB::lookup(uint32_t sid, uint32_t ssid,
                Addr va, bool updStats)
{
    const Entry *result = NULL;

    Set &set = sets[pickSetIdx(va)];

    for (size_t i = 0; i < set.size(); i++) {
        const Entry &e = set[i];

        if (e.valid && (e.va & e.vaMask) == (va & e.vaMask) &&
            e.sid==sid && e.ssid==ssid)
        {
            if (result != NULL)
                panic("SMMUTLB: duplicate entry found!\n");

            result = &e;
            break;
        }
    }

    if (updStats) {
        if (result)
            result->lastUsed = useStamp++;

        totalLookups++;
        if (result == NULL)
            totalMisses++;
    }

    return result;
}

const SMMUTLB::Entry*
SMMUTLB::lookupAnyVA(uint32_t sid, uint32_t ssid, bool updStats)
{
    const Entry *result = NULL;

    for (size_t s = 0; s < sets.size(); s++) {
        Set &set = sets[s];

        for (size_t i = 0; i < set.size(); i++) {
            const Entry &e = set[i];

            if (e.valid && e.sid==sid && e.ssid==ssid) {
                result = &e;
                break;
            }
        }
    }

    if (updStats) {
        totalLookups++;
        if (result == NULL)
            totalMisses++;
    }

    return result;
}

void
SMMUTLB::store(const Entry &incoming, AllocPolicy alloc)
{
    if (!incoming.valid)
        panic("Tried to store an invalid entry\n");

    incoming.lastUsed = 0;

    const Entry *existing =
        lookup(incoming.sid, incoming.ssid, incoming.va, false);

    if (existing) {
        *const_cast<Entry *> (existing) = incoming;
    } else {
        Set &set = sets[pickSetIdx(incoming.va)];
        set[pickEntryIdxToReplace(set, alloc)] = incoming;
    }

    totalUpdates++;
}

void
SMMUTLB::invalidateSSID(uint32_t sid, uint32_t ssid)
{
    Set &set = sets[pickSetIdx(sid, ssid)];

    for (size_t i = 0; i < set.size(); i++) {
        Entry &e = set[i];

        if (e.sid == sid && e.ssid == ssid)
            e.valid = false;
    }
}

void
SMMUTLB::invalidateSID(uint32_t sid)
{
    for (size_t s = 0; s < sets.size(); s++) {
        Set &set = sets[s];

        for (size_t i = 0; i < set.size(); i++) {
            Entry &e = set[i];

            if (e.sid == sid)
                e.valid = false;
        }
    }
}

void
SMMUTLB::invalidateVA(Addr va, uint16_t asid, uint16_t vmid)
{
    Set &set = sets[pickSetIdx(va)];

    for (size_t i = 0; i < set.size(); i++) {
        Entry &e = set[i];

        if ((e.va & e.vaMask) == (va & e.vaMask) &&
            e.asid==asid && e.vmid==vmid)
        {
            e.valid = false;
        }
    }
}

void
SMMUTLB::invalidateVAA(Addr va, uint16_t vmid)
{
    Set &set = sets[pickSetIdx(va)];

    for (size_t i = 0; i < set.size(); i++) {
        Entry &e = set[i];

        if ((e.va & e.vaMask) == (va & e.vaMask) && e.vmid==vmid)
            e.valid = false;
    }
}

void
SMMUTLB::invalidateASID(uint16_t asid, uint16_t vmid)
{
    for (size_t s = 0; s < sets.size(); s++) {
        Set &set = sets[s];

        for (size_t i = 0; i < set.size(); i++) {
            Entry &e = set[i];

            if (e.asid==asid && e.vmid==vmid)
                e.valid = false;
        }
    }
}

void
SMMUTLB::invalidateVMID(uint16_t vmid)
{
    for (size_t s = 0; s < sets.size(); s++) {
        Set &set = sets[s];

        for (size_t i = 0; i < set.size(); i++) {
            Entry &e = set[i];

            if (e.vmid == vmid)
                e.valid = false;
        }
    }
}

void
SMMUTLB::invalidateAll()
{
    for (size_t s = 0; s < sets.size(); s++) {
        Set &set = sets[s];

        for (size_t i = 0; i < set.size(); i++)
            set[i].valid = false;
    }
}

size_t
SMMUTLB::pickSetIdx(Addr va) const
{
    return (va >> 12) % sets.size();
}

size_t
SMMUTLB::pickSetIdx(uint32_t sid, uint32_t ssid) const
{
    return (sid^ssid) % sets.size();
}

size_t
SMMUTLB::pickEntryIdxToReplace(const Set &set, AllocPolicy alloc)
{
    if (alloc == ALLOC_LAST_WAY)
        return associativity - 1;

    uint32_t lru_tick = UINT32_MAX;
    size_t lru_idx = 0;
    size_t max_idx =
        alloc==ALLOC_ANY_BUT_LAST_WAY ?
            set.size()-1 : set.size();

    for (size_t i = 0; i < max_idx; i++) {
        if (!set[i].valid) {
            insertions++;
            return i;
        }

        if (set[i].lastUsed < lru_tick) {
            lru_idx = i;
            lru_tick = set[i].lastUsed;
        }
    }

    switch (replacementPolicy) {
    case SMMU_CACHE_REPL_ROUND_ROBIN:
        switch (alloc) {
        case ALLOC_ANY_WAY:
            return nextToReplace = ((nextToReplace+1) % associativity);
        case ALLOC_ANY_BUT_LAST_WAY:
            return nextToReplace = ((nextToReplace+1) % (associativity-1));
        default:
            panic("Unknown allocation mode %d\n", alloc);
        }

    case SMMU_CACHE_REPL_RANDOM:
        switch (alloc) {
        case ALLOC_ANY_WAY:
            return random.random<size_t>(0, associativity-1);
        case ALLOC_ANY_BUT_LAST_WAY:
            return random.random<size_t>(0, associativity-2);
        default:
            panic("Unknown allocation mode %d\n", alloc);
        }

    case SMMU_CACHE_REPL_LRU:
        return lru_idx;

    default:
        panic("Unknown replacement policy %d\n", replacementPolicy);
    }
}



/*
 * ARMArchTLB
 */

ARMArchTLB::ARMArchTLB(unsigned numEntries, unsigned _associativity,
                       const std::string &policy)
:
    SMMUv3BaseCache(policy, ARMARCHTLB_SEED),
    associativity(_associativity)
{
    if (associativity == 0)
        associativity = numEntries; // fully associative

    if (numEntries == 0)
        fatal("ARMArchTLB must have at least one entry\n");

    if (associativity > numEntries)
        fatal("ARMArchTLB associativity cannot be higher than "
              "its number of entries\n");

    unsigned num_sets = numEntries / associativity;

    if (num_sets*associativity != numEntries)
        fatal("Number of ARMArchTLB entries must be divisible "
              "by its associativity\n");

    Entry e;
    e.valid = false;

    Set set(associativity, e);
    sets.resize(num_sets, set);
}

const ARMArchTLB::Entry *
ARMArchTLB::lookup(Addr va, uint16_t asid, uint16_t vmid, bool updStats)
{
    const Entry *result = NULL;

    Set &set = sets[pickSetIdx(va, asid, vmid)];

    for (size_t i = 0; i < set.size(); i++) {
        const Entry &e = set[i];

        if (e.valid && (e.va & e.vaMask) == (va & e.vaMask) &&
            e.asid==asid && e.vmid==vmid)
        {
            if (result != NULL)
                panic("ARMArchTLB: duplicate entry found!\n");

            result = &e;
            break;
        }
    }

    if (updStats) {
        if (result)
            result->lastUsed = useStamp++;

        totalLookups++;
        if (result == NULL)
            totalMisses++;
    }

    return result;
}

void
ARMArchTLB::store(const Entry &incoming)
{
    if (!incoming.valid)
        panic("Tried to store an invalid entry\n");

    incoming.lastUsed = 0;

    const Entry *existing =
        lookup(incoming.va, incoming.asid, incoming.vmid, false);

    if (existing) {
        *const_cast<Entry *> (existing) = incoming;
    } else {
        Set &set = sets[pickSetIdx(incoming.va, incoming.asid, incoming.vmid)];
        set[pickEntryIdxToReplace(set)] = incoming;
    }

    totalUpdates++;
}

void
ARMArchTLB::invalidateVA(Addr va, uint16_t asid, uint16_t vmid)
{
    Set &set = sets[pickSetIdx(va, asid, vmid)];

    for (size_t i = 0; i < set.size(); i++) {
        Entry &e = set[i];

        if ((e.va & e.vaMask) == (va & e.vaMask) &&
            e.asid==asid && e.vmid==vmid)
        {
            e.valid = false;
        }
    }
}

void
ARMArchTLB::invalidateVAA(Addr va, uint16_t vmid)
{
    for (size_t s = 0; s < sets.size(); s++) {
        Set &set = sets[s];

        for (size_t i = 0; i < set.size(); i++) {
            Entry &e = set[i];

            if ((e.va & e.vaMask) == (va & e.vaMask) && e.vmid==vmid)
                e.valid = false;
        }
    }
}

void
ARMArchTLB::invalidateASID(uint16_t asid, uint16_t vmid)
{
    for (size_t s = 0; s < sets.size(); s++) {
        Set &set = sets[s];

        for (size_t i = 0; i < set.size(); i++) {
            Entry &e = set[i];

            if (e.asid==asid && e.vmid==vmid)
                e.valid = false;
        }
    }
}

void
ARMArchTLB::invalidateVMID(uint16_t vmid)
{
    for (size_t s = 0; s < sets.size(); s++) {
        Set &set = sets[s];

        for (size_t i = 0; i < set.size(); i++) {
            Entry &e = set[i];

            if (e.vmid == vmid)
                e.valid = false;
        }
    }
}

void
ARMArchTLB::invalidateAll()
{
    for (size_t s = 0; s < sets.size(); s++) {
        Set &set = sets[s];

        for (size_t i = 0; i < set.size(); i++)
            set[i].valid = false;
    }
}

size_t
ARMArchTLB::pickSetIdx(Addr va, uint16_t asid, uint16_t vmid) const
{
    return ((va >> 12) ^ asid ^ vmid) % sets.size();
}

size_t
ARMArchTLB::pickEntryIdxToReplace(const Set &set)
{
    size_t lru_idx = 0;
    uint32_t lru_tick = UINT32_MAX;

    for (size_t i = 0; i < set.size(); i++) {
        if (!set[i].valid) {
            insertions++;
            return i;
        }

        if (set[i].lastUsed < lru_tick) {
            lru_idx = i;
            lru_tick = set[i].lastUsed;
        }
    }

    switch (replacementPolicy) {
    case SMMU_CACHE_REPL_ROUND_ROBIN:
        return nextToReplace = ((nextToReplace+1) % associativity);

    case SMMU_CACHE_REPL_RANDOM:
        return random.random<size_t>(0, associativity-1);

    case SMMU_CACHE_REPL_LRU:
        return lru_idx;

    default:
        panic("Unknown replacement policy %d\n", replacementPolicy);
    }

}

/*
 * IPACache
 */

IPACache::IPACache(unsigned numEntries, unsigned _associativity,
                   const std::string &policy)
:
    SMMUv3BaseCache(policy, IPACACHE_SEED),
    associativity(_associativity)
{
    if (associativity == 0)
        associativity = numEntries; // fully associative

    if (numEntries == 0)
        fatal("IPACache must have at least one entry\n");

    if (associativity > numEntries)
        fatal("IPACache associativity cannot be higher than "
              "its number of entries\n");

    unsigned num_sets = numEntries / associativity;

    if (num_sets*associativity != numEntries)
        fatal("Number of IPACache entries must be divisible "
              "by its associativity\n");

    Entry e;
    e.valid = false;

    Set set(associativity, e);
    sets.resize(num_sets, set);
}

const IPACache::Entry*
IPACache::lookup(Addr ipa, uint16_t vmid, bool updStats)
{
    const Entry *result = NULL;

    Set &set = sets[pickSetIdx(ipa, vmid)];

    for (size_t i = 0; i < set.size(); i++) {
        const Entry &e = set[i];

        if (e.valid && (e.ipa & e.ipaMask) == (ipa & e.ipaMask) &&
            e.vmid==vmid)
        {
            if (result != NULL)
                panic("IPACache: duplicate entry found!\n");

            result = &e;
            break;
        }
    }

    if (updStats) {
        if (result)
            result->lastUsed = useStamp++;

        totalLookups++;
        if (result == NULL)
            totalMisses++;
    }

    return result;
}

void
IPACache::store(const Entry &incoming)
{
    if (!incoming.valid)
        panic("Tried to store an invalid entry\n");

    incoming.lastUsed = 0;

    const Entry *existing = lookup(incoming.ipa, incoming.vmid, false);

    if (existing) {
        *const_cast<Entry *> (existing) = incoming;
    } else {
        Set &set = sets[pickSetIdx(incoming.ipa, incoming.vmid)];
        set[pickEntryIdxToReplace(set)] = incoming;
    }

    totalUpdates++;
}

void
IPACache::invalidateIPA(Addr ipa, uint16_t vmid)
{
    Set &set = sets[pickSetIdx(ipa, vmid)];

    for (size_t i = 0; i < set.size(); i++) {
        Entry &e = set[i];

        if ((e.ipa & e.ipaMask) == (ipa & e.ipaMask) && e.vmid==vmid)
            e.valid = false;
    }
}

void
IPACache::invalidateIPAA(Addr ipa)
{
    for (size_t s = 0; s < sets.size(); s++) {
        Set &set = sets[s];

        for (size_t i = 0; i < set.size(); i++) {
            Entry &e = set[i];

            if ((e.ipa & e.ipaMask) == (ipa & e.ipaMask))
                e.valid = false;
        }
    }
}

void
IPACache::invalidateVMID(uint16_t vmid)
{
    for (size_t s = 0; s < sets.size(); s++) {
        Set &set = sets[s];

        for (size_t i = 0; i < set.size(); i++) {
            Entry &e = set[i];

            if (e.vmid == vmid)
                e.valid = false;
        }
    }
}

void
IPACache::invalidateAll()
{
    for (size_t s = 0; s < sets.size(); s++) {
        Set &set = sets[s];

        for (size_t i = 0; i < set.size(); i++)
            set[i].valid = false;
    }
}

size_t
IPACache::pickSetIdx(Addr va, uint16_t vmid) const
{
    return ((va >> 12) ^ vmid) % sets.size();
}

size_t
IPACache::pickEntryIdxToReplace(const Set &set)
{
    size_t lru_idx = 0;
    uint32_t lru_tick = UINT32_MAX;

    for (size_t i = 0; i < set.size(); i++) {
        if (!set[i].valid) {
            insertions++;
            return i;
        }

        if (set[i].lastUsed < lru_tick) {
            lru_idx = i;
            lru_tick = set[i].lastUsed;
        }
    }

    switch (replacementPolicy) {
    case SMMU_CACHE_REPL_ROUND_ROBIN:
        return nextToReplace = ((nextToReplace+1) % associativity);

    case SMMU_CACHE_REPL_RANDOM:
        return random.random<size_t>(0, associativity-1);

    case SMMU_CACHE_REPL_LRU:
        return lru_idx;

    default:
        panic("Unknown replacement policy %d\n", replacementPolicy);
    }

}

/*
 * ConfigCache
 */

ConfigCache::ConfigCache(unsigned numEntries, unsigned _associativity,
                         const std::string &policy)
:
    SMMUv3BaseCache(policy, CONFIGCACHE_SEED),
    associativity(_associativity)
{
    if (associativity == 0)
        associativity = numEntries; // fully associative

    if (numEntries == 0)
        fatal("ConfigCache must have at least one entry\n");

    if (associativity > numEntries)
        fatal("ConfigCache associativity cannot be higher than "
              "its number of entries\n");

    unsigned num_sets = numEntries / associativity;

    if (num_sets*associativity != numEntries)
        fatal("Number of ConfigCache entries must be divisible "
              "by its associativity\n");

    Entry e;
    e.valid = false;

    Set set(associativity, e);
    sets.resize(num_sets, set);
}

const ConfigCache::Entry *
ConfigCache::lookup(uint32_t sid, uint32_t ssid, bool updStats)
{
    const Entry *result = NULL;

    Set &set = sets[pickSetIdx(sid, ssid)];

    for (size_t i = 0; i < set.size(); i++) {
        const Entry &e = set[i];

        if (e.valid && e.sid==sid && e.ssid==ssid)
        {
            if (result != NULL)
                panic("ConfigCache: duplicate entry found!\n");

            result = &e;
            break;
        }
    }

    if (updStats) {
        if (result)
            result->lastUsed = useStamp++;

        totalLookups++;
        if (result == NULL)
            totalMisses++;
    }

    return result;
}

void
ConfigCache::store(const Entry &incoming)
{
    if (!incoming.valid)
        panic("Tried to store an invalid entry\n");

    incoming.lastUsed = 0;

    const Entry *existing = lookup(incoming.sid, incoming.ssid, false);

    if (existing) {
        *const_cast<Entry *> (existing) = incoming;
    } else {
        Set &set = sets[pickSetIdx(incoming.sid, incoming.ssid)];
        set[pickEntryIdxToReplace(set)] = incoming;
    }

    totalUpdates++;
}

void
ConfigCache::invalidateSSID(uint32_t sid, uint32_t ssid)
{
    Set &set = sets[pickSetIdx(sid, ssid)];

    for (size_t i = 0; i < set.size(); i++) {
        Entry &e = set[i];

        if (e.sid==sid && e.ssid==ssid)
            e.valid = false;
    }
}

void
ConfigCache::invalidateSID(uint32_t sid)
{
    for (size_t s = 0; s < sets.size(); s++) {
        Set &set = sets[s];

        for (size_t i = 0; i < set.size(); i++) {
            Entry &e = set[i];

            if (e.sid == sid)
                e.valid = false;
        }
    }
}

void
ConfigCache::invalidateAll()
{
    for (size_t s = 0; s < sets.size(); s++) {
        Set &set = sets[s];

        for (size_t i = 0; i < set.size(); i++)
            set[i].valid = false;
    }
}

size_t
ConfigCache::pickSetIdx(uint32_t sid, uint32_t ssid) const
{
    return (sid^ssid) % sets.size();
}

size_t
ConfigCache::pickEntryIdxToReplace(const Set &set)
{
    size_t lru_idx = 0;
    uint32_t lru_tick = UINT32_MAX;

    for (size_t i = 0; i < set.size(); i++) {
        if (!set[i].valid) {
            insertions++;
            return i;
        }

        if (set[i].lastUsed < lru_tick) {
            lru_idx = i;
            lru_tick = set[i].lastUsed;
        }
    }

    switch (replacementPolicy) {
    case SMMU_CACHE_REPL_ROUND_ROBIN:
        return nextToReplace = ((nextToReplace+1) % associativity);

    case SMMU_CACHE_REPL_RANDOM:
        return random.random<size_t>(0, associativity-1);

    case SMMU_CACHE_REPL_LRU:
        return lru_idx;

    default:
        panic("Unknown replacement policy %d\n", replacementPolicy);
    }

}

/*
 * WalkCache
 */

WalkCache::WalkCache(const std::array<unsigned, 2*WALK_CACHE_LEVELS> &_sizes,
                     unsigned _associativity, const std::string &policy) :
    SMMUv3BaseCache(policy, WALKCACHE_SEED),
    associativity(_associativity),
    sizes()
{
    unsigned numEntries = std::accumulate(&_sizes[0],
                                          &_sizes[2*WALK_CACHE_LEVELS], 0);

    if (associativity == 0)
        associativity = numEntries; // fully associative

    if (numEntries == 0)
        fatal("WalkCache must have at least one entry\n");

    for (size_t i = 0; i < 2*WALK_CACHE_LEVELS; i++){
        if (_sizes[i] % associativity != 0)
              fatal("Number of WalkCache entries at each level must be "
                    "divisible by WalkCache associativity\n");

        sizes[i] = _sizes[i] /  associativity;
        offsets[i] = i==0 ? 0 : offsets[i-1] + sizes[i-1];
    }

    if (associativity > numEntries)
        fatal("WalkCache associativity cannot be higher than "
              "its number of entries\n");

    unsigned num_sets = numEntries / associativity;

    if (num_sets*associativity != numEntries)
        fatal("Number of WalkCache entries must be divisible "
              "by its associativity\n");

    Entry e;
    e.valid = false;

    Set set(associativity, e);
    sets.resize(num_sets, set);
}

const WalkCache::Entry*
WalkCache::lookup(Addr va, Addr vaMask,
                  uint16_t asid, uint16_t vmid,
                  unsigned stage, unsigned level,
                  bool updStats)
{
    const Entry *result = NULL;

    Set &set = sets[pickSetIdx(va, vaMask, stage, level)];

    for (size_t i = 0; i < set.size(); i++) {
        const Entry &e = set[i];

        if (e.valid && (e.va & e.vaMask) == (va & e.vaMask) &&
            e.asid==asid && e.vmid==vmid && e.stage==stage && e.level==level)
        {
            if (result != NULL)
                panic("WalkCache: duplicate entry found!\n");

            result = &e;
            break;
        }
    }

    if (updStats) {
        if (result)
            result->lastUsed = useStamp++;

        totalLookups++;
        if (result == NULL)
            totalMisses++;

        lookupsByStageLevel[stage-1][level]++;
        totalLookupsByStageLevel[stage-1][level]++;
        if (result == NULL) {
            missesByStageLevel[stage-1][level]++;
            totalMissesByStageLevel[stage-1][level]++;
        }
    }

    return result;
}

void
WalkCache::store(const Entry &incoming)
{
    if (!incoming.valid)
        panic("Tried to store an invalid entry\n");

    assert(incoming.stage==1 || incoming.stage==2);
    assert(incoming.level<=WALK_CACHE_LEVELS);

    incoming.lastUsed = 0;

    const Entry *existing = lookup(incoming.va, incoming.vaMask,
                                   incoming.asid, incoming.vmid,
                                   incoming.stage, incoming.level, false);

    if (existing) {
        *const_cast<Entry *> (existing) = incoming;
    } else {
        Set &set = sets[pickSetIdx(incoming.va, incoming.vaMask,
                                   incoming.stage, incoming.level)];
        set[pickEntryIdxToReplace(set, incoming.stage, incoming.level)] =
            incoming;
    }

    totalUpdates++;
    updatesByStageLevel[incoming.stage-1][incoming.level]++;
    totalUpdatesByStageLevel[incoming.stage-1][incoming.level]++;
}

void
WalkCache::invalidateVA(Addr va, uint16_t asid, uint16_t vmid)
{
    panic("%s unimplemented\n", __func__);
}

void
WalkCache::invalidateVAA(Addr va, uint16_t vmid)
{
    panic("%s unimplemented\n", __func__);
}

void
WalkCache::invalidateASID(uint16_t asid, uint16_t vmid)
{
    panic("%s unimplemented\n", __func__);
}

void
WalkCache::invalidateVMID(uint16_t vmid)
{
    for (size_t s = 0; s < sets.size(); s++) {
        Set &set = sets[s];

        for (size_t i = 0; i < set.size(); i++) {
            Entry &e = set[i];

            if (e.vmid == vmid)
                e.valid = false;
        }
    }
}

void
WalkCache::invalidateAll()
{
    for (size_t s = 0; s < sets.size(); s++) {
        Set &set = sets[s];

        for (size_t i = 0; i < set.size(); i++)
            set[i].valid = false;
    }
}

size_t
WalkCache::pickSetIdx(Addr va, Addr vaMask,
                      unsigned stage, unsigned level) const
{
    (void) stage;

    int size, offset;

    switch (stage) {
        case 1:
            assert (level<=3);
            size = sizes[0*WALK_CACHE_LEVELS + level];
            offset = offsets[0*WALK_CACHE_LEVELS + level];
            break;

        case 2:
            assert (level<=3);
            size = sizes[1*WALK_CACHE_LEVELS + level];
            offset = offsets[1*WALK_CACHE_LEVELS + level];
            break;

        default:
            panic("bad stage");
    }

    return ((va >> findLsbSet(vaMask)) % size) + offset;
}

size_t
WalkCache::pickEntryIdxToReplace(const Set &set,
                                 unsigned stage, unsigned level)
{
    size_t lru_idx = 0;
    uint32_t lru_tick = UINT32_MAX;

    for (size_t i = 0; i < set.size(); i++) {
        if (!set[i].valid) {
            insertions++;
            insertionsByStageLevel[stage-1][level]++;
            return i;
        }

        if (set[i].lastUsed < lru_tick) {
            lru_idx = i;
            lru_tick = set[i].lastUsed;
        }
    }

    switch (replacementPolicy) {
    case SMMU_CACHE_REPL_ROUND_ROBIN:
        return nextToReplace = ((nextToReplace+1) % associativity);

    case SMMU_CACHE_REPL_RANDOM:
        return random.random<size_t>(0, associativity-1);

    case SMMU_CACHE_REPL_LRU:
        return lru_idx;

    default:
        panic("Unknown replacement policy %d\n", replacementPolicy);
    }

}

void
WalkCache::regStats(const std::string &name)
{
    using namespace Stats;

    SMMUv3BaseCache::regStats(name);

    for (int s = 0; s < 2; s++) {
        for (int l = 0; l < WALK_CACHE_LEVELS; l++) {
            averageLookupsByStageLevel[s][l]
                .name(csprintf("%s.averageLookupsS%dL%d", name, s+1, l))
                .desc("Average number lookups per second")
                .flags(pdf);

            totalLookupsByStageLevel[s][l]
                .name(csprintf("%s.totalLookupsS%dL%d", name, s+1, l))
                .desc("Total number of lookups")
                .flags(pdf);

            averageLookupsByStageLevel[s][l] =
                totalLookupsByStageLevel[s][l] / simSeconds;


            averageMissesByStageLevel[s][l]
                .name(csprintf("%s.averageMissesS%dL%d", name, s+1, l))
                .desc("Average number misses per second")
                .flags(pdf);

            totalMissesByStageLevel[s][l]
                .name(csprintf("%s.totalMissesS%dL%d", name, s+1, l))
                .desc("Total number of misses")
                .flags(pdf);

            averageMissesByStageLevel[s][l] =
                totalMissesByStageLevel[s][l] / simSeconds;


            averageUpdatesByStageLevel[s][l]
                .name(csprintf("%s.averageUpdatesS%dL%d", name, s+1, l))
                .desc("Average number updates per second")
                .flags(pdf);

            totalUpdatesByStageLevel[s][l]
                .name(csprintf("%s.totalUpdatesS%dL%d", name, s+1, l))
                .desc("Total number of updates")
                .flags(pdf);

            averageUpdatesByStageLevel[s][l] =
                totalUpdatesByStageLevel[s][l] / simSeconds;


            averageHitRateByStageLevel[s][l]
                .name(csprintf("%s.averageHitRateS%dL%d", name, s+1, l))
                .desc("Average hit rate")
                .flags(pdf);

            averageHitRateByStageLevel[s][l] =
                (totalLookupsByStageLevel[s][l] -
                 totalMissesByStageLevel[s][l])
                / totalLookupsByStageLevel[s][l];

            insertionsByStageLevel[s][l]
                .name(csprintf("%s.insertionsS%dL%d", name, s+1, l))
                .desc("Number of insertions (not replacements)")
                .flags(pdf);
        }
    }
}
