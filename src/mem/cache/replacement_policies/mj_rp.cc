/**
 * Copyright (c) 2018-2020 Inria
 * All rights reserved.
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

#include "mem/cache/replacement_policies/mj_rp.hh"

#include <cassert>
#include <cmath>
#include <memory>

#include "params/MJRP.hh"
#include "sim/cur_tick.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(ReplacementPolicy, replacement_policy);
namespace replacement_policy
{

static uint64_t murmur_hash(uint64_t pc)
{
    pc ^= pc >> 33;
    pc *= 0xff51afd7ed558ccdL;
    pc ^= pc >> 33;
    pc *= 0xc4ceb9fe1a85ec53L;
    pc ^= pc >> 33;
    return pc;
}

static uint64_t get_pc_signature(uint64_t pc, bool is_hit)
{
    return murmur_hash((pc << 1) | (is_hit)) & (PC_SIG_MASK);
}

MJRP::MJRP(const Params& p) : Base(p)
{
    sampled_cache.init();
    etr_counters.init(LLC_NUM_SETS, LLC_NUM_WAYS);
}

void MJRP::invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
{
    // Reset last touch timestamp
    std::static_pointer_cast<MJReplData>(replacement_data)->lastTouchTick =
        Tick(0);
}

void MJRP::touch(
    const std::shared_ptr<ReplacementData>& replacement_data) const
{
    panic("CAN'T OPERATE WITHOUT ACCESS PACKET INFORMATION");
}

void MJRP::touch(const std::shared_ptr<ReplacementData>& replacement_data,
                 const PacketPtr pkt)
{
    warn("ENTER TOUCH\n");
    auto castedData = std::static_pointer_cast<MJReplData>(replacement_data);
    uint64_t ip = 0;
    if (pkt->req->hasPC())
        ip = pkt->req->getPC();
    auto fullAddr = pkt->getAddr();
    auto set = castedData->set;
    auto way = castedData->way;
    uint16_t pcSignature = get_pc_signature(ip, true);
    castedData->generatingPCSignature = pcSignature;
    if (sampled_cache.is_set_to_sample(set))
    {
        auto sampled_set = sampled_cache.get_set(fullAddr);
        auto sampled_tag = sampled_cache.get_tag(fullAddr);
        auto sampled_way = sampled_cache.is_present(sampled_tag, sampled_set);
        if (sampled_way == -1)
            goto out;

        auto& entry = sampled_cache.at(sampled_set, sampled_way);
        auto last_sig = entry.last_pc_signature;
        auto last_access_timestamp = entry.last_access_timestamp;
        auto sset_timestamp = etr_counters.get_set_timestamp(sampled_set);
        int diff = abs(last_access_timestamp - sset_timestamp);

        if (diff <= 127)
        {
            int rdp_value = rdp.get_value_of(pcSignature);
            int new_val = 0;

            if (rdp_value == std::numeric_limits<uint8_t>::max())
                new_val = diff;
            else
                new_val = rdp_value + (rdp_value > diff)
                              ? std::min(1, diff / 16)
                              : -std::min(1, diff / 16);

            rdp.set_value_of(pcSignature, new_val);
            sampled_cache.at(sampled_set, sampled_way).valid = false;
        }

        int lru_way = -1;
        int lru_rd = -1;
        for (int w = 0; w < SAMPLED_CACHE_WAYS; w++)
        {
            if (!sampled_cache.at(sampled_set, w).valid)
            {
                lru_way = w;
                lru_rd = INF_RD + 1;
                continue;
            }

            uint64_t last_timestamp =
                sampled_cache.at(sampled_set, w).last_access_timestamp;
            int set_timestamp = etr_counters.get_set_timestamp(sampled_set);
            int diff = 0;
            if (set_timestamp > last_timestamp)
                diff = set_timestamp > last_timestamp;
            else
            {
                diff = set_timestamp + (1 << TIMESTAMP_BITS);
                diff -= last_timestamp;
            }
            if (diff > INF_RD)
            {
                lru_way = w;
                lru_rd = INF_RD + 1;
                penalize_block(sampled_set, w);
            }
            else if (diff > lru_rd)
            {
                lru_way = w;
                lru_rd = diff;
            }
        }
        penalize_block(sampled_set, lru_way);

        for (int w = 0; w < SAMPLED_CACHE_WAYS; w++)
        {
            auto& tmpEntry = sampled_cache.at(sampled_set, w);
            if (!tmpEntry.valid)
            {
                tmpEntry.valid = true;
                tmpEntry.last_pc_signature = pcSignature;
                tmpEntry.tag = sampled_tag;
                tmpEntry.last_access_timestamp =
                    etr_counters.get_set_timestamp(set);
                break;
            }
        }
        etr_counters.increment_timestamp(sampled_set);
    }
out:

    if (etr_counters.get_current_set_clock(set) == GRANULARITY)
    {
        for (int w = 0; w < LLC_NUM_WAYS; w++)
        {
            if ((uint32_t)w != way &&
                abs(etr_counters.get_estimated_time_remaining(set, w)) <
                    INF_ETR)
                etr_counters.downgrade(set, w);
        }
        etr_counters.reset_set_clock(set);
    }
    etr_counters.set_current_set_clock(
        set, etr_counters.get_current_set_clock(set) + 1);

    if (way < LLC_NUM_WAYS)
    {
        if (rdp.get_value_of(pcSignature) ==
            std::numeric_limits<uint8_t>::max())
        {
            etr_counters.set_estimated_time_remaining(set, way, 0);
        }
        else
        {
            if (rdp.get_value_of(pcSignature) > MAX_RD)
                etr_counters.set_estimated_time_remaining(set, way, INF_ETR);
            else
                etr_counters.set_estimated_time_remaining(
                    set, way, rdp.get_value_of(pcSignature) / GRANULARITY);
        }
    }

    warn("EXIT TOUCH\n");
    return;
}

void MJRP::reset(
    const std::shared_ptr<ReplacementData>& replacement_data) const
{
    panic("CAN'T OPERATE WITHOUT ACCESS PACKET INFORMATION");
}

void MJRP::reset(const std::shared_ptr<ReplacementData>& replacement_data,
                 const PacketPtr pkt)
{
    warn("ENTER RESET\n");
    std::static_pointer_cast<MJReplData>(replacement_data)->lastTouchTick =
        curTick();
    std::static_pointer_cast<MJReplData>(replacement_data)
        ->generatingPCSignature =
        pkt->req->hasPC() ? get_pc_signature(pkt->req->getPC(), false) : 0;
    std::static_pointer_cast<MJReplData>(replacement_data)->blkAddr =
        pkt->getAddr();

    auto castedData = std::static_pointer_cast<MJReplData>(replacement_data);
    uint64_t ip = 0;
    if (pkt->req->hasPC())
        ip = pkt->req->getPC();
    auto fullAddr = pkt->getAddr();
    auto set = castedData->set;
    auto way = castedData->way;
    uint16_t pcSignature = get_pc_signature(ip, false);
    castedData->generatingPCSignature = pcSignature;
    if (sampled_cache.is_set_to_sample(set))
    {
        auto sampled_set = sampled_cache.get_set(fullAddr);
        auto sampled_tag = sampled_cache.get_tag(fullAddr);
        auto sampled_way = sampled_cache.is_present(sampled_tag, sampled_set);
        if (sampled_way == -1)
            goto out;

        auto& entry = sampled_cache.at(sampled_set, sampled_way);
        auto last_sig = entry.last_pc_signature;
        auto last_access_timestamp = entry.last_access_timestamp;
        int diff = abs(last_access_timestamp -
                       etr_counters.get_set_timestamp(sampled_set));

        if (diff <= 127)
        {
            int rdp_value = rdp.get_value_of(pcSignature);
            int new_val = 0;

            if (rdp_value == std::numeric_limits<uint8_t>::max())
                new_val = diff;
            else
                new_val = rdp_value + (rdp_value > diff)
                              ? std::min(1, diff / 16)
                              : -std::min(1, diff / 16);

            rdp.set_value_of(pcSignature, new_val);
            sampled_cache.at(sampled_set, sampled_way).valid = false;
        }

        int lru_way = -1;
        int lru_rd = -1;
        for (int w = 0; w < SAMPLED_CACHE_WAYS; w++)
        {
            if (!sampled_cache.at(sampled_set, w).valid)
            {
                lru_way = w;
                lru_rd = INF_RD + 1;
                continue;
            }

            uint64_t last_timestamp =
                sampled_cache.at(sampled_set, w).last_access_timestamp;
            int set_timestamp = etr_counters.get_set_timestamp(sampled_set);
            int diff = 0;
            if (set_timestamp > last_timestamp)
                diff = set_timestamp > last_timestamp;
            else
            {
                diff = set_timestamp + (1 << TIMESTAMP_BITS);
                diff -= last_timestamp;
            }
            if (diff > INF_RD)
            {
                lru_way = w;
                lru_rd = INF_RD + 1;
                penalize_block(sampled_set, w);
            }
            else if (diff > lru_rd)
            {
                lru_way = w;
                lru_rd = diff;
            }
        }
        penalize_block(sampled_set, lru_way);

        for (int w = 0; w < SAMPLED_CACHE_WAYS; w++)
        {
            auto& tmpEntry = sampled_cache.at(sampled_set, w);
            if (!tmpEntry.valid)
            {
                tmpEntry.valid = true;
                tmpEntry.last_pc_signature = pcSignature;
                tmpEntry.tag = sampled_tag;
                tmpEntry.last_access_timestamp =
                    etr_counters.get_set_timestamp(set);
                break;
            }
        }
        etr_counters.increment_timestamp(sampled_set);
    }
out:

    if (etr_counters.get_current_set_clock(set) == GRANULARITY)
    {
        for (int w = 0; w < LLC_NUM_WAYS; w++)
        {
            if ((uint32_t)w != way &&
                abs(etr_counters.get_estimated_time_remaining(set, w)) <
                    INF_ETR)
                etr_counters.downgrade(set, w);
        }
        etr_counters.reset_set_clock(set);
    }
    etr_counters.set_current_set_clock(
        set, etr_counters.get_current_set_clock(set) + 1);

    if (way < LLC_NUM_WAYS)
    {
        if (rdp.get_value_of(pcSignature) ==
            std::numeric_limits<uint8_t>::max())
        {
            etr_counters.set_estimated_time_remaining(set, way, 0);
        }
        else
        {
            if (rdp.get_value_of(pcSignature) > MAX_RD)
                etr_counters.set_estimated_time_remaining(set, way, INF_ETR);
            else
                etr_counters.set_estimated_time_remaining(
                    set, way, rdp.get_value_of(pcSignature) / GRANULARITY);
        }
    }

    warn("EXIT RESET\n");
}

ReplaceableEntry*
MJRP::getVictim(const ReplacementCandidates& candidates) const
{
    warn("ENTER VICTIM\n");
    // There must be at least one replacement candidate
    assert(candidates.size() > 0);

    uint32_t way_to_evict = etr_counters.way_to_evict(candidates[0]->getSet());
    ReplaceableEntry* victim = candidates[0];
    ReplaceableEntry* lruCandidate = candidates[0];
    for (const auto& candidate : candidates)
    {
        // Update victim entry if necessary
        if (candidate->getWay() == way_to_evict)
        {
            victim = candidate;
        }

        if (std::static_pointer_cast<MJReplData>(candidate->replacementData)
                ->lastTouchTick <
            std::static_pointer_cast<MJReplData>(victim->replacementData)
                ->lastTouchTick)
        {
            lruCandidate = candidate;
        }

        std::static_pointer_cast<MJReplData>(candidate->replacementData)->way =
            candidate->getWay();
        std::static_pointer_cast<MJReplData>(candidate->replacementData)->set =
            candidate->getSet();
    }

    if (victim->getWay() != way_to_evict)
    {
        victim = lruCandidate;
        warn("PICKING LRU\n");
    }

    warn("EXIT VICTIM\n");
    return victim;
}

std::shared_ptr<ReplacementData> MJRP::instantiateEntry()
{
    return std::shared_ptr<ReplacementData>(new MJReplData());
}

} // namespace replacement_policy
} // namespace gem5
