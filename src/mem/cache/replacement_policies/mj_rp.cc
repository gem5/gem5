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
    counters.init(LLC_NUM_SETS, LLC_NUM_WAYS);
}

void MJRP::invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
{
    auto set = std::static_pointer_cast<MJReplData>(replacement_data)->set;
    auto way = std::static_pointer_cast<MJReplData>(replacement_data)->way;

    if (sampled_cache.is_set_to_sample(set))
    {
        auto fullAddr =
            std::static_pointer_cast<MJReplData>(replacement_data)->blkAddr;
        auto sampled_set = sampled_cache.get_set(fullAddr);
        auto sampled_tag = sampled_cache.get_tag(fullAddr);
        auto sampled_way = sampled_cache.is_present(sampled_tag, sampled_set);
        if (sampled_way == -1)
            return;
        auto& entry = sampled_cache.at(sampled_set, sampled_way);
        if (!entry.valid)
            return;
        entry.valid = false;
        rdp.set_value_of(entry.last_pc_signature, MAX_RD);
        counters.set_estimated_time_remaining(set, way, INF_ETR);
    }
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
    auto fullAddr = pkt->getAddr() == 0 ? castedData->blkAddr : pkt->getAddr();
    auto set = castedData->set;
    auto way = castedData->way;
    uint16_t pcSignature = get_pc_signature(ip, true);
    castedData->generatingPC = ip;
    if (sampled_cache.is_set_to_sample(set))
    {
        auto sampled_set = sampled_cache.get_set(fullAddr);
        auto sampled_tag = sampled_cache.get_tag(fullAddr);
        auto sampled_way = sampled_cache.is_present(sampled_tag, sampled_set);
        if (sampled_way != -1)
        {
            auto& entry = sampled_cache.at(sampled_set, sampled_way);
            auto last_sig = entry.last_pc_signature;
            auto last_access_timestamp = entry.last_access_timestamp;
            auto sset_timestamp = counters.get_set_timestamp(set);
            int diff = abs(last_access_timestamp - sset_timestamp);

            if (diff <= 127)
            {
                int rdp_value = rdp.get_value_of(last_sig);
                int new_val = 0;
                if (rdp_value == std::numeric_limits<uint8_t>::max())
                    new_val = diff;
                else
                    new_val = rdp_value + ((rdp_value > diff)
                                               ? std::min(1, diff / 16)
                                               : -std::min(1, diff / 16));

                rdp.set_value_of(last_sig, new_val);
                sampled_cache.at(sampled_set, sampled_way).valid = false;
            }
        }

        int penalized_block = -1;
        int reuse_distance = -1;
        for (int i = 0; i < SAMPLED_CACHE_WAYS; i++)
        {
            if (!sampled_cache.at(sampled_set, i).valid)
            {
                penalized_block = i;
                reuse_distance = INF_RD + 1;
                continue;
            }

            uint64_t last_timestamp =
                sampled_cache.at(sampled_set, i).last_access_timestamp;
            int set_timestamp = counters.get_set_timestamp(set);
            int diff = 0;
            if (set_timestamp > last_timestamp)
                diff = set_timestamp - last_timestamp;
            else
            {
                diff = set_timestamp + (1 << TIMESTAMP_WIDTH);
                diff -= last_timestamp;
            }
            if (diff > INF_RD)
            {
                penalized_block = i;
                reuse_distance = INF_RD + 1;
                penalize_block(sampled_set, i);
            }
            else if (diff > reuse_distance)
            {
                penalized_block = i;
                reuse_distance = diff;
            }
        }
        penalize_block(sampled_set, penalized_block);

        auto& invalidEntry = sampled_cache.get_invalid_entry(sampled_set);
        if (!invalidEntry.valid)
        {
            invalidEntry.valid = true;
            invalidEntry.last_pc_signature = pcSignature;
            invalidEntry.tag = sampled_tag;
            invalidEntry.last_access_timestamp =
                counters.get_set_timestamp(set);
        }
        counters.progress_set_clock(set);
    }

    counters.set_current_set_clock(set,
                                   counters.get_current_set_clock(set) + 1);
    if (counters.get_current_set_clock(set) == MAX_ETR_CLOCK)
    {
        for (int i = 0; i < LLC_NUM_WAYS; i++)
        {
            if (i == way)
                continue;
            /* WE ONLY AGE NON-SCANNING LINES. Those are the lines that we
             * denote as not being accessed again, and they priorizied as
             * choices for eviction. */
            if (std::abs(counters.get_estimated_time_remaining(set, i)) <
                INF_ETR)
                counters.decrement_estimated_time_remaining(set, i);
        }
        /* Every (eight = MAX_ETR_CLOCK) set accesses, the set’s clock is reset
         * to 0, and every line in the set is aged. */
        counters.reset_set_clock(set);
    }

    /*
     * On insertion and promotion, a line’s ETR is initialized with its
     * predicted reuse distance obtained from the RDP "Mockingjay tracks
     * coarse-grained reuse distances (and corresponding ETRs), which are
     * obtained by dividing the precise value by a constant factor f, where f
     * is set to 8 in our evaluation" This ensures that non-scanning lines' ETR
     * values are effectively decrement every 8 accesses by 1.
     */
    if (rdp.get_value_of(pcSignature) == std::numeric_limits<uint8_t>::max())
    {
        counters.set_estimated_time_remaining(set, way, 0);
    }
    else
    {
        if (rdp.get_value_of(pcSignature) > MAX_RD)
            counters.set_estimated_time_remaining(set, way, INF_ETR);
        else
            counters.set_estimated_time_remaining(
                set, way, rdp.get_value_of(pcSignature) / CONSTANT_FACTOR_F);
    }

    warn("EXIT TOUCH\n");
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
    std::static_pointer_cast<MJReplData>(replacement_data)->generatingPC =
        pkt->req->hasPC() ? pkt->req->getPC() : 0;
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
    castedData->generatingPC = ip;
    if (sampled_cache.is_set_to_sample(set))
    {
        auto sampled_set = sampled_cache.get_set(fullAddr);
        auto sampled_tag = sampled_cache.get_tag(fullAddr);
        auto sampled_way = sampled_cache.is_present(sampled_tag, sampled_set);
        if (sampled_way != -1)
        {
            auto& entry = sampled_cache.at(sampled_set, sampled_way);
            auto last_sig = entry.last_pc_signature;
            auto last_access_timestamp = entry.last_access_timestamp;
            auto sset_timestamp = counters.get_set_timestamp(set);
            int diff = abs(last_access_timestamp - sset_timestamp);

            if (diff <= 127)
            {
                int rdp_value = rdp.get_value_of(last_sig);
                int new_val = 0;
                if (rdp_value == std::numeric_limits<uint8_t>::max())
                    new_val = diff;
                else
                    new_val = rdp_value + ((rdp_value > diff)
                                               ? std::min(1, diff / 16)
                                               : -std::min(1, diff / 16));

                rdp.set_value_of(last_sig, new_val);
                sampled_cache.at(sampled_set, sampled_way).valid = false;
            }
        }

        int penalized_block = -1;
        int reuse_distance = -1;
        for (int i = 0; i < SAMPLED_CACHE_WAYS; i++)
        {
            if (!sampled_cache.at(sampled_set, i).valid)
            {
                penalized_block = i;
                reuse_distance = INF_RD + 1;
                continue;
            }

            uint64_t last_timestamp =
                sampled_cache.at(sampled_set, i).last_access_timestamp;
            int set_timestamp = counters.get_set_timestamp(set);
            int diff = 0;
            if (set_timestamp > last_timestamp)
                diff = set_timestamp - last_timestamp;
            else
            {
                diff = set_timestamp + (1 << TIMESTAMP_WIDTH);
                diff -= last_timestamp;
            }
            if (diff > INF_RD)
            {
                penalized_block = i;
                reuse_distance = INF_RD + 1;
                penalize_block(sampled_set, i);
            }
            else if (diff > reuse_distance)
            {
                penalized_block = i;
                reuse_distance = diff;
            }
        }
        penalize_block(sampled_set, penalized_block);

        auto& invalidEntry = sampled_cache.get_invalid_entry(sampled_set);
        if (!invalidEntry.valid)
        {
            invalidEntry.valid = true;
            invalidEntry.last_pc_signature = pcSignature;
            invalidEntry.tag = sampled_tag;
            invalidEntry.last_access_timestamp =
                counters.get_set_timestamp(set);
        }
        counters.progress_set_clock(set);
    }

    counters.set_current_set_clock(set,
                                   counters.get_current_set_clock(set) + 1);
    if (counters.get_current_set_clock(set) == MAX_ETR_CLOCK)
    {
        for (int i = 0; i < LLC_NUM_WAYS; i++)
        {
            if (i == way)
                continue;
            /* WE ONLY AGE NON-SCANNING LINES. Those are the lines that we
             * denote as not being accessed again, and they priorizied as
             * choices for eviction. */
            if (std::abs(counters.get_estimated_time_remaining(set, i)) <
                INF_ETR)
                counters.decrement_estimated_time_remaining(set, i);
        }
        /* Every (eight = MAX_ETR_CLOCK) set accesses, the set’s clock is reset
         * to 0, and every line in the set is aged. */
        counters.reset_set_clock(set);
    }

    /*
     * On insertion and promotion, a line’s ETR is initialized with its
     * predicted reuse distance obtained from the RDP "Mockingjay tracks
     * coarse-grained reuse distances (and corresponding ETRs), which are
     * obtained by dividing the precise value by a constant factor f, where f
     * is set to 8 in our evaluation" This ensures that non-scanning lines' ETR
     * values are effectively decrement every 8 accesses by 1.
     */
    if (rdp.get_value_of(pcSignature) == std::numeric_limits<uint8_t>::max())
    {
        counters.set_estimated_time_remaining(set, way, 0);
    }
    else
    {
        if (rdp.get_value_of(pcSignature) > MAX_RD)
            counters.set_estimated_time_remaining(set, way, INF_ETR);
        else
            counters.set_estimated_time_remaining(
                set, way, rdp.get_value_of(pcSignature) / CONSTANT_FACTOR_F);
    }

    warn("EXIT RESET\n");
}

ReplaceableEntry*
MJRP::getVictim(const ReplacementCandidates& candidates) const
{
    warn("ENTER VICTIM\n");
    // There must be at least one replacement candidate
    assert(candidates.size() > 0);

    auto ret = counters.way_to_evict(candidates[0]->getSet());
    uint32_t way_to_evict = ret.first;
    uint32_t etr = ret.second;

    ReplaceableEntry* victim = candidates[0];
    for (const auto& candidate : candidates)
    {
        // Update victim entry if necessary
        if (candidate->getWay() == way_to_evict)
        {
            victim = candidate;
        }

        std::static_pointer_cast<MJReplData>(candidate->replacementData)->way =
            candidate->getWay();
        std::static_pointer_cast<MJReplData>(candidate->replacementData)->set =
            candidate->getSet();
    }

    uint64_t pc_signature = get_pc_signature(
        std::static_pointer_cast<MJReplData>(victim->replacementData)
            ->generatingPC,
        false);

    if (rdp.get_value_of(pc_signature) !=
            std::numeric_limits<uint8_t>::max() &&
        (rdp.get_value_of(pc_signature) > MAX_RD ||
         rdp.get_value_of(pc_signature) / CONSTANT_FACTOR_F > etr))
        return nullptr;

    warn("EXIT VICTIM\n");
    return victim;
}

std::shared_ptr<ReplacementData> MJRP::instantiateEntry()
{
    return std::shared_ptr<ReplacementData>(new MJReplData());
}

} // namespace replacement_policy
} // namespace gem5
