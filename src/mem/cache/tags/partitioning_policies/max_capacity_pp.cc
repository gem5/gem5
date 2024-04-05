/*
 * Copyright (c) 2024 ARM Limited
 * All rights reserved.
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

#include "mem/cache/tags/partitioning_policies/max_capacity_pp.hh"

#include <algorithm>
#include <string>

#include "base/logging.hh"
#include "base/trace.hh"
#include "params/MaxCapacityPartitioningPolicy.hh"

namespace gem5
{

namespace partitioning_policy
{

MaxCapacityPartitioningPolicy::MaxCapacityPartitioningPolicy
    (const MaxCapacityPartitioningPolicyParams &params):
    BasePartitioningPolicy(params),
    totalBlockCount(params.cache_size / params.blk_size),
    partitionIDs(params.partition_ids),
    capacities(params.capacities)
{
    // check if ids and capacities vectors are the same length
    if (this->partitionIDs.size() != this->capacities.size()) {
        fatal("MaxCapacity Partitioning Policy configuration invalid: ids and "
            "capacities arrays are not equal lengths");
    }

    // check allocations and create map
    for (auto i = 0; i < this->partitionIDs.size(); i++) {
        const uint64_t partition_id = this->partitionIDs[i];
        const double cap_frac = capacities[i];

        // check Capacity Fraction (cap_frac) is actually a fraction in [0,1]
        if (!(cap_frac >= 0 && cap_frac <= 1)) {
            fatal("MaxCapacity Partitioning Policy for PartitionID %d has "
                "Capacity Fraction %f outside of [0,1] range", partition_id,
                cap_frac);
        }

        const uint64_t allocated_block_cnt = cap_frac * totalBlockCount;
        partitionIdMaxCapacity.emplace(partition_id, allocated_block_cnt);

        DPRINTF(PartitionPolicy, "Configured MaxCapacity Partitioning Policy "
            "for PartitionID: %d to use portion of size %f (%d cache blocks "
            "of %d total)\n", partition_id, cap_frac, allocated_block_cnt,
            totalBlockCount);

    }
}

void
MaxCapacityPartitioningPolicy::filterByPartition(
    std::vector<ReplaceableEntry *> &entries,
    const uint64_t id) const
{
    if (// No entries to filter
        entries.empty() ||
        // This partition_id is not policed
        partitionIdMaxCapacity.find(id) == partitionIdMaxCapacity.end() ||
        // This partition_id has not yet used the cache
        partitionIdCurCapacity.find(id) == partitionIdCurCapacity.end() ||
        // The partition_id usage is below the maximum
        partitionIdCurCapacity.at(id) < partitionIdMaxCapacity.at(id))
        return;

    // Limit reached, restrict allocation only to blocks owned by
    // the Partition ID
    entries.erase(std::remove_if(entries.begin(), entries.end(),
        [id](ReplaceableEntry *entry) {
            CacheBlk *blk = static_cast<CacheBlk *>(entry);
            return blk->getPartitionId() != id;
        }), entries.end());
}

void
MaxCapacityPartitioningPolicy::notifyAcquire(const uint64_t partition_id)
{
    // sanity check current allocation does not exceed its configured maximum
    assert(partitionIdCurCapacity[partition_id] <=
        partitionIdMaxCapacity[partition_id]);

    partitionIdCurCapacity[partition_id] += 1;
}

void
MaxCapacityPartitioningPolicy::notifyRelease(const uint64_t partition_id)
{
    // sanity check current allocation will not cause underflow
    assert(partitionIdCurCapacity[partition_id] > 0);

    partitionIdCurCapacity[partition_id] -= 1;
}

} // namespace partitioning_policy

} // namespace gem5
