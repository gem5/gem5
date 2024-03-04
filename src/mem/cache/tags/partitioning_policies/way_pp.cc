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

#include "mem/cache/tags/partitioning_policies/way_pp.hh"

#include <algorithm>

#include "base/logging.hh"
#include "base/trace.hh"
#include "params/WayPartitioningPolicy.hh"
#include "way_allocation.hh"

namespace gem5
{

namespace partitioning_policy
{

WayPartitioningPolicy::WayPartitioningPolicy
    (const WayPartitioningPolicyParams &params): BasePartitioningPolicy(params)
{
    // get cache associativity and check it is usable for this policy
    const auto cache_assoc = params.cache_associativity;
    assert(cache_assoc > 0);

    // iterate over all provided allocations
    for (const auto allocation: params.allocations) {
        const auto alloc_id = allocation->getPartitionId();

        // save way allocations in policy
        for (const auto way: allocation->getWays()) {

            // check if allocations are valid
            fatal_if(way >= cache_assoc, "Way Partitioning Policy allocation "
                "for PartitionID: %d, Way: %d cannot be fullfiled as cache "
                "associativity is %d", alloc_id, way, cache_assoc);

            if (this->partitionIdWays[alloc_id].count(way) == 0) {
                this->partitionIdWays[alloc_id].emplace(way);
            } else {
                // do not add duplicate allocation to policy and warn
                warn("Duplicate Way Partitioning Policy allocation for "
                    "PartitionID: %d, Way: %d",
                    alloc_id, way);
            }
        }

        // report allocation of policies
        DPRINTF(PartitionPolicy, "Allocated %d ways in WayPartitioningPolicy "
            "for PartitionID: %d \n", allocation->getWays().size(),
            alloc_id);
    }
}

void
WayPartitioningPolicy::filterByPartition(
    std::vector<ReplaceableEntry *> &entries,
    const uint64_t partition_id) const
{
    if (// No entries to filter
        entries.empty() ||
        // This partition_id is not policed
        partitionIdWays.find(partition_id) == partitionIdWays.end()) {
        return;
    } else {
        const auto entries_to_remove = std::remove_if(
            entries.begin(),
            entries.end(),
            [this, partition_id]
            (ReplaceableEntry *entry)
            {
                return partitionIdWays.at(partition_id).find(entry->getWay())
                    == partitionIdWays.at(partition_id).end();
            }
        );

        entries.erase(entries_to_remove, entries.end());
    }
}

} // namespace partitioning_policy

} // namespace gem5
