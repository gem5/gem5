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

#ifndef __MEM_CACHE_TAGS_PARTITIONING_POLICIES_MAX_CAPACITY_HH__
#define __MEM_CACHE_TAGS_PARTITIONING_POLICIES_MAX_CAPACITY_HH__

#include <unordered_map>
#include <vector>

#include "debug/PartitionPolicy.hh"
#include "mem/cache/cache_blk.hh"
#include "mem/cache/tags/partitioning_policies/base_pp.hh"
#include "params/BasePartitioningPolicy.hh"
#include "params/MaxCapacityPartitioningPolicy.hh"

namespace gem5
{

namespace partitioning_policy
{

/**
 * A MaxCapacityPartitioningPolicy filters the cache blocks available to a
 * memory requestor (identified via PartitionID) based on count of already
 * allocated blocks. The number of cache blocks a specific memory requestor
 * can have access to is determined by its provided capacities allocation in
 * the [0, 1] range. This policy has no effect on requests with unregistered
 * PartitionIDs.
 *
 * @see BasePartitioningPolicy
 */
class MaxCapacityPartitioningPolicy : public BasePartitioningPolicy
{
  public:
    MaxCapacityPartitioningPolicy
    (const MaxCapacityPartitioningPolicyParams &params);

    void
    filterByPartition(std::vector<ReplaceableEntry *> &entries,
                      const uint64_t partition_id) const override;

    void
    notifyAcquire(const uint64_t partition_id) override;

    void
    notifyRelease(const uint64_t partition_id) override;

  private:
    /**
    * Cache size in number of bytes
    */
    const uint64_t cacheSize;

    /**
    * Cache block size in number of bytes
    */
    const uint64_t blkSize;

    /**
    * Vector of partitionIDs the policy operates on
    */
    const std::vector< uint64_t > partitionIDs;

    /**
    * Vector of capacity fractions to enforce on the policied partitionIDs
    */
    const std::vector< double > capacities;

    /**
    * Map of PartitionIDs and maximum allocatable cache block counts;
    * On evictions full partitions are prioritized.
    */
    std::unordered_map< uint64_t, uint64_t > partitionIdMaxCapacity;

    /**
    * Map of PartitionIDs and currently allocated blck coutns
    */
    std::unordered_map< uint64_t, uint64_t > partitionIdCurCapacity;
};

} // namespace partitioning_policy

} // namespace gem5

#endif // __MEM_CACHE_TAGS_PARTITIONING_POLICIES_MAX_CAPACITY_HH__
