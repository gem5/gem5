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

#ifndef __MEM_CACHE_TAGS_PARTITIONING_POLICIES_WAY_HH__
#define __MEM_CACHE_TAGS_PARTITIONING_POLICIES_WAY_HH__

#include <unordered_map>
#include <unordered_set>

#include "debug/PartitionPolicy.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"
#include "mem/cache/tags/partitioning_policies/base_pp.hh"
#include "params/WayPartitioningPolicy.hh"

namespace gem5
{

namespace partitioning_policy
{

/**
 * A WayPartitioningPolicy filters the cache blocks available to a memory
 * requestor (identified via PartitionID) based on the cache ways allocated to
 * that requestor. This policy has no effect on requests with unregistered
 * PartitionIDs.
 *
 * @see BasePartitioningPolicy
 */
class WayPartitioningPolicy : public BasePartitioningPolicy
{
  public:
    WayPartitioningPolicy(const WayPartitioningPolicyParams &params);

    void
    filterByPartition(std::vector<ReplaceableEntry *> &entries,
                        const uint64_t partition_id) const override;

    /**
    * Empty implementation as block allocations do not vary with number of
    * allocated blocks for this policy
    * @param partition_id PartitionID of the upstream memory request
    */
    void
    notifyAcquire(const uint64_t partition_id) override {};

    /**
    * Empty implementation as block allocations do not vary with number of
    * allocated blocks for this policy
    * @param partition_id PartitionID of the upstream memory request
    */
    void
    notifyRelease(const uint64_t partition_id) override {};

    void addWayToPartition(uint64_t partition_id, unsigned way);
    void removeWayToPartition(uint64_t partition_id, unsigned way);

  private:
    /**
    * Map of policied PartitionIDs and their associated cache ways
    */
    std::unordered_map< uint64_t, std::unordered_set< unsigned > >
        partitionIdWays;
};

} // namespace partitioning_policy

} // namespace gem5

#endif // __MEM_CACHE_TAGS_PARTITIONING_POLICIES_WAY_HH__
