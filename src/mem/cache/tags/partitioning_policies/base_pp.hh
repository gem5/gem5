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

#ifndef __MEM_CACHE_TAGS_PARTITIONING_POLICIES_BASE_HH__
#define __MEM_CACHE_TAGS_PARTITIONING_POLICIES_BASE_HH__

#include <vector>

#include "params/BasePartitioningPolicy.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class ReplaceableEntry;

namespace partitioning_policy
{

/**
 * A Partitioning Policy is a cache partitioning mechanism that limits the
 * cache block allocations in a cache based on a PartitionID identifier. This
 * identifier may be set to any upstream memory request by attaching the
 * PartitionID to it. The way the partition ID is attached/extracted
 * from the request depends on the partitioning manager.
 *
 * See the use of the PartitionFieldExtension in Arm as an example.
 *
 * When partitioning policies are in place, the allocatable cache blocks for
 * this memory request will be filtered based on its PartitionID.
 *
 */
class BasePartitioningPolicy : public SimObject
{
  public:
    BasePartitioningPolicy(const BasePartitioningPolicyParams &params);

    /**
     * Filters the allocatable cache blocks for a memory request based on its
     * PartitionID and policy allocation
     * @param entries candidate cache blocks for this request; filtered in
     * place
     * @param partition_id PartitionID of the upstream memory request
     */
    virtual void filterByPartition(std::vector<ReplaceableEntry *> &entries,
                                   const uint64_t partition_id) const = 0;

    /**
     * Notify of acquisition of ownership of a cache line
     * @param partition_id PartitionID of the upstream memory request
     */
    virtual void notifyAcquire(const uint64_t partition_id) = 0;

    /**
     * Notify of release of ownership of a cache line
     * @param partition_id PartitionID of the upstream memory request
     */
    virtual void notifyRelease(const uint64_t partition_id) = 0;
};

} // namespace partitioning_policy

} // namespace gem5

#endif // __MEM_CACHE_TAGS_PARTITIONING_POLICIES_BASE_HH__
