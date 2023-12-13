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

#ifndef __MEM_CACHE_TAGS_PARTITIONING_POLICIES_WAY_ALLOCATION_HH__
#define __MEM_CACHE_TAGS_PARTITIONING_POLICIES_WAY_ALLOCATION_HH__

#include <vector>

#include "params/WayPolicyAllocation.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class ReplaceableEntry;

namespace partitioning_policy
{

/**
 * A WayPolicyAllocation holds a single PartitionID->Ways allocation for Way
 * Partitioning Policies.
 *
 * @see WayPartitioningPolicy
 */
class WayPolicyAllocation : public SimObject
{
  public:
    WayPolicyAllocation(const WayPolicyAllocationParams &params);

    /**
    * Way Policy Allocation _ways getter
    * @return Allocation ways
    */
    std::vector< uint64_t > getWays() const;

    /**
    * Way Policy Allocation _partitionId getter
    * @return Allocation Partition ID
    */
    uint64_t getPartitionId() const;

  private:
    /**
    * Vector of ways to allocated to the PartitionID
    */
    const std::vector< uint64_t > _ways;

    /**
    * PartitionID on which allocation should be enforced
    */
    const uint64_t _partitionId;
};

} // namespace partitioning_policy

} // namespace gem5

#endif // __MEM_CACHE_TAGS_PARTITIONING_POLICIES_WAY_ALLOCATION_HH__
