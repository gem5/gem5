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

#ifndef __MEM_CACHE_TAGS_PARTITIONING_POLICIES_FIELD_EXTENTION_HH__
#define __MEM_CACHE_TAGS_PARTITIONING_POLICIES_FIELD_EXTENTION_HH__

#include "base/extensible.hh"
#include "mem/packet.hh"
#include "mem/request.hh"

namespace gem5
{

namespace partitioning_policy
{

const uint64_t DEFAULT_PARTITION_ID = 0;
const uint64_t DEFAULT_PARTITION_MONITORING_ID = 0;

class PartitionFieldExtention :
    public Extension<Request, PartitionFieldExtention>
{
  public:
    std::unique_ptr<ExtensionBase> clone() const override;
    PartitionFieldExtention() = default;

    /**
     * _partitionID getter
     * @return extension Partition ID
     */
    uint64_t getPartitionID() const;

    /**
     * _partitionMonitoringID getter
     * @return extension Partition Monitoring ID
     */
    uint64_t getPartitionMonitoringID() const;

    /**
     * _partitionID setter
     * @param id Partition ID to set for the extension
     */
    void setPartitionID(uint64_t id);

    /**
     * _partitionMonitoringID setter
     * @param id Partition Monitoring ID to set for the extension
     */
    void setPartitionMonitoringID(uint64_t id);

  private:
    uint64_t _partitionID = DEFAULT_PARTITION_ID;
    uint64_t _partitionMonitoringID = DEFAULT_PARTITION_MONITORING_ID;
};

/**
 * Helper function to retrieve PartitionID from a packet; Returns packet
 * PartitionID if available or DEFAULT_PARTITION_ID if extention is not set
 * @param pkt pointer to packet (PacketPtr)
 * @return packet PartitionID.
 */
uint64_t readPacketPartitionID(PacketPtr pkt);

} // namespace partitioning_policy

} // namespace gem5

#endif // __MEM_CACHE_TAGS_PARTITIONING_POLICIES_FIELD_EXTENTION_HH__
