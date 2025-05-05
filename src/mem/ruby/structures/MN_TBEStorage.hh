/*
 * Copyright (c) 2021-2022 ARM Limited
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
 */

#ifndef __MEM_RUBY_STRUCTURES_MN_TBESTORAGE_HH__
#define __MEM_RUBY_STRUCTURES_MN_TBESTORAGE_HH__

#include <cassert>
#include <unordered_map>
#include <vector>

#include <base/statistics.hh>

#include "mem/ruby/common/MachineID.hh"
#include "mem/ruby/structures/TBEStorage.hh"

namespace gem5
{

namespace ruby
{

namespace CHI
{

// MN_TBEStorage is composed of multiple TBEStorage
// partitions that could be used for specific types of TBEs.
// Partition number 0 is the generic partition and will
// store any kind of TBEs.
// Space for specific TBEs will be looked first into the matching
// partition, and when no space is available the generic one will
// be used
template <class RetryEntry>
class MN_TBEStorage
{
  public:
    MN_TBEStorage(statistics::Group *parent,
                  std::initializer_list<TBEStorage *> _partitions)
      : m_stats(parent),
        partitions(_partitions)
    {}

    // Returns the current number of slots allocated
    int
    size() const
    {
        int total = 0;
        for (auto part : partitions) {
            total += part->size();
        }
        return total;
    }

    // Returns the total capacity of this TBEStorage table
    int
    capacity() const
    {
        int total = 0;
        for (auto part : partitions) {
            total += part->capacity();
        }
        return total;
    }

    // Returns number of slots currently reserved
    int
    reserved() const
    {
        int total = 0;
        for (auto part : partitions) {
            total += part->reserved();
        }
        return total;
    }

    // Returns the number of slots available for objects of a certain type;
    int
    slotsAvailable(int partition) const
    {
        auto generic_slots = partitions[0]->slotsAvailable();
        if (partition) {
            return partitions[partition]->slotsAvailable() +
                generic_slots;
        } else {
            return generic_slots;
        }
    }

    // Returns the TBEStorage utilization
    float utilization() const { return size() / (float)capacity(); }

    // Returns true if slotsAvailable(partition) >= n;
    //     current_time is always ignored
    // This allows this class to be used with check_allocate in SLICC to
    // trigger resource stalls when there are no slots available
    bool
    areNSlotsAvailable(int n, int partition,
                       Tick current_time = 0) const
    {
        return slotsAvailable(partition) >= n;
    }

    // Increase/decrease the number of reserved slots. Having reserved slots
    // reduces the number of slots available for allocation
    void
    incrementReserved(int partition)
    {
        if (partition &&
            partitions[partition]->areNSlotsAvailable(1)) {
            partitions[partition]->incrementReserved();
        } else {
            partitions[0]->incrementReserved();
        }
        m_stats.avg_reserved = reserved();
    }

    void
    decrementReserved(int partition)
    {
        if (partition && (partitions[partition]->reserved() > 0)) {
            partitions[partition]->decrementReserved();
        } else {
            partitions[0]->decrementReserved();
        }
        m_stats.avg_reserved = reserved();
    }

    // Assign a TBETable entry to a free slot and returns the slot number.
    // Notice we don't need any info from TBETable and just track the number
    // of entries assigned to each slot.
    // This funcion requires slotsAvailable() > 0
    int
    addEntryToNewSlot(int partition)
    {
        if (partition && partitions[partition]->areNSlotsAvailable(1)) {
            int part_slot = partitions[partition]->addEntryToNewSlot();

            m_stats.avg_size = size();
            m_stats.avg_util = utilization();

            return part_slot;
        } else {
            int generic_slot = partitions[0]->addEntryToNewSlot();

            m_stats.avg_size = size();
            m_stats.avg_util = utilization();

            return partitions[partition]->capacity() + generic_slot;
        }
    }

    // addEntryToSlot(int) is not supported.

    // Remove an entry from an existing non-empty slot. The slot becomes
    // available again when the number of assigned entries == 0
    void
    removeEntryFromSlot(int slot, int partition)
    {
        auto part_capacity = partitions[partition]->capacity();
        if (slot < part_capacity) {
            partitions[partition]->removeEntryFromSlot(slot);
        } else {
            partitions[0]->removeEntryFromSlot(
                slot - part_capacity);
        }

        m_stats.avg_size = size();
        m_stats.avg_util = utilization();
    }

    // Insert a "retry entry" into the queue
    void
    emplaceRetryEntry(RetryEntry entry)
    {
        m_retryEntries.push_back(entry);
    }

    // Check if a retry is possible
    bool
    hasPossibleRetry()
    {
        auto retry_iter = getNextRetryEntryIter();
        return retry_iter != m_retryEntries.end();
    }

    // Peek what the next thing to retry should be
    // Should only be called if hasPossibleRetry() returns true
    RetryEntry
    popNextRetryEntry()
    {
        auto retry_iter = getNextRetryEntryIter();
        assert(retry_iter != m_retryEntries.end());

        auto entry = *retry_iter;

        m_retryEntries.erase(retry_iter);

        return entry;
    }

  private:
    struct MN_TBEStorageStats : public statistics::Group
    {
        MN_TBEStorageStats(statistics::Group *parent)
          : statistics::Group(parent),
            ADD_STAT(avg_size, "Avg. number of slots allocated"),
            ADD_STAT(avg_util, "Avg. utilization"),
            ADD_STAT(avg_reserved, "Avg. number of slots reserved")
        {}

        // Statistical variables
        statistics::Average avg_size;
        statistics::Average avg_util;
        statistics::Average avg_reserved;
    } m_stats;

    std::vector<TBEStorage *> partitions;

    std::list<RetryEntry> m_retryEntries;

    typename std::list<RetryEntry>::iterator
    getNextRetryEntryIter()
    {
        auto begin_it = m_retryEntries.begin();
        auto end_it = m_retryEntries.end();

        for (auto it = begin_it; it != end_it; it++) {
            if (areNSlotsAvailable(1, it->getisNonSync()))
                return it;
        }

        return end_it;
    }
};

} // namespace CHI

} // namespace ruby

} // namespace gem5

#endif
