/*
 * Copyright (c) 2015-2016 ARM Limited
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

/**
 * @file Declaration of a queue structure to manage uncacheable write
 * and writebacks.
 */

#ifndef __MEM_CACHE_WRITE_QUEUE_HH__
#define __MEM_CACHE_WRITE_QUEUE_HH__

#include <string>

#include "base/types.hh"
#include "mem/cache/queue.hh"
#include "mem/cache/write_queue_entry.hh"
#include "mem/packet.hh"

namespace gem5
{

/**
 * A write queue for all eviction packets, i.e. writebacks and clean
 * evictions, as well as uncacheable writes.
 */
class WriteQueue : public Queue<WriteQueueEntry>
{
  public:
    /**
     * Create a write queue with a given number of entries.
     * @param num_entries The number of entries in this queue.
     * @param reserve The maximum number of entries needed to satisfy
     *        any access.
     */
    WriteQueue(const std::string &_label, int num_entries, int reserve,
               const std::string &name);

    /**
     * Allocates a new WriteQueueEntry for the request and size. This
     * places the request as the first target in the WriteQueueEntry.
     *
     * @param blk_addr The address of the block.
     * @param blk_size The number of bytes to request.
     * @param pkt The original write.
     * @param when_ready When is the WriteQueueEntry be ready to act upon.
     * @param order The logical order of this WriteQueueEntry
     *
     * @return The a pointer to the WriteQueueEntry allocated.
     *
     * @pre There are free entries.
     */
    WriteQueueEntry *allocate(Addr blk_addr, unsigned blk_size, PacketPtr pkt,
                              Tick when_ready, Counter order);

    /**
     * Mark the given entry as in service. This removes the entry from
     * the readyList or deallocates the entry if it does not expect a
     * response (writeback/eviction rather than an uncacheable write).
     *
     * @param entry The entry to mark in service.
     */
    void markInService(WriteQueueEntry *entry);
};

} // namespace gem5

#endif //__MEM_CACHE_WRITE_QUEUE_HH__
