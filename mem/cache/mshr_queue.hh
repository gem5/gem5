/*
 * Copyright (c) 2012-2013, 2015-2016, 2018 ARM Limited
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
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

/** @file
 * Declaration of a structure to manage MSHRs.
 */

#ifndef __MEM_CACHE_MSHR_QUEUE_HH__
#define __MEM_CACHE_MSHR_QUEUE_HH__

#include <string>

#include "base/types.hh"
#include "mem/cache/mshr.hh"
#include "mem/cache/queue.hh"
#include "mem/packet.hh"

namespace gem5
{

/**
 * A Class for maintaining a list of pending and allocated memory requests.
 */
class MSHRQueue : public Queue<MSHR>
{
  private:

    /**
     * The number of entries to reserve for future demand accesses.
     * Prevent prefetcher from taking all mshr entries
     */
    const int demandReserve;

  public:

    /**
     * Create a queue with a given number of entries.
     * @param num_entrys The number of entries in this queue.
     * @param reserve The minimum number of entries needed to satisfy
     * any access.
     * @param demand_reserve The minimum number of entries needed to satisfy
     * demand accesses.
     */
    MSHRQueue(const std::string &_label, int num_entries, int reserve,
              int demand_reserve, std::string cache_name);

    /**
     * Allocates a new MSHR for the request and size. This places the request
     * as the first target in the MSHR.
     *
     * @param blk_addr The address of the block.
     * @param blk_size The number of bytes to request.
     * @param pkt The original miss.
     * @param when_ready When should the MSHR be ready to act upon.
     * @param order The logical order of this MSHR
     * @param alloc_on_fill Should the cache allocate a block on fill
     *
     * @return The a pointer to the MSHR allocated.
     *
     * @pre There are free entries.
     */
    MSHR *allocate(Addr blk_addr, unsigned blk_size, PacketPtr pkt,
                   Tick when_ready, Counter order, bool alloc_on_fill);

    /**
     * Deallocate a MSHR and its targets
     */
    void deallocate(MSHR *mshr) override;

    /**
     * Moves the MSHR to the front of the pending list if it is not
     * in service.
     * @param mshr The entry to move.
     */
    void moveToFront(MSHR *mshr);

    /**
     * Adds a delay to the provided MSHR and moves MSHRs that will be
     * ready earlier than this entry to the top of the list
     *
     * @param mshr that needs to be delayed
     * @param delay_ticks ticks of the desired delay
     */
    void delay(MSHR *mshr, Tick delay_ticks);

    /**
     * Mark the given MSHR as in service. This removes the MSHR from the
     * readyList or deallocates the MSHR if it does not expect a response.
     *
     * @param mshr The MSHR to mark in service.
     * @param pending_modified_resp Whether we expect a modified response
     *                              from another cache
     */
    void markInService(MSHR *mshr, bool pending_modified_resp);

    /**
     * Mark an in service entry as pending, used to resend a request.
     * @param mshr The MSHR to resend.
     */
    void markPending(MSHR *mshr);

    /**
     * Deallocate top target, possibly freeing the MSHR
     * @return if MSHR queue is no longer full
     */
    bool forceDeallocateTarget(MSHR *mshr);

    /**
     * Returns true if the pending list is not empty.
     * @return True if there are outstanding requests.
     */
    bool havePending() const
    {
        return !readyList.empty();
    }

    /**
     * Returns true if sufficient mshrs for prefetch.
     * @return True if sufficient mshrs for prefetch.
     */
    bool canPrefetch() const
    {
        // @todo we may want to revisit the +1, currently added to
        // keep regressions unchanged
        return (allocated < numEntries - (numReserve + 1 + demandReserve));
    }
};

} // namespace gem5

#endif //__MEM_CACHE_MSHR_QUEUE_HH__
