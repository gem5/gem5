/*
 * Copyright (c) 2012-2013 ARM Limited
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
 *
 * Authors: Erik Hallnor
 *          Andreas Sandberg
 */

/** @file
 * Declaration of a structure to manage MSHRs.
 */

#ifndef __MEM__CACHE__MISS__MSHR_QUEUE_HH__
#define __MEM__CACHE__MISS__MSHR_QUEUE_HH__

#include <vector>

#include "mem/cache/mshr.hh"
#include "mem/packet.hh"
#include "sim/drain.hh"

/**
 * A Class for maintaining a list of pending and allocated memory requests.
 */
class MSHRQueue : public Drainable
{
  private:
    /** Local label (for functional print requests) */
    const std::string label;

    // Parameters
    /**
     * The total number of entries in this queue. This number is set as the
     * number of entries requested plus (numReserve - 1). This allows for
     * the same number of effective entries while still maintaining the reserve.
     */
    const int numEntries;

    /**
     * The number of entries to hold in reserve. This is needed because copy
     * operations can allocate upto 4 entries at one time.
     */
    const int numReserve;

    /**  MSHR storage. */
    std::vector<MSHR> registers;
    /** Holds pointers to all allocated entries. */
    MSHR::List allocatedList;
    /** Holds pointers to entries that haven't been sent to the bus. */
    MSHR::List readyList;
    /** Holds non allocated entries. */
    MSHR::List freeList;

    /** Drain manager to inform of a completed drain */
    DrainManager *drainManager;

    MSHR::Iterator addToReadyList(MSHR *mshr);


  public:
    /** The number of allocated entries. */
    int allocated;
    /** The number of entries that have been forwarded to the bus. */
    int inServiceEntries;
    /** The index of this queue within the cache (MSHR queue vs. write
     * buffer). */
    const int index;

    /**
     * Create a queue with a given number of entries.
     * @param num_entrys The number of entries in this queue.
     * @param reserve The minimum number of entries needed to satisfy
     * any access.
     */
    MSHRQueue(const std::string &_label, int num_entries, int reserve,
              int index);

    /**
     * Find the first MSHR that matches the provided address.
     * @param addr The address to find.
     * @param is_secure True if the target memory space is secure.
     * @return Pointer to the matching MSHR, null if not found.
     */
    MSHR *findMatch(Addr addr, bool is_secure) const;

    /**
     * Find and return all the matching entries in the provided vector.
     * @param addr The address to find.
     * @param is_secure True if the target memory space is secure.
     * @param matches The vector to return pointers to the matching entries.
     * @return True if any matches are found, false otherwise.
     * @todo Typedef the vector??
     */
    bool findMatches(Addr addr, bool is_secure,
                     std::vector<MSHR*>& matches) const;

    /**
     * Find any pending requests that overlap the given request.
     * @param pkt The request to find.
     * @param is_secure True if the target memory space is secure.
     * @return A pointer to the earliest matching MSHR.
     */
    MSHR *findPending(Addr addr, int size, bool is_secure) const;

    bool checkFunctional(PacketPtr pkt, Addr blk_addr);

    /**
     * Allocates a new MSHR for the request and size. This places the request
     * as the first target in the MSHR.
     * @param pkt The request to handle.
     * @param size The number in bytes to fetch from memory.
     * @return The a pointer to the MSHR allocated.
     *
     * @pre There are free entries.
     */
    MSHR *allocate(Addr addr, int size, PacketPtr &pkt,
                   Tick when, Counter order);

    /**
     * Removes the given MSHR from the queue. This places the MSHR on the
     * free list.
     * @param mshr
     */
    void deallocate(MSHR *mshr);

    /**
     * Remove a MSHR from the queue. Returns an iterator into the
     * allocatedList for faster squash implementation.
     * @param mshr The MSHR to remove.
     * @return An iterator to the next entry in the allocatedList.
     */
    MSHR::Iterator deallocateOne(MSHR *mshr);

    /**
     * Moves the MSHR to the front of the pending list if it is not
     * in service.
     * @param mshr The entry to move.
     */
    void moveToFront(MSHR *mshr);

    /**
     * Mark the given MSHR as in service. This removes the MSHR from the
     * readyList. Deallocates the MSHR if it does not expect a response.
     * @param mshr The MSHR to mark in service.
     */
    void markInService(MSHR *mshr, PacketPtr pkt);

    /**
     * Mark an in service entry as pending, used to resend a request.
     * @param mshr The MSHR to resend.
     */
    void markPending(MSHR *mshr);

    /**
     * Squash outstanding requests with the given thread number. If a request
     * is in service, just squashes the targets.
     * @param threadNum The thread to squash.
     */
    void squash(int threadNum);

    /**
     * Returns true if the pending list is not empty.
     * @return True if there are outstanding requests.
     */
    bool havePending() const
    {
        return !readyList.empty();
    }

    /**
     * Returns true if there are no free entries.
     * @return True if this queue is full.
     */
    bool isFull() const
    {
        return (allocated > numEntries - numReserve);
    }

    /**
     * Returns the MSHR at the head of the readyList.
     * @return The next request to service.
     */
    MSHR *getNextMSHR() const
    {
        if (readyList.empty() || readyList.front()->readyTime > curTick()) {
            return NULL;
        }
        return readyList.front();
    }

    Tick nextMSHRReadyTime() const
    {
        return readyList.empty() ? MaxTick : readyList.front()->readyTime;
    }

    unsigned int drain(DrainManager *dm);
};

#endif //__MEM__CACHE__MISS__MSHR_QUEUE_HH__
