/*
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
 */

/** @file
 * Declaration of a structure to manage MSHRs.
 */

#ifndef __MSHR_QUEUE_HH__
#define __MSHR_QUEUE_HH__

#include <vector>
#include "mem/cache/miss/mshr.hh"

/**
 * A Class for maintaining a list of pending and allocated memory requests.
 */
class MSHRQueue {
  private:
    /**  MSHR storage. */
    MSHR* registers;
    /** Holds pointers to all allocated MSHRs. */
    MSHR::List allocatedList;
    /** Holds pointers to MSHRs that haven't been sent to the bus. */
    MSHR::List pendingList;
    /** Holds non allocated MSHRs. */
    MSHR::List freeList;

    // Parameters
    /**
     * The total number of MSHRs in this queue. This number is set as the
     * number of MSHRs requested plus (numReserve - 1). This allows for
     * the same number of effective MSHRs while still maintaining the reserve.
     */
    const int numMSHRs;

    /**
     * The number of MSHRs to hold in reserve. This is needed because copy
     * operations can allocate upto 4 MSHRs at one time.
     */
    const int numReserve;

  public:
    /** The number of allocated MSHRs. */
    int allocated;
    /** The number of MSHRs that have been forwarded to the bus. */
    int inServiceMSHRs;
    /** The number of targets waiting for response. */
    int allocatedTargets;

    /**
     * Create a queue with a given number of MSHRs.
     * @param num_mshrs The number of MSHRs in this queue.
     * @param reserve The minimum number of MSHRs needed to satisfy any access.
     */
    MSHRQueue(int num_mshrs, int reserve = 1);

    /** Destructor */
    ~MSHRQueue();

    /**
     * Find the first MSHR that matches the provide address and asid.
     * @param addr The address to find.
     * @param asid The address space id.
     * @return Pointer to the matching MSHR, null if not found.
     */
    MSHR* findMatch(Addr addr) const;

    /**
     * Find and return all the matching MSHRs in the provided vector.
     * @param addr The address to find.
     * @param asid The address space ID.
     * @param matches The vector to return pointers to the matching MSHRs.
     * @return True if any matches are found, false otherwise.
     * @todo Typedef the vector??
     */
    bool findMatches(Addr addr, std::vector<MSHR*>& matches) const;

    /**
     * Find any pending requests that overlap the given request.
     * @param pkt The request to find.
     * @return A pointer to the earliest matching MSHR.
     */
    MSHR* findPending(PacketPtr &pkt) const;

    /**
     * Allocates a new MSHR for the request and size. This places the request
     * as the first target in the MSHR.
     * @param pkt The request to handle.
     * @param size The number in bytes to fetch from memory.
     * @return The a pointer to the MSHR allocated.
     *
     * @pre There are free MSHRs.
     */
    MSHR* allocate(PacketPtr &pkt, int size = 0);

    /**
     * Allocate a read request for the given address, and places the given
     * target on the target list.
     * @param addr The address to fetch.
     * @param asid The address space for the fetch.
     * @param size The number of bytes to request.
     * @param target The first target for the request.
     * @return Pointer to the new MSHR.
     */
    MSHR* allocateFetch(Addr addr, int size, PacketPtr &target);

    /**
     * Allocate a target list for the given address.
     * @param addr The address to fetch.
     * @param asid The address space for the fetch.
     * @param size The number of bytes to request.
     * @return Pointer to the new MSHR.
     */
    MSHR* allocateTargetList(Addr addr, int size);

    /**
     * Removes the given MSHR from the queue. This places the MSHR on the
     * free list.
     * @param mshr
     */
    void deallocate(MSHR* mshr);

    /**
     * Allocates a target to the given MSHR. Used to keep track of the number
     * of outstanding targets.
     * @param mshr The MSHR to allocate the target to.
     * @param pkt The target request.
     */
    void allocateTarget(MSHR* mshr, PacketPtr &pkt)
    {
        mshr->allocateTarget(pkt);
        allocatedTargets += 1;
    }

    /**
     * Remove a MSHR from the queue. Returns an iterator into the allocatedList
     * for faster squash implementation.
     * @param mshr The MSHR to remove.
     * @return An iterator to the next entry in the allocatedList.
     */
    MSHR::Iterator deallocateOne(MSHR* mshr);

    /**
     * Moves the MSHR to the front of the pending list if it is not in service.
     * @param mshr The mshr to move.
     */
    void moveToFront(MSHR *mshr);

    /**
     * Mark the given MSHR as in service. This removes the MSHR from the
     * pendingList. Deallocates the MSHR if it does not expect a response.
     * @param mshr The MSHR to mark in service.
     */
    void markInService(MSHR* mshr);

    /**
     * Mark an in service mshr as pending, used to resend a request.
     * @param mshr The MSHR to resend.
     * @param cmd The command to resend.
     */
    void markPending(MSHR* mshr, MemCmd cmd);

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
        return !pendingList.empty();
    }

    /**
     * Returns true if there are no free MSHRs.
     * @return True if this queue is full.
     */
    bool isFull() const
    {
        return (allocated > numMSHRs - numReserve);
    }

    /**
     * Returns the request at the head of the pendingList.
     * @return The next request to service.
     */
    PacketPtr getReq() const
    {
        if (pendingList.empty()) {
            return NULL;
        }
        MSHR* mshr = pendingList.front();
        return mshr->pkt;
    }

    /**
     * Returns the number of outstanding targets.
     * @return the number of allocated targets.
     */
    int getAllocatedTargets() const
    {
        return allocatedTargets;
    }

};

#endif //__MSHR_QUEUE_HH__
