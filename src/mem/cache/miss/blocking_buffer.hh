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

/**
 * @file
 * Declaration of a simple buffer for a blocking cache.
 */

#ifndef __BLOCKING_BUFFER_HH__
#define __BLOCKING_BUFFER_HH__

#include <vector>

#include "base/misc.hh" // for fatal()
#include "mem/cache/miss/miss_buffer.hh"
#include "mem/cache/miss/mshr.hh"

/**
 * Miss and writeback storage for a blocking cache.
 */
class BlockingBuffer : public MissBuffer
{
protected:
    /** Miss storage. */
    MSHR miss;
    /** WB storage. */
    MSHR wb;

public:
    /**
     * Builds and initializes this buffer.
     * @param write_allocate If true, treat write misses the same as reads.
     */
    BlockingBuffer(bool write_allocate)
        : MissBuffer(write_allocate)
    {
    }

    /**
     * Register statistics for this object.
     * @param name The name of the parent cache.
     */
    void regStats(const std::string &name);

    /**
     * Handle a cache miss properly. Requests the bus and marks the cache as
     * blocked.
     * @param pkt The request that missed in the cache.
     * @param blk_size The block size of the cache.
     * @param time The time the miss is detected.
     */
    void handleMiss(PacketPtr &pkt, int blk_size, Tick time);

    /**
     * Fetch the block for the given address and buffer the given target.
     * @param addr The address to fetch.
     * @param asid The address space of the address.
     * @param blk_size The block size of the cache.
     * @param time The time the miss is detected.
     * @param target The target for the fetch.
     */
    MSHR* fetchBlock(Addr addr, int blk_size, Tick time,
                     PacketPtr &target)
    {
        fatal("Unimplemented");
        M5_DUMMY_RETURN
    }

    /**
     * Selects a outstanding request to service.
     * @return The request to service, NULL if none found.
     */
    PacketPtr getPacket();

    /**
     * Set the command to the given bus command.
     * @param pkt The request to update.
     * @param cmd The bus command to use.
     */
    void setBusCmd(PacketPtr &pkt, MemCmd cmd);

    /**
     * Restore the original command in case of a bus transmission error.
     * @param pkt The request to reset.
     */
    void restoreOrigCmd(PacketPtr &pkt);

    /**
     * Marks a request as in service (sent on the bus). This can have side
     * effect since storage for no response commands is deallocated once they
     * are successfully sent.
     * @param pkt The request that was sent on the bus.
     */
    void markInService(PacketPtr &pkt, MSHR* mshr);

    /**
     * Frees the resources of the request and unblock the cache.
     * @param pkt The request that has been satisfied.
     * @param time The time when the request is satisfied.
     */
    void handleResponse(PacketPtr &pkt, Tick time);

    /**
     * Removes all outstanding requests for a given thread number. If a request
     * has been sent to the bus, this function removes all of its targets.
     * @param threadNum The thread number of the requests to squash.
     */
    void squash(int threadNum);

    /**
     * Return the current number of outstanding misses.
     * @return the number of outstanding misses.
     */
    int getMisses()
    {
        return miss.getNumTargets();
    }

    /**
     * Searches for the supplied address in the miss "queue".
     * @param addr The address to look for.
     * @param asid The address space id.
     * @return A pointer to miss if it matches.
     */
    MSHR* findMSHR(Addr addr);

    /**
     * Searches for the supplied address in the write buffer.
     * @param addr The address to look for.
     * @param asid The address space id.
     * @param writes List of pointers to the matching writes.
     * @return True if there is a matching write.
     */
    bool findWrites(Addr addr, std::vector<MSHR*>& writes);

    /**
     * Perform a writeback of dirty data to the given address.
     * @param addr The address to write to.
     * @param asid The address space id.
     * @param size The number of bytes to write.
     * @param data The data to write, can be NULL.
     * @param compressed True if the data is compressed.
     */
    void doWriteback(Addr addr,
                     int size, uint8_t *data, bool compressed);

    /**
     * Perform a writeback request.
     * @param pkt The writeback request.
     */
    void doWriteback(PacketPtr &pkt);

    /**
     * Returns true if there are outstanding requests.
     * @return True if there are outstanding requests.
     */
    bool havePending()
    {
        return !miss.inService || !wb.inService;
    }

    /**
     * Add a target to the given MSHR. This assumes it is in the miss queue.
     * @param mshr The mshr to add a target to.
     * @param pkt The target to add.
     */
    void addTarget(MSHR *mshr, PacketPtr &pkt)
    {
        fatal("Shouldn't call this on a blocking buffer.");
    }

    /**
     * Dummy implmentation.
     */
    MSHR* allocateTargetList(Addr addr)
    {
        fatal("Unimplemented");
        M5_DUMMY_RETURN
    }
};

#endif // __BLOCKING_BUFFER_HH__
