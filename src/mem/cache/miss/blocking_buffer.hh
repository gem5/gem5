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

#include "mem/cache/miss/mshr.hh"
#include "base/statistics.hh"

class BaseCache;
class BasePrefetcher;

/**
 * Miss and writeback storage for a blocking cache.
 */
class BlockingBuffer
{
protected:
    /** Miss storage. */
    MSHR miss;
    /** WB storage. */
    MSHR wb;

    //Params

    /** Allocate on write misses. */
    const bool writeAllocate;

    /** Pointer to the parent cache. */
    BaseCache* cache;

    BasePrefetcher* prefetcher;

    /** Block size of the parent cache. */
    int blkSize;

    // Statistics
    /**
     * @addtogroup CacheStatistics
     * @{
     */
    /** Number of blocks written back per thread. */
    Stats::Vector<> writebacks;

    /**
     * @}
     */

public:
    /**
     * Builds and initializes this buffer.
     * @param write_allocate If true, treat write misses the same as reads.
     */
    BlockingBuffer(bool write_allocate)
        : writeAllocate(write_allocate)
    {
    }

    /**
     * Register statistics for this object.
     * @param name The name of the parent cache.
     */
    void regStats(const std::string &name);

    /**
     * Called by the parent cache to set the back pointer.
     * @param _cache A pointer to the parent cache.
     */
    void setCache(BaseCache *_cache);

    void setPrefetcher(BasePrefetcher *_prefetcher);

    /**
     * Handle a cache miss properly. Requests the bus and marks the cache as
     * blocked.
     * @param req The request that missed in the cache.
     * @param blk_size The block size of the cache.
     * @param time The time the miss is detected.
     */
    void handleMiss(Packet * &pkt, int blk_size, Tick time);

    /**
     * Fetch the block for the given address and buffer the given target.
     * @param addr The address to fetch.
     * @param asid The address space of the address.
     * @param blk_size The block size of the cache.
     * @param time The time the miss is detected.
     * @param target The target for the fetch.
     */
    MSHR* fetchBlock(Addr addr, int asid, int blk_size, Tick time,
                     Packet * &target)
    {
        fatal("Unimplemented");
    }

    /**
     * Selects a outstanding request to service.
     * @return The request to service, NULL if none found.
     */
    Packet * getPacket();

    /**
     * Set the command to the given bus command.
     * @param req The request to update.
     * @param cmd The bus command to use.
     */
    void setBusCmd(Packet * &pkt, Packet::Command cmd);

    /**
     * Restore the original command in case of a bus transmission error.
     * @param req The request to reset.
     */
    void restoreOrigCmd(Packet * &pkt);

    /**
     * Marks a request as in service (sent on the bus). This can have side
     * effect since storage for no response commands is deallocated once they
     * are successfully sent.
     * @param req The request that was sent on the bus.
     */
    void markInService(Packet * &pkt);

    /**
     * Frees the resources of the request and unblock the cache.
     * @param req The request that has been satisfied.
     * @param time The time when the request is satisfied.
     */
    void handleResponse(Packet * &pkt, Tick time);

    /**
     * Removes all outstanding requests for a given thread number. If a request
     * has been sent to the bus, this function removes all of its targets.
     * @param req->getThreadNum()ber The thread number of the requests to squash.
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
    MSHR* findMSHR(Addr addr, int asid)
    {
        if (miss.addr == addr && miss.pkt)
            return &miss;
        return NULL;
    }

    /**
     * Searches for the supplied address in the write buffer.
     * @param addr The address to look for.
     * @param asid The address space id.
     * @param writes List of pointers to the matching writes.
     * @return True if there is a matching write.
     */
    bool findWrites(Addr addr, int asid, std::vector<MSHR*>& writes)
    {
        if (wb.addr == addr && wb.pkt) {
            writes.push_back(&wb);
            return true;
        }
        return false;
    }



    /**
     * Perform a writeback of dirty data to the given address.
     * @param addr The address to write to.
     * @param asid The address space id.
     * @param size The number of bytes to write.
     * @param data The data to write, can be NULL.
     * @param compressed True if the data is compressed.
     */
    void doWriteback(Addr addr, int asid,
                     int size, uint8_t *data, bool compressed);

    /**
     * Perform a writeback request.
     * @param req The writeback request.
     */
    void doWriteback(Packet * &pkt);

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
     * @param req The target to add.
     */
    void addTarget(MSHR *mshr, Packet * &pkt)
    {
        fatal("Shouldn't call this on a blocking buffer.");
    }

    /**
     * Dummy implmentation.
     */
    MSHR* allocateTargetList(Addr addr, int asid)
    {
        fatal("Unimplemented");
    }
};

#endif // __BLOCKING_BUFFER_HH__
