/*
 * Copyright (c) 2003-2006 The Regents of The University of Michigan
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
 * Authors: Steve Reinhardt
 */

/**
 * @file
 * MissBuffer declaration.
 */

#ifndef __MISS_BUFFER_HH__
#define __MISS_BUFFER_HH__

class BaseCache;
class BasePrefetcher;
class MSHR;

/**
 * Abstract base class for cache miss buffering.
 */
class MissBuffer
{
  protected:
    /** True if the cache should allocate on a write miss. */
    const bool writeAllocate;

    /** Pointer to the parent cache. */
    BaseCache *cache;

    /** The Prefetcher */
    BasePrefetcher *prefetcher;

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
    MissBuffer(bool write_allocate)
        : writeAllocate(write_allocate)
    {
    }

    virtual ~MissBuffer() {}

    /**
     * Called by the parent cache to set the back pointer.
     * @param _cache A pointer to the parent cache.
     */
    void setCache(BaseCache *_cache);

    void setPrefetcher(BasePrefetcher *_prefetcher);

    /**
     * Register statistics for this object.
     * @param name The name of the parent cache.
     */
    virtual void regStats(const std::string &name);

    /**
     * Handle a cache miss properly. Either allocate an MSHR for the request,
     * or forward it through the write buffer.
     * @param pkt The request that missed in the cache.
     * @param blk_size The block size of the cache.
     * @param time The time the miss is detected.
     */
    virtual void handleMiss(PacketPtr &pkt, int blk_size, Tick time) = 0;

    /**
     * Fetch the block for the given address and buffer the given target.
     * @param addr The address to fetch.
     * @param asid The address space of the address.
     * @param blk_size The block size of the cache.
     * @param time The time the miss is detected.
     * @param target The target for the fetch.
     */
    virtual MSHR *fetchBlock(Addr addr, int blk_size, Tick time,
                             PacketPtr &target) = 0;

    /**
     * Selects a outstanding request to service.
     * @return The request to service, NULL if none found.
     */
    virtual PacketPtr getPacket() = 0;

    /**
     * Set the command to the given bus command.
     * @param pkt The request to update.
     * @param cmd The bus command to use.
     */
    virtual void setBusCmd(PacketPtr &pkt, MemCmd cmd) = 0;

    /**
     * Restore the original command in case of a bus transmission error.
     * @param pkt The request to reset.
     */
    virtual void restoreOrigCmd(PacketPtr &pkt) = 0;

    /**
     * Marks a request as in service (sent on the bus). This can have side
     * effect since storage for no response commands is deallocated once they
     * are successfully sent.
     * @param pkt The request that was sent on the bus.
     */
    virtual void markInService(PacketPtr &pkt, MSHR* mshr) = 0;

    /**
     * Collect statistics and free resources of a satisfied request.
     * @param pkt The request that has been satisfied.
     * @param time The time when the request is satisfied.
     */
    virtual void handleResponse(PacketPtr &pkt, Tick time) = 0;

    /**
     * Removes all outstanding requests for a given thread number. If a request
     * has been sent to the bus, this function removes all of its targets.
     * @param threadNum The thread number of the requests to squash.
     */
    virtual void squash(int threadNum) = 0;

    /**
     * Return the current number of outstanding misses.
     * @return the number of outstanding misses.
     */
    virtual int getMisses() = 0;

    /**
     * Searches for the supplied address in the miss queue.
     * @param addr The address to look for.
     * @param asid The address space id.
     * @return The MSHR that contains the address, NULL if not found.
     * @warning Currently only searches the miss queue. If non write allocate
     * might need to search the write buffer for coherence.
     */
    virtual MSHR* findMSHR(Addr addr) = 0;

    /**
     * Searches for the supplied address in the write buffer.
     * @param addr The address to look for.
     * @param asid The address space id.
     * @param writes The list of writes that match the address.
     * @return True if any writes are found
     */
    virtual bool findWrites(Addr addr, std::vector<MSHR*>& writes) = 0;

    /**
     * Perform a writeback of dirty data to the given address.
     * @param addr The address to write to.
     * @param asid The address space id.
     * @param xc The execution context of the address space.
     * @param size The number of bytes to write.
     * @param data The data to write, can be NULL.
     * @param compressed True if the data is compressed.
     */
    virtual void doWriteback(Addr addr, int size, uint8_t *data,
                             bool compressed) = 0;

    /**
     * Perform the given writeback request.
     * @param pkt The writeback request.
     */
    virtual void doWriteback(PacketPtr &pkt) = 0;

    /**
     * Returns true if there are outstanding requests.
     * @return True if there are outstanding requests.
     */
    virtual bool havePending() = 0;

    /**
     * Add a target to the given MSHR. This assumes it is in the miss queue.
     * @param mshr The mshr to add a target to.
     * @param pkt The target to add.
     */
    virtual void addTarget(MSHR *mshr, PacketPtr &pkt) = 0;

    /**
     * Allocate a MSHR to hold a list of targets to a block involved in a copy.
     * If the block is marked done then the MSHR already holds the data to
     * fill the block. Otherwise the block needs to be fetched.
     * @param addr The address to buffer.
     * @param asid The address space ID.
     * @return A pointer to the allocated MSHR.
     */
    virtual MSHR* allocateTargetList(Addr addr) = 0;
};

#endif //__MISS_BUFFER_HH__
