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
 * Miss and writeback queue declarations.
 */

#ifndef __MISS_QUEUE_HH__
#define __MISS_QUEUE_HH__

#include <vector>

#include "mem/cache/miss/mshr.hh"
#include "mem/cache/miss/mshr_queue.hh"
#include "base/statistics.hh"

class BaseCache;
class BasePrefetcher;
/**
 * Manages cache misses and writebacks. Contains MSHRs to store miss data
 * and the writebuffer for writes/writebacks.
 * @todo need to handle data on writes better (encapsulate).
 * @todo need to make replacements/writebacks happen in Cache::access
 */
class MissQueue
{
  protected:
    /** The MSHRs. */
    MSHRQueue mq;
    /** Write Buffer. */
    MSHRQueue wb;

    // PARAMTERS

    /** The number of MSHRs in the miss queue. */
    const int numMSHR;
    /** The number of targets for each MSHR. */
    const int numTarget;
    /** The number of write buffers. */
    const int writeBuffers;
    /** True if the cache should allocate on a write miss. */
    const bool writeAllocate;
    /** Pointer to the parent cache. */
    BaseCache* cache;

    /** The Prefetcher */
    BasePrefetcher *prefetcher;

    /** The block size of the parent cache. */
    int blkSize;

    /** Increasing order number assigned to each incoming request. */
    uint64_t order;

    bool prefetchMiss;

    // Statistics
    /**
     * @addtogroup CacheStatistics
     * @{
     */
    /** Number of blocks written back per thread. */
    Stats::Vector<> writebacks;

    /** Number of misses that hit in the MSHRs per command and thread. */
    Stats::Vector<> mshr_hits[NUM_MEM_CMDS];
    /** Demand misses that hit in the MSHRs. */
    Stats::Formula demandMshrHits;
    /** Total number of misses that hit in the MSHRs. */
    Stats::Formula overallMshrHits;

    /** Number of misses that miss in the MSHRs, per command and thread. */
    Stats::Vector<> mshr_misses[NUM_MEM_CMDS];
    /** Demand misses that miss in the MSHRs. */
    Stats::Formula demandMshrMisses;
    /** Total number of misses that miss in the MSHRs. */
    Stats::Formula overallMshrMisses;

    /** Number of misses that miss in the MSHRs, per command and thread. */
    Stats::Vector<> mshr_uncacheable[NUM_MEM_CMDS];
    /** Total number of misses that miss in the MSHRs. */
    Stats::Formula overallMshrUncacheable;

    /** Total cycle latency of each MSHR miss, per command and thread. */
    Stats::Vector<> mshr_miss_latency[NUM_MEM_CMDS];
    /** Total cycle latency of demand MSHR misses. */
    Stats::Formula demandMshrMissLatency;
    /** Total cycle latency of overall MSHR misses. */
    Stats::Formula overallMshrMissLatency;

    /** Total cycle latency of each MSHR miss, per command and thread. */
    Stats::Vector<> mshr_uncacheable_lat[NUM_MEM_CMDS];
    /** Total cycle latency of overall MSHR misses. */
    Stats::Formula overallMshrUncacheableLatency;

    /** The total number of MSHR accesses per command and thread. */
    Stats::Formula mshrAccesses[NUM_MEM_CMDS];
    /** The total number of demand MSHR accesses. */
    Stats::Formula demandMshrAccesses;
    /** The total number of MSHR accesses. */
    Stats::Formula overallMshrAccesses;

    /** The miss rate in the MSHRs pre command and thread. */
    Stats::Formula mshrMissRate[NUM_MEM_CMDS];
    /** The demand miss rate in the MSHRs. */
    Stats::Formula demandMshrMissRate;
    /** The overall miss rate in the MSHRs. */
    Stats::Formula overallMshrMissRate;

    /** The average latency of an MSHR miss, per command and thread. */
    Stats::Formula avgMshrMissLatency[NUM_MEM_CMDS];
    /** The average latency of a demand MSHR miss. */
    Stats::Formula demandAvgMshrMissLatency;
    /** The average overall latency of an MSHR miss. */
    Stats::Formula overallAvgMshrMissLatency;

    /** The average latency of an MSHR miss, per command and thread. */
    Stats::Formula avgMshrUncacheableLatency[NUM_MEM_CMDS];
    /** The average overall latency of an MSHR miss. */
    Stats::Formula overallAvgMshrUncacheableLatency;

    /** The number of times a thread hit its MSHR cap. */
    Stats::Vector<> mshr_cap_events;
    /** The number of times software prefetches caused the MSHR to block. */
    Stats::Vector<> soft_prefetch_mshr_full;

    Stats::Scalar<> mshr_no_allocate_misses;

    /**
     * @}
     */

  private:
    /** Pointer to the MSHR that has no targets. */
    MSHR* noTargetMSHR;

    /**
     * Allocate a new MSHR to handle the provided miss.
     * @param req The miss to buffer.
     * @param size The number of bytes to fetch.
     * @param time The time the miss occurs.
     * @return A pointer to the new MSHR.
     */
    MSHR* allocateMiss(Packet * &pkt, int size, Tick time);

    /**
     * Allocate a new WriteBuffer to handle the provided write.
     * @param req The write to handle.
     * @param size The number of bytes to write.
     * @param time The time the write occurs.
     * @return A pointer to the new write buffer.
     */
    MSHR* allocateWrite(Packet * &pkt, int size, Tick time);

  public:
    /**
     * Simple Constructor. Initializes all needed internal storage and sets
     * parameters.
     * @param numMSHRs The number of outstanding misses to handle.
     * @param numTargets The number of outstanding targets to each miss.
     * @param write_buffers The number of outstanding writes to handle.
     * @param write_allocate If true, treat write misses the same as reads.
     */
    MissQueue(int numMSHRs, int numTargets, int write_buffers,
              bool write_allocate, bool prefetch_miss);

    /**
     * Deletes all allocated internal storage.
     */
    ~MissQueue();

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
     * Handle a cache miss properly. Either allocate an MSHR for the request,
     * or forward it through the write buffer.
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
                     Packet * &target);

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
     * Collect statistics and free resources of a satisfied request.
     * @param req The request that has been satisfied.
     * @param time The time when the request is satisfied.
     */
    void handleResponse(Packet * &pkt, Tick time);

    /**
     * Removes all outstanding requests for a given thread number. If a request
     * has been sent to the bus, this function removes all of its targets.
     * @param req->getThreadNum()ber The thread number of the requests to squash.
     */
    void squash(int req->getThreadNum()ber);

    /**
     * Return the current number of outstanding misses.
     * @return the number of outstanding misses.
     */
    int getMisses()
    {
        return mq.getAllocatedTargets();
    }

    /**
     * Searches for the supplied address in the miss queue.
     * @param addr The address to look for.
     * @param asid The address space id.
     * @return The MSHR that contains the address, NULL if not found.
     * @warning Currently only searches the miss queue. If non write allocate
     * might need to search the write buffer for coherence.
     */
    MSHR* findMSHR(Addr addr, int asid) const;

    /**
     * Searches for the supplied address in the write buffer.
     * @param addr The address to look for.
     * @param asid The address space id.
     * @param writes The list of writes that match the address.
     * @return True if any writes are found
     */
    bool findWrites(Addr addr, int asid, std::vector<MSHR*>& writes) const;

    /**
     * Perform a writeback of dirty data to the given address.
     * @param addr The address to write to.
     * @param asid The address space id.
     * @param xc The execution context of the address space.
     * @param size The number of bytes to write.
     * @param data The data to write, can be NULL.
     * @param compressed True if the data is compressed.
     */
    void doWriteback(Addr addr, int asid,
                     int size, uint8_t *data, bool compressed);

    /**
     * Perform the given writeback request.
     * @param req The writeback request.
     */
    void doWriteback(Packet * &pkt);

    /**
     * Returns true if there are outstanding requests.
     * @return True if there are outstanding requests.
     */
    bool havePending();

    /**
     * Add a target to the given MSHR. This assumes it is in the miss queue.
     * @param mshr The mshr to add a target to.
     * @param req The target to add.
     */
    void addTarget(MSHR *mshr, Packet * &pkt)
    {
        mq.allocateTarget(mshr, pkt);
    }

    /**
     * Allocate a MSHR to hold a list of targets to a block involved in a copy.
     * If the block is marked done then the MSHR already holds the data to
     * fill the block. Otherwise the block needs to be fetched.
     * @param addr The address to buffer.
     * @param asid The address space ID.
     * @return A pointer to the allocated MSHR.
     */
    MSHR* allocateTargetList(Addr addr, int asid);

};

#endif //__MISS_QUEUE_HH__
