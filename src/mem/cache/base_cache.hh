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
 *          Steve Reinhardt
 *          Ron Dreslinski
 */

/**
 * @file
 * Declares a basic cache interface BaseCache.
 */

#ifndef __BASE_CACHE_HH__
#define __BASE_CACHE_HH__

#include <vector>
#include <string>
#include <list>
#include <inttypes.h>

#include "base/misc.hh"
#include "base/statistics.hh"
#include "base/trace.hh"
#include "mem/cache/miss/mshr_queue.hh"
#include "mem/mem_object.hh"
#include "mem/packet.hh"
#include "mem/tport.hh"
#include "mem/request.hh"
#include "sim/eventq.hh"
#include "sim/sim_exit.hh"

/**
 * Reasons for Caches to be Blocked.
 */
enum BlockedCause{
    Blocked_NoMSHRs,
    Blocked_NoTargets,
    Blocked_NoWBBuffers,
    Blocked_Coherence,
    NUM_BLOCKED_CAUSES
};

/**
 * Reasons for cache to request a bus.
 */
enum RequestCause{
    Request_MSHR,
    Request_WB,
    Request_Coherence,
    Request_PF
};

class MSHR;
/**
 * A basic cache interface. Implements some common functions for speed.
 */
class BaseCache : public MemObject
{
    class CachePort : public SimpleTimingPort
    {
      public:
        BaseCache *cache;

      protected:
        Event *responseEvent;

        CachePort(const std::string &_name, BaseCache *_cache);

        virtual void recvStatusChange(Status status);

        virtual int deviceBlockSize();

        bool recvRetryCommon();

      public:
        void setOtherPort(CachePort *_otherPort) { otherPort = _otherPort; }

        void setBlocked();

        void clearBlocked();

        void checkAndSendFunctional(PacketPtr pkt);

        CachePort *otherPort;

        bool blocked;

        bool waitingOnRetry;

        bool mustSendRetry;

        /**
         * Bit vector for the outstanding requests for the master interface.
         */
        uint8_t requestCauses;

        bool isBusRequested() { return requestCauses != 0; }

        void requestBus(RequestCause cause, Tick time)
        {
            DPRINTF(Cache, "Asserting bus request for cause %d\n", cause);
            if (!isBusRequested() && !waitingOnRetry) {
                assert(!sendEvent->scheduled());
                sendEvent->schedule(time);
            }
            requestCauses |= (1 << cause);
        }

        void deassertBusRequest(RequestCause cause)
        {
            DPRINTF(Cache, "Deasserting bus request for cause %d\n", cause);
            requestCauses &= ~(1 << cause);
        }

        void respond(PacketPtr pkt, Tick time) {
            schedSendTiming(pkt, time);
        }
    };

  public: //Made public so coherence can get at it.
    CachePort *cpuSidePort;
    CachePort *memSidePort;

  protected:

    /** Miss status registers */
    MSHRQueue mshrQueue;

    /** Write/writeback buffer */
    MSHRQueue writeBuffer;

    /** Block size of this cache */
    const int blkSize;

    /** The number of targets for each MSHR. */
    const int numTarget;

    /** Increasing order number assigned to each incoming request. */
    uint64_t order;

    /**
     * Bit vector of the blocking reasons for the access path.
     * @sa #BlockedCause
     */
    uint8_t blocked;

    /** Stores time the cache blocked for statistics. */
    Tick blockedCycle;

    /** Pointer to the MSHR that has no targets. */
    MSHR *noTargetMSHR;

    /** The number of misses to trigger an exit event. */
    Counter missCount;

    /** The drain event. */
    Event *drainEvent;

  public:
    // Statistics
    /**
     * @addtogroup CacheStatistics
     * @{
     */

    /** Number of hits per thread for each type of command. @sa Packet::Command */
    Stats::Vector<> hits[MemCmd::NUM_MEM_CMDS];
    /** Number of hits for demand accesses. */
    Stats::Formula demandHits;
    /** Number of hit for all accesses. */
    Stats::Formula overallHits;

    /** Number of misses per thread for each type of command. @sa Packet::Command */
    Stats::Vector<> misses[MemCmd::NUM_MEM_CMDS];
    /** Number of misses for demand accesses. */
    Stats::Formula demandMisses;
    /** Number of misses for all accesses. */
    Stats::Formula overallMisses;

    /**
     * Total number of cycles per thread/command spent waiting for a miss.
     * Used to calculate the average miss latency.
     */
    Stats::Vector<> missLatency[MemCmd::NUM_MEM_CMDS];
    /** Total number of cycles spent waiting for demand misses. */
    Stats::Formula demandMissLatency;
    /** Total number of cycles spent waiting for all misses. */
    Stats::Formula overallMissLatency;

    /** The number of accesses per command and thread. */
    Stats::Formula accesses[MemCmd::NUM_MEM_CMDS];
    /** The number of demand accesses. */
    Stats::Formula demandAccesses;
    /** The number of overall accesses. */
    Stats::Formula overallAccesses;

    /** The miss rate per command and thread. */
    Stats::Formula missRate[MemCmd::NUM_MEM_CMDS];
    /** The miss rate of all demand accesses. */
    Stats::Formula demandMissRate;
    /** The miss rate for all accesses. */
    Stats::Formula overallMissRate;

    /** The average miss latency per command and thread. */
    Stats::Formula avgMissLatency[MemCmd::NUM_MEM_CMDS];
    /** The average miss latency for demand misses. */
    Stats::Formula demandAvgMissLatency;
    /** The average miss latency for all misses. */
    Stats::Formula overallAvgMissLatency;

    /** The total number of cycles blocked for each blocked cause. */
    Stats::Vector<> blocked_cycles;
    /** The number of times this cache blocked for each blocked cause. */
    Stats::Vector<> blocked_causes;

    /** The average number of cycles blocked for each blocked cause. */
    Stats::Formula avg_blocked;

    /** The number of fast writes (WH64) performed. */
    Stats::Scalar<> fastWrites;

    /** The number of cache copies performed. */
    Stats::Scalar<> cacheCopies;

    /** Number of blocks written back per thread. */
    Stats::Vector<> writebacks;

    /** Number of misses that hit in the MSHRs per command and thread. */
    Stats::Vector<> mshr_hits[MemCmd::NUM_MEM_CMDS];
    /** Demand misses that hit in the MSHRs. */
    Stats::Formula demandMshrHits;
    /** Total number of misses that hit in the MSHRs. */
    Stats::Formula overallMshrHits;

    /** Number of misses that miss in the MSHRs, per command and thread. */
    Stats::Vector<> mshr_misses[MemCmd::NUM_MEM_CMDS];
    /** Demand misses that miss in the MSHRs. */
    Stats::Formula demandMshrMisses;
    /** Total number of misses that miss in the MSHRs. */
    Stats::Formula overallMshrMisses;

    /** Number of misses that miss in the MSHRs, per command and thread. */
    Stats::Vector<> mshr_uncacheable[MemCmd::NUM_MEM_CMDS];
    /** Total number of misses that miss in the MSHRs. */
    Stats::Formula overallMshrUncacheable;

    /** Total cycle latency of each MSHR miss, per command and thread. */
    Stats::Vector<> mshr_miss_latency[MemCmd::NUM_MEM_CMDS];
    /** Total cycle latency of demand MSHR misses. */
    Stats::Formula demandMshrMissLatency;
    /** Total cycle latency of overall MSHR misses. */
    Stats::Formula overallMshrMissLatency;

    /** Total cycle latency of each MSHR miss, per command and thread. */
    Stats::Vector<> mshr_uncacheable_lat[MemCmd::NUM_MEM_CMDS];
    /** Total cycle latency of overall MSHR misses. */
    Stats::Formula overallMshrUncacheableLatency;

    /** The total number of MSHR accesses per command and thread. */
    Stats::Formula mshrAccesses[MemCmd::NUM_MEM_CMDS];
    /** The total number of demand MSHR accesses. */
    Stats::Formula demandMshrAccesses;
    /** The total number of MSHR accesses. */
    Stats::Formula overallMshrAccesses;

    /** The miss rate in the MSHRs pre command and thread. */
    Stats::Formula mshrMissRate[MemCmd::NUM_MEM_CMDS];
    /** The demand miss rate in the MSHRs. */
    Stats::Formula demandMshrMissRate;
    /** The overall miss rate in the MSHRs. */
    Stats::Formula overallMshrMissRate;

    /** The average latency of an MSHR miss, per command and thread. */
    Stats::Formula avgMshrMissLatency[MemCmd::NUM_MEM_CMDS];
    /** The average latency of a demand MSHR miss. */
    Stats::Formula demandAvgMshrMissLatency;
    /** The average overall latency of an MSHR miss. */
    Stats::Formula overallAvgMshrMissLatency;

    /** The average latency of an MSHR miss, per command and thread. */
    Stats::Formula avgMshrUncacheableLatency[MemCmd::NUM_MEM_CMDS];
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

    /**
     * Register stats for this object.
     */
    virtual void regStats();

  public:

    class Params
    {
      public:
        /** The hit latency for this cache. */
        int hitLatency;
        /** The block size of this cache. */
        int blkSize;
        int numMSHRs;
        int numTargets;
        int numWriteBuffers;
        /**
         * The maximum number of misses this cache should handle before
         * ending the simulation.
         */
        Counter maxMisses;

        /**
         * Construct an instance of this parameter class.
         */
        Params(int _hitLatency, int _blkSize,
               int _numMSHRs, int _numTargets, int _numWriteBuffers,
               Counter _maxMisses)
            : hitLatency(_hitLatency), blkSize(_blkSize),
              numMSHRs(_numMSHRs), numTargets(_numTargets),
              numWriteBuffers(_numWriteBuffers), maxMisses(_maxMisses)
        {
        }
    };

    /**
     * Create and initialize a basic cache object.
     * @param name The name of this cache.
     * @param hier_params Pointer to the HierParams object for this hierarchy
     * of this cache.
     * @param params The parameter object for this BaseCache.
     */
    BaseCache(const std::string &name, Params &params);

    ~BaseCache()
    {
    }

    virtual void init();

    /**
     * Query block size of a cache.
     * @return  The block size
     */
    int getBlockSize() const
    {
        return blkSize;
    }


    Addr blockAlign(Addr addr) const { return (addr & ~(blkSize - 1)); }


    /**
     * Returns true if the cache is blocked for accesses.
     */
    bool isBlocked()
    {
        return blocked != 0;
    }

    /**
     * Marks the access path of the cache as blocked for the given cause. This
     * also sets the blocked flag in the slave interface.
     * @param cause The reason for the cache blocking.
     */
    void setBlocked(BlockedCause cause)
    {
        uint8_t flag = 1 << cause;
        if (blocked == 0) {
            blocked_causes[cause]++;
            blockedCycle = curTick;
        }
        int old_state = blocked;
        if (!(blocked & flag)) {
            //Wasn't already blocked for this cause
            blocked |= flag;
            DPRINTF(Cache,"Blocking for cause %s\n", cause);
            if (!old_state)
                cpuSidePort->setBlocked();
        }
    }

    /**
     * Marks the cache as unblocked for the given cause. This also clears the
     * blocked flags in the appropriate interfaces.
     * @param cause The newly unblocked cause.
     * @warning Calling this function can cause a blocked request on the bus to
     * access the cache. The cache must be in a state to handle that request.
     */
    void clearBlocked(BlockedCause cause)
    {
        uint8_t flag = 1 << cause;
        DPRINTF(Cache,"Unblocking for cause %s, causes left=%i\n",
                cause, blocked);
        if (blocked & flag)
        {
            blocked &= ~flag;
            if (!isBlocked()) {
                blocked_cycles[cause] += curTick - blockedCycle;
                DPRINTF(Cache,"Unblocking from all causes\n");
                cpuSidePort->clearBlocked();
            }
        }
    }

    /**
     * True if the memory-side bus should be requested.
     * @return True if there are outstanding requests for the master bus.
     */
    bool isMemSideBusRequested()
    {
        return memSidePort->isBusRequested();
    }

    /**
     * Request the master bus for the given cause and time.
     * @param cause The reason for the request.
     * @param time The time to make the request.
     */
    void requestMemSideBus(RequestCause cause, Tick time)
    {
        memSidePort->requestBus(cause, time);
    }

    /**
     * Clear the master bus request for the given cause.
     * @param cause The request reason to clear.
     */
    void deassertMemSideBusRequest(RequestCause cause)
    {
        memSidePort->deassertBusRequest(cause);
        // checkDrain();
    }

    virtual unsigned int drain(Event *de);

    virtual bool inCache(Addr addr) = 0;

    virtual bool inMissQueue(Addr addr) = 0;

    void incMissCount(PacketPtr pkt)
    {
        misses[pkt->cmdToIndex()][0/*pkt->req->getThreadNum()*/]++;

        if (missCount) {
            --missCount;
            if (missCount == 0)
                exitSimLoop("A cache reached the maximum miss count");
        }
    }

};

#endif //__BASE_CACHE_HH__
