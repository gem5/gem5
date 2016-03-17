/*
 * Copyright (c) 2012-2013, 2015-2016 ARM Limited
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
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 * Miss Status and Handling Register (MSHR) declaration.
 */

#ifndef __MEM_CACHE_MSHR_HH__
#define __MEM_CACHE_MSHR_HH__

#include <list>

#include "base/printable.hh"
#include "mem/cache/queue_entry.hh"

class Cache;

/**
 * Miss Status and handling Register. This class keeps all the information
 * needed to handle a cache miss including a list of target requests.
 * @sa  \ref gem5MemorySystem "gem5 Memory System"
 */
class MSHR : public QueueEntry, public Printable
{

    /**
     * Consider the queues friends to avoid making everything public.
     */
    template<typename Entry>
    friend class Queue;
    friend class MSHRQueue;

  private:

    /** Flag set by downstream caches */
    bool downstreamPending;

    /**
     * Here we use one flag to track both if:
     *
     * 1. We are going to become owner or not, i.e., we will get the
     * block in an ownership state (Owned or Modified) with BlkDirty
     * set. This determines whether or not we are going to become the
     * responder and ordering point for future requests that we snoop.
     *
     * 2. We know that we are going to get a writable block, i.e. we
     * will get the block in writable state (Exclusive or Modified
     * state) with BlkWritable set. That determines whether additional
     * targets with needsWritable set will be able to be satisfied, or
     * if not should be put on the deferred list to possibly wait for
     * another request that does give us writable access.
     *
     * Condition 2 is actually just a shortcut that saves us from
     * possibly building a deferred target list and calling
     * promoteWritable() every time we get a writable block. Condition
     * 1, tracking ownership, is what is important. However, we never
     * receive ownership without marking the block dirty, and
     * consequently use pendingModified to track both ownership and
     * writability rather than having separate pendingDirty and
     * pendingWritable flags.
     */
    bool pendingModified;

    /** Did we snoop an invalidate while waiting for data? */
    bool postInvalidate;

    /** Did we snoop a read while waiting for data? */
    bool postDowngrade;

  public:

    /** True if the entry is just a simple forward from an upper level */
    bool isForward;

    class Target {
      public:

        enum Source {
            FromCPU,
            FromSnoop,
            FromPrefetcher
        };

        const Tick recvTime;  //!< Time when request was received (for stats)
        const Tick readyTime; //!< Time when request is ready to be serviced
        const Counter order;  //!< Global order (for memory consistency mgmt)
        const PacketPtr pkt;  //!< Pending request packet.
        const Source source;  //!< Request from cpu, memory, or prefetcher?
        const bool markedPending; //!< Did we mark upstream MSHR
                                  //!< as downstreamPending?

        Target(PacketPtr _pkt, Tick _readyTime, Counter _order,
               Source _source, bool _markedPending)
            : recvTime(curTick()), readyTime(_readyTime), order(_order),
              pkt(_pkt), source(_source), markedPending(_markedPending)
        {}
    };

    class TargetList : public std::list<Target> {

      public:
        bool needsWritable;
        bool hasUpgrade;

        TargetList();
        void resetFlags() { needsWritable = hasUpgrade = false; }
        bool isReset() const { return !needsWritable && !hasUpgrade; }
        void add(PacketPtr pkt, Tick readyTime, Counter order,
                 Target::Source source, bool markPending);

        /**
         * Convert upgrades to the equivalent request if the cache line they
         * refer to would have been invalid (Upgrade -> ReadEx, SC* -> Fail).
         * Used to rejig ordering between targets waiting on an MSHR. */
        void replaceUpgrades();

        void clearDownstreamPending();
        bool checkFunctional(PacketPtr pkt);
        void print(std::ostream &os, int verbosity,
                   const std::string &prefix) const;
    };

    /** A list of MSHRs. */
    typedef std::list<MSHR *> List;
    /** MSHR list iterator. */
    typedef List::iterator Iterator;

    /** Keep track of whether we should allocate on fill or not */
    bool allocOnFill;

    /** The pending* and post* flags are only valid if inService is
     *  true.  Using the accessor functions lets us detect if these
     *  flags are accessed improperly.
     */

    /** True if we need to get a writable copy of the block. */
    bool needsWritable() const { return targets.needsWritable; }

    bool isPendingModified() const {
        assert(inService); return pendingModified;
    }

    bool hasPostInvalidate() const {
        assert(inService); return postInvalidate;
    }

    bool hasPostDowngrade() const {
        assert(inService); return postDowngrade;
    }

    bool sendPacket(Cache &cache);

  private:

    /**
     * Pointer to this MSHR on the ready list.
     * @sa MissQueue, MSHRQueue::readyList
     */
    Iterator readyIter;

    /**
     * Pointer to this MSHR on the allocated list.
     * @sa MissQueue, MSHRQueue::allocatedList
     */
    Iterator allocIter;

    /** List of all requests that match the address */
    TargetList targets;

    TargetList deferredTargets;

  public:

    /**
     * Allocate a miss to this MSHR.
     * @param blk_addr The address of the block.
     * @param blk_size The number of bytes to request.
     * @param pkt The original miss.
     * @param when_ready When should the MSHR be ready to act upon.
     * @param _order The logical order of this MSHR
     * @param alloc_on_fill Should the cache allocate a block on fill
     */
    void allocate(Addr blk_addr, unsigned blk_size, PacketPtr pkt,
                  Tick when_ready, Counter _order, bool alloc_on_fill);

    void markInService(bool pending_modified_resp);

    void clearDownstreamPending();

    /**
     * Mark this MSHR as free.
     */
    void deallocate();

    /**
     * Add a request to the list of targets.
     * @param target The target.
     */
    void allocateTarget(PacketPtr target, Tick when, Counter order,
                        bool alloc_on_fill);
    bool handleSnoop(PacketPtr target, Counter order);

    /** A simple constructor. */
    MSHR();

    /**
     * Returns the current number of allocated targets.
     * @return The current number of allocated targets.
     */
    int getNumTargets() const
    { return targets.size() + deferredTargets.size(); }

    /**
     * Returns true if there are targets left.
     * @return true if there are targets
     */
    bool hasTargets() const { return !targets.empty(); }

    /**
     * Returns a reference to the first target.
     * @return A pointer to the first target.
     */
    Target *getTarget()
    {
        assert(hasTargets());
        return &targets.front();
    }

    /**
     * Pop first target.
     */
    void popTarget()
    {
        targets.pop_front();
    }

    bool promoteDeferredTargets();

    void promoteWritable();

    bool checkFunctional(PacketPtr pkt);

    /**
     * Prints the contents of this MSHR for debugging.
     */
    void print(std::ostream &os,
               int verbosity = 0,
               const std::string &prefix = "") const;
    /**
     * A no-args wrapper of print(std::ostream...)  meant to be
     * invoked from DPRINTFs avoiding string overheads in fast mode
     *
     * @return string with mshr fields + [deferred]targets
     */
    std::string print() const;
};

#endif // __MEM_CACHE_MSHR_HH__
