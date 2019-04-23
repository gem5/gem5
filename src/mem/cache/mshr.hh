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
 *          Nikos Nikoleris
 */

/**
 * @file
 * Miss Status and Handling Register (MSHR) declaration.
 */

#ifndef __MEM_CACHE_MSHR_HH__
#define __MEM_CACHE_MSHR_HH__

#include <cassert>
#include <iosfwd>
#include <list>
#include <string>
#include <vector>

#include "base/printable.hh"
#include "base/types.hh"
#include "mem/cache/queue_entry.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "sim/core.hh"

class BaseCache;

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

    /** Track if we sent this as a whole line write or not */
    bool wasWholeLineWrite;

    /** True if the entry is just a simple forward from an upper level */
    bool isForward;

    class Target : public QueueEntry::Target {
      public:

        enum Source {
            FromCPU,
            FromSnoop,
            FromPrefetcher
        };

        const Source source;  //!< Request from cpu, memory, or prefetcher?

        /**
         * We use this flag to track whether we have cleared the
         * downstreamPending flag for the MSHR of the cache above
         * where this packet originates from and guard noninitial
         * attempts to clear it.
         *
         * The flag markedPending needs to be updated when the
         * TargetList is in service which can be:
         * 1) during the Target instantiation if the MSHR is in
         * service and the target is not deferred,
         * 2) when the MSHR becomes in service if the target is not
         * deferred,
         * 3) or when the TargetList is promoted (deferredTargets ->
         * targets).
         */
        bool markedPending;

        const bool allocOnFill;   //!< Should the response servicing this
                                  //!< target list allocate in the cache?

        Target(PacketPtr _pkt, Tick _readyTime, Counter _order,
               Source _source, bool _markedPending, bool alloc_on_fill)
            : QueueEntry::Target(_pkt, _readyTime, _order), source(_source),
              markedPending(_markedPending), allocOnFill(alloc_on_fill)
        {}
    };

    class TargetList : public std::list<Target> {

      public:
        bool needsWritable;
        bool hasUpgrade;
        /** Set when the response should allocate on fill */
        bool allocOnFill;
        /**
         * Determine whether there was at least one non-snooping
         * target coming from another cache.
         */
        bool hasFromCache;

        TargetList();

        /**
         * Use the provided packet and the source to update the
         * flags of this TargetList.
         *
         * @param pkt Packet considered for the flag update
         * @param source Indicates the source of the packet
         * @param alloc_on_fill Whether the pkt would allocate on a fill
         */
        void updateFlags(PacketPtr pkt, Target::Source source,
                         bool alloc_on_fill);

        /**
         * Reset state
         *
         * @param blk_addr Address of the cache block
         * @param blk_size Size of the cache block
         */
        void init(Addr blk_addr, Addr blk_size) {
            blkAddr = blk_addr;
            blkSize = blk_size;
            writesBitmap.resize(blk_size);

            resetFlags();
        }

        void resetFlags() {
            onlyWrites = true;
            std::fill(writesBitmap.begin(), writesBitmap.end(), false);

            needsWritable = false;
            hasUpgrade = false;
            allocOnFill = false;
            hasFromCache = false;
        }

        /**
         * Goes through the list of targets and uses them to populate
         * the flags of this TargetList. When the function returns the
         * flags are consistent with the properties of packets in the
         * list.
         */
        void populateFlags();

        /**
         * Add the specified packet in the TargetList. This function
         * stores information related to the added packet and updates
         * accordingly the flags.
         *
         * @param pkt Packet considered for adding
         */
        void updateWriteFlags(PacketPtr pkt) {
             const Request::FlagsType noMergeFlags =
                 Request::UNCACHEABLE |
                 Request::STRICT_ORDER | Request::MMAPPED_IPR |
                 Request::PRIVILEGED | Request::LLSC |
                 Request::MEM_SWAP | Request::MEM_SWAP_COND |
                 Request::SECURE;

             // if we have already seen writes for the full block stop
             // here, this might be a full line write followed by
             // other compatible requests (e.g., reads)
             if (!isWholeLineWrite()) {
                 bool can_merge_write = pkt->isWrite() &&
                     ((pkt->req->getFlags() & noMergeFlags) == 0);
                 onlyWrites &= can_merge_write;
                 if (onlyWrites) {
                     auto offset = pkt->getOffset(blkSize);
                     auto begin = writesBitmap.begin() + offset;
                     std::fill(begin, begin + pkt->getSize(), true);
                 }
             }
         }

        /**
         * Tests if the flags of this TargetList have their default
         * values.
         *
         * @return True if the TargetList are reset, false otherwise.
         */
        bool isReset() const {
            return !needsWritable && !hasUpgrade && !allocOnFill &&
                !hasFromCache && onlyWrites;
        }

        /**
         * Add the specified packet in the TargetList. This function
         * stores information related to the added packet and updates
         * accordingly the flags.
         *
         * @param pkt Packet considered for adding
         * @param readTime Tick at which the packet is processed by this cache
         * @param order A counter giving a unique id to each target
         * @param source Indicates the source agent of the packet
         * @param markPending Set for deferred targets or pending MSHRs
         * @param alloc_on_fill Whether it should allocate on a fill
         */
        void add(PacketPtr pkt, Tick readyTime, Counter order,
                 Target::Source source, bool markPending, bool alloc_on_fill);

        /**
         * Convert upgrades to the equivalent request if the cache line they
         * refer to would have been invalid (Upgrade -> ReadEx, SC* -> Fail).
         * Used to rejig ordering between targets waiting on an MSHR. */
        void replaceUpgrades();

        void clearDownstreamPending();
        void clearDownstreamPending(iterator begin, iterator end);
        bool trySatisfyFunctional(PacketPtr pkt);
        void print(std::ostream &os, int verbosity,
                   const std::string &prefix) const;

        /**
         * Check if this list contains only compatible writes, and if they
         * span the entire cache line. This is used as part of the
         * miss-packet creation. Note that new requests may arrive after a
         * miss-packet has been created, and for the fill we therefore use
         * the wasWholeLineWrite field.
         */
        bool isWholeLineWrite() const
        {
            return onlyWrites &&
                std::all_of(writesBitmap.begin(),
                            writesBitmap.end(), [](bool i) { return i; });
        }

      private:
        /** Address of the cache block for this list of targets. */
        Addr blkAddr;

        /** Size of the cache block. */
        Addr blkSize;

        /** Are we only dealing with writes. */
        bool onlyWrites;

        // NOTE: std::vector<bool> might not meet satisfy the
        // ForwardIterator requirement and therefore cannot be used
        // for writesBitmap.
        /**
         * Track which bytes are written by requests in this target
         * list.
         */
        std::vector<char> writesBitmap;
    };

    /** A list of MSHRs. */
    typedef std::list<MSHR *> List;
    /** MSHR list iterator. */
    typedef List::iterator Iterator;

    /** The pending* and post* flags are only valid if inService is
     *  true.  Using the accessor functions lets us detect if these
     *  flags are accessed improperly.
     */

    /** True if we need to get a writable copy of the block. */
    bool needsWritable() const { return targets.needsWritable; }

    bool isCleaning() const {
        PacketPtr pkt = targets.front().pkt;
        return pkt->isClean();
    }

    bool isPendingModified() const {
        assert(inService); return pendingModified;
    }

    bool hasPostInvalidate() const {
        assert(inService); return postInvalidate;
    }

    bool hasPostDowngrade() const {
        assert(inService); return postDowngrade;
    }

    bool sendPacket(BaseCache &cache) override;

    bool allocOnFill() const {
        return targets.allocOnFill;
    }

    /**
     * Determine if there are non-deferred requests from other caches
     *
     * @return true if any of the targets is from another cache
     */
    bool hasFromCache() const {
        return targets.hasFromCache;
    }

  private:
    /**
     * Promotes deferred targets that satisfy a predicate
     *
     * Deferred targets are promoted to the target list if they
     * satisfy a given condition. The operation stops at the first
     * deferred target that doesn't satisfy the condition.
     *
     * @param pred A condition on a Target
     */
    void promoteIf(const std::function<bool (Target &)>& pred);

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
     * Check if this MSHR contains only compatible writes, and if they
     * span the entire cache line. This is used as part of the
     * miss-packet creation. Note that new requests may arrive after a
     * miss-packet has been created, and for the fill we therefore use
     * the wasWholeLineWrite field.
     */
    bool isWholeLineWrite() const {
        return targets.isWholeLineWrite();
    }

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
     * Extracts the subset of the targets that can be serviced given a
     * received response. This function returns the targets list
     * unless the response is a ReadRespWithInvalidate. The
     * ReadRespWithInvalidate is only invalidating response that its
     * invalidation was not expected when the request (a
     * ReadSharedReq) was sent out. For ReadRespWithInvalidate we can
     * safely service only the first FromCPU target and all FromSnoop
     * targets (inform all snoopers that we no longer have the block).
     *
     * @param pkt The response from the downstream memory
     */
    TargetList extractServiceableTargets(PacketPtr pkt);

    /**
     * Returns true if there are targets left.
     * @return true if there are targets
     */
    bool hasTargets() const { return !targets.empty(); }

    /**
     * Returns a reference to the first target.
     * @return A pointer to the first target.
     */
    QueueEntry::Target *getTarget() override
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

    /**
     * Promotes deferred targets that do not require writable
     *
     * Move targets from the deferred targets list to the target list
     * starting from the first deferred target until the first target
     * that is a cache maintenance operation or needs a writable copy
     * of the block
     */
    void promoteReadable();

    /**
     * Promotes deferred targets that do not require writable
     *
     * Requests in the deferred target list are moved to the target
     * list up until the first target that is a cache maintenance
     * operation or needs a writable copy of the block
     */
    void promoteWritable();

    bool trySatisfyFunctional(PacketPtr pkt);

    /**
     * Adds a delay relative to the current tick to the current MSHR
     * @param delay_ticks the desired delay in ticks
     */
    void delay(Tick delay_ticks)
    {
        assert(readyTime <= curTick());
        readyTime = curTick() + delay_ticks;
    }

    /**
     * Prints the contents of this MSHR for debugging.
     */
    void print(std::ostream &os,
               int verbosity = 0,
               const std::string &prefix = "") const override;
    /**
     * A no-args wrapper of print(std::ostream...)  meant to be
     * invoked from DPRINTFs avoiding string overheads in fast mode
     *
     * @return string with mshr fields + [deferred]targets
     */
    std::string print() const;

    bool matchBlockAddr(const Addr addr, const bool is_secure) const override;
    bool matchBlockAddr(const PacketPtr pkt) const override;
    bool conflictAddr(const QueueEntry* entry) const override;
};

#endif // __MEM_CACHE_MSHR_HH__
