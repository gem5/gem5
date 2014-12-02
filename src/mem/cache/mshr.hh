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

#ifndef __MSHR_HH__
#define __MSHR_HH__

#include <list>

#include "base/printable.hh"
#include "mem/packet.hh"

class CacheBlk;
class MSHRQueue;

/**
 * Miss Status and handling Register. This class keeps all the information
 * needed to handle a cache miss including a list of target requests.
 * @sa  \ref gem5MemorySystem "gem5 Memory System"
 */
class MSHR : public Packet::SenderState, public Printable
{

    /**
     * Consider the MSHRQueue a friend to avoid making everything public
     */
    friend class MSHRQueue;

  private:

    /** Cycle when ready to issue */
    Tick readyTime;

    /** True if the request is uncacheable */
    bool _isUncacheable;

    /** Flag set by downstream caches */
    bool downstreamPending;

    /** Will we have a dirty copy after this request? */
    bool pendingDirty;

    /** Did we snoop an invalidate while waiting for data? */
    bool postInvalidate;

    /** Did we snoop a read while waiting for data? */
    bool postDowngrade;

  public:

    class Target {
      public:

        enum Source {
            FromCPU,
            FromSnoop,
            FromPrefetcher
        };

        Tick recvTime;  //!< Time when request was received (for stats)
        Tick readyTime; //!< Time when request is ready to be serviced
        Counter order;  //!< Global order (for memory consistency mgmt)
        PacketPtr pkt;  //!< Pending request packet.
        Source source;  //!< Did request come from cpu, memory, or prefetcher?
        bool markedPending; //!< Did we mark upstream MSHR
                            //!<  as downstreamPending?

        Target(PacketPtr _pkt, Tick _readyTime, Counter _order,
               Source _source, bool _markedPending)
            : recvTime(curTick()), readyTime(_readyTime), order(_order),
              pkt(_pkt), source(_source), markedPending(_markedPending)
        {}
    };

    class TargetList : public std::list<Target> {
        /** Target list iterator. */
        typedef std::list<Target>::iterator Iterator;
        typedef std::list<Target>::const_iterator ConstIterator;

      public:
        bool needsExclusive;
        bool hasUpgrade;

        TargetList();
        void resetFlags() { needsExclusive = hasUpgrade = false; }
        bool isReset()    { return !needsExclusive && !hasUpgrade; }
        void add(PacketPtr pkt, Tick readyTime, Counter order,
                 Target::Source source, bool markPending);
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
    /** MSHR list const_iterator. */
    typedef List::const_iterator ConstIterator;

    /** Pointer to queue containing this MSHR. */
    MSHRQueue *queue;

    /** Order number assigned by the miss queue. */
    Counter order;

    /** Address of the request. */
    Addr addr;

    /** Size of the request. */
    int size;

    /** True if the request targets the secure memory space. */
    bool isSecure;

    /** True if the request has been sent to the bus. */
    bool inService;

    /** True if the request is just a simple forward from an upper level */
    bool isForward;

    /** The pending* and post* flags are only valid if inService is
     *  true.  Using the accessor functions lets us detect if these
     *  flags are accessed improperly.
     */

    /** True if we need to get an exclusive copy of the block. */
    bool needsExclusive() const { return targets.needsExclusive; }

    bool isPendingDirty() const {
        assert(inService); return pendingDirty;
    }

    bool hasPostInvalidate() const {
        assert(inService); return postInvalidate;
    }

    bool hasPostDowngrade() const {
        assert(inService); return postDowngrade;
    }

    /** Thread number of the miss. */
    ThreadID threadNum;

  private:

    /** Data buffer (if needed).  Currently used only for pending
     * upgrade handling. */
    uint8_t *data;

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

    bool isUncacheable() const { return _isUncacheable; }

    /**
     * Allocate a miss to this MSHR.
     * @param cmd The requesting command.
     * @param addr The address of the miss.
     * @param asid The address space id of the miss.
     * @param size The number of bytes to request.
     * @param pkt  The original miss.
     */
    void allocate(Addr addr, int size, PacketPtr pkt,
                  Tick when, Counter _order);

    bool markInService(PacketPtr pkt);

    void clearDownstreamPending();

    /**
     * Mark this MSHR as free.
     */
    void deallocate();

    /**
     * Add a request to the list of targets.
     * @param target The target.
     */
    void allocateTarget(PacketPtr target, Tick when, Counter order);
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

    bool isForwardNoResponse() const
    {
        if (getNumTargets() != 1)
            return false;
        const Target *tgt = &targets.front();
        return tgt->source == Target::FromCPU && !tgt->pkt->needsResponse();
    }

    bool promoteDeferredTargets();

    void handleFill(Packet *pkt, CacheBlk *blk);

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

#endif //__MSHR_HH__
