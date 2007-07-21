/*
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

#include "mem/packet.hh"

class CacheBlk;
class MSHRQueue;

/**
 * Miss Status and handling Register. This class keeps all the information
 * needed to handle a cache miss including a list of target requests.
 */
class MSHR : public Packet::SenderState
{

  public:

    class Target {
      public:
        Tick recvTime;  //!< Time when request was received (for stats)
        Tick readyTime; //!< Time when request is ready to be serviced
        Counter order;  //!< Global order (for memory consistency mgmt)
        PacketPtr pkt;  //!< Pending request packet.
        bool cpuSide;   //!< Did request come from cpu side or mem side?

        bool isCpuSide() { return cpuSide; }

        Target(PacketPtr _pkt, Tick _readyTime, Counter _order, bool _cpuSide)
            : recvTime(curTick), readyTime(_readyTime), order(_order),
              pkt(_pkt), cpuSide(_cpuSide)
        {}
    };

    /** Defines the Data structure of the MSHR targetlist. */
    typedef std::list<Target> TargetList;
    /** Target list iterator. */
    typedef std::list<Target>::iterator TargetListIterator;
    /** A list of MSHRs. */
    typedef std::list<MSHR *> List;
    /** MSHR list iterator. */
    typedef List::iterator Iterator;
    /** MSHR list const_iterator. */
    typedef List::const_iterator ConstIterator;

    /** Pointer to queue containing this MSHR. */
    MSHRQueue *queue;

    /** Cycle when ready to issue */
    Tick readyTime;

    /** Order number assigned by the miss queue. */
    Counter order;

    /** Address of the request. */
    Addr addr;

    /** Size of the request. */
    int size;

    /** True if the request has been sent to the bus. */
    bool inService;

    /** True if we will be putting the returned block in the cache */
    bool isCacheFill;
    /** True if we need to get an exclusive copy of the block. */
    bool needsExclusive;

    /** True if the request is uncacheable */
    bool _isUncacheable;

    bool deferredNeedsExclusive;
    bool pendingInvalidate;
    bool pendingShared;

    /** Thread number of the miss. */
    short threadNum;
    /** The number of currently allocated targets. */
    short ntargets;


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

private:
    /** List of all requests that match the address */
    TargetList targets;

    TargetList deferredTargets;

public:

    bool isUncacheable() { return _isUncacheable; }

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
    /** A simple destructor. */
    ~MSHR();

    /**
     * Returns the current number of allocated targets.
     * @return The current number of allocated targets.
     */
    int getNumTargets() { return ntargets; }

    /**
     * Returns a pointer to the target list.
     * @return a pointer to the target list.
     */
    TargetList* getTargetList() { return &targets; }

    /**
     * Returns true if there are targets left.
     * @return true if there are targets
     */
    bool hasTargets() { return !targets.empty(); }

    /**
     * Returns a reference to the first target.
     * @return A pointer to the first target.
     */
    Target *getTarget() { assert(hasTargets());  return &targets.front(); }

    /**
     * Pop first target.
     */
    void popTarget()
    {
        --ntargets;
        targets.pop_front();
    }

    bool isSimpleForward()
    {
        if (getNumTargets() != 1)
            return false;
        Target *tgt = getTarget();
        return tgt->isCpuSide() && !tgt->pkt->needsResponse();
    }

    bool promoteDeferredTargets();

    void handleFill(Packet *pkt, CacheBlk *blk);

    /**
     * Prints the contents of this MSHR to stderr.
     */
    void dump();
};

#endif //__MSHR_HH__
