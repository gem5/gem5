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
        Tick time;      //!< Time when request was received (for stats)
        PacketPtr pkt;  //!< Pending request packet.
        bool cpuSide;   //!< Did request come from cpu side or mem side?

        bool isCpuSide() { return cpuSide; }

        Target(PacketPtr _pkt, bool _cpuSide, Tick _time = curTick)
            : time(_time), pkt(_pkt), cpuSide(_cpuSide)
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

    /** Address of the request. */
    Addr addr;

    /** Size of the request. */
    int size;

    /** Data associated with the request (if a write). */
    uint8_t *writeData;

    /** True if the request has been sent to the bus. */
    bool inService;

    /** True if we will be putting the returned block in the cache */
    bool isCacheFill;
    /** True if we need to get an exclusive copy of the block. */
    bool needsExclusive;
    /** True if the request is uncacheable */
    bool _isUncacheable;

    /** True if the request that has been sent to the bus is for en
     * exclusive copy. */
    bool inServiceForExclusive;
    /** Thread number of the miss. */
    short threadNum;
    /** The number of currently allocated targets. */
    short ntargets;
    /** Order number of assigned by the miss queue. */
    uint64_t order;

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
    void allocate(Addr addr, int size, PacketPtr pkt);

    /**
     * Allocate this MSHR as a buffer for the given request.
     * @param target The memory request to buffer.
     */
    void allocateAsBuffer(PacketPtr target);

    /**
     * Mark this MSHR as free.
     */
    void deallocate();

    /**
     * Add a request to the list of targets.
     * @param target The target.
     */
    void allocateTarget(PacketPtr target, bool cpuSide);

    /** A simple constructor. */
    MSHR();
    /** A simple destructor. */
    ~MSHR();

    /**
     * Returns the current number of allocated targets.
     * @return The current number of allocated targets.
     */
    int getNumTargets()
    {
        return ntargets;
    }

    /**
     * Returns a pointer to the target list.
     * @return a pointer to the target list.
     */
    TargetList* getTargetList()
    {
        return &targets;
    }

    /**
     * Returns a reference to the first target.
     * @return A pointer to the first target.
     */
    Target *getTarget()
    {
        return &targets.front();
    }

    /**
     * Pop first target.
     */
    void popTarget()
    {
        --ntargets;
        targets.pop_front();
    }

    /**
     * Returns true if there are targets left.
     * @return true if there are targets
     */
    bool hasTargets()
    {
        return !targets.empty();
    }

    /**
     * Prints the contents of this MSHR to stderr.
     */
    void dump();
};

#endif //__MSHR_HH__
