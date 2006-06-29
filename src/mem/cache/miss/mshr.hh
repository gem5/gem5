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

#include "mem/packet.hh"
#include <list>
#include <deque>

class MSHR;

/**
 * Miss Status and handling Register. This class keeps all the information
 * needed to handle a cache miss including a list of target requests.
 */
class MSHR {
  public:
    /** Defines the Data structure of the MSHR targetlist. */
    typedef std::list<Packet *> TargetList;
    /** Target list iterator. */
    typedef std::list<Packet *>::iterator TargetListIterator;
    /** A list of MSHRs. */
    typedef std::list<MSHR *> List;
    /** MSHR list iterator. */
    typedef List::iterator Iterator;
    /** MSHR list const_iterator. */
    typedef List::const_iterator ConstIterator;

    /** Address of the miss. */
    Addr addr;
    /** Adress space id of the miss. */
    short asid;
    /** True if the request has been sent to the bus. */
    bool inService;
    /** Thread number of the miss. */
    int threadNum;
    /** The request that is forwarded to the next level of the hierarchy. */
    Packet * pkt;
    /** The number of currently allocated targets. */
    short ntargets;
    /** The original requesting command. */
    Packet::Command originalCmd;
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
    /**
     * Allocate a miss to this MSHR.
     * @param cmd The requesting command.
     * @param addr The address of the miss.
     * @param asid The address space id of the miss.
     * @param size The number of bytes to request.
     * @param req  The original miss.
     */
    void allocate(Packet::Command cmd, Addr addr, int asid, int size,
                  Packet * &pkt);

    /**
     * Allocate this MSHR as a buffer for the given request.
     * @param target The memory request to buffer.
     */
    void allocateAsBuffer(Packet * &target);

    /**
     * Mark this MSHR as free.
     */
    void deallocate();

    /**
     * Add a request to the list of targets.
     * @param target The target.
     */
    void allocateTarget(Packet * &target);

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
        return(ntargets);
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
    Packet * getTarget()
    {
        return targets.front();
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
