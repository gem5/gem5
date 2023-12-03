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
 */

/**
 * @file
 * Write queue entry
 */

#ifndef __MEM_CACHE_WRITE_QUEUE_ENTRY_HH__
#define __MEM_CACHE_WRITE_QUEUE_ENTRY_HH__

#include <cassert>
#include <iosfwd>
#include <list>
#include <string>

#include "base/printable.hh"
#include "base/types.hh"
#include "mem/cache/queue_entry.hh"
#include "mem/packet.hh"

namespace gem5
{

class BaseCache;

/**
 * Write queue entry
 */
class WriteQueueEntry : public QueueEntry, public Printable
{
    /**
     * Consider the queues friends to avoid making everything public.
     */
    template <typename Entry>
    friend class Queue;
    friend class WriteQueue;

  public:
    class TargetList : public std::list<Target>
    {
      public:
        TargetList() {}

        void add(PacketPtr pkt, Tick readyTime, Counter order);
        bool trySatisfyFunctional(PacketPtr pkt);
        void print(std::ostream &os, int verbosity,
                   const std::string &prefix) const;
    };

    /** A list of write queue entriess. */
    typedef std::list<WriteQueueEntry *> List;
    /** WriteQueueEntry list iterator. */
    typedef List::iterator Iterator;

    bool sendPacket(BaseCache &cache) override;

  private:
    /**
     * Pointer to this entry on the ready list.
     * @sa MissQueue, WriteQueue::readyList
     */
    Iterator readyIter;

    /**
     * Pointer to this entry on the allocated list.
     * @sa MissQueue, WriteQueue::allocatedList
     */
    Iterator allocIter;

    /** List of all requests that match the address */
    TargetList targets;

  public:
    /** A simple constructor. */
    WriteQueueEntry(const std::string &name) : QueueEntry(name) {}

    /**
     * Allocate a miss to this entry.
     * @param blk_addr The address of the block.
     * @param blk_size The number of bytes to request.
     * @param pkt The original write.
     * @param when_ready When should the write be sent out.
     * @param _order The logical order of this write.
     */
    void allocate(Addr blk_addr, unsigned blk_size, PacketPtr pkt,
                  Tick when_ready, Counter _order);

    /**
     * Mark this entry as free.
     */
    void deallocate();

    /**
     * Returns the current number of allocated targets.
     * @return The current number of allocated targets.
     */
    int
    getNumTargets() const
    {
        return targets.size();
    }

    /**
     * Returns true if there are targets left.
     * @return true if there are targets
     */
    bool
    hasTargets() const
    {
        return !targets.empty();
    }

    /**
     * Returns a reference to the first target.
     * @return A pointer to the first target.
     */
    Target *
    getTarget() override
    {
        assert(hasTargets());
        return &targets.front();
    }

    /**
     * Pop first target.
     */
    void
    popTarget()
    {
        targets.pop_front();
    }

    bool trySatisfyFunctional(PacketPtr pkt);

    /**
     * Prints the contents of this MSHR for debugging.
     */
    void print(std::ostream &os, int verbosity = 0,
               const std::string &prefix = "") const override;
    /**
     * A no-args wrapper of print(std::ostream...)  meant to be
     * invoked from DPRINTFs avoiding string overheads in fast mode
     *
     * @return string with mshr fields
     */
    std::string print() const;

    bool matchBlockAddr(const Addr addr, const bool is_secure) const override;
    bool matchBlockAddr(const PacketPtr pkt) const override;
    bool conflictAddr(const QueueEntry *entry) const override;
};

} // namespace gem5

#endif // __MEM_CACHE_WRITE_QUEUE_ENTRY_HH__
