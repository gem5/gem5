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
 * Generic queue entry
 */

#ifndef __MEM_CACHE_QUEUE_ENTRY_HH__
#define __MEM_CACHE_QUEUE_ENTRY_HH__

#include "base/named.hh"
#include "base/types.hh"
#include "mem/packet.hh"

namespace gem5
{

class BaseCache;

/**
 * A queue entry base class, to be used by both the MSHRs and
 * write-queue entries.
 */
class QueueEntry : public Packet::SenderState, public Named
{

    /**
     * Consider the Queue a friend to avoid making everything public
     */
    template <class Entry>
    friend class Queue;

  protected:

    /** Tick when ready to issue */
    Tick readyTime;

    /** True if the entry is uncacheable */
    bool _isUncacheable;

  public:
    /**
     * A queue entry is holding packets that will be serviced as soon as
     * resources are available. Since multiple references to the same
     * address can arrive while a packet is not serviced, each packet is
     * stored in a target containing its availability, order and other info,
     * and the queue entry stores these similar targets in a list.
     */
    class Target
    {
      public:
        const Tick recvTime;  //!< Time when request was received (for stats)
        const Tick readyTime; //!< Time when request is ready to be serviced
        const Counter order;  //!< Global order (for memory consistency mgmt)
        PacketPtr pkt;  //!< Pending request packet.

        /**
         * Default constructor. Assigns the current tick as the arrival time
         * of the packet.
         *
         * @param _pkt The pending request packet.
         * @param ready_time The tick at which the packet will be serviceable.
         * @param _order Global order.
         */
        Target(PacketPtr _pkt, Tick ready_time, Counter _order)
            : recvTime(curTick()), readyTime(ready_time), order(_order),
              pkt(_pkt)
        {}
    };

    /** True if the entry has been sent downstream. */
    bool inService;

    /** Order number assigned to disambiguate writes and misses. */
    Counter order;

    /** Block aligned address. */
    Addr blkAddr;

    /** Block size of the cache. */
    unsigned blkSize;

    /** True if the entry targets the secure memory space. */
    bool isSecure;

    QueueEntry(const std::string &name)
        : Named(name),
          readyTime(0), _isUncacheable(false),
          inService(false), order(0), blkAddr(0), blkSize(0), isSecure(false)
    {}

    bool isUncacheable() const { return _isUncacheable; }

    /**
     * Check if entry corresponds to the one being looked for.
     *
     * @param addr Address to match against.
     * @param is_secure Whether the target should be in secure space or not.
     * @return True if entry matches given information.
     */
    virtual bool matchBlockAddr(const Addr addr, const bool is_secure)
                                                            const = 0;

    /**
     * Check if entry contains a packet that corresponds to the one being
     * looked for.
     *
     * @param pkt The packet to search for.
     * @return True if any of its targets' packets matches the given one.
     */
    virtual bool matchBlockAddr(const PacketPtr pkt) const = 0;

    /**
     * Check if given entry's packets conflict with this' entries packets.
     *
     * @param entry Other entry to compare against.
     * @return True if entry matches given information.
     */
    virtual bool conflictAddr(const QueueEntry* entry) const = 0;

    /**
     * Send this queue entry as a downstream packet, with the exact
     * behaviour depending on the specific entry type.
     */
    virtual bool sendPacket(BaseCache &cache) = 0;

    /**
     * Returns a pointer to the first target.
     *
     * @return A pointer to the first target.
     */
    virtual Target* getTarget() = 0;
};

} // namespace gem5

#endif // __MEM_CACHE_QUEUE_ENTRY_HH__
