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
 *          Andreas Hansson
 */

/**
 * @file
 * Generic queue entry
 */

#ifndef __MEM_CACHE_QUEUE_ENTRY_HH__
#define __MEM_CACHE_QUEUE_ENTRY_HH__

#include "mem/packet.hh"

class Cache;

/**
 * A queue entry base class, to be used by both the MSHRs and
 * write-queue entries.
 */
class QueueEntry : public Packet::SenderState
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

    QueueEntry() : readyTime(0), _isUncacheable(false),
                   inService(false), order(0), blkAddr(0), blkSize(0),
                   isSecure(false)
    {}

    bool isUncacheable() const { return _isUncacheable; }

    /**
     * Send this queue entry as a downstream packet, with the exact
     * behaviour depending on the specific entry type.
     */
    virtual bool sendPacket(Cache &cache) = 0;

};

#endif // __MEM_CACHE_QUEUE_ENTRY_HH__
