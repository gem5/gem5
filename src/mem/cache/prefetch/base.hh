/*
 * Copyright (c) 2013-2014 ARM Limited
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
 * Copyright (c) 2005 The Regents of The University of Michigan
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
 * Authors: Ron Dreslinski
 *          Mitch Hayenga
 */

/**
 * @file
 * Miss and writeback queue declarations.
 */

#ifndef __MEM_CACHE_PREFETCH_BASE_HH__
#define __MEM_CACHE_PREFETCH_BASE_HH__

#include <cstdint>

#include "base/statistics.hh"
#include "base/types.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "sim/clocked_object.hh"
#include "sim/probe/probe.hh"

class BaseCache;
struct BasePrefetcherParams;

class BasePrefetcher : public ClockedObject
{
    class PrefetchListener : public ProbeListenerArgBase<PacketPtr>
    {
      public:
        PrefetchListener(BasePrefetcher &_parent, ProbeManager *pm,
                         const std::string &name)
            : ProbeListenerArgBase(pm, name),
              parent(_parent) {}
        void notify(const PacketPtr &pkt) override;
      protected:
        BasePrefetcher &parent;
    };

    std::vector<PrefetchListener *> listeners;
  protected:

    /**
     * Class containing the information needed by the prefetch to train and
     * generate new prefetch requests.
     */
    class PrefetchInfo {
        /** The address. */
        Addr address;
        /** The program counter that generated this address. */
        Addr pc;
        /** The requestor ID that generated this address. */
        MasterID masterId;
        /** Validity bit for the PC of this address. */
        bool validPC;
        /** Whether this address targets the secure memory space. */
        bool secure;

      public:
        /**
         * Obtains the address value of this Prefetcher address.
         * @return the addres value.
         */
        Addr getAddr() const
        {
            return address;
        }

        /**
         * Returns true if the address targets the secure memory space.
         * @return true if the address targets the secure memory space.
         */
        bool isSecure() const
        {
            return secure;
        }

        /**
         * Returns the program counter that generated this request.
         * @return the pc value
         */
        Addr getPC() const
        {
            assert(hasPC());
            return pc;
        }

        /**
         * Returns true if the associated program counter is valid
         * @return true if the program counter has a valid value
         */
        bool hasPC() const
        {
            return validPC;
        }

        /**
         * Gets the requestor ID that generated this address
         * @return the requestor ID that generated this address
         */
        MasterID getMasterId() const
        {
            return masterId;
        }

        /**
         * Check for equality
         * @param pfi PrefetchInfo to compare against
         * @return True if this object and the provided one are equal
         */
        bool sameAddr(PrefetchInfo const &pfi) const
        {
            return this->getAddr() == pfi.getAddr() &&
                this->isSecure() == pfi.isSecure();
        }

        /**
         * Constructs a PrefetchInfo using a PacketPtr.
         * @param pkt PacketPtr used to generate the PrefetchInfo
         * @param addr the address value of the new object
         */
        PrefetchInfo(PacketPtr pkt, Addr addr);

        /**
         * Constructs a PrefetchInfo using a new address value and
         * another PrefetchInfo as a reference.
         * @param pfi PrefetchInfo used to generate this new object
         * @param addr the address value of the new object
         */
        PrefetchInfo(PrefetchInfo const &pfi, Addr addr);
    };

    // PARAMETERS

    /** Pointr to the parent cache. */
    BaseCache* cache;

    /** The block size of the parent cache. */
    unsigned blkSize;

    /** log_2(block size of the parent cache). */
    unsigned lBlkSize;

    /** Only consult prefetcher on cache misses? */
    const bool onMiss;

    /** Consult prefetcher on reads? */
    const bool onRead;

    /** Consult prefetcher on reads? */
    const bool onWrite;

    /** Consult prefetcher on data accesses? */
    const bool onData;

    /** Consult prefetcher on instruction accesses? */
    const bool onInst;

    /** Request id for prefetches */
    const MasterID masterId;

    const Addr pageBytes;

    /** Prefetch on every access, not just misses */
    const bool prefetchOnAccess;

    /** Use Virtual Addresses for prefetching */
    const bool useVirtualAddresses;

    /** Determine if this access should be observed */
    bool observeAccess(const PacketPtr &pkt) const;

    /** Determine if address is in cache */
    bool inCache(Addr addr, bool is_secure) const;

    /** Determine if address is in cache miss queue */
    bool inMissQueue(Addr addr, bool is_secure) const;

    /** Determine if addresses are on the same page */
    bool samePage(Addr a, Addr b) const;
    /** Determine the address of the block in which a lays */
    Addr blockAddress(Addr a) const;
    /** Determine the address of a at block granularity */
    Addr blockIndex(Addr a) const;
    /** Determine the address of the page in which a lays */
    Addr pageAddress(Addr a) const;
    /** Determine the page-offset of a  */
    Addr pageOffset(Addr a) const;
    /** Build the address of the i-th block inside the page */
    Addr pageIthBlockAddress(Addr page, uint32_t i) const;

    Stats::Scalar pfIssued;

  public:

    BasePrefetcher(const BasePrefetcherParams *p);

    virtual ~BasePrefetcher() {}

    void setCache(BaseCache *_cache);

    /**
     * Notify prefetcher of cache access (may be any access or just
     * misses, depending on cache parameters.)
     */
    virtual void notify(const PacketPtr &pkt, const PrefetchInfo &pfi) = 0;

    virtual PacketPtr getPacket() = 0;

    virtual Tick nextPrefetchReadyTime() const = 0;

    /**
     * Register local statistics.
     */
    void regStats() override;

    /**
     * Register probe points for this object.
     */
    void regProbeListeners() override;

    /**
     * Process a notification event from the ProbeListener.
     * @param pkt The memory request causing the event
     */
    void probeNotify(const PacketPtr &pkt);

    /**
     * Add a SimObject and a probe name to listen events from
     * @param obj The SimObject pointer to listen from
     * @param name The probe name
     */
    void addEventProbe(SimObject *obj, const char *name);
};
#endif //__MEM_CACHE_PREFETCH_BASE_HH__
