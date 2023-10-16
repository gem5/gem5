/*
 * Copyright (c) 2013-2016,2019 ARM Limited
 * All rights reserved
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
 * Definition of a snoop filter.
 */

#ifndef __MEM_SNOOP_FILTER_HH__
#define __MEM_SNOOP_FILTER_HH__

#include <bitset>
#include <unordered_map>
#include <utility>

#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/qport.hh"
#include "params/SnoopFilter.hh"
#include "sim/sim_object.hh"
#include "sim/system.hh"

namespace gem5
{

/**
 * This snoop filter keeps track of which connected port has a
 * particular line of data. It can be queried (through lookup*) on
 * memory requests from above (reads / writes / ...); and also from
 * below (snoops). The snoop filter precisely knows about the location
 * of lines "above" it through a map from cache line address to
 * sharers/ports. The snoop filter ties into the flows of requests
 * (when they succeed at the lower interface), regular responses from
 * below and also responses from sideway's caches (in update*). This
 * allows the snoop filter to model cache-line residency by snooping
 * the messages.
 *
 * The tracking happens in two fields to be able to distinguish
 * between in-flight requests (in requested) and already pulled in
 * lines (in holder). This distinction is used for producing tighter
 * assertions and tracking request completion. For safety, (requested
 * | holder) should be notified and the requesting MSHRs will take
 * care of ordering.
 *
 * Overall, some trickery is required because:
 * (1) snoops are not followed by an ACK, but only evoke a response if
 *     they need to (hit dirty)
 * (2) side-channel information is funnelled through direct modifications of
 *     pkt, instead of proper messages through the bus
 * (3) there are no clean evict messages telling the snoop filter that a local,
 *     upper cache dropped a line, making the snoop filter pessimistic for now
 * (4) ordering: there is no single point of order in the system.  Instead,
 *     requesting MSHRs track order between local requests and remote snoops
 */
class SnoopFilter : public SimObject
{
  public:

    // Change for systems with more than 256 ports tracked by this object
    static const int SNOOP_MASK_SIZE = 256;

    typedef std::vector<QueuedResponsePort*> SnoopList;

    SnoopFilter (const SnoopFilterParams &p) :
        SimObject(p), reqLookupResult(cachedLocations.end()),
        linesize(p.system->cacheLineSize()), lookupLatency(p.lookup_latency),
        maxEntryCount(p.max_capacity / p.system->cacheLineSize()),
        stats(this)
    {
    }

    /**
     * Init a new snoop filter and tell it about all the cpu_sideports
     * of the enclosing bus.
     *
     * @param _cpu_side_ports Response ports that the bus is attached to.
     */
    void setCPUSidePorts(const SnoopList& _cpu_side_ports) {
        localResponsePortIds.resize(_cpu_side_ports.size(), InvalidPortID);

        PortID id = 0;
        for (const auto& p : _cpu_side_ports) {
            // no need to track this port if it is not snooping
            if (p->isSnooping()) {
                cpuSidePorts.push_back(p);
                localResponsePortIds[p->getId()] = id++;
            }
        }

        // make sure we can deal with this many ports
        fatal_if(id > SNOOP_MASK_SIZE,
                 "Snoop filter only supports %d snooping ports, got %d\n",
                 SNOOP_MASK_SIZE, id);
    }

    /**
     * Lookup a request (from a CPU-side port) in the snoop filter and
     * return a list of other CPU-side ports that need forwarding of the
     * resulting snoops.  Additionally, update the tracking structures
     * with new request information. Note that the caller must also
     * call finishRequest once it is known if the request needs to
     * retry or not.
     *
     * @param cpkt              Pointer to the request packet. Not changed.
     * @param cpu_side_port     Response port where the request came from.
     * @return Pair of a vector of snoop target ports and lookup latency.
     */
    std::pair<SnoopList, Cycles> lookupRequest(const Packet* cpkt,
                                        const ResponsePort& cpu_side_port);

    /**
     * For an un-successful request, revert the change to the snoop
     * filter. Also take care of erasing any null entries. This method
     * relies on the result from lookupRequest being stored in
     * reqLookupResult.
     *
     * @param will_retry    This request will retry on this bus / snoop filter
     * @param addr          Packet address, merely for sanity checking
     */
    void finishRequest(bool will_retry, Addr addr, bool is_secure);

    /**
     * Handle an incoming snoop from below (the memory-side port). These
     * can upgrade the tracking logic and may also benefit from
     * additional steering thanks to the snoop filter.
     *
     * @param cpkt Pointer to const Packet containing the snoop.
     * @return Pair with a vector of ResponsePorts that need snooping and a
     * lookup latency.
     */
    std::pair<SnoopList, Cycles> lookupSnoop(const Packet* cpkt);

    /**
     * Let the snoop filter see any snoop responses that turn into
     * request responses and indicate cache to cache transfers. These
     * will update the corresponding state in the filter.
     *
     * @param cpkt     Pointer to const Packet holding the snoop response.
     * @param rsp_port ResponsePort that sends the response.
     * @param req_port ResponsePort that made the original request and is the
     *                 destination of the snoop response.
     */
    void updateSnoopResponse(const Packet *cpkt, const ResponsePort& rsp_port,
                             const ResponsePort& req_port);

    /**
     * Pass snoop responses that travel downward through the snoop
     * filter and let them update the snoop filter state.  No
     * additional routing happens.
     *
     * @param cpkt     Pointer to const Packet holding the snoop response.
     * @param rsp_port ResponsePort that sends the response.
     * @param req_port RequestPort through which the response is forwarded.
     */
    void updateSnoopForward(const Packet *cpkt, const ResponsePort& rsp_port,
                            const RequestPort& req_port);

    /**
     * Update the snoop filter with a response from below (outer /
     * other cache, or memory) and update the tracking information in
     * the snoop filter.
     *
     * @param cpkt          Pointer to const Packet holding the snoop response.
     * @param cpu_side_port ResponsePort that made the original request and
     *                      is the target of this response.
     */
    void updateResponse(const Packet *cpkt, const ResponsePort& cpu_side_port);

    virtual void regStats();

  protected:

    /**
     * The underlying type for the bitmask we use for tracking. This
     * limits the number of snooping ports supported per crossbar.
     */
    typedef std::bitset<SNOOP_MASK_SIZE> SnoopMask;

    /**
    * Per cache line item tracking a bitmask of ResponsePorts who have an
    * outstanding request to this line (requested) or already share a
    * cache line with this address (holder).
    */
    struct SnoopItem
    {
        SnoopMask requested;
        SnoopMask holder;
    };
    /**
     * HashMap of SnoopItems indexed by line address
     */
    typedef std::unordered_map<Addr, SnoopItem> SnoopFilterCache;

    /**
     * Simple factory methods for standard return values.
     */
    std::pair<SnoopList, Cycles> snoopAll(Cycles latency) const
    {
        return std::make_pair(cpuSidePorts, latency);
    }
    std::pair<SnoopList, Cycles> snoopSelected(const SnoopList&
                                _cpu_side_ports, Cycles latency) const
    {
        return std::make_pair(_cpu_side_ports, latency);
    }
    std::pair<SnoopList, Cycles> snoopDown(Cycles latency) const
    {
        SnoopList empty;
        return std::make_pair(empty , latency);
    }

    /**
     * Convert a single port to a corresponding, one-hot bitmask
     * @param port ResponsePort that should be converted.
     * @return One-hot bitmask corresponding to the port.
     */
    SnoopMask portToMask(const ResponsePort& port) const;
    /**
     * Converts a bitmask of ports into the corresponing list of ports
     * @param ports SnoopMask of the requested ports
     * @return SnoopList containing all the requested ResponsePorts
     */
    SnoopList maskToPortList(SnoopMask ports) const;

  private:

    /**
     * Removes snoop filter items which have no requestors and no holders.
     */
    void eraseIfNullEntry(SnoopFilterCache::iterator& sf_it);

    /** Simple hash set of cached addresses. */
    SnoopFilterCache cachedLocations;

    /**
     * A request lookup must be followed by a call to finishRequest to inform
     * the operation's success. If a retry is needed, however, all changes
     * made to the snoop filter while performing the lookup must be undone.
     * This structure keeps track of the state previous to such changes.
     */
    struct ReqLookupResult
    {
        /** Iterator used to store the result from lookupRequest. */
        SnoopFilterCache::iterator it;

        /**
         * Variable to temporarily store value of snoopfilter entry
         * in case finishRequest needs to undo changes made in lookupRequest
         * (because of crossbar retry)
         */
        SnoopItem retryItem;

        /**
         * The constructor must be informed of the internal cache's end
         * iterator, so do not allow the compiler to implictly define it.
         *
         * @param end_it Iterator to the end of the internal cache.
         */
        ReqLookupResult(SnoopFilterCache::iterator end_it)
            : it(end_it), retryItem{0, 0}
        {
        }
        ReqLookupResult() = delete;
    } reqLookupResult;

    /** List of all attached snooping CPU-side ports. */
    SnoopList cpuSidePorts;
    /** Track the mapping from port ids to the local mask ids. */
    std::vector<PortID> localResponsePortIds;
    /** Cache line size. */
    const Addr linesize;
    /** Latency for doing a lookup in the filter */
    const Cycles lookupLatency;
    /** Max capacity in terms of cache blocks tracked, for sanity checking */
    const unsigned maxEntryCount;

    /**
     * Use the lower bits of the address to keep track of the line status
     */
    enum LineStatus
    {
        /** block holds data from the secure memory space */
        LineSecure = 0x01,
    };

    /** Statistics */
    struct SnoopFilterStats : public statistics::Group
    {
        SnoopFilterStats(statistics::Group *parent);

        statistics::Scalar totRequests;
        statistics::Scalar hitSingleRequests;
        statistics::Scalar hitMultiRequests;

        statistics::Scalar totSnoops;
        statistics::Scalar hitSingleSnoops;
        statistics::Scalar hitMultiSnoops;
    } stats;
};

inline SnoopFilter::SnoopMask
SnoopFilter::portToMask(const ResponsePort& port) const
{
    assert(port.getId() != InvalidPortID);
    // if this is not a snooping port, return a zero mask
    return !port.isSnooping() ? 0 :
        ((SnoopMask)1) << localResponsePortIds[port.getId()];
}

inline SnoopFilter::SnoopList
SnoopFilter::maskToPortList(SnoopMask port_mask) const
{
    SnoopList res;
    for (const auto& p : cpuSidePorts)
        if ((port_mask & portToMask(*p)).any())
            res.push_back(p);
    return res;
}

} // namespace gem5

#endif // __MEM_SNOOP_FILTER_HH__
