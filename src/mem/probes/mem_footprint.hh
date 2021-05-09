/*
 * Copyright (c) 2016 Google Inc.
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and
 * shall not be construed as granting a license to any other
 * intellectual property including but not limited to intellectual
 * property relating to a hardware implementation of the
 * functionality of the software licensed hereunder.  You may use the
 * software subject to the license terms below provided that you
 * ensure that this notice is replicated unmodified and in its
 * entirety in all distributions of the software, modified or
 * unmodified, in source code or in binary form.
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

#ifndef __MEM_PROBES_MEM_FOOTPRINT_HH__
#define __MEM_PROBES_MEM_FOOTPRINT_HH__

#include <unordered_set>

#include "base/callback.hh"
#include "mem/packet.hh"
#include "mem/probes/base.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

namespace gem5
{

struct MemFootprintProbeParams;

/// Probe to track footprint of accessed memory
/// Two granularity of footprint measurement i.e. cache line and page
class MemFootprintProbe : public BaseMemProbe
{
  public:
    typedef std::unordered_set<Addr> AddrSet;

    MemFootprintProbe(const MemFootprintProbeParams &p);
    // Fix footprint tracking state on stat reset
    void statReset();

  protected:
    /// Cache Line size for footprint measurement (log2)
    const uint8_t cacheLineSizeLg2;
    /// Page size for footprint measurement (log2)
    const uint8_t pageSizeLg2;
    const uint64_t totalCacheLinesInMem;
    const uint64_t totalPagesInMem;

    void insertAddr(Addr addr, AddrSet *set, uint64_t limit);
    void handleRequest(const probing::PacketInfo &pkt_info) override;

    struct MemFootprintProbeStats : public statistics::Group
    {
        MemFootprintProbeStats(MemFootprintProbe *parent);

        /// Footprint at cache line size granularity
        statistics::Scalar cacheLine;
        /// Footprint at cache line size granularity, since simulation begin
        statistics::Scalar cacheLineTotal;
        /// Footprint at page granularity
        statistics::Scalar page;
        /// Footprint at page granularity, since simulation begin
        statistics::Scalar pageTotal;
    };

    // Addr set to track unique cache lines accessed
    AddrSet cacheLines;
    // Addr set to track unique cache lines accessed since simulation begin
    AddrSet cacheLinesAll;
    // Addr set to track unique pages accessed
    AddrSet pages;
    // Addr set to track unique pages accessed since simulation begin
    AddrSet pagesAll;
    System *system;

    MemFootprintProbeStats stats;
};

} // namespace gem5

#endif  //__MEM_PROBES_MEM_FOOTPRINT_HH__
