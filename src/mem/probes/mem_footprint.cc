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

#include "mem/probes/mem_footprint.hh"

#include "base/intmath.hh"
#include "params/MemFootprintProbe.hh"

MemFootprintProbe::MemFootprintProbe(MemFootprintProbeParams *p)
    : BaseMemProbe(p),
      cacheLineSizeLg2(floorLog2(p->system->cacheLineSize())),
      pageSizeLg2(floorLog2(p->page_size)),
      totalCacheLinesInMem(p->system->memSize() / p->system->cacheLineSize()),
      totalPagesInMem(p->system->memSize() / p->page_size),
      cacheLines(),
      cacheLinesAll(),
      pages(),
      pagesAll(),
      system(p->system)
{
    fatal_if(!isPowerOf2(system->cacheLineSize()),
             "MemFootprintProbe expects cache line size is power of 2.");
    fatal_if(!isPowerOf2(p->page_size),
             "MemFootprintProbe expects page size parameter is power of 2");
}

void
MemFootprintProbe::regStats()
{
    BaseMemProbe::regStats();

    using namespace Stats;
    // clang-format off
    fpCacheLine.name(name() + ".cacheline")
        .desc("Memory footprint at cache line granularity")
        .flags(nozero | nonan);
    fpCacheLineTotal.name(name() + ".cacheline_total")
        .desc("Total memory footprint at cache line granularity since "
              "simulation begin")
        .flags(nozero | nonan);
    fpPage.name(name() + ".page")
        .desc("Memory footprint at page granularity")
        .flags(nozero | nonan);
    fpPageTotal.name(name() + ".page_total")
        .desc("Total memory footprint at page granularity since simulation "
              "begin")
        .flags(nozero | nonan);
    // clang-format on

    registerResetCallback([this]() { statReset(); });
}

void
MemFootprintProbe::insertAddr(Addr addr, AddrSet *set, uint64_t limit)
{
    set->insert(addr);
    assert(set->size() <= limit);
}

void
MemFootprintProbe::handleRequest(const ProbePoints::PacketInfo &pi)
{
    if (!pi.cmd.isRequest() || !system->isMemAddr(pi.addr))
        return;

    const Addr cl_addr = (pi.addr >> cacheLineSizeLg2) << cacheLineSizeLg2;
    const Addr page_addr = (pi.addr >> pageSizeLg2) << pageSizeLg2;
    insertAddr(cl_addr, &cacheLines, totalCacheLinesInMem);
    insertAddr(cl_addr, &cacheLinesAll, totalCacheLinesInMem);
    insertAddr(page_addr, &pages, totalPagesInMem);
    insertAddr(page_addr, &pagesAll, totalPagesInMem);

    assert(cacheLines.size() <= cacheLinesAll.size());
    assert(pages.size() <= pagesAll.size());

    fpCacheLine = cacheLines.size() << cacheLineSizeLg2;
    fpCacheLineTotal = cacheLinesAll.size() << cacheLineSizeLg2;
    fpPage = pages.size() << pageSizeLg2;
    fpPageTotal = pagesAll.size() << pageSizeLg2;
}

void
MemFootprintProbe::statReset()
{
    cacheLines.clear();
    pages.clear();
}

MemFootprintProbe *
MemFootprintProbeParams::create()
{
    return new MemFootprintProbe(this);
}
