/*
 * Copyright (c) 2023 ARM Limited
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

#include "mem/ruby/structures/RubyPrefetcherProxy.hh"

#include "debug/HWPrefetch.hh"
#include "mem/ruby/system/RubySystem.hh"

namespace gem5
{

namespace ruby
{

RubyPrefetcherProxy::RubyPrefetcherProxy(AbstractController *_parent,
                                         prefetch::Base *_prefetcher,
                                         MessageBuffer *_pf_queue)
    : Named(_parent->name()),
      prefetcher(_prefetcher),
      cacheCntrl(_parent),
      pfQueue(_pf_queue),
      pfEvent([this] { issuePrefetch(); }, name()),
      ppHit(nullptr),
      ppMiss(nullptr),
      ppFill(nullptr),
      ppDataUpdate(nullptr)
{
    fatal_if(!cacheCntrl,
             "initializing a RubyPrefetcherProxy without a parent");
    if (prefetcher) {
        fatal_if(
            !pfQueue,
            "%s initializing a RubyPrefetcherProxy without a prefetch queue",
            name());
        prefetcher->setParentInfo(cacheCntrl->params().system,
                                  cacheCntrl->getProbeManager(),
                                  RubySystem::getBlockSizeBytes());
    }
}

void
RubyPrefetcherProxy::scheduleNextPrefetch()
{
    if (pfEvent.scheduled())
        return;

    Tick next_pf_time = std::max(prefetcher->nextPrefetchReadyTime(),
                                 cacheCntrl->clockEdge(Cycles(1)));
    if (next_pf_time != MaxTick) {
        DPRINTF(HWPrefetch, "Next prefetch ready at %d\n", next_pf_time);
        cacheCntrl->schedule(&pfEvent, next_pf_time);
    }
}

void
RubyPrefetcherProxy::deschedulePrefetch()
{
    if (pfEvent.scheduled())
        cacheCntrl->deschedule(&pfEvent);
}

void
RubyPrefetcherProxy::completePrefetch(Addr addr)
{
    assert(makeLineAddress(addr) == addr);
    assert(issuedPfPkts.count(addr) == 1);
    DPRINTF(HWPrefetch, "Prefetch request for addr %#x completed\n", addr);
    delete issuedPfPkts[addr];
    issuedPfPkts.erase(addr);
}

void
RubyPrefetcherProxy::issuePrefetch()
{
    assert(prefetcher);
    assert(pfQueue);

    if (pfQueue->areNSlotsAvailable(1, curTick())) {
        PacketPtr pkt = prefetcher->getPacket();

        if (pkt) {
            DPRINTF(HWPrefetch, "Next prefetch ready %s\n", pkt->print());
            unsigned blk_size = RubySystem::getBlockSizeBytes();
            Addr line_addr = pkt->getBlockAddr(blk_size);

            if (issuedPfPkts.count(line_addr) == 0) {
                DPRINTF(HWPrefetch,
                        "Issued PF request for paddr=%#x, "
                        "line_addr=%#x, is_write=%d\n",
                        pkt->getAddr(), line_addr, pkt->needsWritable());

                RubyRequestType req_type = pkt->needsWritable() ?
                                               RubyRequestType_ST :
                                               RubyRequestType_LD;

                std::shared_ptr<RubyRequest> msg =
                    std::make_shared<RubyRequest>(
                        cacheCntrl->clockEdge(), pkt->getAddr(), blk_size,
                        0, // pc
                        req_type, RubyAccessMode_Supervisor, pkt,
                        PrefetchBit_Yes);

                // enqueue request into prefetch queue to the cache
                pfQueue->enqueue(msg, cacheCntrl->clockEdge(),
                                 cacheCntrl->cyclesToTicks(Cycles(1)));

                // track all pending PF requests
                issuedPfPkts[line_addr] = pkt;
            } else {
                DPRINTF(HWPrefetch, "Aborted PF request for address being "
                                    "prefetched\n");
                delete pkt;
            }
        }
    } else {
        DPRINTF(HWPrefetch, "No prefetch slots are available\n");
    }

    scheduleNextPrefetch();
}

void
RubyPrefetcherProxy::notifyPfHit(const RequestPtr &req, bool is_read,
                                 const DataBlock &data_blk)
{
    assert(ppHit);
    assert(req);
    Packet pkt(req,
               is_read ? Packet::makeReadCmd(req) : Packet::makeWriteCmd(req));
    // NOTE: for now we only communicate physical address with prefetchers
    pkt.dataStaticConst<uint8_t>(
        data_blk.getData(getOffset(req->getPaddr()), pkt.getSize()));
    DPRINTF(HWPrefetch, "notify hit: %s\n", pkt.print());
    ppHit->notify(CacheAccessProbeArg(&pkt, *this));
    scheduleNextPrefetch();
}

void
RubyPrefetcherProxy::notifyPfMiss(const RequestPtr &req, bool is_read,
                                  const DataBlock &data_blk)
{
    assert(ppMiss);
    assert(req);
    Packet pkt(req,
               is_read ? Packet::makeReadCmd(req) : Packet::makeWriteCmd(req));
    // NOTE: for now we only communicate physical address with prefetchers
    pkt.dataStaticConst<uint8_t>(
        data_blk.getData(getOffset(req->getPaddr()), pkt.getSize()));
    DPRINTF(HWPrefetch, "notify miss: %s\n", pkt.print());
    ppMiss->notify(CacheAccessProbeArg(&pkt, *this));
    scheduleNextPrefetch();
}

void
RubyPrefetcherProxy::notifyPfFill(const RequestPtr &req,
                                  const DataBlock &data_blk, bool from_pf)
{
    assert(ppFill);
    assert(req);
    Packet pkt(req, Packet::makeReadCmd(req));
    if (from_pf)
        pkt.cmd = Packet::Command::HardPFReq;
    // NOTE: for now we only communicate physical address with prefetchers
    pkt.dataStaticConst<uint8_t>(
        data_blk.getData(getOffset(req->getPaddr()), pkt.getSize()));
    DPRINTF(HWPrefetch, "notify fill: %s\n", pkt.print());
    ppFill->notify(CacheAccessProbeArg(&pkt, *this));
    scheduleNextPrefetch();
}

void
RubyPrefetcherProxy::notifyPfEvict(Addr blkAddr, bool hwPrefetched,
                                   RequestorID requestorID)
{
    DPRINTF(HWPrefetch, "notify evict: %#x hw_pf=%d\n", blkAddr, hwPrefetched);
    CacheDataUpdateProbeArg data_update(blkAddr, false, requestorID, *this);
    data_update.hwPrefetched = hwPrefetched;
    ppDataUpdate->notify(data_update);
    scheduleNextPrefetch();
}

void
RubyPrefetcherProxy::regProbePoints()
{
    assert(cacheCntrl);
    ppHit = new ProbePointArg<CacheAccessProbeArg>(
        cacheCntrl->getProbeManager(), "Hit");
    ppMiss = new ProbePointArg<CacheAccessProbeArg>(
        cacheCntrl->getProbeManager(), "Miss");
    ppFill = new ProbePointArg<CacheAccessProbeArg>(
        cacheCntrl->getProbeManager(), "Fill");
    ppDataUpdate = new ProbePointArg<CacheDataUpdateProbeArg>(
        cacheCntrl->getProbeManager(), "Data Update");
}

} // namespace ruby
} // namespace gem5
