/*
 * Copyright (c) 2021 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "arch/amdgpu/vega/tlb_coalescer.hh"

#include <cstring>

#include "arch/amdgpu/common/gpu_translation_state.hh"
#include "arch/amdgpu/vega/pagetable.hh"
#include "arch/generic/mmu.hh"
#include "base/logging.hh"
#include "debug/GPUTLB.hh"
#include "sim/process.hh"

namespace gem5
{

VegaTLBCoalescer::VegaTLBCoalescer(const VegaTLBCoalescerParams &p)
    : ClockedObject(p),
      TLBProbesPerCycle(p.probesPerCycle),
      coalescingWindow(p.coalescingWindow),
      disableCoalescing(p.disableCoalescing),
      probeTLBEvent([this] { processProbeTLBEvent(); }, "Probe the TLB below",
                    false, Event::CPU_Tick_Pri),
      cleanupEvent([this] { processCleanupEvent(); },
                   "Cleanup issuedTranslationsTable hashmap", false,
                   Event::Maximum_Pri),
      tlb_level(p.tlb_level),
      maxDownstream(p.maxDownstream),
      numDownstream(0)
{
    // create the response ports based on the number of connected ports
    for (size_t i = 0; i < p.port_cpu_side_ports_connection_count; ++i) {
        cpuSidePort.push_back(
            new CpuSidePort(csprintf("%s-port%d", name(), i), this, i));
    }

    // create the request ports based on the number of connected ports
    for (size_t i = 0; i < p.port_mem_side_ports_connection_count; ++i) {
        memSidePort.push_back(
            new MemSidePort(csprintf("%s-port%d", name(), i), this, i));
    }
}

Port &
VegaTLBCoalescer::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "cpu_side_ports") {
        if (idx >= static_cast<PortID>(cpuSidePort.size())) {
            panic("VegaTLBCoalescer::getPort: unknown index %d\n", idx);
        }

        return *cpuSidePort[idx];
    } else if (if_name == "mem_side_ports") {
        if (idx >= static_cast<PortID>(memSidePort.size())) {
            panic("VegaTLBCoalescer::getPort: unknown index %d\n", idx);
        }

        return *memSidePort[idx];
    } else {
        panic("VegaTLBCoalescer::getPort: unknown port %s\n", if_name);
    }
}

/*
 * This method returns true if the <incoming_pkt>
 * can be coalesced with <coalesced_pkt> and false otherwise.
 * A given set of rules is checked.
 * The rules can potentially be modified based on the TLB level.
 */
bool
VegaTLBCoalescer::canCoalesce(PacketPtr incoming_pkt, PacketPtr coalesced_pkt)
{
    if (disableCoalescing)
        return false;

    GpuTranslationState *incoming_state =
        safe_cast<GpuTranslationState *>(incoming_pkt->senderState);

    GpuTranslationState *coalesced_state =
        safe_cast<GpuTranslationState *>(coalesced_pkt->senderState);

    // Rule 1: Coalesce requests only if they
    // fall within the same virtual page
    Addr incoming_virt_page_addr =
        roundDown(incoming_pkt->req->getVaddr(), VegaISA::PageBytes);

    Addr coalesced_virt_page_addr =
        roundDown(coalesced_pkt->req->getVaddr(), VegaISA::PageBytes);

    if (incoming_virt_page_addr != coalesced_virt_page_addr)
        return false;

    //* Rule 2: Coalesce requests only if they
    // share a TLB Mode, i.e. they are both read
    // or write requests.
    BaseMMU::Mode incoming_mode = incoming_state->tlbMode;
    BaseMMU::Mode coalesced_mode = coalesced_state->tlbMode;

    if (incoming_mode != coalesced_mode)
        return false;

    // when we can coalesce a packet update the reqCnt
    // that is the number of packets represented by
    // this coalesced packet
    if (!incoming_state->isPrefetch)
        coalesced_state->reqCnt.back() += incoming_state->reqCnt.back();

    return true;
}

/*
 * We need to update the physical addresses of all the translation requests
 * that were coalesced into the one that just returned.
 */
void
VegaTLBCoalescer::updatePhysAddresses(PacketPtr pkt)
{
    Addr virt_page_addr = roundDown(pkt->req->getVaddr(), VegaISA::PageBytes);

    DPRINTF(GPUTLB, "Update phys. addr. for %d coalesced reqs for page %#x\n",
            issuedTranslationsTable[virt_page_addr].size(), virt_page_addr);

    GpuTranslationState *sender_state =
        safe_cast<GpuTranslationState *>(pkt->senderState);

    // Make a copy. This gets deleted after the first is sent back on the port
    assert(sender_state->tlbEntry);
    VegaISA::VegaTlbEntry tlb_entry =
        *safe_cast<VegaISA::VegaTlbEntry *>(sender_state->tlbEntry);
    Addr first_entry_vaddr = tlb_entry.vaddr;
    Addr first_entry_paddr = tlb_entry.paddr;
    int page_size = tlb_entry.size();
    bool uncacheable = tlb_entry.uncacheable();
    int first_hit_level = sender_state->hitLevel;
    bool is_system = pkt->req->systemReq();

    for (int i = 0; i < issuedTranslationsTable[virt_page_addr].size(); ++i) {
        PacketPtr local_pkt = issuedTranslationsTable[virt_page_addr][i];
        GpuTranslationState *sender_state =
            safe_cast<GpuTranslationState *>(local_pkt->senderState);

        // we are sending the packet back, so pop the reqCnt associated
        // with this level in the TLB hiearchy
        if (!sender_state->isPrefetch) {
            sender_state->reqCnt.pop_back();
            localCycles += curCycle();
        }

        /*
         * Only the first packet from this coalesced request has been
         * translated. Grab the translated phys. page addr and update the
         * physical addresses of the remaining packets with the appropriate
         * page offsets.
         */
        if (i) {
            Addr paddr = first_entry_paddr +
                         (local_pkt->req->getVaddr() & (page_size - 1));
            local_pkt->req->setPaddr(paddr);

            if (uncacheable)
                local_pkt->req->setFlags(Request::UNCACHEABLE);

            // update senderState->tlbEntry, so we can insert
            // the correct TLBEentry in the TLBs above.

            // auto p = sender_state->tc->getProcessPtr();
            if (sender_state->tlbEntry == NULL) {
                // not set by lower(l2) coalescer
                sender_state->tlbEntry = new VegaISA::VegaTlbEntry(
                    1 /* VMID TODO */, first_entry_vaddr, first_entry_paddr,
                    tlb_entry.logBytes, tlb_entry.pte);
            }

            // update the hitLevel for all uncoalesced reqs
            // so that each packet knows where it hit
            // (used for statistics in the CUs)
            sender_state->hitLevel = first_hit_level;
        }

        // Copy PTE system bit information to coalesced requests
        local_pkt->req->setSystemReq(is_system);

        ResponsePort *return_port = sender_state->ports.back();
        sender_state->ports.pop_back();

        // Translation is done - Convert to a response pkt if necessary and
        // send the translation back
        if (local_pkt->isRequest()) {
            local_pkt->makeTimingResponse();
        }

        return_port->sendTimingResp(local_pkt);
    }

    // schedule clean up for end of this cycle
    // This is a maximum priority event and must be on
    // the same cycle as GPUTLB cleanup event to prevent
    // race conditions with an IssueProbeEvent caused by
    // MemSidePort::recvReqRetry
    cleanupQueue.push(virt_page_addr);

    if (!cleanupEvent.scheduled())
        schedule(cleanupEvent, curTick());
}

// Receive translation requests, create a coalesced request,
// and send them to the TLB (TLBProbesPerCycle)
bool
VegaTLBCoalescer::CpuSidePort::recvTimingReq(PacketPtr pkt)
{
    // first packet of a coalesced request
    PacketPtr first_packet = nullptr;
    // true if we are able to do coalescing
    bool didCoalesce = false;
    // number of coalesced reqs for a given window
    int coalescedReq_cnt = 0;

    GpuTranslationState *sender_state =
        safe_cast<GpuTranslationState *>(pkt->senderState);

    bool update_stats = !sender_state->isPrefetch;

    if (coalescer->tlb_level == 1 && coalescer->mustStallCUPort(this))
        return false;

    // push back the port to remember the path back
    sender_state->ports.push_back(this);

    if (update_stats) {
        // if reqCnt is empty then this packet does not represent
        // multiple uncoalesced reqs(pkts) but just a single pkt.
        // If it does though then the reqCnt for each level in the
        // hierarchy accumulates the total number of reqs this packet
        // represents
        int req_cnt = 1;

        if (!sender_state->reqCnt.empty())
            req_cnt = sender_state->reqCnt.back();

        sender_state->reqCnt.push_back(req_cnt);

        // update statistics
        coalescer->uncoalescedAccesses++;
        req_cnt = sender_state->reqCnt.back();
        DPRINTF(GPUTLB, "receiving pkt w/ req_cnt %d\n", req_cnt);
        coalescer->queuingCycles -= (coalescer->curCycle() * req_cnt);
        coalescer->localqueuingCycles -= coalescer->curCycle();
        coalescer->localCycles -= coalescer->curCycle();
    }

    // Coalesce based on the time the packet arrives at the coalescer (here).
    if (!sender_state->issueTime)
        sender_state->issueTime = curTick();

    // The tick index is used as a key to the coalescerFIFO hashmap.
    // It is shared by all candidates that fall within the
    // given coalescingWindow.
    Tick tick_index = sender_state->issueTime / coalescer->coalescingWindow;

    if (coalescer->coalescerFIFO.count(tick_index)) {
        coalescedReq_cnt = coalescer->coalescerFIFO[tick_index].size();
    }

    // see if we can coalesce the incoming pkt with another
    // coalesced request with the same tick_index
    for (int i = 0; i < coalescedReq_cnt; ++i) {
        first_packet = coalescer->coalescerFIFO[tick_index][i][0];

        if (coalescer->canCoalesce(pkt, first_packet)) {
            coalescer->coalescerFIFO[tick_index][i].push_back(pkt);

            DPRINTF(GPUTLB, "Coalesced req %i w/ tick_index %d has %d reqs\n",
                    i, tick_index,
                    coalescer->coalescerFIFO[tick_index][i].size());

            didCoalesce = true;
            break;
        }
    }

    // if this is the first request for this tick_index
    // or we did not manage to coalesce, update stats
    // and make necessary allocations.
    if (!coalescedReq_cnt || !didCoalesce) {
        if (update_stats)
            coalescer->coalescedAccesses++;

        std::vector<PacketPtr> new_array;
        new_array.push_back(pkt);
        coalescer->coalescerFIFO[tick_index].push_back(new_array);

        DPRINTF(GPUTLB,
                "coalescerFIFO[%d] now has %d coalesced reqs after "
                "push\n",
                tick_index, coalescer->coalescerFIFO[tick_index].size());
    }

    // schedule probeTLBEvent next cycle to send the
    // coalesced requests to the TLB
    if (!coalescer->probeTLBEvent.scheduled()) {
        coalescer->schedule(coalescer->probeTLBEvent,
                            curTick() + coalescer->clockPeriod());
    }

    return true;
}

void
VegaTLBCoalescer::CpuSidePort::recvReqRetry()
{
    panic("recvReqRetry called");
}

void
VegaTLBCoalescer::CpuSidePort::recvFunctional(PacketPtr pkt)
{
    GpuTranslationState *sender_state =
        safe_cast<GpuTranslationState *>(pkt->senderState);

    bool update_stats = !sender_state->isPrefetch;

    if (update_stats)
        coalescer->uncoalescedAccesses++;

    Addr virt_page_addr = roundDown(pkt->req->getVaddr(), VegaISA::PageBytes);
    int map_count = coalescer->issuedTranslationsTable.count(virt_page_addr);

    if (map_count) {
        DPRINTF(GPUTLB,
                "Warning! Functional access to addr %#x sees timing "
                "req. pending\n",
                virt_page_addr);
    }

    coalescer->memSidePort[0]->sendFunctional(pkt);
}

AddrRangeList
VegaTLBCoalescer::CpuSidePort::getAddrRanges() const
{
    // currently not checked by the requestor
    AddrRangeList ranges;

    return ranges;
}

/*
 *  a translation completed and returned
 */
bool
VegaTLBCoalescer::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    coalescer->updatePhysAddresses(pkt);

    if (coalescer->tlb_level != 1)
        return true;

    coalescer->decrementNumDownstream();

    DPRINTF(GPUTLB,
            "recvTimingReq: clscr = %p, numDownstream = %d, max = %d\n",
            coalescer, coalescer->numDownstream, coalescer->maxDownstream);

    coalescer->unstallPorts();
    return true;
}

void
VegaTLBCoalescer::MemSidePort::recvReqRetry()
{
    // we've receeived a retry. Schedule a probeTLBEvent
    if (!coalescer->probeTLBEvent.scheduled())
        coalescer->schedule(coalescer->probeTLBEvent,
                            curTick() + coalescer->clockPeriod());
}

void
VegaTLBCoalescer::MemSidePort::recvFunctional(PacketPtr pkt)
{
    fatal("Memory side recvFunctional() not implemented in TLB coalescer.\n");
}

/*
 * Here we scan the coalescer FIFO and issue the max
 * number of permitted probes to the TLB below. We
 * permit bypassing of coalesced requests for the same
 * tick_index.
 *
 * We do not access the next tick_index unless we've
 * drained the previous one. The coalesced requests
 * that are successfully sent are moved to the
 * issuedTranslationsTable table (the table which keeps
 * track of the outstanding reqs)
 */
void
VegaTLBCoalescer::processProbeTLBEvent()
{
    // number of TLB probes sent so far
    int sent_probes = 0;

    // It is set to true either when the recvTiming of the TLB below
    // returns false or when there is another outstanding request for the
    // same virt. page.

    DPRINTF(GPUTLB, "triggered VegaTLBCoalescer %s\n", __func__);

    if ((tlb_level == 1) && (availDownstreamSlots() == 0)) {
        DPRINTF(GPUTLB, "IssueProbeEvent - no downstream slots, bail out\n");
        return;
    }

    for (auto iter = coalescerFIFO.begin(); iter != coalescerFIFO.end();) {
        int coalescedReq_cnt = iter->second.size();
        int i = 0;
        int vector_index = 0;

        DPRINTF(GPUTLB, "coalescedReq_cnt is %d for tick_index %d\n",
                coalescedReq_cnt, iter->first);

        while (i < coalescedReq_cnt) {
            ++i;
            PacketPtr first_packet = iter->second[vector_index][0];
            // The request to coalescer is origanized as follows.
            // The coalescerFIFO is a map which is indexed by coalescingWindow
            //  cycle. Only requests that falls in the same coalescingWindow
            //  considered for coalescing. Each entry of a coalescerFIFO is a
            //  vector of vectors. There is one entry for each different
            //  virtual page number and it contains vector of all request that
            //  are coalesced for the same virtual page address

            // compute virtual page address for this request
            Addr virt_page_addr =
                roundDown(first_packet->req->getVaddr(), VegaISA::PageBytes);

            // is there another outstanding request for the same page addr?
            int pending_reqs = issuedTranslationsTable.count(virt_page_addr);

            if (pending_reqs) {
                DPRINTF(GPUTLB,
                        "Cannot issue - There are pending reqs for "
                        "page %#x\n",
                        virt_page_addr);

                ++vector_index;
                continue;
            }

            // send the coalesced request for virt_page_addr
            if (!memSidePort[0]->sendTimingReq(first_packet)) {
                DPRINTF(GPUTLB, "Failed to send TLB request for page %#x",
                        virt_page_addr);

                // No need for a retries queue since we are already
                // buffering the coalesced request in coalescerFIFO.
                // Arka:: No point trying to send other requests to TLB at
                // this point since it is busy. Retries will be called later
                // by the TLB below
                return;
            } else {
                if (tlb_level == 1)
                    incrementNumDownstream();

                GpuTranslationState *tmp_sender_state =
                    safe_cast<GpuTranslationState *>(
                        first_packet->senderState);

                bool update_stats = !tmp_sender_state->isPrefetch;

                if (update_stats) {
                    // req_cnt is total number of packets represented
                    // by the one we just sent counting all the way from
                    // the top of TLB hiearchy (i.e., from the CU)
                    int req_cnt = tmp_sender_state->reqCnt.back();
                    queuingCycles += (curCycle() * req_cnt);

                    DPRINTF(GPUTLB, "%s sending pkt w/ req_cnt %d\n", name(),
                            req_cnt);

                    // pkt_cnt is number of packets we coalesced into the one
                    // we just sent but only at this coalescer level
                    int pkt_cnt = iter->second[vector_index].size();
                    localqueuingCycles += (curCycle() * pkt_cnt);
                }

                DPRINTF(GPUTLB, "Successfully sent TLB request for page %#x\n",
                        virt_page_addr);

                // copy coalescedReq to issuedTranslationsTable
                issuedTranslationsTable[virt_page_addr] =
                    iter->second[vector_index];

                // erase the entry of this coalesced req
                iter->second.erase(iter->second.begin() + vector_index);

                if (iter->second.empty())
                    assert(i == coalescedReq_cnt);

                sent_probes++;

                if (sent_probes == TLBProbesPerCycle ||
                    ((tlb_level == 1) && (!availDownstreamSlots()))) {
                    // Before returning make sure that empty vectors are taken
                    //  out. Not a big issue though since a later invocation
                    //  will take it out anyway.
                    if (iter->second.empty())
                        coalescerFIFO.erase(iter);

                    // schedule probeTLBEvent next cycle to send the
                    // coalesced requests to the TLB
                    if (!probeTLBEvent.scheduled()) {
                        schedule(probeTLBEvent,
                                 cyclesToTicks(curCycle() + Cycles(1)));
                    }
                    return;
                }
            }
        }

        // if there are no more coalesced reqs for this tick_index
        // erase the hash_map with the first iterator
        if (iter->second.empty()) {
            coalescerFIFO.erase(iter++);
        } else {
            ++iter;
        }
    }
}

void
VegaTLBCoalescer::processCleanupEvent()
{
    while (!cleanupQueue.empty()) {
        Addr cleanup_addr = cleanupQueue.front();
        cleanupQueue.pop();
        issuedTranslationsTable.erase(cleanup_addr);

        DPRINTF(GPUTLB, "Cleanup - Delete coalescer entry with key %#x\n",
                cleanup_addr);
    }
}

void
VegaTLBCoalescer::regStats()
{
    ClockedObject::regStats();

    uncoalescedAccesses.name(name() + ".uncoalesced_accesses")
        .desc("Number of uncoalesced TLB accesses");

    coalescedAccesses.name(name() + ".coalesced_accesses")
        .desc("Number of coalesced TLB accesses");

    queuingCycles.name(name() + ".queuing_cycles")
        .desc("Number of cycles spent in queue");

    localqueuingCycles.name(name() + ".local_queuing_cycles")
        .desc("Number of cycles spent in queue for all incoming reqs");

    localCycles.name(name() + ".local_cycles")
        .desc("Number of cycles spent in queue for all incoming reqs");

    localLatency.name(name() + ".local_latency")
        .desc("Avg. latency over all incoming pkts");

    latency.name(name() + ".latency")
        .desc("Avg. latency over all incoming pkts");

    localLatency = localqueuingCycles / uncoalescedAccesses;
    latency = localCycles / uncoalescedAccesses;
}

void
VegaTLBCoalescer::insertStalledPortIfNotMapped(CpuSidePort *port)
{
    assert(tlb_level == 1);
    if (stalledPortsMap.count(port) != 0)
        return; // we already know this port is stalled

    stalledPortsMap[port] = port;
    stalledPortsQueue.push(port);
    DPRINTF(GPUTLB,
            "insertStalledPortIfNotMapped: port %p, mapSz = %d, qsz = %d\n",
            port, stalledPortsMap.size(), stalledPortsQueue.size());
}

bool
VegaTLBCoalescer::mustStallCUPort(CpuSidePort *port)
{
    assert(tlb_level == 1);

    DPRINTF(GPUTLB, "mustStallCUPort: downstream = %d, max = %d\n",
            numDownstream, maxDownstream);

    if (availDownstreamSlots() == 0 || numDownstream == maxDownstream) {
        warn("RED ALERT - VegaTLBCoalescer::mustStallCUPort\n");
        insertStalledPortIfNotMapped(port);
        return true;
    } else
        return false;
}

void
VegaTLBCoalescer::unstallPorts()
{
    assert(tlb_level == 1);
    if (!stalledPorts() || availDownstreamSlots() == 0)
        return;

    DPRINTF(GPUTLB, "unstallPorts()\n");
    /*
     * this check is needed because we can be called from recvTiiningResponse()
     * or, synchronously due to having called sendRetry, from recvTimingReq()
     */
    if (availDownstreamSlots() == 0) // can happen if retry sent 1 downstream
        return;
    /*
     *  Consider this scenario
     *        1) max downstream is reached
     *        2) port1 tries to send a req, cant => stalledPortsQueue = [port1]
     *        3) port2 tries to send a req, cant => stalledPortsQueue = [port1,
     *              port2]
     *        4) a request completes and we remove port1 from both data
     *              structures & call
     *             sendRetry => stalledPortsQueue = [port2]
     *        5) port1 sends one req downstream and a second is rejected
     *             => stalledPortsQueue = [port2, port1]
     *
     *        so we round robin and each stalled port can send 1 req on retry
     */
    assert(availDownstreamSlots() == 1);
    auto port = stalledPortsQueue.front();
    DPRINTF(GPUTLB, "sending retry for port = %p(%s)\n", port, port->name());
    stalledPortsQueue.pop();
    auto iter = stalledPortsMap.find(port);
    assert(iter != stalledPortsMap.end());
    stalledPortsMap.erase(iter);
    port->sendRetryReq(); // cu will synchronously call recvTimingReq
}

} // namespace gem5
