/*
 * Copyright (c) 2011-2015 Advanced Micro Devices, Inc.
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

#include "arch/amdgpu/common/tlb_coalescer.hh"

#include <cstring>

#include "arch/amdgpu/common/gpu_translation_state.hh"
#include "arch/x86/page_size.hh"
#include "base/logging.hh"
#include "debug/GPUTLB.hh"
#include "sim/process.hh"

namespace gem5
{

TLBCoalescer::TLBCoalescer(const Params &p)
    : ClockedObject(p),
      TLBProbesPerCycle(p.probesPerCycle),
      coalescingWindow(p.coalescingWindow),
      disableCoalescing(p.disableCoalescing),
      probeTLBEvent([this]{ processProbeTLBEvent(); },
                    "Probe the TLB below",
                    false, Event::CPU_Tick_Pri),
      cleanupEvent([this]{ processCleanupEvent(); },
                   "Cleanup issuedTranslationsTable hashmap",
                   false, Event::Maximum_Pri),
      stats(this)
{
    // create the response ports based on the number of connected ports
    for (size_t i = 0; i < p.port_cpu_side_ports_connection_count; ++i) {
        cpuSidePort.push_back(new CpuSidePort(csprintf("%s-port%d", name(), i),
                                              this, i));
    }

    // create the request ports based on the number of connected ports
    for (size_t i = 0; i < p.port_mem_side_ports_connection_count; ++i) {
        memSidePort.push_back(new MemSidePort(csprintf("%s-port%d", name(), i),
                                              this, i));
    }
}

Port &
TLBCoalescer::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "cpu_side_ports") {
        if (idx >= static_cast<PortID>(cpuSidePort.size())) {
            panic("TLBCoalescer::getPort: unknown index %d\n", idx);
        }

        return *cpuSidePort[idx];
    } else  if (if_name == "mem_side_ports") {
        if (idx >= static_cast<PortID>(memSidePort.size())) {
            panic("TLBCoalescer::getPort: unknown index %d\n", idx);
        }

        return *memSidePort[idx];
    } else {
        panic("TLBCoalescer::getPort: unknown port %s\n", if_name);
    }
}

/*
 * This method returns true if the <incoming_pkt>
 * can be coalesced with <coalesced_pkt> and false otherwise.
 * A given set of rules is checked.
 * The rules can potentially be modified based on the TLB level.
 */
bool
TLBCoalescer::canCoalesce(PacketPtr incoming_pkt, PacketPtr coalesced_pkt)
{
    if (disableCoalescing)
        return false;

    GpuTranslationState *incoming_state =
      safe_cast<GpuTranslationState*>(incoming_pkt->senderState);

    GpuTranslationState *coalesced_state =
     safe_cast<GpuTranslationState*>(coalesced_pkt->senderState);

    // Rule 1: Coalesce requests only if they
    // fall within the same virtual page
    Addr incoming_virt_page_addr = roundDown(incoming_pkt->req->getVaddr(),
                                             X86ISA::PageBytes);

    Addr coalesced_virt_page_addr = roundDown(coalesced_pkt->req->getVaddr(),
                                              X86ISA::PageBytes);

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
TLBCoalescer::updatePhysAddresses(PacketPtr pkt)
{
    Addr virt_page_addr = roundDown(pkt->req->getVaddr(), X86ISA::PageBytes);

    DPRINTF(GPUTLB, "Update phys. addr. for %d coalesced reqs for page %#x\n",
            issuedTranslationsTable[virt_page_addr].size(), virt_page_addr);

    GpuTranslationState *sender_state =
        safe_cast<GpuTranslationState*>(pkt->senderState);

    X86ISA::TlbEntry *tlb_entry =
        safe_cast<X86ISA::TlbEntry *>(sender_state->tlbEntry);
    assert(tlb_entry);
    Addr first_entry_vaddr = tlb_entry->vaddr;
    Addr first_entry_paddr = tlb_entry->paddr;
    int page_size = tlb_entry->size();
    bool uncacheable = tlb_entry->uncacheable;
    int first_hit_level = sender_state->hitLevel;

    // Get the physical page address of the translated request
    // Using the page_size specified in the TLBEntry allows us
    // to support different page sizes.
    Addr phys_page_paddr = pkt->req->getPaddr();
    phys_page_paddr &= ~(page_size - 1);

    for (int i = 0; i < issuedTranslationsTable[virt_page_addr].size(); ++i) {
        PacketPtr local_pkt = issuedTranslationsTable[virt_page_addr][i];
        GpuTranslationState *sender_state =
            safe_cast<GpuTranslationState*>(
                    local_pkt->senderState);

        // we are sending the packet back, so pop the reqCnt associated
        // with this level in the TLB hiearchy
        if (!sender_state->isPrefetch)
            sender_state->reqCnt.pop_back();

        /*
         * Only the first packet from this coalesced request has been
         * translated. Grab the translated phys. page addr and update the
         * physical addresses of the remaining packets with the appropriate
         * page offsets.
         */
        if (i) {
            Addr paddr = phys_page_paddr;
            paddr |= (local_pkt->req->getVaddr() & (page_size - 1));
            local_pkt->req->setPaddr(paddr);

            if (uncacheable)
                local_pkt->req->setFlags(Request::UNCACHEABLE);

            // update senderState->tlbEntry, so we can insert
            // the correct TLBEentry in the TLBs above.
            auto p = sender_state->tc->getProcessPtr();
            sender_state->tlbEntry =
                new X86ISA::TlbEntry(p->pid(), first_entry_vaddr,
                    first_entry_paddr, false, false);

            // update the hitLevel for all uncoalesced reqs
            // so that each packet knows where it hit
            // (used for statistics in the CUs)
            sender_state->hitLevel = first_hit_level;
        }

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
TLBCoalescer::CpuSidePort::recvTimingReq(PacketPtr pkt)
{
    // first packet of a coalesced request
    PacketPtr first_packet = nullptr;
    // true if we are able to do coalescing
    bool didCoalesce = false;
    // number of coalesced reqs for a given window
    int coalescedReq_cnt = 0;

    GpuTranslationState *sender_state =
        safe_cast<GpuTranslationState*>(pkt->senderState);

    // push back the port to remember the path back
    sender_state->ports.push_back(this);

    bool update_stats = !sender_state->isPrefetch;

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
        coalescer->stats.uncoalescedAccesses++;
        req_cnt = sender_state->reqCnt.back();
        DPRINTF(GPUTLB, "receiving pkt w/ req_cnt %d\n", req_cnt);
        coalescer->stats.queuingCycles -= (curTick() * req_cnt);
        coalescer->stats.localqueuingCycles -= curTick();
    }

    // FIXME if you want to coalesce not based on the issueTime
    // of the packets (i.e., from the compute unit's perspective)
    // but based on when they reached this coalescer then
    // remove the following if statement and use curTick() or
    // coalescingWindow for the tick_index.
    if (!sender_state->issueTime)
       sender_state->issueTime = curTick();

    // The tick index is used as a key to the coalescerFIFO hashmap.
    // It is shared by all candidates that fall within the
    // given coalescingWindow.
    int64_t tick_index = sender_state->issueTime / coalescer->coalescingWindow;

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
            coalescer->stats.coalescedAccesses++;

        std::vector<PacketPtr> new_array;
        new_array.push_back(pkt);
        coalescer->coalescerFIFO[tick_index].push_back(new_array);

        DPRINTF(GPUTLB, "coalescerFIFO[%d] now has %d coalesced reqs after "
                "push\n", tick_index,
                coalescer->coalescerFIFO[tick_index].size());
    }

    //schedule probeTLBEvent next cycle to send the
    //coalesced requests to the TLB
    if (!coalescer->probeTLBEvent.scheduled()) {
        coalescer->schedule(coalescer->probeTLBEvent,
                curTick() + coalescer->clockPeriod());
    }

    return true;
}

void
TLBCoalescer::CpuSidePort::recvReqRetry()
{
    panic("recvReqRetry called");
}

void
TLBCoalescer::CpuSidePort::recvFunctional(PacketPtr pkt)
{

    GpuTranslationState *sender_state =
        safe_cast<GpuTranslationState*>(pkt->senderState);

    bool update_stats = !sender_state->isPrefetch;

    if (update_stats)
        coalescer->stats.uncoalescedAccesses++;

    // If there is a pending timing request for this virtual address
    // print a warning message. This is a temporary caveat of
    // the current simulator where atomic and timing requests can
    // coexist. FIXME remove this check/warning in the future.
    Addr virt_page_addr = roundDown(pkt->req->getVaddr(), X86ISA::PageBytes);
    int map_count = coalescer->issuedTranslationsTable.count(virt_page_addr);

    if (map_count) {
        DPRINTF(GPUTLB, "Warning! Functional access to addr %#x sees timing "
                "req. pending\n", virt_page_addr);
    }

    coalescer->memSidePort[0]->sendFunctional(pkt);
}

AddrRangeList
TLBCoalescer::CpuSidePort::getAddrRanges() const
{
    // currently not checked by the requestor
    AddrRangeList ranges;

    return ranges;
}

bool
TLBCoalescer::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    // a translation completed and returned
    coalescer->updatePhysAddresses(pkt);

    return true;
}

void
TLBCoalescer::MemSidePort::recvReqRetry()
{
    //we've receeived a retry. Schedule a probeTLBEvent
    if (!coalescer->probeTLBEvent.scheduled())
        coalescer->schedule(coalescer->probeTLBEvent,
                curTick() + coalescer->clockPeriod());
}

void
TLBCoalescer::MemSidePort::recvFunctional(PacketPtr pkt)
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
TLBCoalescer::processProbeTLBEvent()
{
    // number of TLB probes sent so far
    int sent_probes = 0;
    // rejected denotes a blocking event
    bool rejected = false;

    // It is set to true either when the recvTiming of the TLB below
    // returns false or when there is another outstanding request for the
    // same virt. page.

    DPRINTF(GPUTLB, "triggered TLBCoalescer %s\n", __func__);

    for (auto iter = coalescerFIFO.begin();
         iter != coalescerFIFO.end() && !rejected; ) {
        int coalescedReq_cnt = iter->second.size();
        int i = 0;
        int vector_index = 0;

        DPRINTF(GPUTLB, "coalescedReq_cnt is %d for tick_index %d\n",
               coalescedReq_cnt, iter->first);

        while (i < coalescedReq_cnt) {
            ++i;
            PacketPtr first_packet = iter->second[vector_index][0];

            // compute virtual page address for this request
            Addr virt_page_addr = roundDown(first_packet->req->getVaddr(),
                    X86ISA::PageBytes);

            // is there another outstanding request for the same page addr?
            int pending_reqs =
                issuedTranslationsTable.count(virt_page_addr);

            if (pending_reqs) {
                DPRINTF(GPUTLB, "Cannot issue - There are pending reqs for "
                        "page %#x\n", virt_page_addr);

                ++vector_index;
                rejected = true;

                continue;
            }

            // send the coalesced request for virt_page_addr
            if (!memSidePort[0]->sendTimingReq(first_packet)) {
                DPRINTF(GPUTLB, "Failed to send TLB request for page %#x\n",
                       virt_page_addr);

                // No need for a retries queue since we are already buffering
                // the coalesced request in coalescerFIFO.
                rejected = true;
                ++vector_index;
            } else {
                GpuTranslationState *tmp_sender_state =
                    safe_cast<GpuTranslationState*>
                    (first_packet->senderState);

                bool update_stats = !tmp_sender_state->isPrefetch;

                if (update_stats) {
                    // req_cnt is total number of packets represented
                    // by the one we just sent counting all the way from
                    // the top of TLB hiearchy (i.e., from the CU)
                    int req_cnt = tmp_sender_state->reqCnt.back();
                    stats.queuingCycles += (curTick() * req_cnt);

                    DPRINTF(GPUTLB, "%s sending pkt w/ req_cnt %d\n",
                            name(), req_cnt);

                    // pkt_cnt is number of packets we coalesced into the one
                    // we just sent but only at this coalescer level
                    int pkt_cnt = iter->second[vector_index].size();
                    stats.localqueuingCycles += (curTick() * pkt_cnt);
                }

                DPRINTF(GPUTLB, "Successfully sent TLB request for page %#x",
                       virt_page_addr);

                //copy coalescedReq to issuedTranslationsTable
                issuedTranslationsTable[virt_page_addr]
                    = iter->second[vector_index];

                //erase the entry of this coalesced req
                iter->second.erase(iter->second.begin() + vector_index);

                if (iter->second.empty())
                    assert(i == coalescedReq_cnt);

                sent_probes++;
                if (sent_probes == TLBProbesPerCycle)
                   return;
            }
        }

        //if there are no more coalesced reqs for this tick_index
        //erase the hash_map with the first iterator
        if (iter->second.empty()) {
            coalescerFIFO.erase(iter++);
        } else {
            ++iter;
        }
    }
}

void
TLBCoalescer::processCleanupEvent()
{
    while (!cleanupQueue.empty()) {
        Addr cleanup_addr = cleanupQueue.front();
        cleanupQueue.pop();
        issuedTranslationsTable.erase(cleanup_addr);

        DPRINTF(GPUTLB, "Cleanup - Delete coalescer entry with key %#x\n",
                cleanup_addr);
    }
}

TLBCoalescer::TLBCoalescerStats::TLBCoalescerStats(statistics::Group *parent)
    : statistics::Group(parent),
      ADD_STAT(uncoalescedAccesses, "Number of uncoalesced TLB accesses"),
      ADD_STAT(coalescedAccesses, "Number of coalesced TLB accesses"),
      ADD_STAT(queuingCycles, "Number of cycles spent in queue"),
      ADD_STAT(localqueuingCycles,
               "Number of cycles spent in queue for all incoming reqs"),
      ADD_STAT(localLatency, "Avg. latency over all incoming pkts")
{
    localLatency = localqueuingCycles / uncoalescedAccesses;
}

} // namespace gem5
