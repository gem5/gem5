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

#include "arch/amdgpu/vega/tlb.hh"

#include <cmath>
#include <cstring>

#include "arch/amdgpu/common/gpu_translation_state.hh"
#include "arch/amdgpu/vega/faults.hh"
#include "arch/amdgpu/vega/pagetable_walker.hh"
#include "debug/GPUPrefetch.hh"
#include "debug/GPUTLB.hh"
#include "dev/amdgpu/amdgpu_device.hh"

namespace gem5
{
namespace VegaISA
{

// we have no limit for the number of translations we send
// downstream as we depend on the limit of the coalescer
// above us
GpuTLB::GpuTLB(const VegaGPUTLBParams &p)
    :  ClockedObject(p), walker(p.walker),
      gpuDevice(p.gpu_device), size(p.size), stats(this),
      cleanupEvent([this]{ cleanup(); }, name(), false,
                   Event::Maximum_Pri)
{
    assoc = p.assoc;
    assert(assoc <= size);
    numSets = size/assoc;
    allocationPolicy = p.allocationPolicy;
    hasMemSidePort = false;

    tlb.assign(size, VegaTlbEntry());

    freeList.resize(numSets);
    entryList.resize(numSets);

    for (int set = 0; set < numSets; ++set) {
        for (int way = 0; way < assoc; ++way) {
            int x = set * assoc + way;
            freeList[set].push_back(&tlb.at(x));
        }
    }

    FA = (size == assoc);
    setMask = numSets - 1;

    maxCoalescedReqs = p.maxOutstandingReqs;


    outstandingReqs = 0;
    hitLatency = p.hitLatency;
    missLatency1 = p.missLatency1;
    missLatency2 = p.missLatency2;

    // create the response ports based on the number of connected ports
    for (size_t i = 0; i < p.port_cpu_side_ports_connection_count; ++i) {
        cpuSidePort.push_back(new CpuSidePort(csprintf("%s-port%d",
                              name(), i), this, i));
    }

    // create the requestor ports based on the number of connected ports
    for (size_t i = 0; i < p.port_mem_side_ports_connection_count; ++i) {
        memSidePort.push_back(new MemSidePort(csprintf("%s-port%d",
                              name(), i), this, i));
    }

    // assuming one walker per TLB, set our walker's TLB to this TLB.
    walker->setTLB(this);

    // gpuDevice should be non-null in full system only and is set by GpuTLB
    // params from the config file.
    if (gpuDevice) {
        gpuDevice->getVM().registerTLB(this);
    }
}

GpuTLB::~GpuTLB()
{
}

Port &
GpuTLB::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "cpu_side_ports") {
        if (idx >= static_cast<PortID>(cpuSidePort.size())) {
            panic("TLBCoalescer::getPort: unknown index %d\n", idx);
        }

        return *cpuSidePort[idx];
    } else if (if_name == "mem_side_ports") {
        if (idx >= static_cast<PortID>(memSidePort.size())) {
            panic("TLBCoalescer::getPort: unknown index %d\n", idx);
        }

        hasMemSidePort = true;

        return *memSidePort[idx];
    } else {
        panic("TLBCoalescer::getPort: unknown port %s\n", if_name);
    }
}

Fault
GpuTLB::createPagefault(Addr vaddr, Mode mode)
{
    DPRINTF(GPUTLB, "GPUTLB: Raising page fault.\n");
    ExceptionCode code;
    if (mode == BaseMMU::Read)
        code = ExceptionCode::LOAD_PAGE;
    else if (mode == BaseMMU::Write)
        code = ExceptionCode::STORE_PAGE;
    else
        code = ExceptionCode::INST_PAGE;
    return std::make_shared<PageFault>(vaddr, code, true, mode, true);
}

Addr
GpuTLB::pageAlign(Addr vaddr)
{
    Addr pageMask = mask(VegaISA::PageShift);
    return (vaddr & ~pageMask);
}

VegaTlbEntry*
GpuTLB::insert(Addr vpn, VegaTlbEntry &entry)
{
    VegaTlbEntry *newEntry = nullptr;

    /**
     * vpn holds the virtual page address assuming native page size.
     * However, we need to check the entry size as Vega supports
     * flexible page sizes of arbitrary size. The set will assume
     * native page size but the vpn needs to be fixed up to consider
     * the flexible page size.
     */
    Addr real_vpn = vpn & ~(entry.size() - 1);

    /**
     * Also fix up the ppn as this is used in the math later to compute paddr.
     */
    Addr real_ppn = entry.paddr & ~(entry.size() - 1);

    int set = (real_vpn >> VegaISA::PageShift) & setMask;

    DPRINTF(GPUTLB, "Inserted %#lx -> %#lx of size %#lx into set %d\n",
            real_vpn, real_ppn, entry.size(), set);

    if (!freeList[set].empty()) {
        newEntry = freeList[set].front();
        freeList[set].pop_front();
    } else {
        newEntry = entryList[set].back();
        entryList[set].pop_back();
    }

    *newEntry = entry;
    newEntry->vaddr = real_vpn;
    newEntry->paddr = real_ppn;
    entryList[set].push_front(newEntry);

    return newEntry;
}

GpuTLB::EntryList::iterator
GpuTLB::lookupIt(Addr va, bool update_lru)
{
    int set = (va >> VegaISA::PageShift) & setMask;

    if (FA) {
        assert(!set);
    }

    auto entry = entryList[set].begin();
    for (; entry != entryList[set].end(); ++entry) {
        int page_size = (*entry)->size();

        if ((*entry)->vaddr <= va && (*entry)->vaddr + page_size > va) {
            DPRINTF(GPUTLB, "Matched vaddr %#x to entry starting at %#x "
                    "with size %#x.\n", va, (*entry)->vaddr, page_size);

            if (update_lru) {
                entryList[set].push_front(*entry);
                entryList[set].erase(entry);
                entry = entryList[set].begin();
            }

            break;
        }
    }

    return entry;
}

VegaTlbEntry*
GpuTLB::lookup(Addr va, bool update_lru)
{
    int set = (va >> VegaISA::PageShift) & setMask;

    auto entry = lookupIt(va, update_lru);

    if (entry == entryList[set].end())
        return nullptr;
    else
        return *entry;
}

void
GpuTLB::invalidateAll()
{
    DPRINTF(GPUTLB, "Invalidating all entries.\n");

    for (int i = 0; i < numSets; ++i) {
        while (!entryList[i].empty()) {
            VegaTlbEntry *entry = entryList[i].front();
            entryList[i].pop_front();
            freeList[i].push_back(entry);
        }
    }
}

void
GpuTLB::demapPage(Addr va, uint64_t asn)
{

    int set = (va >> VegaISA::PageShift) & setMask;
    auto entry = lookupIt(va, false);

    if (entry != entryList[set].end()) {
        freeList[set].push_back(*entry);
        entryList[set].erase(entry);
    }
}



/**
 * TLB_lookup will only perform a TLB lookup returning the TLB entry on a TLB
 * hit and nullptr on a TLB miss.
 * Many of the checks about different modes have been converted to
 * assertions, since these parts of the code are not really used.
 * On a hit it will update the LRU stack.
 */
VegaTlbEntry *
GpuTLB::tlbLookup(const RequestPtr &req, bool update_stats)
{
    Addr vaddr = req->getVaddr();
    Addr alignedVaddr = pageAlign(vaddr);
    DPRINTF(GPUTLB, "TLB Lookup for vaddr %#x.\n", vaddr);

    //update LRU stack on a hit
    VegaTlbEntry *entry = lookup(alignedVaddr, true);

    if (!update_stats) {
        // functional tlb access for memory initialization
        // i.e., memory seeding or instr. seeding -> don't update
        // TLB and stats
        return entry;
    }

    stats.localNumTLBAccesses++;

    if (!entry) {
        stats.localNumTLBMisses++;
    } else {
        stats.localNumTLBHits++;
    }

    return entry;
}

Walker*
GpuTLB::getWalker()
{
    return walker;
}


void
GpuTLB::serialize(CheckpointOut &cp) const
{
}

void
GpuTLB::unserialize(CheckpointIn &cp)
{
}

/**
 * Do the TLB lookup for this coalesced request and schedule
 * another event <TLB access latency> cycles later.
 */

void
GpuTLB::issueTLBLookup(PacketPtr pkt)
{
    assert(pkt);
    assert(pkt->senderState);

    /**
     * The page size is not fixed in Vega and tracking events by VPN could
     * potentially lead to redundant page walks by using the smallest page
     * size. The actual VPN can be determined after the first walk is done
     * and fixed up later.
     */
    Addr virt_page_addr = roundDown(pkt->req->getVaddr(),
                                    VegaISA::PageBytes);

    GpuTranslationState *sender_state =
            safe_cast<GpuTranslationState*>(pkt->senderState);

    bool update_stats = !sender_state->isPrefetch;

    DPRINTF(GPUTLB, "Translation req. for virt. page addr %#x\n",
            virt_page_addr);

    int req_cnt = sender_state->reqCnt.back();

    if (update_stats) {
        stats.accessCycles -= (curCycle() * req_cnt);
        stats.localCycles -= curCycle();
        stats.globalNumTLBAccesses += req_cnt;
    }

    tlbOutcome lookup_outcome = TLB_MISS;
    const RequestPtr &tmp_req = pkt->req;

    // Access the TLB and figure out if it's a hit or a miss.
    auto entry = tlbLookup(tmp_req, update_stats);

    if (entry) {
        lookup_outcome = TLB_HIT;
        // Put the entry in SenderState
        VegaTlbEntry *entry = lookup(virt_page_addr, false);
        assert(entry);

        // Set if this is a system request
        pkt->req->setSystemReq(entry->pte.s);

        Addr alignedPaddr = pageAlign(entry->paddr);
        sender_state->tlbEntry =
            new VegaTlbEntry(1 /* VMID */, virt_page_addr, alignedPaddr,
                            entry->logBytes, entry->pte);

        if (update_stats) {
            // the reqCnt has an entry per level, so its size tells us
            // which level we are in
            sender_state->hitLevel = sender_state->reqCnt.size();
            stats.globalNumTLBHits += req_cnt;
        }
    } else {
        if (update_stats)
            stats.globalNumTLBMisses += req_cnt;
    }

    /*
     * We now know the TLB lookup outcome (if it's a hit or a miss), as
     * well as the TLB access latency.
     *
     * We create and schedule a new TLBEvent which will help us take the
     * appropriate actions (e.g., update TLB on a hit, send request to
     * lower level TLB on a miss, or start a page walk if this was the
     * last-level TLB)
     */
    TLBEvent *tlb_event =
        new TLBEvent(this, virt_page_addr, lookup_outcome, pkt);

    if (translationReturnEvent.count(virt_page_addr)) {
        panic("Virtual Page Address %#x already has a return event\n",
              virt_page_addr);
    }

    translationReturnEvent[virt_page_addr] = tlb_event;
    assert(tlb_event);

    DPRINTF(GPUTLB, "schedule translationReturnEvent @ curTick %d\n",
            curTick() + cyclesToTicks(Cycles(hitLatency)));

    schedule(tlb_event, curTick() + cyclesToTicks(Cycles(hitLatency)));
}

GpuTLB::TLBEvent::TLBEvent(GpuTLB* _tlb, Addr _addr,
    tlbOutcome tlb_outcome, PacketPtr _pkt)
        : Event(CPU_Tick_Pri), tlb(_tlb), virtPageAddr(_addr),
          outcome(tlb_outcome), pkt(_pkt)
{
}

/**
 * Do Paging protection checks. If we encounter a page fault, then
 * an assertion is fired.
 */
void
GpuTLB::pagingProtectionChecks(PacketPtr pkt, VegaTlbEntry * tlb_entry,
        Mode mode)
{
    // Do paging protection checks.
    bool badWrite = (!tlb_entry->writable());

    if (mode == BaseMMU::Write && badWrite) {
        // The page must have been present to get into the TLB in
        // the first place. We'll assume the reserved bits are
        // fine even though we're not checking them.
        fatal("Page fault on addr %lx PTE=%#lx", pkt->req->getVaddr(),
                (uint64_t)tlb_entry->pte);
    }
}

void
GpuTLB::walkerResponse(VegaTlbEntry& entry, PacketPtr pkt)
{
    DPRINTF(GPUTLB, "WalkerResponse for %#lx. Entry: (%#lx, %#lx, %#lx)\n",
            pkt->req->getVaddr(), entry.vaddr, entry.paddr, entry.size());

    Addr virt_page_addr = roundDown(pkt->req->getVaddr(),
                                    VegaISA::PageBytes);

    Addr page_addr = entry.pte.ppn << VegaISA::PageShift;
    Addr paddr = page_addr + (entry.vaddr & mask(entry.logBytes));
    pkt->req->setPaddr(paddr);
    pkt->req->setSystemReq(entry.pte.s);

    GpuTranslationState *sender_state =
        safe_cast<GpuTranslationState*>(pkt->senderState);
    sender_state->tlbEntry = new VegaTlbEntry(entry);

    handleTranslationReturn(virt_page_addr, TLB_MISS, pkt);
}

/**
 * handleTranslationReturn is called on a TLB hit,
 * when a TLB miss returns or when a page fault returns.
 * The latter calls handelHit with TLB miss as tlbOutcome.
 */
void
GpuTLB::handleTranslationReturn(Addr virt_page_addr,
    tlbOutcome tlb_outcome, PacketPtr pkt)
{
    assert(pkt);
    Addr vaddr = pkt->req->getVaddr();

    GpuTranslationState *sender_state =
        safe_cast<GpuTranslationState*>(pkt->senderState);

    Mode mode = sender_state->tlbMode;

    VegaTlbEntry *local_entry, *new_entry;

    int req_cnt = sender_state->reqCnt.back();
    bool update_stats = !sender_state->isPrefetch;

    if (update_stats) {
        stats.accessCycles += (req_cnt * curCycle());
        stats.localCycles += curCycle();
    }

    if (tlb_outcome == TLB_HIT) {
        DPRINTF(GPUTLB, "Translation Done - TLB Hit for addr %#x\n",
            vaddr);
        local_entry = safe_cast<VegaTlbEntry *>(sender_state->tlbEntry);
    } else {
        DPRINTF(GPUTLB, "Translation Done - TLB Miss for addr %#x\n",
                vaddr);

        /**
         * We are returning either from a page walk or from a hit at a
         * lower TLB level. The senderState should be "carrying" a pointer
         * to the correct TLBEntry.
         */
        new_entry = safe_cast<VegaTlbEntry *>(sender_state->tlbEntry);
        assert(new_entry);
        local_entry = new_entry;

        if (allocationPolicy) {
            assert(new_entry->pte);
            DPRINTF(GPUTLB, "allocating entry w/ addr %#lx of size %#lx\n",
                    virt_page_addr, new_entry->size());

            local_entry = insert(virt_page_addr, *new_entry);
        }

        assert(local_entry);
    }

    /**
     * At this point the packet carries an up-to-date tlbEntry pointer
     * in its senderState.
     * Next step is to do the paging protection checks.
     */
    DPRINTF(GPUTLB, "Entry found with vaddr %#x,  doing protection checks "
            "while paddr was %#x.\n", local_entry->vaddr,
            local_entry->paddr);

    pagingProtectionChecks(pkt, local_entry, mode);
    int page_size = local_entry->size();
    Addr paddr = local_entry->paddr | (vaddr & (page_size - 1));
    DPRINTF(GPUTLB, "Translated %#x -> %#x.\n", vaddr, paddr);

    // Since this packet will be sent through the cpu side port, it must be
    // converted to a response pkt if it is not one already
    if (pkt->isRequest()) {
        pkt->makeTimingResponse();
    }

    pkt->req->setPaddr(paddr);

    if (local_entry->uncacheable()) {
         pkt->req->setFlags(Request::UNCACHEABLE);
    }

    //send packet back to coalescer
    cpuSidePort[0]->sendTimingResp(pkt);
    //schedule cleanup event
    cleanupQueue.push(virt_page_addr);

    DPRINTF(GPUTLB, "Scheduled %#lx for cleanup\n", virt_page_addr);

    // schedule this only once per cycle.
    // The check is required because we might have multiple translations
    // returning the same cycle
    // this is a maximum priority event and must be on the same cycle
    // as the cleanup event in TLBCoalescer to avoid a race with
    // IssueProbeEvent caused by TLBCoalescer::MemSidePort::recvReqRetry
    if (!cleanupEvent.scheduled())
        schedule(cleanupEvent, curTick());
}

/**
 * Here we take the appropriate actions based on the result of the
 * TLB lookup.
 */
void
GpuTLB::translationReturn(Addr virtPageAddr, tlbOutcome outcome,
                          PacketPtr pkt)
{
    DPRINTF(GPUTLB, "Triggered TLBEvent for addr %#x\n", virtPageAddr);

    assert(translationReturnEvent[virtPageAddr]);
    assert(pkt);

    GpuTranslationState *tmp_sender_state =
        safe_cast<GpuTranslationState*>(pkt->senderState);

    int req_cnt = tmp_sender_state->reqCnt.back();
    bool update_stats = !tmp_sender_state->isPrefetch;


    if (outcome == TLB_HIT) {
        handleTranslationReturn(virtPageAddr, TLB_HIT, pkt);

    } else if (outcome == TLB_MISS) {

        DPRINTF(GPUTLB, "This is a TLB miss\n");
        if (hasMemSidePort) {
            // the one cyle added here represent the delay from when we get
            // the reply back till when we propagate it to the coalescer
            // above.

            /**
             * There is a TLB below. Send the coalesced request.
             * We actually send the very first packet of all the
             * pending packets for this virtual page address.
             */
            tmp_sender_state->deviceId = 1;
            tmp_sender_state->pasId = 0;

            if (!memSidePort[0]->sendTimingReq(pkt)) {
                DPRINTF(GPUTLB, "Failed sending translation request to "
                        "lower level TLB for addr %#x\n", virtPageAddr);

                memSidePort[0]->retries.push_back(pkt);
            } else {
                DPRINTF(GPUTLB, "Sent translation request to lower level "
                        "TLB for addr %#x\n", virtPageAddr);
            }
        } else {
            //this is the last level TLB. Start a page walk
            DPRINTF(GPUTLB, "Last level TLB - start a page walk for "
                    "addr %#x\n", virtPageAddr);

            if (update_stats)
                stats.pageTableCycles -= (req_cnt*curCycle());

            TLBEvent *tlb_event = translationReturnEvent[virtPageAddr];
            assert(tlb_event);
            tlb_event->updateOutcome(PAGE_WALK);
            schedule(tlb_event,
                     curTick() + cyclesToTicks(Cycles(missLatency2)));
        }
    } else if (outcome == PAGE_WALK) {
        if (update_stats)
            stats.pageTableCycles += (req_cnt*curCycle());

        // Need to access the page table and update the TLB
        DPRINTF(GPUTLB, "Doing a page walk for address %#x\n",
                virtPageAddr);

        Addr base = gpuDevice->getVM().getPageTableBase(1);
        Addr vaddr = pkt->req->getVaddr();
        walker->setDevRequestor(gpuDevice->vramRequestorId());

        // Do page table walk
        walker->startTiming(pkt, base, vaddr, BaseMMU::Mode::Read);
    } else if (outcome == MISS_RETURN) {
        /** we add an extra cycle in the return path of the translation
         * requests in between the various TLB levels.
         */
        handleTranslationReturn(virtPageAddr, TLB_MISS, pkt);
    } else {
        panic("Unexpected TLB outcome %d", outcome);
    }
}

void
GpuTLB::TLBEvent::process()
{
    tlb->translationReturn(virtPageAddr, outcome, pkt);
}

const char*
GpuTLB::TLBEvent::description() const
{
    return "trigger translationDoneEvent";
}

void
GpuTLB::TLBEvent::updateOutcome(tlbOutcome _outcome)
{
    outcome = _outcome;
}

Addr
GpuTLB::TLBEvent::getTLBEventVaddr()
{
    return virtPageAddr;
}

/**
 * recvTiming receives a coalesced timing request from a TLBCoalescer
 * and it calls issueTLBLookup()
 * It only rejects the packet if we have exceeded the max
 * outstanding number of requests for the TLB
 */
bool
GpuTLB::CpuSidePort::recvTimingReq(PacketPtr pkt)
{
    bool ret = false;
    [[maybe_unused]] Addr virt_page_addr = roundDown(pkt->req->getVaddr(),
                                                     VegaISA::PageBytes);

    if (tlb->outstandingReqs < tlb->maxCoalescedReqs) {
        assert(!tlb->translationReturnEvent.count(virt_page_addr));
        tlb->issueTLBLookup(pkt);
        // update number of outstanding translation requests
        tlb->outstandingReqs++;
        ret = true;
    } else {
        DPRINTF(GPUTLB, "Reached maxCoalescedReqs number %d\n",
                tlb->outstandingReqs);
        tlb->stats.maxDownstreamReached++;
        ret = false;

    }

    if (tlb->outstandingReqs > tlb->stats.outstandingReqsMax.value())
        tlb->stats.outstandingReqsMax = tlb->outstandingReqs;

    return ret;
}

/**
 * handleFuncTranslationReturn is called on a TLB hit,
 * when a TLB miss returns or when a page fault returns.
 * It updates LRU, inserts the TLB entry on a miss
 * depending on the allocation policy and does the required
 * protection checks. It does NOT create a new packet to
 * update the packet's addr; this is done in hsail-gpu code.
 */
void
GpuTLB::handleFuncTranslationReturn(PacketPtr pkt, tlbOutcome tlb_outcome)
{
    GpuTranslationState *sender_state =
        safe_cast<GpuTranslationState*>(pkt->senderState);

    Mode mode = sender_state->tlbMode;
    Addr vaddr = pkt->req->getVaddr();

    VegaTlbEntry *local_entry, *new_entry;

    if (tlb_outcome == TLB_HIT) {
        DPRINTF(GPUTLB, "Functional Translation Done - TLB hit for addr "
                "%#x\n", vaddr);

        local_entry = safe_cast<VegaTlbEntry *>(sender_state->tlbEntry);
    } else {
        DPRINTF(GPUTLB, "Functional Translation Done - TLB miss for addr "
                "%#x\n", vaddr);

        /**
         * We are returning either from a page walk or from a hit at a
         * lower TLB level. The senderState should be "carrying" a pointer
         * to the correct TLBEntry.
         */
        new_entry = safe_cast<VegaTlbEntry *>(sender_state->tlbEntry);
        assert(new_entry);
        local_entry = new_entry;

        if (allocationPolicy) {
            Addr virt_page_addr = roundDown(vaddr, VegaISA::PageBytes);

            DPRINTF(GPUTLB, "allocating entry w/ addr %#lx\n",
                    virt_page_addr);

            local_entry = insert(virt_page_addr, *new_entry);
        }

        assert(local_entry);
    }

    DPRINTF(GPUTLB, "Entry found with vaddr %#x, doing protection checks "
            "while paddr was %#x.\n", local_entry->vaddr,
            local_entry->paddr);

    /**
     * Do paging checks if it's a normal functional access.  If it's for a
     * prefetch, then sometimes you can try to prefetch something that
     * won't pass protection. We don't actually want to fault becuase there
     * is no demand access to deem this a violation.  Just put it in the
     * TLB and it will fault if indeed a future demand access touches it in
     * violation.
     *
     * This feature could be used to explore security issues around
     * speculative memory accesses.
     */
    if (!sender_state->isPrefetch && sender_state->tlbEntry)
        pagingProtectionChecks(pkt, local_entry, mode);

    int page_size = local_entry->size();
    Addr paddr = local_entry->paddr | (vaddr & (page_size - 1));
    DPRINTF(GPUTLB, "Translated %#x -> %#x.\n", vaddr, paddr);

    pkt->req->setPaddr(paddr);

    if (local_entry->uncacheable())
         pkt->req->setFlags(Request::UNCACHEABLE);
}

// This is used for atomic translations. Need to
// make it all happen during the same cycle.
void
GpuTLB::CpuSidePort::recvFunctional(PacketPtr pkt)
{
    GpuTranslationState *sender_state =
        safe_cast<GpuTranslationState*>(pkt->senderState);

    bool update_stats = !sender_state->isPrefetch;

    Addr virt_page_addr = roundDown(pkt->req->getVaddr(),
                                    VegaISA::PageBytes);

    // do the TLB lookup without updating the stats
    bool success = tlb->tlbLookup(pkt->req, update_stats);
    tlbOutcome tlb_outcome = success ? TLB_HIT : TLB_MISS;

    // functional mode means no coalescing
    // global metrics are the same as the local metrics
    if (update_stats) {
        tlb->stats.globalNumTLBAccesses++;

        if (success) {
            sender_state->hitLevel = sender_state->reqCnt.size();
            tlb->stats.globalNumTLBHits++;
        } else {
            tlb->stats.globalNumTLBMisses++;
        }
    }

    if (!success) {
        if (tlb->hasMemSidePort) {
            // there is a TLB below -> propagate down the TLB hierarchy
            tlb->memSidePort[0]->sendFunctional(pkt);
            // If no valid translation from a prefetch, then just return
            if (sender_state->isPrefetch && !pkt->req->hasPaddr())
                return;
        } else {
            // Need to access the page table and update the TLB
            DPRINTF(GPUTLB, "Doing a page walk for address %#x\n",
                    virt_page_addr);

            Addr vaddr = pkt->req->getVaddr();
            [[maybe_unused]] Addr alignedVaddr =
                tlb->pageAlign(virt_page_addr);
            assert(alignedVaddr == virt_page_addr);

            unsigned logBytes;
            PageTableEntry pte;

            // Initialize walker state for VMID
            Addr base = tlb->gpuDevice->getVM().getPageTableBase(1);
            tlb->walker->setDevRequestor(tlb->gpuDevice->vramRequestorId());

            // Do page table walk
            Fault fault = tlb->walker->startFunctional(base, vaddr, pte,
                                                       logBytes,
                                                       BaseMMU::Mode::Read);
            if (fault != NoFault) {
                fatal("Translation fault in TLB at %d!", __LINE__);
            }

            // PPN is already shifted by fragment so we only shift by native
            // page size. Fragment is still used via logBytes to select lower
            // bits from vaddr.
            Addr page_addr = pte.ppn << PageShift;
            Addr paddr = page_addr + (vaddr & mask(logBytes));
            Addr alignedPaddr = tlb->pageAlign(paddr);
            pkt->req->setPaddr(paddr);
            pkt->req->setSystemReq(pte.s);

            if (!sender_state->isPrefetch) {
                assert(paddr);

                DPRINTF(GPUTLB, "Mapping %#x to %#x\n", vaddr, paddr);

                sender_state->tlbEntry =
                    new VegaTlbEntry(1 /* VMID */, virt_page_addr,
                                 alignedPaddr, logBytes, pte);
            } else {
                // If this was a prefetch, then do the normal thing if it
                // was a successful translation.  Otherwise, send an empty
                // TLB entry back so that it can be figured out as empty
                // and handled accordingly.
                if (paddr) {
                    DPRINTF(GPUTLB, "Mapping %#x to %#x\n", vaddr, paddr);

                    sender_state->tlbEntry =
                        new VegaTlbEntry(1 /* VMID */, virt_page_addr,
                                     alignedPaddr, logBytes, pte);
                } else {
                    DPRINTF(GPUPrefetch, "Prefetch failed %#x\n", vaddr);

                    sender_state->tlbEntry = nullptr;

                    return;
                }
            }
        }
    } else {
        VegaTlbEntry *entry = tlb->lookup(virt_page_addr, update_stats);
        assert(entry);

        if (sender_state->isPrefetch) {
            DPRINTF(GPUPrefetch, "Functional Hit for vaddr %#x\n",
                    entry->vaddr);
        }

        sender_state->tlbEntry = new VegaTlbEntry(1 /* VMID */, entry->vaddr,
                                                 entry->paddr, entry->logBytes,
                                                 entry->pte);
    }

    // This is the function that would populate pkt->req with the paddr of
    // the translation. But if no translation happens (i.e Prefetch fails)
    // then the early returns in the above code wiill keep this function
    // from executing.
    tlb->handleFuncTranslationReturn(pkt, tlb_outcome);
}

void
GpuTLB::CpuSidePort::recvReqRetry()
{
    // The CPUSidePort never sends anything but replies. No retries
    // expected.
    panic("recvReqRetry called");
}

AddrRangeList
GpuTLB::CpuSidePort::getAddrRanges() const
{
    // currently not checked by the requestor
    AddrRangeList ranges;

    return ranges;
}

/**
 * MemSidePort receives the packet back.
 * We need to call the handleTranslationReturn
 * and propagate up the hierarchy.
 */
bool
GpuTLB::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    Addr virt_page_addr = roundDown(pkt->req->getVaddr(),
                                    VegaISA::PageBytes);

    DPRINTF(GPUTLB, "MemSidePort recvTiming for virt_page_addr %#x\n",
            virt_page_addr);

    TLBEvent *tlb_event = tlb->translationReturnEvent[virt_page_addr];
    assert(tlb_event);
    assert(virt_page_addr == tlb_event->getTLBEventVaddr());

    tlb_event->updateOutcome(MISS_RETURN);
    tlb->schedule(tlb_event, curTick()+tlb->clockPeriod());

    return true;
}

void
GpuTLB::MemSidePort::recvReqRetry()
{
    // No retries should reach the TLB. The retries
    // should only reach the TLBCoalescer.
    panic("recvReqRetry called");
}

void
GpuTLB::cleanup()
{
    while (!cleanupQueue.empty()) {
        Addr cleanup_addr = cleanupQueue.front();
        cleanupQueue.pop();

        DPRINTF(GPUTLB, "Deleting return event for %#lx\n", cleanup_addr);

        // delete TLBEvent
        TLBEvent * old_tlb_event = translationReturnEvent[cleanup_addr];
        delete old_tlb_event;
        translationReturnEvent.erase(cleanup_addr);

        // update number of outstanding requests
        outstandingReqs--;
    }

    /** the higher level coalescer should retry if it has
     * any pending requests.
     */
    for (int i = 0; i < cpuSidePort.size(); ++i) {
        cpuSidePort[i]->sendRetryReq();
    }
}

GpuTLB::VegaTLBStats::VegaTLBStats(statistics::Group *parent)
    : statistics::Group(parent),
      ADD_STAT(maxDownstreamReached, "Number of refused translation requests"),
      ADD_STAT(outstandingReqsMax, "Maximum count in coalesced request queue"),
      ADD_STAT(localNumTLBAccesses, "Number of TLB accesses"),
      ADD_STAT(localNumTLBHits, "Number of TLB hits"),
      ADD_STAT(localNumTLBMisses, "Number of TLB misses"),
      ADD_STAT(localTLBMissRate, "TLB miss rate"),
      ADD_STAT(globalNumTLBAccesses, "Number of TLB accesses"),
      ADD_STAT(globalNumTLBHits, "Number of TLB hits"),
      ADD_STAT(globalNumTLBMisses, "Number of TLB misses"),
      ADD_STAT(globalTLBMissRate, "TLB miss rate"),
      ADD_STAT(accessCycles, "Cycles spent accessing this TLB level"),
      ADD_STAT(pageTableCycles, "Cycles spent accessing the page table"),
      ADD_STAT(localCycles, "Number of cycles spent in queue for all "
               "incoming reqs"),
      ADD_STAT(localLatency, "Avg. latency over incoming coalesced reqs")
{
    localTLBMissRate = 100 * localNumTLBMisses / localNumTLBAccesses;
    globalTLBMissRate = 100 * globalNumTLBMisses / globalNumTLBAccesses;

    localLatency = localCycles / localNumTLBAccesses;
}

} // namespace VegaISA
} // namespace gem5
