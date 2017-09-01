/*
 * Copyright (c) 2013-2014 ARM Limited
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
 *
 * Authors: Andrew Bardsley
 */

#include "cpu/minor/lsq.hh"

#include <iomanip>
#include <sstream>

#include "arch/locked_mem.hh"
#include "arch/mmapped_ipr.hh"
#include "cpu/minor/cpu.hh"
#include "cpu/minor/exec_context.hh"
#include "cpu/minor/execute.hh"
#include "cpu/minor/pipeline.hh"
#include "debug/Activity.hh"
#include "debug/MinorMem.hh"

namespace Minor
{

/** Returns the offset of addr into an aligned a block of size block_size */
static Addr
addrBlockOffset(Addr addr, unsigned int block_size)
{
    return addr & (block_size - 1);
}

/** Returns true if the given [addr .. addr+size-1] transfer needs to be
 *  fragmented across a block size of block_size */
static bool
transferNeedsBurst(Addr addr, unsigned int size, unsigned int block_size)
{
    return (addrBlockOffset(addr, block_size) + size) > block_size;
}

LSQ::LSQRequest::LSQRequest(LSQ &port_, MinorDynInstPtr inst_, bool isLoad_,
    PacketDataPtr data_, uint64_t *res_) :
    SenderState(),
    port(port_),
    inst(inst_),
    isLoad(isLoad_),
    data(data_),
    packet(NULL),
    request(),
    fault(NoFault),
    res(res_),
    skipped(false),
    issuedToMemory(false),
    state(NotIssued)
{ }

LSQ::AddrRangeCoverage
LSQ::LSQRequest::containsAddrRangeOf(
    Addr req1_addr, unsigned int req1_size,
    Addr req2_addr, unsigned int req2_size)
{
    /* 'end' here means the address of the byte just past the request
     *  blocks */
    Addr req2_end_addr = req2_addr + req2_size;
    Addr req1_end_addr = req1_addr + req1_size;

    AddrRangeCoverage ret;

    if (req1_addr >= req2_end_addr || req1_end_addr <= req2_addr)
        ret = NoAddrRangeCoverage;
    else if (req1_addr <= req2_addr && req1_end_addr >= req2_end_addr)
        ret = FullAddrRangeCoverage;
    else
        ret = PartialAddrRangeCoverage;

    return ret;
}

LSQ::AddrRangeCoverage
LSQ::LSQRequest::containsAddrRangeOf(LSQRequestPtr other_request)
{
    return containsAddrRangeOf(request.getPaddr(), request.getSize(),
        other_request->request.getPaddr(), other_request->request.getSize());
}

bool
LSQ::LSQRequest::isBarrier()
{
    return inst->isInst() && inst->staticInst->isMemBarrier();
}

bool
LSQ::LSQRequest::needsToBeSentToStoreBuffer()
{
    return state == StoreToStoreBuffer;
}

void
LSQ::LSQRequest::setState(LSQRequestState new_state)
{
    DPRINTFS(MinorMem, (&port), "Setting state from %d to %d for request:"
        " %s\n", state, new_state, *inst);
    state = new_state;
}

bool
LSQ::LSQRequest::isComplete() const
{
    /* @todo, There is currently only one 'completed' state.  This
     *  may not be a good choice */
    return state == Complete;
}

void
LSQ::LSQRequest::reportData(std::ostream &os) const
{
    os << (isLoad ? 'R' : 'W') << ';';
    inst->reportData(os);
    os << ';' << state;
}

std::ostream &
operator <<(std::ostream &os, LSQ::AddrRangeCoverage coverage)
{
    switch (coverage) {
      case LSQ::PartialAddrRangeCoverage:
        os << "PartialAddrRangeCoverage";
        break;
      case LSQ::FullAddrRangeCoverage:
        os << "FullAddrRangeCoverage";
        break;
      case LSQ::NoAddrRangeCoverage:
        os << "NoAddrRangeCoverage";
        break;
      default:
        os << "AddrRangeCoverage-" << static_cast<int>(coverage);
        break;
    }
    return os;
}

std::ostream &
operator <<(std::ostream &os, LSQ::LSQRequest::LSQRequestState state)
{
    switch (state) {
      case LSQ::LSQRequest::NotIssued:
        os << "NotIssued";
        break;
      case LSQ::LSQRequest::InTranslation:
        os << "InTranslation";
        break;
      case LSQ::LSQRequest::Translated:
        os << "Translated";
        break;
      case LSQ::LSQRequest::Failed:
        os << "Failed";
        break;
      case LSQ::LSQRequest::RequestIssuing:
        os << "RequestIssuing";
        break;
      case LSQ::LSQRequest::StoreToStoreBuffer:
        os << "StoreToStoreBuffer";
        break;
      case LSQ::LSQRequest::StoreInStoreBuffer:
        os << "StoreInStoreBuffer";
        break;
      case LSQ::LSQRequest::StoreBufferIssuing:
        os << "StoreBufferIssuing";
        break;
      case LSQ::LSQRequest::RequestNeedsRetry:
        os << "RequestNeedsRetry";
        break;
      case LSQ::LSQRequest::StoreBufferNeedsRetry:
        os << "StoreBufferNeedsRetry";
        break;
      case LSQ::LSQRequest::Complete:
        os << "Complete";
        break;
      default:
        os << "LSQRequestState-" << static_cast<int>(state);
        break;
    }
    return os;
}

void
LSQ::clearMemBarrier(MinorDynInstPtr inst)
{
    bool is_last_barrier =
        inst->id.execSeqNum >= lastMemBarrier[inst->id.threadId];

    DPRINTF(MinorMem, "Moving %s barrier out of store buffer inst: %s\n",
        (is_last_barrier ? "last" : "a"), *inst);

    if (is_last_barrier)
        lastMemBarrier[inst->id.threadId] = 0;
}

void
LSQ::SingleDataRequest::finish(const Fault &fault_, RequestPtr request_,
                               ThreadContext *tc, BaseTLB::Mode mode)
{
    fault = fault_;

    port.numAccessesInDTLB--;

    DPRINTFS(MinorMem, (&port), "Received translation response for"
        " request: %s\n", *inst);

    makePacket();

    setState(Translated);
    port.tryToSendToTransfers(this);

    /* Let's try and wake up the processor for the next cycle */
    port.cpu.wakeupOnEvent(Pipeline::ExecuteStageId);
}

void
LSQ::SingleDataRequest::startAddrTranslation()
{
    ThreadContext *thread = port.cpu.getContext(
        inst->id.threadId);

    port.numAccessesInDTLB++;

    setState(LSQ::LSQRequest::InTranslation);

    DPRINTFS(MinorMem, (&port), "Submitting DTLB request\n");
    /* Submit the translation request.  The response will come through
     *  finish/markDelayed on the LSQRequest as it bears the Translation
     *  interface */
    thread->getDTBPtr()->translateTiming(
        &request, thread, this, (isLoad ? BaseTLB::Read : BaseTLB::Write));
}

void
LSQ::SingleDataRequest::retireResponse(PacketPtr packet_)
{
    DPRINTFS(MinorMem, (&port), "Retiring packet\n");
    packet = packet_;
    packetInFlight = false;
    setState(Complete);
}

void
LSQ::SplitDataRequest::finish(const Fault &fault_, RequestPtr request_,
                              ThreadContext *tc, BaseTLB::Mode mode)
{
    fault = fault_;

    port.numAccessesInDTLB--;

    unsigned int M5_VAR_USED expected_fragment_index =
        numTranslatedFragments;

    numInTranslationFragments--;
    numTranslatedFragments++;

    DPRINTFS(MinorMem, (&port), "Received translation response for fragment"
        " %d of request: %s\n", expected_fragment_index, *inst);

    assert(request_ == fragmentRequests[expected_fragment_index]);

    /* Wake up next cycle to get things going again in case the
     *  tryToSendToTransfers does take */
    port.cpu.wakeupOnEvent(Pipeline::ExecuteStageId);

    if (fault != NoFault) {
        /* tryToSendToTransfers will handle the fault */

        DPRINTFS(MinorMem, (&port), "Faulting translation for fragment:"
            " %d of request: %s\n",
            expected_fragment_index, *inst);

        setState(Translated);
        port.tryToSendToTransfers(this);
    } else if (numTranslatedFragments == numFragments) {
        makeFragmentPackets();

        setState(Translated);
        port.tryToSendToTransfers(this);
    } else {
        /* Avoid calling translateTiming from within ::finish */
        assert(!translationEvent.scheduled());
        port.cpu.schedule(translationEvent, curTick());
    }
}

LSQ::SplitDataRequest::SplitDataRequest(LSQ &port_, MinorDynInstPtr inst_,
    bool isLoad_, PacketDataPtr data_, uint64_t *res_) :
    LSQRequest(port_, inst_, isLoad_, data_, res_),
    translationEvent([this]{ sendNextFragmentToTranslation(); },
                     "translationEvent"),
    numFragments(0),
    numInTranslationFragments(0),
    numTranslatedFragments(0),
    numIssuedFragments(0),
    numRetiredFragments(0),
    fragmentRequests(),
    fragmentPackets()
{
    /* Don't know how many elements are needed until the request is
     *  populated by the caller. */
}

LSQ::SplitDataRequest::~SplitDataRequest()
{
    for (auto i = fragmentRequests.begin();
        i != fragmentRequests.end(); i++)
    {
        delete *i;
    }

    for (auto i = fragmentPackets.begin();
         i != fragmentPackets.end(); i++)
    {
        delete *i;
    }
}

void
LSQ::SplitDataRequest::makeFragmentRequests()
{
    Addr base_addr = request.getVaddr();
    unsigned int whole_size = request.getSize();
    unsigned int line_width = port.lineWidth;

    unsigned int fragment_size;
    Addr fragment_addr;

    /* Assume that this transfer is across potentially many block snap
     * boundaries:
     *
     * |      _|________|________|________|___     |
     * |     |0| 1      | 2      | 3      | 4 |    |
     * |     |_|________|________|________|___|    |
     * |       |        |        |        |        |
     *
     *  The first transfer (0) can be up to lineWidth in size.
     *  All the middle transfers (1-3) are lineWidth in size
     *  The last transfer (4) can be from zero to lineWidth - 1 in size
     */
    unsigned int first_fragment_offset =
        addrBlockOffset(base_addr, line_width);
    unsigned int last_fragment_size =
        addrBlockOffset(base_addr + whole_size, line_width);
    unsigned int first_fragment_size =
        line_width - first_fragment_offset;

    unsigned int middle_fragments_total_size =
        whole_size - (first_fragment_size + last_fragment_size);

    assert(addrBlockOffset(middle_fragments_total_size, line_width) == 0);

    unsigned int middle_fragment_count =
        middle_fragments_total_size / line_width;

    numFragments = 1 /* first */ + middle_fragment_count +
        (last_fragment_size == 0 ? 0 : 1);

    DPRINTFS(MinorMem, (&port), "Dividing transfer into %d fragmentRequests."
        " First fragment size: %d Last fragment size: %d\n",
        numFragments, first_fragment_size,
        (last_fragment_size == 0 ? line_width : last_fragment_size));

    assert(((middle_fragment_count * line_width) +
        first_fragment_size + last_fragment_size) == whole_size);

    fragment_addr = base_addr;
    fragment_size = first_fragment_size;

    /* Just past the last address in the request */
    Addr end_addr = base_addr + whole_size;

    for (unsigned int fragment_index = 0; fragment_index < numFragments;
         fragment_index++)
    {
        bool M5_VAR_USED is_last_fragment = false;

        if (fragment_addr == base_addr) {
            /* First fragment */
            fragment_size = first_fragment_size;
        } else {
            if ((fragment_addr + line_width) > end_addr) {
                /* Adjust size of last fragment */
                fragment_size = end_addr - fragment_addr;
                is_last_fragment = true;
            } else {
                /* Middle fragments */
                fragment_size = line_width;
            }
        }

        Request *fragment = new Request();

        fragment->setContext(request.contextId());
        fragment->setVirt(0 /* asid */,
            fragment_addr, fragment_size, request.getFlags(),
            request.masterId(),
            request.getPC());

        DPRINTFS(MinorMem, (&port), "Generating fragment addr: 0x%x size: %d"
            " (whole request addr: 0x%x size: %d) %s\n",
            fragment_addr, fragment_size, base_addr, whole_size,
            (is_last_fragment ? "last fragment" : ""));

        fragment_addr += fragment_size;

        fragmentRequests.push_back(fragment);
    }
}

void
LSQ::SplitDataRequest::makeFragmentPackets()
{
    Addr base_addr = request.getVaddr();

    DPRINTFS(MinorMem, (&port), "Making packets for request: %s\n", *inst);

    for (unsigned int fragment_index = 0; fragment_index < numFragments;
         fragment_index++)
    {
        Request *fragment = fragmentRequests[fragment_index];

        DPRINTFS(MinorMem, (&port), "Making packet %d for request: %s"
            " (%d, 0x%x)\n",
            fragment_index, *inst,
            (fragment->hasPaddr() ? "has paddr" : "no paddr"),
            (fragment->hasPaddr() ? fragment->getPaddr() : 0));

        Addr fragment_addr = fragment->getVaddr();
        unsigned int fragment_size = fragment->getSize();

        uint8_t *request_data = NULL;

        if (!isLoad) {
            /* Split data for Packets.  Will become the property of the
             *  outgoing Packets */
            request_data = new uint8_t[fragment_size];
            std::memcpy(request_data, data + (fragment_addr - base_addr),
                fragment_size);
        }

        assert(fragment->hasPaddr());

        PacketPtr fragment_packet =
            makePacketForRequest(*fragment, isLoad, this, request_data);

        fragmentPackets.push_back(fragment_packet);
        /* Accumulate flags in parent request */
        request.setFlags(fragment->getFlags());
    }

    /* Might as well make the overall/response packet here */
    /* Get the physical address for the whole request/packet from the first
     *  fragment */
    request.setPaddr(fragmentRequests[0]->getPaddr());
    makePacket();
}

void
LSQ::SplitDataRequest::startAddrTranslation()
{
    setState(LSQ::LSQRequest::InTranslation);

    makeFragmentRequests();

    numInTranslationFragments = 0;
    numTranslatedFragments = 0;

    /* @todo, just do these in sequence for now with
     * a loop of:
     * do {
     *  sendNextFragmentToTranslation ; translateTiming ; finish
     * } while (numTranslatedFragments != numFragments);
     */

    /* Do first translation */
    sendNextFragmentToTranslation();
}

PacketPtr
LSQ::SplitDataRequest::getHeadPacket()
{
    assert(numIssuedFragments < numFragments);

    return fragmentPackets[numIssuedFragments];
}

void
LSQ::SplitDataRequest::stepToNextPacket()
{
    assert(numIssuedFragments < numFragments);

    numIssuedFragments++;
}

void
LSQ::SplitDataRequest::retireResponse(PacketPtr response)
{
    assert(numRetiredFragments < numFragments);

    DPRINTFS(MinorMem, (&port), "Retiring fragment addr: 0x%x size: %d"
        " offset: 0x%x (retired fragment num: %d) %s\n",
        response->req->getVaddr(), response->req->getSize(),
        request.getVaddr() - response->req->getVaddr(),
        numRetiredFragments,
        (fault == NoFault ? "" : fault->name()));

    numRetiredFragments++;

    if (skipped) {
        /* Skip because we already knew the request had faulted or been
         *  skipped */
        DPRINTFS(MinorMem, (&port), "Skipping this fragment\n");
    } else if (response->isError()) {
        /* Mark up the error and leave to execute to handle it */
        DPRINTFS(MinorMem, (&port), "Fragment has an error, skipping\n");
        setSkipped();
        packet->copyError(response);
    } else {
        if (isLoad) {
            if (!data) {
                /* For a split transfer, a Packet must be constructed
                 *  to contain all returning data.  This is that packet's
                 *  data */
                data = new uint8_t[request.getSize()];
            }

            /* Populate the portion of the overall response data represented
             *  by the response fragment */
            std::memcpy(
                data + (response->req->getVaddr() - request.getVaddr()),
                response->getConstPtr<uint8_t>(),
                response->req->getSize());
        }
    }

    /* Complete early if we're skipping are no more in-flight accesses */
    if (skipped && !hasPacketsInMemSystem()) {
        DPRINTFS(MinorMem, (&port), "Completed skipped burst\n");
        setState(Complete);
        if (packet->needsResponse())
            packet->makeResponse();
    }

    if (numRetiredFragments == numFragments)
        setState(Complete);

    if (!skipped && isComplete()) {
        DPRINTFS(MinorMem, (&port), "Completed burst %d\n", packet != NULL);

        DPRINTFS(MinorMem, (&port), "Retired packet isRead: %d isWrite: %d"
             " needsResponse: %d packetSize: %s requestSize: %s responseSize:"
             " %s\n", packet->isRead(), packet->isWrite(),
             packet->needsResponse(), packet->getSize(), request.getSize(),
             response->getSize());

        /* A request can become complete by several paths, this is a sanity
         *  check to make sure the packet's data is created */
        if (!data) {
            data = new uint8_t[request.getSize()];
        }

        if (isLoad) {
            DPRINTFS(MinorMem, (&port), "Copying read data\n");
            std::memcpy(packet->getPtr<uint8_t>(), data, request.getSize());
        }
        packet->makeResponse();
    }

    /* Packets are all deallocated together in ~SplitLSQRequest */
}

void
LSQ::SplitDataRequest::sendNextFragmentToTranslation()
{
    unsigned int fragment_index = numTranslatedFragments;

    ThreadContext *thread = port.cpu.getContext(
        inst->id.threadId);

    DPRINTFS(MinorMem, (&port), "Submitting DTLB request for fragment: %d\n",
        fragment_index);

    port.numAccessesInDTLB++;
    numInTranslationFragments++;

    thread->getDTBPtr()->translateTiming(
        fragmentRequests[fragment_index], thread, this, (isLoad ?
        BaseTLB::Read : BaseTLB::Write));
}

bool
LSQ::StoreBuffer::canInsert() const
{
    /* @todo, support store amalgamation */
    return slots.size() < numSlots;
}

void
LSQ::StoreBuffer::deleteRequest(LSQRequestPtr request)
{
    auto found = std::find(slots.begin(), slots.end(), request);

    if (found != slots.end()) {
        DPRINTF(MinorMem, "Deleting request: %s %s %s from StoreBuffer\n",
            request, *found, *(request->inst));
        slots.erase(found);

        delete request;
    }
}

void
LSQ::StoreBuffer::insert(LSQRequestPtr request)
{
    if (!canInsert()) {
        warn("%s: store buffer insertion without space to insert from"
            " inst: %s\n", name(), *(request->inst));
    }

    DPRINTF(MinorMem, "Pushing store: %s into store buffer\n", request);

    numUnissuedAccesses++;

    if (request->state != LSQRequest::Complete)
        request->setState(LSQRequest::StoreInStoreBuffer);

    slots.push_back(request);

    /* Let's try and wake up the processor for the next cycle to step
     *  the store buffer */
    lsq.cpu.wakeupOnEvent(Pipeline::ExecuteStageId);
}

LSQ::AddrRangeCoverage
LSQ::StoreBuffer::canForwardDataToLoad(LSQRequestPtr request,
    unsigned int &found_slot)
{
    unsigned int slot_index = slots.size() - 1;
    auto i = slots.rbegin();
    AddrRangeCoverage ret = NoAddrRangeCoverage;

    /* Traverse the store buffer in reverse order (most to least recent)
     *  and try to find a slot whose address range overlaps this request */
    while (ret == NoAddrRangeCoverage && i != slots.rend()) {
        LSQRequestPtr slot = *i;

        if (slot->packet &&
            slot->inst->id.threadId == request->inst->id.threadId) {
            AddrRangeCoverage coverage = slot->containsAddrRangeOf(request);

            if (coverage != NoAddrRangeCoverage) {
                DPRINTF(MinorMem, "Forwarding: slot: %d result: %s thisAddr:"
                    " 0x%x thisSize: %d slotAddr: 0x%x slotSize: %d\n",
                    slot_index, coverage,
                    request->request.getPaddr(), request->request.getSize(),
                    slot->request.getPaddr(), slot->request.getSize());

                found_slot = slot_index;
                ret = coverage;
            }
        }

        i++;
        slot_index--;
    }

    return ret;
}

/** Fill the given packet with appropriate date from slot slot_number */
void
LSQ::StoreBuffer::forwardStoreData(LSQRequestPtr load,
    unsigned int slot_number)
{
    assert(slot_number < slots.size());
    assert(load->packet);
    assert(load->isLoad);

    LSQRequestPtr store = slots[slot_number];

    assert(store->packet);
    assert(store->containsAddrRangeOf(load) == FullAddrRangeCoverage);

    Addr load_addr = load->request.getPaddr();
    Addr store_addr = store->request.getPaddr();
    Addr addr_offset = load_addr - store_addr;

    unsigned int load_size = load->request.getSize();

    DPRINTF(MinorMem, "Forwarding %d bytes for addr: 0x%x from store buffer"
        " slot: %d addr: 0x%x addressOffset: 0x%x\n",
        load_size, load_addr, slot_number,
        store_addr, addr_offset);

    void *load_packet_data = load->packet->getPtr<void>();
    void *store_packet_data = store->packet->getPtr<uint8_t>() + addr_offset;

    std::memcpy(load_packet_data, store_packet_data, load_size);
}

void
LSQ::StoreBuffer::countIssuedStore(LSQRequestPtr request)
{
    /* Barriers are accounted for as they are cleared from
     *  the queue, not after their transfers are complete */
    if (!request->isBarrier())
        numUnissuedAccesses--;
}

void
LSQ::StoreBuffer::step()
{
    DPRINTF(MinorMem, "StoreBuffer step numUnissuedAccesses: %d\n",
        numUnissuedAccesses);

    if (numUnissuedAccesses != 0 && lsq.state == LSQ::MemoryRunning) {
        /* Clear all the leading barriers */
        while (!slots.empty() &&
            slots.front()->isComplete() && slots.front()->isBarrier())
        {
            LSQRequestPtr barrier = slots.front();

            DPRINTF(MinorMem, "Clearing barrier for inst: %s\n",
                *(barrier->inst));

            numUnissuedAccesses--;
            lsq.clearMemBarrier(barrier->inst);
            slots.pop_front();

            delete barrier;
        }

        auto i = slots.begin();
        bool issued = true;
        unsigned int issue_count = 0;

        /* Skip trying if the memory system is busy */
        if (lsq.state == LSQ::MemoryNeedsRetry)
            issued = false;

        /* Try to issue all stores in order starting from the head
         *  of the queue.  Responses are allowed to be retired
         *  out of order */
        while (issued &&
            issue_count < storeLimitPerCycle &&
            lsq.canSendToMemorySystem() &&
            i != slots.end())
        {
            LSQRequestPtr request = *i;

            DPRINTF(MinorMem, "Considering request: %s, sentAllPackets: %d"
                " state: %s\n",
                *(request->inst), request->sentAllPackets(),
                request->state);

            if (request->isBarrier() && request->isComplete()) {
                /* Give up at barriers */
                issued = false;
            } else if (!(request->state == LSQRequest::StoreBufferIssuing &&
                request->sentAllPackets()))
            {
                DPRINTF(MinorMem, "Trying to send request: %s to memory"
                    " system\n", *(request->inst));

                if (lsq.tryToSend(request)) {
                    countIssuedStore(request);
                    issue_count++;
                } else {
                    /* Don't step on to the next store buffer entry if this
                     *  one hasn't issued all its packets as the store
                     *  buffer must still enforce ordering */
                    issued = false;
                }
            }
            i++;
        }
    }
}

void
LSQ::completeMemBarrierInst(MinorDynInstPtr inst,
    bool committed)
{
    if (committed) {
        /* Not already sent to the store buffer as a store request? */
        if (!inst->inStoreBuffer) {
            /* Insert an entry into the store buffer to tick off barriers
             *  until there are none in flight */
            storeBuffer.insert(new BarrierDataRequest(*this, inst));
        }
    } else {
        /* Clear the barrier anyway if it wasn't actually committed */
        clearMemBarrier(inst);
    }
}

void
LSQ::StoreBuffer::minorTrace() const
{
    unsigned int size = slots.size();
    unsigned int i = 0;
    std::ostringstream os;

    while (i < size) {
        LSQRequestPtr request = slots[i];

        request->reportData(os);

        i++;
        if (i < numSlots)
            os << ',';
    }

    while (i < numSlots) {
        os << '-';

        i++;
        if (i < numSlots)
            os << ',';
    }

    MINORTRACE("addr=%s num_unissued_stores=%d\n", os.str(),
        numUnissuedAccesses);
}

void
LSQ::tryToSendToTransfers(LSQRequestPtr request)
{
    if (state == MemoryNeedsRetry) {
        DPRINTF(MinorMem, "Request needs retry, not issuing to"
            " memory until retry arrives\n");
        return;
    }

    if (request->state == LSQRequest::InTranslation) {
        DPRINTF(MinorMem, "Request still in translation, not issuing to"
            " memory\n");
        return;
    }

    assert(request->state == LSQRequest::Translated ||
        request->state == LSQRequest::RequestIssuing ||
        request->state == LSQRequest::Failed ||
        request->state == LSQRequest::Complete);

    if (requests.empty() || requests.front() != request) {
        DPRINTF(MinorMem, "Request not at front of requests queue, can't"
            " issue to memory\n");
        return;
    }

    if (transfers.unreservedRemainingSpace() == 0) {
        DPRINTF(MinorMem, "No space to insert request into transfers"
            " queue\n");
        return;
    }

    if (request->isComplete() || request->state == LSQRequest::Failed) {
        DPRINTF(MinorMem, "Passing a %s transfer on to transfers"
            " queue\n", (request->isComplete() ? "completed" : "failed"));
        request->setState(LSQRequest::Complete);
        request->setSkipped();
        moveFromRequestsToTransfers(request);
        return;
    }

    if (!execute.instIsRightStream(request->inst)) {
        /* Wrong stream, try to abort the transfer but only do so if
         *  there are no packets in flight */
        if (request->hasPacketsInMemSystem()) {
            DPRINTF(MinorMem, "Request's inst. is from the wrong stream,"
                " waiting for responses before aborting request\n");
        } else {
            DPRINTF(MinorMem, "Request's inst. is from the wrong stream,"
                " aborting request\n");
            request->setState(LSQRequest::Complete);
            request->setSkipped();
            moveFromRequestsToTransfers(request);
        }
        return;
    }

    if (request->fault != NoFault) {
        if (request->inst->staticInst->isPrefetch()) {
            DPRINTF(MinorMem, "Not signalling fault for faulting prefetch\n");
        }
        DPRINTF(MinorMem, "Moving faulting request into the transfers"
            " queue\n");
        request->setState(LSQRequest::Complete);
        request->setSkipped();
        moveFromRequestsToTransfers(request);
        return;
    }

    bool is_load = request->isLoad;
    bool is_llsc = request->request.isLLSC();
    bool is_swap = request->request.isSwap();
    bool bufferable = !(request->request.isStrictlyOrdered() ||
        is_llsc || is_swap);

    if (is_load) {
        if (numStoresInTransfers != 0) {
            DPRINTF(MinorMem, "Load request with stores still in transfers"
                " queue, stalling\n");
            return;
        }
    } else {
        /* Store.  Can it be sent to the store buffer? */
        if (bufferable && !request->request.isMmappedIpr()) {
            request->setState(LSQRequest::StoreToStoreBuffer);
            moveFromRequestsToTransfers(request);
            DPRINTF(MinorMem, "Moving store into transfers queue\n");
            return;
        }
    }

    /* Check if this is the head instruction (and so must be executable as
     *  its stream sequence number was checked above) for loads which must
     *  not be speculatively issued and stores which must be issued here */
    if (!bufferable) {
        if (!execute.instIsHeadInst(request->inst)) {
            DPRINTF(MinorMem, "Memory access not the head inst., can't be"
                " sure it can be performed, not issuing\n");
            return;
        }

        unsigned int forwarding_slot = 0;

        if (storeBuffer.canForwardDataToLoad(request, forwarding_slot) !=
            NoAddrRangeCoverage)
        {
            DPRINTF(MinorMem, "Memory access can receive forwarded data"
                " from the store buffer, need to wait for store buffer to"
                " drain\n");
            return;
        }
    }

    /* True: submit this packet to the transfers queue to be sent to the
     * memory system.
     * False: skip the memory and push a packet for this request onto
     * requests */
    bool do_access = true;

    if (!is_llsc) {
        /* Check for match in the store buffer */
        if (is_load) {
            unsigned int forwarding_slot = 0;
            AddrRangeCoverage forwarding_result =
                storeBuffer.canForwardDataToLoad(request,
                forwarding_slot);

            switch (forwarding_result) {
              case FullAddrRangeCoverage:
                /* Forward data from the store buffer into this request and
                 *  repurpose this request's packet into a response packet */
                storeBuffer.forwardStoreData(request, forwarding_slot);
                request->packet->makeResponse();

                /* Just move between queues, no access */
                do_access = false;
                break;
              case PartialAddrRangeCoverage:
                DPRINTF(MinorMem, "Load partly satisfied by store buffer"
                    " data. Must wait for the store to complete\n");
                return;
                break;
              case NoAddrRangeCoverage:
                DPRINTF(MinorMem, "No forwardable data from store buffer\n");
                /* Fall through to try access */
                break;
            }
        }
    } else {
        if (!canSendToMemorySystem()) {
            DPRINTF(MinorMem, "Can't send request to memory system yet\n");
            return;
        }

        SimpleThread &thread = *cpu.threads[request->inst->id.threadId];

        TheISA::PCState old_pc = thread.pcState();
        ExecContext context(cpu, thread, execute, request->inst);

        /* Handle LLSC requests and tests */
        if (is_load) {
            TheISA::handleLockedRead(&context, &request->request);
        } else {
            do_access = TheISA::handleLockedWrite(&context,
                &request->request, cacheBlockMask);

            if (!do_access) {
                DPRINTF(MinorMem, "Not perfoming a memory "
                    "access for store conditional\n");
            }
        }
        thread.pcState(old_pc);
    }

    /* See the do_access comment above */
    if (do_access) {
        if (!canSendToMemorySystem()) {
            DPRINTF(MinorMem, "Can't send request to memory system yet\n");
            return;
        }

        /* Remember if this is an access which can't be idly
         *  discarded by an interrupt */
        if (!bufferable && !request->issuedToMemory) {
            numAccessesIssuedToMemory++;
            request->issuedToMemory = true;
        }

        if (tryToSend(request)) {
            moveFromRequestsToTransfers(request);
        }
    } else {
        request->setState(LSQRequest::Complete);
        moveFromRequestsToTransfers(request);
    }
}

bool
LSQ::tryToSend(LSQRequestPtr request)
{
    bool ret = false;

    if (!canSendToMemorySystem()) {
        DPRINTF(MinorMem, "Can't send request: %s yet, no space in memory\n",
            *(request->inst));
    } else {
        PacketPtr packet = request->getHeadPacket();

        DPRINTF(MinorMem, "Trying to send request: %s addr: 0x%x\n",
            *(request->inst), packet->req->getVaddr());

        /* The sender state of the packet *must* be an LSQRequest
         *  so the response can be correctly handled */
        assert(packet->findNextSenderState<LSQRequest>());

        if (request->request.isMmappedIpr()) {
            ThreadContext *thread =
                cpu.getContext(cpu.contextToThread(
                                request->request.contextId()));

            if (request->isLoad) {
                DPRINTF(MinorMem, "IPR read inst: %s\n", *(request->inst));
                TheISA::handleIprRead(thread, packet);
            } else {
                DPRINTF(MinorMem, "IPR write inst: %s\n", *(request->inst));
                TheISA::handleIprWrite(thread, packet);
            }

            request->stepToNextPacket();
            ret = request->sentAllPackets();

            if (!ret) {
                DPRINTF(MinorMem, "IPR access has another packet: %s\n",
                    *(request->inst));
            }

            if (ret)
                request->setState(LSQRequest::Complete);
            else
                request->setState(LSQRequest::RequestIssuing);
        } else if (dcachePort.sendTimingReq(packet)) {
            DPRINTF(MinorMem, "Sent data memory request\n");

            numAccessesInMemorySystem++;

            request->stepToNextPacket();

            ret = request->sentAllPackets();

            switch (request->state) {
              case LSQRequest::Translated:
              case LSQRequest::RequestIssuing:
                /* Fully or partially issued a request in the transfers
                 *  queue */
                request->setState(LSQRequest::RequestIssuing);
                break;
              case LSQRequest::StoreInStoreBuffer:
              case LSQRequest::StoreBufferIssuing:
                /* Fully or partially issued a request in the store
                 *  buffer */
                request->setState(LSQRequest::StoreBufferIssuing);
                break;
              default:
                assert(false);
                break;
            }

            state = MemoryRunning;
        } else {
            DPRINTF(MinorMem,
                "Sending data memory request - needs retry\n");

            /* Needs to be resent, wait for that */
            state = MemoryNeedsRetry;
            retryRequest = request;

            switch (request->state) {
              case LSQRequest::Translated:
              case LSQRequest::RequestIssuing:
                request->setState(LSQRequest::RequestNeedsRetry);
                break;
              case LSQRequest::StoreInStoreBuffer:
              case LSQRequest::StoreBufferIssuing:
                request->setState(LSQRequest::StoreBufferNeedsRetry);
                break;
              default:
                assert(false);
                break;
            }
        }
    }

    if (ret)
        threadSnoop(request);

    return ret;
}

void
LSQ::moveFromRequestsToTransfers(LSQRequestPtr request)
{
    assert(!requests.empty() && requests.front() == request);
    assert(transfers.unreservedRemainingSpace() != 0);

    /* Need to count the number of stores in the transfers
     *  queue so that loads know when their store buffer forwarding
     *  results will be correct (only when all those stores
     *  have reached the store buffer) */
    if (!request->isLoad)
        numStoresInTransfers++;

    requests.pop();
    transfers.push(request);
}

bool
LSQ::canSendToMemorySystem()
{
    return state == MemoryRunning &&
        numAccessesInMemorySystem < inMemorySystemLimit;
}

bool
LSQ::recvTimingResp(PacketPtr response)
{
    LSQRequestPtr request =
        safe_cast<LSQRequestPtr>(response->popSenderState());

    DPRINTF(MinorMem, "Received response packet inst: %s"
        " addr: 0x%x cmd: %s\n",
        *(request->inst), response->getAddr(),
        response->cmd.toString());

    numAccessesInMemorySystem--;

    if (response->isError()) {
        DPRINTF(MinorMem, "Received error response packet: %s\n",
            *request->inst);
    }

    switch (request->state) {
      case LSQRequest::RequestIssuing:
      case LSQRequest::RequestNeedsRetry:
        /* Response to a request from the transfers queue */
        request->retireResponse(response);

        DPRINTF(MinorMem, "Has outstanding packets?: %d %d\n",
            request->hasPacketsInMemSystem(), request->isComplete());

        break;
      case LSQRequest::StoreBufferIssuing:
      case LSQRequest::StoreBufferNeedsRetry:
        /* Response to a request from the store buffer */
        request->retireResponse(response);

        /* Remove completed requests unless they are barriers (which will
         *  need to be removed in order */
        if (request->isComplete()) {
            if (!request->isBarrier()) {
                storeBuffer.deleteRequest(request);
            } else {
                DPRINTF(MinorMem, "Completed transfer for barrier: %s"
                    " leaving the request as it is also a barrier\n",
                    *(request->inst));
            }
        }
        break;
      default:
        /* Shouldn't be allowed to receive a response from another
         *  state */
        assert(false);
        break;
    }

    /* We go to idle even if there are more things in the requests queue
     * as it's the job of step to actually step us on to the next
     * transaction */

    /* Let's try and wake up the processor for the next cycle */
    cpu.wakeupOnEvent(Pipeline::ExecuteStageId);

    /* Never busy */
    return true;
}

void
LSQ::recvReqRetry()
{
    DPRINTF(MinorMem, "Received retry request\n");

    assert(state == MemoryNeedsRetry);

    switch (retryRequest->state) {
      case LSQRequest::RequestNeedsRetry:
        /* Retry in the requests queue */
        retryRequest->setState(LSQRequest::Translated);
        break;
      case LSQRequest::StoreBufferNeedsRetry:
        /* Retry in the store buffer */
        retryRequest->setState(LSQRequest::StoreInStoreBuffer);
        break;
      default:
        assert(false);
    }

    /* Set state back to MemoryRunning so that the following
     *  tryToSend can actually send.  Note that this won't
     *  allow another transfer in as tryToSend should
     *  issue a memory request and either succeed for this
     *  request or return the LSQ back to MemoryNeedsRetry */
    state = MemoryRunning;

    /* Try to resend the request */
    if (tryToSend(retryRequest)) {
        /* Successfully sent, need to move the request */
        switch (retryRequest->state) {
          case LSQRequest::RequestIssuing:
            /* In the requests queue */
            moveFromRequestsToTransfers(retryRequest);
            break;
          case LSQRequest::StoreBufferIssuing:
            /* In the store buffer */
            storeBuffer.countIssuedStore(retryRequest);
            break;
          default:
            assert(false);
            break;
        }

        retryRequest = NULL;
    }
}

LSQ::LSQ(std::string name_, std::string dcache_port_name_,
    MinorCPU &cpu_, Execute &execute_,
    unsigned int in_memory_system_limit, unsigned int line_width,
    unsigned int requests_queue_size, unsigned int transfers_queue_size,
    unsigned int store_buffer_size,
    unsigned int store_buffer_cycle_store_limit) :
    Named(name_),
    cpu(cpu_),
    execute(execute_),
    dcachePort(dcache_port_name_, *this, cpu_),
    lastMemBarrier(cpu.numThreads, 0),
    state(MemoryRunning),
    inMemorySystemLimit(in_memory_system_limit),
    lineWidth((line_width == 0 ? cpu.cacheLineSize() : line_width)),
    requests(name_ + ".requests", "addr", requests_queue_size),
    transfers(name_ + ".transfers", "addr", transfers_queue_size),
    storeBuffer(name_ + ".storeBuffer",
        *this, store_buffer_size, store_buffer_cycle_store_limit),
    numAccessesInMemorySystem(0),
    numAccessesInDTLB(0),
    numStoresInTransfers(0),
    numAccessesIssuedToMemory(0),
    retryRequest(NULL),
    cacheBlockMask(~(cpu_.cacheLineSize() - 1))
{
    if (in_memory_system_limit < 1) {
        fatal("%s: executeMaxAccessesInMemory must be >= 1 (%d)\n", name_,
            in_memory_system_limit);
    }

    if (store_buffer_cycle_store_limit < 1) {
        fatal("%s: executeLSQMaxStoreBufferStoresPerCycle must be"
            " >= 1 (%d)\n", name_, store_buffer_cycle_store_limit);
    }

    if (requests_queue_size < 1) {
        fatal("%s: executeLSQRequestsQueueSize must be"
            " >= 1 (%d)\n", name_, requests_queue_size);
    }

    if (transfers_queue_size < 1) {
        fatal("%s: executeLSQTransfersQueueSize must be"
            " >= 1 (%d)\n", name_, transfers_queue_size);
    }

    if (store_buffer_size < 1) {
        fatal("%s: executeLSQStoreBufferSize must be"
            " >= 1 (%d)\n", name_, store_buffer_size);
    }

    if ((lineWidth & (lineWidth - 1)) != 0) {
        fatal("%s: lineWidth: %d must be a power of 2\n", name(), lineWidth);
    }
}

LSQ::~LSQ()
{ }

LSQ::LSQRequest::~LSQRequest()
{
    if (packet)
        delete packet;
    if (data)
        delete [] data;
}

/**
 *  Step the memory access mechanism on to its next state.  In reality, most
 *  of the stepping is done by the callbacks on the LSQ but this
 *  function is responsible for issuing memory requests lodged in the
 *  requests queue.
 */
void
LSQ::step()
{
    /* Try to move address-translated requests between queues and issue
     *  them */
    if (!requests.empty())
        tryToSendToTransfers(requests.front());

    storeBuffer.step();
}

LSQ::LSQRequestPtr
LSQ::findResponse(MinorDynInstPtr inst)
{
    LSQ::LSQRequestPtr ret = NULL;

    if (!transfers.empty()) {
        LSQRequestPtr request = transfers.front();

        /* Same instruction and complete access or a store that's
         *  capable of being moved to the store buffer */
        if (request->inst->id == inst->id) {
            bool complete = request->isComplete();
            bool can_store = storeBuffer.canInsert();
            bool to_store_buffer = request->state ==
                LSQRequest::StoreToStoreBuffer;

            if ((complete && !(request->isBarrier() && !can_store)) ||
                (to_store_buffer && can_store))
            {
                ret = request;
            }
        }
    }

    if (ret) {
        DPRINTF(MinorMem, "Found matching memory response for inst: %s\n",
            *inst);
    } else {
        DPRINTF(MinorMem, "No matching memory response for inst: %s\n",
            *inst);
    }

    return ret;
}

void
LSQ::popResponse(LSQ::LSQRequestPtr response)
{
    assert(!transfers.empty() && transfers.front() == response);

    transfers.pop();

    if (!response->isLoad)
        numStoresInTransfers--;

    if (response->issuedToMemory)
        numAccessesIssuedToMemory--;

    if (response->state != LSQRequest::StoreInStoreBuffer) {
        DPRINTF(MinorMem, "Deleting %s request: %s\n",
            (response->isLoad ? "load" : "store"),
            *(response->inst));

        delete response;
    }
}

void
LSQ::sendStoreToStoreBuffer(LSQRequestPtr request)
{
    assert(request->state == LSQRequest::StoreToStoreBuffer);

    DPRINTF(MinorMem, "Sending store: %s to store buffer\n",
        *(request->inst));

    request->inst->inStoreBuffer = true;

    storeBuffer.insert(request);
}

bool
LSQ::isDrained()
{
    return requests.empty() && transfers.empty() &&
        storeBuffer.isDrained();
}

bool
LSQ::needsToTick()
{
    bool ret = false;

    if (canSendToMemorySystem()) {
        bool have_translated_requests = !requests.empty() &&
            requests.front()->state != LSQRequest::InTranslation &&
            transfers.unreservedRemainingSpace() != 0;

        ret = have_translated_requests ||
            storeBuffer.numUnissuedStores() != 0;
    }

    if (ret)
        DPRINTF(Activity, "Need to tick\n");

    return ret;
}

void
LSQ::pushRequest(MinorDynInstPtr inst, bool isLoad, uint8_t *data,
                 unsigned int size, Addr addr, Request::Flags flags,
                 uint64_t *res)
{
    bool needs_burst = transferNeedsBurst(addr, size, lineWidth);
    LSQRequestPtr request;

    /* Copy given data into the request.  The request will pass this to the
     *  packet and then it will own the data */
    uint8_t *request_data = NULL;

    DPRINTF(MinorMem, "Pushing request (%s) addr: 0x%x size: %d flags:"
        " 0x%x%s lineWidth : 0x%x\n",
        (isLoad ? "load" : "store"), addr, size, flags,
            (needs_burst ? " (needs burst)" : ""), lineWidth);

    if (!isLoad) {
        /* request_data becomes the property of a ...DataRequest (see below)
         *  and destroyed by its destructor */
        request_data = new uint8_t[size];
        if (flags & Request::CACHE_BLOCK_ZERO) {
            /* For cache zeroing, just use zeroed data */
            std::memset(request_data, 0, size);
        } else {
            std::memcpy(request_data, data, size);
        }
    }

    if (needs_burst) {
        request = new SplitDataRequest(
            *this, inst, isLoad, request_data, res);
    } else {
        request = new SingleDataRequest(
            *this, inst, isLoad, request_data, res);
    }

    if (inst->traceData)
        inst->traceData->setMem(addr, size, flags);

    int cid = cpu.threads[inst->id.threadId]->getTC()->contextId();
    request->request.setContext(cid);
    request->request.setVirt(0 /* asid */,
        addr, size, flags, cpu.dataMasterId(),
        /* I've no idea why we need the PC, but give it */
        inst->pc.instAddr());

    requests.push(request);
    request->startAddrTranslation();
}

void
LSQ::pushFailedRequest(MinorDynInstPtr inst)
{
    LSQRequestPtr request = new FailedDataRequest(*this, inst);
    requests.push(request);
}

void
LSQ::minorTrace() const
{
    MINORTRACE("state=%s in_tlb_mem=%d/%d stores_in_transfers=%d"
        " lastMemBarrier=%d\n",
        state, numAccessesInDTLB, numAccessesInMemorySystem,
        numStoresInTransfers, lastMemBarrier[0]);
    requests.minorTrace();
    transfers.minorTrace();
    storeBuffer.minorTrace();
}

LSQ::StoreBuffer::StoreBuffer(std::string name_, LSQ &lsq_,
    unsigned int store_buffer_size,
    unsigned int store_limit_per_cycle) :
    Named(name_), lsq(lsq_),
    numSlots(store_buffer_size),
    storeLimitPerCycle(store_limit_per_cycle),
    slots(),
    numUnissuedAccesses(0)
{
}

PacketPtr
makePacketForRequest(Request &request, bool isLoad,
    Packet::SenderState *sender_state, PacketDataPtr data)
{
    PacketPtr ret = isLoad ? Packet::createRead(&request)
                           : Packet::createWrite(&request);

    if (sender_state)
        ret->pushSenderState(sender_state);

    if (isLoad)
        ret->allocate();
    else
        ret->dataDynamic(data);

    return ret;
}

void
LSQ::issuedMemBarrierInst(MinorDynInstPtr inst)
{
    assert(inst->isInst() && inst->staticInst->isMemBarrier());
    assert(inst->id.execSeqNum > lastMemBarrier[inst->id.threadId]);

    /* Remember the barrier.  We only have a notion of one
     *  barrier so this may result in some mem refs being
     *  delayed if they are between barriers */
    lastMemBarrier[inst->id.threadId] = inst->id.execSeqNum;
}

void
LSQ::LSQRequest::makePacket()
{
    /* Make the function idempotent */
    if (packet)
        return;

    // if the translation faulted, do not create a packet
    if (fault != NoFault) {
        assert(packet == NULL);
        return;
    }

    packet = makePacketForRequest(request, isLoad, this, data);
    /* Null the ret data so we know not to deallocate it when the
     * ret is destroyed.  The data now belongs to the ret and
     * the ret is responsible for its destruction */
    data = NULL;
}

std::ostream &
operator <<(std::ostream &os, LSQ::MemoryState state)
{
    switch (state) {
      case LSQ::MemoryRunning:
        os << "MemoryRunning";
        break;
      case LSQ::MemoryNeedsRetry:
        os << "MemoryNeedsRetry";
        break;
      default:
        os << "MemoryState-" << static_cast<int>(state);
        break;
    }
    return os;
}

void
LSQ::recvTimingSnoopReq(PacketPtr pkt)
{
    /* LLSC operations in Minor can't be speculative and are executed from
     * the head of the requests queue.  We shouldn't need to do more than
     * this action on snoops. */
    for (ThreadID tid = 0; tid < cpu.numThreads; tid++) {
        if (cpu.getCpuAddrMonitor(tid)->doMonitor(pkt)) {
            cpu.wakeup(tid);
        }
    }

    if (pkt->isInvalidate() || pkt->isWrite()) {
        for (ThreadID tid = 0; tid < cpu.numThreads; tid++) {
            TheISA::handleLockedSnoop(cpu.getContext(tid), pkt,
                                      cacheBlockMask);
        }
    }
}

void
LSQ::threadSnoop(LSQRequestPtr request)
{
    /* LLSC operations in Minor can't be speculative and are executed from
     * the head of the requests queue.  We shouldn't need to do more than
     * this action on snoops. */
    ThreadID req_tid = request->inst->id.threadId;
    PacketPtr pkt = request->packet;

    for (ThreadID tid = 0; tid < cpu.numThreads; tid++) {
        if (tid != req_tid) {
            if (cpu.getCpuAddrMonitor(tid)->doMonitor(pkt)) {
                cpu.wakeup(tid);
            }

            if (pkt->isInvalidate() || pkt->isWrite()) {
                TheISA::handleLockedSnoop(cpu.getContext(tid), pkt,
                                          cacheBlockMask);
            }
        }
    }
}

}
