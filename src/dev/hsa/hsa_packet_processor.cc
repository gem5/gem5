/*
 * Copyright (c) 2015-2018 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
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
 *
 * Authors: Eric van Tassell
 */

#include "dev/hsa/hsa_packet_processor.hh"

#include <cstring>

#include "base/chunk_generator.hh"
#include "base/compiler.hh"
#include "debug/HSAPacketProcessor.hh"
#include "dev/dma_device.hh"
#include "dev/hsa/hsa_device.hh"
#include "dev/hsa/hsa_packet.hh"
#include "dev/hsa/hw_scheduler.hh"
#include "mem/packet_access.hh"
#include "mem/page_table.hh"
#include "sim/process.hh"
#include "sim/syscall_emul_buf.hh"
#include "sim/system.hh"

#define HSAPP_EVENT_DESCRIPTION_GENERATOR(XEVENT) \
  const char*                                    \
  HSAPacketProcessor::XEVENT::description() const       \
  {                                              \
      return #XEVENT;                            \
  }

#define PKT_TYPE(PKT) ((hsa_packet_type_t)(((PKT->header) >> \
            HSA_PACKET_HEADER_TYPE) & (HSA_PACKET_HEADER_WIDTH_TYPE - 1)))

HSAPP_EVENT_DESCRIPTION_GENERATOR(UpdateReadDispIdDmaEvent)
HSAPP_EVENT_DESCRIPTION_GENERATOR(CmdQueueCmdDmaEvent)
HSAPP_EVENT_DESCRIPTION_GENERATOR(QueueProcessEvent)
HSAPP_EVENT_DESCRIPTION_GENERATOR(DepSignalsReadDmaEvent)

HSAPacketProcessor::HSAPacketProcessor(const Params *p)
    : DmaDevice(p), numHWQueues(p->numHWQueues), pioAddr(p->pioAddr),
      pioSize(PAGE_SIZE), pioDelay(10), pktProcessDelay(p->pktProcessDelay)
{
    DPRINTF(HSAPacketProcessor, "%s:\n", __FUNCTION__);
    hwSchdlr = new HWScheduler(this, p->wakeupDelay);
    regdQList.resize(numHWQueues);
    for (int i = 0; i < numHWQueues; i++) {
        regdQList[i] = new RQLEntry(this, i);
    }
}

HSAPacketProcessor::~HSAPacketProcessor()
{
    for (auto &queue : regdQList) {
        delete queue;
    }
}

void
HSAPacketProcessor::unsetDeviceQueueDesc(uint64_t queue_id)
{
    hwSchdlr->unregisterQueue(queue_id);
}

void
HSAPacketProcessor::setDeviceQueueDesc(uint64_t hostReadIndexPointer,
                                       uint64_t basePointer,
                                       uint64_t queue_id,
                                       uint32_t size)
{
    DPRINTF(HSAPacketProcessor,
             "%s:base = %p, qID = %d, ze = %d\n", __FUNCTION__,
             (void *)basePointer, queue_id, size);
    hwSchdlr->registerNewQueue(hostReadIndexPointer,
                               basePointer, queue_id, size);
}

AddrRangeList
HSAPacketProcessor::getAddrRanges() const
{
    assert(pioSize != 0);

    AddrRangeList ranges;
    ranges.push_back(RangeSize(pioAddr, pioSize));

    return ranges;
}

// Basically only processes writes to the queue doorbell register.
Tick
HSAPacketProcessor::write(Packet *pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);

    // TODO: How to get pid??
    Addr M5_VAR_USED daddr = pkt->getAddr() - pioAddr;

    DPRINTF(HSAPacketProcessor,
          "%s: write of size %d to reg-offset %d (0x%x)\n",
          __FUNCTION__, pkt->getSize(), daddr, daddr);

    uint32_t doorbell_reg = pkt->getLE<uint32_t>();

    DPRINTF(HSAPacketProcessor,
            "%s: write data 0x%x to offset %d (0x%x)\n",
            __FUNCTION__, doorbell_reg, daddr, daddr);
    hwSchdlr->write(daddr, doorbell_reg);
    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
HSAPacketProcessor::read(Packet *pkt)
{
    pkt->makeAtomicResponse();
    pkt->setBadAddress();
    return pioDelay;
}

void
HSAPacketProcessor::translateOrDie(Addr vaddr, Addr &paddr)
{
    // Grab the process and try to translate the virtual address with it; with
    // new extensions, it will likely be wrong to just arbitrarily grab context
    // zero.
    auto process = sys->getThreadContext(0)->getProcessPtr();

    if (!process->pTable->translate(vaddr, paddr))
        fatal("failed translation: vaddr 0x%x\n", vaddr);
}

void
HSAPacketProcessor::dmaVirt(DmaFnPtr dmaFn, Addr addr, unsigned size,
                         Event *event, void *data, Tick delay)
{
    if (size == 0) {
        schedule(event, curTick() + delay);
        return;
    }

    // move the buffer data pointer with the chunks
    uint8_t *loc_data = (uint8_t*)data;

    for (ChunkGenerator gen(addr, size, PAGE_SIZE); !gen.done(); gen.next()) {
        Addr phys;

        // translate pages into their corresponding frames
        translateOrDie(gen.addr(), phys);

        // only send event on last transfer; transfers complete in-order
        Event *ev = gen.last() ? event : NULL;

        (this->*dmaFn)(phys, gen.size(), ev, loc_data, delay);

        loc_data += gen.size();
    }
}

void
HSAPacketProcessor::dmaReadVirt(Addr host_addr, unsigned size,
                             Event *event, void *data, Tick delay)
{
    DPRINTF(HSAPacketProcessor,
            "%s:host_addr = 0x%lx, size = %d\n", __FUNCTION__, host_addr, size);
    dmaVirt(&DmaDevice::dmaRead, host_addr, size, event, data, delay);
}

void
HSAPacketProcessor::dmaWriteVirt(Addr host_addr, unsigned size,
                              Event *event, void *data, Tick delay)
{
    dmaVirt(&DmaDevice::dmaWrite, host_addr, size, event, data, delay);
}

HSAPacketProcessor::UpdateReadDispIdDmaEvent::
        UpdateReadDispIdDmaEvent()
    : Event(Default_Pri, AutoDelete)
{
    DPRINTF(HSAPacketProcessor, "%s:\n", __FUNCTION__);
    setFlags(AutoDelete);
}

void
HSAPacketProcessor::updateReadIndex(int pid, uint32_t rl_idx)
{
    AQLRingBuffer* aqlbuf = regdQList[rl_idx]->qCntxt.aqlBuf;
    HSAQueueDescriptor* qDesc = regdQList[rl_idx]->qCntxt.qDesc;
    auto *dmaEvent = new UpdateReadDispIdDmaEvent();

    DPRINTF(HSAPacketProcessor,
            "%s: read-pointer offset [0x%x]\n", __FUNCTION__, aqlbuf->rdIdx());

    dmaWriteVirt((Addr)qDesc->hostReadIndexPtr,
                 sizeof(aqlbuf->rdIdx()),
                 dmaEvent, aqlbuf->rdIdxPtr());

    DPRINTF(HSAPacketProcessor,
            "%s: rd-ptr offset [0x%x], wr-ptr offset [0x%x], space used = %d," \
            " q size = %d, is_empty = %s, active list ID = %d\n", __FUNCTION__,
            qDesc->readIndex, qDesc->writeIndex, qDesc->spaceUsed(),
            qDesc->numElts, qDesc->isEmpty()? "true" : "false", rl_idx);
    if (qDesc->writeIndex != aqlbuf->wrIdx()) {
        getCommandsFromHost(pid, rl_idx);
    }
}

HSAPacketProcessor::CmdQueueCmdDmaEvent::
CmdQueueCmdDmaEvent(HSAPacketProcessor *_hsaPP, int _pid, bool _isRead,
                    uint32_t _ix_start, unsigned _num_pkts,
                    dma_series_ctx *_series_ctx, void *_dest_4debug)
    : Event(Default_Pri, AutoDelete), hsaPP(_hsaPP), pid(_pid), isRead(_isRead),
      ix_start(_ix_start), num_pkts(_num_pkts), series_ctx(_series_ctx),
      dest_4debug(_dest_4debug)
{
    setFlags(AutoDelete);

    DPRINTF(HSAPacketProcessor, "%s, ix = %d, npkts = %d," \
            "active list ID = %d\n", __FUNCTION__,
            _ix_start, num_pkts, series_ctx->rl_idx);
}

void
HSAPacketProcessor::CmdQueueCmdDmaEvent::process()
{
    uint32_t rl_idx = series_ctx->rl_idx;
    AQLRingBuffer *aqlRingBuffer M5_VAR_USED =
        hsaPP->regdQList[rl_idx]->qCntxt.aqlBuf;
    HSAQueueDescriptor* qDesc =
        hsaPP->regdQList[rl_idx]->qCntxt.qDesc;
    DPRINTF(HSAPacketProcessor, ">%s, ix = %d, npkts = %d," \
            " pktsRemaining = %d, active list ID = %d\n", __FUNCTION__,
            ix_start, num_pkts, series_ctx->pkts_2_go,
            rl_idx);
    if (isRead) {
        series_ctx->pkts_2_go -= num_pkts;
        if (series_ctx->pkts_2_go == 0) {
            // Mark DMA as completed
            qDesc->dmaInProgress = false;
            DPRINTF(HSAPacketProcessor,
                    "%s: schedule Qwakeup next cycle, rdIdx %d, wrIdx %d," \
                    " dispIdx %d, active list ID = %d\n",
                    __FUNCTION__, aqlRingBuffer->rdIdx(),
                    aqlRingBuffer->wrIdx(), aqlRingBuffer->dispIdx(), rl_idx);
            // schedule queue wakeup
            hsaPP->schedAQLProcessing(rl_idx);
            delete series_ctx;
        }
    }
}

void
HSAPacketProcessor::schedAQLProcessing(uint32_t rl_idx)
{
    RQLEntry *queue = regdQList[rl_idx];
    if (!queue->aqlProcessEvent.scheduled()) {
        Tick processingTick = curTick() + pktProcessDelay;
        schedule(queue->aqlProcessEvent, processingTick);
        DPRINTF(HSAPacketProcessor, "AQL processing scheduled at tick: %d\n",
                processingTick);
    } else {
        DPRINTF(HSAPacketProcessor, "AQL processing already scheduled\n");
    }
}

bool
HSAPacketProcessor::processPkt(void* pkt, uint32_t rl_idx, Addr host_pkt_addr)
{
    bool is_submitted = false;
    SignalState *dep_sgnl_rd_st = &(regdQList[rl_idx]->depSignalRdState);
    // Dependency signals are not read yet. And this can only be a retry.
    // The retry logic will schedule the packet processor wakeup
    if (dep_sgnl_rd_st->pendingReads != 0) {
        return false;
    }
    // `pkt` can be typecasted to any type of AQL packet since they all
    // have header information at offset zero
    auto disp_pkt = (_hsa_dispatch_packet_t *)pkt;
    hsa_packet_type_t pkt_type = PKT_TYPE(disp_pkt);
    if (pkt_type == HSA_PACKET_TYPE_VENDOR_SPECIFIC) {
        DPRINTF(HSAPacketProcessor, "%s: submitting vendor specific pkt" \
                " active list ID = %d\n", __FUNCTION__, rl_idx);
        // Submit packet to HSA device (dispatcher)
        hsa_device->submitVendorPkt((void *)disp_pkt, rl_idx, host_pkt_addr);
        is_submitted = true;
    } else if (pkt_type == HSA_PACKET_TYPE_KERNEL_DISPATCH) {
        DPRINTF(HSAPacketProcessor, "%s: submitting kernel dispatch pkt" \
                " active list ID = %d\n", __FUNCTION__, rl_idx);
        // Submit packet to HSA device (dispatcher)
        hsa_device->submitDispatchPkt((void *)disp_pkt, rl_idx, host_pkt_addr);
        is_submitted = true;
    } else if (pkt_type == HSA_PACKET_TYPE_BARRIER_AND) {
        DPRINTF(HSAPacketProcessor, "%s: Processing barrier packet" \
                " active list ID = %d\n", __FUNCTION__, rl_idx);
        auto bar_and_pkt = (_hsa_barrier_and_packet_t *)pkt;
        bool isReady = true;
        // Loop thorugh all the completion signals to see if this barrier
        // packet is ready.
        for (int i = 0; i < NumSignalsPerBarrier; i++) {
            // dep_signal = zero imply no signal connected
            if (bar_and_pkt->dep_signal[i]) {
                // The signal value is aligned 8 bytes from
                // the actual handle in the runtime
                uint64_t signal_addr =
                    (uint64_t) (((uint64_t *) bar_and_pkt->dep_signal[i]) + 1);
                hsa_signal_value_t *signal_val =
                    &(dep_sgnl_rd_st->values[i]);
                DPRINTF(HSAPacketProcessor, "%s: Barrier pkt dep sgnl[%d]" \
                       " , sig addr %x, value %d active list ID = %d\n",
                       __FUNCTION__, i, signal_addr,
                       *signal_val, rl_idx);
                // The if condition will be executed everytime except the
                // very first time this barrier packet is encounteresd.
                if (dep_sgnl_rd_st->allRead) {
                    if (*signal_val != 0) {
                        // This signal is not yet ready, read it again
                        isReady = false;
                        DepSignalsReadDmaEvent *sgnl_rd_evnt =
                            new DepSignalsReadDmaEvent(dep_sgnl_rd_st);
                        dmaReadVirt(signal_addr, sizeof(hsa_signal_value_t),
                                    sgnl_rd_evnt, signal_val);
                        dep_sgnl_rd_st->pendingReads++;
                        DPRINTF(HSAPacketProcessor, "%s: Pending reads %d," \
                            " active list %d\n", __FUNCTION__,
                            dep_sgnl_rd_st->pendingReads, rl_idx);
                    }
                } else {
                    // This signal is not yet ready, read it again
                    isReady = false;
                    DepSignalsReadDmaEvent *sgnl_rd_evnt =
                        new DepSignalsReadDmaEvent(dep_sgnl_rd_st);
                    dmaReadVirt(signal_addr, sizeof(hsa_signal_value_t),
                                sgnl_rd_evnt, signal_val);
                    dep_sgnl_rd_st->pendingReads++;
                    DPRINTF(HSAPacketProcessor, "%s: Pending reads %d," \
                        " active list %d\n", __FUNCTION__,
                        dep_sgnl_rd_st->pendingReads, rl_idx);
                }
            }
        }
        if (isReady) {
            assert(dep_sgnl_rd_st->pendingReads == 0);
            DPRINTF(HSAPacketProcessor, "%s: Barrier packet completed" \
                    " active list ID = %d\n", __FUNCTION__, rl_idx);
            // TODO: Completion signal of barrier packet to be
            // atomically decremented here
            finishPkt((void*)bar_and_pkt, rl_idx);
            is_submitted = true;
            // Reset signal values
            dep_sgnl_rd_st->resetSigVals();
            // The completion signal is connected
            if (bar_and_pkt->completion_signal != 0) {
                // The signal value is aligned 8 bytes
                // from the actual handle in the runtime
                uint64_t signal_addr =
                    (uint64_t) (((uint64_t *)
                    bar_and_pkt->completion_signal) + 1);
                DPRINTF(HSAPacketProcessor, "Triggering barrier packet" \
                       " completion signal: %x!\n", signal_addr);
                /**
                 * HACK: The semantics of the HSA signal is to
                 * decrement the current signal value.
                 * I'm going to cheat here and read out
                 * the value from main memory using functional
                 * access, and then just DMA the decremented value.
                 * The reason for this is that the DMASequencer does
                 * not support atomic operations.
                 */
                auto tc = sys->getThreadContext(0);
                auto &virt_proxy = tc->getVirtProxy();
                TypedBufferArg<uint64_t> prev_signal(signal_addr);
                prev_signal.copyIn(virt_proxy);

                hsa_signal_value_t *new_signal = new hsa_signal_value_t;
                *new_signal = (hsa_signal_value_t) *prev_signal - 1;

                dmaWriteVirt(signal_addr,
                             sizeof(hsa_signal_value_t), NULL, new_signal, 0);
            }
        }
        if (dep_sgnl_rd_st->pendingReads > 0) {
            // Atleast one DepSignalsReadDmaEvent is scheduled this cycle
            dep_sgnl_rd_st->allRead = false;
            dep_sgnl_rd_st->discardRead = false;
        }
    } else if (pkt_type == HSA_PACKET_TYPE_BARRIER_OR) {
        fatal("Unsupported packet type HSA_PACKET_TYPE_BARRIER_OR");
    } else if (pkt_type == HSA_PACKET_TYPE_INVALID) {
        fatal("Unsupported packet type HSA_PACKET_TYPE_INVALID");
    } else {
        fatal("Unsupported packet type %d\n", pkt_type);
    }
    return is_submitted;
}

// Wakes up every fixed time interval (pktProcessDelay) and processes a single
// packet from the queue that scheduled this wakeup. If there are more
// packets in that queue, the next wakeup is scheduled.
void
HSAPacketProcessor::QueueProcessEvent::process()
{
    AQLRingBuffer *aqlRingBuffer = hsaPP->regdQList[rqIdx]->qCntxt.aqlBuf;
    DPRINTF(HSAPacketProcessor,
            "%s: Qwakeup , rdIdx %d, wrIdx %d," \
            " dispIdx %d, active list ID = %d\n",
            __FUNCTION__, aqlRingBuffer->rdIdx(),
            aqlRingBuffer->wrIdx(), aqlRingBuffer->dispIdx(), rqIdx);
    // In the future, we may support batch processing of packets.
    // Then, we can just remove the break statements and the code
    // will support batch processing. That is why we are using a
    // "while loop" here instead on an "if" condition.
    while (hsaPP->regdQList[rqIdx]->dispPending()) {
        void *pkt = aqlRingBuffer->ptr(aqlRingBuffer->dispIdx());
        DPRINTF(HSAPacketProcessor, "%s: Attempting dispatch @ dispIdx[%d]\n",
                __FUNCTION__, aqlRingBuffer->dispIdx());
        Addr host_addr = aqlRingBuffer->hostDispAddr();
        if (hsaPP->processPkt(pkt, rqIdx, host_addr)) {
             aqlRingBuffer->incDispIdx(1);
             DPRINTF(HSAPacketProcessor, "%s: Increment dispIdx[%d]\n",
                     __FUNCTION__, aqlRingBuffer->dispIdx());
             if (hsaPP->regdQList[rqIdx]->dispPending()) {
                 hsaPP->schedAQLProcessing(rqIdx);
             }
             break;
        } else {
            // This queue is blocked, scheduled a processing event
            hsaPP->schedAQLProcessing(rqIdx);
            break;
        }
    }
}

void
HSAPacketProcessor::SignalState::handleReadDMA()
{
    assert(pendingReads > 0);
    pendingReads--;
    if (pendingReads == 0) {
        allRead = true;
        if (discardRead) {
            resetSigVals();
        }
    }
}

void
HSAPacketProcessor::getCommandsFromHost(int pid, uint32_t rl_idx)
{
    HSAQueueDescriptor* qDesc = regdQList[rl_idx]->qCntxt.qDesc;
    AQLRingBuffer *aqlRingBuffer = regdQList[rl_idx]->qCntxt.aqlBuf;

    DPRINTF(HSAPacketProcessor,
            "%s: read-pointer offset[0x%x], write-pointer offset[0x%x]"
            " doorbell(%d)[0x%x] \n",
            __FUNCTION__, qDesc->readIndex,
            qDesc->writeIndex, pid, qDesc->doorbellPointer);

    if (qDesc->dmaInProgress) {
        // we'll try again when this dma transfer completes in updateReadIndex
        return;
    }
    uint32_t num_umq = qDesc->spaceUsed();
    if (num_umq == 0)
        return; // nothing to be gotten
    uint32_t umq_nxt = qDesc->readIndex;
    // Total AQL buffer size
    uint32_t ttl_aql_buf = aqlRingBuffer->numObjs();
    // Available AQL buffer size. If the available buffer is less than
    // demanded, number of available buffer is returned
    uint32_t got_aql_buf = aqlRingBuffer->allocEntry(num_umq);
    qDesc->readIndex += got_aql_buf;
    uint32_t dma_start_ix = (aqlRingBuffer->wrIdx() - got_aql_buf) %
        ttl_aql_buf;
    dma_series_ctx *series_ctx = NULL;

    DPRINTF(HSAPacketProcessor, "%s: umq_nxt = %d, ttl_aql_buf = %d, "
            "dma_start_ix = %d, num_umq = %d\n", __FUNCTION__, umq_nxt,
            ttl_aql_buf, dma_start_ix, num_umq);

    if (got_aql_buf == 0) {
        // we'll try again when some dma bufs are freed in freeEntry
        qDesc->stalledOnDmaBufAvailability = true;
        return;
    } else {
        qDesc->stalledOnDmaBufAvailability = false;
    }

    uint32_t dma_b4_wrap = ttl_aql_buf - dma_start_ix;
    while (got_aql_buf != 0 && num_umq != 0) {
        uint32_t umq_b4_wrap = qDesc->numObjs() -
            (umq_nxt % qDesc->objSize());
        uint32_t num_2_xfer
            = std::min({umq_b4_wrap, dma_b4_wrap, num_umq, got_aql_buf});
        if (!series_ctx) {
            qDesc->dmaInProgress = true;
            series_ctx = new dma_series_ctx(got_aql_buf, got_aql_buf,
                                            dma_start_ix, rl_idx);
        }

        void *aql_buf = aqlRingBuffer->ptr(dma_start_ix);
        CmdQueueCmdDmaEvent *dmaEvent
            = new CmdQueueCmdDmaEvent(this, pid, true, dma_start_ix,
                                      num_2_xfer, series_ctx, aql_buf);
        DPRINTF(HSAPacketProcessor,
                "%s: aql_buf = %p, umq_nxt = %d, dma_ix = %d, num2xfer = %d\n",
                __FUNCTION__, aql_buf, umq_nxt, dma_start_ix, num_2_xfer);

        dmaReadVirt(qDesc->ptr(umq_nxt), num_2_xfer * qDesc->objSize(),
                    dmaEvent, aql_buf);

        aqlRingBuffer->saveHostDispAddr(qDesc->ptr(umq_nxt), num_2_xfer,
                                        dma_start_ix);

        num_umq -= num_2_xfer;
        got_aql_buf -= num_2_xfer;
        dma_start_ix = (dma_start_ix + num_2_xfer) % ttl_aql_buf;
        umq_nxt = (umq_nxt + num_2_xfer) % qDesc->numObjs();
        if (got_aql_buf == 0 && num_umq != 0) {
            // There are more packets in the queue but
            // not enough DMA buffers. Set the stalledOnDmaBufAvailability,
            // we will try again in freeEntry
            qDesc->stalledOnDmaBufAvailability = true;
        }
    }
}

void
HSAPacketProcessor::displayQueueDescriptor(int pid, uint32_t rl_idx)
{
    HSAQueueDescriptor* M5_VAR_USED qDesc = regdQList[rl_idx]->qCntxt.qDesc;
    DPRINTF(HSAPacketProcessor,
            "%s: pid[%d], basePointer[0x%lx], dBPointer[0x%lx], "
            "writeIndex[0x%x], readIndex[0x%x], size(bytes)[0x%x]\n",
            __FUNCTION__, pid, qDesc->basePointer,
            qDesc->doorbellPointer, qDesc->writeIndex,
            qDesc->readIndex, qDesc->numElts);
}

AQLRingBuffer::AQLRingBuffer(uint32_t size,
                             const std::string name)
        : _name(name), _wrIdx(0), _rdIdx(0), _dispIdx(0)
{
    _aqlBuf.resize(size);
    _aqlComplete.resize(size);
    _hostDispAddresses.resize(size);
    // Mark all packets as invalid and incomplete
    for (auto& it : _aqlBuf)
        it.header = HSA_PACKET_TYPE_INVALID;
    std::fill(_aqlComplete.begin(), _aqlComplete.end(), false);
}

bool
AQLRingBuffer::freeEntry(void *pkt)
{
    _aqlComplete[(hsa_kernel_dispatch_packet_t *) pkt - _aqlBuf.data()] = true;
    DPRINTF(HSAPacketProcessor, "%s: pkt_ix = %d; "\
            " # free entries = %d, wrIdx = %d, rdIdx = %d\n", __FUNCTION__,
            (hsa_kernel_dispatch_packet_t *) pkt - _aqlBuf.data(),
            nFree(), wrIdx(), rdIdx());
    // Packets can complete out-of-order. This code "retires" packets in-order
    // by updating the read pointer in the MQD when a contiguous chunk of
    // packets have finished.
    uint32_t old_rdIdx = rdIdx();
    while (_aqlComplete[rdIdx() % numObjs()]) {
       _aqlComplete[rdIdx() % numObjs()] = false;
       _aqlBuf[rdIdx() % numObjs()].header = HSA_PACKET_TYPE_INVALID;
       incRdIdx(1);
    }
    return (old_rdIdx != rdIdx());
}

void
HSAPacketProcessor::setDevice(HSADevice *dev)
{
    this->hsa_device = dev;
}

int
AQLRingBuffer::allocEntry(uint32_t nBufReq)
{
    DPRINTF(HSAPacketProcessor, "%s: nReq = %d\n", __FUNCTION__, nBufReq);
    if (nFree() == 0) {
        DPRINTF(HSAPacketProcessor, "%s: return = %d\n", __FUNCTION__, 0);
        return 0;
    }

    if (nBufReq > nFree())
        nBufReq = nFree();

    DPRINTF(HSAPacketProcessor, "%s: ix1stFree = %d\n", __FUNCTION__, wrIdx());
    incWrIdx(nBufReq);
    DPRINTF(HSAPacketProcessor, "%s: return = %d, wrIdx = %d\n",
            __FUNCTION__, nBufReq, wrIdx());
    return nBufReq;
}

HSAPacketProcessor *
HSAPacketProcessorParams::create()
{
    return new HSAPacketProcessor(this);
}

void
HSAPacketProcessor::finishPkt(void *pvPkt, uint32_t rl_idx)
{
    HSAQueueDescriptor* qDesc = regdQList[rl_idx]->qCntxt.qDesc;
    if (regdQList[rl_idx]->qCntxt.aqlBuf->freeEntry(pvPkt))
        updateReadIndex(0, rl_idx);
    DPRINTF(HSAPacketProcessor,
            "%s: rd-ptr offset [0x%x], wr-ptr offset [0x%x], space used = %d," \
            " q size = %d, stalled = %s, empty = %s, active list ID = %d\n",
            __FUNCTION__, qDesc->readIndex, qDesc->writeIndex,
            qDesc->spaceUsed(), qDesc->numElts,
            qDesc->stalledOnDmaBufAvailability? "true" : "false",
            qDesc->isEmpty()? "true" : "false", rl_idx);
    // DMA buffer is freed, check the queue to see if there are DMA
    // accesses blocked becasue of non-availability of DMA buffer
    if (qDesc->stalledOnDmaBufAvailability) {
        assert(!qDesc->isEmpty());
        getCommandsFromHost(0, rl_idx); // TODO:assign correct pid
                                        // when implementing
                                        // multi-process support
    }
}
