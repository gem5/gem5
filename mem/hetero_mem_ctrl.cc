/*
 * Copyright (c) 2010-2020 ARM Limited
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
 * Copyright (c) 2013 Amin Farmahini-Farahani
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
 */

#include "mem/hetero_mem_ctrl.hh"

#include "base/trace.hh"
#include "debug/DRAM.hh"
#include "debug/Drain.hh"
#include "debug/MemCtrl.hh"
#include "debug/NVM.hh"
#include "debug/QOS.hh"
#include "mem/dram_interface.hh"
#include "mem/mem_interface.hh"
#include "mem/nvm_interface.hh"
#include "sim/system.hh"

namespace gem5
{

namespace memory
{

HeteroMemCtrl::HeteroMemCtrl(const HeteroMemCtrlParams &p) :
    MemCtrl(p),
    nvm(p.nvm)
{
    DPRINTF(MemCtrl, "Setting up controller\n");
    readQueue.resize(p.qos_priorities);
    writeQueue.resize(p.qos_priorities);

    fatal_if(dynamic_cast<DRAMInterface*>(dram) == nullptr,
            "HeteroMemCtrl's dram interface must be of type DRAMInterface.\n");
    fatal_if(dynamic_cast<NVMInterface*>(nvm) == nullptr,
            "HeteroMemCtrl's nvm interface must be of type NVMInterface.\n");

    // hook up interfaces to the controller
    dram->setCtrl(this, commandWindow);
    nvm->setCtrl(this, commandWindow);

    readBufferSize = dram->readBufferSize + nvm->readBufferSize;
    writeBufferSize = dram->writeBufferSize + nvm->writeBufferSize;

    writeHighThreshold = writeBufferSize * p.write_high_thresh_perc / 100.0;
    writeLowThreshold = writeBufferSize * p.write_low_thresh_perc / 100.0;

    // perform a basic check of the write thresholds
    if (p.write_low_thresh_perc >= p.write_high_thresh_perc)
        fatal("Write buffer low threshold %d must be smaller than the "
              "high threshold %d\n", p.write_low_thresh_perc,
              p.write_high_thresh_perc);
}

Tick
HeteroMemCtrl::recvAtomic(PacketPtr pkt)
{
    Tick latency = 0;

    if (dram->getAddrRange().contains(pkt->getAddr())) {
        latency = MemCtrl::recvAtomicLogic(pkt, dram);
    } else if (nvm->getAddrRange().contains(pkt->getAddr())) {
        latency = MemCtrl::recvAtomicLogic(pkt, nvm);
    } else {
        panic("Can't handle address range for packet %s\n", pkt->print());
    }

    return latency;
}

bool
HeteroMemCtrl::recvTimingReq(PacketPtr pkt)
{
    // This is where we enter from the outside world
    DPRINTF(MemCtrl, "recvTimingReq: request %s addr %#x size %d\n",
            pkt->cmdString(), pkt->getAddr(), pkt->getSize());

    panic_if(pkt->cacheResponding(), "Should not see packets where cache "
             "is responding");

    panic_if(!(pkt->isRead() || pkt->isWrite()),
             "Should only see read and writes at memory controller\n");

    // Calc avg gap between requests
    if (prevArrival != 0) {
        stats.totGap += curTick() - prevArrival;
    }
    prevArrival = curTick();

    // What type of media does this packet access?
    bool is_dram;
    if (dram->getAddrRange().contains(pkt->getAddr())) {
        is_dram = true;
    } else if (nvm->getAddrRange().contains(pkt->getAddr())) {
        is_dram = false;
    } else {
        panic("Can't handle address range for packet %s\n",
              pkt->print());
    }

    // Find out how many memory packets a pkt translates to
    // If the burst size is equal or larger than the pkt size, then a pkt
    // translates to only one memory packet. Otherwise, a pkt translates to
    // multiple memory packets
    unsigned size = pkt->getSize();
    uint32_t burst_size = is_dram ? dram->bytesPerBurst() :
                                    nvm->bytesPerBurst();
    unsigned offset = pkt->getAddr() & (burst_size - 1);
    unsigned int pkt_count = divCeil(offset + size, burst_size);

    // run the QoS scheduler and assign a QoS priority value to the packet
    qosSchedule( { &readQueue, &writeQueue }, burst_size, pkt);

    // check local buffers and do not accept if full
    if (pkt->isWrite()) {
        assert(size != 0);
        if (writeQueueFull(pkt_count)) {
            DPRINTF(MemCtrl, "Write queue full, not accepting\n");
            // remember that we have to retry this port
            retryWrReq = true;
            stats.numWrRetry++;
            return false;
        } else {
            addToWriteQueue(pkt, pkt_count, is_dram ? dram : nvm);
            // If we are not already scheduled to get a request out of the
            // queue, do so now
            if (!nextReqEvent.scheduled()) {
                DPRINTF(MemCtrl, "Request scheduled immediately\n");
                schedule(nextReqEvent, curTick());
            }
            stats.writeReqs++;
            stats.bytesWrittenSys += size;
        }
    } else {
        assert(pkt->isRead());
        assert(size != 0);
        if (readQueueFull(pkt_count)) {
            DPRINTF(MemCtrl, "Read queue full, not accepting\n");
            // remember that we have to retry this port
            retryRdReq = true;
            stats.numRdRetry++;
            return false;
        } else {
            if (!addToReadQueue(pkt, pkt_count, is_dram ? dram : nvm)) {
                // If we are not already scheduled to get a request out of the
                // queue, do so now
                if (!nextReqEvent.scheduled()) {
                    DPRINTF(MemCtrl, "Request scheduled immediately\n");
                    schedule(nextReqEvent, curTick());
                }
            }
            stats.readReqs++;
            stats.bytesReadSys += size;
        }
    }

    return true;
}

void
HeteroMemCtrl::processRespondEvent(MemInterface* mem_intr,
                        MemPacketQueue& queue,
                        EventFunctionWrapper& resp_event,
                        bool& retry_rd_req)
{
    DPRINTF(MemCtrl,
            "processRespondEvent(): Some req has reached its readyTime\n");

    if (queue.front()->isDram()) {
        MemCtrl::processRespondEvent(dram, queue, resp_event, retry_rd_req);
    } else {
        MemCtrl::processRespondEvent(nvm, queue, resp_event, retry_rd_req);
    }
}

MemPacketQueue::iterator
HeteroMemCtrl::chooseNext(MemPacketQueue& queue, Tick extra_col_delay,
                    MemInterface* mem_int)
{
    // This method does the arbitration between requests.

    MemPacketQueue::iterator ret = queue.end();

    if (!queue.empty()) {
        if (queue.size() == 1) {
            // available rank corresponds to state refresh idle
            MemPacket* mem_pkt = *(queue.begin());
            if (packetReady(mem_pkt, mem_pkt->isDram()? dram : nvm)) {
                ret = queue.begin();
                DPRINTF(MemCtrl, "Single request, going to a free rank\n");
            } else {
                DPRINTF(MemCtrl, "Single request, going to a busy rank\n");
            }
        } else if (memSchedPolicy == enums::fcfs) {
            // check if there is a packet going to a free rank
            for (auto i = queue.begin(); i != queue.end(); ++i) {
                MemPacket* mem_pkt = *i;
                if (packetReady(mem_pkt, mem_pkt->isDram()? dram : nvm)) {
                    ret = i;
                    break;
                }
            }
        } else if (memSchedPolicy == enums::frfcfs) {
            Tick col_allowed_at;
            std::tie(ret, col_allowed_at)
                    = chooseNextFRFCFS(queue, extra_col_delay, mem_int);
        } else {
            panic("No scheduling policy chosen\n");
        }
    }
    return ret;
}

std::pair<MemPacketQueue::iterator, Tick>
HeteroMemCtrl::chooseNextFRFCFS(MemPacketQueue& queue, Tick extra_col_delay,
                          MemInterface* mem_intr)
{

    auto selected_pkt_it = queue.end();
    auto nvm_pkt_it = queue.end();
    Tick col_allowed_at = MaxTick;
    Tick nvm_col_allowed_at = MaxTick;

    std::tie(selected_pkt_it, col_allowed_at) =
            MemCtrl::chooseNextFRFCFS(queue, extra_col_delay, dram);

    std::tie(nvm_pkt_it, nvm_col_allowed_at) =
            MemCtrl::chooseNextFRFCFS(queue, extra_col_delay, nvm);


    // Compare DRAM and NVM and select NVM if it can issue
    // earlier than the DRAM packet
    if (col_allowed_at > nvm_col_allowed_at) {
        selected_pkt_it = nvm_pkt_it;
        col_allowed_at = nvm_col_allowed_at;
    }

    return std::make_pair(selected_pkt_it, col_allowed_at);
}


Tick
HeteroMemCtrl::doBurstAccess(MemPacket* mem_pkt, MemInterface* mem_intr)
{
    // mem_intr will be dram by default in HeteroMemCtrl

    // When was command issued?
    Tick cmd_at;

    if (mem_pkt->isDram()) {
        cmd_at = MemCtrl::doBurstAccess(mem_pkt, mem_intr);
        // Update timing for NVM ranks if NVM is configured on this channel
        nvm->addRankToRankDelay(cmd_at);
        // Since nextBurstAt and nextReqAt are part of the interface, making
        // sure that they are same for both nvm and dram interfaces
        nvm->nextBurstAt = dram->nextBurstAt;
        nvm->nextReqTime = dram->nextReqTime;

    } else {
        cmd_at = MemCtrl::doBurstAccess(mem_pkt, nvm);
        // Update timing for NVM ranks if NVM is configured on this channel
        dram->addRankToRankDelay(cmd_at);
        dram->nextBurstAt = nvm->nextBurstAt;
        dram->nextReqTime = nvm->nextReqTime;
    }

    return cmd_at;
}

bool
HeteroMemCtrl::memBusy(MemInterface* mem_intr) {

    // mem_intr in case of HeteroMemCtrl will always be dram

    // check ranks for refresh/wakeup - uses busStateNext, so done after
    // turnaround decisions
    // Default to busy status and update based on interface specifics
    bool dram_busy, nvm_busy = true;
    // DRAM
    dram_busy = mem_intr->isBusy(false, false);
    // NVM
    bool read_queue_empty = totalReadQueueSize == 0;
    bool all_writes_nvm = nvm->numWritesQueued == totalWriteQueueSize;
    nvm_busy = nvm->isBusy(read_queue_empty, all_writes_nvm);

    // Default state of unused interface is 'true'
    // Simply AND the busy signals to determine if system is busy
    if (dram_busy && nvm_busy) {
        // if all ranks are refreshing wait for them to finish
        // and stall this state machine without taking any further
        // action, and do not schedule a new nextReqEvent
        return true;
    } else {
        return false;
    }
}

void
HeteroMemCtrl::nonDetermReads(MemInterface* mem_intr)
{
    // mem_intr by default points to dram in case
    // of HeteroMemCtrl, therefore, calling nonDetermReads
    // from MemCtrl using nvm interace
    MemCtrl::nonDetermReads(nvm);
}

bool
HeteroMemCtrl::nvmWriteBlock(MemInterface* mem_intr)
{
    // mem_intr by default points to dram in case
    // of HeteroMemCtrl, therefore, calling nvmWriteBlock
    // from MemCtrl using nvm interface
    return MemCtrl::nvmWriteBlock(nvm);
}

Tick
HeteroMemCtrl::minReadToWriteDataGap()
{
    return std::min(dram->minReadToWriteDataGap(),
                    nvm->minReadToWriteDataGap());
}

Tick
HeteroMemCtrl::minWriteToReadDataGap()
{
    return std::min(dram->minWriteToReadDataGap(),
                    nvm->minWriteToReadDataGap());
}

Addr
HeteroMemCtrl::burstAlign(Addr addr, MemInterface* mem_intr) const
{
    // mem_intr will point to dram interface in HeteroMemCtrl
    if (mem_intr->getAddrRange().contains(addr)) {
        return (addr & ~(Addr(mem_intr->bytesPerBurst() - 1)));
    } else {
        assert(nvm->getAddrRange().contains(addr));
        return (addr & ~(Addr(nvm->bytesPerBurst() - 1)));
    }
}

bool
HeteroMemCtrl::pktSizeCheck(MemPacket* mem_pkt, MemInterface* mem_intr) const
{
    // mem_intr will point to dram interface in HeteroMemCtrl
    if (mem_pkt->isDram()) {
        return (mem_pkt->size <= mem_intr->bytesPerBurst());
    } else {
        return (mem_pkt->size <= nvm->bytesPerBurst());
    }
}

void
HeteroMemCtrl::recvFunctional(PacketPtr pkt)
{
    bool found;

    found = MemCtrl::recvFunctionalLogic(pkt, dram);

    if (!found) {
        found = MemCtrl::recvFunctionalLogic(pkt, nvm);
    }

    if (!found) {
        panic("Can't handle address range for packet %s\n", pkt->print());
    }
}

bool
HeteroMemCtrl::allIntfDrained() const
{
    // ensure dram is in power down and refresh IDLE states
    bool dram_drained = dram->allRanksDrained();
    // No outstanding NVM writes
    // All other queues verified as needed with calling logic
    bool nvm_drained = nvm->allRanksDrained();
    return (dram_drained && nvm_drained);
}

DrainState
HeteroMemCtrl::drain()
{
    // if there is anything in any of our internal queues, keep track
    // of that as well
    if (!(!totalWriteQueueSize && !totalReadQueueSize && respQueue.empty() &&
          allIntfDrained())) {

        DPRINTF(Drain, "Memory controller not drained, write: %d, read: %d,"
                " resp: %d\n", totalWriteQueueSize, totalReadQueueSize,
                respQueue.size());

        // the only queue that is not drained automatically over time
        // is the write queue, thus kick things into action if needed
        if (!totalWriteQueueSize && !nextReqEvent.scheduled()) {
            schedule(nextReqEvent, curTick());
        }

        dram->drainRanks();

        return DrainState::Draining;
    } else {
        return DrainState::Drained;
    }
}

void
HeteroMemCtrl::drainResume()
{
    if (!isTimingMode && system()->isTimingMode()) {
        // if we switched to timing mode, kick things into action,
        // and behave as if we restored from a checkpoint
        startup();
        dram->startup();
    } else if (isTimingMode && !system()->isTimingMode()) {
        // if we switch from timing mode, stop the refresh events to
        // not cause issues with KVM
        dram->suspend();
    }

    // update the mode
    isTimingMode = system()->isTimingMode();
}

AddrRangeList
HeteroMemCtrl::getAddrRanges()
{
    AddrRangeList ranges;
    ranges.push_back(dram->getAddrRange());
    ranges.push_back(nvm->getAddrRange());
    return ranges;
}

} // namespace memory
} // namespace gem5
