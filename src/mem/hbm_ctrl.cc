/*
 * Copyright (c) 2022 The Regents of the University of California
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

#include "mem/hbm_ctrl.hh"

#include "base/trace.hh"
#include "debug/DRAM.hh"
#include "debug/Drain.hh"
#include "debug/MemCtrl.hh"
#include "debug/QOS.hh"
#include "mem/dram_interface.hh"
#include "mem/mem_interface.hh"
#include "sim/system.hh"

namespace gem5
{

namespace memory
{

HBMCtrl::HBMCtrl(const HBMCtrlParams &p)
    : MemCtrl(p),
      retryRdReqPC1(false),
      retryWrReqPC1(false),
      nextReqEventPC1(
          [this] {
              processNextReqEvent(pc1Int, respQueuePC1, respondEventPC1,
                                  nextReqEventPC1, retryWrReqPC1);
          },
          name()),
      respondEventPC1(
          [this] {
              processRespondEvent(pc1Int, respQueuePC1, respondEventPC1,
                                  retryRdReqPC1);
          },
          name()),
      pc1Int(p.dram_2)
{
    DPRINTF(MemCtrl, "Setting up HBM controller\n");

    pc0Int = dynamic_cast<DRAMInterface *>(dram);

    assert(dynamic_cast<DRAMInterface *>(p.dram_2) != nullptr);

    readBufferSize = pc0Int->readBufferSize + pc1Int->readBufferSize;
    writeBufferSize = pc0Int->writeBufferSize + pc1Int->writeBufferSize;

    fatal_if(!pc0Int, "Memory controller must have pc0 interface");
    fatal_if(!pc1Int, "Memory controller must have pc1 interface");

    pc0Int->setCtrl(this, commandWindow, 0);
    pc1Int->setCtrl(this, commandWindow, 1);

    writeHighThreshold =
        (writeBufferSize / 2 * p.write_high_thresh_perc) / 100.0;
    writeLowThreshold =
        (writeBufferSize / 2 * p.write_low_thresh_perc) / 100.0;
}

void
HBMCtrl::init()
{
    MemCtrl::init();
}

void
HBMCtrl::startup()
{
    MemCtrl::startup();

    isTimingMode = system()->isTimingMode();
    if (isTimingMode) {
        // shift the bus busy time sufficiently far ahead that we never
        // have to worry about negative values when computing the time for
        // the next request, this will add an insignificant bubble at the
        // start of simulation
        pc1Int->nextBurstAt = curTick() + pc1Int->commandOffset();
    }
}

Tick
HBMCtrl::recvAtomic(PacketPtr pkt)
{
    Tick latency = 0;

    if (pc0Int->getAddrRange().contains(pkt->getAddr())) {
        latency = recvAtomicLogic(pkt, pc0Int);
    } else if (pc1Int->getAddrRange().contains(pkt->getAddr())) {
        latency = recvAtomicLogic(pkt, pc1Int);
    } else {
        panic("Can't handle address range for packet %s\n", pkt->print());
    }

    return latency;
}

void
HBMCtrl::recvFunctional(PacketPtr pkt)
{
    bool found = recvFunctionalLogic(pkt, pc0Int);

    if (!found) {
        found = recvFunctionalLogic(pkt, pc1Int);
    }

    if (!found) {
        panic("Can't handle address range for packet %s\n", pkt->print());
    }
}

Tick
HBMCtrl::recvAtomicBackdoor(PacketPtr pkt, MemBackdoorPtr &backdoor)
{
    Tick latency = recvAtomic(pkt);

    if (pc0Int && pc0Int->getAddrRange().contains(pkt->getAddr())) {
        pc0Int->getBackdoor(backdoor);
    } else if (pc1Int && pc1Int->getAddrRange().contains(pkt->getAddr())) {
        pc1Int->getBackdoor(backdoor);
    } else {
        panic("Can't handle address range for packet %s\n", pkt->print());
    }
    return latency;
}

void
HBMCtrl::recvMemBackdoorReq(const MemBackdoorReq &req,
                            MemBackdoorPtr &backdoor)
{
    auto &range = req.range();
    if (pc0Int && pc0Int->getAddrRange().isSubset(range)) {
        pc0Int->getBackdoor(backdoor);
    } else if (pc1Int && pc1Int->getAddrRange().isSubset(range)) {
        pc1Int->getBackdoor(backdoor);
    } else {
        panic("Can't handle address range for range %s\n", range.to_string());
    }
}

bool
HBMCtrl::writeQueueFullPC0(unsigned int neededEntries) const
{
    DPRINTF(MemCtrl, "Write queue limit %d, PC0 size %d, entries needed %d\n",
            writeBufferSize / 2, pc0Int->writeQueueSize, neededEntries);

    unsigned int wrsize_new = (pc0Int->writeQueueSize + neededEntries);
    return wrsize_new > (writeBufferSize / 2);
}

bool
HBMCtrl::writeQueueFullPC1(unsigned int neededEntries) const
{
    DPRINTF(MemCtrl, "Write queue limit %d, PC1 size %d, entries needed %d\n",
            writeBufferSize / 2, pc1Int->writeQueueSize, neededEntries);

    unsigned int wrsize_new = (pc1Int->writeQueueSize + neededEntries);
    return wrsize_new > (writeBufferSize / 2);
}

bool
HBMCtrl::readQueueFullPC0(unsigned int neededEntries) const
{
    DPRINTF(MemCtrl, "Read queue limit %d, PC0 size %d, entries needed %d\n",
            readBufferSize / 2, pc0Int->readQueueSize + respQueue.size(),
            neededEntries);

    unsigned int rdsize_new =
        pc0Int->readQueueSize + respQueue.size() + neededEntries;
    return rdsize_new > (readBufferSize / 2);
}

bool
HBMCtrl::readQueueFullPC1(unsigned int neededEntries) const
{
    DPRINTF(MemCtrl, "Read queue limit %d, PC1 size %d, entries needed %d\n",
            readBufferSize / 2, pc1Int->readQueueSize + respQueuePC1.size(),
            neededEntries);

    unsigned int rdsize_new =
        pc1Int->readQueueSize + respQueuePC1.size() + neededEntries;
    return rdsize_new > (readBufferSize / 2);
}

bool
HBMCtrl::recvTimingReq(PacketPtr pkt)
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
    bool is_pc0;

    // TODO: make the interleaving bit across pseudo channels a parameter
    if (bits(pkt->getAddr(), 6) == 0) {
        is_pc0 = true;
    } else {
        is_pc0 = false;
    }

    // Find out how many memory packets a pkt translates to
    // If the burst size is equal or larger than the pkt size, then a pkt
    // translates to only one memory packet. Otherwise, a pkt translates to
    // multiple memory packets
    unsigned size = pkt->getSize();
    uint32_t burst_size = pc0Int->bytesPerBurst();
    unsigned offset = pkt->getAddr() & (burst_size - 1);
    unsigned int pkt_count = divCeil(offset + size, burst_size);

    // run the QoS scheduler and assign a QoS priority value to the packet
    qosSchedule({ &readQueue, &writeQueue }, burst_size, pkt);

    // check local buffers and do not accept if full
    if (pkt->isWrite()) {
        if (is_pc0) {
            if (writeQueueFullPC0(pkt_count)) {
                DPRINTF(MemCtrl, "Write queue full, not accepting\n");
                // remember that we have to retry this port
                retryWrReq = true;
                stats.numWrRetry++;
                return false;
            } else {
                addToWriteQueue(pkt, pkt_count, pc0Int);
                if (!nextReqEvent.scheduled()) {
                    DPRINTF(MemCtrl, "Request scheduled immediately\n");
                    schedule(nextReqEvent, curTick());
                }
                stats.writeReqs++;
                stats.bytesWrittenSys += size;
            }
        } else {
            if (writeQueueFullPC1(pkt_count)) {
                DPRINTF(MemCtrl, "Write queue full, not accepting\n");
                // remember that we have to retry this port
                retryWrReqPC1 = true;
                stats.numWrRetry++;
                return false;
            } else {
                addToWriteQueue(pkt, pkt_count, pc1Int);
                if (!nextReqEventPC1.scheduled()) {
                    DPRINTF(MemCtrl, "Request scheduled immediately\n");
                    schedule(nextReqEventPC1, curTick());
                }
                stats.writeReqs++;
                stats.bytesWrittenSys += size;
            }
        }
    } else {
        assert(pkt->isRead());
        assert(size != 0);

        if (is_pc0) {
            if (readQueueFullPC0(pkt_count)) {
                DPRINTF(MemCtrl, "Read queue full, not accepting\n");
                // remember that we have to retry this port
                retryRdReq = true;
                stats.numRdRetry++;
                return false;
            } else {
                if (!addToReadQueue(pkt, pkt_count, pc0Int)) {
                    if (!nextReqEvent.scheduled()) {
                        DPRINTF(MemCtrl, "Request scheduled immediately\n");
                        schedule(nextReqEvent, curTick());
                    }
                }

                stats.readReqs++;
                stats.bytesReadSys += size;
            }
        } else {
            if (readQueueFullPC1(pkt_count)) {
                DPRINTF(MemCtrl, "Read queue full, not accepting\n");
                // remember that we have to retry this port
                retryRdReqPC1 = true;
                stats.numRdRetry++;
                return false;
            } else {
                if (!addToReadQueue(pkt, pkt_count, pc1Int)) {
                    if (!nextReqEventPC1.scheduled()) {
                        DPRINTF(MemCtrl, "Request scheduled immediately\n");
                        schedule(nextReqEventPC1, curTick());
                    }
                }
                stats.readReqs++;
                stats.bytesReadSys += size;
            }
        }
    }

    return true;
}

void
HBMCtrl::pruneRowBurstTick()
{
    auto it = rowBurstTicks.begin();
    while (it != rowBurstTicks.end()) {
        auto current_it = it++;
        if (getBurstWindow(curTick()) > *current_it) {
            DPRINTF(MemCtrl, "Removing burstTick for %d\n", *current_it);
            rowBurstTicks.erase(current_it);
        }
    }
}

void
HBMCtrl::pruneColBurstTick()
{
    auto it = colBurstTicks.begin();
    while (it != colBurstTicks.end()) {
        auto current_it = it++;
        if (getBurstWindow(curTick()) > *current_it) {
            DPRINTF(MemCtrl, "Removing burstTick for %d\n", *current_it);
            colBurstTicks.erase(current_it);
        }
    }
}

void
HBMCtrl::pruneBurstTick()
{
    pruneRowBurstTick();
    pruneColBurstTick();
}

Tick
HBMCtrl::verifySingleCmd(Tick cmd_tick, Tick max_cmds_per_burst, bool row_cmd)
{
    // start with assumption that there is no contention on command bus
    Tick cmd_at = cmd_tick;

    // get tick aligned to burst window
    Tick burst_tick = getBurstWindow(cmd_tick);

    // verify that we have command bandwidth to issue the command
    // if not, iterate over next window(s) until slot found

    if (row_cmd) {
        while (rowBurstTicks.count(burst_tick) >= max_cmds_per_burst) {
            DPRINTF(MemCtrl, "Contention found on row command bus at %d\n",
                    burst_tick);
            burst_tick += commandWindow;
            cmd_at = burst_tick;
        }
        DPRINTF(MemCtrl, "Now can send a row cmd_at %d\n", cmd_at);
        rowBurstTicks.insert(burst_tick);

    } else {
        while (colBurstTicks.count(burst_tick) >= max_cmds_per_burst) {
            DPRINTF(MemCtrl, "Contention found on col command bus at %d\n",
                    burst_tick);
            burst_tick += commandWindow;
            cmd_at = burst_tick;
        }
        DPRINTF(MemCtrl, "Now can send a col cmd_at %d\n", cmd_at);
        colBurstTicks.insert(burst_tick);
    }
    return cmd_at;
}

Tick
HBMCtrl::verifyMultiCmd(Tick cmd_tick, Tick max_cmds_per_burst,
                        Tick max_multi_cmd_split)
{
    // start with assumption that there is no contention on command bus
    Tick cmd_at = cmd_tick;

    // get tick aligned to burst window
    Tick burst_tick = getBurstWindow(cmd_tick);

    // Command timing requirements are from 2nd command
    // Start with assumption that 2nd command will issue at cmd_at and
    // find prior slot for 1st command to issue
    // Given a maximum latency of max_multi_cmd_split between the commands,
    // find the burst at the maximum latency prior to cmd_at
    Tick burst_offset = 0;
    Tick first_cmd_offset = cmd_tick % commandWindow;
    while (max_multi_cmd_split > (first_cmd_offset + burst_offset)) {
        burst_offset += commandWindow;
    }
    // get the earliest burst aligned address for first command
    // ensure that the time does not go negative
    Tick first_cmd_tick = burst_tick - std::min(burst_offset, burst_tick);

    // Can required commands issue?
    bool first_can_issue = false;
    bool second_can_issue = false;
    // verify that we have command bandwidth to issue the command(s)
    while (!first_can_issue || !second_can_issue) {
        bool same_burst = (burst_tick == first_cmd_tick);
        auto first_cmd_count = rowBurstTicks.count(first_cmd_tick);
        auto second_cmd_count =
            same_burst ? first_cmd_count + 1 : rowBurstTicks.count(burst_tick);

        first_can_issue = first_cmd_count < max_cmds_per_burst;
        second_can_issue = second_cmd_count < max_cmds_per_burst;

        if (!second_can_issue) {
            DPRINTF(MemCtrl, "Contention (cmd2) found on command bus at %d\n",
                    burst_tick);
            burst_tick += commandWindow;
            cmd_at = burst_tick;
        }

        // Verify max_multi_cmd_split isn't violated when command 2 is shifted
        // If commands initially were issued in same burst, they are
        // now in consecutive bursts and can still issue B2B
        bool gap_violated = !same_burst && ((burst_tick - first_cmd_tick) >
                                            max_multi_cmd_split);

        if (!first_can_issue || (!second_can_issue && gap_violated)) {
            DPRINTF(MemCtrl, "Contention (cmd1) found on command bus at %d\n",
                    first_cmd_tick);
            first_cmd_tick += commandWindow;
        }
    }

    // Add command to burstTicks
    rowBurstTicks.insert(burst_tick);
    rowBurstTicks.insert(first_cmd_tick);

    return cmd_at;
}

void
HBMCtrl::drainResume()
{
    MemCtrl::drainResume();

    if (!isTimingMode && system()->isTimingMode()) {
        // if we switched to timing mode, kick things into action,
        // and behave as if we restored from a checkpoint
        startup();
        pc1Int->startup();
    } else if (isTimingMode && !system()->isTimingMode()) {
        // if we switch from timing mode, stop the refresh events to
        // not cause issues with KVM
        if (pc1Int) {
            pc1Int->drainRanks();
        }
    }

    // update the mode
    isTimingMode = system()->isTimingMode();
}

AddrRangeList
HBMCtrl::getAddrRanges()
{
    AddrRangeList ranges;
    ranges.push_back(pc0Int->getAddrRange());
    ranges.push_back(pc1Int->getAddrRange());
    return ranges;
}

} // namespace memory
} // namespace gem5
