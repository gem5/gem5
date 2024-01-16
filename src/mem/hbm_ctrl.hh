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

/**
 * @file
 * HBMCtrl declaration
 */

#ifndef __HBM_CTRL_HH__
#define __HBM_CTRL_HH__

#include <deque>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "mem/mem_ctrl.hh"
#include "params/HBMCtrl.hh"

namespace gem5
{

namespace memory
{

class MemInterface;
class DRAMInterface;


/**
 * HBM2 is divided into two pseudo channels which have independent data buses
 * but share a command bus (separate row and column command bus). Therefore,
 * the HBM memory controller should be able to control both pseudo channels.
 * This HBM memory controller inherits from gem5's default
 * memory controller (pseudo channel 0) and manages the additional HBM pseudo
 * channel (pseudo channel 1).
 */
class HBMCtrl : public MemCtrl
{

  protected:

    bool respQEmpty() override
    {
        return (respQueue.empty() && respQueuePC1.empty());
    }

  private:

    /**
     * Remember if we have to retry a request for second pseudo channel.
     */
    bool retryRdReqPC1;
    bool retryWrReqPC1;

    /**
     * Remove commands that have already issued from rowBurstTicks
     * and colBurstTicks
     */
    void pruneBurstTick() override;

    AddrRangeList getAddrRanges() override;

  public:
    HBMCtrl(const HBMCtrlParams &p);

    void pruneRowBurstTick();
    void pruneColBurstTick();

    /**
     * Check for command bus contention for single cycle command.
     * If there is contention, shift command to next burst.
     * Check verifies that the commands issued per burst is less
     * than a defined max number, maxCommandsPerWindow.
     * Therefore, contention per cycle is not verified and instead
     * is done based on a burst window.
     *
     * @param cmd_tick Initial tick of command, to be verified
     * @param max_cmds_per_burst Number of commands that can issue
     *                           in a burst window
     * @return tick for command issue without contention
     */
    Tick verifySingleCmd(Tick cmd_tick, Tick max_cmds_per_burst,
                        bool row_cmd) override;

    /**
     * Check for command bus contention for multi-cycle (2 currently)
     * command. If there is contention, shift command(s) to next burst.
     * Check verifies that the commands issued per burst is less
     * than a defined max number, maxCommandsPerWindow.
     * Therefore, contention per cycle is not verified and instead
     * is done based on a burst window.
     * For HBM2, only row cmds (activate) can be multi-cycle
     *
     * @param cmd_tick Initial tick of command, to be verified
     * @param max_multi_cmd_split Maximum delay between commands
     * @param max_cmds_per_burst Number of commands that can issue
     *                           in a burst window
     * @return tick for command issue without contention
     */
    Tick verifyMultiCmd(Tick cmd_tick, Tick max_cmds_per_burst,
                        Tick max_multi_cmd_split = 0) override;

    /**
     * NextReq and Respond events for second pseudo channel
     *
     */
    EventFunctionWrapper nextReqEventPC1;
    EventFunctionWrapper respondEventPC1;

    /**
     * Check if the read queue partition of both pseudo
     * channels has room for more entries. This is used when the HBM ctrl
     * is run with partitioned queues
     *
     * @param pkt_count The number of entries needed in the read queue
     * @return true if read queue partition is full, false otherwise
     */
    bool readQueueFullPC0(unsigned int pkt_count) const;
    bool readQueueFullPC1(unsigned int pkt_count) const;

    /**
     * Check if the write queue partition of both pseudo
     * channels has room for more entries. This is used when the HBM ctrl
     * is run with partitioned queues
     *
     * @param pkt_count The number of entries needed in the write queue
     * @return true if write queue is full, false otherwise
     */
    bool writeQueueFullPC0(unsigned int pkt_count) const;
    bool writeQueueFullPC1(unsigned int pkt_count) const;

    /**
     * Following counters are used to keep track of the entries in read/write
     * queue for each pseudo channel (useful when the partitioned queues are
     * used)
     */
    uint64_t readQueueSizePC0 = 0;
    uint64_t readQueueSizePC1 = 0;
    uint64_t writeQueueSizePC0 = 0;
    uint64_t writeQueueSizePC1 = 0;

    /**
     * Response queue for pkts sent to second pseudo channel
     * The first pseudo channel uses MemCtrl::respQueue
     */
    std::deque<MemPacket*> respQueuePC1;

    /**
     * Holds count of row commands issued in burst window starting at
     * defined Tick. This is used to ensure that the row command bandwidth
     * does not exceed the allowable media constraints.
     */
    std::unordered_multiset<Tick> rowBurstTicks;

    /**
     * This is used to ensure that the column command bandwidth
     * does not exceed the allowable media constraints. HBM2 has separate
     * command bus for row and column commands
     */
    std::unordered_multiset<Tick> colBurstTicks;

    /**
     * Pointers to interfaces of the two pseudo channels
     * pc0Int is same as MemCtrl::dram (it will be pointing to
     * the DRAM interface defined in base MemCtrl)
     */
    DRAMInterface* pc0Int;
    DRAMInterface* pc1Int;

    /**
     * This indicates if the R/W queues will be partitioned among
     * pseudo channels
     */
    bool partitionedQ;

  public:

    /**
     * Is there a respondEvent scheduled?
     *
     * @return true if event is scheduled
     */
    bool respondEventScheduled(uint8_t pseudo_channel) const override
    {
        if (pseudo_channel == 0) {
            return MemCtrl::respondEventScheduled(pseudo_channel);
        } else {
            assert(pseudo_channel == 1);
            return respondEventPC1.scheduled();
        }
    }

    /**
     * Is there a read/write burst Event scheduled?
     *
     * @return true if event is scheduled
     */
    bool requestEventScheduled(uint8_t pseudo_channel) const override
    {
        if (pseudo_channel == 0) {
            return MemCtrl::requestEventScheduled(pseudo_channel);
        } else {
            assert(pseudo_channel == 1);
            return nextReqEventPC1.scheduled();
        }
    }

    /**
     * restart the controller scheduler
     *
     * @param Tick to schedule next event
     * @param pseudo_channel pseudo channel number for which scheduler
     * needs to restart
     */
    void restartScheduler(Tick tick, uint8_t pseudo_channel) override
    {
        if (pseudo_channel == 0) {
            MemCtrl::restartScheduler(tick);
        } else {
            schedule(nextReqEventPC1, tick);
        }
    }


    virtual void init() override;
    virtual void startup() override;
    virtual void drainResume() override;


  protected:
    Tick recvAtomic(PacketPtr pkt) override;
    Tick recvAtomicBackdoor(PacketPtr pkt, MemBackdoorPtr &backdoor) override;
    void recvFunctional(PacketPtr pkt) override;
    bool recvTimingReq(PacketPtr pkt) override;

};

} // namespace memory
} // namespace gem5

#endif //__HBM_CTRL_HH__
