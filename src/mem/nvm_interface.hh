/*
 * Copyright (c) 2012-2020 ARM Limited
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

/**
 * @file
 * NVMInterface declaration
 */

#ifndef __NVM_INTERFACE_HH__
#define __NVM_INTERFACE_HH__

#include "mem/mem_interface.hh"
#include "params/NVMInterface.hh"

namespace gem5
{

namespace memory
{

/**
 * Interface to NVM devices with media specific parameters,
 * statistics, and functions.
 * The NVMInterface includes a class for individual ranks
 * and per rank functions.
 */
class NVMInterface : public MemInterface
{
  private:
    /**
     * NVM rank class simply includes a vector of banks.
     */
    class Rank : public EventManager
    {
      public:

        /**
         * Current Rank index
         */
        uint8_t rank;

        /**
         * Vector of NVM banks. Each rank is made of several banks
         * that can be accessed in parallel.
         */
        std::vector<Bank> banks;

        Rank(const NVMInterfaceParams &_p, int _rank,
             NVMInterface& _nvm);
    };

    /**
     * NVM specific device and channel characteristics
     */
    const uint32_t maxPendingWrites;
    const uint32_t maxPendingReads;
    const bool twoCycleRdWr;

    /**
     * NVM specific timing requirements
     */
    const Tick tREAD;
    const Tick tWRITE;
    const Tick tSEND;

    struct NVMStats : public statistics::Group
    {
        NVMStats(NVMInterface &nvm);

        void regStats() override;

        NVMInterface &nvm;

        /** NVM stats */
        statistics::Scalar readBursts;
        statistics::Scalar writeBursts;

        statistics::Vector perBankRdBursts;
        statistics::Vector perBankWrBursts;

        // Latencies summed over all requests
        statistics::Scalar totQLat;
        statistics::Scalar totBusLat;
        statistics::Scalar totMemAccLat;

        // Average latencies per request
        statistics::Formula avgQLat;
        statistics::Formula avgBusLat;
        statistics::Formula avgMemAccLat;

        statistics::Scalar nvmBytesRead;
        statistics::Scalar nvmBytesWritten;

        // Average bandwidth
        statistics::Formula avgRdBW;
        statistics::Formula avgWrBW;
        statistics::Formula peakBW;
        statistics::Formula busUtil;
        statistics::Formula busUtilRead;
        statistics::Formula busUtilWrite;

        /** NVM stats */
        statistics::Histogram pendingReads;
        statistics::Histogram pendingWrites;
        statistics::Histogram bytesPerBank;
    };
    NVMStats stats;

    void processWriteRespondEvent();
    EventFunctionWrapper writeRespondEvent;

    void processReadReadyEvent();
    EventFunctionWrapper readReadyEvent;

    /**
      * Vector of nvm ranks
      */
    std::vector<Rank*> ranks;

    /**
     * Holding queue for non-deterministic write commands, which
     * maintains writes that have been issued but have not completed
     * Stored seperately mostly to keep the code clean and help with
     * events scheduling.
     * This mimics a buffer on the media controller and therefore is
     * not added to the main write queue for sizing
     */
    std::list<Tick> writeRespQueue;

    std::deque<Tick> readReadyQueue;

    /**
     * Check if the write response queue is empty
     *
     * @param Return true if empty
     */
    bool writeRespQueueEmpty() const { return writeRespQueue.empty(); }

    /**
     * Till when must we wait before issuing next read command?
     */
    Tick nextReadAt;

    // keep track of reads that have issued for which data is either
    // not yet ready or has not yet been transferred to the ctrl
    uint16_t numPendingReads;
    uint16_t numReadDataReady;

  public:
    // keep track of the number of reads that have yet to be issued
    uint16_t numReadsToIssue;

    /**
     * Initialize the NVM interface and verify parameters
     */
    void init() override;

    /**
     * Setup the rank based on packet received
     *
     * @param integer value of rank to be setup. used to index ranks vector
     * @param are we setting up rank for read or write packet?
     */
    void setupRank(const uint8_t rank, const bool is_read) override;

    MemPacket* decodePacket(const PacketPtr pkt, Addr pkt_addr,
                           unsigned int size, bool is_read,
                           uint8_t pseudo_channel = 0) override;

    /**
     * Check drain state of NVM interface
     *
     * @return true if write response queue is empty
     *
     */
    bool allRanksDrained() const override { return writeRespQueueEmpty(); }

    /*
     * @return time to offset next command
     */
    Tick commandOffset() const override { return tBURST; }

    /**
     * Check if a burst operation can be issued to the NVM
     *
     * @param Return true if RD/WR can issue
     *                    for reads, also verfy that ready count
     *                    has been updated to a non-zero value to
     *                    account for race conditions between events
     */
    bool burstReady(MemPacket* pkt) const override;

    /**
     * This function checks if ranks are busy.
     * This state is true when either:
     * 1) There is no command with read data ready to transmit or
     * 2) The NVM inteface has reached the maximum number of outstanding
     *    writes commands.
     * @param read_queue_empty There are no read queued
     * @param all_writes_nvm   All writes in queue are for NVM interface
     * @return true of NVM is busy
     *
     */
    bool isBusy(bool read_queue_empty, bool all_writes_nvm) override;
    /**
     * For FR-FCFS policy, find first NVM command that can issue
     * default to first command to prepped region
     *
     * @param queue Queued requests to consider
     * @param min_col_at Minimum tick for 'seamless' issue
     * @return an iterator to the selected packet, else queue.end()
     * @return the tick when the packet selected will issue
     */
    std::pair<MemPacketQueue::iterator, Tick>
    chooseNextFRFCFS(MemPacketQueue& queue, Tick min_col_at) const override;

    /**
     *  Add rank to rank delay to bus timing to all NVM banks in alli ranks
     *  when access to an alternate interface is issued
     *
     *  param cmd_at Time of current command used as starting point for
     *               addition of rank-to-rank delay
     */
    void addRankToRankDelay(Tick cmd_at) override;

    /**
     * Following two functions are not required for nvm interface
     */
    void respondEvent(uint8_t rank) override { };

    void checkRefreshState(uint8_t rank) override { };

    /**
     * Select read command to issue asynchronously
     */
    void chooseRead(MemPacketQueue& queue) override;

    /*
     * Function to calulate unloaded access latency
     */
    Tick accessLatency() const override { return (tREAD + tSEND); }

    /**
     * Check if the write response queue has reached defined threshold
     *
     * @param Return true if full
     */
    bool
    writeRespQueueFull() const override
    {
        return writeRespQueue.size() == maxPendingWrites;
    }

    bool
    readsWaitingToIssue() const override
    {
        return ((numReadsToIssue != 0) &&
                (numPendingReads < maxPendingReads));
    }

    /**
     * Actually do the burst and update stats.
     *
     * @param pkt The packet created from the outside world pkt
     * @param next_burst_at Minimum bus timing requirement from controller
     * @return pair, tick when current burst is issued and
     *               tick when next burst can issue
     */
    std::pair<Tick, Tick>
    doBurstAccess(MemPacket* pkt, Tick next_burst_at,
                  const std::vector<MemPacketQueue>& queue) override;

    /**
     * The next three functions are DRAM-specific and will be ignored by NVM.
     */
    void drainRanks() override { }
    void suspend() override { }
    void startup() override { }

    NVMInterface(const NVMInterfaceParams &_p);
};

} // namespace memory
} // namespace gem5

#endif //__NVM_INTERFACE_HH__
