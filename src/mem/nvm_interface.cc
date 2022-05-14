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

#include "mem/nvm_interface.hh"

#include "base/bitfield.hh"
#include "base/cprintf.hh"
#include "base/trace.hh"
#include "debug/NVM.hh"
#include "sim/system.hh"

namespace gem5
{

namespace memory
{

NVMInterface::NVMInterface(const NVMInterfaceParams &_p)
    : MemInterface(_p),
      maxPendingWrites(_p.max_pending_writes),
      maxPendingReads(_p.max_pending_reads),
      twoCycleRdWr(_p.two_cycle_rdwr),
      tREAD(_p.tREAD), tWRITE(_p.tWRITE), tSEND(_p.tSEND),
      stats(*this),
      writeRespondEvent([this]{ processWriteRespondEvent(); }, name()),
      readReadyEvent([this]{ processReadReadyEvent(); }, name()),
      nextReadAt(0), numPendingReads(0), numReadDataReady(0),
      numReadsToIssue(0)
{
    DPRINTF(NVM, "Setting up NVM Interface\n");

    fatal_if(!isPowerOf2(burstSize), "NVM burst size %d is not allowed, "
             "must be a power of two\n", burstSize);

    // sanity check the ranks since we rely on bit slicing for the
    // address decoding
    fatal_if(!isPowerOf2(ranksPerChannel), "NVM rank count of %d is "
             "not allowed, must be a power of two\n", ranksPerChannel);

    for (int i =0; i < ranksPerChannel; i++) {
        // Add NVM ranks to the system
        DPRINTF(NVM, "Creating NVM rank %d \n", i);
        Rank* rank = new Rank(_p, i, *this);
        ranks.push_back(rank);
    }

    uint64_t capacity = 1ULL << ceilLog2(AbstractMemory::size());

    DPRINTF(NVM, "NVM capacity %lld (%lld) bytes\n", capacity,
            AbstractMemory::size());

    rowsPerBank = capacity / (rowBufferSize *
                    banksPerRank * ranksPerChannel);
}

NVMInterface::Rank::Rank(const NVMInterfaceParams &_p,
                         int _rank, NVMInterface& _nvm)
    : EventManager(&_nvm), rank(_rank), banks(_p.banks_per_rank)
{
    for (int b = 0; b < _p.banks_per_rank; b++) {
        banks[b].bank = b;
        // No bank groups; simply assign to bank number
        banks[b].bankgr = b;
    }
}

void
NVMInterface::init()
{
    AbstractMemory::init();
}

void NVMInterface::setupRank(const uint8_t rank, const bool is_read)
{
    if (is_read) {
        // increment count to trigger read and track number of reads in Q
        numReadsToIssue++;
    } else {
        // increment count to track number of writes in Q
        numWritesQueued++;
    }
}

MemPacket*
NVMInterface::decodePacket(const PacketPtr pkt, Addr pkt_addr,
                       unsigned size, bool is_read, uint8_t pseudo_channel)
{
    // decode the address based on the address mapping scheme, with
    // Ro, Ra, Co, Ba and Ch denoting row, rank, column, bank and
    // channel, respectively
    uint8_t rank;
    uint8_t bank;
    // use a 64-bit unsigned during the computations as the row is
    // always the top bits, and check before creating the packet
    uint64_t row;

    // Get packed address, starting at 0
    Addr addr = getCtrlAddr(pkt_addr);

    // truncate the address to a memory burst, which makes it unique to
    // a specific buffer, row, bank, rank and channel
    addr = addr / burstSize;

    // we have removed the lowest order address bits that denote the
    // position within the column
    if (addrMapping == enums::RoRaBaChCo || addrMapping == enums::RoRaBaCoCh) {
        // the lowest order bits denote the column to ensure that
        // sequential cache lines occupy the same row
        addr = addr / burstsPerRowBuffer;

        // after the channel bits, get the bank bits to interleave
        // over the banks
        bank = addr % banksPerRank;
        addr = addr / banksPerRank;

        // after the bank, we get the rank bits which thus interleaves
        // over the ranks
        rank = addr % ranksPerChannel;
        addr = addr / ranksPerChannel;

        // lastly, get the row bits, no need to remove them from addr
        row = addr % rowsPerBank;
    } else if (addrMapping == enums::RoCoRaBaCh) {
        // with emerging technologies, could have small page size with
        // interleaving granularity greater than row buffer
        if (burstsPerStripe > burstsPerRowBuffer) {
            // remove column bits which are a subset of burstsPerStripe
            addr = addr / burstsPerRowBuffer;
        } else {
            // remove lower column bits below channel bits
            addr = addr / burstsPerStripe;
        }

        // start with the bank bits, as this provides the maximum
        // opportunity for parallelism between requests
        bank = addr % banksPerRank;
        addr = addr / banksPerRank;

        // next get the rank bits
        rank = addr % ranksPerChannel;
        addr = addr / ranksPerChannel;

        // next, the higher-order column bites
        if (burstsPerStripe < burstsPerRowBuffer) {
            addr = addr / (burstsPerRowBuffer / burstsPerStripe);
        }

        // lastly, get the row bits, no need to remove them from addr
        row = addr % rowsPerBank;
    } else
        panic("Unknown address mapping policy chosen!");

    assert(rank < ranksPerChannel);
    assert(bank < banksPerRank);
    assert(row < rowsPerBank);
    assert(row < Bank::NO_ROW);

    DPRINTF(NVM, "Address: %#x Rank %d Bank %d Row %d\n",
            pkt_addr, rank, bank, row);

    // create the corresponding memory packet with the entry time and
    // ready time set to the current tick, the latter will be updated
    // later
    uint16_t bank_id = banksPerRank * rank + bank;

    return new MemPacket(pkt, is_read, false, pseudo_channel, rank, bank, row,
                   bank_id, pkt_addr, size);
}

std::pair<MemPacketQueue::iterator, Tick>
NVMInterface::chooseNextFRFCFS(MemPacketQueue& queue, Tick min_col_at) const
{
    // remember if we found a hit, but one that cannit issue seamlessly
    bool found_prepped_pkt = false;

    auto selected_pkt_it = queue.end();
    Tick selected_col_at = MaxTick;

    for (auto i = queue.begin(); i != queue.end() ; ++i) {
        MemPacket* pkt = *i;

        // select optimal NVM packet in Q
        if (!pkt->isDram()) {
            const Bank& bank = ranks[pkt->rank]->banks[pkt->bank];
            const Tick col_allowed_at = pkt->isRead() ? bank.rdAllowedAt :
                                                        bank.wrAllowedAt;

            // check if rank is not doing a refresh and thus is available,
            // if not, jump to the next packet
            if (burstReady(pkt)) {
                DPRINTF(NVM, "%s bank %d - Rank %d available\n", __func__,
                        pkt->bank, pkt->rank);

                // no additional rank-to-rank or media delays
                if (col_allowed_at <= min_col_at) {
                    // FCFS within entries that can issue without
                    // additional delay, such as same rank accesses
                    // or media delay requirements
                    selected_pkt_it = i;
                    selected_col_at = col_allowed_at;
                    // no need to look through the remaining queue entries
                    DPRINTF(NVM, "%s Seamless buffer hit\n", __func__);
                    break;
                } else if (!found_prepped_pkt) {
                    // packet is to prepped region but cannnot issue
                    // seamlessly; remember this one and continue
                    selected_pkt_it = i;
                    selected_col_at = col_allowed_at;
                    DPRINTF(NVM, "%s Prepped packet found \n", __func__);
                    found_prepped_pkt = true;
                }
            } else {
                DPRINTF(NVM, "%s bank %d - Rank %d not available\n", __func__,
                        pkt->bank, pkt->rank);
            }
        }
    }

    if (selected_pkt_it == queue.end()) {
        DPRINTF(NVM, "%s no available NVM ranks found\n", __func__);
    }

    return std::make_pair(selected_pkt_it, selected_col_at);
}

void
NVMInterface::chooseRead(MemPacketQueue& queue)
{
    Tick cmd_at = std::max(curTick(), nextReadAt);

    // This method does the arbitration between non-deterministic read
    // requests to NVM. The chosen packet is not removed from the queue
    // at this time. Removal from the queue will occur when the data is
    // ready and a separate SEND command is issued to retrieve it via the
    // chooseNext function in the top-level controller.
    assert(!queue.empty());

    assert(numReadsToIssue > 0);
    numReadsToIssue--;
    // For simplicity, issue non-deterministic reads in order (fcfs)
    for (auto i = queue.begin(); i != queue.end() ; ++i) {
        MemPacket* pkt = *i;

        // Find 1st NVM read packet that hasn't issued read command
        if (pkt->readyTime == MaxTick && !pkt->isDram() && pkt->isRead()) {
           // get the bank
           Bank& bank_ref = ranks[pkt->rank]->banks[pkt->bank];

            // issueing a read, inc counter and verify we haven't overrun
            numPendingReads++;
            assert(numPendingReads <= maxPendingReads);

            // increment the bytes accessed and the accesses per row
            bank_ref.bytesAccessed += burstSize;

            // Verify command bandiwth to issue
            // Host can issue read immediately uith buffering closer
            // to the NVM. The actual execution at the NVM may be delayed
            // due to busy resources
            if (twoCycleRdWr) {
                cmd_at = ctrl->verifyMultiCmd(cmd_at,
                                              maxCommandsPerWindow, tCK);
            } else {
                cmd_at = ctrl->verifySingleCmd(cmd_at,
                                              maxCommandsPerWindow, false);
            }

            // Update delay to next read
            // Ensures single read command issued per cycle
            nextReadAt = cmd_at + tCK;

            // If accessing a new location in this bank, update timing
            // and stats
            if (bank_ref.openRow != pkt->row) {
                // update the open bank, re-using row field
                bank_ref.openRow = pkt->row;

                // sample the bytes accessed to a buffer in this bank
                // here when we are re-buffering the data
                stats.bytesPerBank.sample(bank_ref.bytesAccessed);
                // start counting anew
                bank_ref.bytesAccessed = 0;

                // holdoff next command to this bank until the read completes
                // and the data has been successfully buffered
                // can pipeline accesses to the same bank, sending them
                // across the interface B2B, but will incur full access
                // delay between data ready responses to different buffers
                // in a bank
                bank_ref.actAllowedAt = std::max(cmd_at,
                                        bank_ref.actAllowedAt) + tREAD;
            }
            // update per packet readyTime to holdoff burst read operation
            // overloading readyTime, which will be updated again when the
            // burst is issued
            pkt->readyTime = std::max(cmd_at, bank_ref.actAllowedAt);

            DPRINTF(NVM, "Issuing NVM Read to bank %d at tick %d. "
                         "Data ready at %d\n",
                         bank_ref.bank, cmd_at, pkt->readyTime);

            // Insert into read ready queue. It will be handled after
            // the media delay has been met
            if (readReadyQueue.empty()) {
                assert(!readReadyEvent.scheduled());
                schedule(readReadyEvent, pkt->readyTime);
            } else if (readReadyEvent.when() > pkt->readyTime) {
                // move it sooner in time, to the first read with data
                reschedule(readReadyEvent, pkt->readyTime);
            } else {
                assert(readReadyEvent.scheduled());
            }
            readReadyQueue.push_back(pkt->readyTime);

            // found an NVM read to issue - break out
            break;
        }
    }
}

void
NVMInterface::processReadReadyEvent()
{
    // signal that there is read data ready to be transmitted
    numReadDataReady++;

    DPRINTF(NVM,
            "processReadReadyEvent(): Data for an NVM read is ready. "
            "numReadDataReady is %d\t numPendingReads is %d\n",
             numReadDataReady, numPendingReads);

    // Find lowest ready time and verify it is equal to curTick
    // also find the next lowest to schedule next event
    // Done with this response, erase entry
    auto ready_it = readReadyQueue.begin();
    Tick next_ready_at = MaxTick;
    for (auto i = readReadyQueue.begin(); i != readReadyQueue.end() ; ++i) {
        if (*ready_it > *i) {
            next_ready_at = *ready_it;
            ready_it = i;
        } else if ((next_ready_at > *i) && (i != ready_it)) {
            next_ready_at = *i;
        }
    }

    // Verify we found the time of this event and remove it
    assert(*ready_it == curTick());
    readReadyQueue.erase(ready_it);

    if (!readReadyQueue.empty()) {
        assert(readReadyQueue.front() >= curTick());
        assert(!readReadyEvent.scheduled());
        schedule(readReadyEvent, next_ready_at);
    }

    // It is possible that a new command kicks things back into
    // action before reaching this point but need to ensure that we
    // continue to process new commands as read data becomes ready
    // This will also trigger a drain if needed
    if (!ctrl->requestEventScheduled()) {
        DPRINTF(NVM, "Restart controller scheduler immediately\n");
        ctrl->restartScheduler(curTick());
    }
}

bool
NVMInterface::burstReady(MemPacket* pkt) const {
    bool read_rdy =  pkt->isRead() && (ctrl->inReadBusState(true)) &&
               (pkt->readyTime <= curTick()) && (numReadDataReady > 0);
    bool write_rdy =  !pkt->isRead() && !ctrl->inReadBusState(true) &&
                !writeRespQueueFull();
    return (read_rdy || write_rdy);
}

    std::pair<Tick, Tick>
NVMInterface::doBurstAccess(MemPacket* pkt, Tick next_burst_at,
                  const std::vector<MemPacketQueue>& queue)
{
    DPRINTF(NVM, "NVM Timing access to addr %#x, rank/bank/row %d %d %d\n",
            pkt->addr, pkt->rank, pkt->bank, pkt->row);

    // get the bank
    Bank& bank_ref = ranks[pkt->rank]->banks[pkt->bank];

    // respect any constraints on the command
    const Tick bst_allowed_at = pkt->isRead() ?
                                bank_ref.rdAllowedAt : bank_ref.wrAllowedAt;

    // we need to wait until the bus is available before we can issue
    // the command; need minimum of tBURST between commands
    Tick cmd_at = std::max(bst_allowed_at, curTick());

    // we need to wait until the bus is available before we can issue
    // the command; need minimum of tBURST between commands
    cmd_at = std::max(cmd_at, next_burst_at);

    // Verify there is command bandwidth to issue
    // Read burst (send command) is a simple data access and only requires
    // one command cycle
    // Write command may require multiple cycles to enable larger address space
    if (pkt->isRead() || !twoCycleRdWr) {
        cmd_at = ctrl->verifySingleCmd(cmd_at, maxCommandsPerWindow, false);
    } else {
        cmd_at = ctrl->verifyMultiCmd(cmd_at, maxCommandsPerWindow, tCK);
    }
    // update the packet ready time to reflect when data will be transferred
    // Use the same bus delays defined for NVM
    pkt->readyTime = cmd_at + tSEND + tBURST;

    Tick dly_to_rd_cmd;
    Tick dly_to_wr_cmd;
    for (auto n : ranks) {
        for (int i = 0; i < banksPerRank; i++) {
            // base delay is a function of tBURST and bus turnaround
            dly_to_rd_cmd = pkt->isRead() ? tBURST : writeToReadDelay();
            dly_to_wr_cmd = pkt->isRead() ? readToWriteDelay() : tBURST;

            if (pkt->rank != n->rank) {
                // adjust timing for different ranks
                // Need to account for rank-to-rank switching with tCS
                dly_to_wr_cmd = rankToRankDelay();
                dly_to_rd_cmd = rankToRankDelay();
            }
            n->banks[i].rdAllowedAt = std::max(cmd_at + dly_to_rd_cmd,
                                      n->banks[i].rdAllowedAt);

            n->banks[i].wrAllowedAt = std::max(cmd_at + dly_to_wr_cmd,
                                      n->banks[i].wrAllowedAt);
        }
    }

    DPRINTF(NVM, "NVM Access to %#x, ready at %lld.\n",
            pkt->addr, pkt->readyTime);

    if (pkt->isRead()) {
        // completed the read, decrement counters
        assert(numPendingReads != 0);
        assert(numReadDataReady != 0);

        numPendingReads--;
        numReadDataReady--;
    } else {
        // Adjust number of NVM writes in Q
        assert(numWritesQueued > 0);
        numWritesQueued--;

        // increment the bytes accessed and the accesses per row
        // only increment for writes as the reads are handled when
        // the non-deterministic read is issued, before the data transfer
        bank_ref.bytesAccessed += burstSize;

        // Commands will be issued serially when accessing the same bank
        // Commands can issue in parallel to different banks
        if ((bank_ref.bank == pkt->bank) &&
            (bank_ref.openRow != pkt->row)) {
           // update the open buffer, re-using row field
           bank_ref.openRow = pkt->row;

           // sample the bytes accessed to a buffer in this bank
           // here when we are re-buffering the data
           stats.bytesPerBank.sample(bank_ref.bytesAccessed);
           // start counting anew
           bank_ref.bytesAccessed = 0;
        }

        // Determine when write will actually complete, assuming it is
        // scheduled to push to NVM immediately
        // update actAllowedAt to serialize next command completion that
        // accesses this bank; must wait until this write completes
        // Data accesses to the same buffer in this bank
        // can issue immediately after actAllowedAt expires, without
        // waiting additional delay of tWRITE. Can revisit this
        // assumption/simplification in the future.
        bank_ref.actAllowedAt = std::max(pkt->readyTime,
                                bank_ref.actAllowedAt) + tWRITE;

        // Need to track number of outstanding writes to
        // ensure 'buffer' on media controller does not overflow
        assert(!writeRespQueueFull());

        // Insert into write done queue. It will be handled after
        // the media delay has been met
        if (writeRespQueueEmpty()) {
            assert(!writeRespondEvent.scheduled());
            schedule(writeRespondEvent, bank_ref.actAllowedAt);
        } else {
            assert(writeRespondEvent.scheduled());
        }
        writeRespQueue.push_back(bank_ref.actAllowedAt);
        writeRespQueue.sort();
        if (writeRespondEvent.when() > bank_ref.actAllowedAt) {
            DPRINTF(NVM, "Rescheduled respond event from %lld to %11d\n",
                writeRespondEvent.when(), bank_ref.actAllowedAt);
            DPRINTF(NVM, "Front of response queue is %11d\n",
                writeRespQueue.front());
            reschedule(writeRespondEvent, bank_ref.actAllowedAt);
        }

    }

    // Update the stats
    if (pkt->isRead()) {
        stats.readBursts++;
        stats.bytesRead += burstSize;
        stats.perBankRdBursts[pkt->bankId]++;
        stats.pendingReads.sample(numPendingReads);

        // Update latency stats
        stats.totMemAccLat += pkt->readyTime - pkt->entryTime;
        stats.totBusLat += tBURST;
        stats.totQLat += cmd_at - pkt->entryTime;
    } else {
        stats.writeBursts++;
        stats.bytesWritten += burstSize;
        stats.perBankWrBursts[pkt->bankId]++;
    }

    return std::make_pair(cmd_at, cmd_at + tBURST);
}

void
NVMInterface::processWriteRespondEvent()
{
    DPRINTF(NVM,
            "processWriteRespondEvent(): A NVM write reached its readyTime.  "
            "%d remaining pending NVM writes\n", writeRespQueue.size());

    // Update stat to track histogram of pending writes
    stats.pendingWrites.sample(writeRespQueue.size());

    // Done with this response, pop entry
    writeRespQueue.pop_front();

    if (!writeRespQueue.empty()) {
        assert(writeRespQueue.front() >= curTick());
        assert(!writeRespondEvent.scheduled());
        schedule(writeRespondEvent, writeRespQueue.front());
    }

    // It is possible that a new command kicks things back into
    // action before reaching this point but need to ensure that we
    // continue to process new commands as writes complete at the media and
    // credits become available. This will also trigger a drain if needed
    if (!ctrl->requestEventScheduled()) {
        DPRINTF(NVM, "Restart controller scheduler immediately\n");
        ctrl->restartScheduler(curTick());
    }
}

void
NVMInterface::addRankToRankDelay(Tick cmd_at)
{
    // update timing for NVM ranks due to bursts issued
    // to ranks for other media interfaces
    for (auto n : ranks) {
        for (int i = 0; i < banksPerRank; i++) {
            // different rank by default
            // Need to only account for rank-to-rank switching
            n->banks[i].rdAllowedAt = std::max(cmd_at + rankToRankDelay(),
                                             n->banks[i].rdAllowedAt);
            n->banks[i].wrAllowedAt = std::max(cmd_at + rankToRankDelay(),
                                             n->banks[i].wrAllowedAt);
        }
    }
}

bool
NVMInterface::isBusy(bool read_queue_empty, bool all_writes_nvm)
{
     DPRINTF(NVM,"isBusy: numReadDataReady = %d\n", numReadDataReady);
     // Determine NVM is busy and cannot issue a burst
     // A read burst cannot issue when data is not ready from the NVM
     // Also check that we have reads queued to ensure we can change
     // bus direction to service potential write commands.
     // A write cannot issue once we've reached MAX pending writes
     // Only assert busy for the write case when there are also
     // no reads in Q and the write queue only contains NVM commands
     // This allows the bus state to switch and service reads
     return (ctrl->inReadBusState(true) ?
                 (numReadDataReady == 0) && !read_queue_empty :
                 writeRespQueueFull() && read_queue_empty &&
                                         all_writes_nvm);
}

NVMInterface::NVMStats::NVMStats(NVMInterface &_nvm)
    : statistics::Group(&_nvm),
    nvm(_nvm),

    ADD_STAT(readBursts, statistics::units::Count::get(),
             "Number of NVM read bursts"),
    ADD_STAT(writeBursts, statistics::units::Count::get(),
             "Number of NVM write bursts"),

    ADD_STAT(perBankRdBursts, statistics::units::Count::get(),
             "Per bank write bursts"),
    ADD_STAT(perBankWrBursts, statistics::units::Count::get(),
             "Per bank write bursts"),

    ADD_STAT(totQLat, statistics::units::Tick::get(),
             "Total ticks spent queuing"),
    ADD_STAT(totBusLat, statistics::units::Tick::get(),
             "Total ticks spent in databus transfers"),
    ADD_STAT(totMemAccLat, statistics::units::Tick::get(),
             "Total ticks spent from burst creation until serviced "
             "by the NVM"),
    ADD_STAT(avgQLat, statistics::units::Rate<
                statistics::units::Tick, statistics::units::Count>::get(),
             "Average queueing delay per NVM burst"),
    ADD_STAT(avgBusLat, statistics::units::Rate<
                statistics::units::Tick, statistics::units::Count>::get(),
             "Average bus latency per NVM burst"),
    ADD_STAT(avgMemAccLat, statistics::units::Rate<
                statistics::units::Tick, statistics::units::Count>::get(),
             "Average memory access latency per NVM burst"),

    ADD_STAT(avgRdBW, statistics::units::Rate<
                statistics::units::Byte, statistics::units::Second>::get(),
             "Average DRAM read bandwidth in MiBytes/s"),
    ADD_STAT(avgWrBW, statistics::units::Rate<
                statistics::units::Byte, statistics::units::Second>::get(),
             "Average DRAM write bandwidth in MiBytes/s"),
    ADD_STAT(peakBW, statistics::units::Rate<
                statistics::units::Byte, statistics::units::Second>::get(),
             "Theoretical peak bandwidth in MiByte/s"),
    ADD_STAT(busUtil, statistics::units::Ratio::get(),
             "NVM Data bus utilization in percentage"),
    ADD_STAT(busUtilRead, statistics::units::Ratio::get(),
             "NVM Data bus read utilization in percentage"),
    ADD_STAT(busUtilWrite, statistics::units::Ratio::get(),
             "NVM Data bus write utilization in percentage"),

    ADD_STAT(pendingReads, statistics::units::Count::get(),
             "Reads issued to NVM for which data has not been transferred"),
    ADD_STAT(pendingWrites, statistics::units::Count::get(),
             "Number of outstanding writes to NVM"),
    ADD_STAT(bytesPerBank, statistics::units::Byte::get(),
             "Bytes read within a bank before loading new bank")

{
}

void
NVMInterface::NVMStats::regStats()
{
    using namespace statistics;

    perBankRdBursts.init(nvm.ranksPerChannel == 0 ? 1 :
              nvm.banksPerRank * nvm.ranksPerChannel);

    perBankWrBursts.init(nvm.ranksPerChannel == 0 ? 1 :
              nvm.banksPerRank * nvm.ranksPerChannel);

    avgQLat.precision(2);
    avgBusLat.precision(2);
    avgMemAccLat.precision(2);

    avgRdBW.precision(2);
    avgWrBW.precision(2);
    peakBW.precision(2);

    busUtil.precision(2);
    busUtilRead.precision(2);
    busUtilWrite.precision(2);

    pendingReads
        .init(nvm.maxPendingReads)
        .flags(nozero);

    pendingWrites
        .init(nvm.maxPendingWrites)
        .flags(nozero);

    bytesPerBank
        .init(nvm.rowBufferSize)
        .flags(nozero);

    avgQLat = totQLat / readBursts;
    avgBusLat = totBusLat / readBursts;
    avgMemAccLat = totMemAccLat / readBursts;

    avgRdBW = (bytesRead / 1000000) / simSeconds;
    avgWrBW = (bytesWritten / 1000000) / simSeconds;
    peakBW = (sim_clock::Frequency / nvm.tBURST) *
              nvm.burstSize / 1000000;

    busUtil = (avgRdBW + avgWrBW) / peakBW * 100;
    busUtilRead = avgRdBW / peakBW * 100;
    busUtilWrite = avgWrBW / peakBW * 100;
}

} // namespace memory
} // namespace gem5
