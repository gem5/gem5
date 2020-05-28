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

#include "mem/dram_ctrl.hh"

#include "base/bitfield.hh"
#include "base/trace.hh"
#include "debug/DRAM.hh"
#include "debug/DRAMPower.hh"
#include "debug/DRAMState.hh"
#include "debug/Drain.hh"
#include "debug/QOS.hh"
#include "sim/system.hh"

using namespace std;
using namespace Data;

DRAMCtrl::DRAMCtrl(const DRAMCtrlParams* p) :
    QoS::MemCtrl(p),
    port(name() + ".port", *this), isTimingMode(false),
    retryRdReq(false), retryWrReq(false),
    nextReqEvent([this]{ processNextReqEvent(); }, name()),
    respondEvent([this]{ processRespondEvent(); }, name()),
    deviceSize(p->device_size),
    deviceBusWidth(p->device_bus_width), burstLength(p->burst_length),
    deviceRowBufferSize(p->device_rowbuffer_size),
    devicesPerRank(p->devices_per_rank),
    burstSize((devicesPerRank * burstLength * deviceBusWidth) / 8),
    rowBufferSize(devicesPerRank * deviceRowBufferSize),
    columnsPerRowBuffer(rowBufferSize / burstSize),
    columnsPerStripe(range.interleaved() ? range.granularity() / burstSize : 1),
    ranksPerChannel(p->ranks_per_channel),
    bankGroupsPerRank(p->bank_groups_per_rank),
    bankGroupArch(p->bank_groups_per_rank > 0),
    banksPerRank(p->banks_per_rank), rowsPerBank(0),
    readBufferSize(p->read_buffer_size),
    writeBufferSize(p->write_buffer_size),
    writeHighThreshold(writeBufferSize * p->write_high_thresh_perc / 100.0),
    writeLowThreshold(writeBufferSize * p->write_low_thresh_perc / 100.0),
    minWritesPerSwitch(p->min_writes_per_switch),
    writesThisTime(0), readsThisTime(0),
    tCK(p->tCK), tRTW(p->tRTW), tCS(p->tCS), tBURST(p->tBURST),
    tBURST_MIN(p->tBURST_MIN),
    tCCD_L_WR(p->tCCD_L_WR),
    tCCD_L(p->tCCD_L), tRCD(p->tRCD), tCL(p->tCL), tRP(p->tRP), tRAS(p->tRAS),
    tWR(p->tWR), tRTP(p->tRTP), tRFC(p->tRFC), tREFI(p->tREFI), tRRD(p->tRRD),
    tRRD_L(p->tRRD_L), tPPD(p->tPPD), tAAD(p->tAAD), tXAW(p->tXAW),
    tXP(p->tXP), tXS(p->tXS),
    clkResyncDelay(tCL + p->tBURST_MAX),
    maxCommandsPerBurst(burstLength / p->beats_per_clock),
    dataClockSync(p->data_clock_sync),
    twoCycleActivate(p->two_cycle_activate),
    activationLimit(p->activation_limit), rankToRankDly(tCS + tBURST),
    wrToRdDly(tCL + tBURST + p->tWTR), rdToWrDly(tRTW + tBURST),
    wrToRdDlySameBG(tCL + p->tBURST_MAX + p->tWTR_L),
    rdToWrDlySameBG(tRTW + p->tBURST_MAX),
    burstInterleave(tBURST != tBURST_MIN),
    burstDataCycles(burstInterleave ? p->tBURST_MAX / 2 : tBURST),
    memSchedPolicy(p->mem_sched_policy), addrMapping(p->addr_mapping),
    pageMgmt(p->page_policy),
    maxAccessesPerRow(p->max_accesses_per_row),
    frontendLatency(p->static_frontend_latency),
    backendLatency(p->static_backend_latency),
    nextBurstAt(0), prevArrival(0),
    nextReqTime(0),
    stats(*this),
    activeRank(0), timeStampOffset(0),
    lastStatsResetTick(0), enableDRAMPowerdown(p->enable_dram_powerdown)
{
    // sanity check the ranks since we rely on bit slicing for the
    // address decoding
    fatal_if(!isPowerOf2(ranksPerChannel), "DRAM rank count of %d is not "
             "allowed, must be a power of two\n", ranksPerChannel);

    fatal_if(!isPowerOf2(burstSize), "DRAM burst size %d is not allowed, "
             "must be a power of two\n", burstSize);
    readQueue.resize(p->qos_priorities);
    writeQueue.resize(p->qos_priorities);

    for (int i = 0; i < ranksPerChannel; i++) {
        Rank* rank = new Rank(*this, p, i);
        ranks.push_back(rank);
    }

    // perform a basic check of the write thresholds
    if (p->write_low_thresh_perc >= p->write_high_thresh_perc)
        fatal("Write buffer low threshold %d must be smaller than the "
              "high threshold %d\n", p->write_low_thresh_perc,
              p->write_high_thresh_perc);

    // determine the rows per bank by looking at the total capacity
    uint64_t capacity = ULL(1) << ceilLog2(AbstractMemory::size());

    // determine the dram actual capacity from the DRAM config in Mbytes
    uint64_t deviceCapacity = deviceSize / (1024 * 1024) * devicesPerRank *
        ranksPerChannel;

    // if actual DRAM size does not match memory capacity in system warn!
    if (deviceCapacity != capacity / (1024 * 1024))
        warn("DRAM device capacity (%d Mbytes) does not match the "
             "address range assigned (%d Mbytes)\n", deviceCapacity,
             capacity / (1024 * 1024));

    DPRINTF(DRAM, "Memory capacity %lld (%lld) bytes\n", capacity,
            AbstractMemory::size());

    DPRINTF(DRAM, "Row buffer size %d bytes with %d columns per row buffer\n",
            rowBufferSize, columnsPerRowBuffer);

    rowsPerBank = capacity / (rowBufferSize * banksPerRank * ranksPerChannel);

    // some basic sanity checks
    if (tREFI <= tRP || tREFI <= tRFC) {
        fatal("tREFI (%d) must be larger than tRP (%d) and tRFC (%d)\n",
              tREFI, tRP, tRFC);
    }

    // basic bank group architecture checks ->
    if (bankGroupArch) {
        // must have at least one bank per bank group
        if (bankGroupsPerRank > banksPerRank) {
            fatal("banks per rank (%d) must be equal to or larger than "
                  "banks groups per rank (%d)\n",
                  banksPerRank, bankGroupsPerRank);
        }
        // must have same number of banks in each bank group
        if ((banksPerRank % bankGroupsPerRank) != 0) {
            fatal("Banks per rank (%d) must be evenly divisible by bank groups "
                  "per rank (%d) for equal banks per bank group\n",
                  banksPerRank, bankGroupsPerRank);
        }
        // tCCD_L should be greater than minimal, back-to-back burst delay
        if (tCCD_L <= tBURST) {
            fatal("tCCD_L (%d) should be larger than tBURST (%d) when "
                  "bank groups per rank (%d) is greater than 1\n",
                  tCCD_L, tBURST, bankGroupsPerRank);
        }
        // tCCD_L_WR should be greater than minimal, back-to-back burst delay
        if (tCCD_L_WR <= tBURST) {
            fatal("tCCD_L_WR (%d) should be larger than tBURST (%d) when "
                  "bank groups per rank (%d) is greater than 1\n",
                  tCCD_L_WR, tBURST, bankGroupsPerRank);
        }
        // tRRD_L is greater than minimal, same bank group ACT-to-ACT delay
        // some datasheets might specify it equal to tRRD
        if (tRRD_L < tRRD) {
            fatal("tRRD_L (%d) should be larger than tRRD (%d) when "
                  "bank groups per rank (%d) is greater than 1\n",
                  tRRD_L, tRRD, bankGroupsPerRank);
        }
    }

}

void
DRAMCtrl::init()
{
    MemCtrl::init();

   if (!port.isConnected()) {
        fatal("DRAMCtrl %s is unconnected!\n", name());
    } else {
        port.sendRangeChange();
    }

    // a bit of sanity checks on the interleaving, save it for here to
    // ensure that the system pointer is initialised
    if (range.interleaved()) {
        if (addrMapping == Enums::RoRaBaChCo) {
            if (rowBufferSize != range.granularity()) {
                fatal("Channel interleaving of %s doesn't match RoRaBaChCo "
                      "address map\n", name());
            }
        } else if (addrMapping == Enums::RoRaBaCoCh ||
                   addrMapping == Enums::RoCoRaBaCh) {
            // for the interleavings with channel bits in the bottom,
            // if the system uses a channel striping granularity that
            // is larger than the DRAM burst size, then map the
            // sequential accesses within a stripe to a number of
            // columns in the DRAM, effectively placing some of the
            // lower-order column bits as the least-significant bits
            // of the address (above the ones denoting the burst size)
            assert(columnsPerStripe >= 1);

            // channel striping has to be done at a granularity that
            // is equal or larger to a cache line
            if (system()->cacheLineSize() > range.granularity()) {
                fatal("Channel interleaving of %s must be at least as large "
                      "as the cache line size\n", name());
            }

            // ...and equal or smaller than the row-buffer size
            if (rowBufferSize < range.granularity()) {
                fatal("Channel interleaving of %s must be at most as large "
                      "as the row-buffer size\n", name());
            }
            // this is essentially the check above, so just to be sure
            assert(columnsPerStripe <= columnsPerRowBuffer);
        }
    }
}

void
DRAMCtrl::startup()
{
    // remember the memory system mode of operation
    isTimingMode = system()->isTimingMode();

    if (isTimingMode) {
        // timestamp offset should be in clock cycles for DRAMPower
        timeStampOffset = divCeil(curTick(), tCK);

        // update the start tick for the precharge accounting to the
        // current tick
        for (auto r : ranks) {
            r->startup(curTick() + tREFI - tRP);
        }

        // shift the bus busy time sufficiently far ahead that we never
        // have to worry about negative values when computing the time for
        // the next request, this will add an insignificant bubble at the
        // start of simulation
        nextBurstAt = curTick() + tRP + tRCD;
    }
}

Tick
DRAMCtrl::recvAtomic(PacketPtr pkt)
{
    DPRINTF(DRAM, "recvAtomic: %s 0x%x\n", pkt->cmdString(), pkt->getAddr());

    panic_if(pkt->cacheResponding(), "Should not see packets where cache "
             "is responding");

    // do the actual memory access and turn the packet into a response
    access(pkt);

    Tick latency = 0;
    if (pkt->hasData()) {
        // this value is not supposed to be accurate, just enough to
        // keep things going, mimic a closed page
        latency = tRP + tRCD + tCL;
    }
    return latency;
}

bool
DRAMCtrl::readQueueFull(unsigned int neededEntries) const
{
    DPRINTF(DRAM, "Read queue limit %d, current size %d, entries needed %d\n",
            readBufferSize, totalReadQueueSize + respQueue.size(),
            neededEntries);

    auto rdsize_new = totalReadQueueSize + respQueue.size() + neededEntries;
    return rdsize_new > readBufferSize;
}

bool
DRAMCtrl::writeQueueFull(unsigned int neededEntries) const
{
    DPRINTF(DRAM, "Write queue limit %d, current size %d, entries needed %d\n",
            writeBufferSize, totalWriteQueueSize, neededEntries);

    auto wrsize_new = (totalWriteQueueSize + neededEntries);
    return  wrsize_new > writeBufferSize;
}

DRAMCtrl::DRAMPacket*
DRAMCtrl::decodeAddr(const PacketPtr pkt, Addr dramPktAddr, unsigned size,
                     bool isRead) const
{
    // decode the address based on the address mapping scheme, with
    // Ro, Ra, Co, Ba and Ch denoting row, rank, column, bank and
    // channel, respectively
    uint8_t rank;
    uint8_t bank;
    // use a 64-bit unsigned during the computations as the row is
    // always the top bits, and check before creating the DRAMPacket
    uint64_t row;

    // truncate the address to a DRAM burst, which makes it unique to
    // a specific column, row, bank, rank and channel
    Addr addr = dramPktAddr / burstSize;

    // we have removed the lowest order address bits that denote the
    // position within the column
    if (addrMapping == Enums::RoRaBaChCo || addrMapping == Enums::RoRaBaCoCh) {
        // the lowest order bits denote the column to ensure that
        // sequential cache lines occupy the same row
        addr = addr / columnsPerRowBuffer;

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
    } else if (addrMapping == Enums::RoCoRaBaCh) {
        // optimise for closed page mode and utilise maximum
        // parallelism of the DRAM (at the cost of power)

        // take out the lower-order column bits
        addr = addr / columnsPerStripe;

        // start with the bank bits, as this provides the maximum
        // opportunity for parallelism between requests
        bank = addr % banksPerRank;
        addr = addr / banksPerRank;

        // next get the rank bits
        rank = addr % ranksPerChannel;
        addr = addr / ranksPerChannel;

        // next, the higher-order column bites
        addr = addr / (columnsPerRowBuffer / columnsPerStripe);

        // lastly, get the row bits, no need to remove them from addr
        row = addr % rowsPerBank;
    } else
        panic("Unknown address mapping policy chosen!");

    assert(rank < ranksPerChannel);
    assert(bank < banksPerRank);
    assert(row < rowsPerBank);
    assert(row < Bank::NO_ROW);

    DPRINTF(DRAM, "Address: %lld Rank %d Bank %d Row %d\n",
            dramPktAddr, rank, bank, row);

    // create the corresponding DRAM packet with the entry time and
    // ready time set to the current tick, the latter will be updated
    // later
    uint16_t bank_id = banksPerRank * rank + bank;
    return new DRAMPacket(pkt, isRead, rank, bank, row, bank_id, dramPktAddr,
                          size, ranks[rank]->banks[bank], *ranks[rank]);
}

void
DRAMCtrl::addToReadQueue(PacketPtr pkt, unsigned int pktCount)
{
    // only add to the read queue here. whenever the request is
    // eventually done, set the readyTime, and call schedule()
    assert(!pkt->isWrite());

    assert(pktCount != 0);

    // if the request size is larger than burst size, the pkt is split into
    // multiple DRAM packets
    // Note if the pkt starting address is not aligened to burst size, the
    // address of first DRAM packet is kept unaliged. Subsequent DRAM packets
    // are aligned to burst size boundaries. This is to ensure we accurately
    // check read packets against packets in write queue.
    const Addr base_addr = getCtrlAddr(pkt->getAddr());
    Addr addr = base_addr;
    unsigned pktsServicedByWrQ = 0;
    BurstHelper* burst_helper = NULL;
    for (int cnt = 0; cnt < pktCount; ++cnt) {
        unsigned size = std::min((addr | (burstSize - 1)) + 1,
                                 base_addr + pkt->getSize()) - addr;
        stats.readPktSize[ceilLog2(size)]++;
        stats.readBursts++;
        stats.masterReadAccesses[pkt->masterId()]++;

        // First check write buffer to see if the data is already at
        // the controller
        bool foundInWrQ = false;
        Addr burst_addr = burstAlign(addr);
        // if the burst address is not present then there is no need
        // looking any further
        if (isInWriteQueue.find(burst_addr) != isInWriteQueue.end()) {
            for (const auto& vec : writeQueue) {
                for (const auto& p : vec) {
                    // check if the read is subsumed in the write queue
                    // packet we are looking at
                    if (p->addr <= addr &&
                       ((addr + size) <= (p->addr + p->size))) {

                        foundInWrQ = true;
                        stats.servicedByWrQ++;
                        pktsServicedByWrQ++;
                        DPRINTF(DRAM,
                                "Read to addr %lld with size %d serviced by "
                                "write queue\n",
                                addr, size);
                        stats.bytesReadWrQ += burstSize;
                        break;
                    }
                }
            }
        }

        // If not found in the write q, make a DRAM packet and
        // push it onto the read queue
        if (!foundInWrQ) {

            // Make the burst helper for split packets
            if (pktCount > 1 && burst_helper == NULL) {
                DPRINTF(DRAM, "Read to addr %lld translates to %d "
                        "dram requests\n", pkt->getAddr(), pktCount);
                burst_helper = new BurstHelper(pktCount);
            }

            DRAMPacket* dram_pkt = decodeAddr(pkt, addr, size, true);
            dram_pkt->burstHelper = burst_helper;

            assert(!readQueueFull(1));
            stats.rdQLenPdf[totalReadQueueSize + respQueue.size()]++;

            DPRINTF(DRAM, "Adding to read queue\n");

            readQueue[dram_pkt->qosValue()].push_back(dram_pkt);

            ++dram_pkt->rankRef.readEntries;

            // log packet
            logRequest(MemCtrl::READ, pkt->masterId(), pkt->qosValue(),
                       dram_pkt->addr, 1);

            // Update stats
            stats.avgRdQLen = totalReadQueueSize + respQueue.size();
        }

        // Starting address of next dram pkt (aligend to burstSize boundary)
        addr = (addr | (burstSize - 1)) + 1;
    }

    // If all packets are serviced by write queue, we send the repsonse back
    if (pktsServicedByWrQ == pktCount) {
        accessAndRespond(pkt, frontendLatency);
        return;
    }

    // Update how many split packets are serviced by write queue
    if (burst_helper != NULL)
        burst_helper->burstsServiced = pktsServicedByWrQ;

    // If we are not already scheduled to get a request out of the
    // queue, do so now
    if (!nextReqEvent.scheduled()) {
        DPRINTF(DRAM, "Request scheduled immediately\n");
        schedule(nextReqEvent, curTick());
    }
}

void
DRAMCtrl::addToWriteQueue(PacketPtr pkt, unsigned int pktCount)
{
    // only add to the write queue here. whenever the request is
    // eventually done, set the readyTime, and call schedule()
    assert(pkt->isWrite());

    // if the request size is larger than burst size, the pkt is split into
    // multiple DRAM packets
    const Addr base_addr = getCtrlAddr(pkt->getAddr());
    Addr addr = base_addr;
    for (int cnt = 0; cnt < pktCount; ++cnt) {
        unsigned size = std::min((addr | (burstSize - 1)) + 1,
                                 base_addr + pkt->getSize()) - addr;
        stats.writePktSize[ceilLog2(size)]++;
        stats.writeBursts++;
        stats.masterWriteAccesses[pkt->masterId()]++;

        // see if we can merge with an existing item in the write
        // queue and keep track of whether we have merged or not
        bool merged = isInWriteQueue.find(burstAlign(addr)) !=
            isInWriteQueue.end();

        // if the item was not merged we need to create a new write
        // and enqueue it
        if (!merged) {
            DRAMPacket* dram_pkt = decodeAddr(pkt, addr, size, false);

            assert(totalWriteQueueSize < writeBufferSize);
            stats.wrQLenPdf[totalWriteQueueSize]++;

            DPRINTF(DRAM, "Adding to write queue\n");

            writeQueue[dram_pkt->qosValue()].push_back(dram_pkt);
            isInWriteQueue.insert(burstAlign(addr));

            // log packet
            logRequest(MemCtrl::WRITE, pkt->masterId(), pkt->qosValue(),
                       dram_pkt->addr, 1);

            assert(totalWriteQueueSize == isInWriteQueue.size());

            // Update stats
            stats.avgWrQLen = totalWriteQueueSize;

            // increment write entries of the rank
            ++dram_pkt->rankRef.writeEntries;
        } else {
            DPRINTF(DRAM, "Merging write burst with existing queue entry\n");

            // keep track of the fact that this burst effectively
            // disappeared as it was merged with an existing one
            stats.mergedWrBursts++;
        }

        // Starting address of next dram pkt (aligend to burstSize boundary)
        addr = (addr | (burstSize - 1)) + 1;
    }

    // we do not wait for the writes to be send to the actual memory,
    // but instead take responsibility for the consistency here and
    // snoop the write queue for any upcoming reads
    // @todo, if a pkt size is larger than burst size, we might need a
    // different front end latency
    accessAndRespond(pkt, frontendLatency);

    // If we are not already scheduled to get a request out of the
    // queue, do so now
    if (!nextReqEvent.scheduled()) {
        DPRINTF(DRAM, "Request scheduled immediately\n");
        schedule(nextReqEvent, curTick());
    }
}

void
DRAMCtrl::printQs() const
{
#if TRACING_ON
    DPRINTF(DRAM, "===READ QUEUE===\n\n");
    for (const auto& queue : readQueue) {
        for (const auto& packet : queue) {
            DPRINTF(DRAM, "Read %lu\n", packet->addr);
        }
    }

    DPRINTF(DRAM, "\n===RESP QUEUE===\n\n");
    for (const auto& packet : respQueue) {
        DPRINTF(DRAM, "Response %lu\n", packet->addr);
    }

    DPRINTF(DRAM, "\n===WRITE QUEUE===\n\n");
    for (const auto& queue : writeQueue) {
        for (const auto& packet : queue) {
            DPRINTF(DRAM, "Write %lu\n", packet->addr);
        }
    }
#endif // TRACING_ON
}

bool
DRAMCtrl::recvTimingReq(PacketPtr pkt)
{
    // This is where we enter from the outside world
    DPRINTF(DRAM, "recvTimingReq: request %s addr %lld size %d\n",
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


    // Find out how many dram packets a pkt translates to
    // If the burst size is equal or larger than the pkt size, then a pkt
    // translates to only one dram packet. Otherwise, a pkt translates to
    // multiple dram packets
    unsigned size = pkt->getSize();
    unsigned offset = pkt->getAddr() & (burstSize - 1);
    unsigned int dram_pkt_count = divCeil(offset + size, burstSize);

    // run the QoS scheduler and assign a QoS priority value to the packet
    qosSchedule( { &readQueue, &writeQueue }, burstSize, pkt);

    // check local buffers and do not accept if full
    if (pkt->isWrite()) {
        assert(size != 0);
        if (writeQueueFull(dram_pkt_count)) {
            DPRINTF(DRAM, "Write queue full, not accepting\n");
            // remember that we have to retry this port
            retryWrReq = true;
            stats.numWrRetry++;
            return false;
        } else {
            addToWriteQueue(pkt, dram_pkt_count);
            stats.writeReqs++;
            stats.bytesWrittenSys += size;
        }
    } else {
        assert(pkt->isRead());
        assert(size != 0);
        if (readQueueFull(dram_pkt_count)) {
            DPRINTF(DRAM, "Read queue full, not accepting\n");
            // remember that we have to retry this port
            retryRdReq = true;
            stats.numRdRetry++;
            return false;
        } else {
            addToReadQueue(pkt, dram_pkt_count);
            stats.readReqs++;
            stats.bytesReadSys += size;
        }
    }

    return true;
}

void
DRAMCtrl::processRespondEvent()
{
    DPRINTF(DRAM,
            "processRespondEvent(): Some req has reached its readyTime\n");

    DRAMPacket* dram_pkt = respQueue.front();

    // if a read has reached its ready-time, decrement the number of reads
    // At this point the packet has been handled and there is a possibility
    // to switch to low-power mode if no other packet is available
    --dram_pkt->rankRef.readEntries;
    DPRINTF(DRAM, "number of read entries for rank %d is %d\n",
            dram_pkt->rank, dram_pkt->rankRef.readEntries);

    // counter should at least indicate one outstanding request
    // for this read
    assert(dram_pkt->rankRef.outstandingEvents > 0);
    // read response received, decrement count
    --dram_pkt->rankRef.outstandingEvents;

    // at this moment should not have transitioned to a low-power state
    assert((dram_pkt->rankRef.pwrState != PWR_SREF) &&
           (dram_pkt->rankRef.pwrState != PWR_PRE_PDN) &&
           (dram_pkt->rankRef.pwrState != PWR_ACT_PDN));

    // track if this is the last packet before idling
    // and that there are no outstanding commands to this rank
    if (dram_pkt->rankRef.isQueueEmpty() &&
        dram_pkt->rankRef.outstandingEvents == 0 &&
        dram_pkt->rankRef.inRefIdleState() && enableDRAMPowerdown) {
        // verify that there are no events scheduled
        assert(!dram_pkt->rankRef.activateEvent.scheduled());
        assert(!dram_pkt->rankRef.prechargeEvent.scheduled());

        // if coming from active state, schedule power event to
        // active power-down else go to precharge power-down
        DPRINTF(DRAMState, "Rank %d sleep at tick %d; current power state is "
                "%d\n", dram_pkt->rank, curTick(), dram_pkt->rankRef.pwrState);

        // default to ACT power-down unless already in IDLE state
        // could be in IDLE if PRE issued before data returned
        PowerState next_pwr_state = PWR_ACT_PDN;
        if (dram_pkt->rankRef.pwrState == PWR_IDLE) {
            next_pwr_state = PWR_PRE_PDN;
        }

        dram_pkt->rankRef.powerDownSleep(next_pwr_state, curTick());
    }

    if (dram_pkt->burstHelper) {
        // it is a split packet
        dram_pkt->burstHelper->burstsServiced++;
        if (dram_pkt->burstHelper->burstsServiced ==
            dram_pkt->burstHelper->burstCount) {
            // we have now serviced all children packets of a system packet
            // so we can now respond to the requester
            // @todo we probably want to have a different front end and back
            // end latency for split packets
            accessAndRespond(dram_pkt->pkt, frontendLatency + backendLatency);
            delete dram_pkt->burstHelper;
            dram_pkt->burstHelper = NULL;
        }
    } else {
        // it is not a split packet
        accessAndRespond(dram_pkt->pkt, frontendLatency + backendLatency);
    }

    assert(respQueue.front() == dram_pkt);
    respQueue.pop_front();

    if (!respQueue.empty()) {
        assert(respQueue.front()->readyTime >= curTick());
        assert(!respondEvent.scheduled());
        schedule(respondEvent, respQueue.front()->readyTime);
    } else {
        // if there is nothing left in any queue, signal a drain
        if (drainState() == DrainState::Draining &&
            !totalWriteQueueSize && !totalReadQueueSize && allRanksDrained()) {

            DPRINTF(Drain, "DRAM controller done draining\n");
            signalDrainDone();
        } else if ((dram_pkt->rankRef.refreshState == REF_PRE) &&
                   !dram_pkt->rankRef.prechargeEvent.scheduled()) {
            // kick the refresh event loop into action again if banks already
            // closed and just waiting for read to complete
            schedule(dram_pkt->rankRef.refreshEvent, curTick());
        }
    }

    delete dram_pkt;

    // We have made a location in the queue available at this point,
    // so if there is a read that was forced to wait, retry now
    if (retryRdReq) {
        retryRdReq = false;
        port.sendRetryReq();
    }
}

DRAMCtrl::DRAMPacketQueue::iterator
DRAMCtrl::chooseNext(DRAMPacketQueue& queue, Tick extra_col_delay)
{
    // This method does the arbitration between requests.

    DRAMCtrl::DRAMPacketQueue::iterator ret = queue.end();

    if (!queue.empty()) {
        if (queue.size() == 1) {
            // available rank corresponds to state refresh idle
            DRAMPacket* dram_pkt = *(queue.begin());
            if (ranks[dram_pkt->rank]->inRefIdleState()) {
                ret = queue.begin();
                DPRINTF(DRAM, "Single request, going to a free rank\n");
            } else {
                DPRINTF(DRAM, "Single request, going to a busy rank\n");
            }
        } else if (memSchedPolicy == Enums::fcfs) {
            // check if there is a packet going to a free rank
            for (auto i = queue.begin(); i != queue.end(); ++i) {
                DRAMPacket* dram_pkt = *i;
                if (ranks[dram_pkt->rank]->inRefIdleState()) {
                    ret = i;
                    break;
                }
            }
        } else if (memSchedPolicy == Enums::frfcfs) {
            ret = chooseNextFRFCFS(queue, extra_col_delay);
        } else {
            panic("No scheduling policy chosen\n");
        }
    }
    return ret;
}

DRAMCtrl::DRAMPacketQueue::iterator
DRAMCtrl::chooseNextFRFCFS(DRAMPacketQueue& queue, Tick extra_col_delay)
{
    // Only determine this if needed
    vector<uint32_t> earliest_banks(ranksPerChannel, 0);

    // Has minBankPrep been called to populate earliest_banks?
    bool filled_earliest_banks = false;
    // can the PRE/ACT sequence be done without impacting utlization?
    bool hidden_bank_prep = false;

    // search for seamless row hits first, if no seamless row hit is
    // found then determine if there are other packets that can be issued
    // without incurring additional bus delay due to bank timing
    // Will select closed rows first to enable more open row possibilies
    // in future selections
    bool found_hidden_bank = false;

    // remember if we found a row hit, not seamless, but bank prepped
    // and ready
    bool found_prepped_pkt = false;

    // if we have no row hit, prepped or not, and no seamless packet,
    // just go for the earliest possible
    bool found_earliest_pkt = false;

    auto selected_pkt_it = queue.end();

    // time we need to issue a column command to be seamless
    const Tick min_col_at = std::max(nextBurstAt + extra_col_delay, curTick());

    for (auto i = queue.begin(); i != queue.end() ; ++i) {
        DRAMPacket* dram_pkt = *i;
        const Bank& bank = dram_pkt->bankRef;
        const Tick col_allowed_at = dram_pkt->isRead() ? bank.rdAllowedAt :
                                                         bank.wrAllowedAt;

        DPRINTF(DRAM, "%s checking packet in bank %d, row %d\n",
                __func__, dram_pkt->bankRef.bank, dram_pkt->row);

        // check if rank is not doing a refresh and thus is available, if not,
        // jump to the next packet
        if (dram_pkt->rankRef.inRefIdleState()) {

            DPRINTF(DRAM,
                    "%s bank %d - Rank %d available\n", __func__,
                    dram_pkt->bankRef.bank, dram_pkt->rankRef.rank);

            // check if it is a row hit
            if (bank.openRow == dram_pkt->row) {
                // no additional rank-to-rank or same bank-group
                // delays, or we switched read/write and might as well
                // go for the row hit
                if (col_allowed_at <= min_col_at) {
                    // FCFS within the hits, giving priority to
                    // commands that can issue seamlessly, without
                    // additional delay, such as same rank accesses
                    // and/or different bank-group accesses
                    DPRINTF(DRAM, "%s Seamless row buffer hit\n", __func__);
                    selected_pkt_it = i;
                    // no need to look through the remaining queue entries
                    break;
                } else if (!found_hidden_bank && !found_prepped_pkt) {
                    // if we did not find a packet to a closed row that can
                    // issue the bank commands without incurring delay, and
                    // did not yet find a packet to a prepped row, remember
                    // the current one
                    selected_pkt_it = i;
                    found_prepped_pkt = true;
                    DPRINTF(DRAM, "%s Prepped row buffer hit\n", __func__);
                }
            } else if (!found_earliest_pkt) {
                // if we have not initialised the bank status, do it
                // now, and only once per scheduling decisions
                if (!filled_earliest_banks) {
                    // determine entries with earliest bank delay
                    std::tie(earliest_banks, hidden_bank_prep) =
                        minBankPrep(queue, min_col_at);
                    filled_earliest_banks = true;
                }

                // bank is amongst first available banks
                // minBankPrep will give priority to packets that can
                // issue seamlessly
                if (bits(earliest_banks[dram_pkt->rank],
                         dram_pkt->bank, dram_pkt->bank)) {
                    found_earliest_pkt = true;
                    found_hidden_bank = hidden_bank_prep;

                    // give priority to packets that can issue
                    // bank commands 'behind the scenes'
                    // any additional delay if any will be due to
                    // col-to-col command requirements
                    if (hidden_bank_prep || !found_prepped_pkt)
                        selected_pkt_it = i;
                }
            }
        } else {
            DPRINTF(DRAM, "%s bank %d - Rank %d not available\n", __func__,
                    dram_pkt->bankRef.bank, dram_pkt->rankRef.rank);
        }
    }

    if (selected_pkt_it == queue.end()) {
        DPRINTF(DRAM, "%s no available ranks found\n", __func__);
    }

    return selected_pkt_it;
}

void
DRAMCtrl::accessAndRespond(PacketPtr pkt, Tick static_latency)
{
    DPRINTF(DRAM, "Responding to Address %lld.. ",pkt->getAddr());

    bool needsResponse = pkt->needsResponse();
    // do the actual memory access which also turns the packet into a
    // response
    access(pkt);

    // turn packet around to go back to requester if response expected
    if (needsResponse) {
        // access already turned the packet into a response
        assert(pkt->isResponse());
        // response_time consumes the static latency and is charged also
        // with headerDelay that takes into account the delay provided by
        // the xbar and also the payloadDelay that takes into account the
        // number of data beats.
        Tick response_time = curTick() + static_latency + pkt->headerDelay +
                             pkt->payloadDelay;
        // Here we reset the timing of the packet before sending it out.
        pkt->headerDelay = pkt->payloadDelay = 0;

        // queue the packet in the response queue to be sent out after
        // the static latency has passed
        port.schedTimingResp(pkt, response_time);
    } else {
        // @todo the packet is going to be deleted, and the DRAMPacket
        // is still having a pointer to it
        pendingDelete.reset(pkt);
    }

    DPRINTF(DRAM, "Done\n");

    return;
}

void
DRAMCtrl::pruneBurstTick()
{
    auto it = burstTicks.begin();
    while (it != burstTicks.end()) {
        auto current_it = it++;
        if (curTick() > *current_it) {
            DPRINTF(DRAM, "Removing burstTick for %d\n", *current_it);
            burstTicks.erase(current_it);
        }
    }
}

Tick
DRAMCtrl::getBurstWindow(Tick cmd_tick)
{
    // get tick aligned to burst window
    Tick burst_offset = cmd_tick % burstDataCycles;
    return (cmd_tick - burst_offset);
}

Tick
DRAMCtrl::verifySingleCmd(Tick cmd_tick)
{
    // start with assumption that there is no contention on command bus
    Tick cmd_at = cmd_tick;

    // get tick aligned to burst window
    Tick burst_tick = getBurstWindow(cmd_tick);

    // verify that we have command bandwidth to issue the command
    // if not, iterate over next window(s) until slot found
    while (burstTicks.count(burst_tick) >= maxCommandsPerBurst) {
        DPRINTF(DRAM, "Contention found on command bus at %d\n", burst_tick);
        burst_tick += burstDataCycles;
        cmd_at = burst_tick;
    }

    // add command into burst window and return corresponding Tick
    burstTicks.insert(burst_tick);
    return cmd_at;
}

Tick
DRAMCtrl::verifyMultiCmd(Tick cmd_tick, Tick max_multi_cmd_split)
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
    Tick first_cmd_offset = cmd_tick % burstDataCycles;
    while (max_multi_cmd_split > (first_cmd_offset + burst_offset)) {
        burst_offset += burstDataCycles;
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
        auto first_cmd_count = burstTicks.count(first_cmd_tick);
        auto second_cmd_count = same_burst ? first_cmd_count + 1 :
                                   burstTicks.count(burst_tick);

        first_can_issue = first_cmd_count < maxCommandsPerBurst;
        second_can_issue = second_cmd_count < maxCommandsPerBurst;

        if (!second_can_issue) {
            DPRINTF(DRAM, "Contention (cmd2) found on command bus at %d\n",
                    burst_tick);
            burst_tick += burstDataCycles;
            cmd_at = burst_tick;
        }

        // Verify max_multi_cmd_split isn't violated when command 2 is shifted
        // If commands initially were issued in same burst, they are
        // now in consecutive bursts and can still issue B2B
        bool gap_violated = !same_burst &&
             ((burst_tick - first_cmd_tick) > max_multi_cmd_split);

        if (!first_can_issue || (!second_can_issue && gap_violated)) {
            DPRINTF(DRAM, "Contention (cmd1) found on command bus at %d\n",
                    first_cmd_tick);
            first_cmd_tick += burstDataCycles;
        }
    }

    // Add command to burstTicks
    burstTicks.insert(burst_tick);
    burstTicks.insert(first_cmd_tick);

    return cmd_at;
}

void
DRAMCtrl::activateBank(Rank& rank_ref, Bank& bank_ref,
                       Tick act_tick, uint32_t row)
{
    assert(rank_ref.actTicks.size() == activationLimit);

    // verify that we have command bandwidth to issue the activate
    // if not, shift to next burst window
    Tick act_at;
    if (twoCycleActivate)
        act_at = verifyMultiCmd(act_tick, tAAD);
    else
        act_at = verifySingleCmd(act_tick);

    DPRINTF(DRAM, "Activate at tick %d\n", act_at);

    // update the open row
    assert(bank_ref.openRow == Bank::NO_ROW);
    bank_ref.openRow = row;

    // start counting anew, this covers both the case when we
    // auto-precharged, and when this access is forced to
    // precharge
    bank_ref.bytesAccessed = 0;
    bank_ref.rowAccesses = 0;

    ++rank_ref.numBanksActive;
    assert(rank_ref.numBanksActive <= banksPerRank);

    DPRINTF(DRAM, "Activate bank %d, rank %d at tick %lld, now got %d active\n",
            bank_ref.bank, rank_ref.rank, act_at,
            ranks[rank_ref.rank]->numBanksActive);

    rank_ref.cmdList.push_back(Command(MemCommand::ACT, bank_ref.bank,
                               act_at));

    DPRINTF(DRAMPower, "%llu,ACT,%d,%d\n", divCeil(act_at, tCK) -
            timeStampOffset, bank_ref.bank, rank_ref.rank);

    // The next access has to respect tRAS for this bank
    bank_ref.preAllowedAt = act_at + tRAS;

    // Respect the row-to-column command delay for both read and write cmds
    bank_ref.rdAllowedAt = std::max(act_at + tRCD, bank_ref.rdAllowedAt);
    bank_ref.wrAllowedAt = std::max(act_at + tRCD, bank_ref.wrAllowedAt);

    // start by enforcing tRRD
    for (int i = 0; i < banksPerRank; i++) {
        // next activate to any bank in this rank must not happen
        // before tRRD
        if (bankGroupArch && (bank_ref.bankgr == rank_ref.banks[i].bankgr)) {
            // bank group architecture requires longer delays between
            // ACT commands within the same bank group.  Use tRRD_L
            // in this case
            rank_ref.banks[i].actAllowedAt = std::max(act_at + tRRD_L,
                                             rank_ref.banks[i].actAllowedAt);
        } else {
            // use shorter tRRD value when either
            // 1) bank group architecture is not supportted
            // 2) bank is in a different bank group
            rank_ref.banks[i].actAllowedAt = std::max(act_at + tRRD,
                                             rank_ref.banks[i].actAllowedAt);
        }
    }

    // next, we deal with tXAW, if the activation limit is disabled
    // then we directly schedule an activate power event
    if (!rank_ref.actTicks.empty()) {
        // sanity check
        if (rank_ref.actTicks.back() &&
           (act_at - rank_ref.actTicks.back()) < tXAW) {
            panic("Got %d activates in window %d (%llu - %llu) which "
                  "is smaller than %llu\n", activationLimit, act_at -
                  rank_ref.actTicks.back(), act_at,
                  rank_ref.actTicks.back(), tXAW);
        }

        // shift the times used for the book keeping, the last element
        // (highest index) is the oldest one and hence the lowest value
        rank_ref.actTicks.pop_back();

        // record an new activation (in the future)
        rank_ref.actTicks.push_front(act_at);

        // cannot activate more than X times in time window tXAW, push the
        // next one (the X + 1'st activate) to be tXAW away from the
        // oldest in our window of X
        if (rank_ref.actTicks.back() &&
           (act_at - rank_ref.actTicks.back()) < tXAW) {
            DPRINTF(DRAM, "Enforcing tXAW with X = %d, next activate "
                    "no earlier than %llu\n", activationLimit,
                    rank_ref.actTicks.back() + tXAW);
            for (int j = 0; j < banksPerRank; j++)
                // next activate must not happen before end of window
                rank_ref.banks[j].actAllowedAt =
                    std::max(rank_ref.actTicks.back() + tXAW,
                             rank_ref.banks[j].actAllowedAt);
        }
    }

    // at the point when this activate takes place, make sure we
    // transition to the active power state
    if (!rank_ref.activateEvent.scheduled())
        schedule(rank_ref.activateEvent, act_at);
    else if (rank_ref.activateEvent.when() > act_at)
        // move it sooner in time
        reschedule(rank_ref.activateEvent, act_at);
}

void
DRAMCtrl::prechargeBank(Rank& rank_ref, Bank& bank, Tick pre_tick,
                        bool auto_or_preall, bool trace)
{
    // make sure the bank has an open row
    assert(bank.openRow != Bank::NO_ROW);

    // sample the bytes per activate here since we are closing
    // the page
    stats.bytesPerActivate.sample(bank.bytesAccessed);

    bank.openRow = Bank::NO_ROW;

    Tick pre_at = pre_tick;
    if (auto_or_preall) {
        // no precharge allowed before this one
        bank.preAllowedAt = pre_at;
    } else {
        // Issuing an explicit PRE command
        // Verify that we have command bandwidth to issue the precharge
        // if not, shift to next burst window
        pre_at = verifySingleCmd(pre_tick);
        // enforce tPPD
        for (int i = 0; i < banksPerRank; i++) {
            rank_ref.banks[i].preAllowedAt = std::max(pre_at + tPPD,
                                             rank_ref.banks[i].preAllowedAt);
        }
    }

    Tick pre_done_at = pre_at + tRP;

    bank.actAllowedAt = std::max(bank.actAllowedAt, pre_done_at);

    assert(rank_ref.numBanksActive != 0);
    --rank_ref.numBanksActive;

    DPRINTF(DRAM, "Precharging bank %d, rank %d at tick %lld, now got "
            "%d active\n", bank.bank, rank_ref.rank, pre_at,
            rank_ref.numBanksActive);

    if (trace) {

        rank_ref.cmdList.push_back(Command(MemCommand::PRE, bank.bank,
                                   pre_at));
        DPRINTF(DRAMPower, "%llu,PRE,%d,%d\n", divCeil(pre_at, tCK) -
                timeStampOffset, bank.bank, rank_ref.rank);
    }

    // if we look at the current number of active banks we might be
    // tempted to think the DRAM is now idle, however this can be
    // undone by an activate that is scheduled to happen before we
    // would have reached the idle state, so schedule an event and
    // rather check once we actually make it to the point in time when
    // the (last) precharge takes place
    if (!rank_ref.prechargeEvent.scheduled()) {
        schedule(rank_ref.prechargeEvent, pre_done_at);
        // New event, increment count
        ++rank_ref.outstandingEvents;
    } else if (rank_ref.prechargeEvent.when() < pre_done_at) {
        reschedule(rank_ref.prechargeEvent, pre_done_at);
    }
}

void
DRAMCtrl::doDRAMAccess(DRAMPacket* dram_pkt)
{
    DPRINTF(DRAM, "Timing access to addr %lld, rank/bank/row %d %d %d\n",
            dram_pkt->addr, dram_pkt->rank, dram_pkt->bank, dram_pkt->row);

    // first clean up the burstTick set, removing old entries
    // before adding new entries for next burst
    pruneBurstTick();

    // get the rank
    Rank& rank = dram_pkt->rankRef;

    // are we in or transitioning to a low-power state and have not scheduled
    // a power-up event?
    // if so, wake up from power down to issue RD/WR burst
    if (rank.inLowPowerState) {
        assert(rank.pwrState != PWR_SREF);
        rank.scheduleWakeUpEvent(tXP);
    }

    // get the bank
    Bank& bank = dram_pkt->bankRef;

    // for the state we need to track if it is a row hit or not
    bool row_hit = true;

    // Determine the access latency and update the bank state
    if (bank.openRow == dram_pkt->row) {
        // nothing to do
    } else {
        row_hit = false;

        // If there is a page open, precharge it.
        if (bank.openRow != Bank::NO_ROW) {
            prechargeBank(rank, bank, std::max(bank.preAllowedAt, curTick()));
        }

        // next we need to account for the delay in activating the
        // page
        Tick act_tick = std::max(bank.actAllowedAt, curTick());

        // Record the activation and deal with all the global timing
        // constraints caused be a new activation (tRRD and tXAW)
        activateBank(rank, bank, act_tick, dram_pkt->row);
    }

    // respect any constraints on the command (e.g. tRCD or tCCD)
    const Tick col_allowed_at = dram_pkt->isRead() ?
                                          bank.rdAllowedAt : bank.wrAllowedAt;

    // we need to wait until the bus is available before we can issue
    // the command; need minimum of tBURST between commands
    Tick cmd_at = std::max({col_allowed_at, nextBurstAt, curTick()});

    // verify that we have command bandwidth to issue the burst
    // if not, shift to next burst window
    if (dataClockSync && ((cmd_at - rank.lastBurstTick) > clkResyncDelay))
        cmd_at = verifyMultiCmd(cmd_at, tCK);
    else
        cmd_at = verifySingleCmd(cmd_at);

    // if we are interleaving bursts, ensure that
    // 1) we don't double interleave on next burst issue
    // 2) we are at an interleave boundary; if not, shift to next boundary
    Tick burst_gap = tBURST_MIN;
    if (burstInterleave) {
        if (cmd_at == (rank.lastBurstTick + tBURST_MIN)) {
            // already interleaving, push next command to end of full burst
            burst_gap = tBURST;
        } else if (cmd_at < (rank.lastBurstTick + tBURST)) {
            // not at an interleave boundary after bandwidth check
            // Shift command to tBURST boundary to avoid data contention
            // Command will remain in the same burstTicks window given that
            // tBURST is less than tBURST_MAX
            cmd_at = rank.lastBurstTick + tBURST;
        }
    }
    DPRINTF(DRAM, "Schedule RD/WR burst at tick %d\n", cmd_at);

    // update the packet ready time
    dram_pkt->readyTime = cmd_at + tCL + tBURST;

    rank.lastBurstTick = cmd_at;

    // update the time for the next read/write burst for each
    // bank (add a max with tCCD/tCCD_L/tCCD_L_WR here)
    Tick dly_to_rd_cmd;
    Tick dly_to_wr_cmd;
    for (int j = 0; j < ranksPerChannel; j++) {
        for (int i = 0; i < banksPerRank; i++) {
            // next burst to same bank group in this rank must not happen
            // before tCCD_L.  Different bank group timing requirement is
            // tBURST; Add tCS for different ranks
            if (dram_pkt->rank == j) {
                if (bankGroupArch &&
                   (bank.bankgr == ranks[j]->banks[i].bankgr)) {
                    // bank group architecture requires longer delays between
                    // RD/WR burst commands to the same bank group.
                    // tCCD_L is default requirement for same BG timing
                    // tCCD_L_WR is required for write-to-write
                    // Need to also take bus turnaround delays into account
                    dly_to_rd_cmd = dram_pkt->isRead() ?
                                    tCCD_L : std::max(tCCD_L, wrToRdDlySameBG);
                    dly_to_wr_cmd = dram_pkt->isRead() ?
                                    std::max(tCCD_L, rdToWrDlySameBG) :
                                    tCCD_L_WR;
                } else {
                    // tBURST is default requirement for diff BG timing
                    // Need to also take bus turnaround delays into account
                    dly_to_rd_cmd = dram_pkt->isRead() ? burst_gap : wrToRdDly;
                    dly_to_wr_cmd = dram_pkt->isRead() ? rdToWrDly : burst_gap;
                }
            } else {
                // different rank is by default in a different bank group and
                // doesn't require longer tCCD or additional RTW, WTR delays
                // Need to account for rank-to-rank switching with tCS
                dly_to_wr_cmd = rankToRankDly;
                dly_to_rd_cmd = rankToRankDly;
            }
            ranks[j]->banks[i].rdAllowedAt = std::max(cmd_at + dly_to_rd_cmd,
                                             ranks[j]->banks[i].rdAllowedAt);
            ranks[j]->banks[i].wrAllowedAt = std::max(cmd_at + dly_to_wr_cmd,
                                             ranks[j]->banks[i].wrAllowedAt);
        }
    }

    // Save rank of current access
    activeRank = dram_pkt->rank;

    // If this is a write, we also need to respect the write recovery
    // time before a precharge, in the case of a read, respect the
    // read to precharge constraint
    bank.preAllowedAt = std::max(bank.preAllowedAt,
                                 dram_pkt->isRead() ? cmd_at + tRTP :
                                 dram_pkt->readyTime + tWR);

    // increment the bytes accessed and the accesses per row
    bank.bytesAccessed += burstSize;
    ++bank.rowAccesses;

    // if we reached the max, then issue with an auto-precharge
    bool auto_precharge = pageMgmt == Enums::close ||
        bank.rowAccesses == maxAccessesPerRow;

    // if we did not hit the limit, we might still want to
    // auto-precharge
    if (!auto_precharge &&
        (pageMgmt == Enums::open_adaptive ||
         pageMgmt == Enums::close_adaptive)) {
        // a twist on the open and close page policies:
        // 1) open_adaptive page policy does not blindly keep the
        // page open, but close it if there are no row hits, and there
        // are bank conflicts in the queue
        // 2) close_adaptive page policy does not blindly close the
        // page, but closes it only if there are no row hits in the queue.
        // In this case, only force an auto precharge when there
        // are no same page hits in the queue
        bool got_more_hits = false;
        bool got_bank_conflict = false;

        // either look at the read queue or write queue
        const std::vector<DRAMPacketQueue>& queue =
                dram_pkt->isRead() ? readQueue : writeQueue;

        for (uint8_t i = 0; i < numPriorities(); ++i) {
            auto p = queue[i].begin();
            // keep on looking until we find a hit or reach the end of the queue
            // 1) if a hit is found, then both open and close adaptive policies keep
            // the page open
            // 2) if no hit is found, got_bank_conflict is set to true if a bank
            // conflict request is waiting in the queue
            // 3) make sure we are not considering the packet that we are
            // currently dealing with
            while (!got_more_hits && p != queue[i].end()) {
                if (dram_pkt != (*p)) {
                    bool same_rank_bank = (dram_pkt->rank == (*p)->rank) &&
                                          (dram_pkt->bank == (*p)->bank);

                    bool same_row = dram_pkt->row == (*p)->row;
                    got_more_hits |= same_rank_bank && same_row;
                    got_bank_conflict |= same_rank_bank && !same_row;
                }
                ++p;
            }

            if (got_more_hits)
                break;
        }

        // auto pre-charge when either
        // 1) open_adaptive policy, we have not got any more hits, and
        //    have a bank conflict
        // 2) close_adaptive policy and we have not got any more hits
        auto_precharge = !got_more_hits &&
            (got_bank_conflict || pageMgmt == Enums::close_adaptive);
    }

    // DRAMPower trace command to be written
    std::string mem_cmd = dram_pkt->isRead() ? "RD" : "WR";

    // MemCommand required for DRAMPower library
    MemCommand::cmds command = (mem_cmd == "RD") ? MemCommand::RD :
                                                   MemCommand::WR;

    // Update bus state to reflect when previous command was issued
    nextBurstAt = cmd_at + burst_gap;
    DPRINTF(DRAM, "Access to %lld, ready at %lld next burst at %lld.\n",
            dram_pkt->addr, dram_pkt->readyTime, nextBurstAt);

    dram_pkt->rankRef.cmdList.push_back(Command(command, dram_pkt->bank,
                                        cmd_at));

    DPRINTF(DRAMPower, "%llu,%s,%d,%d\n", divCeil(cmd_at, tCK) -
            timeStampOffset, mem_cmd, dram_pkt->bank, dram_pkt->rank);

    // if this access should use auto-precharge, then we are
    // closing the row after the read/write burst
    if (auto_precharge) {
        // if auto-precharge push a PRE command at the correct tick to the
        // list used by DRAMPower library to calculate power
        prechargeBank(rank, bank, std::max(curTick(), bank.preAllowedAt),
                      true);

        DPRINTF(DRAM, "Auto-precharged bank: %d\n", dram_pkt->bankId);
    }

    // Update the minimum timing between the requests, this is a
    // conservative estimate of when we have to schedule the next
    // request to not introduce any unecessary bubbles. In most cases
    // we will wake up sooner than we have to.
    nextReqTime = nextBurstAt - (tRP + tRCD);

    // Update the stats and schedule the next request
    if (dram_pkt->isRead()) {
        ++readsThisTime;
        if (row_hit)
            stats.readRowHits++;
        stats.bytesReadDRAM += burstSize;
        stats.perBankRdBursts[dram_pkt->bankId]++;

        // Update latency stats
        stats.totMemAccLat += dram_pkt->readyTime - dram_pkt->entryTime;
        stats.masterReadTotalLat[dram_pkt->masterId()] +=
            dram_pkt->readyTime - dram_pkt->entryTime;

        stats.totBusLat += tBURST;
        stats.totQLat += cmd_at - dram_pkt->entryTime;
        stats.masterReadBytes[dram_pkt->masterId()] += dram_pkt->size;
    } else {
        ++writesThisTime;
        if (row_hit)
            stats.writeRowHits++;
        stats.bytesWritten += burstSize;
        stats.perBankWrBursts[dram_pkt->bankId]++;
        stats.masterWriteBytes[dram_pkt->masterId()] += dram_pkt->size;
        stats.masterWriteTotalLat[dram_pkt->masterId()] +=
            dram_pkt->readyTime - dram_pkt->entryTime;
    }
}

void
DRAMCtrl::processNextReqEvent()
{
    // transition is handled by QoS algorithm if enabled
    if (turnPolicy) {
        // select bus state - only done if QoS algorithms are in use
        busStateNext = selectNextBusState();
    }

    // detect bus state change
    bool switched_cmd_type = (busState != busStateNext);
    // record stats
    recordTurnaroundStats();

    DPRINTF(DRAM, "QoS Turnarounds selected state %s %s\n",
            (busState==MemCtrl::READ)?"READ":"WRITE",
            switched_cmd_type?"[turnaround triggered]":"");

    if (switched_cmd_type) {
        if (busState == READ) {
            DPRINTF(DRAM,
                    "Switching to writes after %d reads with %d reads "
                    "waiting\n", readsThisTime, totalReadQueueSize);
            stats.rdPerTurnAround.sample(readsThisTime);
            readsThisTime = 0;
        } else {
            DPRINTF(DRAM,
                    "Switching to reads after %d writes with %d writes "
                    "waiting\n", writesThisTime, totalWriteQueueSize);
            stats.wrPerTurnAround.sample(writesThisTime);
            writesThisTime = 0;
        }
    }

    // updates current state
    busState = busStateNext;

    // check ranks for refresh/wakeup - uses busStateNext, so done after turnaround
    // decisions
    int busyRanks = 0;
    for (auto r : ranks) {
        if (!r->inRefIdleState()) {
            if (r->pwrState != PWR_SREF) {
                // rank is busy refreshing
                DPRINTF(DRAMState, "Rank %d is not available\n", r->rank);
                busyRanks++;

                // let the rank know that if it was waiting to drain, it
                // is now done and ready to proceed
                r->checkDrainDone();
            }

            // check if we were in self-refresh and haven't started
            // to transition out
            if ((r->pwrState == PWR_SREF) && r->inLowPowerState) {
                DPRINTF(DRAMState, "Rank %d is in self-refresh\n", r->rank);
                // if we have commands queued to this rank and we don't have
                // a minimum number of active commands enqueued,
                // exit self-refresh
                if (r->forceSelfRefreshExit()) {
                    DPRINTF(DRAMState, "rank %d was in self refresh and"
                           " should wake up\n", r->rank);
                    //wake up from self-refresh
                    r->scheduleWakeUpEvent(tXS);
                    // things are brought back into action once a refresh is
                    // performed after self-refresh
                    // continue with selection for other ranks
                }
            }
        }
    }

    if (busyRanks == ranksPerChannel) {
        // if all ranks are refreshing wait for them to finish
        // and stall this state machine without taking any further
        // action, and do not schedule a new nextReqEvent
        return;
    }

    // when we get here it is either a read or a write
    if (busState == READ) {

        // track if we should switch or not
        bool switch_to_writes = false;

        if (totalReadQueueSize == 0) {
            // In the case there is no read request to go next,
            // trigger writes if we have passed the low threshold (or
            // if we are draining)
            if (!(totalWriteQueueSize == 0) &&
                (drainState() == DrainState::Draining ||
                 totalWriteQueueSize > writeLowThreshold)) {

                DPRINTF(DRAM, "Switching to writes due to read queue empty\n");
                switch_to_writes = true;
            } else {
                // check if we are drained
                // not done draining until in PWR_IDLE state
                // ensuring all banks are closed and
                // have exited low power states
                if (drainState() == DrainState::Draining &&
                    respQueue.empty() && allRanksDrained()) {

                    DPRINTF(Drain, "DRAM controller done draining\n");
                    signalDrainDone();
                }

                // nothing to do, not even any point in scheduling an
                // event for the next request
                return;
            }
        } else {

            bool read_found = false;
            DRAMPacketQueue::iterator to_read;
            uint8_t prio = numPriorities();

            for (auto queue = readQueue.rbegin();
                 queue != readQueue.rend(); ++queue) {

                prio--;

                DPRINTF(QOS,
                        "DRAM controller checking READ queue [%d] priority [%d elements]\n",
                        prio, queue->size());

                // Figure out which read request goes next
                // If we are changing command type, incorporate the minimum
                // bus turnaround delay which will be tCS (different rank) case
                to_read = chooseNext((*queue), switched_cmd_type ? tCS : 0);

                if (to_read != queue->end()) {
                    // candidate read found
                    read_found = true;
                    break;
                }
            }

            // if no read to an available rank is found then return
            // at this point. There could be writes to the available ranks
            // which are above the required threshold. However, to
            // avoid adding more complexity to the code, return and wait
            // for a refresh event to kick things into action again.
            if (!read_found) {
                DPRINTF(DRAM, "No Reads Found - exiting\n");
                return;
            }

            auto dram_pkt = *to_read;

            assert(dram_pkt->rankRef.inRefIdleState());

            doDRAMAccess(dram_pkt);

            // Every respQueue which will generate an event, increment count
            ++dram_pkt->rankRef.outstandingEvents;
            // sanity check
            assert(dram_pkt->size <= burstSize);
            assert(dram_pkt->readyTime >= curTick());

            // log the response
            logResponse(MemCtrl::READ, (*to_read)->masterId(),
                        dram_pkt->qosValue(), dram_pkt->getAddr(), 1,
                        dram_pkt->readyTime - dram_pkt->entryTime);


            // Insert into response queue. It will be sent back to the
            // requester at its readyTime
            if (respQueue.empty()) {
                assert(!respondEvent.scheduled());
                schedule(respondEvent, dram_pkt->readyTime);
            } else {
                assert(respQueue.back()->readyTime <= dram_pkt->readyTime);
                assert(respondEvent.scheduled());
            }

            respQueue.push_back(dram_pkt);

            // we have so many writes that we have to transition
            if (totalWriteQueueSize > writeHighThreshold) {
                switch_to_writes = true;
            }

            // remove the request from the queue - the iterator is no longer valid .
            readQueue[dram_pkt->qosValue()].erase(to_read);
        }

        // switching to writes, either because the read queue is empty
        // and the writes have passed the low threshold (or we are
        // draining), or because the writes hit the hight threshold
        if (switch_to_writes) {
            // transition to writing
            busStateNext = WRITE;
        }
    } else {

        bool write_found = false;
        DRAMPacketQueue::iterator to_write;
        uint8_t prio = numPriorities();

        for (auto queue = writeQueue.rbegin();
             queue != writeQueue.rend(); ++queue) {

            prio--;

            DPRINTF(QOS,
                    "DRAM controller checking WRITE queue [%d] priority [%d elements]\n",
                    prio, queue->size());

            // If we are changing command type, incorporate the minimum
            // bus turnaround delay
            to_write = chooseNext((*queue),
                                  switched_cmd_type ? std::min(tRTW, tCS) : 0);

            if (to_write != queue->end()) {
                write_found = true;
                break;
            }
        }

        // if there are no writes to a rank that is available to service
        // requests (i.e. rank is in refresh idle state) are found then
        // return. There could be reads to the available ranks. However, to
        // avoid adding more complexity to the code, return at this point and
        // wait for a refresh event to kick things into action again.
        if (!write_found) {
            DPRINTF(DRAM, "No Writes Found - exiting\n");
            return;
        }

        auto dram_pkt = *to_write;

        assert(dram_pkt->rankRef.inRefIdleState());
        // sanity check
        assert(dram_pkt->size <= burstSize);

        doDRAMAccess(dram_pkt);

        // removed write from queue, decrement count
        --dram_pkt->rankRef.writeEntries;

        // Schedule write done event to decrement event count
        // after the readyTime has been reached
        // Only schedule latest write event to minimize events
        // required; only need to ensure that final event scheduled covers
        // the time that writes are outstanding and bus is active
        // to holdoff power-down entry events
        if (!dram_pkt->rankRef.writeDoneEvent.scheduled()) {
            schedule(dram_pkt->rankRef.writeDoneEvent, dram_pkt->readyTime);
            // New event, increment count
            ++dram_pkt->rankRef.outstandingEvents;

        } else if (dram_pkt->rankRef.writeDoneEvent.when() <
                   dram_pkt->readyTime) {

            reschedule(dram_pkt->rankRef.writeDoneEvent, dram_pkt->readyTime);
        }

        isInWriteQueue.erase(burstAlign(dram_pkt->addr));

        // log the response
        logResponse(MemCtrl::WRITE, dram_pkt->masterId(),
                    dram_pkt->qosValue(), dram_pkt->getAddr(), 1,
                    dram_pkt->readyTime - dram_pkt->entryTime);


        // remove the request from the queue - the iterator is no longer valid
        writeQueue[dram_pkt->qosValue()].erase(to_write);

        delete dram_pkt;

        // If we emptied the write queue, or got sufficiently below the
        // threshold (using the minWritesPerSwitch as the hysteresis) and
        // are not draining, or we have reads waiting and have done enough
        // writes, then switch to reads.
        bool below_threshold =
            totalWriteQueueSize + minWritesPerSwitch < writeLowThreshold;

        if (totalWriteQueueSize == 0 ||
            (below_threshold && drainState() != DrainState::Draining) ||
            (totalReadQueueSize && writesThisTime >= minWritesPerSwitch)) {

            // turn the bus back around for reads again
            busStateNext = READ;

            // note that the we switch back to reads also in the idle
            // case, which eventually will check for any draining and
            // also pause any further scheduling if there is really
            // nothing to do
        }
    }
    // It is possible that a refresh to another rank kicks things back into
    // action before reaching this point.
    if (!nextReqEvent.scheduled())
        schedule(nextReqEvent, std::max(nextReqTime, curTick()));

    // If there is space available and we have writes waiting then let
    // them retry. This is done here to ensure that the retry does not
    // cause a nextReqEvent to be scheduled before we do so as part of
    // the next request processing
    if (retryWrReq && totalWriteQueueSize < writeBufferSize) {
        retryWrReq = false;
        port.sendRetryReq();
    }
}

pair<vector<uint32_t>, bool>
DRAMCtrl::minBankPrep(const DRAMPacketQueue& queue,
                      Tick min_col_at) const
{
    Tick min_act_at = MaxTick;
    vector<uint32_t> bank_mask(ranksPerChannel, 0);

    // latest Tick for which ACT can occur without incurring additoinal
    // delay on the data bus
    const Tick hidden_act_max = std::max(min_col_at - tRCD, curTick());

    // Flag condition when burst can issue back-to-back with previous burst
    bool found_seamless_bank = false;

    // Flag condition when bank can be opened without incurring additional
    // delay on the data bus
    bool hidden_bank_prep = false;

    // determine if we have queued transactions targetting the
    // bank in question
    vector<bool> got_waiting(ranksPerChannel * banksPerRank, false);
    for (const auto& p : queue) {
        if (p->rankRef.inRefIdleState())
            got_waiting[p->bankId] = true;
    }

    // Find command with optimal bank timing
    // Will prioritize commands that can issue seamlessly.
    for (int i = 0; i < ranksPerChannel; i++) {
        for (int j = 0; j < banksPerRank; j++) {
            uint16_t bank_id = i * banksPerRank + j;

            // if we have waiting requests for the bank, and it is
            // amongst the first available, update the mask
            if (got_waiting[bank_id]) {
                // make sure this rank is not currently refreshing.
                assert(ranks[i]->inRefIdleState());
                // simplistic approximation of when the bank can issue
                // an activate, ignoring any rank-to-rank switching
                // cost in this calculation
                Tick act_at = ranks[i]->banks[j].openRow == Bank::NO_ROW ?
                    std::max(ranks[i]->banks[j].actAllowedAt, curTick()) :
                    std::max(ranks[i]->banks[j].preAllowedAt, curTick()) + tRP;

                // When is the earliest the R/W burst can issue?
                const Tick col_allowed_at = (busState == READ) ?
                                              ranks[i]->banks[j].rdAllowedAt :
                                              ranks[i]->banks[j].wrAllowedAt;
                Tick col_at = std::max(col_allowed_at, act_at + tRCD);

                // bank can issue burst back-to-back (seamlessly) with
                // previous burst
                bool new_seamless_bank = col_at <= min_col_at;

                // if we found a new seamless bank or we have no
                // seamless banks, and got a bank with an earlier
                // activate time, it should be added to the bit mask
                if (new_seamless_bank ||
                    (!found_seamless_bank && act_at <= min_act_at)) {
                    // if we did not have a seamless bank before, and
                    // we do now, reset the bank mask, also reset it
                    // if we have not yet found a seamless bank and
                    // the activate time is smaller than what we have
                    // seen so far
                    if (!found_seamless_bank &&
                        (new_seamless_bank || act_at < min_act_at)) {
                        std::fill(bank_mask.begin(), bank_mask.end(), 0);
                    }

                    found_seamless_bank |= new_seamless_bank;

                    // ACT can occur 'behind the scenes'
                    hidden_bank_prep = act_at <= hidden_act_max;

                    // set the bit corresponding to the available bank
                    replaceBits(bank_mask[i], j, j, 1);
                    min_act_at = act_at;
                }
            }
        }
    }

    return make_pair(bank_mask, hidden_bank_prep);
}

DRAMCtrl::Rank::Rank(DRAMCtrl& _memory, const DRAMCtrlParams* _p, int rank)
    : EventManager(&_memory), memory(_memory),
      pwrStateTrans(PWR_IDLE), pwrStatePostRefresh(PWR_IDLE),
      pwrStateTick(0), refreshDueAt(0), pwrState(PWR_IDLE),
      refreshState(REF_IDLE), inLowPowerState(false), rank(rank),
      readEntries(0), writeEntries(0), outstandingEvents(0),
      wakeUpAllowedAt(0), power(_p, false), banks(_p->banks_per_rank),
      numBanksActive(0), actTicks(_p->activation_limit, 0), lastBurstTick(0),
      writeDoneEvent([this]{ processWriteDoneEvent(); }, name()),
      activateEvent([this]{ processActivateEvent(); }, name()),
      prechargeEvent([this]{ processPrechargeEvent(); }, name()),
      refreshEvent([this]{ processRefreshEvent(); }, name()),
      powerEvent([this]{ processPowerEvent(); }, name()),
      wakeUpEvent([this]{ processWakeUpEvent(); }, name()),
      stats(_memory, *this)
{
    for (int b = 0; b < _p->banks_per_rank; b++) {
        banks[b].bank = b;
        // GDDR addressing of banks to BG is linear.
        // Here we assume that all DRAM generations address bank groups as
        // follows:
        if (_p->bank_groups_per_rank > 0) {
            // Simply assign lower bits to bank group in order to
            // rotate across bank groups as banks are incremented
            // e.g. with 4 banks per bank group and 16 banks total:
            //    banks 0,4,8,12  are in bank group 0
            //    banks 1,5,9,13  are in bank group 1
            //    banks 2,6,10,14 are in bank group 2
            //    banks 3,7,11,15 are in bank group 3
            banks[b].bankgr = b % _p->bank_groups_per_rank;
        } else {
            // No bank groups; simply assign to bank number
            banks[b].bankgr = b;
        }
    }
}

void
DRAMCtrl::Rank::startup(Tick ref_tick)
{
    assert(ref_tick > curTick());

    pwrStateTick = curTick();

    // kick off the refresh, and give ourselves enough time to
    // precharge
    schedule(refreshEvent, ref_tick);
}

void
DRAMCtrl::Rank::suspend()
{
    deschedule(refreshEvent);

    // Update the stats
    updatePowerStats();

    // don't automatically transition back to LP state after next REF
    pwrStatePostRefresh = PWR_IDLE;
}

bool
DRAMCtrl::Rank::isQueueEmpty() const
{
    // check commmands in Q based on current bus direction
    bool no_queued_cmds = ((memory.busStateNext == READ) && (readEntries == 0))
                          || ((memory.busStateNext == WRITE) &&
                              (writeEntries == 0));
    return no_queued_cmds;
}

void
DRAMCtrl::Rank::checkDrainDone()
{
    // if this rank was waiting to drain it is now able to proceed to
    // precharge
    if (refreshState == REF_DRAIN) {
        DPRINTF(DRAM, "Refresh drain done, now precharging\n");

        refreshState = REF_PD_EXIT;

        // hand control back to the refresh event loop
        schedule(refreshEvent, curTick());
    }
}

void
DRAMCtrl::Rank::flushCmdList()
{
    // at the moment sort the list of commands and update the counters
    // for DRAMPower libray when doing a refresh
    sort(cmdList.begin(), cmdList.end(), DRAMCtrl::sortTime);

    auto next_iter = cmdList.begin();
    // push to commands to DRAMPower
    for ( ; next_iter != cmdList.end() ; ++next_iter) {
         Command cmd = *next_iter;
         if (cmd.timeStamp <= curTick()) {
             // Move all commands at or before curTick to DRAMPower
             power.powerlib.doCommand(cmd.type, cmd.bank,
                                      divCeil(cmd.timeStamp, memory.tCK) -
                                      memory.timeStampOffset);
         } else {
             // done - found all commands at or before curTick()
             // next_iter references the 1st command after curTick
             break;
         }
    }
    // reset cmdList to only contain commands after curTick
    // if there are no commands after curTick, updated cmdList will be empty
    // in this case, next_iter is cmdList.end()
    cmdList.assign(next_iter, cmdList.end());
}

void
DRAMCtrl::Rank::processActivateEvent()
{
    // we should transition to the active state as soon as any bank is active
    if (pwrState != PWR_ACT)
        // note that at this point numBanksActive could be back at
        // zero again due to a precharge scheduled in the future
        schedulePowerEvent(PWR_ACT, curTick());
}

void
DRAMCtrl::Rank::processPrechargeEvent()
{
    // counter should at least indicate one outstanding request
    // for this precharge
    assert(outstandingEvents > 0);
    // precharge complete, decrement count
    --outstandingEvents;

    // if we reached zero, then special conditions apply as we track
    // if all banks are precharged for the power models
    if (numBanksActive == 0) {
        // no reads to this rank in the Q and no pending
        // RD/WR or refresh commands
        if (isQueueEmpty() && outstandingEvents == 0 &&
            memory.enableDRAMPowerdown) {
            // should still be in ACT state since bank still open
            assert(pwrState == PWR_ACT);

            // All banks closed - switch to precharge power down state.
            DPRINTF(DRAMState, "Rank %d sleep at tick %d\n",
                    rank, curTick());
            powerDownSleep(PWR_PRE_PDN, curTick());
        } else {
            // we should transition to the idle state when the last bank
            // is precharged
            schedulePowerEvent(PWR_IDLE, curTick());
        }
    }
}

void
DRAMCtrl::Rank::processWriteDoneEvent()
{
    // counter should at least indicate one outstanding request
    // for this write
    assert(outstandingEvents > 0);
    // Write transfer on bus has completed
    // decrement per rank counter
    --outstandingEvents;
}

void
DRAMCtrl::Rank::processRefreshEvent()
{
    // when first preparing the refresh, remember when it was due
    if ((refreshState == REF_IDLE) || (refreshState == REF_SREF_EXIT)) {
        // remember when the refresh is due
        refreshDueAt = curTick();

        // proceed to drain
        refreshState = REF_DRAIN;

        // make nonzero while refresh is pending to ensure
        // power down and self-refresh are not entered
        ++outstandingEvents;

        DPRINTF(DRAM, "Refresh due\n");
    }

    // let any scheduled read or write to the same rank go ahead,
    // after which it will
    // hand control back to this event loop
    if (refreshState == REF_DRAIN) {
        // if a request is at the moment being handled and this request is
        // accessing the current rank then wait for it to finish
        if ((rank == memory.activeRank)
            && (memory.nextReqEvent.scheduled())) {
            // hand control over to the request loop until it is
            // evaluated next
            DPRINTF(DRAM, "Refresh awaiting draining\n");

            return;
        } else {
            refreshState = REF_PD_EXIT;
        }
    }

    // at this point, ensure that rank is not in a power-down state
    if (refreshState == REF_PD_EXIT) {
        // if rank was sleeping and we have't started exit process,
        // wake-up for refresh
        if (inLowPowerState) {
            DPRINTF(DRAM, "Wake Up for refresh\n");
            // save state and return after refresh completes
            scheduleWakeUpEvent(memory.tXP);
            return;
        } else {
            refreshState = REF_PRE;
        }
    }

    // at this point, ensure that all banks are precharged
    if (refreshState == REF_PRE) {
        // precharge any active bank
        if (numBanksActive != 0) {
            // at the moment, we use a precharge all even if there is
            // only a single bank open
            DPRINTF(DRAM, "Precharging all\n");

            // first determine when we can precharge
            Tick pre_at = curTick();

            for (auto &b : banks) {
                // respect both causality and any existing bank
                // constraints, some banks could already have a
                // (auto) precharge scheduled
                pre_at = std::max(b.preAllowedAt, pre_at);
            }

            // make sure all banks per rank are precharged, and for those that
            // already are, update their availability
            Tick act_allowed_at = pre_at + memory.tRP;

            for (auto &b : banks) {
                if (b.openRow != Bank::NO_ROW) {
                    memory.prechargeBank(*this, b, pre_at, true, false);
                } else {
                    b.actAllowedAt = std::max(b.actAllowedAt, act_allowed_at);
                    b.preAllowedAt = std::max(b.preAllowedAt, pre_at);
                }
            }

            // precharge all banks in rank
            cmdList.push_back(Command(MemCommand::PREA, 0, pre_at));

            DPRINTF(DRAMPower, "%llu,PREA,0,%d\n",
                    divCeil(pre_at, memory.tCK) -
                            memory.timeStampOffset, rank);
        } else if ((pwrState == PWR_IDLE) && (outstandingEvents == 1))  {
            // Banks are closed, have transitioned to IDLE state, and
            // no outstanding ACT,RD/WR,Auto-PRE sequence scheduled
            DPRINTF(DRAM, "All banks already precharged, starting refresh\n");

            // go ahead and kick the power state machine into gear since
            // we are already idle
            schedulePowerEvent(PWR_REF, curTick());
        } else {
            // banks state is closed but haven't transitioned pwrState to IDLE
            // or have outstanding ACT,RD/WR,Auto-PRE sequence scheduled
            // should have outstanding precharge or read response event
            assert(prechargeEvent.scheduled() ||
                   memory.respondEvent.scheduled());
            // will start refresh when pwrState transitions to IDLE
        }

        assert(numBanksActive == 0);

        // wait for all banks to be precharged or read to complete
        // When precharge commands are done, power state machine will
        // transition to the idle state, and automatically move to a
        // refresh, at that point it will also call this method to get
        // the refresh event loop going again
        // Similarly, when read response completes, if all banks are
        // precharged, will call this method to get loop re-started
        return;
    }

    // last but not least we perform the actual refresh
    if (refreshState == REF_START) {
        // should never get here with any banks active
        assert(numBanksActive == 0);
        assert(pwrState == PWR_REF);

        Tick ref_done_at = curTick() + memory.tRFC;

        for (auto &b : banks) {
            b.actAllowedAt = ref_done_at;
        }

        // at the moment this affects all ranks
        cmdList.push_back(Command(MemCommand::REF, 0, curTick()));

        // Update the stats
        updatePowerStats();

        DPRINTF(DRAMPower, "%llu,REF,0,%d\n", divCeil(curTick(), memory.tCK) -
                memory.timeStampOffset, rank);

        // Update for next refresh
        refreshDueAt += memory.tREFI;

        // make sure we did not wait so long that we cannot make up
        // for it
        if (refreshDueAt < ref_done_at) {
            fatal("Refresh was delayed so long we cannot catch up\n");
        }

        // Run the refresh and schedule event to transition power states
        // when refresh completes
        refreshState = REF_RUN;
        schedule(refreshEvent, ref_done_at);
        return;
    }

    if (refreshState == REF_RUN) {
        // should never get here with any banks active
        assert(numBanksActive == 0);
        assert(pwrState == PWR_REF);

        assert(!powerEvent.scheduled());

        if ((memory.drainState() == DrainState::Draining) ||
            (memory.drainState() == DrainState::Drained)) {
            // if draining, do not re-enter low-power mode.
            // simply go to IDLE and wait
            schedulePowerEvent(PWR_IDLE, curTick());
        } else {
            // At the moment, we sleep when the refresh ends and wait to be
            // woken up again if previously in a low-power state.
            if (pwrStatePostRefresh != PWR_IDLE) {
                // power State should be power Refresh
                assert(pwrState == PWR_REF);
                DPRINTF(DRAMState, "Rank %d sleeping after refresh and was in "
                        "power state %d before refreshing\n", rank,
                        pwrStatePostRefresh);
                powerDownSleep(pwrState, curTick());

            // Force PRE power-down if there are no outstanding commands
            // in Q after refresh.
            } else if (isQueueEmpty() && memory.enableDRAMPowerdown) {
                // still have refresh event outstanding but there should
                // be no other events outstanding
                assert(outstandingEvents == 1);
                DPRINTF(DRAMState, "Rank %d sleeping after refresh but was NOT"
                        " in a low power state before refreshing\n", rank);
                powerDownSleep(PWR_PRE_PDN, curTick());

            } else {
                // move to the idle power state once the refresh is done, this
                // will also move the refresh state machine to the refresh
                // idle state
                schedulePowerEvent(PWR_IDLE, curTick());
            }
        }

        // At this point, we have completed the current refresh.
        // In the SREF bypass case, we do not get to this state in the
        // refresh STM and therefore can always schedule next event.
        // Compensate for the delay in actually performing the refresh
        // when scheduling the next one
        schedule(refreshEvent, refreshDueAt - memory.tRP);

        DPRINTF(DRAMState, "Refresh done at %llu and next refresh"
                " at %llu\n", curTick(), refreshDueAt);
    }
}

void
DRAMCtrl::Rank::schedulePowerEvent(PowerState pwr_state, Tick tick)
{
    // respect causality
    assert(tick >= curTick());

    if (!powerEvent.scheduled()) {
        DPRINTF(DRAMState, "Scheduling power event at %llu to state %d\n",
                tick, pwr_state);

        // insert the new transition
        pwrStateTrans = pwr_state;

        schedule(powerEvent, tick);
    } else {
        panic("Scheduled power event at %llu to state %d, "
              "with scheduled event at %llu to %d\n", tick, pwr_state,
              powerEvent.when(), pwrStateTrans);
    }
}

void
DRAMCtrl::Rank::powerDownSleep(PowerState pwr_state, Tick tick)
{
    // if low power state is active low, schedule to active low power state.
    // in reality tCKE is needed to enter active low power. This is neglected
    // here and could be added in the future.
    if (pwr_state == PWR_ACT_PDN) {
        schedulePowerEvent(pwr_state, tick);
        // push command to DRAMPower
        cmdList.push_back(Command(MemCommand::PDN_F_ACT, 0, tick));
        DPRINTF(DRAMPower, "%llu,PDN_F_ACT,0,%d\n", divCeil(tick,
                memory.tCK) - memory.timeStampOffset, rank);
    } else if (pwr_state == PWR_PRE_PDN) {
        // if low power state is precharge low, schedule to precharge low
        // power state. In reality tCKE is needed to enter active low power.
        // This is neglected here.
        schedulePowerEvent(pwr_state, tick);
        //push Command to DRAMPower
        cmdList.push_back(Command(MemCommand::PDN_F_PRE, 0, tick));
        DPRINTF(DRAMPower, "%llu,PDN_F_PRE,0,%d\n", divCeil(tick,
                memory.tCK) - memory.timeStampOffset, rank);
    } else if (pwr_state == PWR_REF) {
        // if a refresh just occurred
        // transition to PRE_PDN now that all banks are closed
        // precharge power down requires tCKE to enter. For simplicity
        // this is not considered.
        schedulePowerEvent(PWR_PRE_PDN, tick);
        //push Command to DRAMPower
        cmdList.push_back(Command(MemCommand::PDN_F_PRE, 0, tick));
        DPRINTF(DRAMPower, "%llu,PDN_F_PRE,0,%d\n", divCeil(tick,
                memory.tCK) - memory.timeStampOffset, rank);
    } else if (pwr_state == PWR_SREF) {
        // should only enter SREF after PRE-PD wakeup to do a refresh
        assert(pwrStatePostRefresh == PWR_PRE_PDN);
        // self refresh requires time tCKESR to enter. For simplicity,
        // this is not considered.
        schedulePowerEvent(PWR_SREF, tick);
        // push Command to DRAMPower
        cmdList.push_back(Command(MemCommand::SREN, 0, tick));
        DPRINTF(DRAMPower, "%llu,SREN,0,%d\n", divCeil(tick,
                memory.tCK) - memory.timeStampOffset, rank);
    }
    // Ensure that we don't power-down and back up in same tick
    // Once we commit to PD entry, do it and wait for at least 1tCK
    // This could be replaced with tCKE if/when that is added to the model
    wakeUpAllowedAt = tick + memory.tCK;

    // Transitioning to a low power state, set flag
    inLowPowerState = true;
}

void
DRAMCtrl::Rank::scheduleWakeUpEvent(Tick exit_delay)
{
    Tick wake_up_tick = std::max(curTick(), wakeUpAllowedAt);

    DPRINTF(DRAMState, "Scheduling wake-up for rank %d at tick %d\n",
            rank, wake_up_tick);

    // if waking for refresh, hold previous state
    // else reset state back to IDLE
    if (refreshState == REF_PD_EXIT) {
        pwrStatePostRefresh = pwrState;
    } else {
        // don't automatically transition back to LP state after next REF
        pwrStatePostRefresh = PWR_IDLE;
    }

    // schedule wake-up with event to ensure entry has completed before
    // we try to wake-up
    schedule(wakeUpEvent, wake_up_tick);

    for (auto &b : banks) {
        // respect both causality and any existing bank
        // constraints, some banks could already have a
        // (auto) precharge scheduled
        b.wrAllowedAt = std::max(wake_up_tick + exit_delay, b.wrAllowedAt);
        b.rdAllowedAt = std::max(wake_up_tick + exit_delay, b.rdAllowedAt);
        b.preAllowedAt = std::max(wake_up_tick + exit_delay, b.preAllowedAt);
        b.actAllowedAt = std::max(wake_up_tick + exit_delay, b.actAllowedAt);
    }
    // Transitioning out of low power state, clear flag
    inLowPowerState = false;

    // push to DRAMPower
    // use pwrStateTrans for cases where we have a power event scheduled
    // to enter low power that has not yet been processed
    if (pwrStateTrans == PWR_ACT_PDN) {
        cmdList.push_back(Command(MemCommand::PUP_ACT, 0, wake_up_tick));
        DPRINTF(DRAMPower, "%llu,PUP_ACT,0,%d\n", divCeil(wake_up_tick,
                memory.tCK) - memory.timeStampOffset, rank);

    } else if (pwrStateTrans == PWR_PRE_PDN) {
        cmdList.push_back(Command(MemCommand::PUP_PRE, 0, wake_up_tick));
        DPRINTF(DRAMPower, "%llu,PUP_PRE,0,%d\n", divCeil(wake_up_tick,
                memory.tCK) - memory.timeStampOffset, rank);
    } else if (pwrStateTrans == PWR_SREF) {
        cmdList.push_back(Command(MemCommand::SREX, 0, wake_up_tick));
        DPRINTF(DRAMPower, "%llu,SREX,0,%d\n", divCeil(wake_up_tick,
                memory.tCK) - memory.timeStampOffset, rank);
    }
}

void
DRAMCtrl::Rank::processWakeUpEvent()
{
    // Should be in a power-down or self-refresh state
    assert((pwrState == PWR_ACT_PDN) || (pwrState == PWR_PRE_PDN) ||
           (pwrState == PWR_SREF));

    // Check current state to determine transition state
    if (pwrState == PWR_ACT_PDN) {
        // banks still open, transition to PWR_ACT
        schedulePowerEvent(PWR_ACT, curTick());
    } else {
        // transitioning from a precharge power-down or self-refresh state
        // banks are closed - transition to PWR_IDLE
        schedulePowerEvent(PWR_IDLE, curTick());
    }
}

void
DRAMCtrl::Rank::processPowerEvent()
{
    assert(curTick() >= pwrStateTick);
    // remember where we were, and for how long
    Tick duration = curTick() - pwrStateTick;
    PowerState prev_state = pwrState;

    // update the accounting
    stats.memoryStateTime[prev_state] += duration;

    // track to total idle time
    if ((prev_state == PWR_PRE_PDN) || (prev_state == PWR_ACT_PDN) ||
        (prev_state == PWR_SREF)) {
        stats.totalIdleTime += duration;
    }

    pwrState = pwrStateTrans;
    pwrStateTick = curTick();

    // if rank was refreshing, make sure to start scheduling requests again
    if (prev_state == PWR_REF) {
        // bus IDLED prior to REF
        // counter should be one for refresh command only
        assert(outstandingEvents == 1);
        // REF complete, decrement count and go back to IDLE
        --outstandingEvents;
        refreshState = REF_IDLE;

        DPRINTF(DRAMState, "Was refreshing for %llu ticks\n", duration);
        // if moving back to power-down after refresh
        if (pwrState != PWR_IDLE) {
            assert(pwrState == PWR_PRE_PDN);
            DPRINTF(DRAMState, "Switching to power down state after refreshing"
                    " rank %d at %llu tick\n", rank, curTick());
        }

        // completed refresh event, ensure next request is scheduled
        if (!memory.nextReqEvent.scheduled()) {
            DPRINTF(DRAM, "Scheduling next request after refreshing"
                           " rank %d\n", rank);
            schedule(memory.nextReqEvent, curTick());
        }
    }

    if ((pwrState == PWR_ACT) && (refreshState == REF_PD_EXIT)) {
        // have exited ACT PD
        assert(prev_state == PWR_ACT_PDN);

        // go back to REF event and close banks
        refreshState = REF_PRE;
        schedule(refreshEvent, curTick());
    } else if (pwrState == PWR_IDLE) {
        DPRINTF(DRAMState, "All banks precharged\n");
        if (prev_state == PWR_SREF) {
            // set refresh state to REF_SREF_EXIT, ensuring inRefIdleState
            // continues to return false during tXS after SREF exit
            // Schedule a refresh which kicks things back into action
            // when it finishes
            refreshState = REF_SREF_EXIT;
            schedule(refreshEvent, curTick() + memory.tXS);
        } else {
            // if we have a pending refresh, and are now moving to
            // the idle state, directly transition to, or schedule refresh
            if ((refreshState == REF_PRE) || (refreshState == REF_PD_EXIT)) {
                // ensure refresh is restarted only after final PRE command.
                // do not restart refresh if controller is in an intermediate
                // state, after PRE_PDN exit, when banks are IDLE but an
                // ACT is scheduled.
                if (!activateEvent.scheduled()) {
                    // there should be nothing waiting at this point
                    assert(!powerEvent.scheduled());
                    if (refreshState == REF_PD_EXIT) {
                        // exiting PRE PD, will be in IDLE until tXP expires
                        // and then should transition to PWR_REF state
                        assert(prev_state == PWR_PRE_PDN);
                        schedulePowerEvent(PWR_REF, curTick() + memory.tXP);
                    } else if (refreshState == REF_PRE) {
                        // can directly move to PWR_REF state and proceed below
                        pwrState = PWR_REF;
                    }
                } else {
                    // must have PRE scheduled to transition back to IDLE
                    // and re-kick off refresh
                    assert(prechargeEvent.scheduled());
                }
            }
        }
    }

    // transition to the refresh state and re-start refresh process
    // refresh state machine will schedule the next power state transition
    if (pwrState == PWR_REF) {
        // completed final PRE for refresh or exiting power-down
        assert(refreshState == REF_PRE || refreshState == REF_PD_EXIT);

        // exited PRE PD for refresh, with no pending commands
        // bypass auto-refresh and go straight to SREF, where memory
        // will issue refresh immediately upon entry
        if (pwrStatePostRefresh == PWR_PRE_PDN && isQueueEmpty() &&
           (memory.drainState() != DrainState::Draining) &&
           (memory.drainState() != DrainState::Drained) &&
           memory.enableDRAMPowerdown) {
            DPRINTF(DRAMState, "Rank %d bypassing refresh and transitioning "
                    "to self refresh at %11u tick\n", rank, curTick());
            powerDownSleep(PWR_SREF, curTick());

            // Since refresh was bypassed, remove event by decrementing count
            assert(outstandingEvents == 1);
            --outstandingEvents;

            // reset state back to IDLE temporarily until SREF is entered
            pwrState = PWR_IDLE;

        // Not bypassing refresh for SREF entry
        } else {
            DPRINTF(DRAMState, "Refreshing\n");

            // there should be nothing waiting at this point
            assert(!powerEvent.scheduled());

            // kick the refresh event loop into action again, and that
            // in turn will schedule a transition to the idle power
            // state once the refresh is done
            schedule(refreshEvent, curTick());

            // Banks transitioned to IDLE, start REF
            refreshState = REF_START;
        }
    }

}

void
DRAMCtrl::Rank::updatePowerStats()
{
    // All commands up to refresh have completed
    // flush cmdList to DRAMPower
    flushCmdList();

    // Call the function that calculates window energy at intermediate update
    // events like at refresh, stats dump as well as at simulation exit.
    // Window starts at the last time the calcWindowEnergy function was called
    // and is upto current time.
    power.powerlib.calcWindowEnergy(divCeil(curTick(), memory.tCK) -
                                    memory.timeStampOffset);

    // Get the energy from DRAMPower
    Data::MemoryPowerModel::Energy energy = power.powerlib.getEnergy();

    // The energy components inside the power lib are calculated over
    // the window so accumulate into the corresponding gem5 stat
    stats.actEnergy += energy.act_energy * memory.devicesPerRank;
    stats.preEnergy += energy.pre_energy * memory.devicesPerRank;
    stats.readEnergy += energy.read_energy * memory.devicesPerRank;
    stats.writeEnergy += energy.write_energy * memory.devicesPerRank;
    stats.refreshEnergy += energy.ref_energy * memory.devicesPerRank;
    stats.actBackEnergy += energy.act_stdby_energy * memory.devicesPerRank;
    stats.preBackEnergy += energy.pre_stdby_energy * memory.devicesPerRank;
    stats.actPowerDownEnergy += energy.f_act_pd_energy * memory.devicesPerRank;
    stats.prePowerDownEnergy += energy.f_pre_pd_energy * memory.devicesPerRank;
    stats.selfRefreshEnergy += energy.sref_energy * memory.devicesPerRank;

    // Accumulate window energy into the total energy.
    stats.totalEnergy += energy.window_energy * memory.devicesPerRank;
    // Average power must not be accumulated but calculated over the time
    // since last stats reset. SimClock::Frequency is tick period not tick
    // frequency.
    //              energy (pJ)     1e-9
    // power (mW) = ----------- * ----------
    //              time (tick)   tick_frequency
    stats.averagePower = (stats.totalEnergy.value() /
                          (curTick() - memory.lastStatsResetTick)) *
                         (SimClock::Frequency / 1000000000.0);
}

void
DRAMCtrl::Rank::computeStats()
{
    DPRINTF(DRAM,"Computing stats due to a dump callback\n");

    // Update the stats
    updatePowerStats();

    // final update of power state times
    stats.memoryStateTime[pwrState] += (curTick() - pwrStateTick);
    pwrStateTick = curTick();
}

void
DRAMCtrl::Rank::resetStats() {
    // The only way to clear the counters in DRAMPower is to call
    // calcWindowEnergy function as that then calls clearCounters. The
    // clearCounters method itself is private.
    power.powerlib.calcWindowEnergy(divCeil(curTick(), memory.tCK) -
                                    memory.timeStampOffset);

}

DRAMCtrl::DRAMStats::DRAMStats(DRAMCtrl &_dram)
    : Stats::Group(&_dram),
    dram(_dram),

    ADD_STAT(readReqs, "Number of read requests accepted"),
    ADD_STAT(writeReqs, "Number of write requests accepted"),

    ADD_STAT(readBursts,
             "Number of DRAM read bursts, "
             "including those serviced by the write queue"),
    ADD_STAT(writeBursts,
             "Number of DRAM write bursts, "
             "including those merged in the write queue"),
    ADD_STAT(servicedByWrQ,
             "Number of DRAM read bursts serviced by the write queue"),
    ADD_STAT(mergedWrBursts,
             "Number of DRAM write bursts merged with an existing one"),

    ADD_STAT(neitherReadNorWriteReqs,
             "Number of requests that are neither read nor write"),

    ADD_STAT(perBankRdBursts, "Per bank write bursts"),
    ADD_STAT(perBankWrBursts, "Per bank write bursts"),

    ADD_STAT(avgRdQLen, "Average read queue length when enqueuing"),
    ADD_STAT(avgWrQLen, "Average write queue length when enqueuing"),

    ADD_STAT(totQLat, "Total ticks spent queuing"),
    ADD_STAT(totBusLat, "Total ticks spent in databus transfers"),
    ADD_STAT(totMemAccLat,
             "Total ticks spent from burst creation until serviced "
             "by the DRAM"),
    ADD_STAT(avgQLat, "Average queueing delay per DRAM burst"),
    ADD_STAT(avgBusLat, "Average bus latency per DRAM burst"),
    ADD_STAT(avgMemAccLat, "Average memory access latency per DRAM burst"),

    ADD_STAT(numRdRetry, "Number of times read queue was full causing retry"),
    ADD_STAT(numWrRetry, "Number of times write queue was full causing retry"),

    ADD_STAT(readRowHits, "Number of row buffer hits during reads"),
    ADD_STAT(writeRowHits, "Number of row buffer hits during writes"),
    ADD_STAT(readRowHitRate, "Row buffer hit rate for reads"),
    ADD_STAT(writeRowHitRate, "Row buffer hit rate for writes"),

    ADD_STAT(readPktSize, "Read request sizes (log2)"),
    ADD_STAT(writePktSize, "Write request sizes (log2)"),

    ADD_STAT(rdQLenPdf, "What read queue length does an incoming req see"),
    ADD_STAT(wrQLenPdf, "What write queue length does an incoming req see"),

    ADD_STAT(bytesPerActivate, "Bytes accessed per row activation"),

    ADD_STAT(rdPerTurnAround,
             "Reads before turning the bus around for writes"),
    ADD_STAT(wrPerTurnAround,
             "Writes before turning the bus around for reads"),

    ADD_STAT(bytesReadDRAM, "Total number of bytes read from DRAM"),
    ADD_STAT(bytesReadWrQ, "Total number of bytes read from write queue"),
    ADD_STAT(bytesWritten, "Total number of bytes written to DRAM"),
    ADD_STAT(bytesReadSys, "Total read bytes from the system interface side"),
    ADD_STAT(bytesWrittenSys,
             "Total written bytes from the system interface side"),

    ADD_STAT(avgRdBW, "Average DRAM read bandwidth in MiByte/s"),
    ADD_STAT(avgWrBW, "Average achieved write bandwidth in MiByte/s"),
    ADD_STAT(avgRdBWSys, "Average system read bandwidth in MiByte/s"),
    ADD_STAT(avgWrBWSys, "Average system write bandwidth in MiByte/s"),
    ADD_STAT(peakBW, "Theoretical peak bandwidth in MiByte/s"),

    ADD_STAT(busUtil, "Data bus utilization in percentage"),
    ADD_STAT(busUtilRead, "Data bus utilization in percentage for reads"),
    ADD_STAT(busUtilWrite, "Data bus utilization in percentage for writes"),

    ADD_STAT(totGap, "Total gap between requests"),
    ADD_STAT(avgGap, "Average gap between requests"),

    ADD_STAT(masterReadBytes, "Per-master bytes read from memory"),
    ADD_STAT(masterWriteBytes, "Per-master bytes write to memory"),
    ADD_STAT(masterReadRate,
             "Per-master bytes read from memory rate (Bytes/sec)"),
    ADD_STAT(masterWriteRate,
             "Per-master bytes write to memory rate (Bytes/sec)"),
    ADD_STAT(masterReadAccesses,
             "Per-master read serviced memory accesses"),
    ADD_STAT(masterWriteAccesses,
             "Per-master write serviced memory accesses"),
    ADD_STAT(masterReadTotalLat,
             "Per-master read total memory access latency"),
    ADD_STAT(masterWriteTotalLat,
             "Per-master write total memory access latency"),
    ADD_STAT(masterReadAvgLat,
             "Per-master read average memory access latency"),
    ADD_STAT(masterWriteAvgLat,
             "Per-master write average memory access latency"),

    ADD_STAT(pageHitRate, "Row buffer hit rate, read and write combined")
{
}

void
DRAMCtrl::DRAMStats::regStats()
{
    using namespace Stats;

    assert(dram._system);
    const auto max_masters = dram._system->maxMasters();

    perBankRdBursts.init(dram.banksPerRank * dram.ranksPerChannel);
    perBankWrBursts.init(dram.banksPerRank * dram.ranksPerChannel);

    avgRdQLen.precision(2);
    avgWrQLen.precision(2);
    avgQLat.precision(2);
    avgBusLat.precision(2);
    avgMemAccLat.precision(2);

    readRowHitRate.precision(2);
    writeRowHitRate.precision(2);

    readPktSize.init(ceilLog2(dram.burstSize) + 1);
    writePktSize.init(ceilLog2(dram.burstSize) + 1);

    rdQLenPdf.init(dram.readBufferSize);
    wrQLenPdf.init(dram.writeBufferSize);

    bytesPerActivate
        .init(dram.maxAccessesPerRow ?
              dram.maxAccessesPerRow : dram.rowBufferSize)
        .flags(nozero);

    rdPerTurnAround
        .init(dram.readBufferSize)
        .flags(nozero);
    wrPerTurnAround
        .init(dram.writeBufferSize)
        .flags(nozero);

    avgRdBW.precision(2);
    avgWrBW.precision(2);
    avgRdBWSys.precision(2);
    avgWrBWSys.precision(2);
    peakBW.precision(2);
    busUtil.precision(2);
    avgGap.precision(2);
    busUtilWrite.precision(2);
    pageHitRate.precision(2);


    // per-master bytes read and written to memory
    masterReadBytes
        .init(max_masters)
        .flags(nozero | nonan);

    masterWriteBytes
        .init(max_masters)
        .flags(nozero | nonan);

    // per-master bytes read and written to memory rate
    masterReadRate
        .flags(nozero | nonan)
        .precision(12);

    masterReadAccesses
        .init(max_masters)
        .flags(nozero);

    masterWriteAccesses
        .init(max_masters)
        .flags(nozero);

    masterReadTotalLat
        .init(max_masters)
        .flags(nozero | nonan);

    masterReadAvgLat
        .flags(nonan)
        .precision(2);


    busUtilRead
        .precision(2);

    masterWriteRate
        .flags(nozero | nonan)
        .precision(12);

    masterWriteTotalLat
        .init(max_masters)
        .flags(nozero | nonan);

    masterWriteAvgLat
        .flags(nonan)
        .precision(2);

    for (int i = 0; i < max_masters; i++) {
        const std::string master = dram._system->getMasterName(i);
        masterReadBytes.subname(i, master);
        masterReadRate.subname(i, master);
        masterWriteBytes.subname(i, master);
        masterWriteRate.subname(i, master);
        masterReadAccesses.subname(i, master);
        masterWriteAccesses.subname(i, master);
        masterReadTotalLat.subname(i, master);
        masterReadAvgLat.subname(i, master);
        masterWriteTotalLat.subname(i, master);
        masterWriteAvgLat.subname(i, master);
    }

    // Formula stats
    avgQLat = totQLat / (readBursts - servicedByWrQ);
    avgBusLat = totBusLat / (readBursts - servicedByWrQ);
    avgMemAccLat = totMemAccLat / (readBursts - servicedByWrQ);

    readRowHitRate = (readRowHits / (readBursts - servicedByWrQ)) * 100;
    writeRowHitRate = (writeRowHits / (writeBursts - mergedWrBursts)) * 100;

    avgRdBW = (bytesReadDRAM / 1000000) / simSeconds;
    avgWrBW = (bytesWritten / 1000000) / simSeconds;
    avgRdBWSys = (bytesReadSys / 1000000) / simSeconds;
    avgWrBWSys = (bytesWrittenSys / 1000000) / simSeconds;
    peakBW = (SimClock::Frequency / dram.burstDataCycles) *
              dram.burstSize / 1000000;

    busUtil = (avgRdBW + avgWrBW) / peakBW * 100;

    avgGap = totGap / (readReqs + writeReqs);

    busUtilRead = avgRdBW / peakBW * 100;
    busUtilWrite = avgWrBW / peakBW * 100;

    pageHitRate = (writeRowHits + readRowHits) /
        (writeBursts - mergedWrBursts + readBursts - servicedByWrQ) * 100;

    masterReadRate = masterReadBytes / simSeconds;
    masterWriteRate = masterWriteBytes / simSeconds;
    masterReadAvgLat = masterReadTotalLat / masterReadAccesses;
    masterWriteAvgLat = masterWriteTotalLat / masterWriteAccesses;
}

void
DRAMCtrl::DRAMStats::resetStats()
{
    dram.lastStatsResetTick = curTick();
}

DRAMCtrl::RankStats::RankStats(DRAMCtrl &_memory, Rank &_rank)
    : Stats::Group(&_memory, csprintf("rank%d", _rank.rank).c_str()),
    rank(_rank),

    ADD_STAT(actEnergy, "Energy for activate commands per rank (pJ)"),
    ADD_STAT(preEnergy, "Energy for precharge commands per rank (pJ)"),
    ADD_STAT(readEnergy, "Energy for read commands per rank (pJ)"),
    ADD_STAT(writeEnergy, "Energy for write commands per rank (pJ)"),
    ADD_STAT(refreshEnergy, "Energy for refresh commands per rank (pJ)"),
    ADD_STAT(actBackEnergy, "Energy for active background per rank (pJ)"),
    ADD_STAT(preBackEnergy, "Energy for precharge background per rank (pJ)"),
    ADD_STAT(actPowerDownEnergy,
             "Energy for active power-down per rank (pJ)"),
    ADD_STAT(prePowerDownEnergy,
             "Energy for precharge power-down per rank (pJ)"),
    ADD_STAT(selfRefreshEnergy, "Energy for self refresh per rank (pJ)"),

    ADD_STAT(totalEnergy, "Total energy per rank (pJ)"),
    ADD_STAT(averagePower, "Core power per rank (mW)"),

    ADD_STAT(totalIdleTime, "Total Idle time Per DRAM Rank"),
    ADD_STAT(memoryStateTime, "Time in different power states")
{
}

void
DRAMCtrl::RankStats::regStats()
{
    Stats::Group::regStats();

    memoryStateTime.init(6);
    memoryStateTime.subname(0, "IDLE");
    memoryStateTime.subname(1, "REF");
    memoryStateTime.subname(2, "SREF");
    memoryStateTime.subname(3, "PRE_PDN");
    memoryStateTime.subname(4, "ACT");
    memoryStateTime.subname(5, "ACT_PDN");
}

void
DRAMCtrl::RankStats::resetStats()
{
    Stats::Group::resetStats();

    rank.resetStats();
}

void
DRAMCtrl::RankStats::preDumpStats()
{
    Stats::Group::preDumpStats();

    rank.computeStats();
}

void
DRAMCtrl::recvFunctional(PacketPtr pkt)
{
    // rely on the abstract memory
    functionalAccess(pkt);
}

Port &
DRAMCtrl::getPort(const string &if_name, PortID idx)
{
    if (if_name != "port") {
        return QoS::MemCtrl::getPort(if_name, idx);
    } else {
        return port;
    }
}

DrainState
DRAMCtrl::drain()
{
    // if there is anything in any of our internal queues, keep track
    // of that as well
    if (!(!totalWriteQueueSize && !totalReadQueueSize && respQueue.empty() &&
          allRanksDrained())) {

        DPRINTF(Drain, "DRAM controller not drained, write: %d, read: %d,"
                " resp: %d\n", totalWriteQueueSize, totalReadQueueSize,
                respQueue.size());

        // the only queue that is not drained automatically over time
        // is the write queue, thus kick things into action if needed
        if (!totalWriteQueueSize && !nextReqEvent.scheduled()) {
            schedule(nextReqEvent, curTick());
        }

        // also need to kick off events to exit self-refresh
        for (auto r : ranks) {
            // force self-refresh exit, which in turn will issue auto-refresh
            if (r->pwrState == PWR_SREF) {
                DPRINTF(DRAM,"Rank%d: Forcing self-refresh wakeup in drain\n",
                        r->rank);
                r->scheduleWakeUpEvent(tXS);
            }
        }

        return DrainState::Draining;
    } else {
        return DrainState::Drained;
    }
}

bool
DRAMCtrl::allRanksDrained() const
{
    // true until proven false
    bool all_ranks_drained = true;
    for (auto r : ranks) {
        // then verify that the power state is IDLE ensuring all banks are
        // closed and rank is not in a low power state. Also verify that rank
        // is idle from a refresh point of view.
        all_ranks_drained = r->inPwrIdleState() && r->inRefIdleState() &&
            all_ranks_drained;
    }
    return all_ranks_drained;
}

void
DRAMCtrl::drainResume()
{
    if (!isTimingMode && system()->isTimingMode()) {
        // if we switched to timing mode, kick things into action,
        // and behave as if we restored from a checkpoint
        startup();
    } else if (isTimingMode && !system()->isTimingMode()) {
        // if we switch from timing mode, stop the refresh events to
        // not cause issues with KVM
        for (auto r : ranks) {
            r->suspend();
        }
    }

    // update the mode
    isTimingMode = system()->isTimingMode();
}

DRAMCtrl::MemoryPort::MemoryPort(const std::string& name, DRAMCtrl& _memory)
    : QueuedSlavePort(name, &_memory, queue), queue(_memory, *this, true),
      memory(_memory)
{ }

AddrRangeList
DRAMCtrl::MemoryPort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(memory.getAddrRange());
    return ranges;
}

void
DRAMCtrl::MemoryPort::recvFunctional(PacketPtr pkt)
{
    pkt->pushLabel(memory.name());

    if (!queue.trySatisfyFunctional(pkt)) {
        // Default implementation of SimpleTimingPort::recvFunctional()
        // calls recvAtomic() and throws away the latency; we can save a
        // little here by just not calculating the latency.
        memory.recvFunctional(pkt);
    }

    pkt->popLabel();
}

Tick
DRAMCtrl::MemoryPort::recvAtomic(PacketPtr pkt)
{
    return memory.recvAtomic(pkt);
}

bool
DRAMCtrl::MemoryPort::recvTimingReq(PacketPtr pkt)
{
    // pass it to the memory controller
    return memory.recvTimingReq(pkt);
}

DRAMCtrl*
DRAMCtrlParams::create()
{
    return new DRAMCtrl(this);
}
