/*
 * Copyright (c) 2010-2014 ARM Limited
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
 *
 * Authors: Andreas Hansson
 *          Ani Udipi
 *          Neha Agarwal
 */

#include "base/bitfield.hh"
#include "base/trace.hh"
#include "debug/DRAM.hh"
#include "debug/DRAMState.hh"
#include "debug/Drain.hh"
#include "mem/dram_ctrl.hh"
#include "sim/system.hh"

using namespace std;

DRAMCtrl::DRAMCtrl(const DRAMCtrlParams* p) :
    AbstractMemory(p),
    port(name() + ".port", *this),
    retryRdReq(false), retryWrReq(false),
    rowHitFlag(false), busState(READ),
    nextReqEvent(this), respondEvent(this), activateEvent(this),
    prechargeEvent(this), refreshEvent(this), powerEvent(this),
    drainManager(NULL),
    deviceBusWidth(p->device_bus_width), burstLength(p->burst_length),
    deviceRowBufferSize(p->device_rowbuffer_size),
    devicesPerRank(p->devices_per_rank),
    burstSize((devicesPerRank * burstLength * deviceBusWidth) / 8),
    rowBufferSize(devicesPerRank * deviceRowBufferSize),
    columnsPerRowBuffer(rowBufferSize / burstSize),
    ranksPerChannel(p->ranks_per_channel),
    banksPerRank(p->banks_per_rank), channels(p->channels), rowsPerBank(0),
    readBufferSize(p->read_buffer_size),
    writeBufferSize(p->write_buffer_size),
    writeHighThreshold(writeBufferSize * p->write_high_thresh_perc / 100.0),
    writeLowThreshold(writeBufferSize * p->write_low_thresh_perc / 100.0),
    minWritesPerSwitch(p->min_writes_per_switch),
    writesThisTime(0), readsThisTime(0),
    tWTR(p->tWTR), tRTW(p->tRTW), tBURST(p->tBURST),
    tRCD(p->tRCD), tCL(p->tCL), tRP(p->tRP), tRAS(p->tRAS),
    tRFC(p->tRFC), tREFI(p->tREFI), tRRD(p->tRRD),
    tXAW(p->tXAW), activationLimit(p->activation_limit),
    memSchedPolicy(p->mem_sched_policy), addrMapping(p->addr_mapping),
    pageMgmt(p->page_policy),
    maxAccessesPerRow(p->max_accesses_per_row),
    frontendLatency(p->static_frontend_latency),
    backendLatency(p->static_backend_latency),
    busBusyUntil(0), refreshDueAt(0), refreshState(REF_IDLE),
    pwrStateTrans(PWR_IDLE), pwrState(PWR_IDLE), prevArrival(0),
    nextReqTime(0), pwrStateTick(0), numBanksActive(0)
{
    // create the bank states based on the dimensions of the ranks and
    // banks
    banks.resize(ranksPerChannel);
    actTicks.resize(ranksPerChannel);
    for (size_t c = 0; c < ranksPerChannel; ++c) {
        banks[c].resize(banksPerRank);
        actTicks[c].resize(activationLimit, 0);
    }

    // perform a basic check of the write thresholds
    if (p->write_low_thresh_perc >= p->write_high_thresh_perc)
        fatal("Write buffer low threshold %d must be smaller than the "
              "high threshold %d\n", p->write_low_thresh_perc,
              p->write_high_thresh_perc);

    // determine the rows per bank by looking at the total capacity
    uint64_t capacity = ULL(1) << ceilLog2(AbstractMemory::size());

    DPRINTF(DRAM, "Memory capacity %lld (%lld) bytes\n", capacity,
            AbstractMemory::size());

    DPRINTF(DRAM, "Row buffer size %d bytes with %d columns per row buffer\n",
            rowBufferSize, columnsPerRowBuffer);

    rowsPerBank = capacity / (rowBufferSize * banksPerRank * ranksPerChannel);

    if (range.interleaved()) {
        if (channels != range.stripes())
            fatal("%s has %d interleaved address stripes but %d channel(s)\n",
                  name(), range.stripes(), channels);

        if (addrMapping == Enums::RoRaBaChCo) {
            if (rowBufferSize != range.granularity()) {
                fatal("Interleaving of %s doesn't match RoRaBaChCo "
                      "address map\n", name());
            }
        } else if (addrMapping == Enums::RoRaBaCoCh) {
            if (system()->cacheLineSize() != range.granularity()) {
                fatal("Interleaving of %s doesn't match RoRaBaCoCh "
                      "address map\n", name());
            }
        } else if (addrMapping == Enums::RoCoRaBaCh) {
            if (system()->cacheLineSize() != range.granularity())
                fatal("Interleaving of %s doesn't match RoCoRaBaCh "
                      "address map\n", name());
        }
    }

    // some basic sanity checks
    if (tREFI <= tRP || tREFI <= tRFC) {
        fatal("tREFI (%d) must be larger than tRP (%d) and tRFC (%d)\n",
              tREFI, tRP, tRFC);
    }
}

void
DRAMCtrl::init()
{
    if (!port.isConnected()) {
        fatal("DRAMCtrl %s is unconnected!\n", name());
    } else {
        port.sendRangeChange();
    }
}

void
DRAMCtrl::startup()
{
    // update the start tick for the precharge accounting to the
    // current tick
    pwrStateTick = curTick();

    // shift the bus busy time sufficiently far ahead that we never
    // have to worry about negative values when computing the time for
    // the next request, this will add an insignificant bubble at the
    // start of simulation
    busBusyUntil = curTick() + tRP + tRCD + tCL;

    // print the configuration of the controller
    printParams();

    // kick off the refresh, and give ourselves enough time to
    // precharge
    schedule(refreshEvent, curTick() + tREFI - tRP);
}

Tick
DRAMCtrl::recvAtomic(PacketPtr pkt)
{
    DPRINTF(DRAM, "recvAtomic: %s 0x%x\n", pkt->cmdString(), pkt->getAddr());

    // do the actual memory access and turn the packet into a response
    access(pkt);

    Tick latency = 0;
    if (!pkt->memInhibitAsserted() && pkt->hasData()) {
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
            readBufferSize, readQueue.size() + respQueue.size(),
            neededEntries);

    return
        (readQueue.size() + respQueue.size() + neededEntries) > readBufferSize;
}

bool
DRAMCtrl::writeQueueFull(unsigned int neededEntries) const
{
    DPRINTF(DRAM, "Write queue limit %d, current size %d, entries needed %d\n",
            writeBufferSize, writeQueue.size(), neededEntries);
    return (writeQueue.size() + neededEntries) > writeBufferSize;
}

DRAMCtrl::DRAMPacket*
DRAMCtrl::decodeAddr(PacketPtr pkt, Addr dramPktAddr, unsigned size,
                       bool isRead)
{
    // decode the address based on the address mapping scheme, with
    // Ro, Ra, Co, Ba and Ch denoting row, rank, column, bank and
    // channel, respectively
    uint8_t rank;
    uint8_t bank;
    uint16_t row;

    // truncate the address to the access granularity
    Addr addr = dramPktAddr / burstSize;

    // we have removed the lowest order address bits that denote the
    // position within the column
    if (addrMapping == Enums::RoRaBaChCo) {
        // the lowest order bits denote the column to ensure that
        // sequential cache lines occupy the same row
        addr = addr / columnsPerRowBuffer;

        // take out the channel part of the address
        addr = addr / channels;

        // after the channel bits, get the bank bits to interleave
        // over the banks
        bank = addr % banksPerRank;
        addr = addr / banksPerRank;

        // after the bank, we get the rank bits which thus interleaves
        // over the ranks
        rank = addr % ranksPerChannel;
        addr = addr / ranksPerChannel;

        // lastly, get the row bits
        row = addr % rowsPerBank;
        addr = addr / rowsPerBank;
    } else if (addrMapping == Enums::RoRaBaCoCh) {
        // take out the channel part of the address
        addr = addr / channels;

        // next, the column
        addr = addr / columnsPerRowBuffer;

        // after the column bits, we get the bank bits to interleave
        // over the banks
        bank = addr % banksPerRank;
        addr = addr / banksPerRank;

        // after the bank, we get the rank bits which thus interleaves
        // over the ranks
        rank = addr % ranksPerChannel;
        addr = addr / ranksPerChannel;

        // lastly, get the row bits
        row = addr % rowsPerBank;
        addr = addr / rowsPerBank;
    } else if (addrMapping == Enums::RoCoRaBaCh) {
        // optimise for closed page mode and utilise maximum
        // parallelism of the DRAM (at the cost of power)

        // take out the channel part of the address, not that this has
        // to match with how accesses are interleaved between the
        // controllers in the address mapping
        addr = addr / channels;

        // start with the bank bits, as this provides the maximum
        // opportunity for parallelism between requests
        bank = addr % banksPerRank;
        addr = addr / banksPerRank;

        // next get the rank bits
        rank = addr % ranksPerChannel;
        addr = addr / ranksPerChannel;

        // next the column bits which we do not need to keep track of
        // and simply skip past
        addr = addr / columnsPerRowBuffer;

        // lastly, get the row bits
        row = addr % rowsPerBank;
        addr = addr / rowsPerBank;
    } else
        panic("Unknown address mapping policy chosen!");

    assert(rank < ranksPerChannel);
    assert(bank < banksPerRank);
    assert(row < rowsPerBank);

    DPRINTF(DRAM, "Address: %lld Rank %d Bank %d Row %d\n",
            dramPktAddr, rank, bank, row);

    // create the corresponding DRAM packet with the entry time and
    // ready time set to the current tick, the latter will be updated
    // later
    uint16_t bank_id = banksPerRank * rank + bank;
    return new DRAMPacket(pkt, isRead, rank, bank, row, bank_id, dramPktAddr,
                          size, banks[rank][bank]);
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
    Addr addr = pkt->getAddr();
    unsigned pktsServicedByWrQ = 0;
    BurstHelper* burst_helper = NULL;
    for (int cnt = 0; cnt < pktCount; ++cnt) {
        unsigned size = std::min((addr | (burstSize - 1)) + 1,
                        pkt->getAddr() + pkt->getSize()) - addr;
        readPktSize[ceilLog2(size)]++;
        readBursts++;

        // First check write buffer to see if the data is already at
        // the controller
        bool foundInWrQ = false;
        for (auto i = writeQueue.begin(); i != writeQueue.end(); ++i) {
            // check if the read is subsumed in the write entry we are
            // looking at
            if ((*i)->addr <= addr &&
                (addr + size) <= ((*i)->addr + (*i)->size)) {
                foundInWrQ = true;
                servicedByWrQ++;
                pktsServicedByWrQ++;
                DPRINTF(DRAM, "Read to addr %lld with size %d serviced by "
                        "write queue\n", addr, size);
                bytesReadWrQ += burstSize;
                break;
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
            rdQLenPdf[readQueue.size() + respQueue.size()]++;

            DPRINTF(DRAM, "Adding to read queue\n");

            readQueue.push_back(dram_pkt);

            // Update stats
            avgRdQLen = readQueue.size() + respQueue.size();
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
    Addr addr = pkt->getAddr();
    for (int cnt = 0; cnt < pktCount; ++cnt) {
        unsigned size = std::min((addr | (burstSize - 1)) + 1,
                        pkt->getAddr() + pkt->getSize()) - addr;
        writePktSize[ceilLog2(size)]++;
        writeBursts++;

        // see if we can merge with an existing item in the write
        // queue and keep track of whether we have merged or not so we
        // can stop at that point and also avoid enqueueing a new
        // request
        bool merged = false;
        auto w = writeQueue.begin();

        while(!merged && w != writeQueue.end()) {
            // either of the two could be first, if they are the same
            // it does not matter which way we go
            if ((*w)->addr >= addr) {
                // the existing one starts after the new one, figure
                // out where the new one ends with respect to the
                // existing one
                if ((addr + size) >= ((*w)->addr + (*w)->size)) {
                    // check if the existing one is completely
                    // subsumed in the new one
                    DPRINTF(DRAM, "Merging write covering existing burst\n");
                    merged = true;
                    // update both the address and the size
                    (*w)->addr = addr;
                    (*w)->size = size;
                } else if ((addr + size) >= (*w)->addr &&
                           ((*w)->addr + (*w)->size - addr) <= burstSize) {
                    // the new one is just before or partially
                    // overlapping with the existing one, and together
                    // they fit within a burst
                    DPRINTF(DRAM, "Merging write before existing burst\n");
                    merged = true;
                    // the existing queue item needs to be adjusted with
                    // respect to both address and size
                    (*w)->size = (*w)->addr + (*w)->size - addr;
                    (*w)->addr = addr;
                }
            } else {
                // the new one starts after the current one, figure
                // out where the existing one ends with respect to the
                // new one
                if (((*w)->addr + (*w)->size) >= (addr + size)) {
                    // check if the new one is completely subsumed in the
                    // existing one
                    DPRINTF(DRAM, "Merging write into existing burst\n");
                    merged = true;
                    // no adjustments necessary
                } else if (((*w)->addr + (*w)->size) >= addr &&
                           (addr + size - (*w)->addr) <= burstSize) {
                    // the existing one is just before or partially
                    // overlapping with the new one, and together
                    // they fit within a burst
                    DPRINTF(DRAM, "Merging write after existing burst\n");
                    merged = true;
                    // the address is right, and only the size has
                    // to be adjusted
                    (*w)->size = addr + size - (*w)->addr;
                }
            }
            ++w;
        }

        // if the item was not merged we need to create a new write
        // and enqueue it
        if (!merged) {
            DRAMPacket* dram_pkt = decodeAddr(pkt, addr, size, false);

            assert(writeQueue.size() < writeBufferSize);
            wrQLenPdf[writeQueue.size()]++;

            DPRINTF(DRAM, "Adding to write queue\n");

            writeQueue.push_back(dram_pkt);

            // Update stats
            avgWrQLen = writeQueue.size();
        } else {
            // keep track of the fact that this burst effectively
            // disappeared as it was merged with an existing one
            mergedWrBursts++;
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
DRAMCtrl::printParams() const
{
    // Sanity check print of important parameters
    DPRINTF(DRAM,
            "Memory controller %s physical organization\n"      \
            "Number of devices per rank   %d\n"                 \
            "Device bus width (in bits)   %d\n"                 \
            "DRAM data bus burst (bytes)  %d\n"                 \
            "Row buffer size (bytes)      %d\n"                 \
            "Columns per row buffer       %d\n"                 \
            "Rows    per bank             %d\n"                 \
            "Banks   per rank             %d\n"                 \
            "Ranks   per channel          %d\n"                 \
            "Total mem capacity (bytes)   %u\n",
            name(), devicesPerRank, deviceBusWidth, burstSize, rowBufferSize,
            columnsPerRowBuffer, rowsPerBank, banksPerRank, ranksPerChannel,
            rowBufferSize * rowsPerBank * banksPerRank * ranksPerChannel);

    string scheduler =  memSchedPolicy == Enums::fcfs ? "FCFS" : "FR-FCFS";
    string address_mapping = addrMapping == Enums::RoRaBaChCo ? "RoRaBaChCo" :
        (addrMapping == Enums::RoRaBaCoCh ? "RoRaBaCoCh" : "RoCoRaBaCh");
    string page_policy = pageMgmt == Enums::open ? "OPEN" :
        (pageMgmt == Enums::open_adaptive ? "OPEN (adaptive)" :
        (pageMgmt == Enums::close_adaptive ? "CLOSE (adaptive)" : "CLOSE"));

    DPRINTF(DRAM,
            "Memory controller %s characteristics\n"    \
            "Read buffer size     %d\n"                 \
            "Write buffer size    %d\n"                 \
            "Write high thresh    %d\n"                 \
            "Write low thresh     %d\n"                 \
            "Scheduler            %s\n"                 \
            "Address mapping      %s\n"                 \
            "Page policy          %s\n",
            name(), readBufferSize, writeBufferSize, writeHighThreshold,
            writeLowThreshold, scheduler, address_mapping, page_policy);

    DPRINTF(DRAM, "Memory controller %s timing specs\n" \
            "tRCD      %d ticks\n"                        \
            "tCL       %d ticks\n"                        \
            "tRP       %d ticks\n"                        \
            "tBURST    %d ticks\n"                        \
            "tRFC      %d ticks\n"                        \
            "tREFI     %d ticks\n"                        \
            "tWTR      %d ticks\n"                        \
            "tRTW      %d ticks\n"                        \
            "tXAW (%d) %d ticks\n",
            name(), tRCD, tCL, tRP, tBURST, tRFC, tREFI, tWTR,
            tRTW, activationLimit, tXAW);
}

void
DRAMCtrl::printQs() const {
    DPRINTF(DRAM, "===READ QUEUE===\n\n");
    for (auto i = readQueue.begin() ;  i != readQueue.end() ; ++i) {
        DPRINTF(DRAM, "Read %lu\n", (*i)->addr);
    }
    DPRINTF(DRAM, "\n===RESP QUEUE===\n\n");
    for (auto i = respQueue.begin() ;  i != respQueue.end() ; ++i) {
        DPRINTF(DRAM, "Response %lu\n", (*i)->addr);
    }
    DPRINTF(DRAM, "\n===WRITE QUEUE===\n\n");
    for (auto i = writeQueue.begin() ;  i != writeQueue.end() ; ++i) {
        DPRINTF(DRAM, "Write %lu\n", (*i)->addr);
    }
}

bool
DRAMCtrl::recvTimingReq(PacketPtr pkt)
{
    /// @todo temporary hack to deal with memory corruption issues until
    /// 4-phase transactions are complete
    for (int x = 0; x < pendingDelete.size(); x++)
        delete pendingDelete[x];
    pendingDelete.clear();

    // This is where we enter from the outside world
    DPRINTF(DRAM, "recvTimingReq: request %s addr %lld size %d\n",
            pkt->cmdString(), pkt->getAddr(), pkt->getSize());

    // simply drop inhibited packets for now
    if (pkt->memInhibitAsserted()) {
        DPRINTF(DRAM, "Inhibited packet -- Dropping it now\n");
        pendingDelete.push_back(pkt);
        return true;
    }

    // Calc avg gap between requests
    if (prevArrival != 0) {
        totGap += curTick() - prevArrival;
    }
    prevArrival = curTick();


    // Find out how many dram packets a pkt translates to
    // If the burst size is equal or larger than the pkt size, then a pkt
    // translates to only one dram packet. Otherwise, a pkt translates to
    // multiple dram packets
    unsigned size = pkt->getSize();
    unsigned offset = pkt->getAddr() & (burstSize - 1);
    unsigned int dram_pkt_count = divCeil(offset + size, burstSize);

    // check local buffers and do not accept if full
    if (pkt->isRead()) {
        assert(size != 0);
        if (readQueueFull(dram_pkt_count)) {
            DPRINTF(DRAM, "Read queue full, not accepting\n");
            // remember that we have to retry this port
            retryRdReq = true;
            numRdRetry++;
            return false;
        } else {
            addToReadQueue(pkt, dram_pkt_count);
            readReqs++;
            bytesReadSys += size;
        }
    } else if (pkt->isWrite()) {
        assert(size != 0);
        if (writeQueueFull(dram_pkt_count)) {
            DPRINTF(DRAM, "Write queue full, not accepting\n");
            // remember that we have to retry this port
            retryWrReq = true;
            numWrRetry++;
            return false;
        } else {
            addToWriteQueue(pkt, dram_pkt_count);
            writeReqs++;
            bytesWrittenSys += size;
        }
    } else {
        DPRINTF(DRAM,"Neither read nor write, ignore timing\n");
        neitherReadNorWrite++;
        accessAndRespond(pkt, 1);
    }

    return true;
}

void
DRAMCtrl::processRespondEvent()
{
    DPRINTF(DRAM,
            "processRespondEvent(): Some req has reached its readyTime\n");

    DRAMPacket* dram_pkt = respQueue.front();

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

    delete respQueue.front();
    respQueue.pop_front();

    if (!respQueue.empty()) {
        assert(respQueue.front()->readyTime >= curTick());
        assert(!respondEvent.scheduled());
        schedule(respondEvent, respQueue.front()->readyTime);
    } else {
        // if there is nothing left in any queue, signal a drain
        if (writeQueue.empty() && readQueue.empty() &&
            drainManager) {
            drainManager->signalDrainDone();
            drainManager = NULL;
        }
    }

    // We have made a location in the queue available at this point,
    // so if there is a read that was forced to wait, retry now
    if (retryRdReq) {
        retryRdReq = false;
        port.sendRetry();
    }
}

void
DRAMCtrl::chooseNext(std::deque<DRAMPacket*>& queue)
{
    // This method does the arbitration between requests. The chosen
    // packet is simply moved to the head of the queue. The other
    // methods know that this is the place to look. For example, with
    // FCFS, this method does nothing
    assert(!queue.empty());

    if (queue.size() == 1) {
        DPRINTF(DRAM, "Single request, nothing to do\n");
        return;
    }

    if (memSchedPolicy == Enums::fcfs) {
        // Do nothing, since the correct request is already head
    } else if (memSchedPolicy == Enums::frfcfs) {
        reorderQueue(queue);
    } else
        panic("No scheduling policy chosen\n");
}

void
DRAMCtrl::reorderQueue(std::deque<DRAMPacket*>& queue)
{
    // Only determine this when needed
    uint64_t earliest_banks = 0;

    // Search for row hits first, if no row hit is found then schedule the
    // packet to one of the earliest banks available
    bool found_earliest_pkt = false;
    auto selected_pkt_it = queue.begin();

    for (auto i = queue.begin(); i != queue.end() ; ++i) {
        DRAMPacket* dram_pkt = *i;
        const Bank& bank = dram_pkt->bankRef;
        // Check if it is a row hit
        if (bank.openRow == dram_pkt->row) {
            DPRINTF(DRAM, "Row buffer hit\n");
            selected_pkt_it = i;
            break;
        } else if (!found_earliest_pkt) {
            // No row hit, go for first ready
            if (earliest_banks == 0)
                earliest_banks = minBankFreeAt(queue);

            // Bank is ready or is the first available bank
            if (bank.freeAt <= curTick() ||
                bits(earliest_banks, dram_pkt->bankId, dram_pkt->bankId)) {
                // Remember the packet to be scheduled to one of the earliest
                // banks available
                selected_pkt_it = i;
                found_earliest_pkt = true;
            }
        }
    }

    DRAMPacket* selected_pkt = *selected_pkt_it;
    queue.erase(selected_pkt_it);
    queue.push_front(selected_pkt);
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

        // @todo someone should pay for this
        pkt->busFirstWordDelay = pkt->busLastWordDelay = 0;

        // queue the packet in the response queue to be sent out after
        // the static latency has passed
        port.schedTimingResp(pkt, curTick() + static_latency);
    } else {
        // @todo the packet is going to be deleted, and the DRAMPacket
        // is still having a pointer to it
        pendingDelete.push_back(pkt);
    }

    DPRINTF(DRAM, "Done\n");

    return;
}

pair<Tick, Tick>
DRAMCtrl::estimateLatency(DRAMPacket* dram_pkt, Tick inTime)
{
    // If a request reaches a bank at tick 'inTime', how much time
    // *after* that does it take to finish the request, depending
    // on bank status and page open policy. Note that this method
    // considers only the time taken for the actual read or write
    // to complete, NOT any additional time thereafter for tRAS or
    // tRP.
    Tick accLat = 0;
    Tick bankLat = 0;
    rowHitFlag = false;
    Tick potentialActTick;

    const Bank& bank = dram_pkt->bankRef;

    if (bank.openRow == dram_pkt->row) {
        // When we have a row-buffer hit,
        // we don't care about tRAS having expired or not,
        // but do care about bank being free for access
        rowHitFlag = true;

        // When a series of requests arrive to the same row,
        // DDR systems are capable of streaming data continuously
        // at maximum bandwidth (subject to tCCD). Here, we approximate
        // this condition, and assume that if whenever a bank is already
        // busy and a new request comes in, it can be completed with no
        // penalty beyond waiting for the existing read to complete.
        if (bank.freeAt > inTime) {
            accLat += bank.freeAt - inTime;
            bankLat += 0;
        } else {
            // CAS latency only
            accLat += tCL;
            bankLat += tCL;
        }
    } else {
        // Row-buffer miss, need to close existing row
        // once tRAS has expired, then open the new one,
        // then add cas latency.
        Tick freeTime = std::max(bank.tRASDoneAt, bank.freeAt);

        if (freeTime > inTime)
            accLat += freeTime - inTime;

        // If the there is no open row, then there is no precharge
        // delay, otherwise go with tRP
        Tick precharge_delay = bank.openRow == Bank::NO_ROW ? 0 : tRP;

        //The bank is free, and you may be able to activate
        potentialActTick = inTime + accLat + precharge_delay;
        if (potentialActTick < bank.actAllowedAt)
            accLat += bank.actAllowedAt - potentialActTick;

        accLat += precharge_delay + tRCD + tCL;
        bankLat += precharge_delay + tRCD + tCL;
    }

    DPRINTF(DRAM, "Returning < %lld, %lld > from estimateLatency()\n",
            bankLat, accLat);

    return make_pair(bankLat, accLat);
}

void
DRAMCtrl::recordActivate(Tick act_tick, uint8_t rank, uint8_t bank,
                         uint16_t row)
{
    assert(0 <= rank && rank < ranksPerChannel);
    assert(actTicks[rank].size() == activationLimit);

    DPRINTF(DRAM, "Activate at tick %d\n", act_tick);

    // update the open row
    assert(banks[rank][bank].openRow == Bank::NO_ROW);
    banks[rank][bank].openRow = row;

    // start counting anew, this covers both the case when we
    // auto-precharged, and when this access is forced to
    // precharge
    banks[rank][bank].bytesAccessed = 0;
    banks[rank][bank].rowAccesses = 0;

    ++numBanksActive;
    assert(numBanksActive <= banksPerRank * ranksPerChannel);

    DPRINTF(DRAM, "Activate bank at tick %lld, now got %d active\n",
            act_tick, numBanksActive);

    // start by enforcing tRRD
    for(int i = 0; i < banksPerRank; i++) {
        // next activate must not happen before tRRD
        banks[rank][i].actAllowedAt = act_tick + tRRD;
    }

    // tRC should be added to activation tick of the bank currently accessed,
    // where tRC = tRAS + tRP, this is just for a check as actAllowedAt for same
    // bank is already captured by bank.freeAt and bank.tRASDoneAt
    banks[rank][bank].actAllowedAt = act_tick + tRAS + tRP;

    // next, we deal with tXAW, if the activation limit is disabled
    // then we are done
    if (actTicks[rank].empty())
        return;

    // sanity check
    if (actTicks[rank].back() && (act_tick - actTicks[rank].back()) < tXAW) {
        // @todo For now, stick with a warning
        warn("Got %d activates in window %d (%d - %d) which is smaller "
             "than %d\n", activationLimit, act_tick - actTicks[rank].back(),
             act_tick, actTicks[rank].back(), tXAW);
    }

    // shift the times used for the book keeping, the last element
    // (highest index) is the oldest one and hence the lowest value
    actTicks[rank].pop_back();

    // record an new activation (in the future)
    actTicks[rank].push_front(act_tick);

    // cannot activate more than X times in time window tXAW, push the
    // next one (the X + 1'st activate) to be tXAW away from the
    // oldest in our window of X
    if (actTicks[rank].back() && (act_tick - actTicks[rank].back()) < tXAW) {
        DPRINTF(DRAM, "Enforcing tXAW with X = %d, next activate no earlier "
                "than %d\n", activationLimit, actTicks[rank].back() + tXAW);
            for(int j = 0; j < banksPerRank; j++)
                // next activate must not happen before end of window
                banks[rank][j].actAllowedAt = actTicks[rank].back() + tXAW;
    }

    // at the point when this activate takes place, make sure we
    // transition to the active power state
    if (!activateEvent.scheduled())
        schedule(activateEvent, act_tick);
    else if (activateEvent.when() > act_tick)
        // move it sooner in time
        reschedule(activateEvent, act_tick);
}

void
DRAMCtrl::processActivateEvent()
{
    // we should transition to the active state as soon as any bank is active
    if (pwrState != PWR_ACT)
        // note that at this point numBanksActive could be back at
        // zero again due to a precharge scheduled in the future
        schedulePowerEvent(PWR_ACT, curTick());
}

void
DRAMCtrl::prechargeBank(Bank& bank, Tick free_at)
{
    // make sure the bank has an open row
    assert(bank.openRow != Bank::NO_ROW);

    // sample the bytes per activate here since we are closing
    // the page
    bytesPerActivate.sample(bank.bytesAccessed);

    bank.openRow = Bank::NO_ROW;

    bank.freeAt = free_at;

    assert(numBanksActive != 0);
    --numBanksActive;

    DPRINTF(DRAM, "Precharged bank, done at tick %lld, now got %d active\n",
            bank.freeAt, numBanksActive);

    // if we look at the current number of active banks we might be
    // tempted to think the DRAM is now idle, however this can be
    // undone by an activate that is scheduled to happen before we
    // would have reached the idle state, so schedule an event and
    // rather check once we actually make it to the point in time when
    // the (last) precharge takes place
    if (!prechargeEvent.scheduled())
        schedule(prechargeEvent, free_at);
    else if (prechargeEvent.when() < free_at)
        reschedule(prechargeEvent, free_at);
}

void
DRAMCtrl::processPrechargeEvent()
{
    // if we reached zero, then special conditions apply as we track
    // if all banks are precharged for the power models
    if (numBanksActive == 0) {
        // we should transition to the idle state when the last bank
        // is precharged
        schedulePowerEvent(PWR_IDLE, curTick());
    }
}

void
DRAMCtrl::doDRAMAccess(DRAMPacket* dram_pkt)
{

    DPRINTF(DRAM, "Timing access to addr %lld, rank/bank/row %d %d %d\n",
            dram_pkt->addr, dram_pkt->rank, dram_pkt->bank, dram_pkt->row);

    // estimate the bank and access latency
    pair<Tick, Tick> lat = estimateLatency(dram_pkt, curTick());
    Tick bankLat = lat.first;
    Tick accessLat = lat.second;
    Tick actTick;

    // This request was woken up at this time based on a prior call
    // to estimateLatency(). However, between then and now, both the
    // accessLatency and/or busBusyUntil may have changed. We need
    // to correct for that.

    Tick addDelay = (curTick() + accessLat < busBusyUntil) ?
        busBusyUntil - (curTick() + accessLat) : 0;

    Bank& bank = dram_pkt->bankRef;

    // Update bank state
    if (rowHitFlag) {
        bank.freeAt = curTick() + addDelay + accessLat;
    } else {
        // If there is a page open, precharge it.
        if (bank.openRow != Bank::NO_ROW) {
            prechargeBank(bank, std::max(std::max(bank.freeAt,
                                                  bank.tRASDoneAt),
                                         curTick()) + tRP);
        }

        // Any precharge is already part of the latency
        // estimation, so update the bank free time
        bank.freeAt = curTick() + addDelay + accessLat;

        // any waiting for banks account for in freeAt
        actTick = bank.freeAt - tCL - tRCD;

        // If you activated a new row do to this access, the next access
        // will have to respect tRAS for this bank
        bank.tRASDoneAt = actTick + tRAS;

        recordActivate(actTick, dram_pkt->rank, dram_pkt->bank,
                       dram_pkt->row);
    }

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
        const deque<DRAMPacket*>& queue = dram_pkt->isRead ? readQueue :
            writeQueue;
        auto p = queue.begin();
        // make sure we are not considering the packet that we are
        // currently dealing with (which is the head of the queue)
        ++p;

        // keep on looking until we have found required condition or
        // reached the end
        while (!(got_more_hits &&
                 (got_bank_conflict || pageMgmt == Enums::close_adaptive)) &&
               p != queue.end()) {
            bool same_rank_bank = (dram_pkt->rank == (*p)->rank) &&
                (dram_pkt->bank == (*p)->bank);
            bool same_row = dram_pkt->row == (*p)->row;
            got_more_hits |= same_rank_bank && same_row;
            got_bank_conflict |= same_rank_bank && !same_row;
            ++p;
        }

        // auto pre-charge when either
        // 1) open_adaptive policy, we have not got any more hits, and
        //    have a bank conflict
        // 2) close_adaptive policy and we have not got any more hits
        auto_precharge = !got_more_hits &&
            (got_bank_conflict || pageMgmt == Enums::close_adaptive);
    }

    // if this access should use auto-precharge, then we are
    // closing the row
    if (auto_precharge) {
        prechargeBank(bank, std::max(bank.freeAt, bank.tRASDoneAt) + tRP);

        DPRINTF(DRAM, "Auto-precharged bank: %d\n", dram_pkt->bankId);
    }

    DPRINTF(DRAM, "doDRAMAccess::bank.freeAt is %lld\n", bank.freeAt);

    // Update request parameters
    dram_pkt->readyTime = curTick() + addDelay + accessLat + tBURST;

    DPRINTF(DRAM, "Req %lld: curtick is %lld accessLat is %d " \
                  "readytime is %lld busbusyuntil is %lld. " \
                  "Scheduling at readyTime\n", dram_pkt->addr,
                   curTick(), accessLat, dram_pkt->readyTime, busBusyUntil);

    // Make sure requests are not overlapping on the databus
    assert(dram_pkt->readyTime - busBusyUntil >= tBURST);

    // Update bus state
    busBusyUntil = dram_pkt->readyTime;

    DPRINTF(DRAM,"Access time is %lld\n",
            dram_pkt->readyTime - dram_pkt->entryTime);

    // Update the minimum timing between the requests, this is a
    // conservative estimate of when we have to schedule the next
    // request to not introduce any unecessary bubbles. In most cases
    // we will wake up sooner than we have to.
    nextReqTime = busBusyUntil - (tRP + tRCD + tCL);

    // Update the stats and schedule the next request
    if (dram_pkt->isRead) {
        ++readsThisTime;
        if (rowHitFlag)
            readRowHits++;
        bytesReadDRAM += burstSize;
        perBankRdBursts[dram_pkt->bankId]++;

        // Update latency stats
        totMemAccLat += dram_pkt->readyTime - dram_pkt->entryTime;
        totBankLat += bankLat;
        totBusLat += tBURST;
        totQLat += dram_pkt->readyTime - dram_pkt->entryTime - bankLat -
            tBURST;
    } else {
        ++writesThisTime;
        if (rowHitFlag)
            writeRowHits++;
        bytesWritten += burstSize;
        perBankWrBursts[dram_pkt->bankId]++;
    }
}

void
DRAMCtrl::moveToRespQ()
{
    // Remove from read queue
    DRAMPacket* dram_pkt = readQueue.front();
    readQueue.pop_front();

    // sanity check
    assert(dram_pkt->size <= burstSize);

    // Insert into response queue sorted by readyTime
    // It will be sent back to the requestor at its
    // readyTime
    if (respQueue.empty()) {
        respQueue.push_front(dram_pkt);
        assert(!respondEvent.scheduled());
        assert(dram_pkt->readyTime >= curTick());
        schedule(respondEvent, dram_pkt->readyTime);
    } else {
        bool done = false;
        auto i = respQueue.begin();
        while (!done && i != respQueue.end()) {
            if ((*i)->readyTime > dram_pkt->readyTime) {
                respQueue.insert(i, dram_pkt);
                done = true;
            }
            ++i;
        }

        if (!done)
            respQueue.push_back(dram_pkt);

        assert(respondEvent.scheduled());

        if (respQueue.front()->readyTime < respondEvent.when()) {
            assert(respQueue.front()->readyTime >= curTick());
            reschedule(respondEvent, respQueue.front()->readyTime);
        }
    }
}

void
DRAMCtrl::processNextReqEvent()
{
    if (busState == READ_TO_WRITE) {
        DPRINTF(DRAM, "Switching to writes after %d reads with %d reads "
                "waiting\n", readsThisTime, readQueue.size());

        // sample and reset the read-related stats as we are now
        // transitioning to writes, and all reads are done
        rdPerTurnAround.sample(readsThisTime);
        readsThisTime = 0;

        // now proceed to do the actual writes
        busState = WRITE;
    } else if (busState == WRITE_TO_READ) {
        DPRINTF(DRAM, "Switching to reads after %d writes with %d writes "
                "waiting\n", writesThisTime, writeQueue.size());

        wrPerTurnAround.sample(writesThisTime);
        writesThisTime = 0;

        busState = READ;
    }

    if (refreshState != REF_IDLE) {
        // if a refresh waiting for this event loop to finish, then hand
        // over now, and do not schedule a new nextReqEvent
        if (refreshState == REF_DRAIN) {
            DPRINTF(DRAM, "Refresh drain done, now precharging\n");

            refreshState = REF_PRE;

            // hand control back to the refresh event loop
            schedule(refreshEvent, curTick());
        }

        // let the refresh finish before issuing any further requests
        return;
    }

    // when we get here it is either a read or a write
    if (busState == READ) {

        // track if we should switch or not
        bool switch_to_writes = false;

        if (readQueue.empty()) {
            // In the case there is no read request to go next,
            // trigger writes if we have passed the low threshold (or
            // if we are draining)
            if (!writeQueue.empty() &&
                (drainManager || writeQueue.size() > writeLowThreshold)) {

                switch_to_writes = true;
            } else {
                // check if we are drained
                if (respQueue.empty () && drainManager) {
                    drainManager->signalDrainDone();
                    drainManager = NULL;
                }

                // nothing to do, not even any point in scheduling an
                // event for the next request
                return;
            }
        } else {
            // Figure out which read request goes next, and move it to the
            // front of the read queue
            chooseNext(readQueue);

            doDRAMAccess(readQueue.front());

            // At this point we're done dealing with the request
            // It will be moved to a separate response queue with a
            // correct readyTime, and eventually be sent back at that
            // time
            moveToRespQ();

            // we have so many writes that we have to transition
            if (writeQueue.size() > writeHighThreshold) {
                switch_to_writes = true;
            }
        }

        // switching to writes, either because the read queue is empty
        // and the writes have passed the low threshold (or we are
        // draining), or because the writes hit the hight threshold
        if (switch_to_writes) {
            // transition to writing
            busState = READ_TO_WRITE;

            // add a bubble to the data bus, as defined by the
            // tRTW parameter
            busBusyUntil += tRTW;

            // update the minimum timing between the requests,
            // this shifts us back in time far enough to do any
            // bank preparation
            nextReqTime = busBusyUntil - (tRP + tRCD + tCL);
        }
    } else {
        chooseNext(writeQueue);
        DRAMPacket* dram_pkt = writeQueue.front();
        // sanity check
        assert(dram_pkt->size <= burstSize);
        doDRAMAccess(dram_pkt);

        writeQueue.pop_front();
        delete dram_pkt;

        // If we emptied the write queue, or got sufficiently below the
        // threshold (using the minWritesPerSwitch as the hysteresis) and
        // are not draining, or we have reads waiting and have done enough
        // writes, then switch to reads.
        if (writeQueue.empty() ||
            (writeQueue.size() + minWritesPerSwitch < writeLowThreshold &&
             !drainManager) ||
            (!readQueue.empty() && writesThisTime >= minWritesPerSwitch)) {
            // turn the bus back around for reads again
            busState = WRITE_TO_READ;

            // note that the we switch back to reads also in the idle
            // case, which eventually will check for any draining and
            // also pause any further scheduling if there is really
            // nothing to do

            // here we get a bit creative and shift the bus busy time not
            // just the tWTR, but also a CAS latency to capture the fact
            // that we are allowed to prepare a new bank, but not issue a
            // read command until after tWTR, in essence we capture a
            // bubble on the data bus that is tWTR + tCL
            busBusyUntil += tWTR + tCL;

            // update the minimum timing between the requests, this shifts
            // us back in time far enough to do any bank preparation
            nextReqTime = busBusyUntil - (tRP + tRCD + tCL);
        }
    }

    schedule(nextReqEvent, std::max(nextReqTime, curTick()));

    // If there is space available and we have writes waiting then let
    // them retry. This is done here to ensure that the retry does not
    // cause a nextReqEvent to be scheduled before we do so as part of
    // the next request processing
    if (retryWrReq && writeQueue.size() < writeBufferSize) {
        retryWrReq = false;
        port.sendRetry();
    }
}

uint64_t
DRAMCtrl::minBankFreeAt(const deque<DRAMPacket*>& queue) const
{
    uint64_t bank_mask = 0;
    Tick freeAt = MaxTick;

    // detemrine if we have queued transactions targetting the
    // bank in question
    vector<bool> got_waiting(ranksPerChannel * banksPerRank, false);
    for (auto p = queue.begin(); p != queue.end(); ++p) {
        got_waiting[(*p)->bankId] = true;
    }

    for (int i = 0; i < ranksPerChannel; i++) {
        for (int j = 0; j < banksPerRank; j++) {
            // if we have waiting requests for the bank, and it is
            // amongst the first available, update the mask
            if (got_waiting[i * banksPerRank + j] &&
                banks[i][j].freeAt <= freeAt) {
                // reset bank mask if new minimum is found
                if (banks[i][j].freeAt < freeAt)
                    bank_mask = 0;
                // set the bit corresponding to the available bank
                uint8_t bit_index = i * ranksPerChannel + j;
                replaceBits(bank_mask, bit_index, bit_index, 1);
                freeAt = banks[i][j].freeAt;
            }
        }
    }
    return bank_mask;
}

void
DRAMCtrl::processRefreshEvent()
{
    // when first preparing the refresh, remember when it was due
    if (refreshState == REF_IDLE) {
        // remember when the refresh is due
        refreshDueAt = curTick();

        // proceed to drain
        refreshState = REF_DRAIN;

        DPRINTF(DRAM, "Refresh due\n");
    }

    // let any scheduled read or write go ahead, after which it will
    // hand control back to this event loop
    if (refreshState == REF_DRAIN) {
        if (nextReqEvent.scheduled()) {
            // hand control over to the request loop until it is
            // evaluated next
            DPRINTF(DRAM, "Refresh awaiting draining\n");

            return;
        } else {
            refreshState = REF_PRE;
        }
    }

    // at this point, ensure that all banks are precharged
    if (refreshState == REF_PRE) {
        // precharge any active bank if we are not already in the idle
        // state
        if (pwrState != PWR_IDLE) {
            DPRINTF(DRAM, "Precharging all\n");
            for (int i = 0; i < ranksPerChannel; i++) {
                for (int j = 0; j < banksPerRank; j++) {
                    if (banks[i][j].openRow != Bank::NO_ROW) {
                        // respect both causality and any existing bank
                        // constraints
                        Tick free_at =
                            std::max(std::max(banks[i][j].freeAt,
                                              banks[i][j].tRASDoneAt),
                                     curTick()) + tRP;

                        prechargeBank(banks[i][j], free_at);
                    }
                }
            }
        } else {
            DPRINTF(DRAM, "All banks already precharged, starting refresh\n");

            // go ahead and kick the power state machine into gear if
            // we are already idle
            schedulePowerEvent(PWR_REF, curTick());
        }

        refreshState = REF_RUN;
        assert(numBanksActive == 0);

        // wait for all banks to be precharged, at which point the
        // power state machine will transition to the idle state, and
        // automatically move to a refresh, at that point it will also
        // call this method to get the refresh event loop going again
        return;
    }

    // last but not least we perform the actual refresh
    if (refreshState == REF_RUN) {
        // should never get here with any banks active
        assert(numBanksActive == 0);
        assert(pwrState == PWR_REF);

        Tick banksFree = curTick() + tRFC;

        for (int i = 0; i < ranksPerChannel; i++) {
            for (int j = 0; j < banksPerRank; j++) {
                banks[i][j].freeAt = banksFree;
            }
        }

        // make sure we did not wait so long that we cannot make up
        // for it
        if (refreshDueAt + tREFI < banksFree) {
            fatal("Refresh was delayed so long we cannot catch up\n");
        }

        // compensate for the delay in actually performing the refresh
        // when scheduling the next one
        schedule(refreshEvent, refreshDueAt + tREFI - tRP);

        assert(!powerEvent.scheduled());

        // move to the idle power state once the refresh is done, this
        // will also move the refresh state machine to the refresh
        // idle state
        schedulePowerEvent(PWR_IDLE, banksFree);

        DPRINTF(DRAMState, "Refresh done at %llu and next refresh at %llu\n",
                banksFree, refreshDueAt + tREFI);
    }
}

void
DRAMCtrl::schedulePowerEvent(PowerState pwr_state, Tick tick)
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
DRAMCtrl::processPowerEvent()
{
    // remember where we were, and for how long
    Tick duration = curTick() - pwrStateTick;
    PowerState prev_state = pwrState;

    // update the accounting
    pwrStateTime[prev_state] += duration;

    pwrState = pwrStateTrans;
    pwrStateTick = curTick();

    if (pwrState == PWR_IDLE) {
        DPRINTF(DRAMState, "All banks precharged\n");

        // if we were refreshing, make sure we start scheduling requests again
        if (prev_state == PWR_REF) {
            DPRINTF(DRAMState, "Was refreshing for %llu ticks\n", duration);
            assert(pwrState == PWR_IDLE);

            // kick things into action again
            refreshState = REF_IDLE;
            assert(!nextReqEvent.scheduled());
            schedule(nextReqEvent, curTick());
        } else {
            assert(prev_state == PWR_ACT);

            // if we have a pending refresh, and are now moving to
            // the idle state, direclty transition to a refresh
            if (refreshState == REF_RUN) {
                // there should be nothing waiting at this point
                assert(!powerEvent.scheduled());

                // update the state in zero time and proceed below
                pwrState = PWR_REF;
            }
        }
    }

    // we transition to the refresh state, let the refresh state
    // machine know of this state update and let it deal with the
    // scheduling of the next power state transition as well as the
    // following refresh
    if (pwrState == PWR_REF) {
        DPRINTF(DRAMState, "Refreshing\n");
        // kick the refresh event loop into action again, and that
        // in turn will schedule a transition to the idle power
        // state once the refresh is done
        assert(refreshState == REF_RUN);
        processRefreshEvent();
    }
}

void
DRAMCtrl::regStats()
{
    using namespace Stats;

    AbstractMemory::regStats();

    readReqs
        .name(name() + ".readReqs")
        .desc("Number of read requests accepted");

    writeReqs
        .name(name() + ".writeReqs")
        .desc("Number of write requests accepted");

    readBursts
        .name(name() + ".readBursts")
        .desc("Number of DRAM read bursts, "
              "including those serviced by the write queue");

    writeBursts
        .name(name() + ".writeBursts")
        .desc("Number of DRAM write bursts, "
              "including those merged in the write queue");

    servicedByWrQ
        .name(name() + ".servicedByWrQ")
        .desc("Number of DRAM read bursts serviced by the write queue");

    mergedWrBursts
        .name(name() + ".mergedWrBursts")
        .desc("Number of DRAM write bursts merged with an existing one");

    neitherReadNorWrite
        .name(name() + ".neitherReadNorWriteReqs")
        .desc("Number of requests that are neither read nor write");

    perBankRdBursts
        .init(banksPerRank * ranksPerChannel)
        .name(name() + ".perBankRdBursts")
        .desc("Per bank write bursts");

    perBankWrBursts
        .init(banksPerRank * ranksPerChannel)
        .name(name() + ".perBankWrBursts")
        .desc("Per bank write bursts");

    avgRdQLen
        .name(name() + ".avgRdQLen")
        .desc("Average read queue length when enqueuing")
        .precision(2);

    avgWrQLen
        .name(name() + ".avgWrQLen")
        .desc("Average write queue length when enqueuing")
        .precision(2);

    totQLat
        .name(name() + ".totQLat")
        .desc("Total ticks spent queuing");

    totBankLat
        .name(name() + ".totBankLat")
        .desc("Total ticks spent accessing banks");

    totBusLat
        .name(name() + ".totBusLat")
        .desc("Total ticks spent in databus transfers");

    totMemAccLat
        .name(name() + ".totMemAccLat")
        .desc("Total ticks spent from burst creation until serviced "
              "by the DRAM");

    avgQLat
        .name(name() + ".avgQLat")
        .desc("Average queueing delay per DRAM burst")
        .precision(2);

    avgQLat = totQLat / (readBursts - servicedByWrQ);

    avgBankLat
        .name(name() + ".avgBankLat")
        .desc("Average bank access latency per DRAM burst")
        .precision(2);

    avgBankLat = totBankLat / (readBursts - servicedByWrQ);

    avgBusLat
        .name(name() + ".avgBusLat")
        .desc("Average bus latency per DRAM burst")
        .precision(2);

    avgBusLat = totBusLat / (readBursts - servicedByWrQ);

    avgMemAccLat
        .name(name() + ".avgMemAccLat")
        .desc("Average memory access latency per DRAM burst")
        .precision(2);

    avgMemAccLat = totMemAccLat / (readBursts - servicedByWrQ);

    numRdRetry
        .name(name() + ".numRdRetry")
        .desc("Number of times read queue was full causing retry");

    numWrRetry
        .name(name() + ".numWrRetry")
        .desc("Number of times write queue was full causing retry");

    readRowHits
        .name(name() + ".readRowHits")
        .desc("Number of row buffer hits during reads");

    writeRowHits
        .name(name() + ".writeRowHits")
        .desc("Number of row buffer hits during writes");

    readRowHitRate
        .name(name() + ".readRowHitRate")
        .desc("Row buffer hit rate for reads")
        .precision(2);

    readRowHitRate = (readRowHits / (readBursts - servicedByWrQ)) * 100;

    writeRowHitRate
        .name(name() + ".writeRowHitRate")
        .desc("Row buffer hit rate for writes")
        .precision(2);

    writeRowHitRate = (writeRowHits / (writeBursts - mergedWrBursts)) * 100;

    readPktSize
        .init(ceilLog2(burstSize) + 1)
        .name(name() + ".readPktSize")
        .desc("Read request sizes (log2)");

     writePktSize
        .init(ceilLog2(burstSize) + 1)
        .name(name() + ".writePktSize")
        .desc("Write request sizes (log2)");

     rdQLenPdf
        .init(readBufferSize)
        .name(name() + ".rdQLenPdf")
        .desc("What read queue length does an incoming req see");

     wrQLenPdf
        .init(writeBufferSize)
        .name(name() + ".wrQLenPdf")
        .desc("What write queue length does an incoming req see");

     bytesPerActivate
         .init(maxAccessesPerRow)
         .name(name() + ".bytesPerActivate")
         .desc("Bytes accessed per row activation")
         .flags(nozero);

     rdPerTurnAround
         .init(readBufferSize)
         .name(name() + ".rdPerTurnAround")
         .desc("Reads before turning the bus around for writes")
         .flags(nozero);

     wrPerTurnAround
         .init(writeBufferSize)
         .name(name() + ".wrPerTurnAround")
         .desc("Writes before turning the bus around for reads")
         .flags(nozero);

    bytesReadDRAM
        .name(name() + ".bytesReadDRAM")
        .desc("Total number of bytes read from DRAM");

    bytesReadWrQ
        .name(name() + ".bytesReadWrQ")
        .desc("Total number of bytes read from write queue");

    bytesWritten
        .name(name() + ".bytesWritten")
        .desc("Total number of bytes written to DRAM");

    bytesReadSys
        .name(name() + ".bytesReadSys")
        .desc("Total read bytes from the system interface side");

    bytesWrittenSys
        .name(name() + ".bytesWrittenSys")
        .desc("Total written bytes from the system interface side");

    avgRdBW
        .name(name() + ".avgRdBW")
        .desc("Average DRAM read bandwidth in MiByte/s")
        .precision(2);

    avgRdBW = (bytesReadDRAM / 1000000) / simSeconds;

    avgWrBW
        .name(name() + ".avgWrBW")
        .desc("Average achieved write bandwidth in MiByte/s")
        .precision(2);

    avgWrBW = (bytesWritten / 1000000) / simSeconds;

    avgRdBWSys
        .name(name() + ".avgRdBWSys")
        .desc("Average system read bandwidth in MiByte/s")
        .precision(2);

    avgRdBWSys = (bytesReadSys / 1000000) / simSeconds;

    avgWrBWSys
        .name(name() + ".avgWrBWSys")
        .desc("Average system write bandwidth in MiByte/s")
        .precision(2);

    avgWrBWSys = (bytesWrittenSys / 1000000) / simSeconds;

    peakBW
        .name(name() + ".peakBW")
        .desc("Theoretical peak bandwidth in MiByte/s")
        .precision(2);

    peakBW = (SimClock::Frequency / tBURST) * burstSize / 1000000;

    busUtil
        .name(name() + ".busUtil")
        .desc("Data bus utilization in percentage")
        .precision(2);

    busUtil = (avgRdBW + avgWrBW) / peakBW * 100;

    totGap
        .name(name() + ".totGap")
        .desc("Total gap between requests");

    avgGap
        .name(name() + ".avgGap")
        .desc("Average gap between requests")
        .precision(2);

    avgGap = totGap / (readReqs + writeReqs);

    // Stats for DRAM Power calculation based on Micron datasheet
    busUtilRead
        .name(name() + ".busUtilRead")
        .desc("Data bus utilization in percentage for reads")
        .precision(2);

    busUtilRead = avgRdBW / peakBW * 100;

    busUtilWrite
        .name(name() + ".busUtilWrite")
        .desc("Data bus utilization in percentage for writes")
        .precision(2);

    busUtilWrite = avgWrBW / peakBW * 100;

    pageHitRate
        .name(name() + ".pageHitRate")
        .desc("Row buffer hit rate, read and write combined")
        .precision(2);

    pageHitRate = (writeRowHits + readRowHits) /
        (writeBursts - mergedWrBursts + readBursts - servicedByWrQ) * 100;

    pwrStateTime
        .init(5)
        .name(name() + ".memoryStateTime")
        .desc("Time in different power states");
    pwrStateTime.subname(0, "IDLE");
    pwrStateTime.subname(1, "REF");
    pwrStateTime.subname(2, "PRE_PDN");
    pwrStateTime.subname(3, "ACT");
    pwrStateTime.subname(4, "ACT_PDN");
}

void
DRAMCtrl::recvFunctional(PacketPtr pkt)
{
    // rely on the abstract memory
    functionalAccess(pkt);
}

BaseSlavePort&
DRAMCtrl::getSlavePort(const string &if_name, PortID idx)
{
    if (if_name != "port") {
        return MemObject::getSlavePort(if_name, idx);
    } else {
        return port;
    }
}

unsigned int
DRAMCtrl::drain(DrainManager *dm)
{
    unsigned int count = port.drain(dm);

    // if there is anything in any of our internal queues, keep track
    // of that as well
    if (!(writeQueue.empty() && readQueue.empty() &&
          respQueue.empty())) {
        DPRINTF(Drain, "DRAM controller not drained, write: %d, read: %d,"
                " resp: %d\n", writeQueue.size(), readQueue.size(),
                respQueue.size());
        ++count;
        drainManager = dm;

        // the only part that is not drained automatically over time
        // is the write queue, thus kick things into action if needed
        if (!writeQueue.empty() && !nextReqEvent.scheduled()) {
            schedule(nextReqEvent, curTick());
        }
    }

    if (count)
        setDrainState(Drainable::Draining);
    else
        setDrainState(Drainable::Drained);
    return count;
}

DRAMCtrl::MemoryPort::MemoryPort(const std::string& name, DRAMCtrl& _memory)
    : QueuedSlavePort(name, &_memory, queue), queue(_memory, *this),
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

    if (!queue.checkFunctional(pkt)) {
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
