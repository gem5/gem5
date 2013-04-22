/*
 * Copyright (c) 2010-2012 ARM Limited
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
 * Authors: Andreas Hansson
 *          Ani Udipi
 */

#include "base/trace.hh"
#include "debug/Drain.hh"
#include "debug/DRAM.hh"
#include "debug/DRAMWR.hh"
#include "mem/simple_dram.hh"

using namespace std;

SimpleDRAM::SimpleDRAM(const SimpleDRAMParams* p) :
    AbstractMemory(p),
    port(name() + ".port", *this),
    retryRdReq(false), retryWrReq(false),
    rowHitFlag(false), stopReads(false), actTicks(p->activation_limit, 0),
    writeEvent(this), respondEvent(this),
    refreshEvent(this), nextReqEvent(this), drainManager(NULL),
    bytesPerCacheLine(0),
    linesPerRowBuffer(p->lines_per_rowbuffer),
    ranksPerChannel(p->ranks_per_channel),
    banksPerRank(p->banks_per_rank), channels(p->channels), rowsPerBank(0),
    readBufferSize(p->read_buffer_size),
    writeBufferSize(p->write_buffer_size),
    writeThresholdPerc(p->write_thresh_perc),
    tWTR(p->tWTR), tBURST(p->tBURST),
    tRCD(p->tRCD), tCL(p->tCL), tRP(p->tRP),
    tRFC(p->tRFC), tREFI(p->tREFI),
    tXAW(p->tXAW), activationLimit(p->activation_limit),
    memSchedPolicy(p->mem_sched_policy), addrMapping(p->addr_mapping),
    pageMgmt(p->page_policy),
    busBusyUntil(0), writeStartTime(0),
    prevArrival(0), numReqs(0)
{
    // create the bank states based on the dimensions of the ranks and
    // banks
    banks.resize(ranksPerChannel);
    for (size_t c = 0; c < ranksPerChannel; ++c) {
        banks[c].resize(banksPerRank);
    }

    // round the write threshold percent to a whole number of entries
    // in the buffer
    writeThreshold = writeBufferSize * writeThresholdPerc / 100.0;
}

void
SimpleDRAM::init()
{
    if (!port.isConnected()) {
        fatal("SimpleDRAM %s is unconnected!\n", name());
    } else {
        port.sendRangeChange();
    }

    // get the burst size from the connected port as it is currently
    // assumed to be equal to the cache line size
    bytesPerCacheLine = port.peerBlockSize();

    // we could deal with plenty options here, but for now do a quick
    // sanity check
    if (bytesPerCacheLine != 64 && bytesPerCacheLine != 32)
        panic("Unexpected burst size %d", bytesPerCacheLine);

    // determine the rows per bank by looking at the total capacity
    uint64_t capacity = ULL(1) << ceilLog2(AbstractMemory::size());

    DPRINTF(DRAM, "Memory capacity %lld (%lld) bytes\n", capacity,
            AbstractMemory::size());
    rowsPerBank = capacity / (bytesPerCacheLine * linesPerRowBuffer *
                              banksPerRank * ranksPerChannel);

    if (range.interleaved()) {
        if (channels != range.stripes())
            panic("%s has %d interleaved address stripes but %d channel(s)\n",
                  name(), range.stripes(), channels);

        if (addrMapping == Enums::RaBaChCo) {
            if (bytesPerCacheLine * linesPerRowBuffer !=
                range.granularity()) {
                panic("Interleaving of %s doesn't match RaBaChCo address map\n",
                      name());
            }
        } else if (addrMapping == Enums::RaBaCoCh) {
            if (bytesPerCacheLine != range.granularity()) {
                panic("Interleaving of %s doesn't match RaBaCoCh address map\n",
                      name());
            }
        } else if (addrMapping == Enums::CoRaBaCh) {
            if (bytesPerCacheLine != range.granularity())
                panic("Interleaving of %s doesn't match CoRaBaCh address map\n",
                      name());
        }
    }
}

void
SimpleDRAM::startup()
{
    // print the configuration of the controller
    printParams();

    // kick off the refresh
    schedule(refreshEvent, curTick() + tREFI);
}

Tick
SimpleDRAM::recvAtomic(PacketPtr pkt)
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
SimpleDRAM::readQueueFull() const
{
    DPRINTF(DRAM, "Read queue limit %d current size %d\n",
            readBufferSize, readQueue.size() + respQueue.size());

    return (readQueue.size() + respQueue.size()) == readBufferSize;
}

bool
SimpleDRAM::writeQueueFull() const
{
    DPRINTF(DRAM, "Write queue limit %d current size %d\n",
            writeBufferSize, writeQueue.size());
    return writeQueue.size() == writeBufferSize;
}

SimpleDRAM::DRAMPacket*
SimpleDRAM::decodeAddr(PacketPtr pkt)
{
    // decode the address based on the address mapping scheme, with
    // Ra, Co, Ba and Ch denoting rank, column, bank and channel,
    // respectively
    uint8_t rank;
    uint16_t bank;
    uint16_t row;

    Addr addr = pkt->getAddr();

    // truncate the address to the access granularity
    addr = addr / bytesPerCacheLine;

    // we have removed the lowest order address bits that denote the
    // position within the cache line
    if (addrMapping == Enums::RaBaChCo) {
        // the lowest order bits denote the column to ensure that
        // sequential cache lines occupy the same row
        addr = addr / linesPerRowBuffer;

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
    } else if (addrMapping == Enums::RaBaCoCh) {
        // take out the channel part of the address
        addr = addr / channels;

        // next, the column
        addr = addr / linesPerRowBuffer;

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
    } else if (addrMapping == Enums::CoRaBaCh) {
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
        addr = addr / linesPerRowBuffer;

        // lastly, get the row bits
        row = addr % rowsPerBank;
        addr = addr / rowsPerBank;
    } else
        panic("Unknown address mapping policy chosen!");

    assert(rank < ranksPerChannel);
    assert(bank < banksPerRank);
    assert(row < rowsPerBank);

    DPRINTF(DRAM, "Address: %lld Rank %d Bank %d Row %d\n",
            pkt->getAddr(), rank, bank, row);

    // create the corresponding DRAM packet with the entry time and
    // ready time set to the current tick, the latter will be updated
    // later
    return new DRAMPacket(pkt, rank, bank, row, pkt->getAddr(),
                          banks[rank][bank]);
}

void
SimpleDRAM::addToReadQueue(PacketPtr pkt)
{
    // only add to the read queue here. whenever the request is
    // eventually done, set the readyTime, and call schedule()
    assert(!pkt->isWrite());

    // First check write buffer to see if the data is already at
    // the controller
    list<DRAMPacket*>::const_iterator i;
    Addr addr = pkt->getAddr();

    // @todo: add size check
    for (i = writeQueue.begin(); i != writeQueue.end(); ++i) {
        if ((*i)->addr == addr){
            servicedByWrQ++;
            DPRINTF(DRAM, "Read to %lld serviced by write queue\n", addr);
            bytesRead += bytesPerCacheLine;
            bytesConsumedRd += pkt->getSize();
            accessAndRespond(pkt);
            return;
        }
    }

    DRAMPacket* dram_pkt = decodeAddr(pkt);

    assert(readQueue.size() + respQueue.size() < readBufferSize);
    rdQLenPdf[readQueue.size() + respQueue.size()]++;

    DPRINTF(DRAM, "Adding to read queue\n");

    readQueue.push_back(dram_pkt);

    // Update stats
    uint32_t bank_id = banksPerRank * dram_pkt->rank + dram_pkt->bank;
    assert(bank_id < ranksPerChannel * banksPerRank);
    perBankRdReqs[bank_id]++;

    avgRdQLen = readQueue.size() + respQueue.size();

    // If we are not already scheduled to get the read request out of
    // the queue, do so now
    if (!nextReqEvent.scheduled() && !stopReads) {
        DPRINTF(DRAM, "Request scheduled immediately\n");
        schedule(nextReqEvent, curTick());
    }
}

void
SimpleDRAM::processWriteEvent()
{
    assert(!writeQueue.empty());
    uint32_t numWritesThisTime = 0;

    DPRINTF(DRAMWR, "Beginning DRAM Writes\n");
    Tick temp1 M5_VAR_USED = std::max(curTick(), busBusyUntil);
    Tick temp2 M5_VAR_USED = std::max(curTick(), maxBankFreeAt());

    // @todo: are there any dangers with the untimed while loop?
    while (!writeQueue.empty()) {
        if (numWritesThisTime > writeThreshold) {
            DPRINTF(DRAMWR, "Hit write threshold %d\n", writeThreshold);
            break;
        }

        chooseNextWrite();
        DRAMPacket* dram_pkt = writeQueue.front();
        // What's the earliest the request can be put on the bus
        Tick schedTime = std::max(curTick(), busBusyUntil);

        DPRINTF(DRAMWR, "Asking for latency estimate at %lld\n",
                schedTime + tBURST);

        pair<Tick, Tick> lat = estimateLatency(dram_pkt, schedTime + tBURST);
        Tick accessLat = lat.second;

        // look at the rowHitFlag set by estimateLatency
        if (rowHitFlag)
            writeRowHits++;

        Bank& bank = dram_pkt->bank_ref;

        if (pageMgmt == Enums::open) {
            bank.openRow = dram_pkt->row;
            bank.freeAt = schedTime + tBURST + std::max(accessLat, tCL);
            busBusyUntil = bank.freeAt - tCL;

            if (!rowHitFlag) {
                bank.tRASDoneAt = bank.freeAt + tRP;
                recordActivate(bank.freeAt - tCL - tRCD);
                busBusyUntil = bank.freeAt - tCL - tRCD;
            }
        } else if (pageMgmt == Enums::close) {
            bank.freeAt = schedTime + tBURST + accessLat + tRP + tRP;
            // Work backwards from bank.freeAt to determine activate time
            recordActivate(bank.freeAt - tRP - tRP - tCL - tRCD);
            busBusyUntil = bank.freeAt - tRP - tRP - tCL - tRCD;
            DPRINTF(DRAMWR, "processWriteEvent::bank.freeAt for "
                    "banks_id %d is %lld\n",
                    dram_pkt->rank * banksPerRank + dram_pkt->bank,
                    bank.freeAt);
        } else
            panic("Unknown page management policy chosen\n");

        DPRINTF(DRAMWR, "Done writing to address %lld\n", dram_pkt->addr);

        DPRINTF(DRAMWR, "schedtime is %lld, tBURST is %lld, "
                "busbusyuntil is %lld\n",
                schedTime, tBURST, busBusyUntil);

        writeQueue.pop_front();
        delete dram_pkt;

        numWritesThisTime++;
    }

    DPRINTF(DRAMWR, "Completed %d writes, bus busy for %lld ticks,"\
            "banks busy for %lld ticks\n", numWritesThisTime,
            busBusyUntil - temp1, maxBankFreeAt() - temp2);

    // Update stats
    avgWrQLen = writeQueue.size();

    // turn the bus back around for reads again
    busBusyUntil += tWTR;
    stopReads = false;

    if (retryWrReq) {
        retryWrReq = false;
        port.sendRetry();
    }

    // if there is nothing left in any queue, signal a drain
    if (writeQueue.empty() && readQueue.empty() &&
        respQueue.empty () && drainManager) {
        drainManager->signalDrainDone();
        drainManager = NULL;
    }

    // Once you're done emptying the write queue, check if there's
    // anything in the read queue, and call schedule if required. The
    // retry above could already have caused it to be scheduled, so
    // first check
    if (!nextReqEvent.scheduled())
        schedule(nextReqEvent, busBusyUntil);
}

void
SimpleDRAM::triggerWrites()
{
    DPRINTF(DRAM, "Writes triggered at %lld\n", curTick());
    // Flag variable to stop any more read scheduling
    stopReads = true;

    writeStartTime = std::max(busBusyUntil, curTick()) + tWTR;

    DPRINTF(DRAM, "Writes scheduled at %lld\n", writeStartTime);

    assert(writeStartTime >= curTick());
    assert(!writeEvent.scheduled());
    schedule(writeEvent, writeStartTime);
}

void
SimpleDRAM::addToWriteQueue(PacketPtr pkt)
{
    // only add to the write queue here. whenever the request is
    // eventually done, set the readyTime, and call schedule()
    assert(pkt->isWrite());

    DRAMPacket* dram_pkt = decodeAddr(pkt);

    assert(writeQueue.size() < writeBufferSize);
    wrQLenPdf[writeQueue.size()]++;

    DPRINTF(DRAM, "Adding to write queue\n");

    writeQueue.push_back(dram_pkt);

    // Update stats
    uint32_t bank_id = banksPerRank * dram_pkt->rank + dram_pkt->bank;
    assert(bank_id < ranksPerChannel * banksPerRank);
    perBankWrReqs[bank_id]++;

    avgWrQLen = writeQueue.size();

    // we do not wait for the writes to be send to the actual memory,
    // but instead take responsibility for the consistency here and
    // snoop the write queue for any upcoming reads

    bytesConsumedWr += pkt->getSize();
    bytesWritten += bytesPerCacheLine;
    accessAndRespond(pkt);

    // If your write buffer is starting to fill up, drain it!
    if (writeQueue.size() > writeThreshold && !stopReads){
        triggerWrites();
    }
}

void
SimpleDRAM::printParams() const
{
    // Sanity check print of important parameters
    DPRINTF(DRAM,
            "Memory controller %s physical organization\n"      \
            "Bytes per cacheline  %d\n"                         \
            "Lines per row buffer %d\n"                         \
            "Rows  per bank       %d\n"                         \
            "Banks per rank       %d\n"                         \
            "Ranks per channel    %d\n"                         \
            "Total mem capacity   %u\n",
            name(), bytesPerCacheLine, linesPerRowBuffer, rowsPerBank,
            banksPerRank, ranksPerChannel, bytesPerCacheLine *
            linesPerRowBuffer * rowsPerBank * banksPerRank * ranksPerChannel);

    string scheduler =  memSchedPolicy == Enums::fcfs ? "FCFS" : "FR-FCFS";
    string address_mapping = addrMapping == Enums::RaBaChCo ? "RaBaChCo" :
        (addrMapping == Enums::RaBaCoCh ? "RaBaCoCh" : "CoRaBaCh");
    string page_policy = pageMgmt == Enums::open ? "OPEN" : "CLOSE";

    DPRINTF(DRAM,
            "Memory controller %s characteristics\n"    \
            "Read buffer size     %d\n"                 \
            "Write buffer size    %d\n"                 \
            "Write buffer thresh  %d\n"                 \
            "Scheduler            %s\n"                 \
            "Address mapping      %s\n"                 \
            "Page policy          %s\n",
            name(), readBufferSize, writeBufferSize, writeThreshold,
            scheduler, address_mapping, page_policy);

    DPRINTF(DRAM, "Memory controller %s timing specs\n" \
            "tRCD      %d ticks\n"                        \
            "tCL       %d ticks\n"                        \
            "tRP       %d ticks\n"                        \
            "tBURST    %d ticks\n"                        \
            "tRFC      %d ticks\n"                        \
            "tREFI     %d ticks\n"                        \
            "tWTR      %d ticks\n"                        \
            "tXAW (%d) %d ticks\n",
            name(), tRCD, tCL, tRP, tBURST, tRFC, tREFI, tWTR,
            activationLimit, tXAW);
}

void
SimpleDRAM::printQs() const {

    list<DRAMPacket*>::const_iterator i;

    DPRINTF(DRAM, "===READ QUEUE===\n\n");
    for (i = readQueue.begin() ;  i != readQueue.end() ; ++i) {
        DPRINTF(DRAM, "Read %lu\n", (*i)->addr);
    }
    DPRINTF(DRAM, "\n===RESP QUEUE===\n\n");
    for (i = respQueue.begin() ;  i != respQueue.end() ; ++i) {
        DPRINTF(DRAM, "Response %lu\n", (*i)->addr);
    }
    DPRINTF(DRAM, "\n===WRITE QUEUE===\n\n");
    for (i = writeQueue.begin() ;  i != writeQueue.end() ; ++i) {
        DPRINTF(DRAM, "Write %lu\n", (*i)->addr);
    }
}

bool
SimpleDRAM::recvTimingReq(PacketPtr pkt)
{
    /// @todo temporary hack to deal with memory corruption issues until
    /// 4-phase transactions are complete
    for (int x = 0; x < pendingDelete.size(); x++)
        delete pendingDelete[x];
    pendingDelete.clear();

    // This is where we enter from the outside world
    DPRINTF(DRAM, "recvTimingReq: request %s addr %lld size %d\n",
            pkt->cmdString(),pkt->getAddr(), pkt->getSize());

    // simply drop inhibited packets for now
    if (pkt->memInhibitAsserted()) {
        DPRINTF(DRAM,"Inhibited packet -- Dropping it now\n");
        pendingDelete.push_back(pkt);
        return true;
    }

   if (pkt->getSize() == bytesPerCacheLine)
       cpuReqs++;

   // Every million accesses, print the state of the queues
   if (numReqs % 1000000 == 0)
       printQs();

    // Calc avg gap between requests
    if (prevArrival != 0) {
        totGap += curTick() - prevArrival;
    }
    prevArrival = curTick();

    unsigned size = pkt->getSize();
    if (size > bytesPerCacheLine)
        panic("Request size %d is greater than burst size %d",
              size, bytesPerCacheLine);

    // check local buffers and do not accept if full
    if (pkt->isRead()) {
        assert(size != 0);
        if (readQueueFull()) {
            DPRINTF(DRAM, "Read queue full, not accepting\n");
            // remember that we have to retry this port
            retryRdReq = true;
            numRdRetry++;
            return false;
        } else {
            readPktSize[ceilLog2(size)]++;
            addToReadQueue(pkt);
            readReqs++;
            numReqs++;
        }
    } else if (pkt->isWrite()) {
        assert(size != 0);
        if (writeQueueFull()) {
            DPRINTF(DRAM, "Write queue full, not accepting\n");
            // remember that we have to retry this port
            retryWrReq = true;
            numWrRetry++;
            return false;
        } else {
            writePktSize[ceilLog2(size)]++;
            addToWriteQueue(pkt);
            writeReqs++;
            numReqs++;
        }
    } else {
        DPRINTF(DRAM,"Neither read nor write, ignore timing\n");
        neitherReadNorWrite++;
        accessAndRespond(pkt);
    }

    retryRdReq = false;
    retryWrReq = false;
    return true;
}

void
SimpleDRAM::processRespondEvent()
{
    DPRINTF(DRAM,
            "processRespondEvent(): Some req has reached its readyTime\n");

     PacketPtr pkt = respQueue.front()->pkt;

     // Actually responds to the requestor
     bytesConsumedRd += pkt->getSize();
     bytesRead += bytesPerCacheLine;
     accessAndRespond(pkt);

     delete respQueue.front();
     respQueue.pop_front();

     // Update stats
     avgRdQLen = readQueue.size() + respQueue.size();

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
SimpleDRAM::chooseNextWrite()
{
    // This method does the arbitration between write requests. The
    // chosen packet is simply moved to the head of the write
    // queue. The other methods know that this is the place to
    // look. For example, with FCFS, this method does nothing
    assert(!writeQueue.empty());

    if (writeQueue.size() == 1) {
        DPRINTF(DRAMWR, "Single write request, nothing to do\n");
        return;
    }

    if (memSchedPolicy == Enums::fcfs) {
        // Do nothing, since the correct request is already head
    } else if (memSchedPolicy == Enums::frfcfs) {
        list<DRAMPacket*>::iterator i = writeQueue.begin();
        bool foundRowHit = false;
        while (!foundRowHit && i != writeQueue.end()) {
            DRAMPacket* dram_pkt = *i;
            const Bank& bank = dram_pkt->bank_ref;
            if (bank.openRow == dram_pkt->row) { //FR part
                DPRINTF(DRAMWR, "Write row buffer hit\n");
                writeQueue.erase(i);
                writeQueue.push_front(dram_pkt);
                foundRowHit = true;
            } else { //FCFS part
                ;
            }
            ++i;
        }
    } else
        panic("No scheduling policy chosen\n");

    DPRINTF(DRAMWR, "Selected next write request\n");
}

bool
SimpleDRAM::chooseNextRead()
{
    // This method does the arbitration between read requests. The
    // chosen packet is simply moved to the head of the queue. The
    // other methods know that this is the place to look. For example,
    // with FCFS, this method does nothing
    if (readQueue.empty()) {
        DPRINTF(DRAM, "No read request to select\n");
        return false;
    }

    // If there is only one request then there is nothing left to do
    if (readQueue.size() == 1)
        return true;

    if (memSchedPolicy == Enums::fcfs) {
        // Do nothing, since the request to serve is already the first
        // one in the read queue
    } else if (memSchedPolicy == Enums::frfcfs) {
        for (list<DRAMPacket*>::iterator i = readQueue.begin();
             i != readQueue.end() ; ++i) {
            DRAMPacket* dram_pkt = *i;
            const Bank& bank = dram_pkt->bank_ref;
            // Check if it is a row hit
            if (bank.openRow == dram_pkt->row) { //FR part
                DPRINTF(DRAM, "Row buffer hit\n");
                readQueue.erase(i);
                readQueue.push_front(dram_pkt);
                break;
            } else { //FCFS part
                ;
            }
        }
    } else
        panic("No scheduling policy chosen!\n");

    DPRINTF(DRAM, "Selected next read request\n");
    return true;
}

void
SimpleDRAM::accessAndRespond(PacketPtr pkt)
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

        // queue the packet in the response queue to be sent out the
        // next tick
        port.schedTimingResp(pkt, curTick() + 1);
    } else {
        // @todo the packet is going to be deleted, and the DRAMPacket
        // is still having a pointer to it
        pendingDelete.push_back(pkt);
    }

    DPRINTF(DRAM, "Done\n");

    return;
}

pair<Tick, Tick>
SimpleDRAM::estimateLatency(DRAMPacket* dram_pkt, Tick inTime)
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

    const Bank& bank = dram_pkt->bank_ref;
    if (pageMgmt == Enums::open) { // open-page policy
        if (bank.openRow == dram_pkt->row) {
            // When we have a row-buffer hit,
            // we don't care about tRAS having expired or not,
            // but do care about bank being free for access
            rowHitFlag = true;

            if (bank.freeAt < inTime) {
               // CAS latency only
               accLat += tCL;
               bankLat += tCL;
            } else {
                accLat += 0;
                bankLat += 0;
            }

        } else {
            // Row-buffer miss, need to close existing row
            // once tRAS has expired, then open the new one,
            // then add cas latency.
            Tick freeTime = std::max(bank.tRASDoneAt, bank.freeAt);

            if (freeTime > inTime)
               accLat += freeTime - inTime;

            accLat += tRP + tRCD + tCL;
            bankLat += tRP + tRCD + tCL;
        }
    } else if (pageMgmt == Enums::close) {
        // With a close page policy, no notion of
        // bank.tRASDoneAt
        if (bank.freeAt > inTime)
            accLat += bank.freeAt - inTime;

        // page already closed, simply open the row, and
        // add cas latency
        accLat += tRCD + tCL;
        bankLat += tRCD + tCL;
    } else
        panic("No page management policy chosen\n");

    DPRINTF(DRAM, "Returning < %lld, %lld > from estimateLatency()\n",
            bankLat, accLat);

    return make_pair(bankLat, accLat);
}

void
SimpleDRAM::processNextReqEvent()
{
    scheduleNextReq();
}

void
SimpleDRAM::recordActivate(Tick act_tick)
{
    assert(actTicks.size() == activationLimit);

    DPRINTF(DRAM, "Activate at tick %d\n", act_tick);

    // sanity check
    if (actTicks.back() && (act_tick - actTicks.back()) < tXAW) {
        panic("Got %d activates in window %d (%d - %d) which is smaller "
              "than %d\n", activationLimit, act_tick - actTicks.back(),
              act_tick, actTicks.back(), tXAW);
    }

    // shift the times used for the book keeping, the last element
    // (highest index) is the oldest one and hence the lowest value
    actTicks.pop_back();

    // record an new activation (in the future)
    actTicks.push_front(act_tick);

    // cannot activate more than X times in time window tXAW, push the
    // next one (the X + 1'st activate) to be tXAW away from the
    // oldest in our window of X
    if (actTicks.back() && (act_tick - actTicks.back()) < tXAW) {
        DPRINTF(DRAM, "Enforcing tXAW with X = %d, next activate no earlier "
                "than %d\n", activationLimit, actTicks.back() + tXAW);
        for(int i = 0; i < ranksPerChannel; i++)
            for(int j = 0; j < banksPerRank; j++)
                // next activate must not happen before end of window
                banks[i][j].freeAt = std::max(banks[i][j].freeAt,
                                              actTicks.back() + tXAW);
    }
}

void
SimpleDRAM::doDRAMAccess(DRAMPacket* dram_pkt)
{

    DPRINTF(DRAM, "Timing access to addr %lld, rank/bank/row %d %d %d\n",
            dram_pkt->addr, dram_pkt->rank, dram_pkt->bank, dram_pkt->row);

    // estimate the bank and access latency
    pair<Tick, Tick> lat = estimateLatency(dram_pkt, curTick());
    Tick bankLat = lat.first;
    Tick accessLat = lat.second;

    // This request was woken up at this time based on a prior call
    // to estimateLatency(). However, between then and now, both the
    // accessLatency and/or busBusyUntil may have changed. We need
    // to correct for that.

    Tick addDelay = (curTick() + accessLat < busBusyUntil) ?
        busBusyUntil - (curTick() + accessLat) : 0;

    Bank& bank = dram_pkt->bank_ref;

    // Update bank state
    if (pageMgmt == Enums::open) {
        bank.openRow = dram_pkt->row;
        bank.freeAt = curTick() + addDelay + accessLat;
        // If you activated a new row do to this access, the next access
        // will have to respect tRAS for this bank. Assume tRAS ~= 3 * tRP.
        // Also need to account for t_XAW
        if (!rowHitFlag) {
            bank.tRASDoneAt = bank.freeAt + tRP;
            recordActivate(bank.freeAt - tCL - tRCD); //since this is open page,
                                                      //no tRP by default
        }
    } else if (pageMgmt == Enums::close) { // accounting for tRAS also
        // assuming that tRAS ~= 3 * tRP, and tRC ~= 4 * tRP, as is common
        // (refer Jacob/Ng/Wang and Micron datasheets)
        bank.freeAt = curTick() + addDelay + accessLat + tRP + tRP;
        recordActivate(bank.freeAt - tRP - tRP - tCL - tRCD); //essentially (freeAt - tRC)
        DPRINTF(DRAM,"doDRAMAccess::bank.freeAt is %lld\n",bank.freeAt);
    } else
        panic("No page management policy chosen\n");

    // Update request parameters
    dram_pkt->readyTime = curTick() + addDelay + accessLat + tBURST;


    DPRINTF(DRAM, "Req %lld: curtick is %lld accessLat is %d " \
                  "readytime is %lld busbusyuntil is %lld. " \
                  "Scheduling at readyTime\n", dram_pkt->addr,
                   curTick(), accessLat, dram_pkt->readyTime, busBusyUntil);

    // Make sure requests are not overlapping on the databus
    assert (dram_pkt->readyTime - busBusyUntil >= tBURST);

    // Update bus state
    busBusyUntil = dram_pkt->readyTime;

    DPRINTF(DRAM,"Access time is %lld\n",
            dram_pkt->readyTime - dram_pkt->entryTime);

    // Update stats
    totMemAccLat += dram_pkt->readyTime - dram_pkt->entryTime;
    totBankLat += bankLat;
    totBusLat += tBURST;
    totQLat += dram_pkt->readyTime - dram_pkt->entryTime - bankLat - tBURST;

    if (rowHitFlag)
        readRowHits++;

    // At this point we're done dealing with the request
    // It will be moved to a separate response queue with a
    // correct readyTime, and eventually be sent back at that
    //time
    moveToRespQ();

    // The absolute soonest you have to start thinking about the
    // next request is the longest access time that can occur before
    // busBusyUntil. Assuming you need to meet tRAS, then precharge,
    // open a new row, and access, it is ~4*tRCD.


    Tick newTime = (busBusyUntil > 4 * tRCD) ?
                   std::max(busBusyUntil - 4 * tRCD, curTick()) :
                   curTick();

    if (!nextReqEvent.scheduled() && !stopReads){
        schedule(nextReqEvent, newTime);
    } else {
        if (newTime < nextReqEvent.when())
            reschedule(nextReqEvent, newTime);
    }


}

void
SimpleDRAM::moveToRespQ()
{
    // Remove from read queue
    DRAMPacket* dram_pkt = readQueue.front();
    readQueue.pop_front();

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
        list<DRAMPacket*>::iterator i = respQueue.begin();
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
SimpleDRAM::scheduleNextReq()
{
    DPRINTF(DRAM, "Reached scheduleNextReq()\n");

    // Figure out which read request goes next, and move it to the
    // front of the read queue
    if (!chooseNextRead()) {
        // In the case there is no read request to go next, see if we
        // are asked to drain, and if so trigger writes, this also
        // ensures that if we hit the write limit we will do this
        // multiple times until we are completely drained
        if (drainManager && !writeQueue.empty() && !writeEvent.scheduled())
            triggerWrites();
    } else {
        doDRAMAccess(readQueue.front());
    }
}

Tick
SimpleDRAM::maxBankFreeAt() const
{
    Tick banksFree = 0;

    for(int i = 0; i < ranksPerChannel; i++)
        for(int j = 0; j < banksPerRank; j++)
            banksFree = std::max(banks[i][j].freeAt, banksFree);

    return banksFree;
}

void
SimpleDRAM::processRefreshEvent()
{
    DPRINTF(DRAM, "Refreshing at tick %ld\n", curTick());

    Tick banksFree = std::max(curTick(), maxBankFreeAt()) + tRFC;

    for(int i = 0; i < ranksPerChannel; i++)
        for(int j = 0; j < banksPerRank; j++)
            banks[i][j].freeAt = banksFree;

    schedule(refreshEvent, curTick() + tREFI);
}

void
SimpleDRAM::regStats()
{
    using namespace Stats;

    AbstractMemory::regStats();

    readReqs
        .name(name() + ".readReqs")
        .desc("Total number of read requests seen");

    writeReqs
        .name(name() + ".writeReqs")
        .desc("Total number of write requests seen");

    servicedByWrQ
        .name(name() + ".servicedByWrQ")
        .desc("Number of read reqs serviced by write Q");

    cpuReqs
        .name(name() + ".cpureqs")
        .desc("Reqs generatd by CPU via cache - shady");

    neitherReadNorWrite
        .name(name() + ".neitherReadNorWrite")
        .desc("Reqs where no action is needed");

    perBankRdReqs
        .init(banksPerRank * ranksPerChannel)
        .name(name() + ".perBankRdReqs")
        .desc("Track reads on a per bank basis");

    perBankWrReqs
        .init(banksPerRank * ranksPerChannel)
        .name(name() + ".perBankWrReqs")
        .desc("Track writes on a per bank basis");

    avgRdQLen
        .name(name() + ".avgRdQLen")
        .desc("Average read queue length over time")
        .precision(2);

    avgWrQLen
        .name(name() + ".avgWrQLen")
        .desc("Average write queue length over time")
        .precision(2);

    totQLat
        .name(name() + ".totQLat")
        .desc("Total cycles spent in queuing delays");

    totBankLat
        .name(name() + ".totBankLat")
        .desc("Total cycles spent in bank access");

    totBusLat
        .name(name() + ".totBusLat")
        .desc("Total cycles spent in databus access");

    totMemAccLat
        .name(name() + ".totMemAccLat")
        .desc("Sum of mem lat for all requests");

    avgQLat
        .name(name() + ".avgQLat")
        .desc("Average queueing delay per request")
        .precision(2);

    avgQLat = totQLat / (readReqs - servicedByWrQ);

    avgBankLat
        .name(name() + ".avgBankLat")
        .desc("Average bank access latency per request")
        .precision(2);

    avgBankLat = totBankLat / (readReqs - servicedByWrQ);

    avgBusLat
        .name(name() + ".avgBusLat")
        .desc("Average bus latency per request")
        .precision(2);

    avgBusLat = totBusLat / (readReqs - servicedByWrQ);

    avgMemAccLat
        .name(name() + ".avgMemAccLat")
        .desc("Average memory access latency")
        .precision(2);

    avgMemAccLat = totMemAccLat / (readReqs - servicedByWrQ);

    numRdRetry
        .name(name() + ".numRdRetry")
        .desc("Number of times rd buffer was full causing retry");

    numWrRetry
        .name(name() + ".numWrRetry")
        .desc("Number of times wr buffer was full causing retry");

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

    readRowHitRate = (readRowHits / (readReqs - servicedByWrQ)) * 100;

    writeRowHitRate
        .name(name() + ".writeRowHitRate")
        .desc("Row buffer hit rate for writes")
        .precision(2);

    writeRowHitRate = (writeRowHits / writeReqs) * 100;

    readPktSize
        .init(ceilLog2(bytesPerCacheLine) + 1)
        .name(name() + ".readPktSize")
        .desc("Categorize read packet sizes");

     writePktSize
        .init(ceilLog2(bytesPerCacheLine) + 1)
        .name(name() + ".writePktSize")
        .desc("Categorize write packet sizes");

     rdQLenPdf
        .init(readBufferSize)
        .name(name() + ".rdQLenPdf")
        .desc("What read queue length does an incoming req see");

     wrQLenPdf
        .init(writeBufferSize)
        .name(name() + ".wrQLenPdf")
        .desc("What write queue length does an incoming req see");


    bytesRead
        .name(name() + ".bytesRead")
        .desc("Total number of bytes read from memory");

    bytesWritten
        .name(name() + ".bytesWritten")
        .desc("Total number of bytes written to memory");

    bytesConsumedRd
        .name(name() + ".bytesConsumedRd")
        .desc("bytesRead derated as per pkt->getSize()");

    bytesConsumedWr
        .name(name() + ".bytesConsumedWr")
        .desc("bytesWritten derated as per pkt->getSize()");

    avgRdBW
        .name(name() + ".avgRdBW")
        .desc("Average achieved read bandwidth in MB/s")
        .precision(2);

    avgRdBW = (bytesRead / 1000000) / simSeconds;

    avgWrBW
        .name(name() + ".avgWrBW")
        .desc("Average achieved write bandwidth in MB/s")
        .precision(2);

    avgWrBW = (bytesWritten / 1000000) / simSeconds;

    avgConsumedRdBW
        .name(name() + ".avgConsumedRdBW")
        .desc("Average consumed read bandwidth in MB/s")
        .precision(2);

    avgConsumedRdBW = (bytesConsumedRd / 1000000) / simSeconds;

    avgConsumedWrBW
        .name(name() + ".avgConsumedWrBW")
        .desc("Average consumed write bandwidth in MB/s")
        .precision(2);

    avgConsumedWrBW = (bytesConsumedWr / 1000000) / simSeconds;

    peakBW
        .name(name() + ".peakBW")
        .desc("Theoretical peak bandwidth in MB/s")
        .precision(2);

    peakBW = (SimClock::Frequency / tBURST) * bytesPerCacheLine / 1000000;

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
}

void
SimpleDRAM::recvFunctional(PacketPtr pkt)
{
    // rely on the abstract memory
    functionalAccess(pkt);
}

BaseSlavePort&
SimpleDRAM::getSlavePort(const string &if_name, PortID idx)
{
    if (if_name != "port") {
        return MemObject::getSlavePort(if_name, idx);
    } else {
        return port;
    }
}

unsigned int
SimpleDRAM::drain(DrainManager *dm)
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
        // is the write queue, thus trigger writes if there are any
        // waiting and no reads waiting, otherwise wait until the
        // reads are done
        if (readQueue.empty() && !writeQueue.empty() &&
            !writeEvent.scheduled())
            triggerWrites();
    }

    if (count)
        setDrainState(Drainable::Draining);
    else
        setDrainState(Drainable::Drained);
    return count;
}

SimpleDRAM::MemoryPort::MemoryPort(const std::string& name, SimpleDRAM& _memory)
    : QueuedSlavePort(name, &_memory, queue), queue(_memory, *this),
      memory(_memory)
{ }

AddrRangeList
SimpleDRAM::MemoryPort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(memory.getAddrRange());
    return ranges;
}

void
SimpleDRAM::MemoryPort::recvFunctional(PacketPtr pkt)
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
SimpleDRAM::MemoryPort::recvAtomic(PacketPtr pkt)
{
    return memory.recvAtomic(pkt);
}

bool
SimpleDRAM::MemoryPort::recvTimingReq(PacketPtr pkt)
{
    // pass it to the memory controller
    return memory.recvTimingReq(pkt);
}

SimpleDRAM*
SimpleDRAMParams::create()
{
    return new SimpleDRAM(this);
}
