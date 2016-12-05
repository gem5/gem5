/*
 * Copyright (c) 2012-2013, 2016 ARM Limited
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
 * Authors: Thomas Grass
 *          Andreas Hansson
 *          Sascha Bischoff
 *          Neha Agarwal
 */

#include <algorithm>

#include "base/random.hh"
#include "base/trace.hh"
#include "cpu/testers/traffic_gen/generators.hh"
#include "debug/TrafficGen.hh"
#include "proto/packet.pb.h"

BaseGen::BaseGen(const std::string& _name, MasterID master_id, Tick _duration)
    : _name(_name), masterID(master_id), duration(_duration)
{
}

PacketPtr
BaseGen::getPacket(Addr addr, unsigned size, const MemCmd& cmd,
                   Request::FlagsType flags)
{
    // Create new request
    Request *req = new Request(addr, size, flags, masterID);
    // Dummy PC to have PC-based prefetchers latch on; get entropy into higher
    // bits
    req->setPC(((Addr)masterID) << 2);

    // Embed it in a packet
    PacketPtr pkt = new Packet(req, cmd);

    uint8_t* pkt_data = new uint8_t[req->getSize()];
    pkt->dataDynamic(pkt_data);

    if (cmd.isWrite()) {
        std::fill_n(pkt_data, req->getSize(), (uint8_t)masterID);
    }

    return pkt;
}

void
LinearGen::enter()
{
    // reset the address and the data counter
    nextAddr = startAddr;
    dataManipulated = 0;
}

PacketPtr
LinearGen::getNextPacket()
{
    // choose if we generate a read or a write here
    bool isRead = readPercent != 0 &&
        (readPercent == 100 || random_mt.random(0, 100) < readPercent);

    assert((readPercent == 0 && !isRead) || (readPercent == 100 && isRead) ||
           readPercent != 100);

    DPRINTF(TrafficGen, "LinearGen::getNextPacket: %c to addr %x, size %d\n",
            isRead ? 'r' : 'w', nextAddr, blocksize);

    // Add the amount of data manipulated to the total
    dataManipulated += blocksize;

    PacketPtr pkt = getPacket(nextAddr, blocksize,
                              isRead ? MemCmd::ReadReq : MemCmd::WriteReq);

    // increment the address
    nextAddr += blocksize;

    // If we have reached the end of the address space, reset the
    // address to the start of the range
    if (nextAddr > endAddr) {
        DPRINTF(TrafficGen, "Wrapping address to the start of "
                "the range\n");
        nextAddr = startAddr;
    }

    return pkt;
}

Tick
LinearGen::nextPacketTick(bool elastic, Tick delay) const
{
    // Check to see if we have reached the data limit. If dataLimit is
    // zero we do not have a data limit and therefore we will keep
    // generating requests for the entire residency in this state.
    if (dataLimit && dataManipulated >= dataLimit) {
        DPRINTF(TrafficGen, "Data limit for LinearGen reached.\n");
        // there are no more requests, therefore return MaxTick
        return MaxTick;
    } else {
        // return the time when the next request should take place
        Tick wait = random_mt.random(minPeriod, maxPeriod);

        // compensate for the delay experienced to not be elastic, by
        // default the value we generate is from the time we are
        // asked, so the elasticity happens automatically
        if (!elastic) {
            if (wait < delay)
                wait = 0;
            else
                wait -= delay;
        }

        return curTick() + wait;
    }
}

void
RandomGen::enter()
{
    // reset the counter to zero
    dataManipulated = 0;
}

PacketPtr
RandomGen::getNextPacket()
{
    // choose if we generate a read or a write here
    bool isRead = readPercent != 0 &&
        (readPercent == 100 || random_mt.random(0, 100) < readPercent);

    assert((readPercent == 0 && !isRead) || (readPercent == 100 && isRead) ||
           readPercent != 100);

    // address of the request
    Addr addr = random_mt.random(startAddr, endAddr - 1);

    // round down to start address of block
    addr -= addr % blocksize;

    DPRINTF(TrafficGen, "RandomGen::getNextPacket: %c to addr %x, size %d\n",
            isRead ? 'r' : 'w', addr, blocksize);

    // add the amount of data manipulated to the total
    dataManipulated += blocksize;

    // create a new request packet
    return getPacket(addr, blocksize,
                     isRead ? MemCmd::ReadReq : MemCmd::WriteReq);
}

PacketPtr
DramGen::getNextPacket()
{
    // if this is the first of the packets in series to be generated,
    // start counting again
    if (countNumSeqPkts == 0) {
        countNumSeqPkts = numSeqPkts;

        // choose if we generate a read or a write here
        isRead = readPercent != 0 &&
            (readPercent == 100 || random_mt.random(0, 100) < readPercent);

        assert((readPercent == 0 && !isRead) ||
               (readPercent == 100 && isRead) ||
               readPercent != 100);

        // pick a random bank
        unsigned int new_bank =
            random_mt.random<unsigned int>(0, nbrOfBanksUtil - 1);

        // pick a random rank
        unsigned int new_rank =
            random_mt.random<unsigned int>(0, nbrOfRanks - 1);

        // Generate the start address of the command series
        // routine will update addr variable with bank, rank, and col
        // bits updated for random traffic mode
        genStartAddr(new_bank, new_rank);

    } else {
        // increment the column by one
        if (addrMapping == 1)
            // addrMapping=1: RoRaBaCoCh/RoRaBaChCo
            // Simply increment addr by blocksize to increment the column by one
            addr += blocksize;

        else if (addrMapping == 0) {
            // addrMapping=0: RoCoRaBaCh
            // Explicity increment the column bits
            unsigned int new_col = ((addr / blocksize / nbrOfBanksDRAM / nbrOfRanks) %
                                    (pageSize / blocksize)) + 1;
            replaceBits(addr, blockBits + bankBits + rankBits + pageBits - 1,
                        blockBits + bankBits + rankBits, new_col);
        }
    }

    DPRINTF(TrafficGen, "DramGen::getNextPacket: %c to addr %x, "
            "size %d, countNumSeqPkts: %d, numSeqPkts: %d\n",
            isRead ? 'r' : 'w', addr, blocksize, countNumSeqPkts, numSeqPkts);

    // create a new request packet
    PacketPtr pkt = getPacket(addr, blocksize,
                              isRead ? MemCmd::ReadReq : MemCmd::WriteReq);

    // add the amount of data manipulated to the total
    dataManipulated += blocksize;

    // subtract the number of packets remained to be generated
    --countNumSeqPkts;

    // return the generated packet
    return pkt;
}

PacketPtr
DramRotGen::getNextPacket()
{
    // if this is the first of the packets in series to be generated,
    // start counting again
    if (countNumSeqPkts == 0) {
        countNumSeqPkts = numSeqPkts;

        // choose if we generate a read or a write here
        if (readPercent == 50) {
           if ((nextSeqCount % nbrOfBanksUtil) == 0) {
               // Change type after all banks have been rotated
               // Otherwise, keep current value
               isRead = !isRead;
           }
        } else {
           // Set randomly based on percentage
           isRead = readPercent != 0;
        }

        assert((readPercent == 0 && !isRead) ||
               (readPercent == 100 && isRead) ||
               readPercent != 100);

         // Overwrite random bank value
         // Rotate across banks
         unsigned int new_bank = nextSeqCount % nbrOfBanksUtil;

         // Overwrite random rank value
         // Will rotate to the next rank after rotating through all banks,
         // for each specified command type.

         // Use modular function to ensure that calculated rank is within
         // system limits after state transition
         unsigned int new_rank = (nextSeqCount / maxSeqCountPerRank) %
             nbrOfRanks;

         // Increment nextSeqCount
         // Roll back to 0 after completing a full rotation across
         // banks, command type, and ranks
         nextSeqCount = (nextSeqCount + 1) %
             (nbrOfRanks * maxSeqCountPerRank);

         DPRINTF(TrafficGen, "DramRotGen::getNextPacket nextSeqCount: %d "
                 "new_rank: %d  new_bank: %d\n",
                 nextSeqCount, new_rank, new_bank);

        // Generate the start address of the command series
        // routine will update addr variable with bank, rank, and col
        // bits updated for rotation scheme
        genStartAddr(new_bank, new_rank);

    } else {
        // increment the column by one
        if (addrMapping == 1)
            // addrMapping=1: RoRaBaCoCh/RoRaBaChCo
            // Simply increment addr by blocksize to increment the column by one
            addr += blocksize;

        else if (addrMapping == 0) {
            // addrMapping=0: RoCoRaBaCh
            // Explicity increment the column bits
            unsigned int new_col = ((addr / blocksize / nbrOfBanksDRAM / nbrOfRanks) %
                                    (pageSize / blocksize)) + 1;
            replaceBits(addr, blockBits + bankBits + rankBits + pageBits - 1,
                        blockBits + bankBits + rankBits, new_col);
        }
    }

    DPRINTF(TrafficGen, "DramRotGen::getNextPacket: %c to addr %x, "
            "size %d, countNumSeqPkts: %d, numSeqPkts: %d\n",
            isRead ? 'r' : 'w', addr, blocksize, countNumSeqPkts, numSeqPkts);

    // create a new request packet
    PacketPtr pkt = getPacket(addr, blocksize,
                              isRead ? MemCmd::ReadReq : MemCmd::WriteReq);

    // add the amount of data manipulated to the total
    dataManipulated += blocksize;

    // subtract the number of packets remained to be generated
    --countNumSeqPkts;

    // return the generated packet
    return pkt;
}

void
DramGen::genStartAddr(unsigned int new_bank, unsigned int new_rank)
{
    // start by picking a random address in the range
    addr = random_mt.random<Addr>(startAddr, endAddr - 1);

    // round down to start address of a block, i.e. a DRAM burst
    addr -= addr % blocksize;

    // insert the bank bits at the right spot, and align the
    // address to achieve the required hit length, this involves
    // finding the appropriate start address such that all
    // sequential packets target successive columns in the same
    // page

    // for example, if we have a stride size of 192B, which means
    // for LPDDR3 where burstsize = 32B we have numSeqPkts = 6,
    // the address generated previously can be such that these
    // 192B cross the page boundary, hence it needs to be aligned
    // so that they all belong to the same page for page hit
    unsigned int columns_per_page = pageSize / blocksize;

    // pick a random column, but ensure that there is room for
    // numSeqPkts sequential columns in the same page
    unsigned int new_col =
        random_mt.random<unsigned int>(0, columns_per_page - numSeqPkts);

    if (addrMapping == 1) {
        // addrMapping=1: RoRaBaCoCh/RoRaBaChCo
        // Block bits, then page bits, then bank bits, then rank bits
        replaceBits(addr, blockBits + pageBits + bankBits - 1,
                    blockBits + pageBits, new_bank);
        replaceBits(addr, blockBits + pageBits - 1, blockBits, new_col);
        if (rankBits != 0) {
            replaceBits(addr, blockBits + pageBits + bankBits +rankBits - 1,
                        blockBits + pageBits + bankBits, new_rank);
        }
    } else if (addrMapping == 0) {
        // addrMapping=0: RoCoRaBaCh
        // Block bits, then bank bits, then rank bits, then page bits
        replaceBits(addr, blockBits + bankBits - 1, blockBits, new_bank);
        replaceBits(addr, blockBits + bankBits + rankBits + pageBits - 1,
                    blockBits + bankBits + rankBits, new_col);
        if (rankBits != 0) {
            replaceBits(addr, blockBits + bankBits + rankBits - 1,
                        blockBits + bankBits, new_rank);
        }
    }
}

Tick
RandomGen::nextPacketTick(bool elastic, Tick delay) const
{
    // Check to see if we have reached the data limit. If dataLimit is
    // zero we do not have a data limit and therefore we will keep
    // generating requests for the entire residency in this state.
    if (dataLimit && dataManipulated >= dataLimit)
    {
        DPRINTF(TrafficGen, "Data limit for RandomGen reached.\n");
        // No more requests. Return MaxTick.
        return MaxTick;
    } else {
        // return the time when the next request should take place
        Tick wait = random_mt.random(minPeriod, maxPeriod);

        // compensate for the delay experienced to not be elastic, by
        // default the value we generate is from the time we are
        // asked, so the elasticity happens automatically
        if (!elastic) {
            if (wait < delay)
                wait = 0;
            else
                wait -= delay;
        }

        return curTick() + wait;
    }
}

TraceGen::InputStream::InputStream(const std::string& filename)
    : trace(filename)
{
    init();
}

void
TraceGen::InputStream::init()
{
    // Create a protobuf message for the header and read it from the stream
    ProtoMessage::PacketHeader header_msg;
    if (!trace.read(header_msg)) {
        panic("Failed to read packet header from trace\n");
    } else if (header_msg.tick_freq() != SimClock::Frequency) {
        panic("Trace was recorded with a different tick frequency %d\n",
              header_msg.tick_freq());
    }
}

void
TraceGen::InputStream::reset()
{
    trace.reset();
    init();
}

bool
TraceGen::InputStream::read(TraceElement& element)
{
    ProtoMessage::Packet pkt_msg;
    if (trace.read(pkt_msg)) {
        element.cmd = pkt_msg.cmd();
        element.addr = pkt_msg.addr();
        element.blocksize = pkt_msg.size();
        element.tick = pkt_msg.tick();
        element.flags = pkt_msg.has_flags() ? pkt_msg.flags() : 0;
        return true;
    }

    // We have reached the end of the file
    return false;
}

Tick
TraceGen::nextPacketTick(bool elastic, Tick delay) const
{
    if (traceComplete) {
        DPRINTF(TrafficGen, "No next tick as trace is finished\n");
        // We are at the end of the file, thus we have no more data in
        // the trace Return MaxTick to signal that there will be no
        // more transactions in this active period for the state.
        return MaxTick;
    }

    assert(nextElement.isValid());

    DPRINTF(TrafficGen, "Next packet tick is %d\n", tickOffset +
            nextElement.tick);

    // if the playback is supposed to be elastic, add the delay
    if (elastic)
        tickOffset += delay;

    return std::max(tickOffset + nextElement.tick, curTick());
}

void
TraceGen::enter()
{
    // update the trace offset to the time where the state was entered.
    tickOffset = curTick();

    // clear everything
    currElement.clear();

    // read the first element in the file and set the complete flag
    traceComplete = !trace.read(nextElement);
}

PacketPtr
TraceGen::getNextPacket()
{
    // shift things one step forward
    currElement = nextElement;
    nextElement.clear();

    // read the next element and set the complete flag
    traceComplete = !trace.read(nextElement);

    // it is the responsibility of the traceComplete flag to ensure we
    // always have a valid element here
    assert(currElement.isValid());

    DPRINTF(TrafficGen, "TraceGen::getNextPacket: %c %d %d %d 0x%x\n",
            currElement.cmd.isRead() ? 'r' : 'w',
            currElement.addr,
            currElement.blocksize,
            currElement.tick,
            currElement.flags);

    PacketPtr pkt = getPacket(currElement.addr + addrOffset,
                              currElement.blocksize,
                              currElement.cmd, currElement.flags);

    if (!traceComplete)
        DPRINTF(TrafficGen, "nextElement: %c addr %d size %d tick %d (%d)\n",
                nextElement.cmd.isRead() ? 'r' : 'w',
                nextElement.addr,
                nextElement.blocksize,
                nextElement.tick + tickOffset,
                nextElement.tick);

    return pkt;
}

void
TraceGen::exit()
{
    // Check if we reached the end of the trace file. If we did not
    // then we want to generate a warning stating that not the entire
    // trace was played.
    if (!traceComplete) {
        warn("Trace player %s was unable to replay the entire trace!\n",
             name());
    }

    // Clear any flags and start over again from the beginning of the
    // file
    trace.reset();
}
