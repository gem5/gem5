/*
 * Copyright (c) 2020 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed here under.  You may use the software subject to the license
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
 */

#include "cpu/testers/traffic_gen/hybrid_gen.hh"

#include <algorithm>

#include "base/trace.hh"
#include "debug/TrafficGen.hh"
#include "enums/AddrMap.hh"

namespace gem5
{

HybridGen::HybridGen(SimObject &obj,
               RequestorID requestor_id, Tick _duration,
               Addr start_addr_dram, Addr end_addr_dram,
               Addr blocksize_dram,
               Addr start_addr_nvm, Addr end_addr_nvm,
               Addr blocksize_nvm,
               Addr cacheline_size,
               Tick min_period, Tick max_period,
               uint8_t read_percent, Addr data_limit,
               unsigned int num_seq_pkts_dram, unsigned int page_size_dram,
               unsigned int nbr_of_banks_dram,
               unsigned int nbr_of_banks_util_dram,
               unsigned int num_seq_pkts_nvm, unsigned int buffer_size_nvm,
               unsigned int nbr_of_banks_nvm,
               unsigned int nbr_of_banks_util_nvm,
               enums::AddrMap addr_mapping,
               unsigned int nbr_of_ranks_dram,
               unsigned int nbr_of_ranks_nvm,
               uint8_t nvm_percent)
       : BaseGen(obj, requestor_id, _duration),
         startAddrDram(start_addr_dram),
         endAddrDram(end_addr_dram),
         blocksizeDram(blocksize_dram),
         startAddrNvm(start_addr_nvm),
         endAddrNvm(end_addr_nvm),
         blocksizeNvm(blocksize_nvm),
         cacheLineSize(cacheline_size),
         minPeriod(min_period), maxPeriod(max_period),
         readPercent(read_percent), dataLimit(data_limit),
         numSeqPktsDram(num_seq_pkts_dram),
         numSeqPktsNvm(num_seq_pkts_nvm),
         countNumSeqPkts(0), addr(0),
         pageSizeDram(page_size_dram),
         pageBitsDram(floorLog2(pageSizeDram / blocksizeDram)),
         bankBitsDram(floorLog2(nbr_of_banks_dram)),
         blockBitsDram(floorLog2(blocksizeDram)),
         nbrOfBanksDram(nbr_of_banks_dram),
         nbrOfBanksUtilDram(nbr_of_banks_util_dram),
         bufferSizeNvm(buffer_size_nvm),
         pageBitsNvm(floorLog2(bufferSizeNvm / blocksizeNvm)),
         bankBitsNvm(floorLog2(nbr_of_banks_nvm)),
         blockBitsNvm(floorLog2(blocksizeNvm)),
         nbrOfBanksNvm(nbr_of_banks_nvm),
         nbrOfBanksUtilNvm(nbr_of_banks_util_nvm),
         addrMapping(addr_mapping),
         nbrOfRanksDram(nbr_of_ranks_dram),
         rankBitsDram(floorLog2(nbrOfRanksDram)),
         nbrOfRanksNvm(nbr_of_ranks_nvm),
         rankBitsNvm(floorLog2(nbrOfRanksNvm)),
         nvmPercent(nvm_percent),
         isRead(true),
         isNvm(false),
         dataManipulated(0)
{
    if (blocksizeDram > cacheLineSize)
        fatal("TrafficGen %s Dram block size (%d) is larger than "
              "cache line size (%d)\n", name(),
              blocksizeDram, cacheLineSize);

    if (blocksizeNvm > cacheLineSize)
        fatal("TrafficGen %s Nvm block size (%d) is larger than "
              "cache line size (%d)\n", name(),
              blocksizeNvm, cacheLineSize);

    if (readPercent > 100)
        fatal("%s cannot have more than 100% reads", name());

    if (minPeriod > maxPeriod)
        fatal("%s cannot have min_period > max_period", name());

    if (nbrOfBanksUtilDram > nbrOfBanksDram)
        fatal("Attempting to use more Dram banks (%d) than "
              "what is available (%d)\n",
              nbrOfBanksUtilDram, nbrOfBanksDram);

    if (nbrOfBanksUtilNvm > nbrOfBanksNvm)
        fatal("Attempting to use more Nvm banks (%d) than "
              "what is available (%d)\n",
              nbrOfBanksUtilNvm, nbrOfBanksNvm);
}

void
HybridGen::enter()
{
    // reset the counter to zero
    dataManipulated = 0;
}

PacketPtr
HybridGen::getNextPacket()
{
    // if this is the first of the packets in series to be generated,
    // start counting again
    if (countNumSeqPkts == 0) {
        isNvm = nvmPercent != 0 &&
            (nvmPercent == 100 || rng->random(0, 100) < nvmPercent);

        // choose if we generate a read or a write here
        isRead = readPercent != 0 &&
            (readPercent == 100 || rng->random(0, 100) < readPercent);

        assert((readPercent == 0 && !isRead) ||
               (readPercent == 100 && isRead) ||
               readPercent != 100);

        if (isNvm) {
            // Select the appropriate parameters for this interface
            numSeqPkts = numSeqPktsNvm;
            startAddr = startAddrNvm;
            endAddr = endAddrNvm;
            blocksize = blocksizeNvm;
            pageSize = bufferSizeNvm;
            pageBits = pageBitsNvm;
            bankBits = bankBitsNvm;
            blockBits = blockBitsNvm;
            nbrOfBanks = nbrOfBanksNvm;
            nbrOfBanksUtil = nbrOfBanksUtilNvm;
            nbrOfRanks = nbrOfRanksNvm;
            rankBits = rankBitsNvm;
        } else {
            // Select the appropriate parameters for this interface
            numSeqPkts = numSeqPktsDram;
            startAddr = startAddrDram;
            endAddr = endAddrDram;
            blocksize = blocksizeDram;
            pageSize = pageSizeDram;
            pageBits = pageBitsDram;
            bankBits = bankBitsDram;
            blockBits = blockBitsDram;
            nbrOfBanks = nbrOfBanksDram;
            nbrOfBanksUtil = nbrOfBanksUtilDram;
            nbrOfRanks = nbrOfRanksDram;
            rankBits = rankBitsDram;
        }

        countNumSeqPkts = numSeqPkts;

        // pick a random bank
        unsigned int new_bank =
            rng->random<unsigned int>(0, nbrOfBanksUtil - 1);

        // pick a random rank
        unsigned int new_rank =
            rng->random<unsigned int>(0, nbrOfRanks - 1);

        // Generate the start address of the command series
        // routine will update addr variable with bank, rank, and col
        // bits updated for random traffic mode
        genStartAddr(new_bank, new_rank);


    } else {
        // increment the column by one
        if (addrMapping == enums::RoRaBaCoCh ||
            addrMapping == enums::RoRaBaChCo)
            // Simply increment addr by blocksize to increment
            // the column by one
            addr += blocksize;

        else if (addrMapping == enums::RoCoRaBaCh) {
            // Explicity increment the column bits
            unsigned int new_col = ((addr / blocksize /
                                       nbrOfBanks / nbrOfRanks) %
                                   (pageSize / blocksize)) + 1;
            replaceBits(addr, blockBits + bankBits + rankBits + pageBits - 1,
                        blockBits + bankBits + rankBits, new_col);
        }
    }

    DPRINTF(TrafficGen, "HybridGen::getNextPacket: %c to addr %#x, "
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
HybridGen::genStartAddr(unsigned int new_bank, unsigned int new_rank)
{
    // start by picking a random address in the range
    addr = rng->random<Addr>(startAddr, endAddr - 1);

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
    unsigned int burst_per_page = pageSize / blocksize;

    // pick a random column, but ensure that there is room for
    // numSeqPkts sequential columns in the same page
    unsigned int new_col =
        rng->random<unsigned int>(0, burst_per_page - numSeqPkts);

    if (addrMapping == enums::RoRaBaCoCh ||
        addrMapping == enums::RoRaBaChCo) {
        // Block bits, then page bits, then bank bits, then rank bits
        replaceBits(addr, blockBits + pageBits + bankBits - 1,
                    blockBits + pageBits, new_bank);
        replaceBits(addr, blockBits + pageBits - 1, blockBits, new_col);
        if (rankBits != 0) {
            replaceBits(addr, blockBits + pageBits + bankBits +rankBits - 1,
                        blockBits + pageBits + bankBits, new_rank);
        }
    } else if (addrMapping == enums::RoCoRaBaCh) {
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
HybridGen::nextPacketTick(bool elastic, Tick delay) const
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
        Tick wait = rng->random(minPeriod, maxPeriod);

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

} // namespace gem5
