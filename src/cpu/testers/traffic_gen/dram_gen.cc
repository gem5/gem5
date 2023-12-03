/*
 * Copyright (c) 2012-2013, 2016-2019 ARM Limited
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

#include "cpu/testers/traffic_gen/dram_gen.hh"

#include <algorithm>

#include "base/random.hh"
#include "base/trace.hh"
#include "debug/TrafficGen.hh"
#include "enums/AddrMap.hh"

namespace gem5
{

DramGen::DramGen(SimObject &obj, RequestorID requestor_id, Tick _duration,
                 Addr start_addr, Addr end_addr, Addr _blocksize,
                 Addr cacheline_size, Tick min_period, Tick max_period,
                 uint8_t read_percent, Addr data_limit,
                 unsigned int num_seq_pkts, unsigned int page_size,
                 unsigned int nbr_of_banks_DRAM,
                 unsigned int nbr_of_banks_util, enums::AddrMap addr_mapping,
                 unsigned int nbr_of_ranks)
    : RandomGen(obj, requestor_id, _duration, start_addr, end_addr, _blocksize,
                cacheline_size, min_period, max_period, read_percent,
                data_limit),
      numSeqPkts(num_seq_pkts),
      countNumSeqPkts(0),
      addr(0),
      isRead(true),
      pageSize(page_size),
      pageBits(floorLog2(page_size / _blocksize)),
      bankBits(floorLog2(nbr_of_banks_DRAM)),
      blockBits(floorLog2(_blocksize)),
      nbrOfBanksDRAM(nbr_of_banks_DRAM),
      nbrOfBanksUtil(nbr_of_banks_util),
      addrMapping(addr_mapping),
      rankBits(floorLog2(nbr_of_ranks)),
      nbrOfRanks(nbr_of_ranks)
{
    if (nbr_of_banks_util > nbr_of_banks_DRAM)
        fatal("Attempting to use more banks (%d) than "
              "what is available (%d)\n",
              nbr_of_banks_util, nbr_of_banks_DRAM);
}

PacketPtr
DramGen::getNextPacket()
{
    // if this is the first of the packets in series to be generated,
    // start counting again
    if (countNumSeqPkts == 0) {
        countNumSeqPkts = numSeqPkts;

        // choose if we generate a read or a write here
        isRead = readPercent != 0 && (readPercent == 100 ||
                                      random_mt.random(0, 100) < readPercent);

        assert((readPercent == 0 && !isRead) ||
               (readPercent == 100 && isRead) || readPercent != 100);

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
        if (addrMapping == enums::RoRaBaCoCh ||
            addrMapping == enums::RoRaBaChCo)
            // Simply increment addr by blocksize to increment
            // the column by one
            addr += blocksize;

        else if (addrMapping == enums::RoCoRaBaCh) {
            // Explicity increment the column bits
            unsigned int new_col =
                ((addr / blocksize / nbrOfBanksDRAM / nbrOfRanks) %
                 (pageSize / blocksize)) +
                1;
            replaceBits(addr, blockBits + bankBits + rankBits + pageBits - 1,
                        blockBits + bankBits + rankBits, new_col);
        }
    }

    DPRINTF(TrafficGen,
            "DramGen::getNextPacket: %c to addr %x, "
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

    if (addrMapping == enums::RoRaBaCoCh || addrMapping == enums::RoRaBaChCo) {
        // Block bits, then page bits, then bank bits, then rank bits
        replaceBits(addr, blockBits + pageBits + bankBits - 1,
                    blockBits + pageBits, new_bank);
        replaceBits(addr, blockBits + pageBits - 1, blockBits, new_col);
        if (rankBits != 0) {
            replaceBits(addr, blockBits + pageBits + bankBits + rankBits - 1,
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

} // namespace gem5
