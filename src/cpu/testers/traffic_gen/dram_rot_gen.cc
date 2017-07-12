/*
 * Copyright (c) 2012-2013, 2016-2017 ARM Limited
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
 *
 * Authors: Thomas Grass
 *          Andreas Hansson
 *          Sascha Bischoff
 *          Neha Agarwal
 */

#include "cpu/testers/traffic_gen/dram_rot_gen.hh"

#include <algorithm>

#include "base/random.hh"
#include "base/trace.hh"
#include "debug/TrafficGen.hh"
#include "proto/packet.pb.h"

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
            // Simply increment addr by blocksize to
            // increment the column by one
            addr += blocksize;

        else if (addrMapping == 0) {
            // addrMapping=0: RoCoRaBaCh
            // Explicity increment the column bits

                    unsigned int new_col = ((addr / blocksize /
                                      nbrOfBanksDRAM / nbrOfRanks) %
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
