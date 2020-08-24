/*
 * Copyright (c) 2012-2013, 2017-2019 ARM Limited
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

/**
 * @file
 * Declaration of the DRAM generator for issuing variable page
 * hit length requests and bank utilisation.
 */

#ifndef __CPU_TRAFFIC_GEN_DRAM_GEN_HH__
#define __CPU_TRAFFIC_GEN_DRAM_GEN_HH__

#include "base/bitfield.hh"
#include "base/intmath.hh"
#include "enums/AddrMap.hh"
#include "mem/packet.hh"
#include "random_gen.hh"

/**
 * DRAM specific generator is for issuing request with variable page
 * hit length and bank utilization. Currently assumes a single
 * channel configuration.
 */
class DramGen : public RandomGen
{

  public:

    /**
     * Create a DRAM address sequence generator.
     *
     * @param obj SimObject owning this sequence generator
     * @param requestor_id RequestorID related to the memory requests
     * @param _duration duration of this state before transitioning
     * @param start_addr Start address
     * @param end_addr End address
     * @param _blocksize Size used for transactions injected
     * @param cacheline_size cache line size in the system
     * @param min_period Lower limit of random inter-transaction time
     * @param max_period Upper limit of random inter-transaction time
     * @param read_percent Percent of transactions that are reads
     * @param data_limit Upper limit on how much data to read/write
     * @param num_seq_pkts Number of packets per stride, each of _blocksize
     * @param page_size Page size (bytes) used in the DRAM
     * @param nbr_of_banks_DRAM Total number of banks in DRAM
     * @param nbr_of_banks_util Number of banks to utilized,
     *                          for N banks, we will use banks: 0->(N-1)
     * @param addr_mapping Address mapping to be used,
     *                     assumes single channel system
     */
    DramGen(SimObject &obj,
            RequestorID requestor_id, Tick _duration,
            Addr start_addr, Addr end_addr,
            Addr _blocksize, Addr cacheline_size,
            Tick min_period, Tick max_period,
            uint8_t read_percent, Addr data_limit,
            unsigned int num_seq_pkts, unsigned int page_size,
            unsigned int nbr_of_banks_DRAM, unsigned int nbr_of_banks_util,
            Enums::AddrMap addr_mapping,
            unsigned int nbr_of_ranks);

    PacketPtr getNextPacket();

    /** Insert bank, rank, and column bits into packed
     *  address to create address for 1st command in a
     *  series
     * @param new_bank Bank number of next packet series
     * @param new_rank Rank value of next packet series
    */
    void genStartAddr(unsigned int new_bank , unsigned int new_rank);

  protected:

    /** Number of sequential DRAM packets to be generated per cpu request */
    const unsigned int numSeqPkts;

    /** Track number of sequential packets generated for a request  */
    unsigned int countNumSeqPkts;

    /** Address of request */
    Addr addr;

    /** Remember type of requests to be generated in series */
    bool isRead;

    /** Page size of DRAM */
    const unsigned int pageSize;

    /** Number of page bits in DRAM address */
    const unsigned int pageBits;

    /** Number of bank bits in DRAM address*/
    const unsigned int bankBits;

    /** Number of block bits in DRAM address */
    const unsigned int blockBits;

    /** Number of banks in DRAM */
    const unsigned int nbrOfBanksDRAM;

    /** Number of banks to be utilized for a given configuration */
    const unsigned int nbrOfBanksUtil;

    /** Address mapping to be used */
    Enums::AddrMap addrMapping;

    /** Number of rank bits in DRAM address*/
    const unsigned int rankBits;

    /** Number of ranks to be utilized for a given configuration */
    const unsigned int nbrOfRanks;

};

#endif
