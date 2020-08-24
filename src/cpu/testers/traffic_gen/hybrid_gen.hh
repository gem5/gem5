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
 *
 * Authors: Wendy Elsasser
 */

/**
 * @file
 * Declaration of the NVM generator for issuing variable buffer
 * hit length requests and bank utilisation.
 */

#ifndef __CPU_TRAFFIC_GEN_HYBRID_GEN_HH__
#define __CPU_TRAFFIC_GEN_HYBRID_GEN_HH__

#include "base/bitfield.hh"
#include "base/intmath.hh"
#include "base_gen.hh"
#include "enums/AddrMap.hh"
#include "mem/packet.hh"

/**
 * Hybrid NVM + DRAM specific generator is for issuing request with variable
 * buffer hit length and bank utilization. Currently assumes a single
 * channel configuration.
 */
class HybridGen : public BaseGen
{

  public:

    /**
     * Create a hybrid DRAM + NVM address sequence generator.
     *
     * @param obj SimObject owning this sequence generator
     * @param requestor_id RequestorID related to the memory requests
     * @param _duration duration of this state before transitioning
     * @param start_addr_dram Start address for DRAM range
     * @param end_addr_dram End address for DRAM range
     * @param _blocksize_dram Size used for transactions injected
     * @param start_addr_nvm Start address for NVM range
     * @param end_addr_nvm End address for NVM range
     * @param _blocksize_nvm Size used for transactions injected
     * @param cacheline_size cache line size in the system
     * @param min_period Lower limit of random inter-transaction time
     * @param max_period Upper limit of random inter-transaction time
     * @param read_percent Percent of transactions that are reads
     * @param data_limit Upper limit on how much data to read/write
     * @param num_seq_pkts_dram Number of packets per stride, each _blocksize
     * @param page_size_dram Buffer size (bytes) used in the NVM
     * @param nbr_of_banks_dram Total number of parallel banks in NVM
     * @param nbr_of_banks_util_dram Number of banks to utilized,
     *                          for N banks, we will use banks: 0->(N-1)
     * @param num_seq_pkts_nvm Number of packets per stride, each _blocksize
     * @param buffer_size_nvm Buffer size (bytes) used in the NVM
     * @param nbr_of_banks_nvm Total number of parallel banks in NVM
     * @param nbr_of_banks_util_nvm Number of banks to utilized,
     *                          for N banks, we will use banks: 0->(N-1)
     * @param addr_mapping Address mapping to be used,
     *                     assumes single channel system
     * @param nbr_of_ranks_dram Number of DRAM ranks
     * @param nbr_of_ranks_nvm Number of NVM ranks
     * @param nvm_percent Percentage of traffic going to NVM
     */
    HybridGen(SimObject &obj,
           RequestorID requestor_id, Tick _duration,
           Addr start_addr_dram, Addr end_addr_dram,
           Addr blocksize_dram,
           Addr start_addr_nvm, Addr end_addr_nvm,
           Addr blocksize_nvm,
           Addr cacheline_size,
           Tick min_period, Tick max_period,
           uint8_t read_percent, Addr data_limit,
           unsigned int num_seq_pkts_dram, unsigned int page_size_dram,
           unsigned int nbr_of_banks_dram, unsigned int nbr_of_banks_util_dram,
           unsigned int num_seq_pkts_nvm, unsigned int buffer_size_nvm,
           unsigned int nbr_of_banks_nvm, unsigned int nbr_of_banks_util_nvm,
           Enums::AddrMap addr_mapping,
           unsigned int nbr_of_ranks_dram,
           unsigned int nbr_of_ranks_nvm,
           uint8_t nvm_percent);

    void enter();

    PacketPtr getNextPacket();

    /** Insert bank, rank, and column bits into packed
     *  address to create address for 1st command in a
     *  series
     * @param new_bank Bank number of next packet series
     * @param new_rank Rank value of next packet series
    */
    void genStartAddr(unsigned int new_bank , unsigned int new_rank);

    Tick nextPacketTick(bool elastic, Tick delay) const;

  protected:
    /** Start of DRAM address range */
    const Addr startAddrDram;

    /** End of DRAM address range */
    const Addr endAddrDram;

    /** Blocksize and address increment for DRAM */
    const Addr blocksizeDram;

    /** Start of DRAM address range */
    const Addr startAddrNvm;

    /** End of DRAM address range */
    const Addr endAddrNvm;

    /** Blocksize and address increment for DRAM */
    const Addr blocksizeNvm;

    /** Cache line size in the simulated system */
    const Addr cacheLineSize;

    /** Request generation period */
    const Tick minPeriod;
    const Tick maxPeriod;

     /** Percent of generated transactions that should be reads */
    const uint8_t readPercent;

    /** Maximum amount of data to manipulate */
    const Addr dataLimit;

    /** Number of sequential packets to be generated per cpu request */
    const unsigned int numSeqPktsDram;
    const unsigned int numSeqPktsNvm;

    /** Track number of sequential packets generated for a request  */
    unsigned int countNumSeqPkts;

    /** Address of request */
    Addr addr;

    /** Page size of DRAM */
    const unsigned int pageSizeDram;

    /** Number of page bits in DRAM address */
    const unsigned int pageBitsDram;

    /** Number of bank bits in DRAM address*/
    const unsigned int bankBitsDram;

    /** Number of block bits in DRAM address */
    const unsigned int blockBitsDram;

    /** Number of banks in DRAM */
    const unsigned int nbrOfBanksDram;

    /** Number of banks to be utilized for a given configuration */
    const unsigned int nbrOfBanksUtilDram;

    /** Buffer size of NVM */
    const unsigned int bufferSizeNvm;

    /** Number of buffer bits in NVM address */
    const unsigned int pageBitsNvm;

    /** Number of bank bits in NVM address*/
    const unsigned int bankBitsNvm;

    /** Number of block bits in NVM address */
    const unsigned int blockBitsNvm;

    /** Number of banks in NVM */
    const unsigned int nbrOfBanksNvm;

    /** Number of banks to be utilized for a given configuration */
    const unsigned int nbrOfBanksUtilNvm;

    /** Address mapping to be used */
    Enums::AddrMap addrMapping;

    /** Number of ranks to be utilized for a given configuration */
    const unsigned int nbrOfRanksDram;

    /** Number of rank bits in DRAM address*/
    const unsigned int rankBitsDram;

    /** Number of ranks to be utilized for a given configuration */
    const unsigned int nbrOfRanksNvm;

    /** Number of rank bits in DRAM address*/
    const unsigned int rankBitsNvm;

    /** Percent of generated transactions that should go to NVM */
    const uint8_t nvmPercent;

    /** Remember type of requests to be generated in series */
    bool isRead;

    /** Remember the interface to be generated in series */
    bool isNvm;

    /**
     * Counter to determine the amount of data
     * manipulated. Used to determine if we should continue
     * generating requests.
     */
    Addr dataManipulated;

    /** Number of sequential DRAM packets to be generated per cpu request */
    unsigned int numSeqPkts;

    /** Start of address range */
    Addr startAddr;

    /** End of address range */
    Addr endAddr;

    /** Blocksize and address increment */
    Addr blocksize;

    /** Page size of DRAM */
    unsigned int pageSize;

    /** Number of page bits in DRAM address */
    unsigned int pageBits;

    /** Number of bank bits in DRAM address*/
    unsigned int bankBits;

    /** Number of block bits in DRAM address */
    unsigned int blockBits;

    /** Number of banks in DRAM */
    unsigned int nbrOfBanks;

    /** Number of banks to be utilized for a given configuration */
    unsigned int nbrOfBanksUtil;

    /** Number of ranks to be utilized for a given configuration */
    unsigned int nbrOfRanks;

    /** Number of rank bits in DRAM address*/
    unsigned int rankBits;

};

#endif
