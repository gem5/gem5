/*
 * Copyright (c) 2003-2004 The Regents of The University of Michigan
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
 * Authors: Ron Dreslinski
 *          Ali Saidi
 */

/**
 * @file
 * Derrive a class from PhysicalMemory that support DRAM like timing access.
 */
#ifndef __MEM_DRAM_HH__
#define __MEM_DRAM_HH__

#include "base/statistics.hh"
#include "mem/physical.hh"

class DRAMMemory : public PhysicalMemory
{
  protected:
    /* added for dram support */
    const int cpu_ratio; // ratio between CPU speed and memory bus speed
    const int bus_width;  // memory access bus width (in bytes)
    /* memory access latency (<first_chunk> <inter_chunk>) */
    const std::string mem_type;
    const std::string mem_actpolicy;
    const std::string memctrladdr_type;
    const int act_lat;
    const int cas_lat;
    const int war_lat;
    const int pre_lat;
    const int dpl_lat;
    const int trc_lat;
    const int num_banks;
    const int num_cpus;

    int bank_size;
    int num_rows;
    int *active_row;
    int last_bank;
    int last_row;
    Tick *busy_until;
    int last_dev;
    bool lastCmdIsRead;
    int precharge;

    /* memory access statistics */
    int same_row_read_access;
    int srr_after_read;
    int srr_after_write;
    int same_row_write_access;
    int srw_after_read;
    int srw_after_write;

    int same_bank_read_access;
    int sbr_after_read;
    int sbr_after_write;
    int same_bank_write_access;
    int sbw_after_read;
    int sbw_after_write;

    int other_bank_read_access_hit;
    int obr_after_read_hit;
    int obr_after_write_hit;
    int other_bank_write_access_hit;
    int obw_after_read_hit;
    int obw_after_write_hit;
    // DR
    // int other_bank_read_access_miss;
    int obr_after_read_miss;
    int obr_after_write_miss;
    // DR
    // int other_bank_write_access_miss;
    int obw_after_read_miss;
    int obw_after_write_miss;

    int total_access;

    int adjacent_access;
    int adjacent_read;
    int adjacent_write;
    int command_overlapping;
    int best_case;
    int in_between_case;
    int worst_case;
    int full_overlapping;
    int partial_overlapping;

    int mem_access_details;
    int memctrlpipe_enable;

    Tick time_last_access;


    Stats::Vector<> accesses;
    Stats::Vector<> bytesRequested;
    Stats::Vector<> bytesSent;
    Stats::Vector<> compressedAccesses;

    Stats::Vector<> cycles_nCKE;
    Stats::Vector<> cycles_all_precharge_CKE;
    Stats::Vector<> cycles_all_precharge_nCKE;
    Stats::Vector<> cycles_bank_active_nCKE;
    Stats::Vector<> cycles_avg_ACT;
    Stats::Vector<> cycles_read_out;
    Stats::Vector<> cycles_write_in;
    Stats::Vector<> cycles_between_misses;
    Stats::Vector<> other_bank_read_access_miss;
    Stats::Vector<> other_bank_write_access_miss;
    Stats::Scalar<> total_latency;
    Stats::Scalar<> total_icache_req;
    Stats::Scalar<> total_arb_latency;
    Stats::Formula avg_latency;
    Stats::Formula avg_arb_latency;
    Stats::Vector2d<> bank_access_profile;


  protected:
    Tick calculateLatency(Packet *pkt);
    int prechargeBanksAround(int bank);

  public:
    struct Params : public PhysicalMemory::Params
    {
        /* additional params for dram protocol*/
        int cpu_ratio;
        int bus_width;

        std::string mem_type; /* DRDRAM, SDRAM */
        std::string mem_actpolicy; /* closed, open */
        std::string memctrladdr_type; /* interleaved, anythingelse */

        int act_lat;
        int cas_lat;
        int war_lat;
        int pre_lat;
        int dpl_lat;
        int trc_lat;
        int num_banks;
        int num_cpus;

    };
    virtual void regStats();
    DRAMMemory(Params *p);
};

#endif// __MEM_DRAM_HH__

