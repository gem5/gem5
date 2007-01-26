/*
 * Copyright (c) 2004 The Regents of The University of Michigan
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
 * Authors: Ali Saidi
 *          Ron Dreslinski
 */

/*
 Copyright (c) 2000 Computer Engineering and Communication Networks Lab (TIK)
 Swiss Federal Institute of Technology (ETH) Zurich, Switzerland

 All rights reserved.
 Permission is hereby granted, without written agreement and without
 license or royalty fees, to use, copy, modify, and distribute this
 software and its documentation for any purpose, provided that the above
 copyright notice and the following two paragraphs appear in all copies
 of this software.

 IN NO EVENT SHALL THE TIK OR THE ETH ZURICH BE LIABLE TO ANY PARTY
 FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES
 ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF
 THE TIK OR THE ETH ZURICH HAVE BEEN ADVISED OF THE POSSIBILITY OF
 SUCH DAMAGE.

 THE TIK AND THE ETH ZURICH SPECIFICALLY DISCLAIM ANY WARRANTIES,
 INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE
 PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND TIK AND THE ETH ZURICH
 HAVE NO OBLIGATION TO PROVIDE MAINTENANCE, SUPPORT, UPDATES,
 ENHANCEMENTS, OR MODIFICATIONS.
*/

/* authors: Andreas Romer 4/99 - 7/99
            Matthias Gries 4/99 - 2/01


References: http://www.tik.ee.ethz.ch/
======================================
-> Publications http://www.tik.ee.ethz.ch/db/tik/publications/form_search_publications.php3


Matthias Gries: A Survey of Synchronous RAM Architectures.
TIK Report Nr. 71, Computer Engineering and Networks Lab (TIK),
Swiss Federal Institute of Technology (ETH) Zurich, April, 1999

-> DRAM survey


Matthias Gries, Andreas Romer: Performance Evaluation of Recent
DRAM Architectures for Embedded Systems.
TIK Report Nr. 82, Computer Engineering and Networks Lab (TIK),
Swiss Federal Institute of Technology (ETH) Zurich, November, 1999.

-> description of the DRAM and controller models for SimpleScalar in the appendix
(note that the current software version additionally supports overlapping in
 closed-page mode with slightly modified timing)


Matthias Gries: The Impact of Recent DRAM Architectures on Embedded Systems Performance.
Euromicro2000, Symposium on Digital Systems Design, IEEE Computer, Maastricht, Netherlands,
Vol. 1, pages 282-289, September, 2000.

-> performance study with SimpleScalar


Matthias Gries: Modeling a Memory Subsystem with Petri Nets: a Case Study.
A. Yakovlev, L. Gomes, and L. Lavagno (Eds), Hardware Design and Petri Nets,
Kluwer Academic, pages 291-310, March, 2000.

-> SDRAM + controller performance model as a high-level Petri net
*/

/**
 * @file
 * Definition of a DRAM like main memory.
 */


#include "mem/dram.hh"
#include "sim/builder.hh"
#include <stdlib.h>
#include <string>

extern int maxThreadsPerCPU;

/* SDRAM system: PC100/PC133 2-2-2 DIMM timing according to
   PC SDRAM Specification, Rev. 1.7, Intel Corp, Nov. 1999.

   64 bit DIMM consists of four 16x organized 256 Mbit SDRAMs, 128 MByte of main memory in total.*/
/* the settings above must be changed if another memory is used */
/* DRDRAM system: 16 bit channel, four chips (single RIMM), 128 MByte of main memory in total.
   Timing: Rambus Inc, Direct RDRAM, preliminary information, 256/288Mbit: 40-800 timing */


#define  DR_STACK_BASE 0x8000000 /* total size of memory: 128 Mbyte */
#define  DR_BANK_SIZE  0x100000  /* size of a bank      :   1 Mbyte */
#define  DR_ROW_SIZE 0x800       /* size of a row       :   2 Kbyte */
#define  DR_NUM_BANKS (DR_STACK_BASE/DR_BANK_SIZE)      /* number of banks        : 128 */
#define  DR_NUM_ROWS (DR_BANK_SIZE/DR_ROW_SIZE)         /* number of rows per bank: 512 */
#define  DR_DATA_BASE  0x4000000 /* Size of textsegment :  64 Mbyte */
#define  DR_NUM_BYTE_MEM  16     /* data packet capacity:  16 byte  */
#define  DR_NUM_DEVS 4   /* number of devices along channel */
#define  DR_BANK_SAMP 16 /* 16 banks are together in one group in each device: bank 15 and 16 have no shared SAMPs */
#define  DR_T_PACKET 4   /* number of cycles (in 400 MHz) the memory needs to deliver a data packet */
#define  DR_T_RCD 7  /* RAS to CAS delay */
#define  DR_T_CAC 8  /* read access delay: number of cylces from read to data  (trailing to leading edge of packet!) */
#define  DR_T_CWD 6  /* Write delay: number of cylces from write to write data (trailing to leading edge of packet!) */
#define  DR_T_RP  8  /* row precharge delay */
#define  DR_T_RTR 8  /* retire delay*/
#define  DR_T_RDP 4  /* min delay from read to precharge in cycles */
#define  DR_T_PP  8  /* precharge to precharge time to any bank in the same device */
#define  DR_T_RAS 20 /* minimal row active time */
/*the settings above need to be changed if the memory is altered*/
#define  DR_DYNAMIC_SIZE (DR_STACK_BASE - DR_DATA_BASE) /* size of the heap and stack at most: 64 Mbyte */
// #define  DR_NUM_BANKS (DR_STACK_BASE/DR_BANK_SIZE)      /* number of banks        : 128 */
// #define  DR_NUM_ROWS (DR_BANK_SIZE/DR_ROW_SIZE)         /* number of rows per bank: 512 */
#define  DR_T_OWR (DR_T_CWD + DR_T_PACKET - DR_T_RTR)   /* overlap after write retire   */
#define  DR_T_HELP (DR_T_CAC+DR_T_PACKET-DR_T_RDP+DR_T_PACKET) /* used for read after read with precharge */
/*delays until data is ready/written to the memory for the DRDRAM*/
#define  DR_T_READ_READ_SROW    (DR_T_CAC + DR_T_PACKET) /* RAR, row hit, current bank */
#define  DR_T_READ_WRITE_SROW   (DR_T_CAC + DR_T_PACKET) /* RAW, row hit, current bank */
#define  DR_T_WRITE_READ_SROW   (DR_T_CWD + DR_T_PACKET) /* WAR, row hit, current bank */
#define  DR_T_WRITE_WRITE_SROW  (DR_T_CWD + DR_T_PACKET) /* WAW, row hit, current bank */
#define  DR_T_READ_READ_SBANK   (DR_T_RP+DR_T_RCD+DR_T_CAC+DR_T_PACKET) /* RAR, row miss, current bank */
#define  DR_T_READ_WRITE_SBANK  (DR_T_RP+DR_T_RCD+DR_T_CAC+DR_T_PACKET) /* RAW, row miss, current bank */
#define  DR_T_WRITE_READ_SBANK  (DR_T_RP+DR_T_RCD+DR_T_CWD+DR_T_PACKET) /* WAR, row miss, current bank */
#define  DR_T_WRITE_WRITE_SBANK (DR_T_RP+DR_T_RCD+DR_T_CWD+DR_T_PACKET) /* WAR, row miss, current bank */
#define  DR_T_READ_READ_OBANK   (DR_T_PP+DR_T_RP+DR_T_RCD+DR_T_CAC+DR_T_PACKET) /* RAR, row miss, another bank */
#define  DR_T_READ_WRITE_OBANK  (DR_T_PP+DR_T_RP+DR_T_RCD+DR_T_CAC+DR_T_PACKET) /* RAW, row miss, another bank */
#define  DR_T_WRITE_READ_OBANK  (DR_T_PP+DR_T_RP+DR_T_RCD+DR_T_CWD+DR_T_PACKET) /* WAR, row miss, another bank */
#define  DR_T_WRITE_WRITE_OBANK (DR_T_PP+DR_T_RP+DR_T_RCD+DR_T_CWD+DR_T_PACKET) /* WAR, row miss, another bank */
/* best-case latencies (due to overlap / row hits in another bank) */
#define  DR_BEST_T_READ_READ_SROW    0                               /* RAR, row hit, current bank */
#define  DR_BEST_T_READ_WRITE_SROW   (DR_T_CAC+DR_T_PACKET-DR_T_OWR) /* RAW, row hit, current bank */
#define  DR_BEST_T_WRITE_READ_SROW   0                               /* WAR, row hit, current bank */
#define  DR_BEST_T_WRITE_WRITE_SROW  (DR_T_CWD+DR_T_PACKET-DR_T_OWR) /* WAR, row hit, current bank */
#define  DR_BEST_T_READ_READ_SBANK   (DR_T_RCD + DR_T_CAC)                            /* RAR, row miss, current bank */
#define  DR_BEST_T_READ_WRITE_SBANK  (DR_T_RP-DR_T_OWR+DR_T_RCD+DR_T_CAC+DR_T_PACKET) /* RAW, row miss, current bank */
#define  DR_BEST_T_WRITE_READ_SBANK  (DR_T_RCD+DR_T_CWD)                              /* WAR, row miss, current bank */
#define  DR_BEST_T_WRITE_WRITE_SBANK (DR_T_RP-DR_T_OWR+DR_T_RCD+DR_T_CWD+DR_T_PACKET) /* WAW, row miss, current bank */
#define  DR_BEST_T_READ_READ_OBANK   0                   /* RAR, row miss/hit, another bank   */
#define  DR_BEST_T_READ_WRITE_OBANK  (DR_T_PACKET+DR_T_CAC-DR_T_OWR) /* RAW, row miss/hit, another bank */
#define  DR_BEST_T_WRITE_READ_OBANK  0                   /* WAR, row miss/hit, another bank   */
#define  DR_BEST_T_WRITE_WRITE_OBANK 0                   /* WAW, row miss/hit, another bank   */
#define  DR_BEST_T_READ_WRITE_ODEV   (DR_T_CAC-DR_T_CWD) /* RAW, row miss/hit, another device */


#define MIN(a,b) ((a<b) ? a : b)
#define  SD_ROW_SIZE 0x1000      /* size of a row       :   4 Kbyte */



DRAMMemory::DRAMMemory(Params *p)
    : PhysicalMemory(p), cpu_ratio(p->cpu_ratio), bus_width(p->bus_width),
      mem_type(p->mem_type), mem_actpolicy(p->mem_actpolicy),
      memctrladdr_type(p->memctrladdr_type), act_lat(p->act_lat),
      cas_lat(p->cas_lat), war_lat(p->war_lat),
      pre_lat(p->pre_lat), dpl_lat(p->dpl_lat),
      trc_lat(p->trc_lat), num_banks(p->num_banks),
      num_cpus(p->num_cpus), last_dev(DR_NUM_DEVS+1),
      lastCmdIsRead(true), precharge(0), same_row_read_access(0), srr_after_read(0),
      srr_after_write(0), same_row_write_access(0), srw_after_read(0),
      srw_after_write(0), same_bank_read_access(0), sbr_after_read(0),
      sbr_after_write(0), same_bank_write_access(0), sbw_after_read(0),
      sbw_after_write(0), other_bank_read_access_hit(0), obr_after_read_hit(0),
      obr_after_write_hit(0), other_bank_write_access_hit(0),
      obw_after_read_hit(0), obw_after_write_hit(0), obr_after_read_miss(0),
      obr_after_write_miss(0),
      obw_after_read_miss(0), obw_after_write_miss(0), total_access(0),
      adjacent_access(0), adjacent_read(0), adjacent_write(0),
      command_overlapping(0), best_case(0), in_between_case(0), worst_case(0),
      full_overlapping(0), partial_overlapping(0), mem_access_details(false),
      memctrlpipe_enable(false), time_last_access(0)
{
    warn("This DRAM module has not been tested with the new memory system at all!");
        bank_size = (params()->addrRange.size() + 1) / num_banks;
        num_rows = bank_size / SD_ROW_SIZE; /* 0x1000 size of row 4Kbtye */
        active_row = new int[num_banks];
        last_bank = num_banks+1;
        last_row  = num_rows;
        busy_until = new Tick[num_banks];
        std::memset(busy_until,0,sizeof(Tick)*num_banks); /* initiliaze */

}

void
DRAMMemory::regStats()
{
    using namespace Stats;

    accesses
        .init(maxThreadsPerCPU)
        .name(name() + ".accesses")
        .desc("total number of accesses")
        .flags(total)
        ;

    bytesRequested
        .init(maxThreadsPerCPU)
        .name(name() + ".bytes_requested")
        .desc("total number of bytes requested")
        .flags(total)
        ;

    bytesSent
        .init(maxThreadsPerCPU)
        .name(name() + ".bytes_sent")
        .desc("total number of bytes sent")
        .flags(total)
        ;

    compressedAccesses
        .init(maxThreadsPerCPU)
        .name(name() + ".compressed_responses")
        .desc("total number of accesses that are compressed")
        .flags(total)
        ;

        // stats for power modelling
    cycles_nCKE
        .init(1)
        .name(name() + ".cycles_nCKE")
        .desc("cycles when CKE is low")
        .flags(total)
        ;

    cycles_all_precharge_CKE
        .init(1)
        .name(name() + ".cycles_all_precharge_CKE")
        .desc("cycles when all banks precharged")
        .flags(total)
        ;

    cycles_all_precharge_nCKE
        .init(1)
        .name(name() + ".cycles_all_precharge_nCKE")
        .desc("cycles when all banks precharged and CKE is low")
        .flags(total)
        ;

    cycles_bank_active_nCKE
        .init(1)
        .name(name() + ".cycles_bank_active_nCKE")
        .desc("cycles when banks active and CKE low")
        .flags(total)
        ;

        // we derive this from other stats later
        // so DR TODO for now this counter is unused
        cycles_avg_ACT
        .init(1)
        .name(name() + ".cycles_avg_ACT")
        .desc("avg cycles between ACT commands")
        .flags(total)
        ;

        cycles_read_out
        .init(1)
        .name(name() + ".cycles_read_out")
        .desc("cycles outputting read data")
        .flags(total)
        ;

        cycles_write_in
        .init(1)
        .name(name() + ".cycles_write_in")
        .desc("cycles inputting write data")
        .flags(total)
        ;

        cycles_between_misses
        .init(1)
        .name(name() + ".cycles_between_misses")
        .desc("cycles between open page misses")
        .flags(total)
        ;

        other_bank_read_access_miss
        .init(1)
        .name(name() + ".other_bank_read_access_miss")
        .desc("read miss count")
        .flags(total)
        ;

        other_bank_write_access_miss
        .init(1)
        .name(name() + ".other_bank_write_access_miss")
        .desc("write miss count")
        .flags(total)
        ;

    // DR TODO for now, only output stats which are involved in power equations
        total_latency
        .name(name() + ".total_latency")
        .desc("total DRAM latency")
        ;

        total_arb_latency
        .name(name() + ".total_arb_latency")
        .desc("total arbitration latency")
        ;

        avg_latency
        .name(name() + ".avg_latency")
        .desc("average DRAM latency")
        ;

        avg_arb_latency
        .name(name() + ".avg_arb_latency")
        .desc("average arbitration DRAM latency")
        ;

        bank_access_profile
        .init(num_banks,num_cpus)
        .name(name() + "[cpu][bank]")
        .desc("DRAM bank access profile")
        ;

        total_icache_req
        .name(name() + ".total_icache_req")
        .desc("total number of requests from icache")
        ;

        avg_latency = total_latency / accesses;
        avg_arb_latency = total_arb_latency / accesses;
}




// DR DEBUG: assume we have a 500 MHz CPU and 100 MHz RAM
// static float cpu_ratio = 5; // ratio between CPU speed and memory bus speed
// DR TODO: get this parameter from the simulation

static char *mem_access_output=NULL;
                /* latency of access [CPU cycles]*/
Tick
DRAMMemory::calculateLatency(PacketPtr pkt)
{

  bool cmdIsRead = pkt->isRead();

  int lat=0, temp=0, current_bank=0;
  int current_row=0, current_device=0;

  int was_miss = 0;	// determines if there was an active row miss this access

  //md_addr_t physic_address; /* linear memory address to be accessed */
  Addr physic_address; /* linear memory address to be accessed */

  int num_blocks=0;
  int corrected_overlap, /* overlap of consecutive accesses [CPU cycles] */
    overlap=0;           /* overlap of consecutive accesses [mem bus cycles] */
  int adjacent=0; /* 1 indicates that current bank is adjacent to the last accessed  one*/

  int chunks = (pkt->getSize() + (bus_width - 1)) / bus_width; // burst length
  assert(chunks >0);
  physic_address = pkt->getAddr();

    ///////////////////////////////////////////////////////////////////////////
    // DR added more stats for power modelling
    // NOTE:
    // for DRAM closed-page, automatic precharge after read or write,
    // i.e. whenever idle


    // count number of cycles where dram is not busy, use for CKE low signal
    // calculate as percentage of all clock cycles
    // if busy, do not add to idle count.  Else add cycles since last access
/* #define  SD_NUM_BANKS (SD_STACK_BASE/SD_BANK_SIZE)     */      /* number of banks */
/* #define  SD_NUM_ROWS (SD_BANK_SIZE/SD_ROW_SIZE)       */       /* number of rows per bank */
/*delays until data is ready/written to the memory for the SDRAM*/
        int SD_T_READ_READ_SROW = cas_lat; /* RAR, row hit, current bank  */
        int SD_T_READ_WRITE_SROW = cas_lat; /* RAW, row hit, current bank  */
        int SD_T_WRITE_READ_SROW = war_lat-1; /* WAR, row hit, current bank  */
        int SD_T_WRITE_WRITE_SROW = 0; /* WAW, row hit, current bank  */
        int SD_T_READ_READ_SBANK = (pre_lat+act_lat+cas_lat); /* RAR, row miss, current bank */
        int SD_T_READ_WRITE_SBANK = (pre_lat+act_lat+cas_lat+(dpl_lat-1)); /* RAW, row miss, current bank */
        int SD_T_WRITE_READ_SBANK = (pre_lat+act_lat); /* WAR, row miss, current bank */
        int SD_T_WRITE_WRITE_SBANK = (pre_lat+act_lat+(dpl_lat-1)); /* WAW, row miss, current bank */
        int SD_T_READ_READ_OBANK = (pre_lat+act_lat+cas_lat); /* RAR, row miss, another bank */
        int SD_T_READ_WRITE_OBANK = (pre_lat+act_lat+cas_lat); /* RAW, row miss, another bank */
        int SD_T_WRITE_READ_OBANK = (pre_lat+act_lat); /* WAR, row miss, another bank */
        int SD_T_WRITE_WRITE_OBANK = (pre_lat+act_lat); /* WAW, row miss, another bank */
/* best-case latencies (due to overlap / row hits in another bank) */
        int SD_BEST_T_READ_READ_SROW = 0; /* RAR, row hit, current bank  */
        int SD_BEST_T_READ_READ_SBANK = (act_lat+cas_lat); /* RAR, row miss, current bank */
        int SD_BEST_T_WRITE_READ_SBANK = (act_lat); /* WAR, row miss, current bank */
        int SD_BEST_T_READ_READ_OBANK = 0; /* RAR, row miss/hit, another bank */
        int SD_BEST_T_READ_WRITE_OBANK = cas_lat; /* RAW, row miss/hit, another bank */
        int SD_BEST_T_WRITE_READ_OBANK = (war_lat -1); /* WAR, row miss/hit, another bank */
        int SD_BEST_T_WRITE_WRITE_OBANK = 0; /* WAW, row miss/hit, another bank */

    Tick time_since_last_access = curTick-time_last_access;
    Tick time_last_miss = 0;	// used for keeping track of times between activations (page misses)
    //int was_idle = (curTick > busy_until);
        bool srow_flag = false;
        int timing_correction = 0;

    int was_idle = (curTick > busy_until[current_bank]);
    cycles_nCKE[0] += was_idle ? MIN(curTick-busy_until[current_bank], time_since_last_access) : 0;

    // bank is precharged
    //active_row[current_bank] == DR_NUM_ROWS
    int all_precharged = 1;
    int bank_max = num_banks;
    int row_max = num_rows;

    if( (mem_type == "SDRAM") && (mem_actpolicy == "closed") ) {
        // SDRAM does not use the active_row array in closed_page mode
        // TODO: handle closed page operation

    } else {		// DRDRAM uses the active_row array
        for( int i = 0; i < bank_max; i++ ) {
                if( (active_row[current_bank] != row_max)) all_precharged = 0;
        }
    }

    if(all_precharged) {
        if(was_idle) {
                cycles_all_precharge_nCKE[0] += MIN(curTick-busy_until[current_bank], time_since_last_access);
                cycles_all_precharge_CKE[0] += MIN(0, busy_until[current_bank]-time_last_access);
        }
        else {
                cycles_all_precharge_CKE[0] += time_since_last_access;
        }
    } else { // some bank is active
        if(was_idle) {
                cycles_bank_active_nCKE[0] += MIN(curTick-busy_until[current_bank], time_since_last_access);
        }
        else {
        }
    }

    if( cmdIsRead ) {
        cycles_read_out[0] += chunks;
    } else {
        cycles_write_in[0] += chunks;
    }


    time_last_access = curTick;
    ////////////////////////////////////////////////////////////////////////////

    if ((mem_type == "SDRAM") && (mem_actpolicy == "open"))
      {
        /* Split transaction on m5 makes it challenging to  */
        /* model the DRAM. A single cycle latency is assumed */
        /* for dequeueing an address bus request. In response to  */
        /* that, the current DRAM implementation assumes that a */
        /* seperate DRAM command generator / controller exists per */
        /* bank and the dequeued addresses are queued to these */
        /* controllers. We can view this as an ideal scenario for */
        /* a shared DRAM command generator / controller with */
        /* support for overlapping DRAM commands. */
        /* Compare DRAM PRE,ACT,CAS etc. latencies, DRAM clock  */
        /* frequency and the number of banks to determine whether */
        /* the ideal scenario with a shared DRAM command generator */
        /* is equivalent to having multiple DRAM command generators */
        /* per bank */
        if ((memctrladdr_type != "interleaved"))/* i.e. mc_type is linear */
          {
            current_bank=physic_address/bank_size;
            temp=physic_address-current_bank*bank_size;/*address in bank*/
            current_row=temp/SD_ROW_SIZE;
          }
        else/* mc_type interleaved */
          /* This memory controller maps the addresses differently
           * depending on the row_size, every row is mapped to another
           * bank. Thus, the text segment uses half of every bank, the heap
           * the next quarter of each bank, and the stack the rest.
           */

          {
            num_blocks = physic_address/SD_ROW_SIZE; /* row number */
            current_bank=num_blocks%num_banks;
            current_row=num_blocks/num_banks;
          }

        if (mem_access_details == true)
          {
                // DR TODO
            //fprintf(mem_accessfd,"       %09u  %4d   %3d\n",physic_address,current_row,current_bank);
          }
        else
          {
            if (mem_access_output!=0)
              {
                //fprintf(mem_accessfd,"\n");
              }
          }
        total_access++;

        if (memctrlpipe_enable == true)
          {
            overlap=(int)(busy_until[current_bank] - curTick);
          }
        else overlap = 0;

        if (cpu_ratio < 1.0)
          {
            corrected_overlap = overlap*((int)(1/cpu_ratio)); /* floor */
          }
        else
          {
            corrected_overlap = (int) (overlap/cpu_ratio);
          }

        /*fprintf(stderr,"%10.0f %10.0f %4d %4d ",(double)busy_until, (double)curTick, overlap, corrected_overlap); debugging*/

        if (cmdIsRead == lastCmdIsRead)/*same command*/
          {
            if (current_bank == last_bank)/*same bank*/
              {
                if (current_row == last_row)/*same row*/
                  {
                          /* Page Hit */
                    if (cmdIsRead)
                      {
                        if (corrected_overlap > 0)/*overlapping*/
                          {
                            /*best case*/
                            if (corrected_overlap >= cas_lat)
                              {
                                lat=SD_BEST_T_READ_READ_SROW;
                                srow_flag = true;
                                best_case++;
                                full_overlapping++;
                              }
                            else/*in between case*/
                              {
                                lat = cas_lat-corrected_overlap;
                                srow_flag = true;
                                in_between_case++;
                                partial_overlapping++;
                              }
                          }
                        else
                          {
                            /*worst case*/
                            lat = SD_T_READ_READ_SROW;
                                srow_flag = true;
                            worst_case++;
                          }
                        same_row_read_access++;
                        srr_after_read++;
                      }
                    else/*write*/
                      {/*no option case*/
                        lat = SD_T_WRITE_WRITE_SROW;
                        srow_flag = true;
                        same_row_write_access++;
                        srw_after_write++;
                        worst_case++;
                      }
                  }
                else /*other row in same bank*/
                  {
                        /* Page miss */
                    if (cmdIsRead)
                      {
                        if (corrected_overlap > 0)/*overlapping*/
                          {
                            if (corrected_overlap >= pre_lat)/*best case*/
                              {
                                lat = SD_BEST_T_READ_READ_SBANK;
                                best_case++;
                                full_overlapping++;
                              }
                            else/*in between case*/
                              {
                                lat = SD_T_READ_READ_SBANK-corrected_overlap;
                                in_between_case++;
                                partial_overlapping++;
                              }
                          }
                        else/*worst case*/
                          {
                            lat = SD_T_READ_READ_SBANK;
                            worst_case++;
                          }
                        same_bank_read_access++;
                        sbr_after_read++;
                      }
                    else/*write*/
                      {/*no option case*/
                        lat = SD_T_WRITE_WRITE_SBANK;
                        same_bank_write_access++;
                        sbw_after_write++;
                        worst_case++;
                      }
                  }
              }
            else /*other bank*/
              {
                if (cmdIsRead)
                  {
                        /* Page empty */
                    if (current_row == active_row[current_bank])/*row is still active*/
                      {
                        if (corrected_overlap > 0 )/*overlapping*/
                          {
                            if(corrected_overlap >= pre_lat)/*best case*/
                              {
                                lat = SD_BEST_T_READ_READ_OBANK;
                                best_case++;
                                full_overlapping++;
                              }
                            else/*in between case*/
                              {
                                lat = SD_T_READ_READ_OBANK - corrected_overlap;
                                in_between_case++;
                                partial_overlapping++;
                              }
                          }
                        else/*in between case*/
                          {
                            lat = SD_T_READ_READ_OBANK;
                            in_between_case++;
                          }
                        other_bank_read_access_hit++;
                        obr_after_read_hit++;
                      }
                    else/*row is not active*/
                      {
                        if (corrected_overlap > 0 )/*overlapping*/
                          {
                            if(corrected_overlap >= SD_T_READ_READ_OBANK )/*best case*/
                              {
                                lat = SD_BEST_T_READ_READ_OBANK;
                                best_case++;
                                full_overlapping++;
                              }
                            else/*in between case*/
                              {
                                lat = SD_T_READ_READ_OBANK-corrected_overlap;
                                in_between_case++;
                                partial_overlapping++;
                              }
                          }
                        else/*worst case*/
                          {
                            lat = SD_T_READ_READ_OBANK;
                            worst_case++;
                          }

                          // DR keep track of time between misses
                          was_miss = 1;

                        other_bank_read_access_miss[0]++;
                        obr_after_read_miss++;
                      }
                  }
                else/*write*/
                  {
                    if (current_row == active_row[current_bank])/*row is still active*/
                      { /*best_case*/
                        lat = SD_BEST_T_WRITE_WRITE_OBANK;
                        best_case++;
                        other_bank_write_access_hit++;
                        obw_after_write_hit++;
                      }
                    else/*row is not active*/
                      {
                        if (corrected_overlap > 0 )/*overlapping*/
                          {
                            if(corrected_overlap >=SD_T_WRITE_WRITE_OBANK)/*best case*/
                              {
                                lat = SD_BEST_T_WRITE_WRITE_OBANK;
                                best_case++;
                                full_overlapping++;
                              }
                            else/*in between case*/
                              {
                                lat = SD_T_WRITE_WRITE_OBANK-corrected_overlap;
                                in_between_case++;
                                partial_overlapping++;
                              }
                          }
                        else/*worst case*/
                          {
                            lat = SD_T_WRITE_WRITE_OBANK;
                            worst_case++;
                          }

                          // DR keep track of time between misses
                          was_miss = 1;

                        other_bank_write_access_miss[0]++;
                        obw_after_write_miss++;

                      }
                  }
              }
          }
        else /*lastCmdIsRead != cmdIsRead*/
          {
            if (current_bank == last_bank)/*same bank*/
              {
                if (current_row == last_row)/*same row*/
                  {
                        /* Page Hit */
                    if (cmdIsRead)
                      {/*worst case*/
                        lat = SD_T_READ_WRITE_SROW;
                        srow_flag = true;
                        same_row_read_access++;
                        srr_after_write++;
                        worst_case++;
                      }
                    else/*write*/
                      {/*worst case*/
                        lat = SD_T_WRITE_READ_SROW;
                        srow_flag = true;
                        same_row_write_access++;
                        srw_after_read++;
                        worst_case++;
                      }
                  }
                else /*other row in same bank*/
                  {
                        /* Page Miss */
                    if (cmdIsRead)
                      {/*worst case*/
                        lat = SD_T_READ_WRITE_SBANK;
                        same_bank_read_access++;
                        sbr_after_write++;
                        worst_case++;
                      }
                    else/*write*/
                      {
                        if (corrected_overlap > 0 )/*overlapping*/
                          {
                            if (corrected_overlap >= pre_lat)/*best case*/
                              {
                                lat = SD_BEST_T_WRITE_READ_SBANK;
                                best_case++;
                                full_overlapping++;
                              }
                            else/*in between case*/
                              {
                                lat = SD_T_WRITE_READ_SBANK-corrected_overlap;
                                in_between_case++;
                                partial_overlapping++;
                              }
                          }
                        else/*worst case*/
                          {
                            lat = SD_T_WRITE_READ_OBANK;
                            worst_case++;
                          }
                        same_bank_write_access++;
                        sbw_after_read++;
                      }
                  }
              }
            else /*other bank*/
              {
                        /* Page empty */
                if (cmdIsRead)
                  {
                    if (current_row == active_row[current_bank])/*row is still active*/
                      { /*best case*/
                        lat = SD_BEST_T_READ_WRITE_OBANK;
                        best_case++;
                        other_bank_read_access_hit++;
                        obr_after_write_hit++;
                      }
                    else/*row is not active*/
                      {
                        if (corrected_overlap > 0 )/*overlapping*/
                          {
                            if(corrected_overlap >= (pre_lat+act_lat))/*best case*/
                              {
                                lat = SD_BEST_T_READ_WRITE_OBANK;
                                best_case++;
                                full_overlapping++;
                              }
                            else/*in between case*/
                              {
                                lat = SD_T_READ_WRITE_OBANK-corrected_overlap;
                                in_between_case++;
                                partial_overlapping++;
                              }
                          }
                        else/*worst case*/
                          {
                            lat = SD_T_READ_WRITE_OBANK;
                            worst_case++;
                          }
                          // DR keep track of time between misses
                          was_miss = 1;

                        other_bank_read_access_miss[0]++;
                        obr_after_write_miss++;
                      }
                  }
                else/*write*/
                  {
                    if (current_row == active_row[current_bank])/*row is still active*/
                      { /*best case*/
                        lat = SD_BEST_T_WRITE_READ_OBANK;
                        best_case++;
                        other_bank_write_access_hit++;
                        obw_after_read_hit++;
                      }
                    else/*row is not active*/
                      {
                        if (corrected_overlap > 0 )/*overlapping*/
                          {
                            if (corrected_overlap >= (SD_T_WRITE_READ_OBANK-SD_BEST_T_WRITE_READ_OBANK))/*best case*/
                              {/*best case*/
                                lat = SD_BEST_T_WRITE_READ_OBANK;
                                best_case++;
                                full_overlapping++;
                              }
                            else/*in between case*/
                              {
                                lat = SD_T_WRITE_READ_OBANK-corrected_overlap;
                                in_between_case++;
                                partial_overlapping++;
                              }
                          }
                        else/*worst case*/
                          {
                            lat = SD_T_WRITE_READ_OBANK;
                            worst_case++;
                          }

                          // DR keep track of time between misses
                          was_miss = 1;

                        other_bank_write_access_miss[0]++;
                        obw_after_read_miss++;
                      }
                  }
              }
          }
        /*fprintf(stderr,"%4d %4d ",lat,active_row[current_bank]);debugging*/

        lat += chunks; /* burst length added*/
        if(srow_flag == false)
                timing_correction = cpu_ratio*(trc_lat - pre_lat - act_lat - cas_lat - 1);


        /*fprintf(stderr,"%4d ",lat);debugging*/

        active_row[current_bank]=current_row; /* open-page (hit) register */
        lastCmdIsRead = cmdIsRead;
        last_bank = current_bank;
        last_row  = current_row;

        if (cpu_ratio < 1.0)
          {
            lat = (lat+((int)(1/cpu_ratio)-1))/(int)(1/cpu_ratio);
          }
        else
          {
            temp = (int)(lat*cpu_ratio);
            lat = (lat*cpu_ratio == temp)?temp:(temp+1); /*round up*/
          }

        /*fprintf(stderr,"%4d \n",lat);debugging*/

        if (overlap <= 0) /*memory interface is not busy*/
          {
            if (memctrlpipe_enable == true)
              {
              busy_until[current_bank]=curTick+lat+
                        timing_correction;
              }
            else
              {
                if (busy_until[current_bank] >= curTick)
                  {
                    busy_until[current_bank]+=(lat+
                                timing_correction);
                        total_arb_latency += (busy_until[current_bank]
                                - curTick - lat
                                - timing_correction);
                    lat=busy_until[current_bank] - curTick;
                  }
                else busy_until[current_bank]=curTick+lat+
                        timing_correction;
              }
          }
        else/*the memory request will be satisfied temp cycles after curTick*/
          {
            busy_until[current_bank] +=(lat+
                        timing_correction);
            command_overlapping++;
            lat+=overlap;
                total_arb_latency += overlap;
          }

          // DR for power stats
          if( was_miss ) {
                cycles_between_misses[0] += (busy_until[current_bank] - time_last_miss);
                time_last_miss = busy_until[current_bank];
          }
        // cout <<"cpu id = " << _cpu_num << "current_bank = " << current_bank << endl;
        // if((_cpu_num < num_cpus) && (_cpu_num >= 0))
    //	bank_access_profile[_cpu_num][current_bank]++;

        return lat;
      }


    /***********************************************************/
    /********************     DRDRAM     ***********************/
    /***********************************************************/

    else if ((mem_type == "DRDRAM") && (mem_actpolicy == "open"))/*DRDRAM*/ /*a closed bank has an activ_row number of DR_NUM_ROWS: highest +1*/
      {
        if ((memctrladdr_type != "interleaved"))/* i.e. mc_type is linear */
          {
            current_bank=physic_address/DR_BANK_SIZE;
            temp=physic_address-current_bank*DR_BANK_SIZE;/*address in bank*/
            current_row=temp/DR_ROW_SIZE;
            current_device=current_bank/(DR_NUM_BANKS/DR_NUM_DEVS);
          }

        else/*mc_type interleaved*/
          /* This memory controller maps the addresses differently
           * depending on the row-size, every row is mapped to another
           * bank. So the text segment uses half of every bank. The heap
           * the next quarter of each bank and the stack the rest.
           */

          {
            num_blocks = physic_address/DR_ROW_SIZE; /* row number */
            current_bank=(num_blocks%DR_NUM_BANKS)*2; /*every 'second' bank will be used*/
            /*banks above DR_NUM_BANKS are the uneven banks*/
            current_bank = ((current_bank <  DR_NUM_BANKS) ? current_bank:(current_bank - DR_NUM_BANKS+1));
            current_row=num_blocks/DR_NUM_BANKS;
            current_device=current_bank/(DR_NUM_BANKS/DR_NUM_DEVS);
          }
        if (abs(current_bank-last_bank)==1)/*access to an adjacent bank*/
          {
            if (!((current_bank%DR_BANK_SAMP == (DR_BANK_SAMP-1))&&(last_bank%DR_BANK_SAMP == 0))/*not 15/16 (current/last)*/
                &&(!((last_bank%DR_BANK_SAMP == (DR_BANK_SAMP-1))&&(current_bank%DR_BANK_SAMP == 0))))/*not 16/15(current/last)*/
              {
                adjacent_access++;
                adjacent=1;/*an adjacent bank is accessed*/
                if (cmdIsRead)
                  adjacent_read++;
                else
                  adjacent_write++;
              }
          }
        precharge=0;/*at this moment no bank needs to be precharged*/
        if (active_row[current_bank] == DR_NUM_ROWS)/*bank is precharged*/
          {
            if (prechargeBanksAround(current_bank)> 0)/*a bank next to the current is activated*/
              {
                if ((adjacent==1)&&(precharge==1))
                  {
                    /*since adjacent banks share SAMPs, this access would be the same as (in terms of latency)
                     *an access to another row in the same bank if only one adjacent bank was active*/
                    last_bank = current_bank;
                    last_row = current_row+1;
                    precharge=0;/*set to 0 for next memory access*/
                  }
              }
          }
        if (mem_access_details == true)
          {
            //fprintf(mem_accessfd,"       %09u  %4d   %3d  %15d\n",physic_address,current_row,current_bank,(int)adjacent_access);
          }
        else
          {
            if (mem_access_output!=NULL)
              {
                //fprintf(mem_accessfd,"\n");
              }
          }
        total_access++;

        if (memctrlpipe_enable == true)
          {
            overlap=(int)(busy_until[current_bank] - curTick);
          }
        else overlap=0;

        if (cpu_ratio < 1.0)
          {
            corrected_overlap = overlap*((int)(1/cpu_ratio)); /* floor */
          }
        else
          {
            corrected_overlap = (int) (overlap/cpu_ratio);
          }

        /*fprintf(stderr,"%10.0f %10.0f %6d %6d %2d %2d ",(double)busy_until, (double)curTick, overlap, corrected_overlap,precharge,adjacent);debugging*/

        if (cmdIsRead == lastCmdIsRead)/*same command*/
          {
            if (current_bank == last_bank)/*same bank*/
              {
                if (current_row == last_row)/*same row*/
                  {
                    if (cmdIsRead)
                      {
                        if (corrected_overlap > 0)/*overlapping*/
                          {
                            /*best case*/
                            if (corrected_overlap >= DR_T_READ_READ_SROW)
                              {
                                lat=DR_BEST_T_READ_READ_SROW;
                                srow_flag = true;
                                best_case++;
                                full_overlapping++;
                              }
                            else/*in between case*/
                              {
                                lat = DR_T_READ_READ_SROW-corrected_overlap;
                                srow_flag = true;
                                in_between_case++;
                                partial_overlapping++;
                              }
                          }
                        else
                          {
                            /*worst case*/
                            lat = DR_T_READ_READ_SROW;
                                srow_flag = true;
                            worst_case++;
                          }
                        same_row_read_access++;
                        srr_after_read++;
                      }
                    else/*write, always retire the previous data*/
                      {
                        if (corrected_overlap > 0)/*overlapping*/
                          {
                            /*best case*/
                            if (corrected_overlap >= DR_T_OWR)
                              {
                                lat=DR_BEST_T_WRITE_WRITE_SROW;
                                srow_flag = true;
                                best_case++;
                                full_overlapping++;
                              }
                            else/*in between case*/
                              {
                                lat = DR_T_WRITE_WRITE_SROW-corrected_overlap;
                                srow_flag = true;
                                in_between_case++;
                                partial_overlapping++;
                              }
                          }
                        else
                          {
                            /*worst case*/
                            lat = DR_T_WRITE_WRITE_SROW;
                                srow_flag = true;
                            worst_case++;
                          }
                        same_row_write_access++;
                        srw_after_write++;
                      }
                  }
                else /*other row in same bank*/
                  {
                    if (cmdIsRead)
                      {
                        if (corrected_overlap > 0)/*overlapping*/
                          {
                            if (corrected_overlap >= DR_T_HELP)/*best case*/
                              {
                                lat = DR_BEST_T_READ_READ_SBANK;
                                best_case++;
                                full_overlapping++;
                              }
                            else/*in between case*/
                              {
                                lat = DR_T_READ_READ_SBANK-corrected_overlap;
                                in_between_case++;
                                partial_overlapping++;
                              }
                          }
                        else/*worst case*/
                          {
                            lat = DR_T_READ_READ_SBANK;
                            worst_case++;
                          }
                        same_bank_read_access++;
                        sbr_after_read++;
                      }
                    else/*write*/
                      {
                        if (corrected_overlap > 0)/*overlapping*/
                          {
                            if (corrected_overlap >= DR_T_OWR)/*best case*/
                              {
                                lat = DR_BEST_T_WRITE_WRITE_SBANK;
                                best_case++;
                                full_overlapping++;
                              }
                            else/*in between case*/
                              {
                                lat = DR_T_WRITE_WRITE_SBANK-corrected_overlap;
                                in_between_case++;
                                partial_overlapping++;
                              }
                          }
                        else/*worst case*/
                          {
                            lat = DR_T_WRITE_WRITE_SBANK;
                            worst_case++;
                          }
                        same_bank_write_access++;
                        sbw_after_write++;
                      }
                  }
              }
            else /*other bank*/
              {
                if (cmdIsRead)
                  {
                    if (current_row == active_row[current_bank])/*row is still active*/
                      {
                        if (corrected_overlap > 0 )/*overlapping*/
                          {
                            if(corrected_overlap >= (DR_T_CAC+DR_T_PACKET))/*best case*/
                              {
                                lat = DR_BEST_T_READ_READ_OBANK;
                                best_case++;
                                full_overlapping++;
                              }
                            else/*in between case*/
                              {
                                lat = DR_T_CAC+DR_T_PACKET-corrected_overlap;
                                in_between_case++;
                                partial_overlapping++;
                              }
                          }
                        else/*in between case*/
                          {
                            lat = DR_T_CAC+DR_T_PACKET;
                            in_between_case++;
                          }
                        other_bank_read_access_hit++;
                        obr_after_read_hit++;
                      }
                    else/*row is not active or bank is precharged/not active*/
                      {
                        if (active_row[current_bank]!=DR_NUM_ROWS)/*row is not active, but bank is active*/
                          {
                            if (corrected_overlap > 0 )/*overlapping*/
                              {
                                if(corrected_overlap >= (DR_T_RP+DR_T_RCD+DR_T_CAC+DR_T_PACKET))/*best case*/
                                  {
                                    lat = DR_BEST_T_READ_READ_OBANK;
                                    best_case++;
                                    full_overlapping++;
                                  }
                                else/*in between case*/
                                  {
                                    lat = DR_T_RP+DR_T_RCD+DR_T_CAC+DR_T_PACKET-corrected_overlap;
                                    in_between_case++;
                                    partial_overlapping++;
                                  }
                              }
                            else/*worst case*/
                              {
                                lat = DR_T_RP+DR_T_RCD+DR_T_CAC+DR_T_PACKET;
                                in_between_case++;
                              }
                          }
                        else/*current_row == DR_NUM_ROWS:precharged or inactive*/
                          {
                            if(precharge == 0)/*no adjacent bank is active*/
                              {
                                if (corrected_overlap > 0 )/*overlapping*/
                                  {
                                    if(corrected_overlap >= (DR_T_RCD+DR_T_CAC+DR_T_PACKET))/*best case*/
                                      {
                                        lat = DR_BEST_T_READ_READ_OBANK;
                                        best_case++;
                                        full_overlapping++;
                                      }
                                    else/*in between case*/
                                      {
                                        lat = DR_T_RCD+DR_T_CAC+DR_T_PACKET-corrected_overlap;
                                        in_between_case++;
                                        partial_overlapping++;
                                      }
                                  }
                                else/*worst case*/
                                  {
                                    lat = DR_T_RCD+DR_T_CAC+DR_T_PACKET;
                                    in_between_case++;
                                  }
                              }
                            else/*one ore two adjacent banks are active*/
                              {
                                if (precharge == 1)
                                  {
                                    if (corrected_overlap > 0 )/*overlapping*/
                                      {
                                        if(corrected_overlap >= (DR_T_RP+DR_T_RCD+DR_T_CAC+DR_T_PACKET))/*best case*/
                                          {
                                            lat = DR_BEST_T_READ_READ_OBANK;
                                            best_case++;
                                            full_overlapping++;
                                          }
                                        else/*in between case*/
                                          {
                                            lat = (DR_T_RP+DR_T_RCD+DR_T_CAC+DR_T_PACKET)-corrected_overlap;
                                            in_between_case++;
                                            partial_overlapping++;
                                          }
                                      }
                                    else/*worst case*/
                                      {
                                        lat = (DR_T_RP+DR_T_RCD+DR_T_CAC+DR_T_PACKET);
                                        in_between_case++;
                                      }
                                  }
                                else /*precharge ==2: two rows must be precharged*/
                                  {
                                    if (adjacent == 1)/*these banks are adjacent*/
                                      {
                                        if (corrected_overlap > 0 )/*overlapping*/
                                          {
                                            if(corrected_overlap >= DR_T_PP+2*DR_T_PACKET-DR_T_RDP+DR_T_CAC)/*best case*/
                                              {
                                                lat = DR_T_RDP+DR_T_RP+DR_T_RCD-DR_T_PACKET;
                                                in_between_case++;
                                                full_overlapping++;
                                              }
                                            else/*in between case*/
                                              {
                                                lat = DR_T_READ_READ_OBANK-corrected_overlap;
                                                in_between_case++;
                                                partial_overlapping++;
                                              }
                                          }
                                        else/*worst case*/
                                          {
                                            lat = DR_T_READ_READ_OBANK;
                                            worst_case++;
                                          }
                                      }
                                    else/*adjacent == 0: two not adjacent banks need to be precharged*/
                                      {
                                        if (corrected_overlap > 0 )/*overlapping*/
                                          {
                                            if(corrected_overlap >= DR_T_READ_READ_OBANK)/*best case*/
                                              {
                                                lat = DR_BEST_T_READ_READ_OBANK;
                                                best_case++;
                                                full_overlapping++;
                                              }
                                            else/*in between case*/
                                              {
                                                lat = DR_T_READ_READ_OBANK-corrected_overlap;
                                                in_between_case++;
                                                partial_overlapping++;
                                              }
                                          }
                                        else/*worst case*/
                                          {
                                            lat = DR_T_READ_READ_OBANK;
                                            worst_case++;
                                          }
                                      }
                                  }
                              }
                          }
                        other_bank_read_access_miss[0]++;
                        obr_after_read_miss++;
                      }
                  }
                else/*write*/
                  {
                    if (current_row == active_row[current_bank])/*row is still active*/
                      {
                        if (corrected_overlap > 0 )/*overlapping*/
                          {
                            if(corrected_overlap >= (DR_T_CWD+DR_T_PACKET))/*best case*/
                              {
                                lat = DR_BEST_T_WRITE_WRITE_OBANK;
                                best_case++;
                                full_overlapping++;
                              }
                            else/*in between case*/
                              {
                                lat = DR_T_CWD+DR_T_PACKET-corrected_overlap;
                                in_between_case++;
                                partial_overlapping++;
                              }
                          }
                        else/*worst case*/
                          {
                            lat = DR_T_CWD+DR_T_PACKET;
                            in_between_case++;
                          }
                        other_bank_write_access_hit++;
                        obw_after_write_hit++;
                      }
                    else/*row is not active or bank is precharged/not active*/
                      {
                        if (active_row[current_bank] != DR_NUM_ROWS)/*row is not active,but bank is active*/
                          {
                            if (corrected_overlap > 0 )/*overlapping*/
                              {
                                if(corrected_overlap >= (DR_T_RP+DR_T_RCD+DR_T_CWD+DR_T_PACKET))/*best case*/
                                  {
                                    lat = DR_BEST_T_WRITE_WRITE_OBANK;
                                    best_case++;
                                    full_overlapping++;
                                  }
                                else/*in between case*/
                                  {
                                    lat = DR_T_RP+DR_T_RCD+DR_T_CWD+DR_T_PACKET-corrected_overlap;
                                    in_between_case++;
                                    partial_overlapping++;
                                  }
                              }
                            else/*worst case*/
                              {
                                lat = DR_T_RP+DR_T_RCD+DR_T_CWD+DR_T_PACKET;
                                in_between_case++;
                              }
                          }
                        else/*current_row == DR_NUM_ROWS:precharged or inactive*/
                          {
                            if(precharge == 0)/*no adjacent bank is active*/
                              {
                                if (corrected_overlap > 0 )/*overlapping*/
                                  {
                                    if(corrected_overlap >= (DR_T_RCD+DR_T_CWD+DR_T_PACKET))/*best case*/
                                      {
                                        lat = DR_BEST_T_WRITE_WRITE_OBANK;
                                        best_case++;
                                        full_overlapping++;
                                      }
                                    else/*in between case*/
                                      {
                                        lat = DR_T_RCD+DR_T_CWD+DR_T_PACKET-corrected_overlap;
                                        in_between_case++;
                                        partial_overlapping++;
                                      }
                                  }
                                else/*worst case*/
                                  {
                                    lat = DR_T_RCD+DR_T_CWD+DR_T_PACKET;
                                    in_between_case++;
                                  }
                              }
                            else/*one ore two adjacent banks are active*/
                              {
                                if (precharge == 1)/*last_bank is no adjacent*/
                                  {
                                    if (corrected_overlap > 0 )/*overlapping*/
                                      {
                                        if(corrected_overlap >= (DR_T_RP+DR_T_RCD+DR_T_CWD+DR_T_PACKET))/*best case*/
                                          {
                                            lat = DR_BEST_T_WRITE_WRITE_OBANK;
                                            best_case++;
                                            full_overlapping++;
                                          }
                                        else/*in between case*/
                                          {
                                            lat = (DR_T_RP+DR_T_RCD+DR_T_CWD+DR_T_PACKET)-corrected_overlap;
                                            in_between_case++;
                                            partial_overlapping++;
                                          }
                                      }
                                    else/*worst case*/
                                      {
                                        lat = (DR_T_RP+DR_T_RCD+DR_T_CWD+DR_T_PACKET);
                                        in_between_case++;
                                      }
                                  }
                                else /*precharge ==2: two rows have to be precharged*/
                                  {
                                    if (adjacent == 1)/*these banks are adjacent*/
                                      {
                                        if (corrected_overlap > 0 )/*overlapping*/
                                          {
                                            if(corrected_overlap >= DR_T_OWR+DR_T_PP)/*best case*/
                                              {
                                                lat = DR_T_WRITE_WRITE_OBANK-DR_T_OWR-DR_T_PP;
                                                in_between_case++;
                                                full_overlapping++;
                                              }
                                            else/*in between case*/
                                              {
                                                lat = DR_T_WRITE_WRITE_OBANK-corrected_overlap;
                                                in_between_case++;
                                                partial_overlapping++;
                                              }
                                          }
                                        else/*worst case*/
                                          {
                                            lat = DR_T_WRITE_WRITE_OBANK;
                                            worst_case++;
                                          }
                                      }
                                    else/*adjacent == 0: two not adjacent banks need to be precharged*/
                                      {
                                        if (corrected_overlap > 0 )/*overlapping*/
                                          {
                                            if(corrected_overlap >= DR_T_WRITE_WRITE_OBANK)/*best case*/
                                              {
                                                lat = DR_BEST_T_WRITE_WRITE_OBANK;
                                                best_case++;
                                                full_overlapping++;
                                              }
                                            else/*in between case*/
                                              {
                                                lat = DR_T_WRITE_WRITE_OBANK-corrected_overlap;
                                                in_between_case++;
                                                partial_overlapping++;
                                              }
                                          }
                                        else/*worst case*/
                                          {
                                            lat = DR_T_WRITE_WRITE_OBANK;
                                            worst_case++;
                                          }
                                      }
                                  }
                              }
                          }
                        other_bank_write_access_miss[0]++;
                        obw_after_write_miss++;
                      }
                  }
              }
          }
        else /*lastCmdIsRead != cmdIsRead*/
          {
            if (current_bank == last_bank)/*same bank*/
              {
                if (current_row == last_row)/*same row*/
                  {
                    if (cmdIsRead)
                      {
                        if (corrected_overlap > 0)/*overlapping*/
                          {
                            /*best case*/
                            if (corrected_overlap >= DR_T_OWR)
                              {
                                lat=DR_BEST_T_READ_WRITE_SROW;
                                srow_flag = true;
                                best_case++;
                                full_overlapping++;
                              }
                            else/*in between case*/
                              {
                                lat = DR_T_READ_WRITE_SROW-corrected_overlap;
                                srow_flag = true;
                                in_between_case++;
                                partial_overlapping++;
                              }
                          }
                        else
                          {
                            /*worst case*/
                            lat = DR_T_READ_WRITE_SROW;
                                srow_flag = true;
                            worst_case++;
                          }
                        same_row_read_access++;
                        srr_after_write++;
                      }
                    else/*write*/
                      {
                        if (corrected_overlap > 0)/*overlapping*/
                          {
                            /*best case*/
                            if (corrected_overlap >= DR_T_WRITE_READ_SROW)
                              {
                                lat=DR_BEST_T_WRITE_READ_SROW;
                                srow_flag = true;
                                best_case++;
                                full_overlapping++;
                              }
                            else/*in between case*/
                              {
                                lat = DR_T_WRITE_READ_SROW-corrected_overlap;
                                srow_flag = true;
                                in_between_case++;
                                partial_overlapping++;
                              }
                          }
                        else
                          {
                            /*worst case*/
                            lat = DR_T_WRITE_READ_SROW;
                                srow_flag = true;
                            worst_case++;
                          }
                        same_row_write_access++;
                        srw_after_read++;
                      }
                  }
                else /*other row in same bank*/
                  {
                    if (cmdIsRead)
                      {
                        if (corrected_overlap > 0 )/*overlapping*/
                          {
                            if (corrected_overlap >= DR_T_OWR)/*best case*/
                              {
                                lat = DR_BEST_T_READ_WRITE_SBANK;
                                best_case++;
                                full_overlapping++;
                              }
                            else/*in between case*/
                              {
                                lat = DR_T_READ_WRITE_SBANK-corrected_overlap;
                                in_between_case++;
                                partial_overlapping++;
                              }
                          }
                        else/*worst case*/
                          {
                            lat = DR_T_READ_WRITE_SBANK;
                            worst_case++;
                          }
                        same_bank_read_access++;
                        sbr_after_write++;
                      }
                    else/*write*/
                      {
                        if (corrected_overlap > 0 )/*overlapping*/
                          {
                            if (corrected_overlap >= DR_T_HELP)/*best case*/
                              {
                                lat = DR_BEST_T_WRITE_READ_SBANK;
                                best_case++;
                                full_overlapping++;
                              }
                            else/*in between case*/
                              {
                                lat = DR_T_WRITE_READ_SBANK-corrected_overlap;
                                in_between_case++;
                                partial_overlapping++;
                              }
                          }
                        else/*worst case*/
                          {
                            lat = DR_T_WRITE_READ_SBANK;
                            worst_case++;
                          }
                        same_bank_write_access++;
                        sbw_after_read++;
                      }
                  }
              }
            else /*other bank*/
              {
                if (cmdIsRead)
                  {
                    if (current_row == active_row[current_bank])/*row is still active*/
                      {
                        if (corrected_overlap > 0 )/*overlapping*/
                          {
                            if(last_dev != current_device)
                              {
                                if(corrected_overlap >= DR_T_CWD+DR_T_PACKET)/*best case*/
                                  {
                                    lat = DR_BEST_T_READ_WRITE_ODEV;
                                    best_case++;
                                    full_overlapping++;
                                  }
                                else/*in between case*/
                                  {
                                    lat = DR_T_CAC+DR_T_PACKET-corrected_overlap;
                                    in_between_case++;
                                    partial_overlapping++;
                                  }
                              }
                            else /* same device */
                              {
                                if(corrected_overlap >= DR_T_OWR)/*best case*/
                                  {
                                    lat = DR_BEST_T_READ_WRITE_OBANK;
                                    best_case++;
                                    full_overlapping++;
                                  }
                                else/*in between case*/
                                  {
                                    lat = DR_T_CAC+DR_T_PACKET-corrected_overlap;
                                    in_between_case++;
                                    partial_overlapping++;
                                  }
                              }
                          }
                        else/*in between case, no overlap*/
                          {
                            lat = DR_T_CAC+DR_T_PACKET;
                            in_between_case++;
                          }
                        other_bank_read_access_hit++;
                        obr_after_write_hit++;
                      }

                    else/*row is not active or bank is precharged/not active*/
                      {
                        if (active_row[current_bank] != DR_NUM_ROWS)/*row is not active,but bank is active*/
                          {
                            if (corrected_overlap > 0 )/*overlapping*/
                              {
                                if (last_dev != current_device)
                                  {
                                    if(corrected_overlap >= (DR_T_RP+DR_T_RCD+DR_T_CWD+DR_T_PACKET))/*best case*/
                                      {
                                        lat = DR_BEST_T_READ_WRITE_ODEV;
                                        best_case++;
                                        full_overlapping++;
                                      }
                                    else/*in between case*/
                                      {
                                        lat = DR_T_RP+DR_T_RCD+DR_T_CAC+DR_T_PACKET-corrected_overlap;
                                        in_between_case++;
                                        partial_overlapping++;
                                      }
                                  }
                                else /* same device */
                                  {
                                    if(corrected_overlap >= (DR_T_RP+DR_T_RCD+DR_T_OWR))/*best case*/
                                      {
                                        lat = DR_BEST_T_READ_WRITE_OBANK;
                                        best_case++;
                                        full_overlapping++;
                                      }
                                    else/*in between case*/
                                      {
                                        lat = DR_T_RP+DR_T_RCD+DR_T_CAC+DR_T_PACKET-corrected_overlap;
                                        in_between_case++;
                                        partial_overlapping++;
                                      }
                                  }
                              }
                            else/*worst case*/
                              {
                                lat = DR_T_RP+DR_T_RCD+DR_T_CAC+DR_T_PACKET;
                                in_between_case++;
                              }
                          }
                        else/*current_row == DR_NUM_ROWS:precharged or inactive*/
                          {
                            if(precharge == 0)/*no adjacent bank is active*/
                              {
                                if (corrected_overlap > 0 )/*overlapping*/
                                  {
                                    if(last_dev != current_device)
                                      {
                                        if(corrected_overlap >= (DR_T_RCD+DR_T_CWD+DR_T_PACKET))/*best case*/
                                          {
                                            lat = DR_BEST_T_READ_WRITE_ODEV;
                                            best_case++;
                                            full_overlapping++;
                                          }
                                        else/*in between case*/
                                          {
                                            lat = DR_T_RCD+DR_T_CAC+DR_T_PACKET-corrected_overlap;
                                            in_between_case++;
                                            partial_overlapping++;
                                          }
                                      }
                                    else /* same device */
                                      {
                                        if(corrected_overlap >= (DR_T_RCD+DR_T_OWR))/*best case*/
                                          {
                                            lat = DR_BEST_T_READ_WRITE_OBANK;
                                            best_case++;
                                            full_overlapping++;
                                          }
                                        else/*in between case*/
                                          {
                                            lat = DR_T_RCD+DR_T_CAC+DR_T_PACKET-corrected_overlap;
                                            in_between_case++;
                                            partial_overlapping++;
                                          }
                                      }
                                  }
                                else/*worst case*/
                                  {
                                    lat = DR_T_RCD+DR_T_CAC+DR_T_PACKET;
                                    in_between_case++;
                                  }
                              }
                            else/*one or two adjacent banks are active*/
                              {
                                if (precharge == 1)/*an adjacent bank (!=last_bank) needs to be precharged*/
                                  {
                                    if (corrected_overlap > 0 )/*overlapping*/
                                      {
                                        if(last_dev != current_device)
                                          {
                                            if(corrected_overlap >= (DR_T_RP+DR_T_RCD+DR_T_CWD+DR_T_PACKET))/*best case*/
                                              {
                                                lat = DR_BEST_T_READ_WRITE_ODEV;
                                                best_case++;
                                                full_overlapping++;
                                              }
                                            else/*in between case*/
                                              {
                                                lat = (DR_T_RP+DR_T_RCD+DR_T_CAC+DR_T_PACKET)-corrected_overlap;
                                                in_between_case++;
                                                partial_overlapping++;
                                              }
                                          }
                                        else /* same device */
                                          {
                                            if(corrected_overlap >= (DR_T_RP+DR_T_RCD+DR_T_OWR))/*best case*/
                                              {
                                                lat = DR_BEST_T_READ_WRITE_OBANK;
                                                best_case++;
                                                full_overlapping++;
                                              }
                                            else/*in between case*/
                                              {
                                                lat = (DR_T_RP+DR_T_RCD+DR_T_CAC+DR_T_PACKET)-corrected_overlap;
                                                in_between_case++;
                                                partial_overlapping++;
                                              }
                                          }
                                      }
                                    else/*worst case*/
                                      {
                                        lat = (DR_T_RP+DR_T_RCD+DR_T_CAC+DR_T_PACKET);
                                        in_between_case++;
                                      }
                                  }
                                else /*precharge ==2: two rows have to be precharged*/
                                  {
                                    if (adjacent == 1) /* the banks are adjacent */
                                      {
                                        if (corrected_overlap > 0 )/*overlapping*/
                                          {
                                            if(corrected_overlap >= DR_T_OWR + DR_T_PP)/*best case*/
                                              {
                                                lat = DR_T_READ_WRITE_OBANK-DR_T_OWR - DR_T_PP;
                                                in_between_case++;
                                                full_overlapping++;
                                              }
                                            else/*in between case*/
                                              {
                                                lat = DR_T_READ_WRITE_OBANK-corrected_overlap;
                                                in_between_case++;
                                                partial_overlapping++;
                                              }
                                          }
                                        else/*worst case*/
                                          {
                                            lat = DR_T_READ_WRITE_OBANK;
                                            worst_case++;
                                          }
                                      }
                                    else/*adjacent == 0: two not adjacent banks need to be precharged*/
                                      {
                                        if (corrected_overlap > 0 )/*overlapping*/
                                          {
                                            if (last_dev != current_device)
                                              {
                                                if(corrected_overlap >= (DR_T_PP+DR_T_RP+DR_T_RCD+DR_T_CWD+DR_T_PACKET))/*best case*/
                                                  {
                                                    lat = DR_BEST_T_READ_WRITE_ODEV;
                                                    best_case++;
                                                    full_overlapping++;
                                                  }
                                                else/*in between case*/
                                                  {
                                                    lat = DR_T_READ_WRITE_OBANK-corrected_overlap;
                                                    in_between_case++;
                                                    partial_overlapping++;
                                                  }
                                              }
                                            else /* same device */
                                              {
                                                if(corrected_overlap >= (DR_T_PP+DR_T_RP+DR_T_RCD+DR_T_OWR))/*best case*/
                                                  {
                                                    lat = DR_BEST_T_READ_WRITE_OBANK;
                                                    best_case++;
                                                    full_overlapping++;
                                                  }
                                                else/*in between case*/
                                                  {
                                                    lat = DR_T_READ_WRITE_OBANK-corrected_overlap;
                                                    in_between_case++;
                                                    partial_overlapping++;
                                                  }
                                              }
                                          }
                                        else/*worst case*/
                                          {
                                            lat = DR_T_READ_WRITE_OBANK;
                                            worst_case++;
                                          }
                                      }
                                  }
                              }
                          }
                        other_bank_read_access_miss[0]++;
                        obr_after_write_miss++;
                      }
                  }
                else/*write*/
                  {
                    if (current_row == active_row[current_bank])/*row is still active*/
                      {
                        if (corrected_overlap > 0 )/*overlapping*/
                          {
                            if(corrected_overlap >= DR_T_CWD+DR_T_PACKET)/*best case*/
                              {
                                lat = DR_BEST_T_WRITE_READ_OBANK;
                                best_case++;
                                full_overlapping++;
                              }
                            else/*in between case*/
                              {
                                lat = DR_T_CWD+DR_T_PACKET-corrected_overlap;
                                in_between_case++;
                                partial_overlapping++;
                              }
                          }
                        else/*in between case*/
                          {
                            lat = DR_T_CWD+DR_T_PACKET;
                            in_between_case++;
                          }
                        other_bank_write_access_hit++;
                        obw_after_read_hit++;
                      }
                    else/*row is not active or bank is precharged/not active*/
                      {
                        if (active_row[current_bank] != DR_NUM_ROWS)/*row is not active,but bank is active*/
                          {
                            if (corrected_overlap > 0 )/*overlapping*/
                              {
                                if(corrected_overlap >= (DR_T_RP+DR_T_RCD+DR_T_CWD+DR_T_PACKET))/*best case*/
                                  {
                                    lat = DR_BEST_T_WRITE_READ_OBANK;
                                    best_case++;
                                    full_overlapping++;
                                  }
                                else/*in between case*/
                                  {
                                    lat = DR_T_RP+DR_T_RCD+DR_T_CWD+DR_T_PACKET-corrected_overlap;
                                    in_between_case++;
                                    partial_overlapping++;
                                  }
                              }
                            else/*worst case*/
                              {
                                lat = DR_T_RP+DR_T_RCD+DR_T_CWD+DR_T_PACKET;
                                in_between_case++;
                              }
                          }
                        else/*current_row == DR_NUM_ROWS:precharged or inactive*/
                          {
                            if(precharge == 0)/*no adjacent bank is active*/
                              {
                                if (corrected_overlap > 0 )/*overlapping*/
                                  {
                                    if(corrected_overlap >= (DR_T_RCD+DR_T_CWD+DR_T_PACKET))/*best case*/
                                      {
                                        lat = DR_BEST_T_WRITE_READ_OBANK;
                                        best_case++;
                                        full_overlapping++;
                                      }
                                    else/*in between case*/
                                      {
                                        lat = DR_T_RCD+DR_T_CWD+DR_T_PACKET-corrected_overlap;
                                        in_between_case++;
                                        partial_overlapping++;
                                      }
                                  }
                                else/*worst case*/
                                  {
                                    lat = DR_T_RCD+DR_T_CWD+DR_T_PACKET;
                                    in_between_case++;
                                  }
                              }
                            else/*one or two adjacent banks are active*/
                              {
                                if (precharge == 1)/*an adjacent bank (!=last_bank)  needs to be precharged first*/
                                  {
                                    if (corrected_overlap > 0 )/*overlapping*/
                                      {
                                        if(corrected_overlap >= (DR_T_RP+DR_T_RCD+DR_T_CWD+DR_T_PACKET))/*best case*/
                                          {
                                            lat = DR_BEST_T_WRITE_READ_OBANK;
                                            best_case++;
                                            full_overlapping++;
                                          }
                                        else/*in between case*/
                                          {
                                            lat = (DR_T_RP+DR_T_RCD+DR_T_CWD+DR_T_PACKET)-corrected_overlap;
                                            in_between_case++;
                                            partial_overlapping++;
                                          }
                                      }
                                    else/*worst case*/
                                      {
                                        lat = (DR_T_RP+DR_T_RCD+DR_T_CWD+DR_T_PACKET);
                                        in_between_case++;
                                      }
                                  }
                                else /*precharge ==2: two rows have to be precharged*/
                                  {
                                    if (adjacent == 1)/*these banks are adjacent*/
                                      {
                                        if (corrected_overlap > 0 )/*overlapping*/
                                          {
                                            if(corrected_overlap >= DR_T_PP-DR_T_RDP+2*DR_T_PACKET+DR_T_CAC)/*best case*/
                                              {
                                                lat = DR_T_WRITE_READ_OBANK-(DR_T_PP-DR_T_RDP+2*DR_T_PACKET+DR_T_CAC);
                                                in_between_case++;
                                                full_overlapping++;
                                              }
                                            else/*in between case*/
                                              {
                                                lat = DR_T_WRITE_READ_OBANK-corrected_overlap;
                                                in_between_case++;
                                                partial_overlapping++;
                                              }
                                          }
                                        else/*worst case*/
                                          {
                                            lat = DR_T_WRITE_READ_OBANK;
                                            worst_case++;
                                          }
                                      }
                                    else/*adjacent == 0: two not adjacent banks need to be precharged*/
                                      {
                                        if (corrected_overlap > 0 )/*overlapping*/
                                          {
                                            if(corrected_overlap >= DR_T_WRITE_READ_OBANK)/*best case*/
                                              {
                                                lat = DR_BEST_T_WRITE_READ_OBANK;
                                                best_case++;
                                                full_overlapping++;
                                              }
                                            else/*in between case*/
                                              {
                                                lat = DR_T_WRITE_READ_OBANK-corrected_overlap;
                                                in_between_case++;
                                                partial_overlapping++;
                                              }
                                          }
                                        else/*worst case*/
                                          {
                                            lat = DR_T_WRITE_READ_OBANK;
                                            worst_case++;
                                          }
                                      }
                                  }
                              }
                          }
                        other_bank_write_access_miss[0]++;
                        obw_after_read_miss++;
                      }
                  }
              }
          }
        /*fprintf(stderr,"%4d %4d ",lat,active_row[current_bank]);debugging*/

        lat += chunks * DR_T_PACKET; /*every 128 bit need DR_NUM_CYCLES*/

        /*fprintf(stderr,"%4d ",lat);debugging*/

        if (cpu_ratio < 1.0)
          {
            lat = (lat+((int)(1/cpu_ratio)-1))/(int)(1/cpu_ratio);
          }
        else
          {
            temp = (int)(lat*cpu_ratio);
            lat = (lat*cpu_ratio == temp)?temp:(temp+1); /*round up*/
          }

        active_row[current_bank]=current_row;
        lastCmdIsRead = cmdIsRead;
        last_bank=current_bank;
        last_row = current_row;
        last_dev = current_device;

        /*fprintf(stderr,"%4d \n",lat);debugging*/

        if (overlap <= 0) /*memory interface is not busy*/
          {
            if (memctrlpipe_enable == true)
              {
                busy_until[current_bank] =curTick+lat;
              }
            else
              {
                if (busy_until[current_bank] >= curTick)
                  {
                    busy_until[current_bank] +=lat;
                    lat=busy_until[current_bank] - curTick;
                  }
                else busy_until[current_bank] = curTick+lat;
              }
          }
        else/*the memory request will be satisfied temp cycles after curTick*/
          {
            busy_until[current_bank] +=lat;
            command_overlapping++;
            lat+=overlap;
          }

        // if((_cpu_num < num_cpus) && (_cpu_num >= 0))
                // cout <<"cpu id = " << _cpu_num << "current_bank = " << current_bank << endl;
    //	bank_access_profile[_cpu_num][current_bank]++;
        return lat;
      }

    /******************************************************************/

        else if ((mem_type== "SDRAM") && (mem_actpolicy == "closed") && (memctrlpipe_enable == true))
           /* SDRAM closed-page with overlap, 2/00 MG */
      {
        if ((memctrladdr_type != "interleaved"))/* i.e. mc_type is linear*/
          {
            // current_bank=physic_address/SD_BANK_SIZE;
            current_bank=physic_address/bank_size;
          }
        else/*mc_type interleaved*/
          /* This memory management unit maps the addresses different
           * depending on the row_size, every row is mapped to another
           * bank. So the text segment uses half of every bank. The heap
           * the next quarter of each bank and the stack the rest.
           */
          {
            num_blocks = physic_address/SD_ROW_SIZE; /* row number */
            // current_bank=num_blocks%SD_NUM_BANKS;
            current_bank=num_blocks%num_banks;
          }

        if (mem_access_details == true)
          {
            //fprintf(mem_accessfd,"       %09u   %3d\n",physic_address,current_bank);
          }
        else
          {
            if (mem_access_output!=NULL)
              {
                //fprintf(mem_accessfd,"\n");
              }
          }
        total_access++;

        overlap=(int)(busy_until[current_bank] - curTick);

        if (current_bank == last_bank)/*same bank*/
          {
            if ((lastCmdIsRead == cmdIsRead) && (cmdIsRead))/* RAR */
              {
                lat = act_lat + cas_lat;
              }
            else if ((lastCmdIsRead == cmdIsRead) && (!cmdIsRead)) /* WAW */
              {
                lat = act_lat;
              }
            else if ((lastCmdIsRead != cmdIsRead) && (cmdIsRead)) /* RAW */
              {
                lat = act_lat + cas_lat;
              }
            else /* WAR */
              {
                lat = act_lat;
              }
          }
        else /* other bank */
          {
            if (cpu_ratio < 1.0)
              {
                corrected_overlap = overlap*((int)(1/cpu_ratio)); /* floor */
              }
            else
              {
                corrected_overlap = (int) (overlap/cpu_ratio);
              }

            if ((lastCmdIsRead == cmdIsRead) && (cmdIsRead))/* RAR */
              {
                if (corrected_overlap > act_lat + cas_lat)
                  {
                    lat = 0;
                    best_case++;
                    full_overlapping++;
                  }
                else if (corrected_overlap > 0)
                  {
                    lat = act_lat + cas_lat - corrected_overlap;
                    in_between_case++;
                    partial_overlapping++;
                  }
                else
                  {
                    lat = act_lat + cas_lat;
                    worst_case++;
                  }
              }
            else if ((lastCmdIsRead == cmdIsRead) && (!cmdIsRead)) /* WAW */
              {
                if (corrected_overlap > act_lat + pre_lat + (dpl_lat-1))
                  {
                    lat = - pre_lat - dpl_lat +1;
                    best_case++;
                    full_overlapping++;
                  }
                else if (corrected_overlap > 0)
                  {
                    lat = act_lat - corrected_overlap;
                    in_between_case++;
                    partial_overlapping++;
                  }
                else
                  {
                    lat = act_lat;
                    worst_case++;
                  }
              }
            else if ((lastCmdIsRead != cmdIsRead) && (cmdIsRead)) /* RAW */
              {
                if (corrected_overlap > cas_lat + pre_lat + dpl_lat - 1 )
                  {
                    lat = act_lat - (pre_lat + dpl_lat - 1);
                    best_case++;
                    partial_overlapping++;
                  }
                else if (corrected_overlap > 0)
                  {
                    lat = act_lat + cas_lat - corrected_overlap;
                    in_between_case++;
                    partial_overlapping++;
                  }
                else
                  {
                    lat = act_lat + cas_lat;
                    worst_case++;
                  }
              }
            else /* WAR */
              {
                if (corrected_overlap > act_lat - (dpl_lat-1))
                  {
                    lat = dpl_lat-1;
                    best_case++;
                    partial_overlapping++;
                  }
                else if (corrected_overlap > 0)
                  {
                    lat = act_lat - corrected_overlap;
                    in_between_case++;
                    partial_overlapping++;
                  }
                else
                  {
                    lat = act_lat;
                    worst_case++;
                  }
              }
          }
        lastCmdIsRead = cmdIsRead;
        last_bank=current_bank;
        last_row = current_row;

        lat += chunks;

        if (cpu_ratio < 1.0)
          {
            lat = (lat+((int)(1/cpu_ratio)-1))/(int)(1/cpu_ratio);
          }
        else
          {
            temp = (int)(lat*cpu_ratio);
            lat = (lat*cpu_ratio == temp)?temp:(temp+1); /*round up*/
          }

        /*fprintf(stderr,"%4d \n",lat); debugging */

        if (overlap <= 0) /*memory interface is not busy*/
          {
            busy_until[current_bank] = curTick+lat;
          }
        else /*the memory request will be satisfied temp cycles after curTick*/
          {
            busy_until[current_bank] +=lat;
            command_overlapping++;
                total_arb_latency += overlap;
            lat+=overlap;
          }
        if (!cmdIsRead)
          {
           temp = (int)(((dpl_lat-1) + pre_lat)*cpu_ratio);
           busy_until[current_bank] += (((dpl_lat-1) + pre_lat)*cpu_ratio == temp)?temp:(temp+1);
          }



        /*fprintf(stderr,"%10.0f %10.0f %4d %4d \n",(double)busy_until, (double)curTick, overlap, lat);debug*/
        // if((_cpu_num < num_cpus) && (_cpu_num >= 0))
                // cout <<"cpu id = " << _cpu_num << "current_bank = " << current_bank << endl;
    //	bank_access_profile[_cpu_num][current_bank]++;
        return lat;
      }

    /******************************************************************/

    else if ((mem_type == "DRDRAM") && (mem_actpolicy == "closed") && (memctrlpipe_enable == true))
      /* DRDRAM closed-page with overlap*/
     {

        if ((memctrladdr_type != "interleaved"))/*i.e. mc_type is linear*/
          {
            current_bank=physic_address/DR_BANK_SIZE;
            current_device=current_bank/(DR_NUM_BANKS/DR_NUM_DEVS);
          }
        else/*mc_type interleaved*/
          /* This memory management unit maps the addresses different
           * depending on the row-size, every row is mapped to another
           * bank. So the text segment uses half of every bank. The heap
           * the next quarter of each bank and the stack the rest.
           */
          {
            num_blocks = physic_address/DR_ROW_SIZE; /* row number */
            current_bank=(num_blocks%DR_NUM_BANKS)*2; /*every 'second' bank will be used*/
            /*banks above DR_NUM_BANKS are the uneven banks*/
            current_bank = ((current_bank <  DR_NUM_BANKS) ? current_bank:(current_bank - DR_NUM_BANKS+1));
            current_device=current_bank/(DR_NUM_BANKS/DR_NUM_DEVS);
          }


        if (mem_access_details == true)
          {
            //fprintf(mem_accessfd,"       %09u   %3d  \n",physic_address,current_bank);
          }
        else
          {
            if (mem_access_output!=NULL)
              {
                //fprintf(mem_accessfd,"\n");
              }
          }
        total_access++;

        overlap=(int)(busy_until[current_bank] - curTick);

        if (cpu_ratio < 1.0)
          {
            corrected_overlap = overlap*((int)(1/cpu_ratio)); /* floor */
          }
        else
          {
            corrected_overlap = (int) (overlap/cpu_ratio);
          }

        if (current_bank == last_bank)/*same bank*/
          {
            if ((lastCmdIsRead == cmdIsRead) && (cmdIsRead))/* RAR */
              {
                lat = DR_T_RCD + DR_T_CAC + DR_T_PACKET;
                worst_case++;
              }
            else if ((lastCmdIsRead == cmdIsRead) && (!cmdIsRead)) /* WAW */
              {
                lat = DR_T_RCD + DR_T_CWD + DR_T_PACKET;
                worst_case++;
              }
            else if ((lastCmdIsRead != cmdIsRead) && (cmdIsRead)) /* RAW */
              {
                lat = DR_T_RCD + DR_T_CAC + DR_T_PACKET;
                worst_case++;
              }
            else /* WAR */
              {
                lat = DR_T_RCD + DR_T_CWD + DR_T_PACKET;
                worst_case++;
              }
          }
        else /* other bank */
          {
            if ((lastCmdIsRead == cmdIsRead) && (cmdIsRead))/* RAR */
              {
                if (corrected_overlap > DR_T_RAS + DR_T_RP - 2 * DR_T_PACKET)
                  {
                    lat = - DR_T_RAS + DR_T_RCD + DR_T_PACKET + DR_T_CAC;
                    best_case++;
                    full_overlapping++;
                  }
                else if (corrected_overlap > 0)
                  {
                    lat = DR_T_RCD + DR_T_CAC + DR_T_PACKET - corrected_overlap;
                    in_between_case++;
                    partial_overlapping++;
                  }
                else
                  {
                    lat = DR_T_RCD + DR_T_CAC + DR_T_PACKET;
                    worst_case++;
                  }
              }
            else if ((lastCmdIsRead == cmdIsRead) && (!cmdIsRead)) /* WAW */
              {
                if (corrected_overlap > DR_T_RCD + DR_T_RTR  + DR_T_RP)
                  {
                    lat = - DR_T_CWD - 2 * DR_T_PACKET + DR_T_RTR;
                    best_case++;
                    full_overlapping++;
                  }
                else if (corrected_overlap > 0)
                  {
                    lat = DR_T_RCD + DR_T_CWD + DR_T_PACKET - corrected_overlap;
                    in_between_case++;
                    partial_overlapping++;
                  }
                else
                  {
                    lat = DR_T_RCD + DR_T_CWD + DR_T_PACKET;
                    worst_case++;
                  }
              }
            else if ((lastCmdIsRead != cmdIsRead) && (cmdIsRead)) /* RAW */
              {
                if (current_device == last_dev) /* same device */
                  {
                    if (corrected_overlap >  DR_T_RCD + DR_T_RP)
                      {
                        lat = DR_T_PACKET + DR_T_CAC - DR_T_RP;
                        best_case++;
                        partial_overlapping++;
                      }
                    else if (corrected_overlap > 0)
                      {
                        lat = DR_T_RCD + DR_T_CAC + DR_T_PACKET - corrected_overlap;
                        in_between_case++;
                        partial_overlapping++;
                      }
                    else
                      {
                        lat = DR_T_RCD + DR_T_CAC + DR_T_PACKET;
                        worst_case++;
                      }
                  }
                else /* other device */
                  {
                    if (corrected_overlap > DR_T_RCD + DR_T_RP + 2 * DR_T_PACKET)
                      {
                        lat = - DR_T_PACKET + DR_T_CAC - DR_T_RP;
                        best_case++;
                        partial_overlapping++;
                      }
                    else if (corrected_overlap > 0)
                      {
                        lat = DR_T_RCD + DR_T_CAC + DR_T_PACKET - corrected_overlap;
                        in_between_case++;
                        partial_overlapping++;
                      }
                    else
                      {
                        lat = DR_T_RCD + DR_T_CAC + DR_T_PACKET;
                        worst_case++;
                      }
                  }
              }
            else /* WAR */
              {
                if (corrected_overlap > DR_T_RAS + DR_T_RP - 2 * DR_T_PACKET - (DR_T_CAC - DR_T_CWD))
                  {
                    lat = - DR_T_RAS + DR_T_RCD + DR_T_PACKET + DR_T_CAC;
                    best_case++;
                    full_overlapping++;
                  }
                else if (corrected_overlap > 0)
                  {
                    lat = DR_T_RCD + DR_T_CWD + DR_T_PACKET - corrected_overlap;
                    in_between_case++;
                    partial_overlapping++;
                  }
                else
                  {
                    lat = DR_T_RCD + DR_T_CWD + DR_T_PACKET;
                    worst_case++;
                  }
              }
          }

        lat += chunks * DR_T_PACKET; /*every 128 bit need DR_NUM_CYCLES*/

        /*fprintf(stderr,"%4d ",lat);debugging*/

        if (cpu_ratio < 1.0)
          {
            lat = (lat+((int)(1/cpu_ratio)-1))/(int)(1/cpu_ratio);
          }
        else
          {
            temp = (int)(lat*cpu_ratio);
            lat = (lat*cpu_ratio == temp)?temp:(temp+1); /*round up*/
          }

        lastCmdIsRead=cmdIsRead;
        last_bank=current_bank;
        last_dev = current_device;

        /*fprintf(stderr,"%4d \n",lat);debugging*/

        if (overlap <= 0) /*memory interface is not busy*/
          {
            busy_until[current_bank] = curTick+lat;
          }
        else/*the memory request will be satisfied temp cycles after curTick*/
          {
            busy_until[current_bank] +=lat;
            command_overlapping++;
            lat+=overlap;
          }

        /*fprintf(stderr,"%10.0f %10.0f %4d %4d \n",(double)busy_until, (double)curTick, overlap, lat);*/

        if (cmdIsRead)
          {
            if (cpu_ratio < 1.0)
              {
                busy_until[current_bank] += abs(((DR_T_RAS - (DR_T_RCD + DR_T_PACKET + DR_T_CAC)) + 1)/((int)(1/cpu_ratio))); /* CPU clock cycles */
              }
            else
              {
                busy_until[current_bank] += (int) abs(((DR_T_RAS - (DR_T_RCD + DR_T_PACKET + DR_T_CAC)) + 1)*cpu_ratio); /* CPU clock cycles */
              }
          }
         else /* !cmdIsRead */
          {
            if (cpu_ratio < 1.0)
              {
                busy_until[current_bank] += abs((-DR_T_PACKET + DR_T_RTR + DR_T_RP - DR_T_CWD + 1)/((int)(1/cpu_ratio))); /* CPU clock cycles */
              }
            else
              {
                busy_until[current_bank] += (int) abs((-DR_T_PACKET + DR_T_RTR + DR_T_RP - DR_T_CWD + 1)*cpu_ratio); /* CPU clock cycles */
              }
          }

        // if((_cpu_num < num_cpus) && (_cpu_num >= 0))
                // cout <<"cpu id = " << _cpu_num << "current_bank = " << current_bank << endl;
    //	bank_access_profile[_cpu_num][current_bank]++;
        return lat;
      }

    /******************************************************************/

    else if ((mem_type == "SDRAM") && (mem_actpolicy == "closed") && (memctrlpipe_enable == false))
      /* SDRAM closed-page without overlap, 7/99 MG */
      {
        if (mem_access_output != NULL)
          {
            //fprintf(mem_accessfd,"\n");
          }
        assert(chunks >0);

        if (cmdIsRead)
          {
            lat = act_lat + cas_lat;
          }
        else /* !cmdIsRead */
          {
            lat = act_lat;
          }
        total_access++;
        lat += chunks;

        overlap=(int)(busy_until[current_bank] - curTick);
        lastCmdIsRead=cmdIsRead;

        if (cpu_ratio < 1.0)
          {
            lat = (lat+((int)(1/cpu_ratio)-1))/(int)(1/cpu_ratio);
          }
        else
          {
            temp = (int)(lat*cpu_ratio);
            lat = (lat*cpu_ratio == temp)?temp:(temp+1); /*round up*/
          }

        if (overlap <= 0) /*memory interface is not busy*/
          {
            busy_until[current_bank] = curTick+lat;
          }
        else/*the memory request will be satisfied temp cycles after curTick*/
          {
            busy_until[current_bank] +=lat;
            command_overlapping++;
            lat+=overlap;
          }
        if (!cmdIsRead)
          {
            temp = (int)(((dpl_lat-1) + pre_lat)*cpu_ratio);
            busy_until[current_bank] += (((dpl_lat-1) + pre_lat)*cpu_ratio == temp)?temp:(temp+1);
          }

        // if((_cpu_num < num_cpus) && (_cpu_num >= 0))
                // cout <<"cpu id = " << _cpu_num << "current_bank = " << current_bank << endl;
    //	bank_access_profile[_cpu_num][current_bank]++;
        return lat;
      }

    /******************************************************************/

    else if ((mem_type == "DRDRAM") && (mem_actpolicy == "closed") && (memctrlpipe_enable == false))
      /* DRDRAM closed-page without overlap */
      {
        if (cmdIsRead)
          {
            lat = DR_T_RCD + DR_T_CAC + DR_T_PACKET; /*  DR_T_RP + */
          }
        else /* !cmdIsRead */
          {
            lat = DR_T_RCD + DR_T_CWD + DR_T_PACKET; /* DR_T_RP + */
          }
        total_access++;
        overlap=(int)(busy_until[current_bank] - curTick);
        lat += chunks * DR_T_PACKET; /*every 128 bit need DR_NUM_CYCLES*/

        if (cpu_ratio < 1.0)
          {
            lat = (lat+((int)(1/cpu_ratio)-1))/(int)(1/cpu_ratio);
          }
        else
          {
            temp = (int)(lat*cpu_ratio);
            lat = (lat*cpu_ratio == temp)?temp:(temp+1); /*round up*/
          }

        lastCmdIsRead=cmdIsRead;

        if (overlap <= 0) /*memory interface is not busy*/
          {
            busy_until[current_bank] = curTick+lat;
          }
        else/*the memory request will be satisfied temp cycles after curTick*/
          {
            busy_until[current_bank] += lat;
            command_overlapping++;
            lat+=overlap;
          }

        if (cmdIsRead)
          {
            if (cpu_ratio < 1.0)
              {
                busy_until[current_bank] += abs(((DR_T_RAS - (DR_T_RCD + DR_T_PACKET + DR_T_CAC)) + 1)/((int)(1/cpu_ratio))); /* CPU clock cycles */
              }
            else
              {
                busy_until[current_bank] += (int) abs(((DR_T_RAS - (DR_T_RCD + DR_T_PACKET + DR_T_CAC)) + 1)*cpu_ratio); /* CPU clock cycles */
              }
          }
         else /* !cmdIsRead */
          {
            if (cpu_ratio < 1.0)
              {
                busy_until[current_bank] += abs((-DR_T_PACKET + DR_T_RTR + DR_T_RP - DR_T_CWD + 1)/((int)(1/cpu_ratio))); /* CPU clock cycles */
              }
            else
              {
                busy_until[current_bank] += (int) abs((-DR_T_PACKET + DR_T_RTR + DR_T_RP - DR_T_CWD + 1)*cpu_ratio); /* CPU clock cycles */
              }
          }
        // if((_cpu_num < num_cpus) && (_cpu_num >= 0))
                // cout <<"cpu id = " << _cpu_num << "current_bank = " << current_bank << endl;
    //	bank_access_profile[_cpu_num][current_bank]++;
        return lat;
      }

    /******************************************************************/

    else /*STD*/
      {
        if (mem_access_output != NULL)
          {
            //fprintf(mem_accessfd,"\n");
          }
        assert(chunks >0);
        // if((_cpu_num < num_cpus) && (_cpu_num >= 0))
                // cout <<"cpu id = " << _cpu_num << "current_bank = " << current_bank << endl;
    //	bank_access_profile[_cpu_num][current_bank]++;
        return(/* first chunk latency */act_lat +
               (/* remainder chunk latency */cas_lat * (chunks - 1)));
      }

}

/*end added by ar, MG*/

/*begin added by ar, MG*/

/* ================ helper functions ========================= */

/****** DRDRAM specific: shared sense amplifiers ******/
/* precharges the adjacent banks and returns the number of them (1 or 2)*/
int /*number of precharged banks*/
DRAMMemory::prechargeBanksAround(int bank)/*access to bank */
{
int temp;

temp=bank%DR_BANK_SAMP;

if (temp == 0) /*bank 0, 16,32 ....*/
  {
    if (active_row[bank+1]!=DR_NUM_ROWS)
      {
        precharge++;
        active_row[bank+1]=DR_NUM_ROWS;
      }
  }
else
  {
    if (temp==DR_BANK_SAMP-1)/*banks 15,31 ...*/
      {
        if (active_row[bank-1]!=DR_NUM_ROWS)
          {
            precharge++;
            active_row[bank-1]=DR_NUM_ROWS;
          }
      }
    else
      {
        if (active_row[bank-1]!=DR_NUM_ROWS)
          {
            precharge++;
            active_row[bank-1]=DR_NUM_ROWS;
          }
        if (active_row[bank+1]!=DR_NUM_ROWS)
          {
            precharge++;
            active_row[bank+1]=DR_NUM_ROWS;
          }
      }
  }
return precharge;
}


#ifndef DOXYGEN_SHOULD_SKIP_THIS

BEGIN_DECLARE_SIM_OBJECT_PARAMS(DRAMMemory)

    Param<std::string> file;
    Param<Range<Addr> > range;
    Param<Tick> latency;
    /* additional params for dram protocol*/
    Param<int> cpu_ratio;
    Param<std::string> mem_type;
    Param<std::string> mem_actpolicy;
    Param<std::string> memctrladdr_type;
    Param<int> bus_width;
    Param<int> act_lat;
    Param<int> cas_lat;
    Param<int> war_lat;
    Param<int> pre_lat;
    Param<int> dpl_lat;
    Param<int> trc_lat;
    Param<int> num_banks;
    Param<int> num_cpus;

END_DECLARE_SIM_OBJECT_PARAMS(DRAMMemory)

BEGIN_INIT_SIM_OBJECT_PARAMS(DRAMMemory)

    INIT_PARAM_DFLT(file, "memory mapped file", ""),
    INIT_PARAM(range, "Device Address Range"),
    INIT_PARAM(latency, "Memory access latency"),

    /* additional params for dram protocol*/
    INIT_PARAM_DFLT(cpu_ratio,"ratio between CPU speed and memory bus speed",5),
    INIT_PARAM_DFLT(mem_type,"type of DRAM","SDRAM"),
    INIT_PARAM_DFLT(mem_actpolicy,"open / closed page policy","open"),
    INIT_PARAM_DFLT(memctrladdr_type,"interleaved or direct mapping","interleaved"),
    INIT_PARAM_DFLT(bus_width,"memory access bus width",16),
    INIT_PARAM_DFLT(act_lat,"RAS to CAS delay",2),
    INIT_PARAM_DFLT(cas_lat,"CAS delay",1),
    INIT_PARAM_DFLT(war_lat,"write after read delay",2),
    INIT_PARAM_DFLT(pre_lat,"precharge delay",2),
    INIT_PARAM_DFLT(dpl_lat,"data in to precharge delay",2),
    INIT_PARAM_DFLT(trc_lat,"row cycle delay",6),
    INIT_PARAM_DFLT(num_banks,"Number of Banks",4),
    INIT_PARAM_DFLT(num_cpus,"Number of CPUs connected to DRAM",4)

END_INIT_SIM_OBJECT_PARAMS(DRAMMemory)

CREATE_SIM_OBJECT(DRAMMemory)
{
    DRAMMemory::Params *p = new DRAMMemory::Params;
    p->name = getInstanceName();
    p->addrRange = range;
    p->latency = latency;

    /* additional params for dram */
    p->cpu_ratio = cpu_ratio;
    p->bus_width = bus_width;
    p->mem_type = mem_type;
    p->mem_actpolicy = mem_actpolicy;
    p->memctrladdr_type = memctrladdr_type;
    p->act_lat = act_lat;
    p->cas_lat = cas_lat;
    p->war_lat = war_lat;
    p->pre_lat = pre_lat;
    p->dpl_lat = dpl_lat;
    p->trc_lat = trc_lat;
    p->num_banks = num_banks;
    p->num_cpus = num_cpus;

    return new DRAMMemory(p);
}

REGISTER_SIM_OBJECT("DRAMMemory", DRAMMemory)

#endif // DOXYGEN_SHOULD_SKIP_THIS


