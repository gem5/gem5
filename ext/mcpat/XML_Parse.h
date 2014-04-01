/*****************************************************************************
 *                                McPAT
 *                      SOFTWARE LICENSE AGREEMENT
 *            Copyright 2012 Hewlett-Packard Development Company, L.P.
 *                          All Rights Reserved
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
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.”
 *
 ***************************************************************************/

#ifndef XML_PARSE_H_
#define XML_PARSE_H_


//#ifdef WIN32
//#define _CRT_SECURE_NO_DEPRECATE
//#endif

#include <stdio.h>
#include <string.h>

#include <iostream>

#include "xmlParser.h"
using namespace std;

/*
void myfree(char *t); // {free(t);}
ToXMLStringTool tx,tx2;
*/
//all subnodes at the level of system.core(0-n)
//cache_policy is added into cache property arrays;//0 no write or write-though with non-write allocate;1 write-back with write-allocate

typedef struct{
        int prediction_width;
        char prediction_scheme[20];
        int predictor_size;
        int predictor_entries;
        int local_predictor_size[20];
        int local_predictor_entries;
        int global_predictor_entries;
        int global_predictor_bits;
        int chooser_predictor_entries;
        int chooser_predictor_bits;
        double predictor_accesses;
} predictor_systemcore;
typedef struct{
        int number_entries;
        int cache_policy;//0 no write or write-though with non-write allocate;1 write-back with write-allocate
        double total_hits;
        double total_accesses;
        double total_misses;
        double conflicts;
} itlb_systemcore;
typedef struct{
        //params
        double icache_config[20];
        int buffer_sizes[20];
        int cache_policy;//0 no write or write-though with non-write allocate;1 write-back with write-allocate
        //stats
        double total_accesses;
        double read_accesses;
        double read_misses;
        double replacements;
        double read_hits;
        double total_hits;
        double total_misses;
        double miss_buffer_access;
        double fill_buffer_accesses;
        double prefetch_buffer_accesses;
        double prefetch_buffer_writes;
        double prefetch_buffer_reads;
        double prefetch_buffer_hits;
        double conflicts;
} icache_systemcore;
typedef struct{
        //params
        int number_entries;
        int cache_policy;//0 no write or write-though with non-write allocate;1 write-back with write-allocate
        //stats
        double total_accesses;
        double read_accesses;
        double write_accesses;
        double write_hits;
        double read_hits;
        double read_misses;
        double write_misses;
        double total_hits;
        double total_misses;
        double conflicts;
} dtlb_systemcore;
typedef struct{
        //params
        double dcache_config[20];
        int buffer_sizes[20];
        int cache_policy;//0 no write or write-though with non-write allocate;1 write-back with write-allocate
        //stats
        double total_accesses;
        double read_accesses;
        double write_accesses;
        double total_hits;
        double total_misses;
        double read_hits;
        double write_hits;
        double read_misses;
        double write_misses;
        double replacements;
        double write_backs;
        double miss_buffer_access;
        double fill_buffer_accesses;
        double prefetch_buffer_accesses;
        double prefetch_buffer_writes;
        double prefetch_buffer_reads;
        double prefetch_buffer_hits;
        double wbb_writes;
        double wbb_reads;
        double conflicts;
} dcache_systemcore;
typedef struct{
        //params
        int BTB_config[20];
        //stats
        double total_accesses;
        double read_accesses;
        double write_accesses;
        double total_hits;
        double total_misses;
        double read_hits;
        double write_hits;
        double read_misses;
        double write_misses;
        double replacements;
} BTB_systemcore;
typedef struct{
        //all params at the level of system.core(0-n)
        int clock_rate;
        bool opt_local;
        bool x86;
        int machine_bits;
        int virtual_address_width;
        int physical_address_width;
        int opcode_width;
        int micro_opcode_width;
        int instruction_length;
        int machine_type;
        int internal_datapath_width;
        int number_hardware_threads;
        int fetch_width;
        int number_instruction_fetch_ports;
        int decode_width;
        int issue_width;
        int peak_issue_width;
        int commit_width;
        int pipelines_per_core[20];
        int pipeline_depth[20];
        char FPU[20];
        char divider_multiplier[20];
        int ALU_per_core;
        double FPU_per_core;
        int MUL_per_core;
        int instruction_buffer_size;
        int decoded_stream_buffer_size;
        int instruction_window_scheme;
        int instruction_window_size;
        int fp_instruction_window_size;
        int ROB_size;
        int archi_Regs_IRF_size;
        int archi_Regs_FRF_size;
        int phy_Regs_IRF_size;
        int phy_Regs_FRF_size;
        int rename_scheme;
        int register_windows_size;
        char LSU_order[20];
        int store_buffer_size;
        int load_buffer_size;
        int memory_ports;
        char Dcache_dual_pump[20];
        int RAS_size;
        int fp_issue_width;
        int prediction_width;
        int number_of_BTB;
        int number_of_BPT;

        //all stats at the level of system.core(0-n)
        double total_instructions;
        double int_instructions;
        double fp_instructions;
        double branch_instructions;
        double branch_mispredictions;
        double committed_instructions;
        double committed_int_instructions;
        double committed_fp_instructions;
        double load_instructions;
        double store_instructions;
        double total_cycles;
        double idle_cycles;
        double busy_cycles;
        double instruction_buffer_reads;
        double instruction_buffer_write;
        double ROB_reads;
        double ROB_writes;
        double rename_accesses;
        double fp_rename_accesses;
        double rename_reads;
        double rename_writes;
        double fp_rename_reads;
        double fp_rename_writes;
        double inst_window_reads;
        double inst_window_writes;
        double inst_window_wakeup_accesses;
        double inst_window_selections;
        double fp_inst_window_reads;
        double fp_inst_window_writes;
        double fp_inst_window_wakeup_accesses;
        double fp_inst_window_selections;
        double archi_int_regfile_reads;
        double archi_float_regfile_reads;
        double phy_int_regfile_reads;
        double phy_float_regfile_reads;
        double phy_int_regfile_writes;
        double phy_float_regfile_writes;
        double archi_int_regfile_writes;
        double archi_float_regfile_writes;
        double int_regfile_reads;
        double float_regfile_reads;
        double int_regfile_writes;
        double float_regfile_writes;
        double windowed_reg_accesses;
        double windowed_reg_transports;
        double function_calls;
        double context_switches;
        double ialu_accesses;
        double fpu_accesses;
        double mul_accesses;
        double cdb_alu_accesses;
        double cdb_mul_accesses;
        double cdb_fpu_accesses;
        double load_buffer_reads;
        double load_buffer_writes;
        double load_buffer_cams;
        double store_buffer_reads;
        double store_buffer_writes;
        double store_buffer_cams;
        double store_buffer_forwards;
        double main_memory_access;
        double main_memory_read;
        double main_memory_write;
        double pipeline_duty_cycle;

        double IFU_duty_cycle ;
        double BR_duty_cycle ;
        double LSU_duty_cycle ;
        double MemManU_I_duty_cycle;
        double MemManU_D_duty_cycle ;
        double ALU_duty_cycle ;
        double MUL_duty_cycle ;
        double FPU_duty_cycle ;
        double ALU_cdb_duty_cycle ;
        double MUL_cdb_duty_cycle ;
        double FPU_cdb_duty_cycle ;

        //all subnodes at the level of system.core(0-n)
        predictor_systemcore predictor;
        itlb_systemcore itlb;
        icache_systemcore icache;
        dtlb_systemcore dtlb;
        dcache_systemcore dcache;
        BTB_systemcore BTB;

} system_core;
typedef struct{
        //params
        int Directory_type;
        double Dir_config[20];
        int buffer_sizes[20];
        int clockrate;
        int ports[20];
        int device_type;
        int cache_policy;//0 no write or write-though with non-write allocate;1 write-back with write-allocate
        char threeD_stack[20];
        //stats
        double total_accesses;
        double read_accesses;
        double write_accesses;
        double read_misses;
        double write_misses;
        double conflicts;
        double duty_cycle;
} system_L1Directory;
typedef struct{
        //params
        int Directory_type;
        double Dir_config[20];
        int buffer_sizes[20];
        int clockrate;
        int ports[20];
        int device_type;
        int cache_policy;//0 no write or write-though with non-write allocate;1 write-back with write-allocate
        char threeD_stack[20];
        //stats
        double total_accesses;
        double read_accesses;
        double write_accesses;
        double read_misses;
        double write_misses;
        double conflicts;
        double duty_cycle;
} system_L2Directory;
typedef struct{
        //params
        double L2_config[20];
        int clockrate;
        int ports[20];
        int device_type;
        int cache_policy;//0 no write or write-though with non-write allocate;1 write-back with write-allocate
        char threeD_stack[20];
        int buffer_sizes[20];
        //stats
        double total_accesses;
        double read_accesses;
        double write_accesses;
        double total_hits;
        double total_misses;
        double read_hits;
        double write_hits;
        double read_misses;
        double write_misses;
        double replacements;
        double write_backs;
        double miss_buffer_accesses;
        double fill_buffer_accesses;
        double prefetch_buffer_accesses;
        double prefetch_buffer_writes;
        double prefetch_buffer_reads;
        double prefetch_buffer_hits;
        double wbb_writes;
        double wbb_reads;
        double conflicts;
        double duty_cycle;

        bool   merged_dir;
        double homenode_read_accesses;
        double homenode_write_accesses;
        double homenode_read_hits;
        double homenode_write_hits;
        double homenode_read_misses;
        double homenode_write_misses;
        double dir_duty_cycle;
} system_L2;
typedef struct{
        //params
        double L3_config[20];
        int clockrate;
        int ports[20];
        int device_type;
        int cache_policy;//0 no write or write-though with non-write allocate;1 write-back with write-allocate
        char threeD_stack[20];
        int buffer_sizes[20];
        //stats
        double total_accesses;
        double read_accesses;
        double write_accesses;
        double total_hits;
        double total_misses;
        double read_hits;
        double write_hits;
        double read_misses;
        double write_misses;
        double replacements;
        double write_backs;
        double miss_buffer_accesses;
        double fill_buffer_accesses;
        double prefetch_buffer_accesses;
        double prefetch_buffer_writes;
        double prefetch_buffer_reads;
        double prefetch_buffer_hits;
        double wbb_writes;
        double wbb_reads;
        double conflicts;
        double duty_cycle;

        bool   merged_dir;
        double homenode_read_accesses;
        double homenode_write_accesses;
        double homenode_read_hits;
        double homenode_write_hits;
        double homenode_read_misses;
        double homenode_write_misses;
        double dir_duty_cycle;
} system_L3;
typedef struct{
        //params
        int number_of_inputs_of_crossbars;
        int number_of_outputs_of_crossbars;
        int flit_bits;
        int input_buffer_entries_per_port;
        int ports_of_input_buffer[20];
        //stats
        double crossbar_accesses;
} xbar0_systemNoC;
typedef struct{
        //params
        int clockrate;
        bool type;
        bool has_global_link;
        char topology[20];
        int horizontal_nodes;
        int vertical_nodes;
        int link_throughput;
        int link_latency;
        int input_ports;
        int output_ports;
        int virtual_channel_per_port;
        int flit_bits;
        int input_buffer_entries_per_vc;
        int ports_of_input_buffer[20];
        int dual_pump;
        int number_of_crossbars;
        char crossbar_type[20];
        char crosspoint_type[20];
        xbar0_systemNoC xbar0;
        int arbiter_type;
        double chip_coverage;
        //stats
        double total_accesses;
        double duty_cycle;
        double route_over_perc;
} system_NoC;
typedef struct{
        //params
        int mem_tech_node;
        int device_clock;
        int peak_transfer_rate;
        int internal_prefetch_of_DRAM_chip;
        int capacity_per_channel;
        int number_ranks;
        int num_banks_of_DRAM_chip;
        int Block_width_of_DRAM_chip;
        int output_width_of_DRAM_chip;
        int page_size_of_DRAM_chip;
        int burstlength_of_DRAM_chip;
        //stats
        double memory_accesses;
        double memory_reads;
        double memory_writes;
} system_mem;
typedef struct{
        //params
    //Common Param for mc and fc
        double peak_transfer_rate;
        int number_mcs;
        bool withPHY;
        int type;

        //FCParam
        //stats
        double duty_cycle;
        double total_load_perc;

        //McParam
        int mc_clock;
    int llc_line_length;
        int memory_channels_per_mc;
        int number_ranks;
        int req_window_size_per_channel;
        int IO_buffer_size_per_channel;
        int databus_width;
        int addressbus_width;
        bool LVDS;

        //stats
        double memory_accesses;
        double memory_reads;
        double memory_writes;
} system_mc;

typedef struct{
        //params
    int clockrate;
        int number_units;
        int type;
        //stats
        double duty_cycle;
        double total_load_perc;
} system_niu;

typedef struct{
        //params
    int clockrate;
        int number_units;
        int num_channels;
        int type;
        bool withPHY;
        //stats
        double duty_cycle;
        double total_load_perc;
} system_pcie;

typedef struct{
        //All number_of_* at the level of 'system' Ying 03/21/2009
        int number_of_cores;
        int number_of_L1Directories;
        int number_of_L2Directories;
        int number_of_L2s;
        bool Private_L2;
        int number_of_L3s;
        int number_of_NoCs;
        int number_of_dir_levels;
    int domain_size;
    int first_level_dir;
        // All params at the level of 'system'
        int homogeneous_cores;
        int homogeneous_L1Directories;
        int homogeneous_L2Directories;
        double core_tech_node;
        int target_core_clockrate;
        int target_chip_area;
        int temperature;
        int number_cache_levels;
        int L1_property;
        int L2_property;
        int homogeneous_L2s;
        int L3_property;
        int homogeneous_L3s;
        int homogeneous_NoCs;
        int homogeneous_ccs;
        int Max_area_deviation;
        int Max_power_deviation;
        int device_type;
        bool longer_channel_device;
        bool Embedded;
        bool opt_dynamic_power;
        bool opt_lakage_power;
        bool opt_clockrate;
        bool opt_area;
        int interconnect_projection_type;
        int machine_bits;
        int virtual_address_width;
        int physical_address_width;
        int virtual_memory_page_size;
    double total_cycles;
        //system.core(0-n):3rd level
        system_core core[64];
        system_L1Directory L1Directory[64];
        system_L2Directory L2Directory[64];
        system_L2 L2[64];
        system_L3 L3[64];
    system_NoC NoC[64];
    system_mem mem;
        system_mc mc;
        system_mc flashc;
        system_niu niu;
        system_pcie pcie;
} root_system;

class ParseXML
{
public:
        void parse(char* filepath);
    void initialize();
public:
        root_system sys;
};


#endif /* XML_PARSE_H_ */




