/*****************************************************************************
 *                                McPAT
 *                      SOFTWARE LICENSE AGREEMENT
 *            Copyright 2012 Hewlett-Packard Development Company, L.P.
 *            Copyright (c) 2010-2013 Advanced Micro Devices, Inc.
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
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***************************************************************************/

#ifndef BASIC_COMPONENTS_H_
#define BASIC_COMPONENTS_H_

#include <vector>

#include "component.h"
#include "parameter.h"
#include "xmlParser.h"

/**
 * TODO: Since revisions to McPAT aim to make the component hierarchy more
 * modular, many of the parameter and statistics classes/structs included in
 * this file should be moved to the files for their respective components.
 */
const double cdb_overhead = 1.1;

enum FU_type {
    FPU,
    ALU,
    MUL
};

enum Core_type {
    OOO,
    Inorder
};

enum Renaming_type {
    RAMbased,
    CAMbased
};

enum Scheduler_type {
    PhysicalRegFile,
    ReservationStation
};

enum Cache_type {
    DATA_CACHE,
    INSTRUCTION_CACHE,
    MIXED
};

enum CacheLevel {
    L1,
    L2,
    L3,
    L1Directory,
    L2Directory
};

enum MemoryCtrl_type {
    MC,    //memory controller
    FLASHC //flash controller
};

enum Dir_type {
    ST,//shadowed tag
    DC,//directory cache
    SBT,//static bank tag
    NonDir

};

enum Cache_policy {
    Write_through,
    Write_back
};

enum Device_ty {
    Core_device,
    Uncore_device,
    LLC_device
};

enum Access_mode {
    Normal,
    Sequential,
    Fast
};

class statsComponents {
public:
    double access;
    double hit;
    double miss;

    statsComponents() : access(0), hit(0), miss(0)  {}
    statsComponents(const statsComponents & obj) {
        *this = obj;
    }
    statsComponents & operator=(const statsComponents & rhs) {
        access = rhs.access;
        hit = rhs.hit;
        miss  = rhs.miss;
        return *this;
    }
    void reset() {
        access = 0;
        hit = 0;
        miss = 0;
    }

    friend statsComponents operator+(const statsComponents & x,
                                     const statsComponents & y);
    friend statsComponents operator*(const statsComponents & x,
                                     double const * const y);
};

class statsDef {
public:
    statsComponents readAc;
    statsComponents writeAc;
    statsComponents searchAc;
    statsComponents dataReadAc;
    statsComponents dataWriteAc;
    statsComponents tagReadAc;
    statsComponents tagWriteAc;

    statsDef() : readAc(), writeAc(), searchAc() { }
    void reset() {
        readAc.reset();
        writeAc.reset();
        searchAc.reset();
    }

    friend statsDef operator+(const statsDef & x, const statsDef & y);
    friend statsDef operator*(const statsDef & x, double const * const y);
};

/**
 * An object to store the computed data that will be output from McPAT on a
 * per-component-instance basis. Currently, this includes the amount of storage
 * that the component comprises, its chip area, and power and energy
 * calculations.
 */
class McPATOutput {
public:
    // Storage is in bytes (B)
    double storage;
    // Area is in mm^2
    double area;
    // Peak Dynamic Power is in W
    double peak_dynamic_power;
    // Subthreshold Leakage Power is in W
    double subthreshold_leakage_power;
    // Gate Leakage Power is in W
    double gate_leakage_power;
    // Runtime Dynamic Energy is in J
    double runtime_dynamic_energy;

    void reset();

    friend McPATOutput operator+(const McPATOutput &lhs, const McPATOutput &rhs);
    void operator+=(const McPATOutput &rhs);
};

/**
 * A McPATComponent encompasses all the parts that are common to any component
 * for which McPAT may compute and print power, area, and timing data. It
 * includes a pointer to the XML data from which the component gathers its
 * input parameters, it stores the variables that are commonly used in all
 * components, and it maintains the hierarchical structure to recursively
 * compute and print output. This is a base class from which all components
 * should inherit these functionality (possibly through other descended
 * classes.
*/
class McPATComponent : public Component {
public:
    static bool debug;

    // Variables shared across the system by all McPATComponents
    static bool opt_for_clk;
    static int longer_channel_device;
    static double execution_time;
    static int physical_address_width;
    static int virtual_address_width;
    static int virtual_memory_page_size;
    static int data_path_width;

    // Although these two variables are static right now, they need to be
    // modulated on a per-frequency-domain basis eventually.
    static double target_core_clockrate;
    static double total_cycles;

    XMLNode* xml_data;
    InputParameter interface_ip;
    string name;
    // Number of cycles per second (consider changing name)
    double clockRate;
    vector<McPATComponent*> children;
    // The data structure that is printed in displayData
    McPATOutput output_data;
    // Set this to contain the stats to calculate peak dynamic power
    statsDef tdp_stats;
    // Set this to contain the stats to calculate runtime dynamic energy/power
    statsDef rtp_stats;
    // Holds the peak dynamic power calculation
    powerDef power_t;
    // Holds the runtime dynamic power calculation
    powerDef rt_power;

    McPATComponent();
    // Which of these is a better way of doing things?!
    McPATComponent(XMLNode* _xml_data);
    McPATComponent(XMLNode* _xml_data, InputParameter* _interface_ip);
    virtual void recursiveInstantiate();
    virtual void computeArea();
    // This function should probably be pure virtual, but it's too early in
    // the modifying process to know for sure. Note that each component has
    // to calculate it's own power consumption
    virtual void computeEnergy();
    virtual void displayData(uint32_t indent, int plevel);
    ~McPATComponent();

  protected:
    void errorUnspecifiedParam(string param);
    void errorNonPositiveParam(string param);
    void warnUnrecognizedComponent(XMLCSTR component);
    void warnUnrecognizedParam(XMLCSTR param);
    void warnUnrecognizedStat(XMLCSTR stat);
    void warnIncompleteComponentType(XMLCSTR type);
    void warnMissingComponentType(XMLCSTR id);
    void warnMissingParamName(XMLCSTR id);
    void warnMissingStatName(XMLCSTR id);
};

double longer_channel_device_reduction(
    enum Device_ty device_ty = Core_device,
    enum Core_type core_ty = Inorder);

class CoreParameters {
public:
    bool opt_local;
    bool x86;
    bool Embedded;
    enum Core_type core_ty;
    enum Renaming_type rm_ty;
    enum Scheduler_type scheu_ty;
    double clockRate;
    int arch_ireg_width;
    int arch_freg_width;
    int archi_Regs_IRF_size;
    int archi_Regs_FRF_size;
    int phy_ireg_width;
    int phy_freg_width;
    int num_IRF_entry;
    int num_FRF_entry;
    int num_ifreelist_entries;
    int num_ffreelist_entries;
    int fetchW;
    int decodeW;
    int issueW;
    int peak_issueW;
    int commitW;
    int peak_commitW;
    int predictionW;
    int fp_issueW;
    int fp_decodeW;
    int perThreadState;
    int globalCheckpoint;
    int instruction_length;
    int pc_width;
    int opcode_width;
    int micro_opcode_length;
    int num_hthreads;
    int pipeline_stages;
    int fp_pipeline_stages;
    int num_pipelines;
    int num_fp_pipelines;
    int num_alus;
    int num_muls;
    double num_fpus;
    int int_data_width;
    int fp_data_width;
    int v_address_width;
    int p_address_width;
    bool regWindowing;
    bool multithreaded;
    double pppm_lkg_multhread[4];
    int ROB_size;
    int ROB_assoc;
    int ROB_nbanks;
    int ROB_tag_width;
    int scheduler_assoc;
    int scheduler_nbanks;
    int register_window_size;
    double register_window_throughput;
    double register_window_latency;
    int register_window_assoc;
    int register_window_nbanks;
    int register_window_tag_width;
    int register_window_rw_ports;
    int phy_Regs_IRF_size;
    int phy_Regs_IRF_assoc;
    int phy_Regs_IRF_nbanks;
    int phy_Regs_IRF_tag_width;
    int phy_Regs_IRF_rd_ports;
    int phy_Regs_IRF_wr_ports;
    int phy_Regs_FRF_size;
    int phy_Regs_FRF_assoc;
    int phy_Regs_FRF_nbanks;
    int phy_Regs_FRF_tag_width;
    int phy_Regs_FRF_rd_ports;
    int phy_Regs_FRF_wr_ports;
    int front_rat_nbanks;
    int front_rat_rw_ports;
    int retire_rat_nbanks;
    int retire_rat_rw_ports;
    int freelist_nbanks;
    int freelist_rw_ports;
    int memory_ports;
    int load_buffer_size;
    int load_buffer_assoc;
    int load_buffer_nbanks;
    int store_buffer_size;
    int store_buffer_assoc;
    int store_buffer_nbanks;
    int instruction_window_size;
    int fp_instruction_window_size;
    int instruction_buffer_size;
    int instruction_buffer_assoc;
    int instruction_buffer_nbanks;
    int instruction_buffer_tag_width;
    int number_instruction_fetch_ports;
    int RAS_size;
    int execu_int_bypass_ports;
    int execu_mul_bypass_ports;
    int execu_fp_bypass_ports;
    Wire_type execu_bypass_wire_type;
    Wire_type execu_broadcast_wt;
    int execu_wire_mat_type;
    double execu_bypass_base_width;
    double execu_bypass_base_height;
    int execu_bypass_start_wiring_level;
    double execu_bypass_route_over_perc;
    double broadcast_numerator;
};

class CoreStatistics {
public:
    double pipeline_duty_cycle;
    double total_cycles;
    double busy_cycles;
    double idle_cycles;
    double IFU_duty_cycle;
    double BR_duty_cycle;
    double LSU_duty_cycle;
    double MemManU_I_duty_cycle;
    double MemManU_D_duty_cycle;
    double ALU_duty_cycle;
    double MUL_duty_cycle;
    double FPU_duty_cycle;
    double ALU_cdb_duty_cycle;
    double MUL_cdb_duty_cycle;
    double FPU_cdb_duty_cycle;
    double ROB_reads;
    double ROB_writes;
    double total_instructions;
    double int_instructions;
    double fp_instructions;
    double branch_instructions;
    double branch_mispredictions;
    double load_instructions;
    double store_instructions;
    double committed_instructions;
    double committed_int_instructions;
    double committed_fp_instructions;
    double rename_reads;
    double rename_writes;
    double fp_rename_reads;
    double fp_rename_writes;
    double inst_window_reads;
    double inst_window_writes;
    double inst_window_wakeup_accesses;
    double fp_inst_window_reads;
    double fp_inst_window_writes;
    double fp_inst_window_wakeup_accesses;
    double int_regfile_reads;
    double float_regfile_reads;
    double int_regfile_writes;
    double float_regfile_writes;
    double context_switches;
    double ialu_accesses;
    double fpu_accesses;
    double mul_accesses;
    double cdb_alu_accesses;
    double cdb_fpu_accesses;
    double cdb_mul_accesses;
    double function_calls;
};

class MCParameters {
public:
    double clockRate;
    enum MemoryCtrl_type mc_type;
    double num_mcs;
    int num_channels;
    int llcBlockSize;
    int dataBusWidth;
    int databus_width;
    int llc_line_length;
    int req_window_size_per_channel;
    int IO_buffer_size_per_channel;
    int addressbus_width;
    int opcodeW;
    int type;
    bool LVDS;
    bool withPHY;
    int peak_transfer_rate;
    int number_ranks;
    int reorder_buffer_assoc;
    int reorder_buffer_nbanks;
    int read_buffer_assoc;
    int read_buffer_nbanks;
    int read_buffer_tag_width;
    int write_buffer_assoc;
    int write_buffer_nbanks;
    int write_buffer_tag_width;
};

class MCStatistics {
public:
    double duty_cycle;
    double perc_load;
    double reads;
    double writes;
};

class NIUParameters {
  public:
    double clockRate;
    int num_units;
    int type;
};

class NIUStatistics {
  public:
    double duty_cycle;
    double perc_load;
};

class PCIeParameters {
  public:
    double clockRate;
    int num_channels;
    int num_units;
    bool withPHY;
    int type;
};

class PCIeStatistics {
  public:
    double duty_cycle;
    double perc_load;
};
#endif /* BASIC_COMPONENTS_H_ */
