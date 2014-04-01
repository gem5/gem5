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

#ifndef BASIC_COMPONENTS_H_
#define BASIC_COMPONENTS_H_

#include <vector>

#include "XML_Parse.h"
#include "parameter.h"

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

enum cache_level {
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

class statsComponents
{
  public:
    double access;
    double hit;
    double miss;

    statsComponents() : access(0), hit(0), miss(0)  {}
    statsComponents(const statsComponents & obj) { *this = obj; }
    statsComponents & operator=(const statsComponents & rhs)
    {
      access = rhs.access;
      hit = rhs.hit;
      miss  = rhs.miss;
      return *this;
    }
    void reset() { access = 0; hit = 0; miss = 0;}

    friend statsComponents operator+(const statsComponents & x, const statsComponents & y);
    friend statsComponents operator*(const statsComponents & x, double const * const y);
};

class statsDef
{
  public:
    statsComponents readAc;
    statsComponents writeAc;
    statsComponents searchAc;

    statsDef() : readAc(), writeAc(),searchAc() { }
    void reset() { readAc.reset(); writeAc.reset();searchAc.reset();}

    friend statsDef operator+(const statsDef & x, const statsDef & y);
    friend statsDef operator*(const statsDef & x, double const * const y);
};

double longer_channel_device_reduction(
                enum Device_ty device_ty=Core_device,
                enum Core_type core_ty=Inorder);

class CoreDynParam {
public:
        CoreDynParam(){};
        CoreDynParam(ParseXML *XML_interface, int ithCore_);
        //    :XML(XML_interface),
        //     ithCore(ithCore_)
        //     core_ty(inorder),
        //     rm_ty(CAMbased),
        //     scheu_ty(PhysicalRegFile),
        //     clockRate(1e9),//1GHz
        //     arch_ireg_width(32),
        //     arch_freg_width(32),
        //     phy_ireg_width(128),
        //     phy_freg_width(128),
        //     perThreadState(8),
        //     globalCheckpoint(32),
        //     instructionLength(32){};
        //ParseXML * XML;
        bool opt_local;
        bool x86;
        bool Embedded;
    enum Core_type  core_ty;
        enum Renaming_type rm_ty;
    enum Scheduler_type scheu_ty;
    double clockRate,executionTime;
    int  arch_ireg_width, arch_freg_width, phy_ireg_width, phy_freg_width;
    int  num_IRF_entry, num_FRF_entry, num_ifreelist_entries, num_ffreelist_entries;
    int  fetchW, decodeW,issueW,peak_issueW, commitW,peak_commitW, predictionW, fp_issueW, fp_decodeW;
    int  perThreadState, globalCheckpoint, instruction_length, pc_width, opcode_length, micro_opcode_length;
    int  num_hthreads, pipeline_stages, fp_pipeline_stages, num_pipelines, num_fp_pipelines;
    int  num_alus, num_muls;
    double num_fpus;
    int  int_data_width, fp_data_width,v_address_width, p_address_width;
    double pipeline_duty_cycle, total_cycles, busy_cycles, idle_cycles;
    bool regWindowing,multithreaded;
    double pppm_lkg_multhread[4];
        double IFU_duty_cycle,BR_duty_cycle,LSU_duty_cycle,MemManU_I_duty_cycle,
               MemManU_D_duty_cycle, ALU_duty_cycle,MUL_duty_cycle,
               FPU_duty_cycle, ALU_cdb_duty_cycle,MUL_cdb_duty_cycle,
               FPU_cdb_duty_cycle;
    ~CoreDynParam(){};
};

class CacheDynParam {
public:
        CacheDynParam(){};
        CacheDynParam(ParseXML *XML_interface, int ithCache_);
    string name;
        enum Dir_type    dir_ty;
        double clockRate,executionTime;
    double    capacity, blockW, assoc, nbanks;
    double throughput, latency;
    double duty_cycle, dir_duty_cycle;
    //double duty_cycle;
    int missb_size, fu_size, prefetchb_size, wbb_size;
    ~CacheDynParam(){};
};

class MCParam {
public:
        MCParam(){};
        MCParam(ParseXML *XML_interface, int ithCache_);
    string name;
    double  clockRate,num_mcs, peakDataTransferRate, num_channels;
    //  double mcTEPowerperGhz;
    //	double mcPHYperGbit;
    //	double area;
    int	   llcBlockSize, dataBusWidth, addressBusWidth;
    int    opcodeW;
    int    memAccesses;
    int    memRank;
    int    type;
    double frontend_duty_cycle, duty_cycle, perc_load;
    double executionTime, reads, writes;
    bool   LVDS, withPHY;

    ~MCParam(){};
};

class NoCParam {
public:
        NoCParam(){};
        NoCParam(ParseXML *XML_interface, int ithCache_);
    string name;
    double  clockRate;
    int	   flit_size;
    int    input_ports, output_ports, min_ports, global_linked_ports;
    int    virtual_channel_per_port,input_buffer_entries_per_vc;
    int    horizontal_nodes,vertical_nodes, total_nodes;
    double executionTime, total_access, link_throughput,link_latency,
                   duty_cycle, chip_coverage, route_over_perc;
    bool   has_global_link, type;

    ~NoCParam(){};
};

class ProcParam {
public:
        ProcParam(){};
        ProcParam(ParseXML *XML_interface, int ithCache_);
    string name;
    int  numCore, numL2, numL3, numNOC, numL1Dir, numL2Dir,numMC, numMCChannel;
    bool homoCore, homoL2, homoL3, homoNOC, homoL1Dir, homoL2Dir;

    ~ProcParam(){};
};

class NIUParam {
public:
        NIUParam(){};
        NIUParam(ParseXML *XML_interface, int ithCache_);
    string name;
    double  clockRate;
    int    num_units;
    int    type;
    double duty_cycle, perc_load;
    ~NIUParam(){};
};

class PCIeParam {
public:
        PCIeParam(){};
        PCIeParam(ParseXML *XML_interface, int ithCache_);
    string name;
    double  clockRate;
    int    num_channels, num_units;
    bool   withPHY;
    int    type;
    double duty_cycle, perc_load;
    ~PCIeParam(){};
};
#endif /* BASIC_COMPONENTS_H_ */
