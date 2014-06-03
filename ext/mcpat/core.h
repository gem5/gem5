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


#ifndef CORE_H_
#define CORE_H_

#include "array.h"
#include "basic_components.h"
#include "cacheunit.h"
#include "interconnect.h"
#include "logic.h"
#include "parameter.h"

// Macros used in the various core-related classes
#define NUM_SOURCE_OPERANDS 2
#define NUM_INT_INST_SOURCE_OPERANDS 2

class BranchPredictorParameters {
public:
    int assoc;
    int nbanks;
    int local_l1_predictor_size;
    int local_l2_predictor_size;
    int local_predictor_entries;
    int global_predictor_bits;
    int global_predictor_entries;
    int chooser_predictor_bits;
    int chooser_predictor_entries;
};

class BranchPredictor : public McPATComponent {
public:
    ArrayST* globalBPT;
    ArrayST* localBPT;
    ArrayST* L1_localBPT;
    ArrayST* L2_localBPT;
    ArrayST* chooser;
    ArrayST* RAS;

    InputParameter interface_ip;
    CoreParameters core_params;
    CoreStatistics core_stats;
    BranchPredictorParameters branch_pred_params;
    double scktRatio, chip_PR_overhead, macro_PR_overhead;
    bool exist;

    BranchPredictor(XMLNode* _xml_data, InputParameter* interface_ip_,
                    const CoreParameters & _core_params,
                    const CoreStatistics & _core_stats,
                    bool exsit = true);
    void set_params_stats();
    void computeEnergy();
    void displayData(uint32_t indent = 0, int plevel = 100);
    ~BranchPredictor();
};

class InstFetchParameters {
public:
    int btb_size;
    int btb_block_size;
    int btb_assoc;
    int btb_num_banks;
    int btb_latency;
    int btb_throughput;
    int btb_rw_ports;
};

class InstFetchStatistics {
public:
    double btb_read_accesses;
    double btb_write_accesses;
};

class InstFetchU : public McPATComponent {
public:
    CacheUnit* icache;
    ArrayST* IB;
    ArrayST* BTB;
    BranchPredictor* BPT;
    InstructionDecoder* ID_inst;
    InstructionDecoder* ID_operand;
    InstructionDecoder* ID_misc;

    InputParameter interface_ip;
    CoreParameters core_params;
    CoreStatistics core_stats;
    InstFetchParameters inst_fetch_params;
    InstFetchStatistics inst_fetch_stats;
    double scktRatio, chip_PR_overhead, macro_PR_overhead;
    enum Cache_policy cache_p;
    bool exist;

    InstFetchU(XMLNode* _xml_data, InputParameter* interface_ip_,
               const CoreParameters & _core_params,
               const CoreStatistics & _core_stats,
               bool exsit = true);
    void set_params_stats();
    void computeEnergy();
    void displayData(uint32_t indent = 0, int plevel = 100);
    ~InstFetchU();
};


class SchedulerU : public McPATComponent {
public:
    static int ROB_STATUS_BITS;

    ArrayST* int_inst_window;
    ArrayST* fp_inst_window;
    ArrayST* ROB;
    selection_logic* int_instruction_selection;
    selection_logic* fp_instruction_selection;

    InputParameter interface_ip;
    CoreParameters core_params;
    CoreStatistics core_stats;
    double scktRatio, chip_PR_overhead, macro_PR_overhead;
    double Iw_height, fp_Iw_height, ROB_height;
    bool exist;

    SchedulerU(XMLNode* _xml_data, InputParameter* interface_ip_,
               const CoreParameters & _core_params,
               const CoreStatistics & _core_stats,
               bool exist_ = true);
    void computeEnergy();
    void displayData(uint32_t indent = 0, int plevel = 100);
    ~SchedulerU();
};

class RENAMINGU : public McPATComponent {
public:
    ArrayST* iFRAT;
    ArrayST* fFRAT;
    ArrayST* iRRAT;
    ArrayST* fRRAT;
    ArrayST* ifreeL;
    ArrayST* ffreeL;
    dep_resource_conflict_check* idcl;
    dep_resource_conflict_check* fdcl;
    ArrayST* RAHT;

    InputParameter interface_ip;
    CoreParameters core_params;
    CoreStatistics core_stats;
    bool exist;

    RENAMINGU(XMLNode* _xml_data, InputParameter* interface_ip_,
              const CoreParameters & _core_params,
              const CoreStatistics & _core_stats,
              bool exist_ = true);
    void computeEnergy();
    void displayData(uint32_t indent = 0, int plevel = 100);
    ~RENAMINGU();
};

class LoadStoreU : public McPATComponent {
public:
    CacheUnit* dcache;
    ArrayST* LSQ;
    ArrayST* LoadQ;

    InputParameter interface_ip;
    CoreParameters core_params;
    CoreStatistics core_stats;
    enum Cache_policy cache_p;
    double scktRatio, chip_PR_overhead, macro_PR_overhead;
    double lsq_height;
    bool exist;

    LoadStoreU(XMLNode* _xml_data, InputParameter* interface_ip_,
               const CoreParameters & _core_params,
               const CoreStatistics & _core_stats,
               bool exist_ = true);
    void computeEnergy();
    void displayData(uint32_t indent = 0, int plevel = 100);
    ~LoadStoreU();
};

class MemoryManagementParams {
public:
    int itlb_number_entries;
    double itlb_latency;
    double itlb_throughput;
    int itlb_assoc;
    int itlb_nbanks;
    int dtlb_number_entries;
    double dtlb_latency;
    double dtlb_throughput;
    int dtlb_assoc;
    int dtlb_nbanks;
};

class MemoryManagementStats {
public:
    double itlb_total_accesses;
    double itlb_total_misses;
    double itlb_conflicts;
    double dtlb_read_accesses;
    double dtlb_read_misses;
    double dtlb_write_accesses;
    double dtlb_write_misses;
    double dtlb_conflicts;
};

class MemManU : public McPATComponent {
public:
    ArrayST* itlb;
    ArrayST* dtlb;

    InputParameter interface_ip;
    CoreParameters core_params;
    CoreStatistics core_stats;
    MemoryManagementParams mem_man_params;
    MemoryManagementStats mem_man_stats;
    double scktRatio, chip_PR_overhead, macro_PR_overhead;
    bool exist;

    MemManU(XMLNode* _xml_data, InputParameter* interface_ip_,
            const CoreParameters & _core_params,
            const CoreStatistics & _core_stats, bool exist_ = true);
    void set_params_stats();
    void computeEnergy();
    void displayData(uint32_t indent = 0, int plevel = 100);
    ~MemManU();
};

class RegFU : public McPATComponent {
public:
    static int RFWIN_ACCESS_MULTIPLIER;

    ArrayST* IRF;
    ArrayST* FRF;
    ArrayST* RFWIN;

    InputParameter interface_ip;
    CoreParameters core_params;
    CoreStatistics core_stats;
    double scktRatio, chip_PR_overhead, macro_PR_overhead;
    double int_regfile_height, fp_regfile_height;
    bool exist;

    RegFU(XMLNode* _xml_data,
          InputParameter* interface_ip_, const CoreParameters & _core_params,
          const CoreStatistics & _core_stats,
          bool exist_ = true);
    void computeEnergy();
    void displayData(uint32_t indent = 0, int plevel = 100);
    ~RegFU();
};

class EXECU : public McPATComponent {
public:
    RegFU* rfu;
    SchedulerU* scheu;
    FunctionalUnit* fp_u;
    FunctionalUnit* exeu;
    FunctionalUnit* mul;
    Interconnect* int_bypass;
    Interconnect* intTagBypass;
    Interconnect* int_mul_bypass;
    Interconnect* intTag_mul_Bypass;
    Interconnect* fp_bypass;
    Interconnect* fpTagBypass;

    InputParameter interface_ip;
    double scktRatio, chip_PR_overhead, macro_PR_overhead;
    double lsq_height;
    CoreParameters core_params;
    CoreStatistics core_stats;
    bool exist;

    EXECU(XMLNode* _xml_data, InputParameter* interface_ip_,
          double lsq_height_, const CoreParameters & _core_params,
          const CoreStatistics & _core_stats, bool exist_ = true);
    void computeEnergy();
    void displayData(uint32_t indent = 0, int plevel = 100);
    ~EXECU();
};


class Core : public McPATComponent {
public:
    InstFetchU* ifu;
    LoadStoreU* lsu;
    MemManU* mmu;
    EXECU* exu;
    RENAMINGU* rnu;
    Pipeline* corepipe;
    UndiffCore* undiffCore;
    CacheUnit* l2cache;

    int ithCore;
    InputParameter interface_ip;
    double scktRatio, chip_PR_overhead, macro_PR_overhead;
    CoreParameters core_params;
    CoreStatistics core_stats;

    // TODO: Migrate component ID handling into the XML data to remove this
    //       ithCore variable
    Core(XMLNode* _xml_data, int _ithCore, InputParameter* interface_ip_);
    void initialize_params();
    void initialize_stats();
    void set_core_param();
    void computeEnergy();
    ~Core();
};

#endif /* CORE_H_ */
