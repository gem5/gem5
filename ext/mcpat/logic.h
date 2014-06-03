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
#ifndef LOGIC_H_
#define LOGIC_H_

#include <cmath>
#include <cstring>
#include <iostream>

#include "arch_const.h"
#include "basic_circuit.h"
#include "basic_components.h"
#include "cacti_interface.h"
#include "component.h"
#include "const.h"
#include "decoder.h"
#include "parameter.h"
#include "xmlParser.h"

using namespace std;

class selection_logic : public McPATComponent {
public:
    bool is_default;
    InputParameter l_ip;
    uca_org_t local_result;
    int win_entries;
    int issue_width;
    double accesses;
    int num_threads;
    enum Device_ty device_ty;
    enum Core_type core_ty;

    selection_logic(XMLNode* _xml_data, bool _is_default, int _win_entries,
                    int issue_width_, const InputParameter* configure_interface,
                    string _name, double _accesses,
                    double clockRate_ = 0.0f,
                    enum Device_ty device_ty_ = Core_device,
                    enum Core_type core_ty_ = Inorder);
    void computeArea();
    void computeEnergy();
        void leakage_feedback(double temperature); // TODO
    // TODO: Add a deconstructor
};

class dep_resource_conflict_check : public McPATComponent {
public:
    InputParameter l_ip;
    uca_org_t local_result;
    double WNORn, WNORp, Wevalinvp, Wevalinvn, Wcompn, Wcompp, Wcomppreequ;
    CoreParameters  coredynp;
    int compare_bits;
    bool is_default;
    statsDef stats_t;

    dep_resource_conflict_check(XMLNode* _xml_data, const string _name,
                                const InputParameter *configure_interface,
                                const CoreParameters & dyn_p_, int compare_bits_,
                                double clockRate_ = 0.0f,
                                bool _is_default = true);
    void conflict_check_power();
    double compare_cap();
    void computeEnergy() {};
    ~dep_resource_conflict_check() {
        local_result.cleanup();
    }

        void leakage_feedback(double temperature);
};

class InstructionDecoder: public McPATComponent {
public:
    Decoder* final_dec;
    Predec* pre_dec;

    bool is_default;
    int opcode_length;
    int num_decoders;
    bool x86;
    int num_decoder_segments;
    int num_decoded_signals;
    InputParameter l_ip;
    uca_org_t local_result;
    enum Device_ty device_ty;
    enum Core_type core_ty;
    statsDef stats_t;

    InstructionDecoder(XMLNode* _xml_data, const string _name, bool _is_default,
                       const InputParameter *configure_interface,
                       int opcode_length_, int num_decoders_, bool x86_,
                       double clockRate_ = 0.0f,
                       enum Device_ty device_ty_ = Core_device,
                       enum Core_type core_ty_ = Inorder);
    InstructionDecoder();
    void computeEnergy() {};
    void inst_decoder_delay_power();
    ~InstructionDecoder();
        void leakage_feedback(double temperature);
};

// TODO: This should be defined elsewhere? This isn't a true McPATComponent
class DFFCell : public Component {
public:
    InputParameter l_ip;
    bool is_dram;
    double cell_load;
    double WdecNANDn;
    double WdecNANDp;
    double clock_cap;
    int    model;
    int    n_switch;
    int    n_keep_1;
    int    n_keep_0;
    int    n_clock;
    powerDef e_switch;
    powerDef e_keep_1;
    powerDef e_keep_0;
    powerDef e_clock;

    DFFCell(bool _is_dram, double _WdecNANDn, double _WdecNANDp, double _cell_load,
            const InputParameter *configure_interface);
    double fpfp_node_cap(unsigned int fan_in, unsigned int fan_out);
    void compute_DFF_cell(void);
    ~DFFCell() {};
};

// TODO: This is a very ambiguous component. Try to refactor it.
class Pipeline : public McPATComponent {
public:
    InputParameter l_ip;
    uca_org_t local_result;
    CoreParameters coredynp;
    enum Device_ty device_ty;
    bool is_core_pipeline, is_default;
    double num_piperegs;
    bool process_ind;
    double WNANDn;
    double WNANDp;
    double load_per_pipeline_stage;

    Pipeline(XMLNode* _xml_data, const InputParameter *configure_interface,
             const CoreParameters & dyn_p_,
             enum Device_ty device_ty_ = Core_device,
             bool _is_core_pipeline = true, bool _is_default = true);
    void compute_stage_vector();
    /**
     * TODO: compute() completes work that should be completed in computeArea()
     * and computeEnergy() recursively. Consider shifting these calculations
     * around to be consistent with rest of hierarchy
     */
    void compute();
    void computeArea() {};
    // TODO: Move energy computation to this function to unify hierarchy
    void computeEnergy() {};
    ~Pipeline() {
        local_result.cleanup();
    };

};

class FunctionalUnit : public McPATComponent {
public:
    InputParameter interface_ip;
    CoreParameters core_params;
    CoreStatistics core_stats;
    double FU_height;
    double num_fu;
    double energy;
    double base_energy;
    double per_access_energy;
    bool is_default;
    enum FU_type fu_type;
    statsDef stats_t;

    FunctionalUnit(XMLNode* _xml_data, InputParameter* interface_ip_,
                   const CoreParameters & _core_params,
                   const CoreStatistics & _core_stats, enum FU_type fu_type);
    void computeEnergy();
    void leakage_feedback(double temperature);
    ~FunctionalUnit() {};
};

// TODO: This is a very ambiguous component. Try to refactor it.
class UndiffCore : public McPATComponent {
public:
    InputParameter interface_ip;
    CoreParameters coredynp;
    double scktRatio;
    double chip_PR_overhead;
    double macro_PR_overhead;
    enum Core_type core_ty;
    bool opt_performance;
    bool embedded;
    double pipeline_stage;
    double num_hthreads;
    double issue_width;
    bool is_default;
    bool exist;

    UndiffCore(XMLNode* _xml_data, InputParameter* interface_ip_,
               const CoreParameters & dyn_p_,
               bool exist_ = true);
    void computeArea() {};
    // TODO: Move energy computation to this function to unify hierarchy
    void computeEnergy() {};
    ~UndiffCore() {};
};
#endif /* LOGIC_H_ */
