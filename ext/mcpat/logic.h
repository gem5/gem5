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
#ifndef LOGIC_H_
#define LOGIC_H_

#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>

#include "XML_Parse.h"
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

class selection_logic : public Component{
public:
        selection_logic(bool _is_default, int    win_entries_,
                            int  issue_width_, const InputParameter *configure_interface,
                            enum Device_ty device_ty_=Core_device,
                            enum Core_type core_ty_=Inorder);//, const ParseXML *_XML_interface);
        bool is_default;
        InputParameter l_ip;
        uca_org_t local_result;
        const ParseXML *XML_interface;
        int win_entries;
        int issue_width;
        int num_threads;
        enum Device_ty device_ty;
        enum Core_type core_ty;

        void selection_power();
        void leakage_feedback(double temperature); // TODO
};

class dep_resource_conflict_check : public Component{
public:
        dep_resource_conflict_check(const InputParameter *configure_interface, const CoreDynParam & dyn_p_, int compare_bits_, bool _is_default=true);
        InputParameter l_ip;
        uca_org_t local_result;
        double WNORn, WNORp, Wevalinvp, Wevalinvn, Wcompn, Wcompp, Wcomppreequ;
        CoreDynParam  coredynp;
        int compare_bits;
        bool is_default;
        statsDef       tdp_stats;
        statsDef       rtp_stats;
        statsDef       stats_t;
        powerDef       power_t;

        void conflict_check_power();
        double compare_cap();
        ~dep_resource_conflict_check(){
                local_result.cleanup();
        }

        void leakage_feedback(double temperature);
};

class inst_decoder: public Component{
public:
        inst_decoder(bool _is_default, const InputParameter *configure_interface,
                        int opcode_length_,
                        int num_decoders_,
                        bool x86_,
                        enum Device_ty device_ty_=Core_device,
                        enum Core_type core_ty_=Inorder);
        inst_decoder();
        bool is_default;
        int  opcode_length;
        int  num_decoders;
        bool x86;
        int  num_decoder_segments;
        int  num_decoded_signals;
        InputParameter l_ip;
        uca_org_t local_result;
        enum Device_ty device_ty;
        enum Core_type core_ty;

        Decoder * final_dec;
        Predec *  pre_dec;

        statsDef       tdp_stats;
        statsDef       rtp_stats;
        statsDef       stats_t;
        powerDef       power_t;
        void inst_decoder_delay_power();
        ~inst_decoder();
        void leakage_feedback(double temperature);
};

class DFFCell : public Component {
public:
        DFFCell(bool _is_dram, double _WdecNANDn, double _WdecNANDp,double _cell_load,
                          const InputParameter *configure_interface);
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

        double fpfp_node_cap(unsigned int fan_in, unsigned int fan_out);
        void compute_DFF_cell(void);
        };

class Pipeline : public Component{
public:
        Pipeline(const InputParameter *configure_interface, const CoreDynParam & dyn_p_, enum Device_ty device_ty_=Core_device, bool _is_core_pipeline=true, bool _is_default=true);
        InputParameter l_ip;
        uca_org_t local_result;
        CoreDynParam  coredynp;
        enum Device_ty device_ty;
        bool is_core_pipeline, is_default;
        double num_piperegs;
//	int pipeline_stages;
//	int tot_stage_vector, per_stage_vector;
        bool process_ind;
        double WNANDn ;
        double WNANDp;
        double load_per_pipeline_stage;
//	int  Hthread,  num_thread, fetchWidth, decodeWidth, issueWidth, commitWidth, instruction_length;
//	int  PC_width, opcode_length, num_arch_reg_tag, data_width,num_phsical_reg_tag, address_width;
//	bool thread_clock_gated;
//	bool in_order, multithreaded;
        void compute_stage_vector();
        void compute();
        ~Pipeline(){
                local_result.cleanup();
        };

};

//class core_pipeline :public pipeline{
//public:
//	int  Hthread,  num_thread, fetchWidth, decodeWidth, issueWidth, commitWidth, instruction_length;
//	int  PC_width, opcode_length, num_arch_reg_tag, data_width,num_phsical_reg_tag, address_width;
//	bool thread_clock_gated;
//	bool in_order, multithreaded;
//	core_pipeline(bool _is_default, const InputParameter *configure_interface);
//	virtual void compute_stage_vector();
//
//};

class FunctionalUnit :public Component{
public:
        ParseXML *XML;
        int  ithCore;
        InputParameter interface_ip;
        CoreDynParam  coredynp;
        double FU_height;
        double clockRate,executionTime;
        double num_fu;
        double energy, base_energy,per_access_energy, leakage, gate_leakage;
        bool  is_default;
        enum FU_type fu_type;
        statsDef       tdp_stats;
        statsDef       rtp_stats;
        statsDef       stats_t;
        powerDef       power_t;

        FunctionalUnit(ParseXML *XML_interface, int ithCore_, InputParameter* interface_ip_,const CoreDynParam & dyn_p_, enum FU_type fu_type);
    void computeEnergy(bool is_tdp=true);
        void displayEnergy(uint32_t indent = 0,int plevel = 100, bool is_tdp=true);
    void leakage_feedback(double temperature);

};

class UndiffCore :public Component{
public:
        UndiffCore(ParseXML* XML_interface, int ithCore_, InputParameter* interface_ip_, const CoreDynParam & dyn_p_, bool exist_=true, bool embedded_=false);
        ParseXML *XML;
        int  ithCore;
        InputParameter interface_ip;
        CoreDynParam  coredynp;
        double clockRate,executionTime;
        double scktRatio, chip_PR_overhead, macro_PR_overhead;
        enum  Core_type core_ty;
        bool   opt_performance, embedded;
        double pipeline_stage,num_hthreads,issue_width;
        bool   is_default;

    void displayEnergy(uint32_t indent = 0,int plevel = 100, bool is_tdp=true);
        ~UndiffCore(){};
        bool exist;


};
#endif /* LOGIC_H_ */
