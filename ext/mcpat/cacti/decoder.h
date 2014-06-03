/*****************************************************************************
 *                                McPAT/CACTI
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


#ifndef __DECODER_H__
#define __DECODER_H__

#include <vector>

#include "area.h"
#include "component.h"
#include "parameter.h"

using namespace std;


class Decoder : public Component {
public:
    Decoder(
        int _num_dec_signals,
        bool flag_way_select,
        double _C_ld_dec_out,
        double _R_wire_dec_out,
        bool fully_assoc_,
        bool is_dram_,
        bool is_wl_tr_,
        const Area & cell_);

    bool   exist;
    int    num_in_signals;
    double C_ld_dec_out;
    double R_wire_dec_out;
    int    num_gates;
    int    num_gates_min;
    double w_dec_n[MAX_NUMBER_GATES_STAGE];
    double w_dec_p[MAX_NUMBER_GATES_STAGE];
    double delay;
    //powerDef power;
    bool   fully_assoc;
    bool   is_dram;
    bool   is_wl_tr;
    const  Area & cell;


    void   compute_widths();
    void   compute_area();
    double compute_delays(double inrisetime);  // return outrisetime

    void leakage_feedback(double temperature);
};



class PredecBlk : public Component {
public:
    PredecBlk(
        int num_dec_signals,
        Decoder * dec,
        double C_wire_predec_blk_out,
        double R_wire_predec_blk_out,
        int    num_dec_per_predec,
        bool   is_dram_,
        bool   is_blk1);

    Decoder * dec;
    bool exist;
    int number_input_addr_bits;
    double C_ld_predec_blk_out;
    double R_wire_predec_blk_out;
    int branch_effort_nand2_gate_output;
    int branch_effort_nand3_gate_output;
    bool   flag_two_unique_paths;
    int flag_L2_gate;
    int number_inputs_L1_gate;
    int number_gates_L1_nand2_path;
    int number_gates_L1_nand3_path;
    int number_gates_L2;
    int min_number_gates_L1;
    int min_number_gates_L2;
    int num_L1_active_nand2_path;
    int num_L1_active_nand3_path;
    double w_L1_nand2_n[MAX_NUMBER_GATES_STAGE];
    double w_L1_nand2_p[MAX_NUMBER_GATES_STAGE];
    double w_L1_nand3_n[MAX_NUMBER_GATES_STAGE];
    double w_L1_nand3_p[MAX_NUMBER_GATES_STAGE];
    double w_L2_n[MAX_NUMBER_GATES_STAGE];
    double w_L2_p[MAX_NUMBER_GATES_STAGE];
    double delay_nand2_path;
    double delay_nand3_path;
    powerDef power_nand2_path;
    powerDef power_nand3_path;
    powerDef power_L2;

    bool is_dram_;

    void compute_widths();
    void compute_area();

    void leakage_feedback(double temperature);

    pair<double, double> compute_delays(pair<double, double> inrisetime); // <nand2, nand3>
    // return <outrise_nand2, outrise_nand3>
};


class PredecBlkDrv : public Component {
public:
    PredecBlkDrv(
        int   way_select,
        PredecBlk * blk_,
        bool  is_dram);

    int flag_driver_exists;
    int number_input_addr_bits;
    int number_gates_nand2_path;
    int number_gates_nand3_path;
    int min_number_gates;
    int num_buffers_driving_1_nand2_load;
    int num_buffers_driving_2_nand2_load;
    int num_buffers_driving_4_nand2_load;
    int num_buffers_driving_2_nand3_load;
    int num_buffers_driving_8_nand3_load;
    int num_buffers_nand3_path;
    double c_load_nand2_path_out;
    double c_load_nand3_path_out;
    double r_load_nand2_path_out;
    double r_load_nand3_path_out;
    double width_nand2_path_n[MAX_NUMBER_GATES_STAGE];
    double width_nand2_path_p[MAX_NUMBER_GATES_STAGE];
    double width_nand3_path_n[MAX_NUMBER_GATES_STAGE];
    double width_nand3_path_p[MAX_NUMBER_GATES_STAGE];
    double delay_nand2_path;
    double delay_nand3_path;
    powerDef power_nand2_path;
    powerDef power_nand3_path;

    PredecBlk * blk;
    Decoder   * dec;
    bool  is_dram_;
    int   way_select;

    void compute_widths();
    void compute_area();

    void leakage_feedback(double temperature);


    pair<double, double> compute_delays(
        double inrisetime_nand2_path,
        double inrisetime_nand3_path);  // return <outrise_nand2, outrise_nand3>

    inline int num_addr_bits_nand2_path() {
        return num_buffers_driving_1_nand2_load +
               num_buffers_driving_2_nand2_load +
               num_buffers_driving_4_nand2_load;
    }
    inline int num_addr_bits_nand3_path() {
        return num_buffers_driving_2_nand3_load +
               num_buffers_driving_8_nand3_load;
    }
    double get_rdOp_dynamic_E(int num_act_mats_hor_dir);
};



class Predec : public Component {
public:
    Predec(
        PredecBlkDrv * drv1,
        PredecBlkDrv * drv2);

    double compute_delays(double inrisetime);  // return outrisetime

    void leakage_feedback(double temperature);
    PredecBlk    * blk1;
    PredecBlk    * blk2;
    PredecBlkDrv * drv1;
    PredecBlkDrv * drv2;

    powerDef block_power;
    powerDef driver_power;

private:
    // returns <delay, risetime>
    pair<double, double> get_max_delay_before_decoder(
        pair<double, double> input_pair1,
        pair<double, double> input_pair2);
};



class Driver : public Component {
public:
    Driver(double c_gate_load_, double c_wire_load_, double r_wire_load_, bool is_dram);

    int    number_gates;
    int    min_number_gates;
    double width_n[MAX_NUMBER_GATES_STAGE];
    double width_p[MAX_NUMBER_GATES_STAGE];
    double c_gate_load;
    double c_wire_load;
    double r_wire_load;
    double delay;
    powerDef power;
    bool   is_dram_;

    void   compute_widths();
    double compute_delay(double inrisetime);
};


#endif
