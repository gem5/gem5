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



#include <cassert>
#include <cmath>
#include <iostream>

#include "area.h"
#include "decoder.h"
#include "parameter.h"

using namespace std;


Decoder::Decoder(
    int    _num_dec_signals,
    bool   flag_way_select,
    double _C_ld_dec_out,
    double _R_wire_dec_out,
    bool   fully_assoc_,
    bool   is_dram_,
    bool   is_wl_tr_,
    const  Area & cell_)
        : exist(false),
        C_ld_dec_out(_C_ld_dec_out),
        R_wire_dec_out(_R_wire_dec_out),
        num_gates(0), num_gates_min(2),
        delay(0),
        //power(),
        fully_assoc(fully_assoc_), is_dram(is_dram_),
        is_wl_tr(is_wl_tr_), cell(cell_) {

    for (int i = 0; i < MAX_NUMBER_GATES_STAGE; i++) {
        w_dec_n[i] = 0;
        w_dec_p[i] = 0;
    }

    /*
     * _num_dec_signals is the number of decoded signal as output
     * num_addr_bits_dec is the number of signal to be decoded
     * as the decoders input.
     */
    int num_addr_bits_dec = _log2(_num_dec_signals);

    if (num_addr_bits_dec < 4) {
        if (flag_way_select) {
            exist = true;
            num_in_signals = 2;
        } else {
            num_in_signals = 0;
        }
    } else {
        exist = true;

        if (flag_way_select) {
            num_in_signals = 3;
        } else {
            num_in_signals = 2;
        }
    }

    assert(cell.h > 0);
    assert(cell.w > 0);
    // the height of a row-decoder-driver cell is fixed to be 4 * cell.h;
    //area.h = 4 * cell.h;
    area.h = g_tp.h_dec * cell.h;

    compute_widths();
    compute_area();
}



void Decoder::compute_widths() {
    double F;
    double p_to_n_sz_ratio = pmos_to_nmos_sz_ratio(is_dram, is_wl_tr);
    double gnand2     = (2 + p_to_n_sz_ratio) / (1 + p_to_n_sz_ratio);
    double gnand3     = (3 + p_to_n_sz_ratio) / (1 + p_to_n_sz_ratio);

    if (exist) {
        if (num_in_signals == 2 || fully_assoc) {
            w_dec_n[0] = 2 * g_tp.min_w_nmos_;
            w_dec_p[0] = p_to_n_sz_ratio * g_tp.min_w_nmos_;
            F = gnand2;
        } else {
            w_dec_n[0] = 3 * g_tp.min_w_nmos_;
            w_dec_p[0] = p_to_n_sz_ratio * g_tp.min_w_nmos_;
            F = gnand3;
        }

        F *= C_ld_dec_out / (gate_C(w_dec_n[0], 0, is_dram, false, is_wl_tr) +
                             gate_C(w_dec_p[0], 0, is_dram, false, is_wl_tr));
        num_gates = logical_effort(
                        num_gates_min,
                        num_in_signals == 2 ? gnand2 : gnand3,
                        F,
                        w_dec_n,
                        w_dec_p,
                        C_ld_dec_out,
                        p_to_n_sz_ratio,
                        is_dram,
                        is_wl_tr,
                        g_tp.max_w_nmos_dec);
    }
}



void Decoder::compute_area() {
    double cumulative_area = 0;
    double cumulative_curr = 0;  // cumulative leakage current
    double cumulative_curr_Ig = 0;  // cumulative leakage current

    if (exist) { // First check if this decoder exists
        if (num_in_signals == 2) {
            cumulative_area =
                compute_gate_area(NAND, 2, w_dec_p[0], w_dec_n[0], area.h);
            cumulative_curr =
                cmos_Isub_leakage(w_dec_n[0], w_dec_p[0], 2, nand, is_dram);
            cumulative_curr_Ig =
                cmos_Ig_leakage(w_dec_n[0], w_dec_p[0], 2, nand, is_dram);
        } else if (num_in_signals == 3) {
            cumulative_area =
                compute_gate_area(NAND, 3, w_dec_p[0], w_dec_n[0], area.h);
            cumulative_curr =
                cmos_Isub_leakage(w_dec_n[0], w_dec_p[0], 3, nand, is_dram);;
            cumulative_curr_Ig =
                cmos_Ig_leakage(w_dec_n[0], w_dec_p[0], 3, nand, is_dram);
        }

        for (int i = 1; i < num_gates; i++) {
            cumulative_area +=
                compute_gate_area(INV, 1, w_dec_p[i], w_dec_n[i], area.h);
            cumulative_curr +=
                cmos_Isub_leakage(w_dec_n[i], w_dec_p[i], 1, inv, is_dram);
            cumulative_curr_Ig =
                cmos_Ig_leakage(w_dec_n[i], w_dec_p[i], 1, inv, is_dram);
        }
        power.readOp.leakage = cumulative_curr * g_tp.peri_global.Vdd;
        power.readOp.gate_leakage = cumulative_curr_Ig * g_tp.peri_global.Vdd;

        area.w = (cumulative_area / area.h);
    }
}



double Decoder::compute_delays(double inrisetime) {
    if (exist) {
        double ret_val = 0;  // outrisetime
        int    i;
        double rd, tf, this_delay, c_load, c_intrinsic, Vpp;
        double Vdd = g_tp.peri_global.Vdd;

        if ((is_wl_tr) && (is_dram)) {
            Vpp = g_tp.vpp;
        } else if (is_wl_tr) {
            Vpp = g_tp.sram_cell.Vdd;
        } else {
            Vpp = g_tp.peri_global.Vdd;
        }

        // first check whether a decoder is required at all
        rd = tr_R_on(w_dec_n[0], NCH, num_in_signals, is_dram, false, is_wl_tr);
        c_load = gate_C(w_dec_n[1] + w_dec_p[1], 0.0, is_dram, false, is_wl_tr);
        c_intrinsic = drain_C_(w_dec_p[0], PCH, 1, 1, area.h, is_dram, false, is_wl_tr) * num_in_signals +
                      drain_C_(w_dec_n[0], NCH, num_in_signals, 1, area.h, is_dram, false, is_wl_tr);
        tf = rd * (c_intrinsic + c_load);
        this_delay = horowitz(inrisetime, tf, 0.5, 0.5, RISE);
        delay += this_delay;
        inrisetime = this_delay / (1.0 - 0.5);
        power.readOp.dynamic += (c_load + c_intrinsic) * Vdd * Vdd;

        for (i = 1; i < num_gates - 1; ++i) {
            rd = tr_R_on(w_dec_n[i], NCH, 1, is_dram, false, is_wl_tr);
            c_load = gate_C(w_dec_p[i+1] + w_dec_n[i+1], 0.0, is_dram, false, is_wl_tr);
            c_intrinsic = drain_C_(w_dec_p[i], PCH, 1, 1, area.h, is_dram, false, is_wl_tr) +
                          drain_C_(w_dec_n[i], NCH, 1, 1, area.h, is_dram, false, is_wl_tr);
            tf = rd * (c_intrinsic + c_load);
            this_delay = horowitz(inrisetime, tf, 0.5, 0.5, RISE);
            delay += this_delay;
            inrisetime = this_delay / (1.0 - 0.5);
            power.readOp.dynamic += (c_load + c_intrinsic) * Vdd * Vdd;
        }

        // add delay of final inverter that drives the wordline
        i = num_gates - 1;
        c_load = C_ld_dec_out;
        rd = tr_R_on(w_dec_n[i], NCH, 1, is_dram, false, is_wl_tr);
        c_intrinsic = drain_C_(w_dec_p[i], PCH, 1, 1, area.h, is_dram, false, is_wl_tr) +
                      drain_C_(w_dec_n[i], NCH, 1, 1, area.h, is_dram, false, is_wl_tr);
        tf = rd * (c_intrinsic + c_load) + R_wire_dec_out * c_load / 2;
        this_delay = horowitz(inrisetime, tf, 0.5, 0.5, RISE);
        delay  += this_delay;
        ret_val = this_delay / (1.0 - 0.5);
        power.readOp.dynamic += c_load * Vpp * Vpp + c_intrinsic * Vdd * Vdd;

        return ret_val;
    } else {
        return 0.0;
    }
}

void Decoder::leakage_feedback(double temperature)
{
  double cumulative_curr = 0;  // cumulative leakage current
  double cumulative_curr_Ig = 0;  // cumulative leakage current

  if (exist)
  { // First check if this decoder exists
    if (num_in_signals == 2)
    {
      cumulative_curr = cmos_Isub_leakage(w_dec_n[0], w_dec_p[0], 2, nand,is_dram);
      cumulative_curr_Ig = cmos_Ig_leakage(w_dec_n[0], w_dec_p[0], 2, nand,is_dram);
    }
    else if (num_in_signals == 3)
    {
      cumulative_curr = cmos_Isub_leakage(w_dec_n[0], w_dec_p[0], 3, nand, is_dram);;
      cumulative_curr_Ig = cmos_Ig_leakage(w_dec_n[0], w_dec_p[0], 3, nand, is_dram);
    }

    for (int i = 1; i < num_gates; i++)
    {
      cumulative_curr += cmos_Isub_leakage(w_dec_n[i], w_dec_p[i], 1, inv, is_dram);
      cumulative_curr_Ig = cmos_Ig_leakage(w_dec_n[i], w_dec_p[i], 1, inv, is_dram);
    }

    power.readOp.leakage = cumulative_curr * g_tp.peri_global.Vdd;
    power.readOp.gate_leakage = cumulative_curr_Ig * g_tp.peri_global.Vdd;
  }
}

PredecBlk::PredecBlk(
    int    num_dec_signals,
    Decoder * dec_,
    double C_wire_predec_blk_out,
    double R_wire_predec_blk_out_,
    int    num_dec_per_predec,
    bool   is_dram,
    bool   is_blk1)
    : dec(dec_),
        exist(false),
        number_input_addr_bits(0),
        C_ld_predec_blk_out(0),
        R_wire_predec_blk_out(0),
        branch_effort_nand2_gate_output(1),
        branch_effort_nand3_gate_output(1),
        flag_two_unique_paths(false),
        flag_L2_gate(0),
        number_inputs_L1_gate(0),
        number_gates_L1_nand2_path(0),
        number_gates_L1_nand3_path(0),
        number_gates_L2(0),
        min_number_gates_L1(2),
        min_number_gates_L2(2),
        num_L1_active_nand2_path(0),
        num_L1_active_nand3_path(0),
        delay_nand2_path(0),
        delay_nand3_path(0),
        power_nand2_path(),
        power_nand3_path(),
        power_L2(),
        is_dram_(is_dram) {
    int    branch_effort_predec_out;
    double C_ld_dec_gate;
    int    num_addr_bits_dec = _log2(num_dec_signals);
    int    blk1_num_input_addr_bits = (num_addr_bits_dec + 1) / 2;
    int    blk2_num_input_addr_bits = num_addr_bits_dec - blk1_num_input_addr_bits;

    w_L1_nand2_n[0] = 0;
    w_L1_nand2_p[0] = 0;
    w_L1_nand3_n[0] = 0;
    w_L1_nand3_p[0] = 0;

    if (is_blk1 == true) {
        if (num_addr_bits_dec <= 0) {
            return;
        } else if (num_addr_bits_dec < 4) {
            // Just one predecoder block is required with NAND2 gates. No decoder required.
            // The first level of predecoding directly drives the decoder output load
            exist = true;
            number_input_addr_bits = num_addr_bits_dec;
            R_wire_predec_blk_out = dec->R_wire_dec_out;
            C_ld_predec_blk_out = dec->C_ld_dec_out;
        } else {
            exist = true;
            number_input_addr_bits   = blk1_num_input_addr_bits;
            branch_effort_predec_out = (1 << blk2_num_input_addr_bits);
            C_ld_dec_gate = num_dec_per_predec * gate_C(dec->w_dec_n[0] + dec->w_dec_p[0], 0, is_dram_, false, false);
            R_wire_predec_blk_out = R_wire_predec_blk_out_;
            C_ld_predec_blk_out = branch_effort_predec_out * C_ld_dec_gate + C_wire_predec_blk_out;
        }
    } else {
        if (num_addr_bits_dec >= 4) {
            exist = true;
            number_input_addr_bits   = blk2_num_input_addr_bits;
            branch_effort_predec_out = (1 << blk1_num_input_addr_bits);
            C_ld_dec_gate = num_dec_per_predec * gate_C(dec->w_dec_n[0] + dec->w_dec_p[0], 0, is_dram_, false, false);
            R_wire_predec_blk_out = R_wire_predec_blk_out_;
            C_ld_predec_blk_out = branch_effort_predec_out * C_ld_dec_gate + C_wire_predec_blk_out;
        }
    }

    compute_widths();
    compute_area();
}



void PredecBlk::compute_widths() {
    double F, c_load_nand3_path, c_load_nand2_path;
    double p_to_n_sz_ratio = pmos_to_nmos_sz_ratio(is_dram_);
    double gnand2 = (2 + p_to_n_sz_ratio) / (1 + p_to_n_sz_ratio);
    double gnand3 = (3 + p_to_n_sz_ratio) / (1 + p_to_n_sz_ratio);

    if (exist == false) return;


    switch (number_input_addr_bits) {
    case 1:
        flag_two_unique_paths           = false;
        number_inputs_L1_gate           = 2;
        flag_L2_gate                    = 0;
        break;
    case 2:
        flag_two_unique_paths           = false;
        number_inputs_L1_gate           = 2;
        flag_L2_gate                    = 0;
        break;
    case 3:
        flag_two_unique_paths           = false;
        number_inputs_L1_gate           = 3;
        flag_L2_gate                    = 0;
        break;
    case 4:
        flag_two_unique_paths           = false;
        number_inputs_L1_gate           = 2;
        flag_L2_gate                    = 2;
        branch_effort_nand2_gate_output = 4;
        break;
    case 5:
        flag_two_unique_paths           = true;
        flag_L2_gate                    = 2;
        branch_effort_nand2_gate_output = 8;
        branch_effort_nand3_gate_output = 4;
        break;
    case 6:
        flag_two_unique_paths           = false;
        number_inputs_L1_gate           = 3;
        flag_L2_gate                    = 2;
        branch_effort_nand3_gate_output = 8;
        break;
    case 7:
        flag_two_unique_paths           = true;
        flag_L2_gate                    = 3;
        branch_effort_nand2_gate_output = 32;
        branch_effort_nand3_gate_output = 16;
        break;
    case 8:
        flag_two_unique_paths           = true;
        flag_L2_gate                    = 3;
        branch_effort_nand2_gate_output = 64;
        branch_effort_nand3_gate_output = 32;
        break;
    case 9:
        flag_two_unique_paths           = false;
        number_inputs_L1_gate           = 3;
        flag_L2_gate                    = 3;
        branch_effort_nand3_gate_output = 64;
        break;
    default:
        assert(0);
        break;
    }

    // find the number of gates and sizing in second level of predecoder (if there is a second level)
    if (flag_L2_gate) {
        if (flag_L2_gate == 2) { // 2nd level is a NAND2 gate
            w_L2_n[0] = 2 * g_tp.min_w_nmos_;
            F = gnand2;
        } else { // 2nd level is a NAND3 gate
            w_L2_n[0] = 3 * g_tp.min_w_nmos_;
            F = gnand3;
        }
        w_L2_p[0] = p_to_n_sz_ratio * g_tp.min_w_nmos_;
        F *= C_ld_predec_blk_out / (gate_C(w_L2_n[0], 0, is_dram_) + gate_C(w_L2_p[0], 0, is_dram_));
        number_gates_L2 = logical_effort(
                              min_number_gates_L2,
                              flag_L2_gate == 2 ? gnand2 : gnand3,
                              F,
                              w_L2_n,
                              w_L2_p,
                              C_ld_predec_blk_out,
                              p_to_n_sz_ratio,
                              is_dram_, false,
                              g_tp.max_w_nmos_);

        // Now find the number of gates and widths in first level of predecoder
        if ((flag_two_unique_paths) || (number_inputs_L1_gate == 2)) {
            // Whenever flag_two_unique_paths is true, it means first level of
            // decoder employs
            // both NAND2 and NAND3 gates. Or when number_inputs_L1_gate is 2,
            // it means
            // a NAND2 gate is used in the first level of the predecoder
            c_load_nand2_path = branch_effort_nand2_gate_output *
                                (gate_C(w_L2_n[0], 0, is_dram_) +
                                 gate_C(w_L2_p[0], 0, is_dram_));
            w_L1_nand2_n[0] = 2 * g_tp.min_w_nmos_;
            w_L1_nand2_p[0] = p_to_n_sz_ratio * g_tp.min_w_nmos_;
            F = gnand2 * c_load_nand2_path /
                (gate_C(w_L1_nand2_n[0], 0, is_dram_) +
                 gate_C(w_L1_nand2_p[0], 0, is_dram_));
            number_gates_L1_nand2_path = logical_effort(
                                             min_number_gates_L1,
                                             gnand2,
                                             F,
                                             w_L1_nand2_n,
                                             w_L1_nand2_p,
                                             c_load_nand2_path,
                                             p_to_n_sz_ratio,
                                             is_dram_, false,
                                             g_tp.max_w_nmos_);
        }

        //Now find widths of gates along path in which first gate is a NAND3
        if ((flag_two_unique_paths) || (number_inputs_L1_gate == 3)) { // Whenever flag_two_unique_paths is TRUE, it means first level of decoder employs
            // both NAND2 and NAND3 gates. Or when number_inputs_L1_gate is 3, it means
            // a NAND3 gate is used in the first level of the predecoder
            c_load_nand3_path = branch_effort_nand3_gate_output *
                                (gate_C(w_L2_n[0], 0, is_dram_) +
                                 gate_C(w_L2_p[0], 0, is_dram_));
            w_L1_nand3_n[0] = 3 * g_tp.min_w_nmos_;
            w_L1_nand3_p[0] = p_to_n_sz_ratio * g_tp.min_w_nmos_;
            F = gnand3 * c_load_nand3_path /
                (gate_C(w_L1_nand3_n[0], 0, is_dram_) +
                 gate_C(w_L1_nand3_p[0], 0, is_dram_));
            number_gates_L1_nand3_path = logical_effort(
                                             min_number_gates_L1,
                                             gnand3,
                                             F,
                                             w_L1_nand3_n,
                                             w_L1_nand3_p,
                                             c_load_nand3_path,
                                             p_to_n_sz_ratio,
                                             is_dram_, false,
                                             g_tp.max_w_nmos_);
        }
    } else { // find number of gates and widths in first level of predecoder block when there is no second level
        if (number_inputs_L1_gate == 2) {
            w_L1_nand2_n[0] = 2 * g_tp.min_w_nmos_;
            w_L1_nand2_p[0] = p_to_n_sz_ratio * g_tp.min_w_nmos_;
            F = gnand2 * C_ld_predec_blk_out /
                (gate_C(w_L1_nand2_n[0], 0, is_dram_) +
                 gate_C(w_L1_nand2_p[0], 0, is_dram_));
            number_gates_L1_nand2_path = logical_effort(
                                             min_number_gates_L1,
                                             gnand2,
                                             F,
                                             w_L1_nand2_n,
                                             w_L1_nand2_p,
                                             C_ld_predec_blk_out,
                                             p_to_n_sz_ratio,
                                             is_dram_, false,
                                             g_tp.max_w_nmos_);
        } else if (number_inputs_L1_gate == 3) {
            w_L1_nand3_n[0] = 3 * g_tp.min_w_nmos_;
            w_L1_nand3_p[0] = p_to_n_sz_ratio * g_tp.min_w_nmos_;
            F = gnand3 * C_ld_predec_blk_out /
                (gate_C(w_L1_nand3_n[0], 0, is_dram_) +
                 gate_C(w_L1_nand3_p[0], 0, is_dram_));
            number_gates_L1_nand3_path = logical_effort(
                                             min_number_gates_L1,
                                             gnand3,
                                             F,
                                             w_L1_nand3_n,
                                             w_L1_nand3_p,
                                             C_ld_predec_blk_out,
                                             p_to_n_sz_ratio,
                                             is_dram_, false,
                                             g_tp.max_w_nmos_);
        }
    }
}



void PredecBlk::compute_area() {
    if (exist) { // First check whether a predecoder block is needed
        int num_L1_nand2 = 0;
        int num_L1_nand3 = 0;
        int num_L2 = 0;
        double tot_area_L1_nand3  = 0;
        double leak_L1_nand3      = 0;
        double gate_leak_L1_nand3 = 0;

        double tot_area_L1_nand2  = compute_gate_area(NAND, 2, w_L1_nand2_p[0], w_L1_nand2_n[0], g_tp.cell_h_def);
        double leak_L1_nand2      = cmos_Isub_leakage(w_L1_nand2_n[0], w_L1_nand2_p[0], 2, nand, is_dram_);
        double gate_leak_L1_nand2 = cmos_Ig_leakage(w_L1_nand2_n[0], w_L1_nand2_p[0], 2, nand, is_dram_);
        if (number_inputs_L1_gate != 3) {
            tot_area_L1_nand3 = 0;
            leak_L1_nand3 = 0;
            gate_leak_L1_nand3 = 0;
        } else {
            tot_area_L1_nand3  = compute_gate_area(NAND, 3, w_L1_nand3_p[0], w_L1_nand3_n[0], g_tp.cell_h_def);
            leak_L1_nand3      = cmos_Isub_leakage(w_L1_nand3_n[0], w_L1_nand3_p[0], 3, nand);
            gate_leak_L1_nand3 = cmos_Ig_leakage(w_L1_nand3_n[0], w_L1_nand3_p[0], 3, nand);
        }

        switch (number_input_addr_bits) {
        case 1: //2 NAND2 gates
            num_L1_nand2 = 2;
            num_L2       = 0;
            num_L1_active_nand2_path = 1;
            num_L1_active_nand3_path = 0;
            break;
        case 2: //4 NAND2 gates
            num_L1_nand2 = 4;
            num_L2       = 0;
            num_L1_active_nand2_path = 1;
            num_L1_active_nand3_path = 0;
            break;
        case 3: //8 NAND3 gates
            num_L1_nand3 = 8;
            num_L2       = 0;
            num_L1_active_nand2_path = 0;
            num_L1_active_nand3_path = 1;
            break;
        case 4: //4 + 4 NAND2 gates
            num_L1_nand2 = 8;
            num_L2       = 16;
            num_L1_active_nand2_path = 2;
            num_L1_active_nand3_path = 0;
            break;
        case 5: //4 NAND2 gates, 8 NAND3 gates
            num_L1_nand2 = 4;
            num_L1_nand3 = 8;
            num_L2       = 32;
            num_L1_active_nand2_path = 1;
            num_L1_active_nand3_path = 1;
            break;
        case 6: //8 + 8 NAND3 gates
            num_L1_nand3 = 16;
            num_L2       = 64;
            num_L1_active_nand2_path = 0;
            num_L1_active_nand3_path = 2;
            break;
        case 7: //4 + 4 NAND2 gates, 8 NAND3 gates
            num_L1_nand2 = 8;
            num_L1_nand3 = 8;
            num_L2       = 128;
            num_L1_active_nand2_path = 2;
            num_L1_active_nand3_path = 1;
            break;
        case 8: //4 NAND2 gates, 8 + 8 NAND3 gates
            num_L1_nand2 = 4;
            num_L1_nand3 = 16;
            num_L2       = 256;
            num_L1_active_nand2_path = 2;
            num_L1_active_nand3_path = 2;
            break;
        case 9: //8 + 8 + 8 NAND3 gates
            num_L1_nand3 = 24;
            num_L2       = 512;
            num_L1_active_nand2_path = 0;
            num_L1_active_nand3_path = 3;
            break;
        default:
            break;
        }

        for (int i = 1; i < number_gates_L1_nand2_path; ++i) {
            tot_area_L1_nand2  += compute_gate_area(INV, 1, w_L1_nand2_p[i], w_L1_nand2_n[i], g_tp.cell_h_def);
            leak_L1_nand2      += cmos_Isub_leakage(w_L1_nand2_n[i], w_L1_nand2_p[i], 2, nand, is_dram_);
            gate_leak_L1_nand2 += cmos_Ig_leakage(w_L1_nand2_n[i], w_L1_nand2_p[i], 2, nand, is_dram_);
        }
        tot_area_L1_nand2  *= num_L1_nand2;
        leak_L1_nand2      *= num_L1_nand2;
        gate_leak_L1_nand2 *= num_L1_nand2;

        for (int i = 1; i < number_gates_L1_nand3_path; ++i) {
            tot_area_L1_nand3  += compute_gate_area(INV, 1, w_L1_nand3_p[i], w_L1_nand3_n[i], g_tp.cell_h_def);
            leak_L1_nand3      += cmos_Isub_leakage(w_L1_nand3_n[i], w_L1_nand3_p[i], 3, nand, is_dram_);
            gate_leak_L1_nand3 += cmos_Ig_leakage(w_L1_nand3_n[i], w_L1_nand3_p[i], 3, nand, is_dram_);
        }
        tot_area_L1_nand3  *= num_L1_nand3;
        leak_L1_nand3      *= num_L1_nand3;
        gate_leak_L1_nand3 *= num_L1_nand3;

        double cumulative_area_L1 = tot_area_L1_nand2 + tot_area_L1_nand3;
        double cumulative_area_L2 = 0.0;
        double leakage_L2         = 0.0;
        double gate_leakage_L2    = 0.0;

        if (flag_L2_gate == 2) {
            cumulative_area_L2 = compute_gate_area(NAND, 2, w_L2_p[0], w_L2_n[0], g_tp.cell_h_def);
            leakage_L2         = cmos_Isub_leakage(w_L2_n[0], w_L2_p[0], 2, nand, is_dram_);
            gate_leakage_L2    = cmos_Ig_leakage(w_L2_n[0], w_L2_p[0], 2, nand, is_dram_);
        } else if (flag_L2_gate == 3) {
            cumulative_area_L2 = compute_gate_area(NAND, 3, w_L2_p[0], w_L2_n[0], g_tp.cell_h_def);
            leakage_L2         = cmos_Isub_leakage(w_L2_n[0], w_L2_p[0], 3, nand, is_dram_);
            gate_leakage_L2    = cmos_Ig_leakage(w_L2_n[0], w_L2_p[0], 3, nand, is_dram_);
        }

        for (int i = 1; i < number_gates_L2; ++i) {
            cumulative_area_L2 += compute_gate_area(INV, 1, w_L2_p[i], w_L2_n[i], g_tp.cell_h_def);
            leakage_L2         += cmos_Isub_leakage(w_L2_n[i], w_L2_p[i], 2, inv, is_dram_);
            gate_leakage_L2    += cmos_Ig_leakage(w_L2_n[i], w_L2_p[i], 2, inv, is_dram_);
        }
        cumulative_area_L2 *= num_L2;
        leakage_L2         *= num_L2;
        gate_leakage_L2    *= num_L2;

        power_nand2_path.readOp.leakage = leak_L1_nand2 * g_tp.peri_global.Vdd;
        power_nand3_path.readOp.leakage = leak_L1_nand3 * g_tp.peri_global.Vdd;
        power_L2.readOp.leakage         = leakage_L2    * g_tp.peri_global.Vdd;
        area.set_area(cumulative_area_L1 + cumulative_area_L2);
        power_nand2_path.readOp.gate_leakage = gate_leak_L1_nand2 * g_tp.peri_global.Vdd;
        power_nand3_path.readOp.gate_leakage = gate_leak_L1_nand3 * g_tp.peri_global.Vdd;
        power_L2.readOp.gate_leakage         = gate_leakage_L2    * g_tp.peri_global.Vdd;
    }
}



pair<double, double> PredecBlk::compute_delays(
    pair<double, double> inrisetime) { // <nand2, nand3>
    pair<double, double> ret_val;
    ret_val.first  = 0;  // outrisetime_nand2_path
    ret_val.second = 0;  // outrisetime_nand3_path

    double inrisetime_nand2_path = inrisetime.first;
    double inrisetime_nand3_path = inrisetime.second;
    int    i;
    double rd, c_load, c_intrinsic, tf, this_delay;
    double Vdd = g_tp.peri_global.Vdd;

    // TODO: following delay calculation part can be greatly simplified.
    // first check whether a predecoder block is required
    if (exist) {
        //Find delay in first level of predecoder block
        //First find delay in path
        if ((flag_two_unique_paths) || (number_inputs_L1_gate == 2)) {
            //First gate is a NAND2 gate
            rd = tr_R_on(w_L1_nand2_n[0], NCH, 2, is_dram_);
            c_load = gate_C(w_L1_nand2_n[1] + w_L1_nand2_p[1], 0.0, is_dram_);
            c_intrinsic = 2 * drain_C_(w_L1_nand2_p[0], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
                          drain_C_(w_L1_nand2_n[0], NCH, 2, 1, g_tp.cell_h_def, is_dram_);
            tf = rd * (c_intrinsic + c_load);
            this_delay = horowitz(inrisetime_nand2_path, tf, 0.5, 0.5, RISE);
            delay_nand2_path += this_delay;
            inrisetime_nand2_path = this_delay / (1.0 - 0.5);
            power_nand2_path.readOp.dynamic += (c_load + c_intrinsic) * Vdd * Vdd;

            //Add delays of all but the last inverter in the chain
            for (i = 1; i < number_gates_L1_nand2_path - 1; ++i) {
                rd = tr_R_on(w_L1_nand2_n[i], NCH, 1, is_dram_);
                c_load = gate_C(w_L1_nand2_n[i+1] + w_L1_nand2_p[i+1], 0.0, is_dram_);
                c_intrinsic = drain_C_(w_L1_nand2_p[i], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
                              drain_C_(w_L1_nand2_n[i], NCH, 1, 1, g_tp.cell_h_def, is_dram_);
                tf = rd * (c_intrinsic + c_load);
                this_delay = horowitz(inrisetime_nand2_path, tf, 0.5, 0.5, RISE);
                delay_nand2_path += this_delay;
                inrisetime_nand2_path = this_delay / (1.0 - 0.5);
                power_nand2_path.readOp.dynamic += (c_intrinsic + c_load) * Vdd * Vdd;
            }

            //Add delay of the last inverter
            i = number_gates_L1_nand2_path - 1;
            rd = tr_R_on(w_L1_nand2_n[i], NCH, 1, is_dram_);
            if (flag_L2_gate) {
                c_load = branch_effort_nand2_gate_output *
                    (gate_C(w_L2_n[0], 0, is_dram_) +
                     gate_C(w_L2_p[0], 0, is_dram_));
                c_intrinsic = drain_C_(w_L1_nand2_p[i], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
                              drain_C_(w_L1_nand2_n[i], NCH, 1, 1, g_tp.cell_h_def, is_dram_);
                tf = rd * (c_intrinsic + c_load);
                this_delay = horowitz(inrisetime_nand2_path, tf, 0.5, 0.5, RISE);
                delay_nand2_path += this_delay;
                inrisetime_nand2_path = this_delay / (1.0 - 0.5);
                power_nand2_path.readOp.dynamic += (c_intrinsic + c_load) * Vdd * Vdd;
            } else { //First level directly drives decoder output load
                c_load = C_ld_predec_blk_out;
                c_intrinsic = drain_C_(w_L1_nand2_p[i], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
                              drain_C_(w_L1_nand2_n[i], NCH, 1, 1, g_tp.cell_h_def, is_dram_);
                tf = rd * (c_intrinsic + c_load) + R_wire_predec_blk_out * c_load / 2;
                this_delay = horowitz(inrisetime_nand2_path, tf, 0.5, 0.5, RISE);
                delay_nand2_path += this_delay;
                ret_val.first = this_delay / (1.0 - 0.5);
                power_nand2_path.readOp.dynamic += (c_intrinsic + c_load) * Vdd * Vdd;
            }
        }

        if ((flag_two_unique_paths) || (number_inputs_L1_gate == 3)) {
            //Check if the number of gates in the first level is more than 1.
            //First gate is a NAND3 gate
            rd = tr_R_on(w_L1_nand3_n[0], NCH, 3, is_dram_);
            c_load = gate_C(w_L1_nand3_n[1] + w_L1_nand3_p[1], 0.0, is_dram_);
            c_intrinsic = 3 * drain_C_(w_L1_nand3_p[0], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
                          drain_C_(w_L1_nand3_n[0], NCH, 3, 1, g_tp.cell_h_def, is_dram_);
            tf = rd * (c_intrinsic + c_load);
            this_delay = horowitz(inrisetime_nand3_path, tf, 0.5, 0.5, RISE);
            delay_nand3_path += this_delay;
            inrisetime_nand3_path = this_delay / (1.0 - 0.5);
            power_nand3_path.readOp.dynamic += (c_intrinsic + c_load) * Vdd * Vdd;

            //Add delays of all but the last inverter in the chain
            for (i = 1; i < number_gates_L1_nand3_path - 1; ++i) {
                rd = tr_R_on(w_L1_nand3_n[i], NCH, 1, is_dram_);
                c_load = gate_C(w_L1_nand3_n[i+1] + w_L1_nand3_p[i+1], 0.0, is_dram_);
                c_intrinsic = drain_C_(w_L1_nand3_p[i], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
                              drain_C_(w_L1_nand3_n[i], NCH, 1, 1, g_tp.cell_h_def, is_dram_);
                tf = rd * (c_intrinsic + c_load);
                this_delay = horowitz(inrisetime_nand3_path, tf, 0.5, 0.5, RISE);
                delay_nand3_path += this_delay;
                inrisetime_nand3_path = this_delay / (1.0 - 0.5);
                power_nand3_path.readOp.dynamic += (c_intrinsic + c_load) * Vdd * Vdd;
            }

            //Add delay of the last inverter
            i = number_gates_L1_nand3_path - 1;
            rd = tr_R_on(w_L1_nand3_n[i], NCH, 1, is_dram_);
            if (flag_L2_gate) {
                c_load = branch_effort_nand3_gate_output *
                    (gate_C(w_L2_n[0], 0, is_dram_) + gate_C(w_L2_p[0], 0,
                                                             is_dram_));
                c_intrinsic = drain_C_(w_L1_nand3_p[i], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
                              drain_C_(w_L1_nand3_n[i], NCH, 1, 1, g_tp.cell_h_def, is_dram_);
                tf = rd * (c_intrinsic + c_load);
                this_delay = horowitz(inrisetime_nand3_path, tf, 0.5, 0.5, RISE);
                delay_nand3_path += this_delay;
                inrisetime_nand3_path = this_delay / (1.0 - 0.5);
                power_nand3_path.readOp.dynamic += (c_intrinsic + c_load) * Vdd * Vdd;
            } else { //First level directly drives decoder output load
                c_load = C_ld_predec_blk_out;
                c_intrinsic = drain_C_(w_L1_nand3_p[i], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
                              drain_C_(w_L1_nand3_n[i], NCH, 1, 1, g_tp.cell_h_def, is_dram_);
                tf = rd * (c_intrinsic + c_load) + R_wire_predec_blk_out * c_load / 2;
                this_delay = horowitz(inrisetime_nand3_path, tf, 0.5, 0.5, RISE);
                delay_nand3_path += this_delay;
                ret_val.second = this_delay / (1.0 - 0.5);
                power_nand3_path.readOp.dynamic += (c_intrinsic + c_load) * Vdd * Vdd;
            }
        }

        // Find delay through second level
        if (flag_L2_gate) {
            if (flag_L2_gate == 2) {
                rd = tr_R_on(w_L2_n[0], NCH, 2, is_dram_);
                c_load = gate_C(w_L2_n[1] + w_L2_p[1], 0.0, is_dram_);
                c_intrinsic = 2 * drain_C_(w_L2_p[0], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
                              drain_C_(w_L2_n[0], NCH, 2, 1, g_tp.cell_h_def, is_dram_);
                tf = rd * (c_intrinsic + c_load);
                this_delay = horowitz(inrisetime_nand2_path, tf, 0.5, 0.5, RISE);
                delay_nand2_path += this_delay;
                inrisetime_nand2_path = this_delay / (1.0 - 0.5);
                power_L2.readOp.dynamic += (c_intrinsic + c_load) * Vdd * Vdd;
            } else { // flag_L2_gate = 3
                rd = tr_R_on(w_L2_n[0], NCH, 3, is_dram_);
                c_load = gate_C(w_L2_n[1] + w_L2_p[1], 0.0, is_dram_);
                c_intrinsic = 3 * drain_C_(w_L2_p[0], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
                              drain_C_(w_L2_n[0], NCH, 3, 1, g_tp.cell_h_def, is_dram_);
                tf = rd * (c_intrinsic + c_load);
                this_delay = horowitz(inrisetime_nand3_path, tf, 0.5, 0.5, RISE);
                delay_nand3_path += this_delay;
                inrisetime_nand3_path = this_delay / (1.0 - 0.5);
                power_L2.readOp.dynamic += (c_intrinsic + c_load) * Vdd * Vdd;
            }

            for (i = 1; i < number_gates_L2 - 1; ++i) {
                rd = tr_R_on(w_L2_n[i], NCH, 1, is_dram_);
                c_load = gate_C(w_L2_n[i+1] + w_L2_p[i+1], 0.0, is_dram_);
                c_intrinsic = drain_C_(w_L2_p[i], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
                              drain_C_(w_L2_n[i], NCH, 1, 1, g_tp.cell_h_def, is_dram_);
                tf = rd * (c_intrinsic + c_load);
                this_delay = horowitz(inrisetime_nand2_path, tf, 0.5, 0.5, RISE);
                delay_nand2_path += this_delay;
                inrisetime_nand2_path = this_delay / (1.0 - 0.5);
                this_delay = horowitz(inrisetime_nand3_path, tf, 0.5, 0.5, RISE);
                delay_nand3_path += this_delay;
                inrisetime_nand3_path = this_delay / (1.0 - 0.5);
                power_L2.readOp.dynamic += (c_intrinsic + c_load) * Vdd * Vdd;
            }

            //Add delay of final inverter that drives the wordline decoders
            i = number_gates_L2 - 1;
            c_load = C_ld_predec_blk_out;
            rd = tr_R_on(w_L2_n[i], NCH, 1, is_dram_);
            c_intrinsic = drain_C_(w_L2_p[i], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
                          drain_C_(w_L2_n[i], NCH, 1, 1, g_tp.cell_h_def, is_dram_);
            tf = rd * (c_intrinsic + c_load) + R_wire_predec_blk_out * c_load / 2;
            this_delay = horowitz(inrisetime_nand2_path, tf, 0.5, 0.5, RISE);
            delay_nand2_path += this_delay;
            ret_val.first = this_delay / (1.0 - 0.5);
            this_delay = horowitz(inrisetime_nand3_path, tf, 0.5, 0.5, RISE);
            delay_nand3_path += this_delay;
            ret_val.second = this_delay / (1.0 - 0.5);
            power_L2.readOp.dynamic += (c_intrinsic + c_load) * Vdd * Vdd;
        }
    }

    delay = (ret_val.first > ret_val.second) ? ret_val.first : ret_val.second;
    return ret_val;
}

void PredecBlk::leakage_feedback(double temperature)
{
  if (exist)
  { // First check whether a predecoder block is needed
    int num_L1_nand2 = 0;
    int num_L1_nand3 = 0;
    int num_L2 = 0;
    double leak_L1_nand3      =0;
    double gate_leak_L1_nand3 =0;

    double leak_L1_nand2      = cmos_Isub_leakage(w_L1_nand2_n[0], w_L1_nand2_p[0], 2, nand, is_dram_);
    double gate_leak_L1_nand2 = cmos_Ig_leakage(w_L1_nand2_n[0], w_L1_nand2_p[0], 2, nand, is_dram_);
    if (number_inputs_L1_gate != 3) {
      leak_L1_nand3 = 0;
      gate_leak_L1_nand3 =0;
    }
    else {
      leak_L1_nand3      = cmos_Isub_leakage(w_L1_nand3_n[0], w_L1_nand3_p[0], 3, nand);
      gate_leak_L1_nand3 = cmos_Ig_leakage(w_L1_nand3_n[0], w_L1_nand3_p[0], 3, nand);
    }

    switch (number_input_addr_bits)
    {
      case 1: //2 NAND2 gates
        num_L1_nand2 = 2;
        num_L2       = 0;
        num_L1_active_nand2_path =1;
        num_L1_active_nand3_path =0;
        break;
      case 2: //4 NAND2 gates
        num_L1_nand2 = 4;
        num_L2       = 0;
        num_L1_active_nand2_path =1;
        num_L1_active_nand3_path =0;
        break;
      case 3: //8 NAND3 gates
        num_L1_nand3 = 8;
        num_L2       = 0;
        num_L1_active_nand2_path =0;
        num_L1_active_nand3_path =1;
        break;
      case 4: //4 + 4 NAND2 gates
        num_L1_nand2 = 8;
        num_L2       = 16;
        num_L1_active_nand2_path =2;
        num_L1_active_nand3_path =0;
        break;
      case 5: //4 NAND2 gates, 8 NAND3 gates
        num_L1_nand2 = 4;
        num_L1_nand3 = 8;
        num_L2       = 32;
        num_L1_active_nand2_path =1;
        num_L1_active_nand3_path =1;
        break;
      case 6: //8 + 8 NAND3 gates
        num_L1_nand3 = 16;
        num_L2       = 64;
        num_L1_active_nand2_path =0;
        num_L1_active_nand3_path =2;
        break;
      case 7: //4 + 4 NAND2 gates, 8 NAND3 gates
        num_L1_nand2 = 8;
        num_L1_nand3 = 8;
        num_L2       = 128;
        num_L1_active_nand2_path =2;
        num_L1_active_nand3_path =1;
        break;
      case 8: //4 NAND2 gates, 8 + 8 NAND3 gates
        num_L1_nand2 = 4;
        num_L1_nand3 = 16;
        num_L2       = 256;
        num_L1_active_nand2_path =2;
        num_L1_active_nand3_path =2;
        break;
      case 9: //8 + 8 + 8 NAND3 gates
        num_L1_nand3 = 24;
        num_L2       = 512;
        num_L1_active_nand2_path =0;
        num_L1_active_nand3_path =3;
        break;
      default:
        break;
    }

    for (int i = 1; i < number_gates_L1_nand2_path; ++i)
    {
      leak_L1_nand2      += cmos_Isub_leakage(w_L1_nand2_n[i], w_L1_nand2_p[i], 2, nand, is_dram_);
      gate_leak_L1_nand2 += cmos_Ig_leakage(w_L1_nand2_n[i], w_L1_nand2_p[i], 2, nand, is_dram_);
    }
    leak_L1_nand2      *= num_L1_nand2;
    gate_leak_L1_nand2 *= num_L1_nand2;

    for (int i = 1; i < number_gates_L1_nand3_path; ++i)
    {
      leak_L1_nand3      += cmos_Isub_leakage(w_L1_nand3_n[i], w_L1_nand3_p[i], 3, nand, is_dram_);
      gate_leak_L1_nand3 += cmos_Ig_leakage(w_L1_nand3_n[i], w_L1_nand3_p[i], 3, nand, is_dram_);
    }
    leak_L1_nand3      *= num_L1_nand3;
    gate_leak_L1_nand3 *= num_L1_nand3;

    double leakage_L2         = 0.0;
    double gate_leakage_L2    = 0.0;

    if (flag_L2_gate == 2)
    {
      leakage_L2         = cmos_Isub_leakage(w_L2_n[0], w_L2_p[0], 2, nand, is_dram_);
      gate_leakage_L2    = cmos_Ig_leakage(w_L2_n[0], w_L2_p[0], 2, nand, is_dram_);
    }
    else if (flag_L2_gate == 3)
    {
      leakage_L2         = cmos_Isub_leakage(w_L2_n[0], w_L2_p[0], 3, nand, is_dram_);
      gate_leakage_L2    = cmos_Ig_leakage(w_L2_n[0], w_L2_p[0], 3, nand, is_dram_);
    }

    for (int i = 1; i < number_gates_L2; ++i)
    {
      leakage_L2         += cmos_Isub_leakage(w_L2_n[i], w_L2_p[i], 2, inv, is_dram_);
      gate_leakage_L2    += cmos_Ig_leakage(w_L2_n[i], w_L2_p[i], 2, inv, is_dram_);
    }
    leakage_L2         *= num_L2;
    gate_leakage_L2    *= num_L2;

    power_nand2_path.readOp.leakage = leak_L1_nand2 * g_tp.peri_global.Vdd;
    power_nand3_path.readOp.leakage = leak_L1_nand3 * g_tp.peri_global.Vdd;
    power_L2.readOp.leakage         = leakage_L2    * g_tp.peri_global.Vdd;

    power_nand2_path.readOp.gate_leakage = gate_leak_L1_nand2 * g_tp.peri_global.Vdd;
    power_nand3_path.readOp.gate_leakage = gate_leak_L1_nand3 * g_tp.peri_global.Vdd;
    power_L2.readOp.gate_leakage         = gate_leakage_L2    * g_tp.peri_global.Vdd;
  }
}

PredecBlkDrv::PredecBlkDrv(
    int    way_select_,
    PredecBlk * blk_,
    bool   is_dram)
        : flag_driver_exists(0),
        number_gates_nand2_path(0),
        number_gates_nand3_path(0),
        min_number_gates(2),
        num_buffers_driving_1_nand2_load(0),
        num_buffers_driving_2_nand2_load(0),
        num_buffers_driving_4_nand2_load(0),
        num_buffers_driving_2_nand3_load(0),
        num_buffers_driving_8_nand3_load(0),
        num_buffers_nand3_path(0),
        c_load_nand2_path_out(0),
        c_load_nand3_path_out(0),
        r_load_nand2_path_out(0),
        r_load_nand3_path_out(0),
        delay_nand2_path(0),
        delay_nand3_path(0),
        power_nand2_path(),
        power_nand3_path(),
        blk(blk_), dec(blk->dec),
        is_dram_(is_dram),
        way_select(way_select_) {
    for (int i = 0; i < MAX_NUMBER_GATES_STAGE; i++) {
        width_nand2_path_n[i] = 0;
        width_nand2_path_p[i] = 0;
        width_nand3_path_n[i] = 0;
        width_nand3_path_p[i] = 0;
    }

    number_input_addr_bits = blk->number_input_addr_bits;

    if (way_select > 1) {
        flag_driver_exists     = 1;
        number_input_addr_bits = way_select;
        if (dec->num_in_signals == 2) {
            c_load_nand2_path_out = gate_C(dec->w_dec_n[0] + dec->w_dec_p[0], 0, is_dram_);
            num_buffers_driving_2_nand2_load = number_input_addr_bits;
        } else if (dec->num_in_signals == 3) {
            c_load_nand3_path_out = gate_C(dec->w_dec_n[0] + dec->w_dec_p[0], 0, is_dram_);
            num_buffers_driving_2_nand3_load = number_input_addr_bits;
        }
    } else if (way_select == 0) {
        if (blk->exist) {
            flag_driver_exists = 1;
        }
    }

    compute_widths();
    compute_area();
}



void PredecBlkDrv::compute_widths() {
    // The predecode block driver accepts as input the address bits from the h-tree network. For
    // each addr bit it then generates addr and addrbar as outputs. For now ignore the effect of
    // inversion to generate addrbar and simply treat addrbar as addr.

    double F;
    double p_to_n_sz_ratio = pmos_to_nmos_sz_ratio(is_dram_);

    if (flag_driver_exists) {
        double C_nand2_gate_blk = gate_C(blk->w_L1_nand2_n[0] + blk->w_L1_nand2_p[0], 0, is_dram_);
        double C_nand3_gate_blk = gate_C(blk->w_L1_nand3_n[0] + blk->w_L1_nand3_p[0], 0, is_dram_);

        if (way_select == 0) {
            if (blk->number_input_addr_bits == 1) {
                //2 NAND2 gates
                num_buffers_driving_2_nand2_load = 1;
                c_load_nand2_path_out            = 2 * C_nand2_gate_blk;
            } else if (blk->number_input_addr_bits == 2) {
                //4 NAND2 gates  one 2-4 decoder
                num_buffers_driving_4_nand2_load = 2;
                c_load_nand2_path_out            = 4 * C_nand2_gate_blk;
            } else if (blk->number_input_addr_bits == 3) {
                //8 NAND3 gates  one 3-8 decoder
                num_buffers_driving_8_nand3_load = 3;
                c_load_nand3_path_out            = 8 * C_nand3_gate_blk;
            } else if (blk->number_input_addr_bits == 4) {
                //4 + 4 NAND2 gates two 2-4 decoder
                num_buffers_driving_4_nand2_load = 4;
                c_load_nand2_path_out            = 4 * C_nand2_gate_blk;
            } else if (blk->number_input_addr_bits == 5) {
                //4 NAND2 gates, 8 NAND3 gates one 2-4 decoder and one 3-8
                //decoder
                num_buffers_driving_4_nand2_load = 2;
                num_buffers_driving_8_nand3_load = 3;
                c_load_nand2_path_out            = 4 * C_nand2_gate_blk;
                c_load_nand3_path_out            = 8 * C_nand3_gate_blk;
            } else if (blk->number_input_addr_bits == 6) {
                //8 + 8 NAND3 gates two 3-8 decoder
                num_buffers_driving_8_nand3_load = 6;
                c_load_nand3_path_out            = 8 * C_nand3_gate_blk;
            } else if (blk->number_input_addr_bits == 7) {
                //4 + 4 NAND2 gates, 8 NAND3 gates two 2-4 decoder and one 3-8
                //decoder
                num_buffers_driving_4_nand2_load = 4;
                num_buffers_driving_8_nand3_load = 3;
                c_load_nand2_path_out            = 4 * C_nand2_gate_blk;
                c_load_nand3_path_out            = 8 * C_nand3_gate_blk;
            } else if (blk->number_input_addr_bits == 8) {
                //4 NAND2 gates, 8 + 8 NAND3 gates one 2-4 decoder and two 3-8
                //decoder
                num_buffers_driving_4_nand2_load = 2;
                num_buffers_driving_8_nand3_load = 6;
                c_load_nand2_path_out            = 4 * C_nand2_gate_blk;
                c_load_nand3_path_out            = 8 * C_nand3_gate_blk;
            } else if (blk->number_input_addr_bits == 9) {
                //8 + 8 + 8 NAND3 gates three 3-8 decoder
                num_buffers_driving_8_nand3_load = 9;
                c_load_nand3_path_out            = 8 * C_nand3_gate_blk;
            }
        }

        if ((blk->flag_two_unique_paths) ||
                (blk->number_inputs_L1_gate == 2) ||
                (number_input_addr_bits == 0) ||
                ((way_select) && (dec->num_in_signals == 2))) {
            //this means that way_select is driving NAND2 in decoder.
            width_nand2_path_n[0] = g_tp.min_w_nmos_;
            width_nand2_path_p[0] = p_to_n_sz_ratio * width_nand2_path_n[0];
            F = c_load_nand2_path_out / gate_C(width_nand2_path_n[0] + width_nand2_path_p[0], 0, is_dram_);
            number_gates_nand2_path = logical_effort(
                                          min_number_gates,
                                          1,
                                          F,
                                          width_nand2_path_n,
                                          width_nand2_path_p,
                                          c_load_nand2_path_out,
                                          p_to_n_sz_ratio,
                                          is_dram_, false, g_tp.max_w_nmos_);
        }

        if ((blk->flag_two_unique_paths) ||
                (blk->number_inputs_L1_gate == 3) ||
                ((way_select) && (dec->num_in_signals == 3))) {
            //this means that way_select is driving NAND3 in decoder.
            width_nand3_path_n[0] = g_tp.min_w_nmos_;
            width_nand3_path_p[0] = p_to_n_sz_ratio * width_nand3_path_n[0];
            F = c_load_nand3_path_out / gate_C(width_nand3_path_n[0] + width_nand3_path_p[0], 0, is_dram_);
            number_gates_nand3_path = logical_effort(
                                          min_number_gates,
                                          1,
                                          F,
                                          width_nand3_path_n,
                                          width_nand3_path_p,
                                          c_load_nand3_path_out,
                                          p_to_n_sz_ratio,
                                          is_dram_, false, g_tp.max_w_nmos_);
        }
    }
}



void PredecBlkDrv::compute_area() {
    double area_nand2_path = 0;
    double area_nand3_path = 0;
    double leak_nand2_path = 0;
    double leak_nand3_path = 0;
    double gate_leak_nand2_path = 0;
    double gate_leak_nand3_path = 0;

    if (flag_driver_exists) {
        // first check whether a predecoder block driver is needed
        for (int i = 0; i < number_gates_nand2_path; ++i) {
            area_nand2_path +=
                compute_gate_area(INV, 1, width_nand2_path_p[i],
                                  width_nand2_path_n[i], g_tp.cell_h_def);
            leak_nand2_path +=
                cmos_Isub_leakage(width_nand2_path_n[i], width_nand2_path_p[i],
                                  1, inv, is_dram_);
            gate_leak_nand2_path +=
                cmos_Ig_leakage(width_nand2_path_n[i], width_nand2_path_p[i],
                                1, inv, is_dram_);
        }
        area_nand2_path *= (num_buffers_driving_1_nand2_load +
                            num_buffers_driving_2_nand2_load +
                            num_buffers_driving_4_nand2_load);
        leak_nand2_path *= (num_buffers_driving_1_nand2_load +
                            num_buffers_driving_2_nand2_load +
                            num_buffers_driving_4_nand2_load);
        gate_leak_nand2_path *= (num_buffers_driving_1_nand2_load +
                                 num_buffers_driving_2_nand2_load +
                                 num_buffers_driving_4_nand2_load);

        for (int i = 0; i < number_gates_nand3_path; ++i) {
            area_nand3_path +=
                compute_gate_area(INV, 1, width_nand3_path_p[i],
                                  width_nand3_path_n[i], g_tp.cell_h_def);
            leak_nand3_path +=
                cmos_Isub_leakage(width_nand3_path_n[i], width_nand3_path_p[i],
                                  1, inv, is_dram_);
            gate_leak_nand3_path +=
                cmos_Ig_leakage(width_nand3_path_n[i], width_nand3_path_p[i],
                                1, inv, is_dram_);
        }
        area_nand3_path *= (num_buffers_driving_2_nand3_load + num_buffers_driving_8_nand3_load);
        leak_nand3_path *= (num_buffers_driving_2_nand3_load + num_buffers_driving_8_nand3_load);
        gate_leak_nand3_path *= (num_buffers_driving_2_nand3_load + num_buffers_driving_8_nand3_load);

        power_nand2_path.readOp.leakage = leak_nand2_path * g_tp.peri_global.Vdd;
        power_nand3_path.readOp.leakage = leak_nand3_path * g_tp.peri_global.Vdd;
        power_nand2_path.readOp.gate_leakage = gate_leak_nand2_path * g_tp.peri_global.Vdd;
        power_nand3_path.readOp.gate_leakage = gate_leak_nand3_path * g_tp.peri_global.Vdd;
        area.set_area(area_nand2_path + area_nand3_path);
    }
}



pair<double, double> PredecBlkDrv::compute_delays(
    double inrisetime_nand2_path,
    double inrisetime_nand3_path) {
    pair<double, double> ret_val;
    ret_val.first  = 0;  // outrisetime_nand2_path
    ret_val.second = 0;  // outrisetime_nand3_path
    int i;
    double rd, c_gate_load, c_load, c_intrinsic, tf, this_delay;
    double Vdd = g_tp.peri_global.Vdd;

    if (flag_driver_exists) {
        for (i = 0; i < number_gates_nand2_path - 1; ++i) {
            rd = tr_R_on(width_nand2_path_n[i], NCH, 1, is_dram_);
            c_gate_load = gate_C(width_nand2_path_p[i+1] + width_nand2_path_n[i+1], 0.0, is_dram_);
            c_intrinsic = drain_C_(width_nand2_path_p[i], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
                          drain_C_(width_nand2_path_n[i], NCH, 1, 1, g_tp.cell_h_def, is_dram_);
            tf = rd * (c_intrinsic + c_gate_load);
            this_delay = horowitz(inrisetime_nand2_path, tf, 0.5, 0.5, RISE);
            delay_nand2_path += this_delay;
            inrisetime_nand2_path = this_delay / (1.0 - 0.5);
            power_nand2_path.readOp.dynamic += (c_gate_load + c_intrinsic) * 0.5 * Vdd * Vdd;
        }

        // Final inverter drives the predecoder block or the decoder output load
        if (number_gates_nand2_path != 0) {
            i = number_gates_nand2_path - 1;
            rd = tr_R_on(width_nand2_path_n[i], NCH, 1, is_dram_);
            c_intrinsic = drain_C_(width_nand2_path_p[i], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
                          drain_C_(width_nand2_path_n[i], NCH, 1, 1, g_tp.cell_h_def, is_dram_);
            c_load = c_load_nand2_path_out;
            tf = rd * (c_intrinsic + c_load) + r_load_nand2_path_out * c_load / 2;
            this_delay = horowitz(inrisetime_nand2_path, tf, 0.5, 0.5, RISE);
            delay_nand2_path += this_delay;
            ret_val.first = this_delay / (1.0 - 0.5);
            power_nand2_path.readOp.dynamic += (c_intrinsic + c_load) * 0.5 * Vdd * Vdd;
//      cout<< "c_intrinsic = " << c_intrinsic << "c_load" << c_load <<endl;
        }

        for (i = 0; i < number_gates_nand3_path - 1; ++i) {
            rd = tr_R_on(width_nand3_path_n[i], NCH, 1, is_dram_);
            c_gate_load = gate_C(width_nand3_path_p[i+1] + width_nand3_path_n[i+1], 0.0, is_dram_);
            c_intrinsic = drain_C_(width_nand3_path_p[i], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
                          drain_C_(width_nand3_path_n[i], NCH, 1, 1, g_tp.cell_h_def, is_dram_);
            tf = rd * (c_intrinsic + c_gate_load);
            this_delay = horowitz(inrisetime_nand3_path, tf, 0.5, 0.5, RISE);
            delay_nand3_path += this_delay;
            inrisetime_nand3_path = this_delay / (1.0 - 0.5);
            power_nand3_path.readOp.dynamic += (c_gate_load + c_intrinsic) * 0.5 * Vdd * Vdd;
        }

        // Final inverter drives the predecoder block or the decoder output load
        if (number_gates_nand3_path != 0) {
            i = number_gates_nand3_path - 1;
            rd = tr_R_on(width_nand3_path_n[i], NCH, 1, is_dram_);
            c_intrinsic = drain_C_(width_nand3_path_p[i], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
                          drain_C_(width_nand3_path_n[i], NCH, 1, 1, g_tp.cell_h_def, is_dram_);
            c_load = c_load_nand3_path_out;
            tf = rd * (c_intrinsic + c_load) + r_load_nand3_path_out * c_load / 2;
            this_delay = horowitz(inrisetime_nand3_path, tf, 0.5, 0.5, RISE);
            delay_nand3_path += this_delay;
            ret_val.second = this_delay / (1.0 - 0.5);
            power_nand3_path.readOp.dynamic += (c_intrinsic + c_load) * 0.5 * Vdd * Vdd;
        }
    }
    return ret_val;
}


double PredecBlkDrv::get_rdOp_dynamic_E(int num_act_mats_hor_dir) {
    return (num_addr_bits_nand2_path()*power_nand2_path.readOp.dynamic +
            num_addr_bits_nand3_path()*power_nand3_path.readOp.dynamic) * num_act_mats_hor_dir;
}



Predec::Predec(
    PredecBlkDrv * drv1_,
    PredecBlkDrv * drv2_)
        : blk1(drv1_->blk), blk2(drv2_->blk), drv1(drv1_), drv2(drv2_) {
    driver_power.readOp.leakage = drv1->power_nand2_path.readOp.leakage +
                                  drv1->power_nand3_path.readOp.leakage +
                                  drv2->power_nand2_path.readOp.leakage +
                                  drv2->power_nand3_path.readOp.leakage;
    block_power.readOp.leakage = blk1->power_nand2_path.readOp.leakage +
                                 blk1->power_nand3_path.readOp.leakage +
                                 blk1->power_L2.readOp.leakage +
                                 blk2->power_nand2_path.readOp.leakage +
                                 blk2->power_nand3_path.readOp.leakage +
                                 blk2->power_L2.readOp.leakage;
    power.readOp.leakage = driver_power.readOp.leakage + block_power.readOp.leakage;

    driver_power.readOp.gate_leakage = drv1->power_nand2_path.readOp.gate_leakage +
                                       drv1->power_nand3_path.readOp.gate_leakage +
                                       drv2->power_nand2_path.readOp.gate_leakage +
                                       drv2->power_nand3_path.readOp.gate_leakage;
    block_power.readOp.gate_leakage = blk1->power_nand2_path.readOp.gate_leakage +
                                      blk1->power_nand3_path.readOp.gate_leakage +
                                      blk1->power_L2.readOp.gate_leakage +
                                      blk2->power_nand2_path.readOp.gate_leakage +
                                      blk2->power_nand3_path.readOp.gate_leakage +
                                      blk2->power_L2.readOp.gate_leakage;
    power.readOp.gate_leakage = driver_power.readOp.gate_leakage + block_power.readOp.gate_leakage;
}

void PredecBlkDrv::leakage_feedback(double temperature)
{
  double leak_nand2_path = 0;
  double leak_nand3_path = 0;
  double gate_leak_nand2_path = 0;
  double gate_leak_nand3_path = 0;

  if (flag_driver_exists)
  { // first check whether a predecoder block driver is needed
    for (int i = 0; i < number_gates_nand2_path; ++i)
    {
      leak_nand2_path += cmos_Isub_leakage(width_nand2_path_n[i], width_nand2_path_p[i], 1, inv,is_dram_);
      gate_leak_nand2_path += cmos_Ig_leakage(width_nand2_path_n[i], width_nand2_path_p[i], 1, inv,is_dram_);
    }
    leak_nand2_path *= (num_buffers_driving_1_nand2_load +
                        num_buffers_driving_2_nand2_load +
                        num_buffers_driving_4_nand2_load);
    gate_leak_nand2_path *= (num_buffers_driving_1_nand2_load +
                            num_buffers_driving_2_nand2_load +
                            num_buffers_driving_4_nand2_load);

    for (int i = 0; i < number_gates_nand3_path; ++i)
    {
      leak_nand3_path += cmos_Isub_leakage(width_nand3_path_n[i], width_nand3_path_p[i], 1, inv,is_dram_);
      gate_leak_nand3_path += cmos_Ig_leakage(width_nand3_path_n[i], width_nand3_path_p[i], 1, inv,is_dram_);
    }
    leak_nand3_path *= (num_buffers_driving_2_nand3_load + num_buffers_driving_8_nand3_load);
    gate_leak_nand3_path *= (num_buffers_driving_2_nand3_load + num_buffers_driving_8_nand3_load);

    power_nand2_path.readOp.leakage = leak_nand2_path * g_tp.peri_global.Vdd;
    power_nand3_path.readOp.leakage = leak_nand3_path * g_tp.peri_global.Vdd;
    power_nand2_path.readOp.gate_leakage = gate_leak_nand2_path * g_tp.peri_global.Vdd;
    power_nand3_path.readOp.gate_leakage = gate_leak_nand3_path * g_tp.peri_global.Vdd;
  }
}

double Predec::compute_delays(double inrisetime) {
    // TODO: Jung Ho thinks that predecoder block driver locates between decoder and predecoder block.
    pair<double, double> tmp_pair1, tmp_pair2;
    tmp_pair1 = drv1->compute_delays(inrisetime, inrisetime);
    tmp_pair1 = blk1->compute_delays(tmp_pair1);
    tmp_pair2 = drv2->compute_delays(inrisetime, inrisetime);
    tmp_pair2 = blk2->compute_delays(tmp_pair2);
    tmp_pair1 = get_max_delay_before_decoder(tmp_pair1, tmp_pair2);

    driver_power.readOp.dynamic =
        drv1->num_addr_bits_nand2_path() * drv1->power_nand2_path.readOp.dynamic +
        drv1->num_addr_bits_nand3_path() * drv1->power_nand3_path.readOp.dynamic +
        drv2->num_addr_bits_nand2_path() * drv2->power_nand2_path.readOp.dynamic +
        drv2->num_addr_bits_nand3_path() * drv2->power_nand3_path.readOp.dynamic;

    block_power.readOp.dynamic =
        blk1->power_nand2_path.readOp.dynamic * blk1->num_L1_active_nand2_path +
        blk1->power_nand3_path.readOp.dynamic * blk1->num_L1_active_nand3_path +
        blk1->power_L2.readOp.dynamic +
        blk2->power_nand2_path.readOp.dynamic * blk1->num_L1_active_nand2_path  +
        blk2->power_nand3_path.readOp.dynamic * blk1->num_L1_active_nand3_path +
        blk2->power_L2.readOp.dynamic;

    power.readOp.dynamic = driver_power.readOp.dynamic + block_power.readOp.dynamic;

    delay = tmp_pair1.first;
    return  tmp_pair1.second;
}

void Predec::leakage_feedback(double temperature)
{
  drv1->leakage_feedback(temperature);
  drv2->leakage_feedback(temperature);
  blk1->leakage_feedback(temperature);
  blk2->leakage_feedback(temperature);

  driver_power.readOp.leakage = drv1->power_nand2_path.readOp.leakage +
                                drv1->power_nand3_path.readOp.leakage +
                                drv2->power_nand2_path.readOp.leakage +
                                drv2->power_nand3_path.readOp.leakage;
  block_power.readOp.leakage = blk1->power_nand2_path.readOp.leakage +
                               blk1->power_nand3_path.readOp.leakage +
                               blk1->power_L2.readOp.leakage +
                               blk2->power_nand2_path.readOp.leakage +
                               blk2->power_nand3_path.readOp.leakage +
                               blk2->power_L2.readOp.leakage;
  power.readOp.leakage = driver_power.readOp.leakage + block_power.readOp.leakage;

  driver_power.readOp.gate_leakage = drv1->power_nand2_path.readOp.gate_leakage +
                                  drv1->power_nand3_path.readOp.gate_leakage +
                                  drv2->power_nand2_path.readOp.gate_leakage +
                                  drv2->power_nand3_path.readOp.gate_leakage;
  block_power.readOp.gate_leakage = blk1->power_nand2_path.readOp.gate_leakage +
                                 blk1->power_nand3_path.readOp.gate_leakage +
                                 blk1->power_L2.readOp.gate_leakage +
                                 blk2->power_nand2_path.readOp.gate_leakage +
                                 blk2->power_nand3_path.readOp.gate_leakage +
                                 blk2->power_L2.readOp.gate_leakage;
  power.readOp.gate_leakage = driver_power.readOp.gate_leakage + block_power.readOp.gate_leakage;
}

// returns <delay, risetime>
pair<double, double> Predec::get_max_delay_before_decoder(
    pair<double, double> input_pair1,
    pair<double, double> input_pair2) {
    pair<double, double> ret_val;
    double delay;

    delay = drv1->delay_nand2_path + blk1->delay_nand2_path;
    ret_val.first  = delay;
    ret_val.second = input_pair1.first;
    delay = drv1->delay_nand3_path + blk1->delay_nand3_path;
    if (ret_val.first < delay) {
        ret_val.first  = delay;
        ret_val.second = input_pair1.second;
    }
    delay = drv2->delay_nand2_path + blk2->delay_nand2_path;
    if (ret_val.first < delay) {
        ret_val.first  = delay;
        ret_val.second = input_pair2.first;
    }
    delay = drv2->delay_nand3_path + blk2->delay_nand3_path;
    if (ret_val.first < delay) {
        ret_val.first  = delay;
        ret_val.second = input_pair2.second;
    }

    return ret_val;
}



Driver::Driver(double c_gate_load_, double c_wire_load_, double r_wire_load_,
               bool is_dram)
    : number_gates(0),
      min_number_gates(2),
      c_gate_load(c_gate_load_),
      c_wire_load(c_wire_load_),
      r_wire_load(r_wire_load_),
      delay(0),
      power(),
      is_dram_(is_dram) {
    for (int i = 0; i < MAX_NUMBER_GATES_STAGE; i++) {
        width_n[i] = 0;
        width_p[i] = 0;
    }

    compute_widths();
}


void Driver::compute_widths() {
    double p_to_n_sz_ratio = pmos_to_nmos_sz_ratio(is_dram_);
    double c_load = c_gate_load + c_wire_load;
    width_n[0] = g_tp.min_w_nmos_;
    width_p[0] = p_to_n_sz_ratio * g_tp.min_w_nmos_;

    double F = c_load / gate_C(width_n[0] + width_p[0], 0, is_dram_);
    number_gates = logical_effort(
                       min_number_gates,
                       1,
                       F,
                       width_n,
                       width_p,
                       c_load,
                       p_to_n_sz_ratio,
                       is_dram_, false,
                       g_tp.max_w_nmos_);
}



double Driver::compute_delay(double inrisetime) {
    int    i;
    double rd, c_load, c_intrinsic, tf;
    double this_delay = 0;

    for (i = 0; i < number_gates - 1; ++i) {
        rd = tr_R_on(width_n[i], NCH, 1, is_dram_);
        c_load = gate_C(width_n[i+1] + width_p[i+1], 0.0, is_dram_);
        c_intrinsic = drain_C_(width_p[i], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
                      drain_C_(width_n[i], NCH, 1, 1, g_tp.cell_h_def, is_dram_);
        tf = rd * (c_intrinsic + c_load);
        this_delay = horowitz(inrisetime, tf, 0.5, 0.5, RISE);
        delay += this_delay;
        inrisetime = this_delay / (1.0 - 0.5);
        power.readOp.dynamic += (c_intrinsic + c_load) * g_tp.peri_global.Vdd *
            g_tp.peri_global.Vdd;
        power.readOp.leakage +=
            cmos_Isub_leakage(width_n[i], width_p[i], 1, inv, is_dram_) *
            g_tp.peri_global.Vdd;
        power.readOp.gate_leakage +=
            cmos_Ig_leakage(width_n[i], width_p[i], 1, inv, is_dram_) *
            g_tp.peri_global.Vdd;
    }

    i = number_gates - 1;
    c_load = c_gate_load + c_wire_load;
    rd = tr_R_on(width_n[i], NCH, 1, is_dram_);
    c_intrinsic = drain_C_(width_p[i], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
        drain_C_(width_n[i], NCH, 1, 1, g_tp.cell_h_def, is_dram_);
    tf = rd * (c_intrinsic + c_load) + r_wire_load *
        (c_wire_load / 2 + c_gate_load);
    this_delay = horowitz(inrisetime, tf, 0.5, 0.5, RISE);
    delay += this_delay;
    power.readOp.dynamic += (c_intrinsic + c_load) * g_tp.peri_global.Vdd *
        g_tp.peri_global.Vdd;
    power.readOp.leakage +=
        cmos_Isub_leakage(width_n[i], width_p[i], 1, inv, is_dram_) *
        g_tp.peri_global.Vdd;
    power.readOp.gate_leakage +=
        cmos_Ig_leakage(width_n[i], width_p[i], 1, inv, is_dram_) *
        g_tp.peri_global.Vdd;

    return this_delay / (1.0 - 0.5);
}

