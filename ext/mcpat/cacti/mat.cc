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

#include "mat.h"

Mat::Mat(const DynamicParameter & dyn_p)
    : dp(dyn_p),
      power_subarray_out_drv(),
      delay_fa_tag(0), delay_cam(0),
      delay_before_decoder(0), delay_bitline(0),
      delay_wl_reset(0), delay_bl_restore(0),
      delay_searchline(0), delay_matchchline(0),
      delay_cam_sl_restore(0), delay_cam_ml_reset(0),
      delay_fa_ram_wl(0), delay_hit_miss_reset(0),
      delay_hit_miss(0),
      subarray(dp, dp.fully_assoc),
      power_bitline(), per_bitline_read_energy(0),
      deg_bl_muxing(dp.deg_bl_muxing),
      num_act_mats_hor_dir(dyn_p.num_act_mats_hor_dir),
      delay_writeback(0),
      cell(subarray.cell), cam_cell(subarray.cam_cell),
      is_dram(dyn_p.is_dram),
      pure_cam(dyn_p.pure_cam),
      num_mats(dp.num_mats),
      power_sa(), delay_sa(0),
      leak_power_sense_amps_closed_page_state(0),
      leak_power_sense_amps_open_page_state(0),
      delay_subarray_out_drv(0),
      delay_comparator(0), power_comparator(),
      num_do_b_mat(dyn_p.num_do_b_mat), num_so_b_mat(dyn_p.num_so_b_mat),
      num_subarrays_per_mat(dp.num_subarrays / dp.num_mats),
      num_subarrays_per_row(dp.Ndwl / dp.num_mats_h_dir) {
    assert(num_subarrays_per_mat <= 4);
    assert(num_subarrays_per_row <= 2);
    is_fa = (dp.fully_assoc) ? true : false;
    camFlag = (is_fa || pure_cam);//although cam_cell.w = cell.w for fa, we still differentiate them.

    if (is_fa || pure_cam) {
        num_subarrays_per_row = num_subarrays_per_mat > 2 ?
            num_subarrays_per_mat / 2 : num_subarrays_per_mat;
    }

    if (dp.use_inp_params == 1) {
        RWP  = dp.num_rw_ports;
        ERP  = dp.num_rd_ports;
        EWP  = dp.num_wr_ports;
        SCHP = dp.num_search_ports;
    } else {
        RWP = g_ip->num_rw_ports;
        ERP = g_ip->num_rd_ports;
        EWP = g_ip->num_wr_ports;
        SCHP = g_ip->num_search_ports;

    }

    double number_sa_subarray;

    if (!is_fa && !pure_cam) {
        number_sa_subarray = subarray.num_cols / deg_bl_muxing;
    } else if (is_fa && !pure_cam) {
        number_sa_subarray =  (subarray.num_cols_fa_cam + subarray.num_cols_fa_ram) / deg_bl_muxing;
    }

    else {
        number_sa_subarray =  (subarray.num_cols_fa_cam) / deg_bl_muxing;
    }

    int    num_dec_signals           = subarray.num_rows;
    double C_ld_bit_mux_dec_out      = 0;
    double C_ld_sa_mux_lev_1_dec_out = 0;
    double C_ld_sa_mux_lev_2_dec_out = 0;
    double R_wire_wl_drv_out;

    if (!is_fa && !pure_cam) {
        R_wire_wl_drv_out = subarray.num_cols * cell.w * g_tp.wire_local.R_per_um;
    } else if (is_fa && !pure_cam) {
        R_wire_wl_drv_out = (subarray.num_cols_fa_cam * cam_cell.w + subarray.num_cols_fa_ram * cell.w) * g_tp.wire_local.R_per_um ;
    } else {
        R_wire_wl_drv_out = (subarray.num_cols_fa_cam * cam_cell.w ) * g_tp.wire_local.R_per_um;
    }

    double R_wire_bit_mux_dec_out = num_subarrays_per_row * subarray.num_cols * g_tp.wire_inside_mat.R_per_um * cell.w;//TODO:revisit for FA
    double R_wire_sa_mux_dec_out  = num_subarrays_per_row * subarray.num_cols * g_tp.wire_inside_mat.R_per_um * cell.w;

    if (deg_bl_muxing > 1) {
        C_ld_bit_mux_dec_out =
            (2 * num_subarrays_per_mat * subarray.num_cols / deg_bl_muxing) *
            gate_C(g_tp.w_nmos_b_mux, 0, is_dram) +  // 2 transistor per cell
            num_subarrays_per_row * subarray.num_cols *
            g_tp.wire_inside_mat.C_per_um * cell.get_w();
    }

    if (dp.Ndsam_lev_1 > 1) {
        C_ld_sa_mux_lev_1_dec_out =
            (num_subarrays_per_mat * number_sa_subarray / dp.Ndsam_lev_1) *
            gate_C(g_tp.w_nmos_sa_mux, 0, is_dram) +
            num_subarrays_per_row * subarray.num_cols *
            g_tp.wire_inside_mat.C_per_um * cell.get_w();
    }
    if (dp.Ndsam_lev_2 > 1) {
        C_ld_sa_mux_lev_2_dec_out =
            (num_subarrays_per_mat * number_sa_subarray /
             (dp.Ndsam_lev_1 * dp.Ndsam_lev_2)) *
            gate_C(g_tp.w_nmos_sa_mux, 0, is_dram) +
            num_subarrays_per_row * subarray.num_cols *
            g_tp.wire_inside_mat.C_per_um * cell.get_w();
    }

    if (num_subarrays_per_row >= 2) {
        // wire heads for both right and left side of a mat, so half the resistance
        R_wire_bit_mux_dec_out /= 2.0;
        R_wire_sa_mux_dec_out  /= 2.0;
    }


    row_dec = new Decoder(
        num_dec_signals,
        false,
        subarray.C_wl,
        R_wire_wl_drv_out,
        false/*is_fa*/,
        is_dram,
        true,
        camFlag ? cam_cell : cell);
//  if (is_fa && (!dp.is_tag))
//  {
//    row_dec->exist = true;
//  }
    bit_mux_dec = new Decoder(
        deg_bl_muxing,// This number is 1 for FA or CAM
        false,
        C_ld_bit_mux_dec_out,
        R_wire_bit_mux_dec_out,
        false/*is_fa*/,
        is_dram,
        false,
        camFlag ? cam_cell : cell);
    sa_mux_lev_1_dec = new Decoder(
        dp.deg_senseamp_muxing_non_associativity, // This number is 1 for FA or CAM
        dp.number_way_select_signals_mat ? true : false,//only sa_mux_lev_1_dec needs way select signal
        C_ld_sa_mux_lev_1_dec_out,
        R_wire_sa_mux_dec_out,
        false/*is_fa*/,
        is_dram,
        false,
        camFlag ? cam_cell : cell);
    sa_mux_lev_2_dec = new Decoder(
        dp.Ndsam_lev_2, // This number is 1 for FA or CAM
        false,
        C_ld_sa_mux_lev_2_dec_out,
        R_wire_sa_mux_dec_out,
        false/*is_fa*/,
        is_dram,
        false,
        camFlag ? cam_cell : cell);

    double C_wire_predec_blk_out;
    double R_wire_predec_blk_out;

    if (!is_fa && !pure_cam) {

        C_wire_predec_blk_out  = num_subarrays_per_row * subarray.num_rows * g_tp.wire_inside_mat.C_per_um * cell.h;
        R_wire_predec_blk_out  = num_subarrays_per_row * subarray.num_rows * g_tp.wire_inside_mat.R_per_um * cell.h;

    } else { //for pre-decode block's load is same for both FA and CAM
        C_wire_predec_blk_out  = subarray.num_rows * g_tp.wire_inside_mat.C_per_um * cam_cell.h;
        R_wire_predec_blk_out  = subarray.num_rows * g_tp.wire_inside_mat.R_per_um * cam_cell.h;
    }


    if (is_fa || pure_cam)
        num_dec_signals += _log2(num_subarrays_per_mat);

    PredecBlk * r_predec_blk1 = new PredecBlk(
        num_dec_signals,
        row_dec,
        C_wire_predec_blk_out,
        R_wire_predec_blk_out,
        num_subarrays_per_mat,
        is_dram,
        true);
    PredecBlk * r_predec_blk2 = new PredecBlk(
        num_dec_signals,
        row_dec,
        C_wire_predec_blk_out,
        R_wire_predec_blk_out,
        num_subarrays_per_mat,
        is_dram,
        false);
    PredecBlk * b_mux_predec_blk1 = new PredecBlk(deg_bl_muxing, bit_mux_dec, 0, 0, 1, is_dram, true);
    PredecBlk * b_mux_predec_blk2 = new PredecBlk(deg_bl_muxing, bit_mux_dec, 0, 0, 1, is_dram, false);
    PredecBlk * sa_mux_lev_1_predec_blk1 = new PredecBlk(dyn_p.deg_senseamp_muxing_non_associativity, sa_mux_lev_1_dec, 0, 0, 1, is_dram, true);
    PredecBlk * sa_mux_lev_1_predec_blk2 = new PredecBlk(dyn_p.deg_senseamp_muxing_non_associativity, sa_mux_lev_1_dec, 0, 0, 1, is_dram, false);
    PredecBlk * sa_mux_lev_2_predec_blk1 = new PredecBlk(dp.Ndsam_lev_2, sa_mux_lev_2_dec, 0, 0, 1, is_dram, true);
    PredecBlk * sa_mux_lev_2_predec_blk2 = new PredecBlk(dp.Ndsam_lev_2, sa_mux_lev_2_dec, 0, 0, 1, is_dram, false);
    dummy_way_sel_predec_blk1 = new PredecBlk(1, sa_mux_lev_1_dec, 0, 0, 0, is_dram, true);
    dummy_way_sel_predec_blk2 = new PredecBlk(1, sa_mux_lev_1_dec, 0, 0, 0, is_dram, false);

    PredecBlkDrv * r_predec_blk_drv1 = new PredecBlkDrv(0, r_predec_blk1, is_dram);
    PredecBlkDrv * r_predec_blk_drv2 = new PredecBlkDrv(0, r_predec_blk2, is_dram);
    PredecBlkDrv * b_mux_predec_blk_drv1 = new PredecBlkDrv(0, b_mux_predec_blk1, is_dram);
    PredecBlkDrv * b_mux_predec_blk_drv2 = new PredecBlkDrv(0, b_mux_predec_blk2, is_dram);
    PredecBlkDrv * sa_mux_lev_1_predec_blk_drv1 = new PredecBlkDrv(0, sa_mux_lev_1_predec_blk1, is_dram);
    PredecBlkDrv * sa_mux_lev_1_predec_blk_drv2 = new PredecBlkDrv(0, sa_mux_lev_1_predec_blk2, is_dram);
    PredecBlkDrv * sa_mux_lev_2_predec_blk_drv1 = new PredecBlkDrv(0, sa_mux_lev_2_predec_blk1, is_dram);
    PredecBlkDrv * sa_mux_lev_2_predec_blk_drv2 = new PredecBlkDrv(0, sa_mux_lev_2_predec_blk2, is_dram);
    way_sel_drv1 = new PredecBlkDrv(dyn_p.number_way_select_signals_mat, dummy_way_sel_predec_blk1, is_dram);
    dummy_way_sel_predec_blk_drv2 = new PredecBlkDrv(1, dummy_way_sel_predec_blk2, is_dram);

    r_predec            = new Predec(r_predec_blk_drv1, r_predec_blk_drv2);
    b_mux_predec        = new Predec(b_mux_predec_blk_drv1, b_mux_predec_blk_drv2);
    sa_mux_lev_1_predec = new Predec(sa_mux_lev_1_predec_blk_drv1, sa_mux_lev_1_predec_blk_drv2);
    sa_mux_lev_2_predec = new Predec(sa_mux_lev_2_predec_blk_drv1, sa_mux_lev_2_predec_blk_drv2);

    subarray_out_wire   = new Wire(g_ip->wt, subarray.area.h);//Bug should be subarray.area.w Owen and Sheng

    double driver_c_gate_load;
    double driver_c_wire_load;
    double driver_r_wire_load;

    if (is_fa || pure_cam)

    {   //Although CAM and RAM use different bl pre-charge driver, assuming the precharge p size is the same
        driver_c_gate_load = (subarray.num_cols_fa_cam ) *
            gate_C(2 * g_tp.w_pmos_bl_precharge + g_tp.w_pmos_bl_eq, 0,
                   is_dram, false, false);
        driver_c_wire_load = subarray.num_cols_fa_cam * cam_cell.w *
            g_tp.wire_outside_mat.C_per_um;
        driver_r_wire_load = subarray.num_cols_fa_cam * cam_cell.w *
            g_tp.wire_outside_mat.R_per_um;
        cam_bl_precharge_eq_drv = new Driver(
            driver_c_gate_load,
            driver_c_wire_load,
            driver_r_wire_load,
            is_dram);

        if (!pure_cam) {
            //This is only used for fully asso not pure CAM
            driver_c_gate_load = (subarray.num_cols_fa_ram ) *
                gate_C(2 * g_tp.w_pmos_bl_precharge + g_tp.w_pmos_bl_eq, 0,
                       is_dram, false, false);
            driver_c_wire_load = subarray.num_cols_fa_ram * cell.w *
                g_tp.wire_outside_mat.C_per_um;
            driver_r_wire_load = subarray.num_cols_fa_ram * cell.w *
                g_tp.wire_outside_mat.R_per_um;
            bl_precharge_eq_drv = new Driver(
                driver_c_gate_load,
                driver_c_wire_load,
                driver_r_wire_load,
                is_dram);
        }
    }

    else {
        driver_c_gate_load =  subarray.num_cols * gate_C(2 * g_tp.w_pmos_bl_precharge + g_tp.w_pmos_bl_eq, 0, is_dram, false, false);
        driver_c_wire_load =  subarray.num_cols * cell.w * g_tp.wire_outside_mat.C_per_um;
        driver_r_wire_load =  subarray.num_cols * cell.w * g_tp.wire_outside_mat.R_per_um;
        bl_precharge_eq_drv = new Driver(
            driver_c_gate_load,
            driver_c_wire_load,
            driver_r_wire_load,
            is_dram);
    }
    double area_row_decoder = row_dec->area.get_area() * subarray.num_rows * (RWP + ERP + EWP);
    double w_row_decoder    = area_row_decoder / subarray.area.get_h();

    double h_bit_mux_sense_amp_precharge_sa_mux_write_driver_write_mux =
        compute_bit_mux_sa_precharge_sa_mux_wr_drv_wr_mux_h();

    double h_subarray_out_drv = subarray_out_wire->area.get_area() *
                                (subarray.num_cols / (deg_bl_muxing * dp.Ndsam_lev_1 * dp.Ndsam_lev_2)) / subarray.area.get_w();


    h_subarray_out_drv *= (RWP + ERP + SCHP);

    double h_comparators                = 0.0;
    double w_row_predecode_output_wires = 0.0;
    double h_bit_mux_dec_out_wires      = 0.0;
    double h_senseamp_mux_dec_out_wires = 0.0;

    if ((!is_fa) && (dp.is_tag)) {
        //tagbits = (4 * num_cols_subarray / (deg_bl_muxing * dp.Ndsam_lev_1 * dp.Ndsam_lev_2)) / num_do_b_mat;
        h_comparators  = compute_comparators_height(dp.tagbits, dyn_p.num_do_b_mat, subarray.area.get_w());
        h_comparators *= (RWP + ERP);
    }


    int branch_effort_predec_blk1_out = (1 << r_predec_blk2->number_input_addr_bits);
    int branch_effort_predec_blk2_out = (1 << r_predec_blk1->number_input_addr_bits);
    w_row_predecode_output_wires   = (branch_effort_predec_blk1_out + branch_effort_predec_blk2_out) *
                                     g_tp.wire_inside_mat.pitch * (RWP + ERP + EWP);


    double h_non_cell_area = (num_subarrays_per_mat / num_subarrays_per_row) *
                             (h_bit_mux_sense_amp_precharge_sa_mux_write_driver_write_mux +
                              h_subarray_out_drv + h_comparators);

    double w_non_cell_area = MAX(w_row_predecode_output_wires, num_subarrays_per_row * w_row_decoder);

    if (deg_bl_muxing > 1) {
        h_bit_mux_dec_out_wires = deg_bl_muxing * g_tp.wire_inside_mat.pitch * (RWP + ERP);
    }
    if (dp.Ndsam_lev_1 > 1) {
        h_senseamp_mux_dec_out_wires =  dp.Ndsam_lev_1 * g_tp.wire_inside_mat.pitch * (RWP + ERP);
    }
    if (dp.Ndsam_lev_2 > 1) {
        h_senseamp_mux_dec_out_wires += dp.Ndsam_lev_2 * g_tp.wire_inside_mat.pitch * (RWP + ERP);
    }

    double h_addr_datain_wires;
    if (!g_ip->ver_htree_wires_over_array) {
        h_addr_datain_wires = (dp.number_addr_bits_mat +
                               dp.number_way_select_signals_mat +
                               (dp.num_di_b_mat + dp.num_do_b_mat) /
                               num_subarrays_per_row) *
            g_tp.wire_inside_mat.pitch * (RWP + ERP + EWP);

        if (is_fa || pure_cam) {
            h_addr_datain_wires =
                (dp.number_addr_bits_mat +
                 dp.number_way_select_signals_mat +  //TODO: revisit
                 (dp.num_di_b_mat + dp.num_do_b_mat ) / num_subarrays_per_row) *
                g_tp.wire_inside_mat.pitch * (RWP + ERP + EWP) +
                (dp.num_si_b_mat + dp.num_so_b_mat ) / num_subarrays_per_row *
                g_tp.wire_inside_mat.pitch * SCHP;
        }
        //h_non_cell_area = 2 * h_bit_mux_sense_amp_precharge_sa_mux +
        //MAX(h_addr_datain_wires, 2 * h_subarray_out_drv);
        h_non_cell_area = (h_bit_mux_sense_amp_precharge_sa_mux_write_driver_write_mux + h_comparators +
                           h_subarray_out_drv) * (num_subarrays_per_mat / num_subarrays_per_row) +
                          h_addr_datain_wires +
                          h_bit_mux_dec_out_wires +
                          h_senseamp_mux_dec_out_wires;

    }

    // double area_rectangle_center_mat = h_non_cell_area * w_non_cell_area;
    double area_mat_center_circuitry = (r_predec_blk_drv1->area.get_area() +
                                        b_mux_predec_blk_drv1->area.get_area() +
                                        sa_mux_lev_1_predec_blk_drv1->area.get_area() +
                                        sa_mux_lev_2_predec_blk_drv1->area.get_area() +
                                        way_sel_drv1->area.get_area() +
                                        r_predec_blk_drv2->area.get_area() +
                                        b_mux_predec_blk_drv2->area.get_area() +
                                        sa_mux_lev_1_predec_blk_drv2->area.get_area() +
                                        sa_mux_lev_2_predec_blk_drv2->area.get_area() +
                                        r_predec_blk1->area.get_area() +
                                        b_mux_predec_blk1->area.get_area() +
                                        sa_mux_lev_1_predec_blk1->area.get_area() +
                                        sa_mux_lev_2_predec_blk1->area.get_area() +
                                        r_predec_blk2->area.get_area() +
                                        b_mux_predec_blk2->area.get_area() +
                                        sa_mux_lev_1_predec_blk2->area.get_area() +
                                        sa_mux_lev_2_predec_blk2->area.get_area() +
                                        bit_mux_dec->area.get_area() +
                                        sa_mux_lev_1_dec->area.get_area() +
                                        sa_mux_lev_2_dec->area.get_area()) * (RWP + ERP + EWP);

    double area_efficiency_mat;

//  if (!is_fa)
//  {
    assert(num_subarrays_per_mat / num_subarrays_per_row > 0);
    area.h = (num_subarrays_per_mat / num_subarrays_per_row) *
        subarray.area.h + h_non_cell_area;
    area.w = num_subarrays_per_row * subarray.area.get_w() + w_non_cell_area;
    area.w = (area.h * area.w + area_mat_center_circuitry) / area.h;
    area_efficiency_mat = subarray.area.get_area() * num_subarrays_per_mat *
        100.0 / area.get_area();

//    cout<<"h_bit_mux_sense_amp_precharge_sa_mux_write_driver_write_mux"<<h_bit_mux_sense_amp_precharge_sa_mux_write_driver_write_mux<<endl;
//    cout<<"h_comparators"<<h_comparators<<endl;
//    cout<<"h_subarray_out_drv"<<h_subarray_out_drv<<endl;
//    cout<<"h_addr_datain_wires"<<h_addr_datain_wires<<endl;
//    cout<<"h_bit_mux_dec_out_wires"<<h_bit_mux_dec_out_wires<<endl;
//    cout<<"h_senseamp_mux_dec_out_wires"<<h_senseamp_mux_dec_out_wires<<endl;
//    cout<<"h_non_cell_area"<<h_non_cell_area<<endl;
//    cout<<"area.h =" << (num_subarrays_per_mat/num_subarrays_per_row)* subarray.area.h<<endl;
//    cout<<"w_non_cell_area"<<w_non_cell_area<<endl;
//    cout<<"area_mat_center_circuitry"<<area_mat_center_circuitry<<endl;

    assert(area.h > 0);
    assert(area.w > 0);
//  }
//  else
//  {
//    area.h = (num_subarrays_per_mat / num_subarrays_per_row) * subarray.area.get_h() + h_non_cell_area;
//    area.w = num_subarrays_per_row * subarray.area.get_w() + w_non_cell_area;
//    area.w = (area.h*area.w + area_mat_center_circuitry) / area.h;
//    area_efficiency_mat = subarray.area.get_area() * num_subarrays_per_row * 100.0 / area.get_area();
//  }
}



Mat::~Mat() {
    delete row_dec;
    delete bit_mux_dec;
    delete sa_mux_lev_1_dec;
    delete sa_mux_lev_2_dec;

    delete r_predec->blk1;
    delete r_predec->blk2;
    delete b_mux_predec->blk1;
    delete b_mux_predec->blk2;
    delete sa_mux_lev_1_predec->blk1;
    delete sa_mux_lev_1_predec->blk2;
    delete sa_mux_lev_2_predec->blk1;
    delete sa_mux_lev_2_predec->blk2;
    delete dummy_way_sel_predec_blk1;
    delete dummy_way_sel_predec_blk2;

    delete r_predec->drv1;
    delete r_predec->drv2;
    delete b_mux_predec->drv1;
    delete b_mux_predec->drv2;
    delete sa_mux_lev_1_predec->drv1;
    delete sa_mux_lev_1_predec->drv2;
    delete sa_mux_lev_2_predec->drv1;
    delete sa_mux_lev_2_predec->drv2;
    delete way_sel_drv1;
    delete dummy_way_sel_predec_blk_drv2;

    delete r_predec;
    delete b_mux_predec;
    delete sa_mux_lev_1_predec;
    delete sa_mux_lev_2_predec;

    delete subarray_out_wire;
    if (!pure_cam)
        delete bl_precharge_eq_drv;

    if (is_fa || pure_cam) {
        delete sl_precharge_eq_drv ;
        delete sl_data_drv ;
        delete cam_bl_precharge_eq_drv;
        delete ml_precharge_drv;
        delete ml_to_ram_wl_drv;
    }
}



double Mat::compute_delays(double inrisetime) {
    int k;
    double rd, C_intrinsic, C_ld, tf, R_bl_precharge, r_b_metal, R_bl, C_bl;
    double outrisetime_search, outrisetime, row_dec_outrisetime;
    // delay calculation for tags of fully associative cache
    if (is_fa || pure_cam) {
        //Compute search access time
        outrisetime_search = compute_cam_delay(inrisetime);
        if (is_fa) {
            bl_precharge_eq_drv->compute_delay(0);
            k = ml_to_ram_wl_drv->number_gates - 1;
            rd = tr_R_on(ml_to_ram_wl_drv->width_n[k], NCH, 1, is_dram, false, true);
            C_intrinsic = drain_C_(ml_to_ram_wl_drv->width_n[k], PCH, 1, 1, 4 *
                                   cell.h, is_dram, false, true) +
                drain_C_(ml_to_ram_wl_drv->width_n[k], NCH, 1, 1, 4 * cell.h,
                         is_dram, false, true);
            C_ld = ml_to_ram_wl_drv->c_gate_load +
                ml_to_ram_wl_drv->c_wire_load;
            tf = rd * (C_intrinsic + C_ld) + ml_to_ram_wl_drv->r_wire_load * C_ld / 2;
            delay_wl_reset = horowitz(0, tf, 0.5, 0.5, RISE);

            R_bl_precharge = tr_R_on(g_tp.w_pmos_bl_precharge, PCH, 1, is_dram, false, false);
            r_b_metal = cam_cell.h * g_tp.wire_local.R_per_um;//dummy rows in sram are filled in
            R_bl = subarray.num_rows * r_b_metal;
            C_bl = subarray.C_bl;
            delay_bl_restore = bl_precharge_eq_drv->delay +
                log((g_tp.sram.Vbitpre - 0.1 * dp.V_b_sense) /
                    (g_tp.sram.Vbitpre - dp.V_b_sense)) *
                (R_bl_precharge * C_bl + R_bl * C_bl / 2);


            outrisetime_search = compute_bitline_delay(outrisetime_search);
            outrisetime_search = compute_sa_delay(outrisetime_search);
        }
        outrisetime_search = compute_subarray_out_drv(outrisetime_search);
        subarray_out_wire->set_in_rise_time(outrisetime_search);
        outrisetime_search = subarray_out_wire->signal_rise_time();
        delay_subarray_out_drv_htree = delay_subarray_out_drv + subarray_out_wire->delay;


        //TODO: this is just for compute plain read/write energy for fa and cam, plain read/write access timing need to be revisited.
        outrisetime = r_predec->compute_delays(inrisetime);
        row_dec_outrisetime = row_dec->compute_delays(outrisetime);

        outrisetime = b_mux_predec->compute_delays(inrisetime);
        bit_mux_dec->compute_delays(outrisetime);

        outrisetime = sa_mux_lev_1_predec->compute_delays(inrisetime);
        sa_mux_lev_1_dec->compute_delays(outrisetime);

        outrisetime = sa_mux_lev_2_predec->compute_delays(inrisetime);
        sa_mux_lev_2_dec->compute_delays(outrisetime);

        if (pure_cam) {
            outrisetime = compute_bitline_delay(row_dec_outrisetime);
            outrisetime = compute_sa_delay(outrisetime);
        }
        return outrisetime_search;
    } else {
        bl_precharge_eq_drv->compute_delay(0);
        if (row_dec->exist == true) {
            int k = row_dec->num_gates - 1;
            double rd = tr_R_on(row_dec->w_dec_n[k], NCH, 1, is_dram, false, true);
            // TODO: this 4*cell.h number must be revisited
            double C_intrinsic = drain_C_(row_dec->w_dec_p[k], PCH, 1, 1, 4 *
                                          cell.h, is_dram, false, true) +
                drain_C_(row_dec->w_dec_n[k], NCH, 1, 1, 4 * cell.h, is_dram,
                         false, true);
            double C_ld = row_dec->C_ld_dec_out;
            double tf = rd * (C_intrinsic + C_ld) + row_dec->R_wire_dec_out * C_ld / 2;
            delay_wl_reset = horowitz(0, tf, 0.5, 0.5, RISE);
        }
        double R_bl_precharge = tr_R_on(g_tp.w_pmos_bl_precharge, PCH, 1, is_dram, false, false);
        double r_b_metal = cell.h * g_tp.wire_local.R_per_um;
        double R_bl = subarray.num_rows * r_b_metal;
        double C_bl = subarray.C_bl;

        if (is_dram) {
            delay_bl_restore = bl_precharge_eq_drv->delay + 2.3 * (R_bl_precharge * C_bl + R_bl * C_bl / 2);
        } else {
            delay_bl_restore = bl_precharge_eq_drv->delay +
                log((g_tp.sram.Vbitpre - 0.1 * dp.V_b_sense) /
                    (g_tp.sram.Vbitpre - dp.V_b_sense)) *
                (R_bl_precharge * C_bl + R_bl * C_bl / 2);
        }
    }



    outrisetime = r_predec->compute_delays(inrisetime);
    row_dec_outrisetime = row_dec->compute_delays(outrisetime);

    outrisetime = b_mux_predec->compute_delays(inrisetime);
    bit_mux_dec->compute_delays(outrisetime);

    outrisetime = sa_mux_lev_1_predec->compute_delays(inrisetime);
    sa_mux_lev_1_dec->compute_delays(outrisetime);

    outrisetime = sa_mux_lev_2_predec->compute_delays(inrisetime);
    sa_mux_lev_2_dec->compute_delays(outrisetime);

    outrisetime = compute_bitline_delay(row_dec_outrisetime);
    outrisetime = compute_sa_delay(outrisetime);
    outrisetime = compute_subarray_out_drv(outrisetime);
    subarray_out_wire->set_in_rise_time(outrisetime);
    outrisetime = subarray_out_wire->signal_rise_time();

    delay_subarray_out_drv_htree = delay_subarray_out_drv + subarray_out_wire->delay;

    if (dp.is_tag == true && dp.fully_assoc == false) {
        compute_comparator_delay(0);
    }

    if (row_dec->exist == false) {
        delay_wl_reset = MAX(r_predec->blk1->delay, r_predec->blk2->delay);
    }
    return outrisetime;
}



double Mat::compute_bit_mux_sa_precharge_sa_mux_wr_drv_wr_mux_h() {

    double height =
        compute_tr_width_after_folding(g_tp.w_pmos_bl_precharge,
                                       camFlag ? cam_cell.w :
                                       cell.w / (2 * (RWP + ERP + SCHP))) +
        // precharge circuitry
        compute_tr_width_after_folding(g_tp.w_pmos_bl_eq,
                                       camFlag ? cam_cell.w :
                                       cell.w / (RWP + ERP + SCHP));

    if (deg_bl_muxing > 1) {
        // col mux tr height
        height +=
            compute_tr_width_after_folding(g_tp.w_nmos_b_mux,
                                           cell.w / (2 * (RWP + ERP)));
        // height += deg_bl_muxing * g_tp.wire_inside_mat.pitch * (RWP + ERP);  // bit mux dec out wires height
    }

    height += height_sense_amplifier(/*camFlag? sram_cell.w:*/cell.w * deg_bl_muxing / (RWP + ERP));  // sense_amp_height

    if (dp.Ndsam_lev_1 > 1) {
        height += compute_tr_width_after_folding(
                      g_tp.w_nmos_sa_mux, cell.w * dp.Ndsam_lev_1 / (RWP + ERP));  // sense_amp_mux_height
        //height_senseamp_mux_decode_output_wires =  Ndsam * wire_inside_mat_pitch * (RWP + ERP);
    }

    if (dp.Ndsam_lev_2 > 1) {
        height += compute_tr_width_after_folding(
                      g_tp.w_nmos_sa_mux, cell.w * deg_bl_muxing * dp.Ndsam_lev_1 / (RWP + ERP));  // sense_amp_mux_height
        //height_senseamp_mux_decode_output_wires =  Ndsam * wire_inside_mat_pitch * (RWP + ERP);

        // add height of inverter-buffers between the two levels (pass-transistors) of sense-amp mux
        height += 2 * compute_tr_width_after_folding(
                      pmos_to_nmos_sz_ratio(is_dram) * g_tp.min_w_nmos_, cell.w * dp.Ndsam_lev_2 / (RWP + ERP));
        height += 2 * compute_tr_width_after_folding(g_tp.min_w_nmos_, cell.w * dp.Ndsam_lev_2 / (RWP + ERP));
    }

    // TODO: this should be uncommented...
    /*if (deg_bl_muxing * dp.Ndsam_lev_1 * dp.Ndsam_lev_2 > 1)
      {
    //height_write_mux_decode_output_wires = deg_bl_muxing * Ndsam * g_tp.wire_inside_mat.pitch * (RWP + EWP);
    double width_write_driver_write_mux  = width_write_driver_or_write_mux();
    double height_write_driver_write_mux = compute_tr_width_after_folding(2 * width_write_driver_write_mux,
    cell.w *
    // deg_bl_muxing *
    dp.Ndsam_lev_1 * dp.Ndsam_lev_2 / (RWP + EWP));
    height += height_write_driver_write_mux;
    }*/

    return height;
}



double Mat::compute_cam_delay(double inrisetime) {

    double out_time_ramp, this_delay;
    double Rwire, tf, c_intrinsic, rd, Cwire, c_gate_load;


    double Wdecdrivep, Wdecdriven, Wfadriven, Wfadrivep, Wfadrive2n, Wfadrive2p, Wfadecdrive1n, Wfadecdrive1p,
    Wfadecdrive2n, Wfadecdrive2p, Wfadecdriven, Wfadecdrivep, Wfaprechn, Wfaprechp,
    Wdummyn, Wdummyinvn, Wdummyinvp, Wfainvn, Wfainvp, Waddrnandn, Waddrnandp,
    Wfanandn, Wfanandp, Wfanorn, Wfanorp, Wdecnandn, Wdecnandp, W_hit_miss_n, W_hit_miss_p;

    double c_matchline_metal, r_matchline_metal, c_searchline_metal, r_searchline_metal,  dynSearchEng;
    int Htagbits;

    double driver_c_gate_load;
    double driver_c_wire_load;
    double driver_r_wire_load;
    //double searchline_precharge_time;

    double leak_power_cc_inverters_sram_cell         = 0;
    double leak_power_acc_tr_RW_or_WR_port_sram_cell = 0;
    double leak_power_RD_port_sram_cell              = 0;
    double leak_power_SCHP_port_sram_cell            = 0;
    double leak_comparator_cam_cell                  =0;

    double gate_leak_comparator_cam_cell          = 0;
    double gate_leak_power_cc_inverters_sram_cell = 0;
    double gate_leak_power_RD_port_sram_cell      = 0;
    double gate_leak_power_SCHP_port_sram_cell    = 0;

    c_matchline_metal   = cam_cell.get_w() * g_tp.wire_local.C_per_um;
    c_searchline_metal  = cam_cell.get_h() * g_tp.wire_local.C_per_um;
    r_matchline_metal   = cam_cell.get_w() * g_tp.wire_local.R_per_um;
    r_searchline_metal  = cam_cell.get_h() * g_tp.wire_local.R_per_um;

    dynSearchEng = 0.0;
    delay_matchchline = 0.0;
    double p_to_n_sizing_r = pmos_to_nmos_sz_ratio(is_dram);
    bool linear_scaling = false;

    if (linear_scaling) {
        Wdecdrivep    =  450 * g_ip->F_sz_um;//this was 360 micron for the 0.8 micron process
        Wdecdriven    =  300 * g_ip->F_sz_um;//this was 240 micron for the 0.8 micron process
        Wfadriven     = 62.5 * g_ip->F_sz_um;//this was  50 micron for the 0.8 micron process
        Wfadrivep     =  125 * g_ip->F_sz_um;//this was 100 micron for the 0.8 micron process
        Wfadrive2n    =  250 * g_ip->F_sz_um;//this was 200 micron for the 0.8 micron process
        Wfadrive2p    =  500 * g_ip->F_sz_um;//this was 400 micron for the 0.8 micron process
        Wfadecdrive1n = 6.25 * g_ip->F_sz_um;//this was   5 micron for the 0.8 micron process
        Wfadecdrive1p = 12.5 * g_ip->F_sz_um;//this was  10 micron for the 0.8 micron process
        Wfadecdrive2n =   25 * g_ip->F_sz_um;//this was  20 micron for the 0.8 micron process
        Wfadecdrive2p =   50 * g_ip->F_sz_um;//this was  40 micron for the 0.8 micron process
        Wfadecdriven  = 62.5 * g_ip->F_sz_um;//this was  50 micron for the 0.8 micron process
        Wfadecdrivep  =  125 * g_ip->F_sz_um;//this was 100 micron for the 0.8 micron process
        Wfaprechn     =  7.5 * g_ip->F_sz_um;//this was   6 micron for the 0.8 micron process
        Wfainvn       = 12.5 * g_ip->F_sz_um;//this was  10 micron for the 0.8 micron process
        Wfainvp       =   25 * g_ip->F_sz_um;//this was  20 micron for the 0.8 micron process
        Wfanandn      =   25 * g_ip->F_sz_um;//this was  20 micron for the 0.8 micron process
        Wfanandp      = 37.5 * g_ip->F_sz_um;//this was  30 micron for the 0.8 micron process
        Wdecnandn     = 12.5 * g_ip->F_sz_um;//this was  10 micron for the 0.8 micron process
        Wdecnandp     = 37.5 * g_ip->F_sz_um;//this was  30 micron for the 0.8 micron process

        Wfaprechp     = 12.5 * g_ip->F_sz_um;//this was  10 micron for the 0.8 micron process
        Wdummyn       = 12.5 * g_ip->F_sz_um;//this was  10 micron for the 0.8 micron process
        Wdummyinvn    =   75 * g_ip->F_sz_um;//this was  60 micron for the 0.8 micron process
        Wdummyinvp    =  100 * g_ip->F_sz_um;//this was  80 micron for the 0.8 micron process
        Waddrnandn    = 62.5 * g_ip->F_sz_um;//this was  50 micron for the 0.8 micron process
        Waddrnandp    = 62.5 * g_ip->F_sz_um;//this was  50 micron for the 0.8 micron process
        Wfanorn       = 6.25 * g_ip->F_sz_um;//this was   5 micron for the 0.8 micron process
        Wfanorp       = 12.5 * g_ip->F_sz_um;//this was  10 micron for the 0.8 micron process
        W_hit_miss_n    = Wdummyn;
        W_hit_miss_p    = g_tp.min_w_nmos_*p_to_n_sizing_r;
        //TODO: this number should updated using new layout; from the NAND to output NOR should be computed using logical effort
    } else {
        Wdecdrivep    =  450 * g_ip->F_sz_um;//this was 360 micron for the 0.8 micron process
        Wdecdriven    =  300 * g_ip->F_sz_um;//this was 240 micron for the 0.8 micron process
        Wfadriven     = 62.5 * g_ip->F_sz_um;//this was  50 micron for the 0.8 micron process
        Wfadrivep     =  125 * g_ip->F_sz_um;//this was 100 micron for the 0.8 micron process
        Wfadrive2n    =  250 * g_ip->F_sz_um;//this was 200 micron for the 0.8 micron process
        Wfadrive2p    =  500 * g_ip->F_sz_um;//this was 400 micron for the 0.8 micron process
        Wfadecdrive1n = 6.25 * g_ip->F_sz_um;//this was   5 micron for the 0.8 micron process
        Wfadecdrive1p = 12.5 * g_ip->F_sz_um;//this was  10 micron for the 0.8 micron process
        Wfadecdrive2n =   25 * g_ip->F_sz_um;//this was  20 micron for the 0.8 micron process
        Wfadecdrive2p =   50 * g_ip->F_sz_um;//this was  40 micron for the 0.8 micron process
        Wfadecdriven  = 62.5 * g_ip->F_sz_um;//this was  50 micron for the 0.8 micron process
        Wfadecdrivep  =  125 * g_ip->F_sz_um;//this was 100 micron for the 0.8 micron process
        Wfaprechn     =  7.5 * g_ip->F_sz_um;//this was   6 micron for the 0.8 micron process
        Wfainvn       = 12.5 * g_ip->F_sz_um;//this was  10 micron for the 0.8 micron process
        Wfainvp       =   25 * g_ip->F_sz_um;//this was  20 micron for the 0.8 micron process
        Wfanandn      =   25 * g_ip->F_sz_um;//this was  20 micron for the 0.8 micron process
        Wfanandp      = 37.5 * g_ip->F_sz_um;//this was  30 micron for the 0.8 micron process
        Wdecnandn     = 12.5 * g_ip->F_sz_um;//this was  10 micron for the 0.8 micron process
        Wdecnandp     = 37.5 * g_ip->F_sz_um;//this was  30 micron for the 0.8 micron process

        Wfaprechp     = g_tp.w_pmos_bl_precharge;//this was  10 micron for the 0.8 micron process
        Wdummyn       = g_tp.cam.cell_nmos_w;
        Wdummyinvn    =   75 * g_ip->F_sz_um;//this was  60 micron for the 0.8 micron process
        Wdummyinvp    =  100 * g_ip->F_sz_um;//this was  80 micron for the 0.8 micron process
        Waddrnandn    = 62.5 * g_ip->F_sz_um;//this was  50 micron for the 0.8 micron process
        Waddrnandp    = 62.5 * g_ip->F_sz_um;//this was  50 micron for the 0.8 micron process
        Wfanorn       = 6.25 * g_ip->F_sz_um;//this was   5 micron for the 0.8 micron process
        Wfanorp       = 12.5 * g_ip->F_sz_um;//this was  10 micron for the 0.8 micron process
        W_hit_miss_n    = Wdummyn;
        W_hit_miss_p    = g_tp.min_w_nmos_*p_to_n_sizing_r;
    }

    Htagbits = (int)(ceil ((double) (subarray.num_cols_fa_cam) / 2.0));

    /* First stage, searchline is precharged. searchline data driver drives the searchline to open (if miss) the comparators.
       search_line_delay, search_line_power, search_line_restore_delay for cycle time computation.
       From the driver(am and an) to the comparators in all the rows including the dummy row,
       Assuming that comparators in both the normal matching line and the dummy matching line have the same sizing */

    //Searchline precharge circuitry is same as that of bitline. However, no sharing between search ports and r/w ports
    //Searchline precharge routes horizontally
    driver_c_gate_load = subarray.num_cols_fa_cam * gate_C(2 * g_tp.w_pmos_bl_precharge + g_tp.w_pmos_bl_eq, 0, is_dram, false, false);
    driver_c_wire_load = subarray.num_cols_fa_cam * cam_cell.w * g_tp.wire_outside_mat.C_per_um;
    driver_r_wire_load = subarray.num_cols_fa_cam * cam_cell.w * g_tp.wire_outside_mat.R_per_um;

    sl_precharge_eq_drv = new Driver(
        driver_c_gate_load,
        driver_c_wire_load,
        driver_r_wire_load,
        is_dram);

    //searchline data driver ; subarray.num_rows + 1 is because of the dummy row
    //data drv should only have gate_C not 2*gate_C since the two searchlines are differential--same as bitlines
    driver_c_gate_load = (subarray.num_rows + 1) * gate_C(Wdummyn, 0, is_dram, false, false);
    driver_c_wire_load = (subarray.num_rows + 1) * c_searchline_metal;
    driver_r_wire_load = (subarray.num_rows + 1) * r_searchline_metal;
    sl_data_drv = new Driver(
        driver_c_gate_load,
        driver_c_wire_load,
        driver_r_wire_load,
        is_dram);

    sl_precharge_eq_drv->compute_delay(0);
    double R_bl_precharge = tr_R_on(g_tp.w_pmos_bl_precharge, PCH, 1, is_dram, false, false);//Assuming CAM and SRAM have same Pre_eq_dr
    double r_b_metal = cam_cell.h * g_tp.wire_local.R_per_um;
    double R_bl = (subarray.num_rows + 1) * r_b_metal;
    double C_bl = subarray.C_bl_cam;
    delay_cam_sl_restore = sl_precharge_eq_drv->delay
        + log(g_tp.cam.Vbitpre) * (R_bl_precharge * C_bl + R_bl * C_bl / 2);

    out_time_ramp = sl_data_drv->compute_delay(inrisetime);//After entering one mat, start to consider the inrisetime from 0(0 is passed from outside)

    //matchline ops delay
    delay_matchchline += sl_data_drv->delay;

    /* second stage, from the trasistors in the comparators(both normal row and dummy row) to the NAND gates that combins both half*/
    //matchline delay, matchline power, matchline_reset for cycle time computation,

    ////matchline precharge circuitry routes vertically
    //There are two matchline precharge driver chains per subarray.
    driver_c_gate_load = (subarray.num_rows + 1) * gate_C(Wfaprechp, 0, is_dram);
    driver_c_wire_load = (subarray.num_rows + 1) * c_searchline_metal;
    driver_r_wire_load = (subarray.num_rows + 1) * r_searchline_metal;

    ml_precharge_drv = new Driver(
        driver_c_gate_load,
        driver_c_wire_load,
        driver_r_wire_load,
        is_dram);

    ml_precharge_drv->compute_delay(0);


    rd =  tr_R_on(Wdummyn, NCH, 2, is_dram);
    c_intrinsic = Htagbits *
        (2 * drain_C_(Wdummyn, NCH, 2, 1, g_tp.cell_h_def,
                      is_dram)//TODO: the cell_h_def should be revisit
         + drain_C_(Wfaprechp, PCH, 1, 1, g_tp.cell_h_def, is_dram) /
         Htagbits);//since each halve only has one precharge tx per matchline

    Cwire = c_matchline_metal * Htagbits;
    Rwire = r_matchline_metal * Htagbits;
    c_gate_load = gate_C(Waddrnandn + Waddrnandp, 0, is_dram);

    double R_ml_precharge = tr_R_on(Wfaprechp, PCH, 1, is_dram);
    //double r_ml_metal = cam_cell.w * g_tp.wire_local.R_per_um;
    double R_ml = Rwire;
    double C_ml = Cwire + c_intrinsic;
    //TODO: latest CAM has sense amps on matchlines too
    delay_cam_ml_reset = ml_precharge_drv->delay
        + log(g_tp.cam.Vbitpre) * (R_ml_precharge * C_ml + R_ml * C_ml / 2);

    //matchline ops delay
    tf = rd * (c_intrinsic + Cwire / 2 + c_gate_load) + Rwire * (Cwire / 2 + c_gate_load);
    this_delay = horowitz(out_time_ramp, tf, VTHFA2, VTHFA3, FALL);
    delay_matchchline += this_delay;
    out_time_ramp = this_delay / VTHFA3;

    dynSearchEng += ((c_intrinsic + Cwire + c_gate_load) *
                     (subarray.num_rows + 1)) //TODO: need to be precise
        * g_tp.peri_global.Vdd * g_tp.peri_global.Vdd *
        2;//each subarry has two halves

    /* third stage, from the NAND2 gates to the drivers in the dummy row */
    rd = tr_R_on(Waddrnandn, NCH, 2, is_dram);
    c_intrinsic = drain_C_(Waddrnandn, NCH, 2, 1, g_tp.cell_h_def, is_dram) +
                  drain_C_(Waddrnandp, PCH, 1, 1, g_tp.cell_h_def, is_dram) * 2;
    c_gate_load = gate_C(Wdummyinvn + Wdummyinvp, 0, is_dram);
    tf = rd * (c_intrinsic + c_gate_load);
    this_delay = horowitz(out_time_ramp, tf, VTHFA3, VTHFA4, RISE);
    out_time_ramp = this_delay / (1 - VTHFA4);
    delay_matchchline += this_delay;

    //only the dummy row has the extra inverter between NAND and NOR gates
    dynSearchEng += (c_intrinsic * (subarray.num_rows + 1) + c_gate_load * 2) *
        g_tp.peri_global.Vdd * g_tp.peri_global.Vdd;//  * Ntbl;

    /* fourth stage, from the driver in dummy matchline to the NOR2 gate which drives the wordline of the data portion */
    rd = tr_R_on(Wdummyinvn, NCH, 1, is_dram);
    c_intrinsic = drain_C_(Wdummyinvn, NCH, 1, 1, g_tp.cell_h_def, is_dram) + drain_C_(Wdummyinvp, NCH, 1, 1, g_tp.cell_h_def, is_dram);
    Cwire = c_matchline_metal * Htagbits +  c_searchline_metal *
        (subarray.num_rows + 1) / 2;
    Rwire = r_matchline_metal * Htagbits +  r_searchline_metal *
        (subarray.num_rows + 1) / 2;
    c_gate_load = gate_C(Wfanorn + Wfanorp, 0, is_dram);
    tf = rd * (c_intrinsic + Cwire + c_gate_load) + Rwire * (Cwire / 2 + c_gate_load);
    this_delay = horowitz (out_time_ramp, tf, VTHFA4, VTHFA5, FALL);
    out_time_ramp = this_delay / VTHFA5;
    delay_matchchline += this_delay;

    dynSearchEng += (c_intrinsic + Cwire + subarray.num_rows * c_gate_load) *
        g_tp.peri_global.Vdd * g_tp.peri_global.Vdd;//* Ntbl;

    /*final statge from the NOR gate to drive the wordline of the data portion */

    //searchline data driver There are two matchline precharge driver chains per subarray.
    driver_c_gate_load = gate_C(W_hit_miss_n, 0, is_dram, false, false);//nmos of the pull down logic
    driver_c_wire_load = subarray.C_wl_ram;
    driver_r_wire_load = subarray.R_wl_ram;

    ml_to_ram_wl_drv = new Driver(
        driver_c_gate_load,
        driver_c_wire_load,
        driver_r_wire_load,
        is_dram);



    rd = tr_R_on(Wfanorn, NCH, 1, is_dram);
    c_intrinsic = 2 * drain_C_(Wfanorn, NCH, 1, 1, g_tp.cell_h_def, is_dram) +
        drain_C_(Wfanorp, NCH, 1, 1, g_tp.cell_h_def, is_dram);
    c_gate_load = gate_C(ml_to_ram_wl_drv->width_n[0] + ml_to_ram_wl_drv->width_p[0], 0, is_dram);
    tf = rd * (c_intrinsic + c_gate_load);
    this_delay = horowitz (out_time_ramp, tf, 0.5, 0.5, RISE);
    out_time_ramp = this_delay / (1 - 0.5);
    delay_matchchline += this_delay;

    out_time_ramp   = ml_to_ram_wl_drv->compute_delay(out_time_ramp);

    //c_gate_load energy is computed in ml_to_ram_wl_drv
    dynSearchEng  += (c_intrinsic) * g_tp.peri_global.Vdd * g_tp.peri_global.Vdd;//* Ntbl;


    /* peripheral-- hitting logic "CMOS VLSI Design Fig11.51*/
    /*Precharge the hitting logic */
    c_intrinsic = 2 *
        drain_C_(W_hit_miss_p, NCH, 2, 1, g_tp.cell_h_def, is_dram);
    Cwire = c_searchline_metal * subarray.num_rows;
    Rwire = r_searchline_metal * subarray.num_rows;
    c_gate_load = drain_C_(W_hit_miss_n, NCH, 1, 1, g_tp.cell_h_def, is_dram) *
        subarray.num_rows;

    rd = tr_R_on(W_hit_miss_p, PCH, 1, is_dram, false, false);
    //double r_ml_metal = cam_cell.w * g_tp.wire_local.R_per_um;
    double R_hit_miss = Rwire;
    double C_hit_miss = Cwire + c_intrinsic;
    delay_hit_miss_reset = log(g_tp.cam.Vbitpre) *
        (rd * C_hit_miss + R_hit_miss * C_hit_miss / 2);
    dynSearchEng  += (c_intrinsic + Cwire + c_gate_load) * g_tp.peri_global.Vdd * g_tp.peri_global.Vdd;

    /*hitting logic evaluation */
    c_intrinsic = 2 *
        drain_C_(W_hit_miss_n, NCH, 2, 1, g_tp.cell_h_def, is_dram);
    Cwire = c_searchline_metal * subarray.num_rows;
    Rwire = r_searchline_metal * subarray.num_rows;
    c_gate_load = drain_C_(W_hit_miss_n, NCH, 1, 1, g_tp.cell_h_def, is_dram) *
        subarray.num_rows;

    rd = tr_R_on(W_hit_miss_n, PCH, 1, is_dram, false, false);
    tf = rd * (c_intrinsic + Cwire / 2 + c_gate_load) + Rwire * (Cwire / 2 + c_gate_load);

    delay_hit_miss = horowitz(0, tf, 0.5, 0.5, FALL);

    if (is_fa)
        delay_matchchline += MAX(ml_to_ram_wl_drv->delay, delay_hit_miss);

    dynSearchEng  += (c_intrinsic + Cwire + c_gate_load) * g_tp.peri_global.Vdd * g_tp.peri_global.Vdd;

    /* TODO: peripheral-- Priority Encoder, usually this is not necessary in processor components*/

    power_matchline.searchOp.dynamic = dynSearchEng;

    //leakage in one subarray
    double Iport     = cmos_Isub_leakage(g_tp.cam.cell_a_w, 0,  1, nmos, false, true);//TODO: how much is the idle time? just by *2?
    double Iport_erp = cmos_Isub_leakage(g_tp.cam.cell_a_w, 0,  2, nmos, false, true);
    double Icell = cmos_Isub_leakage(g_tp.cam.cell_nmos_w, g_tp.cam.cell_pmos_w,
                                     1, inv, false, true) * 2;
    //approx XOR with Inv
    double Icell_comparator = cmos_Isub_leakage(Wdummyn, Wdummyn, 1, inv,
                                                false, true) * 2;

    leak_power_cc_inverters_sram_cell         = Icell * g_tp.cam_cell.Vdd;
    leak_comparator_cam_cell                  = Icell_comparator * g_tp.cam_cell.Vdd;
    leak_power_acc_tr_RW_or_WR_port_sram_cell = Iport * g_tp.cam_cell.Vdd;
    leak_power_RD_port_sram_cell              = Iport_erp * g_tp.cam_cell.Vdd;
    leak_power_SCHP_port_sram_cell            = 0;//search port and r/w port are sperate, therefore no access txs in search ports

    power_matchline.searchOp.leakage += leak_power_cc_inverters_sram_cell +
                                        leak_comparator_cam_cell +
                                        leak_power_acc_tr_RW_or_WR_port_sram_cell +
                                        leak_power_acc_tr_RW_or_WR_port_sram_cell * (RWP + EWP - 1) +
                                        leak_power_RD_port_sram_cell * ERP +
                                        leak_power_SCHP_port_sram_cell * SCHP;
//  power_matchline.searchOp.leakage += leak_comparator_cam_cell;
    power_matchline.searchOp.leakage *= (subarray.num_rows + 1) *
        subarray.num_cols_fa_cam;//TODO:dumy line precise
    power_matchline.searchOp.leakage += (subarray.num_rows + 1) *
        cmos_Isub_leakage(0, Wfaprechp, 1, pmos) * g_tp.cam_cell.Vdd;
    power_matchline.searchOp.leakage += (subarray.num_rows + 1) *
        cmos_Isub_leakage(Waddrnandn, Waddrnandp, 2, nand) * g_tp.cam_cell.Vdd;
    power_matchline.searchOp.leakage += (subarray.num_rows + 1) *
        cmos_Isub_leakage(Wfanorn, Wfanorp, 2, nor) * g_tp.cam_cell.Vdd;
    //In idle states, the hit/miss txs are closed (on) therefore no Isub
    power_matchline.searchOp.leakage += 0;// subarray.num_rows * cmos_Isub_leakage(W_hit_miss_n, 0,1, nmos) * g_tp.cam_cell.Vdd+
    // + cmos_Isub_leakage(0, W_hit_miss_p,1, pmos) * g_tp.cam_cell.Vdd;

    //in idle state, Ig_on only possibly exist in access transistors of read only ports
    double Ig_port_erp = cmos_Ig_leakage(g_tp.cam.cell_a_w, 0, 1, nmos, false, true);
    double Ig_cell = cmos_Ig_leakage(g_tp.cam.cell_nmos_w, g_tp.cam.cell_pmos_w,
                                     1, inv, false, true) * 2;
    double Ig_cell_comparator = cmos_Ig_leakage(Wdummyn, Wdummyn, 1, inv,
                                                false, true) * 2;

    gate_leak_comparator_cam_cell = Ig_cell_comparator * g_tp.cam_cell.Vdd;
    gate_leak_power_cc_inverters_sram_cell = Ig_cell * g_tp.cam_cell.Vdd;
    gate_leak_power_RD_port_sram_cell = Ig_port_erp * g_tp.sram_cell.Vdd;
    gate_leak_power_SCHP_port_sram_cell = 0;

    //cout<<"power_matchline.searchOp.leakage"<<power_matchline.searchOp.leakage<<endl;

    power_matchline.searchOp.gate_leakage +=
        gate_leak_power_cc_inverters_sram_cell;
    power_matchline.searchOp.gate_leakage += gate_leak_comparator_cam_cell;
    power_matchline.searchOp.gate_leakage +=
        gate_leak_power_SCHP_port_sram_cell * SCHP +
        gate_leak_power_RD_port_sram_cell * ERP;
    power_matchline.searchOp.gate_leakage *= (subarray.num_rows + 1) *
        subarray.num_cols_fa_cam;//TODO:dumy line precise
    power_matchline.searchOp.gate_leakage += (subarray.num_rows + 1) *
        cmos_Ig_leakage(0, Wfaprechp, 1, pmos) * g_tp.cam_cell.Vdd;
    power_matchline.searchOp.gate_leakage += (subarray.num_rows + 1) *
        cmos_Ig_leakage(Waddrnandn, Waddrnandp, 2, nand) * g_tp.cam_cell.Vdd;
    power_matchline.searchOp.gate_leakage += (subarray.num_rows + 1) *
        cmos_Ig_leakage(Wfanorn, Wfanorp, 2, nor) * g_tp.cam_cell.Vdd;
    power_matchline.searchOp.gate_leakage += subarray.num_rows *
        cmos_Ig_leakage(W_hit_miss_n, 0, 1, nmos) * g_tp.cam_cell.Vdd +
        + cmos_Ig_leakage(0, W_hit_miss_p, 1, pmos) * g_tp.cam_cell.Vdd;


    return out_time_ramp;
}


double Mat::width_write_driver_or_write_mux() {
    // calculate resistance of SRAM cell pull-up PMOS transistor
    // cam and sram have same cell trasistor properties
    double R_sram_cell_pull_up_tr  = tr_R_on(g_tp.sram.cell_pmos_w, NCH, 1, is_dram, true);
    double R_access_tr             = tr_R_on(g_tp.sram.cell_a_w,    NCH, 1, is_dram, true);
    double target_R_write_driver_and_mux = (2 * R_sram_cell_pull_up_tr - R_access_tr) / 2;
    double width_write_driver_nmos = R_to_w(target_R_write_driver_and_mux, NCH, is_dram);

    return width_write_driver_nmos;
}



double Mat::compute_comparators_height(
    int tagbits,
    int number_ways_in_mat,
    double subarray_mem_cell_area_width) {
    double nand2_area = compute_gate_area(NAND, 2, 0, g_tp.w_comp_n, g_tp.cell_h_def);
    double cumulative_area = nand2_area * number_ways_in_mat * tagbits / 4;
    return cumulative_area / subarray_mem_cell_area_width;
}



double Mat::compute_bitline_delay(double inrisetime) {
    double V_b_pre, v_th_mem_cell, V_wl;
    double tstep;
    double dynRdEnergy = 0.0, dynWriteEnergy = 0.0;
    double R_cell_pull_down = 0.0, R_cell_acc = 0.0, r_dev = 0.0;
    int deg_senseamp_muxing = dp.Ndsam_lev_1 * dp.Ndsam_lev_2;

    double R_b_metal = camFlag ? cam_cell.h : cell.h * g_tp.wire_local.R_per_um;
    double R_bl      = subarray.num_rows * R_b_metal;
    double C_bl      = subarray.C_bl;

    // TODO: no leakage for DRAMs?
    double leak_power_cc_inverters_sram_cell = 0;
    double gate_leak_power_cc_inverters_sram_cell = 0;
    double leak_power_acc_tr_RW_or_WR_port_sram_cell = 0;
    double leak_power_RD_port_sram_cell = 0;
    double gate_leak_power_RD_port_sram_cell = 0;

    if (is_dram == true) {
        V_b_pre = g_tp.dram.Vbitpre;
        v_th_mem_cell = g_tp.dram_acc.Vth;
        V_wl = g_tp.vpp;
        //The access transistor is not folded. So we just need to specify a
        // threshold value for the folding width that is equal to or greater
        // than Wmemcella.
        R_cell_acc = tr_R_on(g_tp.dram.cell_a_w, NCH, 1, true, true);
        r_dev = g_tp.dram_cell_Vdd / g_tp.dram_cell_I_on + R_bl / 2;
    } else { //SRAM
        V_b_pre = g_tp.sram.Vbitpre;
        v_th_mem_cell = g_tp.sram_cell.Vth;
        V_wl = g_tp.sram_cell.Vdd;
        R_cell_pull_down = tr_R_on(g_tp.sram.cell_nmos_w, NCH, 1, false, true);
        R_cell_acc = tr_R_on(g_tp.sram.cell_a_w, NCH, 1, false, true);

        //Leakage current of an SRAM cell
        //TODO: how much is the idle time? just by *2?
        double Iport = cmos_Isub_leakage(g_tp.sram.cell_a_w, 0,  1, nmos,
                                         false, true);
        double Iport_erp = cmos_Isub_leakage(g_tp.sram.cell_a_w, 0,  2, nmos,
                                             false, true);
        double Icell = cmos_Isub_leakage(g_tp.sram.cell_nmos_w,
                                         g_tp.sram.cell_pmos_w, 1, inv, false,
                                         true) * 2;//two invs per cell

        leak_power_cc_inverters_sram_cell         = Icell * g_tp.sram_cell.Vdd;
        leak_power_acc_tr_RW_or_WR_port_sram_cell = Iport * g_tp.sram_cell.Vdd;
        leak_power_RD_port_sram_cell              = Iport_erp * g_tp.sram_cell.Vdd;


        //in idle state, Ig_on only possibly exist in access transistors of read only ports
        double Ig_port_erp = cmos_Ig_leakage(g_tp.sram.cell_a_w, 0, 1, nmos,
                                             false, true);
        double Ig_cell = cmos_Ig_leakage(g_tp.sram.cell_nmos_w,
                                         g_tp.sram.cell_pmos_w, 1, inv, false,
                                         true);

        gate_leak_power_cc_inverters_sram_cell = Ig_cell * g_tp.sram_cell.Vdd;
        gate_leak_power_RD_port_sram_cell = Ig_port_erp * g_tp.sram_cell.Vdd;
    }


    double C_drain_bit_mux = drain_C_(g_tp.w_nmos_b_mux, NCH, 1, 0,
                                      camFlag ? cam_cell.w : cell.w /
                                      (2 * (RWP + ERP + SCHP)), is_dram);
    double R_bit_mux = tr_R_on(g_tp.w_nmos_b_mux, NCH, 1, is_dram);
    double C_drain_sense_amp_iso = drain_C_(g_tp.w_iso, PCH, 1, 0,
                                            camFlag ? cam_cell.w :
                                            cell.w * deg_bl_muxing /
                                            (RWP + ERP + SCHP), is_dram);
    double R_sense_amp_iso = tr_R_on(g_tp.w_iso, PCH, 1, is_dram);
    double C_sense_amp_latch = gate_C(g_tp.w_sense_p + g_tp.w_sense_n, 0,
                                      is_dram) +
        drain_C_(g_tp.w_sense_n, NCH, 1, 0, camFlag ? cam_cell.w :
                 cell.w * deg_bl_muxing / (RWP + ERP + SCHP), is_dram) +
        drain_C_(g_tp.w_sense_p, PCH, 1, 0, camFlag ? cam_cell.w :
                 cell.w * deg_bl_muxing / (RWP + ERP + SCHP), is_dram);
    double C_drain_sense_amp_mux = drain_C_(g_tp.w_nmos_sa_mux, NCH, 1, 0,
                                            camFlag ? cam_cell.w :
                                            cell.w * deg_bl_muxing /
                                            (RWP + ERP + SCHP), is_dram);

    if (is_dram) {
        double fraction = dp.V_b_sense / ((g_tp.dram_cell_Vdd / 2) *
                                          g_tp.dram_cell_C /
                                          (g_tp.dram_cell_C + C_bl));
        tstep = 2.3 * fraction * r_dev *
            (g_tp.dram_cell_C * (C_bl + 2 * C_drain_sense_amp_iso +
                                 C_sense_amp_latch + C_drain_sense_amp_mux)) /
            (g_tp.dram_cell_C + (C_bl + 2 * C_drain_sense_amp_iso +
                                 C_sense_amp_latch + C_drain_sense_amp_mux));
        delay_writeback = tstep;
        dynRdEnergy += (C_bl + 2 * C_drain_sense_amp_iso + C_sense_amp_latch +
                        C_drain_sense_amp_mux) *
            (g_tp.dram_cell_Vdd / 2) *
            g_tp.dram_cell_Vdd /* subarray.num_cols * num_subarrays_per_mat*/;
        dynWriteEnergy += (C_bl + 2 * C_drain_sense_amp_iso + C_sense_amp_latch) *
            (g_tp.dram_cell_Vdd / 2) *
            g_tp.dram_cell_Vdd /* subarray.num_cols * num_subarrays_per_mat*/ *
            num_act_mats_hor_dir * 100;
        per_bitline_read_energy = (C_bl + 2 * C_drain_sense_amp_iso +
                                   C_sense_amp_latch + C_drain_sense_amp_mux) *
            (g_tp.dram_cell_Vdd / 2) * g_tp.dram_cell_Vdd;
    } else {
        double tau;

        if (deg_bl_muxing > 1) {
            tau = (R_cell_pull_down + R_cell_acc) *
                (C_bl + 2 * C_drain_bit_mux + 2 * C_drain_sense_amp_iso +
                 C_sense_amp_latch + C_drain_sense_amp_mux) +
                R_bl * (C_bl / 2 + 2 * C_drain_bit_mux + 2 *
                        C_drain_sense_amp_iso + C_sense_amp_latch +
                        C_drain_sense_amp_mux) +
                R_bit_mux * (C_drain_bit_mux + 2 * C_drain_sense_amp_iso +
                             C_sense_amp_latch + C_drain_sense_amp_mux) +
                R_sense_amp_iso * (C_drain_sense_amp_iso + C_sense_amp_latch +
                                   C_drain_sense_amp_mux);
            dynRdEnergy += (C_bl + 2 * C_drain_bit_mux) * 2 * dp.V_b_sense *
                g_tp.sram_cell.Vdd;
            dynRdEnergy += (2 * C_drain_sense_amp_iso + C_sense_amp_latch +
                            C_drain_sense_amp_mux) *
                2 * dp.V_b_sense * g_tp.sram_cell.Vdd *
                (1.0/*subarray.num_cols * num_subarrays_per_mat*/ /
                 deg_bl_muxing);
            dynWriteEnergy += ((1.0/*subarray.num_cols *num_subarrays_per_mat*/ /
                                deg_bl_muxing) / deg_senseamp_muxing) *
                num_act_mats_hor_dir * (C_bl + 2 * C_drain_bit_mux) *
                g_tp.sram_cell.Vdd * g_tp.sram_cell.Vdd * 2;
            //Write Ops are differential for SRAM
        } else {
            tau = (R_cell_pull_down + R_cell_acc) *
                  (C_bl + C_drain_sense_amp_iso + C_sense_amp_latch + C_drain_sense_amp_mux) + R_bl * C_bl / 2 +
                  R_sense_amp_iso * (C_drain_sense_amp_iso + C_sense_amp_latch + C_drain_sense_amp_mux);
            dynRdEnergy += (C_bl + 2 * C_drain_sense_amp_iso + C_sense_amp_latch + C_drain_sense_amp_mux) *
                           2 * dp.V_b_sense * g_tp.sram_cell.Vdd /* subarray.num_cols * num_subarrays_per_mat*/;
            dynWriteEnergy += (((1.0/*subarray.num_cols * num_subarrays_per_mat*/ /
                                 deg_bl_muxing) / deg_senseamp_muxing) *
                               num_act_mats_hor_dir * C_bl) *
                g_tp.sram_cell.Vdd * g_tp.sram_cell.Vdd * 2;

        }
        tstep = tau * log(V_b_pre / (V_b_pre - dp.V_b_sense));
        power_bitline.readOp.leakage =
            leak_power_cc_inverters_sram_cell +
            leak_power_acc_tr_RW_or_WR_port_sram_cell +
            leak_power_acc_tr_RW_or_WR_port_sram_cell * (RWP + EWP - 1) +
            leak_power_RD_port_sram_cell * ERP;
        power_bitline.readOp.gate_leakage = gate_leak_power_cc_inverters_sram_cell +
                                            gate_leak_power_RD_port_sram_cell * ERP;

    }

//  cout<<"leak_power_cc_inverters_sram_cell"<<leak_power_cc_inverters_sram_cell<<endl;
//  cout<<"leak_power_acc_tr_RW_or_WR_port_sram_cell"<<leak_power_acc_tr_RW_or_WR_port_sram_cell<<endl;
//  cout<<"leak_power_acc_tr_RW_or_WR_port_sram_cell"<<leak_power_acc_tr_RW_or_WR_port_sram_cell<<endl;
//  cout<<"leak_power_RD_port_sram_cell"<<leak_power_RD_port_sram_cell<<endl;


    /* take input rise time into account */
    double m = V_wl / inrisetime;
    if (tstep <= (0.5 * (V_wl - v_th_mem_cell) / m)) {
        delay_bitline = sqrt(2 * tstep * (V_wl - v_th_mem_cell) / m);
    } else {
        delay_bitline = tstep + (V_wl - v_th_mem_cell) / (2 * m);
    }

    bool is_fa = (dp.fully_assoc) ? true : false;

    if (dp.is_tag == false || is_fa == false) {
        power_bitline.readOp.dynamic  = dynRdEnergy;
        power_bitline.writeOp.dynamic = dynWriteEnergy;
    }

    double outrisetime = 0;
    return outrisetime;
}



double Mat::compute_sa_delay(double inrisetime) {
    //int num_sa_subarray = subarray.num_cols / deg_bl_muxing; //in a subarray

    //Bitline circuitry leakage.
    double Iiso     = simplified_pmos_leakage(g_tp.w_iso, is_dram);
    double IsenseEn = simplified_nmos_leakage(g_tp.w_sense_en, is_dram);
    double IsenseN  = simplified_nmos_leakage(g_tp.w_sense_n, is_dram);
    double IsenseP  = simplified_pmos_leakage(g_tp.w_sense_p, is_dram);

    double lkgIdlePh  = IsenseEn;//+ 2*IoBufP;
    //double lkgWritePh = Iiso + IsenseEn;// + 2*IoBufP + 2*Ipch;
    double lkgReadPh  = Iiso + IsenseN + IsenseP;//+ IoBufN + IoBufP + 2*IsPch ;
    //double lkgRead = lkgReadPh * num_sa_subarray * 4 * num_act_mats_hor_dir +
    //    lkgIdlePh * num_sa_subarray * 4 * (num_mats - num_act_mats_hor_dir);
    double lkgIdle = lkgIdlePh /*num_sa_subarray * num_subarrays_per_mat*/;
    leak_power_sense_amps_closed_page_state = lkgIdlePh * g_tp.peri_global.Vdd /* num_sa_subarray * num_subarrays_per_mat*/;
    leak_power_sense_amps_open_page_state   = lkgReadPh * g_tp.peri_global.Vdd /* num_sa_subarray * num_subarrays_per_mat*/;

    // sense amplifier has to drive logic in "data out driver" and sense precharge load.
    // load seen by sense amp. New delay model for sense amp that is sensitive to both the output time
    //constant as well as the magnitude of input differential voltage.
    double C_ld = gate_C(g_tp.w_sense_p + g_tp.w_sense_n, 0, is_dram) +
        drain_C_(g_tp.w_sense_n, NCH, 1, 0,
                 camFlag ? cam_cell.w : cell.w * deg_bl_muxing /
                 (RWP + ERP + SCHP), is_dram) +
        drain_C_(g_tp.w_sense_p, PCH, 1, 0, camFlag ?
                 cam_cell.w : cell.w * deg_bl_muxing / (RWP + ERP + SCHP),
                 is_dram) +
        drain_C_(g_tp.w_iso, PCH, 1, 0, camFlag ?
                 cam_cell.w : cell.w * deg_bl_muxing / (RWP + ERP + SCHP),
                 is_dram) +
        drain_C_(g_tp.w_nmos_sa_mux, NCH, 1, 0, camFlag ?
                 cam_cell.w : cell.w * deg_bl_muxing / (RWP + ERP + SCHP),
                 is_dram);
    double tau = C_ld / g_tp.gm_sense_amp_latch;
    delay_sa = tau * log(g_tp.peri_global.Vdd / dp.V_b_sense);
    power_sa.readOp.dynamic = C_ld * g_tp.peri_global.Vdd * g_tp.peri_global.Vdd /* num_sa_subarray
                            num_subarrays_per_mat * num_act_mats_hor_dir*/;
    power_sa.readOp.leakage = lkgIdle * g_tp.peri_global.Vdd;

    double outrisetime = 0;
    return outrisetime;
}



double Mat::compute_subarray_out_drv(double inrisetime) {
    double C_ld, rd, tf, this_delay;
    double p_to_n_sz_r = pmos_to_nmos_sz_ratio(is_dram);

    // delay of signal through pass-transistor of first level of sense-amp mux to input of inverter-buffer.
    rd = tr_R_on(g_tp.w_nmos_sa_mux, NCH, 1, is_dram);
    C_ld = dp.Ndsam_lev_1 * drain_C_(g_tp.w_nmos_sa_mux, NCH, 1, 0,
                                     camFlag ? cam_cell.w : cell.w *
                                     deg_bl_muxing / (RWP + ERP + SCHP),
                                     is_dram) +
        gate_C(g_tp.min_w_nmos_ + p_to_n_sz_r * g_tp.min_w_nmos_, 0.0, is_dram);
    tf = rd * C_ld;
    this_delay = horowitz(inrisetime, tf, 0.5, 0.5, RISE);
    delay_subarray_out_drv += this_delay;
    inrisetime = this_delay / (1.0 - 0.5);
    power_subarray_out_drv.readOp.dynamic += C_ld * 0.5 * g_tp.peri_global.Vdd * g_tp.peri_global.Vdd;
    power_subarray_out_drv.readOp.leakage += 0;  // for now, let leakage of the pass transistor be 0
    power_subarray_out_drv.readOp.gate_leakage +=
        cmos_Ig_leakage(g_tp.w_nmos_sa_mux, 0, 1, nmos) * g_tp.peri_global.Vdd;
    // delay of signal through inverter-buffer to second level of sense-amp mux.
    // internal delay of buffer
    rd = tr_R_on(g_tp.min_w_nmos_, NCH, 1, is_dram);
    C_ld = drain_C_(g_tp.min_w_nmos_, NCH, 1, 1, g_tp.cell_h_def, is_dram) +
           drain_C_(p_to_n_sz_r * g_tp.min_w_nmos_, PCH, 1, 1, g_tp.cell_h_def, is_dram) +
           gate_C(g_tp.min_w_nmos_ + p_to_n_sz_r * g_tp.min_w_nmos_, 0.0, is_dram);
    tf = rd * C_ld;
    this_delay = horowitz(inrisetime, tf, 0.5, 0.5, RISE);
    delay_subarray_out_drv += this_delay;
    inrisetime = this_delay / (1.0 - 0.5);
    power_subarray_out_drv.readOp.dynamic      += C_ld * 0.5 * g_tp.peri_global.Vdd * g_tp.peri_global.Vdd;
    power_subarray_out_drv.readOp.leakage +=
        cmos_Isub_leakage(g_tp.min_w_nmos_, p_to_n_sz_r * g_tp.min_w_nmos_, 1,
                          inv, is_dram) * g_tp.peri_global.Vdd;
    power_subarray_out_drv.readOp.gate_leakage +=
        cmos_Ig_leakage(g_tp.min_w_nmos_, p_to_n_sz_r * g_tp.min_w_nmos_, 1,
                        inv) * g_tp.peri_global.Vdd;

    // inverter driving drain of pass transistor of second level of sense-amp mux.
    rd = tr_R_on(g_tp.min_w_nmos_, NCH, 1, is_dram);
    C_ld = drain_C_(g_tp.min_w_nmos_, NCH, 1, 1, g_tp.cell_h_def, is_dram) +
        drain_C_(p_to_n_sz_r * g_tp.min_w_nmos_, PCH, 1, 1, g_tp.cell_h_def,
                 is_dram) +
        drain_C_(g_tp.w_nmos_sa_mux, NCH, 1, 0, camFlag ?
                 cam_cell.w : cell.w * deg_bl_muxing * dp.Ndsam_lev_1 /
                 (RWP + ERP + SCHP), is_dram);
    tf = rd * C_ld;
    this_delay = horowitz(inrisetime, tf, 0.5, 0.5, RISE);
    delay_subarray_out_drv += this_delay;
    inrisetime = this_delay / (1.0 - 0.5);
    power_subarray_out_drv.readOp.dynamic      += C_ld * 0.5 * g_tp.peri_global.Vdd * g_tp.peri_global.Vdd;
    power_subarray_out_drv.readOp.leakage +=
        cmos_Isub_leakage(g_tp.min_w_nmos_, p_to_n_sz_r * g_tp.min_w_nmos_, 1,
                          inv) * g_tp.peri_global.Vdd;
    power_subarray_out_drv.readOp.gate_leakage +=
        cmos_Ig_leakage(g_tp.min_w_nmos_, p_to_n_sz_r * g_tp.min_w_nmos_, 1,
                        inv) * g_tp.peri_global.Vdd;


    // delay of signal through pass-transistor to input of subarray output driver.
    rd = tr_R_on(g_tp.w_nmos_sa_mux, NCH, 1, is_dram);
    C_ld = dp.Ndsam_lev_2 *
        drain_C_(g_tp.w_nmos_sa_mux, NCH, 1, 0, camFlag ? cam_cell.w :
                 cell.w * deg_bl_muxing * dp.Ndsam_lev_1 / (RWP + ERP + SCHP),
                 is_dram) +
           //gate_C(subarray_out_wire->repeater_size * g_tp.min_w_nmos_ * (1 + p_to_n_sz_r), 0.0, is_dram);
        gate_C(subarray_out_wire->repeater_size *
               (subarray_out_wire->wire_length /
                subarray_out_wire->repeater_spacing) * g_tp.min_w_nmos_ *
               (1 + p_to_n_sz_r), 0.0, is_dram);
    tf = rd * C_ld;
    this_delay = horowitz(inrisetime, tf, 0.5, 0.5, RISE);
    delay_subarray_out_drv += this_delay;
    inrisetime = this_delay / (1.0 - 0.5);
    power_subarray_out_drv.readOp.dynamic += C_ld * 0.5 * g_tp.peri_global.Vdd * g_tp.peri_global.Vdd;
    power_subarray_out_drv.readOp.leakage += 0;  // for now, let leakage of the pass transistor be 0
    power_subarray_out_drv.readOp.gate_leakage +=
        cmos_Ig_leakage(g_tp.w_nmos_sa_mux, 0, 1, nmos) * g_tp.peri_global.Vdd;


    return inrisetime;
}



double Mat::compute_comparator_delay(double inrisetime) {
    int A = g_ip->tag_assoc;

    int tagbits_ = dp.tagbits / 4; // Assuming there are 4 quarter comparators. input tagbits is already
    // a multiple of 4.

    /* First Inverter */
    double Ceq = gate_C(g_tp.w_comp_inv_n2 + g_tp.w_comp_inv_p2, 0, is_dram) +
                 drain_C_(g_tp.w_comp_inv_p1, PCH, 1, 1, g_tp.cell_h_def, is_dram) +
                 drain_C_(g_tp.w_comp_inv_n1, NCH, 1, 1, g_tp.cell_h_def, is_dram);
    double Req = tr_R_on(g_tp.w_comp_inv_p1, PCH, 1, is_dram);
    double tf  = Req * Ceq;
    double st1del = horowitz(inrisetime, tf, VTHCOMPINV, VTHCOMPINV, FALL);
    double nextinputtime = st1del / VTHCOMPINV;
    power_comparator.readOp.dynamic += 0.5 * Ceq * g_tp.peri_global.Vdd * g_tp.peri_global.Vdd * 4 * A;

    //For each degree of associativity
    //there are 4 such quarter comparators
    double lkgCurrent = cmos_Isub_leakage(g_tp.w_comp_inv_n1,
                                          g_tp.w_comp_inv_p1, 1, inv,
                                          is_dram) * 4 * A;
    double gatelkgCurrent = cmos_Ig_leakage(g_tp.w_comp_inv_n1,
                                            g_tp.w_comp_inv_p1, 1, inv,
                                            is_dram) * 4 * A;
    /* Second Inverter */
    Ceq = gate_C(g_tp.w_comp_inv_n3 + g_tp.w_comp_inv_p3, 0, is_dram) +
          drain_C_(g_tp.w_comp_inv_p2, PCH, 1, 1, g_tp.cell_h_def, is_dram) +
          drain_C_(g_tp.w_comp_inv_n2, NCH, 1, 1, g_tp.cell_h_def, is_dram);
    Req = tr_R_on(g_tp.w_comp_inv_n2, NCH, 1, is_dram);
    tf = Req * Ceq;
    double st2del = horowitz(nextinputtime, tf, VTHCOMPINV, VTHCOMPINV, RISE);
    nextinputtime = st2del / (1.0 - VTHCOMPINV);
    power_comparator.readOp.dynamic += 0.5 * Ceq * g_tp.peri_global.Vdd * g_tp.peri_global.Vdd * 4 * A;
    lkgCurrent += cmos_Isub_leakage(g_tp.w_comp_inv_n2, g_tp.w_comp_inv_p2, 1,
                                    inv, is_dram) * 4 * A;
    gatelkgCurrent += cmos_Ig_leakage(g_tp.w_comp_inv_n2, g_tp.w_comp_inv_p2, 1,
                                      inv, is_dram) * 4 * A;

    /* Third Inverter */
    Ceq = gate_C(g_tp.w_eval_inv_n + g_tp.w_eval_inv_p, 0, is_dram) +
          drain_C_(g_tp.w_comp_inv_p3, PCH, 1, 1, g_tp.cell_h_def, is_dram) +
          drain_C_(g_tp.w_comp_inv_n3, NCH, 1, 1, g_tp.cell_h_def, is_dram);
    Req = tr_R_on(g_tp.w_comp_inv_p3, PCH, 1, is_dram);
    tf = Req * Ceq;
    double st3del = horowitz(nextinputtime, tf, VTHCOMPINV, VTHEVALINV, FALL);
    nextinputtime = st3del / (VTHEVALINV);
    power_comparator.readOp.dynamic += 0.5 * Ceq * g_tp.peri_global.Vdd * g_tp.peri_global.Vdd * 4 * A;
    lkgCurrent += cmos_Isub_leakage(g_tp.w_comp_inv_n3, g_tp.w_comp_inv_p3, 1,
                                    inv, is_dram) * 4 * A;
    gatelkgCurrent += cmos_Ig_leakage(g_tp.w_comp_inv_n3, g_tp.w_comp_inv_p3,
                                      1, inv, is_dram) * 4 * A;

    /* Final Inverter (virtual ground driver) discharging compare part */
    double r1 = tr_R_on(g_tp.w_comp_n, NCH, 2, is_dram);
    double r2 = tr_R_on(g_tp.w_eval_inv_n, NCH, 1, is_dram); /* was switch */
    double c2 = (tagbits_) * (drain_C_(g_tp.w_comp_n, NCH, 1, 1,
                                       g_tp.cell_h_def, is_dram) +
                              drain_C_(g_tp.w_comp_n, NCH, 2, 1,
                                       g_tp.cell_h_def, is_dram)) +
        drain_C_(g_tp.w_eval_inv_p, PCH, 1, 1, g_tp.cell_h_def, is_dram) +
        drain_C_(g_tp.w_eval_inv_n, NCH, 1, 1, g_tp.cell_h_def, is_dram);
    double c1 = (tagbits_) * (drain_C_(g_tp.w_comp_n, NCH, 1, 1,
                                       g_tp.cell_h_def, is_dram) +
                              drain_C_(g_tp.w_comp_n, NCH, 2, 1,
                                       g_tp.cell_h_def, is_dram)) +
        drain_C_(g_tp.w_comp_p, PCH, 1, 1, g_tp.cell_h_def, is_dram) +
        gate_C(WmuxdrvNANDn + WmuxdrvNANDp, 0, is_dram);
    power_comparator.readOp.dynamic += 0.5 * c2 * g_tp.peri_global.Vdd * g_tp.peri_global.Vdd * 4 * A;
    power_comparator.readOp.dynamic += c1 * g_tp.peri_global.Vdd * g_tp.peri_global.Vdd *  (A - 1);
    lkgCurrent += cmos_Isub_leakage(g_tp.w_eval_inv_n, g_tp.w_eval_inv_p, 1,
                                    inv, is_dram) * 4 * A;
    lkgCurrent += cmos_Isub_leakage(g_tp.w_comp_n, g_tp.w_comp_n, 1, inv,
                                    is_dram) * 4 * A; // stack factor of 0.2

    gatelkgCurrent += cmos_Ig_leakage(g_tp.w_eval_inv_n, g_tp.w_eval_inv_p, 1,
                                      inv, is_dram) * 4 * A;
    //for gate leakage this equals to a inverter
    gatelkgCurrent += cmos_Ig_leakage(g_tp.w_comp_n, g_tp.w_comp_n, 1, inv,
                                      is_dram) * 4 * A;

    /* time to go to threshold of mux driver */
    double tstep = (r2 * c2 + (r1 + r2) * c1) * log(1.0 / VTHMUXNAND);
    /* take into account non-zero input rise time */
    double m = g_tp.peri_global.Vdd / nextinputtime;
    double Tcomparatorni;

    if ((tstep) <= (0.5*(g_tp.peri_global.Vdd - g_tp.peri_global.Vth) / m)) {
        double a = m;
        double b = 2 * ((g_tp.peri_global.Vdd * VTHEVALINV) -
                        g_tp.peri_global.Vth);
        double c = -2 * (tstep) * (g_tp.peri_global.Vdd -
                                   g_tp.peri_global.Vth) + 1 / m *
            ((g_tp.peri_global.Vdd * VTHEVALINV) - g_tp.peri_global.Vth) *
            ((g_tp.peri_global.Vdd * VTHEVALINV) - g_tp.peri_global.Vth);
        Tcomparatorni = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
    } else {
        Tcomparatorni = (tstep) + (g_tp.peri_global.Vdd +
                                   g_tp.peri_global.Vth) / (2 * m) -
            (g_tp.peri_global.Vdd * VTHEVALINV) / m;
    }
    delay_comparator = Tcomparatorni + st1del + st2del + st3del;
    power_comparator.readOp.leakage = lkgCurrent * g_tp.peri_global.Vdd;
    power_comparator.readOp.gate_leakage = gatelkgCurrent * g_tp.peri_global.Vdd;

    return Tcomparatorni / (1.0 - VTHMUXNAND);;
}



void Mat::compute_power_energy() {
    //for cam and FA, power.readOp is the plain read power, power.searchOp is the associative search related power
    //when search all subarrays and all mats are fully active
    //when plain read/write only one subarray in a single mat is active.

    // add energy consumed in predecoder drivers. This unit is shared by all subarrays in a mat.
    power.readOp.dynamic += r_predec->power.readOp.dynamic +
                            b_mux_predec->power.readOp.dynamic +
                            sa_mux_lev_1_predec->power.readOp.dynamic +
                            sa_mux_lev_2_predec->power.readOp.dynamic;

    // add energy consumed in decoders
    power_row_decoders.readOp.dynamic        = row_dec->power.readOp.dynamic;
    if (!(is_fa || pure_cam))
        power_row_decoders.readOp.dynamic        *= num_subarrays_per_mat;

    // add energy consumed in bitline prechagers, SAs, and bitlines
    if (!(is_fa || pure_cam)) {
        // add energy consumed in bitline prechagers
        power_bl_precharge_eq_drv.readOp.dynamic = bl_precharge_eq_drv->power.readOp.dynamic;
        power_bl_precharge_eq_drv.readOp.dynamic *= num_subarrays_per_mat;

        //Add sense amps energy
        num_sa_subarray = subarray.num_cols / deg_bl_muxing;
        power_sa.readOp.dynamic *= num_sa_subarray * num_subarrays_per_mat ;

        // add energy consumed in bitlines
        //cout<<"bitline power"<<power_bitline.readOp.dynamic<<endl;
        power_bitline.readOp.dynamic *= num_subarrays_per_mat *
            subarray.num_cols;
        power_bitline.writeOp.dynamic *= num_subarrays_per_mat *
            subarray.num_cols;
        //cout<<"bitline power"<<power_bitline.readOp.dynamic<<"subarray"<<num_subarrays_per_mat<<"cols"<<subarray.num_cols<<endl;
        //Add subarray output energy
        power_subarray_out_drv.readOp.dynamic =
            (power_subarray_out_drv.readOp.dynamic + subarray_out_wire->power.readOp.dynamic) * num_do_b_mat;

        power.readOp.dynamic += power_bl_precharge_eq_drv.readOp.dynamic +
                                power_sa.readOp.dynamic +
                                power_bitline.readOp.dynamic +
                                power_subarray_out_drv.readOp.dynamic;

        power.readOp.dynamic += power_row_decoders.readOp.dynamic +
                                bit_mux_dec->power.readOp.dynamic +
                                sa_mux_lev_1_dec->power.readOp.dynamic +
                                sa_mux_lev_2_dec->power.readOp.dynamic +
                                power_comparator.readOp.dynamic;
    }

    else if (is_fa) {
        //for plain read/write only one subarray in a mat is active
        // add energy consumed in bitline prechagers
        power_bl_precharge_eq_drv.readOp.dynamic = bl_precharge_eq_drv->power.readOp.dynamic
                + cam_bl_precharge_eq_drv->power.readOp.dynamic;
        power_bl_precharge_eq_drv.searchOp.dynamic = bl_precharge_eq_drv->power.readOp.dynamic;

        //Add sense amps energy
        num_sa_subarray = (subarray.num_cols_fa_cam +
                           subarray.num_cols_fa_ram) / deg_bl_muxing;
        num_sa_subarray_search = subarray.num_cols_fa_ram / deg_bl_muxing;
        power_sa.searchOp.dynamic = power_sa.readOp.dynamic *
            num_sa_subarray_search;
        power_sa.readOp.dynamic *= num_sa_subarray;


        // add energy consumed in bitlines
        power_bitline.searchOp.dynamic = power_bitline.readOp.dynamic;
        power_bitline.readOp.dynamic *= (subarray.num_cols_fa_cam +
                                         subarray.num_cols_fa_ram);
        power_bitline.writeOp.dynamic *= (subarray.num_cols_fa_cam +
                                          subarray.num_cols_fa_ram);
        power_bitline.searchOp.dynamic *= subarray.num_cols_fa_ram;

        //Add subarray output energy
        power_subarray_out_drv.searchOp.dynamic =
            (power_subarray_out_drv.readOp.dynamic + subarray_out_wire->power.readOp.dynamic) * num_so_b_mat;
        power_subarray_out_drv.readOp.dynamic =
            (power_subarray_out_drv.readOp.dynamic + subarray_out_wire->power.readOp.dynamic) * num_do_b_mat;


        power.readOp.dynamic += power_bl_precharge_eq_drv.readOp.dynamic +
                                power_sa.readOp.dynamic +
                                power_bitline.readOp.dynamic +
                                power_subarray_out_drv.readOp.dynamic;

        power.readOp.dynamic += power_row_decoders.readOp.dynamic +
                                bit_mux_dec->power.readOp.dynamic +
                                sa_mux_lev_1_dec->power.readOp.dynamic +
                                sa_mux_lev_2_dec->power.readOp.dynamic +
                                power_comparator.readOp.dynamic;

        //add energy consumed inside cam
        power_matchline.searchOp.dynamic *= num_subarrays_per_mat;
        power_searchline_precharge = sl_precharge_eq_drv->power;
        power_searchline_precharge.searchOp.dynamic = power_searchline_precharge.readOp.dynamic * num_subarrays_per_mat;
        power_searchline = sl_data_drv->power;
        power_searchline.searchOp.dynamic = power_searchline.readOp.dynamic *
            subarray.num_cols_fa_cam * num_subarrays_per_mat;;
        power_matchline_precharge  = ml_precharge_drv->power;
        power_matchline_precharge.searchOp.dynamic =
            power_matchline_precharge.readOp.dynamic * num_subarrays_per_mat;
        power_ml_to_ram_wl_drv = ml_to_ram_wl_drv->power;
        power_ml_to_ram_wl_drv.searchOp.dynamic =
            ml_to_ram_wl_drv->power.readOp.dynamic;

        power_cam_all_active.searchOp.dynamic = power_matchline.searchOp.dynamic;
        power_cam_all_active.searchOp.dynamic +=
            power_searchline_precharge.searchOp.dynamic;
        power_cam_all_active.searchOp.dynamic +=
            power_searchline.searchOp.dynamic;
        power_cam_all_active.searchOp.dynamic +=
            power_matchline_precharge.searchOp.dynamic;

        power.searchOp.dynamic += power_cam_all_active.searchOp.dynamic;
        //power.searchOp.dynamic += ml_to_ram_wl_drv->power.readOp.dynamic;

    } else {
        // add energy consumed in bitline prechagers
        power_bl_precharge_eq_drv.readOp.dynamic = cam_bl_precharge_eq_drv->power.readOp.dynamic;
        //power_bl_precharge_eq_drv.readOp.dynamic *= num_subarrays_per_mat;
        //power_bl_precharge_eq_drv.searchOp.dynamic = cam_bl_precharge_eq_drv->power.readOp.dynamic;
        //power_bl_precharge_eq_drv.searchOp.dynamic *= num_subarrays_per_mat;

        //Add sense amps energy
        num_sa_subarray = subarray.num_cols_fa_cam / deg_bl_muxing;
        power_sa.readOp.dynamic *= num_sa_subarray;//*num_subarrays_per_mat;
        power_sa.searchOp.dynamic = 0;

        power_bitline.readOp.dynamic *= subarray.num_cols_fa_cam;
        power_bitline.searchOp.dynamic = 0;
        power_bitline.writeOp.dynamic *= subarray.num_cols_fa_cam;

        power_subarray_out_drv.searchOp.dynamic =
            (power_subarray_out_drv.readOp.dynamic + subarray_out_wire->power.readOp.dynamic) * num_so_b_mat;
        power_subarray_out_drv.readOp.dynamic =
            (power_subarray_out_drv.readOp.dynamic + subarray_out_wire->power.readOp.dynamic) * num_do_b_mat;

        power.readOp.dynamic += power_bl_precharge_eq_drv.readOp.dynamic +
                                power_sa.readOp.dynamic +
                                power_bitline.readOp.dynamic +
                                power_subarray_out_drv.readOp.dynamic;

        power.readOp.dynamic += power_row_decoders.readOp.dynamic +
                                bit_mux_dec->power.readOp.dynamic +
                                sa_mux_lev_1_dec->power.readOp.dynamic +
                                sa_mux_lev_2_dec->power.readOp.dynamic +
                                power_comparator.readOp.dynamic;


        ////add energy consumed inside cam
        power_matchline.searchOp.dynamic *= num_subarrays_per_mat;
        power_searchline_precharge = sl_precharge_eq_drv->power;
        power_searchline_precharge.searchOp.dynamic = power_searchline_precharge.readOp.dynamic * num_subarrays_per_mat;
        power_searchline = sl_data_drv->power;
        power_searchline.searchOp.dynamic = power_searchline.readOp.dynamic *
            subarray.num_cols_fa_cam * num_subarrays_per_mat;;
        power_matchline_precharge  = ml_precharge_drv->power;
        power_matchline_precharge.searchOp.dynamic =
            power_matchline_precharge.readOp.dynamic * num_subarrays_per_mat;
        power_ml_to_ram_wl_drv = ml_to_ram_wl_drv->power;
        power_ml_to_ram_wl_drv.searchOp.dynamic =
            ml_to_ram_wl_drv->power.readOp.dynamic;

        power_cam_all_active.searchOp.dynamic =
            power_matchline.searchOp.dynamic;
        power_cam_all_active.searchOp.dynamic +=
            power_searchline_precharge.searchOp.dynamic;
        power_cam_all_active.searchOp.dynamic +=
            power_searchline.searchOp.dynamic;
        power_cam_all_active.searchOp.dynamic +=
            power_matchline_precharge.searchOp.dynamic;

        power.searchOp.dynamic += power_cam_all_active.searchOp.dynamic;
        //power.searchOp.dynamic += ml_to_ram_wl_drv->power.readOp.dynamic;

    }



    // calculate leakage power
    if (!(is_fa || pure_cam)) {
        int number_output_drivers_subarray = num_sa_subarray / (dp.Ndsam_lev_1 * dp.Ndsam_lev_2);

        power_bitline.readOp.leakage            *= subarray.num_rows * subarray.num_cols * num_subarrays_per_mat;
        power_bl_precharge_eq_drv.readOp.leakage = bl_precharge_eq_drv->power.readOp.leakage * num_subarrays_per_mat;
        power_sa.readOp.leakage *= num_sa_subarray * num_subarrays_per_mat *
            (RWP + ERP);

        //num_sa_subarray             = subarray.num_cols / deg_bl_muxing;
        power_subarray_out_drv.readOp.leakage =
            (power_subarray_out_drv.readOp.leakage + subarray_out_wire->power.readOp.leakage) *
            number_output_drivers_subarray * num_subarrays_per_mat * (RWP + ERP);

        power.readOp.leakage += power_bitline.readOp.leakage +
                                power_bl_precharge_eq_drv.readOp.leakage +
                                power_sa.readOp.leakage +
                                power_subarray_out_drv.readOp.leakage;
        //cout<<"leakage"<<power.readOp.leakage<<endl;

        power_comparator.readOp.leakage *= num_do_b_mat * (RWP + ERP);
        power.readOp.leakage += power_comparator.readOp.leakage;

        //cout<<"leakage1"<<power.readOp.leakage<<endl;

        // leakage power
        power_row_decoders.readOp.leakage = row_dec->power.readOp.leakage * subarray.num_rows * num_subarrays_per_mat;
        power_bit_mux_decoders.readOp.leakage      = bit_mux_dec->power.readOp.leakage * deg_bl_muxing;
        power_sa_mux_lev_1_decoders.readOp.leakage = sa_mux_lev_1_dec->power.readOp.leakage * dp.Ndsam_lev_1;
        power_sa_mux_lev_2_decoders.readOp.leakage = sa_mux_lev_2_dec->power.readOp.leakage * dp.Ndsam_lev_2;

        power.readOp.leakage += r_predec->power.readOp.leakage +
                                b_mux_predec->power.readOp.leakage +
                                sa_mux_lev_1_predec->power.readOp.leakage +
                                sa_mux_lev_2_predec->power.readOp.leakage +
                                power_row_decoders.readOp.leakage +
                                power_bit_mux_decoders.readOp.leakage +
                                power_sa_mux_lev_1_decoders.readOp.leakage +
                                power_sa_mux_lev_2_decoders.readOp.leakage;
        //cout<<"leakage2"<<power.readOp.leakage<<endl;

        //++++Below is gate leakage
        power_bitline.readOp.gate_leakage            *= subarray.num_rows * subarray.num_cols * num_subarrays_per_mat;
        power_bl_precharge_eq_drv.readOp.gate_leakage = bl_precharge_eq_drv->power.readOp.gate_leakage * num_subarrays_per_mat;
        power_sa.readOp.gate_leakage *= num_sa_subarray *
            num_subarrays_per_mat * (RWP + ERP);

        //num_sa_subarray             = subarray.num_cols / deg_bl_muxing;
        power_subarray_out_drv.readOp.gate_leakage =
            (power_subarray_out_drv.readOp.gate_leakage + subarray_out_wire->power.readOp.gate_leakage) *
            number_output_drivers_subarray * num_subarrays_per_mat * (RWP + ERP);

        power.readOp.gate_leakage += power_bitline.readOp.gate_leakage +
                                     power_bl_precharge_eq_drv.readOp.gate_leakage +
                                     power_sa.readOp.gate_leakage +
                                     power_subarray_out_drv.readOp.gate_leakage;
        //cout<<"leakage"<<power.readOp.leakage<<endl;

        power_comparator.readOp.gate_leakage *= num_do_b_mat * (RWP + ERP);
        power.readOp.gate_leakage += power_comparator.readOp.gate_leakage;

        //cout<<"leakage1"<<power.readOp.gate_leakage<<endl;

        // gate_leakage power
        power_row_decoders.readOp.gate_leakage = row_dec->power.readOp.gate_leakage * subarray.num_rows * num_subarrays_per_mat;
        power_bit_mux_decoders.readOp.gate_leakage      = bit_mux_dec->power.readOp.gate_leakage * deg_bl_muxing;
        power_sa_mux_lev_1_decoders.readOp.gate_leakage = sa_mux_lev_1_dec->power.readOp.gate_leakage * dp.Ndsam_lev_1;
        power_sa_mux_lev_2_decoders.readOp.gate_leakage = sa_mux_lev_2_dec->power.readOp.gate_leakage * dp.Ndsam_lev_2;

        power.readOp.gate_leakage += r_predec->power.readOp.gate_leakage +
                                     b_mux_predec->power.readOp.gate_leakage +
                                     sa_mux_lev_1_predec->power.readOp.gate_leakage +
                                     sa_mux_lev_2_predec->power.readOp.gate_leakage +
                                     power_row_decoders.readOp.gate_leakage +
                                     power_bit_mux_decoders.readOp.gate_leakage +
                                     power_sa_mux_lev_1_decoders.readOp.gate_leakage +
                                     power_sa_mux_lev_2_decoders.readOp.gate_leakage;
    } else if (is_fa) {
        int number_output_drivers_subarray = num_sa_subarray;// / (dp.Ndsam_lev_1 * dp.Ndsam_lev_2);

        power_bitline.readOp.leakage            *= subarray.num_rows * subarray.num_cols * num_subarrays_per_mat;
        power_bl_precharge_eq_drv.readOp.leakage = bl_precharge_eq_drv->power.readOp.leakage * num_subarrays_per_mat;
        power_bl_precharge_eq_drv.searchOp.leakage = cam_bl_precharge_eq_drv->power.readOp.leakage * num_subarrays_per_mat;
        power_sa.readOp.leakage *= num_sa_subarray * num_subarrays_per_mat *
            (RWP + ERP + SCHP);

        //cout<<"leakage3"<<power.readOp.leakage<<endl;


        power_subarray_out_drv.readOp.leakage =
            (power_subarray_out_drv.readOp.leakage + subarray_out_wire->power.readOp.leakage) *
            number_output_drivers_subarray * num_subarrays_per_mat * (RWP + ERP + SCHP);

        power.readOp.leakage += power_bitline.readOp.leakage +
                                power_bl_precharge_eq_drv.readOp.leakage +
                                power_bl_precharge_eq_drv.searchOp.leakage +
                                power_sa.readOp.leakage +
                                power_subarray_out_drv.readOp.leakage;

        //cout<<"leakage4"<<power.readOp.leakage<<endl;

        // leakage power
        power_row_decoders.readOp.leakage = row_dec->power.readOp.leakage * subarray.num_rows * num_subarrays_per_mat;
        power.readOp.leakage += r_predec->power.readOp.leakage +
                                power_row_decoders.readOp.leakage;

        //cout<<"leakage5"<<power.readOp.leakage<<endl;

        //inside cam
        power_cam_all_active.searchOp.leakage = power_matchline.searchOp.leakage;
        power_cam_all_active.searchOp.leakage +=
            sl_precharge_eq_drv->power.readOp.leakage;
        power_cam_all_active.searchOp.leakage +=
            sl_data_drv->power.readOp.leakage * subarray.num_cols_fa_cam;
        power_cam_all_active.searchOp.leakage +=
            ml_precharge_drv->power.readOp.dynamic;
        power_cam_all_active.searchOp.leakage *=
            num_subarrays_per_mat;

        power.readOp.leakage += power_cam_all_active.searchOp.leakage;

//	  cout<<"leakage6"<<power.readOp.leakage<<endl;

        //+++Below is gate leakage
        power_bitline.readOp.gate_leakage            *= subarray.num_rows * subarray.num_cols * num_subarrays_per_mat;
        power_bl_precharge_eq_drv.readOp.gate_leakage = bl_precharge_eq_drv->power.readOp.gate_leakage * num_subarrays_per_mat;
        power_bl_precharge_eq_drv.searchOp.gate_leakage = cam_bl_precharge_eq_drv->power.readOp.gate_leakage * num_subarrays_per_mat;
        power_sa.readOp.gate_leakage *= num_sa_subarray *
            num_subarrays_per_mat * (RWP + ERP + SCHP);

        //cout<<"leakage3"<<power.readOp.gate_leakage<<endl;


        power_subarray_out_drv.readOp.gate_leakage =
            (power_subarray_out_drv.readOp.gate_leakage + subarray_out_wire->power.readOp.gate_leakage) *
            number_output_drivers_subarray * num_subarrays_per_mat * (RWP + ERP + SCHP);

        power.readOp.gate_leakage += power_bitline.readOp.gate_leakage +
                                     power_bl_precharge_eq_drv.readOp.gate_leakage +
                                     power_bl_precharge_eq_drv.searchOp.gate_leakage +
                                     power_sa.readOp.gate_leakage +
                                     power_subarray_out_drv.readOp.gate_leakage;

        //cout<<"leakage4"<<power.readOp.gate_leakage<<endl;

        // gate_leakage power
        power_row_decoders.readOp.gate_leakage = row_dec->power.readOp.gate_leakage * subarray.num_rows * num_subarrays_per_mat;
        power.readOp.gate_leakage += r_predec->power.readOp.gate_leakage +
                                     power_row_decoders.readOp.gate_leakage;

        //cout<<"leakage5"<<power.readOp.gate_leakage<<endl;

        //inside cam
        power_cam_all_active.searchOp.gate_leakage =
            power_matchline.searchOp.gate_leakage;
        power_cam_all_active.searchOp.gate_leakage +=
            sl_precharge_eq_drv->power.readOp.gate_leakage;
        power_cam_all_active.searchOp.gate_leakage +=
            sl_data_drv->power.readOp.gate_leakage * subarray.num_cols_fa_cam;
        power_cam_all_active.searchOp.gate_leakage +=
            ml_precharge_drv->power.readOp.dynamic;
        power_cam_all_active.searchOp.gate_leakage *= num_subarrays_per_mat;

        power.readOp.gate_leakage += power_cam_all_active.searchOp.gate_leakage;

    } else {
        int number_output_drivers_subarray = num_sa_subarray;// / (dp.Ndsam_lev_1 * dp.Ndsam_lev_2);

        //power_bitline.readOp.leakage            *= subarray.num_rows * subarray.num_cols * num_subarrays_per_mat;
        //power_bl_precharge_eq_drv.readOp.leakage = bl_precharge_eq_drv->power.readOp.leakage * num_subarrays_per_mat;
        power_bl_precharge_eq_drv.searchOp.leakage = cam_bl_precharge_eq_drv->power.readOp.leakage * num_subarrays_per_mat;
        power_sa.readOp.leakage *= num_sa_subarray * num_subarrays_per_mat *
            (RWP + ERP + SCHP);


        power_subarray_out_drv.readOp.leakage =
            (power_subarray_out_drv.readOp.leakage + subarray_out_wire->power.readOp.leakage) *
            number_output_drivers_subarray * num_subarrays_per_mat * (RWP + ERP + SCHP);

        power.readOp.leakage += //power_bitline.readOp.leakage +
            //power_bl_precharge_eq_drv.readOp.leakage +
            power_bl_precharge_eq_drv.searchOp.leakage +
            power_sa.readOp.leakage +
            power_subarray_out_drv.readOp.leakage;

        // leakage power
        power_row_decoders.readOp.leakage = row_dec->power.readOp.leakage *
            subarray.num_rows * num_subarrays_per_mat * (RWP + ERP + EWP);
        power.readOp.leakage += r_predec->power.readOp.leakage +
                                power_row_decoders.readOp.leakage;

        //inside cam
        power_cam_all_active.searchOp.leakage = power_matchline.searchOp.leakage;
        power_cam_all_active.searchOp.leakage +=
            sl_precharge_eq_drv->power.readOp.leakage;
        power_cam_all_active.searchOp.leakage +=
            sl_data_drv->power.readOp.leakage * subarray.num_cols_fa_cam;
        power_cam_all_active.searchOp.leakage +=
            ml_precharge_drv->power.readOp.dynamic;
        power_cam_all_active.searchOp.leakage *= num_subarrays_per_mat;

        power.readOp.leakage += power_cam_all_active.searchOp.leakage;

        //+++Below is gate leakage
        power_bl_precharge_eq_drv.searchOp.gate_leakage = cam_bl_precharge_eq_drv->power.readOp.gate_leakage * num_subarrays_per_mat;
        power_sa.readOp.gate_leakage *= num_sa_subarray *
            num_subarrays_per_mat * (RWP + ERP + SCHP);


        power_subarray_out_drv.readOp.gate_leakage =
            (power_subarray_out_drv.readOp.gate_leakage + subarray_out_wire->power.readOp.gate_leakage) *
            number_output_drivers_subarray * num_subarrays_per_mat * (RWP + ERP + SCHP);

        power.readOp.gate_leakage += //power_bitline.readOp.gate_leakage +
            //power_bl_precharge_eq_drv.readOp.gate_leakage +
            power_bl_precharge_eq_drv.searchOp.gate_leakage +
            power_sa.readOp.gate_leakage +
            power_subarray_out_drv.readOp.gate_leakage;

        // gate_leakage power
        power_row_decoders.readOp.gate_leakage =
            row_dec->power.readOp.gate_leakage * subarray.num_rows *
            num_subarrays_per_mat * (RWP + ERP + EWP);
        power.readOp.gate_leakage += r_predec->power.readOp.gate_leakage +
                                     power_row_decoders.readOp.gate_leakage;

        //inside cam
        power_cam_all_active.searchOp.gate_leakage =
            power_matchline.searchOp.gate_leakage;
        power_cam_all_active.searchOp.gate_leakage +=
            sl_precharge_eq_drv->power.readOp.gate_leakage;
        power_cam_all_active.searchOp.gate_leakage +=
            sl_data_drv->power.readOp.gate_leakage * subarray.num_cols_fa_cam;
        power_cam_all_active.searchOp.gate_leakage +=
            ml_precharge_drv->power.readOp.dynamic;
        power_cam_all_active.searchOp.gate_leakage *=
            num_subarrays_per_mat;

        power.readOp.gate_leakage += power_cam_all_active.searchOp.gate_leakage;
    }
}

