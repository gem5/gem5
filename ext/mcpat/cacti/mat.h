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



#ifndef __MAT_H__
#define __MAT_H__

#include "component.h"
#include "decoder.h"
#include "subarray.h"
#include "wire.h"

class Mat : public Component {
public:
    Mat(const DynamicParameter & dyn_p);
    ~Mat();
    double compute_delays(double inrisetime);  // return outrisetime
    void compute_power_energy();

    const DynamicParameter & dp;

    // TODO: clean up pointers and powerDefs below
    Decoder * row_dec;
    Decoder * bit_mux_dec;
    Decoder * sa_mux_lev_1_dec;
    Decoder * sa_mux_lev_2_dec;
    PredecBlk * dummy_way_sel_predec_blk1;
    PredecBlk * dummy_way_sel_predec_blk2;
    PredecBlkDrv * way_sel_drv1;
    PredecBlkDrv * dummy_way_sel_predec_blk_drv2;

    Predec * r_predec;
    Predec * b_mux_predec;
    Predec * sa_mux_lev_1_predec;
    Predec * sa_mux_lev_2_predec;

    Wire   * subarray_out_wire;
    Driver * bl_precharge_eq_drv;
    Driver * cam_bl_precharge_eq_drv;//bitline pre-charge circuit is separated for CAM and RAM arrays.
    Driver * ml_precharge_drv;//matchline prechange driver
    Driver * sl_precharge_eq_drv;//searchline prechage driver
    Driver * sl_data_drv;//search line data driver
    Driver * ml_to_ram_wl_drv;//search line data driver


    powerDef power_row_decoders;
    powerDef power_bit_mux_decoders;
    powerDef power_sa_mux_lev_1_decoders;
    powerDef power_sa_mux_lev_2_decoders;
    powerDef power_fa_cam;  // TODO: leakage power is not computed yet
    powerDef power_bl_precharge_eq_drv;
    powerDef power_subarray_out_drv;
    powerDef power_cam_all_active;
    powerDef power_searchline_precharge;
    powerDef power_matchline_precharge;
    powerDef power_ml_to_ram_wl_drv;

    double   delay_fa_tag, delay_cam;
    double   delay_before_decoder;
    double   delay_bitline;
    double   delay_wl_reset;
    double   delay_bl_restore;

    double   delay_searchline;
    double   delay_matchchline;
    double   delay_cam_sl_restore;
    double   delay_cam_ml_reset;
    double   delay_fa_ram_wl;

    double   delay_hit_miss_reset;
    double   delay_hit_miss;

    Subarray subarray;
    powerDef power_bitline, power_searchline, power_matchline;
    double   per_bitline_read_energy;
    int      deg_bl_muxing;
    int      num_act_mats_hor_dir;
    double   delay_writeback;
    Area     cell, cam_cell;
    bool     is_dram, is_fa, pure_cam, camFlag;
    int      num_mats;
    powerDef power_sa;
    double   delay_sa;
    double   leak_power_sense_amps_closed_page_state;
    double   leak_power_sense_amps_open_page_state;
    double   delay_subarray_out_drv;
    double   delay_subarray_out_drv_htree;
    double   delay_comparator;
    powerDef power_comparator;
    int      num_do_b_mat;
    int      num_so_b_mat;
    int      num_sa_subarray;
    int      num_sa_subarray_search;
    double   C_bl;

    uint32_t num_subarrays_per_mat;  // the number of subarrays in a mat
    uint32_t num_subarrays_per_row;  // the number of subarrays in a row of a mat


private:
    double compute_bit_mux_sa_precharge_sa_mux_wr_drv_wr_mux_h();
    double width_write_driver_or_write_mux();
    double compute_comparators_height(int tagbits, int number_ways_in_mat, double subarray_mem_cell_area_w);
    double compute_cam_delay(double inrisetime);
    double compute_bitline_delay(double inrisetime);
    double compute_sa_delay(double inrisetime);
    double compute_subarray_out_drv(double inrisetime);
    double compute_comparator_delay(double inrisetime);

    int RWP;
    int ERP;
    int EWP;
    int SCHP;
};



#endif
