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

#include "subarray.h"

Subarray::Subarray(const DynamicParameter & dp_, bool is_fa_):
        dp(dp_), num_rows(dp.num_r_subarray), num_cols(dp.num_c_subarray),
        num_cols_fa_cam(dp.tag_num_c_subarray), num_cols_fa_ram(dp.data_num_c_subarray),
        cell(dp.cell), cam_cell(dp.cam_cell), is_fa(is_fa_) {
    //num_cols=7;
    //cout<<"num_cols ="<< num_cols <<endl;
    if (!(is_fa || dp.pure_cam)) {
        // ECC overhead
        num_cols += (g_ip->add_ecc_b_ ? (int)ceil(num_cols /
                                                  num_bits_per_ecc_b_) : 0);
        uint32_t ram_num_cells_wl_stitching =
            (dp.ram_cell_tech_type == lp_dram)   ? dram_num_cells_wl_stitching_ :
            (dp.ram_cell_tech_type == comm_dram) ? comm_dram_num_cells_wl_stitching_ : sram_num_cells_wl_stitching_;

        area.h = cell.h * num_rows;

        area.w = cell.w * num_cols +
                 ceil(num_cols / ram_num_cells_wl_stitching) * g_tp.ram_wl_stitching_overhead_;  // stitching overhead
    } else { //cam fa

        //should not add dummy row here since the dummy row do not need decoder
        if (is_fa) { // fully associative cache
            num_cols_fa_cam  += g_ip->add_ecc_b_ ? (int)ceil(num_cols_fa_cam / num_bits_per_ecc_b_) : 0;
            num_cols_fa_ram  += (g_ip->add_ecc_b_ ? (int)ceil(num_cols_fa_ram / num_bits_per_ecc_b_) : 0);
            num_cols = num_cols_fa_cam + num_cols_fa_ram;
        } else {
            num_cols_fa_cam  += g_ip->add_ecc_b_ ? (int)ceil(num_cols_fa_cam / num_bits_per_ecc_b_) : 0;
            num_cols_fa_ram  = 0;
            num_cols = num_cols_fa_cam;
        }

        area.h = cam_cell.h * (num_rows + 1);//height of subarray is decided by CAM array. blank space in sram array are filled with dummy cells
        area.w = cam_cell.w * num_cols_fa_cam + cell.w * num_cols_fa_ram
            + ceil((num_cols_fa_cam + num_cols_fa_ram) /
                   sram_num_cells_wl_stitching_) *
            g_tp.ram_wl_stitching_overhead_
            //the overhead for the NAND gate to connect the two halves
            + 16 * g_tp.wire_local.pitch
            //the overhead for the drivers from matchline to wordline of RAM
            + 128 * g_tp.wire_local.pitch;
    }

    assert(area.h > 0);
    assert(area.w > 0);
    compute_C();
}



Subarray::~Subarray() {
}



double Subarray::get_total_cell_area() {
//  return (is_fa==false? cell.get_area() * num_rows * num_cols
//		  //: cam_cell.h*(num_rows+1)*(num_cols_fa_cam + sram_cell.get_area()*num_cols_fa_ram));
//		  : cam_cell.get_area()*(num_rows+1)*(num_cols_fa_cam + num_cols_fa_ram));
//		  //: cam_cell.get_area()*(num_rows+1)*num_cols_fa_cam + sram_cell.get_area()*(num_rows+1)*num_cols_fa_ram);//for FA, this area does not include the dummy cells in SRAM arrays.

    if (!(is_fa || dp.pure_cam))
        return (cell.get_area() * num_rows * num_cols);
    else if (is_fa) {
        //for FA, this area includes the dummy cells in SRAM arrays.
        //return (cam_cell.get_area()*(num_rows+1)*(num_cols_fa_cam + num_cols_fa_ram));
        //cout<<"diff" <<cam_cell.get_area()*(num_rows+1)*(num_cols_fa_cam + num_cols_fa_ram)- cam_cell.h*(num_rows+1)*(cam_cell.w*num_cols_fa_cam + cell.w*num_cols_fa_ram)<<endl;
        return (cam_cell.h * (num_rows + 1) *
                (cam_cell.w*num_cols_fa_cam + cell.w*num_cols_fa_ram));
    } else {
        return (cam_cell.get_area() * (num_rows + 1) * num_cols_fa_cam );
    }


}



void Subarray::compute_C() {
    double c_w_metal = cell.w * g_tp.wire_local.C_per_um;
    double r_w_metal = cell.w * g_tp.wire_local.R_per_um;
    double C_b_metal = cell.h * g_tp.wire_local.C_per_um;
    double C_b_row_drain_C;

    if (dp.is_dram) {
        C_wl = (gate_C_pass(g_tp.dram.cell_a_w, g_tp.dram.b_w, true, true) + c_w_metal) * num_cols;

        if (dp.ram_cell_tech_type == comm_dram) {
            C_bl = num_rows * C_b_metal;
        } else {
            C_b_row_drain_C = drain_C_(g_tp.dram.cell_a_w, NCH, 1, 0, cell.w, true, true) / 2.0;  // due to shared contact
            C_bl = num_rows * (C_b_row_drain_C + C_b_metal);
        }
    } else {
        if (!(is_fa || dp.pure_cam)) {
            C_wl = (gate_C_pass(g_tp.sram.cell_a_w,
                                (g_tp.sram.b_w - 2 * g_tp.sram.cell_a_w) / 2.0,
                                false, true) * 2 +
                    c_w_metal) * num_cols;
            C_b_row_drain_C = drain_C_(g_tp.sram.cell_a_w, NCH, 1, 0, cell.w, false, true) / 2.0;  // due to shared contact
            C_bl = num_rows * (C_b_row_drain_C + C_b_metal);
        } else {
            //Following is wordline not matchline
            //CAM portion
            c_w_metal = cam_cell.w * g_tp.wire_local.C_per_um;
            r_w_metal = cam_cell.w * g_tp.wire_local.R_per_um;
            C_wl_cam = (gate_C_pass(g_tp.cam.cell_a_w,
                                    (g_tp.cam.b_w - 2 * g_tp.cam.cell_a_w) /
                                    2.0, false, true) * 2 +
                        c_w_metal) * num_cols_fa_cam;
            R_wl_cam = (r_w_metal) * num_cols_fa_cam;

            if (!dp.pure_cam) {
                //RAM portion
                c_w_metal = cell.w * g_tp.wire_local.C_per_um;
                r_w_metal = cell.w * g_tp.wire_local.R_per_um;
                C_wl_ram = (gate_C_pass(g_tp.sram.cell_a_w,
                                        (g_tp.sram.b_w - 2 *
                                         g_tp.sram.cell_a_w) / 2.0, false,
                                        true) * 2 +
                            c_w_metal) * num_cols_fa_ram;
                R_wl_ram = (r_w_metal) * num_cols_fa_ram;
            } else {
                C_wl_ram = R_wl_ram = 0;
            }
            C_wl = C_wl_cam + C_wl_ram;
            C_wl += (16 + 128) * g_tp.wire_local.pitch *
                g_tp.wire_local.C_per_um;

            R_wl = R_wl_cam + R_wl_ram;
            R_wl += (16 + 128) * g_tp.wire_local.pitch *
                g_tp.wire_local.R_per_um;

            //there are two ways to write to a FA,
            //1) Write to CAM array then force a match on match line to active the corresponding wordline in RAM;
            //2) using separate wordline for read/write and search in RAM.
            //We are using the second approach.

            //Bitline CAM portion This is bitline not searchline. We assume no sharing between bitline and searchline according to SUN's implementations.
            C_b_metal = cam_cell.h * g_tp.wire_local.C_per_um;
            C_b_row_drain_C = drain_C_(g_tp.cam.cell_a_w, NCH, 1, 0, cam_cell.w, false, true) / 2.0;  // due to shared contact
            C_bl_cam = (num_rows + 1) * (C_b_row_drain_C + C_b_metal);
            //height of subarray is decided by CAM array. blank space in sram array are filled with dummy cells
            C_b_row_drain_C = drain_C_(g_tp.sram.cell_a_w, NCH, 1, 0, cell.w, false, true) / 2.0;  // due to shared contact
            C_bl = (num_rows + 1) * (C_b_row_drain_C + C_b_metal);

        }
    }
}


