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



#include <iomanip>
#include <iostream>
#include <string>

#include "area.h"
#include "parameter.h"

using namespace std;


InputParameter * g_ip;
TechnologyParameter g_tp;



void TechnologyParameter::DeviceType::display(uint32_t indent) {
    string indent_str(indent, ' ');

    cout << indent_str << "C_g_ideal = " << setw(12) << C_g_ideal << " F/um" << endl;
    cout << indent_str << "C_fringe  = " << setw(12) << C_fringe  << " F/um" << endl;
    cout << indent_str << "C_overlap = " << setw(12) << C_overlap << " F/um" << endl;
    cout << indent_str << "C_junc    = " << setw(12) << C_junc    << " F/um^2" << endl;
    cout << indent_str << "l_phy     = " << setw(12) << l_phy     << " um" << endl;
    cout << indent_str << "l_elec    = " << setw(12) << l_elec    << " um" << endl;
    cout << indent_str << "R_nch_on  = " << setw(12) << R_nch_on  << " ohm-um" << endl;
    cout << indent_str << "R_pch_on  = " << setw(12) << R_pch_on  << " ohm-um" << endl;
    cout << indent_str << "Vdd       = " << setw(12) << Vdd       << " V" << endl;
    cout << indent_str << "Vth       = " << setw(12) << Vth       << " V" << endl;
    cout << indent_str << "I_on_n    = " << setw(12) << I_on_n    << " A/um" << endl;
    cout << indent_str << "I_on_p    = " << setw(12) << I_on_p    << " A/um" << endl;
    cout << indent_str << "I_off_n   = " << setw(12) << I_off_n   << " A/um" << endl;
    cout << indent_str << "I_off_p   = " << setw(12) << I_off_p   << " A/um" << endl;
    cout << indent_str << "C_ox      = " << setw(12) << C_ox      << " F/um^2" << endl;
    cout << indent_str << "t_ox      = " << setw(12) << t_ox      << " um" << endl;
    cout << indent_str << "n_to_p_eff_curr_drv_ratio = " << n_to_p_eff_curr_drv_ratio << endl;
}



void TechnologyParameter::InterconnectType::display(uint32_t indent) {
    string indent_str(indent, ' ');

    cout << indent_str << "pitch    = " << setw(12) << pitch    << " um" << endl;
    cout << indent_str << "R_per_um = " << setw(12) << R_per_um << " ohm/um" << endl;
    cout << indent_str << "C_per_um = " << setw(12) << C_per_um << " F/um" << endl;
}

void TechnologyParameter::ScalingFactor::display(uint32_t indent) {
    string indent_str(indent, ' ');

    cout << indent_str << "logic_scaling_co_eff    = " << setw(12) << logic_scaling_co_eff << endl;
    cout << indent_str << "curr_core_tx_density = " << setw(12) << core_tx_density << " # of tx/um^2" << endl;
}

void TechnologyParameter::MemoryType::display(uint32_t indent) {
    string indent_str(indent, ' ');

    cout << indent_str << "b_w         = " << setw(12) << b_w << " um" << endl;
    cout << indent_str << "b_h         = " << setw(12) << b_h << " um" << endl;
    cout << indent_str << "cell_a_w    = " << setw(12) << cell_a_w << " um" << endl;
    cout << indent_str << "cell_pmos_w = " << setw(12) << cell_pmos_w << " um" << endl;
    cout << indent_str << "cell_nmos_w = " << setw(12) << cell_nmos_w << " um" << endl;
    cout << indent_str << "Vbitpre     = " << setw(12) << Vbitpre << " V" << endl;
}



void TechnologyParameter::display(uint32_t indent) {
    string indent_str(indent, ' ');

    cout << indent_str << "ram_wl_stitching_overhead_ = " << setw(12) << ram_wl_stitching_overhead_ << " um" << endl;
    cout << indent_str << "min_w_nmos_                = " << setw(12) << min_w_nmos_                << " um" << endl;
    cout << indent_str << "max_w_nmos_                = " << setw(12) << max_w_nmos_                << " um" << endl;
    cout << indent_str << "unit_len_wire_del          = " << setw(12) << unit_len_wire_del          << " s/um^2" << endl;
    cout << indent_str << "FO4                        = " << setw(12) << FO4                        << " s" << endl;
    cout << indent_str << "kinv                       = " << setw(12) << kinv                       << " s" << endl;
    cout << indent_str << "vpp                        = " << setw(12) << vpp                        << " V" << endl;
    cout << indent_str << "w_sense_en                 = " << setw(12) << w_sense_en                 << " um" << endl;
    cout << indent_str << "w_sense_n                  = " << setw(12) << w_sense_n                  << " um" << endl;
    cout << indent_str << "w_sense_p                  = " << setw(12) << w_sense_p                  << " um" << endl;
    cout << indent_str << "w_iso                      = " << setw(12) << w_iso                      << " um" << endl;
    cout << indent_str << "w_poly_contact             = " << setw(12) << w_poly_contact             << " um" << endl;
    cout << indent_str << "spacing_poly_to_poly       = " << setw(12) << spacing_poly_to_poly       << " um" << endl;
    cout << indent_str << "spacing_poly_to_contact    = " << setw(12) << spacing_poly_to_contact    << " um" << endl;
    cout << endl;
    cout << indent_str << "w_comp_inv_p1              = " << setw(12) << w_comp_inv_p1 << " um" << endl;
    cout << indent_str << "w_comp_inv_p2              = " << setw(12) << w_comp_inv_p2 << " um" << endl;
    cout << indent_str << "w_comp_inv_p3              = " << setw(12) << w_comp_inv_p3 << " um" << endl;
    cout << indent_str << "w_comp_inv_n1              = " << setw(12) << w_comp_inv_n1 << " um" << endl;
    cout << indent_str << "w_comp_inv_n2              = " << setw(12) << w_comp_inv_n2 << " um" << endl;
    cout << indent_str << "w_comp_inv_n3              = " << setw(12) << w_comp_inv_n3 << " um" << endl;
    cout << indent_str << "w_eval_inv_p               = " << setw(12) << w_eval_inv_p  << " um" << endl;
    cout << indent_str << "w_eval_inv_n               = " << setw(12) << w_eval_inv_n  << " um" << endl;
    cout << indent_str << "w_comp_n                   = " << setw(12) << w_comp_n      << " um" << endl;
    cout << indent_str << "w_comp_p                   = " << setw(12) << w_comp_p      << " um" << endl;
    cout << endl;
    cout << indent_str << "dram_cell_I_on             = " << setw(12) << dram_cell_I_on << " A/um" << endl;
    cout << indent_str << "dram_cell_Vdd              = " << setw(12) << dram_cell_Vdd  << " V" << endl;
    cout << indent_str << "dram_cell_I_off_worst_case_len_temp = " << setw(12) << dram_cell_I_off_worst_case_len_temp << " A/um" << endl;
    cout << indent_str << "dram_cell_C                = " << setw(12) << dram_cell_C               << " F" << endl;
    cout << indent_str << "gm_sense_amp_latch         = " << setw(12) << gm_sense_amp_latch        << " F/s" << endl;
    cout << endl;
    cout << indent_str << "w_nmos_b_mux               = " << setw(12) << w_nmos_b_mux              << " um" << endl;
    cout << indent_str << "w_nmos_sa_mux              = " << setw(12) << w_nmos_sa_mux             << " um" << endl;
    cout << indent_str << "w_pmos_bl_precharge        = " << setw(12) << w_pmos_bl_precharge       << " um" << endl;
    cout << indent_str << "w_pmos_bl_eq               = " << setw(12) << w_pmos_bl_eq              << " um" << endl;
    cout << indent_str << "MIN_GAP_BET_P_AND_N_DIFFS  = " << setw(12) << MIN_GAP_BET_P_AND_N_DIFFS << " um" << endl;
    cout << indent_str << "HPOWERRAIL                 = " << setw(12) << HPOWERRAIL                << " um" << endl;
    cout << indent_str << "cell_h_def                 = " << setw(12) << cell_h_def                << " um" << endl;

    cout << endl;
    cout << indent_str << "SRAM cell transistor: " << endl;
    sram_cell.display(indent + 2);

    cout << endl;
    cout << indent_str << "DRAM access transistor: " << endl;
    dram_acc.display(indent + 2);

    cout << endl;
    cout << indent_str << "DRAM wordline transistor: " << endl;
    dram_wl.display(indent + 2);

    cout << endl;
    cout << indent_str << "peripheral global transistor: " << endl;
    peri_global.display(indent + 2);

    cout << endl;
    cout << indent_str << "wire local" << endl;
    wire_local.display(indent + 2);

    cout << endl;
    cout << indent_str << "wire inside mat" << endl;
    wire_inside_mat.display(indent + 2);

    cout << endl;
    cout << indent_str << "wire outside mat" << endl;
    wire_outside_mat.display(indent + 2);

    cout << endl;
    cout << indent_str << "SRAM" << endl;
    sram.display(indent + 2);

    cout << endl;
    cout << indent_str << "DRAM" << endl;
    dram.display(indent + 2);
}


DynamicParameter::DynamicParameter():
        use_inp_params(0), cell(), is_valid(true) {
}



DynamicParameter::DynamicParameter(
    bool is_tag_,
    int pure_ram_,
    int pure_cam_,
    double Nspd_,
    unsigned int Ndwl_,
    unsigned int Ndbl_,
    unsigned int Ndcm_,
    unsigned int Ndsam_lev_1_,
    unsigned int Ndsam_lev_2_,
    bool is_main_mem_):
    is_tag(is_tag_), pure_ram(pure_ram_), pure_cam(pure_cam_), tagbits(0),
    Nspd(Nspd_), Ndwl(Ndwl_), Ndbl(Ndbl_), Ndcm(Ndcm_),
    Ndsam_lev_1(Ndsam_lev_1_), Ndsam_lev_2(Ndsam_lev_2_),
    number_way_select_signals_mat(0), V_b_sense(0), use_inp_params(0),
    is_main_mem(is_main_mem_), cell(), is_valid(false) {
    ram_cell_tech_type = (is_tag) ? g_ip->tag_arr_ram_cell_tech_type : g_ip->data_arr_ram_cell_tech_type;
    is_dram            = ((ram_cell_tech_type == lp_dram) || (ram_cell_tech_type == comm_dram));

    unsigned int capacity_per_die = g_ip->cache_sz / NUMBER_STACKED_DIE_LAYERS;  // capacity per stacked die layer
    const TechnologyParameter::InterconnectType & wire_local = g_tp.wire_local;
    fully_assoc = (g_ip->fully_assoc) ? true : false;

    // fully-assocative cache -- ref: CACTi 2.0 report
    if (fully_assoc || pure_cam) {
        if (Ndwl != 1 ||            //Ndwl is fixed to 1 for FA
                Ndcm != 1 ||            //Ndcm is fixed to 1 for FA
                Nspd < 1 || Nspd > 1 || //Nspd is fixed to 1 for FA
                Ndsam_lev_1 != 1 ||     //Ndsam_lev_1 is fixed to one
                Ndsam_lev_2 != 1 ||     //Ndsam_lev_2 is fixed to one
                Ndbl < 2) {
            return;
        }
    }

    if ((is_dram) && (!is_tag) && (Ndcm > 1)) {
        return;  // For a DRAM array, each bitline has its own sense-amp
    }

    // If it's not an FA tag/data array, Ndwl should be at least two and Ndbl should be
    // at least two because an array is assumed to have at least one mat. And a mat
    // is formed out of two horizontal subarrays and two vertical subarrays
    if (fully_assoc == false && (Ndwl < 1 || Ndbl < 1)) {
        return;
    }

    //***********compute row, col of an subarray
    if (!(fully_assoc || pure_cam)) {
        //Not fully_asso nor cam
        // if data array, let tagbits = 0
        if (is_tag) {
            if (g_ip->specific_tag) {
                tagbits = g_ip->tag_w;
            } else {
                tagbits = ADDRESS_BITS + EXTRA_TAG_BITS - _log2(capacity_per_die) +
                          _log2(g_ip->tag_assoc * 2 - 1) - _log2(g_ip->nbanks);

            }
            tagbits = (((tagbits + 3) >> 2) << 2);

            num_r_subarray = (int)ceil(capacity_per_die / (g_ip->nbanks *
                                       g_ip->block_sz * g_ip->tag_assoc * Ndbl * Nspd));// + EPSILON);
            num_c_subarray = (int)ceil((tagbits * g_ip->tag_assoc * Nspd / Ndwl));// + EPSILON);
            //burst_length = 1;
        } else {
            num_r_subarray = (int)ceil(capacity_per_die / (g_ip->nbanks *
                                       g_ip->block_sz * g_ip->data_assoc * Ndbl * Nspd));// + EPSILON);
            num_c_subarray = (int)ceil((8 * g_ip->block_sz * g_ip->data_assoc * Nspd / Ndwl));// + EPSILON); + EPSILON);
            // burst_length = g_ip->block_sz * 8 / g_ip->out_w;
        }

        if (num_r_subarray < MINSUBARRAYROWS) return;
        if (num_r_subarray == 0) return;
        if (num_r_subarray > MAXSUBARRAYROWS) return;
        if (num_c_subarray < MINSUBARRAYCOLS) return;
        if (num_c_subarray > MAXSUBARRAYCOLS) return;

    }

    else {//either fully-asso or cam
        if (pure_cam) {
            if (g_ip->specific_tag) {
                tagbits = int(ceil(g_ip->tag_w / 8.0) * 8);
            } else {
                tagbits = int(ceil((ADDRESS_BITS + EXTRA_TAG_BITS) / 8.0) * 8);
//			  cout<<"Pure CAM needs tag width to be specified"<<endl;
//			  exit(0);
            }
            //tagbits = (((tagbits + 3) >> 2) << 2);

            //TODO: error check input of tagbits and blocksize
            //TODO: for pure CAM, g_ip->block should be number of entries.
            tag_num_r_subarray = (int)ceil(capacity_per_die /
                                           (g_ip->nbanks * tagbits / 8.0 * Ndbl));
            //tag_num_c_subarray = (int)(tagbits  + EPSILON);
            tag_num_c_subarray = tagbits;
            if (tag_num_r_subarray == 0) return;
            if (tag_num_r_subarray > MAXSUBARRAYROWS) return;
            if (tag_num_c_subarray < MINSUBARRAYCOLS) return;
            if (tag_num_c_subarray > MAXSUBARRAYCOLS) return;
            num_r_subarray = tag_num_r_subarray;
        } else { //fully associative
            if (g_ip->specific_tag) {
                tagbits = g_ip->tag_w;
            } else {
                tagbits = ADDRESS_BITS + EXTRA_TAG_BITS - _log2(g_ip->block_sz);//TODO: should be the page_offset=log2(page size), but this info is not avail with CACTI, for McPAT this is no problem.
            }
            tagbits = (((tagbits + 3) >> 2) << 2);

            tag_num_r_subarray = (int)(capacity_per_die /
                                       (g_ip->nbanks * g_ip->block_sz * Ndbl));
            tag_num_c_subarray = (int)ceil((tagbits * Nspd / Ndwl));// + EPSILON);
            if (tag_num_r_subarray == 0) return;
            if (tag_num_r_subarray > MAXSUBARRAYROWS) return;
            if (tag_num_c_subarray < MINSUBARRAYCOLS) return;
            if (tag_num_c_subarray > MAXSUBARRAYCOLS) return;

            data_num_r_subarray = tag_num_r_subarray;
            data_num_c_subarray = 8 * g_ip->block_sz;
            if (data_num_r_subarray == 0) return;
            if (data_num_r_subarray > MAXSUBARRAYROWS) return;
            if (data_num_c_subarray < MINSUBARRAYCOLS) return;
            if (data_num_c_subarray > MAXSUBARRAYCOLS) return;
            num_r_subarray = tag_num_r_subarray;
        }
    }

    num_subarrays = Ndwl * Ndbl;
    //****************end of computation of row, col of an subarray

    // calculate wire parameters
    if (fully_assoc || pure_cam) {
        cam_cell.h = g_tp.cam.b_h + 2 * wire_local.pitch *
            (g_ip->num_rw_ports - 1 + g_ip->num_rd_ports + g_ip->num_wr_ports)
            + 2 * wire_local.pitch * (g_ip->num_search_ports - 1) +
            wire_local.pitch * g_ip->num_se_rd_ports;
        cam_cell.w = g_tp.cam.b_w + 2 * wire_local.pitch *
            (g_ip->num_rw_ports - 1 + g_ip->num_rd_ports + g_ip->num_wr_ports)
            + 2 * wire_local.pitch * (g_ip->num_search_ports - 1) +
            wire_local.pitch * g_ip->num_se_rd_ports;

        cell.h = g_tp.sram.b_h + 2 * wire_local.pitch *
            (g_ip->num_wr_ports + g_ip->num_rw_ports - 1 + g_ip->num_rd_ports)
            + 2 * wire_local.pitch * (g_ip->num_search_ports - 1);
        cell.w = g_tp.sram.b_w + 2 * wire_local.pitch *
            (g_ip->num_rw_ports - 1 + (g_ip->num_rd_ports -
                                       g_ip->num_se_rd_ports)
             + g_ip->num_wr_ports) + g_tp.wire_local.pitch *
            g_ip->num_se_rd_ports + 2 * wire_local.pitch *
            (g_ip->num_search_ports - 1);
    } else {
        if (is_tag) {
            cell.h = g_tp.sram.b_h + 2 * wire_local.pitch * (g_ip->num_rw_ports - 1 + g_ip->num_rd_ports +
                     g_ip->num_wr_ports);
            cell.w = g_tp.sram.b_w + 2 * wire_local.pitch * (g_ip->num_rw_ports - 1 + g_ip->num_wr_ports +
                     (g_ip->num_rd_ports - g_ip->num_se_rd_ports)) +
                     wire_local.pitch * g_ip->num_se_rd_ports;
        } else {
            if (is_dram) {
                cell.h = g_tp.dram.b_h;
                cell.w = g_tp.dram.b_w;
            } else {
                cell.h = g_tp.sram.b_h + 2 * wire_local.pitch * (g_ip->num_wr_ports +
                         g_ip->num_rw_ports - 1 + g_ip->num_rd_ports);
                cell.w = g_tp.sram.b_w + 2 * wire_local.pitch * (g_ip->num_rw_ports - 1 +
                         (g_ip->num_rd_ports - g_ip->num_se_rd_ports) +
                         g_ip->num_wr_ports) + g_tp.wire_local.pitch * g_ip->num_se_rd_ports;
            }
        }
    }

    double c_b_metal = cell.h * wire_local.C_per_um;
    double C_bl;

    if (!(fully_assoc || pure_cam)) {
        if (is_dram) {
            deg_bl_muxing = 1;
            if (ram_cell_tech_type == comm_dram) {
                C_bl  = num_r_subarray * c_b_metal;
                V_b_sense = (g_tp.dram_cell_Vdd / 2) * g_tp.dram_cell_C /
                    (g_tp.dram_cell_C + C_bl);
                if (V_b_sense < VBITSENSEMIN) {
                    return;
                }
                V_b_sense = VBITSENSEMIN;  // in any case, we fix sense amp input signal to a constant value
                dram_refresh_period = 64e-3;
            } else {
                double Cbitrow_drain_cap = drain_C_(g_tp.dram.cell_a_w, NCH, 1, 0, cell.w, true, true) / 2.0;
                C_bl  = num_r_subarray * (Cbitrow_drain_cap + c_b_metal);
                V_b_sense = (g_tp.dram_cell_Vdd / 2) * g_tp.dram_cell_C /
                    (g_tp.dram_cell_C + C_bl);

                if (V_b_sense < VBITSENSEMIN) {
                    return; //Sense amp input signal is smaller that minimum allowable sense amp input signal
                }
                V_b_sense = VBITSENSEMIN; // in any case, we fix sense amp input signal to a constant value
                //v_storage_worst = g_tp.dram_cell_Vdd / 2 - VBITSENSEMIN * (g_tp.dram_cell_C + C_bl) / g_tp.dram_cell_C;
                //dram_refresh_period = 1.1 * g_tp.dram_cell_C * v_storage_worst / g_tp.dram_cell_I_off_worst_case_len_temp;
                dram_refresh_period = 0.9 * g_tp.dram_cell_C * VDD_STORAGE_LOSS_FRACTION_WORST * g_tp.dram_cell_Vdd / g_tp.dram_cell_I_off_worst_case_len_temp;
            }
        } else { //SRAM
            V_b_sense = (0.05 * g_tp.sram_cell.Vdd > VBITSENSEMIN) ? 0.05 * g_tp.sram_cell.Vdd : VBITSENSEMIN;
            deg_bl_muxing = Ndcm;
            // "/ 2.0" below is due to the fact that two adjacent access transistors share drain
            // contacts in a physical layout
            double Cbitrow_drain_cap = drain_C_(g_tp.sram.cell_a_w, NCH, 1, 0, cell.w, false, true) / 2.0;
            C_bl = num_r_subarray * (Cbitrow_drain_cap + c_b_metal);
            dram_refresh_period = 0;
        }
    } else {
        c_b_metal = cam_cell.h * wire_local.C_per_um;//IBM and SUN design, SRAM array uses dummy cells to fill the blank space due to mismatch on CAM-RAM
        V_b_sense = (0.05 * g_tp.sram_cell.Vdd > VBITSENSEMIN) ? 0.05 * g_tp.sram_cell.Vdd : VBITSENSEMIN;
        deg_bl_muxing = 1;//FA fix as 1
        // "/ 2.0" below is due to the fact that two adjacent access transistors share drain
        // contacts in a physical layout
        double Cbitrow_drain_cap = drain_C_(g_tp.cam.cell_a_w, NCH, 1, 0, cam_cell.w, false, true) / 2.0;//TODO: comment out these two lines
        C_bl = num_r_subarray * (Cbitrow_drain_cap + c_b_metal);
        dram_refresh_period = 0;
    }


    // do/di: data in/out, for fully associative they are the data width for normal read and write
    // so/si: search data in/out, for fully associative they are the data width for the search ops
    // for CAM, si=di, but so = matching address. do = data out = di (for normal read/write)
    // so/si needs broadcase while do/di do not

    if (fully_assoc || pure_cam) {
        switch (Ndbl) {
        case (0):
            cout <<  "   Invalid Ndbl \n" << endl;
            exit(0);
            break;
        case (1):
            num_mats_h_dir = 1;//one subarray per mat
            num_mats_v_dir = 1;
            break;
        case (2):
            num_mats_h_dir = 1;//two subarrays per mat
            num_mats_v_dir = 1;
            break;
        default:
            num_mats_h_dir = int(floor(sqrt(Ndbl / 4.0)));//4 subbarrys per mat
            num_mats_v_dir = int(Ndbl / 4.0 / num_mats_h_dir);
        }
        num_mats = num_mats_h_dir * num_mats_v_dir;

        if (fully_assoc) {
            num_so_b_mat   = data_num_c_subarray;
            num_do_b_mat   = data_num_c_subarray + tagbits;
        } else {
            num_so_b_mat = int(ceil(log2(num_r_subarray)) + ceil(log2(num_subarrays)));//the address contains the matched data
            num_do_b_mat = tagbits;
        }
    } else {
        num_mats_h_dir = MAX(Ndwl / 2, 1);
        num_mats_v_dir = MAX(Ndbl / 2, 1);
        num_mats       = num_mats_h_dir * num_mats_v_dir;
        num_do_b_mat = MAX((num_subarrays / num_mats) * num_c_subarray /
                           (deg_bl_muxing * Ndsam_lev_1 * Ndsam_lev_2), 1);
    }

    if (!(fully_assoc || pure_cam) && (num_do_b_mat <
                                       (num_subarrays / num_mats))) {
        return;
    }


    int deg_sa_mux_l1_non_assoc;
    //TODO:the i/o for subbank is not necessary and should be removed.
    if (!(fully_assoc || pure_cam)) {
        if (!is_tag) {
            if (is_main_mem == true) {
                num_do_b_subbank = g_ip->int_prefetch_w * g_ip->out_w;
                deg_sa_mux_l1_non_assoc = Ndsam_lev_1;
            } else {
                if (g_ip->fast_access == true) {
                    num_do_b_subbank = g_ip->out_w * g_ip->data_assoc;
                    deg_sa_mux_l1_non_assoc = Ndsam_lev_1;
                } else {

                    num_do_b_subbank = g_ip->out_w;
                    deg_sa_mux_l1_non_assoc = Ndsam_lev_1 / g_ip->data_assoc;
                    if (deg_sa_mux_l1_non_assoc < 1) {
                        return;
                    }

                }
            }
        } else {
            num_do_b_subbank = tagbits * g_ip->tag_assoc;
            if (num_do_b_mat < tagbits) {
                return;
            }
            deg_sa_mux_l1_non_assoc = Ndsam_lev_1;
            //num_do_b_mat = g_ip->tag_assoc / num_mats_h_dir;
        }
    } else {
        if (fully_assoc) {
            num_so_b_subbank = 8 * g_ip->block_sz;//TODO:internal perfetch should be considered also for fa
            num_do_b_subbank = num_so_b_subbank + tag_num_c_subarray;
        } else {
            num_so_b_subbank = int(ceil(log2(num_r_subarray)) + ceil(log2(num_subarrays)));//the address contains the matched data
            num_do_b_subbank = tag_num_c_subarray;
        }

        deg_sa_mux_l1_non_assoc = 1;
    }

    deg_senseamp_muxing_non_associativity = deg_sa_mux_l1_non_assoc;

    if (fully_assoc || pure_cam) {
        num_act_mats_hor_dir = 1;
        num_act_mats_hor_dir_sl = num_mats_h_dir;//TODO: this is unnecessary, since search op, num_mats is used
    } else {
        num_act_mats_hor_dir = num_do_b_subbank / num_do_b_mat;
        if (num_act_mats_hor_dir == 0) {
            return;
        }
    }

    //compute num_do_mat for tag
    if (is_tag) {
        if (!(fully_assoc || pure_cam)) {
            num_do_b_mat     = g_ip->tag_assoc / num_act_mats_hor_dir;
            num_do_b_subbank = num_act_mats_hor_dir * num_do_b_mat;
        }
    }

    if ((g_ip->is_cache == false && is_main_mem == true) ||
        (PAGE_MODE == 1 && is_dram)) {
        if (num_act_mats_hor_dir * num_do_b_mat * Ndsam_lev_1 * Ndsam_lev_2 !=
            (int)g_ip->page_sz_bits) {
            return;
        }
    }

//  if (is_tag == false && g_ip->is_cache == true && !fully_assoc && !pure_cam && //TODO: TODO burst transfer should also apply to RAM arrays
    if (is_tag == false && g_ip->is_main_mem == true &&
        num_act_mats_hor_dir*num_do_b_mat*Ndsam_lev_1*Ndsam_lev_2 <
        ((int) g_ip->out_w * (int) g_ip->burst_len * (int) g_ip->data_assoc)) {
        return;
    }

    if (num_act_mats_hor_dir > num_mats_h_dir) {
        return;
    }


    //compute di for mat subbank and bank
    if (!(fully_assoc || pure_cam)) {
        if (!is_tag) {
            if (g_ip->fast_access == true) {
                num_di_b_mat = num_do_b_mat / g_ip->data_assoc;
            } else {
                num_di_b_mat = num_do_b_mat;
            }
        } else {
            num_di_b_mat = tagbits;
        }
    } else {
        if (fully_assoc) {
            num_di_b_mat = num_do_b_mat;
            //*num_subarrays/num_mats; bits per mat of CAM/FA is as same as cache,
            //but inside the mat wire tracks need to be reserved for search data bus
            num_si_b_mat = tagbits;
        } else {
            num_di_b_mat = tagbits;
            num_si_b_mat = tagbits;//*num_subarrays/num_mats;
        }

    }

    num_di_b_subbank       = num_di_b_mat * num_act_mats_hor_dir;//normal cache or normal r/w for FA
    num_si_b_subbank       = num_si_b_mat; //* num_act_mats_hor_dir_sl; inside the data is broadcast

    int num_addr_b_row_dec     = _log2(num_r_subarray);
    if  ((fully_assoc || pure_cam))
        num_addr_b_row_dec     += _log2(num_subarrays / num_mats);
    int number_subbanks        = num_mats / num_act_mats_hor_dir;
    number_subbanks_decode = _log2(number_subbanks);//TODO: add log2(num_subarray_per_bank) to FA/CAM

    num_rw_ports = g_ip->num_rw_ports;
    num_rd_ports = g_ip->num_rd_ports;
    num_wr_ports = g_ip->num_wr_ports;
    num_se_rd_ports = g_ip->num_se_rd_ports;
    num_search_ports = g_ip->num_search_ports;

    if (is_dram && is_main_mem) {
        number_addr_bits_mat = MAX((unsigned int) num_addr_b_row_dec,
                                   _log2(deg_bl_muxing) + _log2(deg_sa_mux_l1_non_assoc) + _log2(Ndsam_lev_2));
    } else {
        number_addr_bits_mat = num_addr_b_row_dec + _log2(deg_bl_muxing) +
                               _log2(deg_sa_mux_l1_non_assoc) + _log2(Ndsam_lev_2);
    }

    if (!(fully_assoc || pure_cam)) {
        if (is_tag) {
            num_di_b_bank_per_port = tagbits;
            num_do_b_bank_per_port = g_ip->data_assoc;
        } else {
            num_di_b_bank_per_port = g_ip->out_w + g_ip->data_assoc;
            num_do_b_bank_per_port = g_ip->out_w;
        }
    } else {
        if (fully_assoc) {
            num_di_b_bank_per_port = g_ip->out_w + tagbits;//TODO: out_w or block_sz?
            num_si_b_bank_per_port = tagbits;
            num_do_b_bank_per_port = g_ip->out_w + tagbits;
            num_so_b_bank_per_port = g_ip->out_w;
        } else {
            num_di_b_bank_per_port = tagbits;
            num_si_b_bank_per_port = tagbits;
            num_do_b_bank_per_port = tagbits;
            num_so_b_bank_per_port = int(ceil(log2(num_r_subarray)) + ceil(log2(num_subarrays)));
        }
    }

    if ((!is_tag) && (g_ip->data_assoc > 1) && (!g_ip->fast_access)) {
        number_way_select_signals_mat = g_ip->data_assoc;
    }

    // add ECC adjustment to all data signals that traverse on H-trees.
    if (g_ip->add_ecc_b_ == true) {
        num_do_b_mat += (int) (ceil(num_do_b_mat / num_bits_per_ecc_b_));
        num_di_b_mat += (int) (ceil(num_di_b_mat / num_bits_per_ecc_b_));
        num_di_b_subbank += (int) (ceil(num_di_b_subbank / num_bits_per_ecc_b_));
        num_do_b_subbank += (int) (ceil(num_do_b_subbank / num_bits_per_ecc_b_));
        num_di_b_bank_per_port += (int) (ceil(num_di_b_bank_per_port / num_bits_per_ecc_b_));
        num_do_b_bank_per_port += (int) (ceil(num_do_b_bank_per_port / num_bits_per_ecc_b_));

        num_so_b_mat += (int) (ceil(num_so_b_mat / num_bits_per_ecc_b_));
        num_si_b_mat += (int) (ceil(num_si_b_mat / num_bits_per_ecc_b_));
        num_si_b_subbank += (int) (ceil(num_si_b_subbank / num_bits_per_ecc_b_));
        num_so_b_subbank += (int) (ceil(num_so_b_subbank / num_bits_per_ecc_b_));
        num_si_b_bank_per_port += (int) (ceil(num_si_b_bank_per_port / num_bits_per_ecc_b_));
        num_so_b_bank_per_port += (int) (ceil(num_so_b_bank_per_port / num_bits_per_ecc_b_));
    }

    is_valid = true;
}

