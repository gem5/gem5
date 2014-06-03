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



#include <iostream>

#include "bank.h"

Bank::Bank(const DynamicParameter & dyn_p):
        dp(dyn_p), mat(dp),
        num_addr_b_mat(dyn_p.number_addr_bits_mat),
        num_mats_hor_dir(dyn_p.num_mats_h_dir),
        num_mats_ver_dir(dyn_p.num_mats_v_dir) {
    int RWP;
    int ERP;
    int EWP;
    int SCHP;

    if (dp.use_inp_params) {
        RWP  = dp.num_rw_ports;
        ERP  = dp.num_rd_ports;
        EWP  = dp.num_wr_ports;
        SCHP = dp.num_search_ports;
    } else {
        RWP  = g_ip->num_rw_ports;
        ERP  = g_ip->num_rd_ports;
        EWP  = g_ip->num_wr_ports;
        SCHP = g_ip->num_search_ports;
    }

    int total_addrbits = (dp.number_addr_bits_mat +
                          dp.number_subbanks_decode) * (RWP + ERP + EWP);
    int datainbits     = dp.num_di_b_bank_per_port * (RWP + EWP);
    int dataoutbits    = dp.num_do_b_bank_per_port * (RWP + ERP);
    int searchinbits;
    int searchoutbits;

    if (dp.fully_assoc || dp.pure_cam) {
        datainbits   = dp.num_di_b_bank_per_port * (RWP + EWP);
        dataoutbits  = dp.num_do_b_bank_per_port * (RWP + ERP);
        searchinbits    = dp.num_si_b_bank_per_port * SCHP;
        searchoutbits   = dp.num_so_b_bank_per_port * SCHP;
    }

    if (!(dp.fully_assoc || dp.pure_cam)) {
        if (g_ip->fast_access && dp.is_tag == false) {
            dataoutbits *= g_ip->data_assoc;
        }

        htree_in_add = new Htree2(g_ip->wt, (double) mat.area.w,
                                  (double)mat.area.h,
                                  total_addrbits, datainbits, 0, dataoutbits,
                                  0, num_mats_ver_dir * 2, num_mats_hor_dir * 2,
                                  Add_htree);
        htree_in_data = new Htree2(g_ip->wt, (double) mat.area.w,
                                   (double)mat.area.h,
                                   total_addrbits, datainbits, 0, dataoutbits,
                                   0, num_mats_ver_dir * 2, num_mats_hor_dir * 2,
                                   Data_in_htree);
        htree_out_data = new Htree2(g_ip->wt, (double) mat.area.w,
                                    (double)mat.area.h,
                                    total_addrbits, datainbits, 0, dataoutbits,
                                    0, num_mats_ver_dir * 2,
                                    num_mats_hor_dir * 2, Data_out_htree);

//  htree_out_data = new Htree2 (g_ip->wt,(double) 100, (double)100,
//                total_addrbits, datainbits, 0,dataoutbits,0, num_mats_ver_dir*2, num_mats_hor_dir*2, Data_out_htree);

        area.w = htree_in_data->area.w;
        area.h = htree_in_data->area.h;
    } else {
        htree_in_add =
            new Htree2(g_ip->wt, (double) mat.area.w, (double)mat.area.h,
                       total_addrbits, datainbits, searchinbits, dataoutbits,
                       searchoutbits, num_mats_ver_dir * 2,
                       num_mats_hor_dir * 2, Add_htree);
        htree_in_data =
            new Htree2(g_ip->wt, (double) mat.area.w, (double)mat.area.h,
                       total_addrbits, datainbits, searchinbits, dataoutbits,
                       searchoutbits, num_mats_ver_dir * 2,
                       num_mats_hor_dir * 2, Data_in_htree);
        htree_out_data =
            new Htree2(g_ip->wt, (double) mat.area.w, (double)mat.area.h,
                       total_addrbits, datainbits, searchinbits, dataoutbits,
                       searchoutbits, num_mats_ver_dir * 2,
                       num_mats_hor_dir * 2, Data_out_htree);
        htree_in_search =
            new Htree2(g_ip->wt, (double) mat.area.w, (double)mat.area.h,
                       total_addrbits, datainbits, searchinbits, dataoutbits,
                       searchoutbits, num_mats_ver_dir * 2,
                       num_mats_hor_dir * 2, Data_in_htree, true, true);
        htree_out_search =
            new Htree2 (g_ip->wt, (double) mat.area.w, (double)mat.area.h,
                        total_addrbits, datainbits, searchinbits, dataoutbits,
                        searchoutbits, num_mats_ver_dir * 2,
                        num_mats_hor_dir * 2, Data_out_htree, true);

        area.w = htree_in_data->area.w;
        area.h = htree_in_data->area.h;
    }

    num_addr_b_row_dec = _log2(mat.subarray.num_rows);
    num_addr_b_routed_to_mat_for_act = num_addr_b_row_dec;
    num_addr_b_routed_to_mat_for_rd_or_wr =
        num_addr_b_mat - num_addr_b_row_dec;
}



Bank::~Bank() {
    delete htree_in_add;
    delete htree_out_data;
    delete htree_in_data;
    if (dp.fully_assoc || dp.pure_cam) {
        delete htree_in_search;
        delete htree_out_search;
    }
}



double Bank::compute_delays(double inrisetime) {
    return mat.compute_delays(inrisetime);
}



void Bank::compute_power_energy() {
    mat.compute_power_energy();

    if (!(dp.fully_assoc || dp.pure_cam)) {
        power.readOp.dynamic += mat.power.readOp.dynamic * dp.num_act_mats_hor_dir;
        power.readOp.leakage += mat.power.readOp.leakage * dp.num_mats;
        power.readOp.gate_leakage += mat.power.readOp.gate_leakage * dp.num_mats;

        power.readOp.dynamic += htree_in_add->power.readOp.dynamic;
        power.readOp.dynamic += htree_out_data->power.readOp.dynamic;

        power.readOp.leakage += htree_in_add->power.readOp.leakage;
        power.readOp.leakage += htree_in_data->power.readOp.leakage;
        power.readOp.leakage += htree_out_data->power.readOp.leakage;
        power.readOp.gate_leakage += htree_in_add->power.readOp.gate_leakage;
        power.readOp.gate_leakage += htree_in_data->power.readOp.gate_leakage;
        power.readOp.gate_leakage += htree_out_data->power.readOp.gate_leakage;
    } else {

        power.readOp.dynamic += mat.power.readOp.dynamic ;//for fa and cam num_act_mats_hor_dir is 1 for plain r/w
        power.readOp.leakage += mat.power.readOp.leakage * dp.num_mats;
        power.readOp.gate_leakage += mat.power.readOp.gate_leakage * dp.num_mats;

        power.searchOp.dynamic += mat.power.searchOp.dynamic * dp.num_mats;
        power.searchOp.dynamic += mat.power_bl_precharge_eq_drv.searchOp.dynamic +
                                  mat.power_sa.searchOp.dynamic +
                                  mat.power_bitline.searchOp.dynamic +
                                  mat.power_subarray_out_drv.searchOp.dynamic +
                                  mat.ml_to_ram_wl_drv->power.readOp.dynamic;

        power.readOp.dynamic += htree_in_add->power.readOp.dynamic;
        power.readOp.dynamic += htree_out_data->power.readOp.dynamic;

        power.searchOp.dynamic += htree_in_search->power.searchOp.dynamic;
        power.searchOp.dynamic += htree_out_search->power.searchOp.dynamic;

        power.readOp.leakage += htree_in_add->power.readOp.leakage;
        power.readOp.leakage += htree_in_data->power.readOp.leakage;
        power.readOp.leakage += htree_out_data->power.readOp.leakage;
        power.readOp.leakage += htree_in_search->power.readOp.leakage;
        power.readOp.leakage += htree_out_search->power.readOp.leakage;


        power.readOp.gate_leakage += htree_in_add->power.readOp.gate_leakage;
        power.readOp.gate_leakage += htree_in_data->power.readOp.gate_leakage;
        power.readOp.gate_leakage += htree_out_data->power.readOp.gate_leakage;
        power.readOp.gate_leakage += htree_in_search->power.readOp.gate_leakage;
        power.readOp.gate_leakage += htree_out_search->power.readOp.gate_leakage;

    }

}

