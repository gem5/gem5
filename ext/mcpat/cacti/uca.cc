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



#include <cmath>
#include <iostream>

#include "uca.h"

UCA::UCA(const DynamicParameter & dyn_p)
    : dp(dyn_p), bank(dp), nbanks(g_ip->nbanks), refresh_power(0) {
    int num_banks_ver_dir = 1 << ((bank.area.h > bank.area.w) ? _log2(nbanks)
                                  / 2 : (_log2(nbanks) - _log2(nbanks) / 2));
    int num_banks_hor_dir = nbanks / num_banks_ver_dir;

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

    num_addr_b_bank = (dp.number_addr_bits_mat + dp.number_subbanks_decode) *
        (RWP + ERP + EWP);
    num_di_b_bank   = dp.num_di_b_bank_per_port * (RWP + EWP);
    num_do_b_bank   = dp.num_do_b_bank_per_port * (RWP + ERP);
    num_si_b_bank   = dp.num_si_b_bank_per_port * SCHP;
    num_so_b_bank   = dp.num_so_b_bank_per_port * SCHP;

    if (!dp.fully_assoc && !dp.pure_cam) {

        if (g_ip->fast_access && dp.is_tag == false) {
            num_do_b_bank *= g_ip->data_assoc;
        }

        htree_in_add = new Htree2(g_ip->wt, bank.area.w, bank.area.h,
                                  num_addr_b_bank, num_di_b_bank, 0,
                                  num_do_b_bank, 0, num_banks_ver_dir * 2,
                                  num_banks_hor_dir * 2, Add_htree, true);
        htree_in_data = new Htree2(g_ip->wt, bank.area.w, bank.area.h,
                                   num_addr_b_bank, num_di_b_bank, 0,
                                   num_do_b_bank, 0, num_banks_ver_dir * 2,
                                   num_banks_hor_dir * 2, Data_in_htree, true);
        htree_out_data = new Htree2(g_ip->wt, bank.area.w, bank.area.h,
                                    num_addr_b_bank, num_di_b_bank, 0,
                                    num_do_b_bank, 0, num_banks_ver_dir * 2,
                                    num_banks_hor_dir * 2, Data_out_htree, true);
    }

    else {

        htree_in_add = new Htree2(g_ip->wt, bank.area.w, bank.area.h,
                                  num_addr_b_bank, num_di_b_bank,
                                  num_si_b_bank, num_do_b_bank, num_so_b_bank,
                                  num_banks_ver_dir * 2, num_banks_hor_dir * 2,
                                  Add_htree, true);
        htree_in_data = new Htree2(g_ip->wt, bank.area.w, bank.area.h,
                                   num_addr_b_bank, num_di_b_bank,
                                   num_si_b_bank, num_do_b_bank, num_so_b_bank,
                                   num_banks_ver_dir * 2, num_banks_hor_dir * 2,
                                   Data_in_htree, true);
        htree_out_data = new Htree2(g_ip->wt, bank.area.w, bank.area.h,
                                    num_addr_b_bank, num_di_b_bank,
                                    num_si_b_bank, num_do_b_bank,
                                    num_so_b_bank, num_banks_ver_dir * 2,
                                    num_banks_hor_dir * 2, Data_out_htree, true);
        htree_in_search = new Htree2(g_ip->wt, bank.area.w, bank.area.h,
                                     num_addr_b_bank, num_di_b_bank,
                                     num_si_b_bank, num_do_b_bank,
                                     num_so_b_bank, num_banks_ver_dir * 2,
                                     num_banks_hor_dir * 2, Data_in_htree, true);
        htree_out_search = new Htree2(g_ip->wt, bank.area.w, bank.area.h,
                                      num_addr_b_bank, num_di_b_bank,
                                      num_si_b_bank, num_do_b_bank,
                                      num_so_b_bank, num_banks_ver_dir * 2,
                                      num_banks_hor_dir * 2, Data_out_htree,
                                      true);
    }

    area.w = htree_in_data->area.w;
    area.h = htree_in_data->area.h;

    area_all_dataramcells = bank.mat.subarray.get_total_cell_area() * dp.num_subarrays * g_ip->nbanks;
//  cout<<"area cell"<<area_all_dataramcells<<endl;
//  cout<<area.get_area()<<endl;
    // delay calculation
    double inrisetime = 0.0;
    compute_delays(inrisetime);
    compute_power_energy();
}



UCA::~UCA() {
    delete htree_in_add;
    delete htree_in_data;
    delete htree_out_data;
}



double UCA::compute_delays(double inrisetime) {
    double outrisetime = bank.compute_delays(inrisetime);

    double delay_array_to_mat = htree_in_add->delay + bank.htree_in_add->delay;
    double max_delay_before_row_decoder = delay_array_to_mat + bank.mat.r_predec->delay;
    delay_array_to_sa_mux_lev_1_decoder = delay_array_to_mat +
                                          bank.mat.sa_mux_lev_1_predec->delay +
                                          bank.mat.sa_mux_lev_1_dec->delay;
    delay_array_to_sa_mux_lev_2_decoder = delay_array_to_mat +
                                          bank.mat.sa_mux_lev_2_predec->delay +
                                          bank.mat.sa_mux_lev_2_dec->delay;
    double delay_inside_mat = bank.mat.row_dec->delay + bank.mat.delay_bitline + bank.mat.delay_sa;

    delay_before_subarray_output_driver =
        MAX(MAX(max_delay_before_row_decoder + delay_inside_mat,  // row_path
                delay_array_to_mat + bank.mat.b_mux_predec->delay + bank.mat.bit_mux_dec->delay + bank.mat.delay_sa),  // col_path
            MAX(delay_array_to_sa_mux_lev_1_decoder,    // sa_mux_lev_1_path
                delay_array_to_sa_mux_lev_2_decoder));  // sa_mux_lev_2_path
    delay_from_subarray_out_drv_to_out = bank.mat.delay_subarray_out_drv_htree +
                                         bank.htree_out_data->delay + htree_out_data->delay;
    access_time                        = bank.mat.delay_comparator;

    double ram_delay_inside_mat;
    if (dp.fully_assoc) {
        //delay of FA contains both CAM tag and RAM data
        { //delay of CAM
            ram_delay_inside_mat = bank.mat.delay_bitline + bank.mat.delay_matchchline;
            access_time = htree_in_add->delay + bank.htree_in_add->delay;
            //delay of fully-associative data array
            access_time += ram_delay_inside_mat + delay_from_subarray_out_drv_to_out;
        }
    } else {
        access_time = delay_before_subarray_output_driver + delay_from_subarray_out_drv_to_out; //data_acc_path
    }

    if (dp.is_main_mem) {
        double t_rcd       = max_delay_before_row_decoder + delay_inside_mat;
        double cas_latency = MAX(delay_array_to_sa_mux_lev_1_decoder, delay_array_to_sa_mux_lev_2_decoder) +
                             delay_from_subarray_out_drv_to_out;
        access_time = t_rcd + cas_latency;
    }

    double temp;

    if (!dp.fully_assoc) {
        temp = delay_inside_mat + bank.mat.delay_wl_reset + bank.mat.delay_bl_restore;//TODO: Sheng: revisit
        if (dp.is_dram) {
            temp += bank.mat.delay_writeback;  // temp stores random cycle time
        }


        temp = MAX(temp, bank.mat.r_predec->delay);
        temp = MAX(temp, bank.mat.b_mux_predec->delay);
        temp = MAX(temp, bank.mat.sa_mux_lev_1_predec->delay);
        temp = MAX(temp, bank.mat.sa_mux_lev_2_predec->delay);
    } else {
        ram_delay_inside_mat = bank.mat.delay_bitline + bank.mat.delay_matchchline;
        temp = ram_delay_inside_mat + bank.mat.delay_cam_sl_restore + bank.mat.delay_cam_ml_reset + bank.mat.delay_bl_restore
               + bank.mat.delay_hit_miss_reset + bank.mat.delay_wl_reset;

        temp = MAX(temp, bank.mat.b_mux_predec->delay);//TODO: Sheng revisit whether distinguish cam and ram bitline etc.
        temp = MAX(temp, bank.mat.sa_mux_lev_1_predec->delay);
        temp = MAX(temp, bank.mat.sa_mux_lev_2_predec->delay);
    }

    // The following is true only if the input parameter "repeaters_in_htree" is set to false --Nav
    if (g_ip->rpters_in_htree == false) {
        temp = MAX(temp, bank.htree_in_add->max_unpipelined_link_delay);
    }
    cycle_time = temp;

    double delay_req_network = max_delay_before_row_decoder;
    double delay_rep_network = delay_from_subarray_out_drv_to_out;
    multisubbank_interleave_cycle_time = MAX(delay_req_network, delay_rep_network);

    if (dp.is_main_mem) {
        multisubbank_interleave_cycle_time = htree_in_add->delay;
        precharge_delay = htree_in_add->delay +
                          bank.htree_in_add->delay + bank.mat.delay_writeback +
                          bank.mat.delay_wl_reset + bank.mat.delay_bl_restore;
        cycle_time = access_time + precharge_delay;
    } else {
        precharge_delay = 0;
    }

    double dram_array_availability = 0;
    if (dp.is_dram) {
        dram_array_availability = (1 - dp.num_r_subarray * cycle_time / dp.dram_refresh_period) * 100;
    }

    return outrisetime;
}



// note: currently, power numbers are for a bank of an array
void UCA::compute_power_energy() {
    bank.compute_power_energy();
    power = bank.power;

    power_routing_to_bank.readOp.dynamic  = htree_in_add->power.readOp.dynamic + htree_out_data->power.readOp.dynamic;
    power_routing_to_bank.writeOp.dynamic = htree_in_add->power.readOp.dynamic + htree_in_data->power.readOp.dynamic;
    if (dp.fully_assoc || dp.pure_cam)
        power_routing_to_bank.searchOp.dynamic =
            htree_in_search->power.searchOp.dynamic +
            htree_out_search->power.searchOp.dynamic;

    power_routing_to_bank.readOp.leakage +=
        htree_in_add->power.readOp.leakage +
        htree_in_data->power.readOp.leakage +
        htree_out_data->power.readOp.leakage;

    power_routing_to_bank.readOp.gate_leakage +=
        htree_in_add->power.readOp.gate_leakage +
        htree_in_data->power.readOp.gate_leakage +
        htree_out_data->power.readOp.gate_leakage;
    if (dp.fully_assoc || dp.pure_cam) {
        power_routing_to_bank.readOp.leakage += htree_in_search->power.readOp.leakage + htree_out_search->power.readOp.leakage;
        power_routing_to_bank.readOp.gate_leakage += htree_in_search->power.readOp.gate_leakage + htree_out_search->power.readOp.gate_leakage;
    }

    power.searchOp.dynamic += power_routing_to_bank.searchOp.dynamic;
    power.readOp.dynamic += power_routing_to_bank.readOp.dynamic;
    power.readOp.leakage += power_routing_to_bank.readOp.leakage;
    power.readOp.gate_leakage += power_routing_to_bank.readOp.gate_leakage;

    // calculate total write energy per access
    power.writeOp.dynamic = power.readOp.dynamic
                            - bank.mat.power_bitline.readOp.dynamic * dp.num_act_mats_hor_dir
                            + bank.mat.power_bitline.writeOp.dynamic * dp.num_act_mats_hor_dir
                            - power_routing_to_bank.readOp.dynamic
                            + power_routing_to_bank.writeOp.dynamic
                            + bank.htree_in_data->power.readOp.dynamic
                            - bank.htree_out_data->power.readOp.dynamic;

    if (dp.is_dram == false) {
        power.writeOp.dynamic -= bank.mat.power_sa.readOp.dynamic * dp.num_act_mats_hor_dir;
    }

    dyn_read_energy_from_closed_page = power.readOp.dynamic;
    dyn_read_energy_from_open_page   = power.readOp.dynamic -
                                       (bank.mat.r_predec->power.readOp.dynamic +
                                        bank.mat.power_row_decoders.readOp.dynamic +
                                        bank.mat.power_bl_precharge_eq_drv.readOp.dynamic +
                                        bank.mat.power_sa.readOp.dynamic +
                                        bank.mat.power_bitline.readOp.dynamic) * dp.num_act_mats_hor_dir;

    dyn_read_energy_remaining_words_in_burst =
        (MAX((g_ip->burst_len / g_ip->int_prefetch_w), 1) - 1) *
        ((bank.mat.sa_mux_lev_1_predec->power.readOp.dynamic +
          bank.mat.sa_mux_lev_2_predec->power.readOp.dynamic +
          bank.mat.power_sa_mux_lev_1_decoders.readOp.dynamic +
          bank.mat.power_sa_mux_lev_2_decoders.readOp.dynamic +
          bank.mat.power_subarray_out_drv.readOp.dynamic)     * dp.num_act_mats_hor_dir +
         bank.htree_out_data->power.readOp.dynamic +
         power_routing_to_bank.readOp.dynamic);
    dyn_read_energy_from_closed_page += dyn_read_energy_remaining_words_in_burst;
    dyn_read_energy_from_open_page   += dyn_read_energy_remaining_words_in_burst;

    activate_energy = htree_in_add->power.readOp.dynamic +
                      bank.htree_in_add->power_bit.readOp.dynamic * bank.num_addr_b_routed_to_mat_for_act +
                      (bank.mat.r_predec->power.readOp.dynamic +
                       bank.mat.power_row_decoders.readOp.dynamic +
                       bank.mat.power_sa.readOp.dynamic) * dp.num_act_mats_hor_dir;
    read_energy    = (htree_in_add->power.readOp.dynamic +
                      bank.htree_in_add->power_bit.readOp.dynamic * bank.num_addr_b_routed_to_mat_for_rd_or_wr +
                      (bank.mat.sa_mux_lev_1_predec->power.readOp.dynamic  +
                       bank.mat.sa_mux_lev_2_predec->power.readOp.dynamic  +
                       bank.mat.power_sa_mux_lev_1_decoders.readOp.dynamic +
                       bank.mat.power_sa_mux_lev_2_decoders.readOp.dynamic +
                       bank.mat.power_subarray_out_drv.readOp.dynamic) * dp.num_act_mats_hor_dir +
                      bank.htree_out_data->power.readOp.dynamic +
                      htree_in_data->power.readOp.dynamic) * g_ip->burst_len;
    write_energy   = (htree_in_add->power.readOp.dynamic +
                      bank.htree_in_add->power_bit.readOp.dynamic * bank.num_addr_b_routed_to_mat_for_rd_or_wr +
                      htree_in_data->power.readOp.dynamic +
                      bank.htree_in_data->power.readOp.dynamic +
                      (bank.mat.sa_mux_lev_1_predec->power.readOp.dynamic  +
                       bank.mat.sa_mux_lev_2_predec->power.readOp.dynamic  +
                       bank.mat.power_sa_mux_lev_1_decoders.readOp.dynamic +
                       bank.mat.power_sa_mux_lev_2_decoders.readOp.dynamic) * dp.num_act_mats_hor_dir) * g_ip->burst_len;
    precharge_energy = (bank.mat.power_bitline.readOp.dynamic +
                        bank.mat.power_bl_precharge_eq_drv.readOp.dynamic) * dp.num_act_mats_hor_dir;

    leak_power_subbank_closed_page =
        (bank.mat.r_predec->power.readOp.leakage +
         bank.mat.b_mux_predec->power.readOp.leakage +
         bank.mat.sa_mux_lev_1_predec->power.readOp.leakage +
         bank.mat.sa_mux_lev_2_predec->power.readOp.leakage +
         bank.mat.power_row_decoders.readOp.leakage +
         bank.mat.power_bit_mux_decoders.readOp.leakage +
         bank.mat.power_sa_mux_lev_1_decoders.readOp.leakage +
         bank.mat.power_sa_mux_lev_2_decoders.readOp.leakage +
         bank.mat.leak_power_sense_amps_closed_page_state) * dp.num_act_mats_hor_dir;

    leak_power_subbank_closed_page +=
        (bank.mat.r_predec->power.readOp.gate_leakage +
         bank.mat.b_mux_predec->power.readOp.gate_leakage +
         bank.mat.sa_mux_lev_1_predec->power.readOp.gate_leakage +
         bank.mat.sa_mux_lev_2_predec->power.readOp.gate_leakage +
         bank.mat.power_row_decoders.readOp.gate_leakage +
         bank.mat.power_bit_mux_decoders.readOp.gate_leakage +
         bank.mat.power_sa_mux_lev_1_decoders.readOp.gate_leakage +
         bank.mat.power_sa_mux_lev_2_decoders.readOp.gate_leakage) * dp.num_act_mats_hor_dir; //+
    //bank.mat.leak_power_sense_amps_closed_page_state) * dp.num_act_mats_hor_dir;

    leak_power_subbank_open_page =
        (bank.mat.r_predec->power.readOp.leakage +
         bank.mat.b_mux_predec->power.readOp.leakage +
         bank.mat.sa_mux_lev_1_predec->power.readOp.leakage +
         bank.mat.sa_mux_lev_2_predec->power.readOp.leakage +
         bank.mat.power_row_decoders.readOp.leakage +
         bank.mat.power_bit_mux_decoders.readOp.leakage +
         bank.mat.power_sa_mux_lev_1_decoders.readOp.leakage +
         bank.mat.power_sa_mux_lev_2_decoders.readOp.leakage +
         bank.mat.leak_power_sense_amps_open_page_state) * dp.num_act_mats_hor_dir;

    leak_power_subbank_open_page +=
        (bank.mat.r_predec->power.readOp.gate_leakage +
         bank.mat.b_mux_predec->power.readOp.gate_leakage +
         bank.mat.sa_mux_lev_1_predec->power.readOp.gate_leakage +
         bank.mat.sa_mux_lev_2_predec->power.readOp.gate_leakage +
         bank.mat.power_row_decoders.readOp.gate_leakage +
         bank.mat.power_bit_mux_decoders.readOp.gate_leakage +
         bank.mat.power_sa_mux_lev_1_decoders.readOp.gate_leakage +
         bank.mat.power_sa_mux_lev_2_decoders.readOp.gate_leakage ) * dp.num_act_mats_hor_dir;
    //bank.mat.leak_power_sense_amps_open_page_state) * dp.num_act_mats_hor_dir;

    leak_power_request_and_reply_networks =
        power_routing_to_bank.readOp.leakage +
        bank.htree_in_add->power.readOp.leakage +
        bank.htree_in_data->power.readOp.leakage +
        bank.htree_out_data->power.readOp.leakage;

    leak_power_request_and_reply_networks +=
        power_routing_to_bank.readOp.gate_leakage +
        bank.htree_in_add->power.readOp.gate_leakage +
        bank.htree_in_data->power.readOp.gate_leakage +
        bank.htree_out_data->power.readOp.gate_leakage;

    if (dp.fully_assoc || dp.pure_cam) {
        leak_power_request_and_reply_networks += htree_in_search->power.readOp.leakage + htree_out_search->power.readOp.leakage;
        leak_power_request_and_reply_networks += htree_in_search->power.readOp.gate_leakage + htree_out_search->power.readOp.gate_leakage;
    }


    // if DRAM, add contribution of power spent in row predecoder drivers,
    // blocks and decoders to refresh power
    if (dp.is_dram) {
        refresh_power  = (bank.mat.r_predec->power.readOp.dynamic * dp.num_act_mats_hor_dir +
                          bank.mat.row_dec->power.readOp.dynamic) * dp.num_r_subarray * dp.num_subarrays;
        refresh_power += bank.mat.per_bitline_read_energy * dp.num_c_subarray * dp.num_r_subarray * dp.num_subarrays;
        refresh_power += bank.mat.power_bl_precharge_eq_drv.readOp.dynamic * dp.num_act_mats_hor_dir;
        refresh_power += bank.mat.power_sa.readOp.dynamic * dp.num_act_mats_hor_dir;
        refresh_power /= dp.dram_refresh_period;
    }


    if (dp.is_tag == false) {
        power.readOp.dynamic  = dyn_read_energy_from_closed_page;
        power.writeOp.dynamic = dyn_read_energy_from_closed_page
                                - dyn_read_energy_remaining_words_in_burst
                                - bank.mat.power_bitline.readOp.dynamic * dp.num_act_mats_hor_dir
                                + bank.mat.power_bitline.writeOp.dynamic * dp.num_act_mats_hor_dir
                                + (power_routing_to_bank.writeOp.dynamic -
                                   power_routing_to_bank.readOp.dynamic -
                                   bank.htree_out_data->power.readOp.dynamic +
                                   bank.htree_in_data->power.readOp.dynamic) *
                                (MAX((g_ip->burst_len / g_ip->int_prefetch_w), 1) - 1); //FIXME

        if (dp.is_dram == false) {
            power.writeOp.dynamic -= bank.mat.power_sa.readOp.dynamic * dp.num_act_mats_hor_dir;
        }
    }

    // if DRAM, add refresh power to total leakage
    if (dp.is_dram) {
        power.readOp.leakage += refresh_power;
    }

    // TODO: below should be  avoided.
    /*if (dp.is_main_mem)
    {
      power.readOp.leakage += MAIN_MEM_PER_CHIP_STANDBY_CURRENT_mA * 1e-3 * g_tp.peri_global.Vdd / g_ip->nbanks;
    }*/

    assert(power.readOp.dynamic  > 0);
    assert(power.writeOp.dynamic > 0);
    assert(power.readOp.leakage  > 0);
}

