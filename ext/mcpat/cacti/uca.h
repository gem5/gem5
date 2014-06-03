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



#ifndef __UCA_H__
#define __UCA_H__

#include "area.h"
#include "bank.h"
#include "component.h"
#include "htree2.h"
#include "parameter.h"

class UCA : public Component {
public:
    UCA(const DynamicParameter & dyn_p);
    ~UCA();
    double compute_delays(double inrisetime);  // returns outrisetime
    void   compute_power_energy();

    DynamicParameter dp;
    Bank   bank;

    Htree2   * htree_in_add;
    Htree2   * htree_in_data;
    Htree2   * htree_out_data;
    Htree2   * htree_in_search;
    Htree2   * htree_out_search;

    powerDef power_routing_to_bank;

    uint32_t nbanks;

    int   num_addr_b_bank;
    int   num_di_b_bank;
    int   num_do_b_bank;
    int   num_si_b_bank;
    int   num_so_b_bank;
    int RWP;
    int ERP;
    int EWP;
    int SCHP;
    double area_all_dataramcells;

    double dyn_read_energy_from_closed_page;
    double dyn_read_energy_from_open_page;
    double dyn_read_energy_remaining_words_in_burst;

    double refresh_power;  // only for DRAM
    double activate_energy;
    double read_energy;
    double write_energy;
    double precharge_energy;
    double leak_power_subbank_closed_page;
    double leak_power_subbank_open_page;
    double leak_power_request_and_reply_networks;

    double delay_array_to_sa_mux_lev_1_decoder;
    double delay_array_to_sa_mux_lev_2_decoder;
    double delay_before_subarray_output_driver;
    double delay_from_subarray_out_drv_to_out;
    double access_time;
    double precharge_delay;
    double multisubbank_interleave_cycle_time;
};

#endif

