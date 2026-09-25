/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * (University of Wisconsin-Madison)
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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
 *
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
 */

#include "cacti_wrapper.hh"

uca_org_t
cactiWrapper(unsigned num_of_bytes, unsigned wordsize, unsigned num_ports,
             int cache_type)
{
    int cache_size = num_of_bytes;
    int line_size = wordsize; // in bytes
    if (wordsize < 4) {
        line_size = 4;
    }
    if (cache_size / line_size < 64) {
        cache_size = line_size * 64; // min scratchpad size: 64 words
    }
    int associativity = 1;
    int rw_ports = num_ports;
    if (rw_ports == 0) {
        rw_ports = 1;
    }
    int excl_read_ports = 0;
    int excl_write_ports = 0;
    int single_ended_read_ports = 0;
    int search_ports = 0;
    int banks = 1;
    double tech_node = 40; // in nm
    int page_sz = 0;
    int burst_length = 8;
    int pre_width = 8;
    int output_width = wordsize * 8;
    int specific_tag = false;
    int tag_width = 0;
    int access_mode = 2;
    int cache = cache_type;
    int main_mem = 0;
    int obj_func_delay = 0;
    int obj_func_dynamic_power = 0;
    int obj_func_leakage_power = 100;
    int obj_func_area = 0;
    int obj_func_cycle_time = 0;
    int dev_func_delay = 20;
    int dev_func_dynamic_power = 100000;
    int dev_func_leakage_power = 100000;
    int dev_func_area = 1000000;
    int dev_func_cycle_time = 1000000;
    int ed_ed2_none = 2;
    int temp = 300;
    int wt = 0;
    int data_arr_ram_cell_tech_flavor_in = 0;
    int data_arr_peri_global_tech_flavor_in = 0;
    int tag_arr_ram_cell_tech_flavor_in = 0;
    int tag_arr_peri_global_tech_flavor_in = 0;
    int interconnect_projection_type_in = 1;
    int wire_inside_mat_type_in = 1;
    int wire_outside_mat_type_in = 1;
    int REPEATERS_IN_HTREE_SEGMENTS_in = 1;
    int VERTICAL_HTREE_WIRES_OVER_THE_ARRAY_in = 0;
    int BROADCAST_ADDR_DATAIN_OVER_VERTICAL_HTREES_in = 0;
    int force_wiretype = 1;
    int wiretype = 30;
    int force_config = 0;
    int ndwl = 1;
    int ndbl = 1;
    int nspd = 0;
    int ndcm = 1;
    int ndsam1 = 0;
    int ndsam2 = 0;
    int ecc = 0;
    return cacti_interface(
        cache_size, line_size, associativity, rw_ports, excl_read_ports,
        excl_write_ports, single_ended_read_ports, search_ports, banks,
        tech_node, output_width, specific_tag, tag_width, access_mode, cache,
        main_mem, obj_func_delay, obj_func_dynamic_power,
        obj_func_leakage_power, obj_func_cycle_time, obj_func_area,
        dev_func_delay, dev_func_dynamic_power, dev_func_leakage_power,
        dev_func_area, dev_func_cycle_time, ed_ed2_none, temp, wt,
        data_arr_ram_cell_tech_flavor_in, data_arr_peri_global_tech_flavor_in,
        tag_arr_ram_cell_tech_flavor_in, tag_arr_peri_global_tech_flavor_in,
        interconnect_projection_type_in, wire_inside_mat_type_in,
        wire_outside_mat_type_in, REPEATERS_IN_HTREE_SEGMENTS_in,
        VERTICAL_HTREE_WIRES_OVER_THE_ARRAY_in,
        BROADCAST_ADDR_DATAIN_OVER_VERTICAL_HTREES_in, page_sz, burst_length,
        pre_width, force_wiretype, wiretype, force_config, ndwl, ndbl, nspd,
        ndcm, ndsam1, ndsam2, ecc);
}
