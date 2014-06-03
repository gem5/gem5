/*****************************************************************************
 *                                McPAT
 *                      SOFTWARE LICENSE AGREEMENT
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
 * Authors: Joel Hestness
 *          Yasuko Eckert
 *
 ***************************************************************************/

#include <cmath>
#include <iostream>

#include "area.h"
#include "cachearray.h"
#include "common.h"
#include "decoder.h"
#include "parameter.h"

using namespace std;

double CacheArray::area_efficiency_threshold = 20.0;
int CacheArray::ed = 0;
//Fixed number, make sure timing can be satisfied.
int CacheArray::delay_wt = 100;
int CacheArray::cycle_time_wt = 1000;
//Fixed number, This is used to exhaustive search for individual components.
int CacheArray::area_wt = 10;
//Fixed number, This is used to exhaustive search for individual components.
int CacheArray::dynamic_power_wt = 10;
int CacheArray::leakage_power_wt = 10;
//Fixed number, make sure timing can be satisfied.
int CacheArray::delay_dev = 1000000;
int CacheArray::cycle_time_dev = 100;
//Fixed number, This is used to exhaustive search for individual components.
int CacheArray::area_dev = 1000000;
//Fixed number, This is used to exhaustive search for individual components.
int CacheArray::dynamic_power_dev = 1000000;
int CacheArray::leakage_power_dev = 1000000;
int CacheArray::cycle_time_dev_threshold = 10;

CacheArray::CacheArray(XMLNode* _xml_data,
                 const InputParameter *configure_interface, string _name,
                 enum Device_ty device_ty_, double _clockRate,
                 bool opt_local_, enum Core_type core_ty_, bool _is_default)
        : McPATComponent(_xml_data), l_ip(*configure_interface),
        device_ty(device_ty_), opt_local(opt_local_), core_ty(core_ty_),
        is_default(_is_default), sbt_dir_overhead(0) {
    name = _name;
    clockRate = _clockRate;
    if (l_ip.cache_sz < MIN_BUFFER_SIZE) {
        l_ip.cache_sz = MIN_BUFFER_SIZE;
    }

    if (!l_ip.error_checking(name)) {
        exit(1);
    }

    sbt_tdp_stats.reset();
    sbt_rtp_stats.reset();

    // Compute initial search point
    local_result.valid = false;
    compute_base_power();

    // Set up the cache by searching design space with cacti
    list<uca_org_t > candidate_solutions(0);
    list<uca_org_t >::iterator candidate_iter, min_dynamic_energy_iter;
    uca_org_t* temp_res = NULL;
    double throughput = l_ip.throughput;
    double latency = l_ip.latency;
    bool throughput_overflow = true;
    bool latency_overflow = true;

    if ((local_result.cycle_time - throughput) <= 1e-10 )
        throughput_overflow = false;
    if ((local_result.access_time - latency) <= 1e-10)
        latency_overflow = false;

    if (opt_for_clk && opt_local) {
        if (throughput_overflow || latency_overflow) {
            l_ip.ed = ed;

            l_ip.delay_wt = delay_wt;
            l_ip.cycle_time_wt = cycle_time_wt;

            l_ip.area_wt = area_wt;
            l_ip.dynamic_power_wt = dynamic_power_wt;
            l_ip.leakage_power_wt = leakage_power_wt;

            l_ip.delay_dev = delay_dev;
            l_ip.cycle_time_dev = cycle_time_dev;

            l_ip.area_dev = area_dev;
            l_ip.dynamic_power_dev = dynamic_power_dev;
            l_ip.leakage_power_dev = leakage_power_dev;

            //Reset overflow flag before start optimization iterations
            throughput_overflow = true;
            latency_overflow = true;

            //Clean up the result for optimized for ED^2P
            temp_res = &local_result;
            temp_res->cleanup();
        }


        while ((throughput_overflow || latency_overflow) &&
               l_ip.cycle_time_dev > cycle_time_dev_threshold) {
            compute_base_power();

            //This is the time_dev to be used for next iteration
            l_ip.cycle_time_dev -= cycle_time_dev_threshold;

            //      from best area to worst area -->worst timing to best timing
            if ((((local_result.cycle_time - throughput) <= 1e-10 ) &&
                 (local_result.access_time - latency) <= 1e-10) ||
                (local_result.data_array2->area_efficiency <
                 area_efficiency_threshold && l_ip.assoc == 0)) {
                //if no satisfiable solution is found,the most aggressive one
                //is left
                candidate_solutions.push_back(local_result);
                if (((local_result.cycle_time - throughput) <= 1e-10) &&
                    ((local_result.access_time - latency) <= 1e-10)) {
                    //ensure stop opt not because of cam
                    throughput_overflow = false;
                    latency_overflow = false;
                }

            } else {
                if ((local_result.cycle_time - throughput) <= 1e-10)
                    throughput_overflow = false;
                if ((local_result.access_time - latency) <= 1e-10)
                    latency_overflow = false;

                //if not >10 local_result is the last result, it cannot be
                //cleaned up
                if (l_ip.cycle_time_dev > cycle_time_dev_threshold) {
                    //Only solutions not saved in the list need to be
                    //cleaned up
                    temp_res = &local_result;
                    temp_res->cleanup();
                }
            }
        }


        if (l_ip.assoc > 0) {
            //For array structures except CAM and FA, Give warning but still
            //provide a result with best timing found
            if (throughput_overflow == true)
                cout << "Warning: " << name
                     << " array structure cannot satisfy throughput constraint."
                     << endl;
            if (latency_overflow == true)
                cout << "Warning: " << name
                     << " array structure cannot satisfy latency constraint."
                     << endl;
        }

        double min_dynamic_energy = BIGNUM;
        if (candidate_solutions.empty() == false) {
            local_result.valid = true;
            for (candidate_iter = candidate_solutions.begin();
                 candidate_iter != candidate_solutions.end();
                 ++candidate_iter) {
                if (min_dynamic_energy >
                    (candidate_iter)->power.readOp.dynamic) {
                    min_dynamic_energy =
                        (candidate_iter)->power.readOp.dynamic;
                    min_dynamic_energy_iter = candidate_iter;
                    local_result = *(min_dynamic_energy_iter);

                } else {
                    candidate_iter->cleanup() ;
                }

            }


        }
        candidate_solutions.clear();
    }

    double long_channel_device_reduction =
        longer_channel_device_reduction(device_ty, core_ty);

    double macro_layout_overhead = g_tp.macro_layout_overhead;
    double chip_PR_overhead = g_tp.chip_layout_overhead;
    double total_overhead = macro_layout_overhead * chip_PR_overhead;
    local_result.area *= total_overhead;

    //maintain constant power density
    double pppm_t[4]    = {total_overhead, 1, 1, total_overhead};

    double sckRation = g_tp.sckt_co_eff;
    local_result.power.readOp.dynamic *= sckRation;
    local_result.power.writeOp.dynamic *= sckRation;
    local_result.power.searchOp.dynamic *= sckRation;
    local_result.power.readOp.leakage *= l_ip.nbanks;
    local_result.power.readOp.longer_channel_leakage =
        local_result.power.readOp.leakage * long_channel_device_reduction;
    local_result.power = local_result.power * pppm_t;

    local_result.data_array2->power.readOp.dynamic *= sckRation;
    local_result.data_array2->power.writeOp.dynamic *= sckRation;
    local_result.data_array2->power.searchOp.dynamic *= sckRation;
    local_result.data_array2->power.readOp.leakage *= l_ip.nbanks;
    local_result.data_array2->power.readOp.longer_channel_leakage =
        local_result.data_array2->power.readOp.leakage *
        long_channel_device_reduction;
    local_result.data_array2->power = local_result.data_array2->power * pppm_t;


    if (!(l_ip.pure_cam || l_ip.pure_ram || l_ip.fully_assoc) && l_ip.is_cache) {
        local_result.tag_array2->power.readOp.dynamic *= sckRation;
        local_result.tag_array2->power.writeOp.dynamic *= sckRation;
        local_result.tag_array2->power.searchOp.dynamic *= sckRation;
        local_result.tag_array2->power.readOp.leakage *= l_ip.nbanks;
        local_result.tag_array2->power.readOp.longer_channel_leakage =
            local_result.tag_array2->power.readOp.leakage *
            long_channel_device_reduction;
        local_result.tag_array2->power =
            local_result.tag_array2->power * pppm_t;
    }
}

void CacheArray::compute_base_power() {
    local_result = cacti_interface(&l_ip);
}

void CacheArray::computeArea() {
    area.set_area(local_result.area);
    output_data.area = local_result.area / 1e6;
}

void CacheArray::computeEnergy() {
    // Set the leakage power numbers
    output_data.subthreshold_leakage_power = local_result.power.readOp.leakage;
    output_data.gate_leakage_power = local_result.power.readOp.gate_leakage;

    if (l_ip.assoc && l_ip.is_cache) {
        // This is a standard cache array with data and tags
        // Calculate peak dynamic power
        output_data.peak_dynamic_power =
            (local_result.tag_array2->power.readOp.dynamic +
             local_result.data_array2->power.readOp.dynamic) *
            tdp_stats.readAc.hit +
            (local_result.tag_array2->power.readOp.dynamic) *
            tdp_stats.readAc.miss +
            (local_result.tag_array2->power.readOp.dynamic +
             local_result.data_array2->power.writeOp.dynamic) *
            tdp_stats.writeAc.hit +
            (local_result.tag_array2->power.readOp.dynamic) *
            tdp_stats.writeAc.miss;
        output_data.peak_dynamic_power *= clockRate;

        // Calculate the runtime dynamic power
        output_data.runtime_dynamic_energy =
            local_result.data_array2->power.readOp.dynamic *
            rtp_stats.dataReadAc.access +
            local_result.data_array2->power.writeOp.dynamic *
            rtp_stats.dataWriteAc.access +
            (local_result.tag_array2->power.readOp.dynamic *
             rtp_stats.tagReadAc.access +
             local_result.tag_array2->power.writeOp.dynamic *
             rtp_stats.tagWriteAc.access) * l_ip.assoc;
    } else {
        // Calculate peak dynamic power
        output_data.peak_dynamic_power =
                local_result.power.readOp.dynamic * tdp_stats.readAc.access +
                local_result.power.writeOp.dynamic * tdp_stats.writeAc.access +
                local_result.power.searchOp.dynamic * tdp_stats.searchAc.access;
        output_data.peak_dynamic_power *= clockRate;

        // Calculate the runtime dynamic power
        output_data.runtime_dynamic_energy =
                local_result.power.readOp.dynamic * rtp_stats.readAc.access +
                local_result.power.writeOp.dynamic * rtp_stats.writeAc.access +
                local_result.power.searchOp.dynamic * rtp_stats.searchAc.access;
    }

    // An SBT directory has more dynamic power
    if (sbt_dir_overhead > 0) {
        // Calculate peak dynamic power
        output_data.peak_dynamic_power +=
            (computeSBTDynEnergy(&sbt_tdp_stats) * clockRate);

        // Calculate the runtime dynamic power
        output_data.runtime_dynamic_energy +=
            computeSBTDynEnergy(&sbt_rtp_stats);
    }
}

CacheArray::~CacheArray() {
    local_result.cleanup();
}
