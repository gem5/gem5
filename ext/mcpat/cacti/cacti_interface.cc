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

#include <pthread.h>

#include <algorithm>
#include <cmath>
#include <ctime>
#include <iostream>

#include "Ucache.h"
#include "area.h"
#include "basic_circuit.h"
#include "cacti_interface.h"
#include "component.h"
#include "const.h"
#include "parameter.h"

using namespace std;


bool mem_array::lt(const mem_array * m1, const mem_array * m2) {
    if (m1->Nspd < m2->Nspd) return true;
    else if (m1->Nspd > m2->Nspd) return false;
    else if (m1->Ndwl < m2->Ndwl) return true;
    else if (m1->Ndwl > m2->Ndwl) return false;
    else if (m1->Ndbl < m2->Ndbl) return true;
    else if (m1->Ndbl > m2->Ndbl) return false;
    else if (m1->deg_bl_muxing < m2->deg_bl_muxing) return true;
    else if (m1->deg_bl_muxing > m2->deg_bl_muxing) return false;
    else if (m1->Ndsam_lev_1 < m2->Ndsam_lev_1) return true;
    else if (m1->Ndsam_lev_1 > m2->Ndsam_lev_1) return false;
    else if (m1->Ndsam_lev_2 < m2->Ndsam_lev_2) return true;
    else return false;
}



void uca_org_t::find_delay() {
    mem_array * data_arr = data_array2;
    mem_array * tag_arr  = tag_array2;

    // check whether it is a regular cache or scratch ram
    if (g_ip->pure_ram || g_ip->pure_cam || g_ip->fully_assoc) {
        access_time = data_arr->access_time;
    }
    // Both tag and data lookup happen in parallel
    // and the entire set is sent over the data array h-tree without
    // waiting for the way-select signal --TODO add the corresponding
    // power overhead Nav
    else if (g_ip->fast_access == true) {
        access_time = MAX(tag_arr->access_time, data_arr->access_time);
    }
    // Tag is accessed first. On a hit, way-select signal along with the
    // address is sent to read/write the appropriate block in the data
    // array
    else if (g_ip->is_seq_acc == true) {
        access_time = tag_arr->access_time + data_arr->access_time;
    }
    // Normal access: tag array access and data array access happen in parallel.
    // But, the data array will wait for the way-select and transfer only the
    // appropriate block over the h-tree.
    else {
        access_time = MAX(tag_arr->access_time + data_arr->delay_senseamp_mux_decoder,
                          data_arr->delay_before_subarray_output_driver) +
                      data_arr->delay_from_subarray_output_driver_to_output;
    }
}



void uca_org_t::find_energy() {
    if (!(g_ip->pure_ram || g_ip->pure_cam || g_ip->fully_assoc))
        power = data_array2->power + tag_array2->power;
    else
        power = data_array2->power;
}



void uca_org_t::find_area() {
    if (g_ip->pure_ram || g_ip->pure_cam || g_ip->fully_assoc) {
        cache_ht  = data_array2->height;
        cache_len = data_array2->width;
    } else {
        cache_ht  = MAX(tag_array2->height, data_array2->height);
        cache_len = tag_array2->width + data_array2->width;
    }
    area = cache_ht * cache_len;
}

void uca_org_t::adjust_area() {
    double area_adjust;
    if (g_ip->pure_ram || g_ip->pure_cam || g_ip->fully_assoc) {
        if (data_array2->area_efficiency / 100.0 < 0.2) {
            //area_adjust = sqrt(area/(area*(data_array2->area_efficiency/100.0)/0.2));
            area_adjust = sqrt(0.2 / (data_array2->area_efficiency / 100.0));
            cache_ht  = cache_ht / area_adjust;
            cache_len = cache_len / area_adjust;
        }
    }
    area = cache_ht * cache_len;
}

void uca_org_t::find_cyc() {
    if ((g_ip->pure_ram || g_ip->pure_cam || g_ip->fully_assoc)) {
        cycle_time = data_array2->cycle_time;
    } else {
        cycle_time = MAX(tag_array2->cycle_time,
                         data_array2->cycle_time);
    }
}

uca_org_t :: uca_org_t()
        : tag_array2(0),
        data_array2(0) {

}

void uca_org_t :: cleanup() {
    if (data_array2 != 0)
        delete data_array2;
    if (tag_array2 != 0)
        delete tag_array2;
}
